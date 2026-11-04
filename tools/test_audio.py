"""Check `asm/stream.S` (the rotor sound's streaming decoder) in the 6502 interpreter, with a modelled
Ultimate Audio sampler. The sound is cut short for the interpreter (`--pages` pages of the real stream,
the loop its last `--ring` samples), and the ring kept that long:

- `stream_init` + `stream_run(0)` until it is ahead (the boot) fill half the ring;
  `stream_start` sets both voices round it (left from the ring, right from the Haas bytes before it).
- Then `stream_run(F)` once a drawn frame (F: NTSC frames since the start, `--ticks` a frame), as the
  main loop does, until the stream ends. At every call the ring must hold every sample the playback (at
  the sampler's nominal 16,025.6 Hz, and 10% off it either way) has yet to reach, and nothing either voice has still to play may have
  been overwritten.
- At the end the ring is the loop: `ring[n mod L] == pcm[n]` for the last L samples, against the
  host's decode (`chopper pcm`, `digital::decode`), and the Haas bytes are zero.

Twice: as packaged, and with the code stream moved so its fetches cross a 512 KiB REU boundary (the
split path). Without a sampler nothing may happen.

    python tools/test_audio.py [--prg dist/chopper.prg] [--elf dist/chopper.elf] [--reu dist/chopper.reu]
"""
import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import emu  # noqa: E402

RETURN = 0xFFF0
HEADER = 86 << 16
H_LOOP_START, H_LOOP_END, H_STREAM, H_GROUPS = 20, 24, 32, 36
PCM, HAAS, DIVIDER = 0xC00000, 160, 390
OVERLAY_STREAM = (117 << 16) + 0x4800  # format::OVERLAY_STREAM
WRITTEN = 1 + 2 + 3 + 3 + 2 + 2 + 3 + 2 + 2 + 3 + 3  # stream_state's WRITTEN (asm/stream.S)
RATE = 6.25e6 / DIVIDER  # the sampler's samples a second
ROOT = Path(__file__).resolve().parent.parent


def u32(b, at):
    return int.from_bytes(b[at:at + 4], 'little')


def be24(io, at):
    return io[at] << 16 | io[at + 1] << 8 | io[at + 2]


def call(cpu, entry, a=0, x=0):
    cpu.a, cpu.x = a, x
    cpu.pc = entry
    cpu.push((RETURN - 1) >> 8)
    cpu.push((RETURN - 1) & 0xFF)
    while cpu.pc != RETURN:
        cpu.step()
    return cpu.a


def machine(prg, reu, sym, sampler):
    mem = bytearray(0x10000)
    mem[0x801:0x801 + len(prg) - 2] = prg[2:]
    size = sym['__streamtext_size']
    mem[0xE000:0xE000 + size] = reu[OVERLAY_STREAM:OVERLAY_STREAM + size]
    cpu = emu.C64(mem, reu)
    if sampler:
        cpu.io[0xF21] = 0x10
    return cpu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prg', default='dist/chopper.prg')
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--reu', default='dist/chopper.reu')
    ap.add_argument('--pages', type=int, default=120)
    ap.add_argument('--ring', type=int, default=20_007)
    ap.add_argument('--ticks', type=int, default=4)
    args = ap.parse_args()
    prg = Path(args.prg).read_bytes()
    sym = emu.symbols(args.elf)
    ref = ROOT / 'target/rotor/rotor.pcm'
    exe = ROOT / 'target/release/chopper'
    subprocess.run([str(exe), 'pcm', '--out', str(ref)], check=True, capture_output=True)
    want = ref.read_bytes()
    base = bytearray(16 << 20)
    packaged = Path(args.reu).read_bytes()
    base[:len(packaged)] = packaged
    ring = args.ring
    end = args.pages * 256
    assert ring > 512 and end > ring and ring % 256, 'a ring of pages, short of the sound, ending mid-page'
    state = sym['stream_state']

    # No sampler: nothing happens, and the stream is over before it began.
    cpu = machine(prg, bytearray(base), sym, sampler=False)
    assert call(cpu, sym['stream_init']) == 0 and call(cpu, sym['stream_run']) == 2
    assert cpu.dmas == 0 and cpu.io[0xF20] == 0, 'the decoder touched the REU or the sampler without a sampler'

    stream = u32(base, HEADER + H_STREAM)
    moved = 0x67FE20  # a fetch of 255 bytes crosses $680000
    # (The sampler's real rate is unknown to the decoder: it must hold up 10% off either way.)
    for name, at, k in (('as packaged', stream, 1.0), ('across a 512 KiB boundary', moved, 1.0),
                        ('the sampler 10% slow', stream, 0.9), ('the sampler 10% fast', stream, 1.1)):
        rate = RATE * k
        reu = bytearray(base)
        for field, v in ((H_GROUPS, end // 8), (H_LOOP_END, end), (H_LOOP_START, end - ring)):
            reu[HEADER + field:HEADER + field + 4] = v.to_bytes(4, 'little')
        if at != stream:
            reu[at:at + end * 3 // 8 + 512] = base[stream:stream + end * 3 // 8 + 512]
            reu[HEADER + H_STREAM:HEADER + H_STREAM + 4] = at.to_bytes(4, 'little')
        reu[PCM - HAAS:PCM + ring + 4096] = b'\x55' * (HAAS + ring + 4096)  # stale: must be overwritten / zeroed
        cpu = machine(prg, reu, sym, sampler=True)
        written = lambda: (cpu.mem[state + WRITTEN] | cpu.mem[state + WRITTEN + 1] << 8) * 256  # noqa: E731

        # The boot: ahead, then the sampler on.
        assert call(cpu, sym['stream_init']) == 1
        while (r := call(cpu, sym['stream_run'])) == 0:
            pass
        assert r == 1 and written() == ring // 2 // 256 * 256, f'{name}: {written()} written ahead'
        call(cpu, sym['stream_start'], DIVIDER & 0xFF, DIVIDER >> 8)
        io = cpu.io
        voices = {
            'left': (0xF20, PCM, ring + 1, 0, ring, 0),
            'right': (0xF40, PCM - HAAS, ring + HAAS + 1, HAAS, ring + HAAS, 15),
        }
        for v, (r, start, length, ra, rb, pan) in voices.items():
            got = (be24(io, r + 5), be24(io, r + 9), be24(io, r + 17), be24(io, r + 21), io[r + 2],
                   io[r + 14] << 8 | io[r + 15], io[r + 4], io[r])
            assert got == (start, length, ra, rb, pan, DIVIDER, 1, 3), f'{name}: {v} voice {got}'

        # The frames: the playback at the sampler's rate; the decoder paced by the frame count.
        cycles0, pages0 = cpu.cycles, written() // 256
        frames, calls, lead = 0, 0, ring
        while True:
            frames += args.ticks
            played = int(frames * rate / 60)
            calls += 1
            # Between frames the window is the other overlays' and the geometry's.
            cpu.mem[0xE000:0xF800] = bytes((i * 151 + calls) & 0xFF for i in range(0x1800))
            cpu.mem[0xE000:0xE000 + sym['__streamtext_size']] = reu[OVERLAY_STREAM:OVERLAY_STREAM + sym['__streamtext_size']]
            r = call(cpu, sym['stream_run'], frames & 0xFF, frames >> 8)
            w = written()
            # Every sample the playback reaches before the next call is written; none either voice has
            # still to play (the trailing one HAAS behind) is overwritten.
            if w < end:
                assert w >= int((frames + args.ticks) * rate / 60), f'{name}: frame {frames}: {w} written, the playback at {played}'
            assert w <= played - HAAS + ring, f'{name}: frame {frames}: {w} written, overwriting at {played}'
            lead = min(lead, w - played)
            if r == 2:
                break
            assert calls < 10_000, f'{name}: never ends'
        assert w == end, f'{name}: {w} written of {end}'
        got = bytes(reu[PCM:PCM + ring])
        loop = bytearray(ring)
        for n in range(end - ring, end):
            loop[n % ring] = want[n]
        if got != bytes(loop):
            bad = next(i for i in range(ring) if got[i] != loop[i])
            raise SystemExit(f'{name}: the ring differs at slot {bad} (got {got[bad]}, want {loop[bad]})')
        assert reu[PCM - HAAS:PCM] == bytes(HAAS), f'{name}: the Haas prefix is not zero'
        assert reu[PCM + ring:PCM + ring + 4096] == b'\x55' * 4096, f'{name}: written past the ring'
        per_page = (cpu.cycles - cycles0) / (w // 256 - pages0)
        print(f'{name}: the ring is the loop after {calls} frames; the decoder at least {lead} samples ahead; '
              f'{per_page:.0f} cycles a page ({per_page * rate / 256 / 64e6 * 100:.1f}% of 64 MHz while it streams)')


if __name__ == '__main__':
    main()
