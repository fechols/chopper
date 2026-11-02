"""Check `asm/audio.S` (audio_boot) in the 6502 interpreter: with a modelled Ultimate Audio sampler it
must decode the rotor's DLT5 to exactly the host's PCM (`chopper pcm`, `digital::decode`) in the REU,
zero the Haas prefix, and set both sampler voices up on it. Runs the first `--groups` 8-frame groups (the
whole sound takes too long here), twice: as packaged, and with the code stream moved so its fetches
cross a 512 KiB REU boundary (the split path). Without a sampler it must do nothing.

    python tools/test_audio.py [--prg dist/chopper.prg] [--elf dist/chopper.elf] [--reu dist/chopper.reu] [--groups 2600]
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
ROOT = Path(__file__).resolve().parent.parent


def u32(b, at):
    return int.from_bytes(b[at:at + 4], 'little')


def be24(io, at):
    return io[at] << 16 | io[at + 1] << 8 | io[at + 2]


def run(image, entry, reu, sampler):
    mem = bytearray(image)
    cpu = emu.C64(mem, reu)
    if sampler:
        cpu.io[0xF21] = 0x10
    cpu.pc = entry
    cpu.push((RETURN - 1) >> 8)
    cpu.push((RETURN - 1) & 0xFF)
    while cpu.pc != RETURN:
        cpu.step()
    return cpu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prg', default='dist/chopper.prg')
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--reu', default='dist/chopper.reu')
    ap.add_argument('--groups', type=int, default=2600)
    args = ap.parse_args()
    prg = Path(args.prg).read_bytes()
    image = bytearray(0x10000)
    image[0x801:0x801 + len(prg) - 2] = prg[2:]
    entry = emu.symbols(args.elf)['audio_boot']
    ref = ROOT / 'target/rotor/rotor.pcm'
    exe = ROOT / 'target/release/chopper'
    subprocess.run([str(exe), 'pcm', '--out', str(ref)], check=True, capture_output=True)
    want = ref.read_bytes()
    base = bytearray(16 << 20)
    packaged = Path(args.reu).read_bytes()
    base[:len(packaged)] = packaged
    loop_a, loop_b = u32(base, HEADER + H_LOOP_START), u32(base, HEADER + H_LOOP_END)
    n = args.groups * 8

    # No sampler: nothing happens.
    cpu = run(image, entry, bytearray(base), sampler=False)
    assert cpu.dmas == 0 and cpu.io[0xF20] == 0, 'audio_boot touched the REU or sampler without a sampler'

    stream = u32(base, HEADER + H_STREAM)
    moved = 0x67FE20  # the second fetch of 255 bytes crosses $680000
    for name, at in (('as packaged', stream), ('across a 512 KiB boundary', moved)):
        reu = bytearray(base)
        reu[HEADER + H_GROUPS:HEADER + H_GROUPS + 4] = args.groups.to_bytes(4, 'little')
        if at != stream:
            reu[at:at + n * 3 // 8 + 512] = base[stream:stream + n * 3 // 8 + 512]
            reu[HEADER + H_STREAM:HEADER + H_STREAM + 4] = at.to_bytes(4, 'little')
        reu[PCM - HAAS:PCM] = b'\x55' * HAAS  # must be zeroed
        cpu = run(image, entry, reu, sampler=True)
        got = bytes(reu[PCM:PCM + n])
        if got != want[:n]:
            bad = next(i for i in range(n) if got[i] != want[i])
            raise SystemExit(f'{name}: PCM differs from frame {bad} (got {got[bad]}, want {want[bad]})')
        assert reu[PCM - HAAS:PCM] == bytes(HAAS), f'{name}: the Haas prefix is not zero'
        io = cpu.io
        voices = {
            'left': (0xF20, PCM, loop_b + 1, loop_a, loop_b, 0),
            'right': (0xF40, PCM - HAAS, loop_b + HAAS + 1, loop_a + HAAS, loop_b + HAAS, 15),
        }
        for v, (r, start, length, ra, rb, pan) in voices.items():
            got_regs = (be24(io, r + 5), be24(io, r + 9), be24(io, r + 17), be24(io, r + 21), io[r + 2],
                        io[r + 14] << 8 | io[r + 15], io[r + 4], io[r])
            assert got_regs == (start, length, ra, rb, pan, DIVIDER, 1, 3), f'{name}: {v} voice {got_regs}'
        per_frame = cpu.cycles / n
        print(f'{name}: {n} frames match; {per_frame:.0f} cycles a frame '
              f'(the whole {len(want) / 16000:.1f} s: {per_frame * len(want) / 64e6:.1f} s at 64 MHz)')


if __name__ == '__main__':
    main()
