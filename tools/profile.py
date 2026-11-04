#!/usr/bin/env python3
"""Cycles per function of the MODE 7 PRG's geometry (between the geometry stopwatch's start and stop) over
a few frames, in the 6502 interpreter (`tools/emu.py`'s machine). Functions are the ELF's symbols.

    python tools/profile.py [--frames 2]
"""
import argparse
import bisect
import collections
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import emu  # noqa: E402


def text_symbols(elf):
    out = subprocess.run(['wsl', '-d', 'Ubuntu-24.04', '--', 'bash', '-lc',
                          f'~/.local/llvm-mos/bin/llvm-nm -n -C {emu.wsl(elf)}'],
                         capture_output=True, text=True, check=True).stdout
    syms = []
    for line in out.splitlines():
        p = line.split(None, 2)
        # (Linker markers such as __heap_default_limit or __hitext_start are not functions.)
        if len(p) == 3 and p[1] in 'tTW' and not re.match(r'__\w+_(limit|start|end|size)$', p[2]):
            syms.append((int(p[0], 16), p[2][:70]))
    return syms


# Window 3 (the clipping overlay at $F010) first: it lies inside window 1's range.
WINDOWS = ((0xF010, 0xF800), (0xE000, 0xF800), (0xA000, 0xAF00))


def overlay_symbols(elf):
    """The overlays' functions by section, and each overlay's first 16 bytes (to tell which is loaded)."""
    out = subprocess.run(['wsl', '-d', 'Ubuntu-24.04', '--', 'bash', '-lc',
                          f'~/.local/llvm-mos/bin/llvm-objdump -t -C {emu.wsl(elf)}'],
                         capture_output=True, text=True, check=True).stdout
    by = collections.defaultdict(list)
    for line in out.splitlines():
        # addr, flags (7 columns), section, size, name: functions ('F') and asm labels alike
        m = re.match(r'([0-9a-f]{8}) (.{7}) (\S+)\s+[0-9a-f]{8} (.+)$', line)
        if m and m[3].endswith('text') and m[3] != '.text' and 'd' not in m[2] and 'f' not in m[2]:
            a = int(m[1], 16)
            if any(lo <= a < hi for lo, hi in WINDOWS) and not re.match(r'__\w+_(limit|start|end|size)$', m[4]):
                by[m[3]].append((a, m[4][:70]))
    b = Path(elf).read_bytes()
    import struct
    shoff, = struct.unpack_from('<I', b, 0x20)
    shentsize, shnum, shstrndx = struct.unpack_from('<HHH', b, 0x2E)
    hdrs = [struct.unpack_from('<IIIIII', b, shoff + i * shentsize) for i in range(shnum)]
    names = hdrs[shstrndx][4]
    heads, sizes = {}, {}
    for name_off, _, flags, addr, off, size in hdrs:
        n = b[names + name_off:b.index(b'\0', names + name_off)].decode()
        if n in by and size >= 16:
            heads[n] = (addr, bytes(b[off:off + 16]))
            sizes[n] = size
    return {n: sorted(v) for n, v in by.items()}, heads, sizes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=2)
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--render', action='store_true', help='profile the band loop instead of the geometry')
    ap.add_argument('--setup', action='store_true', help='profile the frame set-up instead of the geometry')
    ap.add_argument('--callers', metavar='NAME', help='also list the call paths (3 deep) into functions named NAME')
    ap.add_argument('--start', type=int, help='start the flight at this tick (as emu.py --start)')
    ap.add_argument('--skip', type=int, default=1, help='with --start: frames run before profiling')
    ap.add_argument('--mode', choices=['full', 'doubled'], help='hold MODE (else AUTO picks it)')
    ap.add_argument('--dma-us', type=float, default=2.0, help='C64U cost of one REU command, in us (the emulator charges none)')
    args = ap.parse_args()
    start = emu.host_agent('dist/chopper.reu', args.start) if args.start is not None else None
    prg = Path('dist/chopper.prg').read_bytes()
    mem = bytearray(0x10000)
    mem[0x801:0x801 + len(prg) - 2] = prg[2:]
    cpu = emu.C64(mem, bytearray(Path('dist/chopper.reu').read_bytes()))
    cpu.pc = emu.symbols(args.elf)['_start']
    cpu.push(0xFF)
    cpu.push(0xFE)
    syms = text_symbols(args.elf)
    osyms, heads, sizes = overlay_symbols(args.elf)

    oaddrs = {sec: [x for x, _ in v] for sec, v in osyms.items()}

    def name_of(a):
        """The function at `a`, overlays resolved by what is loaded in their window now."""
        for lo, hi in WINDOWS:
            if lo <= a < hi:
                for sec, (base, head) in heads.items():
                    if base == lo and a < base + sizes[sec] and bytes(cpu.mem[lo:lo + 16]) == head:
                        v = osyms[sec]
                        i = bisect.bisect_right(oaddrs[sec], a) - 1
                        return v[i][1] if i >= 0 else sec
        i = bisect.bisect_right(addrs, a) - 1
        return syms[i][1] if i >= 0 else hex(a)
    addrs = [a for a, _ in syms]
    by = collections.Counter()
    calls = collections.Counter()
    names = {a: n for a, n in syms}
    frames, total = 0, 0
    stack, paths = [], collections.Counter()
    skip = args.skip if args.start is not None else 0
    dmas = [0, 0]  # REU commands and bytes in the profiled phase
    inside = False
    exec0 = cpu.reu_exec

    def counted(command):
        if inside:
            r = cpu.regs
            dmas[0] += 1
            dmas[1] += (r[7] | r[8] << 8) or 0x10000
        exec0(command)
    cpu.reu_exec = counted
    while frames < args.frames + skip:
        if start is not None and cpu.mem[emu.STATUS] == 0xC1:
            cpu.mem[emu.AGENT:emu.AGENT + len(start)] = start
            cpu.mem[emu.AGENT + len(start):emu.AGENT + len(start) + 4] = args.start.to_bytes(4, 'little')
            start = None
        if args.mode and cpu.mem[emu.STATUS] == 0xC1:
            cpu.mem[emu.STATUS + 7] = 1 if args.mode == 'doubled' else 0
        n = len(cpu.watch)
        # set-up: events 1-2; geometry: 3-4; the band loop: 5-6
        inside = frames >= skip and n % 6 == (5 if args.render else 1 if args.setup else 3)
        pc, c0 = cpu.pc, cpu.cycles
        cpu.step()
        if inside:
            by[name_of(pc)] += cpu.cycles - c0
            total += cpu.cycles - c0
            if cpu.mem[pc] == 0x20:  # jsr
                t = cpu.mem[pc + 1] | cpu.mem[pc + 2] << 8
                calls[names.get(t, hex(t))] += 1
                if args.callers and args.callers in names.get(t, ''):
                    short = lambda a: name_of(a).split('::h')[0].replace('chopper::', '')
                    paths[' < '.join(short(a) for a in reversed(stack[-3:]))] += 1
        if cpu.mem[pc] == 0x20:
            stack.append(cpu.mem[pc + 1] | cpu.mem[pc + 2] << 8)
        elif cpu.mem[pc] == 0x60 and stack:
            stack.pop()
        if len(cpu.watch) != n and len(cpu.watch) % 6 == 0:
            frames += 1
    for name, c in by.most_common(20):
        n = calls[name] / args.frames
        per = f'{n:7.0f} calls {c / args.frames / n:6.0f} cyc' if n else ''
        print(f'{c / total * 100:5.1f}%  {c / args.frames / 64000:6.2f} ms  {name:12s} {per}')
    print(f'total {total / args.frames / 64000:.2f} ms a frame')
    d = dmas[0] / args.frames
    print(f'REU: {d:.0f} commands, {dmas[1] / args.frames:.0f} bytes a frame '
          f'(~{d * args.dma_us / 1000:.2f} ms more on the C64U at {args.dma_us} us a command)')
    for path, n in paths.most_common(25):
        print(f'{n / args.frames:7.1f}  {path}')


if __name__ == '__main__':
    main()
