#!/usr/bin/env python3
"""Cycles per function of the MODE 7 PRG's geometry (between the geometry stopwatch's start and stop) over
a few frames, in the 6502 interpreter (`tools/emu.py`'s machine). Functions are the ELF's symbols.

    python tools/profile.py [--frames 2]
"""
import argparse
import bisect
import collections
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
        if len(p) == 3 and p[1] in 'tTW':
            syms.append((int(p[0], 16), p[2][:70]))
    return syms


WINDOWS = ((0xE000, 0xF800), (0xA000, 0xAF00))


def overlay_symbols(elf):
    """The overlays' functions by section, and each overlay's first 16 bytes (to tell which is loaded)."""
    out = subprocess.run(['wsl', '-d', 'Ubuntu-24.04', '--', 'bash', '-lc',
                          f'~/.local/llvm-mos/bin/llvm-objdump -t -C {emu.wsl(elf)}'],
                         capture_output=True, text=True, check=True).stdout
    by = collections.defaultdict(list)
    for line in out.splitlines():
        p = line.split(None, 5)
        if len(p) == 6 and p[2] == 'F' and p[3].endswith('text') and p[3] != '.text':
            a = int(p[0], 16)
            if any(lo <= a < hi for lo, hi in WINDOWS):
                by[p[3]].append((a, p[5][:70]))
    b = Path(elf).read_bytes()
    import struct
    shoff, = struct.unpack_from('<I', b, 0x20)
    shentsize, shnum, shstrndx = struct.unpack_from('<HHH', b, 0x2E)
    hdrs = [struct.unpack_from('<IIIIII', b, shoff + i * shentsize) for i in range(shnum)]
    names = hdrs[shstrndx][4]
    heads = {}
    for name_off, _, flags, addr, off, size in hdrs:
        n = b[names + name_off:b.index(b'\0', names + name_off)].decode()
        if n in by and size >= 16:
            heads[n] = (addr, bytes(b[off:off + 16]))
    return {n: sorted(v) for n, v in by.items()}, heads


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=2)
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--render', action='store_true', help='profile the band loop instead of the geometry')
    ap.add_argument('--setup', action='store_true', help='profile the frame set-up instead of the geometry')
    ap.add_argument('--callers', metavar='NAME', help='also list the call paths (3 deep) into functions named NAME')
    args = ap.parse_args()
    prg = Path('dist/chopper.prg').read_bytes()
    mem = bytearray(0x10000)
    mem[0x801:0x801 + len(prg) - 2] = prg[2:]
    cpu = emu.C64(mem, bytearray(Path('dist/chopper.reu').read_bytes()))
    cpu.pc = emu.symbols(args.elf)['_start']
    cpu.push(0xFF)
    cpu.push(0xFE)
    syms = text_symbols(args.elf)
    osyms, heads = overlay_symbols(args.elf)

    def name_of(a):
        """The function at `a`, overlays resolved by what is loaded in their window now."""
        for lo, hi in WINDOWS:
            if lo <= a < hi:
                for sec, (base, head) in heads.items():
                    if base == lo and bytes(cpu.mem[lo:lo + 16]) == head:
                        v = osyms[sec]
                        i = bisect.bisect_right([x for x, _ in v], a) - 1
                        return v[i][1] if i >= 0 else sec
        i = bisect.bisect_right(addrs, a) - 1
        return syms[i][1] if i >= 0 else hex(a)
    addrs = [a for a, _ in syms]
    by = collections.Counter()
    calls = collections.Counter()
    names = {a: n for a, n in syms}
    frames, total = 0, 0
    stack, paths = [], collections.Counter()
    while frames < args.frames:
        n = len(cpu.watch)
        # set-up: events 1-2; geometry: 3-4; the band loop: 5-6
        inside = n % 6 == (5 if args.render else 1 if args.setup else 3)
        pc, c0 = cpu.pc, cpu.cycles
        cpu.step()
        if inside:
            i = bisect.bisect_right(addrs, pc) - 1
            by[syms[i][1] if i >= 0 else '?'] += cpu.cycles - c0
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
    for path, n in paths.most_common(25):
        print(f'{n / args.frames:7.1f}  {path}')


if __name__ == '__main__':
    main()
