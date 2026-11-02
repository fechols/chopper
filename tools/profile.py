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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=2)
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--render', action='store_true', help='profile the band loop instead of the geometry')
    args = ap.parse_args()
    prg = Path('dist/chopper.prg').read_bytes()
    mem = bytearray(0x10000)
    mem[0x801:0x801 + len(prg) - 2] = prg[2:]
    cpu = emu.C64(mem, bytearray(Path('dist/chopper.reu').read_bytes()))
    cpu.pc = emu.symbols(args.elf)['_start']
    cpu.push(0xFF)
    cpu.push(0xFE)
    syms = text_symbols(args.elf)
    addrs = [a for a, _ in syms]
    by = collections.Counter()
    frames, total = 0, 0
    while frames < args.frames:
        n = len(cpu.watch)
        inside = n % 6 == (5 if args.render else 3)  # geometry: events 3-4; the band loop: 5-6
        pc, c0 = cpu.pc, cpu.cycles
        cpu.step()
        if inside:
            i = bisect.bisect_right(addrs, pc) - 1
            by[syms[i][1] if i >= 0 else '?'] += cpu.cycles - c0
            total += cpu.cycles - c0
        if len(cpu.watch) != n and len(cpu.watch) % 6 == 0:
            frames += 1
    for name, c in by.most_common(20):
        print(f'{c / total * 100:5.1f}%  {c / args.frames / 64000:6.2f} ms  {name}')
    print(f'total {total / args.frames / 64000:.2f} ms a frame')


if __name__ == '__main__':
    main()
