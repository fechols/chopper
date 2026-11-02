#!/usr/bin/env python3
"""Check `asm/poly.S` (poly_row) in the 6502 interpreter against the per-pixel specification
(`poly::paint_row`): random cell rows, random bitmap contents, every pixel phase. Pattern:
`../talonhd/tools/test_blit.py`.

    python tools/test_poly.py [--prg dist/chopper.prg] [--elf dist/chopper.elf] [--cases 3000]
"""
import argparse
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_cpu import Cpu  # noqa: E402
from emu import symbols  # noqa: E402

ARGS = 0xC780
BASE = 0x6000 + 7 * 320
RETURN = 0xFFF0


def paint(mem, xl, xr, pat, base):
    for k in range(8):
        p = pat[k & 1]
        for x in range(xl[k], xr[k]):
            a = base + (x >> 2) * 8 + k
            m = 0xC0 >> (2 * (x & 3))
            d = mem[a]
            mem[a] = d ^ ((d ^ p) & m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prg', default='dist/chopper.prg')
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--cases', type=int, default=3000)
    args = ap.parse_args()
    prg = Path(args.prg).read_bytes()
    image = bytearray(0x10000)
    image[0x801:0x801 + len(prg) - 2] = prg[2:]
    entry = symbols(args.elf)['poly_row']
    rng = random.Random(7)
    cycles = []
    for case in range(args.cases):
        l0, r0 = rng.randrange(161), rng.randrange(161)
        xl, xr = [], []
        for _ in range(8):
            c = rng.randrange(6)
            if c == 0:
                a, b = rng.randrange(161), rng.randrange(161)
            elif c == 1 and case % 3 == 0:
                a, b = 0, 0
            else:
                a, b = max(0, l0 - rng.randrange(6)), min(160, r0 + rng.randrange(6))
            xl.append(a)
            xr.append(b)
        pat = [rng.randrange(256), rng.randrange(256)]
        mem = bytearray(image)
        for i in range(320):
            mem[BASE + i] = rng.randrange(256)
        want = bytearray(mem)
        paint(want, xl, xr, pat, BASE)
        mem[ARGS:ARGS + 20] = bytes(xl + xr + pat + [BASE & 255, BASE >> 8])
        # Only its zero page, the stack and this cell row may be written.
        cpu = Cpu(mem, [(0xA0, 0xB0), (0x100, 0x200), (BASE, BASE + 320)])
        cpu.pc = entry
        cpu.push((RETURN - 1) >> 8)
        cpu.push((RETURN - 1) & 0xFF)
        while cpu.pc != RETURN:
            cpu.step()
        got = mem[BASE:BASE + 320]
        if got != want[BASE:BASE + 320]:
            bad = [i for i in range(320) if got[i] != want[BASE + i]]
            raise SystemExit(f'case {case}: xl {xl} xr {xr} pat {pat}: {len(bad)} bytes differ, first at +{bad[0]}')
        cycles.append(cpu.cycles)
    print(f'{args.cases} rows match; {sum(cycles) / len(cycles):.0f} cycles a row on average')


if __name__ == '__main__':
    main()
