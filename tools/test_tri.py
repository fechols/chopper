#!/usr/bin/env python3
"""Check `asm/tri.S` (tri_row once per cell row, as geo_draw calls it, with poly_row) in the 6502 interpreter against a port of `geom::walk`
(with `poly::paint_row` per cell row): random set-up triangles, including far off-screen edges.

    python tools/test_tri.py [--prg dist/chopper.prg] [--elf dist/chopper.elf] [--cases 2000]
"""
import argparse
import random
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_cpu import Cpu  # noqa: E402
from emu import symbols  # noqa: E402
from test_poly import paint  # noqa: E402

TRI_ARGS, POLY_ARGS = 0xC7A0, 0xC780
BITMAP = 0x6000
RETURN = 0xFFF0


def wrap(v):
    return (v + 2**31) % 2**32 - 2**31


def pixel(x):
    return min(160, max(0, (x + 0x7FFF) >> 16))


def walk(mem, w):
    (lx, ls), (sx, ss), lower, first, mid, end, pat = w
    for r in range(first >> 3, ((end - 1) >> 3) + 1):
        xl, xr = [0] * 8, [0] * 8
        for k in range(8):
            line = r * 8 + k
            if line < first or line >= end:
                continue
            if line == mid:
                sx, ss = lower
            a, b = pixel(lx), pixel(sx)
            xl[k], xr[k] = min(a, b), max(a, b)
            lx, sx = wrap(lx + ls), wrap(sx + ss)
        paint(mem, xl, xr, pat, BITMAP + r * 320)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prg', default='dist/chopper.prg')
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--cases', type=int, default=2000)
    args = ap.parse_args()
    prg = Path(args.prg).read_bytes()
    image = bytearray(0x10000)
    image[0x801:0x801 + len(prg) - 2] = prg[2:]
    entry = symbols(args.elf)['tri_row']
    rng = random.Random(11)
    total = 0
    for case in range(args.cases):
        first = rng.randrange(200)
        end = rng.randrange(first + 1, 201)
        mid = rng.randrange(first, end + 1)
        span = rng.choice([200, 2000, 40000])

        def edge():
            return (wrap(rng.randrange(-span, span) << 16 | rng.randrange(65536)), rng.randrange(-4 << 16, 4 << 16) * rng.choice([1, 1, 20]))
        lng, sht, low = edge(), edge(), edge()
        pat = [rng.randrange(256), rng.randrange(256)]
        mem = bytearray(image)
        for i in range(8000):
            mem[BITMAP + i] = rng.randrange(256)
        want = bytearray(mem)
        walk(want, (lng, sht, low, first, mid, end, pat))
        a = b''.join(struct.pack('<ii', *e) for e in (lng, sht, low)) + bytes([first, mid, end]) + struct.pack('<H', BITMAP)
        mem[TRI_ARGS:TRI_ARGS + len(a)] = a
        mem[POLY_ARGS + 16:POLY_ARGS + 18] = bytes(pat)
        cpu = Cpu(mem, [(0xA0, 0xB8), (0x100, 0x200), (BITMAP, BITMAP + 8000), (POLY_ARGS, POLY_ARGS + 20), (TRI_ARGS, TRI_ARGS + 29)])
        for row in range(first >> 3, ((end - 1) >> 3) + 1):
            cpu.a = row
            cpu.pc = entry
            cpu.push((RETURN - 1) >> 8)
            cpu.push((RETURN - 1) & 0xFF)
            while cpu.pc != RETURN:
                cpu.step()
        if mem[BITMAP:BITMAP + 8000] != want[BITMAP:BITMAP + 8000]:
            bad = [i for i in range(8000) if mem[BITMAP + i] != want[BITMAP + i]]
            raise SystemExit(f'case {case}: first {first} mid {mid} end {end} edges {lng} {sht} {low}: '
                             f'{len(bad)} bytes differ, first at {bad[0]}')
        total += cpu.cycles
    print(f'{args.cases} triangles match; {total / args.cases:.0f} cycles each')


if __name__ == '__main__':
    main()
