#!/usr/bin/env python3
"""Check `asm/fixed.S` (m7_umul16) in the 6502 interpreter: random and edge-case 16x16 products, with the
quarter-square tables `fixed::init_tables` builds at boot.

    python tools/test_fixed.py [--prg dist/chopper.prg] [--elf dist/chopper.elf] [--cases 20000]
"""
import argparse
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_cpu import Cpu  # noqa: E402
from emu import symbols  # noqa: E402

RETURN = 0xFFF0
RC2, RC3 = 0x04, 0x05


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prg', default='dist/chopper.prg')
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--cases', type=int, default=20000)
    args = ap.parse_args()
    prg = Path(args.prg).read_bytes()
    mem = bytearray(0x10000)
    mem[0x801:0x801 + len(prg) - 2] = prg[2:]
    for n in range(512):
        q = n * n >> 2
        mem[0x0400 + n] = q & 255
        mem[0x0600 + n] = q >> 8
        g = abs(n - 255) ** 2 >> 2
        mem[0xF100 + n] = g & 255
        mem[0xF300 + n] = g >> 8
    for zp, page in ((0x81, 0x04), (0x83, 0x06), (0x85, 0xF1), (0x87, 0xF3)):
        mem[zp] = page  # m7_umul_init
    entry = symbols(args.elf)['m7_umul16']
    rng = random.Random(3)
    edge = [0, 1, 2, 127, 128, 254, 255, 256, 257, 0x7FFF, 0x8000, 0xFFFF, 0xFF00, 0x00FF, 0x0100, 0xFE01]
    cases = [(a, b) for a in edge for b in edge] + [(rng.randrange(65536), rng.randrange(65536)) for _ in range(args.cases)]
    total = 0
    for a, b in cases:
        cpu = Cpu(mem, [(0x02, 0x08), (0x80, 0x9C), (0x100, 0x200)])
        cpu.a, cpu.x = a & 255, a >> 8
        mem[RC2], mem[RC3] = b & 255, b >> 8
        cpu.pc = entry
        cpu.push((RETURN - 1) >> 8)
        cpu.push((RETURN - 1) & 0xFF)
        while cpu.pc != RETURN:
            cpu.step()
        got = cpu.a | cpu.x << 8 | mem[RC2] << 16 | mem[RC3] << 24
        if got != a * b:
            raise SystemExit(f'{a} * {b}: got {got}, want {a * b}')
        total += cpu.cycles
    print(f'{len(cases)} products exact; {total / len(cases):.0f} cycles each')


if __name__ == '__main__':
    main()
