#!/usr/bin/env python3
"""Check `asm/fixed.S` (m7_umul16, m7_smul16) in the 6502 interpreter: random and edge-case 16x16
products, unsigned and signed, with the quarter-square tables `fixed::init_tables` builds at boot.

    python tools/test_fixed.py [--elf dist/chopper.elf] [--cases 20000]
"""
import argparse
import random
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_cpu import Cpu  # noqa: E402
from emu import symbols  # noqa: E402

RETURN = 0xFFF0
RC2, RC3 = 0x04, 0x05


def load(elf, mem):
    """The ELF's sections at their run addresses (the relocated kernels too; not the overlays)."""
    b = Path(elf).read_bytes()
    shoff, = struct.unpack_from('<I', b, 0x20)
    shentsize, shnum, _ = struct.unpack_from('<HHH', b, 0x2E)
    for i in range(shnum):
        _, kind, flags, addr, off, size = struct.unpack_from('<IIIIII', b, shoff + i * shentsize)
        if kind == 1 and flags & 2 and size and addr + size <= 0x10000 and addr not in (0xE000, 0xA000):
            mem[addr:addr + size] = b[off:off + size]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--cases', type=int, default=20000)
    args = ap.parse_args()
    mem = bytearray(0x10000)
    load(args.elf, mem)
    for n in range(512):
        q = n * n >> 2
        mem[0x0400 + n] = q & 255
        mem[0x0600 + n] = q >> 8
        g = abs(n - 255) ** 2 >> 2
        mem[0x5000 + n] = g & 255
        mem[0x5200 + n] = g >> 8
    for zp, page in ((0x81, 0x04), (0x83, 0x06), (0x85, 0x50), (0x87, 0x52)):
        mem[zp] = page  # m7_umul_init
    syms = symbols(args.elf)
    rng = random.Random(3)
    edge = [0, 1, 2, 127, 128, 254, 255, 256, 257, 0x7FFF, 0x8000, 0xFFFF, 0xFF00, 0x00FF, 0x0100, 0xFE01, 0xFF80]
    cases = [(a, b) for a in edge for b in edge] + [(rng.randrange(65536), rng.randrange(65536)) for _ in range(args.cases)]
    cases += [(rng.randrange(-300, 300) & 0xFFFF, rng.randrange(65536)) for _ in range(args.cases // 4)]
    signed = lambda x: x - 0x10000 if x & 0x8000 else x
    for name, want in (('m7_umul16', lambda a, b: a * b), ('m7_smul16', lambda a, b: (signed(a) * signed(b)) & 0xFFFFFFFF)):
        if name in syms:
            run(mem, syms[name], name, cases, want)


def run(mem, entry, name, cases, want):
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
        if got != want(a, b):
            raise SystemExit(f'{name}: {a:#06x} * {b:#06x}: got {got:#010x}, want {want(a, b):#010x}')
        total += cpu.cycles
    print(f'{name}: {len(cases)} products exact; {total / len(cases):.0f} cycles each')


if __name__ == '__main__':
    main()
