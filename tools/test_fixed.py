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
    if 'm7_div_clamp' in syms:
        div_clamp(mem, syms['m7_div_clamp'], rng, args.cases)
    if '__wrap___divsi3' in syms:
        divsi3(mem, syms['__wrap___divsi3'], rng, args.cases)
    if '__wrap___mulsi3' in syms:
        mulsi3(mem, syms['__wrap___mulsi3'], rng, args.cases)
    if '__wrap___udivsi3' in syms:
        udivsi3(mem, syms['__wrap___udivsi3'], rng, args.cases)
    if '__wrap___udivhi3' in syms:
        udivhi3(mem, syms['__wrap___udivhi3'], rng)


def udivhi3(mem, entry, rng):
    """__wrap___udivhi3 (asm/fixed.S) against n // d for u16: every divisor with edge dividends, and
    random pairs."""
    cases = [(n, d) for d in range(1, 65536) for n in (0, d - 1, d, 65535)]
    cases += [(rng.randrange(65536), rng.randrange(1, 65536)) for _ in range(100000)]
    total = 0
    for n, d in cases:
        cpu = Cpu(mem, [(0x02, 0x40), (0x80, 0x9C), (0x100, 0x200)])
        cpu.a, cpu.x = n & 255, n >> 8
        mem[RC2], mem[RC3] = d & 255, d >> 8
        cpu.pc = entry
        cpu.push((RETURN - 1) >> 8)
        cpu.push((RETURN - 1) & 0xFF)
        while cpu.pc != RETURN:
            cpu.step()
        got = cpu.a | cpu.x << 8
        if got != n // d:
            raise SystemExit(f'__udivhi3: {n} / {d}: got {got}, want {n // d}')
        total += cpu.cycles
    print(f'__udivhi3: {len(cases)} quotients exact; {total / len(cases):.0f} cycles each')


def udivsi3(mem, entry, rng, n_cases):
    """__wrap___udivsi3 (asm/fixed.S) against n // d, unsigned: the byte-divisor loop and the wide one
    (divisors to 2^32 - 1: the remainder's 33rd bit)."""
    ds = (1, 2, 5, 133, 255, 256, 1024, 2047, 65535, 65536, 0x7FFFFFFF, 0x80000000, 0x80000001, 0xFFFFFFFE, 0xFFFFFFFF)
    ns = (0, 1, 0x3FFFFFF, 0xFFFFFFFF, 0x80000000, 0x7FFFFFFF, 12345678)
    cases = [(n, d) for d in ds for n in ns + (d - 1, d, d + 1, 2 * d - 1) if n < 2 ** 32]
    cases += [(0x3FFFFFF, m) for m in range(1024, 2048)]  # fixed::init_tables' reciprocals
    cases += [(rng.randrange(2 ** 32), rng.randrange(1, 256)) for _ in range(n_cases)]
    cases += [(rng.randrange(2 ** 32), rng.randrange(1, 2 ** 32) >> rng.randrange(32) or 7) for _ in range(n_cases // 2)]
    total = 0
    for n, d in cases:
        got, cycles = call32(mem, entry, n, d)
        if got != n // d:
            raise SystemExit(f'__udivsi3: {n} / {d}: got {got}, want {n // d}')
        total += cycles
    print(f'__udivsi3: {len(cases)} quotients exact; {total / len(cases):.0f} cycles each')


def call32(mem, entry, a, b):
    """Run a libcall (a, b: i32) -> i32 to its return; the result and the cycles it took."""
    u, v = a & 0xFFFFFFFF, b & 0xFFFFFFFF
    cpu = Cpu(mem, [(0x02, 0x40), (0x80, 0x9C), (0x100, 0x200), (0xCB00, 0xD000)])
    mem[0x02], mem[0x03] = 0x00, 0xD0  # the soft stack (libcrt's routines may use it)
    cpu.a, cpu.x = u & 255, u >> 8 & 255
    mem[0x04:0x0A] = bytes((u >> 16 & 255, u >> 24, v & 255, v >> 8 & 255, v >> 16 & 255, v >> 24))
    cpu.pc = entry
    cpu.push((RETURN - 1) >> 8)
    cpu.push((RETURN - 1) & 0xFF)
    while cpu.pc != RETURN:
        cpu.step()
    return cpu.a | cpu.x << 8 | mem[RC2] << 16 | mem[RC3] << 24, cpu.cycles


def mulsi3(mem, entry, rng, n_cases):
    """__wrap___mulsi3 (asm/fixed.S) against the low 32 bits of a * b (one to three 16 x 16 products)."""
    edge = (0, 1, -1, 2, 255, 256, 0xFFFF, -0xFFFF, 0x10000, -0x10000, -0x10001, 0x7FFFFFFF, -0x80000000, 12345, -32768)
    cases = [(a, b) for a in edge for b in edge]
    cases += [(rng.randrange(-65535, 65536), rng.randrange(-65535, 65536)) for _ in range(n_cases)]
    cases += [(rng.randrange(-2 ** 31, 2 ** 31), rng.randrange(-2 ** 31, 2 ** 31) >> rng.randrange(32)) for _ in range(n_cases // 4)]
    fast, total = 0, 0
    for a, b in cases:
        got, cycles = call32(mem, entry, a, b)
        if got != (a * b) & 0xFFFFFFFF:
            raise SystemExit(f'__mulsi3: {a} * {b}: got {got:#010x}, want {(a * b) & 0xFFFFFFFF:#010x}')
        if 0 <= a < 65536 and 0 <= b < 65536:
            fast, total = fast + 1, total + cycles
    print(f'__mulsi3: {len(cases)} products exact; {total / max(fast, 1):.0f} cycles each for unsigned 16-bit operands')


def divsi3(mem, entry, rng, n_cases):
    """__wrap___divsi3 (asm/fixed.S, the i32 divide libcall) against n / d truncated toward zero: the byte-divisor loop and
    the builtin __udivsi3 path."""
    def want(n, d):
        q = abs(n) // abs(d)
        return -q if (n < 0) != (d < 0) else q
    ds = (1, 2, 3, 5, 7, 128, 133, 254, 255, 256, 257, 320, 65535, 65536, 2 ** 31 - 1, -1, -5, -133, -255, -256, -2 ** 31)
    ns = (0, 1, -1, 4, -4, 255, 256, 2 ** 31 - 1, -2 ** 31, -2 ** 31 + 1, 16384 * 17280, -16384 * 17280)
    cases = [(n, d) for d in ds for n in ns + (d - 1, d, d + 1, -d) if not (n == -2 ** 31 and d == -1)]
    cases += [(rng.randrange(-2 ** 31, 2 ** 31), rng.randrange(1, 256) * rng.choice((1, -1))) for _ in range(n_cases)]
    cases += [(rng.randrange(-2 ** 31, 2 ** 31), rng.randrange(-2 ** 31, 2 ** 31) >> rng.randrange(32) or 3) for _ in range(n_cases // 4)]
    cases += [(rng.randrange(-16384, 16385) * rng.choice((64, -1248, 256, -256, 17280)), rng.choice((5, 133))) for _ in range(n_cases)]
    total = 0
    for n, d in cases:
        n = max(-2 ** 31, min(2 ** 31 - 1, n))
        u, v = n & 0xFFFFFFFF, d & 0xFFFFFFFF
        cpu = Cpu(mem, [(0x02, 0x40), (0x80, 0x9C), (0x100, 0x200), (0xCB00, 0xD000)])
        mem[0x02], mem[0x03] = 0x00, 0xD0  # the soft stack (the builtin may use it)
        cpu.a, cpu.x = u & 255, u >> 8 & 255
        mem[0x04:0x0A] = bytes((u >> 16 & 255, u >> 24, v & 255, v >> 8 & 255, v >> 16 & 255, v >> 24))
        cpu.pc = entry
        cpu.push((RETURN - 1) >> 8)
        cpu.push((RETURN - 1) & 0xFF)
        while cpu.pc != RETURN:
            cpu.step()
        got = cpu.a | cpu.x << 8 | mem[RC2] << 16 | mem[RC3] << 24
        got -= 1 << 32 if got & 0x80000000 else 0
        if got != want(n, d):
            raise SystemExit(f'__divsi3: {n} / {d}: got {got}, want {want(n, d)}')
        total += cpu.cycles
    print(f'__divsi3: {len(cases)} quotients exact; {total / len(cases):.0f} cycles each')


def div_clamp(mem, entry, rng, n_cases):
    """m7_div_clamp(n: i32, z: u16) -> i16 against (n / z).clamp(-1000, 1000), truncated, for 0 < z < 32768."""
    def want(n, z):
        q = abs(n) // z
        return max(-1000, min(1000, -q if n < 0 else q))
    zs = [1, 2, 7, 8, 9, 100, 133, 1000, 4096, 32767]
    cases = [(s * (base * z + d), z) for z in zs for base in (0, 1, 999, 1000, 1001, 1023, 1024, 1025, 40000)
             for d in (-1, 0, 1) for s in (1, -1)]
    cases += [(-2 ** 31, 1), (2 ** 31 - 1, 1), (-2 ** 31, 32767), (2 ** 31 - 1, 32767)]
    for _ in range(n_cases):
        z = rng.randrange(1, 32768)
        k = rng.randrange(3)
        n = rng.randrange(-2 ** 31, 2 ** 31) if k == 0 else rng.randrange(-1024 * z, 1024 * z) if k == 1 else \
            rng.randrange(-32768, 32768) * rng.choice((160, 133))
        cases.append((n, z))
    total = 0
    for n, z in cases:
        n = max(-2 ** 31, min(2 ** 31 - 1, n))
        u = n & 0xFFFFFFFF
        cpu = Cpu(mem, [(0x02, 0x14), (0x80, 0x9C), (0x100, 0x200)])
        cpu.a, cpu.x = u & 255, u >> 8 & 255
        mem[RC2], mem[RC3], mem[0x06], mem[0x07] = u >> 16 & 255, u >> 24, z & 255, z >> 8
        cpu.pc = entry
        cpu.push((RETURN - 1) >> 8)
        cpu.push((RETURN - 1) & 0xFF)
        while cpu.pc != RETURN:
            cpu.step()
        got = cpu.a | cpu.x << 8
        got -= 0x10000 if got & 0x8000 else 0
        if got != want(n, z):
            raise SystemExit(f'm7_div_clamp: {n} / {z}: got {got}, want {want(n, z)}')
        total += cpu.cycles
    print(f'm7_div_clamp: {len(cases)} quotients exact; {total / len(cases):.0f} cycles each')


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
