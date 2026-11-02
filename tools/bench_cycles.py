#!/usr/bin/env python3
"""Count the 6502 cycles of the milestone 2 bench (dist/bench.prg) in Talon HD's interpreter
(tools/native_cpu.py, vendored), with no hardware. The cycle counts exclude the C64U's I/O wait: an
I/O write is timed separately (1.26 us measured), so its own `sta` cycles are reported apart.

    python tools/bench_cycles.py [--elf dist/bench.prg.elf] [--prg dist/bench.prg]
"""
import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_cpu import Cpu  # noqa: E402

# slot: (iterations, what one iteration is, mode7::cycles estimate excluding I/O stores)
SLOTS = {
    0: (4096, 'empty loop', None),
    1: (4096, '1-byte DMA, command only', None),
    2: (4096, '1-byte DMA, lo + command', None),
    3: (4096, '1-byte DMA, lo + mid + command', None),
    4: (4096, 'per-byte DMA kernel byte', 97),
    5: (1024, 'gather byte, 4 texels', 4 * 70 + 17),
    6: (2048, 'gather byte, 1 texel', 70 + 17),
    7: (4096, 'I/O write alone', None),
}


class Bench(Cpu):
    def __init__(self, mem):
        super().__init__(mem, [])
        self.start = None
        self.stopped = 0
        self.io_at_start = 0
        self.io_stores = 0
        self.results = []
        self.done = False

    def read(self, address):
        address &= 0xFFFF
        if address in (0xDD04, 0xDD05):
            # CIA 2 timer A counting down from $FFFF at 1 MHz while the CPU runs at 64 MHz.
            t = 0xFFFF - ((self.cycles - self.start) // 64 if self.start is not None else self.stopped)
            return (t >> (8 * (address - 0xDD04))) & 0xFF
        if 0xD000 <= address < 0xE000:
            return 0
        return self.mem[address]

    def write(self, address, value):
        address &= 0xFFFF
        value &= 0xFF
        if 0xD000 <= address < 0xE000:
            self.io_stores += 1
            if address == 0xDD0E:
                if value == 0x11:
                    self.start = self.cycles
                    self.io_at_start = self.io_stores
                elif value == 0 and self.start is not None:
                    # The timer-start store pair is inside the window; leave it, it is constant per slot.
                    self.results.append((self.cycles - self.start, self.io_stores - self.io_at_start - 1))
                    self.stopped = (self.cycles - self.start) // 64
                    self.start = None
            return
        if address == 0xC03F and value == 0xC1:
            self.done = True
        self.mem[address] = value


def wsl(path):
    p = Path(path).resolve()
    return f'/mnt/{p.drive[0].lower()}' + p.as_posix()[2:]


def symbols(elf):
    out = subprocess.run(['wsl', '-d', 'Ubuntu-24.04', '--', 'bash', '-lc',
                          f'~/.local/llvm-mos/bin/llvm-nm -n {wsl(elf)}'],
                         capture_output=True, text=True, check=True).stdout
    return {p[2]: int(p[0], 16) for p in (l.split() for l in out.splitlines()) if len(p) == 3}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prg', default='dist/bench.prg')
    ap.add_argument('--elf', default='dist/bench.prg.elf')
    args = ap.parse_args()
    prg = Path(args.prg).read_bytes()
    load = prg[0] | prg[1] << 8
    mem = bytearray(0x10000)
    mem[0xE000:0x10000] = b"`" * 0x2000  # KERNAL calls return at once
    mem[load:load + len(prg) - 2] = prg[2:]
    cpu = Bench(mem)
    cpu.pc = symbols(str(Path(args.elf).resolve()))['_start']
    cpu.push(0xFF)
    cpu.push(0xFE)
    for _ in range(200_000_000):
        if cpu.done:
            break
        cpu.step()
    else:
        raise SystemExit('bench did not finish')
    lo = mem[0xC040] | mem[0xC041] << 8
    mid = mem[0xC042] | mem[0xC043] << 8
    print(f'slot 4 register writes (C replica): lo {lo}, mid {mid} over 4096 bytes '
          f'= {(lo + mid + 4096) / 4096:.2f} I/O writes a byte incl. the command')
    print(f'{"slot":>4} {"per iteration":<32} {"cycles":>8} {"I/O st":>7} {"non-I/O":>8} {"estimate":>8}')
    probes = mem[0xC046] | mem[0xC047] << 8
    print(f'turbo probes: {probes}, done marker ${mem[0xC03F]:02X}')
    for slot, (cyc, io) in enumerate(cpu.results[probes:]):
        n, what, est = SLOTS[slot]
        per, io_per = cyc / n, io / n
        rest = per - 4 * io_per  # an absolute store is 4 cycles
        print(f'{slot:>4} {what:<32} {per:>8.1f} {io_per:>7.2f} {rest:>8.1f} {est if est else "":>8}')


if __name__ == '__main__':
    main()
