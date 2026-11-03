#!/usr/bin/env python3
"""Run the MODE 7 PRG in a 6502 interpreter with just enough C64 around it: a REU (1764 registers,
fixed-address control, length left at 1, 512 KiB wrap) over the baked image, the raster counter, CIA
timers counting at 1/64 of the CPU clock (64 MHz turbo), colour RAM. No video, no IRQs.

Every frame it shows (a `$DD00` bank flip) is checked byte for byte against the host renderer
(`target/release/chopper frame`), and the floor's 6502 cycles and I/O writes are reported. Pattern:
`../talonhd/tools/emu64.py`.

    python tools/emu.py [--prg dist/chopper.prg] [--elf dist/chopper.elf] [--reu dist/chopper.reu]
                        [--frames 4] [--doubled-after 2] [--start T] [--leap N]

`--start T` starts the flight at tick T: once the main loop is about to run, the host's agent state at
T (`chopper agent`) and the clock (T + N) are poked into `layout::AGENT` / `CLOCK`. Each frame the agent's
bytes are checked against the host's at the frame's tick too, and the soft stack's depth is reported.
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_cpu import Cpu, KernelError  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
STATUS = 0xCA80
AGENT = 0xCAA0      # layout::AGENT (agent::Agent::BYTES), then CLOCK
AGENT_BYTES = 36
SOFT_STACK, SOFT_STACK_END = 0xCD80, 0xD000
BUFFERS = [0x6000]
TURBO = 64          # CPU cycles per Phi2 tick
LINE_CYCLES = 65 * TURBO
LINES = 263


class Timer:
    def __init__(self):
        self.latch = 0xFFFF
        self.running = False
        self.start = 0        # Phi2 time of the last (force) load
        self.initial = 0xFFFF
        self.frozen = 0xFFFF
        self.ctrl = 0

    def underflows(self, t):
        if not self.running:
            return 0
        e = t - self.start
        if e <= self.initial:
            return 0
        return 1 + (e - self.initial - 1) // (self.latch + 1)

    def value(self, t):
        if not self.running:
            return self.frozen
        e = t - self.start
        if e <= self.initial:
            return self.initial - e
        return self.latch - ((e - self.initial - 1) % (self.latch + 1))


class Cia:
    def __init__(self):
        self.a, self.b = Timer(), Timer()
        self.b_counts_a = False
        self.b_base = 0

    def b_value(self, t):
        if self.b_counts_a and self.b.running:
            return (self.b.initial - (self.a.underflows(t) - self.b_base)) & 0xFFFF
        return self.b.value(t)

    def read(self, reg, t):
        if reg == 4:
            return self.a.value(t) & 0xFF
        if reg == 5:
            return self.a.value(t) >> 8
        if reg == 6:
            return self.b_value(t) & 0xFF
        if reg == 7:
            return self.b_value(t) >> 8
        if reg == 0:
            return 0xFF  # no joystick
        return 0

    def write(self, reg, value, t):
        if reg == 4:
            self.a.latch = (self.a.latch & 0xFF00) | value
        elif reg == 5:
            self.a.latch = (self.a.latch & 0x00FF) | value << 8
        elif reg == 6:
            self.b.latch = (self.b.latch & 0xFF00) | value
        elif reg == 7:
            self.b.latch = (self.b.latch & 0x00FF) | value << 8
        elif reg == 14:
            self.control(self.a, value, t)
        elif reg == 15:
            self.b_counts_a = (value >> 5) & 3 == 2
            self.control(self.b, value, t)
            self.b_base = self.a.underflows(t)

    def control(self, timer, value, t):
        if value & 1:
            if value & 0x10 or not timer.running:
                timer.start = t
                timer.initial = timer.latch if value & 0x10 else timer.frozen
            timer.running = True
        else:
            if timer.running:
                timer.frozen = timer.value(t)
            timer.running = False


class C64(Cpu):
    def __init__(self, mem, reu):
        super().__init__(mem, [])
        self.reu = reu
        self.io = bytearray(0x1000)
        self.cia = [Cia(), Cia()]
        self.regs = bytearray(16)
        self.latch = bytearray(3)   # values last written to $DF04-$DF06 (C64U: a write reloads all three)
        self.flips = []
        self.io_writes = 0
        self.dmas = 0
        self.watch = []   # (cycles, io_writes, dmas) at each CIA 1 stopwatch start/stop
        self.wait_cycles = 0
        self.last_d012 = None

    def t(self):
        return self.cycles // TURBO

    def read(self, address):
        address &= 0xFFFF
        if 0xD000 <= address < 0xE000:
            if address == 0xD012:
                # Raster polls back to back are band_publish waiting: count the time between them.
                if self.last_d012 is not None and self.cycles - self.last_d012 < 64:
                    self.wait_cycles += self.cycles - self.last_d012
                self.last_d012 = self.cycles
            if address in (0xD011, 0xD012):
                line = (self.cycles // LINE_CYCLES) % LINES
                return (self.io[0x011] & 0x7F) | (0x80 if line > 255 else 0) if address == 0xD011 else line & 0xFF
            if 0xDC00 <= address < 0xDE00:
                return self.cia[(address >> 8) & 1].read(address & 0xF, self.t())
            if 0xDF00 <= address < 0xDF10:
                return self.regs[address & 0xF]
            return self.io[address - 0xD000]
        return self.mem[address]

    def write(self, address, value):
        address &= 0xFFFF
        value &= 0xFF
        if 0xD000 <= address < 0xE000:
            self.io_writes += 1
            if 0xDC00 <= address < 0xDE00:
                cia = (address >> 8) & 1
                if cia == 0 and (address & 0xF) == 14:
                    self.watch.append((self.cycles, self.io_writes, self.dmas))
                self.cia[cia].write(address & 0xF, value, self.t())
                if address == 0xDD00 and (self.io[0xD00] & 3) != (value & 3) and self.mem[STATUS] == 0xC1:
                    self.flips.append(value & 3)
                if address == 0xDD00:
                    self.io[0xD00] = value
                return
            if 0xDF00 <= address < 0xDF10:
                self.regs[address & 0xF] = value
                if 0xDF04 <= address <= 0xDF06:
                    self.latch[address - 0xDF04] = value
                    self.regs[4:7] = self.latch
                if address == 0xDF01 and value & 0x80:
                    self.reu_exec(value)
                return
            self.io[address - 0xD000] = value
            if address == 0xD018:
                self.check_vic()
            return
        self.mem[address] = value

    def check_vic(self):
        # In VIC banks 0 and 2 the VIC sees the character ROM at $1000-$1FFF of the bank, not RAM: a screen
        # or bitmap there shows ROM (found on the C64U: a screen at $9C00 showed garbage colours).
        # Checked when $D018 is written: the flip writes $DD00 then $D018, so the pair is final then.
        bank = 3 - (self.io[0xD00] & 3)
        d018 = self.io[0x018]
        screen = (d018 >> 4) * 0x400
        if bank in (0, 2) and 0x1000 <= screen < 0x2000 and self.mem[STATUS] == 0xC1:
            raise KernelError(f'screen at ${bank * 0x4000 + screen:04X}: the VIC sees character ROM there')

    def reu_exec(self, command):
        r = self.regs
        c64 = r[2] | r[3] << 8
        addr = r[4] | r[5] << 8 | r[6] << 16
        n = (r[7] | r[8] << 8) or 0x10000
        if command & 3 not in (0, 1):
            raise KernelError(f'REU command ${command:02X} not modelled')
        stash = command & 3 == 0
        if not r[0xA] & 0x40 and (addr >> 19) != ((addr + n - 1) >> 19):
            raise KernelError(f'a DMA across a 512 KiB bank: ${addr:06X} + {n}')  # TALON HD's rule
        for _ in range(n):
            if stash:
                if 0xD000 <= c64 < 0xE000:
                    raise KernelError(f'stash from I/O ${c64:04X}')
                self.reu[addr % len(self.reu)] = self.mem[c64]
                c64 = (c64 + 1) & 0xFFFF
                addr = (addr & ~0x7FFFF) | ((addr + 1) & 0x7FFFF)
                continue
            v = self.reu[addr % len(self.reu)]
            if 0xD000 <= c64 < 0xE000:
                if 0xD800 <= c64 < 0xDC00:
                    self.io[c64 - 0xD000] = v & 15
                else:
                    raise KernelError(f'DMA into I/O ${c64:04X}')
            else:
                self.mem[c64] = v
            if not r[0xA] & 0x80:
                c64 = (c64 + 1) & 0xFFFF
            if not r[0xA] & 0x40:
                addr = (addr & ~0x7FFFF) | ((addr + 1) & 0x7FFFF)
        r[2], r[3] = c64 & 0xFF, c64 >> 8
        r[4], r[5], r[6] = addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF
        r[7], r[8] = 1, 0
        self.dmas += 1


def wsl(path):
    p = Path(path).resolve()
    return f'/mnt/{p.drive[0].lower()}' + p.as_posix()[2:]


def symbols(elf):
    out = subprocess.run(['wsl', '-d', 'Ubuntu-24.04', '--', 'bash', '-lc',
                          f'~/.local/llvm-mos/bin/llvm-nm -n {wsl(elf)}'],
                         capture_output=True, text=True, check=True).stdout
    return {p[2]: int(p[0], 16) for p in (l.split() for l in out.splitlines()) if len(p) == 3}


def host_agent(reu, tick):
    """The host's agent state after `tick` ticks (`chopper agent`, `agent::Agent::to_bytes`)."""
    exe = ROOT / 'target/release/chopper.exe'
    if not exe.exists():
        exe = ROOT / 'target/release/chopper'
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / 'agent.bin'
        subprocess.run([str(exe), 'agent', '--reu', str(reu), '--at', str(tick), '--out', str(out)],
                       check=True, capture_output=True)
        return out.read_bytes()


def host_frame(reu, tick, doubled):
    exe = ROOT / 'target/release/chopper.exe'
    if not exe.exists():
        exe = ROOT / 'target/release/chopper'
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / 'f'
        cmd = [str(exe), 'frame', '--reu', str(reu), '--tick', str(tick), '--out', str(out)]
        if doubled:
            cmd.append('--doubled')
        subprocess.run(cmd, check=True, capture_output=True)
        return out.with_suffix('.bin').read_bytes()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prg', default='dist/chopper.prg')
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--reu', default='dist/chopper.reu')
    ap.add_argument('--frames', type=int, default=4)
    ap.add_argument('--doubled-after', type=int, default=2, help='switch MODE to doubled after this many frames')
    ap.add_argument('--start', type=int, help='start the flight at this tick: poke the host agent and clock')
    ap.add_argument('--leap', type=int, default=0, help='and the clock this many ticks further on (the first '
                    'frame runs them all through the AI overlay)')
    args = ap.parse_args()
    if args.leap and args.start is None:
        args.start = 0
    start = host_agent(args.reu, args.start) if args.start is not None else None

    prg = Path(args.prg).read_bytes()
    load = prg[0] | prg[1] << 8
    mem = bytearray(0x10000)
    mem[load:load + len(prg) - 2] = prg[2:]
    reu = bytearray(Path(args.reu).read_bytes())
    cpu = C64(mem, reu)
    cpu.pc = symbols(args.elf)['_start']
    cpu.push(0xFF)
    cpu.push(0xFE)

    failures, checked, done = 0, 0, 0
    steps = 0
    low_sp = SOFT_STACK_END
    pending = None
    modes = []   # MODE as each render pass started
    # Three stopwatches a frame: setup (events 1-2), geometry (3-4), the band loop (5-6). A frame is on
    # screen when its band loop ends; the status block (tick) is written right after, so check it at the
    # next frame's first event. One buffer: the bitmap at $6000.
    while checked < args.frames:
        if start is not None and cpu.mem[STATUS] == 0xC1:
            # The main loop is about to start: the agent and its clock at the start tick.
            cpu.mem[AGENT:AGENT + len(start)] = start
            cpu.mem[AGENT + len(start):AGENT + len(start) + 4] = (args.start + args.leap).to_bytes(4, 'little')
            start = None
        n_watch = len(cpu.watch)
        cpu.step()
        # The soft stack pointer (__rc0/__rc1) at each JSR, once it is set up (not mid-update).
        if cpu.mem[cpu.pc] == 0x20 and cpu.mem[3] >= 0xC0:
            low_sp = min(low_sp, cpu.mem[2] | cpu.mem[3] << 8)
        steps += 1
        if steps > 400_000_000:
            raise SystemExit('no frames shown')
        if len(cpu.watch) == n_watch:
            continue
        k = len(cpu.watch) % 6
        if k == 5:
            modes.append(cpu.mem[STATUS + 7] & 1)
        elif k == 0 and cpu.mem[STATUS] == 0xC1:
            done += 1
            pending = (cpu.watch[-2], cpu.watch[-1], modes[-1], cpu.watch[-4], cpu.watch[-3])
            if done == args.doubled_after:
                cpu.mem[STATUS + 7] = 1
        elif k == 1 and pending:
            ((c0, io0, d0), (c1, io1, d1), doubled, (g0, _, _), (g1, _, _)) = pending
            pending = None
            checked += 1
            st = cpu.mem[STATUS:STATUS + 23]
            tick = st[2] | st[3] << 8 | st[21] << 16 | st[22] << 24
            got = bytes(cpu.mem[BUFFERS[0]:BUFFERS[0] + 8000])
            want = host_frame(args.reu, tick, doubled)
            bad = sum(a != b for a, b in zip(got, want))
            dmas = d1 - d0
            print(f'frame {checked}: tick {tick}, {"doubled" if doubled else "full"}: {bad} mismatched bytes; '
                  f'bands {(c1 - c0) / 64000:.2f} ms ({dmas} DMAs), geometry {(g1 - g0) / 64000:.2f} ms, '
                  f'{st[18]} triangles; raster wait {cpu.wait_cycles / 64000:.2f} ms')
            cpu.wait_cycles = 0
            # The agent (layout::AGENT) is still the frame's: the next one has not flown yet.
            agent = bytes(cpu.mem[AGENT:AGENT + AGENT_BYTES])
            want_agent = host_agent(args.reu, tick)
            if agent != want_agent:
                print(f'  agent differs from the host at tick {tick}: C64 {agent.hex()} host {want_agent.hex()}')
                bad += 1
            failures += bad != 0
            for i in [i for i in range(8000) if got[i] != want[i]][:8]:
                row, col, line = i // 320, (i % 320) // 8, i % 8
                print(f'  byte {i}: cell row {row} column {col} line {line} (y {row * 8 + line}): '
                      f'C64 ${got[i]:02X} host ${want[i]:02X}')
    # The soft stack (below $D000, under llvm-mos's static frames, target-c64/link.ld).
    print(f'soft stack: {SOFT_STACK_END - low_sp} of {SOFT_STACK_END - SOFT_STACK} bytes used')
    if low_sp < SOFT_STACK:
        raise SystemExit('the soft stack reached the static frames')
    if failures:
        raise SystemExit(f'{failures} frames differ from the host')
    print(f'{checked} frames match the host')


if __name__ == '__main__':
    main()
