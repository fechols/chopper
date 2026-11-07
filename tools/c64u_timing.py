"""The C64U's I/O timing as measured by tools/iobench.py (docs/validation.md, "C64U I/O timing"): the CPU
is a uniform 64.37 MHz, but an access to the REU's registers or a CIA's waits for a bus cycle. An access
waits for the first bus edge at or after both now and the end of the bus's last use, holds the bus for N
cycles (a DMA: 1 + its bytes / 4), and lets the CPU go on R cycles after that edge. Other I/O (VIC, SID) costs ~1 cycle more than
RAM.

The interpreters (tools/emu.py `C64`, tools/iobench.py's bench) count an instruction's cycles before its
access runs, so `now` is the instruction's end; the fitted R absorbs that.

    model = BusModel()
    cpu.cycles = model.access(cpu.cycles, address, write, value, length)
"""
import math

# CPU cycles a second (iobench group A) and the CIA's Phi2 (NTSC).
CPU_HZ = 64_370_000
PHI2_HZ = 1_022_727
TURBO = CPU_HZ / PHI2_HZ    # CPU cycles a Phi2 tick (~62.94)

# Fitted by `python tools/iobench.py fit` (FIT names the run and the errors).
PARAMS = dict(
    t_reu=64.75,    # an REU bus cycle, CPU cycles
    r_reg=44,       # the CPU resumes this long after the edge of a register write
    r_read=34,      # ... of a register read
    r_dma=55,       # ... of a DMA command (the transfer then holds the bus for its bytes)
    t_cia=TURBO,    # a CIA bus cycle (Phi2)
    r_cia=0,
    io_extra=1,     # other I/O: cycles more than RAM
    rephase=0,      # 1: REU edges count from the end of its last use; 0: a free-running clock (fits better)
)
FIT = ('2026-11-06, output/iobench-20261106-184049.json: rms 3.1-3.6% a slot on every group (fitted on B, C, '
       "G; E, F, D, C' not fitted), modelled ~3% fast throughout; floor-replica deltas within 6%")


def kind(address, write, value):
    """'dma', 'reu', 'reu_read', 'cia', 'io' or None (RAM) for an access to `address`."""
    if 0xDF00 <= address < 0xE000:
        if write:
            return 'dma' if address == 0xDF01 and value & 0x80 else 'reu'
        return 'reu_read'
    if 0xDC00 <= address < 0xDE00:
        return 'cia'
    if 0xD000 <= address < 0xE000:
        return 'io'
    return None


class BusModel:
    def __init__(self, **params):
        p = dict(PARAMS)
        p.update(params)
        self.__dict__.update(p)
        self.busy = {'reu': None, 'cia': None}
        self.stall = 0
        self.by_addr = {}   # address -> [accesses, stall cycles]

    def access(self, now, address, write=True, value=0, length=1):
        """The CPU's cycle count once an access at `now` (an instruction's end) has been let go."""
        k = kind(address, write, value)
        if k is None:
            return now
        if k == 'io':
            self.stall += self.io_extra
            return now + self.io_extra
        bus = 'cia' if k == 'cia' else 'reu'
        t = self.t_cia if bus == 'cia' else self.t_reu
        busy = self.busy[bus]
        if busy is None:
            edge = now
        elif bus == 'reu' and self.rephase:
            # Edges every t from the end of the last use (the REU's own state machine).
            edge = busy + t * max(0, math.ceil((now - busy) / t - 1e-9))
        else:
            edge = t * math.ceil(max(now, busy) / t - 1e-9)
        r = {'dma': self.r_dma, 'reu': self.r_reg, 'reu_read': self.r_read, 'cia': self.r_cia}[k]
        # A DMA moves ~4 bytes a bus cycle (iobench group H: 1-4 bytes cost the same, 255 bytes ~63 cycles).
        n = 1 + -(-length // 4) if k == 'dma' else 1
        if k == 'dma':
            r += (n - 2) * t
        self.busy[bus] = edge + n * t
        resume = max(math.ceil(edge + r - 1e-9), now)
        self.stall += resume - now
        st = self.by_addr.setdefault(address, [0, 0])
        st[0] += 1
        st[1] += resume - now
        return resume
