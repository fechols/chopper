#!/usr/bin/env python3
"""C64U I/O timing benchmark: what an I/O write, a DMA command and the CPU work between them cost on the
C64 Ultimate at 64 MHz. The floor kernel's hardware times (2026-11-06) did not follow "an I/O write is
~1.07 us, a cycle 16 ns, and they add": moving the same writes closer together cost ~1 us a byte. This
measures the rules instead of guessing them.

The test list (`tests()`) is the one source of the asm (`target-c64/bench/iobench.S`, generated), the slot
names and the analysis. Each slot times N iterations of its body with CIA 2 timer A (Phi2 ticks, IRQs
off), u16 ticks at RESULTS + 2 * slot.

    python tools/iobench.py gen       write target-c64/bench/iobench.S
    python tools/iobench.py build     gen, then build dist/iobench.prg (WSL, llvm-mos) and audit branches
    python tools/iobench.py cycles    each slot's CPU cycles in the interpreter (no I/O wait): the x-axis
    python tools/iobench.py run       run it on the C64U (C64U_HOST), save output/iobench-<time>.json
    python tools/iobench.py report [JSON]   join the run with the cycles: tables, sweeps, model fits
    python tools/iobench.py fit [JSON]      fit tools/c64u_timing.py's bus model to a run, report its error

The C64U is shared: check that no other session holds it before `run` (it replaces whatever runs there).
"""
import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))
import c64u_timing  # noqa: E402

ASM = ROOT / 'target-c64/bench/iobench.S'
PRG = ROOT / 'dist/iobench.prg'
ELF = ROOT / 'dist/iobench.prg.elf'
OUT = ROOT / 'output'
RESULTS = 0xC400      # u16 ticks a slot (up to 256 slots)
DONE = 0xC03F         # $C1 done, $EE no turbo (bench_cycles.Bench stops on $C1 here)
PROBES = 0xC046       # turbo probes taken (u16), as main.c
TARGET = 0xC200       # the fixed C64 address of the DMAs
CONTROL = 0xC300      # a byte no DMA writes
PHI2 = 1_022_727      # NTSC Phi2 (CIA ticks a second)

# Zero page while the bench runs ($80-$FF are saved to $C700 and restored around it).
ZP = dict(CNT=0x90, RUNS=0x92, REM=0x93, LEN=0x94, XB=0x95, UL=0x96, UH=0x97, UHH=0x98, VL=0x99,
          VH=0x9A, VHH=0x9B, DUL=0x9C, DUH=0x9D, DUS=0x9E, DVL=0x9F, DVH=0xA0, DVS=0xA1, SHMID=0xA2,
          SHLO=0xA3, ZTMP=0xA4)

B_PADS = list(range(0, 65, 2)) + list(range(72, 193, 8))
BP_PADS = [0, 16, 32, 48, 64, 96, 128]
D_PADS = [0, 2, 4, 8, 16, 32, 64]
E_PADS = [0, 32, 64, 96, 128]
G_PADS = [0, 16, 64]
H_LENGTHS = [1, 2, 4, 8, 16, 32, 64, 128, 255]

# Synthetic runs for the floor replicas: 39 bytes, no tile crossed. (u, v as 8.8 starts; du, dv 8.8.)
RUNS = {
    'v-major': dict(u=0x4080, v=0x1000, du=0x0033, dv=0x014D),   # mid changes every byte (like tick 12400)
    'u-forward': dict(u=0x4080, v=0x1000, du=0x00E6, dv=0x000D),  # mostly last + 1 (like tick 8000)
}
RUN_LEN, RUN_COUNT = 39, 105


def pad(n):
    """n cycles of padding (n = 0 or n >= 2): nops, one `bit zp` (3) for an odd count."""
    assert n == 0 or n >= 2, n
    out = []
    if n % 2:
        out.append('    bit $ea')
        n -= 3
    out += ['    nop'] * (n // 2)
    return out


def reu_setup(c64=TARGET, mode=0x80, a=0x91, x=0, y=0):
    """REU registers for 1-byte fetches: C64 address, REU 0, length 1, $DF0A mode; then A, X, Y."""
    return [f'    lda #${c64 & 0xFF:02x}', '    sta $df02', f'    lda #${c64 >> 8:02x}', '    sta $df03',
            '    lda #0', '    sta $df04', '    sta $df05', '    sta $df06', '    sta $df08', '    sta $df09',
            '    lda #1', '    sta $df07', f'    lda #${mode:02x}', '    sta $df0a',
            f'    ldx #${x:02x}', f'    ldy #${y:02x}', f'    lda #${a:02x}']


class Test:
    def __init__(self, group, name, body, n=4096, setup=(), align=None, param=None, ios=0, extra=()):
        self.group, self.name, self.body, self.n = group, name, list(body), n
        self.setup, self.align, self.param, self.ios, self.extra = list(setup), align, param, ios, list(extra)


def floor_replica(variant, s):
    """One of the three floor plain-loop layouts (asm/floor.S), labels prefixed `s`, over RUN_COUNT runs
    reset from the template in zero page; `lookup` is a stub (no tile is crossed). Returns the timed
    code (it counts its own runs: the slot loop runs it once)."""
    L = lambda n: f'{s}_{n}'
    reset = []
    for name, off in (('UL', 0xC0), ('UH', 0xC1), ('UHH', 0xC2), ('VL', 0xC3), ('VH', 0xC4), ('VHH', 0xC5)):
        reset += [f'    lda ${off:02x}', f'    sta {name}']
    reset += [f'    lda #{RUN_LEN}', '    sta REM', '    sta LEN', '    lda #$ff', '    sta SHLO']
    u_step = ['    clc', '    lda UL', '    adc DUL', '    sta UL', '    lda UH', '    adc DUH', '    sta UH',
              '    lda UHH', '    adc DUS', '    cmp UHH', f'    bne {L("ptile_u")}']
    fetch = [f'{L("pbyte")}:', '    lda #$91', '    sta $df01', '    dec REM', f'    beq {L("pdone")}']
    if variant == 'base':
        start = ['    lda XB', '    sta $df02', '    lda VH', '    sta $df05', '    lda UH', '    sta $df04']
        loop = fetch + u_step + [
            f'{L("pv")}:', '    clc', '    lda VL', '    adc DVL', '    sta VL', '    lda VH', '    adc DVH',
            '    sta VH', '    sta $df05', '    lda VHH', '    adc DVS', '    cmp VHH', f'    bne {L("ptile_v")}',
            f'{L("plo")}:', '    lda UH', '    sta $df04', f'    jmp {L("pbyte")}',
            f'{L("ptile_u")}:', '    sta UHH', '    jsr io_lookup', f'    jmp {L("pv")}',
            f'{L("ptile_v")}:', '    sta VHH', '    jsr io_lookup', f'    jmp {L("plo")}',
            f'{L("pdone")}:', '    lda VH', '    sta SHMID']
    elif variant == 'clustered':
        start = ['    lda XB', '    sta $df02', f'    jmp {L("paddr")}']
        loop = fetch + u_step + [
            f'{L("pv")}:', '    clc', '    lda VL', '    adc DVL', '    sta VL', '    lda VH', '    adc DVH',
            '    sta VH', '    lda VHH', '    adc DVS', '    cmp VHH', f'    bne {L("ptile_v")}',
            f'{L("paddr")}:', '    lda VH', '    cmp SHMID', f'    beq {L("a1")}', '    sta $df05',
            '    sta SHMID', f'    jmp {L("a2")}', f'{L("a1")}:', '    lda UH', f'    beq {L("a2")}', '    clc',
            '    sbc SHLO', f'    beq {L("a3")}', f'{L("a2")}:', '    lda UH', '    sta $df04', f'{L("a3")}:',
            '    lda UH', '    sta SHLO', f'    jmp {L("pbyte")}',
            f'{L("ptile_u")}:', '    sta UHH', '    jsr io_lookup', f'    jmp {L("pv")}',
            f'{L("ptile_v")}:', '    sta VHH', '    jsr io_lookup', f'    jmp {L("paddr")}',
            f'{L("pdone")}:']
    elif variant == 'e2':
        start = ['    lda XB', '    sta $df02', '    lda VH', '    sta $df05', '    sta SHMID', '    lda UH',
                 '    sta $df04', '    sta SHLO']
        loop = fetch + u_step + [
            f'{L("pv")}:', '    clc', '    lda VL', '    adc DVL', '    sta VL', '    lda VH', '    adc DVH',
            '    sta VH', '    lda VHH', '    adc DVS', '    tay', '    ldx #0', '    lda VH', '    cmp SHMID',
            f'    beq {L("e1")}', '    sta $df05', '    sta SHMID', '    inx', f'{L("e1")}:', '    cpy VHH',
            f'    bne {L("ptile_v")}',
            f'{L("plo")}:', '    txa', f'    bne {L("e2")}', '    lda UH', f'    beq {L("e2")}', '    clc',
            '    sbc SHLO', f'    beq {L("e3")}', f'{L("e2")}:', '    lda UH', '    sta $df04', f'{L("e3")}:',
            '    lda UH', '    sta SHLO', f'    jmp {L("pbyte")}',
            f'{L("ptile_u")}:', '    sta UHH', '    jsr io_lookup', f'    jmp {L("pv")}',
            f'{L("ptile_v")}:', '    sty VHH', '    jsr io_lookup', f'    jmp {L("plo")}',
            f'{L("pdone")}:']
    else:
        raise ValueError(variant)
    return ([f'    lda #{RUN_COUNT}', '    sta RUNS', f'{L("run")}:'] + reset + start + loop +
            ['    dec RUNS', f'    beq {L("rdone")}', f'    jmp {L("run")}', f'{L("rdone")}:'])


def floor_setup(run):
    """The run template at $C0-$C5 (u lo/hi/tile, v lo/hi/tile), the steps, and the REU state: incrementing
    addresses, C64 $C800 + XB (XB = 0: each run refills $C800-$C826), mid shadowed as written."""
    u, v, du, dv = run['u'], run['v'], run['du'], run['dv']
    vals = [('$c0', u & 0xFF), ('$c1', u >> 8), ('$c2', 0), ('$c3', v & 0xFF), ('$c4', v >> 8), ('$c5', 0),
            ('DUL', du & 0xFF), ('DUH', du >> 8), ('DUS', 0), ('DVL', dv & 0xFF), ('DVH', dv >> 8), ('DVS', 0),
            ('XB', 0), ('SHMID', 0)]
    out = reu_setup(c64=0xC800, mode=0)
    for name, val in vals:
        out += [f'    lda #${val:02x}', f'    sta {name}']
    return out


def tests():
    t = []
    # A: is the CPU a uniform 64 MHz? (No I/O.)
    t.append(Test('A', 'empty loop', []))
    t.append(Test('A', 'nop x32', ['    nop'] * 32))
    t.append(Test('A', 'lda zp x16', ['    lda $a4'] * 16))
    t.append(Test('A', 'lda abs x16', [f'    lda ${CONTROL:04x}'] * 16))
    t.append(Test('A', 'lda abs,x page-crossing x16', [f'    lda ${CONTROL + 1:04x},x'] * 16,
                  setup=['    ldx #$ff']))
    t.append(Test('A', 'sta abs x16', [f'    sta ${CONTROL:04x}'] * 16))
    t.append(Test('A', 'taken branch x16', ['    clc'] + [x for i in range(16) for x in (f'    bcc {{s}}_b{i}',
                                                                                   f'{{s}}_b{i}:')]))
    t.append(Test('A', 'jmp x16', [x for i in range(16) for x in (f'    jmp {{s}}_j{i}', f'{{s}}_j{i}:')]))
    t.append(Test('A', 'jsr/rts x8', ['    jsr io_rts'] * 8))
    t.append(Test('A', 'self-modifying sta+lda# x16',
                  [x for i in range(16) for x in (f'    sta {{s}}_m{i}+1', f'{{s}}_m{i}:', '    lda #0')]))
    # A': the same 88-cycle body at four code placements.
    body = ['    clc', '    lda $a4', '    adc $a4', '    sta $a4'] * 8
    for off in (0x00, 0x7E, 0xF0, 0xF8):
        t.append(Test("A'", f'88-cycle body at page+${off:02X}', body, align=off, param=off))
    # B: a register write ($DF04, no DMA) and P cycles of work.
    for p in B_PADS:
        t.append(Test('B', f'sta $df04 + {p}', ['    sta $df04'] + pad(p), setup=reu_setup(), param=p, ios=1))
    for p in BP_PADS:
        t.append(Test("B'", f'sta $d020 + {p}', ['    sta $d020'] + pad(p), setup=['    lda #0'], param=p, ios=1))
    # C: a 1-byte DMA (REU -> C64 $C200, fixed C64 address) and P cycles of work.
    for p in B_PADS:
        t.append(Test('C', f'dma + {p}', ['    sta $df01'] + pad(p), setup=reu_setup(), param=p, ios=1))
    for mode, what, n, c64 in ((0x00, 'incrementing', 2048, 0xC800), (0x80, 'fixed C64', 4096, TARGET),
                               (0xC0, 'fixed C64+REU', 4096, TARGET)):
        for p in (0, 32):
            t.append(Test("C'", f'dma {what} + {p}', ['    sta $df01'] + pad(p), n=n,
                          setup=reu_setup(c64=c64, mode=mode), param=(what, p), ios=1))
    # D: reading the byte the DMA just wrote, against reading another.
    for p in D_PADS:
        for addr, what in ((TARGET, 'target'), (CONTROL, 'control')):
            t.append(Test('D', f'dma + {p} + ldx {what}', ['    sta $df01'] + pad(p) + [f'    ldx ${addr:04x}'],
                          setup=reu_setup(), param=(p, what), ios=1))
    # E: k writes back to back then P, against each write followed by P/k (same total).
    writes = ['    stx $df05', '    sty $df04', '    sta $df01']
    for k in (1, 2, 3):
        ws = writes[3 - k:]
        for p in E_PADS:
            each = 2 * round(p / k / 2)
            if 0 < each < 2:
                each = 2
            t.append(Test('E', f'{k} clustered + {each * k}', ws + pad(each * k), setup=reu_setup(),
                          param=(k, 'clustered', each * k), ios=k))
            t.append(Test('E', f'{k} spread, {each} each', [x for w in ws for x in [w] + pad(each)],
                          setup=reu_setup(), param=(k, 'spread', each * k), ios=k))
    # F: the floor's plain loop, three layouts, on two synthetic runs.
    for run in RUNS:
        for variant in ('base', 'clustered', 'e2'):
            t.append(Test('F', f'floor {variant}, {run}', [], n=RUN_LEN * RUN_COUNT,
                          setup=floor_setup(RUNS[run]), param=(variant, run), extra=[variant]))
    # G: the game's other I/O (appended, so the earlier slots keep their numbers): a SID write (the HUD
    # IRQ's sound), a CIA write, the raster read band_publish polls, the REU status read.
    for op, what in (('sta $d400', 'SID write'), ('sta $dc0c', 'CIA write'), ('lda $d012', 'raster read'),
                     ('lda $df00', 'REU status read')):
        for p in G_PADS:
            t.append(Test('G', f'{what} + {p}', [f'    {op}'] + pad(p), setup=['    lda #0'], param=(what, p), ios=1))
    # H: a DMA of L bytes (REU -> C64 $C200, fixed C64 address), the length rewritten each time (the C64U
    # leaves it at 1 after a transfer). Iterations sized for ~50 ms at 1 us a byte.
    for length in H_LENGTHS:
        n = min(4096, 50_000 // (length + 3)) & ~0xFF or 128
        t.append(Test('H', f'dma of {length} bytes', ['    stx $df07', '    sta $df01'], n=n,
                      setup=reu_setup(x=length), param=length, ios=2))
    return t


def gen():
    ts = tests()
    assert len(ts) <= 256
    out = [
        '; GENERATED by tools/iobench.py (`gen`): do not edit. C64U I/O timing benchmark; see that script.',
        f'; {len(ts)} slots, u16 CIA 2 timer A ticks at ${RESULTS:04X} + 2 * slot. IRQs off throughout.',
        '',
    ]
    out += [f'{k} = ${v:02x}' for k, v in ZP.items()]
    out += ['',
            '.macro tstart', '    lda #$ff', '    sta $dd04', '    sta $dd05', '    lda #$11', '    sta $dd0e', '.endm',
            '', '.macro tstop slot', '    lda #$00', '    sta $dd0e', '    lda $dd04', '    eor #$ff',
            f'    sta ${RESULTS:04x}+\\slot*2', '    lda $dd05', '    eor #$ff', f'    sta ${RESULTS:04x}+1+\\slot*2',
            '.endm', '', '.section .text.iobench,"ax",@progbits', '',
            '; Turbo probe (4096 x 5 cycles) into $C044/$C045, as bench.S.',
            '.globl io_turbo_probe', 'io_turbo_probe:', '    sei', '    ldy #16', '    ldx #0', '    tstart',
            'iotp_l:', '    inx', '    bne iotp_l', '    dey', '    bne iotp_l', '    lda #$00', '    sta $dd0e',
            '    lda $dd04', '    eor #$ff', '    sta $c044', '    lda $dd05', '    eor #$ff', '    sta $c045',
            '    cli', '    rts', '',
            'io_rts:', '    rts', 'io_lookup:', '    rts', '',
            '.globl iobench_all', 'iobench_all:', '    sei',
            '    ldx #$7f', 'iob_save:', '    lda $80,x', '    sta $c700,x', '    dex', '    bpl iob_save', '']
    for i, t in enumerate(ts):
        s = f's{i}'
        out.append(f'; slot {i} [{t.group}] {t.name} (x{t.n})')
        if t.align is not None:
            out += [f'    jmp {s}_go', '    .balign 256', f'    .fill {t.align}, 1, $ea', f'{s}_go:']
        out += t.setup
        if t.extra:
            # A floor replica counts its own runs and bytes.
            out += ['    tstart'] + floor_replica(t.extra[0], s) + [f'    tstop {i}', '']
            continue
        lo, hi = t.n & 0xFF, (t.n >> 8) + (1 if t.n & 0xFF else 0)
        # (tstart clobbers A: the setup's A is kept in ZTMP and reloaded once the timer runs.)
        out += ['    sta ZTMP', f'    lda #${lo:02x}', '    sta CNT', f'    lda #${hi:02x}', '    sta CNT+1',
                '    tstart', '    lda ZTMP', f'{s}_loop:']
        out += [line.replace('{s}', s) for line in t.body]
        out += ['    dec CNT', f'    beq {s}_c', f'    jmp {s}_loop', f'{s}_c:', '    dec CNT+1', f'    beq {s}_done',
                f'    jmp {s}_loop', f'{s}_done:', f'    tstop {i}', '']
    out += ['    ldx #$7f', 'iob_load:', '    lda $c700,x', '    sta $80,x', '    dex', '    bpl iob_load',
            '    lda #0', '    sta $df0a', '    cli', '    rts', '']
    text = '\n'.join(out) + '\n'
    ASM.write_bytes(text.encode())
    print(f'{ASM.relative_to(ROOT)}: {len(ts)} slots')
    return ts


def wsl_path(p):
    p = Path(p).resolve()
    return f'/mnt/{p.drive[0].lower()}' + p.as_posix()[2:]


def build():
    gen()
    root = wsl_path(ROOT)
    r = subprocess.run(['wsl', '-d', 'Ubuntu-24.04', '--', 'bash', '-lc',
                        f'cd {root} && bash target-c64/bench/build.sh && python3 tools/branch-audit.py '
                        f'target-c64/bench/iobench.S'], capture_output=True, text=True)
    print(r.stdout[-1500:], r.stderr[-1500:])
    if r.returncode != 0 or 'wrapped: 0' not in r.stdout:
        raise SystemExit(f'build FAILED (exit {r.returncode})')
    print('build OK:', PRG.relative_to(ROOT), PRG.stat().st_size, 'bytes')


def cycles():
    """Each slot's CPU cycles and I/O stores in the interpreter, cached by the PRG's hash."""
    digest = hashlib.sha1(PRG.read_bytes()).hexdigest()[:12]
    cache = OUT / f'iobench-cycles-{digest}.json'
    if cache.exists():
        return json.loads(cache.read_text())
    import bench_cycles
    prg = PRG.read_bytes()
    load = prg[0] | prg[1] << 8
    mem = bytearray(0x10000)
    mem[0xE000:0x10000] = b'`' * 0x2000  # KERNAL calls return at once
    mem[load:load + len(prg) - 2] = prg[2:]
    cpu = bench_cycles.Bench(mem)
    cpu.pc = bench_cycles.symbols(str(ELF))['_start']
    cpu.push(0xFF)
    cpu.push(0xFE)
    for _ in range(400_000_000):
        if cpu.done:
            break
        cpu.step()
    else:
        raise SystemExit('the bench did not finish in the interpreter')
    probes = mem[PROBES] | mem[PROBES + 1] << 8
    res = [dict(cycles=c, io=io) for c, io in cpu.results[probes:]]
    OUT.mkdir(exist_ok=True)
    cache.write_text(json.dumps(res))
    return res


def trace():
    """Every I/O access each slot makes, in the interpreter (no stalls): per slot (its no-stall cycles,
    [(cycle since the timer started, address, write, value)])."""
    import bench_cycles

    class Tracer(bench_cycles.Bench):
        def __init__(self, mem):
            super().__init__(mem)
            self.slots, self.events = [], []

        def read(self, address):
            address &= 0xFFFF
            if self.start is not None and 0xD000 <= address < 0xE000 and address not in (0xDD04, 0xDD05):
                self.events.append((self.cycles - self.start, address, False, 0))
            return super().read(address)

        def write(self, address, value):
            address &= 0xFFFF
            if address == 0xDD0E and value == 0 and self.start is not None:
                self.slots.append((self.cycles - self.start, self.events))
                self.events = []
            elif address == 0xDD0E and value == 0x11:
                self.events = []
            elif self.start is not None and 0xD000 <= address < 0xE000:
                self.events.append((self.cycles - self.start, address, True, value & 0xFF))
            super().write(address, value)

    prg = PRG.read_bytes()
    load = prg[0] | prg[1] << 8
    mem = bytearray(0x10000)
    mem[0xE000:0x10000] = b'`' * 0x2000
    mem[load:load + len(prg) - 2] = prg[2:]
    cpu = Tracer(mem)
    cpu.pc = bench_cycles.symbols(str(ELF))['_start']
    cpu.push(0xFF)
    cpu.push(0xFE)
    for _ in range(400_000_000):
        if cpu.done:
            break
        cpu.step()
    probes = mem[PROBES] | mem[PROBES + 1] << 8
    return cpu.slots[probes:]


def replay(slot, n, model, cap=150):
    """Modelled CIA ticks an iteration for a traced slot: the first `cap` iterations' accesses replayed
    through `model` (the loops are periodic; the floor replicas are replayed whole)."""
    total, events = slot
    per = total / n
    t_cap = total if n <= cap or not events else per * cap
    offset = 0
    for t, addr, write, value in events:
        if t >= t_cap:
            break
        now = t + offset
        offset += model.access(now, addr, write, value) - now
    iters = n if t_cap == total else cap
    return (t_cap + offset) / iters / c64u_timing.TURBO


def fit(path):
    """Fit the bus model's parameters to a run (groups B, C, G) and report its error on all groups."""
    data = json.loads(Path(path).read_text())
    ts, traced, ticks = tests(), trace(), data['ticks']
    assert len(ts) == len(traced) == len(ticks)
    meas = [k / t.n for t, k in zip(ts, ticks)]

    def err(groups, **params):
        out = []
        for i, t in enumerate(ts):
            if t.group in groups:
                m = c64u_timing.BusModel(**params)
                out.append((replay(traced[i], t.n, m) - meas[i]) / meas[i])
        return math.sqrt(sum(e * e for e in out) / len(out)), max(abs(e) for e in out)

    p = dict(c64u_timing.PARAMS)

    def grid(groups, **axes):
        # Exhaustive over the given axes (each a list), the rest of p fixed; returns the best and its rms.
        names = list(axes)
        best = None

        def walk(k, cur):
            nonlocal best
            if k == len(names):
                e = err(groups, **dict(p, **cur))[0]
                if best is None or e < best[0]:
                    best = (e, dict(cur))
                return
            for v in axes[names[k]]:
                cur[names[k]] = v
                walk(k + 1, cur)
        walk(0, {})
        p.update(best[1])
        return best[0]

    b_slots = {'B'}
    grid(b_slots, rephase=[0, 1], t_reu=[x / 4 for x in range(248, 281)], r_reg=list(range(36, 62)))
    grid({'C'}, r_dma=list(range(40, 80)))
    grid(b_slots | {'C'}, t_reu=[p['t_reu'] + d / 4 for d in range(-4, 5)], r_reg=[p['r_reg'] + d for d in (-1, 0, 1)],
         r_dma=[p['r_dma'] + d for d in (-1, 0, 1)])
    grid({'G'}, r_read=list(range(10, 62, 2)), r_cia=list(range(0, 62, 2)))
    print('fitted:', {k: round(v, 2) for k, v in p.items()})
    for g in ('A', 'B', "B'", 'C', "C'", 'D', 'E', 'F', 'G'):
        rms, worst = err({g}, **p)
        print(f'  group {g:<2}: rms {100 * rms:5.2f}%  worst {100 * worst:5.2f}%' +
              ('  (fitted)' if g in ('B', 'C', 'G') else ''))
    print('  F slots, measured -> modelled us a byte:')
    for i, t in enumerate(ts):
        if t.group == 'F':
            m = c64u_timing.BusModel(**p)
            print(f'    {t.name:<32} {meas[i] * 1e6 / PHI2:6.3f} -> {replay(traced[i], t.n, m) * 1e6 / PHI2:6.3f}')
    return p


def run(host):
    import c64u
    dev = c64u.Device(host)
    dev.write(DONE, b'\x00')
    t0 = time.time()
    dev.run_prg(PRG)
    time.sleep(3)
    while True:
        flag = dev.read(DONE, 1)[0]
        if flag in (0xC1, 0xEE):
            break
        if time.time() - t0 > 180:
            raise SystemExit('no result after 180 s')
        time.sleep(1)
    n = len(tests())
    raw = dev.read(RESULTS, 2 * n)
    ticks = [raw[2 * i] | raw[2 * i + 1] << 8 for i in range(n)]
    probes = int.from_bytes(dev.read(PROBES, 2), 'little')
    OUT.mkdir(exist_ok=True)
    path = OUT / f'iobench-{time.strftime("%Y%m%d-%H%M%S")}.json'
    path.write_text(json.dumps(dict(host=host, turbo=flag == 0xC1, probes=probes, ticks=ticks,
                                    prg=hashlib.sha1(PRG.read_bytes()).hexdigest()[:12])))
    print(f'{path.relative_to(ROOT)}: {n} slots in {time.time() - t0:.1f} s, turbo {"on" if flag == 0xC1 else "OFF"}')
    return path


def report(path):
    data = json.loads(Path(path).read_text())
    ts, cyc, ticks = tests(), cycles(), data['ticks']
    assert len(cyc) == len(ts) == len(ticks), (len(cyc), len(ts), len(ticks))
    rows = []
    for t, c, k in zip(ts, cyc, ticks):
        rows.append(dict(t=t, cpi=c['cycles'] / t.n, io=c['io'] / t.n, tpi=k / t.n, us=k / t.n * 1e6 / PHI2))
    # The CPU clock from the no-I/O loops (A): cycles per tick.
    a = [r for r in rows if r['t'].group == 'A' and r['t'].name != 'empty loop']
    mhz = sum(r['cpi'] for r in a) / sum(r['us'] for r in a)
    print(f'turbo {"on" if data["turbo"] else "OFF"} after {data["probes"]} probes; CPU {mhz:.2f} MHz from group A; '
          f'1 CIA tick = {1e6 / PHI2:.4f} us')

    def table(group, title):
        print(f'\n== {title}')
        print(f'{"slot":<44} {"cycles/it":>9} {"io/it":>6} {"ticks/it":>9} {"us/it":>8} {"excess us":>9}')
        for r in rows:
            if r['t'].group == group:
                print(f'{r["t"].name:<44} {r["cpi"]:>9.1f} {r["io"]:>6.2f} {r["tpi"]:>9.3f} {r["us"]:>8.3f} '
                      f'{r["us"] - r["cpi"] / mhz:>9.3f}')

    table('A', 'A: CPU only (excess should be ~0)')
    table("A'", "A': code placement")
    for g, title in (('B', 'B: sta $df04 + P cycles'), ('C', 'C: 1-byte DMA + P cycles')):
        table(g, title)
        sweep([r for r in rows if r['t'].group == g], mhz, title)
    table("B'", "B': sta $d020 + P (VIC)")
    table("C'", "C': DMA address modes")
    table('D', 'D: read after DMA')
    table('E', 'E: k writes clustered vs spread')
    print('\n   clustered - spread (us/it):')
    es = {r['t'].param: r['us'] for r in rows if r['t'].group == 'E'}
    for k in (1, 2, 3):
        print(f'   k={k}: ' + '  '.join(f'P={p}: {es[(k, "clustered", p)] - es[(k, "spread", p)]:+.3f}'
                                     for (kk, kind, p) in es if kk == k and kind == 'clustered'))
    table('G', "G: the game's other I/O")
    table('H', 'H: a DMA of L bytes (length rewritten each time)')
    table('F', 'F: floor plain-loop replicas (an iteration = a byte)')
    fs = {r['t'].param: r['us'] for r in rows if r['t'].group == 'F'}
    for run in RUNS:
        b = fs[('base', run)]
        print(f'   {run}: base {b:.3f} us a byte; clustered {fs[("clustered", run)] - b:+.3f}; '
              f'e2 {fs[("e2", run)] - b:+.3f}')


def sweep(rs, mhz, title):
    """µs a write against the CPU cycles between writes (the iteration's cycles minus the 4-cycle store): an
    ASCII plot, the additive fit and the bus-slot fit (each write waits for a bus-cycle edge; the CPU then
    resumes after `d` us and runs G cycles before the next)."""
    pts = [(r['cpi'] - 4, r['us']) for r in rs]
    lo, hi = min(u for _, u in pts), max(u for _, u in pts)
    print(f'\n   {title}: us a write against the cycles between writes')
    for g, u in pts:
        bar = '#' * int(round((u - 0) / hi * 50))
        print(f'   {g:>6.1f} {u:>7.3f} {bar}')
    # Additive: u = a + g / f.
    n = len(pts)
    sx, sy = sum(g for g, _ in pts), sum(u for _, u in pts)
    sxx, sxy = sum(g * g for g, _ in pts), sum(g * u for g, u in pts)
    b = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    a = (sy - b * sx) / n
    add_rms = math.sqrt(sum((a + b * g - u) ** 2 for g, u in pts) / n)
    # Bus slots: u = T * ceil((d + g / mhz) / T), T a bus cycle (fit near 1/Phi2), d the resume delay.
    best = None
    for ti in range(900, 1101):
        T = ti / 1000 * 1e6 / PHI2
        for di in range(0, 101):
            d = di / 100 * T
            rms = math.sqrt(sum((T * max(1, math.ceil((d + g / mhz) / T - 1e-9)) - u) ** 2 for g, u in pts) / n)
            if best is None or rms < best[0]:
                best = (rms, T, d)
    print(f'   additive: {a:.3f} us + {b * 1000:.2f} ns a cycle (CPU {1000 / mhz:.2f} ns), rms {add_rms * 1000:.0f} ns')
    print(f'   bus slots: T {best[1]:.4f} us, resume {best[2]:.3f} us, rms {best[0] * 1000:.0f} ns')


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('gen')
    sub.add_parser('build')
    sub.add_parser('cycles')
    r = sub.add_parser('run')
    r.add_argument('--host', default=os.environ.get('C64U_HOST', '192.168.66.178'))
    fp = sub.add_parser('fit')
    fp.add_argument('json', nargs='?')
    rp = sub.add_parser('report')
    rp.add_argument('json', nargs='?')
    args = ap.parse_args()
    if args.cmd == 'gen':
        gen()
    elif args.cmd == 'build':
        build()
    elif args.cmd == 'cycles':
        for i, (t, c) in enumerate(zip(tests(), cycles())):
            print(f'{i:>3} [{t.group:<2}] {t.name:<44} {c["cycles"] / t.n:>8.2f} cycles/it {c["io"] / t.n:>5.2f} io/it')
    elif args.cmd == 'run':
        report(run(args.host))
    elif args.cmd == 'fit':
        fit(args.json or max(OUT.glob('iobench-2*.json'), key=lambda p: p.stat().st_mtime))
    elif args.cmd == 'report':
        path = args.json or max(OUT.glob('iobench-2*.json'), key=lambda p: p.stat().st_mtime)
        report(path)


if __name__ == '__main__':
    main()
