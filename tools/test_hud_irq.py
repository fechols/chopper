#!/usr/bin/env python3
"""Run the linked HUD raster IRQ (`asm/hud.S` `hud_irq`, from dist/chopper.prg) on random tables and check
what it sets up, row by row, against `chopper::hud`'s contract (after ../talonhd/tools/test_hud_irq.py):

- each row's 8 Ys, 8 X lows, `$D010`, `$D015` and the 8 sprite pointers (`$5FF8`) from the table shown,
  the Ys moved by the shake's HDY and the Xs by its HDX (`hud_xy`: a carry flips the sprite's `$D010` bit);
- row 0's shake (`hud_shake`, modelled by `shake` below): a rise of the hit flash starts SHAKE_FRAMES of it,
  the offsets from its tables (halved over the last frames), the fine scroll in `$D016`/`$D011`;
- the next row's IRQ line (plus HDY) in `$D012`, or row 0's (`TOP_LINE`) after the last row;
- row 0 takes the other table when READY is set (and clears READY);
- the raster interrupt acknowledged, A and X restored, nothing written outside the VIC, the pointers and
  the HUD page.

    python tools/test_hud_irq.py [--prg dist/chopper.prg] [--elf dist/chopper.elf] [--tables 60]
"""
import argparse
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from emu import C64, symbols  # noqa: E402
from native_cpu import I  # noqa: E402

HUD = 0x8B00
READY, FRONT, NEXT, BUF = HUD, HUD + 1, HUD + 2, HUD + 0x10
ROW, ROWS, TOP_LINE = 27, 4, 20
R_LINE, R_MSB, R_EN, R_Y, R_X, R_P = 0, 1, 2, 3, 11, 19
PTRS = 0x5FF8
SENTINEL = 0xFFF0
# The shake (asm/hud.S `hud_shake`): its state in the HUD page, the hit flash it watches.
SHAKE, LASTF, HDX, HDY = HUD + 0x0A, HUD + 0x0B, HUD + 0x0C, HUD + 0x0D
FLASH = 0x8ED3
SHAKE_FRAMES, REST_X, REST_Y = 30, 4, 3
SHAKE_DX = [2, -3, 1, -4, 3, -1, 0, -2, 3, -4, 2, -1, 1, -3, 3, -2]
SHAKE_DY = [3, -2, 1, -3, 4, -1, 2, -3, 0, 3, -2, 4, -1, 1, -3, 2]


def shake(flash, lastf, frames):
    """`hud_shake` on row 0: (SHAKE after, the picture's offsets p and q)."""
    if flash > lastf:
        frames = SHAKE_FRAMES
    if frames == 0:
        return 0, 0, 0
    i = frames & 15
    p, q = SHAKE_DX[i], SHAKE_DY[i]
    if frames < 10:
        p, q = p >> 1, q >> 1
    return frames - 1, p, q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prg', default='dist/chopper.prg')
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--tables', type=int, default=60)
    args = ap.parse_args()
    prg = Path(args.prg).read_bytes()
    load = prg[0] | prg[1] << 8
    base = bytearray(0x10000)
    base[load:load + len(prg) - 2] = prg[2:]
    irq = symbols(args.elf)['hud_irq']
    # The handler runs relocated (asm/startup.S copies .victext's pages from its load image).
    rng = random.Random(1983)
    worst = 0
    for t in range(args.tables):
        mem = bytearray(base)
        cpu = C64(mem, bytearray(16))
        # Startup's relocation of .victext (the handler's section): from its load image to $9B00.
        syms = symbols(args.elf) if t == 0 else syms
        src, dst, pages = syms['__victext_load_start'], syms['__victext_start'], syms['__victext_pages']
        mem[dst:dst + 256 * pages] = mem[src:src + 256 * pages]
        # Two random tables, 1-4 rows each.
        for half in (0, 0x80):
            n = rng.randint(1, ROWS)
            line = TOP_LINE
            for r in range(ROWS):
                at = BUF + half + r * ROW
                if r < n:
                    mem[at + R_LINE] = line
                    for k in range(1, ROW):
                        mem[at + k] = rng.randrange(256)
                    line = min(250, line + rng.randint(2, 60))
                else:
                    mem[at:at + ROW] = bytes(ROW)
        mem[FRONT] = rng.choice((0, 0x80))
        mem[READY] = rng.randint(0, 1)
        mem[NEXT] = 0
        # The shake: under way or not, the flash rising or not; no pitch sweeps.
        mem[SHAKE] = rng.choice((0, 0, rng.randint(1, SHAKE_FRAMES)))
        mem[LASTF] = rng.randrange(256)
        mem[FLASH] = rng.choice((mem[LASTF], rng.randrange(256)))
        mem[0xFF20] = mem[0xFF27] = 0
        cpu.io[0x011] = rng.choice((0x00, 0x10))
        for call in range(ROWS + 2):
            front, ready, nxt = mem[FRONT], mem[READY], mem[NEXT]
            if nxt == 0 and ready:
                front ^= 0x80
            row = BUF + front + nxt
            if nxt == 0:
                frames, p, q = shake(mem[FLASH], mem[LASTF], mem[SHAKE])
                hdx, hdy = (REST_X - p) & 255, -q & 255
                display = cpu.io[0x011] & 0x10
            a, x = rng.randrange(256), rng.randrange(256)
            cpu.a, cpu.x = a, x
            cpu.io[0x019] = 1
            cpu.push(SENTINEL >> 8)
            cpu.push(SENTINEL & 0xFF)
            cpu.push(cpu.p)
            cpu.p |= I
            cpu.pc = irq
            before = bytes(mem)
            steps = 0
            while cpu.pc != SENTINEL:
                cpu.step()
                steps += 1
                assert steps < 1000, 'the handler does not return'
            worst = max(worst, cpu.cycles)
            cpu.cycles = 0
            where = f'table {t} call {call} (row offset {nxt})'
            assert (cpu.a, cpu.x) == (a, x), f'{where}: A/X not restored'
            assert cpu.io[0x019] & 1 == 0, f'{where}: not acknowledged'
            if nxt == 0:
                assert (mem[SHAKE], mem[LASTF]) == (frames, before[FLASH]), f'{where}: shake state'
                assert (mem[HDX], mem[HDY]) == (hdx, hdy), f'{where}: shake offsets'
                assert cpu.io[0x016] == (p + (REST_X | 0x10)) & 255, f'{where}: $D016'
                assert cpu.io[0x011] & 0x7F == (display | 0x20 | REST_Y) + q & 0x7F, f'{where}: $D011'
            msb = mem[row + R_MSB]
            for c in range(8):
                x = mem[row + R_X + c] + hdx
                msb ^= (x > 255) << c
                assert cpu.io[0x001 + 2 * c] == (mem[row + R_Y + c] + hdy) & 255, f'{where}: sprite {c} Y'
                assert cpu.io[0x000 + 2 * c] == x & 255, f'{where}: sprite {c} X'
                assert mem[PTRS + c] == mem[row + R_P + c], f'{where}: sprite {c} pointer'
            assert cpu.io[0x010] == msb and cpu.io[0x015] == mem[row + R_EN], f'{where}: $D010/$D015'
            if nxt == 0 and ready:
                assert mem[FRONT] == front and mem[READY] == 0, f'{where}: no table swap'
            following = nxt + ROW
            if following < ROWS * ROW and mem[BUF + front + following + R_LINE]:
                line = (mem[BUF + front + following + R_LINE] + hdy) & 255
                assert mem[NEXT] == following and cpu.io[0x012] == line, f'{where}: next row'
            else:
                assert mem[NEXT] == 0 and cpu.io[0x012] == TOP_LINE, f'{where}: back to row 0'
            changed = [i for i in range(0x10000) if mem[i] != before[i]]
            allowed = set(range(PTRS, PTRS + 8)) | set(range(HUD, HUD + 0x100)) | set(range(0x100, 0x200))
            stray = [i for i in changed if i not in allowed]
            assert not stray, f'{where}: wrote ${stray[0]:04X}'
    print(f'hud_irq matches the contract on {args.tables} table pairs; at most {worst} cycles a row '
          f'({worst / 64:.1f} us at 64 MHz, before the I/O bus waits)')


if __name__ == '__main__':
    main()
