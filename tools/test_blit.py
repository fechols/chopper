#!/usr/bin/env python3
"""Run the linked sprite kernel `draw_cols_asm` (from dist/chopper.prg, relocated as the boot does) in a
6502 interpreter and compare it with a direct port of the Rust reference `chopper::sprite::draw_cols` on
randomized column records (item count, then Run / Mix items, clipped to [TOP, BOT)), drawn one screen band
at a time into the 320-byte STAGE row as `sprite_band` does. Copied from ../talonhd/tools/test_blit.py
(e0e891c) by way of ../mode7 (5014bc1); main() is for the band layout and the relocation.

    python tools/test_blit.py [dist/chopper.prg] [dist/chopper.elf]
"""
import random
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_cpu import Cpu  # noqa: E402

JOB, CHUNK, CHUNK_LEN, STAGE = 0x8D70, 0x5400, 0x280, 0x8C00
SENTINEL = 0xFFFF
BITMAP = 0x6000


def symbol(elf, name):
    nm = Path.home() / '.local/llvm-mos/bin/llvm-nm'
    for cmd in ([str(nm), elf], ['wsl', '-d', 'Ubuntu-24.04', '--', 'bash', '-lc',
                                  f'~/.local/llvm-mos/bin/llvm-nm {wsl_path(elf)}']):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
            break
        except (OSError, subprocess.CalledProcessError):
            continue
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == name:
            return int(parts[0], 16)
    raise SystemExit(f'{name} not in {elf}')


def wsl_path(p):
    p = str(Path(p).resolve())
    return '/mnt/' + p[0].lower() + p[2:].replace('\\', '/')


def line_offset(y):
    return (y >> 3) * 320 + (y & 7)


def reference(ram, dst, y, top, bot, src, ncols):
    for _ in range(ncols):
        items = ram[src]
        src += 1
        for _ in range(items):
            b0, b1 = ram[src], ram[src + 1]
            line = b0 | (b1 & 1) << 8
            if b1 & 0x80:
                if top <= line < bot:
                    a = dst + line_offset((y + line) & 0xFFFF)
                    ram[a] = ram[a] & ram[src + 3] | ram[src + 2]
                src += 4
            else:
                n = ram[src + 2]
                data = src + 3
                for l in range(max(line, top), min(line + n, bot)):
                    ram[dst + line_offset((y + l) & 0xFFFF)] = ram[data + l - line]
                src = data + n
        dst += 8


def random_records(rng, h, ncols, band):
    """Column records for lines [band[0], band[1]) of an h-line sprite, as the encoder lays them out."""
    out = bytearray()
    for _ in range(ncols):
        items, body, line = 0, bytearray(), band[0]
        while line < band[1] and items < 250:
            line += rng.choice([0, 0, 1, 2, 5])
            if line >= band[1]:
                break
            if rng.random() < 0.45:
                body += bytes([line & 255, line >> 8 | 0x80, rng.randrange(256), rng.randrange(256)])
                line += 1
            else:
                n = min(rng.randint(1, 255), band[1] - line)
                body += bytes([line & 255, line >> 8, n]) + bytes(rng.randrange(256) for _ in range(n))
                line += n
            items += 1
        out += bytes([items]) + body
    return out


def main():
    prg = Path(sys.argv[1] if len(sys.argv) > 1 else 'dist/chopper.prg').read_bytes()
    elf = sys.argv[2] if len(sys.argv) > 2 else 'dist/chopper.elf'
    entry = symbol(elf, 'draw_cols_asm')
    # draw_cols_asm runs from .hitext: do the boot copy (asm/startup.S) here.
    load, run, pages = (symbol(elf, n) for n in ('__hitext_load_start', '__hitext_start', '__hitext_pages'))
    base = prg[0] | prg[1] << 8
    image = bytearray(65536)
    image[base:base + len(prg) - 2] = prg[2:]
    image[run:run + 256 * pages] = image[load:load + 256 * pages]
    rng = random.Random(1)
    cases = 0
    while cases < 400:
        # As sprite_band hands it over: one screen band (the 320-byte STAGE row, lines 0-7), the sprite's
        # top somewhere around it, one sprite band (32 lines when tall) crossing it.
        h = rng.choice([1, 5, 8, 10, 32, 44, 66, 76, 120])
        sband = rng.randrange(25)
        stop = rng.randint(sband * 8 - h + 1, sband * 8 + 7)
        lo = sband * 8 - stop
        top, bot = max(lo, 0), min(lo + 8, h)
        if top >= bot:
            continue
        if h <= 32:
            band = (0, h)
        else:
            b = rng.randint(top >> 5, (bot - 1) >> 5)
            band = (b * 32, min(h, b * 32 + 32))
        ncols = rng.randint(1, 13)
        col = rng.randrange(40 - ncols + 1)
        recs = random_records(rng, h, ncols, band)
        if len(recs) > CHUNK_LEN:
            continue
        y = stop - sband * 8
        mem = bytearray(image)
        for a in range(STAGE, STAGE + 320):
            mem[a] = rng.randrange(256)
        mem[CHUNK:CHUNK + len(recs)] = recs
        dst = STAGE + col * 8
        job = bytearray(16)
        job[0:2] = dst.to_bytes(2, 'little')
        job[2:4] = (y & 0xFFFF).to_bytes(2, 'little')
        job[4:6] = top.to_bytes(2, 'little')
        job[6:8] = bot.to_bytes(2, 'little')
        job[8:10] = CHUNK.to_bytes(2, 'little')
        job[10] = ncols
        mem[JOB:JOB + 16] = job
        want = bytearray(mem)
        reference(want, dst, y, top, bot, CHUNK, ncols)
        cpu = Cpu(mem, [(0xF0, 0x100), (0x100, 0x200), (STAGE, STAGE + 320)])
        cpu.pc = entry
        cpu.push((SENTINEL - 1) >> 8)
        cpu.push((SENTINEL - 1) & 255)
        for _ in range(5_000_000):
            cpu.step()
            if cpu.pc == SENTINEL:
                break
        else:
            raise SystemExit(f'case {cases}: no return')
        diff = [a for a in range(STAGE, STAGE + 320) if mem[a] != want[a]]
        if diff:
            a = diff[0]
            raise SystemExit(f'case {cases} h={h} y={y} top={top} bot={bot} band={band} ncols={ncols}: '
                             f'{len(diff)} bytes differ, first ${a:04x} got {mem[a]:02x} want {want[a]:02x}')
        cases += 1
    print('draw_cols_asm matches the reference on 400 random column sets, one band at a time')


if __name__ == '__main__':
    main()
