#!/usr/bin/env python3
"""The C64's memory map as built: what holds each byte of the 64 KiB at run time, the free gaps, and each
code overlay's headroom (`docs/memory.md` is this tool's output).

- Code and data from the ELF (`dist/chopper.elf`): every allocated section at its run address (its VMA;
  the load images in the boot region are gone after the boot and are not counted).
- The overlay windows ($E000 and $A000) hold one overlay at a time: listed with each overlay's size against
  the window, not as occupied space.
- Fixed data areas (`src/layout.rs` and friends), by hand below: keep in step when they move.

    python tools/memmap.py [--elf dist/chopper.elf] [--min 32]
"""
import argparse
import struct
from pathlib import Path

# (start, end exclusive, name): the run-time data areas the code addresses directly.
DATA = [
    (0x0002, 0x0100, 'zero page (compiler $02-$7F, fixed.S $80-$9B, kernels $A0-$FF)'),
    (0x0100, 0x0200, 'hardware stack'),
    (0x0400, 0x0800, 'fixed::SQ_LO/SQ_HI multiply tables'),
    (0x5000, 0x5400, 'fixed::SQ_NEG_LO/HI multiply tables'),
    (0x5400, 0x5680, 'SPRITE_CHUNK'),
    (0x5A80, 0x5C00, 'HUD sprite blocks 0-5 (hud::SLOT_ADDR)'),
    (0x5C00, 0x6000, 'screen RAM + sprite pointers'),
    (0x6000, 0x7F40, 'bitmap'),
    (0x7F40, 0x8000, 'HUD sprite blocks 6-8'),
    (0x8000, 0x8300, 'scene block (SCENE_RAM, up to SCENE_MAX)'),
    (0x8A10, 0x8AB0, 'actors::MESHBUF'),
    (0x8AB0, 0x8AE1, 'combat::WRECKS'),
    (0x8B00, 0x8C00, 'hud::PAGE (IRQ state, two tables)'),
    (0x8C00, 0x8D40, 'STAGE (band; set-up scratch: HUD S_*, world SHOTS, actors DYN)'),
    (0x8D40, 0x8D70, 'SPRITE_REC (+ actors::DYN tail)'),
    (0x8D70, 0x8DE2, 'sprite JOB / BSTART / OFFS'),
    (0x8E00, 0x8E40, 'GEO_ARGS'),
    (0x8E40, 0x8E84, 'DIR_RAM'),
    (0x8E84, 0x8ED8, 'combat::PROJ + PLAYER'),
    (0x8F00, 0x8F84, 'XFORM_RAM (geo_xform/geo_affine tables)'),
    (0x8F84, 0x8FF5, 'combat::FXDYN'),
    (0x9000, 0x9800, 'fixed::RECIP'),
    (0x9800, 0x9A80, 'VERTS (set-up: world page half 0)'),
    (0xA000, 0xAF00, 'BAND_RAM (geom.S band state) = window 2'),
    (0xB760, 0xB7B0, 'combat::FX'),
    (0xB800, 0xBB00, 'BT ray table (set-up: world page half 1)'),
    (0xBF00, 0xC000, 'LINEBUF'),
    (0xC000, 0xC400, 'TILES'),
    (0xC400, 0xC640, 'SPANS'),
    (0xC640, 0xC6A0, 'RAYS / SpanArgs'),
    (0xC6A0, 0xC756, 'WALK_ARGS'),
    (0xC780, 0xC7A0, 'POLY_ARGS'),
    (0xC7A0, 0xC7D4, 'TRI_ARGS'),
    (0xC7D4, 0xC7E0, 'FLOOR_SHADOW'),
    (0xC7E0, 0xC800, 'PATS'),
    (0xC800, 0xC910, 'BOUNCE'),
    (0xC910, 0xCA70, 'CELL_RAM (set-up: combat::OBJLIST)'),
    (0xCA80, 0xCA97, 'STATUS'),
    (0xCAA0, 0xCAEE, 'AGENT, CLOCK, GAUGES, NEAR, PHASE'),
    (0xCB00, 0xCD80, 'llvm-mos static frames (.noinit region)'),
    (0xCD80, 0xD000, 'soft stack'),
    (0xD000, 0xE000, 'I/O (RAM beneath: free)'),
    (0xE000, 0xF800, 'Work = window 1'),
    (0xFFFA, 0x10000, 'vectors'),
]
WINDOWS = {0xE000: ('window 1', 0x1800), 0xA000: ('window 2', 0x0F00), 0xF010: ('window 3', 0x07F0)}


def sections(elf):
    b = Path(elf).read_bytes()
    shoff, = struct.unpack_from('<I', b, 0x20)
    shentsize, shnum, shstrndx = struct.unpack_from('<HHH', b, 0x2E)
    hdrs = [struct.unpack_from('<IIIIIIIIII', b, shoff + i * shentsize) for i in range(shnum)]
    names = hdrs[shstrndx]
    out = []
    for h in hdrs:
        name_off, kind, flags, addr, off, size = h[:6]
        name = b[names[4] + name_off:b.index(b'\0', names[4] + name_off)].decode()
        if flags & 2 and size and addr < 0x10000:
            out.append((addr, addr + size, name))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', default='dist/chopper.elf')
    ap.add_argument('--min', type=int, default=32, help='smallest gap listed')
    args = ap.parse_args()
    secs = sections(args.elf)
    overlays = [(a, e, n) for a, e, n in secs if any(a == w for w in WINDOWS)]
    code = [(a, e, n) for a, e, n in secs if (a, e, n) not in overlays and n != '.boottext']
    used = [0] * 0x10000
    for a, e, n in code + DATA:
        if a in WINDOWS and (a, e, n) in DATA:
            pass
        for x in range(a, min(e, 0x10000)):
            used[x] = 1
    # The relocated sections occupy whole pages (asm/startup.S copies pages).
    for a, e, n in code:
        if n in ('.lowtext', '.spantext', '.victext', '.bandtext', '.bandtext2', '.hitext'):
            for x in range(a, (e + 255) & ~255):
                used[x] = 1
    print('## Sections at run time\n')
    for a, e, n in sorted(code):
        print(f'- `{n}`: ${a:04X}-${e - 1:04X} ({e - a} bytes)')
    print('\n## Overlays (one at a time in their window)\n')
    for w, (wname, wlen) in WINDOWS.items():
        for a, e, n in sorted((x for x in overlays if x[0] == w), key=lambda x: x[1] - x[0], reverse=True):
            print(f'- {wname} ${w:04X}: `{n}` {e - a} of {wlen} bytes, {wlen - (e - a)} free')
    print(f'\n## Free gaps (at least {args.min} bytes)\n')
    total = 0
    x = 0x0200
    while x < 0x10000:
        if used[x]:
            x += 1
            continue
        y = x
        while y < 0x10000 and not used[y]:
            y += 1
        if y - x >= args.min:
            print(f'- ${x:04X}-${y - 1:04X}: {y - x} bytes')
            total += y - x
        x = y
    print(f'\nFree in gaps of {args.min}+ bytes: {total} bytes')


if __name__ == '__main__':
    main()
