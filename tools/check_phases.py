#!/usr/bin/env python3
"""Audit REU code paging in the linked C64 program (src/paging.rs rules; from ../talonhd/tools/check_phases.py):

- resident code (every section that is not a `.phase.*`) JSRs/JMPs into the window only at a declared
  phase entry point (`ENTRIES`, each called right after `page`);
- a phase's code only jumps into its own functions or into resident code (never another phase, which
  shares the same addresses), and never through an indirect JMP;
- no resident data refers into the window except the phase scratch (not checked: TALON HD's audit does
  not either; keep tables resident).

Runs where llvm-mos is installed (WSL): python3 tools/check_phases.py dist/chopper.elf
"""
import os
import re
import subprocess
import sys
from pathlib import Path

WINDOW = (0x4000, 0xC000)
ENTRIES = ('geometry_frame', 'pixels_frame', 'pixels_panel')
LLVM = Path(os.environ.get('LLVM_MOS', Path.home() / '.local/llvm-mos')) / 'bin'


def objdump(elf, *args):
    return subprocess.run([str(LLVM / 'llvm-objdump'), *args, elf], capture_output=True, text=True,
                          check=True).stdout


def main():
    elf = sys.argv[1] if len(sys.argv) > 1 else 'dist/chopper.elf'
    sections = {}
    for line in objdump(elf, '-h').splitlines():
        p = line.split()
        if len(p) >= 4 and p[0].isdigit():
            sections[p[1]] = (int(p[3], 16), int(p[2], 16))
    phases = {n: s for n, s in sections.items() if n.startswith('.phase.')}
    code = [n for n in sections if n.startswith('.text')]
    entries = set()
    found = set()
    owned = {n: [] for n in phases}
    for line in objdump(elf, '-t').splitlines():
        p = line.split()
        if len(p) >= 2 and p[-1] in ENTRIES:
            entries.add(int(p[0], 16))
            found.add(p[-1])
        if len(p) >= 5 and p[-3] in owned:
            owned[p[-3]].append((int(p[0], 16), max(int(p[-2], 16), 1)))
    if found != set(ENTRIES):
        raise SystemExit(f'missing phase entries: {set(ENTRIES) - found}')
    errors = []
    call = re.compile(r'^\s*([0-9a-f]+):.*\t(jsr|jmp)\t(\(?)\$([0-9a-f]+)')
    for name in code + list(phases):
        start, size = sections[name]
        for line in objdump(elf, '-d', f'--section={name}').splitlines():
            m = call.match(line)
            if not m:
                continue
            at, op, indirect, target = int(m.group(1), 16), m.group(2), m.group(3), int(m.group(4), 16)
            in_window = WINDOW[0] <= target < WINDOW[1]
            if name in phases:
                if indirect:
                    errors.append(f'{name} ${at:04x}: indirect {op} (${target:04x})')
                elif in_window and not start <= target < start + size:
                    errors.append(f'{name} ${at:04x}: {op} ${target:04x} outside the phase')
                elif in_window:
                    starts = {a for a, n in owned[name]}
                    home = [(a, n) for a, n in owned[name] if a <= at < a + n]
                    if not (target in starts or any(a <= target < a + n for a, n in home)):
                        errors.append(f'{name} ${at:04x}: {op} ${target:04x} is not a function of this phase')
            elif in_window and (indirect or target not in entries):
                errors.append(f'{name} ${at:04x}: resident {op} into the window ${target:04x} (not a phase entry)')
    for e in errors[:40]:
        print(e)
    if errors:
        raise SystemExit(f'{len(errors)} paging violation(s)')
    print(f'paging ok: {len(code)} resident code sections, {len(phases)} phases ({", ".join(phases)})')


if __name__ == '__main__':
    main()
