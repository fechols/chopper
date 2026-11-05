#!/usr/bin/env python3
"""Cross-overlay references in the C64 build: every call (or data reference) from code in one REU overlay
to a symbol in another overlay that is not in RAM at the same time (target-c64/src/main.rs's fetch order).

Overlays share windows ($E000, $A000, $F010), so such a call jumps into whatever overlay is loaded there:
the effects pass once called the war's `heading` (`.text.world`) and ran its own code from the middle.

Reads the post-LTO object of a `-C save-temps` build (as target/sizes.sh makes it, in target/c64-sizes;
`--build` makes it first) through llvm-objdump in WSL. Exits 1 when it finds any.

    python tools/overlay_calls.py [--build]
"""
import subprocess
import sys
from collections import defaultdict

ROOT = '/mnt/c/Work/Projects/GitHub/chopper'
OVERLAYS = ['ai', 'hud', 'geomsetup', 'actors', 'world', 'world2', 'world3', 'world4', 'combat', 'fx',
            'cockpit', 'dyn', 'combatlib', 'deck', 'clip', 'sfx', 'sfxeng', 'sfxlib']
WORLD = {'world', 'world2', 'world3', 'world4'}
# Which overlays are in RAM while each one's code runs (besides the resident program). From the main
# loop: the AI and the war's phase with the deck (window 2); combat and fx with combatlib; the cockpit
# (window 2) while fx, the HUD, the actors and the grids' set-up have window 1; the moving objects (dyn)
# with the clipping (window 3).
TOGETHER = {
    'ai': {'deck'},
    **{w: {'deck'} for w in WORLD},
    'deck': {'ai'} | WORLD,
    'combat': {'combatlib'},
    'fx': {'combatlib', 'cockpit'},
    'combatlib': {'combat', 'fx'},
    'cockpit': {'fx', 'hud', 'actors', 'geomsetup'},
    'hud': {'cockpit'},
    'actors': {'cockpit'},
    'geomsetup': {'cockpit'},
    'sfx': {'sfxlib'},
    'sfxeng': {'sfxlib'},
    'sfxlib': {'sfx', 'sfxeng'},
    'dyn': {'clip'},
    'clip': {'dyn'},
}


def wsl(cmd):
    return subprocess.run(['wsl', '-d', 'Ubuntu-24.04', '--', 'bash', '-lc', cmd], capture_output=True,
                          text=True).stdout


def overlay(section):
    """The overlay an input section goes to (link.ld: `.text.<name>` and `.text.<name>.*`), or None."""
    for o in OVERLAYS:
        for kind in ('.text.', '.rodata.', '.data.'):
            if section == kind + o or section.startswith(kind + o + '.'):
                return o
    return None


def main():
    if '--build' in sys.argv:
        print(wsl(f'cd {ROOT} && bash target/sizes.sh 2>&1 | head -3'), end='')
    obj = wsl(f'ls -t {ROOT}/target/c64-sizes/mos-c64-none/release/deps/chopper_c64-*.rcgu.o | head -1').strip()
    if not obj:
        sys.exit('no save-temps object: run with --build (or target/sizes.sh) first')
    od = '~/.local/llvm-mos/bin/llvm-objdump'
    # Each symbol's section, and each section's functions by offset (to name a section+offset target).
    where, funcs = {}, defaultdict(list)
    for line in wsl(f'{od} -t -C {obj}').splitlines():
        p = line.split(None, 5)
        if len(p) == 6 and len(p[0]) == 8 and p[3].startswith('.'):
            name = p[5].split()[-1] if ' ' in p[5] else p[5]
            where[name] = p[3]
            if p[2] == 'F':
                funcs[p[3]].append((int(p[0], 16), name.rsplit('::h', 1)[0]))

    def named(sym, addend):
        best = [n for at, n in sorted(funcs.get(sym, [])) if at <= addend]
        return best[-1] if best else sym
    # Each section's references.
    bad, seen = [], defaultdict(set)
    section = None
    for line in wsl(f'{od} -r {obj}').splitlines():
        if line.startswith('RELOCATION RECORDS FOR ['):
            section = line[len('RELOCATION RECORDS FOR ['):-2]
            continue
        p = line.split()
        if len(p) < 3 or section is None:
            continue
        sym, _, add = p[2].partition('+')
        sym = sym.split('-')[0]
        src, dst = overlay(section), overlay(where.get(sym, sym))
        if src is None or dst is None or src == dst:
            continue
        name = named(sym, int(add, 16) if add else 0)
        seen[(src, dst)].add(name)
        if dst not in TOGETHER.get(src, set()):
            bad.append((src, dst, name))
    for (src, dst), syms in sorted(seen.items()):
        ok = dst in TOGETHER.get(src, set())
        print(f"{'ok ' if ok else 'BAD'} {src} -> {dst}: {', '.join(sorted(syms)[:6])}{' ...' if len(syms) > 6 else ''}")
    if bad:
        print(f'{len(bad)} references into overlays not loaded with the caller')
        sys.exit(1)
    print('no references into overlays not loaded with the caller')


if __name__ == '__main__':
    main()
