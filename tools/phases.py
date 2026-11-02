#!/usr/bin/env python3
"""Extract the paged code phases (src/paging.rs) from the linked C64 program and stamp the PRG.
(Pattern: ../talonhd/tools/phases.py.)

Every phase is linked with a load address outside the PRG (target-c64/link.ld `phimg`), so the PRG stays
small; `chopper package` puts dist/phases/<name>.bin into the REU image. The stamp (paging::stamp, same
algorithm here) ties the two: it is patched into the PRG's PHASE_STAMP and written into the image header;
the PRG refuses a mismatch.

    python3 tools/phases.py dist/chopper.elf dist/chopper.prg dist/phases   (scripts/build-target.sh)
"""
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LLVM = Path(os.environ.get('LLVM_MOS', Path.home() / '.local/llvm-mos')) / 'bin'
PRG_BASE = 0x0801
WINDOW_LEN = 0x8000


def names():
    src = (ROOT / 'src/paging.rs').read_text(encoding='utf-8')
    m = re.search(r'pub const NAMES: \[&str; PHASES as usize\] = \[([^\]]*)\]', src)
    return re.findall(r'"(\w+)"', m.group(1))


def stamp(images):
    s = 0x5A5A
    for img in images:
        for b in img:
            s = ((s << 1) | (s >> 15)) & 0xFFFF
            s = (s + b) & 0xFFFF
        s = (s + len(img)) & 0xFFFF
    return s


def main():
    elf, prg, out = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    images = []
    for n in names():
        path = out / f'{n}.bin'
        subprocess.run([str(LLVM / 'llvm-objcopy'), '-O', 'binary', f'--only-section=.phase.{n}', elf, str(path)],
                       check=True)
        images.append(path.read_bytes())
        if len(images[-1]) > WINDOW_LEN:
            raise SystemExit(f'phase {n}: {len(images[-1])} bytes > {WINDOW_LEN}')
    s = stamp(images)
    nm = subprocess.run([str(LLVM / 'llvm-nm'), elf], capture_output=True, text=True, check=True).stdout
    addr = next(int(p[0], 16) for p in (l.split() for l in nm.splitlines()) if p and p[-1] == 'PHASE_STAMP')
    data = bytearray(Path(prg).read_bytes())
    off = addr - PRG_BASE + 2
    if data[off:off + 2] != b'\xff\xff':
        raise SystemExit(f'PHASE_STAMP at ${addr:04x}: expected the $FFFF placeholder in the PRG')
    data[off:off + 2] = s.to_bytes(2, 'little')
    Path(prg).write_bytes(data)
    sizes = ', '.join(f'{n} {len(i)}' for n, i in zip(names(), images))
    print(f'phases: {sizes}; stamp {s:04x}')


if __name__ == '__main__':
    main()
