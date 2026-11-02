#!/usr/bin/env python3
"""Fail the build when a 6502 conditional branch cannot reach its label.

(Copied from ../sueme64/tools/branch-audit.py; MODE 7 adds numeric local labels, `bne 1f` / `bcc 2b`.)

The llvm-mos assembler silently wraps an out-of-range branch to a label in
the same section instead of reporting an error, which produces code that
assembles, links and runs — into the wrong place. The displacement is
resolved at assembly time, so the wrapped form cannot be told from a valid
branch by looking at the object alone. This audit therefore re-assembles each
source with debug line information and retained local labels, reads the
label every branch names in the source, and compares that label's address
with the branch's encoded target. Usage: branch-audit.py <source.S> [...]
Exit status 1 when any branch does not reach its label.
"""
import argparse
import os
import pathlib
import re
import subprocess
import sys
import tempfile

sdk = pathlib.Path(os.environ.get("LLVM_MOS", str(pathlib.Path.home() / ".local/llvm-mos"))) / "bin"
compiler = os.environ.get("MOS_CLANG", str(sdk / "mos-c64-clang"))
objdump = os.environ.get("MOS_OBJDUMP", str(sdk / "llvm-objdump"))
nm = os.environ.get("MOS_NM", str(sdk / "llvm-nm"))
line_pattern = re.compile(r"^;\s+(.*):(\d+)\s*$")
branch_pattern = re.compile(r"^\s*([0-9a-f]+):\s+(b(?:ne|eq|cc|cs|pl|mi|vc|vs))\s+(-?)\$([0-9a-f]+)")
source_pattern = re.compile(r"^\s*(?:\d+:\s*)?(b(?:ne|eq|cc|cs|pl|mi|vc|vs))\s+([A-Za-z_.$][\w.$]*|\d+[fb])\s*(?:;.*)?$")
numeric_def = re.compile(r"^\s*(\d+):")

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--define", action="append", default=[], metavar="NAME=VALUE",
                    help="assembler definition, matching the production invocation")
parser.add_argument("sources", nargs="+")
args = parser.parse_args()
defines = [f"-Wa,-defsym,{value}" for value in args.define]

bad = 0
audited = 0
with tempfile.TemporaryDirectory() as scratch:
    for source in args.sources:
        source_path = pathlib.Path(source)
        lines = source_path.read_text(encoding="utf-8", errors="replace").splitlines()
        obj = pathlib.Path(scratch) / (source_path.stem + ".o")
        subprocess.run([compiler, "-c", "-g", "-Wa,-L", *defines, str(source_path), "-o", str(obj)], check=True)
        symbols = {}
        for entry in subprocess.run([nm, str(obj)], text=True, capture_output=True, check=True).stdout.splitlines():
            parts = entry.split()
            if len(parts) == 3 and parts[1].lower() in ("t", "d", "r", "n"):
                symbols.setdefault(parts[2], int(parts[0], 16))
        text = subprocess.run([objdump, "-d", "-l", "--no-show-raw-insn", str(obj)], text=True,
                              capture_output=True, check=True).stdout
        # Source line -> address of its first instruction (for numeric labels).
        line_addr = {}
        current_line = None
        for line in text.splitlines():
            m = line_pattern.match(line)
            if m:
                current_line = int(m.group(2))
                continue
            a = re.match(r"^\s*([0-9a-f]+):", line)
            if a and current_line is not None:
                line_addr.setdefault(current_line, int(a.group(1), 16))

        def numeric(ref, at):
            n, direction = ref[:-1], ref[-1]
            rng = range(at, len(lines) + 1) if direction == "f" else range(at - 1, 0, -1)
            for ln in rng:
                d = numeric_def.match(lines[ln - 1])
                if d and d.group(1) == n:
                    for k in range(ln, len(lines) + 1):
                        if k in line_addr:
                            return line_addr[k]
                    return None
            return None

        current_line = None
        for line in text.splitlines():
            m = line_pattern.match(line)
            if m:
                current_line = int(m.group(2))
                continue
            m = branch_pattern.match(line)
            if not m:
                continue
            audited += 1
            pc = int(m.group(1), 16)
            # A wrapped branch near the start of a section lands before it;
            # objdump prints that target with a sign ("beq -$31").
            encoded = int(m.group(4), 16) * (-1 if m.group(3) else 1)
            if current_line is None or current_line > len(lines):
                continue
            s = source_pattern.match(lines[current_line - 1])
            if not s:
                continue
            label = s.group(2)
            if label[0].isdigit():
                intended = numeric(label, current_line)
                if intended is None:
                    continue
            elif label in symbols:
                intended = symbols[label]
            else:
                continue
            if intended != encoded:
                print(f"WRAPPED BRANCH in {source}:{current_line}: {lines[current_line - 1].strip()} "
                      f"(at ${pc:04x}, label at ${intended:04x}, encoded ${encoded:04x}, "
                      f"displacement {intended - (pc + 2):+d})")
                bad += 1
print(f"branch audit: {audited} branches in {len(args.sources)} source(s); wrapped: {bad}")
sys.exit(1 if bad else 0)
