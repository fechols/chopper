#!/usr/bin/env bash
# Build the bench PRG with llvm-mos (WSL Ubuntu-24.04): bash target-c64/bench/build.sh
set -euo pipefail
cd "$(dirname "$0")"
CLANG="${MOS_CLANG:-$HOME/.local/llvm-mos/bin/mos-c64-clang}"
mkdir -p ../../dist
"$CLANG" -Os -o ../../dist/bench.prg main.c bench.S -Wl,--Map=../../dist/bench.map
"$HOME/.local/llvm-mos/bin/llvm-objdump" -d ../../dist/bench.prg.elf > ../../dist/bench.lst
echo built ../../dist/bench.prg
