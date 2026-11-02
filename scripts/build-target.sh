#!/usr/bin/env bash
# Build the C64 program (WSL/Linux with rust-mos + llvm-mos SDK); from ../talonhd/scripts/build-target.sh.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
source scripts/env.sh
native_target="$root/target-c64/target"
(cd target-c64 && CARGO_TARGET_DIR="$native_target" cargo build --release "$@")
mkdir -p dist
cp "$native_target/mos-c64-none/release/chopper-c64" dist/chopper.prg
for elf in "$native_target/mos-c64-none/release/deps"/chopper_c64-*.elf; do
    if [[ "${elf%.elf}" -ef "$native_target/mos-c64-none/release/chopper-c64" ]]; then
        cp "$elf" dist/chopper.elf
    fi
done
"$LLVM_MOS/bin/llvm-size" -A dist/chopper.elf | grep -v '^\.debug' || true
ls -l dist/chopper.prg
