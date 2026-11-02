#!/usr/bin/env bash
# Build everything and run it on the C64 Ultimate (Git Bash on Windows, or Linux):
#   1. the C64 program in WSL (scripts/build-target.sh: PRG, phases, stamp, audits);
#   2. the REU image on the host (`chopper package`, which embeds dist/phases);
#   3. run the PRG, then upload the REU image (the PRG waits for it).
# Host from C64U_HOST (default in tools/c64u.py).
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
if command -v wsl.exe >/dev/null; then
    wsl.exe -d Ubuntu-24.04 -- bash -lc "cd '$(wslpath -a . 2>/dev/null || echo /mnt/c${root#/c})' && bash scripts/build-target.sh"
    # Git Bash's /usr/bin/link shadows the MSVC linker: drop it from PATH for cargo.
    PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '^/usr/bin$' | paste -sd:) cargo run --release -q -- package dist/chopper.reu
else
    bash scripts/build-target.sh
    cargo run --release -q -- package dist/chopper.reu
fi
python tools/c64u.py run dist/chopper.prg
python tools/c64u.py upload-reu dist/chopper.reu
