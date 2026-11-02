#!/usr/bin/env bash
export LLVM_MOS="${LLVM_MOS:-${HOME}/.local/llvm-mos}"
export MOS_CLANG="${MOS_CLANG:-${LLVM_MOS}/bin/mos-c64-clang}"
export RUST_TARGET_PATH="${RUST_TARGET_PATH:-${HOME}/.local/rust-mos-targets}"
export CARGO_TARGET_MOS_C64_NONE_LINKER="$MOS_CLANG"
export PATH="${CARGO_HOME:-${HOME}/.cargo}/bin:${LLVM_MOS}/bin:${PATH}"
