// Assembles asm/*.S with llvm-mos and links with link.ld (pattern: ../mode7/target-c64/build.rs).
use std::{env, path::PathBuf, process::Command};

fn main() {
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let compiler = env::var("MOS_CLANG").unwrap_or_else(|_| "mos-c64-clang".into());
    let sources: Vec<_> = std::fs::read_dir(root.join("asm"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "S"))
        .collect();
    for source in &sources {
        println!("cargo:rerun-if-changed={}", source.display());
        let object = out.join(source.file_stem().unwrap()).with_extension("o");
        let status = Command::new(&compiler)
            .args(["-c", "-g", "-o"])
            .arg(&object)
            .arg(source)
            .status()
            .expect("cannot run mos-c64-clang");
        assert!(status.success(), "assembly failed: {}", source.display());
        println!("cargo:rustc-link-arg={}", object.display());
    }
    // llvm-mc silently mis-encodes an out-of-range branch: audit every source.
    let audit = root.join("../tools/branch-audit.py");
    println!("cargo:rerun-if-changed={}", audit.display());
    let status = Command::new("python3").arg(&audit).args(&sources).status().expect("cannot run python3");
    assert!(status.success(), "branch audit failed");
    println!("cargo:rerun-if-changed=link.ld");
    println!("cargo:rerun-if-env-changed=MOS_CLANG");
    // Phases overlap on purpose (one window); keep relocations for audits.
    println!("cargo:rustc-link-arg=-Wl,--no-check-sections");
    println!("cargo:rustc-link-arg=-T");
    println!("cargo:rustc-link-arg={}", root.join("link.ld").display());
    println!("cargo:rustc-link-arg=-Wl,-Map,{}", out.join("chopper.map").display());
}
