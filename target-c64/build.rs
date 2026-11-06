// Assembles asm/*.S with llvm-mos and links with link.ld (from ../talonhd/target-c64/build.rs).
use std::{env, path::PathBuf, process::Command};

fn main() {
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let compiler = env::var("MOS_CLANG").unwrap_or_else(|_| "mos-c64-clang".into());
    // The boot splash (tools/encode-splash.py), `.incbin`'d by asm/splash.S.
    let splash_dir = root.join("../Assets/Splash");
    let splash = splash_dir.join("chopper_splash.bin");
    println!("cargo:rerun-if-changed={}", splash.display());
    let image = std::fs::read(&splash).expect("missing Assets/Splash/chopper_splash.bin: run tools/encode-splash.py");
    assert!(image.len() == 10001 && image[9000..].iter().all(|&c| c < 16),
            "the splash is 8000 bitmap, 1000 screen and 1000 colour bytes and the background");
    for name in ["startup", "reu", "floor", "poly", "tri", "fixed", "geom", "spans", "blit", "stream", "hud", "splash"] {
        let source = root.join(format!("asm/{name}.S"));
        println!("cargo:rerun-if-changed={}", source.display());
        let object = out.join(format!("{name}.o"));
        let status = Command::new(&compiler)
            .args(["-c", "-g", "-I"])
            .arg(&splash_dir)
            .arg("-o")
            .arg(&object)
            .arg(&source)
            .status()
            .expect("cannot run mos-c64-clang");
        assert!(status.success(), "assembly failed: {name}");
        println!("cargo:rustc-link-arg={}", object.display());
    }
    // llvm-mc silently mis-encodes an out-of-range branch: audit every source (tools/branch-audit.py).
    let audit = root.join("../tools/branch-audit.py");
    println!("cargo:rerun-if-changed={}", audit.display());
    let sources: Vec<_> = std::fs::read_dir(root.join("asm"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "S"))
        .collect();
    let status = Command::new("python3")
        .arg(&audit)
        .arg("--include")
        .arg(&splash_dir)
        .args(&sources)
        .status()
        .expect("cannot run python3");
    assert!(status.success(), "branch audit failed");
    println!("cargo:rerun-if-changed=link.ld");
    // A new build stamp whenever any source changes: `chopper bake --elf` copies it into the image with
    // the REU overlays, and the PRG waits for an image with its own.
    for dir in ["src", "../src"] {
        println!("cargo:rerun-if-changed={}", root.join(dir).display());
    }
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis() as u32 | 1;
    println!("cargo:rustc-env=CHOPPER_BUILD={stamp}");
    println!("cargo:rerun-if-env-changed=MOS_CLANG");
    // Pass -T to clang directly (not via -Wl) so it replaces the default script.
    println!("cargo:rustc-link-arg=-T");
    println!("cargo:rustc-link-arg={}", root.join("link.ld").display());
    println!("cargo:rustc-link-arg=-Wl,-Map,{}", out.join("chopper.map").display());
    // The i32 divide libcall goes to asm/fixed.S's `__wrap___divsi3` (a byte-divisor fast path; libcrt's
    // divmod object, which every divide pulls in, still provides `__real___divsi3`).
    println!("cargo:rustc-link-arg=-Wl,--wrap=__divsi3");
    // And the u32 divide, on the same core (libcrt's `__udivsi3` is then never linked).
    println!("cargo:rustc-link-arg=-Wl,--wrap=__udivsi3");
    // And the u16 divide (`__wrap___udivhi3`: 16 steps; libcrt's is 196 bytes).
    println!("cargo:rustc-link-arg=-Wl,--wrap=__udivhi3");
    // Likewise the i32 multiply: `__wrap___mulsi3`, up to three 16 Ãƒâ€” 16 quarter-square products (libcrt's
    // `__mulsi3` is then never linked).
    println!("cargo:rustc-link-arg=-Wl,--wrap=__mulsi3");
}
