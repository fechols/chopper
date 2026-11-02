//! Host tools:
//!
//! - `chopper bake [OUT]`: the REU image (`dist/chopper.reu`).
//! - `chopper frame [--reu R] [--tick N] [--doubled] [--out O]`: autopilot frame `tick` as the C64 draws
//!   it (`O.png`, and `O.bin`, the 8,000 bitmap bytes `tools/emu.py` compares).
//! - `chopper map [--out O] [--level K]`: the ground map (one pixel per texel, 4 pixels per byte decoded).
//! - `chopper flight [--out DIR] [--every N] [--frames K]`: a strip of autopilot frames as PNGs, with the
//!   floor's DMA and run counts.

use chopper::format::COLORS;
use chopper::layout::BUFFERS;
use chopper::palette::PALETTE;
use chopper::{package, png, world};
use std::path::PathBuf;

const SEED: u32 = 1983;

fn arg(args: &[String], name: &str) -> Option<String> {
    args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned()
}

fn num(args: &[String], name: &str, default: f32) -> f32 {
    arg(args, name).map_or(default, |s| s.parse().unwrap_or_else(|_| panic!("{name}: not a number: {s}")))
}

fn bake(args: &[String]) -> Result<(), String> {
    let out = PathBuf::from(args.get(1).cloned().unwrap_or("dist/chopper.reu".into()));
    let image = package::bake(&world::generate(SEED));
    if let Some(dir) = out.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    std::fs::write(&out, &image).map_err(|e| format!("{}: {e}", out.display()))?;
    println!("{}: {} bytes, package version {}", out.display(), image.len(), chopper::format::PACKAGE_VERSION);
    Ok(())
}

fn frame(args: &[String]) -> Result<(), String> {
    let reu = std::fs::read(arg(args, "--reu").unwrap_or("dist/chopper.reu".into())).map_err(|e| e.to_string())?;
    let tick = num(args, "--tick", 0.0) as u16;
    let doubled = args.iter().any(|a| a == "--doubled");
    let out = PathBuf::from(arg(args, "--out").unwrap_or("output/frame".into()));
    let (m, st) = package::render(&reu, tick, doubled);
    let buf = BUFFERS[0];
    png::save_wide(&out.with_extension("png"), 160, &m.decode(buf, COLORS[0]))?;
    let bitmap = &m.ram[buf.bitmap as usize..buf.bitmap as usize + 8000];
    std::fs::write(out.with_extension("bin"), bitmap).map_err(|e| e.to_string())?;
    println!(
        "tick {tick} {}: {} DMAs, {} I/O writes, {} runs ({} ground); {} objects in {} cells, {} triangles, {} dropped",
        if doubled { "doubled" } else { "full" },
        st.dmas,
        st.io_writes,
        st.spans,
        st.ground_spans,
        st.geom.objects,
        st.geom.cells,
        st.tris,
        st.dropped + st.geom.dropped,
    );
    Ok(())
}

/// The map at mip `level`, each texel byte decoded to its four pixels.
fn map(args: &[String]) -> Result<(), String> {
    let out = PathBuf::from(arg(args, "--out").unwrap_or("output/map.png".into()));
    let level = num(args, "--level", 1.0) as usize;
    let w = world::generate(SEED);
    let levels = world::mips(&w.texels);
    let tex = &levels[level];
    let n = (tex.len() as f64).sqrt() as usize;
    let mut rgb = Vec::with_capacity(n * 4 * n);
    // North (v up) at the top.
    for v in (0..n).rev() {
        for u in 0..n {
            let b = tex[v * n + u];
            for k in 0..4 {
                rgb.push(PALETTE[COLORS[((b >> (6 - 2 * k)) & 3) as usize] as usize]);
            }
        }
    }
    png::save(&out, (n * 4) as u32, n as u32, &rgb)?;
    println!("{}: level {level}, {n}x{n} texels", out.display());
    Ok(())
}

fn flight(args: &[String]) -> Result<(), String> {
    let reu = package::bake(&world::generate(SEED));
    let dir = PathBuf::from(arg(args, "--out").unwrap_or("output/flight".into()));
    let every = num(args, "--every", 512.0) as u16;
    let frames = num(args, "--frames", 8.0) as u16;
    for f in 0..frames {
        let tick = f * every;
        for doubled in [false, true] {
            let (m, st) = package::render(&reu, tick, doubled);
            let name = format!("t{tick:05}{}", if doubled { "d" } else { "" });
            png::save_wide(&dir.join(format!("{name}.png")), 160, &m.decode(BUFFERS[0], COLORS[0]))?;
            println!("{name}: {} DMAs, {} I/O writes, {} runs ({} ground)", st.dmas, st.io_writes, st.spans, st.ground_spans);
        }
    }
    Ok(())
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("bake") => bake(&args),
        Some("frame") => frame(&args),
        Some("map") => map(&args),
        Some("flight") => flight(&args),
        _ => Err("usage: chopper bake [OUT] | frame [--reu R] [--tick N] [--doubled] [--out O] | map [--out O] [--level K] | flight [--out DIR] [--every N] [--frames K]".into()),
    }
}
