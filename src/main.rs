//! Host tools:
//!
//! - `chopper bake [OUT]`: the REU image (`dist/chopper.reu`).
//! - `chopper frame [--reu R] [--tick N] [--doubled] [--out O]`: autopilot frame `tick` as the C64 draws
//!   it (`O.png`, and `O.bin`, the 8,000 bitmap bytes `tools/emu.py` compares).
//! - `chopper map [--out O] [--level K]`: the ground map (one pixel per texel, 4 pixels per byte decoded).
//! - `chopper flight [--out DIR] [--every N] [--frames K]`: a strip of autopilot frames as PNGs, with the
//!   floor's DMA and run counts.
//! - `chopper agent [--reu R] [--at T] [--out O] [--trace N]`: the attract-mode agent's state after `T`
//!   ticks (`agent::Agent::to_bytes`, what `tools/emu.py --start` pokes), or every `N`th tick traced.
//! - `chopper brain`: the GOAP operators' plans and the baked tree's size.
//! - `chopper pcm [--out O]`: the rotor sound decoded.

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
    let tick = num(args, "--tick", 0.0) as u32;
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
    let every = num(args, "--every", 512.0) as u32;
    let frames = num(args, "--frames", 8.0) as u32;
    let first = num(args, "--from", chopper::agent::LIFT as f32) as u32;
    for f in 0..frames {
        let tick = first + f * every;
        for doubled in [false, true] {
            let (m, st) = package::render(&reu, tick, doubled);
            let name = format!("t{tick:05}{}", if doubled { "d" } else { "" });
            png::save_wide(&dir.join(format!("{name}.png")), 160, &m.decode(BUFFERS[0], COLORS[0]))?;
            println!("{name}: {} DMAs, {} I/O writes, {} runs ({} ground)", st.dmas, st.io_writes, st.spans, st.ground_spans);
        }
    }
    Ok(())
}

/// The agent's state after `--at` ticks, or a trace of its flight.
fn agent(args: &[String]) -> Result<(), String> {
    let reu = match arg(args, "--reu") {
        Some(p) => std::fs::read(p).map_err(|e| e.to_string())?,
        None => package::bake(&world::generate(SEED)),
    };
    let at = num(args, "--at", 0.0) as u32;
    if let Some(every) = arg(args, "--trace") {
        let every: u32 = every.parse().map_err(|_| "--trace N")?;
        let mut a = chopper::agent::Agent::new();
        let mut r: &[u8] = &reu;
        while a.t < at {
            a.step(&mut r);
            if a.t % every == 0 {
                println!(
                    "t {:6} {:8} u {:7.1} v {:7.1} h {:5.1} yaw {:5.1} wp ({}, {})",
                    a.t,
                    chopper::agent::action::NAMES[a.action as usize],
                    a.u as f32 / 256.0,
                    a.v as f32 / 256.0,
                    a.h as f32 / 256.0,
                    a.yaw as f32 * 360.0 / 65536.0,
                    a.wu,
                    a.wv
                );
            }
        }
        return Ok(());
    }
    let a = chopper::agent::at(&reu, at);
    let out = PathBuf::from(arg(args, "--out").unwrap_or("output/agent.bin".into()));
    if let Some(dir) = out.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    std::fs::write(&out, a.to_bytes()).map_err(|e| e.to_string())?;
    println!("{a:?}");
    Ok(())
}

/// The planner's operators, some plans, and the tree.
fn brain() -> Result<(), String> {
    use chopper::goap::{self, OPS, SYMBOLS, s};
    let names = |st: u16| SYMBOLS.iter().enumerate().filter(|(k, _)| st >> k & 1 != 0).map(|(_, y)| y.name).collect::<Vec<_>>().join(" ");
    for st in [s::GROUNDED, s::GROUNDED | s::ROTOR, s::BLOCKED, s::BLOCKED | s::BLOCKED_L, s::BLOCKED | s::BLOCKED_L | s::BLOCKED_R, s::WP_L, s::WP_R | s::BLOCKED_R, s::HIGH, s::HIGH | s::OVER] {
        let plan = goap::plan(st).map(|p| p.iter().map(|&k| OPS[k].name).collect::<Vec<_>>().join(" -> "));
        println!("[{}]: {}", names(st), plan.unwrap_or("none".into()));
    }
    let table = goap::table();
    let tree = goap::pilot_tree(&table);
    let blob = goap::compile();
    println!("tree: {} nodes, depth {}; blob {} bytes", tree.nodes(), tree.depth(), blob.len());
    Ok(())
}

/// The rotor sound decoded (`digital::decode`, signed 8-bit), the reference for tools/test_audio.py.
fn pcm(args: &[String]) -> Result<(), String> {
    let out = PathBuf::from(arg(args, "--out").unwrap_or("output/rotor.pcm".into()));
    let pcm: Vec<u8> = chopper::digital::decode(chopper::digital::ROTOR).iter().map(|&s| s as u8).collect();
    if let Some(dir) = out.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    std::fs::write(&out, &pcm).map_err(|e| e.to_string())?;
    println!("{}: {} frames", out.display(), pcm.len());
    Ok(())
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("bake") => bake(&args),
        Some("frame") => frame(&args),
        Some("map") => map(&args),
        Some("flight") => flight(&args),
        Some("pcm") => pcm(&args),
        Some("agent") => agent(&args),
        Some("brain") => brain(),
        _ => Err("usage: chopper bake [OUT] | frame [--reu R] [--tick N] [--doubled] [--out O] | map [--out O] [--level K] | flight [--out DIR] [--from T] [--every N] [--frames K] | pcm [--out O] | agent [--reu R] [--at T] [--out O] [--trace N] | brain".into()),
    }
}
