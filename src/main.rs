//! Host tools:
//!
//! - `chopper bake [OUT] [--elf E]`: the REU image (`dist/chopper.reu`), with the REU overlays and build
//!   stamp of the C64 build `E` (`dist/chopper.elf`): bake after building the C64 program.
//! - `chopper frame [--reu R] [--tick N] [--doubled] [--lock HEX] [--out O]`: autopilot frame `tick` as
//!   the C64 draws it, HUD included (`O.png`, and `O.bin`, what `tools/emu.py` compares: the 8,000
//!   bitmap bytes, screen RAM and colour RAM, 1,000 bytes each, then the HUD's table, sprite blocks and
//!   lock state; `--lock` is the lock state before the frame).
//! - `chopper map [--out O] [--level K]`: the ground map (one pixel per texel, 4 pixels per byte decoded).
//! - `chopper flight [--out DIR] [--every N] [--frames K]`: a strip of autopilot frames as PNGs, with the
//!   floor's DMA and run counts.
//! - `chopper agent [--reu R] [--at T] [--out O] [--trace N]`: the attract-mode agent's state after `T`
//!   ticks (`agent::Agent::to_bytes`, what `tools/emu.py --start` pokes), or every `N`th tick traced.
//! - `chopper brain`: the GOAP operators' plans and the baked tree's size.
//! - `chopper pcm [--out O]`: the rotor sound decoded.
//! - `chopper cockpit [--out DIR]`: contact sheets of every gauge's frames, the panel, and the tach's
//!   spool curve measured from the sound.

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
    let out = PathBuf::from(args.get(1).filter(|a| !a.starts_with("--")).cloned().unwrap_or("dist/chopper.reu".into()));
    let mut image = package::bake(&world::generate(SEED));
    let elf = arg(args, "--elf").unwrap_or("dist/chopper.elf".into());
    match std::fs::read(&elf) {
        Ok(e) => package::install_overlays(&mut image, &e).map_err(|err| format!("{elf}: {err}"))?,
        Err(_) => eprintln!("{elf}: not found: the image has no REU overlays (build the C64 program first)"),
    }
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
    let lock = match arg(args, "--lock") {
        Some(h) => {
            let b: Vec<u8> = (0..h.len()).step_by(2).map(|i| u8::from_str_radix(&h[i..i + 2], 16).unwrap_or(0)).collect();
            [b[0], b[1], b[2], b[3]]
        }
        None => [chopper::hud::NONE, 0, 0, 0],
    };
    let (m, st) = package::render_from(&reu, tick, doubled, lock);
    let buf = BUFFERS[0];
    let mut rgb = m.decode(buf, COLORS[0]);
    let back = m.ram[chopper::hud::BACK as usize];
    chopper::hud::composite(&m, back, &mut rgb);
    png::save_wide(&out.with_extension("png"), 160, &rgb)?;
    let mut bin = m.ram[buf.bitmap as usize..buf.bitmap as usize + 8000].to_vec();
    bin.extend_from_slice(&m.ram[buf.screen as usize..buf.screen as usize + 1000]);
    bin.extend(m.color_ram[..1000].iter().map(|c| c & 15));
    // The HUD: the table written, every sprite block, the lock state after the frame.
    bin.extend_from_slice(&m.ram[(chopper::hud::BUF + back as u16) as usize..][..chopper::hud::TABLE]);
    for a in chopper::hud::SLOT_ADDR {
        bin.extend_from_slice(&m.ram[a as usize..a as usize + 64]);
    }
    bin.extend_from_slice(&m.ram[chopper::hud::LOCK_STATE as usize..][..chopper::hud::LOCK_BYTES]);
    std::fs::write(out.with_extension("bin"), bin).map_err(|e| e.to_string())?;
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
            let mut rgb = m.decode(BUFFERS[0], COLORS[0]);
            chopper::hud::composite(&m, m.ram[chopper::hud::BACK as usize], &mut rgb);
            png::save_wide(&dir.join(format!("{name}.png")), 160, &rgb)?;
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

/// Every gauge frame as a contact sheet (drawn through `cockpit::draw` on a model, as the C64 does), and
/// the panel with the gauges at rest.
fn cockpit(args: &[String]) -> Result<(), String> {
    use chopper::cockpit::{self, FRAMES, ROW, TALL, WIDE, gauge};
    let dir = PathBuf::from(arg(args, "--out").unwrap_or("output/cockpit".into()));
    let reu = package::bake(&world::generate(SEED));
    let mut m = chopper::model::Machine::new(&reu);
    package::setup(&mut m, BUFFERS[0]);
    let spool = chopper::instruments::spool();
    for (k, p) in spool.iter().enumerate() {
        if k % 8 == 0 || k + 1 == spool.len() {
            println!("tach frame {k:2}: {:5.1} s, rotor {p:5.1}%", ((k as u32) << cockpit::TACH_SHIFT) as f32 / 60.0);
        }
    }
    for g in 0..gauge::COUNT {
        let (w, h) = (WIDE[g] as usize * 4, TALL as usize * 8);
        let per_row = 16usize;
        let n = FRAMES[g] as usize;
        let rows = n.div_ceil(per_row);
        let sw = per_row * (w + 2);
        let mut sheet = vec![[40u8, 40, 40]; sw * rows * (h + 2)];
        for k in 0..n {
            cockpit::draw(&mut m, g, k as u8);
            let px = m.decode(BUFFERS[0], COLORS[0]);
            let (ox, oy) = ((k % per_row) * (w + 2), (k / per_row) * (h + 2));
            for y in 0..h {
                for x in 0..w {
                    sheet[(oy + y) * sw + ox + x] = px[(ROW as usize * 8 + y) * 160 + cockpit::COL[g] as usize * 4 + x];
                }
            }
        }
        let path = dir.join(format!("{}.png", gauge::NAMES[g].to_lowercase()));
        png::save_wide(&path, sw, &sheet)?;
        println!("{}: {n} frames", path.display());
    }
    let mut m = chopper::model::Machine::new(&reu);
    package::setup(&mut m, BUFFERS[0]);
    let a = chopper::agent::at(&reu, chopper::agent::LIFT + 3000);
    cockpit::update(&mut m, &a, &mut [0xFF; gauge::COUNT]);
    let px = m.decode(BUFFERS[0], COLORS[0]);
    png::save_wide(&dir.join("panel.png"), 160, &px[cockpit::PANEL_ROW as usize * 8 * 160..])?;
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
        Some("cockpit") => cockpit(&args),
        _ => Err("usage: chopper bake [OUT] | frame [--reu R] [--tick N] [--doubled] [--out O] | map [--out O] [--level K] | flight [--out DIR] [--from T] [--every N] [--frames K] | pcm [--out O] | agent [--reu R] [--at T] [--out O] [--trace N] | brain | cockpit [--out DIR]".into()),
    }
}
