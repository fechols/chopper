//! Host tools:
//!
//! - `chopper bake [OUT] [--elf E]`: the REU image (`dist/chopper.reu`), with the REU overlays and build
//!   stamp of the C64 build `E` (`dist/chopper.elf`): bake after building the C64 program.
//! - `chopper frame [--reu R] [--tick N | --state S] [--doubled | --full] [--lock HEX] [--out O]`: autopilot frame
//!   `tick` as the C64 draws it, HUD included, or the frame stepped from snapshot `S` (`snapshot`, the
//!   C64's state at the frame's start, as `tools/emu.py` captures it). `O.png`, and `O.bin`, what
//!   `tools/emu.py` compares: the 8,000 bitmap bytes, screen RAM and colour RAM, 1,000 bytes each, the
//!   HUD's table, sprite blocks and lock state, then the agent and the gauges' frames after the frame.
//! - `chopper map [--out O] [--level K]`: the ground map (one pixel per texel, 4 pixels per byte decoded).
//! - `chopper flight [--out DIR] [--every N] [--frames K]`: a strip of autopilot frames as PNGs, with the
//!   floor's DMA and run counts.
//! - `chopper agent [--reu R] [--at T] [--out O] [--trace N]`: the attract-mode agent's state after `T`
//!   ticks (`agent::Agent::to_bytes`, what `tools/emu.py --start` pokes), or every `N`th tick traced.
//! - `chopper brain`: the GOAP operators' plans and the baked tree's size.
//! - `chopper pcm [--out O]`: the rotor sound decoded.
//! - `chopper cockpit [--out DIR]`: contact sheets of every gauge's frames, the panel, and the tach's
//!   spool curve measured from the sound.
//! - `chopper effects [--out DIR]`: contact sheets of the effect sprites (`effects`), one per effect: every
//!   3rd scale down, its frames across, each drawn back out of the baked image by the C64's sprite code,
//!   on the grass, pixels doubled across and zoomed 2x.

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
    let lines = if args.iter().any(|a| a == "--doubled") {
        chopper::floor::Lines::Doubled
    } else if args.iter().any(|a| a == "--full") {
        chopper::floor::Lines::Full
    } else {
        // (The C64's boot default.)
        chopper::floor::Lines::Adaptive
    };
    let out = PathBuf::from(arg(args, "--out").unwrap_or("output/frame".into()));
    let lock = match arg(args, "--lock") {
        Some(h) => {
            let b: Vec<u8> = (0..h.len()).step_by(2).map(|i| u8::from_str_radix(&h[i..i + 2], 16).unwrap_or(0)).collect();
            let mut lock = chopper::hud::NO_LOCK;
            for (l, &x) in lock.iter_mut().zip(&b) {
                *l = x;
            }
            lock
        }
        None => chopper::hud::NO_LOCK,
    };
    let (m, st) = match arg(args, "--state") {
        Some(p) => {
            let snap = chopper::snapshot::Snapshot::parse(&std::fs::read(&p).map_err(|e| format!("{p}: {e}"))?)?;
            package::render_snapshot(&reu, &snap, lines)?
        }
        None => package::render_from(&reu, tick, lines, lock),
    };
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
    // The state carried to the next frame: the agent, the gauges' frames.
    let agent = chopper::layout::AGENT as usize;
    bin.extend_from_slice(&m.ram[agent..agent + chopper::agent::Agent::BYTES]);
    let gauges = chopper::layout::GAUGES as usize;
    bin.extend_from_slice(&m.ram[gauges..gauges + chopper::cockpit::gauge::COUNT]);
    // The war after the frame: the units near, then the page file.
    let near = chopper::world_sim::NEAR as usize;
    bin.extend_from_slice(&m.ram[near..=chopper::world_sim::TURN as usize]);
    let war = chopper::world_sim::STATE as usize;
    bin.extend_from_slice(&m.reu[war..war + chopper::world_sim::STATE_LEN as usize]);
    let brains = chopper::world_sim::BRAINS as usize;
    bin.extend_from_slice(&m.reu[brains..brains + chopper::world_sim::BRAINS_LEN as usize]);
    let pilot = chopper::format::AI_TREES as usize;
    bin.extend_from_slice(&m.reu[pilot..pilot + chopper::brain::PILOT_LEN as usize]);
    // The player's fight and the effects playing.
    let fight = chopper::combat::STATE as usize;
    bin.extend_from_slice(&m.ram[fight..fight + chopper::combat::STATE_LEN as usize]);
    let fx = chopper::combat::FX as usize;
    bin.extend_from_slice(&m.ram[fx..fx + (chopper::combat::FX_MAX as u16 * chopper::combat::FX_ENTRY) as usize]);
    std::fs::write(out.with_extension("bin"), bin).map_err(|e| e.to_string())?;
    println!(
        "tick {tick} {}: {} lines, {} DMAs, {} I/O writes, {} runs ({} ground); {} objects in {} cells, {} triangles, {} dropped",
        format!("{lines:?}").to_lowercase(),
        st.drawn_lines,
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
        use chopper::floor::Lines;
        for (lines, tag) in [(Lines::Full, ""), (Lines::Doubled, "d"), (Lines::Adaptive, "a")] {
            let (m, st) = package::render(&reu, tick, lines);
            let name = format!("t{tick:05}{tag}");
            let mut rgb = m.decode(BUFFERS[0], COLORS[0]);
            chopper::hud::composite(&m, m.ram[chopper::hud::BACK as usize], &mut rgb);
            png::save_wide(&dir.join(format!("{name}.png")), 160, &rgb)?;
            println!("{name}: {} lines, {} DMAs, {} I/O writes, {} runs ({} ground)", st.drawn_lines, st.dmas, st.io_writes, st.spans, st.ground_spans);
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
        let mut a = chopper::agent::Agent::boot();
        let mut r: &[u8] = &reu;
        while a.t < at {
            let t = a.t + 1;
            a.fly(&mut r, t);
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
    let names = |st: u32| SYMBOLS.iter().enumerate().filter(|(k, _)| st >> k & 1 != 0).map(|(_, y)| y.name).collect::<Vec<_>>().join(" ");
    for st in [s::GROUNDED, s::GROUNDED | s::ROTOR, s::BLOCKED, s::BLOCKED | s::BLOCKED_L, s::BLOCKED | s::BLOCKED_L | s::BLOCKED_R, s::WP_L, s::WP_R | s::BLOCKED_R, s::HIGH, s::HIGH | s::OVER, s::HUNT, s::HUNT | s::WP_L, s::HUNT | s::CLOSE, s::HUNT | s::CLOSE | s::WP_R, s::HUNT | s::LOCKED, s::HURT, s::HURT | s::CLOSE, s::HURT | s::ARRIVED, s::HURT | s::GROUNDED | s::ROTOR] {
        let plan = goap::plan(st).map(|p| p.iter().map(|&k| OPS[k].name).collect::<Vec<_>>().join(" -> "));
        println!("[{}]: {}", names(st), plan.unwrap_or("none".into()));
    }
    let table = goap::table();
    let tree = goap::pilot_tree(&table);
    let blob = goap::compile();
    let learns = table.iter().filter(|&&a| a >= goap::LEARN).count();
    println!("tree: {} nodes, depth {}; blob {} bytes; {learns} states at a learn node", tree.nodes(), tree.depth(), blob.len());
    Ok(())
}

/// Every gauge frame as a contact sheet (drawn through `cockpit::draw` on a model, as the C64 does), and
/// the panel with the gauges at rest.
fn cockpit(args: &[String]) -> Result<(), String> {
    use chopper::cockpit::{self, FRAMES, ROW_AT, TALL_AT, WIDE, gauge};
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
        let (w, h) = (WIDE[g] as usize * 4, TALL_AT[g] as usize * 8);
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
                    sheet[(oy + y) * sw + ox + x] = px[(ROW_AT[g] as usize * 8 + y) * 160 + cockpit::COL[g] as usize * 4 + x];
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

/// The war on its own: who holds the island, minute by minute.
fn war(args: &[String]) -> Result<(), String> {
    let minutes = num(args, "--minutes", 10.0) as u32;
    let mut reu = package::bake(&world::generate(SEED));
    // The director's knobs: --knobs ENEMY,ALLIED,PER_PAD,WAVE,WAVE_PER_PAD (strengths in units).
    if let Some(k) = arg(args, "--knobs") {
        let v: Vec<u16> = k.split(',').map(|x| x.parse().map_err(|_| format!("--knobs: {x}?"))).collect::<Result<_, _>>()?;
        let [e, a, p, w, wp] = v[..] else { return Err("--knobs ENEMY,ALLIED,PER_PAD,WAVE,WAVE_PER_PAD".into()) };
        chopper::war::set_knobs(&mut reu, chopper::world_sim::Knobs { strength: [e, a], per_pad: p, wave: w as u8, wave_per_pad: wp as u8 });
    }
    let quiet = args.iter().any(|a| a == "--quiet");
    let mut m = chopper::model::Machine::new(&reu);
    for minute in 0..=minutes {
        let r = chopper::war::report(&m.reu);
        let pads: String = r.owner.iter().map(|&o| if o == chopper::actors::ENEMY { 'E' } else { 'A' }).collect();
        let fleet = if args.iter().any(|a| a == "--fleet") {
            // Ships afloat per side, and the carrier's hit points (0: sunk).
            use chopper::world_sim::{self as ws, ALIVE, SIDE, rec, stats};
            let (mut ships, mut carrier) = ([0u16; 2], 0u8);
            for id in 0..ws::COUNT as usize {
                let r = &m.reu[ws::UNITS as usize + id * ws::RECORD as usize..][..ws::RECORD as usize];
                let k = r[rec::KIND as usize] as usize % chopper::actors::kind::COUNT as usize;
                if r[rec::FLAGS as usize] & ALIVE != 0 && stats::CLASS[k] == 4 {
                    ships[(r[rec::FLAGS as usize] & SIDE) as usize] += 1;
                    if k == chopper::actors::kind::CARRIER as usize {
                        carrier = r[rec::HP as usize];
                    }
                }
            }
            format!("; ships {}/{}, carrier {carrier:3}", ships[0], ships[1])
        } else {
            String::new()
        };
        println!(
            "minute {minute:3}: enemy {:3} alive ({:4} vehicles), {:3} lost; allies {:3} alive ({:4} vehicles), {:3} lost; pads {pads}{fleet}",
            r.alive[0], r.vehicles[0], r.lost[0], r.alive[1], r.vehicles[1], r.lost[1]
        );
        if !quiet && (minute % 5 == 0 || minute == minutes) {
            // North at the top: E enemy, A allied, * contested.
            for sy in (0..16).rev() {
                let row: String = (0..16).map(|sx| [' ', 'E', 'A', '*'][r.control[sy * 16 + sx] as usize]).collect();
                println!("    |{row}|");
            }
        }
        if minute < minutes {
            chopper::war::run(&mut m, minute * 3600, 3600, 4);
        }
    }
    if args.iter().any(|a| a == "--learned") {
        // Each kind's learn nodes: their odds of the first choice (of 256), prior -> now.
        use chopper::brain::{LEAF, unit};
        let blob = chopper::roles::compile();
        let base = chopper::format::TREES as usize;
        for k in 0..chopper::actors::kind::COUNT as usize {
            let root = u16::from_le_bytes([blob[chopper::brain::ROOTS + 2 * k], blob[chopper::brain::ROOTS + 2 * k + 1]]) as usize;
            let prev = if k == 0 { chopper::brain::HEADER_BYTES } else { u16::from_le_bytes([blob[chopper::brain::ROOTS + 2 * k - 2], blob[chopper::brain::ROOTS + 2 * k - 1]]) as usize + 6 };
            let moves: Vec<String> = (prev..=root)
                .step_by(6)
                .filter(|&o| blob[o] >= unit::LEARN0 && blob[o] != LEAF)
                .map(|o| {
                    let (y, n) = (u16::from_le_bytes([blob[o + 4], blob[o + 5]]) as usize, u16::from_le_bytes([blob[o + 2], blob[o + 3]]) as usize);
                    format!("{}/{} {}->{}", unit::NAMES[blob[y + 1] as usize], unit::NAMES[blob[n + 1] as usize], blob[o + 1], m.reu[base + o + 1])
                })
                .collect();
            println!("{:>13}: {}", chopper::actors::kind::NAMES[k], moves.join(", "));
        }
    }
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

/// Contact sheets of the effect sprites as baked: scales 0, 3, .. 30 down, frames across.
fn effects(args: &[String]) -> Result<(), String> {
    use chopper::effects::{EFFECTS, FRAMES, NAMES, SCALES};
    use chopper::effects_bake::{draw_back, read_record};
    let dir = PathBuf::from(arg(args, "--out").unwrap_or("output/effects".into()));
    let reu = package::bake(&world::generate(SEED));
    let u = chopper::effects_bake::install(&mut reu.clone());
    println!(
        "{} frames, 4 phases each: {} bytes of blocks, {} of padding, ${:06X}-${:06X}; largest block {}, largest band chunk {}",
        u.frames,
        u.blocks,
        u.padding,
        chopper::format::EFFECTS_BLOCKS,
        u.end,
        u.max_block,
        u.max_chunk
    );
    let mut m = chopper::model::Machine::new(&reu);
    let grass = PALETTE[COLORS[0] as usize];
    let gap = [40u8, 40, 40];
    let rows: Vec<u8> = (0..SCALES as u8).step_by(3).collect();
    for e in 0..EFFECTS as u8 {
        let nf = FRAMES[e as usize];
        let wmax = rows.iter().map(|&s| read_record(&m.reu, e, s, 0).w as usize).max().unwrap();
        let cw = 2 * wmax + 4;
        let (w, h) = (4 + nf as usize * cw, 4 + rows.iter().map(|&s| read_record(&m.reu, e, s, 0).h as usize + 4).sum::<usize>());
        let mut rgb = vec![gap; w * h];
        let mut y0 = 4;
        for &s in &rows {
            for f in 0..nf {
                let img = draw_back(&mut m, e, s, f, 0);
                let x0 = 4 + f as usize * cw;
                for y in 0..img.h {
                    for x in 0..img.w {
                        let c = img.get(x, y).map_or(grass, |c| PALETTE[COLORS[c as usize] as usize]);
                        let at = (y0 + y as usize) * w + x0 + 2 * x as usize;
                        rgb[at] = c;
                        rgb[at + 1] = c;
                    }
                }
            }
            y0 += read_record(&m.reu, e, s, 0).h as usize + 4;
        }
        let zoom: Vec<[u8; 3]> = (0..2 * h).flat_map(|y| (0..2 * w).map(move |x| (x, y))).map(|(x, y)| rgb[(y / 2) * w + x / 2]).collect();
        let out = dir.join(format!("{}.png", NAMES[e as usize]));
        png::save(&out, 2 * w as u32, 2 * h as u32, &zoom)?;
        println!("{}: {nf} frames x {SCALES} scales", out.display());
    }
    Ok(())
}

/// The cloud shapes as baked, drawn back out by the C64's sprite code on the sky: shapes down, every third
/// roll angle across, each frame's anchor marked in green: output/clouds.png.
fn clouds(args: &[String]) -> Result<(), String> {
    use chopper::clouds::{ANGLES, SHAPES, record};
    use chopper::effects_bake::{draw_back_at, read_record_at};
    let out = PathBuf::from(arg(args, "--out").unwrap_or("output/clouds.png".into()));
    let reu = package::bake(&world::generate(SEED));
    let mut m = chopper::model::Machine::new(&reu);
    let sky = PALETTE[COLORS[1] as usize];
    let mark = PALETTE[COLORS[0] as usize];
    let angles: Vec<u8> = (0..ANGLES as u8).step_by(3).collect();
    let (cw, ch) = (2 * (2 * chopper::clouds::REACH_X as usize) + 8, (chopper::clouds::REACH_UP + chopper::clouds::REACH_DOWN) as usize + 4);
    let (w, h) = (angles.len() * cw, SHAPES * ch);
    let mut rgb = vec![sky; w * h];
    let mut bytes = 0u32;
    for s in 0..SHAPES as u8 {
        for (i, &a) in angles.iter().enumerate() {
            let r = read_record_at(&m.reu, record(s, a));
            bytes += r.phase.iter().map(|p| p.2 as u32).sum::<u32>();
            let img = draw_back_at(&mut m, record(s, a), 0);
            // The anchor at the cell's (centre, REACH_UP + 2).
            let (ox, oy) = (i * cw + cw / 2, s as usize * ch + chopper::clouds::REACH_UP as usize + 2);
            for y in 0..img.h {
                for x in 0..img.w {
                    if let Some(c) = img.get(x, y) {
                        let at = (oy + y as usize - r.ay as usize) * w + ox + 2 * x as usize - 2 * r.ax as usize;
                        rgb[at] = PALETTE[COLORS[c as usize] as usize];
                        rgb[at + 1] = rgb[at];
                    }
                }
            }
            rgb[oy * w + ox] = mark;
        }
    }
    println!("{} frames (every third angle shown): {bytes} bytes of blocks in those", SHAPES * angles.len());
    let z = 2;
    let zoom: Vec<[u8; 3]> = (0..z * h).flat_map(|y| (0..z * w).map(move |x| (x, y))).map(|(x, y)| rgb[(y / z) * w + x / z]).collect();
    png::save(&out, (z * w) as u32, (z * h) as u32, &zoom)?;
    println!("{}", out.display());
    Ok(())
}

/// Wrecks and the living side by side ahead of a hovering camera, drawn at a few ages of the wrecks
/// (`world_sim::WRECK_TICKS`): output/wrecks-<seconds>.png.
fn wrecks(args: &[String]) -> Result<(), String> {
    use chopper::actors::{ENEMY, kind};
    use chopper::layout::{AGENT, CLOCK};
    use chopper::world_sim::{self as ws, ALIVE, NEAR, RECORD, rec};
    let reu = std::fs::read(arg(args, "--reu").unwrap_or("dist/chopper.reu".into())).map_err(|e| e.to_string())?;
    let out = PathBuf::from(arg(args, "--out").unwrap_or("output/wrecks".into()));
    let mut a = chopper::agent::Agent::new();
    let t0 = 20_000u32;
    (a.u, a.v, a.h, a.grounded, a.mode, a.t) = (1024 << 8, (num(args, "--v", 1112.0) as i32) << 8, (num(args, "--h", 5.0) as i32) << 8, 0, chopper::agent::MODE_PLAYER, t0);
    a.dyaw = (a.yaw >> 8) as u8;
    a.pitch = num(args, "--pitch", -2000.0) as i16;
    // A row ahead: (kind, alive) pairs, ids 200 on.
    let row = [(kind::TANK, false), (kind::TANK, true), (kind::APC, false), (kind::TRUCK, false), (kind::AA, false), (kind::GUNSHIP, false)];
    // (The last a little short of the wrecks' lifetime, whatever it is.)
    for age in [1u32, 12, ws::WRECK_TICKS as u32 / 60 - 2] {
        let died = (t0 - age * 60) as u16;
        let mut units = Vec::new();
        for (j, &(k, alive)) in row.iter().enumerate() {
            let mut r = vec![0u8; RECORD as usize];
            r[rec::KIND as usize] = k;
            r[rec::FLAGS as usize] = ENEMY | if alive { ALIVE } else { 0 };
            r[rec::HP as usize] = 20;
            r[rec::ACTION as usize] = ws::act::HOLD;
            r[rec::YAW as usize] = (j * 40) as u8;
            let (u, v) = (1024 - 25 + 10 * j as u16, 1130 + 6 * (j as u16 % 2));
            r[rec::U as usize..rec::U as usize + 2].copy_from_slice(&(u << 4).to_le_bytes());
            r[rec::V as usize..rec::V as usize + 2].copy_from_slice(&(v << 4).to_le_bytes());
            r[rec::LAST as usize..rec::LAST as usize + 2].copy_from_slice(&died.to_le_bytes());
            units.push((ws::UNITS + (200 + j as u32) * RECORD as u32, r));
        }
        let mut near = vec![row.len() as u8];
        for j in 0..row.len() as u16 {
            near.extend((200 + j).to_le_bytes());
        }
        let snap = chopper::snapshot::Snapshot {
            ram: vec![(AGENT, a.to_bytes().to_vec()), (CLOCK, t0.to_le_bytes().to_vec()), (NEAR, near), (ws::PHASE, vec![ws::DECIDE])],
            reu: units,
        };
        let (m, _) = package::render_snapshot(&reu, &snap, chopper::floor::Lines::Full)?;
        let mut rgb = m.decode(BUFFERS[0], COLORS[0]);
        chopper::hud::composite(&m, m.ram[chopper::hud::BACK as usize], &mut rgb);
        let p = PathBuf::from(format!("{}-{age:02}s.png", out.display()));
        png::save_wide(&p, 160, &rgb)?;
        println!("{}", p.display());
    }
    Ok(())
}

/// The carrier: the view from its deck at the start (output/carrier-deck.png) and on approach from astern
/// (output/carrier-approach.png), at tick --at.
fn carrier_views(args: &[String]) -> Result<(), String> {
    use chopper::layout::{AGENT, CLOCK};
    let reu = std::fs::read(arg(args, "--reu").unwrap_or("dist/chopper.reu".into())).map_err(|e| e.to_string())?;
    let t = num(args, "--at", 600.0) as u32;
    let d = chopper::world_sim::carrier(&mut &reu[..], t as u16).ok_or("no carrier afloat")?;
    let id = (chopper::world_sim::HEADER + chopper::world_sim::head::CARRIER as u32) as usize;
    let mut deck = chopper::agent::Agent::new();
    (deck.t, deck.deck) = (t, chopper::agent::PLACE_CARRIER);
    let mut air = chopper::agent::Agent::new();
    let back = num(args, "--back", 40.0) as i32;
    let (s, c) = (chopper::camera::sin(d.yaw), chopper::camera::cos(d.yaw));
    (air.t, air.grounded, air.yaw, air.pitch) = (t, 0, d.yaw, num(args, "--pitch", -1500.0) as i16);
    air.u = ((d.u as i32) << 4) - (s * (back << 8) >> 14);
    air.v = ((d.v as i32) << 4) - (c * (back << 8) >> 14);
    air.h = num(args, "--h", 12.0) as i32 * 256;
    for (name, a) in [("deck", deck), ("approach", air)] {
        let snap = chopper::snapshot::Snapshot {
            ram: vec![
                (AGENT, a.to_bytes().to_vec()),
                (CLOCK, t.to_le_bytes().to_vec()),
                (chopper::world_sim::NEAR, vec![1, reu[id], reu[id + 1]]),
                (chopper::world_sim::PHASE, vec![chopper::world_sim::DECIDE]),
            ],
            reu: vec![],
        };
        let (m, _) = package::render_snapshot(&reu, &snap, chopper::floor::Lines::Full)?;
        let mut rgb = m.decode(BUFFERS[0], COLORS[0]);
        chopper::hud::composite(&m, m.ram[chopper::hud::BACK as usize], &mut rgb);
        let p = PathBuf::from(format!("output/carrier-{name}.png"));
        png::save_wide(&p, 160, &rgb)?;
        println!("{}", p.display());
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
        Some("pcm") => pcm(&args),
        Some("agent") => agent(&args),
        Some("brain") => brain(),
        Some("cockpit") => cockpit(&args),
        Some("war") => war(&args),
        Some("effects") => effects(&args),
        Some("clouds") => clouds(&args),
        Some("wrecks") => wrecks(&args),
        Some("carrier") => carrier_views(&args),
        _ => Err("usage: chopper bake [OUT] | frame [--reu R] [--tick N] [--doubled | --full] [--out O] | map [--out O] [--level K] | flight [--out DIR] [--from T] [--every N] [--frames K] | pcm [--out O] | agent [--reu R] [--at T] [--out O] [--trace N] | brain | cockpit [--out DIR] | effects [--out DIR] | war [--minutes M] [--knobs E,A,P,W,WP] [--quiet] [--fleet] | wrecks [--reu R] [--out DIR] [--v V] [--h H] [--pitch P] | carrier [--reu R] [--at T] [--back B] [--h H] [--pitch P]".into()),
    }
}
