//! `chopper compare [--view roto|floor|both] [--angle DEG] [--zoom Z] [--height H] [--x X] [--y Y] [--out DIR]`
//!
//! Renders a view with every path on the C64 model, prints the modeled C64U cost, and writes
//! `<out>/<view>_<path>.png` plus a 2×2 sheet `<out>/<view>_sheet.png` (reference, dma, dma-pre, cpu4).

use chopper::camera::{self, Frame, View};
use chopper::cost::{Cost, FRAME_US};
use chopper::layout::BUFFERS;
use chopper::model::Machine;
use chopper::palette::PALETTE;
use chopper::{map, mode7 as m7, png};
use std::path::PathBuf;

const COLORS: [u8; 4] = [5, 12, 1, 2]; // grass, tarmac, paint, kerbs
const SKY: u8 = 14;

#[derive(Clone, Copy)]
enum Path {
    Dma,
    DmaPre,
    Cpu(u8),
}

impl Path {
    fn name(self) -> String {
        match self {
            Path::Dma => "dma".into(),
            Path::DmaPre => "dma-pre".into(),
            Path::Cpu(n) => format!("cpu{n}"),
        }
    }
}

fn render(path: Path, f: &Frame, image: &[u8]) -> (Vec<[u8; 3]>, Cost, Option<Cost>) {
    let mut m = Machine::new(image);
    let buf = BUFFERS[0];
    m7::setup_colors(&mut m, buf, COLORS, SKY, f.horizon_row);
    let (cost, fill) = match path {
        Path::Dma => (m7::per_byte_dma(&mut m, buf, &f.lines, m7::DmaMode::RAW), None),
        Path::DmaPre => (m7::per_byte_dma(&mut m, buf, &f.lines, m7::DmaMode::PRE), None),
        Path::Cpu(n) => {
            let fill = m7::fill_cache(&mut m, f.window.0, f.window.1);
            (m7::cpu_gather(&mut m, buf, &f.lines, n), Some(fill))
        }
    };
    assert_eq!(m.stats.stray_writes, 0);
    (m.decode(buf, COLORS[0]), cost, fill)
}

fn colorize(codes: &[u8], horizon_row: u16) -> Vec<[u8; 3]> {
    codes
        .iter()
        .enumerate()
        .map(|(i, &c)| {
            let row = (i / 160 / 8) as u16;
            let col = if c == 3 && row < horizon_row { SKY } else { COLORS[c as usize] };
            PALETTE[col as usize]
        })
        .collect()
}

fn sheet(panels: &[&[[u8; 3]]]) -> (u32, u32, Vec<[u8; 3]>) {
    const GAP: usize = 4;
    let (pw, ph) = (320, 200);
    let (w, h) = (2 * pw + GAP, 2 * ph + GAP);
    let mut out = vec![[32u8, 32, 32]; w * h];
    for (i, p) in panels.iter().enumerate() {
        let (ox, oy) = ((i % 2) * (pw + GAP), (i / 2) * (ph + GAP));
        for y in 0..ph {
            for x in 0..pw {
                out[(oy + y) * w + ox + x] = p[y * 160 + x / 2];
            }
        }
    }
    (w as u32, h as u32, out)
}

fn arg(args: &[String], name: &str) -> Option<String> {
    args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned()
}

fn num(args: &[String], name: &str, default: f32) -> f32 {
    arg(args, name).map_or(default, |s| s.parse().unwrap_or_else(|_| panic!("{name}: not a number: {s}")))
}

fn bake(args: &[String]) -> Result<(), String> {
    let out = PathBuf::from(args.get(1).cloned().unwrap_or("dist/chopper.reu".into()));
    let image = chopper::package::bake(&map::generate());
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
    let (m, cost, geo) = chopper::package::render(&reu, tick, doubled);
    let buf = BUFFERS[0];
    png::save_wide(&out.with_extension("png"), 160, &m.decode(buf, COLORS[0]))?;
    let bitmap = &m.ram[buf.bitmap as usize..buf.bitmap as usize + 8000];
    std::fs::write(out.with_extension("bin"), bitmap).map_err(|e| e.to_string())?;
    println!(
        "tick {tick} {}: {} DMAs, {} I/O writes, modeled {:.2} ms; {} objects, {} triangles ({} dropped)",
        if doubled { "doubled" } else { "full" },
        cost.dma_cmds,
        cost.io_writes,
        cost.us() / 1000.0,
        geo.objects,
        geo.tris,
        geo.dropped
    );
    Ok(())
}

/// `mode7 walks [--reu R] [--tick N]`: every set-up triangle of a frame, as the C64 passes them to
/// `asm/tri.S` (debugging the C64 against the host).
fn walks(args: &[String]) -> Result<(), String> {
    use chopper::geom::{self, Walk};
    use chopper::hw::Hw;
    struct Rec(Machine, Vec<Walk>);
    impl Hw for Rec {
        fn peek(&mut self, a: u16) -> u8 {
            self.0.peek(a)
        }
        fn poke(&mut self, a: u16, v: u8) {
            self.0.poke(a, v)
        }
        fn reu_exec(&mut self, c: u8) {
            self.0.reu_exec(c)
        }
    }
    let reu = std::fs::read(arg(args, "--reu").unwrap_or("dist/chopper.reu".into())).map_err(|e| e.to_string())?;
    let tick = num(args, "--tick", 0.0) as u16;
    let mut r = Rec(Machine::new(&reu), Vec::new());
    let cam = chopper::package::camera(&reu, tick);
    let mut work = Box::new(geom::Work::new());
    geom::draw(&mut r, chopper::package::scene(&reu), cam, BUFFERS[0].bitmap, &mut work, &mut Box::new(geom::Verts::new()));
    // Each triangle as set up (before its walk).
    let (order, n, _, _) = geom::bin(&work);
    for &i in &order[..n as usize] {
        if let Some(w) = geom::setup(BUFFERS[0].bitmap, &work.tri(i)) {
            r.1.push(w);
        }
    }
    for w in &r.1 {
        println!("{} {} {} {} {} {} {} {} {} {}", w.long.x, w.long.step, w.short.x, w.short.step, w.lower.x, w.lower.step, w.first, w.mid, w.end, w.pat[0]);
    }
    Ok(())
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("bake") => return bake(&args),
        Some("frame") => return frame(&args),
        Some("walks") => return walks(&args),
        Some("compare") => {}
        _ => return Err("usage: chopper bake [OUT] | frame [--reu R] [--tick N] [--doubled] [--out O] | compare [...]".into()),
    }
    let out = PathBuf::from(arg(&args, "--out").unwrap_or("output".into()));
    let which = arg(&args, "--view").unwrap_or("both".into());
    let angle = num(&args, "--angle", 30.0).to_radians();
    let (x, y) = (num(&args, "--x", 220.0), num(&args, "--y", 128.0));
    let mut views = Vec::new();
    if which == "roto" || which == "both" {
        views.push(("roto", View::Roto { x, y, angle, zoom: num(&args, "--zoom", 0.25) }));
    }
    if which == "floor" || which == "both" {
        views.push(("floor", View::Floor { x, y, angle, height: num(&args, "--height", 12.0), horizon: 48, focal: 160.0 }));
    }
    if views.is_empty() {
        return Err(format!("unknown view {which}"));
    }

    eprintln!("baking the REU image (raw plane + 240 pre-sampled planes)...");
    let codes = map::generate();
    let image = map::bake(&codes);

    for (name, view) in views {
        let f = camera::frame(&view);
        let reference = colorize(&camera::reference(&f, &codes), f.horizon_row);
        png::save_wide(&out.join(format!("{name}_reference.png")), 160, &reference)?;
        println!("\n{name}: {view:?}, cache window {:?}", f.window);
        println!("{:<8} {:>9} {:>9} {:>7} {:>10} {:>9} {:>7}", "path", "texels/ln", "I/O wr", "DMAs", "cycles", "ms", "fps");
        let mut panels = vec![reference];
        for path in [Path::Dma, Path::DmaPre, Path::Cpu(1), Path::Cpu(2), Path::Cpu(4)] {
            let (rgb, cost, fill) = render(path, &f, &image);
            let across = match path {
                Path::Dma | Path::Cpu(1) => 40,
                Path::Cpu(2) => 80,
                _ => 160,
            };
            let ms = cost.us() / 1000.0;
            let fps = (1000.0 / ms).min(1_000_000.0 / FRAME_US);
            println!("{:<8} {:>9} {:>9} {:>7} {:>10} {:>9.2} {:>7.1}", path.name(), across, cost.io_writes, cost.dma_cmds, cost.cycles, ms, fps);
            if let (Path::Cpu(4), Some(fill)) = (path, fill) {
                println!("  cache fill (whole 128x128 window, once per recentre): {} DMAs, {:.2} ms", fill.dma_cmds, fill.us() / 1000.0);
            }
            png::save_wide(&out.join(format!("{name}_{}.png", path.name())), 160, &rgb)?;
            if matches!(path, Path::Dma | Path::DmaPre | Path::Cpu(4)) {
                panels.push(rgb);
            }
        }
        let refs: Vec<&[[u8; 3]]> = panels.iter().map(|p| p.as_slice()).collect();
        let (w, h, px) = sheet(&refs);
        png::save(&out.join(format!("{name}_sheet.png")), w, h, &px)?;
    }
    println!("\nPNGs in {}", out.display());
    Ok(())
}
