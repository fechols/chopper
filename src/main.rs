//! Host tools: world generation, previews and LOD budget measurements.

use chopper::camera::{Camera, WorldPos};
use chopper::render::{PALETTE_RGB, Renderer};
use chopper::screen::Screen;
use chopper::terrain::{self, Governor, Lod, SIZE, TerrainData};
use chopper::worldgen::{self, World};
use chopper::{horizon, png};
use std::path::{Path, PathBuf};

const SKY: u8 = 14;
const SEA: u8 = 6;
const SEED: u32 = 1983;

fn usage() -> ! {
    eprintln!(
        "usage:\n  chopper map <out.png>\n  chopper view <out.png> <x> <agl> <z> <yaw°> <pitch°> <roll°>\n  chopper tour <out-dir>"
    );
    std::process::exit(2);
}

fn angle(degrees: f64) -> u16 {
    ((degrees / 360.0 * 65536.0).round() as i64).rem_euclid(65536) as u16
}

struct Frame {
    renderer: Renderer,
    stats: terrain::Stats,
}

fn render_frame(world: &World, camera: &Camera, lod: Lod) -> Frame {
    let mut renderer = Renderer::new();
    renderer.begin(SKY);
    horizon::draw_ground(&mut renderer, camera, SEA);
    let stats = terrain::render(world, camera, lod, &mut renderer);
    renderer.finish();
    Frame { renderer, stats }
}

fn save(frame: &Frame, path: &Path, caption: &str) {
    let mut screen = Screen::new();
    screen.draw_view(&frame.renderer, |_, _| {});
    let text = caption.to_ascii_uppercase();
    screen.draw_panel(|band, px| {
        px.fill(11);
        chopper::render::draw_text(band, px, 4, 140, text.as_bytes(), 1, 1);
    });
    screen.save_png(path).unwrap();
}

fn camera_at(world: &World, x: i32, agl: i32, z: i32, yaw: f64, pitch: f64, roll: f64) -> Camera {
    let ground = terrain::height_at(world, x, z);
    Camera::new(WorldPos { x, y: ground + agl, z }, angle(yaw), angle(pitch), angle(roll))
}

/// Run the governor to its steady state on one view.
fn settle(world: &World, camera: &Camera) -> (Frame, Lod) {
    let mut governor = Governor::new();
    let mut frame = render_frame(world, camera, governor.lod);
    for _ in 0..40 {
        governor.update(frame.renderer.triangle_count as u32);
        frame = render_frame(world, camera, governor.lod);
    }
    (frame, governor.lod)
}

fn report(name: &str, frame: &Frame, lod: Lod) {
    let s = frame.stats;
    println!(
        "{name:>12}: res {:5.2}  nodes {:4}  culled {:4}  leaves {:4}  terrain tris {:3}  commands {:3}  dropped {}  reads {}  violations {}",
        lod.resolution_q8 as f64 / 256.0,
        s.nodes,
        s.culled,
        s.leaves,
        s.triangles,
        frame.renderer.triangle_count,
        frame.renderer.dropped,
        s.reads,
        s.restriction_violations
    );
}

fn map(world: &World, path: &Path) {
    let scale = 2;
    let w = SIZE / scale;
    let mut rgb = Vec::with_capacity(w * w);
    for y in 0..w {
        for x in 0..w {
            let (i, j) = (x * scale, (w - 1 - y) * scale);
            let k = world.kind(i, j) as usize;
            let shade = if world.height(i, j) > world.height((i + 2).min(SIZE - 1), (j + 2).min(SIZE - 1)) { 0 } else { 1 };
            rgb.push(PALETTE_RGB[terrain::RAMPS[k][shade] as usize]);
        }
    }
    for pad in &world.pads {
        let (x, y) = (pad.i / scale, w - 1 - pad.j / scale);
        for d in 0..5 {
            rgb[y * w + x - 2 + d] = [255, 0, 0];
            rgb[(y - 2 + d) * w + x] = [255, 0, 0];
        }
    }
    png::save(path, w as u32, w as u32, &rgb).unwrap();
}

/// Records which samples a frame touches (the REU fetch footprint).
struct Tracing<'a> {
    world: &'a World,
    touched: std::cell::RefCell<std::collections::HashSet<(usize, usize)>>,
}

impl TerrainData for Tracing<'_> {
    fn height(&self, i: usize, j: usize) -> u8 {
        self.touched.borrow_mut().insert((i, j));
        self.world.height(i, j)
    }
    fn kind(&self, i: usize, j: usize) -> u8 {
        self.touched.borrow_mut().insert((i, j));
        self.world.kind(i, j)
    }
    fn d2(&self, i: usize, j: usize) -> u8 {
        self.touched.borrow_mut().insert((i, j));
        self.world.d2(i, j)
    }
    fn hmax(&self, i: usize, j: usize) -> u8 {
        self.touched.borrow_mut().insert((i, j));
        self.world.hmax(i, j)
    }
}

/// Fly a circuit at 12 fps and report the per-frame sample footprint: with
/// 4-byte interleaved records, each unique sample is one small REU DMA.
fn streaming(world: &World) {
    let tracing = Tracing { world, touched: Default::default() };
    let mut governor = Governor::new();
    let mut previous = std::collections::HashSet::new();
    let (mut total, mut new_total, mut worst, frames) = (0usize, 0usize, 0usize, 240);
    for f in 0..frames {
        // 50 m/s (200 units/s) at 12 fps around a 6 km circle, 150 m up.
        let t = f as f64 / 12.0;
        let a = t * 200.0 / 6000.0;
        let (x, z) = ((6000.0 * a.sin()) as i32, (-6000.0 * a.cos()) as i32 - 2000);
        let yaw = 90.0 + a.to_degrees();
        let camera = camera_at(world, x, 600, z, yaw, -6.0, 15.0);
        tracing.touched.borrow_mut().clear();
        let mut renderer = Renderer::new();
        renderer.begin(SKY);
        horizon::draw_ground(&mut renderer, &camera, SEA);
        terrain::render(&tracing, &camera, governor.lod, &mut renderer);
        governor.update(renderer.triangle_count as u32);
        let touched = tracing.touched.borrow().clone();
        let new = touched.difference(&previous).count();
        total += touched.len();
        new_total += new;
        worst = worst.max(touched.len());
        previous = touched;
    }
    println!(
        "streaming: {} unique samples/frame (worst {worst}), {} new/frame; per-sample DMA at ~5.5 us: {:.1} ms/frame",
        total / frames,
        new_total / frames,
        (total / frames) as f64 * 5.5 / 1000.0
    );
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let world = worldgen::generate(SEED);
    match args.first().map(String::as_str) {
        Some("map") if args.len() == 2 => map(&world, Path::new(&args[1])),
        Some("package") if args.len() == 2 => {
            // Code phases from the C64 build (scripts/build-target.sh), if present.
            let phases: Vec<Vec<u8>> = chopper::paging::NAMES
                .iter()
                .map(|n| std::fs::read(format!("dist/phases/{n}.bin")).unwrap_or_default())
                .collect();
            let image = chopper::package::build(&world, &phases);
            let sizes: Vec<String> = chopper::paging::NAMES.iter().zip(&phases).map(|(n, p)| format!("{n} {}", p.len())).collect();
            println!("phases: {}", sizes.join(", "));
            let path = Path::new(&args[1]);
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir).unwrap();
            }
            std::fs::write(path, &image).unwrap();
            println!("{}: {} bytes", path.display(), image.len());
        }
        Some("view") if args.len() == 8 => {
            let n = |i: usize| args[i].parse::<f64>().unwrap_or_else(|_| usage());
            let camera = camera_at(&world, n(2) as i32, n(3) as i32, n(4) as i32, n(5), n(6), n(7));
            let (frame, lod) = settle(&world, &camera);
            report("view", &frame, lod);
            save(&frame, Path::new(&args[1]), "view");
        }
        Some("tour") if args.len() == 2 => {
            let dir = PathBuf::from(&args[1]);
            let base = worldgen::pad_position(&world, &world.pads[0]);
            let views: [(&str, i32, i32, i32, f64, f64, f64); 8] = [
                ("base", base.x - 600, 24, base.z - 900, 20.0, -4.0, 0.0),
                ("lowlevel", base.x, 200, base.z + 2000, 0.0, -3.0, 0.0),
                ("climb", base.x, 1200, base.z + 3000, 10.0, -8.0, 0.0),
                ("bank", base.x + 3000, 600, base.z + 6000, 40.0, -6.0, 30.0),
                ("mountains", 0, 900, 4000, 45.0, -2.0, 0.0),
                ("valley", -6000, 300, 0, 80.0, -4.0, -10.0),
                ("coast", 0, 400, -26000, 0.0, -5.0, 0.0),
                ("high", 0, 3000, -9000, 0.0, -20.0, 0.0),
            ];
            for (name, x, agl, z, yaw, pitch, roll) in views {
                let camera = camera_at(&world, x, agl, z, yaw, pitch, roll);
                let (frame, lod) = settle(&world, &camera);
                report(name, &frame, lod);
                save(&frame, &dir.join(format!("{name}.png")), name);
            }
            streaming(&world);
        }
        _ => usage(),
    }
}
