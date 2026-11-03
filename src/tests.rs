use crate::floor::{self, COLS, FloorCam, MAX_SPANS, Rays, Span, VIEW_LINES};
use crate::layout::BUFFERS;
use crate::model::Machine;
use std::sync::OnceLock;

/// The scene block as the C64 fetches it.
fn scene(reu: &[u8]) -> &[u8] {
    let s = crate::format::SCENE as usize;
    let len = u16::from_le_bytes([reu[s], reu[s + 1]]) as usize;
    &reu[s..s + len]
}

/// Ticks of the flight the tests look at: the pad, the lift-off, and a long roam.
const FLIGHT: u32 = crate::agent::LIFT + 40_000;

/// The agent at every tick of the flight (the host reference), once.
fn flight() -> &'static [crate::agent::Agent] {
    static F: OnceLock<Vec<crate::agent::Agent>> = OnceLock::new();
    F.get_or_init(|| {
        let mut reu: &[u8] = package();
        let mut a = crate::agent::Agent::new();
        let mut out = vec![a];
        while a.t < FLIGHT {
            a.step(&mut reu);
            out.push(a);
        }
        out
    })
}

/// The autopilot's view at tick `t` of the flight.
fn flight_view(t: u32) -> crate::geom::View {
    crate::geom::View { cam: flight()[t as usize].camera().0, pop: 0 }
}

/// Sample points of the roam: `tick` of `LAP`, 4 ticks apart from 10 s after the lift-off.
const LAP: u16 = 4096;

fn lap_view(tick: u16) -> crate::geom::View {
    flight_view(lap_t(tick))
}

fn lap_t(tick: u16) -> u32 {
    crate::agent::LIFT + 600 + 4 * tick as u32
}

/// Autopilot frame `tick`'s objects drawn whole into buffer 0 of a model of the package.
fn render_objects(tick: u16) -> (Machine, crate::geom::Stats) {
    let reu = package();
    let mut m = Machine::new(reu);
    let mut work = Box::new(crate::geom::Work::new());
    let mut st = crate::geom::draw(&mut m, scene(reu), &lap_view(tick), &mut work, &mut Box::new(crate::geom::Verts::new()));
    let (tris, dropped) = crate::geom::raster_bands(&mut m, &work, BUFFERS[0].bitmap);
    st.tris = tris;
    st.dropped = st.dropped.saturating_add(dropped);
    (m, st)
}

/// A camera at map position (u, v) and height h (texels) with the given orientation (256ths of a turn
/// for yaw, degrees for pitch and roll).
fn camera_at(u: f64, v: f64, h: f64, yaw: u16, pitch: i32, roll: i32) -> FloorCam {
    let m = crate::camera::Matrix14::orientation(yaw, (pitch * 182) as u16, (roll * 182) as u16);
    FloorCam { u: (u * 256.0) as i32, v: (v * 256.0) as i32, h: (h * 256.0) as u16, r: m.m }
}

fn package() -> &'static Vec<u8> {
    static P: OnceLock<Vec<u8>> = OnceLock::new();
    P.get_or_init(|| crate::package::bake(&crate::world::generate(1983)))
}

/// The exact ground texel (level 0, f64) each byte of line `y` samples, or `None` for sky / haze.
fn exact_line(cam: &FloorCam, y: u16) -> Vec<Option<(f64, f64)>> {
    let r = |i: usize, j: usize| cam.r[i][j] as f64 / 16384.0;
    let h = cam.h as f64 / 256.0;
    (0..COLS as usize)
        .map(|b| {
            let d = [((4 * b + 2) as f64 - 80.0) / floor::FX as f64, (68.0 - (y as f64 + 0.5)) / floor::FY as f64, 1.0];
            let w: [f64; 3] = core::array::from_fn(|i| r(i, 0) * d[0] + r(i, 1) * d[1] + r(i, 2) * d[2]);
            if w[1] >= 0.0 {
                return None;
            }
            let t = h / -w[1];
            Some((cam.u as f64 / 256.0 + t * w[0], cam.v as f64 / 256.0 + t * w[2]))
        })
        .collect()
}

#[test]
fn every_ground_byte_samples_within_a_texel_of_the_exact_ray() {
    // Random cameras, including steep pitch and ±60° roll: each run's bytes, stepped as the kernel steps
    // them, stay within ¾ of a level texel of the exact ray / ground intersection: the runs' affine
    // error (~0.4, `floor::ERR`), the 8.8 step truncated over up to 32 bytes (~0.12) and the 16-bit
    // ground points (a few hundredths).
    let mut rng = Rng(0x2468_ace1);
    let deg = |d: i32| (d * 182) as u16;
    let mut checked = 0;
    for _ in 0..300 {
        let yaw = rng.next() as u16;
        let pitch = deg(rng.below(70) as i32 - 50);
        let roll = deg(rng.below(121) as i32 - 60);
        let m = crate::camera::Matrix14::orientation(yaw, pitch, roll);
        let cam = FloorCam {
            u: (300 << 8) + rng.below(1400 << 8) as i32,
            v: (300 << 8) + rng.below(1400 << 8) as i32,
            h: (2 * 256 + rng.below(120 * 256)) as u16,
            r: m.m,
        };
        let rays = Rays::new(&cam);
        let mut spans = [Span::default(); MAX_SPANS];
        for y in (0..VIEW_LINES).step_by(5) {
            let n = floor::make_line(&cam, &rays, &rays.line(y), floor::HAZE_BYTE, &mut spans);
            let exact = exact_line(&cam, y);
            let mut b = 0usize;
            for s in &spans[..n] {
                assert!(s.len.is_power_of_two() || s.fill().is_some(), "{s:?}");
                if s.fill().is_none() {
                    let (mut u, mut v) = (s.u, s.v);
                    let unit = (1 << s.level()) as f64;
                    for i in 0..s.len as usize {
                        let (eu, ev) = exact[b + i].expect("a ground run over sky");
                        let (gu, gv) = (u as f64 / 256.0 * unit, v as f64 / 256.0 * unit);
                        let err = (gu - eu).abs().max((gv - ev).abs()) / unit;
                        assert!(err <= 0.75, "camera {cam:?} line {y} byte {}: off by {err:.3} level-{} texels", b + i, s.level());
                        u += s.du as i32;
                        v += s.dv as i32;
                        checked += 1;
                    }
                }
                b += s.len as usize;
            }
            assert_eq!(b, COLS as usize);
        }
    }
    assert!(checked > 100_000, "{checked}");
}

/// `make_line` as it was before the thresholds: `w` evaluated at every candidate run end and every
/// sky or haze byte. The thresholds and the galloping must give the same runs.
fn searched_line(cam: &FloorCam, rays: &Rays, a: &[i32; 3], out: &mut [Span; MAX_SPANS]) -> usize {
    use floor::{HAZE_BYTE, SKY_BYTE, ground_offset, level_for, mul_small, put_fill};
    let w_at = |b: u8| -(a[1] + mul_small(rays.bb[1], b));
    let at = |b: u8| {
        let d = rays.bt(b);
        ground_offset(rays.hc, a[0] + d[0], a[2] + d[2], w_at(b))
    };
    let (mut n, mut b) = (0, 0u8);
    while b < COLS {
        let w0 = w_at(b);
        if w0 < rays.w_haze {
            put_fill(out, &mut n, 1, if w0 <= 0 { SKY_BYTE } else { HAZE_BYTE });
            b += 1;
            continue;
        }
        let mut k = 5;
        while (1u8 << k) > COLS - b {
            k -= 1;
        }
        while k > 0 {
            let w1 = w_at(b + (1 << k));
            if w1 >= rays.w_haze && floor::ERR * (rays.bb[1].abs() << (2 * k)) <= w0 + w1 {
                break;
            }
            k -= 1;
        }
        let len = 1u8 << k;
        if w_at(b + len) < rays.w_haze {
            put_fill(out, &mut n, 1, HAZE_BYTE);
            b += 1;
            continue;
        }
        let (p0, p1) = (at(b), at(b + len));
        let (du, dv) = ((p1.0 - p0.0) >> k, (p1.1 - p0.1) >> k);
        match level_for(du, dv) {
            Some(level) => {
                out[n] = Span {
                    len,
                    flags: level,
                    u: (cam.u + p0.0) >> level,
                    v: (cam.v + p0.1) >> level,
                    du: (du >> level) as i16,
                    dv: (dv >> level) as i16,
                };
                n += 1;
            }
            None => put_fill(out, &mut n, len, HAZE_BYTE),
        }
        b += len;
    }
    n
}

#[test]
fn thresholds_and_galloping_give_the_searched_runs() {
    let mut rng = Rng(0x1357_9bdf);
    let deg = |d: i32| (d * 182) as u16;
    for _ in 0..300 {
        let m = crate::camera::Matrix14::orientation(rng.next() as u16, deg(rng.below(90) as i32 - 60), deg(rng.below(181) as i32 - 90));
        let cam = FloorCam {
            u: rng.below(2048 << 8) as i32,
            v: rng.below(2048 << 8) as i32,
            h: (64 + rng.below(200 * 256)) as u16,
            r: m.m,
        };
        let rays = Rays::new(&cam);
        let (mut x, mut y) = ([Span::default(); MAX_SPANS], [Span::default(); MAX_SPANS]);
        for line in 0..VIEW_LINES {
            let a = rays.line(line);
            let n = floor::make_line(&cam, &rays, &a, floor::HAZE_BYTE, &mut x);
            let m = searched_line(&cam, &rays, &a, &mut y);
            assert_eq!(&x[..n], &y[..m], "camera {cam:?} line {line}");
        }
    }
}

#[test]
fn a_level_line_is_one_run() {
    // No roll: every line is at one depth, so its ground is one run (as in mode7).
    let m = crate::camera::Matrix14::orientation(0x1234, (-12 * 182) as u16, 0);
    let cam = FloorCam { u: 1000 << 8, v: 900 << 8, h: 20 * 256, r: m.m };
    let rays = Rays::new(&cam);
    let mut spans = [Span::default(); MAX_SPANS];
    for y in 0..VIEW_LINES {
        let n = floor::make_line(&cam, &rays, &rays.line(y), floor::HAZE_BYTE, &mut spans);
        let ground = spans[..n].iter().filter(|s| s.fill().is_none()).count();
        assert!(ground <= 2, "line {y}: {:?}", &spans[..n]);
    }
}

#[test]
fn tiles_map_to_their_banks_and_the_sea_beyond() {
    use crate::format::{LEVEL_BANK, SEA_BANK};
    assert_eq!(floor::tile_bank(0, 0, 0), LEVEL_BANK[0]);
    assert_eq!(floor::tile_bank(0, 7, 7), LEVEL_BANK[0] + 63);
    assert_eq!(floor::tile_bank(0, 8, 0), SEA_BANK);
    assert_eq!(floor::tile_bank(0, 0xFF, 3), SEA_BANK); // just west of the map
    assert_eq!(floor::tile_bank(3, 0, 0), LEVEL_BANK[3]);
    assert_eq!(floor::tile_bank(3, 1, 0), SEA_BANK);
    // A texel's address in the image holds that texel.
    let reu = package();
    let w = crate::world::generate(1983);
    for (u, v) in [(0usize, 0usize), (255, 256), (1024, 760), (2047, 2047), (1500, 33)] {
        let a = floor::texel_addr(0, (u as i32) << 8, (v as i32) << 8) as usize;
        assert_eq!(reu[a], crate::world::dither(w.texels[v * 2048 + u], u, v), "texel ({u}, {v})");
    }
}

#[test]
fn an_address_write_reloads_the_whole_reu_address() {
    // C64U behaviour (2026-11-01): after a 1-byte fetch the address auto-increments, but writing the mid
    // register reloads lo from the value last written to it, not the incremented one.
    use crate::hw::{Hw, reu};
    let mut image = vec![0u8; 0x300];
    for (i, b) in image.iter_mut().enumerate() {
        *b = i as u8 ^ (i >> 8) as u8 * 0x40;
    }
    let mut m = Machine::new(&image);
    for (r, v) in [(reu::C64_LO, 0x00), (reu::C64_HI, 0x20), (reu::LEN_LO, 1), (reu::LEN_HI, 0), (reu::REU_LO, 0x10), (reu::REU_MID, 0), (reu::REU_BANK, 0)] {
        m.poke(r, v);
    }
    m.reu_exec(reu::FETCH); // reads $000010, address now $000011
    m.poke(reu::REU_MID, 1); // reloads lo = $10, not $11
    m.reu_exec(reu::FETCH);
    assert_eq!(m.ram[0x2001], image[0x110]);
}

#[test]
fn every_frame_dma_stays_inside_a_512k_bank() {
    let reu = package();
    for tick in [0u16, 512, 1024, 1536, 3000] {
        for doubled in [false, true] {
            let (m, _) = crate::package::render(reu, lap_t(tick), doubled);
            assert_eq!((m.stats.bank_wraps, m.stats.stray_writes), (0, 0), "tick {tick} doubled {doubled}");
        }
    }
}

/// xorshift32, for repeatable random cases.
struct Rng(u32);

impl Rng {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }
    fn below(&mut self, n: u32) -> u32 {
        self.next() % n
    }
}

#[test]
fn the_cell_writer_paints_exactly_the_pixels_of_each_line() {
    use crate::poly::{PATTERNS, Row, fill_row, paint_row};
    let mut rng = Rng(0x1234_5678);
    let base = BUFFERS[0].bitmap + 7 * 320;
    for case in 0..4000 {
        let mut row = Row { pat: PATTERNS[rng.below(16) as usize], base, ..Row::default() };
        // Mostly convex-looking rows (shared bounds) so full cells happen; some random and empty lines.
        let (l0, r0) = (rng.below(161) as u8, rng.below(161) as u8);
        for k in 0..8 {
            let (a, b) = match rng.below(6) {
                0 => (rng.below(161) as u8, rng.below(161) as u8),
                1 if case % 3 == 0 => (0, 0),
                _ => (l0.saturating_sub(rng.below(6) as u8), r0.saturating_add(rng.below(6) as u8).min(160)),
            };
            row.xl[k] = a;
            row.xr[k] = b;
        }
        let mut a = Machine::new(&[]);
        for i in 0..320u16 {
            a.ram[(base + i) as usize] = rng.next() as u8;
        }
        let mut b = Machine::new(&[]);
        b.ram.copy_from_slice(&a.ram);
        fill_row(&mut a, &row);
        paint_row(&mut b, &row);
        assert_eq!(a.ram[base as usize..base as usize + 320], b.ram[base as usize..base as usize + 320], "case {case}: {row:?}");
    }
}

#[test]
fn triangles_cover_the_pixel_centres_inside_them() {
    use crate::geom::{Tri, triangle};
    let mut rng = Rng(0x9E37_79B9);
    let buf = BUFFERS[0];
    for case in 0..300 {
        let span = if case % 4 == 0 { 400 } else { 180 };
        let mut t = Tri { shade: 3, ..Tri::default() };
        for k in 0..3 {
            t.x[k] = (rng.below(span * 16) as i32 - (span as i32 - 160) * 8) as i16;
            t.y[k] = (rng.below(span * 16) as i32 - (span as i32 - 200) * 8) as i16;
        }
        let mut m = Machine::new(&[]);
        triangle(&mut m, buf.bitmap, &t);
        let codes = m.codes(buf);
        let p: Vec<(f64, f64)> = (0..3).map(|k| (t.x[k] as f64 / 16.0, t.y[k] as f64 / 16.0)).collect();
        let edge = |a: (f64, f64), b: (f64, f64), x: f64, y: f64| (b.0 - a.0) * (y - a.1) - (b.1 - a.1) * (x - a.0);
        // (The triangle pipeline draws the view's lines only, above the cockpit.)
        for y in 0..crate::geom::LINES as usize {
            for x in 0..160 {
                let (cx, cy) = (x as f64 + 0.5, y as f64 + 0.5);
                let e = [edge(p[0], p[1], cx, cy), edge(p[1], p[2], cx, cy), edge(p[2], p[0], cx, cy)];
                let lens = [(p[0], p[1]), (p[1], p[2]), (p[2], p[0])].map(|(a, b)| ((b.0 - a.0).hypot(b.1 - a.1)).max(1e-9));
                let inside = e.iter().all(|&v| v > 0.0) || e.iter().all(|&v| v < 0.0);
                // Edge slopes divide by an 11-bit reciprocal (`fixed`): x drifts up to |dx|/2048 pixels
                // (plus Q4 vertices), so allow a sliver near the edges, more for the huge cases.
                let tol = if span > 200 { 0.3 } else { 0.12 };
                let near = (0..3).any(|i| (e[i] / lens[i]).abs() < tol);
                let got = codes[y * 160 + x] == 3;
                assert!(got == inside || near, "case {case} {t:?}: pixel {x},{y} drawn {got}, inside {inside}");
            }
        }
    }
}

#[test]
fn a_face_toward_the_camera_is_drawn_and_away_is_culled() {
    use crate::geom::{View, Work, draw};
    use crate::scene::{Material, Mesh, Object, bake_with, install};
    // Camera at (128, 128), 4 texels up, looking north (+v); a wall 20 texels ahead, across the view.
    for (toward, want) in [(-1.0f32, 2u8), (1.0, 0)] {
        let mut mesh = Mesh::default();
        mesh.quad([[-4.0, 0.0, 0.0], [4.0, 0.0, 0.0], [4.0, 0.0, 8.0], [-4.0, 0.0, 8.0]], [0.0, toward, 0.0], Material::Wall);
        let baked = bake_with(&[mesh], &[Object { u: 128.0, v: 148.0, y: 0.0, yaw: 0, mesh: 0 }]);
        let mut image = vec![0u8; crate::format::IMAGE_END as usize];
        install(&mut image, &baked);
        let mut m = Machine::new(&image);
        let mut work = Box::new(Work::new());
        let view = View { cam: camera_at(128.0, 128.0, 4.0, 0, 0, 0), pop: 1 };
        draw(&mut m, &baked.block, &view, &mut work, &mut Box::new(crate::geom::Verts::new()));
        let (tris, _) = crate::geom::raster_bands(&mut m, &work, BUFFERS[0].bitmap);
        assert_eq!(tris, want, "normal along v {toward}");
    }
}

#[test]
fn the_geometry_frames_draw_inside_ram() {
    let mut total = 0u32;
    for tick in (0..LAP).step_by(256) {
        let (m, st) = render_objects(tick);
        assert_eq!((m.stats.stray_writes, st.dropped), (0, 0), "tick {tick}");
        total += st.tris as u32;
    }
    assert!(total > 100, "only {total} triangles over the lap");
}

#[test]
fn the_reciprocal_divide_stays_close_to_the_exact_one() {
    use crate::fixed::{norm, recip};
    for v in 1..=32000u16 {
        let (m, e) = norm(v);
        let approx = recip(m) as f64 / 2f64.powi(26 + e as i32);
        let rel = (approx * v as f64 - 1.0).abs();
        assert!(rel < 1.0 / 1024.0, "1/{v}: relative error {rel}");
    }
}

#[test]
fn a_ground_point_projects_onto_the_floor_byte_that_samples_it() {
    // Random cameras (pitch, roll): for floor bytes on the ground, the exact ground point each samples,
    // taken into camera space and projected, lands within a pixel of that byte's sample point.
    use crate::geom::{Look, View, project};
    let mut rng = Rng(0x5eed_1234);
    let mut checked = 0;
    for _ in 0..200 {
        let cam = camera_at(1024.0, 1024.0, 4.0 + rng.below(40) as f64, rng.next() as u16, rng.below(60) as i32 - 45, rng.below(121) as i32 - 60);
        let look = Look::new(&View { cam, pop: 0 });
        for y in (0..VIEW_LINES).step_by(7) {
            for (b, p) in exact_line(&cam, y).iter().enumerate() {
                let Some((u, v)) = *p else { continue };
                let d = [(u - 1024.0) * 16.0, -(cam.h as f64 / 16.0), (v - 1024.0) * 16.0];
                if d[0].abs() > 2000.0 || d[2].abs() > 2000.0 {
                    continue;
                }
                let c = look.to_camera(d.map(|x| x.round() as i16));
                if c[2] < crate::geom::NEAR {
                    continue;
                }
                let (sx, sy) = project(c[0] as i16, c[1] as i16, c[2] as i16);
                let (ex, ey) = ((4 * b + 2) as f64, y as f64 + 0.5);
                let (dx, dy) = (sx as f64 / 16.0 - ex, sy as f64 / 16.0 - ey);
                // Q4 positions and the truncated rotation move a near point by up to ~1/z of the focal
                // length; the reciprocal adds 1/1024 of the offset from the centre.
                let tol = 0.5 + 200.0 / c[2] as f64;
                assert!(dx.abs() < tol && dy.abs() < tol, "{cam:?} line {y} byte {b} (z {}): off by ({dx:.2}, {dy:.2})", c[2]);
                checked += 1;
            }
        }
    }
    assert!(checked > 10_000, "{checked}");
}

#[test]
fn the_grid_visits_every_object_the_cull_accepts() {
    // Brute force: every object of the package against the per-object cull, at many cameras; the grid
    // walk must accept exactly as many (a cell skipped wrongly would lose some).
    use crate::geom::{BIG_CELL, BIG_GRID, BIG_SHIFT, CELL, GRID, Look, Q, SHIFT, View, object_centre, pop_depth, sphere_visible};
    use crate::format::{BIG_DIR, OBJ_DIR, OBJ_RECORDS};
    let reu = package();
    let block = scene(reu);
    // Every cell of both grids: its centre (Q4), its records, the records' shift.
    let mut cells = Vec::new();
    for (dir, n, cell, sh) in [(OBJ_DIR, GRID, CELL * Q, SHIFT), (BIG_DIR, BIG_GRID, BIG_CELL * Q, BIG_SHIFT)] {
        for k in 0..n * n {
            let e = &reu[dir as usize + 4 * k..][..4];
            let (first, count) = (u16::from_le_bytes([e[0], e[1]]) as usize, e[2] as usize);
            let centre = ((k % n) as i32 * cell + cell / 2, (k / n) as i32 * cell + cell / 2);
            for r in 0..count {
                cells.push((centre, sh, &reu[OBJ_RECORDS as usize + 8 * (first + r)..][..8]));
            }
        }
    }
    let mut rng = Rng(0xC0FFEE);
    let mut total = 0u32;
    for case in 0..400 {
        let cam = if case % 2 == 0 {
            flight_view(lap_t(rng.below(LAP as u32) as u16)).cam
        } else {
            camera_at(300.0 + rng.below(1450) as f64, 300.0 + rng.below(1450) as f64, 2.0 + rng.below(60) as f64, rng.next() as u16, rng.below(70) as i32 - 50, rng.below(121) as i32 - 60)
        };
        let pop = [0u8, 1, 2, 4, 8][rng.below(5) as usize];
        let view = View { cam, pop };
        let look = Look::new(&view);
        let mut want = 0u32;
        for &((cu, cv), sh, rec) in &cells {
            // Only near enough for the offsets to fit i16 (the walk's reach is well inside this).
            if (cu - (cam.u >> 4)).abs() > 20000 || (cv - (cam.v >> 4)).abs() > 20000 {
                continue;
            }
            let rad = block[8 + rec[crate::geom::scene::O_MESH] as usize * 10 + 7];
            let c = object_centre(&look, look.cell_sums(cu, cv), sh, rec);
            if sphere_visible(c, (rad as i32) << 4, pop_depth(rad, look.scale)) {
                want += 1;
            }
        }
        let mut m = Machine::new(reu);
        let mut work = Box::new(crate::geom::Work::new());
        let st = crate::geom::draw(&mut m, block, &view, &mut work, &mut Box::new(crate::geom::Verts::new()));
        assert_eq!(st.objects as u32, want.min(255), "case {case}: {cam:?} pop {pop}");
        total += want;
    }
    assert!(total > 2000, "only {total} objects accepted");
}

#[test]
fn band_rendering_equals_whole_triangle_painting() {
    // Per pixel, drawing band by band (each triangle one row per band, in painter's order) must equal
    // painting whole triangles back to front, on real autopilot frames.
    use crate::geom::{Verts, Work, draw, raster, raster_bands};
    let reu = package();
    let buf = BUFFERS[0];
    let mut checked = 0;
    for tick in (0..LAP).step_by(128) {
        let mut a = Machine::new(reu);
        let mut work = Box::new(Work::new());
        draw(&mut a, scene(reu), &lap_view(tick), &mut work, &mut Box::new(Verts::new()));
        let mut b = Machine::new(&[]);
        b.ram.copy_from_slice(&a.ram);
        let whole = raster(&mut a, &work, buf.bitmap);
        let (banded, dropped) = raster_bands(&mut b, &work, buf.bitmap);
        assert_eq!(dropped, 0, "tick {tick}");
        assert_eq!(whole, banded, "tick {tick}");
        assert_eq!(a.ram[buf.bitmap as usize..][..8000], b.ram[buf.bitmap as usize..][..8000], "tick {tick}");
        checked += whole as u32;
    }
    assert!(checked > 200, "only {checked} triangles");
}

#[test]
fn a_full_pool_drops_the_later_triangles() {
    // 120 overlapping triangles all starting in band 3: the first POOL in painter's order are drawn.
    use crate::geom::{POOL, Work, push, raster_bands};
    let mut w = Box::new(Work::new());
    w.head.fill(crate::geom::NONE);
    for k in 0..120i16 {
        push(&mut w, [10 * 16, (60 + (k % 50)) * 16, 10 * 16], [30 * 16, 40 * 16, 90 * 16], (k % 4) as u8, 200 - k as u8);
    }
    let mut m = Machine::new(&[]);
    let (tris, dropped) = raster_bands(&mut m, &w, BUFFERS[0].bitmap);
    assert_eq!((tris as usize, dropped as usize), (POOL, 120 - POOL));
}

#[test]
fn a_far_object_through_its_affine_map_lands_where_its_projection_does() {
    // Every mesh, random camera orientations, random centres in view beyond the switch distance: each
    // vertex through `affine` lands within 2 pixels of `project(xform(..))` (about 1.4 but in the corners
    // of the view), and every row sum fits in i16 with room.
    use crate::geom::{AFFINE_SHIFT, FAR, Obj, Verts, affine, project, scene as sc, vert, xform, xform_affine};
    let s = scene(package());
    let mut rng = Rng(0xaff1_2e00);
    let (mut checked, mut worst, mut big) = (0, 0f64, 0);
    for _ in 0..4000 {
        let k = rng.below(s[sc::N_MESHES] as u32) as usize;
        let mr = sc::MESHES + k * sc::MESH;
        let rad = s[mr + 7] as i32;
        let lo = rad << AFFINE_SHIFT;
        if lo >= FAR {
            continue;
        }
        let z = lo + 1 + rng.below((FAR - lo) as u32) as i32;
        let x = (rng.below(2 * z as u32 + 1) as i32 - z) * 23 / 20; // the cull lets |x| reach about 1.19z
        let y = (rng.below(z as u32 + 1) as i32 - z / 2) * 6 / 5; // and |y| 0.65z
        let r = crate::camera::Matrix14::orientation(rng.next() as u16, rng.next() as u16, rng.next() as u16).m;
        let o = Obj {
            verts: u16::from_le_bytes([s[mr], s[mr + 1]]),
            nv: s[mr + 2],
            faces: u16::from_le_bytes([s[mr + 3], s[mr + 4]]),
            nf: s[mr + 5],
            t: 2 + s[mr + 6],
            org: [x as i16, y as i16, z as i16],
            m: r.map(|row| row.map(|v| v as i16)),
        };
        let a = affine(&o);
        // the sums stay within i16 (`affine`'s bound), with room
        for i in 0..o.nv as usize {
            let p = o.verts as usize + i * sc::VERTEX;
            let c = [s[p] as i8 as i32, s[p + 2] as i8 as i32, s[p + 1] as i8 as i32];
            for row in &a.g {
                let sum = row[0] as i32 * c[0] + row[1] as i32 * c[1] + row[2] as i32 * c[2];
                big = big.max(sum.abs());
            }
        }
        let (mut full, mut far) = (Verts::new(), Verts::new());
        xform(s, &o, &mut full);
        xform_affine(s, &o, &a, &mut far);
        for i in 0..o.nv as usize {
            let (px, py) = project(full.get(vert::X, i), full.get(vert::Y, i), full.get(vert::Z, i));
            let dx = (far.get(vert::SX, i) - px) as f64 / 16.0;
            let dy = (far.get(vert::SY, i) - py) as f64 / 16.0;
            let err = dx.abs().max(dy.abs());
            worst = worst.max(err);
            assert!(err < 2.0, "mesh {k} at {:?}: vertex {i} off by ({dx:.2}, {dy:.2})", o.org);
            checked += 1;
        }
    }
    println!("{checked} vertices, worst {worst:.2} px, largest sum {big}");
    assert!(checked > 5000 && big < 24000, "{checked} vertices checked, largest sum {big}");
}

#[test]
fn the_rotor_sound_decodes_and_its_loop_seam_is_smooth() {
    use crate::digital::{Dlt5, ROTOR, ROTOR_LOOP_END, ROTOR_LOOP_START, decode};
    let d = Dlt5::parse(ROTOR).unwrap();
    assert_eq!((d.rate, d.frames), (16000, ROTOR_LOOP_END));
    let pcm = decode(ROTOR);
    let (a, b) = (ROTOR_LOOP_START as usize, ROTOR_LOOP_END as usize);
    // The sampler jumps from frame b - 1 to frame a: no bigger a step than the loop's own 99th percentile.
    let mut steps: Vec<i32> = pcm[a..b].windows(2).map(|w| (w[1] as i32 - w[0] as i32).abs()).collect();
    steps.sort_unstable();
    let seam = (pcm[a] as i32 - pcm[b - 1] as i32).abs();
    assert!(seam <= steps[steps.len() * 99 / 100], "seam step {seam}");
    // Full power is loud and doesn't clip.
    let rms = (pcm[a..b].iter().map(|&s| (s as f64).powi(2)).sum::<f64>() / (b - a) as f64).sqrt();
    assert!((10.0..60.0).contains(&rms), "loop level {rms}");
    // The encoder's own decode (tools/make-rotor.py's target/rotor/rotor-dlt5.wav), when there.
    if let Ok(wav) = std::fs::read("target/rotor/rotor-dlt5.wav") {
        let data = &wav[44..];
        assert_eq!(data.len() / 2, pcm.len());
        for (i, (c, &s)) in data.chunks(2).zip(&pcm).enumerate() {
            assert_eq!(i16::from_le_bytes([c[0], c[1]]), (s as i16) << 8, "frame {i}");
        }
    }
}

#[test]
fn the_ground_and_lift_off_frames_draw_inside_ram() {
    use crate::agent::LIFT;
    for t in [0, LIFT - 1, LIFT + 60, LIFT + 200, LIFT + 400] {
        let reu = package();
        let mut m = Machine::new(reu);
        let mut work = Box::new(crate::geom::Work::new());
        let st = crate::geom::draw(&mut m, scene(reu), &flight_view(t), &mut work, &mut Box::new(crate::geom::Verts::new()));
        let (_, dropped) = crate::geom::raster_bands(&mut m, &work, BUFFERS[0].bitmap);
        let (_, fs) = crate::package::render(reu, t, false);
        assert_eq!((m.stats.stray_writes, st.dropped, dropped), (0, 0, 0), "frame {t}");
        println!("frame {t}: {} objects, {} runs", st.objects, fs.spans);
    }
}

#[test]
fn the_brain_tree_replays_the_plans() {
    // Every feature block built from each threshold's two sides (and the extremes) walks the baked tree
    // to the planner's action for its symbol state.
    use crate::goap::{SYMBOLS, state_of, table};
    let table = table();
    let blob = crate::goap::compile();
    let mut values: Vec<Vec<u8>> = vec![vec![0, 255]; crate::agent::feature::COUNT];
    for sy in SYMBOLS {
        values[sy.feature as usize].extend([sy.threshold - 1, sy.threshold]);
    }
    for v in &mut values {
        v.sort();
        v.dedup();
    }
    let mut image = vec![0u8; blob.len()];
    image.copy_from_slice(&blob);
    let mut reu: &[u8] = &image;
    let mut idx = vec![0usize; values.len()];
    let mut n = 0;
    'all: loop {
        let f: Vec<u8> = idx.iter().zip(&values).map(|(&i, v)| v[i]).collect();
        let got = crate::brain::walk(&mut reu, 0, crate::brain::PILOT, &f);
        assert_eq!(got, table[state_of(&f) as usize], "features {f:?}");
        n += 1;
        for k in 0..idx.len() {
            idx[k] += 1;
            if idx[k] < values[k].len() {
                continue 'all;
            }
            idx[k] = 0;
        }
        break;
    }
    let tree = crate::goap::pilot_tree(&table);
    assert!(tree.depth() < crate::brain::MAX_DEPTH as usize);
    println!("{n} feature blocks; tree {} nodes, depth {}, blob {} bytes", tree.nodes(), tree.depth(), blob.len());
}

#[test]
fn the_planner_lifts_off_and_climbs_or_steers() {
    use crate::goap::{OPS, plan, s};
    let first = |st: u16| OPS[plan(st).unwrap()[0]].name;
    assert_eq!(first(s::GROUNDED), "wait");
    assert_eq!(first(s::GROUNDED | s::ROTOR), "lift");
    assert_eq!(first(s::LOW), "climb-up");
    assert_eq!(first(s::BLOCKED | s::BLOCKED_L), "evade-r");
    assert_eq!(first(s::BLOCKED | s::BLOCKED_L | s::BLOCKED_R), "climb-over");
    assert_eq!(first(s::WP_L), "turn-l");
    assert_eq!(first(s::HIGH), "descend");
    // Every state has a plan.
    assert!((0..1u32 << crate::goap::N).all(|st| crate::goap::goal(st as u16) || plan(st as u16).is_some()));
}

#[test]
fn the_flight_waits_for_the_rotor_then_roams_clear_of_the_mountains() {
    use crate::agent::{EYE, LIFT};
    let hills = crate::goap::mountains(&crate::world::generate(1983));
    assert!(!hills.is_empty());
    assert_eq!(crate::agent::HOME, (crate::world::PADS[0].u as i32, crate::world::PADS[0].v as i32));
    let f = flight();
    let mut closest = f64::MAX;
    let (mut waypoints, mut last) = (0, (0, 0));
    let mut actions = [0u32; crate::agent::action::COUNT as usize];
    for a in f {
        let (u, v, h) = (a.u as f64 / 256.0, a.v as f64 / 256.0, a.h as f64 / 256.0);
        if a.t <= LIFT {
            assert_eq!((a.u, a.v, a.h), (crate::agent::HOME.0 << 8, crate::agent::HOME.1 << 8, EYE), "tick {}: off the pad", a.t);
            continue;
        }
        assert!((0.0..2048.0).contains(&u) && (0.0..2048.0).contains(&v), "tick {}: off the map at ({u:.0}, {v:.0})", a.t);
        // Above every mountain's cone (its peak may sit up to 0.075 r off the centre).
        for &(mu, mv, r, top) in &hills {
            let d = ((u - mu).hypot(v - mv) - 0.08 * r).max(0.0);
            if d < r {
                let surface = top * (1.0 - d / r);
                closest = closest.min(h - surface);
                assert!(h > surface + 2.0, "tick {}: {h:.1} texels up, {:.1} over the mountain at ({mu:.0}, {mv:.0})", a.t, h - surface);
            }
        }
        actions[a.action as usize] += 1;
        if (a.wu, a.wv) != last {
            waypoints += 1;
            last = (a.wu, a.wv);
        }
    }
    assert!(f[LIFT as usize + 1].h > EYE, "no lift-off at the rotor's full power");
    println!("{waypoints} waypoints; closest {closest:.1} texels over a mountain; actions {actions:?}");
    assert!(waypoints >= 10, "only {waypoints} waypoints in {} ticks", FLIGHT - LIFT);
}

#[test]
fn the_flight_banks_smoothly() {
    // The flight model's bank: never faster than its rate limit, never past 30°, and seldom reversed.
    let f = &flight()[crate::agent::LIFT as usize..];
    let deg = |x: i16| x as f64 * 360.0 / 65536.0;
    let max_rate = f.windows(2).map(|w| (w[1].roll - w[0].roll).abs()).max().unwrap();
    let peak = f.iter().map(|a| a.roll.abs()).max().unwrap();
    // A reversal: from one side past 5° to the other past 5°.
    let mut side = 0i16;
    let mut reversals = 0;
    for a in f {
        let s = if a.roll > 5 * 182 { 1 } else if a.roll < -5 * 182 { -1 } else { 0 };
        if s != 0 && s == -side {
            reversals += 1;
        }
        if s != 0 {
            side = s;
        }
    }
    let minutes = f.len() as f64 / 3600.0;
    println!(
        "bank: peak {:.1}°, fastest {:.1}°/s; {:.1} reversals a minute",
        deg(peak),
        deg(max_rate) * 60.0,
        reversals as f64 / minutes
    );
    assert!(deg(max_rate) * 60.0 < 25.0 && deg(peak) <= 30.5);
}
