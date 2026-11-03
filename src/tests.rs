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

#[test]
fn the_gauges_read_the_flight() {
    use crate::cockpit::{FRAMES, TACH_FRAMES, gauge::*, lamp, readings};
    let mut seen = vec![[false; 256]; COUNT];
    let mut tach = 0;
    for a in flight() {
        let mut r = [0u8; COUNT];
        readings(a, &mut r);
        for g in 0..COUNT {
            assert!((r[g] as u16) < FRAMES[g], "tick {}: {} reads frame {} of {}", a.t, NAMES[g], r[g], FRAMES[g]);
            seen[g][r[g] as usize] = true;
        }
        if a.t < crate::agent::LIFT {
            // On the pad: still, level, the rotor lamp on, the tach winding up with the sound.
            assert_eq!((r[ASI], r[VSI], r[ATT], r[ALT]), (0, 20, 12 * crate::cockpit::ATT_PITCHES as u8 + 8, 0), "tick {}", a.t);
            assert_eq!(r[WARN], lamp::RPM, "tick {}", a.t);
            assert!(r[RPM] >= tach && r[RPM] < TACH_FRAMES as u8 - 1);
            tach = r[RPM];
        } else {
            assert_eq!(r[RPM], TACH_FRAMES as u8 - 1, "tick {}: the rotor is at 100% from the lift", a.t);
            assert_eq!(r[WARN] & lamp::RPM, 0);
        }
    }
    for g in [ASI, ATT, ALT, HDG, VSI] {
        let n = seen[g].iter().filter(|&&s| s).count();
        assert!(n >= 8, "{} shows only {n} readings in the flight", NAMES[g]);
    }
}

#[test]
fn the_tach_follows_the_rotor_sound() {
    use crate::instruments::{BEAT_FROM, blade_beat, spool};
    let pcm = crate::digital::decode(crate::digital::ROTOR);
    let full = blade_beat(&pcm, crate::digital::ROTOR_LOOP_START as f32);
    assert!((9.5..11.5).contains(&full), "governed blade beat {full} Hz");
    let early = blade_beat(&pcm, BEAT_FROM * 16_000.0);
    assert!((0.4..0.7).contains(&(early / full)), "blade beat at {BEAT_FROM} s: {early} Hz of {full}");
    let s = spool();
    assert_eq!(s.len(), crate::cockpit::TACH_FRAMES as usize);
    assert_eq!((s[0], *s.last().unwrap()), (0.0, 100.0));
    assert!(s.windows(2).all(|w| w[0] <= w[1]), "the rotor only speeds up: {s:?}");
}

#[test]
fn the_gauge_frames_are_baked_and_move() {
    use crate::cockpit::{BASE, END, FRAMES, FRAME_STRIDE_SHIFT, TALL, WIDE, gauge::*};
    assert!(END <= crate::format::IMAGE_END);
    let reu = package();
    let frame = |g: usize, k: u16| {
        let at = BASE[g] as usize + ((k as usize) << FRAME_STRIDE_SHIFT);
        &reu[at..at + WIDE[g] as usize * 8 * TALL as usize]
    };
    for g in [ASI, ALT, WARN, HDG, VSI] {
        for k in 1..FRAMES[g] {
            assert_ne!(frame(g, k - 1), frame(g, k), "{} frames {} and {} look the same", NAMES[g], k - 1, k);
        }
    }
    for g in 0..COUNT {
        assert!(frame(g, FRAMES[g] - 1).iter().any(|&b| b != 0), "{} not baked", NAMES[g]);
    }
}

/// The view's horizon drops toward the side the helicopter banks to, and so does the attitude gauge's.
#[test]
fn the_attitude_gauge_banks_with_the_view() {
    use crate::cockpit::{ATT_PITCHES, gauge, readings};
    let reu = package();
    // The reference flight `package::render` draws (from the carrier's deck), every tick.
    let mut f = vec![crate::agent::Agent::boot()];
    while f.len() < FLIGHT as usize {
        let mut a = *f.last().unwrap();
        let t = a.t + 1;
        a.fly(&mut &reu[..], t);
        f.push(a);
    }
    for want_right in [true, false] {
        let a = f.iter().filter(|a| a.grounded == 0).max_by_key(|a| if want_right { a.roll } else { -a.roll }).unwrap();
        let (m, _) = crate::package::render(reu, a.t, false);
        // First line below the sky in a left and a right byte column.
        let horizon = |col: u16| (0..VIEW_LINES).find(|&y| m.ram[(BUFFERS[0].bitmap + crate::layout::line_offset(y) + col * 8) as usize] != floor::SKY_BYTE).unwrap();
        let (l, r) = (horizon(4), horizon(35));
        let mut k = [0u8; gauge::COUNT];
        readings(a, &mut k);
        let (tilt, _) = crate::instruments::attitude(k[gauge::ATT] as u16);
        assert!(k[gauge::ATT] as u16 / ATT_PITCHES != 12, "tick {}: roll {} reads level", a.t, a.roll);
        // Tilt clockwise: the right side lower (larger y).
        assert_eq!(r > l, tilt > 0.0, "tick {}: roll {}: view horizon left {l} right {r}, gauge tilt {tilt}", a.t, a.roll);
    }
}

/// The packed HUD rows keep the multiplexer's timing (TALON HD's): at most 8 sprites a row, each row's
/// IRQ at least 2 lines after the last, a channel reprogrammed only once its sprite has ended (`SPAN`)
/// and its new sprite starting `LEAD` lines after the IRQ, every sprite inside the view.
#[test]
fn the_hud_rows_keep_the_multiplexer_timing() {
    use crate::hud::{LEAD, MAX, ROW, ROWS, SPAN, Sprite, TABLE, TOP_LINE, Y_MAX, pack, row};
    let mut rng = Rng(0x48_5544);
    for _ in 0..2000 {
        let n = 1 + rng.below(MAX as u32) as usize;
        let mut list: Vec<Sprite> = (0..n).map(|k| Sprite { x: rng.below(344) as u16, y: 30 + rng.below(140) as u8, ptr: k as u8 }).collect();
        let mut table = [0u8; TABLE];
        let shown = pack(&mut list, &mut table) as usize;
        assert!(shown <= n);
        let rows: Vec<&[u8]> = table.chunks(ROW).take_while(|r| r[row::LINE] != 0).collect();
        assert!(!rows.is_empty() && rows.len() <= ROWS && rows[0][row::LINE] == TOP_LINE);
        let mut seen = std::collections::HashSet::new();
        for (r, t) in rows.iter().enumerate() {
            let line = t[row::LINE];
            for c in 0..8 {
                if t[row::EN] >> c & 1 == 0 {
                    continue;
                }
                let (y, ptr) = (t[row::Y + c], t[row::PTR + c]);
                assert!(y <= Y_MAX, "row {r} channel {c}: Y {y}");
                let new = r == 0 || rows[r - 1][row::EN] >> c & 1 == 0 || rows[r - 1][row::Y + c] != y || rows[r - 1][row::PTR + c] != ptr;
                if new {
                    assert!(y >= line + LEAD, "row {r} (IRQ {line}) channel {c}: Y {y}");
                    if r > 0 && rows[r - 1][row::EN] >> c & 1 != 0 {
                        assert!(rows[r - 1][row::Y + c] + SPAN <= line, "row {r}: channel {c} reprogrammed while its sprite shows");
                    }
                    seen.insert(ptr);
                }
            }
            if r > 0 {
                assert!(line >= rows[r - 1][row::LINE] + 2);
            }
        }
        assert_eq!(seen.len(), shown, "each shown sprite in one row");
    }
}

/// The HUD's projection (16-bit, quarter texels) lands within a pixel or so of the 3D view's
/// (`geom::project`).
#[test]
fn the_hud_projects_where_the_view_draws() {
    use crate::geom::{Look, View, project};
    use crate::hud::{Eye, NEAR, screen, to_camera};
    let mut rng = Rng(0x0e1e_0001);
    let mut checked = 0;
    for _ in 0..500 {
        let cam = camera_at(1024.0, 1024.0, 4.0 + rng.below(40) as f64, rng.next() as u16, rng.below(40) as i32 - 30, rng.below(61) as i32 - 30);
        let (pu, pv, ph) = (900 + rng.below(250) as i16, 900 + rng.below(250) as i16, rng.below(30) as i16);
        let c = to_camera(&Eye::new(&cam), [pu << 2, ph << 2, pv << 2]);
        if c[2] < NEAR * 8 {
            continue;
        }
        let (x, y) = screen(c);
        let look = Look::new(&View { cam, pop: 0 });
        let g = look.to_camera([(pu as i32 * 16 - (cam.u >> 4)) as i16, (ph as i32 * 16 - (cam.h as i32 >> 4)) as i16, (pv as i32 * 16 - (cam.v >> 4)) as i16]);
        let (sx, sy) = project(g[0] as i16, g[1] as i16, g[2] as i16);
        let (ex, ey) = (sx as f64 / 8.0, sy as f64 / 16.0);
        // (Where it matters: on the view, and a little beyond its edges.)
        if !(-40.0..360.0).contains(&ex) || !(-40.0..180.0).contains(&ey) {
            continue;
        }
        // Two lines / 2.5 hires pixels: quarter texels and truncated rotations here, the reciprocal table
        // and Q4 there, magnified for points near the camera.
        assert!((x as f64 - ex).abs() <= 2.5 && (y as f64 - ey).abs() <= 2.0, "{cam:?} ({pu},{pv},{ph}): HUD ({x},{y}) view ({ex:.1},{ey:.1})");
        checked += 1;
    }
    assert!(checked > 100, "{checked}");
}

/// A HUD on a model with one target at (1024, 900), 3 texels up: the frame's sprites and the lock state
/// after `frames` frames 8 ticks apart, the camera at (1024, 800), 10 texels up, turned `yaw` (binary
/// angle; 0 north) and level, the waypoint at (1024, 1200).
fn hud_run(yaw: u16, frames: u32) -> (Machine, Vec<crate::hud::Sprite>) {
    use crate::hud::*;
    let mut m = Machine::new(package());
    init(&mut m);
    // (The frame's targets, as `combat::update` lists them.)
    for (i, b) in [1u8, 0, 4, 132, 3, 3].into_iter().enumerate() {
        m.ram[S_TARGETS as usize + i] = b;
    }
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.wu, a.wv) = (1024 << 8, 800 << 8, 10 << 8, 1024, 1200);
    let cam = camera_at(1024.0, 800.0, 10.0, yaw, 0, 0);
    for f in 0..frames {
        a.t = 100 + 8 * f;
        m.ram[READY as usize] = 0;
        update(&mut m, &a, &cam);
    }
    let n = m.ram[S_COUNT as usize] as usize;
    let list = (0..n)
        .map(|i| {
            let at = S_LIST as usize + 4 * i;
            Sprite { x: m.ram[at] as u16 | (m.ram[at + 1] as u16) << 8, y: m.ram[at + 2], ptr: m.ram[at + 3] }
        })
        .collect();
    (m, list)
}

/// The seeker locks on by itself: a target held in the diamond reticle locks after `LOCK_TICKS`, the
/// diamond on it and `LOCK` under it; a target outside the reticle gets a box but no lock.
#[test]
fn the_seeker_locks_onto_a_target_in_the_reticle() {
    use crate::hud::*;
    let (m, list) = hud_run(0, 20);
    assert_eq!((m.ram[LOCK_STATE as usize], m.ram[LOCK_STATE as usize + 1]), (0, LOCK_TICKS));
    assert_eq!(m.ram[(S_SHAPES + slot::DIAMOND as u16) as usize], shape::LOCKED);
    assert!(list.iter().any(|s| s.ptr == pointer(slot::LOCK)), "LOCK shown");
    assert!(list.iter().any(|s| s.ptr == pointer(slot::BOX)), "the target boxed");
    // Part way: still closing in.
    let (m, _) = hud_run(0, 6);
    let ticks = m.ram[LOCK_STATE as usize + 1];
    assert!(ticks > 0 && ticks < LOCK_TICKS, "{ticks}");
    assert_ne!(m.ram[(S_SHAPES + slot::DIAMOND as u16) as usize], shape::LOCKED);
    // Turned 20° away: boxed, not locked; the diamond waits at the boresight.
    let (m, list) = hud_run(20 * 182, 20);
    assert_eq!(m.ram[LOCK_STATE as usize], NONE);
    assert!(list.iter().any(|s| s.ptr == pointer(slot::BOX)));
    assert_eq!(m.ram[(S_SHAPES + slot::DIAMOND as u16) as usize], shape::DIAMOND);
}

/// The waypoint caret sits on the waypoint when it is in view; when it is behind or off to a side, an
/// arrow at the view's edge points toward it.
#[test]
fn the_waypoint_caret_tracks_the_waypoint() {
    use crate::hud::*;
    // Ahead (north): the caret, near the view's centre column.
    let (m, list) = hud_run(0, 1);
    assert_eq!(m.ram[(S_SHAPES + slot::CARET as u16) as usize], shape::CARET);
    let caret = list.iter().find(|s| s.ptr == pointer(slot::CARET)).unwrap();
    assert!((caret.x as i32 + 12 - 24 - 160).abs() <= 2, "caret at x {}", caret.x);
    // Facing (just off) south: behind, the arrow at the left or right edge.
    let (m, list) = hud_run(0x8000 + 182, 1);
    let s = m.ram[(S_SHAPES + slot::CARET as u16) as usize];
    assert!(s == shape::ARROW + 2 || s == shape::ARROW + 6, "shape {s}");
    let arrow = list.iter().find(|s| s.ptr == pointer(slot::CARET)).unwrap();
    assert!(arrow.x < 60 || arrow.x > 280, "arrow at x {}", arrow.x);
    // Facing east: the waypoint (north) is to the left.
    let (m, _) = hud_run(0x4000, 1);
    assert_eq!(m.ram[(S_SHAPES + slot::CARET as u16) as usize], shape::ARROW + 6);
}

/// The player's stick: no lift before the rotor is up to speed; forward lifts off and speeds up; left
/// turns left; the height looks after itself (never into a mountain, whatever the stick does); hovering
/// with the stick back settles onto the ground.
#[test]
fn the_player_flies_with_one_stick() {
    use crate::agent::{Agent, LIFT, MODE_PLAYER, THROTTLE_MAX, input};
    let mut reu: &[u8] = package();
    let mut a = Agent::new();
    a.mode = MODE_PLAYER;
    // (Fire held, as a player would: `main.rs` hands the player the controls each frame, except past the
    // sea limit, where a random stick may wander.)
    let run = |a: &mut Agent, reu: &mut &[u8], stick: u8, ticks: u32| {
        a.input = stick;
        for _ in 0..ticks {
            a.mode = MODE_PLAYER;
            a.step(reu);
        }
    };
    run(&mut a, &mut reu, input::UP, LIFT - 60);
    assert_eq!((a.grounded, a.h), (1, crate::agent::EYE), "lifted before the rotor was up to speed");
    run(&mut a, &mut reu, input::UP, 600);
    assert_eq!(a.grounded, 0, "no lift-off");
    assert_eq!(a.throttle, THROTTLE_MAX);
    assert!(a.speed() > 100, "speed {}", a.speed());
    let yaw = a.yaw;
    run(&mut a, &mut reu, input::LEFT, 120);
    let turned = a.yaw.wrapping_sub(yaw) as i16;
    assert!(turned < -2000, "turned {turned}");
    // A long, random stick: never below a mountain's top within its footprint.
    let mountains = crate::goap::mountains(&crate::world::generate(1983));
    let mut rng = Rng(0x57_1C);
    for _ in 0..200 {
        let stick = [0, input::UP, input::LEFT, input::RIGHT, input::UP | input::LEFT, input::UP | input::RIGHT][rng.below(6) as usize];
        for _ in 0..60 {
            run(&mut a, &mut reu, stick, 1);
            let (u, v, h) = (a.u as f64 / 256.0, a.v as f64 / 256.0, a.h as f64 / 256.0);
            for &(mu, mv, mr, mh) in &mountains {
                let d = ((u - mu).powi(2) + (v - mv).powi(2)).sqrt();
                if d < mr {
                    let top = mh * (1.0 - d / mr);
                    assert!(h > top, "tick {}: inside a mountain at ({u:.0}, {v:.0}), {h:.1} under {top:.1}", a.t);
                }
            }
        }
    }
    // Slow to a hover, then hold back: down onto the ground.
    run(&mut a, &mut reu, input::DOWN, 600);
    assert_eq!(a.grounded, 1, "did not land: h {} speed {}", a.h, a.speed());
}

/// The moving objects: the demo convoy and gunship seen from close by, drawn among the static scene (the
/// picture goes to `output/m2/close.png`), and turned the way they drive.
#[test]
fn moving_objects_are_drawn_where_they_are() {
    use crate::layout::BUFFERS;
    let reu = package();
    let mut m = Machine::new(reu);
    crate::package::setup(&mut m, BUFFERS[0]);
    // Looking north at the convoy on the loop round (1024, 820), 4 texels up, nose 10° down.
    let cam = camera_at(1020.0, 835.0, 4.0, 0, -10, 0);
    let t = 900;
    crate::actors::demo(&mut m, t, &cam);
    let n = m.ram[crate::actors::DYN as usize];
    assert!(n >= 4, "only {n} of the demo's objects in view");
    let scene = crate::package::scene_block(reu);
    let st = crate::package::render_floor(&mut m, BUFFERS[0], &scene, &cam, false);
    let mut rgb = m.decode(BUFFERS[0], crate::format::COLORS[0]);
    crate::hud::composite(&m, 0, &mut rgb);
    crate::png::save_wide(std::path::Path::new("output/m2/close.png"), 160, &rgb).unwrap();
    assert!(st.tris as u32 + st.dropped as u32 > st.geom.tris as u32, "{st:?}");
}

/// The war runs by itself: after ten minutes both sides have fought (losses on each, reinforcements
/// arrived), every unit is on the map, and the bookkeeping holds: the living counts and each sector's
/// counts are exactly the living units in it.
#[test]
fn the_war_goes_on_and_keeps_its_books() {
    use crate::world_sim::{self as ws, ALIVE, COUNT, RECORD, SIDE, rec};
    let mut m = Machine::new(package());
    crate::war::run(&mut m, 0, 10 * 3600, 4);
    let r = crate::war::report(&m.reu);
    assert!(r.lost[0] > 20 && r.lost[1] > 20, "losses {:?}", r.lost);
    assert!(r.alive[0] > 20 && r.alive[1] > 10, "alive {:?}", r.alive);
    let mut alive = [0u16; 2];
    let mut sectors = vec![[0u8; 2]; 256];
    for id in 0..COUNT as usize {
        let at = ws::UNITS as usize + id * RECORD as usize;
        let u = &m.reu[at..at + RECORD as usize];
        let (pu, pv) = (u16::from_le_bytes([u[4], u[5]]) >> 4, u16::from_le_bytes([u[6], u[7]]) >> 4);
        assert!(pu < 2048 && pv < 2048, "unit {id} at ({pu}, {pv})");
        if u[rec::FLAGS as usize] & ALIVE != 0 {
            let side = (u[rec::FLAGS as usize] & SIDE) as usize;
            alive[side] += 1;
            let s = u[rec::SECTOR as usize] as usize;
            assert_eq!(s, ((pu >> 7) | (pv >> 7) << 4) as usize, "unit {id}'s sector");
            sectors[s][side] += 1;
        }
    }
    assert_eq!(alive, r.alive, "the living counts");
    for (s, c) in sectors.iter().enumerate() {
        let a = ws::SECTORS as usize + s * ws::SECTOR as usize;
        assert_eq!([m.reu[a], m.reu[a + 1]], *c, "sector {s}'s counts");
    }
}

/// The front: over two hours, pads change hands both ways (each side takes one), home never falls, and
/// neither side is wiped out.
#[test]
fn the_front_swings_both_ways() {
    use crate::actors::{ALLIED, ENEMY};
    let mut m = Machine::new(package());
    let (mut taken, mut prev) = ([0u32; 2], crate::war::report(&m.reu).owner);
    for minute in 0..120 {
        crate::war::run(&mut m, minute * 3600, 3600, 4);
        let r = crate::war::report(&m.reu);
        for (p, (&a, &b)) in prev.iter().zip(&r.owner).enumerate() {
            if a != b {
                taken[b as usize] += 1;
            }
            assert!(p != 0 || b == ALLIED, "home fell at minute {minute}");
        }
        assert!(r.alive[ENEMY as usize] > 0 && r.alive[ALLIED as usize] > 0, "a side wiped out at minute {minute}");
        prev = r.owner;
    }
    assert!(taken[0] >= 2 && taken[1] >= 2, "pads taken (enemy, allies): {taken:?}");
}

/// The fleets meet: within an hour, enemy boats are sunk and the carrier group takes fire, and the
/// carrier is still afloat at the end.
#[test]
fn the_fleets_meet() {
    use crate::actors::{ALLIED, ENEMY, kind};
    use crate::world_sim::{self as ws, ALIVE, SIDE, rec, stats};
    let mut m = Machine::new(package());
    let (mut sunk, mut hit) = (false, false);
    for minute in 0..60 {
        crate::war::run(&mut m, minute * 3600, 3600, 4);
        for id in 0..ws::COUNT as usize {
            let r = &m.reu[ws::UNITS as usize + id * ws::RECORD as usize..][..ws::RECORD as usize];
            let k = r[rec::KIND as usize] as usize % kind::COUNT as usize;
            if stats::CLASS[k] != 4 {
                continue;
            }
            let (alive, side) = (r[rec::FLAGS as usize] & ALIVE != 0, r[rec::FLAGS as usize] & SIDE);
            sunk |= side == ENEMY && !alive;
            hit |= side == ALLIED && (!alive || r[rec::HP as usize] < stats::HP[k]);
        }
    }
    assert!(sunk && hit, "enemy boats sunk {sunk}, allied ships hit {hit}");
    let carrier = (0..ws::COUNT as usize).map(|id| &m.reu[ws::UNITS as usize + id * ws::RECORD as usize..][..ws::RECORD as usize]).find(|r| r[rec::KIND as usize] == kind::CARRIER).unwrap();
    assert!(carrier[rec::FLAGS as usize] & ALIVE != 0, "the carrier sank");
}

/// The near list and `NEAR_FLAG` agree: a living unit is flagged exactly when it is listed (drawn), so one
/// left out of a full list (or pushed out by the carrier) can't fire at the player unseen.
#[test]
fn near_flag_means_listed() {
    use crate::world_sim::{self as ws, ALIVE, NEAR_FLAG, carrier, rec};
    let mut m = Machine::new(package());
    let mut t = 0u32;
    while t < 10 * 3600 {
        // The player by the carrier's group, where the fleets meet.
        let d = carrier(&mut &m.reu[..], t as u16).expect("a carrier");
        let (pu, pv) = ((d.u >> 4) as i16, (d.v >> 4) as i16);
        ws::step_page(&mut m, t as u16, pu, pv);
        t += 4;
        let n = m.ram[ws::NEAR as usize].min(ws::NEAR_MAX);
        let listed: Vec<u16> = (0..n as usize).map(|k| u16::from_le_bytes([m.ram[ws::NEAR as usize + 1 + 2 * k], m.ram[ws::NEAR as usize + 2 + 2 * k]])).collect();
        for id in 0..ws::COUNT {
            // (The page in RAM is stashed at the end of its phase: read the REU's copy.)
            let r = &m.reu[ws::UNITS as usize + id as usize * ws::RECORD as usize..][..ws::RECORD as usize];
            let (alive, flagged) = (r[rec::FLAGS as usize] & ALIVE != 0, r[rec::FLAGS as usize] & NEAR_FLAG != 0);
            if alive {
                assert_eq!(flagged, listed.contains(&id), "unit {id} at tick {t}: flagged {flagged}");
            }
        }
    }
}

/// A full near list: the carrier coming near takes the last place, and the unit it pushes out is unflagged
/// at once (in the REU: it is in a later page).
#[test]
fn the_carrier_pushes_the_last_near_unit_out_unflagged() {
    use crate::world_sim::{self as ws, NEAR_FLAG, carrier, head, rec};
    let mut m = Machine::new(package());
    let id = u16::from_le_bytes([m.reu[(ws::HEADER + head::CARRIER as u32) as usize], m.reu[(ws::HEADER + head::CARRIER as u32) as usize + 1]]);
    let flags = |m: &Machine, u: u16| m.reu[ws::UNITS as usize + u as usize * ws::RECORD as usize + rec::FLAGS as usize];
    // Twelve units from the pages after the carrier's, listed and flagged.
    let later = (id / ws::PAGE + 1) * ws::PAGE;
    let full: Vec<u16> = (later..ws::COUNT).take(ws::NEAR_MAX as usize).collect();
    assert_eq!(full.len(), ws::NEAR_MAX as usize, "units after the carrier's page (it is unit {id})");
    m.ram[ws::NEAR as usize] = ws::NEAR_MAX;
    for (k, &u) in full.iter().enumerate() {
        m.ram[ws::NEAR as usize + 1 + 2 * k..][..2].copy_from_slice(&u.to_le_bytes());
        let f = ws::UNITS as usize + u as usize * ws::RECORD as usize + rec::FLAGS as usize;
        m.reu[f] |= NEAR_FLAG;
    }
    // The pages up to the carrier's move, the player beside it.
    let d = carrier(&mut &m.reu[..], 0).unwrap();
    let (pu, pv) = ((d.u >> 4) as i16, (d.v >> 4) as i16);
    for f in 0..=3 * (id / ws::PAGE) {
        ws::step_page(&mut m, 4 * f, pu, pv);
    }
    let listed: Vec<u16> = (0..ws::NEAR_MAX as usize).map(|k| u16::from_le_bytes([m.ram[ws::NEAR as usize + 1 + 2 * k], m.ram[ws::NEAR as usize + 2 + 2 * k]])).collect();
    let out = full[ws::NEAR_MAX as usize - 1];
    assert!(listed.contains(&id) && !listed.contains(&out), "the carrier in the last place: {listed:?}");
    assert_eq!(flags(&m, out) & NEAR_FLAG, 0, "unit {out} pushed out but still flagged");
    assert!(flags(&m, full[0]) & NEAR_FLAG != 0);
}

/// On long laps of the war (15 ticks a frame, as on a slow C64 frame), the carrier's deck still moves
/// every tick, never jumping: its projection covers a whole lap.
#[test]
fn deck_moves_smoothly_on_long_laps() {
    use crate::world_sim::{self as ws, carrier, stats};
    let mut m = Machine::new(package());
    let speed = stats::SPEED[crate::actors::kind::CARRIER as usize] as i32;
    let mut last = carrier(&mut &m.reu[..], 0).unwrap();
    let (mut t, mut still) = (0u32, 0u32);
    while t < 20 * 3600 {
        ws::step_page(&mut m, t as u16, -4000, -4000);
        for k in 1..=15 {
            let Some(d) = carrier(&mut &m.reu[..], (t + k) as u16) else { return };
            let step = (d.u as i32 - last.u as i32).abs().max((d.v as i32 - last.v as i32).abs());
            assert!(step <= speed + 1, "the deck jumped {step}/16 texel at tick {}", t + k);
            still += (step == 0 && d.yaw == last.yaw) as u32;
            last = d;
        }
        t += 15;
    }
    assert!(still < 20 * 3600 / 10, "the deck stood still for {still} ticks");
}

/// The carrier on station with the enemy flotilla near (the player on its deck, as at the start): it
/// keeps a course, turning back at most at its road's ends and on a fresh retreat, never back and forth
/// every lap of the war (a retreat turns a unit back once, `act::BACK`).
#[test]
fn the_carrier_does_not_see_saw() {
    use crate::world_sim::{self as ws, carrier};
    let mut m = Machine::new(package());
    let start = carrier(&mut &m.reu[..], 0).unwrap();
    let (mut last, mut way) = ((start.u as i32, start.v as i32), (0i32, 0i32));
    let mut reversals = 0;
    for t in (0..3 * 3600u32).step_by(8) {
        let d = carrier(&mut &m.reu[..], t as u16).unwrap();
        ws::step_page(&mut m, t as u16, (d.u >> 4) as i16, (d.v >> 4) as i16);
        // Its way made good over each 256 ticks (2 war laps' worth or so).
        if t % 256 == 0 && t > 0 {
            let now = (d.u as i32, d.v as i32);
            let w = (now.0 - last.0, now.1 - last.1);
            if w.0 * way.0 + w.1 * way.1 < 0 {
                reversals += 1;
            }
            if w != (0, 0) {
                way = w;
            }
            last = now;
        }
    }
    assert!(reversals <= 2, "the carrier turned back {reversals} times in 3 minutes");
}

/// Decisions as acted on: a retreat turns back only when not turned already, whatever came between,
/// and once clear of foes the unit turns back to its way; the carrier keeps its station and does not
/// learn from it.
#[test]
fn decisions_as_acted_on() {
    use crate::brain::unit;
    use crate::world_sim::{act, chosen};
    // (mv, turned, clear, rally, carrier, static)
    assert_eq!(chosen(unit::RETREAT, false, false, false, false, false), (act::RETREAT, true));
    assert_eq!(chosen(unit::RETREAT, true, false, false, false, false), (act::BACK, true));
    assert_eq!(chosen(unit::ATTACK, true, false, false, false, false), (act::FIGHT, true));
    // Clear of foes, turned back: it holds until friends gather (or it tires of waiting: `rally` either
    // way), then back to its way, once, not learned from.
    assert_eq!(chosen(unit::ADVANCE, true, true, false, false, false), (act::HOLD, false));
    assert_eq!(chosen(unit::ADVANCE, true, true, true, false, false), (act::RETREAT, false));
    assert_eq!(chosen(unit::ATTACK, true, true, true, false, false), (act::RETREAT, false));
    assert_eq!(chosen(unit::ADVANCE, false, true, false, false, false), (act::MOVE, true));
    assert_eq!(chosen(unit::RETREAT, false, false, false, true, false), (act::MOVE, false));
    assert_eq!(chosen(unit::RETREAT, true, false, false, true, false), (act::MOVE, false));
    assert_eq!(chosen(unit::HOLD, false, false, false, false, false), (act::HOLD, true));
    assert_eq!(chosen(unit::ADVANCE, false, false, false, false, true), (act::HOLD, true));
    assert_eq!(chosen(unit::ADVANCE, false, false, false, false, false), (act::MOVE, true));
}

/// A retreat turns a unit back once: over 10 minutes of war, no unit turns back along its road twice
/// without foes having been away from it between (whatever else it decided meanwhile).
#[test]
fn retreats_do_not_see_saw() {
    use crate::world_sim::{self as ws, carrier, trace};
    let mut m = Machine::new(package());
    trace::LOG.with_borrow_mut(|l| l.clear());
    for t in (0..10 * 3600u32).step_by(8) {
        let d = carrier(&mut &m.reu[..], t as u16).unwrap();
        ws::step_page(&mut m, t as u16, (d.u >> 4) as i16, (d.v >> 4) as i16);
    }
    let log = trace::LOG.with_borrow(|l| l.clone());
    let mut armed = vec![true; ws::COUNT as usize];
    let (mut again, mut reversals) = (0, 0);
    for (id, e) in log {
        if e == trace::CLEAR {
            armed[id as usize] = true;
        } else if e == trace::REVERSED {
            reversals += 1;
            again += !armed[id as usize] as u32;
            armed[id as usize] = false;
        }
    }
    assert!(reversals > 0, "no retreats to judge");
    assert_eq!(again, 0, "{again} of {reversals} turns back came with foes near all along since the last");
}

/// A straggler goes back in: fallen back (turned), clear of foes and alone, it holds a while for friends
/// to gather, but not for good.
#[test]
fn stragglers_go_back_in() {
    use crate::world_sim::{self as ws, ALIVE, RECORD, TURNED, act, rec};
    let mut m = Machine::new(package());
    // The first mobile unit of the first page, fallen back into an empty sector of its own.
    let at = |id: usize| ws::UNITS as usize + id * RECORD as usize;
    let id = (0..ws::PAGE as usize)
        .find(|&id| {
            let r = &m.reu[at(id)..];
            let k = r[rec::KIND as usize] as usize % crate::actors::kind::COUNT as usize;
            r[rec::FLAGS as usize] & ALIVE != 0 && ws::stats::SPEED[k] > 0 && r[rec::ROUTE as usize] != 0xFF && k != crate::actors::kind::CARRIER as usize
        })
        .expect("a mobile unit");
    let r = at(id);
    m.reu[r + rec::FLAGS as usize] |= TURNED;
    m.reu[r + rec::ACTION as usize] = act::HOLD;
    let side = (m.reu[r + rec::FLAGS as usize] & ws::SIDE) as usize;
    let sector = m.reu[r + rec::SECTOR as usize] as usize;
    let s = ws::SECTORS as usize + sector * ws::SECTOR as usize;
    let mut counts = [0u8; 2];
    counts[side] = 1;
    // Decisions until it goes back in (about 8 on average).
    let held = (0..200)
        .take_while(|_| {
            m.reu[s..s + 2].copy_from_slice(&counts);
            ws::decide_page(&mut m);
            m.reu[r + rec::ACTION as usize] == act::HOLD
        })
        .count();
    assert!(held < 100, "a straggler held through {held} decisions");
    assert_eq!(m.reu[r + rec::ACTION as usize], act::RETREAT, "back in, on its way");
}

/// A tank of side 0 on its front road, alive, at `leg` of it, at texels (`u`, `v`) heading `yaw`, last
/// moved `dt` ticks before `now`, doing `action` (`flags` more): its record's address.
#[cfg(test)]
fn on_the_road(m: &mut Machine, leg: u8, (u, v): (f64, f64), yaw: u8, now: u16, dt: u16, action: u8, flags: u8) -> usize {
    use crate::world_sim::{self as ws, ALIVE, rec};
    assert_eq!(u16::from_le_bytes([m.reu[ws::HEADER as usize], m.reu[ws::HEADER as usize + 1]]), 0, "the first page next");
    let r = ws::UNITS as usize;
    let rec_ = &mut m.reu[r..r + ws::RECORD as usize];
    rec_[rec::KIND as usize] = crate::actors::kind::TANK;
    rec_[rec::FLAGS as usize] = ALIVE | flags;
    rec_[rec::HP as usize] = ws::stats::HP[crate::actors::kind::TANK as usize];
    rec_[rec::ACTION as usize] = action;
    rec_[rec::SPEED as usize] = ws::stats::SPEED[crate::actors::kind::TANK as usize];
    rec_[rec::ROUTE as usize] = ws::FRONT_ROUTE[0];
    rec_[rec::LEG as usize] = leg;
    rec_[rec::U as usize..rec::U as usize + 2].copy_from_slice(&((u * 16.0) as u16).to_le_bytes());
    rec_[rec::V as usize..rec::V as usize + 2].copy_from_slice(&((v * 16.0) as u16).to_le_bytes());
    (rec_[rec::YAW as usize], rec_[rec::YAW_FINE as usize], rec_[rec::COURSE as usize]) = (yaw, 0, yaw);
    rec_[rec::LAST as usize..rec::LAST as usize + 2].copy_from_slice(&now.wrapping_sub(dt).to_le_bytes());
    r
}

/// Round its road's end, a unit turned back on a retreat stays so: chased there, clearing it would only
/// have it turn back and forth at the end while foes stay near.
#[test]
fn road_ends_keep_a_retreat() {
    use crate::world_sim::{self as ws, TURNED, act, rec};
    let mut m = Machine::new(package());
    let (start, len, mode) = ws::route(&mut m, ws::FRONT_ROUTE[0]);
    assert!(mode != ws::LOOP && len >= 2);
    // On its last waypoint, standing (no step): done with it, round the end.
    let (u, v) = ws::waypoint(&mut m, start + len - 1);
    let r = on_the_road(&mut m, len - 1, (u as f64 + 0.5, v as f64 + 0.5), 0, 4000, 0, act::BACK, TURNED);
    ws::move_page(&mut m, 4000, -4000, -4000);
    assert_eq!(m.reu[r + rec::LEG as usize], len, "round the end");
    assert_ne!(m.reu[r + rec::FLAGS as usize] & TURNED, 0, "still turned back round the end");
}

/// A retreat falls back to the waypoint behind it, even when a long step (catching up) has just taken
/// it past that one: it now goes the other way, so that waypoint is ahead, not passed.
#[test]
fn retreats_fall_back_to_their_waypoint() {
    use crate::world_sim::{self as ws, CATCH_UP, act, rec};
    let mut m = Machine::new(package());
    let (start, len, mode) = ws::route(&mut m, ws::FRONT_ROUTE[0]);
    assert!(mode != ws::LOOP && len >= 3);
    // Bound for waypoint 2, an eighth of this step short of waypoint 1 on the way there: the step takes
    // it past 1 (within the step, behind it), and then it turns back.
    let (a, b) = (ws::waypoint(&mut m, start + 1), ws::waypoint(&mut m, start + 2));
    let (dx, dy) = ((b.0 - a.0) as f64, (b.1 - a.1) as f64);
    let n = (dx * dx + dy * dy).sqrt();
    let s = (ws::stats::SPEED[crate::actors::kind::TANK as usize] as u32 * CATCH_UP as u32 / 16) as f64;
    assert!(n > s, "a leg longer than the step");
    let at = (a.0 as f64 + 0.5 - dx / n * s / 8.0, a.1 as f64 + 0.5 - dy / n * s / 8.0);
    let r = on_the_road(&mut m, 2, at, ws::heading(dx as i16, dy as i16), 4000, CATCH_UP, act::RETREAT, 0);
    ws::move_page(&mut m, 4000, -4000, -4000);
    // Back along the road (legs len.. in reverse): bound for waypoint 1.
    let leg = m.reu[r + rec::LEG as usize];
    assert!(leg >= len, "turned back: leg {leg}");
    assert_eq!(2 * len - 2 - leg, 1, "bound for waypoint 1 (leg {leg} of a road of {len})");
}

/// A respawned unit faces its way, not the way it faced when it fell: off the carrier's deck, the deck's
/// heading (it turns off it on its arc); elsewhere, its first leg.
#[test]
fn respawns_face_their_way() {
    use crate::world_sim::{self as ws, ALIVE, CARRIER_AIR, COUNT, RECORD, carrier, rec};
    let mut m = Machine::new(package());
    let at = |id: u16| ws::UNITS as usize + id as usize * RECORD as usize;
    let mut was: Vec<bool> = (0..COUNT).map(|id| m.reu[at(id) + rec::FLAGS as usize] & ALIVE != 0).collect();
    let (mut air, mut ground) = (0, 0);
    for t in (0..30 * 3600u32).step_by(8) {
        let d = carrier(&mut &m.reu[..], t as u16).unwrap();
        ws::step_page(&mut m, t as u16, (d.u >> 4) as i16, (d.v >> 4) as i16);
        for id in 0..COUNT {
            let r = &m.reu[at(id)..][..RECORD as usize];
            let alive = r[rec::FLAGS as usize] & ALIVE != 0;
            if alive && !was[id as usize] && r[rec::ROUTE as usize] != 0xFF {
                let yaw = u16::from_le_bytes([r[rec::YAW_FINE as usize], r[rec::YAW as usize]]);
                if r[rec::ROUTE as usize] == CARRIER_AIR {
                    let deck = carrier(&mut &m.reu[..], t as u16).unwrap();
                    assert_eq!(yaw, deck.yaw, "unit {id} off the deck at {t}");
                    air += 1;
                } else {
                    assert_eq!(yaw, (r[rec::COURSE as usize] as u16) << 8, "unit {id} respawned at {t}");
                    ground += 1;
                }
            }
            was[id as usize] = alive;
        }
    }
    assert!(air > 0 && ground > 0, "respawns off the deck {air}, elsewhere {ground}");
}

/// The sea routes stay at sea: every waypoint of the flotilla's raid and the carrier group's station is
/// water, inside the sea room off the map's edge, and so is every leg of the carrier's station. (The
/// raid's legs, 30° of bearing apart, still cut across headlands: known, not yet fixed.)
#[test]
fn sea_routes_stay_at_sea() {
    use crate::war::{routes, water};
    let w = crate::world::generate(1983);
    let rs = routes(&w);
    for k in [4, 5] {
        let pts = &rs[k].0;
        for &(u, v) in pts {
            assert!(water(&w, u, v), "route {k}: waypoint ({u}, {v}) on land");
            assert!((48..2000).contains(&u) && (48..2000).contains(&v), "route {k}: waypoint ({u}, {v}) by the edge");
        }
    }
    for leg in rs[5].0.windows(2) {
        let ((u0, v0), (u1, v1)) = (leg[0], leg[1]);
        let n = ((u1 - u0).abs().max((v1 - v0).abs()) / 4).max(1);
        for i in 0..=n {
            let (u, v) = (u0 + (u1 - u0) * i / n, v0 + (v1 - v0) * i / n);
            assert!(water(&w, u, v), "the carrier's station crosses land at ({u}, {v})");
        }
    }
}

/// Every moving unit turns on an arc at its kind's rate (`stats::TURN`), never snapping round: on long
/// laps of the war (15 ticks a frame), each tick its heading turns by at most its rate and it moves by at
/// most its speed. The carrier, on its station, turns at its waypoints too.
#[test]
fn units_turn_on_arcs() {
    use crate::world_sim::{self as ws, ALIVE, COUNT, RECORD, rec, stats};
    let mut m = Machine::new(package());
    let head = |m: &Machine, id: u16| -> [u8; ws::CARRIER_RECORD] { m.reu[ws::UNITS as usize + id as usize * RECORD as usize..][..ws::CARRIER_RECORD].try_into().unwrap() };
    let mut last: Vec<Option<(u16, u16, u16)>> = vec![None; COUNT as usize];
    let (mut turned, mut t) = (0u32, 0u32);
    while t < 10 * 3600 {
        ws::step_page(&mut m, t as u16, -4000, -4000);
        for k in 1..=15 {
            for id in 0..COUNT {
                let r = head(&m, id);
                let kind = r[rec::KIND as usize] as usize % stats::TURN.len();
                if r[rec::FLAGS as usize] & ALIVE == 0 || r[rec::ACTION as usize] == ws::act::HOLD {
                    last[id as usize] = None;
                    continue;
                }
                let now = ws::live(&r, (t + k) as u16);
                if let Some(was) = last[id as usize] {
                    // (Its heading kept whole across its steps: no slack for a step landing back.)
                    let rate = ((1u32 << stats::TURN[kind]) << 5) + 1;
                    let dy = (now.2.wrapping_sub(was.2) as i16).unsigned_abs() as u32;
                    assert!(dy <= rate, "kind {kind} (unit {id}) turned {dy}/65536 in a tick at {}", t + k);
                    let step = (now.0 as i32 - was.0 as i32).abs().max((now.1 as i32 - was.1 as i32).abs());
                    assert!(step <= stats::SPEED[kind] as i32 + 1, "kind {kind} (unit {id}) jumped {step}/16 texel at {}", t + k);
                    turned += (dy > 0) as u32;
                }
                last[id as usize] = Some(now);
            }
        }
        t += 15;
    }
    assert!(turned > 1000, "hardly any turning: {turned}");
}

/// A waypoint inside a unit's turning circle (one it could not turn onto) counts as reached, for every
/// moving kind and wherever round the circle, however [`ws::dist`] estimates it: no unit circles a
/// waypoint for ever.
#[test]
fn slow_turners_take_waypoints_abeam() {
    use crate::world_sim::{self as ws, stats};
    for kind in 0..stats::TURN.len() {
        let speed = stats::SPEED[kind] as u16;
        if speed == 0 {
            continue;
        }
        // The radius as `live` turns it (1/16 texel), in texels.
        let rad = ((speed * 326) >> stats::TURN[kind]) as f64 / 16.0;
        let reach = ws::reach(kind);
        for a in 0..256 {
            for side in [-1.0, 1.0] {
                // A point inside the circle centred abeam (either side) of a unit heading north at the origin.
                let t = a as f64 * std::f64::consts::TAU / 256.0;
                let (x, y) = (side * rad + 0.98 * rad * t.cos(), 0.98 * rad * t.sin());
                let d = ws::dist(x.round() as i16, y.round() as i16);
                assert!(d <= reach, "kind {kind}: a waypoint inside its circle at ({x:.1}, {y:.1}) is {d} away, past {reach}");
            }
        }
    }
}

/// Done with a waypoint: one holding still keeps it, however near; one passed counts by the way it is
/// heading now (mid-turn), not the course it is turning to.
#[test]
fn waypoints_done_with() {
    use crate::world_sim::done_with;
    // Holding beside it.
    assert!(!done_with(0, 5, 0, 0, 0, true));
    assert!(done_with(0, 5, 0, 0, 0, false));
    // Just behind it (bearing 128 off its heading), within a step: passed.
    assert!(done_with(8, 5, 160, 128, 0, false));
    // Just ahead of it: not passed (on to it).
    assert!(!done_with(8, 5, 160, 0, 0, false));
    // Out of the step's reach: not passed, behind or not.
    assert!(!done_with(30, 5, 160, 128, 0, false));
}

/// A turn takes as long to the left as to the right, and its heading is kept whole across steps (the
/// record's `YAW_FINE`): no step lands it back.
#[test]
fn turns_are_the_same_both_ways() {
    use crate::world_sim::{self as ws, ALIVE, act, rec, stats};
    for kind in 0..stats::TURN.len() as u8 {
        if stats::SPEED[kind as usize] == 0 {
            continue;
        }
        let ticks = |course: u8| {
            let mut r = [0u8; ws::CARRIER_RECORD];
            r[rec::KIND as usize] = kind;
            r[rec::FLAGS as usize] = ALIVE;
            r[rec::ACTION as usize] = act::MOVE;
            r[rec::ROUTE as usize] = 0;
            r[rec::SPEED as usize] = stats::SPEED[kind as usize];
            r[rec::U as usize..rec::U as usize + 2].copy_from_slice(&16000u16.to_le_bytes());
            r[rec::V as usize..rec::V as usize + 2].copy_from_slice(&16000u16.to_le_bytes());
            r[rec::COURSE as usize] = course;
            // Stepped every 3 ticks, as a fine heading that must not be lost each time.
            let mut t = 0u16;
            loop {
                let (u, v, yaw) = ws::live(&r, t + 3);
                t += 3;
                if yaw == (course as u16) << 8 {
                    return t;
                }
                assert!(t < 4000, "kind {kind} never reached {course}");
                r[rec::U as usize..rec::U as usize + 2].copy_from_slice(&u.to_le_bytes());
                r[rec::V as usize..rec::V as usize + 2].copy_from_slice(&v.to_le_bytes());
                r[rec::YAW as usize] = (yaw >> 8) as u8;
                r[rec::YAW_FINE as usize] = yaw as u8;
                r[rec::LAST as usize..rec::LAST as usize + 2].copy_from_slice(&t.to_le_bytes());
            }
        };
        let (right, left) = (ticks(64), ticks(192));
        assert!(right.abs_diff(left) <= 3, "kind {kind}: right {right} ticks, left {left}");
        let want = (64u32 << 8) / ((1u32 << stats::TURN[kind as usize]) << 5);
        assert!((right as u32).abs_diff(want) <= 3, "kind {kind}: a quarter turn in {right} ticks, not {want}");
    }
}

/// The carrier: an agent placed on its deck rides it (keeping its place on the deck while the ship
/// steams on with the war), then the autopilot lifts off and climbs away from it.
#[test]
fn the_deck_carries_the_helicopter_until_it_lifts_off() {
    use crate::agent::{Agent, LIFT, ON_DECK, PLACE_CARRIER};
    use crate::world_sim::{self as ws, DECK_TOP, carrier};
    let mut m = Machine::new(package());
    let mut a = Agent::new();
    a.deck = PLACE_CARRIER;
    let d0 = carrier(&mut &m.reu[..], 0).expect("a carrier");
    let mut t = 0u32;
    while t < LIFT - 60 {
        let (pu, pv) = a.at();
        ws::step_page(&mut m, t as u16, pu, pv);
        t += 4;
        a.fly(&mut &m.reu[..], t);
        assert_eq!((a.deck, a.grounded), (ON_DECK, 1), "off the deck at tick {t}");
    }
    let d = carrier(&mut &m.reu[..], a.t as u16).unwrap();
    let moved = ((d.u as i32 - d0.u as i32).abs() + (d.v as i32 - d0.v as i32).abs()) >> 4;
    assert!(moved > 40, "the carrier moved only {moved} texels");
    // Still aft on the deck, facing the bow, on top of it.
    let (du, dv) = ((a.u >> 4) - d.u as i32, (a.v >> 4) - d.v as i32);
    let back = (crate::camera::sin(d.yaw) * -64 >> 14, crate::camera::cos(d.yaw) * -64 >> 14);
    assert!((du - back.0).abs() <= 2 && (dv - back.1).abs() <= 2, "on the deck at ({du}, {dv}), not ({}, {})", back.0, back.1);
    assert_eq!((a.yaw, a.h), (d.yaw, crate::agent::DECK_EYE + DECK_TOP));
    // The rotor up to speed: off it and away.
    while t < LIFT + 30 * 60 {
        let (pu, pv) = a.at();
        ws::step_page(&mut m, t as u16, pu, pv);
        t += 4;
        a.fly(&mut &m.reu[..], t);
    }
    assert_eq!((a.deck, a.grounded), (0, 0));
    assert!(a.h > crate::agent::DECK_EYE + DECK_TOP + (8 << 8), "only {} up", a.h >> 8);
}

/// The soft boundary at sea: flying straight out east with the fire button held, the SEA lamp lights
/// first; past the limit the autopilot takes over (fire can't take it back out there) and, the button
/// let go, flies home over the island.
#[test]
fn the_sea_lamp_warns_and_the_autopilot_turns_back() {
    use crate::agent::{Agent, CRUISE, EYE, HOME, LIFT, MODE_AUTO, MODE_PLAYER, SEA_LIMIT, THROTTLE_MAX, input};
    use crate::cockpit::{gauge, lamp, readings};
    let reu = package();
    let mut a = Agent::new();
    (a.t, a.u, a.v, a.h, a.yaw, a.grounded) = (LIFT + 100, 1900 << 8, 1024 << 8, EYE + ((CRUISE as i32) << 8), 0x4000, 0);
    (a.mode, a.throttle, a.vf) = (MODE_PLAYER, THROTTLE_MAX, 8192);
    let lamp_on = |a: &Agent| {
        let mut k = [0u8; gauge::COUNT];
        readings(a, &mut k);
        k[gauge::WARN] & lamp::SEA != 0
    };
    assert!(!lamp_on(&a), "the lamp over the map");
    let (mut warned, mut taken, mut far, mut held) = (false, false, 0i16, 0u32);
    // The stick forward and fire held (`main.rs`: fire hands the player the controls every frame), until
    // ten seconds after the takeover.
    while held < 600 {
        a.input = input::UP | input::FIRE;
        a.mode = MODE_PLAYER;
        let t = a.t + 4;
        a.fly(&mut &reu[..], t);
        warned |= lamp_on(&a) && !taken;
        if a.mode == MODE_AUTO {
            assert!(warned, "taken over without the lamp first");
            assert_eq!((a.wu, a.wv), (HOME.0 as i16, HOME.1 as i16), "for home");
            taken = true;
        }
        held += 4 * taken as u32;
        far = far.max(a.off_map());
        assert!(a.t < LIFT + 100 + 60 * 60, "never taken over: {} texels out", a.off_map());
    }
    // (The autopilot's turn, at most 20° of bank, carries it about 200 texels further.)
    assert!(far <= SEA_LIMIT + 240, "{far} texels out to sea");
    // Fire let go: home over the island within 90 s.
    a.input = 0;
    let back = a.t + 90 * 60;
    while a.off_map() > 0 {
        let t = a.t + 4;
        a.fly(&mut &reu[..], t);
        assert!(a.t < back, "still {} texels out", a.off_map());
    }
    assert!(!lamp_on(&a));
}

/// A hit that takes the damage to `DOWN` downs the player even landed at a base (where it is repaired a
/// point a frame): the repair comes after the check.
#[test]
fn a_lethal_hit_on_the_pad_downs_the_player() {
    use crate::combat::{DOWN, PLAYER, player};
    let mut m = Machine::new(package());
    crate::combat::init(&mut m);
    let a = crate::agent::Agent::new();
    assert_eq!(a.grounded, 1);
    m.ram[(PLAYER + player::DAMAGE) as usize] = DOWN - 1;
    assert!(!crate::combat::update(&mut m, &a, &a.camera().0), "not down yet");
    assert_eq!(m.ram[(PLAYER + player::DAMAGE) as usize], DOWN - 2, "repaired on the pad");
    m.ram[(PLAYER + player::DAMAGE) as usize] = DOWN;
    assert!(crate::combat::update(&mut m, &a, &a.camera().0), "down on the pad");
}

/// Going down: the agent stays where it went down until its next flight (the frame drawn after is the
/// crash), then it is on the carrier's deck or at home, in turn.
#[test]
fn going_down_keeps_the_crash_spot_for_a_frame() {
    use crate::agent::{Agent, HOME, ON_DECK, PLACE_CARRIER, PLACE_HOME};
    let reu = package();
    let mut crash = Agent::new();
    (crash.u, crash.v, crash.h, crash.yaw, crash.grounded) = (300 << 8, 400 << 8, 20 << 8, 0x1234, 0);
    for (t, place) in [(4096 * 2 + 10, PLACE_CARRIER), (4096 * 3 + 10, PLACE_HOME)] {
        crash.t = t;
        let mut a = crash;
        a.reset();
        assert_eq!((a.u, a.v, a.h, a.yaw, a.deck), (crash.u, crash.v, crash.h, crash.yaw, place));
        a.fly(&mut &reu[..], t + 1);
        if place == PLACE_CARRIER {
            assert_eq!(a.deck, ON_DECK, "on the deck");
        } else {
            assert_eq!((a.deck, a.at()), (0, (HOME.0 as i16, HOME.1 as i16)), "at home");
        }
    }
}

/// The host's reference flight starts as the C64 does: on the carrier's deck.
#[test]
fn the_reference_starts_on_the_carrier() {
    let a = crate::agent::at(package(), 1);
    assert_eq!((a.deck, a.grounded), (crate::agent::ON_DECK, 1));
}

/// Half the map away from the carrier on one axis (the offset at `i16::MIN`), low: not over its deck.
#[test]
fn far_from_the_deck_is_not_over_it() {
    use crate::agent::{Agent, EYE};
    use crate::world_sim::{DECK_TOP, carrier};
    let reu = package();
    let d = carrier(&mut &reu[..], 100).unwrap();
    let mut a = Agent::new();
    (a.t, a.u, a.v, a.h) = (100, ((d.u as i32) + 0x8000) << 4, (d.v as i32) << 4, EYE);
    a.fly(&mut &reu[..], 101);
    assert!(a.h < EYE + DECK_TOP, "lifted onto a deck 2048 texels away");
}

/// Landing: hovering with the stick back settles the player onto the deck when over it (riding it), into
/// the sea when not (ditched, and down with a splash).
#[test]
fn landing_on_the_deck_or_in_the_sea() {
    use crate::agent::{Agent, DITCHED, EYE, MODE_PLAYER, ON_DECK, input};
    use crate::world_sim::{DECK_TOP, carrier};
    let mut m = Machine::new(package());
    crate::combat::init(&mut m);
    crate::war::run(&mut m, 0, 6000, 4);
    let d = carrier(&mut &m.reu[..], 6000).unwrap();
    // The war goes on while they fly (the deck moves on under them).
    let fly = |m: &mut Machine, a: &mut Agent| {
        while a.t < 6000 + 600 {
            let (pu, pv) = a.at();
            crate::world_sim::step_page(m, a.t as u16, pu, pv);
            let t = a.t + 4;
            a.fly(&mut &m.reu[..], t);
        }
    };
    let hover = |u: i32, v: i32| {
        let mut a = Agent::new();
        (a.u, a.v, a.h, a.grounded, a.mode, a.t, a.yaw) = (u, v, EYE + DECK_TOP + (3 << 8), 0, MODE_PLAYER, 6000, d.yaw);
        a.input = input::DOWN;
        a
    };
    let mut a = hover((d.u as i32) << 4, (d.v as i32) << 4);
    let mut m2 = Machine::new(&m.reu);
    m2.ram.copy_from_slice(&m.ram);
    fly(&mut m2, &mut a);
    assert_eq!((a.deck, a.grounded), (ON_DECK, 1), "landed on the deck");
    // 30 texels off its beam: the sea.
    let mut b = hover(((d.u as i32) << 4) + (crate::camera::cos(d.yaw) * (30 << 8) >> 14), ((d.v as i32) << 4) - (crate::camera::sin(d.yaw) * (30 << 8) >> 14));
    fly(&mut m, &mut b);
    assert_eq!(b.deck, DITCHED, "ditched");
    let cam = b.camera().0;
    assert!(crate::combat::update(&mut m, &b, &cam), "down after ditching");
    let fx = crate::combat::FX as usize;
    assert!((0..crate::combat::FX_MAX as usize).any(|k| m.ram[fx + k * crate::combat::FX_ENTRY as usize] == crate::effects::effect::SPLASH + 1), "a splash");
}

/// The units' brain: the baked tree reproduces the planner's decision in every situation; healthy, a unit
/// attacks, against the odds too; wounded, attacking is a learned choice against retreating (the attack
/// likelier to start with); with no foes a mobile unit advances.
#[test]
fn the_units_tree_replays_the_plans() {
    use crate::brain::unit;
    use crate::roles::{Decision, decide, table, tree};
    let t = tree();
    for (s, d) in table().iter().enumerate() {
        assert_eq!(t.eval(s as u8), *d, "situation {s:06b}");
    }
    let s = |bits: &[usize]| bits.iter().fold(0u8, |a, &b| a | 1 << b);
    assert_eq!(decide(s(&[unit::MOBILE, unit::ARMED, unit::READY])), Decision::Do(unit::ADVANCE));
    assert_eq!(decide(s(&[unit::FOES, unit::MOBILE, unit::ARMED, unit::READY])), Decision::Do(unit::ATTACK), "healthy and even");
    assert_eq!(decide(s(&[unit::FOES, unit::OUTNUMBERED, unit::MOBILE, unit::ARMED, unit::READY])), Decision::Do(unit::ATTACK), "healthy, against the odds");
    for hurt in [&[unit::FOES, unit::WOUNDED, unit::MOBILE, unit::ARMED, unit::READY][..], &[unit::FOES, unit::WOUNDED, unit::OUTNUMBERED, unit::MOBILE, unit::ARMED, unit::READY]] {
        match decide(s(hurt)) {
            Decision::Choose { yes, no, p } => {
                assert_eq!((yes, no), (unit::ATTACK, unit::RETREAT));
                assert!(p > 128, "attack should start likelier: {p}");
            }
            d => panic!("wounded: {d:?}"),
        }
    }
    assert_eq!(decide(s(&[unit::FOES, unit::ARMED])), Decision::Do(unit::HOLD), "a static unit reloading holds");
    // The blob: each kind's tree walks to the same moves (learn nodes at their priors aside).
    let blob = crate::roles::compile();
    assert!(blob.len() <= crate::brain::UNIT_BYTES);
}

/// Learning: a learn node's odds move toward the branch that paid off and away from the one that did not,
/// stay inside the band, and the older walk on the trail moves half as far.
#[test]
fn learning_moves_the_odds_toward_what_paid() {
    use crate::brain::{LEARN_MAX, LEARN_MIN, learn, unit};
    use crate::hw::Hw;
    let mut m = Machine::new(package());
    let base = crate::layout::BRAIN_RAM;
    crate::hw::fetch(&mut m, crate::format::TREES, base, crate::brain::UNIT_BYTES as u16);
    // Two learn nodes of tree 0: find them by their feature.
    let blob = crate::roles::compile();
    let nodes: Vec<u16> = (crate::brain::HEADER_BYTES..blob.len()).step_by(6).filter(|&o| blob[o] >= unit::LEARN0 && blob[o] != crate::brain::LEAF).map(|o| o as u16).take(2).collect();
    assert_eq!(nodes.len(), 2);
    let p = |m: &mut Machine, n: u16| m.peek(base + n + 1);
    let (a0, b0) = (p(&mut m, nodes[0]), p(&mut m, nodes[1]));
    // Took yes on the newer, no on the older; a good outcome.
    learn(&mut m, base, 0, &[(nodes[0], 1), (nodes[1], 0)], 10);
    let (a1, b1) = (p(&mut m, nodes[0]), p(&mut m, nodes[1]));
    assert!(a1 > a0 && b1 < b0, "{a0}->{a1}, {b0}->{b1}");
    assert_eq!((a1 - a0) as i32, 2 * (b0 - b1) as i32, "the older walk at half the step");
    // Again and again: pinned to the band, never out of it.
    for _ in 0..200 {
        learn(&mut m, base, 0, &[(nodes[0], 1), (0, 0)], 15);
    }
    assert_eq!(p(&mut m, nodes[0]), LEARN_MAX);
    for _ in 0..400 {
        learn(&mut m, base, 0, &[(nodes[0], 1), (0, 0)], -15);
    }
    assert_eq!(p(&mut m, nodes[0]), LEARN_MIN);
}

/// The war teaches the brains: after twenty minutes some kinds' learn nodes have moved off their priors,
/// all within the band.
#[test]
fn the_war_teaches_the_brains() {
    use crate::brain::{LEARN_MAX, LEARN_MIN, unit};
    let mut m = Machine::new(package());
    crate::war::run(&mut m, 0, 20 * 3600, 4);
    let blob = crate::roles::compile();
    let base = crate::format::TREES as usize;
    let mut moved = 0;
    for o in (crate::brain::HEADER_BYTES..blob.len()).step_by(6).filter(|&o| blob[o] >= unit::LEARN0 && blob[o] != crate::brain::LEAF) {
        let now = m.reu[base + o + 1];
        assert!((LEARN_MIN..=LEARN_MAX).contains(&now));
        moved += (now != blob[o + 1]) as usize;
    }
    assert!(moved >= 3, "only {moved} learn nodes moved");
}

/// The effect sprites' directory: every frame's record is the sprite format's (`format::frame`), its four
/// phase blocks lie in `EFFECTS_BLOCKS..EFFECTS_BLOCKS_END`, each inside one 512 KiB REU region, no band
/// fetches more than `SPRITE_CHUNK_LEN`, and the slots past an effect's frames are empty.
#[test]
fn effect_records_point_inside_their_blocks() {
    use crate::effects::{EFFECTS, FRAMES, MAX_FRAMES, SCALES, lines};
    use crate::effects_bake::{band_chunk, read_record};
    use crate::format::{EFFECTS_BLOCKS, EFFECTS_BLOCKS_END, FRAME_RECORD, MAX_SPRITE_SIZE, REU_BANK};
    let reu = package();
    for e in 0..EFFECTS as u8 {
        for s in 0..SCALES as u8 {
            for f in 0..MAX_FRAMES as u8 {
                let at = crate::effects::record(e, s, f) as usize;
                if f >= FRAMES[e as usize] {
                    assert!(reu[at..at + FRAME_RECORD as usize].iter().all(|&b| b == 0), "effect {e} scale {s}: slot {f} not empty");
                    continue;
                }
                let r = read_record(reu, e, s, f);
                assert_eq!(r.h, lines(s), "effect {e} scale {s} frame {f}");
                assert!(r.w >= 1 && r.w <= MAX_SPRITE_SIZE && r.w <= 160 && r.h <= MAX_SPRITE_SIZE);
                assert!((r.ax as u16) < r.w && (r.ay as u16) < r.h);
                assert_eq!(r.phases, 4);
                let nbands = if r.h <= 1 << r.shift { 1 } else { r.h.div_ceil(1 << r.shift) };
                assert_eq!(r.bands as u16, nbands);
                for (p, &(wb, block, len)) in r.phase.iter().enumerate() {
                    assert_eq!(wb as u16, (r.w + p as u16).div_ceil(4));
                    let end = block + len as u32;
                    assert!(block >= EFFECTS_BLOCKS && end <= EFFECTS_BLOCKS_END, "effect {e} scale {s} frame {f} phase {p}: ${block:06X}");
                    assert_eq!(block / REU_BANK, (end - 1) / REU_BANK, "effect {e} scale {s} frame {f} phase {p} crosses a 512 KiB region");
                    let bytes = &reu[block as usize..end as usize];
                    for b in 0..nbands as usize {
                        assert!(band_chunk(bytes, b) <= crate::layout::SPRITE_CHUNK_LEN as u32);
                    }
                }
            }
        }
    }
}

/// Every phase of every effect frame, drawn back out of the image by the C64's sprite code, is exactly
/// the code map `tools/effects.py` made.
#[test]
fn effect_frames_draw_back_exactly() {
    use crate::effects::{EFFECTS, SCALES};
    let mut m = Machine::new(package());
    let src = crate::effects_bake::load();
    for e in 0..EFFECTS {
        for s in 0..SCALES {
            for (f, want) in src[e][s].frames.iter().enumerate() {
                for p in 0..4 {
                    let got = crate::effects_bake::draw_back(&mut m, e as u8, s as u8, f as u8, p);
                    assert_eq!((got.w, got.h), (want.w, want.h));
                    assert!(got.pixels == want.pixels, "effect {e} scale {s} frame {f} phase {p}");
                }
            }
        }
    }
}

/// `effects::scale_for` picks the scale nearest the projected height, clamped, nearer meaning bigger.
#[test]
fn effect_scale_follows_the_projected_height() {
    use crate::effects::{MAX_LINES, MIN_LINES, SCALES, lines, scale_for};
    let h = 48u16;
    for z in (16..20_000).step_by(7) {
        let want = (h as f64 * 133.0 / z as f64).clamp(MIN_LINES as f64, MAX_LINES as f64);
        let got = lines(scale_for(z, h)) as f64;
        assert!((got - want).abs() <= 1.0 + 1e-9, "z {z}: {got} lines for {want}");
        assert!(scale_for(z + 7, h) <= scale_for(z, h));
    }
    assert_eq!(scale_for(0, h), SCALES as u8 - 1);
    assert_eq!(scale_for(1 << 20, h), 0);
}

/// Wrecks: a unit dead near the player is drawn as its kind's wreck with a smoke column (placed with the
/// effects, no effect slot) for `WRECK_TICKS`, then no longer.
#[test]
fn wrecks_smoke_then_go() {
    use crate::agent::{Agent, MODE_PLAYER};
    use crate::combat::{self, FXDYN, OBJLIST, WRECKS};
    use crate::world_sim::{self as ws, NEAR, RECORD, WRECK_TICKS, rec};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = Agent::new();
    (a.u, a.v, a.h, a.grounded, a.mode) = (1024 << 8, 1100 << 8, 10 << 8, 0, MODE_PLAYER);
    let at = ws::UNITS as usize + 7 * RECORD as usize;
    let r = &mut m.reu[at..at + RECORD as usize];
    r.fill(0);
    r[rec::KIND as usize] = crate::actors::kind::TANK;
    r[rec::FLAGS as usize] = crate::actors::ENEMY;
    r[rec::U as usize..rec::U as usize + 2].copy_from_slice(&(1024u16 << 4).to_le_bytes());
    r[rec::V as usize..rec::V as usize + 2].copy_from_slice(&(1150u16 << 4).to_le_bytes());
    r[rec::LAST as usize..rec::LAST as usize + 2].copy_from_slice(&1000u16.to_le_bytes());
    for (i, b) in [1u8, 7, 0].into_iter().enumerate() {
        m.ram[NEAR as usize + i] = b;
    }
    let cam = camera_at(1024.0, 1100.0, 10.0, 0, -11, 0);
    a.t = 1300;
    combat::update(&mut m, &a, &cam);
    assert_eq!(m.ram[OBJLIST as usize + 1], crate::actors::wreck_of(crate::actors::kind::TANK), "drawn as a wreck");
    assert_eq!(m.ram[WRECKS as usize], 1, "smoking");
    combat::fx(&mut m, &a, &cam);
    assert_eq!(m.ram[FXDYN as usize], 1, "its smoke column placed");
    a.t = 1000 + WRECK_TICKS as u32;
    combat::update(&mut m, &a, &cam);
    assert_eq!(m.ram[WRECKS as usize], 0, "burnt out");
}

/// The player's fight: an enemy unit held in the gun's cone is hit (dust kicked up) until it explodes and
/// burns, counted as a kill; a locked missile flies to another, hits and explodes; the effects playing are
/// placed and reach the depth buckets as sprites.
#[test]
fn the_gun_and_missiles_hit_and_the_effects_play() {
    use crate::agent::{Agent, MODE_PLAYER, input};
    use crate::combat::{self, FX, FXDYN, PLAYER, player};
    use crate::world_sim::{self as ws, ALIVE, NEAR, RECORD, rec};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    // Two enemy tanks ahead (north) of a player 10 texels up at (1024, 1100), hovering, facing north.
    let mut a = Agent::new();
    (a.u, a.v, a.h, a.grounded, a.mode) = (1024 << 8, 1100 << 8, 10 << 8, 0, MODE_PLAYER);
    let put = |m: &mut Machine, id: u16, u: u16, v: u16| {
        let at = ws::UNITS as usize + id as usize * RECORD as usize;
        let r = &mut m.reu[at..at + RECORD as usize];
        r.fill(0);
        r[rec::KIND as usize] = crate::actors::kind::TANK;
        r[rec::FLAGS as usize] = ALIVE | crate::actors::ENEMY;
        r[rec::HP as usize] = 20;
        r[rec::ACTION as usize] = ws::act::HOLD;
        r[rec::U as usize..rec::U as usize + 2].copy_from_slice(&(u << 4).to_le_bytes());
        r[rec::V as usize..rec::V as usize + 2].copy_from_slice(&(v << 4).to_le_bytes());
    };
    put(&mut m, 7, 1024, 1150);
    put(&mut m, 9, 1080, 1180);
    for (i, b) in [2u8, 7, 0, 9, 0].into_iter().enumerate() {
        m.ram[NEAR as usize + i] = b;
    }
    // Nose down 11° (the cruise attitude): the tank 50 texels ahead on the ground sits in the cone.
    let cam = camera_at(1024.0, 1100.0, 10.0, 0, -11, 0);
    a.input = input::FIRE;
    let mut spawned = std::collections::HashSet::new();
    for f in 0..200u32 {
        a.t = 1000 + 4 * f;
        combat::update(&mut m, &a, &cam);
        combat::fx(&mut m, &a, &cam);
        for k in 0..combat::FX_MAX as usize {
            let id = m.ram[FX as usize + 10 * k];
            if id != 0 {
                spawned.insert(id - 1);
            }
        }
        if m.reu[ws::UNITS as usize + 7 * RECORD as usize + rec::FLAGS as usize] & ALIVE == 0 {
            break;
        }
    }
    assert_eq!(m.reu[ws::UNITS as usize + 7 * RECORD as usize + rec::FLAGS as usize] & ALIVE, 0, "the tank in the cone survived the gun");
    assert_eq!(m.ram[(PLAYER + player::KILLS) as usize], 1);
    use crate::effects::effect::{EXPLOSION, IMPACT, WRECK_FIRE};
    assert!(spawned.contains(&IMPACT) && spawned.contains(&EXPLOSION) && spawned.contains(&WRECK_FIRE), "{spawned:?}");
    // The effects in view are placed, and become sprites in the buckets.
    assert!(m.ram[FXDYN as usize] > 0, "no effect placed");
    let mut w = Box::new(crate::geom::Work::new());
    let mut v = Box::new(crate::geom::Verts::new());
    crate::geom::walk_args(&mut m, crate::layout::WALK_ARGS, &crate::geom::View { cam, pop: 0 });
    m.ram[crate::actors::DYN as usize] = 0;
    crate::geom::push_dynamic(&mut m, &mut w, &mut v);
    assert!((0..w.n).any(|i| w.tri(i).shade & crate::geom::SPRITE != 0), "no sprite pushed");
    // The other tank: locked (target 0 of the list, the first living enemy), fire 2.
    m.ram[crate::hud::LOCK_STATE as usize] = 0;
    m.ram[crate::hud::LOCK_STATE as usize + 1] = crate::hud::LOCK_TICKS;
    a.input = input::FIRE2;
    combat::update(&mut m, &a, &cam);
    assert_eq!(m.ram[(PLAYER + player::MISSILES) as usize], combat::MISSILES - 1, "no missile");
    a.input = 0;
    let hp0 = m.reu[ws::UNITS as usize + 9 * RECORD as usize + rec::HP as usize];
    for f in 0..200u32 {
        a.t += 4;
        let _ = f;
        combat::update(&mut m, &a, &cam);
        combat::fx(&mut m, &a, &cam);
    }
    let hp = m.reu[ws::UNITS as usize + 9 * RECORD as usize + rec::HP as usize];
    assert!(hp < hp0, "the missile missed: hp {hp0} -> {hp}");
}

/// Near-plane clipping: a face reaching behind the camera is cut at the plane and still drawn, one
/// triangle with one corner in front, two with two; the cut corners land where the exact projection of
/// the edge's crossing does (to the plane's Q4 precision).
#[test]
fn faces_reaching_behind_the_camera_are_cut_at_the_near_plane() {
    use crate::geom::{NEAR, Verts, Work, clip, vert};
    let q = 16.0f64; // Q4
    let proj = |x: f64, y: f64, z: f64| (80.0 * 16.0 + 1280.0 * x / z, 68.0 * 16.0 - 2128.0 * y / z);
    // Camera space, texels: corners at depth 10, 6 and −4 (behind).
    let corners = [(-3.0f64, -1.0f64, 10.0f64), (4.0, -1.5, 6.0), (1.0, -2.0, -4.0)];
    for (behind, want) in [(1usize, 2u8), (2, 1)] {
        let mut v = Verts::new();
        let mut c = corners;
        if behind == 2 {
            c[1].2 = -3.0;
        }
        for (i, &(x, y, z)) in c.iter().enumerate() {
            v.set(vert::X, i, (x * q) as i16);
            v.set(vert::Y, i, (y * q) as i16);
            v.set(vert::Z, i, (z * q) as i16);
            if z * q >= NEAR as f64 {
                let (sx, sy) = proj(x, y, z);
                v.set(vert::SX, i, sx as i16);
                v.set(vert::SY, i, sy as i16);
            }
        }
        let mut w = Work::new();
        w.clear();
        w.clip[0] = 0;
        w.nclip = 1;
        // Both windings: one of them faces the camera.
        for face in [[0u8, 1, 2, 7], [0, 2, 1, 7]] {
            clip(&face, &mut v, &mut w);
        }
        assert_eq!(w.n, want, "{behind} corner(s) behind: triangles pushed");
        // Every pushed corner is a corner in front or an exact crossing of the near plane.
        let near = NEAR as f64 / q;
        let mut points = Vec::new();
        for (i, &(x, y, z)) in c.iter().enumerate() {
            if z >= near {
                points.push(proj(x, y, z));
            }
            let (x2, y2, z2) = c[(i + 1) % 3];
            if (z >= near) != (z2 >= near) {
                let t = (z - near) / (z - z2);
                points.push(proj(x + (x2 - x) * t, y + (y2 - y) * t, near));
            }
        }
        for k in 0..w.n {
            let tri = w.tri(k);
            for j in 0..3 {
                let (sx, sy) = (tri.x[j] as f64, tri.y[j] as f64);
                // (A cut corner is a Q4 camera-space point: at the 2-texel plane one Q4 step is 2.5
                // pixels across, 66.5/16 down.)
                assert!(points.iter().any(|&(px, py)| (px - sx).abs() <= 48.0 && (py - sy).abs() <= 80.0), "corner ({sx}, {sy}) not on the cut face: {points:?}");
            }
        }
    }
}

/// The waypoint arrow near dead astern: the camera's smallest steps either side of it (the carrier's yaw
/// steps, as at the start on its deck) keep the arrow on one side; a clear turn moves it across.
#[test]
fn the_waypoint_arrow_keeps_its_side_near_dead_astern() {
    use crate::hud::{S_SHAPES, build, init, shape, slot};
    let mut m = Machine::new(package());
    init(&mut m);
    let mut a = crate::agent::Agent::new();
    // The waypoint 300 texels due south; the helicopter facing north (yaw 0), so it is dead astern.
    (a.u, a.v, a.wu, a.wv, a.t) = (1000 << 8, 1000 << 8, 1000, 700, 9000);
    let arrow = |m: &mut Machine, a: &crate::agent::Agent| {
        build(m, a, &a.camera().0);
        m.ram[(S_SHAPES + slot::CARET as u16) as usize]
    };
    // Clearly to one side first, then near dead astern.
    a.yaw = 40 * 256;
    let first = arrow(&mut m, &a);
    assert!([shape::ARROW + 2, shape::ARROW + 6].contains(&first), "a side arrow: {first}");
    for k in 0..40 {
        a.yaw = if k % 2 == 0 { 0xFF00 } else { 0x0100 };
        a.u += 3 * (k % 3 - 1) << 4;
        assert_eq!(arrow(&mut m, &a), first, "flipped at step {k}");
    }
    // A clear turn the other way: the arrow crosses over.
    a.yaw = 0u16.wrapping_sub(40 * 256);
    assert_ne!(arrow(&mut m, &a), first);
}

/// Replaying the start on the carrier's deck and the autopilot's flight (the war stepped every frame as
/// on the C64, 8 ticks a frame): the living units listed and on screen in one frame and dropped from the
/// near list in the next while still alive and on screen, and the frames whose draw list reached the
/// actors' cap.
fn near_churn(seconds: u32) -> (u32, u32) {
    use crate::hud::{Eye, to_camera};
    use crate::world_sim::{self as ws, ALIVE, rec};
    let mut m = Machine::new(package());
    crate::combat::init(&mut m);
    let mut a = crate::agent::Agent::boot();
    let (mut t, mut prev, mut dropped, mut frames) = (0u32, Vec::<u16>::new(), 0u32, 0u32);
    let (mut capped, mut most) = (0u32, 0u8);
    while t < seconds * 60 {
        let (pu, pv) = a.at();
        ws::step_page(&mut m, t as u16, pu, pv);
        t += 8;
        a.fly(&mut &m.reu[..], t);
        let eye = Eye::new(&a.camera().0);
        let on_screen = |m: &Machine, id: u16| {
            let r: &[u8; ws::CARRIER_RECORD] = m.reu[ws::UNITS as usize + id as usize * ws::RECORD as usize..][..ws::CARRIER_RECORD].try_into().unwrap();
            if r[rec::FLAGS as usize] & ALIVE == 0 {
                return false;
            }
            let (u, v, _) = ws::live(r, t as u16);
            let c = to_camera(&eye, [(u >> 2) as i16, (r[rec::H as usize] as i16) << 2, (v >> 2) as i16]);
            c[2] > 8 && c[2] < 200 * 4 && c[0].abs() < c[2]
        };
        // The draw list as the C64 builds it: the combat pass, then the actors' cull (`DYN_MAX` at most).
        let cam = a.camera().0;
        crate::combat::update(&mut m, &a, &cam);
        let objects = m.ram[crate::combat::OBJLIST as usize];
        crate::actors::prepare(&mut m, &cam, crate::combat::OBJLIST);
        let drawn = m.ram[crate::actors::DYN as usize];
        capped += (drawn == crate::actors::DYN_MAX) as u32;
        most = most.max(objects);
        let n = m.ram[ws::NEAR as usize].min(ws::NEAR_MAX) as usize;
        let listed: Vec<u16> = (0..n).map(|k| u16::from_le_bytes([m.ram[ws::NEAR as usize + 1 + 2 * k], m.ram[ws::NEAR as usize + 2 + 2 * k]])).collect();
        dropped += prev.iter().filter(|&&id| !listed.contains(&id) && on_screen(&m, id)).count() as u32;
        prev = listed.into_iter().filter(|&id| on_screen(&m, id)).collect();
        frames += 1;
    }
    eprintln!("draw list: {most} objects at most; the actors' cap of {} reached in {capped} frames", crate::actors::DYN_MAX);
    (dropped, capped)
}

/// Ten minutes from the start on the deck: no unit on screen is ever dropped from the near list, and
/// the actors' cap never cuts the draw list (units seen to vanish were faces crossing the near plane,
/// now cut instead of dropped).
#[test]
fn units_on_screen_stay_listed() {
    let (dropped, capped) = near_churn(600);
    assert_eq!((dropped, capped), (0, 0), "units on screen dropped from the near list; frames at the actors' cap");
}
