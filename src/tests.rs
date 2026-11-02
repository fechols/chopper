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

/// The autopilot's view at `tick`.
fn lap_view(tick: u16) -> crate::geom::View {
    crate::geom::View { cam: crate::pilot::autopilot(tick).0, pop: 0 }
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
            let (m, _) = crate::package::render(reu, tick, doubled);
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
    for tick in (0..crate::pilot::LAP).step_by(256) {
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
    use crate::geom::{Look, SPAN, View, pop_depth, sphere_visible};
    use crate::format::{OBJ_RECORDS};
    let reu = package();
    let block = scene(reu);
    let n = {
        let w = crate::world::generate(1983);
        crate::scene::objects(&w).len()
    };
    let records = &reu[OBJ_RECORDS as usize..][..8 * n];
    let mut rng = Rng(0xC0FFEE);
    let mut total = 0u32;
    for case in 0..400 {
        let cam = if case % 2 == 0 {
            crate::pilot::autopilot(rng.below(crate::pilot::LAP as u32) as u16).0
        } else {
            camera_at(300.0 + rng.below(1450) as f64, 300.0 + rng.below(1450) as f64, 2.0 + rng.below(60) as f64, rng.next() as u16, rng.below(70) as i32 - 50, rng.below(121) as i32 - 60)
        };
        let pop = [0u8, 1, 2, 4, 8][rng.below(5) as usize];
        let view = View { cam, pop };
        let look = Look::new(&view);
        let mut want = 0u32;
        for rec in records.chunks(8) {
            let (ou, ov) = (u16::from_le_bytes([rec[0], rec[1]]), u16::from_le_bytes([rec[2], rec[3]]));
            let oy = i16::from_le_bytes([rec[4], rec[5]]);
            let (dx, dz) = ((((ou as i32) << 3) - cam.u) >> 4, (((ov as i32) << 3) - cam.v) >> 4);
            if dx.abs() > SPAN || dz.abs() > SPAN {
                continue;
            }
            let rad = block[8 + rec[7] as usize * 10 + 7];
            let c = look.to_camera([dx as i16, oy.wrapping_sub(look.y), dz as i16]);
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
    for tick in (0..crate::pilot::LAP).step_by(128) {
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
