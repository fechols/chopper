use crate::camera::{self, View};
use crate::cost::Cost;
use crate::hw;
use crate::layout::{BUFFERS, CACHE};
use crate::map;
use crate::mode7 as m7;
use crate::model::Machine;
use std::sync::OnceLock;

fn plane() -> &'static (Vec<u8>, Vec<u8>) {
    static P: OnceLock<(Vec<u8>, Vec<u8>)> = OnceLock::new();
    P.get_or_init(|| {
        let codes = map::generate();
        let image = map::bake(&codes);
        (codes, image)
    })
}

#[test]
fn the_cost_model_reproduces_talonhds_bench() {
    // talonhd docs/validation.md: REU fetch of 1 / 8 / 64 / 512 bytes.
    for (n, measured) in [(1u32, 9.8f32), (8, 12.3), (64, 26.5), (512, 130.7)] {
        let c = Cost { io_writes: 8, dma_cmds: 1, dma_bytes: n, cycles: 0 };
        let err = (c.us() - measured).abs() / measured;
        assert!(err < 0.10, "{n} B: model {:.1} us vs measured {measured} us", c.us());
    }
}

#[test]
fn the_dda_stays_within_a_texel_of_the_exact_transform() {
    for view in [
        View::Roto { x: 100.3, y: 77.7, angle: 0.52, zoom: 0.37 },
        View::Floor { x: 40.0, y: 200.0, angle: 2.1, height: 12.0, horizon: 48, focal: 160.0 },
    ] {
        let f = camera::frame(&view);
        for (l, e) in f.lines.iter().zip(&f.exact) {
            let Some((p, st)) = e else { continue };
            for x in 0..160u16 {
                for (q, step, exact) in [(l.u0, l.du, p[0] + x as f32 * st[0]), (l.v0, l.dv, p[1] + x as f32 * st[1])] {
                    let dda = q.wrapping_add((step as u16).wrapping_mul(x)) as f32 / 256.0;
                    let d = (dda - exact.rem_euclid(256.0)).rem_euclid(256.0);
                    assert!(d.min(256.0 - d) <= 1.0, "{view:?}: pixel {x} off by {d}");
                }
            }
        }
    }
}

#[test]
fn every_path_matches_the_reference_at_one_texel_per_byte() {
    // Angle 0, 4 multicolour pixels per texel, integer start: all paths sample the same texels.
    let (codes, image) = plane();
    let f = camera::frame(&View::Roto { x: 120.0, y: 90.0, angle: 0.0, zoom: 0.125 });
    let want = camera::reference(&f, codes);
    let buf = BUFFERS[0];
    for path in 0..3 {
        let mut m = Machine::new(image);
        match path {
            0 => drop(m7::per_byte_dma(&mut m, buf, &f.lines, m7::DmaMode::RAW)),
            1 => drop(m7::per_byte_dma(&mut m, buf, &f.lines, m7::DmaMode::PRE)),
            _ => {
                m7::fill_cache(&mut m, f.window.0, f.window.1);
                m7::cpu_gather(&mut m, buf, &f.lines, 4);
            }
        }
        let got = m.codes(buf);
        let bad = got.iter().zip(&want).filter(|(a, b)| a != b).count();
        assert_eq!(bad, 0, "path {path}: {bad} pixels differ");
        assert_eq!((m.stats.stray_writes, m.stats.bank_wraps), (0, 0));
    }
}

#[test]
fn per_byte_dma_fetches_every_byte_once_and_skips_unchanged_registers() {
    let (_, image) = plane();
    let f = camera::frame(&View::Roto { x: 120.0, y: 90.0, angle: 0.0, zoom: 0.125 });
    let mut m = Machine::new(image);
    let c = m7::per_byte_dma(&mut m, BUFFERS[0], &f.lines, m7::DmaMode::RAW);
    assert_eq!((c.dma_cmds, c.dma_bytes, m.stats.commands), (8000, 8000, 8000));
    // At most lo + mid + bank + command per byte, plus the 4 setup writes; at this zoom far fewer.
    assert!(c.io_writes < 4 + 8000 * 3, "{} I/O writes", c.io_writes);
}

#[test]
fn the_cache_holds_the_window() {
    let (codes, image) = plane();
    let mut m = Machine::new(image);
    let (wu, wv) = (200u8, 250u8);
    m7::fill_cache(&mut m, wu, wv);
    for dv in 0..128u8 {
        for du in 0..128u8 {
            let (u, v) = (wu.wrapping_add(du), wv.wrapping_add(dv));
            let got = m.ram[(CACHE + (v as u16 & 127) * 128 + (u as u16 & 127)) as usize];
            assert_eq!(got, codes[v as usize * 256 + u as usize] * 0x55, "texel ({u}, {v})");
        }
    }
    let _ = hw::reu::FETCH;
}

fn package() -> &'static Vec<u8> {
    static P: OnceLock<Vec<u8>> = OnceLock::new();
    P.get_or_init(|| crate::package::bake(&plane().0))
}

#[test]
fn line_tables_plus_the_camera_track_the_exact_floor() {
    use crate::format::*;
    let reu = package();
    for tick in [0u16, 300, 777, 1500] {
        let lines = crate::package::frame_lines(reu, tick);
        let e = HEADER as usize + H_PATH as usize + tick as usize * PATH_ENTRY as usize;
        let (x, y, a) = (u16::from_le_bytes([reu[e], reu[e + 1]]), u16::from_le_bytes([reu[e + 2], reu[e + 3]]), reu[e + 4]);
        let f = camera::frame(&View::Floor {
            x: x as f32 / 256.0,
            y: y as f32 / 256.0,
            angle: a as f32 / 256.0 * std::f32::consts::TAU,
            height: CAMERA_HEIGHT,
            horizon: HORIZON,
            focal: FOCAL,
        });
        let mut drawn = 0;
        for (l, e) in lines.iter().zip(&f.exact) {
            if l.sky || l.bank == FOG_BANK {
                continue;
            }
            let (p, st) = e.expect("a drawn line is a floor line");
            drawn += 1;
            for x in [0u16, 80, 159] {
                for (q, step, exact) in [(l.u0, l.du, p[0] + x as f32 * st[0]), (l.v0, l.dv, p[1] + x as f32 * st[1])] {
                    let dda = q.wrapping_add((step as u16).wrapping_mul(x)) as f32 / 256.0;
                    let d = (dda - exact.rem_euclid(256.0)).rem_euclid(256.0);
                    assert!(d.min(256.0 - d) <= 1.0, "tick {tick}: pixel {x} off by {d}");
                }
            }
        }
        assert!(drawn > 140, "tick {tick}: only {drawn} floor lines drawn");
    }
}

#[test]
fn doubled_mode_repeats_each_even_line() {
    let reu = package();
    let (full, _) = crate::package::render_floor(reu, 640, false);
    let (dbl, cost) = crate::package::render_floor(reu, 640, true);
    assert_eq!(cost.dma_cmds, 40 * (200 - crate::format::HORIZON as u32) / 2);
    let (a, b) = (full.codes(BUFFERS[0]), dbl.codes(BUFFERS[0]));
    for y in 0..200usize {
        let src = if y >= crate::format::HORIZON as usize { y & !1 } else { y };
        assert_eq!(&b[y * 160..][..160], &a[src * 160..][..160], "line {y}");
    }
    assert_eq!(dbl.stats.stray_writes, 0);
}

#[test]
fn the_package_has_its_header_and_path() {
    use crate::format::*;
    let reu = package();
    let h = HEADER as usize;
    assert_eq!(&reu[h..h + 8], MAGIC);
    assert_eq!(u16::from_le_bytes([reu[h + 8], reu[h + 9]]), PACKAGE_VERSION);
    assert_eq!(u16::from_le_bytes([reu[h + 10], reu[h + 11]]), PATH_LEN);
    // Consecutive path frames move a little and turn by at most a step.
    let path = crate::package::path();
    for w in path.windows(2) {
        let d = (w[1].0.wrapping_sub(w[0].0) as i16).unsigned_abs() + (w[1].1.wrapping_sub(w[0].1) as i16).unsigned_abs();
        assert!(d < 256, "jump of {d}/256 texels");
        assert!((w[1].2.wrapping_sub(w[0].2) as i8).unsigned_abs() <= 2);
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
    for tick in [0u16, 512, 1024, 1536] {
        for doubled in [false, true] {
            let (m, _, _) = crate::package::render(reu, tick, doubled);
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
        for y in 0..200 {
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
    use crate::geom::{Camera, Work, draw};
    use crate::scene::{Material, Mesh, Object, bake_with};
    // Camera at (128, 128) looking along -v (angle 0); a wall 20 texels ahead.
    for (toward, want) in [(1.0f32, 2u8), (-1.0, 0)] {
        let mut mesh = Mesh::default();
        mesh.quad([[-4.0, 0.0, 0.0], [4.0, 0.0, 0.0], [4.0, 0.0, 8.0], [-4.0, 0.0, 8.0]], [0.0, toward, 0.0], Material::Red);
        let (scene, records) = bake_with(&[mesh], &[Object { x: 128.0, y: 108.0, yaw: 0, mesh: 0 }]);
        let mut image = vec![0u8; crate::format::REU_SIZE as usize];
        crate::scene::install(&mut image, &scene, &records);
        let mut m = Machine::new(&image);
        let mut work = Box::new(Work::new());
        draw(&mut m, &scene, Camera { x: 128 << 8, y: 128 << 8, angle: 0, pop: crate::geom::POP }, BUFFERS[0].bitmap, &mut work, &mut Box::new(crate::geom::Verts::new()));
        let (tris, _, _) = crate::geom::raster_bands(&mut m, &work, BUFFERS[0].bitmap);
        assert_eq!(tris, want, "normal along v {toward}");
    }
}

#[test]
fn the_geometry_frames_draw_inside_ram() {
    let reu = package();
    let mut total = 0u32;
    for tick in (0..2048u16).step_by(128) {
        let (m, _, st) = crate::package::render(reu, tick, false);
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
fn projection_matches_the_floor_camera() {
    use crate::geom::project;
    // A ground point straight ahead at distance d lands on the line the floor puts there.
    for d in [8.0f64, 13.0, 20.0, 50.0, 100.0] {
        let (sx, sy) = project(0, 0, (d * 16.0) as i16);
        let want = 48.0 + 160.0 * 12.0 / d;
        assert!((sx as f64 / 16.0 - 80.0).abs() < 0.07, "{d}: x {sx}");
        assert!((sy as f64 / 16.0 - want).abs() < 0.2, "{d}: y {} want {want}", sy as f64 / 16.0);
    }
}

#[test]
fn the_grid_visits_every_object_the_cull_accepts() {
    // Brute force: every object of the scene against the per-object cull, at many cameras; each one the
    // cull accepts must sit in a cell of that angle's visibility list (or the global list).
    use crate::geom::{CELL_PAD, NEAR, Q, cell_of, cos, pop_depth, pop_scale, sin, visible_cells};
    let (block, records) = crate::scene::bake();
    let reu = package();
    let mut rng = Rng(0xC0FFEE);
    let lists: Vec<Vec<(i8, i8, u8)>> = (0..=255u8).map(visible_cells).collect();
    let dir = u16::from_le_bytes([block[4], block[5]]) as usize;
    let global = (u16::from_le_bytes([block[dir + 1024], block[dir + 1025]]) as usize, block[dir + 1026] as usize);
    for case in 0..3000 {
        let cam = if case % 2 == 0 {
            crate::package::camera(reu, rng.below(2048) as u16)
        } else {
            crate::geom::Camera { x: rng.next() as u16, y: rng.next() as u16, angle: rng.next() as u8, pop: 0 }
        };
        let (cx, cy) = cell_of(cam.x, cam.y);
        let (c, s) = (cos(cam.angle), sin(cam.angle));
        // A random pop-in size; cells the walk skips for it must hold nothing the cull accepts.
        let pop = [1u8, 2, 4, 6, 10, 20, 80, 255][rng.below(8) as usize];
        let scale = pop_scale(pop);
        for (k, rec) in records.chunks(6).enumerate() {
            let (ox, oy, mesh) = (u16::from_le_bytes([rec[0], rec[1]]), u16::from_le_bytes([rec[2], rec[3]]), rec[5] as usize);
            let rt = block[8 + mesh * 10 + 7];
            let r = rt as i32 * Q;
            let dx = (ox.wrapping_sub(cam.x) as i16 >> 4) as i32;
            let dy = (oy.wrapping_sub(cam.y) as i16 >> 4) as i32;
            if dx.abs() > crate::geom::SPAN || dy.abs() > crate::geom::SPAN {
                continue;
            }
            let x0 = (dx * c as i32 + dy * s as i32) >> 14;
            let z0 = (dx * s as i32 - dy * c as i32) >> 14;
            if z0 + r < NEAR || z0 - r > pop_depth(rt, scale) || x0.abs() - r > z0 + r {
                continue;
            }
            if (global.0..global.0 + global.1).contains(&k) {
                continue;
            }
            assert!(r <= CELL_PAD * Q);
            let (ocx, ocy) = cell_of(ox, oy);
            let off = (((ocx as i8 - cx as i8 + 8).rem_euclid(16) - 8), ((ocy as i8 - cy as i8 + 8).rem_euclid(16) - 8));
            let entry = lists[cam.angle as usize].iter().find(|e| (e.0, e.1) == off);
            let Some(&(_, _, near)) = entry else { panic!("case {case}: object {k} at cell offset {off:?} accepted but not listed ({cam:?})") };
            let at = dir + 4 * (ocy as usize * 16 + ocx as usize);
            let rmax = block[at + 3];
            let walked = (near as i32) << 4 <= pop_depth(rmax, scale) + ((rmax as i32) << 4);
            assert!(walked, "case {case}: object {k} accepted but its cell (nearest {near}, largest radius {rmax}) skipped at pop {pop}");
        }
    }
    let longest = lists.iter().map(|l| l.len()).max().unwrap();
    assert!(longest < 200, "{longest} cells in a list");
}

#[test]
fn band_rendering_equals_whole_triangle_painting() {
    // Per pixel, drawing band by band (each triangle one row per band, in painter's order) must equal
    // painting whole triangles back to front, on real lap frames.
    use crate::geom::{Verts, Work, draw, raster, raster_bands};
    let reu = package();
    let buf = BUFFERS[0];
    let mut checked = 0;
    for tick in (0..2048u16).step_by(64) {
        let (mut a, _) = crate::package::render_floor(reu, tick, false);
        let mut work = Box::new(Work::new());
        draw(&mut a, crate::package::scene(reu), crate::package::camera(reu, tick), buf.bitmap, &mut work, &mut Box::new(Verts::new()));
        let mut b = Machine::new(&[]);
        b.ram.copy_from_slice(&a.ram);
        let (whole, top_a) = raster(&mut a, &work, buf.bitmap);
        let (banded, top_b, dropped) = raster_bands(&mut b, &work, buf.bitmap);
        assert_eq!(dropped, 0, "tick {tick}");
        assert_eq!((whole, top_a), (banded, top_b), "tick {tick}");
        assert_eq!(a.ram[buf.bitmap as usize..][..8000], b.ram[buf.bitmap as usize..][..8000], "tick {tick}");
        checked += whole as u32;
    }
    assert!(checked > 500, "only {checked} triangles");
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
    let (tris, top, dropped) = raster_bands(&mut m, &w, BUFFERS[0].bitmap);
    assert_eq!((tris as usize, dropped as usize, top), (POOL, 120 - POOL, 3));
}
