use crate::floor::{self, COLS, FloorCam, Lines, MAX_SPANS, Rays, Span, VIEW_LINES};
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
    let (tris, dropped, _) = crate::geom::raster_bands(&mut m, &work, BUFFERS[0].bitmap);
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

/// A little-endian word of RAM.
fn rd16(m: &Machine, a: u16) -> u16 {
    u16::from_le_bytes([m.ram[a as usize], m.ram[a as usize + 1]])
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
            let d = [((4 * b + 2) as f64 - 80.0) / floor::FX as f64, (crate::geom::CY as f64 - (y as f64 + 0.5)) / floor::FY as f64, 1.0];
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
            let n = floor::make_line(&cam, &rays, &rays.line(y), floor::HAZE_BYTE, rays.fog().at(y), &mut spans, &mut floor::Patches::default()).0;
            let exact = exact_line(&cam, y);
            let mut b = 0usize;
            for (i, s) in spans[..n].iter().enumerate() {
                // (The last run is cut one byte short: `floor::DRAWN`.)
                assert!(s.len.is_power_of_two() || s.fill().is_some() || i + 1 == n && (s.len + 1).is_power_of_two(), "{s:?}");
                if s.fill().is_none() {
                    // (Map coordinates: the level's place in its bank, `floor::MIP_OFFSET`, taken off.)
                    let off = (floor::MIP_OFFSET[s.level() as usize] as i32) << 8;
                    let (mut u, mut v) = (s.u - off, s.v - off);
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
            assert_eq!(b, floor::DRAWN as usize);
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
        let level = level_for(du, dv).unwrap_or(floor::LEVELS - 1);
        out[n] = Span {
            len,
            flags: level,
            u: ((cam.u + p0.0) >> level) + ((floor::MIP_OFFSET[level as usize] as i32) << 8),
            v: ((cam.v + p0.1) >> level) + ((floor::MIP_OFFSET[level as usize] as i32) << 8),
            du: (du >> level) as i16,
            dv: (dv >> level) as i16,
        };
        n += 1;
        b += len;
    }
    out[n - 1].len -= 1;
    if out[n - 1].len == 0 {
        n -= 1;
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
            let n = floor::make_line(&cam, &rays, &a, floor::HAZE_BYTE, rays.fog().at(line), &mut x, &mut floor::Patches::default()).0;
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
        let n = floor::make_line(&cam, &rays, &rays.line(y), floor::HAZE_BYTE, rays.fog().at(y), &mut spans, &mut floor::Patches::default()).0;
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
    for k in 4..floor::LEVELS {
        assert_eq!(floor::tile_bank(k, 0, 0), LEVEL_BANK[k as usize]);
        assert_eq!(floor::tile_bank(k, 0, 1), SEA_BANK);
        assert_eq!(floor::tile_bank(k, 0xFF, 0), SEA_BANK);
    }
    // A texel's address in the image holds that texel.
    let reu = package();
    let w = crate::world::generate(1983);
    for (u, v) in [(0usize, 0usize), (255, 256), (1024, 760), (2047, 2047), (1500, 33)] {
        let a = floor::texel_addr(0, (u as i32) << 8, (v as i32) << 8) as usize;
        assert_eq!(reu[a], crate::world::dither(w.texels[v * 2048 + u], u, v), "texel ({u}, {v})");
    }
    // Every level's texels where a run reads them (`floor::MIP_OFFSET` added), and the sea just off the
    // map at the shared-bank levels (out to 60 texels on every side).
    let mips = crate::world::mips(&w.texels);
    for k in 1..floor::LEVELS {
        let n = 2048i32 >> k;
        let off = (floor::MIP_OFFSET[k as usize] as i32) << 8;
        let at = |u: i32, v: i32| reu[floor::texel_addr(k, (u << 8) + off, (v << 8) + off) as usize];
        for (u, v) in [(0, 0), (n - 1, n - 1), (n / 2, 3), (5, n - 2)] {
            let want = crate::world::dither(mips[k as usize][(v * n + u) as usize], u as usize, v as usize);
            assert_eq!(at(u, v), want, "level {k} texel ({u}, {v})");
        }
        if k >= 4 {
            for d in [1, 20, 60] {
                for (u, v) in [(-d, n / 2), (n - 1 + d, n / 2), (n / 2, -d), (n / 2, n - 1 + d), (-d, -d), (n - 1 + d, n - 1 + d)] {
                    assert_eq!(at(u, v), crate::world::texel::WATER, "level {k} off the map at ({u}, {v})");
                }
            }
        }
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
        for lines in [Lines::Full, Lines::Doubled, Lines::Adaptive] {
            let (m, _) = crate::package::render(reu, lap_t(tick), lines);
            assert_eq!((m.stats.bank_wraps, m.stats.stray_writes), (0, 0), "tick {tick} {lines:?}");
        }
    }
}

/// `floor::double_mask`'s pairs, a byte per band, as it writes them to `layout::DOUBLE_MASK`.
fn adaptive_pairs(m: &mut Machine, cam: &FloorCam) -> [u8; floor::VIEW_BANDS as usize] {
    use crate::hw::Hw;
    floor::double_mask(m, cam);
    core::array::from_fn(|b| m.peek(crate::layout::DOUBLE_MASK + b as u16))
}

/// `floor::double_mask` against the texel steps worked out from the rays themselves: for every byte of
/// a line, `∂P/∂y` and `∂P/∂b` (the ground point's steps a line and a byte, up to their common factor
/// `H / w²`) from `D = a0 + y·ay + b·bb`, in the maximum norm. A pair the mask draws once has every
/// byte of both lines within the budget (`DBL_NUM · ρ ≤ DBL_DEN / 2`: a line steps at most 1.5·ρ, one
/// texel of the level read at 3/4); a pair it draws twice has a line over it (both up to the Q7 rounding).
#[test]
fn the_adaptive_pairs_are_the_ones_whose_lines_repeat_a_texel() {
    use crate::floor::{COLS, DBL_DEN, DBL_NUM};
    let mut m = Machine::new(&[]);
    let mut rng = Rng(0x5eed_d0b1);
    let budget = DBL_DEN as f64 / (2.0 * DBL_NUM as f64);
    let (mut pairs, mut doubled) = (0u32, 0u32);
    for i in 0..400 {
        let yaw = rng.next() as u16;
        let pitch = rng.below(120) as i32 - 90;
        let roll = if i % 4 == 0 { 0 } else { rng.below(361) as i32 - 180 };
        let cam = camera_at(1000.0, 1000.0, 2.0 + rng.below(100) as f64, yaw, pitch, roll);
        let rays = Rays::new(&cam);
        let mask = adaptive_pairs(&mut m, &cam);
        let f = |v: [i32; 3]| v.map(|c| c as f64);
        let (a0, ay, bb) = (f(rays.a0), f(rays.ay), f(rays.bb));
        // The worst ρ of line y over its bytes (byte COLS − 1 too: the mask is planned over COLS).
        let rho = |y: u16| -> f64 {
            (0..COLS)
                .map(|b| {
                    let d: [f64; 3] = core::array::from_fn(|k| a0[k] + y as f64 * ay[k] + b as f64 * bb[k]);
                    let w = -d[1];
                    let step = |s: [f64; 3]| (s[0] * w + d[0] * s[1]).abs().max((s[2] * w + d[2] * s[1]).abs());
                    step(ay) / step(bb)
                })
                .fold(0.0, f64::max)
        };
        for y in (0..floor::VIEW_LINES).step_by(2) {
            let worst = rho(y).max(rho(y + 1));
            let on = mask[(y / 8) as usize] >> ((y % 8) / 2) & 1 != 0;
            pairs += 1;
            doubled += on as u32;
            if on {
                assert!(worst <= budget * 1.03, "pitch {pitch} roll {roll} yaw {yaw}: pair {y} doubled at ρ {worst:.4}");
            } else {
                assert!(worst > budget * 0.97, "pitch {pitch} roll {roll} yaw {yaw}: pair {y} not doubled at ρ {worst:.4}");
            }
        }
    }
    println!("{doubled} of {pairs} pairs drawn once");
}

#[test]
fn the_adaptive_pairs_follow_pitch_and_roll() {
    let mut mm = Machine::new(&[]);
    let mut double_mask = |cam: &FloorCam| adaptive_pairs(&mut mm, cam);
    let count = |m: [u8; 16]| m.iter().map(|b| b.count_ones()).sum::<u32>();
    let bit = |m: &[u8; 16], p: usize| m[p / 4] >> (p % 4) & 1;
    // (Yaw 0: the texel grid square to the view. The maximum norm, as the mip levels use it, makes the
    // budget tighter turned toward a diagonal: checked last.)
    let mut at = |pitch, roll| double_mask(&camera_at(1000.0, 1000.0, 30.0, 0, pitch, roll));
    // Without roll the ratio grows toward the horizon (on both sides: far up in the sky the lines are all
    // fill), so the ground's doubled pairs are the bottom ones, from the first that passes down, and the
    // pairs around the horizon are drawn line by line. (Level flight: the horizon is at line 67.)
    let ground = |m: &[u8; 16], from: usize| -> u32 {
        let first = (from..64).find(|&p| bit(m, p) != 0).unwrap_or(64);
        assert!((first..64).all(|p| bit(m, p) != 0), "the ground's doubled pairs from pair {first} down: {m:x?}");
        64 - first as u32
    };
    let level = at(0, 0);
    assert_eq!(level[0], 0xF, "level flight: the top band, high in the sky");
    assert!(bit(&level, 33) == 0 && bit(&level, 34) == 0, "level flight: the horizon's pairs: {level:x?}");
    let m = at(-20, 0);
    let (l0, l20, l45) = (ground(&level, 34), ground(&m, 0), ground(&at(-45, 0), 0));
    assert!(l0 < l20 && l20 < l45, "more pairs pitched down: {l0}, {l20}, {l45}");
    assert!(l45 >= 50, "pitched 45° down: most pairs");
    assert_eq!(count(at(-60, 0)), 64, "pitched 60° down: all");
    for roll in [90, -90] {
        assert!(count(at(-10, roll)) >= 56, "rolled {roll}°: the lines run along the depth");
    }
    let diag = count(double_mask(&camera_at(1000.0, 1000.0, 30.0, 0x2000, -20, 0)));
    // (Turned 45°, a line steps along the diagonal, where `level_for` reads a finer level, and the edge
    // bytes' rays run along the texel axes: the budget is spent sooner.)
    assert!(diag < count(m), "turned 45°: fewer pairs ({diag})");
}

/// Adaptive frames against full ones over the lap: how many lines they skip, and how many of the view's
/// bytes differ from the full frame's (only the skipped lines can).
#[test]
fn adaptive_frames_skip_lines_that_barely_change() {
    let reu = package();
    let (mut full_dmas, mut ada_dmas, mut lines, mut frames) = (0u32, 0u32, 0u32, 0u32);
    let (mut differ, mut compared) = (0u32, 0u32);
    for tick in [0u16, 512, 1024, 1536, 2048, 3000, 4096, 6000] {
        let (mf, sf) = crate::package::render(reu, lap_t(tick), Lines::Full);
        let (ma, sa) = crate::package::render(reu, lap_t(tick), Lines::Adaptive);
        full_dmas += sf.dmas;
        ada_dmas += sa.dmas;
        lines += sa.drawn_lines;
        frames += 1;
        let buf = crate::layout::BUFFERS[0];
        for band in 0..floor::VIEW_BANDS {
            for i in 0..crate::layout::DRAWN_COLS * 8 {
                let a = buf.bitmap + band * crate::layout::ROW_STRIDE + i;
                compared += 1;
                differ += (mf.ram[a as usize] != ma.ram[a as usize]) as u32;
            }
        }
    }
    println!(
        "adaptive: {lines} of {} lines drawn, {ada_dmas} of {full_dmas} DMAs ({:.1}%), {differ} of {compared} view bytes differ ({:.2}%)",
        frames * floor::VIEW_LINES as u32,
        100.0 * ada_dmas as f64 / full_dmas as f64,
        100.0 * differ as f64 / compared as f64
    );
    assert!(ada_dmas <= full_dmas);
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
            // (The drawn columns: the last never shows.)
            for x in 0..4 * crate::layout::DRAWN_COLS as usize {
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
        let (tris, _, _) = crate::geom::raster_bands(&mut m, &work, BUFFERS[0].bitmap);
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
    use crate::geom::{BIG_CELL, BIG_GRID, BIG_SHIFT, CELL, FAR, FAR_BIG, GRID, Look, Q, SHIFT, View, object_centre, pop_depth, sphere_visible};
    use crate::format::{BIG_DIR, OBJ_DIR, OBJ_RECORDS};
    let reu = package();
    let block = scene(reu);
    // Every cell of both grids: its centre (Q4), its records, the records' shift.
    let mut cells = Vec::new();
    for (dir, n, cell, sh, cap) in [(OBJ_DIR, GRID, CELL * Q, SHIFT, FAR), (BIG_DIR, BIG_GRID, BIG_CELL * Q, BIG_SHIFT, FAR_BIG)] {
        for k in 0..n * n {
            let e = &reu[dir as usize + 4 * k..][..4];
            let (first, count) = (u16::from_le_bytes([e[0], e[1]]) as usize, e[2] as usize);
            let centre = ((k % n) as i32 * cell + cell / 2, (k / n) as i32 * cell + cell / 2);
            for r in 0..count {
                cells.push((centre, sh, cap as i16, &reu[OBJ_RECORDS as usize + 8 * (first + r)..][..8]));
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
        for &((cu, cv), sh, cap, rec) in &cells {
            // Only near enough for the offsets to fit i16 (the walk's reach is well inside this).
            if (cu - (cam.u >> 4)).abs() > 20000 || (cv - (cam.v >> 4)).abs() > 20000 {
                continue;
            }
            let rad = block[8 + rec[crate::geom::scene::O_MESH] as usize * 10 + 7];
            let c = object_centre(&look, look.cell_sums(cu, cv), sh, rec);
            if sphere_visible(c, (rad as i32) << 4, pop_depth(rad, look.scale, cap)) {
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
        let (banded, dropped, far) = raster_bands(&mut b, &work, buf.bitmap);
        // (`raster` draws every far item: a far one left out would show only as a bitmap mismatch.)
        assert_eq!((dropped, far), (0, 0), "tick {tick}");
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
    let (tris, dropped, _) = raster_bands(&mut m, &w, BUFFERS[0].bitmap);
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
        let (_, dropped, _) = crate::geom::raster_bands(&mut m, &work, BUFFERS[0].bitmap);
        let (_, fs) = crate::package::render(reu, t, Lines::Full);
        assert_eq!((m.stats.stray_writes, st.dropped, dropped), (0, 0, 0), "frame {t}");
        println!("frame {t}: {} objects, {} runs", st.objects, fs.spans);
    }
}

#[test]
fn the_brain_tree_replays_the_plans() {
    // Every symbol state a feature block can show (each feature at a value on the right side of all its
    // symbols' thresholds) walks the baked tree to the planner's action, or at a learn node to its pair's
    // first action under the odds (a random byte of 0) and its second over them (255).
    use crate::agent::feature;
    use crate::goap::{LEARN, N, SYMBOLS, learn_pair, state_of, table};
    let table = table();
    let blob = crate::goap::compile();
    assert!(blob.len() as u32 <= crate::brain::PILOT_LEN, "the pilot's blob outgrew PILOT_LEN");
    let mut reu: &[u8] = &blob;
    let mut candidates: Vec<Vec<u8>> = vec![vec![0, 255]; feature::COUNT];
    for sy in SYMBOLS {
        candidates[sy.feature as usize].extend([sy.threshold - 1, sy.threshold]);
    }
    let (mut shown, mut learns) = (0, 0);
    'states: for st in 0..1u32 << N {
        let mut f = [0u8; feature::BLOCK];
        for (k, c) in candidates.iter().enumerate() {
            let fits = |v: u8| SYMBOLS.iter().enumerate().filter(|(_, sy)| sy.feature as usize == k).all(|(j, sy)| ((v >= sy.threshold) == sy.ge) == (st >> j & 1 != 0));
            match c.iter().find(|&&v| fits(v)) {
                Some(&v) => f[k] = v,
                None => continue 'states,
            }
        }
        assert_eq!(state_of(&f) as u32, st);
        shown += 1;
        let want = table[st as usize];
        let walk = |f: &mut [u8; feature::BLOCK], r: u8, reu: &mut &[u8]| {
            f[feature::RAND] = r;
            crate::brain::walk(reu, 0, crate::brain::PILOT, &f[..]).0
        };
        let (yes, no) = if want >= LEARN { learns += 1; learn_pair(want) } else { (want, want) };
        assert_eq!(walk(&mut f, 0, &mut reu), yes, "state {st:#06x}: under the odds");
        assert_eq!(walk(&mut f, 255, &mut reu), no, "state {st:#06x}: over the odds");
    }
    let tree = crate::goap::pilot_tree(&table);
    assert!(tree.depth() < crate::brain::MAX_DEPTH as usize);
    println!("{shown} states shown, {learns} at a learn node; tree {} nodes, depth {}, blob {} bytes", tree.nodes(), tree.depth(), blob.len());
}

#[test]
fn the_planner_lifts_off_and_climbs_or_steers() {
    use crate::goap::{OPS, plan, s};
    let first = |st: u32| OPS[plan(st).unwrap()[0]].name;
    assert_eq!(first(s::GROUNDED), "wait");
    assert_eq!(first(s::GROUNDED | s::ROTOR), "lift");
    assert_eq!(first(s::LOW), "climb-up");
    assert_eq!(first(s::BLOCKED | s::BLOCKED_L), "evade-r");
    assert_eq!(first(s::BLOCKED | s::BLOCKED_L | s::BLOCKED_R), "climb-over");
    assert_eq!(first(s::WP_L), "turn-l");
    assert_eq!(first(s::HIGH), "descend");
    // Hurt, home beyond a hill: over it, not round (home is ringed by hills: round them, it circles).
    assert_eq!(first(s::HURT | s::BLOCKED | s::BLOCKED_FAR), "climb-over");
    // Every state has a plan, the fight's too: planless, the table flies FORWARD, into whatever is ahead.
    let planless: Vec<u32> = (0..1u32 << crate::goap::N).filter(|&st| !crate::goap::goal(st) && plan(st).is_none()).collect();
    assert!(planless.is_empty(), "{} states without a plan, the first {:#07x}", planless.len(), planless[0]);
}

#[test]
fn the_planner_hunts_and_goes_home_hurt() {
    // The fight's rows: the action the table gives (or a learn node's pair).
    use crate::agent::action::*;
    use crate::goap::{LEARN, learn_pair, s, table};
    let t = table();
    let act = |st: u32| {
        let a = t[st as usize];
        if a >= LEARN { learn_pair(a) } else { (a, a) }
    };
    let cruise = s::ROTOR;
    assert_eq!(act(cruise | s::HUNT | s::LOCKED | s::CLOSE), (LAUNCH, STRAFE), "locked, in gun range: launch, or strafe (learned)");
    assert_eq!(act(cruise | s::HUNT | s::LOCKED), (LAUNCH, LAUNCH), "locked, out of gun range: launch");
    assert_eq!(act(cruise | s::HUNT | s::LOCKED | s::WP_L).0, LAUNCH, "locked: launch");
    assert_eq!(act(cruise | s::HUNT | s::CLOSE), (STRAFE, STRAFE), "in gun range, ahead: strafe");
    assert_eq!(act(cruise | s::HUNT), (FORWARD, FORWARD), "ahead, out of range: run in");
    assert_eq!(act(cruise | s::HUNT | s::WP_L), (TURN_L, TURN_L), "to the left: turn");
    assert_eq!(act(cruise | s::HUNT | s::CLOSE | s::WP_R), (TURN_R, TURN_R), "off to the right, outside the circle: turn");
    assert_eq!(act(cruise | s::HUNT | s::CLOSE | s::WP_R | s::INSIDE), (FORWARD, FORWARD), "inside the turning circle: extend");
    assert_eq!(act(cruise | s::HURT | s::WP_L | s::INSIDE), (FORWARD, FORWARD), "home inside the circle: out first");
    assert_eq!(act(cruise | s::WP_L | s::INSIDE), (FORWARD, FORWARD), "a roaming waypoint inside the circle: out first");
    let blocked = act(cruise | s::HUNT | s::CLOSE | s::BLOCKED);
    assert!(blocked.0 != STRAFE && blocked.1 != STRAFE, "a mountain ahead: no strafing into it");
    assert_eq!(act(cruise | s::HURT), (FORWARD, FORWARD), "hurt, home ahead: fly to it");
    assert_eq!(act(cruise | s::HURT | s::WP_L), (TURN_L, TURN_L), "hurt, home to the left: turn");
    assert_eq!(act(cruise | s::HURT | s::CLOSE), (SLOW, SLOW), "near home: slow");
    assert_eq!(act(cruise | s::HURT | s::CLOSE | s::ARRIVED), (LAND, LAND), "over home: land");
    assert_eq!(act(cruise | s::HURT | s::CLOSE | s::ARRIVED | s::LOW), (LAND, LAND), "on the way down: land");
    assert_eq!(act(cruise | s::HURT | s::ARRIVED | s::GROUNDED), (IDLE, IDLE), "landed home: rearm");
    assert_eq!(act(cruise | s::HURT | s::GROUNDED), (LIFT, LIFT), "landed away from home: up again");
    assert_eq!(act(cruise | s::GROUNDED), (LIFT, LIFT), "whole again: off");
    assert_eq!(act(cruise | s::HUNT | s::HURT), (FORWARD, FORWARD), "hurt beats hunting");
    assert_eq!(act(cruise | s::HUNT | s::LOCKED | s::LOW).0, CLIMB, "low: up first");
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
            // AP is the autopilot's: lit airborne, and off the moment the player takes the aircraft.
            assert_eq!(r[WARN] & lamp::AP != 0, a.grounded == 0, "tick {}", a.t);
            let mut p = *a;
            p.take_over();
            readings(&p, &mut r);
            assert_eq!(r[WARN] & lamp::AP, 0, "tick {}: AP lit under the player", a.t);
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
    use crate::cockpit::{BASE, END, FRAMES, FRAME_STRIDE_SHIFT, TALL_AT, WIDE, gauge::*};
    assert!(END <= crate::format::IMAGE_END);
    let reu = package();
    let frame = |g: usize, k: u16| {
        let at = BASE[g] as usize + ((k as usize) << FRAME_STRIDE_SHIFT);
        &reu[at..at + WIDE[g] as usize * 8 * TALL_AT[g] as usize]
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
        let (m, _) = crate::package::render(reu, a.t, Lines::Full);
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
    for (i, b) in [1u8, 0, 4, 132, 3, 3, 7, 0].into_iter().enumerate() {
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
    assert_eq!((rd16(&m, LOCK_STATE), m.ram[LOCK_STATE as usize + 2]), (7, LOCK_TICKS), "vehicle 7 locked");
    assert_eq!(m.ram[(S_SHAPES + slot::DIAMOND as u16) as usize], shape::LOCKED);
    assert!(list.iter().any(|s| s.ptr == pointer(slot::LOCK)), "LOCK shown");
    assert!(list.iter().any(|s| s.ptr == pointer(slot::BOX)), "the target boxed");
    // Part way: still closing in.
    let (m, _) = hud_run(0, 6);
    let ticks = m.ram[LOCK_STATE as usize + 2];
    assert!(ticks > 0 && ticks < LOCK_TICKS, "{ticks}");
    assert_ne!(m.ram[(S_SHAPES + slot::DIAMOND as u16) as usize], shape::LOCKED);
    // Turned 20° away: boxed, not locked; the diamond waits at the boresight.
    let (m, list) = hud_run(20 * 182, 20);
    assert_eq!(rd16(&m, LOCK_STATE), crate::world_sim::NOBODY);
    assert!(list.iter().any(|s| s.ptr == pointer(slot::BOX)));
    assert_eq!(m.ram[(S_SHAPES + slot::DIAMOND as u16) as usize], shape::DIAMOND);
}

/// The lock stays on its vehicle: two of a formation in the reticle, taking turns at being the nearer
/// and swapping places in the target list every frame, and the lock still builds to `LOCK` on the first.
#[test]
fn the_seeker_keeps_its_vehicle_as_the_formation_shuffles() {
    use crate::hud::*;
    let mut m = Machine::new(package());
    init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.wu, a.wv) = (1024 << 8, 800 << 8, 10 << 8, 1024, 1200);
    let cam = camera_at(1024.0, 800.0, 10.0, 0, 0, 0);
    let mut first = None;
    for f in 0..20u32 {
        // (u, v, h, vid) each, the nearer listed first.
        let (near, far) = if f % 2 == 0 { (7u16, 8u16) } else { (8, 7) };
        let mut t = vec![2u8];
        for (vid, v) in [(near, 900u16), (far, 901)] {
            t.extend_from_slice(&1024u16.to_le_bytes());
            t.extend_from_slice(&v.to_le_bytes());
            t.push(3);
            t.extend_from_slice(&vid.to_le_bytes());
        }
        m.ram[S_TARGETS as usize..][..t.len()].copy_from_slice(&t);
        a.t = 100 + 8 * f;
        m.ram[READY as usize] = 0;
        update(&mut m, &a, &cam);
        let on = rd16(&m, LOCK_STATE);
        assert_eq!(*first.get_or_insert(on), on, "frame {f}: the lock moved to another vehicle");
    }
    assert_eq!(first, Some(7), "on the nearer at the start");
    assert_eq!(m.ram[LOCK_STATE as usize + 2], LOCK_TICKS, "locked");
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
    use crate::agent::{Agent, LIFT, THROTTLE_MAX, input};
    let mut reu: &[u8] = package();
    let mut a = Agent::new();
    a.take_over();
    // (Fire held, as a player would: `main.rs` hands the player the controls each frame, except past the
    // sea limit, where a random stick may wander.)
    let run = |a: &mut Agent, reu: &mut &[u8], stick: u8, ticks: u32| {
        a.input = stick;
        for _ in 0..ticks {
            a.take_over();
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

/// The player's helicopter at cruise over (`u`, `v`), heading `yaw`, at half throttle and its speed.
fn player_at(u: i32, v: i32, yaw: u16) -> crate::agent::Agent {
    use crate::agent::{Agent, CRUISE, EYE, LIFT, THROTTLE_MAX};
    let mut a = Agent::new();
    (a.t, a.u, a.v, a.h, a.yaw, a.grounded) = (LIFT + 100, u << 8, v << 8, EYE + ((CRUISE as i32) << 8), yaw, 0);
    a.take_over();
    (a.throttle, a.vf) = (THROTTLE_MAX / 2, 4096);
    a
}

/// The copilot (the attract-mode pilot, flying to a waypoint ahead on the player's course): let go after
/// a turn, the wings come level, the course holds, and the height stays at cruise.
#[test]
fn hands_off_the_copilot_levels_and_holds_course() {
    use crate::agent::{CRUISE, input};
    const DEG: i16 = 182;
    let mut reu: &[u8] = package();
    let mountains = crate::goap::mountains(&crate::world::generate(1983));
    // A start and heading with no mountain near the next 700 texels (turned 15° left of it, to turn
    // back onto it).
    let clear = |u0: f64, v0: f64, yaw: u16| {
        let a = yaw as f64 / 65536.0 * std::f64::consts::TAU;
        (0..=70).all(|k| {
            let (u, v) = (u0 + a.sin() * k as f64 * 10.0, v0 + a.cos() * k as f64 * 10.0);
            mountains.iter().all(|&(mu, mv, mr, _)| ((u - mu).powi(2) + (v - mv).powi(2)).sqrt() > mr + 40.0)
                && (60.0..1988.0).contains(&u) && (60.0..1988.0).contains(&v)
        })
    };
    let (u0, v0, yaw) = (0..64u32)
        .flat_map(|p| (0..32u16).map(move |k| (400.0 + (p % 8) as f64 * 160.0, 400.0 + (p / 8) as f64 * 160.0, k << 11)))
        .find(|&(u, v, y)| clear(u, v, y) && clear(u, v, y.wrapping_add(15 * DEG as u16)))
        .expect("no clear heading");
    let mut a = player_at(u0 as i32, v0 as i32, yaw.wrapping_add(15 * DEG as u16));
    a.input = input::LEFT;
    while (a.yaw.wrapping_sub(yaw) as i16) > 0 {
        a.step(&mut reu);
        assert!(a.t < crate::agent::LIFT + 1000, "never turned");
    }
    a.input = 0;
    for k in 0..400 {
        a.step(&mut reu);
        if std::env::var("COPILOT_TRACE").is_ok() && k % 10 == 0 {
            println!("{k} roll {} yaw {} steer {} action {}", a.roll / 18, (a.yaw.wrapping_sub(yaw) as i16) / 18, a.steer / 18, a.action);
        }
    }
    assert!(a.roll.abs() < 2 * DEG, "still banked {}°", a.roll / DEG);
    let course = a.yaw;
    for _ in 0..1200 {
        a.step(&mut reu);
        let alt = (a.h >> 8) as i16 - CRUISE;
        assert!((-2..3).contains(&alt), "tick {}: {alt} texels off cruise", a.t);
    }
    let drift = a.yaw.wrapping_sub(course) as i16;
    assert!(drift.abs() < 3 * DEG, "drifted {}°", drift / DEG);
}

/// Hands off, at a mountain: the copilot gets over (or round) it, back down to cruise, and back on the
/// course it was on.
#[test]
fn the_copilot_climbs_over_what_is_ahead() {
    use crate::agent::CRUISE;
    const DEG: i16 = 182;
    let mut reu: &[u8] = package();
    let mountains = crate::goap::mountains(&crate::world::generate(1983));
    // The tallest mountain well inside the map, from 220 texels south of it, heading north.
    let &(mu, mv, mr, mh) = mountains
        .iter()
        .filter(|m| (400.0..1650.0).contains(&m.0) && (500.0..1500.0).contains(&m.1))
        .max_by(|a, b| a.3.total_cmp(&b.3))
        .expect("no mountain");
    let mut a = player_at(mu as i32, (mv - mr - 220.0) as i32, 0);
    a.input = 0;
    let (mut top, mut wide, mut passed) = (0i32, 0f64, 0u32);
    while passed < 1500 {
        a.step(&mut reu);
        let (u, v, h) = (a.u as f64 / 256.0, a.v as f64 / 256.0, a.h as f64 / 256.0);
        let d = ((u - mu).powi(2) + (v - mv).powi(2)).sqrt();
        if d < mr {
            assert!(h > mh * (1.0 - d / mr), "tick {}: into the mountain", a.t);
        }
        if (v - mv).abs() < mr {
            (top, wide) = (top.max(a.h >> 8), wide.max((u - mu).abs()));
        }
        passed += (v > mv + mr + 40.0) as u32;
        assert!(a.t < crate::agent::LIFT + 100 + 6000, "never past: at ({u:.0}, {v:.0})");
    }
    assert!(top as f64 > mh || wide > mr, "neither over nor round: top {top} of {mh}, {wide:.0} off of {mr}");
    println!("past the {mh}-texel mountain: up to {top} texels, {wide:.0} texels off its middle (radius {mr})");
    let alt = (a.h >> 8) as i16 - CRUISE;
    assert!((-2..3).contains(&alt), "{alt} texels off cruise after it");
    let off = a.yaw as i16;
    assert!(off.abs() < 10 * DEG, "course {}° off north after it", off / DEG);
}

/// Hands off across the hills: from south of each mountain well inside the map, heading north, the
/// copilot never flies into the ground, always gets past (no circling back to a point behind it), and is
/// heading back on its course afterwards (nearly always within 10°), down at cruise height (the hills
/// north of home go on: well past them, not just past the one it set out at).
#[test]
fn hands_off_the_copilot_crosses_the_hills_on_course() {
    const DEG: f64 = 182.04;
    let mut reu: &[u8] = package();
    let mountains = crate::goap::mountains(&crate::world::generate(1983));
    let (mut runs, mut on_course, mut worst_end, mut highest) = (0u32, 0u32, 0f64, 0f64);
    for &(mu, mv, mr, mh) in mountains.iter().filter(|m| (300.0..1750.0).contains(&m.0) && (400.0..1600.0).contains(&m.1)) {
        let mut a = player_at(mu as i32, (mv - mr - 220.0) as i32, 0);
        a.input = 0;
        let (mut wide, mut passed) = (0f64, 0u32);
        while passed < 4000 {
            a.step(&mut reu);
            let (u, v, h) = (a.u as f64 / 256.0, a.v as f64 / 256.0, a.h as f64 / 256.0);
            let d = ((u - mu).powi(2) + (v - mv).powi(2)).sqrt();
            assert!(d >= mr || h > mh * (1.0 - d / mr), "({mu:.0}, {mv:.0}): into it at tick {}", a.t);
            if (v - mv).abs() < mr {
                wide = wide.max((u - mu).abs());
            }
            passed += (v > mv + mr + 40.0) as u32;
            assert!(a.t < crate::agent::LIFT + 100 + 12000, "({mu:.0}, {mv:.0}): never past, at ({u:.0}, {v:.0}), yaw {:.0}°", (a.yaw as i16) as f64 / DEG);
        }
        let end = ((a.yaw as i16) as f64 / DEG).abs();
        let line = (a.u as f64 / 256.0 - mu).abs();
        let high = a.h as f64 / 256.0;
        println!("({mu:.0}, {mv:.0}) r {mr:.0} h {mh:.0}: {wide:.0} off its middle, then {line:.0} texels off the line, {end:.0}° off course, {high:.0} up");
        highest = highest.max(high);
        runs += 1;
        on_course += (end < 10.0) as u32;
        worst_end = worst_end.max(end);
    }
    println!("{on_course} of {runs} within 10° of the course after, worst {worst_end:.0}°, highest {highest:.0}");
    assert!(runs >= 40 && on_course * 10 >= runs * 9 && worst_end < 40.0, "{on_course} of {runs} back on course, worst {worst_end:.0}°");
    assert!(highest < (crate::agent::CRUISE + 4) as f64, "as high as {highest:.0} after");
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
    let st = crate::package::render_floor(&mut m, BUFFERS[0], &scene, &cam, Lines::Full);
    let mut rgb = m.decode(BUFFERS[0], crate::format::COLORS[0]);
    crate::hud::composite(&m, 0, &mut rgb);
    crate::png::save_screen(std::path::Path::new("output/m2/close.png"), &rgb).unwrap();
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
    trace::ON.set(true);
    for t in (0..10 * 3600u32).step_by(8) {
        let d = carrier(&mut &m.reu[..], t as u16).unwrap();
        ws::step_page(&mut m, t as u16, (d.u >> 4) as i16, (d.v >> 4) as i16);
    }
    trace::ON.set(false);
    let log = trace::LOG.with_borrow_mut(std::mem::take);
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
    a.dyaw = (a.yaw >> 8) as u8;
    let warn = |a: &Agent| {
        let mut k = [0u8; gauge::COUNT];
        readings(a, &mut k);
        k[gauge::WARN]
    };
    let lamp_on = |a: &Agent| warn(a) >= lamp::SEA;
    assert!(!lamp_on(&a), "the lamp over the map");
    let (mut warned, mut taken, mut far, mut held) = (false, false, 0i16, 0u32);
    // The stick forward and fire held (`main.rs`: fire hands the player the controls every frame), until
    // ten seconds after the takeover.
    while held < 600 {
        a.input = input::UP | input::FIRE;
        a.take_over();
        let t = a.t + 4;
        a.fly(&mut &reu[..], t);
        warned |= lamp_on(&a) && !taken;
        if a.mode == MODE_AUTO {
            assert!(warned, "taken over without the lamp first");
            assert_eq!((a.wu, a.wv), (HOME.0 as i16, HOME.1 as i16), "for home");
            // The autopilot's: SEA with AP.
            assert_eq!(warn(&a), lamp::SEA_AP, "{} texels out", a.off_map());
            taken = true;
        } else if warned {
            // The player's: SEA alone, no AP.
            assert_eq!(warn(&a), lamp::SEA, "{} texels out", a.off_map());
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
    assert!(!crate::combat::update(&mut m, &mut a.clone(), &a.camera().0), "not down yet");
    assert_eq!(m.ram[(PLAYER + player::DAMAGE) as usize], DOWN - 2, "repaired on the pad");
    m.ram[(PLAYER + player::DAMAGE) as usize] = DOWN;
    assert!(crate::combat::update(&mut m, &mut a.clone(), &a.camera().0), "down on the pad");
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
        a.dyaw = (a.yaw >> 8) as u8;
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
    assert!(crate::combat::update(&mut m, &mut b.clone(), &cam), "down after ditching");
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
    use crate::effects::{EFFECTS, FRAMES, SCALES, lines};
    use crate::effects_bake::{band_chunk, read_record};
    use crate::format::{EFFECTS_BLOCKS, EFFECTS_BLOCKS_END, FRAME_RECORD, MAX_SPRITE_SIZE, REU_BANK};
    let reu = package();
    for e in 0..EFFECTS as u8 {
        for s in 0..SCALES as u8 {
            for f in 0..1u8 << crate::effects::SLOT_SHIFT[e as usize] {
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
                    let low = block >= EFFECTS_BLOCKS && end <= crate::format::EFFECTS_LOW_END;
                    assert!(low || block >= crate::format::EFFECTS_HIGH && end <= EFFECTS_BLOCKS_END, "effect {e} scale {s} frame {f} phase {p}: ${block:06X}");
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

/// With every effect slot taken, a new effect takes the oldest playing one's, never a wreck's fire still
/// waiting for its explosion to turn to smoke.
#[test]
fn a_waiting_wreck_fire_keeps_its_slot() {
    use crate::combat::{self, FIRE_AFTER, FX, FX_ENTRY, FX_MAX, spawn};
    use crate::effects::effect;
    use crate::hw::Hw;
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let now = 1000u16;
    // A kill: its explosion and its fire to come; then the rest of the slots, muzzle flashes, newer.
    spawn(&mut m, effect::EXPLOSION, 0, 0, 0, now);
    spawn(&mut m, effect::WRECK_FIRE, 0, 0, 0, now.wrapping_add(FIRE_AFTER));
    for k in 0..FX_MAX as u16 - 2 {
        spawn(&mut m, effect::MUZZLE, 0, 0, 0, now + 1 + k);
    }
    spawn(&mut m, effect::IMPACT, 0, 0, 0, now + 20);
    let ids: Vec<u8> = (0..FX_MAX as u16).map(|k| m.peek(FX + k * FX_ENTRY)).collect();
    assert!(ids.contains(&(effect::WRECK_FIRE + 1)), "the fire was taken: {ids:?}");
    assert!(!ids.contains(&(effect::EXPLOSION + 1)), "the oldest (the explosion) should have gone: {ids:?}");
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
    a.dyaw = (a.yaw >> 8) as u8;
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
    combat::update(&mut m, &mut a.clone(), &cam);
    assert_eq!(m.ram[OBJLIST as usize + 1], crate::actors::wreck_of(crate::actors::kind::TANK), "drawn as a wreck");
    assert_eq!(m.ram[WRECKS as usize], 1, "smoking");
    combat::fx(&mut m, &a, &cam);
    // (Not counting the clouds, also listed there.)
    let keys: Vec<u8> = (0..m.ram[FXDYN as usize] as usize).map(|k| m.ram[FXDYN as usize + 8 + 8 * k]).filter(|&k| k != crate::clouds::KEY).collect();
    assert_eq!(keys.len(), 1, "its smoke column placed");
    // (Short of the fog's buckets, which `bin` takes nearest first: `combat::effects`.)
    assert!(keys.iter().all(|&k| k < crate::geom::FAR_KEY), "{keys:?}");
    a.t = 1000 + WRECK_TICKS as u32;
    combat::update(&mut m, &mut a.clone(), &cam);
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
    a.dyaw = (a.yaw >> 8) as u8;
    let put = |m: &mut Machine, id: u16, u: u16, v: u16| {
        let at = ws::UNITS as usize + id as usize * RECORD as usize;
        let r = &mut m.reu[at..at + RECORD as usize];
        r.fill(0);
        r[rec::KIND as usize] = crate::actors::kind::TANK;
        r[rec::FLAGS as usize] = ALIVE | crate::actors::ENEMY;
        r[rec::HP as usize] = 20;
        // One tank (its platoon's others fallen).
        r[rec::LOST as usize] = 0xFE;
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
        combat::update(&mut m, &mut a.clone(), &cam);
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
    // The other tank (platoon 9, its one vehicle): locked, fire 2.
    m.ram[crate::hud::LOCK_STATE as usize..][..2].copy_from_slice(&ws::vehicle(9, 0).to_le_bytes());
    m.ram[crate::hud::LOCK_STATE as usize + 2] = crate::hud::LOCK_TICKS;
    a.input = input::FIRE2;
    combat::update(&mut m, &mut a.clone(), &cam);
    assert_eq!(m.ram[(PLAYER + player::MISSILES) as usize], combat::MISSILES - 1, "no missile");
    a.input = 0;
    let hp0 = m.reu[ws::UNITS as usize + 9 * RECORD as usize + rec::HP as usize];
    for f in 0..200u32 {
        a.t += 4;
        let _ = f;
        combat::update(&mut m, &mut a.clone(), &cam);
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
    let proj = |x: f64, y: f64, z: f64| (80.0 * 16.0 + 1280.0 * x / z, crate::geom::CY as f64 * 16.0 - 2128.0 * y / z);
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
    let (mut t, mut prev, mut dropped) = (0u32, Vec::<u16>::new(), 0u32);
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
        crate::combat::update(&mut m, &mut a.clone(), &cam);
        let objects = m.ram[crate::combat::OBJLIST as usize];
        crate::actors::prepare(&mut m, &cam, crate::combat::OBJLIST);
        let drawn = m.ram[crate::actors::DYN as usize];
        capped += (drawn == crate::actors::DYN_MAX) as u32;
        most = most.max(objects);
        let n = m.ram[ws::NEAR as usize].min(ws::NEAR_MAX) as usize;
        let listed: Vec<u16> = (0..n).map(|k| u16::from_le_bytes([m.ram[ws::NEAR as usize + 1 + 2 * k], m.ram[ws::NEAR as usize + 2 + 2 * k]])).collect();
        dropped += prev.iter().filter(|&&id| !listed.contains(&id) && on_screen(&m, id)).count() as u32;
        prev = listed.into_iter().filter(|&id| on_screen(&m, id)).collect();
    }
    eprintln!("draw list: {most} objects at most; the actors' cap of {} reached in {capped} frames", crate::actors::DYN_MAX);
    (dropped, capped)
}

/// Ten minutes from the start on the deck: no unit on screen is ever dropped from the near list (units
/// seen to vanish were faces crossing the near plane, now cut instead of dropped), and the actors' cap
/// (`actors::DYN_MAX`, as many as `DYN` holds below `JOB`) is reached in under a tenth of the frames
/// (about 5½% with platoons: 247 of 4500). With platoons the cap is their level of detail: the draw
/// list is their nearest vehicles, those in front first (`squad::expand`), and the cull takes them in
/// that order after the projectiles (`actors::prepare`), so what it cuts is the farthest vehicles, never
/// the player's fire (`the_cull_cuts_the_farthest_vehicles_not_the_fire`). Reached in most frames, the
/// budget would be wrong.
#[test]
fn the_cull_cuts_the_farthest_vehicles_not_the_fire() {
    use crate::actors::{ALLIED, DYN, DYN_ENTRY, DYN_MAX, ENEMY, kind, mesh_of, prepare};
    use crate::combat::OBJLIST;
    let mut m = Machine::new(package());
    let cam = camera_at(1024.0, 1100.0, 10.0, 0, -11, 0);
    // (mesh, yaw, u, v, h 1/16 texel) into the list.
    let list = |m: &mut Machine, objs: &[(u8, u16)]| {
        m.ram[OBJLIST as usize] = objs.len() as u8;
        for (k, &(mesh, v)) in objs.iter().enumerate() {
            let at = OBJLIST as usize + 1 + 8 * k;
            m.ram[at] = mesh;
            m.ram[at + 1] = 0;
            m.ram[at + 2..at + 4].copy_from_slice(&(1024u16 << 4).to_le_bytes());
            m.ram[at + 4..at + 6].copy_from_slice(&(v << 4).to_le_bytes());
            m.ram[at + 6..at + 8].copy_from_slice(&(4u16 << 4).to_le_bytes());
        }
    };
    let placed = |m: &mut Machine, objs: &[(u8, u16)]| -> Vec<Vec<u8>> {
        list(m, objs);
        prepare(m, &cam, OBJLIST);
        let n = m.ram[DYN as usize] as usize;
        (0..n).map(|k| m.ram[DYN as usize + 1 + k * DYN_ENTRY as usize..][..DYN_ENTRY as usize].to_vec()).collect()
    };
    // A dozen and one tanks ahead, nearest first, then a tracer and a missile in flight among them.
    let mut objs: Vec<(u8, u16)> = (0..DYN_MAX as u16 + 1).map(|k| (mesh_of(kind::TANK, ENEMY), 1130 + 5 * k)).collect();
    let fire = [(mesh_of(kind::TRACER, ALLIED), 1120), (mesh_of(kind::MISSILE, ALLIED), 1125)];
    objs.extend_from_slice(&fire);
    let all = placed(&mut m, &objs);
    assert_eq!(all.len(), DYN_MAX as usize, "the cap reached");
    for f in fire {
        let alone = placed(&mut m, &[f]);
        assert_eq!(alone.len(), 1, "in view");
        assert!(all.contains(&alone[0]), "projectile {} cut", f.0);
    }
    // The vehicles drawn are the nearest: the farthest three cut.
    for (k, &t) in objs[..DYN_MAX as usize - 2].iter().enumerate() {
        assert!(all.contains(&placed(&mut m, &[t])[0]), "tank {k} cut");
    }
}

#[test]
fn units_on_screen_stay_listed() {
    let (dropped, capped) = near_churn(600);
    assert_eq!(dropped, 0, "units on screen dropped from the near list");
    assert!(capped <= 600 * 60 / 8 / 10, "the actors' cap reached in {capped} frames (over 10%)");
}

#[test]
fn far_units_draw_their_low_poly_mesh() {
    use crate::actors::{DYN, DYN_ENTRY, LOD_Z, MESH_RECORD, dyn_entry, kind, mesh_of, prepare};
    use crate::hw::Hw;
    let bank = crate::scene::vehicle_bank();
    let rec = |mesh: usize| &bank[mesh * MESH_RECORD..][..MESH_RECORD];
    // Each unit's far mesh: 6 faces, with none of its own (the big ships, projectiles and meshes of 6
    // faces or fewer keep theirs).
    let mut fars = 0;
    for k in 0..kind::COUNT {
        for side in 0..2 {
            let near = rec(mesh_of(k, side) as usize);
            if matches!(k, kind::CARRIER | kind::DESTROYER | kind::TRACER | kind::MISSILE) || near[5] <= 6 {
                assert_eq!(near[3], 0, "{}: no far mesh", kind::NAMES[k as usize]);
                continue;
            }
            let far = rec(near[3] as usize);
            assert_ne!(near[3], 0, "{}: a far mesh", kind::NAMES[k as usize]);
            assert_eq!((far[3], far[5]), (0, 6), "{}: its far mesh", kind::NAMES[k as usize]);
            fars += 1;
        }
    }
    assert!(fars >= 16, "{fars} far meshes");
    // A tank near and one beyond LOD_Z, straight ahead of a camera looking north.
    let reu = package();
    let mut m = Machine::new(reu);
    let cam = camera_at(1024.0, 1000.0, 6.0, 0, 0, 0);
    let list = crate::combat::OBJLIST;
    let tank = mesh_of(kind::TANK, crate::actors::ENEMY);
    m.poke(list, 2);
    for (k, ahead) in [(0u16, 20i32), (1, LOD_Z / 16 + 30)] {
        let v = ((1000 + ahead) << 4) as u16;
        for (i, b) in [tank, 0, 0x00, 0x40, v as u8, (v >> 8) as u8, 0, 0].into_iter().enumerate() {
            m.poke(list + 1 + 8 * k + i as u16, b);
        }
    }
    prepare(&mut m, &cam, list);
    assert_eq!(m.peek(DYN), 2, "both in view");
    let blob = |m: &mut Machine, k: u16| {
        let at = DYN + 1 + k * DYN_ENTRY;
        (m.peek(at + dyn_entry::BLOB) as u16 | (m.peek(at + dyn_entry::BLOB + 1) as u16) << 8, m.peek(at + dyn_entry::NF))
    };
    let near = rec(tank as usize);
    let far = rec(near[3] as usize);
    assert_eq!(blob(&mut m, 0), (u16::from_le_bytes([near[0], near[1]]), near[5]), "the near tank: its own mesh");
    assert_eq!(blob(&mut m, 1), (u16::from_le_bytes([far[0], far[1]]), 6), "the far tank: the wedge");
}

/// What an autopilot sortie did (`sortie`).
#[derive(Debug, Default)]
struct Sortie {
    kills: u32,
    launched: u32,
    bursts: u32,
    downs: u32,
    /// Landings at home while hurt, and the ticks of the first (and of the lift-off after it).
    landed: u32,
    first_landing: Option<u32>,
    off_again: Option<u32>,
    /// Damage and missiles when whole again after the first landing.
    rearmed: Option<(u8, u8)>,
}

/// The autopilot from boot for `seconds` of clock, framed as the C64 frames it (8 ticks a frame, the war
/// and the gauges taking turns): the agent, the war's page, the fight, the effects, the HUD's seeker.
/// `hurt` pokes that much damage once, a minute after lift-off. Never inside a mountain (fighting,
/// hunting or going home hurt, as roaming: `the_flight_waits_for_the_rotor_then_roams_clear_of_the_mountains`).
fn sortie(m: &mut Machine, seconds: u32, hurt: Option<u8>) -> Sortie {
    use crate::agent::{HOME, LIFT};
    use crate::combat::{MISSILES, PLAYER, player};
    let hills = crate::goap::mountains(&crate::world::generate(1983));
    crate::package::setup(m, BUFFERS[0]);
    crate::combat::init(m);
    crate::hud::init(m);
    let mut a = crate::agent::Agent::boot();
    let mut out = Sortie::default();
    let (mut t, mut turn, mut missiles, mut bursts, mut was_down) = (0u32, 0u8, MISSILES, 0u8, false);
    while t < seconds * 60 {
        t += 8;
        let reu = m.reu.clone();
        a.fly(&mut &reu[..], t);
        let (u, v, h) = (a.u as f64 / 256.0, a.v as f64 / 256.0, a.h as f64 / 256.0);
        for &(mu, mv, r, top) in &hills {
            // (The cone's peak may sit up to 0.075 r off the centre.)
            let d = ((u - mu).hypot(v - mv) - 0.08 * r).max(0.0);
            assert!(d >= r || h > top * (1.0 - d / r), "tick {}: into the mountain at ({mu:.0}, {mv:.0}), {}, combat {}", a.t, crate::agent::action::NAMES[a.action as usize], a.combat);
        }
        if turn == 0 {
            let (pu, pv) = a.at();
            crate::world_sim::step_page(m, a.t as u16, pu, pv);
        }
        turn ^= 1;
        if hurt.is_some() && a.t >= LIFT + 3600 && a.t < LIFT + 3608 {
            m.ram[(PLAYER + player::DAMAGE) as usize] = hurt.unwrap();
        }
        let kills = u16::from_le_bytes([m.ram[(PLAYER + player::KILLS) as usize], m.ram[(PLAYER + player::KILLS) as usize + 1]]);
        let cam = a.camera().0;
        if crate::combat::update(m, &mut a, &cam) {
            a.reset();
            out.downs += 1;
            out.kills += kills as u32;
            (missiles, bursts) = (MISSILES, 0);
            continue;
        }
        crate::combat::fx(m, &a, &a.camera().0);
        crate::hud::update(m, &a, &a.camera().0);
        let now = m.ram[(PLAYER + player::MISSILES) as usize];
        out.launched += missiles.saturating_sub(now) as u32;
        missiles = now;
        let b = m.ram[(PLAYER + player::BURSTS) as usize];
        out.bursts += b.wrapping_sub(bursts) as u32;
        bursts = b;
        // Down at home, hurt: a landing.
        let pad = crate::combat::PAD as i32;
        let home = ((a.u >> 8) - HOME.0).abs() <= pad && ((a.v >> 8) - HOME.1).abs() <= pad;
        let down = a.grounded != 0 && home && a.t > LIFT + 60;
        if down && !was_down {
            out.landed += 1;
            out.first_landing.get_or_insert(a.t);
        }
        if out.first_landing.is_some() && out.off_again.is_none() && a.grounded == 0 {
            out.off_again = Some(a.t);
            out.rearmed = Some((m.ram[(PLAYER + player::DAMAGE) as usize], m.ram[(PLAYER + player::MISSILES) as usize]));
        }
        was_down = down;
        let window = std::env::var("SORTIE_TRACE").ok().and_then(|w| w.split_once('-').map(|(a, b)| (a.parse::<u32>().unwrap_or(0), b.parse::<u32>().unwrap_or(0))));
        let traced = match window { Some((from, to)) => a.t >= from && a.t < to, None => std::env::var("SORTIE_TRACE").is_ok() && t % 240 == 0 };
        if traced {
            let mut f = [0u8; crate::agent::feature::BLOCK];
            a.clone().features(&mut &reu[..], &mut f);
            eprintln!("  features {f:?}");
            eprintln!(
                "t {} at ({}, {}) h {} wp ({}, {}) action {} combat {} grounded {} vf {} damage {} missiles {}",
                a.t, a.u >> 8, a.v >> 8, a.h >> 8, a.wu, a.wv, crate::agent::action::NAMES[a.action as usize], a.combat, a.grounded, a.vf,
                m.ram[(PLAYER + player::DAMAGE) as usize], m.ram[(PLAYER + player::MISSILES) as usize]
            );
        }
    }
    out.kills += u16::from_le_bytes([m.ram[(PLAYER + player::KILLS) as usize], m.ram[(PLAYER + player::KILLS) as usize + 1]]) as u32;
    out
}

#[test]
fn the_autopilot_fights() {
    // Five minutes after lift-off: it finds the front, fires its gun and missiles, and kills.
    let mut m = Machine::new(package());
    let s = sortie(&mut m, crate::agent::LIFT / 60 + 300, None);
    println!("{s:?}");
    assert!(s.bursts > 0, "no gun fired");
    assert!(s.launched > 0, "no missile launched");
    assert!(s.kills > 0, "no kills");
}

#[test]
fn the_autopilot_goes_home_to_rearm() {
    // Hurt a minute after lift-off: home, down on the pad, repaired and rearmed, and off again.
    use crate::combat::MISSILES;
    let mut m = Machine::new(package());
    let s = sortie(&mut m, crate::agent::LIFT / 60 + 300, Some(60));
    println!("{s:?}");
    let landed = s.first_landing.expect("never landed at home");
    assert!(landed < crate::agent::LIFT + 3600 + 150 * 60, "landed only at tick {landed}");
    assert_eq!(s.rearmed.expect("never off again"), (0, MISSILES), "off again whole and rearmed");
}

/// The pilot's learn nodes in a REU image: (blob offset, odds).
fn pilot_learns(reu: &[u8]) -> Vec<(usize, u8)> {
    let base = crate::format::AI_TREES as usize;
    let len = u16::from_le_bytes([reu[base + 4], reu[base + 5]]) as usize;
    (crate::brain::HEADER_BYTES..len)
        .step_by(crate::brain::NODE_BYTES)
        .filter(|&at| reu[base + at] == crate::brain::PILOT_LEARN)
        .map(|at| (at, reu[base + at + 1]))
        .collect()
}

#[test]
fn the_pilot_learns_from_what_came_of_its_choices() {
    // A better than expected outcome makes the branch each trail entry took likelier (the older at half
    // the step); a worse one, less likely; never out of LEARN_MIN..LEARN_MAX.
    use crate::brain::{LEARN_MAX, LEARN_MIN};
    use crate::combat::{PLAYER, learn_pilot, player};
    let mut m = Machine::new(package());
    let nodes = pilot_learns(&m.reu);
    assert!(nodes.len() >= 2, "learn nodes: {nodes:?}");
    assert!(nodes.iter().all(|&(_, p)| p == 128), "they start even");
    let ((a, _), (b, _)) = (nodes[0], nodes[1]);
    let odds = |m: &Machine, at: usize| m.reu[crate::format::AI_TREES as usize + at + 1];
    // Newest: node a, its yes branch; older: node b, its no branch.
    let t = (PLAYER + player::TRAIL) as usize;
    m.ram[t..t + 2].copy_from_slice(&(a as u16 | 0x8000).to_le_bytes());
    m.ram[t + 4..t + 6].copy_from_slice(&(b as u16).to_le_bytes());
    learn_pilot(&mut m, 40);
    let (ya, yb) = (odds(&m, a), odds(&m, b));
    assert!(ya > 128, "yes, then a good outcome: likelier ({ya})");
    assert!(yb < 128, "no, then a good outcome: yes less likely ({yb})");
    assert_eq!(ya - 128, 2 * (128 - yb), "the older at half the step");
    for _ in 0..200 {
        learn_pilot(&mut m, -100);
    }
    assert_eq!((odds(&m, a), odds(&m, b)), (LEARN_MIN, LEARN_MAX), "bad outcomes, to the band's edge");
}

#[test]
fn the_pilot_learns_in_flight() {
    // Twenty minutes of the autopilot's war: its close calls' odds move with what came of them.
    let mut m = Machine::new(package());
    let before = pilot_learns(&m.reu);
    let s = sortie(&mut m, crate::agent::LIFT / 60 + 20 * 60, None);
    let after = pilot_learns(&m.reu);
    println!("{s:?}");
    let base = crate::format::AI_TREES as usize;
    let leaf = |at: usize| crate::agent::action::NAMES[m.reu[base + at + 1] as usize];
    for (&(at, p0), &(_, p1)) in before.iter().zip(&after) {
        let (no, yes) = (u16::from_le_bytes([m.reu[base + at + 2], m.reu[base + at + 3]]), u16::from_le_bytes([m.reu[base + at + 4], m.reu[base + at + 5]]));
        println!("learn node at {at} ({} or {}): odds of the first {p0} -> {p1}", leaf(yes as usize), leaf(no as usize));
    }
    assert!(after.iter().any(|&(_, p)| p != 128), "nothing learned");
    assert!(s.kills > 0);
}

/// A platoon's damage spills over its members: the hit one falls first, then the dice pick; each falls
/// at its kind's hit points, the wounded one at what it has left; the platoon is dead with its last, and
/// comes back (a reinforcement) at full strength.
#[test]
fn damage_spills_over_a_platoons_members() {
    use crate::actors::kind::{CARRIER, TANK};
    use crate::world_sim::{self as ws, NONE_FELL, stats};
    let hp = stats::HP[TANK as usize];
    let full = stats::MEMBERS[TANK as usize];
    // Under the wounded one's hit points: none falls.
    assert_eq!(ws::spill(TANK as usize, 30, 0, 29, 3, 7), (1, 0, NONE_FELL));
    // Just enough: member 3 (the hit one) falls, the next is whole.
    assert_eq!(ws::spill(TANK as usize, 30, 0, 30, 3, 7), (hp, 1 << 3, 3));
    // More: two fall, the hit one first, and the third is wounded.
    let (h, lost, first) = ws::spill(TANK as usize, 30, 0, 30 + hp + 5, 3, 7);
    assert_eq!((h, first, ws::fallen(lost)), (hp - 5, 3, 2));
    assert_ne!(lost & 1 << 3, 0);
    // 255 is under the whole platoon's worth: some stand.
    let (h, lost, _) = ws::spill(TANK as usize, hp, 0, 255, NONE_FELL, 1);
    assert_eq!(ws::fallen(lost) as u16, 255 / hp as u16);
    assert_eq!(h, hp - (255 % hp as u16) as u8);
    // The last one falls: dead, and `lost` cleared for a respawn at full strength.
    let (h, lost, _) = ws::spill(TANK as usize, 1, (1 << full) - 2, 1, NONE_FELL, 1);
    assert_eq!((h, lost), (0, 0));
    assert_eq!(ws::standing(TANK as usize, 0), full);
    // A hit member already fallen: the dice pick a standing one.
    let (_, lost, first) = ws::spill(TANK as usize, 5, 1 << 3, 5, 3, 9);
    assert!(first != 3 && first < full && lost == (1 << 3 | 1 << first));
    // A single unit falls as it always has.
    assert_eq!(ws::spill(CARRIER as usize, 10, 0, 10, NONE_FELL, 0), (0, 0, 0));
}

/// The hits of a platoon's fire: over all the dice, each of its `n` members' shots hits with its odds
/// (the expected number exactly); one member's, with the odds.
#[test]
fn a_platoons_hits_scale_with_its_members() {
    use crate::world_sim::hits;
    for n in 1..=12u8 {
        for odds in [0u8, 1, 60, 90, 140, 255] {
            let total: u32 = (0..=255u8).map(|d| hits(n, odds, d) as u32).sum();
            assert_eq!(total, n as u32 * odds as u32, "{n} at {odds}");
        }
    }
    assert_eq!((0..=255u8).filter(|&d| hits(1, 120, d) > 0).count(), 120);
}

/// The squad pass: near platoons as their standing members in formation round where they are, nearest
/// first, at most `SLOTS`; the carrier and its deck first; a fallen member not there; the same every
/// time.
#[test]
fn near_platoons_expand_into_their_vehicles() {
    use crate::actors::kind::{APC, CARRIER, TANK};
    use crate::squad::{self, MEMBERS, SLOTS, entry};
    use crate::world_sim::{self as ws, ALIVE, NEAR, RECORD, rec, stats};
    let mut m = Machine::new(package());
    let put = |m: &mut Machine, id: u16, kind: u8, u: u16, v: u16, lost: u16, action: u8| {
        let at = ws::UNITS as usize + id as usize * RECORD as usize;
        let r = &mut m.reu[at..at + RECORD as usize];
        r.fill(0);
        r[rec::KIND as usize] = kind;
        r[rec::FLAGS as usize] = ALIVE | crate::actors::ENEMY;
        r[rec::HP as usize] = stats::HP[kind as usize];
        r[rec::ACTION as usize] = action;
        r[rec::ROUTE as usize] = 0xFF;
        r[rec::U as usize..][..2].copy_from_slice(&(u << 4).to_le_bytes());
        r[rec::V as usize..][..2].copy_from_slice(&(v << 4).to_le_bytes());
        r[rec::LOST as usize..][..2].copy_from_slice(&lost.to_le_bytes());
    };
    // A tank platoon less member 2 near, an APC platoon further and another further still, the carrier
    // far.
    put(&mut m, 3, TANK, 1030, 1110, 1 << 2, ws::act::FIGHT);
    put(&mut m, 4, APC, 1060, 1160, 0, ws::act::MOVE);
    put(&mut m, 6, APC, 1100, 1250, 0, ws::act::MOVE);
    put(&mut m, 5, CARRIER, 1300, 1300, 0, ws::act::MOVE);
    m.ram[NEAR as usize] = 4;
    for (k, id) in [3u16, 6, 4, 5].into_iter().enumerate() {
        m.ram[NEAR as usize + 1 + 2 * k..][..2].copy_from_slice(&id.to_le_bytes());
    }
    squad::expand(&mut m, 100, 1024, 1100, 0);
    let list = |m: &mut Machine| -> Vec<(u16, u8, u16, u16, u8)> {
        let n = m.ram[MEMBERS as usize];
        (0..n)
            .map(|k| {
                let e = squad::nth(m, k) as usize;
                let w = |a: u16| u16::from_le_bytes([m.ram[e + a as usize], m.ram[e + a as usize + 1]]);
                (w(entry::VID), m.ram[e + entry::MESH as usize], w(entry::U) >> 4, w(entry::V) >> 4, m.ram[e + entry::D as usize])
            })
            .collect()
    };
    let got = list(&mut m);
    assert_eq!(got.len(), SLOTS as usize, "7 tanks, 20 APCs, the carrier and its deck: the nearest 20");
    assert_eq!(got[0].0, 5, "the carrier first");
    assert_eq!((got[1].0, got[1].1), (ws::NOBODY, crate::actors::DECK), "then its deck");
    assert!(got[2..].windows(2).all(|w| w[0].4 <= w[1].4), "nearest first: {got:?}");
    let tanks: Vec<u8> = got.iter().filter(|e| ws::platoon(e.0) == 3).map(|e| ws::member(e.0)).collect();
    assert_eq!(tanks.len(), 7, "the tanks standing");
    assert!(!tanks.contains(&2), "the fallen one is not there");
    assert_eq!(got.iter().filter(|e| ws::platoon(e.0) == 4).count(), 10, "the nearer APCs");
    assert_eq!(got.iter().filter(|e| ws::platoon(e.0) == 6).count(), SLOTS as usize - 2 - 7 - 10, "the nearest of the others");
    // Spread round the platoon's place (a line abreast, across its heading).
    let us: std::collections::HashSet<(u16, u16)> = got.iter().filter(|e| ws::platoon(e.0) == 3).map(|e| (e.2, e.3)).collect();
    assert_eq!(us.len(), 7, "seven places: {us:?}");
    let first = got.clone();
    squad::expand(&mut m, 100, 1024, 1100, 0);
    assert_eq!(list(&mut m), first, "the same every time");
}

/// The player's gun or missile on a vehicle: the hit member falls (when its hit points run out), the
/// others stand and are still listed; a kill of a vehicle, not of the platoon.
#[test]
fn the_player_kills_the_vehicle_hit() {
    use crate::actors::kind::TANK;
    use crate::squad::{self, MEMBERS};
    use crate::world_sim::{self as ws, ALIVE, NEAR, RECORD, rec, stats};
    let mut m = Machine::new(package());
    let at = ws::UNITS as usize + 3 * RECORD as usize;
    let r = &mut m.reu[at..at + RECORD as usize];
    r.fill(0);
    r[rec::KIND as usize] = TANK;
    r[rec::FLAGS as usize] = ALIVE | crate::actors::ENEMY;
    r[rec::HP as usize] = stats::HP[TANK as usize];
    r[rec::ROUTE as usize] = 0xFF;
    r[rec::U as usize..][..2].copy_from_slice(&(1030u16 << 4).to_le_bytes());
    r[rec::V as usize..][..2].copy_from_slice(&(1110u16 << 4).to_le_bytes());
    m.ram[NEAR as usize] = 1;
    m.ram[NEAR as usize + 1..][..2].copy_from_slice(&3u16.to_le_bytes());
    assert!(!crate::combat::damage_unit(&mut m, ws::vehicle(3, 5), 20), "wounded");
    assert!(crate::combat::damage_unit(&mut m, ws::vehicle(3, 5), 20), "killed");
    let lost = u16::from_le_bytes([m.reu[at + rec::LOST as usize], m.reu[at + rec::LOST as usize + 1]]);
    assert_eq!(lost, 1 << 5);
    assert_ne!(m.reu[at + rec::FLAGS as usize] & ALIVE, 0, "the platoon fights on");
    squad::expand(&mut m, 0, 1024, 1100, 0);
    assert_eq!(m.ram[MEMBERS as usize], stats::MEMBERS[TANK as usize] - 1);
    assert!(squad::find(&mut m, ws::vehicle(3, 5)).is_none() && squad::find(&mut m, ws::vehicle(3, 4)).is_some());
}

/// A vehicle the war kills in sight blows up where `squad::expand` drew it: at its platoon's live place
/// and heading (moving and turning since its last step), not its record's.
#[test]
fn a_war_kill_explodes_where_its_vehicle_was_drawn() {
    use crate::actors::kind::TANK;
    use crate::combat::{FX, FX_ENTRY, FX_MAX};
    use crate::squad::{self, entry};
    use crate::world_sim::{self as ws, ALIVE, NEAR, NEAR_FLAG, RECORD, SHOTS, rec};
    let mut m = Machine::new(package());
    crate::combat::init(&mut m);
    let at = ws::UNITS as usize + 3 * RECORD as usize;
    let r = &mut m.reu[at..at + RECORD as usize];
    r.fill(0);
    r[rec::KIND as usize] = TANK;
    r[rec::FLAGS as usize] = ALIVE | NEAR_FLAG | crate::actors::ENEMY;
    r[rec::HP as usize] = 1;
    r[rec::ACTION as usize] = ws::act::MOVE;
    r[rec::SPEED as usize] = 8;
    r[rec::ROUTE as usize] = 0;
    // Heading north, turning east.
    r[rec::COURSE as usize] = 64;
    r[rec::U as usize..][..2].copy_from_slice(&(1030u16 << 4).to_le_bytes());
    r[rec::V as usize..][..2].copy_from_slice(&(1110u16 << 4).to_le_bytes());
    m.ram[NEAR as usize] = 1;
    m.ram[NEAR as usize + 1..][..2].copy_from_slice(&3u16.to_le_bytes());
    let now = 60;
    squad::expand(&mut m, now, 1024, 1100, 0);
    let drawn: Vec<(u8, u16, u16)> = (0..m.ram[squad::MEMBERS as usize])
        .map(|k| {
            let e = squad::nth(&mut m, k);
            (ws::member(rd16(&m, e + entry::VID)), rd16(&m, e + entry::U), rd16(&m, e + entry::V))
        })
        .collect();
    m.ram[SHOTS as usize] = 1;
    m.ram[SHOTS as usize + 1..][..2].copy_from_slice(&3u16.to_le_bytes());
    m.ram[SHOTS as usize + 3] = 1;
    ws::apply_shots(&mut m, now);
    let lost = u16::from_le_bytes([m.reu[at + rec::LOST as usize], m.reu[at + rec::LOST as usize + 1]]);
    assert_eq!(lost.count_ones(), 1, "one fell");
    let fell = lost.trailing_zeros() as u8;
    let &(_, u, v) = drawn.iter().find(|d| d.0 == fell).expect("the fallen one was drawn");
    let e = (0..FX_MAX as u16).map(|k| FX + k * FX_ENTRY).find(|&e| m.ram[e as usize] == crate::effects::effect::EXPLOSION + 1).expect("an explosion");
    let (eu, ev) = (rd16(&m, e + 4), rd16(&m, e + 6));
    assert!((eu as i16 - u as i16).abs() <= 16 && (ev as i16 - v as i16).abs() <= 16, "exploded at ({eu}, {ev}), drawn at ({u}, {v})");
}

/// The battle on the horizon's flashes only take free effect slots: never one from the fight at hand.
#[test]
fn far_fire_never_evicts_a_near_effect() {
    use crate::combat::{FX, FX_ENTRY, FX_MAX, spawn};
    use crate::effects::effect::{EXPLOSION, MUZZLE};
    let mut m = Machine::new(package());
    crate::combat::init(&mut m);
    for k in 0..FX_MAX as u16 {
        spawn(&mut m, EXPLOSION, k, 0, 0, 100);
    }
    crate::world_sim::effect(&mut m, MUZZLE, 9, 9, 0, 101, false);
    assert!((0..FX_MAX as u16).all(|k| m.ram[(FX + k * FX_ENTRY) as usize] == EXPLOSION + 1));
    m.ram[(FX + 5 * FX_ENTRY) as usize] = 0;
    crate::world_sim::effect(&mut m, MUZZLE, 9, 9, 0, 101, false);
    assert_eq!(m.ram[(FX + 5 * FX_ENTRY) as usize], MUZZLE + 1, "a free slot taken");
    // Full, the far flash the newest: the fight at hand's next effect takes the far one's slot, not its
    // own oldest.
    spawn(&mut m, EXPLOSION, 99, 0, 0, 102);
    assert_eq!(rd16(&m, FX + 5 * FX_ENTRY + 4), 99, "the far flash evicted first");
    assert!((0..FX_MAX as u16).all(|k| m.ram[(FX + k * FX_ENTRY + 1) as usize] == 0), "no far one left");
}

/// The battle on the horizon's flashes are drawn after the fight at hand's effects and the wrecks'
/// smoke: when there are more than `FXDYN_MAX` in view, the far ones are cut.
#[test]
fn far_fire_is_cut_before_a_near_effect() {
    use crate::agent::{Agent, MODE_PLAYER};
    use crate::combat::{self, FX, FX_ENTRY, FX_MAX, FXDYN, FXDYN_ENTRY, FXDYN_MAX, spawn};
    use crate::effects::effect::{EXPLOSION, MUZZLE};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = Agent::new();
    (a.u, a.v, a.h, a.grounded, a.mode) = (1024 << 8, 1100 << 8, 10 << 8, 0, MODE_PLAYER);
    a.dyaw = (a.yaw >> 8) as u8;
    a.t = 1000;
    let cam = camera_at(1024.0, 1100.0, 10.0, 0, -11, 0);
    // Far flashes in every slot but the last, spread across the view 40 texels ahead; the player's
    // explosion in the last, to one side.
    for k in 0..FX_MAX as u16 - 1 {
        spawn(&mut m, MUZZLE | combat::FAR, (1010 + 2 * k) << 4, 1140 << 4, 0, 1000);
    }
    spawn(&mut m, EXPLOSION, 1040 << 4, 1140 << 4, 0, 1000);
    assert_eq!(m.ram[(FX + (FX_MAX as u16 - 1) * FX_ENTRY) as usize], EXPLOSION + 1);
    combat::fx(&mut m, &a, &cam);
    let n = m.ram[FXDYN as usize];
    assert_eq!(n, FXDYN_MAX, "the cap reached");
    // After the clouds (listed first: never the ones cut), the first placed is the near explosion (the
    // far ones after it): the rightmost anchor.
    let fx: Vec<u16> = (0..n as u16).filter(|k| m.ram[(FXDYN + 1 + k * FXDYN_ENTRY + 7) as usize] != crate::clouds::KEY).collect();
    assert!(fx.len() < n as usize && fx[0] == n as u16 - fx.len() as u16, "the clouds first");
    let x = |k: u16| rd16(&m, FXDYN + 1 + k * FXDYN_ENTRY) as i16;
    assert!(fx[1..].iter().all(|&k| x(fx[0]) > x(k)), "the explosion placed first: {:?}", fx.iter().map(|&k| x(k)).collect::<Vec<_>>());
}

/// The sound effects' placing: a source on the left is louder in the left ear; one behind is muffled
/// (both cutoffs lower than ahead at the same range), the far ear more; a far one is quieter.
#[test]
fn a_sound_is_placed_by_its_bearing_and_range() {
    use crate::sfx::place;
    // Facing north (yaw 0): east is to the right.
    let (l, r, cl, cr, itd) = place(-30, 0, 0, 0, 0, false);
    assert!(l > r && cl > cr, "left: ({l}, {r}) cut ({cl}, {cr})");
    assert_eq!(itd, -crate::sfx::ITD_MAX, "abeam on the left: the right ear lags most");
    let (l, r, _, _, itd) = place(30, 0, 0, 0, 0, false);
    assert!(r > l, "right: ({l}, {r})");
    assert_eq!(itd, crate::sfx::ITD_MAX, "abeam on the right: the left ear lags most");
    let (al, ar, acl, acr, aitd) = place(0, 30, 0, 0, 0, false);
    let (bl, br, bcl, bcr, bitd) = place(0, -30, 0, 0, 0, false);
    assert_eq!((aitd, bitd), (0, 0), "dead ahead and astern: no lag");
    assert_eq!((al, ar), (bl, br), "ahead and behind as loud");
    assert!(bcl + 60 < acl && bcr + 60 < acr, "behind muffled: ahead ({acl}, {acr}), behind ({bcl}, {bcr})");
    // Turned to face east, the same source north is now on the left.
    let (l, r, _, _, _) = place(0, 30, 0, 0x4000, 0, false);
    assert!(l > r, "turned: ({l}, {r})");
    let near = place(0, 20, 0, 0, 0, false);
    let far = place(0, 300, 0, 0, 0, false);
    assert!(far.0 < near.0 && far.2 < near.2, "far quieter and duller: {near:?} {far:?}");
    assert!(place(0, 300, 0, 0, 0, true).0 <= far.0, "the battle on the horizon quieter still");
}

/// The sound effects' pairs: two sounds at once; a near explosion takes a pair from a far muzzle flash,
/// never the other way round; the lock sounds once a lock.
#[test]
fn the_sound_pairs_go_to_the_loudest_and_most_important() {
    use crate::combat::{self, FAR, spawn};
    use crate::effects::effect::{EXPLOSION, MUZZLE};
    use crate::sfx::{self, PAIR, STATE, pair, patch, state};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    sfx::update(&mut m, &a);
    let playing = |m: &Machine| -> Vec<u8> { (0..sfx::PAIRS).map(|p| m.ram[(STATE + state::PAIRS + p * PAIR + pair::PATCH) as usize]).collect() };
    assert_eq!(playing(&m), [0, 0], "silent at first");
    // Two far muzzle flashes: both pairs.
    a.t += 4;
    spawn(&mut m, MUZZLE | FAR, 1024 << 4, 1400 << 4, 0, a.t as u16 - 1);
    spawn(&mut m, MUZZLE | FAR, 1100 << 4, 1400 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), [patch::MUZZLE + 1; 2]);
    // A near explosion takes one.
    a.t += 4;
    spawn(&mut m, EXPLOSION, 1030 << 4, 1110 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert!(playing(&m).contains(&(patch::EXPLOSION + 1)), "{:?}", playing(&m));
    // Both pairs exploding near: a far flash gets neither.
    a.t += 4;
    spawn(&mut m, EXPLOSION, 1020 << 4, 1105 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), [patch::EXPLOSION + 1; 2]);
    a.t += 4;
    spawn(&mut m, MUZZLE | FAR, 1024 << 4, 1400 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), [patch::EXPLOSION + 1; 2], "the far flash took a pair from an explosion");
    // The lock: once, as it completes.
    let locks = |m: &mut Machine, a: &mut crate::agent::Agent, ticks: u8| {
        a.t += 200;
        m.ram[crate::hud::LOCK_STATE as usize + 2] = ticks;
        sfx::update(m, a);
        playing(m).contains(&(patch::LOCK + 1))
    };
    assert!(!locks(&mut m, &mut a, 40), "not locked yet");
    assert!(locks(&mut m, &mut a, crate::hud::LOCK_TICKS), "locked");
    assert!(!locks(&mut m, &mut a, crate::hud::LOCK_TICKS), "only once a lock");
}

/// The player's own sounds (gun, launch, hit, lock) keep to one pair, and the world's effects keep the
/// other: the gun's bursts go onto the pair already firing, a near explosion never takes it, a launch cuts
/// in over the gun on that same pair, a burst while the launch is held is not heard, and the gun is back
/// once the launch's gate is released; a hit and the lock land there too.
#[test]
fn the_players_sounds_keep_to_one_pair_and_a_launch_wins_over_the_gun() {
    use crate::combat::{self, MISSILE_TTL, PLAYER, PROJ, PROJ_ENTRY, PROJ_MAX, TRACER_TTL, player, proj, spawn};
    use crate::effects::effect::EXPLOSION;
    use crate::sfx::{self, PAIR, PATCHES, STATE, field, pair, patch, state};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    sfx::update(&mut m, &a);
    let playing = |m: &Machine| -> Vec<u8> { (0..sfx::PAIRS).map(|p| m.ram[(STATE + state::PAIRS + p * PAIR + pair::PATCH) as usize]).collect() };
    // One new projectile of `kind` this frame (the others gone).
    let fire = |m: &mut Machine, kind: u8| {
        for k in 0..PROJ_MAX {
            m.ram[(PROJ + k * PROJ_ENTRY + proj::KIND) as usize] = 0;
        }
        if kind != 0 {
            m.ram[(PROJ + proj::KIND) as usize] = kind;
            m.ram[(PROJ + proj::TTL) as usize] = if kind == crate::actors::kind::MISSILE { MISSILE_TTL } else { TRACER_TTL };
        }
    };
    // A near explosion: the world's pair.
    a.t += 6;
    spawn(&mut m, EXPLOSION, 1030 << 4, 1110 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    let world = playing(&m).iter().position(|&id| id == patch::EXPLOSION + 1).expect("the explosion sounds") as usize;
    let own = 1 - world;
    // A burst a frame: on the other pair, and only there.
    for k in 0..4 {
        a.t += 6;
        fire(&mut m, crate::actors::kind::TRACER);
        sfx::update(&mut m, &a);
        assert_eq!(playing(&m)[own], patch::GUN + 1, "burst {k}: {:?}", playing(&m));
        assert_eq!(playing(&m)[world], patch::EXPLOSION + 1, "burst {k}: the world's pair taken");
    }
    // Another near explosion: onto the world's pair, never the gun's.
    a.t += 6;
    fire(&mut m, crate::actors::kind::TRACER);
    spawn(&mut m, EXPLOSION, 1020 << 4, 1105 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), { let mut v = [0; 2]; v[own] = patch::GUN + 1; v[world] = patch::EXPLOSION + 1; v });
    // A missile, the gun still firing: the launch takes the gun's pair, the world's untouched.
    a.t += 6;
    fire(&mut m, crate::actors::kind::MISSILE);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m)[own], patch::LAUNCH + 1, "the launch over the gun");
    assert_eq!(playing(&m)[world], patch::EXPLOSION + 1, "the launch kept to the player's pair");
    // A burst while the launch is held: not heard.
    let gate = PATCHES[patch::LAUNCH as usize][field::GATE] as u32;
    a.t += gate - 2;
    fire(&mut m, crate::actors::kind::TRACER);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m)[own], patch::LAUNCH + 1, "the gun over a held launch");
    // Once the launch's gate is released, the gun is back.
    a.t += 4;
    fire(&mut m, crate::actors::kind::TRACER);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m)[own], patch::GUN + 1, "the gun after the launch's attack");
    // A hit on the player, then the lock: on the player's pair too.
    a.t += 4;
    fire(&mut m, 0);
    m.ram[(PLAYER + player::FLASH) as usize] = 5;
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m)[own], patch::HIT + 1, "the hit on the player's pair");
    a.t += PATCHES[patch::HIT as usize][field::GATE] as u32 + 1;
    m.ram[crate::hud::LOCK_STATE as usize + 2] = crate::hud::LOCK_TICKS;
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m)[own], patch::LOCK + 1, "the lock on the player's pair");
    assert_eq!(playing(&m)[world], patch::EXPLOSION + 1, "the world's pair kept throughout");
}

/// After a stall nothing recent reads as new (every live tracer and effect would); the next frame's are.
#[test]
fn a_long_stall_replays_no_sounds() {
    use crate::combat::{self, PROJ, TRACER_TTL, proj, spawn};
    use crate::sfx::{self, PAIR, STATE, pair, patch, state};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    sfx::update(&mut m, &a);
    let playing = |m: &Machine| -> Vec<u8> { (0..sfx::PAIRS).map(|p| m.ram[(STATE + state::PAIRS + p * PAIR + pair::PATCH) as usize]).collect() };
    // A tracer and an explosion of a few ticks back, then a frame 300 ticks on.
    m.ram[PROJ as usize + proj::KIND as usize] = crate::actors::kind::TRACER;
    m.ram[PROJ as usize + proj::TTL as usize] = TRACER_TTL - 5;
    spawn(&mut m, crate::effects::effect::EXPLOSION, 1030 << 4, 1110 << 4, 0, 1295);
    a.t += 300;
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), [0, 0], "a stall: nothing new");
    // A fresh tracer the next frame plays.
    a.t += 6;
    m.ram[PROJ as usize + proj::TTL as usize] = TRACER_TTL;
    sfx::update(&mut m, &a);
    assert!(playing(&m).contains(&(patch::GUN + 1)), "{:?}", playing(&m));
}

/// The frame's vehicles in `squad::MEMBERS` for the sound tests: (vehicle, kind, u, v) in texels, in that
/// order (enemy side; the distance byte left 0: the engines measure their own).
fn list_vehicles(m: &mut Machine, vehicles: &[(u16, u8, u16, u16)]) {
    use crate::squad::{ENTRIES, ENTRY, MEMBERS, ORDER, entry};
    m.ram[MEMBERS as usize] = vehicles.len() as u8;
    for (k, &(vid, kind, u, v)) in vehicles.iter().enumerate() {
        let e = (ENTRIES + k as u16 * ENTRY) as usize;
        m.ram[ORDER as usize + k] = (k as u16 * ENTRY) as u8;
        m.ram[e + entry::VID as usize..][..2].copy_from_slice(&vid.to_le_bytes());
        m.ram[e + entry::MESH as usize] = crate::actors::mesh_of(kind, 0);
        m.ram[e + entry::U as usize..][..2].copy_from_slice(&(u << 4).to_le_bytes());
        m.ram[e + entry::V as usize..][..2].copy_from_slice(&(v << 4).to_le_bytes());
        m.ram[e + entry::H as usize..][..2].copy_from_slice(&0u16.to_le_bytes());
    }
}

/// The engines: the two nearest vehicles that have one sound on the pairs (a SAM site makes none); the
/// same vehicle keeps its pair (not restarted), its revs rising as it moves; a one-shot takes a pair from
/// an engine, which is back once it is done; a vehicle gone from the list is stopped.
#[test]
fn the_nearest_vehicles_engines_sound_and_give_way_to_effects() {
    use crate::actors::kind::{GUNBOAT, SAM, TANK, TRUCK};
    use crate::combat::{self, spawn};
    use crate::sfx::{self, PAIR, STATE, pair, patch, state};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    let at = |p: u16| (STATE + state::PAIRS + p * PAIR) as usize;
    let playing = |m: &Machine| -> Vec<u8> { (0..sfx::PAIRS).map(|p| m.ram[at(p) + pair::PATCH as usize]).collect() };
    let vid = |m: &Machine, p: u16| u16::from_le_bytes([m.ram[at(p) + pair::VID as usize], m.ram[at(p) + pair::VID as usize + 1]]);
    // A SAM site nearest, a tank behind the player, a truck ahead, a gunboat far off.
    list_vehicles(&mut m, &[(1, SAM, 1024, 1105), (2, TANK, 1024, 1090), (3, TRUCK, 1030, 1112), (4, GUNBOAT, 1024, 1300)]);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), [patch::TANK + 1, patch::TRUCK + 1], "the two nearest with engines");
    assert_eq!((vid(&m, 0), vid(&m, 1)), (2, 3));
    // The tank moving on (2 texels in 4 ticks: 8/16 texel a tick, revs up a step a frame; restarted, they
    // would be back at the start's).
    for k in 1..=3u16 {
        a.t += 4;
        list_vehicles(&mut m, &[(1, SAM, 1024, 1105), (2, TANK, 1024, 1090 + 2 * k), (3, TRUCK, 1030, 1112), (4, GUNBOAT, 1024, 1300)]);
        sfx::update(&mut m, &a);
    }
    assert_eq!(m.ram[at(0) + pair::SCORE as usize], sfx::REVS_START + 12, "its revs drifting up a tick's worth a tick");
    assert_eq!(m.ram[at(1) + pair::SCORE as usize], sfx::REVS_START - 12, "the truck's drifting down, standing");
    // A near explosion takes a pair; the engine is back once it is done.
    a.t += 4;
    spawn(&mut m, crate::effects::effect::EXPLOSION, 1030 << 4, 1110 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert!(playing(&m).contains(&(patch::EXPLOSION + 1)), "{:?}", playing(&m));
    a.t += 200;
    sfx::update(&mut m, &a);
    sfx::update(&mut m, &a);
    let mut now = playing(&m);
    now.sort();
    assert_eq!(now, [patch::TRUCK + 1, patch::TANK + 1], "the engines back");
    // The tank gone (out of the list): the gunboat in its place.
    a.t += 4;
    list_vehicles(&mut m, &[(3, TRUCK, 1030, 1112), (4, GUNBOAT, 1024, 1300)]);
    sfx::update(&mut m, &a);
    let mut now = playing(&m);
    now.sort();
    assert_eq!(now, [patch::TRUCK + 1, patch::BOAT + 1]);
    // Nothing near: both stopped.
    a.t += 4;
    list_vehicles(&mut m, &[]);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), [0, 0]);
    // One engine: an explosion goes to the free pair, not the engine's.
    a.t += 4;
    list_vehicles(&mut m, &[(3, TRUCK, 1030, 1112)]);
    sfx::update(&mut m, &a);
    a.t += 4;
    spawn(&mut m, crate::effects::effect::EXPLOSION, 1030 << 4, 1110 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    let mut now = playing(&m);
    now.sort();
    assert_eq!(now, [patch::EXPLOSION + 1, patch::TRUCK + 1], "the engine left playing");
}

/// A platoon's vehicles changing places in the order keep its engine going (not restarted, revs kept):
/// the pair follows whichever member is listed.
#[test]
fn an_engine_follows_its_platoon_as_its_members_change_places() {
    use crate::actors::kind::TRUCK;
    use crate::sfx::{self, PAIR, STATE, pair, patch, state};
    use crate::world_sim::vehicle;
    let mut m = Machine::new(package());
    crate::combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    let at = |p: u16| (STATE + state::PAIRS + p * PAIR) as usize;
    let vid = |m: &Machine, p: u16| u16::from_le_bytes([m.ram[at(p) + pair::VID as usize], m.ram[at(p) + pair::VID as usize + 1]]);
    // One platoon of trucks (5), its members 0 and 1 nearest, then 2 and 3 (members 0 and 1 gone on).
    let (m0, m1, m2, m3) = (vehicle(5, 0), vehicle(5, 1), vehicle(5, 2), vehicle(5, 3));
    list_vehicles(&mut m, &[(m0, TRUCK, 1024, 1110), (m1, TRUCK, 1026, 1112)]);
    sfx::update(&mut m, &a);
    assert_eq!([m.ram[at(0)], m.ram[at(1)]], [patch::TRUCK + 1; 2]);
    let revs = [m.ram[at(0) + pair::SCORE as usize], m.ram[at(1) + pair::SCORE as usize]];
    // The next members listed a texel off (16/16: more than the frame's 4 ticks), all standing.
    a.t += 4;
    list_vehicles(&mut m, &[(m2, TRUCK, 1024, 1111), (m3, TRUCK, 1026, 1113)]);
    let mut log = SidLog { m: &mut m, writes: vec![] };
    sfx::update(&mut log, &a);
    assert!(!restarted(&log.writes), "not restarted");
    assert_eq!([m.ram[at(0)], m.ram[at(1)]], [patch::TRUCK + 1; 2]);
    let now = [m.ram[at(0) + pair::SCORE as usize], m.ram[at(1) + pair::SCORE as usize]];
    assert!(now[0] <= revs[0] && now[1] <= revs[1], "the handover's gap not taken for driving: revs {revs:?} then {now:?}");
    let mut now = [vid(&m, 0), vid(&m, 1)];
    now.sort();
    assert_eq!(now, [m2, m3], "following the members listed");
}

/// The carrier is a giant diesel: its own patch, a 30 Hz and a 36 Hz voice beating (a rumble), never
/// revving: the same pitch standing or sailing.
#[test]
fn the_carrier_chugs_like_a_giant_diesel() {
    use crate::actors::kind::CARRIER;
    use crate::sfx::{self, LEFT, STATE, pair, patch, state};
    let mut m = Machine::new(package());
    crate::combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    list_vehicles(&mut m, &[(9, CARRIER, 1024, 1110)]);
    sfx::update(&mut m, &a);
    let p0 = (STATE + state::PAIRS) as usize;
    assert_eq!(m.ram[p0 + pair::PATCH as usize], patch::DIESEL + 1);
    // Voices 1 and 2's frequencies (left chip), sailing on for a while then standing (the player aboard:
    // no Doppler).
    let mut seen = vec![];
    for k in 0..40u16 {
        a.t += 4;
        let v = if k < 20 { 1110 + 2 * k } else { 1148 };
        a.v = ((v - 10) as i32) << 8;
        list_vehicles(&mut m, &[(9, CARRIER, 1024, v)]);
        let mut log = SidLog { m: &mut m, writes: vec![] };
        sfx::update(&mut log, &a);
        let f = |r: u16| {
            let lo = log.writes.iter().rev().find(|(a, _)| *a == r).map_or(0, |w| w.1);
            let hi = log.writes.iter().rev().find(|(a, _)| *a == r + 1).map_or(0, |w| w.1);
            u16::from_le_bytes([lo, hi])
        };
        seen.push((f(LEFT), f(LEFT + 7)));
    }
    assert!(seen.iter().all(|&f| f == (0x0200, 0x0260)), "the diesel's pitch never moves: {seen:?}");
}

/// Whether any voice was held silent (its control register written with the test bit): a restart.
fn restarted(writes: &[(u16, u8)]) -> bool {
    writes.iter().any(|&(a, d)| (0xD400..0xD480).contains(&a) && [4, 11, 18].contains(&((a - 0xD400) % 0x20)) && d & 0x08 != 0)
}

/// The SID writes, in order, over a `Machine` (the raster line's reads logged too, as `$D012`, 0).
struct SidLog<'a> {
    m: &'a mut Machine,
    writes: Vec<(u16, u8)>,
}

impl crate::hw::Hw for SidLog<'_> {
    fn peek(&mut self, addr: u16) -> u8 {
        if addr == 0xD012 {
            self.writes.push((addr, 0));
        }
        self.m.peek(addr)
    }
    fn poke(&mut self, addr: u16, value: u8) {
        if (0xD400..0xD480).contains(&addr) {
            self.writes.push((addr, value));
        }
        self.m.poke(addr, value)
    }
    fn reu_exec(&mut self, command: u8) {
        self.m.reu_exec(command)
    }
}

/// The ears' lag: a sound on the right is gated in the right chip first and in the left one its lag in
/// raster lines later (one ahead: both at once); an engine's lag follows the player turning, the lagging
/// ear's chip run slow meanwhile (never restarted).
#[test]
fn a_sound_reaches_the_far_ear_later() {
    use crate::actors::kind::TANK;
    use crate::combat::{self, spawn};
    use crate::sfx::{self, ITD_MAX, LEFT, PAIR, RIGHT, STATE, pair, state};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    sfx::update(&mut m, &a);
    // The gates (a control register written with the gate bit) on either chip, and the raster reads
    // from the first to the last.
    let gates = |w: &[(u16, u8)]| -> (Vec<usize>, Vec<usize>, usize) {
        let ctrl = |a: u16| (0..sfx::PAIRS).any(|p| (0..3).any(|v| a == p * 0x20 + 7 * v + 4));
        let l: Vec<usize> = w.iter().enumerate().filter(|(_, (a, d))| *a >= LEFT && *a < RIGHT && ctrl(*a - LEFT) && d & 1 != 0).map(|(i, _)| i).collect();
        let r: Vec<usize> = w.iter().enumerate().filter(|(_, (a, d))| *a >= RIGHT && ctrl(*a - RIGHT) && d & 1 != 0).map(|(i, _)| i).collect();
        let (lo, hi) = (*l.first().unwrap().min(r.first().unwrap()), *l.last().unwrap().max(r.last().unwrap()));
        (l, r, w[lo..hi].iter().filter(|(a, _)| *a == 0xD012).count())
    };
    // An explosion abeam to the right (facing north), level with the player.
    a.t += 4;
    spawn(&mut m, crate::effects::effect::EXPLOSION, 1040 << 4, 1100 << 4, 10 << 4, a.t as u16 - 1);
    let mut log = SidLog { m: &mut m, writes: vec![] };
    sfx::update(&mut log, &a);
    let (l, r, lines) = gates(&log.writes);
    assert!(r.last() < l.first(), "the right ear first: left {l:?} right {r:?}");
    assert!(lines > ITD_MAX as usize, "the left ear {ITD_MAX} lines later: {lines} raster reads");
    // Dead ahead: no wait.
    a.t += 200;
    sfx::update(&mut m, &a);
    a.t += 4;
    spawn(&mut m, crate::effects::effect::EXPLOSION, 1024 << 4, 1110 << 4, 0, a.t as u16 - 1);
    let mut log = SidLog { m: &mut m, writes: vec![] };
    sfx::update(&mut log, &a);
    // (The wait reads the line once to begin with.)
    assert!(gates(&log.writes).2 <= 1, "ahead: both ears at once");
    // A tank's engine ahead, then the player (on the ground with it, level) turned to face west: it is
    // abeam on the right.
    a.h = 0;
    a.t += 200;
    sfx::update(&mut m, &a);
    list_vehicles(&mut m, &[(7, TANK, 1024, 1120)]);
    a.t += 4;
    sfx::update(&mut m, &a);
    let at = |p: u16| (STATE + state::PAIRS + p * PAIR) as usize;
    let p = (0..sfx::PAIRS).find(|&p| m.ram[at(p) + pair::VID as usize] == 7).expect("the tank's engine");
    let lag = |m: &Machine| m.ram[at(p) + pair::ITD as usize] as i8;
    assert_eq!(lag(&m), 0);
    a.yaw = 0xC000;
    // Uneven frames: a chip run slow is credited the next frame's ticks (as long as it really ran slow).
    let mut slowed = false;
    let mut slow_before = false;
    for dt in [3u16, 5, 4, 3, 5, 4, 4, 4] {
        a.t += dt as u32;
        let before = lag(&m);
        let mut log = SidLog { m: &mut m, writes: vec![] };
        sfx::update(&mut log, &a);
        // Voice 1's frequency as last written on each chip.
        let f = |c: u16| {
            let lo = log.writes.iter().rev().find(|(a, _)| *a == c).map_or(0, |w| w.1);
            let hi = log.writes.iter().rev().find(|(a, _)| *a == c + 1).map_or(0, |w| w.1);
            u16::from_le_bytes([lo, hi])
        };
        let (fl, fr) = (f(LEFT + p * 0x20), f(RIGHT + p * 0x20));
        assert!(!restarted(&log.writes), "not restarted");
        let credit = if slow_before { dt as i8 } else { 0 };
        assert_eq!(lag(log.m), before + credit, "the lag credited the ticks its chip ran slow");
        assert!(fl <= fr, "only the left ear's chip slowed: {fl:#x} {fr:#x}");
        slow_before = fl < fr;
        slowed |= slow_before;
    }
    assert!(slowed);
    assert!((ITD_MAX - 3..=ITD_MAX + 3).contains(&lag(&m)), "the lag drawn to abeam's (within half a frame's step): {}", lag(&m));
}

/// The ears' model, swept round the player (every bearing, near to far, level with it and below it,
/// level and banked): its cues agree and stay consistent.
/// - **Mirror-symmetric:** a source and its mirror image across the nose (the bank mirrored too) have
///   opposite lags, swapped levels and cutoffs.
/// - **Continuous:** a step of the bearing (1.4°) moves the lag a line at most, and the near ear's level
///   a step at most (the far one's two, at the 4-bit floor), from 30 texels out (nearer, the whole
///   texels of the offsets step the bearing by degrees).
/// - **Lateral:** the near ear is never quieter nor duller than the far one, and the lag's sign agrees.
/// - **Consistent with distance:** the lag and the shadow (the cutoffs' ratio) are the same near and far;
///   the pan never collapses to the centre while the near ear has 3 steps or more and the source is
///   well off the nose.
/// - **Independent:** the head's shadow (`cl / cr`) is the same octave ahead as behind.
/// - **Overhead:** a source passed straight under the player centres (no lag) and never flips side.
#[test]
fn a_sounds_place_is_consistent() {
    use crate::sfx::{ITD_MAX, place};
    let bearing = |k: u16, d: i16| -> (i16, i16) {
        let a = (k << 8) as u16;
        ((crate::camera::sin(a) * d as i32 / 16384) as i16, (crate::camera::cos(a) * d as i32 / 16384) as i16)
    };
    for &roll in &[0u16, 20 * 182] {
        for &dz in &[0i16, 20, 60] {
            for &d in &[8i16, 30, 100, 300, 700] {
                let mut last: Option<(u8, u8, u8, u8, i8)> = None;
                for k in 0..=256u16 {
                    let (dx, dy) = bearing(k & 255, d);
                    let p = place(dx, dy, dz, 0, roll, false);
                    let (l, r, cl, cr, itd) = p;
                    let mirror = place(-dx, dy, dz, 0, roll.wrapping_neg(), false);
                    assert_eq!(mirror, (r, l, cr, cl, -itd), "({dx}, {dy}, {dz}) roll {roll}: mirrored");
                    assert!(itd.abs() <= ITD_MAX);
                    if itd > 0 {
                        assert!(r >= l && cr >= cl, "({dx}, {dy}, {dz}) roll {roll}: on the right, the right ear nearer: {p:?}");
                    } else if itd < 0 {
                        assert!(l >= r && cl >= cr, "({dx}, {dy}, {dz}) roll {roll}: on the left: {p:?}");
                    }
                    // (Continuity from 30 texels: nearer, whole texels step the bearing by degrees.)
                    if let Some(q) = last.filter(|_| d >= 30) {
                        assert!((itd - q.4).abs() <= 1, "({dx}, {dy}, {dz}) roll {roll}: the lag jumps {q:?} -> {p:?}");
                        let near = l.max(r).abs_diff(q.0.max(q.1));
                        assert!(near <= 1, "({dx}, {dy}, {dz}) roll {roll}: the near ear jumps {q:?} -> {p:?}");
                        assert!(l.abs_diff(q.0) <= 2 && r.abs_diff(q.1) <= 2, "({dx}, {dy}, {dz}) roll {roll}: a level jumps {q:?} -> {p:?}");
                    }
                    last = Some(p);
                }
            }
        }
    }
    // Abeam to the right at every range, level with the player: the same lag and shadow; the far ear
    // quieter while the near ear has 3 steps or more.
    let abeam: Vec<_> = [8i16, 30, 60, 100, 200, 300].iter().map(|&d| place(d, 0, 0, 0, 0, false)).collect();
    let first = abeam[0].2 as u32 * 256 / abeam[0].3 as u32;
    for p in &abeam {
        assert_eq!(p.4, ITD_MAX, "abeam: the whole lag at every range: {abeam:?}");
        assert!(p.1 < 3 || p.0 < p.1, "abeam: the pan holds with range: {abeam:?}");
        let ratio = p.2 as u32 * 256 / p.3 as u32;
        assert!(ratio.abs_diff(first) <= 12, "the head's shadow the same near and far: {abeam:?}");
    }
    // Off the nose by 30° and more, at ranges where the near ear has 3 steps or more: never centred.
    for k in 22..=106u16 {
        for &d in &[20i16, 60, 120] {
            let (dx, dy) = bearing(k, d);
            let (l, r, _, _, itd) = place(dx, dy, 0, 0, 0, false);
            if r >= 3 {
                assert!(l < r && itd > 0, "({dx}, {dy}): a source on the right is heard there: ({l}, {r}) lag {itd}");
            }
        }
    }
    // The shadow ahead and behind: the same octave (the ratio within a step of the tables).
    let (_, _, acl, acr, _) = place(30, 30, 0, 0, 0, false);
    let (_, _, bcl, bcr, _) = place(30, -30, 0, 0, 0, false);
    let (ra, rb) = (acl as u32 * 256 / acr as u32, bcl as u32 * 256 / bcr as u32);
    assert!(ra.abs_diff(rb) <= 12, "the shadow the same ahead ({acl}/{acr}) as behind ({bcl}/{bcr})");
    // Passing over a source at 20 texels' height, from 40 texels to its left to 40 to its right: the lag
    // swings from the left to the right through 0 straight over it, a line at most a texel, never back.
    let mut last = place(-40, 0, 20, 0, 0, false).4;
    for x in -40..=40i16 {
        let itd = place(x, 0, 20, 0, 0, false).4;
        assert!(itd >= last && itd - last <= 1, "overhead at {x}: the lag {last} -> {itd}");
        if x == 0 {
            assert_eq!(itd, 0, "straight under: centred");
        }
        last = itd;
    }
    // Banked right, a source straight below is heard on the right (the ears tilt with the view).
    let (l, r, _, _, itd) = place(0, 0, 30, 0, 20 * 182, false);
    assert!(itd > 0 && r > l, "banked right: below is on the right: ({l}, {r}) lag {itd}");
}

/// The ears' tables are shaped as their documents say: the lag Woodworth's (each 16th's mean, to a
/// line), every table rising (the lag a line a step), and the volumes the nearest by ratio.
#[test]
fn the_ears_tables_are_a_heads() {
    use crate::sfx::{ILD, ITD_LINES, ITD_MAX, SHADOW, VOL};
    let ww = |x: f64| x.min(1.0).asin() + x.min(1.0);
    let mean = |i: usize| {
        let (lo, hi) = (((i as f64 - 0.5) / 16.0).max(0.0), ((i as f64 + 0.5) / 16.0).min(1.0));
        (0..200).map(|k| ww(lo + (hi - lo) * (k as f64 + 0.5) / 200.0)).sum::<f64>() / 200.0
    };
    for i in 0..17 {
        assert_eq!(ITD_LINES[i] as f64, (ITD_MAX as f64 * mean(i) / mean(16)).round(), "the lag at {i}/16");
        assert_eq!(ILD[i] as usize, (28 * i + 8) / 16, "the level gap at {i}/16");
        if i > 0 {
            assert!(ITD_LINES[i] - ITD_LINES[i - 1] <= 1 && ILD[i] >= ILD[i - 1] && SHADOW[i] >= SHADOW[i - 1]);
        }
    }
    for (a, &v) in VOL.iter().enumerate() {
        let t = 15.0 * 2f64.powf(-(a as f64) / 16.0);
        let best = (1..=15u8).min_by(|&x, &y| (x as f64 / t).ln().abs().total_cmp(&(y as f64 / t).ln().abs())).unwrap();
        assert_eq!(v, best, "the volume at {a}/16 octave down");
    }
    assert!(15.0 * 2f64.powf(-(VOL.len() as f64) / 16.0) < 0.75, "silent past the table");
}

/// The IRQ's slews (the host's copy): each value glides toward its target, a step at least and never
/// past it, onto its own register (`asm/hud.S` `sfx_reg`), within half a second.
#[test]
fn the_sound_slews_glide_to_their_targets() {
    use crate::sfx::{SLEW, SLEW_CH, slew};
    let mut m = Machine::new(package());
    let (tg, cur) = (SLEW as usize, (SLEW + SLEW_CH) as usize);
    let targets = [0x1F, 0xC0, 0x10, 0x40, 0x13, 0xFF, 0x1A, 0x01];
    for ch in 0..SLEW_CH as usize {
        m.ram[tg + ch] = targets[ch];
        m.ram[cur + ch] = 0;
    }
    m.ram[cur + 7] = 0xFF;
    let mut frames = 0;
    let mut log = SidLog { m: &mut m, writes: vec![] };
    while (0..SLEW_CH as usize).any(|ch| log.m.ram[cur + ch] != targets[ch]) {
        let before: Vec<u8> = log.m.ram[cur..cur + SLEW_CH as usize].to_vec();
        slew(&mut log);
        let m = &log.m;
        for ch in 0..SLEW_CH as usize {
            let (b, n, t) = (before[ch] as i16, m.ram[cur + ch] as i16, targets[ch] as i16);
            assert!(b == t || ((n - b).signum() == (t - b).signum() && (t - n).abs() < (t - b).abs()), "ch {ch}: {b} -> {n} toward {t}");
        }
        frames += 1;
        assert!(frames < 30, "glides there within half a second");
    }
    let regs = [0xD418u16, 0xD416, 0xD458, 0xD456, 0xD438, 0xD436, 0xD478, 0xD476];
    for (ch, &r) in regs.iter().enumerate() {
        let last = log.writes.iter().rev().find(|w| w.0 == r).map(|w| w.1);
        assert_eq!(last, Some(targets[ch]), "ch {ch} onto ${r:04X} (asm/hud.S sfx_reg)");
    }
}

/// A raster line scripted: `$D012` reads the lines given in turn (the last one held).
struct Raster<'a> {
    m: &'a mut Machine,
    lines: Vec<u8>,
    reads: usize,
}

impl crate::hw::Hw for Raster<'_> {
    fn peek(&mut self, addr: u16) -> u8 {
        if addr == 0xD012 {
            self.reads += 1;
            return self.lines[(self.reads - 1).min(self.lines.len() - 1)];
        }
        self.m.peek(addr)
    }
    fn poke(&mut self, addr: u16, value: u8) {
        self.m.poke(addr, value)
    }
    fn reu_exec(&mut self, command: u8) {
        self.m.reu_exec(command)
    }
}

/// The ears' wait counts the lines that pass, not the changes it sees: the raster IRQ taking 15 lines at
/// once ends a 10-line wait; the frame's wrap (311 to 0: its low byte 55 to 0) is one line.
#[test]
fn the_ears_wait_counts_lines_passed() {
    let mut m = Machine::new(package());
    let mut r = Raster { m: &mut m, lines: vec![100, 101, 116, 117, 118], reads: 0 };
    crate::sfx::wait(&mut r, 10);
    assert_eq!(r.reads, 3, "done once the IRQ's 15 lines are by");
    let mut r = Raster { m: &mut m, lines: vec![53, 54, 55, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9], reads: 0 };
    crate::sfx::wait(&mut r, 5);
    assert_eq!(r.reads, 6, "the wrap a line");
}

/// Doppler: a standing truck dead ahead, the player flying at it then past it: its pitch above its
/// revs' while closing, below once opening, and at them hovering still.
#[test]
fn an_engines_pitch_rises_closing_and_falls_passing() {
    use crate::actors::kind::TRUCK;
    use crate::sfx::{self, ENGINES, LEFT, STATE, field, pair, patch, state};
    let mut m = Machine::new(package());
    crate::combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1060 << 8, 10 << 8, 1000);
    list_vehicles(&mut m, &[(3, TRUCK, 1024, 1100)]);
    sfx::update(&mut m, &a);
    let s = (STATE + state::PAIRS) as usize;
    assert_eq!(m.ram[s + pair::PATCH as usize], patch::TRUCK + 1);
    let t = &ENGINES[(patch::TRUCK - patch::ENGINE) as usize];
    // A frame (4 ticks) with the player at `v` (texels): voice 1's frequency on the left chip, and its
    // revs' alone (no Doppler).
    let frame = |m: &mut Machine, a: &mut crate::agent::Agent, v: i32| {
        a.t += 4;
        a.v = v << 8;
        let mut log = SidLog { m, writes: vec![] };
        sfx::update(&mut log, a);
        assert!(!restarted(&log.writes), "never restarted");
        let lo = log.writes.iter().rev().find(|(r, _)| *r == LEFT).map_or(0, |w| w.1);
        let hi = log.writes.iter().rev().find(|(r, _)| *r == LEFT + 1).map_or(0, |w| w.1);
        let revs = log.m.ram[s + pair::SCORE as usize] as u16;
        let base = (t[0] as u16) << 8 | t[5] as u16;
        (u16::from_le_bytes([lo, hi]), base + ((t[0] as u16 * t[field::REV_DEPTH] as u16 * revs) >> 4))
    };
    // Hovering: at its revs' pitch.
    for _ in 0..4 {
        let (f, revs) = frame(&mut m, &mut a, 1060);
        assert_eq!(f, revs, "still: no Doppler");
    }
    // Flying north at 0.5 texel a tick (2 a frame): closing, then past it and opening.
    let mut dops = vec![];
    let mut v = 1060;
    for _ in 0..40 {
        v += 2;
        let (f, revs) = frame(&mut m, &mut a, v);
        let dop = m.ram[s + pair::DOP as usize] as i8;
        dops.push(dop);
        assert_eq!(f.cmp(&revs), dop.cmp(&0), "pitch {f:#x} against the revs' {revs:#x}, Doppler {dop}");
    }
    let (top, bottom) = (*dops.iter().max().unwrap(), *dops.iter().min().unwrap());
    // 2 × (32/16 texel closing in a 4-tick frame: 8/16 a tick) = 16, either way.
    assert!((12..=sfx::DOP_MAX).contains(&top), "closing: {dops:?}");
    assert!((-sfx::DOP_MAX..=-12).contains(&bottom), "opening: {dops:?}");
    let up = dops.iter().position(|&d| d == top).unwrap();
    let down = dops.iter().position(|&d| d == bottom).unwrap();
    assert!(up < down, "up closing, then down past it: {dops:?}");
}

/// Engines are not given up to the small stuff: a near crackle and the far battle's gunfire leave
/// them be; a near explosion takes one.
#[test]
fn engines_give_way_only_to_near_sounds_of_note() {
    use crate::actors::kind::{TANK, TRUCK};
    use crate::combat::{self, FAR, spawn};
    use crate::effects::effect::{EXPLOSION, MUZZLE, WRECK_FIRE};
    use crate::sfx::{self, PAIR, STATE, pair, patch, state};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    let playing = |m: &Machine| -> Vec<u8> {
        let mut v: Vec<u8> = (0..sfx::PAIRS).map(|p| m.ram[(STATE + state::PAIRS + p * PAIR + pair::PATCH) as usize]).collect();
        v.sort();
        v
    };
    list_vehicles(&mut m, &[(2, TANK, 1024, 1090), (3, TRUCK, 1030, 1112)]);
    sfx::update(&mut m, &a);
    let engines = vec![patch::TRUCK + 1, patch::TANK + 1];
    assert_eq!(playing(&m), engines);
    a.t += 4;
    spawn(&mut m, WRECK_FIRE, 1026 << 4, 1104 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), engines, "a crackle");
    a.t += 4;
    spawn(&mut m, MUZZLE | FAR, 1024 << 4, 1300 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert_eq!(playing(&m), engines, "the far battle");
    a.t += 4;
    spawn(&mut m, EXPLOSION, 1030 << 4, 1110 << 4, 0, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    assert!(playing(&m).contains(&(patch::EXPLOSION + 1)), "a near explosion: {:?}", playing(&m));
}

#[test]
fn nothing_draws_into_the_hidden_column_39() {
    // Bitmap column 39 never shows in the 38-column window (`layout::DRAWN_COLS`): the floor, the
    // triangles, the sprites, the band publish and the cockpit's gauges must leave it alone. A sentinel
    // in every cell row's column 39, then whole frames (ground, lift-off, a heavy one; full and doubled).
    use crate::hw::Hw;
    use crate::layout::{DRAWN_COLS, ROW_STRIDE, ROWS};
    let reu = package();
    let bitmap = BUFFERS[0].bitmap;
    let col39 = |r: u16, l: u16| bitmap + r * ROW_STRIDE + DRAWN_COLS * 8 + l;
    for t in [0, crate::agent::LIFT + 200, 9300, 9315] {
        for doubled in [false, true] {
            let mut m = Machine::new(reu);
            crate::package::setup(&mut m, BUFFERS[0]);
            for r in 0..ROWS {
                for l in 0..8 {
                    m.poke(col39(r, l), 0xA5 ^ (r * 8 + l) as u8);
                }
            }
            let agent = crate::agent::at(reu, t);
            crate::cockpit::update(&mut m, &agent, &mut [0xFF; crate::cockpit::gauge::COUNT]);
            let (cam, _) = agent.camera();
            crate::hud::init(&mut m);
            crate::hud::update(&mut m, &agent, &cam);
            crate::actors::update(&mut m, &cam);
            let st = crate::package::render_floor(&mut m, BUFFERS[0], &crate::package::scene_block(reu), &cam, if doubled { Lines::Doubled } else { Lines::Full });
            for r in 0..ROWS {
                for l in 0..8 {
                    assert_eq!(m.peek(col39(r, l)), 0xA5 ^ (r * 8 + l) as u8, "tick {t} doubled {doubled}: cell row {r} line {l}");
                }
            }
            println!("tick {t} doubled {doubled}: {} triangles, column 39 untouched", st.tris);
        }
    }
}

#[test]
fn the_fog_line_and_the_horizon_are_painted_to_the_pixel() {
    // Random rolled cameras: after the runs and the fog patches, every pixel of every line is sky,
    // haze or ground exactly as its own depth says (`floor::FogArgs`: ground nearer than the fog depth,
    // haze above the horizon), give or take the thresholds' rounding: the edges are straight lines at
    // multicolour-pixel resolution, not byte steps. (A haze byte of code 2 everywhere tells haze from
    // sky's code 1.)
    let mut rng = Rng(0x0bad_cafe);
    let deg = |d: i32| (d * 182) as u16;
    const HAZE: u8 = 0xAA;
    let (mut painted, mut pixels, mut unpainted) = (0u32, 0u32, 0u32);
    for _ in 0..300 {
        let yaw = rng.next() as u16;
        let pitch = deg(rng.below(40) as i32 - 30);
        let roll = deg(rng.below(121) as i32 - 60);
        let m = crate::camera::Matrix14::orientation(yaw, pitch, roll);
        let cam = FloorCam { u: (300 << 8) + rng.below(1400 << 8) as i32, v: (300 << 8) + rng.below(1400 << 8) as i32, h: (2 * 256 + rng.below(120 * 256)) as u16, r: m.m };
        let rays = Rays::new(&cam);
        assert_eq!(floor::FogArgs::of_cam(&cam), rays.fog());
        if rays.fog().dir == 0 && rays.dw[0] != 0 {
            // (Within about half a degree of level: the lines jump 64 pixels or more a line, unpainted.)
            unpainted += 1;
            continue;
        }
        let dw0 = rays.dw[0] as f64;
        let fog_w = (rays.w_haze + 2 * rays.dw[0].abs()) as f64;
        let mut spans = [Span::default(); MAX_SPANS];
        let mut fog = floor::Patches::default();
        for y in 0..VIEW_LINES {
            let a = rays.line(y);
            let n = floor::make_line(&cam, &rays, &a, HAZE, rays.fog().at(y), &mut spans, &mut fog).0;
            assert!(fog.n as usize <= floor::MAX_PATCHES);
            painted += fog.n as u32;
            // Per pixel: 0 sky, 1 haze, 2 ground, as drawn.
            let mut px = [0u8; 4 * floor::DRAWN as usize];
            let mut b = 0usize;
            for s in &spans[..n] {
                for i in 0..s.len as usize {
                    for p in 0..4 {
                        px[4 * (b + i) + p] = match s.fill() {
                            None => 2,
                            Some(f) => if f == HAZE { 1 } else { assert_eq!(f, floor::SKY_BYTE); 0 },
                        };
                    }
                }
                b += s.len as usize;
            }
            for &[at, keep, bits] in &fog.p[..fog.n as usize] {
                for p in 0..4 {
                    let sh = 6 - 2 * p;
                    if keep >> sh & 3 == 0 {
                        px[4 * at as usize + p] = match bits >> sh & 3 { 2 => 1, 1 => 0, c => panic!("paint code {c}") };
                    } else {
                        assert_eq!(keep >> sh & 3, 3);
                    }
                }
            }
            for (i, &got) in px.iter().enumerate() {
                let (bb, p) = (i / 4, (i % 4) as f64);
                let wc = -(a[1] + floor::mul_small(rays.bb[1], bb as u8)) as f64;
                let w = wc - (2.0 * p - 3.0) * dw0 / 8.0;
                let want = if w >= fog_w { 2 } else if w > 0.0 { 1 } else { 0 };
                // (16.16 crossings: within a sixteenth of a pixel.)
                let tol = dw0.abs() / 64.0 + 2.0;
                let near = (w - fog_w).abs() <= tol || w.abs() <= tol;
                assert!(got == want || near, "camera {cam:?} line {y} pixel {i}: drawn {got}, depth {w} says {want} (fog {fog_w})");
                pixels += 1;
            }
        }
    }
    assert!(painted > 5_000 && unpainted < 10, "{painted} {unpainted}");
    println!("{pixels} pixels, {painted} painted bytes, {unpainted} cameras unpainted");
}

/// The second line of a doubled pair (`floor::second_line`) has its own fog line and horizon to the
/// pixel, as its own depth says, over the first line's texels. Painting only turns ground to haze or
/// sky, so where this line's ground would reach past the first's the edge stays at the first line's
/// (counted, rare in the pairs adaptive doubling picks); ground never shows where it should not. The
/// haze is in the other phase. (Ground drawn as code 0, the first line's haze 0xAA as code 2, so the
/// second's, 0xFF, is code 3.)
#[test]
fn a_doubled_pairs_second_line_has_its_own_fog_line() {
    use crate::hw::Hw;
    let mut rng = Rng(0x0f06_5eed);
    let deg = |d: i32| (d * 182) as u16;
    const HAZE1: u8 = 0xAA;
    const HAZE2: u8 = HAZE1 ^ (floor::HAZE_BYTE ^ floor::HAZE_ODD);
    const BUF: u16 = crate::layout::LINEBUF;
    let mut m = Machine::new(&[]);
    let (mut pixels, mut held, mut pairs) = (0u32, 0u32, 0u32);
    // (An adaptive pair's fog edge moves too little for a fifth patch: `second_line` keeps to 4.)
    for _ in 0..400 {
        let yaw = rng.next() as u16;
        let pitch = deg(rng.below(60) as i32 - 45);
        let roll = deg(rng.below(181) as i32 - 90);
        let m14 = crate::camera::Matrix14::orientation(yaw, pitch, roll);
        let cam = FloorCam { u: (300 << 8) + rng.below(1400 << 8) as i32, v: (300 << 8) + rng.below(1400 << 8) as i32, h: (2 * 256 + rng.below(120 * 256)) as u16, r: m14.m };
        let rays = Rays::new(&cam);
        let fa = rays.fog();
        if fa.dir == 0 {
            continue;
        }
        let mask = adaptive_pairs(&mut m, &cam);
        let dw0 = rays.dw[0] as f64;
        let fog_w = (rays.w_haze + 2 * rays.dw[0].abs()) as f64;
        let mut spans = [Span::default(); MAX_SPANS];
        let mut fog = floor::Patches::default();
        for y in (0..VIEW_LINES).step_by(2) {
            if !floor::doubled(mask[(y / 8) as usize], (y % 8) as u8) {
                continue;
            }
            let (n, gb) = floor::make_line(&cam, &rays, &rays.line(y), HAZE1, fa.at(y), &mut spans, &mut fog);
            let mut x = 0u16;
            for s in &spans[..n] {
                for i in 0..s.len as u16 {
                    m.poke(BUF + x + i, s.fill().unwrap_or(0));
                }
                x += s.len as u16;
            }
            floor::apply_fog(&mut m, &fog, BUF);
            let fits = floor::second_line(&mut m, &spans[..n], &mut fog, fa.dir, gb, HAZE2, fa.at(y + 1), BUF);
            assert!(fits, "camera {cam:?} line {}: a fifth fog patch on an adaptive pair", y + 1);
            pairs += 1;
            let a = rays.line(y + 1);
            for i in 0..4 * floor::DRAWN as usize {
                let (bb, p) = (i / 4, i % 4);
                let code = m.peek(BUF + bb as u16) >> (6 - 2 * p) & 3;
                let got = match code { 0 => 2, 3 => 1, 1 => 0, _ => panic!("line {y}: the first line's haze left at byte {bb}") };
                let wc = -(a[1] + floor::mul_small(rays.bb[1], bb as u8)) as f64;
                let w = wc - (2.0 * p as f64 - 3.0) * dw0 / 8.0;
                let want = if w >= fog_w { 2 } else if w > 0.0 { 1 } else { 0 };
                let tol = dw0.abs() / 64.0 + 2.0;
                let near = (w - fog_w).abs() <= tol || w.abs() <= tol;
                pixels += 1;
                if got == want || near {
                    continue;
                }
                assert!(got != 2, "camera {cam:?} line {}: ground at pixel {i} where the depth says {want}", y + 1);
                held += 1;
            }
        }
    }
    println!("{pairs} doubled pairs, {pixels} second-line pixels, {held} held at the first line's edge");
    assert!(pairs > 2_000 && (held as f64) < pixels as f64 * 0.002, "{held} of {pixels}");
}

/// The clouds on the horizon: a level camera sees two or more (the ring is ~35° apart, the view ~90°),
/// each standing 12-22 lines above the horizon ahead (`geom::CY`; higher off the axis), in the farthest
/// depth bucket, level; none behind it; and the ring drifts east (to the right, facing north) with the u32
/// clock, smoothly across the u16 tick's wrap.
#[test]
fn the_clouds_stand_on_the_horizon_and_drift() {
    use crate::clouds::{DRIFT_SHIFT, KEY, LEVEL, place, record};
    use crate::combat::{FXDYN, FXDYN_ENTRY};
    let list = |cam: &FloorCam, t: u32| -> Vec<(i16, i16)> {
        let mut m = Machine::new(package());
        let n = place(&mut m, &crate::hud::Eye::new(cam), (t >> DRIFT_SHIFT) as u16, 0);
        (0..n as u16)
            .map(|k| {
                let at = FXDYN + 1 + k * FXDYN_ENTRY;
                assert_eq!(m.ram[(at + 7) as usize], KEY, "the farthest bucket");
                let rec = rd16(&m, at + 4) as u32 | (m.ram[(at + 6) as usize] as u32) << 16;
                assert!((0..4).any(|s| rec == record(s, LEVEL)), "a level cloud's record: ${rec:06X}");
                let r = crate::effects_bake::read_record_at(&m.reu, rec);
                assert!(r.h > 0 && r.phases == 4, "a baked cloud");
                (rd16(&m, at) as i16, rd16(&m, at + 2) as i16)
            })
            .collect()
    };
    let cy = crate::geom::CY as i16;
    for yaw in (0..8u16).map(|k| k << 13) {
        let cam = camera_at(1024.0, 1100.0, 10.0, yaw, 0, 0);
        let seen = list(&cam, 0);
        assert!((2..=4).contains(&seen.len()), "yaw {yaw:#06x}: {seen:?}");
        for &(x, y) in &seen {
            // (12-22 lines up ahead, and 1/cos of the bearing off the axis more toward the sides.)
            assert!((cy - 42..cy - 11).contains(&y), "yaw {yaw:#06x}: base {y}, horizon {cy}");
            // (Kept while a frame can still reach the screen: `clouds::REACH_X` pixels either side.)
            let r = crate::clouds::REACH_X;
            assert!((-r..160 + r).contains(&x), "yaw {yaw:#06x}: x {x}");
        }
    }
    // Matched by their base line (one may leave or enter at an edge; a base rises a little off the axis):
    // 3000 ticks move them right a little,
    // across the u16 clock's wrap as anywhere.
    let cam = camera_at(1024.0, 1100.0, 10.0, 0, 0, 0);
    for t in [1000u32, 65536 - 1500] {
        let (a, b) = (list(&cam, t), list(&cam, t + 3000));
        let inner: Vec<_> = a.iter().filter(|p| (0..150).contains(&p.0)).collect();
        assert!(!inner.is_empty());
        assert!(inner.iter().all(|p| b.iter().any(|q| (q.1 - p.1).abs() <= 4 && (1..12).contains(&(q.0 - p.0)))), "{t}: {a:?} -> {b:?}");
    }
}

/// The clouds roll with the horizon: `clouds::angle` picks the level frame for a level camera, and for a
/// rolled one the step nearest the roll (the horizon's tilt on the screen), turned the same way as the
/// horizon (its right end lower on the screen: a higher index). Past the baked range it clamps.
#[test]
fn the_clouds_roll_with_the_horizon() {
    use crate::clouds::{LEVEL, STEP_DEG};
    let eye = |roll: i32| crate::hud::Eye::new(&camera_at(1024.0, 1100.0, 10.0, 0x3000, 0, roll));
    // (`push_dynamic` reads world up from the walk's `Look::rt`: the same numbers as the HUD's eye.)
    let angle = |e: &crate::hud::Eye| crate::clouds::angle(e.m[1], e.m[4]);
    for roll in [-20, 0, 20] {
        let cam = camera_at(1024.0, 1100.0, 10.0, 0x3000, 0, roll);
        let rt = crate::geom::Look::new(&crate::geom::View { cam, pop: 0 }).rt;
        assert_eq!((rt[0][1], rt[1][1]), (eye(roll).m[1], eye(roll).m[4]));
    }
    assert_eq!(angle(&eye(0)), LEVEL);
    for roll in [-28, -20, -9, -4, 4, 9, 20, 28] {
        let e = eye(roll);
        // The horizon's tilt: two level directions ahead, 20° either side.
        let at = |deg: f64| {
            let a = 0x3000 as f64 / 65536.0 * 360.0 + deg;
            let (s, c) = (a.to_radians().sin(), a.to_radians().cos());
            let p = [e.p[0].wrapping_add((4096.0 * s) as i16), e.p[1], e.p[2].wrapping_add((4096.0 * c) as i16)];
            crate::hud::screen(crate::hud::to_camera(&e, p))
        };
        let ((x0, y0), (x1, y1)) = (at(-20.0), at(20.0));
        // (Hires pixels across are 133/160 of a line.)
        let tilt = ((y1 - y0) as f64).atan2((x1 - x0) as f64 * 133.0 / 160.0).to_degrees();
        let a = angle(&e) as i32;
        let want = LEVEL as i32 + (tilt / STEP_DEG as f64).round() as i32;
        assert!((a - want).abs() <= 1, "roll {roll}: tilt {tilt:.1}°, index {a}, want {want}");
        assert_eq!(a > LEVEL as i32, tilt > 0.0, "roll {roll}: the same way");
    }
    // (Whichever way a roll tilts the horizon, rolling on past 30° stays at that end.)
    let end = |roll: i32| if angle(&eye(roll)) > LEVEL { crate::clouds::ANGLES as u8 - 1 } else { 0 };
    assert_eq!(angle(&eye(60)), end(28));
    assert_eq!(angle(&eye(-60)), end(-28));
}

/// Every cloud frame (each shape at each angle, every phase), drawn back out of the image by the C64's
/// sprite code, is exactly the image baked; the blocks sit in `CLOUDS_BLOCKS..CLOUDS_END`, each inside one
/// 512 KiB region, and the effects end before them.
#[test]
fn cloud_frames_draw_back_exactly() {
    use crate::clouds::{ANGLES, SHAPES, bake, record};
    use crate::format::{CLOUDS_BLOCKS, CLOUDS_END, REU_BANK};
    let mut m = Machine::new(package());
    for s in 0..SHAPES {
        for a in 0..ANGLES {
            let (want, ax, ay) = bake::image(s, a);
            let r = crate::effects_bake::read_record_at(&m.reu, record(s as u8, a as u8));
            assert_eq!((r.ax, r.ay), (ax, ay));
            for (p, &(_, block, len)) in r.phase.iter().enumerate() {
                assert!(block >= CLOUDS_BLOCKS && block + len as u32 <= CLOUDS_END && block / REU_BANK == (block + len as u32 - 1) / REU_BANK, "cloud {s} angle {a} phase {p}");
                let got = crate::effects_bake::draw_back_at(&mut m, record(s as u8, a as u8), p as u8);
                assert_eq!((got.w, got.h), (want.w, want.h));
                assert!(got.pixels == want.pixels, "cloud {s} angle {a} phase {p}");
            }
        }
    }
    let mut reu = vec![0u8; crate::format::IMAGE_END as usize];
    assert!(crate::effects_bake::install(&mut reu).end <= CLOUDS_BLOCKS);
}

/// The clouds are drawn first (behind the hazed mountains) and as near items: sure of their slots, so
/// they never cost a near item its slot; only far (hazed) triangles give way.
#[test]
fn the_clouds_are_drawn_first_and_cost_only_far_triangles() {
    use crate::geom::{HAZED, SPRITE, Work, bin};
    let render = |t: u32, clouds: bool| {
        let reu = package();
        let mut m = Machine::new(reu);
        crate::package::setup(&mut m, crate::layout::BUFFERS[0]);
        let agent = crate::agent::at(reu, t);
        let (cam, _) = agent.camera();
        crate::hud::init(&mut m);
        crate::actors::update(&mut m, &cam);
        crate::combat::fx(&mut m, &agent, &cam);
        if !clouds {
            m.ram[crate::combat::FXDYN as usize] = 0;
        }
        let scene = crate::package::scene_block(reu);
        crate::package::render_floor(&mut m, crate::layout::BUFFERS[0], &scene, &cam, Lines::Full)
    };
    for t in [600u32, 6000, 7800, 9300] {
        let (with, without) = (render(t, true), render(t, false));
        assert!(with.tris > without.tris, "tick {t}: no cloud drawn");
        assert_eq!(with.dropped, without.dropped, "tick {t}: the clouds cost near slots");
    }
    // Painter's order: the clouds' bucket first, then the far triangles.
    let mut w = Work::new();
    crate::geom::push(&mut w, [0, 160, 0], [0, 0, 160], HAZED | crate::geom::HAZE_PAT, crate::geom::FAR_KEY + 3);
    crate::geom::push_sprite(&mut w, 10, 10, 8, 2, 0, 3, crate::clouds::KEY);
    let (order, n, _, _) = bin(&w);
    assert_eq!(n, 2);
    assert_eq!(w.tri(order[0]).shade, SPRITE, "the cloud first");
}

/// Clouds never flicker: every cloud `clouds::place` lists (at most `clouds::MOST`) is pushed into the
/// Work and given a slot in the band pool, over the autopilot's flight and over the whole map from low to
/// high, level and rolled (u 1280 v 1280, 12 high, facing south, rolled 20° lost one as a far item: the
/// near mountains' over-count had the bands it reached). A full Work still takes them
/// (`geom::SPRITE_ROOM`), and so does a full effects list (the clouds are listed first).
#[test]
fn the_clouds_are_never_dropped() {
    use crate::clouds::{KEY, MOST};
    use crate::combat::{FXDYN, FXDYN_ENTRY, FXDYN_MAX};
    use crate::geom::{Bands, MAX_TRIS, SPRITE, TRI_LIMIT, Verts, View, Work};
    let reu = package();
    let scene = crate::package::scene_block(reu);
    let clouds_in = |m: &Machine| {
        let n = m.ram[FXDYN as usize] as u16;
        (0..n).filter(|k| m.ram[(FXDYN + 1 + k * FXDYN_ENTRY + 7) as usize] == KEY).count()
    };
    let mut frames = 0;
    let mut one = |what: String, cam: FloorCam, agent: Option<crate::agent::Agent>| {
        let mut m = Machine::new(reu);
        crate::package::setup(&mut m, crate::layout::BUFFERS[0]);
        crate::hud::init(&mut m);
        crate::actors::update(&mut m, &cam);
        match agent {
            Some(a) => crate::combat::fx(&mut m, &a, &cam),
            None => {
                let n = crate::clouds::place(&mut m, &crate::hud::Eye::new(&cam), 0, 0);
                m.ram[FXDYN as usize] = n;
            }
        }
        let listed = clouds_in(&m);
        assert!(listed <= MOST as usize, "{what}: {listed} clouds listed");
        let mut w = Box::new(Work::new());
        let mut v = Box::new(Verts::new());
        let view = View { cam, pop: 0 };
        crate::geom::draw(&mut m, &scene, &view, &mut w, &mut v);
        crate::geom::walk_args(&mut m, crate::layout::WALK_ARGS, &view);
        crate::geom::push_dynamic(&mut m, &mut w, &mut v);
        let mut i = w.head[KEY as usize];
        let mut pushed = 0;
        while i != crate::geom::NONE {
            pushed += 1;
            i = w.next[i as usize];
        }
        assert_eq!(pushed, listed, "{what}: a cloud left out of the Work");
        let mut b = Bands::new(&w);
        for band in 0..crate::geom::BANDS {
            b.band(&mut m, &w, band, crate::layout::STAGE);
        }
        assert_eq!(b.clouds_dropped, 0, "{what}: a cloud left out of the band pool ({} far items dropped)", b.far_dropped);
        frames += 1;
    };
    // The frame that lost one.
    one("u 1280 v 1280".into(), camera_at(1280.0, 1280.0, 12.0, 0x8000, 0, 20), None);
    for t in (0..20000).step_by(97) {
        let agent = crate::agent::at(reu, t);
        let (cam, _) = agent.camera();
        one(format!("tick {t}"), cam, Some(agent));
    }
    for u in (0..2048).step_by(256) {
        for v in (0..2048).step_by(256) {
            for h in [12.0, 40.0, 120.0] {
                for yaw in (0..8u16).map(|k| k << 13) {
                    for roll in [-20, 0, 20] {
                        one(format!("u {u} v {v} h {h} yaw {yaw:#06x} roll {roll}"), camera_at(u as f64, v as f64, h, yaw, 0, roll), None);
                    }
                }
            }
        }
    }
    // At most `MOST` on screen, over every bearing, roll and a pitch either way.
    let mut most = 0;
    for yaw in (0..256u16).map(|k| k << 8) {
        for roll in [-30, -15, 0, 15, 30] {
            for pitch in [-20, 0, 20] {
                let mut m = Machine::new(reu);
                let n = crate::clouds::place(&mut m, &crate::hud::Eye::new(&camera_at(1024.0, 1100.0, 40.0, yaw, pitch, roll)), 0, 0);
                most = most.max(n);
            }
        }
    }
    assert!(most <= MOST, "{most} clouds at once");
    // A full Work: the triangles stop short, the clouds still go in.
    let mut w = Work::new();
    for k in 0..300 {
        crate::geom::push(&mut w, [0, 160, 0], [0, 0, 160 + (k % 7) as i16], 3, 10);
    }
    assert_eq!(w.n as usize, TRI_LIMIT);
    for _ in 0..MOST {
        crate::geom::push_sprite(&mut w, 10, 10, 8, 2, 0, 3, KEY);
    }
    assert_eq!(w.n as usize, MAX_TRIS);
    assert_eq!((0..w.n).filter(|&i| w.tri(i).shade == SPRITE).count(), MOST as usize);
    // The asm's copy of the limit, and room in the list.
    let asm = include_str!("../target-c64/asm/geom.S");
    assert!(asm.contains(&format!("TRI_LIMIT = {TRI_LIMIT} ")), "asm/geom.S's TRI_LIMIT");
    assert!(FXDYN_MAX > MOST);
    println!("{frames} frames");
}

/// `depth_key`: 2/3-texel buckets to 128 texels, 5.3 to the fog (`FAR_KEY` there), 21 beyond; monotone,
/// and nothing within `FAR_BIG` and its pad reaches the clouds' bucket (255). The asm `facekey` matches
/// it (the emulator's frames).
#[test]
fn depth_keys_keep_their_precision_to_the_fog() {
    use crate::geom::{FAR, FAR_BIG, FAR_KEY, depth_key};
    let q = 16;
    assert_eq!((depth_key(6143), depth_key(6144), depth_key(6144 + 255), depth_key(6144 + 256)), (191, 192, 192, 193));
    assert_eq!((depth_key(3 * FAR - 1), depth_key(3 * FAR)), (FAR_KEY - 1, FAR_KEY));
    assert_eq!((depth_key(3 * FAR + 1023), depth_key(3 * FAR + 1024)), (FAR_KEY, FAR_KEY + 1));
    assert_eq!((depth_key(-5), depth_key(1 << 20)), (0, 255));
    let mut last = 0;
    for s in (-64..3 * (FAR_BIG + 255 * q)).step_by(7) {
        let k = depth_key(s);
        assert!(k >= last, "monotone at {s}");
        last = k;
    }
    assert!(depth_key(3 * (FAR_BIG + 51 * q)) < crate::clouds::KEY);
    // Two objects 6 texels apart at 200 texels sort apart (they shared a bucket at 10.7 texels).
    let at = |t: i32| depth_key(3 * t * q);
    assert!(at(200) < at(206));
}

/// A distant mountain banked over: its ring's end can rise past its peak and be the leftmost (or
/// rightmost) vertex too; the silhouette's top is taken from the others, so its triangle keeps its area.
#[test]
fn banked_silhouettes_keep_their_area() {
    use crate::geom::{Obj, Verts, Work, silhouette, vert};
    let mut degenerate_before = 0;
    for roll in [-30i32, -28, -20, 0, 20, 28, 30] {
        // A wide, low cone (radius 50, height 28) on screen: a ring of 12 and the peak, rolled.
        let th = (roll as f64).to_radians();
        let (c, s) = (th.cos(), th.sin());
        let mut pts: Vec<(f64, f64)> = (0..12).map(|k| {
            let a = k as f64 * std::f64::consts::TAU / 12.0;
            (50.0 * a.cos(), 6.0 * a.sin())
        }).collect();
        pts.push((0.0, -28.0));
        let mut v = Verts::new();
        for (i, &(x, y)) in pts.iter().enumerate() {
            v.set(vert::SX, i, (16.0 * (1000.0 + x * c - y * s)).round() as i16);
            v.set(vert::SY, i, (16.0 * (500.0 + x * s + y * c)).round() as i16);
            v.set(vert::Z, i, 9000);
        }
        let o = Obj { nv: pts.len() as u8, ..Obj::default() };
        // (The old pick: the top among all, which can be the left or right one.)
        let (sx, sy) = (|i: usize| v.get(vert::SX, i), |i: usize| v.get(vert::SY, i));
        let n = pts.len();
        let (l, r) = ((0..n).min_by_key(|&i| (sx(i), i)).unwrap(), (0..n).max_by_key(|&i| (sx(i), usize::MAX - i)).unwrap());
        let t = (0..n).min_by_key(|&i| (sy(i), i)).unwrap();
        degenerate_before += (t == l || t == r) as u32;
        let mut w = Work::new();
        silhouette(&o, &v, &mut w);
        assert_eq!(w.n, 1, "roll {roll}: drawn");
        let tri = w.tri(0);
        let a = (tri.x[1] as i32 - tri.x[0] as i32) * (tri.y[2] as i32 - tri.y[0] as i32) - (tri.y[1] as i32 - tri.y[0] as i32) * (tri.x[2] as i32 - tri.x[0] as i32);
        assert!(a > 16 * 16 * 2000, "roll {roll}: area {a} {tri:?}");
    }
    assert!(degenerate_before > 0, "the case the old pick lost");
}

/// `clouds::place`'s depth cull (`CULL_Z`) never drops a cloud the full projection would place: over yaw,
/// pitch, roll and drift its list equals one projected without it.
#[test]
fn the_clouds_depth_cull_loses_nothing() {
    use crate::clouds::{KEY, LEVEL, REACH_DOWN, REACH_UP, REACH_X, RING, place, record};
    use crate::combat::{FXDYN, FXDYN_ENTRY};
    use crate::hud::{Eye, NEAR, screen, to_camera};
    let mut m = Machine::new(package());
    let (mut seen, mut early) = (0, 0);
    for yaw in (0..16u16).map(|k| k << 12 | 0x123) {
        for pitch in [-20, -10, 0, 10, 20] {
            for roll in [-30, -15, 0, 15, 30] {
                for drift in [0u16, 0x5555, 0xC000] {
                    let eye = Eye::new(&camera_at(1024.0, 1100.0, 10.0, yaw, pitch, roll));
                    let n = place(&mut m, &eye, drift, 0);
                    let got: Vec<[u8; 8]> = (0..n as u16).map(|k| {
                        let at = (FXDYN + 1 + k * FXDYN_ENTRY) as usize;
                        m.ram[at..at + 8].try_into().unwrap()
                    }).collect();
                    let mut want = Vec::new();
                    for c in RING.iter() {
                        let a = ((c[0] as u16) << 8).wrapping_add(drift);
                        let (s, co) = ((crate::camera::sin(a) >> 2) as i16, (crate::camera::cos(a) >> 2) as i16);
                        let p = [eye.p[0].wrapping_add(s), eye.p[1].wrapping_add(c[2] as i16 * 4), eye.p[2].wrapping_add(co)];
                        let cam = to_camera(&eye, p);
                        early += (cam[2] < crate::clouds::CULL_Z) as usize;
                        if cam[2] < NEAR {
                            continue;
                        }
                        let (x, y) = screen(cam);
                        if !(-2 * REACH_X..320 + 2 * REACH_X).contains(&x) || !(-REACH_DOWN..crate::geom::LINES as i16 + REACH_UP).contains(&y) {
                            continue;
                        }
                        let ([x0, x1], [y0, y1], [r0, r1, r2, _]) = ((x >> 1).to_le_bytes(), y.to_le_bytes(), record(c[1], LEVEL).to_le_bytes());
                        want.push([x0, x1, y0, y1, r0, r1, r2, KEY]);
                    }
                    assert_eq!(got, want, "yaw {yaw:#06x} pitch {pitch} roll {roll} drift {drift:#06x}");
                    seen += want.len();
                }
            }
        }
    }
    // (Most of the ring stops at the depth.)
    assert!(seen > 0 && early > 2 * seen, "{seen} placed, {early} stopped early");
}

/// A one-shot's lag follows the player turning once its sweep is done ([`crate::sfx`] `track`), and its
/// frequencies are written only when the slow chip changes: none while it is settled.
#[test]
fn a_settled_one_shot_leaves_its_frequencies_be() {
    use crate::combat::{self, spawn};
    use crate::sfx::{self, LEFT, PAIR, PAIR_STEP, RIGHT, STATE, SWEEP, SWEEP_PAIR, patch, state};
    let mut m = Machine::new(package());
    combat::init(&mut m);
    let mut a = crate::agent::Agent::new();
    (a.u, a.v, a.h, a.t) = (1024 << 8, 1100 << 8, 10 << 8, 1000);
    sfx::update(&mut m, &a);
    // An explosion ahead, level with the player.
    a.t += 4;
    spawn(&mut m, crate::effects::effect::EXPLOSION, 1024 << 4, 1130 << 4, 10 << 4, a.t as u16 - 1);
    sfx::update(&mut m, &a);
    let at = |p: u16| (STATE + state::PAIRS + p * PAIR) as usize;
    let p = (0..sfx::PAIRS).find(|&p| m.ram[at(p)] == patch::EXPLOSION + 1).expect("the explosion's pair");
    // Its sweep done (the IRQ's, which the host hasn't).
    m.ram[(SWEEP + p * SWEEP_PAIR) as usize] = 0;
    // The pair's frequency writes in a frame (`dt` ticks on).
    let freqs = |m: &mut Machine, a: &mut crate::agent::Agent, dt: u32| {
        a.t += dt;
        let mut log = SidLog { m, writes: vec![] };
        sfx::update(&mut log, a);
        let ours = |r: u16| [LEFT, RIGHT].iter().any(|&c| (0..3).any(|v| [0, 1].iter().any(|&k| r == c + p * PAIR_STEP + 7 * v + k)));
        log.writes.iter().filter(|(r, _)| ours(*r)).count()
    };
    assert_eq!(freqs(&mut m, &mut a, 4), 0, "settled ahead: nothing written");
    // Turned to face west: it is abeam on the right, the left ear's chip slowed.
    a.yaw = 0xC000;
    assert!(freqs(&mut m, &mut a, 4) > 0, "the turn slows a chip");
    let mut writes = vec![];
    for _ in 0..12 {
        writes.push(freqs(&mut m, &mut a, 4));
    }
    assert!(writes.ends_with(&[0, 0, 0]), "settled again: nothing written ({writes:?})");
    assert!(m.ram[at(p)] == patch::EXPLOSION + 1, "still playing");
}
