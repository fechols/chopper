//! The cockpit's art (host): every frame of every gauge (`cockpit`) and the static panel, drawn as
//! vector shapes at 16 samples a pixel and reduced to the C64's multicolour cells.
//!
//! A cell shows `$D021` (code 0: green, `format::COLORS[0]`, shared with the view) and three colours
//! of its own (screen RAM's two nibbles, colour RAM). Those are fixed per cell for all of a gauge's
//! frames (only bitmap bytes are DMA'd at run time), so each cell's three are the colours its samples
//! use most over all the frames; a pixel then takes the palette colour most of its samples vote for,
//! "ink" (markings, needles) counting triple so hairlines survive.
//!
//! The dual tach follows the start-up sound: the rotor's speed is the blade beat measured in the
//! decoded audio (`spool`).

use crate::cockpit::{self, ATT_PITCHES, FRAMES, PANEL_ROW, PANEL_ROWS, TALL, WIDE, gauge};
use crate::layout::COLS;
use crate::palette::PALETTE;

pub const BLACK: u8 = 0;
pub const WHITE: u8 = 1;
pub const RED: u8 = 2;
pub const BLUE: u8 = 6;
pub const YELLOW: u8 = 7;
pub const ORANGE: u8 = 8;
pub const BROWN: u8 = 9;
pub const LRED: u8 = 10;
pub const DGREY: u8 = 11;
pub const GREY: u8 = 12;
pub const LBLUE: u8 = 14;
pub const LGREY: u8 = 15;
/// `$D021` (code 0), free in every cell.
pub const GREEN: u8 = crate::format::COLORS[0];

/// Samples a pixel (multicolour pixels are 2 hires pixels wide; the art is drawn in hires pixels).
const SX: usize = 4;
const SY: usize = 4;

fn ink(c: u8) -> u32 {
    match c {
        WHITE | YELLOW | RED | LRED | ORANGE => 3,
        c if c == GREEN => 3,
        _ => 1,
    }
}

fn dist(a: u8, b: u8) -> i32 {
    let (p, q) = (PALETTE[a as usize], PALETTE[b as usize]);
    (0..3).map(|i| (p[i] as i32 - q[i] as i32).pow(2)).sum()
}

// ---- 3×5 font (multicolour pixels), drawn on whole pixels so it stays crisp ----

fn glyph(c: char) -> [u8; 5] {
    match c {
        '0' => [7, 5, 5, 5, 7],
        '1' => [2, 6, 2, 2, 7],
        '2' => [7, 1, 7, 4, 7],
        '3' => [7, 1, 3, 1, 7],
        '4' => [5, 5, 7, 1, 1],
        '5' => [7, 4, 7, 1, 7],
        '6' => [7, 4, 7, 5, 7],
        '7' => [7, 1, 2, 2, 2],
        '8' => [7, 5, 7, 5, 7],
        '9' => [7, 5, 7, 1, 7],
        'A' => [2, 5, 7, 5, 5],
        'C' => [7, 4, 4, 4, 7],
        'D' => [6, 5, 5, 5, 6],
        'E' => [7, 4, 6, 4, 7],
        'G' => [7, 4, 5, 5, 7],
        'H' => [5, 5, 7, 5, 5],
        'I' => [7, 2, 2, 2, 7],
        'L' => [4, 4, 4, 4, 7],
        'M' => [5, 7, 7, 5, 5],
        'N' => [6, 5, 5, 5, 5],
        'P' => [7, 5, 7, 4, 4],
        'R' => [6, 5, 6, 5, 5],
        'S' => [7, 4, 7, 1, 7],
        'T' => [7, 2, 2, 2, 2],
        'U' => [5, 5, 5, 5, 7],
        'V' => [5, 5, 5, 5, 2],
        'W' => [5, 5, 7, 7, 5],
        _ => [0; 5],
    }
}

/// Does `text`, centred on (`cx`, `cy`) (hires pixels), cover the multicolour pixel (`mx`, `my`)?
fn text(text: &str, cx: f32, cy: f32, mx: i32, my: i32) -> bool {
    let n = text.chars().count() as i32;
    let w = 4 * n - 1;
    let left = (cx / 2.0 - w as f32 / 2.0).round() as i32;
    let top = (cy - 2.5).round() as i32;
    let (x, y) = (mx - left, my - top);
    if x < 0 || x >= w || !(0..5).contains(&y) || x % 4 == 3 {
        return false;
    }
    let g = glyph(text.chars().nth((x / 4) as usize).unwrap());
    g[y as usize] >> (2 - x % 4) & 1 != 0
}

// ---- geometry ----

/// Polar coordinates about (`cx`, `cy`): radius and bearing in degrees (0 up, clockwise).
fn polar(x: f32, y: f32, cx: f32, cy: f32) -> (f32, f32) {
    let (dx, dy) = (x - cx, y - cy);
    ((dx * dx + dy * dy).sqrt(), dx.atan2(-dy).to_degrees())
}

fn wrap(a: f32) -> f32 {
    (a + 540.0).rem_euclid(360.0) - 180.0
}

/// A radial tick at bearing `at` from radius `r0` to `r1`, `w` pixels wide.
fn tick(r: f32, a: f32, at: f32, r0: f32, r1: f32, w: f32) -> bool {
    (r0..=r1).contains(&r) && wrap(a - at).abs().to_radians() * r <= w / 2.0
}

/// An arc band between bearings `a0..a1` (clockwise), radii `r0..r1`.
fn arc(r: f32, a: f32, a0: f32, a1: f32, r0: f32, r1: f32) -> bool {
    (r0..=r1).contains(&r) && wrap(a - a0) >= 0.0 && wrap(a - a0) <= a1 - a0
}

/// A tapered needle from the centre toward bearing `at`, `len` long with a `tail`, `w` wide at its root.
fn needle(dx: f32, dy: f32, at: f32, len: f32, tail: f32, w: f32) -> bool {
    let (s, c) = at.to_radians().sin_cos();
    let along = dx * s - dy * c;
    let across = (dx * c + dy * s).abs();
    (-tail..=len).contains(&along) && across <= w / 2.0 * (1.0 - 0.7 * along.max(0.0) / len)
}

/// A dial's surround: the panel, corner screws, the black face and its rim. `None` inside the face.
fn surround(x: f32, y: f32, w: f32) -> Option<u8> {
    let (r, _) = polar(x, y, w / 2.0, 24.0);
    if r < FACE {
        return None;
    }
    if r < RIM {
        return Some(BLACK);
    }
    for (sx, sy) in [(3.5, 3.5), (w - 3.5, 3.5), (3.5, 44.5), (w - 3.5, 44.5)] {
        let d = ((x - sx).powi(2) + (y - sy).powi(2)).sqrt();
        if d < 1.6 {
            return Some(if (x - sx) + (y - sy) < -0.5 { LGREY } else { GREY });
        }
    }
    Some(DGREY)
}

const FACE: f32 = 20.0;
const RIM: f32 = 23.0;

// ---- the gauges ----

/// Airspeed: 0-160 kt over 330°, green 40-100, yellow 100-130, red line 130.
fn asi(k: u16, x: f32, y: f32, mx: i32, my: i32) -> u8 {
    let (cx, cy) = (24.0, 24.0);
    let (r, a) = polar(x, y, cx, cy);
    let deg = |kt: f32| kt * 330.0 / 160.0;
    if needle(x - cx, y - cy, deg(k as f32 * 2.5), 18.0, 4.0, 2.4) {
        return WHITE;
    }
    if r < 2.0 {
        return GREY;
    }
    if tick(r, a, deg(130.0), 15.0, 19.5, 1.4) {
        return RED;
    }
    for kt in (0..=160).step_by(10) {
        let major = kt % 20 == 0;
        if tick(r, a, deg(kt as f32), if major { 15.0 } else { 17.0 }, 19.5, 1.1) {
            return WHITE;
        }
    }
    if arc(r, a, deg(40.0), deg(100.0), 17.5, 19.5) {
        return GREEN;
    }
    if arc(r, a, deg(100.0), deg(130.0), 17.5, 19.5) {
        return YELLOW;
    }
    for (kt, s) in [(40.0, "4"), (80.0, "8"), (120.0, "12")] {
        let (sn, cs) = deg(kt).to_radians().sin_cos();
        if text(s, cx + 11.0 * sn, cy - 11.0 * cs, mx, my) {
            return WHITE;
        }
    }
    BLACK
}

/// The horizon's tilt (degrees, clockwise on screen) and its offset (pixels, down) for attitude frame
/// `k`: banking right turns the horizon counter-clockwise; nose down raises it.
pub fn attitude(k: u16) -> (f32, f32) {
    let roll = (k / ATT_PITCHES) as f32 - 12.0;
    let pitch = (k % ATT_PITCHES) as f32 - 8.0;
    (-roll * 2.8125, pitch * 1.40625 * 0.6)
}

/// Attitude: the sky/ground ball with its pitch ladder, the bank scale and pointer, the fixed aircraft.
fn att(k: u16, x: f32, y: f32, _mx: i32, _my: i32) -> u8 {
    let (cx, cy) = (24.0, 24.0);
    let (dx, dy) = (x - cx, y - cy);
    let (r, a) = polar(x, y, cx, cy);
    let (tilt, off) = attitude(k);
    // The fixed aircraft: wings, centre dot.
    if (dy + 0.0).abs() <= 0.7 && (4.5..=12.0).contains(&dx.abs()) || r < 1.3 {
        return YELLOW;
    }
    if (0.0..=2.0).contains(&dy) && (4.5..=5.7).contains(&dx.abs()) {
        return YELLOW;
    }
    let (s, c) = tilt.to_radians().sin_cos();
    // Along the horizon, and across it (down toward the ground).
    let u = dx * c + dy * s;
    let w = -dx * s + dy * c - off;
    // The bank scale, fixed on the rim, over the ball's edge.
    if r >= 17.0 {
        for b in [-60.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 60.0f32] {
            if tick(r, a, b, 17.0, 19.5, if b.abs() == 30.0 || b == 0.0 { 1.5 } else { 1.0 }) {
                return WHITE;
            }
        }
        return BLACK;
    }
    // The bank pointer, on the ball, pointing out at the scale.
    if (14.5..17.0).contains(&r) && wrap(a - tilt).abs().to_radians() * r <= 1.9 * (17.0 - r) / 2.5 {
        return WHITE;
    }
    if w.abs() < 0.5 {
        return WHITE;
    }
    for (p, half) in [(-9.0, 5.0), (-4.5, 3.0), (4.5, 3.0), (9.0, 5.0)] {
        if (w - p).abs() < 0.45 && u.abs() <= half {
            return WHITE;
        }
    }
    if w < 0.0 { LBLUE } else { BROWN }
}

/// Altimeter: hundreds (long needle, 1000 ft a turn) and thousands (short, wide), 0-9 around.
fn alt(k: u16, x: f32, y: f32, mx: i32, my: i32) -> u8 {
    let (cx, cy) = (24.0, 24.0);
    let (r, a) = polar(x, y, cx, cy);
    let hundreds = (k & 63) as f32 / 64.0;
    let thousands = (k >> 6) as f32 + hundreds;
    if needle(x - cx, y - cy, hundreds * 360.0, 18.0, 4.0, 2.2) {
        return WHITE;
    }
    if needle(x - cx, y - cy, thousands * 36.0, 10.5, 2.0, 3.6) {
        return WHITE;
    }
    if r < 2.0 {
        return GREY;
    }
    for i in 0..50 {
        let major = i % 5 == 0;
        if tick(r, a, i as f32 * 7.2, if major { 17.0 } else { 18.3 }, 19.5, if major { 1.3 } else { 0.8 }) {
            return if major { WHITE } else { GREY };
        }
    }
    for d in 0..10 {
        let (s, c) = (d as f32 * 36.0).to_radians().sin_cos();
        if text(&d.to_string(), cx + 13.0 * s, cy - 13.0 * c, mx, my) {
            return WHITE;
        }
    }
    BLACK
}

/// The caution lamps: RPM (red), LOW ALT (amber), AP (green), dark when off.
fn warn(k: u16, x: f32, y: f32, mx: i32, my: i32) -> u8 {
    let lamps = [(cockpit::lamp::RPM, "RPM", RED), (cockpit::lamp::LOW_ALT, "ALT", YELLOW), (cockpit::lamp::AP, "AP", GREEN)];
    for (i, (bit, name, lit)) in lamps.into_iter().enumerate() {
        let top = 16.0 * i as f32;
        if !(top..top + 16.0).contains(&y) {
            continue;
        }
        let on = k as u8 & bit != 0;
        let (lx0, lx1, ly0, ly1) = (3.0, 29.0, top + 3.0, top + 13.0);
        if !(lx0..lx1).contains(&x) || !(ly0..ly1).contains(&y) {
            return DGREY;
        }
        if text(name, 16.0, top + 8.0, mx, my) {
            return if on { BLACK } else { DGREY };
        }
        return if on { lit } else { BLACK };
    }
    DGREY
}

/// The rotor's speed (percent) through the start-up, one per tach frame: the blade beat measured in
/// the decoded sound (`blade_beat`) as a fraction of the beat at the loop (the governed 100%), from
/// `BEAT_FROM`; before that the rotor is too slow to beat clearly while the turbine winds up, so it eases
/// up from rest at `LIGHT_OFF` to the first measurement.
pub fn spool() -> Vec<f32> {
    let pcm = crate::digital::decode(crate::digital::ROTOR);
    let rate = 16_000.0;
    let lift = crate::agent::LIFT as f32 / 60.0;
    let beats: Vec<(f32, f32)> = (BEAT_FROM as u32..=lift as u32).map(|t| (t as f32, blade_beat(&pcm, t as f32 * rate))).collect();
    let full = beats.last().unwrap().1;
    // Monotone (the rotor only speeds up), to 100% at the lift.
    let mut pct = Vec::new();
    let mut top = 0.0f32;
    for &(t, b) in &beats {
        top = top.max(b / full * 100.0).min(100.0);
        pct.push((t, top));
    }
    let first = pct[0].1;
    (0..cockpit::TACH_FRAMES)
        .map(|k| {
            let t = ((k as u32) << cockpit::TACH_SHIFT) as f32 / 60.0;
            if k + 1 == cockpit::TACH_FRAMES {
                100.0
            } else if t < LIGHT_OFF {
                0.0
            } else if t < BEAT_FROM {
                first * ((t - LIGHT_OFF) / (BEAT_FROM - LIGHT_OFF)).powf(1.6)
            } else {
                let i = pct.iter().rposition(|&(pt, _)| pt <= t).unwrap();
                let (t0, p0) = pct[i];
                let (t1, p1) = pct[(i + 1).min(pct.len() - 1)];
                if t1 > t0 { p0 + (p1 - p0) * (t - t0) / (t1 - t0) } else { p0 }
            }
        })
        .collect()
}

/// Seconds into the sound: the engine lights off (its whine starts), and the blade beat is clear.
pub const LIGHT_OFF: f32 = 4.0;
pub const BEAT_FROM: f32 = 40.0;

/// The blade-passing rate (Hz) around sample `at` of 16 kHz PCM: the strongest period, 2-16 Hz, of the
/// loudness envelope (1 kHz) over 4 s.
pub fn blade_beat(pcm: &[i8], at: f32) -> f32 {
    let env: Vec<f32> = (0..4000)
        .map(|i| {
            let s = at as i64 - 32_000 + i * 16;
            (0..16).map(|j| pcm.get((s + j).max(0) as usize).map_or(0.0, |&v| (v as f32).abs())).sum::<f32>() / 16.0
        })
        .collect();
    let mean = env.iter().sum::<f32>() / env.len() as f32;
    let e: Vec<f32> = env.iter().map(|v| v - mean).collect();
    let (lag, _) = (62..500)
        .map(|lag| (lag, e.iter().zip(&e[lag..]).map(|(a, b)| a * b).sum::<f32>()))
        .fold((0, f32::MIN), |m, (l, v)| if v > m.1 { (l, v) } else { m });
    1000.0 / lag as f32
}

/// Dual tach: rotor (long) and engine (short) needles, married through the start, 0-110% over 300°;
/// green 95-105, red line 107.
fn rpm(k: u16, x: f32, y: f32, mx: i32, my: i32, spool: &[f32]) -> u8 {
    let (cx, cy) = (24.0, 24.0);
    let (r, a) = polar(x, y, cx, cy);
    let deg = |p: f32| -150.0 + p * 300.0 / 110.0;
    let p = spool[k as usize];
    if needle(x - cx, y - cy, deg(p), 18.0, 3.0, 2.2) || needle(x - cx, y - cy, deg(p), 12.5, 0.0, 3.4) {
        return WHITE;
    }
    if r < 2.0 {
        return GREY;
    }
    if tick(r, a, deg(107.0), 15.0, 19.5, 1.4) {
        return RED;
    }
    for p in (0..=110).step_by(10) {
        if tick(r, a, deg(p as f32), if p % 50 == 0 { 15.0 } else { 17.0 }, 19.5, 1.1) {
            return WHITE;
        }
    }
    if arc(r, a, deg(95.0), deg(105.0), 16.5, 19.5) {
        return GREEN;
    }
    for (p, s) in [(0.0, "0"), (50.0, "5"), (100.0, "10")] {
        let (sn, cs) = deg(p).to_radians().sin_cos();
        if text(s, cx + 11.0 * sn, cy - 11.0 * cs, mx, my) {
            return WHITE;
        }
    }
    BLACK
}

/// Heading: the compass card turns under the fixed lubber line and aircraft.
fn hdg(k: u16, x: f32, y: f32, mx: i32, my: i32) -> u8 {
    let (cx, cy) = (24.0, 24.0);
    let (dx, dy) = (x - cx, y - cy);
    let (r, a) = polar(x, y, cx, cy);
    let heading = k as f32 * 360.0 / 128.0;
    // The aircraft: fuselage, wings, tail.
    if dx.abs() <= 0.8 && (-6.0..=5.0).contains(&dy) || (dy + 1.0).abs() <= 0.8 && dx.abs() <= 5.5 || (dy - 4.5).abs() <= 0.6 && dx.abs() <= 2.2 {
        return YELLOW;
    }
    // The lubber line.
    if r >= 17.5 && r <= 20.0 && wrap(a).abs() * r.to_radians() <= 1.6 * (20.0 - r) / 2.5 + 0.3 {
        return YELLOW;
    }
    for b in (0..360).step_by(10) {
        let major = b % 30 == 0;
        if tick(r, a, b as f32 - heading, if major { 15.5 } else { 17.5 }, 19.5, if major { 1.2 } else { 0.9 }) {
            return WHITE;
        }
    }
    for (b, s) in [(0.0, "N"), (90.0, "E"), (180.0, "S"), (270.0, "W")] {
        let (sn, cs) = (b - heading).to_radians().sin_cos();
        if text(s, cx + 12.0 * sn, cy - 12.0 * cs, mx, my) {
            return WHITE;
        }
    }
    BLACK
}

/// Vertical speed: 0 at nine o'clock, climb clockwise (up), ±20 steps over ±165°.
fn vsi(k: u16, x: f32, y: f32, mx: i32, my: i32) -> u8 {
    let (cx, cy) = (24.0, 24.0);
    let (r, a) = polar(x, y, cx, cy);
    let deg = |v: f32| -90.0 + v * 8.25;
    if needle(x - cx, y - cy, deg(k as f32 - 20.0), 18.0, 4.0, 2.4) {
        return WHITE;
    }
    if r < 2.0 {
        return GREY;
    }
    for v in (-20..=20).step_by(2) {
        let major = v % 10 == 0;
        if tick(r, a, deg(v as f32), if major { 15.0 } else { 17.5 }, 19.5, 1.0) {
            return WHITE;
        }
    }
    for (v, s) in [(0.0, "0"), (10.0, "1"), (-10.0, "1"), (20.0, "2"), (-20.0, "2")] {
        let (sn, cs) = deg(v).to_radians().sin_cos();
        if text(s, cx + 11.5 * sn, cy - 11.5 * cs, mx, my) {
            return WHITE;
        }
    }
    BLACK
}

/// Gauge `g`'s frame `k` at hires (`x`, `y`) in its box, multicolour pixel (`mx`, `my`).
fn gauge_art(g: usize, k: u16, x: f32, y: f32, mx: i32, my: i32, spool: &[f32]) -> u8 {
    let w = WIDE[g] as f32 * 8.0;
    if g != gauge::WARN {
        if let Some(c) = surround(x, y, w) {
            return c;
        }
    }
    match g {
        gauge::ASI => asi(k, x, y, mx, my),
        gauge::ATT => att(k, x, y, mx, my),
        gauge::ALT => alt(k, x, y, mx, my),
        gauge::WARN => warn(k, x, y, mx, my),
        gauge::RPM => rpm(k, x, y, mx, my, spool),
        gauge::HDG => hdg(k, x, y, mx, my),
        _ => vsi(k, x, y, mx, my),
    }
}

/// The static panel at hires (`x`, `y`) of the cockpit rows (`mx`, `my` its multicolour pixel).
fn panel_art(x: f32, y: f32, mx: i32, my: i32) -> u8 {
    // The glareshield's lip.
    if y < 5.0 {
        return BLACK;
    }
    if y < 6.0 {
        return GREY;
    }
    // The placards under the dials.
    if y >= 56.0 {
        for g in 0..gauge::COUNT {
            let cx = (cockpit::COL[g] as f32 + WIDE[g] as f32 / 2.0) * 8.0;
            if text(gauge::NAMES[g], cx, 59.5, mx, my) {
                return LGREY;
            }
        }
    }
    let _ = x;
    DGREY
}

/// One image's colour samples: `w × h` multicolour pixels, `SX × SY` each.
fn samples(w: usize, h: usize, art: impl Fn(f32, f32, i32, i32) -> u8) -> Vec<u8> {
    let mut out = Vec::with_capacity(w * h * SX * SY);
    for my in 0..h {
        for mx in 0..w {
            for j in 0..SY {
                for i in 0..SX {
                    let x = 2.0 * mx as f32 + (i as f32 + 0.5) * 2.0 / SX as f32;
                    let y = my as f32 + (j as f32 + 0.5) / SY as f32;
                    out.push(art(x, y, mx as i32, my as i32));
                }
            }
        }
    }
    out
}

/// Each cell's three colours (codes 1, 2, 3) from the samples of all of an image's frames.
fn palettes(w: usize, h: usize, frames: &[Vec<u8>]) -> Vec<[u8; 3]> {
    let (cw, ch) = (w / 4, h / 8);
    let mut out = Vec::with_capacity(cw * ch);
    for cy in 0..ch {
        for cx in 0..cw {
            let mut count = [0u32; 16];
            for f in frames {
                for my in cy * 8..cy * 8 + 8 {
                    for mx in cx * 4..cx * 4 + 4 {
                        for &c in &f[(my * w + mx) * SX * SY..][..SX * SY] {
                            count[c as usize] += ink(c);
                        }
                    }
                }
            }
            count[GREEN as usize] = 0;
            let mut order: Vec<u8> = (0..16).collect();
            order.sort_by_key(|&c| (std::cmp::Reverse(count[c as usize]), c));
            let mut p = [order[0], order[1], order[2]];
            for k in 1..3 {
                if count[p[k] as usize] == 0 {
                    p[k] = p[0];
                }
            }
            out.push(p);
        }
    }
    out
}

/// An image's bitmap bytes in bitmap order (cell rows, then cells, then lines), each pixel the palette
/// colour its samples vote for.
fn quantise(w: usize, h: usize, s: &[u8], pal: &[[u8; 3]]) -> Vec<u8> {
    let cw = w / 4;
    let mut out = vec![0u8; w * h / 4];
    for my in 0..h {
        for mx in 0..w {
            let p = pal[(my / 8) * cw + mx / 4];
            let choice = [GREEN, p[0], p[1], p[2]];
            let mut vote = [0u32; 4];
            for &c in &s[(my * w + mx) * SX * SY..][..SX * SY] {
                // Only green itself takes code 0 (a colour missing from the cell goes to its nearest own colour).
                let code = if c == GREEN { 0 } else { (1..4).min_by_key(|&k| dist(c, choice[k])).unwrap() };
                vote[code] += ink(c);
            }
            let code = (0..4).max_by_key(|&k| (vote[k], std::cmp::Reverse(k))).unwrap() as u8;
            let byte = (my / 8) * cw * 8 + (mx / 4) * 8 + my % 8;
            out[byte] |= code << (6 - 2 * (mx % 4));
        }
    }
    out
}

/// Everything the cockpit needs in the REU.
pub struct Baked {
    /// The static panel: bitmap (`PANEL_BITMAP`), screen RAM, colour RAM (`PANEL_CELLS` each).
    pub panel: Vec<u8>,
    /// Per gauge, per frame: `TALL` cell rows of `WIDE × 8` bytes.
    pub frames: Vec<Vec<Vec<u8>>>,
}

pub fn bake() -> Baked {
    let spool = spool();
    let h = TALL as usize * 8;
    let mut frames = Vec::new();
    let mut pals = Vec::new();
    for g in 0..gauge::COUNT {
        let w = WIDE[g] as usize * 4;
        let s: Vec<Vec<u8>> = (0..FRAMES[g]).map(|k| samples(w, h, |x, y, mx, my| gauge_art(g, k, x, y, mx, my, &spool))).collect();
        let pal = palettes(w, h, &s);
        frames.push(s.iter().map(|f| quantise(w, h, f, &pal)).collect::<Vec<_>>());
        pals.push(pal);
    }
    // The panel, its gauge cells taking the gauges' palettes and first frames.
    let (pw, ph) = (COLS as usize * 4, PANEL_ROWS as usize * 8);
    let s = samples(pw, ph, panel_art);
    let mut pal = palettes(pw, ph, std::slice::from_ref(&s));
    let mut bitmap = quantise(pw, ph, &s, &pal);
    let row0 = (cockpit::ROW - PANEL_ROW) as usize;
    for g in 0..gauge::COUNT {
        let wb = WIDE[g] as usize;
        for r in 0..TALL as usize {
            for c in 0..wb {
                let cell = (row0 + r) * COLS as usize + cockpit::COL[g] as usize + c;
                pal[cell] = pals[g][r * wb + c];
                bitmap[cell * 8..cell * 8 + 8].copy_from_slice(&frames[g][0][(r * wb + c) * 8..][..8]);
            }
        }
    }
    let mut panel = bitmap;
    panel.extend(pal.iter().map(|p| p[0] << 4 | p[1]));
    panel.extend(pal.iter().map(|p| p[2]));
    Baked { panel, frames }
}

/// Write the cockpit into an image (`cockpit::PANEL`, `cockpit::BASE`).
pub fn install(reu: &mut [u8], b: &Baked) {
    reu[cockpit::PANEL as usize..][..b.panel.len()].copy_from_slice(&b.panel);
    for (g, frames) in b.frames.iter().enumerate() {
        for (k, f) in frames.iter().enumerate() {
            let at = cockpit::BASE[g] as usize + (k << cockpit::FRAME_STRIDE_SHIFT);
            reu[at..at + f.len()].copy_from_slice(f);
        }
    }
}
