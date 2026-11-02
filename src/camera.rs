//! Host-side camera setup: views → per-line transforms ([`Line`]), plus a float reference render. On the
//! C64 these lines come from tables; here they are computed in `f32` and quantized exactly as the C64
//! would store them.

use crate::format::{ANGLES, ZOOMS, pre_bank};
use crate::layout::CACHE_SIZE;
use crate::mode7::Line;
use std::f32::consts::TAU;

#[derive(Clone, Copy, Debug)]
pub enum View {
    /// Top-down rotozoom centred on `(x, y)`; `zoom` = texels per hires pixel.
    Roto { x: f32, y: f32, angle: f32, zoom: f32 },
    /// Perspective floor: camera at `(x, y)`, `height` above the plane, looking along `angle`.
    Floor { x: f32, y: f32, angle: f32, height: f32, horizon: u16, focal: f32 },
}

/// A view's lines and the cache window the CPU path uses.
pub struct Frame {
    pub lines: [Line; 200],
    /// Plane position of each line's pixel 0 and step per multicolor pixel, unquantized (None: sky).
    pub exact: [Option<([f32; 2], [f32; 2])>; 200],
    pub window: (u8, u8),
    /// First cell row below the horizon (0 for a rotozoom).
    pub horizon_row: u16,
}

fn fixed(t: f32) -> u16 {
    ((t * 256.0).round() as i64).rem_euclid(65536) as u16
}

fn step(s: f32) -> i16 {
    (s * 256.0).round().clamp(-32768.0, 32767.0) as i16
}

/// The pre-sampled variant nearest to a line's step per multicolor pixel.
pub fn variant(du: f32, dv: f32) -> u8 {
    let a = ((dv.atan2(du) / TAU * ANGLES as f32).round() as i32).rem_euclid(ANGLES as i32) as u8;
    let s = (du * du + dv * dv).sqrt().max(1e-6);
    let z = ((s / 0.25).log2().round() as i32).clamp(0, ZOOMS as i32 - 1) as u8;
    pre_bank(a, z)
}

pub fn frame(view: &View) -> Frame {
    let mut exact = [None; 200];
    let mut horizon_row = 0;
    match *view {
        View::Roto { x, y, angle, zoom } => {
            let (s, c) = angle.sin_cos();
            let (right, down) = ([c, s], [-s, c]);
            for (l, e) in exact.iter_mut().enumerate() {
                let dy = (l as f32 + 0.5 - 100.0) * zoom;
                let st = [2.0 * zoom * right[0], 2.0 * zoom * right[1]];
                let p = [x + dy * down[0] - 80.0 * st[0], y + dy * down[1] - 80.0 * st[1]];
                *e = Some((p, st));
            }
        }
        View::Floor { x, y, angle, height, horizon, focal } => {
            assert!(horizon % 8 == 0, "the horizon sits on a cell row (colour RAM)");
            horizon_row = horizon / 8;
            let (s, c) = angle.sin_cos();
            let (right, fwd) = ([c, s], [s, -c]);
            for (l, e) in exact.iter_mut().enumerate().skip(horizon as usize) {
                let dist = height * focal / (l as f32 - horizon as f32 + 0.5);
                let z = dist / focal; // texels per hires pixel
                let st = [2.0 * z * right[0], 2.0 * z * right[1]];
                let p = [x + fwd[0] * dist - 80.0 * st[0], y + fwd[1] * dist - 80.0 * st[1]];
                *e = Some((p, st));
            }
        }
    }
    // Cache window: centred on the mean line centre of the nearer half of the plane lines.
    let plane: Vec<_> = exact.iter().flatten().collect();
    let near = &plane[plane.len() / 2..];
    let (mut cu, mut cv) = (0.0, 0.0);
    for (p, st) in near {
        cu += p[0] + 80.0 * st[0];
        cv += p[1] + 80.0 * st[1];
    }
    let n = near.len().max(1) as f32;
    let half = CACHE_SIZE as f32 / 2.0;
    let window = (((cu / n - half).round() as i32).rem_euclid(256) as u8, ((cv / n - half).round() as i32).rem_euclid(256) as u8);

    let mut lines = [Line::SKY; 200];
    for (ln, e) in lines.iter_mut().zip(&exact) {
        let Some((p, st)) = e else { continue };
        let mut l = Line { u0: fixed(p[0]), v0: fixed(p[1]), du: step(st[0]), dv: step(st[1]), ..Line::default() };
        l.bank = variant(st[0], st[1]);
        (l.clip0, l.clip1) = clip(&l, window);
        *ln = l;
    }
    Frame { lines, exact, window, horizon_row }
}

/// The bytes of a line whose four pixels all fall inside the cache window, as the 8.8 DDA steps them.
fn clip(l: &Line, (wu, wv): (u8, u8)) -> (u8, u8) {
    let inside = |c: u16| {
        (0..4).all(|k| {
            let x = (4 * c + k) as u16;
            let u = (l.u0.wrapping_add((l.du as u16).wrapping_mul(x)) >> 8) as u8;
            let v = (l.v0.wrapping_add((l.dv as u16).wrapping_mul(x)) >> 8) as u8;
            u.wrapping_sub(wu) < 128 && v.wrapping_sub(wv) < 128
        })
    };
    let Some(first) = (0..40).find(|&c| inside(c)) else { return (0, 0) };
    let end = (first..40).find(|&c| !inside(c)).unwrap_or(40);
    (first as u8, end as u8)
}

/// Exact render: every pixel samples the plane at its centre (`codes` is the 256×256 code map).
/// Returns 160×200 codes, the sky as 3.
pub fn reference(f: &Frame, codes: &[u8]) -> Vec<u8> {
    let mut out = vec![3u8; 160 * 200];
    for (l, e) in f.exact.iter().enumerate() {
        let Some((p, st)) = e else { continue };
        for x in 0..160 {
            let t = x as f32 + 0.5;
            let u = ((p[0] + t * st[0]).floor() as i32).rem_euclid(256) as usize;
            let v = ((p[1] + t * st[1]).floor() as i32).rem_euclid(256) as usize;
            out[l * 160 + x] = codes[v * 256 + u];
        }
    }
    out
}
