//! The ground below the horizon: the part of the viewport whose view
//! direction points downward, as one screen-space polygon. It is painted
//! first (behind all terrain), so the sea fills everything beyond the
//! terrain's view distance at any pitch and roll.

use crate::camera::Camera;
use crate::projection::{CENTER_X, CENTER_Y, SCALE_X, SCALE_Y};
use crate::render::{Renderer, VIEW_HEIGHT, WIDTH};

/// Clip the viewport rectangle to the half-plane below the horizon. Returns
/// the clockwise polygon (up to 5 points) and its length.
pub fn ground_polygon(camera: &Camera) -> ([(i16, i16); 6], usize) {
    // World up in camera space.
    let up = camera.rotate([0, 1 << 14, 0]);
    // A screen point's view direction is ((sx − cx)/SX, −(sy − cy)/SY, 1).
    // f > 0 above the horizon; linear in (sx, sy).
    let f = |sx: i32, sy: i32| -> i64 {
        let dx = (sx - CENTER_X as i32) as i64 * SCALE_Y as i64;
        let dy = -(sy - CENTER_Y as i32) as i64 * SCALE_X as i64;
        let dz = (SCALE_X * SCALE_Y) as i64;
        dx * up.x as i64 + dy * up.y as i64 + dz * up.z as i64
    };
    let (w, h) = (WIDTH as i32, VIEW_HEIGHT as i32);
    // Clockwise on screen (y down): TL, TR, BR, BL.
    let rect = [(0, 0), (w, 0), (w, h), (0, h)];
    let mut out = [(0i16, 0i16); 6];
    let mut n = 0;
    for k in 0..4 {
        let (p, q) = (rect[k], rect[(k + 1) % 4]);
        let (fp, fq) = (f(p.0, p.1), f(q.0, q.1));
        if fp <= 0 {
            out[n] = (p.0 as i16, p.1 as i16);
            n += 1;
        }
        if (fp <= 0) != (fq <= 0) {
            // Shift the f values down so the products fit i64.
            let (a, b) = (fp >> 16, fq >> 16);
            let den = a - b;
            let lerp = |u: i32, v: i32| -> i16 {
                if den == 0 { u as i16 } else { (u as i64 + ((v - u) as i64 * a * 2 + den) / (den * 2)) as i16 }
            };
            out[n] = (lerp(p.0, q.0), lerp(p.1, q.1));
            n += 1;
        }
    }
    (out, n)
}

pub fn draw_ground(renderer: &mut Renderer, camera: &Camera, color: u8) {
    let (points, n) = ground_polygon(camera);
    renderer.backdrop(&points[..n], color);
}
