//! A test plane (host): a 256×256 four-code racetrack, and the REU image built from it.
//!
//! Codes: 0 grass (`$D021`), 1 tarmac, 2 white paint, 3 red kerbs and flower beds.

use crate::format::{ANGLES, MAP_BANK, MAP_SIZE, REU_SIZE, SKY_BANK, ZOOM_STEP, ZOOMS, pre_bank};
use std::f32::consts::TAU;

const N: usize = MAP_SIZE;

/// The circuit's centreline at parameter `t` (0..1).
pub fn centre(t: f32) -> [f32; 2] {
    let a = t * TAU;
    [128.0 + 92.0 * a.cos() + 18.0 * (3.0 * a).cos(), 128.0 + 80.0 * a.sin() + 22.0 * (2.0 * a).sin()]
}

pub fn generate() -> Vec<u8> {
    const SAMPLES: usize = 720;
    let pts: Vec<[f32; 2]> = (0..SAMPLES).map(|i| centre(i as f32 / SAMPLES as f32)).collect();
    let mut codes = vec![0u8; N * N];
    for v in 0..N {
        for u in 0..N {
            let p = [u as f32 + 0.5, v as f32 + 0.5];
            let (mut best, mut at) = (f32::MAX, 0);
            for (i, q) in pts.iter().enumerate() {
                let d = (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
                if d < best {
                    best = d;
                    at = i;
                }
            }
            let d = best.sqrt();
            let code = if d < 0.8 {
                if (at / 6) % 2 == 0 { 2 } else { 1 } // centre dashes
            } else if d < 8.0 {
                // start line: a chequered band across the track at t = 0
                if at < 4 || at >= SAMPLES - 4 { if (u + v) % 2 == 0 { 2 } else { 1 } } else { 1 }
            } else if d < 9.0 {
                2 // edge line
            } else if d < 11.0 {
                if (at / 5) % 2 == 0 { 3 } else { 2 } // kerbs
            } else {
                // flower beds scattered over the grass
                let h = (u as u32).wrapping_mul(73_856_093) ^ (v as u32).wrapping_mul(19_349_663);
                if h % 97 == 0 { 3 } else { 0 }
            };
            codes[v * N + u] = code;
        }
    }
    codes
}

/// The REU image: the raw plane, the 240 pre-sampled planes and the sky bank.
pub fn bake(codes: &[u8]) -> Vec<u8> {
    assert_eq!(codes.len(), N * N);
    let mut reu = vec![0u8; REU_SIZE as usize];
    let at = |bank: u8, i: usize| ((bank as usize) << 16) + i;
    for (i, &c) in codes.iter().enumerate() {
        reu[at(MAP_BANK, i)] = c * 0x55;
    }
    let code = |u: f32, v: f32| codes[(v.floor() as i32).rem_euclid(256) as usize * N + (u.floor() as i32).rem_euclid(256) as usize];
    for a in 0..ANGLES {
        let (s, c) = (a as f32 / ANGLES as f32 * TAU).sin_cos();
        for z in 0..ZOOMS {
            let step = ZOOM_STEP[z as usize] as f32 / 256.0;
            let base = at(pre_bank(a, z), 0);
            for v in 0..N {
                for u in 0..N {
                    let mut b = 0u8;
                    for k in 0..4 {
                        let t = k as f32 * step;
                        b |= code(u as f32 + t * c, v as f32 + t * s) << (6 - 2 * k);
                    }
                    reu[base + v * N + u] = b;
                }
            }
        }
    }
    reu[at(SKY_BANK, 0)..at(SKY_BANK, N * N)].fill(0xFF);
    reu
}
