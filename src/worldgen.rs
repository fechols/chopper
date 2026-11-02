//! Offline world generator (host): an island heightmap with valleys, a
//! mountain range, coastline and hand-placed features, plus the per-node
//! roughness (`d2`) and maximum-height matrices the LOD walk needs.

use crate::terrain::{self, D2_ONE, HEIGHT_SCALE, K_Q8, LEVELS, SIZE, SPACING, TerrainData, kind};
use std::vec;
use std::vec::Vec;

pub struct World {
    pub heights: Vec<u8>,
    pub kinds: Vec<u8>,
    pub d2: Vec<u8>,
    pub hmax: Vec<u8>,
    pub pads: Vec<Pad>,
}

/// A flattened landing pad / base.
#[derive(Clone, Copy, Debug)]
pub struct Pad {
    pub i: usize,
    pub j: usize,
    pub radius: usize,
}

impl TerrainData for World {
    fn height(&self, i: usize, j: usize) -> u8 {
        self.heights[j * SIZE + i]
    }
    fn kind(&self, i: usize, j: usize) -> u8 {
        self.kinds[j * SIZE + i]
    }
    fn d2(&self, i: usize, j: usize) -> u8 {
        self.d2[j * SIZE + i]
    }
    fn hmax(&self, i: usize, j: usize) -> u8 {
        self.hmax[j * SIZE + i]
    }
}

fn hash(x: i32, y: i32, seed: u32) -> f64 {
    let mut h = (x as u32).wrapping_mul(0x8da6_b343) ^ (y as u32).wrapping_mul(0xd816_3841) ^ seed.wrapping_mul(0xcb1a_b31f);
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^= h >> 15;
    (h & 0xffff) as f64 / 65535.0
}

fn value_noise(x: f64, y: f64, seed: u32) -> f64 {
    let (xi, yi) = (x.floor() as i32, y.floor() as i32);
    let (fx, fy) = (x - xi as f64, y - yi as f64);
    let s = |t: f64| t * t * (3.0 - 2.0 * t);
    let (sx, sy) = (s(fx), s(fy));
    let a = hash(xi, yi, seed);
    let b = hash(xi + 1, yi, seed);
    let c = hash(xi, yi + 1, seed);
    let d = hash(xi + 1, yi + 1, seed);
    let top = a + (b - a) * sx;
    let bottom = c + (d - c) * sx;
    top + (bottom - top) * sy
}

fn fbm(x: f64, y: f64, octaves: u32, seed: u32) -> f64 {
    let (mut sum, mut amp, mut freq, mut norm) = (0.0, 1.0, 1.0, 0.0);
    for o in 0..octaves {
        sum += amp * value_noise(x * freq, y * freq, seed + o);
        norm += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    sum / norm
}

/// Ridged noise for mountain crests.
fn ridged(x: f64, y: f64, octaves: u32, seed: u32) -> f64 {
    let (mut sum, mut amp, mut freq, mut norm) = (0.0, 1.0, 1.0, 0.0);
    for o in 0..octaves {
        let n = 1.0 - (2.0 * value_noise(x * freq, y * freq, seed + 101 + o) - 1.0).abs();
        sum += amp * n * n;
        norm += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    sum / norm
}

pub fn generate(seed: u32) -> World {
    let n = SIZE;
    let mut elevation = vec![0f64; n * n];
    let centre = (n - 1) as f64 / 2.0;
    for j in 0..n {
        for i in 0..n {
            let (x, y) = (i as f64 / centre - 1.0, j as f64 / centre - 1.0);
            // Island mask: a wobbly disc filling ~80% of the map.
            let angle = y.atan2(x);
            let wobble = 0.08 * (3.0 * angle + 1.3).sin() + 0.05 * (5.0 * angle).cos();
            let r = (x * x + y * y).sqrt() / (0.82 + wobble);
            let mask = (1.0 - r * r).clamp(-1.0, 1.0);
            let hills = fbm(x * 6.0 + 50.0, y * 6.0 + 50.0, 6, seed);
            // A mountain range running across the north-east.
            let range = (-((x - y * 0.6 - 0.25).powi(2) / 0.12)).exp() * (1.0 - (y + 0.1).abs()).max(0.0);
            let peaks = ridged(x * 4.0, y * 4.0, 5, seed);
            let e = mask * 0.25 + (hills - 0.42) * 0.7 + range * peaks * mask.max(0.0) * 1.6;
            elevation[j * n + i] = e;
        }
    }
    // Map elevation to bytes: sea level at e = 0 maps to height 4.
    let mut heights = vec![0u8; n * n];
    for k in 0..n * n {
        let e = elevation[k];
        heights[k] = if e <= 0.0 { 2 } else { (4.0 + e * 130.0).min(255.0) as u8 };
    }
    let pads = vec![
        // Home base on the south coast plain, training pads inland.
        Pad { i: 470, j: 300, radius: 6 },
        Pad { i: 560, j: 420, radius: 3 },
        Pad { i: 380, j: 520, radius: 3 },
        Pad { i: 640, j: 610, radius: 3 },
        Pad { i: 520, j: 700, radius: 3 },
    ];
    let mut kinds = vec![kind::WATER; n * n];
    for j in 0..n {
        for i in 0..n {
            let h = heights[j * n + i];
            let slope = {
                let at = |a: usize, b: usize| heights[b.min(n - 1) * n + a.min(n - 1)] as i32;
                ((at(i + 1, j) - at(i.saturating_sub(1), j)).abs() + (at(i, j + 1) - at(i, j.saturating_sub(1))).abs()) as u32
            };
            let wiggle = hash(i as i32 / 6, j as i32 / 6, seed + 9);
            kinds[j * n + i] = match h {
                0..=3 => kind::WATER,
                4..=7 => kind::SAND,
                _ if h > 120 => kind::SNOW,
                _ if slope > 10 || h > 85 => kind::ROCK,
                _ if wiggle < 0.18 && h < 40 => kind::FIELD,
                _ if wiggle > 0.55 => kind::FOREST,
                _ => kind::GRASS,
            };
        }
    }
    let mut world = World { heights, kinds, d2: vec![0; n * n], hmax: vec![0; n * n], pads: pads.clone() };
    for pad in &pads {
        flatten(&mut world, *pad);
    }
    compute_lod(&mut world);
    world
}

/// Level a square pad to the height at its centre and pave it.
fn flatten(world: &mut World, pad: Pad) {
    let level = world.heights[pad.j * SIZE + pad.i].max(8);
    let blend = pad.radius + 4;
    for j in pad.j - blend..=pad.j + blend {
        for i in pad.i - blend..=pad.i + blend {
            let d = (i as isize - pad.i as isize).unsigned_abs().max((j as isize - pad.j as isize).unsigned_abs());
            let k = j * SIZE + i;
            if d <= pad.radius {
                world.heights[k] = level;
                world.kinds[k] = kind::CONCRETE;
            } else {
                let t = (d - pad.radius) as f64 / (blend - pad.radius) as f64;
                let h = level as f64 * (1.0 - t) + world.heights[k] as f64 * t;
                world.heights[k] = h.round() as u8;
                if world.kinds[k] == kind::WATER {
                    world.kinds[k] = kind::SAND;
                }
            }
        }
    }
}

/// Röttger's d2 (with upward propagation by K) and per-node maximum height,
/// bottom-up, written at each node's centre sample.
pub fn compute_lod(world: &mut World) {
    let h = |w: &World, i: usize, j: usize| w.heights[j * SIZE + i] as i32 * HEIGHT_SCALE;
    for level in 1..=LEVELS {
        let half = 1usize << (level - 1);
        let edge = (2 * half) as i32 * SPACING;
        let mut j = half;
        while j < SIZE {
            let mut i = half;
            while i < SIZE {
                let (w, e, s, nn) = (i - half, i + half, j - half, j + half);
                let corners = [h(world, w, s), h(world, e, s), h(world, e, nn), h(world, w, nn)];
                let mut dev = 0;
                dev = dev.max((h(world, i, s) - (corners[0] + corners[1]) / 2).abs());
                dev = dev.max((h(world, e, j) - (corners[1] + corners[2]) / 2).abs());
                dev = dev.max((h(world, i, nn) - (corners[2] + corners[3]) / 2).abs());
                dev = dev.max((h(world, w, j) - (corners[3] + corners[0]) / 2).abs());
                dev = dev.max((h(world, i, j) - (corners[0] + corners[2]) / 2).abs());
                dev = dev.max((h(world, i, j) - (corners[1] + corners[3]) / 2).abs());
                // Kind boundaries count as roughness so coastlines and pads keep detail.
                let kinds = [world.kinds[j * SIZE + i], world.kinds[s * SIZE + w], world.kinds[nn * SIZE + e]];
                if kinds[0] != kinds[1] || kinds[0] != kinds[2] {
                    dev = dev.max(edge / 8);
                }
                let mut d2 = (dev * D2_ONE + edge - 1) / edge;
                let mut top = [w, i, e].iter().flat_map(|&a| [s, j, nn].map(|b| world.heights[b * SIZE + a])).max().unwrap();
                if half >= 2 {
                    let q = (half / 2) as isize;
                    let (si, sj) = (i as isize, j as isize);
                    for (ci, cj) in [(si - q, sj - q), (si + q, sj - q), (si - q, sj + q), (si + q, sj + q)] {
                        let k = cj as usize * SIZE + ci as usize;
                        d2 = d2.max((world.d2[k] as i32 * K_Q8 + 255) / 256);
                        top = top.max(world.hmax[k]);
                    }
                    // Also the children of the four edge neighbours that touch
                    // this node's edges: a subdividing one forces this node to
                    // subdivide, keeping adjacent leaves within one level.
                    let far = 3 * q;
                    for (ci, cj) in [
                        (si - far, sj - q), (si - far, sj + q), (si + far, sj - q), (si + far, sj + q),
                        (si - q, sj - far), (si + q, sj - far), (si - q, sj + far), (si + q, sj + far),
                    ] {
                        if ci >= 0 && cj >= 0 && (ci as usize) < SIZE && (cj as usize) < SIZE {
                            let k = cj as usize * SIZE + ci as usize;
                            d2 = d2.max((world.d2[k] as i32 * K_Q8 + 255) / 256);
                        }
                    }
                }
                world.d2[j * SIZE + i] = d2.min(255) as u8;
                world.hmax[j * SIZE + i] = top;
                i += 2 * half;
            }
            j += 2 * half;
        }
    }
}

/// World position of a pad's centre, at ground level.
pub fn pad_position(world: &World, pad: &Pad) -> crate::camera::WorldPos {
    terrain::world_pos(world, pad.i, pad.j)
}
