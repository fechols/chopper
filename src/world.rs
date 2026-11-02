//! The ground map (host): a 2048×2048-texel island, painted in texel bytes (four multicolor pixels each,
//! so a texel is one of the four global colours or a dither of them), and its mip levels.
//!
//! Codes (`format::COLORS`): 0 green, 1 light blue, 2 brown, 3 blue.

use crate::floor::{LEVELS, MAP_TEXELS};

const N: usize = MAP_TEXELS as usize;

/// Texel bytes (pixels left to right in bits 7-6, 5-4, 3-2, 1-0).
pub mod texel {
    pub const WATER: u8 = 0xFF;
    pub const SHALLOW: u8 = 0xF7;
    pub const SAND: u8 = 0x99;
    pub const GRASS: u8 = 0x00;
    pub const MEADOW: u8 = 0x02;
    pub const FOREST: u8 = 0x33;
    pub const FIELD_A: u8 = 0x22;
    pub const FIELD_B: u8 = 0x88;
    pub const ROCK: u8 = 0xAA;
    pub const SCREE: u8 = 0xA2; // brown with a green pixel (no sky colour on the ground)
    pub const ROAD: u8 = 0xAA;
    pub const PAD: u8 = 0x55;
    pub const PAD_MARK: u8 = 0xAA;
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
    let (a, b) = (hash(xi, yi, seed), hash(xi + 1, yi, seed));
    let (c, d) = (hash(xi, yi + 1, seed), hash(xi + 1, yi + 1, seed));
    let top = a + (b - a) * sx;
    top + (c + (d - c) * sx - top) * sy
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

/// A landing pad: centre texel and half-size.
#[derive(Clone, Copy, Debug)]
pub struct Pad {
    pub u: usize,
    pub v: usize,
    pub half: usize,
}

pub struct World {
    /// Level 0, row-major (`v · 2048 + u`).
    pub texels: Vec<u8>,
    /// Elevation 0..1 per texel (for placing hills and mountains as objects later).
    pub elevation: Vec<f32>,
    pub pads: Vec<Pad>,
}

/// The home base and the training pads.
pub const PADS: [Pad; 5] = [
    Pad { u: 1024, v: 760, half: 10 },
    Pad { u: 1180, v: 900, half: 6 },
    Pad { u: 860, v: 1060, half: 6 },
    Pad { u: 1240, v: 1250, half: 6 },
    Pad { u: 980, v: 1400, half: 6 },
];

pub fn generate(seed: u32) -> World {
    let mut texels = vec![texel::WATER; N * N];
    let mut elevation = vec![0f32; N * N];
    let c = (N - 1) as f64 / 2.0;
    for v in 0..N {
        for u in 0..N {
            let (x, y) = (u as f64 / c - 1.0, v as f64 / c - 1.0);
            let angle = y.atan2(x);
            let wobble = 0.08 * (3.0 * angle + 1.3).sin() + 0.05 * (5.0 * angle).cos() + 0.06 * (fbm(x * 3.0, y * 3.0, 3, seed + 50) - 0.5);
            let r = (x * x + y * y).sqrt() / (0.8 + wobble);
            let mask = 1.0 - r * r;
            let hills = fbm(x * 5.0 + 50.0, y * 5.0 + 50.0, 6, seed);
            let range = (-((x - y * 0.6 - 0.2).powi(2) / 0.03)).exp() * (1.0 - (y + 0.1).abs()).max(0.0);
            let e = mask * 0.3 + (hills - 0.45) * 0.5 + range * 0.7 * mask.max(0.0);
            elevation[v * N + u] = e as f32;
            let detail = hash(u as i32, v as i32, seed + 7);
            let patch = fbm(x * 40.0, y * 40.0, 2, seed + 9);
            texels[v * N + u] = if e < -0.04 {
                texel::WATER
            } else if e < 0.0 {
                texel::SHALLOW
            } else if e < 0.03 {
                texel::SAND
            } else if e > 0.42 {
                // The range: rock and scree on the floor (its height comes as 3D objects). No snow: it
                // would share the sky's colour.
                if detail < 0.3 { texel::SCREE } else { texel::ROCK }
            } else if patch > 0.6 {
                texel::FOREST
            } else if patch < 0.3 && e < 0.25 {
                // Fields in strips.
                if (u / 3 + v / 11) % 2 == 0 { texel::FIELD_A } else { texel::FIELD_B }
            } else if detail < 0.08 {
                texel::MEADOW
            } else {
                texel::GRASS
            };
        }
    }
    let mut world = World { texels, elevation, pads: PADS.to_vec() };
    roads(&mut world);
    for pad in PADS {
        paint_pad(&mut world, pad);
    }
    world
}

/// Dirt roads between consecutive pads (straight, 2 texels wide).
fn roads(world: &mut World) {
    for w in PADS.windows(2) {
        let (a, b) = (w[0], w[1]);
        let steps = (a.u.abs_diff(b.u)).max(a.v.abs_diff(b.v)) * 2;
        for s in 0..=steps {
            let t = s as f64 / steps as f64;
            let u = (a.u as f64 + (b.u as f64 - a.u as f64) * t) as usize;
            let v = (a.v as f64 + (b.v as f64 - a.v as f64) * t) as usize;
            for (du, dv) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                let k = (v + dv) * N + u + du;
                if world.texels[k] != texel::WATER {
                    world.texels[k] = texel::ROAD;
                }
            }
        }
    }
}

/// A square pad with an H.
fn paint_pad(world: &mut World, pad: Pad) {
    let h = pad.half as isize;
    for dv in -h..=h {
        for du in -h..=h {
            let (u, v) = ((pad.u as isize + du) as usize, (pad.v as isize + dv) as usize);
            let (a, b) = (du.abs(), dv.abs());
            // The H: two uprights and a crossbar, inside a border.
            let mark = a < h - 1 && b < h - 1 && ((a >= h / 2 - 1 && a <= h / 2) || (b == 0 && a < h / 2));
            world.texels[v * N + u] = if mark { texel::PAD_MARK } else { texel::PAD };
        }
    }
}

/// Mip level `k` (1..LEVELS-1) from level `k − 1`: the most common byte of each 2×2 block.
/// Texel `(u, v)`'s byte as stored: dither patterns shifted by whole pixels from texel to texel, so the
/// odd pixels of a pattern do not land in the same screen columns line after line (the floor copies
/// a texel byte to a screen byte, always 4-pixel aligned). A pattern with period 2 alternates its
/// phase in a checkerboard; one with period 4 takes one of 4 rotations at random. Solid colours are
/// unchanged; the fields' stripes keep their phase.
pub fn dither(byte: u8, u: usize, v: usize) -> u8 {
    if byte == texel::FIELD_A || byte == texel::FIELD_B {
        return byte;
    }
    let r = if byte.rotate_right(4) == byte {
        (u + v) as u32 & 1
    } else {
        let h = (u as u32).wrapping_mul(0x9E37_79B1) ^ (v as u32).wrapping_mul(0x85EB_CA77);
        (h ^ h >> 15).wrapping_mul(0x2C1B_3C6D) >> 30
    };
    byte.rotate_right(2 * r)
}

pub fn mips(level0: &[u8]) -> Vec<Vec<u8>> {
    let mut levels = vec![level0.to_vec()];
    for k in 1..LEVELS as usize {
        let n = N >> k;
        let prev = &levels[k - 1];
        let pn = n * 2;
        let mut next = vec![0u8; n * n];
        for v in 0..n {
            for u in 0..n {
                let q = [prev[2 * v * pn + 2 * u], prev[2 * v * pn + 2 * u + 1], prev[(2 * v + 1) * pn + 2 * u], prev[(2 * v + 1) * pn + 2 * u + 1]];
                let best = q.iter().copied().max_by_key(|&x| (q.iter().filter(|&&y| y == x).count(), x == q[0])).unwrap();
                next[v * n + u] = best;
            }
        }
        levels.push(next);
    }
    levels
}
