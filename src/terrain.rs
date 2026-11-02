//! Terrain LOD: Röttger et al., "Real-Time Generation of Continuous Levels of
//! Detail for Height Fields" (1998), adapted to a flat-shaded painter's
//! renderer with a 256-command budget.
//!
//! The world is a (2^LEVELS + 1)² byte heightmap. Every quadtree node is
//! identified by its centre sample; centres of different nodes never
//! coincide, so per-node data (`d2` roughness, `hmax`) lives in byte matrices
//! the same size as the heightmap — all of it streamable from the REU.
//!
//! A node subdivides when `l < d · C · max(c · d2, 1)` (l: L1 distance from the
//! camera to its centre, d: its edge length). Propagating `d2` upward with
//! `K = C / (2(C − 1))` guarantees neighbouring leaves differ by at most one
//! level, so a leaf only has to add the midpoint of each edge whose same-size
//! neighbour subdivides: no cracks, no skirts. A leaf is drawn as a corner fan
//! of its 4–8 boundary vertices (2–6 triangles); its centre is not needed.

use crate::camera::{Camera, WorldPos};
use crate::{EXTERIOR_TERRAIN, GeometrySink, Vec3};

pub const LEVELS: u32 = 10;
pub const SIZE: usize = (1 << LEVELS) + 1;
/// World units between samples (16 m at 0.25 m per unit).
pub const SPACING: i32 = 64;
/// World units per height step (2 m).
pub const HEIGHT_SCALE: i32 = 8;
/// `d2` is stored as `ceil(16 · deviation / edge length)`, saturating at 255.
pub const D2_ONE: i32 = 16;
/// Nodes farther than this (Euclidean lower bound) are culled.
pub const FAR: i32 = 28_000;
/// The minimum global resolution C (Q8). It fixes the propagation factor K.
pub const C_MIN_Q8: i32 = 2 * 256;

/// Terrain kinds: each has a lit / mid / shadow colour ramp.
pub mod kind {
    pub const WATER: u8 = 0;
    pub const SAND: u8 = 1;
    pub const GRASS: u8 = 2;
    pub const FOREST: u8 = 3;
    pub const ROCK: u8 = 4;
    pub const SNOW: u8 = 5;
    pub const CONCRETE: u8 = 6;
    pub const FIELD: u8 = 7;
    pub const COUNT: usize = 8;
}

pub const RAMPS: [[u8; 3]; kind::COUNT] = [
    [6, 6, 6],    // water
    [7, 8, 8],    // sand
    [13, 5, 5],   // grass
    [5, 5, 11],   // forest
    [15, 12, 11], // rock
    [1, 15, 12],  // snow
    [15, 12, 12], // concrete
    [7, 13, 5],   // field
];

/// Per-sample world data. Implementations: the host's in-memory world, and
/// on the target a RAM cache filled from the REU.
pub trait TerrainData {
    fn height(&self, i: usize, j: usize) -> u8;
    fn kind(&self, i: usize, j: usize) -> u8;
    /// Roughness of the node centred at (i, j).
    fn d2(&self, i: usize, j: usize) -> u8;
    /// Highest sample under the node centred at (i, j).
    fn hmax(&self, i: usize, j: usize) -> u8;
}

pub fn world_x(i: usize) -> i32 {
    (i as i32 - (SIZE as i32 >> 1)) * SPACING
}

pub fn world_pos(data: &impl TerrainData, i: usize, j: usize) -> WorldPos {
    WorldPos { x: world_x(i), y: data.height(i, j) as i32 * HEIGHT_SCALE, z: world_x(j) }
}

/// Fractional grid coordinate of a world coordinate.
pub fn grid_of(world: i32) -> i32 {
    world.div_euclid(SPACING) + (SIZE as i32 >> 1)
}

/// Terrain height (world units) under a world position, interpolated on the
/// finest triangles (split SW–NE).
pub fn height_at(data: &impl TerrainData, x: i32, z: i32) -> i32 {
    let half = SIZE as i32 >> 1;
    let gx = x + half * SPACING;
    let gz = z + half * SPACING;
    let max = (SIZE as i32 - 1) * SPACING - 1;
    let (gx, gz) = (gx.clamp(0, max), gz.clamp(0, max));
    let (i, j) = ((gx / SPACING) as usize, (gz / SPACING) as usize);
    let (fx, fz) = (gx % SPACING, gz % SPACING);
    let h = |di: usize, dj: usize| data.height(i + di, j + dj) as i32 * HEIGHT_SCALE;
    let (h00, h10, h01, h11) = (h(0, 0), h(1, 0), h(0, 1), h(1, 1));
    if fx >= fz {
        h00 + ((h10 - h00) * fx + (h11 - h10) * fz) / SPACING
    } else {
        h00 + ((h11 - h01) * fx + (h01 - h00) * fz) / SPACING
    }
}

/// Level-of-detail settings; `resolution_q8` (c) is what the per-frame
/// budget governor adjusts.
#[derive(Clone, Copy, Debug)]
pub struct Lod {
    pub resolution_q8: i32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub nodes: u32,
    pub culled: u32,
    pub leaves: u32,
    pub triangles: u32,
    /// Samples read (heights, kinds, d2, hmax) — the streaming footprint.
    pub reads: u32,
    /// Neighbouring leaves more than one level apart (would crack). Must be 0.
    pub restriction_violations: u32,
}

struct Walk<'a, D: TerrainData, S: GeometrySink> {
    data: &'a D,
    camera: &'a Camera,
    lod: Lod,
    sink: &'a mut S,
    stats: Stats,
    /// The camera's height above the ground beneath it: the same for every
    /// node, so neighbouring centres' distances differ by at most one edge
    /// length (the bound Röttger's crack-free proof needs).
    altitude: u32,
}

/// World light direction (towards the sun, from the north-west and above).
const LIGHT: [i32; 3] = [-4, 7, 3];

/// Emit the visible terrain into `sink`. All arithmetic is 32-bit or
/// narrower so the walk compiles to reasonable 6502 code.
pub fn render<D: TerrainData, S: GeometrySink>(data: &D, camera: &Camera, lod: Lod, sink: &mut S) -> Stats {
    let half = SIZE >> 1;
    let ground = height_at(data, camera.position.x, camera.position.z);
    let altitude = (camera.position.y - ground).max(0) as u32;
    let mut walk = Walk { data, camera, lod, sink, stats: Stats::default(), altitude };
    walk.node(half, half, half);
    walk.stats
}

const _: () = assert!(C_MIN_Q8 == 512, "subdivides() hard-codes C = 2");

impl<D: TerrainData, S: GeometrySink> Walk<'_, D, S> {
    /// Röttger's subdivision predicate for the node centred at (i, j) with
    /// half-size h, or false for nodes outside the map:
    /// `l < d · C · max(c · d2, 1)` with C = 2 and the factor in Q8.
    // Not inlined: `node` recurses, so whatever it inlines lands on the small
    // soft stack at every level; these get static frames on the C64.
    #[inline(never)]
    fn subdivides(&mut self, i: isize, j: isize, h: usize) -> bool {
        if h < 2 || i < 0 || j < 0 || i as usize >= SIZE || j as usize >= SIZE {
            return false;
        }
        let (i, j) = (i as usize, j as usize);
        self.stats.reads += 1;
        let c = &self.camera.position;
        let l = (world_x(i) - c.x).unsigned_abs() + (world_x(j) - c.z).unsigned_abs() + self.altitude;
        let d = 2 * h as u32 * SPACING as u32;
        let rough = (self.lod.resolution_q8 as u32 * self.data.d2(i, j) as u32 / D2_ONE as u32).clamp(256, 1 << 19);
        // 2d·rough/256 = (2d/16)·(rough/16); d is a multiple of 128.
        l < (2 * d >> 4) * (rough >> 4)
    }

    fn node(&mut self, i: usize, j: usize, h: usize) {
        self.stats.nodes += 1;
        if self.culled(i, j, h) {
            self.stats.culled += 1;
            return;
        }
        if self.subdivides(i as isize, j as isize, h) {
            let q = h / 2;
            self.node(i - q, j - q, q);
            self.node(i - q, j + q, q);
            self.node(i + q, j - q, q);
            self.node(i + q, j + q, q);
        } else {
            self.leaf(i, j, h);
        }
    }

    /// Bounding-sphere test against the view distance and the four side
    /// planes (|x| < z, |y| < 0.51 z) in camera space.
    // Not inlined: `node` recurses, so whatever it inlines lands on the small
    // soft stack at every level; these get static frames on the C64.
    #[inline(never)]
    fn culled(&mut self, i: usize, j: usize, h: usize) -> bool {
        self.stats.reads += 1;
        let top = self.data.hmax(i, j) as i32 * HEIGHT_SCALE;
        let extent = h as i32 * SPACING;
        // Conservative radius of the box (extent, extent, top/2):
        // √(2e² + t²) ≤ √2·e + t.
        let r = (extent * 1449 >> 10) + top / 2 + 1;
        let c = &self.camera.position;
        let d = [world_x(i) - c.x, top / 2 - c.y, world_x(j) - c.z];
        let horizontal = d[0].abs().max(d[2].abs());
        if horizontal > FAR + r {
            return true;
        }
        if horizontal <= r || d.iter().any(|v| v.abs() > crate::camera::REACH) {
            return false;
        }
        // Camera-space centre: |m| ≤ 2^14 and |d| ≤ 32,000 keep the sums in i32.
        let m = &self.camera.view.m;
        let v: [i32; 3] = core::array::from_fn(|k| (m[k][0] * d[0] + m[k][1] * d[1] + m[k][2] * d[2]) >> 14);
        let [x, y, z] = v;
        // Plane x = z has unit normal (1, 0, -1)/√2: distance (x − z)/√2 > r.
        let r_sqrt2 = r * 1449 >> 10;
        if z < -r || x - z > r_sqrt2 || -x - z > r_sqrt2 {
            return true;
        }
        // Plane 133·y = 68·z: normal length 149.4.
        let r_vertical = r * 149 + (r >> 1);
        133 * y - 68 * z > r_vertical || -133 * y - 68 * z > r_vertical
    }

    // Not inlined: `node` recurses, so whatever it inlines lands on the small
    // soft stack at every level; these get static frames on the C64.
    #[inline(never)]
    fn leaf(&mut self, i: usize, j: usize, h: usize) {
        self.stats.leaves += 1;
        let (ii, jj) = (i as isize, j as isize);
        let hs = h as isize;
        // Boundary clockwise seen from above: SW, W, NW, N, NE, E, SE, S.
        // A midpoint is present when the neighbour across that edge subdivides.
        let mut points = [(0usize, 0usize); 8];
        let mut n = 0;
        let corners = [(ii - hs, jj - hs), (ii - hs, jj + hs), (ii + hs, jj + hs), (ii + hs, jj - hs)];
        let mids = [(ii - hs, jj), (ii, jj + hs), (ii + hs, jj), (ii, jj - hs)];
        let neighbours = [(ii - 2 * hs, jj), (ii, jj + 2 * hs), (ii + 2 * hs, jj), (ii, jj - 2 * hs)];
        for side in 0..4 {
            points[n] = (corners[side].0 as usize, corners[side].1 as usize);
            n += 1;
            let (ni, nj) = neighbours[side];
            if self.subdivides(ni, nj, h) {
                points[n] = (mids[side].0 as usize, mids[side].1 as usize);
                n += 1;
                #[cfg(any(test, feature = "host"))]
                self.check_restriction(ii, jj, hs, ni, nj);
            }
        }
        self.stats.reads += n as u32 + 1;
        let kind = self.data.kind(i, j) as usize % kind::COUNT;
        let mut world = [WorldPos::default(); 8];
        let mut cam = [Vec3::ZERO; 8];
        for k in 0..n {
            world[k] = world_pos(self.data, points[k].0, points[k].1);
            match self.camera.to_camera(world[k]) {
                Some(v) => cam[k] = v,
                None => return,
            }
        }
        // Horizontal offsets within the leaf are multiples of h·SPACING.
        let unit_shift = (h as u32 * SPACING as u32).trailing_zeros();
        if n == 4 {
            // Two triangles: pick the diagonal that best matches the centre.
            let centre = self.data.height(i, j) as i32 * HEIGHT_SCALE;
            let d02 = ((world[0].y + world[2].y) / 2 - centre).abs();
            let d13 = ((world[1].y + world[3].y) / 2 - centre).abs();
            let (first, second) = if d02 <= d13 { ([0, 1, 2], [0, 2, 3]) } else { ([1, 2, 3], [1, 3, 0]) };
            self.emit(&world, &cam, first, kind, unit_shift);
            self.emit(&world, &cam, second, kind, unit_shift);
        } else {
            for k in 1..n - 1 {
                self.emit(&world, &cam, [0, k, k + 1], kind, unit_shift);
            }
        }
    }

    /// Host check: the neighbour's children along this edge must be leaves,
    /// or they would add vertices this leaf does not have (a crack).
    #[cfg(any(test, feature = "host"))]
    fn check_restriction(&mut self, ii: isize, jj: isize, hs: isize, ni: isize, nj: isize) {
        let q = hs / 2;
        for (di, dj) in [(-q, -q), (-q, q), (q, -q), (q, q)] {
            let (ci, cj) = (ni + di, nj + dj);
            let touches = (ci - ii).abs() == 2 * hs - q && (cj - jj).abs() < hs
                || (cj - jj).abs() == 2 * hs - q && (ci - ii).abs() < hs;
            if touches && self.subdivides(ci, cj, q as usize) {
                self.stats.restriction_violations += 1;
            }
        }
    }

    // Not inlined: `node` recurses, so whatever it inlines lands on the small
    // soft stack at every level; these get static frames on the C64.
    #[inline(never)]
    fn emit(&mut self, world: &[WorldPos; 8], cam: &[Vec3; 8], t: [usize; 3], kind: usize, unit_shift: u32) {
        let [a, b, c] = t.map(|k| world[k]);
        let shade = shade(a, b, c, unit_shift);
        let color = RAMPS[kind][shade];
        self.stats.triangles += 1;
        self.sink.triangle(cam[t[0]], cam[t[1]], cam[t[2]], color, EXTERIOR_TERRAIN);
    }
}

/// 0 = lit, 1 = mid, 2 = shadow, from the angle between the triangle's normal
/// and the light. Horizontal edges are whole multiples of the leaf unit
/// (2^unit_shift), so they become small integers; heights are scaled to match
/// with 6 fractional bits. Everything stays in i32.
fn shade(a: WorldPos, b: WorldPos, c: WorldPos, unit_shift: u32) -> usize {
    let horizontal = |v: i32| (v >> unit_shift) << 6;
    let vertical = |v: i32| (v << 6) >> unit_shift;
    let u = [horizontal(b.x - a.x), vertical(b.y - a.y), horizontal(b.z - a.z)];
    let v = [horizontal(c.x - a.x), vertical(c.y - a.y), horizontal(c.z - a.z)];
    let mut n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    while n.iter().any(|x| x.abs() >= 1 << 9) {
        n = n.map(|x| x >> 1);
    }
    let dot = n[0] * LIGHT[0] + n[1] * LIGHT[1] + n[2] * LIGHT[2];
    if dot <= 0 {
        return 2;
    }
    // Exact cos² comparisons; |n| < 2^9 and |LIGHT|² = 74 keep them in i32.
    let nn = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
    let ll = LIGHT[0] * LIGHT[0] + LIGHT[1] * LIGHT[1] + LIGHT[2] * LIGHT[2];
    let dot2 = dot * dot;
    if dot2 * 25 >= 16 * nn * ll {
        0 // cos >= 0.8
    } else if dot2 * 10 >= 3 * nn * ll {
        1 // cos >= 0.55
    } else {
        2
    }
}

/// Propagation factor K = C / (2(C − 1)), Q8.
pub const K_Q8: i32 = (C_MIN_Q8 * 256) / (2 * (C_MIN_Q8 - 256));

#[cfg(test)]
mod tests {
    use super::*;

    fn reference(a: WorldPos, b: WorldPos, c: WorldPos) -> usize {
        let u = [(b.x - a.x) as f64, (b.y - a.y) as f64, (b.z - a.z) as f64];
        let v = [(c.x - a.x) as f64, (c.y - a.y) as f64, (c.z - a.z) as f64];
        let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        let l = LIGHT.map(|x| x as f64);
        let cos = (n[0] * l[0] + n[1] * l[1] + n[2] * l[2])
            / ((n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() * (l[0] * l[0] + l[1] * l[1] + l[2] * l[2]).sqrt());
        if cos >= 0.8 { 0 } else if cos * cos >= 0.3 { 1 } else { 2 }
    }

    #[test]
    fn integer_shading_matches_float_mostly() {
        let mut state = 99u32;
        let mut next = |m: u32| {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            (state >> 8) % m
        };
        let (mut same, mut total) = (0, 0);
        for _ in 0..5000 {
            let shift = 6 + next(8);
            let unit = 1i32 << shift;
            let y = |n: u32| n as i32 * HEIGHT_SCALE;
            let a = WorldPos { x: 0, y: y(next(200)), z: 0 };
            let b = WorldPos { x: 0, y: y(next(200)), z: 2 * unit };
            let c = WorldPos { x: 2 * unit, y: y(next(200)), z: 2 * unit };
            let got = shade(a, b, c, shift);
            let want = reference(a, b, c);
            assert!(got.abs_diff(want) <= 1, "{a:?} {b:?} {c:?}: {got} vs {want}");
            total += 1;
            same += (got == want) as u32;
        }
        assert!(same * 100 >= total * 90, "{same}/{total}");
    }
}

/// Per-frame budget governor: scales the resolution toward a command
/// budget (measured after culling), damped to avoid popping.
pub struct Governor {
    pub lod: Lod,
    pub budget: u16,
}

impl Default for Governor {
    fn default() -> Self {
        Self::new()
    }
}

impl Governor {
    #[inline(always)]
    pub const fn new() -> Self {
        Self { lod: Lod { resolution_q8: 256 * 8 }, budget: 150 }
    }

    /// Inlined: it runs from resident code (on the C64 the terrain module is paged).
    #[inline(always)]
    pub fn update(&mut self, commands: u32) {
        let r = &mut self.lod.resolution_q8;
        if commands > self.budget as u32 {
            *r = (*r - (*r >> 3)).max(64);
        } else if commands < self.budget as u32 * 85 / 100 {
            *r = (*r + (*r >> 4) + 1).min(256 * 256);
        }
    }
}
