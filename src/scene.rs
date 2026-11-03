//! The 3D scene (host): low-poly meshes (trees, buildings, pad markers, mountains) placed on the
//! generated island (`world`), baked into the scene block, the object directory and records that
//! `geom::draw` streams (`geom::scene` layout, `format::OBJ_DIR`).

use crate::format::SCENE_MAX;
use crate::geom::scene as fmt;
use std::f32::consts::TAU;

/// The sun (plane axes u, v, then up), for the faces' baked Lambert shade.
const SUN: [f32; 3] = [0.45, 0.35, 0.82];

/// Shade ramps (`poly::PATTERNS` indices: codes 0 green, 1 light blue, 2 brown, 3 blue), lit to dark.
#[derive(Clone, Copy)]
pub enum Material {
    /// Trees: green and blue (plain green would vanish on the grass).
    Foliage,
    /// Mountains: brown, lit with light blue, shaded with blue.
    Rock,
    /// Walls: light blue to brown.
    Wall,
    /// Roofs: brown to blue.
    Roof,
    /// Pad markers: light blue.
    Marker,
    /// The enemy's vehicles and sites: brown, shaded to blue.
    Enemy,
    /// The allies': light blue, shaded to brown.
    Allied,
    /// Tracers and missiles: light blue throughout.
    Bright,
}

impl Material {
    fn ramp(self) -> [u8; 4] {
        match self {
            Material::Foliage => [9, 15, 3, 3],
            Material::Rock => [5, 2, 8, 13],
            Material::Wall => [1, 5, 11, 2],
            Material::Roof => [2, 8, 13, 3],
            Material::Marker => [1, 1, 7, 7],
            Material::Enemy => [2, 8, 13, 3],
            Material::Allied => [1, 5, 11, 2],
            Material::Bright => [1, 1, 1, 1],
        }
    }
}

/// A mesh in texels (plane axes u, v, and up h), faces wound outward.
#[derive(Default, Clone)]
pub struct Mesh {
    pub verts: Vec<[f32; 3]>,
    pub faces: Vec<([usize; 3], u8)>,
    /// Level of detail: (mesh drawn instead, beyond this many texels along the view).
    pub lod: Option<(u8, u8)>,
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The winding `geom` draws: a face is in front when `(b-a)×(c-a)` (plane axes u, v, up) points at the
/// viewer (checked by `tests::a_face_toward_the_camera_is_drawn_and_away_is_culled`).
pub fn normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    cross(sub(b, a), sub(c, a))
}

impl Mesh {
    /// The bounding radius about the origin, whole texels rounded up, plus one.
    pub fn radius(&self) -> u8 {
        self.verts.iter().map(|v| dot(*v, *v).sqrt()).fold(0f32, f32::max).ceil() as u8 + 1
    }

    /// A shared vertex (each is transformed once a frame).
    fn vert(&mut self, p: [f32; 3]) -> usize {
        if let Some(i) = self.verts.iter().position(|q| sub(*q, p).iter().all(|d| d.abs() < 1e-4)) {
            return i;
        }
        self.verts.push(p);
        self.verts.len() - 1
    }

    /// A quad `p0..p3` (in order round its edge), wound so its normal points along `out`.
    pub fn quad(&mut self, p: [[f32; 3]; 4], out: [f32; 3], mat: Material) {
        let mut p = p;
        if dot(normal(p[0], p[1], p[2]), out) < 0.0 {
            p.reverse();
        }
        let n = normal(p[0], p[1], p[2]);
        let len = dot(n, n).sqrt().max(1e-6);
        let lit = dot(n, SUN) / len / dot(SUN, SUN).sqrt();
        let level = if lit > 0.6 { 0 } else if lit > 0.2 { 1 } else if lit > -0.2 { 2 } else { 3 };
        let shade = mat.ramp()[level];
        let i: Vec<usize> = p.iter().map(|&q| self.vert(q)).collect();
        self.faces.push(([i[0], i[1], i[2]], shade));
        self.faces.push(([i[0], i[2], i[3]], shade));
    }

    /// A triangle, wound so its normal points along `out`.
    pub fn tri(&mut self, p: [[f32; 3]; 3], out: [f32; 3], mat: Material) {
        let mut p = p;
        if dot(normal(p[0], p[1], p[2]), out) < 0.0 {
            p.swap(1, 2);
        }
        let n = normal(p[0], p[1], p[2]);
        let lit = dot(n, SUN) / dot(n, n).sqrt().max(1e-6) / dot(SUN, SUN).sqrt();
        let level = if lit > 0.6 { 0 } else if lit > 0.2 { 1 } else if lit > -0.2 { 2 } else { 3 };
        let i = p.map(|q| self.vert(q));
        self.faces.push((i, mat.ramp()[level]));
    }

    /// A traffic cone: `n` sides, bands of (top height, top radius, material) stacked from a base of
    /// radius `r0`, closed by a tip at `tip`.
    pub fn cone(&mut self, n: usize, r0: f32, bands: &[(f32, f32, Material)], tip: (f32, Material)) {
        let ring = |k: usize, h: f32, r: f32| {
            let a = k as f32 / n as f32 * TAU;
            [r * a.cos(), r * a.sin(), h]
        };
        let (mut h0, mut r) = (0.0, r0);
        for &(h1, r1, mat) in bands {
            for k in 0..n {
                let mid = (k as f32 + 0.5) / n as f32 * TAU;
                let out = [mid.cos(), mid.sin(), 0.3];
                self.quad([ring(k, h0, r), ring(k + 1, h0, r), ring(k + 1, h1, r1), ring(k, h1, r1)], out, mat);
            }
            (h0, r) = (h1, r1);
        }
        for k in 0..n {
            let mid = (k as f32 + 0.5) / n as f32 * TAU;
            self.tri([ring(k, h0, r), ring(k + 1, h0, r), [0.0, 0.0, tip.0]], [mid.cos(), mid.sin(), 0.5], tip.1);
        }
    }

    /// An axis-aligned box (sides of `side`, top `top`, none when it is hidden under something; no
    /// bottom).
    pub fn cuboid(&mut self, lo: [f32; 3], hi: [f32; 3], side: Material, top: Option<Material>) {
        let c = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
        let corner = |i: usize| [if i & 1 != 0 { hi[0] } else { lo[0] }, if i & 2 != 0 { hi[1] } else { lo[1] }, if i & 4 != 0 { hi[2] } else { lo[2] }];
        let faces: [([usize; 4], Option<Material>); 5] =
            [([4, 5, 7, 6], top), ([0, 1, 5, 4], Some(side)), ([2, 3, 7, 6], Some(side)), ([0, 2, 6, 4], Some(side)), ([1, 3, 7, 5], Some(side))];
        for (f, m) in faces {
            let Some(m) = m else { continue };
            let p = f.map(corner);
            let fc = [0, 1, 2].map(|a| p.iter().map(|q| q[a]).sum::<f32>() / 4.0);
            self.quad(p, sub(fc, c), m);
        }
    }

    /// A gabled roof over `lo..hi` (u, v) from eaves height `h0` to a ridge at `h1` along u.
    pub fn gable(&mut self, lo: [f32; 2], hi: [f32; 2], h0: f32, h1: f32, wall: Material, roof: Material) {
        let mv = (lo[1] + hi[1]) / 2.0;
        let (a, b, c, d) = ([lo[0], lo[1], h0], [hi[0], lo[1], h0], [hi[0], hi[1], h0], [lo[0], hi[1], h0]);
        let (r0, r1) = ([lo[0], mv, h1], [hi[0], mv, h1]);
        self.quad([a, b, r1, r0], [0.0, -1.0, 1.0], roof);
        self.quad([d, c, r1, r0], [0.0, 1.0, 1.0], roof);
        self.tri([a, d, r0], [-1.0, 0.0, 0.0], wall);
        self.tri([b, c, r1], [1.0, 0.0, 0.0], wall);
    }

    /// `other`'s faces added, moved by `at`.
    pub fn add(&mut self, other: &Mesh, at: [f32; 3]) {
        for (f, shade) in &other.faces {
            let i = f.map(|k| {
                let v = other.verts[k];
                self.vert([v[0] + at[0], v[1] + at[1], v[2] + at[2]])
            });
            self.faces.push((i, *shade));
        }
    }

    /// A thin plate both ways round (`p0..p3`): seen from either side.
    pub fn plate(&mut self, p: [[f32; 3]; 4], mat: Material) {
        let n = normal(p[0], p[1], p[2]);
        self.quad(p, n, mat);
        self.quad(p, [-n[0], -n[1], -n[2]], mat);
    }

    /// A mountain: a ring of `n` jittered points (radius `r`, from `seed`) round a peak `h` high.
    pub fn mountain(&mut self, n: usize, r: f32, h: f32, seed: u32) {
        let jit = |k: usize, s: u32| {
            let x = (k as u32).wrapping_mul(0x9E37_79B1) ^ seed.wrapping_mul(0x85EB_CA77) ^ s;
            let x = (x ^ x >> 15).wrapping_mul(0x2C1B_3C6D);
            (x >> 8) as f32 / (1u32 << 24) as f32
        };
        let ring: Vec<[f32; 3]> = (0..n)
            .map(|k| {
                let a = (k as f32 + 0.4 * jit(k, 1)) / n as f32 * TAU;
                let rr = r * (0.7 + 0.3 * jit(k, 2));
                [rr * a.cos(), rr * a.sin(), 0.0]
            })
            .collect();
        let peak = [r * 0.15 * (jit(0, 3) - 0.5), r * 0.15 * (jit(0, 4) - 0.5), h];
        for k in 0..n {
            let (p, q) = (ring[k], ring[(k + 1) % n]);
            let out = [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0, h * 0.3];
            self.tri([p, q, peak], out, Material::Rock);
        }
    }
}

/// The mountain meshes' (base radius, peak height), texels (`goap::hazard_map` uses them too).
pub const MOUNTAIN_SHAPES: [(f32, f32); mesh::MOUNTAINS as usize] = [(40.0, 34.0), (30.0, 22.0), (50.0, 28.0)];

/// An object on the map: position (texels), base height (texels above the ground), yaw (256 a turn),
/// mesh.
#[derive(Clone, Copy, Debug)]
pub struct Object {
    pub u: f32,
    pub v: f32,
    pub y: f32,
    pub yaw: u8,
    pub mesh: u8,
}

/// Mesh indices.
pub mod mesh {
    pub const TREE: u8 = 0;
    pub const HOUSE: u8 = 1;
    pub const HANGAR: u8 = 2;
    pub const TOWER: u8 = 3;
    pub const MARKER: u8 = 4;
    /// Three mountain shapes.
    pub const MOUNTAIN: u8 = 5;
    pub const MOUNTAINS: u8 = 3;
}

/// The meshes, in texels (u, v, up; one texel is about 8 m).
pub fn meshes() -> Vec<Mesh> {
    let mut out = Vec::new();
    // Tree: a three-sided spike, about 24 m tall.
    let mut m = Mesh::default();
    m.cone(3, 0.9, &[], (3.0, Material::Foliage));
    out.push(m);
    // Far objects get one depth key (`geom::affine`), so each mesh must draw right with its front faces
    // in reverse list order: convex, hidden faces left out, and parts that can be in front listed first.
    // House: walls and a gabled roof (the box's top is hidden under it).
    let mut m = Mesh::default();
    m.cuboid([-1.5, -1.0, 0.0], [1.5, 1.0, 1.0], Material::Wall, None);
    m.gable([-1.5, -1.0], [1.5, 1.0], 1.0, 1.8, Material::Wall, Material::Roof);
    out.push(m);
    // Hangar: a long box with a low roof.
    let mut m = Mesh::default();
    m.cuboid([-5.0, -3.0, 0.0], [5.0, 3.0, 2.5], Material::Wall, Some(Material::Roof));
    out.push(m);
    // Control tower: a cab on a shaft (the cab first, drawn over the shaft; the shaft's top is inside it).
    let mut m = Mesh::default();
    m.cuboid([-1.2, -1.2, 5.0], [1.2, 1.2, 6.2], Material::Roof, Some(Material::Roof));
    m.cuboid([-0.5, -0.5, 0.0], [0.5, 0.5, 5.0], Material::Wall, None);
    out.push(m);
    // Pad marker: a small pyramid.
    let mut m = Mesh::default();
    m.cone(3, 0.5, &[], (0.8, Material::Marker));
    out.push(m);
    // Mountains.
    for (k, (r, h)) in MOUNTAIN_SHAPES.into_iter().enumerate() {
        let mut m = Mesh::default();
        m.mountain(6, r, h, 17 + k as u32);
        out.push(m);
    }
    out
}

fn hash(u: i32, v: i32, salt: u32) -> f32 {
    let x = (u as u32).wrapping_mul(0x9E37_79B1) ^ (v as u32).wrapping_mul(0x85EB_CA77) ^ salt.wrapping_mul(0xC2B2_AE3D);
    let x = (x ^ x >> 15).wrapping_mul(0x2C1B_3C6D);
    ((x ^ x >> 13) >> 8) as f32 / (1u32 << 24) as f32
}

/// The objects on `world`: mountains along the range, trees in the forests, the base at the first pad
/// (hangar, tower, houses), markers at every pad's corners and a few houses by the others.
pub fn objects(world: &crate::world::World) -> Vec<Object> {
    use crate::world::texel;
    const N: usize = crate::floor::MAP_TEXELS as usize;
    let at = |u: usize, v: usize| world.texels[v * N + u];
    let mut out = Vec::new();
    let yaw = |u: i32, v: i32| (hash(u, v, 3) * 256.0) as u8;
    // Mountains: on a 56-texel lattice (jittered) where the range is high, sunk a texel into the ground;
    // none over a pad or its buildings.
    for v in (28..N - 28).step_by(56) {
        for u in (28..N - 28).step_by(56) {
            let ju = u as f32 + (hash(u as i32, v as i32, 1) - 0.5) * 30.0;
            let jv = v as f32 + (hash(u as i32, v as i32, 2) - 0.5) * 30.0;
            let e = world.elevation[jv as usize * N + ju as usize];
            let k = (hash(u as i32, v as i32, 4) * mesh::MOUNTAINS as f32) as u8;
            let clear = MOUNTAIN_SHAPES[k as usize].0 + 24.0;
            let on_pad = world.pads.iter().any(|p| (ju - p.u as f32).hypot(jv - p.v as f32) < clear + p.half as f32);
            if e > 0.5 && !on_pad {
                out.push(Object { u: ju, v: jv, y: -1.0, yaw: yaw(u as i32, v as i32), mesh: mesh::MOUNTAIN + k });
            }
        }
    }
    // Trees: one per 10-texel square of forest, about a third of them.
    for v in (5..N - 5).step_by(10) {
        for u in (5..N - 5).step_by(10) {
            let ju = u + (hash(u as i32, v as i32, 5) * 8.0) as usize - 4;
            let jv = v + (hash(u as i32, v as i32, 6) * 8.0) as usize - 4;
            if at(ju, jv) == texel::FOREST && hash(u as i32, v as i32, 7) < 0.33 {
                out.push(Object { u: ju as f32 + 0.5, v: jv as f32 + 0.5, y: 0.0, yaw: yaw(u as i32, v as i32), mesh: mesh::TREE });
            }
        }
    }
    for (k, pad) in world.pads.iter().enumerate() {
        let (pu, pv, h) = (pad.u as f32, pad.v as f32, pad.half as f32);
        for (du, dv) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            out.push(Object { u: pu + du * (h + 1.5), v: pv + dv * (h + 1.5), y: 0.0, yaw: 0, mesh: mesh::MARKER });
        }
        if k == 0 {
            out.push(Object { u: pu + h + 10.0, v: pv, y: 0.0, yaw: 64, mesh: mesh::HANGAR });
            out.push(Object { u: pu - h - 6.0, v: pv - h - 2.0, y: 0.0, yaw: 0, mesh: mesh::TOWER });
            for i in 0..4 {
                out.push(Object { u: pu - h - 8.0 + 5.0 * i as f32, v: pv + h + 9.0, y: 0.0, yaw: 0, mesh: mesh::HOUSE });
            }
        } else {
            for i in 0..3 {
                let a = (k * 3 + i) as f32 * 2.1;
                let o = Object { u: pu + (h + 9.0) * a.cos(), v: pv + (h + 9.0) * a.sin(), y: 0.0, yaw: (a / TAU * 256.0) as u8, mesh: mesh::HOUSE };
                out.push(o);
            }
        }
    }
    out
}

/// The moving objects' meshes (`actors::kind`), each in the enemy's colours then the allies'
/// (`actors::mesh_of`): modelled facing +v (yaw 0 is north), in texels, at most 16 faces each.
pub fn vehicles() -> Vec<Mesh> {
    use crate::actors::kind;
    let mut out = Vec::new();
    for k in 0..kind::COUNT {
        for mat in [Material::Enemy, Material::Allied] {
            out.push(vehicle(k, mat));
        }
    }
    out
}

fn vehicle(k: u8, mat: Material) -> Mesh {
    use crate::actors::kind;
    let mut m = Mesh::default();
    let pyramid = |r: f32, h: f32, mat: Material| {
        let mut t = Mesh::default();
        t.cone(4, r, &[], (h, mat));
        t
    };
    match k {
        kind::TRUCK => m.cuboid([-0.4, -0.9, 0.0], [0.4, 0.9, 0.7], mat, Some(mat)),
        kind::APC => {
            m.cuboid([-0.5, -0.8, 0.0], [0.5, 0.8, 0.5], mat, Some(mat));
            m.add(&pyramid(0.3, 0.35, mat), [0.0, 0.0, 0.5]);
        }
        kind::TANK => {
            m.cuboid([-0.55, -0.8, 0.0], [0.55, 0.8, 0.35], mat, Some(mat));
            m.add(&pyramid(0.45, 0.4, mat), [0.0, 0.0, 0.35]);
            m.quad([[-0.06, 0.2, 0.55], [0.06, 0.2, 0.55], [0.06, 1.4, 0.55], [-0.06, 1.4, 0.55]], [0.0, 0.0, 1.0], mat);
        }
        kind::AA => {
            m.cuboid([-0.5, -0.7, 0.0], [0.5, 0.7, 0.4], mat, Some(mat));
            m.add(&pyramid(0.4, 0.35, mat), [0.0, 0.0, 0.4]);
            for x in [-0.15, 0.15] {
                m.tri([[x, 0.0, 0.6], [x, 1.1, 1.2], [x + 0.05, 0.0, 0.7]], [0.0, 0.0, 1.0], mat);
            }
        }
        kind::SAM => {
            m.cuboid([-0.5, -0.8, 0.0], [0.5, 0.8, 0.35], mat, Some(mat));
            for x in [-0.25, 0.25] {
                m.quad([[x - 0.1, -0.6, 0.4], [x + 0.1, -0.6, 0.4], [x + 0.1, 0.8, 1.2], [x - 0.1, 0.8, 1.2]], [0.0, -0.5, 1.0], Material::Bright);
            }
        }
        kind::RADAR => {
            m.cone(3, 0.25, &[], (1.6, mat));
            m.plate([[-0.7, 0.0, 1.2], [0.7, 0.0, 1.2], [0.7, 0.2, 2.0], [-0.7, 0.2, 2.0]], Material::Bright);
        }
        kind::GUNBOAT | kind::MISSILE_BOAT => {
            m.cuboid([-0.45, -1.2, 0.0], [0.45, 1.2, 0.35], mat, Some(mat));
            m.add(&pyramid(0.35, 0.45, mat), [0.0, -0.2, 0.35]);
            if k == kind::MISSILE_BOAT {
                m.quad([[-0.2, 0.3, 0.45], [0.2, 0.3, 0.45], [0.2, 1.0, 0.8], [-0.2, 1.0, 0.8]], [0.0, -0.5, 1.0], Material::Bright);
            }
        }
        kind::GUNSHIP => {
            m.cuboid([-0.3, -0.9, 0.3], [0.3, 0.9, 0.8], mat, Some(mat));
            m.plate([[-0.08, -0.9, 0.6], [0.08, -0.9, 0.6], [0.08, -2.0, 0.7], [-0.08, -2.0, 0.7]], mat);
            m.quad([[-1.2, -1.2, 1.0], [1.2, -1.2, 1.0], [1.2, 1.2, 1.0], [-1.2, 1.2, 1.0]], [0.0, 0.0, 1.0], Material::Bright);
        }
        kind::CARRIER => {
            // A flight deck (a long flat box, its top a pad) and the island to starboard.
            m.cuboid([-2.0, -7.0, 0.0], [2.0, 7.0, 1.2], mat, Some(Material::Marker));
            m.add(&pyramid(0.6, 1.4, mat), [1.4, 1.0, 1.2]);
        }
        kind::DESTROYER => {
            m.cuboid([-0.7, -2.5, 0.0], [0.7, 2.5, 0.5], mat, Some(mat));
            m.add(&pyramid(0.5, 0.8, mat), [0.0, 0.5, 0.5]);
        }
        kind::DEPOT => m.cone(5, 0.8, &[(1.0, 0.8, mat)], (1.4, mat)),
        kind::BUNKER => m.cone(4, 1.2, &[(0.5, 0.8, mat)], (0.7, mat)),
        kind::TRACER => m.plate([[-0.06, 0.0, 0.0], [0.06, 0.0, 0.0], [0.06, 1.5, 0.0], [-0.06, 1.5, 0.0]], Material::Bright),
        _ => {
            // Missile: a cross of two plates.
            m.plate([[-0.12, -0.4, 0.0], [0.12, -0.4, 0.0], [0.12, 0.4, 0.0], [-0.12, 0.4, 0.0]], Material::Bright);
            m.plate([[0.0, -0.4, -0.12], [0.0, -0.4, 0.12], [0.0, 0.4, 0.12], [0.0, 0.4, -0.12]], Material::Bright);
        }
    }
    m
}

/// A mesh as the 3D code reads it: the vertex shift, the bounding radius, then the vertices (3 × i8 each,
/// quarter texels `>> shift`) and the faces (3 indices, winding reversed for the camera's mirrored axes,
/// and the shade).
pub fn encode(m: &Mesh) -> (u8, u8, Vec<u8>, Vec<u8>) {
    assert!(m.verts.len() <= crate::geom::MAX_VERTS && m.faces.len() <= 255);
    let extent = m.verts.iter().flat_map(|v| v.iter()).fold(0f32, |a, &c| a.max(c.abs()));
    let shift = (0..=4u8).find(|&s| extent * 4.0 / (1 << s) as f32 <= 127.0).expect("mesh too large");
    let verts = m.verts.iter().flat_map(|v| v.map(|c| (c * 4.0 / (1 << shift) as f32).round() as i8 as u8)).collect();
    let faces = m.faces.iter().flat_map(|(f, shade)| [f[0] as u8, f[2] as u8, f[1] as u8, *shade]).collect();
    (shift, m.radius(), verts, faces)
}

/// The vehicles' mesh bank (`format::VEHICLES`): a table of `actors::MESH_RECORD` bytes per mesh (the
/// blob's offset in the bank u16, its length u16, vertices, faces, shift, radius), then the blobs
/// (vertices, then faces).
pub fn vehicle_bank() -> Vec<u8> {
    use crate::actors::{MESH_RECORD, MESHBUF_LEN};
    let ms = vehicles();
    let mut out = vec![0u8; ms.len() * MESH_RECORD];
    for (k, m) in ms.iter().enumerate() {
        assert!(m.faces.len() <= 16, "vehicle mesh {k}: {} faces", m.faces.len());
        let (shift, r, verts, faces) = encode(m);
        let blob = [verts, faces].concat();
        assert!(blob.len() <= MESHBUF_LEN as usize, "vehicle mesh {k}: {} bytes", blob.len());
        let at = out.len() as u16;
        let rec_at = k * MESH_RECORD;
        out[rec_at..rec_at + 2].copy_from_slice(&at.to_le_bytes());
        out[rec_at + 2..rec_at + 4].copy_from_slice(&(blob.len() as u16).to_le_bytes());
        out[rec_at + 4] = m.verts.len() as u8;
        out[rec_at + 5] = m.faces.len() as u8;
        out[rec_at + 6] = shift;
        out[rec_at + 7] = r;
        out.extend(blob);
    }
    out
}

/// The baked scene: the block (meshes, fetched whole at boot), the directory and the records.
pub struct Baked {
    pub block: Vec<u8>,
    pub dir: Vec<u8>,
    pub records: Vec<u8>,
}

/// Bake `world`'s scene.
pub fn bake(world: &crate::world::World) -> Baked {
    bake_with(&meshes(), &objects(world))
}

pub fn bake_with(meshes: &[Mesh], objects: &[Object]) -> Baked {
    use crate::geom::{BIG_CELL, BIG_GRID, CELL, CELL_PAD, GRID};
    let mut out = vec![0u8; fmt::MESHES + meshes.len() * fmt::MESH];
    out[fmt::N_MESHES] = meshes.len() as u8;
    let mut radius = Vec::new();
    for (k, m) in meshes.iter().enumerate() {
        assert!(m.verts.len() <= crate::geom::MAX_VERTS && m.faces.len() <= 255);
        let extent = m.verts.iter().flat_map(|v| v.iter()).fold(0f32, |a, &c| a.max(c.abs()));
        let shift = (0..=4u8).find(|&s| extent * 4.0 / (1 << s) as f32 <= 127.0).expect("mesh too large");
        let r = m.radius();
        radius.push(r);
        let rec = fmt::MESHES + k * fmt::MESH;
        let vo = out.len();
        for v in &m.verts {
            out.extend(v.map(|c| (c * 4.0 / (1 << shift) as f32).round() as i8 as u8));
        }
        let fo = out.len();
        // The mesh helpers wind for (u, v, up); the camera's axes are (u, up, v), a mirror of those, so
        // the winding is reversed.
        for (f, shade) in &m.faces {
            out.extend([f[0] as u8, f[2] as u8, f[1] as u8, *shade]);
        }
        out[rec..rec + 2].copy_from_slice(&(vo as u16).to_le_bytes());
        out[rec + 2] = m.verts.len() as u8;
        out[rec + 3..rec + 5].copy_from_slice(&(fo as u16).to_le_bytes());
        out[rec + 5] = m.faces.len() as u8;
        out[rec + 6] = shift;
        out[rec + 7] = r;
        let (lod, dist) = m.lod.unwrap_or((0xFF, 0));
        out[rec + 8] = lod;
        out[rec + 9] = dist;
    }
    // File each object under its centre's cell; one that reaches past the padding goes in the big grid.
    // A record holds the offset from the cell's centre: quarter texels in the grid, whole ones in the big
    // grid (`geom::SHIFT`, `geom::BIG_SHIFT`), so positions round to those.
    let mut lists: Vec<Vec<[u8; fmt::OBJECT]>> = vec![Vec::new(); GRID * GRID + BIG_GRID * BIG_GRID];
    for o in objects {
        let r = radius[o.mesh as usize] as i32;
        let big = r > CELL_PAD;
        assert!(r <= 160, "mesh {} too big for the big grid", o.mesh);
        // units per texel, cells per row, units per cell
        let (per, n, cell) = if big { (1.0, BIG_GRID, BIG_CELL) } else { (4.0, GRID, CELL * 4) };
        let (u, v) = ((o.u * per).round() as i32, (o.v * per).round() as i32);
        let (cu, cv) = (u.div_euclid(cell), v.div_euclid(cell));
        assert!((0..n as i32).contains(&cu) && (0..n as i32).contains(&cv), "object off the map at {}, {}", o.u, o.v);
        let off = |x: i32, c: i32| {
            let d = x - (c * cell + cell / 2);
            assert!((-128..128).contains(&d));
            d as i8 as u8
        };
        let dy = (o.y * per).round() as i32;
        assert!((-128..128).contains(&dy), "object height {}", o.y);
        let at = if big { GRID * GRID + cv as usize * BIG_GRID + cu as usize } else { cv as usize * GRID + cu as usize };
        // Yaw to the nearest quarter turn (`geom::object` turns meshes by quarters only).
        lists[at].push([off(u, cu), off(v, cv), dy as i8 as u8, o.yaw.wrapping_add(32) & 0xC0, o.mesh, 0, 0, 0]);
    }
    let (mut dir, mut records) = (Vec::new(), Vec::new());
    for l in &lists {
        assert!(l.len() <= crate::layout::CELL_MAX as usize, "{} objects in one cell", l.len());
        let first = (records.len() / fmt::OBJECT) as u16;
        dir.extend(first.to_le_bytes());
        dir.push(l.len() as u8);
        dir.push(l.iter().map(|r| radius[r[crate::geom::scene::O_MESH] as usize]).max().unwrap_or(0));
        for r in l {
            records.extend(r);
        }
    }
    let len = out.len() as u16;
    out[fmt::LEN..fmt::LEN + 2].copy_from_slice(&len.to_le_bytes());
    assert!(out.len() <= SCENE_MAX as usize, "scene block {} bytes", out.len());
    assert!(records.len() <= 0x10000, "{} object records", records.len() / fmt::OBJECT);
    Baked { block: out, dir, records }
}

/// Write the geometry into an REU image.
pub fn install(reu: &mut [u8], b: &Baked) {
    use crate::format::{OBJ_DIR, OBJ_RECORDS, SCENE};
    reu[SCENE as usize..SCENE as usize + b.block.len()].copy_from_slice(&b.block);
    reu[OBJ_DIR as usize..OBJ_DIR as usize + b.dir.len()].copy_from_slice(&b.dir);
    reu[OBJ_RECORDS as usize..OBJ_RECORDS as usize + b.records.len()].copy_from_slice(&b.records);
}
