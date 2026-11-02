//! The test scene (host): stunt pieces and trackside furniture placed along the circuit, baked into the
//! scene block `geom::draw` reads (`geom::scene` layout).

use crate::format::SCENE_MAX;
use crate::geom::scene as fmt;
use std::f32::consts::TAU;

/// The sun (plane axes u, v, then up), for the faces' baked Lambert shade.
const SUN: [f32; 3] = [0.45, 0.35, 0.82];

/// Shade ramps (`poly::PATTERNS` indices), lit to dark.
#[derive(Clone, Copy)]
pub enum Material {
    Grey,
    Red,
    White,
}

impl Material {
    fn ramp(self) -> [u8; 4] {
        match self {
            Material::Grey => [5, 1, 11, 10],
            Material::Red => [8, 3, 9, 15],
            Material::White => [2, 5, 1, 11],
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

    /// An axis-aligned box (sides of `side`, top `top`; no bottom).
    pub fn cuboid(&mut self, lo: [f32; 3], hi: [f32; 3], side: Material, top: Material) {
        let c = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
        let corner = |i: usize| [if i & 1 != 0 { hi[0] } else { lo[0] }, if i & 2 != 0 { hi[1] } else { lo[1] }, if i & 4 != 0 { hi[2] } else { lo[2] }];
        let faces: [([usize; 4], Material); 5] =
            [([4, 5, 7, 6], top), ([0, 1, 5, 4], side), ([2, 3, 7, 6], side), ([0, 2, 6, 4], side), ([1, 3, 7, 5], side)];
        for (f, m) in faces {
            let p = f.map(corner);
            let fc = [0, 1, 2].map(|a| p.iter().map(|q| q[a]).sum::<f32>() / 4.0);
            self.quad(p, sub(fc, c), m);
        }
    }

    /// Wedge ramp: `w` wide (x), rising along -y over `len` to `h`.
    pub fn ramp(&mut self, w: f32, len: f32, h: f32) {
        let (x0, x1, y0, y1) = (-w / 2.0, w / 2.0, len / 2.0, -len / 2.0);
        let surface = [[x0, y0, 0.0], [x1, y0, 0.0], [x1, y1, h], [x0, y1, h]];
        self.quad(surface, [0.0, len, h * 4.0], Material::Grey);
        self.quad([[x0, y1, 0.0], [x1, y1, 0.0], [x1, y1, h], [x0, y1, h]], [0.0, -1.0, 0.0], Material::Red);
        for (x, out) in [(x0, -1.0), (x1, 1.0)] {
            let i = [self.vert([x, y0, 0.0]), self.vert([x, y1, 0.0]), self.vert([x, y1, h])];
            let (a, b, c) = (self.verts[i[0]], self.verts[i[1]], self.verts[i[2]]);
            let n = normal(a, b, c);
            let f = if n[0] * out > 0.0 { [i[0], i[1], i[2]] } else { [i[0], i[2], i[1]] };
            let lit = dot([out, 0.0, 0.0], SUN);
            self.faces.push((f, if lit > 0.2 { 8 } else { 3 }));
        }
    }

    /// A vertical loop: `n` segments of a band `w` wide (x), radius `r` (inner surface) and `t` thick,
    /// standing on the ground, its axis along x.
    pub fn looping(&mut self, n: usize, w: f32, r: f32, t: f32) {
        let at = |k: usize, rad: f32, x: f32| {
            let a = k as f32 / n as f32 * TAU;
            [x, rad * a.sin(), r + t - rad * a.cos()]
        };
        for k in 0..n {
            let k1 = (k + 1) % n;
            let (x0, x1) = (-w / 2.0, w / 2.0);
            let mid = (k as f32 + 0.5) / n as f32 * TAU;
            let radial = [0.0, mid.sin(), -mid.cos()];
            let inward = [0.0, -radial[1], -radial[2]];
            // Inner (driving) surface faces the loop's axis; outer faces away.
            let stripe = if k % 2 == 0 { Material::Grey } else { Material::White };
            self.quad([at(k, r, x0), at(k, r, x1), at(k1, r, x1), at(k1, r, x0)], inward, stripe);
            self.quad([at(k, r + t, x0), at(k, r + t, x1), at(k1, r + t, x1), at(k1, r + t, x0)], radial, Material::Grey);
            for (x, out) in [(x0, -1.0), (x1, 1.0)] {
                self.quad([at(k, r, x), at(k, r + t, x), at(k1, r + t, x), at(k1, r, x)], [out, 0.0, 0.0], Material::Red);
            }
        }
    }
}

pub struct Object {
    pub x: f32,
    pub y: f32,
    /// 256 steps a turn; the mesh's -v axis points along it (as the camera's view does).
    pub yaw: u8,
    pub mesh: u8,
}

/// The track direction at `t` as a yaw, and the unit tangent and left normal.
/// mode7's circuit centreline (`../mode7/src/map.rs`), which these placeholder props still follow until
/// chopper's own objects replace them.
pub fn centre(t: f32) -> [f32; 2] {
    let a = t * TAU;
    [128.0 + 92.0 * a.cos() + 18.0 * (3.0 * a).cos(), 128.0 + 80.0 * a.sin() + 22.0 * (2.0 * a).sin()]
}

fn frame_at(t: f32) -> ([f32; 2], u8, [f32; 2]) {
    let p = centre(t);
    let q = centre(t + 1e-4);
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    let l = (dx * dx + dy * dy).sqrt();
    let a = dx.atan2(-dy);
    let yaw = ((a / TAU * 256.0).round() as i32).rem_euclid(256) as u8;
    (p, yaw, [dy / l, -dx / l])
}

pub fn meshes() -> Vec<Mesh> {
    let mut out = Vec::new();
    // 0: traffic cone with a triangular base, the cheapest that reads as a cone (red, a white stripe, red
    // tip; there is no orange in the four colours). Beyond 32 texels (under ~15 pixels tall) mesh 4, a
    // plain red one, is drawn instead.
    let mut m = Mesh::default();
    m.cone(3, 1.6, &[(1.2, 1.15, Material::Red), (1.9, 0.85, Material::White)], (3.2, Material::Red));
    m.lod = Some((4, 32));
    out.push(m);
    // 1: start gantry over the track (the camera, 12 texels up, passes under the beam).
    let mut m = Mesh::default();
    for x in [-14.0f32, 13.0] {
        m.cuboid([x, -0.5, 0.0], [x + 1.0, 0.5, 18.0], Material::Grey, Material::Grey);
    }
    m.cuboid([-14.0, -1.0, 16.0], [14.0, 1.0, 19.0], Material::White, Material::Grey);
    out.push(m);
    // 2: jump ramp.
    let mut m = Mesh::default();
    m.ramp(10.0, 16.0, 5.0);
    out.push(m);
    // 3: the loop.
    let mut m = Mesh::default();
    m.looping(16, 8.0, 10.0, 1.0);
    out.push(m);
    // 4: the far cone.
    let mut m = Mesh::default();
    m.cone(3, 1.6, &[], (3.2, Material::Red));
    out.push(m);
    out
}

pub fn objects() -> Vec<Object> {
    let mut out = Vec::new();
    // Cones along both sides, every 1/64 of the lap (~8 texels apart).
    for i in 0..64 {
        let t = (i as f32 + 0.5) / 64.0;
        let (p, yaw, n) = frame_at(t);
        for side in [1.0, -1.0] {
            out.push(Object { x: p[0] + n[0] * 14.0 * side, y: p[1] + n[1] * 14.0 * side, yaw, mesh: 0 });
        }
    }
    let (p, yaw, _) = frame_at(0.0);
    out.push(Object { x: p[0], y: p[1], yaw, mesh: 1 });
    for t in [0.2, 0.55, 0.8] {
        let (p, yaw, n) = frame_at(t);
        out.push(Object { x: p[0] + n[0] * 26.0, y: p[1] + n[1] * 26.0, yaw, mesh: 2 });
    }
    for t in [0.38, 0.68] {
        let (p, yaw, n) = frame_at(t);
        // Beside the track, its axis across it so the camera sees the ring.
        out.push(Object { x: p[0] - n[0] * 30.0, y: p[1] - n[1] * 30.0, yaw: yaw.wrapping_add(64), mesh: 3 });
    }
    out
}

fn fixed(t: f32) -> u16 {
    ((t * 256.0).round() as i64).rem_euclid(65536) as u16
}

/// The scene block and the object records (sorted by grid cell, for `format::OBJECTS`).
pub fn bake() -> (Vec<u8>, Vec<u8>) {
    bake_with(&meshes(), &objects())
}

pub fn bake_with(meshes: &[Mesh], objects: &[Object]) -> (Vec<u8>, Vec<u8>) {
    use crate::geom::{CELL_PAD, GRID, cell_of};
    let mut out = vec![0u8; fmt::MESHES + meshes.len() * fmt::MESH];
    out[fmt::N_MESHES] = meshes.len() as u8;
    let mut radius = Vec::new();
    for (k, m) in meshes.iter().enumerate() {
        assert!(m.verts.len() <= crate::geom::MAX_VERTS && m.faces.len() <= 255);
        let extent = m.verts.iter().flat_map(|v| v.iter()).fold(0f32, |a, &c| a.max(c.abs()));
        let shift = (0..=4u8).find(|&s| extent * 4.0 / (1 << s) as f32 <= 127.0).expect("mesh too large");
        let r = m.verts.iter().map(|v| dot(*v, *v).sqrt()).fold(0f32, f32::max).ceil() as u8 + 1;
        radius.push(r);
        let rec = fmt::MESHES + k * fmt::MESH;
        let vo = out.len();
        for v in &m.verts {
            out.extend(v.map(|c| (c * 4.0 / (1 << shift) as f32).round() as i8 as u8));
        }
        let fo = out.len();
        for (f, shade) in &m.faces {
            out.extend([f[0] as u8, f[1] as u8, f[2] as u8, *shade]);
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
    // File each object under its centre's cell; one that reaches past the padding (its mesh's bounding
    // radius, height included, as the cull uses it) goes in the global list, checked every frame.
    let global = GRID * GRID;
    let mut lists: Vec<Vec<[u8; fmt::OBJECT]>> = vec![Vec::new(); global + 1];
    for o in objects {
        let (x, y) = (fixed(o.x), fixed(o.y));
        let (cx, cy) = cell_of(x, y);
        let at = if radius[o.mesh as usize] as i32 > CELL_PAD { global } else { cy as usize * GRID + cx as usize };
        let [x0, x1] = x.to_le_bytes();
        let [y0, y1] = y.to_le_bytes();
        lists[at].push([x0, x1, y0, y1, o.yaw, o.mesh]);
    }
    let dir = out.len();
    out[fmt::DIR..fmt::DIR + 2].copy_from_slice(&(dir as u16).to_le_bytes());
    let mut records = Vec::new();
    for l in &lists {
        assert!(l.len() <= crate::layout::CELL_MAX as usize, "{} objects in one cell", l.len());
        let first = (records.len() / fmt::OBJECT) as u16;
        out.extend(first.to_le_bytes());
        out.push(l.len() as u8);
        out.push(l.iter().map(|r| radius[r[5] as usize]).max().unwrap_or(0));
        for r in l {
            records.extend(r);
        }
    }
    let len = out.len() as u16;
    out[fmt::LEN..fmt::LEN + 2].copy_from_slice(&len.to_le_bytes());
    assert!(out.len() <= SCENE_MAX as usize, "scene block {} bytes", out.len());
    assert!(records.len() <= 0x10000, "{} object records", records.len() / fmt::OBJECT);
    (out, records)
}

/// Write the geometry into an REU image: the scene block, the object records and the 256 visibility
/// lists.
pub fn install(reu: &mut [u8], block: &[u8], records: &[u8]) {
    use crate::format::{OBJECTS, SCENE, VIS_STRIDE, vis_list};
    reu[SCENE as usize..SCENE as usize + block.len()].copy_from_slice(block);
    reu[OBJECTS as usize..OBJECTS as usize + records.len()].copy_from_slice(records);
    for a in 0..=255u8 {
        let cells = crate::geom::visible_cells(a);
        assert!(cells.len() < crate::geom::VIS_DEPTH as usize && crate::geom::VIS_DEPTH as u32 * 2 == VIS_STRIDE);
        let at = vis_list(a) as usize;
        reu[at] = cells.len() as u8;
        for (k, (i, j, near)) in cells.iter().enumerate() {
            reu[at + 1 + k] = (*i as u8 & 15) | (*j as u8) << 4;
            reu[at + crate::geom::VIS_DEPTH as usize + k] = *near;
        }
    }
}
