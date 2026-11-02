//! Flat-shaded 3D over the floor (`no_std`, shared by the host and the C64, so both compute the same
//! bits): per-object yaw transform into the floor camera's space, near-plane clip, projection matching
//! the floor exactly, backface cull, a one-pass bucket depth sort (painter's), and the triangle set-up
//! that feeds [`Hw::tri_walk`] (`asm/tri.S` on the C64), which feeds [`Hw::poly_row`] (`asm/poly.S`).
//!
//! Units: camera space in 1/16 texel ("Q4"), screen in 1/16 multicolour pixel; pixel `p` spans
//! `[p, p+1)` and is covered when its centre is (the floor's sampling too). All products go through
//! [`crate::fixed`] (a 16×16 multiply; division by reciprocal table).
//!
//! Camera (`camera::View::Floor`): looking along `(sin a, -cos a)` with right `(cos a, sin a)`, at height
//! `CAMERA_HEIGHT`, focal `FOCAL` (hires), horizon line `HORIZON`:
//! `sx = 80 + 80·X/Z`, `sy = 48 + 160·(12 - Y)/Z`.

use crate::fixed::{self, mul, mul_wide, mulsu, sar};
use crate::hw::Hw;
use crate::layout::{CELL_MAX, CELL_RAM, ROW_STRIDE, VIS_RAM};
use crate::poly::{PATTERNS, Row};

/// Camera-space units per texel.
pub const Q: i32 = 16;
/// Near plane: 8 texels (the floor's nearest line is 12.7 texels away).
pub const NEAR: i32 = 8 * Q;
/// Objects whose centre is farther than this are skipped. The plane wraps at 256 texels, and the
/// visibility lists reach 7 cells from the camera's cell, so every object this near is in a listed cell
/// wherever the camera is inside its own.
pub const FAR: i32 = 96 * Q;
/// The object grid: 16 × 16 cells of 16 texels over the plane. An object is filed under the cell of its
/// centre; objects reaching more than [`CELL_PAD`] texels from it (horizontally) go in the global list.
pub const CELL: i32 = 16;
pub const GRID: usize = 16;
pub const CELL_PAD: i32 = 16;
/// Cells the visibility lists reach, either way from the camera's cell.
pub const REACH: i32 = 7;
/// Objects farther than this from the camera along either plane axis are skipped, so every object the
/// cull accepts is within [`REACH`] cells (`(15 + 96) / 16 < 7`).
pub const SPAN: i32 = 96 * Q;
const _: () = assert!(CELL - 1 + SPAN / Q < (REACH + 1) * CELL);
/// Screen coordinates are clamped to ±1000 pixels (Q4), which keeps every edge product in i32.
pub const LIMIT: i32 = 1000 * 16;

/// `sin` of `a`/256 turn, Q14, a quarter wave.
const SIN_Q: [i16; 65] = [
    0, 402, 804, 1205, 1606, 2006, 2404, 2801, 3196, 3590, 3981, 4370, 4756, 5139, 5520, 5897, 6270, 6639,
    7005, 7366, 7723, 8076, 8423, 8765, 9102, 9434, 9760, 10080, 10394, 10702, 11003, 11297, 11585, 11866,
    12140, 12406, 12665, 12916, 13160, 13395, 13623, 13842, 14053, 14256, 14449, 14635, 14811, 14978, 15137,
    15286, 15426, 15557, 15679, 15791, 15893, 15986, 16069, 16143, 16207, 16261, 16305, 16340, 16364, 16379,
    16384,
];

pub fn sin(a: u8) -> i16 {
    let q = (a & 63) as usize;
    match a >> 6 {
        0 => SIN_Q[q],
        1 => SIN_Q[64 - q],
        2 => -SIN_Q[q],
        _ => -SIN_Q[64 - q],
    }
}

pub fn cos(a: u8) -> i16 {
    sin(a.wrapping_add(64))
}

/// Scene block (`format::SCENE`, fetched whole to `layout::SCENE_RAM`):
/// `[0]` length u16, `[2]` meshes u8, `[4]` grid directory offset u16, mesh table at [`scene::MESHES`].
/// The directory has `GRID²` cells plus the global list, each the first object (u16, an index into
/// `format::OBJECTS`), the count (u8) and the largest bounding radius among them (u8, texels).
pub mod scene {
    pub const LEN: usize = 0;
    pub const N_MESHES: usize = 2;
    pub const DIR: usize = 4;
    pub const DIR_ENTRY: usize = 4;
    pub const GLOBAL: usize = super::GRID * super::GRID;
    pub const MESHES: usize = 8;
    /// Mesh record: vertex offset u16, vertex count u8, face offset u16, face count u8, shift u8,
    /// bounding radius u8 (texels, about the mesh origin), level of detail: the mesh drawn instead
    /// (255: none) beyond a distance u8 (texels, along the view).
    pub const MESH: usize = 10;
    const _: () = assert!(MESH == 10 && OBJECT == 6 && DIR_ENTRY == 4 && super::Q == 16, "draw scales by shifts");
    /// Object record: x u16, y u16 (8.8 texels), yaw u8, mesh u8.
    pub const OBJECT: usize = 6;
    /// Vertex: x, y (plane axes, u and v), h (up), i8 in quarter texels << shift.
    pub const VERTEX: usize = 3;
    /// Face: three vertex indices, shade (`poly::PATTERNS` index), wound so that the front is clockwise
    /// on screen (y down).
    pub const FACE: usize = 4;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Camera {
    pub x: u16,
    pub y: u16,
    pub angle: u8,
    /// Pop-in size: an object is drawn once its bounding radius would cover at least this many pixels
    /// (0: [`POP`]). Its pop distance is `80·r / pop` texels along the view, capped at [`FAR`]: small
    /// objects appear close, big ones far away.
    pub pop: u8,
}

/// The default pop-in size (pixels of bounding radius): cones (radius 5) appear at ~66 texels.
pub const POP: u8 = 6;

/// Depth (Q4) per texel of bounding radius at which an object shrinks to `pop` pixels: `80·16 / pop`.
pub fn pop_scale(pop: u8) -> i16 {
    let p = if pop == 0 { POP } else { pop };
    (1280 / p as u16) as i16
}

/// The depth (Q4) beyond which an object of bounding radius `r` texels is not drawn.
pub fn pop_depth(r: u8, scale: i16) -> i32 {
    mul(r as i16, scale).min(FAR)
}

/// Per-frame values the cull shares: the camera's cos / sin and [`pop_scale`].
#[derive(Clone, Copy)]
struct Look {
    c: i16,
    s: i16,
    scale: i16,
}

/// A projected triangle, ready to draw.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tri {
    pub x: [i16; 3],
    pub y: [i16; 3],
    pub shade: u8,
}

pub const MAX_TRIS: usize = 255;
pub const MAX_VERTS: usize = 128;
pub const NONE: u8 = 0xFF;

/// One object's transformed vertices, as `asm/geom.S` keeps them (at `layout::VERTS` on the C64): ten
/// byte arrays of [`MAX_VERTS`], lo/hi of camera-space x, y, z and screen x, y.
#[repr(C)]
pub struct Verts {
    a: [[u8; MAX_VERTS]; 10],
}

pub mod vert {
    pub const X: usize = 0;
    pub const Y: usize = 1;
    pub const Z: usize = 2;
    pub const SX: usize = 3;
    pub const SY: usize = 4;
}

impl Verts {
    pub const fn new() -> Self {
        Verts { a: [[0; MAX_VERTS]; 10] }
    }
    #[inline]
    pub fn get(&self, field: usize, i: usize) -> i16 {
        i16::from_le_bytes([self.a[2 * field][i], self.a[2 * field + 1][i]])
    }
    #[inline]
    pub fn set(&mut self, field: usize, i: usize, v: i16) {
        let [lo, hi] = v.to_le_bytes();
        self.a[2 * field][i] = lo;
        self.a[2 * field + 1][i] = hi;
    }
}

impl Default for Verts {
    fn default() -> Self {
        Self::new()
    }
}

/// The frame's triangles (screen x, y of three corners and a shade, as byte arrays of 256) and the depth
/// buckets: `head[key]` and `next[i]` chain triangle indices ([`NONE`] ends a chain). `clip` lists the
/// faces of the current object that straddle the near plane (`asm/geom.S` leaves those to Rust).
/// At `layout::WORK` on the C64.
#[repr(C)]
pub struct Work {
    t: [[u8; 256]; 13],
    pub head: [u8; 256],
    pub next: [u8; 256],
    pub clip: [u8; 256],
    pub n: u8,
    pub dropped: u8,
    pub nclip: u8,
}

/// Byte offsets of the [`Work`] arrays (for `asm/geom.S`).
pub mod work {
    pub const X0: u16 = 0x000;
    pub const Y0: u16 = 0x600;
    pub const SHADE: u16 = 0xC00;
    pub const HEAD: u16 = 0xD00;
    pub const NEXT: u16 = 0xE00;
    pub const CLIP: u16 = 0xF00;
    pub const N: u16 = 0x1000;
    pub const DROPPED: u16 = 0x1001;
    pub const NCLIP: u16 = 0x1002;
}

impl Work {
    pub const fn new() -> Self {
        Work { t: [[0; 256]; 13], head: [NONE; 256], next: [NONE; 256], clip: [0; 256], n: 0, dropped: 0, nclip: 0 }
    }
    pub fn tri(&self, i: u8) -> Tri {
        let i = i as usize;
        let w = |k: usize| i16::from_le_bytes([self.t[k][i], self.t[k + 1][i]]);
        Tri { x: [w(0), w(2), w(4)], y: [w(6), w(8), w(10)], shade: self.t[12][i] }
    }
    fn set_tri(&mut self, i: u8, t: &Tri) {
        let i = i as usize;
        for k in 0..3 {
            let ([xl, xh], [yl, yh]) = (t.x[k].to_le_bytes(), t.y[k].to_le_bytes());
            self.t[2 * k][i] = xl;
            self.t[2 * k + 1][i] = xh;
            self.t[6 + 2 * k][i] = yl;
            self.t[6 + 2 * k + 1][i] = yh;
        }
        self.t[12][i] = t.shade;
    }
}

impl Default for Work {
    fn default() -> Self {
        Self::new()
    }
}

/// What a frame's geometry did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub objects: u8,
    pub tris: u8,
    /// Triangles that did not fit in [`MAX_TRIS`].
    pub dropped: u8,
    /// Topmost cell row drawn (25: none), so the caller can repair the sky above the horizon.
    pub top_row: u8,
    /// Grid cells (and the global list) whose objects were streamed in.
    pub cells: u8,
}

#[inline]
fn rd16(s: &[u8], at: usize) -> u16 {
    s[at] as u16 | (s[at + 1] as u16) << 8
}

fn clamp(v: i32) -> i16 {
    v.clamp(-LIMIT, LIMIT) as i16
}

/// Screen position (Q4 pixels) of a camera-space point with `z >= NEAR`.
///
/// `1/z` comes from the reciprocal table: `z = m·2^e`, `1/z ~ recip(m) / 2^(26+e)`, so
/// `x·1280/z = (x·recip(m) >> 8)·5 / 2^(10+e)` (and 10 for `sy`'s 2560).
pub fn project(x: i16, y: i16, z: i16) -> (i16, i16) {
    let (m, e) = fixed::norm(z as u16);
    let r = fixed::recip(m);
    let s = (10 + e) as u8;
    let qx = mulsu(x, r) >> 8;
    let qy = mulsu((12 * Q) as i16 - y, r) >> 8;
    let sx = 80 * 16 + sar(qx * 5, s);
    let sy = 48 * 16 + sar(qy * 10, s);
    (clamp(sx), clamp(sy))
}

/// Twice the signed screen area; > 0 for a front face (clockwise with y down). Corners are clamped to
/// ±16000, so every difference fits in i16 and the two products' difference in i32.
fn area(x: [i16; 3], y: [i16; 3]) -> i32 {
    mul(x[1] - x[0], y[2] - y[0]) - mul(y[1] - y[0], x[2] - x[0])
}

/// Append a front-facing triangle to depth bucket `key` (`asm/geom.S`'s `push`).
pub fn push(w: &mut Work, x: [i16; 3], y: [i16; 3], shade: u8, key: u8) {
    if area(x, y) <= 0 {
        return;
    }
    if w.n as usize >= MAX_TRIS {
        w.dropped = w.dropped.saturating_add(1);
        return;
    }
    let i = w.n;
    w.set_tri(i, &Tri { x, y, shade });
    w.next[i as usize] = w.head[key as usize];
    w.head[key as usize] = i;
    w.n += 1;
}

/// The point where edge `p`→`q` (`p` in front, `q` behind the near plane) crosses it.
fn cut(p: (i32, i32, i32), q: (i32, i32, i32)) -> (i16, i16, i16) {
    // Camera-space coordinates are i16, so the differences and t fit too: one 16x16 multiply each.
    let (dz, t) = (q.2 - p.2, (NEAR - p.2) as i16);
    ((p.0 + mul((q.0 - p.0) as i16, t) / dz) as i16, (p.1 + mul((q.1 - p.1) as i16, t) / dz) as i16, NEAR as i16)
}

/// One object in view, ready for `asm/geom.S` (`geo_args`): its mesh's vertices and faces (offsets in
/// the scene block), the vertex shift `t` (vertices are i8 << t in Q4), its centre in camera space and
/// the rotation from mesh to camera (Q14).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Obj {
    pub verts: u16,
    pub nv: u8,
    pub faces: u16,
    pub nf: u8,
    pub t: u8,
    pub x0: i16,
    pub z0: i16,
    pub cc: i16,
    pub ss: i16,
}

/// `asm/geom.S`'s argument block (`layout::GEO_ARGS`).
pub mod geo_args {
    /// u16: absolute address of the vertices (C64 RAM).
    pub const VPTR: u16 = 0;
    pub const NV: u16 = 2;
    /// u16: absolute address of the faces.
    pub const FPTR: u16 = 3;
    pub const NF: u16 = 5;
    /// u8: right shift of the rotated products (14 - t).
    pub const SH: u16 = 6;
    /// u8: t, the height's left shift.
    pub const T: u16 = 7;
    pub const X0: u16 = 8;
    pub const Z0: u16 = 10;
    pub const CC: u16 = 12;
    pub const SS: u16 = 14;
    /// u16: bitmap of the raster pass.
    pub const BITMAP: u16 = 16;
    /// out u8: triangles drawn, topmost cell row.
    pub const TRIS: u16 = 18;
    pub const TOP: u16 = 19;
    pub const SIZE: u16 = 20;
}

/// Transform and project one object's vertices (`asm/geom.S` `geo_xform`). With `v` = i8 << t:
/// `x = x0 + (vx·cc + vy·ss) >> (14 - t)`, `z = z0 + (vx·ss - vy·cc) >> (14 - t)`, `y = vh << t`.
pub fn xform(scene: &[u8], o: &Obj, v: &mut Verts) {
    let sh = 14 - o.t;
    for i in 0..o.nv as usize {
        let p = o.verts as usize + i * scene::VERTEX;
        let (a, b, h) = (scene[p] as i8 as i32, scene[p + 1] as i8 as i32, scene[p + 2] as i8 as i16);
        let x = (o.x0 as i32 + ((a * o.cc as i32 + b * o.ss as i32) >> sh)) as i16;
        let z = (o.z0 as i32 + ((a * o.ss as i32 - b * o.cc as i32) >> sh)) as i16;
        let y = h << o.t;
        v.set(vert::X, i, x);
        v.set(vert::Y, i, y);
        v.set(vert::Z, i, z);
        if z as i32 >= NEAR {
            let (sx, sy) = project(x, y, z);
            v.set(vert::SX, i, sx);
            v.set(vert::SY, i, sy);
        }
    }
}

/// A face's depth bucket: the sum of its corners' z, / 32, clamped to 0..255.
fn key(v: &Verts, idx: [usize; 3]) -> u8 {
    let z = |k: usize| v.get(vert::Z, idx[k]) as i32;
    ((z(0) + z(1) + z(2)) >> 5).clamp(0, 255) as u8
}

/// One object's faces (`asm/geom.S` `geo_faces`): faces wholly in front of the near plane are culled and
/// pushed; faces straddling it are listed in `w.clip` for [`clip_faces`].
pub fn faces(scene: &[u8], o: &Obj, v: &Verts, w: &mut Work) {
    w.nclip = 0;
    for f in 0..o.nf {
        let at = o.faces as usize + f as usize * scene::FACE;
        let idx = [scene[at] as usize, scene[at + 1] as usize, scene[at + 2] as usize];
        let front = idx.iter().filter(|&&i| v.get(vert::Z, i) as i32 >= NEAR).count();
        if front == 3 {
            let x = idx.map(|i| v.get(vert::SX, i));
            let y = idx.map(|i| v.get(vert::SY, i));
            push(w, x, y, scene[at + 3], key(v, idx));
        } else if front > 0 {
            w.clip[w.nclip as usize] = f;
            w.nclip += 1;
        }
    }
}

/// The faces `faces` left in `w.clip`: Sutherland-Hodgman against the near plane (3 or 4 points, a fan).
fn clip_faces(scene: &[u8], o: &Obj, v: &Verts, w: &mut Work) {
    for c in 0..w.nclip as usize {
        let at = o.faces as usize + w.clip[c] as usize * scene::FACE;
        let idx = [scene[at] as usize, scene[at + 1] as usize, scene[at + 2] as usize];
        let p = |k: usize| (v.get(vert::X, idx[k]) as i32, v.get(vert::Y, idx[k]) as i32, v.get(vert::Z, idx[k]) as i32);
        let mut out = [(0i16, 0i16); 4];
        let mut n = 0;
        for k in 0..3 {
            let (a, b) = (p(k), p((k + 1) % 3));
            if a.2 >= NEAR {
                out[n] = project(a.0 as i16, a.1 as i16, a.2 as i16);
                n += 1;
            }
            if (a.2 >= NEAR) != (b.2 >= NEAR) {
                let q = if a.2 >= NEAR { cut(a, b) } else { cut(b, a) };
                out[n] = project(q.0, q.1, q.2);
                n += 1;
            }
        }
        let k = key(v, idx);
        for t in 1..n - 1 {
            push(w, [out[0].0, out[t].0, out[t + 1].0], [out[0].1, out[t].1, out[t + 1].1], scene[at + 3], k);
        }
    }
}

/// Draw the buckets back to front, whole triangles at a time: triangles drawn and the topmost cell row
/// (the per-pixel equivalent of [`raster_bands`] while nothing is dropped).
pub fn raster<H: Hw + ?Sized>(hw: &mut H, w: &Work, bitmap: u16) -> (u8, u8) {
    let (mut tris, mut top) = (0u8, 25u8);
    for key in (0..256).rev() {
        let mut i = w.head[key];
        while i != NONE {
            if let Some(t) = setup(bitmap, &w.tri(i)) {
                walk(hw, &t);
                tris += 1;
                top = top.min(t.first >> 3);
            }
            i = w.next[i as usize];
        }
    }
    (tris, top)
}

/// The cell (column, row) of a position (8.8 texels).
pub fn cell_of(x: u16, y: u16) -> (u8, u8) {
    ((x >> 12) as u8, (y >> 12) as u8)
}

/// Cull one object (a `scene::OBJECT` record) and, if it may be in view, transform, sort and clip it.
fn object<H: Hw + ?Sized>(hw: &mut H, scene: &[u8], rec: &[u8; scene::OBJECT], cam: Camera, look: Look, w: &mut Work, v: &mut Verts, st: &mut Stats) {
    let (c, s) = (look.c, look.s);
    let (ox, oy, yaw, mesh) = (u16::from_le_bytes([rec[0], rec[1]]), u16::from_le_bytes([rec[2], rec[3]]), rec[4], rec[5] as usize);
    // Offsets mod 256 texels, in Q4.
    let dx = ox.wrapping_sub(cam.x) as i16 >> 4;
    let dy = oy.wrapping_sub(cam.y) as i16 >> 4;
    if (dx as i32).abs() > SPAN || (dy as i32).abs() > SPAN {
        return;
    }
    let m = scene::MESHES + (mesh << 3) + (mesh << 1);
    let r = (scene[m + 7] as i32) << 4;
    let x0 = (mul(dx, c) + mul(dy, s)) >> 14;
    let z0 = (mul(dx, s) - mul(dy, c)) >> 14;
    if z0 + r < NEAR || z0 - r > pop_depth(scene[m + 7], look.scale) || x0.abs() - r > z0 + r {
        return;
    }
    st.objects += 1;
    // Level of detail: a simpler mesh far away.
    let (lod, lod_z) = (scene[m + 8], (scene[m + 9] as i32) << 4);
    let m = if lod != 0xFF && z0 > lod_z { scene::MESHES + ((lod as usize) << 3) + ((lod as usize) << 1) } else { m };
    let d = cam.angle.wrapping_sub(yaw);
    let ob = Obj {
        verts: rd16(scene, m),
        nv: scene[m + 2].min(MAX_VERTS as u8),
        faces: rd16(scene, m + 3),
        nf: scene[m + 5],
        t: 2 + scene[m + 6],
        x0: x0 as i16,
        z0: z0 as i16,
        cc: cos(d),
        ss: sin(d),
    };
    hw.xform(scene, &ob, v);
    hw.faces(scene, &ob, v, w);
    clip_faces(scene, &ob, v, w);
}

/// Stream one directory entry's objects from the REU into `CELL_RAM` and cull / draw each. `near`: the
/// nearest depth (texels) a listed cell's objects can have; the cell is skipped unfetched when even its
/// largest object would pop in nearer than that.
fn cell<H: Hw + ?Sized>(hw: &mut H, scene: &[u8], entry: usize, near: Option<u8>, cam: Camera, look: Look, w: &mut Work, v: &mut Verts, st: &mut Stats) {
    // (Records are 6 bytes: x6 as shifts, which the 6502 build would otherwise call a multiply for.)
    let at = rd16(scene, scene::DIR) as usize + (entry << 2);
    let n = scene[at + 2].min(CELL_MAX as u8) as u16;
    if n == 0 {
        return;
    }
    if let Some(near) = near {
        let rmax = scene[at + 3];
        if (near as i32) << 4 > pop_depth(rmax, look.scale) + ((rmax as i32) << 4) {
            return;
        }
    }
    let first = rd16(scene, at);
    let six = |v: u16| (v << 2) + (v << 1);
    let offset = ((first as u32) << 2) + ((first as u32) << 1);
    crate::hw::fetch(hw, crate::format::OBJECTS + offset, CELL_RAM, six(n));
    st.cells += 1;
    let mut p = CELL_RAM;
    for _ in 0..n {
        let mut rec = [0u8; scene::OBJECT];
        for (k, b) in rec.iter_mut().enumerate() {
            *b = hw.peek(p + k as u16);
        }
        object(hw, scene, &rec, cam, look, w, v, st);
        p += scene::OBJECT as u16;
    }
}

/// Cull, transform, clip and sort the objects that may be in view into `w`'s depth buckets (drawn by
/// [`raster_bands`] on the host, `asm/geom.S` band by band on the C64). `bitmap` is unused.
/// The candidates: the global list, then the cells of this angle's visibility list (`format::vis_list`),
/// each streamed from the REU.
#[inline(never)]
pub fn draw<H: Hw + ?Sized>(hw: &mut H, scene: &[u8], cam: Camera, bitmap: u16, w: &mut Work, v: &mut Verts) -> Stats {
    let mut st = Stats::default();
    w.n = 0;
    w.dropped = 0;
    w.head.fill(NONE);
    let look = Look { c: cos(cam.angle), s: sin(cam.angle), scale: pop_scale(cam.pop) };
    cell(hw, scene, scene::GLOBAL, None, cam, look, w, v, &mut st);
    crate::hw::fetch(hw, crate::format::vis_list(cam.angle), VIS_RAM, crate::format::VIS_STRIDE as u16);
    let (cx, cy) = cell_of(cam.x, cam.y);
    let n = hw.peek(VIS_RAM);
    // Entries run far to near, each with the nearest depth its cell's objects can have.
    for k in 0..n as u16 {
        let near = hw.peek(VIS_RAM + VIS_DEPTH + k);
        let off = hw.peek(VIS_RAM + 1 + k);
        let (dx, dy) = (((off << 4) as i8) >> 4, (off as i8) >> 4);
        let col = (cx as i8).wrapping_add(dx) as u8 & (GRID as u8 - 1);
        let row = (cy as i8).wrapping_add(dy) as u8 & (GRID as u8 - 1);
        cell(hw, scene, row as usize * GRID + col as usize, Some(near), cam, look, w, v, &mut st);
    }
    let _ = bitmap;
    st.dropped = w.dropped;
    st
}

/// Offset of the per-entry depths in a visibility list (`format::vis_list`): for entry k, the nearest
/// camera-space depth (texels, floored, 0 if behind) of any object centre its cell can hold.
pub const VIS_DEPTH: u16 = 128;

/// The cells (offsets from the camera's cell, within [`REACH`]) that can hold an object the per-object
/// cull in [`draw`] accepts when the camera looks along `angle`, far to near, each with its nearest depth
/// (host bake). An object of
/// horizontal radius r <= [`CELL_PAD`] is accepted only if its centre is in `NEAR - r <= z <= FAR + r`,
/// `|x| <= z + 2r`; its centre is anywhere in its cell and the camera anywhere in its own, so a cell
/// offset (i, j) covers centres at offsets in `(16i - 16, 16i + 16) × (16j - 16, 16j + 16)` texels. Those
/// squares are tested against the acceptance region with r = `CELL_PAD` by separating axes.
#[cfg(feature = "host")]
pub fn visible_cells(angle: u8) -> Vec<(i8, i8, u8)> {
    let a = angle as f64 / 256.0 * core::f64::consts::TAU;
    let (s, c) = a.sin_cos();
    let to_cam = |p: (f64, f64)| (p.0 * c + p.1 * s, p.0 * s - p.1 * c);
    let (pad, q) = (CELL_PAD as f64, Q as f64);
    let (zmin, zmax) = (NEAR as f64 / q - pad, FAR as f64 / q + pad);
    // The acceptance region (camera space x, z), widened by half a texel for fixed-point rounding.
    let e = 0.5;
    let region = [(zmin + 2.0 * pad + e, zmin - e), (zmax + 2.0 * pad + e, zmax + e), (-(zmax + 2.0 * pad + e), zmax + e), (-(zmin + 2.0 * pad + e), zmin - e)];
    let separated = |pa: &[(f64, f64)], pb: &[(f64, f64)]| {
        for poly in [pa, pb] {
            for k in 0..poly.len() {
                let (p, q) = (poly[k], poly[(k + 1) % poly.len()]);
                let axis = (q.1 - p.1, p.0 - q.0);
                let proj = |pts: &[(f64, f64)]| {
                    pts.iter().map(|t| t.0 * axis.0 + t.1 * axis.1).fold((f64::MAX, f64::MIN), |(lo, hi), d| (lo.min(d), hi.max(d)))
                };
                let ((a0, a1), (b0, b1)) = (proj(pa), proj(pb));
                if a1 < b0 || b1 < a0 {
                    return true;
                }
            }
        }
        false
    };
    let mut out = Vec::new();
    for j in -REACH..=REACH {
        for i in -REACH..=REACH {
            let (x0, x1, y0, y1) = ((i * CELL - CELL) as f64, (i * CELL + CELL) as f64, (j * CELL - CELL) as f64, (j * CELL + CELL) as f64);
            let square = [to_cam((x0, y0)), to_cam((x1, y0)), to_cam((x1, y1)), to_cam((x0, y1))];
            if !separated(&square, &region) {
                let centre = to_cam(((i * CELL) as f64, (j * CELL) as f64));
                let near = square.iter().map(|p| p.1).fold(f64::MAX, f64::min) - e;
                out.push((i as i8, j as i8, centre.1, near.floor().clamp(0.0, 255.0) as u8));
            }
        }
    }
    out.sort_by(|a, b| b.2.total_cmp(&a.2));
    out.into_iter().map(|(i, j, _, near)| (i, j, near)).collect()
}

/// An edge's x (Q16 pixels) on successive line centres.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Edge {
    pub x: i32,
    pub step: i32,
}

impl Edge {
    /// The edge from `(xa, ya)` to `(xb, yb)` (Q4, `ya < yb`) at the centre of `line`.
    fn new(xa: i32, ya: i32, xb: i32, yb: i32, line: i32) -> Edge {
        // Q16 pixels per Q4 line unit: (xb - xa)·2^12 / (yb - ya), the divide by the reciprocal table
        // (`fixed::norm`: 11 significant bits of the height, so x drifts at most |dx|/2048 pixels).
        let (m, e) = fixed::norm((yb - ya) as u16);
        let per = sar(mulsu((xb - xa) as i16, fixed::recip(m)), (14 + e) as u8);
        Edge { x: (xa << 12).wrapping_add(mul_wide((line * 16 + 8 - ya) as i16, per)), step: per << 4 }
    }
}

/// A set-up triangle: lines `first..end`, bounded by `long` and `short`; at line `mid` `short` becomes
/// `lower`. The C64 passes it in memory at `layout::TRI_ARGS` in this order (i32s little-endian), with
/// the pattern already in `POLY_ARGS` (`asm/tri.S`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Walk {
    pub long: Edge,
    pub short: Edge,
    pub lower: Edge,
    pub first: u8,
    pub mid: u8,
    pub end: u8,
    pub pat: [u8; 2],
    pub bitmap: u16,
}

pub mod walk_args {
    pub const LONG: u16 = 0;
    pub const SHORT: u16 = 8;
    pub const LOWER: u16 = 16;
    pub const FIRST: u16 = 24;
    pub const MID: u16 = 25;
    pub const END: u16 = 26;
    pub const BITMAP: u16 = 27;
    pub const SIZE: u16 = 29;
}

/// First line whose centre is at or below `y` (Q4).
fn line_at(y: i32) -> i32 {
    (y - 8 + 15) >> 4
}

/// The pixel a Q16 left/right bound starts at (its centre is at or right of the bound), clamped.
pub fn pixel(x: i32) -> u8 {
    ((x + 0x7FFF) >> 16).clamp(0, 160) as u8
}

/// Set up one triangle (the C64 does this part in Rust): sort by y, clip the lines to the screen, and
/// the three edges at their first lines. `None` when it covers no line centre.
pub fn setup(bitmap: u16, t: &Tri) -> Option<Walk> {
    let mut v = [(t.x[0] as i32, t.y[0] as i32), (t.x[1] as i32, t.y[1] as i32), (t.x[2] as i32, t.y[2] as i32)];
    if v[0].1 > v[1].1 {
        v.swap(0, 1);
    }
    if v[1].1 > v[2].1 {
        v.swap(1, 2);
    }
    if v[0].1 > v[1].1 {
        v.swap(0, 1);
    }
    let first = line_at(v[0].1).max(0);
    let end = line_at(v[2].1).min(200);
    if first >= end {
        return None;
    }
    let mid = line_at(v[1].1).clamp(first, end);
    let long = Edge::new(v[0].0, v[0].1, v[2].0, v[2].1, first);
    let short = if first < mid { Edge::new(v[0].0, v[0].1, v[1].0, v[1].1, first) } else { Edge::default() };
    let lower = if mid < end { Edge::new(v[1].0, v[1].1, v[2].0, v[2].1, mid) } else { Edge::default() };
    Some(Walk {
        long,
        short,
        lower,
        first: first as u8,
        mid: mid as u8,
        end: end as u8,
        pat: PATTERNS[(t.shade & 15) as usize],
        bitmap,
    })
}

/// Walk a set-up triangle's lines, one cell row at a time into [`Hw::poly_row`]. The reference for
/// `asm/tri.S`.
pub fn walk<H: Hw + ?Sized>(hw: &mut H, w: &Walk) {
    let mut w = *w;
    for r in w.first >> 3..=(w.end - 1) >> 3 {
        walk_row(hw, &mut w, r);
    }
}

/// One cell row `r` of a set-up triangle (`asm/tri.S` `tri_row`): its lines in `first..end`, the edges
/// advanced in `w` (`short` becomes `lower` at line `mid`), into `w.bitmap + r * 320`.
pub fn walk_row<H: Hw + ?Sized>(hw: &mut H, w: &mut Walk, r: u8) {
    let mut row = Row { pat: w.pat, base: w.bitmap.wrapping_add(r as u16 * ROW_STRIDE), ..Row::default() };
    for k in 0..8u8 {
        let line = r * 8 + k;
        if line < w.first || line >= w.end {
            continue;
        }
        if line == w.mid {
            w.short = w.lower;
        }
        let (a, b) = (pixel(w.long.x), pixel(w.short.x));
        row.xl[k as usize] = a.min(b);
        row.xr[k as usize] = a.max(b);
        w.long.x = w.long.x.wrapping_add(w.long.step);
        w.short.x = w.short.x.wrapping_add(w.short.step);
    }
    hw.poly_row(&row);
}

/// Active triangles a band can carry (`asm/geom.S`'s slot pool).
pub const POOL: usize = 96;

/// The triangles in painter's order (buckets far to near, each chain in order), and each one's first
/// band (`None` if it starts below the screen): `asm/geom.S` `geo_bin`.
pub fn bin(w: &Work) -> ([u8; 256], u16, [[u8; 256]; BANDS], [u8; BANDS]) {
    let (mut order, mut n) = ([0u8; 256], 0u16);
    let (mut bins, mut lens) = ([[0u8; 256]; BANDS], [0u8; BANDS]);
    for key in (0..256).rev() {
        let mut i = w.head[key];
        while i != NONE {
            let t = w.tri(i);
            let top = t.y[0].min(t.y[1]).min(t.y[2]) as i32;
            let first = line_at(top).max(0);
            if first < 200 {
                let b = (first >> 3) as usize;
                bins[b][lens[b] as usize] = n as u8;
                lens[b] += 1;
            }
            order[n as usize] = i;
            n += 1;
            i = w.next[i as usize];
        }
    }
    (order, n, bins, lens)
}

/// Cell rows (bands) of the screen.
pub const BANDS: usize = 25;

/// Draw the sorted triangles band by band, as the C64 publishes them (the reference for `asm/geom.S`
/// `geo_bin` / `geo_activate` / `geo_draw`): a triangle is set up when its first band comes, joins the
/// active list in painter's order (at most [`POOL`]; the rest are dropped), is walked one row per band
/// and leaves after its last. Per pixel this is the whole-frame painter's order. Returns (triangles,
/// topmost band drawn, dropped).
pub fn raster_bands<H: Hw + ?Sized>(hw: &mut H, w: &Work, bitmap: u16) -> (u8, u8, u8) {
    let (order, _, bins, lens) = bin(w);
    let mut active: [(u8, Walk); POOL] = [(0, Walk::default()); POOL];
    let mut n = 0usize;
    let (mut tris, mut top, mut dropped) = (0u8, 25u8, 0u8);
    for band in 0..BANDS {
        // Activate this band's newcomers (in painter's order), then merge them into the active list.
        let mut new: [(u8, Walk); POOL] = [(0, Walk::default()); POOL];
        let mut m = 0usize;
        for &seq in &bins[band][..lens[band] as usize] {
            let Some(walk) = setup(bitmap, &w.tri(order[seq as usize])) else { continue };
            if n + m >= POOL {
                dropped = dropped.saturating_add(1);
                continue;
            }
            new[m] = (seq, walk);
            m += 1;
            tris += 1;
        }
        let mut merged: [(u8, Walk); POOL] = [(0, Walk::default()); POOL];
        let (mut i, mut j) = (0, 0);
        for slot in merged.iter_mut().take(n + m) {
            *slot = if j >= m || (i < n && active[i].0 < new[j].0) {
                i += 1;
                active[i - 1]
            } else {
                j += 1;
                new[j - 1]
            };
        }
        active = merged;
        n += m;
        if n > 0 {
            top = top.min(band as u8);
        }
        let mut kept = 0;
        for k in 0..n {
            let (seq, mut walk) = active[k];
            walk_row(hw, &mut walk, band as u8);
            if walk.end as usize > band * 8 + 8 {
                active[kept] = (seq, walk);
                kept += 1;
            }
        }
        n = kept;
    }
    (tris, top, dropped)
}

/// Draw one triangle; returns its top cell row, or `None` when it covers no line centre.
#[inline(never)]
pub fn triangle<H: Hw + ?Sized>(hw: &mut H, bitmap: u16, t: &Tri) -> Option<u8> {
    let w = setup(bitmap, t)?;
    walk(hw, &w);
    Some(w.first >> 3)
}
