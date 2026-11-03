//! Flat-shaded 3D over the floor (`no_std`, shared by the host and the C64, so both compute the same
//! bits): per-object rotation into the floor camera's space (any yaw, pitch and roll), near-plane clip,
//! projection matching the floor exactly, backface cull, a one-pass bucket depth sort (painter's), and
//! the triangle set-up that feeds [`Hw::tri_walk`] (`asm/tri.S` on the C64), which feeds
//! [`Hw::poly_row`] (`asm/poly.S`). Pre-rendered sprites ride the same depth buckets ([`push_sprite`]).
//!
//! Units: camera space in 1/16 texel ("Q4"), screen in 1/16 multicolour pixel; pixel `p` spans
//! `[p, p+1)` and is covered when its centre is (the floor's sampling too). All products go through
//! [`crate::fixed`] (a 16×16 multiply; division by reciprocal table).
//!
//! World axes: x = u (east), y = up, z = v (north), as the floor's. Camera space: X right, Y up, Z along
//! the view (`floor::FloorCam::r`'s columns). Projection (`floor::FX`, `floor::FY`, the axis at line
//! 67.5's centre, i.e. 68 in `[y, y+1)` line units): `sx = 80 + 80·X/Z`, `sy = 68 − 133·Y/Z`.

use crate::fixed::{self, mul, mul_wide, mulsu, sar};
use crate::floor::FloorCam;
use crate::hw::Hw;
use crate::layout::{CELL_MAX, CELL_RAM, DIR_RAM, ROW_STRIDE};
use crate::poly::{PATTERNS, Row};

/// Camera-space units per texel.
pub const Q: i32 = 16;
/// Near plane: 2 texels. Faces crossing it are left out (no clipping), so it is close.
pub const NEAR: i32 = 2 * Q;
/// Objects whose centre is farther than this (plus their radius) along the view are skipped: the floor
/// is haze beyond it (`floor::HAZE_TEXELS`).
pub const FAR: i32 = crate::floor::HAZE_TEXELS * Q;
/// The object grid: [`GRID`]² cells of [`CELL`] texels over the map. An object is filed under the cell
/// of its centre; objects reaching more than [`CELL_PAD`] texels from it go in the big grid instead:
/// [`BIG_GRID`]² cells of [`BIG_CELL`] texels.
pub const CELL: i32 = 64;
pub const GRID: usize = 32;
pub const CELL_PAD: i32 = 32;
pub const BIG_CELL: i32 = 256;
pub const BIG_GRID: usize = 8;
/// Object records hold offsets from their cell's centre in quarter texels (the grid: `<< SHIFT` is Q4)
/// and whole texels (the big grid).
pub const SHIFT: u8 = 2;
pub const BIG_SHIFT: u8 = 4;
/// Cells either way from the camera's cell that can hold an object the cull accepts: within [`FAR`]
/// plus its radius along the view and 45° to either side, so up to about √2 of that away.
pub const REACH: i32 = 8;
pub const BIG_REACH: i32 = 3;
const _: () = assert!(CELL - 1 + (FAR / Q + CELL_PAD) * 17 / 12 < (REACH + 1) * CELL);
const _: () = assert!(BIG_CELL - 1 + (FAR / Q + 160) * 17 / 12 < (BIG_REACH + 1) * BIG_CELL);
/// Screen coordinates are clamped to ±1000 pixels (Q4), which keeps every edge product in i32.
pub const LIMIT: i32 = 1000 * 16;
/// Screen lines the band pipeline draws (the view above the cockpit) and its bands.
pub const LINES: i32 = crate::floor::VIEW_LINES as i32;
pub const BANDS: usize = crate::floor::VIEW_BANDS as usize;

/// Scene block (`format::SCENE`, fetched whole to `layout::SCENE_RAM`): `[0]` length u16, `[2]` meshes
/// u8, mesh table at [`scene::MESHES`]. The object directory and records live in the REU
/// (`format::OBJ_DIR`, `format::OBJ_RECORDS`).
pub mod scene {
    pub const LEN: usize = 0;
    pub const N_MESHES: usize = 2;
    pub const MESHES: usize = 8;
    /// Mesh record: vertex offset u16, vertex count u8, face offset u16, face count u8, shift u8,
    /// bounding radius u8 (texels, about the mesh origin), level of detail: the mesh drawn instead
    /// (255: none) beyond a distance u8 (texels, along the view).
    pub const MESH: usize = 10;
    const _: () = assert!(MESH == 10 && OBJECT == 8 && DIR_ENTRY == 4 && super::Q == 16, "draw scales by shifts");
    /// Directory entry (one per grid cell, then one per big-grid cell): the first object (u16, an index
    /// into the records), the count (u8) and the largest bounding radius among them (u8, texels).
    pub const DIR_ENTRY: usize = 4;
    /// Object record: the offset from its cell's centre (east, north, up: i8 each, in quarter texels in
    /// the grid, whole texels in the big grid; see [`super::object_centre`]), yaw u8 (256 a turn, quarter
    /// turns only: the top two bits), mesh u8, 3 spare.
    pub const OBJECT: usize = 8;
    pub const O_DU: usize = 0;
    pub const O_DV: usize = 1;
    pub const O_DY: usize = 2;
    pub const O_YAW: usize = 3;
    pub const O_MESH: usize = 4;
    /// Vertex: u, v (east, north) and h (up), i8 in quarter texels << shift.
    pub const VERTEX: usize = 3;
    /// Face: three vertex indices, shade (`poly::PATTERNS` index), wound so that the front is clockwise
    /// on screen (y down).
    pub const FACE: usize = 4;
}

/// The camera the geometry sees: the floor's ([`FloorCam`]) and the pop-in size.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct View {
    pub cam: FloorCam,
    /// Pop-in size: an object is drawn once its bounding radius would cover at least this many pixels
    /// (0: [`POP`]). Its pop distance is `80·r / pop` texels along the view, capped at [`FAR`]: small
    /// objects appear close, big ones far away.
    pub pop: u8,
}

/// Sprite items in [`Work`] have this shade bit set: x0 = left pixel, x1/x2 = REU block (lo 16, then
/// bank | band shift << 8), y0 = top line, y1 = height, y2 = byte columns.
pub const SPRITE: u8 = 0x80;

/// The default pop-in size (pixels of bounding radius): trees (radius 4) appear at 80 texels.
pub const POP: u8 = 4;

/// Depth (Q4) per texel of bounding radius at which an object shrinks to `pop` pixels: `80·16 / pop`.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.geomsetup"))]
pub fn pop_scale(pop: u8) -> i16 {
    let p = if pop == 0 { POP } else { pop };
    (1280 / p as u16) as i16
}

/// The depth (Q4) beyond which an object of bounding radius `r` texels is not drawn.
pub fn pop_depth(r: u8, scale: i16) -> i32 {
    mul(r as i16, scale).min(FAR)
}

/// Per-frame values the cull shares: world → camera (`Rᵀ`, Q14), the camera in Q4 texels, and
/// [`pop_scale`].
#[derive(Clone, Copy, Debug)]
pub struct Look {
    pub rt: [[i16; 3]; 3],
    /// Camera position: u, v in Q8 texels (as `FloorCam`), height in Q4.
    pub u: i32,
    pub v: i32,
    pub y: i16,
    pub scale: i16,
}

impl Look {
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.geomsetup"))]
    pub fn new(view: &View) -> Self {
        let r = &view.cam.r;
        Look {
            rt: core::array::from_fn(|j| core::array::from_fn(|k| r[k][j] as i16)),
            u: view.cam.u,
            v: view.cam.v,
            y: (view.cam.h >> 4) as i16,
            scale: pop_scale(view.pop),
        }
    }

    /// A world offset (Q4 texels, x/y/z = east/up/north) in camera space (Q4).
    pub fn to_camera(&self, d: [i16; 3]) -> [i32; 3] {
        self.sums(d).map(|x| x >> 14)
    }

    /// [`Look::to_camera`] before its shift: `Rᵀ·d`, Q18. Exact, so sums of these add up exactly.
    pub fn sums(&self, d: [i16; 3]) -> [i32; 3] {
        core::array::from_fn(|j| mul(self.rt[j][0], d[0]) + mul(self.rt[j][1], d[1]) + mul(self.rt[j][2], d[2]))
    }

    /// [`Look::sums`] of a cell centre (`cu`, `cv`: Q4 texels on the map) at ground level.
    pub fn cell_sums(&self, cu: i32, cv: i32) -> [i32; 3] {
        self.sums([(cu - (self.u >> 4)) as i16, self.y.wrapping_neg(), (cv - (self.v >> 4)) as i16])
    }
}

/// An object's centre in camera space (Q4) from its cell's [`Look::cell_sums`] `s` and its record: the
/// offset `(du, dy, dv) << sh` (Q4) rotated and added before the shift (`asm/geom.S` `orow`).
pub fn object_centre(look: &Look, s: [i32; 3], sh: u8, rec: &[u8]) -> [i32; 3] {
    let d = [scene::O_DU, scene::O_DY, scene::O_DV].map(|k| (rec[k] as i8 as i16) << sh);
    let t = look.sums(d);
    core::array::from_fn(|j| (s[j] + t[j]) >> 14)
}

/// Whether a sphere of radius `r` (Q4) round camera-space point `c` may be in view: in front of the near
/// plane, within `far`, inside the side planes (`|X| ≤ Z`, 45° each way: `r·√2 ≤ 3r/2`) and the top and
/// bottom planes (`|Y|·133 ≤ Z·68`: `r·150` bounds `r·√(133² + 68²)`).
#[inline(never)]
pub fn sphere_visible(c: [i32; 3], r: i32, far: i32) -> bool {
    let (x, y, z) = (c[0], c[1], c[2]);
    z + r >= NEAR && z - r <= far && x.abs() - z <= r + (r >> 1) && y.abs() * 133 - z * 68 <= r * 150
}

/// A projected triangle, ready to draw.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tri {
    pub x: [i16; 3],
    pub y: [i16; 3],
    pub shade: u8,
}

pub const MAX_TRIS: usize = 255;
pub const MAX_VERTS: usize = 64;
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
    /// Empty the buckets for a new frame.
    pub fn clear(&mut self) {
        self.n = 0;
        self.dropped = 0;
        self.head.fill(NONE);
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
    /// Grid cells whose objects were streamed in.
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
/// `1/z` comes from the reciprocal table: `z = m·2^e`, `1/z ~ recip(m) / 2^(26+e)`, so with the scaled
/// reciprocals `rx = recip(m)·5 >> 3` and `ry = recip(m)·133 >> 8` (both u16, once per point),
/// `x·1280/z = x·rx / 2^(15+e)` and `−y·2128/z = −y·ry / 2^(14+e)`.
pub fn project(x: i16, y: i16, z: i16) -> (i16, i16) {
    let (m, e) = fixed::norm(z as u16);
    let r = fixed::recip(m) as u32;
    let (rx, ry) = ((r * 5 >> 3) as u16, (r * 133 >> 8) as u16);
    let sx = 80 * 16 + sar(mulsu(x, rx), (15 + e) as u8);
    let sy = 68 * 16 + sar(mulsu(y.wrapping_neg(), ry), (14 + e) as u8);
    (clamp(sx), clamp(sy))
}

/// Twice the signed screen area; > 0 for a front face (clockwise with y down). Corners are clamped to
/// ±16000, so every difference fits in i16 and the two products' difference in i32.
fn area(x: [i16; 3], y: [i16; 3]) -> i32 {
    mul(x[1] - x[0], y[2] - y[0]) - mul(y[1] - y[0], x[2] - x[0])
}

/// A depth bucket from a camera-space depth sum `s` (three corners, Q4): 2/3-texel buckets out to about
/// 128 texels, then 16 times coarser to beyond the haze; nearer is a lower key.
pub fn depth_key(s: i32) -> u8 {
    if s < 6144 { (s >> 5).max(0) as u8 } else { (192 + ((s - 6144) >> 8)).min(255) as u8 }
}

/// Append a sprite to depth bucket `key` (an item the band pipeline draws in painter's order).
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.dyn"))]
pub fn push_sprite(w: &mut Work, left: i16, top: i16, h: u16, wb: u8, block: u32, shift: u8, key: u8) {
    if w.n as usize >= MAX_TRIS {
        w.dropped = w.dropped.saturating_add(1);
        return;
    }
    let i = w.n;
    let x = [left, block as u16 as i16, ((block >> 16) as u16 | (shift as u16) << 8) as i16];
    w.set_tri(i, &Tri { x, y: [top, h as i16, wb as i16], shade: SPRITE });
    w.next[i as usize] = w.head[key as usize];
    w.head[key as usize] = i;
    w.n += 1;
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

/// One object in view, ready for `asm/geom.S` (`geo_args`): its mesh's vertices and faces (offsets in
/// the scene block), the vertex shift `t` (vertices are i8 << t in Q4), its origin in camera space and
/// the rotation from mesh (u, up, v) to camera (Q14, rows X, Y, Z).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Obj {
    pub verts: u16,
    pub nv: u8,
    pub faces: u16,
    pub nf: u8,
    pub t: u8,
    pub org: [i16; 3],
    pub m: [[i16; 3]; 3],
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
    /// u8: t.
    pub const T: u16 = 7;
    /// i16 × 3: the origin in camera space.
    pub const ORG: u16 = 8;
    /// i16 × 9: the rotation, row-major (X row: u, up, v; then Y, Z).
    pub const M: u16 = 14;
    /// u16: bitmap of the raster pass.
    pub const BITMAP: u16 = 32;
    /// out u8: triangles drawn, topmost cell row.
    pub const TRIS: u16 = 34;
    pub const TOP: u16 = 35;
    pub const SIZE: u16 = 36;
}

/// Transform and project one object's vertices (`asm/geom.S` `geo_xform`). With mesh vertex `(a, b, h)`
/// (u, v, up; i8 << t in Q4) and the rotation pre-shifted to `m' = sign(m)·(|m| >> (6 − t))` once per
/// object: `P_j = org_j + (m'[j][0]·a + m'[j][1]·h + m'[j][2]·b) >> 8`.
pub fn xform(scene: &[u8], o: &Obj, v: &mut Verts) {
    let s = 6 - o.t;
    let m: [[i16; 3]; 3] = o.m.map(|row| row.map(|x| if x < 0 { -((x.unsigned_abs() >> s) as i16) } else { x >> s }));
    for i in 0..o.nv as usize {
        let p = o.verts as usize + i * scene::VERTEX;
        let (a, b, h) = (scene[p] as i8 as i16, scene[p + 1] as i8 as i16, scene[p + 2] as i8 as i16);
        let c: [i16; 3] = core::array::from_fn(|j| {
            let s = mul(m[j][0], a) + mul(m[j][1], h) + mul(m[j][2], b);
            o.org[j].wrapping_add((s >> 8) as i16)
        });
        v.set(vert::X, i, c[0]);
        v.set(vert::Y, i, c[1]);
        v.set(vert::Z, i, c[2]);
        if c[2] as i32 >= NEAR {
            let (sx, sy) = project(c[0], c[1], c[2]);
            v.set(vert::SX, i, sx);
            v.set(vert::SY, i, sy);
        }
    }
}

/// A far object's screen map (weak perspective): the projection linearised at its centre. The centre
/// projects to `(sx, sy)`, and mesh vertex `c = (a, h, b)` to `sx + (g[0]·c) >> 6`, `sy + (g[1]·c) >> 6`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Affine {
    pub sx: i16,
    pub sy: i16,
    pub g: [[i16; 3]; 2],
}

/// Objects nearer than `rad << AFFINE_SHIFT` (Q4; `rad` the mesh's bounding radius in texels, so 8
/// radii) take the full 3D path. Farther, the linearisation is off by about `rad / z` of the object's
/// size: about a pixel at the switch.
pub const AFFINE_SHIFT: u8 = 7;

/// `p >> s` rounded toward zero (`asm/geom.S` shifts magnitudes).
fn shr_mag(p: i32, s: u8) -> i32 {
    if p < 0 { -((-p) >> s) } else { p >> s }
}

/// [`Affine`] for object `o` (`asm/geom.S` `geo_affine`), an object at least [`AFFINE_SHIFT`]'s 8 radii
/// away and in view. Then `|g| < 1024`, and each row's sum stays within i16: the offsets are at most
/// `2128·1.2/z` times the radius (Q4), about 20 pixels, so the sums (64 times that) are below 21000.
///
/// With the centre `(x, y, z)` projected by [`project`] and `ex = x/z`, `ey = y/z` taken from it (Q7),
/// the screen offset of a camera-space offset `d` is `1280/z·(dX − ex·dZ)` and `−2128/z·(dY − ey·dZ)`.
/// With the rotation `m` (Q14) and vertices `c << t` (Q4), that is `g = (1280/z)·a0·2^(t − 8)` and
/// `(2128/z)·a1·2^(t − 8)` (in 1/64) for `a0 = m0 − ex·m2`, `a1 = ey·m2 − m1`; `1/z` is the top byte
/// of [`project`]'s scaled reciprocals: `1280/z ≈ (rx >> 8) / 2^(7 + e)`, `2128/z ≈ (ry >> 8) / 2^(6 + e)`.
/// Every product is 8×16 bits and every shift rounds toward zero, as the 6502 does it.
pub fn affine(o: &Obj) -> Affine {
    let [x, y, z] = o.org;
    let (sx, sy) = project(x, y, z);
    let (m, e) = fixed::norm(z as u16);
    let r = fixed::recip(m) as u32;
    let (rx, ry) = ((r * 5 >> 11) as i32, (r * 133 >> 16) as i32);
    let ex = shr_mag(mul(sx - 1280, 205), 11).clamp(-255, 255);
    let ey = shr_mag(mul(1088 - sy, 123), 11).clamp(-255, 255);
    let (shx, shy) = ((15 + e - o.t as i8) as u8, (14 + e - o.t as i8) as u8);
    let mut g = [[0i16; 3]; 2];
    for k in 0..3 {
        let m2 = o.m[2][k] as i32;
        let a0 = (o.m[0][k] as i32 - shr_mag(ex * m2, 7)) as i16 as i32;
        let a1 = (shr_mag(ey * m2, 7) - o.m[1][k] as i32) as i16 as i32;
        g[0][k] = shr_mag(a0 * rx, shx) as i16;
        g[1][k] = shr_mag(a1 * ry, shy) as i16;
    }
    Affine { sx, sy, g }
}

/// A far object's vertices through its [`Affine`] (`asm/geom.S` `afloop`): screen x, y, and z = the
/// centre's for all (so [`faces`] gives the object one depth key and draws its faces in reverse order).
pub fn xform_affine(scene: &[u8], o: &Obj, a: &Affine, v: &mut Verts) {
    for i in 0..o.nv as usize {
        let p = o.verts as usize + i * scene::VERTEX;
        let c = [scene[p] as i8 as i32, scene[p + 2] as i8 as i32, scene[p + 1] as i8 as i32];
        let row = |g: &[i16; 3]| ((g[0] as i32 * c[0] + g[1] as i32 * c[1] + g[2] as i32 * c[2]) as i16) >> 6;
        v.set(vert::Z, i, o.org[2]);
        v.set(vert::SX, i, a.sx + row(&a.g[0]));
        v.set(vert::SY, i, a.sy + row(&a.g[1]));
    }
}

/// A face's depth bucket: [`depth_key`] of its corners' z sum.
fn key(v: &Verts, idx: [usize; 3]) -> u8 {
    let z = |k: usize| v.get(vert::Z, idx[k]) as i32;
    depth_key(z(0) + z(1) + z(2))
}

/// One object's faces (`asm/geom.S` `geo_faces`): faces wholly in front of the near plane are culled and
/// pushed; faces straddling it are listed in `w.clip` (and left out).
pub fn faces(scene: &[u8], o: &Obj, v: &mut Verts, w: &mut Work) {
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
    if w.nclip > 0 {
        clip(&scene[o.faces as usize..], v, w);
    }
}

/// The two spare vertices [`clip`] writes its cuts into (meshes keep below them).
pub const CLIP_VERTS: usize = MAX_VERTS - 2;

/// The faces `faces` listed in `w.clip` (straddling the near plane): each cut at `z = NEAR` and pushed,
/// a triangle (one corner in front) or two (two in front), so a big face reaching behind the camera (a
/// mountain passed close by, the deck underfoot) stays drawn up to the plane. The cuts are vertices
/// `CLIP_VERTS..`, so the C64 pushes them with `asm/geom.S`'s own `tryface` (`geo_tri`). (`geo_faces`
/// calls it on the C64, from its own overlay over the main window's top: `format::OVERLAY_CLIP`.)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.clip"))]
pub fn clip(faces: &[u8], v: &mut Verts, w: &mut Work) {
    let mut k = 0;
    while k < w.nclip {
        let at = w.clip[k as usize] as usize * scene::FACE;
        k += 1;
        let byte = |i: usize| {
            // (The baked faces are in range: no bounds checks in the C64's overlay.)
            #[cfg(target_arch = "mos")]
            unsafe {
                *faces.get_unchecked(at + i)
            }
            #[cfg(not(target_arch = "mos"))]
            faces[at + i]
        };
        clip_face(v, w, [byte(0), byte(1), byte(2)], byte(3));
    }
}

/// One straddling face (corners `idx`, `shade`): its corners in front and its crossings of the plane,
/// in order round it (so the winding holds), pushed as one triangle or two.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.clip"))]
fn clip_face(v: &mut Verts, w: &mut Work, idx: [u8; 3], shade: u8) {
    let (mut ring, mut n, mut spare) = ([0u8; 4], 0u8, CLIP_VERTS as u8);
    let mut e = 0;
    while e < 3 {
        let a = idx[e] & (MAX_VERTS as u8 - 1);
        let b = idx[if e == 2 { 0 } else { e + 1 }] & (MAX_VERTS as u8 - 1);
        e += 1;
        let fa = in_front(v, a);
        if fa {
            ring[(n & 3) as usize] = a;
            n += 1;
        }
        if fa != in_front(v, b) {
            if fa { cut(v, a, b, spare) } else { cut(v, b, a, spare) }
            ring[(n & 3) as usize] = spare;
            n += 1;
            spare += 1;
        }
    }
    tri(v, w, ring[0], ring[1], ring[2], shade);
    if n == 4 {
        tri(v, w, ring[0], ring[2], ring[3], shade);
    }
}

#[inline(always)]
fn in_front(v: &Verts, i: u8) -> bool {
    v.get(vert::Z, i as usize) >= NEAR as i16
}

/// Vertex `s`: where the edge from `f` (in front) to `g` (behind) crosses `z = NEAR`, projected. 16-bit
/// throughout (the C64's code stays small).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.clip"))]
fn cut(v: &mut Verts, f: u8, g: u8, s: u8) {
    let (f, g, s) = (f as usize, g as usize, s as usize);
    let zf = v.get(vert::Z, f);
    // The fraction (zf − NEAR) / (zf − zg) of the way, Q16 (below 1): shift and subtract, both halved
    // until the difference fits 15 bits.
    let (mut den, mut rem) = (zf.wrapping_sub(v.get(vert::Z, g)) as u16, zf.wrapping_sub(NEAR as i16) as u16);
    while den >= 0x8000 {
        den >>= 1;
        rem >>= 1;
    }
    let mut t = 0u16;
    for _ in 0..16 {
        rem <<= 1;
        t <<= 1;
        if rem >= den {
            rem -= den;
            t |= 1;
        }
    }
    let x = lerp(v, vert::X, f, g, t);
    let y = lerp(v, vert::Y, f, g, t);
    v.set(vert::X, s, x);
    v.set(vert::Y, s, y);
    v.set(vert::Z, s, NEAR as i16);
    // `project` at z = NEAR (2 texels): 80 + 80·X/Z and 68 − 133·Y/Z in Q4, i.e. 1280 + 40·x and
    // 1088 − 66.5·y, from inputs clamped to stay inside the ±LIMIT corners.
    v.set(vert::SX, s, 80 * 16 + x.clamp(-432, 368) * 40);
    v.set(vert::SY, s, 68 * 16 - (y.clamp(-224, 225) * 133 >> 1));
}

/// Coordinate `c` the fraction `t` (Q16) of the way from vertex `f` to `g`: the high half of a 16 × 16
/// product, no 32-bit shifts.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.clip"))]
fn lerp(v: &Verts, c: usize, f: usize, g: usize, t: u16) -> i16 {
    let p = v.get(c, f);
    let d = v.get(c, g).wrapping_sub(p);
    let q = (fixed::umul(d.unsigned_abs(), t) >> 16) as i16;
    p.wrapping_add(if d < 0 { -q } else { q })
}

/// Push the triangle of vertices `a, b, c` (`asm/geom.S` `tryface` on the C64, the same sum).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.clip"))]
fn tri(v: &Verts, w: &mut Work, a: u8, b: u8, c: u8, shade: u8) {
    #[cfg(target_arch = "mos")]
    {
        let _ = (v, w);
        unsafe extern "C" {
            fn geo_tri();
        }
        // `tryface`'s corner indices and shade (its zero page: IA, IB, IC, SHD).
        unsafe {
            core::ptr::write_volatile(0xE7 as *mut u8, a);
            core::ptr::write_volatile(0xE8 as *mut u8, b);
            core::ptr::write_volatile(0xE9 as *mut u8, c);
            core::ptr::write_volatile(0xEA as *mut u8, shade);
            geo_tri();
        }
    }
    #[cfg(not(target_arch = "mos"))]
    {
        let idx = [a as usize, b as usize, c as usize];
        push(w, idx.map(|i| v.get(vert::SX, i)), idx.map(|i| v.get(vert::SY, i)), shade, key(v, idx));
    }
}

/// Draw the buckets back to front, whole triangles at a time into `bitmap` (the per-pixel equivalent of
/// [`raster_bands`] while nothing is dropped): triangles drawn.
pub fn raster<H: Hw + ?Sized>(hw: &mut H, w: &Work, bitmap: u16) -> u8 {
    let mut tris = 0u8;
    for key in (0..256).rev() {
        let mut i = w.head[key];
        while i != NONE {
            match activate(bitmap, &w.tri(i)) {
                Some(Item::Tri(t)) => {
                    walk(hw, &t);
                    tris += 1;
                }
                Some(Item::Sprite(sp)) => {
                    for band in 0..BANDS as u8 {
                        crate::sprite::sprite_band(hw, &sp, band, bitmap.wrapping_add(band as u16 * ROW_STRIDE));
                    }
                    tris += 1;
                }
                None => {}
            }
            i = w.next[i as usize];
        }
    }
    tris
}

/// Cull one object (a `scene::OBJECT` record) and, if it may be in view, transform and sort it (the
/// reference for `asm/geom.S` `wobject`).
#[inline(never)]
fn object<H: Hw + ?Sized>(hw: &mut H, scene: &[u8], rec: &[u8; scene::OBJECT], look: &Look, s: [i32; 3], sh: u8, w: &mut Work, v: &mut Verts, st: &mut Stats) {
    let (yaw, mesh) = (rec[scene::O_YAW], rec[scene::O_MESH] as usize);
    let m = scene::MESHES + mesh * scene::MESH;
    let rad = scene[m + 7];
    let c = object_centre(look, s, sh, rec);
    if !sphere_visible(c, (rad as i32) << 4, pop_depth(rad, look.scale)) {
        return;
    }
    st.objects = st.objects.wrapping_add(1);
    // Level of detail: a simpler mesh far away.
    let (lod, lod_z) = (scene[m + 8], (scene[m + 9] as i32) << 4);
    let m = if lod != 0xFF && c[2] > lod_z { scene::MESHES + lod as usize * scene::MESH } else { m };
    // Mesh (a, up, b) → world: `yaw` in quarter turns about up (columns swapped or negated, no
    // multiplies); then world → camera.
    let rt = &look.rt;
    let rot: [[i16; 3]; 3] = core::array::from_fn(|j| {
        let (a, b) = (rt[j][0], rt[j][2]);
        let (c0, c2) = match yaw >> 6 {
            0 => (a, b),
            1 => (b.wrapping_neg(), a),
            2 => (a.wrapping_neg(), b.wrapping_neg()),
            _ => (b, a.wrapping_neg()),
        };
        [c0, rt[j][1], c2]
    });
    let ob = Obj {
        verts: rd16(scene, m),
        nv: scene[m + 2].min(MAX_VERTS as u8),
        faces: rd16(scene, m + 3),
        nf: scene[m + 5],
        t: 2 + scene[m + 6],
        org: c.map(|x| x as i16),
        m: rot,
    };
    // Far: weak perspective, no divide per vertex.
    if c[2] > (rad as i32) << AFFINE_SHIFT {
        xform_affine(scene, &ob, &affine(&ob), v);
    } else {
        hw.xform(scene, &ob, v);
    }
    hw.faces(scene, &ob, v, w);
}

/// Stream one cell's objects (directory entry at `DIR_RAM + at`) from the REU into `CELL_RAM` and cull /
/// draw each.
#[inline(never)]
fn cell<H: Hw + ?Sized>(hw: &mut H, scene: &[u8], at: u16, look: &Look, s: [i32; 3], sh: u8, w: &mut Work, v: &mut Verts, st: &mut Stats) {
    let n = hw.peek(DIR_RAM + at + 2).min(CELL_MAX as u8) as u16;
    if n == 0 {
        return;
    }
    let first = hw.peek(DIR_RAM + at) as u32 | (hw.peek(DIR_RAM + at + 1) as u32) << 8;
    crate::hw::fetch(hw, crate::format::OBJ_RECORDS + (first << 3), CELL_RAM, n << 3);
    st.cells = st.cells.wrapping_add(1);
    let mut p = CELL_RAM;
    for _ in 0..n {
        let mut rec = [0u8; scene::OBJECT];
        for (k, b) in rec.iter_mut().enumerate() {
            *b = hw.peek(p + k as u16);
        }
        object(hw, scene, &rec, look, s, sh, w, v, st);
        p += scene::OBJECT as u16;
    }
}

/// One grid's walk this frame, as `geo_walk` is given it (`walk_args`): the rows `nv` of `nu` cells round
/// the camera (clipped to the map), their directory entries (row by row from `dir`, `stride` bytes
/// apart), the first cell's centre at ground level relative to the camera (`d0`, Q4 world axes), the
/// cell size (`1 << k`, Q4), the cells' bounding sphere (`r`, Q4: half-diagonal plus padding) and reach
/// (`far`), how far (`rn`, Q4) an object's centre can be from its cell's, and the shift of the records'
/// offsets (`sh`: 2 for quarter texels, 4 for whole ones).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GridIn {
    pub dir: u32,
    pub stride: u16,
    pub nu: u8,
    pub nv: u8,
    pub d0: [i16; 3],
    pub k: u8,
    pub r: i16,
    pub far: i16,
    pub rn: i16,
    pub sh: u8,
}

impl GridIn {
    /// The grid of `grid`² cells of `cell_q` (Q4, a power of two) whose directory is at `dir`, within
    /// `reach` cells of the camera's; `pad`: how far (Q4) beyond its cell an object can reach.
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.geomsetup"))]
    pub fn new(look: &Look, dir: u32, grid: usize, cell_q: i32, reach: i32, pad: i32, sh: u8) -> Self {
        // Q8 texels → cells.
        let k = cell_q.trailing_zeros();
        let (cu, cv) = (look.u >> (k + 4), look.v >> (k + 4));
        let (u0, u1) = ((cu - reach).max(0), (cu + reach).min(grid as i32 - 1));
        let (v0, v1) = ((cv - reach).max(0), (cv + reach).min(grid as i32 - 1));
        if u0 > u1 || v0 > v1 {
            return GridIn::default();
        }
        let half = cell_q >> 1;
        GridIn {
            dir: dir + ((v0 as u32 * grid as u32 + u0 as u32) << 2),
            stride: (grid as u16) << 2,
            nu: (u1 - u0 + 1) as u8,
            nv: (v1 - v0 + 1) as u8,
            d0: [(u0 * cell_q + half - (look.u >> 4)) as i16, look.y.wrapping_neg(), (v0 * cell_q + half - (look.v >> 4)) as i16],
            k: k as u8,
            // Half-diagonal (< 0.75 cell) plus the objects' reach and height.
            r: (half + (half >> 1) + pad) as i16,
            far: (FAR + pad) as i16,
            // 3/4 of a cell covers the half-diagonal (0.71) with the centres' height (9 texels) too.
            rn: (half + (half >> 1)) as i16,
            sh,
        }
    }
}

/// The frame's grids: the big one (mountains), then the fine one.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.geomsetup"))]
pub fn grids(look: &Look) -> [GridIn; 2] {
    use crate::format::{BIG_DIR, OBJ_DIR};
    [
        GridIn::new(look, BIG_DIR, BIG_GRID, BIG_CELL * Q, BIG_REACH, 255 * Q, BIG_SHIFT),
        GridIn::new(look, OBJ_DIR, GRID, CELL * Q, REACH, (CELL_PAD + 16) * Q, SHIFT),
    ]
}

/// What `geo_walk` derives from a [`GridIn`] (`wsetup`): the cells' exact [`Look::sums`], the first's
/// (`s0`) and the steps one cell east and north (`su`, `sv`: Rᵀ's columns times the cell); and for the
/// cull, the cell centres at 8 texels up in camera space, i16: the first (`base`) and the steps (`eu`,
/// `ev`), those sums shifted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GridWalk {
    pub g: GridIn,
    pub base: [i16; 3],
    pub eu: [i16; 3],
    pub ev: [i16; 3],
    pub s0: [i32; 3],
    pub su: [i32; 3],
    pub sv: [i32; 3],
}

impl GridWalk {
    pub fn new(look: &Look, g: GridIn) -> Self {
        let rt = &look.rt;
        let s0 = look.sums(g.d0);
        let su: [i32; 3] = core::array::from_fn(|j| (rt[j][0] as i32) << g.k);
        let sv: [i32; 3] = core::array::from_fn(|j| (rt[j][2] as i32) << g.k);
        GridWalk {
            g,
            base: core::array::from_fn(|j| ((s0[j] + ((rt[j][1] as i32) << 7)) >> 14) as i16),
            eu: su.map(|x| (x >> 14) as i16),
            ev: sv.map(|x| (x >> 14) as i16),
            s0,
            su,
            sv,
        }
    }
}

/// Walk one grid ([`GridWalk`]): fetch each row's directory entries to `DIR_RAM`; stream every cell
/// whose sphere may be in view and that holds an object big enough to have popped in (the reference
/// for `asm/geom.S` `geo_walk`).
#[inline(never)]
fn grid<H: Hw + ?Sized>(hw: &mut H, scene: &[u8], look: &Look, gw: &GridWalk, w: &mut Work, v: &mut Verts, st: &mut Stats) {
    let add = |a: [i16; 3], b: [i16; 3]| [a[0].wrapping_add(b[0]), a[1].wrapping_add(b[1]), a[2].wrapping_add(b[2])];
    let add32 = |a: [i32; 3], b: [i32; 3]| [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
    let g = &gw.g;
    let n = g.nu as u16;
    let (mut row, mut dir, mut srow) = (gw.base, g.dir, gw.s0);
    for _ in 0..g.nv {
        crate::hw::fetch(hw, dir, DIR_RAM, n << 2);
        let (mut c, mut s) = (row, srow);
        for k in 0..n {
            let cc = c.map(|x| x as i32);
            let r = g.r as i32;
            // Skipped unfetched when even its largest object (radius `rmax`) would pop in nearer than
            // any object of the cell can be.
            let rmax = hw.peek(DIR_RAM + (k << 2) + 3);
            if sphere_visible(cc, r, g.far as i32) && cc[2] - g.rn as i32 - ((rmax as i32) << 4) <= pop_depth(rmax, look.scale) {
                cell(hw, scene, k << 2, look, s, g.sh, w, v, st);
            }
            c = add(c, gw.eu);
            s = add32(s, gw.su);
        }
        row = add(row, gw.ev);
        srow = add32(srow, gw.sv);
        dir += g.stride as u32;
    }
}

/// Cull, transform and sort the objects that may be in view into `w`'s depth buckets (drawn band by
/// band by [`Bands`] on the host, `asm/geom.S` on the C64): the big grid's cells, then the grid's, each
/// streamed from the REU. Faces crossing the near plane are left out.
#[inline(never)]
pub fn draw<H: Hw + ?Sized>(hw: &mut H, scene: &[u8], view: &View, w: &mut Work, v: &mut Verts) -> Stats {
    let mut st = Stats::default();
    w.clear();
    let look = Look::new(view);
    for g in grids(&look) {
        grid(hw, scene, &look, &GridWalk::new(&look, g), w, v, &mut st);
    }
    st.dropped = w.dropped;
    st
}

/// Draw the frame's moving objects (`actors::DYN`, ready in camera space) after the static ones: each
/// one's mesh streamed from the REU (`format::VEHICLES`) into `actors::MESHBUF`, turned to its yaw (mesh
/// (a, up, b) → world by the yaw's sine and cosine, then world → camera by the frame's `Rᵀ` from
/// `walk_args`' block), transformed and filed by depth like a static object (always the near path: there
/// are few).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.dyn"))]
pub fn push_dynamic<H: Hw + ?Sized>(hw: &mut H, w: &mut Work, v: &mut Verts) {
    use crate::actors::{DYN, DYN_ENTRY, DYN_MAX, MESHBUF, dyn_entry};
    use crate::hud::mul16;
    use crate::layout::{SCENE_RAM, WALK_ARGS};
    let rd = |hw: &mut H, a: u16| (hw.peek(a) as u16 | (hw.peek(a + 1) as u16) << 8) as i16;
    let mut rt = [[0i16; 3]; 3];
    for (j, row) in rt.iter_mut().enumerate() {
        for (k, x) in row.iter_mut().enumerate() {
            *x = rd(hw, WALK_ARGS + walk::RT + 2 * (3 * j + k) as u16);
        }
    }
    let n = hw.peek(DYN).min(DYN_MAX);
    for i in 0..n as u16 {
        let at = DYN + 1 + i * DYN_ENTRY;
        let len = hw.peek(at + dyn_entry::LEN) as u16;
        let blob = rd(hw, at + dyn_entry::BLOB) as u16 as u32;
        crate::hw::fetch(hw, crate::format::VEHICLES + blob, MESHBUF, len);
        let (s, co) = (rd(hw, at + dyn_entry::SIN), rd(hw, at + dyn_entry::COS));
        let mut m = [[0i16; 3]; 3];
        for j in 0..3 {
            let (a, b) = (rt[j][0], rt[j][2]);
            m[j] = [((mul16(a, co) - mul16(b, s)) >> 14) as i16, rt[j][1], ((mul16(a, s) + mul16(b, co)) >> 14) as i16];
        }
        let base = MESHBUF.wrapping_sub(SCENE_RAM);
        let nv = hw.peek(at + dyn_entry::NV).min(MAX_VERTS as u8);
        let org = [rd(hw, at + dyn_entry::ORG), rd(hw, at + dyn_entry::ORG + 2), rd(hw, at + dyn_entry::ORG + 4)];
        let ob = Obj { verts: base, nv, faces: base + 3 * nv as u16, nf: hw.peek(at + dyn_entry::NF), t: 2 + hw.peek(at + dyn_entry::SHIFT), org, m };
        // (The host's 3D reads the mesh from a slice laid out as C64 RAM from `SCENE_RAM`; the C64's reads
        // RAM itself.)
        #[cfg(feature = "host")]
        let mem: alloc_vec::Vec<u8> = (0..base + len).map(|i| hw.peek(SCENE_RAM.wrapping_add(i))).collect();
        #[cfg(not(feature = "host"))]
        let mem: [u8; 0] = [];
        hw.xform(&mem, &ob, v);
        hw.faces(&mem, &ob, v, w);
    }
    // The effects (`combat::FXDYN`, placed in the set-up): each frame's sprite, its phase for its x.
    use crate::format::frame;
    let n = hw.peek(crate::combat::FXDYN).min(crate::combat::FXDYN_MAX);
    let mut rec = [0u8; crate::format::FRAME_RECORD as usize];
    for i in 0..n as u16 {
        let at = crate::combat::FXDYN + 1 + i * crate::combat::FXDYN_ENTRY;
        let (x, y) = (rd(hw, at), rd(hw, at + 2));
        let addr = rd(hw, at + 4) as u16 as u32 | (hw.peek(at + 6) as u32) << 16;
        crate::hw::read_into(hw, addr, crate::layout::BOUNCE, &mut rec);
        let h = u16::from_le_bytes([rec[frame::H], rec[frame::H + 1]]);
        if h == 0 {
            continue;
        }
        let left = x - rec[frame::AX] as i16;
        let top = y - rec[frame::AY] as i16;
        let p = if rec[frame::PHASE_COUNT] == 4 { (left & 3) as usize } else { 0 };
        let ph = frame::PHASES + p * frame::PHASE_LEN;
        let block = rec[ph + 1] as u32 | (rec[ph + 2] as u32) << 8 | (rec[ph + 3] as u32) << 16;
        push_sprite(w, left, top, h, rec[ph], block, rec[frame::BAND_SHIFT], hw.peek(at + 7));
    }
}

#[cfg(feature = "host")]
mod alloc_vec {
    pub use std::vec::Vec;
}

/// `asm/geom.S` `geo_walk`'s argument block (`layout::WALK_ARGS`): what [`draw`] uses, written once a
/// frame by [`walk_args`].
pub mod walk {
    /// i16 × 9: `Look::rt`, row-major.
    pub const RT: u16 = 0;
    /// i32: the camera's u and v (Q8 texels); i16: its height (Q4); i16: the pop scale.
    pub const U: u16 = 18;
    pub const V: u16 = 22;
    pub const Y: u16 = 26;
    pub const SCALE: u16 = 28;
    /// Two grids of `GRID_LEN` bytes: a [`super::GridIn`] (dir u24, stride u16, nu, nv, d0 3 × i16, k,
    /// r, far, rn i16, sh; `IN_LEN` bytes), then room for what `geo_walk` derives ([`super::GridWalk`]:
    /// base/eu/ev 3 × i16, s0/su/sv 3 × i32).
    pub const GRIDS: u16 = 30;
    pub const IN_LEN: u16 = 21;
    pub const GRID_LEN: u16 = IN_LEN + 54;
    /// out u8: objects drawn, cells streamed.
    pub const OBJECTS: u16 = GRIDS + 2 * GRID_LEN;
    pub const CELLS: u16 = OBJECTS + 1;
    pub const SIZE: u16 = CELLS + 1;
}

/// Write `geo_walk`'s arguments for `view` at `at`.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.geomsetup"))]
pub fn walk_args<H: Hw + ?Sized>(hw: &mut H, at: u16, view: &View) {
    let look = Look::new(view);
    let mut p = at;
    let mut put = |hw: &mut H, bytes: &[u8]| {
        for &b in bytes {
            hw.poke(p, b);
            p += 1;
        }
    };
    for row in &look.rt {
        for &x in row {
            put(hw, &x.to_le_bytes());
        }
    }
    put(hw, &look.u.to_le_bytes());
    put(hw, &look.v.to_le_bytes());
    put(hw, &look.y.to_le_bytes());
    put(hw, &look.scale.to_le_bytes());
    for g in grids(&look) {
        put(hw, &g.dir.to_le_bytes()[..3]);
        put(hw, &g.stride.to_le_bytes());
        put(hw, &[g.nu, g.nv]);
        for x in g.d0 {
            put(hw, &x.to_le_bytes());
        }
        put(hw, &[g.k]);
        put(hw, &g.r.to_le_bytes());
        put(hw, &g.far.to_le_bytes());
        put(hw, &g.rn.to_le_bytes());
        put(hw, &[g.sh]);
        for _ in walk::IN_LEN..walk::GRID_LEN {
            put(hw, &[0]);
        }
    }
    put(hw, &[0, 0]);
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
    let end = line_at(v[2].1).min(LINES);
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
            let first = if t.shade & SPRITE != 0 {
                (t.y[0] as i32).max(0)
            } else {
                line_at(t.y[0].min(t.y[1]).min(t.y[2]) as i32).max(0)
            };
            if first < LINES {
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

/// The sorted triangles drawn band by band, as the C64 publishes them (the reference for `asm/geom.S`
/// `geo_bin` / `geo_activate` / `geo_draw`): a triangle is set up when its first band comes, joins the
/// active list in painter's order (at most [`POOL`]; the rest are dropped), is walked one row per band
/// and leaves after its last. Per pixel this is the whole-frame painter's order.
pub struct Bands {
    order: [u8; 256],
    bins: [[u8; 256]; BANDS],
    lens: [u8; BANDS],
    active: [(u8, Item); POOL],
    n: usize,
    /// Triangles and sprites set up, and dropped (pool full).
    pub tris: u8,
    pub dropped: u8,
}

impl Bands {
    /// `asm/geom.S` `geo_bin`.
    pub fn new(w: &Work) -> Self {
        let (order, _, bins, lens) = bin(w);
        Bands { order, bins, lens, active: [(0, Item::default()); POOL], n: 0, tris: 0, dropped: 0 }
    }

    /// Band `band`'s rows of the active items into the 320-byte row at `row` (`geo_activate`, then
    /// `geo_draw`: the C64 draws into `STAGE`).
    pub fn band<H: Hw + ?Sized>(&mut self, hw: &mut H, w: &Work, band: usize, row: u16) {
        // Set-up bitmap such that this band's row lands at `row`.
        let bitmap = row.wrapping_sub(band as u16 * ROW_STRIDE);
        // Activate this band's newcomers (in painter's order), then merge them into the active list.
        let mut new: [(u8, Item); POOL] = [(0, Item::default()); POOL];
        let mut m = 0usize;
        for &seq in &self.bins[band][..self.lens[band] as usize] {
            let Some(item) = activate(bitmap, &w.tri(self.order[seq as usize])) else { continue };
            if self.n + m >= POOL {
                self.dropped = self.dropped.saturating_add(1);
                continue;
            }
            new[m] = (seq, item);
            m += 1;
            self.tris += 1;
        }
        let (n, active) = (self.n, &self.active);
        let mut merged: [(u8, Item); POOL] = [(0, Item::default()); POOL];
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
        self.active = merged;
        self.n += m;
        let mut kept = 0;
        for k in 0..self.n {
            let (seq, mut item) = self.active[k];
            let end = match &mut item {
                Item::Tri(walk) => {
                    walk.bitmap = bitmap;
                    walk_row(hw, walk, band as u8);
                    walk.end
                }
                Item::Sprite(s) => {
                    crate::sprite::sprite_band(hw, s, band as u8, row);
                    s.end
                }
            };
            if end as usize > band * 8 + 8 {
                self.active[kept] = (seq, item);
                kept += 1;
            }
        }
        self.n = kept;
    }
}

/// All bands of [`Bands`] straight into `bitmap`'s rows: (triangles, dropped).
pub fn raster_bands<H: Hw + ?Sized>(hw: &mut H, w: &Work, bitmap: u16) -> (u8, u8) {
    let mut b = Bands::new(w);
    for band in 0..BANDS {
        b.band(hw, w, band, bitmap.wrapping_add(band as u16 * ROW_STRIDE));
    }
    (b.tris, b.dropped)
}

/// An active item of the band pipeline: a set-up triangle or a placed sprite.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Item {
    Tri(Walk),
    Sprite(crate::sprite::Placed),
}

impl Default for Item {
    fn default() -> Self {
        Item::Tri(Walk::default())
    }
}

/// Set up an item when its first band comes: a triangle's edges, or a sprite's screen clip.
pub fn activate(bitmap: u16, t: &Tri) -> Option<Item> {
    if t.shade & SPRITE != 0 {
        let block = t.x[1] as u16 as u32 | ((t.x[2] as u16 & 0xFF) as u32) << 16;
        let shift = (t.x[2] as u16 >> 8) as u8;
        crate::sprite::place(block, shift, t.y[2] as u8, t.y[1] as u16, t.x[0], t.y[0]).map(Item::Sprite)
    } else {
        setup(bitmap, t).map(Item::Tri)
    }
}

/// Draw one triangle; returns its top cell row, or `None` when it covers no line centre.
#[inline(never)]
pub fn triangle<H: Hw + ?Sized>(hw: &mut H, bitmap: u16, t: &Tri) -> Option<u8> {
    let w = setup(bitmap, t)?;
    walk(hw, &w);
    Some(w.first >> 3)
}
