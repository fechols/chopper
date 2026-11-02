//! Solid-polygon renderer (host only): a flat-shaded triangle mesh, seen
//! straight from above through an orthographic camera, becomes a sprite
//! frame in the four global multicolor codes. Copied unchanged from `../talonhd/src/raster3d.rs`
//! (e0e891c); MODE 7 uses it to pre-render the racers (`crate::car`).
//!
//! The look (TALON HD art direction):
//! - **One key light per stage** ([`Light`]: its time of day), shared by the stage's sprites and its
//!   level geometry; S1's morning sun is upper-left and toward the viewer.
//! - **Cast shadows** on level geometry ([`render_scene`], a shadow map along the light).
//! - **Spotlights** ([`Spot`], [`render_scene_lit`]): real cone lights on level geometry (searchlights, a
//!   lighthouse lamp), lifting what they hit along a void → dark → light ramp, with soft ray-traced shadows.
//! - **Faceted:** one intensity per face (Lambert on the face normal).
//! - **Ramps:** each material shades through an ordered list of codes, dark →
//!   light. Intensity is quantized in half steps: a whole step is a solid
//!   code, a half step (the middle 40 % between two codes) a 50 % checkerboard. The checker is
//!   anchored in sprite space, so it moves with the ship and never crawls.
//! - **Emissive** materials (cores, engines, shots) ignore the light.
//!
//! Units: one model unit is one hires pixel (half a multicolor pixel across,
//! one line down). Model axes: x right, y forward (up the screen), z toward
//! the viewer. Each pixel takes 4×4 samples; a pixel is opaque when at least
//! half its samples hit the mesh, and takes the face covering most of them.

use crate::encode::SpriteImage;

pub type V3 = [f64; 3];

#[derive(Clone, Debug)]
pub struct Material {
    /// Multicolor codes, darkest first.
    pub ramp: Vec<u8>,
    /// Scales the light (0..1].
    pub albedo: f64,
    /// Lowest half-step level a lit face may fall to (keeps silhouettes off `void`).
    pub floor: u8,
    /// Ignores the light: always the ramp's last code.
    pub emissive: bool,
}

impl Material {
    pub fn lit(ramp: &[u8], albedo: f64, floor: u8) -> Material {
        Material { ramp: ramp.to_vec(), albedo, floor, emissive: false }
    }
    pub fn glow(code: u8) -> Material {
        Material { ramp: vec![code], albedo: 1.0, floor: 0, emissive: true }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    /// Material index per triangle.
    pub mats: Vec<u8>,
    pub materials: Vec<Material>,
}

impl Mesh {
    pub fn new(materials: Vec<Material>) -> Mesh {
        Mesh { materials, ..Mesh::default() }
    }
    pub fn vert(&mut self, p: V3) -> u32 {
        self.verts.push(p);
        self.verts.len() as u32 - 1
    }
    pub fn tri(&mut self, a: V3, b: V3, c: V3, mat: u8) {
        let (a, b, c) = (self.vert(a), self.vert(b), self.vert(c));
        self.tris.push([a, b, c]);
        self.mats.push(mat);
    }
    /// A planar or near-planar polygon as a triangle fan.
    pub fn poly(&mut self, pts: &[V3], mat: u8) {
        for i in 1..pts.len() - 1 {
            self.tri(pts[0], pts[i], pts[i + 1], mat);
        }
    }
    /// The polygon and its mirror image across x = 0.
    pub fn sym(&mut self, pts: &[V3], mat: u8) {
        self.poly(pts, mat);
        let m: Vec<V3> = pts.iter().map(|p| [-p[0], p[1], p[2]]).collect();
        self.poly(&m, mat);
    }
    /// An axis-aligned box from `lo` to `hi` (all six faces).
    pub fn cuboid(&mut self, lo: V3, hi: V3, mat: u8) {
        let p = |x: usize, y: usize, z: usize| [[lo[0], hi[0]][x], [lo[1], hi[1]][y], [lo[2], hi[2]][z]];
        self.poly(&[p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1)], mat);
        self.poly(&[p(0, 0, 0), p(1, 0, 0), p(1, 1, 0), p(0, 1, 0)], mat);
        self.poly(&[p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1)], mat);
        self.poly(&[p(0, 1, 0), p(1, 1, 0), p(1, 1, 1), p(0, 1, 1)], mat);
        self.poly(&[p(0, 0, 0), p(0, 1, 0), p(0, 1, 1), p(0, 0, 1)], mat);
        self.poly(&[p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1)], mat);
    }
    /// A box with a pyramid-like top: base `lo..hi` rising to an inset top at `top_z`.
    pub fn frustum(&mut self, lo: V3, hi: V3, inset: f64, top_z: f64, mat: u8) {
        let b = [[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]], [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]]];
        let t = [
            [lo[0] + inset, lo[1] + inset, top_z],
            [hi[0] - inset, lo[1] + inset, top_z],
            [hi[0] - inset, hi[1] - inset, top_z],
            [lo[0] + inset, hi[1] - inset, top_z],
        ];
        self.poly(&t, mat);
        for i in 0..4 {
            let j = (i + 1) % 4;
            self.poly(&[b[i], b[j], t[j], t[i]], mat);
        }
    }
    /// Append `other`, offset by `at`, with its materials remapped by `mat_base`.
    /// A ring of `n` points at radius `r` and height `z` (x right, y forward), starting at `phase` radians.
    pub fn ring(n: usize, r: f64, z: f64, phase: f64) -> Vec<V3> {
        (0..n)
            .map(|i| {
                let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
                [r * a.cos(), r * a.sin(), z]
            })
            .collect()
    }
    /// Quads joining two rings of equal length (closed loop).
    pub fn loft(&mut self, a: &[V3], b: &[V3], mat: u8) {
        for i in 0..a.len() {
            let j = (i + 1) % a.len();
            self.poly(&[a[i], a[j], b[j], b[i]], mat);
        }
    }
    /// A box beam from `p0` to `p1`, `w` wide (horizontal, across the beam) and `h` tall (z).
    pub fn beam(&mut self, p0: V3, p1: V3, w: f64, h: f64, mat: u8) {
        let d = sub(p1, p0);
        let side = normalize(cross(d, [0.0, 0.0, 1.0]));
        let up = normalize(cross(side, d));
        let corner = |p: V3, s: f64, u: f64| -> V3 {
            [p[0] + side[0] * s * w / 2.0 + up[0] * u * h / 2.0, p[1] + side[1] * s * w / 2.0 + up[1] * u * h / 2.0, p[2] + side[2] * s * w / 2.0 + up[2] * u * h / 2.0]
        };
        let a: Vec<V3> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].iter().map(|&(s, u)| corner(p0, s, u)).collect();
        let b: Vec<V3> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].iter().map(|&(s, u)| corner(p1, s, u)).collect();
        self.loft(&a, &b, mat);
        self.poly(&a, mat);
        self.poly(&b, mat);
    }
    /// An `n`-sided prism (barrel, mast) of radius `r` from `p0` to `p1`, capped.
    pub fn tube(&mut self, p0: V3, p1: V3, r: f64, n: usize, mat: u8, cap_mat: u8) {
        let d = normalize(sub(p1, p0));
        let helper = if d[2].abs() < 0.9 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
        let u = normalize(cross(d, helper));
        let v = cross(d, u);
        let ring = |c: V3| -> Vec<V3> {
            (0..n)
                .map(|i| {
                    let a = i as f64 * std::f64::consts::TAU / n as f64;
                    let (s, co) = a.sin_cos();
                    [c[0] + r * (u[0] * co + v[0] * s), c[1] + r * (u[1] * co + v[1] * s), c[2] + r * (u[2] * co + v[2] * s)]
                })
                .collect()
        };
        let (a, b) = (ring(p0), ring(p1));
        self.loft(&a, &b, mat);
        self.poly(&a, mat);
        self.poly(&b, cap_mat);
    }
    /// A pointed shard: a triangular base around `base` facing `tip`.
    pub fn shard(&mut self, base: V3, tip: V3, r: f64, mat: u8) {
        let d = normalize(sub(tip, base));
        let helper = if d[2].abs() < 0.9 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
        let u = normalize(cross(d, helper));
        let v = cross(d, u);
        let b: Vec<V3> = (0..3)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / 3.0;
                let (s, c) = a.sin_cos();
                [base[0] + r * (u[0] * c + v[0] * s), base[1] + r * (u[1] * c + v[1] * s), base[2] + r * (u[2] * c + v[2] * s)]
            })
            .collect();
        for i in 0..3 {
            self.tri(b[i], b[(i + 1) % 3], tip, mat);
        }
        self.poly(&b, mat);
    }
    pub fn add(&mut self, other: &Mesh, at: V3, mat_base: u8) {
        let base = self.verts.len() as u32;
        self.verts.extend(other.verts.iter().map(|v| [v[0] + at[0], v[1] + at[1], v[2] + at[2]]));
        self.tris.extend(other.tris.iter().map(|t| t.map(|i| i + base)));
        self.mats.extend(other.mats.iter().map(|m| m + mat_base));
    }
}

/// Orientation in radians: `roll` about the forward axis (banking), then
/// `pitch` about x (nose toward the viewer), then `yaw` about the view axis.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Orient {
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
}

impl Orient {
    pub fn apply(&self, p: V3) -> V3 {
        let (sr, cr) = self.roll.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let (sy, cy) = self.yaw.sin_cos();
        // Roll about y.
        let (x, y, z) = (p[0] * cr + p[2] * sr, p[1], -p[0] * sr + p[2] * cr);
        // Pitch about x.
        let (x, y, z) = (x, y * cp - z * sp, y * sp + z * cp);
        // Yaw about z (counter-clockwise seen from above).
        [x * cy - y * sy, x * sy + y * cy, z]
    }
}

impl From<(f64, f64, f64)> for Orient {
    fn from((yaw, pitch, roll): (f64, f64, f64)) -> Orient {
        Orient { yaw, pitch, roll }
    }
}

/// The S1 key light (morning): upper-left, toward the viewer.
pub const LIGHT: V3 = [-0.45, 0.55, 0.70];
const AMBIENT: f64 = 0.18;

/// A stage's key light: its time of day. `dir` points toward the sun (x right, y up the screen, z toward
/// the viewer). Shading is normalised to the morning sun's height, so a flat top is 1.0 at S1's light and
/// darker under a lower sun; `intensity` scales the sun, `ambient` is the floor every face keeps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Light {
    pub dir: V3,
    pub ambient: f64,
    pub intensity: f64,
}

/// S1 BREAKWATER, morning (and the light every sprite was approved under).
pub const MORNING: Light = Light { dir: LIGHT, ambient: AMBIENT, intensity: 1.0 };

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn normalize(a: V3) -> V3 {
    let l = dot(a, a).sqrt();
    [a[0] / l, a[1] / l, a[2] / l]
}

/// Shade of one face under the morning light.
fn shade(m: &Material, normal: V3) -> (u8, u8) {
    shade_lit(m, normal, &MORNING, false)
}

/// Shade of one face: (code a, code b); a == b is solid, else a checkerboard. `shadowed`: the sun is
/// blocked, only the ambient term remains.
fn shade_lit(m: &Material, normal: V3, light: &Light, shadowed: bool) -> (u8, u8) {
    let n = m.ramp.len();
    if m.emissive || n == 1 {
        let c = *m.ramp.last().unwrap();
        return (c, c);
    }
    // Normalized so a face pointing straight up (at the viewer) gets 1.0; faces tilted toward the key
    // light get up to 1/l.z (~1.42): with an albedo below 1, flat tops sit mid-ramp and light-facing
    // facets catch a highlight.
    let l = normalize(light.dir);
    let reference = normalize(LIGHT)[2];
    let lambert = if shadowed { 0.0 } else { (dot(normal, l) / reference).clamp(0.0, 1.0 / reference) * light.intensity };
    let lit = (light.ambient + (1.0 - light.ambient) * lambert) * m.albedo;
    // Position along the ramp; the checker covers only the middle 40 % of each step, so most faces
    // are solid and a checker always means a real mid-tone.
    let t = lit.clamp(0.0, 1.0) * (n - 1) as f64;
    let (i, f) = (t.floor() as usize, t.fract());
    let q = if i >= n - 1 { 2 * (n - 1) } else if f < 0.3 { 2 * i } else if f > 0.7 { 2 * i + 2 } else { 2 * i + 1 };
    let q = q.clamp(m.floor as usize, 2 * (n - 1));
    (m.ramp[q / 2], m.ramp[q.div_ceil(2)])
}

/// Where the model sits on the canvas: the model origin lands at hires
/// (`cx`, `cy`) of a `w`×`h` multicolor-pixel canvas; `scale` multiplies
/// model units.
#[derive(Clone, Copy, Debug)]
pub struct View {
    pub w: u16,
    pub h: u16,
    pub cx: f64,
    pub cy: f64,
    pub scale: f64,
}

impl View {
    /// Origin at the canvas centre.
    pub fn centered(w: u16, h: u16, scale: f64) -> View {
        View { w, h, cx: w as f64, cy: h as f64 / 2.0, scale }
    }
}

const SS: usize = 4;

/// Render `mesh` at orientation `o`.
pub fn render(mesh: &Mesh, o: Orient, view: View) -> SpriteImage {
    let (w, h) = (view.w as usize, view.h as usize);
    let (sw, sh) = (w * SS, h * SS);
    let mut depth = vec![f64::NEG_INFINITY; sw * sh];
    let mut face = vec![u32::MAX; sw * sh];
    // Sample (i, j) sits at hires x = (i + 0.5) / 2, line y = (j + 0.5) / 4.
    let pts: Vec<V3> = mesh
        .verts
        .iter()
        .map(|&v| {
            let r = o.apply([v[0] * view.scale, v[1] * view.scale, v[2] * view.scale]);
            // Screen: x right in samples, y down in samples, z toward the viewer.
            [(view.cx + r[0]) * 2.0, (view.cy - r[1]) * 4.0, r[2]]
        })
        .collect();
    let mut shades = Vec::with_capacity(mesh.tris.len());
    for (ti, t) in mesh.tris.iter().enumerate() {
        let (a, b, c) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
        // Normal in model-view space (undo the sample scaling).
        let ua = [a[0] / 2.0, -a[1] / 4.0, a[2]];
        let ub = [b[0] / 2.0, -b[1] / 4.0, b[2]];
        let uc = [c[0] / 2.0, -c[1] / 4.0, c[2]];
        let n = cross(sub(ub, ua), sub(uc, ua));
        let len = dot(n, n).sqrt();
        let normal = if len > 1e-9 {
            let n = [n[0] / len, n[1] / len, n[2] / len];
            // Winding-agnostic: the visible side faces the viewer.
            if n[2] < 0.0 { [-n[0], -n[1], -n[2]] } else { n }
        } else {
            [0.0, 0.0, 1.0]
        };
        shades.push(shade(&mesh.materials[mesh.mats[ti] as usize], normal));

        let area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        if area.abs() < 1e-9 {
            continue;
        }
        let x0 = (a[0].min(b[0]).min(c[0]).floor().max(0.0)) as usize;
        let x1 = (a[0].max(b[0]).max(c[0]).ceil().min(sw as f64)) as usize;
        let y0 = (a[1].min(b[1]).min(c[1]).floor().max(0.0)) as usize;
        let y1 = (a[1].max(b[1]).max(c[1]).ceil().min(sh as f64)) as usize;
        for sy in y0..y1 {
            let py = sy as f64 + 0.5;
            for sx in x0..x1 {
                let px = sx as f64 + 0.5;
                let w0 = ((b[0] - px) * (c[1] - py) - (b[1] - py) * (c[0] - px)) / area;
                let w1 = ((c[0] - px) * (a[1] - py) - (c[1] - py) * (a[0] - px)) / area;
                let w2 = 1.0 - w0 - w1;
                if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                    continue;
                }
                let z = w0 * a[2] + w1 * b[2] + w2 * c[2];
                let i = sy * sw + sx;
                if z > depth[i] {
                    depth[i] = z;
                    face[i] = ti as u32;
                }
            }
        }
    }

    let mut img = SpriteImage::new(view.w, view.h);
    let mut counts: Vec<(u32, u8)> = Vec::with_capacity(SS * SS);
    for y in 0..h {
        for x in 0..w {
            counts.clear();
            let mut covered = 0;
            for j in 0..SS {
                for i in 0..SS {
                    let f = face[(y * SS + j) * sw + x * SS + i];
                    if f == u32::MAX {
                        continue;
                    }
                    covered += 1;
                    match counts.iter_mut().find(|c| c.0 == f) {
                        Some(c) => c.1 += 1,
                        None => counts.push((f, 1)),
                    }
                }
            }
            if covered * 2 < SS * SS {
                continue;
            }
            let best = counts.iter().max_by_key(|c| (c.1, std::cmp::Reverse(c.0))).unwrap().0;
            let (ca, cb) = shades[best as usize];
            img.set(x as u16, y as u16, Some(if (x + y) & 1 == 0 { ca } else { cb }));
        }
    }
    img
}

/// A shadow map over a whole scene: the mesh rasterized along the light into a grid of the depth nearest
/// the sun (one cell per hires px of the plane across the light).
pub struct ShadowMap {
    e1: V3,
    e2: V3,
    l: V3,
    u0: f64,
    v0: f64,
    w: usize,
    h: usize,
    depth: Vec<f64>,
}

impl ShadowMap {
    pub fn new(mesh: &Mesh, light: &Light) -> ShadowMap {
        let l = normalize(light.dir);
        // A basis across the light.
        let up = if l[2].abs() < 0.9 { [0.0, 0.0, 1.0] } else { [0.0, 1.0, 0.0] };
        let e1 = normalize(cross(up, l));
        let e2 = cross(l, e1);
        let proj = |p: V3| (dot(p, e1), dot(p, e2), dot(p, l));
        let pts: Vec<(f64, f64, f64)> = mesh.verts.iter().map(|&v| proj(v)).collect();
        let (mut u0, mut v0, mut u1, mut v1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for &(u, v, _) in &pts {
            u0 = u0.min(u);
            v0 = v0.min(v);
            u1 = u1.max(u);
            v1 = v1.max(v);
        }
        let (u0, v0) = (u0.floor() - 1.0, v0.floor() - 1.0);
        let (w, h) = ((u1 - u0).ceil() as usize + 2, (v1 - v0).ceil() as usize + 2);
        let mut depth = vec![f64::NEG_INFINITY; w * h];
        for t in &mesh.tris {
            let (a, b, c) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
            let area = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
            if area.abs() < 1e-9 {
                continue;
            }
            let x0 = (a.0.min(b.0).min(c.0) - u0).floor().max(0.0) as usize;
            let x1 = ((a.0.max(b.0).max(c.0) - u0).ceil() as usize).min(w);
            let y0 = (a.1.min(b.1).min(c.1) - v0).floor().max(0.0) as usize;
            let y1 = ((a.1.max(b.1).max(c.1) - v0).ceil() as usize).min(h);
            for y in y0..y1 {
                let py = y as f64 + 0.5 + v0;
                for x in x0..x1 {
                    let px = x as f64 + 0.5 + u0;
                    let w0 = ((b.0 - px) * (c.1 - py) - (b.1 - py) * (c.0 - px)) / area;
                    let w1 = ((c.0 - px) * (a.1 - py) - (c.1 - py) * (a.0 - px)) / area;
                    let w2 = 1.0 - w0 - w1;
                    if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                        continue;
                    }
                    let d = w0 * a.2 + w1 * b.2 + w2 * c.2;
                    let i = y * w + x;
                    if d > depth[i] {
                        depth[i] = d;
                    }
                }
            }
        }
        ShadowMap { e1, e2, l, u0, v0, w, h, depth }
    }

    /// True when something lies between `p` (on a surface with normal `n`) and the sun. The lookup point
    /// is pushed off the surface along its normal, plus a small bias, so steep faces (a hangar's curved
    /// roof) don't shadow themselves while low objects (bushes) still cast.
    pub fn shadowed_at(&self, p: V3, n: V3) -> bool {
        self.shadowed([p[0] + n[0] * 2.0, p[1] + n[1] * 2.0, p[2] + n[2] * 2.0])
    }

    /// True when something lies between `p` and the sun (with a small bias against self-shadowing).
    pub fn shadowed(&self, p: V3) -> bool {
        let (u, v, d) = (dot(p, self.e1) - self.u0, dot(p, self.e2) - self.v0, dot(p, self.l));
        if u < 0.0 || v < 0.0 {
            return false;
        }
        let (x, y) = (u as usize, v as usize);
        if x >= self.w || y >= self.h {
            return false;
        }
        d < self.depth[y * self.w + x] - 1.0
    }
}

/// Render level geometry (no orientation: model space is the world) under `light`, with shadows from
/// `shadows` (built once for the whole scene). Pixels no triangle covers come out transparent.
pub fn render_scene(mesh: &Mesh, view: View, light: &Light, shadows: &ShadowMap) -> SpriteImage {
    render_scene_lit(mesh, view, light, shadows, &[], [0, 0, 0])
}

/// A spotlight: a cone from `pos` along `dir` (unit), full inside the half-angle `inner`, fading out at
/// `outer` (radians); its lens is a disc `radius` wide across the beam (soft shadows), `strength` scales it.
#[derive(Clone, Copy, Debug)]
pub struct Spot {
    pub pos: V3,
    pub dir: V3,
    pub inner: f64,
    pub outer: f64,
    pub radius: f64,
    pub strength: f64,
}

/// Lens samples per shadow test: a centre and a ring.
const LENS: usize = 8;

/// One spot ready for tracing: its lens points and the triangles that can shadow its pool.
struct SpotTrace {
    spot: Spot,
    lens: [V3; LENS],
    cos_in: f64,
    cos_out: f64,
    /// Candidate occluders: (triangle index, its bounding box).
    tris: Vec<(u32, [V3; 2])>,
}

impl SpotTrace {
    fn new(mesh: &Mesh, spot: Spot) -> SpotTrace {
        let d = normalize(spot.dir);
        let up = if d[2].abs() < 0.9 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
        let (e1, e2) = (normalize(cross(up, d)), cross(d, normalize(cross(up, d))));
        let mut lens = [spot.pos; LENS];
        for (k, l) in lens.iter_mut().enumerate().skip(1) {
            let a = k as f64 / (LENS - 1) as f64 * std::f64::consts::TAU;
            let r = spot.radius * if k % 2 == 0 { 1.0 } else { 0.6 };
            for i in 0..3 {
                l[i] = spot.pos[i] + (a.cos() * e1[i] + a.sin() * e2[i]) * r;
            }
        }
        // The pool's footprint on the ground plane: the outer cone's edge rays down to z 0.
        let (mut lo, mut hi) = ([spot.pos[0], spot.pos[1]], [spot.pos[0], spot.pos[1]]);
        let (so, co) = spot.outer.sin_cos();
        for k in 0..32 {
            let a = k as f64 / 32.0 * std::f64::consts::TAU;
            let r = [d[0] * co + (a.cos() * e1[0] + a.sin() * e2[0]) * so, d[1] * co + (a.cos() * e1[1] + a.sin() * e2[1]) * so, d[2] * co + (a.cos() * e1[2] + a.sin() * e2[2]) * so];
            let t = if r[2] < -1e-3 { (spot.pos[2] / -r[2]).min(400.0) } else { 400.0 };
            for i in 0..2 {
                let v = spot.pos[i] + r[i] * t;
                lo[i] = lo[i].min(v);
                hi[i] = hi[i].max(v);
            }
        }
        let tris = mesh
            .tris
            .iter()
            .enumerate()
            .filter_map(|(ti, t)| {
                let v = [mesh.verts[t[0] as usize], mesh.verts[t[1] as usize], mesh.verts[t[2] as usize]];
                let bmin = [v[0][0].min(v[1][0]).min(v[2][0]), v[0][1].min(v[1][1]).min(v[2][1]), v[0][2].min(v[1][2]).min(v[2][2])];
                let bmax = [v[0][0].max(v[1][0]).max(v[2][0]), v[0][1].max(v[1][1]).max(v[2][1]), v[0][2].max(v[1][2]).max(v[2][2])];
                let inside = bmax[0] >= lo[0] - 2.0 && bmin[0] <= hi[0] + 2.0 && bmax[1] >= lo[1] - 2.0 && bmin[1] <= hi[1] + 2.0;
                inside.then_some((ti as u32, [bmin, bmax]))
            })
            .collect();
        SpotTrace { spot, lens, cos_in: spot.inner.cos(), cos_out: spot.outer.cos(), tris }
    }

    /// Cone × facing at world point `p` on a face with normal `n` (0..1, no shadow).
    fn reach(&self, p: V3, n: V3) -> f64 {
        let v = sub(p, self.spot.pos);
        let len = dot(v, v).sqrt();
        if len < 1e-6 {
            return 0.0;
        }
        let v = [v[0] / len, v[1] / len, v[2] / len];
        let c = dot(v, normalize(self.spot.dir));
        if c <= self.cos_out {
            return 0.0;
        }
        let t = ((c - self.cos_out) / (self.cos_in - self.cos_out)).clamp(0.0, 1.0);
        let cone = t * t * (3.0 - 2.0 * t);
        let facing = (-dot(n, v)).max(0.0);
        cone * (0.3 + 0.7 * facing) * self.spot.strength
    }

    /// The share of the lens that `p` (on triangle `own`, normal `n`) sees.
    fn visible(&self, mesh: &Mesh, p: V3, n: V3, own: u32) -> f64 {
        let o = [p[0] + n[0] * 0.05, p[1] + n[1] * 0.05, p[2] + n[2] * 0.05];
        let mut seen = 0;
        for l in &self.lens {
            let d = sub(*l, o);
            let (bmin, bmax) = ([o[0].min(l[0]), o[1].min(l[1]), o[2].min(l[2])], [o[0].max(l[0]), o[1].max(l[1]), o[2].max(l[2])]);
            let blocked = self.tris.iter().any(|&(ti, [tmin, tmax])| {
                ti != own
                    && tmax[0] >= bmin[0]
                    && tmin[0] <= bmax[0]
                    && tmax[1] >= bmin[1]
                    && tmin[1] <= bmax[1]
                    && tmax[2] >= bmin[2]
                    && tmin[2] <= bmax[2]
                    && segment_hits(mesh, ti, o, d)
            });
            if !blocked {
                seen += 1;
            }
        }
        seen as f64 / LENS as f64
    }
}

/// Möller–Trumbore: does the segment `o` → `o + d` cross triangle `ti` (strictly between its ends)?
fn segment_hits(mesh: &Mesh, ti: u32, o: V3, d: V3) -> bool {
    let t = mesh.tris[ti as usize];
    let (a, b, c) = (mesh.verts[t[0] as usize], mesh.verts[t[1] as usize], mesh.verts[t[2] as usize]);
    let (e1, e2) = (sub(b, a), sub(c, a));
    let pv = cross(d, e2);
    let det = dot(e1, pv);
    if det.abs() < 1e-12 {
        return false;
    }
    let inv = 1.0 / det;
    let tv = sub(o, a);
    let u = dot(tv, pv) * inv;
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let qv = cross(tv, e1);
    let v = dot(d, qv) * inv;
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    let s = dot(e2, qv) * inv;
    s > 1e-4 && s < 1.0 - 1e-4
}

/// [`render_scene`] with spotlights: each pixel the spots reach is lifted along `ramp` (void, dark, light
/// codes; half steps 0..4) by their summed light, which is traced per sample (cone, facing) and per pixel
/// (shadow, over the lens). Codes outside the ramp (the Lattice's `hot`) stay as they are.
pub fn render_scene_lit(mesh: &Mesh, view: View, light: &Light, shadows: &ShadowMap, spots: &[Spot], ramp: [u8; 3]) -> SpriteImage {
    let traces: Vec<SpotTrace> = spots.iter().map(|&s| SpotTrace::new(mesh, s)).collect();
    let (w, h) = (view.w as usize, view.h as usize);
    let (sw, sh) = (w * SS, h * SS);
    let mut depth = vec![f64::NEG_INFINITY; sw * sh];
    let mut face = vec![u32::MAX; sw * sh];
    let pts: Vec<V3> = mesh.verts.iter().map(|&v| [(view.cx + v[0]) * 2.0, (view.cy - v[1]) * 4.0, v[2]]).collect();
    let mut normals = Vec::with_capacity(mesh.tris.len());
    for (ti, t) in mesh.tris.iter().enumerate() {
        let (va, vb, vc) = (mesh.verts[t[0] as usize], mesh.verts[t[1] as usize], mesh.verts[t[2] as usize]);
        let n = cross(sub(vb, va), sub(vc, va));
        let len = dot(n, n).sqrt();
        normals.push(if len > 1e-9 {
            let n = [n[0] / len, n[1] / len, n[2] / len];
            if n[2] < 0.0 { [-n[0], -n[1], -n[2]] } else { n }
        } else {
            [0.0, 0.0, 1.0]
        });
        let (a, b, c) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
        if a[1].max(b[1]).max(c[1]) < 0.0 || a[1].min(b[1]).min(c[1]) > sh as f64 {
            continue;
        }
        let area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        if area.abs() < 1e-9 {
            continue;
        }
        let x0 = (a[0].min(b[0]).min(c[0]).floor().max(0.0)) as usize;
        let x1 = (a[0].max(b[0]).max(c[0]).ceil().min(sw as f64)) as usize;
        let y0 = (a[1].min(b[1]).min(c[1]).floor().max(0.0)) as usize;
        let y1 = (a[1].max(b[1]).max(c[1]).ceil().min(sh as f64)) as usize;
        for sy in y0..y1 {
            let py = sy as f64 + 0.5;
            for sx in x0..x1 {
                let px = sx as f64 + 0.5;
                let w0 = ((b[0] - px) * (c[1] - py) - (b[1] - py) * (c[0] - px)) / area;
                let w1 = ((c[0] - px) * (a[1] - py) - (c[1] - py) * (a[0] - px)) / area;
                let w2 = 1.0 - w0 - w1;
                if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                    continue;
                }
                let z = w0 * a[2] + w1 * b[2] + w2 * c[2];
                let i = sy * sw + sx;
                if z > depth[i] {
                    depth[i] = z;
                    face[i] = ti as u32;
                }
            }
        }
    }
    let mut img = SpriteImage::new(view.w, view.h);
    let mut counts: Vec<(u32, u8)> = Vec::with_capacity(SS * SS);
    for y in 0..h {
        for x in 0..w {
            counts.clear();
            let mut covered = 0;
            for j in 0..SS {
                for i in 0..SS {
                    let f = face[(y * SS + j) * sw + x * SS + i];
                    if f == u32::MAX {
                        continue;
                    }
                    covered += 1;
                    match counts.iter_mut().find(|c| c.0 == f) {
                        Some(c) => c.1 += 1,
                        None => counts.push((f, 1)),
                    }
                }
            }
            if covered * 2 < SS * SS {
                continue;
            }
            let best = counts.iter().max_by_key(|c| (c.1, std::cmp::Reverse(c.0))).unwrap().0;
            // The pixel centre in world space: depth from the centre sample of the winning face.
            let centre = (y * SS + SS / 2) * sw + x * SS + SS / 2;
            let z = if face[centre] == best { depth[centre] } else {
                (0..SS * SS).map(|k| (y * SS + k / SS) * sw + x * SS + k % SS).find(|&i| face[i] == best).map(|i| depth[i]).unwrap()
            };
            let p = [(x * 2) as f64 + 1.0 - view.cx, view.cy - y as f64 - 0.5, z];
            let m = &mesh.materials[mesh.mats[best as usize] as usize];
            let n = normals[best as usize];
            let (mut ca, mut cb) = shade_lit(m, n, light, shadows.shadowed_at(p, n));
            if !traces.is_empty() {
                let level = |c: u8| ramp.iter().position(|&r| r == c);
                if let (Some(la), Some(lb)) = (level(ca), level(cb)) {
                    // Cone and facing over this face's samples (smooth pool edges), the shadow once.
                    let mut lift = 0.0;
                    for tr in &traces {
                        let (mut sum, mut k) = (0.0, 0);
                        for j in 0..SS {
                            for i in 0..SS {
                                let si = (y * SS + j) * sw + x * SS + i;
                                if face[si] != best {
                                    continue;
                                }
                                let q = [(x * 2) as f64 + (i as f64 + 0.5) * 2.0 / SS as f64 - view.cx, view.cy - y as f64 - (j as f64 + 0.5) / SS as f64, depth[si]];
                                sum += tr.reach(q, n);
                                k += 1;
                            }
                        }
                        let reach = sum / k.max(1) as f64;
                        if reach > 0.02 {
                            lift += reach * tr.visible(mesh, p, n, best);
                        }
                    }
                    let q = ((la + lb) as f64 + (lift * 3.0).round()).min(4.0) as usize;
                    (ca, cb) = (ramp[q / 2], ramp[q.div_ceil(2)]);
                }
            }
            img.set(x as u16, y as u16, Some(if (x + y) & 1 == 0 { ca } else { cb }));
        }
    }
    img
}

/// FNV-1a over a frame's pixels (golden tests).
pub fn hash(img: &SpriteImage) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for p in &img.pixels {
        h ^= p.map_or(0xFF, |c| c as u64);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}
