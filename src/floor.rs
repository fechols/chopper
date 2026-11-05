//! The floor: a 2048×2048-texel ground map seen from any altitude, pitch and roll, drawn with one REU
//! DMA per bitmap byte straight from the map's raw texel bytes.
//!
//! **Map** (`format`): one byte per texel (four multicolor pixels, so a texel is a colour or a dither),
//! in tiles of 256×256 texels, one 64 KiB REU bank each, so a texel's address is `bank:v_lo:u_lo` as in
//! mode7. Mip levels 1-3 (1/2, 1/4, 1/8 size) are tiled the same way. [`tile_bank`] maps a level's tile
//! coordinates to its bank; tiles off the map show the sea.
//!
//! **Rays**: screen byte `b` of view line `y` samples pixel `(4b + 2, y + 0.5)`. Its camera-space ray
//! `((x − 80)/FX, (CY − y)/FY, 1)` rotated into the world is affine in `b`: `D = A_y + b·B`. Below the
//! horizon (`w = −D_y > 0`) it meets the ground at `cam + H·(D_x, D_z)/w`.
//!
//! **Spans**: a line is cut into runs. Sky and haze runs are filled with a constant byte (no DMA).
//! Ground runs have a power-of-two length, exact texel positions at both ends (one reciprocal and two
//! multiplies), and a constant step per byte between them (a shift). A run is as long as the affine
//! error allows (from the change of `w` across it alone, so no division is wasted on rejected runs): a
//! level line (no roll) is one run. Each run reads the mip level its step needs.
//!
//! [`make_line`] is shared by the host and the C64 (same bits); [`draw_line`] is the reference for
//! `target-c64/asm/floor.S`.

use crate::fixed::{mul, mulsu, recip};
use crate::hw::{Hw, reu};

/// Horizontal and vertical focal lengths in multicolor pixels: about 90° across, and square pixels on
/// screen (a multicolor pixel is ~1.66 times as wide as it is tall).
pub const FX: i32 = 80;
pub const FY: i32 = 133;
/// View lines (16 cell rows) above the cockpit, and the optical axis' row.
pub const VIEW_LINES: u16 = 128;
pub const VIEW_BANDS: u16 = VIEW_LINES / 8;
pub const CY_HALF: i32 = 135; // 2 · geom::CY − 1: line y's centre is at (CY_HALF − 2y)/2 above the axis
/// Bytes per line.
/// Bytes a line's runs are planned over (the screen's width, so the power-of-two runs split it as they
/// always have), and the bytes drawn: the last run stops one short (`layout::DRAWN_COLS`).
pub const COLS: u8 = 40;
pub const DRAWN: u8 = crate::layout::DRAWN_COLS as u8;
const _: () = assert!(DRAWN + 1 == COLS);

/// Map size in texels (level 0) and tiles.
pub const MAP_TEXELS: u32 = 2048;
pub const MAP_TILES: u8 = (MAP_TEXELS / 256) as u8;
pub const LEVELS: u8 = 6;
/// Levels 3 and up are one tile each (256², 128², 64²); only the levels below have a tile → bank table
/// on the C64 (`layout::TILES`; `asm/floor.S` `lookup` tests tile 0 itself for the rest).
pub const TABLE_LEVELS: u8 = 3;
const _: () = assert!(MAP_TILES >> TABLE_LEVELS == 1);
/// Where a one-tile level's texels sit in its bank, in texels along u and v: levels 4 and 5 (128², 64²)
/// are centred in banks they share (`format::LEVEL_BANK`), so the texels just off the map read the sea
/// filled around them (64 and 96 texels of it: 1024 and 3072 level-0 texels, far past the fog).
pub const MIP_OFFSET: [u8; LEVELS as usize] = [0, 0, 0, 0, 64, 96];

/// Fill bytes: sky (code 1 in every pixel), haze (codes 1 and 0 alternating; the other phase on odd
/// lines, [`haze_byte`], so it is a checkerboard rather than columns).
pub const SKY_BYTE: u8 = 0x55;
pub const HAZE_BYTE: u8 = 0x44;
pub const HAZE_ODD: u8 = 0x11;

/// The haze byte of line `line` of a band (drawn lines alternate: in line-doubled mode a drawn line
/// covers two).
pub fn haze_byte(line: u8, doubled: bool) -> u8 {
    if (line >> doubled as u8) & 1 == 0 { HAZE_BYTE } else { HAZE_ODD }
}
/// In AUTO (`layout::status::mode`), the lines this frame's floor is drawn in, from the last frame's
/// times in the status block: line-doubled after a heavy frame, full after a light one, as it was
/// between. (On the C64 in the geometry set-up's overlay, before the rays.)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.geomsetup"))]
pub fn pick_lines<H: Hw + ?Sized>(hw: &mut H) {
    use crate::layout::{STATUS, status, status::mode};
    let m = hw.peek(STATUS + status::MODE);
    if m & mode::AUTO == 0 {
        return;
    }
    let rd = |hw: &mut H, a: u16| hw.peek(STATUS + a) as u16 | (hw.peek(STATUS + a + 1) as u16) << 8;
    // (In 4 µs units: the render is kept in them already.)
    let took = (rd(hw, status::SETUP_US) >> 2) + (rd(hw, status::GEOM_US) >> 2) + rd(hw, status::RENDER_US);
    let lines = if took > mode::HEAVY {
        mode::DOUBLED
    } else if took < mode::LIGHT {
        0
    } else {
        m & mode::DOUBLED
    };
    hw.poke(STATUS + status::MODE, mode::AUTO | lines);
}

/// Ground beyond this many texels (along the ray, roughly) is drawn as haze.
pub const HAZE_TEXELS: i32 = 320;
/// The steepest step a run may read from a level, 8.8 texels per byte (above it, the next level).
pub const MAX_STEP: i32 = 0x180;

/// The floor camera: position in 16.8 texels (u east, v north), altitude in 8.8 texels, orientation
/// (Q14, camera → world: columns right, up, forward; world axes u, up, v).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FloorCam {
    pub u: i32,
    pub v: i32,
    pub h: u16,
    pub r: [[i32; 3]; 3],
}

/// Per-frame ray set-up: line `y`'s ray at byte `b` is `a + y·ay + b·bb` (Q22 world directions, except
/// that the x and z components are pre-multiplied by `h / 2^L`, `L` = `h`'s bit length, so a ground
/// point needs no multiply by the altitude).
#[derive(Clone, Copy, Debug)]
pub struct Rays {
    pub a0: [i32; 3],
    pub ay: [i32; 3],
    pub bb: [i32; 3],
    /// How much `w` drops over `2^k` bytes: `bb.y << k`.
    pub dw: [i32; 6],
    /// For k ≥ 1, a run of `2^k` bytes from a byte with `w = w0` stays on visible ground within the error
    /// budget exactly when `w0 ≥ t[k]` (see [`make_line`]). `t[0]`: a one-byte run whose far end is
    /// still visible ground needs `w0 ≥ t[0]`.
    pub t: [i32; 6],
    /// `w` below which the ground is haze.
    pub w_haze: i32,
    /// `26 − L`: the exponent the altitude scaling leaves for [`ground_offset`].
    pub hc: u8,
}

/// The fog line, painted over the drawn line at multicolour-pixel resolution (`layout::FOG`, for
/// `asm/spans.S`). `w` is affine on screen, so where it crosses the fog depth `w_haze + 2·|dw[0]|` is a
/// straight line, and where it crosses 0 (the horizon) a parallel one, while the runs only split at
/// bytes (4 pixels). Line `y`'s crossings plus half a pixel, `at(y)`, are `pf` and `ph` (16.16 pixels, at
/// line 0) plus `y · step`: on line `y` the pixels left of `pf`'s integer part are ground when `dir` is 1
/// (`w` falls along the line; the other side when it is `$FF`), and those past `ph`'s are sky. The fog
/// depth's margin over `w_haze` puts every ground pixel in a byte the runs draw as ground, so painting
/// only covers ground with haze or sky (never needs a texel the runs did not fetch). `dir` 0: nothing
/// painted (level, or the lines jump 64 pixels or more a line, where byte steps do not show).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C, packed)]
pub struct FogArgs {
    pub dir: u8,
    pub pf: i32,
    pub ph: i32,
    pub step: i32,
}
pub const FOG_BYTES: usize = 13;
const _: () = assert!(core::mem::size_of::<FogArgs>() == FOG_BYTES);

/// `n / d` in 16.16, floored (`d > 0`, under 2^20), or `None` from 2^13 up: one i32 divide for the
/// integer part, two byte-sized steps of long division for the fraction (no i64 on the C64). (On the
/// C64 in the actors' overlay, with [`write_fog`].)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.actors"))]
fn q16(n: i32, d: i32) -> Option<i32> {
    let mut q = sdiv(n, d);
    let mut r = n - q * d;
    if r < 0 {
        q -= 1;
        r += d;
    }
    if q.unsigned_abs() >= 1 << 13 {
        return None;
    }
    let a = r << 8;
    let q1 = sdiv(a, d);
    let q2 = sdiv((a - q1 * d) << 8, d);
    Some((q << 16) + (q1 << 8) + q2)
}

/// `n / d`, truncated, out of line: with the remainder taken as `n − q·d` next to it, an inline divide
/// becomes compiler_builtins' `__divmodsi4` on the C64 (~1.6 KiB resident), not `asm/fixed.S`'s.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.actors"))]
fn sdiv(n: i32, d: i32) -> i32 {
    n / d
}

impl FogArgs {
    /// From the fog's `w_haze`, `w`'s drop a byte `dw0`, line 0's `w` at byte 0 `w0` and its drop a line
    /// `a`: byte `b` of line `y` (centre pixel `4b + 2`) has `w0 − y·a − b·dw0`, so `w` crosses `t` at pixel
    /// `2 + 4(w0 − y·a − t)/dw0`.
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.actors"))]
    fn new(w_haze: i32, dw0: i32, w0: i32, a: i32) -> Self {
        if dw0 == 0 {
            return Self::default();
        }
        let (sg, d) = (dw0.signum(), 2 * dw0.abs());
        let line = |t: i32| q16(sg * (8 * (w0 - t) + 5 * dw0), d);
        match (line(w_haze + 2 * dw0.abs()), line(0), q16(sg * -8 * a, d)) {
            (Some(pf), Some(ph), Some(step)) if step.unsigned_abs() <= 1 << 22 => Self { dir: if dw0 > 0 { 1 } else { 0xFF }, pf, ph, step },
            _ => Self::default(),
        }
    }

    /// [`Rays::fog`] from the camera alone: the rays' y components and `w_haze`, as `Rays::new` makes them
    /// (the same expressions; `the_fog_line_and_the_horizon_are_painted_to_the_pixel` checks they agree).
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.actors"))]
    pub fn of_cam(cam: &FloorCam) -> Self {
        let (right, up, fwd) = (cam.r[1][0], cam.r[1][1], cam.r[1][2]);
        let dw0 = div8(right * (256 * 4 / 16), FX5);
        let ay = div8(-(up * 256), FY as u8);
        let a0 = fwd * 256 + div8(right * (-78 * 256 / 16), FX5) + div8(up * (CY_HALF * 128), FY as u8);
        Self::new(div8((cam.h as i32) << 8, FX5), dw0, -a0, ay)
    }

    /// Line `y`'s `[pf, ph]` (the C64 adds `step` up a drawn line at a time, [`write_fog`]: the same sums).
    pub fn at(&self, y: u16) -> [i32; 2] {
        let (pf, ph, step) = (self.pf, self.ph, self.step);
        let k = step.wrapping_mul(y as i32);
        [pf.wrapping_add(k), ph.wrapping_add(k)]
    }
}

/// The frame's fog line into `layout::FOG` for `asm/spans.S` (which steps it twice a drawn line when
/// `layout::FOG_DOUBLED` says the lines are doubled). In the actors' overlay: the geometry set-up's,
/// where the rays are made, has no room for its 32-bit arithmetic, so it is made from the camera
/// ([`FogArgs::of_cam`]).
#[cfg(target_arch = "mos")]
#[inline(never)]
#[unsafe(link_section = ".text.actors")]
pub fn write_fog(cam: &FloorCam) {
    unsafe { core::ptr::write_volatile(crate::layout::FOG as *mut FogArgs, FogArgs::of_cam(cam)) };
}

/// The most bytes a line paints: up to 3 at the ground's fog end (the fog depth is 2 bytes' `dw` past the
/// runs' ground) and 1 where the haze meets the sky.
pub const MAX_PATCHES: usize = 4;

/// A line's painted bytes (`layout::FOG_PATCHES`): `p[i] = [byte, keep, bits]`, the drawn byte becoming
/// `byte & keep | bits`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct Patches {
    pub n: u8,
    pub p: [[u8; 3]; MAX_PATCHES],
}
const _: () = assert!(core::mem::size_of::<Patches>() == 1 + 3 * MAX_PATCHES);

/// The error budget: a run of `len` bytes is accepted when `ERR · len² · |Δw per byte| ≤ w0 + w1`.
/// The affine error is about `|Δf|·|Δw| / (2(w0 + w1))` with `|Δf| ≤ 1.5·len` level texels, so `ERR = 2`
/// keeps it under about 0.4 texel.
pub const ERR: i32 = 2;

/// `Rays::new`'s divisors as bytes: `FX / 16` and `HAZE_TEXELS / 64` (both 5), and `FY`. (Changing
/// FX, FY or HAZE_TEXELS means redoing these reductions.)
const FX5: u8 = 5;
const _: () = assert!(FX == 16 * FX5 as i32 && HAZE_TEXELS == 64 * FX5 as i32 && FY < 256);

/// `((c · h) >> l) as i32` (the i64 product's), for `l <= 16`, in two 32-bit products of 16-bit operands
/// (an i64 product is libcrt's `__muldi3`, 234 resident bytes on the C64; these stay `__mulsi3` libcalls,
/// which `asm/fixed.S` gives a 16 × 16 fast path). With `c = hi · 2^16 + lo`, `hi · h · 2^16` is a multiple
/// of `2^l`, so the shift splits exactly; the wrapping sums agree with the i64 result modulo 2^32.
#[inline(always)]
fn scale_h(c: i32, h: u16, l: u32) -> i32 {
    let (hi, lo) = (c >> 16, (c & 0xFFFF) as u32);
    (hi * h as i32).wrapping_shl(16 - l).wrapping_add(((lo * h as u32) >> l) as i32)
}

/// `n / d`, truncated. On the C64 the i32 divide is still the `__divsi3` libcall (so `Rays::new` keeps
/// its static frame: a call to an asm extern would put it on the soft stack), which `asm/fixed.S`
/// provides with a fast path for divisors that fit a byte.
#[inline(always)]
fn div8(n: i32, d: u8) -> i32 {
    n / d as i32
}

impl Rays {
    /// `cam.h` must be at least 64 (a quarter texel). (On the C64 in the geometry set-up's overlay.)
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.geomsetup"))]
    pub fn new(cam: &FloorCam) -> Self {
        let col = |j: usize| [cam.r[0][j], cam.r[1][j], cam.r[2][j]];
        let (right, up, fwd) = (col(0), col(1), col(2));
        // Q14 → Q22 is ×256. Byte step: 4 pixels / FX. Byte 0's centre: pixel 2, (2 − 80)/80 = −78/80.
        // Line 0's centre: (CY_HALF/2)/FY above the axis.
        // (Divided by bytes, the factors 16 of FX and 64 of HAZE_TEXELS taken out: exact, as truncation
        // of 16x / 80 is that of x / 5.)
        let mut bb = right.map(|c| div8(c * (256 * 4 / 16), FX5));
        let mut ay = up.map(|c| div8(-(c * 256), FY as u8));
        let mut a0: [i32; 3] = core::array::from_fn(|k| fwd[k] * 256 + div8(right[k] * (-78 * 256 / 16), FX5) + div8(up[k] * (CY_HALF * 128), FY as u8));
        let l = 16 - cam.h.leading_zeros();
        let scale = |c: &mut i32| *c = scale_h(*c, cam.h, l);
        for v in [&mut a0, &mut ay, &mut bb] {
            scale(&mut v[0]);
            scale(&mut v[2]);
        }
        // A ray of length ~1 (Q22 = 2^22) reaches HAZE_TEXELS when w = H/HAZE_TEXELS (H in 8.8 texels).
        let w_haze = div8((cam.h as i32) << 8, FX5);
        // |Δw per byte| ≤ 2^22 / 20; ERR · 4^5 · that stays within i32.
        let dw = core::array::from_fn(|k| bb[1] << k);
        // w1 = w0 − dw[k]: `w1 ≥ w_haze` and `limit ≤ w0 + w1 = 2·w0 − dw[k]`.
        let t = core::array::from_fn(|k| {
            if k == 0 {
                w_haze + dw[0]
            } else {
                let limit = ERR * (bb[1].abs() << (2 * k));
                (w_haze + dw[k]).max((limit + dw[k] + 1) >> 1)
            }
        });
        Self { a0, ay, bb, dw, t, w_haze, hc: (26 - l) as u8 }
    }

    /// The frame's fog line ([`FogArgs`]).
    #[inline(always)]
    pub fn fog(&self) -> FogArgs {
        FogArgs::new(self.w_haze, self.dw[0], -self.a0[1], self.ay[1])
    }

    /// Line `y`'s ray at byte 0 (the frame loop steps it by [`Rays::ay`] instead).
    pub fn line(&self, y: u16) -> [i32; 3] {
        core::array::from_fn(|k| self.a0[k] + mul_small(self.ay[k], y as u8))
    }

    /// `b · bb`: byte `b`'s ray is the line's ray at byte 0 plus this. The C64 keeps the x and z
    /// components as a table (`layout::BT`, field-major: row `4c + i` holds byte i of component c for
    /// b = 0..=COLS), built by adding `bb` up, which gives the same values.
    pub fn bt(&self, b: u8) -> [i32; 3] {
        self.bb.map(|c| mul_small(c, b))
    }
}

/// `asm/spans.S`'s arguments (`layout::RAYS`): written once a frame, `a` (the line's ray at byte 0)
/// before each line; the kernel leaves the run count in `n`. `bt_build` fills `layout::BT` from it.
#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct SpanArgs {
    pub t: [i32; 6],
    pub dw: [i32; 6],
    pub w_haze: i32,
    pub cam_u: i32,
    pub cam_v: i32,
    pub hc: u8,
    pub a: [i32; 3],
    pub n: u8,
    /// `bb`'s x and z, for `asm/spans.S`'s `bt_build`.
    pub bbx: i32,
    pub bbz: i32,
    /// The line's haze byte ([`haze_byte`]).
    pub haze: u8,
}
const _: () = assert!(core::mem::size_of::<SpanArgs>() == 83);

impl SpanArgs {
    pub fn new(cam: &FloorCam, rays: &Rays) -> Self {
        Self { t: rays.t, dw: rays.dw, w_haze: rays.w_haze, cam_u: cam.u, cam_v: cam.v, hc: rays.hc, a: rays.a0, n: 0, bbx: rays.bb[0], bbz: rays.bb[2], haze: HAZE_BYTE }
    }
}

/// One run of a line: `len` bytes, either a constant fill (`flags` bit 7, the byte in `u`), or ground
/// read from mip level `flags` starting at `(u, v)` (16.8 texels at that level) and stepping `(du, dv)`
/// (8.8) per byte. `asm/spans.S` writes them for `asm/floor.S` as 12-byte records: len, flags, then
/// the fill byte, or u and v (the low three bytes each), du, dv.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Span {
    pub len: u8,
    pub flags: u8,
    pub u: i32,
    pub v: i32,
    pub du: i16,
    pub dv: i16,
}
pub const SPAN_BYTES: usize = 12;

impl Span {
    pub fn fill(&self) -> Option<u8> {
        if self.flags & 0x80 != 0 { Some(self.u as u8) } else { None }
    }
    pub fn level(&self) -> u8 {
        self.flags & 7
    }
    fn filled(len: u8, byte: u8) -> Self {
        Span { len, flags: 0x80, u: byte as i32, ..Span::default() }
    }
}

/// The most runs a line can need (40 bytes).
pub const MAX_SPANS: usize = 40;

/// `v ≈ m · 2^e` with `m` in `1024..2048` (`v > 0`).
fn norm16(v: u16) -> (u16, i8) {
    let (mut m, mut e) = (v, 0i8);
    while m >= 2048 {
        m >>= 1;
        e += 1;
    }
    while m < 1024 {
        m <<= 1;
        e -= 1;
    }
    (m, e)
}

/// The ground point (16.8 texels, level 0) relative to the camera of the ray whose (altitude-scaled,
/// [`Rays`]) x and z components are `x`, `z` and whose `w` is `w` (at least `w_haze`):
/// `H · (x, z) / w`. The components are used as 16-bit values (`>> 8`: at most ~1.2·10⁻⁴ of the
/// distance off) and `1 / w` as a 16-bit factor whose exponent is rounded to whole bytes (at least 9
/// significant bits: 0.2%), so the 6502 needs one normalisation, two 16×16 multiplies and whole-byte
/// shifts. Both errors stay far below a texel of the mip level read at that distance.
pub fn ground_offset(hc: u8, x: i32, z: i32, w: i32) -> (i32, i32) {
    let (m, k) = norm16((w >> 8) as u16);
    // H·x/w = X·2^L/w ≈ X · recip(m) · 2^−(hc + k), recip(m) ≈ 2^26/m; take recip's low bits off so the
    // remaining exponent is whole bytes.
    let ck = (hc as i8 + k) as u8;
    debug_assert!(ck < 32);
    let p = recip(m) >> (ck & 7);
    let e = ck & !7;
    let off = |n: i32| mulsu((n >> 8) as i16, p) >> e;
    (off(x), off(z))
}

/// The mip level whose step stays within [`MAX_STEP`], or `None` beyond the last.
pub(crate) fn level_for(du: i32, dv: i32) -> Option<u8> {
    let s = du.abs().max(dv.abs());
    (0..LEVELS).find(|&k| s >> k <= MAX_STEP)
}

/// Append `len` bytes of `fill`, extending the last run when it is the same fill.
pub(crate) fn put_fill(out: &mut [Span; MAX_SPANS], n: &mut usize, len: u8, fill: u8) {
    if *n > 0 && out[*n - 1].fill() == Some(fill) {
        out[*n - 1].len += len;
    } else {
        out[*n] = Span::filled(len, fill);
        *n += 1;
    }
}

fn add(a: &[i32; 3], b: &[i32; 3]) -> [i32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// What a byte with depth `w` shows: 0 sky, 1 haze, 2 ground.
fn class(rays: &Rays, w: i32) -> u8 {
    if w <= 0 {
        0
    } else if w < rays.w_haze {
        1
    } else {
        2
    }
}

/// One line's runs into `out` from its ray at byte 0 (`Rays::line`); returns how many. The runs are
/// planned over the [`COLS`] bytes in order, then the last is cut one byte short (dropped if that was
/// all of it): they cover the [`DRAWN`] bytes. Sky (`w ≤ 0`) and haze bytes become fills; ground runs are as long as the error
/// budget allows (a power of two, so the step is a shift), with exact end points.
///
/// `w` is linear along the line (`w(b + 2^k) = w(b) − dw[k]`), so the run length is found by comparing
/// `w` at the run's start with the frame's thresholds `Rays::t`, and a stretch of sky or haze (`w` is
/// monotone, so it is one stretch) by galloping over it in power-of-two steps.
pub fn make_line(cam: &FloorCam, rays: &Rays, a: &[i32; 3], haze: u8, fog_at: [i32; 2], out: &mut [Span; MAX_SPANS], fog: &mut Patches) -> usize {
    let ray_at = |b: u8| add(a, &rays.bt(b));
    fog.n = 0;
    // The ground's fog end (`w` is monotone, so the ground is one stretch): when w falls along the line,
    // the first fill after it (or 40); when it rises, its first byte (or 40).
    let dir = rays.fog().dir;
    let mut gb = if dir == 0xFF { COLS } else { 0 };
    let mut n = 0;
    let mut b = 0u8;
    let mut w0 = -a[1];
    // The previous run's end point, reused as the next run's start.
    let mut start: Option<(i32, i32)> = None;
    while b < COLS {
        let c = class(rays, w0);
        if c < 2 {
            // The longest stretch of the same fill: m bytes, w the last one's depth.
            let (mut m, mut w) = (1u8, w0);
            for k in (0..6).rev() {
                if b + m + (1 << k) <= COLS {
                    let wn = w - rays.dw[k];
                    if class(rays, wn) == c {
                        m += 1 << k;
                        w = wn;
                    }
                }
            }
            if dir == 1 && ground_last(out, n) {
                gb = b;
            }
            put_fill(out, &mut n, m, if c == 0 { SKY_BYTE } else { haze });
            b += m;
            w0 = w - rays.dw[0];
            start = None;
            continue;
        }
        // The longest power-of-two run that stays on visible ground within the error budget.
        let mut k = 5;
        while (1u8 << k) > COLS - b {
            k -= 1;
        }
        while k > 0 && w0 < rays.t[k] {
            k -= 1;
        }
        // A one-byte run whose far end is already haze: its step (so its level) is unknown, and it is the
        // last ground byte before the haze, so it joins the haze.
        if k == 0 && w0 < rays.t[0] {
            if dir == 1 && ground_last(out, n) {
                gb = b;
            }
            put_fill(out, &mut n, 1, haze);
            b += 1;
            w0 -= rays.dw[0];
            start = None;
            continue;
        }
        if dir == 0xFF && !ground_last(out, n) {
            gb = b;
        }
        let len = 1u8 << k;
        let w1 = w0 - rays.dw[k];
        let p0 = match start {
            Some(p) => p,
            None => {
                let d = ray_at(b);
                ground_offset(rays.hc, d[0], d[2], w0)
            }
        };
        let d = ray_at(b + len);
        let p1 = ground_offset(rays.hc, d[0], d[2], w1);
        let (du, dv) = ((p1.0 - p0.0) >> k, (p1.1 - p0.1) >> k);
        // (Steeper than the last level allows: the last level still, rather than a run of haze.)
        let level = level_for(du, dv).unwrap_or(LEVELS - 1);
        out[n] = Span {
            len,
            flags: level,
            u: ((cam.u + p0.0) >> level) + ((MIP_OFFSET[level as usize] as i32) << 8),
            v: ((cam.v + p0.1) >> level) + ((MIP_OFFSET[level as usize] as i32) << 8),
            du: (du >> level) as i16,
            dv: (dv >> level) as i16,
        };
        n += 1;
        start = Some(p1);
        b += len;
        w0 = w1;
    }
    if dir == 1 && ground_last(out, n) {
        gb = COLS;
    }
    if dir != 0 {
        paint(dir, fog_at, gb, haze, fog);
    }
    out[n - 1].len -= 1;
    if out[n - 1].len == 0 {
        n -= 1;
    }
    n
}

/// Whether the last of the `n` runs is ground.
fn ground_last(out: &[Span; MAX_SPANS], n: usize) -> bool {
    n > 0 && out[n - 1].fill().is_none()
}

/// The first `k` pixels of a byte (`LMASK[k]`) and the last `k` (`RMASK[k]`).
const LMASK: [u8; 5] = [0x00, 0xC0, 0xF0, 0xFC, 0xFF];
const RMASK: [u8; 5] = [0x00, 0x03, 0x0F, 0x3F, 0xFF];

/// A painted byte: `keep` its ground pixels, `sky` its sky pixels, the rest haze.
fn patch(fog: &mut Patches, e: u8, keep: u8, sky: u8, haze: u8) {
    if (fog.n as usize) < MAX_PATCHES {
        fog.p[fog.n as usize] = [e, keep, haze & !(keep | sky) | SKY_BYTE & sky];
        fog.n += 1;
    }
}

/// The line's painted bytes from its crossings `at` ([`FogArgs`]) and the ground's fog end `gb`: the
/// ground bytes the fog line covers part or all of, and the byte the horizon splits (when the runs did
/// not draw it as ground; the hidden byte 39 never).
fn paint(dir: u8, at: [i32; 2], gb: u8, haze: u8, fog: &mut Patches) {
    let px = |p: i32| (p >> 16).clamp(0, 4 * COLS as i32) as u8;
    let (g, s) = (px(at[0]), px(at[1]));
    let c4 = |x: i32| x.clamp(0, 4) as usize;
    let hs = s >> 2;
    if dir == 1 {
        for e in g >> 2..gb.min(DRAWN) {
            let x = 4 * e as i32;
            patch(fog, e, LMASK[c4(g as i32 - x)], RMASK[c4(x + 4 - s as i32)], haze);
        }
        if s & 3 != 0 && hs >= gb && hs < DRAWN {
            patch(fog, hs, 0, RMASK[4 - (s & 3) as usize], haze);
        }
    } else {
        for e in gb..((g + 3) >> 2).min(DRAWN) {
            let x = 4 * e as i32;
            patch(fog, e, RMASK[c4(x + 4 - g as i32)], LMASK[c4(s as i32 - x)], haze);
        }
        if s & 3 != 0 && hs < gb && hs < DRAWN {
            patch(fog, hs, 0, LMASK[(s & 3) as usize], haze);
        }
    }
}

/// REU bank of tile `(tx, ty)` (mod 16) of mip `level`; off the map, the sea.
pub fn tile_bank(level: u8, tx: u8, ty: u8) -> u8 {
    let tiles = (MAP_TILES >> level).max(1);
    let (tx, ty) = (tx & 15, ty & 15);
    if tx < tiles && ty < tiles { crate::format::LEVEL_BANK[level as usize] + ty * tiles + tx } else { crate::format::SEA_BANK }
}

/// The 256-entry table `asm/floor.S` looks banks up in: entry `ty·16 + tx`.
pub fn tile_table(level: u8) -> [u8; 256] {
    core::array::from_fn(|i| tile_bank(level, i as u8 & 15, (i >> 4) as u8))
}

/// REU address of a level's texel at 16.8 coordinates.
pub fn texel_addr(level: u8, u: i32, v: i32) -> u32 {
    let bank = tile_bank(level, (u >> 16) as u8, (v >> 16) as u8);
    (bank as u32) << 16 | ((v >> 8) as u32 & 0xFF) << 8 | ((u >> 8) as u32 & 0xFF)
}

/// What a run costs the kernel (for profiles): DMAs and register writes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LineCost {
    pub dmas: u32,
    pub io_writes: u32,
}

/// Draw one line's runs into the [`DRAWN`] bytes at `buf` (the reference for `asm/floor.S` `floor_line`):
/// fills by CPU stores; ground by one REU DMA per byte, the C64 address auto-incrementing within a run.
/// REU address registers: mid and bank when they change (`shadow`), lo always and last (the C64U
/// reloads all three from the last written values on any address write). `$DF07/$DF08 = 1` and
/// `$DF0A = 0` are set once per frame by [`floor_begin`].
pub fn draw_line<H: Hw + ?Sized>(hw: &mut H, spans: &[Span], fog: &Patches, buf: u16, shadow: &mut [u8; 2]) -> LineCost {
    let mut cost = LineCost::default();
    let mut x = 0u16;
    for s in spans {
        match s.fill() {
            Some(byte) => {
                for i in 0..s.len as u16 {
                    hw.poke(buf + x + i, byte);
                }
            }
            None => {
                let at = buf + x;
                hw.poke(reu::C64_LO, at as u8);
                hw.poke(reu::C64_HI, (at >> 8) as u8);
                cost.io_writes += 2;
                let (mut u, mut v) = (s.u, s.v);
                let level = s.level();
                for _ in 0..s.len {
                    let addr = texel_addr(level, u, v);
                    let (lo, mid, bank) = (addr as u8, (addr >> 8) as u8, (addr >> 16) as u8);
                    if shadow[0] != mid {
                        hw.poke(reu::REU_MID, mid);
                        shadow[0] = mid;
                        cost.io_writes += 1;
                    }
                    if shadow[1] != bank {
                        hw.poke(reu::REU_BANK, bank);
                        shadow[1] = bank;
                        cost.io_writes += 1;
                    }
                    hw.poke(reu::REU_LO, lo);
                    hw.reu_exec(reu::FETCH);
                    cost.io_writes += 2;
                    cost.dmas += 1;
                    u = u.wrapping_add(s.du as i32);
                    v = v.wrapping_add(s.dv as i32);
                }
            }
        }
        x += s.len as u16;
    }
    debug_assert_eq!(x, DRAWN as u16);
    // The fog line over it.
    for &[at, keep, bits] in &fog.p[..fog.n as usize] {
        let a = buf + at as u16;
        let b = hw.peek(a);
        hw.poke(a, b & keep | bits);
    }
    cost
}

/// Once a frame before any [`draw_line`]: one-byte transfers, no fixed address, and a known REU address
/// in the shadows.
pub fn floor_begin<H: Hw + ?Sized>(hw: &mut H) -> [u8; 2] {
    hw.poke(reu::LEN_LO, 1);
    hw.poke(reu::LEN_HI, 0);
    hw.poke(reu::CONTROL, 0);
    hw.poke(reu::REU_LO, 0);
    hw.poke(reu::REU_MID, 0);
    hw.poke(reu::REU_BANK, 0);
    [0, 0]
}

/// Copy a drawn line ([`DRAWN`] bytes at `buf`) into band line `line` (0..7) of the 320-byte cell-row stage,
/// and into the next line too when `doubled`.
pub fn copy_line<H: Hw + ?Sized>(hw: &mut H, buf: u16, stage: u16, line: u8, doubled: bool) {
    for col in 0..DRAWN as u16 {
        let b = hw.peek(buf + col);
        hw.poke(stage + col * 8 + line as u16, b);
        if doubled {
            hw.poke(stage + col * 8 + line as u16 + 1, b);
        }
    }
}

/// `a · b` for a small non-negative `b` (bytes along a line), exactly; kept here so the C64 does not pull
/// in a general 32-bit multiply.
#[inline]
pub fn mul_small(a: i32, b: u8) -> i32 {
    let hi = mul((a >> 16) as i16, b as i16);
    let lo = mulsu(b as i16, a as u16);
    (hi << 16).wrapping_add(lo)
}

#[cfg(test)]
#[test]
fn rays_divisors_reduced_exactly() {
    // Every Q14 rotation entry, and every altitude: the reduced forms give the original quotients.
    for c in -16384..=16384i32 {
        assert_eq!(div8(c * (256 * 4 / 16), FX5), c * (256 * 4) / FX, "bb {c}");
        assert_eq!(div8(-(c * 256), FY as u8), -(c * 256) / FY, "ay {c}");
        assert_eq!(div8(c * (-78 * 256 / 16), FX5), c * (-78 * 256) / FX, "a0 right {c}");
        assert_eq!(div8(c * (CY_HALF * 128), FY as u8), c * (CY_HALF * 128) / FY, "a0 up {c}");
    }
    for h in 0..=u16::MAX {
        assert_eq!(div8((h as i32) << 8, FX5), ((h as i32) << 14) / HAZE_TEXELS, "w_haze {h}");
    }
}

#[cfg(test)]
#[test]
fn scale_h_is_the_i64_shifted_product() {
    let mut seed = 0x2545_F491u32;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed
    };
    let reference = |c: i32, h: u16, l: u32| ((c as i64 * h as i64) >> l) as i32;
    let edges = [0i32, 1, -1, 0xFFFF, 0x1_0000, -0x1_0000, -0x1_0001, 0x7F_FFFF, -0x80_0000, 10_400_000, -10_400_000, i32::MAX, i32::MIN];
    // Every altitude (with its own shift, as `Rays::new` takes it) for the edges and some random c.
    let mut cs: Vec<i32> = edges.to_vec();
    cs.extend((0..40).map(|_| rnd() as i32 >> (rnd() % 8)));
    for &c in &cs {
        for h in 0..=u16::MAX {
            let l = 16 - h.leading_zeros();
            assert_eq!(scale_h(c, h, l), reference(c, h, l), "c {c} h {h}");
        }
    }
    // Random c, h and every shift.
    for _ in 0..200_000 {
        let (c, h, l) = (rnd() as i32 >> (rnd() % 12), rnd() as u16, rnd() % 17);
        assert_eq!(scale_h(c, h, l), reference(c, h, l), "c {c} h {h} l {l}");
    }
}
