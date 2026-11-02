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
//! Ground runs have a power-of-two length, exact texel positions at both ends (one reciprocal and three
//! multiplies), and a constant step per byte between them (a shift). A run is as long as the affine
//! error allows (from the change of `w` across it alone, so no division is wasted on rejected runs): a
//! level line (no roll) is one run. Each run reads the mip level its step needs.
//!
//! [`make_line`] is shared by the host and the C64 (same bits); [`draw_line`] is the reference for
//! `target-c64/asm/floor.S`.

use crate::fixed::{mul, mulsu, recip, umul};
use crate::hw::{Hw, reu};

/// Horizontal and vertical focal lengths in multicolor pixels: about 90° across, and square pixels on
/// screen (a multicolor pixel is ~1.66 times as wide as it is tall).
pub const FX: i32 = 80;
pub const FY: i32 = 133;
/// View lines (17 cell rows) above the cockpit, and the optical axis' row.
pub const VIEW_LINES: u16 = 136;
pub const VIEW_BANDS: u16 = VIEW_LINES / 8;
pub const CY_HALF: i32 = 135; // 2 · (VIEW_LINES / 2) − 1: line y's centre is at (CY_HALF − 2y)/2 above the axis
/// Bytes per line.
pub const COLS: u8 = 40;

/// Map size in texels (level 0) and tiles.
pub const MAP_TEXELS: u32 = 2048;
pub const MAP_TILES: u8 = (MAP_TEXELS / 256) as u8;
pub const LEVELS: u8 = 4;

/// Fill bytes: sky (code 1 in every pixel), haze (codes 1 and 0 alternating).
pub const SKY_BYTE: u8 = 0x55;
pub const HAZE_BYTE: u8 = 0x44;
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

/// Per-frame ray set-up: line `y`'s ray at byte `b` is `a + y·ay + b·bb` (Q22 world directions).
#[derive(Clone, Copy, Debug)]
pub struct Rays {
    pub a0: [i32; 3],
    pub ay: [i32; 3],
    pub bb: [i32; 3],
    /// A run of `2^k` bytes is short enough when `limit[k] ≤ w0 + w1` (see [`make_line`]).
    pub limit: [i32; 6],
    /// `w` below which the ground is haze.
    pub w_haze: i32,
}

/// The error budget: a run of `len` bytes is accepted when `ERR · len² · |Δw per byte| ≤ w0 + w1`.
/// The affine error is about `|Δf|·|Δw| / (2(w0 + w1))` with `|Δf| ≤ 1.5·len` level texels, so `ERR = 2`
/// keeps it under about 0.4 texel.
pub const ERR: i32 = 2;

impl Rays {
    pub fn new(cam: &FloorCam) -> Self {
        let col = |j: usize| [cam.r[0][j], cam.r[1][j], cam.r[2][j]];
        let (right, up, fwd) = (col(0), col(1), col(2));
        // Q14 → Q22 is ×256. Byte step: 4 pixels / FX. Byte 0's centre: pixel 2, (2 − 80)/80 = −78/80.
        // Line 0's centre: (CY_HALF/2)/FY above the axis.
        let bb = right.map(|c| c * (256 * 4) / FX);
        let ay = up.map(|c| -(c * 256) / FY);
        let a0 = core::array::from_fn(|k| fwd[k] * 256 + right[k] * (-78 * 256) / FX + up[k] * (CY_HALF * 128) / FY);
        // |Δw per byte| ≤ 2^22 / 20; ERR · 4^5 · that stays within i32.
        let limit = core::array::from_fn(|k| ERR * (bb[1].abs() << (2 * k)));
        // A ray of length ~1 (Q22 = 2^22) reaches HAZE_TEXELS when w = H/HAZE_TEXELS (H in 8.8 texels).
        let w_haze = ((cam.h as i32) << 14) / HAZE_TEXELS;
        Self { a0, ay, bb, limit, w_haze }
    }

    /// Line `y`'s ray at byte 0 (the frame loop steps it by [`Rays::ay`] instead).
    pub fn line(&self, y: u16) -> [i32; 3] {
        core::array::from_fn(|k| self.a0[k] + mul_small(self.ay[k], y as u8))
    }

    /// `b · bb`: byte `b`'s ray is the line's ray at byte 0 plus this. The C64 keeps it as a table
    /// (`layout::BT`, field-major: row `4c + i` holds byte i of component c for b = 0..40), built by
    /// adding `bb` up, which gives the same values.
    pub fn bt(&self, b: u8) -> [i32; 3] {
        self.bb.map(|c| mul_small(c, b))
    }
}

/// `asm/spans.S`'s arguments (`layout::RAYS`): written once a frame, `a` (the line's ray at byte 0)
/// before each line; the kernel leaves the run count in `n`. `Rays::bt` goes to `layout::BT`.
#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct SpanArgs {
    pub limit: [i32; 6],
    pub w_haze: i32,
    pub cam_u: i32,
    pub cam_v: i32,
    pub h: u16,
    pub a: [i32; 3],
    pub n: u8,
}
const _: () = assert!(core::mem::size_of::<SpanArgs>() == 51);

impl SpanArgs {
    pub fn new(cam: &FloorCam, rays: &Rays) -> Self {
        Self { limit: rays.limit, w_haze: rays.w_haze, cam_u: cam.u, cam_v: cam.v, h: cam.h, a: rays.a0, n: 0 }
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
        self.flags & 3
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

/// The ground point (16.8 texels, level 0) of ray `d` (`w = −d.y`, at least `w_haze`), relative to the
/// camera: `H · (d.x, d.z) / w`. The ray's components are used as 16-bit values (`>> 8`: at most
/// ~6·10⁻⁵ of the distance off), `H / w` as a 16-bit factor whose exponent is rounded to whole bytes
/// (at least 9 significant bits: 0.2%), so the 6502 needs one normalisation, three 16×16 multiplies and
/// whole-byte shifts. Both errors stay far below a texel of the mip level read at that distance.
pub fn ground_offset(h: u16, d: &[i32; 3]) -> (i32, i32) {
    let x = (d[0] >> 8) as i16;
    let w = ((-d[1]) >> 8) as u16;
    let z = (d[2] >> 8) as i16;
    let (m, k) = norm16(w);
    // h / w ≈ (h · recip(m)) · 2^(−26 − k), recip(m) ≈ 2^26 / m.
    let hr = umul(h, recip(m));
    // Shift h·r right by s ≥ 16 − (its spare top bits) so it fits 16 bits and the remaining exponent
    // e = s − 26 − k is a whole number of bytes.
    let mut s = 0i8;
    while (hr >> s) >= 0x1_0000 {
        s += 1;
    }
    while (s - 26 - k).rem_euclid(8) != 0 {
        s += 1;
    }
    let p = (hr >> s) as u16;
    let e = s - 26 - k;
    let off = |n: i16| {
        let v = mulsu(n, p);
        if e >= 0 { v << e } else { v >> -e }
    };
    (off(x), off(z))
}

/// The mip level whose step stays within [`MAX_STEP`], or `None` beyond the last.
fn level_for(du: i32, dv: i32) -> Option<u8> {
    let s = du.abs().max(dv.abs());
    (0..LEVELS).find(|&k| s >> k <= MAX_STEP)
}

fn add(a: &[i32; 3], b: &[i32; 3]) -> [i32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// Append `len` bytes of `fill`, extending the last run when it is the same fill.
fn put_fill(out: &mut [Span; MAX_SPANS], n: &mut usize, len: u8, fill: u8) {
    if *n > 0 && out[*n - 1].fill() == Some(fill) {
        out[*n - 1].len += len;
    } else {
        out[*n] = Span::filled(len, fill);
        *n += 1;
    }
}

/// One line's runs into `out` from its ray at byte 0 (`Rays::line`); returns how many. The runs cover
/// the 40 bytes in order. Sky (`w ≤ 0`) and haze bytes become fills; ground runs are as long as
/// [`Rays::limit`] allows (a power of two, so the step is a shift), with exact end points.
pub fn make_line(cam: &FloorCam, rays: &Rays, a: &[i32; 3], out: &mut [Span; MAX_SPANS]) -> usize {
    let w_at = |b: u8| -(a[1] + mul_small(rays.bb[1], b));
    let ray_at = |b: u8| add(a, &rays.bt(b));
    let mut n = 0;
    let mut b = 0u8;
    // The previous run's end point, reused as the next run's start.
    let mut start: Option<(i32, i32)> = None;
    while b < COLS {
        let w0 = w_at(b);
        if w0 < rays.w_haze {
            put_fill(out, &mut n, 1, if w0 <= 0 { SKY_BYTE } else { HAZE_BYTE });
            b += 1;
            start = None;
            continue;
        }
        // The longest power-of-two run that stays on visible ground within the error budget.
        let mut k = 5;
        while (1u8 << k) > COLS - b {
            k -= 1;
        }
        while k > 0 {
            let w1 = w_at(b + (1 << k));
            if w1 >= rays.w_haze && rays.limit[k] <= w0 + w1 {
                break;
            }
            k -= 1;
        }
        let len = 1u8 << k;
        // A one-byte run whose far end is already haze: its step (so its level) is unknown, and it is the
        // last ground byte before the haze, so it joins the haze.
        if w_at(b + len) < rays.w_haze {
            put_fill(out, &mut n, 1, HAZE_BYTE);
            b += 1;
            start = None;
            continue;
        }
        let p0 = match start {
            Some(p) => p,
            None => ground_offset(cam.h, &ray_at(b)),
        };
        let p1 = ground_offset(cam.h, &ray_at(b + len));
        let (du, dv) = ((p1.0 - p0.0) >> k, (p1.1 - p0.1) >> k);
        match level_for(du, dv) {
            Some(level) => {
                out[n] = Span {
                    len,
                    flags: level,
                    u: (cam.u + p0.0) >> level,
                    v: (cam.v + p0.1) >> level,
                    du: (du >> level) as i16,
                    dv: (dv >> level) as i16,
                };
                n += 1;
                start = Some(p1);
            }
            None => {
                put_fill(out, &mut n, len, HAZE_BYTE);
                start = None;
            }
        }
        b += len;
    }
    n
}

/// REU bank of tile `(tx, ty)` (mod 16) of mip `level`; off the map, the sea.
pub fn tile_bank(level: u8, tx: u8, ty: u8) -> u8 {
    let tiles = MAP_TILES >> level;
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

/// Draw one line's runs into the 40 bytes at `buf` (the reference for `asm/floor.S` `floor_line`):
/// fills by CPU stores; ground by one REU DMA per byte, the C64 address auto-incrementing within a run.
/// REU address registers: mid and bank when they change (`shadow`), lo always and last (the C64U
/// reloads all three from the last written values on any address write). `$DF07/$DF08 = 1` and
/// `$DF0A = 0` are set once per frame by [`floor_begin`].
pub fn draw_line<H: Hw + ?Sized>(hw: &mut H, spans: &[Span], buf: u16, shadow: &mut [u8; 2]) -> LineCost {
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
    debug_assert_eq!(x, COLS as u16);
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

/// Copy a drawn line (40 bytes at `buf`) into band line `line` (0..7) of the 320-byte cell-row stage,
/// and into the next line too when `doubled`.
pub fn copy_line<H: Hw + ?Sized>(hw: &mut H, buf: u16, stage: u16, line: u8, doubled: bool) {
    for col in 0..COLS as u16 {
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
