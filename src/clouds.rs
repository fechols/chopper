//! Clouds on the horizon (`no_std`, shared): a ring of flat, pre-rendered cloud sprites at bearings around
//! the sky, above the horizon, turning slowly with the clock (the wind). They are at infinity: a cloud is
//! a direction, so it moves on the screen only as the camera turns, pitches and rolls, and as the ring
//! drifts. Each one rides the effects' path: [`place`] appends it to `combat::FXDYN` (after the effects,
//! so a full list cuts the clouds first), and `geom::push_dynamic` pushes it as a sprite in the farthest
//! depth bucket, so the hazed mountains are drawn over it.
//!
//! A sprite can't rotate, so every shape is baked at every roll angle the chopper reaches ([`ANGLES`]
//! steps of [`STEP_DEG`]°, ±30°). [`place`] lists the level frame; `geom::push_dynamic` turns it to the
//! frame [`angle`] picks for the horizon's tilt (it has the camera's matrix, and the room).
//!
//! They are thunderheads in the distance: towers of round billows, dense blue (the darkest the view has:
//! `format::COLORS` is green, light blue, brown, blue, with no white) on the light-blue sky, transparent
//! around them, solid along the storm's base and dithered lighter up the tower, upright at every angle.
//! (Their round outline, not their shade, tells them from the hazed mountains' 1-3 checker.)

use crate::format::{CLOUDS_DIR, FRAME_RECORD};
use crate::hw::Hw;

/// The cloud shapes, the roll angles each is baked at (index [`LEVEL`] is level; index `a` turns it
/// `(a − LEVEL)·STEP_DEG` degrees clockwise on the screen), and the ring.
pub const SHAPES: usize = 4;
pub const ANGLES: usize = 31;
pub const STEP_DEG: i32 = 2;
pub const LEVEL: u8 = (ANGLES / 2) as u8;
pub const CLOUDS: usize = 10;
/// Each cloud: its bearing (a 256th of a turn, 0 north, 64 east), its shape, and how high its base
/// stands over the horizon, `e`: the up component of a direction 4096 long horizontally is `4·e`, so the
/// base is about `e / 7.7` lines above the horizon ahead (12-22 lines here), higher toward the sides.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.fx.clouds"))]
pub static RING: [[u8; 3]; CLOUDS] = [
    [3, 2, 130],
    [27, 0, 165],
    [52, 3, 95],
    [80, 1, 150],
    [104, 2, 110],
    [131, 0, 170],
    [157, 3, 100],
    [183, 1, 140],
    [206, 2, 160],
    [234, 0, 120],
];
/// The ring turns one 65536th of a turn every `1 << DRIFT_SHIFT` ticks of the u32 clock: once in
/// 262,144 ticks (about 73 minutes).
pub const DRIFT_SHIFT: u32 = 2;
/// Clouds are the farthest thing drawn (`geom::depth_key`'s last bucket). The key also tells
/// `geom::push_dynamic` an entry is a cloud (FXDYN's 8 bytes have no spare bit: the record is a full REU
/// address), so it is reserved: no static item reaches it (`geom`'s assert on `depth_key`) and the
/// effects stop short of the fog's buckets (`combat::effects`).
pub const KEY: u8 = 255;
/// `tan` of the odd degrees 1, 3, .. 29 (Q12): the boundaries between the angle steps.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.dyn.clouds"))]
static TAN: [u16; LEVEL as usize] = [71, 215, 358, 503, 649, 796, 946, 1098, 1252, 1410, 1572, 1739, 1910, 2087, 2270];
const _: () = assert!(STEP_DEG * LEVEL as i32 == 30 && ANGLES == 2 * LEVEL as usize + 1);

/// The anchor's reach: no baked frame extends more than `REACH_X` multicolour pixels left or right of its
/// anchor, `REACH_UP` lines above it or `REACH_DOWN` below it (checked by [`bake::install`]); [`place`]
/// culls by these.
pub const REACH_X: i16 = 40;
pub const REACH_UP: i16 = 64;
pub const REACH_DOWN: i16 = 24;

/// [`place`]'s first cull: a cloud is only drawn while its anchor is within the reach of the screen,
/// at most 1.5 half-widths across (`hud::screen`: 160 hires pixels a half-width, `2·REACH_X` past the
/// edge) and about 1 down or up (`CY + REACH_UP` over 133), so its direction (at least 4096 long) is
/// within 1/√(1 + 1.5² + 1²) ≈ 0.49 of the view axis: past `0.44·4096` deep (a margin for the rounding,
/// and the ring's tilt). (Checked against the full projection by a test.)
pub const CULL_Z: i16 = 1792;
const _: () = assert!(CULL_Z >= crate::hud::NEAR);

/// The REU address of `shape`'s frame record at angle `angle` (`format::frame`).
pub const fn record(shape: u8, angle: u8) -> u32 {
    CLOUDS_DIR + ((shape as u16 * ANGLES as u16 + angle as u16) * FRAME_RECORD as u16) as u32
}

/// The angle index matching the horizon's tilt on the screen. World up in camera space is `(ux, uy, uz)`
/// (Q14: `hud::Eye::m[1]`, `[4]`, `[7]`, `geom::Look::rt[0][1]`, `[1][1]`, `[2][1]`); the horizon (the
/// directions with no up component) runs down to the right by `ux / uy` lines a line across (the
/// screen's pixels are square), so its tilt is `atan(ux / uy)`, rounded to a step by counting the
/// odd-degree tangents below it (stopping at the first one above). (In the moving objects' overlay, for `geom::push_dynamic`.)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.dyn"))]
pub fn angle(ux: i16, uy: i16) -> u8 {
    let steps = if uy <= 0 {
        LEVEL
    } else {
        let x = (ux.unsigned_abs() as i32) << 12;
        // (Ascending: the count stops at the first boundary past the tilt.)
        TAN.iter().take_while(|&&t| crate::hud::mul16(uy, t as i16) < x).count() as u8
    };
    if ux < 0 { LEVEL - steps } else { LEVEL + steps }
}

/// Append the clouds in view to `combat::FXDYN` from entry `n` on (at most `FXDYN_MAX`), and return
/// the new count; `drift` is the ring's turn (`clock >> DRIFT_SHIFT`). In the effects' overlay, after
/// the effects.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.fx"))]
pub fn place<H: Hw + ?Sized>(hw: &mut H, eye: &crate::hud::Eye, drift: u16, mut n: u8) -> u8 {
    use crate::combat::{FXDYN, FXDYN_ENTRY, FXDYN_MAX};
    for c in RING.iter() {
        if n >= FXDYN_MAX {
            break;
        }
        let a = ((c[0] as u16) << 8).wrapping_add(drift);
        // A direction 4096 long horizontally (Q14 sine and cosine / 4), from the eye.
        let (s, co) = ((crate::camera::sin(a) >> 2) as i16, (crate::camera::cos(a) >> 2) as i16);
        let p = [eye.p[0].wrapping_add(s), eye.p[1].wrapping_add(c[2] as i16 * 4), eye.p[2].wrapping_add(co)];
        let cam = crate::hud::to_camera(eye, p);
        // (Most of the ring is off to the sides or behind: no projection, `screen`'s two divides.)
        if cam[2] < CULL_Z {
            continue;
        }
        let (x, y) = crate::hud::screen(cam);
        // (Anchored at the base's centre: on screen while within the frames' reach.)
        if !(-2 * REACH_X..320 + 2 * REACH_X).contains(&x) || !(-REACH_DOWN..crate::geom::LINES as i16 + REACH_UP).contains(&y) {
            continue;
        }
        let ([x0, x1], [y0, y1]) = ((x >> 1).to_le_bytes(), y.to_le_bytes());
        // (Level: `geom::push_dynamic` turns it to the horizon's tilt.)
        let [r0, r1, r2, _] = record(c[1], LEVEL).to_le_bytes();
        let at = FXDYN + 1 + n as u16 * FXDYN_ENTRY;
        for (k, b) in [x0, x1, y0, y1, r0, r1, r2, KEY].into_iter().enumerate() {
            hw.poke(at + k as u16, b);
        }
        n += 1;
    }
    n
}

/// The shapes' baker (host): `tools/clouds.py`'s code maps (thunderheads from Ludo.ai stills, at every
/// angle: `Assets/Clouds/baked/clouds.bin`), encoded like the effects (`effects_bake::encode`).
#[cfg(feature = "host")]
pub mod bake {
    use super::{ANGLES, REACH_DOWN, REACH_UP, REACH_X, SHAPES, record};
    use crate::encode::SpriteImage;
    use crate::format::{CLOUDS_BLOCKS, CLOUDS_END, FRAME_RECORD, REU_BANK, frame, phase};

    const BAKED: &[u8] = include_bytes!("../Assets/Clouds/baked/clouds.bin");

    /// Shape `s` at angle index `a`: the image and its anchor (the base's centre), from `BAKED`
    /// (`b"CLD1"`, shapes, angles, then per frame `w, h, ax, ay` and its pixels run-length coded as
    /// `tools/effects.py`'s: `(value << 5) | (run − 1)`, value 4 transparent).
    pub fn image(s: usize, a: usize) -> (SpriteImage, u8, u8) {
        let b = BAKED;
        assert!(&b[..4] == b"CLD1" && b[4] as usize == SHAPES && b[5] as usize == ANGLES, "clouds.bin: not {SHAPES} shapes at {ANGLES} angles (run tools/clouds.py)");
        let mut at = 6;
        for k in 0..=s * ANGLES + a {
            let [w, h, ax, ay] = b[at..at + 4].try_into().unwrap();
            at += 4;
            let mut img = SpriteImage::new(w as u16, h as u16);
            img.phases = 4;
            let mut i = 0;
            while i < img.pixels.len() {
                let (code, run) = (b[at] >> 5, (b[at] & 31) as usize + 1);
                at += 1;
                assert!(code <= 4 && i + run <= img.pixels.len(), "clouds.bin: bad run");
                img.pixels[i..i + run].fill(if code == 4 { None } else { Some(code) });
                i += run;
            }
            if k == s * ANGLES + a {
                let fits = ax as i16 <= REACH_X && w as i16 - ax as i16 <= REACH_X && ay as i16 <= REACH_UP && h as i16 - ay as i16 <= REACH_DOWN;
                assert!(fits, "cloud {s} angle {a}: {w}x{h} anchored at ({ax}, {ay}), past the reach");
                return (img, ax, ay);
            }
        }
        unreachable!()
    }

    /// Bake every shape at every angle into the image: records at [`record`], blocks from
    /// `format::CLOUDS_BLOCKS`. Returns the first free address.
    pub fn install(reu: &mut [u8]) -> u32 {
        let mut at = CLOUDS_BLOCKS;
        for s in 0..SHAPES {
            for a in 0..ANGLES {
                let (img, ax, ay) = image(s, a);
                let (shift, blocks) = crate::effects_bake::encode(&img);
                let nbands = if img.h <= 1 << shift { 1 } else { img.h.div_ceil(1 << shift) };
                let mut rec = [0u8; FRAME_RECORD as usize];
                rec[frame::W..frame::W + 2].copy_from_slice(&img.w.to_le_bytes());
                rec[frame::H..frame::H + 2].copy_from_slice(&img.h.to_le_bytes());
                rec[frame::PHASE_COUNT] = 4;
                rec[frame::BAND_SHIFT] = shift;
                rec[frame::BANDS] = nbands as u8;
                rec[frame::AX] = ax;
                rec[frame::AY] = ay;
                for (p, block) in blocks.iter().enumerate() {
                    let len = block.bytes.len() as u32;
                    // Keep every block inside one 512 KiB region: no DMA into it wraps.
                    if at / REU_BANK != (at + len - 1) / REU_BANK {
                        at = (at / REU_BANK + 1) * REU_BANK;
                    }
                    assert!(at + len <= CLOUDS_END, "clouds: blocks past CLOUDS_END");
                    reu[at as usize..][..len as usize].copy_from_slice(&block.bytes);
                    let r = frame::PHASES + p * frame::PHASE_LEN;
                    rec[r + phase::WB] = block.wb as u8;
                    rec[r + phase::BLOCK..r + phase::BLOCK + 3].copy_from_slice(&at.to_le_bytes()[..3]);
                    rec[r + phase::LEN..r + phase::LEN + 2].copy_from_slice(&(len.min(0xFFFF) as u16).to_le_bytes());
                    at += len;
                }
                reu[record(s as u8, a as u8) as usize..][..rec.len()].copy_from_slice(&rec);
            }
        }
        at
    }
}
