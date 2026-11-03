//! Effect sprites (`no_std`, shared): explosions, hits, splashes, smoke, muzzle flashes and burning
//! wrecks, pre-rendered from Ludo.ai animations (`Assets/Effects`, `tools/effects.py`) and baked into the
//! REU (`effects_bake`) at every height from [`MIN_LINES`] to [`MAX_LINES`] lines in steps of
//! [`STEP_LINES`], so the game picks the scale nearest an effect's projected size ([`scale_for`]).
//!
//! Each frame has a directory record at [`record`] in the sprite format (`format::frame`, `FRAME_RECORD`
//! bytes): width, height, 4 phases (any x), band shift, bands, the phases' blocks (in
//! `format::EFFECTS_BLOCKS`, each inside one 512 KiB REU region) and the anchor (bottom centre: the
//! ground point; the muzzle flash's is its centre). To draw frame `f` of effect `e` at screen point
//! (`x`, `y`) (multicolour pixels, lines) with depth key `key`: fetch the record at
//! `record(e, scale_for(z, HEIGHT_Q4[e]), f)`; `left = x - AX`, `top = y - AY`; phase `p = left & 3`;
//! then `geom::push_sprite(w, left, top, H, phase[p].WB, phase[p].BLOCK, BAND_SHIFT, key)`.

use crate::format::{EFFECTS_DIR, FRAME_RECORD};

/// The effects, in directory order.
pub mod effect {
    pub const EXPLOSION: u8 = 0;
    pub const IMPACT: u8 = 1;
    pub const SPLASH: u8 = 2;
    pub const SMOKE: u8 = 3;
    pub const MUZZLE: u8 = 4;
    pub const WRECK_FIRE: u8 = 5;
}
pub const EFFECTS: usize = 6;
pub const NAMES: [&str; EFFECTS] = ["explosion", "impact", "splash", "smoke", "muzzle", "wreck_fire"];
/// Frames per effect (the directory has [`MAX_FRAMES`] slots for each; the rest are zero). `SMOKE` and
/// `WRECK_FIRE` loop; the others play once.
pub const FRAMES: [u8; EFFECTS] = [8, 6, 6, 8, 4, 8];
pub const MAX_FRAMES: usize = 8;
pub const LOOPS: [bool; EFFECTS] = [false, false, false, true, false, true];
/// The effects' sizes in the world (the height of the whole frame, 1/16 texel): what [`scale_for`] takes.
/// A vehicle is about 0.7 texel tall.
pub const HEIGHT_Q4: [u16; EFFECTS] = [48, 16, 28, 72, 8, 40];

/// Scales: frames `MIN_LINES`, `MIN_LINES + STEP_LINES`, .. `MAX_LINES` lines tall.
pub const MIN_LINES: u16 = 4;
pub const STEP_LINES: u16 = 2;
pub const SCALES: usize = 31;
pub const MAX_LINES: u16 = MIN_LINES + STEP_LINES * (SCALES as u16 - 1);
const _: () = assert!(MAX_LINES == 64);
const _: () = {
    let mut i = 0;
    while i < EFFECTS {
        assert!(FRAMES[i] as usize <= MAX_FRAMES);
        i += 1;
    }
};
/// The directory fits its bank.
pub const DIR_LEN: u32 = (EFFECTS * SCALES * MAX_FRAMES) as u32 * FRAME_RECORD;
const _: () = assert!(DIR_LEN <= 0x10000);

/// Lines of scale `s`.
pub const fn lines(scale: u8) -> u16 {
    MIN_LINES + STEP_LINES * scale as u16
}

/// REU address of the directory record of `frame` of `effect` at `scale`.
pub const fn record(effect: u8, scale: u8, frame: u8) -> u32 {
    EFFECTS_DIR + ((effect as u32 * SCALES as u32 + scale as u32) * MAX_FRAMES as u32 + frame as u32) * FRAME_RECORD
}

/// The scale whose height is nearest an effect `height_q4` tall (1/16 texel) seen at depth `depth_q4`
/// (camera Z, 1/16 texel): it spans `height · 133 / z` lines (`geom`'s projection), clamped to the
/// scales there are.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.fx"))]
pub fn scale_for(depth_q4: i32, height_q4: u16) -> u8 {
    if depth_q4 <= 0 {
        return SCALES as u8 - 1;
    }
    // The first scale whose height, half a step up, reaches 133 h / z: no divide (z × lines against 133 h).
    let (h, z) = (height_q4 as u32 * 133, depth_q4 as u32);
    let mut s = 0u8;
    while (s as usize) < SCALES - 1 && (lines(s) as u32 + STEP_LINES as u32 / 2) * z <= h {
        s += 1;
    }
    s
}
