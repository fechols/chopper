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
//! then `geom::push_sprite(w, left, top, H, phase[p].WB, phase[p].BLOCK, BAND_SHIFT, key, false)`.

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
/// Frames per effect (the directory has `1 << SLOT_SHIFT[e]` slots for each at every scale; the rest
/// are zero). `SMOKE` and `WRECK_FIRE` loop (a power of two each, so the frame wraps with a mask); the
/// others play once.
pub const FRAMES: [u8; EFFECTS] = [64, 6, 6, 128, 4, 8];
pub const SLOT_SHIFT: [u8; EFFECTS] = [6, 3, 3, 7, 3, 3];
pub const MAX_FRAMES: usize = 128;
/// Each effect's first directory slot (`record`): the slots of the effects before it, at every scale.
pub const BASE: [u16; EFFECTS] = {
    let mut b = [0u16; EFFECTS];
    let mut i = 1;
    while i < EFFECTS {
        b[i] = b[i - 1] + ((SCALES as u16) << SLOT_SHIFT[i - 1]);
        i += 1;
    }
    b
};
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
        assert!(FRAMES[i] as usize <= MAX_FRAMES && FRAMES[i] as u16 <= 1 << SLOT_SHIFT[i]);
        assert!(!LOOPS[i] || FRAMES[i] as u16 == 1 << SLOT_SHIFT[i]);
        i += 1;
    }
};
/// The directory: up to the blocks, inside one 512 KiB region (no record's DMA wraps).
pub const DIR_LEN: u32 = (BASE[EFFECTS - 1] as u32 + ((SCALES as u32) << SLOT_SHIFT[EFFECTS - 1])) * FRAME_RECORD;
const _: () = assert!(
    EFFECTS_DIR + DIR_LEN <= crate::format::EFFECTS_BLOCKS
        && EFFECTS_DIR / crate::format::REU_BANK == (EFFECTS_DIR + DIR_LEN - 1) / crate::format::REU_BANK
);

/// Lines of scale `s`.
pub const fn lines(scale: u8) -> u16 {
    MIN_LINES + STEP_LINES * scale as u16
}

/// REU address of the directory record of `frame` of `effect` at `scale`.
pub const fn record(effect: u8, scale: u8, frame: u8) -> u32 {
    let e = effect as usize;
    EFFECTS_DIR + (BASE[e] as u32 + ((scale as u32) << SLOT_SHIFT[e]) + frame as u32) * FRAME_RECORD
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
