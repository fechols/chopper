//! The cockpit's instruments: a helicopter six-pack (airspeed, attitude, altimeter, the rotor/engine
//! dual tach, heading, vertical speed) under the caution lamps on the glareshield, in the panel under the
//! view (cell rows 16-24; everything that matters inside the screen's safe area, `layout::SAFE_X`/`SAFE_Y`,
//! which the damage shake never hides: cell columns 1-37, rows up to 23). Every reading of every gauge is baked ahead (`instruments`, host) as a whole opaque face,
//! cell-aligned and already in bitmap order, so drawing one is a DMA per cell row straight from the REU
//! into the bitmap, with no CPU copying; a gauge is redrawn only when its reading changes. Each cell's
//! colours (screen and colour RAM) are fixed per gauge and set once with the static panel.
//!
//! On the C64 this runs from its own code overlay (`.text.cockpit`, loaded in place at `$A000` with the
//! PRG, stashed to the REU at boot and fetched back each frame before the band state reuses `$A000`).

use crate::agent::{Agent, EYE};
use crate::format::COCKPIT;
use crate::hw::{self, Hw};
use crate::layout::{BUFFERS, CELLS, COLOR_RAM, COLS, ROW_STRIDE, ROWS};

/// The gauges, left to right (the caution lamps, `WARN`, are the strip above the dials).
pub mod gauge {
    pub const ASI: usize = 0;
    pub const ATT: usize = 1;
    pub const ALT: usize = 2;
    pub const WARN: usize = 3;
    pub const RPM: usize = 4;
    pub const HDG: usize = 5;
    pub const VSI: usize = 6;
    pub const COUNT: usize = 7;
    pub const NAMES: [&str; COUNT] = ["IAS", "ATT", "ALT", "CAUT", "RPM", "HDG", "VSI"];
}

/// The panel: cell rows `PANEL_ROW..ROWS`: the glareshield with the caution lamps (an annunciator
/// strip, at eye level as in a real cockpit), the dials, their placards, and a plain bottom row (only
/// partly visible, and not while the shake moves the picture up).
pub const PANEL_ROW: u16 = 16;
pub const PANEL_ROWS: u16 = ROWS - PANEL_ROW;
/// The dials' cell rows: `ROW..ROW + TALL`; the placards' row after them.
pub const ROW: u16 = 17;
pub const TALL: u16 = 6;
pub const PLACARD_ROW: u16 = ROW + TALL;
/// Each gauge's first cell row, height, first cell column and width in cells: six 6 × 6 dials in columns
/// 1-37 with a gap in the middle, the 19-column lamp strip (four lamps of 4 cells) centred over them.
pub const ROW_AT: [u8; gauge::COUNT] = [ROW as u8, ROW as u8, ROW as u8, PANEL_ROW as u8, ROW as u8, ROW as u8, ROW as u8];
pub const TALL_AT: [u8; gauge::COUNT] = [TALL as u8, TALL as u8, TALL as u8, 1, TALL as u8, TALL as u8, TALL as u8];
pub const COL: [u8; gauge::COUNT] = [1, 7, 13, 10, 20, 26, 32];
pub const WIDE: [u8; gauge::COUNT] = [6, 6, 6, 19, 6, 6, 6];
/// The lamps' cells in the strip: lamp `i` from column `5 i`, `LAMP_WIDE` wide.
pub const LAMP_WIDE: u8 = 4;
/// Frames baked per gauge (`readings` returns `0..FRAMES[g]`).
pub const FRAMES: [u16; gauge::COUNT] = [64, ATT_ROLLS * ATT_PITCHES, 3 * 64, 10, TACH_FRAMES, 128, 41];
/// Attitude frames: roll index × `ATT_PITCHES` + pitch index.
pub const ATT_ROLLS: u16 = 25;
pub const ATT_PITCHES: u16 = 10;
/// Dual-tach frames: one per 64 ticks of the start-up (about a second); the last is 100%, reached at
/// `agent::LIFT`, the sound's 1:20.
pub const TACH_SHIFT: u32 = 6;
pub const TACH_FRAMES: u16 = (crate::agent::LIFT >> TACH_SHIFT) as u16 + 1;
/// Each frame's REU block: `TALL` rows of `WIDE × 8` bytes, at a power-of-two stride.
pub const FRAME_STRIDE_SHIFT: u32 = 9;

/// The caution lamps (`WARN`'s frame bits).
pub mod lamp {
    /// Rotor below 100% (on the pad until the lift).
    pub const RPM: u8 = 1;
    /// Airborne under `LOW_ALT`.
    pub const LOW_ALT: u8 = 2;
    /// The autopilot flying (off the pad; dark while the player has the aircraft, `agent::MODE_PLAYER`).
    pub const AP: u8 = 4;
    /// Far out to sea (`agent::SEA_WARN` beyond the map): turn back, or the autopilot will. Two frames of
    /// its own, in place of the others (out there the rotor is up and the helicopter airborne): 8 alone,
    /// the player still flying; `SEA_AP` with AP, the autopilot turning back (past `agent::SEA_LIMIT`).
    /// The frames run to the HUD's shapes (`format::HUD_SHAPES`): no room for more.
    pub const SEA: u8 = 8;
    pub const SEA_AP: u8 = SEA + 1;
    /// The lamps lit in frame `k`.
    pub const fn lit(k: u8) -> u8 {
        if k == SEA_AP { SEA | AP } else { k }
    }
}
/// Altitude (8.8 texels over the pad's eye height) the low-altitude lamp comes on below.
pub const LOW_ALT: i32 = 6 * 256;

/// The static panel: the cockpit rows' bitmap, then their screen RAM, then their colour RAM.
pub const PANEL: u32 = COCKPIT;
pub const PANEL_BITMAP: u16 = PANEL_ROWS * ROW_STRIDE;
pub const PANEL_CELLS: u16 = PANEL_ROWS * COLS;
/// The gauges' frames, gauge after gauge from `GAUGES`.
pub const GAUGES: u32 = COCKPIT + 0x1000;
pub const BASE: [u32; gauge::COUNT] = bases();
pub const END: u32 = BASE[gauge::COUNT - 1] + ((FRAMES[gauge::COUNT - 1] as u32) << FRAME_STRIDE_SHIFT);

const fn bases() -> [u32; gauge::COUNT] {
    let mut b = [0u32; gauge::COUNT];
    let mut at = GAUGES;
    let mut g = 0;
    while g < gauge::COUNT {
        b[g] = at;
        at += (FRAMES[g] as u32) << FRAME_STRIDE_SHIFT;
        g += 1;
    }
    b
}

const _: () = assert!(PANEL + PANEL_BITMAP as u32 + 2 * PANEL_CELLS as u32 <= GAUGES);
const _: () = {
    let mut g = 0;
    while g < gauge::COUNT {
        assert!(FRAMES[g] <= 255 && (WIDE[g] as u32 * 8 * TALL_AT[g] as u32) <= 1 << FRAME_STRIDE_SHIFT);
        // Inside the safe area: columns 1-37 (hires 8-303), rows up to 23 (lines to 191).
        assert!(COL[g] >= 1 && COL[g] as u16 + WIDE[g] as u16 <= 38);
        assert!(ROW_AT[g] as u16 >= PANEL_ROW && ROW_AT[g] as u16 + TALL_AT[g] as u16 <= 24);
        g += 1;
    }
    assert!(PLACARD_ROW < 24 && (crate::layout::SAFE_X.start as u16) <= 8 && crate::layout::SAFE_X.end as u16 >= 38 * 8);
    assert!(crate::layout::SAFE_Y.end as u16 >= 24 * 8);
    assert!(PANEL_ROW * 8 == crate::floor::VIEW_LINES);
    assert!(CELLS == ROWS * COLS);
};

// The C64 reads these from the overlay (statics in its section: a `const` table would land in the
// resident `.rodata`).
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit.tables"))]
static DEST: [u16; gauge::COUNT] = dests();
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit.tables"))]
static BYTES: [u8; gauge::COUNT] = {
    let mut b = [0u8; gauge::COUNT];
    let mut g = 0;
    while g < gauge::COUNT {
        b[g] = WIDE[g] * 8;
        g += 1;
    }
    b
};
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit.tables"))]
static BASES: [u32; gauge::COUNT] = BASE;
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit.tables"))]
static TALLS: [u8; gauge::COUNT] = TALL_AT;

const fn dests() -> [u16; gauge::COUNT] {
    let mut d = [0u16; gauge::COUNT];
    let mut g = 0;
    while g < gauge::COUNT {
        d[g] = BUFFERS[0].bitmap + ROW_AT[g] as u16 * ROW_STRIDE + COL[g] as u16 * 8;
        g += 1;
    }
    d
}

/// Each gauge's frame for the agent's state (the host bakes frame `k` of gauge `g` to show what this
/// maps to `k`).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit"))]
pub fn readings(a: &Agent, out: &mut [u8; gauge::COUNT]) {
    use gauge::*;
    // Airspeed: 2.5 kt a frame (`vf` 8192 = 0.5 texel a tick, the cruise, reads 80 kt).
    out[ASI] = (a.vf >> 8).clamp(0, 63) as u8;
    // Attitude: roll in 2.8° steps (512 binary angle) over ±34°, pitch in 1.4° steps from −11° to +1.4°.
    let roll = ((a.roll + 256) >> 9).clamp(-12, 12) + 12;
    let pitch = ((a.pitch + 128) >> 8).clamp(-8, 1) + 8;
    out[ATT] = (roll * ATT_PITCHES as i16 + pitch) as u8;
    // Altimeter: over the pad, 1 texel = 31.25 ft; the long needle in 64ths of 1000 ft, the short one
    // in thousands (to 2999 ft).
    let alt = (a.h - EYE).max(0);
    out[ALT] = if alt >> 13 > 2 { 191 } else { ((alt >> 13) << 6 | (alt >> 7) & 63) as u8 };
    let mut lamps = 0;
    if a.t < crate::agent::LIFT {
        lamps |= lamp::RPM;
    }
    if a.grounded == 0 {
        if a.mode == crate::agent::MODE_AUTO {
            lamps |= lamp::AP;
        }
        if alt < LOW_ALT {
            lamps |= lamp::LOW_ALT;
        }
    }
    if a.off_map() > crate::agent::SEA_WARN {
        lamps = if lamps & lamp::AP != 0 { lamp::SEA_AP } else { lamp::SEA };
    }
    out[WARN] = lamps;
    // The dual tach follows the start-up sound to 100% at the lift.
    out[RPM] = (a.t >> TACH_SHIFT).min(TACH_FRAMES as u32 - 1) as u8;
    // Heading: 2.8° a frame.
    out[HDG] = (a.yaw >> 9) as u8;
    // Vertical speed: the climb rate in halves.
    out[VSI] = ((a.climb >> 1).clamp(-20, 20) + 20) as u8;
}

/// Draw frame `k` of gauge `g`: a DMA per cell row, REU → bitmap.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit"))]
pub fn draw<H: Hw + ?Sized>(hw: &mut H, g: usize, k: u8) {
    let g = g % gauge::COUNT;
    let n = BYTES[g] as u16;
    let mut src = BASES[g] + ((k as u32) << FRAME_STRIDE_SHIFT);
    let mut dst = DEST[g];
    for _ in 0..TALLS[g] {
        hw::fetch(hw, src, dst, n);
        src += n as u32;
        dst += ROW_STRIDE;
    }
}

/// Redraw the gauges whose readings changed since `last` (all of them when `last` is all `$FF`).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit"))]
pub fn update<H: Hw + ?Sized>(hw: &mut H, a: &Agent, last: &mut [u8; gauge::COUNT]) {
    let mut now = [0u8; gauge::COUNT];
    readings(a, &mut now);
    for g in 0..gauge::COUNT {
        if now[g] != last[g] {
            draw(hw, g, now[g]);
            last[g] = now[g];
        }
    }
}

/// The static panel (once, after the screen is set up): its bitmap, screen RAM and colour RAM.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit"))]
pub fn panel<H: Hw + ?Sized>(hw: &mut H) {
    let buf = BUFFERS[0];
    let cell = PANEL_ROW * COLS;
    hw::fetch(hw, PANEL, buf.bitmap + PANEL_ROW * ROW_STRIDE, PANEL_BITMAP);
    hw::fetch(hw, PANEL + PANEL_BITMAP as u32, buf.screen + cell, PANEL_CELLS);
    hw::fetch(hw, PANEL + PANEL_BITMAP as u32 + PANEL_CELLS as u32, COLOR_RAM + cell, PANEL_CELLS);
}
