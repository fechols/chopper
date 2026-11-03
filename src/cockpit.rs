//! The cockpit's instruments: a helicopter six-pack (airspeed, attitude, altimeter, a caution-lamp
//! strip, the rotor/engine dual tach, heading, vertical speed) in the panel under the view (cell rows
//! 17-24). Every reading of every gauge is baked ahead (`instruments`, host) as a whole opaque face,
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

/// The gauges, left to right.
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

/// The panel: cell rows `PANEL_ROW..ROWS`; the dials' cell rows `ROW..ROW + TALL`.
pub const PANEL_ROW: u16 = 17;
pub const PANEL_ROWS: u16 = ROWS - PANEL_ROW;
pub const ROW: u16 = 18;
pub const TALL: u16 = 6;
/// Each gauge's first cell column and width in cells.
pub const COL: [u8; gauge::COUNT] = [0, 6, 12, 18, 22, 28, 34];
pub const WIDE: [u8; gauge::COUNT] = [6, 6, 6, 4, 6, 6, 6];
/// Frames baked per gauge (`readings` returns `0..FRAMES[g]`).
pub const FRAMES: [u16; gauge::COUNT] = [64, ATT_ROLLS * ATT_PITCHES, 3 * 64, 8, TACH_FRAMES, 128, 41];
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
    /// The autopilot flying (off the pad).
    pub const AP: u8 = 4;
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
        assert!(FRAMES[g] <= 255 && (WIDE[g] as u32 * 8 * TALL as u32) <= 1 << FRAME_STRIDE_SHIFT);
        assert!(COL[g] as u16 + WIDE[g] as u16 <= COLS);
        g += 1;
    }
    assert!(ROW + TALL <= ROWS && ROW >= PANEL_ROW);
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

const fn dests() -> [u16; gauge::COUNT] {
    let mut d = [0u16; gauge::COUNT];
    let mut g = 0;
    while g < gauge::COUNT {
        d[g] = BUFFERS[0].bitmap + ROW * ROW_STRIDE + COL[g] as u16 * 8;
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
        lamps |= lamp::AP;
        if alt < LOW_ALT {
            lamps |= lamp::LOW_ALT;
        }
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
    for _ in 0..TALL {
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
