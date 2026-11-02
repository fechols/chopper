//! Pre-rendered sprites in the band pipeline (`no_std`, the reference for `asm/geom.S` `sprite_band` and
//! `asm/blit.S` `draw_cols_asm`). A sprite is TALON HD's banded Run/Mix block (`crate::encode`,
//! `format::frame`): drawn one screen band at a time, each band fetching only the sprite band(s) it
//! crosses (band start, the on-screen columns' offsets, their records) and scattering them with
//! [`draw_cols`], so a sprite sorts with the triangles in painter's order.

use crate::format::{ITEM_MIX, MIX_LEN, RUN_HEADER};
use crate::hw::{self, Hw};
use crate::layout::{self, COLS};

/// `draw_cols`' arguments (`layout::JOB`, little-endian), TALON HD's.
pub mod job {
    /// u16: address of the first column's top cell.
    pub const DST: u16 = 0;
    /// u16: screen line of sprite line 0, mod 65536 (here: relative to the band's first line).
    pub const Y: u16 = 2;
    /// u16: first sprite line to draw; u16: end (exclusive).
    pub const TOP: u16 = 4;
    pub const BOT: u16 = 6;
    /// u16: RAM address of the first column record.
    pub const SRC: u16 = 8;
    /// u8: column records to draw (each 8 bytes right of the last).
    pub const NCOLS: u16 = 10;
}

fn peek16<H: Hw + ?Sized>(hw: &mut H, a: u16) -> u16 {
    hw.peek(a) as u16 | (hw.peek(a + 1) as u16) << 8
}

fn poke16<H: Hw + ?Sized>(hw: &mut H, a: u16, v: u16) {
    hw.poke(a, v as u8);
    hw.poke(a + 1, (v >> 8) as u8);
}

/// Reference sprite kernel, copied from `../talonhd/src/render.rs` `draw_cols` (e0e891c; the C64 runs
/// `asm/blit.S`): draw `NCOLS` consecutive column records from RAM at `SRC` into the column at `DST` and
/// the ones right of it, lines clipped to [`TOP`, `BOT`), sprite line `l` at line `Y + l`.
pub fn draw_cols<H: Hw + ?Sized>(hw: &mut H) {
    let mut dst = peek16(hw, layout::JOB + job::DST);
    let y = peek16(hw, layout::JOB + job::Y);
    let top = peek16(hw, layout::JOB + job::TOP);
    let bot = peek16(hw, layout::JOB + job::BOT);
    let mut src = peek16(hw, layout::JOB + job::SRC);
    let ncols = hw.peek(layout::JOB + job::NCOLS);
    for _ in 0..ncols {
        let items = hw.peek(src);
        src += 1;
        for _ in 0..items {
            let b0 = hw.peek(src);
            let b1 = hw.peek(src + 1);
            let line = b0 as u16 | ((b1 & 1) as u16) << 8;
            if b1 & ITEM_MIX != 0 {
                if line >= top && line < bot {
                    let a = dst.wrapping_add(layout::line_offset(y.wrapping_add(line)));
                    let v = hw.peek(a) & hw.peek(src + 3) | hw.peek(src + 2);
                    hw.poke(a, v);
                }
                src += MIX_LEN;
            } else {
                let len = hw.peek(src + 2) as u16;
                let data = src + RUN_HEADER;
                let (s, e) = (line.max(top), (line + len).min(bot));
                let mut l = s;
                while l < e {
                    let v = hw.peek(data + (l - line));
                    hw.poke(dst.wrapping_add(layout::line_offset(y.wrapping_add(l))), v);
                    l += 1;
                }
                src = data + len;
            }
        }
        dst = dst.wrapping_add(8);
    }
}

/// A sprite placed on screen, as the band pipeline carries it: its phase block (REU), band shift, size
/// and position, clipped to the screen's byte columns.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Placed {
    pub block: u32,
    pub shift: u8,
    pub h: u16,
    /// Screen line of sprite line 0.
    pub top: i16,
    /// On-screen byte columns: sprite columns `j0..j0 + ncols`, at screen column `col`.
    pub j0: u8,
    pub ncols: u8,
    pub col: u8,
    /// End line (exclusive, <= `geom::LINES`): the band pipeline retires the sprite after it.
    pub end: u8,
}

/// Clip a sprite whose top-left is at (`left` pixels, `top` lines), `wb` byte columns of its phase
/// (`left & 3`) wide and `h` lines tall: `None` when no pixel is on screen.
pub fn place(block: u32, shift: u8, wb: u8, h: u16, left: i16, top: i16) -> Option<Placed> {
    let base = left >> 2;
    let j0 = (-base).max(0);
    let j1 = (wb as i16).min(COLS as i16 - base);
    let bottom = top as i32 + h as i32;
    let lines = crate::geom::LINES;
    if j0 >= j1 || top as i32 >= lines || bottom <= 0 {
        return None;
    }
    Some(Placed {
        block,
        shift,
        h,
        top,
        j0: j0 as u8,
        ncols: (j1 - j0) as u8,
        col: (base + j0) as u8,
        end: bottom.min(lines) as u8,
    })
}

fn le24<H: Hw + ?Sized>(hw: &mut H, a: u16) -> u32 {
    hw.peek(a) as u32 | (hw.peek(a + 1) as u32) << 8 | (hw.peek(a + 2) as u32) << 16
}

/// Draw the lines of `s` that fall in screen band `band` into the 320-byte row at `row` (the band's
/// first cell): per sprite band crossed, fetch its start, the on-screen columns' offsets and their
/// records (one DMA each), then [`draw_cols`] / `draw_cols_asm` with the band's line clip.
pub fn sprite_band<H: Hw + ?Sized>(hw: &mut H, s: &Placed, band: u8, row: u16) {
    let lo = band as i16 * 8 - s.top;
    let (t, b) = (lo.max(0), (lo + 8).min(s.h as i16));
    if t >= b {
        return;
    }
    let (t, b) = (t as u16, b as u16);
    for sb in t >> s.shift..=(b - 1) >> s.shift {
        hw::fetch(hw, s.block + 3 * sb as u32, layout::SPRITE_BSTART, 3);
        let band_at = s.block + le24(hw, layout::SPRITE_BSTART);
        let n = s.ncols as u16;
        hw::fetch(hw, band_at + 2 * s.j0 as u32, layout::SPRITE_OFFS, 2 * (n + 1));
        let (start, end) = (peek16(hw, layout::SPRITE_OFFS), peek16(hw, layout::SPRITE_OFFS + 2 * n));
        hw::fetch(hw, band_at + start as u32, layout::SPRITE_CHUNK, end - start);
        poke16(hw, layout::JOB + job::DST, row.wrapping_add((s.col as u16) << 3));
        poke16(hw, layout::JOB + job::Y, (s.top as u16).wrapping_sub(band as u16 * 8));
        poke16(hw, layout::JOB + job::TOP, t);
        poke16(hw, layout::JOB + job::BOT, b);
        poke16(hw, layout::JOB + job::SRC, layout::SPRITE_CHUNK);
        hw.poke(layout::JOB + job::NCOLS, s.ncols);
        hw.draw_cols();
    }
}
