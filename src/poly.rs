//! The cell-aware row writer: one cell row (8 lines) of a convex polygon in the 4-colour bitmap, "just
//! write bytes". A cell's 8 lines are 8 consecutive bytes, so the cell is the unit: cells every line of
//! the row covers completely get 8 whole-byte stores of the pattern (no reads, no masks); only the
//! partly covered bytes at the ends of each line are read-modify-written.
//!
//! This is the reference for `target-c64/asm/poly.S` (`tools/test_poly.py` checks it byte for byte);
//! [`paint_row`] is the per-pixel specification both must equal.

use crate::hw::Hw;

/// One cell row of a polygon: line `k` (`y & 7`) covers pixels `xl[k]..xr[k]` (empty when `xl >= xr`),
/// painted with `pat[k & 1]` (a whole byte of four 2-bit codes: even lines, odd lines).
/// The C64 passes it in memory at `layout::POLY_ARGS` in this order (`base` little-endian).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Row {
    pub xl: [u8; 8],
    pub xr: [u8; 8],
    pub pat: [u8; 2],
    /// Bitmap address of the cell row (`bitmap + row * 320`).
    pub base: u16,
}

pub mod args {
    pub const XL: u16 = 0;
    pub const XR: u16 = 8;
    pub const PAT: u16 = 16;
    pub const BASE: u16 = 18;
    pub const SIZE: u16 = 20;
}

/// Pixels `s..4` of a byte (`s` = 0..3).
pub const LEFT_MASK: [u8; 4] = [0xFF, 0x3F, 0x0F, 0x03];
/// Pixels `0..e` of a byte (`e` = 1..4; entry 0 unused).
pub const RIGHT_MASK: [u8; 5] = [0x00, 0xC0, 0xF0, 0xFC, 0xFF];

/// What [`fill_row`] did (for the cost model and the status block).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RowWork {
    pub full_cells: u16,
    pub masked: u16,
    pub stores: u16,
}

/// The cells `c0..c1` every line of `row` covers completely, if any.
pub fn full_cells(row: &Row) -> Option<(u8, u8)> {
    let (mut l, mut r) = (0u8, 255u8);
    for k in 0..8 {
        if row.xl[k] >= row.xr[k] {
            return None;
        }
        l = l.max(row.xl[k]);
        r = r.min(row.xr[k]);
    }
    let (c0, c1) = (l.div_ceil(4), r / 4);
    (c0 < c1).then_some((c0, c1))
}

#[inline]
fn masked<H: Hw + ?Sized>(hw: &mut H, addr: u16, pat: u8, mask: u8) {
    let d = hw.peek(addr);
    hw.poke(addr, d ^ ((d ^ pat) & mask));
}

/// Pixels `a..b` of line `k`.
fn part<H: Hw + ?Sized>(hw: &mut H, base: u16, k: u8, a: u8, b: u8, pat: u8, w: &mut RowWork) {
    if a >= b {
        return;
    }
    let (ca, cb) = (a >> 2, (b - 1) >> 2);
    let mut addr = base + k as u16 + ca as u16 * 8;
    let right = RIGHT_MASK[(b - 4 * cb) as usize];
    if ca == cb {
        masked(hw, addr, pat, LEFT_MASK[(a & 3) as usize] & right);
        w.masked += 1;
        return;
    }
    masked(hw, addr, pat, LEFT_MASK[(a & 3) as usize]);
    for _ in ca + 1..cb {
        addr += 8;
        hw.poke(addr, pat);
        w.stores += 1;
    }
    masked(hw, addr + 8, pat, right);
    w.masked += 2;
}

/// Draw one cell row: the per-line ends, then the full cells, 8 stores each.
pub fn fill_row<H: Hw + ?Sized>(hw: &mut H, row: &Row) -> RowWork {
    let mut w = RowWork::default();
    let full = full_cells(row);
    for k in 0..8u8 {
        let (a, b) = (row.xl[k as usize], row.xr[k as usize]);
        if a >= b {
            continue;
        }
        let pat = row.pat[(k & 1) as usize];
        match full {
            Some((c0, c1)) => {
                part(hw, row.base, k, a, 4 * c0, pat, &mut w);
                part(hw, row.base, k, 4 * c1, b, pat, &mut w);
            }
            None => part(hw, row.base, k, a, b, pat, &mut w),
        }
    }
    if let Some((c0, c1)) = full {
        for c in c0..c1 {
            for k in 0..8u16 {
                hw.poke(row.base + c as u16 * 8 + k, row.pat[(k & 1) as usize]);
            }
        }
        w.full_cells += (c1 - c0) as u16;
    }
    w
}

/// The specification: every covered pixel gets its pattern's code, one pixel at a time.
pub fn paint_row<H: Hw + ?Sized>(hw: &mut H, row: &Row) {
    for k in 0..8u16 {
        let pat = row.pat[(k & 1) as usize];
        for x in row.xl[k as usize]..row.xr[k as usize] {
            let addr = row.base + (x as u16 >> 2) * 8 + k;
            let mask = 0xC0u8 >> (2 * (x & 3));
            masked(hw, addr, pat, mask);
        }
    }
}

/// The 16 shades: (even line, odd line) bytes. 0-3 solid codes, then two-code checkers.
pub const PATTERNS: [[u8; 2]; 16] = {
    const fn solid(c: u8) -> [u8; 2] {
        [c * 0x55, c * 0x55]
    }
    const fn checker(a: u8, b: u8) -> [u8; 2] {
        let ab = a << 6 | b << 4 | a << 2 | b;
        let ba = b << 6 | a << 4 | b << 2 | a;
        [ab, ba]
    }
    [
        solid(0),
        solid(1),
        solid(2),
        solid(3),
        checker(0, 1),
        checker(1, 2),
        checker(0, 2),
        checker(1, 3),
        checker(2, 3),
        checker(0, 3),
        // Horizontal stripes (line parity): a softer mix.
        [0x00, 0x55],
        [0x55, 0xAA],
        [0x55, 0xFF],
        [0xAA, 0xFF],
        [0x00, 0xAA],
        [0x00, 0xFF],
    ]
};
