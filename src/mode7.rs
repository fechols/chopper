//! Mode 7: a 256×256-texel plane drawn into a 160×200 multicolor bitmap with one affine transform per
//! line (SNES HDMA style: rotation, zoom, and a perspective floor when the lines' scales differ).
//!
//! Three ways, each counting its work in the C64U's measured units ([`crate::cost`]):
//! - [`per_byte_dma`] (`pre = false`): one REU DMA per bitmap byte from the raw plane. The bitmap is
//!   walked in memory order, so the C64 address auto-increments and only the REU address registers that
//!   changed are written. One texel per byte: 40 texels across.
//! - [`per_byte_dma`] (`pre = true`): the same walk from pre-sampled planes ([`crate::format::pre_bank`]):
//!   each REU byte already holds four texels along the line's direction and zoom. 160 texels across; the
//!   16 MiB REU pays for the resolution.
//! - [`cpu_gather`]: the DDA on the CPU, reading 1, 2 or 4 texels a byte from a 128×128 RAM cache that
//!   [`fill_cache`] DMAs from the REU in long runs.

use crate::cost::Cost;
use crate::format::{MAP_BANK, SKY_BANK};
use crate::hw::{self, Hw, reu};
use crate::layout::{self, Buffer, CACHE, CACHE_SIZE};

/// Code 3 in every pixel: the sky.
pub const SKY_BYTE: u8 = 0xFF;
/// Code 0 (`$D021`) in every pixel: what the CPU path shows outside its cache window.
pub const FOG_BYTE: u8 = 0x00;

/// One line's transform.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Line {
    /// Plane position of pixel 0, 8.8 texels (wraps at 256 like the plane).
    pub u0: u16,
    pub v0: u16,
    /// Step per multicolor pixel, 8.8 texels.
    pub du: i16,
    pub dv: i16,
    /// Pre-sampled variant for this line's direction and zoom (`per_byte_dma` with `pre`).
    pub bank: u8,
    /// `cpu_gather`: bytes `clip0..clip1` read the cache; the others are [`FOG_BYTE`].
    pub clip0: u8,
    pub clip1: u8,
    pub sky: bool,
}

impl Line {
    pub const SKY: Line = Line { u0: 0, v0: 0, du: 0, dv: 0, bank: SKY_BANK, clip0: 0, clip1: 0, sky: true };
}

/// 6502 cycles of the kernels the paths stand for (16 ns each on the C64U). I/O writes are not in here.
/// `DMA_BYTE`, `TEXEL` and `STORE` are counted from `target-c64/bench/bench.S` in the 6502 interpreter
/// (`python tools/bench_cycles.py`); the rest are estimates.
pub mod cycles {
    /// `per_byte_dma`, per byte: step the line's u and v 8.8 accumulators in the 8-line table
    /// (2 × `lda acc,x / clc / adc step,x / sta acc,x / lda acc+8,x / adc step+8,x / sta acc+8,x` = 56),
    /// compare the REU address shadow for lo and mid (2 × 12) and bank (3), advance the shadow (7), loop (7).
    pub const DMA_BYTE: u32 = 88;
    /// `per_byte_dma`, per cell row: load the 8 lines' records into the accumulator table.
    pub const DMA_ROW: u32 = 8 * 40;
    /// `cpu_gather`, per texel: u step (`lda uf / adc / sta / tya / adc / and #$7f / tay`, `clc`s),
    /// v step and row lookup (`lda vf / adc / sta / lda vi / adc / sta / tax / lda hi,x / sta p+1 /
    /// lda lo,x / sta p`), read and merge (`lda (p),y / and #m / ora acc / sta acc`).
    pub const TEXEL: u32 = 72;
    /// `cpu_gather`, per byte: clear the accumulator, self-modified `sta abs`, operand `+= 8`, loop.
    pub const STORE: u32 = 35;
    /// A byte filled with a constant (sky, fog).
    pub const FILL: u32 = 10;
    /// `cpu_gather`, per line: load the line record, set the destination, apply the clip.
    pub const LINE: u32 = 80;
    /// `fill_cache`, per DMA: the loop around `hw::fetch`.
    pub const CACHE_DMA: u32 = 40;
}

#[inline]
fn io<H: Hw + ?Sized>(hw: &mut H, cost: &mut Cost, reg: u16, value: u8) {
    hw.poke(reg, value);
    cost.io_writes += 1;
}

/// `x * d / 2` in 8.8, wrapping (half-pixel offsets).
#[inline]
fn half(d: i16, x: i32) -> u16 {
    ((d as i32 * x) >> 1) as u16
}

/// How [`per_byte_dma`] walks the bitmap.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DmaMode {
    /// Sample the pre-sampled planes (`Line::bank`) instead of the raw plane.
    pub pre: bool,
    /// Line-doubled: one 2-byte DMA per line pair with the REU address fixed (`$DF0A` bit 6), so both
    /// lines get the even line's byte.
    pub doubled: bool,
    /// First cell row drawn (the rows above are left alone: the sky is drawn once).
    pub first_row: u16,
}

impl DmaMode {
    pub const RAW: DmaMode = DmaMode { pre: false, doubled: false, first_row: 0 };
    pub const PRE: DmaMode = DmaMode { pre: true, doubled: false, first_row: 0 };
}

/// Draw `lines` into `buf` with one REU DMA per byte, or per line pair (see the module docs). This is
/// the reference for `target-c64/asm/floor.S`.
pub fn per_byte_dma<H: Hw + ?Sized>(hw: &mut H, buf: Buffer, lines: &[Line; 200], mode: DmaMode) -> Cost {
    let mut cost = Cost::default();
    let start = buf.bitmap + mode.first_row * layout::ROW_STRIDE;
    let (span, step) = if mode.doubled { (2u8, 2usize) } else { (1, 1) };
    io(hw, &mut cost, reu::C64_LO, start as u8);
    io(hw, &mut cost, reu::C64_HI, (start >> 8) as u8);
    io(hw, &mut cost, reu::LEN_LO, span);
    io(hw, &mut cost, reu::LEN_HI, 0);
    io(hw, &mut cost, reu::CONTROL, if mode.doubled { 0x40 } else { 0 });
    // The REU address registers as the last DMA left them (they auto-increment unless fixed).
    let mut shadow: Option<u32> = None;
    for row in mode.first_row as usize..layout::ROWS as usize {
        // The raw plane is sampled at the byte's centre (pixel 1.5), a pre-sampled one from pixel 0.
        let off = if mode.pre { 0 } else { 3 };
        let mut acc = [(0u16, 0u16); 8];
        for l in (0..8).step_by(step) {
            let ln = &lines[row * 8 + l];
            acc[l] = (ln.u0.wrapping_add(half(ln.du, off)), ln.v0.wrapping_add(half(ln.dv, off)));
        }
        cost.cycles += cycles::DMA_ROW;
        for _col in 0..layout::COLS {
            for l in (0..8).step_by(step) {
                let ln = &lines[row * 8 + l];
                let (u, v) = acc[l];
                let addr = if ln.sky {
                    (SKY_BANK as u32) << 16
                } else {
                    let bank = if mode.pre { ln.bank } else { MAP_BANK };
                    (bank as u32) << 16 | (v & 0xFF00) as u32 | (u >> 8) as u32
                };
                // The C64U reloads the whole REU address from the values last written whenever one
                // address register is written, losing the auto-increment (found on hardware). Doubled
                // (fixed address) relies on no increment and keeps all three shadowed; full writes mid
                // and bank when they change, then lo always.
                let order: &[(u32, u16)] = if mode.doubled {
                    &[(0, reu::REU_LO), (8, reu::REU_MID), (16, reu::REU_BANK)]
                } else {
                    &[(8, reu::REU_MID), (16, reu::REU_BANK), (0, reu::REU_LO)]
                };
                for &(shift, reg) in order {
                    let want = (addr >> shift) as u8;
                    if (shift == 0 && !mode.doubled) || shadow.is_none_or(|s| (s >> shift) as u8 != want) {
                        io(hw, &mut cost, reg, want);
                    }
                }
                if mode.doubled {
                    // A transfer leaves the length register at 1 (17xx without autoload).
                    io(hw, &mut cost, reu::LEN_LO, span);
                }
                hw.reu_exec(reu::FETCH);
                cost.io_writes += 1;
                cost.dma_cmds += 1;
                cost.dma_bytes += span as u32;
                shadow = Some(addr);
                acc[l] = (u.wrapping_add((ln.du as u16).wrapping_mul(4)), v.wrapping_add((ln.dv as u16).wrapping_mul(4)));
                cost.cycles += cycles::DMA_BYTE;
            }
        }
    }
    if mode.doubled {
        io(hw, &mut cost, reu::CONTROL, 0);
    }
    cost
}

/// This frame's lines from a per-angle line table (`format::line_table`, as the C64 fetches it) and the
/// camera position (8.8 texels): the C64's arithmetic exactly, two 16-bit adds a line.
pub fn lines_at(table: &[u8], cam_x: u16, cam_y: u16) -> [Line; 200] {
    use crate::format::{LINES, field};
    assert_eq!(table.len(), crate::format::LINE_BLOCK as usize);
    let f = |k: usize, l: usize| table[k * LINES + l];
    let w = |k: usize, l: usize| f(k, l) as u16 | (f(k + 1, l) as u16) << 8;
    let mut out = [Line::SKY; 200];
    for (l, ln) in out.iter_mut().enumerate() {
        let bank = f(field::BANK, l);
        if bank == SKY_BANK {
            continue;
        }
        // du*4 never overflows for drawn lines (fog lines are zero), so the shift recovers du exactly.
        *ln = Line {
            u0: cam_x.wrapping_add(w(field::U0, l)),
            v0: cam_y.wrapping_add(w(field::V0, l)),
            du: (w(field::DU4, l) as i16) >> 2,
            dv: (w(field::DV4, l) as i16) >> 2,
            bank,
            ..Line::default()
        };
    }
    out
}

/// Draw `lines` into `buf` on the CPU from the texel cache, `texels` (1, 2 or 4) samples per byte.
pub fn cpu_gather<H: Hw + ?Sized>(hw: &mut H, buf: Buffer, lines: &[Line; 200], texels: u8) -> Cost {
    assert!(matches!(texels, 1 | 2 | 4));
    let mut cost = Cost::default();
    let n = texels as i32;
    let width = 4 / n; // pixels per sample
    let bits = 8 / texels as u32;
    for (y, ln) in lines.iter().enumerate() {
        cost.cycles += cycles::LINE;
        let dst = buf.bitmap + layout::line_offset(y as u16);
        if ln.sky {
            for c in 0..layout::COLS {
                hw.poke(dst + c * 8, SKY_BYTE);
            }
            cost.cycles += layout::COLS as u32 * cycles::FILL;
            continue;
        }
        // Each sample sits at the centre of the pixels it paints.
        let mut u = ln.u0.wrapping_add(half(ln.du, width - 1));
        let mut v = ln.v0.wrapping_add(half(ln.dv, width - 1));
        let (su, sv) = ((ln.du as u16).wrapping_mul(width as u16), (ln.dv as u16).wrapping_mul(width as u16));
        for c in 0..layout::COLS {
            let inside = (ln.clip0 as u16..ln.clip1 as u16).contains(&c);
            let mut b = FOG_BYTE;
            if inside {
                b = 0;
                for j in 0..texels as u32 {
                    let t = hw.peek(CACHE + ((v >> 8) & 127) * CACHE_SIZE + ((u >> 8) & 127));
                    let mask = (((1u16 << bits) - 1) << (8 - bits * (j + 1))) as u8;
                    b |= t & mask;
                    u = u.wrapping_add(su);
                    v = v.wrapping_add(sv);
                }
                cost.cycles += texels as u32 * cycles::TEXEL + cycles::STORE;
            } else {
                u = u.wrapping_add(su.wrapping_mul(texels as u16));
                v = v.wrapping_add(sv.wrapping_mul(texels as u16));
                cost.cycles += cycles::FILL;
            }
            hw.poke(dst + c * 8, b);
        }
    }
    cost
}

/// Load the 128×128 cache window with corner `(wu, wv)` from the raw plane: per row at most two DMAs
/// (the window wraps in the cache at multiples of 128 texels).
pub fn fill_cache<H: Hw + ?Sized>(hw: &mut H, wu: u8, wv: u8) -> Cost {
    let mut cost = Cost::default();
    for r in 0..CACHE_SIZE as u8 {
        let v = wv.wrapping_add(r);
        let row = CACHE + (v as u16 & 127) * CACHE_SIZE;
        let first = CACHE_SIZE - (wu as u16 & 127);
        for (u, len) in [(wu, first), (wu.wrapping_add(first as u8), CACHE_SIZE - first)] {
            if len == 0 {
                continue;
            }
            let addr = (MAP_BANK as u32) << 16 | (v as u32) << 8 | u as u32;
            hw::fetch(hw, addr, row + (u as u16 & 127), len);
            cost.io_writes += 8;
            cost.dma_cmds += 1;
            cost.dma_bytes += len as u32;
            cost.cycles += cycles::CACHE_DMA;
        }
    }
    cost
}

/// Colours: code 0 `$D021`, 1/2 screen RAM, 3 colour RAM, which is `sky` above cell row `horizon_row`
/// and `ground3` below it.
pub fn setup_colors<H: Hw + ?Sized>(hw: &mut H, buf: Buffer, colors: [u8; 4], sky: u8, horizon_row: u16) {
    hw.poke(0xD021, colors[0]);
    hw.fill(buf.screen, layout::CELLS, colors[1] << 4 | colors[2]);
    hw.fill(layout::COLOR_RAM, horizon_row * layout::COLS, sky);
    hw.fill(layout::COLOR_RAM + horizon_row * layout::COLS, layout::CELLS - horizon_row * layout::COLS, colors[3]);
}
