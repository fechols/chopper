//! Machine access used by the generic renderer, and REU DMA helpers built on
//! nothing but register pokes. The C64 implements [`Hw`] with volatile MMIO;
//! the host implements it with [`crate::model::Machine`].

use crate::format::{MAX_DMA, REU_BANK};

/// REU registers (1764/1750 compatible).
pub mod reu {
    pub const COMMAND: u16 = 0xDF01;
    pub const C64_LO: u16 = 0xDF02;
    pub const C64_HI: u16 = 0xDF03;
    pub const REU_LO: u16 = 0xDF04;
    pub const REU_MID: u16 = 0xDF05;
    pub const REU_BANK: u16 = 0xDF06;
    pub const LEN_LO: u16 = 0xDF07;
    pub const LEN_HI: u16 = 0xDF08;
    pub const IRQ_MASK: u16 = 0xDF09;
    pub const CONTROL: u16 = 0xDF0A;
    /// Execute immediately, REU → C64.
    pub const FETCH: u8 = 0x91;
    /// Execute immediately, C64 → REU.
    pub const STASH: u8 = 0x90;
}

pub trait Hw {
    fn peek(&mut self, addr: u16) -> u8;
    fn poke(&mut self, addr: u16, value: u8);
    /// Write the REU command register, starting a DMA. Must be opaque to the
    /// optimizer on the C64: the DMA writes RAM behind the compiler's back.
    fn reu_exec(&mut self, command: u8);
    /// Store `value` into `len` bytes of RAM at `addr`.
    fn fill(&mut self, addr: u16, len: u16, value: u8) {
        for i in 0..len {
            self.poke(addr.wrapping_add(i), value);
        }
    }
    /// Draw one cell row of a polygon (`poly::fill_row`; the C64 runs `asm/poly.S`).
    /// Draw sprite column records (`sprite::draw_cols`; the C64 runs `asm/blit.S` inside `asm/geom.S`).
    fn draw_cols(&mut self) {
        crate::sprite::draw_cols(self);
    }
    fn poly_row(&mut self, row: &crate::poly::Row) {
        crate::poly::fill_row(self, row);
    }
    /// Transform one object's vertices (`geom::xform`; the C64 runs `asm/geom.S`).
    fn xform(&mut self, scene: &[u8], o: &crate::geom::Obj, v: &mut crate::geom::Verts) {
        crate::geom::xform(scene, o, v);
    }
    /// Cull and sort one object's faces (`geom::faces`; the C64 runs `asm/geom.S`).
    fn faces(&mut self, scene: &[u8], o: &crate::geom::Obj, v: &crate::geom::Verts, w: &mut crate::geom::Work) {
        crate::geom::faces(scene, o, v, w);
    }
    /// Profiling hook: charge the time since the previous mark to `slot`.
    #[inline(always)]
    fn mark(&mut self, _slot: u8) {}
}

#[inline]
fn poke16<H: Hw + ?Sized>(hw: &mut H, addr: u16, value: u16) {
    hw.poke(addr, value as u8);
    hw.poke(addr + 1, (value >> 8) as u8);
}

/// Load the REU address registers (start of a DMA chain).
#[inline]
pub fn set_reu_addr<H: Hw + ?Sized>(hw: &mut H, addr: u32) {
    hw.poke(reu::REU_LO, addr as u8);
    hw.poke(reu::REU_MID, (addr >> 8) as u8);
    hw.poke(reu::REU_BANK, (addr >> 16) as u8);
}

/// Continue a chain: fetch `len` bytes from the REU address left behind by
/// the previous command into `c64`.
#[inline]
pub fn chain_fetch<H: Hw + ?Sized>(hw: &mut H, c64: u16, len: u16) {
    poke16(hw, reu::C64_LO, c64);
    poke16(hw, reu::LEN_LO, len);
    hw.reu_exec(reu::FETCH);
}

/// Reset `$DF09`/`$DF0A` once; every helper here assumes both stay zero
/// (each register write costs a slow I/O bus cycle under turbo).
pub fn init<H: Hw + ?Sized>(hw: &mut H) {
    hw.poke(reu::IRQ_MASK, 0);
    hw.poke(reu::CONTROL, 0);
}

fn transfer<H: Hw + ?Sized>(hw: &mut H, command: u8, mut c64: u16, mut addr: u32, mut len: u16) {
    while len != 0 {
        let to_bank = REU_BANK - (addr & (REU_BANK - 1));
        let mut n = len.min(MAX_DMA);
        if (n as u32) > to_bank {
            n = to_bank as u16;
        }
        set_reu_addr(hw, addr);
        poke16(hw, reu::C64_LO, c64);
        poke16(hw, reu::LEN_LO, n);
        hw.reu_exec(command);
        c64 = c64.wrapping_add(n);
        addr += n as u32;
        len -= n;
    }
}

/// REU → C64, split at 512 KiB banks and [`MAX_DMA`].
pub fn fetch<H: Hw + ?Sized>(hw: &mut H, addr: u32, c64: u16, len: u16) {
    transfer(hw, reu::FETCH, c64, addr, len);
}

/// C64 → REU, split at 512 KiB banks and [`MAX_DMA`].
pub fn stash<H: Hw + ?Sized>(hw: &mut H, c64: u16, addr: u32, len: u16) {
    transfer(hw, reu::STASH, c64, addr, len);
}

/// Read REU bytes into a host-side/stack buffer via a bounce through C64 RAM
/// at `bounce`. On the C64 `bounce` is the buffer's own address.
pub fn read_into<H: Hw + ?Sized>(hw: &mut H, addr: u32, bounce: u16, out: &mut [u8]) {
    fetch(hw, addr, bounce, out.len() as u16);
    for (i, b) in out.iter_mut().enumerate() {
        *b = hw.peek(bounce.wrapping_add(i as u16));
    }
}
