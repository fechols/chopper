//! Host model (vendored from gclass `a5775bf`) of the parts of a C64 + REU
//! the renderer touches: 64 KiB RAM
//! (`$01=$35`: RAM everywhere except I/O), color RAM, `$DD00`, and the REU
//! registers with per-byte 19-bit address wrap, auto-increment, fixed-address
//! control and the "registers keep their final values" behavior that chained
//! DMAs rely on.
//!
//! MODE 7 addition (found on the C64U, 2026-11-01): writing any REU address register (`$DF04-$DF06`)
//! reloads the whole address counter from the values last *written* to all three, so a transfer's
//! auto-increment survives only until the next address write.

use crate::format::REU_SIZE;
use crate::hw::{Hw, reu};
use crate::layout::{self, Buffer};
use crate::palette::PALETTE;

pub struct Machine {
    pub ram: Vec<u8>,
    pub color_ram: [u8; 1024],
    pub reu: Vec<u8>,
    regs: [u8; 16],
    /// The values last written to `$DF04-$DF06`.
    latch: [u8; 3],
    pub dd00: u8,
    /// Last values written to `$D011` and `$D021`.
    pub d011: u8,
    pub d021: u8,
    /// Sprite colors `$D027-$D02E` as last written.
    pub sprite_color: [u8; 8],
    pub stats: Stats,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub commands: u64,
    pub bytes: u64,
    /// Commands whose REU address wrapped inside a 512 KiB bank.
    pub bank_wraps: u64,
    /// DMA commands that wrote somewhere other than RAM/color RAM.
    pub stray_writes: u64,
    pub peeks: u64,
    pub pokes: u64,
}

impl Machine {
    pub fn new(image: &[u8]) -> Self {
        let mut reu = vec![0u8; REU_SIZE as usize];
        reu[..image.len()].copy_from_slice(image);
        Machine { ram: vec![0; 65536], color_ram: [0; 1024], reu, regs: [0; 16], latch: [0; 3], dd00: 0x03, d011: 0, d021: 0, sprite_color: [0; 8], stats: Stats::default() }
    }

    fn is_io(addr: u16) -> bool {
        (0xD000..0xE000).contains(&addr)
    }

    fn reg(&self, r: u16) -> u8 {
        self.regs[(r - 0xDF00) as usize]
    }

    /// The visible lines of `buf` (24-row mode, `fine` = scroll fine) as RGB.
    pub fn visible(&self, buf: Buffer, d021: u8, fine: u8) -> Vec<[u8; 3]> {
        let all = self.decode(buf, d021);
        let top = fine as usize * 160;
        all[top..top + layout::VISIBLE as usize * 160].to_vec()
    }

    /// The bytes the VIC shows for `buf`, decoded to (code, cell) RGB pixels.
    pub fn decode(&self, buf: Buffer, d021: u8) -> Vec<[u8; 3]> {
        let mut out = vec![[0u8; 3]; 160 * 200];
        for y in 0..200u16 {
            for x in 0..160u16 {
                let cell = (y / 8) * 40 + x / 4;
                let byte = self.ram[(buf.bitmap + layout::line_offset(y) + (x / 4) * 8) as usize];
                let code = (byte >> (6 - 2 * (x & 3))) & 3;
                let scr = self.ram[(buf.screen + cell) as usize];
                let color = match code {
                    0 => d021,
                    1 => scr >> 4,
                    2 => scr & 15,
                    _ => self.color_ram[cell as usize] & 15,
                };
                out[y as usize * 160 + x as usize] = PALETTE[color as usize & 15];
            }
        }
        out
    }

    /// Multicolor code of every pixel of `buf`'s bitmap.
    pub fn codes(&self, buf: Buffer) -> Vec<u8> {
        let mut out = vec![0u8; 160 * 200];
        for y in 0..200u16 {
            for x in 0..160u16 {
                let byte = self.ram[(buf.bitmap + layout::line_offset(y) + (x / 4) * 8) as usize];
                out[y as usize * 160 + x as usize] = (byte >> (6 - 2 * (x & 3))) & 3;
            }
        }
        out
    }

    pub fn front(&self) -> Option<Buffer> {
        layout::BUFFERS.iter().copied().find(|b| b.dd00 == self.dd00 & 3)
    }
}

impl Hw for Machine {
    fn peek(&mut self, addr: u16) -> u8 {
        self.stats.peeks += 1;
        match addr {
            0xD800..=0xDBFF => self.color_ram[(addr - 0xD800) as usize] | 0xF0,
            0xDD00 => self.dd00,
            0xDF00..=0xDF0F => self.reg(addr),
            a if Self::is_io(a) => 0xFF,
            a => self.ram[a as usize],
        }
    }

    fn poke(&mut self, addr: u16, value: u8) {
        self.stats.pokes += 1;
        match addr {
            0xD800..=0xDBFF => self.color_ram[(addr - 0xD800) as usize] = value & 15,
            0xDD00 => self.dd00 = value,
            0xD011 => self.d011 = value,
            0xD021 => self.d021 = value & 15,
            0xD027..=0xD02E => self.sprite_color[(addr - 0xD027) as usize] = value & 15,
            0xDF01 => self.reu_exec(value),
            0xDF04..=0xDF06 => {
                self.latch[(addr - 0xDF04) as usize] = value;
                self.regs[4..7].copy_from_slice(&self.latch);
            }
            0xDF00..=0xDF0F => self.regs[(addr - 0xDF00) as usize] = value,
            a if Self::is_io(a) => {}
            a => self.ram[a as usize] = value,
        }
    }

    fn reu_exec(&mut self, command: u8) {
        assert!(command & 0x80 != 0, "REU command without execute bit");
        let mut c64 = self.reg(reu::C64_LO) as u16 | (self.reg(reu::C64_HI) as u16) << 8;
        let mut addr = self.reg(reu::REU_LO) as u32
            | (self.reg(reu::REU_MID) as u32) << 8
            | (self.reg(reu::REU_BANK) as u32) << 16;
        let raw = self.reg(reu::LEN_LO) as u32 | (self.reg(reu::LEN_HI) as u32) << 8;
        let len = if raw == 0 { 65536 } else { raw };
        let ctrl = self.reg(reu::CONTROL);
        self.stats.commands += 1;
        self.stats.bytes += len as u64;
        let mut wrapped = false;
        let mut stray = false;
        for _ in 0..len {
            let phys = (addr as usize) % self.reu.len();
            match command & 3 {
                1 => {
                    let v = self.reu[phys];
                    match c64 {
                        0xD800..=0xDBFF => self.color_ram[(c64 - 0xD800) as usize] = v & 15,
                        a if Self::is_io(a) => stray = true,
                        a => self.ram[a as usize] = v,
                    }
                }
                0 => {
                    let v = match c64 {
                        0xD800..=0xDBFF => self.color_ram[(c64 - 0xD800) as usize],
                        a if Self::is_io(a) => {
                            stray = true;
                            0xFF
                        }
                        a => self.ram[a as usize],
                    };
                    self.reu[phys] = v;
                }
                _ => panic!("swap/verify not modeled"),
            }
            if ctrl & 0x80 == 0 {
                c64 = c64.wrapping_add(1);
            }
            if ctrl & 0x40 == 0 {
                let next = (addr & !0x7FFFF) | ((addr + 1) & 0x7FFFF);
                wrapped |= next & 0x7FFFF == 0;
                addr = next;
            }
        }
        // A transfer that ends exactly on a bank boundary is fine; only a
        // byte transferred *after* the wrap is wrong.
        let ended_on_boundary = wrapped && addr & 0x7FFFF == 0;
        if wrapped && !ended_on_boundary {
            self.stats.bank_wraps += 1;
        }
        if stray {
            self.stats.stray_writes += 1;
        }
        let r = |reg: u16| (reg - 0xDF00) as usize;
        self.regs[r(reu::C64_LO)] = c64 as u8;
        self.regs[r(reu::C64_HI)] = (c64 >> 8) as u8;
        self.regs[r(reu::REU_LO)] = addr as u8;
        self.regs[r(reu::REU_MID)] = (addr >> 8) as u8;
        self.regs[r(reu::REU_BANK)] = (addr >> 16) as u8;
        self.regs[r(reu::LEN_LO)] = 1;
        self.regs[r(reu::LEN_HI)] = 0;
    }
}

