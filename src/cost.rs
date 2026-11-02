//! Time model from TALON HD's measured C64U unit costs (`../talonhd/docs/validation.md`, "Renderer unit
//! costs on the C64U", 2026-11-01, `bench` build at 64 MHz):
//!
//! | primitive | measured |
//! |---|---|
//! | I/O register write | 1.26 us |
//! | RAM store `sta abs,x` (5 cycles) | 0.08 us → 16 ns a 6502 cycle |
//! | RAM→RAM byte copy (9 cycles) | 0.14 us (model: 0.144) |
//! | blend (~16 cycles) | 0.27 us (model: 0.256) |
//! | REU fetch 1 / 8 / 64 / 512 B | 9.8 / 12.3 / 26.5 / 130.7 us |
//!
//! A REU fetch is its register writes (8 for a full setup) plus 0.237 us a byte (the slope between the
//! 1-byte and 512-byte measurements).

pub const IO_WRITE_US: f32 = 1.26;
pub const DMA_BYTE_US: f32 = 0.237;
pub const CYCLE_US: f32 = 0.016;
/// One NTSC frame.
pub const FRAME_US: f32 = 16_683.0;

/// Work a renderer did, in the units the C64U was measured in. An I/O write's own `sta` is inside
/// [`IO_WRITE_US`]; `cycles` counts the rest of the 6502 code.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Cost {
    pub io_writes: u32,
    pub dma_cmds: u32,
    pub dma_bytes: u32,
    pub cycles: u32,
}

impl Cost {
    pub fn us(&self) -> f32 {
        self.io_writes as f32 * IO_WRITE_US + self.dma_bytes as f32 * DMA_BYTE_US + self.cycles as f32 * CYCLE_US
    }
}

impl core::ops::AddAssign for Cost {
    fn add_assign(&mut self, o: Cost) {
        self.io_writes += o.io_writes;
        self.dma_cmds += o.dma_cmds;
        self.dma_bytes += o.dma_bytes;
        self.cycles += o.cycles;
    }
}
