//! The REU image: what the host bakes and the C64 reads.
//!
//! - `$000000`: header — `MAGIC` (8 bytes), `VERSION` (u16 LE), the phase
//!   stamp (u16 LE), then each phase's length in bytes (u16 LE).
//! - `$002000`: code phases, one 32 KiB slot each (`paging::slot`).
//! - `$020000`: terrain records, one per sample in row-major order
//!   (`j · SIZE + i`), 4 bytes each: height, kind, d2, hmax. The interleave
//!   makes every sample one 4-byte DMA; records are 4-aligned, so none
//!   straddles a 512 KiB REU bank.

use crate::paging;
use crate::terrain::SIZE;

pub const MAGIC: &[u8; 8] = b"CHOPPER!";
/// Bump whenever the layout changes: the PRG refuses a mismatched image.
pub const VERSION: u16 = 2;
pub const HEADER: u32 = 0;
pub const HEADER_LEN: u16 = 12 + 2 * paging::PHASES as u16;
pub const PHASES: u32 = 0x2000;
pub const TERRAIN: u32 = 0x2_0000;
pub const RECORD: u32 = 4;
pub const TERRAIN_BYTES: u32 = SIZE as u32 * SIZE as u32 * RECORD;
pub const END: u32 = TERRAIN + TERRAIN_BYTES;
const _: () = assert!(paging::slot(paging::PHASES) <= TERRAIN);
const _: () = assert!(END <= 16 << 20);

/// REU address of sample (i, j)'s record.
#[inline(always)]
pub fn record(i: usize, j: usize) -> u32 {
    TERRAIN + (j as u32 * SIZE as u32 + i as u32) * RECORD
}

/// Bake the image (host). `phases` are the code phase images in id order
/// (empty slices for a host-only image).
#[cfg(feature = "host")]
pub fn build(world: &crate::worldgen::World, phases: &[std::vec::Vec<u8>]) -> std::vec::Vec<u8> {
    assert_eq!(phases.len(), paging::PHASES as usize);
    let mut image = std::vec![0u8; END as usize];
    image[..8].copy_from_slice(MAGIC);
    image[8..10].copy_from_slice(&VERSION.to_le_bytes());
    let refs: std::vec::Vec<&[u8]> = phases.iter().map(|p| p.as_slice()).collect();
    image[10..12].copy_from_slice(&paging::stamp(&refs).to_le_bytes());
    for (p, img) in phases.iter().enumerate() {
        assert!(img.len() <= paging::WINDOW_LEN as usize, "phase {p} too large");
        image[12 + 2 * p..14 + 2 * p].copy_from_slice(&(img.len() as u16).to_le_bytes());
        let at = paging::slot(p as u8) as usize;
        image[at..at + img.len()].copy_from_slice(img);
    }
    for j in 0..SIZE {
        for i in 0..SIZE {
            let k = j * SIZE + i;
            let at = record(i, j) as usize;
            image[at..at + 4].copy_from_slice(&[world.heights[k], world.kinds[k], world.d2[k], world.hmax[k]]);
        }
    }
    image
}
