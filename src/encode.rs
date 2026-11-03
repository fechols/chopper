//! Sprite encoder (host only): indexed sprite → four (or one) phase blocks of banded column records
//! (`crate::format`: band table, column offsets, Run/Mix items with inline pixel bytes). Copied from
//! `../talonhd/src/encode.rs` (e0e891c); there the pixel packing is vendored from gclass (`a5775bf`)
//! and the block layout is TALON HD's.

use crate::format::{BAND_LINES, BAND_SHIFT_ONE, BAND_SHIFT_TALL, ITEM_MIX, MAX_SPRITE_SIZE};

/// A sprite frame in multicolor codes. `None` is transparent; `Some(code)`
/// is an opaque pixel in any of the four global codes.
#[derive(Clone, Debug)]
pub struct SpriteImage {
    pub w: u16,
    pub h: u16,
    pub pixels: Vec<Option<u8>>,
    /// Stored phases: 4 (drawn at any x) or 1 (drawn at `x & !3`).
    pub phases: u8,
}

impl SpriteImage {
    pub fn new(w: u16, h: u16) -> Self {
        assert!((1..=MAX_SPRITE_SIZE).contains(&w) && (1..=MAX_SPRITE_SIZE).contains(&h));
        SpriteImage { w, h, pixels: vec![None; w as usize * h as usize], phases: 4 }
    }
    pub fn get(&self, x: u16, y: u16) -> Option<u8> {
        self.pixels[y as usize * self.w as usize + x as usize]
    }
    pub fn set(&mut self, x: u16, y: u16, v: Option<u8>) {
        let w = self.w as usize;
        self.pixels[y as usize * w + x as usize] = v;
    }
}

/// One phase, encoded: its byte columns and its block (`crate::format`).
#[derive(Clone, Debug)]
pub struct Block {
    pub wb: u16,
    pub bytes: Vec<u8>,
}

/// The band height (log2) and count of a `h`-line sprite.
pub fn bands(h: u16) -> (u8, u16) {
    if h <= BAND_LINES { (BAND_SHIFT_ONE, 1) } else { (BAND_SHIFT_TALL, h.div_ceil(1 << BAND_SHIFT_TALL)) }
}

/// Bit position of pixel `k` (0 = leftmost) within a multicolor byte.
#[inline]
pub fn pixel_shift(k: u16) -> u16 {
    6 - 2 * (k & 3)
}

/// Encode phase `p` (pixels shifted right by `p`) as a block.
pub fn encode_block(img: &SpriteImage, p: u16) -> Block {
    encode_block_banded(img, p, bands(img.h).0)
}

/// [`encode_block`] with `2^shift`-line bands (`BAND_SHIFT_ONE`: one band), for sprites whose columns
/// would not fit `layout::SPRITE_CHUNK_LEN` per band otherwise (`sprite::sprite_band` takes any shift).
pub fn encode_block_banded(img: &SpriteImage, p: u16, shift: u8) -> Block {
    assert!(p < 4);
    let wb = (img.w + p).div_ceil(4);
    let h = img.h as usize;
    let mut bits = vec![0u8; wb as usize * h];
    let mut mask = vec![0u8; wb as usize * h];
    for y in 0..img.h {
        for x in 0..img.w {
            if let Some(code) = img.get(x, y) {
                let px = x + p;
                let i = (px / 4) as usize * h + y as usize;
                let sh = pixel_shift(px);
                bits[i] |= (code & 3) << sh;
                mask[i] |= 3 << sh;
            }
        }
    }
    let nbands = if img.h <= 1 << shift { 1 } else { img.h.div_ceil(1 << shift) };
    let band_h = 1usize << shift;
    // Band table, then the bands.
    let mut bytes = vec![0u8; (nbands as usize + 1) * 3];
    for b in 0..nbands as usize {
        let start = bytes.len();
        bytes[b * 3..b * 3 + 3].copy_from_slice(&(start as u32).to_le_bytes()[..3]);
        let (l0, l1) = (b * band_h, ((b + 1) * band_h).min(h));
        let offs_at = bytes.len();
        bytes.resize(offs_at + (wb as usize + 1) * 2, 0);
        for j in 0..wb as usize {
            let off = (bytes.len() - start) as u16;
            bytes[offs_at + 2 * j..offs_at + 2 * j + 2].copy_from_slice(&off.to_le_bytes());
            let (cb, cm) = (&bits[j * h..(j + 1) * h], &mask[j * h..(j + 1) * h]);
            let count_at = bytes.len();
            bytes.push(0);
            let mut items = 0u16;
            let mut y = l0;
            while y < l1 {
                let m = cm[y];
                if m == 0xFF {
                    let s = y;
                    while y < l1 && cm[y] == 0xFF && y - s < 255 {
                        y += 1;
                    }
                    bytes.extend_from_slice(&[s as u8, (s >> 8) as u8, (y - s) as u8]);
                    bytes.extend_from_slice(&cb[s..y]);
                    items += 1;
                    continue;
                }
                if m != 0 {
                    bytes.extend_from_slice(&[y as u8, (y >> 8) as u8 | ITEM_MIX, cb[y], !m]);
                    items += 1;
                }
                y += 1;
            }
            assert!(items <= 255, "a band column with {items} items");
            bytes[count_at] = items as u8;
        }
        let end = (bytes.len() - start) as u16;
        bytes[offs_at + 2 * wb as usize..offs_at + 2 * wb as usize + 2].copy_from_slice(&end.to_le_bytes());
        assert!(bytes.len() - start <= 0xFFFF, "a band over 64 KiB");
    }
    let end = bytes.len();
    let nb = nbands as usize;
    bytes[nb * 3..nb * 3 + 3].copy_from_slice(&(end as u32).to_le_bytes()[..3]);
    Block { wb, bytes }
}
