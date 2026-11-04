//! The effect sprites' baker (host): the code maps `tools/effects.py` made from the Ludo.ai animations
//! (`Assets/Effects/baked/<name>.bin`) encoded as sprite blocks (`encode`, all 4 phases) into the REU at
//! `format::EFFECTS_BLOCKS`, with their directory records (`format::frame`) at `effects::record`; and the
//! reverse, a frame drawn back out of an image by the C64's own sprite code (`sprite::sprite_band` on a
//! model), for the preview and the tests.

use crate::effects::{EFFECTS, FRAMES, MAX_FRAMES, NAMES, SCALES, lines, record};
use crate::encode::{Block, SpriteImage, bands, encode_block_banded};
use crate::format::{EFFECTS_BLOCKS, EFFECTS_BLOCKS_END, EFFECTS_HIGH, EFFECTS_LOW_END, REU_BANK, frame, phase};
use crate::hw::Hw;
use crate::layout::{self, BUFFERS};
use crate::model::Machine;

const BAKED: [&[u8]; EFFECTS] = [
    include_bytes!("../Assets/Effects/baked/explosion.bin"),
    include_bytes!("../Assets/Effects/baked/impact.bin"),
    include_bytes!("../Assets/Effects/baked/splash.bin"),
    include_bytes!("../Assets/Effects/baked/smoke.bin"),
    include_bytes!("../Assets/Effects/baked/muzzle.bin"),
    include_bytes!("../Assets/Effects/baked/wreck_fire.bin"),
];

/// One scale of an effect: its frames (all the same size) and anchor.
#[derive(Clone, Debug)]
pub struct Scale {
    pub ax: u8,
    pub ay: u8,
    pub frames: Vec<SpriteImage>,
}

/// Every effect's scales, decoded from `tools/effects.py`'s files (`[effect][scale]`).
pub fn load() -> Vec<Vec<Scale>> {
    (0..EFFECTS).map(|e| parse(BAKED[e]).unwrap_or_else(|err| panic!("{}.bin: {err}", NAMES[e]))).collect()
}

fn parse(b: &[u8]) -> Result<Vec<Scale>, String> {
    if b.len() < 6 || &b[..4] != b"EFX1" {
        return Err("not an EFX1 file".into());
    }
    let (nf, ns) = (b[4] as usize, b[5] as usize);
    if ns != SCALES || nf > MAX_FRAMES {
        return Err(format!("{nf} frames, {ns} scales"));
    }
    let mut at = 6;
    let mut out = Vec::new();
    for s in 0..ns {
        let [w, h, ax, ay] = b.get(at..at + 4).ok_or("short")?.try_into().unwrap();
        at += 4;
        if h as u16 != lines(s as u8) {
            return Err(format!("scale {s}: {h} lines"));
        }
        let mut frames = Vec::new();
        for _ in 0..nf {
            let mut img = SpriteImage::new(w as u16, h as u16);
            img.phases = 4;
            let mut i = 0;
            while i < img.pixels.len() {
                let v = *b.get(at).ok_or("short")?;
                at += 1;
                let (code, run) = (v >> 5, (v & 31) as usize + 1);
                if code > 4 || i + run > img.pixels.len() {
                    return Err("bad run".into());
                }
                img.pixels[i..i + run].fill(if code == 4 { None } else { Some(code) });
                i += run;
            }
            frames.push(img);
        }
        out.push(Scale { ax, ay, frames });
    }
    if at != b.len() {
        return Err("trailing bytes".into());
    }
    Ok(out)
}

/// What [`install`] wrote.
#[derive(Clone, Copy, Debug, Default)]
pub struct Usage {
    pub frames: u32,
    /// Bytes of blocks, and of padding left at 512 KiB region ends.
    pub blocks: u32,
    pub padding: u32,
    /// The first free REU address after the blocks.
    pub end: u32,
    /// The largest block, and the largest chunk one band draws (`layout::SPRITE_CHUNK_LEN`).
    pub max_block: u32,
    pub max_chunk: u32,
}

/// Bytes of the column records band `b` of `block` draws when all its columns are on screen.
pub fn band_chunk(block: &[u8], b: usize) -> u32 {
    let le24 = |a: usize| block[a] as u32 | (block[a + 1] as u32) << 8 | (block[a + 2] as u32) << 16;
    let at = le24(3 * b) as usize;
    let wb = {
        // The band's column offsets: wb + 1 of them, the first right after them.
        let first = u16::from_le_bytes([block[at], block[at + 1]]) as usize;
        first / 2 - 1
    };
    let end = u16::from_le_bytes([block[at + 2 * wb], block[at + 2 * wb + 1]]) as u32;
    let start = u16::from_le_bytes([block[at], block[at + 1]]) as u32;
    end - start
}

/// A frame's four phases, in the tallest bands (`encode::bands`, else 16, else 8 lines) whose column
/// records fit `layout::SPRITE_CHUNK_LEN` (what one band fetches to draw).
pub fn encode(img: &SpriteImage) -> (u8, Vec<Block>) {
    for shift in [bands(img.h).0, 4, 3] {
        let blocks: Vec<Block> = (0..4).map(|p| encode_block_banded(img, p, shift)).collect();
        let nbands = if img.h <= 1 << shift { 1 } else { img.h.div_ceil(1 << shift) } as usize;
        if blocks.iter().all(|b| (0..nbands).all(|k| band_chunk(&b.bytes, k) <= layout::SPRITE_CHUNK_LEN as u32)) {
            return (shift, blocks);
        }
    }
    panic!("effects: a {}x{} frame's columns do not fit SPRITE_CHUNK_LEN even in 8-line bands", img.w, img.h);
}

/// Encode every effect into the image: blocks from `EFFECTS_BLOCKS`, directory records at
/// `effects::record`.
pub fn install(reu: &mut [u8]) -> Usage {
    let mut u = Usage::default();
    let mut at = EFFECTS_BLOCKS;
    for (e, scales) in load().iter().enumerate() {
        assert_eq!(scales[0].frames.len(), FRAMES[e] as usize, "{}: frame count", NAMES[e]);
        for (s, sc) in scales.iter().enumerate() {
            for (f, img) in sc.frames.iter().enumerate() {
                let mut rec = [0u8; crate::format::FRAME_RECORD as usize];
                rec[frame::W..frame::W + 2].copy_from_slice(&img.w.to_le_bytes());
                rec[frame::H..frame::H + 2].copy_from_slice(&img.h.to_le_bytes());
                rec[frame::PHASE_COUNT] = 4;
                let (shift, blocks) = encode(img);
                let nbands = if img.h <= 1 << shift { 1 } else { img.h.div_ceil(1 << shift) };
                rec[frame::BAND_SHIFT] = shift;
                rec[frame::BANDS] = nbands as u8;
                rec[frame::AX] = sc.ax;
                rec[frame::AY] = sc.ay;
                for (p, block) in blocks.iter().enumerate() {
                    let len = block.bytes.len() as u32;
                    // Not into the rotor sound's PCM: on above it.
                    if at < EFFECTS_HIGH && at + len > EFFECTS_LOW_END {
                        u.padding += EFFECTS_LOW_END.saturating_sub(at);
                        at = EFFECTS_HIGH;
                    }
                    // Keep every block inside one 512 KiB region: no DMA into it wraps.
                    if at / REU_BANK != (at + len - 1) / REU_BANK {
                        let next = (at / REU_BANK + 1) * REU_BANK;
                        u.padding += next - at;
                        at = next;
                    }
                    assert!(at + len <= EFFECTS_BLOCKS_END, "effects: blocks past EFFECTS_BLOCKS_END");
                    reu[at as usize..][..len as usize].copy_from_slice(&block.bytes);
                    for b in 0..nbands as usize {
                        u.max_chunk = u.max_chunk.max(band_chunk(&block.bytes, b));
                    }
                    let r = frame::PHASES + p as usize * frame::PHASE_LEN;
                    rec[r + phase::WB] = block.wb as u8;
                    rec[r + phase::BLOCK..r + phase::BLOCK + 3].copy_from_slice(&at.to_le_bytes()[..3]);
                    rec[r + phase::LEN..r + phase::LEN + 2].copy_from_slice(&(len.min(0xFFFF) as u16).to_le_bytes());
                    u.blocks += len;
                    u.max_block = u.max_block.max(len);
                    at += len;
                }
                let r = record(e as u8, s as u8, f as u8) as usize;
                reu[r..r + rec.len()].copy_from_slice(&rec);
                u.frames += 1;
            }
        }
    }
    assert!(u.max_chunk <= layout::SPRITE_CHUNK_LEN as u32, "effects: a band of {} bytes", u.max_chunk);
    u.end = at;
    u
}

/// A frame's directory record, read back: (w, h, ax, ay, band shift, phases' (wb, block)).
#[derive(Clone, Copy, Debug)]
pub struct Record {
    pub w: u16,
    pub h: u16,
    pub ax: u8,
    pub ay: u8,
    pub shift: u8,
    pub bands: u8,
    pub phases: u8,
    pub phase: [(u8, u32, u16); 4],
}

pub fn read_record(reu: &[u8], effect: u8, scale: u8, f: u8) -> Record {
    let r = &reu[record(effect, scale, f) as usize..][..crate::format::FRAME_RECORD as usize];
    let mut phase = [(0, 0, 0); 4];
    for (p, ph) in phase.iter_mut().enumerate() {
        let a = frame::PHASES + p * frame::PHASE_LEN;
        *ph = (
            r[a + phase::WB],
            r[a + phase::BLOCK] as u32 | (r[a + phase::BLOCK + 1] as u32) << 8 | (r[a + phase::BLOCK + 2] as u32) << 16,
            u16::from_le_bytes([r[a + phase::LEN], r[a + phase::LEN + 1]]),
        );
    }
    Record {
        w: u16::from_le_bytes([r[frame::W], r[frame::W + 1]]),
        h: u16::from_le_bytes([r[frame::H], r[frame::H + 1]]),
        ax: r[frame::AX],
        ay: r[frame::AY],
        shift: r[frame::BAND_SHIFT],
        bands: r[frame::BANDS],
        phases: r[frame::PHASE_COUNT],
        phase,
    }
}

/// Draw phase `p` of a frame out of the image in `m` with the C64's sprite code (`sprite::place`,
/// `sprite::sprite_band`, `draw_cols`) into buffer 0's bitmap, once over all-0 bytes and once over
/// all-$FF bytes, and read it back: a pixel that differs is transparent.
pub fn draw_back(m: &mut Machine, effect: u8, scale: u8, f: u8, p: u8) -> SpriteImage {
    let r = read_record(&m.reu, effect, scale, f);
    let (wb, block, _) = r.phase[p as usize];
    let bitmap = BUFFERS[0].bitmap;
    let (left, top) = (8 + p as i16, 4i16);
    let mut planes = [vec![0u8; r.w as usize * r.h as usize], vec![0u8; r.w as usize * r.h as usize]];
    for (k, fill) in [0x00u8, 0xFF].into_iter().enumerate() {
        m.fill(bitmap, crate::floor::VIEW_BANDS * layout::ROW_STRIDE, fill);
        let placed = crate::sprite::place(block, r.shift, wb, r.h, left, top).expect("on screen");
        for band in 0..crate::floor::VIEW_BANDS as u8 {
            crate::sprite::sprite_band(m, &placed, band, bitmap + band as u16 * layout::ROW_STRIDE);
        }
        for y in 0..r.h {
            for x in 0..r.w {
                let (sx, sy) = (left as u16 + x, top as u16 + y);
                let byte = m.peek(bitmap + layout::line_offset(sy) + (sx / 4) * 8);
                planes[k][y as usize * r.w as usize + x as usize] = (byte >> (6 - 2 * (sx & 3))) & 3;
            }
        }
    }
    let mut img = SpriteImage::new(r.w, r.h);
    for (i, px) in img.pixels.iter_mut().enumerate() {
        *px = (planes[0][i] == planes[1][i]).then_some(planes[0][i]);
    }
    img
}
