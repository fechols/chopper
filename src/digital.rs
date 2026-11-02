//! DLT5 digital audio (host): the rotor sound, its header, and the reference decoder the C64's
//! `asm/digital.S` must match byte for byte. The format and the decode rule come from `../sueme64`
//! (`src/digital.rs`, `tools/encode-digital-music`; see THIRD-PARTY.md); chopper uses mono only.
//!
//! A DLT5 file: a 32-byte header ("DLT5", rate u32, frames u32, stream bytes u32, entries per book u16
//! = 8, books u16, bits u8 = 3, channels u8 = 1, ten zero bytes), the books (8 × i16 deltas each), the
//! code stream (3-bit codes, least significant bit first, 8 codes in 3 bytes), then a CRC-8 per 1 KiB of
//! stream. Each sample is `pred = pred +sat book[code]` (`pred` starts at 0), published as
//! `(pred + 128) >> 8` (signed 8-bit). The frames are split into segments of whole 8-frame groups, one
//! book each: the first `groups % books` segments one group longer.

/// The rotor sound (`tools/make-rotor.py`): 16 kHz mono, 0:00-1:45 of the source with its loop.
pub const ROTOR: &[u8] = include_bytes!("../Assets/Sound_Effects/rotor.dlt");
/// The loop (`Assets/Sound_Effects/rotor.json`): after frame `ROTOR_LOOP_END - 1` the sampler goes back
/// to `ROTOR_LOOP_START`. The end of the file is the end of the loop.
pub const ROTOR_LOOP_START: u32 = 1_275_670;
pub const ROTOR_LOOP_END: u32 = 1_687_808;

pub const HEADER: usize = 32;
pub const BOOK: usize = 16;
pub const GROUP: u32 = 8;

/// A mono DLT5 file's header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Dlt5 {
    pub rate: u32,
    pub frames: u32,
    pub stream_bytes: u32,
    pub books: u16,
    /// Frames in a short segment; the first `long` segments have `GROUP` more.
    pub per_segment: u32,
    pub long: u16,
}

impl Dlt5 {
    pub fn parse(b: &[u8]) -> Option<Self> {
        let u32_at = |o: usize| u32::from_le_bytes(b[o..o + 4].try_into().unwrap());
        let u16_at = |o: usize| u16::from_le_bytes([b[o], b[o + 1]]);
        if b.len() < HEADER || &b[..4] != b"DLT5" || u16_at(16) != 8 || b[20] != 3 || b[21] != 1 || b[22..32].iter().any(|&x| x != 0) {
            return None;
        }
        let (frames, books) = (u32_at(8), u16_at(18));
        let groups = frames.div_ceil(GROUP);
        if books == 0 || books as u32 > groups {
            return None;
        }
        let d = Dlt5 {
            rate: u32_at(4),
            frames,
            stream_bytes: u32_at(12),
            books,
            per_segment: groups / books as u32 * GROUP,
            long: (groups % books as u32) as u16,
        };
        (d.stream_bytes == (frames * 3).div_ceil(8) && b.len() >= d.stream_offset() + d.stream_bytes as usize).then_some(d)
    }

    pub fn stream_offset(&self) -> usize {
        HEADER + self.books as usize * BOOK
    }

    /// Frames in segment `s`.
    pub fn segment_len(&self, s: u16) -> u32 {
        self.per_segment + if s < self.long { GROUP } else { 0 }
    }
}

/// The published sample of a predictor.
pub fn rounded(pred: i16) -> i8 {
    ((pred as i32 + 128) >> 8).min(127) as i8
}

/// Decode a whole mono DLT5 file to signed 8-bit PCM.
pub fn decode(b: &[u8]) -> Vec<i8> {
    let d = Dlt5::parse(b).expect("not a mono DLT5 file");
    let mut out = Vec::with_capacity(d.frames as usize);
    let mut at = d.stream_offset();
    let (mut bits, mut nbits) = (0u32, 0u32);
    let mut pred = 0i16;
    for s in 0..d.books {
        let book: [i16; 8] = core::array::from_fn(|k| {
            let o = HEADER + s as usize * BOOK + 2 * k;
            i16::from_le_bytes([b[o], b[o + 1]])
        });
        let n = d.segment_len(s).min(d.frames - out.len() as u32);
        for _ in 0..n {
            if nbits < 3 {
                bits |= (b[at] as u32) << nbits;
                at += 1;
                nbits += 8;
            }
            pred = pred.saturating_add(book[(bits & 7) as usize]);
            bits >>= 3;
            nbits -= 3;
            out.push(rounded(pred));
        }
    }
    assert_eq!(out.len(), d.frames as usize);
    out
}
