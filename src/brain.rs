//! Baked AI decision trees (`goap::compile` on the host), walked from the REU at run time, in the format
//! of `../impact`'s CPU-fighter brain (`impact/src/brain.rs`, the same author, MIT):
//!
//! - The blob begins with a [`HEADER_BYTES`] header: magic `CHAI`, a u16 length, then one u16 root offset
//!   per tree ([`TREES`]: the attract-mode pilot first; enemies will follow).
//! - A node is six bytes `[feature, threshold, ge: u16, lt: u16]`. The walk moves to blob offset `ge` when
//!   `features[feature] >= threshold`, otherwise to `lt`. Children always lie before their parent (lower
//!   offsets), so every walk ends; identical subtrees are shared. A `feature` of [`LEAF`] ends the walk,
//!   and `threshold` is then the action.
//! - The walker fetches one node per step (impact's fetches a 48-byte window ending at the node, to save
//!   DMAs on a 1 MHz machine; at 64 MHz the six bytes of RAM in the static frames matter more).

/// The blob's magic.
pub const MAGIC: [u8; 4] = *b"CHAI";
/// Header bytes (nodes start here, so a window never reaches below offset 0).
pub const HEADER_BYTES: usize = 64;
/// The trees' root offsets, u16 each, from this header offset.
pub const ROOTS: usize = 6;
pub const TREES: usize = 1;
/// The attract-mode autopilot's tree.
pub const PILOT: usize = 0;
pub const NODE_BYTES: usize = 6;
/// A walk deeper than this returns action 0, so a damaged blob cannot hang the C64.
pub const MAX_DEPTH: u8 = 40;
pub const LEAF: u8 = 0xFF;
/// Room for the blob in the REU (`format::AI_TREES`).
pub const MAX_BYTES: usize = 0x8000;

/// REU reads, as the C64 does them (`hw::fetch`) or from a baked image on the host.
pub trait Reu {
    fn read(&mut self, addr: u32, out: &mut [u8]);
}

impl Reu for &[u8] {
    fn read(&mut self, addr: u32, out: &mut [u8]) {
        out.copy_from_slice(&self[addr as usize..addr as usize + out.len()]);
    }
}

/// Walk tree `tree` of the blob at REU `base` over `features`: the leaf's action.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
pub fn walk<R: Reu + ?Sized>(reu: &mut R, base: u32, tree: usize, features: &[u8]) -> u8 {
    let mut root = [0u8; 2];
    reu.read(base + (ROOTS + 2 * tree) as u32, &mut root);
    let mut at = u16::from_le_bytes(root);
    let mut n = [0u8; NODE_BYTES];
    for _ in 0..MAX_DEPTH {
        reu.read(base + at as u32, &mut n);
        if n[0] == LEAF {
            return n[1];
        }
        let f = features.get(n[0] as usize).copied().unwrap_or(0);
        at = if f >= n[1] { u16::from_le_bytes([n[2], n[3]]) } else { u16::from_le_bytes([n[4], n[5]]) };
    }
    0
}
