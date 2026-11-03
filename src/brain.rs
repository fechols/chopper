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

/// The war's units' trees (`roles`, in `format::TREES`): their features, moves and learn nodes.
pub mod unit {
    /// Features (0 or 1), each a symbol of the planner: wounded, foes in the sector, outnumbered there,
    /// weapon ready, armed, able to move; then three fresh random bytes for the learn nodes.
    pub const WOUNDED: usize = 0;
    pub const FOES: usize = 1;
    pub const OUTNUMBERED: usize = 2;
    pub const READY: usize = 3;
    pub const ARMED: usize = 4;
    pub const MOBILE: usize = 5;
    pub const SYMBOLS: usize = 6;
    pub const LEARN0: u8 = 6;
    pub const FEATURES: usize = 9;
    /// Moves (leaves).
    pub const ADVANCE: u8 = 0;
    pub const HOLD: u8 = 1;
    pub const ATTACK: u8 = 2;
    pub const RETREAT: u8 = 3;
    pub const NAMES: [&str; 4] = ["advance", "hold", "attack", "retreat"];
    /// Each kind's learning baseline (i8, the reward × 8 it has come to expect) in the header.
    pub const BASELINES: usize = 48;
}
/// A learn node's odds stay in this band, so neither branch is ever lost.
pub const LEARN_MIN: u8 = 8;
pub const LEARN_MAX: u8 = 248;
/// Room for the units' blob (`format::TREES`; the decide phase works on it in RAM at `layout::BRAIN_RAM`).
pub const UNIT_BYTES: usize = 0x0A00;

/// Walk unit tree `tree` of the blob at `ram` (C64 memory): the move, and the first learn node passed (its blob
/// offset, or 0) with the branch taken (1: yes, the odds' branch). A learn node's feature is a fresh random
/// byte: under the node's odds, yes.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world3"))]
pub fn walk_learn<H: crate::hw::Hw + ?Sized>(hw: &mut H, ram: u16, tree: u8, features: &[u8; unit::FEATURES]) -> (u8, u16, u8) {
    let rd = |hw: &mut H, a: u16| hw.peek(a) as u16 | (hw.peek(a + 1) as u16) << 8;
    let mut at = rd(hw, ram + ROOTS as u16 + 2 * tree as u16);
    let (mut learned, mut took) = (0u16, 0u8);
    for _ in 0..MAX_DEPTH {
        let a = ram.wrapping_add(at);
        let n = [hw.peek(a), hw.peek(a + 1)];
        if n[0] == LEAF {
            return (n[1], learned, took);
        }
        let f = features[n[0] as usize % unit::FEATURES];
        let ge = f >= n[1];
        if n[0] >= unit::LEARN0 && learned == 0 {
            learned = at;
            took = !ge as u8;
        }
        at = rd(hw, a + if ge { 2 } else { 4 });
    }
    (0, learned, took)
}

/// Teach kind `tree`'s learn nodes on a unit's trail (the last two walks' learn node and branch, newest
/// first): `reward` (damage dealt minus taken since its last decision) against the kind's baseline; the
/// choice that came before a better than expected outcome is made likelier, a worse one less likely, the
/// older walk at half the step (Impact's rule).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world3"))]
pub fn learn<H: crate::hw::Hw + ?Sized>(hw: &mut H, ram: u16, tree: u8, trail: &[(u16, u8); 2], reward: i8) {
    let b = ram + (unit::BASELINES + tree as usize) as u16;
    let baseline = hw.peek(b) as i8;
    let adv = (reward as i16 - (baseline as i16 >> 3)).clamp(-127, 127);
    hw.poke(b, (baseline as i16 + adv).clamp(-128, 127) as i8 as u8);
    if adv == 0 {
        return;
    }
    let step = ((adv.unsigned_abs() << 1).min(32)) as u8;
    for (k, &(node, took)) in trail.iter().enumerate() {
        let s = step >> k;
        if node == 0 || s == 0 {
            continue;
        }
        let a = ram.wrapping_add(node + 1);
        let v = hw.peek(a);
        let p = if (took != 0) == (adv > 0) { v.saturating_add(s).min(LEARN_MAX) } else { v.saturating_sub(s).max(LEARN_MIN) };
        hw.poke(a, p);
    }
}
