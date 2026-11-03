//! Snapshots of the C64's state at the start of a frame (host): what `tools/emu.py` captures and
//! `chopper frame --state` steps from, so the host reproduces one frame of a game whose state depends on
//! everything before it (the player's input, the frame rate, the war), not just on the tick.
//!
//! A snapshot is a list of memory ranges: C64 RAM or REU, each `kind` u8 (`RAM` / `REU`), address u32,
//! length u16, then the bytes. [`STATE_RAM`] and [`STATE_REU`] name what the C64 carries from frame to
//! frame; the frame's output is checked over the same ranges afterwards.

use crate::layout;

pub const MAGIC: &[u8; 4] = b"CHSN";
pub const RAM: u8 = 0;
pub const REU: u8 = 1;

/// C64 RAM carried across frames: the agent, its clock, the gauges' frames and the units near; the HUD's
/// lock state.
pub const STATE_RAM: &[(u16, u16)] = &[
    (layout::AGENT, crate::world_sim::PHASE + 1 - layout::AGENT),
    (crate::hud::LOCK_STATE, crate::hud::LOCK_BYTES as u16),
];
/// REU ranges the game writes as it plays: the war.
pub const STATE_REU: &[(u32, u32)] = &[(crate::world_sim::STATE, crate::world_sim::STATE_LEN)];

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Snapshot {
    pub ram: Vec<(u16, Vec<u8>)>,
    pub reu: Vec<(u32, Vec<u8>)>,
}

impl Snapshot {
    pub fn parse(b: &[u8]) -> Result<Self, String> {
        if b.len() < 4 || &b[..4] != MAGIC {
            return Err("not a chopper snapshot".into());
        }
        let mut s = Snapshot::default();
        let mut at = 4;
        while at < b.len() {
            if at + 7 > b.len() {
                return Err("truncated snapshot".into());
            }
            let kind = b[at];
            let addr = u32::from_le_bytes([b[at + 1], b[at + 2], b[at + 3], b[at + 4]]);
            let len = u16::from_le_bytes([b[at + 5], b[at + 6]]) as usize;
            at += 7;
            let bytes = b.get(at..at + len).ok_or("truncated snapshot")?.to_vec();
            at += len;
            match kind {
                RAM => s.ram.push((addr as u16, bytes)),
                REU => s.reu.push((addr, bytes)),
                _ => return Err(format!("snapshot range kind {kind}")),
            }
        }
        Ok(s)
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        let ranges = self.ram.iter().map(|(a, b)| (RAM, *a as u32, b)).chain(self.reu.iter().map(|(a, b)| (REU, *a, b)));
        for (kind, addr, bytes) in ranges {
            out.push(kind);
            out.extend_from_slice(&addr.to_le_bytes());
            out.extend_from_slice(&(bytes.len() as u16).to_le_bytes());
            out.extend_from_slice(bytes);
        }
        out
    }

    /// The bytes at C64 address `addr` (`len` of them), if a range holds them all.
    pub fn ram_at(&self, addr: u16, len: usize) -> Option<&[u8]> {
        self.ram.iter().find_map(|(a, b)| {
            let off = addr.checked_sub(*a)? as usize;
            b.get(off..off + len)
        })
    }
}
