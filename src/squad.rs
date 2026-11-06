//! Platoons near the player as their vehicles: each of the war's records near the player (`world_sim::NEAR`)
//! is a platoon of up to `world_sim::MEMBERS_MAX` vehicles of its kind (`stats::MEMBERS`, less its fallen,
//! `rec::LOST`). Each frame, before the combat pass, [`expand`] places each standing member in its
//! platoon's formation round where the platoon is now (`world_sim::live`), and lists the nearest
//! [`SLOTS`] of them (and the near wrecks, and the carrier's deck) in [`MEMBERS`], nearest first: the
//! combat pass draws the first few, targets the enemies among them, and hits them by their vehicle ids
//! (`world_sim::vehicle`).
//!
//! The formation: a staggered column on the move, a line abreast fighting or holding, a V in the air;
//! spaced by class.

use crate::actors::kind;
use crate::hw::{self, Hw};
use crate::world_sim::{self as ws, ALIVE, RECORD, SIDE, act, rec, stats};

/// The frame's vehicles near the player: a count, then [`SLOTS`] entries' offsets nearest first
/// ([`ORDER`]: from [`ENTRIES`], so no multiply finds one), then the entries ([`ENTRY`] bytes each:
/// `entry`). In the page after the relocated `victext` (RAM to the CPU, never the VIC's).
pub const MEMBERS: u16 = 0x9F00;
pub const SLOTS: u8 = 20;
pub const ORDER: u16 = MEMBERS + 1;
pub const ENTRIES: u16 = ORDER + SLOTS as u16;
pub const ENTRY: u16 = 11;
const _: () = assert!(ENTRIES + SLOTS as u16 * ENTRY <= 0xA000 && SLOTS as u16 * ENTRY <= 256);
/// An entry: the vehicle (u16: `world_sim::vehicle`; `NOBODY` for the deck), then as `combat::OBJLIST`
/// lists an object to draw (mesh, yaw, u, v, h: 1/16 texel), then its distance (texels / 2, at most 255).
pub mod entry {
    pub const VID: u16 = 0;
    pub const MESH: u16 = 2;
    pub const YAW: u16 = 3;
    pub const U: u16 = 4;
    pub const V: u16 = 6;
    pub const H: u16 = 8;
    pub const D: u16 = 10;
}

/// The `k`th nearest's entry.
#[inline(always)]
pub fn nth<H: Hw + ?Sized>(hw: &mut H, k: u8) -> u16 {
    ENTRIES + hw.peek(ORDER + k as u16) as u16
}

/// Formations.
pub const COLUMN: u8 = 0;
pub const LINE: u8 = 1;
pub const VEE: u8 = 2;

/// Member `m`'s place in a formation, (along, across) in half spacings (along: ahead; across: to the
/// right); the leader (0) at the platoon's place. (Worked out, not a table: the overlays' tables would be
/// resident.)
#[inline(always)]
pub fn place(shape: u8, m: u8) -> (i8, i8) {
    let k = m.div_ceil(2) as i8;
    let across = if m & 1 != 0 { 2 * k } else { -2 * k };
    match shape {
        // On the move: one behind the other, a little to each side in turn.
        COLUMN => (-2 * m as i8, (m & 1) as i8),
        // Fighting: abreast, each side in turn, every other pair a little back.
        LINE => (-(k & 1), across),
        // In the air: back from the leader both ways.
        _ => (-2 * k, across),
    }
}

/// A platoon's formation for its kind and what it is doing.
#[inline(always)]
pub fn shape(kind: usize, action: u8) -> u8 {
    if stats::CLASS[kind] == 2 {
        VEE
    } else if action == act::FIGHT || action == act::HOLD {
        LINE
    } else {
        COLUMN
    }
}

/// Half a spacing (1/16 texel): by class, ground 1.5 texels, air 2.5, sea 3, times
/// `actors::UNIT_SCALE` as the units are drawn (ground 3, air 5, sea 6: a truck is 3.6 long).
#[inline(always)]
pub fn half(kind: usize) -> i16 {
    crate::actors::UNIT_SCALE
        * match stats::CLASS[kind] {
            2 => 40,
            4 => 48,
            _ => 24,
        }
}

/// Half a spacing ([`half`]) ahead along heading `yaw` (1/16 texel, u and v).
#[inline(always)]
pub fn basis(kind: usize, yaw: u16) -> (i16, i16) {
    let sp = half(kind);
    (crate::fixed::mul14(crate::camera::sin(yaw) as i16, sp), crate::fixed::mul14(crate::camera::cos(yaw) as i16, sp))
}

/// `x` times a small `k`, by shifts and adds.
#[inline(always)]
fn times(mut x: i16, k: i8) -> i16 {
    let mut n = k.unsigned_abs();
    let mut a = 0i16;
    while n != 0 {
        if n & 1 != 0 {
            a = a.wrapping_add(x);
        }
        x = x.wrapping_shl(1);
        n >>= 1;
    }
    if k < 0 { a.wrapping_neg() } else { a }
}

/// Member `m`'s place from its platoon's (1/16 texel), in formation `shape` with half a spacing ahead
/// (`fu`, `fv`): to the right is (`fv`, -`fu`).
#[inline(always)]
pub fn spot(fu: i16, fv: i16, shape: u8, m: u8) -> (i16, i16) {
    let (al, ac) = place(shape, m % ws::MEMBERS_MAX);
    (times(fu, al).wrapping_add(times(fv, ac)), times(fv, al).wrapping_sub(times(fu, ac)))
}

/// Member `m`'s place from its platoon's (1/16 texel), the platoon of `kind` doing `action` and heading
/// `yaw`.
#[inline(always)]
pub fn offset(kind: usize, action: u8, yaw: u16, m: u8) -> (i16, i16) {
    let (fu, fv) = basis(kind, yaw);
    spot(fu, fv, shape(kind, action), m)
}

#[inline(always)]
fn rd<H: Hw + ?Sized>(hw: &mut H, a: u16) -> u16 {
    hw.peek(a) as u16 | (hw.peek(a + 1) as u16) << 8
}

#[inline(always)]
fn wr<H: Hw + ?Sized>(hw: &mut H, a: u16, w: u16) {
    hw.poke(a, w as u8);
    hw.poke(a + 1, (w >> 8) as u8);
}

/// The near platoons' vehicles into [`MEMBERS`] at tick `now`, the player at texels (`pu`, `pv`) heading
/// `yaw`; the near wrecks' smoke into `combat::WRECKS`. A platoon behind the player is listed as
/// [`BEHIND`] further off: the slots go to what is in view first.
///
/// Two passes: each platoon where it is now ([`PLATOONS`], in order of distance), then their members,
/// the nearest platoons first: once the list is full, a platoon none of whose members could be nearer
/// than its farthest is passed over whole.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.squad"))]
pub fn expand<H: Hw + ?Sized>(hw: &mut H, now: u16, pu: i16, pv: i16, yaw: u16) {
    let (fs, fc) = (crate::camera::sin(yaw) as i16, crate::camera::cos(yaw) as i16);
    use crate::combat::{WRECK_ENTRY, WRECK_MAX, WRECKS};
    hw.poke(MEMBERS, 0);
    let mut wrecks = 0u8;
    // Platoons for the second pass, and the next one's offset in `PLATOONS`.
    let (mut np, mut at) = (0u8, 0u8);
    let n = hw.peek(ws::NEAR).min(ws::NEAR_MAX);
    let mut r = [0u8; RECORD as usize];
    for k in 0..n as u16 {
        let id = rd(hw, ws::NEAR + 1 + 2 * k);
        hw::read_into(hw, ws::UNITS + id as u32 * RECORD as u32, crate::layout::BOUNCE, &mut r);
        let head: &[u8; ws::CARRIER_RECORD] = r[..ws::CARRIER_RECORD].try_into().unwrap();
        // Where it is now: moved on from its last step (the dead stay where they fell).
        let (u, v, yaw) = ws::live_inline(head, now);
        let flags = r[rec::FLAGS as usize];
        let kd = r[rec::KIND as usize] % kind::COUNT;
        let h = (r[rec::H as usize] as u16) << 4;
        let y8 = (yaw >> 8) as u8;
        // Behind the player (by more than a formation's length)?
        let (dx, dy) = ((u >> 4) as i16 - pu, (v >> 4) as i16 - pv);
        let back = if crate::fixed::mul14(fs, dx) + crate::fixed::mul14(fc, dy) < -SPAN { BEHIND } else { 0 };
        let d = ((ws::dist(dx, dy) >> 1).min(255) as u8).saturating_add(back);
        if flags & ALIVE == 0 {
            put(hw, ws::vehicle(id, ws::WRECK_MEMBER), crate::actors::wreck_of(kd), y8, u, v, h, d);
            // A wreck: its smoke column, while it lasts (rising from a texel and a half up, times
            // `actors::UNIT_SCALE` as the wreck is drawn: the wreck shows under it).
            let age = now.wrapping_sub(u16::from_le_bytes([r[16], r[17]]));
            if age < ws::WRECK_TICKS && wrecks < WRECK_MAX {
                let w = WRECKS + 1 + wrecks as u16 * WRECK_ENTRY;
                wr(hw, w, u);
                wr(hw, w + 2, v);
                wr(hw, w + 4, h + 24 * crate::actors::UNIT_SCALE as u16);
                wr(hw, w + 6, age);
                wrecks += 1;
            }
            continue;
        }
        let mesh = crate::actors::mesh_of(kd, flags & SIDE);
        if kd == kind::CARRIER {
            // Its flight deck with it: both first (one lands on it).
            put(hw, id, mesh, y8, u, v, h, 0);
            put(hw, ws::NOBODY, crate::actors::DECK, y8, u, v, h, 0);
            continue;
        }
        // The platoon for the second pass, in order of its distance.
        let p = PLATOONS + at as u16;
        wr(hw, p + plat::ID, id);
        wr(hw, p + plat::U, u);
        wr(hw, p + plat::V, v);
        wr(hw, p + plat::YAW, yaw);
        wr(hw, p + plat::H, h);
        hw.poke(p + plat::KIND, kd);
        hw.poke(p + plat::MESH, mesh);
        hw.poke(p + plat::BACK, back);
        hw.poke(p + plat::D, d);
        wr(hw, p + plat::LOST, u16::from_le_bytes([r[rec::LOST as usize], r[rec::LOST as usize + 1]]));
        hw.poke(p + plat::FORM, shape(kd as usize, r[rec::ACTION as usize]));
        let mut i = np;
        while i > 0 {
            let prev = hw.peek(PLATOON_ORDER + i as u16 - 1);
            if d >= hw.peek(PLATOONS + prev as u16 + plat::D) {
                break;
            }
            hw.poke(PLATOON_ORDER + i as u16, prev);
            i -= 1;
        }
        hw.poke(PLATOON_ORDER + i as u16, at);
        np += 1;
        at += PLATOON as u8;
    }
    hw.poke(WRECKS, wrecks);
    for j in 0..np {
        let p = PLATOONS + hw.peek(PLATOON_ORDER + j as u16) as u16;
        let kd = hw.peek(p + plat::KIND) as usize;
        let members = stats::members(kd);
        // None of it nearer than the farthest listed: passed over.
        if far(hw) <= hw.peek(p + plat::D).saturating_sub(members * spacing(kd) + 4) {
            continue;
        }
        let (u, v, yaw) = (rd(hw, p + plat::U), rd(hw, p + plat::V), rd(hw, p + plat::YAW));
        let (id, h, lost) = (rd(hw, p + plat::ID), rd(hw, p + plat::H), rd(hw, p + plat::LOST));
        let (mesh, back, form) = (hw.peek(p + plat::MESH), hw.peek(p + plat::BACK), hw.peek(p + plat::FORM));
        let (fu, fv) = basis(kd, yaw);
        for m in 0..members {
            if lost & (1 << m) != 0 {
                continue;
            }
            let (du, dv) = spot(fu, fv, form, m);
            let (mu, mv) = (u.wrapping_add(du as u16), v.wrapping_add(dv as u16));
            let d = ((ws::dist((mu >> 4) as i16 - pu, (mv >> 4) as i16 - pv) >> 1).min(255) as u8).saturating_add(back);
            if d < far(hw) {
                put(hw, ws::vehicle(id, m), mesh, (yaw >> 8) as u8, mu, mv, h, d);
            }
        }
    }
}

/// The near platoons where they are now, for `expand`'s second pass: [`PLATOON`] bytes each (`plat`),
/// then their offsets in order of distance. (In the war's page, free in the set-up once the war's step
/// is done.)
const PLATOONS: u16 = ws::PAGE_RAM[0];
const PLATOON: u16 = 18;
const PLATOON_ORDER: u16 = PLATOONS + ws::NEAR_MAX as u16 * PLATOON;
const _: () = assert!(PLATOON_ORDER + ws::NEAR_MAX as u16 <= PLATOONS + ws::HALF * RECORD && ws::NEAR_MAX as u16 * PLATOON <= 256);
mod plat {
    pub const ID: u16 = 0;
    pub const U: u16 = 2;
    pub const V: u16 = 4;
    pub const YAW: u16 = 6;
    pub const H: u16 = 8;
    pub const KIND: u16 = 10;
    pub const MESH: u16 = 11;
    pub const BACK: u16 = 12;
    pub const D: u16 = 13;
    pub const LOST: u16 = 14;
    pub const FORM: u16 = 16;
}

/// How far (texels / 2) a member of a platoon of `kind` can be from its place, per member: its half
/// spacing ([`half`]) in texels, and one more for `ws::dist`'s overestimate.
#[inline(always)]
fn spacing(kind: usize) -> u8 {
    (half(kind) >> 4) as u8 + 1
}

/// The farthest listed vehicle's distance (texels / 2): 255 while the list has room.
#[inline(always)]
fn far<H: Hw + ?Sized>(hw: &mut H) -> u8 {
    if hw.peek(MEMBERS) < SLOTS {
        return 255;
    }
    let last = hw.peek(ORDER + SLOTS as u16 - 1);
    hw.peek(ENTRIES + last as u16 + entry::D)
}

/// How much further off (texels / 2) a platoon behind the player is listed, and how far behind (texels)
/// it is.
const BEHIND: u8 = 96;
const SPAN: i16 = 16;

/// A vehicle into [`MEMBERS`] in order of its distance `d` from the player (texels / 2), if it is among
/// the nearest [`SLOTS`] (the farthest given way to it).
#[allow(clippy::too_many_arguments)]
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.squad"))]
fn put<H: Hw + ?Sized>(hw: &mut H, vid: u16, mesh: u8, yaw: u8, u: u16, v: u16, h: u16, d: u8) {
    let n = hw.peek(MEMBERS);
    let (slot, mut i) = if n < SLOTS {
        hw.poke(MEMBERS, n + 1);
        // (n · 11 by shifts.)
        const _: () = assert!(ENTRY == 11);
        ((n << 3) + (n << 1) + n, n)
    } else {
        // Full: in place of the farthest, if nearer.
        let last = hw.peek(ORDER + SLOTS as u16 - 1);
        if d >= hw.peek(ENTRIES + last as u16 + entry::D) {
            return;
        }
        (last, SLOTS - 1)
    };
    while i > 0 {
        let prev = hw.peek(ORDER + i as u16 - 1);
        if d >= hw.peek(ENTRIES + prev as u16 + entry::D) {
            break;
        }
        hw.poke(ORDER + i as u16, prev);
        i -= 1;
    }
    hw.poke(ORDER + i as u16, slot);
    let e = ENTRIES + slot as u16;
    wr(hw, e + entry::VID, vid);
    hw.poke(e + entry::MESH, mesh);
    hw.poke(e + entry::YAW, yaw);
    wr(hw, e + entry::U, u);
    wr(hw, e + entry::V, v);
    wr(hw, e + entry::H, h);
    hw.poke(e + entry::D, d);
}

/// Vehicle `vid`'s entry in [`MEMBERS`] this frame, if listed.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn find<H: Hw + ?Sized>(hw: &mut H, vid: u16) -> Option<u16> {
    let n = hw.peek(MEMBERS).min(SLOTS);
    (0..n as u16).map(|k| ENTRIES + k * ENTRY).find(|&e| rd(hw, e + entry::VID) == vid)
}
