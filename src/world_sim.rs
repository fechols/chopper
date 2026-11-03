//! The living world: a war on the island that goes on wherever the player is, kept in the REU as a page
//! file (`format::WORLD`) and stepped round-robin.
//!
//! - [`UNITS`]: [`COUNT`] records of [`RECORD`] bytes, both sides' units (`rec` offsets).
//! - [`SECTORS`]: 16×16 sectors of 128 texels, [`SECTOR`] bytes each: how many of each side's units are
//!   in it, and each side's representative there (the unit of that side stepped there last: whom the
//!   other side's units in the sector fight).
//! - [`ROUTES`]: roads, sea lanes and air patrols as waypoint lists (baked by `war`).
//! - [`HEADER`]: the next page, the director's wave clock and budgets, each side's units alive.
//!
//! Each frame [`step_page`] pages [`PAGE`] records into RAM ([`PAGE_RAM`]: two halves, free during the
//! set-up) and runs one of two phases over them, each its own code overlay, in turn (round-robin: one
//! overlay a frame):
//! - **move** (`.text.world`): along its route since its last step, its sector's counts kept, listed if
//!   near the player; a dead unit respawned if its side has reinforcements due;
//! - **fight** (`.text.world2`): it represents its side in its sector and fights the other side's
//!   representative there by dice; the director, once round the file.
//!
//! So every unit moves and fights about once a second and a half: a human reaction time, by design.
//! Units near the player are listed in [`NEAR`] for the actors (drawn and, later, fought up close).

use crate::format::WORLD;
use crate::hw::{self, Hw};

pub const UNITS: u32 = WORLD;
pub const COUNT: u16 = 480;
pub const RECORD: u16 = 32;
pub const SECTORS: u32 = WORLD + 0x4000;
pub const SECTOR: u16 = 8;
pub const ROUTES: u32 = WORLD + 0x4800;
pub const HEADER: u32 = WORLD + 0x5000;
/// The REU the war writes as it runs (the snapshot gate carries it).
pub const STATE: u32 = WORLD;
pub const STATE_LEN: u32 = 0x5000 + HEADER_LEN as u32;

/// A unit's record.
pub mod rec {
    pub const KIND: u16 = 0;
    /// `FLAGS` bits below.
    pub const FLAGS: u16 = 1;
    pub const HP: u16 = 2;
    /// What it is doing (`act`).
    pub const ACTION: u16 = 3;
    /// Position, 1/16 texel (u16), height in texels (u8), heading (256 a turn), speed (1/16 texel a tick).
    pub const U: u16 = 4;
    pub const V: u16 = 6;
    pub const H: u16 = 8;
    pub const YAW: u16 = 9;
    pub const SPEED: u16 = 10;
    pub const ROUTE: u16 = 11;
    pub const LEG: u16 = 12;
    pub const COOL: u16 = 13;
    /// The unit last fought (u16; `NOBODY`).
    pub const TARGET: u16 = 14;
    /// The tick of its last step (u16, wrapping).
    pub const LAST: u16 = 16;
    /// Damage dealt and taken since its last decision (for learning).
    pub const DEALT: u16 = 19;
    pub const TAKEN: u16 = 20;
    pub const SEED: u16 = 21;
    /// Its sector (`sx | sy << 4`).
    pub const SECTOR: u16 = 28;
}
pub const ALIVE: u8 = 0x80;
pub const NEAR_FLAG: u8 = 0x40;
pub const SIDE: u8 = 0x01;
pub const NOBODY: u16 = 0xFFFF;

/// What a unit is doing.
pub mod act {
    pub const MOVE: u8 = 0;
    pub const FIGHT: u8 = 1;
    pub const HOLD: u8 = 2;
}

/// A sector: units of each side (u8 × 2), each side's representative (u16 × 2).
pub mod sec {
    pub const COUNT: u16 = 0;
    pub const REP: u16 = 2;
}

/// Routes: [`ROUTE_COUNT`] entries of (first waypoint u8, waypoints u8, mode u8, spare), then waypoints
/// (u, v texels, u16 each).
pub const ROUTE_COUNT: u16 = 16;
pub const ROUTE_ENTRY: u16 = 4;
pub const WAYPOINTS: u32 = ROUTES + ROUTE_COUNT as u32 * ROUTE_ENTRY as u32;
/// Route modes: back and forth along it, or round it.
pub const PING_PONG: u8 = 0;
pub const LOOP: u8 = 1;

/// The header: next page (u16), the wave clock (u16 ticks), per side: wave budget (u8 × 2), alive (u16 ×
/// 2), units lost (u16 × 2).
pub mod head {
    pub const CURSOR: u16 = 0;
    pub const WAVE: u16 = 2;
    pub const BUDGET: u16 = 4;
    pub const ALIVE: u16 = 6;
    pub const LOST: u16 = 10;
}
pub const HEADER_LEN: u16 = 16;

/// Records a page (`COUNT` is a whole number of pages).
pub const PAGE: u16 = 48;
pub const HALF: u16 = PAGE / 2;
const _: () = assert!(COUNT % PAGE == 0 && COUNT as u32 * RECORD as u32 <= SECTORS - UNITS);
/// The page in RAM: two halves free during the set-up (`geom::Verts`, rebuilt in the geometry; the ray
/// table, rebuilt by `bt_build` after the set-up).
pub const PAGE_RAM: [u16; 2] = [crate::layout::VERTS, crate::layout::BT];
const _: () = assert!(crate::layout::VERTS as u32 + HALF as u32 * RECORD as u32 <= 0x9B00);
const _: () = assert!(HALF as u32 * RECORD as u32 <= 0x300);

/// Units near the player (the actors draw them): a count, then their ids (u16); then the phase the next
/// page runs (`MOVE` / `FIGHT`: which overlay the main loop fetches).
pub const NEAR: u16 = crate::layout::GAUGES + crate::cockpit::gauge::COUNT as u16;
pub const NEAR_MAX: u8 = 12;
pub const NEAR_LEN: u16 = 1 + 2 * NEAR_MAX as u16;
pub const PHASE: u16 = NEAR + NEAR_LEN;
pub const MOVE: u8 = 0;
pub const FIGHT: u8 = 1;
const _: () = assert!(PHASE + 1 <= 0xCB00);
/// Within this many texels of the player (each way).
pub const NEAR_RANGE: i16 = 320;

/// Per kind: hit points, speed (1/16 texel a tick), the chance of a hit a second (of 256) and its damage,
/// the reload (ticks), and what it can hit (bit 0 ground, bit 1 air).
pub mod stats {
    use crate::actors::kind::COUNT;
    //                                   truck apc tank aa  sam radar gunb msl  gun  dep  bunk carr dest tr  msl
    pub const HP: [u8; COUNT as usize] = [8, 20, 40, 16, 12, 10, 24, 20, 24, 30, 60, 250, 120, 1, 1];
    pub const SPEED: [u8; COUNT as usize] = [2, 2, 1, 1, 0, 0, 2, 2, 6, 0, 0, 1, 2, 0, 0];
    pub const HIT: [u8; COUNT as usize] = [0, 90, 120, 140, 100, 0, 90, 80, 110, 0, 0, 60, 120, 0, 0];
    pub const DAMAGE: [u8; COUNT as usize] = [0, 3, 8, 4, 12, 0, 3, 10, 6, 0, 0, 4, 10, 0, 0];
    /// Fights between shots (each about a second and a half).
    pub const RELOAD: [u8; COUNT as usize] = [0, 0, 1, 0, 2, 0, 0, 2, 0, 0, 0, 0, 1, 0, 0];
    pub const TARGETS: [u8; COUNT as usize] = [0, 1, 1, 2, 2, 0, 3, 2, 3, 0, 0, 2, 3, 0, 0];
    /// Kinds that fly.
    pub const AIR: u8 = crate::actors::kind::GUNSHIP;
}

/// A record's byte or word in the page.
#[inline(always)]
fn at(slot: u16) -> u16 {
    if slot < HALF { PAGE_RAM[0] + slot * RECORD } else { PAGE_RAM[1] + (slot - HALF) * RECORD }
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

/// One byte of the REU (through the bounce buffer), and back.
#[inline(always)]
fn reu_get<H: Hw + ?Sized>(hw: &mut H, addr: u32) -> u8 {
    let mut b = [0u8; 1];
    hw::read_into(hw, addr, crate::layout::BOUNCE, &mut b);
    b[0]
}

#[inline(always)]
fn reu_put<H: Hw + ?Sized>(hw: &mut H, addr: u32, v: u8) {
    hw.poke(crate::layout::BOUNCE, v);
    hw::stash(hw, crate::layout::BOUNCE, addr, 1);
}

#[inline(always)]
fn reu_get16<H: Hw + ?Sized>(hw: &mut H, addr: u32) -> u16 {
    reu_get(hw, addr) as u16 | (reu_get(hw, addr + 1) as u16) << 8
}

#[inline(always)]
fn reu_put16<H: Hw + ?Sized>(hw: &mut H, addr: u32, w: u16) {
    reu_put(hw, addr, w as u8);
    reu_put(hw, addr + 1, (w >> 8) as u8);
}

/// An 8-bit xorshift.
#[inline(always)]
fn roll(s: u8) -> u8 {
    let mut x = if s == 0 { 0xA5 } else { s };
    x ^= x << 3;
    x ^= x >> 5;
    x ^ x << 4
}

/// The heading (256 a turn, clockwise from north as the agent's) of a direction, to about 6°.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world"))]
pub fn heading(dx: i16, dy: i16) -> u8 {
    let (ax, ay) = (dx.unsigned_abs(), dy.unsigned_abs());
    // The angle from the nearer axis, in 32nds of a quarter turn, from the ratio (0..=32 over an octant).
    let (lo, hi) = if ax < ay { (ax, ay) } else { (ay, ax) };
    let r = if hi == 0 { 0 } else { ((lo as u32 * 32) / hi as u32) as u8 };
    let a = if ax < ay { r } else { 64 - r };
    // Into the quadrant: north is +v (dy > 0), east +u.
    match (dx >= 0, dy >= 0) {
        (true, true) => a,
        (true, false) => 128 - a,
        (false, false) => 128 + a,
        (false, true) => (256 - a as u16) as u8,
    }
}

/// The page of `PAGE` records from `first` into RAM, and back.
#[inline(always)]
fn page<H: Hw + ?Sized>(hw: &mut H, first: u16, store: bool) {
    for half in 0..2u16 {
        let addr = UNITS + (first + half * HALF) as u32 * RECORD as u32;
        if store {
            hw::stash(hw, PAGE_RAM[half as usize], addr, HALF * RECORD);
        } else {
            hw::fetch(hw, addr, PAGE_RAM[half as usize], HALF * RECORD);
        }
    }
}

/// The next page's phase at tick `now`, the player at texels (`pu`, `pv`): the main loop has fetched
/// that phase's overlay (`PHASE`).
pub fn step_page<H: Hw + ?Sized>(hw: &mut H, now: u16, pu: i16, pv: i16) {
    if hw.peek(PHASE) == MOVE {
        move_page(hw, now, pu, pv);
    } else {
        fight_page(hw, now);
    }
}

/// The move phase over the next page.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world"))]
pub fn move_page<H: Hw + ?Sized>(hw: &mut H, now: u16, pu: i16, pv: i16) {
    let first = reu_get16(hw, HEADER + head::CURSOR as u32);
    page(hw, first, false);
    for slot in 0..PAGE {
        move_unit(hw, first, slot, now, pu, pv);
    }
    page(hw, first, true);
    hw.poke(PHASE, FIGHT);
}

/// The fight phase over the same page, then on to the next.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
pub fn fight_page<H: Hw + ?Sized>(hw: &mut H, now: u16) {
    let first = reu_get16(hw, HEADER + head::CURSOR as u32);
    page(hw, first, false);
    for slot in 0..PAGE {
        fight_unit(hw, first, slot);
    }
    page(hw, first, true);
    let next = (first + PAGE) % COUNT;
    reu_put16(hw, HEADER + head::CURSOR as u32, next);
    if next == 0 {
        director(hw, now);
    }
    hw.poke(PHASE, MOVE);
}

/// The director, once round the whole page file: every `WAVE_TICKS`, each side behind its strength gets
/// a wave (budget) of reinforcements, spent as dead slots of its side come round.
pub const WAVE_TICKS: u16 = 60 * 60;
pub const STRENGTH: [u16; 2] = [180, 150];
pub const WAVE: u8 = 12;

#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
fn director<H: Hw + ?Sized>(hw: &mut H, now: u16) {
    let last = reu_get16(hw, HEADER + head::WAVE as u32);
    if now.wrapping_sub(last) < WAVE_TICKS {
        return;
    }
    reu_put16(hw, HEADER + head::WAVE as u32, now);
    for side in 0..2u32 {
        let alive = reu_get16(hw, HEADER + head::ALIVE as u32 + 2 * side);
        if alive < STRENGTH[side as usize] {
            reu_put(hw, HEADER + head::BUDGET as u32 + side, WAVE);
        }
    }
}

/// A unit's sector's record in the REU.
#[inline(always)]
fn sector_at(sector: u8) -> u32 {
    SECTORS + sector as u32 * SECTOR as u32
}

/// Count a unit of `side` in or out of a sector, and of the living.
#[inline(always)]
fn census<H: Hw + ?Sized>(hw: &mut H, sector: u8, side: u8, add: bool) {
    let a = sector_at(sector) + sec::COUNT as u32 + side as u32;
    let n = reu_get(hw, a);
    reu_put(hw, a, if add { n.saturating_add(1) } else { n.saturating_sub(1) });
}

/// The living count of a side, up or down.
#[inline(always)]
fn living<H: Hw + ?Sized>(hw: &mut H, side: u8, add: bool) {
    let a = HEADER + head::ALIVE as u32 + 2 * side as u32;
    let n = reu_get16(hw, a);
    reu_put16(hw, a, if add { n + 1 } else { n.saturating_sub(1) });
    if !add {
        let l = HEADER + head::LOST as u32 + 2 * side as u32;
        let n = reu_get16(hw, l);
        reu_put16(hw, l, n + 1);
    }
}

/// Waypoint `k` of route `r` (texels), and the route's length and mode.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world"))]
fn route<H: Hw + ?Sized>(hw: &mut H, r: u8) -> (u8, u8, u8) {
    let mut e = [0u8; ROUTE_ENTRY as usize];
    hw::read_into(hw, ROUTES + (r as u32 & 15) * ROUTE_ENTRY as u32, crate::layout::BOUNCE, &mut e);
    (e[0], e[1], e[2])
}

#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world"))]
fn waypoint<H: Hw + ?Sized>(hw: &mut H, k: u8) -> (i16, i16) {
    let mut w = [0u8; 4];
    hw::read_into(hw, WAYPOINTS + k as u32 * 4, crate::layout::BOUNCE, &mut w);
    (u16::from_le_bytes([w[0], w[1]]) as i16, u16::from_le_bytes([w[2], w[3]]) as i16)
}

/// One unit's move: respawn if dead and due; else along its route; its sector kept; listed if near.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world"))]
fn move_unit<H: Hw + ?Sized>(hw: &mut H, first: u16, slot: u16, now: u16, pu: i16, pv: i16) {
    let r = at(slot);
    let id = first + slot;
    let flags = hw.peek(r + rec::FLAGS);
    let side = flags & SIDE;
    let kind = hw.peek(r + rec::KIND) as usize % crate::actors::kind::COUNT as usize;
    let dt = now.wrapping_sub(rd(hw, r + rec::LAST)).min(255);
    wr(hw, r + rec::LAST, now);
    let route_no = hw.peek(r + rec::ROUTE);
    if flags & ALIVE == 0 {
        // A dead slot: respawn at its route's start if its side has reinforcements due.
        let budget = HEADER + head::BUDGET as u32 + side as u32;
        let b = reu_get(hw, budget);
        if b == 0 || route_no == 0xFF {
            return;
        }
        reu_put(hw, budget, b - 1);
        let (start, _, _) = route(hw, route_no);
        let (u, v) = waypoint(hw, start);
        wr(hw, r + rec::U, (u as u16) << 4);
        wr(hw, r + rec::V, (v as u16) << 4);
        hw.poke(r + rec::LEG, 0);
        hw.poke(r + rec::HP, stats::HP[kind]);
        hw.poke(r + rec::FLAGS, (flags | ALIVE) & !NEAR_FLAG);
        hw.poke(r + rec::ACTION, act::MOVE);
        let sector = sector_of(hw, r);
        hw.poke(r + rec::SECTOR, sector);
        census(hw, sector, side, true);
        living(hw, side, true);
        return;
    }
    let speed = stats::SPEED[kind];
    if speed > 0 && route_no != 0xFF && hw.peek(r + rec::ACTION) != act::HOLD {
        let (start, len, mode) = route(hw, route_no);
        let mut leg = hw.peek(r + rec::LEG);
        // Round a loop; along a road and back (legs len.. take its waypoints in reverse).
        let len = len.max(1);
        let k = if mode == LOOP || leg < len { leg % len } else { (2 * len - 2).wrapping_sub(leg) % len };
        let (tu, tv) = waypoint(hw, start + k);
        let (u, v) = (rd(hw, r + rec::U), rd(hw, r + rec::V));
        let (dx, dy) = (tu - (u >> 4) as i16, tv - (v >> 4) as i16);
        let (ax, ay) = (dx.unsigned_abs(), dy.unsigned_abs());
        let dist = ax.max(ay) + (ax.min(ay) >> 1);
        // 1/16 texels this step, along the heading.
        let step = speed as u16 * dt;
        let yaw = heading(dx, dy);
        hw.poke(r + rec::YAW, yaw);
        if dist <= (step >> 4) + 1 {
            wr(hw, r + rec::U, (tu as u16) << 4);
            wr(hw, r + rec::V, (tv as u16) << 4);
            leg = leg.wrapping_add(1);
            let span = if mode == LOOP { len } else { (2 * len).saturating_sub(2).max(1) };
            hw.poke(r + rec::LEG, if leg >= span { 0 } else { leg });
        } else {
            let a = (yaw as u16) << 8;
            let du = crate::hud::mul16(crate::camera::sin(a) as i16, step as i16) >> 14;
            let dv = crate::hud::mul16(crate::camera::cos(a) as i16, step as i16) >> 14;
            wr(hw, r + rec::U, (u as i16).wrapping_add(du as i16) as u16);
            wr(hw, r + rec::V, (v as i16).wrapping_add(dv as i16) as u16);
        }
        let sector = sector_of(hw, r);
        let old = hw.peek(r + rec::SECTOR);
        if sector != old {
            census(hw, old, side, false);
            census(hw, sector, side, true);
            hw.poke(r + rec::SECTOR, sector);
        }
    }
    // Near the player: listed for the actors.
    let (u, v) = ((rd(hw, r + rec::U) >> 4) as i16, (rd(hw, r + rec::V) >> 4) as i16);
    let near = (u - pu).abs() < NEAR_RANGE && (v - pv).abs() < NEAR_RANGE;
    hw.poke(r + rec::FLAGS, if near { flags | NEAR_FLAG } else { flags & !NEAR_FLAG });
    near_list(hw, id, near);
}

/// One unit's fight: it represents its side in its sector, and fires at the other side's representative
/// there when it can.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
fn fight_unit<H: Hw + ?Sized>(hw: &mut H, first: u16, slot: u16) {
    let r = at(slot);
    let id = first + slot;
    let flags = hw.peek(r + rec::FLAGS);
    if flags & ALIVE == 0 {
        return;
    }
    let side = flags & SIDE;
    let kind = hw.peek(r + rec::KIND) as usize % crate::actors::kind::COUNT as usize;
    let s = sector_at(hw.peek(r + rec::SECTOR));
    reu_put16(hw, s + sec::REP as u32 + 2 * side as u32, id);
    let cool = hw.peek(r + rec::COOL);
    let them = 1 - side;
    let foe = reu_get16(hw, s + sec::REP as u32 + 2 * them as u32);
    let present = reu_get(hw, s + sec::COUNT as u32 + them as u32) > 0;
    let armed = stats::HIT[kind] > 0;
    if hw.peek(r + rec::ACTION) != act::HOLD {
        hw.poke(r + rec::ACTION, if present && armed { act::FIGHT } else { act::MOVE });
    }
    if cool > 0 {
        hw.poke(r + rec::COOL, cool - 1);
    } else if present && armed && foe < COUNT {
        let seed = roll(hw.peek(r + rec::SEED));
        hw.poke(r + rec::SEED, seed);
        if fight(hw, first, id, kind, foe, seed) {
            hw.poke(r + rec::COOL, stats::RELOAD[kind]);
        }
    }
}

/// A unit's sector from its position.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world"))]
fn sector_of<H: Hw + ?Sized>(hw: &mut H, r: u16) -> u8 {
    let (u, v) = (rd(hw, r + rec::U), rd(hw, r + rec::V));
    ((u >> 11) & 15) as u8 | (((v >> 11) & 15) as u8) << 4
}

/// Unit `id` (of `kind`) fires at `foe` if it can hit its kind: a hit with the kind's chance, for its
/// damage; the foe dies at 0. Whether it fired.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
fn fight<H: Hw + ?Sized>(hw: &mut H, first: u16, id: u16, kind: usize, foe: u16, dice: u8) -> bool {
    // The foe's record: in the page, or in the REU.
    let in_page = foe.wrapping_sub(first) < PAGE;
    let base = UNITS + foe as u32 * RECORD as u32;
    let fr = at(foe.wrapping_sub(first));
    let get = |hw: &mut H, o: u16| if in_page { hw.peek(fr + o) } else { reu_get(hw, base + o as u32) };
    let foe_flags = get(hw, rec::FLAGS);
    let foe_kind = get(hw, rec::KIND);
    let air = foe_kind == stats::AIR;
    if foe_flags & ALIVE == 0 || stats::TARGETS[kind] & if air { 2 } else { 1 } == 0 {
        return false;
    }
    let me = at(id.wrapping_sub(first));
    wr(hw, me + rec::TARGET, foe);
    if dice >= stats::HIT[kind] {
        return true;
    }
    let dmg = stats::DAMAGE[kind];
    let hp = get(hw, rec::HP).saturating_sub(dmg);
    let taken = get(hw, rec::TAKEN).saturating_add(dmg);
    let dealt = hw.peek(me + rec::DEALT).saturating_add(dmg);
    hw.poke(me + rec::DEALT, dealt);
    let put = |hw: &mut H, o: u16, v: u8| if in_page { hw.poke(fr + o, v) } else { reu_put(hw, base + o as u32, v) };
    put(hw, rec::HP, hp);
    put(hw, rec::TAKEN, taken);
    if hp == 0 {
        put(hw, rec::FLAGS, foe_flags & !(ALIVE | NEAR_FLAG));
        let sector = get(hw, rec::SECTOR);
        census(hw, sector, foe_flags & SIDE, false);
        living(hw, foe_flags & SIDE, false);
    }
    true
}

/// Keep unit `id` in the near list or out of it.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world"))]
fn near_list<H: Hw + ?Sized>(hw: &mut H, id: u16, near: bool) {
    let n = hw.peek(NEAR).min(NEAR_MAX);
    let mut k = 0;
    while k < n && rd(hw, NEAR + 1 + 2 * k as u16) != id {
        k += 1;
    }
    if near && k == n && n < NEAR_MAX {
        wr(hw, NEAR + 1 + 2 * n as u16, id);
        hw.poke(NEAR, n + 1);
    } else if !near && k < n {
        // Remove: the last moves into its place.
        let last = rd(hw, NEAR + 1 + 2 * (n - 1) as u16);
        wr(hw, NEAR + 1 + 2 * k as u16, last);
        hw.poke(NEAR, n - 1);
    }
}

/// The units near the player as objects to draw, each moved on from its last step by its heading and
/// speed (they step about once a second; drawn every frame). Dead ones stay where they fell.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.actors"))]
pub fn near_objects<H: Hw + ?Sized>(hw: &mut H, now: u16, out: &mut [crate::actors::Object]) -> usize {
    let n = (hw.peek(NEAR).min(NEAR_MAX) as usize).min(out.len());
    let mut r = [0u8; 18];
    for (k, o) in out.iter_mut().enumerate().take(n) {
        let id = hw.peek(NEAR + 1 + 2 * k as u16) as u16 | (hw.peek(NEAR + 2 + 2 * k as u16) as u16) << 8;
        hw::read_into(hw, UNITS + id as u32 * RECORD as u32, crate::layout::BOUNCE, &mut r);
        let kind = r[rec::KIND as usize];
        let alive = r[rec::FLAGS as usize] & ALIVE != 0;
        let yaw = r[rec::YAW as usize];
        let mut u = u16::from_le_bytes([r[4], r[5]]) as i32;
        let mut v = u16::from_le_bytes([r[6], r[7]]) as i32;
        if alive && r[rec::ACTION as usize] != act::HOLD {
            let dt = now.wrapping_sub(u16::from_le_bytes([r[16], r[17]])).min(120) as i32;
            let d = r[rec::SPEED as usize] as i32 * dt;
            let a = (yaw as u16) << 8;
            u += crate::camera::sin(a) * d >> 14;
            v += crate::camera::cos(a) * d >> 14;
        }
        *o = crate::actors::Object { mesh: crate::actors::mesh_of(kind, r[1] & SIDE), yaw, u: u as u16, v: v as u16, h: (r[rec::H as usize] as u16) << 4 };
    }
    n
}
