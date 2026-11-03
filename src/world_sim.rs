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
//! - **decide** (`.text.world3`): it learns from what its last moves brought (damage dealt minus taken)
//!   and walks its kind's decision tree (`roles`, `brain::walk_learn`): advance, hold, attack or retreat;
//! - **fight** (`.text.world2`): it represents its side in its sector and, if it chose to attack, fires
//!   at the other side's representative there by dice; the director, once round the file.
//!
//! So every unit moves, decides and fights about every two seconds: a human reaction time, by design.
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
/// The REU the war writes as it runs (the snapshot gate carries it), and the learned brains'.
pub const STATE: u32 = WORLD;
pub const STATE_LEN: u32 = 0x5000 + HEADER_LEN as u32;
pub const BRAINS: u32 = crate::format::TREES;
pub const BRAINS_LEN: u32 = crate::brain::UNIT_BYTES as u32;

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
    /// The tick of its last step (u16, wrapping); dead, about when it fell (a wreck's age).
    pub const LAST: u16 = 16;
    /// Damage dealt and taken since its last decision (for learning).
    pub const DEALT: u16 = 19;
    pub const TAKEN: u16 = 20;
    pub const SEED: u16 = 21;
    /// Its last two decisions' learn nodes (blob offset u16, 0 none; branch taken u8), newest first.
    pub const TRAIL: u16 = 22;
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
    /// Turn back along its road (the move phase then carries on as `MOVE`).
    pub const RETREAT: u8 = 3;
}

/// A sector: units of each side (u8 × 2), each side's representative (u16 × 2).
pub mod sec {
    pub const COUNT: u16 = 0;
    pub const REP: u16 = 2;
}

/// Routes: [`ROUTE_COUNT`] entries of (first waypoint u8, waypoints in use u8, mode u8, waypoints u8),
/// then waypoints (u, v texels, u16 each; u's top bits a pad, `PAD_SHIFT`). A front road's waypoints in use
/// are the director's (the front); the others use all.
pub const ROUTE_COUNT: u16 = 16;
pub const ROUTE_ENTRY: u16 = 4;
pub const WAYPOINTS: u32 = ROUTES + ROUTE_COUNT as u32 * ROUTE_ENTRY as u32;
/// Route modes: back and forth along it, or round it.
pub const PING_PONG: u8 = 0;
pub const LOOP: u8 = 1;

/// The header: next page (u16), the wave clock (u16 ticks), per side: wave budget (u8 × 2), alive (u16 ×
/// 2), units lost (u16 × 2); then each pad's owner (`world::PADS`, a side).
pub mod head {
    pub const CURSOR: u16 = 0;
    pub const WAVE: u16 = 2;
    pub const BUDGET: u16 = 4;
    pub const ALIVE: u16 = 6;
    pub const LOST: u16 = 10;
    pub const OWNER: u16 = 16;
    /// The director's knobs (`Knobs`, baked; a mission may set them): strength (u8 × 2, in pairs of
    /// units), strength per pad the other side holds (pairs), wave, wave per pad.
    pub const KNOBS: u16 = 21;
    /// Per pad: the director's passes in a row the other side has held it (`HOLD_PASSES` takes it).
    pub const PRESSURE: u16 = 26;
    /// The carrier's unit id (u16; its deck is a landing place, `carrier`).
    pub const CARRIER: u16 = 32;
}
pub const HEADER_LEN: u16 = 34;
/// The pads (`world::PADS`; `war` checks the count).
pub const PADS: u16 = 5;
const _: () = assert!(head::OWNER + PADS <= HEADER_LEN);

/// The front: each side's road to the enemy (`FRONT_ROUTE`) runs through pads (a waypoint's pad, plus
/// one, in its u's top bits: `PAD_SHIFT`). The director ends each road at the first pad the other side
/// holds; a pad whose sector holds enough of the other side changes
/// hands (`CAPTURE` or more, and more than twice the owner's, for `HOLD_PASSES` passes in a row). Reinforcements on a front road arrive at
/// its last pad before the front (the defender's short supply line). So the front walks, both ways.
pub const FRONT_ROUTE: [u8; 2] = [2, 3];
pub const PAD_SHIFT: u16 = 12;
pub const CAPTURE: u8 = 2;
/// The allies' air patrol: its gunships launch from the carrier (`war::route::ALLIED_AIR`).
pub const CARRIER_AIR: u8 = 7;
pub const HOLD_PASSES: u8 = 3;
/// The allies' home pad (`world::PADS`).
pub const HOME: usize = 0;

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
pub const DECIDE: u8 = 1;
pub const FIGHT: u8 = 2;
/// Once round the page file, a frame of the director's (`.text.world4`) before the next lap.
pub const DIRECT: u8 = 3;
const _: () = assert!(PHASE + 1 <= 0xCB00);
/// Within this many texels of the player (each way).
pub const NEAR_RANGE: i16 = 320;
/// The dead stay where they fell this long (ticks; drawn, smoking, near the player), before their slot
/// can be a reinforcement.
pub const WRECK_TICKS: u16 = 40 * 60;
/// Enemies fire on the player within this many texels (|du| + |dv|).
pub const PLAYER_RANGE: u16 = 200;
/// The agent's airspeed field (`Agent::to_bytes`: `vf`, u16 at 18).
const PLAYER_SPEED: u16 = 18;

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
    /// What each kind can hit, by class (`CLASS`): ground 1, air 2, sea 4.
    pub const TARGETS: [u8; COUNT as usize] = [0, 1, 1, 2, 2, 0, 7, 6, 7, 0, 0, 2, 7, 0, 0];
    /// Each kind's class: ground 1, air 2, sea 4.
    pub const CLASS: [u8; COUNT as usize] = [1, 1, 1, 1, 1, 1, 4, 4, 2, 1, 1, 4, 4, 0, 0];
    /// Kinds that fly.
    pub const AIR: u8 = crate::actors::kind::GUNSHIP;
}

/// Where a unit is now (1/16 texel): moved on from its last step along its heading (the dead and the
/// holding stay put). `r`: its record's first 18 bytes. (On the C64 in the deck's overlay, with the
/// AI's and the war's move phase; the player's fight has its own copy, `live_inline`.)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.deck"))]
pub fn live(r: &[u8], now: u16) -> (u16, u16) {
    live_inline(r, now)
}

/// [`live`], inlined where it is called.
#[inline(always)]
pub fn live_inline(r: &[u8], now: u16) -> (u16, u16) {
    let (mut u, mut v) = (u16::from_le_bytes([r[4], r[5]]), u16::from_le_bytes([r[6], r[7]]));
    if r[rec::FLAGS as usize] & ALIVE != 0 && r[rec::ACTION as usize] != act::HOLD {
        // (At most 255 × 120: the products fit the 16 × 16 multiply.)
        let d = (r[rec::SPEED as usize] as u16 * now.wrapping_sub(u16::from_le_bytes([r[16], r[17]])).min(120)) as i16;
        let yaw = (r[rec::YAW as usize] as u16) << 8;
        u = u.wrapping_add((crate::fixed::mul(crate::camera::sin(yaw) as i16, d) >> 14) as u16);
        v = v.wrapping_add((crate::fixed::mul(crate::camera::cos(yaw) as i16, d) >> 14) as u16);
    }
    (u, v)
}

/// The carrier's deck now: its centre (1/16 texel) and heading.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Deck {
    pub u: u16,
    pub v: u16,
    pub yaw: u16,
}

/// The deck's top above the sea (8.8 texels: the hull's 1.2, `scene::vehicle`), and the half-width and
/// half-length one can land on (1/16 texel).
pub const DECK_TOP: i32 = 307;
pub const DECK_HALF: (i16, i16) = (27, 104);

/// The carrier's deck at tick `now`, if it is afloat.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.deck"))]
pub fn carrier<R: crate::brain::Reu + ?Sized>(reu: &mut R, now: u16) -> Option<Deck> {
    let mut id = [0u8; 2];
    reu.read(HEADER + head::CARRIER as u32, &mut id);
    let mut r = [0u8; 18];
    reu.read(UNITS + u16::from_le_bytes(id) as u32 * RECORD as u32, &mut r);
    if r[rec::FLAGS as usize] & ALIVE == 0 || r[rec::KIND as usize] != crate::actors::kind::CARRIER {
        return None;
    }
    let (u, v) = live(&r, now);
    Some(Deck { u, v, yaw: (r[rec::YAW as usize] as u16) << 8 })
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
    let mut b = [0u8; 2];
    hw::read_into(hw, addr, crate::layout::BOUNCE, &mut b);
    u16::from_le_bytes(b)
}

#[inline(always)]
fn reu_put16<H: Hw + ?Sized>(hw: &mut H, addr: u32, w: u16) {
    wr(hw, crate::layout::BOUNCE, w);
    hw::stash(hw, crate::layout::BOUNCE, addr, 2);
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
    // 32 lo / hi in 16 bits (offsets on the map are under 2048 texels).
    let r = if hi == 0 { 0 } else { (lo.min(2047) * 32 / hi) as u8 };
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
    match hw.peek(PHASE) {
        MOVE => move_page(hw, now, pu, pv),
        DECIDE => decide_page(hw),
        FIGHT => fight_page(hw, now, pu, pv),
        _ => direct_page(hw, now),
    }
}

/// The decide phase over the page.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world3"))]
pub fn decide_page<H: Hw + ?Sized>(hw: &mut H) {
    let first = reu_get16(hw, HEADER + head::CURSOR as u32);
    page(hw, first, false);
    // The brains in RAM for the page (walked and taught there), back in one.
    let brains = crate::layout::BRAIN_RAM;
    hw::fetch(hw, BRAINS, brains, BRAINS_LEN as u16);
    for slot in 0..PAGE {
        decide_unit(hw, slot);
    }
    hw::stash(hw, brains, BRAINS, BRAINS_LEN as u16);
    page(hw, first, true);
    hw.poke(PHASE, FIGHT);
}

/// One unit's decision: learn from its last ones, then walk its kind's tree over its situation.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world3"))]
fn decide_unit<H: Hw + ?Sized>(hw: &mut H, slot: u16) {
    use crate::brain::unit;
    let r = at(slot);
    let flags = hw.peek(r + rec::FLAGS);
    if flags & ALIVE == 0 {
        return;
    }
    let kind = hw.peek(r + rec::KIND) % crate::actors::kind::COUNT;
    let k = kind as usize;
    // What its last decisions brought: damage dealt minus taken.
    let reward = (hw.peek(r + rec::DEALT) as i16 - hw.peek(r + rec::TAKEN) as i16).clamp(-15, 15) as i8;
    hw.poke(r + rec::DEALT, 0);
    hw.poke(r + rec::TAKEN, 0);
    let t = r + rec::TRAIL;
    let trail = [(rd(hw, t), hw.peek(t + 2)), (rd(hw, t + 3), hw.peek(t + 5))];
    if reward != 0 {
        crate::brain::learn(hw, crate::layout::BRAIN_RAM, kind, &trail, reward);
    }
    // Its situation.
    let side = flags & SIDE;
    let s = sector_at(hw.peek(r + rec::SECTOR));
    let mut counts = [0u8; 2];
    hw::read_into(hw, s + sec::COUNT as u32, crate::layout::BOUNCE, &mut counts);
    let (foes, friends) = (counts[1 - side as usize], counts[side as usize]);
    let mut seed = hw.peek(r + rec::SEED);
    let mut f = [0u8; unit::FEATURES];
    f[unit::WOUNDED] = ((hw.peek(r + rec::HP) as u16) * 2 < stats::HP[k] as u16) as u8;
    f[unit::FOES] = (foes > 0) as u8;
    f[unit::OUTNUMBERED] = (foes > friends) as u8;
    f[unit::READY] = (hw.peek(r + rec::COOL) == 0) as u8;
    f[unit::ARMED] = (stats::HIT[k] > 0) as u8;
    f[unit::MOBILE] = (stats::SPEED[k] > 0 && hw.peek(r + rec::ROUTE) != 0xFF) as u8;
    for x in f[unit::LEARN0 as usize..].iter_mut() {
        seed = roll(seed);
        *x = seed;
    }
    hw.poke(r + rec::SEED, seed);
    let (mv, node, took) = crate::brain::walk_learn(hw, crate::layout::BRAIN_RAM, kind, &f);
    // The trail: this decision's learn node first, the last one's after it.
    for i in 0..3 {
        let b = hw.peek(t + i);
        hw.poke(t + 3 + i, b);
    }
    wr(hw, t, node);
    hw.poke(t + 2, took);
    let static_ = f[unit::MOBILE] == 0;
    hw.poke(r + rec::ACTION, match mv {
        unit::ATTACK => act::FIGHT,
        unit::RETREAT => act::RETREAT,
        unit::HOLD => act::HOLD,
        _ if static_ => act::HOLD,
        _ => act::MOVE,
    });
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
    hw.poke(PHASE, DECIDE);
}

/// The fight phase over the same page, then on to the next.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
pub fn fight_page<H: Hw + ?Sized>(hw: &mut H, now: u16, pu: i16, pv: i16) {
    let first = reu_get16(hw, HEADER + head::CURSOR as u32);
    page(hw, first, false);
    hw.poke(SHOTS, 0);
    for slot in 0..PAGE {
        fight_unit(hw, first, slot, now, pu, pv);
    }
    page(hw, first, true);
    apply_shots(hw, now);
    let next = if first + PAGE >= COUNT { 0 } else { first + PAGE };
    reu_put16(hw, HEADER + head::CURSOR as u32, next);
    hw.poke(PHASE, if next == 0 { DIRECT } else { MOVE });
}

/// The director's phase: then the next lap's first move.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world4"))]
pub fn direct_page<H: Hw + ?Sized>(hw: &mut H, now: u16) {
    director(hw, now);
    hw.poke(PHASE, MOVE);
}

/// The director, once round the whole page file (its own phase, `DIRECT`): every `WAVE_TICKS`, the front moves (`front`), then
/// each side behind its strength gets a wave (budget) of reinforcements, spent as dead slots of its side
/// come round. Strength and waves grow with the pads the other side holds: a side pushed back fights
/// nearer its depots (so the front swings both ways rather than one side being wiped out).
pub const WAVE_TICKS: u16 = 60 * 60;

/// The director's knobs, as in the header (`head::KNOBS`).
#[derive(Clone, Copy, Debug)]
pub struct Knobs {
    /// Each side's strength (units) with no pads lost.
    pub strength: [u16; 2],
    /// More strength for each pad the other side holds.
    pub per_pad: u16,
    /// A wave's units, and more for each pad the other side holds.
    pub wave: u8,
    pub wave_per_pad: u8,
}

impl Knobs {
    /// Tuned with `chopper war --knobs` so the front swings both ways over an hour, every few minutes to a
    /// quarter of an hour, and home holds (`the_front_swings_both_ways`).
    pub const DEFAULT: Self = Self { strength: [100, 110], per_pad: 24, wave: 12, wave_per_pad: 4 };

    pub fn to_bytes(self) -> [u8; 5] {
        [(self.strength[0] / 2) as u8, (self.strength[1] / 2) as u8, (self.per_pad / 2) as u8, self.wave, self.wave_per_pad]
    }
}

#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world4"))]
fn director<H: Hw + ?Sized>(hw: &mut H, now: u16) {
    let last = reu_get16(hw, HEADER + head::WAVE as u32);
    if now.wrapping_sub(last) < WAVE_TICKS {
        return;
    }
    reu_put16(hw, HEADER + head::WAVE as u32, now);
    front(hw);
    // The owners and the knobs, one read.
    let mut h = [0u8; (HEADER_LEN - head::OWNER) as usize];
    hw::read_into(hw, HEADER + head::OWNER as u32, crate::layout::BOUNCE, &mut h);
    let k = (head::KNOBS - head::OWNER) as usize;
    for side in 0..2u32 {
        let theirs = h[..PADS as usize].iter().filter(|&&o| o as u32 != side).count() as u8;
        let strength = (h[k + side as usize] as u16 + h[k + 2] as u16 * theirs as u16) * 2;
        let alive = reu_get16(hw, HEADER + head::ALIVE as u32 + 2 * side);
        if alive < strength {
            reu_put(hw, HEADER + head::BUDGET as u32 + side, h[k + 3] + h[k + 4] * theirs);
        }
    }
}

/// The front: pads change hands, then each side's road reaches to the first pad the other side holds.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world4"))]
fn front<H: Hw + ?Sized>(hw: &mut H) {
    // The owners (and knobs, between), and the pressure on each pad.
    let mut h = [0u8; (head::PRESSURE + PADS - head::OWNER) as usize];
    hw::read_into(hw, HEADER + head::OWNER as u32, crate::layout::BOUNCE, &mut h);
    let (owner, pressure) = h.split_at_mut((head::PRESSURE - head::OWNER) as usize);
    for side in 0..2 {
        let entry = ROUTES + FRONT_ROUTE[side] as u32 * ROUTE_ENTRY as u32;
        let mut e = [0u8; ROUTE_ENTRY as usize];
        hw::read_into(hw, entry, crate::layout::BOUNCE, &mut e);
        let (first, all) = (e[0], e[3]);
        let mut reach = all;
        for k in 0..all {
            let mut wp = [0u8; 4];
            hw::read_into(hw, WAYPOINTS + (first + k) as u32 * 4, crate::layout::BOUNCE, &mut wp);
            let (u, v) = (u16::from_le_bytes([wp[0], wp[1]]), u16::from_le_bytes([wp[2], wp[3]]));
            let pad = (u >> PAD_SHIFT) as usize;
            if pad == 0 || pad > PADS as usize {
                continue;
            }
            let p = pad - 1;
            // Taken? (Each side's road visits the pads it fights over; once a side, once is enough.) Home
            // is the allies' last stand: never taken.
            if side == 0 && p != HOME {
                let sector = (((u & 0x7FF) >> 7) | ((v >> 7) << 4)) as u8;
                let mut counts = [0u8; 2];
                hw::read_into(hw, sector_at(sector) + sec::COUNT as u32, crate::layout::BOUNCE, &mut counts);
                let o = owner[p] as usize & 1;
                if counts[1 - o] >= CAPTURE && counts[1 - o] > 2 * counts[o] {
                    pressure[p] += 1;
                    if pressure[p] >= HOLD_PASSES {
                        owner[p] = 1 - o as u8;
                        pressure[p] = 0;
                    }
                } else {
                    pressure[p] = 0;
                }
            }
            if owner[p] != side as u8 {
                reach = k + 1;
                break;
            }
        }
        hw.poke(crate::layout::BOUNCE, reach);
        hw::stash(hw, crate::layout::BOUNCE, entry + 1, 1);
    }
    for (k, &b) in h.iter().enumerate() {
        hw.poke(crate::layout::BOUNCE + k as u16, b);
    }
    hw::stash(hw, crate::layout::BOUNCE, HEADER + head::OWNER as u32, h.len() as u16);
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
    ((u16::from_le_bytes([w[0], w[1]]) & ((1 << PAD_SHIFT) - 1)) as i16, u16::from_le_bytes([w[2], w[3]]) as i16)
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
    let last = rd(hw, r + rec::LAST);
    let route_no = hw.peek(r + rec::ROUTE);
    if flags & ALIVE == 0 {
        // A wreck for `WRECK_TICKS` (listed near the player, then gone); then a free slot, respawned if its
        // side has reinforcements due.
        let fresh = now.wrapping_sub(last) < WRECK_TICKS;
        let (u, v) = ((rd(hw, r + rec::U) >> 4) as i16, (rd(hw, r + rec::V) >> 4) as i16);
        near_list(hw, id, fresh && (u - pu).abs() < NEAR_RANGE && (v - pv).abs() < NEAR_RANGE, false);
        if fresh {
            return;
        }
        let budget = HEADER + head::BUDGET as u32 + side as u32;
        let b = reu_get(hw, budget);
        if b == 0 || route_no == 0xFF {
            return;
        }
        wr(hw, r + rec::LAST, now);
        reu_put(hw, budget, b - 1);
        // At the route's start; on its side's front road, the last waypoint before the front.
        let (start, len, _) = route(hw, route_no);
        let k = if route_no == FRONT_ROUTE[side as usize] { len.saturating_sub(2) } else { 0 };
        let (u, v) = waypoint(hw, start + k);
        let (mut u, mut v) = ((u as u16) << 4, (v as u16) << 4);
        if route_no == CARRIER_AIR {
            // Off the carrier's deck, wherever it is (when afloat).
            let id = reu_get16(hw, HEADER + head::CARRIER as u32);
            let mut c = [0u8; 18];
            hw::read_into(hw, UNITS + id as u32 * RECORD as u32, crate::layout::BOUNCE, &mut c);
            if c[rec::FLAGS as usize] & ALIVE != 0 {
                (u, v) = live(&c, now);
            }
        }
        wr(hw, r + rec::U, u);
        wr(hw, r + rec::V, v);
        hw.poke(r + rec::LEG, k);
        hw.poke(r + rec::HP, stats::HP[kind]);
        hw.poke(r + rec::FLAGS, (flags | ALIVE) & !NEAR_FLAG);
        hw.poke(r + rec::ACTION, act::MOVE);
        let sector = sector_of(hw, r);
        hw.poke(r + rec::SECTOR, sector);
        census(hw, sector, side, true);
        living(hw, side, true);
        return;
    }
    let dt = now.wrapping_sub(last).min(255);
    wr(hw, r + rec::LAST, now);
    // The carrier's damage control: a hit point back about every fourth lap.
    if kind == crate::actors::kind::CARRIER as usize && now & 0x180 == 0 {
        let hp = hw.peek(r + rec::HP);
        if hp < stats::HP[kind] {
            hw.poke(r + rec::HP, hp + 1);
        }
    }
    let speed = stats::SPEED[kind];
    if speed > 0 && route_no != 0xFF && hw.peek(r + rec::ACTION) != act::HOLD {
        let (start, len, mode) = route(hw, route_no);
        let mut leg = hw.peek(r + rec::LEG);
        if hw.peek(r + rec::ACTION) == act::RETREAT {
            // Turn back along a road: the same waypoint, from the other way.
            if mode != LOOP && len > 1 {
                leg = (2 * len - 1).wrapping_sub(leg) % (2 * len - 2);
            }
            hw.poke(r + rec::ACTION, act::MOVE);
        }
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
    near_list(hw, id, near, kind == crate::actors::kind::CARRIER as usize);
}

/// One unit's fight: it represents its side in its sector, and fires at the other side's representative
/// there when it can.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
fn fight_unit<H: Hw + ?Sized>(hw: &mut H, first: u16, slot: u16, now: u16, pu: i16, pv: i16) {
    let r = at(slot);
    let id = first + slot;
    let flags = hw.peek(r + rec::FLAGS);
    if flags & ALIVE == 0 {
        return;
    }
    let side = flags & SIDE;
    let kind = hw.peek(r + rec::KIND) as usize % crate::actors::kind::COUNT as usize;
    let s = sector_at(hw.peek(r + rec::SECTOR));
    let them = 1 - side;
    let mut sr = [0u8; SECTOR as usize];
    hw::read_into(hw, s, crate::layout::BOUNCE, &mut sr);
    let rep = (sec::REP + 2 * them as u16) as usize;
    let foe = u16::from_le_bytes([sr[rep], sr[rep + 1]]);
    let present = sr[(sec::COUNT + them as u16) as usize] > 0;
    wr(hw, crate::layout::BOUNCE, id);
    hw::stash(hw, crate::layout::BOUNCE, s + sec::REP as u32 + 2 * side as u32, 2);
    let cool = hw.peek(r + rec::COOL);
    let fighting = hw.peek(r + rec::ACTION) == act::FIGHT;
    // The player, if near and in range of an enemy that can hit aircraft and chose to fight: fired on with
    // human aim (the odds halved, and halved again when the player is fast).
    let (u, v) = ((rd(hw, r + rec::U) >> 4) as i16, (rd(hw, r + rec::V) >> 4) as i16);
    let range = (u - pu).unsigned_abs() + (v - pv).unsigned_abs();
    if cool == 0 && fighting && side == crate::actors::ENEMY && flags & NEAR_FLAG != 0 && stats::TARGETS[kind] & 2 != 0 && range < PLAYER_RANGE {
        let seed = roll(hw.peek(r + rec::SEED));
        hw.poke(r + rec::SEED, seed);
        let fast = rd(hw, crate::layout::AGENT + PLAYER_SPEED) > 6000;
        let odds = stats::HIT[kind] >> (1 + fast as u8);
        hw.poke(r + rec::COOL, stats::RELOAD[kind]);
        // Its muzzle flash: who is shooting.
        muzzle(hw, r, now);
        if seed < odds {
            use crate::combat::{PLAYER, player};
            let d = hw.peek(PLAYER + player::DAMAGE).saturating_add(stats::DAMAGE[kind]);
            hw.poke(PLAYER + player::DAMAGE, d);
            hw.poke(PLAYER + player::FLASH, 12);
            let dealt = hw.peek(r + rec::DEALT).saturating_add(stats::DAMAGE[kind]);
            hw.poke(r + rec::DEALT, dealt);
        }
        return;
    }
    if cool > 0 {
        hw.poke(r + rec::COOL, cool - 1);
    } else if present && fighting && foe < COUNT {
        let seed = roll(hw.peek(r + rec::SEED));
        hw.poke(r + rec::SEED, seed);
        if fight(hw, first, id, kind, foe, seed, now) {
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

/// The fight phase's hits, applied once its page is back in the REU: a count, then (unit u16, damage u8)
/// each (the band stage, free during the set-up).
pub const SHOTS: u16 = crate::layout::STAGE;
const _: () = assert!(1 + 3 * PAGE <= crate::layout::ROW_STRIDE);

/// Unit `id` (of `kind`) fires at `foe` if it can hit its kind: a hit with the kind's chance, for its
/// damage (listed, to be applied after the page is written back). Whether it fired.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
fn fight<H: Hw + ?Sized>(hw: &mut H, first: u16, id: u16, kind: usize, foe: u16, dice: u8, now: u16) -> bool {
    let base = UNITS + foe as u32 * RECORD as u32;
    // The foe's kind, flags and where it is, in one.
    let mut f = [0u8; rec::H as usize + 1];
    hw::read_into(hw, base, crate::layout::BOUNCE, &mut f);
    let (class, foe_flags) = (stats::CLASS[f[rec::KIND as usize] as usize % crate::actors::kind::COUNT as usize], f[rec::FLAGS as usize]);
    if foe_flags & ALIVE == 0 || stats::TARGETS[kind] & class == 0 {
        return false;
    }
    let me = at(id.wrapping_sub(first));
    wr(hw, me + rec::TARGET, foe);
    // Near the player, the fire is seen: the shot, and the hit.
    let seen = (hw.peek(me + rec::FLAGS) | foe_flags) & NEAR_FLAG != 0;
    if seen {
        muzzle(hw, me, now);
    }
    if dice >= stats::HIT[kind] {
        return true;
    }
    if seen {
        let p = |k: u16| u16::from_le_bytes([f[k as usize], f[k as usize + 1]]);
        crate::combat::spawn(hw, crate::effects::effect::IMPACT, p(rec::U), p(rec::V), (f[rec::H as usize] as u16 + 1) << 4, now);
    }
    let dmg = stats::DAMAGE[kind];
    let dealt = hw.peek(me + rec::DEALT).saturating_add(dmg);
    hw.poke(me + rec::DEALT, dealt);
    let n = hw.peek(SHOTS);
    let at = SHOTS + 1 + 3 * n as u16;
    wr(hw, at, foe);
    hw.poke(at + 2, dmg);
    hw.poke(SHOTS, n + 1);
    true
}

/// The listed hits into the REU: hit points down, damage taken up (for learning), dead at 0 with the
/// bookkeeping.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
fn apply_shots<H: Hw + ?Sized>(hw: &mut H, now: u16) {
    let n = hw.peek(SHOTS).min(PAGE as u8);
    for k in 0..n as u16 {
        let foe = rd(hw, SHOTS + 1 + 3 * k);
        let dmg = hw.peek(SHOTS + 3 + 3 * k);
        // The foe's record in one, back in one (through the bounce buffer).
        let base = UNITS + foe as u32 * RECORD as u32;
        let b = crate::layout::BOUNCE;
        hw::fetch(hw, base, b, RECORD);
        let flags = hw.peek(b + rec::FLAGS);
        if flags & ALIVE == 0 {
            continue;
        }
        let hp = hw.peek(b + rec::HP).saturating_sub(dmg);
        hw.poke(b + rec::HP, hp);
        let taken = hw.peek(b + rec::TAKEN).saturating_add(dmg);
        hw.poke(b + rec::TAKEN, taken);
        if hp == 0 {
            hw.poke(b + rec::FLAGS, flags & !(ALIVE | NEAR_FLAG));
            if flags & NEAR_FLAG != 0 {
                // Killed in sight: it blows up, then burns.
                let (u, v, h) = (rd(hw, b + rec::U), rd(hw, b + rec::V), (hw.peek(b + rec::H) as u16) << 4);
                crate::combat::spawn(hw, crate::effects::effect::EXPLOSION, u, v, h, now);
                crate::combat::spawn(hw, crate::effects::effect::WRECK_FIRE, u, v, h, now.wrapping_add(40));
            }
        }
        let sector = hw.peek(b + rec::SECTOR);
        hw::stash(hw, b, base, RECORD);
        if hp == 0 {
            census(hw, sector, flags & SIDE, false);
            living(hw, flags & SIDE, false);
        }
    }
}

/// A muzzle flash at the unit whose page record is at `r` (a little above it).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world2"))]
fn muzzle<H: Hw + ?Sized>(hw: &mut H, r: u16, now: u16) {
    let (u, v, h) = (rd(hw, r + rec::U), rd(hw, r + rec::V), (hw.peek(r + rec::H) as u16 + 1) << 4);
    crate::combat::spawn(hw, crate::effects::effect::MUZZLE, u, v, h, now);
}

/// Keep unit `id` in the near list or out of it.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.world"))]
fn near_list<H: Hw + ?Sized>(hw: &mut H, id: u16, near: bool, first: bool) {
    let n = hw.peek(NEAR).min(NEAR_MAX);
    let mut k = 0;
    while k < n && rd(hw, NEAR + 1 + 2 * k as u16) != id {
        k += 1;
    }
    if near && k == n && n < NEAR_MAX {
        wr(hw, NEAR + 1 + 2 * n as u16, id);
        hw.poke(NEAR, n + 1);
    } else if near && k == n && first {
        // The list full: one that must be there (the carrier) takes the last place.
        wr(hw, NEAR + 1 + 2 * (n - 1) as u16, id);
    } else if !near && k < n {
        // Remove: the last moves into its place.
        let last = rd(hw, NEAR + 1 + 2 * (n - 1) as u16);
        wr(hw, NEAR + 1 + 2 * k as u16, last);
        hw.poke(NEAR, n - 1);
    }
}
