//! The player's fight: the gun (fire), missiles (fire 2, at the locked target), the war's units near
//! enough to be targets, the projectiles in flight, and the damage taken.
//!
//! - Targets: the living enemy units near the player (`world_sim::NEAR`), moved on from their last step,
//!   written for the HUD (`hud::S_TARGETS`, in the near list's order; the seeker locks onto one of them).
//! - The gun: while fire is held, a burst every [`GUN_EVERY`] ticks: a tracer flies from the nose, and an
//!   enemy unit inside the boresight's cone ([`CONE_X`] × [`CONE_Y`] pixels) within [`GUN_RANGE`] is hit
//!   with [`GUN_HIT`] odds for [`GUN_DAMAGE`].
//! - Missiles: fire 2 with a lock launches one at the locked unit; it homes and hits for
//!   [`MISSILE_DAMAGE`].
//! - The player's damage is taken in the war's fight phase (`world_sim`); at [`DOWN`] the helicopter is
//!   down: the autopilot takes over again at home, rearmed.
//!
//! Hits go straight into the units' records in the REU (after the frame's page is written back), with
//! the kill's bookkeeping (sector counts, the living, the lost) and the learning's (damage taken).

use crate::agent::Agent;
use crate::hw::{self, Hw};
use crate::squad::{MEMBERS, SLOTS, entry};
use crate::world_sim::{self as ws, ALIVE, NEAR_FLAG, RECORD, SIDE, rec};

/// Projectiles in flight: [`PROJ_MAX`] entries of [`PROJ_ENTRY`] bytes: kind (`actors::kind`, 0 free),
/// ticks left, position (1/16 texel, u16 × 3), heading (256 a turn), climb (1/16 texel a tick, i8), target
/// unit (u16, missiles), spare.
pub const PROJ: u16 = 0x8E84;
pub const PROJ_MAX: u16 = 6;
pub const PROJ_ENTRY: u16 = 12;
pub mod proj {
    pub const KIND: u16 = 0;
    pub const TTL: u16 = 1;
    pub const U: u16 = 2;
    pub const V: u16 = 4;
    pub const H: u16 = 6;
    pub const YAW: u16 = 8;
    pub const CLIMB: u16 = 9;
    pub const TARGET: u16 = 10;
}
/// The player's state: damage taken (0..=`DOWN`), missiles left, the gun's cooldown (ticks), bursts fired,
/// kills (u16), the last fire 2 (for its press), the hit flash (ticks), the tick of the last frame (u16);
/// the autopilot's prey (unit, u16; `NOBODY` none); damage dealt and taken (counters, wrapping: the
/// learning's rewards), and its learning trail: the last two learn nodes its decisions passed (newest
/// first), each the node (blob offset | 1 << 15 for its yes branch, u16) and the counters then.
pub const PLAYER: u16 = PROJ + PROJ_MAX * PROJ_ENTRY;
pub mod player {
    pub const DAMAGE: u16 = 0;
    pub const MISSILES: u16 = 1;
    pub const COOL: u16 = 2;
    pub const BURSTS: u16 = 3;
    pub const KILLS: u16 = 4;
    pub const FIRE2: u16 = 6;
    pub const FLASH: u16 = 7;
    pub const LAST: u16 = 8;
    pub const LAST_FX: u16 = 10;
    pub const PREY: u16 = 12;
    pub const DEALT: u16 = 14;
    pub const TAKEN: u16 = 15;
    pub const TRAIL: u16 = 16;
    pub const TRAIL_ENTRY: u16 = 4;
}
pub const PLAYER_LEN: u16 = 24;
const _: () = assert!(PLAYER + PLAYER_LEN <= crate::layout::XFORM_RAM);
/// Everything of this the frames carry (the snapshot gate).
pub const STATE: u16 = PROJ;
pub const STATE_LEN: u16 = PROJ_MAX * PROJ_ENTRY + PLAYER_LEN;

/// The effects playing (`effects`): [`FX_MAX`] entries of [`FX_ENTRY`] bytes: effect + 1 (0 free), [`FAR`] or 0,
/// the tick it began (u16), where (1/16 texel, u16 × 3). After the relocated `bandtext`'s last page.
pub const FX: u16 = 0xB760;
pub const FX_MAX: u8 = 16;
pub const FX_ENTRY: u16 = 10;
const _: () = assert!(FX + FX_MAX as u16 * FX_ENTRY <= 0xB800);
/// The wrecks near the player smoking (`world_sim::WRECK_TICKS`), listed by `update` for `effects`: a
/// count, then (u, v, h 1/16 texel, age in ticks) u16 each. (After the mesh buffer, before the HUD's
/// page.)
pub const WRECKS: u16 = 0x8AB0;
pub const WRECK_MAX: u8 = 6;
pub const WRECK_ENTRY: u16 = 8;
const _: () = assert!(WRECKS + 1 + WRECK_MAX as u16 * WRECK_ENTRY <= crate::hud::PAGE);
/// The frame's effects in view, for `geom::push_dynamic`: a count, then [`FXDYN_ENTRY`] bytes each: left
/// x (multicolour pixels) and y (line) of the anchor (i16 × 2), the frame record's REU address (u24),
/// the depth key. (After the transform tables, before the reciprocals.)
pub const FXDYN: u16 = 0x8F84;
pub const FXDYN_ENTRY: u16 = 8;
/// (Fewer than the effects and the wrecks: the far battle's flashes and the clouds fill the free slots,
/// and the room before the reciprocals is this.)
pub const FXDYN_MAX: u8 = 15;
const _: () = assert!(FXDYN >= crate::layout::XFORM_RAM + 0x84 && FXDYN + 1 + FXDYN_MAX as u16 * FXDYN_ENTRY <= crate::fixed::RECIP);
/// Effect frames last `1 << FX_TICKS_SHIFT` ticks; looping effects (smoke, a burning wreck) play this
/// long.
pub const FX_TICKS_SHIFT: u32 = 2;
pub const FX_LIFE: u16 = 20 * 60;
/// A wreck's fire starts this many ticks after its explosion: once the fireball has turned to smoke (the
/// explosion's frame 36 of 64).
pub const FIRE_AFTER: u16 = 36 << FX_TICKS_SHIFT;

/// Start `effect` at (`u`, `v`, `h`) (1/16 texel): a free slot, else the oldest far one, else the oldest.
/// `effect | FAR` (the war's battle on the horizon): a free slot only, and marked (the second byte), so
/// the fight at hand takes its slot first and draws first (`effects`). (Resident on the C64: the
/// player's fight and the war's both start effects.) The age is signed: one not begun yet (a wreck's fire
/// after its explosion) is the newest, never the oldest.
#[inline(never)]
pub fn spawn<H: Hw + ?Sized>(hw: &mut H, effect: u8, u: u16, v: u16, h: u16, now: u16) {
    let far = effect & FAR;
    let (mut best, mut age) = (0, i16::MIN);
    for k in 0..FX_MAX as u16 {
        let e = FX + k * FX_ENTRY;
        if hw.peek(e) == 0 {
            best = e;
            break;
        }
        let a = if hw.peek(e + 1) != 0 { i16::MAX } else { now.wrapping_sub(rd(hw, e + 2)) as i16 };
        if far == 0 && a >= age {
            (best, age) = (e, a);
        }
    }
    if best == 0 {
        return;
    }
    hw.poke(best, (effect & !FAR) + 1);
    hw.poke(best + 1, far);
    wr(hw, best + 2, now);
    wr(hw, best + 4, u);
    wr(hw, best + 6, v);
    wr(hw, best + 8, h);
}

/// [`spawn`]'s mark for an effect far off.
pub const FAR: u8 = 0x80;
const _: () = assert!((crate::effects::EFFECTS as u8) < FAR);

/// Is (`u`, `v`) (1/16 texel) over the sea (the hazard map's bit 7)?
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn sea<H: Hw + ?Sized>(hw: &mut H, u: u16, v: u16) -> bool {
    let cell = (v >> (4 + crate::format::HAZARD_SHIFT)) as u32 * crate::format::HAZARD_CELLS as u32 + (u >> (4 + crate::format::HAZARD_SHIFT)) as u32;
    reu_get(hw, crate::format::HAZARD + cell) & 0x80 != 0
}

/// The effects on: each one's frame, its scale for its distance, placed for the drawing; done ones freed.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.fx"))]
fn effects<H: Hw + ?Sized>(hw: &mut H, eye: &crate::hud::Eye, now: u16, drift: u16) {
    use crate::effects::{FRAMES, HEIGHT_Q4, LOOPS, record, scale_for};
    let mut n = 0u8;
    let wrecks = hw.peek(WRECKS).min(WRECK_MAX) as u16;
    // The fight at hand's effects, then the wrecks' smoke columns, then the battle on the horizon's (so
    // those are what the cap cuts).
    let later = FX_MAX as u16 + wrecks;
    for k in 0..later + FX_MAX as u16 {
        let (effect, f, e) = if k < FX_MAX as u16 || k >= later {
            let far = k >= later;
            let e = FX + (if far { k - later } else { k }) * FX_ENTRY;
            let id = hw.peek(e);
            if id == 0 || (hw.peek(e + 1) != 0) != far {
                continue;
            }
            let effect = (id - 1) % crate::effects::EFFECTS as u8;
            let age = now.wrapping_sub(rd(hw, e + 2));
            if age >= 0x8000 {
                // Not begun yet (a wreck's fire after its explosion).
                continue;
            }
            let frames = FRAMES[effect as usize] as u16;
            let mut f = age >> FX_TICKS_SHIFT;
            if f >= frames {
                if !LOOPS[effect as usize] || age >= FX_LIFE {
                    hw.poke(e, 0);
                    continue;
                }
                // (The looping effects' frames are a power of two.)
                f &= frames - 1;
            }
            (effect, f, e + 4)
        } else {
            let e = WRECKS + 1 + (k - FX_MAX as u16) * WRECK_ENTRY;
            (crate::effects::effect::SMOKE, (rd(hw, e + 6) >> FX_TICKS_SHIFT) & (FRAMES[crate::effects::effect::SMOKE as usize] as u16 - 1), e)
        };
        // (Full: the rest still freed when done, not drawn.)
        if n >= FXDYN_MAX {
            continue;
        }
        let p = [(rd(hw, e) >> 2) as i16, (rd(hw, e + 4) >> 2) as i16, (rd(hw, e + 2) >> 2) as i16];
        let c = crate::hud::to_camera(eye, p);
        if c[2] < crate::hud::NEAR {
            continue;
        }
        let (x, y) = crate::hud::screen(c);
        if !(-64..384).contains(&x) || !(-64..200).contains(&y) {
            continue;
        }
        let z4 = c[2] as i32 * 4;
        let rec = record(effect, scale_for(z4, HEIGHT_Q4[effect as usize]), f as u8);
        let at = FXDYN + 1 + n as u16 * FXDYN_ENTRY;
        wr(hw, at, (x >> 1) as u16);
        wr(hw, at + 2, y as u16);
        wr(hw, at + 4, rec as u16);
        hw.poke(at + 6, (rec >> 16) as u8);
        // (Nearer than the fog's buckets: `geom::bin` takes those nearest first, one flat haze shade, so a
        // flash binned among them could be drawn under a mountain behind it; and the last bucket is the
        // clouds', which `geom::push_dynamic` turns to the horizon's tilt.)
        hw.poke(at + 7, crate::geom::depth_key(3 * z4).min(crate::geom::FAR_KEY - 1));
        n += 1;
    }
    // The clouds on the horizon, in the slots left.
    let n = crate::clouds::place(hw, eye, drift, n);
    hw.poke(FXDYN, n);
}

pub const GUN_EVERY: u8 = 6;
pub const GUN_RANGE: i16 = 80 * 4;
pub const GUN_HIT: u8 = 180;
pub const GUN_DAMAGE: u8 = 2;
/// The gun's range as a vehicle's listed distance (`squad::entry::D`, texels / 2: `GUN_RANGE` is in
/// quarter texels), with `world_sim::dist`'s overestimate of up to an eighth.
const CONE_D: u8 = (GUN_RANGE / 8 + GUN_RANGE / 64 + 1) as u8;
pub const CONE_X: i16 = 12;
pub const CONE_Y: i16 = 10;
pub const MISSILES: u8 = 8;
pub const MISSILE_DAMAGE: u8 = 40;
pub const DOWN: u8 = 100;
/// Landed within this many texels of home (each way), the helicopter is rearmed and repaired.
pub const PAD: i16 = 16;
/// The autopilot goes home to land, rearm and repair at this much damage (or with no missiles left).
pub const HURT: u8 = 50;
/// It hunts enemies within this range (texels, the larger axis plus half the smaller): the missile's.
pub const PREY_RANGE: i16 = 250;
/// A kill's worth to the learning, over the damage it dealt.
pub const KILL_BONUS: u8 = 20;
/// Projectile speeds (1/16 texel a tick) and lives (ticks).
pub const TRACER_SPEED: u16 = 32;
pub const TRACER_TTL: u8 = 60;
pub const MISSILE_SPEED: u16 = 14;
pub const MISSILE_TTL: u8 = 255;

#[inline(always)]
fn rd<H: Hw + ?Sized>(hw: &mut H, a: u16) -> u16 {
    hw.peek(a) as u16 | (hw.peek(a + 1) as u16) << 8
}

#[inline(always)]
fn wr<H: Hw + ?Sized>(hw: &mut H, a: u16, w: u16) {
    hw.poke(a, w as u8);
    hw.poke(a + 1, (w >> 8) as u8);
}

#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn reu_get<H: Hw + ?Sized>(hw: &mut H, addr: u32) -> u8 {
    let mut b = [0u8; 1];
    hw::read_into(hw, addr, crate::layout::BOUNCE, &mut b);
    b[0]
}

#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn reu_put<H: Hw + ?Sized>(hw: &mut H, addr: u32, v: u8) {
    hw.poke(crate::layout::BOUNCE, v);
    hw::stash(hw, crate::layout::BOUNCE, addr, 1);
}

/// Damage vehicle `vid` (`world_sim::vehicle`; its platoon's record in the REU): its platoon's hit points
/// down, the hit member falling first (`world_sim::spill`), the damage taken up (for its learning); the
/// platoon dead with its last member, with the bookkeeping. Whether a member fell.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn damage_unit<H: Hw + ?Sized>(hw: &mut H, vid: u16, dmg: u8) -> bool {
    let base = ws::UNITS + ws::platoon(vid) as u32 * RECORD as u32;
    // The record in one, back in one (through the bounce buffer).
    let b = crate::layout::BOUNCE;
    hw::fetch(hw, base, b, RECORD);
    let flags = hw.peek(b + rec::FLAGS);
    if flags & ALIVE == 0 {
        return false;
    }
    let kind = hw.peek(b + rec::KIND) as usize % crate::actors::kind::COUNT as usize;
    // (The dice rolled on, as the war's shots roll them.)
    let seed = ws::roll(hw.peek(b + rec::SEED));
    hw.poke(b + rec::SEED, seed);
    let (hp, lost, fell) = ws::spill(kind, hw.peek(b + rec::HP), rd(hw, b + rec::LOST), dmg, ws::member(vid), seed);
    hw.poke(b + rec::HP, hp);
    wr(hw, b + rec::LOST, lost);
    let taken = hw.peek(b + rec::TAKEN).saturating_add(dmg);
    hw.poke(b + rec::TAKEN, taken);
    if hp > 0 {
        hw::stash(hw, b, base, RECORD);
        return fell != ws::NONE_FELL;
    }
    hw.poke(b + rec::FLAGS, flags & !(ALIVE | NEAR_FLAG));
    let side = (flags & SIDE) as u32;
    let sector = hw.peek(b + rec::SECTOR);
    hw::stash(hw, b, base, RECORD);
    let c = ws::SECTORS + sector as u32 * ws::SECTOR as u32 + ws::sec::COUNT as u32 + side;
    let n = reu_get(hw, c);
    reu_put(hw, c, n.saturating_sub(1));
    for (field, up) in [(ws::head::ALIVE, false), (ws::head::LOST, true)] {
        let a = ws::HEADER + field as u32 + 2 * side;
        let n = reu_get(hw, a) as u16 | (reu_get(hw, a + 1) as u16) << 8;
        let n = if up { n + 1 } else { n.saturating_sub(1) };
        reu_put(hw, a, n as u8);
        reu_put(hw, a + 1, (n >> 8) as u8);
    }
    true
}

/// A free projectile slot (or none).
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn launch<H: Hw + ?Sized>(hw: &mut H, kind: u8, ttl: u8, a: &Agent, target: u16) {
    // A free slot; a missile takes a tracer's if there is none.
    for k in 0..2 * PROJ_MAX {
        let p = PROJ + (k % PROJ_MAX) * PROJ_ENTRY;
        let was = hw.peek(p + proj::KIND);
        if was == 0 || k >= PROJ_MAX && kind == crate::actors::kind::MISSILE && was == crate::actors::kind::TRACER {
            hw.poke(p + proj::KIND, kind);
            hw.poke(p + proj::TTL, ttl);
            wr(hw, p + proj::U, (a.u >> 4) as u16);
            wr(hw, p + proj::V, (a.v >> 4) as u16);
            wr(hw, p + proj::H, (a.h >> 4) as u16);
            hw.poke(p + proj::YAW, (a.yaw >> 8) as u8);
            // Along the nose: nose down (pitch negative) descends.
            hw.poke(p + proj::CLIMB, (a.pitch >> 8) as i8 as u8);
            wr(hw, p + proj::TARGET, target);
            return;
        }
    }
}

/// [`ws::heading`] for the effects pass (the missiles homing), in its overlay: the war's copy is in the
/// move phase's, at the same addresses as this one (calling it ran this overlay's code from the middle:
/// the first missile in flight on the C64U wrote projectiles over the HUD's interrupt code).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.fx"))]
fn heading(dx: i16, dy: i16) -> u8 {
    ws::heading_inline(dx, dy)
}

/// Whether a mesh (`squad::entry::MESH`) is a living enemy's.
#[inline(always)]
fn enemy(mesh: u8) -> bool {
    mesh < 2 * crate::actors::kind::COUNT && mesh & 1 == crate::actors::ENEMY
}

/// The frame's fight, after the war's page and before the HUD: the targets for the HUD, the gun and
/// missiles, the projectiles moved, the player down or not. Returns whether the player is down (the main
/// loop then restarts the agent at home).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combat"))]
pub fn update<H: Hw + ?Sized>(hw: &mut H, a: &mut Agent, cam: &crate::floor::FloorCam) -> bool {
    use crate::hud::{Eye, NEAR, S_TARGETS, TARGET, TARGETS, on_screen, screen, to_camera};
    let eye = Eye::new(cam);
    let now = a.t as u16;
    let dt = now.wrapping_sub(rd(hw, PLAYER + player::LAST)).min(255) as u8;
    wr(hw, PLAYER + player::LAST, now);
    let (pu, pv) = a.at();
    // (On the C64 the main loop runs the squad pass, from its own overlay, just before.)
    #[cfg(not(target_arch = "mos"))]
    crate::squad::expand(hw, now, pu, pv, a.yaw);
    // The targets: the enemy vehicles near (`squad::MEMBERS`, nearest first), where they are now; the
    // nearest drawn.
    let n = hw.peek(MEMBERS).min(SLOTS);
    let mut count = 0u8;
    let mut in_cone = (ws::NOBODY, 0u16);
    let mut cone_z = i16::MAX;
    // The autopilot's prey: the one it had, while it lives and is near; else the nearest in range that no
    // missile is after.
    let auto = a.mode == crate::agent::MODE_AUTO;
    let had = rd(hw, PLAYER + player::PREY);
    let (hs, hc) = (crate::camera::sin(a.yaw) as i16, crate::camera::cos(a.yaw) as i16);
    use crate::fixed::mul14;
    let mut prey = (ws::NOBODY, 0i16, 0i16, PREY_RANGE);
    for k in 0..n {
        let e = crate::squad::nth(hw, k);
        // Drawn: (mesh, yaw, u, v, h) into the list.
        if k < OBJLIST_UNITS {
            for i in 0..8 {
                let b = hw.peek(e + entry::MESH + i);
                hw.poke(OBJLIST + 1 + 8 * k as u16 + i, b);
            }
        }
        if !enemy(hw.peek(e + entry::MESH)) || count as usize >= TARGETS {
            continue;
        }
        let id = rd(hw, e + entry::VID);
        let (u, v) = (rd(hw, e + entry::U), rd(hw, e + entry::V));
        let (tu, tv, th) = ((u >> 4) as i16, (v >> 4) as i16, (rd(hw, e + entry::H) >> 4) as i16 + 1);
        let at = S_TARGETS + 1 + count as u16 * TARGET as u16;
        wr(hw, at, tu as u16);
        wr(hw, at + 2, tv as u16);
        hw.poke(at + 4, th as u8);
        wr(hw, at + 5, id);
        count += 1;
        if auto {
            let (dx, dy) = ((tu - pu).abs(), (tv - pv).abs());
            let d = dx.max(dy) + (dx.min(dy) >> 1);
            if id == had {
                prey = (id, tu, tv, -1);
            } else if d < prey.3 && !chased(hw, id) {
                prey = (id, tu, tv, d);
            }
        }
        // In the gun's cone? (Only one within its range, ahead and near the nose's line, is worth the
        // transform: within a quarter of the way ahead and 4 texels to either side, where the cone is
        // under a tenth, and pitch and roll do not turn the nose's line off the heading.)
        let (fwd, lat) = (mul14(hs, tu - pu) + mul14(hc, tv - pv), mul14(hc, tu - pu) - mul14(hs, tv - pv));
        if hw.peek(e + entry::D) <= CONE_D && fwd > 0 && lat.unsigned_abs() * 4 <= fwd as u16 + 16 {
            let c = to_camera(&eye, [tu << 2, th << 2, tv << 2]);
            if c[2] >= NEAR && c[2] <= GUN_RANGE {
                let (x, y) = screen(c);
                if on_screen(x, y) && (x - crate::hud::CX).abs() <= CONE_X && (y - crate::hud::CY).abs() <= CONE_Y && c[2] < cone_z {
                    (in_cone, cone_z) = ((id, e), c[2]);
                }
            }
        }
    }
    hw.poke(S_TARGETS, count);
    hw.poke(OBJLIST, n.min(OBJLIST_UNITS));
    // The gun.
    let input = a.input;
    let cool = hw.peek(PLAYER + player::COOL).saturating_sub(dt);
    hw.poke(PLAYER + player::COOL, cool);
    // (The autopilot's trigger: its tree's strafe; its fire 2, a launch.)
    let firing = if auto { a.action == crate::agent::action::STRAFE } else { input & crate::agent::input::FIRE != 0 };
    if firing && cool == 0 && a.grounded == 0 {
        hw.poke(PLAYER + player::COOL, GUN_EVERY);
        let bursts = hw.peek(PLAYER + player::BURSTS).wrapping_add(1);
        hw.poke(PLAYER + player::BURSTS, bursts);
        launch(hw, crate::actors::kind::TRACER, TRACER_TTL, a, ws::NOBODY);
        if in_cone.0 != ws::NOBODY && (bursts.wrapping_mul(97) ^ now as u8) < GUN_HIT {
            let e = in_cone.1;
            let (tu, tv, th) = (rd(hw, e + entry::U), rd(hw, e + entry::V), rd(hw, e + entry::H));
            let fx = if sea(hw, tu, tv) { crate::effects::effect::SPLASH } else { crate::effects::effect::IMPACT };
            spawn(hw, fx, tu, tv, th, now);
            dealt(hw, GUN_DAMAGE);
            if damage_unit(hw, in_cone.0, GUN_DAMAGE) {
                let k = rd(hw, PLAYER + player::KILLS);
                wr(hw, PLAYER + player::KILLS, k + 1);
                dealt(hw, KILL_BONUS);
                spawn(hw, crate::effects::effect::EXPLOSION, tu, tv, th, now);
                spawn(hw, crate::effects::effect::WRECK_FIRE, tu, tv, th, now.wrapping_add(FIRE_AFTER));
            }
        }
    }
    // Missiles: on fire 2's press, at the locked target.
    let fire2 = if auto { (a.action == crate::agent::action::LAUNCH) as u8 } else { input & crate::agent::input::FIRE2 };
    let was = hw.peek(PLAYER + player::FIRE2);
    hw.poke(PLAYER + player::FIRE2, fire2);
    let locked = hw.peek(crate::hud::LOCK_STATE + 2) >= crate::hud::LOCK_TICKS;
    // The locked vehicle (`hud::LOCK_STATE`), while still listed.
    let id = rd(hw, crate::hud::LOCK_STATE);
    let missiles = hw.peek(PLAYER + player::MISSILES);
    if fire2 != 0 && was == 0 && locked && missiles > 0 && id != ws::NOBODY && crate::squad::find(hw, id).is_some() {
        hw.poke(PLAYER + player::MISSILES, missiles - 1);
        launch(hw, crate::actors::kind::MISSILE, MISSILE_TTL, a, id);
    }
    // Projectiles that arrived (the effects pass moved them): missiles at their targets, tracers in the
    // ground or the sea.
    for k in 0..PROJ_MAX {
        let p = PROJ + k * PROJ_ENTRY;
        let kind = hw.peek(p + proj::KIND);
        let (u, v, h) = (rd(hw, p + proj::U), rd(hw, p + proj::V), rd(hw, p + proj::H));
        if kind == LANDED {
            let fx = if sea(hw, u, v) { crate::effects::effect::SPLASH } else { crate::effects::effect::IMPACT };
            spawn(hw, fx, u, v, 0, now);
            hw.poke(p + proj::KIND, 0);
        } else if kind == crate::actors::kind::MISSILE {
            let id = rd(hw, p + proj::TARGET);
            // Its target where it is now, while listed (else it flies on, lost).
            let Some(e) = crate::squad::find(hw, id) else { continue };
            let (tu, tv, th) = (rd(hw, e + entry::U), rd(hw, e + entry::V), rd(hw, e + entry::H));
            if ((tu >> 4) as i16 - (u >> 4) as i16).abs() < 3 && ((tv >> 4) as i16 - (v >> 4) as i16).abs() < 3 && ((th >> 4) as i16 - (h >> 4) as i16).abs() < 3 {
                spawn(hw, crate::effects::effect::EXPLOSION, tu, tv, th, now);
                dealt(hw, MISSILE_DAMAGE);
                if damage_unit(hw, id, MISSILE_DAMAGE) {
                    let k = rd(hw, PLAYER + player::KILLS);
                    wr(hw, PLAYER + player::KILLS, k + 1);
                    dealt(hw, KILL_BONUS);
                    spawn(hw, crate::effects::effect::WRECK_FIRE, tu, tv, th, now.wrapping_add(FIRE_AFTER));
                }
                hw.poke(p + proj::KIND, 0);
            }
        }
    }
    // Hit flash fades; down at full damage.
    let flash = hw.peek(PLAYER + player::FLASH);
    hw.poke(PLAYER + player::FLASH, flash.saturating_sub(dt));
    // Down at full damage; ditched in the sea (`agent::DITCHED`) too, with a splash. (Before any repair:
    // a hit that downs it on the pad still does.)
    // The learning: a decision at a learn node files it on the trail, crediting the last ones with what
    // came of them; going down credits them too, and starts afresh.
    let ditched = a.deck == crate::agent::DITCHED;
    let damage = hw.peek(PLAYER + player::DAMAGE);
    let down = ditched || damage >= DOWN;
    let node = u16::from_le_bytes(a.learned);
    if node != 0 || down {
        file_learned(hw, node);
        a.learned = [0; 2];
    }
    if down {
        init(hw);
        if ditched {
            spawn(hw, crate::effects::effect::SPLASH, (a.u >> 4) as u16, (a.v >> 4) as u16, 0, now);
        }
        return true;
    }
    // Landed at a base (home's pad, the carrier's deck): rearmed, and repaired a point a frame.
    let home = (a.u >> 8) as i16 - crate::agent::HOME.0 as i16;
    let homev = (a.v >> 8) as i16 - crate::agent::HOME.1 as i16;
    let based = a.grounded != 0 && (a.deck == crate::agent::ON_DECK || home.abs() <= PAD && homev.abs() <= PAD);
    if based {
        hw.poke(PLAYER + player::MISSILES, MISSILES);
        hw.poke(PLAYER + player::DAMAGE, damage.saturating_sub(1));
    }
    // The autopilot's fight for its tree: hurt (home: from its damage or an empty rack, until landed there
    // and whole again), else hunting its prey (the waypoint is it), locked on with a missile to fire.
    use crate::agent::combat::{HUNT, HURT as SORE, LOCKED};
    let mut flags = 0;
    if auto {
        let missiles = hw.peek(PLAYER + player::MISSILES);
        let sore = (a.combat & SORE != 0 && !(based && damage == 0)) || damage >= HURT || missiles == 0;
        if sore {
            flags = SORE;
            (a.wu, a.wv) = (crate::agent::HOME.0 as i16, crate::agent::HOME.1 as i16);
            prey.0 = ws::NOBODY;
        } else if prey.0 != ws::NOBODY {
            flags = HUNT | if locked && missiles > 0 && id != ws::NOBODY { LOCKED } else { 0 };
            (a.wu, a.wv) = (prey.1, prey.2);
        }
    }
    a.combat = flags;
    wr(hw, PLAYER + player::PREY, prey.0);
    false
}

/// Whether a missile in flight is after unit `id`.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
fn chased<H: Hw + ?Sized>(hw: &mut H, id: u16) -> bool {
    (0..PROJ_MAX).any(|k| {
        let p = PROJ + k * PROJ_ENTRY;
        hw.peek(p + proj::KIND) == crate::actors::kind::MISSILE && rd(hw, p + proj::TARGET) == id
    })
}

/// Damage the player dealt (the learning's counter).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
fn dealt<H: Hw + ?Sized>(hw: &mut H, d: u8) {
    let n = hw.peek(PLAYER + player::DEALT).wrapping_add(d);
    hw.poke(PLAYER + player::DEALT, n);
}

/// A decision at a learn node (`node`, or 0 going down): the trail credited with what came of its
/// decisions since the newest, then the node filed as the newest, with the counters now.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
fn file_learned<H: Hw + ?Sized>(hw: &mut H, node: u16) {
    let t = PLAYER + player::TRAIL;
    let (d, tk) = (hw.peek(PLAYER + player::DEALT), hw.peek(PLAYER + player::TAKEN));
    let reward = (d.wrapping_sub(hw.peek(t + 2)) as i16 - tk.wrapping_sub(hw.peek(t + 3)) as i16).clamp(-128, 127) as i8;
    learn_pilot(hw, reward);
    for k in (0..player::TRAIL_ENTRY).rev() {
        let b = hw.peek(t + k);
        hw.poke(t + player::TRAIL_ENTRY + k, b);
    }
    wr(hw, t, node);
    hw.poke(t + 2, d);
    hw.poke(t + 3, tk);
}

/// The pilot's learning (`brain::learn`'s rule on its blob in the REU): `reward` (damage dealt minus
/// taken since the trail's newest decision) against the pilot's baseline; the branch each learn node on
/// the trail took is made likelier after a better than expected outcome, less likely after a worse one,
/// the older at half the step.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn learn_pilot<H: Hw + ?Sized>(hw: &mut H, reward: i8) {
    use crate::brain::{LEARN_MAX, LEARN_MIN, PILOT_BASELINE};
    let base = crate::format::AI_TREES;
    let b = base + PILOT_BASELINE as u32;
    let baseline = reu_get(hw, b) as i8;
    let adv = (reward as i16 - (baseline as i16 >> 3)).clamp(-127, 127);
    reu_put(hw, b, (baseline as i16 + adv).clamp(-128, 127) as i8 as u8);
    if adv == 0 {
        return;
    }
    let step = ((adv.unsigned_abs() << 1).min(32)) as u8;
    for k in 0..2u16 {
        let node = rd(hw, PLAYER + player::TRAIL + k * player::TRAIL_ENTRY);
        let s = step >> k;
        if node & 0x7FFF == 0 || s == 0 {
            continue;
        }
        let at = base + (node & 0x7FFF) as u32 + 1;
        let v = reu_get(hw, at);
        let p = if (node >> 15 != 0) == (adv > 0) { v.saturating_add(s).min(LEARN_MAX) } else { v.saturating_sub(s).max(LEARN_MIN) };
        reu_put(hw, at, p);
    }
}

/// The effects pass (after the main one, its own overlay): projectiles moved (missiles turning toward
/// their targets), listed to draw (`PROJOBJ`); the effects playing placed.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.fx"))]
pub fn fx<H: Hw + ?Sized>(hw: &mut H, a: &Agent, cam: &crate::floor::FloorCam) {
    let eye = crate::hud::Eye::new(cam);
    let now = a.t as u16;
    let dt = now.wrapping_sub(rd(hw, PLAYER + player::LAST_FX)).min(255) as u8;
    wr(hw, PLAYER + player::LAST_FX, now);
    // The projectiles: on along their heading (missiles turning toward their target), out of life.
    for k in 0..PROJ_MAX {
        let p = PROJ + k * PROJ_ENTRY;
        let kind = hw.peek(p + proj::KIND);
        if kind == 0 {
            continue;
        }
        let ttl = hw.peek(p + proj::TTL).saturating_sub(dt);
        if ttl == 0 {
            hw.poke(p + proj::KIND, 0);
            continue;
        }
        hw.poke(p + proj::TTL, ttl);
        let (mut u, mut v, mut h) = (rd(hw, p + proj::U) as i16, rd(hw, p + proj::V) as i16, rd(hw, p + proj::H) as i16);
        let mut yaw = hw.peek(p + proj::YAW);
        let mut climb = hw.peek(p + proj::CLIMB) as i8 as i16;
        let speed = if kind == crate::actors::kind::MISSILE { MISSILE_SPEED } else { TRACER_SPEED };
        // A missile turns toward its target, while it is listed (else it flies on, lost).
        let target = rd(hw, p + proj::TARGET);
        if let Some(e) = if kind == crate::actors::kind::MISSILE { crate::squad::find(hw, target) } else { None } {
            let (tu, tv, th) = (rd(hw, e + entry::U) as i16, rd(hw, e + entry::V) as i16, rd(hw, e + entry::H) as i16);
            let (dx, dy, dh) = ((tu >> 4) - (u >> 4), (tv >> 4) - (v >> 4), th - h);
            // Turn toward it: 8 a tick at most; climb toward its height.
            let want = heading(dx, dy);
            let d = want.wrapping_sub(yaw) as i8;
            yaw = yaw.wrapping_add(d.clamp(-8, 8) as u8);
            climb = (dh >> 4).clamp(-8, 8);
            hw.poke(p + proj::YAW, yaw);
            hw.poke(p + proj::CLIMB, climb as i8 as u8);
        }
        let step = speed as i16 * dt as i16;
        let a16 = (yaw as u16) << 8;
        u = u.wrapping_add((crate::hud::mul16(crate::camera::sin(a16) as i16, step) >> 14) as i16);
        v = v.wrapping_add((crate::hud::mul16(crate::camera::cos(a16) as i16, step) >> 14) as i16);
        h += climb * dt as i16;
        if h <= 0 && kind == crate::actors::kind::TRACER {
            // Into the ground (or the sea): the main pass makes the dust or spray.
            hw.poke(p + proj::KIND, LANDED);
        }
        let h = h.max(0);
        wr(hw, p + proj::U, u as u16);
        wr(hw, p + proj::V, v as u16);
        wr(hw, p + proj::H, h as u16);
    }
    effects(hw, &eye, now, (a.t >> crate::clouds::DRIFT_SHIFT) as u16);
    // The projectiles to draw, after the near units.
    let mut n = hw.peek(OBJLIST).min(OBJLIST_UNITS) as u16;
    for k in 0..PROJ_MAX {
        let p = PROJ + k * PROJ_ENTRY;
        let kind = hw.peek(p + proj::KIND);
        if kind == 0 || kind == LANDED {
            continue;
        }
        let at = OBJLIST + 1 + 8 * n;
        hw.poke(at, crate::actors::mesh_of(kind, crate::actors::ALLIED));
        let yaw = hw.peek(p + proj::YAW);
        hw.poke(at + 1, yaw);
        for w in 0..6 {
            let b = hw.peek(p + proj::U + w);
            hw.poke(at + 2 + w, b);
        }
        n += 1;
    }
    hw.poke(OBJLIST, n as u8);
}

/// The frame's objects to draw, for the actors overlay: a count, then (mesh, yaw, u, v, h) 8 bytes each:
/// the near units (the main pass), then the projectiles (the effects pass). In the cell buffer, free
/// until the geometry.
pub const OBJLIST: u16 = crate::layout::CELL_RAM;
/// The nearest vehicles (`squad::MEMBERS`: the carrier's deck among them), the projectiles.
pub const OBJLIST_UNITS: u8 = crate::world_sim::NEAR_MAX + 1;
const _: () = assert!(OBJLIST_UNITS <= SLOTS);
pub const OBJLIST_MAX: u8 = OBJLIST_UNITS + PROJ_MAX as u8;
const _: () = assert!(1 + 8 * OBJLIST_MAX as u16 <= crate::layout::CELL_MAX * 8);

/// A projectile that hit the ground, for the main pass to turn into dust or spray.
pub const LANDED: u8 = 0xFE;

/// A fresh start: no projectiles, no damage, fully armed.
pub fn init<H: Hw + ?Sized>(hw: &mut H) {
    hw.fill(STATE, STATE_LEN, 0);
    hw.fill(FX, FX_MAX as u16 * FX_ENTRY, 0);
    hw.poke(FXDYN, 0);
    hw.poke(PLAYER + player::MISSILES, MISSILES);
}
