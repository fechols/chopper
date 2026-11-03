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
/// kills (u16), the last fire 2 (for its press), the hit flash (ticks), the tick of the last frame (u16).
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
}
pub const PLAYER_LEN: u16 = 12;
const _: () = assert!(PLAYER + PLAYER_LEN <= crate::layout::XFORM_RAM);
/// Everything of this the frames carry (the snapshot gate).
pub const STATE: u16 = PROJ;
pub const STATE_LEN: u16 = PROJ_MAX * PROJ_ENTRY + PLAYER_LEN;

/// The effects playing (`effects`): [`FX_MAX`] entries of [`FX_ENTRY`] bytes: effect + 1 (0 free), spare,
/// the tick it began (u16), where (1/16 texel, u16 × 3). After the relocated `bandtext`'s last page.
pub const FX: u16 = 0xB760;
pub const FX_MAX: u8 = 8;
pub const FX_ENTRY: u16 = 10;
const _: () = assert!(FX + FX_MAX as u16 * FX_ENTRY <= 0xB800);
/// The frame's effects in view, for `geom::push_dynamic`: a count, then [`FXDYN_ENTRY`] bytes each: left
/// x (multicolour pixels) and y (line) of the anchor (i16 × 2), the frame record's REU address (u24),
/// the depth key. (After the mesh buffer, before the HUD's page.)
pub const FXDYN: u16 = 0x8AB0;
pub const FXDYN_ENTRY: u16 = 8;
const _: () = assert!(FXDYN + 1 + FX_MAX as u16 * FXDYN_ENTRY <= crate::hud::PAGE);
/// Effect frames last `1 << FX_TICKS_SHIFT` ticks; looping effects (smoke, a burning wreck) play this
/// long.
pub const FX_TICKS_SHIFT: u32 = 2;
pub const FX_LIFE: u16 = 20 * 60;

/// Start `effect` at (`u`, `v`, `h`) (1/16 texel): a free slot, or the oldest.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn spawn<H: Hw + ?Sized>(hw: &mut H, effect: u8, u: u16, v: u16, h: u16, now: u16) {
    let (mut best, mut age) = (0u16, 0u16);
    for k in 0..FX_MAX as u16 {
        let e = FX + k * FX_ENTRY;
        let a = if hw.peek(e) == 0 { 0xFFFF } else { now.wrapping_sub(rd(hw, e + 2)) };
        if a >= age {
            (best, age) = (e, a);
        }
    }
    hw.poke(best, effect + 1);
    wr(hw, best + 2, now);
    wr(hw, best + 4, u);
    wr(hw, best + 6, v);
    wr(hw, best + 8, h);
}

/// Is (`u`, `v`) (1/16 texel) over the sea (the hazard map's bit 7)?
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn sea<H: Hw + ?Sized>(hw: &mut H, u: u16, v: u16) -> bool {
    let cell = (v >> (4 + crate::format::HAZARD_SHIFT)) as u32 * crate::format::HAZARD_CELLS as u32 + (u >> (4 + crate::format::HAZARD_SHIFT)) as u32;
    reu_get(hw, crate::format::HAZARD + cell) & 0x80 != 0
}

/// The effects on: each one's frame, its scale for its distance, placed for the drawing; done ones freed.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.fx"))]
fn effects<H: Hw + ?Sized>(hw: &mut H, eye: &crate::hud::Eye, now: u16) {
    use crate::effects::{FRAMES, HEIGHT_Q4, LOOPS, record, scale_for};
    let mut n = 0u8;
    for k in 0..FX_MAX as u16 {
        let e = FX + k * FX_ENTRY;
        let id = hw.peek(e);
        if id == 0 {
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
            // (The looping effects have 8 frames.)
            f &= 7;
        }
        let p = [(rd(hw, e + 4) >> 2) as i16, (rd(hw, e + 8) >> 2) as i16, (rd(hw, e + 6) >> 2) as i16];
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
        hw.poke(at + 7, crate::geom::depth_key(3 * z4));
        n += 1;
    }
    hw.poke(FXDYN, n);
}

pub const GUN_EVERY: u8 = 6;
pub const GUN_RANGE: i16 = 80 * 4;
pub const GUN_HIT: u8 = 180;
pub const GUN_DAMAGE: u8 = 2;
pub const CONE_X: i16 = 12;
pub const CONE_Y: i16 = 10;
pub const MISSILES: u8 = 8;
pub const MISSILE_DAMAGE: u8 = 40;
pub const DOWN: u8 = 100;
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

/// Damage unit `id` (in the REU): its hit points down, its damage taken up (for its learning); dead at 0,
/// with the bookkeeping. Whether it died.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combatlib"))]
pub fn damage_unit<H: Hw + ?Sized>(hw: &mut H, id: u16, dmg: u8) -> bool {
    let base = ws::UNITS + id as u32 * RECORD as u32;
    let flags = reu_get(hw, base + rec::FLAGS as u32);
    if flags & ALIVE == 0 {
        return false;
    }
    let hp = reu_get(hw, base + rec::HP as u32).saturating_sub(dmg);
    reu_put(hw, base + rec::HP as u32, hp);
    let taken = reu_get(hw, base + rec::TAKEN as u32).saturating_add(dmg);
    reu_put(hw, base + rec::TAKEN as u32, taken);
    if hp > 0 {
        return false;
    }
    reu_put(hw, base + rec::FLAGS as u32, flags & !(ALIVE | NEAR_FLAG));
    let side = (flags & SIDE) as u32;
    let sector = reu_get(hw, base + rec::SECTOR as u32);
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

/// The frame's fight, after the war's page and before the HUD: the targets for the HUD, the gun and
/// missiles, the projectiles moved, the player down or not. Returns whether the player is down (the main
/// loop then restarts the agent at home).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.combat"))]
pub fn update<H: Hw + ?Sized>(hw: &mut H, a: &Agent, cam: &crate::floor::FloorCam) -> bool {
    use crate::hud::{Eye, NEAR, S_TARGETS, TARGET, TARGETS, on_screen, screen, to_camera};
    let eye = Eye::new(cam);
    let now = a.t as u16;
    let dt = now.wrapping_sub(rd(hw, PLAYER + player::LAST)).min(255) as u8;
    wr(hw, PLAYER + player::LAST, now);
    // The targets: living enemy units near, where they are now.
    let n = hw.peek(ws::NEAR).min(ws::NEAR_MAX);
    let mut count = 0u8;
    let mut in_cone = ws::NOBODY;
    let mut cone_z = i16::MAX;
    let mut r = [0u8; 18];
    for k in 0..n as u16 {
        let id = rd(hw, ws::NEAR + 1 + 2 * k);
        hw::read_into(hw, ws::UNITS + id as u32 * RECORD as u32, crate::layout::BOUNCE, &mut r);
        let flags = r[rec::FLAGS as usize];
        let alive = flags & ALIVE != 0;
        // Where it is now: moved on from its last step (the dead stay where they fell).
        let (mut u, mut v) = (u16::from_le_bytes([r[4], r[5]]) as i32, u16::from_le_bytes([r[6], r[7]]) as i32);
        if alive && r[rec::ACTION as usize] != ws::act::HOLD {
            let d = r[rec::SPEED as usize] as i32 * now.wrapping_sub(u16::from_le_bytes([r[16], r[17]])).min(120) as i32;
            let yaw = (r[rec::YAW as usize] as u16) << 8;
            u += crate::camera::sin(yaw) * d >> 14;
            v += crate::camera::cos(yaw) * d >> 14;
        }
        // Drawn: (mesh, yaw, u, v, h) into the list.
        let o = OBJLIST + 1 + 8 * k;
        hw.poke(o, crate::actors::mesh_of(r[rec::KIND as usize], flags & SIDE));
        hw.poke(o + 1, r[rec::YAW as usize]);
        wr(hw, o + 2, u as u16);
        wr(hw, o + 4, v as u16);
        wr(hw, o + 6, (r[rec::H as usize] as u16) << 4);
        if !alive || flags & SIDE != crate::actors::ENEMY || count as usize >= TARGETS {
            continue;
        }
        let (tu, tv, th) = ((u >> 4) as i16, (v >> 4) as i16, r[rec::H as usize] as i16 + 1);
        let at = S_TARGETS + 1 + count as u16 * TARGET as u16;
        wr(hw, at, tu as u16);
        wr(hw, at + 2, tv as u16);
        hw.poke(at + 4, th as u8);
        count += 1;
        // In the gun's cone?
        let c = to_camera(&eye, [tu << 2, th << 2, tv << 2]);
        if c[2] >= NEAR && c[2] <= GUN_RANGE {
            let (x, y) = screen(c);
            if on_screen(x, y) && (x - 160).abs() <= CONE_X && (y - 68).abs() <= CONE_Y && c[2] < cone_z {
                (in_cone, cone_z) = (id, c[2]);
            }
        }
    }
    hw.poke(S_TARGETS, count);
    hw.poke(OBJLIST, n);
    // The gun.
    let input = a.input;
    let cool = hw.peek(PLAYER + player::COOL).saturating_sub(dt);
    hw.poke(PLAYER + player::COOL, cool);
    if input & crate::agent::input::FIRE != 0 && a.mode == crate::agent::MODE_PLAYER && cool == 0 && a.grounded == 0 {
        hw.poke(PLAYER + player::COOL, GUN_EVERY);
        let bursts = hw.peek(PLAYER + player::BURSTS).wrapping_add(1);
        hw.poke(PLAYER + player::BURSTS, bursts);
        launch(hw, crate::actors::kind::TRACER, TRACER_TTL, a, ws::NOBODY);
        if in_cone != ws::NOBODY && (bursts.wrapping_mul(97) ^ now as u8) < GUN_HIT {
            let mut t = [0u8; 9];
            hw::read_into(hw, ws::UNITS + in_cone as u32 * RECORD as u32, crate::layout::BOUNCE, &mut t);
            let (tu, tv, th) = (u16::from_le_bytes([t[4], t[5]]), u16::from_le_bytes([t[6], t[7]]), (t[8] as u16) << 4);
            let fx = if sea(hw, tu, tv) { crate::effects::effect::SPLASH } else { crate::effects::effect::IMPACT };
            spawn(hw, fx, tu, tv, th, now);
            if damage_unit(hw, in_cone, GUN_DAMAGE) {
                let k = rd(hw, PLAYER + player::KILLS);
                wr(hw, PLAYER + player::KILLS, k + 1);
                spawn(hw, crate::effects::effect::EXPLOSION, tu, tv, th, now);
                spawn(hw, crate::effects::effect::WRECK_FIRE, tu, tv, th, now.wrapping_add(40));
            }
        }
    }
    // Missiles: on fire 2's press, at the locked target.
    let fire2 = input & crate::agent::input::FIRE2;
    let was = hw.peek(PLAYER + player::FIRE2);
    hw.poke(PLAYER + player::FIRE2, fire2);
    let locked = hw.peek(crate::hud::LOCK_STATE + 1) >= crate::hud::LOCK_TICKS;
    let target = hw.peek(crate::hud::LOCK_STATE);
    let missiles = hw.peek(PLAYER + player::MISSILES);
    if fire2 != 0 && was == 0 && locked && missiles > 0 && target < count {
        // The locked target's unit: the target list follows the near list, enemies only.
        let mut j = 0;
        let mut id = ws::NOBODY;
        for k in 0..n as u16 {
            let u = rd(hw, ws::NEAR + 1 + 2 * k);
            let f = reu_get(hw, ws::UNITS + u as u32 * RECORD as u32 + rec::FLAGS as u32);
            if f & ALIVE != 0 && f & SIDE == crate::actors::ENEMY {
                if j == target {
                    id = u;
                    break;
                }
                j += 1;
            }
        }
        if id != ws::NOBODY {
            hw.poke(PLAYER + player::MISSILES, missiles - 1);
            launch(hw, crate::actors::kind::MISSILE, MISSILE_TTL, a, id);
        }
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
            let mut t = [0u8; 9];
            hw::read_into(hw, ws::UNITS + id as u32 * RECORD as u32, crate::layout::BOUNCE, &mut t);
            let (tu, tv, th) = (u16::from_le_bytes([t[4], t[5]]), u16::from_le_bytes([t[6], t[7]]), (t[8] as u16) << 4);
            if ((tu >> 4) as i16 - (u >> 4) as i16).abs() < 3 && ((tv >> 4) as i16 - (v >> 4) as i16).abs() < 3 && ((th >> 4) as i16 - (h >> 4) as i16).abs() < 3 {
                spawn(hw, crate::effects::effect::EXPLOSION, tu, tv, th, now);
                if damage_unit(hw, id, MISSILE_DAMAGE) {
                    let k = rd(hw, PLAYER + player::KILLS);
                    wr(hw, PLAYER + player::KILLS, k + 1);
                    spawn(hw, crate::effects::effect::WRECK_FIRE, tu, tv, th, now.wrapping_add(40));
                }
                hw.poke(p + proj::KIND, 0);
            }
        }
    }
    // Hit flash fades; down at full damage.
    let flash = hw.peek(PLAYER + player::FLASH);
    hw.poke(PLAYER + player::FLASH, flash.saturating_sub(dt));
    if hw.peek(PLAYER + player::DAMAGE) >= DOWN {
        init(hw);
        return true;
    }
    false
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
        if kind == crate::actors::kind::MISSILE {
            let id = rd(hw, p + proj::TARGET);
            let base = ws::UNITS + id as u32 * RECORD as u32;
            let mut t = [0u8; 9];
            hw::read_into(hw, base, crate::layout::BOUNCE, &mut t);
            let (tu, tv, th) = (u16::from_le_bytes([t[4], t[5]]) as i16, u16::from_le_bytes([t[6], t[7]]) as i16, (t[8] as i16) << 4);
            let (dx, dy, dh) = ((tu >> 4) - (u >> 4), (tv >> 4) - (v >> 4), th - h);
            // Turn toward it: 8 a tick at most; climb toward its height.
            let want = crate::world_sim::heading(dx, dy);
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
    effects(hw, &eye, now);
    // The projectiles to draw, after the near units.
    let mut n = hw.peek(OBJLIST).min(crate::world_sim::NEAR_MAX) as u16;
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
pub const OBJLIST_MAX: u8 = crate::world_sim::NEAR_MAX + PROJ_MAX as u8;
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
