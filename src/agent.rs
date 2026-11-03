//! The attract-mode helicopter: a small flight sim stepped once per NTSC frame of the clock, with the
//! same bits on the host and the C64, steered by the baked decision tree (`brain`, planned by
//! `goap`). The clock starts with the rotor sound (`asm/audio.S`). The helicopter sits on the home pad
//! until the rotor is at full power (the sound's 1:20), lifts off, and then roams: it flies to seeded
//! random waypoints over the island, climbing over or turning round the mountains its probes see in the
//! baked hazard map (`format::HAZARD`).
//!
//! The tree only chooses what to do; a tiny helicopter flight model does the flying. Each action sets
//! targets for the controls: cyclic (a bank angle, a pitch attitude) and collective (a climb rate).
//! The attitude follows its targets through a lag and a rate limit, as a rotor disc would. The bank
//! turns the helicopter (yaw rate ∝ bank), and the pitch tilts the rotor's thrust: nose down
//! accelerates against drag, so each pitch settles at its own speed. Turns toward the waypoint bank in
//! proportion to the heading error. Nothing jumps: the camera shows the attitude itself.
//!
//! On the C64 the code runs from the AI overlay (`.text.ai`, DMA'd to `layout::WORK` each frame).

use crate::brain::{self, Reu};
use crate::camera::{Matrix14, QUARTER_TURN, sin};
use crate::floor::FloorCam;
use crate::fixed::mul;
use crate::format::{AI_TREES, HAZARD, HAZARD_CELLS, HAZARD_SHIFT};
use crate::world_sim::{CARRIER_RECORD, DECK_HALF, DECK_TOP, Deck, carrier_record, deck_of};

/// On the pad until the rotor is at full power: the sound's 1:20 (`digital::ROTOR_LOOP_START`).
pub const LIFT: u32 = 80 * 60;
/// Eye height on the pad: half a texel (8.8).
pub const EYE: i32 = 128;
/// Eye height over the carrier's deck (8.8): over 1 texel, so the deck at the 2-texel near plane is
/// below the view's bottom edge and the deck fills the view down to it (the pad is floor texture, drawn
/// to the edge at any height).
pub const DECK_EYE: i32 = 288;
/// The home pad (`world::PADS[0]`), texels.
pub const HOME: (i32, i32) = (1024, 760);
/// The soft boundary at sea, texels beyond the map's edge: the caution lamp (`cockpit::lamp::SEA`) past
/// `SEA_WARN`; past `SEA_LIMIT` the autopilot takes over the player's helicopter and turns it for home
/// (its turn carries it about 200 texels further: positions stay well inside 16 bits).
pub const SEA_WARN: i16 = 150;
pub const SEA_LIMIT: i16 = 300;
/// Cruise altitude, texels (over the 6-texel tower; mountains are 22-34).
pub const CRUISE: i16 = 16;
/// Off the pad (`grounded` clears) at this height, 8.8: over the tower before moving.
pub const LIFTED: i32 = 10 * 256;
/// Ticks between decisions.
pub const THINK: u32 = 8;
/// One degree in binary angle units.
const DEG: i16 = 182;
/// The controls' targets per action (indexed by `action`): bank (binary angle; 0 for the turns, whose
/// bank comes from the heading error), pitch attitude (nose down negative: −11° cruises at 0.5 texel a
/// tick), climb rate (8.8 texels a tick).
const TARGET_BANK: [i16; 16] = [0, 0, 0, 0, 0, 0, -30 * DEG, 30 * DEG, 0, 0, 0, 0, 0, 0, 0, 0];
const TARGET_PITCH: [i16; 16] = [0, 0, -1024, -2048, -2048, -2048, -1024, -1024, -2048, -2048, 0, 0, 0, 0, 0, 0];
const TARGET_CLIMB: [i16; 16] = [0, 24, 40, 0, 0, 0, 0, 0, -40, 0, 0, 0, 0, 0, 0, 0];
/// Turns bank between these (binary angle), in proportion to the heading error.
const TURN_BANK_MIN: i16 = 10 * DEG;
const TURN_BANK_MAX: i16 = 20 * DEG;
/// Bank per unit of heading error (`features`' `steer` scale: ±32 ≈ the waypoint abeam).
const STEER_GAIN: i16 = 320;
/// Attitude rate limits (binary angle a tick: about 21°/s roll, 11°/s pitch) after a 1/8 lag.
const ROLL_RATE: i16 = 64;
const PITCH_RATE: i16 = 32;
/// The climb rate's change a tick (8.8 texels a tick²).
const EASE_CLIMB: i16 = 1;
/// Yaw rate = bank >> TURN_SHIFT (30° of bank: 85 binary angle a tick, about 28°/s).
const TURN_SHIFT: u32 = 6;
/// The probes: feature, angle off the heading, texels out (two near ahead, one far, one each side).
const PROBES: [(u8, u16, i16); 5] = [
    (feature::HAZ_NEAR as u8, 0, 24),
    (feature::HAZ_NEAR as u8, 0, 56),
    (feature::HAZ_FAR as u8, 0, 112),
    (feature::HAZ_LEFT as u8, 0xE000, 48),
    (feature::HAZ_RIGHT as u8, 0x2000, 48),
];

/// The decision features (u8 each; `goap::SYMBOLS` names the thresholds the tree tests).
pub mod feature {
    /// 1 on the ground.
    pub const GROUND: usize = 0;
    /// 1 once the rotor is at full power (the clock at `LIFT`).
    pub const ROTOR: usize = 1;
    /// Altitude − cruise altitude + 128, texels.
    pub const ALT: usize = 2;
    /// The waypoint: 64 left, 128 ahead (within about 14°), 192 right.
    pub const BEARING: usize = 3;
    /// Distance to the waypoint, 8-texel units (capped at 255).
    pub const DIST: usize = 4;
    /// The probes: obstacle top − altitude + 128, texels.
    pub const HAZ_NEAR: usize = 5;
    pub const HAZ_FAR: usize = 6;
    pub const HAZ_LEFT: usize = 7;
    pub const HAZ_RIGHT: usize = 8;
    /// The largest of the four.
    pub const HAZ_MAX: usize = 9;
    pub const COUNT: usize = 10;
    /// The feature block's size: a power of two, so slots index it with `& 15` and no bounds check.
    pub const BLOCK: usize = 16;
}

/// The tree's leaves.
pub mod action {
    pub const IDLE: u8 = 0;
    pub const LIFT: u8 = 1;
    pub const CLIMB: u8 = 2;
    pub const FORWARD: u8 = 3;
    pub const TURN_L: u8 = 4;
    pub const TURN_R: u8 = 5;
    pub const EVADE_L: u8 = 6;
    pub const EVADE_R: u8 = 7;
    pub const DESCEND: u8 = 8;
    pub const NEXT_WP: u8 = 9;
    pub const COUNT: u8 = 10;
    pub const NAMES: [&str; COUNT as usize] =
        ["idle", "lift", "climb", "forward", "turn-l", "turn-r", "evade-l", "evade-r", "descend", "next-wp"];
}

/// The helicopter's state: [`Agent::BYTES`] bytes, laid out as [`Agent::to_bytes`] (the C64's `AGENT`
/// in RAM; `tools/emu.py --start` pokes it).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Agent {
    /// Ticks simulated.
    pub t: u32,
    /// Position and altitude, 8.8 texels.
    pub u: i32,
    pub v: i32,
    pub h: i32,
    pub yaw: u16,
    /// Airspeed, 8.8 texels a tick × 64 (`speed()`).
    pub vf: i16,
    /// Attitude: bank (right wing down positive, turning right), pitch (nose down negative).
    pub roll: i16,
    pub pitch: i16,
    pub climb: i16,
    /// The waypoint, texels.
    pub wu: i16,
    pub wv: i16,
    pub seed: u16,
    /// The bank a turn toward the waypoint wants (from the heading error, at each decision).
    pub steer: i16,
    pub action: u8,
    pub grounded: u8,
    /// The stick this frame (`input` bits), who flies (`MODE_*`), the player's speed setting (0..=64).
    pub input: u8,
    pub mode: u8,
    pub throttle: u8,
    /// On the carrier's deck (`ON_DECK`), to be placed on it or at home (`PLACE_CARRIER`, `PLACE_HOME`), or
    /// ditched in the sea (`DITCHED`, for `combat`); 0 otherwise.
    pub deck: u8,
    /// On the deck: where (1/16 texel across and along it) and the heading, relative to the carrier's.
    pub du: i16,
    pub dv: i16,
    pub dyaw: u8,
    pub pad: [u8; 3],
}

/// `Agent::deck`.
pub const ON_DECK: u8 = 1;
pub const DITCHED: u8 = 2;
pub const PLACE_CARRIER: u8 = 3;
pub const PLACE_HOME: u8 = 4;

/// The stick (active high; the main loop reads joystick 2 and button 2 into `Agent::input`).
pub mod input {
    pub const UP: u8 = 1;
    pub const DOWN: u8 = 2;
    pub const LEFT: u8 = 4;
    pub const RIGHT: u8 = 8;
    pub const FIRE: u8 = 16;
    pub const FIRE2: u8 = 32;
}
/// Who flies: the attract-mode autopilot, or the player (from the first press of fire: the main loop
/// switches `Agent::mode` as it reads the stick).
pub const MODE_AUTO: u8 = 0;
pub const MODE_PLAYER: u8 = 1;
/// The player's stick: full throttle (64 ticks from a hover), the bank of a turn, the flare when slowing.
pub const THROTTLE_MAX: u8 = 64;
const PLAYER_BANK: i16 = 25 * DEG;
const FLARE: i16 = 768;

impl Default for Agent {
    fn default() -> Self {
        Self::new()
    }
}

#[inline(always)]
fn ease(x: &mut i16, target: i16, rate: i16) {
    *x = if *x < target { (*x + rate).min(target) } else { (*x - rate).max(target) };
}

/// Toward `target`: an eighth of the way (to within 7), at most `rate` a tick. (Resident on the C64:
/// the overlay is full.)
#[inline(never)]
fn follow(x: &mut i16, target: i16, rate: i16) {
    *x += ((target - *x) >> 3).clamp(-rate, rate);
}

#[inline(always)]
fn clamp8(x: i16) -> u8 {
    x.clamp(0, 255) as u8
}

#[inline(always)]
fn xorshift(mut x: u16) -> u16 {
    x ^= x << 7;
    x ^= x >> 9;
    x ^ x << 8
}

/// `sin` in Q7 (±128): enough for steering and probing, with 16-bit products on the C64.
#[inline(always)]
fn sin7(a: u16) -> i16 {
    (sin(a) >> 7) as i16
}

/// The hazard map's byte for texel (`u`, `v`): the highest obstacle top over its cell (texels), bit 7
/// set over the sea. Off the map: open sea.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
fn cell<R: Reu + ?Sized>(reu: &mut R, u: i16, v: i16) -> u8 {
    let (cu, cv) = ((u >> HAZARD_SHIFT) as u16, (v >> HAZARD_SHIFT) as u16);
    if cu >= HAZARD_CELLS || cv >= HAZARD_CELLS {
        return 0x80;
    }
    let mut b = [0u8];
    reu.read(HAZARD + (cv * HAZARD_CELLS + cu) as u32, &mut b);
    b[0]
}

/// An offset (1/16 texel) turned into a frame headed `yaw`: world to (across, along); `back`, the other
/// way. Q7 turns, 16-bit products.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.deck"))]
fn turn(x: i16, y: i16, yaw: u16, back: bool) -> (i16, i16) {
    let (s, c) = (sin7(yaw), sin7(yaw.wrapping_add(QUARTER_TURN)));
    let s = if back { -s } else { s };
    (((mul(x, c) - mul(y, s)) >> 7) as i16, ((mul(x, s) + mul(y, c)) >> 7) as i16)
}

impl Agent {
    pub const BYTES: usize = 48;

    /// On the home pad, facing the island's centre, engine off.
    pub const fn new() -> Self {
        Agent {
            t: 0,
            u: HOME.0 << 8,
            v: HOME.1 << 8,
            h: EYE,
            yaw: 0,
            vf: 0,
            roll: 0,
            pitch: 0,
            climb: 0,
            wu: HOME.0 as i16,
            wv: HOME.1 as i16,
            seed: 0xACE1,
            steer: 0,
            action: action::IDLE,
            grounded: 1,
            input: 0,
            mode: MODE_AUTO,
            throttle: 0,
            deck: 0,
            du: 0,
            dv: 0,
            dyaw: 0,
            pad: [0; 3],
        }
    }

    /// At boot: on the carrier's deck (placed there at the first `fly`). The C64 and the host's reference
    /// flight ([`at`]) both start so.
    pub const fn boot() -> Self {
        let mut a = Self::new();
        a.deck = PLACE_CARRIER;
        a
    }

    /// After going down: back at home, or on the carrier, in turn every 4096 ticks. Placed there (and
    /// started afresh) at the next `fly`, in the deck's overlay; until then still where it went down, so
    /// the frame drawn after it is the crash, not the new place. (One byte: resident on the C64.)
    #[inline(always)]
    pub fn reset(&mut self) {
        self.deck = if (self.t >> 12) & 1 == 0 { PLACE_CARRIER } else { PLACE_HOME };
    }

    /// Put a placed agent (`PLACE_HOME`, `PLACE_CARRIER`) at home, or on the carrier's deck: aft, facing
    /// the bow, still (at home if the carrier is gone).
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.deck"))]
    fn place<R: Reu + ?Sized>(&mut self, reu: &mut R) {
        let on_carrier = self.deck == PLACE_CARRIER;
        *self = Agent { t: self.t, ..Self::new() };
        if on_carrier {
            if let Some(d) = deck_of(&carrier_record(reu), self.t as u16) {
                (self.deck, self.du, self.dv, self.dyaw) = (ON_DECK, 0, -(4 << 4), 0);
                self.ride(&d);
            }
        }
    }

    /// On the deck: where the carrier has carried it.
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.deck"))]
    fn ride(&mut self, d: &Deck) {
        let (du, dv) = turn(self.du, self.dv, d.yaw, true);
        self.u = (d.u.wrapping_add(du as u16) as i32) << 4;
        self.v = (d.v.wrapping_add(dv as u16) as i32) << 4;
        self.yaw = d.yaw.wrapping_add((self.dyaw as u16) << 8);
        self.vf = 0;
        self.h = self.h.max(DECK_EYE + DECK_TOP);
    }

    /// Airspeed, 8.8 texels a tick.
    #[inline(always)]
    pub fn speed(&self) -> i16 {
        self.vf >> 6
    }

    /// Position, texels.
    #[inline(always)]
    pub fn at(&self) -> (i16, i16) {
        ((self.u >> 8) as i16, (self.v >> 8) as i16)
    }

    /// How far beyond the map's edge (texels; 0 over the map).
    #[inline(always)]
    pub fn off_map(&self) -> i16 {
        const FAR: i16 = crate::floor::MAP_TEXELS as i16 - 1;
        let (u, v) = self.at();
        (-u).max(u - FAR).max(-v).max(v - FAR).max(0)
    }

    /// The obstacle top `d` texels along `angle` (texels).
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    fn probe<R: Reu + ?Sized>(&self, reu: &mut R, angle: u16, d: i16) -> i16 {
        let (u, v) = self.at();
        let u = u + (sin7(angle) * d >> 7);
        let v = v + (sin7(angle.wrapping_add(QUARTER_TURN)) * d >> 7);
        (cell(reu, u, v) & 0x7F) as i16
    }

    /// The decision features.
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    pub fn features<R: Reu + ?Sized>(&mut self, reu: &mut R, f: &mut [u8; feature::BLOCK]) {
        use feature::*;
        *f = [0; BLOCK];
        let alt = (self.h >> 8) as i16;
        f[GROUND] = self.grounded;
        f[ROTOR] = (self.t >= LIFT) as u8;
        f[ALT] = clamp8(alt - CRUISE + 128);
        let (u, v) = self.at();
        let (dx, dy) = (self.wu - u, self.wv - v);
        // Along the heading, and to the left (the side a negative turn swings toward), in 16-texel
        // units times Q6 sines.
        let (s, c) = (sin7(self.yaw) >> 1, sin7(self.yaw.wrapping_add(QUARTER_TURN)) >> 1);
        let (ex, ey) = (dx >> 4, dy >> 4);
        let fwd = ex * s + ey * c;
        let lat = ey * s - ex * c;
        f[BEARING] = if fwd > 0 && lat.abs() < fwd >> 2 { 128 } else if lat > 0 { 64 } else { 192 };
        // The heading error as lat / (fwd + |lat|), ±32 with the waypoint abeam or behind (left
        // positive): both halved until the sum is under 64 (no divide; within a factor of 2).
        let err = if fwd > 0 {
            let (mut n, mut l) = (fwd + lat.abs(), lat);
            while n >= 64 {
                n >>= 1;
                l >>= 1;
            }
            l
        } else if lat > 0 {
            32
        } else {
            -32
        };
        self.steer = (-err * STEER_GAIN).clamp(-TURN_BANK_MAX, TURN_BANK_MAX);
        let (ax, ay) = (dx.abs(), dy.abs());
        f[DIST] = clamp8((ax.max(ay) + (ax.min(ay) >> 1)) >> 3);
        // The probes in feature order (the near pair both into HAZ_NEAR), each obstacle top − altitude.
        let mut max = 0;
        for &(slot, angle, d) in &PROBES {
            let haz = clamp8(self.probe(reu, self.yaw.wrapping_add(angle), d) - alt + 128);
            let slot = slot as usize & 15;
            f[slot] = f[slot].max(haz);
            max = max.max(haz);
        }
        f[HAZ_MAX] = max;
    }

    /// A new waypoint: the first of up to 16 random cells that is clear land (no mountain, no sea) at
    /// least 256 texels away (the old one kept if none is).
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    fn next_waypoint<R: Reu + ?Sized>(&mut self, reu: &mut R) {
        let (pu, pv) = self.at();
        for _ in 0..16 {
            self.seed = xorshift(self.seed);
            // Cells 4..60 each way (unsigned: one compare).
            let (cu, cv) = (self.seed & 63, self.seed >> 8 & 63);
            if cu.wrapping_sub(4) >= 56 || cv.wrapping_sub(4) >= 56 {
                continue;
            }
            let (u, v) = ((cu << HAZARD_SHIFT | 16) as i16, (cv << HAZARD_SHIFT | 16) as i16);
            if (u - pu).abs() + (v - pv).abs() >= 256 && cell(reu, u, v) == 0 {
                self.wu = u;
                self.wv = v;
                return;
            }
        }
    }

    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    fn think<R: Reu + ?Sized>(&mut self, reu: &mut R) {
        let mut f = [0u8; feature::BLOCK];
        self.features(reu, &mut f);
        if self.mode == MODE_PLAYER {
            // The player steers and sets the speed; the helicopter keeps its height: over what is ahead,
            // down to cruise, onto the ground when hovering with the stick held back, off it with the
            // stick forward once the rotor is up to speed.
            use feature::*;
            let blocked = f[HAZ_NEAR] >= 128 || f[HAZ_FAR] >= 128;
            let hover = self.throttle == 0 && self.vf < 256;
            self.action = if self.grounded != 0 {
                if self.t >= LIFT && self.input & input::UP != 0 { action::LIFT } else { action::IDLE }
            } else if blocked || f[ALT] < 126 && !(hover && self.input & input::DOWN != 0) {
                action::CLIMB
            } else if f[ALT] >= 131 || hover && self.input & input::DOWN != 0 {
                action::DESCEND
            } else {
                action::FORWARD
            };
            return;
        }
        let mut a = brain::walk(reu, AI_TREES, brain::PILOT, &f);
        if a == action::NEXT_WP {
            self.next_waypoint(reu);
            self.features(reu, &mut f);
            a = brain::walk(reu, AI_TREES, brain::PILOT, &f);
        }
        self.action = if a < action::NEXT_WP { a } else { action::FORWARD };
        if f[feature::BEARING] != 128 {
            // Flying on past an obstacle: wings level until the waypoint is ahead again.
            self.steer = 0;
        }
    }

    /// One tick: decide (every [`THINK`] ticks), then fly the action.
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    pub fn step<R: Reu + ?Sized>(&mut self, reu: &mut R) {
        let rec = carrier_record(reu);
        self.step_with(reu, &rec);
    }

    /// [`Agent::step`] with the carrier's record (`world_sim::carrier_record`, read once a `fly`: the war
    /// does not move while the agent catches up).
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    fn step_with<R: Reu + ?Sized>(&mut self, reu: &mut R, rec: &[u8; CARRIER_RECORD]) {
        // Too far out to sea: the autopilot takes over, for home (every tick: the fire button hands the
        // player the controls every frame, and beyond the limit it must not).
        if self.mode == MODE_PLAYER && self.off_map() > SEA_LIMIT {
            self.mode = MODE_AUTO;
            (self.wu, self.wv, self.throttle) = (HOME.0 as i16, HOME.1 as i16, 0);
        }
        if self.t % THINK == 0 {
            self.think(reu);
        }
        // The controls' targets.
        // (Tables of 16 and `& 15`: no bounds checks in the overlay.)
        let k = self.action as usize & 15;
        let (bank, pitch) = if self.mode == MODE_PLAYER {
            // The stick: the bank of a turn; the speed setting up and down, the nose down to hold it, up
            // to slow (a flare). On the ground, still.
            let i = self.input;
            if i & input::UP != 0 && self.throttle < THROTTLE_MAX {
                self.throttle += 1;
            }
            if i & input::DOWN != 0 && self.throttle > 0 {
                self.throttle -= 1;
            }
            let bank = if self.grounded != 0 { 0 } else if i & input::LEFT != 0 { -PLAYER_BANK } else if i & input::RIGHT != 0 { PLAYER_BANK } else { 0 };
            let want = -((self.throttle as i16) << 5);
            (bank, if self.grounded != 0 { 0 } else if self.vf > (-want << 2) + 512 { FLARE } else { want })
        } else {
            let bank = match self.action {
                action::TURN_L => -self.steer.abs().max(TURN_BANK_MIN),
                action::TURN_R => self.steer.abs().max(TURN_BANK_MIN),
                action::FORWARD => self.steer,
                _ => TARGET_BANK[k],
            };
            (bank, TARGET_PITCH[k])
        };
        // The attitude follows, lagged and rate-limited; the bank turns, the pitch drives the speed.
        follow(&mut self.roll, bank, ROLL_RATE);
        follow(&mut self.pitch, pitch, PITCH_RATE);
        ease(&mut self.climb, TARGET_CLIMB[k], EASE_CLIMB);
        self.yaw = self.yaw.wrapping_add((self.roll >> TURN_SHIFT) as u16);
        // Thrust along the tilt against drag: vf settles at −4 × pitch (32-tick time constant).
        self.vf = (self.vf + ((-self.pitch - (self.vf >> 2)) >> 3)).max(0);
        let speed = self.speed();
        self.u += (sin7(self.yaw) * speed >> 7) as i32;
        self.v += (sin7(self.yaw.wrapping_add(QUARTER_TURN)) * speed >> 7) as i32;
        self.h += self.climb as i32;
        self.settle(reu, rec);
        self.t += 1;
    }

    /// The floor: the carrier's deck when over it (it matters when low, or riding it), else the ground.
    /// Settling onto it: the deck (riding it from now), the sea (ditched) or the ground. (In the AI's
    /// overlay: the deck's, in at the same time, is also the war's move phase's.)
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    fn settle<R: Reu + ?Sized>(&mut self, reu: &mut R, rec: &[u8; CARRIER_RECORD]) {
        let deck = if self.deck == ON_DECK || self.h < DECK_EYE + DECK_TOP + 512 { deck_of(rec, self.t as u16) } else { None };
        let mut over = None;
        if let Some(d) = &deck {
            let (du, dv) = (((self.u >> 4) as u16).wrapping_sub(d.u) as i16, ((self.v >> 4) as u16).wrapping_sub(d.v) as i16);
            if du.unsigned_abs() < 256 && dv.unsigned_abs() < 256 {
                let (x, y) = turn(du, dv, d.yaw, false);
                if x.abs() <= DECK_HALF.0 && y.abs() <= DECK_HALF.1 {
                    over = Some((x, y, d.yaw));
                }
            }
        }
        let floor = if over.is_some() { DECK_EYE + DECK_TOP } else { EYE };
        if self.h < floor {
            self.h = floor;
            self.climb = self.climb.max(0);
            if self.mode == MODE_PLAYER && self.action == action::DESCEND && self.grounded == 0 {
                self.grounded = 1;
                self.vf = 0;
                if let Some((x, y, yaw)) = over {
                    (self.deck, self.du, self.dv, self.dyaw) = (ON_DECK, x, y, (self.yaw.wrapping_sub(yaw) >> 8) as u8);
                } else if cell(reu, (self.u >> 8) as i16, (self.v >> 8) as i16) & 0x80 != 0 {
                    self.deck = DITCHED;
                }
            }
        }
        if self.h >= LIFTED {
            self.grounded = 0;
        }
        if self.deck == ON_DECK {
            match deck {
                Some(d) if self.grounded != 0 => self.ride(&d),
                // Lifted off; or the carrier went down under it (into the sea).
                Some(_) => self.deck = 0,
                None => self.deck = if self.grounded != 0 { DITCHED } else { 0 },
            }
        }
    }

    /// Catch up with `clock` (the C64's entry to the AI overlay).
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    pub fn fly<R: Reu + ?Sized>(&mut self, reu: &mut R, clock: u32) {
        if self.deck >= PLACE_CARRIER {
            self.place(reu);
        }
        let rec = carrier_record(reu);
        while self.t < clock {
            self.step_with(reu, &rec);
        }
    }

    /// The floor camera and its yaw (for the objects): the helicopter's attitude. (Resident on the C64: the main loop calls it after the overlay.)
    #[inline(never)]
    pub fn camera(&self) -> (FloorCam, u16) {
        let m = Matrix14::orientation(self.yaw, self.pitch as u16, self.roll as u16);
        (FloorCam { u: self.u, v: self.v, h: self.h as u16, r: m.m }, self.yaw)
    }

    /// The inverse of [`Agent::to_bytes`].
    pub fn from_bytes(b: &[u8]) -> Self {
        let w = |k: usize| u16::from_le_bytes([b[16 + 2 * k], b[17 + 2 * k]]);
        let d = |k: usize| i32::from_le_bytes([b[k], b[k + 1], b[k + 2], b[k + 3]]);
        Agent {
            t: d(0) as u32,
            u: d(4),
            v: d(8),
            h: d(12),
            yaw: w(0),
            vf: w(1) as i16,
            roll: w(2) as i16,
            pitch: w(3) as i16,
            climb: w(4) as i16,
            wu: w(5) as i16,
            wv: w(6) as i16,
            seed: w(7),
            steer: w(8) as i16,
            action: b[34],
            grounded: b[35],
            input: b[36],
            mode: b[37],
            throttle: b[38],
            deck: b[39],
            du: w(12) as i16,
            dv: w(13) as i16,
            dyaw: b[44],
            pad: [b[45], b[46], b[47]],
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::BYTES] {
        let mut b = [0u8; Self::BYTES];
        b[0..4].copy_from_slice(&self.t.to_le_bytes());
        b[4..8].copy_from_slice(&self.u.to_le_bytes());
        b[8..12].copy_from_slice(&self.v.to_le_bytes());
        b[12..16].copy_from_slice(&self.h.to_le_bytes());
        for (k, w) in [self.yaw, self.vf as u16, self.roll as u16, self.pitch as u16, self.climb as u16, self.wu as u16, self.wv as u16, self.seed, self.steer as u16]
            .into_iter()
            .enumerate()
        {
            b[16 + 2 * k..18 + 2 * k].copy_from_slice(&w.to_le_bytes());
        }
        b[34] = self.action;
        b[35] = self.grounded;
        b[36] = self.input;
        b[37] = self.mode;
        b[38] = self.throttle;
        b[39] = self.deck;
        b[40..42].copy_from_slice(&self.du.to_le_bytes());
        b[42..44].copy_from_slice(&self.dv.to_le_bytes());
        b[44] = self.dyaw;
        b[45..48].copy_from_slice(&self.pad);
        b
    }
}

/// The agent after `t` ticks from boot, on a baked image (host: the reference the C64 must match).
pub fn at(mut reu: &[u8], t: u32) -> Agent {
    let mut a = Agent::boot();
    a.fly(&mut reu, t);
    a
}
