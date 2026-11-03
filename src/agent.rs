//! The attract-mode helicopter: a small flight sim stepped once per NTSC frame of the clock, with the
//! same bits on the host and the C64, steered by the baked decision tree (`brain`, planned by
//! `goap`). The clock starts with the rotor sound (`asm/audio.S`). The helicopter sits on the home pad
//! until the rotor is at full power (the sound's 1:20), lifts off, and then roams: it flies to seeded
//! random waypoints over the island, climbing over or turning round the mountains its probes see in the
//! baked hazard map (`format::HAZARD`).
//!
//! On the C64 the code runs from the AI overlay (`.text.ai`, DMA'd to `layout::WORK` each frame).

use crate::brain::{self, Reu};
use crate::camera::{Matrix14, QUARTER_TURN, sin};
use crate::floor::FloorCam;
use crate::format::{AI_TREES, HAZARD, HAZARD_CELLS, HAZARD_SHIFT};

/// On the pad until the rotor is at full power: the sound's 1:20 (`digital::ROTOR_LOOP_START`).
pub const LIFT: u32 = 80 * 60;
/// Eye height on the pad: half a texel (8.8).
pub const EYE: i32 = 128;
/// The home pad (`world::PADS[0]`), texels.
pub const HOME: (i32, i32) = (1024, 760);
/// Cruise altitude, texels (over the 6-texel tower; mountains are 22-34).
pub const CRUISE: i16 = 16;
/// Off the pad (`grounded` clears) at this height, 8.8: over the tower before moving.
pub const LIFTED: i32 = 10 * 256;
/// Ticks between decisions.
pub const THINK: u32 = 8;
/// Cruise speed (8.8 texels a tick).
const SPEED: i16 = 128;
/// Each action's targets, indexed by `action`: speed (8.8 texels a tick), turn rate (binary angle a
/// tick), climb rate (8.8 texels a tick).
const TARGET_SPEED: [i16; action::COUNT as usize] = [0, 0, SPEED / 2, SPEED, SPEED, SPEED, SPEED * 3 / 4, SPEED * 3 / 4, SPEED, SPEED];
const TARGET_TURN: [i16; action::COUNT as usize] = [0, 0, 0, 0, -40, 40, -96, 96, 0, 0];
const TARGET_CLIMB: [i16; action::COUNT as usize] = [0, 24, 40, 0, 0, 0, 0, 0, -40, 0];
/// How fast speed, turn and climb follow their targets, per tick.
const EASE_SPEED: i16 = 1;
const EASE_TURN: i16 = 3;
const EASE_CLIMB: i16 = 1;
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
    pub speed: i16,
    pub turn: i16,
    pub climb: i16,
    /// The waypoint, texels.
    pub wu: i16,
    pub wv: i16,
    pub seed: u16,
    pub action: u8,
    pub grounded: u8,
}

impl Default for Agent {
    fn default() -> Self {
        Self::new()
    }
}

#[inline(always)]
fn ease(x: &mut i16, target: i16, rate: i16) {
    *x = if *x < target { (*x + rate).min(target) } else { (*x - rate).max(target) };
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

impl Agent {
    pub const BYTES: usize = 32;

    /// On the home pad, facing the island's centre, engine off.
    pub const fn new() -> Self {
        Agent {
            t: 0,
            u: HOME.0 << 8,
            v: HOME.1 << 8,
            h: EYE,
            yaw: 0,
            speed: 0,
            turn: 0,
            climb: 0,
            wu: HOME.0 as i16,
            wv: HOME.1 as i16,
            seed: 0xACE1,
            action: action::IDLE,
            grounded: 1,
        }
    }

    /// Position, texels.
    #[inline(always)]
    fn at(&self) -> (i16, i16) {
        ((self.u >> 8) as i16, (self.v >> 8) as i16)
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
    pub fn features<R: Reu + ?Sized>(&self, reu: &mut R, f: &mut [u8; feature::COUNT]) {
        use feature::*;
        *f = [0; COUNT];
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
        let (ax, ay) = (dx.abs(), dy.abs());
        f[DIST] = clamp8((ax.max(ay) + (ax.min(ay) >> 1)) >> 3);
        // The probes in feature order (the near pair both into HAZ_NEAR), each obstacle top − altitude.
        let mut max = 0;
        for &(slot, angle, d) in &PROBES {
            let haz = clamp8(self.probe(reu, self.yaw.wrapping_add(angle), d) - alt + 128);
            let slot = slot as usize;
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
            let (cu, cv) = ((self.seed & 63) as i16, (self.seed >> 8 & 63) as i16);
            if !(4..60).contains(&cu) || !(4..60).contains(&cv) {
                continue;
            }
            let (u, v) = (cu << HAZARD_SHIFT | 16, cv << HAZARD_SHIFT | 16);
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
        let mut f = [0u8; feature::COUNT];
        self.features(reu, &mut f);
        let mut a = brain::walk(reu, AI_TREES, brain::PILOT, &f);
        if a == action::NEXT_WP {
            self.next_waypoint(reu);
            self.features(reu, &mut f);
            a = brain::walk(reu, AI_TREES, brain::PILOT, &f);
        }
        self.action = if a < action::NEXT_WP { a } else { action::FORWARD };
    }

    /// One tick: decide (every [`THINK`] ticks), then fly the action.
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    pub fn step<R: Reu + ?Sized>(&mut self, reu: &mut R) {
        if self.t % THINK == 0 {
            self.think(reu);
        }
        let k = self.action as usize;
        ease(&mut self.speed, TARGET_SPEED[k], EASE_SPEED);
        ease(&mut self.turn, TARGET_TURN[k], EASE_TURN);
        ease(&mut self.climb, TARGET_CLIMB[k], EASE_CLIMB);
        self.yaw = self.yaw.wrapping_add(self.turn as u16);
        self.u += (sin7(self.yaw) * self.speed >> 7) as i32;
        self.v += (sin7(self.yaw.wrapping_add(QUARTER_TURN)) * self.speed >> 7) as i32;
        self.h += self.climb as i32;
        if self.h < EYE {
            self.h = EYE;
            self.climb = self.climb.max(0);
        }
        if self.h >= LIFTED {
            self.grounded = 0;
        }
        self.t += 1;
    }

    /// Catch up with `clock` (the C64's entry to the AI overlay).
    #[inline(never)]
    #[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.ai"))]
    pub fn fly<R: Reu + ?Sized>(&mut self, reu: &mut R, clock: u32) {
        while self.t < clock {
            self.step(reu);
        }
    }

    /// The floor camera and its yaw (for the objects): banked with the turn, nose down with the speed,
    /// up with the climb. (Resident on the C64: the main loop calls it after the overlay.)
    #[inline(never)]
    pub fn camera(&self) -> (FloorCam, u16) {
        let roll = self.turn * 70;
        let pitch = self.climb * 20 - self.speed * 14;
        let m = Matrix14::orientation(self.yaw, pitch as u16, roll as u16);
        (FloorCam { u: self.u, v: self.v, h: self.h as u16, r: m.m }, self.yaw)
    }

    pub fn to_bytes(&self) -> [u8; Self::BYTES] {
        let mut b = [0u8; Self::BYTES];
        b[0..4].copy_from_slice(&self.t.to_le_bytes());
        b[4..8].copy_from_slice(&self.u.to_le_bytes());
        b[8..12].copy_from_slice(&self.v.to_le_bytes());
        b[12..16].copy_from_slice(&self.h.to_le_bytes());
        for (k, w) in [self.yaw, self.speed as u16, self.turn as u16, self.climb as u16, self.wu as u16, self.wv as u16, self.seed]
            .into_iter()
            .enumerate()
        {
            b[16 + 2 * k..18 + 2 * k].copy_from_slice(&w.to_le_bytes());
        }
        b[30] = self.action;
        b[31] = self.grounded;
        b
    }
}

/// The agent after `t` ticks from boot, on a baked image (host: the reference the C64 must match).
pub fn at(mut reu: &[u8], t: u32) -> Agent {
    let mut a = Agent::new();
    while a.t < t {
        a.step(&mut reu);
    }
    a
}
