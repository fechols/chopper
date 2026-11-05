//! Sound effects on the SIDs, placed round the player (a cheap HRTF).
//!
//! The C64U's two UltiSIDs, each split in two (`config/chopper.cfg`: range split on A5), are four SIDs:
//! $D400 and $D420 hard left, $D440 and $D460 hard right. They make two stereo pairs ([`PAIRS`]): pair `p`
//! is the left chip at `$D400 + $20 p` and the right one at `$D440 + $20 p`. A sound takes a whole pair:
//! the same three voices on both chips (a [`patch`]), each chip with its own master volume (its ear's
//! gain) and its own low-pass cutoff (its ear's muffling). So the filter colours that one sound only.
//!
//! - **Where it is** ([`place`]): its bearing from the nose pans it (a source on the right is loud in the
//!   right chip, quieter in the left); behind the player the cutoffs close (muffled), and the far ear's
//!   more (the head's shadow); distance lowers the gain and dulls it a little more (the air).
//! - **What sounds** ([`sounds`], once a frame after the effects pass): the frame's new effects (their
//!   slots in `combat::FX` begun since the last frame: impacts, splashes, explosions, wreck fires, the
//!   war's muzzle flashes, the far ones quieter and duller), the player's new tracers and missiles, a hit
//!   on the player (`player::FLASH` rising) and the seeker's lock (`hud::LOCK_STATE` reaching
//!   `LOCK_TICKS`).
//! - **Who gets a pair:** a free one, else the one with the least left of its sound (its score, its
//!   priority × gain, running down over its length), if the new one scores at least that. The gun keeps
//!   to one pair (its bursts onto the pair already firing, which nothing else takes from it), so it is
//!   heard with whatever else is going on.
//! - **Pitch sweeps** run at 60 Hz in the raster IRQ (`asm/hud.S`, from [`SWEEP`]); the frame (13-20 a
//!   second) triggers sounds, releases their gates, sweeps their cutoffs and places them again (the
//!   player turning pans what is playing).
//! - **The ears' lag** ([`place`]'s last): a sound is gated on the near ear's chip first and on the far
//!   one that many raster lines later ([`start`]), as it reaches the far ear later round a head. An
//!   engine's lag then follows the player turning ([`drive`]).
//! - **Engines** ([`engines`]): the two nearest vehicles that have one (`squad::MEMBERS`) take the pairs
//!   no effect is using, held until they drop out of the two; any effect takes a pair from an engine
//!   (a free pair first), and the engine is back once it is done. Their pitch rises while they move.
//!
//! Three overlays: the one-shots ([`sounds`], `.text.sfx`), then the engines ([`motors`], `.text.sfxeng`)
//! in turn in the first window, each with its own patches; placing and starting a sound ([`place`],
//! [`set`], [`start`], fed the patch by its caller) in a library in the second (`.text.sfxlib`, at $A000,
//! fetched before both).
//!
//! The state is at [`STATE`] ($FF00, below the vectors): nothing draws from it, so it is not in the
//! snapshot gate. Cleared at boot (RAM outlives a reset), it is set up on the first frame (its [`MAGIC`]
//! byte), the chips silenced then. After a stall of [`STALL`] ticks or more nothing new sounds that frame
//! (every recent effect and projectile would read as new).

use crate::agent::Agent;
use crate::hw::Hw;

/// The pairs of chips: two sounds at once.
pub const PAIRS: u16 = 2;
/// The left chip of pair `p` is at `LEFT + PAIR_STEP p`, the right one `RIGHT + PAIR_STEP p`.
pub const LEFT: u16 = 0xD400;
pub const RIGHT: u16 = 0xD440;
pub const PAIR_STEP: u16 = 0x20;

/// The state (`$FF00`): [`MAGIC`], the tick of the last frame (u16), the hit flash and the lock ticks
/// then; then a [`PAIR`]-byte entry per pair (`pair`).
pub const STATE: u16 = 0xFF00;
pub const MAGIC: u8 = 0xA5;
pub mod state {
    pub const MAGIC: u16 = 0;
    pub const LAST: u16 = 1;
    pub const FLASH: u16 = 3;
    pub const LOCK: u16 = 4;
    pub const PAIRS: u16 = 5;
}
pub const PAIR: u16 = 13;
/// A pair's entry: its patch + 1 (0: free), flags, where (1/16 texel, u16 × 2), the tick it began (u16:
/// an engine's, its distance last frame instead, 1/16 texel: nothing reads an engine's start), its score
/// then (an engine's: its revs, 0..=[`REVS`]); an engine's vehicle (u16), the ears' lag now (i8,
/// [`place`]'s units: the left ear's behind the right's) and its Doppler shift (i8, [`DOP_MAX`]).
pub mod pair {
    pub const PATCH: u16 = 0;
    pub const FLAGS: u16 = 1;
    pub const U: u16 = 2;
    pub const V: u16 = 4;
    pub const START: u16 = 6;
    pub const SCORE: u16 = 8;
    pub const VID: u16 = 9;
    pub const ITD: u16 = 11;
    pub const DOP: u16 = 12;
}
/// Flags: its gates released; the player's own (centred, wherever the player flies); far off. An
/// engine's: its left chip running slow since the last frame, its right one ([`drive`]).
pub const RELEASED: u8 = 1;
pub const OWN: u8 = 2;
pub const FAR: u8 = 4;
pub const SLOW_L: u8 = 8;
pub const SLOW_R: u8 = 16;

/// The IRQ's pitch sweeps (`asm/hud.S` has these addresses): per pair, the video frames left, then the
/// three voices' frequency high bytes, then their deltas (i8) each frame. Written with the steps 0
/// first and set last, so the IRQ never sees half a sweep.
pub const SWEEP: u16 = 0xFF20;
pub const SWEEP_PAIR: u16 = 7;
const _: () = assert!(STATE + state::PAIRS + PAIRS * PAIR <= SWEEP && SWEEP + PAIRS * SWEEP_PAIR <= 0xFF40);

/// The sounds.
pub mod patch {
    pub const GUN: u8 = 0;
    pub const IMPACT: u8 = 1;
    pub const SPLASH: u8 = 2;
    pub const EXPLOSION: u8 = 3;
    pub const CRACKLE: u8 = 4;
    pub const MUZZLE: u8 = 5;
    pub const LAUNCH: u8 = 6;
    pub const HIT: u8 = 7;
    pub const LOCK: u8 = 8;
    /// The engines (from here on: they loop until stopped, [`super::engines`]).
    pub const TRUCK: u8 = 9;
    pub const TANK: u8 = 10;
    pub const BOAT: u8 = 11;
    pub const SHIP: u8 = 12;
    pub const ROTOR: u8 = 13;
    pub const DIESEL: u8 = 14;
    pub const ENGINE: u8 = TRUCK;
    /// The one-shots ([`super::PATCHES`]), then the engines ([`super::ENGINES`]).
    pub const ONE_SHOTS: usize = ENGINE as usize;
    pub const COUNT: usize = 15;
}

/// A pair playing `id` (its patch + 1) plays an engine.
#[inline(always)]
fn is_engine(id: u8) -> bool {
    id > patch::ENGINE
}

/// Each kind's engine (`actors::kind` order), or none (`SILENT`): the sites, the SAMs on their launchers and
/// the projectiles make none.
const SILENT: u8 = 0xFF;
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxeng"))]
static ENGINE_OF: [u8; crate::actors::kind::COUNT as usize] = {
    use patch::*;
    //  truck  apc    tank  aa    sam     radar   gunboat msl boat gunship depot   bunker  carrier dest  tracer  missile
    [TRUCK, TRUCK, TANK, TANK, SILENT, SILENT, BOAT, BOAT, ROTOR, SILENT, SILENT, DIESEL, SHIP, SILENT, SILENT]
};

/// An engine's revs at most: they drift toward this while it moves and toward 0 while it stands, a tick's
/// worth a tick (about 4 s from one to the other), raising its pitch by its patch's
/// [`field::REV_DEPTH`]. A restart begins halfway ([`REVS_START`]).
pub const REVS: u8 = 255;
pub const REVS_START: u8 = 128;

/// An engine's Doppler shift at most, either way: its pitch raised (closing) or lowered (opening) by
/// `DOP / 256`, about an eighth. Its target is [`DOP_GAIN`] × its closing speed (1/16 texel a tick): the
/// game's speeds are far beyond true Doppler (a gunship at 180 m/s), so this is by ear.
pub const DOP_MAX: i8 = 32;
pub const DOP_GAIN: i16 = 2;

/// A patch: three voices of [`VOICE`] bytes (frequency high byte, pulse width high nibble, waveform
/// (the control register less its gate), attack/decay, sustain/release, the pitch sweep's delta a video
/// frame (i8)), then the filter's resonance and routing ($D417), its mode ($D418's high nibble), the
/// cutoff (the high byte, open: [`place`] closes it), its sweep a tick (i8), the pitch sweep's length
/// (video frames), the gate's (ticks: then the release), the sound's (ticks: then the pair is free) and
/// its priority.
pub const VOICE: usize = 6;
pub const PATCH: usize = 3 * VOICE + 8;
pub mod field {
    pub const ROUTE: usize = 18;
    pub const MODE: usize = 19;
    pub const CUTOFF: usize = 20;
    pub const CUT_SWEEP: usize = 21;
    pub const STEPS: usize = 22;
    pub const GATE: usize = 23;
    /// An engine's (its gate is held): how far its revs raise its pitch, `REV_DEPTH × revs / 4096` (0:
    /// never; 4: a quarter at most).
    pub const REV_DEPTH: usize = GATE;
    pub const LENGTH: usize = 24;
    pub const PRIORITY: usize = 25;
}

/// The waveforms (the control register's high nibble), ring modulation.
const TRI: u8 = 0x10;
const SAW: u8 = 0x20;
const PULSE: u8 = 0x40;
const NOISE: u8 = 0x80;
const RING: u8 = 0x04;
/// The filter's low-pass mode (with $D418's volume nibble).
const LP: u8 = 0x10;

/// The patches (by ear: tune here). Voice 2 rings off voice 1, voice 3 off voice 2.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub static PATCHES: [[u8; PATCH]; patch::ONE_SHOTS] = [
    // GUN: a noise crack, a falling square click, a low thump.
    [0x38, 0, NOISE, 0x04, 0x00, 0, 0x0A, 8, PULSE, 0x03, 0x00, -2i8 as u8, 0x03, 0, TRI, 0x05, 0x00, 0, 0x27, LP, 0xD0, 0, 4, 2, 10, 3],
    // IMPACT: a noise burst, a pitch drop, a ring-mod click.
    [0x24, 0, NOISE, 0x06, 0x00, -1i8 as u8, 0x12, 0, TRI | RING, 0x04, 0x00, -2i8 as u8, 0x05, 0, TRI, 0x06, 0x00, 0, 0x37, LP, 0xA0, -2i8 as u8, 6, 2, 18, 4],
    // SPLASH: filtered noise opening then closing, a slow release.
    [0x30, 0, NOISE, 0x19, 0x09, 0, 0x14, 0, NOISE, 0x29, 0x08, 0, 0x02, 0, TRI, 0x07, 0x00, 0, 0x23, LP, 0x50, -1i8 as u8, 0, 4, 45, 3],
    // EXPLOSION: a noise rumble, a falling triangle boom, a deep noise tail; the cutoff closing.
    [0x0C, 0, NOISE, 0x0A, 0x0A, 0, 0x0A, 0, TRI, 0x09, 0x09, -1i8 as u8, 0x03, 0, NOISE, 0x0B, 0x0B, 0, 0x47, LP, 0xC0, -2i8 as u8, 8, 6, 110, 9],
    // CRACKLE: a wreck catching fire: two short noise snaps.
    [0x60, 0, NOISE, 0x03, 0x00, 0, 0x30, 0, NOISE, 0x05, 0x00, 0, 0x00, 0, 0, 0x00, 0x00, 0, 0x03, LP, 0x80, 0, 0, 1, 10, 1],
    // MUZZLE: the war's gunfire: a sharp noise pop over a falling square.
    [0x2C, 0, NOISE, 0x04, 0x00, 0, 0x07, 8, PULSE, 0x03, 0x00, -1i8 as u8, 0x00, 0, 0, 0x00, 0x00, 0, 0x03, LP, 0xB0, 0, 4, 1, 9, 2],
    // LAUNCH: a rising whoosh (noise and a saw climbing), the filter opening.
    [0x10, 0, NOISE, 0x2A, 0x08, 2, 0x04, 0, SAW, 0x29, 0x08, 1, 0x08, 0, NOISE, 0x09, 0x00, 0, 0x37, LP, 0x40, 6, 30, 8, 60, 7],
    // HIT: a metallic clang: a square ringing a triangle, a noise scrape; unfiltered.
    [0x18, 4, PULSE, 0x08, 0x00, 0, 0x23, 0, TRI | RING, 0x09, 0x00, 0, 0x50, 0, NOISE, 0x05, 0x00, 0, 0x00, LP, 0xFF, 0, 0, 3, 40, 8],
    // LOCK: a two-tone beep, unfiltered.
    [0x40, 8, PULSE, 0x00, 0xA4, 0, 0x50, 8, PULSE, 0x00, 0xA4, 0, 0x00, 0, 0, 0x00, 0x00, 0, 0x00, LP, 0xFF, 0, 0, 10, 24, 6],
];

/// The engines' patches (from [`patch::ENGINE`]; in their own overlay, `.text.sfxeng`).
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxeng"))]
pub static ENGINES: [[u8; PATCH]; patch::COUNT - patch::ONE_SHOTS] = [
    // The engines: slow attacks (they fade in), held until stopped (length 255), no sweeps: each voice's
    // last byte is its frequency's low byte, the gate's byte the revs' depth ([`field::REV_DEPTH`]).
    // TRUCK: a low square buzz over a noise grumble.
    [0x05, 8, PULSE, 0x80, 0xA6, 0, 0x02, 0, NOISE, 0x80, 0x76, 0, 0x00, 0, 0, 0x00, 0x00, 0, 0x13, LP, 0x60, 0, 0, 4, 255, 1],
    // TANK: a deep narrow pulse, a triangle ringing off it (the tracks' clank), a noise rumble.
    [0x03, 4, PULSE, 0x90, 0xB6, 0, 0x07, 0, TRI | RING, 0x90, 0x86, 0, 0x01, 0, NOISE, 0x90, 0x96, 0, 0x27, LP, 0x50, 0, 0, 2, 255, 1],
    // BOAT: an outboard's saw buzz, a triangle under it, the spray's hiss.
    [0x06, 0, SAW, 0x80, 0x96, 0, 0x0C, 0, TRI, 0x80, 0x86, 0, 0x08, 0, NOISE, 0x80, 0x46, 0, 0x15, LP, 0x70, 0, 0, 4, 255, 1],
    // SHIP: a ship's deep triangle drone, a thin low pulse, the wash.
    [0x02, 0, TRI, 0xA0, 0xC8, 0, 0x04, 2, PULSE, 0xA0, 0x88, 0, 0x01, 0, NOISE, 0xA0, 0x98, 0, 0x37, LP, 0x40, 0, 0, 1, 255, 1],
    // ROTOR: a sub-audio narrow pulse (the blades' thumps), a triangle ringing off it, the turbine's hiss.
    [0x01, 2, PULSE, 0x80, 0xA6, 0, 0x0E, 0, TRI | RING, 0x80, 0x86, 0, 0x06, 0, NOISE, 0x80, 0x46, 0, 0x17, LP, 0x60, 0, 0, 2, 255, 1],
    // DIESEL: the carrier's giant diesel, never revving: a rumble, no edges to pop: a 30 Hz triangle and a
    // 36 Hz saw beating against it (a soft throb about 6 times a second, the cylinders), and low noise
    // (the rumble's body), all through a deep low-pass with little resonance (nothing rings).
    [0x02, 0, TRI, 0xA0, 0xC8, 0x00, 0x02, 0, SAW, 0xA0, 0xA8, 0x60, 0x03, 0, NOISE, 0xA0, 0xB8, 0x00, 0x17, LP, 0x18, 0, 0, 0, 255, 1],
];
// An engine sweeps nothing: its voices' last bytes are frequency low bytes, not the IRQ's deltas (its
// sweep never runs), and its start tick is not kept (no cutoff sweep reads its age).
const _: () = {
    let mut k = 0;
    while k < ENGINES.len() {
        assert!(ENGINES[k][field::STEPS] == 0 && ENGINES[k][field::CUT_SWEEP] == 0);
        k += 1;
    }
};

/// A sound's place for each ear: (left volume, right volume, left cutoff factor, right one, the left
/// ear's lag behind the right): volumes 0..=15, factors /256 of the patch's cutoff, the lag in units of
/// a 256th of a tick (65 µs, about a raster line), -[`ITD_MAX`]..=[`ITD_MAX`] (the sound reaching the
/// far ear later round the head: up to about 0.65 ms). `dx`, `dy`: from the player (texels; east,
/// north), `yaw` the player's heading (binary angle, 0 north, clockwise). `far`: the battle on the
/// horizon.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
pub fn place(dx: i16, dy: i16, yaw: u16, far: bool) -> (u8, u8, u8, u8, i8) {
    // Gain 0..=16: full within 16 texels, then falling as 1/distance.
    let (ax, ay) = (dx.unsigned_abs(), dy.unsigned_abs());
    let d = ax.max(ay) + (ax.min(ay) >> 1);
    let mut g = (384 / (d.min(4000) + 8)).min(16) as u8;
    if far {
        g >>= 1;
    }
    // Its bearing from the nose: `side` -16 (left) ..= 16 (right), `ahead` -16 (behind) ..= 16.
    let (side, ahead) = if d == 0 {
        (0, 16)
    } else {
        // (The sine table's step, Q14 in an i16: ±16384, so / 1024 is ±16.)
        let rel = ((crate::world_sim::heading_inline(dx, dy) as u16) << 8).wrapping_sub(yaw);
        // (Toward 0: the two sides alike.)
        ((crate::camera::sin(rel) as i16) / 1024, (crate::camera::sin(rel.wrapping_add(0x4000)) as i16) / 1024)
    };
    // Each ear: down to a quarter on the far side (`near`, `far` the ears' shares of the side, 0..=16).
    let (r, l) = (side.max(0) as u8, (-side).max(0) as u8);
    let vol = |s: u8| ((g as u16 * (16 - (s * 3 >> 2)) as u16 + 8) >> 4).min(15) as u8;
    // Muffled behind (to about 3/8 open), the far ear more (the head's shadow), and with distance.
    let air = (d >> 2).min(60) as u8 + if far { 40 } else { 0 };
    let open = 255u8.saturating_sub(((16 - ahead) as u8) * 5).saturating_sub(air);
    let cut = |s: u8| open.saturating_sub(s * 2).max(16);
    // (Rounded toward 0: the two sides alike.)
    (vol(r), vol(l), cut(r), cut(l), ((side * 5 + if side < 0 { 7 } else { 0 }) >> 3) as i8)
}

/// The ears' lag at most ([`place`]'s units): a source abeam.
pub const ITD_MAX: i8 = 10;

#[inline(always)]
fn rd<H: Hw + ?Sized>(hw: &mut H, a: u16) -> u16 {
    hw.peek(a) as u16 | (hw.peek(a + 1) as u16) << 8
}

#[inline(always)]
fn wr<H: Hw + ?Sized>(hw: &mut H, a: u16, w: u16) {
    hw.poke(a, w as u8);
    hw.poke(a + 1, (w >> 8) as u8);
}

/// The frame's sounds, both halves (the host's: on the C64 the two are in two overlays, fetched in turn).
#[cfg(not(target_arch = "mos"))]
pub fn update<H: Hw + ?Sized>(hw: &mut H, a: &Agent) {
    let dt = sounds(hw, a);
    motors(hw, a, dt);
}

/// The frame's sounds on the C64: the one-shots' overlay fetched and run ([`sounds`]), then the engines'
/// over it ([`motors`]). This library (the second window, fetched by the caller) stays meanwhile.
#[cfg(target_arch = "mos")]
#[inline(never)]
#[unsafe(link_section = ".text.sfxlib")]
pub fn frame<H: Hw + ?Sized>(hw: &mut H, a: &Agent) {
    unsafe extern "C" {
        static __sfxtext_size: u8;
        static __sfxengtext_size: u8;
    }
    use crate::{format, hw::fetch, layout::WORK};
    fetch(hw, format::OVERLAY_SFX, WORK, unsafe { &raw const __sfxtext_size } as u16);
    let dt = sounds(hw, a);
    fetch(hw, format::OVERLAY_SFXENG, WORK, unsafe { &raw const __sfxengtext_size } as u16);
    motors(hw, a, dt);
}

/// The frame's one-shots: the new events onto the pairs, each one-shot playing released, freed or placed
/// again (see the module). The ticks since the last frame, for [`motors`] next.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub fn sounds<H: Hw + ?Sized>(hw: &mut H, a: &Agent) -> u16 {
    use crate::combat::{PLAYER, player};
    let now = a.t as u16;
    if hw.peek(STATE + state::MAGIC) != MAGIC {
        init(hw, now);
    }
    let dt = now.wrapping_sub(rd(hw, STATE + state::LAST)).min(STALL);
    wr(hw, STATE + state::LAST, now);
    let flash = hw.peek(PLAYER + player::FLASH);
    let lock = hw.peek(crate::hud::LOCK_STATE + 2);
    if dt < STALL {
        events(hw, a, now, dt, flash, lock);
    }
    hw.poke(STATE + state::FLASH, flash);
    hw.poke(STATE + state::LOCK, lock);
    for p in 0..PAIRS {
        let s = STATE + state::PAIRS + p * PAIR;
        let id = hw.peek(s + pair::PATCH);
        // (An engine runs until stopped: `motors`.)
        if id == 0 || is_engine(id) {
            continue;
        }
        let t = &PATCHES[(id - 1) as usize % patch::ONE_SHOTS];
        let age = now.wrapping_sub(rd(hw, s + pair::START));
        if age >= t[field::LENGTH] as u16 {
            hw.poke(s + pair::PATCH, 0);
            continue;
        }
        let flags = hw.peek(s + pair::FLAGS);
        if age >= t[field::GATE] as u16 && flags & RELEASED == 0 {
            hw.poke(s + pair::FLAGS, flags | RELEASED);
            release(hw, p, t);
        }
        set(hw, a, p, t, now);
    }
    dt
}

/// The frame's engines, after [`sounds`] (`dt` its): those started, kept or stopped ([`engines`]), then
/// each driven where it is now ([`drive`]).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxeng"))]
pub fn motors<H: Hw + ?Sized>(hw: &mut H, a: &Agent, dt: u16) {
    let now = a.t as u16;
    engines(hw, a, now, dt);
    for p in 0..PAIRS {
        let s = STATE + state::PAIRS + p * PAIR;
        let id = hw.peek(s + pair::PATCH);
        if !is_engine(id) {
            continue;
        }
        let t = engine_patch(id);
        let itd = set(hw, a, p, t, now);
        drive(hw, p, t, itd, dt);
    }
}

/// An engine's patch from its pair's id (its patch + 1).
#[inline(always)]
fn engine_patch(id: u8) -> &'static [u8; PATCH] {
    &ENGINES[(id - 1 - patch::ENGINE) as usize % ENGINES.len()]
}

/// Pair `p`'s gates released (its patch `t`): its sound fades out. (Inline: a copy in each of its
/// callers' overlays, the one-shots' and the engines'.)
#[inline(always)]
fn release<H: Hw + ?Sized>(hw: &mut H, p: u16, t: &[u8; PATCH]) {
    for chip in 0..core::hint::black_box(2u16) {
        for v in 0..3u16 {
            hw.poke(LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP + 7 * v + 4, t[v as usize * VOICE + 2]);
        }
    }
}

/// The engines: the two nearest vehicles that have one (`squad::MEMBERS`, by their true distance: the
/// list puts those behind further off) onto the pairs not playing anything else. A pair already on one
/// of them, or on another of its platoon (a formation's vehicles keep changing places in the order),
/// keeps it, not restarted (its vehicle, place, revs and Doppler brought up to date); one on another
/// platoon's is given the next, or stopped. A near one-shot of note takes a pair from an engine (`play`);
/// the engine is back when it is done.
///
/// Its Doppler shift ([`pair::DOP`]) follows its closing speed: its distance from the player (1/16
/// texel, both moving) against last frame's, toward [`DOP_GAIN`] × that a tick, slewed (smoothing the
/// frames' unevenness).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxeng"))]
fn engines<H: Hw + ?Sized>(hw: &mut H, a: &Agent, now: u16, dt: u16) {
    use crate::squad::{MEMBERS, SLOTS, entry, nth};
    use crate::world_sim::platoon;
    let (pu, pv) = a.at();
    // The player, 1/16 texel.
    let (qu, qv) = ((a.u >> 4) as u16, (a.v >> 4) as u16);
    // The two nearest: their entries (0: none), nearest first.
    let (mut best, mut dist) = ([0u16; 2], [u16::MAX; 2]);
    for k in 0..hw.peek(MEMBERS).min(SLOTS) {
        let e = nth(hw, k);
        let mesh = hw.peek(e + entry::MESH);
        if mesh >= crate::actors::kind::COUNT * 2 || ENGINE_OF[(mesh >> 1) as usize] == SILENT {
            continue;
        }
        let (ax, ay) = (
            ((rd(hw, e + entry::U) >> 4) as i16).wrapping_sub(pu).unsigned_abs(),
            ((rd(hw, e + entry::V) >> 4) as i16).wrapping_sub(pv).unsigned_abs(),
        );
        let d = ax.max(ay) + (ax.min(ay) >> 1);
        if d < dist[0] {
            (best[1], dist[1]) = (best[0], dist[0]);
            (best[0], dist[0]) = (e, d);
        } else if d < dist[1] {
            (best[1], dist[1]) = (e, d);
        }
    }
    // Pairs already on one: kept (that one taken), else free for the next. (Rolled: unrolled, the four
    // tries each had the whole update.)
    let mut keep = [false; PAIRS as usize];
    for p in 0..core::hint::black_box(PAIRS) {
        let s = STATE + state::PAIRS + p * PAIR;
        if !is_engine(hw.peek(s + pair::PATCH)) {
            continue;
        }
        let was = platoon(rd(hw, s + pair::VID));
        for k in 0..core::hint::black_box(2usize) {
            let e = best[k];
            if e != 0 && platoon(rd(hw, e + entry::VID)) == was {
                follow(hw, s, e, qu, qv, dt);
                best[k] = 0;
                keep[p as usize] = true;
                break;
            }
        }
    }
    let mut next = 0usize;
    for p in 0..core::hint::black_box(PAIRS) {
        let s = STATE + state::PAIRS + p * PAIR;
        let id = hw.peek(s + pair::PATCH);
        if keep[p as usize] || (id != 0 && !is_engine(id)) {
            continue;
        }
        while next < 2 && best[next] == 0 {
            next += 1;
        }
        if next < 2 {
            let e = best[next];
            next += 1;
            let sound = ENGINE_OF[(hw.peek(e + entry::MESH) >> 1) as usize];
            let (u, v, vid) = (rd(hw, e + entry::U), rd(hw, e + entry::V), rd(hw, e + entry::VID));
            start(hw, a, p, sound, &ENGINES[(sound - patch::ENGINE) as usize % ENGINES.len()], u, v, 0, now, REVS_START);
            wr(hw, s + pair::VID, vid);
            wr(hw, s + pair::START, span(u.wrapping_sub(qu), v.wrapping_sub(qv)));
            hw.poke(s + pair::DOP, 0);
        } else if id != 0 {
            // An engine no longer among the nearest: faded out.
            release(hw, p, engine_patch(id));
            hw.poke(s + pair::PATCH, 0);
        }
    }
}

/// Pair `s` (its entry) kept on the vehicle at entry `e` (its platoon's): its revs and Doppler brought
/// up to date (the player at (`qu`, `qv`), 1/16 texel), then its vehicle, place and distance. Another of
/// its platoon taken over is not moving: its place and distance taken as they are.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxeng"))]
fn follow<H: Hw + ?Sized>(hw: &mut H, s: u16, e: u16, qu: u16, qv: u16, dt: u16) {
    use crate::squad::entry;
    let (u, v, vid) = (rd(hw, e + entry::U), rd(hw, e + entry::V), rd(hw, e + entry::VID));
    let d = span(u.wrapping_sub(qu), v.wrapping_sub(qv));
    if vid == rd(hw, s + pair::VID) {
        // Its revs drifting up while it moves (a 16th of a texel a tick or more since the last frame),
        // down while it stands: a tick's worth a tick ([`REVS`]).
        let moved = (u.wrapping_sub(rd(hw, s + pair::U)) as i16).unsigned_abs() + (v.wrapping_sub(rd(hw, s + pair::V)) as i16).unsigned_abs();
        let (revs, step) = (hw.peek(s + pair::SCORE), dt.min(255) as u8);
        hw.poke(s + pair::SCORE, if moved >= dt { revs.saturating_add(step) } else { revs.saturating_sub(step) });
        // Its Doppler toward GAIN × the closing a tick, at most 8 a frame (the frames' unevenness
        // smoothed). (The u16 divide: the resident `__wrap___udivhi3`.)
        let closing = rd(hw, s + pair::START).wrapping_sub(d) as i16;
        let speed = ((closing.unsigned_abs().min(2048) * DOP_GAIN as u16) / dt.max(1)).min(DOP_MAX as u16) as i16;
        let target = if closing < 0 { -speed } else { speed };
        let dop = hw.peek(s + pair::DOP) as i8 as i16;
        hw.poke(s + pair::DOP, (dop + (target - dop).clamp(-8, 8)) as u8);
    }
    wr(hw, s + pair::START, d);
    wr(hw, s + pair::U, u);
    wr(hw, s + pair::V, v);
    wr(hw, s + pair::VID, vid);
}

/// How far an offset (1/16 texel, wrapping) reaches: `max + min / 2`, as [`place`]'s.
#[inline(always)]
fn span(du: u16, dv: u16) -> u16 {
    let (ax, ay) = ((du as i16).unsigned_abs(), (dv as i16).unsigned_abs());
    ax.max(ay) + (ax.min(ay) >> 1)
}

/// An engine's voices each frame on pair `p` (its patch `t`): its pitch raised by its revs (the patch's
/// [`field::REV_DEPTH`]) and shifted by its Doppler ([`pair::DOP`], both chips alike); and the ears' lag
/// (`ITD`) drawn toward `itd` ([`place`]'s, for where it is now) without a restart: one ear's chip runs
/// a 256th slow until the next frame, falling a unit behind a tick. The lag is credited when that is
/// done (the next frame's `dt`: the ticks it really ran slow), and a chip is slowed while that brings
/// the lag nearer (by more than half this frame's `dt`, the likely next: the registers of the low drones
/// have no finer step). The oscillators keep their phases between the chips, so the steady drone has
/// its time lag too, not only its levels. (A voice under $100 (below 15 Hz) has no
/// 256th to lose: it keeps no lag, nor needs one.)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxeng"))]
fn drive<H: Hw + ?Sized>(hw: &mut H, p: u16, t: &[u8; PATCH], itd: i8, dt: u16) {
    let s = STATE + state::PAIRS + p * PAIR;
    let flags = hw.peek(s + pair::FLAGS);
    // The last frame's slow chip, its ticks credited: the left ear's lag up, or down.
    let step = dt.min(255) as i16;
    let mut lag = hw.peek(s + pair::ITD) as i8 as i16;
    if flags & SLOW_L != 0 {
        lag += step;
    } else if flags & SLOW_R != 0 {
        lag -= step;
    }
    let lag = lag.clamp(-127, 127);
    hw.poke(s + pair::ITD, lag as u8);
    let diff = itd as i16 - lag;
    // The chip slowed now: the left (its ear later), the right, or neither (2).
    let (slow, mark) = if 2 * diff.abs() <= step {
        (2, 0)
    } else if diff > 0 {
        (0, SLOW_L)
    } else {
        (1, SLOW_R)
    };
    hw.poke(s + pair::FLAGS, flags & !(SLOW_L | SLOW_R) | mark);
    let revs = hw.peek(s + pair::SCORE) as u16;
    let depth = t[field::REV_DEPTH] as u16;
    let dop = hw.peek(s + pair::DOP) as i8 as i16;
    for chip in 0..core::hint::black_box(2u16) {
        let c = LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP;
        for v in 0..core::hint::black_box(3u16) {
            let o = v as usize * VOICE;
            let hi = t[o] as u16;
            let mut f = (hi << 8 | t[o + 5] as u16) + ((hi * depth * revs) >> 4);
            f = f.wrapping_add(((f >> 8) as i16 * dop) as u16);
            if chip == slow {
                f -= f >> 8;
            }
            hw.poke(c + 7 * v, f as u8);
            hw.poke(c + 7 * v + 1, (f >> 8) as u8);
        }
    }
}

/// A frame this many ticks or more after the last: a stall (nothing new sounds).
pub const STALL: u16 = 255;

/// The frame's new events onto the pairs (`dt` the ticks since the last frame; `flash`, `lock` now).
#[inline(always)]
fn events<H: Hw + ?Sized>(hw: &mut H, a: &Agent, now: u16, dt: u16, flash: u8, lock: u8) {
    use crate::combat::{FX, FX_ENTRY, FX_MAX, PROJ, PROJ_ENTRY, PROJ_MAX, proj};
    // The effects begun since the last frame.
    for k in 0..FX_MAX as u16 {
        let e = FX + k * FX_ENTRY;
        let id = hw.peek(e);
        if id == 0 {
            continue;
        }
        let age = now.wrapping_sub(rd(hw, e + 2));
        if age >= dt {
            continue;
        }
        use crate::effects::effect;
        let sound = match (id - 1) % crate::effects::EFFECTS as u8 {
            effect::EXPLOSION => patch::EXPLOSION,
            effect::IMPACT => patch::IMPACT,
            effect::SPLASH => patch::SPLASH,
            effect::MUZZLE => patch::MUZZLE,
            effect::WRECK_FIRE => patch::CRACKLE,
            _ => continue,
        };
        let far = if hw.peek(e + 1) != 0 { FAR } else { 0 };
        let (u, v) = (rd(hw, e + 4), rd(hw, e + 6));
        play(hw, a, sound, u, v, far, now);
    }
    // The player's: new tracers and missiles, a hit, the lock.
    for k in 0..PROJ_MAX {
        let p = PROJ + k * PROJ_ENTRY;
        let (sound, ttl) = match hw.peek(p + proj::KIND) {
            crate::actors::kind::TRACER => (patch::GUN, crate::combat::TRACER_TTL),
            crate::actors::kind::MISSILE => (patch::LAUNCH, crate::combat::MISSILE_TTL),
            _ => continue,
        };
        // (Launched this frame: the effects pass has taken this frame's ticks off its life already.)
        if ((ttl - hw.peek(p + proj::TTL)) as u16) <= dt {
            play(hw, a, sound, 0, 0, OWN, now);
        }
    }
    if flash > hw.peek(STATE + state::FLASH) {
        play(hw, a, patch::HIT, 0, 0, OWN, now);
    }
    if lock >= crate::hud::LOCK_TICKS && hw.peek(STATE + state::LOCK) < crate::hud::LOCK_TICKS {
        play(hw, a, patch::LOCK, 0, 0, OWN, now);
    }
}

/// Sound `sound` at (`u`, `v`) (1/16 texel; `OWN`: the player's) onto a pair, if it wins one.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn play<H: Hw + ?Sized>(hw: &mut H, a: &Agent, sound: u8, u: u16, v: u16, flags: u8, now: u16) {
    let t = &PATCHES[sound as usize];
    let g = if flags & OWN != 0 { 16 } else { gain(a, u, v, flags) };
    let score = (t[field::PRIORITY] as u16 * g as u16).min(255) as u8;
    if score == 0 {
        return;
    }
    // The pair with the least left (a free one: nothing). The gun keeps to one pair: a burst goes onto the
    // pair already firing, and nothing else takes that one from it (so the gun and a launch, say, are
    // heard together).
    let (mut best, mut least) = (0u16, u16::MAX);
    for p in 0..PAIRS {
        let s = STATE + state::PAIRS + p * PAIR;
        let id = hw.peek(s + pair::PATCH);
        if id == patch::GUN + 1 {
            if sound == patch::GUN {
                (best, least) = (p, 0);
                break;
            }
            continue;
        }
        // (A free pair: nothing left; an engine's next to nothing, so a free one goes first. A crackle or
        // the far battle leaves an engine be: they would keep it fading out and in.)
        let left = if id == 0 {
            0
        } else if is_engine(id) {
            if t[field::PRIORITY] < 2 || flags & FAR != 0 {
                continue;
            }
            1
        } else {
            let len = PATCHES[(id - 1) as usize % patch::ONE_SHOTS][field::LENGTH] as u16;
            let age = now.wrapping_sub(rd(hw, s + pair::START)).min(len);
            hw.peek(s + pair::SCORE) as u16 * (len - age) / len
        };
        if left < least {
            (best, least) = (p, left);
        }
    }
    if (score as u16) < least {
        return;
    }
    start(hw, a, best, sound, t, u, v, flags, now, score);
}

/// Sound `sound` (its patch `t`: from the caller's overlay) begun on pair `p` (its state: at (`u`, `v`),
/// `flags`, `score`).
///
/// The IRQ's sweep off; the voices set up on both chips held silent (the test bit), the cutoffs and
/// volumes placed, then gated: the near ear's chip first, the far one's [`place`]'s lag later (raster
/// lines: [`wait`]), so the sound reaches the far ear later, as round a head. Then the sweep on.
#[allow(clippy::too_many_arguments)]
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
fn start<H: Hw + ?Sized>(hw: &mut H, a: &Agent, p: u16, sound: u8, t: &[u8; PATCH], u: u16, v: u16, flags: u8, now: u16, score: u8) {
    let s = STATE + state::PAIRS + p * PAIR;
    hw.poke(s + pair::PATCH, sound + 1);
    hw.poke(s + pair::FLAGS, flags);
    wr(hw, s + pair::U, u);
    wr(hw, s + pair::V, v);
    wr(hw, s + pair::START, now);
    hw.poke(s + pair::SCORE, score);
    let w = SWEEP + p * SWEEP_PAIR;
    hw.poke(w, 0);
    // (Rolled: unrolled, the two chips' writes doubled the overlay.)
    for chip in 0..core::hint::black_box(2u16) {
        let c = LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP;
        for v in 0..3u16 {
            hw.poke(c + 7 * v + 4, 0x08);
        }
        hw.poke(c + 0x17, t[field::ROUTE]);
        for v in 0..3u16 {
            let o = v as usize * VOICE;
            let r = c + 7 * v;
            hw.poke(r, 0);
            hw.poke(r + 1, t[o]);
            hw.poke(r + 2, 0);
            hw.poke(r + 3, t[o + 1]);
            hw.poke(r + 5, t[o + 3]);
            hw.poke(r + 6, t[o + 4]);
        }
    }
    let itd = set(hw, a, p, t, now);
    hw.poke(s + pair::ITD, itd as u8);
    // The near ear (the right when the left lags), then the far.
    let near = (itd > 0) as u16;
    gate(hw, p, near, t);
    wait(hw, itd.unsigned_abs());
    gate(hw, p, near ^ 1, t);
    for v in 0..3u16 {
        hw.poke(w + 1 + v, t[v as usize * VOICE]);
        hw.poke(w + 4 + v, t[v as usize * VOICE + 5]);
    }
    hw.poke(w, t[field::STEPS]);
}

/// Pair `p`'s chip `chip` (0 left, 1 right) gated: its voices sound (the test bit off).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
fn gate<H: Hw + ?Sized>(hw: &mut H, p: u16, chip: u16, t: &[u8; PATCH]) {
    let c = LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP;
    for v in 0..3u16 {
        let wave = t[v as usize * VOICE + 2];
        hw.poke(c + 7 * v + 4, wave | (wave != 0) as u8);
    }
}

/// `lines` raster lines (63.6 µs each: about [`place`]'s lag unit), whatever the CPU's clock. The lines
/// passed are counted, not the changes seen: the raster IRQ coming meanwhile (many lines at once) counts
/// in full. ($D012's low byte falls back at the frame's end, 311 to 0: a line.)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
pub(crate) fn wait<H: Hw + ?Sized>(hw: &mut H, lines: u8) {
    let mut passed = 0u8;
    let mut line = hw.peek(0xD012);
    while passed < lines {
        let now = hw.peek(0xD012);
        let d = now.wrapping_sub(line);
        passed = passed.saturating_add(if d > 128 { 1 } else { d });
        line = now;
    }
}

/// A sound's gain (0..=16) where it is.
#[inline(always)]
fn gain(a: &Agent, u: u16, v: u16, flags: u8) -> u8 {
    let (pu, pv) = a.at();
    let (dx, dy) = (((u >> 4) as i16).wrapping_sub(pu), ((v >> 4) as i16).wrapping_sub(pv));
    let (l, r, _, _, _) = place(dx, dy, a.yaw, flags & FAR != 0);
    l.max(r) + 1
}

/// Pair `p`'s two chips' volumes and cutoffs for where its sound is now (`t` its patch); the ears' lag
/// for there ([`place`]'s).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
fn set<H: Hw + ?Sized>(hw: &mut H, a: &Agent, p: u16, t: &[u8; PATCH], now: u16) -> i8 {
    let s = STATE + state::PAIRS + p * PAIR;
    let flags = hw.peek(s + pair::FLAGS);
    let (vl, vr, cl, cr, itd) = if flags & OWN != 0 {
        (15, 15, 255, 255, 0)
    } else {
        let (pu, pv) = a.at();
        let (dx, dy) = (((rd(hw, s + pair::U) >> 4) as i16).wrapping_sub(pu), ((rd(hw, s + pair::V) >> 4) as i16).wrapping_sub(pv));
        place(dx, dy, a.yaw, flags & FAR != 0)
    };
    // The cutoff swept a tick, then closed for the ear.
    let age = now.wrapping_sub(rd(hw, s + pair::START)).min(255) as i16;
    let open = (t[field::CUTOFF] as i16 + t[field::CUT_SWEEP] as i8 as i16 * age).clamp(0, 255) as u16;
    for chip in 0..core::hint::black_box(2u16) {
        let (vol, cut) = if chip == 0 { (vl, cl) } else { (vr, cr) };
        let c = LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP;
        hw.poke(c + 0x15, 0);
        hw.poke(c + 0x16, ((open * cut as u16) >> 8) as u8);
        hw.poke(c + 0x18, t[field::MODE] | vol);
    }
    itd
}

/// The first frame: the four chips silent, filters low-pass, the state clear.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn init<H: Hw + ?Sized>(hw: &mut H, now: u16) {
    for p in 0..PAIRS {
        hw.poke(SWEEP + p * SWEEP_PAIR, 0);
        for chip in 0..core::hint::black_box(2u16) {
            let c = LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP;
            for r in 0..0x18u16 {
                hw.poke(c + r, 0);
            }
            hw.poke(c + 0x18, LP);
        }
    }
    for i in 1..(state::PAIRS + PAIRS * PAIR) {
        hw.poke(STATE + i, 0);
    }
    wr(hw, STATE + state::LAST, now);
    hw.poke(STATE + state::LOCK, 0xFF);
    hw.poke(STATE + state::MAGIC, MAGIC);
}
