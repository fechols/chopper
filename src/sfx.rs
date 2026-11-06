//! Sound effects on the SIDs, placed round the player (a cheap HRTF).
//!
//! The C64U's two UltiSIDs, each split in two (`config/chopper.cfg`: range split on A5), are four SIDs:
//! $D400 and $D420 hard left, $D440 and $D460 hard right. They make two stereo pairs ([`PAIRS`]): pair `p`
//! is the left chip at `$D400 + $20 p` and the right one at `$D440 + $20 p`. A sound takes a whole pair:
//! the same three voices on both chips (a [`patch`]), each chip with its own master volume (its ear's
//! gain) and its own low-pass cutoff (its ear's muffling). So the filter colours that one sound only.
//!
//! - **Where it is** ([`place`], a model of a head): its lateral angle (in 3D, the ears tilting with
//!   the bank) sets the ears' lag, the far ear's level and its muffling (the head's shadow), from tables
//!   shaped like a head's; behind the player both cutoffs close (muffled); its 3D distance lowers the
//!   gain (1/distance) and dulls it (the air). Levels and cutoffs add in octaves, so the cues agree and
//!   each moves a sound the same whatever the others do.
//! - **Smoothly** (`asm/hud.S` `sfx_slew`, from [`SLEW`]): the frame sets each chip's volume and
//!   cutoff as targets; the raster IRQ glides the chips to them at 60 Hz, so a pan sounds the same
//!   whatever the frame rate.
//! - **What sounds** ([`sounds`], once a frame after the effects pass): the frame's new effects (their
//!   slots in `combat::FX` begun since the last frame: impacts, splashes, explosions, wreck fires, the
//!   war's muzzle flashes, the far ones quieter and duller), the player's new tracers and missiles, a hit
//!   on the player (`player::FLASH` rising) and the seeker's lock (`hud::LOCK_STATE` reaching
//!   `LOCK_TICKS`).
//! - **Who gets a pair:** a free one, else the one with the least left of its sound (its score, its
//!   priority × gain, running down over its length), if the new one scores at least that. The player's
//!   own sounds (the gun, a launch, a hit, the lock: `OWN`) keep to one pair, which nothing else takes
//!   from them, so the other is always the world's (its effects, the engines). On it a sound of higher
//!   priority wins while it is held: a launch cuts in over the gun, and the gun's bursts are back once
//!   the launch's attack is over (its gate released).
//! - **Pitch sweeps** run at 60 Hz in the raster IRQ (`asm/hud.S`, from [`SWEEP`]); the frame (13-20 a
//!   second) triggers sounds, releases their gates, sweeps their cutoffs and places them again (the
//!   player turning pans what is playing).
//! - **The ears' lag** ([`place`]'s last): a sound is gated on the near ear's chip first and on the far
//!   one that many raster lines later ([`start`]), as it reaches the far ear later round a head. Its lag
//!   then follows the player turning ([`lag`]: an engine's in [`drive`], a one-shot's once its pitch
//!   sweep is done in [`track`]).
//! - **Engines** ([`engines`]): the two nearest vehicles that have one (`squad::MEMBERS`) take the pairs
//!   no effect is using, held until they drop out of the two; any effect takes a pair from an engine
//!   (a free pair first), and the engine is back once it is done. Their pitch rises while they move.
//!
//! Three overlays: the one-shots ([`sounds`], `.text.sfx`, with [`place`]: it places every pair each
//! frame, [`aim`]), then the engines ([`motors`], `.text.sfxeng`) in turn in the first window, each with
//! its own patches; setting and starting a sound ([`set`], [`start`], fed the patch by its caller, from
//! its stored place, [`PLACED`]) in a library in the second (`.text.sfxlib`, at $A000, fetched before
//! both).
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
/// Flags: its gates released; the player's own (centred, wherever the player flies); far off; its left
/// chip running slow since the last frame, its right one (the ears' lag drawn along: [`lag`]).
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
/// Each pair's sound's height (u16, 1/16 texel), beside its place in its entry ([`pair::U`]).
pub const HEIGHT: u16 = 0xFF2E;
/// The IRQ's slews (`asm/hud.S` has these addresses): [`SLEW_CH`] targets, then the values on the SIDs.
/// Channel `4 p + 2 chip` is pair `p`'s chip's `$D418` (the filter's mode and the volume), the next
/// its `$D416` (the cutoff). Once a video frame the IRQ moves each value a quarter of the way to its
/// target (at least 1) and writes it, so a sound pans and dulls at the same pace whatever the frame rate
/// (its placing, 13-20 times a second, sets the targets: [`set`]); a sound begun snaps them ([`set`]).
pub const SLEW: u16 = 0xFF32;
pub const SLEW_CH: u16 = 8;
// (`asm/hud.S` has these addresses.)
const _: () = assert!(SWEEP == 0xFF20 && SLEW == 0xFF32);
const _: () = assert!(STATE + state::PAIRS + PAIRS * PAIR <= SWEEP && SWEEP + PAIRS * SWEEP_PAIR <= HEIGHT);
/// Each pair's place ([`place`]'s five bytes: the volumes, the cutoff factors, the lag), worked out in the
/// one-shots' overlay where [`place`] is ([`aim`], each frame, and [`play`] for a sound begun) and read by
/// [`set`] (in the library, also for the engines afterwards: their places a frame old). An engine begun
/// starts silent and centred ([`QUIET`]) and glides to its place from the next frame.
pub const PLACED: u16 = 0xFF42;
pub const PLACE: u16 = 5;
const _: () = assert!(HEIGHT + 2 * PAIRS <= SLEW && SLEW + 2 * SLEW_CH <= PLACED && SLEW_CH == 4 * PAIRS);
/// Non-zero while [`set`] snaps a sound's levels (targets, values and SIDs together): the IRQ's slews
/// skip that video frame (`asm/hud.S` `sfx_slew`). (With a target written and not its value, or a value
/// and not its target, they would glide between the old sound's and the new, writing a volume byte of
/// neither's filter mode.)
pub const HOLD: u16 = 0xFF4C;
const _: () = assert!(PLACED + PAIRS * PLACE <= HOLD && HOLD < 0xFFFA);
/// A place: silent and centred, the cutoffs open (an engine begun).
pub const QUIET: (u8, u8, u8, u8, i8) = (0, 0, 255, 255, 0);

/// Pair `p`'s place stored ([`PLACED`]).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
fn store<H: Hw + ?Sized>(hw: &mut H, p: u16, at: (u8, u8, u8, u8, i8)) {
    let e = PLACED + p * PLACE;
    hw.poke(e, at.0);
    hw.poke(e + 1, at.1);
    hw.poke(e + 2, at.2);
    hw.poke(e + 3, at.3);
    hw.poke(e + 4, at.4 as u8);
}

/// The place of a sound at (`u`, `v`), height `h` (1/16 texel; `flags` its: the player's own centred,
/// the far battle's).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn locate(a: &Agent, u: u16, v: u16, h: u16, flags: u8) -> (u8, u8, u8, u8, i8) {
    if flags & OWN != 0 {
        return (15, 15, 255, 255, 0);
    }
    let (pu, pv) = a.at();
    let (dx, dy, dz) = (((u >> 4) as i16).wrapping_sub(pu), ((v >> 4) as i16).wrapping_sub(pv), ((a.h >> 8) as i16).wrapping_sub((h >> 4) as i16));
    place(dx, dy, dz, a.yaw, a.roll as u16, flags & FAR != 0)
}

/// Every pair playing placed where its sound is now ([`PLACED`]): the frame's, before [`set`] (here for
/// the one-shots, then for the engines in their overlay). An engine is placed where `follow` left it last
/// frame, a frame late: placing it after `follow` would need this overlay and the engines' in the first
/// window at once (a second fetch each frame); the IRQ's slews glide over the step.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn aim<H: Hw + ?Sized>(hw: &mut H, a: &Agent) {
    for p in 0..core::hint::black_box(PAIRS) {
        let s = STATE + state::PAIRS + p * PAIR;
        if hw.peek(s + pair::PATCH) != 0 {
            let at = locate(a, rd(hw, s + pair::U), rd(hw, s + pair::V), rd(hw, HEIGHT + 2 * p), hw.peek(s + pair::FLAGS));
            store(hw, p, at);
        }
    }
}

/// A channel's SID register (from `$D400`), as the IRQ's table (`asm/hud.S` `sfx_reg`).
#[cfg(not(target_arch = "mos"))]
fn slew_reg(ch: u16) -> u16 {
    (LEFT - 0xD400) + (ch & 2) * ((RIGHT - LEFT) / 2) + (ch >> 2) * PAIR_STEP + if ch & 1 == 0 { 0x18 } else { 0x16 }
}

/// The IRQ's slews once (the host's copy of `asm/hud.S` `sfx_slew`, for the tests: the C64 has the IRQ).
#[cfg(not(target_arch = "mos"))]
pub fn slew<H: Hw + ?Sized>(hw: &mut H) {
    if hw.peek(HOLD) != 0 {
        return;
    }
    for ch in 0..SLEW_CH {
        let (t, c) = (hw.peek(SLEW + ch), hw.peek(SLEW + SLEW_CH + ch));
        if t == c {
            continue;
        }
        let n = if t > c { c + ((t - c) >> 2) + 1 } else { c - ((c - t - 1) >> 2) - 1 };
        hw.poke(SLEW + SLEW_CH + ch, n);
        hw.poke(0xD400 + slew_reg(ch), n);
    }
}

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
    [0x30, 0, NOISE, 0x19, 0x09, 0, 0x14, 0, NOISE, 0x29, 0x08, 0, 0x02, 0, TRI, 0x07, 0x00, 0, 0x27, LP, 0x50, -1i8 as u8, 0, 4, 45, 3],
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
    [0x06, 0, SAW, 0x80, 0x96, 0, 0x0C, 0, TRI, 0x80, 0x86, 0, 0x08, 0, NOISE, 0x80, 0x46, 0, 0x17, LP, 0x70, 0, 0, 4, 255, 1],
    // SHIP: a ship's deep triangle drone, a thin low pulse, the wash.
    [0x02, 0, TRI, 0xA0, 0xC8, 0, 0x04, 2, PULSE, 0xA0, 0x88, 0, 0x01, 0, NOISE, 0xA0, 0x98, 0, 0x37, LP, 0x40, 0, 0, 1, 255, 1],
    // ROTOR: a sub-audio narrow pulse (the blades' thumps), a triangle ringing off it, the turbine's hiss.
    [0x01, 2, PULSE, 0x80, 0xA6, 0, 0x0E, 0, TRI | RING, 0x80, 0x86, 0, 0x06, 0, NOISE, 0x80, 0x46, 0, 0x17, LP, 0x60, 0, 0, 2, 255, 1],
    // DIESEL: the carrier's giant diesel, never revving: a rumble, no edges to pop: a 31 Hz triangle and a
    // 37 Hz saw (NTSC, `config/chopper.cfg`; 30 and 36 on PAL) beating against it (a soft throb about 6
    // times a second, the cylinders), and low noise
    // (the rumble's body), all through a deep low-pass with little resonance (nothing rings).
    [0x02, 0, TRI, 0xA0, 0xC8, 0x00, 0x02, 0, SAW, 0xA0, 0xA8, 0x60, 0x03, 0, NOISE, 0xA0, 0xB8, 0x00, 0x17, LP, 0x18, 0, 0, 0, 255, 1],
];
// Every voice of a sound placed in the world goes through its chip's filter (the ears' muffling and the
// head's shadow are the cutoff: a voice past it would be heard unshadowed). The player's own (centred)
// may skip it: the hit's clang and the lock's beep do.
const fn filtered(t: &[u8; PATCH]) -> bool {
    let mut v = 0;
    while v < 3 {
        if t[v * VOICE + 2] != 0 && t[field::ROUTE] & (1 << v) == 0 {
            return false;
        }
        v += 1;
    }
    true
}
const _: () = {
    let mut k = 0;
    while k < PATCHES.len() {
        assert!(filtered(&PATCHES[k]) || k == patch::HIT as usize || k == patch::LOCK as usize);
        k += 1;
    }
    let mut k = 0;
    while k < ENGINES.len() {
        assert!(filtered(&ENGINES[k]));
        k += 1;
    }
};
// An engine sweeps nothing: its voices' last bytes are frequency low bytes, not the IRQ's deltas (its
// sweep never runs), and its start tick is not kept (no cutoff sweep reads its age).
const _: () = {
    let mut k = 0;
    while k < ENGINES.len() {
        assert!(ENGINES[k][field::STEPS] == 0 && ENGINES[k][field::CUT_SWEEP] == 0);
        k += 1;
    }
};

/// A sound's place for each ear (a model of a head, tuned by ear but shaped like a measured one): (left
/// volume, right volume, left cutoff factor, right one, the left ear's lag behind the right): volumes
/// 0..=15, factors /256 of the patch's cutoff, the lag in units of a 256th of a tick (65 µs, about a
/// raster line), -[`ITD_MAX`]..=[`ITD_MAX`]. `dx`, `dy`, `dz`: the source from the player (texels;
/// east, north, down: the player's height over it), `yaw` the player's heading (binary angle, 0 north,
/// clockwise), `roll` its bank (binary angle, right side down positive: the ears tilt with the view).
/// `far`: the battle on the horizon.
///
/// Every cue comes from the source's direction and distance alone, so they always agree:
/// - **Its lateral angle** (`x`: the cosine to the ears' axis, 16ths, 0 ahead, behind or straight
///   below; 16 abeam): the lag ([`ITD_LINES`], round a sphere), the far ear's level ([`ILD`]) and its
///   muffling ([`SHADOW`], the head's shadow).
/// - **Its distance** (3D: a source below is near, and centres as the player passes over it): the gain
///   (as 1/distance) and the air's dulling.
///
/// Levels and cutoffs are worked in 16ths of an octave and added, so each cue moves a sound the same
/// whatever the others do (the shadow is the same octave behind as ahead, the level gap the same near as
/// far) and only the final 4-bit volumes round ([`VOL`], in the log domain: the nearest step by ratio).
/// Behind (`fwd`, the cosine to the nose) both ears are muffled more: the pinnae's front/back cue.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub fn place(dx: i16, dy: i16, dz: i16, yaw: u16, roll: u16, far: bool) -> (u8, u8, u8, u8, i8) {
    let (dh, r) = spans(dx, dy, dz);
    let (mut lat, mut fwd) = (0i8, 16i8);
    if r != 0 {
        let (side, ahead) = bearing(dx, dy, yaw);
        let k = cosine(dh, r);
        lat = onto(side, k);
        fwd = onto(ahead, k);
        if roll != 0 {
            let down = cosine(dz.unsigned_abs(), r) as i8;
            lat = tilt(lat, if dz < 0 { -down } else { down }, roll);
        }
    }
    ears(lat, fwd, r, far)
}

/// The distance across the ground and in 3D (`max + min / 2`, then with the height).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn spans(dx: i16, dy: i16, dz: i16) -> (u16, u16) {
    let (ax, ay, az) = (dx.unsigned_abs(), dy.unsigned_abs(), dz.unsigned_abs());
    let dh = ax.max(ay) + (ax.min(ay) >> 1);
    (dh, dh.max(az).saturating_add(dh.min(az) >> 1))
}

/// The bearing from the nose: (`side` -16 left ..= 16 right, `ahead` -16 ..= 16), [`sin16`]'s.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn bearing(dx: i16, dy: i16, yaw: u16) -> (i8, i8) {
    let rel = crate::world_sim::heading_inline(dx, dy).wrapping_sub(angle8(yaw as i16));
    (sin16(rel), sin16(rel.wrapping_add(64)))
}

/// A binary angle to a byte's (256 a turn), rounded on the magnitude (a bank and its mirror alike).
#[inline(always)]
fn angle8(a: i16) -> u8 {
    let m = ((a.unsigned_abs() + 0x80) >> 8) as u8;
    if a < 0 { m.wrapping_neg() } else { m }
}

/// The sine of a byte's angle in 16ths, rounded ([`SIN16`]): the sounds' own, not `camera::sin`, whose
/// asm multiply puts its callers on llvm-mos's soft stack (more code).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn sin16(a: u8) -> i8 {
    let p = (a & 63) as usize;
    let v = (if a & 64 == 0 { SIN16[p] } else { SIN16[64 - p] }) as i8;
    if a & 128 == 0 { v } else { -v }
}

/// `16 sin` over a quarter turn (65 steps of a byte's angle), rounded.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub static SIN16: [u8; 65] = [
    0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14,
    14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
];

/// `16 n / r` (n ≤ r, r > 0): a cosine in 16ths. (Both scaled into 12 bits, for the 16-bit divide.)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn cosine(n: u16, r: u16) -> u8 {
    let (n, r) = if r > 4095 { (n >> 4, r >> 4) } else { (n, r) };
    ((n << 4) / r.max(1)).min(16) as u8
}

/// A bearing's cosine `c` (16ths) onto the ground's `k`: `c k / 16`, rounded on the magnitude (the two
/// sides alike).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn onto(c: i8, k: u8) -> i8 {
    let m = ((c.unsigned_abs() as u16 * k as u16 + 8) >> 4) as i8;
    if c < 0 { -m } else { m }
}

/// The ears' axis tilted by the bank (`roll`, right side down positive): a source below (`down`, 16ths)
/// moves toward the lower ear. Rounded on the magnitude (the two banks alike).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn tilt(lat: i8, down: i8, roll: u16) -> i8 {
    let a = angle8(roll as i16);
    let (sr, cr) = (sin16(a) as i16, sin16(a.wrapping_add(64)) as i16);
    let v = lat as i16 * cr + down as i16 * sr;
    let m = ((v.unsigned_abs() + 8) >> 4).min(16) as i8;
    if v < 0 { -m } else { m }
}

/// The two ears' volumes, cutoff factors and lag (see [`place`]) from the lateral cosine `lat`, the
/// forward one `fwd` (16ths) and the distance `r` (texels).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn ears(lat: i8, fwd: i8, r: u16, far: bool) -> (u8, u8, u8, u8, i8) {
    let x = (lat.unsigned_abs() as usize).min(16);
    // The distance's attenuation (16ths of an octave of amplitude: 6 dB an octave of distance), full
    // within 16 texels; the horizon's battle 6 dB down.
    let att = log2q(r.min(4000) + 8).saturating_sub(LOG2Q_24) + if far { 16 } else { 0 };
    let near = vol(att);
    let wide = vol(att + ILD[x]);
    // Muffled behind (1.25 octaves, abeam half that), by the air (an octave by 512 texels; the horizon's
    // more), and the far ear by the head's shadow.
    let dull = ((16 - fwd) as u8 * 5 >> 3) + (r >> 5).min(16) as u8 + if far { 6 } else { 0 };
    let open = cutoff(dull);
    let shade = cutoff(dull + SHADOW[x]);
    let itd = ITD_LINES[x] as i8;
    if lat > 0 { (wide, near, shade, open, itd) } else if lat < 0 { (near, wide, open, shade, -itd) } else { (near, near, open, open, 0) }
}

/// The ears' lag at most ([`place`]'s units): a source abeam.
pub const ITD_MAX: i8 = 10;

/// The ears' lag (raster lines) by the lateral cosine (16ths): Woodworth's sphere, `(a/c)(asin x + x)`,
/// each 16th's mean, [`ITD_MAX`] (0.64 ms, a head's) abeam. (Each 16th's mean, not its end: the curve
/// is steep abeam, and this keeps every step to a line.)
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub static ITD_LINES: [u8; 17] = [0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 9, 10];
/// The far ear's level below the near's (16ths of an octave: 0.38 dB) by the lateral cosine: a
/// broadband 10.5 dB abeam, linear in the cosine, as a head's is below its high-band peak.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub static ILD: [u8; 17] = [0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 18, 19, 21, 23, 25, 26, 28];
/// The far ear's cutoff lowered (16ths of an octave) by the lateral cosine: an octave abeam, the head's
/// shadow over the highs.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub static SHADOW: [u8; 17] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
/// A volume (0..=15) by its attenuation from 15 (16ths of an octave): `15 / 2^(a / 16)`, the step
/// nearest by ratio (so a level gap keeps its size in dB as far as 4 bits allow); silent below 0.75.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub static VOL: [u8; 70] = [
    15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
];
/// `16 log2(1 + m / 16)`: an octave's 16ths by the mantissa's top 4 bits.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub static LOG2M: [u8; 16] = [0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15];
/// `255 / 2^(m / 16)`: a cutoff factor by an octave's 16ths.
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub static CUTM: [u8; 16] = [255, 244, 234, 224, 214, 205, 197, 188, 180, 173, 165, 158, 152, 145, 139, 133];
/// [`log2q`] of 24: [`place`]'s distance where the gain begins to fall (16 texels, + 8).
const LOG2Q_24: u8 = 73;

/// `16 log2(n)` (n ≥ 1), to a 16th.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn log2q(mut n: u16) -> u8 {
    let mut e = 15u8;
    while n & 0x8000 == 0 {
        n <<= 1;
        e -= 1;
    }
    (e << 4) + LOG2M[((n >> 11) & 15) as usize]
}

/// A volume by its attenuation ([`VOL`]).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn vol(att: u8) -> u8 {
    if (att as usize) < VOL.len() { VOL[att as usize] } else { 0 }
}

/// A cutoff factor (/256) that many 16ths of an octave down, at most 4 octaves (16).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn cutoff(q: u8) -> u8 {
    let q = q.min(63);
    (CUTM[(q & 15) as usize] >> (q >> 4)).max(16)
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
    aim(hw, a);
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
        let itd = set(hw, p, t, now, false);
        track(hw, p, itd, dt);
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
        let itd = set(hw, p, t, now, false);
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
                let h = rd(hw, e + entry::H);
                wr(hw, HEIGHT + 2 * p, h);
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
            let h = rd(hw, e + entry::H);
            store(hw, p, QUIET);
            start(hw, p, sound, &ENGINES[(sound - patch::ENGINE) as usize % ENGINES.len()], u, v, h, 0, now, REVS_START);
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
/// its platoon taken over is not moving: its place and distance taken as they are. (Its height: the
/// caller's.)
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

/// How far an offset (1/16 texel, wrapping) reaches across the ground: `max + min / 2`, as [`place`]'s.
/// (The engines' Doppler: on the ground, as their overlay has no room for the height's.)
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
    let slow = lag(hw, s, itd, dt);
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

/// The ears' lag of pair `s` (its entry) brought up to date and drawn toward `itd` ([`drive`]'s, and a
/// one-shot's once its sweep is done: [`track`]): the last frame's slow chip credited its ticks (`dt`
/// this frame's), then the chip to slow until the next frame: 0 the left (its ear later), 1 the right,
/// 2 neither. (Inline: a copy in the library, [`track`]'s, and in the engines' overlay, [`drive`]'s.)
#[inline(always)]
fn lag<H: Hw + ?Sized>(hw: &mut H, s: u16, itd: i8, dt: u16) -> u16 {
    let flags = hw.peek(s + pair::FLAGS);
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
    let (slow, mark) = if 2 * diff.abs() <= step {
        (2, 0)
    } else if diff > 0 {
        (0, SLOW_L)
    } else {
        (1, SLOW_R)
    };
    hw.poke(s + pair::FLAGS, flags & !(SLOW_L | SLOW_R) | mark);
    slow
}

/// A one-shot's ears' lag on pair `p` following the player turning, as an engine's ([`drive`], [`lag`]):
/// once its pitch sweep is done (the IRQ's: till then it owns the frequencies, and the lag stays its
/// onset's), each voice at the sweep's last pitch, the lagging ear's chip a 256th slow until the next
/// frame, written only when that chip changes. (A noise voice's generator is clocked by its frequency
/// too: its lag moves the same.)
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
fn track<H: Hw + ?Sized>(hw: &mut H, p: u16, itd: i8, dt: u16) {
    let w = SWEEP + p * SWEEP_PAIR;
    if hw.peek(w) != 0 {
        return;
    }
    let s = STATE + state::PAIRS + p * PAIR;
    let was = hw.peek(s + pair::FLAGS) & (SLOW_L | SLOW_R);
    let slow = lag(hw, s, itd, dt);
    // (The same chip slow as last frame: the chips already are. Just past the sweep both are on its last
    // pitch, neither slow: `start` cleared the marks.)
    if hw.peek(s + pair::FLAGS) & (SLOW_L | SLOW_R) == was {
        return;
    }
    for chip in 0..core::hint::black_box(2u16) {
        let c = LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP;
        for v in 0..core::hint::black_box(3u16) {
            let mut f = (hw.peek(w + 1 + v) as u16) << 8;
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
        let (u, v, h) = (rd(hw, e + 4), rd(hw, e + 6), rd(hw, e + 8));
        play(hw, a, sound, u, v, h, far, now);
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
            play(hw, a, sound, 0, 0, 0, OWN, now);
        }
    }
    if flash > hw.peek(STATE + state::FLASH) {
        play(hw, a, patch::HIT, 0, 0, 0, OWN, now);
    }
    if lock >= crate::hud::LOCK_TICKS && hw.peek(STATE + state::LOCK) < crate::hud::LOCK_TICKS {
        play(hw, a, patch::LOCK, 0, 0, 0, OWN, now);
    }
}

/// Sound `sound` at (`u`, `v`), height `h` (1/16 texel; `OWN`: the player's) onto a pair, if it wins one.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn play<H: Hw + ?Sized>(hw: &mut H, a: &Agent, sound: u8, u: u16, v: u16, h: u16, flags: u8, now: u16) {
    let t = &PATCHES[sound as usize];
    let at = locate(a, u, v, h, flags);
    // (Its gain: the near ear's volume, + 1.)
    let g = if flags & OWN != 0 { 16 } else { at.0.max(at.1) + 1 };
    let score = (t[field::PRIORITY] as u16 * g as u16).min(255) as u8;
    if score == 0 {
        return;
    }
    // The pair with the least left (a free one: nothing). The player's own sounds keep to one pair (the
    // other is the world's): one goes onto the pair already playing one of them, unless that one is still
    // held (before its gate's release) and of a higher priority (a launch over the gun: the gun is back
    // once the launch's attack is over); and nothing else takes that pair from them.
    let (mut best, mut least) = (0u16, u16::MAX);
    for p in 0..PAIRS {
        let s = STATE + state::PAIRS + p * PAIR;
        let id = hw.peek(s + pair::PATCH);
        if id != 0 && !is_engine(id) && hw.peek(s + pair::FLAGS) & OWN != 0 {
            if flags & OWN == 0 {
                continue;
            }
            let on = &PATCHES[(id - 1) as usize % patch::ONE_SHOTS];
            // (From its age: `sounds` marks it released only after the frame's events.)
            if now.wrapping_sub(rd(hw, s + pair::START)) < on[field::GATE] as u16 && on[field::PRIORITY] > t[field::PRIORITY] {
                return;
            }
            (best, least) = (p, 0);
            break;
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
    store(hw, best, at);
    start(hw, best, sound, t, u, v, h, flags, now, score);
}

/// Sound `sound` (its patch `t`: from the caller's overlay) begun on pair `p` (its state: at (`u`, `v`),
/// height `h`, `flags`, `score`).
///
/// The IRQ's sweep off; the voices set up on both chips held silent (the test bit), the cutoffs and
/// volumes placed (snapped: no slew), then gated: the near ear's chip first, the far one's [`place`]'s lag later (raster
/// lines: [`wait`]), so the sound reaches the far ear later, as round a head. Then the sweep on.
#[allow(clippy::too_many_arguments)]
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
fn start<H: Hw + ?Sized>(hw: &mut H, p: u16, sound: u8, t: &[u8; PATCH], u: u16, v: u16, h: u16, flags: u8, now: u16, score: u8) {
    let s = STATE + state::PAIRS + p * PAIR;
    hw.poke(s + pair::PATCH, sound + 1);
    hw.poke(s + pair::FLAGS, flags);
    wr(hw, s + pair::U, u);
    wr(hw, s + pair::V, v);
    wr(hw, HEIGHT + 2 * p, h);
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
        hw.poke(c + 0x15, 0);
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
    let itd = set(hw, p, t, now, true);
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

/// Pair `p`'s two chips' volumes and cutoffs for where its sound is now ([`PLACED`]; `t` its patch), as
/// the IRQ's targets ([`SLEW`]); `snap`: the values too, onto the chips at once (a sound begun). The
/// ears' lag for there.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfxlib"))]
fn set<H: Hw + ?Sized>(hw: &mut H, p: u16, t: &[u8; PATCH], now: u16, snap: bool) -> i8 {
    let s = STATE + state::PAIRS + p * PAIR;
    let e = PLACED + p * PLACE;
    let (vl, vr, cl, cr, itd) = (hw.peek(e), hw.peek(e + 1), hw.peek(e + 2), hw.peek(e + 3), hw.peek(e + 4) as i8);
    // The cutoff swept a tick, then closed for the ear.
    let age = now.wrapping_sub(rd(hw, s + pair::START)).min(255) as i16;
    let open = (t[field::CUTOFF] as i16 + t[field::CUT_SWEEP] as i8 as i16 * age).clamp(0, 255) as u16;
    for chip in 0..core::hint::black_box(2u16) {
        let (vol, cut) = if chip == 0 { (vl, cl) } else { (vr, cr) };
        let (ch, v, c) = (SLEW + 4 * p + 2 * chip, t[field::MODE] | vol, ((open * cut as u16) >> 8) as u8);
        if snap {
            hw.poke(HOLD, 1);
            let r = LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP;
            hw.poke(ch + SLEW_CH, v);
            hw.poke(ch + SLEW_CH + 1, c);
            hw.poke(r + 0x18, v);
            hw.poke(r + 0x16, c);
        }
        hw.poke(ch, v);
        hw.poke(ch + 1, c);
        if snap {
            hw.poke(HOLD, 0);
        }
    }
    itd
}

/// The first frame: the four chips silent, filters low-pass, the state clear.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn init<H: Hw + ?Sized>(hw: &mut H, now: u16) {
    // (The slews first, each at its target: the IRQ leaves the chips be.)
    for ch in 0..SLEW_CH {
        let v = if ch & 1 == 0 { LP } else { 0 };
        hw.poke(SLEW + ch, v);
        hw.poke(SLEW + SLEW_CH + ch, v);
    }
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
