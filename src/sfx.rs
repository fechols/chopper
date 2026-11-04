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
//! - **What sounds** ([`update`], once a frame after the effects pass): the frame's new effects (their
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
pub const PAIR: u16 = 9;
/// A pair's entry: its patch + 1 (0: free), flags, where (1/16 texel, u16 × 2), the tick it began (u16),
/// its score then.
pub mod pair {
    pub const PATCH: u16 = 0;
    pub const FLAGS: u16 = 1;
    pub const U: u16 = 2;
    pub const V: u16 = 4;
    pub const START: u16 = 6;
    pub const SCORE: u16 = 8;
}
/// Flags: its gates released; the player's own (centred, wherever the player flies); far off.
pub const RELEASED: u8 = 1;
pub const OWN: u8 = 2;
pub const FAR: u8 = 4;

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
    pub const COUNT: usize = 9;
}

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
pub static PATCHES: [[u8; PATCH]; patch::COUNT] = [
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

/// A sound's place for each ear: (left volume, right volume, left cutoff factor, right one): volumes
/// 0..=15, factors /256 of the patch's cutoff. `dx`, `dy`: from the player (texels; east, north),
/// `yaw` the player's heading (binary angle, 0 north, clockwise). `far`: the battle on the horizon.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub fn place(dx: i16, dy: i16, yaw: u16, far: bool) -> (u8, u8, u8, u8) {
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
        // (The sine table's step, Q14 in an i16: ±16384, so >> 10 is ±16.)
        let rel = ((crate::world_sim::heading_inline(dx, dy) as u16) << 8).wrapping_sub(yaw);
        ((crate::camera::sin(rel) as i16) >> 10, (crate::camera::sin(rel.wrapping_add(0x4000)) as i16) >> 10)
    };
    // Each ear: down to a quarter on the far side (`near`, `far` the ears' shares of the side, 0..=16).
    let (r, l) = (side.max(0) as u8, (-side).max(0) as u8);
    let vol = |s: u8| ((g as u16 * (16 - (s * 3 >> 2)) as u16 + 8) >> 4).min(15) as u8;
    // Muffled behind (to about 3/8 open), the far ear more (the head's shadow), and with distance.
    let air = (d >> 2).min(60) as u8 + if far { 40 } else { 0 };
    let open = 255u8.saturating_sub(((16 - ahead) as u8) * 5).saturating_sub(air);
    let cut = |s: u8| open.saturating_sub(s * 2).max(16);
    (vol(r), vol(l), cut(r), cut(l))
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

/// The frame's sounds: the new events onto the pairs, each pair playing placed again (see the module).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
pub fn update<H: Hw + ?Sized>(hw: &mut H, a: &Agent) {
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
    // The pairs playing: released, freed or placed again.
    for p in 0..PAIRS {
        let s = STATE + state::PAIRS + p * PAIR;
        let id = hw.peek(s + pair::PATCH);
        if id == 0 {
            continue;
        }
        let t = &PATCHES[(id - 1) as usize % patch::COUNT];
        let age = now.wrapping_sub(rd(hw, s + pair::START));
        if age >= t[field::LENGTH] as u16 {
            hw.poke(s + pair::PATCH, 0);
            continue;
        }
        let flags = hw.peek(s + pair::FLAGS);
        if age >= t[field::GATE] as u16 && flags & RELEASED == 0 {
            hw.poke(s + pair::FLAGS, flags | RELEASED);
            for chip in 0..core::hint::black_box(2u16) {
                for v in 0..3u16 {
                    hw.poke(LEFT + chip * (RIGHT - LEFT) + p * PAIR_STEP + 7 * v + 4, t[v as usize * VOICE + 2]);
                }
            }
        }
        set(hw, a, p, t, now);
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
        let left = if id == 0 {
            0
        } else {
            let len = PATCHES[(id - 1) as usize % patch::COUNT][field::LENGTH] as u16;
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
    let s = STATE + state::PAIRS + best * PAIR;
    hw.poke(s + pair::PATCH, sound + 1);
    hw.poke(s + pair::FLAGS, flags);
    wr(hw, s + pair::U, u);
    wr(hw, s + pair::V, v);
    wr(hw, s + pair::START, now);
    hw.poke(s + pair::SCORE, score);
    // The IRQ's sweep off; the voices restarted (gates off with the test bit, then on), the sweep on.
    let w = SWEEP + best * SWEEP_PAIR;
    hw.poke(w, 0);
    // (Rolled: unrolled, the two chips' writes doubled the overlay.)
    for chip in 0..core::hint::black_box(2u16) {
        let c = LEFT + chip * (RIGHT - LEFT) + best * PAIR_STEP;
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
            hw.poke(r + 4, t[o + 2] | (t[o + 2] != 0) as u8);
        }
    }
    for v in 0..3u16 {
        hw.poke(w + 1 + v, t[v as usize * VOICE]);
        hw.poke(w + 4 + v, t[v as usize * VOICE + 5]);
    }
    hw.poke(w, t[field::STEPS]);
    set(hw, a, best, t, now);
}

/// A sound's gain (0..=16) where it is.
#[inline(always)]
fn gain(a: &Agent, u: u16, v: u16, flags: u8) -> u8 {
    let (pu, pv) = a.at();
    let (dx, dy) = (((u >> 4) as i16).wrapping_sub(pu), ((v >> 4) as i16).wrapping_sub(pv));
    let (l, r, _, _) = place(dx, dy, a.yaw, flags & FAR != 0);
    l.max(r) + 1
}

/// Pair `p`'s two chips' volumes and cutoffs for where its sound is now (`t` its patch).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.sfx"))]
fn set<H: Hw + ?Sized>(hw: &mut H, a: &Agent, p: u16, t: &[u8; PATCH], now: u16) {
    let s = STATE + state::PAIRS + p * PAIR;
    let flags = hw.peek(s + pair::FLAGS);
    let (vl, vr, cl, cr) = if flags & OWN != 0 {
        (15, 15, 255, 255)
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
