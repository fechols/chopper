//! The HUD: hi-res single-colour hardware sprites in light green over the 3D view, multiplexed by a
//! raster IRQ (`target-c64/asm/hud.S`). The IRQ mechanics come from TALON HD's helmet HUD
//! (`../talonhd/src/hud.rs`, `asm/hud.S`): rows of up to 8 sprites, each row set up by an IRQ that
//! fires once the previous row's sprites on the channels it reuses have ended. TALON HD's rows are a
//! fixed grid; here every sprite has its own X and Y, so the frame's virtual sprites are sorted by Y and
//! packed into rows ([`pack`]).
//!
//! What it shows (`update`; on the C64 `prepare` and `finish` run from the cockpit overlay, `build` from
//! the HUD overlay):
//! - the boresight;
//! - the current waypoint: a caret over it, or an arrow at the view's edge pointing toward it, and the
//!   `WPT` label with its range;
//! - target boxes on the targets in view;
//! - the missile seeker's diamond reticle. A target inside it is locked automatically: the diamond moves
//!   onto it and closes in over [`LOCK_TICKS`], then shows `LOCK`. One seeker for every target, in the
//!   air or on the ground.
//!
//! Timing (TALON HD, found on the C64U at 64 MHz): a sprite at Y shows on raster lines Y+1..Y+21, and the
//! VIC fetches its last line early in line Y+21, so a channel may be reprogrammed from line Y+22; a new
//! Y must be written before the VIC compares it, so a row's sprites start at least [`LEAD`] lines after
//! its IRQ. REU DMAs stall the CPU (and so the IRQ); the longest is 352 bytes (a cell's objects), about
//! 1.4 lines, inside the lead.
//!
//! Shared by the host and the C64: both build the same table bytes, block contents and lock state.

use crate::agent::Agent;
use crate::floor::FloorCam;
use crate::format::HUD_SHAPES;
use crate::hw::{self, Hw};

/// Sprite blocks in VIC bank 1 (64 bytes each): after the resident program, and after the bitmap.
pub const SLOT_ADDR: [u16; SLOTS] = [0x5A80, 0x5AC0, 0x5B00, 0x5B40, 0x5B80, 0x5BC0, 0x7F40, 0x7F80, 0x7FC0];
pub const SLOTS: usize = 9;
pub mod slot {
    pub const BORESIGHT: u8 = 0;
    pub const DIAMOND: u8 = 1;
    pub const LOCK: u8 = 2;
    pub const BOX: u8 = 3;
    pub const CARET: u8 = 4;
    pub const LABEL: u8 = 5;
    pub const RANGE: u8 = 6;
}
/// A slot's sprite pointer (its block's offset in the VIC bank / 64).
pub const fn pointer(slot: u8) -> u8 {
    ((SLOT_ADDR[slot as usize] & 0x3FFF) >> 6) as u8
}
/// The sprite pointers: screen RAM + `$3F8`.
pub const POINTERS: u16 = crate::layout::BUFFERS[0].screen + 0x3F8;
pub const COLOUR: u8 = 13;

/// Shapes in the REU (`format::HUD_SHAPES`, 64 bytes each).
pub mod shape {
    pub const BORESIGHT: u8 = 0;
    /// The seeker reticle with no target, then closing in on one (3 steps), then locked.
    pub const DIAMOND: u8 = 1;
    pub const ACQUIRE: u8 = 2;
    pub const LOCKED: u8 = 5;
    pub const LOCK: u8 = 6;
    pub const BOX: u8 = 7;
    /// Over the waypoint (tip at the bottom), then the edge arrows: N, NE, E, SE, S, SW, W, NW.
    pub const CARET: u8 = 8;
    pub const ARROW: u8 = 9;
    pub const LABEL: u8 = 17;
    /// Ranges: tenths of a kilometre 0.0-9.9, then whole kilometres 10-19.
    pub const RANGE: u8 = 18;
    pub const RANGES: u8 = 110;
    pub const COUNT: u16 = RANGE as u16 + RANGES as u16;
}
/// Each shape's anchor: the point placed on the screen position (hires px, sprite lines).
pub const fn anchor(shape: u8) -> (i16, i16) {
    if shape == shape::CARET { (12, 20) } else { (12, 10) }
}

/// The IRQ's state and tables (`asm/hud.S`): one page, after the relocated `spantext`.
pub const PAGE: u16 = 0x8B00;
/// u8: set by the main loop once the frame its table belongs to is on screen; the top IRQ then swaps
/// (and clears it). The next frame's `prepare` clears it too: a table not yet shown is replaced.
pub const READY: u16 = PAGE;
/// u8: the table the IRQ shows (0 or `$80` from `BUF`).
pub const FRONT: u16 = PAGE + 1;
/// u8: the next row's offset in its table (the IRQ's).
pub const NEXT: u16 = PAGE + 2;
/// u8: the table `update` wrote last (0 or `$80`).
pub const BACK: u16 = PAGE + 3;
/// The lock state: the target vehicle (u16, `world_sim::vehicle`; `NOBODY`: none), ticks inside the
/// reticle, the tick of the last update (u16); then the side the waypoint arrow last pointed to
/// (`CARET_SIDE`). Carried from frame to frame. (The vehicle, not its place in the target list: the list
/// is nearest first, and two vehicles of a formation swap places in it.)
pub const LOCK_STATE: u16 = PAGE + 4;
pub const LOCK_BYTES: usize = 6;
/// The lock state with no lock.
pub const NO_LOCK: [u8; LOCK_BYTES] = [0xFF, 0xFF, 0, 0, 0, 0];
const _: () = assert!(crate::world_sim::NOBODY == 0xFFFF);
/// u8: 1 when the waypoint was last to the right (its arrow keeps that side while it is near dead astern).
pub const CARET_SIDE: u16 = LOCK_STATE + 5;
const _: () = assert!(LOCK_STATE + LOCK_BYTES as u16 <= BUF);
pub const NONE: u8 = 0xFF;
pub const BUF: u16 = PAGE + 0x10;
pub const ROWS: usize = 4;
/// A row: its IRQ's line (0: no more rows), `$D010`, `$D015`, then 8 Ys, 8 X lows, 8 pointers.
pub const ROW: usize = 27;
pub mod row {
    pub const LINE: usize = 0;
    pub const MSB: usize = 1;
    pub const EN: usize = 2;
    pub const Y: usize = 3;
    pub const X: usize = 11;
    pub const PTR: usize = 19;
}
pub const TABLE: usize = ROWS * ROW;
const _: () = assert!(BUF as usize + 0x80 + TABLE <= PAGE as usize + 0x100);
/// The raster line of the IRQ that sets up row 0 (the top border).
pub const TOP_LINE: u8 = 20;
/// A row's sprites start this many lines after its IRQ (TALON HD's 2, plus one for the longest DMA).
pub const LEAD: u8 = 3;
/// A channel is free again this many lines after its sprite's Y.
pub const SPAN: u8 = 22;
/// Sprite Ys over the view: from the top border to the view's last line (the cockpit has none).
pub const Y_MIN: u8 = 30;
pub const Y_MAX: u8 = 50 + crate::floor::VIEW_LINES as u8 - 21;

/// Lock-on: ticks inside the reticle to lock; the reticle's half-size (hires px, lines); the range.
pub const LOCK_TICKS: u8 = 90;
pub const RETICLE_X: i16 = 24;
pub const RETICLE_Y: i16 = 20;
/// Quarter texels along the view.
pub const LOCK_RANGE: i16 = 250 * 4;
/// Targets: the most (`S_TARGETS`: count, then `TARGET` bytes each: u, v (i16 texels), centre height (u8
/// texels), the vehicle (u16, `world_sim::vehicle`)) and the range a box shows within (texels).
pub const TARGETS: usize = 12;
pub const TARGET: usize = 7;
pub const BOX_RANGE: i16 = 250;
pub const BOXES: usize = 8;

/// A sprite to show: VIC X and Y, block pointer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sprite {
    pub x: u16,
    pub y: u8,
    pub ptr: u8,
}

/// Up to this many sprites a frame.
pub const MAX: usize = 4 + BOXES + 2;

/// Pack `list` (any order) into the rows of `table`: sorted by Y; a sprite takes a channel of the
/// current row that is free by the row's IRQ line and starts at least `LEAD` lines after it, else
/// starts a new row (its IRQ at the earliest line a channel is free, at least 2 lines after the last
/// IRQ), moving the sprite down if it must; sprites that would leave the view or need a fifth row are
/// dropped. Returns how many were shown.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit"))]
pub fn pack(list: &mut [Sprite], table: &mut [u8; TABLE]) -> u8 {
    // Insertion sort by Y (stable).
    for i in 1..list.len() {
        let mut j = i;
        while j > 0 && list[j - 1].y > list[j].y {
            list.swap(j - 1, j);
            j -= 1;
        }
    }
    *table = [0; TABLE];
    let mut r = 0usize;
    let mut line = TOP_LINE;
    let mut used = 0u8;
    let mut free = [0u8; 8];
    let mut shown = 0;
    table[row::LINE] = TOP_LINE;
    for s in list.iter() {
        let mut y = s.y.max(Y_MIN);
        let mut c = 0;
        while c < 8 && (used >> c & 1 != 0 || free[c] > line || y < line + LEAD) {
            c += 1;
        }
        if c == 8 {
            // A new row: its IRQ when the earliest channel frees.
            if r + 1 == ROWS {
                continue;
            }
            c = 0;
            for k in 1..8 {
                if free[k] < free[c] {
                    c = k;
                }
            }
            let next = free[c].max(line + 2);
            y = y.max(next + LEAD);
            if y > Y_MAX {
                continue;
            }
            let (prev, cur) = table.split_at_mut((r + 1) * ROW);
            cur[..ROW].copy_from_slice(&prev[r * ROW..]);
            r += 1;
            line = next;
            used = 0;
            table[r * ROW + row::LINE] = line;
        }
        if y > Y_MAX {
            continue;
        }
        let t = &mut table[r * ROW..(r + 1) * ROW];
        t[row::Y + c] = y;
        t[row::X + c] = s.x as u8;
        t[row::PTR + c] = s.ptr;
        if s.x >= 256 {
            t[row::MSB] |= 1 << c;
        } else {
            t[row::MSB] &= !(1 << c);
        }
        t[row::EN] |= 1 << c;
        used |= 1 << c;
        free[c] = y + SPAN;
        shown += 1;
    }
    shown
}

/// `a * b`, exactly (the C64's 16 × 16 multiply).
#[inline(always)]
pub fn mul16(a: i16, b: i16) -> i32 {
    crate::fixed::mul(a, b)
}

/// The near plane, quarter texels.
pub const NEAR: i16 = 8;

/// The camera for the HUD's projections, once a frame: world → camera (Q14, 16-bit) and its position
/// (quarter texels).
#[derive(Clone, Copy, Debug)]
pub struct Eye {
    pub m: [i16; 9],
    pub p: [i16; 3],
}

impl Eye {
    #[inline(never)]
    // (Resident on the C64: shared by the overlays.)
    pub fn new(cam: &FloorCam) -> Self {
        let mut m = [0i16; 9];
        for i in 0..3 {
            for j in 0..3 {
                m[3 * j + i] = cam.r[i][j] as i16;
            }
        }
        Eye { m, p: [(cam.u >> 6) as i16, (cam.h >> 6) as i16, (cam.v >> 6) as i16] }
    }
}

/// Camera space (quarter texels: X right, Y up, Z ahead) of a world point (quarter texels: east, up,
/// north). 16-bit throughout: offsets across the whole map fit.
#[inline(never)]
// (Resident on the C64: shared by the overlays.)
pub fn to_camera(eye: &Eye, p: [i16; 3]) -> [i16; 3] {
    // Straight-line (no indexing loop: on the C64 it cost ~5000 cycles of bounds checks and copies).
    let (d0, d1, d2) = (p[0].wrapping_sub(eye.p[0]), p[1].wrapping_sub(eye.p[1]), p[2].wrapping_sub(eye.p[2]));
    let m = &eye.m;
    let row = |a: i16, b: i16, c: i16| ((mul16(a, d0) + mul16(b, d1) + mul16(c, d2)) >> 14) as i16;
    [row(m[0], m[1], m[2]), row(m[3], m[4], m[5]), row(m[6], m[7], m[8])]
}

/// Screen position (hires px, view line) of a camera-space point in front of the near plane:
/// `geom`'s projection (`sx = 80 + 80·X/Z`, `sy = 68 − 133·Y/Z`), in hires pixels across.
#[inline(never)]
// (Resident on the C64: shared by the overlays.)
pub fn screen(c: [i16; 3]) -> (i16, i16) {
    let z = c[2] as i32;
    let x = div_clamp(mul16(c[0], 160), z);
    let y = div_clamp(mul16(c[1], 133), z);
    (160 + x, 68 - y)
}

/// `(n / z).clamp(-1000, 1000)`: only quotients up to 1000 matter, so for `z > 0` a ten-step restoring
/// division of the magnitudes (an i32 divide is ~1900 cycles on the C64), truncated toward zero as `/` is.
/// On the C64 `asm/fixed.S` `m7_div_clamp`, for `0 < z < 32768` only: `screen`'s callers keep
/// `z ≥ NEAR`.
#[inline(never)]
// (Resident on the C64, with `screen`.)
fn div_clamp(n: i32, z: i32) -> i16 {
    #[cfg(target_arch = "mos")]
    unsafe {
        unsafe extern "C" {
            fn m7_div_clamp(n: i32, z: u16) -> i16;
        }
        m7_div_clamp(n, z as u16)
    }
    #[cfg(not(target_arch = "mos"))]
    div_clamp_ref(n, z)
}

/// [`div_clamp`] in Rust (the host's; what the C64's asm matches).
#[cfg_attr(target_arch = "mos", allow(dead_code))]
fn div_clamp_ref(n: i32, z: i32) -> i16 {
    if z <= 0 {
        return (n / z).clamp(-1000, 1000) as i16;
    }
    let m = n.unsigned_abs();
    // z is an i16's value, so z << 10 fits.
    let q = if m >= (z as u32) << 10 {
        1000
    } else {
        let (mut r, mut d, mut q, mut bit) = (m, (z as u32) << 9, 0u16, 512u16);
        while bit != 0 {
            if r >= d {
                r -= d;
                q |= bit;
            }
            d >>= 1;
            bit >>= 1;
        }
        q.min(1000)
    };
    if n < 0 { -(q as i16) } else { q as i16 }
}

#[cfg(test)]
#[test]
fn div_clamp_is_the_clamped_quotient() {
    let mut seed = 0x9E37_79B9u32;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed
    };
    let zs = [1i32, 2, 7, 8, 9, 100, 133, 1000, 4096, 32767];
    for &z in &zs {
        for base in [0i64, 1, 999, 1000, 1001, 1023, 1024, 1025, 40000] {
            for d in [-1i64, 0, 1] {
                for s in [1i64, -1] {
                    let n = (s * (base * z as i64 + d)).clamp(i32::MIN as i64 + 1, i32::MAX as i64) as i32;
                    assert_eq!(div_clamp(n, z), (n / z).clamp(-1000, 1000) as i16, "{n} / {z}");
                }
            }
        }
    }
    for _ in 0..200_000 {
        let z = (rnd() % 32767 + 1) as i32;
        let n = match rnd() % 3 {
            0 => rnd() as i32,
            1 => (rnd() % 2_000_000) as i32 - 1_000_000,
            _ => mul16(rnd() as i16, 160),
        };
        assert_eq!(div_clamp(n, z), (n / z).clamp(-1000, 1000) as i16, "{n} / {z}");
    }
}

/// Whether a screen position is inside the view (with a margin for the sprite).
#[inline(never)]
// (Resident on the C64: shared by the overlays.)
pub fn on_screen(x: i16, y: i16) -> bool {
    x >= 8 && x < 312 && y >= 4 && y < 132
}

/// A shape placed with its anchor at screen (`x` hires px, `y` view line).
#[inline(never)]
// (Resident on the C64: shared by the overlays.)
pub fn place(slot: u8, shape: u8, x: i16, y: i16) -> Sprite {
    let (ax, ay) = anchor(shape);
    let vx = (24 + x - ax).clamp(0, 343) as u16;
    let vy = (50 + y - ay).clamp(0, 255) as u8;
    Sprite { x: vx, y: vy, ptr: pointer(slot) }
}

/// The edge arrow toward a direction on the screen (`dx` right, `dy` down; any scale): its shape and
/// where it sits on the view's edge (stepped out from the centre to a 140 × 52 box).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.hud"))]
fn edge(mut dx: i16, mut dy: i16) -> (u8, i16, i16) {
    while dx > 7 || dx < -7 || dy > 7 || dy < -7 {
        dx >>= 1;
        dy >>= 1;
    }
    if dx == 0 && dy == 0 {
        dy = 1;
    }
    let (ax, ay) = (dx.abs(), dy.abs());
    // Octant: horizontal, vertical or diagonal (tan 22.5° ≈ 5/12).
    let o = if ax * 5 > ay * 12 {
        if dx > 0 { 2 } else { 6 }
    } else if ay * 5 > ax * 12 {
        if dy < 0 { 0 } else { 4 }
    } else if dx > 0 {
        if dy < 0 { 1 } else { 3 }
    } else if dy < 0 {
        7
    } else {
        5
    };
    let (mut x, mut y) = (160i16, 68i16);
    while (x - 160).abs() < 140 && (y - 68).abs() < 52 {
        x += dx;
        y += dy;
    }
    (shape::ARROW + o, x, y)
}

/// Distance in texels, roughly (max + 3/8 min: within 7%).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.hud"))]
fn distance(dx: i16, dy: i16) -> i16 {
    let (a, b) = (dx.abs(), dy.abs());
    let (hi, lo) = if a > b { (a, b) } else { (b, a) };
    hi + (lo * 3 >> 3)
}

/// The range readout's shape for a distance in texels (8 m each).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.hud"))]
pub fn range_shape(d: i16) -> u8 {
    let tenths = mul16(d, 41) >> 9;
    shape::RANGE + if tenths < 100 { tenths as u8 } else { 100 + ((mul16(d, 33) >> 12) - 10).clamp(0, 9) as u8 }
}

/// Scratch RAM for the frame's HUD while it is built (the band stage, free during the set-up): the
/// targets as read from the REU, the sprites, the shape of each block, the sprite count.
pub const SCRATCH: u16 = crate::layout::STAGE;
pub const S_TARGETS: u16 = SCRATCH;
pub const S_LIST: u16 = S_TARGETS + (1 + TARGETS * TARGET) as u16;
pub const S_SHAPES: u16 = S_LIST + (MAX * 4) as u16;
pub const S_COUNT: u16 = S_SHAPES + SHAPE_SLOTS as u16;
/// Blocks whose shapes change per frame (slots 0..).
pub const SHAPE_SLOTS: usize = 7;
const _: () = assert!(S_COUNT < crate::layout::STAGE + crate::layout::ROW_STRIDE);

/// The frame's HUD, step 1 (cockpit overlay): withdraw the last table if the IRQ has not taken it yet
/// (so the back table is free to rewrite without waiting for the top border: the frame it belonged to is
/// gone anyway). The targets are in `S_TARGETS` already (`combat::update`: the enemy units near).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit"))]
pub fn prepare<H: Hw + ?Sized>(hw: &mut H) {
    hw.poke(READY, 0);
}

/// Step 2 (HUD overlay): the sprites and shapes into the scratch RAM, the lock state advanced. No REU
/// access (so no soft stack on the C64).
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.hud"))]
pub fn build<H: Hw + ?Sized>(hw: &mut H, a: &Agent, cam: &FloorCam) {
    let eye = Eye::new(cam);
    let mut n = 0u8;
    for (k, s) in [shape::BORESIGHT, shape::DIAMOND, shape::LOCK, shape::BOX, shape::CARET, shape::LABEL, shape::RANGE].into_iter().enumerate() {
        hw.poke(S_SHAPES + k as u16, s);
    }
    put(hw, &mut n, place(slot::BORESIGHT, shape::BORESIGHT, 160, 68));
    put(hw, &mut n, Sprite { x: 24 + 4, y: 50 + 3, ptr: pointer(slot::LABEL) });
    put(hw, &mut n, Sprite { x: 24 + 28, y: 50 + 3, ptr: pointer(slot::RANGE) });
    // The waypoint: the caret over it, or an arrow at the edge.
    let (u, v) = a.at();
    let c = to_camera(&eye, [a.wu << 2, 0, a.wv << 2]);
    let mut s = shape::CARET;
    let (mut x, mut y) = (0, 0);
    if c[2] >= NEAR {
        (x, y) = screen(c);
    }
    // Its side: the last one kept unless it is clearly to one side (more than a quarter of its depth and
    // 2 texels off the axis): near dead astern the camera's smallest step would flip the arrow over.
    let band = (c[2].unsigned_abs() >> 2) as i16 + 8;
    let right = if c[0] > band { true } else if c[0] < -band { false } else { hw.peek(CARET_SIDE) != 0 };
    hw.poke(CARET_SIDE, right as u8);
    if c[2] < NEAR || !on_screen(x, y) {
        // Behind: toward its side.
        let (dx, dy) = if c[2] >= NEAR { (c[0] >> 2, -(c[1] >> 2)) } else { (if right { 127 } else { -127 }, 0) };
        (s, x, y) = edge(dx.saturating_mul(6), dy.saturating_mul(5));
    }
    hw.poke(S_SHAPES + slot::CARET as u16, s);
    put(hw, &mut n, place(slot::CARET, s, x, y));
    hw.poke(S_SHAPES + slot::RANGE as u16, range_shape(distance(a.wu - u, a.wv - v)));
    // The targets: boxes on those in range and view; the nearest inside the reticle is the seeker's.
    let count = hw.peek(S_TARGETS).min(TARGETS as u8);
    // The seeker keeps the vehicle it is on while that stays in the reticle (else, as the members of a
    // formation take turns at being nearest, its lock would start over and over).
    let rd = |hw: &mut H, at: u16| hw.peek(at) as u16 | (hw.peek(at + 1) as u16) << 8;
    let target = rd(hw, LOCK_STATE);
    let (mut best, mut best_z, mut bx, mut by) = (NONE, i16::MAX, 0, 0);
    let mut boxes = 0;
    for k in 0..count {
        let at = S_TARGETS + 1 + k as u16 * TARGET as u16;
        let tu = (hw.peek(at) as u16 | (hw.peek(at + 1) as u16) << 8) as i16;
        let tv = (hw.peek(at + 2) as u16 | (hw.peek(at + 3) as u16) << 8) as i16;
        if (tu - u).abs() > BOX_RANGE || (tv - v).abs() > BOX_RANGE {
            continue;
        }
        let c = to_camera(&eye, [tu << 2, (hw.peek(at + 4) as i16) << 2, tv << 2]);
        if c[2] < NEAR {
            continue;
        }
        let (x, y) = screen(c);
        if !on_screen(x, y) {
            continue;
        }
        let z = if rd(hw, at + 5) == target { i16::MIN } else { c[2] };
        if (x - 160).abs() <= RETICLE_X && (y - 68).abs() <= RETICLE_Y && c[2] <= LOCK_RANGE && z < best_z {
            (best, best_z, bx, by) = (k, z, x, y);
        }
        if boxes < BOXES {
            put(hw, &mut n, place(slot::BOX, shape::BOX, x, y));
            boxes += 1;
        }
    }
    // The seeker: the lock builds while the same vehicle stays in the reticle.
    let vid = if best == NONE { crate::world_sim::NOBODY } else { rd(hw, S_TARGETS + 1 + best as u16 * TARGET as u16 + 5) };
    let mut ticks = hw.peek(LOCK_STATE + 2);
    let last = rd(hw, LOCK_STATE + 3);
    let dt = (a.t as u16).wrapping_sub(last).min(255) as u8;
    ticks = if best == NONE || vid != target { 0 } else { ticks.saturating_add(dt).min(LOCK_TICKS) };
    hw.poke(LOCK_STATE, vid as u8);
    hw.poke(LOCK_STATE + 1, (vid >> 8) as u8);
    hw.poke(LOCK_STATE + 2, ticks);
    hw.poke(LOCK_STATE + 3, a.t as u8);
    hw.poke(LOCK_STATE + 4, (a.t >> 8) as u8);
    if best == NONE {
        put(hw, &mut n, place(slot::DIAMOND, shape::DIAMOND, 160, 68));
    } else {
        let s = if ticks >= LOCK_TICKS { shape::LOCKED } else { shape::ACQUIRE + ticks / 32 };
        hw.poke(S_SHAPES + slot::DIAMOND as u16, s);
        put(hw, &mut n, place(slot::DIAMOND, s, bx, by));
        if ticks >= LOCK_TICKS {
            put(hw, &mut n, place(slot::LOCK, shape::LOCK, bx, by + 14));
        }
    }
    hw.poke(S_COUNT, n);
}

/// Sprite `n` of the frame's list (scratch RAM), then the next.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.hud"))]
fn put<H: Hw + ?Sized>(hw: &mut H, n: &mut u8, s: Sprite) {
    let at = S_LIST + 4 * *n as u16;
    hw.poke(at, s.x as u8);
    hw.poke(at + 1, (s.x >> 8) as u8);
    hw.poke(at + 2, s.y);
    hw.poke(at + 3, s.ptr);
    *n += 1;
}

/// Step 3 (cockpit overlay): the shapes into their blocks, the sprites packed into the back table.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.cockpit"))]
pub fn finish<H: Hw + ?Sized>(hw: &mut H) {
    for k in 0..SHAPE_SLOTS {
        let id = hw.peek(S_SHAPES + k as u16);
        hw::fetch(hw, HUD_SHAPES + ((id as u32) << 6), SLOT_ADDR[k], 64);
    }
    let n = (hw.peek(S_COUNT) as usize).min(MAX);
    // (On the C64 in the overlay itself, rewritten each frame: not on the soft stack.)
    #[cfg(target_arch = "mos")]
    let (list, table) = unsafe { (&mut *(&raw mut LIST), &mut *(&raw mut TABLE_OUT)) };
    #[cfg(not(target_arch = "mos"))]
    let (list, table) = (&mut [Sprite::default(); MAX], &mut [0u8; TABLE]);
    for (i, s) in list[..n].iter_mut().enumerate() {
        let at = S_LIST + 4 * i as u16;
        *s = Sprite { x: hw.peek(at) as u16 | (hw.peek(at + 1) as u16) << 8, y: hw.peek(at + 2), ptr: hw.peek(at + 3) };
    }
    pack(&mut list[..n], table);
    let back = hw.peek(FRONT) ^ 0x80;
    for (i, &b) in table.iter().enumerate() {
        hw.poke(BUF + back as u16 + i as u16, b);
    }
    hw.poke(BACK, back);
}

#[cfg(target_arch = "mos")]
#[unsafe(link_section = ".text.cockpit.scratch")]
static mut LIST: [Sprite; MAX] = [Sprite { x: 0, y: 0, ptr: 0 }; MAX];
#[cfg(target_arch = "mos")]
#[unsafe(link_section = ".text.cockpit.scratch")]
static mut TABLE_OUT: [u8; TABLE] = [0; TABLE];

/// The whole frame's HUD (the host; the C64 runs the steps from their overlays).
pub fn update<H: Hw + ?Sized>(hw: &mut H, a: &Agent, cam: &FloorCam) {
    prepare(hw);
    build(hw, a, cam);
    finish(hw);
}

/// The IRQ's state at boot: no table ready, table 0 shown (empty), no lock.
#[inline(always)]
pub fn init<H: Hw + ?Sized>(hw: &mut H) {
    hw.fill(PAGE, 0x100, 0);
    for a in SLOT_ADDR {
        hw.fill(a, 64, 0);
    }
    hw.poke(BUF + row::LINE as u16, TOP_LINE);
    hw.poke(BUF + 0x80 + row::LINE as u16, TOP_LINE);
    hw.poke(LOCK_STATE, 0xFF);
    hw.poke(LOCK_STATE + 1, 0xFF);
}

// ------------------------------------------------------------------------------------- host: art

#[cfg(feature = "host")]
pub mod art {
    //! The shapes (24×21, 1 bit: TALON HD's `static_blocks` style) and the demo targets.
    use super::shape;

    type Block = [u8; 64];

    fn set(b: &mut Block, x: i32, y: i32) {
        if (0..24).contains(&x) && (0..21).contains(&y) {
            b[(y * 3 + x / 8) as usize] |= 0x80 >> (x % 8);
        }
    }

    /// 5×7 glyphs (bit 4 leftmost).
    fn glyph(c: char) -> [u8; 7] {
        match c {
            '0' => [0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E],
            '1' => [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
            '2' => [0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F],
            '3' => [0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E],
            '4' => [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
            '5' => [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
            '6' => [0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E],
            '7' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
            '8' => [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
            '9' => [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C],
            'C' => [0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E],
            'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
            'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F],
            'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
            'P' => [0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10],
            'T' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
            'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A],
            '.' => [0, 0, 0, 0, 0, 0, 0x04],
            _ => [0; 7],
        }
    }

    /// Text from `x` on rows `y..y+7` (6 px a character, 2 for '.').
    fn text(b: &mut Block, mut x: i32, y: i32, s: &str) {
        for c in s.chars() {
            let g = glyph(c);
            let (from, w) = if c == '.' { (2, 2) } else { (0, 6) };
            for (r, bits) in g.iter().enumerate() {
                for k in 0..5 {
                    if bits >> (4 - k) & 1 != 0 && k >= from && k < from + w {
                        set(b, x + k - from, y + r as i32);
                    }
                }
            }
            x += w;
        }
    }

    /// A diamond outline of radius `r` (hires px across, lines down) round (12, 10); `fill` for solid.
    fn diamond(b: &mut Block, r: i32, fill: bool) {
        for y in 0..21i32 {
            for x in 0..24i32 {
                let d = (x - 12).abs() + (y - 10).abs();
                if d == r || fill && d < r {
                    set(b, x, y);
                }
            }
        }
    }

    /// An arrow from (12, 10) pointing along (`dx`, `dy`): a chevron of two strokes behind its tip.
    fn arrow(b: &mut Block, dx: f32, dy: f32) {
        let n = (dx * dx + dy * dy).sqrt();
        let (ux, uy) = (dx / n, dy / n);
        let tip = (12.0 + ux * 8.0, 10.0 + uy * 8.0);
        for side in [-1.0f32, 1.0] {
            // Back from the tip at ±40°.
            let (c, s) = (0.766f32, 0.643 * side);
            let (bx, by) = (-(ux * c - uy * s), -(ux * s + uy * c));
            for k in 0..=9 {
                let t = k as f32;
                set(b, (tip.0 + bx * t).round() as i32, (tip.1 + by * t).round() as i32);
            }
        }
        for k in 0..=12 {
            let t = k as f32;
            set(b, (tip.0 - ux * t).round() as i32, (tip.1 - uy * t).round() as i32);
        }
    }

    /// Every shape, in `shape` order.
    pub fn shapes() -> Vec<Block> {
        let mut out = vec![[0u8; 64]; shape::COUNT as usize];
        // The boresight: a gun cross with a gap and a centre dot.
        let b = &mut out[shape::BORESIGHT as usize];
        for k in 4..=8 {
            set(b, 12 - k, 10);
            set(b, 12 + k, 10);
            set(b, 12, 10 - k);
            set(b, 12, 10 + k);
        }
        set(b, 12, 10);
        // The seeker: the reticle, closing in, locked.
        diamond(&mut out[shape::DIAMOND as usize], 10, false);
        for (i, r) in [8, 6, 5].into_iter().enumerate() {
            diamond(&mut out[(shape::ACQUIRE + i as u8) as usize], r, false);
        }
        let b = &mut out[shape::LOCKED as usize];
        diamond(b, 5, false);
        diamond(b, 2, true);
        text(&mut out[shape::LOCK as usize], 0, 7, "LOCK");
        // The target box: corner brackets, 18 × 15.
        let b = &mut out[shape::BOX as usize];
        for (cx, cy, sx, sy) in [(3, 3, 1, 1), (21, 3, -1, 1), (3, 17, 1, -1), (21, 17, -1, -1)] {
            for k in 0..4 {
                set(b, cx + sx * k, cy);
                set(b, cx, cy + sy * k);
            }
        }
        // The waypoint caret: a chevron pointing down at its tip (12, 20), over a short post.
        let b = &mut out[shape::CARET as usize];
        for k in 0..8 {
            set(b, 12 - k, 12 - k);
            set(b, 12 + k, 12 - k);
            set(b, 12 - k, 13 - k);
            set(b, 12 + k, 13 - k);
        }
        for y in 14..=20 {
            set(b, 12, y);
        }
        // The edge arrows: N, NE, E, SE, S, SW, W, NW (screen y down; pixels about square).
        for (i, (dx, dy)) in [(0.0, -1.0), (1.0, -1.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0), (-1.0, 0.0), (-1.0, -1.0)].into_iter().enumerate() {
            arrow(&mut out[(shape::ARROW + i as u8) as usize], dx, dy);
        }
        text(&mut out[shape::LABEL as usize], 0, 7, "WPT");
        for k in 0..shape::RANGES {
            let s = if k < 100 { format!("{}.{}", k / 10, k % 10) } else { format!("{}", k - 90) };
            text(&mut out[(shape::RANGE + k) as usize], 0, 7, &s);
        }
        out
    }

    /// The demo targets until the enemies come: the houses and hangars (centre heights), at most
    /// `TARGETS`, as `format::HUD_TARGETS` bytes.
    pub fn targets(world: &crate::world::World) -> Vec<u8> {
        let objs = crate::scene::objects(world);
        let mut t: Vec<u8> = Vec::new();
        let mut n = 0u8;
        for o in objs.iter().filter(|o| o.mesh == crate::scene::mesh::HOUSE || o.mesh == crate::scene::mesh::HANGAR).take(super::TARGETS) {
            t.extend_from_slice(&(o.u.round() as i16).to_le_bytes());
            t.extend_from_slice(&(o.v.round() as i16).to_le_bytes());
            t.push((o.y + 1.5).round() as u8);
            t.extend_from_slice(&crate::world_sim::NOBODY.to_le_bytes());
            n += 1;
        }
        t.insert(0, n);
        t
    }
}

// ------------------------------------------------------------------------------- host: compositing

/// The HUD as the VIC would show it over `rgb` (160 × 200 multicolour pixels, from `Machine::decode`):
/// per raster line, the sprite registers as the IRQs left them (from table `back`), each enabled sprite
/// covering lines Y+1..Y+21. Each hires pixel is half a multicolour pixel: a pixel is lit if either half is.
#[cfg(feature = "host")]
pub fn composite(m: &crate::model::Machine, back: u8, rgb: &mut [[u8; 3]]) {
    let colour = crate::palette::PALETTE[COLOUR as usize];
    let t = &m.ram[(BUF + back as u16) as usize..][..TABLE];
    for y in 0..200i32 {
        let raster = 51 + y;
        // The last row whose IRQ came before this line.
        let mut r = 0;
        while r + 1 < ROWS && t[(r + 1) * ROW + row::LINE] != 0 && (t[(r + 1) * ROW + row::LINE] as i32) < raster {
            r += 1;
        }
        let row = &t[r * ROW..(r + 1) * ROW];
        for c in 0..8 {
            if row[row::EN] >> c & 1 == 0 {
                continue;
            }
            let sy = row[row::Y + c] as i32;
            if raster <= sy || raster > sy + 21 {
                continue;
            }
            let line = raster - sy - 1;
            let x0 = row[row::X + c] as i32 + if row[row::MSB] >> c & 1 != 0 { 256 } else { 0 } - 24;
            let block = (row[row::PTR + c] as u16) << 6 | 0x4000;
            for k in 0..24 {
                let byte = m.ram[(block + (line * 3 + k / 8) as u16) as usize];
                if byte & (0x80 >> (k % 8)) != 0 {
                    let x = x0 + k;
                    if (0..320).contains(&x) {
                        rgb[(y * 160 + x / 2) as usize] = colour;
                    }
                }
            }
        }
    }
}
