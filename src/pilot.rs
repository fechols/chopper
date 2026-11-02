//! The demo autopilot: a deterministic flight from a frame tick, shared by the host and the C64 (same
//! bits), so `tools/emu.py` can check any frame against the host. A banked circle round the island's
//! centre with altitude, pitch and roll swinging, to exercise every part of the floor.

use crate::camera::{Matrix14, cos, sin};
use crate::floor::FloorCam;

/// Ticks per lap.
pub const LAP: u16 = 4096;

/// One degree in binary angle units (65,536 per turn).
const DEG: i32 = 182;

/// The flight state at `tick`: the floor camera, and its yaw (for the objects).
pub fn autopilot(tick: u16) -> (FloorCam, u16) {
    let a = tick.wrapping_mul((65536 / LAP as u32) as u16);
    // Round (1024, 1024) at a radius of 300 texels, clockwise from above: heading = a + 90°.
    let u = (1024 << 8) + (sin(a) * 300 * 256 >> 14);
    let v = (1024 << 8) + (cos(a) * 300 * 256 >> 14);
    let yaw = a.wrapping_add(0x4000);
    // Altitude 14 ± 8 texels (8.8), pitch −10° ± 5°, roll 20° ± 15°.
    let h = (14 * 256 + (sin(a.wrapping_mul(3)) * 8 * 256 >> 14)) as u16;
    let pitch = ((-10 * DEG) + (sin(a.wrapping_mul(2)) * 5 * DEG >> 14)) as u16;
    let roll = ((20 * DEG) + (sin(a.wrapping_mul(5)) * 15 * DEG >> 14)) as u16;
    let m = Matrix14::orientation(yaw, pitch, roll);
    (FloorCam { u, v, h, r: m.m }, yaw)
}
