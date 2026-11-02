//! Perspective projection, from H.A.V.O.C. (`sueme64/src/projection.rs`).
//!
//! `x' = 80 + 80·x/z` and `y' = CENTER_Y − 133·y/z`, computed as one 16×16
//! product per axis: z is normalised to `d ∈ 128..=255` with a signed exponent
//! `e` (`z ≈ d·2^e`), each axis has a 128-entry table of `scale·32768/d`, and
//! `>> (15 + e)` moves the binary point back. The 80:133 ratio compensates the
//! VIC-II's double-wide multicolor pixels, so the view is undistorted: about
//! 90° across and ±27° vertically in the 136-row viewport.
use crate::Vec3;
use crate::render::VIEW_HEIGHT;

/// Screen column of the optical axis.
pub const CENTER_X: i16 = 80;
/// Screen row of the optical axis: the middle of the 3D viewport.
pub const CENTER_Y: i16 = (VIEW_HEIGHT / 2) as i16;
/// Horizontal and vertical focal lengths in logical pixels.
pub const SCALE_X: u32 = 80;
pub const SCALE_Y: u32 = 133;
/// Projected coordinates are clamped to this magnitude (edge set-up stays in i32).
pub const CLAMP: i32 = 8191;

const fn projection_table(scale: u32) -> [u16; 128] {
    let mut table = [0; 128];
    let mut i = 0;
    while i < 128 {
        let d = i as u32 + 128;
        table[i] = ((scale * 32768 + d / 2) / d) as u16;
        i += 1;
    }
    table
}
/// `80·32768/d` for `d = 128 + index`.
pub static PROJECT_X: [u16; 128] = projection_table(SCALE_X);
/// `133·32768/d` for `d = 128 + index`.
pub static PROJECT_Y: [u16; 128] = projection_table(SCALE_Y);

/// Project a camera-space point with `z >= NEAR`.
pub fn project(point: Vec3) -> (i16, i16) {
    let mut d = point.z as u16;
    if d == 0 {
        return (CENTER_X, CENTER_Y);
    }
    let mut e = 0i8;
    while d > 255 {
        d >>= 1;
        e += 1;
    }
    while d < 128 {
        d <<= 1;
        e -= 1;
    }
    let shift = (15 + e) as u32;
    let x = (point.x as i32 * PROJECT_X[(d - 128) as usize] as i32) >> shift;
    // Negate before the shift so y rounds toward the top of the screen.
    let y = (-(point.y as i32) * PROJECT_Y[(d - 128) as usize] as i32) >> shift;
    (
        (CENTER_X as i32 + x).clamp(-CLAMP, CLAMP) as i16,
        (CENTER_Y as i32 + y).clamp(-CLAMP, CLAMP) as i16,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn projection_matches_exact_division() {
        for y in [i16::MIN, -30000, -1000, -1, 0, 1, 1000, 30000, i16::MAX] {
            for z in [crate::render::NEAR, 255, 256, 4000, 32767] {
                let (_, projected) = project(Vec3 { x: 0, y, z });
                let exact = (CENTER_Y as i64 - (y as i64 * 133) / z as i64).clamp(-8191, 8191) as i16;
                if exact.abs() == 8191 {
                    assert_eq!(projected, exact, "y {y} z {z}");
                } else {
                    assert!((projected - exact).abs() <= 2 + exact.abs() / 64, "y {y} z {z}: {projected} vs {exact}");
                }
            }
        }
    }
}
