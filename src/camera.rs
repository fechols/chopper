//! The free-flight camera: world → camera transform with full yaw, pitch and
//! roll. H.A.V.O.C. never rotated its camera; this is new.
//!
//! World axes: +X east, +Y up, +Z north. Camera axes: +X right, +Y up,
//! +Z forward. Angles are u16 binary angles (65,536 per turn). Positive yaw
//! turns right (clockwise seen from above), positive pitch raises the nose,
//! positive roll banks right (right side down).
//!
//! The matrix is Q14 (16,384 = 1.0). Every world point is transformed from its
//! camera-relative offset in one step, so a vertex shared by two terrain
//! patches always lands on the same camera-space point: no cracks from
//! per-patch rounding.

use crate::Vec3;

pub const ONE: i32 = 1 << 14;
/// Binary angle of a quarter turn.
pub const QUARTER_TURN: u16 = 0x4000;

const fn sin_table() -> [i16; 257] {
    // sin(i·π/512) for a quarter wave, Q14, via a Taylor series in f64.
    let mut table = [0i16; 257];
    let mut i = 0;
    while i <= 256 {
        let x = i as f64 * core::f64::consts::PI / 512.0;
        let x2 = x * x;
        let mut term = x;
        let mut sum = x;
        let mut n = 1;
        while n < 12 {
            term = -term * x2 / ((2 * n) as f64 * (2 * n + 1) as f64);
            sum += term;
            n += 1;
        }
        let v = sum * 16384.0 + 0.5;
        table[i] = v as i16;
        i += 1;
    }
    table
}

/// Quarter-wave sine, 1,024 steps per turn, Q14.
pub static SIN_Q14: [i16; 257] = sin_table();

/// Sine at table step `index` (1,024 steps per turn), Q14.
fn sin_step(index: u16) -> i32 {
    let p = (index & 255) as usize;
    match (index >> 8) & 3 {
        0 => SIN_Q14[p] as i32,
        1 => SIN_Q14[256 - p] as i32,
        2 => -(SIN_Q14[p] as i32),
        _ => -(SIN_Q14[256 - p] as i32),
    }
}

/// Sine of a binary angle, Q14, linearly interpolated between table steps so
/// slow turns rotate smoothly.
pub fn sin(angle: u16) -> i32 {
    let index = angle >> 6;
    let fraction = (angle & 63) as i32;
    let a = sin_step(index);
    let b = sin_step(index.wrapping_add(1));
    a + (((b - a) * fraction + 32) >> 6)
}

pub fn cos(angle: u16) -> i32 {
    sin(angle.wrapping_add(QUARTER_TURN))
}

/// A world position in world units (0.25 m), i32 per axis.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorldPos {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// Row-major Q14 3x3 matrix.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Matrix14 {
    pub m: [[i32; 3]; 3],
}

impl Matrix14 {
    pub const IDENTITY: Self = Self { m: [[ONE, 0, 0], [0, ONE, 0], [0, 0, ONE]] };

    pub fn mul(&self, other: &Self) -> Self {
        let mut m = [[0i32; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, entry) in row.iter_mut().enumerate() {
                // Q14 entries: each product is at most 2^28, so the sum fits i32.
                let sum = self.m[i][0] * other.m[0][j] + self.m[i][1] * other.m[1][j] + self.m[i][2] * other.m[2][j];
                *entry = (sum + (ONE >> 1)) >> 14;
            }
        }
        Self { m }
    }

    pub fn transpose(&self) -> Self {
        let m = self.m;
        Self { m: [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]] }
    }

    /// Orientation (body → world) from yaw, pitch and roll: R_yaw · R_pitch · R_roll.
    pub fn orientation(yaw: u16, pitch: u16, roll: u16) -> Self {
        let (sy, cy) = (sin(yaw), cos(yaw));
        let (sp, cp) = (sin(pitch), cos(pitch));
        let (sr, cr) = (sin(roll), cos(roll));
        let ry = Self { m: [[cy, 0, sy], [0, ONE, 0], [-sy, 0, cy]] };
        let rx = Self { m: [[ONE, 0, 0], [0, cp, sp], [0, -sp, cp]] };
        let rz = Self { m: [[cr, sr, 0], [-sr, cr, 0], [0, 0, ONE]] };
        ry.mul(&rx).mul(&rz)
    }

    /// `self · v` for |v| <= `REACH` per axis (so the sums fit i32); the
    /// result saturates to i16.
    pub fn apply(&self, v: [i32; 3]) -> Vec3 {
        let row = |r: &[i32; 3]| -> i16 {
            let sum = r[0] * v[0] + r[1] * v[1] + r[2] * v[2];
            ((sum + (ONE >> 1)) >> 14).clamp(i16::MIN as i32, i16::MAX as i32) as i16
        };
        Vec3 { x: row(&self.m[0]), y: row(&self.m[1]), z: row(&self.m[2]) }
    }
}

/// Camera-relative offsets beyond this magnitude are not transformed.
pub const REACH: i32 = 32_000;

#[derive(Clone, Copy, Debug)]
pub struct Camera {
    pub position: WorldPos,
    pub yaw: u16,
    pub pitch: u16,
    pub roll: u16,
    /// World → camera rotation (the transposed orientation).
    pub view: Matrix14,
}

impl Camera {
    pub fn new(position: WorldPos, yaw: u16, pitch: u16, roll: u16) -> Self {
        let view = Matrix14::orientation(yaw, pitch, roll).transpose();
        Self { position, yaw, pitch, roll, view }
    }

    /// World point → camera space, or `None` when it is beyond `REACH` on any
    /// axis (the caller culls it; i16 camera space cannot hold it).
    pub fn to_camera(&self, p: WorldPos) -> Option<Vec3> {
        let d = [p.x - self.position.x, p.y - self.position.y, p.z - self.position.z];
        if d.iter().any(|v| v.abs() > REACH) {
            return None;
        }
        Some(self.view.apply(d))
    }

    /// Camera-space direction of a world direction (no translation).
    pub fn rotate(&self, d: [i32; 3]) -> Vec3 {
        self.view.apply(d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sine_table_is_accurate() {
        assert_eq!(sin(0), 0);
        assert_eq!(sin(QUARTER_TURN), ONE);
        assert_eq!(sin(0x8000), 0);
        assert_eq!(sin(0xC000), -ONE);
        for a in (0..=u16::MAX).step_by(97) {
            let exact = (a as f64 / 65536.0 * core::f64::consts::TAU).sin() * 16384.0;
            assert!((sin(a) as f64 - exact).abs() <= 2.0, "angle {a}");
        }
    }

    #[test]
    fn level_camera_keeps_the_horizon_centred() {
        let cam = Camera::new(WorldPos { x: 1000, y: 400, z: -3000 }, 0x2345, 0, 0);
        // A point far ahead at eye height sits on the optical axis' row.
        let ahead = WorldPos {
            x: cam.position.x + (sin(cam.yaw) * 20000 >> 14),
            y: cam.position.y,
            z: cam.position.z + (cos(cam.yaw) * 20000 >> 14),
        };
        let v = cam.to_camera(ahead).unwrap();
        assert!(v.y.abs() <= 1, "{v:?}");
        assert!(v.x.abs() <= 2, "{v:?}");
        assert!((v.z - 20000).abs() <= 4, "{v:?}");
    }

    #[test]
    fn axes_follow_the_conventions() {
        let up = [0, 1000, 0];
        // Nose up: the world's up vector leans forward (+z), and a level
        // point ahead drops below the optical axis.
        let pitched = Camera::new(WorldPos::default(), 0, 0x0800, 0);
        assert!(pitched.rotate(up).z > 0);
        assert!(pitched.rotate([0, 0, 1000]).y < 0);
        // Bank right: the world's up vector appears to the left (-x).
        let banked = Camera::new(WorldPos::default(), 0, 0, 0x0800);
        assert!(banked.rotate(up).x < 0);
        // Yaw right (toward east): an eastern point appears straight ahead.
        let east = Camera::new(WorldPos::default(), QUARTER_TURN, 0, 0);
        let v = east.rotate([1000, 0, 0]);
        assert!(v.z > 990 && v.x.abs() < 2, "{v:?}");
    }

    #[test]
    fn view_is_orthonormal() {
        for (y, p, r) in [(0x1234, 0x0567, 0xF89A), (0x8000, 0x3000, 0x4000), (0xFFFF, 0xC800, 0x0101)] {
            let m = Matrix14::orientation(y, p, r);
            let product = m.mul(&m.transpose());
            for i in 0..3 {
                for j in 0..3 {
                    let expected = if i == j { ONE } else { 0 };
                    assert!((product.m[i][j] - expected).abs() <= 8, "{product:?}");
                }
            }
        }
    }
}
