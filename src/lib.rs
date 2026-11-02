//! Chopper: a free-flight, flat-shaded helicopter sim for the 64 MHz C64
//! Ultimate with a 16 MiB REU, in tribute to Super Huey. The renderer core
//! comes from H.A.V.O.C. (`../sueme64`).
#![no_std]

#[cfg(any(feature = "host", test))]
extern crate std;

pub mod camera;
pub mod horizon;
pub mod math;
pub mod package;
pub mod paging;
pub mod projection;
pub mod render;
pub mod terrain;

#[cfg(feature = "host")]
pub mod png;
#[cfg(feature = "host")]
pub mod screen;
#[cfg(feature = "host")]
pub mod worldgen;

/// A camera-space point in integer world units.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct Vec3 {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0, y: 0, z: 0 };
}

/// Geometry whose pixels exterior terrain must not overwrite (objects that
/// rest on or sink into the ground).
pub const TERRAIN_CLEAR_GEOMETRY: u8 = 0x20;
/// Terrain: sorted with a depth bias and kept behind terrain-clear geometry.
pub const EXTERIOR_TERRAIN: u8 = 0x40;

/// Receives camera-space triangles (clockwise on screen = front face).
pub trait GeometrySink {
    fn triangle(&mut self, a: Vec3, b: Vec3, c: Vec3, color: u8, flags: u8);
    fn checkpoint(&self) -> Option<render::Checkpoint> {
        None
    }
    fn rollback(&mut self, _: render::Checkpoint) {
        panic!("geometry sink does not support transactions");
    }
}
