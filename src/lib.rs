//! MODE 7 — an original stunt car racing game for the 64 MHz C64 Ultimate with a 16 MiB REU: 3D track
//! geometry over a SNES-style "mode 7" ground plane, in TALON HD's four global colours.
//!
//! Milestone 1 (this crate today): the ground plane, three ways, on a byte-exact model of the C64 + REU
//! with a time model from TALON HD's measured C64U unit costs. Without the `host` feature the core is
//! `no_std`, for the C64 build to come.

#![cfg_attr(not(feature = "host"), no_std)]

pub mod cost;
pub mod fixed;
pub mod format;
pub mod geom;
pub mod hw;
pub mod layout;
pub mod mode7;
pub mod poly;

#[cfg(feature = "host")]
pub mod camera;
#[cfg(feature = "host")]
pub mod map;
#[cfg(feature = "host")]
pub mod model;
#[cfg(feature = "host")]
pub mod package;
#[cfg(feature = "host")]
pub mod palette;
#[cfg(feature = "host")]
pub mod png;
#[cfg(feature = "host")]
pub mod scene;

#[cfg(all(test, feature = "host"))]
mod tests;
