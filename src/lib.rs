//! Chopper — a Super Huey tribute for the 64 MHz C64 Ultimate with a 16 MiB REU: a helicopter over a
//! 2048×2048-texel "mode 7" ground map (one REU DMA per bitmap byte, any altitude, pitch and roll) with
//! streamed flat-shaded 3D objects. The renderer comes from `../mode7` (itself from TALON HD). Without the
//! `host` feature the core is `no_std`, for the C64 build.

#![cfg_attr(not(feature = "host"), no_std)]

pub mod agent;
pub mod brain;
pub mod camera;
pub mod cockpit;
pub mod cost;
pub mod fixed;
pub mod format;
pub mod geom;
pub mod hud;
pub mod hw;
pub mod layout;
pub mod floor;
pub mod poly;
pub mod sprite;

#[cfg(feature = "host")]
pub mod digital;
#[cfg(feature = "host")]
pub mod elf;
#[cfg(feature = "host")]
pub mod encode;
#[cfg(feature = "host")]
pub mod goap;
#[cfg(feature = "host")]
pub mod instruments;
#[cfg(feature = "host")]
pub mod model;
#[cfg(feature = "host")]
pub mod package;
#[cfg(feature = "host")]
pub mod palette;
#[cfg(feature = "host")]
pub mod png;
#[cfg(feature = "host")]
pub mod raster3d;
#[cfg(feature = "host")]
pub mod scene;
#[cfg(feature = "host")]
pub mod snapshot;
#[cfg(feature = "host")]
pub mod world;

#[cfg(all(test, feature = "host"))]
mod tests;
