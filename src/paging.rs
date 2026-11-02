//! REU code paging (pattern: ../talonhd/src/paging.rs). Code that cannot all
//! be resident lives in the REU as *phases*, each linked at the same window
//! and fetched just before it runs.
//!
//! - `target-c64/link.ld` links every phase at [`WINDOW`]; `tools/phases.py`
//!   extracts the images to `dist/phases/<name>.bin` and stamps the PRG;
//!   `chopper package` puts them in the REU image at [`slot`].
//! - Phase code may call resident code only (never another phase), and
//!   resident code enters a phase only at a declared entry, after paging it
//!   in. `tools/check_phases.py` audits the linked program.
//! - Statics inside a phase are reset whenever it is paged in; persistent
//!   state is resident.

/// The window: `$4000-$BFFF` (VIC bank 3 holds the display).
pub const WINDOW: u16 = 0x4000;
pub const WINDOW_LEN: u16 = 0x8000;

/// Terrain walk, horizon, camera, projection and clipping.
pub const GEOMETRY: u8 = 0;
/// Depth sort, raster, cell packing and band publishing.
pub const PIXELS: u8 = 1;
pub const PHASES: u8 = 2;
/// Phase names (`dist/phases/<name>.bin`, `.phase.<name>` in link.ld), in id order.
pub const NAMES: [&str; PHASES as usize] = ["geometry", "pixels"];

/// REU address of phase `p`'s image (32 KiB slots; see `package`).
pub const fn slot(p: u8) -> u32 {
    crate::package::PHASES + p as u32 * WINDOW_LEN as u32
}

/// The stamp tying an REU image's phases to the PRG built with them.
pub fn stamp(images: &[&[u8]]) -> u16 {
    let mut s = 0x5A5Au16;
    for img in images {
        for &b in img.iter() {
            s = s.rotate_left(1).wrapping_add(b as u16);
        }
        s = s.wrapping_add(img.len() as u16);
    }
    s
}
