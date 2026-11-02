//! REU layout. Every 64 KiB REU block ("bank" = the `$DF06` register) holds one 256×256-texel tile of
//! the ground map, so a texel's REU address is `bank:v:u` — the floor DDA's integer parts go straight
//! into `$DF06/$DF05/$DF04` (`floor::texel_addr`).
//!
//! | banks | contents |
//! |---|---|
//! | 0-63 | ground map level 0: 2048×2048 texels, 8×8 tiles, tile (tx, ty) in bank `ty·8 + tx` |
//! | 64-79, 80-83, 84 | mip levels 1-3 (1024², 512², 256²), tiled the same way |
//! | 85 | the sea (one tile, shown off the map) |
//! | 86 | header |
//! | 87 / 88 / 89 | objects by grid cell / per-angle visibility lists / the scene block (`geom`, `scene`) |
//!
//! Everything from bank 90 up is free (room for object and enemy models, DLT5 music, ...).

/// 16 MiB REU (C64 Ultimate).
pub const REU_SIZE: u32 = 16 << 20;
/// 1764-compatible address wrap inside a DMA command.
pub const REU_BANK: u32 = 0x8_0000;
/// Longest single DMA (TALON HD: keeps raster IRQs on time).
pub const MAX_DMA: u16 = 256;

/// First bank of each mip level's tiles.
pub const LEVEL_BANK: [u8; 4] = [0, 64, 80, 84];
pub const SEA_BANK: u8 = 85;

/// Package version: the PRG refuses an image whose header carries another one (a PRG and REU from
/// different builds). Bump it whenever this layout changes.
pub const PACKAGE_VERSION: u16 = 1;

/// Header: magic[8], version u16.
pub const HEADER_BANK: u8 = 86;
pub const HEADER: u32 = (HEADER_BANK as u32) << 16;
pub const MAGIC: &[u8; 8] = b"CHOPPER1";
pub const H_VERSION: u32 = 8;

/// Objects, sorted by grid cell (`geom::scene::OBJECT` records), streamed per visible cell each frame.
pub const OBJ_BANK: u8 = 87;
pub const OBJECTS: u32 = (OBJ_BANK as u32) << 16;
/// Per camera angle (256 steps), the grid cells that can hold a visible object, far to near: a count, then
/// one byte per cell, the offset from the camera's cell (dx in the low nibble, dy in the high, signed).
pub const VIS_BANK: u8 = 88;
pub const VIS_STRIDE: u32 = 256;
pub const fn vis_list(angle: u8) -> u32 {
    ((VIS_BANK as u32) << 16) + angle as u32 * VIS_STRIDE
}

/// The 3D scene block (`geom::scene` layout): meshes and the grid directory, fetched whole at boot.
pub const SCENE_BANK: u8 = 89;
pub const SCENE: u32 = (SCENE_BANK as u32) << 16;
pub const SCENE_MAX: u16 = 0x0C00;

/// The image ends here: upload only this much.
pub const IMAGE_END: u32 = (SCENE_BANK as u32 + 1) << 16;

const _: () = assert!(LEVEL_BANK[1] == 64 && LEVEL_BANK[2] == 64 + 16 && LEVEL_BANK[3] == 80 + 4 && SEA_BANK == 84 + 1);

/// The four global colours: code 0 `$D021`, 1/2 screen RAM, 3 colour RAM (uniform over the view, so a
/// tilted horizon needs no colour changes): green grass, light-blue sky, brown earth, blue water.
pub const COLORS: [u8; 4] = [5, 14, 9, 6];
