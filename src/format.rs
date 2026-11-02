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

/// The object directory (`geom::scene::DIR_ENTRY` per cell: the grid's `GRID²`, then the big grid's
/// `BIG_GRID²`), and the object records (`geom::scene::OBJECT`, sorted by cell), streamed per visible
/// cell each frame.
pub const OBJ_DIR: u32 = 87 << 16;
pub const BIG_DIR: u32 = OBJ_DIR + 0x1000;
pub const OBJ_RECORDS: u32 = 88 << 16;

/// The 3D scene block (`geom::scene` layout): meshes and the grid directory, fetched whole at boot.
pub const SCENE_BANK: u8 = 89;
pub const SCENE: u32 = (SCENE_BANK as u32) << 16;
pub const SCENE_MAX: u16 = 0x0300;

/// Sprites (TALON HD's banded Run/Mix blocks, `encode`, drawn by `sprite`): the frame directory in bank
/// 90, the blocks in banks 91-95 (inside one 512 KiB region, so no DMA crosses one).
pub const SPRITE_DIR: u32 = 90 << 16;
pub const SPRITE_BLOCKS: u32 = 91 << 16;
pub const SPRITE_BLOCKS_END: u32 = 96 << 16;
const _: () = assert!(SPRITE_BLOCKS >> 19 == (SPRITE_BLOCKS_END - 1) >> 19);
/// Frame directory record (TALON HD's, plus the anchor): `FRAME_RECORD` bytes per frame.
pub const FRAME_RECORD: u32 = 40;
pub mod frame {
    /// u16 width (multicolour pixels), u16 height (lines).
    pub const W: usize = 0;
    pub const H: usize = 2;
    /// u8: stored phases, 4 (any x) or 1 (phase 0 only, drawn at `x & !3`).
    pub const PHASE_COUNT: usize = 4;
    /// u8: log2 of the band height in lines (`BAND_SHIFT_TALL` or `BAND_SHIFT_ONE`).
    pub const BAND_SHIFT: usize = 5;
    /// u8: number of bands.
    pub const BANDS: usize = 6;
    /// Four phase records (`PHASE_LEN` bytes each, see [`super::phase`]).
    pub const PHASES: usize = 8;
    pub const PHASE_LEN: usize = 6;
    /// u8, u8: the anchor (the point the sprite is placed by), pixels from the left, lines from the top.
    pub const AX: usize = 32;
    pub const AY: usize = 33;
}
pub mod phase {
    /// u8: number of byte columns.
    pub const WB: usize = 0;
    /// u24: REU address of the phase's block.
    pub const BLOCK: usize = 1;
    /// u16: the block's length in bytes (`0xFFFF`: that or more).
    pub const LEN: usize = 4;
}
/// TALON HD's block constants (`encode`): sprites up to `BAND_LINES` tall are one band, taller ones
/// 32-line bands; column record items are Run (`line lo, line hi, length` + bytes) or Mix (`line lo,
/// line hi | ITEM_MIX, bits, keep`).
pub const MAX_SPRITE_SIZE: u16 = 512;
pub const BAND_SHIFT_TALL: u8 = 5;
pub const BAND_LINES: u16 = 32;
pub const BAND_SHIFT_ONE: u8 = 9;
pub const ITEM_MIX: u8 = 0x80;
pub const RUN_HEADER: u16 = 3;
pub const MIX_LEN: u16 = 4;

/// The image ends here: upload only this much.
pub const IMAGE_END: u32 = SPRITE_BLOCKS_END;

const _: () = assert!(LEVEL_BANK[1] == 64 && LEVEL_BANK[2] == 64 + 16 && LEVEL_BANK[3] == 80 + 4 && SEA_BANK == 84 + 1);

/// The four global colours: code 0 `$D021`, 1/2 screen RAM, 3 colour RAM (uniform over the view, so a
/// tilted horizon needs no colour changes): green grass, light-blue sky, brown earth, blue water.
pub const COLORS: [u8; 4] = [5, 14, 9, 6];
