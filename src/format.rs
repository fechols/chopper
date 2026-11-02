//! REU layout. Every 64 KiB REU block ("bank" = the `$DF06` register) holds one 256×256 plane, so a
//! texel's REU address is just `bank:v:u` — the DDA's integer parts go straight into `$DF06/$DF05/$DF04`.

/// 16 MiB REU (C64 Ultimate).
pub const REU_SIZE: u32 = 16 << 20;
/// 1764-compatible address wrap inside a DMA command.
pub const REU_BANK: u32 = 0x8_0000;
/// Longest single DMA (TALON HD: keeps raster IRQs on time).
pub const MAX_DMA: u16 = 256;

/// Plane size in texels (u and v are 8.8 fixed point and wrap here).
pub const MAP_SIZE: usize = 256;

/// The raw plane: one byte per texel, the 2-bit code replicated into all four pixel slots
/// (`code * $55`), so one DMA'd byte paints four pixels of one texel.
pub const MAP_BANK: u8 = 0;

/// Pre-sampled planes for the per-byte DMA path: the byte at `(u, v)` of variant `(angle, zoom)` already
/// holds the four texels a run of four pixels starting there would show, stepping along `angle` by
/// [`ZOOM_STEP`] texels per pixel. 48 directions × 5 zooms = 240 banks = 15 MiB of REU.
pub const ANGLES: u8 = 48;
pub const ZOOMS: u8 = 5;
/// Texels per multicolor pixel of each zoom bucket, in 8.8.
pub const ZOOM_STEP: [u16; ZOOMS as usize] = [0x40, 0x80, 0x100, 0x200, 0x400];
pub const PRE_BANK0: u8 = 1;
/// A bank of `$FF` bytes (code 3 everywhere): the sky above a floor's horizon.
pub const SKY_BANK: u8 = 255;

pub const fn pre_bank(angle: u8, zoom: u8) -> u8 {
    PRE_BANK0 + angle * ZOOMS + zoom
}

const _: () = assert!(PRE_BANK0 as u16 + ANGLES as u16 * ZOOMS as u16 <= SKY_BANK as u16);

/// Package version: the PRG refuses an image whose header carries another one (a PRG and REU from
/// different builds). Bump it whenever this layout changes.
pub const PACKAGE_VERSION: u16 = 8;

/// A bank of `$00` bytes (code 0, `$D021`): the haze between the horizon and the first lines whose step is
/// within the pre-sampled zooms.
pub const FOG_BANK: u8 = 253;

/// Objects, sorted by grid cell (`geom::scene::OBJECT` records), streamed per visible cell each frame.
pub const OBJ_BANK: u8 = 250;
pub const OBJECTS: u32 = (OBJ_BANK as u32) << 16;
/// Per camera angle (256 steps), the grid cells that can hold a visible object, far to near: a count, then
/// one byte per cell, the offset from the camera's cell (dx in the low nibble, dy in the high, signed).
pub const VIS_BANK: u8 = 251;
pub const VIS_STRIDE: u32 = 256;
pub const fn vis_list(angle: u8) -> u32 {
    ((VIS_BANK as u32) << 16) + angle as u32 * VIS_STRIDE
}

/// The 3D scene block (`geom::scene` layout): meshes and the grid directory, fetched whole at boot.
pub const SCENE_BANK: u8 = 252;
pub const SCENE: u32 = (SCENE_BANK as u32) << 16;
pub const SCENE_MAX: u16 = 0x0C00;

/// Header and camera path.
pub const HEADER_BANK: u8 = 254;
pub const HEADER: u32 = (HEADER_BANK as u32) << 16;
pub const MAGIC: &[u8; 8] = b"MODE7001";
/// Header offsets: magic[8], version u16, path entries u16, then the path.
pub const H_VERSION: u32 = 8;
pub const H_PATH_LEN: u32 = 10;
pub const H_PATH: u32 = 16;
/// Camera path: one entry per NTSC frame, looping: x and y (8.8 texels), angle (256 steps a turn).
pub const PATH_LEN: u16 = 2048;
pub const PATH_ENTRY: u32 = 5;

/// Per-angle line tables: for camera angle `a` (256 steps a turn) and the camera at the origin, 200
/// lines, field-major (9 arrays of 200 bytes): u0 lo/hi, v0 lo/hi (8.8 offsets from the camera, pixel 0),
/// du*4 lo/hi, dv*4 lo/hi (8.8 per 4 pixels, i.e. per byte), bank.
pub const LINE_BANK0: u8 = 241;
pub const LINES: usize = 200;
pub const LINE_FIELDS: usize = 9;
pub const LINE_BLOCK: u32 = (LINE_FIELDS * LINES) as u32;
pub mod field {
    pub const U0: usize = 0;
    pub const V0: usize = 2;
    pub const DU4: usize = 4;
    pub const DV4: usize = 6;
    pub const BANK: usize = 8;
}

/// Tables sit 2 KiB apart, so none straddles a 512 KiB bank (a single REU command must not cross one:
/// the 1764's address wraps there; TALON HD's allocator rule).
pub const LINE_STRIDE: u32 = 0x800;

pub const fn line_table(angle: u8) -> u32 {
    ((LINE_BANK0 as u32) << 16) + angle as u32 * LINE_STRIDE
}

const _: () = assert!(LINE_BLOCK <= LINE_STRIDE && REU_BANK % LINE_STRIDE == 0 && ((LINE_BANK0 as u32) << 16) % LINE_STRIDE == 0);

/// The floor: below the horizon (a cell-row boundary; colour RAM above it is the sky's).
pub const HORIZON: u16 = 48;
pub const FLOOR_ROW: u16 = HORIZON / 8;
pub const CAMERA_HEIGHT: f32 = 12.0;
pub const FOCAL: f32 = 160.0;

const _: () = assert!(line_table(255) + LINE_STRIDE <= (FOG_BANK as u32) << 16);
const _: () = assert!(HORIZON % 8 == 0);

/// The four global colours: code 0 `$D021` (grass), 1/2 screen RAM (tarmac, paint), 3 colour RAM (kerbs);
/// colour RAM above the horizon is the sky's.
pub const COLORS: [u8; 4] = [5, 12, 1, 2];
pub const SKY_COLOR: u8 = 14;
