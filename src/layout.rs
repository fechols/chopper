//! Fixed C64 memory layout shared by the renderer and the host model. The buffers are TALON HD's
//! (`../talonhd/src/layout.rs`, vendored from gclass `a5775bf`); the texel cache is mode7's own.
//!
//! | Region            | Buffer A (VIC bank 1) | Buffer B (VIC bank 2) |
//! |-------------------|-----------------------|-----------------------|
//! | screen RAM        | `$5C00` (`$D018=$78`) | `$8C00` (`$D018=$38`) |
//! | bitmap            | `$6000`               | `$A000`               |
//!

/// Screen width in multicolor pixels.
pub const WIDTH: i16 = 160;
pub const HEIGHT: i16 = 200;
pub const COLS: u16 = 40;
pub const ROWS: u16 = 25;
pub const BITMAP_BYTES: u16 = 8000;
pub const CELLS: u16 = 1000;
/// Bytes from one cell row to the next in the bitmap.
pub const ROW_STRIDE: u16 = 320;
/// Lines of the bitmap the VIC shows in 24-row mode.
pub const VISIBLE: i16 = 192;

/// One displayable frame buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Buffer {
    pub screen: u16,
    pub bitmap: u16,
    /// Low two bits written to `$DD00` to show this buffer.
    pub dd00: u8,
    /// `$D018` for this buffer: its screen's offset in the VIC bank, bitmap at +`$2000`. The screens sit
    /// at different offsets: bank 2's VIC sees the character ROM at `$9000-$9FFF`, and bank 1 needs
    /// `$4000-$5BFF` for the program.
    pub d018: u8,
}

/// One visible buffer: the C64 draws a cell row (band) at a time into [`STAGE`] and publishes it when the
/// raster is elsewhere (sueme64's scheme), so a frame tears between bands but needs no second buffer.
pub const BUFFERS: [Buffer; 1] = [Buffer { screen: 0x5C00, bitmap: 0x6000, dd00: 0b10, d018: 0x78 }];

/// The band being drawn (320 bytes), published to the bitmap by `asm/floor.S` `band_publish`.
pub const STAGE: u16 = 0x8C00;
/// `asm/geom.S`'s band state: the slot pool of active triangles and the band lists (to `$AEFF`).
pub const BAND_RAM: u16 = 0xA000;
/// Sprite drawing (`sprite::sprite_band`, `asm/geom.S`): a frame record, `draw_cols`' job, a sprite band's
/// start and its on-screen column offsets, and its records (the chunk, in RAM under the KERNAL ROM).
pub const SPRITE_REC: u16 = 0x8D40;
pub const JOB: u16 = 0x8D70;
pub const SPRITE_BSTART: u16 = 0x8D80;
pub const SPRITE_OFFS: u16 = 0x8D90;
pub const SPRITE_CHUNK: u16 = 0xF500;
pub const SPRITE_CHUNK_LEN: u16 = 0x0280;
const _: () = assert!(STAGE as u32 + ROW_STRIDE as u32 <= SPRITE_REC as u32);
const _: () = assert!(SPRITE_OFFS as u32 + 2 * (COLS as u32 + 1) <= 0x9000);
const _: () = assert!(SPRITE_CHUNK as u32 + SPRITE_CHUNK_LEN as u32 <= 0xFFFA);

pub const COLOR_RAM: u16 = 0xD800;

/// Byte offset of screen line `y` (0..199) within a bitmap column.
#[inline(always)]
pub const fn line_offset(y: u16) -> u16 {
    (y >> 3) * ROW_STRIDE + (y & 7)
}

/// VIC registers for the floor screen: multicolour bitmap (`$D018` is per buffer, `Buffer::d018`).
pub const D011: u8 = 0x3B;
pub const D016: u8 = 0x18;

/// The floor (`floor`, `asm/floor.S`): one drawn line (40 bytes), that line's runs as `floor::make_line`
/// writes them (`floor::Span`, up to 40), and the four mip levels' tile → bank tables (`floor::tile_table`, 256 bytes
/// each, level `k` at `TILES + 256 k`). Lines are generated and drawn one at a time.
pub const LINEBUF: u16 = 0xBF00;
pub const TILES: u16 = 0xC000;
pub const SPANS: u16 = 0xC400;
pub const SPAN_LINE: u16 = (crate::floor::MAX_SPANS * crate::floor::SPAN_BYTES) as u16;
/// `asm/spans.S`'s arguments (`floor::SpanArgs`), and the frame's ray table (`floor::Rays::bt`, built by
/// `bt_build`: rows of 64 bytes, of which 41 are used; rows 0-2 hold bytes 0-2 of x, rows 8-10 of z).
pub const RAYS: u16 = 0xC640;
pub const BT: u16 = 0xB800;
const _: () = assert!(BT as u32 + 12 * 64 <= LINEBUF as u32);
const _: () = assert!(SPANS + SPAN_LINE <= RAYS && RAYS as u32 + core::mem::size_of::<crate::floor::SpanArgs>() as u32 <= POLY_ARGS as u32);
const _: () = assert!(TILES as u32 + 4 * 256 <= SPANS as u32);
/// `asm/poly.S`'s arguments (`poly::args`).
pub const POLY_ARGS: u16 = 0xC780;
/// `asm/tri.S`'s arguments (`geom::walk_args`).
pub const TRI_ARGS: u16 = 0xC7A0;

/// The scene block (`format::SCENE`), fetched at boot: VIC bank 2 below its screen.
pub const SCENE_RAM: u16 = 0x8000;
/// (Relocated code runs from `$8300`: target-c64/link.ld `spanram`.)
const _: () = assert!(SCENE_RAM as u32 + crate::format::SCENE_MAX as u32 <= 0x8300);
const _: () = assert!(STAGE as u32 + ROW_STRIDE as u32 <= 0x9000);
/// `fixed::RECIP`, the reciprocal table (2 KiB), and `geom::Verts` (one object's transformed vertices,
/// `asm/geom.S`) in `$9000-$9FFF`: RAM to the CPU, character ROM to the VIC, so no screen can go there.
const _: () = assert!(crate::fixed::RECIP == 0x9000);
pub const VERTS: u16 = 0x9800;
const _: () = assert!(VERTS as u32 + core::mem::size_of::<crate::geom::Verts>() as u32 <= BAND_RAM as u32);
/// `geom::Work`, the frame's triangles and depth buckets; `$01 = $35` keeps `$E000+` RAM.
pub const WORK: u16 = 0xE000;
const _: () = assert!(WORK as u32 + core::mem::size_of::<crate::geom::Work>() as u32 <= 0xFFFA);
const _: () = assert!(WORK as u32 + core::mem::size_of::<crate::geom::Work>() as u32 <= crate::fixed::SQ_NEG_LO as u32);
/// `asm/geom.S`'s arguments (`geom::geo_args`), and a row of the object directory being streamed
/// (`geom::draw`), after the sprite scratch.
pub const GEO_ARGS: u16 = 0x8E00;
/// `asm/geom.S` `geo_walk`'s arguments (`geom::walk`), after the ray arguments.
pub const WALK_ARGS: u16 = 0xC6A0;
const _: () = assert!(RAYS as u32 + core::mem::size_of::<crate::floor::SpanArgs>() as u32 <= WALK_ARGS as u32);
const _: () = assert!(WALK_ARGS as u32 + crate::geom::walk::SIZE as u32 <= POLY_ARGS as u32);
/// (Relocated code runs from `$9B00`: target-c64/link.ld `vicrom`.)
const _: () = assert!(VERTS as u32 + core::mem::size_of::<crate::geom::Verts>() as u32 <= 0x9B00);
pub const DIR_RAM: u16 = 0x8E40;
/// `asm/geom.S` `geo_xform`'s per-object table (|m| and signs, 27 bytes).
pub const XFORM_RAM: u16 = 0x8F00;
const _: () = assert!(DIR_RAM as u32 + 4 * (2 * crate::geom::REACH as u32 + 1) <= XFORM_RAM as u32);
const _: () = assert!(SPRITE_OFFS as u32 + 2 * (COLS as u32 + 1) <= GEO_ARGS as u32);
const _: () = assert!(GEO_ARGS + crate::geom::geo_args::SIZE <= DIR_RAM);
const _: () = assert!(DIR_RAM as u32 + 4 * (2 * crate::geom::REACH as u32 + 1) <= 0x9000);
/// `asm/floor.S`'s REU address shadows (`SAVE_MID`, `SAVE_BANK`).
pub const FLOOR_SHADOW: u16 = 0xC7D4;

/// `poly::PATTERNS` for `asm/geom.S`: 16 even-line bytes, then 16 odd-line bytes.
pub const PATS: u16 = 0xC7E0;

/// Bounce buffer for small REU reads (header, camera entry).
pub const BOUNCE: u16 = 0xC800;
/// One cell's objects as they are streamed in.
pub const CELL_RAM: u16 = 0xC910;
pub const CELL_MAX: u16 = 44;
const _: () = assert!(CELL_RAM + CELL_MAX * crate::geom::scene::OBJECT as u16 <= STATUS);
/// Status block the host reads (and pokes HOLD / MODE in) over the C64U's REST API.
pub const STATUS: u16 = 0xCA80;
pub const STATUS_MAGIC: u8 = 0xC1;

/// Offsets inside the [`STATUS`] block (little-endian).
pub mod status {
    /// u8: `$C1` once the main loop runs.
    pub const MAGIC: u16 = 0;
    /// u8: index of the buffer shown.
    pub const FRONT: u16 = 1;
    /// u16: the flight's tick (`agent::Agent::t`) of the frame shown, low half (high half: `TICK_HI`).
    pub const TICK: u16 = 2;
    /// u16: the time the last floor took to draw, in units of 4 µs (4 CIA ticks).
    pub const RENDER_US: u16 = 4;
    /// u8: set by the host to freeze the flight's clock (0 = run).
    pub const HOLD: u16 = 6;
    /// u8: 0 full, 1 line-doubled (fire on joystick 2 toggles it).
    pub const MODE: u16 = 7;
    /// u16: frames drawn since boot.
    pub const FRAMES: u16 = 8;
    /// u8: frames drawn in the last 60 NTSC frames (about a second).
    pub const FPS: u16 = 10;
    /// u16: turbo probes taken at boot.
    pub const PROBES: u16 = 12;
    /// u16: microseconds the frame's REU fetches and camera add took.
    pub const SETUP_US: u16 = 14;
    /// u16: microseconds the 3D geometry took (transform to last row written).
    pub const GEOM_US: u16 = 16;
    /// u8: triangles drawn; u8: objects in view.
    pub const TRIS: u16 = 18;
    pub const OBJECTS: u16 = 19;
    /// u8: pop-in size, pixels of bounding radius (`geom::Camera::pop`; poke it to tune; 0 = default).
    pub const POP: u16 = 20;
    pub const TICK_HI: u16 = 21;
    pub const SIZE: u16 = 23;
}

/// The attract-mode helicopter (`agent::Agent`, as `Agent::to_bytes`) and the flight's clock (u32: NTSC
/// frames since the sound started, frozen while `STATUS+HOLD` is set), after the status block; fixed so
/// `tools/emu.py --start` can poke them.
pub const AGENT: u16 = 0xCAA0;
pub const CLOCK: u16 = AGENT + crate::agent::Agent::BYTES as u16;
const _: () = assert!(STATUS + status::SIZE <= AGENT && CLOCK + 4 <= 0xCB00);
const _: () = assert!(core::mem::size_of::<crate::agent::Agent>() == crate::agent::Agent::BYTES);
