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
//! | 90-95 | sprites |
//! | 96-106 | the rotor sound, DLT5 (`digital`) |
//! | 107 | the AI: decision trees, hazard map (`brain`, `goap`) |
//! | 108 | code overlays: the AI's and the HUD's baked from the C64 ELF, the cockpit's stashed at boot (`OVERLAYS`) |
//! | 109-114 | the cockpit: static panel, every gauge frame (`cockpit`, `instruments`) |
//! | 115 | the HUD: sprite shapes, demo targets (`hud`) |
//! | 116-117 | the living world: units and sectors, the roles' trees and learned bytes |
//! | 118 | the moving objects' meshes (`actors`) |
//! | 119, 120-145 | the effect sprites: frame directory, blocks (`effects`) |
//! | `$BFFF60`-`$D9C0FF` | not in the image: the rotor sound decoded to 8-bit PCM at boot |

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
pub const PACKAGE_VERSION: u16 = 3;

/// Header: magic[8], version u16.
pub const HEADER_BANK: u8 = 86;
pub const HEADER: u32 = (HEADER_BANK as u32) << 16;
pub const MAGIC: &[u8; 8] = b"CHOPPER1";
pub const H_VERSION: u32 = 8;
/// u32 × 3: the rotor sound's DLT5 length in bytes, and its loop (frames): the sampler plays from
/// frame 0 to the loop end, then repeats from the loop start (`digital::ROTOR_LOOP_*`).
pub const H_AUDIO_BYTES: u32 = 16;
pub const H_LOOP_START: u32 = 20;
pub const H_LOOP_END: u32 = 24;
/// What `asm/audio.S` needs to walk the DLT5 without dividing: 8-frame groups per segment (u16; the
/// first `H_SEG_LONG` segments have one more), the code stream's REU address (u32) and the groups in
/// all (u32).
pub const H_SEG_GROUPS: u32 = 28;
pub const H_SEG_LONG: u32 = 30;
pub const H_STREAM: u32 = 32;
pub const H_GROUPS: u32 = 36;
/// u32: the C64 build the REU overlays came from (its `CHOPPER_BUILD`; 0 when baked without one). The PRG
/// waits for an image with its own.
pub const H_BUILD: u32 = 40;

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

/// The rotor sound (DLT5, `digital::ROTOR`), from bank 96.
pub const AUDIO_DLT: u32 = 96 << 16;
pub const AUDIO_MAX: u32 = 0xB_0000;
/// Where it is decoded to (`asm/stream.S`): signed 8-bit PCM into a ring from `AUDIO_PCM`, `AUDIO_RING`
/// samples (the loop's length: `digital::ROTOR_LOOP_END - ROTOR_LOOP_START`, checked in `package`), sample
/// n at n mod `AUDIO_RING`; before it `HAAS` zero bytes the right voice starts in (10 ms at 16 kHz: the
/// Haas delay). Zeros in the image until the boot and the main loop decode into it.
pub const AUDIO_PCM: u32 = 0xC0_0000;
pub const AUDIO_RING: u32 = 412_138;
pub const HAAS: u32 = 160;
const _: () = assert!(AUDIO_PCM / REU_BANK == (AUDIO_PCM + AUDIO_RING - 1) / REU_BANK);
/// The decoder (asm/stream.S) keeps half the ring ahead of the playback, so half a lap short of it round
/// the ring: the frame clock's estimate of where the sampler is (it has no readback) may be out by ~16%
/// over the stream either way. It runs at most `STREAM_BUDGET` pages (256 samples) a call. (These four
/// are for the record: asm/stream.S has its own copies.)
pub const STREAM_MARGIN: u32 = AUDIO_RING / 2;
pub const STREAM_BUDGET: u8 = 8;
/// Samples the sampler plays a tick, as the decoder paces it (16,000 Hz / 60 Hz is 266.7, the sampler
/// calibrated to 16 kHz at boot: the estimate runs a little early, well inside the margin).
pub const SAMPLES_PER_TICK: u32 = 267;
/// The sampler's clock divider, nominal (about 6.25 MHz / 16 kHz): the calibration's start and fallback.
pub const AUDIO_DIVIDER: u16 = 390;
const _: () = assert!(SPRITE_BLOCKS_END <= AUDIO_DLT && AUDIO_DLT + AUDIO_MAX <= AUDIO_PCM - HAAS);

/// The AI (bank 107): the decision trees (`brain`, baked by `goap::compile`), then the hazard map the
/// agents probe (`agent`, baked by `goap::hazard_map`): `HAZARD_CELLS`² bytes, row-major by v, one per
/// `1 << HAZARD_SHIFT`-texel cell: the highest obstacle top over the cell in texels (with margins),
/// bit 7 set over the sea.
pub const AI_BANK: u8 = 107;
pub const AI_TREES: u32 = (AI_BANK as u32) << 16;
pub const HAZARD: u32 = AI_TREES + crate::brain::MAX_BYTES as u32;
pub const HAZARD_CELLS: u16 = 64;
pub const HAZARD_SHIFT: u32 = 5;
const _: () = assert!(AUDIO_DLT + AUDIO_MAX <= AI_TREES && HAZARD + (HAZARD_CELLS as u32).pow(2) <= AI_TREES + 0x10000);
const _: () = assert!((HAZARD_CELLS as u32) << HAZARD_SHIFT == crate::floor::MAP_TEXELS as u32);

/// Code overlays, fetched into their windows each frame, all baked into the image from the C64 build's ELF
/// (`chopper bake --elf`, `elf`).
pub const OVERLAYS: u32 = 108 << 16;
pub const OVERLAY_AI: u32 = OVERLAYS;
pub const OVERLAY_COCKPIT: u32 = OVERLAYS + 0x2000;
pub const OVERLAY_HUD: u32 = OVERLAYS + 0x4000;
/// The near-plane clipping's (`geom::clip`), in the main window's top above the triangle work.
pub const OVERLAY_CLIP: u32 = OVERLAYS + 0x3000;
pub const OVERLAY3_MAX: u32 = 0x07F0;
const _: () = assert!(OVERLAY_COCKPIT + OVERLAY2_MAX <= OVERLAY_CLIP && OVERLAY_CLIP + OVERLAY3_MAX <= OVERLAY_HUD);
pub const OVERLAY_GEOM: u32 = OVERLAYS + 0x6000;
pub const OVERLAY_ACTORS: u32 = OVERLAYS + 0x8000;
pub const OVERLAY_DYN: u32 = OVERLAYS + 0xA000;
pub const OVERLAY_WORLD: u32 = OVERLAYS + 0xC000;
pub const OVERLAY_WORLD2: u32 = OVERLAYS + 0xE000;
/// (Bank 108 is full: the decide phase's overlay is after the brains, in bank 117.)
pub const OVERLAY_WORLD3: u32 = TREES + 0x8000;
pub const OVERLAY_COMBAT: u32 = TREES + 0xA000;
pub const OVERLAY_FX: u32 = TREES + 0xC000;
pub const OVERLAY_COMBATLIB: u32 = TREES + 0xE000;
/// The carrier's deck (the second window, with the AI), after the director's.
pub const OVERLAY_DECK: u32 = TREES + 0x3800;
const _: () = assert!(OVERLAY_WORLD4 + OVERLAY_MAX <= OVERLAY_DECK && OVERLAY_DECK + OVERLAY2_MAX <= OVERLAY_WORLD3);
/// The rotor sound's decoder (`.text.stream`, asm/stream.S), after the deck's.
pub const OVERLAY_STREAM: u32 = TREES + 0x4800;
pub const STREAM_MAX: u32 = 0x1100;
const _: () = assert!(OVERLAY_DECK + OVERLAY2_MAX <= OVERLAY_STREAM && OVERLAY_STREAM + STREAM_MAX <= OVERLAY_WORLD3);
/// The war director's (once a lap), after the brains.
pub const OVERLAY_WORLD4: u32 = TREES + 0x2000;
const _: () = assert!(TREES + crate::brain::UNIT_BYTES as u32 <= OVERLAY_WORLD4 && OVERLAY_WORLD4 + OVERLAY_MAX <= OVERLAY_WORLD3);
/// The second window's overlays (`layout::BAND_RAM`, while the band state is spent).
pub const OVERLAY2_MAX: u32 = 0x0F00;
/// The largest REU overlay (its window: `layout::WORK` to the relocated code at `$F800`).
pub const OVERLAY_MAX: u32 = 0x1800;

/// The cockpit (`cockpit`, baked by `instruments`): the static panel, then every frame of every gauge.
pub const COCKPIT: u32 = 109 << 16;
const _: () = assert!(OVERLAYS + 0x10000 <= COCKPIT);

/// The HUD (`hud`): its sprite shapes (64 bytes each), then the demo targets.
pub const HUD_SHAPES: u32 = 115 << 16;
pub const HUD_TARGETS: u32 = HUD_SHAPES + 0x8000;
const _: () = assert!(crate::cockpit::END <= HUD_SHAPES && (crate::hud::shape::COUNT as u32) << 6 <= HUD_TARGETS - HUD_SHAPES);

/// The living world (`world_sim`): units and sectors (116), the roles' trees with their learned bytes
/// (117). The moving objects' meshes (`actors`, `scene::vehicle_bank`): 118.
pub const WORLD: u32 = 116 << 16;
pub const TREES: u32 = 117 << 16;
pub const VEHICLES: u32 = 118 << 16;

/// The effect sprites (`effects`, baked by `effects_bake`): the frame directory (`FRAME_RECORD` bytes per
/// frame, `effects::record`) from bank 120 (a 512 KiB region's start), the blocks (each inside one 512
/// KiB region) from bank 125 up to the rotor sound's ring, then on above it (`EFFECTS_HIGH`) to the end of
/// the REU. (Bank 119 is free.)
pub const EFFECTS_DIR: u32 = 120 << 16;
pub const EFFECTS_BLOCKS: u32 = 125 << 16;
pub const EFFECTS_LOW_END: u32 = AUDIO_PCM - HAAS;
/// Past the rotor sound's ring.
pub const EFFECTS_HIGH: u32 = 199 << 16;
const _: () = assert!(AUDIO_PCM + AUDIO_RING <= EFFECTS_HIGH);
pub const EFFECTS_BLOCKS_END: u32 = REU_SIZE;
const _: () = assert!(VEHICLES + 0x10000 <= EFFECTS_DIR && EFFECTS_DIR % REU_BANK == 0);

/// The image ends here: upload this much. (The rotor sound's ring in it is zeros until it is decoded.)
pub const IMAGE_END: u32 = EFFECTS_BLOCKS_END;

const _: () = assert!(LEVEL_BANK[1] == 64 && LEVEL_BANK[2] == 64 + 16 && LEVEL_BANK[3] == 80 + 4 && SEA_BANK == 84 + 1);

/// The four global colours: code 0 `$D021`, 1/2 screen RAM, 3 colour RAM (uniform over the view, so a
/// tilted horizon needs no colour changes): green grass, light-blue sky, brown earth, blue water.
pub const COLORS: [u8; 4] = [5, 14, 9, 6];
