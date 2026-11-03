/* Chopper C64 layout (from ../mode7/target-c64/link.ld, itself cut down from ../talonhd's).
 * - $0801-$5BFF: the resident program, data and BSS.
 * - The one frame buffer: screen $5C00, bitmap $6000 (VIC bank 1).
 * - $8000 the scene block, $8C00 the band being built and sprite scratch, $8E00 geometry arguments,
 *   $9000 reciprocals, $9800 vertices, $A000 the band state,
 *   $B800 the ray table, $BF00 the line buffer.
 * - $C000 tiles, runs, ray and walk arguments, kernel arguments, $C800 REU bounce / cell objects, $CA80 the
 *   status block, $CB00 llvm-mos's static frames; the soft stack grows down from $D000.
 * - $E000 triangles, $F100 multiply tables, $F500 the sprite chunk.
 * - Relocated code (copied there at boot by asm/startup.S, whole pages): $0200-$03FF, $8300-$8BFF,
 *   $9B00-$9FFF, $AF00-$B7FF, $BB00-$BEFF, $F800-$FEFF. Boot-only code (`.text.boot`) and the
 *   relocated code's load images sit in $5C00-$82FF, the frame buffer and the scene area, free until
 *   the boot is done. The REU overlays (`.text.ai`, `.text.hud`) run at $E000 and are not in the PRG
 *   (see `reuimage`); the cockpit overlay (`.text.cockpit`) runs and loads at $A000 (see `cockpit`). asm/audio.S runs once at boot from $C000 (loaded there with the PRG, after
 *   padding over $8300-$BFFF that the relocation then overwrites).
 * Zero page: the compiler's $02-$7F, fixed.S's $80-$9B, the kernels' $A0-$FF.
 * $01 = $35 (RAM everywhere except I/O); no ROM routines are used. */
__basic_zp_start = 0x0002;
__basic_zp_end = 0x0080;

MEMORY {
    /* Ends at the HUD's sprite blocks ($5A80-$5BFF, chopper::hud::SLOT_ADDR). */
    ram (rw) : ORIGIN = 0x0801, LENGTH = 0x527F
    /* The HUD's sprite blocks (written each frame): padding in the PRG. */
    hudblocks (r) : ORIGIN = 0x5A80, LENGTH = 0x0180
    /* Code where the VIC never looks, page-aligned and whole pages long (the boot copy moves pages). */
    lowram (rwx) : ORIGIN = 0x0200, LENGTH = 0x0200
    spanram (rwx) : ORIGIN = 0x8300, LENGTH = 0x0800   /* after the scene block (format::SCENE_MAX); then the HUD page $8B00 */
    vicrom (rwx) : ORIGIN = 0x9B00, LENGTH = 0x0500    /* after the vertices; the VIC sees ROM there */
    /* llvm-mos's static stack (`.noinit`, the fixed frames of non-recursive functions): after the status
     * block, below the soft stack ($CD80-$CFFF: tools/emu.py checks how much of it a run touches).
     * Never loaded. */
    frames (rw) : ORIGIN = 0xCB00, LENGTH = 0x0280
    bandram (rwx) : ORIGIN = 0xAF00, LENGTH = 0x0900   /* after the band state, below the ray table */
    bandram2 (rwx) : ORIGIN = 0xBB00, LENGTH = 0x0400  /* after the ray table, below the line buffer */
    hiram (rwx) : ORIGIN = 0xF800, LENGTH = 0x0700     /* under the KERNAL, after the sprite chunk */
    /* Boot-only code, then the relocated code's load images. */
    boot (rx) : ORIGIN = 0x5C00, LENGTH = 0x2700
    /* Loaded with the PRG, never used: the relocated code is copied over it at boot. */
    gap (r) : ORIGIN = 0x8300, LENGTH = 0x0900
    /* Free in the PRG (was the AI overlay's load image), between relocated pages. */
    aiload (r) : ORIGIN = 0x8C00, LENGTH = 0x0F00
    gap2 (r) : ORIGIN = 0x9B00, LENGTH = 0x0500
    /* Free in the PRG (was the cockpit overlay's load image; it is an REU overlay now, `.cocktext`). */
    cockpit (r) : ORIGIN = 0xA000, LENGTH = 0x0F00
    gap3 (r) : ORIGIN = 0xAF00, LENGTH = 0x1100
    /* asm/audio.S, run once at boot, loaded in place: the floor's tiles and runs live here afterwards
     * (layout::TILES, SPANS, below RAYS). */
    audio (rx) : ORIGIN = 0xC000, LENGTH = 0x0640
    /* REU overlays' load images: never in the PRG. `chopper bake --elf` copies each overlay's bytes from
     * the ELF into the REU (format::OVERLAY_AI, OVERLAY_HUD). */
    reuimage (r) : ORIGIN = 0x10000, LENGTH = 0x20000
    /* The REU overlays' windows: see `.aitext` and `.cocktext`. */
    window (rx) : ORIGIN = 0xE000, LENGTH = 0x1100
    window2 (rx) : ORIGIN = 0xA000, LENGTH = 0x0F00
}

/* Matched before commodore.ld's catch-all .text (first match wins). */
SECTIONS {
    .boottext : {
        *(.text.boot .text.boot.*)
    } >boot
    .lowtext : {
        *(.text.umul .text.geo_orow)
    } >lowram AT>boot
    .spantext : {
        *(.text.span_line .rodata.span_tables .text.geo_bin .text.band_publish .text.bt_build .text._ZN7chopper2hw5fetch*)
    } >spanram AT>boot
    .victext : {
        *(.text.floor_draw .text.geo_helpers .text.turbo_probe .text.hud_irq .text.hud_irq_enable)
    } >vicrom AT>boot
    .bandtext : {
        *(.text.geo_walk .text.geo_xform)
    } >bandram AT>boot
    .bandtext2 : {
        *(.text.geo_activate .text.ssetup .text.tri_row .rodata.tri_tables)
    } >bandram2 AT>boot
    .hitext : {
        *(.text.draw_cols_asm .rodata.copy_entry .rodata.line_tables .text.sprite_band .text.geo_draw .text.poly_row .rodata.poly_tables)
    } >hiram AT>boot
    /* The REU overlays, taking turns in one window over the frame's spent geometry (layout::WORK: the
     * triangles, depth-bucket heads and links and the clip list, all rewritten before they are read;
     * Work::clear refills the heads once they have run), each fetched in the frame's set-up:
     * - `.text.ai`: agent::Agent::fly and what it calls;
     * - `.text.hud`: hud::build and what it calls;
     * - `.text.geomsetup`: geom::walk_args (the grids' set-up for asm/geom.S geo_walk), before Work::clear;
     * - `.text.actors`: the frame's moving objects (actors);
     * - `.text.world`, `.text.world3`, `.text.world2`: the war's move, decide and fight phases, one a frame
     *   in turn (world_sim). */
    OVERLAY 0xE000 : AT (0x10000) {
        .aitext { *(.text.ai .text.ai.*) }
        .hudtext { *(.text.hud .text.hud.*) }
        .geomtext { *(.text.geomsetup .text.geomsetup.*) }
        .actorstext { *(.text.actors .text.actors.*) }
        .worldtext { *(.text.world .text.world.*) }
        .world2text { *(.text.world2 .text.world2.*) }
        .world3text { *(.text.world3 .text.world3.*) }
    } >window
    /* The second window, over the band state at $A000 (layout::BAND_RAM: spent from the last band to
     * geo_bin), REU overlays too:
     * - `.text.cockpit`: the gauges and the HUD's first and last steps, in the set-up;
     * - `.text.dyn`: the moving objects pushed into the depth buckets, after geo_walk. */
    OVERLAY 0xA000 : AT (0x20000) {
        .cocktext { *(.text.cockpit .text.cockpit.*) }
        .dyntext { *(.text.dyn .text.dyn.*) }
    } >window2
    .audiotext : {
        *(.text.audio_boot)
    } >audio
    .noinit (NOLOAD) : {
        *(.noinit .noinit.* NULL INIT ZPSAVE)
    } >frames
}
__lowtext_start = ADDR(.lowtext);
__lowtext_load_start = LOADADDR(.lowtext);
__lowtext_pages = (SIZEOF(.lowtext) + 255) / 256;
__spantext_start = ADDR(.spantext);
__spantext_load_start = LOADADDR(.spantext);
__spantext_pages = (SIZEOF(.spantext) + 255) / 256;
__victext_start = ADDR(.victext);
__victext_load_start = LOADADDR(.victext);
__victext_pages = (SIZEOF(.victext) + 255) / 256;
__bandtext_start = ADDR(.bandtext);
__bandtext_load_start = LOADADDR(.bandtext);
__bandtext_pages = (SIZEOF(.bandtext) + 255) / 256;
__bandtext2_start = ADDR(.bandtext2);
__bandtext2_load_start = LOADADDR(.bandtext2);
__bandtext2_pages = (SIZEOF(.bandtext2) + 255) / 256;
__hitext_start = ADDR(.hitext);
__hitext_load_start = LOADADDR(.hitext);
__hitext_pages = (SIZEOF(.hitext) + 255) / 256;
__aitext_size = SIZEOF(.aitext);
__hudtext_size = SIZEOF(.hudtext);
__geomtext_size = SIZEOF(.geomtext);
__actorstext_size = SIZEOF(.actorstext);
__worldtext_size = SIZEOF(.worldtext);
__world2text_size = SIZEOF(.world2text);
__world3text_size = SIZEOF(.world3text);
ASSERT(SIZEOF(.world3text) <= 0x1100, "the world's decide overlay is larger than its window ($E000-$F0FF)")
ASSERT(SIZEOF(.world2text) <= 0x1100, "the world's fight overlay is larger than its window ($E000-$F0FF)")
ASSERT(SIZEOF(.worldtext) <= 0x1100, "the world overlay is larger than its window ($E000-$F0FF)")
ASSERT(SIZEOF(.actorstext) <= 0x1100, "the actors overlay is larger than its window ($E000-$F0FF)")
ASSERT(SIZEOF(.geomtext) <= 0x1100, "the geometry set-up overlay is larger than its window ($E000-$F0FF)")
/* The window: up to the multiply tables (fixed::SQ_NEG_LO). */
ASSERT(SIZEOF(.aitext) <= 0x1100, "the AI overlay is larger than its window ($E000-$F0FF)")
ASSERT(SIZEOF(.hudtext) <= 0x1100, "the HUD overlay is larger than its window ($E000-$F0FF)")
__cocktext_size = SIZEOF(.cocktext);
__dyntext_size = SIZEOF(.dyntext);
ASSERT(SIZEOF(.cocktext) <= 0x0F00, "the cockpit overlay is larger than its window ($A000-$AEFF)")
ASSERT(SIZEOF(.dyntext) <= 0x0F00, "the moving objects' overlay is larger than its window ($A000-$AEFF)")

/* The SDK's charset switch (.init.250) calls KERNAL CHROUT, which is unmapped by the time it runs. */
SECTIONS { /DISCARD/ : { *(.init.250) } }

INCLUDE commodore.ld

__stack = 0xD000;

OUTPUT_FORMAT {
    SHORT(ORIGIN(ram))
    FULL(ram)
    FULL(hudblocks)
    FULL(boot)
    FULL(gap)
    FULL(aiload)
    FULL(gap2)
    FULL(cockpit)
    FULL(gap3)
    TRIM(audio)
}
