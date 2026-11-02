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
 *   the boot is done. asm/audio.S runs once at boot from $C000 (loaded there with the PRG, after
 *   padding over $8300-$BFFF that the relocation then overwrites).
 * Zero page: the compiler's $02-$7F, fixed.S's $80-$9B, the kernels' $A0-$FF.
 * $01 = $35 (RAM everywhere except I/O); no ROM routines are used. */
__basic_zp_start = 0x0002;
__basic_zp_end = 0x0080;

MEMORY {
    ram (rw) : ORIGIN = 0x0801, LENGTH = 0x53FF
    /* Code where the VIC never looks, page-aligned and whole pages long (the boot copy moves pages). */
    lowram (rwx) : ORIGIN = 0x0200, LENGTH = 0x0200
    spanram (rwx) : ORIGIN = 0x8300, LENGTH = 0x0900   /* after the scene block (format::SCENE_MAX) */
    vicrom (rwx) : ORIGIN = 0x9B00, LENGTH = 0x0500    /* after the vertices; the VIC sees ROM there */
    /* llvm-mos's static stack (`.noinit`, the fixed frames of non-recursive functions): after the status
     * block, below the soft stack. Never loaded. */
    frames (rw) : ORIGIN = 0xCB00, LENGTH = 0x0200
    bandram (rwx) : ORIGIN = 0xAF00, LENGTH = 0x0900   /* after the band state, below the ray table */
    bandram2 (rwx) : ORIGIN = 0xBB00, LENGTH = 0x0400  /* after the ray table, below the line buffer */
    hiram (rwx) : ORIGIN = 0xF800, LENGTH = 0x0700     /* under the KERNAL, after the sprite chunk */
    /* Boot-only code, then the relocated code's load images. */
    boot (rx) : ORIGIN = 0x5C00, LENGTH = 0x2700
    /* Loaded with the PRG, never used: the relocated code is copied over it at boot. */
    gap (r) : ORIGIN = 0x8300, LENGTH = 0x3D00
    /* asm/audio.S, run once at boot, loaded in place: the floor's tiles and runs live here afterwards
     * (layout::TILES, SPANS, below RAYS). */
    audio (rx) : ORIGIN = 0xC000, LENGTH = 0x0640
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
        *(.text.floor_draw .text.geo_helpers .text.turbo_probe)
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

/* The SDK's charset switch (.init.250) calls KERNAL CHROUT, which is unmapped by the time it runs. */
SECTIONS { /DISCARD/ : { *(.init.250) } }

INCLUDE commodore.ld

__stack = 0xD000;

OUTPUT_FORMAT {
    SHORT(ORIGIN(ram))
    FULL(ram)
    FULL(boot)
    FULL(gap)
    TRIM(audio)
}
