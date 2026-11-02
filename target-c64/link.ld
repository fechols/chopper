/* Chopper C64 layout, with REU code paging in the style of ../talonhd (see src/paging.rs).
 *
 *   $0801-$3FFF  resident: main loop, REU/clock helpers, compiler runtime, the renderer's command
 *                list and the terrain sample cache
 *   $4000-$BFFF  the paging window: one phase at a time (GEOMETRY, PIXELS), fetched from the REU
 *   $0200-$023F  status block (tools/c64u.py status)
 *   $0240-$07FF  soft stack (grows down from $0800)
 *   $C000-$C3E7  screen matrix (VIC bank 3)
 *   $C400-$CFFF  llvm-mos static frames (.noinit)
 *   $D800        colour RAM
 *   $E000-$FF3F  multicolor bitmap (VIC bank 3; the CPU sees RAM because KERNAL is banked out)
 * $01 = $35 (RAM everywhere except I/O); no ROM routines are used. */
__basic_zp_start = 0x0002;
__basic_zp_end = 0x0090;

MEMORY {
    ram (rw) : ORIGIN = 0x0801, LENGTH = 0x37FF
    frames (rw) : ORIGIN = 0xC400, LENGTH = 0x0C00
    /* Every phase is linked at the window; its load image lives outside the PRG (phimg) and reaches
     * the REU through tools/phases.py + `chopper package`. */
    ph_geometry (rwx) : ORIGIN = 0x4000, LENGTH = 0x8000
    ph_pixels (rwx) : ORIGIN = 0x4000, LENGTH = 0x8000
    phimg (r) : ORIGIN = 0x10000, LENGTH = 0x20000
}

/* First match wins, before commodore.ld's catch-all .text. Library modules used by only one phase go
 * to that phase; anything shared (math, compiler runtime) stays resident. */
SECTIONS {
    .phase.geometry : {
        KEEP(*(.phase.geometry.entry))
        *(.text._ZN7chopper7terrain*)
        *(.text._ZN7chopper7horizon*)
        *(.text._ZN7chopper6camera*)
        *(.text._ZN7chopper10projection*)
        *(.text._ZN7chopper6render8Renderer18triangle_reference*)
        *(.text._ZN7chopper6render8Renderer16project_triangle*)
        *(.text._ZN7chopper6render8Renderer8backdrop*)
        *(.text._ZN7chopper6render17near_intersection*)
        *(.text.*GeometrySink$GT$8triangle*)
        *(.text._ZN11chopper_c649autopilot*)
        *(.text._ZN11chopper_c6410ReuTerrain6record*)
        *(.text.*ReuTerrain$u20$as$u20$chopper..terrain..TerrainData*)
    } >ph_geometry AT>phimg
    .phase.pixels : {
        KEEP(*(.phase.pixels.entry))
        *(.text._ZN7chopper6render*)
        *(.text._ZN11chopper_c647publish*)
    } >ph_pixels AT>phimg
    /* PIXELS' scratch (raster edge state, the pixel band): window RAM after its code, never loaded. */
    .pixels_bss (NOLOAD) : {
        *(.pixels_bss .pixels_bss.*)
    } >ph_pixels
    .noinit (NOLOAD) : {
        *(.noinit .noinit.* NULL INIT ZPSAVE)
    } >frames
}
__phase_geometry_size = SIZEOF(.phase.geometry);
__phase_pixels_size = SIZEOF(.phase.pixels);

/* The SDK's charset switch (.init.250) calls KERNAL CHROUT, which is unmapped by the time it runs. */
SECTIONS { /DISCARD/ : { *(.init.250) } }

INCLUDE commodore.ld

__stack = 0x0800;

OUTPUT_FORMAT {
    SHORT(ORIGIN(ram))
    TRIM(ram)
}
