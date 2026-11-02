/* MODE 7 C64 layout (cut down from ../talonhd/target-c64/link.ld). Program, data and BSS in
 * $0801-$5BFF; the scene block at $8000, geometry scratch at $E000. VIC banks 1 and 2 hold the frame
 * buffers (screens $5C00 / $9C00, bitmaps $6000 / $A000);
 * $C000-$C707 the frame's line table, $C800 a bounce buffer, $CA80 the status block; the soft stack
 * grows down from $D000. Zero page: the compiler's $02-$7F, fixed.S's $80-$9B, floor.S's $A0-$FF.
 * $01 = $35 (RAM everywhere except I/O); no ROM routines are used. */
__basic_zp_start = 0x0002;
__basic_zp_end = 0x0080;

MEMORY {
    ram (rw) : ORIGIN = 0x0801, LENGTH = 0x53FF
}

/* The SDK's charset switch (.init.250) calls KERNAL CHROUT, which is unmapped by the time it runs. */
SECTIONS { /DISCARD/ : { *(.init.250) } }

INCLUDE commodore.ld

__stack = 0xD000;

OUTPUT_FORMAT {
    SHORT(ORIGIN(ram))
    TRIM(ram)
}
