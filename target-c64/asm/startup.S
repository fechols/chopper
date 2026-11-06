; Runs before the CRT: take the machine over for good. No ROM, no IRQs. Then copy the relocated code
; (target-c64/link.ld) from its load images to where it runs.
; (From ../talonhd/target-c64/asm/startup.S by way of ../mode7's, with its relocation copy.)
.macro RELOCATE name
    lda #<__\name\()_load_start
    sta chopper_\name\()_src+1
    lda #>__\name\()_load_start
    sta chopper_\name\()_src+2
    ldx #<__\name\()_pages
    beq 3f
    ldy #0
1:
chopper_\name\()_src:
    lda $ffff,y
chopper_\name\()_dst:
    sta __\name\()_start,y
    iny
    bne 1b
    inc chopper_\name\()_src+2
    inc chopper_\name\()_dst+2
    dex
    bne 1b
3:
.endm

.section .init.000,"ax",@progbits
.globl chopper_boot
chopper_boot:
    sei
    cld
    lda #$2f
    sta $00
    lda #$7f
    sta $dc0d
    sta $dd0d
    lda $dc0d
    lda $dd0d
    lda #0
    sta $d01a
    sta $d015
    lda #$ff
    sta $d019
    ; ROM is still mapped; these stores land in the RAM underneath.
    lda #<chopper_ignore_interrupt
    sta $fffa
    sta $fffe
    lda #>chopper_ignore_interrupt
    sta $fffb
    sta $ffff
    lda #$35
    sta $01
    ; Whole pages: each destination is page-aligned and a whole number of pages.
    RELOCATE lowtext
    RELOCATE spantext
    RELOCATE victext
    RELOCATE hitext
3:  nop                 ; (a target inside this section, for tools/branch-audit.py)
    ; The boot splash (asm/splash.S), up until `video_init`. The band code's pages are under its bitmap:
    ; they follow after the boot (`chopper_relocate_band`).
    jsr chopper_splash

; From main.rs once the boot is done and the screen is blanked, before `video_init` fills the frame
; buffer over these load images (boot-only, like them).
.section .text.boot.relocate,"ax",@progbits
.globl chopper_relocate_band
chopper_relocate_band:
    RELOCATE bandtext
    RELOCATE bandtext2
3:
    rts

.section .text.chopper_ignore_interrupt,"ax",@progbits
chopper_ignore_interrupt:
    rti
