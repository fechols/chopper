; The boot splash (Assets/Splash/chopper_splash.bin, from tools/encode-splash.py): a multicolor bitmap in
; VIC bank 2, shown before the CRT and kept up through `boot` (main.rs) till `video_init` flips to the
; frame buffer. Nothing the boot writes lands on it: the bitmap at $A000 and the screen at $8C00 load in
; place with the PRG (target-c64/link.ld `.splashbitmap`, `.splashscreen`), and asm/startup.S's band
; code, whose pages are under the bitmap, is copied there only after the boot (`chopper_relocate_band`).
; (As in ../sueme64 and ../impact.)

.section .splash.bitmap,"a",@progbits
    .incbin "chopper_splash.bin", 0, 8000
.section .splash.screen,"a",@progbits
    .incbin "chopper_splash.bin", 8000, 1000
; The colour RAM's nibbles, then the background: copied out at boot, before fixed::init_tables
; overwrites them ($9000, fixed::RECIP).
.section .splash.colors,"a",@progbits
    .incbin "chopper_splash.bin", 9000, 1001

; From `.init.000` (asm/startup.S), on the PRG's own mapping ($01 = $35). Boot-only: the frame buffer's
; memory, free till the boot is done.
.section .text.boot.splash,"ax",@progbits
.globl chopper_splash
chopper_splash:
    ldx #0
1:
    lda __splash_colors,x
    sta $d800,x
    lda __splash_colors+250,x
    sta $d800+250,x
    lda __splash_colors+500,x
    sta $d800+500,x
    lda __splash_colors+750,x
    sta $d800+750,x
    inx
    cpx #250
    bne 1b
    lda #0
    sta $d020
    lda __splash_colors+1000
    sta $d021
    lda $dd02
    ora #3
    sta $dd02
    lda $dd00
    and #$fc
    ora #1              ; bank 2, $8000-$BFFF
    sta $dd00
    lda #$38            ; screen $8C00, bitmap $A000
    sta $d018
    lda #$18            ; multicolor, 40 columns
    sta $d016
    lda #$3b            ; bitmap, display on, 25 rows
    sta $d011
    rts
