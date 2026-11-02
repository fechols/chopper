; Runs before the C runtime: take the machine over for good. No ROM, no IRQs.
; (From ../mode7/target-c64/asm/startup.S.)
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

.section .text.chopper_ignore_interrupt,"ax",@progbits
chopper_ignore_interrupt:
    rti
