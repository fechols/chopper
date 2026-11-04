; hud_irq: the HUD's sprite multiplexer (raster IRQ), after TALON HD's (../talonhd/target-c64/asm/hud.S),
; with a free X and Y per sprite. Contract: chopper::hud (src/hud.rs).
;
; The page at HUD ($8B00): READY (set by the main loop when a new table is on screen), FRONT (the table
; shown: 0 or $80 from BUF), NEXT (the next row's offset in its table). A table is ROWS rows of ROW
; bytes: the IRQ line that sets the row up (0: no more rows), $D010, $D015, then 8 Ys, 8 X lows and 8
; pointers. Row 0 is set up in the top border (TOP_LINE), where the IRQ also swaps to the new table when
; READY is set.
;
; Touches only A, X (saved), the VIC registers, the sprite pointers and the page, and the sound effects'
; sweeps (the SIDs and chopper::sfx::SWEEP at $FF20): no zero page, no REU, no other Rust state, so it may
; interrupt anything. Chopper keeps $01 = $35 throughout.

HUD      = $8B00
READY    = HUD + 0
FRONT    = HUD + 1
NEXT     = HUD + 2
BUF      = HUD + $10
R_LINE   = 0
R_MSB    = 1
R_EN     = 2
R_Y      = 3
R_X      = 11
R_P      = 19
ROW      = 27
ROWS     = 4
TOP_LINE = 20
PTRS     = $5FF8

.section .text.hud_irq,"ax",@progbits
.globl hud_irq
.type hud_irq,@function
hud_irq:
    pha
    txa
    pha
    lda #$01
    sta $d019               ; acknowledge the raster interrupt

    ldx NEXT
    beq 5f
    jmp 1f                  ; (a later row: past the sweeps, out of a branch's reach)
5:
    ; Row 0 (once a video frame, 60 Hz): the sound effects' pitch sweeps (chopper::sfx::SWEEP, per
    ; pair: frames left, three frequency high bytes, their deltas), onto the pair's left and right SIDs.
    ; (A only: X is NEXT, 0, below.)
    lda $FF20
    beq 3f
    dec $FF20
    lda $FF21
    clc
    adc $FF24
    sta $FF21
    sta $D401
    sta $D441
    lda $FF22
    clc
    adc $FF25
    sta $FF22
    sta $D408
    sta $D448
    lda $FF23
    clc
    adc $FF26
    sta $FF23
    sta $D40F
    sta $D44F
3:
    lda $FF27
    beq 4f
    dec $FF27
    lda $FF28
    clc
    adc $FF2B
    sta $FF28
    sta $D421
    sta $D461
    lda $FF29
    clc
    adc $FF2C
    sta $FF29
    sta $D428
    sta $D468
    lda $FF2A
    clc
    adc $FF2D
    sta $FF2A
    sta $D42F
    sta $D46F
4:
    lda READY               ; row 0: take the new table, if there is one
    beq 1f
    lda FRONT
    eor #$80
    sta FRONT
    lda #0
    sta READY
1:
    txa
    clc
    adc FRONT
    tax                     ; X = this row in the table shown

    lda BUF+R_Y+0,x
    sta $d001
    lda BUF+R_Y+1,x
    sta $d003
    lda BUF+R_Y+2,x
    sta $d005
    lda BUF+R_Y+3,x
    sta $d007
    lda BUF+R_Y+4,x
    sta $d009
    lda BUF+R_Y+5,x
    sta $d00b
    lda BUF+R_Y+6,x
    sta $d00d
    lda BUF+R_Y+7,x
    sta $d00f
    lda BUF+R_X+0,x
    sta $d000
    lda BUF+R_X+1,x
    sta $d002
    lda BUF+R_X+2,x
    sta $d004
    lda BUF+R_X+3,x
    sta $d006
    lda BUF+R_X+4,x
    sta $d008
    lda BUF+R_X+5,x
    sta $d00a
    lda BUF+R_X+6,x
    sta $d00c
    lda BUF+R_X+7,x
    sta $d00e
    lda BUF+R_MSB,x
    sta $d010
    lda BUF+R_EN,x
    sta $d015
    lda BUF+R_P+0,x
    sta PTRS+0
    lda BUF+R_P+1,x
    sta PTRS+1
    lda BUF+R_P+2,x
    sta PTRS+2
    lda BUF+R_P+3,x
    sta PTRS+3
    lda BUF+R_P+4,x
    sta PTRS+4
    lda BUF+R_P+5,x
    sta PTRS+5
    lda BUF+R_P+6,x
    sta PTRS+6
    lda BUF+R_P+7,x
    sta PTRS+7

    ; The next row's IRQ, or row 0's in the top border.
    lda NEXT
    clc
    adc #ROW
    cmp #ROWS*ROW
    bcs 2f
    sta NEXT
    adc FRONT               ; (carry clear)
    tax
    lda BUF+R_LINE,x
    beq 2f
    sta $d012
    pla
    tax
    pla
    rti
2:
    lda #0
    sta NEXT
    lda #TOP_LINE
    sta $d012
    pla
    tax
    pla
    rti
.size hud_irq, .-hud_irq

; Interrupts on (the raster IRQ is armed by the caller).
.section .text.hud_irq_enable,"ax",@progbits
.globl hud_irq_enable
.type hud_irq_enable,@function
hud_irq_enable:
    cli
    rts
.size hud_irq_enable, .-hud_irq_enable
