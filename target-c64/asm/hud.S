; hud_irq: the HUD's sprite multiplexer (raster IRQ), after TALON HD's (../talonhd/target-c64/asm/hud.S),
; with a free X and Y per sprite. Contract: chopper::hud (src/hud.rs).
;
; The page at HUD ($8B00): READY (set by the main loop when a new table is on screen), FRONT (the table
; shown: 0 or $80 from BUF), NEXT (the next row's offset in its table). A table is ROWS rows of ROW
; bytes: the IRQ line that sets the row up (0: no more rows), $D010, $D015, then 8 Ys, 8 X lows and 8
; pointers. Row 0 is set up in the top border (TOP_LINE), where the IRQ also swaps to the new table when
; READY is set.
;
; Touches only A, X, Y (saved), the VIC registers, the sprite pointers and the page, and the sound
; effects' sweeps (the SIDs and chopper::sfx::SWEEP at $FF20): no zero page, no REU, no other Rust state
; (it only reads the player's hit flash), so it may interrupt anything. Chopper keeps $01 = $35 throughout.
;
; The screen is a fixed 38 × 24 window (chopper::layout::D011/D016), so the fine scroll can move the
; picture up to 4 pixels either way without showing its edges; at rest the scroll is (REST_X, REST_Y).
; The damage shake (hud_shake, once a video frame in row 0): a hit (chopper::combat::player::FLASH rising)
; moves the picture by the shake tables for SHAKE_FRAMES frames, and every HUD sprite (and each row's IRQ
; line) the same distance the other way: the HUD jolts against the view, so the player feels shaken. The HUD's
; tables hold positions over the picture at scroll 0; the IRQ adds REST_X (and the shake) to every X.

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
; The shake's state (chopper::hud::SHAKE..): frames left, the hit flash last seen, this frame's sprite
; offsets (hires px right, lines down), and hud_xy's $D010 being built and its bit (hud_shake's scratch).
SHAKE    = HUD + $0A
LASTF    = HUD + $0B
HDX      = HUD + $0C
HDY      = HUD + $0D
HMSB     = HUD + $0E
HBIT     = HUD + $0F
FLASH    = $8ED3            ; chopper::combat::PLAYER + player::FLASH (hud.rs asserts it)
REST_X   = 4                ; chopper::layout::D016 & 7
REST_Y   = 3                ; chopper::layout::D011 & 7
SHAKE_FRAMES = 30

.section .text.hud_irq,"ax",@progbits
.globl hud_irq
.type hud_irq,@function
hud_irq:
    pha
    txa
    pha
    tya
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
    jsr hud_shake           ; row 0: this frame's shake (X stays 0)
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

    jsr hud_xy              ; Ys, Xs and $D010, offset by the shake
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
    clc
    adc HDY                 ; (the row's sprites moved with the shake)
    sta $d012
    jmp 3f
2:
    lda #0
    sta NEXT
    lda #TOP_LINE
    sta $d012
3:
    pla
    tay
    pla
    tax
    pla
    rti
.size hud_irq, .-hud_irq

; The row's sprite Ys and Xs (X = the row in the table; kept) and $D010, each moved by the shake's
; offsets (HDY lines, HDX hires px: 1-8, carrying into the X's high bit). Resident: .victext is full.
.section .text.hud_xy,"ax",@progbits
.globl hud_xy
.type hud_xy,@function
hud_xy:
    lda BUF+R_MSB,x
    sta HMSB
    lda #1
    sta HBIT
    ldy #0
1:
    lda BUF+R_Y,x
    clc
    adc HDY
    sta $d001,y
    lda BUF+R_X,x
    clc
    adc HDX
    sta $d000,y
    bcc 2f
    lda HMSB                ; carried: the high bit flips
    eor HBIT
    sta HMSB
2:
    asl HBIT
    inx
    iny
    iny
    cpy #16
    bne 1b
    lda HMSB
    sta $d010
    txa
    sec
    sbc #8
    tax
    rts
.size hud_xy, .-hud_xy

; Once a video frame (row 0, in the top border): a rise of the hit flash starts SHAKE_FRAMES of shake;
; while it lasts, the picture's offset p (x) and q (y) from the tables (halved over its last frames), else
; none. The fine scroll is the rest scroll plus the offset; the sprites move by REST_X - p and -q. $D011's
; display-enable bit is kept (the boot turns the display on later). Returns X = 0.
.section .text.hud_shake,"ax",@progbits
.globl hud_shake
.type hud_shake,@function
hud_shake:
    lda FLASH
    cmp LASTF
    sta LASTF
    beq 1f
    bcc 1f
    lda #SHAKE_FRAMES
    sta SHAKE
1:
    lda #0
    sta HMSB                ; p
    sta HBIT                ; q
    ldx SHAKE
    beq 4f
    dec SHAKE
    txa
    and #15
    tay
    lda shake_dx,y
    cpx #10
    bcs 2f
    cmp #$80
    ror a
2:
    sta HMSB
    lda shake_dy,y
    cpx #10
    bcs 3f
    cmp #$80
    ror a
3:
    sta HBIT
4:
    lda HMSB
    clc
    adc #REST_X | $10       ; 38 columns, multicolour
    sta $d016
    lda #REST_X
    sec
    sbc HMSB
    sta HDX
    lda #0
    sec
    sbc HBIT
    sta HDY
    lda $d011
    and #$10
    ora #$20 | REST_Y       ; bitmap, 24 rows
    clc
    adc HBIT
    sta $d011
    ldx #0
    rts
.size hud_shake, .-hud_shake

; The picture's offsets a frame, indexed by the frames left: hires px right (-4..+3) and lines down
; (-3..+4), the fine scroll's reach either side of the rest scroll.
.section .rodata.hud_shake,"a",@progbits
shake_dx:
    .byte 2, -3, 1, -4, 3, -1, 0, -2, 3, -4, 2, -1, 1, -3, 3, -2
shake_dy:
    .byte 3, -2, 1, -3, 4, -1, 2, -3, 0, 3, -2, 4, -1, 1, -3, 2

; Interrupts on (the raster IRQ is armed by the caller).
.section .text.hud_irq_enable,"ax",@progbits
.globl hud_irq_enable
.type hud_irq_enable,@function
hud_irq_enable:
    cli
    rts
.size hud_irq_enable, .-hud_irq_enable
