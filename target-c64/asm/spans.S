; Floor run generation: one view line's runs (12-byte records at SPANS, read by asm/floor.S) from the
; line's ray at byte 0 (RAYS.a) and the frame's tables (`floor::SpanArgs` at RAYS, `Rays::bt_rows` at
; BT). Instruction-level port of `floor::make_line` and `floor::ground_offset` (src/floor.rs): the same
; integer steps, so `tools/emu.py` frames match the host's byte for byte. Ground points, steps and map
; coordinates are 24-bit here (their values fit: the host's i32 results have the same low three bytes).
;
;   span_line()   the runs of the line whose ray is RAYS.a; their count in RAYS.n
;
; Zero page $A0-$DA while it runs (nothing kept between calls); the multiply (`umul`, asm/fixed.S) owns
; $80-$9B.

RAYS     = $C640
R_LIMIT  = RAYS + 0
R_WHAZE  = RAYS + 24
R_CAMU   = RAYS + 28
R_CAMV   = RAYS + 32
R_H      = RAYS + 36
R_A      = RAYS + 38
R_N      = RAYS + 50
BT       = $B800
BTX0     = BT + 0 * 64
BTX1     = BT + 1 * 64
BTX2     = BT + 2 * 64
BTY0     = BT + 4 * 64
BTY1     = BT + 5 * 64
BTY2     = BT + 6 * 64
BTY3     = BT + 7 * 64
BTZ0     = BT + 8 * 64
BTZ1     = BT + 9 * 64
BTZ2     = BT + 10 * 64
SPANS    = $C400
RECIP    = $9000
SKY      = $55
HAZE     = $44
COLS     = 40
REC      = 12

; umul (asm/fixed.S): AL:AH * BL:BH -> MP0..MP3
AL  = $90
AH  = $91
BL  = $92
BH  = $93
MP0 = $94
MP1 = $95
MP2 = $96
MP3 = $97

NB     = $A0    ; byte b
KK     = $A1    ; run length log2
LEN    = $A2
NCNT   = $A3    ; records written
LASTF  = $A4    ; the last record's fill byte (0: ground)
SVALID = $A5    ; P1 holds the next run's start
LVL    = $A6
OUTP   = $A7    ; (2) next record
PREVP  = $A9    ; (2) last record written
W0V    = $AB    ; (4) w at b
TT     = $AF    ; (4) w at a candidate end / scratch
P0U    = $B3    ; (3) start point, 16.8 texels from the camera (level 0); P0V follows
P0V    = $B6    ; (3)
P1U    = $B9    ; (3) end point; P1V follows
P1V    = $BC    ; (3)
DU     = $BF    ; (3) the run's step per byte
DV     = $C2    ; (3)
GX     = $C5    ; (2) ground_offset: the ray's x, w, z as 16-bit values
GW     = $C7    ; (2)
GZ     = $C9    ; (2)
GK     = $CB    ; w's exponent
GSP    = $CC    ; the h·r shift
GE     = $CD    ; the final exponent (0, -8, -16, -24)
GP16   = $CE    ; (2) the h/w factor
GNEG   = $D0
GN     = $D1    ; (2) the component being scaled
GV     = $D3    ; (4) its product
GT     = $D7    ; (3) scratch
GRES   = $DA    ; where the point goes (zero page address)

.section .text.span_line,"ax",@progbits
.globl span_line
.type span_line,@function
span_line:
    lda #<SPANS
    sta OUTP
    lda #>SPANS
    sta OUTP+1
    lda #0
    sta NB
    sta NCNT
    sta LASTF
    sta SVALID

byteloop:
    ldy NB
    cpy #COLS
    bcc 1f
    lda NCNT
    sta R_N
    rts
1:  jsr wat
    lda TT
    sta W0V
    lda TT+1
    sta W0V+1
    lda TT+2
    sta W0V+2
    lda TT+3
    sta W0V+3
    jsr belowhaze
    bpl ground
    ; sky (w0 <= 0) or haze: one byte
    lda W0V+3
    bmi 2f
    ora W0V+2
    ora W0V+1
    ora W0V
    beq 2f
    lda #HAZE
    bne 3f
2:  lda #SKY
3:  ldx #1
    stx LEN
    jsr putfill
    lda #0
    sta SVALID
    jmp advance

ground:
    ; k = 5, down while 2^k > 40 - b
    sec
    lda #COLS
    sbc NB
    sta GT
    ldx #5
1:  lda pow,x
    cmp GT
    bcc kloop
    beq kloop
    dex
    jmp 1b
kloop:
    cpx #0
    beq kdone
    stx KK
    ; w1 at b + 2^k
    clc
    lda NB
    adc pow,x
    tay
    jsr wat
    jsr belowhaze
    bmi knext
    ; limit[k] <= w0 + w1?
    clc
    lda W0V
    adc TT
    sta TT
    lda W0V+1
    adc TT+1
    sta TT+1
    lda W0V+2
    adc TT+2
    sta TT+2
    lda W0V+3
    adc TT+3
    sta TT+3
    ldx KK
    ldy koffl,x
    lda TT
    cmp R_LIMIT,y
    lda TT+1
    sbc R_LIMIT+1,y
    lda TT+2
    sbc R_LIMIT+2,y
    lda TT+3
    sbc R_LIMIT+3,y
    bvc 2f
    eor #$80
2:  bpl kdone
knext:
    ldx KK
    dex
    jmp kloop
kdone:
    stx KK
    lda pow,x
    sta LEN
    ; a one-byte run whose far end is haze joins the haze
    clc
    lda NB
    adc LEN
    tay
    jsr wat
    jsr belowhaze
    bpl 3f
    jsr hazerun
    jmp advance
3:  ; p0: the last run's end, or the ground under byte b
    lda SVALID
    beq 4f
    ldx #5
5:  lda P1U,x
    sta P0U,x
    dex
    bpl 5b
    jmp 6f
4:  ldy NB
    ldx #P0U
    jsr goff
6:  ; p1: the ground under byte b + len
    clc
    lda NB
    adc LEN
    tay
    ldx #P1U
    jsr goff
    ; du, dv = (p1 - p0) >> k
    sec
    lda P1U
    sbc P0U
    sta DU
    lda P1U+1
    sbc P0U+1
    sta DU+1
    lda P1U+2
    sbc P0U+2
    sta DU+2
    sec
    lda P1V
    sbc P0V
    sta DV
    lda P1V+1
    sbc P0V+1
    sta DV+1
    lda P1V+2
    sbc P0V+2
    sta DV+2
    ldx KK
    beq 8f
7:  lda DU+2
    asl
    ror DU+2
    ror DU+1
    ror DU
    lda DV+2
    asl
    ror DV+2
    ror DV+1
    ror DV
    dex
    bne 7b
8:  jsr level
    bcc 9f
    jsr hazerun
    jmp advance
9:  jsr putground
    lda #1
    sta SVALID
advance:
    clc
    lda NB
    adc LEN
    sta NB
    jmp byteloop

; TT = w at byte Y: -(a.y + bt[Y].y)
wat:
    clc
    lda R_A+4
    adc BTY0,y
    sta TT
    lda R_A+5
    adc BTY1,y
    sta TT+1
    lda R_A+6
    adc BTY2,y
    sta TT+2
    lda R_A+7
    adc BTY3,y
    sta TT+3
    sec
    lda #0
    sbc TT
    sta TT
    lda #0
    sbc TT+1
    sta TT+1
    lda #0
    sbc TT+2
    sta TT+2
    lda #0
    sbc TT+3
    sta TT+3
    rts

; N set when TT < w_haze (signed).
belowhaze:
    lda TT
    cmp R_WHAZE
    lda TT+1
    sbc R_WHAZE+1
    lda TT+2
    sbc R_WHAZE+2
    lda TT+3
    sbc R_WHAZE+3
    bvc 1f
    eor #$80
1:  rts

; LEN bytes of A: extend the last record when it is the same fill, else a new record.
putfill:
    ldx NCNT
    beq 1f
    cmp LASTF
    bne 1f
    ldy #0
    lda (PREVP),y
    clc
    adc LEN
    sta (PREVP),y
    rts
1:  sta LASTF
    ldy #2
    sta (OUTP),y
    ldy #0
    lda LEN
    sta (OUTP),y
    iny
    lda #$80
    sta (OUTP),y
    jmp newrec

; LEN bytes of haze; no next start.
hazerun:
    lda #0
    sta SVALID
    lda #HAZE
    jmp putfill

; A ground record from P0, DU, DV at level LVL.
putground:
    lda #0
    sta LASTF
    ldy #0
    lda LEN
    sta (OUTP),y
    iny
    lda LVL
    sta (OUTP),y
    ; u = (cam_u + p0u) >> level, v likewise (24-bit)
    clc
    lda R_CAMU
    adc P0U
    sta GT
    lda R_CAMU+1
    adc P0U+1
    sta GT+1
    lda R_CAMU+2
    adc P0U+2
    sta GT+2
    jsr shift3
    ldy #2
    jsr put3
    clc
    lda R_CAMV
    adc P0V
    sta GT
    lda R_CAMV+1
    adc P0V+1
    sta GT+1
    lda R_CAMV+2
    adc P0V+2
    sta GT+2
    jsr shift3
    ldy #5
    jsr put3
    ; du, dv >> level (the low two bytes)
    lda DU
    sta GT
    lda DU+1
    sta GT+1
    lda DU+2
    sta GT+2
    jsr shift3
    ldy #8
    lda GT
    sta (OUTP),y
    iny
    lda GT+1
    sta (OUTP),y
    lda DV
    sta GT
    lda DV+1
    sta GT+1
    lda DV+2
    sta GT+2
    jsr shift3
    ldy #10
    lda GT
    sta (OUTP),y
    iny
    lda GT+1
    sta (OUTP),y
newrec:
    lda OUTP
    sta PREVP
    lda OUTP+1
    sta PREVP+1
    clc
    lda OUTP
    adc #REC
    sta OUTP
    bcc 1f
    inc OUTP+1
1:  inc NCNT
    rts

; GT (24-bit) >>= LVL, arithmetic
shift3:
    ldx LVL
    beq 2f
1:  lda GT+2
    asl
    ror GT+2
    ror GT+1
    ror GT
    dex
    bne 1b
2:  rts

; GT's three bytes to the record at offset Y
put3:
    lda GT
    sta (OUTP),y
    iny
    lda GT+1
    sta (OUTP),y
    iny
    lda GT+2
    sta (OUTP),y
    rts

; LVL = the first level 0..3 with max(|du|, |dv|) >> level <= $180; carry set when none.
level:
    lda DU
    sta GT
    lda DU+1
    sta GT+1
    lda DU+2
    sta GT+2
    bpl 1f
    jsr neggt
1:  lda GT
    sta GV
    lda GT+1
    sta GV+1
    lda GT+2
    sta GV+2
    lda DV
    sta GT
    lda DV+1
    sta GT+1
    lda DV+2
    sta GT+2
    bpl 2f
    jsr neggt
2:  ; GT = max(|du| in GV, |dv| in GT)
    lda GT
    cmp GV
    lda GT+1
    sbc GV+1
    lda GT+2
    sbc GV+2
    bcs 3f
    lda GV
    sta GT
    lda GV+1
    sta GT+1
    lda GV+2
    sta GT+2
3:  lda GT+2
    bne 5f
    ldx #0
4:  ; s <= 385·2^x − 1  <=>  s >> x <= $180
    lda thlo,x
    cmp GT
    lda thhi,x
    sbc GT+1
    bcs 6f
    inx
    cpx #4
    bne 4b
5:  sec
    rts
6:  stx LVL
    clc
    rts

neggt:
    sec
    lda #0
    sbc GT
    sta GT
    lda #0
    sbc GT+1
    sta GT+1
    lda #0
    sbc GT+2
    sta GT+2
    rts

; ground_offset: the ground under byte Y of the line, h · (x, z) / w as two 24-bit values at zero
; page X (u, then v). The ray's components as 16-bit values (bytes 1-2 of each i32); h / w as a 16-bit
; factor with a whole-byte exponent.
goff:
    stx GRES
    clc
    lda R_A
    adc BTX0,y
    lda R_A+1
    adc BTX1,y
    sta GX
    lda R_A+2
    adc BTX2,y
    sta GX+1
    clc
    lda R_A+4
    adc BTY0,y
    sta GT
    lda R_A+5
    adc BTY1,y
    sta GT+1
    lda R_A+6
    adc BTY2,y
    sta GT+2
    sec
    lda #0
    sbc GT
    lda #0
    sbc GT+1
    sta GW
    lda #0
    sbc GT+2
    sta GW+1
    clc
    lda R_A+8
    adc BTZ0,y
    lda R_A+9
    adc BTZ1,y
    sta GZ
    lda R_A+10
    adc BTZ2,y
    sta GZ+1
    lda #0
    sta GK
    ; norm16: bits down while >= 2048, up while < 1024
1:  lda GW+1
    cmp #8
    bcc 2f
    lsr GW+1
    ror GW
    inc GK
    jmp 1b
2:  lda GW+1
    cmp #4
    bcs 3f
    asl GW
    rol GW+1
    dec GK
    jmp 2b
3:  ; BL:BH = RECIP[m - 1024] (u16 at RECIP + 2(m - 1024))
    sec
    lda GW+1
    sbc #4
    sta GT+1
    lda GW
    asl
    sta GT
    rol GT+1
    clc
    lda GT+1
    adc #>RECIP
    sta GT+1
    ldy #0
    lda (GT),y
    sta BL
    iny
    lda (GT),y
    sta BH
    lda R_H
    sta AL
    lda R_H+1
    sta AH
    jsr umul
    ; s: the smallest shift that fits h·r in 16 bits (whole bytes first), then up to a whole-byte
    ; exponent e = s - 26 - k
    lda #0
    sta GSP
4:  lda MP3
    beq 5f
    lda MP1
    sta MP0
    lda MP2
    sta MP1
    lda MP3
    sta MP2
    lda #0
    sta MP3
    clc
    lda GSP
    adc #8
    sta GSP
    jmp 4b
5:  lda MP2
    beq 6f
    lsr MP2
    ror MP1
    ror MP0
    inc GSP
    jmp 5b
6:  sec
    lda GSP
    sbc #26
    sec
    sbc GK
    sta GE
    and #7
    beq 7f
    lsr MP1
    ror MP0
    inc GSP
    jmp 6b
7:  lda MP0
    sta GP16
    lda MP1
    sta GP16+1
    lda GX
    sta GN
    lda GX+1
    sta GN+1
    jsr gcomp
    ldx GRES
    lda GV
    sta 0,x
    lda GV+1
    sta 1,x
    lda GV+2
    sta 2,x
    lda GZ
    sta GN
    lda GZ+1
    sta GN+1
    jsr gcomp
    ldx GRES
    lda GV
    sta 3,x
    lda GV+1
    sta 4,x
    lda GV+2
    sta 5,x
    rts

; GV = (GN · GP16) >> -GE (arithmetic, GE a multiple of 8)
gcomp:
    lda GN+1
    sta GNEG
    bpl 1f
    sec
    lda #0
    sbc GN
    sta AL
    lda #0
    sbc GN+1
    sta AH
    jmp 2f
1:  lda GN
    sta AL
    lda GN+1
    sta AH
2:  lda GP16
    sta BL
    lda GP16+1
    sta BH
    jsr umul
    lda MP0
    sta GV
    lda MP1
    sta GV+1
    lda MP2
    sta GV+2
    lda MP3
    sta GV+3
    lda GNEG
    bpl 3f
    sec
    lda #0
    sbc GV
    sta GV
    lda #0
    sbc GV+1
    sta GV+1
    lda #0
    sbc GV+2
    sta GV+2
    lda #0
    sbc GV+3
    sta GV+3
3:  lda GE
4:  beq 6f
    tax
    lda GV+1
    sta GV
    lda GV+2
    sta GV+1
    lda GV+3
    sta GV+2
    asl
    lda #0
    bcc 5f
    lda #$ff
5:  sta GV+3
    txa
    clc
    adc #8
    jmp 4b
6:  rts
.size span_line, .-span_line

.section .rodata.span_tables,"a",@progbits
pow:   .byte 1, 2, 4, 8, 16, 32
koffl: .byte 0, 4, 8, 12, 16, 20
thlo:  .byte <384, <769, <1539, <3079
thhi:  .byte >384, >769, >1539, >3079
