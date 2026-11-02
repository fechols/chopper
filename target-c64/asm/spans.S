; Floor run generation: one view line's runs (12-byte records at SPANS, read by asm/floor.S) from the
; line's ray at byte 0 (RAYS.a) and the frame's tables (`floor::SpanArgs` at RAYS, the x and z rows of
; `Rays::bt` at BT). Instruction-level port of `floor::make_line` and `floor::ground_offset` (src/floor.rs): the same
; integer steps, so `tools/emu.py` frames match the host's byte for byte. Ground points, steps and map
; coordinates are 24-bit here (their values fit: the host's i32 results have the same low three bytes).
;
;   span_line()   the runs of the line whose ray is RAYS.a; their count in RAYS.n
;   bt_build()    the frame's BT rows (the low three bytes of b · bb.x and b · bb.z, b = 0..40)
;
; Zero page $A0-$DF while it runs (nothing kept between calls); the multiply (`umul`, asm/fixed.S) owns
; $80-$9B.

RAYS     = $C640
R_T      = RAYS + 0
R_DW     = RAYS + 24
R_WHAZE  = RAYS + 48
R_CAMU   = RAYS + 52
R_CAMV   = RAYS + 56
R_HC     = RAYS + 60
R_A      = RAYS + 61
R_N      = RAYS + 73
R_BBX    = RAYS + 74
R_BBZ    = RAYS + 78
R_HAZE   = RAYS + 82
BT       = $B800
BTX0     = BT + 0 * 64
BTX1     = BT + 1 * 64
BTX2     = BT + 2 * 64
BTZ0     = BT + 8 * 64
BTZ1     = BT + 9 * 64
BTZ2     = BT + 10 * 64
SPANS    = $C400
RECIP    = $9000
SKY      = $55
COLS     = 40
REC      = 12

; umul (asm/fixed.S): AL:AH * BL:BH -> MP0..MP3. goff multiplies with the same quarter-square tables
; itself: pointers at SQ + p_lo ($80-$87, umul's) and SQ + p_hi ($88-$8F) set once per point.
SQLO  = $0400
SQHI  = $0600
SQNLO = $F100
SQNHI = $F300
PLA = $80
PHA = $82
NLA = $84
NHA = $86
PLB = $88
PHB = $8A
NLB = $8C
NHB = $8E
T0  = $98
T1  = $99
U0  = $9A
U1  = $9B

; \rlo:\rhi = p_lo * Y (A) or p_hi * Y (B)
.macro PRODA rlo, rhi
    lda (PLA),y
    sec
    sbc (NLA),y
    sta \rlo
    lda (PHA),y
    sbc (NHA),y
    sta \rhi
.endm
.macro PRODB rlo, rhi
    lda (PLB),y
    sec
    sbc (NLB),y
    sta \rlo
    lda (PHB),y
    sbc (NHB),y
    sta \rhi
.endm
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
TT     = $AF    ; (4) w at the run's end / the fill stretch's last byte
P0U    = $B3    ; (3) start point, 16.8 texels from the camera (level 0); P0V follows
P0V    = $B6    ; (3)
P1U    = $B9    ; (3) end point; P1V follows
P1V    = $BC    ; (3)
DU     = $BF    ; (3) the run's step per byte
DV     = $C2    ; (3)
GX     = $C5    ; (2) ground_offset: the ray's x, w, z as 16-bit values
GW     = $C7    ; (2)
GZ     = $C9    ; (2)
GK     = $CB    ; w's exponent, then hc + k
GSP    = $CC    ; scratch
GE     = $CD    ; the final right shift in whole bytes (0-3)
GP16   = $CE    ; (2) the 1/w factor
GNEG   = $D0
GN     = $D1    ; (2) the component being scaled
GV     = $E0    ; (7) its product, sign-extended to 7 bytes
GT     = $D7    ; (3) scratch
GRES   = $DA    ; where the point goes (zero page address)
WN     = $DB    ; (4) a candidate w
CLS    = $DF    ; the fill stretch's class

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
    ; w at byte 0: -a.y
    sec
    lda #0
    sbc R_A+4
    sta W0V
    lda #0
    sbc R_A+5
    sta W0V+1
    lda #0
    sbc R_A+6
    sta W0V+2
    lda #0
    sbc R_A+7
    sta W0V+3

byteloop:
    lda NB
    cmp #COLS
    bcc 1f
    lda NCNT
    sta R_N
    rts
1:  ldx #W0V
    jsr wclass
    cmp #2
    bne fills
    jmp ground

fills:
    ; the longest stretch of this class: LEN bytes, TT the last one's w (galloping, w is monotone)
    sta CLS
    lda #1
    sta LEN
    ldx #3
1:  lda W0V,x
    sta TT,x
    dex
    bpl 1b
    ldx #5
2:  clc
    lda NB
    adc LEN
    adc pow,x
    cmp #COLS + 1
    bcs 4f
    jsr stepw
    stx KK
    ldx #WN
    jsr wclass
    ldx KK
    cmp CLS
    bne 4f
    clc
    lda LEN
    adc pow,x
    sta LEN
    ldy #3
3:  lda WN,y
    sta TT,y
    dey
    bpl 3b
4:  dex
    bpl 2b
    ; w at the next byte
    ldx #0
    jsr stepw
    ldx #3
5:  lda WN,x
    sta W0V,x
    dex
    bpl 5b
    lda CLS
    beq 6f
    lda R_HAZE
    bne 7f
6:  lda #SKY
7:  jsr putfill
    lda #0
    sta SVALID
    jmp advance

ground:
    ; k = the largest with 2^k <= 40 - b, then down while w0 < t[k]
    ldy NB
    ldx kmax,y
2:  cpx #0
    beq 3f
    jsr cmpt
    bpl 4f
    dex
    jmp 2b
3:  ; k = 0: a one-byte run whose far end is haze joins the haze
    jsr cmpt
    bpl 4f
    lda #1
    sta LEN
    jsr stepw0
    jsr w0tt
    jsr hazerun
    jmp advance
4:  stx KK
    lda pow,x
    sta LEN
    ; w1 = w0 - dw[k] in TT
    jsr stepw0
    ; p0: the last run's end, or the ground under byte b
    lda SVALID
    beq 6f
    lda P1U
    sta P0U
    lda P1U+1
    sta P0U+1
    lda P1U+2
    sta P0U+2
    lda P1V
    sta P0V
    lda P1V+1
    sta P0V+1
    lda P1V+2
    sta P0V+2
    jmp 7f
6:  lda W0V+1
    sta GW
    lda W0V+2
    sta GW+1
    ldy NB
    ldx #P0U
    jsr goff
7:  ; p1: the ground under byte b + len
    lda TT+1
    sta GW
    lda TT+2
    sta GW+1
    clc
    lda NB
    adc LEN
    tay
    ldx #P1U
    jsr goff
    lda #1
    sta SVALID
    jsr w0tt
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
    beq 10f
9:  lda DU+2
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
    bne 9b
10: jsr level
    bcc 11f
    lda R_HAZE
    jsr putfill
    jmp advance
11: jsr putground
advance:
    clc
    lda NB
    adc LEN
    sta NB
    jmp byteloop

; A = what the w (32-bit) at zero page X shows: 0 sky (w <= 0), 1 haze (w < w_haze), 2 ground
wclass:
    lda 3,x
    bmi 2f
    ora 2,x
    ora 1,x
    ora 0,x
    beq 2f
    lda 0,x
    cmp R_WHAZE
    lda 1,x
    sbc R_WHAZE+1
    lda 2,x
    sbc R_WHAZE+2
    lda 3,x
    sbc R_WHAZE+3
    bcc 1f
    lda #2
    rts
1:  lda #1
    rts
2:  lda #0
    rts

; WN = TT - dw[X] (X kept)
stepw:
    ldy koffl,x
    sec
    lda TT
    sbc R_DW,y
    sta WN
    lda TT+1
    sbc R_DW+1,y
    sta WN+1
    lda TT+2
    sbc R_DW+2,y
    sta WN+2
    lda TT+3
    sbc R_DW+3,y
    sta WN+3
    rts

; TT = W0V - dw[X] (X kept)
stepw0:
    ldy koffl,x
    sec
    lda W0V
    sbc R_DW,y
    sta TT
    lda W0V+1
    sbc R_DW+1,y
    sta TT+1
    lda W0V+2
    sbc R_DW+2,y
    sta TT+2
    lda W0V+3
    sbc R_DW+3,y
    sta TT+3
    rts

; N set when w0 < t[X] (signed; X kept)
cmpt:
    ldy koffl,x
    lda W0V
    cmp R_T,y
    lda W0V+1
    sbc R_T+1,y
    lda W0V+2
    sbc R_T+2,y
    lda W0V+3
    sbc R_T+3,y
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
    lda R_HAZE
    jmp putfill

; A ground record from P0, DU, DV at level LVL: u = (cam_u + p0u) >> level, v likewise (24-bit),
; du, dv >> level (their low two bytes).
putground:
    lda #0
    sta LASTF
    clc
    lda R_CAMU
    adc P0U
    sta GX
    lda R_CAMU+1
    adc P0U+1
    sta GX+1
    lda R_CAMU+2
    adc P0U+2
    sta GW
    clc
    lda R_CAMV
    adc P0V
    sta GW+1
    lda R_CAMV+1
    adc P0V+1
    sta GZ
    lda R_CAMV+2
    adc P0V+2
    sta GZ+1
    ldx LVL
    beq 2f
1:  lda GW
    asl
    ror GW
    ror GX+1
    ror GX
    lda GZ+1
    asl
    ror GZ+1
    ror GZ
    ror GW+1
    lda DU+2
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
    bne 1b
2:  ldy #0
    lda LEN
    sta (OUTP),y
    iny
    lda LVL
    sta (OUTP),y
    iny
    lda GX
    sta (OUTP),y
    iny
    lda GX+1
    sta (OUTP),y
    iny
    lda GW
    sta (OUTP),y
    iny
    lda GW+1
    sta (OUTP),y
    iny
    lda GZ
    sta (OUTP),y
    iny
    lda GZ+1
    sta (OUTP),y
    iny
    lda DU
    sta (OUTP),y
    iny
    lda DU+1
    sta (OUTP),y
    iny
    lda DV
    sta (OUTP),y
    iny
    lda DV+1
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

; W0V = TT
w0tt:
    lda TT
    sta W0V
    lda TT+1
    sta W0V+1
    lda TT+2
    sta W0V+2
    lda TT+3
    sta W0V+3
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

; ground_offset: the ground under byte Y of the line, H · (x, z) / w as two 24-bit values at zero
; page X (u, then v). The ray's (altitude-scaled) x and z as 16-bit values (bytes 1-2 of each i32),
; w >> 8 in GW (the caller's); 1/w as a 16-bit factor with a whole-byte exponent.
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
    lda R_A+8
    adc BTZ0,y
    lda R_A+9
    adc BTZ1,y
    sta GZ
    lda R_A+10
    adc BTZ2,y
    sta GZ+1
    ; norm16: GW = m · 2^k, m in 1024..2047 (a whole byte first when the high byte is 0)
    ldx #0
    lda GW+1
    bne 1f
    lda GW
    sta GW+1
    lda #0
    sta GW
    ldx #$f8
    lda GW+1
1:  cmp #8
    bcc 2f
    lsr GW+1
    ror GW
    inx
    lda GW+1
    jmp 1b
2:  cmp #4
    bcs 3f
    asl GW
    rol GW+1
    dex
    lda GW+1
    jmp 2b
3:  ; ck = hc + k; e = ck >> 3 bytes, p = RECIP[m - 1024] >> (ck & 7)
    txa
    clc
    adc R_HC
    sta GK
    lsr
    lsr
    lsr
    sta GE
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
    sta GP16
    iny
    lda (GT),y
    sta GP16+1
    lda GK
    and #7
    beq 5f
    tax
4:  lsr GP16+1
    ror GP16
    dex
    bne 4b
5:  ; the tables at p's two bytes
    lda GP16
    sta PLA
    sta PHA
    eor #$ff
    sta NLA
    sta NHA
    lda GP16+1
    sta PLB
    sta PHB
    eor #$ff
    sta NLB
    sta NHB
    lda GX
    sta GN
    lda GX+1
    sta GN+1
    jsr gcomp
    ldx GRES
    ldy GE
    lda GV,y
    sta 0,x
    lda GV+1,y
    sta 1,x
    lda GV+2,y
    sta 2,x
    lda GZ
    sta GN
    lda GZ+1
    sta GN+1
    jsr gcomp
    ldx GRES
    ldy GE
    lda GV,y
    sta 3,x
    lda GV+1,y
    sta 4,x
    lda GV+2,y
    sta 5,x
    rts

; GV (7 bytes, sign-extended) = GN (signed) · p (the tables goff points at)
gcomp:
    lda GN+1
    sta GNEG
    bpl 1f
    sec
    lda #0
    sbc GN
    sta GN
    lda #0
    sbc GN+1
    sta GN+1
1:  ldy GN
    PRODA GV, GV+1
    PRODB T0, T1
    ldy GN+1
    PRODA U0, U1
    PRODB GV+2, GV+3
    clc
    lda GV+1
    adc T0
    sta GV+1
    lda GV+2
    adc T1
    sta GV+2
    bcc 2f
    inc GV+3
    clc
2:  lda GV+1
    adc U0
    sta GV+1
    lda GV+2
    adc U1
    sta GV+2
    bcc 3f
    inc GV+3
3:  lda GNEG
    bmi 4f
    lda #0
    sta GV+4
    sta GV+5
    sta GV+6
    rts
4:  sec
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
    lda #$ff
    sta GV+4
    sta GV+5
    sta GV+6
    rts
.size span_line, .-span_line

; BT's x and z rows by adding bb up (the sums' low three bytes are all goff uses).
.section .text.bt_build,"ax",@progbits
.globl bt_build
.type bt_build,@function
bt_build:
    lda #>SQLO
    sta PLB+1
    lda #>SQHI
    sta PHB+1
    lda #>SQNLO
    sta NLB+1
    lda #>SQNHI
    sta NHB+1
    lda #0
    ldx #5
1:  sta $A0,x
    dex
    bpl 1b
    tax
2:  lda $A0
    sta BTX0,x
    lda $A1
    sta BTX1,x
    lda $A2
    sta BTX2,x
    lda $A3
    sta BTZ0,x
    lda $A4
    sta BTZ1,x
    lda $A5
    sta BTZ2,x
    clc
    lda $A0
    adc R_BBX
    sta $A0
    lda $A1
    adc R_BBX+1
    sta $A1
    lda $A2
    adc R_BBX+2
    sta $A2
    clc
    lda $A3
    adc R_BBZ
    sta $A3
    lda $A4
    adc R_BBZ+1
    sta $A4
    lda $A5
    adc R_BBZ+2
    sta $A5
    inx
    cpx #COLS + 1
    bne 2b
    rts
.size bt_build, .-bt_build

.section .rodata.span_tables,"a",@progbits
pow:   .byte 1, 2, 4, 8, 16, 32
; the largest k with 2^k <= 40 - b, for b = 0..39
kmax:  .byte 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4
       .byte 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0
koffl: .byte 0, 4, 8, 12, 16, 20
thlo:  .byte <384, <769, <1539, <3079
thhi:  .byte >384, >769, >1539, >3079
