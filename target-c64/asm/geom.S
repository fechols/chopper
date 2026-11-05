; Geometry kernels (from ../mode7, made full 3D for chopper). References in src/geom.rs (`xform`,
; `faces`, `Bands` + `setup`); the host runs those, the C64 runs these, and `tools/emu.py` checks whole
; frames byte for byte.
;
;   geo_xform():  one object's vertices -> camera space and screen (VERTS arrays)
;   geo_faces():  one object's faces -> backface cull, depth key, push into the buckets (WORK);
;                 faces straddling the near plane are listed in WORK.clip and cut by Rust `geo_clip`
;   geo_bin() / geo_activate(band) / geo_draw(band): the buckets in painter's order, band by band
;                 (per-triangle set-up when its first band comes, then tri_row per band)
; Arguments in GEO_ARGS ($C7C0, `geom::geo_args`); shade patterns at PATS ($C7E0, even[16], odd[16]).
;
; Zero page: $C0-$EF and $F0-$FF (floor.S's, free while the floor is not drawing); fixed.S has $90-$9B,
; poly.S / tri.S $A0-$B3.

GA     = $8E00
VPTR   = GA + 0
NV     = GA + 2
FPTR   = GA + 3
NF     = GA + 5
SHR    = GA + 6
TT     = GA + 7
ORGX   = GA + 8     ; i16 x 3: the object's origin in camera space
MROT   = GA + 14    ; i16 x 9: mesh (u, up, v) -> camera, row-major
BMP    = GA + 32
TRIS   = GA + 34
TOP    = GA + 35
; The view (`geom::LINES`, `geom::BANDS`): the lines above the cockpit.
LINES  = 128
NBANDS = 16
; The optical axis' line (`geom::CY`), Q4.
CY16   = 1088
; The near plane (`geom::NEAR`, Q4): 2 texels.
NEARZ  = 32
PATE   = $C7E0
PATO   = $C7F0

VERTS  = $9800          ; `geom::Verts`: ten arrays of MAX_VERTS (64)
VXL    = VERTS + $000
VXH    = VERTS + $040
VYL    = VERTS + $080
VYH    = VERTS + $0C0
VZL    = VERTS + $100
VZH    = VERTS + $140
SXL    = VERTS + $180
SXH    = VERTS + $1C0
SYL    = VERTS + $200
SYH    = VERTS + $240

WORK   = $E000
TX0L   = WORK + $000
TX0H   = WORK + $100
TX1L   = WORK + $200
TX1H   = WORK + $300
TX2L   = WORK + $400
TX2H   = WORK + $500
TY0L   = WORK + $600
TY0H   = WORK + $700
TY1L   = WORK + $800
TY1H   = WORK + $900
TY2L   = WORK + $A00
TY2H   = WORK + $B00
TSHADE = WORK + $C00
HEAD   = WORK + $D00
NEXT   = WORK + $E00
CLIP   = WORK + $F00
WN     = WORK + $1000
WDROP  = WORK + $1001
WNCLIP = WORK + $1002

RECIP  = $9000
TRI_ARGS = $C7A0
POLY_PE  = $C790
POLY_PO  = $C791

; fixed.S's multiply registers
AL = $90
AH = $91
BL = $92
BH = $93
P0 = $94
P1 = $95
P2 = $96
P3 = $97

VP     = $C0    ; vertex / face pointer (2)
I      = $C2    ; vertex index; LINE in set-up
LINE   = $C2
VA     = $C3    ; (2) vertex a; YBV in set-up
YBV     = $C3
VB     = $C5    ; (2) vertex b; D in set-up
DD     = $C5
S1     = $C7    ; (4) rotated x sum; PER in set-up; projected sx, sy out
PER    = $C7
S2     = $CB    ; (4) rotated z sum; EX in set-up
EX     = $CB
PX     = $CF    ; (2) camera x; XAV in set-up
XAV     = $CF
PY     = $D1    ; (2) camera y; YAV in set-up
YAV     = $D1
PZ     = $D3    ; (2) camera z; XBV in set-up
XBV     = $D3
M      = $D5    ; (2) mantissa
E      = $D7    ; exponent (signed)
RL     = $D8    ; reciprocal
RH     = $D9
SHF    = $DA
V      = $DB    ; (4) 32-bit work
SGN    = $DF
Q      = $E0    ; (4) 32-bit work; ES in set-up
ES     = $E0
FP     = $E4    ; (2) face pointer; LA in set-up
LA     = $E4
F      = $E6
IA     = $E7
IB     = $E8
IC     = $E9
SHD    = $EA
KEY    = $EB
SH8    = $EC
CNT    = $ED
RPTR     = $EE    ; (2) reciprocal pointer
TXL    = $F0    ; corners of the face / triangle: x lo[3], x hi[3], y lo[3], y hi[3]
TXH    = $F3
TYL    = $F6
TYH    = $F9
FIRST  = $FC
MIDL   = $FD
ENDL   = $FE
IDX    = $FF

; ---------------------------------------------------------------------------------------------------
; Helpers.

.section .text.geo_helpers,"ax",@progbits

; P = AL:AH * BL:BH, both signed.
smul:
    lda AH
    eor BH
    sta SGN
    lda AH
    bpl 1f
    sec
    lda #0
    sbc AL
    sta AL
    lda #0
    sbc AH
    sta AH
1:  lda BH
    bpl 2f
    sec
    lda #0
    sbc BL
    sta BL
    lda #0
    sbc BH
    sta BH
2:  jsr umul
    bit SGN
    bpl 3f
    jsr negp
3:  rts

; P = AL:AH (signed) * BL:BH (unsigned).
smulsu:
    lda AH
    sta SGN
    bpl 1f
    sec
    lda #0
    sbc AL
    sta AL
    lda #0
    sbc AH
    sta AH
1:  jsr umul
    bit SGN
    bpl 2f
    jsr negp
2:  rts

negp:
    sec
    lda #0
    sbc P0
    sta P0
    lda #0
    sbc P1
    sta P1
    lda #0
    sbc P2
    sta P2
    lda #0
    sbc P3
    sta P3
    rts

; V = P
pv:
    lda P0
    sta V
    lda P1
    sta V+1
    lda P2
    sta V+2
    lda P3
    sta V+3
    rts

; V >>= A (arithmetic, A = 0..31)
sarv:
    tay
1:  cpy #8
    bcc 2f
    lda V+1
    sta V
    lda V+2
    sta V+1
    lda V+3
    sta V+2
    cmp #$80
    lda #0
    adc #$ff
    eor #$ff
    sta V+3
    tya
    sec
    sbc #8
    tay
    jmp 1b
2:  cpy #0
    beq 4f
3:  lda V+3
    cmp #$80
    ror V+3
    ror V+2
    ror V+1
    ror V
    dey
    bne 3b
4:  rts

shlv:
    asl V
    rol V+1
    rol V+2
    rol V+3
    rts

; V (i32) -> V:V+1 clamped to -16000..16000.
clampv:
    lda V+3
    bmi 3f
    ora V+2
    bne 2f
    lda V
    cmp #$81            ; 16001 = $3E81
    lda V+1
    sbc #$3e
    bcs 2f
    rts
2:  lda #$80            ; 16000 = $3E80
    sta V
    lda #$3e
    sta V+1
    rts
3:  lda V+3
    and V+2
    cmp #$ff
    bne 4f
    lda V
    cmp #$80            ; -16000 = $C180
    lda V+1
    sbc #$c1
    bcc 4f
    rts
4:  lda #$80
    sta V
    lda #$c1
    sta V+1
    rts

; M (u16, > 0) -> M in 1024..2047, E, and RL:RH = recip(M) (`fixed::norm`, `fixed::recip`).
norm:
    lda #0
    sta E
1:  lda M+1
    cmp #4
    bcs 2f
    asl M
    rol M+1
    dec E
    jmp 1b
2:  lda M+1
    cmp #8
    bcc 3f
    lsr M+1
    ror M
    inc E
    jmp 2b
3:  lda M+1
    sec
    sbc #4
    sta RPTR+1
    lda M
    asl
    sta RPTR
    rol RPTR+1
    lda RPTR+1
    clc
    adc #>RECIP
    sta RPTR+1
    ldy #0
    lda (RPTR),y
    sta RL
    iny
    lda (RPTR),y
    sta RH
    rts

setr:
    lda RL
    sta BL
    lda RH
    sta BH
    rts

; (PX, PY, PZ) -> S1:S1+1 = sx, S1+2:S1+3 = sy (`geom::project`; PZ >= NEAR): 80 + 80x/z, 68 - 133y/z,
; with rx = recip * 5 >> 3 and ry = recip * 133 >> 8 once per point.
project:
    lda PZ
    sta M
    lda PZ+1
    sta M+1
    jsr norm
    ; rx = (r << 2 + r) >> 3
    lda RL
    sta Q
    lda RH
    sta Q+1
    lda #0
    sta Q+2
    asl Q
    rol Q+1
    rol Q+2
    asl Q
    rol Q+1
    rol Q+2
    clc
    lda Q
    adc RL
    sta Q
    lda Q+1
    adc RH
    sta Q+1
    lda Q+2
    adc #0
    .rept 3
    lsr
    ror Q+1
    ror Q
    .endr
    ; sx = 1280 + (x * rx) >> (15 + e)
    lda PX
    sta AL
    lda PX+1
    sta AH
    lda Q
    sta BL
    lda Q+1
    sta BH
    jsr smulsu
    jsr pv
    lda E
    clc
    adc #15
    jsr sarv
    clc
    lda V+1
    adc #$05            ; 1280 = $0500
    sta V+1
    lda V+2
    adc #0
    sta V+2
    lda V+3
    adc #0
    sta V+3
    jsr clampv
    lda V
    sta S1
    lda V+1
    sta S1+1
    ; ry = (r << 7 + r << 2 + r) >> 8
    lda RH
    lsr
    sta Q+2
    lda RL
    ror
    sta Q+1
    lda #0
    ror
    sta Q               ; r << 7
    lda RL
    sta V
    lda RH
    sta V+1
    lda #0
    sta V+2
    asl V
    rol V+1
    rol V+2
    asl V
    rol V+1
    rol V+2             ; r << 2
    clc
    lda V
    adc RL
    sta V
    lda V+1
    adc RH
    sta V+1
    lda V+2
    adc #0
    sta V+2             ; r << 2 + r
    clc
    lda Q
    adc V
    lda Q+1
    adc V+1
    sta BL
    lda Q+2
    adc V+2
    sta BH
    ; sy = CY16 + (-y * ry) >> (14 + e)
    sec
    lda #0
    sbc PY
    sta AL
    lda #0
    sbc PY+1
    sta AH
    jsr smulsu
    jsr pv
    lda E
    clc
    adc #14
    jsr sarv
    clc
    lda V
    adc #<CY16
    sta V
    lda V+1
    adc #>CY16
    sta V+1
    lda V+2
    adc #0
    sta V+2
    lda V+3
    adc #0
    sta V+3
    jsr clampv
    lda V
    sta S1+2
    lda V+1
    sta S1+3
    rts

; A (i8) -> AL:AH sign-extended
sxa:
    sta AL
    ora #$7f
    bmi 1f
    lda #0
1:  sta AH
    rts

; ---------------------------------------------------------------------------------------------------
; geo_xform: per vertex (a, b, h) (u, v, up): P_j = org_j + (m[j][0]*a + m[j][1]*h + m[j][2]*b) >> sh
; for j = x, y, z; project if z >= NEARZ. The vertex components are bytes, so each product is |c| times
; |m| by two 8x8 quarter-square lookups with the tables pointed at |c| once per component (the products
; are exact: the same sums as `geom::xform`).

MAB    = $8F00          ; |m|, 9 x u16 (row-major), set once per object
MSG    = $8F12          ; m's sign bytes, 9
SA     = $C3            ; (9) the three sums, 24-bit each
CSGN   = $E7            ; the component's sign
KCOL   = $E8            ; the component's column (0 a, 1 h, 2 b)
JIDX   = $E9            ; the row
XT0    = $F0            ; |m| of the entry
XT1    = $F1
XP0    = $F2            ; the product, 24-bit (+ a spare byte)
XP1    = $F3
XP2    = $F4
XP3    = $F5
PLA = $80
PHA = $82
NLA = $84
NHA = $86

; \rlo:\rhi = |c| * Y (the tables at |c|)
.macro XPROD rlo, rhi
    lda (PLA),y
    sec
    sbc (NLA),y
    sta \rlo
    lda (PHA),y
    sbc (NHA),y
    sta \rhi
.endm

.section .text.geo_xform,"ax",@progbits
.globl geo_xform
.type geo_xform,@function
geo_xform:
    ; |m| and the signs, once per object
    ldx #0
1:  txa
    asl
    tay
    lda MROT+1,y
    sta MSG,x
    bpl 2f
    sec
    lda #0
    sbc MROT,y
    sta MAB,y
    lda #0
    sbc MROT+1,y
    sta MAB+1,y
    jmp 3f
2:  lda MROT,y
    sta MAB,y
    lda MROT+1,y
    sta MAB+1,y
3:  inx
    cpx #9
    bne 1b
    ; |m| >>= sh - 8 (`geom::xform`'s m'), so a sum's bytes 1-2 are the offset
    lda SHR
    sec
    sbc #8
    beq 6f
    sta XT0
    ldx #16
4:  ldy XT0
5:  lsr MAB+1,x
    ror MAB,x
    dey
    bne 5b
    dex
    dex
    bpl 4b
6:  lda VPTR
    sta VP
    lda VPTR+1
    sta VP+1
    lda #0
    sta I
xloop:
    lda I
    cmp NV
    bcc 1f
    rts
1:  lda #0
    sta SA
    sta SA+1
    sta SA+2
    sta SA+3
    sta SA+4
    sta SA+5
    sta SA+6
    sta SA+7
    sta SA+8
    ldy #0
    lda (VP),y
    ldx #0
    jsr xcomp
    ldy #2
    lda (VP),y
    ldx #1
    jsr xcomp
    ldy #1
    lda (VP),y
    ldx #2
    jsr xcomp
    ; P_j = org_j + bytes 1-2 of S_j
    ldx I
    clc
    lda SA+1
    adc ORGX
    sta PX
    sta VXL,x
    lda SA+2
    adc ORGX+1
    sta PX+1
    sta VXH,x
    clc
    lda SA+4
    adc ORGX+2
    sta PY
    sta VYL,x
    lda SA+5
    adc ORGX+3
    sta PY+1
    sta VYH,x
    clc
    lda SA+7
    adc ORGX+4
    sta PZ
    sta VZL,x
    lda SA+8
    adc ORGX+5
    sta PZ+1
    sta VZH,x
    ; project when z >= NEARZ
    bmi 6f
    bne 5f
    lda PZ
    cmp #NEARZ
    bcc 6f
5:  jsr project
    ldx I
    lda S1
    sta SXL,x
    lda S1+1
    sta SXH,x
    lda S1+2
    sta SYL,x
    lda S1+3
    sta SYH,x
6:  clc
    lda VP
    adc #3
    sta VP
    bcc 7f
    inc VP+1
7:  inc I
    jmp xloop

; S_j += c * m'[j][k] for row \j: |m'| from MAB (Y = 2k at JIDX), its sign from MSG (KCOL = k).
.macro XROW j
    ldy JIDX
    lda MAB+6*\j+1,y
    sta XT1
    lda MAB+6*\j,y
    tay
    XPROD XP0, XP1
    ldy XT1
    XPROD XP2, XP3
    clc
    lda XP1
    adc XP2
    sta XP1
    lda XP3
    adc #0
    sta XP2
    ldx KCOL
    lda MSG+3*\j,x
    eor CSGN
    bmi 1f
    clc
    lda SA+3*\j
    adc XP0
    sta SA+3*\j
    lda SA+3*\j+1
    adc XP1
    sta SA+3*\j+1
    lda SA+3*\j+2
    adc XP2
    sta SA+3*\j+2
    jmp 2f
1:  sec
    lda SA+3*\j
    sbc XP0
    sta SA+3*\j
    lda SA+3*\j+1
    sbc XP1
    sta SA+3*\j+1
    lda SA+3*\j+2
    sbc XP2
    sta SA+3*\j+2
2:
.endm

; Component A (i8) of column X into the three sums: S_j += c * m'[j][X].
xcomp:
    cmp #0
    bne 1f
    rts
1:  stx KCOL
    sta CSGN
    bpl 2f
    eor #$ff
    clc
    adc #1
2:  sta PLA
    sta PHA
    eor #$ff
    sta NLA
    sta NHA
    txa
    asl
    sta JIDX
    XROW 0
    XROW 1
    XROW 2
    rts

; ---------------------------------------------------------------------------------------------------
; geo_affine: a far object's vertices by its weak-perspective map (`geom::affine`, `geom::xform_affine`).
; Once: the centre projected (csx, csy; rx, ry, e as `project` makes them), ex, ey (Q7) from it, and
; g = (1280/z)(m0 - ex m2) 2^(t-8), (2128/z)(ey m2 - m1) 2^(t-8) by the reciprocal's top byte, all
; 8x16 products of magnitudes with shifts toward zero. Per vertex c = (a, h, b):
; sx = csx + (g0 . c) >> 6, sy = csy + (g1 . c) >> 6 with 16-bit sums (they fit: `geom::affine`), each
; |c| * |g| two quarter-square lookups (only the low byte of the high one), and z = the centre's.

AGL    = $8F60          ; |g| lo, 6 (x row k = 0..2, then the y row)
AGH    = $8F66          ; |g| hi
AGS    = $8F6C          ; g's sign bytes
ACX    = $8F72          ; (2) the centre's sx
ACY    = $8F74          ; (2) sy
ARX    = $8F76          ; rx >> 8
ARY    = $8F77          ; ry >> 8
AEXM   = $8F78          ; |ex| (Q7, at most 255)
AEXS   = $8F79          ; its sign
AEYM   = $8F7A          ; |ey|
AEYS   = $8F7B
ASHX   = $8F7C          ; 15 + e - t
ASHY   = $8F7D          ; 14 + e - t
AK     = $8F7E          ; k
AKX    = $8F7F          ; 2k
AM2L   = $8F81          ; |m2k|
AM2H   = $8F82
AM2S   = $8F83          ; its sign
MT0    = $F6            ; (2) a 16-bit magnitude
MT1    = $F7
MS     = $F8            ; its sign
AP0    = $F9            ; (3) a product
AP1    = $FA
AP2    = $FB
AS0    = $C3            ; (2) the x sum
AS1    = $C5            ; (2) the y sum
AHI    = $C7            ; the sum's sign byte, shifting

; g[r][k] (`geom::affine`): T = |e| * |m2k| >> 7 with the sign of e * m2k; D = m[r][k] - T (16-bit);
; a = D for the x row, -D for the y row; |g| = (|a| * \rm) >> \sh, sign a's. AK = k, AKX = 2k.
.macro AENT r, em, es, rm, sh
    lda AM2L
    sta MT0
    lda AM2H
    sta MT1
    lda \em
    jsr m816
    asl AP0
    rol AP1
    rol AP2
    lda \es
    eor AM2S
    bpl 1f
    sec
    lda #0
    sbc AP1
    sta AP1
    lda #0
    sbc AP2
    sta AP2
1:  ldx AKX
    sec
    lda MROT+6*\r,x
    sbc AP1
    sta MT0
    lda MROT+6*\r+1,x
    sbc AP2
    sta MT1
    sta MS
    bpl 2f
    sec
    lda #0
    sbc MT0
    sta MT0
    lda #0
    sbc MT1
    sta MT1
2:  .if \r
    lda MS
    eor #$80
    sta MS
    .endif
    lda \rm
    jsr m816
    ldx \sh
    jsr shr24
    ldx AK
    lda AP0
    sta AGL+3*\r,x
    lda AP1
    sta AGH+3*\r,x
    lda MS
    sta AGS+3*\r,x
.endm

.section .text.geo_affine,"ax",@progbits
.globl geo_affine
.type geo_affine,@function
geo_affine:
    ldx #5
1:  lda ORGX,x
    sta PX,x
    dex
    bpl 1b
    jsr project
    lda S1
    sta ACX
    lda S1+1
    sta ACX+1
    lda S1+2
    sta ACY
    lda S1+3
    sta ACY+1
    lda BH              ; ry >> 8 (project leaves ry in BL:BH)
    sta ARY
    ; rx >> 8 = (r << 2 + r) >> 11
    lda RL
    sta Q
    lda RH
    sta Q+1
    lda #0
    sta Q+2
    asl Q
    rol Q+1
    rol Q+2
    asl Q
    rol Q+1
    rol Q+2
    clc
    lda Q
    adc RL
    lda Q+1
    adc RH
    sta Q+1
    lda Q+2
    adc #0
    .rept 3
    lsr
    ror Q+1
    .endr
    lda Q+1
    sta ARX
    ; the shifts: 15 + e - t, 14 + e - t
    clc
    lda E
    adc #14
    sec
    sbc TT
    sta ASHY
    clc
    adc #1
    sta ASHX
    ; ex = (sx - 1280) * 205 >> 11, ey = (CY16 - sy) * 123 >> 11: magnitudes clamped to 255, signs
    sec
    lda ACX
    sbc #<1280
    sta MT0
    lda ACX+1
    sbc #>1280
    sta MT1
    lda #205
    jsr emag
    sta AEXM
    lda MS
    sta AEXS
    sec
    lda #<CY16
    sbc ACY
    sta MT0
    lda #>CY16
    sbc ACY+1
    sta MT1
    lda #123
    jsr emag
    sta AEYM
    lda MS
    sta AEYS
    ; per column k: |m2k|, then g0k and g1k
    lda #0
    sta AK
acol:
    asl
    sta AKX
    clc
    adc #12
    tax
    lda MROT+1,x
    sta AM2S
    bpl 4f
    sec
    lda #0
    sbc MROT,x
    sta AM2L
    lda #0
    sbc MROT+1,x
    sta AM2H
    jmp 5f
4:  lda MROT,x
    sta AM2L
    lda MROT+1,x
    sta AM2H
5:  AENT 0, AEXM, AEXS, ARX, ASHX
    AENT 1, AEYM, AEYS, ARY, ASHY
    inc AK
    lda AK
    cmp #3
    beq 3f
    jmp acol
3:  ; the vertices
    lda VPTR
    sta VP
    lda VPTR+1
    sta VP+1
    lda #0
    sta I
aloop:
    lda I
    cmp NV
    bcc 1f
    rts
1:  lda #0
    sta AS0
    sta AS0+1
    sta AS1
    sta AS1+1
    ldy #0
    lda (VP),y
    ldx #0
    jsr acomp
    ldy #2
    lda (VP),y
    ldx #1
    jsr acomp
    ldy #1
    lda (VP),y
    ldx #2
    jsr acomp
    ldx I
    lda ORGX+4
    sta VZL,x
    lda ORGX+5
    sta VZH,x
    ; sx = csx + AS0 >> 6
    lda AS0+1
    ora #$7f
    bmi 2f
    lda #0
2:  sta AHI
    asl AS0
    rol AS0+1
    rol AHI
    asl AS0
    rol AS0+1
    rol AHI
    clc
    lda AS0+1
    adc ACX
    sta SXL,x
    lda AHI
    adc ACX+1
    sta SXH,x
    ; sy = csy + AS1 >> 6
    lda AS1+1
    ora #$7f
    bmi 3f
    lda #0
3:  sta AHI
    asl AS1
    rol AS1+1
    rol AHI
    asl AS1
    rol AS1+1
    rol AHI
    clc
    lda AS1+1
    adc ACY
    sta SYL,x
    lda AHI
    adc ACY+1
    sta SYH,x
    clc
    lda VP
    adc #3
    sta VP
    bcc 4f
    inc VP+1
4:  inc I
    jmp aloop

; AS_r += c * g[r][k] for row \r: |g| from AGL/AGH (X = k), its sign from AGS; c's sign in CSGN.
.macro AROW r, sum
    ldy AGH+3*\r,x
    lda (PLA),y
    sec
    sbc (NLA),y
    sta XT1             ; low byte of |c| * hi
    ldy AGL+3*\r,x
    XPROD XP0, XP1
    clc
    lda XP1
    adc XT1
    sta XP1
    lda AGS+3*\r,x
    eor CSGN
    bmi 1f
    clc
    lda \sum
    adc XP0
    sta \sum
    lda \sum+1
    adc XP1
    sta \sum+1
    jmp 2f
1:  sec
    lda \sum
    sbc XP0
    sta \sum
    lda \sum+1
    sbc XP1
    sta \sum+1
2:
.endm

; Component A (i8) of column X into the two sums.
acomp:
    cmp #0
    bne 1f
    rts
1:  sta CSGN
    bpl 2f
    eor #$ff
    clc
    adc #1
2:  sta PLA
    sta PHA
    eor #$ff
    sta NLA
    sta NHA
    AROW 0, AS0
    AROW 1, AS1
    rts

; A = min(|MT0:MT1 * A| >> 11, 255) (MT signed, A an unsigned constant), MS = MT's sign
emag:
    pha
    lda MT1
    sta MS
    bpl 1f
    sec
    lda #0
    sbc MT0
    sta MT0
    lda #0
    sbc MT1
    sta MT1
1:  pla
    jsr m816
    .rept 3
    lsr AP2
    ror AP1
    .endr
    lda AP2
    beq 2f
    lda #255
    rts
2:  lda AP1
    rts

; AP0:AP2 = A (u8) * MT0:MT1 (u16), by the tables
m816:
    cmp #0
    bne 1f
    sta AP0
    sta AP1
    sta AP2
    rts
1:  sta PLA
    sta PHA
    eor #$ff
    sta NLA
    sta NHA
    ldy MT0
    XPROD AP0, AP1
    ldy MT1
    XPROD XP0, XP1
    clc
    lda AP1
    adc XP0
    sta AP1
    lda XP1
    adc #0
    sta AP2
    rts

; AP0:AP2 >>= X (0..15)
shr24:
    cpx #8
    bcc 1f
    lda AP1
    sta AP0
    lda AP2
    sta AP1
    lda #0
    sta AP2
    txa
    sbc #8
    tax
1:  cpx #0
    beq 3f
2:  lsr AP2
    ror AP1
    ror AP0
    dex
    bne 2b
3:  rts
.size geo_affine, .-geo_affine

.section .text.geo_faces,"ax",@progbits
.globl geo_faces
.type geo_faces,@function
geo_faces:
    lda #0
    sta WNCLIP
    sta F
    lda FPTR
    sta FP
    lda FPTR+1
    sta FP+1
floop:
    lda F
    cmp NF
    bcc 1f
    ; Faces straddling the near plane: cut at it and pushed (Rust `geo_clip`, in the second window).
    lda WNCLIP
    beq 5f
    jmp geo_clip
5:  rts
1:  ldy #0
    lda (FP),y
    sta IA
    iny
    lda (FP),y
    sta IB
    iny
    lda (FP),y
    sta IC
    iny
    lda (FP),y
    sta SHD
    lda #0
    sta CNT
    ldx IA
    jsr front
    ldx IB
    jsr front
    ldx IC
    jsr front
    lda CNT
    cmp #3
    bne 2f
    jsr tryface
    jmp 3f
2:  cmp #0
    beq 3f
    ldx WNCLIP
    lda F
    sta CLIP,x
    inc WNCLIP
3:  clc
    lda FP
    adc #4
    sta FP
    bcc 4f
    inc FP+1
4:  inc F
    jmp floop

front:
    lda VZH,x
    bmi 2f
    bne 1f
    lda VZL,x
    cmp #NEARZ
    bcc 2f
1:  inc CNT
2:  rts

; Twice the screen area of triangle IA, IB, IC (`geom::area`), corners loaded into TX/TY: V+0..2 and
; A (its top byte, N its sign).
farea:
    ldx IA
    lda SXL,x
    sta TXL
    lda SXH,x
    sta TXH
    lda SYL,x
    sta TYL
    lda SYH,x
    sta TYH
    ldx IB
    lda SXL,x
    sta TXL+1
    lda SXH,x
    sta TXH+1
    lda SYL,x
    sta TYL+1
    lda SYH,x
    sta TYH+1
    ldx IC
    lda SXL,x
    sta TXL+2
    lda SXH,x
    sta TXH+2
    lda SYL,x
    sta TYL+2
    lda SYH,x
    sta TYH+2
    ; Q = (x1 - x0) * (y2 - y0)
    sec
    lda TXL+1
    sbc TXL
    sta AL
    lda TXH+1
    sbc TXH
    sta AH
    sec
    lda TYL+2
    sbc TYL
    sta BL
    lda TYH+2
    sbc TYH
    sta BH
    jsr smul
    lda P0
    sta Q
    lda P1
    sta Q+1
    lda P2
    sta Q+2
    lda P3
    sta Q+3
    ; P = (y1 - y0) * (x2 - x0)
    sec
    lda TYL+1
    sbc TYL
    sta AL
    lda TYH+1
    sbc TYH
    sta AH
    sec
    lda TXL+2
    sbc TXL
    sta BL
    lda TXH+2
    sbc TXH
    sta BH
    jsr smul
    ; area = Q - P must be > 0
    sec
    lda Q
    sbc P0
    sta V
    lda Q+1
    sbc P1
    sta V+1
    lda Q+2
    sbc P2
    sta V+2
    lda Q+3
    sbc P3
    rts

; One triangle of vertices IA, IB, IC with shade SHD (Rust `geom::clip`'s cuts).
.globl geo_tri
geo_tri:
tryface:
    jsr farea
    bpl 7f
    rts                 ; (back-facing)
7:  ora V+2
    ora V+1
    ora V
    beq 9f
    jsr facekey
    sta KEY
    ; beyond the fog (z sum >= 3 * FAR): the far shade (`geom::far_shade`)
    lda V+2
    bmi 6f
    bne 5f
    lda V+1
    cmp #>(3 * FARQ)
    bcc 6f
5:  lda #HAZED | HAZE_PAT
    sta SHD
6:  lda WN
    cmp #255
    bne 1f
    lda WDROP
    cmp #255
    beq 9f
    inc WDROP
    rts
1:  tax
    lda TXL
    sta TX0L,x
    lda TXH
    sta TX0H,x
    lda TXL+1
    sta TX1L,x
    lda TXH+1
    sta TX1H,x
    lda TXL+2
    sta TX2L,x
    lda TXH+2
    sta TX2H,x
    lda TYL
    sta TY0L,x
    lda TYH
    sta TY0H,x
    lda TYL+1
    sta TY1L,x
    lda TYH+1
    sta TY1H,x
    lda TYL+2
    sta TY2L,x
    lda TYH+2
    sta TY2H,x
    lda SHD
    sta TSHADE,x
    ldy KEY
    lda HEAD,y
    sta NEXT,x
    txa
    sta HEAD,y
    inc WN
9:  rts

; A = the depth key of z sum s = za + zb + zc (`geom::depth_key`): s >> 5 below 6144, then
; 192 + (s - 6144) >> 8 below 3 FAR (15360), then FAR_KEY + (s - 15360) >> 10, clamped to 0..255.
; (V keeps s.)
facekey:
    ldx IA
    lda VZL,x
    sta V
    lda VZH,x
    sta V+1
    ora #$7f
    bmi 1f
    lda #0
1:  sta V+2
    ldx IB
    jsr addz
    ldx IC
    jsr addz
    lda V+2
    bmi 3f
    bne 2f
    lda V+1
    cmp #$18
    bcs 4f
    asl
    asl
    asl
    sta Q
    lda V
    lsr
    lsr
    lsr
    lsr
    lsr
    ora Q
    rts
4:  cmp #$3c
    bcs 6f
    adc #192 - $18       ; (carry clear) 192 + (s >> 8) - 24
    rts
6:  sbc #$3c             ; (carry set) (s >> 8) - 60, quartered: (s - 15360) >> 10
    lsr
    lsr
    clc
    adc #FAR_KEY
    bcc 5f
2:  lda #255
5:  rts
3:  lda #0
    rts

addz:
    clc
    lda V
    adc VZL,x
    sta V
    lda V+1
    adc VZH,x
    sta V+1
    lda VZH,x
    ora #$7f
    bmi 1f
    lda #0
1:  adc V+2
    sta V+2
    rts
.size geo_faces, .-geo_faces

; ---------------------------------------------------------------------------------------------------
; CSWAP i, j: if y[i] > y[j] swap corners i and j.
.macro CSWAP i, j
    lda TYL+\j
    cmp TYL+\i
    lda TYH+\j
    sbc TYH+\i
    bvc 1f
    eor #$80
1:  bpl 2f
    ldx TXL+\i
    lda TXL+\j
    sta TXL+\i
    stx TXL+\j
    ldx TXH+\i
    lda TXH+\j
    sta TXH+\i
    stx TXH+\j
    ldx TYL+\i
    lda TYL+\j
    sta TYL+\i
    stx TYL+\j
    ldx TYH+\i
    lda TYH+\j
    sta TYH+\i
    stx TYH+\j
2:
.endm

; LINEAT k: LA = (y[k] + 7) >> 4 (signed)
.macro LINEAT k
    clc
    lda TYL+\k
    adc #7
    sta LA
    lda TYH+\k
    adc #0
    sta LA+1
    .rept 4
    lda LA+1
    cmp #$80
    ror LA+1
    ror LA
    .endr
.endm

tsetup:
    ldx IDX
    lda TX0L,x
    sta TXL
    lda TX0H,x
    sta TXH
    lda TX1L,x
    sta TXL+1
    lda TX1H,x
    sta TXH+1
    lda TX2L,x
    sta TXL+2
    lda TX2H,x
    sta TXH+2
    lda TY0L,x
    sta TYL
    lda TY0H,x
    sta TYH
    lda TY1L,x
    sta TYL+1
    lda TY1H,x
    sta TYH+1
    lda TY2L,x
    sta TYL+2
    lda TY2H,x
    sta TYH+2
    lda TSHADE,x
    and #15
    tax
    lda PATE,x
    sta POLY_PE
    lda PATO,x
    sta POLY_PO
    CSWAP 0, 1
    CSWAP 1, 2
    CSWAP 0, 1
    ; first = max(line_at(y0), 0); none past 255
    LINEAT 0
    lda LA+1
    bmi 1f
    beq 2f
    clc
    rts                 ; first >= 256: below the screen
1:  lda #0
    sta LA
2:  lda LA
    sta FIRST
    ; end = min(line_at(y2), LINES); none if negative
    LINEAT 2
    lda LA+1
    bpl 3f
    clc
    rts
3:  bne 4f
    lda LA
    cmp #201
    bcc 5f
4:  lda #LINES
    sta LA
5:  lda LA
    sta ENDL
    lda FIRST
    cmp ENDL
    bcc 6f
    clc
    rts                 ; first >= end
6:  ; mid = clamp(line_at(y1), first, end)
    LINEAT 1
    lda LA+1
    bmi 7f
    bne 8f
    lda LA
    cmp FIRST
    bcc 7f
    cmp ENDL
    bcs 8f
    sta MIDL
    jmp 9f
7:  lda FIRST
    sta MIDL
    jmp 9f
8:  lda ENDL
    sta MIDL
9:
    ; long = edge(0, 2, first)
    ldx #0
    ldy #2
    lda FIRST
    jsr edge
    ldx #0
    jsr putedge
    ; short = first < mid ? edge(0, 1, first) : 0
    lda FIRST
    cmp MIDL
    bcs 10f
    ldx #0
    ldy #1
    jsr edge
    jmp 11f
10: jsr zeroedge
11: ldx #8
    jsr putedge
    ; lower = mid < end ? edge(1, 2, mid) : 0
    lda MIDL
    cmp ENDL
    bcs 12f
    ldx #1
    ldy #2
    lda MIDL
    jsr edge
    jmp 13f
12: jsr zeroedge
13: ldx #16
    jsr putedge
    lda FIRST
    sta TRI_ARGS+24
    lda MIDL
    sta TRI_ARGS+25
    lda ENDL
    sta TRI_ARGS+26
    sec                 ; visible: TRI_ARGS and the pattern are set
    rts

zeroedge:
    lda #0
    ldx #3
1:  sta EX,x
    sta ES,x
    dex
    bpl 1b
    rts

; TRI_ARGS + X <- EX + $7FFF (x biased for tri.S's PIXEL, 4 bytes), ES (step, 4 bytes)
putedge:
    clc
    lda EX
    adc #$ff
    sta TRI_ARGS,x
    lda EX+1
    adc #$7f
    sta TRI_ARGS+1,x
    lda EX+2
    adc #0
    sta TRI_ARGS+2,x
    lda EX+3
    adc #0
    sta TRI_ARGS+3,x
    lda ES
    sta TRI_ARGS+4,x
    lda ES+1
    sta TRI_ARGS+5,x
    lda ES+2
    sta TRI_ARGS+6,x
    lda ES+3
    sta TRI_ARGS+7,x
    rts

; edge from corner X to corner Y at line A (`geom::Edge::new`): EX = x, ES = step.
edge:
    sta LINE
    lda TXL,x
    sta XAV
    lda TXH,x
    sta XAV+1
    lda TYL,x
    sta YAV
    lda TYH,x
    sta YAV+1
    lda TXL,y
    sta XBV
    lda TXH,y
    sta XBV+1
    lda TYL,y
    sta YBV
    lda TYH,y
    sta YBV+1
    ; per = ((xb - xa) * recip(norm(yb - ya))) >> (14 + e)
    sec
    lda YBV
    sbc YAV
    sta M
    lda YBV+1
    sbc YAV+1
    sta M+1
    jsr norm
    sec
    lda XBV
    sbc XAV
    sta AL
    lda XBV+1
    sbc XAV+1
    sta AH
    jsr setr
    jsr smulsu
    jsr pv
    lda E
    clc
    adc #14
    jsr sarv
    ; PER = V; ES = V << 4 (by nibbles)
    lda V
    sta PER
    asl
    asl
    asl
    asl
    sta ES
    lda V+1
    sta PER+1
    asl
    asl
    asl
    asl
    sta ES+1
    lda V
    lsr
    lsr
    lsr
    lsr
    ora ES+1
    sta ES+1
    lda V+2
    sta PER+2
    asl
    asl
    asl
    asl
    sta ES+2
    lda V+1
    lsr
    lsr
    lsr
    lsr
    ora ES+2
    sta ES+2
    lda V+3
    sta PER+3
    asl
    asl
    asl
    asl
    sta ES+3
    lda V+2
    lsr
    lsr
    lsr
    lsr
    ora ES+3
    sta ES+3
    ; d = line * 16 + 8 - ya
    lda LINE
    lsr
    lsr
    lsr
    lsr
    sta DD+1
    lda LINE
    asl
    asl
    asl
    asl
    ora #8              ; + 8: the low nibble is clear
    sec
    sbc YAV
    sta DD
    sta AL
    lda DD+1
    sbc YAV+1
    sta DD+1
    sta AH
    ; EX = d * (per & $ffff) + (d * (per >> 16)) << 16
    lda PER
    sta BL
    lda PER+1
    sta BH
    jsr umul
    lda P0
    sta EX
    lda P1
    sta EX+1
    lda P2
    sta EX+2
    lda P3
    sta EX+3
    ; per >> 16: 0 adds nothing, $FFFF subtracts d (mod 2^16), else multiply
    lda PER+2
    and PER+3
    cmp #$ff
    bne 1f
    sec
    lda EX+2
    sbc DD
    sta EX+2
    lda EX+3
    sbc DD+1
    sta EX+3
    jmp 3f
1:  lda PER+2
    ora PER+3
    beq 3f
    lda PER+2
    sta BL
    lda PER+3
    sta BH
    jsr umul
    clc
    lda EX+2
    adc P0
    sta EX+2
    lda EX+3
    adc P1
    sta EX+3
3:  ; + xa << 12: bytes 1-3 of (xa << 16) >> 4, made first (the shifts use the carry), then added
    lda XAV
    asl
    asl
    asl
    asl
    sta V
    lda XAV
    lsr
    lsr
    lsr
    lsr
    sta V+1
    lda XAV+1
    asl
    asl
    asl
    asl
    ora V+1
    sta V+1
    lda XAV+1
    cmp #$80
    ror
    cmp #$80
    ror
    cmp #$80
    ror
    cmp #$80
    ror
    sta V+2
    clc
    lda EX+1
    adc V
    sta EX+1
    lda EX+2
    adc V+1
    sta EX+2
    lda EX+3
    adc V+2
    sta EX+3
    rts
.size tsetup, .-tsetup

; Band rendering (`geom::bin` / `geom::raster_bands`): the C64 draws the screen a cell row (band) at a
; time into STAGE and publishes it, so every triangle is set up when its first band comes and walked one
; row per band from a slot pool, in painter's order.
;
;   geo_bin():          painter's order -> sequence numbers; each triangle filed under its first band
;   geo_activate(band): set up the band's newcomers into free slots (pool full: dropped), merge them
;                       into the active list by sequence; A = triangles active
;   geo_draw(band):     walk each active triangle's row of this band into STAGE; retire the finished
;
; State lives in RAM at BAND_RAM ($A000): the floor kernel runs between these calls and uses zero page
; $A0-$FB.

POOL   = 96
SLOT   = $A000          ; 30 fields x POOL: TRI_ARGS 0..26 (edges, first, mid, end), pe, po, seq
F_PE   = 27
F_PO   = 28
F_SEQ  = 29
ACT    = $AB40          ; active slots, in painter's order
FREE   = $ABA0          ; free slot stack
NEWL   = $AC00          ; this band's newcomers
MERGE  = $AC60
FHEAD  = $ACC0          ; first sequence number of each band's list ($FF: none)
FTAIL  = $ACE0
NACT   = $ACFA
NFREE  = $ACFB
NSEQ   = $ACFC
TRISC  = $ACFD          ; triangles set up this frame
DROPC  = $ACFE          ; triangles dropped (pool full)
FSHD   = $ACF8          ; the activating triangle's shade (tsetup's zero page holds IDX)
FLAST  = $ACF9          ; a far triangle's last band
CAP    = $ACD0          ; per band: the pool slots no near item will need (`geom::Bands`), 16 bytes
BNEXT  = $AD00          ; next sequence number in the same band list
SEQTRI = $AE00          ; sequence number -> triangle index
STAGE  = $8C00

; Zero page $B8-$BF: tsetup uses $C0-$FF, tri_row / poly_row $A0-$B3, band_publish $B4-$B7.
BAND   = $B8
NNEW   = $B9
KK     = $BA
KEPT   = $BB
OO     = $BC
JJ     = $BD
LIMIT  = $BE
BS     = $BF            ; current sequence number

.section .text.geo_bin,"ax",@progbits
.globl geo_bin
.type geo_bin,@function
geo_bin:
    lda #$ff
    ldx #NBANDS - 1
1:  sta FHEAD,x
    dex
    bpl 1b
    lda #0
    ldx #NBANDS - 1
1:  sta CAP,x
    dex
    bpl 1b
    ldx #POOL - 1
2:  txa
    sta FREE,x
    dex
    bpl 2b
    lda #POOL
    sta NFREE
    lda #0
    sta NACT
    sta NSEQ
    sta TRISC
    sta DROPC
    sta TRIS
    lda #NBANDS
    sta TOP
    lda #255            ; the clouds' bucket, then beyond the fog nearest first (`geom::bin`)
    sta KEY
bkey:
    ldx KEY
    lda HEAD,x
btri:
    cmp #$ff
    bne 3f
    jmp bnext
3:  sta IDX
    ldx NSEQ
    sta SEQTRI,x
    ; A sprite's first band: its top line (whole lines, not Q4), < LINES.
    ldx IDX
    lda TSHADE,x
    bpl 20f
    lda TY0H,x
    bpl 21f
    jmp 7f              ; above the screen: band 0
21: beq 22f
    jmp 9f              ; >= 256: below
22: lda TY0L,x
    cmp #LINES
    bcc 23f
    jmp 9f
23: lsr
    lsr
    lsr
    jmp 8f
20:
    ; A triangle's first band: line_at(min y), < LINES
    ldx IDX
    lda TY0L,x
    sta TYL
    lda TY0H,x
    sta TYH
    lda TY1L,x
    sta TYL+1
    lda TY1H,x
    sta TYH+1
    lda TY2L,x
    sta TYL+2
    lda TY2H,x
    sta TYH+2
    ; TYL/TYH <- min of the three (signed)
    ldy #1
4:  lda TYL,y
    cmp TYL
    lda TYH,y
    sbc TYH
    bvc 5f
    eor #$80
5:  bpl 6f
    lda TYL,y
    sta TYL
    lda TYH,y
    sta TYH
6:  iny
    cpy #3
    bne 4b
    LINEAT 0
    lda LA+1
    bmi 7f              ; above the screen: band 0
    bne 9f              ; >= 256: below
    lda LA
    cmp #LINES
    bcs 9f
    lsr
    lsr
    lsr
    jmp 8f
7:  lda #0
8:  sta BAND
    tax
    ldy NSEQ
    lda #$ff
    sta BNEXT,y
    lda FHEAD,x
    cmp #$ff
    bne 10f
    tya
    sta FHEAD,x
    jmp 11f
10: lda FTAIL,x
    tax
    tya
    sta BNEXT,x
    ldx BAND
11: tya
    sta FTAIL,x
    ; a near item: one more starting in this band (CAP counts them, then `bcap` makes them slots); far
    ; ones (HAZED: a triangle beyond the fog, or a cloud sprite) don't count
    ldy IDX
    lda TSHADE,y
    and #HAZED
    bne 9f
    inc CAP,x
9:  inc NSEQ
    ldx IDX
    lda NEXT,x
    jmp btri
bnext:
    lda KEY
    cmp #FAR_KEY
    bcc 13f
    cmp #255
    beq 15f             ; the clouds' bucket done: beyond the fog, nearest first
    cmp #254
    beq 14f
    inc KEY
    jmp bkey
15: lda #FAR_KEY
    sta KEY
    jmp bkey
14: lda #FAR_KEY - 1
    sta KEY
    jmp bkey
13: cmp #0
    beq 12f
    dec KEY
    jmp bkey
12: ; CAP[b] = POOL less the near items starting in band b or above (as if they stayed to the bottom)
    ldx #0
    lda #POOL
bcap:
    sec
    sbc CAP,x
    bcs 15f
    lda #0
15: sta CAP,x
    inx
    cpx #NBANDS
    bne bcap
    rts

.size geo_bin, .-geo_bin

.section .text.geo_activate,"ax",@progbits
.globl geo_activate
.type geo_activate,@function
geo_activate:
    sta BAND
    lda #0
    sta NNEW
    ldx BAND
    lda FHEAD,x
anext:
    cmp #$ff
    bne 1f
    jmp amerge
1:  sta BS
    tax
    lda SEQTRI,x
    sta IDX
    tax
    lda TSHADE,x
    sta FSHD
    bpl 15f
    jsr ssetup
    jmp 16f
15: jsr tsetup
16:
    bcs 13f
    jmp 3f              ; nothing to draw (long branches as bxx/jmp)
13: ; a far item (a triangle beyond the fog, or a cloud sprite) needs a slot in CAP in every band it
    ; spans (`geom::Bands::band`)
    lda FSHD
    and #HAZED
    beq 17f
    lda TRI_ARGS + 26   ; its end line (exclusive; a sprite's slot keeps it there too): last band (end - 1) >> 3
    sec
    sbc #1
    lsr
    lsr
    lsr
    cmp #NBANDS
    bcc 19f
    lda #NBANDS - 1
19: sta FLAST
    ldx BAND
20: lda CAP,x
    beq 22f
    cpx FLAST
    inx
    bcc 20b
    lda NFREE
    beq 22f
    ldx BAND
21: dec CAP,x
    cpx FLAST
    inx
    bcc 21b
    bcs 2f
22: jmp 3f              ; a far item left out: by design, not a drop (`raster_bands` counts near ones)
17: lda NFREE
    bne 2f
18:
    lda DROPC
    cmp #$ff
    bne 14f
    jmp 3f
14: inc DROPC
    jmp 3f
2:  dec NFREE
    ldx NFREE
    lda FREE,x
    tax                 ; the slot
    .irp f, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26
    lda TRI_ARGS + \f
    sta SLOT + \f * POOL,x
    .endr
    lda POLY_PE
    sta SLOT + F_PE * POOL,x
    lda POLY_PO
    sta SLOT + F_PO * POOL,x
    lda BS
    sta SLOT + F_SEQ * POOL,x
    txa
    ldx NNEW
    sta NEWL,x
    inc NNEW
    inc TRISC
3:  ldx BS
    lda BNEXT,x
    jmp anext
amerge:
    ; MERGE <- ACT and NEWL by sequence; then back to ACT
    lda #0
    sta KK
    sta JJ
    sta OO
4:  lda KK
    cmp NACT
    bcs 7f              ; actives used up: take a newcomer
    lda JJ
    cmp NNEW
    bcs 6f              ; newcomers used up: take an active
    ldx KK
    ldy ACT,x
    lda SLOT + F_SEQ * POOL,y
    ldx JJ
    ldy NEWL,x
    cmp SLOT + F_SEQ * POOL,y
    bcs 7f
6:  ldx KK
    lda ACT,x
    inc KK
    jmp 8f
7:  lda JJ
    cmp NNEW
    bcs 9f              ; both used up
    ldx JJ
    lda NEWL,x
    inc JJ
8:  ldx OO
    sta MERGE,x
    inc OO
    jmp 4b
9:  ldx OO
    stx NACT
    dex
    bmi 11f
10: lda MERGE,x
    sta ACT,x
    dex
    bpl 10b
11: lda NACT
    beq 12f
    lda BAND
    cmp TOP
    bcs 12f
    sta TOP
12: lda TRISC
    sta TRIS
    lda NACT
    rts
.size geo_activate, .-geo_activate

.section .text.geo_draw,"ax",@progbits
.globl geo_draw
.type geo_draw,@function
geo_draw:
    sta BAND
    ; tri_row draws into BMP + band * 320: point it at STAGE
    tax
    sec
    lda #<STAGE
    sbc row320lo,x
    sta TRI_ARGS+27
    lda #>STAGE
    sbc row320hi,x
    sta TRI_ARGS+28
    lda BAND
    asl
    asl
    asl
    clc
    adc #8
    sta LIMIT
    lda #0
    sta KK
    sta KEPT
dloop:
    lda KK
    cmp NACT
    bcc 1f
    lda KEPT
    sta NACT
    rts
1:  tax
    lda ACT,x
    tax
    lda SLOT + 24 * POOL,x
    cmp #$ff
    bne 4f
    jsr sprite_band     ; a sprite: its lines in this band
    ldx KK
    lda ACT,x
    tax
    jmp 5f
4:
    .irp f, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26
    lda SLOT + \f * POOL,x
    sta TRI_ARGS + \f
    .endr
    lda SLOT + F_PE * POOL,x
    sta POLY_PE
    lda SLOT + F_PO * POOL,x
    sta POLY_PO
    lda BAND
    jsr tri_row
    ; edge state back: long x, short x and step (short may have become lower)
    ldx KK
    lda ACT,x
    tax
    .irp f, 0,1,2,3,8,9,10,11,12,13,14,15
    lda TRI_ARGS + \f
    sta SLOT + \f * POOL,x
    .endr
5:  ; keep it while it ends below this band
    lda LIMIT
    cmp SLOT + 26 * POOL,x
    bcs 2f
    txa
    ldx KEPT
    sta ACT,x
    inc KEPT
    jmp 3f
2:  txa
    ldx NFREE
    sta FREE,x
    inc NFREE
3:  inc KK
    jmp dloop
.size geo_draw, .-geo_draw

; ---------------------------------------------------------------------------------------------------
; Sprites (`sprite::place`, `sprite::sprite_band`): items with TSHADE bit 7. x0 = left pixel, x1 / x2 =
; the phase block (REU, lo 16, bank | band shift << 8), y0 = top line, y1 = height, y2 = byte columns.
; A sprite's slot: 0-2 block, 3 band shift, 4 height, 5-6 top, 7 first on-screen column of the sprite,
; 8 columns, 9 screen column, 24 = $FF (a sprite), 26 end line.

SPRITE_BSTART = $8D80
SPRITE_OFFS   = $8D90
SPRITE_CHUNK  = $5400
JOB           = $8D70

; zero page (geo_draw's callees: tri_row uses $A0-$B3, draw_cols_asm $F0-$FF)
SBL   = $C0             ; (2) left >> 2, the sprite's first byte column on screen
J0    = $C2             ; (2)
J1    = $C4             ; (2)
SBOT  = $C6             ; (2) top + height

; ssetup: sprite IDX's screen clip into TRI_ARGS (the slot layout above); carry = visible.
.section .text.ssetup,"ax",@progbits
ssetup:
    ldx IDX
    lda TX1L,x
    sta TRI_ARGS+0
    lda TX1H,x
    sta TRI_ARGS+1
    lda TX2L,x
    sta TRI_ARGS+2
    lda TX2H,x
    sta TRI_ARGS+3
    lda TY1L,x
    sta TRI_ARGS+4
    lda TY0L,x
    sta TRI_ARGS+5
    lda TY0H,x
    sta TRI_ARGS+6
    ; base = left >> 2
    lda TX0L,x
    sta SBL
    lda TX0H,x
    sta SBL+1
    .rept 2
    lda SBL+1
    cmp #$80
    ror SBL+1
    ror SBL
    .endr
    ; j0 = max(-base, 0)
    lda SBL+1
    bpl 1f
    sec
    lda #0
    sbc SBL
    sta J0
    lda #0
    sbc SBL+1
    sta J0+1
    jmp 2f
1:  lda #0
    sta J0
    sta J0+1
2:  ; j1 = min(wb, 39 - base) (`layout::DRAWN_COLS`)
    sec
    lda #39
    sbc SBL
    sta J1
    lda #0
    sbc SBL+1
    sta J1+1
    bmi 3f              ; negative: below wb
    bne 4f              ; >= 256: wb
    lda J1
    cmp TY2L,x
    bcc 3f
4:  lda TY2L,x
    sta J1
    lda #0
    sta J1+1
3:  ; visible columns: j0 < j1 (signed)
    lda J0
    cmp J1
    lda J0+1
    sbc J1+1
    bvc 5f
    eor #$80
5:  bmi 6f
    clc
    rts
6:  ; vertical: top < LINES and top + h > 0
    lda TY0H,x
    bmi 7f
    bne 8f
    lda TY0L,x
    cmp #LINES
    bcc 7f
8:  clc
    rts
7:  clc
    lda TY0L,x
    adc TY1L,x
    sta SBOT
    lda TY0H,x
    adc #0
    sta SBOT+1
    bmi 8b
    ora SBOT
    beq 8b
    ; end = min(top + h, LINES)
    lda SBOT+1
    bne 9f
    lda SBOT
    cmp #LINES
    bcc 10f
9:  lda #LINES
10: sta TRI_ARGS+26
    lda J0
    sta TRI_ARGS+7
    lda J1
    sec
    sbc J0
    sta TRI_ARGS+8
    lda SBL
    clc
    adc J0
    sta TRI_ARGS+9
    lda #$ff
    sta TRI_ARGS+24
    sec
    rts
.size ssetup, .-ssetup

; sprite_band(X = slot, BAND): the sprite's lines in this band into STAGE (`sprite::sprite_band`).
SPBLK  = $C8            ; (3) the phase block
SPSH   = $CB            ; band shift
SPH    = $CC            ; height
SPTOP  = $CD            ; (2) top line
SPJ0   = $CF
SPNC   = $D0
SPCOL  = $D1
SPT    = $D2            ; first sprite line drawn
SPB    = $D3            ; end sprite line
SPSB   = $D4            ; sprite band
SPEND  = $D5            ; last sprite band
SPBA   = $D6            ; (3) the sprite band's REU address
SPLO   = $D9            ; (2) band * 8 - top
DMAC   = $DB            ; (2) C64 address
DMAR   = $DD            ; (3) REU address
DMAL   = $E0            ; (2) length

.section .text.sprite_band,"ax",@progbits
sprite_band:
    lda SLOT + 0 * POOL,x
    sta SPBLK
    lda SLOT + 1 * POOL,x
    sta SPBLK+1
    lda SLOT + 2 * POOL,x
    sta SPBLK+2
    lda SLOT + 3 * POOL,x
    sta SPSH
    lda SLOT + 4 * POOL,x
    sta SPH
    lda SLOT + 5 * POOL,x
    sta SPTOP
    lda SLOT + 6 * POOL,x
    sta SPTOP+1
    lda SLOT + 7 * POOL,x
    sta SPJ0
    lda SLOT + 8 * POOL,x
    sta SPNC
    lda SLOT + 9 * POOL,x
    sta SPCOL
    ; lo = band * 8 - top
    lda BAND
    asl
    asl
    asl
    sec
    sbc SPTOP
    sta SPLO
    lda #0
    sbc SPTOP+1
    sta SPLO+1
    ; t = max(lo, 0); b = min(lo + 8, h); nothing if t >= b
    bmi 1f
    bne 9f              ; lo >= 256 > h: below the sprite
    lda SPLO
    sta SPT
    jmp 2f
1:  lda #0
    sta SPT
2:  clc
    lda SPLO
    adc #8
    sta SPB
    lda SPLO+1
    adc #0
    bmi 9f              ; lo + 8 < 0: above the sprite
    bne 3f
    lda SPB
    cmp SPH
    bcc 4f
3:  lda SPH
    sta SPB
4:  lda SPT
    cmp SPB
    bcc 5f
9:  rts
5:  ; sprite bands t >> shift ..= (b - 1) >> shift (a shift of 9: band 0)
    lda SPT
    ldy SPSH
    jsr shr8
    sta SPSB
    lda SPB
    sec
    sbc #1
    ldy SPSH
    jsr shr8
    sta SPEND
    lda #0
    sta $df0a           ; incrementing addresses (floor_row restores the floor's mode)
sbloop:
    ; the band's start: 3 bytes at block + 3 * sb
    lda SPSB
    asl
    clc
    adc SPSB
    clc
    adc SPBLK
    sta DMAR
    lda SPBLK+1
    adc #0
    sta DMAR+1
    lda SPBLK+2
    adc #0
    sta DMAR+2
    lda #<SPRITE_BSTART
    sta DMAC
    lda #>SPRITE_BSTART
    sta DMAC+1
    lda #3
    sta DMAL
    lda #0
    sta DMAL+1
    jsr dma
    clc
    lda SPBLK
    adc SPRITE_BSTART
    sta SPBA
    lda SPBLK+1
    adc SPRITE_BSTART+1
    sta SPBA+1
    lda SPBLK+2
    adc SPRITE_BSTART+2
    sta SPBA+2
    ; the on-screen columns' offsets: 2 * (ncols + 1) bytes at band + 2 * j0
    lda SPJ0
    asl
    php
    clc
    adc SPBA
    sta DMAR
    lda SPBA+1
    adc #0
    sta DMAR+1
    lda SPBA+2
    adc #0
    sta DMAR+2
    plp
    bcc 6f
    inc DMAR+1
    bne 6f
    inc DMAR+2
6:  lda #<SPRITE_OFFS
    sta DMAC
    lda #>SPRITE_OFFS
    sta DMAC+1
    lda SPNC
    clc
    adc #1
    asl
    sta DMAL
    lda #0
    rol
    sta DMAL+1
    jsr dma
    ; their records: offs[0] .. offs[ncols]
    lda SPNC
    asl
    tay
    clc
    lda SPBA
    adc SPRITE_OFFS
    sta DMAR
    lda SPBA+1
    adc SPRITE_OFFS+1
    sta DMAR+1
    lda SPBA+2
    adc #0
    sta DMAR+2
    sec
    lda SPRITE_OFFS,y
    sbc SPRITE_OFFS
    sta DMAL
    lda SPRITE_OFFS+1,y
    sbc SPRITE_OFFS+1
    sta DMAL+1
    lda #<SPRITE_CHUNK
    sta DMAC
    lda #>SPRITE_CHUNK
    sta DMAC+1
    jsr dma
    ; draw_cols_asm: DST = STAGE + col * 8, Y = top - band * 8, lines [t, b), the chunk
    lda SPCOL
    asl
    asl
    asl
    sta JOB+0           ; col * 8: low byte, then col >> 5
    lda SPCOL
    lsr
    lsr
    lsr
    lsr
    lsr
    sta JOB+1
    clc
    lda JOB+0
    adc #<STAGE
    sta JOB+0
    lda JOB+1
    adc #>STAGE
    sta JOB+1
    lda BAND
    asl
    asl
    asl
    sta SPLO
    sec
    lda SPTOP
    sbc SPLO
    sta JOB+2
    lda SPTOP+1
    sbc #0
    sta JOB+3
    lda SPT
    sta JOB+4
    lda #0
    sta JOB+5
    sta JOB+7
    lda SPB
    sta JOB+6
    lda #<SPRITE_CHUNK
    sta JOB+8
    lda #>SPRITE_CHUNK
    sta JOB+9
    lda SPNC
    sta JOB+10
    jsr draw_cols_asm
    lda SPSB
    cmp SPEND
    bcs 7f
    inc SPSB
    jmp sbloop
7:  rts

; A >> Y (Y = 0..9; >= 8 gives 0)
shr8:
    cpy #8
    bcc 1f
    lda #0
    rts
1:  cpy #0
    beq 2f
    lsr
    dey
    jmp 1b
2:  rts

; One REU fetch: DMAL bytes from DMAR (24-bit) to DMAC ($DF0A is 0). The C64U reloads the whole address
; from the values last written on any address write, so all three are written.
dma:
    lda DMAC
    sta $df02
    lda DMAC+1
    sta $df03
    lda DMAR
    sta $df04
    lda DMAR+1
    sta $df05
    lda DMAR+2
    sta $df06
    lda DMAL
    sta $df07
    lda DMAL+1
    sta $df08
    lda #$91
    sta $df01
    rts
.size sprite_band, .-sprite_band

; ---------------------------------------------------------------------------------------------------
; geo_walk(): the frame's objects, from the REU grids into the depth buckets (`geom::draw`, `grid`,
; `cell`, `object`: the same integer steps). Arguments in WALK_ARGS (`geom::walk`, written each frame
; by `geom::walk_args`); the buckets must be empty (`Work::clear`). Per grid: each row's directory
; entries fetched to DIR_RAM; each cell whose sphere may be in view and whose largest object would have
; popped in is fetched to CELL_RAM; each object culled, its rotation and origin put in GEO_ARGS, then
; geo_xform and geo_faces.
;
; Zero page: $A0-$BA the walk (kept across geo_xform / geo_faces), $C0-$DB the object and the sphere
; test (free outside geo_xform / geo_faces / project).

WA      = $C6A0
W_RT    = WA + 0
W_U     = WA + 18
W_V     = WA + 22
W_Y     = WA + 26
W_SCALE = WA + 28
W_GRIDS = WA + 30
W_OBJ   = WA + 184
W_CELLS = WA + 185
G_DIR    = 0          ; `geom::GridIn`, from walk_args
G_STRIDE = 3
G_NU     = 5
G_NV     = 6
G_D0     = 7
G_K      = 13
G_R      = 14
G_FAR    = 16
G_RN     = 18
G_SH     = 20
G_BASE   = 21         ; `geom::GridWalk`, derived by wsetup
G_EU     = 27
G_EV     = 33
G_S0     = 39
G_SU     = 51
G_SV     = 63
G_CAP    = 75         ; `geom::GridIn::cap` (walk::CAP): the objects' cull distance
GRID_LEN = 77
; object records (`geom::scene`)
O_DU     = 0
O_DV     = 1
O_DY     = 2
O_YAW    = 3
O_MESH   = 4
O_LANDMARK = 5          ; `geom::scene::O_LANDMARK`
SILH     = WA + 186     ; this object is drawn as its silhouette (`geom::walk::SILH`, inside walk::SIZE)
SBEST    = WA + 187     ; (4) wsil: the biggest triangle's area so far (`geom::walk::SBEST`)
SBI      = WA + 191     ; wsil: its vertex ($ff: none) (`geom::walk::SBI`; the last byte before RAY_STEP)
; RAM (XFORM_RAM, past geo_xform's MAB / MSG)
WROW     = $8F20        ; (12) the row's first cell: `Look::cell_sums`, 3 x i32
WCEL     = $8F2C        ; (12) this cell's
RAB      = $8F40        ; |rt|, 9 x u16, once a frame
RSG      = $8F52        ; rt's sign bytes, 9
WSHF     = $8F5B        ; the grid's record shift

DIR_RAM   = $8E40
CELL_RAM  = $C910
CELL_MAX  = 44
SCENE_RAM = $8000
OBJ_BANK  = 88
FARQ      = 5120        ; geom::FAR
HAZED     = $40         ; geom::HAZED: a triangle beyond the fog
HAZE_PAT  = 7           ; geom::HAZE_PAT
FAR_KEY   = 228         ; geom::FAR_KEY: the first depth bucket beyond the fog

GP     = $A0    ; (2) the grid's GridWalk
ROWN   = $A2    ; rows left
COLK   = $A3    ; cell in the row
DIRA   = $A4    ; (3) the row's directory entries (REU)
ROWC   = $A7    ; (6) the row's first cell centre (camera space)
CCV    = $AD    ; (6) this cell's centre
RECP   = $B3    ; (2) the object record
RECN   = $B5    ; records left in the cell
MP     = $B6    ; (2) the mesh record
POPD   = $B8    ; (2) a pop depth
LODV   = $BA

SX     = $C0    ; (2) sphere centre x, y, z (camera space, Q4); the object's origin
SY     = $C2
SZ     = $C4
SR     = $C6    ; (2) radius
SF     = $C8    ; (2) reach
RHALF  = $CA    ; (2)
ACC    = $CC    ; (4) 32-bit work
OT     = $D0    ; (3) orow's rotated offset, 24-bit
OJ     = $D3    ; orow's row
OE     = $D4    ; oprod's rt entry
YC     = $D6    ; (2) the rotation's first column entry
YS     = $D8    ; (2) its third
JR     = $DA    ; row offset (0, 6, 12)
JO     = $DB    ; output offset (0, 2, 4)
PDR    = $BB    ; the radius POPD is for ($FF: none yet)
SGNB   = $DC    ; acc_sub's sign byte (V's: free outside geo_xform)

.section .text.geo_walk,"ax",@progbits
.globl geo_walk
.type geo_walk,@function
geo_walk:
    ; |rt| and its signs, for orow's products
    ldx #0
1:  txa
    asl
    tay
    lda W_RT+1,y
    sta RSG,x
    bpl 2f
    sec
    lda #0
    sbc W_RT,y
    sta RAB,y
    lda #0
    sbc W_RT+1,y
    sta RAB+1,y
    jmp 3f
2:  lda W_RT,y
    sta RAB,y
    lda W_RT+1,y
    sta RAB+1,y
3:  inx
    cpx #9
    bne 1b
    lda #$ff
    sta PDR
    lda #0
    sta W_OBJ
    sta W_CELLS
    lda #<W_GRIDS
    sta GP
    lda #>W_GRIDS
    sta GP+1
    jsr wgrid
    lda #<(W_GRIDS + GRID_LEN)
    sta GP
    lda #>(W_GRIDS + GRID_LEN)
    sta GP+1
    jmp wgrid

; One grid (`geom::grid`).
wgrid:
    lda #$ff            ; (POPD's cap is the grid's: work it out afresh)
    sta PDR
    ldy #G_NV
    lda (GP),y
    bne 1f
    rts
1:  sta ROWN
    jsr wsetup
    ldy #0
2:  lda (GP),y
    sta DIRA,y
    iny
    cpy #3
    bne 2b
    ldy #G_BASE
    ldx #0
3:  lda (GP),y
    sta ROWC,x
    iny
    inx
    cpx #6
    bne 3b
    ldy #G_S0
    ldx #0
4:  lda (GP),y
    sta WROW,x
    iny
    inx
    cpx #12
    bne 4b
    ldy #G_SH
    lda (GP),y
    sta WSHF
wrow:
    ; this row's entries -> DIR_RAM (nu * 4 bytes)
    lda #<DIR_RAM
    sta $df02
    lda #>DIR_RAM
    sta $df03
    lda DIRA
    sta $df04
    lda DIRA+1
    sta $df05
    lda DIRA+2
    sta $df06
    ldy #G_NU
    lda (GP),y
    asl
    asl
    sta $df07
    lda #0
    sta $df08
    lda #$91
    sta $df01
    ldx #5
1:  lda ROWC,x
    sta CCV,x
    dex
    bpl 1b
    lda #0
    sta COLK
wcol:
    ; an empty cell: nothing to do (`geom::cell` returns at once)
    lda COLK
    asl
    asl
    tay
    lda DIR_RAM+2,y
    bne 1f
    jmp wnext
1:  ; skipped when z - rn - rmax * 16 > pop_depth(rmax), rmax the cell's largest radius (tested before
    ; the sphere: both must pass)
    lda CCV+4
    sta SZ
    lda CCV+5
    sta SZ+1
    lda DIR_RAM+3,y
    sta SR
    lda #0
    sta SR+1
    ldx #4
2:  asl SR
    rol SR+1
    dex
    bne 2b
    lda DIR_RAM+3,y
    jsr popdepth
    ldx #SZ
    jsr acc_ld
    ldx #SR
    jsr acc_sub
    ldx #POPD
    jsr acc_sub
    ldy #G_RN
    lda (GP),y
    sta SR
    iny
    lda (GP),y
    sta SR+1
    ldx #SR
    jsr acc_sub
    jsr accpos
    bcc 3f
    jmp wnext
3:  ; sphere (cell centre, r, far)
    ldx #5
4:  lda CCV,x
    sta SX,x
    dex
    bpl 4b
    ldy #G_R
    lda (GP),y
    sta SR
    iny
    lda (GP),y
    sta SR+1
    iny
    lda (GP),y
    sta SF
    iny
    lda (GP),y
    sta SF+1
    jsr sphere
    bcc wnext
    jsr wcsum
    lda COLK
    asl
    asl
    tay
    jsr wcell
wnext:
    ; next cell: += eu
    ldy #G_EU
    ldx #0
1:  clc
    lda CCV,x
    adc (GP),y
    sta CCV,x
    iny
    lda CCV+1,x
    adc (GP),y
    sta CCV+1,x
    iny
    inx
    inx
    cpx #6
    bne 1b
    inc COLK
    lda COLK
    ldy #G_NU
    cmp (GP),y
    bcs 2f
    jmp wcol
2:  ; next row: += ev, the entries += stride
    ldy #G_EV
    ldx #0
3:  clc
    lda ROWC,x
    adc (GP),y
    sta ROWC,x
    iny
    lda ROWC+1,x
    adc (GP),y
    sta ROWC+1,x
    iny
    inx
    inx
    cpx #6
    bne 3b
    ldy #G_SV
    ldx #0
5:  clc
    .rept 4
    lda WROW,x
    adc (GP),y
    sta WROW,x
    inx
    iny
    .endr
    cpx #12
    bne 5b
    ldy #G_STRIDE
    clc
    lda DIRA
    adc (GP),y
    sta DIRA
    iny
    lda DIRA+1
    adc (GP),y
    sta DIRA+1
    lda DIRA+2
    adc #0
    sta DIRA+2
    dec ROWN
    beq 4f
    jmp wrow
4:  rts

; The grid's derived fields (`geom::GridWalk::new`) into its block at GP: per row j of rt, s0 = rt . d0,
; base = (s0 + rt[j][1] << 7) >> 14, su = rt[j][0] << k, sv = rt[j][2] << k, eu / ev = those >> 14.
; (Its own section: resident, not relocated.)
.section .text.geo_wsetup,"ax",@progbits
wsetup:
    lda #0
    sta JR              ; rt row offset (0, 6, 12)
    sta JO              ; 4j
ws1:
    ldx JR
    ldy #G_D0
    jsr wdot
    jsr acc_ldp
    ldx JR
    inx
    inx
    ldy #G_D0 + 2
    jsr wdot
    jsr acc_addp
    ldx JR
    inx
    inx
    inx
    inx
    ldy #G_D0 + 4
    jsr wdot
    jsr acc_addp
    lda JO
    clc
    adc #G_S0
    tay
    ldx #0
1:  lda ACC,x
    sta (GP),y
    iny
    inx
    cpx #4
    bne 1b
    ; base
    ldx JR
    inx
    inx
    lda #7
    jsr wpx
    jsr acc_addp
    jsr acc_q14
    lda JO
    lsr
    clc
    adc #G_BASE
    tay
    lda ACC+2
    sta (GP),y
    iny
    lda ACC+3
    sta (GP),y
    ; su, eu
    ldx JR
    ldy #G_K
    lda (GP),y
    jsr wpx
    lda JO
    lsr
    clc
    adc #G_EU
    sta OE
    lda JO
    clc
    adc #G_SU
    tay
    jsr wput
    ; sv, ev
    ldx JR
    inx
    inx
    inx
    inx
    ldy #G_K
    lda (GP),y
    jsr wpx
    lda JO
    lsr
    clc
    adc #G_EV
    sta OE
    lda JO
    clc
    adc #G_SV
    tay
    jsr wput
    lda JR
    clc
    adc #6
    sta JR
    lda JO
    clc
    adc #4
    sta JO
    cmp #12
    beq 2f
    jmp ws1
2:  rts

; P = the rt entry at W_RT + X times the i16 at (GP),Y.
wdot:
    lda W_RT,x
    sta AL
    lda W_RT+1,x
    sta AH
    lda (GP),y
    sta BL
    iny
    lda (GP),y
    sta BH
    jmp smul

; P = the rt entry at W_RT + X, sign-extended, << A (A >= 1).
wpx:
    tay
    lda W_RT,x
    sta P0
    lda W_RT+1,x
    sta P1
    ora #$7f
    bmi 1f
    lda #0
1:  sta P2
    sta P3
2:  asl P0
    rol P1
    rol P2
    rol P3
    dey
    bne 2b
    rts

; P into the block at GP + Y (i32), and (P >> 14) as i16 at GP + OE.
wput:
    ldx #0
1:  lda P0,x
    sta (GP),y
    sta ACC,x
    iny
    inx
    cpx #4
    bne 1b
    jsr acc_q14
    ldy OE
    lda ACC+2
    sta (GP),y
    iny
    lda ACC+3
    sta (GP),y
    rts

; The streamed cell's exact sums (`GridWalk::s0 + row sv + col su`): WCEL = WROW + (COLK * rt[j][0]) << k.
.section .text.geo_wcsum,"ax",@progbits
wcsum:
    lda COLK
    bne 1f
    ldx #11
5:  lda WROW,x
    sta WCEL,x
    dex
    bpl 5b
    rts
1:  sta PLA
    sta PHA
    eor #$ff
    sta NLA
    sta NHA
    lda #0
    sta JR              ; 6j: rt[j][0] in RAB
    sta OJ              ; 3j: its sign in RSG
    sta JO              ; 4j
2:  ldx JR
    ldy RAB,x
    XPROD XP0, XP1
    ldy RAB+1,x
    XPROD XP2, XP3
    ; the product << 8, then << (k - 8)
    lda #0
    sta ACC
    lda XP0
    sta ACC+1
    clc
    lda XP1
    adc XP2
    sta ACC+2
    lda XP3
    adc #0
    sta ACC+3
    ldy #G_K
    lda (GP),y
    sec
    sbc #8
    tax
3:  asl ACC
    rol ACC+1
    rol ACC+2
    rol ACC+3
    dex
    bne 3b
    ldx OJ
    lda RSG,x
    bpl 4f
    jsr acc_neg
4:  ldx JO
    clc
    lda WROW,x
    adc ACC
    sta WCEL,x
    lda WROW+1,x
    adc ACC+1
    sta WCEL+1,x
    lda WROW+2,x
    adc ACC+2
    sta WCEL+2,x
    lda WROW+3,x
    adc ACC+3
    sta WCEL+3,x
    lda JR
    clc
    adc #6
    sta JR
    lda OJ
    clc
    adc #3
    sta OJ
    lda JO
    clc
    adc #4
    sta JO
    cmp #12
    beq 6f
    jmp 2b
6:  rts

.section .text.geo_walk,"ax",@progbits
; One cell (`geom::cell`), its directory entry at DIR_RAM + Y.
wcell:
    lda DIR_RAM+2,y
    bne 1f
    rts
1:  cmp #CELL_MAX + 1
    bcc 2f
    lda #CELL_MAX
2:  sta RECN
    ; records: OBJ_BANK:0000 + first * 8 -> CELL_RAM, count * 8 bytes
    lda #<CELL_RAM
    sta $df02
    lda #>CELL_RAM
    sta $df03
    lda DIR_RAM,y
    asl
    asl
    asl
    sta $df04
    lda DIR_RAM+1,y
    sta ACC
    lda DIR_RAM,y
    lsr
    lsr
    lsr
    lsr
    lsr
    sta ACC+1
    lda ACC
    asl
    asl
    asl
    ora ACC+1
    sta $df05
    lda ACC
    lsr
    lsr
    lsr
    lsr
    lsr
    clc
    adc #OBJ_BANK
    sta $df06
    lda RECN
    asl
    asl
    asl
    sta $df07
    lda #0
    rol
    sta $df08
    lda #$91
    sta $df01
    inc W_CELLS
    lda #<CELL_RAM
    sta RECP
    lda #>CELL_RAM
    sta RECP+1
3:  jsr wobject
    clc
    lda RECP
    adc #8
    sta RECP
    bcc 4f
    inc RECP+1
4:  dec RECN
    bne 3b
    rts

; One object (`geom::object`), its record at RECP.
wobject:
    ; the mesh record: SCENE_RAM + 8 + mesh * 10
    ldy #O_MESH
    lda (RECP),y
    jsr meshp
    ; r = rad << 4, far = pop_depth(rad)
    ldy #7
    lda (MP),y
    sta SR
    lda #0
    sta SR+1
    ldx #4
1:  asl SR
    rol SR+1
    dex
    bne 1b
    lda (MP),y
    jsr popdepth
    lda POPD
    sta SF
    lda POPD+1
    sta SF+1
    ; the centre (`geom::object_centre`): z first, and out early on the depth tests (the same ones
    ; `sphere` makes)
    ldx #2
    jsr orow
    ldx #SZ
    jsr acc_ld
    ldx #SR
    jsr acc_add
    sec
    lda ACC
    sbc #NEARZ
    lda ACC+1
    sbc #0
    lda ACC+2
    sbc #0
    lda ACC+3
    sbc #0
    bmi 9f
    ldx #SZ
    jsr acc_ld
    ldx #SR
    jsr acc_sub
    ldx #SF
    jsr acc_sub
    jsr accpos
    bcs 9f
    ldx #0
    jsr orow
    ldx #1
    jsr orow
    jsr sphere
    bcs 2f
9:  rts
2:  inc W_OBJ
    ; level of detail: lod != $FF and z > lod_z
    ldy #8
    lda (MP),y
    cmp #$ff
    beq 3f
    sta LODV
    iny
    lda (MP),y
    sta ACC
    lda #0
    sta ACC+1
    ldx #4
4:  asl ACC
    rol ACC+1
    dex
    bne 4b
    ; z > lod_z (signed z, lod_z in 0..4080)
    lda SZ+1
    bmi 3f
    lda ACC
    cmp SZ
    lda ACC+1
    sbc SZ+1
    bcs 3f              ; lod_z >= z
    lda LODV
    jsr meshp
3:  ; rotation rows (`geom::object`): yaw in quarter turns q (bits 7-6), columns (a, b) = rt[j][0], rt[j][2]:
    ; q 0: (a, b), 1: (-b, a), 2: (-a, -b), 3: (b, -a); the middle column is rt[j][1]
    ldy #O_YAW
    lda (RECP),y
    sta JO
    ldx #0
5:  stx JR
    bit JO
    bvs 6f
    lda W_RT,x
    sta YC
    lda W_RT+1,x
    sta YC+1
    lda W_RT+4,x
    sta YS
    lda W_RT+5,x
    sta YS+1
    jmp 7f
6:  lda W_RT+4,x            ; odd quarters: swapped
    sta YC
    lda W_RT+5,x
    sta YC+1
    lda W_RT,x
    sta YS
    lda W_RT+1,x
    sta YS+1
7:  lda JO                  ; the first negated for q 1, 2: bit 7 of yaw + 64
    clc
    adc #$40
    bpl 8f
    sec
    lda #0
    sbc YC
    sta YC
    lda #0
    sbc YC+1
    sta YC+1
8:  bit JO                  ; the second for q 2, 3
    bpl 9f
    sec
    lda #0
    sbc YS
    sta YS
    lda #0
    sbc YS+1
    sta YS+1
9:  ldx JR
    lda YC
    sta MROT,x
    lda YC+1
    sta MROT+1,x
    lda W_RT+2,x
    sta MROT+2,x
    lda W_RT+3,x
    sta MROT+3,x
    lda YS
    sta MROT+4,x
    lda YS+1
    sta MROT+5,x
    txa
    clc
    adc #6
    tax
    cpx #18
    beq 10f
    jmp 5b
10: ; the rest of GEO_ARGS: vertices, faces, shift, origin
    ldy #0
    clc
    lda (MP),y
    adc #<SCENE_RAM
    sta VPTR
    iny
    lda (MP),y
    adc #>SCENE_RAM
    sta VPTR+1
    iny
    lda (MP),y
    cmp #64 + 1
    bcc 6f
    lda #64
6:  sta NV
    iny
    clc
    lda (MP),y
    adc #<SCENE_RAM
    sta FPTR
    iny
    lda (MP),y
    adc #>SCENE_RAM
    sta FPTR+1
    iny
    lda (MP),y
    sta NF
    iny
    lda (MP),y
    clc
    adc #2
    sta TT
    eor #$ff
    sec
    adc #14
    sta SHR
    ldx #5
7:  lda SX,x
    sta ORGX,x
    dex
    bpl 7b
    jsr silflag         ; (a landmark beyond the fog: its silhouette, `wfaces`)
    ; far (`geom::AFFINE_SHIFT`: z > rad << 7, SR = rad << 4): weak perspective
    lda SZ+1
    bmi 8f
    lda SR
    sta ACC
    lda SR+1
    asl ACC
    rol
    asl ACC
    rol
    asl ACC
    rol
    sta ACC+1
    lda ACC
    cmp SZ
    lda ACC+1
    sbc SZ+1
    bcs 8f              ; rad << 7 >= z
    jsr geo_affine
    jmp wfaces
8:  jsr geo_xform
    jmp wfaces

; MP = SCENE_RAM + 8 + A * 10
meshp:
    sta MP
    lda #0
    sta MP+1
    asl MP
    rol MP+1            ; 2a
    lda MP
    sta ACC
    lda MP+1
    sta ACC+1
    asl MP
    rol MP+1
    asl MP
    rol MP+1            ; 8a
    clc
    lda MP
    adc ACC
    sta MP
    lda MP+1
    adc ACC+1
    sta MP+1            ; 10a
    clc
    lda MP
    adc #<(SCENE_RAM + 8)
    sta MP
    lda MP+1
    adc #>(SCENE_RAM + 8)
    sta MP+1
    rts

.section .text.geo_orow,"ax",@progbits
; Row X of the object's centre (`geom::object_centre`) into SX + 2X: (WCEL[X] + (rt row X . (du, dy, dv))
; << WSHF) >> 14. Each product is |d| (a byte) times |rt| by two quarter-square lookups (XPROD).
orow:
    stx OJ
    lda #0
    sta OT
    sta OT+1
    sta OT+2
    txa
    asl
    adc OJ
    sta OE              ; entry 3j: column u
    ldy #O_DU
    lda (RECP),y
    jsr oprod
    inc OE              ; up
    ldy #O_DY
    lda (RECP),y
    jsr oprod
    inc OE              ; v
    ldy #O_DV
    lda (RECP),y
    jsr oprod
    lda OT
    sta ACC
    lda OT+1
    sta ACC+1
    lda OT+2
    sta ACC+2
    ora #$7f
    bmi 1f
    lda #0
1:  sta ACC+3
    ldx WSHF
2:  asl ACC
    rol ACC+1
    rol ACC+2
    rol ACC+3
    dex
    bne 2b
    lda OJ
    asl
    asl
    tax
    clc
    lda ACC
    adc WCEL,x
    sta ACC
    lda ACC+1
    adc WCEL+1,x
    sta ACC+1
    lda ACC+2
    adc WCEL+2,x
    sta ACC+2
    lda ACC+3
    adc WCEL+3,x
    sta ACC+3
    jsr acc_q14
    lda OJ
    asl
    tax
    lda ACC+2
    sta SX,x
    lda ACC+3
    sta SX+1,x
    rts

; OT += A (i8, flags set by its load) * rt entry OE.
oprod:
    bne 1f
    rts
1:  sta CSGN
    bpl 2f
    eor #$ff
    clc
    adc #1
2:  sta PLA
    sta PHA
    eor #$ff
    sta NLA
    sta NHA
    ldx OE
    lda RSG,x
    eor CSGN
    sta CSGN
    txa
    asl
    tax
    ldy RAB,x
    XPROD XP0, XP1
    ldy RAB+1,x
    XPROD XP2, XP3
    clc
    lda XP1
    adc XP2
    sta XP1
    lda XP3
    adc #0
    sta XP2
    bit CSGN
    bmi 3f
    clc
    lda OT
    adc XP0
    sta OT
    lda OT+1
    adc XP1
    sta OT+1
    lda OT+2
    adc XP2
    sta OT+2
    rts
3:  sec
    lda OT
    sbc XP0
    sta OT
    lda OT+1
    sbc XP1
    sta OT+1
    lda OT+2
    sbc XP2
    sta OT+2
    rts

.section .text.geo_walk,"ax",@progbits
; POPD = min(A * scale, the grid's cap) (`geom::pop_depth`); the last radius's is kept (they repeat).
popdepth:
    cmp PDR
    bne 2f
    rts
2:  sta PDR
    sta AL
    lda #0
    sta AH
    lda W_SCALE
    sta BL
    lda W_SCALE+1
    sta BH
    jsr umul
    lda P2
    ora P3
    bne 1f
    ldy #G_CAP
    lda (GP),y
    cmp P0
    iny
    lda (GP),y
    sbc P1
    bcc 1f
    lda P0
    sta POPD
    lda P1
    sta POPD+1
    rts
1:  ldy #G_CAP
    lda (GP),y
    sta POPD
    iny
    lda (GP),y
    sta POPD+1
    rts

; Carry set when the sphere (SX, SY, SZ), radius SR, may be in view within SF (`geom::sphere_visible`).
sphere:
    ; z + r - NEAR >= 0
    ldx #SZ
    jsr acc_ld
    ldx #SR
    jsr acc_add
    sec
    lda ACC
    sbc #NEARZ
    lda ACC+1
    sbc #0
    lda ACC+2
    sbc #0
    lda ACC+3
    sbc #0
    bpl 9f
    clc
    rts
9:  ; z - r - far <= 0
    ldx #SZ
    jsr acc_ld
    ldx #SR
    jsr acc_sub
    ldx #SF
    jsr acc_sub
    jsr accpos
    bcc 10f
    clc
    rts
10: ; |x| - z - r - (r >> 1) <= 0
    lda SR+1
    cmp #$80
    ror
    sta RHALF+1
    lda SR
    ror
    sta RHALF
    ldx #SX
    jsr acc_ld
    lda ACC+3
    bpl 1f
    jsr acc_neg
1:  ldx #SZ
    jsr acc_sub
    ldx #SR
    jsr acc_sub
    ldx #RHALF
    jsr acc_sub
    jsr accpos
    bcs 8f
    ; |y| * 133 - z * 68 - r * 150 <= 0
    lda SY
    sta AL
    lda SY+1
    sta AH
    bpl 2f
    sec
    lda #0
    sbc AL
    sta AL
    lda #0
    sbc AH
    sta AH
2:  lda #133
    sta BL
    lda #0
    sta BH
    jsr umul
    jsr acc_ldp
    lda SZ
    sta AL
    lda SZ+1
    sta AH
    lda #CY16 / 16
    sta BL
    lda #0
    sta BH
    jsr smul
    jsr acc_subp
    lda SR
    sta AL
    lda SR+1
    sta AH
    lda #150
    sta BL
    lda #0
    sta BH
    jsr smul
    jsr acc_subp
    jsr accpos
    bcs 8f
    sec
    rts
8:  clc
    rts

; ACC = the i16 at zero page X, sign-extended.
acc_ld:
    lda 0,x
    sta ACC
    lda 1,x
    sta ACC+1
    ora #$7f
    bmi 1f
    lda #0
1:  sta ACC+2
    sta ACC+3
    rts

; ACC += / -= the i16 at zero page X.
acc_add:
    lda 1,x
    ora #$7f
    bmi 1f
    lda #0
1:  pha
    clc
    lda ACC
    adc 0,x
    sta ACC
    lda ACC+1
    adc 1,x
    sta ACC+1
    pla
    pha
    adc ACC+2
    sta ACC+2
    pla
    adc ACC+3
    sta ACC+3
    rts

acc_sub:
    lda 1,x
    ora #$7f
    bmi 1f
    lda #0
1:  sta SGNB
    sec
    lda ACC
    sbc 0,x
    sta ACC
    lda ACC+1
    sbc 1,x
    sta ACC+1
    lda ACC+2
    sbc SGNB
    sta ACC+2
    lda ACC+3
    sbc SGNB
    sta ACC+3
    rts

; ACC = P; ACC += P; ACC -= P (the multiply's 32-bit result).
acc_ldp:
    lda P0
    sta ACC
    lda P1
    sta ACC+1
    lda P2
    sta ACC+2
    lda P3
    sta ACC+3
    rts

acc_addp:
    clc
    lda ACC
    adc P0
    sta ACC
    lda ACC+1
    adc P1
    sta ACC+1
    lda ACC+2
    adc P2
    sta ACC+2
    lda ACC+3
    adc P3
    sta ACC+3
    rts

acc_subp:
    sec
    lda ACC
    sbc P0
    sta ACC
    lda ACC+1
    sbc P1
    sta ACC+1
    lda ACC+2
    sbc P2
    sta ACC+2
    lda ACC+3
    sbc P3
    sta ACC+3
    rts

; ACC <<= 2: ACC+2:ACC+3 is then (ACC >> 14) as i16.
acc_q14:
    asl ACC
    rol ACC+1
    rol ACC+2
    rol ACC+3
    asl ACC
    rol ACC+1
    rol ACC+2
    rol ACC+3
    rts

acc_neg:
    sec
    lda #0
    sbc ACC
    sta ACC
    lda #0
    sbc ACC+1
    sta ACC+1
    lda #0
    sbc ACC+2
    sta ACC+2
    lda #0
    sbc ACC+3
    sta ACC+3
    rts

; Carry set when ACC > 0 (signed).
accpos:
    lda ACC+3
    bmi 1f
    ora ACC+2
    ora ACC+1
    ora ACC
    beq 1f
    sec
    rts
1:  clc
    rts

.size geo_walk, .-geo_walk


; A distant landmark as one triangle: its leftmost and rightmost vertices on screen and the top one of
; the others (the first of equals), in the far shade (`geom::silhouette`). Resident, with `silflag` and `wfaces`: the walk's
; region is full (its tail is combat::FX).
.section .text.geo_sil,"ax",@progbits
; SILH: the object is a landmark wholly beyond the fog (z - r >= FAR; SZ, SR = rad << 4, before the
; transform overwrites them) (`geom::object`).
silflag:
    lda #0
    sta SILH
    ldy #O_LANDMARK
    lda (RECP),y
    beq 9f
    sec
    lda SZ
    sbc SR
    tax
    lda SZ+1
    sbc SR+1
    tay
    txa
    sec
    sbc #<FARQ
    tya
    sbc #>FARQ
    bvc 1f
    eor #$80
1:  bmi 9f
    inc SILH
9:  rts
; After the transform: the faces, or the silhouette.
wfaces:
    lda SILH
    bne wsil
    jmp geo_faces
wsil:
    lda #0
    sta IA
    sta IC
    ldx #1
1:  cpx NV
    bcs 8f
    ldy IA              ; SX[x] < SX[left]
    lda SXL,x
    cmp SXL,y
    lda SXH,x
    sbc SXH,y
    bvc 4f
    eor #$80
4:  bpl 5f
    stx IA
5:  ldy IC              ; SX[right] < SX[x]
    lda SXL,y
    cmp SXL,x
    lda SXH,y
    sbc SXH,x
    bvc 6f
    eor #$80
6:  bpl 7f
    stx IC
7:  inx
    jmp 1b
8:  ; of the others, the one making the biggest triangle with them (`geom::silhouette`); SBI $ff while
    ; none above the chord
    lda #$ff
    sta SBI
    lda #0
    sta SBEST
    sta SBEST+1
    sta SBEST+2
    sta SBEST+3
    ldx #0
1:  cpx NV
    bcs 9f
    cpx IA
    beq 3f
    cpx IC
    beq 3f
    stx IB
    jsr farea           ; area - best > 0: the new best
    sta V+3
    sec
    lda SBEST
    sbc V
    lda SBEST+1
    sbc V+1
    lda SBEST+2
    sbc V+2
    lda SBEST+3
    sbc V+3
    bvc 4f
    eor #$80
4:  bpl 2f
    lda IB
    sta SBI
    ldx #3
5:  lda V,x
    sta SBEST,x
    dex
    bpl 5b
2:  ldx IB              ; (farea took X)
3:  inx
    jmp 1b
9:  lda SBI
    bmi 10f
    sta IB
    lda #HAZED | HAZE_PAT
    sta SHD
    jmp tryface
10: rts
