; MODE 7 geometry kernels. References in src/geom.rs (`xform`, `faces`, `raster` + `setup`); the host
; runs those, the C64 runs these, and `tools/emu.py` checks whole frames byte for byte.
;
;   geo_xform():  one object's vertices -> camera space and screen (VERTS arrays)
;   geo_faces():  one object's faces -> backface cull, depth key, push into the buckets (WORK);
;                 faces straddling the near plane are listed in WORK.clip for Rust
;   geo_bin() / geo_activate(band) / geo_draw(band): the buckets in painter's order, band by band
;                 (per-triangle set-up when its first band comes, then tri_row per band)
; Arguments in GEO_ARGS ($C7C0, `geom::geo_args`); shade patterns at PATS ($C7E0, even[16], odd[16]).
;
; Zero page: $C0-$EF and $F0-$FF (floor.S's, free while the floor is not drawing); fixed.S has $90-$9B,
; poly.S / tri.S $A0-$B3.

GA     = $C7C0
VPTR   = GA + 0
NV     = GA + 2
FPTR   = GA + 3
NF     = GA + 5
SHR    = GA + 6
TT     = GA + 7
X0     = GA + 8
Z0     = GA + 10
CC     = GA + 12
SS     = GA + 14
BMP    = GA + 16
TRIS   = GA + 18
TOP    = GA + 19
PATE   = $C7E0
PATO   = $C7F0

VERTS  = $9800
VXL    = VERTS + $000
VXH    = VERTS + $080
VYL    = VERTS + $100
VYH    = VERTS + $180
VZL    = VERTS + $200
VZH    = VERTS + $280
SXL    = VERTS + $300
SXH    = VERTS + $380
SYL    = VERTS + $400
SYH    = VERTS + $480

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

; V = P >> 8 (arithmetic)
qp:
    lda P1
    sta V
    lda P2
    sta V+1
    lda P3
    sta V+2
    cmp #$80
    lda #0
    adc #$ff
    eor #$ff
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

copyq:
    lda V
    sta Q
    lda V+1
    sta Q+1
    lda V+2
    sta Q+2
    lda V+3
    sta Q+3
    rts

addq:
    clc
    lda V
    adc Q
    sta V
    lda V+1
    adc Q+1
    sta V+1
    lda V+2
    adc Q+2
    sta V+2
    lda V+3
    adc Q+3
    sta V+3
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

; (PX, PY, PZ) -> S1:S1+1 = sx, S1+2:S1+3 = sy (`geom::project`; PZ >= NEAR).
project:
    lda PZ
    sta M
    lda PZ+1
    sta M+1
    jsr norm
    lda E
    clc
    adc #10
    sta SHF
    ; sx = 1280 + ((x * r >> 8) * 5 >> s)
    lda PX
    sta AL
    lda PX+1
    sta AH
    jsr setr
    jsr smulsu
    jsr qp
    jsr copyq
    jsr shlv
    jsr shlv
    jsr addq
    lda SHF
    jsr sarv
    clc
    lda V
    adc #$00            ; 1280 = $0500
    sta V
    lda V+1
    adc #$05
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
    ; sy = 768 + (((192 - y) * r >> 8) * 10 >> s)
    sec
    lda #192
    sbc PY
    sta AL
    lda #0
    sbc PY+1
    sta AH
    jsr setr
    jsr smulsu
    jsr qp
    jsr shlv
    jsr copyq
    jsr shlv
    jsr shlv
    jsr addq
    lda SHF
    jsr sarv
    clc
    lda V
    adc #$00            ; 768 = $0300
    sta V
    lda V+1
    adc #$03
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
; geo_xform: x = x0 + (a*cc + b*ss) >> sh, z = z0 + (a*ss - b*cc) >> sh, y = h << t; project if z >= 128.

.section .text.geo_xform,"ax",@progbits
.globl geo_xform
.type geo_xform,@function
geo_xform:
    lda VPTR
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
1:  ldy #0
    lda (VP),y
    jsr sxa
    lda AL
    sta VA
    lda AH
    sta VA+1
    iny
    lda (VP),y
    jsr sxa
    lda AL
    sta VB
    lda AH
    sta VB+1
    iny
    lda (VP),y
    jsr sxa
    lda AL
    sta PY
    lda AH
    sta PY+1
    ldx TT
2:  asl PY
    rol PY+1
    dex
    bne 2b
    ; S1 = a*cc + b*ss
    lda VA
    sta AL
    lda VA+1
    sta AH
    lda CC
    sta BL
    lda CC+1
    sta BH
    jsr smul
    lda P0
    sta S1
    lda P1
    sta S1+1
    lda P2
    sta S1+2
    lda P3
    sta S1+3
    lda VB
    sta AL
    lda VB+1
    sta AH
    lda SS
    sta BL
    lda SS+1
    sta BH
    jsr smul
    clc
    lda S1
    adc P0
    sta S1
    lda S1+1
    adc P1
    sta S1+1
    lda S1+2
    adc P2
    sta S1+2
    lda S1+3
    adc P3
    sta S1+3
    ; S2 = a*ss - b*cc
    lda VA
    sta AL
    lda VA+1
    sta AH
    lda SS
    sta BL
    lda SS+1
    sta BH
    jsr smul
    lda P0
    sta S2
    lda P1
    sta S2+1
    lda P2
    sta S2+2
    lda P3
    sta S2+3
    lda VB
    sta AL
    lda VB+1
    sta AH
    lda CC
    sta BL
    lda CC+1
    sta BH
    jsr smul
    sec
    lda S2
    sbc P0
    sta S2
    lda S2+1
    sbc P1
    sta S2+1
    lda S2+2
    sbc P2
    sta S2+2
    lda S2+3
    sbc P3
    sta S2+3
    ; x = x0 + (S1 >> sh)
    ldx #3
3:  lda S1,x
    sta V,x
    dex
    bpl 3b
    lda SHR
    jsr sarv
    clc
    lda V
    adc X0
    sta PX
    lda V+1
    adc X0+1
    sta PX+1
    ; z = z0 + (S2 >> sh)
    ldx #3
4:  lda S2,x
    sta V,x
    dex
    bpl 4b
    lda SHR
    jsr sarv
    clc
    lda V
    adc Z0
    sta PZ
    lda V+1
    adc Z0+1
    sta PZ+1
    ldx I
    lda PX
    sta VXL,x
    lda PX+1
    sta VXH,x
    lda PY
    sta VYL,x
    lda PY+1
    sta VYH,x
    lda PZ
    sta VZL,x
    lda PZ+1
    sta VZH,x
    ; project when z >= 128
    lda PZ+1
    bmi 6f
    bne 5f
    lda PZ
    cmp #128
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
.size geo_xform, .-geo_xform

; ---------------------------------------------------------------------------------------------------
; geo_faces: per face: all corners in front -> area cull, key, push; some in front -> CLIP list.

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
    rts
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
    cmp #128
    bcc 2f
1:  inc CNT
2:  rts

tryface:
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
    bmi 9f
    ora V+2
    ora V+1
    ora V
    beq 9f
    jsr facekey
    sta KEY
    lda WN
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

; A = clamp((za + zb + zc) >> 5, 0, 255)
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
    cmp #$20
    bcs 2f
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
2:  lda #255
    rts
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
    ; end = min(line_at(y2), 200); none if negative
    LINEAT 2
    lda LA+1
    bpl 3f
    clc
    rts
3:  bne 4f
    lda LA
    cmp #201
    bcc 5f
4:  lda #200
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

; TRI_ARGS + X <- EX (x, 4 bytes), ES (step, 4 bytes)
putedge:
    ldy #0
1:  lda EX,y
    sta TRI_ARGS,x
    inx
    iny
    cpy #4
    bne 1b
    ldy #0
2:  lda ES,y
    sta TRI_ARGS,x
    inx
    iny
    cpy #4
    bne 2b
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
    ldx #3
1:  lda V,x
    sta PER,x
    sta ES,x
    dex
    bpl 1b
    ; step = per << 4
    ldx #4
2:  asl ES
    rol ES+1
    rol ES+2
    rol ES+3
    dex
    bne 2b
    ; d = line * 16 + 8 - ya
    lda LINE
    sta DD
    lda #0
    sta DD+1
    ldx #4
3:  asl DD
    rol DD+1
    dex
    bne 3b
    clc
    lda DD
    adc #8
    sta DD
    lda DD+1
    adc #0
    sta DD+1
    sec
    lda DD
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
    ; + xa << 12
    lda #0
    sta V
    lda XAV
    sta V+1
    lda XAV+1
    sta V+2
    cmp #$80
    lda #0
    adc #$ff
    eor #$ff
    sta V+3
    ldx #4
4:  jsr shlv
    dex
    bne 4b
    clc
    lda EX
    adc V
    sta EX
    lda EX+1
    adc V+1
    sta EX+1
    lda EX+2
    adc V+2
    sta EX+2
    lda EX+3
    adc V+3
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
    ldx #24
1:  sta FHEAD,x
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
    lda #25
    sta TOP
    lda #255
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
    ; the first band: line_at(min y), < 200
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
    cmp #200
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
9:  inc NSEQ
    ldx IDX
    lda NEXT,x
    jmp btri
bnext:
    lda KEY
    beq 12f
    dec KEY
    jmp bkey
12: rts

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
    jsr tsetup
    bcs 13f
    jmp 3f              ; nothing to draw (long branches as bxx/jmp)
13: lda NFREE
    bne 2f
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
    ; keep it while it ends below this band
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
