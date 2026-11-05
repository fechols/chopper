; MODE 7 triangle line walk. Reference: `geom::walk` (src/geom.rs); `tools/emu.py` checks whole frames
; against the host.
;
; tri_row(A = row): one cell row of a set-up triangle from TRI_ARGS ($C7A0, `geom::walk_args`): long, short and lower edges
; (x + $7FFF, step: i32 Q16 pixels per line), first / mid / end lines, bitmap. The pattern is already in
; POLY_ARGS. Per line: both edges' pixels (centre at or right of the bound, clamped 0..160), ordered
; into XL / XR; per cell row: poly_row.
;
; Zero page $B0-$B7 (floor.S's, free while the floor is not drawing; poly.S has $A0-$AF).

TA     = $C7A0
LX     = TA
LS     = TA + 4
SX     = TA + 8
SS     = TA + 12
WX     = TA + 16
FIRST  = TA + 24
MID    = TA + 25
END    = TA + 26
BMP    = TA + 27

XL     = $C780
XR     = $C788
BASE   = $C792

LINE   = $B0    ; line of the walk
ROW    = $B1
K      = $B2
LP     = $B3

; A = pixel of the Q16 bound at \src: (x + $7FFF) >> 16, clamped to 0..XMAX (`geom::pixel`: the drawn
; columns, `layout::DRAWN_COLS`). The edges are stored with the $7FFF already added (geom.S putedge), so
; that is bytes 2-3.
XMAX = 156
.macro PIXEL src
    lda \src+3
    bne 1f
    lda \src+2
    cmp #XMAX + 1
    bcc 3f
    lda #XMAX
    jmp 3f
1:  bmi 2f
    lda #XMAX
    jmp 3f
2:  lda #0
3:
.endm

.macro ADD32 dst, src
    clc
    lda \dst
    adc \src
    sta \dst
    lda \dst+1
    adc \src+1
    sta \dst+1
    lda \dst+2
    adc \src+2
    sta \dst+2
    lda \dst+3
    adc \src+3
    sta \dst+3
.endm

; tri_row(A = cell row): that row's lines of the set-up triangle in TRI_ARGS (the edges advance), into
; BMP + row * 320 (`geom::walk_row`).
.section .text.tri_row,"ax",@progbits
.globl tri_row
.type tri_row,@function
tri_row:
    sta ROW
    asl
    asl
    asl
    sta LINE
    ldx ROW
    lda row320lo,x
    clc
    adc BMP
    sta BASE
    lda row320hi,x
    adc BMP+1
    sta BASE+1
    ldx #0
kloop:
    stx K
    ; (Long branches as bxx/jmp: llvm-mc mis-encodes out-of-range branches silently.)
    lda LINE
    cmp FIRST
    bcs 4f
    jmp empty
4:  cmp END
    bcc 5f
    jmp empty
5:  cmp MID
    bne edges
    ldy #7
copy:
    lda WX,y
    sta SX,y
    dey
    bpl copy
edges:
    PIXEL LX
    sta LP
    PIXEL SX
    ldx K
    cmp LP
    bcs ordered
    sta XL,x
    lda LP
    sta XR,x
    jmp step
ordered:
    sta XR,x
    lda LP
    sta XL,x
step:
    ADD32 LX, LS
    ADD32 SX, SS
    jmp knext
empty:
    lda #0
    ldx K
    sta XL,x
    sta XR,x
knext:
    inc LINE
    ldx K
    inx
    cpx #8
    beq rowdone
    jmp kloop
rowdone:
    jmp poly_row
.size tri_row, .-tri_row

.section .rodata.tri_tables,"a",@progbits
.globl row320lo
.globl row320hi
row320lo:
    .irp r, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24
    .byte (\r * 320) & 255
    .endr
row320hi:
    .irp r, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24
    .byte (\r * 320) >> 8
    .endr
