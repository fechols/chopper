; MODE 7 polygon cell-row writer: "just write bytes". Reference: `poly::fill_row` (src/poly.rs);
; `tools/test_poly.py` checks this file against `poly::paint_row`'s specification byte for byte.
;
; poly_row(): one cell row (8 lines) of a convex polygon from POLY_ARGS ($C780, `poly::args`):
;   XL[8], XR[8]  line k covers pixels XL[k]..XR[k] (empty when XL >= XR)
;   PE, PO        pattern bytes for even / odd lines
;   BASE          bitmap address of the cell row
; Cells that every line covers completely get 8 plain stores (a cell's 8 lines are 8 consecutive bytes);
; only the ends of each line are read-modify-written: dst ^ ((dst ^ pat) & mask).
;
; Zero page $A0-$AF (floor.S's, free while the floor is not drawing).

ARGS = $C780
XL   = ARGS
XR   = ARGS + 8
PE   = ARGS + 16
PO   = ARGS + 17
BASE = ARGS + 18

P    = $A0      ; byte pointer (2)
LMAX = $A2
RMIN = $A3
C0   = $A4      ; full cells C0..C1
C1   = $A5
FULL = $A6
K    = $A7      ; line in the cell row
PA   = $A8      ; part: pixels PA..PB
PB   = $A9
PAT  = $AA
M    = $AB
N    = $AC
CB   = $AD

.section .text.poly_row,"ax",@progbits
.globl poly_row
.type poly_row,@function
poly_row:
    ; Full cells exist only if every line is non-empty: ceil(max XL / 4) .. floor(min XR / 4).
    lda #0
    sta FULL
    sta LMAX
    lda #$ff
    sta RMIN
    ldx #7
1:  lda XL,x
    cmp XR,x
    bcs 3f
    cmp LMAX
    bcc 2f
    sta LMAX
2:  lda XR,x
    cmp RMIN
    bcs 4f
    sta RMIN
4:  dex
    bpl 1b
    lda LMAX
    clc
    adc #3
    lsr
    lsr
    sta C0
    lda RMIN
    lsr
    lsr
    sta C1
    cmp C0
    beq 3f
    bcc 3f
    inc FULL
3:
    ldx #0
line:
    stx K
    lda XL,x
    cmp XR,x
    bcs next
    txa
    and #1
    tay
    lda PE,y
    sta PAT
    lda FULL
    beq whole
    lda XL,x
    sta PA
    lda C0
    asl
    asl
    sta PB
    jsr part
    ldx K
    lda C1
    asl
    asl
    sta PA
    lda XR,x
    sta PB
    jsr part
    jmp next
whole:
    lda XL,x
    sta PA
    lda XR,x
    sta PB
    jsr part
next:
    ldx K
    inx
    cpx #8
    beq 5f
    jmp line
5:
    lda FULL
    beq done
    ldx C0
    lda c8lo,x
    clc
    adc BASE
    sta P
    lda c8hi,x
    adc BASE+1
    sta P+1
    lda C1
    sec
    sbc C0
    sta N
    ldy #0
cell:
    .rept 4
    lda PE
    sta (P),y
    iny
    lda PO
    sta (P),y
    iny
    .endr
    bne 6f
    inc P+1
6:  dec N
    bne cell
done:
    rts

; Pixels PA..PB of line K in PAT.
part:
    lda PA
    cmp PB
    bcs pret
    lda PB
    sec
    sbc #1
    lsr
    lsr
    sta CB
    lda PA
    lsr
    lsr
    tax                 ; first byte column
    lda c8lo,x
    ora K
    clc
    adc BASE
    sta P
    lda c8hi,x
    adc BASE+1
    sta P+1
    lda CB
    asl
    asl
    sta M
    lda PB
    sec
    sbc M
    tay
    lda rmask,y
    sta M               ; right end mask
    lda PA
    and #3
    tay
    lda lmask,y         ; left end mask
    ldy #0
    cpx CB
    bne multi
    and M
    sta M
    lda (P),y
    eor PAT
    and M
    eor (P),y
    sta (P),y
pret:
    rts
multi:
    sta N
    lda (P),y
    eor PAT
    and N
    eor (P),y
    sta (P),y
    stx N
    lda CB
    clc
    sbc N               ; CB - first - 1 whole bytes
    tax
    ldy #8
    cpx #0
    beq last
mid:
    lda PAT
    sta (P),y
    tya
    clc
    adc #8
    tay
    bcc 7f
    inc P+1
7:  dex
    bne mid
last:
    lda (P),y
    eor PAT
    and M
    eor (P),y
    sta (P),y
    rts
.size poly_row, .-poly_row

.section .rodata.poly_tables,"a",@progbits
lmask: .byte $ff, $3f, $0f, $03
rmask: .byte $00, $c0, $f0, $fc, $ff
c8lo:
    .irp c, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39
    .byte (\c * 8) & 255
    .endr
c8hi:
    .irp c, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39
    .byte (\c * 8) >> 8
    .endr
