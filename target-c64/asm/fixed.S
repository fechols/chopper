; MODE 7 fixed point: a 16x16 -> 32 unsigned multiply by quarter squares, exactly a * b. Reference:
; `fixed::umul` (the host multiplies natively); `tools/test_fixed.py` checks it.
;
; Per 8x8 product: a*b = f(a + b) - g(b - a + 255), f(n) = n*n/4 (SQ, n = 0..511) and g(n) = f(|n - 255|)
; (SQN, n = 0..510), tables `fixed::init_tables` builds at boot. With pointers at SQ + a and SQN + 255 - a,
; both lookups are (pointer),y with y = b: the 6502 does the index add. Partial products of zero high
; bytes are skipped (most operands here are small).
;
;   m7_umul16(a: u16, b: u16) -> u32   llvm-mos C ABI: a in A / X, b in __rc2 / __rc3, result A, X, __rc2, __rc3
;   umul                               asm callers: AL:AH * BL:BH -> P0..P3 (inputs kept)
;
; Zero page $80-$9B (the compiler's ends at $7F, link.ld).

SQLO  = $0400
SQHI  = $0600
SQNLO = $5000
SQNHI = $5200

PLO = $80       ; (2) SQLO + a
PHI = $82       ; (2) SQHI + a
NLO = $84       ; (2) SQNLO + 255 - a
NHI = $86       ; (2) SQNHI + 255 - a

AL = $90
AH = $91
BL = $92
BH = $93
P0 = $94
P1 = $95
P2 = $96
P3 = $97
T0 = $98
T1 = $99
U0 = $9A
U1 = $9B

; Point the tables at multiplier byte A.
.macro SETA
    sta PLO
    sta PHI
    eor #$ff
    sta NLO
    sta NHI
.endm

; \rlo:\rhi = a * (Y)
.macro PROD rlo, rhi
    lda (PLO),y
    sec
    sbc (NLO),y
    sta \rlo
    lda (PHI),y
    sbc (NHI),y
    sta \rhi
.endm

.section .text.m7_umul16,"ax",@progbits
.globl m7_umul16
.type m7_umul16,@function
m7_umul16:
    sta AL
    stx AH
    lda __rc2
    sta BL
    lda __rc3
    sta BH
    jsr umul
    lda P2
    sta __rc2
    lda P3
    sta __rc3
    lda P0
    ldx P1
    rts
.size m7_umul16, .-m7_umul16

; m7_smul16(a: i16, b: i16) -> i32: the magnitudes multiplied (so small negatives keep umul's zero-byte
; skips), negated when the signs differ. Reference: `fixed::smul`.
.section .text.m7_smul16,"ax",@progbits
.globl m7_smul16
.type m7_smul16,@function
m7_smul16:
    sta AL
    stx AH
    txa
    eor __rc3
    sta U0              ; bit 7: the signs differ
    lda __rc2
    sta BL
    lda __rc3
    sta BH
    bpl 1f
    lda #0
    sec
    sbc BL
    sta BL
    lda #0
    sbc BH
    sta BH
1:  lda AH
    bpl 2f
    lda #0
    sec
    sbc AL
    sta AL
    lda #0
    sbc AH
    sta AH
2:  jsr umul
    bit U0
    bpl 3f
    lda #0
    sec
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
3:  lda P2
    sta __rc2
    lda P3
    sta __rc3
    lda P0
    ldx P1
    rts
.size m7_smul16, .-m7_smul16

; Point the table pointers' high bytes at the tables (once, at boot; nothing else writes $80-$87).
.section .text.m7_umul_init,"ax",@progbits
.globl m7_umul_init
.type m7_umul_init,@function
m7_umul_init:
    lda #>SQLO
    sta PLO+1
    lda #>SQHI
    sta PHI+1
    lda #>SQNLO
    sta NLO+1
    lda #>SQNHI
    sta NHI+1
    rts
.size m7_umul_init, .-m7_umul_init

.section .text.umul,"ax",@progbits
.globl umul
.type umul,@function
umul:
    lda #0
    sta P2
    sta P3
    ; AL * BL -> P0:P1
    lda AL
    SETA
    ldy BL
    PROD P0, P1
    ; AL * BH -> + P1:P2
    ldy BH
    beq 1f
    PROD T0, T1
    clc
    lda P1
    adc T0
    sta P1
    lda T1
    adc #0
    sta P2
1:  lda AH
    bne 2f
    rts
2:  SETA
    ; AH * BL -> + P1:P2:P3
    ldy BL
    PROD T0, T1
    clc
    lda P1
    adc T0
    sta P1
    lda P2
    adc T1
    sta P2
    lda #0
    adc #0
    sta P3
    ; AH * BH -> + P2:P3
    ldy BH
    beq 3f
    PROD T0, T1
    clc
    lda P2
    adc T0
    sta P2
    lda P3
    adc T1
    sta P3
3:  rts
.size umul, .-umul
