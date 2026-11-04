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

; m7_div_clamp(n: i32, z: u16) -> i16: (n / z).clamp(-1000, 1000), truncated toward zero, for
; 0 < z < 32768 (`hud::div_clamp`; its callers keep z >= NEAR). |n| >= z << 10 clamps at once; else the
; quotient has ten bits: R = |n| >> 10 (< z), then |n|'s low ten bits shifted in one by one, restoring.
DQ_B0 = __rc6           ; |n| << 8 as five bytes B4..B0, then >> 2: R = B3:B2, the low bits in B1:B0
DQ_B1 = __rc7
DQ_B2 = __rc8
DQ_B3 = __rc9
DQ_B4 = __rc10
DQ_QL = __rc11
DQ_QH = __rc12
DQ_SG = __rc13          ; bit 7: n < 0
DQ_T0 = __rc14          ; z << 2 (17 bits)
DQ_T1 = __rc15
DQ_T2 = __rc16
.section .text.m7_div_clamp,"ax",@progbits
.globl m7_div_clamp
.type m7_div_clamp,@function
m7_div_clamp:
    sta DQ_B1
    stx DQ_B2
    lda __rc2
    sta DQ_B3
    lda __rc3
    sta DQ_B4
    sta DQ_SG
    bpl 1f
    sec
    lda #0
    sbc DQ_B1
    sta DQ_B1
    lda #0
    sbc DQ_B2
    sta DQ_B2
    lda #0
    sbc DQ_B3
    sta DQ_B3
    lda #0
    sbc DQ_B4
    sta DQ_B4
1:  lda __rc4
    asl
    sta DQ_T0
    lda __rc5
    rol
    sta DQ_T1
    lda #0
    rol
    sta DQ_T2
    asl DQ_T0
    rol DQ_T1
    rol DQ_T2
    ; |n| >= z << 10 when its top three bytes reach z << 2 (its low byte against 0).
    lda DQ_B2
    cmp DQ_T0
    lda DQ_B3
    sbc DQ_T1
    lda DQ_B4
    sbc DQ_T2
    bcs dq_clamp
    lda #0
    sta DQ_B0
    ldy #2
2:  lsr DQ_B4
    ror DQ_B3
    ror DQ_B2
    ror DQ_B1
    ror DQ_B0
    dey
    bne 2b
    lda #0
    sta DQ_QL
    sta DQ_QH
    ldy #10
3:  asl DQ_B0
    rol DQ_B1
    rol DQ_B2
    rol DQ_B3
    sec
    lda DQ_B2
    sbc __rc4
    tax
    lda DQ_B3
    sbc __rc5
    bcc 4f
    sta DQ_B3
    stx DQ_B2
4:  rol DQ_QL
    rol DQ_QH
    dey
    bne 3b
    lda DQ_QL
    cmp #<1000
    lda DQ_QH
    sbc #>1000
    bcc 5f
dq_clamp:
    lda #<1000
    sta DQ_QL
    lda #>1000
    sta DQ_QH
5:  bit DQ_SG
    bpl 6f
    sec
    lda #0
    sbc DQ_QL
    tay
    lda #0
    sbc DQ_QH
    tax
    tya
    rts
6:  lda DQ_QL
    ldx DQ_QH
    rts
.size m7_div_clamp, .-m7_div_clamp

; The i32 and u32 divide libcalls (`--wrap=__divsi3`, `--wrap=__udivsi3`, target-c64/build.rs), in place of
; libcrt's (~3400 cycles; 380 bytes for its __udivsi3): n / d truncated toward zero (Rust has ruled out
; d = 0 and MIN / -1). One unsigned core, `ud32`:
; - d < 256 (`floor::Rays::new`'s divisors): the dividend's 32 bits shifted through a 9-bit remainder in A,
;   each quotient bit taking the place of the dividend bit shifted out (~1100-1500 cycles);
; - wider (`fixed::init_tables`' reciprocals, at boot): the same with a 32-bit remainder and its 33rd bit.
DV_N0 = __rc8           ; the dividend, then the quotient
DV_N1 = __rc9
DV_N2 = __rc10
DV_N3 = __rc11
DV_R0 = __rc12          ; the wide remainder
DV_R1 = __rc13
DV_R2 = __rc14
DV_R3 = __rc15
DV_T1 = __rc16          ; a trial difference's middle bytes, the remainder's 33rd bit
DV_T2 = __rc17
DV_T3 = __rc18
DV_SG = __rc19          ; bit 7: the quotient is negative (__divsi3)

.section .text.__wrap___divsi3,"ax",@progbits
.globl __wrap___udivsi3
.type __wrap___udivsi3,@function
__wrap___udivsi3:
    jsr dv_load
    jsr ud32
    jmp dv_ret
.size __wrap___udivsi3, .-__wrap___udivsi3

.globl __wrap___divsi3
.type __wrap___divsi3,@function
__wrap___divsi3:
    jsr dv_load
    lda DV_N3
    eor __rc7
    sta DV_SG
    lda DV_N3
    bpl 1f
    jsr dv_neg
1:  lda __rc7
    bpl 2f
    sec
    lda #0
    sbc __rc4
    sta __rc4
    lda #0
    sbc __rc5
    sta __rc5
    lda #0
    sbc __rc6
    sta __rc6
    lda #0
    sbc __rc7
    sta __rc7
2:  jsr ud32
    bit DV_SG
    bpl dv_ret
    jsr dv_neg
dv_ret:
    lda DV_N2
    sta __rc2
    lda DV_N3
    sta __rc3
    lda DV_N0
    ldx DV_N1
    rts
dv_load:
    sta DV_N0
    stx DV_N1
    lda __rc2
    sta DV_N2
    lda __rc3
    sta DV_N3
    rts
dv_neg:
    sec
    lda #0
    sbc DV_N0
    sta DV_N0
    lda #0
    sbc DV_N1
    sta DV_N1
    lda #0
    sbc DV_N2
    sta DV_N2
    lda #0
    sbc DV_N3
    sta DV_N3
    rts

; N /= d (__rc4-__rc7), unsigned.
ud32:
    lda __rc5
    ora __rc6
    ora __rc7
    bne ud_wide
    ldx #32
    lda #0
3:  asl DV_N0
    rol DV_N1
    rol DV_N2
    rol DV_N3
    rol a
    bcs 4f
    cmp __rc4
    bcc 5f
4:  sbc __rc4
    inc DV_N0
5:  dex
    bne 3b
    rts
ud_wide:
    lda #0
    sta DV_R0
    sta DV_R1
    sta DV_R2
    sta DV_R3
    ldx #32
6:  asl DV_N0
    rol DV_N1
    rol DV_N2
    rol DV_N3
    rol DV_R0
    rol DV_R1
    rol DV_R2
    rol DV_R3
    lda #0
    rol a
    sta DV_T3
    ; R (33 bits) - d: kept when it does not borrow
    sec
    lda DV_R0
    sbc __rc4
    tay
    lda DV_R1
    sbc __rc5
    sta DV_T1
    lda DV_R2
    sbc __rc6
    sta DV_T2
    lda DV_R3
    sbc __rc7
    pha
    lda DV_T3
    sbc #0
    pla
    bcc 7f
    sta DV_R3
    lda DV_T2
    sta DV_R2
    lda DV_T1
    sta DV_R1
    sty DV_R0
    inc DV_N0
7:  dex
    bne 6b
    rts
.size __wrap___divsi3, .-__wrap___divsi3



; __wrap___udivhi3(n: u16, d: u16) -> u16: the u16 divide libcall (`--wrap=__udivhi3`, target-c64/build.rs;
; libcrt's is 196 bytes): 16 steps, the remainder with its 17th bit (d may exceed 32767). d > 0 (Rust
; checks).
DH_N0 = __rc4           ; the dividend, then the quotient
DH_N1 = __rc5
DH_R0 = __rc6           ; the remainder
DH_R1 = __rc7
DH_R2 = __rc8           ; its 17th bit
.section .text.__wrap___udivhi3,"ax",@progbits
.globl __wrap___udivhi3
.type __wrap___udivhi3,@function
__wrap___udivhi3:
    sta DH_N0
    stx DH_N1
    lda #0
    sta DH_R0
    sta DH_R1
    ldx #16
1:  asl DH_N0
    rol DH_N1
    rol DH_R0
    rol DH_R1
    lda #0
    rol a
    sta DH_R2
    sec
    lda DH_R0
    sbc __rc2
    tay
    lda DH_R1
    sbc __rc3
    pha
    lda DH_R2
    sbc #0
    pla
    bcc 2f
    sta DH_R1
    sty DH_R0
    inc DH_N0
2:  dex
    bne 1b
    lda DH_N0
    ldx DH_N1
    rts
.size __wrap___udivhi3, .-__wrap___udivhi3

; __wrap___mulsi3(a: i32, b: i32) -> i32: the i32 multiply libcall (`--wrap=__mulsi3`, target-c64/build.rs),
; in place of libcrt's (~1440 cycles, 103 bytes): the low 32 bits of a * b, the same signed or not, are
; aL * bL + ((aL * bH + aH * bL) mod 2^16) * 2^16 in unsigned 16-bit halves: m7_umul16 for each, the cross
; products skipped when their high half is 0 (one product for 16-bit operands, ~350 cycles).
; It needs the quarter-square tables: no i32 multiply may run before `fixed::init_tables` (which builds
; them by sums for that reason).
MW_A0 = __rc8           ; aL
MW_A1 = __rc9
MW_A2 = __rc10          ; aH
MW_A3 = __rc11
MW_C0 = __rc12          ; the cross products' sum, low 16 bits
MW_C1 = __rc13
.section .text.__wrap___mulsi3,"ax",@progbits
.globl __wrap___mulsi3
.type __wrap___mulsi3,@function
__wrap___mulsi3:
    sta MW_A0
    stx MW_A1
    lda __rc2
    sta MW_A2
    lda __rc3
    sta MW_A3
    lda #0
    sta MW_C0
    sta MW_C1
    ; aL * bH
    lda __rc6
    ora __rc7
    beq 1f
    lda __rc6
    sta __rc2
    lda __rc7
    sta __rc3
    lda MW_A0
    ldx MW_A1
    jsr m7_umul16
    sta MW_C0
    stx MW_C1
    ; + aH * bL
1:  lda MW_A2
    ora MW_A3
    beq 2f
    lda __rc4
    sta __rc2
    lda __rc5
    sta __rc3
    lda MW_A2
    ldx MW_A3
    jsr m7_umul16
    clc
    adc MW_C0
    sta MW_C0
    txa
    adc MW_C1
    sta MW_C1
    ; aL * bL, its high half plus the cross sum
2:  lda __rc4
    sta __rc2
    lda __rc5
    sta __rc3
    lda MW_A0
    ldx MW_A1
    jsr m7_umul16
    pha
    clc
    lda __rc2
    adc MW_C0
    sta __rc2
    lda __rc3
    adc MW_C1
    sta __rc3
    pla
    rts
.size __wrap___mulsi3, .-__wrap___mulsi3

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
