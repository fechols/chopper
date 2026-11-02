; MODE 7 milestone 2: C64U unit costs of the mode 7 kernels (src/mode7.rs `cycles`).
; Each test is timed with CIA 2 timer A (Phi2 ticks, ~0.978 us on NTSC); results are u16 ticks at
; $C000 + 2 * slot. IRQs are off for the whole run. Pattern: ../talonhd/target-c64/asm/bench.S.
;
; slot 0: empty loop x4096          slot 1: 1-byte DMA, command only x4096
; slot 2: lo + command x4096        slot 3: lo + mid + command x4096
; slot 4: the per-byte DMA kernel (8-line DDA, shadowed REU address), 4096 bytes
; slot 5: CPU gather, 4 texels a byte, 1024 bytes      slot 6: CPU gather, 1 texel a byte, 2048 bytes
; slot 7: I/O write alone (sta $df04) x4096

SHLO  = $F0
SHMID = $F1
CNT   = $F2
UF    = $F4
VF    = $F5
VI    = $F6
DUF   = $F7
DUI   = $F8
DVF   = $F9
DVI   = $FA
PTR   = $FB
ACC   = $FD

.macro tstart
    lda #$ff
    sta $dd04
    sta $dd05
    lda #$11
    sta $dd0e
.endm

.macro tstop slot
    lda #$00
    sta $dd0e
    lda $dd04
    eor #$ff
    sta $c000+\slot*2
    lda $dd05
    eor #$ff
    sta $c001+\slot*2
.endm

.macro texel mask
    clc
    lda UF
    adc DUF
    sta UF
    tya
    adc DUI
    and #$7f
    tay
    clc
    lda VF
    adc DVF
    sta VF
    lda VI
    adc DVI
    and #$7f
    sta VI
    tax
    lda rowhi,x
    sta PTR+1
    lda rowlo,x
    sta PTR
    lda (PTR),y
    and #\mask
    ora ACC
    sta ACC
.endm

.section .text.bench,"ax",@progbits

dma_setup:
    lda #$00
    sta $df02
    lda #$40
    sta $df03
    lda #1
    sta $df07
    lda #0
    sta $df08
    sta $df04
    sta $df05
    sta $df06
    sta $df09
    sta $df0a
    rts

gather_setup:
    lda #0
    sta UF
    sta VF
    lda #10
    sta VI
    lda #$bb
    sta DUF
    lda #0
    sta DUI
    sta DVI
    lda #$40
    sta DVF
    ldy #20
    rts

; Turbo probe: slot 0's empty loop (4096 x 5 cycles) timed into $C044/$C045. ~20,500 ticks at 1 MHz,
; ~320 at 64 MHz. The C64U engages turbo seconds after $D031 is written (TALON: ~2.5 s), so main.c
; repeats this until it is fast.
.globl turbo_probe
turbo_probe:
    sei
    ldy #16
    ldx #0
    tstart
0:  inx
    bne 0b
    dey
    bne 0b
    lda #$00
    sta $dd0e
    lda $dd04
    eor #$ff
    sta $c044
    lda $dd05
    eor #$ff
    sta $c045
    cli
    rts

.globl bench_all
bench_all:
    sei

    ; slot 0
    ldy #16
    ldx #0
    tstart
0:  inx
    bne 0b
    dey
    bne 0b
    tstop 0

    ; slot 1
    jsr dma_setup
    ldy #16
    ldx #0
    tstart
    lda #$91
1:  sta $df01
    inx
    bne 1b
    dey
    bne 1b
    tstop 1

    ; slot 2
    jsr dma_setup
    ldy #16
    ldx #0
    tstart
    lda #$91
2:  stx $df04
    sta $df01
    inx
    bne 2b
    dey
    bne 2b
    tstop 2

    ; slot 3
    jsr dma_setup
    ldy #16
    ldx #0
    tstart
    lda #$91
3:  stx $df04
    stx $df05
    sta $df01
    inx
    bne 3b
    dey
    bne 3b
    tstop 3

    ; slot 4: 512 cells x 8 lines
    jsr dma_setup
    lda #0
    sta SHLO
    sta SHMID
    sta CNT
    lda #2
    sta CNT+1
    tstart
4:  ldx #0
5:  clc
    lda ul,x
    adc sul,x
    sta ul,x
    lda uh,x
    adc suh,x
    sta uh,x
    cmp SHLO
    beq 6f
    sta $df04
    sta SHLO
6:  clc
    lda vl,x
    adc svl,x
    sta vl,x
    lda vh,x
    adc svh,x
    sta vh,x
    cmp SHMID
    beq 7f
    sta $df05
    sta SHMID
7:  lda #$91
    sta $df01
    inc SHLO
    bne 8f
    inc SHMID
8:  inx
    cpx #8
    bne 5b
    dec CNT
    bne 4b
    dec CNT+1
    bne 4b
    tstop 4

    ; slot 5: 1024 bytes x 4 texels
    jsr gather_setup
    lda #$00
    sta g4st+1
    lda #$60
    sta g4st+2
    lda #0
    sta CNT
    lda #4
    sta CNT+1
    tstart
10: lda #0
    sta ACC
    texel $c0
    texel $30
    texel $0c
    texel $03
    lda ACC
g4st:
    sta $6000
    lda g4st+1
    clc
    adc #8
    sta g4st+1
    bcc 11f
    inc g4st+2
11: dec CNT
    beq 90f
    jmp 10b         ; the 4-texel body is out of branch range (llvm-mc mis-encodes silently)
90: dec CNT+1
    beq 91f
    jmp 10b
91: tstop 5

    ; slot 6: 2048 bytes x 1 texel
    jsr gather_setup
    lda #$00
    sta g1st+1
    lda #$60
    sta g1st+2
    lda #0
    sta CNT
    lda #8
    sta CNT+1
    tstart
12: lda #0
    sta ACC
    texel $ff
    lda ACC
g1st:
    sta $6000
    lda g1st+1
    clc
    adc #8
    sta g1st+1
    bcc 13f
    inc g1st+2
13: dec CNT
    bne 12b
    dec CNT+1
    bne 12b
    tstop 6

    ; slot 7
    ldy #16
    ldx #0
    tstart
14: stx $df04
    inx
    bne 14b
    dey
    bne 14b
    tstop 7
    cli
    rts
