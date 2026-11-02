; REU and timing helpers.

; reu_execute(command): start an REU command. A foreign call, so LLVM must
; assume any escaped RAM changed (a volatile MMIO store alone does not model
; the DMA's writes).
.section .text.reu_execute,"ax",@progbits
.globl reu_execute
.type reu_execute,@function
reu_execute:
    sta $df01
    rts
.size reu_execute, .-reu_execute

; turbo_probe() -> u16: CIA 2 timer A ticks for 4096 x 5 cycles (~20,500 at
; 1 MHz, ~320 at 64 MHz). The C64U engages turbo ~2 s after $D031 is written;
; main repeats this until it is fast. (From ../mode7/target-c64/asm/floor.S.)
.section .text.turbo_probe,"ax",@progbits
.globl turbo_probe
.type turbo_probe,@function
turbo_probe:
    lda #$ff
    sta $dd04
    sta $dd05
    lda #$11
    sta $dd0e
    ldy #16
    ldx #0
1:  inx
    bne 1b
    dey
    bne 1b
    lda #0
    sta $dd0e
    lda $dd05
    eor #$ff
    tax
    lda $dd04
    eor #$ff
    rts
.size turbo_probe, .-turbo_probe
