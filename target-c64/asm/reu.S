; Start an REU command. A foreign call, so LLVM must assume any escaped RAM
; changed (a volatile MMIO store alone does not model the DMA's writes).
.section .text.reu_execute,"ax",@progbits
.globl reu_execute
.type reu_execute,@function
reu_execute:
    sta $df01
    rts
.size reu_execute, .-reu_execute


; memcpy(dst = __rc2/3, src = __rc4/5, n = A/X): a byte loop, page by page (compiler_builtins' would be
; 536 bytes of resident code for the few small copies the Rust makes). Returns dst in __rc2/3.
.section .text.memcpy,"ax",@progbits
.globl memcpy
.type memcpy,@function
memcpy:
    sta __rc6
    lda __rc3
    pha
    lda __rc5
    pha
    ldy #0
    cpx #0
    beq 2f
1:  lda (__rc4),y
    sta (__rc2),y
    iny
    bne 1b
    inc __rc5
    inc __rc3
    dex
    bne 1b
2:  ldx __rc6
    beq 4f
3:  lda (__rc4),y
    sta (__rc2),y
    iny
    dex
    bne 3b
4:  pla
    sta __rc5
    pla
    sta __rc3
    rts
.size memcpy, .-memcpy
