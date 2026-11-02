; Start an REU command. A foreign call, so LLVM must assume any escaped RAM
; changed (a volatile MMIO store alone does not model the DMA's writes).
.section .text.reu_execute,"ax",@progbits
.globl reu_execute
.type reu_execute,@function
reu_execute:
    sta $df01
    rts
.size reu_execute, .-reu_execute

