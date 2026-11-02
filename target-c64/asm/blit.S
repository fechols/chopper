; Copied from ../talonhd/target-c64/asm/blit.S (e0e891c): draw_cols_asm, its copy table and the line
; tables only (draw_sprite_asm assumes a full bitmap and buffers MODE 7 uses). MODE 7 change: the JOB block
; is at $8D70 (layout::JOB). geom.S's sprite_band calls it with DST in the band being built (STAGE) and
; Y relative to the band, so every line it writes is 0..7 (line_offset = the line).
;
; draw_cols_asm — the sprite kernel. Same contract as the Rust reference `talonhd::render::draw_cols`:
; draw NCOLS consecutive column records from RAM at SRC into the bitmap, the first at DST (its top
; cell), each next one 8 bytes right; sprite line l lands on screen line Y + l, lines clipped to
; [TOP, BOT).
;
; Inputs (JOB block, little-endian):
;   +0  DST    bitmap address of the first column's top cell
;   +2  Y      screen y of sprite line 0 (i16)
;   +4  TOP    first visible sprite line
;   +6  BOT    end of visible sprite lines
;   +8  SRC    RAM address of the first column record
;   +10 NCOLS  records to draw (u8)
; Record (talonhd::format): item count, then items in line order:
;   Run  line lo, line hi (bit 0), length (1..255), then the bytes (fully opaque)
;   Mix  line lo, line hi | $80, bits, keep (dst = dst & keep | bits)
;
; No REU access: the caller fetched the records (one DMA per band chunk). On the C64U an I/O write costs
; 16 RAM stores, so the bytes are scattered by the CPU: a run is copied a cell row (<= 8 bytes, adjacent
; in the bitmap) at a time.
;
; Clobbers A, X, Y and zero page $F0-$FF (outside the compiler's ZP range).
;
; Beware: the assembler does not reject out-of-range branches; it emits a wrong target. tools/test_blit.py
; runs this kernel against the reference.

JOB      = $8D70
J_DST    = JOB + 0
J_Y      = JOB + 2
J_TOP    = JOB + 4
J_BOT    = JOB + 6
J_SRC    = JOB + 8
J_NCOLS  = JOB + 10

zp_src   = $F0     ; 2: record read pointer
zp_dst   = $F2     ; 2: this column's top cell
zp_cols  = $F4     ; 1: columns left
zp_items = $F5     ; 1: items left in this column
zp_line  = $F6     ; 2: item line
zp_ptr   = $F8     ; 2: bitmap write pointer
zp_from  = $FA     ; 2: run byte read pointer
zp_n     = $FC     ; 1: bytes in this cell row
zp_rem   = $FD     ; 1: bytes left in the run
zp_sl    = $FE     ; 1: screen line
zp_len   = $FF     ; 1: run length

.section .text.draw_cols_asm,"ax",@progbits
.globl draw_cols_asm
.type draw_cols_asm,@function
draw_cols_asm:
    lda J_SRC
    sta zp_src
    lda J_SRC+1
    sta zp_src+1
    lda J_DST
    sta zp_dst
    lda J_DST+1
    sta zp_dst+1
    lda J_NCOLS
    bne column
    rts

column:
    sta zp_cols
    ldy #0
    lda (zp_src),y
    sta zp_items
    inc zp_src
    bne item
    inc zp_src+1

item:
    lda zp_items
    bne 1f
    jmp next_column     ; out of branch range
1:  dec zp_items
    ldy #0
    lda (zp_src),y
    sta zp_line
    iny
    lda (zp_src),y
    bpl run
    jmp mix             ; out of branch range

    ; --- run: copy bytes [max(line, TOP), min(line + len, BOT)) ---
run:
    and #1
    sta zp_line+1
    iny
    lda (zp_src),y
    sta zp_len
    ; e = line + len, clamped to BOT (16-bit), kept in zp_ptr for now
    clc
    lda zp_line
    adc zp_len
    sta zp_ptr
    lda zp_line+1
    adc #0
    sta zp_ptr+1
    lda zp_ptr
    cmp J_BOT
    lda zp_ptr+1
    sbc J_BOT+1
    bcc 2f
    lda J_BOT
    sta zp_ptr
    lda J_BOT+1
    sta zp_ptr+1
2:  ; s = max(line, TOP), kept in zp_from for now
    lda zp_line
    cmp J_TOP
    lda zp_line+1
    sbc J_TOP+1
    bcs 3f
    lda J_TOP
    sta zp_from
    lda J_TOP+1
    sta zp_from+1
    jmp 4f
3:  lda zp_line
    sta zp_from
    lda zp_line+1
    sta zp_from+1
4:  ; nothing visible when s >= e
    lda zp_from
    cmp zp_ptr
    lda zp_from+1
    sbc zp_ptr+1
    bcc 90f
    jmp run_done
90:
    sec
    lda zp_ptr
    sbc zp_from
    sta zp_rem
    ; sl = Y + s
    clc
    lda J_Y
    adc zp_from
    sta zp_sl
    ; from = src + (s - line) + 3   (s - line fits a byte)
    sec
    lda zp_from
    sbc zp_line
    clc
    adc zp_src
    sta zp_from
    lda zp_src+1
    adc #0
    sta zp_from+1
    clc
    lda zp_from
    adc #3
    sta zp_from
    bcc 1f
    inc zp_from+1
1:

    ; y indexes the run's bytes for the whole run; each cell row's bitmap pointer is biased by -y so
    ; (zp_ptr),y lands on it while y keeps counting (no source pointer update per row).
    ldy #0
cell:
    ; n = min(8 - (sl & 7), rem)
    lda zp_sl
    and #7
    eor #$ff
    sec
    adc #8
    cmp zp_rem
    bcc 5f
    lda zp_rem
5:  sta zp_n
    ; ptr = dst + line offset(sl) - y   (zp_line is free now: a scratch word)
    sty zp_line
    ldx zp_sl
    clc
    lda zp_dst
    adc line_lo,x
    sta zp_ptr
    lda zp_dst+1
    adc line_hi,x
    sta zp_ptr+1
    sec
    lda zp_ptr
    sbc zp_line
    sta zp_ptr
    bcs 6f
    dec zp_ptr+1
6:  ; copy n bytes: enter the unrolled 8 at 8 - n (Duff's device)
    lda zp_n
    asl a
    tax
    lda copy_entry,x
    sta zp_line
    lda copy_entry+1,x
    sta zp_line+1
    jmp (zp_line)
copy8:
    .rept 8
    lda (zp_from),y
    sta (zp_ptr),y
    iny
    .endr
copied:
    ; sl += n, rem -= n
    clc
    lda zp_sl
    adc zp_n
    sta zp_sl
    sec
    lda zp_rem
    sbc zp_n
    sta zp_rem
    bne cell
run_done:
    ; src += 3 + len
    clc
    lda zp_src
    adc #3
    bcc 8f
    inc zp_src+1
    clc
8:  adc zp_len
    sta zp_src
    bcc 9f
    inc zp_src+1
9:  jmp item

    ; --- mix: blend one byte if its line is visible ---
mix:
    and #1
    sta zp_line+1
    lda zp_line
    cmp J_TOP
    lda zp_line+1
    sbc J_TOP+1
    bcc mix_done
    lda zp_line
    cmp J_BOT
    lda zp_line+1
    sbc J_BOT+1
    bcs mix_done
    clc
    lda J_Y
    adc zp_line
    tax
    clc
    lda zp_dst
    adc line_lo,x
    sta zp_ptr
    lda zp_dst+1
    adc line_hi,x
    sta zp_ptr+1
    ldy #3
    lda (zp_src),y
    ldy #0
    and (zp_ptr),y
    ldy #2
    ora (zp_src),y
    ldy #0
    sta (zp_ptr),y
mix_done:
    clc
    lda zp_src
    adc #4
    sta zp_src
    bcc 1f
    inc zp_src+1
1:  jmp item

next_column:
    clc
    lda zp_dst
    adc #8
    sta zp_dst
    bcc 1f
    inc zp_dst+1
1:  dec zp_cols
    beq 2f
    lda zp_cols
    jmp column
2:  rts
.size draw_cols_asm, .-draw_cols_asm

; draw_cols_asm's unrolled copy: the entry for n bytes, n = 0..8 (each step is 5 bytes: lda (zp),y / sta (zp),y / iny).
.section .rodata.copy_entry,"a",@progbits
copy_entry:
    .word copied, copy8 + 35, copy8 + 30, copy8 + 25, copy8 + 20, copy8 + 15, copy8 + 10, copy8 + 5, copy8

; Byte offset of screen line y within a bitmap column: (y/8)*320 + (y&7). MODE 7 draws one band (cell
; row) at a time with Y relative to it, so only lines 0-7 occur.
.section .rodata.line_tables,"a",@progbits
.globl line_lo
.globl line_hi
line_lo:
.set yy, 0
.rept 8
    .byte ((((yy >> 3) * 320) + (yy & 7)) & 0xff)
.set yy, yy + 1
.endr
line_hi:
.set yy, 0
.rept 8
    .byte ((((yy >> 3) * 320) + (yy & 7)) >> 8)
.set yy, yy + 1
.endr
