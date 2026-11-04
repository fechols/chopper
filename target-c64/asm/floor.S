; Floor kernel: one view line drawn from its runs (`floor::make_line`, encoded by `floor::encode` at
; SPANS), one REU DMA per ground byte into the 40-byte line buffer, then copied into the band stage.
; Reference: `floor::draw_line` and `floor::copy_line` (src/floor.rs); `tools/emu.py` checks the frames
; against the host byte for byte.
;
; Run record (12 bytes, written by asm/spans.S): len, flags (bit 7: fill; else the mip level), then a
; fill byte (at 2), or u (16.8, bytes 2-4), v (5-7), du (8.8, 8-9), dv (10-11). Ground: the REU address is bank:v_hi:u_hi, the bank from the
; level's tile table (TILES + 256·level, entry (v_tile & 15)·16 + (u_tile & 15)), looked up again only
; when a tile coordinate changes. Mid and bank are written when they change (shadowed), lo always and
; last (the C64U reloads the whole address on any address write). The C64 address is set at each ground
; run's start and auto-increments.
;
; In a magnified run (steps within half a texel a byte), a ground byte at the address of the last fetch (the
; texel repeated) is copied from the byte before it instead of fetched again; the next fetch then sets
; its C64 address again. Other runs take the plain loop.
;
; Zero page $A0-$B6 while a line is drawn (nothing is kept there between calls: the REU shadows live in
; SAVE_MID / SAVE_BANK).

RUNP     = $A0    ; (2) the current run record
XB     = $A2    ; next byte of the line buffer
REM    = $A3    ; bytes left in the run
LEN    = $A4    ; in a ground run: nonzero when the next fetch must set its C64 address (XB)
UL     = $A5    ; u: 16.8 (UHH = tile column)
UH     = $A6
UHH    = $A7
VL     = $A8
VH     = $A9
VHH    = $AA
DUL    = $AB    ; du: 8.8, DUS its sign extension
DUH    = $AC
DUS    = $AD
DVL    = $AE
DVH    = $AF
DVS    = $B0
BK     = $B1
SHMID  = $B2
SHBANK = $B3
TP     = $B4    ; (2) the level's tile table
SHLO   = $B6    ; the REU address lo of the last fetch (with SHMID and SHBANK, its whole address)

LINEBUF = $BF00
TILES   = $C000
SPANS   = $C400
STAGE   = $8C00
SAVE_MID  = $C7D4
SAVE_BANK = $C7D5
FETCH   = $91
RECORD  = 12
COLS    = 40

; Mid and bank written when they change (X = how many did).
.macro SHADOWS
    ldx #0
    lda VH
    cmp SHMID
    beq 4f
    sta $df05
    sta SHMID
    inx
4:  lda BK
    cmp SHBANK
    beq 5f
    sta $df06
    sta SHBANK
    inx
5:
.endm

; u += du, v += dv; a new tile coordinate means a new bank.
.macro STEP
    ldx #0
    clc
    lda UL
    adc DUL
    sta UL
    lda UH
    adc DUH
    sta UH
    lda UHH
    adc DUS
    cmp UHH
    beq 6f
    sta UHH
    inx
6:  clc
    lda VL
    adc DVL
    sta VL
    lda VH
    adc DVH
    sta VH
    lda VHH
    adc DVS
    cmp VHH
    beq 7f
    sta VHH
    inx
7:  txa
    beq 8f
    jsr lookup
8:
.endm

.section .text.floor_draw,"ax",@progbits

; floor_begin(): once a frame before any floor_line: one-byte transfers, no fixed address, and a known
; REU address in the shadows (`floor::floor_begin`).
.globl floor_begin
.type floor_begin,@function
floor_begin:
    lda #1
    sta $df07
    lda #0
    sta $df08
    sta $df0a
    sta $df04
    sta $df05
    sta $df06
    sta SAVE_MID
    sta SAVE_BANK
    rts
.size floor_begin, .-floor_begin

; floor_end(): nothing to undo ($DF0A stays 0); kept for the frame loop's shape.
.globl floor_end
.type floor_end,@function
floor_end:
    rts
.size floor_end, .-floor_end

; floor_line(): draw the runs at SPANS (covering 40 bytes) into LINEBUF.
.globl floor_line
.type floor_line,@function
floor_line:
    lda #<SPANS
    sta RUNP
    lda #>SPANS
    sta RUNP+1
    lda SAVE_MID
    sta SHMID
    lda SAVE_BANK
    sta SHBANK
    lda #>LINEBUF
    sta $df03
    lda #0
    sta XB
    sta TP
run:
    ldy #0
    lda (RUNP),y
    sta LEN
    sta REM
    iny
    lda (RUNP),y
    bpl ground
    ; ---- fill run: CPU stores ----
    iny
    lda (RUNP),y
    ldx XB
    ldy REM
1:  sta LINEBUF,x
    inx
    dey
    bne 1b
    stx XB
    jmp next

ground:
    clc
    adc #>TILES
    sta TP+1
    ldy #2
    lda (RUNP),y
    sta UL
    iny
    lda (RUNP),y
    sta UH
    iny
    lda (RUNP),y
    sta UHH
    iny
    lda (RUNP),y
    sta VL
    iny
    lda (RUNP),y
    sta VH
    iny
    lda (RUNP),y
    sta VHH
    iny
    lda (RUNP),y
    sta DUL
    iny
    lda (RUNP),y
    sta DUH
    ldx #0
    asl
    bcc 2f
    dex
2:  stx DUS
    iny
    lda (RUNP),y
    sta DVL
    iny
    lda (RUNP),y
    sta DVH
    ldx #0
    asl
    bcc 3f
    dex
3:  stx DVS
    jsr lookup
    ; A magnified run (both steps within half a texel a byte: -$80 <= d < $80 in 8.8) repeats texels
    ; often enough to look for them; nearer 1 they are too rare to pay for the test.
    lda DUL
    clc
    adc #$80
    lda DUH
    adc #0
    bne plain
    lda DVL
    clc
    adc #$80
    lda DVH
    adc #0
    beq magnified
plain:
    ; The C64 address: LINEBUF + XB (LINEBUF is page-aligned), auto-incremented by each fetch.
    lda XB
    sta $df02
pbyte:
    SHADOWS
    lda UH
    sta $df04
    lda #FETCH
    sta $df01
    STEP
    dec REM
    beq 9f
    jmp pbyte
9:  clc
    lda XB
    adc LEN
    sta XB
    jmp next

magnified:
    ; The first byte is fetched (SHLO cannot match), into LINEBUF + XB (LEN, nonzero, says to set it).
    lda UH
    eor #$ff
    sta SHLO
mbyte:
    SHADOWS
    lda UH
    cmp SHLO
    bne 1f
    txa
    bne 1f
    ; The last fetch's texel again (never a run's first byte, so XB > 0).
    ldx XB
    lda LINEBUF-1,x
    sta LINEBUF,x
    stx LEN
    jmp 6f
    ; The C64 address after a copied byte or at the run's start.
1:  ldy LEN
    beq 2f
    ldy XB
    sty $df02
    ldy #0
    sty LEN
2:  lda UH
    sta $df04
    sta SHLO
    lda #FETCH
    sta $df01
6:  inc XB
    STEP
    dec REM
    beq next
    jmp mbyte

next:
    lda XB
    cmp #COLS
    bcs done
    clc
    lda RUNP
    adc #RECORD
    sta RUNP
    bcc 9f
    inc RUNP+1
9:  jmp run
done:
    lda SHMID
    sta SAVE_MID
    lda SHBANK
    sta SAVE_BANK
    rts

; BK = the level's tile table at (VHH & 15) · 16 + (UHH & 15).
lookup:
    lda VHH
    asl
    asl
    asl
    asl
    sta BK
    lda UHH
    and #15
    ora BK
    tay
    lda (TP),y
    sta BK
    rts
.size floor_line, .-floor_line

; floor_copy(A = band line 0..7, X = doubled): LINEBUF into STAGE column by column (+8 a column), and
; into the next line too when doubled. Unrolled (9 cycles a byte); it lives in the clipping overlay
; (window 3, $F010), which is loaded before the band loop and not overwritten until the next frame.
.section .text.clip.floor_copy,"ax",@progbits
.globl floor_copy
.type floor_copy,@function
floor_copy:
    cpx #0
    beq 1f
    tax
    jsr fc_line
    inx
    bne fc_line         ; (X = line + 1: always)
1:  tax
fc_line:
.set fci, 0
.rept COLS
    lda LINEBUF+fci
    sta STAGE+fci*8,x
.set fci, fci+1
.endr
    rts
.size floor_copy, .-floor_copy

; turbo_probe() -> u16: CIA 2 timer A ticks for 4096 x 5 cycles (~20,500 at 1 MHz, ~320 at 64 MHz).
; The C64U engages turbo ~2 s after $D031 is written; main repeats this until it is fast.
; (Boot only: in the boot code, not relocated.)
.section .text.boot.turbo_probe,"ax",@progbits
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

; band_publish(A = band): copy STAGE (320 bytes) to the bitmap's cell row, once the raster is not in it
; (rows show on raster lines 51 + 8 * band ..; a copy takes about a line, so start 2 lines early).
BITMAP = $6000
PB_TOP = $B4
PB_BOT = $B5
PB_D   = $B6        ; (2)

.section .text.band_publish,"ax",@progbits
.globl band_publish
.type band_publish,@function
band_publish:
    tax
    clc
    lda row320lo,x
    adc #<BITMAP
    sta PB_D
    lda row320hi,x
    adc #>BITMAP
    sta PB_D+1
    txa
    asl
    asl
    asl
    clc
    adc #49
    sta PB_TOP
    adc #10
    sta PB_BOT          ; first safe line after the row
1:  lda $d011
    bmi 2f              ; line >= 256: below every row
    lda $d012
    cmp PB_TOP
    bcc 2f
    cmp PB_BOT
    bcc 1b
2:  ldy #0
3:  lda STAGE,y
    sta (PB_D),y
    iny
    bne 3b
    inc PB_D+1
    ldy #63
4:  lda STAGE+256,y
    sta (PB_D),y
    dey
    bpl 4b
    rts
.size band_publish, .-band_publish
