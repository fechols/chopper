; MODE 7 floor kernel: one REU DMA per bitmap byte (or per line pair), the REU address stepped by a
; per-line DDA. Reference: `mode7::per_byte_dma` with `DmaMode { pre: true, first_row: 6, doubled }`
; (src/mode7.rs); `tools/emu.py` checks this file against it byte for byte.
;
; floor_begin / floor_row / floor_end: draws cell rows of the floor (6-24) one at a time, each into a
; 320-byte destination (the band being built, `layout::STAGE`), from the line table at $C000 (field-major, 200 lines: u0 lo/hi, v0 lo/hi offsets from the camera, du*4 lo/hi, dv*4
; lo/hi, bank) and the camera at CAMX/CAMY (8.8, zero page, set by the caller); the camera is added as
; each cell row's lines are loaded (`mode7::lines_at`). The bitmap is walked in memory order (per cell row: per column: its 8 lines), so the
; C64 address auto-increments. Full: mid and bank are written when they change (shadowed in zero page),
; lo always and last (see fullbyte). Doubled: `$DF0A` bit 6 fixes the REU address and each DMA is 2 bytes (the length register is
; rewritten every time: a transfer leaves it at 1), so lines 2k and 2k+1 get line 2k's byte.
;
; Zero page $A0-$FF is this file's (the compiler's is $02-$8F; no KERNAL).

UL     = $A0    ; 8 lines' u accumulators, lo
UH     = $A8    ;                          hi = REU address lo
VL     = $B0
VH     = $B8    ; = REU address mid
SUL    = $C0    ; steps per byte
SUH    = $C8
SVL    = $D0
SVH    = $D8
BK     = $E0    ; banks
SHLO   = $F0    ; REU address registers as the last DMA left them
SHMID  = $F1
SHBANK = $F2
ROW    = $F3    ; first line of the cell row
COL    = $F4
MODE   = $F5
TMP    = $F6
CAMX   = $F8    ; u16, set by the caller
CAMY   = $FA    ; u16

T_U0L = $C000
T_U0H = $C0C8
T_V0L = $C190
T_V0H = $C258
T_DUL = $C320
T_DUH = $C3E8
T_DVL = $C4B0
T_DVH = $C578
T_BK  = $C640

FIRST_LINE = 48
FETCH      = $91

.section .text.floor_draw,"ax",@progbits
; floor_begin(A = doubled): the length / control registers and a known REU address to shadow, once a
; frame. Nothing else may DMA until floor_end (the shadows and $DF0A must stay valid).
.globl floor_begin
.type floor_begin,@function
CAM_RAM    = $C7D4
FLOOR_MODE = $C7D8

floor_begin:
    sta MODE
    sta FLOOR_MODE
    ldx #1
    lda #0
    ldy MODE
    beq 1f
    ldx #2
    lda #$40
1:  stx $df07
    sta $df0a
    lda #0
    sta $df08
    sta $df04
    sta $df05
    sta $df06
    sta SHLO
    sta SHMID
    sta SHBANK
    rts

; floor_end(): $DF0A back to 0 (every other DMA assumes it).
.globl floor_end
.type floor_end,@function
floor_end:
    lda #0
    sta $df0a
    rts

; floor_row(dest: u16 in A/X, line: u8 in __rc2): one cell row (lines line..line+7; line >= 48) into the
; 320 bytes at dest.
.globl floor_row
.type floor_row,@function
floor_row:
    sta $df02
    stx $df03
    lda __rc2
    sta ROW
    ; The geometry kernels ran since the last row and used this zero page: restore the camera and mode,
    ; and start from a known REU address to shadow.
    ldx #3
1:  lda CAM_RAM,x
    sta CAMX,x
    dex
    bpl 1b
    lda FLOOR_MODE
    sta MODE
    lda #0
    sta $df04
    sta $df05
    sta $df06
    sta SHLO
    sta SHMID
    sta SHBANK

rowloop:
    ldx #0
    ldy ROW
2:  clc
    lda T_U0L,y
    adc CAMX
    sta UL,x
    lda T_U0H,y
    adc CAMX+1
    sta UH,x
    clc
    lda T_V0L,y
    adc CAMY
    sta VL,x
    lda T_V0H,y
    adc CAMY+1
    sta VH,x
    lda T_DUL,y
    sta SUL,x
    lda T_DUH,y
    sta SUH,x
    lda T_DVL,y
    sta SVL,x
    lda T_DVH,y
    sta SVH,x
    lda T_BK,y
    sta BK,x
    iny
    inx
    cpx #8
    bne 2b
    lda #40
    sta COL
    lda MODE
    beq fullcol
    jmp dblcol

; ---- full: one byte a DMA, the REU address auto-increments ----
fullcol:
    ldx #0
fullbyte:
    ; The C64U reloads the whole REU address from the values last written to $DF04-$DF06 whenever one
    ; of them is written (an auto-incremented lo is lost), so mid and bank go first and lo is always
    ; written last: the address never depends on the auto-increment.
    lda VH,x
    cmp SHMID
    beq 4f
    sta $df05
    sta SHMID
4:  lda BK,x
    cmp SHBANK
    beq 5f
    sta $df06
    sta SHBANK
5:  lda UH,x
    sta $df04
    lda #FETCH
    sta $df01
    clc
    lda UL,x
    adc SUL,x
    sta UL,x
    lda UH,x
    adc SUH,x
    sta UH,x
    clc
    lda VL,x
    adc SVL,x
    sta VL,x
    lda VH,x
    adc SVH,x
    sta VH,x
    inx
    cpx #8
    beq 7f
    jmp fullbyte
7:  dec COL
    beq rownext
    jmp fullcol

; ---- doubled: one 2-byte DMA a line pair, the REU address fixed ----
dblcol:
    ldx #0
dblbyte:
    lda UH,x
    cmp SHLO
    beq 3f
    sta $df04
    sta SHLO
3:  lda VH,x
    cmp SHMID
    beq 4f
    sta $df05
    sta SHMID
4:  lda BK,x
    cmp SHBANK
    beq 5f
    sta $df06
    sta SHBANK
5:  lda #2
    sta $df07
    lda #FETCH
    sta $df01
    clc
    lda UL,x
    adc SUL,x
    sta UL,x
    lda UH,x
    adc SUH,x
    sta UH,x
    clc
    lda VL,x
    adc SVL,x
    sta VL,x
    lda VH,x
    adc SVH,x
    sta VH,x
    inx
    inx
    cpx #8
    beq 7f
    jmp dblbyte
7:  dec COL
    beq rownext
    jmp dblcol

rownext:
    rts
.size floor_row, .-floor_row

; turbo_probe() -> u16: CIA 2 timer A ticks for 4096 x 5 cycles (~20,500 at 1 MHz, ~320 at 64 MHz).
; The C64U engages turbo ~2 s after $D031 is written; main repeats this until it is fast.
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

; band_publish(A = band): copy STAGE (320 bytes) to the bitmap's cell row, once the raster is not in it
; (rows show on raster lines 51 + 8 * band ..; a copy takes about a line, so start 2 lines early).
STAGE = $8C00
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
