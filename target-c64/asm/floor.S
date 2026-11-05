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
FLV     = $C609       ; layout::FLOOR_LEVEL: the ground run's mip level
TABLE_LEVELS = 3      ; floor::TABLE_LEVELS: levels from here are one tile (`lookup`)
SEA_BANK = 85         ; format::SEA_BANK
PATCH   = $C5F0       ; layout::FOG_PATCHES (`floor::Patches`): count, then byte, keep, bits
SAVE_MID  = $C7D4
SAVE_BANK = $C7D5
FETCH   = $91
RECORD  = 12
COLS    = 39          ; floor::DRAWN (layout::DRAWN_COLS): the bytes a line draws

; Mid written when it changes (X = 1 if it did). The bank is written by `lookup`, when it changes: a new
; bank means a new tile, so a new u or v texel too (steps here are under a texel).
.macro SHADOWS
    ldx #0
    lda VH
    cmp SHMID
    beq 4f
    sta $df05
    sta SHMID
    inx
4:
.endm

; u += du, v += dv; a new tile coordinate means a new bank (looked up again for each axis that moved).
.macro STEP
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
    jsr lookup
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
    jsr lookup
7:
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
    sta FLV
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
    ; The C64 address: LINEBUF + XB (LINEBUF is page-aligned), auto-incremented by each fetch. Each
    ; step writes the next byte's mid (always) and lo (last); `lookup` writes the bank.
    lda XB
    sta $df02
    lda VH
    sta $df05
    lda UH
    sta $df04
pbyte:
    lda #FETCH
    sta $df01
    dec REM
    beq pdone
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
    bne ptile_u
pv: clc
    lda VL
    adc DVL
    sta VL
    lda VH
    adc DVH
    sta VH
    sta $df05
    lda VHH
    adc DVS
    cmp VHH
    bne ptile_v
plo:
    lda UH
    sta $df04
    jmp pbyte
ptile_u:
    sta UHH
    jsr lookup
    jmp pv
ptile_v:
    sta VHH
    jsr lookup
    jmp plo
pdone:
    lda VH
    sta SHMID
    clc
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
    ; the fog line over it (`floor::draw_line`): LINEBUF[byte] = LINEBUF[byte] & keep | bits
    lda PATCH
    beq 2f
    sta REM
    ldy #0
1:  ldx PATCH+1,y
    lda LINEBUF,x
    and PATCH+2,y
    ora PATCH+3,y
    sta LINEBUF,x
    iny
    iny
    iny
    dec REM
    bne 1b
2:  lda SHMID
    sta SAVE_MID
    lda SHBANK
    sta SAVE_BANK
    rts

; BK = the level's tile table at (VHH & 15) · 16 + (UHH & 15); for a one-tile level, its bank at tile 0
; and the sea elsewhere (`floor::tile_bank`).
lookup:
    lda FLV
    cmp #TABLE_LEVELS
    bcs 2f
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
3:  sta BK
    cmp SHBANK
    beq 1f
    sta $df06
    sta SHBANK
1:  rts
2:  lda UHH
    ora VHH
    and #15
    bne 4f
    ldy FLV
    lda lvbank - TABLE_LEVELS,y
    bne 3b
4:  lda #SEA_BANK
    bne 3b
; format::LEVEL_BANK from TABLE_LEVELS on
lvbank: .byte 84, 89, 86
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

; ray_step(): RAYS.a += RAY_STEP, three i32s (`layout::RAY_STEP`), the next drawn line's ray.
R_A      = $C640 + 61
RAY_STEP = $C760
.section .text.clip.ray_step,"ax",@progbits
.globl ray_step
.type ray_step,@function
ray_step:
.set rsi, 0
.rept 3
    clc
.rept 4
    lda R_A+rsi
    adc RAY_STEP+rsi
    sta R_A+rsi
.set rsi, rsi+1
.endr
.endr
    rts
.size ray_step, .-ray_step

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

; X:Y += 56, into the store at dst's operand.
.macro PBNEXT56 dst
    txa
    clc
    adc #56
    tax
    bcc 6f
    iny
6:  stx \dst+1
    sty \dst+2
.endm

; X:Y += 64, into the store at dst's operand.
.macro PBNEXT dst
    txa
    clc
    adc #64
    tax
    bcc 6f
    iny
6:  stx \dst+1
    sty \dst+2
.endm

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
    adc #46             ; (the shake's fine scroll moves the row by -3..+4 lines)
    sta PB_TOP
    adc #17
    sta PB_BOT          ; first safe line after the row
1:  lda $d011
    bmi 2f              ; line >= 256: below every row
    lda $d012
    cmp PB_TOP
    bcc 2f
    cmp PB_BOT
    bcc 1b
    ; Five 64-byte stripes a pass (the stores' operands set to PB_D + 64 k first).
2:  ldx PB_D
    ldy PB_D+1
    stx pbs0+1
    sty pbs0+2
    PBNEXT pbs1
    PBNEXT pbs2
    PBNEXT pbs3
    PBNEXT56 pbs4           ; (the last stripe from 248: bytes 0-311, the drawn columns)
    ldy #0
3:  lda STAGE,y
pbs0: sta $ffff,y
    lda STAGE+64,y
pbs1: sta $ffff,y
    lda STAGE+128,y
pbs2: sta $ffff,y
    lda STAGE+192,y
pbs3: sta $ffff,y
    lda STAGE+248,y         ; (bytes 248-311: the last overlaps the fourth stripe by 8)
pbs4: sta $ffff,y
    iny
    cpy #64
    bne 3b
    rts
.size band_publish, .-band_publish
