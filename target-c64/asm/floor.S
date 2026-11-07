; Floor kernel: one view line drawn from its runs (`floor::make_line`, encoded by `floor::encode` at
; SPANS), one REU DMA per ground byte into the 40-byte line buffer, then copied into the band stage.
; Reference: `floor::draw_line` and `floor::copy_line` (src/floor.rs); `tools/emu.py` checks the frames
; against the host byte for byte.
;
; Run record (12 bytes, written by asm/spans.S): len, flags (bit 7: fill; else the mip level), then a
; fill byte (at 2), or u (16.8, bytes 2-4), v (5-7), du (8.8, 8-9), dv (10-11). Ground: the REU address is
; bank:v_hi:u_hi, the bank from the level's tile table (TILES + 256·level, entry (v_tile & 15)·16 +
; (u_tile & 15)), looked up again only when a tile coordinate changes.
;
; An I/O write costs about 67 cycles on the C64U, a one-byte DMA about two: a fetch writes as few address
; registers as it can (`ADDRESS`). Mid and bank are written when they change (shadowed), lo last (the C64U
; reloads the whole address from the values last written on any address write). When nothing changed but
; u_hi = the last fetch's + 1, nothing is written at all: the transfer's own increment already holds the
; address (it survives only while no address register is written, so this is the one case it is used).
;
; Plain runs (a step over a texel a byte) fetch each byte into LINEBUF + XB, the C64 address set at the
; run's start and auto-incremented. Runs within a texel a byte (du_hi and dv_hi 0 or -1) fetch each texel
; once into SCRATCH (a fixed C64 address) and store it for every byte it covers: the fractions' carries say
; when u or v crosses a texel (`magnified`), so a repeated texel costs neither a fetch nor a compare.
;
; Zero page $A0-$B7 while a line is drawn (nothing is kept there between calls: the REU shadows live in
; SAVE_MID / SAVE_BANK).

RUNP     = $A0    ; (2) the current run record
XB     = $A2    ; next byte of the line buffer
REM    = $A3    ; bytes left in the run
LEN    = $A4    ; the run's length
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
SHLO   = $B6    ; the REU address lo of the last fetch ($FF: none, so the next writes lo)
BANKOFF = $B7   ; added to every bank: 0 on even lines, format::ODD_BANK_OFFSET on odd (their copy)

LINEBUF = $BF00
SCRATCH = $BFFF       ; the fixed C64 address of a magnified run's fetches (LINEBUF's page, past the line)
TILES   = $C000
SPANS   = $C400
STAGE   = $8C00
FLV     = $C609       ; layout::FLOOR_LEVEL: the ground run's mip level
TABLE_LEVELS = 3      ; floor::TABLE_LEVELS: levels from here are one tile (`lookup`)
SEA_BANK = 85         ; format::SEA_BANK
PATCH   = $C5F0       ; layout::FOG_PATCHES (`floor::Patches`): count, then byte, keep, bits
FOG_RAW = $C61A       ; layout::FOG_RAW: the painted bytes as drawn (byte 3i for patch i)
R_HAZE  = $C640 + 82  ; asm/spans.S R_HAZE: the line's haze byte (`floor::haze_byte`)
SAVE_MID  = $C7D4
SAVE_BANK = $C7D5
FETCH   = $91
RECORD  = 12
COLS    = 39          ; floor::DRAWN (layout::DRAWN_COLS): the bytes a line draws

; The REU address for (VH, UH) with the bank already written by `lookup`: mid if it changed, then lo,
; unless neither changed and UH is the last fetch's lo + 1 (the counter's own increment; never across
; $FF, where it carries into mid). SHLO is $FF at every run's start (floor_line, pdone, mend), so the
; skip trusts only an increment made inside the run: between runs `lookup`'s bank write reloads lo from
; the last value written, and the $DF0A / $DF02 writes might. Inside a run a new bank means a new tile,
; so a new mid or UH = 0 (steps here are under 128 texels), and lo is written whenever the bank was.
.macro ADDRESS
    lda VH
    cmp SHMID
    beq 1f
    sta $df05
    sta SHMID
    jmp 2f
1:  lda UH
    beq 2f
    clc
    sbc SHLO
    beq 3f
2:  lda UH
    sta $df04
3:  lda UH
    sta SHLO
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
    lda #$ff
    sta SHLO
    ; An odd line (its haze is floor::HAZE_ODD) reads the map's pixel-shifted copy.
    ldx #0
    lda R_HAZE
    cmp #$11            ; floor::HAZE_ODD
    bne 1f
    ldx #$c0            ; format::ODD_BANK_OFFSET
1:  stx BANKOFF
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
    ; Within a texel a byte (du_hi and dv_hi 0 or -1): the texel-run path.
    lda DUH
    clc
    adc #1
    cmp #2
    bcs plain
    lda DVH
    clc
    adc #1
    cmp #2
    bcs plain
    jmp magnified
plain:
    ; The C64 address: LINEBUF + XB (LINEBUF is page-aligned), auto-incremented by each fetch. A fetched
    ; byte is the DMA and two register writes, 4 bus cycles; the maths fits in the windows between them
    ; (the u and v steps in the DMA's ~68 cycles, the tile check in mid's ~16: docs/validation.md, "C64U
    ; I/O timing"). Each step writes mid and lo (last); `lookup` writes the bank.
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
    ; (Mid as written; lo unknown to the run fill's command-only test, so it writes lo first.)
    lda VH
    sta SHMID
    lda #$ff
    sta SHLO
    clc
    lda XB
    adc LEN
    sta XB
    jmp next

magnified:
    ; u (v) crosses a texel on a carry out of its fraction when du (dv) >= 0, on none when it is < 0 (du_hi
    ; = -1: the integer part then steps down): the branches are patched to bcs / bcc.
    lda DUH             ; 0 -> $b0 (bcs), $ff -> $90 (bcc)
    and #$20
    eor #$b0
    sta mub
    lda DVH
    and #$20
    eor #$b0
    sta mvb
    sta mvb2
    lda #$80            ; a fixed C64 address: SCRATCH ($DF03 is LINEBUF's page)
    sta $df0a
    lda #<SCRATCH
    sta $df02
    ldx XB
    ldy REM
mfetch:
    ADDRESS
    lda #FETCH
    sta $df01
    lda SCRATCH
mput:
    sta LINEBUF,x
    inx
    dey
    beq mend
    clc
    lda UL
    adc DUL
    sta UL
mub: bcs mucross
    clc
    lda VL
    adc DVL
    sta VL
mvb: bcs mvcross
    lda SCRATCH         ; the same texel
    jmp mput
mucross:
    lda UH              ; (the carry as the fraction left it: +1, or -1 with du_hi = -1)
    adc DUH
    sta UH
    lda UHH
    adc DUS
    cmp UHH
    beq 1f
    sta UHH
    sty REM
    jsr lookup
    ldy REM
1:  clc
    lda VL
    adc DVL
    sta VL
mvb2: bcs mvcross
    jmp mfetch
mvcross:
    lda VH
    adc DVH
    sta VH
    lda VHH
    adc DVS
    cmp VHH
    bne 2f
    jmp mfetch
2:  sta VHH
    sty REM
    jsr lookup
    ldy REM
    jmp mfetch
mend:
    stx XB
    lda #$ff            ; (the next run writes lo first: see ADDRESS)
    sta SHLO
    lda #0
    sta $df0a
    jmp next

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
    jsr fog_apply
    lda SHMID
    sta SAVE_MID
    lda SHBANK
    sta SAVE_BANK
    rts

; The fog line over the line buffer (`floor::apply_fog`): LINEBUF[byte] = LINEBUF[byte] & keep | bits,
; the drawn byte kept at FOG_RAW + 3i for a doubled pair's second line (`fog_second`, asm/spans.S).
.globl fog_apply
fog_apply:
    lda PATCH
    beq 2f
    sta REM
    ldy #0
1:  ldx PATCH+1,y
    lda LINEBUF,x
    sta FOG_RAW,y
    and PATCH+2,y
    ora PATCH+3,y
    sta LINEBUF,x
    iny
    iny
    iny
    dec REM
    bne 1b
2:  rts

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
3:  clc
    adc BANKOFF         ; (the odd lines' copy)
    sta BK
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

; floor_copy(A = band line 0..7): LINEBUF into STAGE column by column (+8 a column); a doubled pair's
; second line is copied after `fog_second`. Unrolled (9 cycles a byte); it lives in the clipping overlay
; (window 3, $F010), which is loaded before the band loop and not overwritten until the next frame.
.section .text.clip.floor_copy,"ax",@progbits
.globl floor_copy
.type floor_copy,@function
floor_copy:
    tax
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
