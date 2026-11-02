; The rotor sound, once at boot (`format::AUDIO_*`, `digital`): decode the DLT5 in the REU to signed 8-bit
; PCM in the REU and start the C64U's Ultimate Audio sampler on it. From then on the sampler plays it by
; itself: frame 0 to the loop end, then the loop forever (its repeat registers), and a second voice 160
; samples (10 ms) behind on the other side: H.A.V.O.C.'s Haas trick, one stream for both ears.
;
; The decode rule and the unrolled 8-sample group come from ../sueme64 target-c64/asm/digital.S @ 8685892
; (`digital_decode_run`'s mono path; see THIRD-PARTY.md): per 3-bit code, pred = pred +sat book[code],
; published (pred + 128) >> 8. The rotor is whole 8-frame groups, so every segment starts on a byte and
; only the aligned path is needed. `digital::decode` is the reference (tools/test_audio.py).
;
; Runs from `.audiotext` at $C000 (target-c64/link.ld: loaded with the PRG, the floor's scratch after the
; boot), with scratch in the frame buffer ($7000, free until video_init) and zero page $A0-$CF.
; Returns A = 1 when the sampler was found and started, 0 otherwise (as in tools/emu.py).

REU_CMD  = $DF01
REU_C64  = $DF02
REU_ADDR = $DF04
REU_LEN  = $DF07
REU_CTRL = $DF0A
STASH    = $90
FETCH    = $91

SAMP_L   = $DF20          ; voice registers: +0 control, +1 volume (reads the version), +2 pan, +4 REU,
SAMP_R   = $DF40          ; +5 start, +9 length, +14 divider, +17 repeat start, +21 repeat end (big-endian)
SAMP_ACK = $DF3F
VERSION  = $10
DIVIDER  = 390            ; format::AUDIO_DIVIDER: 6.25 MHz / 16 kHz
HAAS     = 160            ; format::HAAS
PCM      = $C00000        ; format::AUDIO_PCM
BOOKS    = $600000 + 32   ; format::AUDIO_DLT + the DLT5 header
HEADER   = $560000 + 16   ; format::HEADER + H_AUDIO_BYTES
START_PAGES = 64          ; start the sampler once 64 pages (1 s) are decoded

SBUF     = $7000          ; 255 bytes of code stream (85 groups)
PCMBUF   = $7100          ; a page of PCM
BOOK     = $7200          ; this segment's 8 deltas
HDR      = $7210          ; the package header from H_AUDIO_BYTES (format::H_*)
LOOPA    = HDR + 4
LOOPB    = HDR + 8
SEGG     = HDR + 12
SEGLONG  = HDR + 14
STREAM   = HDR + 16
GROUPS   = HDR + 20

SRC      = $A0            ; (2) the next group in SBUF
OUT      = $A2            ; (2) the next sample in PCMBUF
PRED     = $A4            ; (2) the predictor
B0       = $A6            ; the group's three bytes
B1       = $A7
B2       = $A8
T        = $A9
SLEFT    = $AA            ; groups left in SBUF
GLEFT    = $AB            ; (2) groups left in the segment
SEG      = $AD            ; (2) the segment
GTOT     = $AF            ; (3) groups left in all
SRCR     = $B2            ; (3) the stream's next REU address
PCMR     = $B5            ; (3) the next PCM page's REU address
BOOKR    = $B8            ; (3) the next book's REU address
PAGES    = $BB            ; pages written, to START_PAGES
STARTED  = $BC
RA       = $BD            ; (3) DMA: REU address
CA       = $C0            ; (2) C64 address
LEN      = $C2            ; (2) length, 1..256
CMD      = $C4
FIRST    = $C5
LEN0     = $C6            ; (2)

; One sample: Y = 2 * code. pred +sat= book[code]; A = (pred + 128) >> 8, at most 127.
.macro SAMPLE name
    clc
    lda BOOK,y
    adc PRED
    sta PRED
    iny
    lda BOOK,y
    adc PRED+1
    bvc .Ls_\name
    bmi .Lp_\name
    lda #0
    sta PRED
    lda #128
    bne .Ls_\name
.Lp_\name:
    lda #255
    sta PRED
    lda #127
.Ls_\name:
    sta PRED+1
    lda PRED
    cmp #128
    lda PRED+1
    adc #0
    bvc .Lr_\name
    lda #127
.Lr_\name:
.endm

; \dst (big-endian u24) = \src (little-endian u24) + \add
.macro PUT24 dst, src, add
    clc
    lda \src
    adc #<(\add)
    sta \dst+2
    lda \src+1
    adc #>(\add)
    sta \dst+1
    lda \src+2
    adc #0
    sta \dst
.endm

; \v (u24) += \add (< 65536)
.macro ADD24 v, add
    clc
    lda \v
    adc #<(\add)
    sta \v
    lda \v+1
    adc #>(\add)
    sta \v+1
    bcc .La_\@
    inc \v+2
.La_\@:
.endm

.macro SET24 v, value
    lda #<(\value)
    sta \v
    lda #>(\value)
    sta \v+1
    lda #((\value) >> 16)
    sta \v+2
.endm

.section .text.audio_boot,"ax",@progbits
.globl audio_boot
.type audio_boot,@function
audio_boot:
    lda SAMP_L+1
    cmp #VERSION
    beq 1f
    lda #0
    rts
1:  jsr sampler_stop
    ; the header's audio fields
    SET24 RA, HEADER
    lda #<HDR
    ldx #>HDR
    ldy #24
    jsr fetch
    ; the right voice's 160 zero bytes before the PCM
    ldx #0
    txa
2:  sta PCMBUF,x
    inx
    bne 2b
    SET24 RA, PCM - HAAS
    lda #<PCMBUF
    ldx #>PCMBUF
    ldy #HAAS
    jsr stash
    ; walk the segments
    lda STREAM
    sta SRCR
    lda STREAM+1
    sta SRCR+1
    lda STREAM+2
    sta SRCR+2
    lda GROUPS
    sta GTOT
    lda GROUPS+1
    sta GTOT+1
    lda GROUPS+2
    sta GTOT+2
    SET24 PCMR, PCM
    SET24 BOOKR, BOOKS
    lda #0
    sta SEG
    sta SEG+1
    sta PRED
    sta PRED+1
    sta SLEFT
    sta PAGES
    sta STARTED
    lda #<PCMBUF
    sta OUT
    lda #>PCMBUF
    sta OUT+1
segment:
    lda GTOT
    ora GTOT+1
    ora GTOT+2
    bne 1f
    jmp done
1:  ; this segment's book
    lda BOOKR
    sta RA
    lda BOOKR+1
    sta RA+1
    lda BOOKR+2
    sta RA+2
    lda #<BOOK
    ldx #>BOOK
    ldy #16
    jsr fetch
    ADD24 BOOKR, 16
    ; its groups: SEGG, one more while SEG < SEGLONG
    lda SEGG
    sta GLEFT
    lda SEGG+1
    sta GLEFT+1
    lda SEG
    cmp SEGLONG
    lda SEG+1
    sbc SEGLONG+1
    bcs 2f
    inc GLEFT
    bne 2f
    inc GLEFT+1
2:  inc SEG
    bne group
    inc SEG+1
group:
    lda GLEFT
    ora GLEFT+1
    beq segment
    lda GTOT
    ora GTOT+1
    ora GTOT+2
    bne 1f
    jmp done
1:  lda SLEFT
    bne 2f
    ; the next 255 bytes of stream (85 groups)
    lda SRCR
    sta RA
    lda SRCR+1
    sta RA+1
    lda SRCR+2
    sta RA+2
    lda #<SBUF
    ldx #>SBUF
    ldy #255
    jsr fetch
    ADD24 SRCR, 255
    lda #<SBUF
    sta SRC
    lda #>SBUF
    sta SRC+1
    lda #85
    sta SLEFT
2:  jsr decode8
    dec SLEFT
    lda GLEFT
    bne 3f
    dec GLEFT+1
3:  dec GLEFT
    lda GTOT
    bne 4f
    lda GTOT+1
    bne 5f
    dec GTOT+2
5:  dec GTOT+1
4:  dec GTOT
    ; a full page goes to the REU
    lda OUT
    bne group
    jsr flush
    lda STARTED
    bne group
    lda PAGES
    cmp #START_PAGES
    bcc group
    jsr sampler_start
    jmp group

done:
    lda OUT
    beq 1f
    jsr flush
1:  lda STARTED
    bne 2f
    jsr sampler_start
2:  lda #1
    rts

; PCMBUF up to OUT (a page when OUT has wrapped) to PCMR; PCMR += a page.
flush:
    lda PCMR
    sta RA
    lda PCMR+1
    sta RA+1
    lda PCMR+2
    sta RA+2
    lda #<PCMBUF
    sta CA
    lda #>PCMBUF
    sta CA+1
    lda OUT
    sta LEN
    lda #0
    sta LEN+1
    lda OUT
    bne 1f
    inc LEN+1           ; a whole page
1:  lda #STASH
    jsr dma
    inc PCMR+1
    bne 2f
    inc PCMR+2
2:  inc PAGES
    bne 3f
    dec PAGES           ; (only counted up to START_PAGES)
3:  lda #<PCMBUF
    sta OUT
    rts

; Three code bytes at SRC -> eight samples at OUT (sueme64's aligned mono group).
decode8:
    ldy #0
    lda (SRC),y
    sta B0
    iny
    lda (SRC),y
    sta B1
    iny
    lda (SRC),y
    sta B2
    clc
    lda SRC
    adc #3
    sta SRC
    bcc 1f
    inc SRC+1
1:  ; code 0, bit 0
    lda B0
    asl
    and #14
    tay
    SAMPLE c0
    ldy #0
    sta (OUT),y
    ; code 1, bit 3
    lda B0
    lsr
    lsr
    and #14
    tay
    SAMPLE c1
    ldy #1
    sta (OUT),y
    ; code 2, bit 6
    lda B0
    lsr
    lsr
    lsr
    lsr
    lsr
    and #6
    sta T
    lda B1
    asl
    asl
    asl
    and #8
    ora T
    tay
    SAMPLE c2
    ldy #2
    sta (OUT),y
    ; code 3, bit 9
    lda B1
    and #14
    tay
    SAMPLE c3
    ldy #3
    sta (OUT),y
    ; code 4, bit 12
    lda B1
    lsr
    lsr
    lsr
    and #14
    tay
    SAMPLE c4
    ldy #4
    sta (OUT),y
    ; code 5, bit 15
    lda B1
    lsr
    lsr
    lsr
    lsr
    lsr
    lsr
    and #2
    sta T
    lda B2
    asl
    asl
    and #12
    ora T
    tay
    SAMPLE c5
    ldy #5
    sta (OUT),y
    ; code 6, bit 18
    lda B2
    lsr
    and #14
    tay
    SAMPLE c6
    ldy #6
    sta (OUT),y
    ; code 7, bit 21
    lda B2
    lsr
    lsr
    lsr
    lsr
    and #14
    tay
    SAMPLE c7
    ldy #7
    sta (OUT),y
    clc
    lda OUT
    adc #8
    sta OUT
    rts

; REU -> A:X (C64), Y bytes (1..255), from RA
fetch:
    sta CA
    stx CA+1
    sty LEN
    lda #0
    sta LEN+1
    lda #FETCH
    jmp dma

; A:X (C64) -> REU at RA, Y bytes (1..255)
stash:
    sta CA
    stx CA+1
    sty LEN
    lda #0
    sta LEN+1
    lda #STASH
    ; fall through

; One transfer of LEN (1..256) bytes, command A; split in two where it would cross a 512 KiB boundary
; (the C64U corrupts such a DMA).
dma:
    sta CMD
    lda RA+2
    and #7
    cmp #7
    bne dma1
    lda RA+1
    cmp #$ff
    bne dma1
    sec
    lda #0
    sbc RA
    beq dma1            ; at a page start: a page or less cannot cross
    sta FIRST           ; bytes to the boundary
    lda LEN+1
    bne 1f
    lda FIRST
    cmp LEN
    bcs dma1            ; fits
1:  lda LEN
    sta LEN0
    lda LEN+1
    sta LEN0+1
    lda FIRST
    sta LEN
    lda #0
    sta LEN+1
    jsr dma1
    clc
    lda RA
    adc FIRST
    sta RA
    lda RA+1
    adc #0
    sta RA+1
    lda RA+2
    adc #0
    sta RA+2
    clc
    lda CA
    adc FIRST
    sta CA
    lda CA+1
    adc #0
    sta CA+1
    sec
    lda LEN0
    sbc FIRST
    sta LEN
    lda LEN0+1
    sbc #0
    sta LEN+1
dma1:
    lda CA
    sta REU_C64
    lda CA+1
    sta REU_C64+1
    lda RA
    sta REU_ADDR
    lda RA+1
    sta REU_ADDR+1
    lda RA+2
    sta REU_ADDR+2
    lda LEN
    sta REU_LEN
    lda LEN+1
    sta REU_LEN+1
    lda #0
    sta REU_CTRL
    lda CMD
    sta REU_CMD
    rts

sampler_stop:
    lda #0
    sta SAMP_L+1
    sta SAMP_R+1
    sta SAMP_L
    sta SAMP_R
    lda #$ff
    sta SAMP_ACK
    rts

; Both voices on the PCM: left from PCM, right from PCM - HAAS (the zero bytes), each to the loop end
; then repeating from the loop start (secondreality / sueme64 `configure`: the length is one more than
; the repeat end, which the sampler tests first). Gated together, so they share the sampler's clock.
sampler_start:
    lda #0
    sta SAMP_L+2        ; pan left
    lda #15
    sta SAMP_R+2        ; pan right
    lda #1
    sta SAMP_L+4        ; from the REU
    sta SAMP_R+4
    lda #(PCM >> 16)
    sta SAMP_L+5
    lda #>PCM
    sta SAMP_L+6
    lda #<PCM
    sta SAMP_L+7
    lda #((PCM - HAAS) >> 16)
    sta SAMP_R+5
    lda #>(PCM - HAAS)
    sta SAMP_R+6
    lda #<(PCM - HAAS)
    sta SAMP_R+7
    PUT24 SAMP_L+9, LOOPB, 1
    PUT24 SAMP_R+9, LOOPB, HAAS + 1
    PUT24 SAMP_L+17, LOOPA, 0
    PUT24 SAMP_R+17, LOOPA, HAAS
    PUT24 SAMP_L+21, LOOPB, 0
    PUT24 SAMP_R+21, LOOPB, HAAS
    lda #>DIVIDER
    sta SAMP_L+14
    sta SAMP_R+14
    lda #<DIVIDER
    sta SAMP_L+15
    sta SAMP_R+15
    lda #$ff
    sta SAMP_ACK
    lda #32             ; full volume
    sta SAMP_L+1
    sta SAMP_R+1
    lda #$03            ; 8-bit, repeat, on
    sta SAMP_L
    sta SAMP_R
    lda #1
    sta STARTED
    rts
.size audio_boot, .-audio_boot
