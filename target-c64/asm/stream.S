; The rotor sound, streamed (`format::AUDIO_*`, `digital`): the DLT5 in the REU decoded to signed 8-bit PCM
; into a ring exactly the loop's length (L = loop end - loop start) at `AUDIO_PCM`, sample n at n mod L,
; while the C64U's Ultimate Audio sampler plays the ring round and round. The boot fills the ring ahead
; and starts the sampler (`stream_init`, `stream_run`, `stream_start`); then the main loop runs a few
; pages a frame, kept behind the playback by the frame clock (target-c64/src/main.rs), until the stream
; ends. The last L samples written are the loop, each at the slot the wrapping playback reads it from:
; the loop plays on with no seam and the decoder is never run again. A second voice 160 samples (10 ms)
; behind on the other side: H.A.V.O.C.'s Haas trick, one stream for both ears.
;
; The decode rule and the unrolled 8-sample group come from ../sueme64 target-c64/asm/digital.S @ 8685892
; (`digital_decode_run`'s mono path; see THIRD-PARTY.md): per 3-bit code, pred = pred +sat book[code],
; published (pred + 128) >> 8. The rotor is whole 8-frame groups (and whole 32-group pages), so every
; segment starts on a byte and only the aligned path is needed. `digital::decode` is the reference
; (tools/test_audio.py).
;
; An REU overlay (`.text.stream`, run at $E000 like the others: fetched at boot and while streaming),
; its page buffers above it in the spent window ($F100-$F327: refilled every call), its state (with the
; segment's book) resident (`.bss`), zero page
; $A0-$CF only inside a call.

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
HAAS     = 160            ; format::HAAS
PCM      = $C00000        ; format::AUDIO_PCM
BOOKS    = $600000 + 32   ; format::AUDIO_DLT + the DLT5 header
HEADER   = $560000 + 16   ; format::HEADER + H_AUDIO_BYTES
BUDGET   = 8              ; pages a call at most (format::STREAM_BUDGET)

PCMBUF   = $F100          ; a page of PCM (page-aligned: OUT wraps at its end)
SBUF     = $F200          ; 255 bytes of code stream (85 groups)
HDR      = $F310          ; the package header from H_AUDIO_BYTES (format::H_*), at init
LOOPA    = HDR + 4
LOOPB    = HDR + 8
H_SEGG   = HDR + 12
H_SEGLONG = HDR + 14
STREAM   = HDR + 16
GROUPS   = HDR + 20

SRC      = $A0            ; (2) the next group in SBUF
OUT      = $A2            ; (2) the next sample in PCMBUF
PRED     = $A4            ; (2) the predictor (kept in ST_PRED between calls)
B0       = $A6            ; the group's three bytes
B1       = $A7
B2       = $A8
T        = $A9
SLEFT    = $AA            ; groups left in SBUF
LIMIT    = $AB            ; (2) the pages written to stop at
PLEFT    = $AD            ; pages left this call
FIRST2   = $AE            ; bytes to the ring's end (a split page)
M0       = $B0            ; (3) 11 F
M1       = $B1
M2       = $B2
N0       = $B3            ; (3) 2 F, 8 F
N1       = $B4
N2       = $B5
RA       = $BD            ; (3) DMA: REU address
CA       = $C0            ; (2) C64 address
LEN      = $C2            ; (2) length, 1..256
CMD      = $C4
FIRST    = $C5
LEN0     = $C6            ; (2)

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


; The decoder's state between calls (resident).
.section .bss.stream_state,"aw",@nobits
.globl stream_state
stream_state:
ST_DONE:    .space 1      ; the stream has ended (or there is no sampler)
ST_PRED:    .space 2
SRCR:       .space 3      ; the stream's next group's REU address
BOOKR:      .space 3      ; the next book's REU address
SEG:        .space 2      ; segments begun
GLEFT:      .space 2      ; groups left in the segment
GTOT:       .space 3      ; groups left in all
SEGG:       .space 2      ; groups in a short segment
SEGLONG:    .space 2      ; the first SEGLONG segments are a group longer
RING:       .space 3      ; L, the ring's length (the loop's)
PCMR:       .space 3      ; the next page's offset in the ring
WRITTEN:    .space 2      ; pages written
AHEAD:      .space 2      ; L / 2 >> 8
BOOK:       .space 16     ; this segment's 8 deltas (kept: a segment spans calls, and the window is
                          ; another overlay's between them)
CALIB:      .space 6      ; the boot's (main.rs): CIA ticks for 12,288 samples at divider 390 (u32), the divider set (u16)

.section .text.stream,"ax",@progbits

; stream_init() -> A: 1 with the sampler found (stopped, the Haas bytes zeroed, the state at the stream's
; start), 0 without (the stream is done before it began).
.globl stream_init
.type stream_init,@function
stream_init:
    lda SAMP_L+1
    cmp #VERSION
    beq 1f
    lda #1
    sta ST_DONE
    lda #0
    rts
1:  jsr sampler_stop
    ; the header's audio fields
    SET24 RA, HEADER
    lda #<HDR
    ldx #>HDR
    ldy #24
    jsr fetch
    ; the right voice's 160 zero bytes before the ring
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
    ldx #2
3:  lda STREAM,x
    sta SRCR,x
    lda GROUPS,x
    sta GTOT,x
    lda #0
    sta PCMR,x
    dex
    bpl 3b
    SET24 BOOKR, BOOKS
    lda H_SEGG
    sta SEGG
    lda H_SEGG+1
    sta SEGG+1
    lda H_SEGLONG
    sta SEGLONG
    lda H_SEGLONG+1
    sta SEGLONG+1
    ; L = loop end - loop start
    sec
    lda LOOPB
    sbc LOOPA
    sta RING
    lda LOOPB+1
    sbc LOOPA+1
    sta RING+1
    lda LOOPB+2
    sbc LOOPA+2
    sta RING+2
    ; AHEAD = L / 2 >> 8: the pages to write before the playback starts. Half a lap ahead of the
    ; playback and half a lap short of it round the ring: the frame clock's estimate of the sampler's
    ; place (no readback) may be out by ~16% over the whole stream either way.
    lda RING+2
    lsr
    sta AHEAD+1
    lda RING+1
    ror
    sta AHEAD
    lda #0
    sta ST_DONE
    sta ST_PRED
    sta ST_PRED+1
    sta SEG
    sta SEG+1
    sta GLEFT
    sta GLEFT+1
    sta WRITTEN
    sta WRITTEN+1
    lda #1
    rts
.size stream_init, .-stream_init

; stream_run(A:X = F, NTSC frames since the sampler started) -> A: 0 with BUDGET pages written (call
; again), 1 half a lap ahead of the playback, 2 at the stream's end (now and from then on).
; The pages to have written: LIMIT = F + (11 F >> 8) + AHEAD, i.e. (F · 267 + L / 2) >> 8 or a page
; less (267 samples a frame: format::SAMPLES_PER_TICK, a little over the calibrated 16,000 Hz / 60).
.globl stream_run
.type stream_run,@function
stream_run:
    sta LIMIT
    stx LIMIT+1
    ; M = 11 F = F + 2F + 8F (24 bits)
    sta M0
    stx M1
    lda #0
    sta M2
    lda LIMIT
    asl
    sta N0
    lda LIMIT+1
    rol
    sta N1
    lda #0
    rol
    sta N2
    jsr madd            ; + 2F
    asl N0
    rol N1
    rol N2
    asl N0
    rol N1
    rol N2
    jsr madd            ; + 8F
    clc
    lda LIMIT
    adc M1
    sta LIMIT
    lda LIMIT+1
    adc M2
    sta LIMIT+1
    bcs 0f
    clc
    lda LIMIT
    adc AHEAD
    sta LIMIT
    lda LIMIT+1
    adc AHEAD+1
    sta LIMIT+1
    bcc 1f
0:  lda #$ff            ; (past 64 Ki pages: no limit)
    sta LIMIT
    sta LIMIT+1
1:  lda ST_DONE
    beq 1f
    lda #2
    rts
1:  lda ST_PRED
    sta PRED
    lda ST_PRED+1
    sta PRED+1
    lda #BUDGET
    sta PLEFT
    lda #0
    sta SLEFT           ; fetch the stream afresh from SRCR
page:
    lda GTOT
    ora GTOT+1
    ora GTOT+2
    bne 1f
    lda #1
    sta ST_DONE
    lda #2
    jmp leave
1:  lda WRITTEN
    cmp LIMIT
    lda WRITTEN+1
    sbc LIMIT+1
    bcc 2f
    lda #1
    jmp leave
2:  lda PLEFT
    bne 3f
    lda #0
    jmp leave
3:  lda #<PCMBUF
    sta OUT
    lda #>PCMBUF
    sta OUT+1
group:
    lda GLEFT
    ora GLEFT+1
    bne 4f
    jsr segment
4:  lda SLEFT
    bne 5f
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
    lda #<SBUF
    sta SRC
    lda #>SBUF
    sta SRC+1
    lda #85
    sta SLEFT
5:  jsr decode8
    dec SLEFT
    ADD24 SRCR, 3
    lda GLEFT
    bne 6f
    dec GLEFT+1
6:  dec GLEFT
    lda GTOT
    bne 7f
    lda GTOT+1
    bne 8f
    dec GTOT+2
8:  dec GTOT+1
7:  dec GTOT
    ; 32 groups make the page (the sound is whole pages)
    lda OUT
    bne group
    jsr flush
    inc WRITTEN
    bne 9f
    inc WRITTEN+1
9:  dec PLEFT
    jmp page
leave:
    tax
    lda PRED
    sta ST_PRED
    lda PRED+1
    sta ST_PRED+1
    txa
    rts

; The next segment: its book, its groups (SEGG, one more while SEG < SEGLONG).
segment:
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
    lda SEGG
    sta GLEFT
    lda SEGG+1
    sta GLEFT+1
    lda SEG
    cmp SEGLONG
    lda SEG+1
    sbc SEGLONG+1
    bcs 1f
    inc GLEFT
    bne 1f
    inc GLEFT+1
1:  inc SEG
    bne 2f
    inc SEG+1
2:  rts

; PCMBUF (a page) into the ring at PCMR, split at the ring's end; PCMR a page on, round the ring.
flush:
    clc
    lda PCMR
    adc #<PCM
    sta RA
    lda PCMR+1
    adc #>PCM
    sta RA+1
    lda PCMR+2
    adc #(PCM >> 16)
    sta RA+2
    lda #<PCMBUF
    sta CA
    lda #>PCMBUF
    sta CA+1
    ; to the ring's end: RING - PCMR = (A:X:FIRST2)
    sec
    lda RING
    sbc PCMR
    sta FIRST2
    lda RING+1
    sbc PCMR+1
    tax
    lda RING+2
    sbc PCMR+2
    bne whole           ; 64 KiB or more
    cpx #2
    bcs whole           ; 512 or more
    cpx #1
    bne split           ; under a page: FIRST2 = 1..255
    lda FIRST2
    bne whole           ; 257..511
    ; exactly a page to the end: round to 0
    jsr page_out
    lda #0
    sta PCMR
    sta PCMR+1
    sta PCMR+2
    rts
whole:
    jsr page_out
    inc PCMR+1
    bne 1f
    inc PCMR+2
1:  rts
split:
    lda FIRST2
    sta LEN
    lda #0
    sta LEN+1
    lda #STASH
    jsr dma
    ; the rest from the ring's start
    SET24 RA, PCM
    lda FIRST2
    sta CA
    lda #>PCMBUF
    sta CA+1
    sec
    lda #0
    sbc FIRST2
    sta LEN
    sta PCMR
    lda #0
    sta LEN+1
    sta PCMR+1
    sta PCMR+2
    lda #STASH
    jmp dma

; M += N (24 bits)
madd:
    clc
    lda M0
    adc N0
    sta M0
    lda M1
    adc N1
    sta M1
    lda M2
    adc N2
    sta M2
    rts

page_out:
    lda #0
    sta LEN
    lda #1
    sta LEN+1
    lda #STASH
    jmp dma

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


; stream_calibrate() -> A:X: the sampler's clock divider for the rotor's 16 kHz, measured (sueme64's
; `calibrate`): the sampler's clock is not to be trusted to be 6.25 MHz. Two silent one-shots (both
; voices at volume 0, from the ring before it is filled), 4,096 and 16,384 samples at divider 390, are
; timed with CIA 2's timers chained into a 32-bit down-counter at the bus's 1.02 MHz (the CPU's turbo
; does not enter into it); their difference is 12,288 samples' worth (the start-up cancels), which at
; 16,000 Hz takes 785,454 NTSC ticks, and the time goes as the divider: divider = 390 · 785,454 / ticks,
; rounded. Out of range or never done: 390. Kept in CALIB (the ticks, the divider) for the host to read.
; (The frame clock takes CIA 2 over after.)
C_NOW    = $A0            ; (4) the counter, read
C_BEG    = $A4            ; (4) at the one-shot's start
C_EL     = $A8            ; (4) ticks elapsed; then the 12,288 samples' ticks (the divisor)
C_SH     = $AC            ; (4) the short one-shot's ticks
C_FR     = $B0            ; (3) frames to play
C_DV     = $B3            ; (4) the dividend, then the quotient
C_R      = $B7            ; (4) the remainder
C_Y      = $BB            ; (3) a trial remainder
TARGET   = 390 * 785454   ; divider 390's ticks for 12,288 samples at 16 kHz, times 390

.globl stream_calibrate
.type stream_calibrate,@function
stream_calibrate:
    lda #$ff
    sta $DD04
    sta $DD05
    sta $DD06
    sta $DD07
    lda #$51            ; B: start, force load, count A's underflows
    sta $DD0F
    lda #$11            ; A: start, force load, continuous
    sta $DD0E
    lda #0
    sta C_FR
    sta C_FR+2
    lda #>4096
    sta C_FR+1
    jsr one_shot
    ldx #3
1:  lda C_EL,x
    sta C_SH,x
    dex
    bpl 1b
    lda #>16384
    sta C_FR+1
    jsr one_shot
    ; either never done: 390
    lda C_SH
    ora C_SH+1
    ora C_SH+2
    ora C_SH+3
    beq fb
    lda C_EL
    ora C_EL+1
    ora C_EL+2
    ora C_EL+3
    beq fb
    ; C_EL = long - short
    sec
    ldx #0
    ldy #4
2:  lda C_EL,x
    sbc C_SH,x
    sta C_EL,x
    inx
    dey
    bne 2b
    bcc fb        ; shorter than the short one
    ; 250,000 <= ticks < 1,100,000 ($03D090 .. $10C8E0)
    lda C_EL+3
    bne fb
    lda C_EL
    cmp #<250000
    lda C_EL+1
    sbc #>250000
    lda C_EL+2
    sbc #(250000 >> 16)
    bcc fb
    lda C_EL
    cmp #<1100000
    lda C_EL+1
    sbc #>1100000
    lda C_EL+2
    sbc #(1100000 >> 16)
    bcc divide
fb: jmp fallback
divide:
    ; C_DV = TARGET + ticks / 2
    lda C_EL+2
    lsr
    sta C_Y+2
    lda C_EL+1
    ror
    sta C_Y+1
    lda C_EL
    ror
    clc
    adc #<TARGET
    sta C_DV
    lda C_Y+1
    adc #>TARGET
    sta C_DV+1
    lda C_Y+2
    adc #((TARGET >> 16) & $ff)
    sta C_DV+2
    lda #0
    adc #(TARGET >> 24)
    sta C_DV+3
    ; C_DV /= C_EL (shift and subtract; the quotient in C_DV)
    lda #0
    sta C_R
    sta C_R+1
    sta C_R+2
    sta C_R+3
    ldx #32
3:  asl C_DV
    rol C_DV+1
    rol C_DV+2
    rol C_DV+3
    rol C_R
    rol C_R+1
    rol C_R+2
    rol C_R+3
    sec
    lda C_R
    sbc C_EL
    sta C_Y
    lda C_R+1
    sbc C_EL+1
    sta C_Y+1
    lda C_R+2
    sbc C_EL+2
    sta C_Y+2
    lda C_R+3
    sbc C_EL+3
    bcc 4f
    sta C_R+3
    lda C_Y+2
    sta C_R+2
    lda C_Y+1
    sta C_R+1
    lda C_Y
    sta C_R
    inc C_DV
4:  dex
    bne 3b
    lda C_DV
    ldx C_DV+1
    jmp keep
fallback:
    lda #<390
    ldx #>390
keep:
    sta CALIB+4
    stx CALIB+5
    ldy #3
5:  pha
    lda C_EL,y
    sta CALIB,y
    pla
    dey
    bpl 5b
    rts
.size stream_calibrate, .-stream_calibrate

; Both voices once over C_FR samples at divider 390, silent; C_EL = the CIA ticks until the left says it
; is done (it reads nonzero, its completion flag on), or 0 after 8 s.
one_shot:
    ldx #$20
1:  lda #0
    sta SAMP_L,x
    sta SAMP_L+1,x
    sta SAMP_L+2,x
    sta SAMP_L+17,x
    sta SAMP_L+18,x
    sta SAMP_L+19,x
    sta SAMP_L+21,x
    sta SAMP_L+22,x
    sta SAMP_L+23,x
    lda #1
    sta SAMP_L+4,x
    lda #(PCM >> 16)
    sta SAMP_L+5,x
    lda #>PCM
    sta SAMP_L+6,x
    lda #<PCM
    sta SAMP_L+7,x
    lda C_FR+2
    sta SAMP_L+9,x
    lda C_FR+1
    sta SAMP_L+10,x
    lda C_FR
    sta SAMP_L+11,x
    lda #>390
    sta SAMP_L+14,x
    lda #<390
    sta SAMP_L+15,x
    txa
    sec
    sbc #$20
    tax
    bcs 1b
    lda #$ff
    sta SAMP_ACK
    jsr cia_read
    ldx #3
2:  lda C_NOW,x
    sta C_BEG,x
    dex
    bpl 2b
    lda #5              ; left: on, its completion flag on
    sta SAMP_L
    lda #1
    sta SAMP_R
wait:
    jsr cia_read
    ; C_EL = C_BEG - C_NOW (it counts down)
    sec
    ldx #0
    ldy #4
3:  lda C_BEG,x
    sbc C_NOW,x
    sta C_EL,x
    inx
    dey
    bne 3b
    ; 8 s ($7A1200 ticks): give up
    lda C_EL+3
    bne late
    lda C_EL+2
    cmp #$7A
    bcs late
    lda SAMP_L
    beq wait
    bne mute
late:
    lda #0
    sta C_EL
    sta C_EL+1
    sta C_EL+2
    sta C_EL+3
mute:
    lda #0
    sta SAMP_L+1
    sta SAMP_L
    sta SAMP_R+1
    sta SAMP_R
    lda #$ff
    sta SAMP_ACK
    rts

; C_NOW = CIA 2's chained timers (B:A), B read either side of A until it holds still.
cia_read:
    lda $DD07
    sta C_NOW+3
    lda $DD06
    sta C_NOW+2
    lda $DD04
    sta C_NOW
    lda $DD05
    sta C_NOW+1
    lda $DD06
    cmp C_NOW+2
    bne cia_read
    lda $DD07
    cmp C_NOW+3
    bne cia_read
    rts

; Both voices on the ring: left from PCM, right from PCM - HAAS (the zero bytes), round it (repeat from 0
; to L, offsets from each voice's start; the length one more than the repeat end, which the sampler tests
; first: secondreality / sueme64 `configure`). Gated together, so they share the sampler's clock. A:X = the
; clock divider (the boot's measured one for 16 kHz: `stream_calibrate`).
.globl stream_start
.type stream_start,@function
stream_start:
    sta T
    stx B0
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
    PUT24 SAMP_L+9, RING, 1
    PUT24 SAMP_R+9, RING, HAAS + 1
    lda #0
    sta SAMP_L+17
    sta SAMP_L+18
    sta SAMP_L+19
    sta SAMP_R+17
    sta SAMP_R+18
    lda #HAAS
    sta SAMP_R+19
    PUT24 SAMP_L+21, RING, 0
    PUT24 SAMP_R+21, RING, HAAS
    lda B0
    sta SAMP_L+14
    sta SAMP_R+14
    lda T
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
    rts
.size stream_start, .-stream_start
