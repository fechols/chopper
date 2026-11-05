## Sections at run time

- `.lowtext`: $0200-$0386 (391 bytes)
- `.basic_header`: $0801-$080C (12 bytes)
- `.text`: $080D-$4A81 (17013 bytes)
- `.rodata`: $4A82-$4F68 (1255 bytes)
- `.sparetext`: $5680-$59F4 (885 bytes)
- `.streambss`: $59F5-$5A28 (52 bytes)
- `.spantext`: $8300-$89ED (1774 bytes)
- `.victext`: $9B00-$9EC8 (969 bytes)
- `.bandtext`: $AF00-$B75F (2144 bytes)
- `.bandtext2`: $BB00-$BECD (974 bytes)
- `.noinit`: $CB00-$CD04 (517 bytes)
- `.hitext`: $F800-$FEDD (1758 bytes)

## Overlays (one at a time in their window)

- window 1 $E000: `.worldtext` 6132 of 6144 bytes, 12 free
- window 1 $E000: `.world2text` 6095 of 6144 bytes, 49 free
- window 1 $E000: `.aitext` 6075 of 6144 bytes, 69 free
- window 1 $E000: `.fxtext` 5945 of 6144 bytes, 199 free
- window 1 $E000: `.actorstext` 5837 of 6144 bytes, 307 free
- window 1 $E000: `.geomtext` 5684 of 6144 bytes, 460 free
- window 1 $E000: `.squadtext` 5643 of 6144 bytes, 501 free
- window 1 $E000: `.sfxengtext` 5538 of 6144 bytes, 606 free
- window 1 $E000: `.combattext` 5307 of 6144 bytes, 837 free
- window 1 $E000: `.hudtext` 4453 of 6144 bytes, 1691 free
- window 1 $E000: `.world3text` 3160 of 6144 bytes, 2984 free
- window 1 $E000: `.sfxtext` 3064 of 6144 bytes, 3080 free
- window 1 $E000: `.streamtext` 2178 of 6144 bytes, 3966 free
- window 1 $E000: `.world4text` 1811 of 6144 bytes, 4333 free
- window 2 $A000: `.decktext` 3810 of 3840 bytes, 30 free
- window 2 $A000: `.sfxlibtext` 3610 of 3840 bytes, 230 free
- window 2 $A000: `.dyntext` 3529 of 3840 bytes, 311 free
- window 2 $A000: `.cocktext` 3421 of 3840 bytes, 419 free
- window 2 $A000: `.combatlibtext` 2782 of 3840 bytes, 1058 free
- window 3 $F010: `.cliptext` 1995 of 2032 bytes, 37 free

## Free gaps (at least 32 bytes)

- $4F69-$4FFF: 151 bytes
- $5A29-$5A7F: 87 bytes
- $C300-$C3FF: 256 bytes
- $FF2E-$FFF9: 204 bytes

Free in gaps of 32+ bytes: 698 bytes
