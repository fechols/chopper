## Sections at run time

- `.zp`: $0022-$0030 (15 bytes)
- `.lowtext`: $0200-$0386 (391 bytes)
- `.basic_header`: $0801-$080C (12 bytes)
- `.text`: $080D-$4B22 (17174 bytes)
- `.rodata`: $4B23-$4FE7 (1221 bytes)
- `.sparetext`: $5680-$5A1C (925 bytes)
- `.streambss`: $5A1D-$5A50 (52 bytes)
- `.spantext`: $8300-$8A0B (1804 bytes)
- `.victext`: $9B00-$9EB5 (950 bytes)
- `.bandtext`: $AF00-$B752 (2131 bytes)
- `.bandtext2`: $BB00-$BE8F (912 bytes)
- `.noinit`: $CB00-$CD01 (514 bytes)
- `.hitext`: $F800-$FEDD (1758 bytes)

## Overlays (one at a time in their window)

- window 1 $E000: `.worldtext` 6132 of 6144 bytes, 12 free
- window 1 $E000: `.world2text` 6095 of 6144 bytes, 49 free
- window 1 $E000: `.aitext` 6075 of 6144 bytes, 69 free
- window 1 $E000: `.sfxtext` 5760 of 6144 bytes, 384 free
- window 1 $E000: `.squadtext` 5643 of 6144 bytes, 501 free
- window 1 $E000: `.combattext` 5307 of 6144 bytes, 837 free
- window 1 $E000: `.fxtext` 4667 of 6144 bytes, 1477 free
- window 1 $E000: `.hudtext` 4453 of 6144 bytes, 1691 free
- window 1 $E000: `.geomtext` 4357 of 6144 bytes, 1787 free
- window 1 $E000: `.actorstext` 3214 of 6144 bytes, 2930 free
- window 1 $E000: `.world3text` 3160 of 6144 bytes, 2984 free
- window 1 $E000: `.streamtext` 2178 of 6144 bytes, 3966 free
- window 1 $E000: `.world4text` 1811 of 6144 bytes, 4333 free
- window 2 $A000: `.decktext` 3810 of 3840 bytes, 30 free
- window 2 $A000: `.cocktext` 3390 of 3840 bytes, 450 free
- window 2 $A000: `.dyntext` 3091 of 3840 bytes, 749 free
- window 2 $A000: `.combatlibtext` 2782 of 3840 bytes, 1058 free
- window 3 $F010: `.cliptext` 1572 of 2032 bytes, 460 free

## Free gaps (at least 32 bytes)

- $5A51-$5A7F: 47 bytes
- $C756-$C77F: 42 bytes
- $FF2E-$FFF9: 204 bytes

Free in gaps of 32+ bytes: 293 bytes
