## Sections at run time

- `.lowtext`: $0200-$0386 (391 bytes)
- `.basic_header`: $0801-$080C (12 bytes)
- `.text`: $080D-$4A74 (17000 bytes)
- `.rodata`: $4A75-$4F60 (1260 bytes)
- `.sparetext`: $5680-$59F4 (885 bytes)
- `.streambss`: $59F5-$5A28 (52 bytes)
- `.spantext`: $8300-$89ED (1774 bytes)
- `.victext`: $9B00-$9ECB (972 bytes)
- `.bandtext`: $AF00-$B75F (2144 bytes)
- `.bandtext2`: $BB00-$BECD (974 bytes)
- `.noinit`: $CB00-$CD04 (517 bytes)
- `.hitext`: $F800-$FEDD (1758 bytes)

## Overlays (one at a time in their window)

- window 1 $E000: `.worldtext` 6132 of 6144 bytes, 12 free
- window 1 $E000: `.world2text` 6096 of 6144 bytes, 48 free
- window 1 $E000: `.sfxtext` 6085 of 6144 bytes, 59 free
- window 1 $E000: `.aitext` 6080 of 6144 bytes, 64 free
- window 1 $E000: `.fxtext` 5945 of 6144 bytes, 199 free
- window 1 $E000: `.actorstext` 5837 of 6144 bytes, 307 free
- window 1 $E000: `.sfxengtext` 5773 of 6144 bytes, 371 free
- window 1 $E000: `.geomtext` 5684 of 6144 bytes, 460 free
- window 1 $E000: `.squadtext` 5671 of 6144 bytes, 473 free
- window 1 $E000: `.combattext` 5307 of 6144 bytes, 837 free
- window 1 $E000: `.hudtext` 4453 of 6144 bytes, 1691 free
- window 1 $E000: `.world3text` 3160 of 6144 bytes, 2984 free
- window 1 $E000: `.streamtext` 2178 of 6144 bytes, 3966 free
- window 1 $E000: `.world4text` 1695 of 6144 bytes, 4449 free
- window 2 $A000: `.decktext` 3810 of 3840 bytes, 30 free
- window 2 $A000: `.dyntext` 3529 of 3840 bytes, 311 free
- window 2 $A000: `.cocktext` 3470 of 3840 bytes, 370 free
- window 2 $A000: `.sfxlibtext` 3328 of 3840 bytes, 512 free
- window 2 $A000: `.combatlibtext` 2782 of 3840 bytes, 1058 free
- window 3 $F010: `.cliptext` 1995 of 2032 bytes, 37 free

## Free gaps (at least 32 bytes)

- $4F61-$4FFF: 159 bytes
- $5A29-$5A7F: 87 bytes
- $C300-$C3FF: 256 bytes
- $FF4D-$FFF9: 173 bytes

Free in gaps of 32+ bytes: 675 bytes
