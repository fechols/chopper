## Sections at run time

- `.lowtext`: $0200-$0386 (391 bytes)
- `.basic_header`: $0801-$080C (12 bytes)
- `.text`: $080D-$4AE4 (17112 bytes)
- `.rodata`: $4AE5-$4FCB (1255 bytes)
- `.sparetext`: $5680-$5A19 (922 bytes)
- `.streambss`: $5A1A-$5A4D (52 bytes)
- `.spantext`: $8300-$89ED (1774 bytes)
- `.victext`: $9B00-$9EC8 (969 bytes)
- `.bandtext`: $AF00-$B75F (2144 bytes)
- `.bandtext2`: $BB00-$BECD (974 bytes)
- `.noinit`: $CB00-$CD04 (517 bytes)
- `.hitext`: $F800-$FEDD (1758 bytes)

## Overlays (one at a time in their window)

- window 1 $E000: `.worldtext` 6132 of 6144 bytes, 12 free
- window 1 $E000: `.world2text` 6095 of 6144 bytes, 49 free
- window 1 $E000: `.fxtext` 6085 of 6144 bytes, 59 free
- window 1 $E000: `.aitext` 6075 of 6144 bytes, 69 free
- window 1 $E000: `.actorstext` 5837 of 6144 bytes, 307 free
- window 1 $E000: `.squadtext` 5643 of 6144 bytes, 501 free
- window 1 $E000: `.sfxengtext` 5538 of 6144 bytes, 606 free
- window 1 $E000: `.combattext` 5307 of 6144 bytes, 837 free
- window 1 $E000: `.geomtext` 4727 of 6144 bytes, 1417 free
- window 1 $E000: `.hudtext` 4453 of 6144 bytes, 1691 free
- window 1 $E000: `.world3text` 3160 of 6144 bytes, 2984 free
- window 1 $E000: `.sfxtext` 2898 of 6144 bytes, 3246 free
- window 1 $E000: `.streamtext` 2178 of 6144 bytes, 3966 free
- window 1 $E000: `.world4text` 1811 of 6144 bytes, 4333 free
- window 2 $A000: `.decktext` 3810 of 3840 bytes, 30 free
- window 2 $A000: `.sfxlibtext` 3610 of 3840 bytes, 230 free
- window 2 $A000: `.dyntext` 3590 of 3840 bytes, 250 free
- window 2 $A000: `.cocktext` 3421 of 3840 bytes, 419 free
- window 2 $A000: `.combatlibtext` 2782 of 3840 bytes, 1058 free
- window 3 $F010: `.cliptext` 2015 of 2032 bytes, 17 free

## Free gaps (at least 32 bytes)

- $4FCC-$4FFF: 52 bytes
- $5A4E-$5A7F: 50 bytes
- $C300-$C3FF: 256 bytes
- $C60A-$C63F: 54 bytes
- $FF2E-$FFF9: 204 bytes

Free in gaps of 32+ bytes: 616 bytes
