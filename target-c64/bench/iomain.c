// C64U I/O timing benchmark driver (tools/iobench.py): turbo on, then the generated iobench.S.
// The host reads the results ($C400 + 2 * slot, u16 CIA ticks); $C1 at $C03F when done, $EE if turbo
// never engaged (the results are then at 1 MHz). Turbo probes taken at $C046 (u16), as main.c.
#include <stdint.h>
#include <stdio.h>

#define POKE(a, v) (*(volatile uint8_t *)(a) = (v))
#define PEEK(a) (*(volatile uint8_t *)(a))

extern void io_turbo_probe(void);
extern void iobench_all(void);

int main(void) {
    POKE(0xC03F, 0);
    putchar(147);
    printf("iobench: waiting for turbo...\n");
    POKE(0xD031, 0x8F); // C64U turbo registers: 64 MHz, badlines off
    uint16_t probes = 0, ticks;
    do {
        io_turbo_probe();
        ticks = PEEK(0xC044) | PEEK(0xC045) << 8;
        probes++;
        POKE(0xC046, probes);
        POKE(0xC047, probes >> 8);
    } while (ticks >= 2000 && probes < 3000);
    printf("running...\n");
    iobench_all();
    POKE(0xC03F, ticks < 2000 ? 0xC1 : 0xEE);
    printf("iobench done.\n");
    return 0;
}
