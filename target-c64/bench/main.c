// MODE 7 milestone 2 bench driver: turbo on, tables and DDA state set up, then bench.S.
// Results: u16 ticks at $C000 + 2*slot, write counts of slot 4's kernel at $C040 (lo) / $C042 (mid),
// the last turbo probe at $C044 (u16 ticks for 20,480 cycles), probes taken at $C046 (u16),
// $C1 at $C03F when done, $EE there if turbo never engaged (results then are at 1 MHz).
// It also prints the results and returns to BASIC.
#include <stdint.h>
#include <stdio.h>

#define POKE(a, v) (*(volatile uint8_t *)(a) = (v))

extern void bench_all(void);
extern void turbo_probe(void);
uint8_t ul[8], uh[8], vl[8], vh[8], sul[8], suh[8], svl[8], svh[8];
uint8_t rowlo[128], rowhi[128];

int main(void) {
    POKE(0xC03F, 0);
    putchar(147); // clear screen
    printf("mode 7 bench\n\nwaiting for turbo...\n");
    POKE(0xD031, 0x8F); // C64U turbo registers: 64 MHz, badlines off
    // Wait until turbo is really on (probe < 2,000 ticks = over 10x), up to ~60 s at 1 MHz.
    uint16_t probes = 0, ticks;
    do {
        turbo_probe();
        ticks = *(volatile uint8_t *)0xC044 | *(volatile uint8_t *)0xC045 << 8;
        probes++;
        POKE(0xC046, probes); POKE(0xC047, probes >> 8);
    } while (ticks >= 2000 && probes < 3000);
    for (uint8_t i = 0; i < 128; i++) {
        rowlo[i] = (uint8_t)((i & 1) << 7);
        rowhi[i] = (uint8_t)(0x20 + (i >> 1));
    }
    // A rotozoom at 30 degrees, 0.25 texels per hires px, walked in memory order: per cell a line moves
    // 1.73 texels in u and 1.0 in v; lines start 0.125 apart in u and 0.2165 in v.
    for (uint8_t x = 0; x < 8; x++) {
        uint16_t u = 100 * 256 - x * 32, v = 50 * 256 + x * 55;
        ul[x] = (uint8_t)u; uh[x] = (uint8_t)(u >> 8);
        vl[x] = (uint8_t)v; vh[x] = (uint8_t)(v >> 8);
        sul[x] = 0xBB; suh[x] = 1; svl[x] = 0; svh[x] = 1;
    }
    // Replica of slot 4's register-write decisions (same start state).
    uint16_t lo = 0, mid = 0;
    {
        uint16_t u[8], v[8];
        uint8_t shlo = 0, shmid = 0;
        for (uint8_t x = 0; x < 8; x++) { u[x] = ul[x] | uh[x] << 8; v[x] = vl[x] | vh[x] << 8; }
        for (uint16_t c = 0; c < 512; c++) {
            for (uint8_t x = 0; x < 8; x++) {
                u[x] += 443; v[x] += 256;
                if ((uint8_t)(u[x] >> 8) != shlo) { lo++; shlo = u[x] >> 8; }
                if ((uint8_t)(v[x] >> 8) != shmid) { mid++; shmid = v[x] >> 8; }
                if (++shlo == 0) shmid++;
            }
        }
    }
    POKE(0xC040, lo); POKE(0xC041, lo >> 8);
    POKE(0xC042, mid); POKE(0xC043, mid >> 8);
    printf("turbo on after %u probes\n", probes);
    if (ticks >= 2000) printf("no turbo: 1 mhz numbers!\n");
    printf("running...\n\n");
    bench_all();
    POKE(0xC03F, ticks < 2000 ? 0xC1 : 0xEE);

    static const char *const name[8] = {
        "empty loop          ", "dma 1b, cmd only    ", "dma 1b, lo+cmd      ", "dma 1b, lo+mid+cmd  ",
        "dma kernel, a byte  ", "gather 4 texels     ", "gather 1 texel      ", "i/o write           ",
    };
    static const uint16_t iters[8] = {4096, 4096, 4096, 4096, 4096, 1024, 2048, 4096};
    printf("test                 ticks  us each\n");
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t t = *(volatile uint8_t *)(0xC000 + 2 * i) | *(volatile uint8_t *)(0xC001 + 2 * i) << 8;
        uint32_t h = (uint32_t)t * 100 / iters[i]; // hundredths of a tick
        printf("%s %5u %4u.%02u\n", name[i], t, (unsigned)(h / 100), (unsigned)(h % 100));
    }
    printf("\nkernel writes: lo %u, mid %u\ndone.\n", lo, mid);
    return 0;
}
