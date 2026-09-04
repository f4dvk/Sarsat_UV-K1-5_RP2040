/*
 * dec406_2g_stub.c — the RP2040 firmware decodes 1st-generation (FGB) beacons
 * only. 2nd-generation (SGB / T.018 / DSSS-OQPSK) demodulation is not feasible
 * from FM-discriminator audio on this MCU (see project plan / README).
 *
 * The full, MCU-portable 2G frame parser + BCH(250,202) is kept unbuilt under
 * vendor/decode_sarsat/dec406_v2g.c for a possible future IQ front-end. These
 * stubs satisfy the dec406.c dispatcher without pulling that code in.
 */
#include "dec406.h"
#include <stdio.h>

void decode_2g(const uint8_t *bits)
{
    (void)bits;
    printf("2G (SGB) frame received but 2G decoding is not built on this target\n");
}

void decode_2g_set_frame_mode(int is_self_test)
{
    (void)is_self_test;
}
