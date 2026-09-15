/* sonde_demod.c -- see sonde_demod.h. Pure C, no pico-sdk, host-testable.
 *
 * Bit-cell PLL lifted (same algorithm, re-parameterised) from the proven
 * "DireWolf-style" recovery already validated on real APRS traffic in this
 * project -- see aprs_rx.c's per-sample slicer loop. */

#include "sonde_demod.h"

#include <string.h>

#define SONDE_SLICE_MIN 8   /* floor for the dead-band threshold on silence */

void sonde_demod_init(sonde_demod_t *d, int sample_rate_hz, int baud)
{
    memset(d, 0, sizeof *d);
    d->pll_step = (uint32_t)(((uint64_t)1 << 32) * (uint32_t)baud /
                             (uint32_t)sample_rate_hz);
    d->inited = true;
}

bool sonde_demod_sample(sonde_demod_t *d, int32_t sample, uint8_t *bit_out)
{
    /* Light 1-pole smoothing -- just enough to knock down sample-level ADC
     * noise without smearing transitions at the higher sonde baud rates
     * (a heavier filter, fine for SARSAT's 400 bps, would eat into the bit
     * period at 4800+ baud). */
    d->lp += (sample - d->lp) >> 2;

    /* Adaptive hi/lo envelope -> centre removes DC offset / discriminator
     * bias (tuning error) automatically, same trick as aprs_rx.c. */
    if (d->lp > d->lp_hi) d->lp_hi += (d->lp - d->lp_hi) >> 2;
    else                  d->lp_hi -= (d->lp_hi - d->lp_lo) >> 10;
    if (d->lp < d->lp_lo) d->lp_lo += (d->lp - d->lp_lo) >> 2;
    else                  d->lp_lo += (d->lp_hi - d->lp_lo) >> 10;

    int32_t lpc = d->lp - ((d->lp_hi + d->lp_lo) >> 1);

    /* Slow envelope of |signal| -> adaptive dead-band threshold, more
     * noise-robust than a fixed slice level. */
    int32_t mag = (lpc < 0) ? -lpc : lpc;
    d->env += (mag - d->env) >> 6;
    int32_t th = d->env >> 2;
    if (th < SONDE_SLICE_MIN)
        th = SONDE_SLICE_MIN;

    if (lpc > th)
        d->bit = 0;
    else if (lpc < -th)
        d->bit = 1;
    /* else: inside the dead band -- hold the previous slice */

    bool emitted = false;
    int32_t prev_sign = (int32_t)d->pll >> 31;
    d->pll += d->pll_step;
    if (((int32_t)d->pll >> 31) < prev_sign) {
        *bit_out = (uint8_t)d->bit;
        emitted = true;
    }
    if (d->bit != d->pval) {
        d->pll -= (uint32_t)((int32_t)d->pll >> 2);  /* nudge phase towards the transition */
        d->pval = d->bit;
    }
    return emitted;
}
