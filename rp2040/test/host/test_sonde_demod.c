/* test_sonde_demod -- bit-cell PLL self-consistency (see sonde_demod.h).
 *
 * Synthetic two-level signal only (Gaussian-ish pulse shaping + noise) --
 * proves the PLL algorithm recovers bits it was given, not that it will
 * cope with real radiosonde RF (no reference capture available, see the
 * project's radiosonde plan notes).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sonde_demod.h"

static int fails;

static void chk(const char *name, int cond)
{
    printf("  %-40s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

static unsigned rnd_state = 12345;
static unsigned rnd(void)
{
    rnd_state = rnd_state * 1103515245u + 12345u;
    return (rnd_state >> 16) & 0x7fff;
}

/* Run one (baud, sample_rate, noise_amplitude) combination: build a random
 * bit sequence, synthesise a shaped+noisy two-level waveform, demodulate it,
 * and check the recovered bits match (after allowing the PLL some bit
 * periods to lock, and searching a small alignment window). */
static void run_case(const char *name, int baud, int fs, int n_bits,
                     int32_t amplitude, int32_t noise_amp)
{
    uint8_t *bits_in = malloc((size_t)n_bits);
    for (int i = 0; i < n_bits; i++)
        bits_in[i] = (uint8_t)(rnd() & 1);

    double spb = (double)fs / baud;
    int n_samp = (int)(n_bits * spb) + 8;
    int32_t *samples = malloc(sizeof(int32_t) * (size_t)n_samp);

    int32_t lp_state = 0;
    for (int n = 0; n < n_samp; n++) {
        int bit_idx = (int)(n / spb);
        if (bit_idx >= n_bits) bit_idx = n_bits - 1;
        int32_t target = bits_in[bit_idx] ? -amplitude : amplitude;
        /* 1-pole smoothing to emulate GFSK's gradual (Gaussian-filtered)
         * transitions instead of an unrealistically sharp square wave. */
        lp_state += (target - lp_state) >> 1;
        int32_t noise = noise_amp ? (int32_t)(rnd() % (unsigned)(2 * noise_amp + 1)) - noise_amp : 0;
        samples[n] = lp_state + noise;
    }

    sonde_demod_t d;
    sonde_demod_init(&d, fs, baud);
    uint8_t *bits_out = malloc((size_t)n_bits + 8);
    int n_out = 0;
    for (int n = 0; n < n_samp && n_out < n_bits + 8; n++) {
        uint8_t b;
        if (sonde_demod_sample(&d, samples[n], &b))
            bits_out[n_out++] = b;
    }

    /* the PLL needs a handful of bit periods to lock -- search a small skip
     * window on both ends for the best-matching alignment. */
    int best_skip = 0, best_err = n_bits;
    int compare_len = (n_bits > 40) ? n_bits - 40 : n_bits / 2;
    for (int skip = 0; skip <= 8 && skip + compare_len <= n_out; skip++) {
        int err = 0;
        for (int i = 0; i < compare_len; i++)
            if (bits_out[skip + i] != bits_in[i]) err++;
        if (err < best_err) { best_err = err; best_skip = skip; }
    }

    double err_rate = (double)best_err / compare_len;
    char label[96];
    snprintf(label, sizeof label, "%s: bits recovered (n_out=%d/%d)", name, n_out, n_bits);
    chk(label, n_out >= n_bits - 4);
    snprintf(label, sizeof label, "%s: bit error rate < 2%% (skip=%d, err=%d/%d)",
            name, best_skip, best_err, compare_len);
    chk(label, err_rate < 0.02);

    free(bits_in); free(samples); free(bits_out);
}

int main(void)
{
    /* RS41-like: 4800 baud, 10 samples/bit, light noise */
    run_case("RS41 4800@48000", 4800, 48000, 400, 8000, 400);
    /* DFM-like: 2500 baud, ~19.2 samples/bit at 48 kHz */
    run_case("DFM 2500@48000", 2500, 48000, 400, 8000, 400);
    /* M10-like: 9600 baud, 5 samples/bit at 48 kHz -- the tight case */
    run_case("M10 9600@48000", 9600, 48000, 400, 8000, 300);
    /* noisier RS41 case */
    run_case("RS41 4800@48000 noisy", 4800, 48000, 400, 8000, 1500);

    printf(fails ? "\nFAIL (%d)\n" : "\nall pass\n", fails);
    return fails ? 1 : 0;
}
