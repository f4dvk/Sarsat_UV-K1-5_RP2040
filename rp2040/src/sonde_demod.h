/*
 * sonde_demod.h -- shared bit-level demodulator for radiosonde telemetry
 * (RS41 4800-baud GFSK, DFM 2500-baud FSK, M10/M20 GFSK), Sarsat_UV-K1-5_RP2040.
 *
 * Input is FM-DISCRIMINATED audio (the same kind of signal the SARSAT and
 * APRS decoders already consume from the C-Board's ADC tap) -- radiosondes
 * are binary FSK/GFSK, so after the discriminator the signal is already a
 * two-level (mark/space) baseband waveform; no tone correlation is needed
 * (unlike APRS's Bell-202, which modulates two audio TONES onto the carrier
 * and needs a tone decoder first).
 *
 * Architecture: the same "DireWolf-style" bit PLL already proven in this
 * project for APRS (see aprs_rx.c: a 32-bit phase accumulator that emits a
 * bit on each wrap, nudged towards observed data transitions), reused
 * as-is for a different, configurable baud rate and a plain two-level
 * slice instead of a tone bank. Fixed-point (no floats in the per-sample
 * path), so it runs on an FPU-less RP2040 core, same rationale as
 * audio_slicer.h.
 *
 * ⚠️ Unvalidated against real radiosonde RF (see the project's radiosonde
 * plan notes: no reference capture available; expect on-air correction).
 * The host test only proves the algorithm recovers bits correctly from a
 * synthetic, Gaussian-shaped two-level test signal -- it cannot validate
 * real-world SNR/drift behaviour.
 */
#ifndef SONDE_DEMOD_H
#define SONDE_DEMOD_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* DC-centring (adaptive hi/lo envelope, like aprs_rx's lp_hi/lp_lo) */
    int32_t lp, lp_hi, lp_lo;
    /* dead-band threshold tracking */
    int32_t env;
    /* bit-cell PLL (DireWolf-style phase accumulator) */
    uint32_t pll, pll_step;
    int      bit, pval;
    bool     inited;
} sonde_demod_t;

/* `sample_rate_hz` : ADC capture rate. `baud` : target symbol rate (4800 for
 * RS41, 2500 for DFM, whatever is confirmed on air for M10/M20). */
void sonde_demod_init(sonde_demod_t *d, int sample_rate_hz, int baud);

/* Feed one discriminator sample (nominally |x| <= 32768, same convention as
 * audio_slicer.h). Returns true when a bit-cell boundary was just crossed,
 * with the sliced bit (0/1, NRZ -- no NRZI un-mapping, callers that need
 * NRZI do it themselves) written to *bit_out. May return true 0 or 1 times
 * per call, never more (one call = one sample). */
bool sonde_demod_sample(sonde_demod_t *d, int32_t sample, uint8_t *bit_out);

#endif /* SONDE_DEMOD_H */
