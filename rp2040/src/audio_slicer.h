/*
 * audio_slicer.h — COSPAS-SARSAT 1st-generation (FGB) bit slicer for
 * FM-demodulated audio.
 *
 * This is the F4EHY autocorrelation / bi-phase-L slicer from
 * moricef/Decode_sarsat_406_v1g_v2g (src/audio_capture.c, capture_trame()),
 * re-expressed for the Sarsat_UV-K1-5_RP2040 project:
 *   - fixed-point (int64) correlator instead of double, so it runs on an
 *     FPU-less RP2040 core;
 *   - no WAV reader, no stdin, no clock() timeout, no globals — the caller
 *     hands in a plain sample buffer;
 *   - it returns the sliced bits instead of calling the decoder itself.
 *
 * A bit-for-bit double reference (slicer_run_double) is kept for the host test
 * harness to validate the fixed-point path against.
 */
#ifndef SARSAT_AUDIO_SLICER_H
#define SARSAT_AUDIO_SLICER_H

#include <stdint.h>

/* Upper bound on frame length (long frame = 144 bits). */
#define SLICER_MAX_BITS 144

/* Scan `samples` (FM-demodulated PCM, signed, nominally |x| <= 32768) at
 * `rate` Hz for one FGB burst.
 *
 *   bits_out : caller buffer, >= SLICER_MAX_BITS, filled with 0/1 bytes
 *   returns  : frame length in bits (112 or 144) on success, 0 otherwise
 *
 * `rate / 400` (samples per bit) must be between 12 and 120 inclusive
 * (i.e. 4800 Hz <= rate <= 48000 Hz); the function returns 0 otherwise.
 */
int slicer_run(const int32_t *samples, int n, int rate, uint8_t *bits_out);

/* Same contract; explicit implementations for cross-validation. */
int slicer_run_fixed (const int32_t *samples, int n, int rate, uint8_t *bits_out);
int slicer_run_double(const int32_t *samples, int n, int rate, uint8_t *bits_out);

#endif /* SARSAT_AUDIO_SLICER_H */
