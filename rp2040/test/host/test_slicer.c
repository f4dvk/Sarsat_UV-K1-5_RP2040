/*
 * test_slicer.c — host validation for audio_slicer.c.
 *
 *  1. Fixed-point vs double parity: for a synthetic bi-phase-L burst the two
 *     implementations must return the identical frame length and bits. This is
 *     what proves the int64 conversion introduced no slicing error.
 *  2. Coherence: report whether the sliced frame passes BCH (informational —
 *     the synthetic generator is a rough stand-in for a real FM-demod capture,
 *     which only hardware / a real recording can fully validate).
 *  3. WAV mode:  ./test_slicer file.wav [rate]  runs the full decode path on a
 *     recording (e.g. a beacon captured through the radio in RAW mode).
 *
 * Exit code: non-zero if any fixed-vs-double parity check fails.
 */
#include "audio_slicer.h"
#include "sarsat_decoder.h"
#include "dec406_v1g.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ---- synthetic bi-phase-L FGB burst ------------------------------------- */
/* 15 bit-sync ones + 9-bit frame sync (000101101) + payload, Manchester:
 * '1' = +A then -A, '0' = -A then +A, at 400 bps. */
static int gen_burst(int32_t *buf, int cap, const uint8_t *frame, int nbits,
                     int rate, unsigned seed)
{
    const int spb = rate / 400;           /* samples per bit */
    const int lead = rate / 20;           /* 50 ms unmodulated carrier */
    const int amp = 8000;
    int n = 0;

    unsigned rng = seed ? seed : 1;
    #define NOISE() ( (int)((rng = rng*1103515245u+12345u) >> 20) % 401 - 200 )

    for (int i = 0; i < lead && n < cap; i++) buf[n++] = amp + NOISE();

    for (int b = 0; b < nbits && n + spb <= cap; b++) {
        int first = frame[b] ? amp : -amp;
        for (int k = 0; k < spb; k++)
            buf[n++] = ((k < spb / 2) ? first : -first) + NOISE();
    }
    for (int i = 0; i < lead && n < cap; i++) buf[n++] = NOISE();
    #undef NOISE
    return n;
}

static void frame_from_hex(const char *hex, uint8_t *bits, int nbits)
{
    memset(bits, 0, nbits);
    for (int i = 0; i < 15; i++) bits[i] = 1;
    const int fs[9] = {0,0,0,1,0,1,1,0,1};
    for (int i = 0; i < 9; i++) bits[15 + i] = fs[i];
    for (int i = 0; hex[i] && 24 + i * 4 + 3 < nbits + 0; i++) {
        int v = (hex[i] <= '9') ? hex[i] - '0' :
                (hex[i] | 32) - 'a' + 10;
        int p = 24 + i * 4;
        bits[p]   = (v >> 3) & 1;
        bits[p+1] = (v >> 2) & 1;
        bits[p+2] = (v >> 1) & 1;
        bits[p+3] = (v >> 0) & 1;
    }
}

static int parity_case(const char *hex, int nbits, int rate, unsigned seed)
{
    static int32_t buf[48000];
    uint8_t src[144];
    frame_from_hex(hex, src, nbits);
    int n = gen_burst(buf, (int)(sizeof(buf) / sizeof(buf[0])), src, nbits, rate, seed);

    uint8_t bf[SLICER_MAX_BITS], bd[SLICER_MAX_BITS];
    int lf = slicer_run_fixed(buf, n, rate, bf);
    int ld = slicer_run_double(buf, n, rate, bd);

    int ok = (lf == ld);
    if (ok && lf > 0) ok = (memcmp(bf, bd, lf) == 0);

    const char *crc = "-";
    if (lf == 112 || lf == 144) {
        char fs[145];
        for (int i = 0; i < lf; i++) fs[i] = bf[i] ? '1' : '0';
        fs[lf] = 0;
        BeaconInfo1G info;
        decode_1g_frame(fs, lf, &info);
        crc = info.crc_error ? "BCH-FAIL" : "BCH-OK";
    }

    printf("  %-4d Hz seed=%-3u  fixed=%-3d double=%-3d  %s   [%s]\n",
           rate, seed, lf, ld, crc, ok ? "PARITY OK" : "PARITY MISMATCH");
    return ok;
}

/* ---- minimal WAV reader (44-byte header, 16-bit PCM, mono/stereo) ------- */
static int32_t *read_wav(const char *path, int *n_out, int *rate_out)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    unsigned char h[44];
    if (fread(h, 1, 44, f) != 44 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) {
        fprintf(stderr, "not a RIFF/WAVE file\n"); fclose(f); return NULL;
    }
    int fmt   = h[20] | (h[21] << 8);           /* 1 = PCM, 3 = IEEE float */
    int ch    = h[22] | (h[23] << 8);
    int rate  = h[24] | (h[25] << 8) | (h[26] << 16) | (h[27] << 24);
    int bits  = h[34] | (h[35] << 8);
    if (!((fmt == 1 && (bits == 16 || bits == 8)) || (fmt == 3 && bits == 32))) {
        fprintf(stderr, "unsupported: fmt=%d bits=%d (need PCM 8/16 or float32)\n",
                fmt, bits);
        fclose(f); return NULL;
    }

    size_t cap = 1 << 20, n = 0;
    int32_t *buf = malloc(cap * sizeof(int32_t));
    unsigned char raw[4 * 8];
    int bps = bits / 8;
    while (fread(raw, bps, ch, f) == (size_t)ch) {
        if (n == cap) { cap *= 2; buf = realloc(buf, cap * sizeof(int32_t)); }
        int32_t v;
        if (fmt == 3) {           /* float32 -> int16 scale */
            float fv; memcpy(&fv, raw, 4); v = (int32_t)(fv * 32767.0f);
        } else if (bits == 16) {
            v = (int16_t)(raw[0] | (raw[1] << 8));
        } else {                  /* 8-bit unsigned */
            v = ((int)raw[0] - 128) * 256;
        }
        buf[n++] = v;
    }
    fclose(f);
    *n_out = (int)n;
    *rate_out = rate;
    return buf;
}

int main(int argc, char **argv)
{
    if (argc >= 2) {
        int n = 0, rate = 0;
        int32_t *buf = read_wav(argv[1], &n, &rate);
        if (!buf) return 2;
        if (argc >= 3) rate = atoi(argv[2]);
        printf("WAV %s : %d samples @ %d Hz\n", argv[1], n, rate);

        uint8_t bf[SLICER_MAX_BITS], bd[SLICER_MAX_BITS];
        int lf = slicer_run_fixed(buf, n, rate, bf);
        int ld = slicer_run_double(buf, n, rate, bd);
        int par = (lf == ld) && (lf <= 0 || memcmp(bf, bd, lf) == 0);
        printf("slicer: fixed=%d double=%d  [%s]\n", lf, ld,
               par ? "PARITY OK" : "PARITY MISMATCH");

        sarsat_result_t r;
        if (sarsat_decode_window(buf, n, rate, &r)) {
            printf("DECODED (%d bits), hex ID %s\n", r.frame_bits, r.hex_id);
            for (int i = 0; i < r.n_lines; i++) printf("  | %s\n", r.lines[i]);
        } else {
            printf("no CRC-clean 1G frame in this window\n");
        }
        free(buf);
        return 0;
    }

    /* self-tests */
    const char *v144 = "fffed08e39048d158ac01e3aa482856824ce";
    int rates[] = {12000, 16000, 22050, 48000};
    int fails = 0;

    printf("fixed-vs-double parity (synthetic bi-phase-L bursts):\n");
    for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
        for (unsigned seed = 1; seed <= 3; seed++)
            fails += !parity_case(v144, 144, rates[i], seed);

    printf("\n%s (%d mismatch)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
