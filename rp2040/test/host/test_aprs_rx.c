/*
 * test_aprs_rx — round-trip the APRS RX chain on the host.
 *
 * Encode an AX.25 UI frame with the (host-tested) K5 encoder, synthesize
 * phase-continuous Bell-202 AFSK at 13200 Hz, feed it to aprs_rx sample by
 * sample, and check the packet comes back byte-identical (minus FCS).
 * Also checks: leading/trailing silence, a mistuned (amplitude-varying)
 * copy, and a copy with additive noise.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ax25.h"
#include "aprs_rx.h"

#define FS APRS_RX_SAMPLE_RATE_HZ

#define TONE_BIT(tones, i) (((tones)[(i) >> 3] >> ((i) & 7)) & 1u)

static uint8_t  g_pkt[APRS_RX_MAX_FRAME];
static int      g_pkt_len;
static int      g_pkt_count;
static bool     g_pkt_fixed;

static void on_packet(const uint8_t *ax25, int len, bool fixed, void *user)
{
    (void)user;
    g_pkt_count++;
    g_pkt_len = len;
    g_pkt_fixed = fixed;
    if (len > 0 && len <= (int)sizeof g_pkt)
        memcpy(g_pkt, ax25, len);
}

/* one AFSK run: silence, tones, silence -> aprs_rx. `rate` scales the TX
 * clock (baud and tones; 1.0 = 11 samples/bit-cell at 13200 Hz). */
static void play(aprs_rx_t *r, const uint8_t *tones, int nb,
                 double amp, double noise, unsigned seed, double rate)
{
    srand(seed);
    double phase = 0.0;
    const double two_pi = 6.283185307179586;

    for (int i = 0; i < FS / 5; i++)                 /* 200 ms silence */
        aprs_rx_sample(r, (uint16_t)(2048 + (rand() % 17 - 8)));

    const double tb = 11.0 / rate;
    const double frac = (seed % 11) / 11.0;          /* sub-sample bit phase */
    const int ns = (int)(nb * tb);
    for (int k = 0; k < ns; k++) {
        int b = (int)((k + frac) / tb);
        if (b >= nb) b = nb - 1;
        double f = (TONE_BIT(tones, b) ? 1200.0 : 2200.0) * rate;
        phase += two_pi * f / FS;
        if (phase > two_pi) phase -= two_pi;
        double v = 2048.0 + amp * 1500.0 * sin(phase);
        if (noise > 0.0)
            v += noise * 1500.0 * ((rand() / (double)RAND_MAX) - 0.5);
        int iv = (int)(v + 0.5);
        if (iv < 0) iv = 0;
        if (iv > 4095) iv = 4095;
        aprs_rx_sample(r, (uint16_t)iv);
    }

    for (int i = 0; i < FS / 5; i++)
        aprs_rx_sample(r, (uint16_t)(2048 + (rand() % 17 - 8)));
}

static int one_case(const char *name, double amp, double noise, unsigned seed)
{
    ax25_addr_t src = { "F4DVK", 9 };
    ax25_addr_t dst = { "APZSAR", 0 };
    ax25_addr_t digi[2] = { { "WIDE1", 1 }, { "WIDE2", 1 } };
    const char *info = "!4237.50N/00121.30E>SARSAT-RP2040";

    uint8_t frame[AX25_MAX_FRAME];
    int flen = ax25_build_ui(frame, &dst, &src, digi, 2, info, (int)strlen(info));
    if (!flen) { printf("  %-22s BUILD FAIL\n", name); return 1; }

    static uint8_t tones[(AX25_MAX_BITS + 7) / 8];
    int nb = ax25_hdlc_nrzi(frame, flen, 40, tones);
    if (!nb) { printf("  %-22s HDLC FAIL\n", name); return 1; }

    aprs_rx_t rx;
    aprs_rx_init(&rx, FS, on_packet, NULL);
    g_pkt_count = 0;
    g_pkt_len = 0;

    play(&rx, tones, nb, amp, noise, seed, 1.0);

    /* expected = frame without its 2 FCS bytes */
    int explen = flen - 2;
    int ok = (g_pkt_count == 1) && (g_pkt_len == explen) &&
             (memcmp(g_pkt, frame, explen) == 0);

    printf("  %-22s packets=%d (fixed=%u) fcs_bad=%u  %s\n",
           name, g_pkt_count, rx.n_fixed, rx.n_fcs_bad, ok ? "OK" : "FAIL");

    if (ok && !strcmp(name, "clean")) {
        char lines[6][40];
        int n = aprs_rx_format(g_pkt, g_pkt_len, (char *)lines, 6, 39);
        for (int i = 0; i < n; i++)
            printf("      | %s\n", lines[i]);
    }
    return ok ? 0 : 1;
}

/* a TX whose clock is off-nominal: n distinct frames, all must decode */
static int rate_case(const char *name, double rate, int n)
{
    ax25_addr_t src = { "F4DVK", 9 }, dst = { "APZSAR", 0 };
    ax25_addr_t digi[2] = { { "WIDE1", 1 }, { "WIDE2", 1 } };
    int good = 0;

    for (int i = 0; i < n; i++) {
        char info[64];
        int il = snprintf(info, sizeof info, "!%02d37.%02dN/00121.30E>rate %d/%d",
                          i % 90, (i * 37) % 100, i, n);
        uint8_t frame[AX25_MAX_FRAME];
        int flen = ax25_build_ui(frame, &dst, &src, digi, 2, info, il);
        static uint8_t tones[(AX25_MAX_BITS + 7) / 8];
        int nb = ax25_hdlc_nrzi(frame, flen, 40, tones);

        aprs_rx_t rx;
        aprs_rx_init(&rx, FS, on_packet, NULL);
        g_pkt_count = 0;
        g_pkt_len = 0;
        play(&rx, tones, nb, 1.0, 0.0, 100u + (unsigned)i, rate);
        good += g_pkt_count == 1 && g_pkt_len == flen - 2 &&
                memcmp(g_pkt, frame, (size_t)(flen - 2)) == 0;
    }
    printf("  %-22s %d/%d decoded  %s\n", name, good, n, good == n ? "OK" : "FAIL");
    return good == n ? 0 : 1;
}

/* one bit corrupted on the air: every chain repairs it, so the frame comes
 * out once, restored, and flagged `fixed` (the digipeater then skips it) */
static int repaired_flag_case(void)
{
    ax25_addr_t src = { "F4DVK", 9 }, dst = { "APZSAR", 0 };
    ax25_addr_t digi[1] = { { "WIDE2", 1 } };
    const char *info = "!4237.50N/00121.30E>repaired frame";
    uint8_t frame[AX25_MAX_FRAME];
    int flen = ax25_build_ui(frame, &dst, &src, digi, 1, info, (int)strlen(info));

    uint8_t sent[AX25_MAX_FRAME];
    memcpy(sent, frame, (size_t)flen);
    sent[30] ^= 0x04;                          /* inside the info field */

    static uint8_t tones[(AX25_MAX_BITS + 7) / 8];
    int nb = ax25_hdlc_nrzi(sent, flen, 40, tones);

    aprs_rx_t rx;
    aprs_rx_init(&rx, FS, on_packet, NULL);
    g_pkt_count = 0;
    g_pkt_len = 0;
    g_pkt_fixed = false;
    play(&rx, tones, nb, 1.0, 0.0, 11, 1.0);

    int ok = g_pkt_count == 1 && g_pkt_fixed && g_pkt_len == flen - 2 &&
             memcmp(g_pkt, frame, (size_t)(flen - 2)) == 0;
    printf("  repaired -> flagged   packets=%d fixed=%d  %s\n",
           g_pkt_count, g_pkt_fixed, ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}

/* direct test of the single-bit-flip repair */
static int fix_case(void)
{
    ax25_addr_t src = { "F4DVK", 9 }, dst = { "APZSAR", 0 };
    ax25_addr_t digi[1] = { { "WIDE2", 1 } };
    const char *info = "!4237.50N/00121.30E>test";
    uint8_t f[AX25_MAX_FRAME];
    int flen = ax25_build_ui(f, &dst, &src, digi, 1, info, (int)strlen(info));

    if (aprs_fcs_residue(f, flen) != 0x0F47) { printf("  bit-flip repair       FCS-base FAIL\n"); return 1; }

    int fixed = 0, total = 0;
    for (int i = 0; i < flen - 2; i++) {          /* corrupt each data bit once */
        for (int b = 0; b < 8; b++) {
            uint8_t c[AX25_MAX_FRAME];
            memcpy(c, f, (size_t)flen);
            c[i] ^= (uint8_t)(1u << b);
            total++;
            if (aprs_try_fix(c, flen) && memcmp(c, f, (size_t)flen) == 0)
                fixed++;
        }
    }
    int ok = (fixed == total);
    printf("  bit-flip repair       %d/%d single-bit errors recovered  %s\n",
           fixed, total, ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}

/* direct test of the cross-channel repair: two independent chains, each
 * individually still FCS-bad (2 bit errors on one, 1 on the other), whose
 * errors don't land in the same byte -- aprs_try_fix_cross() should combine
 * them into the correct frame even though aprs_try_fix() (single-bit) alone
 * cannot fix either one. */
static int cross_fix_case(void)
{
    ax25_addr_t src = { "F4DVK", 9 }, dst = { "APZSAR", 0 };
    ax25_addr_t digi[1] = { { "WIDE2", 1 } };
    const char *info = "!4237.50N/00121.30E>cross-fix test frame content";
    uint8_t f[AX25_MAX_FRAME];
    int flen = ax25_build_ui(f, &dst, &src, digi, 1, info, (int)strlen(info));

    if (aprs_fcs_residue(f, flen) != 0x0F47) {
        printf("  cross-channel repair  FCS-base FAIL\n");
        return 1;
    }

    /* all 3 positions sit well inside the info field (address+ctl+pid is
     * dst 7 + src 7 + digi 7 + ctl 1 + pid 1 = 23 bytes), so corrupting them
     * can't confuse ax25_looks_like_ui()'s address-block parsing. */
    const int posA1 = 30, posA2 = 40, posB = 35;

    uint8_t a[AX25_MAX_FRAME];                 /* chain A: 2 bad bits */
    memcpy(a, f, (size_t)flen);
    a[posA1] ^= 0x04;
    a[posA2] ^= 0x10;

    uint8_t b[AX25_MAX_FRAME];                 /* chain B: 1 bad bit, at a
                                                 * position A got right */
    memcpy(b, f, (size_t)flen);
    b[posB] ^= 0x02;

    aprs_rx_t rx;
    memset(&rx, 0, sizeof rx);
    rx.last_cand[1].len   = flen;
    rx.last_cand[1].at    = 0;
    rx.last_cand[1].valid = true;
    memcpy(rx.last_cand[1].data, b, (size_t)flen);
    rx.now = 10;                               /* well inside the repair's
                                                 * ~50 ms staleness window */

    uint8_t try1[AX25_MAX_FRAME];              /* chain A alone: 1-bit repair
                                                 * must fail (2 bad bits) */
    memcpy(try1, a, (size_t)flen);
    bool single_fixed = aprs_try_fix(try1, flen);

    uint8_t try2[AX25_MAX_FRAME];              /* chain A + B's cache: must
                                                 * reconstruct f exactly */
    memcpy(try2, a, (size_t)flen);
    bool cross_fixed = aprs_try_fix_cross(&rx, 0, try2, flen);

    int ok = !single_fixed && cross_fixed && memcmp(try2, f, (size_t)flen) == 0;
    printf("  cross-channel repair  single=%-20s cross=%-6s  %s\n",
           single_fixed ? "fixed(unexpected)" : "still-bad(expected)",
           cross_fixed  ? "fixed" : "FAIL",
           ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}

int main(void)
{
    int bad = 0;
    printf("APRS RX round-trip (encode -> AFSK 13200 Hz -> aprs_rx):\n");
    bad += one_case("clean",           1.00, 0.00, 1);
    bad += one_case("quiet (-12 dB)",  0.25, 0.00, 2);
    bad += one_case("hot (1.35x)",     1.35, 0.00, 3);
    bad += one_case("noisy (SNR~18dB)",1.00, 0.30, 4);
    bad += one_case("noisy quiet",     0.40, 0.15, 5);
    bad += one_case("clipping (2.0x)", 2.00, 0.00, 6);
    bad += one_case("clipping (4.0x)", 4.00, 0.00, 8);
    bad += one_case("clipping (6.0x)", 6.00, 0.00, 9);
    bad += one_case("clip + noise",    3.00, 0.30, 10);
    bad += one_case("noisy (SNR~15dB)",1.00, 0.45, 7);
    bad += rate_case("TX clock +1 %",  1.01, 40);
    bad += rate_case("TX clock -1 %",  0.99, 40);
    bad += repaired_flag_case();
    bad += fix_case();
    bad += cross_fix_case();

    printf(bad ? "\nFAIL\n" : "\nPASS\n");
    return bad ? 1 : 0;
}
