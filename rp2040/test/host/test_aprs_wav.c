/*
 * test_aprs_wav — decode an off-line ADC capture through the real aprs_rx chain.
 *
 * Input: a text file with the RP2040's provisional `[aprs.raw]` dump (the `w`
 * serial command). Lines look like:
 *
 *     [aprs.raw] begin n=17999 fs=13200
 *     [aprs.raw] 7FE7FD8021FA...          (32 samples, 3 hex chars = 12-bit each)
 *     ...
 *     [aprs.raw] end
 *
 * Everything else in the file is ignored, so you can pipe a whole console log.
 *
 *   ./test_aprs_wav capture.txt
 *
 * It feeds the samples to aprs_rx one at a time and prints every HDLC frame
 * candidate (same format as the on-target `[aprs.rx]` line) plus a summary, so
 * a decode problem seen on the air can be dissected on the host.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "aprs_rx.h"

static int g_pkts;

static void on_packet(const uint8_t *ax25, int len, void *user)
{
    (void)user;
    g_pkts++;
    char src[10] = "?";
    if (len >= 14) {
        int k = 0;
        for (int i = 7; i < 13 && k < 6; i++) {
            char c = (char)(ax25[i] >> 1);
            if (c > ' ' && c < 0x7f) src[k++] = c;
        }
        int ssid = (ax25[13] >> 1) & 0x0F;
        if (ssid) { src[k++] = '-'; src[k++] = (char)('0' + ssid); }
        src[k] = 0;
    }
    printf("  >>> PACKET #%d  len=%d  src=%s\n", g_pkts, len, src);
}

static void frame_cb(void *u, const uint8_t *f, int len, int ch, int res, int ui)
{
    (void)u;
    static const char *const R[] = { "OK ", "FIX", "BAD", "SHT", "LNG", "DUP" };
    if (res == APRS_RXR_SHORT) return;
    char src[10] = "?";
    if (len >= 14) {
        int k = 0;
        for (int i = 7; i < 13 && k < 6; i++) {
            char c = (char)(f[i] >> 1);
            if (c > ' ' && c < 0x7f) src[k++] = c;
        }
        int ssid = (f[13] >> 1) & 0x0F;
        if (ssid) { src[k++] = '-'; src[k++] = (char)('0' + ssid); }
        src[k] = 0;
    }
    printf("  ch%d %s len=%-3d ui=%d src=%-8s | ", ch, R[res < 6 ? res : 2],
           len, ui, src);
    for (int i = 0; i < len && i < 24; i++) printf("%02X ", f[i]);
    printf("\n");
}

/* pull every 3-hex-digit ADC sample out of the [aprs.raw] body */
static int load(const char *path, uint16_t **out)
{
    FILE *fp = strcmp(path, "-") ? fopen(path, "r") : stdin;
    if (!fp) { perror(path); exit(1); }

    size_t cap = 1 << 16, n = 0;
    uint16_t *buf = malloc(cap * sizeof *buf);
    char line[512];
    int in_body = 0;

    while (fgets(line, sizeof line, fp)) {
        char *p = strstr(line, "[aprs.raw]");
        if (!p) continue;
        p += 10;
        while (*p == ' ') p++;
        if (!strncmp(p, "begin", 5)) { in_body = 1; continue; }
        if (!strncmp(p, "end", 3))   { in_body = 0; continue; }
        if (!in_body) continue;

        /* a run of hex chars, 3 per sample */
        for (char *q = p; ; ) {
            while (*q && !isxdigit((unsigned char)*q)) q++;
            int c0 = *q, c1 = c0 ? q[1] : 0, c2 = c1 ? q[2] : 0;
            if (!isxdigit(c0) || !isxdigit(c1) || !isxdigit(c2)) break;
            char h[4] = { (char)c0, (char)c1, (char)c2, 0 };
            if (n == cap) { cap *= 2; buf = realloc(buf, cap * sizeof *buf); }
            buf[n++] = (uint16_t)(strtol(h, NULL, 16) & 0x0FFF);
            q += 3;
        }
    }
    if (fp != stdin) fclose(fp);
    *out = buf;
    return (int)n;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <capture.txt|->\n", argv[0]);
        return 2;
    }

    uint16_t *smp;
    int n = load(argv[1], &smp);
    if (n < 100) { fprintf(stderr, "only %d samples parsed\n", n); return 1; }

    printf("loaded %d samples (%.2f s @ %d Hz)\n",
           n, (double)n / APRS_RX_SAMPLE_RATE_HZ, APRS_RX_SAMPLE_RATE_HZ);

    aprs_rx_t rx;
    aprs_rx_init(&rx, APRS_RX_SAMPLE_RATE_HZ, on_packet, NULL);
    aprs_rx_set_frame_cb(&rx, frame_cb, NULL);

    for (int i = 0; i < n; i++)
        aprs_rx_sample(&rx, smp[i]);

    int32_t cdt = 0, env = 0; int car = 0;
    aprs_rx_levels(&rx, &cdt, &env, &car);
    printf("\nsummary: hdlc=%lu  pkts=%lu  fixed=%lu  fcs_bad=%lu  "
           "clip=%lu/1000  env=%ld\n",
           (unsigned long)rx.n_hdlc, (unsigned long)rx.n_packets,
           (unsigned long)rx.n_fixed, (unsigned long)rx.n_fcs_bad,
           (unsigned long)aprs_rx_clip_permille(&rx), (long)env);

    free(smp);
    return 0;
}
