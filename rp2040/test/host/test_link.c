/*
 * test_link.c — host check of the Phase 2 receive path (quansheng_frame_feed).
 *
 * Feeds hand-built reply frames (as the radio / tools/fake_radio.py would send)
 * one byte at a time and verifies the parser recovers the id + payload. Also
 * checks resync after garbage.
 *
 *   ./test_link                 run the built-in vectors
 *   ./test_link <hex>           parse one frame given as a hex string
 */
#include "quansheng_frame.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* local frame builder, mirrors tools/fake_radio.py build() */
static const uint8_t OBF[16] = {
    0x16,0x6C,0x14,0xE6,0x2E,0x91,0x0D,0x40,0x21,0x35,0xD5,0x40,0x13,0x03,0xE9,0x80
};
static size_t build(uint16_t id, const uint8_t *data, uint8_t dlen, uint8_t *out)
{
    uint8_t inner[4 + 255];
    inner[0] = id; inner[1] = id >> 8; inner[2] = dlen; inner[3] = 0;
    memcpy(inner + 4, data, dlen);
    uint16_t crc = quansheng_crc16_xmodem(inner, 4 + dlen);
    inner[4 + dlen] = crc; inner[5 + dlen] = crc >> 8;
    size_t obf = 6 + dlen, o = 0;
    for (size_t i = 0; i < obf; i++) inner[i] ^= OBF[i & 15];
    out[o++] = 0xAB; out[o++] = 0xCD;
    out[o++] = (uint8_t)(4 + dlen); out[o++] = 0;
    memcpy(out + o, inner, obf); o += obf;
    out[o++] = 0xDC; out[o++] = 0xBA;
    return o;
}

static int feed_all(qframe_rx_t *rx, const uint8_t *buf, size_t n,
                    uint16_t *id, const uint8_t **d, uint16_t *dl)
{
    int got = 0;
    for (size_t i = 0; i < n; i++)
        if (quansheng_frame_feed(rx, buf[i], id, d, dl)) got++;
    return got;
}

static int fail;
#define CHECK(c) do { if (!(c)) { printf("  FAIL: %s\n", #c); fail = 1; } } while (0)

int main(int argc, char **argv)
{
    qframe_rx_t rx = {0};
    uint16_t id, dl;
    const uint8_t *d;

    if (argc > 1) {
        uint8_t buf[300]; size_t n = 0;
        for (const char *p = argv[1]; p[0] && p[1] && n < sizeof buf; p += 2) {
            unsigned v; sscanf(p, "%2x", &v); buf[n++] = v;
        }
        int g = feed_all(&rx, buf, n, &id, &d, &dl);
        printf("frames=%d  id=0x%04X  dlen=%u  data=", g, id, dl);
        for (int i = 0; i < dl; i++) printf("%02X ", d[i]);
        printf("\n");
        return 0;
    }

    /* 1. ACK for 0x06C1 */
    {
        uint8_t st = 0, f[64];
        size_t n = build(0x06C1 | 0x8000, &st, 1, f);
        CHECK(feed_all(&rx, f, n, &id, &d, &dl) == 1);
        CHECK(id == 0x86C1 && dl == 1 && d[0] == 0);
        printf("ACK 0x06C1: id=0x%04X status=%u  OK\n", id, d[0]);
    }
    /* 2. HELLO status reply */
    {
        uint32_t freq = 40602500;
        uint8_t p[8] = { 0, 0, freq, freq>>8, freq>>16, freq>>24, 0, 1 }, f[64];
        size_t n = build(0x06CF | 0x8000, p, 8, f);
        CHECK(feed_all(&rx, f, n, &id, &d, &dl) == 1);
        CHECK(id == 0x86CF && dl == 8);
        uint32_t g = d[2] | (d[3]<<8) | (d[4]<<16) | ((uint32_t)d[5]<<24);
        CHECK(g == freq);
        printf("HELLO reply: vfo=%u mod=%u freq=%lu proto=%u  OK\n",
               d[0], d[1], (unsigned long)g, d[7]);
    }
    /* 3. garbage then a good frame -> parser must resync */
    {
        uint8_t g[5] = { 0x11, 0xAB, 0x22, 0xAB, 0xCD }, st = 0, f[64];
        size_t n = build(0x06C0 | 0x8000, &st, 1, f);
        feed_all(&rx, g, sizeof g, &id, &d, &dl);
        CHECK(feed_all(&rx, f, n, &id, &d, &dl) == 1);
        CHECK(id == 0x86C0);
        printf("resync after garbage: id=0x%04X  OK\n", id);
    }
    /* 4. two frames back to back in one feed */
    {
        uint8_t st = 0, f[128]; size_t n = 0;
        n += build(0x06C1 | 0x8000, &st, 1, f + n);
        n += build(0x06C1 | 0x8000, &st, 1, f + n);
        CHECK(feed_all(&rx, f, n, &id, &d, &dl) == 2);
        printf("two frames in one feed: OK\n");
    }

    printf("\n%s\n", fail ? "FAIL" : "PASS");
    return fail;
}
