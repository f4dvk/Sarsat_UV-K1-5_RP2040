/* test_kiss -- KISS/SLIP framing round-trip + escape handling. */
#include <stdio.h>
#include <string.h>
#include "kiss.h"

static int fails;

static void chk(const char *name, int cond)
{
    printf("  %-40s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

/* feed a whole byte stream through the decoder; returns the payload of the
 * first complete data frame (len via *out_len), or -1 if none. */
static int run_decoder(const uint8_t *bytes, int n, uint8_t *out, int *out_len)
{
    kiss_dec_t d;
    memset(&d, 0, sizeof d);
    for (int i = 0; i < n; i++) {
        int fl = kiss_decode_byte(&d, bytes[i]);
        if (fl > 0) { memcpy(out, d.frame, fl); *out_len = fl; return 0; }
    }
    return -1;
}

int main(void)
{
    printf("KISS framing:\n");

    /* 1. plain frame round-trips */
    {
        const uint8_t f[] = { 0x82,0xA0,0xAA,0x64,0x6A,0x9C,0x60,
                              0x8C,0x68,0x88,0xAC,0x96,0x40,0xE1,
                              0x03,0xF0, 'h','e','l','l','o' };
        uint8_t k[128]; int kn = kiss_encode(k, sizeof k, f, (int)sizeof f);
        chk("encode: FEND framed, cmd 0",
            kn > 0 && k[0] == KISS_FEND && k[1] == 0x00 && k[kn-1] == KISS_FEND);
        uint8_t out[KISS_MAX_FRAME]; int ol = 0;
        chk("decode: round-trips", run_decoder(k, kn, out, &ol) == 0);
        chk("  same length", ol == (int)sizeof f);
        chk("  same bytes", memcmp(out, f, sizeof f) == 0);
    }

    /* 2. payload containing FEND / FESC must survive escaping */
    {
        const uint8_t f[] = { 0x00, KISS_FEND, 0x11, KISS_FESC, 0x22,
                              KISS_FEND, KISS_FESC, 0xFF };
        uint8_t k[64]; int kn = kiss_encode(k, sizeof k, f, (int)sizeof f);
        /* the raw stream must not contain a bare FEND except the two delimiters */
        int bare = 0;
        for (int i = 1; i < kn - 1; i++) if (k[i] == KISS_FEND) bare++;
        chk("encode: no bare FEND inside", bare == 0);
        uint8_t out[KISS_MAX_FRAME]; int ol = 0;
        chk("decode: escapes resolve", run_decoder(k, kn, out, &ol) == 0 &&
            ol == (int)sizeof f && memcmp(out, f, sizeof f) == 0);
    }

    /* 3. leading garbage before the first FEND is ignored (host just connected) */
    {
        const uint8_t f[] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17 };
        uint8_t k[64]; int kn = kiss_encode(k, sizeof k, f, (int)sizeof f);
        uint8_t s[80]; int n = 0;
        const char *junk = "=== boot banner ===\r\n";
        for (const char *p = junk; *p; p++) s[n++] = (uint8_t)*p;
        memcpy(s + n, k, kn); n += kn;
        uint8_t out[KISS_MAX_FRAME]; int ol = 0;
        chk("decode: resyncs after junk", run_decoder(s, n, out, &ol) == 0 &&
            ol == (int)sizeof f && memcmp(out, f, sizeof f) == 0);
    }

    /* 4. back-to-back frames sharing one FEND */
    {
        const uint8_t a[] = { 10,11,12,13,14,15,16,17 };
        const uint8_t b[] = { 20,21,22,23,24,25,26,27,28 };
        uint8_t ka[32], kb[32];
        int na = kiss_encode(ka, sizeof ka, a, (int)sizeof a);
        int nb = kiss_encode(kb, sizeof kb, b, (int)sizeof b);
        uint8_t s[80]; int n = 0;
        memcpy(s, ka, na); n = na;
        memcpy(s + n, kb + 1, nb - 1); n += nb - 1;   /* drop kb's leading FEND */
        kiss_dec_t d; memset(&d, 0, sizeof d);
        int seen = 0; uint8_t last[KISS_MAX_FRAME]; int ll = 0;
        for (int i = 0; i < n; i++) {
            int fl = kiss_decode_byte(&d, s[i]);
            if (fl > 0) { seen++; memcpy(last, d.frame, fl); ll = fl; }
        }
        chk("decode: two frames, shared FEND", seen == 2);
        chk("  second frame intact",
            ll == (int)sizeof b && memcmp(last, b, sizeof b) == 0);
    }

    /* 5. a non-data KISS frame (e.g. TXDELAY, command 1) is not forwarded */
    {
        uint8_t s[] = { KISS_FEND, 0x01, 0x28, KISS_FEND };
        uint8_t o[KISS_MAX_FRAME]; int ol = 0;
        chk("decode: command frame dropped", run_decoder(s, sizeof s, o, &ol) != 0);
    }

    /* 6. an over-long frame is dropped, decoder recovers on the next one */
    {
        kiss_dec_t d; memset(&d, 0, sizeof d);
        kiss_decode_byte(&d, KISS_FEND);
        kiss_decode_byte(&d, 0x00);
        for (int i = 0; i < KISS_MAX_FRAME + 50; i++)
            kiss_decode_byte(&d, 0x55);
        int fl = kiss_decode_byte(&d, KISS_FEND);   /* closes the bad frame */
        chk("decode: over-long dropped", fl == 0);
        const uint8_t g[] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
        kiss_decode_byte(&d, KISS_FEND);            /* start a clean frame */
        kiss_decode_byte(&d, 0x00);                 /* data command */
        int seen = 0;
        for (size_t i = 0; i < sizeof g; i++) seen = kiss_decode_byte(&d, g[i]);
        seen = kiss_decode_byte(&d, KISS_FEND);
        chk("  recovers next frame", seen == (int)sizeof g);
    }

    printf(fails ? "\nFAIL\n" : "\nPASS\n");
    return fails ? 1 : 0;
}
