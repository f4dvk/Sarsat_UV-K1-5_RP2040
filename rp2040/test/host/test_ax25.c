/*
 * test_ax25.c — host check of the APRS AX.25 / HDLC encoder
 * (firmware/uv-k5v1-kd8cec/patch/ax25.c).
 *
 * Encodes a UI frame, then independently DECODES the HDLC/NRZI tone stream
 * back (un-NRZI, de-stuff, find flags, verify FCS) and checks the recovered
 * address + info match. Also checks the X.25 FCS residue.
 */
#include "ax25.h"
#include <stdio.h>
#include <string.h>

static int fail;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fail = 1; } } while (0)

/* decode a tone stream back to frame bytes; returns len or -1.
 * un-NRZI + HDLC de-frame (LSB-first, de-stuff, flag = 0x7E). */
static int hdlc_decode(const uint8_t *tones, int nb, uint8_t *out, int outcap)
{
    int prev = tones[0];
    int len = 0, acc = 0, nbit = 0, ones = 0;
    bool sync = false;

    for (int i = 1; i < nb; i++) {
        int b = (tones[i] == prev) ? 1 : 0;   /* NRZI: same tone -> 1 */
        prev = tones[i];

        if (ones == 6 && b == 0) {             /* 0111 1110 -> flag boundary */
            if (sync && len >= 16) return len;               /* closing flag */
            sync = true; len = acc = nbit = 0; ones = 0;     /* opening/idle */
            continue;
        }
        if (ones == 5 && b == 0) { ones = 0; continue; }     /* de-stuff drop */

        ones = b ? ones + 1 : 0;

        if (sync) {
            acc |= b << nbit;
            if (++nbit == 8) {
                if (len >= outcap) return -1;
                out[len++] = (uint8_t)acc; acc = nbit = 0;
            }
        }
    }
    return -1;
}

int main(void)
{
    ax25_addr_t src = { "F4ABC", 9 };
    ax25_addr_t dst = { "APZSAR", 0 };
    ax25_addr_t digi[1] = { { "WIDE1", 1 } };
    const char *info = "!4903.50N/07201.75W>SARSAT-RP2040";

    uint8_t frame[AX25_MAX_FRAME];
    int flen = ax25_build_ui(frame, &dst, &src, digi, 1, info, strlen(info));
    printf("UI frame: %d bytes\n", flen);
    CHECK(flen > 0);

    /* FCS self-check: recompute over the body, compare to the 2 appended bytes */
    uint16_t crc = ax25_fcs(frame, flen - 2);
    CHECK((frame[flen - 2] == (crc & 0xFF)) && (frame[flen - 1] == (crc >> 8)));

    /* address decode sanity: src callsign is at bytes 7..12, shifted << 1 */
    char got[7] = {0};
    for (int i = 0; i < 6; i++) got[i] = frame[7 + i] >> 1;
    CHECK(strncmp(got, "F4ABC ", 6) == 0);

    /* HDLC + NRZI, then decode back */
    uint8_t tones[AX25_MAX_BITS];
    int nb = ax25_hdlc_nrzi(frame, flen, 24, tones);
    printf("HDLC/NRZI: %d bit cells (%d ms @ 1200 bd)\n", nb, nb * 1000 / 1200);
    CHECK(nb > 0);

    uint8_t dec[AX25_MAX_FRAME];
    int dlen = hdlc_decode(tones, nb, dec, sizeof dec);
    printf("decoded back: %d bytes\n", dlen);
    CHECK(dlen == flen);
    CHECK(dlen > 0 && memcmp(dec, frame, flen) == 0);
    if (dlen == flen) {
        uint16_t dcrc = ax25_fcs(dec, dlen - 2);
        CHECK((dec[dlen - 2] == (dcrc & 0xFF)) && (dec[dlen - 1] == (dcrc >> 8)));
        /* info field starts after 7+7+7 (digi) + 2 (ctrl+pid) */
        int ioff = 7 + 7 + 7 + 2;
        CHECK(memcmp(dec + ioff, info, strlen(info)) == 0);
    }

    /* a frame that forces bit-stuffing (many 0xFF-ish) still round-trips */
    const char *info2 = "!0000.00N/00000.00E`\x7f\x7f\x7f stuff test";
    int f2 = ax25_build_ui(frame, &dst, &src, NULL, 0, info2, strlen(info2));
    nb = ax25_hdlc_nrzi(frame, f2, 8, tones);
    dlen = hdlc_decode(tones, nb, dec, sizeof dec);
    CHECK(dlen == f2 && memcmp(dec, frame, f2) == 0);
    printf("bit-stuff round-trip: %s\n", (dlen == f2) ? "ok" : "FAIL");

    printf("\n%s\n", fail ? "FAIL" : "PASS");
    return fail;
}
