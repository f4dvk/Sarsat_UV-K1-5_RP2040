/* AX.25 UI-frame + HDLC encoder — see ax25.h. */
#include "ax25.h"
#include <string.h>
#include <ctype.h>

uint16_t ax25_fcs(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
    return crc ^ 0xFFFF;
}

static int put_addr(uint8_t *p, const ax25_addr_t *a, bool last)
{
    int n = 0;
    for (; n < 6; n++) {
        char c = (n < 6 && a->call[n]) ? a->call[n] : ' ';
        if (c >= 'a' && c <= 'z') c -= 32;
        p[n] = (uint8_t)(c << 1);
    }
    /* SSID octet: 0 1 1 SSID(4) 0 ; LSB=1 on the final address octet */
    p[6] = (uint8_t)(0x60 | ((a->ssid & 0x0F) << 1) | (last ? 1 : 0));
    return 7;
}

int ax25_build_ui(uint8_t *out,
                  const ax25_addr_t *dst, const ax25_addr_t *src,
                  const ax25_addr_t *digi, int ndigi,
                  const char *info, int infolen)
{
    if (!out || !dst || !src || infolen < 0 || infolen > AX25_MAX_INFO)
        return 0;
    if (ndigi < 0 || ndigi > AX25_MAX_DIGI)
        return 0;

    int n = 0;
    n += put_addr(out + n, dst, false);
    n += put_addr(out + n, src, ndigi == 0);
    for (int i = 0; i < ndigi; i++)
        n += put_addr(out + n, &digi[i], i == ndigi - 1);

    out[n++] = 0x03;              /* control: UI */
    out[n++] = 0xF0;              /* PID: no layer 3 */
    memcpy(out + n, info, infolen);
    n += infolen;

    uint16_t crc = ax25_fcs(out, n);
    out[n++] = (uint8_t)(crc & 0xFF);
    out[n++] = (uint8_t)(crc >> 8);
    return n;
}

int ax25_hdlc_nrzi(const uint8_t *frame, int len, int flags, uint8_t *tones)
{
    int nb = 0;
    int ones = 0;             /* consecutive 1s, for bit stuffing */
    int level = 1;            /* current NRZI tone (1 = mark) */

    #define EMIT(bit) do {                              \
        if (nb >= AX25_MAX_BITS) return 0;              \
        if ((bit) == 0) level ^= 1;   /* 0 -> toggle */ \
        tones[nb++] = (uint8_t)level;                   \
    } while (0)

    /* opening flags (not stuffed) */
    for (int f = 0; f < flags; f++)
        for (int b = 0; b < 8; b++)
            EMIT((0x7E >> b) & 1);        /* 0x7E LSB-first: 0 1 1 1 1 1 1 0 */

    /* body: LSB-first, bit-stuff after five 1s */
    for (int i = 0; i < len; i++) {
        for (int b = 0; b < 8; b++) {
            int bit = (frame[i] >> b) & 1;
            EMIT(bit);
            if (bit) {
                if (++ones == 5) { EMIT(0); ones = 0; }
            } else {
                ones = 0;
            }
        }
    }

    /* closing flags (a few, so a slow demod still sees a full one) */
    for (int f = 0; f < 3; f++)
        for (int b = 0; b < 8; b++)
            EMIT((0x7E >> b) & 1);

    #undef EMIT
    return nb;
}
