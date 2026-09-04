/* quansheng_frame.c — see quansheng_frame.h. */
#include "quansheng_frame.h"
#include <string.h>

static const uint8_t kObf[16] = {
    0x16, 0x6C, 0x14, 0xE6, 0x2E, 0x91, 0x0D, 0x40,
    0x21, 0x35, 0xD5, 0x40, 0x13, 0x03, 0xE9, 0x80
};

uint16_t quansheng_crc16_xmodem(const uint8_t *p, size_t n)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t quansheng_frame_build(uint16_t cmd_id, const uint8_t *data, size_t data_len,
                             uint8_t *out, size_t out_cap)
{
    if (data_len > 240) return 0;

    const size_t pay_len = 4 + data_len;          /* inner header + data */
    const size_t obf_len = pay_len + 2;           /* + crc */
    const size_t total   = 4 + obf_len + 2;       /* AB CD .. + DC BA */
    if (out_cap < total) return 0;

    uint8_t inner[4 + 240 + 2];
    inner[0] = (uint8_t)(cmd_id & 0xFF);
    inner[1] = (uint8_t)(cmd_id >> 8);
    inner[2] = (uint8_t)(data_len & 0xFF);
    inner[3] = (uint8_t)(data_len >> 8);
    if (data && data_len) memcpy(inner + 4, data, data_len);

    uint16_t crc = quansheng_crc16_xmodem(inner, pay_len);
    inner[pay_len]     = (uint8_t)(crc & 0xFF);
    inner[pay_len + 1] = (uint8_t)(crc >> 8);

    for (size_t i = 0; i < obf_len; i++) inner[i] ^= kObf[i & 15];

    size_t o = 0;
    out[o++] = 0xAB;
    out[o++] = 0xCD;
    out[o++] = (uint8_t)(pay_len & 0xFF);
    out[o++] = (uint8_t)(pay_len >> 8);
    memcpy(out + o, inner, obf_len);
    o += obf_len;
    out[o++] = 0xDC;
    out[o++] = 0xBA;
    return o;
}

/* ---- streaming receiver -------------------------------------------------- */
/*
 * Wire format (both directions), from egzumer/KD8CEC App/app/uart.c:
 *   AB CD | inner_size:u16 LE | <masked payload> | pad:2 | DC BA
 * where the masked payload is (inner_size) bytes = [id:u16][dlen:u16][data...]
 * XOR'd with kObf. Replies use pad = kObf[..]^0xFF instead of a CRC.
 */
#define QF_DROP1(rx) do { memmove((rx)->buf, (rx)->buf + 1, --(rx)->len); } while (0)

int quansheng_frame_feed(qframe_rx_t *rx, uint8_t b,
                         uint16_t *id, const uint8_t **data, uint16_t *data_len)
{
    if (rx->len >= sizeof(rx->buf))
        rx->len = sizeof(rx->buf) - 1;   /* shouldn't happen; keep newest byte */
    rx->buf[rx->len++] = b;

    /* try to lock a valid frame at buf[0], shifting one byte on any mismatch */
    for (;;) {
        while (rx->len >= 1 && rx->buf[0] != 0xAB) QF_DROP1(rx);
        if (rx->len < 4)
            return 0;
        if (rx->buf[1] != 0xCD) { QF_DROP1(rx); continue; }

        uint16_t isize = rx->buf[2] | ((uint16_t)rx->buf[3] << 8);
        if (isize < 4 || isize > sizeof(rx->buf) - 8) { QF_DROP1(rx); continue; }

        uint16_t total = 4 + isize + 2 + 2;   /* hdr + payload + pad + footer */
        if (rx->len < total)
            return 0;                         /* wait for the rest */

        if (rx->buf[total - 2] != 0xDC || rx->buf[total - 1] != 0xBA) {
            QF_DROP1(rx);
            continue;
        }

        uint16_t n = isize;
        if (n > sizeof(rx->payload)) n = sizeof(rx->payload);
        for (uint16_t i = 0; i < n; i++)
            rx->payload[i] = rx->buf[4 + i] ^ kObf[i & 15];
        *id = rx->payload[0] | ((uint16_t)rx->payload[1] << 8);
        uint16_t dl = rx->payload[2] | ((uint16_t)rx->payload[3] << 8);
        if (dl > (uint16_t)(n - 4)) dl = n - 4;
        *data     = rx->payload + 4;
        *data_len = dl;

        uint16_t rest = rx->len - total;
        memmove(rx->buf, rx->buf + total, rest);
        rx->len = rest;
        return 1;
    }
}
