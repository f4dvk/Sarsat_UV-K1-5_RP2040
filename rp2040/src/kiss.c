/* KISS TNC framing (SLIP) -- see kiss.h. */
#include "kiss.h"

static int put_esc(uint8_t *out, int cap, int n, uint8_t b)
{
    if (b == KISS_FEND) {
        if (n + 2 > cap) return -1;
        out[n++] = KISS_FESC; out[n++] = KISS_TFEND;
    } else if (b == KISS_FESC) {
        if (n + 2 > cap) return -1;
        out[n++] = KISS_FESC; out[n++] = KISS_TFESC;
    } else {
        if (n + 1 > cap) return -1;
        out[n++] = b;
    }
    return n;
}

int kiss_encode(uint8_t *out, int cap, const uint8_t *frame, int len)
{
    if (len <= 0 || cap < 4)
        return 0;

    int n = 0;
    out[n++] = KISS_FEND;
    out[n++] = 0x00;                 /* port 0, command 0 = data */
    for (int i = 0; i < len; i++) {
        n = put_esc(out, cap, n, frame[i]);
        if (n < 0) return 0;
    }
    if (n + 1 > cap) return 0;
    out[n++] = KISS_FEND;
    return n;
}

int kiss_decode_byte(kiss_dec_t *k, uint8_t b)
{
    if (b == KISS_FEND) {
        int out = 0;
        if (k->in_frame && k->have_cmd && k->is_data && !k->overflow &&
            k->len > 0)
            out = k->len;
        /* start a fresh frame */
        k->in_frame = true;
        k->esc      = false;
        k->have_cmd = false;
        k->is_data  = false;
        k->overflow = false;
        k->len      = 0;
        return out;
    }

    if (!k->in_frame)
        return 0;

    if (k->esc) {
        k->esc = false;
        if (b == KISS_TFEND) b = KISS_FEND;
        else if (b == KISS_TFESC) b = KISS_FESC;
        /* an unknown byte after FESC -> keep it verbatim (lenient) */
    } else if (b == KISS_FESC) {
        k->esc = true;
        return 0;
    }

    if (!k->have_cmd) {
        k->have_cmd = true;
        k->is_data  = (b & 0x0F) == 0x00;   /* low nibble = command */
        return 0;
    }

    if (k->len >= KISS_MAX_FRAME) {
        k->overflow = true;
        return 0;
    }
    k->frame[k->len++] = b;
    return 0;
}
