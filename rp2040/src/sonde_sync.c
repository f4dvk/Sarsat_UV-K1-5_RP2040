/* sonde_sync.c -- see sonde_sync.h. Pure C, no pico-sdk, host-testable. */

#include "sonde_sync.h"
#include "sonde_rs41.h"

#include <string.h>

#define SONDE_ST_HUNT       0
#define SONDE_ST_RS41_FRAME 1

#define RS41_HDR_BITS 64        /* RS41_RAW_SYNC, 8 bytes, MSB-first per byte */
#define M10_HDR_BITS  28        /* "5A 4A 93 _A", 3 bytes + a nibble, LSB-first
                                 * per byte -- see sonde_sync.h */
#define RS41_MAX_ERR   3        /* out of 64 bits (~4.7%) */
#define M10_MAX_ERR    2        /* out of 28 bits (~7.1%) */

/* Simulate shifting `bytes` into the stream bit-by-bit, in transmission
 * order, the same way sonde_sync_feed() itself accumulates s->rs41_sr /
 * s->m10_sr -- so the result is directly comparable to those registers.
 * `lsb_first` selects which end of each byte is transmitted first. */
static uint64_t expand_bits(const uint8_t *bytes, int total_bits, bool lsb_first)
{
    uint64_t v = 0;
    int done = 0;
    for (int bi = 0; done < total_bits; bi++) {
        uint8_t b = bytes[bi];
        for (int k = 0; k < 8 && done < total_bits; k++) {
            int bit = lsb_first ? ((b >> k) & 1) : ((b >> (7 - k)) & 1);
            v = (v << 1) | (uint64_t)bit;
            done++;
        }
    }
    return v;
}

static int popcount64(uint64_t v)
{
#ifdef __GNUC__
    return __builtin_popcountll(v);
#else
    int n = 0;
    while (v) { n += (int)(v & 1); v >>= 1; }
    return n;
#endif
}

void sonde_sync_init(sonde_sync_t *s)
{
    memset(s, 0, sizeof *s);
    s->state = SONDE_ST_HUNT;

    s->rs41_pattern = expand_bits(RS41_RAW_SYNC, RS41_HDR_BITS, false);

    static const uint8_t m10_hdr[4] = { 0x5A, 0x4A, 0x93, 0x0A };
    s->m10_pattern = expand_bits(m10_hdr, M10_HDR_BITS, true);
    s->m10_mask    = (M10_HDR_BITS >= 64) ? ~(uint64_t)0
                                          : (((uint64_t)1 << M10_HDR_BITS) - 1);
}

static void start_rs41_frame(sonde_sync_t *s)
{
    s->state     = SONDE_ST_RS41_FRAME;
    s->frame_len = 0;
    s->cur_byte  = 0;
    s->cur_bits  = 0;
}

static sonde_evt_t finish_rs41_frame(sonde_sync_t *s, sonde_rs41_result_t *out)
{
    s->state   = SONDE_ST_HUNT;
    s->rs41_sr = 0;
    s->m10_sr  = 0;

    uint8_t work[SONDE_RS41_FRAME_CAP];
    memcpy(work, s->frame, SONDE_RS41_FRAME_CAP);
    rs41_descramble(work, SONDE_RS41_FRAME_CAP, 0);

    rs41_block_t blocks[16];
    int n = rs41_parse_blocks(work, SONDE_RS41_FRAME_CAP, blocks, 16);

    if (out) {
        memset(out, 0, sizeof *out);
        out->n_blocks = n;
        for (int i = 0; i < n; i++) {
            if (blocks[i].id != RS41_GPSPOS_ID)
                continue;
            rs41_gpspos_t gp;
            if (rs41_parse_gpspos(blocks[i].data, blocks[i].len, &gp)) {
                rs41_ecef_to_geo((double)gp.ecef_x_cm / 100.0,
                                 (double)gp.ecef_y_cm / 100.0,
                                 (double)gp.ecef_z_cm / 100.0,
                                 &out->lat_e5, &out->lon_e5, &out->alt_m);
                out->has_position = true;
            }
            break;
        }
    }
    return SONDE_EVT_RS41;
}

sonde_evt_t sonde_sync_feed(sonde_sync_t *s, uint8_t bit, sonde_rs41_result_t *rs41_out)
{
    if (s->state == SONDE_ST_HUNT) {
        s->rs41_sr = (s->rs41_sr << 1) | (bit & 1u);
        s->m10_sr  = (s->m10_sr  << 1) | (bit & 1u);

        if (popcount64(s->rs41_sr ^ s->rs41_pattern) <= RS41_MAX_ERR) {
            start_rs41_frame(s);
            return SONDE_EVT_NONE;
        }
        if (popcount64((s->m10_sr & s->m10_mask) ^ s->m10_pattern) <= M10_MAX_ERR) {
            s->m10_sr = 0;      /* don't re-match the same tail next bit */
            return SONDE_EVT_M10;
        }
        return SONDE_EVT_NONE;
    }

    /* SONDE_ST_RS41_FRAME: pack post-sync bits MSB-first into frame bytes */
    s->cur_byte = (uint8_t)((s->cur_byte << 1) | (bit & 1u));
    if (++s->cur_bits == 8) {
        if (s->frame_len < SONDE_RS41_FRAME_CAP)
            s->frame[s->frame_len++] = s->cur_byte;
        s->cur_byte = 0;
        s->cur_bits = 0;
    }
    if (s->frame_len >= SONDE_RS41_FRAME_CAP)
        return finish_rs41_frame(s, rs41_out);
    return SONDE_EVT_NONE;
}
