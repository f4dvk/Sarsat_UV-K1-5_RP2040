/* sonde_dfm.c -- see sonde_dfm.h. Pure C, no pico-sdk, host-testable.
 * Frame-level logic ported from dfm09mod.c (radiosonde_auto_rx, GPL-3.0) --
 * see sonde_dfm.h's header comment.
 */

#include "sonde_dfm.h"

#include <string.h>

int sonde_manchester_decode(const uint8_t *chips, int n_chips,
                            uint8_t *bits_out, bool invert,
                            int *violations_out)
{
    int n_pairs = n_chips / 2;
    int out = 0, viol = 0;
    for (int i = 0; i < n_pairs; i++) {
        uint8_t a = chips[2 * i], b = chips[2 * i + 1];
        if (a == b) {                 /* "00" / "11": not a valid transition */
            viol++;
            continue;
        }
        uint8_t bit = (uint8_t)((a == 0 && b == 1) ? 0 : 1);   /* "01"->0, "10"->1 */
        if (invert)
            bit ^= 1u;
        bits_out[out++] = bit;
    }
    if (violations_out)
        *violations_out = viol;
    return out;
}

int sonde_manchester_decode_lenient(const uint8_t *chips, int n_chips,
                                    uint8_t *bits_out, bool invert,
                                    int *violations_out)
{
    int n_pairs = n_chips / 2;
    int viol = 0;
    for (int i = 0; i < n_pairs; i++) {
        uint8_t a = chips[2 * i], b = chips[2 * i + 1];
        uint8_t bit;
        if (a == b) {
            viol++;
            bit = a;   /* no transition to read -- best-effort guess, keeps
                        * this pair's slot instead of shifting everything
                        * after it (see sonde_dfm.h's note on this function) */
        } else {
            bit = (uint8_t)((a == 0 && b == 1) ? 0 : 1);
        }
        if (invert)
            bit ^= 1u;
        bits_out[i] = bit;
    }
    if (violations_out)
        *violations_out = viol;
    return n_pairs;
}

void sonde_dfm_deinterleave(const uint8_t *in, int L, uint8_t *out)
{
    /* dfm09mod.c: block[B*i+j] = str[L*j+i], B=8 */
    for (int j = 0; j < 8; j++)
        for (int i = 0; i < L; i++)
            out[8 * i + j] = in[L * j + i];
}

/* G (8x4 generator) / H (4x8 parity-check), dfm09mod.c's exact matrices --
 * a systematic (data-first) code, NOT the textbook parity-interleaved one. */
static const uint8_t DFM_G[8][4] = {
    {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1},
    {0,1,1,1}, {1,0,1,1}, {1,1,0,1}, {1,1,1,0},
};
static const uint8_t DFM_H[4][8] = {
    {0,1,1,1,1,0,0,0},
    {1,0,1,1,0,1,0,0},
    {1,1,0,1,0,0,1,0},
    {1,1,1,0,0,0,0,1},
};
/* 1-bit-error syndromes, indexed by error position (0-7) -- dfm09mod.c's He[] */
static const uint8_t DFM_HE[8] = { 0x7, 0xB, 0xD, 0xE, 0x8, 0x4, 0x2, 0x1 };

void sonde_hamming84_encode(uint8_t data4, uint8_t *code8)
{
    for (int i = 0; i < 8; i++) {
        uint8_t v = 0;
        for (int j = 0; j < 4; j++)
            v ^= (uint8_t)(DFM_G[i][j] & ((data4 >> j) & 1u));
        code8[i] = v;
    }
}

int sonde_hamming84_decode(const uint8_t *code8, uint8_t *data4_out)
{
    uint8_t syn[4];
    for (int i = 0; i < 4; i++) {
        uint8_t v = 0;
        for (int j = 0; j < 8; j++)
            v ^= (uint8_t)(DFM_H[i][j] & code8[j]);
        syn[i] = v;
    }
    uint8_t synval = (uint8_t)((syn[0] << 3) | (syn[1] << 2) | (syn[2] << 1) | syn[3]);

    uint8_t fixed[8];
    memcpy(fixed, code8, 8);
    int status = 0;
    if (synval != 0) {
        int pos = -1;
        for (int j = 0; j < 8; j++)
            if (synval == DFM_HE[j]) { pos = j; break; }
        if (pos >= 0) {
            fixed[pos] ^= 1u;
            status = 1;
        } else {
            return 2;   /* uncorrectable (2-bit error), data4_out not valid */
        }
    }
    if (data4_out)
        *data4_out = (uint8_t)(fixed[0] | (fixed[1] << 1) | (fixed[2] << 2) | (fixed[3] << 3));
    return status;
}

/* MSB-first bit run -> unsigned value, dfm09mod.c's bits2val() convention */
static uint32_t bits2val(const uint8_t *bits, int len)
{
    uint32_t v = 0;
    for (int i = 0; i < len; i++)
        v |= ((uint32_t)(bits[i] & 1u)) << (len - 1 - i);
    return v;
}

void sonde_dfm_state_init(sonde_dfm_state_t *s)
{
    memset(s, 0, sizeof *s);
}

/* Decode one 56-bit (CONF, L=7) or 104-bit (DATA, L=13) block: deinterleave,
 * Hamming-decode each of the L codewords, and pack the L systematic data
 * nibbles into a flat L*4-bit array (MSB-first per nibble, matching
 * dfm09mod.c's hamming()/sym[] layout). Returns the number of codewords
 * whose Hamming decode was uncorrectable (0 = clean or single-bit-fixed
 * throughout). */
static int decode_block(const uint8_t *frame_bits, int off, int L, uint8_t *data_bits_out)
{
    uint8_t deinterleaved[13 * 8];
    sonde_dfm_deinterleave(frame_bits + off, L, deinterleaved);

    int bad = 0;
    for (int i = 0; i < L; i++) {
        uint8_t nib;
        int st = sonde_hamming84_decode(deinterleaved + 8 * i, &nib);
        if (st == 2) { bad++; nib = 0; }
        for (int j = 0; j < 4; j++)
            data_bits_out[4 * i + j] = (uint8_t)((nib >> j) & 1u);
    }
    return bad;
}

/* dat_bits: 52 individual bits (13 nibbles, MSB-first each -- see
 * decode_block). fr_id lives in the last nibble (bits 48..51). Only the
 * "posmode<=2" (default/most common) sub-frame layout is implemented --
 * see dfm09mod.c's dat_out() for the posmode 3/4 (XDATA) variants, not
 * needed for a basic position fix. */
static void parse_data_block(dfm_gps_t *gps, const uint8_t *dat_bits)
{
    int fr_id = (int)bits2val(dat_bits + 48, 4);

    if (fr_id == 2) {
        int32_t lat_raw = (int32_t)bits2val(dat_bits, 32);
        gps->lat_e5 = (int32_t)((double)lat_raw / 1e7 * 100000.0);
        gps->have_lat = true;
    } else if (fr_id == 3) {
        int32_t lon_raw = (int32_t)bits2val(dat_bits, 32);
        gps->lon_e5 = (int32_t)((double)lon_raw / 1e7 * 100000.0);
        gps->have_lon = true;
    } else if (fr_id == 4) {
        int32_t alt_raw = (int32_t)bits2val(dat_bits, 32);   /* cm */
        gps->alt_m = alt_raw / 100;
        gps->have_alt = true;
    }

    if (gps->have_lat && gps->have_lon)
        gps->has_position = true;
}

bool sonde_dfm_decode_frame(sonde_dfm_state_t *s, const uint8_t *frame_bits_280)
{
    uint8_t conf_data[7 * 4];
    uint8_t dat1_data[13 * 4];
    uint8_t dat2_data[13 * 4];

    decode_block(frame_bits_280, DFM_HEAD_BITS, 7, conf_data);   /* CONF: not
                                                                  * decoded
                                                                  * further
                                                                  * here (SN/
                                                                  * calib) */
    int bad1 = decode_block(frame_bits_280, DFM_HEAD_BITS + DFM_CONF_BITS, 13, dat1_data);
    int bad2 = decode_block(frame_bits_280, DFM_HEAD_BITS + DFM_CONF_BITS + DFM_DAT_BITS,
                            13, dat2_data);

    bool updated = false;
    if (bad1 == 0) { parse_data_block(&s->gps, dat1_data); updated = true; }
    if (bad2 == 0) { parse_data_block(&s->gps, dat2_data); updated = true; }

    (void)conf_data;
    return updated;
}
