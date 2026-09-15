/* test_sonde_dfm -- Manchester decode, (8,4) SECDED Hamming (DFM's exact
 * systematic bit layout), deinterleave, and full synthetic-frame decode --
 * see sonde_dfm.h. The frame-level logic (deinterleave permutation, Hamming
 * G/H matrices, fr_id GPS field layout) is a port of dfm09mod.c
 * (radiosonde_auto_rx, GPL-3.0) -- see sonde_dfm.h's header comment. No real
 * DFM capture was ever successfully bit-recovered by this project (see the
 * radiosonde plan notes), so these tests check the port's self-consistency
 * on synthetic vectors built the same way dfm09mod.c itself would encode
 * them, not an end-to-end decode of real captured audio.
 */
#include <stdio.h>
#include <string.h>

#include "sonde_dfm.h"

static int fails;

static void chk(const char *name, int cond)
{
    printf("  %-46s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

static void test_manchester(void)
{
    /* bits 0,1,1,0,1 -> chips "01","10","10","01","10" (invert=false) */
    uint8_t chips[] = { 0,1, 1,0, 1,0, 0,1, 1,0 };
    uint8_t bits[8];
    int viol = -1;
    int n = sonde_manchester_decode(chips, 10, bits, false, &viol);
    chk("manchester: count", n == 5);
    chk("manchester: no violations", viol == 0);
    chk("manchester: values", bits[0]==0 && bits[1]==1 && bits[2]==1 &&
                              bits[3]==0 && bits[4]==1);

    uint8_t bits_inv[8];
    sonde_manchester_decode(chips, 10, bits_inv, true, NULL);
    chk("manchester: invert flips all", bits_inv[0]==1 && bits_inv[1]==0 &&
                                        bits_inv[2]==0 && bits_inv[3]==1 && bits_inv[4]==0);

    uint8_t bad[] = { 0,1, 0,0, 1,0 };            /* middle pair is invalid */
    uint8_t out[4];
    n = sonde_manchester_decode(bad, 6, out, false, &viol);
    chk("manchester: violation counted", n == 2 && viol == 1);
}

static void test_manchester_lenient(void)
{
    /* same "middle pair is invalid" vector as above, but the lenient
     * variant must keep the slot instead of shrinking (see sonde_dfm.h) --
     * bits 0 and 2 (the valid pairs) must land at the SAME positions they
     * would in a clean 3-pair vector, not shift down by one. */
    uint8_t bad[] = { 0,1, 0,0, 1,0 };
    uint8_t out[3];
    int viol = -1;
    int n = sonde_manchester_decode_lenient(bad, 6, out, false, &viol);
    chk("manchester lenient: keeps all 3 slots", n == 3);
    chk("manchester lenient: violation still counted", viol == 1);
    chk("manchester lenient: valid pairs at their original positions",
        out[0] == 0 && out[2] == 1);

    /* fully clean vector: lenient must agree with the strict decoder when
     * there's nothing to disagree about. */
    uint8_t clean[] = { 0,1, 1,0, 1,0, 0,1, 1,0 };
    uint8_t out2[5];
    n = sonde_manchester_decode_lenient(clean, 10, out2, false, &viol);
    chk("manchester lenient: clean vector matches strict decode",
        n == 5 && viol == 0 && out2[0]==0 && out2[1]==1 && out2[2]==1 &&
        out2[3]==0 && out2[4]==1);
}

static void test_hamming_clean(void)
{
    int all_ok = 1;
    for (int d = 0; d < 16; d++) {
        uint8_t code[8];
        sonde_hamming84_encode((uint8_t)d, code);
        uint8_t got;
        int status = sonde_hamming84_decode(code, &got);
        if (status != 0 || got != d) all_ok = 0;
    }
    chk("hamming84: clean round-trip (all 16 values)", all_ok);
}

static void test_hamming_single_bit(void)
{
    int all_ok = 1;
    for (int d = 0; d < 16 && all_ok; d++) {
        for (int b = 0; b < 8; b++) {
            uint8_t code[8];
            sonde_hamming84_encode((uint8_t)d, code);
            code[b] ^= 1u;
            uint8_t got;
            int status = sonde_hamming84_decode(code, &got);
            if (status != 1 || got != d) { all_ok = 0; break; }
        }
    }
    chk("hamming84: every 1-bit error corrected (16x8 cases)", all_ok);
}

static void test_hamming_double_bit(void)
{
    /* every 2-bit error must be DETECTED (status==2), never mis-corrected
     * to a wrong value with status==0 or 1 -- the SECDED guarantee. */
    int all_detected = 1;
    for (int d = 0; d < 16 && all_detected; d++) {
        for (int b1 = 0; b1 < 8 && all_detected; b1++) {
            for (int b2 = b1 + 1; b2 < 8; b2++) {
                uint8_t code[8];
                sonde_hamming84_encode((uint8_t)d, code);
                code[b1] ^= 1u;
                code[b2] ^= 1u;
                uint8_t got;
                int status = sonde_hamming84_decode(code, &got);
                if (status != 2) { all_detected = 0; break; }
            }
        }
    }
    chk("hamming84: every 2-bit error detected, not mis-corrected (16x28 cases)",
        all_detected);
}

static void test_deinterleave(void)
{
    /* L=7 (CONF-sized): in[L*j+i] should land at out[8*i+j] */
    uint8_t in[7 * 8], out[7 * 8];
    for (int k = 0; k < 7 * 8; k++) in[k] = (uint8_t)(k & 1);   /* distinguishable pattern */
    sonde_dfm_deinterleave(in, 7, out);
    int ok = 1;
    for (int j = 0; j < 8 && ok; j++)
        for (int i = 0; i < 7 && ok; i++)
            if (out[8 * i + j] != in[7 * j + i]) ok = 0;
    chk("deinterleave: L=7 matches formula", ok);

    /* round-trip via the inverse mapping (interleave then deinterleave) */
    uint8_t codewords[13 * 8], transmitted[13 * 8], back[13 * 8];
    for (int k = 0; k < 13 * 8; k++) codewords[k] = (uint8_t)((k * 37 + 5) & 1);
    for (int i = 0; i < 13; i++)
        for (int j = 0; j < 8; j++)
            transmitted[13 * j + i] = codewords[8 * i + j];
    sonde_dfm_deinterleave(transmitted, 13, back);
    chk("deinterleave: L=13 round-trips through its own inverse",
        memcmp(codewords, back, sizeof codewords) == 0);
}

/* Build one 104-bit DATA block (interleaved/transmitted form) carrying a
 * given fr_id and a 32-bit payload value in bits 0..31 (bits 32..47 left at
 * 0, matching dfm09mod.c's dat_out() field layout for fr_id 2/3/4 -- lat/
 * lon/alt each occupy the first 32 bits of their sub-frame, see
 * sonde_dfm.h). */
static void build_dat_block(int fr_id, uint32_t val32, uint8_t *transmitted_104)
{
    uint8_t databits[52];
    memset(databits, 0, sizeof databits);
    for (int i = 0; i < 32; i++)
        databits[i] = (uint8_t)((val32 >> (31 - i)) & 1u);   /* MSB-first */
    for (int i = 0; i < 4; i++)
        databits[48 + i] = (uint8_t)((fr_id >> (3 - i)) & 1u);

    uint8_t codewords[13 * 8];
    for (int i = 0; i < 13; i++) {
        /* sonde_hamming84_encode()'s data4 bit j (C LSB-first) is what ends
         * up at sym[4*i+j] after decode -- so nibble semantic position j
         * (0=first/MSB-ish per bits2val) must land at data4's bit j too. */
        uint8_t nib = (uint8_t)(databits[4*i] | (databits[4*i+1]<<1) |
                                (databits[4*i+2]<<2) | (databits[4*i+3]<<3));
        sonde_hamming84_encode(nib, codewords + 8 * i);
    }
    for (int i = 0; i < 13; i++)
        for (int j = 0; j < 8; j++)
            transmitted_104[13 * j + i] = codewords[8 * i + j];
}

static void test_frame_decode_gps(void)
{
    /* Paris-ish: 48.8566 N, 2.3522 E, 5000 m -- encoded exactly the way
     * dfm09mod.c's dat_out() reads them back (int32, scale 1e7 for lat/lon,
     * 1e2 for alt in cm). */
    int32_t lat_raw = (int32_t)(48.8566 * 1e7);
    int32_t lon_raw = (int32_t)(2.3522 * 1e7);
    int32_t alt_raw = (int32_t)(5000.0 * 100.0);   /* cm */

    uint8_t frame[DFM_FRAME_BITS];
    memset(frame, 0, sizeof frame);   /* header + CONF: content irrelevant here */

    build_dat_block(2, (uint32_t)lat_raw, frame + DFM_HEAD_BITS + DFM_CONF_BITS);
    build_dat_block(3, (uint32_t)lon_raw, frame + DFM_HEAD_BITS + DFM_CONF_BITS + DFM_DAT_BITS);

    sonde_dfm_state_t st;
    sonde_dfm_state_init(&st);
    bool updated = sonde_dfm_decode_frame(&st, frame);

    chk("dfm frame: decode reports an update", updated);
    chk("dfm frame: has_position", st.gps.has_position);
    if (st.gps.has_position) {
        chk("dfm frame: lat within 0.0001 deg",
            st.gps.lat_e5 > 4885560 && st.gps.lat_e5 <= 4885660);
        chk("dfm frame: lon within 0.0001 deg",
            st.gps.lon_e5 > 235120 && st.gps.lon_e5 < 235320);
    } else {
        chk("dfm frame: lat within 0.0001 deg", 0);
        chk("dfm frame: lon within 0.0001 deg", 0);
    }

    /* a 3rd frame (altitude only, fr_id 4) folded into the SAME accumulator
     * should add alt without disturbing the already-latched lat/lon. */
    uint8_t frame2[DFM_FRAME_BITS];
    memset(frame2, 0, sizeof frame2);
    build_dat_block(4, (uint32_t)alt_raw, frame2 + DFM_HEAD_BITS + DFM_CONF_BITS);
    sonde_dfm_decode_frame(&st, frame2);
    chk("dfm frame: alt accumulates across frames",
        st.gps.have_alt && st.gps.alt_m > 4990 && st.gps.alt_m < 5010);
    chk("dfm frame: lat/lon preserved across frames",
        st.gps.lat_e5 > 4885560 && st.gps.lat_e5 <= 4885660);
}

int main(void)
{
    test_manchester();
    test_manchester_lenient();
    test_hamming_clean();
    test_hamming_single_bit();
    test_hamming_double_bit();
    test_deinterleave();
    test_frame_decode_gps();
    printf(fails ? "\nFAIL (%d)\n" : "\nall pass\n", fails);
    return fails ? 1 : 0;
}
