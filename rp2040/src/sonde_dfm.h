/*
 * sonde_dfm.h -- Graw DFM-06/09/17 radiosonde frame layer, Sarsat_UV-K1-5_RP2040.
 *
 * Ported from the reference decoder `demod/mod/dfm09mod.c` in
 * github.com/projecthorus/radiosonde_auto_rx (GPL-3.0, author zilog80).
 * This project relicensed from Apache-2.0 to GPL-3.0 on 2026-09-11
 * specifically to allow this port (explicit user decision) -- see
 * CREDITS.md and the project's radiosonde plan notes for the earlier,
 * unsuccessful attempts to reverse-engineer this frame purely from public
 * write-ups and independent SDR analysis of real captures (2500 Bd chip
 * rate, Manchester coding and the (8,4) Hamming/7-byte-interleave
 * description ARE independently public -- sigidwiki, r00t.cz -- but the
 * exact frame length, block boundaries, deinterleave permutation, Hamming
 * generator/parity matrices and field layout below are not published
 * anywhere non-GPL that could be found).
 *
 * Frame: 280 bits = 16-bit header (0x45CF, Manchester-coded on air) +
 * 56-bit CONF block (7 Hamming(8,4) codewords -- serial number/calibration,
 * not decoded here) + two 104-bit DATA blocks (13 codewords each -- GPS
 * position/time, split over several "sub-frame" IDs). A full GPS fix is
 * assembled from several consecutive 280-bit frames (each carries only 2 of
 * the 9 sub-frame IDs) -- see dfm_gps_t.
 */
#ifndef SONDE_DFM_H
#define SONDE_DFM_H

#include <stdbool.h>
#include <stdint.h>

#define DFM_FRAME_BITS   280
#define DFM_HEAD_BITS     16
#define DFM_CONF_BITS     56   /* L=7  codewords of 8 bits */
#define DFM_DAT_BITS     104   /* L=13 codewords of 8 bits */
#define DFM_HEADER       0x45CF   /* descrambled/Manchester-decoded header  */

/* Decode Manchester-coded chips (2 chips per data bit) into data bits.
 * `chips` holds `n_chips` raw 0/1 slices from sonde_demod at the DFM chip
 * rate (2500/s). A valid Manchester chip pair is "01" or "10"; `invert`
 * swaps which of the two means data-bit 0 vs 1 -- the reference decoder
 * documents DFM-06 and DFM-09/17 using OPPOSITE polarities (dfm09mod.c),
 * so a real receiver must try both.
 *
 * Writes decoded data bits to `bits_out` (up to n_chips/2 of them) and
 * returns the count. `*violations_out` (may be NULL) counts invalid
 * ("00"/"11") pairs encountered. */
int sonde_manchester_decode(const uint8_t *chips, int n_chips,
                            uint8_t *bits_out, bool invert,
                            int *violations_out);

/* Same Manchester decision as sonde_manchester_decode(), but NEVER skips a
 * pair: always writes exactly n_chips/2 bits, guessing `a` for an invalid
 * ("00"/"11") pair instead of dropping it. sonde_manchester_decode()'s
 * skip-and-shrink behaviour is right for DFM, which decodes one whole,
 * already-captured block at a time and just wants a clean pass/fail
 * (test_sonde_dfm.c's "violation counted" test relies on that exact
 * shrink). It is WRONG for a continuous, one-chip-at-a-time stream like
 * M10/M20's: skipping a pair shifts every bit AFTER it by one position for
 * the rest of the capture, silently destroying byte/checksum alignment for
 * the whole remainder of the frame from the very first transmission
 * hiccup -- found on real air (2026-09-11): M10/M20 GPS decode kept failing
 * even once the header/re-anchor logic was fixed, and offline replay of the
 * exact captured chips (`sonde_m10_hunt_t::raw_chips`) found the true M20
 * type marker (0x45 0x20) cleanly, briefly, right where expected -- but the
 * shrinking decode had already lost alignment by then. `violations_out`
 * still reports how many pairs were invalid, for diagnostics/plausibility,
 * but no longer changes the output length. */
int sonde_manchester_decode_lenient(const uint8_t *chips, int n_chips,
                                    uint8_t *bits_out, bool invert,
                                    int *violations_out);

/* Deinterleave one block of L*8 bits (L=7 for CONF, L=13 for DATA): the
 * transmitted stream is 8 "bit-planes" of L bits each (bit-plane r
 * contributes bit r of every one of the L codewords), which spreads a
 * short fade/burst error across many codewords instead of destroying one.
 * `in`/`out` are L*8 bits, one per byte (0/1). */
void sonde_dfm_deinterleave(const uint8_t *in, int L, uint8_t *out);

/* Extended (8,4) Hamming / SECDED code, DFM's exact (systematic, data-first)
 * bit arrangement -- NOT the generic textbook layout (parity bits do not
 * sit at positions 1/2/4 here, they follow all 4 data bits instead).
 * code8[0..3] = data bits d0..d3 (msg[0] is bit "0" of the nibble, i.e.
 * LSB-ish per dfm09mod.c's bits2val "big endian" convention used
 * elsewhere in the file -- data4 bit k maps to code8[k]);
 * code8[4..7] = parity (p0=d1^d2^d3, p1=d0^d2^d3, p2=d0^d1^d3, p3=d0^d1^d2). */
void sonde_hamming84_encode(uint8_t data4, uint8_t *code8 /* 8 bits, one per byte, 0/1 */);

/* Returns: 0 = no error, 1 = single-bit error corrected (data4_out valid),
 * 2 = uncorrectable (2-bit error detected, data4_out NOT valid). */
int sonde_hamming84_decode(const uint8_t *code8, uint8_t *data4_out);

/* ---- frame-level DFM decode ------------------------------------------- */

typedef struct {
    bool has_position;
    int32_t lat_e5, lon_e5;   /* degrees x 1e5, same convention as the rest
                               * of this project's position fields */
    int32_t alt_m;
    bool have_lat, have_lon, have_alt;   /* which sub-frames have arrived so
                                          * far this "GPS epoch" -- see
                                          * sonde_dfm_frame_t below */
} dfm_gps_t;

/* Accumulates a GPS fix across several consecutive DFM frames (each frame
 * only carries 2 of the 9 sub-frame IDs -- see dfm09mod.c's dat_out()).
 * Zero-initialise (or memset 0) before first use. */
typedef struct {
    dfm_gps_t gps;
} sonde_dfm_state_t;

void sonde_dfm_state_init(sonde_dfm_state_t *s);

/* Decode one 280-bit DFM frame (already Manchester-decoded to data bits,
 * MSB-first per byte, starting right after the 16-bit header) into the
 * running GPS accumulator. Returns true if this frame's decode updated
 * `s->gps` (a CRC/Hamming-valid DATA block was found) -- check
 * `s->gps.has_position` separately for whether a full fix (lat+lon, alt
 * optional) is now available. */
bool sonde_dfm_decode_frame(sonde_dfm_state_t *s, const uint8_t *frame_bits_280);

#endif /* SONDE_DFM_H */
