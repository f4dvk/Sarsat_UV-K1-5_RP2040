/* sonde_m10.c -- see sonde_m10.h. Pure C, no pico-sdk, host-testable.
 * Ported from m10mod.c/m20mod.c (radiosonde_auto_rx, GPL-3.0), cross-checked
 * against HTCommander's independent Dart port -- see sonde_m10.h's header
 * comment for the Manchester-decode fix this cross-check produced.
 */

#include "sonde_m10.h"

#include <string.h>

#define M10_SLICE_MIN 8   /* same floor as sonde_demod.c's SONDE_SLICE_MIN */

void sonde_m10_chipdemod_init(sonde_m10_chipdemod_t *d, int sample_rate_hz, int chip_baud)
{
    memset(d, 0, sizeof *d);
    d->pll_step = (uint32_t)(((uint64_t)1 << 32) * (uint32_t)chip_baud /
                             (uint32_t)sample_rate_hz);
}

bool sonde_m10_chipdemod_sample(sonde_m10_chipdemod_t *d, int32_t sample,
                                uint8_t *chip_out, int32_t *cell_avg_out)
{
    /* dead-band slicer, identical to sonde_demod.c -- feeds the PLL nudge
     * only, not the chip decision (see sonde_m10.h's header note). */
    d->lp += (sample - d->lp) >> 2;
    if (d->lp > d->lp_hi) d->lp_hi += (d->lp - d->lp_hi) >> 2;
    else                  d->lp_hi -= (d->lp_hi - d->lp_lo) >> 10;
    if (d->lp < d->lp_lo) d->lp_lo += (d->lp - d->lp_lo) >> 2;
    else                  d->lp_lo += (d->lp_hi - d->lp_lo) >> 10;
    int32_t lpc = d->lp - ((d->lp_hi + d->lp_lo) >> 1);
    int32_t mag = (lpc < 0) ? -lpc : lpc;
    d->env += (mag - d->env) >> 6;
    int32_t th = d->env >> 2;
    if (th < M10_SLICE_MIN) th = M10_SLICE_MIN;
    if (lpc > th)      d->bit = 0;
    else if (lpc < -th) d->bit = 1;

    /* integrate-and-dump: accumulate the RAW sample across the WHOLE chip
     * cell -- see sonde_m10.h's struct comment for why this is a whole-cell
     * sum, not a half/half comparison (a single chip's amplitude doesn't
     * change within itself; averaging over the whole cell is what actually
     * rejects noise, roughly by sqrt(samples-per-chip) over a single point
     * sample). NOT DC-corrected here any more -- the payload's own decode
     * (sonde_m10_bits_from_cells()) compares adjacent cells to each other,
     * which cancels any DC bias by construction; DC-correcting this sum too
     * would just add the dead-band slicer's OWN separately-drifting
     * baseline error into both sides of that comparison for nothing. */
    d->cell_sum += sample;
    d->cell_n   += 1;

    bool emitted = false;
    int32_t prev_sign = (int32_t)d->pll >> 31;
    d->pll += d->pll_step;
    if (((int32_t)d->pll >> 31) > prev_sign) {
        /* full PLL wrap: chip cell complete. `chip_out` keeps the
         * DC-baseline decision (header correlation only, see sonde_m10.h);
         * `cell_avg_out` is the real per-sample average this cell saw, for
         * the payload's self-referential decode. */
        int32_t avg = (int32_t)(d->cell_sum / (d->cell_n ? d->cell_n : 1));
        *chip_out = (avg >= (d->lp_hi + d->lp_lo) / 2) ? 0u : 1u;
        if (cell_avg_out) *cell_avg_out = avg;
        d->cell_sum = 0;
        d->cell_n   = 0;
        emitted = true;
    }
    if (d->bit != d->pval) {
        d->pll -= (uint32_t)((int32_t)d->pll >> 2);
        d->pval = d->bit;
    }
    return emitted;
}

int sonde_m10_bits_from_cells(const int32_t *cells, int n_cells, uint8_t *bits_out, bool invert)
{
    int n_pairs = n_cells / 2;
    for (int i = 0; i < n_pairs; i++) {
        uint8_t bit = (cells[2 * i + 1] > cells[2 * i]) ? 1u : 0u;
        if (invert)
            bit ^= 1u;
        bits_out[i] = bit;
    }
    return n_pairs;
}

/* "10011001100110010100110010011001", m10mod.c/m20mod.c's rawheader[] --
 * identical in both files, expressed at the CHIP level (see sonde_m10.h). */
const uint8_t M10_RAWHEADER_BITS[M10_HEADER_CHIPS] = {
    1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,
    0,1,0,0,1,1,0,0,1,0,0,1,1,0,0,1,
};

void sonde_m10_diff_decode(const uint8_t *raw_bits, int n, uint8_t *out_bits)
{
    /* m10mod.c/m20mod.c: frame_bits[pos] = 0x31 ^ (bit0 ^ bit), stored as
     * ASCII '0'/'1'; bit0 starts at 0x30 ('0'), then becomes the previous
     * raw bit (0/1). Decoded back to a plain 0/1 here (cross-checked
     * against HTCommander's `out[i] = raw[i]==prev ? 1 : 0`, identical):
     *   i==0:  out = NOT(raw[0])                       (bit0 == 0x30 case)
     *   i>=1:  out = NOT(raw[i] XOR raw[i-1])           (bit0 == 0/1 case)
     */
    uint8_t prev_raw = 0;
    for (int i = 0; i < n; i++) {
        uint8_t r = raw_bits[i] & 1u;
        if (i == 0)
            out_bits[i] = (uint8_t)(1u - r);
        else
            out_bits[i] = (uint8_t)(1u - (r ^ prev_raw));
        prev_raw = r;
    }
}

#define POS_GPSLAT 0x1C
#define POS_GPSLON 0x20
#define POS_GPSALT 0x08
#define M20_B60B60 1e6

static int32_t be_i32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
}

/* M20Decoder._update(), a 16-bit shift-register checksum -- ported bit for
 * bit, see sonde_m10.h. */
static uint16_t m20_checksum_update(uint16_t c, uint8_t b)
{
    uint8_t c1 = (uint8_t)(c & 0xFFu);
    b = (uint8_t)(((b >> 1) | ((b & 1u) << 7)) & 0xFFu);
    b = (uint8_t)(b ^ ((b >> 2) & 0xFFu));
    uint8_t t6 = (uint8_t)((c & 1u) ^ ((c >> 2) & 1u) ^ ((c >> 4) & 1u));
    uint8_t t7 = (uint8_t)(((c >> 1) & 1u) ^ ((c >> 3) & 1u) ^ ((c >> 5) & 1u));
    uint8_t t = (uint8_t)((c & 0x3Fu) | (uint8_t)(t6 << 6) | (uint8_t)(t7 << 7));
    uint8_t s = (uint8_t)((c >> 7) & 0xFFu);
    s = (uint8_t)(s ^ ((s >> 2) & 0xFFu));
    uint8_t c0 = (uint8_t)(b ^ t ^ s);
    return (uint16_t)(((uint16_t)c1 << 8) | c0);
}

uint16_t sonde_m20_checksum(const uint8_t *frame_bytes, int len)
{
    uint16_t c = 0;
    for (int i = 0; i < len; i++)
        c = m20_checksum_update(c, frame_bytes[i]);
    return c;
}

bool sonde_m20_parse_gps(const uint8_t *frame_bytes, int len, m10_gps_t *out)
{
    memset(out, 0, sizeof *out);
    if (len < M20_FRAME_BYTES)
        return false;

    /* validity gate: type marker + checksum -- see sonde_m10.h. Far
     * stronger than the first port's bare lat/lon-range plausibility check. */
    if (frame_bytes[0] != 0x45 || frame_bytes[1] != 0x20)
        return true;   /* not a (recognisable) M20 frame -- has_position stays false */
    uint16_t want = (uint16_t)((frame_bytes[M20_POS_CHECK] << 8) |
                               frame_bytes[M20_POS_CHECK + 1]);
    if (sonde_m20_checksum(frame_bytes, M20_POS_CHECK) != want)
        return true;   /* checksum mismatch -- has_position stays false */

    int32_t lat_raw = be_i32(frame_bytes + POS_GPSLAT);
    int32_t lon_raw = be_i32(frame_bytes + POS_GPSLON);
    double lat = (double)lat_raw / M20_B60B60;
    double lon = (double)lon_raw / M20_B60B60;
    int32_t alt_raw = (int32_t)(((uint32_t)frame_bytes[POS_GPSALT] << 16) |
                                ((uint32_t)frame_bytes[POS_GPSALT + 1] << 8) |
                                (uint32_t)frame_bytes[POS_GPSALT + 2]);

    out->lat_e5 = (int32_t)(lat * 100000.0);
    out->lon_e5 = (int32_t)(lon * 100000.0);
    out->alt_m  = alt_raw / 100;   /* M20Decoder: alt = raw/100.0 (cm) */

    /* checksum already validated the frame -- lat==lon==0 here means a
     * genuine "no fix yet" sentinel, not noise (see M20Decoder.decodeFrame). */
    out->has_position = !(lat == 0.0 && lon == 0.0) &&
                        lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 &&
                        out->alt_m >= -1000 && out->alt_m <= 80000;
    return true;
}

/* 2^32/360, m10mod.c/M10Decoder's BAM (Binary Angular Measurement) scale for
 * the Trimble GPS packet's lat/lon fields -- see sonde_m10.h. */
#define M10_BAM_SCALE (1073741824.0 / 90.0)

bool sonde_m10_parse_gps(const uint8_t *frame_bytes, int len, m10_gps_t *out)
{
    memset(out, 0, sizeof *out);
    if (len < M10_FRAME_BYTES)
        return false;

    if (frame_bytes[0] != M10_TYPE0 || frame_bytes[1] != M10_TYPE1)
        return true;   /* not a (recognisable) M10 frame -- has_position stays false */
    uint16_t want = (uint16_t)((frame_bytes[M10_POS_CHECK] << 8) |
                               frame_bytes[M10_POS_CHECK + 1]);
    if (sonde_m20_checksum(frame_bytes, M10_POS_CHECK) != want)   /* same algorithm, see sonde_m10.h */
        return true;   /* checksum mismatch -- has_position stays false */

    int32_t lat_raw = be_i32(frame_bytes + M10_POS_LAT);
    int32_t lon_raw = be_i32(frame_bytes + M10_POS_LON);
    int32_t alt_raw = be_i32(frame_bytes + M10_POS_ALT);
    double lat = (double)lat_raw / M10_BAM_SCALE;
    double lon = (double)lon_raw / M10_BAM_SCALE;
    double alt = (double)alt_raw / 1000.0;   /* M10Decoder: alt = raw/1000.0 (mm) */

    out->lat_e5 = (int32_t)(lat * 100000.0);
    out->lon_e5 = (int32_t)(lon * 100000.0);
    out->alt_m  = (int32_t)alt;

    out->has_position = !(lat == 0.0 && lon == 0.0) &&
                        lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 &&
                        out->alt_m >= -1000 && out->alt_m <= 80000;
    return true;
}

void sonde_m10_hunt_init(sonde_m10_hunt_t *h)
{
    memset(h, 0, sizeof *h);
    uint32_t v = 0;
    for (int i = 0; i < M10_HEADER_CHIPS; i++)
        v = (v << 1) | (uint32_t)(M10_RAWHEADER_BITS[i] & 1u);
    h->pattern = v;
}

static int popcount32(uint32_t v)
{
#ifdef __GNUC__
    return __builtin_popcount(v);
#else
    int n = 0;
    while (v) { n += (int)(v & 1); v >>= 1; }
    return n;
#endif
}

/* MSB-first byte packing, first-arriving bit -> MSB -- matches
 * m10mod.c/m20mod.c's bits2bytes() (see sonde_m10.h). */
static void bits_to_bytes_msb(const uint8_t *bits, int nbits, uint8_t *out)
{
    int nbytes = nbits / 8;
    for (int i = 0; i < nbytes; i++) {
        uint8_t v = 0;
        for (int k = 0; k < 8; k++)
            v = (uint8_t)((v << 1) | (bits[8 * i + k] & 1u));
        out[i] = v;
    }
}

#define M10_MAX_HDR_ERR 2   /* out of 32 chips (~6%) */

bool sonde_m10_hunt_feed(sonde_m10_hunt_t *h, uint8_t chip, int32_t cell_avg, m10_gps_t *out)
{
    /* Correlate on every incoming chip, whether hunting or already capturing
     * -- see the note below on why capturing must keep re-checking too. */
    h->sr = (h->sr << 1) | (uint32_t)(chip & 1u);
    bool fresh_match = popcount32(h->sr ^ h->pattern) <= M10_MAX_HDR_ERR;

    if (!h->locked) {
        if (fresh_match) {
            h->locked = true;
            h->raw_count = 0;
        }
        return false;
    }

    /* Re-anchor on a fresh header match seen WHILE capturing. M10/M20's
     * preamble repeats this same 32-chip pattern several times before the
     * true sync-to-data boundary (confirmed on real air: the header locks
     * reliably every time, yet the GPS parse always failed -- the raw chip
     * dumps showed real, varying signal for only the first ~70-80 ms of
     * each 125 ms (1200-chip) capture window, degenerating into one very
     * long constant run for the rest, meaning the real payload had already
     * ENDED partway through the window). The original code locked on the
     * FIRST periodic match and never looked again, so it was very likely
     * anchoring somewhere in the middle of a still-repeating preamble
     * instead of at its end, burning most of the capture window on preamble
     * repeats instead of the actual frame. Re-checking (and re-anchoring
     * `raw_count` to 0) on every subsequent fresh match makes this track
     * the LATEST periodic repeat instead of freezing on the first one, so
     * the capture window starts as close as possible to where the real,
     * non-periodic payload actually begins. A spurious mid-payload re-match
     * is vanishingly unlikely (~1.2e-7 per chip for random data at
     * M10_MAX_HDR_ERR=2/32) and even then only costs some of the capture
     * window -- sonde_m20_parse_gps()'s checksum still gates the result. */
    if (fresh_match) {
        h->raw_count = 0;
        return false;
    }

    h->raw_cells[h->raw_count++] = cell_avg;
    if (h->raw_count < M10_CAPTURE_CHIPS)
        return false;

    h->locked = false;
    h->sr = 0;

    /* Manchester decode via sonde_m10_bits_from_cells() -- compares each
     * pair of raw chip-cell averages directly against EACH OTHER, not a
     * DC baseline (see sonde_m10.h's header note, fifth pass -- this
     * replaced sonde_manchester_decode_lenient() here after it turned out
     * still not robust enough over a full ~175 ms capture). No violation
     * concept any more, so there is nothing to compare between polarities
     * up front -- decode+parse BOTH and keep whichever validates. */
    uint8_t bits_a[M10_CAPTURE_CHIPS / 2];
    uint8_t bits_b[M10_CAPTURE_CHIPS / 2];
    int n = sonde_m10_bits_from_cells(h->raw_cells, M10_CAPTURE_CHIPS, bits_a, false);
    sonde_m10_bits_from_cells(h->raw_cells, M10_CAPTURE_CHIPS, bits_b, true);

    uint8_t diff_bits[M10_CAPTURE_CHIPS / 2];
    sonde_m10_diff_decode(bits_a, n, diff_bits);
    bits_to_bytes_msb(diff_bits, n - (n % 8), h->frame_bytes);

    /* try M10 first (the sonde this project has actually been tested
     * against on real air, 2026-09-11 -- see sonde_m10.h), fall back to
     * M20 -- both share the same header/Manchester/differential pipeline
     * above and only differ in frame length/marker/field layout from here. */
    sonde_m10_parse_gps(h->frame_bytes, (int)sizeof h->frame_bytes, out);
    if (!out->has_position)
        sonde_m20_parse_gps(h->frame_bytes, (int)sizeof h->frame_bytes, out);

    if (!out->has_position) {
        /* natural polarity didn't validate -- try the other one (see
         * sonde_m10.h: differential decode is invariant to inversion for
         * every bit except the very first one, but a second, cheap full
         * attempt is simpler and safer than special-casing that one bit). */
        sonde_m10_diff_decode(bits_b, n, diff_bits);
        bits_to_bytes_msb(diff_bits, n - (n % 8), h->frame_bytes);
        sonde_m10_parse_gps(h->frame_bytes, (int)sizeof h->frame_bytes, out);
        if (!out->has_position)
            sonde_m20_parse_gps(h->frame_bytes, (int)sizeof h->frame_bytes, out);
    }
    return true;
}
