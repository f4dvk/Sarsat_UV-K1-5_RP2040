/* sonde_m10.c -- see sonde_m10.h. Pure C, no pico-sdk, host-testable.
 * Ported from m10mod.c/m20mod.c (radiosonde_auto_rx, GPL-3.0), cross-checked
 * against HTCommander's independent Dart port -- see sonde_m10.h's header
 * comment for the Manchester-decode fix this cross-check produced.
 */

#include "sonde_m10.h"

#include <string.h>
#include <math.h>
#include <stdio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

#ifndef M10_MAX_HDR_ERR
#define M10_MAX_HDR_ERR 4   /* sur 32 chips ; un faux verrouillage ne coûte qu'une capture rejetée par la somme de contrôle */
#endif

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

/* ===== Récepteur "intégration à cadence fixe" (voir sonde_m10.h) ========= */

static int16_t clamp16(int32_t v) { return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v); }

void sonde_m10_rx_init(sonde_m10_rx_t *rx, float sample_rate_hz, int16_t *buf, int cap)
{
    memset(rx, 0, sizeof *rx);
    rx->buf = buf; rx->cap = cap;
    rx->fs  = sample_rate_hz;
    rx->sps = sample_rate_hz / 9615.0f;
    rx->L   = (int)(rx->sps + 0.5f);
    if (rx->L > 16) rx->L = 16;
    for (int j = 0; j < M10_HEADER_CHIPS; j++)
        rx->lag[j] = (int)((M10_HEADER_CHIPS - 1 - j) * rx->sps + 0.5f);
    rx->N = rx->lag[0] + rx->L;
    if (rx->N > M10_RX_CORR_RING) rx->N = M10_RX_CORR_RING;
    rx->thrN = (int64_t)(M10_RX_CORR_THR * 16.0f * (float)rx->N + 0.5f);
    /* on capture pour le débit le plus lent (9600) + marge de décalage */
    rx->need = (int)(M10_RX_MAX_SYMBOLS * (sample_rate_hz / 9600.0f)) + M10_RX_PREROLL + M10_RX_OFFSET_FWD + 4;
    if (rx->need > rx->cap) rx->need = rx->cap;
}

/* Octets lus à partir de `start` (indice dans buf) avec `sps` échantillons
 * par symbole ; renvoie le nombre d'octets produits (max_bytes au plus). */
static int m10_rx_bytes(const sonde_m10_rx_t *rx, float start, float sps, uint8_t *out, int max_bytes)
{
    float pos = start;
    int prev = 0, nbytes = 0;
    uint8_t acc = 0; int nb = 0;
    const int nbits = max_bytes * 8;
    for (int k = 0; k < nbits; k++) {
        int a0 = (int)(pos + 0.5f), a1 = (int)(pos + sps + 0.5f), a2 = (int)(pos + 2 * sps + 0.5f);
        if (a0 < 0 || a2 > rx->n) break;
        int32_t s1 = 0, s2 = 0;
        for (int i = a0; i < a1; i++) s1 += rx->buf[i];
        for (int i = a1; i < a2; i++) s2 += rx->buf[i];
        int bit = (s2 - s1) >= 0 ? 1 : 0;
        int fb = (k == 0) ? 0 : (bit == prev ? 1 : 0);   /* différentiel ; 1er bit = 0 (MSB de 0x64 / 0x45) */
        prev = bit;
        acc = (uint8_t)((acc << 1) | fb);
        if (++nb == 8) { out[nbytes++] = acc; acc = 0; nb = 0; }
        pos += 2 * sps;
    }
    return nbytes;
}

static int32_t be_i16(const uint8_t *p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }

/* Température extérieure (thermistance Shibaura PB5-41E), rs1729 get_Temp :
 * T/K = 1/(p0 + p1 ln R + p2 ln²R + p3 ln³R), R = Rs/(x - Rs/Rp), x = (4095-ADC)/ADC. */
static bool m10_temp(uint8_t scT, int adc, int16_t *out_c10)
{
    static const float Rs[3] = { 12.1e3f, 36.5e3f, 475.0e3f };
    static const float Rp[3] = { 1e20f, 330.0e3f, 2000.0e3f };
    const float p0 = 1.07303516e-03f, p1 = 2.41296733e-04f, p2 = 2.26744154e-06f, p3 = 6.52855181e-08f;
    if (scT > 2 || adc <= 0 || adc >= 4095) return false;
    float x = (4095.0f - (float)adc) / (float)adc;
    float R = Rs[scT] / (x - Rs[scT] / Rp[scT]);
    if (R <= 0) return false;
    float l = logf(R);
    float T = 1.0f / (p0 + p1 * l + p2 * l * l + p3 * l * l * l) - 273.15f;
    if (T < -120.0f || T > 60.0f) return false;
    *out_c10 = (int16_t)lroundf(T * 10.0f);
    return true;
}

static void m10_telemetry(const uint8_t *fr, int flen, m10_gps_t *out)
{
    out->temp_c10 = INT16_MIN; out->rh_pct = 255; out->vh_dms = 0xFFFF; out->vv_dms = 0;
    out->heading_deg = 0xFFFF; out->sats = 255; out->hh = out->mm = out->ss = 255; out->sn[0] = 0;
    uint8_t type = fr[1];
    out->type = type;
    float vscale; int pvE, pvN, pvU;
    if (type == 0x20) { vscale = 100.0f; pvE = 0x0B; pvN = 0x0D; pvU = 0x18; }
    else              { vscale = 200.0f; pvE = 0x04; pvN = 0x06; pvU = 0x08; }
    if (flen > pvU + 1) {
        float vx = be_i16(fr + pvE) / vscale, vy = be_i16(fr + pvN) / vscale, vz = be_i16(fr + pvU) / vscale;
        float vh = sqrtf(vx * vx + vy * vy);
        float dir = atan2f(vx, vy) * 180.0f / (float)M_PI; if (dir < 0) dir += 360.0f;
        out->vh_dms = (uint16_t)lroundf(vh * 10.0f);
        out->vv_dms = (int16_t)lroundf(vz * 10.0f);
        out->heading_deg = (uint16_t)lroundf(dir) % 360;
    }
    if (type == 0x20) {
        if (flen > 0x11) {                        /* heure : 3 octets BE en secondes de la semaine */
            uint32_t t = ((uint32_t)fr[0x0F] << 16) | ((uint32_t)fr[0x10] << 8) | fr[0x11];
            t %= 86400; out->hh = t / 3600; out->mm = (t % 3600) / 60; out->ss = t % 60;
        }
        if (flen > 0x05) {                        /* température : ADC 16 bits LE avec échelle dans les bits hauts */
            int adc = ((int)fr[0x5] << 8) | fr[0x4]; uint8_t scT = 0;
            if (adc > 8191) { scT = 2; adc -= 8192; } else if (adc > 4095) { scT = 1; adc -= 4096; }
            m10_temp(scT, adc, &out->temp_c10);
        }
        if (flen > 0x14) {                        /* numéro de série M20 */
            uint32_t sn24 = ((uint32_t)fr[0x14] << 16) | ((uint32_t)fr[0x13] << 8) | fr[0x12];
            uint8_t ym = sn24 & 0x7F;
            snprintf(out->sn, sizeof out->sn, "%u%02u-%u-%u%04u", ym / 12, (ym % 12) + 1,
                     (unsigned)(((sn24 >> 7) & 0x7) + 1), (unsigned)((sn24 >> 23) & 1), (unsigned)((sn24 >> 10) & 0x1FFF));
        }
    } else {
        if (flen > 0x1E) {                        /* heure GPS : TOW en ms, BE, à 0x0A ; satellites à 0x1E */
            uint32_t tow = ((uint32_t)fr[0x0A] << 24) | ((uint32_t)fr[0x0B] << 16) | ((uint32_t)fr[0x0C] << 8) | fr[0x0D];
            uint32_t t = (tow / 1000) % 86400; out->hh = t / 3600; out->mm = (t % 3600) / 60; out->ss = t % 60;
            out->sats = fr[0x1E];
        }
        if (flen > 0x40) {                        /* température : échelle à 0x3E, ADC LE à 0x3F/0x40 moins 0xA000 */
            uint8_t scT = fr[0x3E];
            int adc = (int)((uint16_t)(((uint16_t)fr[0x40] << 8) | fr[0x3F]) - 0xA000);
            bool ok = m10_temp(scT, adc, &out->temp_c10);
            if (ok && flen > 0x37) {              /* humidité : rapport des comptages 555 (RH / référence 55 %) */
                uint32_t cRH = fr[0x35] | ((uint32_t)fr[0x36] << 8) | ((uint32_t)fr[0x37] << 16);
                uint32_t c55 = fr[0x32] | ((uint32_t)fr[0x33] << 8) | ((uint32_t)fr[0x34] << 16);
                if (c55 > 0 && cRH > 0) {
                    float T = out->temp_c10 / 10.0f;
                    float rh = ((float)cRH / (float)c55 - 0.8955f) / 0.002f;
                    if (T < 0.0f)   rh += 0.0f - T / 5.5f;
                    if (T < -30.0f) rh *= 1.0f + (-30.0f - T) / 75.0f;
                    if (rh < 0) rh = 0;
                    if (rh > 100) rh = 100;
                    out->rh_pct = (uint8_t)lroundf(rh);
                }
            }
        }
        if (flen > 0x61) {                        /* numéro de série M10 (5 octets à 0x5D) */
            const uint8_t *r = fr + 0x5D;
            unsigned b = r[3] | ((unsigned)r[4] << 8);
            snprintf(out->sn, sizeof out->sn, "%1X%02u-%1X-%1u%04u", (r[2] >> 4) & 0xF, r[2] & 0xF, r[0] & 0xF, (b >> 13) & 0x7, b & 0x1FFF);
        }
    }
}

static bool m10_rx_fill_gps(const uint8_t *fr, int flen, m10_gps_t *out)
{
    double lat, lon, alt;
    uint8_t type = fr[1];
    m10_telemetry(fr, flen, out);
    if (type == 0x20) {                     /* M20 : 0x45 0x20 */
        if (flen < 0x24) return false;
        lat = be_i32(fr + 0x1C) / 1e6; lon = be_i32(fr + 0x20) / 1e6;
        alt = (double)(int32_t)(((uint32_t)fr[0x08] << 16) | ((uint32_t)fr[0x09] << 8) | fr[0x0A]) / 100.0;
    } else if (type == 0x9F || type == 0xAF || type == 0x8F) {   /* M10, M10+, M2K2 */
        if (flen < 0x1A) return false;
        lat = be_i32(fr + 0x0E) / M10_BAM_SCALE; lon = be_i32(fr + 0x12) / M10_BAM_SCALE;
        alt = be_i32(fr + 0x16) / 1000.0;
    } else return false;                    /* 0x49 (double trame) : pas de position ici */
    out->lat_e5 = (int32_t)(lat * 100000.0);
    out->lon_e5 = (int32_t)(lon * 100000.0);
    out->alt_m  = (int32_t)alt;
    out->has_position = !(lat == 0.0 && lon == 0.0) &&
                        lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 &&
                        out->alt_m >= -1000 && out->alt_m <= 80000;
    return out->has_position;
}

bool sonde_m10_rx_decode(sonde_m10_rx_t *rx, m10_gps_t *out)
{
    static const int bauds[2] = { 9615, 9600 };
    uint8_t fr[M10_RX_MAX_BYTES];
    memset(out, 0, sizeof *out);
    rx->frame_len = 0;
    for (int b = 0; b < 2; b++) {
        float sps = rx->fs / (float)bauds[b];
        for (int off = -M10_RX_OFFSET_MAX; off <= M10_RX_OFFSET_FWD; off++) {
            float start = (float)(M10_RX_PREROLL - rx->L / 2 + off);   /* le pic du corrélateur tombe ~1/2 symbole après le début des données */
            if (m10_rx_bytes(rx, start, sps, fr, 1) < 1) continue;
            int flen = fr[0];
            if (flen != 0x64 && flen != 0x45 && flen != 0x76 && flen != 0x43) continue;   /* M10, M20, M10 aux, M20 court */
            int nbytes = flen + 1;
            if (nbytes > M10_RX_MAX_BYTES) continue;
            if (m10_rx_bytes(rx, start, sps, fr, nbytes) < nbytes) continue;
            uint16_t want = (uint16_t)((fr[flen - 1] << 8) | fr[flen]);
            if (sonde_m20_checksum(fr, flen - 1) != want) continue;
            memcpy(rx->frame, fr, (size_t)nbytes);
            rx->frame_len = flen; rx->last_off = off; rx->last_baud = bauds[b];
            rx->frames_ok++;
            m10_rx_fill_gps(fr, flen, out);
            return true;
        }
    }
    return false;
}

static void m10_rx_start_capture(sonde_m10_rx_t *rx)
{
    rx->locks++;
    for (int i = 0; i < M10_RX_PREROLL; i++)
        rx->buf[i] = rx->pre[(rx->pre_i + i) % M10_RX_PREROLL];
    rx->n = M10_RX_PREROLL;
    rx->capturing = true;
}

bool sonde_m10_rx_sample(sonde_m10_rx_t *rx, int32_t sample, m10_gps_t *out)
{
    int16_t s16 = clamp16(sample);
    rx->pre[rx->pre_i] = s16;
    rx->pre_i = (rx->pre_i + 1) % M10_RX_PREROLL;

    /* ---- corrélateur d'en-tête (tourne en permanence, capture comprise) ---- */
    rx->box += s16 - rx->xr[rx->xi];
    rx->xr[rx->xi] = s16;
    if (++rx->xi >= rx->L) rx->xi = 0;
    int32_t q = ((int32_t)s16 * s16) >> 4;
    rx->sq += q - rx->sqr[rx->si];
    rx->sqr[rx->si] = q;
    if (++rx->si >= rx->N) rx->si = 0;
    rx->boxring[rx->bi] = rx->box;
    int64_t corr = 0;
    for (int j = 0; j < M10_HEADER_CHIPS; j++) {
        int idx = rx->bi - rx->lag[j];
        if (idx < 0) idx += M10_RX_CORR_RING;
        corr += M10_RAWHEADER_BITS[j] ? rx->boxring[idx] : -rx->boxring[idx];
    }
    if (++rx->bi >= M10_RX_CORR_RING) rx->bi = 0;
    /* corr² / (énergie × N) avec énergie = sq << 4. Tout en entier par
     * échantillon (pas de division, pas de double : 80 k/s sur un S3 à
     * 160 MHz ou un RP2040 sans FPU) ; le score flottant n'est calculé
     * qu'au-dessus du seuil, pour situer le pic. */
    int64_t c2 = corr * corr;
    float score = 0.0f;
    if (rx->sq > 0 && c2 > rx->thrN * rx->sq)
        score = (float)c2 / ((float)rx->sq * 16.0f * (float)rx->N);
    if (score > rx->max_score) rx->max_score = score;
    bool peak = rx->prevScore > 0.0f && score < rx->prevScore;
    rx->prevScore = score;
    /* Le pic était à l'échantillon précédent : données à partir d'ici. Pendant
     * une capture, seul un pic NETTEMENT meilleur la relance (un faux pic ne
     * doit pas jeter la vraie trame en cours). */
    if (peak && (!rx->capturing || rx->prevScore > rx->cap_score + 0.05f)) {
        rx->last_score = rx->prevScore;
        rx->cap_score  = rx->prevScore;
        m10_rx_start_capture(rx);
    }
    if (!rx->capturing) return false;
    rx->buf[rx->n++] = s16;
    if (rx->n < rx->need) return false;
    rx->capturing = false;
    sonde_m10_rx_decode(rx, out);
    return true;
}
