/* test_sonde_m10 -- chip demod, bit-level Manchester + differential decode,
 * M10/M20 field layout + checksum, and the streaming hunter, see sonde_m10.h.
 *
 * The chip-level header (M10_RAWHEADER_BITS) is confirmed bit-exact against
 * a real M20 capture at 9600 baud (21/21 bursts). The bit-level decode
 * (sonde_m10_bits_from_cells(), fifth pass) is confirmed against real
 * captured audio: replayed offline, it recovers the M10 type marker exactly
 * and the decoded latitude matches the user's ground-truth calibration
 * position (49.65968 N) to 4 decimal places, reproducibly across
 * independent bursts recorded 16 s apart -- see sonde_m10.h's header note.
 * These tests check the port's mechanical self-consistency on synthetic
 * vectors built the same way m10mod.c/m20mod.c themselves would encode them
 * (cross-checked against HTCommander's independent Dart port), plus the
 * specific real-air failure modes found and fixed this session.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sonde_m10.h"

static int fails;

static void chk(const char *name, int cond)
{
    printf("  %-46s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

static unsigned rnd_state = 24681357u;
static unsigned rnd(void)
{
    rnd_state = rnd_state * 1103515245u + 12345u;
    return (rnd_state >> 16) & 0x7fffu;
}

static void test_chipdemod_synthetic(void)
{
    /* same synthetic-signal shape as test_sonde_demod.c's run_case(): a
     * random chip sequence, 1-pole-smoothed (GFSK-like) + noise, at a
     * TIGHT oversampling ratio (32 kHz / 9600 Bd ~ 3.3 samples/chip). This
     * only exercises `chip_out` (the DC-baseline decision), which is used
     * for header correlation ONLY these days -- see
     * test_bits_from_cells_synthetic() for the payload-relevant path. */
    const int fs = 32000, baud = 9600, n_chips = 400;
    double spc = (double)fs / baud;
    int n_samp = (int)(n_chips * spc) + 8;

    uint8_t *chips_in = malloc((size_t)n_chips);
    for (int i = 0; i < n_chips; i++) chips_in[i] = (uint8_t)(rnd() & 1);

    int32_t *samples = malloc(sizeof(int32_t) * (size_t)n_samp);
    int32_t lp_state = 0;
    const int32_t amplitude = 8000, noise_amp = 600;
    for (int n = 0; n < n_samp; n++) {
        int idx = (int)(n / spc);
        if (idx >= n_chips) idx = n_chips - 1;
        int32_t target = chips_in[idx] ? -amplitude : amplitude;
        lp_state += (target - lp_state) >> 1;
        int32_t noise = (int32_t)(rnd() % (unsigned)(2 * noise_amp + 1)) - noise_amp;
        samples[n] = lp_state + noise;
    }

    sonde_m10_chipdemod_t d;
    sonde_m10_chipdemod_init(&d, fs, baud);
    uint8_t *chips_out = malloc((size_t)n_chips + 8);
    int n_out = 0;
    for (int n = 0; n < n_samp && n_out < n_chips + 8; n++) {
        uint8_t c;
        if (sonde_m10_chipdemod_sample(&d, samples[n], &c, NULL))
            chips_out[n_out++] = c;
    }

    int best_skip = 0, best_err = n_chips;
    int compare_len = n_chips - 40;
    for (int skip = 0; skip <= 8 && skip + compare_len <= n_out; skip++) {
        int err = 0;
        for (int i = 0; i < compare_len; i++)
            if (chips_out[skip + i] != chips_in[i]) err++;
        if (err < best_err) { best_err = err; best_skip = skip; }
    }
    double err_rate = (double)best_err / compare_len;
    char label[96];
    snprintf(label, sizeof label, "chipdemod: chips recovered (n_out=%d/%d)", n_out, n_chips);
    chk(label, n_out >= n_chips - 4);
    snprintf(label, sizeof label, "chipdemod: error rate < 5%% (skip=%d, err=%d/%d)",
            best_skip, best_err, compare_len);
    chk(label, err_rate < 0.05);

    free(chips_in); free(samples); free(chips_out);
}

static void test_bits_from_cells_synthetic(void)
{
    /* the ACTUAL payload-decode path: a random BIT sequence, Manchester-
     * encoded into per-chip target amplitudes (bit 0 -> [+A,-A], bit 1 ->
     * [-A,+A], same convention as manchester_encode() below), run through
     * the real chipdemod to get cell_avg_out values, then decoded via
     * sonde_m10_bits_from_cells() -- this is what the WAV cross-check
     * (see sonde_m10.h) validated against real air. */
    const int fs = 96000, baud = 9600, n_bits = 400, n_chips = n_bits * 2;
    double spc = (double)fs / baud;
    int n_samp = (int)(n_chips * spc) + 8;

    uint8_t *bits_in = malloc((size_t)n_bits);
    for (int i = 0; i < n_bits; i++) bits_in[i] = (uint8_t)(rnd() & 1);

    uint8_t *chip_targets = malloc((size_t)n_chips);
    for (int i = 0; i < n_bits; i++) {
        if (bits_in[i] == 0) { chip_targets[2*i] = 0; chip_targets[2*i+1] = 1; }
        else                 { chip_targets[2*i] = 1; chip_targets[2*i+1] = 0; }
    }

    int32_t *samples = malloc(sizeof(int32_t) * (size_t)n_samp);
    int32_t lp_state = 0;
    const int32_t amplitude = 8000, noise_amp = 600;
    for (int n = 0; n < n_samp; n++) {
        int idx = (int)(n / spc);
        if (idx >= n_chips) idx = n_chips - 1;
        int32_t target = chip_targets[idx] ? -amplitude : amplitude;
        lp_state += (target - lp_state) >> 1;
        int32_t noise = (int32_t)(rnd() % (unsigned)(2 * noise_amp + 1)) - noise_amp;
        samples[n] = lp_state + noise;
    }

    sonde_m10_chipdemod_t d;
    sonde_m10_chipdemod_init(&d, fs, baud);
    int32_t *cells_out = malloc(sizeof(int32_t) * (size_t)(n_chips + 8));
    int n_out = 0;
    for (int n = 0; n < n_samp && n_out < n_chips + 8; n++) {
        uint8_t c; int32_t avg;
        if (sonde_m10_chipdemod_sample(&d, samples[n], &c, &avg))
            cells_out[n_out++] = avg;
    }

    uint8_t *bits_out = malloc((size_t)(n_out / 2 + 1));
    int n_bits_out = sonde_m10_bits_from_cells(cells_out, n_out, bits_out, false);

    int best_skip = 0, best_err = n_bits;
    int compare_len = n_bits - 20;
    for (int skip = 0; skip <= 4 && skip + compare_len <= n_bits_out; skip++) {
        int err = 0;
        for (int i = 0; i < compare_len; i++)
            if (bits_out[skip + i] != bits_in[i]) err++;
        if (err < best_err) { best_err = err; best_skip = skip; }
    }
    double err_rate = (double)best_err / compare_len;
    char label[112];
    snprintf(label, sizeof label, "bits_from_cells: bits recovered (n_out=%d/%d)", n_bits_out, n_bits);
    chk(label, n_bits_out >= n_bits - 4);
    snprintf(label, sizeof label, "bits_from_cells: error rate < 2%% (skip=%d, err=%d/%d)",
            best_skip, best_err, compare_len);
    chk(label, err_rate < 0.02);

    free(bits_in); free(chip_targets); free(samples); free(cells_out); free(bits_out);
}

static void test_diff_decode(void)
{
    /* hand-computed against m10mod.c's exact formula, see sonde_m10.c */
    uint8_t raw[]  = { 1,0,1,1,0,0,1 };
    uint8_t want[] = { 0,0,0,1,0,1,0 };
    uint8_t got[7];
    sonde_m10_diff_decode(raw, 7, got);
    chk("diff_decode: matches hand-computed vector", memcmp(got, want, 7) == 0);

    /* an all-same run (no raw transitions) after the first bit should
     * decode to a constant run of 1s (bit0==bit(i-1) -> NOT(0)=1) */
    uint8_t flat[6] = { 1,1,1,1,1,1 };
    uint8_t got2[6];
    sonde_m10_diff_decode(flat, 6, got2);
    int ok = (got2[0] == 0);   /* first bit: NOT(1) = 0 */
    for (int i = 1; i < 6 && ok; i++)
        if (got2[i] != 1) ok = 0;   /* NOT(1^1) = NOT(0) = 1 */
    chk("diff_decode: flat run -> 0 then all 1s", ok);
}

static void test_rawheader_shape(void)
{
    int all_01 = 1;
    for (int i = 0; i < M10_HEADER_CHIPS; i++)
        if (M10_RAWHEADER_BITS[i] > 1) all_01 = 0;
    chk("rawheader: 32 chips, all 0/1", all_01);
}

static void be_put_i32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)((uint32_t)v >> 24);
    p[1] = (uint8_t)((uint32_t)v >> 16);
    p[2] = (uint8_t)((uint32_t)v >> 8);
    p[3] = (uint8_t)((uint32_t)v);
}

/* Build a syntactically valid M20 frame (type marker + real checksum) with
 * the given position, the same way m20mod.c's own encoder would. */
static void build_m20_frame(uint8_t *f, int32_t lat_raw, int32_t lon_raw, int32_t alt_cm)
{
    memset(f, 0, M20_FRAME_BYTES);
    f[0] = 0x45;
    f[1] = 0x20;
    be_put_i32(f + 0x1C, lat_raw);
    be_put_i32(f + 0x20, lon_raw);
    f[0x08] = (uint8_t)((uint32_t)alt_cm >> 16);
    f[0x09] = (uint8_t)((uint32_t)alt_cm >> 8);
    f[0x0A] = (uint8_t)((uint32_t)alt_cm);
    uint16_t cs = sonde_m20_checksum(f, M20_POS_CHECK);
    f[M20_POS_CHECK]     = (uint8_t)(cs >> 8);
    f[M20_POS_CHECK + 1] = (uint8_t)(cs & 0xFF);
}

/* Build a syntactically valid M10 frame (type marker + real checksum) with
 * the given position, the same way m10mod.c's own encoder would. lat/lon in
 * degrees (converted to M10's BAM scale here), alt in whole metres. */
static void build_m10_frame(uint8_t *f, double lat_deg, double lon_deg, int32_t alt_m)
{
    memset(f, 0, M10_FRAME_BYTES);
    f[0] = M10_TYPE0;
    f[1] = M10_TYPE1;
    be_put_i32(f + M10_POS_LAT, (int32_t)(lat_deg * (1073741824.0 / 90.0)));
    be_put_i32(f + M10_POS_LON, (int32_t)(lon_deg * (1073741824.0 / 90.0)));
    be_put_i32(f + M10_POS_ALT, alt_m * 1000);
    uint16_t cs = sonde_m20_checksum(f, M10_POS_CHECK);
    f[M10_POS_CHECK]     = (uint8_t)(cs >> 8);
    f[M10_POS_CHECK + 1] = (uint8_t)(cs & 0xFF);
}

static void test_m10_parse_gps(void)
{
    uint8_t f[M10_FRAME_BYTES];
    build_m10_frame(f, 49.65968, 3.31387, 99);

    m10_gps_t g;
    bool ok = sonde_m10_parse_gps(f, sizeof f, &g);
    chk("m10 parse: returns true (frame long enough)", ok);
    chk("m10 parse: has_position", g.has_position);
    chk("m10 parse: lat within 0.0001 deg",
        g.lat_e5 > 4965868 && g.lat_e5 <= 4966068);
    chk("m10 parse: lon within 0.0001 deg",
        g.lon_e5 > 331287 && g.lon_e5 <= 331487);
    chk("m10 parse: alt = 99 m", g.alt_m == 99);

    m10_gps_t g2;
    chk("m10 parse: rejects a too-short frame",
        sonde_m10_parse_gps(f, 10, &g2) == false);

    m10_gps_t g3;
    uint8_t bad[M10_FRAME_BYTES];
    memcpy(bad, f, sizeof f);
    bad[5] ^= 0xFF;
    sonde_m10_parse_gps(bad, sizeof bad, &g3);
    chk("m10 parse: bad checksum -> no position", !g3.has_position);

    m10_gps_t g4;
    uint8_t not_m10[M10_FRAME_BYTES];
    memcpy(not_m10, f, sizeof f);
    not_m10[0] = 0x45; not_m10[1] = 0x20;   /* M20's marker, not M10's */
    sonde_m10_parse_gps(not_m10, sizeof not_m10, &g4);
    chk("m10 parse: wrong type marker -> no position", !g4.has_position);
}

static void test_checksum_roundtrip(void)
{
    uint8_t f[M20_FRAME_BYTES];
    build_m20_frame(f, (int32_t)(48.8566 * 1e6), (int32_t)(2.3522 * 1e6), 500000);
    uint16_t want = (uint16_t)((f[M20_POS_CHECK] << 8) | f[M20_POS_CHECK + 1]);
    chk("m20 checksum: self-consistent", sonde_m20_checksum(f, M20_POS_CHECK) == want);

    f[10] ^= 0xFF;   /* corrupt a payload byte */
    chk("m20 checksum: detects corruption", sonde_m20_checksum(f, M20_POS_CHECK) != want);
}

static void test_m20_parse_gps(void)
{
    /* Paris-ish: 48.8566 N, 2.3522 E, 5000 m -- M20 scale is a plain 1e6
     * (not RS41's 1e5 or M10's binary-angular-measurement 2^32/360), alt
     * in whole cm. */
    uint8_t f[M20_FRAME_BYTES];
    build_m20_frame(&f[0], (int32_t)(48.8566 * 1e6), (int32_t)(2.3522 * 1e6), 500000);

    m10_gps_t g;
    bool ok = sonde_m20_parse_gps(f, sizeof f, &g);
    chk("m20 parse: returns true (frame long enough)", ok);
    chk("m20 parse: has_position", g.has_position);
    chk("m20 parse: lat within 0.0001 deg",
        g.lat_e5 > 4885560 && g.lat_e5 <= 4885660);
    chk("m20 parse: lon within 0.0001 deg",
        g.lon_e5 > 235120 && g.lon_e5 <= 235220);
    chk("m20 parse: alt = 5000 m", g.alt_m == 5000);

    m10_gps_t g2;
    chk("m20 parse: rejects a too-short frame",
        sonde_m20_parse_gps(f, 10, &g2) == false);

    m10_gps_t g3;
    uint8_t bad[M20_FRAME_BYTES];
    memcpy(bad, f, sizeof f);
    bad[5] ^= 0xFF;   /* corrupt -> checksum should reject */
    sonde_m20_parse_gps(bad, sizeof bad, &g3);
    chk("m20 parse: bad checksum -> no position", !g3.has_position);
}

static void test_no_lock_on_noise(void)
{
    sonde_m10_hunt_t h;
    sonde_m10_hunt_init(&h);
    int hits = 0;
    m10_gps_t g;
    for (int i = 0; i < 20000; i++) {
        uint8_t chip = (uint8_t)(rnd() & 1);
        int32_t cell = chip ? -100 : 100;
        if (sonde_m10_hunt_feed(&h, chip, cell, &g))
            hits++;
    }
    chk("m10 hunt: no false lock on 20000 random chips", hits == 0);
}

/* Inverse of sonde_m10_diff_decode(): given the desired differentially-
 * decoded bits, produce the pre-differential bit stream (this project has
 * no on-air TX path -- this inversion exists only so the test can build a
 * realistic vector, the same way m20mod.c's own encoder would). */
static void diff_encode(const uint8_t *want_bits, int n, uint8_t *raw_out)
{
    uint8_t prev = 0;
    for (int i = 0; i < n; i++) {
        uint8_t r = (i == 0) ? (uint8_t)(1u - want_bits[0])
                             : (uint8_t)(prev ^ (1u - want_bits[i]));
        raw_out[i] = r;
        prev = r;
    }
}

/* Manchester-ENCODE (inverse of sonde_m10_bits_from_cells(): bit 0 must
 * decode from cells[2i]=+A > cells[2i+1]=-A being false, i.e. chip 0 -> +A,
 * chip 1 -> -A) -- bit 0 -> chips (0,1), bit 1 -> chips (1,0), same
 * convention sonde_m10_hunt_feed()'s real chipdemod produces. */
static void manchester_encode(const uint8_t *bits, int n, uint8_t *chips_out)
{
    for (int i = 0; i < n; i++) {
        if (bits[i] == 0) { chips_out[2*i] = 0; chips_out[2*i+1] = 1; }
        else              { chips_out[2*i] = 1; chips_out[2*i+1] = 0; }
    }
}

/* Convert a chip array (0/1, same convention as manchester_encode()'s
 * output) into the raw_cells-shaped int32 magnitudes sonde_m10_hunt_feed()
 * expects for its SECOND parameter -- chip 0 -> positive, chip 1 ->
 * negative, matching what a real sonde_m10_chipdemod_sample() call
 * produces (see sonde_m10.c). */
static void chips_to_cells(const uint8_t *chips, int n, int32_t *cells_out)
{
    for (int i = 0; i < n; i++)
        cells_out[i] = chips[i] ? -100 : 100;
}

static void bytes_to_bits_msb(const uint8_t *bytes, int nbytes, uint8_t *bits_out)
{
    for (int i = 0; i < nbytes; i++)
        for (int k = 0; k < 8; k++)
            bits_out[8 * i + k] = (uint8_t)((bytes[i] >> (7 - k)) & 1u);
}

static void test_hunt_end_to_end(void)
{
    uint8_t f[M20_FRAME_BYTES];
    build_m20_frame(f, (int32_t)(48.8566 * 1e6), (int32_t)(2.3522 * 1e6), 500000);

    int n_bits = M10_CAPTURE_CHIPS / 2;
    int n_frame_bits = (int)sizeof f * 8;
    if (n_frame_bits > n_bits) n_frame_bits = n_bits;   /* shouldn't happen */

    uint8_t want_bits[M10_CAPTURE_CHIPS / 2];
    memset(want_bits, 0, sizeof want_bits);
    bytes_to_bits_msb(f, (int)sizeof f, want_bits);

    uint8_t pre_diff[M10_CAPTURE_CHIPS / 2];
    diff_encode(want_bits, n_bits, pre_diff);
    uint8_t chips[M10_CAPTURE_CHIPS];
    manchester_encode(pre_diff, n_bits, chips);
    int32_t cells[M10_CAPTURE_CHIPS];
    chips_to_cells(chips, M10_CAPTURE_CHIPS, cells);

    sonde_m10_hunt_t h;
    sonde_m10_hunt_init(&h);
    m10_gps_t g;
    int hits = 0;

    for (int i = 0; i < 40; i++) {              /* junk prefix, arbitrary chip phase */
        uint8_t c = (uint8_t)(rnd() & 1);
        if (sonde_m10_hunt_feed(&h, c, c ? -100 : 100, &g)) hits++;
    }
    for (int i = 0; i < M10_HEADER_CHIPS; i++)
        if (sonde_m10_hunt_feed(&h, M10_RAWHEADER_BITS[i],
                                M10_RAWHEADER_BITS[i] ? -100 : 100, &g)) hits++;
    for (int i = 0; i < M10_CAPTURE_CHIPS; i++)
        if (sonde_m10_hunt_feed(&h, chips[i], cells[i], &g)) hits++;

    chk("m10 hunt: exactly one frame captured", hits == 1);
    chk("m10 hunt: has_position", g.has_position);
    chk("m10 hunt: lat within 0.0001 deg",
        g.lat_e5 > 4885560 && g.lat_e5 <= 4885660);
    chk("m10 hunt: lon within 0.0001 deg",
        g.lon_e5 > 235120 && g.lon_e5 <= 235220);
}

/* Same end-to-end check, but for an M10 frame (101 B) rather than M20 (70 B)
 * -- the type this project's actual test sonde turned out to be, 2026-09-11
 * (see sonde_m10.h). Confirms the M10_CAPTURE_CHIPS bump to 1680 actually
 * fits the longer frame and that sonde_m10_hunt_feed()'s M10-first dispatch
 * works end to end, not just sonde_m10_parse_gps() in isolation. */
static void test_hunt_end_to_end_m10(void)
{
    uint8_t f[M10_FRAME_BYTES];
    build_m10_frame(f, 49.65968, 3.31387, 99);

    int n_bits = M10_CAPTURE_CHIPS / 2;
    uint8_t want_bits[M10_CAPTURE_CHIPS / 2];
    memset(want_bits, 0, sizeof want_bits);
    bytes_to_bits_msb(f, (int)sizeof f, want_bits);

    uint8_t pre_diff[M10_CAPTURE_CHIPS / 2];
    diff_encode(want_bits, n_bits, pre_diff);
    uint8_t chips[M10_CAPTURE_CHIPS];
    manchester_encode(pre_diff, n_bits, chips);
    int32_t cells[M10_CAPTURE_CHIPS];
    chips_to_cells(chips, M10_CAPTURE_CHIPS, cells);

    sonde_m10_hunt_t h;
    sonde_m10_hunt_init(&h);
    m10_gps_t g;
    int hits = 0;

    for (int i = 0; i < 40; i++) {
        uint8_t c = (uint8_t)(rnd() & 1);
        if (sonde_m10_hunt_feed(&h, c, c ? -100 : 100, &g)) hits++;
    }
    for (int i = 0; i < M10_HEADER_CHIPS; i++)
        if (sonde_m10_hunt_feed(&h, M10_RAWHEADER_BITS[i],
                                M10_RAWHEADER_BITS[i] ? -100 : 100, &g)) hits++;
    for (int i = 0; i < M10_CAPTURE_CHIPS; i++)
        if (sonde_m10_hunt_feed(&h, chips[i], cells[i], &g)) hits++;

    chk("m10 hunt (M10 frame): exactly one frame captured", hits == 1);
    chk("m10 hunt (M10 frame): has_position", g.has_position);
    chk("m10 hunt (M10 frame): lat within 0.0001 deg",
        g.lat_e5 > 4965868 && g.lat_e5 <= 4966068);
    chk("m10 hunt (M10 frame): lon within 0.0001 deg",
        g.lon_e5 > 331287 && g.lon_e5 <= 331487);
    chk("m10 hunt (M10 frame): alt = 99 m", g.alt_m == 99);
}

/* Regression for the real-air finding (2026-09-11): M10/M20's preamble
 * repeats the 32-chip header pattern several times before the true
 * sync-to-data boundary. The first version of sonde_m10_hunt_feed() locked
 * on the FIRST periodic repeat and stopped looking, so on real captures it
 * kept anchoring somewhere in the middle of a still-repeating preamble --
 * the header always locked, but the GPS parse always failed, because the
 * capture window then ran out (M10_CAPTURE_CHIPS chips later) before the
 * real, non-periodic payload had even started. This test reproduces that
 * shape (several back-to-back header repeats before the real frame) and
 * checks the hunter still ends up decoding the real payload -- it would
 * fail this exact way (has_position false / garbage frame_bytes) against
 * the pre-fix "lock on first match, never re-check" version. */
static void test_hunt_repeated_preamble(void)
{
    uint8_t f[M20_FRAME_BYTES];
    build_m20_frame(f, (int32_t)(49.65968 * 1e6), (int32_t)(3.31387 * 1e6), 9900);

    int n_bits = M10_CAPTURE_CHIPS / 2;
    int n_frame_bits = (int)sizeof f * 8;
    if (n_frame_bits > n_bits) n_frame_bits = n_bits;

    uint8_t want_bits[M10_CAPTURE_CHIPS / 2];
    memset(want_bits, 0, sizeof want_bits);
    bytes_to_bits_msb(f, (int)sizeof f, want_bits);

    uint8_t pre_diff[M10_CAPTURE_CHIPS / 2];
    diff_encode(want_bits, n_bits, pre_diff);
    uint8_t chips[M10_CAPTURE_CHIPS];
    manchester_encode(pre_diff, n_bits, chips);
    int32_t cells[M10_CAPTURE_CHIPS];
    chips_to_cells(chips, M10_CAPTURE_CHIPS, cells);

    sonde_m10_hunt_t h;
    sonde_m10_hunt_init(&h);
    m10_gps_t g;
    int hits = 0;

    for (int i = 0; i < 40; i++) {
        uint8_t c = (uint8_t)(rnd() & 1);
        if (sonde_m10_hunt_feed(&h, c, c ? -100 : 100, &g)) hits++;
    }
    /* 6 back-to-back repeats of the 32-chip header -- a long periodic
     * preamble, same shape suspected on the real air captures. */
    for (int rep = 0; rep < 6; rep++)
        for (int i = 0; i < M10_HEADER_CHIPS; i++)
            if (sonde_m10_hunt_feed(&h, M10_RAWHEADER_BITS[i],
                                    M10_RAWHEADER_BITS[i] ? -100 : 100, &g)) hits++;
    for (int i = 0; i < M10_CAPTURE_CHIPS; i++)
        if (sonde_m10_hunt_feed(&h, chips[i], cells[i], &g)) hits++;

    chk("m10 hunt (repeated preamble): exactly one frame captured", hits == 1);
    chk("m10 hunt (repeated preamble): has_position", g.has_position);
    chk("m10 hunt (repeated preamble): lat within 0.0001 deg",
        g.lat_e5 > 4965868 && g.lat_e5 <= 4966068);
    chk("m10 hunt (repeated preamble): lon within 0.0001 deg",
        g.lon_e5 > 331287 && g.lon_e5 <= 331487);
}

/* Regression for the real-air finding (2026-09-11, continued): a mid-frame
 * transmission glitch used to permanently destroy byte alignment for the
 * rest of the frame (fourth pass, sonde_manchester_decode()'s skip-and-
 * shrink behaviour). The fifth-pass bits_from_cells() decode has no
 * "invalid pair" concept at all any more (every pair of real levels
 * compares to something), so this mostly checks that a single wrong BIT
 * stays local (the differential decode only propagates it 2 bits, same as
 * before) and doesn't cascade. */
static void test_hunt_survives_mid_frame_glitch(void)
{
    uint8_t f[M20_FRAME_BYTES];
    build_m20_frame(f, (int32_t)(49.65968 * 1e6), (int32_t)(3.31387 * 1e6), 9900);

    int n_bits = M10_CAPTURE_CHIPS / 2;
    uint8_t want_bits[M10_CAPTURE_CHIPS / 2];
    memset(want_bits, 0, sizeof want_bits);
    bytes_to_bits_msb(f, (int)sizeof f, want_bits);

    uint8_t pre_diff[M10_CAPTURE_CHIPS / 2];
    diff_encode(want_bits, n_bits, pre_diff);
    uint8_t chips[M10_CAPTURE_CHIPS];
    manchester_encode(pre_diff, n_bits, chips);
    int32_t cells[M10_CAPTURE_CHIPS];
    chips_to_cells(chips, M10_CAPTURE_CHIPS, cells);

    /* glitch one chip-cell's magnitude well inside the payload (bit ~20,
     * ahead of the checksum-covered region) -- attenuate it towards zero
     * rather than flip its sign outright, closer to what a real noisy
     * sample would do. */
    int glitch_cell = 2 * 20;
    cells[glitch_cell] = 5;

    sonde_m10_hunt_t h;
    sonde_m10_hunt_init(&h);
    m10_gps_t g;
    int hits = 0;

    for (int i = 0; i < 40; i++) {
        uint8_t c = (uint8_t)(rnd() & 1);
        if (sonde_m10_hunt_feed(&h, c, c ? -100 : 100, &g)) hits++;
    }
    for (int i = 0; i < M10_HEADER_CHIPS; i++)
        if (sonde_m10_hunt_feed(&h, M10_RAWHEADER_BITS[i],
                                M10_RAWHEADER_BITS[i] ? -100 : 100, &g)) hits++;
    for (int i = 0; i < M10_CAPTURE_CHIPS; i++)
        if (sonde_m10_hunt_feed(&h, chips[i], cells[i], &g)) hits++;

    chk("m10 hunt (mid-frame glitch): exactly one frame captured", hits == 1);
    chk("m10 hunt (mid-frame glitch): has_position", g.has_position);
    chk("m10 hunt (mid-frame glitch): lat within 0.0001 deg",
        g.lat_e5 > 4965868 && g.lat_e5 <= 4966068);
    chk("m10 hunt (mid-frame glitch): lon within 0.0001 deg",
        g.lon_e5 > 331287 && g.lon_e5 <= 331487);
}

int main(void)
{
    test_chipdemod_synthetic();
    test_bits_from_cells_synthetic();
    test_diff_decode();
    test_rawheader_shape();
    test_checksum_roundtrip();
    test_m20_parse_gps();
    test_m10_parse_gps();
    test_no_lock_on_noise();
    test_hunt_end_to_end();
    test_hunt_end_to_end_m10();
    test_hunt_repeated_preamble();
    test_hunt_survives_mid_frame_glitch();
    printf(fails ? "\nFAIL (%d)\n" : "\nall pass\n", fails);
    return fails ? 1 : 0;
}
