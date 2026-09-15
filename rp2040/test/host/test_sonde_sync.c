/* test_sonde_sync -- streaming bit-level sync hunter (see sonde_sync.h).
 *
 * Synthetic bitstreams only (no real capture to validate the RS41 frame
 * decode against end-to-end -- see sonde_rs41.h/test_sonde_rs41.c's same
 * caveat). What IS checked here: the RS41 sync locks and a full frame gets
 * captured+decoded regardless of how much junk precedes it (bit-level, not
 * byte-aligned); the M10/M20 header locks at exactly the right bit and
 * tolerates a couple of bit errors but not more (M10_MAX_ERR in
 * sonde_sync.c); a stream with neither pattern never fires either event.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sonde_rs41.h"
#include "sonde_sync.h"

static int fails;

static void chk(const char *name, int cond)
{
    printf("  %-46s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

static unsigned rnd_state = 987654321u;
static unsigned rnd(void)
{
    rnd_state = rnd_state * 1103515245u + 12345u;
    return (rnd_state >> 16) & 0x7fffu;
}

/* Feed `nbytes` bytes MSB-first (bit 7 first) and return the number of
 * SONDE_EVT_RS41/SONDE_EVT_M10 events seen, writing the RS41 result (if any)
 * to *rs41_out. `n_m10` (may be NULL) counts SONDE_EVT_M10 hits. */
static int feed_bytes_msb(sonde_sync_t *s, const uint8_t *bytes, int nbytes,
                          sonde_rs41_result_t *rs41_out, int *n_m10)
{
    int n_rs41 = 0;
    if (n_m10) *n_m10 = 0;
    for (int i = 0; i < nbytes; i++) {
        for (int k = 7; k >= 0; k--) {
            uint8_t bit = (uint8_t)((bytes[i] >> k) & 1);
            sonde_evt_t e = sonde_sync_feed(s, bit, rs41_out);
            if (e == SONDE_EVT_RS41) n_rs41++;
            if (e == SONDE_EVT_M10 && n_m10) (*n_m10)++;
        }
    }
    return n_rs41;
}

/* geodetic -> ECEF (WGS84), same forward projection test_sonde_rs41.c uses,
 * so this test can build a GPSPOS block with a known-good reference point. */
static void geo_to_ecef(double lat_deg, double lon_deg, double alt_m,
                        double *x, double *y, double *z)
{
    const double a = 6378137.0, f = 1.0 / 298.257223563;
    const double e2 = f * (2 - f);
    const double DEG = 3.14159265358979323846 / 180.0;
    double lat = lat_deg * DEG, lon = lon_deg * DEG;
    double sin_lat = sin(lat), cos_lat = cos(lat);
    double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
    *x = (N + alt_m) * cos_lat * cos(lon);
    *y = (N + alt_m) * cos_lat * sin(lon);
    *z = (N * (1.0 - e2) + alt_m) * sin_lat;
}

static void test_rs41_end_to_end(void)
{
    /* Paris-ish, 5000 m: build the ECEF GPSPOS block from it. */
    double x, y, z;
    geo_to_ecef(48.8566, 2.3522, 5000.0, &x, &y, &z);

    uint8_t plain[SONDE_RS41_FRAME_CAP];
    memset(plain, 0x55, sizeof plain);   /* filler past the one real block */

    int n = 0;
    plain[n++] = RS41_GPSPOS_ID;
    plain[n++] = 18;
    int body = n;
    uint32_t ex = (uint32_t)(int32_t)(x * 100.0);
    uint32_t ey = (uint32_t)(int32_t)(y * 100.0);
    uint32_t ez = (uint32_t)(int32_t)(z * 100.0);
    memcpy(plain + n, &ex, 4); n += 4;
    memcpy(plain + n, &ey, 4); n += 4;
    memcpy(plain + n, &ez, 4); n += 4;
    uint16_t vx = 0, vy = 0, vz = 0;
    memcpy(plain + n, &vx, 2); n += 2;
    memcpy(plain + n, &vy, 2); n += 2;
    memcpy(plain + n, &vz, 2); n += 2;
    uint16_t crc = rs41_crc16(plain + body, 18);
    plain[n++] = (uint8_t)(crc & 0xFF);
    plain[n++] = (uint8_t)(crc >> 8);

    uint8_t onair[SONDE_RS41_FRAME_CAP];
    memcpy(onair, plain, sizeof plain);
    rs41_descramble(onair, sizeof onair, 0);   /* whiten (XOR is its own inverse) */

    /* junk bits, then the raw sync, then the on-air (scrambled) frame --
     * the junk length (37 bits, deliberately not a multiple of 8) checks
     * that the sync is found at an arbitrary bit phase, not just byte-
     * aligned, matching how a real bit-PLL locks (see sonde_sync.h). */
    sonde_sync_t s;
    sonde_sync_init(&s);
    sonde_rs41_result_t r;
    memset(&r, 0, sizeof r);

    int n_rs41 = 0, n_m10 = 0;
    for (int i = 0; i < 37; i++) {
        sonde_evt_t e = sonde_sync_feed(&s, (uint8_t)(rnd() & 1), &r);
        if (e == SONDE_EVT_RS41) n_rs41++;
        if (e == SONDE_EVT_M10)  n_m10++;
    }
    n_rs41 += feed_bytes_msb(&s, RS41_RAW_SYNC, RS41_SYNC_LEN, &r, &n_m10);
    n_rs41 += feed_bytes_msb(&s, onair, sizeof onair, &r, &n_m10);

    chk("RS41: exactly one frame event", n_rs41 == 1);
    chk("RS41: no spurious M10 event",   n_m10 == 0);
    chk("RS41: >=1 CRC-valid block",     r.n_blocks >= 1);
    chk("RS41: has_position",            r.has_position);
    if (r.has_position) {
        chk("RS41: lat within 0.001 deg",
            fabs(r.lat_e5 / 100000.0 - 48.8566) < 0.001);
        chk("RS41: lon within 0.001 deg",
            fabs(r.lon_e5 / 100000.0 - 2.3522) < 0.001);
        chk("RS41: alt within 5 m", fabs((double)r.alt_m - 5000.0) < 5.0);
    } else {
        chk("RS41: lat within 0.001 deg", 0);
        chk("RS41: lon within 0.001 deg", 0);
        chk("RS41: alt within 5 m", 0);
    }
}

static void test_no_sync_in_noise(void)
{
    sonde_sync_t s;
    sonde_sync_init(&s);
    int n_rs41 = 0, n_m10 = 0;
    sonde_rs41_result_t r;
    for (int i = 0; i < 5000; i++) {
        sonde_evt_t e = sonde_sync_feed(&s, (uint8_t)(rnd() & 1), &r);
        if (e == SONDE_EVT_RS41) n_rs41++;
        if (e == SONDE_EVT_M10)  n_m10++;
    }
    /* 5000 random bits, RS41_MAX_ERR=3/64 and M10_MAX_ERR=2/28: astronomically
     * unlikely to false-trigger either -- if this ever fails, the tolerance
     * is too loose for the pattern length and needs tightening. */
    chk("noise: no RS41 false lock", n_rs41 == 0);
    chk("noise: no M10 false lock",  n_m10 == 0);
}

static void test_m10_header_lock(void)
{
    static const uint8_t hdr[4] = { 0x5A, 0x4A, 0x93, 0x0A };   /* 28 bits used */

    sonde_sync_t s;
    sonde_sync_init(&s);
    int n_rs41 = 0, n_m10 = 0, last_m10_at = -1;
    sonde_rs41_result_t r;

    int bitno = 0;
    for (int i = 0; i < 50; i++, bitno++) {              /* junk prefix */
        sonde_evt_t e = sonde_sync_feed(&s, (uint8_t)(rnd() & 1), &r);
        if (e == SONDE_EVT_M10)  { n_m10++; last_m10_at = bitno; }
        if (e == SONDE_EVT_RS41) n_rs41++;
    }
    int header_bits_sent = 0;
    for (int bi = 0; bi < 4 && header_bits_sent < 28; bi++) {
        for (int k = 0; k < 8 && header_bits_sent < 28; k++, bitno++) {
            uint8_t bit = (uint8_t)((hdr[bi] >> k) & 1);   /* LSB-first/byte */
            sonde_evt_t e = sonde_sync_feed(&s, bit, &r);
            if (e == SONDE_EVT_M10)  { n_m10++; last_m10_at = bitno; }
            if (e == SONDE_EVT_RS41) n_rs41++;
            header_bits_sent++;
        }
    }
    chk("M10: header locks exactly once", n_m10 == 1);
    chk("M10: locks on the last header bit fed", last_m10_at == bitno - 1);
    chk("M10: no spurious RS41 event", n_rs41 == 0);
}

static void test_m10_header_tolerance(void)
{
    static const uint8_t hdr[4] = { 0x5A, 0x4A, 0x93, 0x0A };

    /* 2-bit error (== M10_MAX_ERR): still locks */
    {
        sonde_sync_t s;
        sonde_sync_init(&s);
        sonde_rs41_result_t r;
        int n_m10 = 0;
        int bitno = 0;
        for (int bi = 0; bi < 4; bi++)
            for (int k = 0; k < 8 && bi * 8 + k < 28; k++, bitno++) {
                uint8_t bit = (uint8_t)((hdr[bi] >> k) & 1);
                if (bitno == 3 || bitno == 17) bit ^= 1;    /* flip 2 bits */
                sonde_evt_t e = sonde_sync_feed(&s, bit, &r);
                if (e == SONDE_EVT_M10) n_m10++;
            }
        chk("M10: 2-bit error still locks (tol=2)", n_m10 == 1);
    }
    /* 4-bit error (> M10_MAX_ERR): must NOT lock */
    {
        sonde_sync_t s;
        sonde_sync_init(&s);
        sonde_rs41_result_t r;
        int n_m10 = 0;
        int bitno = 0;
        for (int bi = 0; bi < 4; bi++)
            for (int k = 0; k < 8 && bi * 8 + k < 28; k++, bitno++) {
                uint8_t bit = (uint8_t)((hdr[bi] >> k) & 1);
                if (bitno == 2 || bitno == 9 || bitno == 15 || bitno == 22) bit ^= 1;
                sonde_evt_t e = sonde_sync_feed(&s, bit, &r);
                if (e == SONDE_EVT_M10) n_m10++;
            }
        chk("M10: 4-bit error rejected (tol=2)", n_m10 == 0);
    }
}

int main(void)
{
    test_rs41_end_to_end();
    test_no_sync_in_noise();
    test_m10_header_lock();
    test_m10_header_tolerance();
    printf(fails ? "\nFAIL (%d)\n" : "\nall pass\n", fails);
    return fails ? 1 : 0;
}
