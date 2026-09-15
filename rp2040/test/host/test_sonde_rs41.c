/* test_sonde_rs41 -- RS41 frame-layer helpers (see sonde_rs41.h).
 *
 * No real RS41 capture is available (see the project's plan notes) -- these
 * tests check internal self-consistency (the mask/sync pair agree, CRC
 * round-trips, block walking stops on a bad CRC, ECEF<->geodetic round-trips
 * to sub-metre accuracy) rather than validating against a known-good
 * off-air frame. They do NOT prove the CRC variant or byte layout guessed
 * from the public write-up are correct on real hardware.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sonde_rs41.h"

static int fails;

static void chk(const char *name, int cond)
{
    printf("  %-40s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

/* The raw (scrambled) sync and the 64-byte mask were pulled from two
 * independent write-ups of the RS41 format; this checks they actually
 * agree with each other (mask[i] ^ raw_sync[i] should give the commonly
 * cited descrambled header 10 B6 CA 11 22 96 12 F8). */
static void test_mask_sync_consistency(void)
{
    static const uint8_t want_descrambled[RS41_SYNC_LEN] =
        { 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8 };
    uint8_t got[RS41_SYNC_LEN];
    for (int i = 0; i < RS41_SYNC_LEN; i++)
        got[i] = RS41_RAW_SYNC[i] ^ RS41_MASK[i];
    chk("mask/sync cross-check", memcmp(got, want_descrambled, RS41_SYNC_LEN) == 0);
}

static void test_descramble_roundtrip(void)
{
    uint8_t buf[200];
    for (int i = 0; i < 200; i++) buf[i] = (uint8_t)(i * 37 + 5);
    uint8_t orig[200];
    memcpy(orig, buf, sizeof buf);

    rs41_descramble(buf, sizeof buf, 0);
    chk("descramble changes data", memcmp(buf, orig, sizeof buf) != 0);
    rs41_descramble(buf, sizeof buf, 0);            /* XOR is its own inverse */
    chk("descramble round-trips", memcmp(buf, orig, sizeof buf) == 0);

    /* a phase offset must still round-trip against itself */
    uint8_t buf2[40];
    memcpy(buf2, orig, sizeof buf2);
    rs41_descramble(buf2, sizeof buf2, 23);
    rs41_descramble(buf2, sizeof buf2, 23);
    chk("descramble round-trips (phase != 0)", memcmp(buf2, orig, sizeof buf2) == 0);
}

static void test_find_sync(void)
{
    uint8_t stream[64];
    for (int i = 0; i < 64; i++) stream[i] = (uint8_t)(i * 91 + 3);   /* noise */
    memcpy(stream + 20, RS41_RAW_SYNC, RS41_SYNC_LEN);

    chk("exact sync found", rs41_find_sync(stream, sizeof stream, 0) == 20);

    uint8_t noisy[64];
    memcpy(noisy, stream, sizeof stream);
    noisy[21] ^= 0x01;                    /* flip 1 bit inside the sync word */
    chk("1-bit-error sync found (tol=2)", rs41_find_sync(noisy, sizeof noisy, 2) == 20);
    chk("1-bit-error sync rejected (tol=0)", rs41_find_sync(noisy, sizeof noisy, 0) != 20);

    uint8_t none[32];
    for (int i = 0; i < 32; i++) none[i] = (uint8_t)(i * 13 + 1);
    chk("no sync in pure noise (tol=0)", rs41_find_sync(none, sizeof none, 0) == -1);
}

static void test_crc_and_blocks(void)
{
    /* Build two back-to-back blocks (a fake GPSPOS block + a short filler
     * block) with real CRCs, then check rs41_parse_blocks() walks both and
     * stops cleanly if the second block's CRC is corrupted. */
    uint8_t frame[64];
    int n = 0;

    /* block 0: id 0x7B, 18-byte GPSPOS payload */
    rs41_gpspos_t gps_in = { 100, 200, 300, -692, -2673, 55 };
    frame[n++] = RS41_GPSPOS_ID;
    frame[n++] = 18;
    int body0 = n;
    memcpy(frame + n, &(uint32_t){(uint32_t)gps_in.ecef_x_cm}, 4); n += 4;
    memcpy(frame + n, &(uint32_t){(uint32_t)gps_in.ecef_y_cm}, 4); n += 4;
    memcpy(frame + n, &(uint32_t){(uint32_t)gps_in.ecef_z_cm}, 4); n += 4;
    memcpy(frame + n, &(uint16_t){(uint16_t)gps_in.ecef_vx_cms}, 2); n += 2;
    memcpy(frame + n, &(uint16_t){(uint16_t)gps_in.ecef_vy_cms}, 2); n += 2;
    memcpy(frame + n, &(uint16_t){(uint16_t)gps_in.ecef_vz_cms}, 2); n += 2;
    uint16_t crc0 = rs41_crc16(frame + body0, 18);
    frame[n++] = (uint8_t)(crc0 & 0xFF);
    frame[n++] = (uint8_t)(crc0 >> 8);

    /* block 1: id 0x79, 3-byte filler */
    int block1_start = n;
    frame[n++] = 0x79;
    frame[n++] = 3;
    int body1 = n;
    frame[n++] = 0xAA; frame[n++] = 0xBB; frame[n++] = 0xCC;
    uint16_t crc1 = rs41_crc16(frame + body1, 3);
    frame[n++] = (uint8_t)(crc1 & 0xFF);
    frame[n++] = (uint8_t)(crc1 >> 8);

    rs41_block_t blocks[4];
    int nb = rs41_parse_blocks(frame, n, blocks, 4);
    chk("both blocks parsed", nb == 2);
    chk("block 0 id/len", blocks[0].id == RS41_GPSPOS_ID && blocks[0].len == 18);
    chk("block 1 id/len", blocks[1].id == 0x79 && blocks[1].len == 3);

    rs41_gpspos_t gps_out;
    chk("gpspos parses", rs41_parse_gpspos(blocks[0].data, blocks[0].len, &gps_out));
    chk("gpspos fields round-trip",
        gps_out.ecef_x_cm == gps_in.ecef_x_cm && gps_out.ecef_y_cm == gps_in.ecef_y_cm &&
        gps_out.ecef_z_cm == gps_in.ecef_z_cm && gps_out.ecef_vx_cms == gps_in.ecef_vx_cms &&
        gps_out.ecef_vy_cms == gps_in.ecef_vy_cms && gps_out.ecef_vz_cms == gps_in.ecef_vz_cms);

    /* corrupt block 1's CRC -> only block 0 should come back */
    frame[n - 1] ^= 0xFF;
    nb = rs41_parse_blocks(frame, n, blocks, 4);
    chk("bad CRC stops the walk", nb == 1);
    (void)block1_start;
}

static void test_ecef_roundtrip(void)
{
    /* Paris-ish: 48.8566 N, 2.3522 E, 100 m -- forward-project to ECEF with
     * the same WGS84 ellipsoid, then check rs41_ecef_to_geo() recovers it. */
    struct { double lat_deg, lon_deg, alt_m; } pts[] = {
        { 48.8566,   2.3522,   100.0 },
        {  0.0,      0.0,        0.0 },
        { -33.8688, 151.2093,  50.0 },
        { 89.9,      10.0,    500.0 },     /* near the pole */
    };
    const double a = 6378137.0, f = 1.0 / 298.257223563;
    const double b = a * (1 - f), e2 = f * (2 - f);
    const double DEG = 3.14159265358979323846 / 180.0;

    for (unsigned i = 0; i < sizeof pts / sizeof pts[0]; i++) {
        double lat = pts[i].lat_deg * DEG, lon = pts[i].lon_deg * DEG;
        double sin_lat = sin(lat), cos_lat = cos(lat);
        double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
        double x = (N + pts[i].alt_m) * cos_lat * cos(lon);
        double y = (N + pts[i].alt_m) * cos_lat * sin(lon);
        double z = (N * (1.0 - e2) + pts[i].alt_m) * sin_lat;
        (void)b;

        int32_t lat_e5, lon_e5, alt_m;
        rs41_ecef_to_geo(x, y, z, &lat_e5, &lon_e5, &alt_m);

        double got_lat = lat_e5 / 100000.0, got_lon = lon_e5 / 100000.0;
        char name[64];
        snprintf(name, sizeof name, "ecef round-trip #%u lat/lon", i);
        chk(name, fabs(got_lat - pts[i].lat_deg) < 0.0001 &&
                  fabs(got_lon - pts[i].lon_deg) < 0.0001);
        snprintf(name, sizeof name, "ecef round-trip #%u alt", i);
        chk(name, fabs((double)alt_m - pts[i].alt_m) < 1.0);
    }
}

int main(void)
{
    test_mask_sync_consistency();
    test_descramble_roundtrip();
    test_find_sync();
    test_crc_and_blocks();
    test_ecef_roundtrip();
    printf(fails ? "\nFAIL (%d)\n" : "\nall pass\n", fails);
    return fails ? 1 : 0;
}
