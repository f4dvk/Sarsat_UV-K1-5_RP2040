/* sonde_rs41.c -- see sonde_rs41.h. Pure C, no pico-sdk, host-testable. */

#include "sonde_rs41.h"

#include <math.h>
#include <string.h>

const uint8_t RS41_RAW_SYNC[RS41_SYNC_LEN] = {
    0x86, 0x35, 0xF4, 0x40, 0x93, 0xDF, 0x1A, 0x60
};

const uint8_t RS41_MASK[RS41_MASK_LEN] = {
    0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98,
    0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
    0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
    0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
    0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
    0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
    0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
    0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1,
};

void rs41_descramble(uint8_t *data, int len, int mask_phase)
{
    for (int i = 0; i < len; i++)
        data[i] ^= RS41_MASK[(mask_phase + i) % RS41_MASK_LEN];
}

static int popcount8(uint8_t v)
{
#ifdef __GNUC__
    return __builtin_popcount(v);
#else
    int n = 0;
    while (v) { n += v & 1; v >>= 1; }
    return n;
#endif
}

int rs41_find_sync(const uint8_t *raw, int len, int max_bit_errors)
{
    if (len < RS41_SYNC_LEN)
        return -1;
    int best_pos = -1, best_err = max_bit_errors + 1;
    for (int pos = 0; pos <= len - RS41_SYNC_LEN; pos++) {
        int err = 0;
        for (int i = 0; i < RS41_SYNC_LEN && err <= max_bit_errors; i++)
            err += popcount8((uint8_t)(raw[pos + i] ^ RS41_RAW_SYNC[i]));
        if (err <= max_bit_errors && err < best_err) {
            best_err = err;
            best_pos = pos;
            if (err == 0)
                break;      /* can't do better than a perfect match */
        }
    }
    return best_pos;
}

/* Corrected against the reference decoder (rs41mod.c, radiosonde_auto_rx,
 * GPL-3.0 -- see sonde_rs41.h's header comment): CRC-16/CCITT-FALSE
 * parameterisation (poly 0x1021, init 0xFFFF, MSB-first, not reflected, no
 * xorout) -- NOT the CRC-16/KERMIT variant this function used before that
 * cross-check (a plausible-sounding guess from public ham-radio write-ups
 * that turned out wrong; the block CRC never validated on the one real
 * capture this project tried, see the radiosonde plan notes -- this is
 * almost certainly why). The received CRC is still read little-endian from
 * the frame (rs41_parse_blocks() below), matching u2() in the reference. */
uint16_t rs41_crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

int rs41_parse_blocks(const uint8_t *frame, int frame_len,
                      rs41_block_t *out, int max_blocks)
{
    int n = 0, off = 0;
    while (n < max_blocks && off + 2 <= frame_len) {
        uint8_t id   = frame[off];
        uint8_t blen = frame[off + 1];
        if (off + 2 + blen + 2 > frame_len)
            break;                                   /* short/garbled tail */
        const uint8_t *data = frame + off + 2;
        uint16_t crc_calc = rs41_crc16(data, blen);
        uint16_t crc_recv = (uint16_t)(data[blen] | (data[blen + 1] << 8));
        if (crc_calc != crc_recv)
            break;   /* first bad block ends the walk -- a resync mid-frame
                      * would need re-finding the block boundary, not attempted */
        out[n].id   = id;
        out[n].len  = blen;
        out[n].data = data;
        n++;
        off += 2 + blen + 2;
    }
    return n;
}

bool rs41_parse_gpspos(const uint8_t *data, int len, rs41_gpspos_t *out)
{
    if (len < 18)
        return false;
    out->ecef_x_cm = (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                               ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
    out->ecef_y_cm = (int32_t)((uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                               ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24));
    out->ecef_z_cm = (int32_t)((uint32_t)data[8] | ((uint32_t)data[9] << 8) |
                               ((uint32_t)data[10] << 16) | ((uint32_t)data[11] << 24));
    out->ecef_vx_cms = (int16_t)(data[12] | (data[13] << 8));
    out->ecef_vy_cms = (int16_t)(data[14] | (data[15] << 8));
    out->ecef_vz_cms = (int16_t)(data[16] | (data[17] << 8));
    return true;
}

/* WGS84 ellipsoid constants */
#define WGS84_A   6378137.0
#define WGS84_F   (1.0 / 298.257223563)
#define RS41_PI   3.14159265358979323846

void rs41_ecef_to_geo(double x_m, double y_m, double z_m,
                     int32_t *lat_e5, int32_t *lon_e5, int32_t *alt_m)
{
    const double a  = WGS84_A;
    const double f  = WGS84_F;
    const double b  = a * (1.0 - f);
    const double e2  = f * (2.0 - f);                    /* first eccentricity^2  */
    const double ep2 = (a * a - b * b) / (b * b);         /* second eccentricity^2 */

    double p = sqrt(x_m * x_m + y_m * y_m);
    double lon = atan2(y_m, x_m);

    double theta = atan2(z_m * a, p * b);
    double sin_t = sin(theta), cos_t = cos(theta);
    double lat = atan2(z_m + ep2 * b * sin_t * sin_t * sin_t,
                       p - e2 * a * cos_t * cos_t * cos_t);

    double sin_lat = sin(lat);
    double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
    double alt = (fabs(cos(lat)) > 1e-9) ? (p / cos(lat) - N)
                                        : (fabs(z_m) - b);   /* pole special-case */

    *lat_e5 = (int32_t)lround(lat * (180.0 / RS41_PI) * 100000.0);
    *lon_e5 = (int32_t)lround(lon * (180.0 / RS41_PI) * 100000.0);
    *alt_m  = (int32_t)lround(alt);
}
