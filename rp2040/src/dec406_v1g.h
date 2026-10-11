/*
 * SPDX-License-Identifier: MIT
 *
 * dec406_v1g.h — public interface for the COSPAS-SARSAT 1st-generation (FGB /
 * C/S T.001) frame decoder.
 *
 * Ported for the Sarsat_UV-K1-5_RP2040 project from
 *   github.com/moricef/Decode_sarsat_406_v1g_v2g  (src/dec406_v1g.c)
 * Upstream LICENSE: MIT - Copyright (c) 2026 Fabrice Morel (copy kept at
 * vendor/decode_sarsat/LICENSE.moricef-upstream). Original decoder core:
 * F4EHY (dec406_v7, 2020).
 * (Older upstream revisions carried a "CC BY-NC-SA" header; upstream is now
 * MIT throughout -- LICENSE file and source headers.)
 *
 * The enum and struct below were moved out of dec406_v1g.c so that the decode
 * result can be consumed directly (radio screen formatting, host test harness)
 * instead of only via the printf report in decode_1g().
 */
#ifndef SARSAT_DEC406_V1G_H
#define SARSAT_DEC406_V1G_H

#include <stdint.h>

typedef enum {
    PROTOCOL_UNKNOWN,
    PROTOCOL_STANDARD_LOCATION,
    PROTOCOL_NATIONAL_LOCATION,
    PROTOCOL_USER_PROTOCOL,
    PROTOCOL_TEST,
    PROTOCOL_EMERGENCY_ELT,
    PROTOCOL_EMERGENCY_EPIRB,
    PROTOCOL_EMERGENCY_PLB,
    PROTOCOL_RLS_LOCATION,
    PROTOCOL_SHIP_SECURITY
} ProtocolType;

typedef struct {
    double lat;
    double lon;
    double base_lat;   // Base latitude from PDF-1
    double base_lon;   // Base longitude from PDF-1
    char vessel_id[64];
    char hex_id[24];
    uint16_t country_code;
    uint32_t serial;
    uint32_t mmsi;
    uint32_t aircraft_address;
    uint32_t operator_designator;
    uint32_t c_s_ta_number;
    uint8_t beacon_type;
    uint8_t id_type;
    uint8_t emergency_code;
    uint8_t auxiliary_device;
    uint8_t test_flag;
    uint8_t homing_flag;
    uint8_t position_source;
    uint8_t has_position;
    ProtocolType protocol;
    uint8_t frame_type;
    uint8_t crc_error;
    uint8_t activation_method;
    uint8_t location_freshness;
    uint8_t is_test_message;
    uint8_t position_default;
    // Fields for position offsets
    int lat_offset_sign;     // 1 = positive, -1 = negative
    int lon_offset_sign;
    uint8_t lat_offset_min;
    uint8_t lat_offset_sec;  // seconds, 4-second resolution
    uint8_t lon_offset_min;
    uint8_t lon_offset_sec;
    uint8_t protocol_bits;
    uint8_t bch1_fixed;   /* bits corrected by BCH-1 (0 = none needed), see fix_crc1() */
    uint8_t bch2_fixed;   /* bits corrected by BCH-2 (0 = none needed), see fix_crc2() */
} BeaconInfo1G;

/* BCH syndrome checks, detection only (0 = OK, 1 = error) — kept for anyone
 * relying on the pure-detect behaviour; decode_1g_frame() itself now calls
 * fix_crc1()/fix_crc2() instead. `s` is a '0'/'1' character string of at
 * least the frame length. */
int test_crc1(const char *s);   /* BCH-1 / PDF-1, bits 25..106 (82 bits, 21-bit code) */
int test_crc2(const char *s);   /* BCH-2 / PDF-2, bits 107..144 (38 bits, 12-bit code), long frames */

/* BCH correction (C/S T.001 codes): both are linear, so an error's syndrome
 * is the XOR of its individual bits' syndromes -- verified by exhaustive
 * search (host script, every weight-1/2/3 pattern) that BCH-1 corrects up to
 * 3 bit errors and BCH-2 up to 2, with no syndrome collision below that
 * weight, i.e. a match found is the unique correct fix, not a guess. Flips
 * the offending bit(s) of `s` in place. Returns the number of bits fixed (0
 * = already valid), or -1 if no combination within the code's proven radius
 * matches (more damage than it can guarantee -- same as the old test_crc*
 * reporting failure). fix_crc1() takes the radius to try (clamped to 3):
 * decode_1g_frame() uses 3 for long frames, 2 for short ones -- see there.
 * A correction also widens the door to false accepts on hiss-sliced junk,
 * so sarsat_decode_window() additionally requires an exact frame sync on
 * any corrected frame. */
int fix_crc1(char *s, int max_t);
int fix_crc2(char *s);

/* Full parse of a 112- or 144-bit frame given as a '0'/'1' string (index 0 =
 * first bit sync '1'). May flip bits in `frame` in place to correct BCH
 * errors (see fix_crc1()/fix_crc2()) before extracting fields. Fills *info;
 * check info->crc_error before trusting it. */
void decode_1g_frame(char *frame, int frame_length, BeaconInfo1G *info);

/* Legacy entry point: parse `bits` (one byte per bit, value 0/1) and print a
 * human-readable report to stdout. Kept for host debugging / dec406.c dispatch. */
void decode_1g(const uint8_t *bits, int length);

#endif /* SARSAT_DEC406_V1G_H */
