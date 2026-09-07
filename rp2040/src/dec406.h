/*
 * SPDX-License-Identifier: MIT
 *
 * Ported for the Sarsat_UV-K1-5_RP2040 project from
 *   github.com/moricef/Decode_sarsat_406_v1g_v2g
 * Upstream repository LICENSE: MIT - Copyright (c) 2026 Fabrice Morel
 *   (copy kept at vendor/decode_sarsat/LICENSE.moricef-upstream).
 * (Older upstream revisions carried a "CC BY-NC-SA" header; upstream is now MIT
 * throughout -- LICENSE file and source headers.)
 *
 * Upstream authors / contributions:
 *   - dec406_v7 original decoder core: F4EHY (2020)
 *   - Refactoring + 2G support: collaborative development (2025)
 *   - T.018 conformance: BCH + MID database implementation
 */


// dec406.h
#ifndef DEC406_H
#define DEC406_H

#include <stdint.h>

#define FRAME_1G_SHORT 112
#define FRAME_1G_LONG 144
#define FRAME_2G_LENGTH 250

void decode_1g(const uint8_t *bits, int length);
void decode_2g(const uint8_t *bits);
/* Set the self-test mode for the next decode_2g() call, derived from the DSSS
 * PRN mode (self-test beacons use a distinct spreading code). Consumed and
 * reset by decode_2g; paths without PRN info default to Normal. */
void decode_2g_set_frame_mode(int is_self_test);
void decode_beacon(const uint8_t *bits, int length);

#endif
