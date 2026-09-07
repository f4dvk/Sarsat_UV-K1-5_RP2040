/*
 * SPDX-License-Identifier: MIT
 *
 * country_codes.h — ITU-R M.585 Maritime Identification Digits (MID) lookup
 * for COSPAS-SARSAT beacon country codes.
 *
 * Ported for Sarsat_UV-K1-5_RP2040 from moricef/Decode_sarsat_406_v1g_v2g
 * (include/country_codes.h). Upstream defined the whole table `static` in the
 * header, which duplicated ~4 KB of .rodata into every translation unit that
 * included it. Here the table and lookup live in country_codes.c and this
 * header only declares the accessor.
 */
#ifndef COUNTRY_CODES_H
#define COUNTRY_CODES_H

/* Returns a static string; "Unknown" if the MID is not in the table. */
const char *get_country_name(int code);

#endif /* COUNTRY_CODES_H */
