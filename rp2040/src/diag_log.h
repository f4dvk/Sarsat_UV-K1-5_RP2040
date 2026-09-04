/*
 * diag_log.h — Sarsat_UV-K1-5_RP2040 port.
 *
 * Upstream (moricef/Decode_sarsat_406_v1g_v2g) routes verbose diagnostics to
 * stderr with systemd journal priority prefixes. On the microcontroller there
 * is no journal; DIAG/DWARN are compiled out by default and DERR maps to a
 * plain stdio line. Define SARSAT_VERBOSE=1 to get all three on stdout.
 */
#ifndef DIAG_LOG_H
#define DIAG_LOG_H

#include <stdio.h>

#ifndef SARSAT_VERBOSE
#define SARSAT_VERBOSE 0
#endif

#if SARSAT_VERBOSE
#define DIAG(fmt, ...)  printf("[diag] " fmt, ##__VA_ARGS__)
#define DWARN(fmt, ...) printf("[warn] " fmt, ##__VA_ARGS__)
#else
#define DIAG(fmt, ...)  ((void)0)
#define DWARN(fmt, ...) ((void)0)
#endif

#define DERR(fmt, ...)  printf("[err] " fmt, ##__VA_ARGS__)

#endif /* DIAG_LOG_H */
