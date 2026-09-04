/*
 * display_min.h — minimal replacement for moricef's display_utils.h.
 *
 * The upstream display_utils.c pulls in UTM trigonometry, isatty(), localtime()
 * and ANSI-hyperlink terminal output — none of which belong on the RP2040. Only
 * the three symbols actually referenced by dec406_v1g.c are kept here.
 */
#ifndef SARSAT_DISPLAY_MIN_H
#define SARSAT_DISPLAY_MIN_H

#include <stddef.h>

/* "48.51234 N, 2.34567 E" style string. */
void format_coordinates(double lat, double lon, char *buffer, size_t size);

/* No-op on the MCU (kept so the upstream call sites compile unchanged). */
void open_osm_map(double lat, double lon);
void log_to_terminal(const char *message);

#endif /* SARSAT_DISPLAY_MIN_H */
