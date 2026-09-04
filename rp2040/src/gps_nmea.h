/*
 * gps_nmea.h — minimal integer-only NMEA 0183 parser for the APRS tracker.
 *
 * Feed the serial bytes from a GPS module (any GN/GP/GL talker) one at a time.
 * On a checksum-valid RMC/GGA sentence the fix is updated. No float, no libc
 * beyond string.h — host-testable (rp2040/test/host/test_gps).
 *
 * Wiring on the KD8CEC C-Board: GPS TX -> RP2040 GP5 (UART1 RX), 9600 8N1.
 */
#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool     valid;        /* RMC status == 'A' (2D/3D fix)                    */
    int32_t  lat_e5;       /* latitude  * 1e5, + = N                          */
    int32_t  lon_e5;       /* longitude * 1e5, + = E                          */
    uint16_t speed_kmh;    /* ground speed, km/h (from RMC knots)             */
    uint16_t course_deg;   /* true course over ground, 0..359                 */
    int16_t  alt_m;        /* MSL altitude, m (from GGA; 0 until first GGA)   */
    uint8_t  sats;         /* satellites used (from GGA)                      */
    uint32_t seq;          /* bumped on every accepted RMC — lets the caller
                            * tell "fresh fix" from "same fix repeated"       */
} gps_fix_t;

typedef struct {
    char      buf[96];     /* sentence accumulator (NMEA max 82)              */
    uint8_t   len;
    bool      in;          /* seen '$', collecting                            */
    gps_fix_t fix;
} gps_t;

void gps_init(gps_t *g);

/* Feed one received byte. Returns true when a sentence was parsed and the fix
 * struct was touched (RMC or GGA). */
bool gps_feed(gps_t *g, char c);

#endif /* GPS_NMEA_H */
