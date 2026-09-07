/*
 * aprs_parse.h — turn an AX.25 UI frame's info field into structured APRS
 * fields (callsign, symbol, position, course/speed/altitude, comment) plus a
 * great-circle distance/bearing helper.
 *
 * Handles: uncompressed position (! = @ /), MIC-E, objects (;), items ()),
 * status (>), messages (:). Compressed position is decoded to a coarse
 * lat/lon. No float — integer only, host-testable.
 */
#ifndef APRS_PARSE_H
#define APRS_PARSE_H

#include <stdint.h>
#include <stdbool.h>

enum {
    APRS_KIND_OTHER = 0,
    APRS_KIND_POSITION,      /* has a lat/lon                                 */
    APRS_KIND_OBJECT,        /* named object / item, has a lat/lon            */
    APRS_KIND_STATUS,        /* > status text                                 */
    APRS_KIND_MESSAGE,       /* : message to someone                          */
    APRS_KIND_TELEMETRY,     /* T# telemetry                                  */
};

typedef struct {
    char     src[10];        /* "F4DVK-9" (from the AX.25 source address)     */
    uint8_t  kind;

    char     sym_table;      /* '/', '\\', or an overlay char (0 if none)     */
    char     sym_code;

    bool     has_pos;
    int32_t  lat_e5;         /* + = north */
    int32_t  lon_e5;         /* + = east  */

    bool     has_course;
    int16_t  course_deg;     /* 0..359                                        */
    int16_t  speed_kmh;

    bool     has_alt;
    int32_t  alt_m;

    char     name[10];       /* object / item name, or message addressee      */
    char     text[64];       /* comment / status / message body               */

    char     msg_no[8];      /* APRS message number after '{' ("" if none)     */
    bool     is_adrasec;     /* message body begins "ADRASEC" (PCT_Report      */
                             /* position request): lat_e5 / lon_e5 hold the   */
                             /* parsed coordinates for a clear on-screen show  */
} aprs_info_t;

/* Parse `ax25` (the frame WITHOUT the 2 FCS bytes, as delivered by aprs_rx).
 * Returns true if anything useful was extracted. */
bool aprs_parse(const uint8_t *ax25, int len, aprs_info_t *out);

/* The last real digipeater that repeated the frame (AX.25 H bit set, callsign
 * not a WIDEn/TRACEn/RELAY-style alias) -- the station we actually heard it
 * from, e.g. "F8KCS-2". Empty = heard directly. Returns 1 if a call was written. */
int ax25_via_str(const uint8_t *ax25, int len, char *out, int cap);

/* Great-circle distance (metres) and initial bearing (deg, 0..359) from
 * (lat1,lon1) to (lat2,lon2), all in 1e-5 deg. Equirectangular approximation
 * (fine to a few hundred km). Returns false if a coord is out of range. */
bool aprs_geo(int32_t lat1_e5, int32_t lon1_e5,
              int32_t lat2_e5, int32_t lon2_e5,
              uint32_t *dist_m, uint16_t *bearing_deg);

#endif /* APRS_PARSE_H */
