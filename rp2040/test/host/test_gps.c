/* test_gps — feed NMEA strings to the parser, check the fix. */
#include <stdio.h>
#include <string.h>
#include "gps_nmea.h"

static int bad;

static void feed(gps_t *g, const char *s)
{
    for (const char *p = s; *p; p++) gps_feed(g, *p);
}

static void chk(const char *name, int cond)
{
    printf("  %-40s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) bad++;
}

int main(void)
{
    printf("GPS NMEA parser:\n");

    /* canonical RMC/GGA pair (Munich) */
    {
        gps_t g; gps_init(&g);
        feed(&g, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
        feed(&g, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
        chk("RMC valid",        g.fix.valid);
        /* 48 deg 07.038' = 48.11730 -> 4811730 e5 */
        chk("lat ~ 48.1173",    g.fix.lat_e5 > 4811700 && g.fix.lat_e5 < 4811760);
        /* 11 deg 31.000' = 11.51667 -> 1151667 e5 */
        chk("lon ~ 11.5167",    g.fix.lon_e5 > 1151640 && g.fix.lon_e5 < 1151700);
        chk("speed ~ 41 km/h",  g.fix.speed_kmh >= 40 && g.fix.speed_kmh <= 42);
        chk("course 84",        g.fix.course_deg == 84);
        chk("sats 8",           g.fix.sats == 8);
        chk("alt 545",          g.fix.alt_m == 545);
    }

    /* southern hemisphere, GN talker */
    {
        gps_t g; gps_init(&g);
        feed(&g, "$GNRMC,220516,A,3350.000,S,15112.000,E,0.0,0.0,020925,,,A*7E\r\n");
        chk("S lat negative",   g.fix.lat_e5 < 0);
        /* 33 50.000' S -> -33.83333 -> -3383333 */
        chk("lat ~ -33.8333",   g.fix.lat_e5 < -3383300 && g.fix.lat_e5 > -3383370);
        chk("E lon positive",   g.fix.lon_e5 > 0);
    }

    /* bad checksum ignored; V status clears the fix but bumps seq */
    {
        gps_t g; gps_init(&g);
        feed(&g, "$GPRMC,123519,A,4807.038,N,01131.000,E,0,0,230394,,,A*00\r\n");
        chk("bad checksum ignored", !g.fix.valid && g.fix.seq == 0);
        feed(&g, "$GPRMC,123520,V,,,,,,,230394,,,N*5B\r\n");
        chk("V status -> not valid", !g.fix.valid);
        chk("seq advanced",         g.fix.seq == 1);
    }

    /* fragmented byte stream + leading garbage -> resync on '$' */
    {
        gps_t g; gps_init(&g);
        feed(&g, "noise\r\n$GPGGA,,,,,,0,,");
        feed(&g, ",,,,,*4A\r\n");
        feed(&g, "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62\r\n");
        chk("resync after garbage", g.fix.valid && g.fix.lat_e5 < 0);
    }

    printf(bad ? "\nFAIL\n" : "\nPASS\n");
    return bad ? 1 : 0;
}
