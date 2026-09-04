/* gps_nmea.c — see gps_nmea.h. Integer-only NMEA 0183 (RMC + GGA). */
#include "gps_nmea.h"
#include <string.h>

void gps_init(gps_t *g)
{
    memset(g, 0, sizeof *g);
}

/* ---- small integer helpers -------------------------------------------- */

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* integer part of a decimal string ("084.4" -> 84, "-12.0" -> -12) */
static long num_int(const char *s)
{
    int neg = 0;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') s++;
    long v = 0;
    while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
    return neg ? -v : v;
}

/* value * 10, one fractional digit kept ("022.4" -> 224) */
static long num_tenths(const char *s)
{
    int neg = 0;
    if (*s == '-') { neg = 1; s++; }
    long v = 0;
    while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
    v *= 10;
    if (*s == '.' && s[1] >= '0' && s[1] <= '9')
        v += s[1] - '0';
    return neg ? -v : v;
}

/* "DDDMM.mmmmm" + hemisphere -> degrees * 1e5 (signed). ok=false if empty. */
static int32_t coord_e5(const char *s, char hemi, bool *ok)
{
    const char *dot = strchr(s, '.');
    int intlen = dot ? (int)(dot - s) : (int)strlen(s);
    if (intlen < 3) { *ok = false; return 0; }        /* need at least D MM   */

    long deg = 0;
    for (int i = 0; i < intlen - 2; i++) {
        if (s[i] < '0' || s[i] > '9') { *ok = false; return 0; }
        deg = deg * 10 + (s[i] - '0');
    }
    long minw = (s[intlen - 2] - '0') * 10 + (s[intlen - 1] - '0');

    long frac = 0; int fd = 0;
    if (dot)
        for (const char *p = dot + 1; *p >= '0' && *p <= '9' && fd < 5; p++, fd++)
            frac = frac * 10 + (*p - '0');
    while (fd++ < 5) frac *= 10;

    long min_e5 = minw * 100000L + frac;              /* minutes * 1e5        */
    long e5     = deg * 100000L + (min_e5 + 30) / 60; /* + rounding           */
    if (hemi == 'S' || hemi == 'W' || hemi == 's' || hemi == 'w')
        e5 = -e5;
    *ok = true;
    return (int32_t)e5;
}

/* ---- sentence dispatch ----------------------------------------------- */

/* split `body` (no '$', no '*cs') in place into up to `max` field pointers */
static int split(char *body, char **f, int max)
{
    int n = 0;
    f[n++] = body;
    for (char *p = body; *p && n < max; p++)
        if (*p == ',') { *p = 0; f[n++] = p + 1; }
    return n;
}

static void do_rmc(gps_t *g, char **f, int nf)
{
    /* 0:RMC 1:time 2:status 3:lat 4:N/S 5:lon 6:E/W 7:knots 8:course ... */
    if (nf < 9) return;
    bool a = (f[2][0] == 'A');
    g->fix.valid = a;
    g->fix.seq++;
    if (!a) return;

    bool ok1 = false, ok2 = false;
    int32_t la = coord_e5(f[3], f[4][0], &ok1);
    int32_t lo = coord_e5(f[5], f[6][0], &ok2);
    if (ok1 && ok2) { g->fix.lat_e5 = la; g->fix.lon_e5 = lo; }

    long kn10 = num_tenths(f[7]);                 /* knots * 10               */
    g->fix.speed_kmh   = (uint16_t)((kn10 * 1852 + 5000) / 10000);
    g->fix.course_deg  = (uint16_t)(num_int(f[8]) % 360);
}

static void do_gga(gps_t *g, char **f, int nf)
{
    /* 0:GGA 1:time 2:lat 3:N/S 4:lon 5:E/W 6:qual 7:sats 8:hdop 9:alt 10:M .. */
    if (nf < 10) return;
    g->fix.sats = (uint8_t)num_int(f[7]);
    if (f[6][0] != '0' && f[6][0] != 0)          /* fix quality > 0          */
        g->fix.alt_m = (int16_t)num_int(f[9]);
}

/* parse one accumulated sentence (without CR/LF). returns true if RMC/GGA. */
static bool parse(gps_t *g)
{
    char *s = g->buf;
    if (s[0] != '$') return false;

    /* checksum: XOR of everything between '$' and '*' */
    char *star = strchr(s, '*');
    if (star && star[1] && star[2]) {
        uint8_t cs = 0;
        for (char *p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
        int hi = hexval(star[1]), lo = hexval(star[2]);
        if (hi < 0 || lo < 0 || cs != (uint8_t)((hi << 4) | lo))
            return false;
        *star = 0;                                /* drop the "*CS"          */
    }

    char *body = s + 1;                           /* skip '$'                 */
    char *type = body;                            /* "GPRMC" / "GNGGA" / ... */
    if (strlen(type) < 5) return false;
    const char *t3 = type + 2;                    /* talker prefix is 2 chars */

    char *f[24];
    int nf = split(body, f, 24);

    if (!strncmp(t3, "RMC", 3)) { do_rmc(g, f, nf); return true; }
    if (!strncmp(t3, "GGA", 3)) { do_gga(g, f, nf); return true; }
    return false;
}

bool gps_feed(gps_t *g, char c)
{
    if (c == '$') { g->in = true; g->len = 0; g->buf[g->len++] = c; return false; }
    if (!g->in) return false;

    if (c == '\r' || c == '\n') {
        g->buf[g->len] = 0;
        g->in = false;
        bool hit = (g->len > 6) && parse(g);
        g->len = 0;
        return hit;
    }

    if (g->len < sizeof g->buf - 1)
        g->buf[g->len++] = c;
    else
        g->in = false;                            /* overrun -> resync on '$' */
    return false;
}
