/* aprs_parse.c — APRS info-field parser + geo helper. See aprs_parse.h. */
#include "aprs_parse.h"
#include <string.h>

/* ---------------------------------------------------------------- utils ---- */
static int digit2(const char *p)   { return (p[0]-'0')*10 + (p[1]-'0'); }
static bool isdig(char c)          { return c >= '0' && c <= '9'; }

static void copy_str(char *dst, int cap, const char *src, int n)
{
    int k = 0;
    for (int i = 0; i < n && k < cap - 1; i++) {
        char c = src[i];
        if (c == '\r' || c == '\n') break;
        dst[k++] = (c >= 0x20 && c < 0x7F) ? c : '.';
    }
    dst[k] = 0;
}

/* base-91 digit used by compressed APRS */
static int b91(char c) { return (c >= '!' && c <= '{') ? (c - '!') : 0; }

/* course "CSE/SPD" (deg / knots) trailing an uncompressed position */
static void take_course_speed(aprs_info_t *o, const char *p, int n)
{
    if (n >= 7 && isdig(p[0]) && isdig(p[1]) && isdig(p[2]) && p[3] == '/' &&
        isdig(p[4]) && isdig(p[5]) && isdig(p[6])) {
        int crs = (p[0]-'0')*100 + (p[1]-'0')*10 + (p[2]-'0');
        int spd_kt = (p[4]-'0')*100 + (p[5]-'0')*10 + (p[6]-'0');
        o->has_course  = true;
        o->course_deg  = (int16_t)(crs % 360);
        o->speed_kmh   = (int16_t)((spd_kt * 1852 + 500) / 1000);
    }
}

/* "/A=nnnnnn" altitude in feet, anywhere in the comment */
static void take_altitude(aprs_info_t *o, const char *p, int n)
{
    for (int i = 0; i + 9 <= n; i++) {
        if (p[i] == '/' && p[i+1] == 'A' && p[i+2] == '=') {
            long ft = 0;
            bool ok = true;
            for (int k = 0; k < 6; k++) {
                if (!isdig(p[i+3+k])) { ok = false; break; }
                ft = ft * 10 + (p[i+3+k] - '0');
            }
            if (ok) { o->has_alt = true; o->alt_m = (int32_t)((ft * 3048) / 10000); }
            return;
        }
    }
}

/* ----------------------------------------------------- uncompressed pos ---- */
static bool parse_uncompressed(const char *d, int n, aprs_info_t *o)
{
    /* "DDMM.mmN" (8) | sym table (1) | "DDDMM.mmW" (9) | sym code (1) | rest */
    if (n < 19) return false;
    if (!(d[7] == 'N' || d[7] == 'S')) return false;
    if (!(d[17] == 'E' || d[17] == 'W')) return false;

    int lat = digit2(d) * 100000
            + ((digit2(d + 2) * 100 + digit2(d + 5)) * 1000 / 60);
    int lon_deg = (d[9]-'0')*100 + (d[10]-'0')*10 + (d[11]-'0');
    int lon = lon_deg * 100000
            + ((digit2(d + 12) * 100 + digit2(d + 15)) * 1000 / 60);
    o->lat_e5 = (d[7]  == 'S') ? -lat : lat;
    o->lon_e5 = (d[17] == 'W') ? -lon : lon;
    o->sym_table = d[8];
    o->sym_code  = d[18];
    o->has_pos   = true;

    const char *rest = d + 19;
    int rn = n - 19;
    take_course_speed(o, rest, rn);
    if (o->has_course) { rest += 7; rn -= 7; }
    take_altitude(o, rest, rn);
    if (rn >= 9 && rest[0] == '/' && rest[1] == 'A' && rest[2] == '=') {
        rest += 9; rn -= 9;                 /* strip "/A=nnnnnn" from the text */
    }
    copy_str(o->text, sizeof o->text, rest, rn);
    return true;
}

/* ------------------------------------------------------- compressed pos ---- */
static bool parse_compressed(const char *d, int n, aprs_info_t *o)
{
    /* d[0] = sym table / overlay, d[1..4] lat, d[5..8] lon, d[9] sym code,
     * d[10..11] cs, d[12] comp type */
    if (n < 13) return false;
    long y = ((long)b91(d[1])*753571L) + b91(d[2])*8281 + b91(d[3])*91 + b91(d[4]);
    long x = ((long)b91(d[5])*753571L) + b91(d[6])*8281 + b91(d[7])*91 + b91(d[8]);
    o->lat_e5 = (int32_t)(9000000L - (y * 90 * 100 / 380926));   /* 90 - y/380926 deg */
    o->lon_e5 = (int32_t)(-18000000L + (x * 360 * 10 / 190463)); /* -180 + x/190463 deg */
    o->sym_table = d[0];
    o->sym_code  = d[9];
    o->has_pos   = true;
    copy_str(o->text, sizeof o->text, d + 13, n - 13);
    take_altitude(o, d + 13, n - 13);
    return true;
}

/* -------------------------------------------------------------- MIC-E ------ */
static int micE_digit(char c, int *msg /*or NULL*/, int *ns_we /*or NULL*/)
{
    if (c >= '0' && c <= '9') { if (msg) *msg = 0; if (ns_we) *ns_we = 0; return c - '0'; }
    if (c >= 'A' && c <= 'J') { if (msg) *msg = 2; if (ns_we) *ns_we = 1; return c - 'A'; }
    if (c >= 'P' && c <= 'Y') { if (msg) *msg = 1; if (ns_we) *ns_we = 1; return c - 'P'; }
    if (c == 'K' || c == 'L' || c == 'Z') { if (ns_we) *ns_we = (c != 'L'); return 0; }
    return 0;
}

static bool parse_micE(const uint8_t *ax25, int len, const uint8_t *info, int n,
                       aprs_info_t *o)
{
    if (n < 9) return false;
    char de[6];
    for (int i = 0; i < 6; i++) de[i] = (char)(ax25[i] >> 1);  /* dst addr */

    int ns = 0, ofs = 0, we = 0;
    int d0 = micE_digit(de[0], NULL, NULL);
    int d1 = micE_digit(de[1], NULL, NULL);
    int d2 = micE_digit(de[2], NULL, NULL);
    int d3 = micE_digit(de[3], NULL, &ns);
    int d4 = micE_digit(de[4], NULL, &ofs);
    int d5 = micE_digit(de[5], NULL, &we);

    long lat = (d0*10 + d1) * 100000L + ((d2*10 + d3) * 100000L + (d4*10 + d5) * 1000L) / 60;
    o->lat_e5 = ns ? (int32_t)lat : -(int32_t)lat;

    int lon_deg = (info[1] - 28);
    if (ofs) lon_deg += 100;
    if (lon_deg >= 180 && lon_deg <= 189) lon_deg -= 80;
    else if (lon_deg >= 190 && lon_deg <= 199) lon_deg -= 190;
    int lon_min = info[2] - 28;  if (lon_min >= 60) lon_min -= 60;
    int lon_hun = info[3] - 28;
    long lon = lon_deg * 100000L + (lon_min * 100000L + lon_hun * 1000L) / 60;
    o->lon_e5 = we ? -(int32_t)lon : (int32_t)lon;
    o->has_pos = true;

    int sp = info[4] - 28, dc = info[5] - 28, se = info[6] - 28;
    int speed_kt = sp * 10 + dc / 10;
    if (speed_kt >= 800) speed_kt -= 800;
    int course = (dc % 10) * 100 + se;
    if (course >= 400) course -= 400;
    o->has_course = true;
    o->course_deg = (int16_t)(course % 360);
    o->speed_kmh  = (int16_t)((speed_kt * 1852 + 500) / 1000);

    o->sym_code  = (char)info[7];
    o->sym_table = (char)info[8];

    /* comment (may carry "xxx}" Base-91 altitude above the 10 000 m datum) */
    const char *cm = (const char *)info + 9;
    int cn = n - 9;
    if (cn >= 4 && cm[3] == '}') {
        long a = (long)b91(cm[0]) * 8281 + b91(cm[1]) * 91 + b91(cm[2]) - 10000;
        o->has_alt = true; o->alt_m = (int32_t)a;
        cm += 4; cn -= 4;
    }
    take_altitude(o, cm, cn);
    copy_str(o->text, sizeof o->text, cm, cn);
    (void)len;
    return true;
}

/* ---------------------------------------------------------- src address ---- */
static void get_src(const uint8_t *ax25, char *out)
{
    int k = 0;
    for (int i = 0; i < 6; i++) {
        char c = (char)(ax25[7 + i] >> 1);
        if (c != ' ' && k < 8) out[k++] = c;
    }
    int ssid = (ax25[13] >> 1) & 0x0F;
    if (ssid && k < 9) {
        out[k++] = '-';
        if (ssid >= 10 && k < 9) { out[k++] = '1'; ssid -= 10; }
        if (k < 9) out[k++] = (char)('0' + ssid);
    }
    out[k] = 0;
}

/* ---------------------------------------------------------- digi path ------ */
/* WIDEn / TRACEn / RELAY / TEMPn / ECHO / GATE -- generic path aliases, not a
 * real station, so they're skipped in the "Via" display. */
static bool generic_alias(const char *c, int k)
{
    if (k >= 4 && memcmp(c, "WIDE",  4) == 0) return true;
    if (k >= 5 && memcmp(c, "TRACE", 5) == 0) return true;
    if (k >= 4 && memcmp(c, "TEMP",  4) == 0) return true;
    if (k == 5 && memcmp(c, "RELAY", 5) == 0) return true;
    if (k == 4 && memcmp(c, "ECHO",  4) == 0) return true;
    if (k == 4 && memcmp(c, "GATE",  4) == 0) return true;
    return false;
}

/* The digipeaters that actually repeated the frame (AX.25 H bit set), in order,
 * comma-separated -- but WIDEn/TRACEn/RELAY-style path aliases are dropped, so
 * "F8KCS-3,F8KCS-2,WIDE2*" comes out "F8KCS-3,F8KCS-2". Empty = heard directly
 * (or only via generic hops). Returns the callsign count. */
int ax25_via_str(const uint8_t *ax25, int len, char *out, int cap)
{
    out[0] = 0;
    if (len < 14 || cap < 2)
        return 0;

    int pos = 14, w = 0, count = 0;
    bool ext = (ax25[13] & 1);                 /* src SSID byte ends the addrs? */
    while (!ext && pos + 7 <= len) {
        ext = (ax25[pos + 6] & 1);
        if (ax25[pos + 6] & 0x80) {            /* H bit = this hop repeated it */
            char call[8];
            int  k = 0;
            for (int i = 0; i < 6; i++) {
                char c = (char)(ax25[pos + i] >> 1);
                if (c > ' ' && c < 0x7f) call[k++] = c;
            }
            call[k] = 0;
            if (!generic_alias(call, k) && w + 11 < cap) {
                int ssid = (ax25[pos + 6] >> 1) & 0x0F;
                if (count) out[w++] = ',';
                for (int i = 0; i < k; i++) out[w++] = call[i];
                if (ssid) {
                    out[w++] = '-';
                    if (ssid >= 10) { out[w++] = '1'; ssid -= 10; }
                    out[w++] = (char)('0' + ssid);
                }
                out[w] = 0;
                count++;
            }
        }
        pos += 7;
    }
    return count;
}

/* -------------------------------------------------------------- parse ------ */
bool aprs_parse(const uint8_t *ax25, int len, aprs_info_t *out)
{
    memset(out, 0, sizeof *out);
    if (len < 16) return false;
    get_src(ax25, out->src);

    /* skip dst(7) + src(7) + digis, then control + PID */
    int pos = 14;
    bool ext = (ax25[13] & 1);
    while (!ext && pos + 7 <= len - 2) { ext = (ax25[pos + 6] & 1); pos += 7; }
    const uint8_t *info = ax25 + pos + 2;
    int n = len - (pos + 2);
    if (n <= 0) return true;

    char t = (char)info[0];

    if (t == 0x1C || t == 0x1D || t == '`' || t == '\'') {
        out->kind = APRS_KIND_POSITION;
        return parse_micE(ax25, len, info, n, out) || true;
    }

    if (t == '!' || t == '=' || t == '@' || t == '/') {
        out->kind = APRS_KIND_POSITION;
        const char *d = (const char *)info + 1;
        int dn = n - 1;
        if (t == '@' || t == '/') {           /* 7-char timestamp */
            if (dn < 7) return true;
            d += 7; dn -= 7;
        }
        if (dn > 0 && (*d == '/' || *d == '\\' ||
                       (*d >= 'A' && *d <= 'Z') || (*d >= 'a' && *d <= 'j')))
            parse_compressed(d, dn, out);
        else
            parse_uncompressed(d, dn, out);
        return true;
    }

    if (t == ';' && n >= 30) {                 /* object */
        out->kind = APRS_KIND_OBJECT;
        copy_str(out->name, sizeof out->name, (const char *)info + 1, 9);
        const char *d = (const char *)info + 18;  /* after name(9)+flag(1)+ts(7) */
        int dn = n - 18;
        if (dn > 0 && (*d == '/' || *d == '\\'))
            parse_compressed(d, dn, out);
        else
            parse_uncompressed(d, dn, out);
        return true;
    }

    if (t == ')' && n >= 18) {                 /* item */
        out->kind = APRS_KIND_OBJECT;
        int i = 1;
        while (i < n && i < 10 && info[i] != '!' && info[i] != '_') i++;
        copy_str(out->name, sizeof out->name, (const char *)info + 1, i - 1);
        parse_uncompressed((const char *)info + i + 1, n - i - 1, out);
        return true;
    }

    if (t == '>') {
        out->kind = APRS_KIND_STATUS;
        copy_str(out->text, sizeof out->text, (const char *)info + 1, n - 1);
        return true;
    }

    if (t == ':' && n >= 11) {                 /* message */
        out->kind = APRS_KIND_MESSAGE;
        copy_str(out->name, sizeof out->name, (const char *)info + 1, 9);
        int c = 10;
        while (c < n && info[c] != ':') c++;
        copy_str(out->text, sizeof out->text, (const char *)info + c + 1, n - c - 1);
        return true;
    }

    if (t == 'T') {
        out->kind = APRS_KIND_TELEMETRY;
        copy_str(out->text, sizeof out->text, (const char *)info, n);
        return true;
    }

    out->kind = APRS_KIND_OTHER;
    copy_str(out->text, sizeof out->text, (const char *)info, n);
    return true;
}

/* --------------------------------------------------------------- geo ------- */
static const int16_t COS_Q12[91] = {
     4096,  4095,  4094,  4090,  4086,  4080,  4074,  4065,  4056,  4046,
     4034,  4021,  4006,  3991,  3974,  3956,  3937,  3917,  3896,  3873,
     3849,  3824,  3798,  3770,  3742,  3712,  3681,  3650,  3617,  3582,
     3547,  3511,  3474,  3435,  3396,  3355,  3314,  3271,  3228,  3183,
     3138,  3091,  3044,  2996,  2946,  2896,  2845,  2793,  2741,  2687,
     2633,  2578,  2522,  2465,  2408,  2349,  2290,  2231,  2171,  2110,
     2048,  1986,  1923,  1860,  1796,  1731,  1666,  1600,  1534,  1468,
     1401,  1334,  1266,  1198,  1129,  1060,   991,   921,   852,   782,
      711,   641,   570,   499,   428,   357,   286,   214,   143,    71,
        0,
};
static const uint8_t ATAN16_DEG[17] =
    { 0, 4, 7, 11, 14, 17, 21, 24, 27, 29, 32, 35, 37, 39, 41, 43, 45 };

static uint32_t isqrt64(uint64_t v)
{
    uint64_t x = v, r = 0, b = (uint64_t)1 << 62;
    while (b > x) b >>= 2;
    while (b) {
        if (x >= r + b) { x -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return (uint32_t)r;
}

static int atan_deg(int64_t num, int64_t den)   /* num <= den, both >= 0 -> 0..45 */
{
    if (den == 0) return 0;
    int r = (int)((num * 16) / den);            /* 0..16 */
    if (r > 15) return 45;
    int frac = (int)(((num * 16) % den) * 16 / den);
    return ATAN16_DEG[r] + ((ATAN16_DEG[r + 1] - ATAN16_DEG[r]) * frac) / 16;
}

bool aprs_geo(int32_t lat1_e5, int32_t lon1_e5,
              int32_t lat2_e5, int32_t lon2_e5,
              uint32_t *dist_m, uint16_t *bearing_deg)
{
    if (lat1_e5 >  9000000 || lat1_e5 < -9000000 ||
        lat2_e5 >  9000000 || lat2_e5 < -9000000)
        return false;

    int32_t mlat = (lat1_e5 + lat2_e5) / 2;
    int      md  = (mlat < 0 ? -mlat : mlat) / 100000;
    if (md > 90) md = 90;
    int64_t cosm = COS_Q12[md];

    int64_t dlat  = lat2_e5 - lat1_e5;                       /* e5 deg */
    int64_t deast = (int64_t)(lon2_e5 - lon1_e5) * cosm / 4096;

    uint64_t d2 = (uint64_t)(dlat * dlat) + (uint64_t)(deast * deast);
    uint32_t d_e5 = isqrt64(d2);
    *dist_m = (uint32_t)(((uint64_t)d_e5 * 11132) / 10000);  /* 1e-5 deg -> m */

    /* compass bearing: 0 = north, 90 = east */
    int64_t ax = deast < 0 ? -deast : deast;
    int64_t ay = dlat  < 0 ? -dlat  : dlat;
    int a = (ax <= ay) ? atan_deg(ax, ay) : 90 - atan_deg(ay, ax);
    int q;
    if      (dlat >= 0 && deast >= 0) q = a;
    else if (dlat <  0 && deast >= 0) q = 180 - a;
    else if (dlat <  0 && deast <  0) q = 180 + a;
    else                             q = 360 - a;
    *bearing_deg = (uint16_t)(q % 360);
    return true;
}
