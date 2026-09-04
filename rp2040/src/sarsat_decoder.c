/* sarsat_decoder.c — see sarsat_decoder.h. */
#include "sarsat_decoder.h"
#include "audio_slicer.h"
#include "country_codes.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

static const char *protocol_name(ProtocolType p)
{
    switch (p) {
    case PROTOCOL_STANDARD_LOCATION: return "Standard Location";
    case PROTOCOL_NATIONAL_LOCATION: return "National Location";
    case PROTOCOL_USER_PROTOCOL:     return "User-Location";
    case PROTOCOL_TEST:              return "Test";
    case PROTOCOL_EMERGENCY_ELT:     return "ELT-DT Location";
    case PROTOCOL_EMERGENCY_EPIRB:   return "Emergency EPIRB";
    case PROTOCOL_EMERGENCY_PLB:     return "Emergency PLB";
    case PROTOCOL_RLS_LOCATION:      return "RLS Location";
    case PROTOCOL_SHIP_SECURITY:     return "Ship Security";
    default:                         return "Unknown";
    }
}

static void put_line(sarsat_result_t *o, const char *fmt, ...)
{
    if (o->n_lines >= SARSAT_MAX_LINES) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(o->lines[o->n_lines], SARSAT_LINE_CHARS, fmt, ap);
    va_end(ap);
    o->n_lines++;
}

/* Emit `s` across as many lines as needed, breaking on spaces (<= 21 glyphs
 * per line). An optional `prefix` goes on the first line only. */
static void put_wrapped(sarsat_result_t *o, const char *prefix, const char *s)
{
    const int W = SARSAT_LINE_CHARS - 1;
    char buf[SARSAT_LINE_CHARS];
    int col = 0;

    if (prefix && *prefix) {
        col = (int)strlen(prefix);
        if (col > W) col = W;
        memcpy(buf, prefix, col);
    }

    while (*s == ' ') s++;
    while (*s && o->n_lines < SARSAT_MAX_LINES) {
        /* length of the next word */
        int wl = 0;
        while (s[wl] && s[wl] != ' ') wl++;

        if (col > 0 && col + 1 + wl > W) {          /* word won't fit -> flush */
            buf[col] = 0;
            put_line(o, "%s", buf);
            col = 0;
        }
        if (col > 0) buf[col++] = ' ';
        if (wl > W) wl = W;                          /* hard-split huge tokens */
        memcpy(buf + col, s, wl);
        col += wl;
        s += wl;
        while (*s == ' ') s++;
    }
    if (col > 0) { buf[col] = 0; put_line(o, "%s", buf); }
}

void sarsat_format_lines(sarsat_result_t *o)
{
    const BeaconInfo1G *in = &o->info;
    o->n_lines = 0;

    put_line(o, "1G %s%s  proto %u",
             (o->frame_bits == 144) ? "LONG" : "SHORT",
             in->is_test_message ? " TEST" : "", in->protocol_bits);
    put_line(o, "ID %s", o->hex_id);
    put_wrapped(o, "", protocol_name(in->protocol));
    put_line(o, "Country %u", in->country_code);
    put_wrapped(o, "", get_country_name(in->country_code));

    if (in->position_default || (!in->has_position)) {
        put_line(o, "Position: none");
    } else if (in->lat != 0.0 || in->lon != 0.0) {
        put_line(o, "Lat %.5f %c",
                 fabs(in->lat), (in->lat >= 0) ? 'N' : 'S');
        put_line(o, "Lon %.5f %c",
                 fabs(in->lon), (in->lon >= 0) ? 'E' : 'W');
        /* composite vs base (ELT-DT / National with offsets) */
        if ((in->base_lat != in->lat || in->base_lon != in->lon) &&
            (in->base_lat != 0.0 || in->base_lon != 0.0))
            put_line(o, "base %.3f %.3f", in->base_lat, in->base_lon);
    } else {
        put_line(o, "Position: invalid");
    }

    if (in->vessel_id[0])
        put_wrapped(o, "", in->vessel_id);
}

int sarsat_decode_window(const int32_t *samples, int n, int rate,
                         sarsat_result_t *out)
{
    memset(out, 0, sizeof(*out));

    uint8_t bits[SLICER_MAX_BITS];
    int len = slicer_run(samples, n, rate, bits);
    if (len != 112 && len != 144)
        return 0;                       /* no sync / no frame */

    out->frame_bits = len;

    char frame[145];
    for (int i = 0; i < len; i++) frame[i] = bits[i] ? '1' : '0';
    frame[len] = '\0';

    /* raw hex, 4 bits per nibble, sync bits included (matches dec406_hex) */
    for (int i = 0; i * 4 < len && i < (int)sizeof(out->raw_hex) - 1; i++) {
        int v = 0;
        for (int b = 0; b < 4 && i * 4 + b < len; b++)
            v = (v << 1) | bits[i * 4 + b];
        out->raw_hex[i] = "0123456789ABCDEF"[v & 0xF];
    }

    decode_1g_frame(frame, len, &out->info);
    out->crc_ok = !out->info.crc_error;
    if (!out->crc_ok)
        return -1;                      /* frame sliced but BCH uncorrectable */

    out->valid = 1;
    snprintf(out->hex_id, sizeof(out->hex_id), "%s", out->info.hex_id);
    sarsat_format_lines(out);
    return 1;
}
