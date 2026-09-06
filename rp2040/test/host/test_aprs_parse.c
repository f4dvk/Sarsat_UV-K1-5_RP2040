/* test_aprs_parse — APRS info-field parser + geo helper. */
#include <stdio.h>
#include <string.h>
#include "ax25.h"
#include "aprs_parse.h"

static int fails;

static int frame(uint8_t *f, const char *dcall, uint8_t dssid,
                 const char *info, int infolen)
{
    ax25_addr_t dst, src = { "F4DVK", 9 };
    memset(&dst, 0, sizeof dst);
    strncpy(dst.call, dcall, 6);
    dst.ssid = dssid;
    return ax25_build_ui(f, &dst, &src, NULL, 0, info, infolen);
}

static void chk(const char *name, int cond)
{
    printf("  %-28s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

static void near_e5(const char *n, int32_t got, int32_t want)
{
    int d = got - want; if (d < 0) d = -d;
    printf("  %-28s got %d want %d  %s\n", n, got, want, d <= 30 ? "OK" : "FAIL");
    if (d > 30) fails++;
}

int main(void)
{
    uint8_t f[AX25_MAX_FRAME];
    aprs_info_t p;

    printf("APRS info-field parser:\n");

    /* 1. uncompressed position + course/speed + altitude + comment */
    {
        const char *i = "!4237.50N/00121.30E>088/012/A=000500En route";
        int fl = frame(f, "APZSAR", 0, i, (int)strlen(i));
        chk("uncompressed parses", aprs_parse(f, fl - 2, &p) && p.kind == APRS_KIND_POSITION);
        near_e5("  lat", p.lat_e5, 4262500);   /* 42 37.50' N */
        near_e5("  lon", p.lon_e5, 135500);    /* 001 21.30' E */
        chk("  symbol / >", p.sym_table == '/' && p.sym_code == '>');
        chk("  course 88",  p.has_course && p.course_deg == 88);
        chk("  speed ~22km", p.has_course && p.speed_kmh >= 20 && p.speed_kmh <= 24);
        chk("  alt ~152m",  p.has_alt && p.alt_m >= 150 && p.alt_m <= 155);
        chk("  comment",    strcmp(p.text, "En route") == 0);
    }

    /* 2. timestamped position (@) */
    {
        const char *i = "@092345z4830.00N/00200.00E-Paris";
        int fl = frame(f, "APZSAR", 0, i, (int)strlen(i));
        chk("timestamped @ parses", aprs_parse(f, fl - 2, &p) && p.has_pos);
        near_e5("  lat", p.lat_e5, 4850000);
        chk("  comment Paris", strcmp(p.text, "Paris") == 0);
    }

    /* 3. status */
    {
        const char *i = ">Monitoring 144.800";
        int fl = frame(f, "APZSAR", 0, i, (int)strlen(i));
        chk("status parses", aprs_parse(f, fl - 2, &p) && p.kind == APRS_KIND_STATUS);
        chk("  text", strcmp(p.text, "Monitoring 144.800") == 0);
    }

    /* 4. message */
    {
        const char *i = ":F1ABC    :hello there{1";
        int fl = frame(f, "APZSAR", 0, i, (int)strlen(i));
        chk("message parses", aprs_parse(f, fl - 2, &p) && p.kind == APRS_KIND_MESSAGE);
        chk("  addressee", strncmp(p.name, "F1ABC", 5) == 0);
        chk("  body", strncmp(p.text, "hello there", 11) == 0);
    }

    /* 5. MIC-E : 33 25.64 N, 112 07.90 W (dest SSRUVT) */
    {
        char info[16];
        int k = 0;
        info[k++] = 0x60;            /* `  */
        info[k++] = (char)(12 + 28); /* lon deg 12 (+100) */
        info[k++] = (char)(7 + 28);  /* lon min 7 */
        info[k++] = (char)(90 + 28); /* lon hun 90 */
        info[k++] = 0x1C; info[k++] = 0x1C; info[k++] = 0x1C;  /* sp/cse 0 */
        info[k++] = '>';             /* sym code */
        info[k++] = '/';             /* sym table */
        int fl = frame(f, "SSRUVT", 0, info, k);
        chk("MIC-E parses", aprs_parse(f, fl - 2, &p) && p.has_pos);
        near_e5("  lat", p.lat_e5, 3342733);
        near_e5("  lon", p.lon_e5, -11213167);
        chk("  symbol / >", p.sym_table == '/' && p.sym_code == '>');
    }

    /* 4b. message ACK (the "121 MHz report" reply path): a 9-char padded
     * addressee and an "ackNN" body must both come through so the radio can
     * match its own pending report -- see firmware .../aprs.c APRS_MsgCheckAck */
    {
        const char *i = ":F4DVK-9  :ack07";
        int fl = frame(f, "APZSAR", 0, i, (int)strlen(i));
        chk("ack parses as message", aprs_parse(f, fl - 2, &p) && p.kind == APRS_KIND_MESSAGE);
        chk("  addressee F4DVK-9", strncmp(p.name, "F4DVK-9", 7) == 0);
        chk("  body ack07", strcmp(p.text, "ack07") == 0);
    }

    /* 5b. digipeater path (H bit), WIDE aliases stripped */
    {
        ax25_addr_t dst = { "APZSAR", 0 }, src = { "F4DVK", 9 };
        ax25_addr_t digi[2] = { { "F8KCS", 3 }, { "WIDE2", 1 } };
        int fl = ax25_build_ui(f, &dst, &src, digi, 2, "!0000.00N/00000.00E>", 20);
        char via[28];

        chk("direct: no H bit -> empty", ax25_via_str(f, fl - 2, via, sizeof via) == 0
                                         && via[0] == 0);

        f[27] |= 0x80;                    /* only WIDE2-1 marked -> alias, ignore */
        chk("WIDE-only -> still empty", ax25_via_str(f, fl - 2, via, sizeof via) == 0
                                       && via[0] == 0);

        f[20] |= 0x80;                    /* F8KCS-3 also repeated */
        chk("F8KCS-3,WIDE2* -> F8KCS-3", ax25_via_str(f, fl - 2, via, sizeof via) == 1
                                         && strcmp(via, "F8KCS-3") == 0);

        digi[1].ssid = 2; strncpy(digi[1].call, "F8KCS", 6);
        fl = ax25_build_ui(f, &dst, &src, digi, 2, "!0000.00N/00000.00E>", 20);
        f[20] |= 0x80; f[27] |= 0x80;     /* both real digis repeated */
        chk("two digis -> F8KCS-3,F8KCS-2",
            ax25_via_str(f, fl - 2, via, sizeof via) == 2
            && strcmp(via, "F8KCS-3,F8KCS-2") == 0);
    }

    /* 6. geo: Paris -> London ~344 km, bearing ~330 deg */
    {
        uint32_t dm; uint16_t br;
        int ok = aprs_geo(4885660, 235220, 5150740, -12780, &dm, &br);
        printf("  %-28s %lu m  bearing %u\n", "geo Paris->London", (unsigned long)dm, br);
        chk("  distance 330-360 km", ok && dm > 330000 && dm < 360000);
        chk("  bearing 320-340",     br >= 320 && br <= 340);
    }

    printf(fails ? "\nFAIL\n" : "\nPASS\n");
    return fails ? 1 : 0;
}
