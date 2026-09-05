/* test_aprs_digi — WIDEn-N digipeat decision + dedup (see aprs_digi.h). */
#include <stdio.h>
#include <string.h>
#include "ax25.h"
#include "aprs_digi.h"

static int fails;

static void chk(const char *name, int cond)
{
    printf("  %-32s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

/* Build "SRC-ssid > APZSAR" with 0..2 digis, returns the frame length
 * (FCS-stripped, matching what aprs_rx's packet callback delivers). */
static int frame_src(uint8_t *f, const char *src, int src_ssid,
                     const char *w1, int n1, const char *w2, int n2)
{
    ax25_addr_t dst = { "APZSAR", 0 }, s = { "", (uint8_t)src_ssid };
    strncpy(s.call, src, 6);
    ax25_addr_t digi[2];
    int nd = 0;
    if (w1) { memset(&digi[nd], 0, sizeof digi[nd]); strncpy(digi[nd].call, w1, 6); digi[nd].ssid = (uint8_t)n1; nd++; }
    if (w2) { memset(&digi[nd], 0, sizeof digi[nd]); strncpy(digi[nd].call, w2, 6); digi[nd].ssid = (uint8_t)n2; nd++; }
    const char *info = "!4237.50N/00121.30E>test";
    uint8_t full[AX25_MAX_FRAME];
    int flen = ax25_build_ui(full, &dst, &s, digi, nd, info, (int)strlen(info));
    int len = flen - 2;                /* drop the 2 FCS bytes */
    memcpy(f, full, (size_t)len);
    return len;
}
static int frame(uint8_t *f, const char *w1, int n1, const char *w2, int n2)
{
    return frame_src(f, "F4DVK", 9, w1, n1, w2, n2);
}

/* mark a digi slot (0-based index into the digi list, i.e. address #2+idx)
 * as already repeated (H-bit set). */
static void mark_used(uint8_t *f, int idx)
{
    f[14 + idx * 7 + 6] |= 0x80;
}

static int ssid_of(const uint8_t *f, int idx)
{
    return (f[14 + idx * 7 + 6] >> 1) & 0x0F;
}
static bool used(const uint8_t *f, int idx)
{
    return (f[14 + idx * 7 + 6] & 0x80) != 0;
}
/* decode the 6-char shifted-ASCII callsign of a digi slot, trimmed of
 * trailing spaces, for comparing against an expected substituted identity. */
static void call_of(const uint8_t *f, int idx, char out[7])
{
    int base = 14 + idx * 7;
    int n = 6;
    for (int i = 0; i < 6; i++) out[i] = (char)(f[base + i] >> 1);
    while (n > 0 && out[n - 1] == ' ') n--;
    out[n] = 0;
}

int main(void)
{
    printf("APRS digipeat decision (WIDEn-N), no callsign configured:\n");

    /* 1. WIDE1-1, digi_level=WIDE1: single hop, alias kept, H-bit set. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE1", 1, NULL, 0);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE1, NULL, 0);
        char c[7]; call_of(f, 0, c);
        chk("WIDE1-1 -> repeat once, used, no trace",
            n == len && strcmp(c, "WIDE1") == 0 && ssid_of(f, 0) == 0 && used(f, 0));
    }

    /* 2. WIDE2-2, digi_level=WIDE2: first hop decrements to -1, NOT used yet
     *    (multi-hop fan-out); a second pass finishes it to -0, used. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE2", 2, NULL, 0);
        int n1 = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE2, NULL, 0);
        bool step1 = n1 == len && ssid_of(f, 0) == 1 && !used(f, 0);
        int n2 = aprs_digi_process(f, n1, sizeof f, APRS_DIGI_WIDE2, NULL, 0);
        bool step2 = n2 == len && ssid_of(f, 0) == 0 && used(f, 0);
        chk("WIDE2-2 -> 2-hop fan-out then used", step1 && step2);
    }

    /* 3. digi_level too low: WIDE2-1 alone, digi configured WIDE1 only. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE2", 1, NULL, 0);
        uint8_t before[AX25_MAX_FRAME]; memcpy(before, f, (size_t)len);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE1, NULL, 0);
        chk("WIDE2-1 with level=WIDE1 -> no match",
            n == 0 && memcmp(f, before, (size_t)len) == 0);
    }

    /* 4. "WIDE1*,WIDE2-1" (WIDE1 already used) -> skip it, act on WIDE2-1. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE1", 1, "WIDE2", 1);
        mark_used(f, 0);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE2, NULL, 0);
        chk("WIDE1* skipped, WIDE2-1 repeated",
            n == len && used(f, 0) && ssid_of(f, 1) == 0 && used(f, 1));
    }

    /* 5. digi off entirely. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE1", 1, NULL, 0);
        uint8_t before[AX25_MAX_FRAME]; memcpy(before, f, (size_t)len);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_OFF, NULL, 0);
        chk("Digi off -> never repeats",
            n == 0 && memcmp(f, before, (size_t)len) == 0);
    }

    /* 6. explicit callsign (not a generic alias) as the first unused hop. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "F4ABC", 3, NULL, 0);
        uint8_t before[AX25_MAX_FRAME]; memcpy(before, f, (size_t)len);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE3, NULL, 0);
        chk("Explicit callsign -> not touched",
            n == 0 && memcmp(f, before, (size_t)len) == 0);
    }

    /* 7. direct frame (no digi at all) -> nothing to do. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, NULL, 0, NULL, 0);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE3, NULL, 0);
        chk("Direct frame (no path) -> no match", n == 0);
    }

    printf("Traceable digipeating (callsign insertion), WIDE1/2/3:\n");

    /* 8. WIDE1-1 (last hop): "MYCALL-5*,WIDE1*" -- trace hop inserted ahead
     *    of the alias, which keeps its name and gets H-bit set. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE1", 1, NULL, 0);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE1, "MYCALL", 5);
        char a0[7], a1[7]; call_of(f, 0, a0); call_of(f, 1, a1);
        chk("WIDE1-1 + call -> MYCALL-5*,WIDE1*",
            n == len + 7 &&
            strcmp(a0, "MYCALL") == 0 && ssid_of(f, 0) == 5 && used(f, 0) &&
            strcmp(a1, "WIDE1")  == 0 && ssid_of(f, 1) == 0 && used(f, 1));
    }

    /* 9. WIDE3-1 (last hop, n=3): same pattern, confirms it is not WIDE1/2
     *    specific. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE3", 1, NULL, 0);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE3, "MYCALL", 5);
        char a0[7], a1[7]; call_of(f, 0, a0); call_of(f, 1, a1);
        chk("WIDE3-1 + call -> MYCALL-5*,WIDE3*",
            n == len + 7 &&
            strcmp(a0, "MYCALL") == 0 && used(f, 0) &&
            strcmp(a1, "WIDE3")  == 0 && ssid_of(f, 1) == 0 && used(f, 1));
    }

    /* 10. WIDE2-2 (not the last hop): "MYCALL-5*,WIDE2-1" (frame +7 bytes).
     *     A second digipeater (its own callsign) then finds WIDE2-1 as the
     *     first unused entry, decrements to 0, inserts ITS OWN trace hop
     *     ahead of it, and the alias itself is marked used (still named
     *     "WIDE2", never replaced): final path
     *     "MYCALL-5*,OTHER-3*,WIDE2*". */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE2", 2, NULL, 0);
        int n1 = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE2, "MYCALL", 5);
        char a0[7], a1[7]; call_of(f, 0, a0); call_of(f, 1, a1);
        bool step1 = n1 == len + 7 &&
                     strcmp(a0, "MYCALL") == 0 && ssid_of(f, 0) == 5 && used(f, 0) &&
                     strcmp(a1, "WIDE2")  == 0 && ssid_of(f, 1) == 1 && !used(f, 1);
        chk("WIDE2-2 + call -> MYCALL-5*,WIDE2-1", step1);

        int n2 = aprs_digi_process(f, n1, sizeof f, APRS_DIGI_WIDE2, "OTHER", 3);
        char b0[7], b1[7], b2[7]; call_of(f, 0, b0); call_of(f, 1, b1); call_of(f, 2, b2);
        bool step2 = n2 == n1 + 7 &&
                     strcmp(b0, "MYCALL") == 0 && used(f, 0) &&      /* untouched */
                     strcmp(b1, "OTHER")  == 0 && used(f, 1) &&      /* new trace hop */
                     strcmp(b2, "WIDE2")  == 0 && ssid_of(f, 2) == 0 && used(f, 2);
        chk("2nd digi -> MYCALL-5*,OTHER-3*,WIDE2*", step2);
    }

    /* 11. my_call NULL/blank -> falls back to the plain, no-trace behaviour
     *     (same result as case 1, no insertion attempted). */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE1", 1, NULL, 0);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE1, "      ", 3);  /* blank */
        char c[7]; call_of(f, 0, c);
        chk("Blank call -> plain WIDEn* fallback, no growth",
            n == len && strcmp(c, "WIDE1") == 0 && ssid_of(f, 0) == 0 && used(f, 0));
    }

    /* 12. no room to grow -> falls back to the plain in-place update instead
     *     of refusing to digipeat outright. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE2", 2, NULL, 0);
        int n = aprs_digi_process(f, len, len /* no headroom at all */,
                                  APRS_DIGI_WIDE2, "MYCALL", 5);
        chk("No room to insert -> falls back, no growth",
            n == len && ssid_of(f, 0) == 1 && !used(f, 0));
    }

    printf("Anti-loop guards (own callsign):\n");

    /* 13. the frame's SOURCE is our own callsign-SSID -> never repeat it,
     *     even though the path itself would otherwise match. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame_src(f, "MYCALL", 5, "WIDE1", 1, NULL, 0);
        uint8_t before[AX25_MAX_FRAME]; memcpy(before, f, (size_t)len);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE1, "MYCALL", 5);
        chk("Own source -> never repeated",
            n == 0 && memcmp(f, before, (size_t)len) == 0);
    }

    /* 14. our own callsign already appears in the digi list (used, i.e. we
     *     already traced this exact frame once) -> never touch it again,
     *     even though a further unused generic alias remains. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE2", 1, NULL, 0);   /* build the base path */
        /* insert our own already-used trace hop by hand, ahead of the
         * remaining WIDE2-1, to simulate "we already digipeated this". */
        uint8_t g[AX25_MAX_FRAME];
        memcpy(g, f, 14);
        g[14] = (uint8_t)('M' << 1); g[15] = (uint8_t)('Y' << 1);
        g[16] = (uint8_t)('C' << 1); g[17] = (uint8_t)('A' << 1);
        g[18] = (uint8_t)('L' << 1); g[19] = (uint8_t)('L' << 1);
        g[20] = (uint8_t)(0x60 | 0x80 | (5 << 1));           /* MYCALL-5*, not last */
        memcpy(g + 21, f + 14, (size_t)(len - 14));           /* WIDE2-1 follows */
        int glen = len + 7;

        uint8_t before[AX25_MAX_FRAME]; memcpy(before, g, (size_t)glen);
        int n = aprs_digi_process(g, glen, sizeof g, APRS_DIGI_WIDE2, "MYCALL", 5);
        chk("Own callsign already in path -> never repeated again",
            n == 0 && memcmp(g, before, (size_t)glen) == 0);
    }

    /* 15. a DIFFERENT digipeater's callsign in the path must NOT trigger
     *     the guard -- only our own identity matters. */
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE1", 1, NULL, 0);
        int n = aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE1, "OTHER", 2);
        chk("A different callsign in path -> no false positive", n != 0);
    }

    printf("Duplicate suppression:\n");
    {
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE1", 1, NULL, 0);
        aprs_digi_process(f, len, sizeof f, APRS_DIGI_WIDE1, NULL, 0);   /* consume WIDE1-1 */

        bool first  = aprs_digi_seen_recently(f, len, 1000);
        bool second = aprs_digi_seen_recently(f, len, 1500);   /* same content, soon after */
        /* the window slides on every hit (see aprs_digi_seen_recently()'s
         * "keep suppressing while it keeps coming back"), so measure the
         * elapsed-window check from the last hit (1500), not the first. */
        bool later  = aprs_digi_seen_recently(f, len, 1500 + APRS_DIGI_DEDUP_MS + 1);

        chk("first sighting -> not a dup", !first);
        chk("immediate repeat -> suppressed", second);
        chk("after the dedup window -> allowed again", !later);
    }
    {
        /* a different info field must not collide with the frame above */
        uint8_t f[AX25_MAX_FRAME];
        int len = frame(f, "WIDE1", 1, NULL, 0);
        f[len - 1] ^= 0x01;   /* perturb the info field */
        bool dup = aprs_digi_seen_recently(f, len, 2000);
        chk("different info field -> not a dup", !dup);
    }

    printf(fails ? "\nFAIL\n" : "\nPASS\n");
    return fails ? 1 : 0;
}
