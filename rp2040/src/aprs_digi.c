/* aprs_digi.c — see aprs_digi.h. */
#include "aprs_digi.h"
#include <string.h>

/* True if the 6-byte shifted-ASCII callsign field spells "WIDEn" for
 * n in 1..3 (the only aliases this simple digipeater ever acts on -- not
 * TRACEn/RELAY/ECHO/GATE/TEMPn). Returns the level via *level. */
static bool wide_alias(const uint8_t *addr, int *level)
{
    char c[7];
    for (int i = 0; i < 6; i++) c[i] = (char)(addr[i] >> 1);
    int n = 6;
    while (n > 0 && c[n - 1] == ' ') n--;   /* shifted-ASCII is space-padded */
    if (n != 5) return false;
    if (c[0] != 'W' || c[1] != 'I' || c[2] != 'D' || c[3] != 'E') return false;
    if (c[4] < '1' || c[4] > '3') return false;
    *level = c[4] - '0';
    return true;
}

/* true if `call` (up to 6 chars, NUL/space padded) has any real content --
 * a NULL or all-blank callsign never gets stamped into a transmitted frame
 * (matches this project's "never transmit an unconfigured identity" rule,
 * normally enforced by the NOCALL default check on the radio side). */
static bool has_callsign(const char *call)
{
    if (!call) return false;
    for (int i = 0; i < 6 && call[i]; i++)
        if (call[i] != ' ') return true;
    return false;
}

/* write a shifted-ASCII, space-padded callsign address byte group at d+pos:
 * 6 callsign bytes + 1 flags byte (reserved 0x60 | H-bit | ssid<<1 | ext). */
static void write_addr(uint8_t *d, int pos, const char *call, uint8_t ssid,
                       bool used, uint8_t extbit)
{
    for (int i = 0; i < 6; i++) {
        char c = call[i];
        if (c == 0) c = ' ';
        d[pos + i] = (uint8_t)((unsigned char)c << 1);
    }
    d[pos + 6] = (uint8_t)(0x60 | (used ? 0x80 : 0) | ((ssid & 0x0F) << 1) | extbit);
}

/* true if the 6-callsign-bytes + SSID-nibble at d+pos match my_call/my_ssid.
 * `my_call` is known non-blank by every caller here (checked up front). */
static bool addr_is_mycall(const uint8_t *addr, const char *my_call, uint8_t my_ssid)
{
    for (int i = 0; i < 6; i++) {
        char want = my_call[i];
        if (want == 0) want = ' ';
        if ((char)(addr[i] >> 1) != want) return false;
    }
    return ((addr[6] >> 1) & 0x0F) == (my_ssid & 0x0F);
}

int aprs_digi_process(uint8_t *d, int len, int max_len, int digi_level,
                      const char *my_call, uint8_t my_ssid)
{
    if (digi_level <= APRS_DIGI_OFF || len < 14)
        return 0;

    bool trace = has_callsign(my_call);

    if (trace) {
        /* Guard 1: never repeat a frame we ourselves originated (its SOURCE
         * address is our own callsign-SSID) -- heard coming back somehow
         * (another digi's relay, a receiver quirk), never re-transmit it. */
        if (addr_is_mycall(&d[7], my_call, my_ssid))
            return 0;

        /* Guard 2: never repeat a frame that already carries our own
         * callsign ANYWHERE in the digi/via list, used or not -- proof this
         * exact frame already passed through us once (we are the one who
         * would have inserted that trace hop). Unlike the ~30 s content
         * dedup below, this never expires. */
        {
            int  p = 14;
            bool e = (d[13] & 1);
            while (!e) {
                if (p + 7 > len) break;
                e = (d[p + 6] & 1);
                if (addr_is_mycall(&d[p], my_call, my_ssid))
                    return 0;
                p += 7;
            }
        }
    }

    int  pos = 14;
    bool ext = (d[13] & 1);            /* src SSID byte ends the addr list? */

    while (!ext) {
        if (pos + 7 > len)
            return 0;                  /* malformed / truncated -- bail */
        ext = (d[pos + 6] & 1);

        if (d[pos + 6] & 0x80) {       /* H-bit set: already used, skip over */
            pos += 7;
            continue;
        }

        /* First unused address: this is the only one we may act on -- AX.25
         * source routing is strictly positional, never skip ahead to a
         * later hop even if it would otherwise match. */
        int level;
        if (!wide_alias(&d[pos], &level) || level > digi_level)
            return 0;

        int ssid = (d[pos + 6] >> 1) & 0x0F;
        if (ssid == 0)
            return 0;                  /* WIDEn-0 but still unused: malformed */

        ssid--;
        uint8_t extbit   = (uint8_t)(d[pos + 6] & 0x01);   /* preserve */
        bool    now_used = (ssid == 0);                    /* fully satisfied? */

        if (!trace || len + 7 > max_len) {
            /* No callsign configured, or no room to insert a trace hop:
             * plain in-place update -- the alias keeps its own name
             * ("WIDEn"), only its SSID and H-bit change. */
            d[pos + 6] = (uint8_t)(0x60 | (now_used ? 0x80 : 0) | (ssid << 1) | extbit);
            return len;
        }

        /* Insert a trace hop ("MYCALL-N*", H-bit set -- we are the one
         * handling this hop right now) just ahead of the alias, which
         * keeps its own name but gets its SSID decremented and, once fully
         * satisfied, its own H-bit set too: "WIDE2-2" becomes
         * "MYCALL-N*,WIDE2-1" after one hop, "MYCALL-N*,OTHER-N*,WIDE2*"
         * after a second digipeater finishes it -- standard traceable
         * New-N-Paradigm digipeating (matches Dire Wolf's default and most
         * hardware digis): every hop stays visible, not just the last. */
        memmove(d + pos + 7, d + pos, (size_t)(len - pos));
        write_addr(d, pos, my_call, my_ssid, true, 0);              /* not last */
        d[pos + 7 + 6] = (uint8_t)(0x60 | (now_used ? 0x80 : 0) | (ssid << 1) | extbit);
        return len + 7;
    }
    return 0;                          /* no unused digi slot: direct, or
                                         * already fully repeated */
}

/* ------------------------------------------------------- dedup ------------ */
#define DEDUP_N 12

typedef struct {
    uint8_t  key[16];       /* dst[7] + src[7] + 2-byte info hash          */
    uint32_t at_ms;
    bool     valid;
} dedup_entry_t;

static dedup_entry_t s_dedup[DEDUP_N];
static int           s_dedup_next;

static uint16_t info_hash(const uint8_t *p, int n)
{
    uint16_t h = 0;
    for (int i = 0; i < n; i++)
        h = (uint16_t)(h * 33u + p[i]);
    return h;
}

/* offset of the info field: walk past dst+src+digis (same address-list walk
 * as above) then skip the 1-byte control + 1-byte PID. Returns -1 if the
 * frame is too short to even have those two bytes. */
static int info_offset(const uint8_t *d, int len)
{
    int  pos = 14;
    bool ext = (len >= 14) ? (d[13] & 1) : true;
    while (!ext) {
        if (pos + 7 > len) return -1;
        ext = (d[pos + 6] & 1);
        pos += 7;
    }
    return (pos + 2 <= len) ? pos + 2 : -1;
}

static void dedup_key(const uint8_t *d, int len, uint8_t key[16])
{
    memset(key, 0, 16);
    if (len < 14) return;
    memcpy(key, d, 7);          /* dst */
    memcpy(key + 7, d + 7, 7);  /* src */
    int io = info_offset(d, len);
    if (io >= 0) {
        uint16_t h = info_hash(d + io, len - io);
        key[14] = (uint8_t)(h & 0xFFu);
        key[15] = (uint8_t)(h >> 8);
    }
}

bool aprs_digi_seen_recently(const uint8_t *d, int len, uint32_t now_ms)
{
    uint8_t key[16];
    dedup_key(d, len, key);

    for (int i = 0; i < DEDUP_N; i++) {
        if (!s_dedup[i].valid) continue;
        if ((uint32_t)(now_ms - s_dedup[i].at_ms) >= APRS_DIGI_DEDUP_MS) {
            s_dedup[i].valid = false;
            continue;
        }
        if (memcmp(s_dedup[i].key, key, 16) == 0) {
            s_dedup[i].at_ms = now_ms;  /* refresh: keep suppressing while
                                         * it keeps coming back */
            return true;
        }
    }

    dedup_entry_t *e = &s_dedup[s_dedup_next];
    s_dedup_next = (s_dedup_next + 1) % DEDUP_N;
    memcpy(e->key, key, 16);
    e->at_ms = now_ms;
    e->valid = true;
    return false;
}
