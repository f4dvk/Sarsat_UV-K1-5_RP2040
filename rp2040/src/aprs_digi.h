/* aprs_digi.h — WIDEn-N "New N-Paradigm" APRS digipeating decision.
 *
 * Pure logic, no radio/UART/pico-sdk dependency -> host-testable
 * (rp2040/test/host/test_aprs_digi). The caller (main.c) is the one that
 * knows about the UART link to the radio: it hands this module a decoded,
 * FCS-stripped AX.25 frame (same layout aprs_rx's packet callback delivers:
 * dst[7] src[7] digi[7]*n ctrl pid info), gets back a yes/no plus the
 * (possibly mutated in place) frame to retransmit, and is responsible for
 * actually sending it to the radio (which appends FCS and keys up).
 */
#ifndef APRS_DIGI_H
#define APRS_DIGI_H

#include <stdint.h>
#include <stdbool.h>

/* Off / repeat WIDE1-N / also WIDE2-N / also WIDE3-N -- cumulative, matches
 * the radio's "Digi" menu field (aprs_cfg_t::opts bits 5:4). */
enum {
    APRS_DIGI_OFF   = 0,
    APRS_DIGI_WIDE1 = 1,
    APRS_DIGI_WIDE2 = 2,
    APRS_DIGI_WIDE3 = 3,
};

/* Two permanent anti-loop guards, checked before anything else whenever a
 * callsign is configured (both are no-ops, always false, if `my_call` is
 * NULL/blank -- there is no identity to compare against):
 *   - the frame's SOURCE address is our own callsign-SSID: we would be
 *     digipeating a frame we ourselves originated, heard coming back
 *     (relayed by another digi, or some other loop) -- never repeat it.
 *   - our own callsign-SSID already appears ANYWHERE in the digi/via
 *     address list, used or not: this frame has already passed through us
 *     once (we inserted that trace hop ourselves, see below) -- never
 *     touch it again, regardless of what generic aliases remain unused
 *     further down the path. Unlike aprs_digi_seen_recently() (a ~30 s
 *     content-based window), this check never expires: our own callsign in
 *     the path is permanent, unambiguous proof of "already handled by me".
 *
 * Scans the digi/via address list of `d` (len bytes, FCS already stripped,
 * buffer capacity `max_len`) for the first UNUSED (H-bit clear) entry --
 * already-used entries (e.g. a path arriving as "WIDE1*") are skipped over,
 * never re-touched, matching plain AX.25 source-routing: a digipeater only
 * ever acts on the *next* unfulfilled hop, never one further down the path,
 * and never one already marked repeated.
 *
 * If that first-unused entry is a generic "WIDEn" alias (n = 1..3, encoded
 * as literal callsign text, not the SSID) with n <= digi_level, decrements
 * its SSID by one; the alias's own H-bit is set once that reaches 0 (fully
 * satisfied) and left clear otherwise (unused, so a further digipeater down
 * the path can still fulfil the remaining hop count -- this is what gives
 * e.g. WIDE2-2 its 2-hop fan-out) -- the alias's CALLSIGN TEXT never
 * changes either way, it always stays "WIDEn".
 *
 * If a callsign is configured and there is room, a NEW address is also
 * INSERTED just ahead of the alias, carrying `my_call`-`my_ssid` (H-bit
 * set, since we are the one handling this hop right now) -- standard
 * traceable New-N-Paradigm digipeating (matches Dire Wolf's default and
 * most hardware digis): receiving "WIDE2-2" produces "MYCALL-N*,WIDE2-1"
 * after one hop, "MYCALL-N*,OTHER-N*,WIDE2*" after a second digipeater
 * finishes it -- a monitor sees both which stations relayed the packet and
 * how the generic alias itself was consumed, at every hop, not just the
 * last one. Falls back to a plain in-place decrement (no insertion, no
 * trace, alias text/position unchanged) if no callsign is configured, or
 * if the extra 7 bytes would not fit in the `max_len` buffer.
 *
 * Returns the new frame length (>= len, since it may grow by 7 bytes) if
 * this frame should be retransmitted -- `d` holds `max_len`-bounded space
 * for the mutation. Returns 0, `d` untouched, if: digi_level is
 * APRS_DIGI_OFF, either anti-loop guard above fired, the frame is too short
 * to have a digi/via list, the first unused address is not a WIDEn alias in
 * range (an explicit callsign, an out-of-range WIDEn, or a non-WIDE generic
 * alias), or there is no unused address left in the path at all (fully
 * repeated / direct).
 *
 * `my_call`: up to 6 chars, NUL and/or space padded (same layout as
 * aprs_cfg_t::call on the radio side) -- pass NULL to always fall back to
 * the plain, non-traceable, no-growth behaviour and skip both anti-loop
 * guards (no identity to compare against). `my_ssid`: 0..15, ignored if
 * `my_call` is NULL/blank. */
int aprs_digi_process(uint8_t *d, int len, int max_len, int digi_level,
                      const char *my_call, uint8_t my_ssid);

/* Duplicate suppression: a digipeater that hears its own already-repeated
 * frame come back (relayed by another digi further down a multi-hop path)
 * must not repeat it again, even though aprs_digi_process() alone cannot
 * always catch this -- a single digipeater configured for more than one
 * WIDE level (e.g. "WIDE2", which also handles WIDE1) can legitimately act
 * on a frame twice in a row (once per alias) if it hears its own first
 * repeat echoed back before the second alias was consumed. Keeps a small
 * rolling history of (dst+src+info hash) keyed to `now_ms` (any monotonic
 * millisecond-ish counter the caller already has, e.g.
 * to_ms_since_boot(get_absolute_time())); a match within
 * APRS_DIGI_DEDUP_MS suppresses a repeat. Every call records the frame's
 * key (whether or not it was a duplicate), so the window slides forward
 * continuously. Call this once per candidate frame, right before actually
 * transmitting it (after aprs_digi_process() returned true) -- do not call
 * it for frames that were not going to be repeated anyway. */
#define APRS_DIGI_DEDUP_MS 30000u  /* 30 s, matches common digipeater practice */
bool aprs_digi_seen_recently(const uint8_t *d, int len, uint32_t now_ms);

#endif /* APRS_DIGI_H */
