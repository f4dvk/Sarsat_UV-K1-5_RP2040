/*
 * sonde_sync.h — streaming, bit-level frame-sync hunter for the radiosonde
 * decoders (RS41 confirmed usable end-to-end; M10/M20 detection-only -- see
 * below). Sits between sonde_demod (ADC samples -> bits) and the per-type
 * frame layer (sonde_rs41.*): feed it one demodulated bit at a time and it
 * finds the sync word AT ANY BIT PHASE, not just byte-aligned -- the same
 * technique that empirically found the M10/M20 header in a real capture
 * (see the project's radiosonde plan notes): the PLL that recovers bits
 * from the ADC stream locks onto an arbitrary phase, so a byte-aligned
 * search over already-packed bytes can walk right past a sync word that is
 * only ever a few bits away from matching. A 64/28-bit rolling shift
 * register checked on every incoming bit (same idea as the HDLC flag-hunt
 * in aprs_rx.c, generalised to longer, non-byte-multiple patterns) can't
 * miss an alignment like that.
 *
 * Scope:
 *   - RS41: full support. On a sync lock, the next SONDE_RS41_FRAME_CAP
 *     bits are packed MSB-first into bytes, de-whitened (sonde_rs41.h) and
 *     walked into CRC-valid blocks; a GPSPOS (0x7B) block is converted to
 *     lat/lon/alt. ~320 bytes matches the "~320 B/frame, 1/s" figure from
 *     the public write-up this project's RS41 layer was built from.
 *   - M10/M20: DETECTION ONLY. The 4800 baud rate and the `5A 4A 93 _A`
 *     header (3 bytes + a nibble, LSB-first per byte) are empirically
 *     confirmed bit-exact against a real capture (20/21 bursts, see the
 *     project's radiosonde plan notes) -- solid enough to say "a M10/M20 is
 *     here", not solid enough to say where it is: the frame layout past the
 *     header (GPS fields, checksum) is not publicly documented anywhere
 *     that isn't GPL-3.0 (rs1729/RS, radiosonde_auto_rx -- avoided, see
 *     sonde_rs41.h's licensing note), and this project's own further SDR
 *     forensics (an inter-frame XOR-cancellation technique that at least
 *     located a slowly-incrementing counter byte right after the header)
 *     did not get far enough to decode a position. A lock just reports
 *     SONDE_EVT_M10 immediately; nothing is decoded past the header.
 *   - DFM: NOT wired in here. Sigidwiki's DFM-17 sample shows a continuous
 *     transmission (no RF gaps, unlike RS41/M10's ~1 Hz bursts) with data
 *     interleaved over 7 bytes and no publicly-documented sync word -- this
 *     project's own capture analysis could not locate one either (the
 *     interleaving very plausibly hides any fixed byte pattern from a
 *     simple bit/byte correlator like this one). Left for a future capture
 *     or a public write-up that documents the interleave.
 *
 * Pure C, no pico-sdk -- host-testable (rp2040/test/host/test_sonde_sync).
 */
#ifndef SONDE_SYNC_H
#define SONDE_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#define SONDE_RS41_FRAME_CAP 320   /* ~320 B/frame is the public "typical RS41
                                    * frame size" figure (sync excluded) */

typedef enum {
    SONDE_EVT_NONE = 0,
    SONDE_EVT_RS41,     /* a full RS41 frame was captured; see *rs41_out    */
    SONDE_EVT_M10,      /* the M10/M20 header just locked (detection-only) */
} sonde_evt_t;

typedef struct {
    bool    has_position;
    int32_t lat_e5, lon_e5, alt_m;
    int     n_blocks;      /* CRC-valid blocks found (0 if de-whitening/CRC
                            * never validated a single block -- a bit-exact
                            * sync lock on noise is possible, this is what
                            * tells the caller the frame was actually good) */
} sonde_rs41_result_t;

typedef struct {
    /* precomputed once by sonde_sync_init() -- see sonde_sync.c */
    uint64_t rs41_pattern;
    uint64_t m10_pattern, m10_mask;

    /* rolling bit history (only the low N bits of each are meaningful) */
    uint64_t rs41_sr;
    uint64_t m10_sr;

    int     state;         /* SONDE_ST_HUNT or SONDE_ST_RS41_FRAME, .c-internal */
    uint8_t frame[SONDE_RS41_FRAME_CAP];
    int     frame_len;
    uint8_t cur_byte;
    int     cur_bits;
} sonde_sync_t;

void sonde_sync_init(sonde_sync_t *s);

/* Feed one demodulated bit (0/1, LSB used). Returns SONDE_EVT_NONE on most
 * calls; SONDE_EVT_M10 the instant the M10/M20 header locks (nothing else
 * to read); SONDE_EVT_RS41 once SONDE_RS41_FRAME_CAP post-sync bytes have
 * been captured, de-whitened and block-parsed -- `*rs41_out` (may be NULL)
 * is filled in on that call only (n_blocks == 0 means the sync bit-matched
 * but no block's CRC ever validated -- likely a false sync on noise). */
sonde_evt_t sonde_sync_feed(sonde_sync_t *s, uint8_t bit, sonde_rs41_result_t *rs41_out);

#endif /* SONDE_SYNC_H */
