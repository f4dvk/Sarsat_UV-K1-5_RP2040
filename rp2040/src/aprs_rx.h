/*
 * aprs_rx.h — Bell-202 AFSK 1200 baud receiver + AX.25 HDLC de-framer.
 *
 * Streaming: feed one ADC sample at a time (12-bit, raw, DC not removed) at
 * APRS_RX_SAMPLE_RATE_HZ. On a complete, FCS-valid AX.25 frame the packet
 * callback fires with the frame minus the 2 FCS bytes.
 *
 * DSP chain (ported from JN1DFF pico_tnc, BSD-3):
 *   raw ADC -> DC removal -> carrier detect -> band-pass FIR -> hard limiter
 *   -> delay-and-multiply discriminator -> short boxcar low-pass    (shared)
 *   -> [ N parallel slicers at different bias levels
 *        -> DireWolf-style bit PLL -> NRZI -> HDLC de-frame ]      (per chain)
 *   -> FCS-16/X.25 check, with a single-bit-flip repair on failure.
 *
 * AX.25/APRS carries no FEC (the FCS only detects errors); the parallel
 * slicers + bit-flip give Dire-Wolf-style error *recovery* instead.
 *
 * No pico-sdk / no float — host-testable (rp2040/test/host/test_aprs_rx).
 */
#ifndef APRS_RX_H
#define APRS_RX_H

#include <stdint.h>
#include <stdbool.h>

#define APRS_RX_SAMPLE_RATE_HZ 13200      /* 1200 * 11                        */
#define APRS_RX_MAX_FRAME      330        /* AX.25 addr+ctl+pid+info+fcs      */
#define APRS_RX_SLICERS        3          /* parallel decode chains           */

typedef void (*aprs_rx_packet_cb)(const uint8_t *ax25, int len, void *user);

typedef struct {
    const int16_t *a;
    int            n;
    int32_t        x[32];
    int            idx;
} aprs_fir_t;

/* one decode chain: a biased slicer + bit PLL + HDLC de-framer */
typedef struct {
    int32_t  bias_num;          /* slice level = env * bias_num / 8         */
    int      bit;               /* last sliced tone (0/1)                  */
    uint32_t pll;
    int      pval;
    int      nrzi;
    uint8_t  flags;             /* HDLC bit shift register                 */
    int      state;             /* 0 = hunting flag, 1 = in frame          */
    uint8_t  cur_byte;
    int      bit_cnt;
    uint8_t  data[APRS_RX_MAX_FRAME];
    int      data_len;
} aprs_chan_t;

typedef struct {
    /* config (derived from the sample rate at init) */
    int      delay_n;
    uint32_t pll_step;

    /* shared front-end */
    int32_t  dc;                 /* raw << 8                                 */
    int32_t  cdt_lvl;
    bool     cdt;
    aprs_fir_t bpf, lpf;
    int32_t  delay_line[16];
    int      delay_idx;
    int32_t  lp_hi, lp_lo;       /* peak-tracking slicer rails               */
    int32_t  env;                /* slicer amplitude = (hi-lo)/2             */
    uint32_t n_clip, n_samp;     /* ADC-rail hits / total, for a clip %      */

    /* parallel decode chains */
    aprs_chan_t ch[APRS_RX_SLICERS];

    /* last frame candidate delivered/attempted by each chain (post any
     * single-bit repair), for aprs_try_fix_cross(): 3 independent slicers
     * of the same audio usually agree on most of a frame and diverge only
     * at the few byte positions where one of them mis-sliced a bit -- a
     * chain that still fails its own FCS check can often be rescued by
     * swapping in a sibling chain's bytes at just those positions. */
    struct {
        uint8_t  data[APRS_RX_MAX_FRAME];
        int      len;
        uint32_t at;
        bool     valid;
    } last_cand[APRS_RX_SLICERS];

    /* de-dup: suppress the same frame from more than one chain */
    uint8_t  last[APRS_RX_MAX_FRAME];
    int      last_len;
    uint32_t last_at;
    uint32_t now;                /* running sample counter                  */

    /* stats / output */
    aprs_rx_packet_cb cb;
    void            *user;
    uint32_t         n_packets;
    uint32_t         n_fcs_bad;
    uint32_t         n_fixed;    /* recovered by the single-bit-flip repair */
    uint32_t         n_hdlc;     /* HDLC frame candidates seen (all outcomes) */

    /* provisional RX diagnostics (see aprs_rx_set_frame_cb) */
    void (*fcb)(void *user, const uint8_t *frame, int len,
               int chain, int res, int looks_ui);
    void            *fcb_user;
} aprs_rx_t;

/* outcome codes passed to the diagnostic frame callback */
enum {
    APRS_RXR_OK,        /* FCS valid, delivered                       */
    APRS_RXR_FIXED,     /* one bit flipped to pass FCS + UI check     */
    APRS_RXR_FCS_BAD,   /* FCS still bad after the repair -> dropped  */
    APRS_RXR_SHORT,     /* < 18 bytes between flags -> dropped        */
    APRS_RXR_LONG,      /* frame overran the buffer                   */
    APRS_RXR_DEDUP,     /* identical to a frame just delivered        */
};

void     aprs_rx_init(aprs_rx_t *r, int sample_rate_hz,
                      aprs_rx_packet_cb cb, void *user);
void     aprs_rx_sample(aprs_rx_t *r, uint16_t adc12);
bool     aprs_rx_carrier(const aprs_rx_t *r);

/* ADC input clipping since the last call, in per-mille (0..1000), then resets
 * the counters. > ~20 means the radio AF gain is too high for this tap. */
uint32_t aprs_rx_clip_permille(aprs_rx_t *r);

/* diagnostics: `cb` fires for every HDLC frame candidate (data between two
 * flags), whatever the outcome (APRS_RXR_*). `frame` is the demodulated bytes
 * incl. the 2 FCS bytes (minus FCS for OK/DEDUP); `chain` is the slicer index.
 * NULL by default -> zero overhead. */
void aprs_rx_set_frame_cb(aprs_rx_t *r,
        void (*cb)(void *user, const uint8_t *frame, int len,
                   int chain, int res, int looks_ui),
        void *user);

/* live front-end levels for a periodic status line */
void aprs_rx_levels(const aprs_rx_t *r, int32_t *cdt_lvl, int32_t *env, int *carrier);

/* FCS-16/X.25 residue: run over frame incl. the 2 FCS bytes, == 0x0F47 if OK. */
uint16_t aprs_fcs_residue(const uint8_t *data, int len);

/* Single-bit-error repair: flip each bit in turn, keep the flip that makes the
 * FCS residue valid. Returns true (and leaves `d` fixed) on success. */
bool     aprs_try_fix(uint8_t *d, int len);

/* Cross-channel repair: try swapping in chain `sibling`'s cached bytes (see
 * aprs_rx_t::last_cand) at the positions where they differ from `d`, one
 * combination at a time, keeping the first that both passes the FCS and
 * looks like a plausible UI frame. Bails out (returns false) without
 * touching `d` if more than a handful of bytes differ (either not really
 * the same burst, or too many combinations to brute-force cheaply) or if
 * no sibling chain has a same-length, recent-enough candidate cached.
 * Exposed (not just internal to aprs_output()) so the host test can drive
 * it directly against known byte patterns. */
bool     aprs_try_fix_cross(aprs_rx_t *r, int chain, uint8_t *d, int len);

/* Format a raw AX.25 UI frame as TNC2 monitor text ("SRC>DST,PATH:info").
 * Writes up to `max_lines` NUL-terminated lines of `line_chars` glyphs into
 * `out` (out[max_lines][line_chars+1]); returns the line count. */
int      aprs_rx_format(const uint8_t *ax25, int len,
                        char *out, int max_lines, int line_chars);

#endif /* APRS_RX_H */
