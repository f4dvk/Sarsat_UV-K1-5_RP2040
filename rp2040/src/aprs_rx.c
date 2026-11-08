/* aprs_rx.c — Bell-202 AFSK 1200 receiver + AX.25 de-framer. See aprs_rx.h.
 *
 * DSP ported from JN1DFF's pico_tnc (bell202.c / decode.c / filter.c, BSD-3).
 * Kept integer-only and amplitude-agnostic (envelope-tracked slicer) so the
 * same code runs on the RP2040 and on the host test.
 */
#include "aprs_rx.h"
#include <string.h>

/* ---- FIR coefficients (Hamming-windowed sinc, fs = 13200 Hz) ----
 * BPF 900..2700 Hz, 25 taps, same peak gain as the earlier 900..2500 one.
 * The wider top (space tone + its first modulation sideband at 2200+600 Hz)
 * was the single largest gain of the 2026-11 parameter sweep (+2.4 %
 * packets at 0..10 dB SNR); 2600 / 2800 / 3000 Hz all measured worse.
 * Symmetric, so tap order does not matter. */
static const int16_t APRS_BPF_AN[25] = {
        82,    176,    172,    -47,   -188,    168,    411,   -940,
     -3733,  -4756,  -1030,   5469,   8751,   5469,  -1030,  -4756,
     -3733,   -940,    411,    168,   -188,    -47,    172,    176,
        82,
};

/* Post-discriminator low-pass: 21-tap Hamming, cut ~1500 Hz (fs 13200).
 * Removes the delay-and-multiply 2f ripple. History: a 27-tap version
 * spanned ~2.5 bits and smeared edges into neighbours (1-3 bit errors per
 * frame on strong local signals), 13 taps fixed that; with 9 slicers and
 * the lighter hunting PLL, 21 taps (~1.9 bits) measured +1.3 % packets over
 * 13 and 17, and 2000 Hz / 1300 Hz cut-offs measured worse. Symmetric, so
 * tap order is free. DC gain 8 (taps sum ~32768, fir_run >> 12) keeps the
 * scale so env / th / APRS_SLICE_MIN need no retuning. */
static const int16_t APRS_LPF_AN[21] = {
        64,     16,   -119,   -382,   -625,   -466,    498,   2361,
      4686,   6645,   7414,   6645,   4686,   2361,    498,   -466,
      -625,   -382,   -119,     16,     64,
};

/* delay-and-multiply lag: ~446 us (pico_tnc DELAY_US), rounded to samples */
#define APRS_DELAY_US      446

/* carrier detect on the DC-removed sample energy (adaptive-ish, fixed gates) */
#define APRS_CDT_ON        1200
#define APRS_CDT_OFF        400

/* open-squelch channel sense (see aprs_rx.h): hiss energy is measured on the
 * 2nd difference of the AC signal, |4 sin^2(pi f/fs)| -- x0.3 at 1200 Hz,
 * x0.95 at 2200 Hz, x3.5 above 5 kHz at fs = 13.2 kHz -- so the wideband FM
 * hiss dominates and a quieted channel (even one carrying AFSK) drops well
 * below half of the idle level. busy below floor/2, free above 3/4 floor.
 * The floor only tracks while free; after APRS_HISS_STUCK_N samples busy
 * (60 s) it re-learns slowly, so a lowered AF gain cannot pin "busy". */
#define APRS_HISS_MIN       2000      /* below this there is no hiss to judge */
#define APRS_HISS_STUCK_N   (13200u * 60u)

/* slicer: dead-band = env >> APRS_SLICE_SHIFT (env/8: measured +1.5 %
 * packets over env/4, env/2 much worse, env/16 slightly worse), floored so
 * silence does not chatter */
#define APRS_SLICE_SHIFT   3
#define APRS_SLICE_MIN     24

/* Soft clamp of the band-pass output at APRS_LIM_K/16 x its mean magnitude
 * (~78 ms average). A hard sign-limiter here only lets the discriminator move
 * at input zero crossings -- every half bit at 1200 Hz -- which spread bit
 * transitions by +-0.25 bit even on a clean signal: a TX just 0.5 % fast then
 * lost ~1 packet in 6. The relative clamp keeps the limiter's benefits
 * (clipped / diode-clamped / twisted audio, noise peaks) without that.
 * 24/16 measured best (+1 % over 19/16; 14-16 and 32 worse). */
#define APRS_LIM_K         24
#define APRS_LIM_MIN       64

/* bit PLL correction on a tone transition: phase *= 1 - 2^-shift
 * (shift 2 = inertia 0.75, shift 1 = 0.5). Light while hunting so the PLL
 * locks on the first preamble flags (+3 % packets with a 4-flag TXDELAY,
 * +4 % on a +1 % fast TX), heavier in a frame so a noisy edge does not
 * drag it off. Same split as DireWolf (0.5 searching / 0.74 locked). */
#ifndef APRS_PLL_SHIFT_LOCK
#define APRS_PLL_SHIFT_LOCK 2
#endif
#ifndef APRS_PLL_SHIFT_HUNT
#define APRS_PLL_SHIFT_HUNT 1
#endif

/* ------------------------------------------------------------------ FIR ---- */
static void fir_init(aprs_fir_t *f, const int16_t *a, int n)
{
    f->a = a;
    f->n = n;
    f->idx = 0;
    memset(f->x, 0, sizeof f->x);
}

static int32_t fir_run(aprs_fir_t *f, int32_t v)
{
    f->x[f->idx] = v;
    if (++f->idx >= f->n)
        f->idx = 0;

    int64_t acc = 0;
    int k = f->idx;                        /* oldest sample */
    for (int i = 0; i < f->n; i++) {       /* symmetric taps: order is free */
        acc += (int64_t)f->a[i] * f->x[k]; /* x can be ~1e7 -> must be 64-bit */
        if (++k >= f->n)
            k = 0;
    }
    return (int32_t)(acc >> 12);
}

/* ---------------------------------------------------------------- FCS ------ */
uint16_t aprs_fcs_residue(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

/* --------------------------------------------------------------- HDLC ------ */
#define HDLC_HUNT 0
#define HDLC_DATA 1

/* AX.25 has no FEC. Find the single bit flip (if any) that makes the FCS
 * pass: the CRC is linear, so flipping bit b of byte i changes the residue
 * by a pattern that depends only on b and on the distance to the end of the
 * frame. Walk from the last byte backwards, advancing the 8 unit patterns by
 * one byte (8 shift steps) per position and comparing with the residue
 * error: O(64 n) shift steps instead of O(64 n^2) for flip-and-recompute --
 * ~15 ms per 60-byte frame on the RP2040 became ~0.1 ms. CRC-16 has
 * Hamming distance 4 at these lengths, so at most one position matches.
 * Returns bit index (byte * 8 + bit), -1 if none (or the FCS is already OK). */
int aprs_fcs_single_flip(const uint8_t *d, int n)
{
    uint16_t want = aprs_fcs_residue(d, n) ^ 0x0F47;
    if (!want)
        return -1;
    uint16_t u[8];
    for (int b = 0; b < 8; b++)
        u[b] = (uint16_t)(1u << b);
    for (int i = n - 1; i >= 0; i--) {
        for (int b = 0; b < 8; b++) {
            uint16_t c = u[b];
            for (int k = 0; k < 8; k++)
                c = (c & 1) ? (uint16_t)((c >> 1) ^ 0x8408) : (uint16_t)(c >> 1);
            u[b] = c;
            if (c == want)
                return i * 8 + b;
        }
    }
    return -1;
}

bool aprs_try_fix(uint8_t *d, int n)
{
    int k = aprs_fcs_single_flip(d, n);
    if (k < 0)
        return false;
    d[k >> 3] ^= (uint8_t)(1u << (k & 7));
    return true;
}

/* sanity-check an AX.25 UI frame: address field is 7-byte blocks of A-Z /
 * 0-9 / space callsigns ending with the extension bit, then control 0x03 and
 * PID 0xF0. Guards the bit-flip repair against a false FCS match producing a
 * garbage "packet" (the callsign check alone cut those by ~3/4). */
static bool ax25_looks_like_ui(const uint8_t *d, int len)
{
    int pos = 0;                         /* start of the current address */
    for (int blk = 0; blk < 10; blk++) {
        if (pos + 7 > len)
            return false;
        for (int i = 0; i < 6; i++) {    /* callsigns are A-Z, 0-9, space */
            uint8_t ch = d[pos + i] >> 1;
            if (!((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == ' ') ||
                (d[pos + i] & 1))
                return false;
        }
        pos += 6;                        /* SSID byte */
        if (d[pos] & 1) {                /* extension bit -> end of addresses */
            return blk >= 1 && pos + 3 <= len &&
                   d[pos + 1] == 0x03 && d[pos + 2] == 0xF0;
        }
        pos++;
    }
    return false;
}

bool aprs_try_fix_cross(aprs_rx_t *r, int chain, uint8_t *d, int len)
{
    enum { MAX_DIFF = 5 };   /* 2^5 = 32 combinations to try, still cheap */

    for (int j = 0; j < APRS_RX_SLICERS; j++) {
        if (j == chain || !r->last_cand[j].valid)
            continue;
        if (r->last_cand[j].len != len)
            continue;
        /* all chains advance on the same sample clock (independent PLL
         * per chain, but fed the same audio) -- a stale candidate from a
         * much earlier, unrelated burst must not be mistaken for a sibling
         * decode of *this* frame. ~50 ms is generous slack for PLL jitter
         * between chains while still well inside one HDLC frame. */
        if ((r->now - r->last_cand[j].at) > (uint32_t)(APRS_RX_SAMPLE_RATE_HZ / 20))
            continue;

        const uint8_t *other = r->last_cand[j].data;
        int diff_pos[MAX_DIFF];
        int nd = 0;
        bool too_many = false;
        for (int i = 0; i < len; i++) {
            if (d[i] == other[i])
                continue;
            if (nd == MAX_DIFF) { too_many = true; break; }
            diff_pos[nd++] = i;
        }
        if (too_many || nd == 0)
            continue;                  /* not a close enough match to bother */

        uint8_t merged[APRS_RX_MAX_FRAME];
        memcpy(merged, d, (size_t)len);
        for (uint32_t mask = 1; mask < (1u << nd); mask++) {
            for (int k = 0; k < nd; k++)
                merged[diff_pos[k]] = (mask & (1u << k)) ? other[diff_pos[k]]
                                                          : d[diff_pos[k]];
            if (aprs_fcs_residue(merged, len) == 0x0F47 &&
                ax25_looks_like_ui(merged, len - 2)) {
                memcpy(d, merged, (size_t)len);
                return true;
            }
        }
    }
    return false;
}

#define APRS_DIAG(FR, L, RES, UI) \
    do { if (r->fcb) r->fcb(r->fcb_user, (FR), (L), chain, (RES), (UI)); } while (0)

/* sibling chains close the same frame within a few bits of each other */
#define APRS_HOLD_SAMPLES  (APRS_RX_SAMPLE_RATE_HZ / 100)

static void aprs_deliver(aprs_rx_t *r, const uint8_t *d, int plen, bool fixed,
                         int chain, int ui)
{
    /* de-dup: the same frame usually decodes on several chains at once */
    if (r->last_len == plen &&
        (r->now - r->last_at) < (uint32_t)(APRS_RX_SAMPLE_RATE_HZ * 3 / 2) &&
        memcmp(r->last, d, (size_t)plen) == 0) {
        APRS_DIAG(d, plen, APRS_RXR_DEDUP, ui);
        return;
    }
    memcpy(r->last, d, (size_t)plen);
    r->last_len = plen;
    r->last_at  = r->now;

    r->n_packets++;
    APRS_DIAG(d, plen, fixed ? APRS_RXR_FIXED : APRS_RXR_OK, ui);
    if (r->cb)
        r->cb(d, plen, fixed, r->user);
}

static void aprs_output(aprs_rx_t *r, aprs_chan_t *c)
{
    const int chain = (int)(c - r->ch);
    const int len   = c->data_len;

    /* 7 (dst) + 7 (src) + 1 (ctl) + 1 (pid) + 2 (fcs) */
    if (len < 18) {
        if (len > 4) { r->n_hdlc++; APRS_DIAG(c->data, len, APRS_RXR_SHORT, 0); }
        return;
    }
    if (len > APRS_RX_MAX_FRAME) {
        r->n_hdlc++; APRS_DIAG(c->data, APRS_RX_MAX_FRAME, APRS_RXR_LONG, 0);
        return;
    }
    r->n_hdlc++;

    bool ok  = (aprs_fcs_residue(c->data, len) == 0x0F47);
    int  res = ok ? APRS_RXR_OK : APRS_RXR_FIXED;
    if (!ok) {
        /* repair a single bit error, accepting only a flip that also yields a
         * plausible UI frame (a bare FCS match has a ~1 % false-positive rate) */
        int k = aprs_fcs_single_flip(c->data, len);
        if (k >= 0) {
            c->data[k >> 3] ^= (uint8_t)(1u << (k & 7));
            if (ax25_looks_like_ui(c->data, len - 2))
                ok = true;
            else
                c->data[k >> 3] ^= (uint8_t)(1u << (k & 7));
        }
        if (!ok)
            ok = aprs_try_fix_cross(r, chain, c->data, len);
        if (ok)
            r->n_fixed++;
    }

    /* cache this chain's best-effort bytes (post any repair) so a sibling
     * chain that finishes moments later -- or earlier; completion order
     * between the independently-PLL'd chains isn't guaranteed -- can try
     * cross-channel repair (aprs_try_fix_cross()) against it. */
    r->last_cand[chain].len   = len;
    r->last_cand[chain].at    = r->now;
    r->last_cand[chain].valid = true;
    memcpy(r->last_cand[chain].data, c->data, (size_t)len);

    const int ui = ax25_looks_like_ui(c->data, len - 2);

    if (!ok) {
        r->n_fcs_bad++;
        APRS_DIAG(c->data, len, APRS_RXR_FCS_BAD, ui);
        return;
    }

    const int plen = len - 2;                          /* strip the FCS */

    if (res == APRS_RXR_FIXED) {
        if (r->pend_valid && (r->pend_len != plen ||
                              memcmp(r->pend, c->data, (size_t)plen) != 0)) {
            r->pend_valid = false;     /* two repairs disagree: one is wrong */
            return;
        }
        memcpy(r->pend, c->data, (size_t)plen);
        r->pend_len   = plen;
        r->pend_chain = chain;
        r->pend_at    = r->now;
        r->pend_valid = true;
        return;
    }
    r->pend_valid = false;             /* an intact copy beats any repair */
    aprs_deliver(r, c->data, plen, false, chain, ui);
}

#ifdef APRS_RX_DEBUG
#include <stdio.h>
static int g_dbg_bits;
#endif

static void hdlc_bit(aprs_rx_t *r, aprs_chan_t *c, int bit)
{
#ifdef APRS_RX_DEBUG
    if (c == &r->ch[1] && g_dbg_bits < 400) { fprintf(stderr, "%d", bit & 1); g_dbg_bits++; }
#endif
    c->flags = (uint8_t)((c->flags << 1) | (bit & 1));

    if (c->state == HDLC_HUNT) {
        if (c->flags == 0x7E) {
            c->state    = HDLC_DATA;
            c->data_len = 0;
            c->bit_cnt  = 0;
            c->cur_byte = 0;
        }
        return;
    }

    /* HDLC_DATA */
    if ((c->flags & 0x3F) == 0x3F) {        /* >=6 ones -> flag / abort / end */
        aprs_output(r, c);
        c->state = HDLC_HUNT;
        return;
    }
    if ((c->flags & 0x3F) == 0x3E)          /* stuffed 0 after five 1s -> drop */
        return;

    c->cur_byte = (uint8_t)((c->cur_byte >> 1) | ((bit & 1) << 7));
    if (++c->bit_cnt >= 8) {
        if (c->data_len < APRS_RX_MAX_FRAME)
            c->data[c->data_len++] = c->cur_byte;
        else
            c->state = HDLC_HUNT;           /* overrun */
        c->bit_cnt = 0;
    }
}

/* --------------------------------------------------------------- demod ----- */
void aprs_rx_init(aprs_rx_t *r, int sample_rate_hz,
                  aprs_rx_packet_cb cb, void *user)
{
    memset(r, 0, sizeof *r);
    r->cb   = cb;
    r->user = user;

    int d = (sample_rate_hz * APRS_DELAY_US + 500000) / 1000000;
    if (d < 1)  d = 1;
    if (d > 15) d = 15;
    r->delay_n = d;

    /* phase accumulator wraps once per bit: step = 2^32 * 1200 / fs */
    r->pll_step = (uint32_t)(((uint64_t)1 << 32) * 1200u / (uint32_t)sample_rate_hz);

    r->dc = (int32_t)2048 << 8;
    fir_init(&r->bpf, APRS_BPF_AN, 25);
    fir_init(&r->lpf, APRS_LPF_AN, 21);

    /* slice at bias levels spread symmetrically around 0 (default 3 chains at
     * -3/8, 0, +3/8 of the envelope) -- an AC-coupled / diode-clamped feed
     * (the C-Board) makes the discriminator output asymmetric, so the best
     * decision point is often not zero. */
    for (int i = 0; i < APRS_RX_SLICERS; i++) {
        r->ch[i].bias_num = (i - (APRS_RX_SLICERS - 1) / 2) * APRS_RX_BIAS_STEP;
        r->ch[i].state    = HDLC_HUNT;
    }
}

bool aprs_rx_carrier(const aprs_rx_t *r) { return r->sql_open ? r->busy : r->cdt; }

void aprs_rx_set_sql_open(aprs_rx_t *r, bool open)
{
    if (open == r->sql_open)
        return;
    r->sql_open = open;
    r->hf_floor = 0;                 /* re-learn the idle hiss from scratch */
    r->hf_lvl   = 0;
    r->busy     = false;
    r->busy_n   = 0;
    if (!open)
        r->cdt = false;              /* back to the energy gate, from idle */
}

bool aprs_rx_sql_open(const aprs_rx_t *r) { return r->sql_open; }

void aprs_rx_noise_levels(const aprs_rx_t *r, int32_t *hf_lvl, int32_t *hf_floor)
{
    if (hf_lvl)   *hf_lvl   = r->hf_lvl;
    if (hf_floor) *hf_floor = r->hf_floor;
}

/* one AC sample of the open-squelch channel sense (see the defines above) */
static void aprs_rx_sense_hiss(aprs_rx_t *r, int32_t s)
{
    int32_t d2 = s - 2 * r->hp1 + r->hp2;           /* 2nd difference, +-8192 */
    r->hp2 = r->hp1;
    r->hp1 = s;
    r->hf_lvl += ((d2 * d2) - r->hf_lvl) >> 7;      /* ~10 ms */

    if (r->hf_floor < APRS_HISS_MIN) {              /* no reference yet */
        r->hf_floor += (r->hf_lvl - r->hf_floor) >> 6;
        r->busy = false;
        r->busy_n = 0;
        return;
    }
    if (!r->busy) {
        r->hf_floor += (r->hf_lvl - r->hf_floor) >> 12;   /* ~0.3 s */
        if (r->hf_lvl < r->hf_floor / 2) {
            r->busy = true;
            r->busy_n = 0;
        }
    } else {
        if (++r->busy_n > APRS_HISS_STUCK_N)
            r->hf_floor += (r->hf_lvl - r->hf_floor) >> 14;
        if (r->hf_lvl > (r->hf_floor >> 2) * 3)
            r->busy = false;
    }
}

uint32_t aprs_rx_clip_permille(aprs_rx_t *r)
{
    uint32_t pm = r->n_samp ? (uint32_t)((uint64_t)r->n_clip * 1000 / r->n_samp) : 0;
    r->n_clip = r->n_samp = 0;
    return pm;
}

void aprs_rx_set_frame_cb(aprs_rx_t *r,
        void (*cb)(void *user, const uint8_t *frame, int len,
                   int chain, int res, int looks_ui),
        void *user)
{
    r->fcb = cb;
    r->fcb_user = user;
}

void aprs_rx_levels(const aprs_rx_t *r, int32_t *cdt_lvl, int32_t *env, int *carrier)
{
    if (cdt_lvl) *cdt_lvl = r->cdt_lvl;
    if (env)     *env     = r->env;
    if (carrier) *carrier = r->cdt ? 1 : 0;
}

void aprs_rx_sample(aprs_rx_t *r, uint16_t adc12)
{
    int32_t raw = adc12 & 0x0FFF;
    r->now++;

    if (r->pend_valid && r->now - r->pend_at >= APRS_HOLD_SAMPLES) {
        r->pend_valid = false;
        aprs_deliver(r, r->pend, r->pend_len, true, r->pend_chain, 1);
    }

    r->n_samp++;
    if (raw <= 1 || raw >= 4094)                    /* ADC rail = input clipping */
        r->n_clip++;

    /* DC removal (~5 ms time constant) */
    r->dc += ((raw << 8) - r->dc) >> 9;
    int32_t s = raw - (r->dc >> 8);                 /* AC, roughly +-2048 */

    /* carrier detect on short-term energy */
    r->cdt_lvl += ((s * s) - r->cdt_lvl) >> 7;
    if (r->sql_open) {
        aprs_rx_sense_hiss(r, s);                   /* busy/free from quieting */
        r->cdt = true;                              /* decoder always runs    */
    } else if (!r->cdt && r->cdt_lvl > APRS_CDT_ON) {
        r->cdt = true;
    } else if (r->cdt && r->cdt_lvl < APRS_CDT_OFF) {
        r->cdt = false;
        r->lp_hi = r->lp_lo = 0;                     /* squelch shut -> reset */
        for (int i = 0; i < APRS_RX_SLICERS; i++)
            r->ch[i].state = HDLC_HUNT;
        return;
    }
    if (!r->cdt)
        return;

    /* band-pass, then soft clamp, then delay-and-multiply */
    int32_t v = fir_run(&r->bpf, s);                 /* ~ +-16000 */
    r->lim_env += ((v < 0 ? -v : v) - r->lim_env) >> 10;
    int32_t lim = (r->lim_env * APRS_LIM_K) >> 4;
    if (lim < APRS_LIM_MIN)
        lim = APRS_LIM_MIN;
    if (v > lim)       v = lim;
    else if (v < -lim) v = -lim;

    int32_t d = r->delay_line[r->delay_idx];
    r->delay_line[r->delay_idx] = v;
    if (++r->delay_idx >= r->delay_n)
        r->delay_idx = 0;

    int32_t m  = (v >> 4) * (d >> 4);                /* ~ +-1e6, fits int32 */
    int32_t lp = fir_run(&r->lpf, m);               /* discriminator output */

    /* peak-tracking centre: follow the high / low rails of the discriminator
     * output (fast-ish attack rejects noise spikes, slow decay) -> centre =
     * (hi+lo)/2 removes the carrier tuning offset and AFSK baseline wander
     * automatically (that drift was making a different bias chain "win" on
     * every frame). Threshold still comes from a slow EMA envelope, which is
     * more noise-robust than the rail span at low SNR. */
    if (lp > r->lp_hi) r->lp_hi += (lp - r->lp_hi) >> 2;
    else               r->lp_hi -= (r->lp_hi - r->lp_lo) >> 10;   /* ~78 ms */
    if (lp < r->lp_lo) r->lp_lo += (lp - r->lp_lo) >> 2;
    else               r->lp_lo += (r->lp_hi - r->lp_lo) >> 10;

    int32_t lpc = lp - ((r->lp_hi + r->lp_lo) >> 1);

    int32_t mag = lpc < 0 ? -lpc : lpc;
    r->env += (mag - r->env) >> 6;
    int32_t th = r->env >> APRS_SLICE_SHIFT;
    if (th < APRS_SLICE_MIN)
        th = APRS_SLICE_MIN;

    /* per-chain: biased slice + DireWolf-style bit PLL (pico_tnc decode2) */
    for (int i = 0; i < APRS_RX_SLICERS; i++) {
        aprs_chan_t *c = &r->ch[i];
        int32_t x = lpc - ((r->env * c->bias_num) >> 3);

        if (x > th)
            c->bit = 0;
        else if (x < -th)
            c->bit = 1;
        /* else: hold (dead-band) */

        int32_t prev_sign = (int32_t)c->pll >> 31;
        c->pll += r->pll_step;
        if (((int32_t)c->pll >> 31) < prev_sign) {
            hdlc_bit(r, c, c->bit == c->nrzi);       /* NRZI: no change -> 1 */
            c->nrzi = c->bit;
        }
        if (c->bit != c->pval) {
            /* nudge the phase toward the transition: a light PLL while
             * hunting (locks on the first preamble flags), a heavier one once
             * in a frame (rides over noisy edges) -- DireWolf 0.5 / 0.75 */
            int sh = (c->state == HDLC_DATA) ? APRS_PLL_SHIFT_LOCK
                                             : APRS_PLL_SHIFT_HUNT;
            c->pll -= (uint32_t)((int32_t)c->pll >> sh);
            c->pval = c->bit;
        }
    }
}

/* -------------------------------------------------------------- format ----- */
static int addr_str(const uint8_t *a, char *out)
{
    int n = 0;
    for (int i = 0; i < 6; i++) {
        char c = (char)(a[i] >> 1);
        if (c != ' ')
            out[n++] = c;
    }
    int ssid = (a[6] >> 1) & 0x0F;
    if (ssid) {
        out[n++] = '-';
        if (ssid >= 10) { out[n++] = '1'; ssid -= 10; }
        out[n++] = (char)('0' + ssid);
    }
    out[n] = 0;
    return n;
}

/* The radio screen is ~18 glyphs wide, so keep it compact: line 0 = the
 * source callsign, the rest = the info field wrapped. dst (APZxxx) and the
 * digi path are dropped -- they don't fit and matter least on that display. */
int aprs_rx_format(const uint8_t *ax25, int len,
                   char *out, int max_lines, int line_chars)
{
    for (int i = 0; i < max_lines; i++)
        out[i * (line_chars + 1)] = 0;

    if (len < 16)
        return 0;

    char src[12];
    addr_str(ax25 + 7, src);

    /* walk past dst + src + digis to the control/PID, then the info field */
    int  pos = 14;
    bool ext = (ax25[13] & 1);
    while (!ext && pos + 7 <= len - 2) {
        ext = (ax25[pos + 6] & 1);
        pos += 7;
    }
    const uint8_t *info = ax25 + pos + 2;
    int infolen = len - (pos + 2);
    if (infolen < 0) infolen = 0;

    int line = 0;
    {
        char *o = out + line * (line_chars + 1);
        int k = 0;
        for (int i = 0; src[i] && k < line_chars; i++) o[k++] = src[i];
        o[k] = 0;
        line++;
    }
    for (int i = 0; i < infolen && line < max_lines; ) {
        char *o = out + line * (line_chars + 1);
        int k = 0;
        for (; i < infolen && k < line_chars; i++) {
            uint8_t c = info[i];
            o[k++] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
        }
        o[k] = 0;
        line++;
    }
    return line;
}
