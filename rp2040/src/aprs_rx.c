/* aprs_rx.c — Bell-202 AFSK 1200 receiver + AX.25 de-framer. See aprs_rx.h.
 *
 * DSP ported from JN1DFF's pico_tnc (bell202.c / decode.c / filter.c, BSD-3).
 * Kept integer-only and amplitude-agnostic (envelope-tracked slicer) so the
 * same code runs on the RP2040 and on the host test.
 */
#include "aprs_rx.h"
#include <string.h>

/* ---- FIR coefficients (Hamming-windowed, generated for fs = 13200 Hz) ----
 * BPF  900..2500 Hz, 25 taps. Symmetric, so tap order does not matter. */
static const int16_t APRS_BPF_AN[25] = {
       132,    136,     43,    -76,     75,    468,    202,  -1606,
     -3993,  -4136,   -321,   5266,   7944,   5266,   -321,  -4136,
     -3993,  -1606,    202,    468,     75,    -76,     43,    136,
       132,
};

/* Post-discriminator low-pass: 13-tap Hamming, cut ~1500 Hz (fs 13200). Removes
 * the delay-and-multiply 2f ripple. Half the length of the previous 27-tap FIR:
 * that one spanned ~2.5 bit periods and smeared every bit edge into its
 * neighbours (inter-symbol interference) -- on a strong local signal that still
 * left 1-3 scattered bit errors per frame, enough to corrupt the HDLC framing
 * (erratic frame lengths for one station in the RX diagnostics). Symmetric, so
 * tap order is free. DC gain 8 (taps sum 32768, fir_run >> 12) keeps the old
 * scale so env / th / APRS_SLICE_MIN need no retuning. */
static const int16_t APRS_LPF_AN[13] = {
    -132, -128, 238, 1652, 4157, 6703, 7788, 6703, 4157, 1652, 238, -128, -132,
};

/* delay-and-multiply lag: ~446 us (pico_tnc DELAY_US), rounded to samples */
#define APRS_DELAY_US      446

/* carrier detect on the DC-removed sample energy (adaptive-ish, fixed gates) */
#define APRS_CDT_ON        1200
#define APRS_CDT_OFF        400

/* slicer: hysteresis = env/4, floored so silence does not chatter */
#define APRS_SLICE_MIN     24

/* IF limiter: an FM/AFSK discriminator is amplitude-independent, so hard-slice
 * the band-pass output to a fixed magnitude before the delay-and-multiply. A
 * strong (even ADC-clipped) signal then demodulates exactly like a nominal
 * one, and unlike an AGC there are no gain dynamics to add bit jitter. The BPF
 * ahead of it removes the harmonics a square wave would otherwise raise, and
 * the limiter only runs with a carrier present (never on bare noise). */
#define APRS_LIM_MAG       6000

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

/* AX.25 has no FEC. Try to repair a single bit error: for each bit position,
 * flip it and re-test the FCS (crc is cheap, and this only runs on a failure).
 * Keeps the flipped byte on success. */
bool aprs_try_fix(uint8_t *d, int n)
{
    for (int i = 0; i < n; i++) {
        for (int b = 0; b < 8; b++) {
            d[i] ^= (uint8_t)(1u << b);
            if (aprs_fcs_residue(d, n) == 0x0F47)
                return true;
            d[i] ^= (uint8_t)(1u << b);
        }
    }
    return false;
}

/* sanity-check an AX.25 UI frame: address field is 7-byte blocks ending with
 * the extension bit, then control 0x03 and PID 0xF0. Guards the bit-flip
 * repair against a false FCS match producing a garbage "packet". */
static bool ax25_looks_like_ui(const uint8_t *d, int len)
{
    int pos = 6;                         /* first SSID byte */
    for (int blk = 0; blk < 10; blk++) {
        if (pos >= len)
            return false;
        if (d[pos] & 1) {                /* extension bit -> end of addresses */
            return pos + 3 <= len && d[pos + 1] == 0x03 && d[pos + 2] == 0xF0;
        }
        pos += 7;
    }
    return false;
}

#define APRS_DIAG(FR, L, RES, UI) \
    do { if (r->fcb) r->fcb(r->fcb_user, (FR), (L), chain, (RES), (UI)); } while (0)

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
        for (int i = 0; i < len && !ok; i++)
            for (int b = 0; b < 8 && !ok; b++) {
                c->data[i] ^= (uint8_t)(1u << b);
                if (aprs_fcs_residue(c->data, len) == 0x0F47 &&
                    ax25_looks_like_ui(c->data, len - 2))
                    ok = true;
                else
                    c->data[i] ^= (uint8_t)(1u << b);
            }
        if (ok)
            r->n_fixed++;
    }

    const int ui = ax25_looks_like_ui(c->data, len - 2);

    if (!ok) {
        r->n_fcs_bad++;
        APRS_DIAG(c->data, len, APRS_RXR_FCS_BAD, ui);
        return;
    }

    const int plen = len - 2;                          /* strip the FCS */

    /* de-dup: the same frame usually decodes on several chains at once */
    if (r->last_len == plen &&
        (r->now - r->last_at) < (uint32_t)(APRS_RX_SAMPLE_RATE_HZ * 3 / 2) &&
        memcmp(r->last, c->data, (size_t)plen) == 0) {
        APRS_DIAG(c->data, plen, APRS_RXR_DEDUP, ui);
        return;
    }
    memcpy(r->last, c->data, (size_t)plen);
    r->last_len = plen;
    r->last_at  = r->now;

    r->n_packets++;
    APRS_DIAG(c->data, plen, res, ui);
    if (r->cb)
        r->cb(c->data, plen, r->user);
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
    fir_init(&r->lpf, APRS_LPF_AN, 13);

    /* slice at -3/8, 0, +3/8 of the envelope -- an AC-coupled / diode-clamped
     * feed (the C-Board) makes the discriminator output asymmetric, so the
     * best decision point is often not zero. */
    static const int32_t bias[APRS_RX_SLICERS] = { -3, 0, 3 };
    for (int i = 0; i < APRS_RX_SLICERS; i++) {
        r->ch[i].bias_num = (i < (int)(sizeof bias / sizeof bias[0])) ? bias[i] : 0;
        r->ch[i].state    = HDLC_HUNT;
    }
}

bool aprs_rx_carrier(const aprs_rx_t *r) { return r->cdt; }

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

    r->n_samp++;
    if (raw <= 1 || raw >= 4094)                    /* ADC rail = input clipping */
        r->n_clip++;

    /* DC removal (~5 ms time constant) */
    r->dc += ((raw << 8) - r->dc) >> 9;
    int32_t s = raw - (r->dc >> 8);                 /* AC, roughly +-2048 */

    /* carrier detect on short-term energy */
    r->cdt_lvl += ((s * s) - r->cdt_lvl) >> 7;
    if (!r->cdt && r->cdt_lvl > APRS_CDT_ON) {
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

    /* band-pass, then HARD-limit (sign slice), then delay-and-multiply */
    int32_t v = fir_run(&r->bpf, s);                 /* ~ +-16000 */
    v = (v > 0) ? APRS_LIM_MAG : (v < 0) ? -APRS_LIM_MAG : 0;

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
    int32_t th = r->env >> 2;
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
            c->pll -= (uint32_t)((int32_t)c->pll >> 2);
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
