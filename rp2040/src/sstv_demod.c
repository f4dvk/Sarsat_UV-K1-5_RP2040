/* sstv_demod.c -- see sstv_demod.h. Pure C, no pico-sdk, host-testable. */
#include "sstv_demod.h"

#include <string.h>

enum {
    ST_HUNT = 0,      /* looking for a sustained ~1900 Hz leader tone */
    ST_WAIT_BREAK,    /* leader seen, waiting for the dip to ~1200 Hz */
    ST_IN_BREAK,      /* in the ~1200 Hz break, waiting for the rise back */
    ST_LEADER2,       /* anchor set at the break->leader2 edge; riding out
                        * the remaining ~300 ms of the second leader tone */
    /* ⚠️ (2026-10-03, VRAIE cause de "lines=0" malgre VIS/parite trouves --
     * voir sstv_tx.c, sstv_send_vis(), "BUG PROTOCOLAIRE... un dixieme
     * element que nous n'avons jamais envoye" -- le TX radio ENVOIE bien un
     * bit de START (1200 Hz, 30 ms) entre le 2e leader et les 7 bits de
     * donnees, conforme a la norme SSTV reelle, mais ce decodeur RX n'a
     * jamais ete mis a jour pour s'attendre a cette 10e fenetre -- il
     * enchainait directement sur 7 fenetres de "donnees" des la fin du
     * leader2, lisant donc systematiquement la QUEUE du bit de start comme
     * bit0, chaque bit reel decale d'un cran, et le vrai bit6 perdu dans la
     * case "parite" (bits#, retour terrain : code VIS lu = 120 = 60*2 pour
     * Scottie 1, exactement un decalage d'un cran avec un 0 insere en bas).
     * Consomme maintenant cette fenetre de 30 ms separement, SANS la
     * compter comme un bit de donnees, avant d'entrer dans ST_VIS_WINDOW. */
    ST_VIS_START,     /* 30 ms start bit (1200 Hz), consumed but not stored */
    ST_VIS_WINDOW,    /* 9 x 30 ms windows: 7 data bits, parity, stop */
    ST_IMG_FIRSTSYNC, /* Scottie quirk: extra 9 ms sync before line 0 only */
    ST_IMG_GAP1,      /* 1.5 ms gap before Green */
    ST_IMG_GREEN,
    ST_IMG_GAP2,      /* 1.5 ms gap before Blue */
    ST_IMG_BLUE,
    ST_IMG_SYNC,      /* 9 ms sync before Red, every line */
    ST_IMG_GAP3,      /* 1.5 ms gap before Red */
    ST_IMG_RED,
};

#define HZ(v)   ((uint32_t)(v) * 1000u)   /* Hz -> milli-Hz */

#define LEADER_HZ        HZ(1900)
#define BREAK_HZ         HZ(1200)
#define BAND_TOL         HZ(60)   /* +/- tolerance for leader/break bands */

#define SCOTTIE1_SYNC_HZ   HZ(1200)
#define SCOTTIE1_BLACK_HZ  HZ(1500)
#define SCOTTIE1_WHITE_HZ  HZ(2300)

static uint32_t us_to_samples(const sstv_demod_t *d, uint32_t us)
{
    return (uint32_t)(((uint64_t)d->sample_rate_hz * us) / 1000000u);
}

void sstv_demod_init(sstv_demod_t *d, uint32_t sample_rate_hz)
{
    memset(d, 0, sizeof *d);
    d->sample_rate_hz = sample_rate_hz;
    d->pixel = -1;
    d->state = ST_HUNT;
}

/* DC-centre + zero-crossing frequency estimate, same adaptive envelope
 * trick as sonde_demod.c, but reporting a continuous instantaneous
 * frequency (linearly-interpolated fractional-sample crossing time)
 * instead of slicing a bit. */
static void sstv_update_freq(sstv_demod_t *d, int32_t sample)
{
    d->lp += (sample - d->lp) >> 2;

    if (d->lp > d->lp_hi) d->lp_hi += (d->lp - d->lp_hi) >> 2;
    else                  d->lp_hi -= (d->lp_hi - d->lp_lo) >> 10;
    if (d->lp < d->lp_lo) d->lp_lo += (d->lp - d->lp_lo) >> 2;
    else                  d->lp_lo += (d->lp_hi - d->lp_lo) >> 10;

    int32_t centered = d->lp - ((d->lp_hi + d->lp_lo) >> 1);

    d->samples_since_cross++;

    if (d->prev_centered < 0 && centered >= 0) {
        int32_t denom = centered - d->prev_centered;
        int32_t frac_q10 = 0;
        if (denom != 0) {
            frac_q10 = (int32_t)(((int64_t)(-d->prev_centered) * 1024) / denom);
            if (frac_q10 < 0) frac_q10 = 0;
            if (frac_q10 > 1024) frac_q10 = 1024;
        }
        /* true period = samples_since_cross + frac_cur - frac_prev -- see
         * sstv_demod.h's warning on why frac_prev cannot be dropped. */
        int32_t period_q10 = ((int32_t)d->samples_since_cross << 10)
                            + frac_q10 - d->frac_prev_q10;
        if (period_q10 > 0) {
            d->freq_mhz = (uint32_t)(((uint64_t)d->sample_rate_hz * 1000u * 1024u)
                                      / (uint32_t)period_q10);
            d->have_freq = true;
        }
        d->frac_prev_q10 = frac_q10;
        d->samples_since_cross = 0;
    }
    d->prev_centered = centered;
}

static void sstv_enter(sstv_demod_t *d, int state)
{
    d->state = state;
    d->state_samples = 0;
    d->hz_accum = 0;
    d->hz_accum_n = 0;
}

static uint8_t sstv_luma_from_freq(uint32_t freq_mhz)
{
    int32_t v = ((int32_t)(freq_mhz / 1000u) - 1500) * 255;
    v /= (2300 - 1500);
    if (v < 0)   v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

/* One VIS window (30 ms): classify the boxcar-averaged frequency as a data
 * bit (1100 Hz = 1, 1300 Hz = 0, threshold at the 1200 Hz midpoint) --
 * bit_idx 7 is the parity bit, 8 is the stop bit (consumed for timing only,
 * its value is not checked). Returns true once the stop bit window closes,
 * with *vis_ok telling whether the parity check passed. */
static bool sstv_vis_window_done(sstv_demod_t *d)
{
    uint32_t avg = d->hz_accum_n ? (uint32_t)(d->hz_accum / d->hz_accum_n) : 0;
    int bit = (avg < HZ(1200)) ? 1 : 0;

    if (d->vis_bit_idx < 7) {
        if (bit) d->vis_bits_val = (uint8_t)(d->vis_bits_val | (1u << d->vis_bit_idx));
        d->vis_bit_idx++;
        return false;
    }
    if (d->vis_bit_idx == 7) {
        int ones = 0;
        for (int i = 0; i < 7; i++)
            if (d->vis_bits_val & (1u << i)) ones++;
        d->vis_parity_ok = ((ones + bit) & 1) == 0;   /* even parity overall */
        d->vis_bit_idx++;
        return false;
    }
    /* idx == 8: stop bit window just finished */
    return true;
}

bool sstv_demod_sample(sstv_demod_t *d, int32_t sample)
{
    sstv_update_freq(d, sample);
    d->state_samples++;
    if (d->have_freq) {
        d->hz_accum += d->freq_mhz;
        d->hz_accum_n++;
    }

    switch (d->state) {

    case ST_HUNT:
        if (!d->have_freq) return false;
        if (d->freq_mhz > LEADER_HZ - BAND_TOL && d->freq_mhz < LEADER_HZ + BAND_TOL) {
            if (d->state_samples >= us_to_samples(d, 100000)) {  /* 100 ms seen */
                d->n_leader++;
                sstv_enter(d, ST_WAIT_BREAK);
            }
        } else {
            d->state_samples = 0;   /* band lost -- restart the run-length count */
        }
        return false;

    case ST_WAIT_BREAK:
        if (d->have_freq && d->freq_mhz > BREAK_HZ - BAND_TOL && d->freq_mhz < BREAK_HZ + BAND_TOL) {
            d->n_break++;
            sstv_enter(d, ST_IN_BREAK);
        } else if (d->state_samples > us_to_samples(d, 400000))   /* gave up */
            sstv_enter(d, ST_HUNT);
        return false;

    case ST_IN_BREAK:
        if (d->have_freq && d->freq_mhz > LEADER_HZ - BAND_TOL) {
            /* rising edge back towards the leader band: only a real VIS
             * break if it lasted a plausible 3-20 ms */
            uint32_t us = (uint32_t)(((uint64_t)d->state_samples * 1000000u) / d->sample_rate_hz);
            if (us >= 3000 && us <= 20000) {
                d->n_leader2++;
                sstv_enter(d, ST_LEADER2);
            } else
                sstv_enter(d, ST_HUNT);
        } else if (d->state_samples > us_to_samples(d, 30000)) {
            sstv_enter(d, ST_HUNT);   /* stayed low too long -- not a break */
        }
        return false;

    case ST_LEADER2:
        if (d->state_samples >= us_to_samples(d, 300000))
            sstv_enter(d, ST_VIS_START);
        return false;

    case ST_VIS_START:
        if (d->state_samples >= us_to_samples(d, 30000)) {
            d->vis_bits_val = 0;
            d->vis_bit_idx  = 0;
            sstv_enter(d, ST_VIS_WINDOW);
        }
        return false;

    case ST_VIS_WINDOW:
        if (d->state_samples >= us_to_samples(d, 30000)) {
            bool done = sstv_vis_window_done(d);
            if (done) {
                d->n_vis_done++;
                if (d->vis_parity_ok && d->vis_bits_val == 60) {   /* Scottie 1 */
                    d->n_vis_ok++;
                    d->line = 0;
                    sstv_enter(d, ST_IMG_FIRSTSYNC);
                } else {
                    sstv_enter(d, ST_HUNT);
                }
                return false;
            }
            d->state_samples = 0;
            d->hz_accum = 0;
            d->hz_accum_n = 0;
        }
        return false;

    case ST_IMG_FIRSTSYNC:
        if (d->state_samples >= us_to_samples(d, 9000)) {
            d->pixel = -1;
            sstv_enter(d, ST_IMG_GAP1);
        }
        return false;

    case ST_IMG_GAP1:
        if (d->state_samples >= us_to_samples(d, 1500))
            sstv_enter(d, ST_IMG_GREEN);
        return false;

    case ST_IMG_GAP2:
        if (d->state_samples >= us_to_samples(d, 1500))
            sstv_enter(d, ST_IMG_BLUE);
        return false;

    case ST_IMG_SYNC:
        if (d->state_samples >= us_to_samples(d, 9000))
            sstv_enter(d, ST_IMG_GAP3);
        return false;

    case ST_IMG_GAP3:
        if (d->state_samples >= us_to_samples(d, 1500))
            sstv_enter(d, ST_IMG_RED);
        return false;

    case ST_IMG_GREEN:
    case ST_IMG_BLUE:
    case ST_IMG_RED: {
        /* ⚠️ (2026-10-03, retour terrain : "lines=0"... non, "que du bruit"
         * -- voir sstv_demod.h's history) chaque pixel ne prenait qu'UNE
         * SEULE lecture instantanee de d->freq_mhz, pile au moment ou l'on
         * bascule sur px suivant -- polluee par le cycle en cours, souvent
         * entame dans le pixel PRECEDENT. Corrige en faisant la MOYENNE
         * (comme sstv_vis_window_done() le fait deja pour le VIS) de toutes
         * les lectures de freq_mhz tombees PENDANT la fenetre de CE pixel
         * (d->hz_accum/d->hz_accum_n, deja accumules en haut de cette
         * fonction, remis a zero ICI a chaque frontiere de pixel). Une
         * tentative d'aller plus loin (discriminateur FM en quadrature,
         * ~21 mesures/pixel) a ete essayee et abandonnee -- voir
         * sstv_demod.h's long comment -- elle introduisait un regime
         * transitoire pire (~44 colonnes de corruption apres chaque
         * transition franche), decouvert en validation synthetique avant
         * meme d'atteindre le terrain.
         *
         * ⚠️ (2026-10-03, retour terrain : "l'image en Scottie 1 est de
         * travers") -- cause reelle trouvee en VALIDATION SYNTHETIQUE
         * (signal parfait, AUCUNE derive d'horloge simulee, voir le journal
         * de cette session) : 432 us a 48 kHz, c'est 20.736 echantillons --
         * PAS un nombre entier. us_to_samples() tronque a 20 (division
         * entiere), et px = state_samples/20 fait donc avancer l'index de
         * pixel ~3.5% plus vite que le signal reel -- une erreur FIXE et
         * DETERMINISTE (le meme bug se reproduit identiquement sur
         * n'importe quel exemplaire de materiel, aucun rapport avec un
         * ecart d'horloge entre deux cartes), mais qui s'accumule sur les
         * 320 pixels d'UN SEUL canal (~4,9 ms, deja plus d'un pixel) puis
         * ligne apres ligne sur toute l'image (256 lignes) -- bien assez
         * pour expliquer une image visiblement de travers. Corrige en
         * calculant px directement via le ratio EXACT
         * (state_samples * 1e6) / (432 * sample_rate_hz) en arithmetique
         * 64 bits, sans jamais arrondir une "duree par pixel" au prealable
         * -- aucune derive possible, quel que soit le nombre de pixels ou
         * de lignes. Verifie par un test synthetique multi-lignes ad hoc
         * (pas dans la suite committee) : l'erreur de position d'un front
         * vertical connu reste nulle sur 40 lignes apres ce correctif, la
         * ou elle depassait +-160 px (l'image entiere) avant. */
        uint32_t px_u = (uint32_t)(((uint64_t)d->state_samples * 1000000ULL) /
                                    ((uint64_t)432u * d->sample_rate_hz));
        int px = (px_u > (uint32_t)SSTV_SCOTTIE1_WIDTH) ? SSTV_SCOTTIE1_WIDTH : (int)px_u;
        int ch = (d->state == ST_IMG_GREEN) ? SSTV_CH_GREEN :
                 (d->state == ST_IMG_BLUE)  ? SSTV_CH_BLUE  : SSTV_CH_RED;
        if (px != d->pixel) {
            if (d->pixel >= 0 && d->pixel < SSTV_SCOTTIE1_WIDTH && d->hz_accum_n)
                d->frame[ch][d->pixel] =
                    sstv_luma_from_freq((uint32_t)(d->hz_accum / d->hz_accum_n));
            d->pixel = px;
            d->hz_accum = 0;
            d->hz_accum_n = 0;
        }
        if (px_u >= (uint32_t)SSTV_SCOTTIE1_WIDTH) {
            if (d->pixel >= 0 && d->pixel < SSTV_SCOTTIE1_WIDTH && d->hz_accum_n)
                d->frame[ch][d->pixel] =
                    sstv_luma_from_freq((uint32_t)(d->hz_accum / d->hz_accum_n));
            d->pixel = -1;
            if (d->state == ST_IMG_GREEN) {
                sstv_enter(d, ST_IMG_GAP2);
            } else if (d->state == ST_IMG_BLUE) {
                sstv_enter(d, ST_IMG_SYNC);
            } else {
                /* Red just finished: d->line is still the line that was
                 * just captured -- report it via line_ready_idx, THEN
                 * advance d->line to whatever gets scanned next. */
                d->line_ready_idx = d->line;
                if (d->line + 1 >= SSTV_SCOTTIE1_HEIGHT) {
                    d->line = 0;
                    sstv_enter(d, ST_HUNT);   /* frame complete -- hunt again */
                } else {
                    d->line++;
                    sstv_enter(d, ST_IMG_GAP1);   /* next line: no extra sync */
                }
                return true;
            }
        }
        return false;
    }

    default:
        sstv_enter(d, ST_HUNT);
        return false;
    }
}
