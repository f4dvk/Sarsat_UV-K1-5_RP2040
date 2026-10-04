/* sstv_demod.h -- SSTV (Scottie 1 only, for now) receiver: instantaneous-
 * frequency tracking + VIS header detection + per-line RGB decode. Pure C,
 * no pico-sdk, host-testable (see test/host/test_sstv_demod.c).
 *
 * branch SSTV_SSDV: this replaces the abandoned IMGFSK/SSDV image link (see
 * decoder_config.h and git history). Unlike IMGFSK's binary two-level FSK
 * (a bit-cell PLL, sonde_demod.h's job), SSTV encodes each pixel as a
 * continuously-varying AUDIO TONE frequency (1500 Hz black .. 2300 Hz
 * white) -- the right tool here is an instantaneous-frequency estimator,
 * not a bit slicer. Method: DC-centre the signal (same adaptive hi/lo
 * envelope trick as sonde_demod.c/aprs_rx.c), find upward zero-crossings,
 * and linearly interpolate the fractional-sample crossing time between the
 * two straddling samples -- this gives sub-sample-period timing resolution
 * from plain integer/fixed-point math, no NCO/quadrature mixer or floats
 * needed in the per-sample path. One frequency estimate comes out roughly
 * once per audio cycle (~435-667 us in this frequency range), which is
 * comparable to Scottie 1's own 432 us/pixel dwell time -- adequate but not
 * generous margin, expect this to be the first thing worth revisiting from
 * real-air feedback (this project's established pattern: see sonde_demod.h,
 * imgfsk_sync.h's own "unverified on real air" warnings before their first
 * field tests).
 *
 * ⚠️ The period between two crossings is NOT simply the sample count
 * between them: each crossing's own fractional-sample offset (frac_prev_q10
 * below) must be carried over and subtracted, or the estimate has a
 * systematic multi-percent error (~75 Hz at 48 kHz/1900 Hz measured while
 * developing this against a synthetic host-test tone -- large enough to
 * make every VIS/luminance threshold in this file unusable). Fixed and
 * verified against a clean synthetic sine before ever touching real
 * hardware, the same discipline this project's other decoders follow.
 *
 * Timing (VIS header + Scottie 1 scan-line structure) is tracked mostly by
 * elapsed SAMPLE COUNT from two anchor points: the break->leader2 edge (for
 * the VIS header) and the VIS-decode-complete instant (for the image scan).
 * No active per-line re-synchronisation yet (a real decoder would re-lock
 * on each line's sync pulse to absorb clock drift over Scottie 1's ~110 s
 * frame) -- deferred until real hardware shows whether it is actually
 * needed (image slant would be the symptom). Protocol source: the SSTV
 * Handbook (Martin Bruchanov OK2MNM, sstv-handbook.com, chapter 4).
 *
 * ⚠️ (2026-10-03, retour terrain : image toujours bruitee par endroits --
 * petits eclats sombres disperses, pas alignes avec une forme coherente --
 * meme apres avoir corrige le moyennage par pixel ci-dessous) -- tentative
 * d'aller plus loin avec un vrai discriminateur FM en quadrature (melangeur
 * I/Q + filtre passe-bas + produit croise/atan2f, comme un discriminateur
 * FM logiciel classique type gqrx/SDR#) pour moyenner ~21 mesures par pixel
 * au lieu d'une seule -- ABANDONNEE apres validation synthetique : le
 * filtre passe-bas, necessairement lent pour rejeter l'image 2*f0 du
 * melangeur, met des DIZAINES de pixels a se stabiliser apres tout saut de
 * frequence important (un battement entre l'ancienne et la nouvelle
 * frequence, qui ne s'attenue qu'a l'echelle du temps de reponse du
 * filtre) -- confirme sur signal synthetique propre : ~44 colonnes de
 * corruption apres chaque transition franche (fin d'un canal de couleur,
 * debut du suivant), bien pire que le bruit disperse qu'il visait a
 * corriger, puisque CE projet decode des images avec de vrais contours
 * (texte, traits) a chaque ligne, pas seulement des aplats. Abandonnee
 * AVANT le terrain, exactement le but de valider contre un signal
 * synthetique propre d'abord -- cf plus haut. Reste sur le passage par
 * zero (moins de mesures par pixel, mais aucun regime transitoire de ce
 * genre) avec le moyennage par pixel ci-dessous ; une vraie amelioration
 * demanderait un filtre complexe correctement concu (ex. Hilbert + mixage,
 * pas un simple passe-bas reel en cascade), un chantier a part entiere. */
#ifndef SSTV_DEMOD_H
#define SSTV_DEMOD_H

#include <stdint.h>
#include <stdbool.h>

#define SSTV_SCOTTIE1_WIDTH  320
#define SSTV_SCOTTIE1_HEIGHT 256

/* channel indices into sstv_demod_t.frame[][], matching Scottie's own G-B-R
 * transmission order */
#define SSTV_CH_GREEN 0
#define SSTV_CH_BLUE  1
#define SSTV_CH_RED   2

typedef struct {
    /* DC-centring (adaptive hi/lo envelope, same shape as sonde_demod_t) */
    int32_t lp, lp_hi, lp_lo;
    int32_t prev_centered;

    /* Zero-crossing frequency estimator */
    uint32_t sample_rate_hz;
    uint32_t samples_since_cross;
    int32_t  frac_prev_q10;    /* fractional-sample position (Q10) of the
                                 * last crossing within its own bracketing
                                 * sample pair -- MUST be carried over and
                                 * subtracted from the next period, or the
                                 * result has a systematic multi-percent
                                 * error (see this session's derivation) */
    uint32_t freq_mhz;         /* latest instantaneous frequency, milli-Hz */
    bool     have_freq;

    /* VIS header + image state machine */
    int      state;
    uint32_t state_samples;    /* elapsed samples since entering d->state */
    uint64_t hz_accum;         /* running sum of freq_mhz over the current
                                 * window, for boxcar-averaged classification
                                 * (VIS bits) */
    uint32_t hz_accum_n;

    uint8_t  vis_bits_val;
    int      vis_bit_idx;      /* 0..6 data bits, 7 parity, 8 stop */
    bool     vis_parity_ok;    /* set when bit_idx 7 (parity) completes,
                                 * read back when bit_idx 8 (stop) completes
                                 * -- MUST live in the struct, not a local:
                                 * these are two separate top-level calls */

    int      line;             /* index of the line currently being scanned */
    int      line_ready_idx;   /* valid only right after a true return: the
                                 * 0-based index of the just-completed line */
    int      pixel;            /* -1 between pixels, else last pixel index
                                 * written in the channel currently scanning */
    uint8_t  frame[3][SSTV_SCOTTIE1_WIDTH];  /* [SSTV_CH_*][x], current line */

    /* ⚠️ (2026-10-03, diagnostic de regression temporaire -- "pas de
     * decodage", lines=0 persistant malgre une vraie emission Scottie 1 et
     * un signal qui bouge (env/lp non nul et variable)) : compteurs
     * cumulatifs a CHAQUE etape du pipeline VIS, pour savoir ou ca cale --
     * meme principe que n_hdlc/n_ok pour l'APRS interne (voir ce projet,
     * historique). A retirer une fois la regression identifiee. */
    uint32_t n_leader;    /* ST_HUNT -> ST_WAIT_BREAK (leader 1900 Hz trouve) */
    uint32_t n_break;     /* ST_WAIT_BREAK -> ST_IN_BREAK (creux 1200 Hz trouve) */
    uint32_t n_leader2;   /* ST_IN_BREAK -> ST_LEADER2 (front montant valide) */
    uint32_t n_vis_done;  /* ST_VIS_WINDOW termine (9 fenetres), bon ou mauvais */
    uint32_t n_vis_ok;    /* VIS accepte : parite bonne ET code Scottie 1 (60) */
} sstv_demod_t;

void sstv_demod_init(sstv_demod_t *d, uint32_t sample_rate_hz);

/* Feed one discriminator sample (same |x| <= ~32768 convention as
 * sonde_demod.h/audio_slicer.h). Returns true once a full line (Green+Blue+
 * Red, 320 px each) is ready in d->frame[][] -- its 0-based index is
 * d->line_ready_idx (NOT d->line, which by then already points at the next
 * line to be scanned). */
bool sstv_demod_sample(sstv_demod_t *d, int32_t sample);

#endif /* SSTV_DEMOD_H */
