/* sarsat_decoder.c — see sarsat_decoder.h. */
#include "sarsat_decoder.h"
#include "audio_slicer.h"
#include "country_codes.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

static const char *protocol_name(ProtocolType p)
{
    switch (p) {
    case PROTOCOL_STANDARD_LOCATION: return "Standard Location";
    case PROTOCOL_NATIONAL_LOCATION: return "National Location";
    case PROTOCOL_USER_PROTOCOL:     return "User-Location";
    case PROTOCOL_TEST:              return "Test";
    case PROTOCOL_EMERGENCY_ELT:     return "ELT-DT Location";
    case PROTOCOL_EMERGENCY_EPIRB:   return "Emergency EPIRB";
    case PROTOCOL_EMERGENCY_PLB:     return "Emergency PLB";
    case PROTOCOL_RLS_LOCATION:      return "RLS Location";
    case PROTOCOL_SHIP_SECURITY:     return "Ship Security";
    default:                         return "Unknown";
    }
}

static void put_line(sarsat_result_t *o, const char *fmt, ...)
{
    if (o->n_lines >= SARSAT_MAX_LINES) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(o->lines[o->n_lines], SARSAT_LINE_CHARS, fmt, ap);
    va_end(ap);
    o->n_lines++;
}

/* Emit `s` across as many lines as needed, breaking on spaces (<= 21 glyphs
 * per line). An optional `prefix` goes on the first line only. */
static void put_wrapped(sarsat_result_t *o, const char *prefix, const char *s)
{
    const int W = SARSAT_LINE_CHARS - 1;
    char buf[SARSAT_LINE_CHARS];
    int col = 0;

    if (prefix && *prefix) {
        col = (int)strlen(prefix);
        if (col > W) col = W;
        memcpy(buf, prefix, col);
    }

    while (*s == ' ') s++;
    while (*s && o->n_lines < SARSAT_MAX_LINES) {
        /* length of the next word */
        int wl = 0;
        while (s[wl] && s[wl] != ' ') wl++;

        if (col > 0 && col + 1 + wl > W) {          /* word won't fit -> flush */
            buf[col] = 0;
            put_line(o, "%s", buf);
            col = 0;
        }
        if (col > 0) buf[col++] = ' ';
        if (wl > W) wl = W;                          /* hard-split huge tokens */
        memcpy(buf + col, s, wl);
        col += wl;
        s += wl;
        while (*s == ' ') s++;
    }
    if (col > 0) { buf[col] = 0; put_line(o, "%s", buf); }
}

void sarsat_format_lines(sarsat_result_t *o)
{
    const BeaconInfo1G *in = &o->info;
    o->n_lines = 0;

    put_line(o, "1G %s%s  proto %u",
             (o->frame_bits == 144) ? "LONG" : "SHORT",
             in->is_test_message ? " TEST" : "", in->protocol_bits);
    if (in->bch1_fixed || in->bch2_fixed)
        /* un peu de bruit sur la liaison (ou l'enregistrement) a abime 1-3
         * bits, dans la marge garantie des codes BCH -- voir fix_crc1()/
         * fix_crc2() dans dec406_v1g.c -- la trame reste fiable. */
        put_line(o, "BCH fixed %u+%u bit", in->bch1_fixed, in->bch2_fixed);
    put_line(o, "ID %s", o->hex_id);
    put_wrapped(o, "", protocol_name(in->protocol));
    put_line(o, "Country %u", in->country_code);
    put_wrapped(o, "", get_country_name(in->country_code));

    if (in->position_default || (!in->has_position)) {
        put_line(o, "Position: none");
    } else if (in->lat != 0.0 || in->lon != 0.0) {
        put_line(o, "Lat %.5f %c",
                 fabs(in->lat), (in->lat >= 0) ? 'N' : 'S');
        put_line(o, "Lon %.5f %c",
                 fabs(in->lon), (in->lon >= 0) ? 'E' : 'W');
        /* composite vs base (ELT-DT / National with offsets) */
        if ((in->base_lat != in->lat || in->base_lon != in->lon) &&
            (in->base_lat != 0.0 || in->base_lon != 0.0))
            put_line(o, "base %.3f %.3f", in->base_lat, in->base_lon);
    } else {
        put_line(o, "Position: invalid");
    }

    if (in->vessel_id[0])
        put_wrapped(o, "", in->vessel_id);
}

/* Energie AC (variance x blk) du bloc de blk echantillons qui commence en p. */
static int64_t blk_energy(const int32_t *p, int blk)
{
    int64_t sum = 0, sq = 0;
    for (int k = 0; k < blk; k++) { sum += p[k]; sq += (int64_t)p[k] * p[k]; }
    return sq - sum * sum / blk;
}

/* Debut de salve dans la fenetre (echantillon ou demarrer le tranchage).
 *
 * Le tranchage (audio_slicer.c, algorithme moricef) seuille a 1 % du plus fort
 * pic de correlation vu depuis le debut, et ce pic ne redescend jamais. Or la
 * fenetre de capture n'est pas calee sur la salve : elle commence souvent par
 * du souffle pleine echelle (pas de porteuse), qui gonfle ce pic au point
 * qu'une salve plus faible ne franchit plus le seuil -- mesure au banc
 * gen406 : 100 % d'echec au niveau PA4, meme sans bruit sur la porteuse.
 *
 * La porteuse non modulee de tete (~160 ms) capture le recepteur : l'energie
 * du discriminateur s'effondre. On cherche le premier bloc de 5 ms dont
 * l'energie est au moins BURST_DROP fois plus faible que le plus fort bloc
 * deja vu, et qui le reste sur BURST_HOLD blocs (20 ms), puis on demarre
 * juste apres (le claquement de capture tombe dans ce premier bloc). Sans
 * chute nette -- fenetre qui commence deja dans la salve, ou souffle faible
 * -- on garde la fenetre entiere : comportement inchange. Pas de tableau
 * (pile du RP2040 serree, voir rp2040/CMakeLists.txt). */
#define BURST_DROP 4    /* rapport d'energie, ~6 dB */
#define BURST_HOLD 4    /* blocs consecutifs sous le seuil */

static int burst_start(const int32_t *s, int n, int rate)
{
    const int blk  = rate / 200;
    const int nblk = n / blk;
    if (blk < 8 || nblk < BURST_HOLD + 2)
        return 0;

    int64_t emax = 0;
    for (int i = 0; i + BURST_HOLD < nblk; i++) {
        int64_t e = blk_energy(s + i * blk, blk);
        if (e > emax) { emax = e; continue; }
        if (e * BURST_DROP >= emax)
            continue;
        int held = 1;
        for (int j = 1; j < BURST_HOLD && held; j++)
            held = blk_energy(s + (i + j) * blk, blk) * BURST_DROP < emax;
        if (held)
            return (i + 1) * blk;
    }
    return 0;
}

/* Un essai de tranchage + decodage sur samples[0..n-1] ; meme retour que
 * sarsat_decode_window(). */
static int decode_slice(const int32_t *samples, int n, int rate,
                        sarsat_result_t *out)
{
    memset(out, 0, sizeof(*out));

    uint8_t bits[SLICER_MAX_BITS];
    int len = slicer_run(samples, n, rate, bits);
    if (len != 112 && len != 144)
        return 0;                       /* no sync / no frame */

    out->frame_bits = len;

    char frame[145];
    for (int i = 0; i < len; i++) frame[i] = bits[i] ? '1' : '0';
    frame[len] = '\0';

    /* raw hex, 4 bits per nibble, sync bits included (matches dec406_hex) */
    for (int i = 0; i * 4 < len && i < (int)sizeof(out->raw_hex) - 1; i++) {
        int v = 0;
        for (int b = 0; b < 4 && i * 4 + b < len; b++)
            v = (v << 1) | bits[i * 4 + b];
        out->raw_hex[i] = "0123456789ABCDEF"[v & 0xF];
    }

    decode_1g_frame(frame, len, &out->info);
    out->crc_ok = !out->info.crc_error;

    /* Une trame CORRIGEE n'est acceptee qu'avec un mot de synchro de trame
     * exact (bits 16-24 : 000101111 normal, 011010000 test -- C/S T.001). Le
     * tranchage fabrique des trames-dechets a partir du souffle, et la
     * correction BCH en ramene une petite fraction sur un mot de code valide
     * (voir fix_crc1()) : ce mot de synchro, hors de la zone corrigee, les
     * ecarte a 99,6 %. Les trames valides SANS correction ne sont pas
     * concernees : comportement d'avant inchange pour elles. */
    if (out->crc_ok && (out->info.bch1_fixed || out->info.bch2_fixed) &&
        strncmp(frame + 15, "000101111", 9) != 0 &&
        strncmp(frame + 15, "011010000", 9) != 0)
        out->crc_ok = 0;

    if (!out->crc_ok)
        return -1;                      /* frame sliced but BCH uncorrectable */

    out->valid = 1;
    snprintf(out->hex_id, sizeof(out->hex_id), "%s", out->info.hex_id);
    sarsat_format_lines(out);
    return 1;
}

/* Deux essais, sans regression possible : la fenetre entiere d'abord (le
 * comportement d'origine, qui reste prioritaire), puis, faute de trame
 * valide, depuis le debut de salve (burst_start()). Le seuil du tranchage
 * depend de ce qui precede la salve dans les deux sens : un souffle fort le
 * gonfle et fait rater une salve faible (cas du banc gen406), mais sur une
 * salve forte il evite au contraire de reagir aux petites fluctuations de la
 * porteuse (cas de l'enregistrement reel du 2026-08-13, qui ne decode QUE
 * sur la fenetre entiere). Aucun des deux points de depart ne convient
 * toujours, d'ou les deux essais. Le second ne coute que sur un echec. */
int sarsat_decode_window(const int32_t *samples, int n, int rate,
                         sarsat_result_t *out)
{
    int rc = decode_slice(samples, n, rate, out);
    if (rc == 1)
        return 1;

    const int start = burst_start(samples, n, rate);
    if (start <= 0)
        return rc;

    int rc2 = decode_slice(samples + start, n - start, rate, out);
    if (rc2 != 0)
        return rc2;                     /* trame valide, ou tranchee mais BCH faux */
    if (rc < 0)                         /* le 1er essai etait alle plus loin : */
        return decode_slice(samples, n, rate, out);   /* on restitue son resultat */
    return 0;
}
