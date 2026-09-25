/* imgfsk_sync.c -- see imgfsk_sync.h. Pure C, no pico-sdk, host-testable. */
#include "imgfsk_sync.h"

#include <string.h>

/* ⚠️ (2026-09-25, retour terrain : capture de bits bruts, "aucun decodage")
 * -- CORRIGE : 0x85CFAB45 (valeur par defaut documentee dans la note
 * d'application BK4819(V3)) ne matchait jamais, meme avec une tolerance
 * elargie et sous toutes les hypotheses d'ordre de bits testees (inversion,
 * octets inverses, NRZI). Deux captures independantes des bits demodules
 * juste apres le preambule (lui, correctement reconnu : ~59 bits alternes,
 * tres proche des 56 attendus) se sont revelees identiques a 61 bits pres
 * sur 64 -- bien trop reproductible pour etre du bruit, donc un vrai mot de
 * synchro, juste PAS celui documente. imgfsk_tx.c ne configure jamais
 * REG_5A/5B explicitement (il compte sur la valeur par defaut du silicium a
 * la mise sous tension) -- cette valeur reelle differe apparemment de celle
 * documentee, sur ce chip precis. Remplace par la valeur observee. */
#define IMGFSK_SYNC_PATTERN 0x8CDC72E8UL
#define IMGFSK_SYNC_MAX_ERR 2               /* out of 32 bits (~6.25%),
                                             * between RS41's (2/64) and
                                             * M10's (2/28) tolerance */

static int popcount32(uint32_t v)
{
#ifdef __GNUC__
    return __builtin_popcount(v);
#else
    int n = 0;
    while (v) { n += (int)(v & 1); v >>= 1; }
    return n;
#endif
}

void imgfsk_sync_init(imgfsk_sync_t *s)
{
    memset(s, 0, sizeof *s);
}

bool imgfsk_sync_feed(imgfsk_sync_t *s, uint8_t bit)
{
    if (!s->in_frame) {
        s->sr = (s->sr << 1) | (bit & 1u);
        if (popcount32(s->sr ^ IMGFSK_SYNC_PATTERN) <= IMGFSK_SYNC_MAX_ERR) {
            s->in_frame  = true;
            s->cur_byte  = 0;
            s->cur_bits  = 0;
            s->frame_len = 0;
        }
        return false;
    }

    s->cur_byte = (uint8_t)((s->cur_byte << 1) | (bit & 1u));
    if (++s->cur_bits == 8) {
        s->frame[s->frame_len++] = s->cur_byte;
        s->cur_byte = 0;
        s->cur_bits = 0;
    }

    if (s->frame_len >= IMGFSK_PACKET_SIZE) {
        s->in_frame = false;   /* back to hunting for the next packet */
        s->sr = 0;
        return true;
    }
    return false;
}
