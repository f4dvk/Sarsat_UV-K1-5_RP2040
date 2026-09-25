/* imgfsk_sync.c -- see imgfsk_sync.h. Pure C, no pico-sdk, host-testable. */
#include "imgfsk_sync.h"

#include <string.h>

/* ⚠️ (2026-09-25) Historique : 0x8CDC72E8 (une valeur trouvee empiriquement
 * par capture reelle) a brievement remplace la valeur documentee ici, le
 * temps de comprendre POURQUOI le mot de synchro documente ne matchait
 * jamais. Cause reelle trouvee depuis (voir imgfsk_tx.c) : le bit "Enable
 * FSK Scramble" (REG_59<13>) etait actif a l'emission, et la puce ne
 * desembrouille en materiel QUE quand le corelateur materiel BK4819 est
 * aussi utilise en reception (BK4819_PrepareFSKReceive() active le meme
 * bit) -- notre reception etant desormais logicielle (audio brut), ce
 * desembrouillage n'avait plus lieu, et 0x8CDC72E8 n'etait donc que la
 * version EMBROUILLEE du vrai mot de synchro. Le scramble desactive a
 * l'emission, la valeur documentee redevient valide -- revient donc a
 * 0x85CFAB45 (BK4819(V3) Application Note, Sync Byte 0..3, MSB-first). */
#define IMGFSK_SYNC_PATTERN 0x85CFAB45UL
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
