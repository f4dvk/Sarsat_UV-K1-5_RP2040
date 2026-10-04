#ifndef APP_SSTV_TX_H
#define APP_SSTV_TX_H

#include <stdint.h>

/* Send the master image (streamed live from the RP2040) as an SSTV frame.
 * Blocking call, same "key up once for the whole transfer" model as the
 * (removed) IMGFSK test-image sender. Scottie 1: ~110 s. Martin 1: ~115 s.
 * PD90: ~90 s.
 *
 * ⚠️ (2026-10-03, retour utilisateur) PD90 etait documente ici a "~127 s",
 * copie-colle manifeste de la duree de PD120 (voir juste en dessous) : a
 * 320x256 avec PD90_PIXEL_US=532 (pas 190), le calcul exact (sync+porche+
 * 4 sous-balayages par paire de lignes, 128 paires) donne ~90 s, pas 127 --
 * verifie contre PD90_SYNC_US/PD90_PORCH_US/PD90_PIXEL_US (sstv_tx.c)
 * plutot que recopie d'un chiffre voisin.
 *
 * PD120: ~127 s (640x496, la plus grande des quatre -- d'abord tentee puis
 * abandonnee le 2026-09-26 quand son debit de 190 us/pixel a expose un vrai
 * bug du pipeline pixel TIM14 de la radio, puis re-ajoutee le 2026-09-27
 * une fois ce pipeline repense autour d'une file d'avance -- voir les
 * commentaires PD90/PD120 de sstv_tx.c pour l'historique complet). */
void SSTV_SendScottie1(void);
void SSTV_SendMartin1(void);
void SSTV_SendPD90(void);
void SSTV_SendPD120(void);

/* Calibration de l'horloge radio, sans emission (2026-10-04) : RP2040 ->
 * radio SSTV_CMD_CLK_START {duration_us:u32} puis SSTV_CMD_CLK_END {} --
 * voir SSTV_ClockStart/End. Definis dans sstv_rx.h (dispatch uart.c). */
void SSTV_ClockStart(void);
void SSTV_ClockEnd(void);
void SSTV_SetTrim(int16_t ppm);
int16_t SSTV_GetTrim(void);
void    SSTV_LoadTrim(void);

/* radio -> RP2040 (2026-10-04) : "lance une mesure d'horloge, renvoie-moi le
 * resultat" -- ecran SSTVCal (sstv_rx.c). Payload vide. */
#define SSTV_CMD_CLK_REQ 0x06F0u

/* Toggle the RP2040's SSTV test image between the real photo and the
 * ADRASEC logo (CMD_SSTV_IMAGE_SELECT, see sstv_tx.c and rp2040/src/
 * decoder_config.h) -- affects Scottie1/Martin1/PD90 immediately (next
 * SSTV_Send*() call); PD120 always uses the photo regardless of this
 * selector (see rp2040/src/main.c's sstv_image_pd120() comment: the
 * original Pico's flash doesn't have room for a second image at that
 * resolution). */
void SSTV_ToggleImage(void);

#endif
