/* Raw-FSK image link bring-up (branch SSTV_SSDV), K1/K5 V3 only.
 *
 * Not APRS/AX.25 -- see integration.md's "KISS autonome" section for why
 * that path was abandoned. This reuses the BK4829's OTHER raw-FSK/FIFO
 * engine instead: the one already proven correct on this exact chip by the
 * stock AirCopy feature (its own hardware framing -- preamble + sync word +
 * declared length + FIFO, BK4819_ResetFSK()/SendFSKData()), just at a
 * different bit rate and a different (SSDV, 256 B) packet length. The RP2040
 * demodulates the far end with the same bit-cell PLL already proven for the
 * radiosonde decoders (sonde_demod.c) -- see rp2040/src/imgfsk_sync.h.
 *
 * Step 1 scope: send a small, REAL, pre-baked SSDV test image (produced
 * offline by the reference `ssdv` encoder, github.com/fsphil/ssdv -- round-
 * trip verified to decode back to the source photo) to prove the link
 * end-to-end before writing a live on-device JPEG->SSDV encoder for a
 * future camera.
 */
#ifndef APP_IMGFSK_TX_H
#define APP_IMGFSK_TX_H

#include <stdbool.h>

/* Transmit the embedded SSDV test image on whatever channel/frequency is
 * currently selected (like AirCopy / a roger beep -- no VFO borrow, the
 * operator tunes both ends to an agreed test frequency first).
 * `fsk2400`: false = 1200 baud, true = 2400 baud. Blocking. */
void IMGFSK_SendTestImage(bool fsk2400);

/* Zero-argument wrappers for the ACTION_OPT_* side-function-key table
 * (App/app/action.c expects `void (*)(void)`, same shape as APP_RunAprs) --
 * assign from MENU -> F1Shrt/F1Long/F2Shrt/F2Long, like SARSAT/APRS. */
void IMGFSK_Send1200(void);
void IMGFSK_Send2400(void);

#endif /* APP_IMGFSK_TX_H */
