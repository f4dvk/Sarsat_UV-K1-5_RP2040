/* Raw-FSK image link, RX side (branch SSTV_SSDV), K1/K5 V3 only.
 *
 * Hardware demodulation on the BK4819/29's own raw-FSK/FIFO engine -- the
 * exact same one already proven correct, bidirectionally, by the stock
 * AirCopy feature (BK4819_PrepareFSKReceive()/App/driver/bk4829.c). This is
 * the RX counterpart of patch/imgfsk_tx.c's TX (which already works on the
 * air): same registers, same idea, just RX mode bits and continuous
 * listening instead of a one-shot burst.
 *
 * Each captured 256-byte SSDV packet is handed to the RP2040 as-is
 * (CMD_IMGFSK_RXPKT, decoder_config.h) -- this side does no SSDV/JPEG
 * processing, matching the "BK4819 demodulates, RP2040 processes" split.
 */
#ifndef APP_IMGFSK_RX_H
#define APP_IMGFSK_RX_H

/* Assignable to a side-function key (ACTION_OPT_*), like TX's own
 * IMGFSK_Send1200/2400: first press arms listening at that rate, a second
 * press on the SAME key disarms and returns to normal RX. Pressing the
 * OTHER rate's key while armed switches rate directly. */
void IMGFSK_ToggleRx1200(void);
void IMGFSK_ToggleRx2400(void);

/* Call every ~10 ms from APP_TimeSlice10ms(), unconditionally (cheap no-op
 * when not armed). Polls for a completed hardware capture and forwards it
 * to the RP2040; re-arms itself once after a TX (APRS_TxFrame()-style
 * routines reprogram every BK4819/29 register when they finish). */
void IMGFSK_TimeSlice(void);

#endif /* APP_IMGFSK_RX_H */
