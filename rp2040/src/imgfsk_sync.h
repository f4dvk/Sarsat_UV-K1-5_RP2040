/* imgfsk_sync.h -- frame-sync hunter for the SSTV_SSDV raw-FSK image link
 * (branch SSTV_SSDV). Pure C, no pico-sdk, host-testable -- same shape as
 * sonde_sync.h, which this is modelled on directly.
 *
 * The far end (K1/K5 V3, patch/imgfsk_tx.c) drives the BK4829's raw-FSK/FIFO
 * engine -- the one already proven correct by the stock AirCopy feature --
 * at 1200 or 2400 baud, one independently-framed 256-byte SSDV packet at a
 * time: [~7-byte 0xAA/0x55 preamble][4-byte sync 0x85 0xCF 0xAB 0x45][256 B
 * SSDV packet]. The preamble is only there to let this side's bit-cell PLL
 * (sonde_demod.h, reused as-is, just at a different baud) settle before the
 * sync word arrives -- the correlator below ignores it on its own, the same
 * way sonde_sync.c's RS41/M10 hunt ignores whatever precedes their own sync
 * words.
 *
 * ⚠️ Unverified assumption, first thing to check if sync never locks on a
 * real bench capture: the BK4819(V3) Application Note documents the SYNC
 * BYTE *order* (byte 0 first, then 1, 2, 3) but not the bit order *within*
 * each byte. This hunts for 0x85CFAB45 assuming MSB-first-per-byte (the
 * same convention already proven for RS41's sync word in sonde_sync.c) --
 * if that turns out wrong, the fix is a one-line bit-reversal of the
 * pattern, same as M10's header needed the *opposite* (LSB-first) choice.
 */
#ifndef IMGFSK_SYNC_H
#define IMGFSK_SYNC_H

#include <stdint.h>
#include <stdbool.h>

#define IMGFSK_PACKET_SIZE 256   /* one SSDV packet */

typedef struct {
    uint32_t sr;         /* 32-bit rolling shift register, hunt state */
    bool     in_frame;
    uint8_t  cur_byte;
    int      cur_bits;
    int      frame_len;
    uint8_t  frame[IMGFSK_PACKET_SIZE];
} imgfsk_sync_t;

void imgfsk_sync_init(imgfsk_sync_t *s);

/* Feed one demodulated bit (from sonde_demod_sample(), same as the
 * radiosonde decoders). Returns true when a full 256-byte packet has just
 * been captured -- its bytes are in s->frame, MSB-first, ready to hand to
 * a PC-side `ssdv` decoder as-is (no further processing needed here: SSDV's
 * own CRC/Reed-Solomon live inside the packet payload, not this framing). */
bool imgfsk_sync_feed(imgfsk_sync_t *s, uint8_t bit);

#endif /* IMGFSK_SYNC_H */
