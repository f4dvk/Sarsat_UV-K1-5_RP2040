/* imgfsk_sync.h -- frame-sync hunter for the SSTV_SSDV image link (branch
 * SSTV_SSDV). Pure C, no pico-sdk, host-testable -- same shape as
 * sonde_sync.h, which this is modelled on directly.
 *
 * ⚠️ (2026-09-25) RESTORED. This is the project's SECOND attempt at RX: the
 * first (this same file, removed at commit 46e95f0) was set aside in favour
 * of decoding directly in the BK4819/29's own hardware raw-FSK/FIFO engine
 * (the one AirCopy uses) -- not because this software path was unreliable,
 * but because that earlier test needed a C-Board on BOTH ends to try a
 * TX->RX link, versus one C-Board on the RX end only for the hardware path.
 * The hardware path was then extensively field-debugged (~30 commits) but
 * never reached reliable multi-packet reception: the BK4819/29's raw-FSK
 * correlator intermittently stops generating any interrupt after a handful
 * of packets, for a reason that survived registers-identical-to-AirCopy,
 * every re-arm sequence tried, WIDE/NARROW, FM/RAW modulation, AFC disable,
 * the F4HWN auto-sleep timer, and removing the RP2040 link entirely from the
 * loop (still reproduced with the radio's C-Board physically unplugged) --
 * so it is a real, still-unexplained hardware/firmware interaction, not
 * something diagnosable from here. Given TX only ever needed ONE radio
 * either way (the far end doesn't need a C-Board to transmit), this software
 * path is now reinstated for RX, reusing this project's own proven-reliable
 * RAW/DSC audio pipeline (the same one SARSAT and Sonde already decode from
 * for hours at a time without this kind of lock-up) instead of depending on
 * that hardware block.
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
 * ⚠️ (2026-09-25) RESOLVED. The documented sync word (0x85CFAB45) never
 * matched a real capture at first, under any bit-order/inversion/NRZI
 * hypothesis -- two independent raw-bit captures right after the preamble
 * (itself correctly recognised) agreed with EACH OTHER on 61/64 bits (far
 * too reproducible to be noise) but not with the datasheet value, and
 * decoded packets that DID then sync-lock on that observed value never
 * looked like valid SSDV. Real cause (see imgfsk_tx.c's own comment):
 * REG_59's "Enable FSK Scramble" bit (13) was active on TX, and the chip
 * only descrambles automatically IN HARDWARE when the far end also uses its
 * own raw-FSK correlator for RX (BK4819_PrepareFSKReceive() sets the same
 * bit) -- with RX now done in software (raw audio, no correlator), that
 * automatic descrambling never happened, so the hunt was matching the
 * SCRAMBLED preamble tail, not the true sync. Scrambling is now disabled on
 * TX instead of reverse-engineering the (undocumented) descrambler LFSR --
 * the documented 0x85CFAB45 is correct once the source never scrambles. */
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
