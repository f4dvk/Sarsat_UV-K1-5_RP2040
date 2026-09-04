/* Minimal AX.25 UI-frame + HDLC encoder for APRS beaconing.
 *
 * Pure C, no radio deps -> host-testable (rp2040/test/host/test_ax25).
 * Part of the Sarsat_UV-K1-5_RP2040 project (APRS tracker, KD8CEC style).
 */
#ifndef APP_AX25_H
#define APP_AX25_H

#include <stdint.h>
#include <stdbool.h>

#define AX25_MAX_DIGI     2          /* WIDE1-1,WIDE2-1 is the usual max      */
#define AX25_MAX_INFO     80
#define AX25_MAX_FRAME    (7 + 7 + 7*AX25_MAX_DIGI + 2 + AX25_MAX_INFO + 2)
/* worst case tone bits: (preamble + frame*8 * 1.2 stuffing) */
#define AX25_MAX_BITS     2200

typedef struct {
    char    call[7];    /* NUL-padded, <= 6 chars, A-Z 0-9 */
    uint8_t ssid;       /* 0..15 */
} ax25_addr_t;

/* Assemble the AX.25 UI frame (address..control..PID..info..FCS) into `out`.
 * Returns the byte length, or 0 on bad input. */
int ax25_build_ui(uint8_t *out,
                  const ax25_addr_t *dst, const ax25_addr_t *src,
                  const ax25_addr_t *digi, int ndigi,
                  const char *info, int infolen);

/* HDLC-encode `frame` (len bytes): `flags` opening 0x7E flags, bit-stuffed
 * body, one closing flag; then NRZI. `tones` gets one byte per transmitted
 * bit-cell: 0 = 2200 Hz (space), 1 = 1200 Hz (mark). Returns the bit count
 * (<= AX25_MAX_BITS) or 0 on overflow. */
int ax25_hdlc_nrzi(const uint8_t *frame, int len, int flags, uint8_t *tones);

/* CRC-16/X.25 (poly 0x1021 reflected, init 0xFFFF, xorout 0xFFFF). */
uint16_t ax25_fcs(const uint8_t *data, int len);

#endif /* APP_AX25_H */
