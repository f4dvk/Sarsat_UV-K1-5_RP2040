/*
 * quansheng_frame.h — encoder for the Quansheng UV-K5 / UV-K5 V3 / UV-K1 serial
 * frame (egzumer / F4HWN firmware, App/app/uart.c).
 *
 *   AB CD | size:u16 LE | <inner> | crc:u16 LE | DC BA
 *   inner = [ID:u16 LE][data-size:u16 LE][data...]
 *   inner + crc are XOR-masked with the 16-byte obfuscation table.
 *
 * Ported from benshi-esp32-sim/src/UvK5Link.h (sendFrame + crc16Xmodem + kObf)
 * to plain C. TX only for now — the RP2040 pushes SARSAT text to the radio and
 * does not need to parse replies in Phase 1.
 */
#ifndef SARSAT_QUANSHENG_FRAME_H
#define SARSAT_QUANSHENG_FRAME_H

#include <stddef.h>
#include <stdint.h>

/* Serialize one frame into `out` (capacity `out_cap`).
 * `data`/`data_len` is the payload after the 4-byte inner header (<= 240).
 * Returns the number of bytes written, or 0 on error. */
size_t quansheng_frame_build(uint16_t cmd_id, const uint8_t *data, size_t data_len,
                             uint8_t *out, size_t out_cap);

uint16_t quansheng_crc16_xmodem(const uint8_t *p, size_t n);

/* ---- streaming receiver (for radio -> RP2040 replies) ------------------ */

typedef struct {
    uint8_t  buf[300];
    uint16_t len;
    uint8_t  payload[260];   /* stable copy of the last decoded payload */
} qframe_rx_t;

/* Feed one received byte. Returns 1 and fills *id / *data / *data_len when a
 * complete, well-formed frame has been assembled (payload de-obfuscated);
 * 0 otherwise. `data` points into rx->buf and is valid until the next call.
 * Reply frames carry no usable CRC, so none is checked. */
int quansheng_frame_feed(qframe_rx_t *rx, uint8_t byte,
                         uint16_t *id, const uint8_t **data, uint16_t *data_len);

#endif /* SARSAT_QUANSHENG_FRAME_H */
