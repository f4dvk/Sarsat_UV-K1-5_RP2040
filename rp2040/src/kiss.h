/* Minimal KISS TNC framing (SLIP) for the APRS coprocessor.
 *
 * When the radio's APRS menu turns KISS on (APRS_OPT_KISS -> a flag byte in
 * CMD_APRS_CONFIG), the RP2040 stops being an APRS tracker/decoder-for-display
 * and becomes a dumb Bell-202 modem: raw AX.25 frames it demodulates go out
 * the USB-CDC as KISS data frames, and KISS data frames the host sends in are
 * forwarded to the radio (CMD_APRS_DIGI) which appends the FCS and keys up on
 * channel 170 -- exactly the digipeat TX path.
 *
 * Pure C, no pico deps -> host-testable (rp2040/test/host/test_kiss).
 * Part of the Sarsat_UV-K1-5_RP2040 project.
 */
#ifndef KISS_H
#define KISS_H

#include <stdint.h>
#include <stdbool.h>

#define KISS_FEND   0xC0
#define KISS_FESC   0xDB
#define KISS_TFEND  0xDC
#define KISS_TFESC  0xDD

#define KISS_MAX_FRAME  330   /* matches APRS_RX_MAX_FRAME */

/* Wrap `frame` (len bytes of raw AX.25, no FCS) as a KISS data frame
 * (port 0, command 0): FEND, 0x00, SLIP-escaped frame, FEND.
 * Returns the byte count written to `out`, or 0 if it would not fit `cap`. */
int kiss_encode(uint8_t *out, int cap, const uint8_t *frame, int len);

/* Streaming decoder. Feed one received byte at a time. On a complete KISS
 * *data* frame (command nibble 0), returns its payload length and leaves the
 * de-escaped AX.25 bytes in `k->frame`; returns 0 otherwise (mid-frame, a
 * non-data frame, or an over-long frame which is dropped). */
typedef struct {
    uint8_t frame[KISS_MAX_FRAME];
    int     len;
    bool    in_frame;   /* seen the opening FEND                        */
    bool    esc;        /* last byte was FESC                           */
    bool    have_cmd;   /* consumed the command byte of this frame      */
    bool    is_data;    /* command nibble was 0                         */
    bool    overflow;   /* frame exceeded KISS_MAX_FRAME -> drop it     */
} kiss_dec_t;

int kiss_decode_byte(kiss_dec_t *k, uint8_t b);

#endif /* KISS_H */
