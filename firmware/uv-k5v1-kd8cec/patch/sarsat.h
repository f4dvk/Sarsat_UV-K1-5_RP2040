/* SARSAT screen for the Sarsat_UV-K1-5_RP2040 project.
 *
 * The RP2040 C-Board decodes COSPAS-SARSAT 406 MHz 1st-generation beacons from
 * the radio's FM-RAW audio and pushes pre-formatted text lines here over the
 * standard Quansheng UART frame (0x06Cx commands, 38400 8N1). This module just
 * stores and displays them.
 *
 * Protocol: Sarsat_UV-K1-5_RP2040/docs/protocol.md
 */
#ifndef APP_SARSAT_H
#define APP_SARSAT_H

#ifdef ENABLE_SARSAT

#include <stdbool.h>
#include <stdint.h>

#define SARSAT_LINES        16          /* stored lines (scrollable)            */
#define SARSAT_LINE_CHARS   18          /* glyphs (buffer is +1 for the NUL).
                                         * HARD LIMIT: UI_PrintStringSmall* does
                                         * NOT clip -- at Start=2 the 6px/7px
                                         * font overruns gFrameBuffer[row] past
                                         * 18 chars, corrupting the next row and
                                         * (on the bottom row) gEeprom.          */
#define SARSAT_VIS_ROWS     6           /* scrollable rows 1..6; the LCD content
                                        * area is gFrameBuffer[0..6] = 7 rows,
                                        * row 0 is the header                  */

#define SARSAT_CMD_CLEAR    0x06C0u     /* no payload                          */
#define SARSAT_CMD_TEXT     0x06C1u     /* {line:u8, invert:u8, ascii[0..20]}  */
#define SARSAT_CMD_BEACON   0x06C2u     /* packed struct (not used yet)         */
#define SARSAT_CMD_LEVEL    0x06C3u     /* {peak:u16,rms:u16,dc:u16,clip:u8,    */
                                        /*  adcmin:u16,adcmax:u16,verdict:u8}   */
#define SARSAT_CMD_HELLO    0x06CFu     /* {proto_ver:u8}                       */
#define SARSAT_PROTO_VER    1u

/* Phase 2: replies use ID = command | 0x8000.
 *  0x86C0/0x86C1/0x86C2 : {status:u8}  (0 = OK)
 *  0x86CF (HELLO reply) : {vfo:u8, modulation:u8, rx_freq:u32 LE 10Hz,
 *                          screen_open:u8, proto:u8} */

/* set true when a fresh text line arrived; app.c opens the screen */
extern bool gSarsatShowRequest;

/* dispatched from UART_HandleCommand() for the IDs above.
 * data = payload after the 4-byte inner header, size = UART_Command.Header.Size */
void SARSAT_HandleUART(uint16_t id, const uint8_t *data, uint16_t size);

/* blocking full-screen viewer. Radio -> selected VFO, FM discriminator + flat
 * AF, squelch forced open. UP/DOWN scroll, EXIT closes. */
void APP_RunSarsat(void);

#endif /* ENABLE_SARSAT */
#endif /* APP_SARSAT_H */
