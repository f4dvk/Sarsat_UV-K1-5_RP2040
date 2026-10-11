/* Radiosonde screen for the Sarsat_UV-K1-5_RP2040 project.
 *
 * The RP2040 C-Board decodes radiosonde telemetry (RS41 full frame decode;
 * M10/M20 header detection only -- see rp2040/src/sonde_sync.h) from the
 * radio's FM-RAW audio and pushes pre-formatted text lines here, over the
 * same Quansheng UART frame shape as the SARSAT screen (see app/sarsat.h)
 * but its own command IDs (0x06Ex) and its own line buffer -- kept a
 * separate screen/mode rather than folded into SARSAT's because the RP2040
 * needs an explicit "open this screen" signal to know to switch its ADC to
 * the higher sample rate + streaming demod radiosondes need (see
 * decoder_config.h's CFG_SONDE_SAMPLE_RATE_HZ comment): SARSAT and
 * radiosondes share the same 400-406 MHz band, so unlike APRS the RX
 * frequency alone can't tell the RP2040 which decoder to run.
 *
 * Protocol: Sarsat_UV-K1-5_RP2040/docs/protocol.md
 */
#ifndef APP_SONDE_H
#define APP_SONDE_H

#ifdef ENABLE_SONDE

#include <stdbool.h>
#include <stdint.h>

#define SONDE_LINES        8            /* stored lines (scrollable)          */
#define SONDE_LINE_CHARS   18           /* glyphs, see sarsat.h's identical   */
                                        /* HARD LIMIT note -- UI_PrintStringSmall
                                         * does NOT clip at Start=2            */
#define SONDE_VIS_ROWS     6            /* scrollable rows 1..6, same layout   */
                                        /* budget as the SARSAT screen         */

#define SONDE_CMD_CLEAR    0x06E0u      /* no payload                          */
#define SONDE_CMD_TEXT     0x06E1u      /* {line:u8, invert:u8, ascii[0..20]}  */

/* Phase 2 replies use ID = command | 0x8000, {status:u8} (0 = OK) -- same
 * shape as SARSAT's, see sarsat.h. No SONDE_HELLO: the existing SARSAT_HELLO
 * cycle already carries the screen-open signal (byte 6 == 3, see
 * decoder_config.h) the RP2040 needs, piggy-backed via SONDE_ScreenOpen(). */

/* set true when a fresh text line arrives; app.c opens the screen */
extern bool gSondeShowRequest;

/* dispatched from UART_HandleCommand() for the IDs above.
 * data = payload after the 4-byte inner header, size = UART_Command.Header.Size */
void SONDE_HandleUART(uint16_t id, const uint8_t *data, uint16_t size);

/* blocking full-screen viewer. Radio -> selected VFO, squelch forced open.
 * Two selectable RX profiles (see sonde.c's SONDE_ApplyRxProfile()): DSC
 * (default, FM + flat/bypassed filters, matching the V1 port's own working
 * chain) or plain stock RAW (MODULATION_RAW, untouched). UP/DOWN scroll,
 * KEY_5 toggles DSC/RAW, EXIT closes. */
void APP_RunSonde(void);

/* true while APP_RunSonde()'s loop is running. Read by sarsat.c to fold into
 * the SARSAT_HELLO reply's screen-state byte (0/1/2 already used by SARSAT
 * itself, 3 = Sonde screen open). */
bool SONDE_ScreenOpen(void);

#endif /* ENABLE_SONDE */
#endif /* APP_SONDE_H */
