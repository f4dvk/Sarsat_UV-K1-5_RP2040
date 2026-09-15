/* Radiosonde screen for the Sarsat_UV-K1-5_RP2040 project. See the V3 port's
 * app/sonde.h (firmware/uv-k1-k5v3/patch/sonde.h) for the full design note
 * -- this header is otherwise identical, only the .c differs (SendReply()'s
 * signature and the bold-text helper, same small set of differences already
 * documented at the top of sarsat.c). */
#ifndef APP_SONDE_H
#define APP_SONDE_H

#ifdef ENABLE_SONDE

#include <stdbool.h>
#include <stdint.h>

#define SONDE_LINES        8
#define SONDE_LINE_CHARS   18
#define SONDE_VIS_ROWS     6

#define SONDE_CMD_CLEAR    0x06E0u      /* no payload                          */
#define SONDE_CMD_TEXT     0x06E1u      /* {line:u8, invert:u8, ascii[0..20]}  */

extern bool gSondeShowRequest;

void SONDE_HandleUART(uint16_t id, const uint8_t *data, uint16_t size);
void APP_RunSonde(void);
bool SONDE_ScreenOpen(void);

#endif /* ENABLE_SONDE */
#endif /* APP_SONDE_H */
