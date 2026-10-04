#ifndef APP_SSTV_RX_H
#define APP_SSTV_RX_H
#include <stdbool.h>
#include <stdint.h>

/* No register/hardware setup needed: SSTV RX just tells the RP2040 (via
 * CMD_SARSAT_HELLO's screen_state byte, see decoder_config.h) to switch its
 * own ADC capture to the SSTV demod -- same model as the old IMGFSK RX
 * toggle it replaces. */
void SSTV_ToggleRx(void);
bool SSTV_RxActive(void);

/* ⚠️ (2026-10-03) live SSTV RX display -- see rp2040/src/decoder_config.h's
 * CMD_SSTV_RX_CLEAR/CMD_SSTV_RX_ROW comment for the full design (why 128x56
 * not 128x64, why the RP2040 does the resampling/dithering, wire format).
 * These constants MUST match decoder_config.h exactly -- no shared header
 * between the two projects, same situation as every other 0x06xx command
 * already in this file's history (aprs.h, sarsat.h...). */
#define SSTV_CMD_RX_CLEAR      0x06E4u    /* RP2040 -> radio: {} (no payload) */
#define SSTV_CMD_RX_ROW        0x06E5u    /* RP2040 -> radio: {row:u8, bits[16]} */
#define SSTV_CMD_CLK_START     0x06EBu    /* RP2040 -> radio: {duration_us:u32} -- debut mesure horloge (sstv_tx.c) */
#define SSTV_CMD_CLK_END       0x06ECu    /* RP2040 -> radio: {} -- fin mesure horloge (sstv_tx.c) */
#define SSTV_CMD_TRIM          0x06E9u    /* RP2040 -> radio: {trim_ppm:i16} -- correction horloge (sstv_tx.c) */
#define SSTV_RX_SCREEN_WIDTH   128u
#define SSTV_RX_SCREEN_HEIGHT  56u        /* = gFrameBuffer's FRAME_LINES (7) * 8 px */
#define SSTV_RX_ROW_BYTES      (SSTV_RX_SCREEN_WIDTH / 8u)

void SSTV_HandleUART(uint16_t id, const uint8_t *data, uint16_t size);
void APP_RunSstvRx(void);
void APP_RunSstvCal(void);

/* RP2040 -> radio (2026-10-04) : resultat de la mesure d'horloge, {trim_ppm:i16}
 * -- proposition affichee par l'ecran SSTVCal. */
#define SSTV_CMD_CLK_RESULT    0x06EFu

#endif
