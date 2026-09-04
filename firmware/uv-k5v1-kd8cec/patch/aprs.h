/* APRS tracker for the UV-K5 (KD8CEC base), Sarsat_UV-K1-5_RP2040 project.
 *
 * The UV-K5 builds the AX.25/APRS frame and transmits it itself: Bell-202
 * AFSK is generated with the BK4819 tone generator (REG_71 switched between
 * 1200 and 2200 Hz at 1200 baud) and the normal firmware TX path keys the PA.
 * No mic wiring / no C-Board involvement for TX -- same model as KD8CEC.
 *
 * RX (packet decode) is done by the RP2040 C-Board; decoded text comes back
 * over the 0x06Dx serial commands and is shown on this screen.
 */
#ifndef APP_APRS_H
#define APP_APRS_H

#ifdef ENABLE_APRS

#include <stdbool.h>
#include <stdint.h>

/* radio <-> RP2040 APRS commands (see docs/protocol.md) */
#define APRS_CMD_CONFIG   0x06D0u   /* radio -> RP2040: push my call/path/... */
#define APRS_CMD_RXTEXT   0x06D2u   /* RP2040 -> radio: {line:u8, ascii[...]}  */
#define APRS_CMD_RXINFO   0x06D3u   /* RP2040 -> radio: structured decode      */
#define APRS_CMD_GPS      0x06D5u   /* RP2040 -> radio: GPS fix (flags,lat,lon,
                                     * speed_kmh,course,alt,sats)              */

enum { APRS_PATH_NONE, APRS_PATH_W1, APRS_PATH_W2, APRS_PATH_W1W2, APRS_PATH_N };

/* exactly 40 bytes (5 x EEPROM 8-byte pages) */
typedef struct {
    uint8_t  magic;        /* 0xA5 when valid                                */
    char     call[7];      /* NUL-padded, <= 6, A-Z 0-9                       */
    uint8_t  ssid;         /* 0..15                                          */
    uint8_t  path;         /* APRS_PATH_*                                     */
    char     sym_table;    /* '/' or '\\' or an overlay char                 */
    char     sym_code;     /* APRS symbol code                               */
    uint16_t interval_s;   /* auto-beacon period (s), 0 = off                */
    uint8_t  popup_s;      /* auto RX-popup: 0 = off, else 5 / 10 / 20 s     */
    uint8_t  af_gain;      /* C-Board AF level: 0 (or >78, erased) = auto /
                            * stock; 1..78 = fixed, a combined slider over the
                            * BK4819 REG_48 DAC gain + AF Rx Gain-2 (1 ~ -56 dB
                            * .. 78 ~ stock). Volume pot to max, tune on the
                            * SARSAT level screen against the rms bar.        */
    int32_t  lat_e5;       /* latitude  * 1e5  (deg, + = N)                  */
    int32_t  lon_e5;       /* longitude * 1e5  (deg, + = E)                  */
    uint8_t  opts;         /* bit0     : on the APRS band, light the backlight
                            *            ONLY on a decoded packet (not on the
                            *            channel's between-packet noise);
                            * bits 2:1 : APRS-band squelch mode --
                            *            0 stock, 1 fast (min BK4819 open delay
                            *            so the first AX.25 flags survive);
                            * bit3     : beacon position source --
                            *            0 manual (lat_e5/lon_e5), 1 GPS
                            *            (from the C-Board's NMEA input).      */
    char     comment[15];
} aprs_cfg_t;

#define APRS_OPT_BL_DECODE  0x01u
#define APRS_OPT_SQL_SHIFT  1
#define APRS_OPT_SQL_MASK   0x06u   /* 0 = stock, 1 = fast */
#define APRS_OPT_GPS        0x08u   /* beacon from the GPS fix, not lat/lon    */

extern aprs_cfg_t gAprsCfg;
extern bool       gAprsShowRequest;   /* set on an RX packet; app.c opens F+5 */

void APRS_Init(void);                 /* load config from EEPROM             */
void APRS_Beacon(void);               /* build + transmit one position frame */
void APRS_TimeSlice(void);            /* call ~1/s from the main loop        */
void APRS_HandleUART(uint16_t id, const uint8_t *data, uint16_t size);
void APP_RunAprs(void);               /* blocking config / status screen     */
void APRS_ApplyAfGain(void);          /* push gAprsCfg.af_gain to BK4819 REG_48 */
void APRS_SaveConfig(void);           /* persist gAprsCfg to EEPROM          */
bool APRS_QuietBacklight(void);       /* true => suppress the RX-squelch backlight */
void APRS_ApplySquelch(void);         /* APRS-band fast-squelch tweak (from tick) */
bool APRS_KeepAwake(void);            /* true => block battery-save (RX on 144-148) */
void APRS_MyPosition(int32_t *lat_e5, int32_t *lon_e5);  /* GPS fix or manual */
bool APRS_GpsFixValid(void);          /* a fresh GPS fix is available          */
uint8_t APRS_GpsState(void);          /* 0 no module / 1 searching / 2 locked  */

#endif /* ENABLE_APRS */
#endif /* APP_APRS_H */
