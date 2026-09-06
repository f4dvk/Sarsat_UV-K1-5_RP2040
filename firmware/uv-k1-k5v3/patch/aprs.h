/* APRS tracker for the UV-K1 / UV-K5 V3 (F4HWN base), Sarsat_UV-K1-5_RP2040
 * project. Port of the UV-K5 V1 (KD8CEC) module -- see that project's
 * patch/aprs.h for the original design notes; differences are called out
 * inline here.
 *
 * The radio builds the AX.25/APRS frame and transmits it itself: Bell-202
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
#define APRS_CMD_CONFIG   0x06D0u   /* radio -> RP2040: {call[6],ssid,path,
                                     * sym_table,sym_code,digi_level}          */
#define APRS_CMD_RXTEXT   0x06D2u   /* RP2040 -> radio: {line:u8, ascii[...]}  */
#define APRS_CMD_RXINFO   0x06D3u   /* RP2040 -> radio: structured decode      */
#define APRS_CMD_GPS      0x06D5u   /* RP2040 -> radio: GPS fix (flags,lat,lon,
                                     * speed_kmh,course,alt,sats)              */
#define APRS_CMD_DIGI     0x06D6u   /* RP2040 -> radio: raw AX.25 frame to
                                     * digipeat (addr..ctrl..pid..info, no
                                     * FCS -- we append it and key up on
                                     * channel 170, same as our own beacon)    */

enum { APRS_PATH_NONE, APRS_PATH_W1, APRS_PATH_W2, APRS_PATH_W1W2, APRS_PATH_N };

/* Same fixed MR channel (170, 1-based / 169 0-based) and convention as the
 * V1 port and KD8CEC's own C-Board project -- see app/afgain.h for why a
 * fixed slot beats the currently-tuned VFO. This firmware's MR range goes up
 * to MR_CHANNEL_LAST = MR_CHANNELS_MAX - 1 = 1023, so 169 is nowhere near a
 * boundary; no channel-count cap was needed here (unlike the V1 port, whose
 * native 200-channel range got capped to 170 on request). */
#define APRS_TX_CHANNEL   169u

/* 56 bytes (7 x EEPROM 8-byte pages) -- same layout as the V1 port, except
 * af_gain: on this port the C-Board AF level override is the shared
 * app/afgain.h module (gAfGain / AFGAIN_Apply()), not a field of this struct,
 * since the SARSAT screen already needs it independently of ENABLE_APRS.
 * `reserved` keeps the struct's size and EEPROM page count unchanged. */
typedef struct {
    uint8_t  magic;        /* 0xA5 when valid                                */
    char     call[7];      /* NUL-padded, <= 6, A-Z 0-9                       */
    uint8_t  ssid;         /* 0..15                                          */
    uint8_t  path;         /* APRS_PATH_*                                     */
    char     sym_table;    /* '/' or '\\' or an overlay char                 */
    char     sym_code;     /* APRS symbol code                               */
    uint16_t interval_s;   /* auto-beacon period (s), 0 = off                */
    uint8_t  popup_s;      /* auto RX-popup: 0 = off, else 5 / 10 / 20 s     */
    uint8_t  reserved;     /* was af_gain on the V1 port; unused here -- see
                            * app/afgain.h (gAfGain) instead                 */
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
                            *            (from the C-Board's NMEA input);
                            * bits 5:4 : digipeat level -- 0 off, 1 repeat
                            *            WIDE1-N, 2 also WIDE2-N, 3 also
                            *            WIDE3-N (cumulative). Decided and
                            *            applied by the RP2040 (aprs_digi.h,
                            *            it already parses the AX.25 path);
                            *            pushed here via APRS_PushConfig()
                            *            so it can act on it;
                            * bit6     : KISS TNC mode -- the RP2040 becomes a
                            *            dumb Bell-202 modem on its USB, the
                            *            radio disables its own tracker/popup
                            *            and just relays frames.              */
    char     comment[15];
    char     msg_to[10];   /* fixed recipient of the "121 MHz beacon report"
                            * message (APRS screen -> "Send report"): an APRS
                            * addressee, <= 9 chars, e.g. "F4DVK-7". Empty =
                            * feature unavailable (like the NOCALL beacon guard) */
    uint8_t  _rsv[6];      /* pad to 56 B (7 x 8-byte EEPROM pages) so
                            * APRS_Save()'s 8-byte write loop stays exact      */
} aprs_cfg_t;

#define APRS_OPT_BL_DECODE  0x01u
#define APRS_OPT_SQL_SHIFT  1
#define APRS_OPT_SQL_MASK   0x06u   /* 0 = stock, 1 = fast */
#define APRS_OPT_GPS        0x08u   /* beacon from the GPS fix, not lat/lon    */
#define APRS_OPT_DIGI_SHIFT 4
#define APRS_OPT_DIGI_MASK  0x30u   /* 0 off, 1 WIDE1, 2 +WIDE2, 3 +WIDE3      */
#define APRS_OPT_KISS       0x40u   /* KISS TNC mode (RP2040 is the modem)     */

extern aprs_cfg_t gAprsCfg;
extern bool       gAprsShowRequest;   /* set on an RX packet; app.c opens the screen */

void APRS_Init(void);                 /* load config from EEPROM             */
void APRS_Beacon(void);               /* build + transmit one position frame */
void APRS_TimeSlice(void);            /* call every ~10 ms from the main tick,
                                       * unconditionally (screen open or not):
                                       * auto-beacon, fast-squelch reassert,
                                       * "light on frame" dim timer, GPS icon
                                       * blink -- same as the V1 port.       */
void APRS_HandleUART(uint16_t id, const uint8_t *data, uint16_t size);
void APP_RunAprs(void);               /* blocking config / status screen     */
void APRS_SaveConfig(void);           /* persist gAprsCfg to EEPROM          */
bool APRS_QuietBacklight(void);       /* true => suppress the RX-squelch backlight */
void APRS_ApplySquelch(void);         /* APRS-band fast-squelch tweak (from tick) */
bool APRS_KeepAwake(void);            /* true => block battery-save (RX on 144-148) */
void APRS_MyPosition(int32_t *lat_e5, int32_t *lon_e5);  /* GPS fix or manual */
bool APRS_GpsFixValid(void);          /* a fresh GPS fix is available          */
uint8_t APRS_GpsState(void);          /* 0 no module / 1 searching / 2 locked  */

#endif /* ENABLE_APRS */
#endif /* APP_APRS_H */
