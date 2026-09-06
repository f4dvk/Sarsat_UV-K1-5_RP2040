/*
 * decoder_config.h — build-time configuration for the RP2040 SARSAT decoder.
 *
 * Target hardware: KD8CEC's "C-Board (DSP-Board) for UV-K5", *Basic Version*
 *   www.hamskey.com/2024/03/c-board-for-uv-k5.html
 *   - board            : RP2040-Zero (or a single-sided clone)
 *   - RX audio in       : radio speaker output (FM discriminator, flat AF) ->
 *                         0.1uF series cap -> node clamped by two 1N4148
 *                         (to GND and to 3V3) -> GP26
 *   - serial to radio   : GP0 = UART0 TX -> radio serial RX (2.5mm jack)
 *                         GP1 = UART0 RX <- radio serial TX (3.5mm jack ring)
 *   - power             : radio's ~3.3 V rail feeds the RP2040-Zero 3V3 pad
 *                         directly (mic-jack VCC). No on-board regulator used.
 *   - no bias resistor  : the AC-coupled input floats; DC is removed in
 *                         software (window_to_audio()).
 *
 * Verified by static analysis of KD8CEC's stock C-Board image
 * (cboard_v032.uf2, "C-BOARD FOR UV-K5/CEC4 V0.32"):
 *   - UART0 on GP0/GP1 (GPIO func 2)                  = radio link
 *     (stock firmware: 57600 8N1; we use 38400, see below)
 *   - UART1 on GP4/GP5, baud 9600                     = GPS (NMEA), unused here
 *   - GP26 / ADC0, DMA-fed                            = audio in
 *   - GP2 / GP15 / GP22 / GP25 / GP27                 = misc I/O (buttons / LED
 *     / PTT for the monitoring+GPS variants), unused for SARSAT RX
 *   KD8CEC's own C-Board <-> CEC-firmware protocol is proprietary; we run our
 *   own 0x06Cx protocol on the same wires (see docs/protocol.md).
 *
 * Electrical caveats from KD8CEC's article, relevant here:
 *   - UV-K5 speaker audio approaches ~8 V at max volume; the RP2040 ADC is
 *     0..3.3 V. Keep the radio volume low. The 1N4148 pair is the only
 *     over-voltage protection.
 *   - Some UV-K5 units brown out the C-Board (insufficient jack current) — see
 *     the article's "point A / point B" resistor mod.
 *
 * SARSAT is RX-only: the C-Board's mic-audio (PWM) and PTT paths are unused.
 *
 * Board core is selected with -DPICO_BOARD=pico / pico2 at CMake time.
 */
#ifndef SARSAT_DECODER_CONFIG_H
#define SARSAT_DECODER_CONFIG_H

/* ---- audio input (C-Board: SPK -> 0.1uF -> 1N4148 clamp -> GP26) ------- */
/*
 * IMPORTANT: the C-Board "Basic Version" has NO bias resistor on this node, so
 * the 0.1uF-coupled input floats near a rail and the audio gets half-wave
 * clamped by the 1N4148 to GND. Symptom in the log: idle ADC ~10-40 (not
 * ~2048) and every burst reads adc[0..4095]. Add a divider from 3V3 and GND to
 * the GP26 node (2x 100k-220k, or 1M to VREF/2) so it sits at mid-scale --
 * same fix as benshi-esp32-sim's Rb1/Rb2. Nothing downstream works without it.
 */
#define CFG_ADC_GPIO          26      /* ADC0 = the audio node                  */
#define CFG_ADC_CHANNEL       0
/* The CEC C-Board ties GP26/GP27/GP28 (ADC0/1/2) together onto the audio node.
 * We only sample ADC0, but the other two pads must be put in analog mode as
 * well (adc_gpio_init: pulls off + digital input buffer off) or their logic
 * input buffers load the node and draw shoot-through current as the audio
 * swings through the ~1.65 V logic threshold -> DC shift and distortion.
 * Bit i set here = also adc_gpio_init GP(26+i). */
#define CFG_ADC_SHORTED_MASK  0x07u    /* GP26 | GP27 | GP28                     */
#define CFG_SAMPLE_RATE_HZ    16000   /* 12000..48000; 16k -> 40 samples/bit   */
#define CFG_ADC_DC_CENTER     2048    /* 12-bit mid-scale; auto-tracked at run */
#define CFG_AUDIO_GAIN_SHIFT  3       /* (raw - dc) << shift -> slicer domain   */

/* ---- burst capture --------------------------------------------------- */
#define CFG_WINDOW_MS        1100     /* >= 2x a full FGB burst (~520 ms) so a  */
                                      /* whole frame always fits whatever its   */
                                      /* phase in the window                    */
#define CFG_BURST_PEAK_ON     3000    /* gained |sample| for the ABSOLUTE arm    */
                                      /* path (kept as a fallback). The main    */
                                      /* detector is now relative, see below.   */
#define CFG_DECODE_MAX_RMS    4000    /* absolute ceiling: never slice a window  */
                                      /* whose RMS is above this (it is hiss or  */
                                      /* clipping, not a captured carrier).     */
#define CFG_BURST_QUIET_PCT   60      /* RELATIVE burst detector. A 406 MHz FGB  */
                                      /* burst CAPTURES the FM RX, so its window */
                                      /* RMS collapses to well below the tracked */
                                      /* no-carrier hiss floor -- empirically    */
                                      /* 6-12 dB down (x2..x4). Slice a window   */
                                      /* when its RMS is below this percentage   */
                                      /* of the floor. This is independent of    */
                                      /* the absolute AF gain: tune the radio    */
                                      /* level screen to the *hiss* ("OK") and   */
                                      /* the quieter beacon still decodes, so    */
                                      /* the gain need not be pushed hot (which  */
                                      /* would clip a strong 2 m APRS packet on  */
                                      /* the shared audio tap). Raise toward 80  */
                                      /* if bursts are missed, lower toward 40   */
                                      /* if hiss slips through. The [lvl]/[burst]*/
                                      /* logs print rms as a %% of floorRMS.     */
#define CFG_REARM_MS          2000    /* min gap between two decode attempts    */
#define CFG_DEDUP_S          120      /* same hex ID within N s -> terse log    */

/* ---- audio conditioning ------------------------------------------- */
#define CFG_AUDIO_AGC         1       /* 1 = normalise each decoded window to    */
                                      /*     ~CFG_AGC_TARGET RMS before the      */
                                      /*     slicer (the quieted beacon audio is */
                                      /*     weak). 0 = raw.                     */
#define CFG_AGC_TARGET        6000

/* ---- serial link to the radio (C-Board: GP0/GP1 = UART0) ----------- */
#define CFG_RADIO_UART        uart0
#define CFG_RADIO_UART_TX_GPIO 0      /* -> radio serial RX (mic / 2.5mm jack)  */
#define CFG_RADIO_UART_RX_GPIO 1      /* <- radio serial TX (3.5mm jack ring)   */
#define CFG_RADIO_UART_BAUD   38400   /* project choice: the UV-K1 and UV-K5    */
                                      /* SARSAT firmware patches use 38400 too  */
                                      /* (matches egzumer/F4HWN + benshi tools).*/
                                      /* KD8CEC's stock C-Board uses 57600, but */
                                      /* we replace both firmware ends.         */

/* ---- SARSAT application command IDs (radio firmware, Phase 3/4) ----- */
#define CMD_SARSAT_CLEAR     0x06C0   /* no payload                              */
#define CMD_SARSAT_TEXT      0x06C1   /* {line:u8, invert:u8, ascii[...]}       */
#define CMD_SARSAT_BEACON    0x06C2   /* packed struct, see docs/protocol.md    */
#define CMD_SARSAT_LEVEL     0x06C3   /* {peak:u16, rms:u16, dc:u16, clip:u8,   */
                                      /* adc_min:u16, adc_max:u16, verdict:u8}  */
                                      /* LE. Feeds the radio level screen. No   */
                                      /* ACK (sent ~1/s).                       */
#define CMD_SARSAT_HELLO     0x06CF   /* {proto_ver:u8} keepalive               */

/* ---- APRS (RP2040 decodes 144.8 MHz packets, pushes text to the radio) --- */
#define CMD_APRS_CONFIG      0x06D0   /* radio -> RP2040: {call[6], ssid:u8,    */
                                      /*  path:u8, sym_table:u8, sym_code:u8,   */
                                      /*  digi_level:u8, flags:u8}.             */
                                      /*  digi_level: 0 off, 1 WIDE1-N, 2 also  */
                                      /*  WIDE2-N, 3 also WIDE3-N (cumulative). */
                                      /*  flags bit0 = KISS TNC mode (USB-CDC   */
                                      /*  becomes a binary KISS stream; the     */
                                      /*  RP2040 stops decoding-for-display and */
                                      /*  digipeating -- see kiss.h).           */
#define CMD_APRS_RXTEXT      0x06D2   /* RP2040 -> radio: {line:u8, ascii[...]} */
                                      /* line 0xFF = clear the RX view          */
#define CMD_APRS_RXINFO      0x06D3   /* RP2040 -> radio: structured decode     */
                                      /* (kind, symbol, lat/lon, cse/spd/alt,   */
                                      /*  dist/bearing, src, name, text)        */
#define CMD_APRS_GPS         0x06D5   /* RP2040 -> radio: GPS fix               */
                                      /* {flags:u8, lat_e5:i32, lon_e5:i32,     */
                                      /*  speed_kmh:u16, course:u16, alt:i16,   */
                                      /*  sats:u8}. flags bit0 = fix valid.     */
#define CMD_APRS_DIGI        0x06D6   /* RP2040 -> radio: raw AX.25 frame to    */
                                      /* digipeat -- dst[7] src[7] digi[7]*n    */
                                      /* ctrl pid info, NO FCS (the radio       */
                                      /* recomputes it and keys up on channel   */
                                      /* 170, same as its own beacon). Sent     */
                                      /* only when APRS_CONFIG's digi_level     */
                                      /* allowed repeating this frame (see      */
                                      /* aprs_digi.h) and it is not a recent    */
                                      /* duplicate -- OR, in KISS mode, for     */
                                      /* every frame the host asks to send.     */

/* ---- GPS (NMEA in on UART1, C-Board GPS header GP4/GP5) --------------- */
#define CFG_GPS_ENABLE        1
#define CFG_GPS_UART          uart1
#define CFG_GPS_UART_TX_GPIO  4       /* -> GPS RX (unused; module needs no TX) */
#define CFG_GPS_UART_RX_GPIO  5       /* <- GPS TX (NMEA)                       */
#define CFG_GPS_BAUD          9600
#define CFG_GPS_TX_MS         3000    /* push the fix to the radio this often   */
/* mode auto-selects from the radio's reported RX frequency (HELLO reply):     */
#define CFG_APRS_BAND_LO_10HZ  14400000u  /* 144.0 MHz : >= this -> APRS mode   */
#define CFG_APRS_BAND_HI_10HZ  14800000u  /* 148.0 MHz : <= this -> APRS mode   */

#define SARSAT_LVL_OK        0
#define SARSAT_LVL_CLIP      1
#define SARSAT_LVL_BIAS      2
#define SARSAT_LVL_HOT       3
#define SARSAT_LVL_LOW       4

#define SARSAT_LINK_PROTO_VER 1

/* ---- debug logging (USB-CDC console, 115200-agnostic) --------------- */
#define SARSAT_LOG           1       /* 0 = silent                             */
#define CFG_LEVEL_LOG_MS     3000    /* idle "audio level" report period (ms)  */
#define CFG_TX_HEXDUMP       0       /* 1 = also hex-dump every frame sent     */
#define CFG_APRS_RX_DIAG     1       /* 1 = provisional per-frame APRS RX log:  */
                                     /*     every HDLC candidate (OK / FIXED /  */
                                     /*     FCS-BAD / SHORT / DUP) with the     */
                                     /*     source call + a hex head, plus a    */
                                     /*     carrier/level line. Set 0 once the  */
                                     /*     decode issue is understood.         */

#endif /* SARSAT_DECODER_CONFIG_H */
