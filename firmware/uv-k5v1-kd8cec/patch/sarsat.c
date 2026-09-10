/* SARSAT screen — see app/sarsat.h. Part of the Sarsat_UV-K1-5_RP2040 project. */
#include "app/sarsat.h"

#ifdef ENABLE_SARSAT

#include <string.h>

#include "external/printf/printf.h"
#include "app/uart.h"
#include "app/app.h"
#include "driver/keyboard.h"
#include "driver/backlight.h"
#include "driver/st7565.h"
#include "app/chFrScanner.h"
#include "driver/system.h"
#include "driver/bk4819.h"
#include "ui/helper.h"
#include "ui/ui.h"
#include "radio.h"
#include "functions.h"
#include "settings.h"
#include "misc.h"
#ifdef ENABLE_APRS
	#include "app/aprs.h"   /* shared C-Board AF-gain setting (gAprsCfg.af_gain) */
#endif

static char     s_line[SARSAT_LINES][SARSAT_LINE_CHARS + 1];
static uint16_t s_invert;          /* one bit per line, from the RP2040    */
static uint8_t  s_nlines;          /* highest line index + 1 that was set  */
static uint8_t  s_scroll;          /* index of the first visible line      */
static uint8_t  s_proto;
static bool     s_dirty;
static bool     s_screen_open;     /* APP_RunSarsat() is running           */

static uint8_t  s_view;            /* 0 = beacon text, 1 = audio level    */
static struct {
	uint16_t peak, rms, dc, amin, amax;
	uint8_t  clip, verdict;
} s_lvl;

bool gSarsatShowRequest;

/* made non-static by patch/app_uart.c.diff */
extern void SendReply(void *pReply, uint16_t Size);

/* Phase 2: answer the RP2040. Reply ID = command | 0x8000, payload = data. */
static void SARSAT_Reply(uint16_t reply_id, const uint8_t *data, uint8_t dlen)
{
	uint8_t buf[4 + 12];
	buf[0] = reply_id & 0xFF; buf[1] = reply_id >> 8;
	buf[2] = dlen;            buf[3] = 0;
	if (data && dlen)
		memcpy(buf + 4, data, dlen);
	SendReply(buf, 4 + dlen);
}

static void SARSAT_ReplyStatus(void)
{
	/* read the RX frequency from the struct member directly (not via ->pRX,
	 * whose target depends on the FrequencyReverse flag) and sanity-clamp it */
	const VFO_Info_t *v = &gEeprom.VfoInfo[gEeprom.TX_VFO & 1];
	uint32_t f = v->freq_config_RX.Frequency;       /* 10 Hz units */
	if (f < 100000u || f > 135000000u)              /* < 1 MHz or > 1.35 GHz */
		f = 0;

	/* bytes 8..15 carry the operator's APRS position (from the F+5 config) so
	 * the C-Board can add distance / bearing to decoded packets. 0 = unset. */
	int32_t my_lat = 0, my_lon = 0;
#ifdef ENABLE_APRS
	APRS_MyPosition(&my_lat, &my_lon);   /* GPS fix if "Pos GPS", else manual */
#endif
	uint8_t p[16] = {
		(uint8_t)(gEeprom.TX_VFO & 1),
		(uint8_t)(s_screen_open ? MODULATION_FM : v->Modulation),
		(uint8_t)(f      ), (uint8_t)(f >>  8),
		(uint8_t)(f >> 16), (uint8_t)(f >> 24),
		(uint8_t)(s_screen_open ? (s_view == 1 ? 2 : 1) : 0),  /* 2 = level view */
		(uint8_t)SARSAT_PROTO_VER,
		(uint8_t)my_lat, (uint8_t)(my_lat >> 8),
		(uint8_t)(my_lat >> 16), (uint8_t)(my_lat >> 24),
		(uint8_t)my_lon, (uint8_t)(my_lon >> 8),
		(uint8_t)(my_lon >> 16), (uint8_t)(my_lon >> 24),
	};
	SARSAT_Reply(SARSAT_CMD_HELLO | 0x8000u, p, sizeof(p));
}

void SARSAT_HandleUART(uint16_t id, const uint8_t *data, uint16_t size)
{
	/* The C-Board's UART TX toggling the radio's mic/data line can register as
	 * a brief PTT on a TX-capable band (seen as a few-ms key-up right after an
	 * APRS decode on 144.8). Holding SerialConfigInProgress() for ~1 s over the
	 * text burst makes GENERIC_Key_PTT() treat any PTT as "released". Not set
	 * for HELLO so it never blocks the APRS beacon. */
	if (id == SARSAT_CMD_CLEAR || id == SARSAT_CMD_TEXT)
		gSerialConfigCountDown_500ms = 2;

	switch (id)
	{
		case SARSAT_CMD_CLEAR:
		{
			memset(s_line, 0, sizeof(s_line));
			s_invert  = 0;
			s_nlines  = 0;
			s_scroll  = 0;
			s_dirty   = true;
			const uint8_t ok = 0;
			SARSAT_Reply(SARSAT_CMD_CLEAR | 0x8000u, &ok, 1);
			break;
		}

		case SARSAT_CMD_TEXT:
		{
			if (size < 2)
				break;
			const uint8_t idx = data[0];
			const uint8_t inv = data[1];
			if (idx >= SARSAT_LINES) {
				const uint8_t err = 1;
				SARSAT_Reply(SARSAT_CMD_TEXT | 0x8000u, &err, 1);
				break;
			}
			uint16_t n = size - 2;
			if (n > SARSAT_LINE_CHARS)
				n = SARSAT_LINE_CHARS;
			memset(s_line[idx], 0, sizeof(s_line[idx]));
			memcpy(s_line[idx], data + 2, n);
			if (inv)
				s_invert |=  (1u << idx);
			else
				s_invert &= ~(1u << idx);
			if (idx + 1 > s_nlines)
				s_nlines = idx + 1;
			s_dirty            = true;
			gSarsatShowRequest = true;
			const uint8_t ok = 0;
			SARSAT_Reply(SARSAT_CMD_TEXT | 0x8000u, &ok, 1);
			break;
		}

		case SARSAT_CMD_LEVEL:
			if (size >= 12) {
				s_lvl.peak    = data[0]  | (data[1]  << 8);
				s_lvl.rms     = data[2]  | (data[3]  << 8);
				s_lvl.dc      = data[4]  | (data[5]  << 8);
				s_lvl.clip    = data[6];
				s_lvl.amin    = data[7]  | (data[8]  << 8);
				s_lvl.amax    = data[9]  | (data[10] << 8);
				s_lvl.verdict = data[11];
				if (s_view == 1)
					s_dirty = true;
			}
			break;   /* no reply: high-rate telemetry */

		case SARSAT_CMD_HELLO:
			if (size >= 1)
				s_proto = data[0];
			SARSAT_ReplyStatus();
			break;

		case SARSAT_CMD_BEACON:
		{
			const uint8_t ok = 0;
			SARSAT_Reply(SARSAT_CMD_BEACON | 0x8000u, &ok, 1);
#ifdef ENABLE_APRS
			/* packed LE, 29 bytes -- see docs/protocol.md:
			 *  0 frame_bits, 1 protocol, 2 is_test, 3 has_position,
			 *  4..5 country(u16), 6..9 lat_e5(i32), 10..13 lon_e5(i32),
			 *  14..28 hex_id[15]. Feeds the "Send SARSAT" APRS message. */
			if (size >= 29) {
				char id[16];
				memcpy(id, data + 14, 15);
				id[15] = 0;
				int32_t la = (int32_t)(data[6]  | (data[7]  << 8) |
				                      (data[8]  << 16) | (data[9]  << 24));
				int32_t lo = (int32_t)(data[10] | (data[11] << 8) |
				                      (data[12] << 16) | (data[13] << 24));
				APRS_NoteBeacon(id, la, lo,
				                (uint16_t)(data[4] | (data[5] << 8)),
				                data[3], data[2]);
			}
#endif
			break;
		}

		default:
			break;
	}
}

/* every string here is <= SARSAT_LINE_CHARS (18) glyphs -- see the note in
 * sarsat.h: UI_PrintStringSmall* does not clip and overruns the row buffer. */

/* ENABLE_SMALL_BOLD=0 (V1) removed the bold used to emphasise a line; replaced
 * by inverse video tight to the text (see aprs.h). Shared with the APRS screen
 * when both are built; a stand-alone fallback keeps SARSAT-only builds working. */
#ifdef ENABLE_APRS
#define SARSAT_Hilite(row, s)  APRS_HiliteText((row), 1, (s))
#else
static void SARSAT_Hilite(int row, const char *s)
{
	if (row < 0 || row > 6)
		return;
	unsigned x1 = 1 + (unsigned)strlen(s) * 7u + 1;
	if (x1 > 128) x1 = 128;
	for (unsigned i = 0; i < x1; i++)
		gFrameBuffer[row][i] ^= 0x7Fu;   /* 7 px font: keep bottom pixel -> 1 px gap */
}
#endif

static void SARSAT_DrawLevel(void)
{
	static const char *const verd[5] = {   /* all <= 18 glyphs, order = SARSAT_LVL_* */
		"OK reglage correct",
		"ECRETAGE reduire",
		"polariser ADC dc",
		"fort: APRS sature",
		"trop faible",
	};
	char s[SARSAT_LINE_CHARS + 1];
	unsigned f;

	UI_DisplayClear();
	UI_PrintStringSmallNormal("NIVEAU 5:txt UP/DN", 1, 0, 0);
	SARSAT_Hilite(0, "NIVEAU 5:txt UP/DN");

	sprintf(s, "rms%-6u pk%u", s_lvl.rms, s_lvl.peak);
	UI_PrintStringSmallNormal(s, 2, 0, 1);
	sprintf(s, "dc%-6u clip%u%%", s_lvl.dc, s_lvl.clip);
	UI_PrintStringSmallNormal(s, 2, 0, 2);
	sprintf(s, "adc %u-%u", s_lvl.amin, s_lvl.amax);
	UI_PrintStringSmallNormal(s, 2, 0, 3);

#ifdef ENABLE_APRS
	/* UP/DOWN here tune the BK4819 AF gain feeding the C-Board tap -- set the
	 * volume pot to max and adjust this against the rms bar. */
	if (gAprsCfg.af_gain < 1 || gAprsCfg.af_gain > 78)
		strcpy(s, "AF gain: auto");
	else
		sprintf(s, "AF gain: %u", gAprsCfg.af_gain);
	UI_PrintStringSmallNormal(s, 2, 0, 4);
#endif

	f = s_lvl.rms / 300;                 /* full bar ~ rms 4800; aim mid-scale */
	if (f > 16) f = 16;
	memset(s, '#', f);
	memset(s + f, '.', 16 - f);
	s[16] = 0;
	UI_PrintStringSmallNormal(s, 2, 0, 5);

	/* row 6 is the last usable content row (gFrameBuffer has only 7) */
	{
		const char *v = verd[s_lvl.verdict < 5 ? s_lvl.verdict : 0];
		UI_PrintStringSmallNormal(v, 1, 0, 6);
		SARSAT_Hilite(6, v);
	}
	ST7565_BlitFullScreen();
}

static void SARSAT_Draw(void)
{
	char hdr[SARSAT_LINE_CHARS + 1];

	if (s_view == 1) { SARSAT_DrawLevel(); return; }

	UI_DisplayClear();

	/* row 0: fixed header + scroll position (<= 18 glyphs) */
	if (s_nlines == 0)
		strcpy(hdr, "SARSAT 406 ...");
	else {
		unsigned last = s_scroll + SARSAT_VIS_ROWS;
		if (last > s_nlines) last = s_nlines;
		sprintf(hdr, "SARSAT %u-%u/%u%c%c",
		        (unsigned)(s_scroll + 1), last, (unsigned)s_nlines,
		        s_scroll > 0 ? '^' : ' ',
		        last < s_nlines ? 'v' : ' ');
	}
	UI_PrintStringSmallNormal(hdr, 1, 0, 0);
	SARSAT_Hilite(0, hdr);

	/* rows 1..SARSAT_VIS_ROWS: the visible slice of the line buffer */
	for (uint8_t r = 0; r < SARSAT_VIS_ROWS; r++)
	{
		const uint8_t li = s_scroll + r;
		if (li >= s_nlines || !s_line[li][0])
			continue;
		if (s_invert & (1u << li)) {
			UI_PrintStringSmallNormal(s_line[li], 1, 0, r + 1);
			SARSAT_Hilite(r + 1, s_line[li]);
		} else {
			UI_PrintStringSmallNormal(s_line[li], 2, 0, r + 1);
		}
	}

	ST7565_BlitFullScreen();
}

/* clamp s_scroll so the last line can reach the bottom row */
static void SARSAT_Scroll(int delta)
{
	int max = (int)s_nlines - SARSAT_VIS_ROWS;
	if (max < 0) max = 0;
	int v = (int)s_scroll + delta;
	if (v < 0)   v = 0;
	if (v > max) v = max;
	if (v != (int)s_scroll) { s_scroll = (uint8_t)v; s_dirty = true; }
}

bool SARSAT_ScreenOpen(void)
{
    return s_screen_open;
}

void APP_RunSarsat(void)
{
	/* Only take over the radio when it is idle on the main screen. If a menu
	 * or TX is up, leave the request pending and try again next tick -- opening
	 * this blocking screen over a menu left the display / VFO state corrupted
	 * (garbage screen, stuck in MR after a power cycle). */
	if (gScreenToDisplay != DISPLAY_MAIN || gCurrentFunction == FUNCTION_TRANSMIT ||
	    gScanStateDir != SCAN_OFF) {
		gSarsatShowRequest = true;
		return;
	}

	/* Receive on the VFO the user has selected (gEeprom.TX_VFO), whatever the
	 * dual-watch / cross-band state, and force the audio path open (the C-Board
	 * tap must always hear the RF, squelch or not). */
	const uint8_t s_saved_rx_vfo = gEeprom.RX_VFO;
	RADIO_SelectVfos();
	gRxVfo         = gTxVfo;
	gEeprom.RX_VFO = gEeprom.TX_VFO;
	RADIO_SetupRegisters(true);                 // tune BK4819 to that VFO
	APP_StartListening(FUNCTION_MONITOR);       // audio path on, squelch forced open

	/* AF profile for the C-Board decoder, done AFTER APP_StartListening (which
	 * resets the modulation to the VFO's own):
	 *   - MODULATION_FM : the FM DISCRIMINATOR. A carrier tuning error then
	 *     shows up as a slow DC drift (which the RP2040 slicer removes), not as
	 *     a constant beat tone. MODULATION_RAW / BASEBAND1 is an SSB-style raw
	 *     output and beats the strong 406 MHz carrier into a steady whistle -
	 *     that was the reported symptom.
	 *   - REG_2B bits 10/9/8 : drop RX de-emphasis / HPF300 / LPF3k so the
	 *     bi-phase-L transitions pass flat (the "EnterRaw" profile of the
	 *     BK4829 firmware). Remove this write if the extra hiss hurts more than
	 *     the flatness helps.
	 *   - WIDE IF so the beacon's sidebands are not clipped. */
	RADIO_SetModulation(MODULATION_FM);
	BK4819_SetFilterBandwidth(BK4819_FILTER_BW_WIDE, false);
	{
		uint16_t r2b = BK4819_ReadRegister(BK4819_REG_2B);
		r2b |= (1u << 10) | (1u << 9) | (1u << 8);
		BK4819_WriteRegister(BK4819_REG_2B, r2b);
	}
#ifdef ENABLE_APRS
	/* re-assert the C-Board AF gain: RADIO_SetupRegisters / APP_StartListening
	 * above just wrote REG_48 from gEeprom.VOLUME_GAIN/DAC_GAIN, which a menu
	 * calibration reload may have reset to stock -> wrong level on reopen.
	 * Resync "auto"'s stock-knob capture first -- see APRS_ResyncAfGainKnob(). */
	APRS_ResyncAfGainKnob();
	APRS_ApplyAfGain();
#endif

	gSarsatShowRequest = false;
	s_view             = 0;      /* always open on the decode, not the level view */
	s_dirty            = true;
	s_screen_open      = true;
	SARSAT_ReplyStatus();        /* let the C-Board know the screen state early */

	KEY_Code_t   prev_key = KEY_INVALID;
	uint16_t     held     = 0;
	bool         armed    = false;   /* ignore the launch key until released */
	bool         run      = true;
#ifdef ENABLE_APRS
	bool         gain_dirty = false; /* an unsaved AF-gain change on the level view */
#endif
	while (run)
	{
#ifdef ENABLE_UART
		/* drain: the RP2040 sends a burst of ~14 line frames per decode */
		while (UART_IsCommandAvailable())
			UART_HandleCommand();
#endif
		if (s_dirty)
		{
			s_dirty = false;
			SARSAT_Draw();
		}

		const KEY_Code_t key = KEYBOARD_Poll();
		if (key == KEY_INVALID) {
			armed = true;
			held  = 0;
		} else if (armed) {
			bool edge   = (key != prev_key);
			bool repeat = (key == prev_key) && (++held > 15) && (held % 3 == 0);
			if (edge || repeat) {
				switch (key) {
				/* UV-K5: UP/DOWN scroll the decode, or (level view) tune AF gain */
				case KEY_UP:
				case KEY_DOWN: {
					const int d = (key == KEY_UP) ? +1 : -1;
#ifdef ENABLE_APRS
					if (s_view == 1) {
						int v = (gAprsCfg.af_gain >= 1 && gAprsCfg.af_gain <= 78)
						        ? gAprsCfg.af_gain : 0;
						v += d;
						if (v < 0)  v = 0;
						if (v > 78) v = 78;
						gAprsCfg.af_gain = (uint8_t)v;
						APRS_ApplyAfGain();
						gain_dirty = true;
						s_dirty = true;
						break;
					}
#endif
					SARSAT_Scroll(-d);
					break;
				}
				case KEY_5:
#ifdef ENABLE_APRS
					if (s_view == 1 && gain_dirty) {
						APRS_SaveConfig(); gain_dirty = false;
					}
#endif
					s_view ^= 1; s_dirty = true;
					SARSAT_ReplyStatus();   /* tell the C-Board: fast level telemetry on/off */
					break;  /* text <-> level */
				case KEY_EXIT: run = false;       break;
				default: break;
				}
			}
		}
		prev_key = key;

		SYSTEM_DelayMs(20);
	}

	s_screen_open      = false;
	SARSAT_ReplyStatus();            /* screen closed -> C-Board back to full windows */
	gSarsatShowRequest = false;      /* a late 0x06C1 must not re-open at once  */
#ifdef ENABLE_APRS
	if (gain_dirty) APRS_SaveConfig();
#endif

	/* Restore normal operation. This screen can be entered from the 10 ms tick
	 * (auto-open on a decode), not only from a key, so it must NOT rely on the
	 * key-handler tail to finish the cleanup -- do it all here, synchronously:
	 *  - leave FUNCTION_MONITOR (APP_StartListening put us there; otherwise the
	 *    next key just "stops monitoring" and seems dead),
	 *  - restore gEeprom.RX_VFO (entry forced it = TX_VFO),
	 *  - re-read BOTH VFOs from EEPROM: RADIO_ConfigureChannel validates
	 *    gEeprom.ScreenChannel and repairs a stray value -- without this a
	 *    corrupted ScreenChannel showed as "M128, no name/freq" and stuck,
	 *    because the tick has no key-handler tail to run the reconfigure,
	 *  - refresh the backlight + auto-keypad-lock timers (frozen while blocked),
	 *  - switch the display back to MAIN now, not "on request". */
	gMonitor       = false;
	gEeprom.RX_VFO = s_saved_rx_vfo;
	FUNCTION_Select(FUNCTION_FOREGROUND);
	RADIO_ConfigureChannel(0, VFO_CONFIGURE);
	RADIO_ConfigureChannel(1, VFO_CONFIGURE);
	RADIO_SelectVfos();
	RADIO_SetupRegisters(true);

	BACKLIGHT_TurnOn();
	gKeyLockCountdown = 30;

	GUI_SelectNextDisplay(DISPLAY_MAIN);
	gUpdateStatus  = true;
	gUpdateDisplay = true;
}

#endif /* ENABLE_SARSAT */
