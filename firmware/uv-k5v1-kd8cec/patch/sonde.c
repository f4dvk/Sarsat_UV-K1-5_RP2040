/* Radiosonde screen — see app/sonde.h. Part of the Sarsat_UV-K1-5_RP2040
 * project. Structurally a trimmed sibling of app/sarsat.c (same UART link
 * shape, same RF setup rationale -- radiosondes ride the same flat FM
 * discriminator audio in the same 400-406 MHz band as a SARSAT beacon, so
 * the register-level tuning already fought for on air there applies as-is)
 * with its own command IDs / line buffer and no level-tuning sub-view. */
#include "app/sonde.h"

#ifdef ENABLE_SONDE

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
	#include "app/aprs.h"   /* shared C-Board AF-gain setting + APRS_HiliteText */
#endif

static char     s_line[SONDE_LINES][SONDE_LINE_CHARS + 1];
static uint16_t s_invert;
static uint8_t  s_nlines;
static uint8_t  s_scroll;
static bool     s_dirty;
static bool     s_screen_open;     /* APP_RunSonde() is running              */

bool gSondeShowRequest;

/* made non-static by patch/app_uart.c.diff */
extern void SendReply(void *pReply, uint16_t Size);

static void SONDE_Reply(uint16_t reply_id, const uint8_t *data, uint8_t dlen)
{
	uint8_t buf[4 + 12];
	buf[0] = reply_id & 0xFF; buf[1] = reply_id >> 8;
	buf[2] = dlen;            buf[3] = 0;
	if (data && dlen)
		memcpy(buf + 4, data, dlen);
	SendReply(buf, 4 + dlen);
}

bool SONDE_ScreenOpen(void)
{
	return s_screen_open;
}

void SONDE_HandleUART(uint16_t id, const uint8_t *data, uint16_t size)
{
	if (id == SONDE_CMD_CLEAR || id == SONDE_CMD_TEXT)
		gSerialConfigCountDown_500ms = 2;

	switch (id)
	{
		case SONDE_CMD_CLEAR:
		{
			memset(s_line, 0, sizeof(s_line));
			s_invert = 0;
			s_nlines = 0;
			s_scroll = 0;
			s_dirty  = true;
			const uint8_t ok = 0;
			SONDE_Reply(SONDE_CMD_CLEAR | 0x8000u, &ok, 1);
			break;
		}

		case SONDE_CMD_TEXT:
		{
			if (size < 2)
				break;
			const uint8_t idx = data[0];
			const uint8_t inv = data[1];
			if (idx >= SONDE_LINES) {
				const uint8_t err = 1;
				SONDE_Reply(SONDE_CMD_TEXT | 0x8000u, &err, 1);
				break;
			}
			uint16_t n = size - 2;
			if (n > SONDE_LINE_CHARS)
				n = SONDE_LINE_CHARS;
			memset(s_line[idx], 0, sizeof(s_line[idx]));
			memcpy(s_line[idx], data + 2, n);
			if (inv)
				s_invert |=  (1u << idx);
			else
				s_invert &= ~(1u << idx);
			if (idx + 1 > s_nlines)
				s_nlines = idx + 1;
			s_dirty           = true;
			gSondeShowRequest = true;
			const uint8_t ok = 0;
			SONDE_Reply(SONDE_CMD_TEXT | 0x8000u, &ok, 1);
			break;
		}

		default:
			break;
	}
}

/* see sarsat.c's identical macro/fallback: ENABLE_SMALL_BOLD=0 on this
 * firmware, inverse video substitutes for a bold header. */
#ifdef ENABLE_APRS
#define SONDE_Hilite(row, s)  APRS_HiliteText((row), 1, (s))
#else
static void SONDE_Hilite(int row, const char *s)
{
	if (row < 0 || row > 6)
		return;
	unsigned x1 = 1 + (unsigned)strlen(s) * 7u + 1;
	if (x1 > 128) x1 = 128;
	for (unsigned i = 0; i < x1; i++)
		gFrameBuffer[row][i] ^= 0x7Fu;
}
#endif

static void SONDE_Draw(void)
{
	char hdr[SONDE_LINE_CHARS + 1];

	UI_DisplayClear();

	if (s_nlines == 0)
		strcpy(hdr, "SONDE ...");
	else {
		unsigned last = s_scroll + SONDE_VIS_ROWS;
		if (last > s_nlines) last = s_nlines;
		sprintf(hdr, "SONDE %u-%u/%u%c%c",
		        (unsigned)(s_scroll + 1), last, (unsigned)s_nlines,
		        s_scroll > 0 ? '^' : ' ',
		        last < s_nlines ? 'v' : ' ');
	}
	UI_PrintStringSmallNormal(hdr, 1, 0, 0);
	SONDE_Hilite(0, hdr);

	for (uint8_t r = 0; r < SONDE_VIS_ROWS; r++)
	{
		const uint8_t li = s_scroll + r;
		if (li >= s_nlines || !s_line[li][0])
			continue;
		if (s_invert & (1u << li)) {
			UI_PrintStringSmallNormal(s_line[li], 1, 0, r + 1);
			SONDE_Hilite(r + 1, s_line[li]);
		} else {
			UI_PrintStringSmallNormal(s_line[li], 2, 0, r + 1);
		}
	}

	ST7565_BlitFullScreen();
}

static void SONDE_Scroll(int delta)
{
	int max = (int)s_nlines - SONDE_VIS_ROWS;
	if (max < 0) max = 0;
	int v = (int)s_scroll + delta;
	if (v < 0)   v = 0;
	if (v > max) v = max;
	if (v != (int)s_scroll) { s_scroll = (uint8_t)v; s_dirty = true; }
}

void APP_RunSonde(void)
{
	if (gScreenToDisplay != DISPLAY_MAIN || gCurrentFunction == FUNCTION_TRANSMIT ||
	    gScanStateDir != SCAN_OFF) {
		gSondeShowRequest = true;
		return;
	}

	const uint8_t s_saved_rx_vfo = gEeprom.RX_VFO;
	RADIO_SelectVfos();
	gRxVfo         = gTxVfo;
	gEeprom.RX_VFO = gEeprom.TX_VFO;
	RADIO_SetupRegisters(true);
	APP_StartListening(FUNCTION_MONITOR);

	/* ⚠️ SIMPLIFIED (2026-09-13): a PC-side recording of the radio's own audio
	 * output, decoded independently of the RP2040 entirely
	 * (tools/decode_m10_pc.py, a faithful port of sonde_m10.c), had shown a
	 * plain manually-tuned VFO (FM, squelch open, nothing bypassed) decodes a
	 * full M10 position, while the Sonde screen with REG_2B bypassed
	 * (manually, via a direct BK4819_REG_2B poke) did not -- a whole earlier
	 * round of this session's own history chased that discrepancy through
	 * several dead ends (weak-signal filter narrowing, AFC, AGC, REG_48...)
	 * before finding the real cause elsewhere. What was never reconsidered
	 * until now: this project *already* ships a proper, menu-selectable
	 * "DSC" modulation for exactly this profile --
	 * `MODULATION_DISCRI` (`patch/radio.c.diff`/`radio.h.diff`, "DSC" in the
	 * Demodu menu since it was added) -- plain FM demodulation with REG_2B
	 * bypassed, and critically, RADIO_SetModulation() itself now RESTORES
	 * REG_2B (un-bypasses it) for every *other* modulation, something this
	 * screen's own manual poke never did on exit. Using the real mechanism
	 * instead of re-implementing its exact bit pattern by hand here removes
	 * a whole class of "did I bypass AND restore it correctly" bugs for
	 * free, and keeps this screen in sync with any future change to what
	 * MODULATION_DISCRI actually does. */
#ifdef ENABLE_BYP_RAW_DEMODULATORS
	RADIO_SetModulation(MODULATION_DISCRI);
#else
	/* MODULATION_DISCRI doesn't exist without this build flag (this
	 * project's own build.sh always sets it for V1, so this branch is
	 * unreachable in practice -- kept only so ENABLE_SONDE alone, without
	 * ENABLE_BYP_RAW_DEMODULATORS, still compiles, same defensive pattern
	 * as APRS_EnsureChannel() in aprs.c). */
	RADIO_SetModulation(MODULATION_FM);
#endif
	/* TRIED and REVERTED (2026-09-15), kept in sync with V3: AFC forced OFF
	 * for this screen specifically (a local override, without touching
	 * MODULATION_DISCRI's own general AFC-on behaviour used elsewhere on
	 * this firmware too -- a plain VFO, and the SAREX/SARSAT memory channels
	 * APRS_EnsureChannel() seeds in aprs.c). Tested against real M10
	 * captures (tools/decode_m10_pc.py, off the radio's own audio,
	 * independent of the RP2040/ADC) locking the header but never
	 * validating a checksum, decode confidence dropping across the capture
	 * -- a PLL/clock-tracking drift signature, distinct from the ADC-dropout
	 * AFC test further below (already proven on V3 to be a hardware event,
	 * unrelated to AFC). On-air result: fewer, not more, valid decodes with
	 * AFC off here -- and a plain VFO on this same radio, manually tuned to
	 * DSC (AFC on, via MODULATION_DISCRI's own formula, untouched), decoded
	 * noticeably better than this screen with AFC off. Reverted; this
	 * screen simply inherits MODULATION_DISCRI's own AFC-on behaviour again,
	 * nothing to override here. The drift itself is still real and
	 * unexplained -- next guess should be a different variable, not AFC
	 * again. */
	/* ⚠️ FIXED (2026-09-12): this firmware defines ENABLE_AM_FIX by default
	 * (Makefile: `ENABLE_AM_FIX ?= 1`, never overridden by this project's
	 * build.sh), which makes RADIO_SetupRegisters() -- the function a plain
	 * manually-tuned VFO goes through -- call
	 * `BK4819_SetFilterBandwidth(Bandwidth, true)`: the filter bandwidth
	 * stays fixed even if the chip's own RSSI logic judges the signal
	 * "weak", however briefly. This call used to pass `false` instead,
	 * silently REVERTING that back to allowing an automatic narrower
	 * bandwidth on a weak-signal judgement -- a real, previously-missed
	 * difference from the plain-VFO chain the PC-side test (see the note
	 * above) proved decodes cleanly. Even a brief, transient narrowing is
	 * enough to clip the fast transitions a 9600-baud Manchester signal
	 * needs. Now passes `true`, matching RADIO_SetupRegisters()'s own
	 * behaviour on this firmware exactly (this call is therefore
	 * redundant with it in the common case -- kept anyway to guarantee
	 * WIDE specifically regardless of whatever CHANNEL_BANDWIDTH the
	 * user's own VFO happens to be configured with). */
	BK4819_SetFilterBandwidth(BK4819_FILTER_BW_WIDE, true);
	/* TRIED and REVERTED (2026-09-11, diagnostic): disabling AFC here, kept
	 * in sync with V3 to test against a sharp, precisely-repeatable
	 * mid-transmission dropout (raw ADC pinned flat for ~50-80 ms) found on
	 * V3. A follow-up capture on V3 with AFC disabled still showed the
	 * *exact same* dropout -- AFC isn't the cause. Reverted on both; this
	 * firmware still never disables AFC for this screen. */
	/* TRIED and REVERTED (2026-09-11, diagnostic #2): freezing the AGC to a
	 * fixed, strong-signal-appropriate gain index, also kept in sync with
	 * V3 (bypassing BK4819_SetAGC()'s hardcoded index 3 -- see sarsat.c's
	 * own history of this same symptom). A follow-up capture on V3 with the
	 * AGC frozen still showed the exact same dropout -- not the cause
	 * either. Reverted.
	 *
	 * Investigation continues on V3, which has a periodic REG_48 rewrite
	 * this screen never had -- see patch/sonde.c there. Nothing further to
	 * test here for now. */
#ifdef ENABLE_APRS
	APRS_ResyncAfGainKnob();
	APRS_ApplyAfGain();
#endif

	gSondeShowRequest = false;
	s_dirty           = true;
	s_screen_open      = true;
	/* NOTE: same up-to-~5s delay before the C-Board notices the screen is
	 * open as the V3 port -- see sonde.c there for why. */

	KEY_Code_t prev_key = KEY_INVALID;
	uint16_t   held     = 0;
	bool       armed    = false;
	bool       run      = true;
	while (run)
	{
#ifdef ENABLE_UART
		while (UART_IsCommandAvailable())
			UART_HandleCommand();
#endif
		if (s_dirty)
		{
			s_dirty = false;
			SONDE_Draw();
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
				case KEY_UP:
				case KEY_DOWN:
					SONDE_Scroll(-((key == KEY_UP) ? +1 : -1));
					break;
				case KEY_EXIT: run = false; break;
				default: break;
				}
			}
		}
		prev_key = key;

		SYSTEM_DelayMs(20);
	}

	s_screen_open     = false;
	gSondeShowRequest = false;

	/* Un-freeze the AGC by hand before falling back to the normal VFO --
	 * RADIO_SetupAGC() (radio.c) has its own "did the settings actually
	 * change" cache (a static local), which never learned about our direct
	 * BK4819_REG_7E poke above (bypassed on purpose, see the note there,
	 * since BK4819_SetAGC() itself can't select an arbitrary index). If
	 * that cache still holds whatever "AGC enabled" state was current
	 * before this screen opened, RADIO_SetupRegisters()'s own call into
	 * RADIO_SetupAGC() below would see no change and skip writing the
	 * hardware register entirely -- silently leaving the AGC frozen at our
	 * test index for the rest of the radio session (SARSAT/APRS/normal VFO
	 * use included), not just this screen. Poke REG_7E back to auto
	 * (clear the fix-mode bit) directly so this can't depend on that
	 * cache's state either way. */
	{
		uint16_t r7e = BK4819_ReadRegister(BK4819_REG_7E);
		r7e = (uint16_t)(r7e & ~(1u << 15));
		BK4819_WriteRegister(BK4819_REG_7E, r7e);
	}

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

#endif /* ENABLE_SONDE */
