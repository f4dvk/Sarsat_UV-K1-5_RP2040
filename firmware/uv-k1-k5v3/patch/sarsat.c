/* SARSAT screen — see app/sarsat.h. Part of the Sarsat_UV-K1-5_RP2040 project.
 *
 * Port of the UV-K5 V1 (KD8CEC) module to the F4HWN base (UV-K1 / UV-K5 V3,
 * PY32F071 + BK4829). The only real differences from the V1 source are the
 * UART API, which here takes an explicit Port argument (UART_PORT_UART vs
 * UART_PORT_VCP) -- everything else (VFO_Info_t layout, BK4819_* register
 * calls, UI_PrintString*, RADIO_*) is close enough to egzumer/DualTachyon
 * upstream that the logic ports over unchanged. */
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
#include "app/afgain.h"     /* shared C-Board AF-gain setting (gAfGain) */
#ifdef ENABLE_APRS
	#include "app/aprs.h"
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
extern void SendReply(uint32_t Port, void *pReply, uint16_t Size);

/* Phase 2: answer the RP2040. Reply ID = command | 0x8000, payload = data. */
static void SARSAT_Reply(uint16_t reply_id, const uint8_t *data, uint8_t dlen)
{
	uint8_t buf[4 + 12];
	buf[0] = reply_id & 0xFF; buf[1] = reply_id >> 8;
	buf[2] = dlen;            buf[3] = 0;
	if (data && dlen)
		memcpy(buf + 4, data, dlen);
	SendReply(UART_PORT_UART, buf, 4 + dlen);
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
	/* Wire modulation byte (rp2040/src/main.c mod_name[]/MOD_IS_FM_LIKE):
	 * 0 FM, 1 AM, 2 USB, 3 BYP, 4 RAW, 5 DSC -- only 0 and 5 are treated as
	 * "flat FM discriminator, safe to decode". That table was written for
	 * the V1 (KD8CEC) firmware, where index 4 ("RAW") is its baseband/IQ
	 * mode that beats a strong carrier into a whistle (deliberately NOT
	 * flagged FM-like) and index 5 ("DSC", MODULATION_DISCRI) is the flat
	 * discriminator mode added there. This firmware's own MODULATION_RAW
	 * (BK4819_EnterRaw()) is that same flat-discriminator profile, but it
	 * happens to sit at index 4 in *this* firmware's ModulationMode_t enum
	 * -- same wire byte value as V1's very different "RAW", so without this
	 * remap the RP2040 wrongly logs "radio not in FM/DSC" for it. Report it
	 * as wire value 5 ("DSC") instead, matching what it actually is. */
	uint8_t wire_mod;
	if (s_screen_open)
		wire_mod = 0;                       /* FM: the screen forces its own profile */
	else if (v->Modulation == MODULATION_RAW)
		wire_mod = 5;                       /* DSC slot: same flat-discriminator profile */
	else
		wire_mod = (uint8_t)v->Modulation;  /* FM/AM/USB/BYP line up with the wire table */
	uint8_t p[16] = {
		(uint8_t)(gEeprom.TX_VFO & 1),
		wire_mod,
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
	UI_PrintStringSmallBold("NIVEAU 5:txt UP/DN", 2, 0, 0);

	sprintf(s, "rms%-6u pk%u", s_lvl.rms, s_lvl.peak);
	UI_PrintStringSmallNormal(s, 2, 0, 1);
	sprintf(s, "dc%-6u clip%u%%", s_lvl.dc, s_lvl.clip);
	UI_PrintStringSmallNormal(s, 2, 0, 2);
	sprintf(s, "adc %u-%u", s_lvl.amin, s_lvl.amax);
	UI_PrintStringSmallNormal(s, 2, 0, 3);

	/* UP/DOWN here tune the BK4819 AF gain feeding the C-Board tap -- set the
	 * volume pot to max and adjust this against the rms bar. */
	if (gAfGain < 1 || gAfGain > 78)
		strcpy(s, "AF gain: auto");
	else
		sprintf(s, "AF gain: %u", gAfGain);
	UI_PrintStringSmallNormal(s, 2, 0, 4);

	f = s_lvl.rms / 300;                 /* full bar ~ rms 4800; aim mid-scale */
	if (f > 16) f = 16;
	memset(s, '#', f);
	memset(s + f, '.', 16 - f);
	s[16] = 0;
	UI_PrintStringSmallNormal(s, 2, 0, 5);

	/* row 6 is the last usable content row (gFrameBuffer has only 7) */
	UI_PrintStringSmallBold(verd[s_lvl.verdict < 5 ? s_lvl.verdict : 0], 2, 0, 6);
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
	UI_PrintStringSmallBold(hdr, 2, 0, 0);

	/* rows 1..SARSAT_VIS_ROWS: the visible slice of the line buffer */
	for (uint8_t r = 0; r < SARSAT_VIS_ROWS; r++)
	{
		const uint8_t li = s_scroll + r;
		if (li >= s_nlines || !s_line[li][0])
			continue;
		if (s_invert & (1u << li))
			UI_PrintStringSmallBold(s_line[li], 2, 0, r + 1);
		else
			UI_PrintStringSmallNormal(s_line[li], 2, 0, r + 1);
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

/* Sleep one popup-loop tick. Also flushes any pending backlight PWM fade to
 * completion first -- same fix as FOXHUNT_TickDelay() (app/foxhunt.c) and
 * APRS_TickDelay() (patch/aprs.c): on this firmware BACKLIGHT_TurnOn() only
 * arms a fade (target brightness + step), the fade itself only advances via
 * BACKLIGHT_Update(), normally pumped by the 10 ms tick this blocking screen
 * doesn't run. BACKLIGHT_Update() is pumped 16 times back-to-back rather
 * than once per 10 ms slice: the fade always completes in <= 16 steps
 * regardless of the jump size (fadeStep is diff/16), so flushing it up front
 * makes a brightness change look instant instead of visibly ramping over the
 * ~160 ms this screen's own loop would otherwise spread it across (see
 * APRS_TickDelay()'s longer comment for the on-air symptom this avoids). */
static void SARSAT_TickDelay(uint32_t ms)
{
	for (int i = 0; i < 16; i++)
		BACKLIGHT_Update();
	SYSTEM_DelayMs(ms);
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
	 *     that was the reported symptom on the V1/KD8CEC port; assume the same
	 *     here rather than re-discover it on air.
	 *   - REG_2B bits 10/9/8 : drop RX de-emphasis / HPF300 / LPF3k so the
	 *     bi-phase-L transitions pass flat -- this is exactly what
	 *     BK4819_EnterRaw() (driver/bk4829.c) does; written by hand here so the
	 *     modulation stays MODULATION_FM (discriminator), not RAW/baseband.
	 *   - WIDE IF so the beacon's sidebands are not clipped. */
	RADIO_SetModulation(MODULATION_FM);
	BK4819_SetFilterBandwidth(BK4819_FILTER_BW_WIDE, false);
	{
		uint16_t r2b = BK4819_ReadRegister(BK4819_REG_2B);
		r2b |= (1u << 10) | (1u << 9) | (1u << 8);
		BK4819_WriteRegister(BK4819_REG_2B, r2b);
	}
	/* (A REG 0x54/0x55 pin to 0x9009/0x31A9 was tried here, to match the V1's
	 * untouched audio filter -- reverted with the APRS RX-alignment batch that
	 * killed decoding. The SetRxA profile -- FLAT by default -- stands, same as
	 * the state this screen was validated on air with. Set SetRxA=FLAT if in
	 * doubt.) */
	/* EXPERIMENTAL (3rd guess), not yet confirmed on air: disable AFC
	 * (Automatic Frequency Control). Reported symptom this targets: a frame
	 * audible right after opening this screen, no longer audible on
	 * following frames -- "un AGC qui s'ecarte ou AFC ?". AFC continuously
	 * nudges the LO based on the discriminator's DC output to keep a signal
	 * centered; RADIO_SetModulation(MODULATION_FM) just above explicitly
	 * turned it ON (afcDisableRegSpec = (modulation != MODULATION_FM), false
	 * for FM = not disabled). With REG_2B's de-emphasis/HPF/LPF bypassed for
	 * a flat discriminator, AFC sees a very different DC/noise character
	 * than it was tuned for on normal filtered FM audio -- if it drifts the
	 * LO away chasing burst/noise content instead of genuine carrier offset,
	 * that would explain exactly "works right at entry, degrades afterwards"
	 * without touching squelch (the regression before) or overall gain (the
	 * AGC-freeze attempt, also reverted). afcDisableRegSpec is the same
	 * documented RegisterSpec RADIO_SetModulation() itself uses
	 * (driver/bk4819-regs.h) -- narrower and lower-risk than either previous
	 * guess: it only stops ongoing frequency correction, it doesn't change
	 * gain or squelch at all. If this makes things worse, revert it the same
	 * way as the other two -- don't stack a 4th guess on top. */
	BK4819_SetRegValue(afcDisableRegSpec, true);
	/* TRIED and REVERTED (2nd guess after the squelch one above): freeze the
	 * receiver AGC to a fixed gain step (RADIO_SetupAGC(false, true) ->
	 * BK4819_SetAGC(false) -> REG_7E fixed AGC index 3, not user-tunable) to
	 * address "un bout de trame comme un gain automatique... sur la sortie
	 * discri". Measured on air right after: a -107 dBm frame, audible in
	 * plain VFO mode, was NOT audible at all in this screen, while a
	 * -77 dBm frame was -- a ~30 dB-class sensitivity gap. Whether this gap
	 * pre-existed (WIDE filter bandwidth alone costs some SNR margin) or was
	 * introduced/worsened by freezing AGC at whatever gain index 3 happens
	 * to be (plausibly a lower step meant for strong-signal headroom, wrong
	 * for a weak one) isn't established -- reverted rather than guess again.
	 * Re-test the same -107/-77 dBm comparison on this reverted build first,
	 * to know whether the gap is a pre-existing baseline or was this change,
	 * before trying anything else here. */
	/* TRIED and REVERTED: forcing BK4819_SetupSquelch() to the "always open"
	 * values (the same ones this firmware's own SQL=0 uses) to work around a
	 * weak-signal "poc" instead of a full decode. Confirmed on air to be a
	 * net regression -- total silence even on a signal that decoded fine
	 * before this call was added, worse than the original complaint. Root
	 * cause of the "poc" is therefore NOT (only) the squelch thresholds, or
	 * this combination of values has some other side effect on this chip/
	 * firmware that isn't understood yet. Do not re-add this call without
	 * fresh on-air [lvl]/[burst] numbers to actually explain it -- see
	 * patch/integration.md for the history. */
	/* re-assert the C-Board AF gain: RADIO_SetupRegisters / APP_StartListening
	 * above just wrote REG_48 from gEeprom.VOLUME_GAIN/DAC_GAIN, which a menu
	 * calibration reload may have reset to stock -> wrong level on reopen.
	 * Resync "auto"'s stock-knob capture first: without this, "auto" stays
	 * latched to whatever the volume knob was at the *first* SARSAT/APRS
	 * screen opened since power-on, so turning the knob up later (this
	 * screen's own point: pot to max, use the level view instead) had no
	 * effect and read as a sensitivity loss ("needs a stronger signal"). */
	AFGAIN_ResyncKnob();
	AFGAIN_Apply();

	gSarsatShowRequest = false;
	s_view             = 0;      /* always open on the decode, not the level view */
	s_dirty            = true;
	s_screen_open      = true;
	SARSAT_ReplyStatus();        /* let the C-Board know the screen state early */

	KEY_Code_t   prev_key   = KEY_INVALID;
	uint16_t     held       = 0;
	bool         armed      = false; /* ignore the launch key until released */
	bool         run        = true;
	bool         gain_dirty = false; /* an unsaved AF-gain change on the level view */
	uint16_t     regain_10ms = 0;    /* re-assert AFGAIN every ~500 ms (see below) */
	while (run)
	{
#ifdef ENABLE_UART
		/* drain: the RP2040 sends a burst of ~14 line frames per decode */
		while (UART_IsCommandAvailable(UART_PORT_UART))
			UART_HandleCommand(UART_PORT_UART);
#endif
		if (s_dirty)
		{
			s_dirty = false;
			SARSAT_Draw();
		}

		/* a calibration reload elsewhere (VOX/MIC/cal menus, reachable from
		 * this screen only via KEY_EXIT so not normally an issue here, but
		 * cheap enough to just always do) can reset gEeprom.VOLUME_GAIN/
		 * DAC_GAIN to stock -- re-assert the override periodically. */
		if (++regain_10ms >= 25) {            /* ~500 ms (loop is ~20 ms/iter) */
			regain_10ms = 0;
			AFGAIN_Apply();
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
				/* UP/DOWN scroll the decode, or (level view) tune AF gain */
				case KEY_UP:
				case KEY_DOWN: {
					int d = (key == KEY_UP) ? +1 : -1;
					/* SetNav (menu Service) : sur l'UV-K1, les touches
					 * physiques UP/DOWN sont etiquetees LEFT/RIGHT et le
					 * firmware inverse deja Direction partout ailleurs
					 * (menu.c, main.c, scanner.c, spectrum.c...) quand
					 * gEeprom.SET_NAV est faux (defaut sur UV-K1) -- on
					 * suit la meme convention ici pour rester coherent. */
					if (!gEeprom.SET_NAV) d = -d;
					if (s_view == 1) {
						int v = (gAfGain >= 1 && gAfGain <= 78) ? gAfGain : 0;
						v += d;
						if (v < 0)  v = 0;
						if (v > 78) v = 78;
						gAfGain = (uint8_t)v;
						AFGAIN_Apply();
						gain_dirty = true;
						s_dirty = true;
						break;
					}
					SARSAT_Scroll(-d);
					break;
				}
				case KEY_5:
					if (s_view == 1 && gain_dirty) {
						AFGAIN_Save(); gain_dirty = false;
					}
					s_view ^= 1; s_dirty = true;
					SARSAT_ReplyStatus();   /* tell the C-Board: fast level telemetry on/off */
					break;  /* text <-> level */
				case KEY_EXIT: run = false;       break;
				default: break;
				}
			}
		}
		prev_key = key;

		SARSAT_TickDelay(20);
	}

	s_screen_open      = false;
	SARSAT_ReplyStatus();            /* screen closed -> C-Board back to full windows */
	gSarsatShowRequest = false;      /* a late 0x06C1 must not re-open at once  */
	if (gain_dirty) AFGAIN_Save();

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
