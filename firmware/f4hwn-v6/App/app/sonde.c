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
#include "app/afgain.h"     /* shared C-Board AF-gain setting (gAfGain) */

static char     s_line[SONDE_LINES][SONDE_LINE_CHARS + 1];
static uint16_t s_invert;
static uint8_t  s_nlines;
static uint8_t  s_scroll;
static bool     s_dirty;
static bool     s_screen_open;     /* APP_RunSonde() is running             */
static bool     s_dsc_mode;        /* RX profile: true=DSC, false=RAW -- KEY_5 */

bool gSondeShowRequest;

extern void SendReply(uint32_t Port, void *pReply, uint16_t Size);

static void SONDE_Reply(uint16_t reply_id, const uint8_t *data, uint8_t dlen)
{
	uint8_t buf[4 + 12];
	buf[0] = reply_id & 0xFF; buf[1] = reply_id >> 8;
	buf[2] = dlen;            buf[3] = 0;
	if (data && dlen)
		memcpy(buf + 4, data, dlen);
	SendReply(UART_PORT_UART, buf, 4 + dlen);
}

bool SONDE_ScreenOpen(void)
{
	return s_screen_open;
}

void SONDE_HandleUART(uint16_t id, const uint8_t *data, uint16_t size)
{
	/* see sarsat.c's identical guard: a UART TX burst right after a decode
	 * can otherwise register as a brief spurious PTT key-up. */
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

static void SONDE_Draw(void)
{
	char hdr[SONDE_LINE_CHARS + 1];

	UI_DisplayClear();

	/* max content is "SONDE 8-8/8^v DSC" (SONDE_LINES=8, so every %u is a
	 * single digit) = 17 glyphs, one under the 18-glyph hard limit. */
	if (s_nlines == 0)
		sprintf(hdr, "SONDE ... %s", s_dsc_mode ? "DSC" : "RAW");
	else {
		unsigned last = s_scroll + SONDE_VIS_ROWS;
		if (last > s_nlines) last = s_nlines;
		sprintf(hdr, "SONDE %u-%u/%u%c%c %s",
		        (unsigned)(s_scroll + 1), last, (unsigned)s_nlines,
		        s_scroll > 0 ? '^' : ' ',
		        last < s_nlines ? 'v' : ' ',
		        s_dsc_mode ? "DSC" : "RAW");
	}
	UI_PrintStringSmallBold(hdr, 2, 0, 0);

	for (uint8_t r = 0; r < SONDE_VIS_ROWS; r++)
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

static void SONDE_Scroll(int delta)
{
	int max = (int)s_nlines - SONDE_VIS_ROWS;
	if (max < 0) max = 0;
	int v = (int)s_scroll + delta;
	if (v < 0)   v = 0;
	if (v > max) v = max;
	if (v != (int)s_scroll) { s_scroll = (uint8_t)v; s_dirty = true; }
}

/* same backlight-fade flush as SARSAT_TickDelay()/APRS_TickDelay() -- see
 * sarsat.c for the on-air rationale. */
static void SONDE_TickDelay(uint32_t ms)
{
	for (int i = 0; i < 16; i++)
		BACKLIGHT_Update();
	SYSTEM_DelayMs(ms);
}

/* Two selectable RX profiles, toggled with KEY_5 (see APP_RunSonde()'s key
 * loop) without needing to reflash. Turns out they're closer to each other
 * than this project first assumed -- worth spelling out precisely now that
 * `App/driver/bk4829.c:BK4819_EnterRaw()` (what MODULATION_RAW actually does
 * on this firmware) has been read directly rather than guessed from its name:
 *
 *   DSC (default): reproduces V1's own working "DSC" chain
 *     (MODULATION_DISCRI there) -- FM demodulation (BK4819_AF_FM) with
 *     REG_2B (de-emphasis/HPF300/LPF3k) all bypassed and REG_43 held WIDE,
 *     with no weak-signal narrowing (see the REG_43 note below). SetRxA
 *     (REG_54/55) follows whatever the user's own menu choice is -- not
 *     overridden here. A real on-air comparison confirmed this decodes
 *     noticeably better than RAW below, on this radio.
 *
 *   RAW: plain stock F4HWN MODULATION_RAW -- RADIO_SetModulation(MODULATION_RAW),
 *     nothing added on top beyond the fix-ups below. Despite the name this is
 *     NOT a baseband/IQ path: BK4819_EnterRaw() also keeps
 *     BK4819_SetAF(BK4819_AF_FM) and bypasses the exact same REG_2B bits as
 *     DSC above -- so on the *filter-bypass* axis the two modes were already
 *     identical. RAW has no menu entry point of its own upstream
 *     (github.com/armel/uv-k1-k5v3-firmware-custom), so there was never any
 *     tuning of it to preserve -- kept as plain stock rather than layering
 *     DSC's register values on top of it again (redundant on the REG_2B
 *     axis, since DSC's own bits are already what BK4819_EnterRaw() sets).
 *
 * ⚠️ FIXED (2026-09-13, on explicit user request -- "mettre exactement la v3
 * comme la v1"), three axes, all now identical between DSC and RAW and
 * matching V1's DSC exactly:
 *   - AFC / REG_3D: BK4819_EnterRaw() forces AFC OFF (`afcDisableRegSpec =
 *     true`) and, back in RADIO_SetModulation()'s early-return branch for
 *     RAW, REG_3D = 0x0000 -- unlike V1's DISCRI (AFC on, REG_3D = 0x2AAB,
 *     same as its own FM path). REG_3D is re-asserted below to 0x2AAB for
 *     both profiles either way; AFC itself is re-asserted ON here too, for
 *     both -- an AFC-off test was tried and reverted on 2026-09-15, see
 *     that note right above the write for the on-air result.
 *   - AF DAC gain: hardcoded to max by RADIO_SetModulation() for every
 *     modulation on this firmware; V1's own radio.c.diff instead reads
 *     gEeprom.DAC_GAIN (to respect the C-Board's AF-gain override) for
 *     every modulation there, so this screen now does the same.
 *   - REG_43 (RF filter bandwidth): BK4819_SetFilterBandwidth() on THIS
 *     chip (App/driver/bk4829.c) used to completely ignore its
 *     `weak_no_different` argument -- since fixed for good at the driver
 *     level (see firmware/uv-k1-k5v3/build.sh's weak_no_different patch),
 *     so every VFO now shares the "no weak-signal narrowing" behaviour this
 *     screen pioneered. Sonde goes further than a plain VFO: on explicit
 *     user request ("il faut mettre le plus large possible"), the main RF
 *     sub-field <14:12> and the weak-signal one <11:9> (kept equal to it,
 *     same "no narrowing" reasoning as before) were first pushed to 111 =
 *     5.5 kHz, the widest this 3-bit field can encode -- doubled to 11.0 kHz
 *     by the same 25 kHz-mode <5>=1 bit already in the stock WIDE preset.
 *     On-air result (2026-09-15, alongside the AFC-off test reverted just
 *     above): still no valid decode, and a plain V1 VFO on DSC (~7.0 kHz
 *     class filter) did noticeably better -- suggesting 11.0 kHz let in
 *     more out-of-band noise than the wider edges were worth. Backed off to
 *     a midpoint (field 101 = 4.5 -> 9.0 kHz, REG_43 = 0x5A28): still no
 *     better. In parallel, real M10/M20 captures started showing an
 *     already-known symptom at a much bigger scale than before (a squelch
 *     LED blip and a multi-hundred-chip flat-zero ADC dropout mid-burst,
 *     see rp2040/src/sonde_m10.h's own history) -- and a genuine Vero
 *     VR-N76 (also BK4829-based, per the user) decodes the same sonde fine
 *     with HTCommander, ruling out a hard chip limitation. Working
 *     hypothesis: TOO WIDE a filter lets a large FM deviation swing through
 *     unshaped, over-driving the discriminator/AF gain stage into the kind
 *     of noise burst a glitch-based squelch reads as "signal lost" -- wider
 *     was never the fix, it was making this worse. Reverted to the
 *     ORIGINAL value from before any of this widening: REG_43 = 0x3628
 *     (field 011 = 3.5 -> 7.0 kHz, weak-signal field still equalized to it,
 *     no narrowing). Not applied to SARSAT or a plain VFO, which must keep
 *     the standard 25 kHz shape for voice/CTCSS. If 7.0 kHz still shows the
 *     same dropout, the next variable to test is narrower still (NARROW
 *     preset, ~4-5 kHz class), not wider again. */
static void SONDE_ApplyRxProfile(bool dsc)
{
	if (dsc) {
		RADIO_SetModulation(MODULATION_FM);
		uint16_t r2b = BK4819_ReadRegister(BK4819_REG_2B);
		r2b = (uint16_t)(r2b | (1u << 10) | (1u << 9) | (1u << 8));
		BK4819_WriteRegister(BK4819_REG_2B, r2b);
	} else {
		RADIO_SetModulation(MODULATION_RAW);
		/* ⚠️ FIXED (2026-09-13, point 2 of "mettre exactement la v3 comme la
		 * v1"): BK4819_EnterRaw() only ever touches BK4819_SetAF/REG_2B/AFC --
		 * unlike the FM branch of RADIO_SetModulation() (used by DSC above),
		 * it never re-asserts REG_31 (AM demod enable bit), REG_42, REG_2A or
		 * REG_2F. Not just cosmetic parity with DSC: without this, RAW
		 * inherits whatever those registers were left at by an earlier mode
		 * -- if the radio had been in AM right before opening this screen,
		 * REG_31's AM-demod-enable bit would still be set while "receiving"
		 * RAW, a real correctness bug, not just an asymmetry with DSC.
		 * Re-asserted here with the exact same values the FM branch writes. */
		uint16_t r31 = BK4819_ReadRegister(0x31);
		BK4819_WriteRegister(0x31, r31 & 0xfffe);   /* AM Demodulation Disable */
		BK4819_WriteRegister(0x42, 0x6b5a);
		BK4819_WriteRegister(0x2a, 0x7400);
		BK4819_WriteRegister(0x2f, 0x9890);
	}
	/* ⚠️ RESOLVED (2026-09-16, official Beken "BK4829 Registers Table"
	 * datasheet, DRT01-230606-C01, supplied by the user -- the same
	 * document bk4829.c's own REG_43 comment already cites by name):
	 * REG_47<11:8> is documented as "AF Output Selection" (0=Mute,
	 * 1=Normal AF Out, 2=Tone Out for Rx, 3=Beep Out for Tx, 6=CTCSS/CDCSS
	 * Out for Rx Test, 8=FSK Out for Rx Test) -- confirms BK4819_AF_FM (1)
	 * is the right value for that field. More importantly, the datasheet's
	 * own register-default table gives REG_47's power-on/reset value as
	 * 0x6140 -- bit for bit V1's formula ((6u<<12)|(AF<<8)|(1u<<6)), NOT
	 * this driver's own BK4819_SetAF() (0x6042 | (AF<<8) = 0x6142 for FM).
	 * The one-bit difference (bit 1) an earlier note here called "never
	 * explained" is real: bit 1 isn't part of any documented field in this
	 * table, and the chip's own factory default has it at 0 -- V3's stock
	 * driver sets it to 1 for every modulation, on every screen, not just
	 * this one, an undocumented deviation from Beken's own default that
	 * predates this project. RT950's REG_47 = 0xFB67 (tried and reverted
	 * twice above/below -- github.com/Hertzz58/Radtel-RT950-Pro-Firmware
	 * and github.com/JKI757/radtel-950-pro) sets several MORE undocumented
	 * bits on top and broke audio outright (SSB-sounding) -- likely
	 * RT950-PCB-specific, not applicable here. This screen re-asserts the
	 * datasheet-documented default, 0x6140, matching V1's own formula --
	 * already tested on air once before this note existed (during the
	 * BK4929-output-tap session) with no audio-quality complaint, only the
	 * (still unexplained, unrelated) ADC dropout persisting.
	 *
	 * UPDATE (2026-09-16): the shared driver's BK4819_SetAF() (called by
	 * RADIO_SetModulation() just above) now applies this exact same
	 * datasheet-documented value itself, with bit 13 (the only documented
	 * bit of the two, "AF Output Inverse Mode") made user-configurable via
	 * the new "AfInv" menu instead of hard-coded -- see build.sh. This
	 * explicit re-write is kept only to guarantee this screen never
	 * silently regresses to the old 0x6042-style base if the shared driver
	 * patch is ever reverted upstream; it now calls into the same toggle
	 * rather than duplicating the formula. */
	BK4819_SetAF(BK4819_AF_FM);
	/* TRIED and REVERTED (2026-09-15): AFC forced OFF here, to test against
	 * real M10 captures (tools/decode_m10_pc.py, off the radio's own audio,
	 * independent of the RP2040/ADC) locking the header but never validating
	 * a checksum, with the per-quarter decode confidence dropping across the
	 * capture -- a PLL/clock-tracking drift signature. Hypothesis: AFC
	 * chasing a transient DC bias in bursty Manchester data as if it were a
	 * real carrier offset, drifting the LO mid-packet. On-air result:
	 * fewer, not more, valid decodes with AFC off -- and a plain V1 VFO
	 * manually tuned to DSC (AFC on, via MODULATION_DISCRI's own formula)
	 * decoded noticeably better than this screen with AFC off. Reverted;
	 * AFC stays on for this screen, matching that V1 reference point and
	 * the stock DSC/RAW behaviour before this test. The drift itself is
	 * still real and unexplained -- next guess should be a different
	 * variable, not AFC again. */
	BK4819_SetRegValue(afcDisableRegSpec, false);
	BK4819_WriteRegister(BK4819_REG_3D, 0x2AAB);
	BK4819_WriteRegister(BK4819_REG_43, 0x3628);   /* back to 7.0 kHz, wider never helped -- see note above */

	/* ⚠️ NEW (2026-09-16, sur retour explicite -- "tu peux tester") : REG_54/
	 * REG_55 ("300Hz AF Response coefficient for Rx" au datasheet Beken) ne
	 * sont touches nulle part dans cet ecran -- ils heritent donc de ce que
	 * le menu SetRxA (Fonctions) a choisi. Le preset par defaut de ce menu,
	 * "FLAT" (index 0, celui recommande partout dans ce projet en cas de
	 * doute), ecrit REG_54=0x9009 / REG_55=0x3200 -- MAIS la vraie valeur de
	 * reset d'usine documentee par le datasheet est REG_54=0x9009 /
	 * REG_55=0x31A9. "FLAT" n'est donc pas, malgre son nom, exactement le
	 * comportement neutre de la puce -- REG_55 differe (0x3200 contre
	 * 0x31A9). La V1 ne touche jamais ces deux registres et reste donc en
	 * permanence sur la vraie valeur d'usine, sans jamais en avoir le choix.
	 * Force ici la vraie valeur documentee, independamment du choix SetRxA
	 * de l'operateur -- portee locale a cet ecran. Aucune information
	 * detaillee du datasheet sur ce que ces bits encodent precisement (pas
	 * de table de champs comme REG_43) -- test a une seule variable, retour
	 * arriere immediat si ca degrade quoi que ce soit. */
	BK4819_WriteRegister(0x54, 0x9009);
	BK4819_WriteRegister(0x55, 0x31A9);

	/* TRIED and REVERTED (2026-09-15, 2nd attempt): REG_47 = 0xFB67 again,
	 * this time paired with REG_48 built the same way the real RT950 OEM
	 * firmware pairs it (Ghidra decompile, github.com/JKI757/radtel-950-pro
	 * -- read purely to understand register semantics, no code copied):
	 * REG_48 = 0xB00F | (cal_value & 0x3F) << 4, with gEeprom.DAC_GAIN
	 * substituted for their per-radio flash calibration byte (no equivalent
	 * available). REG_37/REG_43/REG_30 already matched between the two
	 * firmwares beforehand. On-air result: audio still came out sounding
	 * like SSB, identical to the first (REG_47-alone) attempt -- so REG_48
	 * was never the missing piece. This confirms the likelier explanation
	 * already flagged before this test: REG_47's correct value probably
	 * depends on how the RF front-end is actually wired on the PCB (mixer/
	 * IF topology), which can differ between the RT950 and this radio even
	 * on the identical BK4829 chip -- no register combination guessed from
	 * outside would fix that. This avenue (porting RT950's exact register
	 * values for REG_47) is now treated as exhausted, not to be retried
	 * without a real schematic or datasheet section for this register --
	 * see the newer note above (2026-09-16) for what REG_47 actually
	 * settled on once the official Beken datasheet became available.
	 * REG_48 restored to this screen's own established formula (honour
	 * gEeprom.DAC_GAIN, the C-Board AF-gain override, same as before any of
	 * this RT950 detour). */
	BK4819_SetRegValue(afDacGainRegSpec, gEeprom.DAC_GAIN & 0xF);
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

	/* ⚠️ REALIGNED ON V1 (2026-09-13, on explicit user request): opens on DSC
	 * -- see SONDE_ApplyRxProfile()'s comment above for the full rationale
	 * and what each of DSC/RAW actually does. KEY_5 (below) toggles between
	 * them without needing to reflash. */
	s_dsc_mode = true;
	SONDE_ApplyRxProfile(s_dsc_mode);
	/* ⚠️ NEW (2026-09-14, on user's on-air observation -- "en FM 25 kHz avec
	 * désaccentuation le signal n'est pas reçu immédiatement, comme un
	 * AFC/AGC hors limite qui met du temps à synchroniser ; pas de souci sur
	 * la V1"): plausible root cause found by reading RADIO_SetupAGC()
	 * (radio.c) on both firmwares. It caches its last (listeningAM, disable)
	 * pair in a static and skips BK4819_InitAGC() -- a live rewrite of the
	 * whole AGC gain-index table (REG_10-14/49/7B) -- whenever the same pair
	 * is asked for again. V3 seeds that cache to 0xFF, so the very next
	 * plain-FM RX setup after ANY state that used a different pair (AM, or
	 * disable=true from a TX/mute) does a REAL table rewrite, plausibly
	 * kicking off a live AGC re-acquisition transient right as the operator
	 * starts listening -- whether that first FM-class touch happens on a
	 * manually-tuned 25 kHz VFO channel or on this screen's own DSC/RAW
	 * profile makes no difference, it's the same shared cache either way.
	 * V1's own RADIO_SetupAGC() seeds that same static to 0 instead of
	 * 0xFF -- which happens to already equal the very first plain-FM
	 * (non-AM, non-disable) request, so BK4819_InitAGC() is never even
	 * called there for ordinary FM listening. That divergence (not a
	 * DSC-vs-RAW register difference -- those are identical now, see
	 * SONDE_ApplyRxProfile()'s own comment above) is the most likely reason
	 * V1 never shows this symptom regardless of what was used right before.
	 *
	 * Not a fix in itself (re-ordering register writes doesn't buy any real
	 * settle TIME -- the AGC loop still needs to physically converge after
	 * a table rewrite, however early in the code that rewrite happens): this
	 * gives the freshly-rewritten table a fixed head start to settle while
	 * the screen is still opening, before the operator's own attention (and
	 * any capture) is on the signal, rather than live during the first
	 * seconds of listening. 300 ms is a guess, matched to the deferred-mute
	 * tuning elsewhere in this project -- lengthen it if a slow first-lock
	 * is still seen on air, or revert this delay outright if it makes no
	 * audible/measurable difference (single-variable test, same discipline
	 * as every other entry in this investigation). V1 doesn't get this delay
	 * -- it was never shown to need it. */
	SONDE_TickDelay(300);
	/* TRIED and REVERTED (2026-09-11, diagnostic): disabling AFC here, to
	 * test it against a sharp, precisely-repeatable mid-transmission dropout
	 * (raw ADC pinned flat for ~50-80 ms, confirmed via the RP2040's own
	 * raw 'y' capture -- i.e. the ADC itself, not a decode artifact) found
	 * inside otherwise-good M20 frames. A follow-up on-air capture with AFC
	 * disabled still showed the *exact same* dropout -- AFC isn't the
	 * cause. Reverted; AFC stays at its stock/enabled state (RADIO_SetupAGC
	 * / RADIO_SetModulation(MODULATION_FM) already turns it on, nothing to
	 * do here). See the REG_48 note below for where this investigation
	 * went next. */
	/* TRIED and REVERTED (2026-09-11, diagnostic #2): freezing the AGC to a
	 * fixed, strong-signal-appropriate gain index (bypassing
	 * BK4819_SetAGC()'s hardcoded index 3, which cost SARSAT ~30 dB -- see
	 * sarsat.c's own history of this same symptom), to test it against the
	 * same dropout. A follow-up on-air capture with the AGC frozen still
	 * showed the *exact same* dropout, unchanged -- AGC gain-switching
	 * isn't the cause either. Reverted.
	 *
	 * ⚠️ CURRENT TEST (2026-09-11, diagnostic #3): re-checked
	 * docs/hardware.md -- the RP2040's ADC node taps the radio's SPEAKER/HP
	 * output, not the BK4819 discriminator pin directly, so anything that
	 * touches the chip's internal RX audio DAC (REG_48) is visible to it.
	 * This screen's own loop (below, "regain_10ms") re-asserts
	 * AFGAIN_Apply() -- which writes REG_48 -- unconditionally every
	 * ~500 ms, including when gAfGain is "auto" (0), where it rewrites the
	 * exact same stock values every time for no functional reason. That is
	 * the one piece of code every earlier test in this investigation left
	 * untouched (AFC/AGC/filter changes all layered around it, never
	 * removed it) -- if a REG_48 write causes so much as a brief internal
	 * mute/anti-click pulse on the chip's audio DAC every time it's
	 * (re)written, an unconditional ~500 ms re-assert would explain a
	 * dropout that no RX-chain setting could ever touch. Guarded below
	 * (same condition AFGAIN_TimeSlice() already uses) so the write is
	 * skipped entirely while gAfGain is "auto" -- pure upside even if this
	 * isn't the cause, since it was a no-op write regardless. If the
	 * dropout disappears, this is confirmed and the guard stays; if not,
	 * this can be reverted next without touching anything else, keeping
	 * this a single-variable test like every guess before it. */
	AFGAIN_ResyncKnob();
	AFGAIN_Apply();

	gSondeShowRequest = false;
	s_dirty           = true;
	s_screen_open     = true;
	/* NOTE: unlike SARSAT/APRS (which push a status reply the instant their
	 * screen opens/closes), the RP2040 only learns "Sonde screen open" on
	 * its next periodic HELLO (main.c, every 5 s) -- so it can take up to
	 * ~5 s after this screen opens before the C-Board switches its ADC into
	 * SONDE capture mode. Acceptable for now (a radiosonde burst repeats
	 * every ~1 s, so a slow-to-arm capture just misses the first one or
	 * two); revisit with an early-push hook if that proves annoying on air. */

	KEY_Code_t   prev_key    = KEY_INVALID;
	uint16_t     held        = 0;
	bool         armed       = false;
	bool         run         = true;
	uint16_t     regain_10ms = 0;
	while (run)
	{
#ifdef ENABLE_UART
		while (UART_IsCommandAvailable(UART_PORT_UART))
			UART_HandleCommand(UART_PORT_UART);
#endif
		if (s_dirty)
		{
			s_dirty = false;
			SONDE_Draw();
		}

		/* ⚠️ TEST (2026-09-11, diagnostic #3b): the previous guard here
		 * ("skip while gAfGain is auto") never actually took effect on the
		 * hardware this was tested on -- the AF gain is configured FIXED
		 * (not auto), so the periodic REG_48 re-assert kept firing every
		 * ~500 ms through that entire test, unchanged. The "REG_48 doesn't
		 * explain the dropout" conclusion drawn from that test was
		 * therefore not actually established -- this variable was never
		 * isolated. Real test this time: the periodic re-assert is
		 * disabled outright, unconditionally, for this screen. It exists
		 * to fight the radio's own background volume-knob tracking (which
		 * can silently overwrite gEeprom.VOLUME_GAIN/DAC_GAIN between
		 * re-asserts, per afgain.c) -- turning the physical volume knob
		 * while this screen is open could therefore make the fixed AF
		 * gain drift until the screen is reopened, for as long as this
		 * test runs. sarsat.c's identical loop is intentionally NOT
		 * touched (never implicated by any capture here, and its much
		 * shorter ~520 ms bursts make a bad-timed rewrite far less likely
		 * to land mid-frame in the first place). If the dropout disappears
		 * with this disabled, REG_48 is confirmed and a permanent fix
		 * (e.g. re-assert only on an actual knob-turn event, not on a
		 * timer) is worth designing; if not, revert (restore the call)
		 * and this is truly closed. */
		(void)regain_10ms;

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
				case KEY_DOWN: {
					int d = (key == KEY_UP) ? +1 : -1;
					if (!gEeprom.SET_NAV) d = -d;   /* see sarsat.c's note */
					SONDE_Scroll(-d);
					break;
				}
				case KEY_5:
					s_dsc_mode = !s_dsc_mode;
					SONDE_ApplyRxProfile(s_dsc_mode);
					s_dirty = true;
					break;
				case KEY_EXIT: run = false; break;
				default: break;
				}
			}
		}
		prev_key = key;

		SONDE_TickDelay(20);
	}

	s_screen_open     = false;
	gSondeShowRequest = false;

	gMonitor       = false;
	gEeprom.RX_VFO = s_saved_rx_vfo;
	FUNCTION_Select(FUNCTION_FOREGROUND);
	RADIO_ConfigureChannel(0, VFO_CONFIGURE);
	RADIO_ConfigureChannel(1, VFO_CONFIGURE);
	RADIO_SelectVfos();
	/* ⚠️ FIX (2026-09-12): neither RADIO_ConfigureChannel() nor
	 * RADIO_SetupRegisters() actually pushes a modulation choice to the
	 * BK4819 -- only an explicit RADIO_SetModulation() call does that (see
	 * the entry code above, and APP_StartListening(), which is what a
	 * normal VFO select relies on and this screen's exit path never calls
	 * again). Every RX profile this screen has used until now happened to
	 * be MODULATION_FM, the same as most users' own VFO, so this gap never
	 * showed -- the RAW test just above would otherwise leave the radio
	 * stuck in BASEBAND1 output after leaving this screen, on every other
	 * function (SARSAT, APRS, normal VFO listening) until the user manually
	 * touched modulation again. Restore the real VFO's own configured
	 * modulation explicitly. */
	RADIO_SetModulation(gRxVfo->Modulation);
	RADIO_SetupRegisters(true);

	BACKLIGHT_TurnOn();
	gKeyLockCountdown = 30;

	GUI_SelectNextDisplay(DISPLAY_MAIN);
	gUpdateStatus  = true;
	gUpdateDisplay = true;
}

#endif /* ENABLE_SONDE */
