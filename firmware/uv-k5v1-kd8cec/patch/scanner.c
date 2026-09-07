/* app/scanner.c -- STUB for the SARSAT/APRS build.
 *
 * The stock KD8CEC/egzumer DISPLAY_SCANNER screen (CTCSS/DCS tone finder,
 * "F+4" active-frequency hunt, and the R-CTCS / R-DCS in-submenu auto-detect)
 * is unrelated to this project and cost ~1.5 KB of flash -- removed to make
 * room. The ordinary memory-channel scan (chFrScanner.c, UP/DOWN scan through
 * the channel list) is a different module and is untouched.
 *
 * These stubs keep every call site linking. SCANNER_TimeSlice10ms() -- pumped
 * unconditionally from APP_TimeSlice10ms() -- immediately bounces the radio
 * back out if anything ever routes to the scanner screen or arms the CSS
 * background scan, and restores the cross-band setting the launch code stashed
 * in gBackup_CROSS_BAND_RX_TX. So F+4 / F+* / the submenu scan key just flash
 * for one tick and return, no dead screen.
 */

#include "app/scanner.h"
#include "misc.h"
#include "settings.h"
#include "ui/ui.h"

DCS_CodeType_t    gScanCssResultType     = CODE_TYPE_OFF;
uint8_t           gScanCssResultCode     = 0xFF;
bool              gScanSingleFrequency   = false;
SCAN_SaveState_t  gScannerSaveState      = SCAN_SAVE_NO_PROMPT;
uint8_t           gScanChannel           = 0;
uint32_t          gScanFrequency         = 0;
SCAN_CssState_t   gScanCssState          = SCAN_CSS_STATE_OFF;
uint8_t           gScanProgressIndicator = 0;
bool              gScanUseCssResult      = false;

void SCANNER_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	(void)Key; (void)bKeyPressed; (void)bKeyHeld;
	gRequestDisplayScreen = DISPLAY_MAIN;
}

void SCANNER_Start(bool singleFreq)
{
	(void)singleFreq;
}

void SCANNER_Stop(void)
{
}

void SCANNER_TimeSlice10ms(void)
{
	if (gCssBackgroundScan) {
		gCssBackgroundScan    = false;
		gRequestDisplayScreen = DISPLAY_MENU;
		gUpdateStatus         = true;
	}
	if (gScreenToDisplay == DISPLAY_SCANNER) {
		gEeprom.CROSS_BAND_RX_TX = gBackup_CROSS_BAND_RX_TX;
		gRequestDisplayScreen   = DISPLAY_MAIN;
		gUpdateStatus           = true;
		gUpdateDisplay          = true;
	}
}

void SCANNER_TimeSlice500ms(void)
{
}

bool SCANNER_IsScanning(void)
{
	return gCssBackgroundScan || (gScreenToDisplay == DISPLAY_SCANNER);
}
