/* ui/scanner.c -- STUB for the SARSAT/APRS build.
 * The DISPLAY_SCANNER screen is removed (see app/scanner.c). This draws
 * nothing; SCANNER_TimeSlice10ms() bounces straight back to DISPLAY_MAIN
 * within one tick so the empty screen is never actually seen.
 */

#include "ui/scanner.h"

void UI_DisplayScanner(void)
{
}
