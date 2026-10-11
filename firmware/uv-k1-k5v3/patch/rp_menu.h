/* F+7 launcher screen for the Sarsat_UV-K1-5_RP2040 project (V3 /
 * uv-k1-k5v3). Lists whichever of SARSAT/APRS/SONDE/SSTV are built in and
 * opens the one the user picks -- UP/DOWN to move, MENU to launch, EXIT to
 * close. Same hand-written list, same look (A/B/C/D letters), as the V1
 * port's own app/rp_menu.c -- not a port of f4hwn-v6's apps/app_menu.c
 * (overlay-app loader, flash slots, installable .app blobs): this base has
 * none of that, nothing to port.
 *
 * Replaces, as of 2026-10-11 (explicit request: remove every other way to
 * reach these screens, on all three radio firmwares), the fixed
 * F+8 -> SARSAT / F+5 -> APRS combos AND the ACTION_OPT_SARSAT/APRS/SONDE/
 * SSTV1/SSTVM1/SSTVPD90/SSTVPD120/SSTV_RX/SSTV_IMG side-function entries
 * (F1/F2) that used to open these screens on this port -- one entry point
 * from here on, SSTV included: its six actions (4 TX modes, RX, image
 * select) are now folded into APP_RunSstvRx() itself (see sstv_rx.c,
 * ported from f4hwn-v6's own SSTV-RP screen), so "D SSTV-RP" is still a
 * single "open a screen" call like the other three.
 */
#ifndef APP_RP_MENU_H
#define APP_RP_MENU_H

#if defined(ENABLE_SARSAT) || defined(ENABLE_APRS) || defined(ENABLE_SONDE) || defined(ENABLE_SSTV)

void APP_RunRpMenu(void);

#endif
#endif /* APP_RP_MENU_H */
