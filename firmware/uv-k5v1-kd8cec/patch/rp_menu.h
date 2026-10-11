/* Minimal F+7 launcher screen for the Sarsat_UV-K1-5_RP2040 project (V1 /
 * uv-k5v1-kd8cec). Lists whichever of SARSAT/APRS/SONDE are built in and
 * opens the one the user picks -- UP/DOWN to move, MENU to launch, EXIT to
 * close. Deliberately NOT a port of f4hwn-v6's apps/app_menu.c: this base
 * has no overlay-app loader (flash slots, installable .app blobs) at all,
 * so there is nothing to port -- this is a small hand-written list, same
 * look as f4hwn-v6's F+7 screen (A/B/C letters), none of its machinery.
 *
 * Replaces the fixed F+0/F+5/F+8 -> Sonde/APRS/SARSAT combos and the
 * ACTION_OPT_SARSAT/APRS/SONDE side-function entries (F1/F2) that used to
 * open these screens on this port: one entry point from here on, same
 * choice already made for uv-k1-k5v3's own F+7 app menu.
 */
#ifndef APP_RP_MENU_H
#define APP_RP_MENU_H

#if defined(ENABLE_SARSAT) || defined(ENABLE_APRS) || defined(ENABLE_SONDE)

void APP_RunRpMenu(void);

#endif
#endif /* APP_RP_MENU_H */
