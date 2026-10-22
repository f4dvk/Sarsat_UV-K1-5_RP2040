/* F+7 launcher screen -- see app/rp_menu.h. Part of the
 * Sarsat_UV-K1-5_RP2040 project. Hand-written list, no overlay-app loader
 * (this base has none) -- just enough to pick which RP2040 C-Board screen
 * to open, same look (A/B/C letters) as f4hwn-v6's F+7 app menu. */
#include "app/rp_menu.h"

#if defined(ENABLE_SARSAT) || defined(ENABLE_APRS) || defined(ENABLE_SONDE)

#include <string.h>

#ifdef ENABLE_SARSAT
#include "app/sarsat.h"
#endif
#ifdef ENABLE_APRS
#include "app/aprs.h"
#endif
#ifdef ENABLE_SONDE
#include "app/sonde.h"
#endif
#include "app/app.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "ui/helper.h"
#include "ui/ui.h"
#include "misc.h"

/* One row per built-in screen, in a fixed order (A, B, C...). */
static const struct { const char *name; void (*run)(void); } RP_ITEMS[] = {
#ifdef ENABLE_APRS
    { "APRS-RP",   &APP_RunAprs   },
#endif
#ifdef ENABLE_SARSAT
    { "SARSAT-RP", &APP_RunSarsat },
#endif
#ifdef ENABLE_SONDE
    { "SONDE-RP",  &APP_RunSonde  },
#endif
#ifdef ENABLE_APRS
    { "GPS-RP",    &APP_RunGps    },
#endif
};
#define RP_N (sizeof(RP_ITEMS) / sizeof(RP_ITEMS[0]))

static void RP_Draw(uint8_t sel)
{
    UI_DisplayClear();
    UI_PrintStringSmallBold("RP2040", 2, 0, 0);
    for (uint8_t i = 0; i < RP_N; i++) {
        char s[12];
        s[0] = (char)('A' + i);
        s[1] = ' ';
        strcpy(s + 2, RP_ITEMS[i].name);
        UI_PrintStringSmallNormal(s, 2, 0, (uint8_t)(i + 1));
    }
    /* UI_PrintStringSmallBold falls back to the Normal font when
     * ENABLE_SMALL_BOLD is off (this build) -- no visible difference then.
     * Invert the selected row's pixels instead, same technique the stock
     * settings menu uses to highlight its own selection (ui/menu.c,
     * UI_DisplayMenu(): "gFrameBuffer[...][i] ^= 0xFF"). Always visible,
     * independent of ENABLE_SMALL_BOLD. */
    for (uint8_t i = 0; i < LCD_WIDTH; i++)
        gFrameBuffer[sel + 1][i] ^= 0xFF;
    ST7565_BlitFullScreen();
}

void APP_RunRpMenu(void)
{
    if (RP_N == 0)
        return;

    uint8_t      sel      = 0;
    bool         dirty    = true;
    KEY_Code_t   prev_key = KEY_INVALID;
    uint16_t     held     = 0;
    bool         armed    = false;   /* ignore the launch key until released */
    bool         run      = true;

    while (run)
    {
        if (dirty) {
            dirty = false;
            RP_Draw(sel);
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
                    sel = (sel == 0) ? (uint8_t)(RP_N - 1) : (uint8_t)(sel - 1);
                    dirty = true;
                    break;
                case KEY_DOWN:
                    sel = (uint8_t)((sel + 1) % RP_N);
                    dirty = true;
                    break;
                case KEY_MENU:
                    RP_ITEMS[sel].run();   /* blocking: runs until its own EXIT */
                    dirty = true;          /* redraw the list on return        */
                    /* The EXIT press that closed that screen is often still
                     * physically held when this loop resumes (its own cleanup
                     * takes a few ms, well under a normal key-up time).
                     * Without this, that same press reads as a fresh
                     * KEY_EXIT edge right here (armed was never cleared,
                     * prev_key is stale from before the launch) and closes
                     * this menu too -- one EXIT, two screens down, landing
                     * on the main VFO instead of back on this list. Require
                     * a release first, same as the launch key itself at
                     * entry. */
                    armed = false;
                    break;
                case KEY_EXIT:
                    run = false;
                    break;
                default:
                    break;
                }
            }
        }
        prev_key = key;
        SYSTEM_DelayMs(20);
    }

    GUI_SelectNextDisplay(DISPLAY_MAIN);
    gUpdateStatus  = true;
    gUpdateDisplay = true;
}

#endif /* ENABLE_SARSAT || ENABLE_APRS || ENABLE_SONDE */
