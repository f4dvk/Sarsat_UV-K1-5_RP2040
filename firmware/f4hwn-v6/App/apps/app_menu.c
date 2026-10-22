/* Copyright 2026 Armel F4HWN
 * https://github.com/armel
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include "apps/app_overlay.h"

#ifdef ENABLE_FEAT_F4HWN_OVERLAY_APPS

#include <string.h>
#include "apps/app_menu.h"
#include "app/app.h"
#include "app/uart.h"
#include "driver/backlight.h"
#include "driver/st7565.h"
#include "driver/keyboard.h"
#include "driver/system.h"
#include "driver/gpio.h"
#include "ui/helper.h"
#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
#include "k5viewer.h"
#endif
#ifdef ENABLE_APRS
#include "app/aprs.h"   /* APP_RunAprs (entree F+7) */
#endif
#ifdef ENABLE_SSTV
#include "app/sstv_rx.h"  /* APP_RunSstvRx (entree F+7) */
#endif
#ifdef ENABLE_SARSAT
#include "app/sarsat.h" /* APP_RunSarsat (entree F+7) */
#endif
#ifdef ENABLE_SONDE
#include "app/sonde.h"  /* APP_RunSonde (entree F+7) */
#endif

/* "F4HWN APPS" banner and the same thin separator used by the multiboot
 * selector.  Bit 3 leaves room for the selected-row capsule's top edge. */
static void app_status_bar(void)
{
    UI_StatusClear();
    GUI_DisplaySmallestInverse("F4HWN APPS", 44, 0, true, true, 84);

    for (uint8_t x = 2u; x < LCD_WIDTH - 2u; x++)
        gFrameBuffer[0][x] |= 0x08u;
}

/* Fixed selection capsule around the primary information (the app name).
 * Slot number stays in the normal font; size is plain 3x5 metadata. */
#define APP_NAME_BOX_START 19u
#define APP_NAME_BOX_END   102u
#define APP_NAME_TEXT_X    21u

static void app_wait_release(void);

static void app_invert_name(uint8_t line)
{
    gFrameBuffer[line][APP_NAME_BOX_START] ^= 0x7Fu;
    for (uint8_t x = APP_NAME_BOX_START + 1u; x < APP_NAME_BOX_END; x++)
    {
        gFrameBuffer[line][x]      ^= 0xFFu;
        gFrameBuffer[line - 1u][x] ^= 0x80u;
    }
    gFrameBuffer[line][APP_NAME_BOX_END] ^= 0x7Fu;
}

/* UP / DOWN repeat while held: 10 ms units before the first repeat and between
 * the next ones (the debounce and the redraw add ~45 ms to each). */
#define APP_KEY_REPEAT_FIRST 40u
#define APP_KEY_REPEAT_NEXT  5u

/* Debounced blocking key read, then wait for release (mirrors mb_get_key).
 * UP and DOWN held past the repeat delay are returned without waiting for the
 * release, again and again: a held key scrolls the list. The release that ends
 * such a hold returns nothing. */
static KEY_Code_t app_get_key(void)
{
    static bool held;   /* the last key was returned by the repeat delay */
#if defined(ENABLE_UART) || defined(ENABLE_USB)
    const uint8_t slot_revision = APP_SlotRevision();
#endif

    for (;;)
    {
#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
        /* APP_MenuOpen() is modal and does not return to APP_Update(). Keep
         * serial key injection and the viewer connection alive while waiting. */
        K5VIEWER_ParseInput();
#endif
#if defined(ENABLE_UART) || defined(ENABLE_USB)
        UART_ServiceCommands();
        if (APP_SlotRevision() != slot_revision)
            return KEY_INVALID;
#endif
        APP_ModalBacklightTick(true);

        if (APP_IsScreenSaverDisplayed())
        {
            if (KEYBOARD_GetKey() != KEY_INVALID)
            {
                APP_ModalScreenSaverExit();
                BACKLIGHT_TurnOn();
                app_wait_release();
                return KEY_INVALID;
            }

            SYSTEM_DelayMs(10);
            continue;
        }

#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
        K5VIEWER_Update(false);
#endif
        KEY_Code_t key = KEYBOARD_Poll();
        if (key != KEY_INVALID)
        {
            SYSTEM_DelayMs(30);
            if (KEYBOARD_Poll() == key)
            {
                BACKLIGHT_TurnOn();
                uint8_t wait = 0u;       /* 0: no repeat, wait for the release */
                if (key == KEY_UP || key == KEY_DOWN)
                    wait = held ? APP_KEY_REPEAT_NEXT : APP_KEY_REPEAT_FIRST;
                while (KEYBOARD_Poll() != KEY_INVALID)
                {
                    SYSTEM_DelayMs(10);
                    APP_ModalBacklightTick(false);
                    if (wait && --wait == 0u)
                    {
                        held = true;
                        return key;
                    }
                }
                if (held)
                    continue;            /* end of a hold: reset by the idle poll below */
                return key;
            }
        }
        else
            held = false;
        SYSTEM_DelayMs(10);
    }
}

static void app_wait_release(void)
{
    uint8_t stable = 0;
    while (stable < 10u)
    {
        if (!GPIO_IsPttPressed() && KEYBOARD_Poll() == KEY_INVALID)
            stable++;
        else
            stable = 0;
        SYSTEM_DelayMs(10);
        APP_ModalBacklightTick(true);
    }
}

static void app_copy(char *dst, uint8_t cap, const char *src, uint8_t src_cap)
{
    uint8_t n = 0;
    while (n + 1u < cap && n < src_cap && src[n])
    {
        dst[n] = src[n];
        n++;
    }
    dst[n] = '\0';
}

/* Display the code payload rounded up to 0.1 KiB, so the compact value never
 * understates the space occupied by the app.  The space before "KB" is omitted
 * because this secondary value is rendered in the tiny 3x5 font. */
static void app_format_size(char out[6], uint32_t bytes)
{
    if (bytes > APP_OVERLAY_MAX)
    {
        memcpy(out, "--KB", 5u);
        return;
    }

    const uint16_t tenths = (uint16_t)((bytes * 10u + 1023u) / 1024u);
    out[0] = (char)('0' + tenths / 10u);
    out[1] = '.';
    out[2] = (char)('0' + tenths % 10u);
    out[3] = 'K';
    out[4] = 'B';
    out[5] = '\0';
}

/* Human-readable action for an APP_LaunchOverlay / APP_ValidateSlot failure. */
static const char *app_err_text(const app_header_t *header, uint8_t rc)
{
    switch (rc)
    {
        case APP_ERR_SLOT:          return "BAD SLOT";
        case APP_ERR_MAGIC:         return "NO APP";
        case APP_ERR_ABI:
            /* api_min == 0 also identifies pre-reset development blobs whose
             * former uint16_t ABI value occupies these two bytes. */
            if (header->api_min == 0u || header->abi_major < APP_ABI_MAJOR)
                return "UPDATE APP";
            return "UPDATE FIRMWARE";
        case APP_ERR_NOT_COMMITTED: return "REINSTALL APP";
        case APP_ERR_SIZE:          return "UPDATE APP";
        case APP_ERR_CRC:           return "REINSTALL APP";
        case APP_ERR_VMA:           return "UPDATE APP";
        case APP_ERR_AUTH:          return "AUTH";
        case APP_ERR_CAP:           return "NOT SUPPORTED";
        default:                    return "ERROR";
    }
}

/* A launch failed: name the app and the action to take, then wait for a key. Without this
 * an incompatible app would silently "do nothing" when selected. */
static void app_show_error(const app_header_t *header, uint8_t rc)
{
    char nm[19];
    app_copy(nm, sizeof(nm), header->name, APP_NAME_LEN);

    UI_DisplayClear();
    UI_StatusClear();
    GUI_DisplaySmallestInverse("APP ERROR", 46, 0, true, true, 82);
    UI_PrintStringSmallNormal(nm, 2, 0, 2);                 /* which app */
    UI_PrintStringSmallNormal(app_err_text(header, rc), 2, 0, 4); /* action */
    UI_PrintStringSmallNormal("Press any key", 2, 0, 6);
    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();

    app_get_key();   /* blocking: dismiss on any key */
}

/* Five visible apps; line 0 holds the separator and line 6 the key hints. */
#define APP_MENU_ROWS 5u
#define APP_MENU_SLOT_COUNT APP_SLOT_COUNT

_Static_assert(APP_MENU_SLOT_COUNT <= APP_SLOT_COUNT,
               "APP_MENU_SLOT_COUNT exceeds the physical app slot count");
_Static_assert(APP_MENU_SLOT_COUNT <= UINT8_MAX,
               "app menu slot indexes exceed uint8_t");

/* Entrees integrees (notre RP2040) en TETE de liste : A APRS-RP, B SARSAT-RP,
 * C SSTV-RP, D SONDE-RP, E GPS-RP. Les emplacements d'overlay suivent (leur numero
 * physique ne change pas, donc le telechargement des applications ne deplace
 * rien). Ces entrees n'ont pas de "slot" physique : elles sont toujours
 * presentes et toujours en premier, independamment de app_scan_slots(). */
#ifdef ENABLE_APRS
#define APP_MENU_APRS_IDX    0u
#define APP_MENU_EXTRA_APRS  1u
#else
#define APP_MENU_EXTRA_APRS  0u
#endif
#ifdef ENABLE_SARSAT
#define APP_MENU_SARSAT_IDX  APP_MENU_EXTRA_APRS
#define APP_MENU_EXTRA_SARSAT 1u
#else
#define APP_MENU_EXTRA_SARSAT 0u
#endif
#ifdef ENABLE_SSTV
#define APP_MENU_SSTV_IDX    (APP_MENU_EXTRA_APRS + APP_MENU_EXTRA_SARSAT)
#define APP_MENU_EXTRA_SSTV  1u
#else
#define APP_MENU_EXTRA_SSTV  0u
#endif
#ifdef ENABLE_SONDE
#define APP_MENU_SONDE_IDX   (APP_MENU_EXTRA_APRS + APP_MENU_EXTRA_SARSAT + APP_MENU_EXTRA_SSTV)
#define APP_MENU_EXTRA_SONDE 1u
#else
#define APP_MENU_EXTRA_SONDE 0u
#endif
/* GPS-RP reuses the GPS fix the C-Board already streams for APRS (s_gps in
 * aprs.c) -- same ENABLE_APRS gate, no separate feature flag. */
#ifdef ENABLE_APRS
#define APP_MENU_GPS_IDX     (APP_MENU_EXTRA_APRS + APP_MENU_EXTRA_SARSAT + APP_MENU_EXTRA_SSTV + APP_MENU_EXTRA_SONDE)
#define APP_MENU_EXTRA_GPS   1u
#else
#define APP_MENU_EXTRA_GPS   0u
#endif
#define APP_MENU_RP_COUNT    (APP_MENU_EXTRA_APRS + APP_MENU_EXTRA_SARSAT + APP_MENU_EXTRA_SSTV + APP_MENU_EXTRA_SONDE + APP_MENU_EXTRA_GPS)

static uint8_t app_scan_slots(uint8_t slots[APP_MENU_SLOT_COUNT])
{
    uint8_t count = 0u;
    for (uint8_t slot = 0; slot < APP_MENU_SLOT_COUNT; slot++)
    {
        app_header_t hdr;
        if (APP_SlotInfo(slot, &hdr) == APP_OK &&
            (hdr.flags & APP_FLAG_COMMITTED))
            slots[count++] = slot;
    }
    return count;
}

/* Dispatch a KEY_MENU on one of the RP2040 "integrated screen" entries
 * (items < APP_MENU_RP_COUNT); caller already checked the range. */
static void app_run_rp_entry(uint8_t item)
{
#ifdef ENABLE_APRS
    if (item == APP_MENU_APRS_IDX) { APP_RunAprs(); return; }
#endif
#ifdef ENABLE_SARSAT
    if (item == APP_MENU_SARSAT_IDX) { APP_RunSarsat(); return; }
#endif
#ifdef ENABLE_SSTV
    if (item == APP_MENU_SSTV_IDX) { APP_RunSstvRx(); return; }
#endif
#ifdef ENABLE_SONDE
    if (item == APP_MENU_SONDE_IDX) { APP_RunSonde(); return; }
#endif
#ifdef ENABLE_APRS
    if (item == APP_MENU_GPS_IDX) { APP_RunGps(); return; }
#endif
    (void)item;
}

/* Draw one of the RP2040 "integrated screen" rows (items < APP_MENU_RP_COUNT):
 * a letter (A, B, ...) instead of a slot number, name only, no size/version. */
static void app_draw_rp_entry(uint8_t item, uint8_t fbLine)
{
    char number[2] = { (char)('A' + item), '\0' };
    UI_PrintStringSmallNormal(number, 2u, 0, fbLine);
#ifdef ENABLE_APRS
    if (item == APP_MENU_APRS_IDX)
    { UI_PrintStringSmallNormal("APRS-RP", APP_NAME_TEXT_X, 0, fbLine); return; }
#endif
#ifdef ENABLE_SARSAT
    if (item == APP_MENU_SARSAT_IDX)
    { UI_PrintStringSmallNormal("SARSAT-RP", APP_NAME_TEXT_X, 0, fbLine); return; }
#endif
#ifdef ENABLE_SSTV
    if (item == APP_MENU_SSTV_IDX)
    { UI_PrintStringSmallNormal("SSTV-RP", APP_NAME_TEXT_X, 0, fbLine); return; }
#endif
#ifdef ENABLE_SONDE
    if (item == APP_MENU_SONDE_IDX)
    { UI_PrintStringSmallNormal("SONDE-RP", APP_NAME_TEXT_X, 0, fbLine); return; }
#endif
#ifdef ENABLE_APRS
    if (item == APP_MENU_GPS_IDX)
    { UI_PrintStringSmallNormal("GPS-RP", APP_NAME_TEXT_X, 0, fbLine); return; }
#endif
}

void APP_MenuOpen(void)
{
    APP_ModalScreenSaverExit();
    BACKLIGHT_TurnOn();

#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
    /* Detach the modal selector from the key state that triggered F+7. The
     * normal K5Viewer updater suppresses frames while a key is held; without
     * clearing this stale state, the selector could never publish its first
     * frame to the viewer. */
    gKeyReading0 = KEY_INVALID;
    gKeyReading1 = KEY_INVALID;
#endif

    /* Apps are installed from UV Studio (0x073x) into physical slots 0..N-1.
     * The radio is a launcher, so list only committed apps while preserving
     * their physical slot numbers for diagnostics and host-tool consistency.
     * The RP2040 "integrated screen" entries (A, B, ...) always come first,
     * regardless of what app_scan_slots() finds. */
    uint8_t installed_slots[APP_MENU_SLOT_COUNT];
    uint8_t slot_revision = APP_SlotRevision();
    uint8_t installed_count = app_scan_slots(installed_slots);
    uint8_t item_count = (uint8_t)(APP_MENU_RP_COUNT + installed_count);

    /* Combined selection index over [0, item_count): < APP_MENU_RP_COUNT is
     * one of the fixed RP2040 entries, the rest maps to installed_slots[]. */
    static uint8_t sel = 0u;
    if (sel >= item_count)
        sel = 0u;
    uint8_t top = sel >= APP_MENU_ROWS ? (uint8_t)(sel - APP_MENU_ROWS + 1u) : 0u;
    /* Right column: the code size, or the version after KEY_STAR. Kept across
     * menu openings, back to the size at power-on. */
    static bool show_version = false;
    app_wait_release();

    for (;;)
    {
        const uint8_t current_revision = APP_SlotRevision();
        if (current_revision != slot_revision)
        {
            installed_count = app_scan_slots(installed_slots);
            item_count = (uint8_t)(APP_MENU_RP_COUNT + installed_count);
            if (sel >= item_count)
                sel = item_count == 0u ? 0u : (uint8_t)(item_count - 1u);
            top = sel >= APP_MENU_ROWS ? (uint8_t)(sel - APP_MENU_ROWS + 1u) : 0u;
            slot_revision = current_revision;
        }

        UI_DisplayClear();
        app_status_bar();   /* also wipes the VFO status line (DW, battery, ...) */

        if (item_count == 0u)
        {
            UI_PrintStringSmallNormal("NO APPS", 0, LCD_WIDTH - 1u, 3);
        }
        else
        {
            /* Slide [top, top+APP_MENU_ROWS) so it contains the selection. */
            if (sel < top)
                top = sel;
            else if (sel >= (uint8_t)(top + APP_MENU_ROWS))
                top = (uint8_t)(sel - APP_MENU_ROWS + 1u);

            for (uint8_t index = top;
                 index < item_count && (uint8_t)(index - top) < APP_MENU_ROWS;
                 index++)
            {
                const uint8_t fbLine = (uint8_t)(index - top + 1u);

                if (index < APP_MENU_RP_COUNT)
                {
                    app_draw_rp_entry(index, fbLine);
                }
                else
                {
                    const uint8_t slot = installed_slots[index - APP_MENU_RP_COUNT];
                    const uint8_t visible_number = (uint8_t)(slot + 1u);
                    char number[3];
                    char name[14];
                    char size[6];

                    number[0] = (char)('0' + visible_number / 10u);
                    number[1] = (char)('0' + visible_number % 10u);
                    number[2] = '\0';

                    UI_PrintStringSmallNormal(number, 2u, 0, fbLine);
                    app_header_t hdr;
                    APP_SlotInfo(slot, &hdr);
                    app_copy(name, sizeof(name), hdr.name, APP_NAME_LEN);
                    if (show_version)
                    {   /* "v1.5": as wide as a size, so a long version is cut */
                        size[0] = 'v';
                        app_copy(size + 1, sizeof(size) - 1u, hdr.version, APP_VERSION_LEN);
                    }
                    else
                        app_format_size(size, hdr.code_size);
                    UI_PrintStringSmallNormal(name, APP_NAME_TEXT_X, 0, fbLine);
                    GUI_DisplaySmallest(size,
                                        (uint8_t)(LCD_WIDTH - 2u - strlen(size) * 4u),
                                        (uint8_t)(fbLine * 8u + 1u), false, true);
                }

                if (index == sel)
                    app_invert_name(fbLine);
            }
        }

        UI_DrawMenuKeyHints(item_count > 0u ? "RUN" : "", "QUIT");

        ST7565_BlitStatusLine();
        ST7565_BlitFullScreen();
#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
        K5VIEWER_Update(false);
#endif

        const KEY_Code_t key = app_get_key();
        if (key == KEY_EXIT)
            return;

        switch (key)
        {
            case KEY_UP:
                if (item_count > 0u)
                    sel = sel == 0u ? (uint8_t)(item_count - 1u)
                                   : (uint8_t)(sel - 1u);
                break;
            case KEY_DOWN:
                if (item_count > 0u)
                    sel = (uint8_t)((sel + 1u) % item_count);
                break;
            case KEY_STAR:
                show_version = !show_version;
                break;
            case KEY_MENU:
            {
                if (item_count == 0u)
                    break;

                if (sel < APP_MENU_RP_COUNT)
                {
                    app_run_rp_entry(sel);
                    app_wait_release();
                    break;
                }

                const uint8_t slot = installed_slots[sel - APP_MENU_RP_COUNT];
                app_header_t hdr;
                APP_SlotInfo(slot, &hdr);
                const uint8_t rc = APP_LaunchOverlay(slot);  /* runs until the app exits */
                if (rc != APP_OK)
                    app_show_error(&hdr, rc);               /* no longer silent */
                app_wait_release();
                /* Radio apps (BEAM, ...) hand back straight to the radio screen. */
                if (rc == APP_OK && (hdr.flags & APP_FLAG_EXIT_TO_MAIN))
                    return;
                break;
            }
            default:
                break;
        }
    }
}

#endif /* ENABLE_FEAT_F4HWN_OVERLAY_APPS */
