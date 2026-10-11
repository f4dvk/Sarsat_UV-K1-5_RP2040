/* See sstv_rx.h. No BK4819 register writes needed -- RX is entirely
 * software on the RP2040 side (sstv_demod.h), reading the same FM-
 * discriminator audio tap as SARSAT/APRS/Sonde. This side only needs to
 * flip a flag the RP2040 can see (patch/sarsat.c's screen_state byte).
 *
 * (2026-10-11) Folded into one screen, reachable only from the F+7 launcher
 * (D SSTV-RP) -- same model as f4hwn-v6's own SSTV-RP screen, which this
 * one is ported from: information view (TX mode + image source in
 * inverted capsules, help line at the bottom), image view (key 4), key 1
 * cycles the TX mode, key 2 toggles the image source, MENU sends, EXIT
 * backs out of the image view then closes. Replaces the old split between
 * this RX-only screen and six separate F1/F2-assignable SSTV1/M1/PD90/
 * PD120/SSTVRx/SSTVImg actions -- one entry point from here on, like
 * SARSAT/APRS/SONDE already are. The "page 5" reception-stats view (clock
 * ppm drift, line/sync counters) this replaced is dropped, same call
 * f4hwn-v6 already made: SSTV_CMD_RX_INFO is still defined (sstv_rx.h) for
 * wire compatibility but no longer handled here. */
#include "app/sstv_rx.h"
#include "app/sstv_tx.h"

#include <string.h>

#include "app/app.h"
#include "app/uart.h"
#include "driver/bk4819.h"     /* BK4819_REG_2B : filtres audio RX (ecran SSTVRx) */
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "functions.h"
#include "radio.h"
#include "ui/helper.h"

/* made non-static by patch/app_uart.c.diff */
extern void SendReply(uint32_t Port, void *pReply, uint16_t Size);

static bool s_active;

void SSTV_ToggleRx(void) { s_active = !s_active; }
bool SSTV_RxActive(void) { return s_active; }

static bool     s_dirty;          /* une ligne image est arrivee (vue image) */
static bool     s_info_redraw;    /* l'ecran d'information doit etre redessine */
static bool     s_image_view;     /* vue image (touche 4) ; sinon information */
static uint16_t s_saved_2b;       /* registre 0x2B d'origine (filtres audio) */
static uint8_t  s_tx_mode;        /* 0 Scottie 1, 1 Martin 1, 2 PD90, 3 PD120 */

_Static_assert(SSTV_RX_SCREEN_HEIGHT <= FRAME_LINES * 8u,
               "SSTV_RX_SCREEN_HEIGHT must fit in gFrameBuffer");

static const char *const s_mode_name[4]  = { "Scottie 1", "Martin 1", "PD90", "PD120" };

static void sstv_tx_run(void)
{
    switch (s_tx_mode) {
    case 1:  SSTV_SendMartin1(); break;
    case 2:  SSTV_SendPD90();    break;
    case 3:  SSTV_SendPD120();   break;
    default: SSTV_SendScottie1(); break;
    }
}

/* Ecran d'information : pas de sprintf (newlib sans _sbrk dans cette base),
 * chaines courtes (16 caracteres au plus par ligne). */
static void sstv_info_draw(void)
{
    const bool logo = strcmp(SSTV_ImageName(), "logo") == 0;

    char mode[20] = "1: ", src[12] = "2: ";
    strcat(mode, s_mode_name[s_tx_mode & 3u]);            /* touche 1 : mode courant */
    strcat(src, logo ? "Logo" : "Photo");                 /* touche 2 : source courante */

    UI_DisplayClear();
    UI_PrintStringSmallNormal(mode, 2, 0, 0);             /* a la place de "Mode" */
    UI_PrintStringSmallNormal(src, 72, 0, 0);             /* a la place de "source" */

    /* aide en bas, petite police */
    GUI_DisplaySmallest("MENU send 1mode 2src 4view", 0, 49, false, true);

    ST7565_BlitFullScreen();
}

/* Demande au RP2040 de renvoyer l'image qu'il a gardee */
static void sstv_request_image(void)
{
    uint8_t rq[4] = { (uint8_t)(SSTV_CMD_RX_REDRAW & 0xFFu), (uint8_t)(SSTV_CMD_RX_REDRAW >> 8), 0, 0 };
    SendReply(UART_PORT_UART, rq, sizeof rq);
}

void APP_RunSstvRx(void)
{
    /* Le RX reste arme tant que l'ecran est ouvert, et est desarme a la
     * fermeture (sinon le RP2040 reste en mode SSTV et bloque APRS/SARSAT). */
    if (!s_active)
        SSTV_ToggleRx();

    APP_StartListening(FUNCTION_MONITOR);   /* un-mute the speaker: hearing the
                                              * SSTV tones is useful confirmation */

    /* Filtres audio RX desactives pendant la reception SSTV : le passe-haut
     * 300 Hz et le passe-bas 3 kHz attenuent la synchro 1200 Hz et les pixels. */
    s_saved_2b = BK4819_ReadRegister(BK4819_REG_2B);
    BK4819_WriteRegister(BK4819_REG_2B, s_saved_2b | (1u << 10) | (1u << 9));

    s_image_view  = false;
    s_info_redraw = true;
    s_dirty       = false;

    int prev_key = KEYBOARD_Poll();          /* la touche qui a ouvert l'ecran ne compte pas */
    bool run = true;
    while (run) {
#ifdef ENABLE_UART
        while (UART_IsCommandAvailable(UART_PORT_UART))
            UART_HandleCommand(UART_PORT_UART);
#endif
        if (s_image_view) {
            if (s_dirty) {
                s_dirty = false;
                ST7565_BlitFullScreen();
            }
        } else if (s_info_redraw) {
            s_info_redraw = false;
            sstv_info_draw();
        }

        int key = KEYBOARD_Poll();
        int ev  = (key == prev_key) ? 0 : key;   /* front, pas niveau */
        prev_key = key;

        if (ev == KEY_EXIT) {
            if (s_image_view) {                  /* image -> information */
                s_image_view  = false;
                s_info_redraw = true;
            } else {
                run = false;
            }
        } else if (ev == KEY_1) {                /* mode TX suivant */
            s_tx_mode = (uint8_t)((s_tx_mode + 1u) & 3u);
            s_info_redraw = true;
        } else if (ev == KEY_2) {                /* source photo / logo */
            SSTV_ToggleImage();                  /* confirmation 600 ms */
            s_info_redraw = true;
        } else if (ev == KEY_4) {                /* image <-> information */
            if (s_image_view) {
                s_image_view  = false;
                s_info_redraw = true;
            } else {
                s_image_view = true;
                UI_DisplayClear();               /* pas d'info sous l'image */
                sstv_request_image();
                s_dirty = true;
            }
        } else if (ev == KEY_MENU) {             /* emettre */
            sstv_tx_run();                       /* bloquant, EXIT abandonne l'envoi */
            if (s_image_view) {
                UI_DisplayClear();
                sstv_request_image();
                s_dirty = true;
            } else {
                s_info_redraw = true;
            }
        }
        SYSTEM_DelayMs(20);
    }

    if (s_active)
        SSTV_ToggleRx();                         /* desarme a la fermeture */

    BK4819_WriteRegister(BK4819_REG_2B, s_saved_2b);   /* filtres audio d'origine */
    FUNCTION_Select(FUNCTION_FOREGROUND);
    RADIO_SetupRegisters(true);
}

void SSTV_HandleUART(uint16_t id, const uint8_t *data, uint16_t size)
{
    switch (id) {
    case SSTV_CMD_RX_CLEAR:
        /* une image commence : tout l'ecran est pour elle (vue image automatique) */
        memset(gFrameBuffer, 0, sizeof(gFrameBuffer[0][0]) * FRAME_LINES * SSTV_RX_SCREEN_WIDTH);
        s_image_view = true;
        s_dirty      = true;
        break;

    case SSTV_CMD_RX_ROW: {
        if (size < 1u + SSTV_RX_ROW_BYTES)
            return;
        uint8_t row = data[0];
        if (row >= SSTV_RX_SCREEN_HEIGHT)
            return;
        /* gFrameBuffer est en colonnes ST7565 : 8 pixels verticaux par octet,
         * FRAME_LINES pages empilees (driver/st7565.h). 1 = pixel fonce, comme
         * rp2040/src/main.c. */
        uint8_t page = (uint8_t)(row >> 3), bit = (uint8_t)(1u << (row & 7u));
        for (uint16_t x = 0; x < SSTV_RX_SCREEN_WIDTH; x++) {
            bool on = (data[1u + (x >> 3)] & (0x80u >> (x & 7u))) != 0;
            if (on) gFrameBuffer[page][x] |= bit;
            else    gFrameBuffer[page][x] &= (uint8_t)~bit;
        }
        s_dirty = true;
        break;
    }
    default:
        /* SSTV_CMD_RX_INFO (statistiques de calibration) : plus utilise */
        break;
    }
}
