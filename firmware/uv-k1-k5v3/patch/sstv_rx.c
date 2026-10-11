/* See sstv_rx.h. No BK4819 register writes needed -- RX is entirely
 * software on the RP2040 side (sstv_demod.h), reading the same FM-
 * discriminator audio tap as SARSAT/APRS/Sonde. This side only needs to
 * flip a flag the RP2040 can see (patch/sarsat.c's screen_state byte).
 *
 * ⚠️ (2026-10-03) live display, added on top of the pre-existing arm/disarm
 * toggle: the RP2040 decodes a real incoming picture (sstv_demod.c) and now
 * streams it back here, screen row by screen row (CMD_SSTV_RX_CLEAR/
 * CMD_SSTV_RX_ROW, see this header and rp2040/src/decoder_config.h for the
 * full design and wire format), so the operator can watch it build up on
 * the radio's own LCD -- same idea as F4HWN's own on-radio SSTV app
 * (uv-k1-k5v3-firmware-custom, feature_update_v6), except the heavy
 * DSP (demod, resampling, dithering) stays on the RP2040, which has far
 * more flash/RAM headroom than this MCU -- this screen only draws the
 * already-reduced 1bpp bytes it is handed. */
#include "app/sstv_rx.h"

#include <string.h>

#include "app/app.h"
#include "app/uart.h"
#include "driver/keyboard.h"
#include "driver/bk4819.h"     /* BK4819_REG_2B : filtres audio RX (ecran SSTVRx) */
#include "driver/st7565.h"
#include "driver/system.h"
#include "functions.h"
#include "radio.h"
#include "ui/helper.h"

static bool s_active;

/* made non-static by patch/app_uart.c.diff */
extern void SendReply(uint32_t Port, void *pReply, uint16_t Size);

void SSTV_ToggleRx(void) { s_active = !s_active; }
bool SSTV_RxActive(void) { return s_active; }

static bool s_dirty;     /* a row arrived (or the screen was cleared) since
                           * the last blit */
static bool s_got_row;   /* at least one row received this picture -- shows
                           * "waiting" instead of a blank screen beforehand */

_Static_assert(SSTV_RX_SCREEN_HEIGHT <= FRAME_LINES * 8u,
               "SSTV_RX_SCREEN_HEIGHT must fit in gFrameBuffer");

static uint16_t      s_saved_2b;        /* registre 0x2B d'origine (filtres audio) */

/* Petit formateur "libelle valeur ppm" : pas de sprintf (newlib sans _sbrk ici). */
static void sstv_cal_line(char *dst, const char *label, int v, const char *suffix)
{
    char digits[8];
    int  n = 0;
    unsigned u = (v < 0) ? (unsigned)(-v) : (unsigned)v;
    do { digits[n++] = (char)('0' + (u % 10u)); u /= 10u; } while (u && n < 7);
    int k = 0;
    while (label[k]) { dst[k] = label[k]; k++; }
    if (v < 0) dst[k++] = '-';
    while (n) dst[k++] = digits[--n];
    while (*suffix) dst[k++] = *suffix++;
    dst[k] = 0;
}

/* ---- Page "5" de l'ecran SSTVRx : informations de reception ---- */
static bool     s_info_view;     /* page informations affichee */
static bool     s_info_ok;       /* au moins une mesure recue du RP2040 */
static uint8_t  s_info_mode;     /* 1 = Scottie 1 */
static uint8_t  s_info_flags;    /* bit0 valide, bit1 final */
static uint16_t s_info_lines;    /* lignes recues */
static uint16_t s_info_iv;       /* synchros retenues */
static int32_t  s_info_ppm;      /* ecart emetteur distant */

static void sstv_info_draw(void)
{
    char line[24];
    UI_DisplayClear();
    UI_PrintStringSmallNormal("RX  [5]=retour", 2, 0, 0);
    UI_PrintStringSmallNormal(s_info_mode == 1 ? "Mode: Scottie 1" : "Mode: inconnu", 2, 0, 1);
    sstv_cal_line(line, "Lignes: ", (int)s_info_lines, "/256");
    UI_PrintStringSmallNormal(line, 2, 0, 2);
    sstv_cal_line(line, "Synchros: ", (int)s_info_iv, "");
    UI_PrintStringSmallNormal(line, 2, 0, 3);
    if (!s_info_ok) {
        UI_PrintStringSmallNormal("Horloge: attente", 2, 0, 4);
    } else {
        UI_PrintStringSmallNormal((s_info_flags & 2u) ? "Horloge: FINAL" : "Horloge: provis.", 2, 0, 4);
        sstv_cal_line(line, "Ecart: ", (int)s_info_ppm, " ppm");
        UI_PrintStringSmallNormal(line, 2, 0, 5);
        sstv_cal_line(line, "TX distant: ", -(int)s_info_ppm, " ppm");
        UI_PrintStringSmallNormal(line, 2, 0, 6);
    }
    ST7565_BlitFullScreen();
}

void SSTV_HandleUART(uint16_t id, const uint8_t *data, uint16_t size)
{
    switch (id) {
    case SSTV_CMD_RX_INFO:
        if (size >= 10u) {
            s_info_mode  = data[0];
            s_info_flags = data[1];
            s_info_lines = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
            s_info_iv    = (uint16_t)(data[4] | ((uint16_t)data[5] << 8));
            s_info_ppm   = (int32_t)((uint32_t)data[6] | ((uint32_t)data[7] << 8) |
                                     ((uint32_t)data[8] << 16) | ((uint32_t)data[9] << 24));
            s_info_ok    = (s_info_flags & 1u) != 0u;
            s_dirty      = true;
        }
        break;

    case SSTV_CMD_RX_CLEAR:
        memset(gFrameBuffer, 0, sizeof(gFrameBuffer[0][0]) * FRAME_LINES * SSTV_RX_SCREEN_WIDTH);
        s_got_row = false;
        s_dirty   = true;
        break;

    case SSTV_CMD_RX_ROW: {
        if (size < 1u + SSTV_RX_ROW_BYTES)
            return;
        uint8_t row = data[0];
        if (row >= SSTV_RX_SCREEN_HEIGHT)
            return;
        /* gFrameBuffer is ST7565 page/column packed: 8 vertical pixels per
         * byte, one byte per column, FRAME_LINES pages stacked top to
         * bottom (see driver/st7565.h) -- a single incoming PIXEL ROW spans
         * one bit position across all SSTV_RX_SCREEN_WIDTH column bytes of
         * one page, not a contiguous run of bytes like a page-aligned blit
         * (aprs.c's draw_icon()) would be. 1 = dark pixel (set bit), see
         * rp2040/src/main.c's sstv_rx_emit_row() for the TX-side convention
         * this must match. */
        uint8_t page = (uint8_t)(row >> 3), bit = (uint8_t)(1u << (row & 7u));
        for (uint16_t x = 0; x < SSTV_RX_SCREEN_WIDTH; x++) {
            bool on = (data[1u + (x >> 3)] & (0x80u >> (x & 7u))) != 0;
            if (on) gFrameBuffer[page][x] |= bit;
            else    gFrameBuffer[page][x] &= (uint8_t)~bit;
        }
        s_got_row = true;
        s_dirty   = true;
        break;
    }
    default:
        break;
    }
}

void APP_RunSstvRx(void)
{
    /* ⚠️ (2026-10-03, retour terrain : "le mode reste SSTV meme apres avoir
     * quitte le menu", qui coupait aussi la detection APRS/SARSAT au
     * passage) -- contrairement a APRS, le RP2040 ne peut tourner que dans
     * UN SEUL mode a la fois (SSTV partage le meme tampon ADC qu'APRS, voir
     * rp2040/src/main.c) : rester arme en arriere-plan apres la fermeture
     * de cet ecran (le modele "APRS continue en fond" copie au depart)
     * bloquait donc le RP2040 en mode SSTV indefiniment, empechant APRS/
     * SARSAT de redevenir actifs tant que l'operateur ne rouvrait pas cet
     * ecran pour le desarmer manuellement -- sauf qu'une fois arme,
     * SSTV_ToggleRx() n'etait plus jamais rappele, donc aucun moyen de le
     * desarmer du tout. Desarme maintenant explicitement a la fermeture. */
    if (!s_active)
        SSTV_ToggleRx();   /* ensure armed -- see SSTV_RxActive()'s own doc */

    APP_StartListening(FUNCTION_MONITOR);   /* un-mute the speaker: hearing the
                                              * SSTV tones is useful confirmation,
                                              * same reasoning as APRS/SARSAT's
                                              * manual open */
    UI_DisplayClear();
    s_dirty = true;

    /* Filtres audio RX desactives pendant la reception SSTV : le passe-haut
     * 300 Hz et le passe-bas 3 kHz de la radio attenuent la synchro 1200 Hz
     * et l'enveloppe des pixels. Restaures a la sortie de l'ecran. */
    s_saved_2b = BK4819_ReadRegister(BK4819_REG_2B);
    BK4819_WriteRegister(BK4819_REG_2B, s_saved_2b | (1u << 10) | (1u << 9));

    int prev_key = KEYBOARD_Poll();          /* la touche qui a ouvert l'ecran ne compte pas */
    bool run = true;
    while (run) {
#ifdef ENABLE_UART
        while (UART_IsCommandAvailable(UART_PORT_UART))
            UART_HandleCommand(UART_PORT_UART);
#endif
        if (s_dirty && s_info_view) {
            s_dirty = false;
            sstv_info_draw();
        } else if (s_dirty) {
            s_dirty = false;
            if (!s_got_row)
                /* row 0 (not the middle): the first real image row overwrites
                 * it almost immediately once a picture starts arriving, so
                 * this never lingers as a visual artefact over the image. */
                UI_PrintStringSmallNormal("waiting for a picture...", 2, 0, 0);
            ST7565_BlitFullScreen();
        }
        int key = KEYBOARD_Poll();
        int ev  = (key == prev_key) ? 0 : key;   /* front, pas niveau */
        prev_key = key;
        if (ev == KEY_EXIT) {
            run = false;
        } else if (ev == KEY_5) {
            s_info_view = !s_info_view;          /* page informations */
            if (!s_info_view) {
                /* retour sur l'image : le RP2040 la renvoie (son tampon l'a gardee) */
                uint8_t rq[4] = { (uint8_t)(SSTV_CMD_RX_REDRAW & 0xFFu), (uint8_t)(SSTV_CMD_RX_REDRAW >> 8), 0, 0 };
                SendReply(UART_PORT_UART, rq, sizeof rq);
            }
            UI_DisplayClear();
            s_dirty = true;
            SYSTEM_DelayMs(20);
        } else {
            SYSTEM_DelayMs(20);   /* no PA4/ADC capture happens on this side
                                   * (the RP2040 does it) -- no reason to
                                   * throttle key polling here, unlike
                                   * aprs.c's own internal-decode screen */
        }
    }

    if (s_active)
        SSTV_ToggleRx();   /* disarm on close -- see the note above */

    BK4819_WriteRegister(BK4819_REG_2B, s_saved_2b);   /* filtres audio d'origine */
    FUNCTION_Select(FUNCTION_FOREGROUND);
    RADIO_SetupRegisters(true);
}
