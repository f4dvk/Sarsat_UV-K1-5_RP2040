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
#include "app/sstv_tx.h"   /* SSTV_ClockStart / SSTV_ClockEnd */

#include <string.h>

#include "app/app.h"
#include "scheduler.h"       /* millis10() -- delai de l'ecran SSTVCal */
#include "app/uart.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "functions.h"
#include "radio.h"
#include "ui/helper.h"

static bool s_active;

void SSTV_ToggleRx(void) { s_active = !s_active; }
bool SSTV_RxActive(void) { return s_active; }

static bool s_dirty;     /* a row arrived (or the screen was cleared) since
                           * the last blit */
static bool s_got_row;   /* at least one row received this picture -- shows
                           * "waiting" instead of a blank screen beforehand */

_Static_assert(SSTV_RX_SCREEN_HEIGHT <= FRAME_LINES * 8u,
               "SSTV_RX_SCREEN_HEIGHT must fit in gFrameBuffer");

/* ---- Ecran de calibration de l'horloge (menu SSTVCal, 2026-10-04) ----
 * MENU : demande une mesure au RP2040 (30 s, sans emission) puis propose la
 * valeur ; MENU a nouveau = appliquer, EXIT = annuler. UP/DOWN = +-250 ppm
 * directement sur le trim courant (enregistre tout de suite). */
static volatile bool s_cal_res_ok;      /* resultat RP2040 recu */
static int16_t       s_cal_res;         /* proposition RP2040, ppm */

/* Petit formateur "libelle valeur ppm" : pas de sprintf (newlib sans _sbrk ici). */
static void sstv_cal_line(char *dst, const char *label, int v)
{
    char digits[8];
    int  n = 0;
    unsigned u = (v < 0) ? (unsigned)(-v) : (unsigned)v;
    do { digits[n++] = (char)('0' + (u % 10u)); u /= 10u; } while (u && n < 7);
    int k = 0;
    while (label[k]) { dst[k] = label[k]; k++; }
    if (v < 0) dst[k++] = '-';
    while (n) dst[k++] = digits[--n];
    dst[k++] = ' ';
    dst[k++] = 'p'; dst[k++] = 'p'; dst[k++] = 'm';
    dst[k] = 0;
}

static void sstv_cal_draw(int state, int16_t trim, int16_t prop)
{
    char line[24];
    UI_DisplayClear();
    UI_PrintStringSmallNormal("Calibr. horloge", 2, 0, 0);
    sstv_cal_line(line, "Trim: ", (int)trim);
    UI_PrintStringSmallNormal(line, 2, 0, 2);
    if (state == 0) {
        UI_PrintStringSmallNormal("MENU = mesurer", 2, 0, 4);
        UI_PrintStringSmallNormal("UP/DN = +-250", 2, 0, 5);
        UI_PrintStringSmallNormal("EXIT = quitter", 2, 0, 6);
    } else if (state == 1) {
        UI_PrintStringSmallNormal("mesure 30 s...", 2, 0, 4);
        UI_PrintStringSmallNormal("ne pas emettre", 2, 0, 5);
    } else {
        sstv_cal_line(line, "Propose: ", (int)prop);
        UI_PrintStringSmallNormal(line, 2, 0, 4);
        UI_PrintStringSmallNormal("MENU = appliquer", 2, 0, 5);
        UI_PrintStringSmallNormal("EXIT = annuler", 2, 0, 6);
    }
    ST7565_BlitFullScreen();
}

void APP_RunSstvCal(void)
{
    enum { ST_IDLE, ST_WAIT, ST_RESULT };
    int     state = ST_IDLE;
    SSTV_LoadTrim();                     /* valeur reelle de l'EEPROM, pas le defaut RAM */
    int16_t trim  = SSTV_GetTrim();
    uint32_t t_req = 0;
    s_cal_res_ok = false;
    sstv_cal_draw(state, trim, 0);
    /* La touche qui a ouvert l'ecran est encore enfoncee : on ne la compte pas.
     * Une touche maintenue ne doit declencher qu'une fois (front montant). */
    int prev_key = KEYBOARD_Poll();
    /* Anti-rebond : une pression peut etre lue deux fois de suite par le clavier.
     * Apres chaque changement d'etat, les touches sont ignorees 500 ms. */
    int      prev_state = state;
    uint32_t t_change   = millis10();

    bool run = true;
    while (run) {
#ifdef ENABLE_UART
        while (UART_IsCommandAvailable(UART_PORT_UART))
            UART_HandleCommand(UART_PORT_UART);
#endif
        if (state == ST_WAIT && s_cal_res_ok) {
            s_cal_res_ok = false;
            state = ST_RESULT;
            sstv_cal_draw(state, trim, s_cal_res);
        } else if (state == ST_WAIT && (millis10() - t_req) > 6000u) {   /* 60 s */
            state = ST_IDLE;     /* pas de reponse du RP2040 : on abandonne */
            sstv_cal_draw(state, trim, 0);
        }

        if (state != prev_state) {
            prev_state = state;
            t_change   = millis10();
        }
        int key = KEYBOARD_Poll();
        int ev  = (key == prev_key) ? 0 : key;   /* front, pas niveau */
        prev_key = key;
        if ((millis10() - t_change) < 50u)       /* anti-rebond, 500 ms */
            ev = 0;
        if (ev == KEY_EXIT) {
            run = false;
        } else if (state == ST_IDLE && ev == KEY_MENU) {
            uint8_t req[4] = { (uint8_t)(SSTV_CMD_CLK_REQ & 0xFFu), (uint8_t)(SSTV_CMD_CLK_REQ >> 8), 0, 0 };
            SendReply(UART_PORT_UART, req, sizeof req);
            t_req = millis10();
            state = ST_WAIT;
            sstv_cal_draw(state, trim, 0);
        } else if (state == ST_RESULT && ev == KEY_MENU) {
            trim = s_cal_res;
            SSTV_SetTrim(trim);
            state = ST_IDLE;
            sstv_cal_draw(state, trim, 0);
        } else if (state == ST_RESULT && ev == KEY_EXIT) {
            state = ST_IDLE;
            sstv_cal_draw(state, trim, 0);
        } else if (state == ST_IDLE && (ev == KEY_UP || ev == KEY_DOWN)) {
            trim = (int16_t)(trim + (ev == KEY_UP ? 250 : -250));
            SSTV_SetTrim(trim);
            sstv_cal_draw(state, trim, 0);
        }
        SYSTEM_DelayMs(20);
    }
}

void SSTV_HandleUART(uint16_t id, const uint8_t *data, uint16_t size)
{
    switch (id) {
    case SSTV_CMD_CLK_RESULT:
        if (size >= 2u) {
            s_cal_res    = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
            s_cal_res_ok = true;
        }
        break;

    case SSTV_CMD_CLK_START:
        SSTV_ClockStart();
        break;

    case SSTV_CMD_CLK_END:
        SSTV_ClockEnd();
        break;

    case SSTV_CMD_TRIM:
        if (size >= 2u)
            SSTV_SetTrim((int16_t)(data[0] | ((uint16_t)data[1] << 8)));
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

    bool run = true;
    while (run) {
#ifdef ENABLE_UART
        while (UART_IsCommandAvailable(UART_PORT_UART))
            UART_HandleCommand(UART_PORT_UART);
#endif
        if (s_dirty) {
            s_dirty = false;
            if (!s_got_row)
                /* row 0 (not the middle): the first real image row overwrites
                 * it almost immediately once a picture starts arriving, so
                 * this never lingers as a visual artefact over the image. */
                UI_PrintStringSmallNormal("waiting for a picture...", 2, 0, 0);
            ST7565_BlitFullScreen();
        }
        if (KEYBOARD_Poll() == KEY_EXIT)
            run = false;
        else
            SYSTEM_DelayMs(20);   /* no PA4/ADC capture happens on this side
                                   * (the RP2040 does it) -- no reason to
                                   * throttle key polling here, unlike
                                   * aprs.c's own internal-decode screen */
    }

    if (s_active)
        SSTV_ToggleRx();   /* disarm on close -- see the note above */

    FUNCTION_Select(FUNCTION_FOREGROUND);
    RADIO_SetupRegisters(true);
}
