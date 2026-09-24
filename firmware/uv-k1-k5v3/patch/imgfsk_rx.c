/* See imgfsk_rx.h. K1/K5 V3 (PY32F071 + BK4829) only. */
#include <stdint.h>
#include <string.h>

#include "app/imgfsk_rx.h"
#include "app/uart.h"

#include "driver/bk4819.h"
#include "functions.h"
#include "radio.h"
#include "settings.h"

extern void SendReply(uint32_t Port, void *pReply, uint16_t Size);

/* ⚠️ (2026-09-25) First RX design (branch SSTV_SSDV) tried to demodulate
 * from the RP2040 side instead, off the same RAW/DSC audio feed SARSAT/
 * Sonde use -- reverted (see decoder_config.h's CMD_IMGFSK_RXPKT comment):
 * it needed a second radio+C-Board pair just to test, and the operator
 * explicitly asked for hardware demod on the BK4829 instead, which this
 * does. Register recipe mirrors imgfsk_tx.c's provenance comment: proven
 * AirCopy values (App/driver/bk4829.c's BK4819_SetupAircopy() /
 * BK4819_PrepareFSKReceive()) with only REG_58 (RX bandwidth: 1.2K vs 2.4K)
 * and REG_5D (256-byte SSDV packet length instead of AirCopy's 72 bytes)
 * changed -- REG_70/72/5C are AirCopy's own values, kept as-is rather than
 * guessed at, since RX has zero on-air validation yet and TX already
 * proved that deviating from the proven baseline (REG_58 first, then the
 * REG_59 scramble bit) is exactly where new bugs show up. */

static bool    s_armed;
static bool    s_fsk2400;
static uint8_t s_pkt[256];
static int     s_widx;   /* words accumulated so far (0..128), see
                          * IMGFSK_TimeSlice()'s comment */
static uint8_t s_saved_battery_save;
static uint8_t s_saved_dual_watch;
static uint8_t s_saved_bandwidth;

/* ⚠️ (2026-09-25, retour terrain : AirCopy stock confirmé fonctionnel entre
 * les deux mêmes postes -- donc le moteur matériel marche vraiment, le bug
 * est spécifique à ce fichier) -- comparé à la vraie séquence d'entrée
 * d'AirCopy (App/helper/boot.c, BOOT_MODE_AIRCOPY) : elle désactive
 * explicitement l'économie de batterie et le dual-watch avant d'armer quoi
 * que ce soit. Absent ici jusqu'à présent -- si l'économie de batterie est
 * active (réglage courant par défaut sur ces radios), le récepteur coupe
 * périodiquement l'écoute pour économiser l'énergie : le corrélateur FSK ne
 * verrait le signal qu'une fraction du temps, potentiellement jamais. Même
 * classe de problème déjà rencontrée et corrigée côté APRS
 * (APRS_KeepAwake(), aprs.h) pour la même raison. Sauvegardés/restaurés
 * plutôt que simplement écrasés, pour ne pas modifier silencieusement les
 * réglages de l'opérateur après un simple test. */
static void imgfsk_rx_arm(bool fsk2400)
{
    s_saved_battery_save = gEeprom.BATTERY_SAVE;
    s_saved_dual_watch   = gEeprom.DUAL_WATCH;
    s_saved_bandwidth    = gRxVfo->CHANNEL_BANDWIDTH;
    gEeprom.BATTERY_SAVE     = 0;
    gEeprom.DUAL_WATCH       = DUAL_WATCH_OFF;
    gRxVfo->CHANNEL_BANDWIDTH = BANDWIDTH_NARROW;   /* see the NARROW comment
                                                    * below -- set on the VFO
                                                    * itself, not just the
                                                    * register, matching
                                                    * AirCopy's own sequence
                                                    * (boot.c) exactly, in
                                                    * case RADIO_SetupRegisters()
                                                    * derives anything else
                                                    * (squelch/AGC) from it */

    RADIO_SetupRegisters(true);   /* normal RX for the currently tuned channel */

    /* ⚠️ (2026-09-25, retour terrain : LED rouge fixe, aucune réaction --
     * pas le moindre flash même pendant une vraie transmission de l'autre
     * poste) -- REG_2B écarté (filtre audio, sans rapport avec le moteur
     * FSK numérique, objection justifiée de l'opérateur). Deuxième piste,
     * trouvée en relisant la séquence d'entrée COMPLÈTE d'AirCopy
     * (App/helper/boot.c, BOOT_MODE_AIRCOPY) plus attentivement : elle
     * force `gRxVfo->CHANNEL_BANDWIDTH = BANDWIDTH_NARROW` -- PAS WIDE. Le
     * WIDE ci-dessous avait été copié par analogie avec l'écran SARSAT (un
     * signal différent, bi-phase-L analogique), sans preuve que ça
     * s'applique au moteur FSK numérique -- et on a maintenant la preuve du
     * contraire : la seule configuration confirmée fonctionnelle (AirCopy
     * stock, entre ces deux mêmes postes) utilise NARROW. Un mauvais
     * réglage de filtre IF expliquerait un silence total et immédiat,
     * exactement le symptôme observé. */
    RADIO_SetModulation(MODULATION_FM);
    BK4819_SetFilterBandwidth(BK4819_FILTER_BW_NARROW, true);

    BK4819_WriteRegister(BK4819_REG_70, 0x00C3u);   /* AirCopy's own value    */
    BK4819_WriteRegister(BK4819_REG_72, 0x3065u);   /* AirCopy's own value    */
    /* REG_58: RX mode = FSK1.2K/2.4K (bits<12:10>=000, AirCopy's own family),
     * RX gain = 3 (bits<9:8>, AirCopy's own value), bandwidth bits<3:1>:
     * 000=1.2K (AirCopy's own 0x00C1 unchanged) / 100=2.4K, enable bit0=1. */
    BK4819_WriteRegister(BK4819_REG_58, fsk2400 ? 0x00C9u : 0x00C1u);
    BK4819_WriteRegister(BK4819_REG_5C, 0x5665u);   /* AirCopy's own value    */
    BK4819_WriteRegister(BK4819_REG_5D, 0xFF00u);   /* 256 B (255<<8), see
                                                     * imgfsk_tx.c's comment  */
    BK4819_WriteRegister(0x5E, 0x3204u);            /* AirCopy's own value    */

    BK4819_PrepareFSKReceive();   /* proven: ResetFSK + RX_TurnOn + IRQ mask +
                                   * preamble/sync -- see App/driver/bk4829.c */
    s_armed   = true;
    s_fsk2400 = fsk2400;
    s_widx    = 0;
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);   /* see disarm's comment */
}

static void imgfsk_rx_disarm(void)
{
    s_armed = false;
    gEeprom.BATTERY_SAVE      = s_saved_battery_save;
    gEeprom.DUAL_WATCH        = s_saved_dual_watch;
    gRxVfo->CHANNEL_BANDWIDTH = s_saved_bandwidth;
    BK4819_ResetFSK();
    RADIO_SetupRegisters(true);
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
}

/* ⚠️ (2026-09-25, retour terrain : "pas de réaction" en armant, puis "pas de
 * bip car le C-Board coupe l'audio, il faut du visuel") -- un bip ne pouvait
 * pas marcher tant que le C-Board est branché (il capte/coupe l'audio pour
 * son propre décodage). Repli sur la LED rouge (normalement TX uniquement,
 * jamais allumée au repos en RX -- aucune ambiguïté possible ici) : allumée
 * en continu tant que l'écoute est armée, éteinte au désarmement -- signal
 * persistant, pas juste un flash qu'on peut rater. */
void IMGFSK_ToggleRx1200(void)
{
    if (s_armed && !s_fsk2400) imgfsk_rx_disarm();
    else                       imgfsk_rx_arm(false);
}

void IMGFSK_ToggleRx2400(void)
{
    if (s_armed && s_fsk2400) imgfsk_rx_disarm();
    else                      imgfsk_rx_arm(true);
}

static void imgfsk_rx_forward(const uint8_t *pkt)
{
    uint8_t b[4 + 256];
    b[0] = 0xE2; b[1] = 0x06;   /* CMD_IMGFSK_RXPKT, 0x06E2 LE -- keep in
                                * sync with rp2040/src/decoder_config.h */
    b[2] = 0x00; b[3] = 0x01;   /* size = 256 LE */
    memcpy(b + 4, pkt, 256);
    SendReply(UART_PORT_UART, b, sizeof(b));
}

/* ⚠️ (2026-09-25, retour terrain : LED armée, mais rien décodé) -- comparé
 * au code réel d'AirCopy (App/app/app.c, CheckRadioInterrupts()) : la FIFO
 * matérielle ne fait que 8 mots (REG_5E), pas assez pour 128 mots (256 o)
 * d'un coup. AirCopy ne lit JAMAIS tout en une fois -- il draine exactement
 * 4 mots à chaque interruption "FIFO presque pleine" (bit 12 de REG_02,
 * REG_5E réglé pour ce seuil), en boucle, et ne traite le tampon qu'une
 * fois les 36 mots (72 o, sa taille de paquet) accumulés. La version
 * précédente lisait 128 mots dès la première interruption vue -- la
 * quasi-totalité aurait été du bruit/valeurs périmées, pas de vraies
 * données, même symptôme que la tentative AFSK abandonnée. Repris ici à
 * l'identique, juste avec 128 mots (256 o, taille SSDV) au lieu de 36. */
#define IMGFSK_IRQ_FIFO_ALMOST_FULL (1u << 12)   /* REG_02 bit 12, App/app/app.c */

void IMGFSK_TimeSlice(void)
{
    static bool was_tx;

    if (!s_armed) { was_tx = false; return; }

    /* APRS_TxFrame()-style routines (and our own imgfsk_tx.c) reprogram
     * every BK4819/29 register when they finish -- do not fight a TX in
     * progress, and re-arm exactly once on the way back to RX (same lesson
     * as the earlier hardware-AFSK attempt: never reset every tick, it
     * never gives the correlator a chance to lock). */
    if (gCurrentFunction == FUNCTION_TRANSMIT) { was_tx = true; return; }
    if (was_tx) { was_tx = false; imgfsk_rx_arm(s_fsk2400); s_widx = 0; return; }

    while (BK4819_ReadRegister(BK4819_REG_0C) & 1u) {
        /* ⚠️ (2026-09-25, retour terrain : rien du tout, même pas de fausses
         * données) -- diagnostic bon marché : bascule la LED verte (distincte
         * du rouge "armé") à CHAQUE interruption matérielle vue, même sans
         * FIFO_ALMOST_FULL. Si elle ne change jamais d'état pendant un test,
         * le moteur FSK ne réagit à rien du tout (mauvaise fréquence/débit,
         * ou le calage lui-même ne convient pas) -- pas la peine de chercher
         * plus loin dans le décodage tant que ce signal n'a pas bougé. */
        static bool s_green_toggle;
        s_green_toggle = !s_green_toggle;
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, s_green_toggle);

        BK4819_WriteRegister(BK4819_REG_02, 0);          /* latch, same order
                                                          * as CheckRadioInterrupts() */
        uint16_t irq = BK4819_ReadRegister(BK4819_REG_02);

        if (!(irq & IMGFSK_IRQ_FIFO_ALMOST_FULL))
            continue;

        for (int i = 0; i < 4 && s_widx < 128; i++) {
            uint16_t w = BK4819_ReadRegister(BK4819_REG_5F);
            s_pkt[s_widx * 2]     = (uint8_t)(w >> 8);
            s_pkt[s_widx * 2 + 1] = (uint8_t)(w & 0xFF);
            s_widx++;
        }

        if (s_widx >= 128) {
            imgfsk_rx_forward(s_pkt);
            s_widx = 0;
            BK4819_PrepareFSKReceive();   /* re-arm for the next packet, same
                                          * as AIRCOPY_StorePacket() does
                                          * unconditionally on completion */
        }
    }
}
