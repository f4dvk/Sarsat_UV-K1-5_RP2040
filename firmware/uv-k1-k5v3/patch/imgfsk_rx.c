/* See imgfsk_rx.h. K1/K5 V3 (PY32F071 + BK4829) only. */
#include <stdint.h>
#include <string.h>

#include "app/imgfsk_rx.h"
#include "app/uart.h"

#include "driver/bk4819.h"
#include "driver/system.h"
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
static bool    s_hw_dirty;   /* see IMGFSK_OnRadioSetupRegisters()'s comment */
static bool    s_fsk2400;
static uint8_t s_pkt[256];
static int     s_widx;   /* words accumulated so far (0..128), see
                          * IMGFSK_TimeSlice()'s comment */
static uint8_t          s_saved_battery_save;
static uint8_t          s_saved_dual_watch;
static uint8_t          s_saved_bandwidth;
static ModulationMode_t s_saved_modulation;

/* ⚠️ (2026-09-25) Rewritten as a byte-for-byte replica of AirCopy's actual,
 * complete, CONFIRMED-WORKING receive sequence (on the operator's own two
 * radios) instead of one guessed-and-tested variable at a time -- on
 * explicit request, after several rounds of single-register hypotheses
 * (REG_2B, WIDE vs NARROW, APP_StartListening, battery-save/dual-watch)
 * each only partially matched what the reference actually does. The full
 * reference is App/helper/boot.c (BOOT_MODE_AIRCOPY, screen entry) +
 * App/app/aircopy.c (AIRCOPY_Key_EXIT(), when the operator starts
 * receiving) + App/driver/bk4829.c (BK4819_SetupAircopy() /
 * BK4819_PrepareFSKReceive()). Differences from that reference that remain
 * here, all deliberate:
 *   - No RADIO_InitInfo() -- that blanks the VFO to AirCopy's own fixed LPD
 *     frequency; we want whatever channel/frequency the operator already
 *     tuned both radios to.
 *   - gRxVfo->Modulation is set to the FIELD value MODULATION_FM (matching
 *     what a freshly RADIO_InitInfo()'d VFO already defaults to, which is
 *     why AirCopy itself never calls RADIO_SetModulation() at all) instead
 *     of calling RADIO_SetModulation() afterward -- if the operator's
 *     channel was actually in a non-FM modulation (RAW, seen on screen
 *     after arming in an earlier attempt), forcing the field before
 *     RADIO_SetupRegisters() runs is the same state AirCopy starts from,
 *     rather than trying to undo RAW-specific setup after the fact.
 *   - REG_58 (bandwidth: 1.2K vs 2.4K) and REG_5D (256-byte SSDV packet
 *     length instead of AirCopy's 72 bytes) still deliberately differ --
 *     the whole point of this module.
 *   - No APP_StartListening()/squelch-forcing: AirCopy itself never calls
 *     it either, so it was a guess that didn't match the proven reference
 *     and is removed here. */
static void imgfsk_rx_arm(bool fsk2400)
{
    s_saved_battery_save = gEeprom.BATTERY_SAVE;
    s_saved_dual_watch   = gEeprom.DUAL_WATCH;
    s_saved_bandwidth    = gRxVfo->CHANNEL_BANDWIDTH;
    s_saved_modulation   = gRxVfo->Modulation;

    gEeprom.BATTERY_SAVE      = 0;              /* boot.c                    */
    gEeprom.DUAL_WATCH        = DUAL_WATCH_OFF; /* boot.c                    */
    gRxVfo->CHANNEL_BANDWIDTH = BANDWIDTH_NARROW; /* boot.c -- NOT wide      */
    gRxVfo->Modulation        = MODULATION_FM;    /* what a fresh VFO already
                                                   * defaults to in boot.c   */

    RADIO_ConfigureSquelchAndOutputPower(gRxVfo);   /* boot.c, same order    */
    gCurrentVfo = gRxVfo;                           /* boot.c, same order    */
    RADIO_SetupRegisters(true);                     /* boot.c                */

    /* ⚠️ (2026-09-25, retour terrain : REG_58 maintenant identique au bit
     * près à la valeur AirCopy prouvée -- 0x00C1, aucune modification --
     * NARROW/FM confirmés appliqués (écran : "FM N"), toujours aucune
     * réaction.) Piste suivante : un délai de stabilisation. AirCopy ne
     * l'écrit jamais explicitement, mais dans son usage réel il y a
     * toujours un délai naturel (écran affiché, l'opérateur lit, appuie
     * sur EXIT) entre RADIO_SetupRegisters() -- qui relance la PLL, un
     * processus analogique -- et l'armement FSK. Ici tout s'enchaîne dans
     * la même fonction, sans pause. Le TX (imgfsk_tx.c, confirmé
     * fonctionnel) a lui un SYSTEM_DelayMs(50) après avoir activé son
     * propre lien -- jamais répliqué côté RX jusqu'ici. */
    SYSTEM_DelayMs(50);

    BK4819_WriteRegister(BK4819_REG_70, 0x00C3u);   /* AirCopy's own value    */
    BK4819_WriteRegister(BK4819_REG_72, 0x3065u);   /* AirCopy's own value    */
    /* REG_58: RX mode = FSK1.2K/2.4K (bits<12:10>=000, AirCopy's own family),
     * RX gain = 3 (bits<9:8>, AirCopy's own value), bandwidth bits<3:1>:
     * 000=1.2K (AirCopy's own 0x00C1 unchanged) / 100=2.4K, enable bit0=1. */
    BK4819_WriteRegister(BK4819_REG_58, fsk2400 ? 0x00C9u : 0x00C1u);
    BK4819_WriteRegister(BK4819_REG_5C, 0x5665u);   /* AirCopy's own value    */
    /* Diagnostic temporaire (repli sur la longueur AirCopy 72 o) concluant :
     * irq=0/fifo=0 même ainsi -- la longueur n'était pas la cause. Retour à
     * la vraie taille SSDV (256 o = 255<<8, voir imgfsk_tx.c) maintenant que
     * la vraie cause probable (REG_3F réécrit par le code de fond, voir
     * imgfsk_ensure_irq_mask()) est traitée séparément. */
    BK4819_WriteRegister(BK4819_REG_5D, 0xFF00u);
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
    gRxVfo->Modulation        = s_saved_modulation;
    BK4819_ResetFSK();
    RADIO_SetupRegisters(true);
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
}

/* ⚠️ (2026-09-25, retour terrain : irq=0/fifo=0 en continu, y compris juste
 * après armement, alors même que la piste REG_3F/imgfsk_ensure_irq_mask()
 * n'a rien changé) -- en lisant en entier App/radio.c de ce firmware
 * (RADIO_SetupRegisters(), pas seulement bk4829.c) : cette fonction met
 * REG_3F à 0 en tout début (attente que la radio soit inactive), PUIS le
 * réécrit en fin de fonction avec son propre masque voix/squelch/CTCSS/VOX
 * -- écrasant systématiquement les bits FSK, quelle que soit la raison de
 * l'appel. Jusqu'ici seule notre propre transition TX->RX
 * (imgfsk_tx.c appelant RADIO_SetupRegisters() dans son démontage) était
 * détectée et réarmée (voir l'ancien "was_tx" ci-dessous, retiré). Mais
 * RADIO_SetupRegisters() est une fonction générale du firmware, appelée
 * pour bien d'autres raisons en tâche de fond (squelch, changement de canal,
 * etc.) -- aucune n'était détectée, donc jamais réarmée : le module pouvait
 * rester cassé indéfiniment après le tout premier appel externe suivant
 * l'armement. Repris du même besoin déjà résolu par GOGUFW-UV-K1-Messenger
 * (MSG_RF_OnRadioSetupRegisters(), appelé depuis LEUR radio.c juste après
 * l'écriture finale de REG_3F, voir build.sh) : marquer l'état matériel
 * "à refaire" à CHAQUE appel, sans condition sur la cause, et laisser
 * IMGFSK_TimeSlice() réarmer au tick suivant plutôt que de réarmer ici même
 * (on est potentiellement encore au milieu de RADIO_SetupRegisters() de
 * l'appelant). */
void IMGFSK_OnRadioSetupRegisters(void)
{
    if (s_armed) s_hw_dirty = true;
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

/* ⚠️ (2026-09-25, retour terrain : "pas de RSSI, pas d'audio" -- normal
 * dans ce mode, ça ne tranche rien -- et la LED verte, elle, ne montrait
 * jamais rien) -- doute sur la LED elle-même (visibilité/durée du flash)
 * plutôt que sur le matériel. Compteurs envoyés au RP2040 par le canal déjà
 * prouvé fonctionnel (CMD_IMGFSK_RXPKT), toutes les ~1 s pendant que
 * l'écoute est armée, pour trancher sans dépendre de la LED. */
static uint32_t s_irq_count, s_fifo_count;

static void imgfsk_rx_send_diag(void)
{
    uint8_t b[12];
    b[0] = 0xE3; b[1] = 0x06;   /* CMD_IMGFSK_RXDIAG, 0x06E3 LE */
    b[2] = 0x08; b[3] = 0x00;   /* size = 8 LE */
    memcpy(b + 4, &s_irq_count, 4);
    memcpy(b + 8, &s_fifo_count, 4);
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

/* ⚠️ (2026-09-25, retour terrain : irq=0/fifo=0 en continu, même avec des
 * registres désormais identiques au bit près à AirCopy) -- trouvé en
 * analysant un troisième projet indépendant qui fait aussi de la messagerie
 * FSK sur ce même chip (github.com/Gogu-Qs/GOGUFW-UV-K1-Messenger,
 * App/app/messenger_rf.c) : son propre historique de bogues documente EXACTEMENT
 * ce symptôme -- "messages se décodent quand REG_3F a les bits IRQ FSK
 * activés (0x3002) et échouent quand F4HWN laisse REG_3F à des valeurs
 * voix uniquement (0x0C0C)". BK4819_PrepareFSKReceive() n'écrit REG_3F
 * (le masque d'ACTIVATION des interruptions) qu'une seule fois, à
 * l'armement -- si un traitement normal de fond (squelch/CTCSS, sans
 * rapport avec ce module) réécrit ensuite REG_3F pour son propre usage,
 * les bits FSK sont silencieusement effacés du masque : le moteur peut
 * très bien fonctionner en interne, son signal n'atteint simplement plus
 * REG_0C. Contrairement à BK4819_PrepareFSKReceive() (armée une seule
 * fois, jamais réécrite après), ce correctif réaffirme -- lecture puis
 * OR, jamais un écrasement complet, pour ne pas casser le squelch/CTCSS
 * normal -- ces bits à CHAQUE cycle, comme le fait cet autre projet
 * (MSG_RF_EnsureFskIrqMask()). Inclut aussi FSK_RX_SYNC, activé là-bas
 * mais absent du masque que pose BK4819_PrepareFSKReceive() lui-même. */
static void imgfsk_ensure_irq_mask(void)
{
    const uint16_t wanted_bits = BK4819_REG_3F_FSK_RX_SYNC |
                                 BK4819_REG_3F_FSK_RX_FINISHED |
                                 BK4819_REG_3F_FSK_FIFO_ALMOST_FULL;
    const uint16_t r3f = BK4819_ReadRegister(BK4819_REG_3F);
    if ((r3f & wanted_bits) != wanted_bits)
        BK4819_WriteRegister(BK4819_REG_3F, r3f | wanted_bits);
}

void IMGFSK_TimeSlice(void)
{
    if (!s_armed) return;

    /* Do not fight a TX in progress (ours or anything else's) -- wait for
     * gCurrentFunction to move off FUNCTION_TRANSMIT before touching any
     * register. Never re-arm every tick either: same lesson as the earlier
     * hardware-AFSK attempt, resetting the engine continuously never gives
     * the correlator a chance to lock. Only re-arm when s_hw_dirty says the
     * hardware state was actually invalidated -- see
     * IMGFSK_OnRadioSetupRegisters()'s comment. */
    if (gCurrentFunction == FUNCTION_TRANSMIT) return;
    if (s_hw_dirty) { imgfsk_rx_arm(s_fsk2400); s_hw_dirty = false; return; }

    imgfsk_ensure_irq_mask();   /* see its own comment -- cheap, must run
                                * every tick, not just once at arm time */

    /* ~1 s at the ~10 ms tick rate this is called at */
    static uint16_t s_diag_ticks;
    if (++s_diag_ticks >= 100) {
        s_diag_ticks = 0;
        imgfsk_rx_send_diag();
    }

    while (BK4819_ReadRegister(BK4819_REG_0C) & 1u) {
        s_irq_count++;
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
        s_fifo_count++;

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
