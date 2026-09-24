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
static uint32_t s_dirty_count;   /* diag: how often that hook actually fires,
                                  * see imgfsk_rx_send_diag()'s comment */
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
    /* ⚠️ (2026-09-24, retour terrain, comparatif demande) -- boot.c (AirCopy)
     * utilise NARROW, essaye ici en comparaison : WIDE s'avere nettement
     * meilleur sur le terrain -- 10 paquets consecutifs recus sans le
     * moindre declenchement du chien de garde (contre une recuperation
     * toutes les 1-2 trames en NARROW), et REG_0C bit 1 (squelch "Link")
     * reste actif en continu au lieu de clignoter par intermittence. NARROW
     * rognait vraisemblablement une partie de la deviation FSK (surtout si
     * l'accord frequence n'est pas parfaitement centre), causant les pertes
     * de verrouillage frequentes chassees sur plusieurs tours precedents.
     * Fixe ici comme reglage definitif, pas juste un essai comparatif. */
    gRxVfo->CHANNEL_BANDWIDTH = BANDWIDTH_WIDE;
    /* ⚠️ (2026-09-24, retour terrain : essai RAW compare a FM) -- RAW s'avere
     * PIRE que FM : paquets nettement plus corrompus/melanges (voir retour
     * terrain "irq=197 fifo=190" fige, RAW actif) qu'avec les duplications
     * ponctuelles observees sous FM. Revenu a MODULATION_FM, la valeur
     * qu'une VFO fraiche a deja par defaut dans boot.c (AirCopy). */
    gRxVfo->Modulation        = MODULATION_FM;

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

    /* ⚠️ (2026-09-24) Corrige : 0x00C3 etait une valeur FAUSSE, prise d'une
     * note erronee plus tot dans cette session -- la vraie valeur ecrite par
     * BK4819_SetupAircopy() (App/driver/bk4829.c, relue directement) est
     * 0x00E0 (Tone2 enable, gain 48). Trouve en corrigeant le meme bug cote
     * TX (imgfsk_tx.c), ou son absence totale expliquait "ca ne reagit pas
     * encore" meme apres la correction REG_3F/timeout. */
    /* ⚠️ (2026-09-24, retour terrain : "le 2400 ne fonctionne pas ... je peux
     * meme regler le tx 1200, rx 2400 et ca decode") -- REG_72 etait fixe a
     * 0x3065 quel que soit fsk2400. D'apres la note d'application BK4819(V3)
     * (deja telechargee cette session) : REG_72 = "TONE2/FSK frequency
     * control word" = freq(Hz) * 10.32444 (XTAL 26 MHz) -- c'est litteralement
     * l'horloge du debit, pas juste "la valeur AirCopy" comme suppose a tort
     * jusqu'ici. 0x3065 = 12389 = 1200 Hz pile -- jamais change, le debit
     * reel restait donc TOUJOURS 1200 en interne, quel que soit le mode
     * choisi au menu, expliquant a la fois la duree TX identique et un RX
     * "2400" qui decode un TX "1200" (les deux tournaient reellement a 1200).
     * Pour 2400 Hz : 2400*10.32444 = 24778.656 -> arrondi 24779 = 0x60CB. */
    BK4819_WriteRegister(BK4819_REG_70, 0x00E0u);   /* AirCopy's own value    */
    BK4819_WriteRegister(BK4819_REG_72, fsk2400 ? 0x60CBu : 0x3065u);
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
    if (s_armed) { s_hw_dirty = true; s_dirty_count++; }
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

/* ⚠️ (2026-09-25, retour terrain : "le rx se bloque après la premiere
 * image, ... ne réagit plus lors d'un envoi", irq/fifo figés à une valeur
 * constante sur de nombreux ticks consecutifs) -- contrairement au blocage
 * paquet-a-paquet (BK4819_PrepareFSKReceive() seul, sans hw_dirty, a deja
 * enchaine 4 paquets sans probleme dans le test precedent), celui-ci
 * n'apparait qu'ENTRE deux sessions d'envoi completes, apres un temps
 * d'ecoute inactive -- compatible avec IMGFSK_OnRadioSetupRegisters()
 * (ajoutee au commit precedent) se declenchant bien plus souvent que prevu
 * en tache de fond (APRS_TimeSlice(), squelch, etc, visibles dans les logs
 * de l'utilisateur) : si s_hw_dirty repasse a vrai a CHAQUE tick, chaque
 * appel a IMGFSK_TimeSlice() rearme aussitot sans jamais atteindre la
 * boucle d'ecoute -- irq/fifo ne bougeraient alors plus jamais, exactement
 * le symptome observe. Compteur ajoute pour verifier cette hypothese avant
 * de corriger a l'aveugle : s_dirty_count doit rester quasi plat si
 * RADIO_SetupRegisters() n'est appele qu'occasionnellement en fond, ou
 * grimper au rythme du tick (~100/s) si c'est la vraie cause. */
/* ⚠️ (2026-09-24, retour terrain : "meme probleme" avec le rearmement
 * complet aussi -- degradation progressive (5 puis 4 puis 0 paquets)
 * identique qu'avec le rearmement leger, dirty= sans rapport avec le
 * nombre de paquets captes avant blocage -- ecarte les DEUX hypotheses
 * logicielles de rearmement testees jusqu'ici. Ajoute la lecture directe de
 * REG_3F (masque d'interruption reellement actif), REG_58 (mode FSK/bande
 * TOUJOURS actif) et REG_0C (statut brut) au moment de l'envoi du diag, pour
 * voir si un registre a change de valeur de facon inattendue au moment du
 * blocage, plutot que de continuer a deviner cote rearmement. */
static void imgfsk_rx_send_diag(void)
{
    uint8_t b[22];
    uint16_t reg3f = BK4819_ReadRegister(BK4819_REG_3F);
    uint16_t reg58 = BK4819_ReadRegister(BK4819_REG_58);
    uint16_t reg0c = BK4819_ReadRegister(BK4819_REG_0C);
    b[0] = 0xE3; b[1] = 0x06;   /* CMD_IMGFSK_RXDIAG, 0x06E3 LE */
    b[2] = 0x12; b[3] = 0x00;   /* size = 18 LE */
    memcpy(b + 4, &s_irq_count, 4);
    memcpy(b + 8, &s_fifo_count, 4);
    memcpy(b + 12, &s_dirty_count, 4);
    memcpy(b + 16, &reg3f, 2);
    memcpy(b + 18, &reg58, 2);
    memcpy(b + 20, &reg0c, 2);
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
#define IMGFSK_IRQ_RX_SYNC          (1u << 1)    /* REG_02 bit 1, BK4819_REG_02_FSK_RX_SYNC */

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

    /* ⚠️ (2026-09-24, retour terrain : blocage sporadique confirme, parfois
     * en plein milieu d'un paquet -- REG_3F/REG_58 intacts, TX confirme
     * valide au meme instant via un recepteur AirCopy d'origine) -- quatre
     * sequences de rearmement post-paquet differentes n'ont pas empeche ce
     * blocage de survenir (voir le commentaire retire plus bas dans cette
     * fonction), ce qui ecarte la sequence de rearmement elle-meme comme
     * cause. Plutot que de continuer a chercher LA bonne recette pour
     * EMPECHER le corrélateur de se figer -- ce que la seule recuperation
     * fiable connue a ce jour (menu, confirmee par le terrain) ne permet
     * pas de comprendre depuis ce siege -- ce chien de garde detecte
     * simplement l'ABSENCE d'activite (aucune interruption FSK vue depuis
     * plusieurs secondes alors que l'ecoute est armee) et force un cycle
     * complet desarmement+reamement, quelle que soit la cause reelle du
     * blocage. Seuil choisi nettement au-dessus de l'espacement normal
     * entre deux paquets (~1.7-2 s a 1200 bauds) pour ne jamais interrompre
     * une reception qui progresse normalement, meme lentement.
     *
     * ⚠️ (2026-09-24, retour terrain : "le reset se fait a chaque trame" --
     * le chien de garde recupere bien a chaque fois, mais se declenche tres
     * souvent, ~1 fois toutes les 1-2 trames) -- verifie dans la note
     * d'application BK4819(V3) (deja telechargee cette session) : aucun bit
     * "depassement FIFO" documente pres de REG_0C pour confirmer ou infirmer
     * l'hypothese d'un vrai buffer plein ; la FIFO Rx fait bien 8 mots avec
     * un seuil "presque pleine" a 4 (REG_5E, valeur par defaut du fabricant,
     * inchangee) -- rien d'anormal cote configuration. Sans bit dedie pour
     * trancher la cause exacte, seuil resserre de 4 s a 2.5 s (encore
     * nettement au-dessus de l'espacement normal ~1.7-2 s) pour recuperer
     * plus vite quelle que soit la cause, en attendant une piste plus sure.
     *
     * ⚠️ (2026-09-24, retour terrain : desactive temporairement pour isoler
     * la cause des resets/duplications rapportes -- confirme : "irq=197
     * fifo=190" reste fige durablement SANS que le chien de garde intervienne
     * (action coupee), preuve que le blocage est reel et independant de lui
     * -- il corrigeait un vrai gel du correlateur, pas un faux reset qu'il
     * aurait lui-meme provoque. Reactive. */
    static uint32_t s_watchdog_last_irq;
    static uint16_t s_watchdog_ticks;
    if (s_irq_count != s_watchdog_last_irq) {
        s_watchdog_last_irq = s_irq_count;
        s_watchdog_ticks = 0;
    } else if (++s_watchdog_ticks >= 250) {   /* ~2.5 s a ~10 ms/tick */
        s_watchdog_ticks = 0;
        imgfsk_rx_disarm();
        SYSTEM_DelayMs(300);
        imgfsk_rx_arm(s_fsk2400);
        return;
    }

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

        /* ⚠️ (2026-09-24, retour terrain : premier paquet reçu -- irq/fifo
         * enfin non nuls -- mais son contenu est la concatenation exacte des
         * 32 premiers octets de DEUX paquets de test DIFFERENTS, verifie
         * octet par octet contre imgfsk_test_data.h) -- chacun des 6 paquets
         * de imgfsk_tx.c est envoye comme sa PROPRE rafale preambulee/
         * synchronisee independamment. Si l'ecoute demarre en cours de
         * sequence (l'operateur arme la RX apres que la TX ait deja
         * commence), le correlateur materiel peut tres bien se reverrouiller
         * (FSK_RX_SYNC) sur le PREAMBULE D'UN PAQUET SUIVANT alors que des
         * octets du paquet precedent, incomplets, etaient deja accumules
         * dans s_pkt -- sans ce traitement, ils etaient simplement concatenes
         * a la suite, produisant un "paquet" corrompu melant deux trames.
         * Chaque reverrouillage (bit 1, deja active par
         * imgfsk_ensure_irq_mask()) doit donc jeter toute capture partielle
         * en cours et repartir de zero. */
        if (irq & IMGFSK_IRQ_RX_SYNC)
            s_widx = 0;

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
            /* ⚠️ (2026-09-24) Quatre variantes de rearmement post-paquet ont
             * ete essayees ici et retirees, aucune n'ayant resolu le
             * blocage rapporte par le terrain (BK4819_PrepareFSKReceive()
             * seul ; imgfsk_rx_arm() seul ; imgfsk_rx_disarm()+arm() ;
             * meme avec 300 ms d'attente entre les deux) : le blocage
             * survient de facon sporadique, parfois EN PLEIN MILIEU d'un
             * paquet (pas seulement juste apres un rearmement), avec REG_3F/
             * REG_58 pourtant intacts et le TX confirme valide au meme
             * moment (recepteur AirCopy d'origine) -- ce n'est donc pas la
             * sequence de rearmement post-paquet qui est en cause. Revenu
             * ici a la version la plus simple, celle qu'utilise AirCopy lui
             * meme entre ses propres paquets (App/app/aircopy.c,
             * AIRCOPY_StorePacket()) ; la recuperation du blocage lui-meme
             * est traitee separement par un chien de garde -- voir son
             * commentaire plus bas. */
            BK4819_PrepareFSKReceive();
        }
    }
}
