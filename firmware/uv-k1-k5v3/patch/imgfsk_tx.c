/* See imgfsk_tx.h. K1/K5 V3 (PY32F071 + BK4829) only. */
#include "app/imgfsk_tx.h"
#include "imgfsk_test_data.h"

#include "driver/bk4819.h"
#include "driver/system.h"
#include "radio.h"

/* ---------------------------------------------------------------------
 * Register recipe: BK4819_SendFSKData()'s own sequence (App/driver/bk4829.c,
 * already proven correct on this exact chip by the stock AirCopy feature),
 * kept byte-for-byte where it doesn't need to change:
 *   REG_59 = 0x0068 : preamble 7 bytes, sync length 4 bytes, no scramble,
 *                      FSK TX/RX both off (idle) -- AirCopy's own value.
 *   REG_59 = 0x8068 / 0x0068 : clear TX FIFO, then un-clear.
 *   REG_59 = 0x2868 : same preamble/sync bits, FSK TX enabled -- this is
 *                      what actually starts the preamble+sync+FIFO burst.
 *   REG_0C bit 0    : "an FSK interrupt is pending" -- polled the same way
 *                      BK4819_SendFSKData()/PrepareFSKReceive() already do.
 * REG_5A/5B (the sync word itself) are deliberately left untouched here --
 * BK4819_ResetFSK() never rewrites them either, so both ends run on the
 * chip's own POR default (Sync Byte 0..3 = 0x85, 0xCF, 0xAB, 0x45, per the
 * BK4819(V3) Application Note's register table), the same one AirCopy
 * relies on implicitly. Only two things actually change
 * from AirCopy: REG_58 (TX mode + bandwidth, 1.2K or 2.4K instead of
 * AirCopy's fixed 1.2K raw mode) and REG_5D (256-byte SSDV packet length
 * instead of AirCopy's fixed 72 bytes).
 * ---------------------------------------------------------------------
 * Framing choice: each of the IMGFSK_TEST_PACKET_COUNT SSDV packets is sent
 * as its OWN independently-preambled/synced burst (a fresh FIFO clear +
 * trigger per packet), not one giant concatenated frame. SSDV is designed
 * for exactly this -- each 256 B packet decodes independently (graceful
 * degradation, missing packets show as grey blocks, not a wrecked image) --
 * so re-synchronising per packet trades a little airtime for much better
 * resilience to a missed bit or two, which matters more on a real link than
 * in this bring-up test.
 * ---------------------------------------------------------------------
 * REG_30 (the RF DSP / PA / PLL enable BK4819_ResetFSK()'s BK4819_Idle()
 * call would zero) is deliberately never touched by the per-packet reset
 * below -- unlike the standalone AirCopy screen, this routine keys up ONCE
 * for the whole test image (like a roger beep), so the normal TX chain
 * (RADIO_SetTxParameters()/BK4819_EnableTXLink()) must stay up across all
 * packets; only BK4819_ResetFSK() itself (which the full teardown at the
 * end calls) is safe to touch REG_30, once un-keyed. */

/* ⚠️ (2026-09-24, retour terrain : "le RX aircopy ne réagit pas" à notre
 * émission -- confirme que le bug est côté TX, pas RX, en comparant à un
 * récepteur AirCopy d'origine, déjà prouvé fonctionnel) -- trouvé en relisant
 * BK4819_SendFSKData() ligne à ligne (App/driver/bk4829.c) : 256 o = 2048
 * bits, soit ~1707 ms à 1200 bauds (le calcul précédent, ~853 ms, était
 * faux -- confusion avec le débit 2400). L'ancien budget de 400 ms coupait
 * donc CHAQUE paquet en plein milieu de son émission, bien avant la fin des
 * bits -- le paquet suivant écrasait alors la FIFO en cours de trame. Porté
 * à 2000 ms (marge au-delà des 1707 ms nécessaires au pire cas, 1200 bauds)
 * ; poll loop re-checks every 5 ms so this is the whole burst's budget, not
 * a per-iteration one. */
#define IMGFSK_FIFO_TIMEOUT_MS   2000
#define IMGFSK_FIFO_POLL_MS      5

static void imgfsk_send_one_packet(const uint8_t *pkt)
{
    /* ⚠️ (2026-09-24, meme retour terrain) -- BK4819_SendFSKData() active
     * explicitement BK4819_REG_3F_FSK_TX_FINISHED avant de declencher la
     * rafale ; notre code ecrivait REG_3F=0 (TOUTES les interruptions
     * masquees) une seule fois avant la boucle d'envoi, dans
     * IMGFSK_SendTestImage() -- la source d'interruption qu'on poll juste en
     * dessous (REG_0C bit 0) ne pouvait alors jamais se lever pour la bonne
     * raison, seul le timeout (voir ci-dessus) faisait sortir la boucle. */
    BK4819_WriteRegister(BK4819_REG_3F, BK4819_REG_3F_FSK_TX_FINISHED);
    BK4819_WriteRegister(BK4819_REG_59, 0x8068);   /* clear TX FIFO */
    BK4819_WriteRegister(BK4819_REG_59, 0x0068);   /* un-clear */

    for (int i = 0; i < IMGFSK_PACKET_SIZE / 2; i++) {
        uint16_t w = ((uint16_t)pkt[i * 2] << 8) | pkt[i * 2 + 1];
        BK4819_WriteRegister(BK4819_REG_5F, w);
    }

    SYSTEM_DelayMs(20);
    BK4819_WriteRegister(BK4819_REG_59, 0x2868);   /* enable FSK TX -> go */

    for (int waited = 0; waited < IMGFSK_FIFO_TIMEOUT_MS; waited += IMGFSK_FIFO_POLL_MS) {
        SYSTEM_DelayMs(IMGFSK_FIFO_POLL_MS);
        if (BK4819_ReadRegister(BK4819_REG_0C) & 1u)
            break;
    }
    BK4819_WriteRegister(BK4819_REG_02, 0);        /* clear, as proven elsewhere */
    BK4819_WriteRegister(BK4819_REG_59, 0x0068);   /* back to idle for the next packet */
}

void IMGFSK_SendTestImage(bool fsk2400)
{
    /* Key up on whatever channel/frequency is already selected -- no VFO
     * borrow, same model as a roger beep / the AirCopy screen: the operator
     * tunes both ends to an agreed test frequency before triggering this. */
    BK4819_DisableDTMF();
    RADIO_SetTxParameters();
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
    SYSTEM_DelayMs(20);
    BK4819_EnableTXLink();
    SYSTEM_DelayMs(50);

    BK4819_WriteRegister(BK4819_REG_59, 0x0068);   /* preamble 7B, sync 4B,
                                                    * no scramble, idle --
                                                    * AirCopy's own value */

    /* ⚠️ (2026-09-24, retour terrain : "ca ne reagit pas encore" meme apres
     * la correction REG_3F/timeout) -- en relisant BK4819_SetupAircopy()
     * elle-meme (App/driver/bk4829.c) au lieu de se fier a une note prise
     * plus tot dans cette session (qui donnait REG_70=0x00C3, une valeur
     * FAUSSE, deja recopiee a tort dans imgfsk_rx.c) : cette fonction, TOUJOURS
     * appelee par AirCopy avant tout TX ou RX, ecrit aussi REG_70 (gain du
     * ton FSK) et REG_72 (mot de controle du debit FSK) -- notre TX ne les
     * ecrivait JAMAIS, en dehors de tout menu AirCopy, laissant ces registres
     * a leur valeur RX/vocale normale du canal courant. Sans REG_72 correct,
     * la deviation/le debit reellement emis est incorrect -- un signal
     * invalide pour N'IMPORTE quel correlateur FSK, expliquant pourquoi meme
     * l'AirCopy d'en face ne reagit a rien, independamment du cote RX. */
    /* ⚠️ (2026-09-24, retour terrain : "le 2400 ne fonctionne pas ... la
     * duree de transmission est identique au 1200") -- REG_72 etait fixe a
     * 0x3065 quel que soit fsk2400. D'apres la note d'application BK4819(V3) :
     * REG_72 = "TONE2/FSK frequency control word" = freq(Hz) * 10.32444
     * (XTAL 26 MHz) -- c'est litteralement l'horloge du debit, pas juste
     * "la valeur AirCopy" comme suppose a tort jusqu'ici (voir aussi
     * imgfsk_rx.c, meme bug, meme retour terrain). 0x3065 = 12389 = 1200 Hz
     * pile -- jamais change, le debit reel restait donc TOUJOURS 1200 en
     * interne quel que soit le mode choisi, expliquant la duree TX
     * identique. Pour 2400 Hz : 2400*10.32444 = 24778.656 -> arrondi 24779
     * = 0x60CB. */
    BK4819_WriteRegister(BK4819_REG_70, 0x00E0u);   /* Tone2 enable, gain 48 */
    BK4819_WriteRegister(BK4819_REG_72, fsk2400 ? 0x60CBu : 0x3065u);

    /* REG_58: FSK1.2K/2.4K TX mode (bits<15:13>=000), bandwidth
     * (bits<3:1>: 000=1.2K, 100=2.4K), FSK enable (bit0=1). */
    BK4819_WriteRegister(BK4819_REG_58, fsk2400 ? 0x0009u : 0x0001u);

    BK4819_WriteRegister(BK4819_REG_5C, 0x5665u);   /* AirCopy's own value,
                                                     * CRC enable + unknown
                                                     * bits -- also missing
                                                     * from TX until now */

    /* REG_5D: FSK data length = 256 bytes (value = length-1 = 255 = 0xFF,
     * low 8 bits at <15:8>, high 3 bits at <7:5> -- same formula AirCopy's
     * own 0x4700/72-byte value follows). */
    BK4819_WriteRegister(BK4819_REG_5D, 0xFF00u);

    for (int p = 0; p < IMGFSK_TEST_PACKET_COUNT; p++) {
        imgfsk_send_one_packet(&g_imgfsk_test_packets[p * IMGFSK_PACKET_SIZE]);
        SYSTEM_DelayMs(20);   /* brief inter-packet gap */
    }

    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
    RADIO_SetupRegisters(true);   /* back to RX, same teardown as APRS_TxFrame() */
}

void IMGFSK_Send1200(void) { IMGFSK_SendTestImage(false); }
void IMGFSK_Send2400(void) { IMGFSK_SendTestImage(true); }
