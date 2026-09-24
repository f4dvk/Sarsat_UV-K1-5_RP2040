/* See imgfsk_rx.h. K1/K5 V3 (PY32F071 + BK4829) only. */
#include <stdint.h>
#include <string.h>

#include "app/imgfsk_rx.h"
#include "app/uart.h"

#include "driver/bk4819.h"
#include "functions.h"
#include "radio.h"

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

static bool s_armed;
static bool s_fsk2400;

static void imgfsk_rx_arm(bool fsk2400)
{
    RADIO_SetupRegisters(true);   /* normal RX for the currently tuned channel */

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
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);   /* see disarm's comment */
}

static void imgfsk_rx_disarm(void)
{
    s_armed = false;
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
    if (was_tx) { was_tx = false; imgfsk_rx_arm(s_fsk2400); return; }

    if ((BK4819_ReadRegister(BK4819_REG_0C) & 1u) == 0)
        return;

    uint8_t pkt[256];
    for (int i = 0; i < 128; i++) {
        uint16_t w = BK4819_ReadRegister(BK4819_REG_5F);
        pkt[i * 2]     = (uint8_t)(w >> 8);
        pkt[i * 2 + 1] = (uint8_t)(w & 0xFF);
    }
    BK4819_WriteRegister(BK4819_REG_02, 0);   /* clear, as proven elsewhere */
    BK4819_PrepareFSKReceive();               /* re-arm for the next packet */

    imgfsk_rx_forward(pkt);
}
