/* C-Board AF output gain override -- see app/afgain.h. */
#include "app/afgain.h"

#include <stdbool.h>

#include "driver/bk4819.h"
#include "driver/eeprom.h"
#include "settings.h"

/* EEPROM: 8 bytes at 0x0A170, the unclaimed tail of the "Settings" PY25Q16
 * sector (0x00A000-0x00A170 is the last mapped byte per
 * driver/eeprom_compat.c's ADDR_MAPPINGS; the rest of that physical sector,
 * up to the next sector at 0x00B000, is real flash but unreachable through
 * the 16-bit EEPROM API until a mapping claims it -- see
 * patch/eeprom_compat.c wiring in build.sh). Same sector as the core radio
 * settings, so it survives a normal reset and is only wiped by "reset ALL"
 * (SETTINGS_FactoryReset(true)) along with everything else in that sector --
 * consistent, not worth a separate protected region for one byte. */
#define AFGAIN_EE_ADDR   0x0A1C0u
#define AFGAIN_EE_MAGIC  0xA5u

uint8_t gAfGain;
static bool    s_inited;
static uint8_t s_stock_volgain = 0xFF;   /* the radio's own volume-knob gain,
                                          * captured once so "auto" restores
                                          * it exactly */

void AFGAIN_Init(void)
{
    if (s_inited)
        return;
    uint8_t buf[8];
    EEPROM_ReadBuffer(AFGAIN_EE_ADDR, buf, sizeof(buf));
    gAfGain  = (buf[0] == AFGAIN_EE_MAGIC) ? buf[1] : 0;
    s_inited = true;
}

void AFGAIN_Save(void)
{
    AFGAIN_Init();
    uint8_t buf[8] = { AFGAIN_EE_MAGIC, gAfGain, 0, 0, 0, 0, 0, 0 };
    EEPROM_WriteBuffer(AFGAIN_EE_ADDR, buf, sizeof buf);
}

void AFGAIN_ResyncKnob(void)
{
    /* Only safe to refresh from the live gEeprom.VOLUME_GAIN when "auto":
     * that's the only case where the live value genuinely is the knob, not
     * some earlier override. Re-entering a screen that uses the override
     * (SARSAT/SONDE/APRS-RP) while a *fixed* gAfGain from an earlier visit
     * is still active in the background would otherwise re-capture the
     * override's own low value as if it were "stock" -- permanently losing
     * the real stock gain for the rest of the power-on session (2026-10-17,
     * user report: a fixed SARSAT level leaking into SetVol/overlay apps
     * with no way back short of a reboot). Keep whatever was already
     * cached instead. */
    if (gAfGain < 1 || gAfGain > 78)
        s_stock_volgain = 0xFF;
}

/* Force the stock (knob) RX audio gain once, ignoring gAfGain entirely --
 * for callers that need a normal-level BK4819 AF signal regardless of any
 * SARSAT override, without disturbing gAfGain itself (2026-10-13, user
 * report: launching an overlay app with a FIXED low SARSAT AF gain starved
 * APRSRX's own demod -- it taps the same BK4819 AF path, via PA4/ADC,
 * upstream of the speaker amp, to sample RX audio for its own software
 * decode; AFGAIN_TimeSlice() keeps re-asserting that low gain every ~500 ms
 * in the background regardless of which screen is open, so the overlay
 * loader can't just skip calling AFGAIN_Apply() at launch -- gEeprom.
 * VOLUME_GAIN is already pinned low by the time it gets there). gAfGain
 * (and thus the override) is restored normally by AFGAIN_Apply() once the
 * app returns. */
void AFGAIN_Bypass(void)
{
    AFGAIN_Init();
    if (s_stock_volgain == 0xFF)
        s_stock_volgain = gEeprom.VOLUME_GAIN;
    gEeprom.VOLUME_GAIN = s_stock_volgain;
    gEeprom.DAC_GAIN    = 0x0F;
    BK4819_SetRxAudioGain();
}

/* Same slider math as the V1 (KD8CEC) firmware's APRS_ApplyAfGain(): spend
 * "AF Rx Gain-2" down from 63 to 8 first (its linear region -- below ~4 is
 * near-mute and very non-linear, ~0.5 dB/step), then the DAC gain from 15 to
 * 0 (~2 dB/step). 1 ~ -52 dB below stock, 78 ~ stock. */
void AFGAIN_Apply(void)
{
    AFGAIN_Init();

    if (s_stock_volgain == 0xFF)
        s_stock_volgain = gEeprom.VOLUME_GAIN;

    const uint8_t v = gAfGain;
    if (v < 1 || v > 78) {                        /* auto = stock RX behaviour */
        gEeprom.VOLUME_GAIN = s_stock_volgain;
        gEeprom.DAC_GAIN    = 0x0F;
    } else {
        int notch = 78 - v;                       /* 0 (loud) .. 77 (quiet) */
        if (notch <= 55) {
            gEeprom.VOLUME_GAIN = (uint8_t)(63 - notch);   /* 63..8 */
            gEeprom.DAC_GAIN    = 0x0F;
        } else {
            int dac = 15 - (notch - 55);                   /* 14..-7 */
            gEeprom.VOLUME_GAIN = 8;
            gEeprom.DAC_GAIN    = (uint8_t)(dac > 0 ? dac : 0);
        }
    }

    /* gEeprom.VOLUME_GAIN/DAC_GAIN -> BK4819 REG_48. RADIO_SetModulation()
     * (called by RADIO_SetupRegisters()/APP_StartListening()) hard-codes the
     * DAC gain to max on its own, but RADIO_SetupRegisters() calls this same
     * BK4819_SetRxAudioGain() right afterwards, from gEeprom -- so calling it
     * again here, last, after the screen has finished (re)configuring the
     * radio, is what actually makes the override stick. */
    BK4819_SetRxAudioGain();
}

/* Call every ~10 ms from the main tick (regardless of whether a screen that
 * uses the override is open). AFGAIN_Apply() is otherwise only called from
 * inside APP_RunSarsat(): a *fixed* (1..78) gAfGain was therefore only in
 * effect while that screen was open -- the instant it closed, the radio's
 * normal volume-knob tracking (which keeps running in the background,
 * unlike this screen's own blocking loop) silently overwrote
 * gEeprom.VOLUME_GAIN/DAC_GAIN back to whatever the knob says. So a level
 * calibrated on the SARSAT level view didn't actually describe what
 * background (auto-popup) decoding used, only what the screen itself used
 * once (re)opened -- inconsistent, confusing to calibrate against. Re-assert
 * in the background too, same ~500 ms cadence as the screen's own re-apply,
 * so a fixed level is the level, everywhere, all the time. "auto" (gAfGain
 * outside 1..78) needs no background enforcement -- that's simply the
 * knob's own normal behaviour, nothing to fight. */
void AFGAIN_TimeSlice(void)
{
    static uint16_t s_tick;
    if (gAfGain < 1 || gAfGain > 78)
        return;
    if (++s_tick < 50)             /* ~500 ms at a 10 ms call rate */
        return;
    s_tick = 0;
    AFGAIN_Apply();
}

uint8_t AFGAIN_StockVolGain(void)
{
    if (gAfGain < 1 || gAfGain > 78)      /* auto: no override active, live *is* stock */
        return gEeprom.VOLUME_GAIN;
    AFGAIN_Init();
    return (s_stock_volgain == 0xFF) ? gEeprom.VOLUME_GAIN : s_stock_volgain;
}
