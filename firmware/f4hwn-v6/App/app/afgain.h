/* C-Board AF output gain override for the Sarsat_UV-K1-5_RP2040 project.
 *
 * Shared by the SARSAT screen (and, once ported, the APRS tracker): a fixed
 * BK4819 RX audio gain (REG_48, "AF Rx Gain-2" + DAC gain) instead of
 * whatever the radio's own volume knob is set to, so the pot can sit at max
 * and the level feeding the RP2040 C-Board's ADC tap stays put and
 * repeatable regardless of the operator's volume setting.
 *
 * Independent, tiny EEPROM-backed module (own magic byte, own address) so
 * it works standalone with just ENABLE_SARSAT -- no dependency on APRS.
 */
#ifndef APP_AFGAIN_H
#define APP_AFGAIN_H

#include <stdint.h>

/* 0 (or out of 1..78, including the erased-EEPROM 0xFF) = auto: leave the
 * radio's own volume-knob gain alone. 1..78 = fixed level, low to high. */
extern uint8_t gAfGain;

void AFGAIN_Init(void);    /* lazy: loads from EEPROM the first time it's touched */
void AFGAIN_Apply(void);   /* push gAfGain to the BK4819 RX audio gain (REG_48) */
void AFGAIN_Save(void);    /* persist gAfGain to EEPROM */

/* "auto" (gAfGain outside 1..78) restores the radio's own volume-knob gain,
 * captured once into a static so repeated AFGAIN_Apply() calls (the ~500 ms
 * re-assert while a screen is open) don't need to re-read it every time. But
 * that means it can go stale: if it was captured early (e.g. the first
 * SARSAT/APRS screen opened after boot, possibly with the knob turned down
 * for an earlier test) it stays latched to that level for the rest of the
 * power-on session even after the knob is turned up -- "auto" then silently
 * clamps RX audio low, which reads as reduced sensitivity ("needs a
 * stronger signal"). Call this once, right before the first AFGAIN_Apply(),
 * whenever a screen that uses the override is (re)opened, so "auto" always
 * reflects the knob position as of *this* opening, not a stale one. */
void AFGAIN_ResyncKnob(void);

/* Force the stock (knob) RX audio gain once, ignoring gAfGain -- for a
 * caller whose audio path needs a normal signal level regardless of a
 * SARSAT fixed override (e.g. an overlay app sampling BK4819's own AF
 * output for its own demod). Leaves gAfGain itself untouched: the next
 * AFGAIN_Apply() (background TimeSlice, or a screen re-opening) restores
 * whatever override was configured, as if this call never happened. */
void AFGAIN_Bypass(void);

/* Call every ~10 ms from the main tick, unconditionally (screen open or not).
 * Keeps a *fixed* gAfGain (1..78) in effect in the background too, not just
 * while APP_RunSarsat() is on screen -- without this, the level you
 * calibrate on the SARSAT level view only applied while that screen was
 * open; the radio's normal volume-knob tracking (still running in the
 * background) silently took back over the instant it closed, so background
 * (auto-popup) decoding used a different, uncalibrated level. No-op in
 * "auto" mode (gAfGain outside 1..78). */
void AFGAIN_TimeSlice(void);

/* The real stock (knob) gain, protected from whatever gAfGain's override is
 * currently forcing into the live gEeprom.VOLUME_GAIN. "auto" (no override
 * active): returns the live value directly, always fresh -- any SetVol menu
 * change takes effect immediately, same as plain F4HWN. A fixed override:
 * returns the cached pre-override snapshot instead of the live (overridden)
 * value. SETTINGS_WriteCurrentVol() must use this, not gEeprom.VOLUME_GAIN
 * directly, or a generic settings-save made while a fixed override is active
 * permanently bakes that override into the radio's own saved volume setting
 * (2026-10-17, user report: SetVol stuck at the SARSAT override level,
 * couldn't be fixed even across a power cycle -- the save kept re-persisting
 * the override every time *anything* triggered SETTINGS_SaveSettings()). */
uint8_t AFGAIN_StockVolGain(void);

#endif /* APP_AFGAIN_H */
