/* See imgfsk_rx.h. K1/K5 V3 (PY32F071 + BK4829) only. */
#include "app/imgfsk_rx.h"

#include "driver/bk4819.h"

static bool s_active;
static bool s_fsk2400;

/* ⚠️ (2026-09-25) LED rouge as a simple "listening requested" indicator --
 * no per-tick upkeep needed any more (unlike the old hardware-correlator
 * design), just set once here on toggle. Same GPIO imgfsk_tx.c keys with
 * during TX, never lit at the same time as a TX burst in practice (the
 * operator arms RX on one radio, TX on the other). */
void IMGFSK_ToggleRx1200(void)
{
    if (s_active && !s_fsk2400) {
        s_active = false;
    } else {
        s_active  = true;
        s_fsk2400 = false;
    }
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, s_active);
}

void IMGFSK_ToggleRx2400(void)
{
    if (s_active && s_fsk2400) {
        s_active = false;
    } else {
        s_active  = true;
        s_fsk2400 = true;
    }
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, s_active);
}

bool IMGFSK_RxActive(void)    { return s_active; }
bool IMGFSK_RxIsFsk2400(void) { return s_fsk2400; }
