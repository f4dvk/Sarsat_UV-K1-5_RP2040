/* Raw-FSK image link, RX side (branch SSTV_SSDV), K1/K5 V3 only.
 *
 * ⚠️ (2026-09-25) REWRITTEN, v2. The original design here drove the
 * BK4819/29's own hardware raw-FSK/FIFO engine (the one already proven
 * correct, bidirectionally, by the stock AirCopy feature) for reception too,
 * mirroring imgfsk_tx.c's TX. It was extensively field-debugged (~30
 * commits) but never reached reliable multi-packet reception: the
 * correlator intermittently stops generating any interrupt after a handful
 * of packets, for a reason that survived every register/timing/re-arm
 * hypothesis tried, WIDE/NARROW, FM/RAW modulation, disabling AFC, the
 * F4HWN auto-sleep timer, and even the RP2040 physically unplugged (still
 * reproduced) -- a real, unexplained hardware/firmware interaction.
 *
 * TX stays on that same hardware engine (proven robust on its own -- a
 * stock AirCopy receiver decodes our TX in a loop up to 100% without ever
 * locking up), but RX now goes back to this project's other proven-reliable
 * path: RAW/DSC audio -> RP2040 ADC -> software demod, the same
 * architecture SARSAT and APRS/Sonde already use for hours at a time
 * without this kind of lock-up (see rp2040/src/imgfsk_sync.c and
 * decoder_config.h's "branch SSTV_SSDV" comment for the RP2040 side).
 *
 * This means the radio side does almost nothing any more: no BK4819
 * register writes, no per-tick polling, no watchdog -- just track whether
 * the operator asked to listen (and at which baud), and report it in the
 * existing CMD_SARSAT_HELLO status the RP2040 already polls every 5 s
 * (screen_state byte, values 4/5) -- the same way it already tells the
 * RP2040 about Sonde's own screen state (App/app/sarsat.c). The RP2040
 * decides on its own when to actually switch its ADC/demod mode; this side
 * only has to keep telling the truth about what was requested. */
#ifndef APP_IMGFSK_RX_H
#define APP_IMGFSK_RX_H

#include <stdbool.h>

/* Assignable to a side-function key (ACTION_OPT_*), like TX's own
 * IMGFSK_Send1200/2400: first press requests listening at that rate, a
 * second press on the SAME key cancels it. Pressing the OTHER rate's key
 * while active switches rate directly. Purely local state (a couple of
 * flags + an LED) -- no radio register touched at all. */
void IMGFSK_ToggleRx1200(void);
void IMGFSK_ToggleRx2400(void);

/* Queried by App/app/sarsat.c when building the periodic CMD_SARSAT_HELLO
 * status (screen_state byte) -- see this header's own comment above and
 * rp2040/src/decoder_config.h's "branch SSTV_SSDV" comment for the wire
 * values (4 = 1200 baud requested, 5 = 2400 baud requested, 0 = neither). */
bool IMGFSK_RxActive(void);
bool IMGFSK_RxIsFsk2400(void);

#endif /* APP_IMGFSK_RX_H */
