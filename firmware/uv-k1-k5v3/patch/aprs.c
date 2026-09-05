/* APRS tracker for the UV-K1 / UV-K5 V3 (F4HWN base) — see aprs.h.
 *
 * Port of the UV-K5 V1 (KD8CEC) module. Differences from that source, all
 * called out inline below:
 *   - SendReply() is not used to ACK any 0x06Dx command the RP2040 sends us
 *     (RXTEXT/RXINFO/GPS/DIGI), same as the V1 port, see docs/protocol.md --
 *     but it IS used for APRS_CMD_CONFIG, a command *we* originate (radio ->
 *     RP2040, unprompted), same as SARSAT_ReplyStatus() already does.
 *   - UART_IsCommandAvailable()/UART_HandleCommand() take an explicit Port
 *     argument on this firmware (UART_PORT_UART vs UART_PORT_VCP).
 *   - The C-Board AF gain override is the shared app/afgain.h module
 *     (gAfGain / AFGAIN_Apply() / AFGAIN_ResyncKnob()), not a field of
 *     aprs_cfg_t -- the SARSAT screen already needs it independently of
 *     ENABLE_APRS on this port.
 *   - millis10() is a real function here (app/scheduler.c, a getter onto the
 *     existing 10 ms SysTick_Handler() counter), not an extern from a KD8CEC-
 *     specific scheduler -- same name, same semantics, zero call-site changes.
 *   - The EEPROM address for gAprsCfg is this port's own reserved slot (see
 *     below), not KD8CEC's CEC_EEPROM_START2.
 * Everything else (VFO_Info_t, BK4819_* register calls, UI_PrintString*,
 * RADIO_*, SETTINGS_*) is close enough to egzumer/DualTachyon upstream that
 * the logic ports over unchanged -- same lesson as the SARSAT screen.
 */
#include "aprs.h"

#ifdef ENABLE_APRS

#include <string.h>

#include "py32f0xx.h"
#include "external/printf/printf.h"
#include "ax25.h"
#include "app/uart.h"
#include "app/afgain.h"     /* shared C-Board AF-gain setting (gAfGain) */
#include "app/chFrScanner.h"
#include "driver/keyboard.h"
#include "driver/backlight.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "driver/systick.h"
#include "driver/bk4819.h"
#include "driver/eeprom.h"
#include "ui/helper.h"
#include "ui/ui.h"
#include "audio.h"
#include "radio.h"
#include "functions.h"
#include "settings.h"
#include "misc.h"
#ifdef ENABLE_SARSAT
#include "app/sarsat.h"     /* SARSAT_ScreenOpen() -- don't beacon under F+8  */
#endif

/* EEPROM: 40 bytes right after app/afgain.h's own 8-byte claim (0x0A170), in
 * the same unclaimed tail of the "Settings" PY25Q16 sector -- see afgain.c
 * for the full explanation of why an address needs an explicit ADDR_MAPPINGS
 * entry on this firmware (build.sh wires this one into
 * App/driver/eeprom_compat.c the same way). Same physical sector as the core
 * radio settings and the AF-gain byte, so it survives a normal reset and is
 * only wiped by "reset ALL". */
#define APRS_EE_ADDR        0x0A178u
#define APRS_EE_MAGIC       0xA5
#define APRS_TXDELAY_FLAGS  40          /* ~0.27 s of preamble tone     */
#define APRS_DEFAULT_FREQ   14480000u   /* 144.800000 MHz, 10 Hz units: the
                                        * channel-170 default the first time
                                        * it is ever seen erased           */
#define AFSK_MARK           1200
#define AFSK_SPACE          2200
#define AFSK_TICKS_PER_BIT  40000u      /* 48 MHz SysTick / 1200 baud          */
#define AFSK_TONE_GAIN      64          /* REG_70 tone1 gain 0..127; deviation.
                                        * roger beep uses 66, DTMF 65. Lower
                                        * this if the frame sounds over-deviated */

aprs_cfg_t gAprsCfg;
bool       gAprsShowRequest;

#define APRS_RX_CHARS 18                /* UI_PrintStringSmall* row limit    */
static char     s_rx[4][APRS_RX_CHARS + 1];  /* raw-text fallback (0x06D2)   */
static uint8_t  s_rxn;
static uint16_t s_rx_pkts;              /* total packets shown by the RP2040 */
static bool     s_rx_dirty;             /* a new packet arrived             */

/* structured decode from the RP2040 (0x06D3). s_rxi.kind mirrors the RP2040's
 * aprs_parse.h enum. */
#define APRS_RX_KIND_OTHER      0
#define APRS_RX_KIND_POSITION   1
#define APRS_RX_KIND_OBJECT     2
#define APRS_RX_KIND_STATUS     3
#define APRS_RX_KIND_MESSAGE    4
#define APRS_RX_KIND_TELEMETRY  5
static struct {
    bool     valid;
    uint8_t  kind, flags;
    char     sym_t, sym_c;
    int32_t  lat, lon;
    uint16_t course, speed, dist_hm, bearing;
    int16_t  alt;
    char     src[10], name[10], via[28], text[44];
    uint32_t at_10ms;
} s_rxi;
static uint32_t s_autopop_block_10ms;   /* no auto-popup until this time    */
static uint32_t s_bl_off_10ms;          /* "light on frame": dim at this time */
static uint32_t s_bl_on_until_10ms;     /* "light on frame": keep reasserting
                                         * BACKLIGHT_TurnOn() until this time --
                                         * a single one-shot call on the RX
                                         * event alone was reported not to
                                         * stick (screen dark through most of
                                         * the popup, only lighting right as
                                         * it closed). Covers background RX
                                         * too (no popup open); the in-popup
                                         * loop has its own per-iteration
                                         * reassertion, see APP_RunAprs(). */
static uint32_t s_next_beacon_10ms;
static bool     s_inited;                /* APRS_Init() has run once        */
extern uint32_t millis10(void);          /* app/scheduler.c                 */

/* ------------------------------------------------------------------ config */
static void APRS_Defaults(void)
{
    memset(&gAprsCfg, 0, sizeof(gAprsCfg));
    gAprsCfg.magic      = APRS_EE_MAGIC;
    strcpy(gAprsCfg.call, "NOCALL");
    gAprsCfg.ssid       = 9;
    gAprsCfg.path       = APRS_PATH_W1W2;
    gAprsCfg.sym_table  = '/';
    gAprsCfg.sym_code   = '>';           /* car */
    gAprsCfg.interval_s = 0;             /* off by default (needs a callsign) */
    gAprsCfg.popup_s    = 10;            /* auto RX-popup 10 s                 */
    gAprsCfg.opts       = (1u << APRS_OPT_SQL_SHIFT);  /* fast squelch, BL stock */
    gAprsCfg.lat_e5     = 0;
    gAprsCfg.lon_e5     = 0;
    strcpy(gAprsCfg.comment, "UV-K5_Sarsat");
}

/* Reserve MR channel 170 (index APRS_TX_CHANNEL) as the fixed APRS TX slot,
 * the way KD8CEC's own C-Board project does. Populated with sane defaults
 * only the first time it is seen still erased (0xFFFFFFFF frequency) -- a
 * user edit through the normal channel menu (frequency/power/mode/width) is
 * never overwritten afterwards. The name is always forced to "APRS" (never
 * left as "CH170"/"MR 170"), including on a channel the user already had
 * something else stored in, since this slot is meant to be dedicated. */
static void APRS_EnsureChannel(void)
{
    uint32_t freq;
    EEPROM_ReadBuffer(APRS_TX_CHANNEL * 16u, &freq, sizeof(freq));
    if (freq == 0xFFFFFFFFu) {
        VFO_Info_t v;
        RADIO_InitInfo(&v, APRS_TX_CHANNEL, APRS_DEFAULT_FREQ);
        v.OUTPUT_POWER = OUTPUT_POWER_MID;
        SETTINGS_SaveChannel(APRS_TX_CHANNEL, 0, &v, 2);
    }
    char nm[11];
    SETTINGS_FetchChannelName(nm, APRS_TX_CHANNEL);
    if (strcmp(nm, "APRS") != 0)
        SETTINGS_SaveChannelName(APRS_TX_CHANNEL, "APRS");
}

static void APRS_PushConfig(void);   /* forward decl: used by APRS_Init() below */

void APRS_Init(void)
{
    EEPROM_ReadBuffer(APRS_EE_ADDR, &gAprsCfg, sizeof(gAprsCfg));
    if (gAprsCfg.magic != APRS_EE_MAGIC || gAprsCfg.call[0] == 0xFF)
        APRS_Defaults();
    APRS_EnsureChannel();
    s_next_beacon_10ms = millis10() + 6000;   /* first beacon 60 s after boot */
    s_inited = true;
    AFGAIN_ResyncKnob();
    AFGAIN_Apply();
    APRS_PushConfig();   /* the RP2040 has no persistent state of its own --
                          * it needs this after every one of its own reboots,
                          * not only after a menu edit (see APRS_PushConfig()
                          * itself, called there too on save). */
}

/* APRS_Init() is not wired into the boot path; run it lazily the first time
 * anything touches the config (the beacon tick, the screen, an RX frame) so
 * gAprsCfg is never left as zeroed BSS with the EEPROM unread. */
static void APRS_Ensure(void)
{
    if (!s_inited)
        APRS_Init();
}

static void APRS_Save(void)
{
    gAprsCfg.magic = APRS_EE_MAGIC;
    for (unsigned i = 0; i < sizeof(gAprsCfg); i += 8)
        EEPROM_WriteBuffer(APRS_EE_ADDR + i, (const uint8_t *)&gAprsCfg + i);
}

void APRS_SaveConfig(void) { APRS_Save(); }   /* public wrapper */

/* made non-static by patch/app_uart.c.diff (equivalent), same as sarsat.c */
extern void SendReply(uint32_t Port, void *pReply, uint16_t Size);

/* Push the config to the RP2040: call/ssid (used for traceable digipeating --
 * stamped into a fully-consumed WIDEn-N slot instead of an anonymous
 * "WIDEn-0*", see aprs_digi.h), path/symbol (kept for our own beacon,
 * currently unused RP2040-side) and digi_level. Called on every
 * menu save AND from APRS_Init() -- the RP2040 keeps no state across its own
 * reboots, so it must get this fresh at least once after either side resets,
 * not only when the user happens to touch the menu. No ACK expected (see the
 * file header note). */
static void APRS_PushConfig(void)
{
    uint8_t b[4 + 11];
    b[0] = APRS_CMD_CONFIG & 0xFF; b[1] = APRS_CMD_CONFIG >> 8;
    b[2] = 11;                     b[3] = 0;
    memcpy(b + 4, gAprsCfg.call, 6);
    b[10] = gAprsCfg.ssid;
    b[11] = gAprsCfg.path;
    b[12] = gAprsCfg.sym_table;
    b[13] = gAprsCfg.sym_code;
    b[14] = (uint8_t)((gAprsCfg.opts & APRS_OPT_DIGI_MASK) >> APRS_OPT_DIGI_SHIFT);
    SendReply(UART_PORT_UART, b, sizeof(b));
}

/* ------------------------------------------- GPS fix from the C-Board (0x06D5) */
static struct {
    bool     seen;         /* at least one 0x06D5 frame ever received         */
    bool     valid;
    int32_t  lat_e5, lon_e5;
    uint16_t speed_kmh, course;
    int16_t  alt;
    uint8_t  sats;
    uint32_t at_10ms;
} s_gps;

bool APRS_GpsFixValid(void)
{
    return s_gps.valid && (int32_t)(millis10() - s_gps.at_10ms) < 1500;  /* 15 s */
}

/* top-bar indicator: 0 = no C-Board GPS frames, 1 = module present but no
 * lock (blinks), 2 = locked (solid). The RP2040 sends 0x06D5 ~every 3 s only
 * while a module is actually talking, so "no frame for 15 s" == no GPS. */
uint8_t APRS_GpsState(void)
{
    if (!s_gps.seen || (int32_t)(millis10() - s_gps.at_10ms) >= 1500)
        return 0;
    return s_gps.valid ? 2 : 1;
}

/* the position to beacon / report: a fresh GPS fix if "Pos GPS" is set, else
 * the manually entered lat/lon. */
void APRS_MyPosition(int32_t *lat_e5, int32_t *lon_e5)
{
    if ((gAprsCfg.opts & APRS_OPT_GPS) && APRS_GpsFixValid()) {
        *lat_e5 = s_gps.lat_e5;
        *lon_e5 = s_gps.lon_e5;
    } else {
        *lat_e5 = gAprsCfg.lat_e5;
        *lon_e5 = gAprsCfg.lon_e5;
    }
}

/* ------------------------------------------------------------ APRS strings */
/* "DDMM.mm" (lat: 2 deg digits, lon: 3) into `p`, returns chars written */
static int fmt_dmm(char *p, int32_t e5, int degdigits, char pos, char neg)
{
    uint32_t a  = (e5 < 0) ? -e5 : e5;
    uint32_t d  = a / 100000u;
    uint32_t mmhh = ((a % 100000u) * 60u) / 1000u;   /* MMHH, i.e. min*100  */
    return sprintf(p, degdigits == 3 ? "%03u%02u.%02u%c" : "%02u%02u.%02u%c",
                   (unsigned)d, (unsigned)(mmhh / 100), (unsigned)(mmhh % 100),
                   (e5 < 0) ? neg : pos);
}

static int APRS_FormatPosition(char *info)
{
    int32_t lat, lon;
    APRS_MyPosition(&lat, &lon);
    const bool gps = (gAprsCfg.opts & APRS_OPT_GPS) && APRS_GpsFixValid();

    int n = 0;
    info[n++] = '!';
    n += fmt_dmm(info + n, lat, 2, 'N', 'S');
    info[n++] = gAprsCfg.sym_table;
    n += fmt_dmm(info + n, lon, 3, 'E', 'W');
    info[n++] = gAprsCfg.sym_code;
    /* CSE/SPD data extension (course deg / speed knots) when moving on GPS */
    if (gps && s_gps.speed_kmh > 3)
        n += sprintf(info + n, "%03u/%03u", (unsigned)(s_gps.course % 360u),
                     (unsigned)((s_gps.speed_kmh * 100u + 92u) / 185u));  /* km/h->kn */
    n += sprintf(info + n, "%.*s", (int)sizeof(gAprsCfg.comment),
                 gAprsCfg.comment);
    return n;
}

/* ------------------------------------------------------------ AFSK on air  */
static uint16_t afsk_scale(uint16_t f)
{
    return (uint16_t)((((uint32_t)f * 1353245u) + (1u << 16)) >> 17);
}

/* Inject the Bell-202 AFSK into the carrier that RADIO_SetTxParameters() has
 * already brought up. Same sequence the firmware uses for the roger beep /
 * DTMF-during-TX (BK4819_PlayRogerNormal, BK4819_PlayDTMFEx):
 *   mute -> REG_70 tone1 -> EnableTXLink -> settle -> set REG_71 -> unmute.
 * BK4819_EnableTXLink() is what puts REG_30 into the TX-DSP state with
 * ENABLE_PA_GAIN / ENABLE_PLL_VCO; writing REG_30 by hand without those
 * collapsed the PA ("keys up but nothing radiates") on the V1 port.
 *
 * Timing: BK4819_WriteRegister is a ~40 us bit-banged transfer, so the naive
 * "write REG_71 every bit + delay" loop runs slow and jitters -- unlockable
 * for a real TNC (found on the V1 port). Instead: write REG_71 only on an
 * actual tone change (NRZI runs stay phase-continuous) and clock the bit
 * cells off a free-running SysTick deadline that absorbs the write time and
 * self-corrects for IRQ jitter, so the average stays exactly 1200 baud. */
static void APRS_SendTones(const uint8_t *tones, int nb)
{
    const uint16_t r_mark  = afsk_scale(AFSK_MARK);
    const uint16_t r_space = afsk_scale(AFSK_SPACE);
    const uint32_t reload  = SysTick->LOAD;      /* 479999 (10 ms window)     */

    BK4819_EnterTxMute();
    BK4819_SetAF(BK4819_AF_MUTE);
    BK4819_WriteRegister(BK4819_REG_70,
        BK4819_REG_70_ENABLE_TONE1 |
        (AFSK_TONE_GAIN << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));
    BK4819_EnableTXLink();               /* REG_30: TX DSP + PA_GAIN + PLL_VCO */
    SYSTEM_DelayMs(50);

    int cur = tones[0] ? 1 : 0;
    BK4819_WriteRegister(BK4819_REG_71, cur ? r_mark : r_space);
    BK4819_ExitTxMute();

    uint32_t last = SysTick->VAL;        /* SysTick is a 24-bit down-counter  */
    uint32_t acc  = 0;
    for (int i = 0; i < nb; i++) {
        const int want = tones[i] ? 1 : 0;
        if (want != cur) {
            BK4819_WriteRegister(BK4819_REG_71, want ? r_mark : r_space);
            cur = want;
        }
        while (acc < AFSK_TICKS_PER_BIT) {
            const uint32_t now = SysTick->VAL;
            acc += (last >= now) ? (last - now)
                                 : (last + reload + 1 - now);
            last = now;
        }
        acc -= AFSK_TICKS_PER_BIT;       /* carry the remainder to the next bit */
    }

    BK4819_EnterTxMute();
    BK4819_WriteRegister(BK4819_REG_70, 0x0000);
    BK4819_WriteRegister(BK4819_REG_30, 0xC1FE);   /* carrier, tone off */
}

static int APRS_Digi(ax25_addr_t *d)
{
    switch (gAprsCfg.path) {
    case APRS_PATH_W1:  strcpy(d[0].call, "WIDE1"); d[0].ssid = 1; return 1;
    case APRS_PATH_W2:  strcpy(d[0].call, "WIDE2"); d[0].ssid = 1; return 1;
    case APRS_PATH_W1W2:
        strcpy(d[0].call, "WIDE1"); d[0].ssid = 1;
        strcpy(d[1].call, "WIDE2"); d[1].ssid = 1; return 2;
    default: return 0;
    }
}

/* Key up on channel 170's own config (frequency / power / mode / bandwidth /
 * offset / CTCSS), never on whatever VFO A/B are currently tuned to, send
 * `frame` (flen bytes, FCS already included) as Bell-202 AFSK, then restore
 * the borrowed VFO slot -- shared by APRS_Beacon() (below) and
 * APRS_Digipeat() so both go out identically. Borrow the TX_VFO slot: save
 * its live content, reload it from channel 170 (RADIO_ConfigureChannel(),
 * the same reader the UI uses when switching to a memory channel), key up
 * on that -- same way FUNCTION_Transmit() does: DTMF off (it retunes the TX
 * audio filter), RADIO_SetTxParameters() (frequency + PA + PrepareTransmit),
 * red LED -- then restore the slot exactly as it was before going back to
 * RX. */
static void APRS_TxFrame(const uint8_t *frame, int flen)
{
    static uint8_t tones[AX25_MAX_BITS];
    int nb = ax25_hdlc_nrzi(frame, flen, APRS_TXDELAY_FLAGS, tones);
    if (!nb) return;

    const uint8_t     txv        = gEeprom.TX_VFO;
    const VFO_Info_t  saved_vfo  = gEeprom.VfoInfo[txv];
    const uint8_t     saved_sc   = gEeprom.ScreenChannel[txv];
    const uint8_t     saved_mrc  = gEeprom.MrChannel[txv];

    gEeprom.ScreenChannel[txv] = APRS_TX_CHANNEL;
    RADIO_ConfigureChannel(txv, VFO_CONFIGURE_RELOAD);   /* re-read channel 170
                                                          * from EEPROM -- plain
                                                          * VFO_CONFIGURE would
                                                          * keep the slot's old
                                                          * (borrowed) content */

    RADIO_SelectVfos();
    gCurrentVfo = gTxVfo;
    BK4819_DisableDTMF();
    RADIO_SetTxParameters();
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
    SYSTEM_DelayMs(20);

    APRS_SendTones(tones, nb);

    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);

    gEeprom.VfoInfo[txv]       = saved_vfo;
    gEeprom.ScreenChannel[txv] = saved_sc;
    gEeprom.MrChannel[txv]     = saved_mrc;
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);          /* back to RX, exactly as before */
}

/* Gates shared by originating a beacon and repeating someone else's frame:
 * never while actually transmitting, never over the SARSAT screen, never
 * while the channel is busy (CSMA -- the caller retries on the next slice
 * for a beacon; a missed digipeat opportunity is simply dropped, which is
 * safe). Beaconing additionally requires a real callsign (never transmit
 * under the NOCALL default) and, in GPS mode, a fresh fix -- digipeating
 * does not: the repeated frame's own source field already identifies the
 * originating station, this radio is only relaying it, not originating it. */
static bool APRS_CanTransmitNow(void)
{
    if (gCurrentFunction == FUNCTION_TRANSMIT)
        return false;
#ifdef ENABLE_SARSAT
    /* Never key up while the SARSAT screen owns the radio: it forces its own
     * FM+WIDE+REG2B receive profile on the currently selected VFO for the
     * C-Board decoder, and a mid-monitoring jump to channel 170 for TX would
     * fight that. In practice APRS_TimeSlice() (the only caller of the
     * auto-beacon path) is not ticked while APP_RunSarsat()'s loop is
     * blocking anyway -- this is belt and suspenders, and covers a future
     * caller too. */
    if (SARSAT_ScreenOpen())
        return false;
#endif
    /* CSMA: never key up while a carrier is present. g_SquelchLost (not
     * gCurrentFunction) is the right test here: per functions.h,
     * FUNCTION_RECEIVE is actually "squelch closed" (idle) and the state
     * that means "a signal is present" is FUNCTION_INCOMING, never tested
     * by the check this replaces. More fundamentally, gCurrentFunction is
     * only updated by the main 10 ms tick, which APP_RunAprs()'s own popup
     * loop blocks for as long as it stays open -- it would sit frozen at
     * whatever it was on entry (almost always mid-reception, since the
     * popup opens ON a decoded packet) regardless of the real channel
     * state. g_SquelchLost stays live in both contexts: normally via that
     * same tick, and inside the popup via its own inline mini-squelch poll
     * (see APP_RunAprs()'s loop) -- the likely cause of "no repetition
     * while the popup is open", reported on air with the previous check. */
    if (g_SquelchLost)
        return false;
    return true;
}

void APRS_Beacon(void)
{
    if (gAprsCfg.call[0] == 0 || strcmp(gAprsCfg.call, "NOCALL") == 0)
        return;
    if (!APRS_CanTransmitNow())
        return;

    /* GPS position mode with no fix yet: don't beacon a stale / zero position */
    if ((gAprsCfg.opts & APRS_OPT_GPS) && !APRS_GpsFixValid())
        return;

    char info[AX25_MAX_INFO + 4];
    int  ilen = APRS_FormatPosition(info);
    if (ilen > AX25_MAX_INFO) ilen = AX25_MAX_INFO;

    ax25_addr_t src, dst, digi[AX25_MAX_DIGI];
    memset(&src, 0, sizeof src); memset(&dst, 0, sizeof dst);
    memcpy(src.call, gAprsCfg.call, 6); src.ssid = gAprsCfg.ssid;
    strcpy(dst.call, "APZSAR");          dst.ssid = 0;
    int nd = APRS_Digi(digi);

    uint8_t frame[AX25_MAX_FRAME];
    int flen = ax25_build_ui(frame, &dst, &src, digi, nd, info, ilen);
    if (!flen) return;

    APRS_TxFrame(frame, flen);
}

/* AX25_MAX_FRAME (112 o) is sized for OUR OWN beacon (2 digis, short info) --
 * a frame the RP2040 hands back for digipeating can have a longer path/info
 * than that, up to the 256 o it itself caps at (rp2040/src/main.c). Sized
 * generously enough for real-world APRS traffic without following the
 * RP2040's own much larger internal margin (APRS_RX_MAX_FRAME=330, sized for
 * its own worst-case demod buffer, not for what any real packet needs). A
 * local (stack) buffer, not static -- this firmware's RAM is already tight. */
#define APRS_DIGI_MAX_FRAME (256 + 2)

/* APRS_CMD_DIGI arrives right on the tail of the very burst it repeats --
 * the radio is still virtually guaranteed to be mid-RX/monitor (squelch
 * hang time) at that exact instant, so a single immediate
 * APRS_CanTransmitNow() attempt lost the CSMA race essentially every time
 * (confirmed on air: the RP2040 logged "repeating", nothing went out).
 * APRS_Beacon() dodges this by being retried every ~3 s from
 * APRS_TimeSlice() until the channel clears -- queue the digipeat frame the
 * same way instead of firing it once from APRS_HandleUART(). One slot is
 * enough: digipeat bursts are sparse relative to the ~3 s beacon retry
 * cadence this reuses. */
static struct {
    uint8_t  data[APRS_DIGI_MAX_FRAME - 2];
    uint16_t len;
    bool     pending;
    uint32_t queued_at_10ms;
} s_digi_pending;

#define APRS_DIGI_MIN_DELAY_10MS 30  /* let the channel settle ~300 ms before
                                      * keying up -- avoids clipping the tail
                                      * of the frame just repeated / the
                                      * originating station's own RX turnaround */
#define APRS_DIGI_MAX_AGE_10MS   300 /* give up after ~3 s of a busy channel */

/* APRS_CMD_DIGI: `data` is a raw AX.25 frame (addr..ctrl..pid..info) the
 * RP2040 has already decided is worth repeating and mutated in place
 * (aprs_digi.h on that side) -- FCS is NOT included, APRS_DigipeatTimeSlice()
 * appends it once it actually transmits. No callsign gate: see
 * APRS_CanTransmitNow()'s comment. */
static void APRS_Digipeat(const uint8_t *data, uint16_t size)
{
    if (size < 16 || size > sizeof(s_digi_pending.data))
        return;   /* too short to be a real frame, or too long for our buffer */

    memcpy(s_digi_pending.data, data, size);
    s_digi_pending.len            = size;
    s_digi_pending.pending        = true;
    s_digi_pending.queued_at_10ms = millis10();
}

/* Called every ~10 ms from APRS_TimeSlice(), unconditionally. */
static void APRS_DigipeatTimeSlice(void)
{
    if (!s_digi_pending.pending)
        return;
    int32_t age = (int32_t)(millis10() - s_digi_pending.queued_at_10ms);
    if (age > APRS_DIGI_MAX_AGE_10MS) {
        s_digi_pending.pending = false;   /* channel stayed busy too long: a
                                           * stale repeat is worse than none */
        return;
    }
    if (age < APRS_DIGI_MIN_DELAY_10MS)
        return;                           /* still settling, try again next tick */
    if (!APRS_CanTransmitNow())
        return;                           /* try again next tick */

    s_digi_pending.pending = false;
    uint8_t frame[APRS_DIGI_MAX_FRAME];
    memcpy(frame, s_digi_pending.data, s_digi_pending.len);
    uint16_t fcs = ax25_fcs(frame, s_digi_pending.len);
    frame[s_digi_pending.len]     = (uint8_t)(fcs & 0xFF);
    frame[s_digi_pending.len + 1] = (uint8_t)(fcs >> 8);

    APRS_TxFrame(frame, s_digi_pending.len + 2);
}

void APRS_TimeSlice(void)
{
    APRS_Ensure();
    APRS_ApplySquelch();               /* APRS-band fast-squelch (cheap: 1 reg read) */
    AFGAIN_TimeSlice();                /* keep a fixed C-Board AF gain in effect
                                        * everywhere, screen open or not     */
    APRS_DigipeatTimeSlice();          /* retry a queued digipeat frame until
                                        * the channel is actually clear      */

    /* "light on frame", background RX (no popup open, e.g. popup disabled or
     * in its post-close cooldown): keep reasserting the backlight for a
     * short window after each decode -- see s_bl_on_until_10ms's comment. */
    if (s_bl_on_until_10ms && (int32_t)(millis10() - s_bl_on_until_10ms) < 0)
        BACKLIGHT_TurnOn();

    /* "light on frame": kill the backlight ~1 s after an auto-popup closes so
     * the screen goes dark again between packets instead of holding the full
     * backlight time. Armed on the popup's exit; cleared by a fresh frame. */
    if (s_bl_off_10ms && (int32_t)(millis10() - s_bl_off_10ms) >= 0) {
        s_bl_off_10ms = 0;
        if (gScreenToDisplay == DISPLAY_MAIN)
            BACKLIGHT_TurnOff();
    }

    /* keep the top-bar GPS symbol animated: redraw the status line on any
     * state change, and ~2x/s while "searching" so it blinks. */
    {
        static uint8_t s_gps_shown, s_gps_blink;
        uint8_t g = APRS_GpsState();
        if (g != s_gps_shown) { s_gps_shown = g; gUpdateStatus = true; }
        if (g == 1 && (uint8_t)(millis10() - s_gps_blink) >= 25) {
            s_gps_blink = (uint8_t)millis10();
            gUpdateStatus = true;
        }
    }

    if (gAprsCfg.interval_s == 0)
        return;
    if ((int32_t)(millis10() - s_next_beacon_10ms) < 0)
        return;

    /* channel busy (RX in progress) or already transmitting: hold off ~3 s */
    if (gCurrentFunction == FUNCTION_TRANSMIT ||
        gCurrentFunction == FUNCTION_RECEIVE  ||
        gCurrentFunction == FUNCTION_MONITOR) {
        s_next_beacon_10ms = millis10() + 300;
        return;
    }

    s_next_beacon_10ms = millis10() + gAprsCfg.interval_s * 100u;
    APRS_Beacon();
}

/* RX VFO in the 144-148 MHz APRS band? (10 Hz freq units) */
static bool aprs_on_band(void)
{
    uint32_t f = gEeprom.VfoInfo[gEeprom.RX_VFO & 1u].freq_config_RX.Frequency;
    return f >= 14400000u && f <= 14800000u;
}

static void aprs_rx_arrived(void)      /* common: count + rate-limited popup */
{
    s_rx_pkts++;
    s_rx_dirty = true;
    if (gAprsCfg.opts & APRS_OPT_BL_DECODE) {  /* a decoded frame lights the screen */
        BACKLIGHT_TurnOn();
        s_bl_off_10ms     = 0;                 /* cancel a pending post-popup dim */
        s_bl_on_until_10ms = millis10() + 150; /* ~1.5 s -- APRS_TimeSlice() keeps
                                                 * reasserting it until then, see
                                                 * s_bl_on_until_10ms's comment */
    }
    if (gAprsCfg.popup_s != 0 &&
        (int32_t)(millis10() - s_autopop_block_10ms) >= 0)
        gAprsShowRequest = true;
}

/* Called from APP_StartListening (very hot): on the APRS band with the
 * "light on frame" option set, the RX-squelch backlight is suppressed -- only
 * aprs_rx_arrived() lights it. Safe before APRS_Init() (gAprsCfg is zeroed). */
bool APRS_QuietBacklight(void)
{
    /* "Light on frame" is meant to pair with the RX popup (dim between
     * packets, light on a decode, matching the popup opening) -- it used to
     * apply band-wide regardless of the Popup setting, so turning popups
     * OFF while leaving this toggled on left the backlight quiet on plain
     * squelch-open forever, with no popup ever there to justify it. Now
     * tied to popup_s != 0 too: with popups off, stock RX backlight is
     * restored, whatever this toggle says. */
    return (gAprsCfg.opts & APRS_OPT_BL_DECODE) && gAprsCfg.popup_s != 0 &&
           aprs_on_band();
}

/* Block the battery-save on the APRS band: FUNCTION_POWER_SAVE cycles the
 * receiver off, which chops the audio the C-Board needs to demodulate a
 * packet. Wired into the gSchedulePowerSave inhibit list in app/app.c, same
 * idea as APRS_KeepAwake() on the SARSAT screen's own inhibit hook (this one
 * is band-based, not screen-based: it applies any time the radio sits on
 * 144-148 MHz, not just while a screen is open). Only while the RX VFO sits
 * in 144-148 MHz -> no effect on normal use. */
bool APRS_KeepAwake(void)
{
    return aprs_on_band();
}

/* APRS-band "fast squelch" (REG_4E): the stock BK4819 setup uses a long
 * squelch OPEN delay (bits 13:11), so the receiver takes tens of ms to
 * un-mute and the first AX.25 flags of a packet are lost. On 144-148 MHz set:
 *   - open  delay 0 (bits 13:11) -> un-mutes on the first flag
 *   - close delay 3 = max (bits 10:9) -> once open it HOLDS through the packet
 * (a close delay of 1 let the squelch snap shut on the brief amplitude dips
 * inside an AFSK burst on the V1 port -- every chatter blanks ~10-20 ms =
 * 12-24 bits, wrecking the HDLC framing; kept at 3 here from the start).
 * Re-asserted every tick because a VFO re-config rewrites REG_4E. Only
 * touches the two delay fields, not the RSSI/noise/glitch thresholds
 * themselves -- unlike the "force squelch always open" attempt that
 * regressed the SARSAT screen on this firmware (see patch/sarsat.c), this is
 * the same narrow register slice the V1 port already validated on air. */
void APRS_ApplySquelch(void)
{
    if (((gAprsCfg.opts & APRS_OPT_SQL_MASK) >> APRS_OPT_SQL_SHIFT) == 0 ||
        !aprs_on_band())
        return;
    uint16_t r = BK4819_ReadRegister(BK4819_REG_4E);
    uint16_t want = (uint16_t)((r & ~0x3E00u) | (0u << 11) | (3u << 9));
    if (want != r)
        BK4819_WriteRegister(BK4819_REG_4E, want);
}

static int16_t rd16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
static int32_t rd32(const uint8_t *p)
{
    return (int32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24));
}
static const uint8_t *rd_str(const uint8_t *p, const uint8_t *end,
                             char *out, int cap)
{
    int k = 0;
    while (p < end && *p) {                        /* consume the whole string */
        if (k < cap - 1) out[k++] = (char)*p;      /* ...even if it overflows  */
        p++;
    }
    out[k] = 0;
    return (p < end) ? p + 1 : end;               /* skip the NUL */
}

/* --------------------------------------------- RX from the RP2040 (0x06Dx) */
void APRS_HandleUART(uint16_t id, const uint8_t *data, uint16_t size)
{
    APRS_Ensure();

    /* suppress a spurious mic-line PTT while the C-Board pushes the packet
     * (a few-ms key-up was seen right after an APRS decode on 144.8, on the
     * V1 port) */
    gSerialConfigCountDown_500ms = 2;

    if (id == APRS_CMD_RXINFO && size >= 22) {
        const uint8_t *p = data, *end = data + size;
        s_rxi.kind    = p[0];
        s_rxi.flags   = p[1];
        s_rxi.sym_t   = (char)p[2];
        s_rxi.sym_c   = (char)p[3];
        s_rxi.lat     = rd32(p + 4);
        s_rxi.lon     = rd32(p + 8);
        s_rxi.course  = (uint16_t)rd16(p + 12);
        s_rxi.speed   = (uint16_t)rd16(p + 14);
        s_rxi.alt     = rd16(p + 16);
        s_rxi.dist_hm = (uint16_t)rd16(p + 18);
        s_rxi.bearing = (uint16_t)rd16(p + 20);
        p += 22;
        p = rd_str(p, end, s_rxi.src,  sizeof s_rxi.src);
        p = rd_str(p, end, s_rxi.name, sizeof s_rxi.name);
        p = rd_str(p, end, s_rxi.via,  sizeof s_rxi.via);
        p = rd_str(p, end, s_rxi.text, sizeof s_rxi.text);
        s_rxi.at_10ms = millis10();
        s_rxi.valid   = true;
        s_rxn = 0;                                 /* the text fallback is stale */
        aprs_rx_arrived();
        return;
    }

    if (id == APRS_CMD_GPS && size >= 15) {
        s_gps.seen      = true;
        s_gps.valid     = (data[0] & 1u) != 0;
        s_gps.lat_e5    = rd32(data + 1);
        s_gps.lon_e5    = rd32(data + 5);
        s_gps.speed_kmh = (uint16_t)rd16(data + 9);
        s_gps.course    = (uint16_t)rd16(data + 11);
        s_gps.alt       = rd16(data + 13);
        s_gps.sats      = (size >= 16) ? data[15] : 0;
        s_gps.at_10ms   = millis10();
        return;
    }

    if (id == APRS_CMD_RXTEXT && size >= 1) {
        uint8_t line = data[0];
        if (line == 0xFF) { s_rxn = 0; s_rxi.valid = false; s_rx_dirty = true; return; }
        if (line >= 4) return;
        uint16_t n = size - 1;
        if (n > APRS_RX_CHARS) n = APRS_RX_CHARS;
        memset(s_rx[line], 0, sizeof(s_rx[line]));
        memcpy(s_rx[line], data + 1, n);
        if (line + 1 > s_rxn) s_rxn = line + 1;
        if (line == 0) { s_rxi.valid = false; aprs_rx_arrived(); }
        else s_rx_dirty = true;
        return;
    }

    if (id == APRS_CMD_DIGI) {
        APRS_Digipeat(data, size);
        return;
    }
}

/* SetNav (menu Service) : sur l'UV-K1 les touches physiques UP/DOWN sont
 * etiquetees LEFT/RIGHT et le firmware inverse deja partout ailleurs
 * (App/app/menu.c, main.c, scanner.c, spectrum.c...) le sens de UP/DOWN
 * quand gEeprom.SET_NAV est faux (defaut sur UV-K1, cf. App/settings.c) --
 * on suit la meme convention ici pour que l'ecran config APRS navigue dans
 * le meme sens que le reste du firmware sur ce boitier. */
static KEY_Code_t nav_key(KEY_Code_t k)
{
    if (gEeprom.SET_NAV || (k != KEY_UP && k != KEY_DOWN))
        return k;
    return (k == KEY_UP) ? KEY_DOWN : KEY_UP;
}

/* ---------------------------------------------------------- config screen  */
enum { F_CALL, F_SSID, F_PATH, F_SYM, F_TEXT, F_INT, F_POPUP, F_AFGAIN, F_SQL,
       F_BLIGHT, F_DIGI, F_POS, F_LAT, F_LON, F_N };

/* char cycling for the keypad-poor text fields: space, A-Z, 0-9, then a few
 * punctuation marks for the comment. */
static const char CALLSET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
static const char TEXTSET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_./+";
static char cyc_char(char c, int dir, const char *set)
{
    const char *p = strchr(set, (c == 0) ? ' ' : c);
    int n = (int)strlen(set);
    int i = p ? (int)(p - set) : 0;
    return set[(i + dir + n) % n];
}
static const char *const PATHS[] = { "none", "WIDE1-1", "WIDE2-1", "WIDE1+2" };
/* a few common symbols: code, label */
static const struct { char t, c; const char *n; } SYMS[] = {
    { '/', '>', "car"    }, { '/', '-', "house"  }, { '/', 'k', "truck" },
    { '/', '[', "runner" }, { '/', 'b', "bike"   }, { '/', 'Y', "yacht" },
    { '/', '\'',"plane"  }, { '/', '_', "wx"     }, { '/', '.', "dot"   },
};
#define NSYM (int)(sizeof(SYMS)/sizeof(SYMS[0]))

static int sym_index(void)
{
    for (int i = 0; i < NSYM; i++)
        if (SYMS[i].t == gAprsCfg.sym_table && SYMS[i].c == gAprsCfg.sym_code)
            return i;
    return 0;
}

static void field_str(int f, char *out)
{
    switch (f) {
    case F_CALL: sprintf(out, "Call  %s", gAprsCfg.call); break;
    case F_SSID: sprintf(out, "SSID  %u", gAprsCfg.ssid); break;
    case F_PATH: sprintf(out, "Path  %s", PATHS[gAprsCfg.path < APRS_PATH_N ?
                                                gAprsCfg.path : 0]); break;
    case F_SYM:  sprintf(out, "Icon  %s", SYMS[sym_index()].n); break;
    case F_TEXT: sprintf(out, "Txt %.14s",
                         gAprsCfg.comment[0] ? gAprsCfg.comment : "(empty)"); break;
    case F_INT:
        if (gAprsCfg.interval_s == 0) strcpy(out, "Interval OFF");
        else sprintf(out, "Interval %us", gAprsCfg.interval_s);
        break;
    case F_POPUP:
        if (gAprsCfg.popup_s == 0) strcpy(out, "Popup OFF");
        else sprintf(out, "Popup %us", gAprsCfg.popup_s);
        break;
    case F_AFGAIN:
        if (gAfGain < 1 || gAfGain > 78) strcpy(out, "AF gain auto");
        else sprintf(out, "AF gain %u", gAfGain);
        break;
    case F_SQL: strcpy(out,
        ((gAprsCfg.opts & APRS_OPT_SQL_MASK) >> APRS_OPT_SQL_SHIFT)
            ? "Squelch fast" : "Squelch stock"); break;
    case F_BLIGHT: strcpy(out, (gAprsCfg.opts & APRS_OPT_BL_DECODE)
        ? "Light on frame" : "Light on RX"); break;
    case F_DIGI: {
        static const char *const D[] = { "Digi  off", "Digi  WIDE1",
                                          "Digi  WIDE1+2", "Digi  WIDE1+2+3" };
        uint8_t lv = (gAprsCfg.opts & APRS_OPT_DIGI_MASK) >> APRS_OPT_DIGI_SHIFT;
        strcpy(out, D[lv <= 3 ? lv : 0]);
        break;
    }
    case F_POS:
        if (!(gAprsCfg.opts & APRS_OPT_GPS)) strcpy(out, "Pos   manual");
        else if (APRS_GpsFixValid())  sprintf(out, "Pos   GPS %usat", s_gps.sats);
        else                          strcpy(out, "Pos   GPS nofix");
        break;
    case F_LAT: {
        long v = gAprsCfg.lat_e5, a = v < 0 ? -v : v;
        sprintf(out, "Lat  %s%ld.%05lu", v < 0 ? "-" : "",
                (long)(a / 100000), (unsigned long)(a % 100000));
        break;
    }
    case F_LON: {
        long v = gAprsCfg.lon_e5, a = v < 0 ? -v : v;
        sprintf(out, "Lon  %s%ld.%05lu", v < 0 ? "-" : "",
                (long)(a / 100000), (unsigned long)(a % 100000));
        break;
    }
    }
}

/* --- keypad digit entry for Lat / Lon ------------------------------------- */
static char s_kbuf[8];      /* typed digits, degrees then fraction           */
static int  s_klen;
static int  s_kneg;         /* 1 = S / W                                     */

static void kbuf_render(char *out, int degdigits)
{
    int p = 0;
    out[p++] = s_kneg ? '-' : '+';
    for (int i = 0; i < degdigits + 5; i++) {
        if (i == degdigits) out[p++] = '.';
        out[p++] = (i < s_klen) ? s_kbuf[i] : '_';
    }
    out[p] = 0;
}

static void kbuf_commit(int f)
{
    if (s_klen == 0)
        return;
    int  degdigits = (f == F_LAT) ? 2 : 3;
    long deg = 0, frac = 0, mul = 10000;
    int  i = 0;
    for (; i < s_klen && i < degdigits; i++) deg = deg * 10 + (s_kbuf[i] - '0');
    for (; i < s_klen && mul > 0; i++) { frac += (s_kbuf[i] - '0') * mul; mul /= 10; }
    long e5 = deg * 100000 + frac;
    if (s_kneg) e5 = -e5;
    if (f == F_LAT) {
        if (e5 >  9000000) e5 =  9000000;
        if (e5 < -9000000) e5 = -9000000;
        gAprsCfg.lat_e5 = e5;
    } else {
        if (e5 >  18000000) e5 =  18000000;
        if (e5 < -18000000) e5 = -18000000;
        gAprsCfg.lon_e5 = e5;
    }
}

static void field_step(int f, int dir)
{
    static const uint16_t INTS[] = { 0, 30, 60, 120, 300, 600, 900, 1800 };
    switch (f) {
    case F_SSID: gAprsCfg.ssid = (gAprsCfg.ssid + dir + 16) & 15; break;
    case F_PATH: gAprsCfg.path = (gAprsCfg.path + dir + APRS_PATH_N) % APRS_PATH_N; break;
    case F_SYM: {
        int i = (sym_index() + dir + NSYM) % NSYM;
        gAprsCfg.sym_table = SYMS[i].t; gAprsCfg.sym_code = SYMS[i].c; break;
    }
    case F_INT: {
        int i = 0, n = (int)(sizeof(INTS)/sizeof(INTS[0]));
        for (; i < n; i++) if (INTS[i] == gAprsCfg.interval_s) break;
        i = (i + dir + n) % n;
        gAprsCfg.interval_s = INTS[i];
        break;
    }
    case F_POPUP: {
        static const uint8_t P[] = { 0, 5, 10, 20 };
        int i = 0, n = (int)(sizeof(P)/sizeof(P[0]));
        for (; i < n; i++) if (P[i] == gAprsCfg.popup_s) break;
        if (i >= n) i = 0;
        gAprsCfg.popup_s = P[(i + dir + n) % n];
        break;
    }
    case F_AFGAIN: {
        int v = (gAfGain >= 1 && gAfGain <= 78) ? gAfGain : 0;   /* 0 = auto */
        v += dir;
        if (v < 0)  v = 0;
        if (v > 78) v = 78;
        gAfGain = (uint8_t)v;
        AFGAIN_Apply();                    /* live */
        break;
    }
    case F_SQL:
        gAprsCfg.opts ^= (1u << APRS_OPT_SQL_SHIFT);   /* toggle stock <-> fast */
        APRS_ApplySquelch();
        break;
    case F_BLIGHT: gAprsCfg.opts ^= APRS_OPT_BL_DECODE; break;
    case F_DIGI: {
        uint8_t lv = (gAprsCfg.opts & APRS_OPT_DIGI_MASK) >> APRS_OPT_DIGI_SHIFT;
        lv = (uint8_t)((lv + dir + 4) % 4);
        gAprsCfg.opts = (uint8_t)((gAprsCfg.opts & ~APRS_OPT_DIGI_MASK) |
                                  (lv << APRS_OPT_DIGI_SHIFT));
        break;
    }
    case F_POS:    gAprsCfg.opts ^= APRS_OPT_GPS; break;   /* manual <-> GPS */
    case F_LAT:
        gAprsCfg.lat_e5 += dir * 100;  /* 0.001 deg step */
        if (gAprsCfg.lat_e5 >  9000000) gAprsCfg.lat_e5 =  9000000;
        if (gAprsCfg.lat_e5 < -9000000) gAprsCfg.lat_e5 = -9000000;
        break;
    case F_LON:
        gAprsCfg.lon_e5 += dir * 100;
        if (gAprsCfg.lon_e5 >  18000000) gAprsCfg.lon_e5 =  18000000;
        if (gAprsCfg.lon_e5 < -18000000) gAprsCfg.lon_e5 = -18000000;
        break;
    }
}

/* The LCD content area is only 7 rows (gFrameBuffer[0..6]; row 0 of the panel
 * is the separate status line). Row 0 here = header/help, rows 1..6 = a
 * scrolling window over the 7 fields F_CALL..F_LON -- writing a 7th field row
 * would land on gFrameBuffer[7], off the end of the buffer and off-screen
 * (that was the "Lon invisible" bug on the V1 port). `first` is the top
 * field of the window. */
#define APRS_VIS_ROWS 6

/* Every string reaching UI_PrintStringSmall* must be <= 18 glyphs -- it does
 * not clip and overruns gFrameBuffer[row] (and, on the last row, gEeprom). */
static void draw_config(int sel, int editing, int callcur, int first)
{
    char s[24];
    UI_DisplayClear();

    if (editing && sel == F_CALL)
        sprintf(s, "CALL char %d  A:ok", callcur + 1);
    else if (editing && sel == F_TEXT)
        sprintf(s, "TEXT pos %d UP/DN", callcur + 1);
    else if (editing && (sel == F_LAT || sel == F_LON))
        strcpy(s, "0-9  *:sign  A:ok");
    else if (editing)
        strcpy(s, "UP/DN then A:ok");
    else
        sprintf(s, "APRS 5:TX *:RX%c%c",
                first > 0 ? '^' : ' ',
                first + APRS_VIS_ROWS < F_N ? 'v' : ' ');
    UI_PrintStringSmallBold(s, 2, 0, 0);

    for (int r = 0; r < APRS_VIS_ROWS; r++) {
        int f = first + r;
        if (f >= F_N)
            break;
        if (editing && f == sel && (f == F_LAT || f == F_LON) && s_klen) {
            char v[16];
            kbuf_render(v, f == F_LAT ? 2 : 3);
            sprintf(s, "%s %s", f == F_LAT ? "Lat " : "Lon ", v);
        } else if (editing && f == sel && f == F_TEXT) {
            int w = 0;                            /* comment with a [x] cursor */
            for (int i = 0; i < 14; i++) {        /* 14 + 2 brackets = 16 <= 18 */
                char ch = gAprsCfg.comment[i] ? gAprsCfg.comment[i] : ' ';
                if (i == callcur) { s[w++] = '['; s[w++] = ch; s[w++] = ']'; }
                else              s[w++] = ch;
            }
            s[w] = 0;
        } else {
            field_str(f, s);
        }
        if (f == sel)
            UI_PrintStringSmallBold(s, 2, 0, r + 1);
        else
            UI_PrintStringSmallNormal(s, 2, 0, r + 1);
    }
    ST7565_BlitFullScreen();
}

/* ---- APRS symbol -> 16x16 icon --------------------------------------------
 * Column-major: bytes 0..15 are the top 8-px LCD page (bit0 = top pixel),
 * bytes 16..31 the bottom page. An icon spans two LCD rows. Kept to a small
 * hand-picked set (aprs.fi-style); anything unmapped falls back to a box.
 * Generated by tools/gen_icons.py on the V1 port (edit the pixel grids
 * there, re-run) -- same table reused verbatim here, it's pure pixel data. */
static const uint8_t ICON_BMP[14][32] = {
    { 0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xB0, 0xD0, 0xD0, 0xD0, 0xD0, 0xB0, 0xE0, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x03, 0x17, 0x3F, 0x3F, 0x17, 0x07, 0x07, 0x07, 0x07, 0x17, 0x3F, 0x3F, 0x17, 0x03, 0x00 },  /*  0 car */
    { 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x7F, 0x7F, 0x43, 0x7F, 0x67, 0x67, 0x67, 0x67, 0x7F, 0x43, 0x7F, 0x7F, 0x7E, 0x00 },  /*  1 house */
    { 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xD8, 0xFC, 0xE4, 0xE4, 0xFC, 0xD8, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x03, 0x03, 0x21, 0x30, 0x3B, 0x0F, 0x07, 0x07, 0x0F, 0x3B, 0x33, 0x23, 0x00, 0x01, 0x03, 0x03 },  /*  2 person */
    { 0x80, 0x40, 0x40, 0x40, 0x80, 0x00, 0xA0, 0x90, 0x90, 0x20, 0x80, 0x40, 0x40, 0x40, 0x80, 0x00, 0x03, 0x04, 0x04, 0x04, 0x03, 0x01, 0x0E, 0x02, 0x0A, 0x05, 0x03, 0x04, 0x04, 0x04, 0x03, 0x00 },  /*  3 bicycle */
    { 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0xC0, 0xE0, 0xE0, 0x20, 0x70, 0xF0, 0xF0, 0x80, 0x80, 0x00, 0x00, 0x07, 0x0F, 0x0D, 0x0F, 0x07, 0x00, 0x0E, 0x0E, 0x0E, 0x0E, 0x00, 0x07, 0x07, 0x01, 0x00 },  /*  4 motorcycle */
    { 0x00, 0x00, 0xF8, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0xF8, 0xA0, 0xA0, 0xA0, 0x60, 0x00, 0x00, 0x0B, 0x1E, 0x1E, 0x0A, 0x02, 0x02, 0x02, 0x0A, 0x1E, 0x1F, 0x0A, 0x02, 0x03, 0x00 },  /*  5 truck */
    { 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0xB0, 0x90, 0x90, 0xB0, 0xC0, 0x80, 0x80, 0x80, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0F, 0x07, 0x03, 0x01 },  /*  6 boat */
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x08, 0x1C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x0E, 0x08 },  /*  7 sailboat */
    { 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF8, 0xFC, 0xFC, 0xF8, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x01, 0x01, 0x01, 0x11, 0x19, 0x1D, 0x0F, 0x07, 0x07, 0x0F, 0x1D, 0x19, 0x11, 0x01, 0x01, 0x00 },  /*  8 aircraft */
    { 0x80, 0x00, 0x40, 0x04, 0xE8, 0xF0, 0xF8, 0xFE, 0xF8, 0xF0, 0xE8, 0x04, 0x40, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0x10, 0x0B, 0x07, 0x0F, 0x3F, 0x0F, 0x07, 0x0B, 0x10, 0x01, 0x00, 0x00, 0x00 },  /*  9 wx */
    { 0x00, 0x00, 0x80, 0x90, 0xA0, 0xC0, 0x80, 0xFC, 0x80, 0xC0, 0xA0, 0x90, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x01, 0x00, 0x1F, 0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00 },  /* 10 digipeater */
    { 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x78, 0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00 },  /* 11 igate */
    { 0x00, 0x00, 0xFC, 0x04, 0x04, 0x24, 0x34, 0x14, 0x94, 0xF4, 0x64, 0x04, 0x04, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x20, 0x20, 0x20, 0x20, 0x2B, 0x2B, 0x20, 0x20, 0x20, 0x20, 0x3F, 0x00, 0x00 },  /* 12 box (default) */
    { 0x20, 0x30, 0x18, 0x0C, 0x70, 0x80, 0xC0, 0xFE, 0xFE, 0xC0, 0x80, 0x70, 0x0C, 0x18, 0x30, 0x20, 0x80, 0xC0, 0xE0, 0x70, 0x5C, 0x07, 0x00, 0x00, 0x00, 0x00, 0x07, 0x5C, 0x70, 0xE0, 0xC0, 0x80 },  /* 13 repeater tower (/r) */
};

static uint8_t icon_index(char t, char c)
{
    if (t != '/') {                       /* alt table or overlay */
        if (c == '#') return 10;
        if (c == '&') return 11;
    }
    switch (c) {
        case '>':                       return 0;
        case '-':                       return 1;
        case '[':                       return 2;
        case 'b':                       return 3;
        case '<':                       return 4;
        case 'k': case 'u': case 'v':   return 5;
        case 's': case 'C':             return 6;
        case 'Y':                       return 7;
        case '\'': case '^': case 'X': case 'g': return 8;
        case '_':                       return 9;
        case '#':                       return 10;
        case '&':                       return 11;
        case 'r': case 'y':             return 13;   /* repeater tower / yagi */
    }
    return 12;
}

/* blit a 16x16 icon at (col, page rows [row, row+1]) */
static void draw_icon(uint8_t idx, uint8_t col, uint8_t row)
{
    if (idx > 13 || row > 5 || col > 112)
        return;
    memcpy(gFrameBuffer[row]     + col, ICON_BMP[idx],      16);
    memcpy(gFrameBuffer[row + 1] + col, ICON_BMP[idx] + 16, 16);
}

static bool APRS_HasRx(void) { return s_rxi.valid || s_rxn > 0; }

/* Received-packet view. The RP2040 C-Board demodulates 144.8 MHz APRS and
 * pushes either a structured decode (0x06D3 -> s_rxi) or, when it cannot parse
 * the info field, raw wrapped text (0x06D2 -> s_rx[]).
 * Layout (header shown only outside the auto-popup): row 0 header; then a
 * 16x16 symbol icon + callsign, Direct/Via path, position / range+bearing /
 * course+speed / wrapped comment. */
static void draw_rx(bool header)
{
    char s[28];
    const uint8_t top = header ? 1 : 0;   /* popup: no "APRS RX" line, shift up */
    UI_DisplayClear();
    if (header) {
        sprintf(s, "APRS RX %u  *:cfg", s_rx_pkts > 999 ? 999 : s_rx_pkts); /* <=18 */
        UI_PrintStringSmallBold(s, 2, 0, 0);
    }

    if (s_rxi.valid) {
        const bool obj = (s_rxi.kind == APRS_RX_KIND_OBJECT);
        const bool msg = (s_rxi.kind == APRS_RX_KIND_MESSAGE);

        draw_icon(icon_index(s_rxi.sym_t, s_rxi.sym_c), 2, top);

        /* right column (col 22): line 1 = name/callsign, line 2 = msg addressee */
        const char *lbl = (obj && s_rxi.name[0]) ? s_rxi.name : s_rxi.src;
        strncpy(s, lbl, 14); s[14] = 0;
        UI_PrintStringSmallNormal(s, 22, 0, top);
        if (msg && s_rxi.name[0]) {
            sprintf(s, ">%.12s", s_rxi.name); s[15] = 0;
            UI_PrintStringSmallNormal(s, 22, 0, top + 1);
        }

        uint8_t row = top + 2;

        /* digi path: "Direct" or "Via A,B" -- wrapped onto up to 2 rows,
         * breaking after a comma so a call is never split. Never overflows. */
        if (row <= 6) {
            char path[36];
            uint8_t n = 0;
            if (s_rxi.via[0]) {
                memcpy(path, "Via ", 4); n = 4;
                for (const char *q = s_rxi.via;
                     *q && n < (uint8_t)(sizeof path - 1); q++)
                    path[n++] = *q;
            } else {
                memcpy(path, "Direct", 6); n = 6;
            }
            path[n] = 0;
            const char *q = path;
            for (int li = 0; li < 2 && *q && row <= 6; li++) {
                char line[APRS_RX_CHARS + 1];
                int  cut = 0, lastc = -1;
                while (q[cut] && cut < APRS_RX_CHARS) {
                    if (q[cut] == ',') lastc = cut;
                    cut++;
                }
                if (q[cut] && lastc >= 0) cut = lastc + 1;   /* break after ',' */
                memcpy(line, q, (size_t)cut);
                line[cut] = 0;
                UI_PrintStringSmallNormal(line, 2, 0, row++);
                q += cut;
            }
        }

        if ((s_rxi.flags & 1) && row <= 6) {         /* position */
            int32_t la = s_rxi.lat, lo = s_rxi.lon;
            char ns = la < 0 ? 'S' : 'N', ew = lo < 0 ? 'W' : 'E';
            if (la < 0) la = -la;
            if (lo < 0) lo = -lo;
            sprintf(s, "%ld.%04ld%c %ld.%04ld%c",
                    (long)(la / 100000), (long)((la % 100000) / 10), ns,
                    (long)(lo / 100000), (long)((lo % 100000) / 10), ew);
            UI_PrintStringSmallNormal(s, 2, 0, row++);
        }
        if ((s_rxi.flags & 8) && row <= 6) {         /* range / bearing to me */
            uint32_t dm = (uint32_t)s_rxi.dist_hm * 100;
            if (dm < 10000)
                sprintf(s, "%lum  %u deg", (unsigned long)dm, s_rxi.bearing);
            else
                sprintf(s, "%lu.%lukm  %u deg", (unsigned long)(dm / 1000),
                        (unsigned long)((dm % 1000) / 100), s_rxi.bearing);
            UI_PrintStringSmallNormal(s, 2, 0, row++);
        }
        if (((s_rxi.flags & 2) || (s_rxi.flags & 4)) && row <= 6) {
            char *w = s;
            if (s_rxi.flags & 2)
                w += sprintf(w, "%ukmh %udeg ", s_rxi.speed, s_rxi.course);
            if (s_rxi.flags & 4)
                sprintf(w, "%dm", s_rxi.alt);
            s[APRS_RX_CHARS] = 0;
            UI_PrintStringSmallNormal(s, 2, 0, row++);
        }
        const char *p = s_rxi.text;                  /* wrapped comment / text */
        while (*p && row <= 6) {
            char line[APRS_RX_CHARS + 1];
            uint8_t k = 0;
            while (*p && k < APRS_RX_CHARS)
                line[k++] = *p++;
            line[k] = 0;
            UI_PrintStringSmallNormal(line, 2, 0, row++);
        }
    } else if (s_rxn == 0) {
        UI_PrintStringSmallNormal("waiting a packet", 2, 0, top + 1);
        UI_PrintStringSmallNormal("radio 144.800 FM", 2, 0, top + 3);
    } else {
        for (uint8_t i = 0; i < s_rxn && i < 4; i++)
            UI_PrintStringSmallNormal(s_rx[i], 2, 0, top + i);
    }
    ST7565_BlitFullScreen();
}

/* Sleep one popup-loop tick. Also flushes any pending backlight PWM fade to
 * completion first -- same rationale as FOXHUNT_TickDelay() in
 * app/foxhunt.c: BACKLIGHT_TurnOn() only *arms* a fade (it sets the target
 * brightness and a step size), the fade itself is only advanced by
 * BACKLIGHT_Update(), which the normal 10 ms tick calls from app.c -- a tick
 * this blocking popup loop does not run, so without pumping it here every
 * BACKLIGHT_TurnOn() in this file (decoded frame, keypress, entry) would
 * never reach the hardware PWM until the loop exits ("backlight only lights
 * up as the popup closes, even on a keypress", reported on air).
 * BACKLIGHT_Update() is pumped 16 times back-to-back rather than once per
 * 10 ms slice: the fade always completes in <= 16 steps regardless of the
 * jump size (fadeStep is diff/16, see BACKLIGHT_SetBrightness()), so 16
 * calls always finish it -- spreading it across the ~160 ms of a slower loop
 * made the ramp plainly visible on screen ("effet de fondu" reported once
 * the backlight fix landed); flushing it up front makes each brightness
 * change look as instant here as it does on the normal (non-blocking) UI,
 * where the same ~160 ms ramp is masked by the screen changing under it. */
static void APRS_TickDelay(uint32_t ms)
{
    for (int i = 0; i < 16; i++)
        BACKLIGHT_Update();
    SYSTEM_DelayMs(ms);
}

void APP_RunAprs(void)
{
    APRS_Ensure();
    AFGAIN_ResyncKnob();
    AFGAIN_Apply();                        /* level right regardless of prior menu use */

    const bool popup = gAprsShowRequest;   /* opened by the tick, not by a key */
    gAprsShowRequest = false;

    /* This blocking screen can be opened from the 10 ms tick (auto-popup on an
     * RX packet), not only from a key -- only take the radio over when it is
     * idle on the main screen, else re-arm and bail (same rule as SARSAT). */
    if (gScreenToDisplay != DISPLAY_MAIN || gCurrentFunction == FUNCTION_TRANSMIT ||
        gScanStateDir != SCAN_OFF) {
        if (popup && APRS_HasRx()) gAprsShowRequest = true;
        return;
    }

    /* RX stays live under the popup so the C-Board keeps decoding (a packet
     * arriving here is shown too and re-arms the auto-close timer). But the
     * blocked main loop can't close the speaker path / RX LED when the carrier
     * drops between packets -- so pump the BK4819 squelch interrupt in the loop
     * and follow it (see the SQUELCH block below). */
    if (!g_SquelchLost) {                 /* channel already idle -> silence it now */
        AUDIO_AudioPathOff();
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    }

    /* auto-popup (from the tick, on a decoded packet) opens on the RX view;
     * a manual key always opens on the config menu (press * for the last RX). */
    int sel = 0, editing = 0, callcur = 0, first = 0, view = popup ? 1 : 0;
    bool armed = false, run = true, closed_by_key = false;
    KEY_Code_t prev = KEY_INVALID;
    bool dirty = true;
    uint16_t last_pkts = s_rx_pkts;

    /* auto-popups close themselves after gAprsCfg.popup_s (unless a key is hit) */
    const bool timed = (popup && gAprsCfg.popup_s);
    uint32_t close_at = timed ? millis10() + gAprsCfg.popup_s * 100u : 0;

    BACKLIGHT_TurnOn();

    while (run) {
#ifdef ENABLE_UART
        while (UART_IsCommandAvailable(UART_PORT_UART))
            UART_HandleCommand(UART_PORT_UART);
#endif
        /* mini squelch: the main loop (which normally does this) is blocked.
         * carrier present  -> speaker path + RX LED on  (C-Board hears it)
         * carrier gone      -> both off, so the channel noise is not heard */
        while (BK4819_ReadRegister(BK4819_REG_0C) & 1u) {
            BK4819_WriteRegister(BK4819_REG_02, 0);
            uint16_t ib = BK4819_ReadRegister(BK4819_REG_02);
            if (ib & BK4819_REG_02_SQUELCH_LOST) {
                g_SquelchLost = true;
                AUDIO_AudioPathOn();
                BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true);
            }
            if (ib & BK4819_REG_02_SQUELCH_FOUND) {
                g_SquelchLost = false;
                AUDIO_AudioPathOff();
                BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
            }
        }

        /* This loop is what receives 0x06D6 (UART_HandleCommand() above, when
         * the RX popup is what's open) and queues it via APRS_Digipeat() --
         * but the retry that actually sends it, APRS_DigipeatTimeSlice(), is
         * normally pumped from APRS_TimeSlice(), which this same blocking
         * loop prevents from running at all. Without this, a digipeat
         * opportunity arriving while the popup is up (the common case: the
         * popup auto-opens on the very packet being repeated) would sit
         * queued until popup_s expires or the 3 s staleness timeout kills
         * it, whichever comes first -- effectively never sent. */
        APRS_DigipeatTimeSlice();

        /* "Light on frame": reported on air as "the backlight stays dark
         * through the whole popup, then flashes on right as it closes" --
         * i.e. only the unconditional BACKLIGHT_TurnOn() in this function's
         * own exit path (below) was ever visible; the one-shot call from
         * aprs_rx_arrived() (fired once, the instant the frame decodes)
         * wasn't sticking, for a reason not pinned down from the source
         * alone. Reasserting it every loop iteration (~20 ms) for as long as
         * the popup/screen stays open is a stronger guarantee regardless of
         * cause -- same fix shape as AFGAIN_TimeSlice() earlier in this
         * project (a one-shot apply that silently got overridden). */
        if (gAprsCfg.opts & APRS_OPT_BL_DECODE)
            BACKLIGHT_TurnOn();

        /* a new packet: while the auto-close timer is running, re-arm it and
         * pull the RX view back up (a keypress has cleared close_at, so a
         * user who navigated away is left alone) */
        if (s_rx_pkts != last_pkts) {
            last_pkts = s_rx_pkts;
            if (close_at) {
                close_at = millis10() + gAprsCfg.popup_s * 100u;
                if (!editing) view = 1;
            }
        }
        if (close_at && (int32_t)(millis10() - close_at) >= 0)
            break;

        if (sel < first)                    first = sel;
        if (sel > first + APRS_VIS_ROWS - 1) first = sel - (APRS_VIS_ROWS - 1);
        if (view == 1 && s_rx_dirty) { s_rx_dirty = false; dirty = true; }
        if (dirty) {
            dirty = false;
            if (view == 1) draw_rx(!popup);   /* popup: hide the "APRS RX" line */
            else           draw_config(sel, editing, callcur, first);
        }

        KEY_Code_t k = KEYBOARD_Poll();
        if (k == KEY_INVALID) { armed = true; prev = k; APRS_TickDelay(20); continue; }
        if (!armed || k == prev) { APRS_TickDelay(20); continue; }
        prev = k;
        dirty = true;
        close_at = 0;                         /* a key cancels the auto-close */
        k = nav_key(k);                       /* UV-K1 SetNav (see nav_key()) */

        /* This loop blocks the main task, whose backlight and auto-keypad-lock
         * countdowns then never tick -- so nothing turns them back on. Refresh
         * both on every key so the screen stays lit and the pad stays live. */
        BACKLIGHT_TurnOn();
        gKeyLockCountdown = 30;

        if (view == 1) {                      /* RX view */
            if      (k == KEY_STAR) view = 0;         /* -> config */
            else if (k == KEY_EXIT) { run = false; closed_by_key = true; }  /* -> close */
            continue;
        }

        if (!editing) {
            switch (k) {
            case KEY_UP:   sel = (sel + F_N - 1) % F_N; break;
            case KEY_DOWN: sel = (sel + 1) % F_N;       break;
            case KEY_MENU:
                editing = 1; callcur = 0; s_klen = 0;
                s_kneg = (sel == F_LAT) ? (gAprsCfg.lat_e5 < 0) :
                         (sel == F_LON) ? (gAprsCfg.lon_e5 < 0) : 0;
                break;
            case KEY_STAR: view = 1;                    break;   /* -> RX view */
            case KEY_5:    APRS_Beacon();               break;
            case KEY_EXIT: run = false; closed_by_key = true; break;
            default: break;
            }
        } else if (sel == F_CALL || sel == F_TEXT) {
            const bool txt = (sel == F_TEXT);
            char *buf = txt ? gAprsCfg.comment : gAprsCfg.call;
            const int  fw = txt ? 14 : 6;
            switch (k) {
            case KEY_UP:   buf[callcur] = cyc_char(buf[callcur], +1,
                                                  txt ? TEXTSET : CALLSET); break;
            case KEY_DOWN: buf[callcur] = cyc_char(buf[callcur], -1,
                                                  txt ? TEXTSET : CALLSET); break;
            case KEY_STAR: callcur = (callcur + 1) % fw; break;
            case KEY_MENU: editing = 0; APRS_Save(); APRS_PushConfig(); break;
            case KEY_EXIT: editing = 0; APRS_Init();    break;   /* reload */
            default: break;
            }
        } else if (sel == F_LAT || sel == F_LON) {
            if (k <= KEY_9) {                       /* a digit */
                int cap = (sel == F_LAT ? 2 : 3) + 5;
                if (s_klen < cap) s_kbuf[s_klen++] = (char)('0' + (int)k);
            } else switch (k) {
            case KEY_STAR: s_kneg ^= 1;                          break;
            case KEY_UP:   if (!s_klen) field_step(sel, +1);     break;
            case KEY_DOWN: if (!s_klen) field_step(sel, -1);     break;
            case KEY_MENU: kbuf_commit(sel); editing = 0;
                           APRS_Save(); APRS_PushConfig();       break;
            case KEY_EXIT: editing = 0; APRS_Init();             break;
            default: break;
            }
        } else {
            switch (k) {
            case KEY_UP:   field_step(sel, +1); break;
            case KEY_DOWN: field_step(sel, -1); break;
            case KEY_MENU: editing = 0; APRS_Save(); APRS_PushConfig(); break;
            case KEY_EXIT: editing = 0; APRS_Init(); break;
            default: break;
            }
        }
        APRS_TickDelay(20);
    }

    /* restore normal operation. This screen can be auto-opened from the 10 ms
     * tick, so finish the cleanup synchronously (same as APP_RunSarsat): leave
     * any monitor function, re-read + validate both VFOs from EEPROM, re-tune,
     * refresh the frozen backlight / key-lock timers, and switch back to MAIN
     * now rather than "on request". */
    gAprsShowRequest      = false;
    /* cooldown before the next auto-popup: 5 s if the user pressed EXIT (they
     * dismissed it), ~0.3 s on a plain timed close so a busy channel's next
     * frame still pops. */
    s_autopop_block_10ms  = millis10() + (closed_by_key ? 500u : 30u);
    gMonitor = false;
    FUNCTION_Select(FUNCTION_FOREGROUND);
    RADIO_ConfigureChannel(0, VFO_CONFIGURE);
    RADIO_ConfigureChannel(1, VFO_CONFIGURE);
    RADIO_SelectVfos();
    RADIO_SetupRegisters(true);        /* AUDIO_AudioPathOff + retune: squelch
                                        * back to normal after the popup */
    BACKLIGHT_TurnOn();
    gKeyLockCountdown = 30;
    /* "light on frame" + this was an auto-popup: dim ~1 s after it closed */
    s_bl_off_10ms = (popup && (gAprsCfg.opts & APRS_OPT_BL_DECODE))
                        ? millis10() + 100 : 0;

    GUI_SelectNextDisplay(DISPLAY_MAIN);
    gUpdateDisplay = true;
    gUpdateStatus  = true;
}

#endif /* ENABLE_APRS */
