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
static bool     s_adrasec;              /* a "REPORT ADRASEC" position request
                                        * arrived -- show a sticky coordinate
                                        * screen, EXIT only, no timeout/popup */
static struct { int32_t lat, lon; uint16_t dist_hm, brg; uint8_t hasd;
                char src[10]; } s_adr;  /* its own copy: a later ordinary frame
                                        * overwrites s_rxi but not this screen  */

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

/* SmartBeaconing: two fixed presets, selected through the F_INT menu row by
 * overloading gAprsCfg.interval_s with sentinel values below the smallest real
 * period (30 s). Needs GPS position mode + a live fix (speed / course). */
#define APRS_SB_CAR  1u                  /* interval_s == 1 : car profile    */
#define APRS_SB_FOOT 2u                  /* interval_s == 2 : on-foot profile */
#define APRS_SB_IS(v) ((v) == APRS_SB_CAR || (v) == APRS_SB_FOOT)

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

/* This firmware is a fixed SARSAT / APRS appliance, so it seeds a few
 * dedicated MR channels with sane defaults:
 *
 *   ch 170  "APRS"    144.800 MHz  FM   wide  HIGH   -- the fixed APRS TX slot
 *   ch 1    "SAREX"   434.200 MHz  RAW  wide  LOW    -- SAR exercise beacon
 *   ch 2    "SARSAT"  406.028 MHz  RAW  wide  LOW    -- real 406 MHz (RX only!)
 *
 * A slot is written ONLY while still erased (0xFFFFFFFF frequency): a channel
 * already storing something -- a user edit, or a channel the operator uses for
 * something else -- is never touched (clear it first if you want the default
 * back). The APRS slot's name is additionally re-pinned to "APRS" on every
 * boot, the way KD8CEC's own C-Board project keeps its TX slot dedicated. */
static void APRS_SeedChannel(uint8_t ch, uint32_t freq,
                             uint8_t modulation, uint8_t power, const char *name)
{
    uint32_t cur;
    EEPROM_ReadBuffer((uint16_t)(ch * 16u), &cur, sizeof cur);
    if (cur != 0xFFFFFFFFu)
        return;                       /* slot in use -- leave it alone */
    VFO_Info_t v;
    RADIO_InitInfo(&v, ch, freq);     /* wide + FM + std step by default */
    v.Modulation   = modulation;
    v.OUTPUT_POWER = power;
    SETTINGS_SaveChannel(ch, 0, &v, 2);
    SETTINGS_SaveChannelName(ch, name);
}

static void APRS_EnsureChannel(void)
{
#ifdef ENABLE_BYP_RAW_DEMODULATORS
    const uint8_t discri = MODULATION_RAW;      /* flat FM discriminator ("RAW") */
#else
    const uint8_t discri = MODULATION_FM;
#endif
    APRS_SeedChannel(APRS_TX_CHANNEL, APRS_DEFAULT_FREQ,
                     MODULATION_FM, OUTPUT_POWER_HIGH, "APRS");
    APRS_SeedChannel(0, 43420000u, discri, OUTPUT_POWER_LOW1, "SAREX");
    APRS_SeedChannel(1, 40602800u, discri, OUTPUT_POWER_LOW1, "SARSAT");

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
    {   /* msg_to is a newer field: an EEPROM written by an older build has
         * junk in those bytes -- accept only a plausible addressee */
        int ok = 1;
        for (int i = 0; i < (int)sizeof gAprsCfg.msg_to; i++) {
            char c = gAprsCfg.msg_to[i];
            if (c == 0) break;
            if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                  c == '-' || c == ' ')) { ok = 0; break; }
        }
        if (!ok) memset(gAprsCfg.msg_to, 0, sizeof gAprsCfg.msg_to);
        gAprsCfg.msg_to[sizeof gAprsCfg.msg_to - 1] = 0;
    }
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
    uint8_t b[4 + 12];
    b[0] = APRS_CMD_CONFIG & 0xFF; b[1] = APRS_CMD_CONFIG >> 8;
    b[2] = 12;                     b[3] = 0;
    memcpy(b + 4, gAprsCfg.call, 6);
    b[10] = gAprsCfg.ssid;
    b[11] = gAprsCfg.path;
    b[12] = gAprsCfg.sym_table;
    b[13] = gAprsCfg.sym_code;
    b[14] = (uint8_t)((gAprsCfg.opts & APRS_OPT_DIGI_MASK) >> APRS_OPT_DIGI_SHIFT);
    b[15] = (gAprsCfg.opts & APRS_OPT_KISS) ? 0x01 : 0x00;   /* flags: KISS */
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
    RADIO_SetupRegisters(true);          /* back to RX, exactly as before --
                                          * this firmware's version also
                                          * re-runs RADIO_SetModulation(), so
                                          * the AF is un-muted from the AFSK TX */

    /* RADIO_SetupRegisters() put REG_4E back to the channel squelch and reset
     * REG_48. On the APRS band that makes the squelch chop the next packet
     * and the C-Board level wrong -- the 10 ms tick normally re-applies both
     * (APRS_TimeSlice()), but it is blocked whenever this TX fired from
     * APP_RunAprs()'s own loop (report retry / digipeat). Re-apply here so
     * every TX path leaves a clean APRS RX behind. No-ops off-band / auto. */
    RADIO_SetModulation(gRxVfo->Modulation);
    APRS_ApplySquelch();
    AFGAIN_Apply();
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

/* Assemble "MYCALL-SSID>APZSAR,<digi...>:<info>" and hand it to APRS_TxFrame().
 * Shared by APRS_Beacon() and APRS_MsgTx(). */
static void APRS_TxInfo(const char *info, int ilen,
                        const ax25_addr_t *digi, int nd)
{
    if (ilen <= 0 || ilen > AX25_MAX_INFO)
        return;

    ax25_addr_t src, dst;
    memset(&src, 0, sizeof src); memset(&dst, 0, sizeof dst);
    memcpy(src.call, gAprsCfg.call, 6); src.ssid = gAprsCfg.ssid;
    strcpy(dst.call, "APZSAR");

    uint8_t frame[AX25_MAX_FRAME];
    int flen = ax25_build_ui(frame, &dst, &src, digi, nd, info, ilen);
    if (flen)
        APRS_TxFrame(frame, flen);
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

    ax25_addr_t digi[AX25_MAX_DIGI];
    memset(digi, 0, sizeof digi);
    int nd = APRS_Digi(digi);
    APRS_TxInfo(info, ilen, digi, nd);
}

/* ---- "121 MHz beacon report" APRS message ("Send report" on the screen) --
 * A canned APRS text message to a fixed recipient (gAprsCfg.msg_to) over a
 * fixed WIDE1-1,WIDE2-2 path, for reporting a 121.5 MHz homing bearing:
 *   "Report Balise 121 MHz S: <0-9> Dir: <0-359|KO>"
 * Sent with an APRS message number so the addressee's client auto-acks; we
 * retry up to APRS_MSG_TRIES times, APRS_MSG_RETRY_10MS apart, until that
 * ack comes back (seen through the RP2040's decoded-message push, 0x06D3)
 * or we give up. Progress shows on the F_SEND menu row. Needs the C-Board
 * connected (it is what hears the ack); with no TNC it always ends "no ack". */
#define APRS_MSG_TRIES        3
#define APRS_MSG_RETRY_10MS   3000     /* 30 s between the (re)transmits     */
#define APRS_MSG_WAIT_10MS    30000    /* keep listening for the ack 5 min   */
#define APRS_MSG_DIR_KO       0xFFFFu

enum { APRS_MSG_IDLE = 0, APRS_MSG_WAIT, APRS_MSG_ACK, APRS_MSG_FAIL };

static struct {
    uint8_t  state;               /* APRS_MSG_*                             */
    uint8_t  seq;                 /* APRS message number, cycles 1..99      */
    uint8_t  tries;               /* (re)transmits done so far (max _TRIES) */
    uint8_t  sig;                 /* signal 0..9                            */
    uint16_t dir;                 /* 0..359, or APRS_MSG_DIR_KO             */
    uint32_t next_at_10ms;        /* next (re)transmit due                  */
    uint32_t deadline_10ms;       /* declare "no ack" past this             */
} s_msg;


/* decimal degrees with 4 decimals, signed ("43.9544", "-1.3644") */
static int APRS_Deg4(char *out, int32_t e5)
{
    uint32_t a   = (e5 < 0) ? (uint32_t)(-e5) : (uint32_t)e5;
    uint32_t deg = a / 100000u;
    uint32_t f4  = (a % 100000u + 5u) / 10u;       /* round to 4 dp */
    if (f4 >= 10000u) { deg++; f4 -= 10000u; }
    return sprintf(out, "%s%u.%04u", (e5 < 0) ? "-" : "",
                   (unsigned)deg, (unsigned)f4);
}

static int APRS_MsgInfo(char *info)
{
    char to[10];
    int n = 0;
    for (; n < 9 && gAprsCfg.msg_to[n]; n++) to[n] = gAprsCfg.msg_to[n];
    for (; n < 9; n++) to[n] = ' ';         /* addressee is exactly 9 chars */
    to[9] = 0;

    char dir[8];
    if (s_msg.dir == APRS_MSG_DIR_KO) strcpy(dir, "KO");
    else                             sprintf(dir, "%u", s_msg.dir);

    /* my resolved position (GPS fix if in GPS mode, else the manual lat/lon);
     * appended in decimal degrees only when it is actually set */
    int32_t lat, lon;
    APRS_MyPosition(&lat, &lon);
    char pos[26];
    pos[0] = 0;
    if (lat || lon) {
        int p = sprintf(pos, " ");
        p += APRS_Deg4(pos + p, lat);
        pos[p++] = ' ';
        APRS_Deg4(pos + p, lon);
    }

    return sprintf(info, ":%s:Report Balise 121 MHz S: %u Dir: %s%s{%02u",
                   to, s_msg.sig, dir, pos, s_msg.seq);
}

static void APRS_MsgTx(void)
{
    char info[AX25_MAX_INFO + 8];
    int  ilen = APRS_MsgInfo(info);

    ax25_addr_t digi[2];
    memset(digi, 0, sizeof digi);
    strcpy(digi[0].call, "WIDE1"); digi[0].ssid = 1;   /* fixed WIDE1-1,WIDE2-2 */
    strcpy(digi[1].call, "WIDE2"); digi[1].ssid = 2;
    APRS_TxInfo(info, ilen, digi, 2);
}

/* begin a report; sig 0 forces "Dir: KO" and the caller skips the direction */
static void APRS_MsgStart(uint8_t sig, uint16_t dir)
{
    if (gAprsCfg.call[0] == 0 || strcmp(gAprsCfg.call, "NOCALL") == 0)
        return;
    if (gAprsCfg.msg_to[0] <= ' ')            /* no recipient set */
        return;
    s_msg.sig   = (sig > 9) ? 9 : sig;
    s_msg.dir   = (sig == 0) ? APRS_MSG_DIR_KO : (dir > 359 ? 359 : dir);
    s_msg.seq   = (uint8_t)(s_msg.seq % 99u) + 1u;
    s_msg.tries = 0;
    s_msg.state = APRS_MSG_WAIT;
    s_msg.next_at_10ms  = millis10();                       /* first TX next slice */
    s_msg.deadline_10ms = millis10() + APRS_MSG_WAIT_10MS;
}

/* pumped from APRS_TimeSlice() and the APP_RunAprs() loop, like digipeat.
 * The 3 (re)transmits go out over the first ~90 s, but we keep the state on
 * WAIT (still matching an incoming ack) until APRS_MSG_WAIT_10MS -- a real
 * APRS ack, digipeated both ways or bounced off an i-gate, routinely takes
 * longer than the retransmit window. */
static void APRS_MsgTimeSlice(void)
{
    if (s_msg.state != APRS_MSG_WAIT)
        return;
    if ((int32_t)(millis10() - s_msg.deadline_10ms) >= 0) {
        s_msg.state = APRS_MSG_FAIL;                        /* no ack, gave up */
        BACKLIGHT_TurnOn();
        return;
    }
    if (s_msg.tries >= APRS_MSG_TRIES)
        return;                                             /* done TXing, listen */
    if ((int32_t)(millis10() - s_msg.next_at_10ms) < 0)
        return;
    if (!APRS_CanTransmitNow())
        return;                                             /* channel busy */
    APRS_MsgTx();
    s_msg.tries++;
    s_msg.next_at_10ms = millis10() + APRS_MSG_RETRY_10MS;
}

/* an APRS message decoded by the RP2040 (0x06D3): is it our "ackNN"?
 * Accepts a late ack that lands after we already gave up (FAIL -> ACK). */
static void APRS_MsgCheckAck(const char *addressee, const char *text)
{
    if ((s_msg.state != APRS_MSG_WAIT && s_msg.state != APRS_MSG_FAIL) ||
        strncmp(text, "ack", 3) != 0)
        return;

    /* base callsign of the addressee (before '-' / space) must be ours; an
     * SSID that differs or is absent is tolerated (some clients ack the bare
     * call). Trim both sides -- gAprsCfg.call can carry a trailing space from
     * the char-cycle editor. */
    char ab[8], mc[8];
    int i = 0, j = 0;
    for (; i < 6 && addressee[i] > ' ' && addressee[i] != '-'; i++)
        ab[i] = addressee[i];
    ab[i] = 0;
    for (; j < 6 && gAprsCfg.call[j] > ' ' && gAprsCfg.call[j] != '-'; j++)
        mc[j] = gAprsCfg.call[j];
    mc[j] = 0;
    if (ab[0] == 0 || strcmp(ab, mc) != 0)
        return;

    /* "ackNN": accept NN == the current report, OR any earlier number of this
     * session -- retested reports carry the same S/Dir text, so a slow client
     * that keeps re-acking "ack01" (or dedups our retries) still confirms the
     * report was received. Only a number past our current seq is rejected. */
    int no = 0;
    for (const char *q = text + 3; *q >= '0' && *q <= '9'; q++)
        no = no * 10 + (*q - '0');
    if (no >= 1 && no <= s_msg.seq) {
        s_msg.state = APRS_MSG_ACK;
        BACKLIGHT_TurnOn();
    }
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

/* SmartBeaconing profiles -- {low, high, slow, fast, turn_min, turn_slope,
 * turn_time}, speeds km/h, angles deg, times s.  Row 0 = car, 1 = on foot.
 * `slow` doubles as the fallback fixed period when GPS is unavailable.        */
static const uint16_t s_sb_prof[2][7] = {
    {    5,   90, 1200,   30,    25,     255,     25 },   /* car    */
    {    2,    8,  600,   90,    35,      80,     45 },   /* on foot */
};
#define APRS_SB_ROW  (gAprsCfg.interval_s == APRS_SB_FOOT)

/* SmartBeaconing (HamHUD-style): variable beacon rate from GPS speed, plus
 * "corner pegging" -- an extra beacon when the course changes by more than a
 * speed-dependent threshold. Returns true when a beacon is due now; latches
 * its own timing.  Only called while a live GPS fix is available.            */
static bool APRS_SmartBeaconDue(void)
{
    static uint32_t last_10ms;
    static uint16_t last_course;

    const uint16_t *p = s_sb_prof[APRS_SB_ROW];
    uint16_t v   = s_gps.speed_kmh;
    uint16_t sec = (uint16_t)((millis10() - last_10ms) / 100u);

    uint16_t rate = (v <= p[0]) ? p[2]
                  : (v >= p[1]) ? p[3]
                  : (uint16_t)((uint32_t)p[3] * p[1] / v);

    bool due = sec >= rate;

    if (!due && v > p[0] && sec >= p[6]) {           /* corner pegging */
        int16_t dc = (int16_t)((int16_t)s_gps.course - (int16_t)last_course);
        if (dc < -180) dc += 360; else if (dc > 180) dc -= 360;
        if (dc < 0) dc = (int16_t)-dc;
        if (dc >= (int16_t)(p[4] + p[5] / v))
            due = true;
    }

    if (due) { last_10ms = millis10(); last_course = s_gps.course; }
    return due;
}

void APRS_TimeSlice(void)
{
    APRS_Ensure();

    /* KISS TNC: the RP2040 (on its USB) is the TNC. The radio only keeps the
     * fast-squelch on, the C-Board AF level applied, and relays the frames the
     * host asks to send (APRS_DigipeatTimeSlice, fed by CMD_APRS_DIGI) -- no
     * own beacon / "121 MHz report" / RX popup. */
    const bool kiss = (gAprsCfg.opts & APRS_OPT_KISS) != 0;

    APRS_ApplySquelch();               /* APRS-band fast-squelch (cheap: 1 reg read) */
    AFGAIN_TimeSlice();                /* keep a fixed C-Board AF gain in effect
                                        * everywhere, screen open or not     */
    APRS_DigipeatTimeSlice();          /* retry a queued frame until the channel
                                        * is clear -- also the KISS TX path   */
    if (kiss)
        return;
    APRS_MsgTimeSlice();               /* retry the "121 MHz report" message
                                        * until its ack comes back            */

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

    const bool sb      = APRS_SB_IS(gAprsCfg.interval_s);
    const bool sb_live = sb && (gAprsCfg.opts & APRS_OPT_GPS) && APRS_GpsFixValid();

    /* SmartBeaconing with no live GPS fix (position set to manual, or GPS mode
     * still searching) falls back to a plain fixed beacon at the profile's
     * slow rate. APRS_Beacon()'s own guard still blocks a zero-position GPS
     * beacon, so "manual + valid lat/lon" keeps working and "GPS + no fix"
     * stays silent. */
    if (!sb_live && (int32_t)(millis10() - s_next_beacon_10ms) < 0)
        return;

    /* channel busy (RX in progress) or already transmitting: hold off ~3 s on
     * the timed paths; live SmartBeaconing just re-checks on the next tick */
    if (gCurrentFunction == FUNCTION_TRANSMIT ||
        gCurrentFunction == FUNCTION_RECEIVE  ||
        gCurrentFunction == FUNCTION_MONITOR) {
        if (!sb_live)
            s_next_beacon_10ms = millis10() + 300;
        return;
    }

    if (sb_live) {
        if (!APRS_SmartBeaconDue())       /* latches its own timing on a yes   */
            return;
    } else {
        uint16_t period = sb ? s_sb_prof[APRS_SB_ROW][2]   /* SB fallback: slow rate */
                             : gAprsCfg.interval_s;
        s_next_beacon_10ms = millis10() + (uint32_t)period * 100u;
    }
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
    if (gAprsCfg.opts & APRS_OPT_KISS)  /* KISS: no display push, guard anyway */
        return;
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

/* NB: a TEST that kept the BK4829 AFC forced OFF on 144-148 MHz used to live
 * here (APRS_DisableAfc(), reasserted every tick like APRS_ApplySquelch()).
 * It mirrored the fix confirmed on air for the SARSAT screen, but on air it
 * made no difference to APRS decoding -- the residual bit errors were shown
 * to be RF-domain (noisier discriminator on this bench), not an LO drift --
 * so it was removed on the user's request. The AFC is back to its stock
 * MODULATION_FM behaviour (enabled), same as the V1 port. */

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
        if (s_rxi.kind == APRS_RX_KIND_MESSAGE)
            APRS_MsgCheckAck(s_rxi.name, s_rxi.text);
        if (s_rxi.flags & 0x10) {                  /* ADRASEC position request */
            s_adr.lat     = s_rxi.lat;
            s_adr.lon     = s_rxi.lon;
            s_adr.dist_hm = s_rxi.dist_hm;
            s_adr.brg     = s_rxi.bearing;
            s_adr.hasd    = (s_rxi.flags & 8) ? 1 : 0;
            memcpy(s_adr.src, s_rxi.src, sizeof s_adr.src);
            s_adrasec        = true;
            gAprsShowRequest = true;               /* open regardless of popup_s */
        }
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
       F_BLIGHT, F_DIGI, F_POS, F_LAT, F_LON, F_MSGTO, F_SEND, F_KISS, F_N };

/* char cycling for the keypad-poor text fields: space, A-Z, 0-9, then a few
 * punctuation marks for the comment. */
static const char CALLSET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
static const char TEXTSET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_./+";
static const char ADDRSET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-";
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
    { '/', '#', "digi"   },   /* green star -- APRS digipeater */
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
        if      (gAprsCfg.interval_s == 0)            strcpy(out, "Interval OFF");
        else if (gAprsCfg.interval_s == APRS_SB_CAR)  strcpy(out, "Interval SB car");
        else if (gAprsCfg.interval_s == APRS_SB_FOOT) strcpy(out, "Interval SB foot");
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
    case F_MSGTO:
        sprintf(out, "To  %.9s", gAprsCfg.msg_to[0] > ' ' ? gAprsCfg.msg_to : "(none)");
        break;
    case F_SEND:
        /* show the message number so the operator can match it to the "ackNN" */
        switch (s_msg.state) {
        case APRS_MSG_WAIT:
            sprintf(out, s_msg.tries < APRS_MSG_TRIES ? "Rpt #%02u TX %u/3"
                                                      : "Rpt #%02u wait ack",
                    s_msg.seq, s_msg.tries);
            break;
        case APRS_MSG_ACK:  sprintf(out, "Rpt #%02u ACK OK", s_msg.seq); break;
        case APRS_MSG_FAIL: sprintf(out, "Rpt #%02u no ack", s_msg.seq); break;
        default:            strcpy(out, "Send report");                  break;
        }
        break;
    case F_KISS:
        strcpy(out, (gAprsCfg.opts & APRS_OPT_KISS) ? "KISS TNC on"
                                                    : "KISS TNC off");
        break;
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
    static const uint16_t INTS[] = { 0, APRS_SB_CAR, APRS_SB_FOOT,
                                     30, 60, 120, 300, 600, 900, 1800 };
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
    case F_KISS:   gAprsCfg.opts ^= APRS_OPT_KISS; APRS_PushConfig(); break;
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

    if (editing && (sel == F_CALL || sel == F_MSGTO))
        sprintf(s, "char %d   A:ok", callcur + 1);
    else if (editing && sel == F_TEXT)
        sprintf(s, "TEXT pos %d UP/DN", callcur + 1);
    else if (editing && (sel == F_LAT || sel == F_LON))
        strcpy(s, "0-9  *:sign  A:ok");
    else if (editing)
        strcpy(s, "UP/DN then A:ok");
    else if (gAprsCfg.opts & APRS_OPT_KISS)
        strcpy(s, "KISS TNC  host USB");
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
        } else if (editing && f == sel && (f == F_TEXT || f == F_MSGTO)) {
            const char *b = (f == F_TEXT) ? gAprsCfg.comment : gAprsCfg.msg_to;
            const int   m = (f == F_TEXT) ? 14 : 9;   /* +[ ] brackets <= 18 */
            int w = 0;
            for (int i = 0; i < m; i++) {
                char ch = b[i] ? b[i] : ' ';
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

/* --- "Send report" wizard: ask Signal (0-9), then Direction (0-359) ------ */
static uint8_t s_wiz;       /* 0 none, 1 asking signal, 2 asking direction   */
static uint8_t s_wizsig;    /* signal digit captured in step 1              */

static void draw_wizard(void)
{
    char s[22];
    UI_DisplayClear();
    if (s_wiz == 1) {
        UI_PrintStringSmallBold("Signal 0-9 ?  0=KO", 2, 0, 2);
    } else {
        int w = sprintf(s, "S:%u  Dir? ", s_wizsig);
        for (int i = 0; i < s_klen && w < 16; i++) s[w++] = s_kbuf[i];
        s[w] = 0;
        UI_PrintStringSmallBold(s, 2, 0, 2);
        UI_PrintStringSmallNormal("0-359    A = send", 2, 0, 4);
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

/* int32 1e-5 deg -> "D MM SS.s H" (degrees / minutes / seconds) */
static void adr_dms(char *o, int32_t e5, char pos, char neg)
{
    long v   = e5 < 0 ? -(long)e5 : (long)e5;
    int  d   = (int)(v / 100000);
    long r   = v % 100000;
    long mx  = r * 60;                         /* arcmin * 1e5 */
    int  m   = (int)(mx / 100000);
    long sx  = (mx % 100000) * 60;             /* arcsec * 1e5 */
    int  s10 = (int)(sx / 10000);              /* arcsec * 10  */
    sprintf(o, "%d %02d %02d.%d %c", d, m, s10 / 10, s10 % 10, e5 < 0 ? neg : pos);
}

static void adr_dec(char *o, int32_t e5)
{
    long v = e5 < 0 ? -(long)e5 : (long)e5;
    sprintf(o, "%s%ld.%05ld", e5 < 0 ? "-" : "", v / 100000, v % 100000);
}

/* sticky coordinate screen for a "REPORT ADRASEC" message from PCT_Report:
 * decimal + degrees/minutes/seconds of the requested point. EXIT to close. */
static void draw_adrasec(void)
{
    char s[24];
    UI_DisplayClear();

    if (s_adr.hasd)                            /* distance/bearing to the point */
        sprintf(s, "ADRASEC %u.%ukm %03u",
                s_adr.dist_hm / 10, s_adr.dist_hm % 10, s_adr.brg);
    else
        sprintf(s, "ADRASEC de %.9s", s_adr.src);
    s[18] = 0;
    UI_PrintStringSmallBold(s, 2, 0, 0);

    /* row 1 blank: a gap under the header. rows 2/3 = lat, row 4 blank,
     * rows 5/6 = lon. (EXIT closes -- the screen is sticky.) */
    adr_dms(s, s_adr.lat, 'N', 'S'); UI_PrintStringSmallNormal(s, 2, 0, 2);
    adr_dec(s, s_adr.lat);           UI_PrintStringSmallNormal(s, 2, 0, 3);
    adr_dms(s, s_adr.lon, 'E', 'W'); UI_PrintStringSmallNormal(s, 2, 0, 5);
    adr_dec(s, s_adr.lon);           UI_PrintStringSmallNormal(s, 2, 0, 6);
    ST7565_BlitFullScreen();
}

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

    /* Manual key open on the APRS band -> re-tune the BK4819 for FM RX on the
     * current VFO. The APRS screen otherwise just inherits whatever RX state
     * it opened on -- BK4819 AF muted after a TX, not even demodulating:
     * "leger souffle, aucun souffle FM, pas de trame". RADIO_SetupRegisters()
     * sets the FM demod (un-mutes the BK4819 AF) + the squelch and leaves the
     * SPEAKER muted -- the mini-squelch loop below then opens it only on a
     * real carrier (so no persistent idle hiss). The operator's own AF-gain
     * setting is kept so the level matches what the C-Board sees. */
    const bool monitor = !popup && aprs_on_band();

    if (monitor) {
        RADIO_SelectVfos();
        RADIO_SetupRegisters(true);
        RADIO_SetModulation(gRxVfo->Modulation);   /* un-mute the BK4819 AF to
                                                    * the VFO's demod (this
                                                    * firmware's RADIO_Setup-
                                                    * Registers() already does
                                                    * it, harmless to repeat) */
        gEnableSpeaker = true;
        APRS_ApplySquelch();
        AFGAIN_Apply();
        g_SquelchLost = false;
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    } else if (!g_SquelchLost) {          /* channel already idle -> silence it now */
        AUDIO_AudioPathOff();
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    }

    /* auto-popup (from the tick, on a decoded packet) opens on the RX view;
     * a manual key always opens on the config menu (press * for the last RX). */
    int sel = 0, editing = 0, callcur = 0, first = 0,
        view = s_adrasec ? 2 : (popup ? 1 : 0);
    bool armed = false, run = true, closed_by_key = false;
    KEY_Code_t prev = KEY_INVALID;
    bool dirty = true;
    uint16_t last_pkts = s_rx_pkts;
    uint8_t  last_msg   = s_msg.state;
    uint8_t  last_tries = s_msg.tries;   /* redraw the "Send report" row on each (re)send */
    s_wiz = 0;

    /* auto-popups close themselves after gAprsCfg.popup_s (unless a key is hit);
     * the ADRASEC screen never auto-closes -- EXIT only */
    const bool timed = (popup && gAprsCfg.popup_s && !s_adrasec);
    uint32_t close_at = timed ? millis10() + gAprsCfg.popup_s * 100u : 0;
    uint32_t msq_mute_at = 0;    /* deferred speaker mute (see mini squelch) */

    BACKLIGHT_TurnOn();

    while (run) {
#ifdef ENABLE_UART
        while (UART_IsCommandAvailable(UART_PORT_UART))
            UART_HandleCommand(UART_PORT_UART);
#endif
        APRS_ApplySquelch();     /* the tick can't -- keep the APRS-band fast
                                  * squelch on so packets that arrive while this
                                  * screen is open are not chopped (1 reg read) */
        /* mini squelch: the main loop (which normally does this) is blocked.
         * carrier present -> speaker path + RX LED on (the C-Board hears it).
         * carrier gone    -> mute the speaker, but only ~2 s LATER: APRS
         * packets come in bursts a fraction of a second apart, and cutting the
         * audio feed to the C-Board between them breaks its bit-PLL / HDLC
         * framing -> "la LED s'allume, du souffle, mais la trame ne passe pas".
         * The deferred mute keeps the feed continuous across those short gaps
         * and only silences the channel after a real lull. */
        while (BK4819_ReadRegister(BK4819_REG_0C) & 1u) {
            BK4819_WriteRegister(BK4819_REG_02, 0);
            uint16_t ib = BK4819_ReadRegister(BK4819_REG_02);
            if (ib & BK4819_REG_02_SQUELCH_LOST) {
                g_SquelchLost = true;
                msq_mute_at = 0;
                AUDIO_AudioPathOn();
                BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true);
            }
            if (ib & BK4819_REG_02_SQUELCH_FOUND) {
                g_SquelchLost = false;
                if (!msq_mute_at) msq_mute_at = millis10() + 200;   /* +2 s */
                BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
            }
        }
        if (msq_mute_at && (int32_t)(millis10() - msq_mute_at) >= 0) {
            msq_mute_at = 0;
            AUDIO_AudioPathOff();
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
        APRS_MsgTimeSlice();             /* keep the "121 MHz report" retrying */

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

        /* a new decoded packet: only an AUTO-popup reacts to it -- keep it on
         * the RX view and re-arm its close timer while traffic flows. A
         * MANUAL open (F+5) stays put on whatever the user is looking at:
         * decoding (and the report-ack match) runs in the background either
         * way; press * for the last RX frame. */
        if (s_rx_pkts != last_pkts) {
            last_pkts = s_rx_pkts;
            if (close_at) {
                close_at = millis10() + gAprsCfg.popup_s * 100u;
                if (!editing && !s_wiz) view = 1;
            }
        }
        if (close_at && (int32_t)(millis10() - close_at) >= 0)
            break;

        /* the message state advances from the tick / this loop's own
         * APRS_MsgTimeSlice(); redraw the F_SEND row when it changes */
        if (s_msg.state != last_msg || s_msg.tries != last_tries) {
            last_msg = s_msg.state; last_tries = s_msg.tries; dirty = true;
        }

        /* an ADRASEC request that arrived while this screen was already open */
        if (s_adrasec && view != 2) { view = 2; close_at = 0; dirty = true; }

        if (sel < first)                    first = sel;
        if (sel > first + APRS_VIS_ROWS - 1) first = sel - (APRS_VIS_ROWS - 1);
        if ((view == 1 || view == 2) && s_rx_dirty) { s_rx_dirty = false; dirty = true; }
        if (dirty) {
            dirty = false;
            if      (s_wiz)     draw_wizard();
            else if (view == 2) draw_adrasec();
            else if (view == 1) draw_rx(!popup);   /* popup: hide the "APRS RX" line */
            else               draw_config(sel, editing, callcur, first);
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

        if (s_wiz) {                          /* "Send report" wizard */
            if (s_wiz == 1) {                 /* step 1: signal digit 0-9 */
                if (k <= KEY_9) {
                    s_wizsig = (uint8_t)k;
                    if (s_wizsig == 0) { APRS_MsgStart(0, 0); s_wiz = 0; }
                    else               { s_wiz = 2; s_klen = 0; }
                } else if (k == KEY_EXIT) {
                    s_wiz = 0;
                }
            } else {                          /* step 2: direction 0-359 */
                if (k <= KEY_9 && s_klen < 3) {
                    int v = 0;
                    for (int i = 0; i < s_klen; i++) v = v * 10 + (s_kbuf[i] - '0');
                    v = v * 10 + (int)k;
                    if (v <= 359) s_kbuf[s_klen++] = (char)('0' + (int)k);
                } else if (k == KEY_MENU && s_klen) {   /* A: send (need a value) */
                    int v = 0;
                    for (int i = 0; i < s_klen; i++) v = v * 10 + (s_kbuf[i] - '0');
                    APRS_MsgStart(s_wizsig, (uint16_t)v);
                    s_wiz = 0;
                } else if (k == KEY_EXIT) {
                    s_wiz = 0;
                }
            }
            APRS_TickDelay(20);
            continue;
        }

        if (view == 2) {                      /* ADRASEC: sticky, EXIT only */
            if (k == KEY_EXIT) { run = false; closed_by_key = true; }
            continue;
        }

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
                if (sel == F_SEND) {          /* launch the report wizard */
                    if (gAprsCfg.call[0] && strcmp(gAprsCfg.call, "NOCALL") != 0 &&
                        gAprsCfg.msg_to[0] > ' ') {
                        s_wiz = 1; s_wizsig = 0; s_klen = 0;
                    }
                    break;
                }
                editing = 1; callcur = 0; s_klen = 0;
                s_kneg = (sel == F_LAT) ? (gAprsCfg.lat_e5 < 0) :
                         (sel == F_LON) ? (gAprsCfg.lon_e5 < 0) : 0;
                break;
            case KEY_STAR: view = 1;                    break;   /* -> RX view */
            case KEY_5:    APRS_Beacon();               break;
            case KEY_EXIT: run = false; closed_by_key = true; break;
            default: break;
            }
        } else if (sel == F_CALL || sel == F_TEXT || sel == F_MSGTO) {
            const bool txt  = (sel == F_TEXT);
            const bool addr = (sel == F_MSGTO);
            char       *buf = addr ? gAprsCfg.msg_to
                                   : txt ? gAprsCfg.comment : gAprsCfg.call;
            const int   fw  = addr ? 9 : txt ? 14 : 6;
            const char *set = addr ? ADDRSET : txt ? TEXTSET : CALLSET;
            switch (k) {
            case KEY_UP:   buf[callcur] = cyc_char(buf[callcur], +1, set); break;
            case KEY_DOWN: buf[callcur] = cyc_char(buf[callcur], -1, set); break;
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
    s_adrasec             = false;   /* consumed; a fresh request re-opens it */
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
