/*
 * main.c — RP2040 COSPAS-SARSAT 406 MHz 1st-generation beacon decoder.
 *
 * Pipeline:  radio FM-discriminator AF (flat) -> ADC0 (DMA, CFG_SAMPLE_RATE_HZ)
 * -> burst detect -> audio_slicer -> dec406_v1g (T.001) -> Quansheng 0xABCD
 * frames to the radio SARSAT screen.
 *
 * RX/monitoring only. This firmware never keys a transmitter. Real distress
 * alerts must be reported to the SAR authorities (CROSS / COSPAS-SARSAT).
 *
 * Debug console: USB-CDC (SARSAT_LOG in decoder_config.h). Lines are tagged:
 *   [lvl]    idle audio level (set the radio volume so peak sits ~4000-12000)
 *   [burst]  level crossed the threshold, running the decoder
 *   [slicer] slicer result (bit count + raw hex, or "no sync")
 *   [decode] BCH result and decoded fields
 *   [tx]     frame(s) sent to the radio
 * Not run on RP2040 hardware yet.
 */
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "decoder_config.h"
#include "sarsat_decoder.h"
#include "quansheng_frame.h"
#include "country_codes.h"
#include "aprs_rx.h"
#include "aprs_parse.h"
#include "aprs_digi.h"
#include "gps_nmea.h"

#if SARSAT_LOG
#define LOG(...)  printf(__VA_ARGS__)
#else
#define LOG(...)  ((void)0)
#endif

#define WINDOW_SAMPLES ((CFG_SAMPLE_RATE_HZ / 1000) * CFG_WINDOW_MS)

static uint16_t  g_adc_raw[WINDOW_SAMPLES];
static int32_t   g_audio[WINDOW_SAMPLES];
static int       g_dma_chan;
static long      g_dc = (long)CFG_ADC_DC_CENTER << 8;   /* DC estimate, raw<<8 */
static uint32_t  g_tx_bytes;                      /* running total to the radio */

typedef struct { int peak; int rms; int raw_min; int raw_max; long dc; int clipped; } audio_stats_t;

/* ---- radio link (Phase 2: read the radio's ACK / status replies) ------ */
static qframe_rx_t    g_rx;
static absolute_time_t g_last_reply;
static int            g_link_up = -1;             /* -1 unknown, 0 down, 1 up */
static uint32_t       g_acks;

/* ---- SARSAT / APRS mode (auto-selected from the radio's RX frequency) --- */
enum { MODE_SARSAT = 0, MODE_APRS = 1 };
static int  g_mode         = MODE_SARSAT;
static int  g_mode_pending  = MODE_SARSAT;
static bool g_lvl_view      = false;   /* radio's SARSAT level screen is open */

/* APRS: continuous ADC ring (DMA wraps the write address); the main loop
 * drains it into the AFSK demod. */
#define APRS_RING_SAMPLES 8192                    /* 2^13, ~620 ms @ 13200 Hz;  */
#define APRS_RING_BITS     14                     /* log2(SAMPLES*2) -- must    */
                                                  /* survive a long frame +    */
                                                  /* the blocking radio_send / */
                                                  /* console printf in the cb  */
static uint16_t g_aprs_ring[APRS_RING_SAMPLES]
    __attribute__((aligned(APRS_RING_SAMPLES * 2)));
static uint32_t g_aprs_rd;
static aprs_rx_t g_aprs;
static uint32_t  g_aprs_pkts;
static int32_t   g_my_lat_e5, g_my_lon_e5;   /* operator position (HELLO reply) */
static uint8_t   g_radio_mod = 0xFF;         /* last modulation from a HELLO reply */
static uint8_t   g_aprs_digi_level;          /* 0=off, pushed by the radio via
                                              * CMD_APRS_CONFIG, see aprs_digi.h */
static char      g_aprs_my_call[7];          /* radio's own call/SSID, pushed the
                                              * same way, for traceable digipeating
                                              * (aprs_digi_process()) -- "" if the
                                              * radio has none configured yet    */
static uint8_t   g_aprs_my_ssid;

#if CFG_APRS_RX_DIAG
/* provisional: 'w' arms a one-shot raw-ADC capture of the next carrier, then
 * dumps it as [aprs.raw] hex so a full frame can be decoded off-line on the
 * host (rp2040/test/host/test_aprs_wav) with per-bit instrumentation. ~1.4 s. */
#define APRS_DUMP_SAMPLES 18000
static uint16_t g_dump[APRS_DUMP_SAMPLES];
static uint32_t g_dump_n;
static enum { DUMP_IDLE, DUMP_ARMED, DUMP_RECORDING, DUMP_READY } g_dump_st;
#endif

static void radio_uart_init(void)
{
    uart_init(CFG_RADIO_UART, CFG_RADIO_UART_BAUD);
    gpio_set_function(CFG_RADIO_UART_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(CFG_RADIO_UART_RX_GPIO, GPIO_FUNC_UART);
    uart_set_format(CFG_RADIO_UART, 8, 1, UART_PARITY_NONE);
    g_last_reply = get_absolute_time();
}

static const char *mod_name(uint8_t m)
{
    static const char *n[] = { "FM", "AM", "USB", "BYP", "RAW", "DSC" };
    return (m < 6) ? n[m] : "?";
}
/* a modulation the C-Board decoder can work from (FM discriminator audio) */
#define MOD_IS_FM_LIKE(m)  ((m) == 0 /*FM*/ || (m) == 5 /*DSC*/)

/* Drain the radio->RP2040 direction, log ACK / status replies. */
static void link_poll(void)
{
    static uint32_t last_f10 = 0xFFFFFFFF;
    static uint8_t  last_vfo = 0xFF, last_mod = 0xFF, last_scr = 0xFF;
    uint16_t id, dl;
    const uint8_t *d;

    while (uart_is_readable(CFG_RADIO_UART)) {
        uint8_t b = uart_getc(CFG_RADIO_UART);
        if (!quansheng_frame_feed(&g_rx, b, &id, &d, &dl))
            continue;

        g_last_reply = get_absolute_time();
        g_acks++;
        if (g_link_up != 1) { g_link_up = 1; LOG("[link]   radio replying - link UP\n"); }

        if (id == (CMD_SARSAT_HELLO | 0x8000) && dl >= 7) {
            uint8_t  vfo = d[0] & 1, mod = d[1], scr = d[6] ? 1 : 0;
            g_lvl_view = (d[6] == 2);   /* 2 = SARSAT level view open -> fast telemetry */
            uint32_t f10 = d[2] | (d[3] << 8) | (d[4] << 16) | ((uint32_t)d[5] << 24);

            if (f10 != last_f10 || vfo != last_vfo || mod != last_mod || scr != last_scr) {
                if (f10 == 0)
                    LOG("[link]   status: VFO %c  %s  RX freq unknown  screen %s\n",
                        'A' + vfo, mod_name(mod), scr ? "open" : "closed");
                else
                    LOG("[link]   status: VFO %c  %s  RX %lu.%05lu MHz  screen %s\n",
                        'A' + vfo, mod_name(mod),
                        (unsigned long)(f10 / 100000), (unsigned long)(f10 % 100000),
                        scr ? "open" : "closed");
                if (!MOD_IS_FM_LIKE(mod))
                    LOG("[link]   ! radio not in FM/DSC - decode needs FM discriminator audio\n");
                last_f10 = f10; last_vfo = vfo; last_mod = mod; last_scr = scr;
            }
            g_radio_mod = mod;

            /* pick the decoder from the RX frequency: 2 m -> APRS, else SARSAT */
            if (f10 != 0)
                g_mode_pending = (f10 >= CFG_APRS_BAND_LO_10HZ &&
                                  f10 <= CFG_APRS_BAND_HI_10HZ)
                                     ? MODE_APRS : MODE_SARSAT;

            /* bytes 8..15: the operator's APRS position (for distance/bearing) */
            if (dl >= 16) {
                g_my_lat_e5 = (int32_t)(d[8] | (d[9] << 8) |
                                        (d[10] << 16) | ((uint32_t)d[11] << 24));
                g_my_lon_e5 = (int32_t)(d[12] | (d[13] << 8) |
                                        (d[14] << 16) | ((uint32_t)d[15] << 24));
            }
        } else if (id == CMD_APRS_CONFIG && dl >= 11) {
            /* {call[6], ssid, path, sym_table, sym_code, digi_level} */
            memcpy(g_aprs_my_call, d, 6);
            g_aprs_my_call[6] = 0;
            g_aprs_my_ssid = d[6];
            if (d[10] != g_aprs_digi_level) {
                static const char *const lv[] = { "off", "WIDE1", "WIDE1+2", "WIDE1+2+3" };
                g_aprs_digi_level = (d[10] <= 3) ? d[10] : 0;
                LOG("[digi]   level: %s  call: %s-%u\n", lv[g_aprs_digi_level],
                    g_aprs_my_call, g_aprs_my_ssid);
            }
        } else if ((id & 0x8000) && (id & 0x00FF) >= 0xC0) {
#if CFG_TX_HEXDUMP
            LOG("[link]   ACK 0x%04X status=%u\n", id & 0x7FFF, dl ? d[0] : 0);
#endif
        }
    }
}

static void radio_send(uint16_t cmd, const uint8_t *data, size_t len)
{
    uint8_t frame[300];
    size_t n = quansheng_frame_build(cmd, data, len, frame, sizeof(frame));
    if (!n) { LOG("[tx]     ERR: frame build failed (cmd %04X)\n", cmd); return; }
    uart_write_blocking(CFG_RADIO_UART, frame, n);
    g_tx_bytes += n;
#if CFG_TX_HEXDUMP
    LOG("[tx]     %04X %u+%u B :", cmd, (unsigned)(n - len - 12), (unsigned)len);
    for (size_t i = 0; i < n; i++) LOG(" %02X", frame[i]);
    LOG("\n");
#endif
}

static void radio_send_level(const audio_stats_t *st)
{
    uint16_t pk = st->peak > 65535 ? 65535 : (uint16_t)st->peak;
    uint16_t rm = st->rms  > 65535 ? 65535 : (uint16_t)st->rms;
    uint16_t dc = (uint16_t)st->dc;
    /* Tuning verdict (no beacon present -> st is the hiss). With the relative
     * burst detector the beacon decodes at any non-clipping hiss level, so HOT
     * is now purely an APRS-clip caution: the shared audio tap is also used for
     * strong 2 m packets, which rail the ADC if the gain is pushed hard. */
    uint8_t verdict =
        st->clipped > 0                 ? SARSAT_LVL_CLIP :
        (st->dc < 700 || st->dc > 3400) ? SARSAT_LVL_BIAS :
        (st->rms > 4500 || st->peak > 15000) ? SARSAT_LVL_HOT :
        st->rms < 800                   ? SARSAT_LVL_LOW  : SARSAT_LVL_OK;
    uint8_t p[12] = {
        pk, pk >> 8, rm, rm >> 8, dc, dc >> 8, (uint8_t)st->clipped,
        (uint8_t)st->raw_min, (uint8_t)(st->raw_min >> 8),
        (uint8_t)st->raw_max, (uint8_t)(st->raw_max >> 8), verdict,
    };
    radio_send(CMD_SARSAT_LEVEL, p, sizeof(p));
}

static void radio_send_text(uint8_t line, uint8_t invert, const char *s)
{
    uint8_t p[SARSAT_LINE_CHARS + 2];
    size_t sl = strnlen(s, SARSAT_LINE_CHARS - 1);
    p[0] = line;
    p[1] = invert;
    memcpy(p + 2, s, sl);
    radio_send(CMD_SARSAT_TEXT, p, sl + 2);
    LOG("[tx]     TEXT L%u%s \"%s\"\n", line, invert ? " (inv)" : "", s);
}

static void radio_push_result(const sarsat_result_t *r)
{
    uint32_t before = g_tx_bytes;
    radio_send(CMD_SARSAT_CLEAR, NULL, 0);
    LOG("[tx]     CLEAR\n");
    for (int i = 0; i < r->n_lines; i++) {
        radio_send_text((uint8_t)i, (i == 0), r->lines[i]);
        sleep_ms(8);   /* let the radio's 256 B UART ring drain between frames */
        link_poll();   /* and drain our RX FIFO of the radio's ACKs */
    }
    LOG("[tx]     pushed %d lines, %lu bytes on UART%d @ %d\n",
        r->n_lines, (unsigned long)(g_tx_bytes - before),
        (CFG_RADIO_UART == uart0) ? 0 : 1, CFG_RADIO_UART_BAUD);
}

/* ---- audio capture --------------------------------------------------- */
static void adc_capture_init(void)
{
    adc_init();
    /* GP26 plus any pad shorted to it on the C-Board (GP27/GP28): analog mode,
     * so the unused pins don't load the audio node. */
    for (int i = 0; i < 3; i++)
        if (CFG_ADC_SHORTED_MASK & (1u << i))
            adc_gpio_init(26 + i);
    adc_gpio_init(CFG_ADC_GPIO);
    adc_select_input(CFG_ADC_CHANNEL);
    adc_fifo_setup(true, true, 1, false, false);
    /* 48 MHz ADC clock / (1 + div) = sample rate */
    adc_set_clkdiv((float)48000000.0f / (float)CFG_SAMPLE_RATE_HZ - 1.0f);

    g_dma_chan = dma_claim_unused_channel(true);
}

static void adc_capture_window(int n)
{
    if (n > WINDOW_SAMPLES) n = WINDOW_SAMPLES;
    if (n < 1)              n = 1;

    dma_channel_config c = dma_channel_get_default_config(g_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);

    adc_fifo_drain();
    adc_run(false);
    dma_channel_configure(g_dma_chan, &c, g_adc_raw, &adc_hw->fifo, n, true);
    adc_run(true);
    dma_channel_wait_for_finish_blocking(g_dma_chan);
    adc_run(false);
}

static uint32_t isqrt_u64(uint64_t v)
{
    uint64_t x = v, r = 0, b = (uint64_t)1 << 62;
    while (b > x) b >>= 2;
    while (b) {
        if (x >= r + b) { x -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return (uint32_t)r;
}


/* ---- interactive serial console (level tuning) ----------------------- */
static bool g_meter;          /* continuous [meter] output for volume setting */

static void console_help(void)
{
    LOG("\n--- serial console -------------------------------------------\n"
        " m : toggle the level METER (per-window peak/rms/dc/clip/adc + bar)\n"
        "     use it to set the radio volume: aim for the HISS at\n"
        "     rms ~2000-3500, clip 0%%, dc near %d. A beacon then reads lower.\n"
        " s : one-shot status line\n"
        " h : this help\n"
        "-------------------------------------------------------------\n\n",
        CFG_ADC_DC_CENTER);
}

static void meter_print(const audio_stats_t *st, int noise_rms, const char *tag)
{
    /* 20-char bar: rms scaled so CFG_DECODE_MAX_RMS lands at ~2/3 */
    int fill = (int)((long)st->rms * 20 / (CFG_DECODE_MAX_RMS * 3 / 2));
    if (fill > 20) fill = 20;
    char bar[21];
    for (int i = 0; i < 20; i++) bar[i] = (i < fill) ? '#' : '-';
    bar[20] = 0;

    LOG("[meter]%s rms=%-5d peak=%-5d dc=%-4ld clip=%-2d%% adc[%d..%d] noise=%-4d |%s| %s\n",
        tag, st->rms, st->peak, st->dc, st->clipped, st->raw_min, st->raw_max,
        noise_rms, bar,
        st->clipped > 0        ? "CLIP - lower volume" :
        st->dc < 700 || st->dc > 3400 ? "check ADC bias" :
        st->rms > CFG_DECODE_MAX_RMS ? "hiss too hot"   :
        st->rms < 800          ? "a bit low"            : "ok");
}

static void console_poll(const audio_stats_t *st, int noise_rms)
{
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        switch (c) {
        case 'm': case 'M':
            g_meter = !g_meter;
            LOG("[meter]  %s\n", g_meter ? "ON  (press m to stop)" : "OFF");
            break;
        case 's': case 'S':
            meter_print(st, noise_rms, " ");
            break;
#if CFG_APRS_RX_DIAG
        case 'w': case 'W':
            g_dump_st = DUMP_ARMED;
            LOG("[aprs.raw] armed - waiting for the next carrier "
                "(tune 144.800, let a beacon come)\n");
            break;
#endif
        case 'h': case 'H': case '?':
            console_help();
            break;
        default:
            break;
        }
        c = getchar_timeout_us(0);
    }
}

/* Convert raw ADC window to signed audio; fill *st.
 * g_dc holds the DC estimate as raw<<8 fixed point. */
static void window_to_audio(audio_stats_t *st, int n)
{
    if (n > WINDOW_SAMPLES) n = WINDOW_SAMPLES;
    if (n < 1)              n = 1;
    int peak = 0, raw_min = 4095, raw_max = 0, clipped = 0;
    uint64_t sq = 0;
    for (int i = 0; i < n; i++) {
        int raw = g_adc_raw[i] & 0x0FFF;
        if (raw < raw_min) raw_min = raw;
        if (raw > raw_max) raw_max = raw;
        if (raw == 0 || raw == 4095) clipped++;

        /* Hardware bias sets the operating point now; keep the DC tracker very
         * slow so it does not chase the FM-hiss envelope between beacons. */
        g_dc += (((long)raw << 8) - g_dc) >> 15;      /* ~2 s tracker */
        long s = ((long)raw - (g_dc >> 8)) << CFG_AUDIO_GAIN_SHIFT;
        if (s >  32767) s =  32767;
        if (s < -32768) s = -32768;
        g_audio[i] = (int32_t)s;

        int a = (s < 0) ? -s : s;
        if (a > peak) peak = a;
        sq += (uint64_t)((int64_t)s * s);
    }
    st->peak    = peak;
    st->rms     = (int)isqrt_u64(sq / (unsigned)n);
    st->raw_min = raw_min;
    st->raw_max = raw_max;
    st->dc      = g_dc >> 8;
    st->clipped = (clipped * 100) / n;                /* % of samples at a rail */
}

/* ---- APRS RX (2 m packet) ------------------------------------------------ */
static void aprs_put_u16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void aprs_put_i32(uint8_t *p, int32_t v)
{
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = (uint32_t)v >> 24;
}

/* ---- GPS (NMEA on UART1) ----------------------------------------------- */
#if CFG_GPS_ENABLE
static gps_t           g_gps;
static absolute_time_t g_gps_fix_at;      /* nil until the first valid RMC   */
static absolute_time_t g_gps_rx_at;       /* last byte seen on the GPS UART  */

static bool gps_have_fix(void)
{
    return g_gps.fix.valid && !is_nil_time(g_gps_fix_at) &&
           absolute_time_diff_us(g_gps_fix_at, get_absolute_time()) < 10000000;
}

/* a GPS module is talking to us right now (bytes in the last few seconds) */
static bool gps_present(void)
{
    return !is_nil_time(g_gps_rx_at) &&
           absolute_time_diff_us(g_gps_rx_at, get_absolute_time()) < 5000000;
}

static void gps_uart_init(void)
{
    gps_init(&g_gps);
    g_gps_fix_at = nil_time;
    g_gps_rx_at  = nil_time;
    uart_init(CFG_GPS_UART, CFG_GPS_BAUD);
    gpio_set_function(CFG_GPS_UART_RX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(CFG_GPS_UART_TX_GPIO, GPIO_FUNC_UART);
    uart_set_format(CFG_GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(CFG_GPS_UART, true);
}

static void gps_service(void)
{
    while (uart_is_readable(CFG_GPS_UART)) {
        char c = (char)uart_getc(CFG_GPS_UART);
        g_gps_rx_at = get_absolute_time();
        if (gps_feed(&g_gps, c) && g_gps.fix.valid)
            g_gps_fix_at = get_absolute_time();
    }
}

/* forward the fix to the radio (its beacon uses it when "Pos GPS" is set;
 * the radio also drives its top-bar GPS symbol from these frames) */
static void gps_push_to_radio(void)
{
    if (!gps_present())               /* no module talking -> stay silent    */
        return;
    const bool ok = gps_have_fix();
    uint8_t p[16];
    p[0] = ok ? 1 : 0;
    aprs_put_i32(p + 1, ok ? g_gps.fix.lat_e5 : 0);
    aprs_put_i32(p + 5, ok ? g_gps.fix.lon_e5 : 0);
    aprs_put_u16(p + 9,  g_gps.fix.speed_kmh);
    aprs_put_u16(p + 11, g_gps.fix.course_deg);
    aprs_put_u16(p + 13, (uint16_t)g_gps.fix.alt_m);
    p[15] = g_gps.fix.sats;
    radio_send(CMD_APRS_GPS, p, sizeof p);
}
#endif /* CFG_GPS_ENABLE */

/* operator position for the RX range/bearing: a live GPS fix wins, else the
 * manual position the radio sent in its HELLO reply. */
static void my_position(int32_t *lat, int32_t *lon)
{
#if CFG_GPS_ENABLE
    if (gps_have_fix()) { *lat = g_gps.fix.lat_e5; *lon = g_gps.fix.lon_e5; return; }
#endif
    *lat = g_my_lat_e5;
    *lon = g_my_lon_e5;
}

/* WIDEn-N digipeat: attempted on every valid frame regardless of whether its
 * info field parses for display (a digipeater repeats the whole packet, it
 * does not need to understand it). See aprs_digi.h for the decision rule. */
static void aprs_try_digipeat(const uint8_t *ax25, int len)
{
    if (g_aprs_digi_level == APRS_DIGI_OFF || len <= 0 || len > 256)
        return;

    uint8_t d[256];
    memcpy(d, ax25, (size_t)len);
    /* may come back longer than `len`: a traced hop inserts a 7-byte address
     * ahead of a still-generic alias (see aprs_digi.h) -- `sizeof d` bounds
     * how far that growth is allowed to go. */
    int nlen = aprs_digi_process(d, len, (int)sizeof d, g_aprs_digi_level,
                                 g_aprs_my_call, g_aprs_my_ssid);
    if (!nlen)
        return;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (aprs_digi_seen_recently(d, nlen, now_ms))
        return;                        /* already repeated this recently */

    LOG("[digi]   repeating #%lu (%d -> %d bytes)\n",
        (unsigned long)g_aprs_pkts, len, nlen);
    radio_send(CMD_APRS_DIGI, d, (size_t)nlen);
    link_poll();
}

/* Called from aprs_rx on a full FCS-valid AX.25 frame. Parse the APRS info and
 * push a structured 0x06D3 to the radio (falls back to 0x06D2 raw text if the
 * info field wasn't understood). */
static void aprs_on_packet(const uint8_t *ax25, int len, void *user)
{
    (void)user;
    g_aprs_pkts++;
    aprs_try_digipeat(ax25, len);

    aprs_info_t ai;
    bool parsed = aprs_parse(ax25, len, &ai);

    if (parsed && (ai.has_pos || ai.kind == APRS_KIND_STATUS ||
                   ai.kind == APRS_KIND_MESSAGE)) {
        uint32_t dist_m = 0; uint16_t brg = 0; bool has_dist = false;
        int32_t my_lat, my_lon;
        my_position(&my_lat, &my_lon);
        if (ai.has_pos && (my_lat || my_lon))
            has_dist = aprs_geo(my_lat, my_lon, ai.lat_e5, ai.lon_e5, &dist_m, &brg);

        char via[28];
        ax25_via_str(ax25, len, via, sizeof via);

        uint8_t p[160];
        int n = 0;
        p[n++] = ai.kind;
        p[n++] = (uint8_t)((ai.has_pos ? 1 : 0) | (ai.has_course ? 2 : 0) |
                           (ai.has_alt ? 4 : 0) | (has_dist ? 8 : 0));
        p[n++] = (uint8_t)ai.sym_table;
        p[n++] = (uint8_t)ai.sym_code;
        aprs_put_i32(p + n, ai.lat_e5); n += 4;
        aprs_put_i32(p + n, ai.lon_e5); n += 4;
        aprs_put_u16(p + n, (uint16_t)ai.course_deg); n += 2;
        aprs_put_u16(p + n, (uint16_t)ai.speed_kmh);  n += 2;
        aprs_put_u16(p + n, (uint16_t)(int16_t)ai.alt_m); n += 2;
        aprs_put_u16(p + n, (uint16_t)(dist_m / 100 > 65535 ? 65535 : dist_m / 100));
        n += 2;
        aprs_put_u16(p + n, brg); n += 2;
        for (const char *s = ai.src;  *s && n < 100; ) p[n++] = *s++;
        p[n++] = 0;
        for (const char *s = ai.name; *s && n < 120; ) p[n++] = *s++;
        p[n++] = 0;
        for (const char *s = via;     *s && n < 150; ) p[n++] = *s++;
        p[n++] = 0;
        for (const char *s = ai.text; *s && n < 158; ) p[n++] = *s++;
        p[n++] = 0;

        LOG("[aprs]   #%lu  %s  %s%s  %s%s\n", (unsigned long)g_aprs_pkts, ai.src,
            via[0] ? "via " : "direct", via, ai.has_pos ? "pos " : "", ai.text);
        radio_send(CMD_APRS_RXINFO, p, (size_t)n);
        link_poll();
        return;
    }

    /* fallback: raw text lines */
    char lines[4][SARSAT_LINE_CHARS + 1];
    int  nl = aprs_rx_format(ax25, len, (char *)lines, 4, SARSAT_LINE_CHARS - 1);
    LOG("[aprs]   #%lu  %s\n", (unsigned long)g_aprs_pkts, nl > 0 ? lines[0] : "?");
    for (int i = 1; i < nl; i++)
        LOG("[aprs]        %s\n", lines[i]);

    uint8_t clr = 0xFF;
    radio_send(CMD_APRS_RXTEXT, &clr, 1);
    for (int i = 0; i < nl && i < 4; i++) {
        uint8_t p[1 + SARSAT_LINE_CHARS];
        size_t  sl = strnlen(lines[i], SARSAT_LINE_CHARS - 1);
        p[0] = (uint8_t)i;
        memcpy(p + 1, lines[i], sl);
        radio_send(CMD_APRS_RXTEXT, p, sl + 1);
        sleep_ms(8);
        link_poll();
    }
}

#if CFG_APRS_RX_DIAG
/* provisional: log HDLC frame candidates + why they did/didn't decode.
 * SHORT fragments (noise finding stray flags) are skipped -- they are the
 * majority and the console printf they'd cost can stall the sample pump. */
static void aprs_rx_diag(void *u, const uint8_t *f, int len, int ch, int res, int ui)
{
    (void)u;
    if (res == APRS_RXR_SHORT)
        return;
    static const char *const R[] = { "OK ", "FIX", "BAD", "SHT", "LNG", "DUP" };

    /* AX.25 source callsign lives in bytes 7..13 (>>1); dst in 0..6 */
    char src[10] = "?";
    if (len >= 14 && (res == APRS_RXR_OK || res == APRS_RXR_FIXED ||
                      res == APRS_RXR_FCS_BAD || res == APRS_RXR_DEDUP)) {
        int k = 0;
        for (int i = 7; i < 13 && k < 6; i++) {
            char c = (char)(f[i] >> 1);
            if (c > ' ' && c < 0x7f) src[k++] = c;
        }
        int ssid = (f[13] >> 1) & 0x0F;
        if (k == 0) k = 1, src[0] = '?';
        if (ssid) { src[k++] = '-'; src[k++] = (char)('0' + ssid); }
        src[k] = 0;
    }

    char hex[3 * 20 + 4];
    int  hn = 0, nb = len < 20 ? len : 20;
    for (int i = 0; i < nb; i++)
        hn += snprintf(hex + hn, sizeof hex - hn, "%02X ", f[i]);
    if (len > nb) snprintf(hex + hn, sizeof hex - hn, "...");

    LOG("[aprs.rx] ch%d %s len=%-3d ui=%d src=%-8s | %s\n",
        ch, R[res < 6 ? res : 2], len, ui, src, hex);
}
#endif

static void aprs_mode_enter(void)
{
    adc_run(false);
    dma_channel_abort(g_dma_chan);
    adc_fifo_drain();
    adc_set_clkdiv((float)48000000.0f / (float)APRS_RX_SAMPLE_RATE_HZ - 1.0f);

    aprs_rx_init(&g_aprs, APRS_RX_SAMPLE_RATE_HZ, aprs_on_packet, NULL);
#if CFG_APRS_RX_DIAG
    aprs_rx_set_frame_cb(&g_aprs, aprs_rx_diag, NULL);
#endif
    g_aprs_rd = 0;

    dma_channel_config c = dma_channel_get_default_config(g_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, APRS_RING_BITS);   /* wrap the write addr */
    channel_config_set_dreq(&c, DREQ_ADC);
    dma_channel_configure(g_dma_chan, &c, g_aprs_ring, &adc_hw->fifo,
                          0xFFFFFFFFu, true);
    adc_run(true);

    g_mode = MODE_APRS;
    LOG("[aprs]   mode APRS  (%d Hz, ring %d smp)  -- tune the radio to "
        "144.800 MHz FM\n", APRS_RX_SAMPLE_RATE_HZ, APRS_RING_SAMPLES);
}

static void sarsat_mode_enter(void)
{
    adc_run(false);
    dma_channel_abort(g_dma_chan);
    adc_fifo_drain();
    adc_set_clkdiv((float)48000000.0f / (float)CFG_SAMPLE_RATE_HZ - 1.0f);
    g_mode = MODE_SARSAT;
    LOG("[link]   mode SARSAT  (%d Hz)  -- 406 / 434.2 MHz FM\n",
        CFG_SAMPLE_RATE_HZ);
}

/* Drain the ADC ring into the AFSK demod. */
static void aprs_service(void)
{
    uint32_t base = (uint32_t)(uintptr_t)g_aprs_ring;
    uint32_t widx = ((dma_hw->ch[g_dma_chan].write_addr - base) / 2)
                    & (APRS_RING_SAMPLES - 1);
    while (g_aprs_rd != widx) {
        uint16_t smp = g_aprs_ring[g_aprs_rd] & 0x0FFF;
        aprs_rx_sample(&g_aprs, smp);
        g_aprs_rd = (g_aprs_rd + 1) & (APRS_RING_SAMPLES - 1);

#if CFG_APRS_RX_DIAG
        if (g_dump_st == DUMP_ARMED && aprs_rx_carrier(&g_aprs)) {
            g_dump_st = DUMP_RECORDING; g_dump_n = 0;
        }
        if (g_dump_st == DUMP_RECORDING) {
            g_dump[g_dump_n++] = smp;
            if (g_dump_n >= APRS_DUMP_SAMPLES ||
                (!aprs_rx_carrier(&g_aprs) && g_dump_n > 4000))
                g_dump_st = DUMP_READY;
        }
#endif
    }
}

#if CFG_APRS_RX_DIAG
/* dump the armed capture as [aprs.raw] lines (32 samples of 12-bit hex each) so
 * the host can rebuild the exact ADC stream and decode it with full logging. */
static void aprs_dump_flush(void)
{
    if (g_dump_st != DUMP_READY)
        return;
    LOG("[aprs.raw] begin n=%lu fs=%d\n", (unsigned long)g_dump_n,
        APRS_RX_SAMPLE_RATE_HZ);
    for (uint32_t i = 0; i < g_dump_n; i += 32) {
        char ln[32 * 3 + 1];
        int k = 0;
        for (uint32_t j = i; j < i + 32 && j < g_dump_n; j++)
            k += snprintf(ln + k, sizeof ln - k, "%03X", g_dump[j]);
        LOG("[aprs.raw] %s\n", ln);
    }
    LOG("[aprs.raw] end\n");
    g_dump_st = DUMP_IDLE;
}
#endif

int main(void)
{
    stdio_init_all();
    radio_uart_init();
    adc_capture_init();
#if CFG_GPS_ENABLE
    gps_uart_init();
#endif

    /* give a USB host a moment to attach so the banner isn't lost */
    for (int i = 0; i < 30 && !stdio_usb_connected(); i++) sleep_ms(100);

    LOG("\n=== SARSAT 406 MHz 1G decoder (RP2040) ===\n");
    LOG("board      : " PICO_BOARD "\n");
    LOG("audio in   : GP%d / ADC%d, %d Hz, %d samples/bit, window %d ms (%d smp)\n",
        CFG_ADC_GPIO, CFG_ADC_CHANNEL, CFG_SAMPLE_RATE_HZ,
        CFG_SAMPLE_RATE_HZ / 400, CFG_WINDOW_MS, WINDOW_SAMPLES);
    LOG("radio link : UART%d TX=GP%d RX=GP%d, %d 8N1\n",
        (CFG_RADIO_UART == uart0) ? 0 : 1,
        CFG_RADIO_UART_TX_GPIO, CFG_RADIO_UART_RX_GPIO, CFG_RADIO_UART_BAUD);
#if CFG_GPS_ENABLE
    LOG("gps        : UART%d RX=GP%d, %d 8N1 NMEA\n",
        (CFG_GPS_UART == uart0) ? 0 : 1, CFG_GPS_UART_RX_GPIO, CFG_GPS_BAUD);
#endif
    LOG("burst arm  : peak >= %d   rearm %d ms\n", CFG_BURST_PEAK_ON, CFG_REARM_MS);
    LOG("Tune the radio to the beacon frequency (406 or 434 MHz exercise), FM.\n");
    LOG("Press 'h' for the serial console (level meter for volume tuning).\n");
    LOG("At idle [lvl] dc must sit near %d (mid-scale). If dc<100 and bursts\n"
        "read adc[0..4095] the ADC has no bias -> add a divider on GP26.\n"
        "Then set the radio volume so a beacon gives [burst] peak ~%d-%d and\n"
        "adc stays inside ~150..3950 (clip%% = 0).\n\n",
        CFG_ADC_DC_CENTER, CFG_BURST_PEAK_ON, CFG_BURST_PEAK_ON * 3);

    absolute_time_t next_hello  = get_absolute_time();
    absolute_time_t next_lvl    = get_absolute_time();
    absolute_time_t next_link   = make_timeout_time_ms(13000);
#if CFG_GPS_ENABLE
    absolute_time_t next_gps    = make_timeout_time_ms(CFG_GPS_TX_MS);
#endif
    absolute_time_t rearm_at    = get_absolute_time();
    absolute_time_t last_decode = get_absolute_time();
    char            last_id[24] = {0};
    int             noise_rms   = 1200;   /* running estimate of the no-signal hiss */
    bool            noise_seeded = false;

    for (;;) {
        link_poll();

#if CFG_GPS_ENABLE
        gps_service();
        if (time_reached(next_gps)) {
            gps_push_to_radio();
            next_gps = make_timeout_time_ms(CFG_GPS_TX_MS);
        }
#endif

        if (g_mode_pending != g_mode) {
            if (g_mode_pending == MODE_APRS) aprs_mode_enter();
            else                             sarsat_mode_enter();
        }

        int64_t since_us = absolute_time_diff_us(g_last_reply, get_absolute_time());
        if (g_link_up == 1 && since_us > 12000000) {
            g_link_up = 0;
            LOG("[link]   link DOWN - no reply from radio for 12 s\n");
        }
        if (time_reached(next_link)) {
            LOG("[link]   %s  acks=%lu  last reply %lds ago  mode %s%s\n",
                g_link_up == 1 ? "UP" : "DOWN",
                (unsigned long)g_acks, (long)(since_us / 1000000),
                g_mode == MODE_APRS ? "APRS" : "SARSAT",
                g_link_up == 1 ? "" :
                "  -- check GP1 wiring + that the radio runs the Phase 2 firmware");
            next_link = make_timeout_time_ms(20000);
        }

        if (time_reached(next_hello)) {
            uint8_t v = SARSAT_LINK_PROTO_VER;
            radio_send(CMD_SARSAT_HELLO, &v, 1);
            next_hello = make_timeout_time_ms(5000);
        }

        /* -------- APRS mode: stream the ADC ring through the AFSK demod ---- */
        if (g_mode == MODE_APRS) {
            aprs_service();
            if (time_reached(next_lvl)) {
                uint32_t clip = aprs_rx_clip_permille(&g_aprs);
                int32_t cdt = 0, env = 0; int car = 0;
                aprs_rx_levels(&g_aprs, &cdt, &env, &car);
                LOG("[aprs]   %s  mod=%s  hdlc=%lu -> pkts=%lu (%lu fix) fcs_bad=%lu  "
                    "clip=%lu.%lu%%  cdt=%ld env=%ld%s\n",
                    car ? "CARRIER" : "idle", mod_name(g_radio_mod),
                    (unsigned long)g_aprs.n_hdlc,
                    (unsigned long)g_aprs.n_packets,
                    (unsigned long)g_aprs.n_fixed,
                    (unsigned long)g_aprs.n_fcs_bad,
                    (unsigned long)(clip / 10), (unsigned long)(clip % 10),
                    (long)cdt, (long)env,
                    clip > 20 ? "  <-- clipping, lower the radio AF gain" : "");
                next_lvl = make_timeout_time_ms(CFG_LEVEL_LOG_MS);
            }
#if CFG_APRS_RX_DIAG
            {
                int c = getchar_timeout_us(0);
                if (c == 'w' || c == 'W') {
                    g_dump_st = DUMP_ARMED;
                    LOG("[aprs.raw] armed - waiting for the next carrier\n");
                } else if (c == 'h' || c == 'H' || c == '?') {
                    LOG("[aprs]   'w' = capture the next burst's raw ADC "
                        "([aprs.raw] hex, decode off-line)\n");
                }
            }
            aprs_dump_flush();
#else
            { int c = getchar_timeout_us(0); (void)c; }   /* keep CDC drained */
#endif
            sleep_us(300);
            continue;
        }

        /* -------- SARSAT mode -------------------------------------------- */
        /* While the radio's level screen is open, capture short windows so the
         * bar refreshes fast (~7 Hz) for AF-gain tuning; a whole beacon burst
         * won't fit, so skip decoding until the user leaves that view. */
        const int wn = g_lvl_view ? (CFG_SAMPLE_RATE_HZ / 8) : WINDOW_SAMPLES;

        adc_capture_window(wn);

        audio_stats_t st;
        window_to_audio(&st, wn);

        console_poll(&st, noise_rms);
        if (g_meter)
            meter_print(&st, noise_rms, " ");

        link_poll();

        /* audio telemetry for the radio's level screen */
        radio_send_level(&st);
        link_poll();

        if (g_lvl_view)
            continue;

        bool armed = time_reached(rearm_at);

        if (!noise_seeded) {                 /* converge the floor fast on boot */
            if (st.rms > 200) noise_rms = st.rms;
            noise_seeded = true;
        }

        /*
         * Burst detection. Full FM hiss (no carrier) has a HIGH RMS; a 406 MHz
         * FGB burst CAPTURES the receiver and its window RMS collapses to well
         * below the tracked hiss floor (6-12 dB, x2..x4). Detect it *relative*
         * to that floor, so the absolute AF gain does not matter -- tuning the
         * radio level screen to the hiss is enough, the quieter beacon still
         * decodes and the gain need not be pushed hot (which would clip a 2 m
         * APRS packet on the shared audio tap). CFG_BURST_PEAK_ON stays as an
         * absolute fallback path.
         */
        long quiet_pct = (long)st.rms * 100 / (noise_rms > 1 ? noise_rms : 1);
        bool quieted   = quiet_pct < CFG_BURST_QUIET_PCT;
        bool is_burst  = st.clipped == 0 && st.rms <= CFG_DECODE_MAX_RMS &&
                         (quieted || st.peak >= CFG_BURST_PEAK_ON);

        if (!is_burst) {
            /* hiss / dead air: track the floor. Update on any window that is
             * not clearly quieter than the current estimate, so the floor
             * still rises to a hot hiss whose peaks are large. */
            if (!quieted)
                noise_rms += (st.rms - noise_rms) >> 3;
            if (time_reached(next_lvl)) {
                LOG("[lvl]    peak=%-5d rms=%-5d (%ld%% of floor) clip=%d%%  "
                    "adc[%d..%d] dc=%ld  floorRMS=%d  %s%s\n",
                    st.peak, st.rms, quiet_pct, st.clipped, st.raw_min, st.raw_max,
                    st.dc, noise_rms, armed ? "idle" : "rearm-wait",
                    (st.dc < 700 || st.dc > 3400) ? "  <-- ADC not mid-scale" :
                    st.clipped > 0               ? "  <-- clipping, lower AF gain" :
                    st.rms > CFG_DECODE_MAX_RMS  ? "  <-- hiss (no carrier)" : "");
                next_lvl = make_timeout_time_ms(CFG_LEVEL_LOG_MS);
            }
            continue;
        }

        if (!armed)
            continue;

#if CFG_AUDIO_AGC
        if (st.rms > 100) {
            int num = CFG_AGC_TARGET, den = st.rms;           /* normalise on RMS */
            for (int i = 0; i < WINDOW_SAMPLES; i++) {
                long v = (long)g_audio[i] * num / den;
                g_audio[i] = (v > 32767) ? 32767 : (v < -32768) ? -32768 : (int32_t)v;
            }
        }
#endif

        LOG("[burst]  peak=%d rms=%d (%ld%% of floor) clip=%d%% adc[%d..%d] dc=%ld "
            "floorRMS=%d -> decoding\n",
            st.peak, st.rms, quiet_pct, st.clipped, st.raw_min, st.raw_max, st.dc,
            noise_rms);

        sarsat_result_t r;
        int rc = sarsat_decode_window(g_audio, WINDOW_SAMPLES,
                                      CFG_SAMPLE_RATE_HZ, &r);

        if (rc == 0) {
            LOG("[slicer] no sync / no frame in this window\n");
            continue;
        }

        LOG("[slicer] %d bits: %s\n", r.frame_bits, r.raw_hex);

        if (rc < 0) {
            LOG("[decode] BCH uncorrectable - frame rejected "
                "(weak signal / mistune / off-band?)\n");
            rearm_at = make_timeout_time_ms(500);
            continue;
        }

        /* BCH corrects a bounded number of bit errors, but a heavily noisy
         * window (weak/marginal signal) can occasionally land close enough
         * to a degenerate codeword to pass as "BCH OK" while being pure
         * noise, not a real beacon -- seen on air as hexID "000000000000000"
         * / country 0 (Unknown) / lat,lon 0,0. No real COSPAS-SARSAT beacon
         * ID is all-zero (every real one carries a non-zero country/protocol
         * field), so treat this one shape of false positive as rejected
         * rather than pushing garbage to the radio screen. */
        {
            bool all_zero = true;
            for (const char *p = r.hex_id; *p; p++)
                if (*p != '0') { all_zero = false; break; }
            if (all_zero) {
                LOG("[decode] BCH OK but hexID all-zero -- false positive, "
                    "rejected\n");
                rearm_at = make_timeout_time_ms(500);
                continue;
            }
        }

        /* de-dup: the beacon re-transmits every ~50 s; keep the log terse and
         * still refresh the radio screen. */
        bool repeat = (strcmp(r.hex_id, last_id) == 0) &&
                      absolute_time_diff_us(last_decode, get_absolute_time())
                          < (int64_t)CFG_DEDUP_S * 1000000;
        snprintf(last_id, sizeof(last_id), "%s", r.hex_id);
        last_decode = get_absolute_time();

        if (repeat) {
            LOG("[decode] BCH OK  %s  %.5f %.5f  (repeat)\n",
                r.hex_id, r.info.lat, r.info.lon);
        } else {
            LOG("[decode] BCH OK\n");
            LOG("[decode] hexID=%s  country=%u (%s)  proto=%u  %s%s\n",
                r.hex_id, r.info.country_code,
                get_country_name(r.info.country_code), r.info.protocol_bits,
                r.info.is_test_message ? "TEST " : "",
                r.info.has_position ? "" : "(no position)");
            if (r.info.has_position && !r.info.position_default)
                LOG("[decode] lat=%.5f lon=%.5f  ident=\"%s\"\n",
                    r.info.lat, r.info.lon, r.info.vessel_id);
        }

        radio_push_result(&r);
        rearm_at = make_timeout_time_ms(CFG_REARM_MS);
    }
}
