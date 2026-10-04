/* test_sstv_demod -- synthesises a valid Scottie 1 VIS header + a few scan
 * lines (known pixel values) as an audio waveform, feeds it through
 * sstv_demod_sample(), and checks the decoded VIS lock and pixel values
 * match. Proves the state machine's timing/logic on a clean synthetic
 * signal -- NOT a substitute for real-air validation (see sstv_demod.h's
 * own warning about the zero-crossing frequency estimator's margin). */
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdlib.h>
#include <string.h>

#include "sstv_demod.h"

static int fails;

static void chk(const char *name, int cond)
{
    printf("  %-46s %s\n", name, cond ? "OK" : "FAIL");
    if (!cond) fails++;
}

#define FS 48000

static double g_phase;
static int32_t *g_samples;
static int      g_n;
static int      g_cap;

static void push_tone(double freq_hz, double duration_s)
{
    int n = (int)(duration_s * FS + 0.5);
    double step = 2.0 * M_PI * freq_hz / FS;
    for (int i = 0; i < n; i++) {
        if (g_n >= g_cap) {
            g_cap = g_cap ? g_cap * 2 : 4096;
            g_samples = realloc(g_samples, sizeof(int32_t) * (size_t)g_cap);
        }
        g_samples[g_n++] = (int32_t)(8000.0 * sin(g_phase));
        g_phase += step;
        if (g_phase > 1e6) g_phase -= 1e6;   /* keep it bounded, harmless wrap */
    }
}

static double luma_to_hz(uint8_t v)
{
    return 1500.0 + (double)v * 800.0 / 255.0;
}

static void push_vis(uint8_t code)
{
    push_tone(1900, 0.300);
    push_tone(1200, 0.010);
    push_tone(1900, 0.300);
    push_tone(1200, 0.030);   /* start bit -- see sstv_demod.c's ST_VIS_START */
    int ones = 0;
    for (int i = 0; i < 7; i++) {
        int bit = (code >> i) & 1;
        if (bit) ones++;
        push_tone(bit ? 1100 : 1300, 0.030);
    }
    int parity = ones & 1;
    push_tone(parity ? 1100 : 1300, 0.030);
    push_tone(1200, 0.030);
}

/* One Scottie 1 line, each channel filled with a constant test value. */
static void push_line(bool first_line, uint8_t g, uint8_t b, uint8_t r)
{
    if (first_line)
        push_tone(1200, 0.009);
    push_tone(1500, 0.0015);
    for (int x = 0; x < SSTV_SCOTTIE1_WIDTH; x++) push_tone(luma_to_hz(g), 0.000432);
    push_tone(1500, 0.0015);
    for (int x = 0; x < SSTV_SCOTTIE1_WIDTH; x++) push_tone(luma_to_hz(b), 0.000432);
    push_tone(1200, 0.009);
    push_tone(1500, 0.0015);
    for (int x = 0; x < SSTV_SCOTTIE1_WIDTH; x++) push_tone(luma_to_hz(r), 0.000432);
}

int main(void)
{
    push_vis(60);   /* Scottie 1 */
    push_line(true,  20, 128, 240);
    push_line(false, 0,  255, 64);

    sstv_demod_t d;
    sstv_demod_init(&d, FS);

    int lines_seen = 0;
    uint8_t seen_g[2], seen_b[2], seen_r[2];
    for (int i = 0; i < g_n; i++) {
        if (sstv_demod_sample(&d, g_samples[i])) {
            if (lines_seen < 2) {
                /* sample mid-line pixels to dodge any single-pixel edge
                 * jitter right at a channel boundary */
                seen_g[lines_seen] = d.frame[SSTV_CH_GREEN][160];
                seen_b[lines_seen] = d.frame[SSTV_CH_BLUE][160];
                seen_r[lines_seen] = d.frame[SSTV_CH_RED][160];
            }
            printf("  -- line ready: idx=%d\n", d.line_ready_idx);
            lines_seen++;
        }
    }

    chk("both lines decoded", lines_seen == 2);
    if (lines_seen >= 2) {
        chk("line0 green ~20",  abs((int)seen_g[0] - 20)  <= 8);
        chk("line0 blue ~128",  abs((int)seen_b[0] - 128) <= 8);
        chk("line0 red ~240",   abs((int)seen_r[0] - 240) <= 8);
        chk("line1 green ~0",   abs((int)seen_g[1] - 0)   <= 8);
        chk("line1 blue ~255",  abs((int)seen_b[1] - 255) <= 8);
        chk("line1 red ~64",    abs((int)seen_r[1] - 64)  <= 8);
    }

    printf(fails ? "\nFAIL (%d)\n" : "\nall pass\n", fails);
    return fails ? 1 : 0;
}
