/*
 * sarsat_decoder.h — hardware-independent glue: audio window -> sliced bits ->
 * T.001 parse -> formatted lines for the radio screen.
 *
 * No RP2040 headers here so the whole decode path builds and runs on a PC
 * (rp2040/test/host).
 */
#ifndef SARSAT_DECODER_H
#define SARSAT_DECODER_H

#include <stdint.h>
#include "dec406_v1g.h"

#define SARSAT_MAX_LINES 14    /* the radio screen scrolls; keep <= its buffer */
#define SARSAT_LINE_CHARS 19   /* 18 glyphs + NUL. The radio's UI_PrintStringSmall
                                * overruns its row buffer past 18 glyphs at the
                                * Start=2 used here, so the wrap width is 18. */

typedef struct {
    int            valid;        /* 1 = a CRC-clean frame was decoded */
    int            frame_bits;   /* slicer output: 0 (no frame), 112 or 144 */
    int            crc_ok;       /* BCH result once frame_bits != 0 */
    char           raw_hex[40];  /* sliced frame as hex (sync included), or "" */
    char           hex_id[24];
    BeaconInfo1G   info;
    char           lines[SARSAT_MAX_LINES][SARSAT_LINE_CHARS];
    int            n_lines;
} sarsat_result_t;

/* Scan one FM-demodulated audio window for a 1G beacon burst. *out is always
 * filled (frame_bits / crc_ok / raw_hex tell you how far it got).
 * Returns:  1 = CRC-clean frame decoded (lines[] ready to send)
 *           0 = slicer found no frame
 *          -1 = frame sliced but BCH uncorrectable */
int sarsat_decode_window(const int32_t *samples, int n, int rate,
                         sarsat_result_t *out);

/* Build the human-readable lines for *out from out->info (called internally by
 * sarsat_decode_window; exposed for tests). */
void sarsat_format_lines(sarsat_result_t *out);

#endif /* SARSAT_DECODER_H */
