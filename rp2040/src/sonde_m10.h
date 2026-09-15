/*
 * sonde_m10.h -- Meteomodem M10/M20 radiosonde frame layer, Sarsat_UV-K1-5_RP2040.
 *
 * Ported from the reference decoders `demod/mod/m10mod.c` (M10) and
 * `demod/mod/m20mod.c` (M20) in github.com/projecthorus/radiosonde_auto_rx
 * (GPL-3.0, author zilog80). This project relicensed from Apache-2.0 to
 * GPL-3.0 on 2026-09-11 specifically to allow this port (explicit user
 * decision) -- see CREDITS.md.
 *
 * ⚠️ MODEM CORRECTED (2026-09-11, second pass): the first port of this file
 * treated the 9600/9615 baud chip stream as already being final data bits
 * (differential-decode only). On-air testing against a known position
 * (49.65968 N, 3.31387 E) found NOTHING at any offset/scale/endianness --
 * an exhaustive negative result across 58 real captures. Re-reading
 * HTCommander's independent Dart port (Ylianst/HTCommander,
 * src/lib/radiosonde/m10_demodulator.dart -- also GPL-3.0-derived per its
 * own header, "ported from radiosonde_auto_rx", so read here under the
 * same relicense) revealed the real pipeline: M10/M20 is Manchester-coded
 * (2 chips/bit) with a DIFFERENTIAL layer OVER the Manchester bits, not
 * instead of it. Corrected pipeline, RF to bytes:
 *   chips (9600/9615 Bd) -> Manchester decode (sonde_manchester_decode(),
 *   the same primitive sonde_dfm.c uses) -> differential/NRZI decode
 *   (sonde_m10_diff_decode(), formula unchanged, the earlier code had this
 *   part right) -> MSB-first byte packing.
 * This project's own 32-bit CHIP-level header correlator (M10_RAWHEADER_BITS,
 * confirmed bit-exact on 21/21 real bursts) was already correct even before
 * this fix: the reference's `rawheader[]` is itself expressed at the chip
 * level (pre-Manchester), matching what this project independently found.
 *
 * The M20 field layout (offsets/scale) was already right (cross-checked
 * against HTCommander's M20Decoder, identical numbers) -- it just never had
 * a chance to see correctly-decoded bytes before now. This pass also adds
 * the M20 custom checksum (ported from M20Decoder._checkM10/_update) as the
 * real frame-validity gate, replacing the weaker "is it in a plausible
 * lat/lon range" heuristic from the first port.
 */
/*
 * ⚠️ DEMODULATOR REPLACED (2026-09-11, third pass): even after the Manchester
 * fix above, a fresh real recording (32 kHz, ~1.7 samples per HALF-chip at
 * 9600 baud -- much tighter than RS41/DFM's oversampling) showed 65-75%
 * Manchester violations post-header -- far worse than noise alone would
 * explain. HTCommander's demodulator (m10_demodulator.dart) does not slice
 * one point per chip like sonde_demod.c's dead-band PLL; it INTEGRATES each
 * half-chip period and compares the two half-averages (a matched-filter/
 * integrate-and-dump approach), which is far more noise-robust at a tight
 * samples-per-chip ratio. sonde_m10_chipdemod_t below adds that on top of
 * this project's existing PLL phase-tracking (kept as-is for clock
 * recovery -- only the final chip DECISION changes from point-sampling to
 * half-cell integration). RS41/DFM/the old 4800-baud M10 header-detector
 * keep using the plain sonde_demod_t (sonde_demod.h) unchanged -- only the
 * new 9600-baud M10/M20 GPS chain uses this.
 *
 * ⚠️ CORRECTED AGAIN (2026-09-11, fourth pass): the first cut of this
 * demodulator compared the two HALVES of one chip cell against each other.
 * test_chipdemod_synthetic() (added this pass) caught that this doesn't
 * converge -- a single chip's amplitude is constant for its whole duration
 * (the Manchester transition is BETWEEN chips, i.e. at the bit level, see
 * m10_demodulator.dart's `_headerChips()`), so there is nothing genuinely
 * different between a chip's two halves beyond noise. Replaced with a
 * WHOLE-cell integrate-and-dump: sum the DC-corrected raw signal across the
 * entire chip cell and slice on its sign -- this is what actually delivers
 * the noise-averaging benefit integrate-and-dump is for. See
 * sonde_m10_chipdemod_t's struct comment below for the full account.
 */
#ifndef SONDE_M10_H
#define SONDE_M10_H

#include <stdbool.h>
#include <stdint.h>

/* Integrate-and-dump chip demodulator -- see the header note above. Reuses
 * this project's proven PLL phase-tracking (sonde_demod.c's dead-band
 * slicer + nudge-on-transition, unchanged) purely for clock recovery; the
 * chip VALUE itself comes from integrating the (DC-corrected) raw signal
 * across the WHOLE chip cell and comparing the sign of that sum, not a
 * single point sample.
 *
 * ⚠️ Note this is deliberately NOT a half-cell comparison: a single M10/M20
 * chip carries a CONSTANT amplitude for its entire duration (see
 * m10_demodulator.dart's `_headerChips()`: each encoded chip is one fixed
 * level for the full chip period; the Manchester transition happens BETWEEN
 * two adjacent chips, i.e. at the BIT level, not inside one chip). An
 * earlier version of this function split each chip cell in half and
 * compared the two halves against EACH OTHER -- that only makes sense for a
 * cell that itself contains a mid-cell transition, which a single chip does
 * not; it was caught by test_chipdemod_synthetic() (14-34% error rate
 * depending on the exact wrap-detection edge used) and replaced with this
 * whole-cell design (<1% error rate on the same synthetic vector). The
 * actual half/half comparison HTCommander performs in `_sliceFrame()` is
 * BIT-level (comparing the two whole CHIPS that make up one Manchester
 * bit), which this project still gets for free downstream: the chips this
 * function emits are Manchester-decoded afterwards by the existing
 * sonde_manchester_decode() (sonde_dfm.h), unchanged. */
typedef struct {
    int32_t  lp, lp_hi, lp_lo, env;   /* dead-band slicer state, for the PLL
                                       * nudge only -- see sonde_demod.c */
    uint32_t pll, pll_step;
    int      bit, pval;
    int64_t  cell_sum;                /* running sum of DC-corrected raw
                                       * samples across the current chip cell */
    int32_t  cell_n;
} sonde_m10_chipdemod_t;

void sonde_m10_chipdemod_init(sonde_m10_chipdemod_t *d, int sample_rate_hz, int chip_baud);

/* Feed one discriminator sample (same convention as sonde_demod_sample()).
 * Returns true when a chip-cell boundary was crossed. `*chip_out` (never
 * NULL) gets the DC-baseline 0/1 decision -- used ONLY for header
 * correlation (sonde_m10_hunt_feed()'s hunting phase), unchanged from the
 * third pass. `*cell_avg_out` (may be NULL) gets the raw per-sample average
 * of this chip cell, DC-UNCORRECTED -- feed a captured run of these into
 * sonde_m10_bits_from_cells() for the payload decode; see its header note
 * on why the payload must NOT go through the DC-baseline 0/1 decision. */
bool sonde_m10_chipdemod_sample(sonde_m10_chipdemod_t *d, int32_t sample,
                                uint8_t *chip_out, int32_t *cell_avg_out);

/* Manchester-decode `n_cells` raw chip-cell averages (from
 * sonde_m10_chipdemod_sample()'s cell_avg_out) into n_cells/2 bits, by
 * comparing each pair of adjacent cells DIRECTLY against each other
 * (`bit = cells[2i+1] > cells[2i]`, optionally inverted) instead of each
 * cell against a separately-tracked DC baseline.
 *
 * ⚠️ DEMODULATOR REPLACED AGAIN (2026-09-11, fifth pass): the whole-cell
 * integrate-and-dump from the third pass (sonde_m10_chipdemod_t's DC-
 * baseline chip decision) was a real improvement over point-sampling, but
 * still wasn't enough: on-air testing (both real hardware captures and a
 * fresh 44 s WAV recording) kept showing the payload degrade into ~50-90%
 * Manchester violations while the header kept locking cleanly. Root cause:
 * this project's dead-band slicer tracks a DC baseline (lp_hi/lp_lo) with
 * its OWN separate time constant, and comparing each chip's integral
 * against that separately-evolving baseline accumulates whatever tracking
 * error the baseline itself picks up over a long (~175 ms) capture. HTCommander's
 * actual reference (`m10_demodulator.dart`'s `_sliceFrame()`) never
 * compares a chip to a baseline at all -- it integrates the TWO CHIPS OF
 * ONE BIT and compares them DIRECTLY to each other (`c2 - c1`), which is
 * self-referential and cancels out any slow baseline drift by construction
 * (both chips see essentially the same local baseline error, so it drops
 * out of the subtraction). Re-reading that comparison and switching to it
 * confirmed the fix immediately: replayed against real captured audio, the
 * decoded M10 type marker (0x64 0x9F) now matches EXACTLY, and the decoded
 * latitude matches the user's ground-truth calibration position
 * (49.65968 N) to 4 decimal places, reproducibly across independent bursts
 * recorded 16 s apart -- the first exact, non-coincidental field match this
 * project has ever achieved for M10/M20. sonde_m10_hunt_t now stores raw
 * per-chip averages (`raw_cells`, int32) during capture instead of decided
 * chips, and this function -- not sonde_manchester_decode_lenient() -- does
 * the M10/M20 payload's Manchester decode; sonde_manchester_decode()/
 * _lenient() stay exactly as they were for DFM and for M10/M20's header
 * correlation (still chip-level, still proven reliable on its own).
 *
 * Always writes exactly n_cells/2 bits (there is no "violation" concept
 * here -- every pair of real levels compares to something, unlike a
 * chip-level Manchester pair that can be invalid). */
int sonde_m10_bits_from_cells(const int32_t *cells, int n_cells,
                              uint8_t *bits_out, bool invert);

/* 32-bit fixed correlation header, expressed at the CHIP level (see the
 * header comment above) -- common to M10 and M20, used to locate the frame
 * before the per-type "Sonde-Header" byte that follows it (0x64 0x9F for
 * M10, 0x45 0x20 for M20 -- checked post-decode in sonde_m20_parse_gps()). */
#define M10_HEADER_CHIPS 32
extern const uint8_t M10_RAWHEADER_BITS[M10_HEADER_CHIPS];   /* 0/1 per byte */

/* Differentially decode `n` bits (already Manchester-decoded -- see
 * sonde_manchester_decode() in sonde_dfm.h) right after the header match:
 * out[i] = NOT(raw[i] XOR raw[i-1]), out[0] = NOT(raw[0]). Unchanged from
 * the first port -- this part was already correct. */
void sonde_m10_diff_decode(const uint8_t *raw_bits, int n, uint8_t *out_bits);

/* M20 GPS field layout (m20mod.c / HTCommander's M20Decoder, cross-checked
 * identical): int32 big-endian, lat/lon scaled by 1e6, alt (3 bytes, cm)
 * /100 -> m. Offsets are relative to `frame_bytes[0]`, which should equal
 * 0x45 (M20's frame-length self-marker byte) once decoding is correct --
 * see sonde_m20_checksum(). */
typedef struct {
    bool has_position;
    int32_t lat_e5, lon_e5;
    int32_t alt_m;
} m10_gps_t;

#define M20_FRAME_BYTES  70   /* M20Decoder.frameLen: frame[0]=0x45=69 payload
                               * bytes, +1 for the indexing HTCommander uses */
#define M20_POS_CHECK    (0x45 - 1)   /* 2-byte checksum position           */

/* M20's custom linear checksum (ported from M20Decoder._checkM10/_update) --
 * the real frame-validity gate: computed over `len` bytes starting at
 * `frame_bytes[0]`, compared against the 2 bytes at frame_bytes[len..len+1]
 * (M20_POS_CHECK is the usual `len`). */
uint16_t sonde_m20_checksum(const uint8_t *frame_bytes, int len);

/* `frame_bytes` must hold at least M20_FRAME_BYTES bytes. Validates
 * frame_bytes[0]==0x45, frame_bytes[1]==0x20 (M20's type marker) and the
 * checksum before trusting the GPS fields -- has_position is only set once
 * all three pass, a much stronger gate than the first port's bare lat/lon
 * range check. Returns false if `len` is too short to hold a full frame. */
bool sonde_m20_parse_gps(const uint8_t *frame_bytes, int len, m10_gps_t *out);

/* M10 GPS field layout (m10mod.c / HTCommander's M10Decoder, cached from the
 * same source as M20's above -- see sonde_dfm.h's provenance note). M10's
 * frame is a 101-byte Trimble Copernicus GPS packet wrapper (M20's 70-byte
 * frame is a different, shorter layout entirely -- confirmed necessary on
 * real air, 2026-09-11: the user's actual sonde is an M10, not M20, which
 * this project had been decoding against the wrong field layout the whole
 * time). Same checksum algorithm as M20 (sonde_m20_checksum() is generic
 * over `len`, reused here as-is), different position/scale: lat/lon are
 * int32 big-endian BAM units (raw / (2^32/360) -> degrees), altitude int32
 * big-endian millimetres (raw / 1000 -> metres). */
#define M10_FRAME_BYTES  101   /* M10Decoder.frameLen */
#define M10_TYPE0        0x64  /* M10Decoder.stdFLen -- frame[0] */
#define M10_TYPE1        0x9F  /* M10Decoder.typeM10 -- frame[1] */
#define M10_POS_CHECK    (M10_TYPE0 - 1)   /* 0x63 -- 2-byte checksum position */
#define M10_POS_LAT      0x0E
#define M10_POS_LON      0x12
#define M10_POS_ALT      0x16

/* `frame_bytes` must hold at least M10_FRAME_BYTES bytes. Same has_position
 * gating philosophy as sonde_m20_parse_gps() (type marker + checksum before
 * trusting the fields). Returns false if `len` is too short. */
bool sonde_m10_parse_gps(const uint8_t *frame_bytes, int len, m10_gps_t *out);

/* ---- streaming sync hunter (9600/9615 baud CHIP stream -> m10_gps_t) --- */
/* Mirrors sonde_sync.c's RS41 hunter design (bit-level, not byte-aligned --
 * see sonde_sync.h), but for M10/M20's 32-chip header + Manchester +
 * differential decode instead of RS41's 64-bit whitened one. A SEPARATE
 * sonde_demod_t instance at 9600 baud feeds this one CHIP at a time (RS41/
 * the old M10-detection-only path stay on their own 4800 baud chain, see
 * main.c). */
#define M10_CAPTURE_CHIPS 1680   /* -> 840 bits -> 105 B after Manchester --
                                 * covers the full M10_FRAME_BYTES (101, the
                                 * longer of the two real frame types) with a
                                 * few bytes of margin for a weak/noisy real
                                 * capture. Was 1200 (75 B, enough for M20's
                                 * 70 B only) until real on-air testing
                                 * (2026-09-11) identified the actual sonde
                                 * as an M10, whose frame doesn't fit in that
                                 * window at all. */

typedef struct {
    uint32_t sr;                        /* 32-bit rolling raw-CHIP shift register */
    uint32_t pattern;                   /* precomputed M10_RAWHEADER_BITS value */
    bool     locked;
    int32_t  raw_cells[M10_CAPTURE_CHIPS]; /* raw chip-cell averages, post-header,
                                            * pre-Manchester -- see
                                            * sonde_m10_bits_from_cells() */
    int      raw_count;
    uint8_t  frame_bytes[M10_CAPTURE_CHIPS / 16];  /* last capture, fully
                                                    * decoded -- valid after
                                                    * any call that returns
                                                    * true; hex-dumping this
                                                    * is the fastest way to
                                                    * cross-check an on-air
                                                    * capture (see the
                                                    * radiosonde plan notes) */
} sonde_m10_hunt_t;

void sonde_m10_hunt_init(sonde_m10_hunt_t *h);

/* Feed one chip-cell result from sonde_m10_chipdemod_sample() -- `chip` (its
 * `chip_out`) drives header correlation while hunting, `cell_avg` (its
 * `cell_avg_out`) is what actually gets captured and decoded once locked
 * (see sonde_m10_bits_from_cells()'s header note on why the payload can't
 * go through the same DC-baseline decision the header correlator uses).
 * Returns true once a full capture window has been Manchester- and
 * differentially-decoded (into `h->frame_bytes`) and parsed (M10 layout
 * tried first, M20 as fallback, both checksum-validated) into `*out` --
 * check `out->has_position` regardless (a header chip-match on noise still
 * returns true with has_position false once the checksum rejects it).
 * Resets to hunting either way. */
bool sonde_m10_hunt_feed(sonde_m10_hunt_t *h, uint8_t chip, int32_t cell_avg, m10_gps_t *out);

#endif /* SONDE_M10_H */
