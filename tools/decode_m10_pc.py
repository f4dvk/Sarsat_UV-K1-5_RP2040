#!/usr/bin/env python3
"""
decode_m10_pc.py -- Standalone M10/M20 radiosonde decoder for a PC sound
card (USB mic/line input), part of the Sarsat_UV-K1-5_RP2040 project.

Demodulator architecture ported from HTCommander's own M10 decoder
(github.com/Ylianst/HTCommander, src/lib/radiosonde/m10_demodulator.dart,
Apache-2.0), on explicit user request after that project's on-air
success prompted a closer look. Earlier versions of this script used a
live, chip-by-chip PLL (a port of this project's own RP2040 firmware
demodulator, rp2040/src/sonde_m10.c) -- this version instead:
  - buffers a whole window of audio,
  - cross-correlates a matched filter (built from the sync bytes' own
    Manchester+differential chip pattern) against the ENTIRE window at a
    nominal samples-per-chip ratio, to find candidate frame starts,
  - REFINES that samples-per-chip ratio from the actual spacing measured
    between several detected frames, instead of trusting the nominal
    constant for the whole frame,
  - then slices the full frame using the refined rate.
This self-calibrates against whatever the true chip rate actually is
(sound card clock, sonde oscillator tolerance...) rather than assuming a
fixed 9600/9615 throughout -- a live PLL's dead-band nudge only corrects
phase a little at a time and can accumulate error over a long frame,
which is exactly the "decode confidence drops toward the end of the
frame" pattern this project chased at length with the old architecture.
The calculated chip rate is printed for every decoded/attempted frame
(see "Debit" below) so a real, on-air mismatch is directly visible.

The frame parsers (parse_m10/parse_m20, GPS field decode, checksum) are
unchanged and were already confirmed bit-exact against HTCommander's own
m10_decoder.dart in an earlier pass -- only the demodulator changed here.

Two modes:
  - Live, from a sound card (default): decode continuously as audio comes
    in. Needs `pip install sounddevice numpy`.
  - Offline, from a WAV file (--wav): decode a file already recorded (by
    this script, Audacity, arecord, etc.) at your leisure, no real-time
    constraint. Needs only `numpy` (uses the stdlib `wave` module to read).

Usage:
    python3 decode_m10_pc.py --list                  # list input devices
    python3 decode_m10_pc.py --device 2               # live, device #2
    python3 decode_m10_pc.py                          # live, default input
    python3 decode_m10_pc.py --wav capture.wav        # offline, from a file
    python3 decode_m10_pc.py --wav capture.wav --dump-raw   # + raw-sample dumps
    python3 decode_m10_pc.py --gain 2.0               # boost a too-quiet input

The radio's line/speaker output should feed the sound card's MIC or LINE
IN (a mic input usually has too much gain for a speaker-level signal --
turn the radio's own volume down, or use a line/aux input if the sound
card has one, to avoid clipping). De-emphasis on the radio and a WIDE FM
filter, same as this project's own Sonde screen profile, give the cleanest
results but are not required to get a look at the signal.

Level: a peak meter (crete, in %/dBFS, against the 16-bit full scale the
audio is normalized to either way) prints roughly once per second of
audio, in both live and --wav mode, so the level can be checked without
any external tool -- watch for "!! ECRETAGE" (peak pinned at/near full
scale, a strong sign of real clipping). --gain applies a digital
multiplier AFTER that measurement (so the meter always reports the true
input level, not the boosted one) -- useful for a too-quiet input, but it
cannot undo real clipping: a signal already flattened at full scale by
the sound card or the radio's own volume stays flattened, just louder.
Fix clipping by lowering the radio's volume or the sound card's own input
gain, not with a --gain below 1.0 here.

Frame layout: M10 (101 B, marker 0x64 0x9F, Trimble/BAM GPS packet,
chip rate 9615 baud) tried alongside M20 (70 B, marker 0x45 0x20, plain
GPS packet, chip rate 9600 baud) on every window -- mirroring
HTCommander's own radiosonde_monitor.dart, which runs one instance of
its demodulator per sonde type with its own chip rate rather than one
shared assumption. This project's real sonde is an M10.
"""

import argparse
import sys
import wave
from collections import deque

try:
    import numpy as np
except ImportError:
    sys.exit("This script needs numpy: pip install numpy")


# ---------------------------------------------------------------------------
# Matched-filter demodulator -- ported from HTCommander's M10Demodulator
# (m10_demodulator.dart). One instance per sonde type (M10, M20): same modem
# (Manchester + differential), different chip rate/frame length/sync bytes.
# ---------------------------------------------------------------------------
def _header_chips(sync0: int, sync1: int):
    """+1/-1 template for the sync bytes' own on-air chip pattern: MSB-first
    bits -> inverse of the differential decode (so the template is what the
    RAW, pre-differential-decode chip stream looks like for these bytes) ->
    Manchester (raw 1 => [-1,+1], raw 0 => [+1,-1], matching how
    _slice_frame() below scores a bit as sign(pol*(c2-c1)))."""
    bits = []
    for byte in (sync0, sync1):
        for k in range(7, -1, -1):
            bits.append((byte >> k) & 1)
    prev = 0
    chips = []
    for out_bit in bits:
        r = prev if out_bit == 1 else (1 - prev)
        prev = r
        if r == 1:
            chips += [-1.0, 1.0]
        else:
            chips += [1.0, -1.0]
    return chips


def bits_to_bytes_msb(bits):
    nbytes = len(bits) // 8
    out = bytearray(nbytes)
    for i in range(nbytes):
        v = 0
        for k in range(8):
            v = (v << 1) | (bits[8 * i + k] & 1)
        out[i] = v
    return bytes(out)


def _quarters(diffs, ndigits=0):
    """Same diagnostic idea as before: average |c2-c1| bit confidence, one
    figure per quarter of the frame. A real signal keeps roughly the same
    magnitude throughout; a figure that drops hard in later quarters is the
    signature of the demodulator's rate estimate drifting off across the
    frame -- directly relevant now that the rate is a single value refined
    once per window, not continuously re-tracked like a live PLL would."""
    n = len(diffs)
    if n == 0:
        return []
    q = max(1, n // 4)
    return [round(sum(diffs[i:i + q]) / len(diffs[i:i + q]), ndigits)
            for i in range(0, n, q)][:4]


class MatchedFilterDemod:
    def __init__(self, name: str, chip_rate: float, frame_bytes: int,
                 sync0: int, sync1: int):
        self.name = name
        self.chip_rate = chip_rate
        self.frame_bytes = frame_bytes
        self.sync0 = sync0
        self.sync1 = sync1
        self.frame_bits = frame_bytes * 8
        self.frame_chips = self.frame_bits * 2
        self.header = np.asarray(_header_chips(sync0, sync1), dtype=np.float64)

    def demodulate(self, pcm: np.ndarray, sample_rate_hz: int):
        """Returns a list of result dicts for every sync match found in this
        window (frame content, GPS parse happens in the caller)."""
        n = len(pcm)
        nominal_sps = sample_rate_hz / self.chip_rate
        hlen = len(self.header)
        if n < int(np.ceil(self.frame_chips * nominal_sps)) + 1:
            return []

        x = pcm.astype(np.float64)
        x -= x.mean()
        std = float(x.std())
        if std <= 0:
            return []

        prefix = np.empty(n + 1, dtype=np.float64)
        prefix[0] = 0.0
        np.cumsum(x, out=prefix[1:])

        sps = nominal_sps
        hdr_span = int(np.ceil(hlen * sps))
        last_offset = n - hdr_span - 1
        if last_offset <= 0:
            return []

        # Cross-correlate the sync template against every possible start
        # offset at once (vectorized boxcar-average-per-chip via the prefix
        # sum, one pass per header chip -- hlen is only 64, so this is
        # ~64 vector ops over `last_offset` elements, not a Python-level
        # loop over every sample).
        offs = np.arange(last_offset, dtype=np.float64)
        score = np.zeros(last_offset, dtype=np.float64)
        for j in range(hlen):
            s_idx = np.round(offs + j * sps).astype(np.int64)
            e_idx = np.round(offs + (j + 1) * sps).astype(np.int64)
            score += self.header[j] * (prefix[e_idx] - prefix[s_idx])
        score *= 1.0 / (hlen * sps * std)

        thr = 0.35
        min_gap = max(1, int(round(self.frame_chips * sps * 0.5)))
        window = max(1, int(round(sps * 2)))
        abs_score = np.abs(score)
        candidates = np.where(abs_score >= thr)[0]

        peaks, peak_pol = [], []
        for i in candidates:
            lo = max(0, i - window)
            hi = min(last_offset - 1, i + window)
            if np.any(abs_score[lo:hi + 1] > abs_score[i]):
                continue  # not a local max in its own neighbourhood
            if peaks and (i - peaks[-1]) < min_gap:
                if abs_score[i] > abs_score[peaks[-1]]:
                    peaks[-1] = int(i)
                    peak_pol[-1] = 1 if score[i] >= 0 else -1
                continue
            peaks.append(int(i))
            peak_pol.append(1 if score[i] >= 0 else -1)
        if not peaks:
            return []

        # Refine samples-per-chip from real inter-peak spacing (median of
        # gaps close to one frame length) -- this is the self-calibration
        # step: whatever the true chip rate is on THIS recording, on THIS
        # sound card, comes out of the data itself instead of trusting the
        # nominal chip_rate constant for the whole frame.
        refined_sps = sps
        target = self.frame_chips * sps
        gaps = [peaks[i] - peaks[i - 1] for i in range(1, len(peaks))
                if target * 0.8 < peaks[i] - peaks[i - 1] < target * 1.2]
        if gaps:
            gaps.sort()
            refined_sps = gaps[len(gaps) // 2] / self.frame_chips

        results = []
        for p0, pol in zip(peaks, peak_pol):
            r = self._slice_frame(prefix, n, p0, refined_sps, pol)
            if r is None:
                continue
            r["demod"] = self.name
            r["score"] = float(abs_score[p0])
            r["nominal_sps"] = sps
            r["refined_sps"] = refined_sps
            r["chip_rate_calculated_hz"] = sample_rate_hz / refined_sps
            r["sample_pos"] = p0
            results.append(r)
        return results

    def _slice_frame(self, prefix, n, p0, sps, pol):
        raw = [0] * self.frame_bits
        diffs_mag = []
        for i in range(self.frame_bits):
            s1 = int(round(p0 + (2 * i) * sps))
            m = int(round(p0 + (2 * i + 1) * sps))
            e = int(round(p0 + (2 * i + 2) * sps))
            if s1 < 0 or e > n or m <= s1 or e <= m:
                return None
            c1 = (prefix[m] - prefix[s1]) / (m - s1)
            c2 = (prefix[e] - prefix[m]) / (e - m)
            diffs_mag.append(abs(c2 - c1))
            raw[i] = 1 if (pol * (c2 - c1)) >= 0 else 0

        # Differential decode (invariant to overall chip-stream inversion,
        # since `pol` already picked the correlation's own sign).
        out = [0] * self.frame_bits
        prev = 0
        for i in range(self.frame_bits):
            out[i] = 1 if raw[i] == prev else 0
            prev = raw[i]

        frame = bits_to_bytes_msb(out)
        if frame[0] != self.sync0 or frame[1] != self.sync1:
            return None
        span_end = int(round(p0 + self.frame_chips * sps))
        return {
            "frame": frame,
            "sample_span": (max(0, int(round(p0))), min(n, span_end)),
            "quarter_confidence": _quarters(diffs_mag),
        }


# ---------------------------------------------------------------------------
# M20's custom 16-bit shift-register checksum (m20_checksum_update() /
# sonde_m20_checksum() in the RP2040 C source) -- shared, generic over
# length, used by both M10 and M20 frame validation. Confirmed bit-exact
# against HTCommander's own M10Decoder._update() in m10_decoder.dart.
# ---------------------------------------------------------------------------
def m20_checksum_update(c: int, b: int) -> int:
    c1 = c & 0xFF
    b = ((b >> 1) | ((b & 1) << 7)) & 0xFF
    b = b ^ ((b >> 2) & 0xFF)
    t6 = (c & 1) ^ ((c >> 2) & 1) ^ ((c >> 4) & 1)
    t7 = ((c >> 1) & 1) ^ ((c >> 3) & 1) ^ ((c >> 5) & 1)
    t = (c & 0x3F) | (t6 << 6) | (t7 << 7)
    s = (c >> 7) & 0xFF
    s = s ^ ((s >> 2) & 0xFF)
    c0 = (b ^ t ^ s) & 0xFF
    return ((c1 << 8) | c0) & 0xFFFF


def m20_checksum(frame_bytes: bytes, length: int) -> int:
    c = 0
    for i in range(length):
        c = m20_checksum_update(c, frame_bytes[i])
    return c


def be_i32(p: bytes, off: int) -> int:
    v = (p[off] << 24) | (p[off + 1] << 16) | (p[off + 2] << 8) | p[off + 3]
    return v - 0x100000000 if v & 0x80000000 else v


# M20: 70-byte frame, marker 0x45 0x20, plain int32 BE lat/lon *1e6, 3-byte alt/100
M20_FRAME_BYTES = 70
M20_POS_CHECK = 0x45 - 1
POS_GPSLAT_M20 = 0x1C
POS_GPSLON_M20 = 0x20
POS_GPSALT_M20 = 0x08


def parse_m20(frame_bytes: bytes):
    out = {"has_position": False, "lat": 0.0, "lon": 0.0, "alt_m": 0}
    if len(frame_bytes) < M20_FRAME_BYTES:
        return out
    if frame_bytes[0] != 0x45 or frame_bytes[1] != 0x20:
        return out
    want = (frame_bytes[M20_POS_CHECK] << 8) | frame_bytes[M20_POS_CHECK + 1]
    if m20_checksum(frame_bytes, M20_POS_CHECK) != want:
        return out
    lat = be_i32(frame_bytes, POS_GPSLAT_M20) / 1e6
    lon = be_i32(frame_bytes, POS_GPSLON_M20) / 1e6
    alt_raw = (frame_bytes[POS_GPSALT_M20] << 16) | \
        (frame_bytes[POS_GPSALT_M20 + 1] << 8) | frame_bytes[POS_GPSALT_M20 + 2]
    out["lat"], out["lon"], out["alt_m"] = lat, lon, alt_raw // 100
    out["has_position"] = (not (lat == 0.0 and lon == 0.0) and
                            -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0 and
                            -1000 <= out["alt_m"] <= 80000)
    return out


# M10: 101-byte frame, marker 0x64 0x9F, Trimble/BAM int32 BE lat/lon, mm alt
M10_FRAME_BYTES = 101
M10_POS_CHECK = 0x64 - 1
M10_POS_LAT = 0x0E
M10_POS_LON = 0x12
M10_POS_ALT = 0x16
M10_BAM_SCALE = 1073741824.0 / 90.0  # 2^30/90 == 2^32/360, matches HTCommander's _b60b60


def parse_m10(frame_bytes: bytes):
    out = {"has_position": False, "lat": 0.0, "lon": 0.0, "alt_m": 0}
    if len(frame_bytes) < M10_FRAME_BYTES:
        return out
    if frame_bytes[0] != 0x64 or frame_bytes[1] != 0x9F:
        return out
    want = (frame_bytes[M10_POS_CHECK] << 8) | frame_bytes[M10_POS_CHECK + 1]
    if m20_checksum(frame_bytes, M10_POS_CHECK) != want:
        return out
    lat = be_i32(frame_bytes, M10_POS_LAT) / M10_BAM_SCALE
    lon = be_i32(frame_bytes, M10_POS_LON) / M10_BAM_SCALE
    alt = be_i32(frame_bytes, M10_POS_ALT) / 1000.0
    out["lat"], out["lon"], out["alt_m"] = lat, lon, int(alt)
    out["has_position"] = (not (lat == 0.0 and lon == 0.0) and
                            -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0 and
                            -1000 <= out["alt_m"] <= 80000)
    return out


# ---------------------------------------------------------------------------
# Diagnostic: flag a suspiciously long run of bit-exact identical RAW audio
# samples -- the flat-zero dropout signature this project has been chasing
# on the RP2040 side. Checked directly on the raw PCM now (not on decimated
# "cells" like the old streaming version), so it's a strictly more direct
# read of the actual audio.
# ---------------------------------------------------------------------------
def find_flat_runs(pcm_slice, min_run: int):
    runs = []
    i = 0
    n = len(pcm_slice)
    while i < n:
        j = i
        while j < n and pcm_slice[j] == pcm_slice[i]:
            j += 1
        if j - i >= min_run:
            runs.append((i, j - i, int(pcm_slice[i])))
        i = j
    return runs


def report(result, full_pcm, sample_rate_hz: int, dump_raw: bool):
    frame = result["frame"]
    gps = result["gps"]
    span0, span1 = result["sample_span"]
    print(f"[m10] demod={result['demod']} frame:", frame.hex())
    print(f"[m10] correlation a l'en-tete: {result['score']:.2f} "
          f"(seuil 0.35 -- plus haut = verrouillage plus net)")
    bit_rate = result["chip_rate_calculated_hz"] / 2.0
    nominal_hz = sample_rate_hz / result["nominal_sps"]
    drift_pct = 100.0 * (result["chip_rate_calculated_hz"] - nominal_hz) / nominal_hz
    print(f"[m10] debit calcule: {result['chip_rate_calculated_hz']:.1f} chips/s "
          f"(nominal {nominal_hz:.1f} chips/s, ecart {drift_pct:+.2f}%), "
          f"soit ~{bit_rate:.0f} bit/s d'info apres Manchester")
    print(f"[m10] confiance par quart (diff. moyenne des paires, plus haut = mieux): "
          f"{result['quarter_confidence']}")
    if gps["has_position"]:
        print(f"[m10] *** POSITION *** lat={gps['lat']:.5f} lon={gps['lon']:.5f} "
              f"alt={gps['alt_m']} m")
    else:
        print("[m10] no valid position (checksum/marker mismatch)")
    pcm_slice = full_pcm[span0:span1]
    min_run = max(10, int(round((sample_rate_hz / result["chip_rate_calculated_hz"]) * 3)))
    runs = find_flat_runs(pcm_slice, min_run)
    if runs:
        for (start, length, val) in runs:
            dur_ms = 1000.0 * length / sample_rate_hz
            print(f"[m10] !! flat run: {length} echantillons bruts (~{dur_ms:.0f} ms) "
                  f"tous == {val} -- la coupure que ce projet chasse")
    if dump_raw:
        print("[m10] rawsamples:", ",".join(str(int(v)) for v in pcm_slice))
    print()


# ---------------------------------------------------------------------------
# Level meter: peak amplitude against the 16-bit full scale every block of
# samples is normalized to (both the WAV and live paths land on the same
# int16-equivalent range before this, gain included -- see LevelMeter.feed()'s
# call site). Reports roughly once per second of AUDIO time (a sample count,
# not a wall-clock timer), so a --wav replay reports at the same cadence
# a live capture would, however fast the file is actually processed.
# ---------------------------------------------------------------------------
FULL_SCALE = 32767


class LevelMeter:
    def __init__(self):
        self.window_n = 0      # samples seen since the last report
        self.window_peak = 0   # peak |sample| in this window, pre-gain
        self.clipped = False

    def feed(self, sample_rate_hz: int, raw_block, gain: float):
        """raw_block: pre-gain samples (what the sound card / WAV actually
        produced) -- clipping is judged on THIS, not on a --gain-boosted
        copy, so the meter always reflects the real input level."""
        if len(raw_block) == 0:
            return
        block_peak = int(np.max(np.abs(raw_block)))
        self.window_peak = max(self.window_peak, block_peak)
        if block_peak >= FULL_SCALE - 4:   # a few LSB of slack for float rounding
            self.clipped = True
        self.window_n += len(raw_block)
        if self.window_n < sample_rate_hz:
            return
        pct = 100.0 * self.window_peak / FULL_SCALE
        dbfs = 20.0 * np.log10(max(self.window_peak, 1) / FULL_SCALE)
        gain_note = f", gain={gain:g}x -> {min(100.0, pct * gain):.0f}%" if gain != 1.0 else ""
        warn = "  !! ECRETAGE (baisser le volume radio ou le gain carte son)" if self.clipped else ""
        print(f"[m10] niveau: crete {self.window_peak}/{FULL_SCALE} "
              f"({pct:.0f}%, {dbfs:.1f} dBFS){gain_note}{warn}")
        self.window_n = 0
        self.window_peak = 0
        self.clipped = False


# ---------------------------------------------------------------------------
# Sample source: a WAV file, or a live sound card via `sounddevice`.
# ---------------------------------------------------------------------------
def iter_wav_samples(path: str, channel: int):
    with wave.open(path, "rb") as w:
        rate = w.getframerate()
        sampwidth = w.getsampwidth()
        nchan = w.getnchannels()
        print(f"[m10] {path}: {rate} Hz, {sampwidth * 8}-bit, {nchan} ch")
        if sampwidth != 2:
            sys.exit("Only 16-bit WAV is supported (record at 16-bit).")
        raw = w.readframes(w.getnframes())
    samples = np.frombuffer(raw, dtype="<i2").astype(np.int64)
    if nchan > 1:
        samples = samples.reshape(-1, nchan)[:, min(channel, nchan - 1)]
    return rate, samples


def iter_live_samples(device, rate: int, channel: int):
    try:
        import sounddevice as sd
    except ImportError:
        sys.exit("Live capture needs: pip install sounddevice")

    q = deque()

    def callback(indata, frames, time_info, status):
        if status:
            print(f"[m10] audio status: {status}", file=sys.stderr)
        col = indata[:, min(channel, indata.shape[1] - 1)]
        q.append((col * 32767.0).astype(np.int64))

    stream = sd.InputStream(device=device, channels=1 if device is None else None,
                             samplerate=rate, callback=callback, dtype="float32")
    print(f"[m10] listening on device={device!r} @ {rate} Hz -- Ctrl+C to stop")
    with stream:
        try:
            while True:
                if q:
                    yield q.popleft()
                else:
                    sd.sleep(20)
        except KeyboardInterrupt:
            print("\n[m10] stopped")


def list_devices():
    try:
        import sounddevice as sd
    except ImportError:
        sys.exit("pip install sounddevice to list devices")
    print(sd.query_devices())


# Window/overlap for the batch matched-filter correlation. A window must
# comfortably span one whole frame (M10's is the longer one, ~168 ms of
# chips plus a variable preamble) with margin; the step is smaller than the
# window so consecutive windows overlap and a frame landing near a window
# boundary is never missed, at the cost of seeing it once in each
# overlapping window -- SEEN_ABS_TOLERANCE below is what deduplicates that.
WINDOW_SECONDS = 1.5
STEP_SECONDS = 0.9
SEEN_ABS_TOLERANCE_SECONDS = 0.4  # less than the ~1 Hz burst repetition rate


class Deduper:
    """Tracks the absolute (whole-stream) sample position of the last
    reported frame per demodulator name, so the same on-air burst seen
    again in an overlapping window is not printed twice."""

    def __init__(self):
        self.last_abs = {}

    def is_new(self, name: str, abs_pos: float, sample_rate_hz: int) -> bool:
        prev = self.last_abs.get(name)
        tol = SEEN_ABS_TOLERANCE_SECONDS * sample_rate_hz
        if prev is not None and abs(abs_pos - prev) < tol:
            return False
        self.last_abs[name] = abs_pos
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list audio input devices and exit")
    ap.add_argument("--device", default=None, help="input device index or name substring")
    ap.add_argument("--rate", type=int, default=48000, help="sample rate Hz (live mode; default 48000)")
    ap.add_argument("--channel", type=int, default=0, help="channel index for stereo input (default 0)")
    ap.add_argument("--wav", default=None, help="decode a 16-bit WAV file instead of live audio")
    ap.add_argument("--dump-raw", action="store_true",
                     help="print the raw sample list for every capture (like the RP2040's own diagnostic)")
    ap.add_argument("--gain", type=float, default=1.0,
                     help="digital gain multiplier applied after the level meter reads the raw input "
                          "(default 1.0 = none) -- boosts a too-quiet signal, cannot undo real clipping")
    args = ap.parse_args()

    if args.list:
        list_devices()
        return

    demods = [
        MatchedFilterDemod("M10", chip_rate=9615, frame_bytes=M10_FRAME_BYTES, sync0=0x64, sync1=0x9F),
        MatchedFilterDemod("M20", chip_rate=9600, frame_bytes=M20_FRAME_BYTES, sync0=0x45, sync1=0x20),
    ]
    meter = LevelMeter()
    dedup = Deduper()
    n_ok = 0
    n_bad = 0

    def process_window(sample_rate_hz, window_pcm, window_abs_start):
        nonlocal n_ok, n_bad
        for demod in demods:
            for r in demod.demodulate(window_pcm, sample_rate_hz):
                abs_pos = window_abs_start + r["sample_pos"]
                if not dedup.is_new(demod.name, abs_pos, sample_rate_hz):
                    continue
                gps = parse_m10(r["frame"]) if demod.name == "M10" else parse_m20(r["frame"])
                if not gps["has_position"]:
                    # a matched frame under one demod can still validate under
                    # the other type's checksum (mirrors the old fallback)
                    other = parse_m20(r["frame"]) if demod.name == "M10" else parse_m10(r["frame"])
                    if other["has_position"]:
                        gps = other
                r["gps"] = gps
                if gps["has_position"]:
                    n_ok += 1
                else:
                    n_bad += 1
                report(r, window_pcm, sample_rate_hz, args.dump_raw)
                print(f"[m10] totals: ok={n_ok} bad={n_bad}\n")

    if args.wav:
        rate, samples = iter_wav_samples(args.wav, args.channel)
        meter.feed(rate, samples, args.gain)
        block = samples.astype(np.float64) * args.gain if args.gain != 1.0 else samples
        step = int(STEP_SECONDS * rate)
        window = int(WINDOW_SECONDS * rate)
        pos = 0
        while pos < len(block):
            end = min(len(block), pos + window)
            process_window(rate, block[pos:end], pos)
            if end >= len(block):
                break
            pos += step
        print(f"[m10] done: ok={n_ok} bad={n_bad}")
    else:
        device = args.device
        if device is not None:
            try:
                device = int(device)
            except ValueError:
                pass  # let sounddevice resolve a name substring
        buf = np.zeros(0, dtype=np.float64)
        buf_abs_start = 0   # absolute stream position of buf[0]
        window = int(WINDOW_SECONDS * args.rate)
        step = int(STEP_SECONDS * args.rate)
        next_process_len = window
        for chunk in iter_live_samples(device, args.rate, args.channel):
            meter.feed(args.rate, chunk, args.gain)
            chunk = chunk.astype(np.float64) * args.gain if args.gain != 1.0 else chunk.astype(np.float64)
            buf = np.concatenate([buf, chunk])
            while len(buf) >= next_process_len:
                process_window(args.rate, buf[:window] if len(buf) >= window else buf, buf_abs_start)
                if len(buf) > step:
                    buf = buf[step:]
                    buf_abs_start += step
                else:
                    buf_abs_start += len(buf)
                    buf = np.zeros(0, dtype=np.float64)
                next_process_len = window
        print(f"[m10] done: ok={n_ok} bad={n_bad}")


if __name__ == "__main__":
    main()
