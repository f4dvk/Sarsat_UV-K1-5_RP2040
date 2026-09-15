#!/usr/bin/env python3
"""
decode_m10_pc.py -- Standalone M10/M20 radiosonde decoder for a PC sound
card (USB mic/line input), part of the Sarsat_UV-K1-5_RP2040 project.

This is a line-for-line Python port of the project's own RP2040 firmware
decoder (rp2040/src/sonde_m10.c / sonde_m10.h, GPL-3.0, ported from
projecthorus/radiosonde_auto_rx -- see that file's header for the full
provenance/porting history). It exists to answer one specific diagnostic
question this session couldn't resolve from the RP2040 side alone: does a
brief, precisely-repeatable flat-zero dropout inside real M10/M20 captures
come from the RP2040/C-Board, or is it already present in the radio's own
analog audio output? Recording that SAME audio into a PC sound card and
running the identical algorithm here, completely independently of the
RP2040's ADC/DMA/firmware, is a clean way to tell the two apart: if this
script sees the same dropout, it's in the radio (or upstream); if it
doesn't, the RP2040/C-Board is implicated after all.

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
    python3 decode_m10_pc.py --wav capture.wav --dump-raw   # + raw-cell dumps

The radio's line/speaker output should feed the sound card's MIC or LINE
IN (a mic input usually has too much gain for a speaker-level signal --
turn the radio's own volume down, or use a line/aux input if the sound
card has one, to avoid clipping). De-emphasis on the radio and a WIDE FM
filter, same as this project's own Sonde screen profile, give the cleanest
results but are not required to get a look at the signal.

Frame layout ported: M10 (101 B, marker 0x64 0x9F, Trimble/BAM GPS packet)
tried first, M20 (70 B, marker 0x45 0x20, plain GPS packet) as a fallback
-- exactly the RP2040 hunt's own order (this project's real sonde is an
M10; M20 support is kept for anyone else running this against one).
"""

import argparse
import struct
import sys
import wave
from collections import deque

try:
    import numpy as np
except ImportError:
    sys.exit("This script needs numpy: pip install numpy")


# ---------------------------------------------------------------------------
# Chip demodulator -- port of sonde_m10_chipdemod_t / sonde_m10_chipdemod_sample()
# (rp2040/src/sonde_m10.c). PLL-based clock recovery (dead-band slicer, used
# ONLY to find chip-cell boundaries) + whole-cell integrate-and-dump for the
# actual chip value. See that file's header comments for why: a single M10/
# M20 chip carries constant amplitude for its whole duration (the Manchester
# transition is BETWEEN chips), so summing the raw signal across the whole
# cell is what rejects noise -- a half-cell or point-sample comparison is not
# equivalent and was tried and rejected during this project's own development.
# ---------------------------------------------------------------------------
M10_SLICE_MIN = 8  # same floor as the C code


class ChipDemod:
    __slots__ = ("lp", "lp_hi", "lp_lo", "env", "pll", "pll_step", "bit",
                 "pval", "cell_sum", "cell_n")

    def __init__(self, sample_rate_hz: int, chip_baud: int = 9600):
        self.lp = 0
        self.lp_hi = 0
        self.lp_lo = 0
        self.env = 0
        self.pll = 0
        # (1<<32) * chip_baud / sample_rate_hz, as an unsigned 32-bit step
        self.pll_step = ((1 << 32) * chip_baud) // sample_rate_hz
        self.bit = 0
        self.pval = 0
        self.cell_sum = 0
        self.cell_n = 0

    def feed(self, sample: int):
        """Returns (emitted, chip_out, cell_avg). chip_out/cell_avg are only
        meaningful when emitted is True (mirrors the C function's bool
        return + out-params)."""
        # dead-band slicer -- PLL nudge only, not the chip decision
        self.lp += (sample - self.lp) >> 2
        if self.lp > self.lp_hi:
            self.lp_hi += (self.lp - self.lp_hi) >> 2
        else:
            self.lp_hi -= (self.lp_hi - self.lp_lo) >> 10
        if self.lp < self.lp_lo:
            self.lp_lo += (self.lp - self.lp_lo) >> 2
        else:
            self.lp_lo += (self.lp_hi - self.lp_lo) >> 10
        lpc = self.lp - ((self.lp_hi + self.lp_lo) >> 1)
        mag = -lpc if lpc < 0 else lpc
        self.env += (mag - self.env) >> 6
        th = self.env >> 2
        if th < M10_SLICE_MIN:
            th = M10_SLICE_MIN
        if lpc > th:
            self.bit = 0
        elif lpc < -th:
            self.bit = 1

        # integrate-and-dump: whole-cell sum, NOT DC-corrected (see the C
        # header note -- the payload's own adjacent-cell comparison cancels
        # DC bias by construction; correcting here would just add the
        # dead-band slicer's own drifting baseline error to both sides).
        self.cell_sum += sample
        self.cell_n += 1

        emitted = False
        chip_out = 0
        cell_avg = 0
        prev_sign = _sign32(self.pll)
        self.pll = (self.pll + self.pll_step) & 0xFFFFFFFF
        if _sign32(self.pll) > prev_sign:
            avg = self.cell_sum // self.cell_n if self.cell_n else 0
            chip_out = 0 if avg >= (self.lp_hi + self.lp_lo) // 2 else 1
            cell_avg = avg
            self.cell_sum = 0
            self.cell_n = 0
            emitted = True

        if self.bit != self.pval:
            self.pll = (self.pll - (_as_i32(self.pll) >> 2)) & 0xFFFFFFFF
            self.pval = self.bit

        return emitted, chip_out, cell_avg


def _as_i32(v: int) -> int:
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v & 0x80000000 else v


def _sign32(v: int) -> int:
    """int32_t >> 31 in C: -1 if the sign bit is set, 0 otherwise."""
    return -1 if (v & 0x80000000) else 0


# ---------------------------------------------------------------------------
# Manchester-via-adjacent-cells, differential decode, byte packing, header
# pattern -- straight ports of the same-named C functions.
# ---------------------------------------------------------------------------
M10_HEADER_CHIPS = 32
M10_RAWHEADER_BITS = [
    1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1,
    0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1,
]
M10_HEADER_PATTERN = 0
for _b in M10_RAWHEADER_BITS:
    M10_HEADER_PATTERN = (M10_HEADER_PATTERN << 1) | _b
M10_MAX_HDR_ERR = 2  # out of 32 chips


def bits_from_cells(cells, invert: bool):
    n_pairs = len(cells) // 2
    out = [0] * n_pairs
    for i in range(n_pairs):
        bit = 1 if cells[2 * i + 1] > cells[2 * i] else 0
        out[i] = bit ^ 1 if invert else bit
    return out


def diff_decode(raw_bits):
    out = [0] * len(raw_bits)
    prev_raw = 0
    for i, r in enumerate(raw_bits):
        r &= 1
        out[i] = (1 - r) if i == 0 else (1 - (r ^ prev_raw))
        prev_raw = r
    return out


def bits_to_bytes_msb(bits):
    nbytes = len(bits) // 8
    out = bytearray(nbytes)
    for i in range(nbytes):
        v = 0
        for k in range(8):
            v = (v << 1) | (bits[8 * i + k] & 1)
        out[i] = v
    return bytes(out)


# ---------------------------------------------------------------------------
# M20's custom 16-bit shift-register checksum (m20_checksum_update() /
# sonde_m20_checksum() in the C source) -- shared, generic over length, used
# by both M10 and M20 frame validation.
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
M10_BAM_SCALE = 1073741824.0 / 90.0  # 2^32/360


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
# Streaming hunter -- port of sonde_m10_hunt_t / sonde_m10_hunt_feed(). Same
# re-anchor-on-every-fresh-header-match behaviour as the firmware (M10/M20's
# preamble repeats the 32-chip pattern several times before the real
# sync-to-data boundary; re-checking on every chip, even while capturing,
# converges on the LAST repeat instead of freezing on the first).
# ---------------------------------------------------------------------------
M10_CAPTURE_CHIPS = 1680  # 105 B after Manchester -- covers the 101 B M10 frame


class Hunt:
    def __init__(self):
        self.sr = 0
        self.locked = False
        self.raw_cells = []

    def feed(self, chip: int, cell_avg: int):
        """Returns a result dict once a full window has been decoded, else None."""
        self.sr = ((self.sr << 1) | (chip & 1)) & 0xFFFFFFFF
        fresh_match = bin(self.sr ^ M10_HEADER_PATTERN).count("1") <= M10_MAX_HDR_ERR

        if not self.locked:
            if fresh_match:
                self.locked = True
                self.raw_cells = []
            return None

        if fresh_match:
            self.raw_cells = []  # re-anchor on the latest preamble repeat
            return None

        self.raw_cells.append(cell_avg)
        if len(self.raw_cells) < M10_CAPTURE_CHIPS:
            return None

        self.locked = False
        self.sr = 0
        cells = self.raw_cells
        self.raw_cells = []

        bits_a = bits_from_cells(cells, False)
        bits_b = bits_from_cells(cells, True)
        n = len(bits_a)
        n8 = n - (n % 8)

        frame_a = bits_to_bytes_msb(diff_decode(bits_a)[:n8])
        result = {"frame": frame_a, "gps": parse_m10(frame_a), "cells": cells}
        if not result["gps"]["has_position"]:
            g = parse_m20(frame_a)
            if g["has_position"]:
                result["gps"] = g

        if not result["gps"]["has_position"]:
            frame_b = bits_to_bytes_msb(diff_decode(bits_b)[:n8])
            g = parse_m10(frame_b)
            if not g["has_position"]:
                g = parse_m20(frame_b)
            if g["has_position"]:
                result["frame"] = frame_b
                result["gps"] = g

        return result


# ---------------------------------------------------------------------------
# Diagnostic: flag a suspiciously long run of bit-exact identical raw cell
# values -- this is exactly the signature this project has been chasing on
# the RP2040 side (a real signal, even fading, keeps some noise; a perfectly
# flat run for dozens+ of cells is not organic). Reported alongside every
# capture (valid or not) so a run here can be directly compared against the
# RP2040's own [sonde] rawcells dumps.
# ---------------------------------------------------------------------------
def find_flat_runs(cells, min_run=20):
    runs = []
    i = 0
    n = len(cells)
    while i < n:
        j = i
        while j < n and cells[j] == cells[i]:
            j += 1
        if j - i >= min_run:
            runs.append((i, j - i, cells[i]))
        i = j
    return runs


def report(result, dump_raw: bool):
    frame = result["frame"]
    gps = result["gps"]
    print("[m10] frame:", frame.hex())
    if gps["has_position"]:
        print(f"[m10] *** POSITION *** lat={gps['lat']:.5f} lon={gps['lon']:.5f} "
              f"alt={gps['alt_m']} m")
    else:
        print("[m10] no valid position (checksum/marker mismatch)")
    runs = find_flat_runs(result["cells"])
    if runs:
        for (start, length, val) in runs:
            print(f"[m10] !! flat run: {length} cells (bytes {start // 16}.."
                  f"{(start + length) // 16}) all == {val} -- "
                  f"the dropout signature this project is chasing")
    if dump_raw:
        print("[m10] rawcells:", ",".join(str(c) for c in result["cells"]))
    print()


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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list audio input devices and exit")
    ap.add_argument("--device", default=None, help="input device index or name substring")
    ap.add_argument("--rate", type=int, default=48000, help="sample rate Hz (live mode; default 48000)")
    ap.add_argument("--channel", type=int, default=0, help="channel index for stereo input (default 0)")
    ap.add_argument("--wav", default=None, help="decode a 16-bit WAV file instead of live audio")
    ap.add_argument("--dump-raw", action="store_true",
                     help="print the raw cell-average list for every capture (like the RP2040's own diagnostic)")
    args = ap.parse_args()

    if args.list:
        list_devices()
        return

    demod = None
    hunt = Hunt()
    n_ok = 0
    n_bad = 0

    def feed_block(sample_rate_hz, block):
        nonlocal demod, n_ok, n_bad
        if demod is None:
            demod = ChipDemod(sample_rate_hz, 9600)
        for s in block:
            emitted, chip_out, cell_avg = demod.feed(int(s))
            if not emitted:
                continue
            result = hunt.feed(chip_out, cell_avg)
            if result is not None:
                if result["gps"]["has_position"]:
                    n_ok += 1
                else:
                    n_bad += 1
                report(result, args.dump_raw)
                print(f"[m10] totals: ok={n_ok} bad={n_bad}\n")

    if args.wav:
        rate, samples = iter_wav_samples(args.wav, args.channel)
        # process in chunks so very long files don't need one giant array copy
        chunk = 1 << 16
        for i in range(0, len(samples), chunk):
            feed_block(rate, samples[i:i + chunk])
        print(f"[m10] done: ok={n_ok} bad={n_bad}")
    else:
        device = args.device
        if device is not None:
            try:
                device = int(device)
            except ValueError:
                pass  # let sounddevice resolve a name substring
        for block in iter_live_samples(device, args.rate, args.channel):
            feed_block(args.rate, block)
        print(f"[m10] done: ok={n_ok} bad={n_bad}")


if __name__ == "__main__":
    main()
