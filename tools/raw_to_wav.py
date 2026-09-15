#!/usr/bin/env python3
"""raw_to_wav.py -- convert an [aprs.raw]/[sonde.raw] console capture into a
.wav file, for off-line analysis of a REAL on-target capture (as opposed to
a separate PC sound-card recording).

On the RP2040 console (USB-CDC serial), while in APRS mode press 'w', or
while in SONDE mode press 'y', to arm a one-shot raw-ADC capture. It prints
something like:

    [sonde.raw] begin n=10000 fs=32000
    [sonde.raw] 7FE7FD8021FA...          (32 samples, 3 hex digits = 12-bit each)
    ...
    [sonde.raw] end

Paste that whole capture (or a whole console log -- everything else is
ignored) into a text file, then:

    python3 tools/raw_to_wav.py capture.txt capture.wav

The 12-bit ADC samples (0..4095, ~2048 = mid-rail) are written out as a
mono 16-bit PCM WAV, DC-centered ((v-2048) << 4) the same way main.c's own
`sample = ((int32_t)raw - 2048) << CFG_AUDIO_GAIN_SHIFT` scales them before
the demodulators see them, so the WAV plays/analyzes at a directly
comparable amplitude to what those decoders actually process.
"""
import re
import struct
import sys

TAG_RE = re.compile(r"\[(aprs|sonde)\.raw\]\s*(.*)")


def load(path):
    fs = None
    samples = []
    in_body = False
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = TAG_RE.search(line)
            if not m:
                continue
            body = m.group(2).strip()
            if body.startswith("begin"):
                fsm = re.search(r"fs=(\d+)", body)
                fs = int(fsm.group(1)) if fsm else None
                in_body = True
                continue
            if body.startswith("end"):
                in_body = False
                continue
            if in_body:
                hexpart = body.split()[0] if body.split() else ""
                for i in range(0, len(hexpart) - 2, 3):
                    samples.append(int(hexpart[i:i + 3], 16))
    return fs, samples


def write_wav(path, fs, samples):
    pcm = bytearray()
    for v in samples:
        v16 = (v - 2048) << 4
        if v16 > 32767:
            v16 = 32767
        if v16 < -32768:
            v16 = -32768
        pcm += struct.pack("<h", v16)

    data_len = len(pcm)
    with open(path, "wb") as f:
        f.write(b"RIFF")
        f.write(struct.pack("<I", 36 + data_len))
        f.write(b"WAVEfmt ")
        f.write(struct.pack("<IHHIIHH", 16, 1, 1, fs, fs * 2, 2, 16))
        f.write(b"data")
        f.write(struct.pack("<I", data_len))
        f.write(pcm)


def main():
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} capture.txt out.wav", file=sys.stderr)
        sys.exit(1)
    fs, samples = load(sys.argv[1])
    if fs is None or not samples:
        print("no [aprs.raw]/[sonde.raw] capture found in that file", file=sys.stderr)
        sys.exit(1)
    write_wav(sys.argv[2], fs, samples)
    print(f"wrote {sys.argv[2]}: {len(samples)} samples @ {fs} Hz "
          f"({len(samples) / fs:.3f} s)")


if __name__ == "__main__":
    main()
