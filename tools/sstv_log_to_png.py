#!/usr/bin/env python3
"""sstv_log_to_png.py -- rebuild a PNG from a [sstv.line] console capture
(branch SSTV_SSDV's Scottie 1 RX, rp2040/src/sstv_demod.c).

On the RP2040 console (USB-CDC serial), while in SSTV RX mode (radio-side
"SSTVRx" menu action armed, see patch/sstv_rx.h), each fully-decoded scan
line is logged as:

    [sstv.line] n=0 ch=G off=0 <64 hex chars>          (32 px, 1 byte/px)
    [sstv.line] n=0 ch=G off=32 <64 hex chars>
    ...                                                (10 chunks: 320 px)
    [sstv.line] n=0 ch=B off=0 ...
    [sstv.line] n=0 ch=R off=0 ...
    [sstv.line] n=1 ch=G off=0 ...
    ...

Paste the whole console log (everything else is ignored) into a text file,
then:

    python3 tools/sstv_log_to_png.py capture.txt out.png

Missing lines/chunks (a real capture will rarely have exactly 256 complete
lines, especially near the start/end of a ~110 s transmission) are left
black -- this is a raw, uncorrected dump of whatever the demod decided, not
a validated image; see sstv_demod.h's own caveats about real-air accuracy.
"""
import re
import sys

from PIL import Image

WIDTH = 320
HEIGHT = 256

LINE_RE = re.compile(
    r"\[sstv\.line\]\s+n=(\d+)\s+ch=([GBR])\s+off=(\d+)\s+([0-9A-Fa-f]+)"
)


def load(path):
    # frame[y][x] = [R, G, B]
    frame = [[[0, 0, 0] for _ in range(WIDTH)] for _ in range(HEIGHT)]
    chan_index = {"R": 0, "G": 1, "B": 2}
    n_chunks = 0
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = LINE_RE.search(line)
            if not m:
                continue
            n, ch, off, hexstr = m.groups()
            n = int(n)
            off = int(off)
            if n >= HEIGHT:
                continue
            data = bytes.fromhex(hexstr)
            ci = chan_index[ch]
            for i, v in enumerate(data):
                x = off + i
                if x < WIDTH:
                    frame[n][x][ci] = v
            n_chunks += 1
    return frame, n_chunks


def main():
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} capture.txt out.png", file=sys.stderr)
        return 1
    frame, n_chunks = load(sys.argv[1])
    print(f"{n_chunks} chunks parsed", file=sys.stderr)

    img = Image.new("RGB", (WIDTH, HEIGHT))
    px = img.load()
    for y in range(HEIGHT):
        for x in range(WIDTH):
            r, g, b = frame[y][x]
            px[x, y] = (r, g, b)
    img.save(sys.argv[2])
    print(f"wrote {sys.argv[2]}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
