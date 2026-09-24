#!/usr/bin/env python3
"""
imgfsk_log_to_ssdv.py -- bench-test helper for the SSTV_SSDV raw-FSK image
link (branch SSTV_SSDV), part of the Sarsat_UV-K1-5_RP2040 project.

The RP2040 (rp2040/src/main.c, fed by imgfsk_sync.c) dumps every captured
256-byte SSDV packet as a plain-text hex line on its USB serial diagnostic
port, e.g.:

    [imgfsk2400] pkt n=1 55660100000500000...   (512 hex chars = 256 bytes)

This script reads that log (from a serial port live, or a file already
captured with a terminal program) and writes out the matching binary
packets concatenated into a single .ssdv file, ready for the reference
decoder: `ssdv -d out.ssdv out.jpg` (github.com/fsphil/ssdv).

Usage:
    python3 imgfsk_log_to_ssdv.py --port /dev/ttyACM0 --baud 115200 -o out.ssdv
    python3 imgfsk_log_to_ssdv.py --file capture.log -o out.ssdv

Live mode needs `pip install pyserial`; offline (--file) mode needs
nothing beyond the standard library.
"""
import argparse
import re
import sys

LINE_RE = re.compile(r"\[imgfsk(1200|2400)\]\s+pkt\s+n=(\d+)\s+([0-9A-Fa-f]+)")


def handle_line(line, out, seen_counts):
    m = LINE_RE.search(line)
    if not m:
        return
    rate, n, hexdata = m.group(1), int(m.group(2)), m.group(3)
    if len(hexdata) != 512:
        print(f"-- skipped {rate} baud pkt n={n}: {len(hexdata)} hex chars, expected 512",
              file=sys.stderr)
        return
    seen_counts[rate] = seen_counts.get(rate, 0) + 1
    out.write(bytes.fromhex(hexdata))
    print(f"pkt n={n} ({rate} baud) -> {seen_counts[rate]} packets written so far")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device, e.g. /dev/ttyACM0 (live mode)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--file", help="read from an already-captured log file instead")
    ap.add_argument("-o", "--out", required=True, help="output .ssdv file")
    args = ap.parse_args()

    if not args.port and not args.file:
        ap.error("need either --port (live) or --file (offline)")

    seen_counts = {}
    with open(args.out, "wb") as out:
        if args.file:
            with open(args.file, "r", errors="replace") as f:
                for line in f:
                    handle_line(line, out, seen_counts)
        else:
            import serial   # only imported for live mode
            with serial.Serial(args.port, args.baud, timeout=1) as ser:
                print(f"listening on {args.port} @ {args.baud} -- Ctrl+C to stop")
                try:
                    while True:
                        line = ser.readline().decode("ascii", errors="replace")
                        if line:
                            handle_line(line, out, seen_counts)
                except KeyboardInterrupt:
                    pass

    total = sum(seen_counts.values())
    print(f"\n{total} packet(s) written to {args.out} ({dict(seen_counts)})")
    print(f"Decode with: ssdv -d {args.out} out.jpg")


if __name__ == "__main__":
    main()
