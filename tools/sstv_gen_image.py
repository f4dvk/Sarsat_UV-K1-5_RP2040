#!/usr/bin/env python3
"""sstv_gen_image.py -- build an rp2040/src/sstv_master_image*.h header (the
image the radio streams for SSTV_Send{Scottie1,Martin1,PD90,PD120}, see
rp2040/src/main.c's sstv_stream_*() and firmware/uv-k1-k5v3/patch/sstv_tx.c)
from any source image (PNG/JPG/...), via Pillow.

Fits the source into the target WxH canvas (default: centered, aspect-ratio
preserved, padded with white -- good for a logo/graphic with flat background;
use --fit cover to crop-to-fill instead, better for a photo), optionally
converts to grayscale (--gray -- R=G=B per pixel, still 3 bytes/px on the
wire: the format every sstv_stream_*() already expects, no protocol change),
and writes the same "RGB RGB RGB..., row-major" C array layout as the
existing headers.

Usage:
    python3 tools/sstv_gen_image.py SOURCE.jpg OUT.h ARRAY_NAME WIDTH HEIGHT \\
        [--gray] [--fit contain|cover] [--bg R,G,B]

Example (the ADRASEC logo, grayscale, both resolutions this project uses):
    python3 tools/sstv_gen_image.py adrasec.jpg \\
        rp2040/src/sstv_master_image_adrasec.h g_sstv_master_image_adrasec \\
        320 256 --gray
    python3 tools/sstv_gen_image.py adrasec.jpg \\
        rp2040/src/sstv_master_image_adrasec_pd120.h \\
        g_sstv_master_image_adrasec_pd120 640 496 --gray
"""
import argparse
import os
import sys

from PIL import Image, ImageOps


def fit_contain(im, w, h, bg):
    src_w, src_h = im.size
    scale = min(w / src_w, h / src_h)
    new_w, new_h = max(1, round(src_w * scale)), max(1, round(src_h * scale))
    resized = im.resize((new_w, new_h), Image.LANCZOS)
    canvas = Image.new("RGB", (w, h), bg)
    canvas.paste(resized, ((w - new_w) // 2, (h - new_h) // 2))
    return canvas


def fit_cover(im, w, h):
    src_w, src_h = im.size
    scale = max(w / src_w, h / src_h)
    new_w, new_h = max(1, round(src_w * scale)), max(1, round(src_h * scale))
    resized = im.resize((new_w, new_h), Image.LANCZOS)
    left = (new_w - w) // 2
    top = (new_h - h) // 2
    return resized.crop((left, top, left + w, top + h))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("source")
    p.add_argument("out_header")
    p.add_argument("array_name")
    p.add_argument("width", type=int)
    p.add_argument("height", type=int)
    p.add_argument("--gray", action="store_true",
                    help="convert to grayscale (R=G=B), e.g. for a logo/line-art "
                         "test image where color carries no useful detail anyway "
                         "and flat/sharp edges make skew or wrong-channel bugs "
                         "obvious at a glance")
    p.add_argument("--bw", action="store_true",
                    help="like --gray, but also threshold to pure black/white "
                         "(0x00/0xFF only, via --threshold) -- the strongest test "
                         "pattern for verifying decode quality: any artefact "
                         "(noise, skew, wrong level) is immediately obvious, no "
                         "shade-of-gray judgment call needed. Implies --gray.")
    p.add_argument("--threshold", type=int, default=128,
                    help="--bw cut point, 0-255 (default: 128)")
    p.add_argument("--fit", choices=("contain", "cover"), default="contain",
                    help="contain (default): whole source visible, padded with "
                         "--bg -- best for a logo. cover: crop to fill, no "
                         "padding -- best for a photo")
    p.add_argument("--bg", default="255,255,255",
                    help="pad color for --fit contain, R,G,B 0-255 (default: white)")
    args = p.parse_args()

    bg = tuple(int(x) for x in args.bg.split(","))
    if len(bg) != 3:
        sys.exit("--bg must be R,G,B")

    im = Image.open(args.source).convert("RGB")
    im = ImageOps.exif_transpose(im)

    if args.fit == "contain":
        im = fit_contain(im, args.width, args.height, bg)
    else:
        im = fit_cover(im, args.width, args.height)

    if args.gray or args.bw:
        im = ImageOps.grayscale(im)
        if args.bw:
            im = im.point(lambda p: 255 if p >= args.threshold else 0)
        im = im.convert("RGB")

    assert im.size == (args.width, args.height)

    data = im.tobytes()  # RGB RGB RGB..., row-major -- exactly the wire format
    assert len(data) == args.width * args.height * 3

    src_name = os.path.basename(args.source)
    with open(args.out_header, "w") as f:
        f.write(f"""/* Auto-generated SSTV master image (tools/sstv_gen_image.py) -- source:
 * {src_name}, fit={args.fit}{"+bw" if args.bw else "+gray" if args.gray else ""}, {args.width}x{args.height}.
 * Format: RGB RGB RGB..., row by row, 3 B/pixel (grayscale is just R=G=B --
 * the wire format every sstv_stream_*() in rp2040/src/main.c already expects
 * is unchanged). */
#include <stdint.h>

#define {args.array_name.upper()}_W {args.width}
#define {args.array_name.upper()}_H {args.height}
static const uint8_t {args.array_name}[{args.array_name.upper()}_W * {args.array_name.upper()}_H * 3] = {{
""")
        for i in range(0, len(data), 20):
            f.write(",".join(f"0x{b:02X}" for b in data[i:i + 20]) + ",\n")
        f.write("};\n")

    print(f"wrote {args.out_header}: {args.width}x{args.height}, "
          f"{len(data)} B source pixels -> {os.path.getsize(args.out_header)} B header")


if __name__ == "__main__":
    main()
