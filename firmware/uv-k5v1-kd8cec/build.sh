#!/usr/bin/env bash
# Reproducible build of the UV-K5 V1 (KD8CEC) firmware with the SARSAT screen.
#
# Base : KD8CEC "uvk5cec-0.3q" source (egzumer lineage, DP32G030 / BK4819).
# Adds : ENABLE_SARSAT (app/sarsat.c + a handful of small hooks) and turns on
#        ENABLE_BYP_RAW_DEMODULATORS so MODULATION_RAW exists.
# Radio UART stays at the DualTachyon divisor (Frequency/39053 ≈ 38400 8N1) —
# no change needed, it already matches the RP2040 side.
#
# Usage:  ./build.sh /path/to/uvk5cec-0.3q
set -euo pipefail

SRC="${1:?usage: build.sh <path-to-uvk5cec-0.3q source tree>}"
HERE="$(cd "$(dirname "$0")" && pwd)"
W="$(mktemp -d)"
trap 'rm -rf "$W"' EXIT

cp -a "$SRC"/. "$W"/
cd "$W"

# 1. stock KD8CEC ceccommon.h uses Windows backslash #include paths -> fix for GCC
sed -i 's#include "driver\\#include "driver/#g; s#include "ui\\#include "ui/#g; s#include "app\\#include "app/#g' ceccommon.h

# 2. drop in the new modules (SARSAT screen + APRS tracker + AX.25)
cp "$HERE/patch/sarsat.c" app/sarsat.c
cp "$HERE/patch/sarsat.h" app/sarsat.h
cp "$HERE/patch/aprs.c"   app/aprs.c
cp "$HERE/patch/aprs.h"   app/aprs.h
cp "$HERE/patch/ax25.c"   app/ax25.c
cp "$HERE/patch/ax25.h"   app/ax25.h

# 3. apply the source hooks
for d in app_uart.c app_app.c app_main.c settings.c radio.h radio.c ui_main.c ui_status.c misc.h Makefile; do
    f="${d/_//}"
    patch -p0 --forward "$f" < "$HERE/patch/${d}.diff"
done

# 4. build (native arm-none-eabi-gcc; openocd targets are not invoked by 'all')
#    Disabled to make room for SARSAT + APRS (flash is 60 KB):
#      SPECTRUM (~7 KB), FMRADIO broadcast (~3 KB), VOX + FLASHLIGHT (~1 KB),
#      AUDIO_BAR (the TX mic-level bar overlay, unused here).
#    MAIN_SCREEN = stock | moto | id91  (redesigned main VFO screen; "stock"
#    keeps the original, "id91" is ~700 B lighter than "moto").
make -j"$(nproc)" \
    ENABLE_SARSAT=1 \
    ENABLE_APRS=1 \
    ENABLE_BYP_RAW_DEMODULATORS=1 \
    ENABLE_SPECTRUM=0 \
    ENABLE_FMRADIO=0 \
    ENABLE_VOX=0 \
    ENABLE_FLASHLIGHT=0 \
    ENABLE_AUDIO_BAR=0 \
    MAIN_SCREEN=moto \
    VERSION_STRING=CEC3qSAR \
    AUTHOR_STRING=KD8CEC_SARSAT

python3 fw-pack.py firmware.bin KD8CEC_SARSAT CEC3qSAR firmware.packed.bin

mkdir -p "$HERE/bin"
cp firmware.bin        "$HERE/bin/uvk5-cec-0.3q-sarsat.bin"
cp firmware.packed.bin "$HERE/bin/uvk5-cec-0.3q-sarsat.packed.bin"
( cd "$HERE/bin" && sha256sum *.bin > sha256.txt )
arm-none-eabi-size firmware
echo "OK -> $HERE/bin/"
