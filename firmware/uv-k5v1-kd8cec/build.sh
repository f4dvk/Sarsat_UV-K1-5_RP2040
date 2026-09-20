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
cp "$HERE/patch/sonde.c"  app/sonde.c
cp "$HERE/patch/sonde.h"  app/sonde.h

#    Replace the CTCSS/DCS + frequency scanner (DISPLAY_SCANNER) with stubs:
#    ~1.5 KB of flash for a feature unrelated to SARSAT/APRS. The normal
#    memory-channel scan (chFrScanner.c) is a separate module, untouched.
#    SCANNER_TimeSlice10ms() bounces straight back out, so F+4 / F+* / the
#    R-CTCS submenu scan key just flash for one tick and return.
cp "$HERE/patch/scanner.c"    app/scanner.c
cp "$HERE/patch/ui_scanner.c" ui/scanner.c

# 3. apply the source hooks.
#    ceccommon.c / ui_menu.{c,h} / app_menu.c : remove KD8CEC's "Live.S" (Live
#    Seek) feature -- the mini RSSI-spectrum overlay shown while seeking. It is
#    off by default, unrelated to SARSAT/APRS, and freed ~800 B of flash for the
#    APRS SmartBeaconing work. The 3 helpers in ceccommon.c are gutted (kept as
#    empty stubs so their call sites need no edit); the menu entry + submenu
#    strings + handlers are dropped (MENU_LIVESEEK enum slot kept, now unused).
#    settings.h / app_action.c / ui_menu.c (this last one already patched
#    above for Live Seek, second hunk added here) : register SARSAT/APRS/
#    SONDE as ACTION_OPT_* options in the existing, stock "Side1/Side2/M key
#    -- short or long (2 s) press -- runs an assignable action" mechanism
#    (Fonctions > SIDE1/SIDE2/M, already present on this firmware exactly as
#    on V3 -- KEY_1/2/M_LONG_PRESS_ACTION, gSubMenu_SIDEFUNCTIONS,
#    action_opt_table[]). On explicit user request ("un appui 2 secondes sur
#    une touche fait la fonction associee [sur V3], peut-on faire de meme sur
#    la v1 ?") -- this project's own screens were only reachable via the
#    fixed F+8/F+5/F+0 combos on V1 until now (see app_main.c.diff below),
#    never through this assignable mechanism, unlike V3 (see
#    firmware/uv-k1-k5v3/build.sh's own ACTION_OPT_SARSAT/APRS/SONDE
#    patches). F+8/F+5/F+0 are left untouched -- this is purely additive, a
#    second way in for whoever prefers a long-press on SIDE1/SIDE2/M to
#    opening the menu to pick which screen.
for d in app_uart.c app_app.c app_main.c settings.c settings.h radio.h radio.c ui_main.c ui_main.h ui_status.c misc.h Makefile \
         ceccommon.c ui_menu.c ui_menu.h app_menu.c app_action.c; do
    f="${d/_//}"
    patch -p0 --forward "$f" < "$HERE/patch/${d}.diff"
done

# 3b. retrait de DTMF (~1,1 Ko de marge flash). Ce projet (SARSAT / APRS)
#     n'utilise aucune fonction DTMF : PTT-ID / ANI, tonalité de courtoisie de
#     fin d'émission, numérotation DTMF manuelle, décodeur DTMF live. Les 5
#     fonctions non gardées de app/dtmf.c sont vidées ; DTMF_ValidateCodes()
#     reste (settings.c la utilise, minuscule) ; les globales restent (bss,
#     ~70 o). Les entrées de menu DTMF apparaissent encore mais ne font plus
#     rien. LTO élague ensuite les chemins d'appel (app/main.c, ui/main.c...).
#     Pour réactiver DTMF : retirer ce bloc (les 5 perl + les 2 grep).
perl -0pi -e 's{void DTMF_Reply\(void\)\n\{.*?\n\}\n}{void DTMF_Reply(void) { }  // Sarsat_UV-K1-5_RP2040: DTMF removed\n}s' app/dtmf.c
perl -0pi -e 's{void DTMF_SendEndOfTransmission\(void\)\n\{.*?\n\}\n}{void DTMF_SendEndOfTransmission(void) { }  // Sarsat_UV-K1-5_RP2040\n}s' app/dtmf.c
perl -0pi -e 's{char DTMF_GetCharacter\(const unsigned int code\)\n\{.*?\n\}\n}{char DTMF_GetCharacter(const unsigned int code) { (void)code; return 0xff; }  // Sarsat_UV-K1-5_RP2040\n}s' app/dtmf.c
perl -0pi -e 's{void DTMF_Append\(const char code\)\n\{.*?\n\}\n}{void DTMF_Append(const char code) { (void)code; }  // Sarsat_UV-K1-5_RP2040\n}s' app/dtmf.c
perl -0pi -e 's{void DTMF_clear_input_box\(void\)\n\{.*?\n\}\n}{void DTMF_clear_input_box(void) { memset(gDTMF_InputBox, 0, sizeof(gDTMF_InputBox)); gDTMF_InputBox_Index = 0; gDTMF_InputMode = false; }  // Sarsat_UV-K1-5_RP2040\n}s' app/dtmf.c

grep -q 'Sarsat_UV-K1-5_RP2040: DTMF removed' app/dtmf.c || { echo "!! dtmf.c : DTMF_Reply non videe"; exit 1; }
[ "$(grep -c 'Sarsat_UV-K1-5_RP2040' app/dtmf.c)" -ge 5 ] || { echo "!! dtmf.c : toutes les fonctions n'ont pas ete videes"; exit 1; }

# 4. build (native arm-none-eabi-gcc; openocd targets are not invoked by 'all')
#    Disabled to make room for SARSAT + APRS + the "121 MHz report" message
#    (flash is 60 KB; ~3.1 KB free after the CTCSS/DCS-scanner removal above):
#      SPECTRUM (~7 KB), FMRADIO broadcast (~3 KB), VOX + FLASHLIGHT (~1 KB),
#      AUDIO_BAR (the TX mic-level bar overlay, unused here),
#      COPY_CHAN_TO_VFO (~110 B, the "copy current channel to VFO" shortcut),
#      SCAN_RANGES (~220 B, scan-by-frequency-range -- scan-by-channel stays),
#      SMALL_BOLD (~576 B, the bold variant of the 6 px font -- headers just
#        render in normal weight; UI_PrintStringSmallBold() falls back cleanly).
#        Freed for the ADRASEC coordinate screen + received-message auto-ACK.
#    MAIN_SCREEN = stock | moto | id91  (redesigned main VFO screen; "stock"
#    keeps the original). "id91" reuses the stock big-digit font + stock
#    horizontal S-meter instead of moto's own font tables -> ~340 B lighter
#    than "moto" for the same icom-style layout. Kept on "id91" for the flash
#    headroom; switch back to "moto" if a build ever has room to spare.
make -j"$(nproc)" \
    ENABLE_SARSAT=1 \
    ENABLE_APRS=1 \
    ENABLE_SONDE=1 \
    ENABLE_BYP_RAW_DEMODULATORS=1 \
    ENABLE_SPECTRUM=0 \
    ENABLE_FMRADIO=0 \
    ENABLE_VOX=0 \
    ENABLE_FLASHLIGHT=0 \
    ENABLE_AUDIO_BAR=0 \
    ENABLE_COPY_CHAN_TO_VFO=0 \
    ENABLE_SCAN_RANGES=0 \
    ENABLE_SMALL_BOLD=0 \
    MAIN_SCREEN=id91 \
    VERSION_STRING=CEC3qSAR \
    AUTHOR_STRING=KD8CEC_SARSAT

python3 fw-pack.py firmware.bin KD8CEC_SARSAT CEC3qSAR firmware.packed.bin

mkdir -p "$HERE/bin"
cp firmware.bin        "$HERE/bin/uvk5-cec-0.3q-sarsat.bin"
cp firmware.packed.bin "$HERE/bin/uvk5-cec-0.3q-sarsat.packed.bin"
( cd "$HERE/bin" && sha256sum *.bin > sha256.txt )
arm-none-eabi-size firmware
echo "OK -> $HERE/bin/"
