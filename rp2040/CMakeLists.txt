# RP2040 / RP2350 build for the SARSAT 1G beacon decoder.
#
#   export PICO_SDK_PATH=~/pico-sdk
#   cmake -S . -B build -DPICO_BOARD=pico      # RP2040 (Pico)
#   cmake -S . -B build -DPICO_BOARD=pico2     # RP2350 (Pico 2)
#   cmake --build build
#
# The 1G decode path is fixed-point by default (works on the FPU-less RP2040).
# On RP2350 you may pass -DSARSAT_SLICER_DOUBLE=ON to use the double slicer.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(sarsat_rp2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

option(SARSAT_SLICER_DOUBLE "Use the double-precision bit slicer" OFF)

add_executable(sarsat_rp2040
    src/main.c
    src/sarsat_decoder.c
    src/audio_slicer.c
    src/dec406.c
    src/dec406_v1g.c
    src/dec406_2g_stub.c
    src/country_codes.c
    src/display_min.c
    src/quansheng_frame.c
    src/aprs_rx.c
    src/aprs_parse.c
    src/aprs_digi.c
    src/gps_nmea.c
    src/kiss.c
    src/sonde_demod.c
    src/sonde_rs41.c
    src/sonde_sync.c
    src/sonde_dfm.c
    src/sonde_m10.c
    src/sstv_demod.c
    src/usb_desc_fix.c
)

# The SDK's CDC notification endpoint is 8 bytes wide, too small for the
# 10-byte SERIAL_STATE notification main.c sends for legacy KISS hosts
# (UI-View32). usb_desc_fix.c re-declares it at 16 bytes and takes over the
# configuration-descriptor callback through the linker (see that file).
target_link_options(sarsat_rp2040 PRIVATE "LINKER:--wrap=tud_descriptor_configuration_cb")

target_include_directories(sarsat_rp2040 PRIVATE src)

# main() ne doit pas inliner les helpers (SSTV, SONDE, APRS) : gcc les copiait
# dans la trame de main() (5656 o avec le streaming SSTV, 840 o sans), et la
# pile de 8 Ko (RAM haute, voir memmap) debordait sur la bss des le premier
# paquet APRS -> RP2040 fige sans message. Mesure : cmake -DCMAKE_C_FLAGS=
# -fstack-usage puis lire les .su (main.c.su).
set_source_files_properties(src/main.c PROPERTIES
    COMPILE_OPTIONS "-fno-inline-small-functions;-fno-inline-functions")

target_compile_definitions(sarsat_rp2040 PRIVATE SARSAT_VERBOSE=0)
# don't let a slow / stalled serial monitor block the ADC sample pump: a
# console write gives up after 1 ms instead of the 500 ms default.
target_compile_definitions(sarsat_rp2040 PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)
# AGWPE opens the COM port without raising DTR, and by default the SDK then
# ignores the port both ways (no KISS RX out, no KISS TX in): treat a mounted
# USB as connected instead.
target_compile_definitions(sarsat_rp2040 PRIVATE PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)
# The SDK reboots into BOOTSEL when a host selects 1200 Bd -- the natural
# setting for a 1200 Bd KISS TNC in UI-View/AGWPE. The COM port would simply
# vanish. picotool still resets through its vendor interface.
target_compile_definitions(sarsat_rp2040 PRIVATE PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=0)
if(SARSAT_SLICER_DOUBLE)
    target_compile_definitions(sarsat_rp2040 PRIVATE SARSAT_SLICER_DOUBLE=1)
endif()

target_compile_options(sarsat_rp2040 PRIVATE -Wall -Wextra)

# dec406_v1g.c is a near-verbatim port of the upstream moricef decoder, which
# uses %u/%X for uint32_t. Harmless on the 32-bit target (same width/ABI as
# unsigned long); silenced here to keep new merges from upstream trivial.
set_source_files_properties(src/dec406_v1g.c PROPERTIES
    COMPILE_OPTIONS "-Wno-format;-Wno-stringop-truncation;-Wno-unused"
    COMPILE_DEFINITIONS "SARSAT_QUIET_DECODER=1")

target_link_libraries(sarsat_rp2040
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_uart
)

# debug console on USB-CDC, radio link owns uart1
pico_enable_stdio_usb(sarsat_rp2040 1)
pico_enable_stdio_uart(sarsat_rp2040 0)

pico_add_extra_outputs(sarsat_rp2040)
