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
)

target_include_directories(sarsat_rp2040 PRIVATE src)

target_compile_definitions(sarsat_rp2040 PRIVATE SARSAT_VERBOSE=0)
# don't let a slow / stalled serial monitor block the ADC sample pump: a
# console write gives up after 1 ms instead of the 500 ms default.
target_compile_definitions(sarsat_rp2040 PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)
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
