# Host (PC) build of the SARSAT 1G decode path — no pico-sdk.
# Validates the port against the upstream moricef decoder before it ever
# touches an RP2040.
#
#   make            build test_decode + test_slicer
#   make parity     diff test_decode against upstream ./build/dec406_hex
#   make check      parity + slicer fixed-vs-double
#   make clean

SRC_DIR   := ../../src
# Upstream reference decoder (github.com/moricef/Decode_sarsat_406_v1g_v2g),
# cloned next to this repo by default. Override: make parity UPSTREAM=/path
UPSTREAM  ?= $(HOME)/Decode_sarsat_406_v1g_v2g

CC        := gcc
CFLAGS    := -std=c11 -Wall -Wextra -O2 -I$(SRC_DIR) \
             -DSARSAT_HOST_BUILD -DSARSAT_VERBOSE=0
LDLIBS    := -lm

ifdef DIAG
CFLAGS    += -DSARSAT_HOST_DIAG
endif

PORT_SRCS := $(SRC_DIR)/dec406_v1g.c $(SRC_DIR)/dec406.c \
             $(SRC_DIR)/dec406_2g_stub.c $(SRC_DIR)/country_codes.c \
             $(SRC_DIR)/display_min.c $(SRC_DIR)/audio_slicer.c \
             $(SRC_DIR)/sarsat_decoder.c $(SRC_DIR)/quansheng_frame.c

APRS_DIR := ../../../firmware/uv-k5v1-kd8cec/patch

.PHONY: all parity check clean
all: test_decode test_slicer test_link test_ax25 test_aprs_rx test_aprs_parse test_gps test_aprs_digi test_kiss

test_decode: hexmain.c $(PORT_SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test_slicer: test_slicer.c $(PORT_SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test_link: test_link.c $(SRC_DIR)/quansheng_frame.c
	$(CC) $(CFLAGS) -o $@ $^

test_ax25: test_ax25.c $(APRS_DIR)/ax25.c
	$(CC) -std=c11 -Wall -Wextra -O2 -I$(APRS_DIR) -o $@ $^

test_aprs_rx: test_aprs_rx.c $(SRC_DIR)/aprs_rx.c $(APRS_DIR)/ax25.c
	$(CC) $(CFLAGS) -I$(APRS_DIR) -o $@ $^ $(LDLIBS)

# off-line decode of a [aprs.raw] capture from the target's `w` command
test_aprs_wav: test_aprs_wav.c $(SRC_DIR)/aprs_rx.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test_aprs_parse: test_aprs_parse.c $(SRC_DIR)/aprs_parse.c $(APRS_DIR)/ax25.c
	$(CC) $(CFLAGS) -I$(APRS_DIR) -o $@ $^ $(LDLIBS)

test_gps: test_gps.c $(SRC_DIR)/gps_nmea.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test_aprs_digi: test_aprs_digi.c $(SRC_DIR)/aprs_digi.c $(APRS_DIR)/ax25.c
	$(CC) $(CFLAGS) -I$(APRS_DIR) -o $@ $^ $(LDLIBS)

test_kiss: test_kiss.c $(SRC_DIR)/kiss.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

parity: test_decode
	@./run_parity.sh "$(UPSTREAM)"

check: all parity
	@./test_slicer
	@./test_link
	@./test_ax25
	@./test_aprs_rx
	@./test_aprs_parse
	@./test_gps
	@./test_aprs_digi
	@./test_kiss

clean:
	rm -f test_decode test_slicer test_link test_ax25 test_aprs_rx test_aprs_parse test_aprs_wav test_gps test_aprs_digi test_kiss fgb_bits.csv
