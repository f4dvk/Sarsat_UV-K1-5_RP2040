# RP2040 ↔ radio link — SARSAT application protocol (draft, Phase 2)

Physical layer (from KD8CEC's C-Board `cboard_v032.uf2`, static analysis):

| line | pin | rate | use |
|------|-----|------|-----|
| radio link | RP2040 **GP0 = UART0 TX**, **GP1 = UART0 RX** | **38400 8N1** | this protocol |
| GPS | GP4/GP5 = UART1 | 9600 8N1 | NMEA, not used by the SARSAT firmware |

We do **not** use KD8CEC's proprietary C-Board framing. Baud is a free choice
because both ends are ours: the SARSAT firmware patches for the UV-K1 and
UV-K5 V1 keep the UART at **38400** (the egzumer/F4HWN default, and what
`benshi-esp32-sim`'s tooling uses). KD8CEC's stock C-Board firmware uses 57600
— irrelevant here since it is replaced.

Transport = the standard Quansheng UART frame (egzumer / F4HWN
`App/app/uart.c`), **RP2040 → radio only** for now:

```
AB CD | size:u16 LE | <inner> | crc16:u16 LE | DC BA
inner = [ID:u16 LE][data_size:u16 LE][data…]
inner + crc are XOR-masked with the 16-byte Obfuscation table
```

38400 8N1. Encoder: `rp2040/src/quansheng_frame.c`.
The radio firmware (Phase 3/4) adds a handler for the IDs below in
`UART_HandleCommand()` plus a small "SARSAT" screen app.

| ID | Name | Payload | Meaning |
|----|------|---------|---------|
| `0x06CF` | `SARSAT_HELLO` | `proto_ver:u8` | keepalive, ~every 5 s. Radio may show a link indicator. |
| `0x06C0` | `SARSAT_CLEAR` | — | clear the SARSAT screen buffer |
| `0x06C1` | `SARSAT_TEXT` | `line:u8, invert:u8, ascii[0..20]` | set one display line (0 = top). `invert` = render reversed. ASCII only, ≤ 21 glyphs. |
| `0x06C2` | `SARSAT_BEACON` | packed struct (below) | machine-readable decode result; radio formats it itself. Optional alternative to `0x06C1`. |
| `0x06C3` | `SARSAT_LEVEL` | `peak:u16,rms:u16,dc:u16,clip:u8,adcmin:u16,adcmax:u16,verdict:u8` LE | audio-level telemetry for the radio's tuning screen, ~1/s, no ACK |
| `0x06D0` | `APRS_CONFIG` | (reserved) radio → RP2040: call / SSID / path / symbol | for digipeat / ack — not wired yet. Radio-side config lives in EEPROM `0x1D00` (40 B; **not** `0x1D50`, which `SETTINGS_SaveSettings()` overwrites). |
| `0x06D2` | `APRS_RXTEXT` | `line:u8, ascii[0..18]` | RP2040 → radio: one line of a decoded 144.8 MHz APRS packet, used only when the info field could **not** be parsed. `line = 0xFF` clears the RX view; `line = 0` is the source callsign, `1..3` the wrapped info field. Not ACKed. |
| `0x06D3` | `APRS_RXINFO` | structured decode (below) | RP2040 → radio: a parsed APRS packet (symbol, lat/lon, course/speed/altitude, range+bearing to the operator, source, object name, digipeater path, comment/status/message text). The radio renders it Kenwood-style with a symbol icon and a "Direct" / "Via …" line. Not ACKed. |
| `0x06D5` | `APRS_GPS` | `flags:u8, lat_e5:i32, lon_e5:i32, speed_kmh:u16, course_deg:u16, alt_m:i16, sats:u8` LE (16 B) | RP2040 → radio, ~every 3 s: the fix from a GPS module on the C-Board's NMEA header (UART1 GP5, 9600 8N1, `$GxRMC`/`$GxGGA`). `flags` bit0 = fix valid. Sent only once a module has been seen. The radio uses it for the beacon when its APRS menu **Pos** field is set to **GPS** (else it beacons the manual lat/lon), and drives a top-bar GPS symbol from it (absent = no frames, blinking = `flags` bit0 clear, solid = fix). The RP2040 also uses its own fix for the RX range/bearing when valid. Not ACKed. |

### Mode selection

The C-Board runs the SARSAT **or** the APRS decoder, never both, and picks from
the RX frequency in the `SARSAT_HELLO` reply: `144.0–148.0 MHz` → APRS (Bell-202
AFSK 1200 + AX.25, `rp2040/src/aprs_rx.c`, ported from JN1DFF pico_tnc), anything
else → SARSAT FGB. The switch takes effect within one HELLO round-trip (~1 s).

`SARSAT_TEXT` is the primary path: all formatting lives in the RP2040
(`sarsat_format_lines()` — one field per line, long strings word-wrapped, up to
`SARSAT_MAX_LINES` = 14). The radio stores the lines and **scrolls** them
(UV-K5: UP/DOWN; UV-K1: the equivalent side keys), 7 rows + a header visible at
once. A decode pushes `CLEAR` then the lines back-to-back; the RP2040 spaces the
frames ~8 ms apart and the radio drains its UART ring in a `while` loop so none
are lost. `SARSAT_BEACON` is provided for a radio that wants to lay the fields
out its own way.

### `APRS_RXINFO` payload (little-endian)

```
u8   kind             0 other, 1 position, 2 object, 3 status, 4 message, 5 telemetry
u8   flags            bit0 has_pos, bit1 has_course/speed, bit2 has_alt, bit3 has_range
char sym_table        '/', '\' or overlay char
char sym_code         APRS symbol code
i32  lat_e5           latitude  × 1e5  (+ = N)
i32  lon_e5           longitude × 1e5  (+ = E)
u16  course_deg       0..359
u16  speed_kmh
i16  alt_m
u16  dist_hm          range to the operator, in hectometres (0 if flag clear)
u16  bearing_deg      bearing to the target from the operator
char src[]            NUL-terminated source callsign
char name[]           NUL-terminated object/item name ("" for a plain position)
char via[]            NUL-terminated: the digipeaters that actually repeated the
                      frame (AX.25 H bit set), comma-separated, WIDEn/TRACEn-style
                      aliases dropped ("F8KCS-3,F8KCS-2"). "" = heard directly.
char text[]           NUL-terminated comment / status / message text
```

Range and bearing are computed on the RP2040 (equirectangular approximation)
from the operator's own position, which the radio sends in its `SARSAT_HELLO`
reply (below). They are omitted (flag clear) until the operator sets a position
in the APRS config screen.

### `SARSAT_BEACON` payload (little-endian)

```
u8   frame_bits        112 or 144
u8   protocol           ProtocolType enum (dec406_v1g.h)
u8   is_test            1 = test/self-test frame
u8   has_position
u16  country_code
i32  lat_1e4            latitude  × 1e4 (0 if no position)
i32  lon_1e4            longitude × 1e4
char hex_id[15]         COSPAS 15-hex beacon ID
char ident[24]          identification string (truncated)
```
Total 55 bytes.

## Phase 2 — the radio → RP2040 direction (ACK / status)

Currently RP2040 → radio is one-way. Phase 2 adds a reply on the same wire:

- **ACK**: the radio answers each `0x06Cx` with ID = `command | 0x8000`, payload
  `status:u8` (0 = OK). Lets the RP2040 log `[link] radio ACK` / detect a dead
  or wrongly-wired link.
- **Status**: the radio answers `SARSAT_HELLO` (`0x06CF | 0x8000`) with
  `vfo:u8, modulation:u8, rx_freq:u32 (10 Hz units), sarsat_screen:u8,
  proto_ver:u8, my_lat_e5:i32, my_lon_e5:i32` (16 bytes; the trailing position
  is 0/absent on older radio builds — the RP2040 accepts an 8-byte reply too).
  `my_lat/lon` is the radio's manual APRS position, or its live GPS fix when the
  APRS menu **Pos** field is **GPS** and a fix is current.
  `sarsat_screen` = **0 closed / 1 decode view / 2 level view**; on `2` the
  RP2040 switches to short capture windows so the level bar refreshes at ~7 Hz
  for AF-gain tuning (and stops decoding until it leaves). The radio also sends
  this reply **unsolicited** on entering/leaving the SARSAT screen and on the
  decode↔level toggle, so the change takes effect without waiting for the 5 s
  HELLO. The RP2040 also warns if the radio is not near 406 MHz or not in FM,
  picks the APRS vs SARSAT decoder from the frequency, and uses the operator
  position for the `APRS_RXINFO` range/bearing.

Both ends need a small change:
- radio firmware: call `SendReply()` from the `0x06Cx` handler.
- RP2040: a frame **parser** (`quansheng_frame.c` is encode-only today) + read
  the reply after `HELLO` in `main.c`.

### Testing Phase 2 without a radio

`tools/fake_radio.py` emulates the radio end. Wire a 3.3 V USB-TTL adapter to
the C-Board UART instead of the handheld (adapter RX ← RP2040 GP0, adapter TX →
GP1, common GND), then:

```
tools/fake_radio.py /dev/ttyUSB0                # decode the RP2040's frames
tools/fake_radio.py /dev/ttyUSB0 --ack          # + reply ACK to each 0x06Cx
tools/fake_radio.py /dev/ttyUSB0 --ack --status # + fake VFO/freq/mod on HELLO
```

- **decode mode** already works and needs no firmware change — it verifies the
  RP2040's frame encoder (CRC-16/XMODEM, obfuscation, framing) and prints every
  `CLEAR` / `TEXT L.. "..."` / `HELLO` exactly as the radio parser sees them.
  Its build/parse round-trips bit-for-bit against `quansheng_frame.c`.
- **--ack / --status** exercise the return path once the RP2040 parser lands.

You can also point it at the real radio's programming cable to watch what the
UV-K5 firmware sends back (nothing yet, until `SendReply()` is added).
