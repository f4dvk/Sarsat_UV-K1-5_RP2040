# Prebuilt RP2040 / RP2350 SARSAT + APRS decoder firmware

| file | board | MCU | text / bss |
|---|---|---|---|
| `sarsat_rp2040-pico.uf2`  | Raspberry Pi Pico / RP2040-Zero | RP2040 (Cortex-M0+, no FPU) | 74 KB / 126 KB |
| `sarsat_rp2040-pico2.uf2` | Raspberry Pi Pico 2 | RP2350 | 69 KB / 126 KB |

Both are the **same firmware** (`../src/`), differing only in the target core.

## Two decoders, auto-selected

The C-Board runs **one** of two decoders at a time, chosen from the RX
frequency the radio reports in its `0x06CF` HELLO reply:

- **~406 / 434 MHz** → COSPAS-SARSAT 1st-gen (FGB / T.001), 1.1 s windows.
- **144.0–148.0 MHz** → **APRS**: continuous Bell-202 AFSK 1200 demod + AX.25
  de-frame. 3 parallel slicers (biased ±3/8 envelope) + a single-bit-flip FCS
  repair (Dire-Wolf-style recovery -- AX.25 has no FEC). The info field is then
  parsed (uncompressed / compressed / MIC-E / object / item / status / message)
  and pushed to the radio as a structured `0x06D3` decode -- symbol, lat/lon,
  course/speed/altitude, and range+bearing to the operator (from the position
  in the radio's HELLO reply) -- so the radio can draw it Kenwood-style. A
  packet whose info field is not understood falls back to `0x06D2` raw text.
  An **IF limiter** normalises the discriminator input, so a strong (even
  ADC-clipped) signal demodulates like a nominal one — no "loud = distorted =
  no decode". The `[aprs]` console line reports `clip=NN.N%` (ADC-rail hits);
  above ~2 % it prints `lower the radio AF gain`. See `[aprs]` lines
  (`M fixed` = frames recovered by the bit-flip).

Tune the radio's VFO and the C-Board follows within ~1 s (a `mode APRS` /
`mode SARSAT` line prints on the switch).

## GPS (optional)

A GPS module on the C-Board's NMEA header (**UART1 GP5, 9600 8N1**) is parsed
(`$GxRMC` + `$GxGGA`) and the fix is forwarded to the radio as `0x06D5` every
~3 s. In the radio's APRS menu (F+5) set **Pos** to **GPS** and the beacon uses
the live position (plus a course/speed extension when moving); leave it on
**manual** to beacon the entered lat/lon. With no module connected nothing is
sent and the behaviour is unchanged. The RP2040 also uses its own fix for the
RX range/bearing when it is current. The radio's top bar shows a small GPS
symbol: **blinking** while the module searches, **solid** once it has a fix,
**nothing** when no module is attached.
The bit slicer is fixed-point (int64) in both so RP2040 works without an FPU;
`-DSARSAT_SLICER_DOUBLE=ON` at CMake time switches to the double slicer (only
worth it on RP2350).

KD8CEC's C-Board uses an **RP2040-Zero**, so `sarsat_rp2040-pico.uf2` is the one
to flash there.

## Flash

Hold BOOTSEL while plugging in USB → a `RPI-RP2` drive appears → copy the `.uf2`
onto it. The board reboots into the firmware.

## Debug console (USB-CDC)

Open the RP2040's USB serial port (`/dev/ttyACM0`, any baud). Lines are tagged:

| tag | when | meaning |
|-----|------|---------|
| `[lvl]` | every ~3 s while idle | `peak` / `rms` of the audio window, raw ADC min..max, DC. **Set the radio volume so `peak` reads ~4000-12000 on a beacon.** |
| `[burst]` | audio crossed the arm threshold | decoder is running on that window |
| `[slicer]` | after each burst | `144 bits: <hex>` (raw sliced frame, sync included) or `no sync / no frame` |
| `[decode]` | after slicing | `BCH OK` + hex ID / country / protocol / position / ident, or `BCH uncorrectable` |
| `[tx]` | on decode, and the 5 s keepalive | `CLEAR`, `TEXT L0 "..."`, byte totals, `HELLO` (with ack count) |
| `[link]` | radio reply, and a 20 s heartbeat | `link UP/DOWN`, `acks=N`, and a `status:` line on change (radio VFO / modulation / RX freq / screen). Warns only if the radio is not in FM. No frequency restriction — 406 or 434 MHz exercise beacons both work. |
| `[meter]` | while the meter is on (`m`) | per-window `rms/peak/dc/clip/adc` + a bar, for setting the radio volume |
| `[aprs]` | APRS mode: on each packet + a ~3 s status | `#N SRC` then wrapped info; status = `CARRIER/idle  mod=FM  hdlc=N -> pkts=N (M fix) fcs_bad=N  clip=X.X%  cdt=N env=N` (`mod` = the radio's current demod from its HELLO reply — must read `FM` or `DSC`; `cdt` = carrier-detect energy, `env` = discriminator envelope) |
| `[aprs.rx]` | **provisional** (`CFG_APRS_RX_DIAG`): each non-trivial HDLC frame candidate | `chC RES len=L ui=U src=CALL \| <hex head>` — `RES` = `OK`/`FIX`(1-bit repaired)/`BAD`(FCS fail)/`LNG`(overrun)/`DUP` (`SHT` fragments are counted but not printed). `ui=1` = the byte structure looks like a valid AX.25 UI frame. Reads: `hdlc=0` forever → nothing demodulated (level/tune/squelch); `BAD` with `ui=1` + readable `src` but the **length varies wildly for the same station** → bit errors corrupting the HDLC framing. Usual causes, in order: the radio squelch **chattering** mid-burst (each open/close blanks ~10–20 ms = 12–24 bits — use the APRS `Squelch fast` option, which now sets close-delay to max, and/or lower the radio `SQL`), FM de-emphasis tilt (`Demodu → DSC` + `W/N → Wide`), tuning offset. Set `CFG_APRS_RX_DIAG 0` for normal use — the console writes cost decode time. |
| `[aprs.raw]` | **provisional** (`CFG_APRS_RX_DIAG`, `w` command) | one-shot raw ADC capture of the next burst, `begin … <12-bit hex, 32 samples/line> … end`. Save the console to a file and decode it off-line with `test/host/test_aprs_wav` for full per-bit instrumentation. |

### Serial console

Type into the USB serial port:

- **`m`** — toggle the level METER. Watch it while turning the radio volume:
  aim for the **hiss** at `rms ~2000-3500`, `clip 0%`, `dc` near mid-scale.
  A beacon burst then reads lower and decodes.
- **`s`** — one-shot meter line.
- **`w`** — (APRS mode, `CFG_APRS_RX_DIAG`) arm a one-shot raw-ADC capture of the
  next burst; it prints as `[aprs.raw]` hex for off-line decoding.
- **`h`** — help.

Set `SARSAT_LOG 0` in `src/decoder_config.h` to silence the log; `CFG_TX_HEXDUMP 1`
to also dump every UART frame + each `[link] ACK` in hex.

## Rebuild

```sh
export PICO_SDK_PATH=~/pico-sdk
cd ..            # rp2040/
cmake -S . -B build -DPICO_BOARD=pico     # or pico2
cmake --build build
```

`sha256.txt` covers the `.uf2` files committed here.

> Not yet run on RP2040 hardware. The decode path is validated on host against
> real beacon recordings — see `../test/host/` and the project README.
