# Sarsat_UV-K1-5_RP2040

An external **RP2040 / RP2350** co-processor that listens to the 406 MHz receive
audio of a Quansheng handheld (UV-K1 new-gen PY32F071, or UV-K5 V1 DP32G030),
**decodes COSPAS-SARSAT 1st-generation (FGB / C/S T.001) distress beacons**, and
sends the decoded identity and position back to the radio to show on its screen —
in the spirit of KD8CEC's APRS/FT8 add-on for the UV-K5.

> **Reception / monitoring only.** This project never transmits on 406 MHz.
> A real distress alert heard on 406 MHz must be reported to the SAR
> authorities (in France: CROSS; internationally: the COSPAS-SARSAT system) —
> a hobby decoder is not an operational alerting channel.

## Architecture

```
   Quansheng UV-K1 / UV-K5 V1                        RP2040 / RP2350
  ┌───────────────────────────┐                 ┌──────────────────────────┐
  │ VFO ~406.025, FM+flat AF  │  speaker / jack │ ADC0 (GP26) + bias divider│
  │ (de-emphasis + AF HPF/LPF │──── audio ─────▶│  → bi-phase-L slicer 400b │
  │  bypassed on BK4829/4819) │                 │  → BCH-1/BCH-2 syndrome   │
  │                           │                 │  → T.001 parser          │
  │ "SARSAT" screen app       │◀── UART 0xABCD ─│ 0x06Cx text commands     │
  │  ID / country / position  │  38400 8N1 GP4  │                          │
  └───────────────────────────┘                 └──────────────────────────┘
```

- **Hardware:** KD8CEC's [C-Board (DSP-Board) for UV-K5](https://www.hamskey.com/2024/03/c-board-for-uv-k5.html),
  *Basic Version* — an **RP2040-Zero**, speaker audio through a `0.1 µF` series
  cap and a two-`1N4148` clamp into **GP26** (ADC0), serial on **GP0/GP1**
  (UART0), board powered from the radio's ~3.3 V rail. See
  [rp2040/src/decoder_config.h](rp2040/src/decoder_config.h) for the full pin map
  and KD8CEC's electrical caveats (≈8 V speaker audio vs 3.3 V ADC — keep the
  volume low; possible brown-out fix inside the radio).
- **Audio in:** radio using its **FM discriminator** with the AF chain flattened
  (RX de-emphasis + HPF300 + LPF3k bypassed, WIDE IF) so the speaker line
  carries the bi-phase-L FGB data. *Not* a baseband/SSB "RAW" mode — that beats
  the 406 MHz carrier into a constant whistle.
- **Text out:** the Quansheng `AB CD … DC BA` serial frame at **38400 8N1**
  (C-Board GP0 = UART0 TX to the radio serial RX), new application commands
  `0x06C0..0x06CF` (see [docs/protocol.md](docs/protocol.md)). This is *our*
  protocol on the C-Board's UART pins — unrelated to KD8CEC's own proprietary
  C-Board↔CEC-firmware link. The C-Board pin map is confirmed by static analysis
  of KD8CEC's `cboard_v032.uf2` (its stock firmware uses 57600; we replace both
  ends and standardise on 38400).

2nd-generation beacons (SGB / T.018 / DSSS-OQPSK) are **out of scope on the
RP2040**: that is an IQ algorithm (2.4576 MHz complex, 131072-point FFTs,
25–30 MB buffers) and an FM discriminator destroys the OQPSK phase. The
MCU-portable 2G frame parser + BCH(250,202) is kept unbuilt under
`vendor/decode_sarsat/` for a possible future IQ front-end.

## Status

| Component | State |
|---|---|
| RP2040 1G decoder (`rp2040/`) | **working on hardware** (C-Board / RP2040-Zero): decodes real FGB beacons incl. weak signal, host tests pass. Prebuilt `rp2040/bin/sarsat_rp2040-{pico,pico2}.uf2`. |
| Radio ↔ RP2040 link (`docs/protocol.md`) | **two-way** — RP2040 → UV-K5 `0x06Cx` at 38400; UV-K5 replies `ID\|0x8000` (ACK + a HELLO status with VFO / modulation / RX freq). RP2040 logs it as `[link]`. |
| **UV-K5 V1 firmware (KD8CEC base)** | **working on hardware** — `firmware/uv-k5v1-kd8cec/`, `ENABLE_SARSAT`, F+8 / auto-open, FM discriminator + REG_2B flat AF, receives on the selected VFO with squelch forced open, shows the 6 decoded lines. |
| UV-K1 firmware (F4HWN base) | not started (Phase 3) |

### Field notes

- **ADC bias is mandatory.** The C-Board Basic Version has none; add a divider
  (2× ~100-220 kΩ, 3V3 / GND) on GP26 so the idle ADC reads mid-scale
  (`[lvl] dc` ~ mid, not ~0). Without it the audio is half-wave clamped and
  nothing decodes.
- Set the radio volume so a beacon window logs `[burst] rms` well under
  `CFG_DECODE_MAX_RMS` (4000) while the between-burst FM hiss stays above it
  (`[noise]`). Tune `CFG_DECODE_MAX_RMS` to the gap between the two.
- Why APRS works on the same board with KD8CEC's firmware but this needed the
  bias fix: AFSK only needs tone/timing and shrugs off clipping; the bi-phase-L
  amplitude-correlation slicer needs a clean bipolar waveform.

## Build — RP2040 firmware

```sh
export PICO_SDK_PATH=~/pico-sdk           # Pico SDK 2.x
cd rp2040
cmake -S . -B build -DPICO_BOARD=pico     # or -DPICO_BOARD=pico2 for RP2350
cmake --build build
# -> build/sarsat_rp2040.uf2
```

The bit slicer is fixed-point (int64) by default so it runs on the FPU-less
RP2040. On RP2350 you may configure `-DSARSAT_SLICER_DOUBLE=ON`.

## Test — host validation (no hardware)

```sh
cd rp2040/test/host
make                 # build test_decode + test_slicer
make parity          # ported T.001 decoder vs upstream moricef dec406_hex
make check           # parity + slicer fixed-vs-double parity
./test_slicer some_beacon.wav [rate]   # full decode path on a real recording
```

`make parity` needs the upstream reference built once:
`make -C /home/stephane/Decode_sarsat_406_v1g_v2g` (produces `build/dec406_hex`).

What the host tests currently prove:

- **Decoder parity** — the ~1400-line T.001 parser + BCH port produces
  byte-identical reports to `moricef/Decode_sarsat_406_v1g_v2g` on the bundled
  FGB test frames (ELT-DT, RLS, invalid-coordinate path).
- **Slicer conversion** — `slicer_run_fixed` (int64) and `slicer_run_double`
  agree bit-for-bit across 12/16/22.05/48 kHz on synthetic bursts.

- **Real recordings** — `./test_slicer <file.wav>` decodes genuine
  FM-demodulated 406 MHz captures (16-bit PCM, e.g. a 32 kHz recording of the
  standard ELT-DT test beacon) to the **same** hex ID / country / composite
  position as the upstream `dec406_audio`, with fixed==double slicer parity.

Still needs RP2040 hardware: the ADC front-end (gain / burst-detect thresholds
in `decoder_config.h`, tuned for the C-Board's AC-coupled clamp with no bias
resistor) and the serial link to the radio. The synthetic bi-phase-L generator
in `test_slicer.c` is only a rough stand-in for the fixed-vs-double stress test
— real recordings are the reference.

## Provenance & licensing

- 1G decoder core (`rp2040/src/dec406*.c`, `audio_slicer.c`, `country_codes.c`)
  is ported from **github.com/moricef/Decode_sarsat_406_v1g_v2g**. Original
  decoder: **F4EHY** `dec406_v7` (2020). Upstream source-file headers carry a
  *CC BY-NC-SA* notice; the upstream repository `LICENSE` file is *MIT*
  (Fabien Morel, 2026). Treat this port as **non-commercial / educational**
  amateur-radio use pending clarification with the upstream author.
- Quansheng serial frame codec (`quansheng_frame.c`) is reworked from
  `benshi-esp32-sim/src/UvK5Link.h`.
- Project-specific glue (`sarsat_decoder.c`, `main.c`, firmware patches) — same
  terms as `benshi-esp32-sim`.
