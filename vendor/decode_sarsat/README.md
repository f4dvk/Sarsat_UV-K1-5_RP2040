# vendor/decode_sarsat

Frozen copies of upstream material from
**github.com/moricef/Decode_sarsat_406_v1g_v2g** that is **not compiled** into
the RP2040 firmware but kept for reference / future work.

| File | Why it's here |
|---|---|
| `dec406_v2g.c`, `dec406_v2g.h` | 2nd-generation (SGB / T.018) frame parser + BCH(250,202) (GF(2⁸) Berlekamp–Massey + Chien). Integer-only, no FFT, no malloc — **portable as-is** to the RP2040, but useless without a 2G demodulator, which is not feasible from FM audio. Wire this in if an IQ front-end is ever added. |
| `audio_capture.c.orig`, `audio_capture.h.orig` | The original F4EHY bit slicer, before the fixed-point rewrite in `rp2040/src/audio_slicer.c`. Reference for behaviour comparison. |

The active, ported 1G path lives in `rp2040/src/` (`dec406.c`, `dec406_v1g.c`,
`audio_slicer.c`, `country_codes.c`, `display_min.c`).
