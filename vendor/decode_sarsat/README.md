# vendor/decode_sarsat

Copies figées de matériel amont provenant de
**github.com/moricef/Decode_sarsat_406_v1g_v2g**, **non compilées** dans le
firmware RP2040 mais conservées pour référence / travaux futurs.

| Fichier | Raison de sa présence |
|---|---|
| `dec406_v2g.c`, `dec406_v2g.h` | Parseur de trame 2ᵉ génération (SGB / T.018) + BCH(250,202) (Berlekamp–Massey + Chien sur GF(2⁸)). Entier pur, sans FFT, sans malloc — **portable tel quel** sur le RP2040, mais inutile sans démodulateur 2G, lequel n'est pas réalisable depuis de l'audio FM. À câbler si un front-end IQ est un jour ajouté. |
| `audio_capture.c.orig`, `audio_capture.h.orig` | Le slicer de bits F4EHY d'origine, avant la réécriture en point fixe dans `rp2040/src/audio_slicer.c`. Référence pour comparer le comportement. |

Le chemin 1G actif et porté vit dans `rp2040/src/` (`dec406.c`, `dec406_v1g.c`,
`audio_slicer.c`, `country_codes.c`, `display_min.c`).
