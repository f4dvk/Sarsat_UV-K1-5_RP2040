# Crédits et licences des composants

Ce dépôt agrège du code d'origines différentes, sous des licences différentes.
Chaque composant conserve la licence de son projet amont ; cette page en fait
l'inventaire.

## Code amont porté ou modifié

| Composant | Emplacement ici | Amont | Licence amont |
|---|---|---|---|
| Firmware Quansheng UV‑K5 V1 (base des patches) | `firmware/uv-k5v1-kd8cec/patch/*.diff` (patches contre KD8CEC `uvk5cec-0.3q`) | [github.com/kd8cec/uv-k5-firmware-cec](https://github.com/kd8cec/uv-k5-firmware-cec) — lignée DualTachyon / egzumer | **Apache‑2.0** |
| Firmware Quansheng UV‑K1 / UV‑K5 V3 (base des patches) | `firmware/uv-k1-k5v3/patch/*` (patches contre `uv-k1-k5v3-firmware-custom`) | egzumer / F4HWN, lignée DualTachyon | **Apache‑2.0** |
| Démodulateur AFSK 1200 Bell‑202 + dé‑framing AX.25 | `rp2040/src/aprs_rx.c` / `aprs_rx.h` (portage DSP entier) | [github.com/JN1DFF/pico_tnc](https://github.com/JN1DFF/pico_tnc) (bell202.c / decode.c / filter.c) | **BSD‑3‑Clause** — Copyright (c) 2021 Kazuhisa Yokota, JN1DFF |
| Décodeur de trame COSPAS‑SARSAT 1ʳᵉ génération (T.001) + BCH | `rp2040/src/dec406.c` / `dec406_v1g.c` / `dec406_v1g.h` / `country_codes.c`, `vendor/decode_sarsat/` | [github.com/moricef/Decode_sarsat_406_v1g_v2g](https://github.com/moricef/Decode_sarsat_406_v1g_v2g) — cœur `dec406_v7` de **F4EHY** (2020) | **⚠️ ambigu — voir ci‑dessous** |
| SDK Raspberry Pi Pico | dépendance de build (non incluse) | [github.com/raspberrypi/pico-sdk](https://github.com/raspberrypi/pico-sdk) | BSD‑3‑Clause |

### ⚠️ Licence du décodeur SARSAT

Le dépôt amont `moricef/Decode_sarsat_406_v1g_v2g` est **contradictoire** :

- le fichier `LICENSE` du dépôt est **MIT** (Copyright (c) 2026 Fabien Morel) —
  copie conservée ici dans `vendor/decode_sarsat/LICENSE.moricef-upstream` ;
- les en‑têtes des fichiers source (`dec406_v1g.c`, `dec406.c`) portent une
  mention **« Licence Creative Commons CC BY‑NC‑SA »**, cœur de code attribué à
  **F4EHY** (2020).

CC BY‑NC‑SA est une licence **non commerciale** et **non recommandée pour du
logiciel**. Tant que cette contradiction n'est pas levée avec les auteurs
amont (moricef / F4EHY), **ce portage doit être considéré comme réservé à un
usage amateur / éducatif non commercial**, et il ne peut pas être qualifié
d'« open source » au sens de l'OSI.

## Code original de ce projet

Le reste — les patches firmware eux‑mêmes, et les modules RP2040
`main.c`, `kiss.c`, `gps_nmea.c`, `aprs_digi.c`, `aprs_parse.c`,
`quansheng_frame.c`, `sarsat_decoder.c`, `audio_slicer.c` (réécriture en point
fixe), `display_min.c`, ainsi que `docs/`, `tools/`, `rp2040/test/` — est
écrit pour ce projet.

Voir `LICENSE` pour les conditions applicables à cette partie.

## Périmètre

Réception / surveillance uniquement. Ce firmware n'émet jamais sur 406 MHz.
Toute alerte de détresse réelle doit être relayée aux autorités compétentes
(CROSS / COSPAS‑SARSAT) ; le projet n'a aucune vocation opérationnelle.
