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
| Décodeur de trame COSPAS‑SARSAT 1ʳᵉ génération (T.001) + BCH | `rp2040/src/dec406.c` / `dec406_v1g.c` / `dec406_v1g.h` / `country_codes.c`, `vendor/decode_sarsat/` | [github.com/moricef/Decode_sarsat_406_v1g_v2g](https://github.com/moricef/Decode_sarsat_406_v1g_v2g) — cœur `dec406_v7` de **F4EHY** (2020) | **MIT** — Copyright (c) 2026 Fabrice Morel (voir ci‑dessous) |
| Décodeurs radiosonde DFM / M10 / M20, vérification croisée RS41 | `rp2040/src/sonde_dfm.c` / `sonde_m10.c` / `sonde_rs41.c` et en‑têtes associés | [github.com/projecthorus/radiosonde_auto_rx](https://github.com/projecthorus/radiosonde_auto_rx) / [github.com/rs1729/RS](https://github.com/rs1729/RS) (auteur **zilog80**) | **GPL‑3.0** — voir la notice en en‑tête de chaque fichier concerné |
| SDK Raspberry Pi Pico | dépendance de build (non incluse) | [github.com/raspberrypi/pico-sdk](https://github.com/raspberrypi/pico-sdk) | BSD‑3‑Clause |

### Licence du décodeur SARSAT

Le fichier `LICENSE` du dépôt amont `moricef/Decode_sarsat_406_v1g_v2g` est
**MIT** (Copyright (c) 2026 Fabrice Morel) — copie conservée ici dans
`vendor/decode_sarsat/LICENSE.moricef-upstream`. C'est cette licence qui régit
le portage présent dans ce dépôt.

Des révisions amont **antérieures** portaient dans l'en‑tête de leurs fichiers
source une mention *« Licence Creative Commons CC BY‑NC‑SA »* ; l'amont est
désormais **MIT partout** (fichier `LICENSE` + en‑têtes de source). Les
en‑têtes des fichiers portés ici sont alignés là‑dessus, l'attribution
F4EHY (2020) / Fabrice Morel (2026) étant conservée. En cas de réutilisation,
vérifie l'état courant de la licence sur le dépôt amont.

### Licence des décodeurs radiosonde (DFM / M10 / M20, vérification RS41)

`radiosonde_auto_rx` et `rs1729/RS` (auteur **zilog80**) sont **GPL‑3.0**.
Jusqu'au 2026‑09‑11 ce dépôt était Apache‑2.0 et ces décodeurs (DFM, M10/M20)
étaient volontairement réimplémentés depuis la seule documentation publique
(sigidwiki, r00t.cz, écrits indépendants) et l'analyse de captures IQ/audio
réelles, sans lire le code GPL, pour ne pas contaminer la licence Apache‑2.0
du dépôt. **Décision explicite de l'utilisateur (2026‑09‑11)** : le dépôt
entier repasse en GPL‑3.0 pour permettre le portage direct depuis ces deux
décodeurs de référence. Chaque fichier concerné (`sonde_dfm.*`, `sonde_m10.*`,
et les parties de `sonde_rs41.*` vérifiées/complétées après coup) porte sa
propre notice de portage en en‑tête, avec le fichier source amont précis.
Le RS41 (`sonde_rs41.c`), initialement écrit depuis la doc publique
(bazjo/RS41_Decoding) avant ce changement, a été vérifié après coup contre
la référence GPL — les emprunts effectifs (s'il y en a eu) sont documentés
dans son en‑tête.

## Code original de ce projet

Le reste — les patches firmware eux‑mêmes, et les modules RP2040
`main.c`, `kiss.c`, `gps_nmea.c`, `aprs_digi.c`, `aprs_parse.c`,
`quansheng_frame.c`, `sarsat_decoder.c`, `audio_slicer.c` (réécriture en point
fixe), `display_min.c`, `sonde_demod.c`, `sonde_sync.c`, ainsi que `docs/`,
`tools/`, `rp2040/test/` — est écrit pour ce projet.

Voir `LICENSE` pour les conditions applicables à cette partie (GPL‑3.0
depuis le 2026‑09‑11, Apache‑2.0 avant).

## Périmètre

Réception / surveillance uniquement. Ce firmware n'émet jamais sur 406 MHz.
Toute alerte de détresse réelle doit être relayée aux autorités compétentes
(CROSS / COSPAS‑SARSAT) ; le projet n'a aucune vocation opérationnelle.
