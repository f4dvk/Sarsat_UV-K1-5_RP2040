# Sarsat_UV-K1-5_RP2040

Un coprocesseur externe **RP2040 / RP2350** qui écoute l'audio de réception
406 MHz d'un portatif Quansheng (UV-K1 nouvelle génération PY32F071, ou
UV-K5 V1 DP32G030), **décode les balises de détresse COSPAS-SARSAT 1ʳᵉ
génération (FGB / C/S T.001)**, et renvoie l'identité et la position décodées à
la radio pour affichage sur son écran — dans l'esprit de l'add-on APRS/FT8 de
KD8CEC pour l'UV-K5.

> **Réception / monitoring uniquement.** Ce projet n'émet jamais sur 406 MHz.
> Une vraie alerte de détresse entendue sur 406 MHz doit être signalée aux
> autorités SAR (en France : le CROSS ; à l'international : le système
> COSPAS-SARSAT) — un décodeur de loisir n'est pas un canal d'alerte
> opérationnel.

## Architecture

```
   Quansheng UV-K1 / UV-K5 V1                        RP2040 / RP2350
  ┌───────────────────────────┐                 ┌──────────────────────────┐
  │ VFO ~406.025, FM+AF plat  │  HP / jack      │ ADC0 (GP26) + pont polar. │
  │ (dé-emphase + HPF/LPF AF  │──── audio ─────▶│  → slicer bi-phase-L 400b │
  │  bypassés sur BK4829/4819)│                 │  → syndrome BCH-1/BCH-2   │
  │                           │                 │  → parseur T.001         │
  │ app écran « SARSAT »      │◀── UART 0xABCD ─│ commandes texte 0x06Cx   │
  │  ID / pays / position     │  38400 8N1 GP4  │                          │
  └───────────────────────────┘                 └──────────────────────────┘
```

- **Matériel :** la [C-Board (DSP-Board) pour UV-K5](https://www.hamskey.com/2024/03/c-board-for-uv-k5.html)
  de KD8CEC, *Basic Version* — une **RP2040-Zero**, l'audio HP à travers un
  condo série de `0,1 µF` et un écrêteur à deux `1N4148` vers **GP26** (ADC0),
  série sur **GP0/GP1** (UART0), carte alimentée par le rail ~3,3 V de la radio.
  Voir [rp2040/src/decoder_config.h](rp2040/src/decoder_config.h) pour le
  brochage complet et les mises en garde électriques de KD8CEC (audio HP ≈ 8 V
  vs ADC 3,3 V — garder le volume bas ; correction possible d'un brown-out à
  l'intérieur de la radio).
- **Entrée audio :** radio en **discriminateur FM** avec la chaîne AF aplatie
  (dé-emphase RX + HPF300 + LPF3k bypassés, FI WIDE) pour que la ligne HP porte
  les données bi-phase-L FGB. *Pas* un mode « RAW » bande de base/BLU — celui-ci
  bat le porteur 406 MHz en un sifflement continu.
- **Sortie texte :** la trame série Quansheng `AB CD … DC BA` à **38400 8N1**
  (C-Board GP0 = UART0 TX vers le RX série de la radio), nouvelles commandes
  applicatives `0x06C0..0x06CF` (voir [docs/protocol.md](docs/protocol.md)).
  C'est *notre* protocole sur les broches UART de la C-Board — sans rapport avec
  le lien propriétaire C-Board↔firmware-CEC de KD8CEC. Le brochage de la C-Board
  est confirmé par analyse statique du `cboard_v032.uf2` de KD8CEC (son firmware
  d'origine utilise 57600 ; on remplace les deux bouts et on standardise sur
  38400).

Les balises 2ᵉ génération (SGB / T.018 / DSSS-OQPSK) sont **hors périmètre sur le
RP2040** : c'est un algorithme IQ (2,4576 MHz complexe, FFT de 131072 points,
buffers de 25–30 Mo) et un discriminateur FM détruit la phase OQPSK. Le parseur
de trame 2G + BCH(250,202) portable sur MCU est conservé non compilé sous
`vendor/decode_sarsat/` pour un éventuel front-end IQ futur.

## État

| Composant | État |
|---|---|
| Décodeur 1G RP2040 (`rp2040/`) | **fonctionne sur matériel** (C-Board / RP2040-Zero) : décode de vraies balises FGB y compris signal faible, tests hôte OK. `.uf2` pré-compilés `rp2040/bin/sarsat_rp2040-{pico,pico2}.uf2`. |
| Lien radio ↔ RP2040 (`docs/protocol.md`) | **bidirectionnel** — RP2040 → UV-K5 `0x06Cx` à 38400 ; l'UV-K5 répond `ID\|0x8000` (ACK + un statut HELLO avec VFO / modulation / fréquence RX). Le RP2040 le logge en `[link]`. |
| **Firmware UV-K5 V1 (base KD8CEC)** | **fonctionne sur matériel** — `firmware/uv-k5v1-kd8cec/`, `ENABLE_SARSAT`, F+8 / auto-ouverture, discriminateur FM + AF plat REG_2B, reçoit sur le VFO sélectionné avec squelch forcé ouvert, affiche les 6 lignes décodées. Ajoute aussi le tracker APRS (TX + RX via la C-Board), l'entrée GPS et un écran VFO principal redessiné — voir `firmware/uv-k5v1-kd8cec/integration.md`. |
| Firmware UV-K1 / UV-K5 V3 (base F4HWN) | **écran SARSAT validé sur matériel** (affichage classique F4HWN) — `firmware/uv-k1-k5v3/`. Réglage de gain C-Board fait (potentiomètre au max). Tracker APRS/GPS/canal dédié **compilé, 0 warning, pas encore testé sur matériel**. Voir `firmware/uv-k1-k5v3/patch/integration.md`. |

### Notes de terrain

- **Le pont de polarisation ADC est obligatoire.** La C-Board Basic Version n'en
  a pas ; ajoute un pont (2× ~100-220 kΩ, 3V3 / GND) sur GP26 pour que l'ADC au
  repos lise à mi-échelle (`[lvl] dc` ~ milieu, pas ~0). Sans lui l'audio est
  écrêté demi-onde et rien ne décode.
- Règle le volume radio pour qu'une fenêtre de balise logge `[burst] rms` bien
  sous `CFG_DECODE_MAX_RMS` (4000) tandis que le souffle FM inter-salves reste
  au-dessus (`[noise]`). Ajuste `CFG_DECODE_MAX_RMS` sur l'écart entre les deux.
- Pourquoi l'APRS marche sur la même carte avec le firmware de KD8CEC mais qu'ici
  la correction de polarisation a été nécessaire : l'AFSK ne demande que
  ton/timing et ignore l'écrêtage ; le slicer par corrélation d'amplitude
  bi-phase-L a besoin d'une forme d'onde bipolaire propre.

## Compiler — firmware RP2040

```sh
export PICO_SDK_PATH=~/pico-sdk           # Pico SDK 2.x
cd rp2040
cmake -S . -B build -DPICO_BOARD=pico     # ou -DPICO_BOARD=pico2 pour RP2350
cmake --build build
# -> build/sarsat_rp2040.uf2
```

Le slicer de bits est en point fixe (int64) par défaut pour tourner sur le
RP2040 sans FPU. Sur RP2350 tu peux configurer `-DSARSAT_SLICER_DOUBLE=ON`.

## Tester — validation sur hôte (sans matériel)

```sh
cd rp2040/test/host
make                 # compile test_decode + test_slicer
make parity          # décodeur T.001 porté vs moricef dec406_hex amont
make check           # parité + parité slicer fixe-vs-double
./test_slicer some_beacon.wav [rate]   # chemin de décodage complet sur un vrai enregistrement
```

`make parity` a besoin de la référence amont
([`github.com/moricef/Decode_sarsat_406_v1g_v2g`](https://github.com/moricef/Decode_sarsat_406_v1g_v2g))
compilée une fois — clonée à côté de ce dépôt puis `make` dedans (produit
`build/dec406_hex`), ou `make parity UPSTREAM=/chemin/vers/Decode_sarsat_406_v1g_v2g`.

Ce que les tests hôte prouvent actuellement :

- **Parité du décodeur** — le parseur T.001 de ~1400 lignes + le port BCH
  produisent des rapports bit à bit identiques à
  `moricef/Decode_sarsat_406_v1g_v2g` sur les trames FGB de test fournies
  (ELT-DT, RLS, chemin coordonnées invalides).
- **Conversion du slicer** — `slicer_run_fixed` (int64) et `slicer_run_double`
  concordent bit à bit à 12/16/22,05/48 kHz sur des salves synthétiques.

- **Vrais enregistrements** — `./test_slicer <file.wav>` décode de vraies
  captures 406 MHz démodulées FM (PCM 16 bits, p. ex. un enregistrement 32 kHz
  de la balise test ELT-DT standard) au **même** hex ID / pays / position
  composite que le `dec406_audio` amont, avec parité slicer fixe==double.

Il reste besoin de matériel RP2040 : le front-end ADC (gain / seuils de
détection de salve dans `decoder_config.h`, calés pour l'écrêteur AC de la
C-Board sans résistance de polarisation) et le lien série vers la radio. Le
générateur bi-phase-L synthétique de `test_slicer.c` n'est qu'un substitut
approximatif pour le stress test fixe-vs-double — les vrais enregistrements sont
la référence.

## Provenance & licence

Le code original de ce projet (patches firmware, modules RP2040 propres,
`docs/`, `tools/`, `rp2040/test/`) est sous **Apache‑2.0** — voir `LICENSE` et
`NOTICE`.

Le dépôt agrège aussi du code amont sous d'autres licences (Apache‑2.0 pour les
deux firmwares Quansheng, BSD‑3‑Clause pour le démodulateur AFSK porté de
`pico_tnc`, MIT/CC‑BY‑NC‑SA pour le décodeur SARSAT porté de
`moricef/Decode_sarsat_406_v1g_v2g`). **`CREDITS.md`** en fait l'inventaire
complet.

> ⚠️ Le décodeur SARSAT 1G (`rp2040/src/dec406*.c`) a une licence amont
> contradictoire (MIT côté fichier `LICENSE`, *CC BY‑NC‑SA* côté en‑têtes
> source, cœur `dec406_v7` de **F4EHY**, 2020). Tant que ce n'est pas clarifié
> avec les auteurs amont, **traiter tout le projet comme radioamateur /
> éducatif non commercial**.
