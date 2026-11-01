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
  │  ID / pays / position     │ 230400 8N1 GP4  │                          │
  └───────────────────────────┘                 └──────────────────────────┘
```

- **Matériel :** la [C-Board (DSP-Board) pour UV-K5](https://www.hamskey.com/2024/03/c-board-for-uv-k5.html)
  de KD8CEC, *Basic Version* — une **RP2040-Zero**, l'audio HP à travers un
  condo série (`1 µF` après modif, `0,1 µF` d'origine) + un pont `2× 22 kΩ` et un
  écrêteur à deux `1N4148` vers **GP26** (ADC0), série sur **GP0/GP1** (UART0),
  carte alimentée par le rail ~3,3 V de la radio.
  Voir [rp2040/src/decoder_config.h](rp2040/src/decoder_config.h) pour le
  brochage complet et les mises en garde électriques de KD8CEC (audio HP ≈ 8 V
  vs ADC 3,3 V — garder le volume bas ; correction possible d'un brown-out à
  l'intérieur de la radio).
- **Entrée audio :** radio en **discriminateur FM** avec la chaîne AF aplatie
  (dé-emphase RX + HPF300 + LPF3k bypassés, FI WIDE) pour que la ligne HP porte
  les données bi-phase-L FGB. *Pas* un mode « RAW » bande de base/BLU — celui-ci
  bat le porteur 406 MHz en un sifflement continu.
- **Sortie texte :** la trame série Quansheng `AB CD … DC BA` à **230400 8N1**
  (C-Board GP0 = UART0 TX vers le RX série de la radio), nouvelles commandes
  applicatives `0x06C0..0x06CF` (voir [docs/protocol.md](docs/protocol.md)).
  C'est *notre* protocole sur les broches UART de la C-Board — sans rapport avec
  le lien propriétaire C-Board↔firmware-CEC de KD8CEC. Le brochage de la C-Board
  est confirmé par analyse statique du `cboard_v032.uf2` de KD8CEC (son firmware
  d'origine utilise 57600 ; on remplace les deux bouts — 38400 au départ, puis
  230400 depuis le 26/09/2026 pour le streaming SSTV temps réel, voir
  `rp2040/src/decoder_config.h`).

Les balises 2ᵉ génération (SGB / T.018 / DSSS-OQPSK) sont **hors périmètre sur le
RP2040** : c'est un algorithme IQ (2,4576 MHz complexe, FFT de 131072 points,
buffers de 25–30 Mo) et un discriminateur FM détruit la phase OQPSK. Le parseur
de trame 2G + BCH(250,202) portable sur MCU est conservé non compilé sous
`vendor/decode_sarsat/` pour un éventuel front-end IQ futur.

## État

| Composant | État |
|---|---|
| Décodeur 1G RP2040 (`rp2040/`) | **fonctionne sur matériel** (C-Board / RP2040-Zero) : décode de vraies balises FGB y compris signal faible, tests hôte OK. `.uf2` pré-compilés `rp2040/bin/sarsat_rp2040-{pico,pico2}.uf2`. |
| Lien radio ↔ RP2040 (`docs/protocol.md`) | **bidirectionnel** — RP2040 → UV-K5 `0x06Cx` à 230400 ; l'UV-K5 répond `ID\|0x8000` (ACK + un statut HELLO avec VFO / modulation / fréquence RX). Le RP2040 le logge en `[link]`. |
| **Firmware UV-K5 V1 (base KD8CEC)** | **fonctionne sur matériel** — `firmware/uv-k5v1-kd8cec/`, `ENABLE_SARSAT`, écran de lancement **F+7** (A APRS-RP / B SARSAT-RP / C SONDE-RP) + auto-ouverture de SARSAT sur trame décodée, discriminateur FM + AF plat REG_2B, reçoit sur le VFO sélectionné avec squelch forcé ouvert, affiche les 6 lignes décodées. Ajoute aussi le tracker APRS (TX + RX via la C-Board), l'entrée GPS et un écran VFO principal redessiné — voir `firmware/uv-k5v1-kd8cec/integration.md`. |
| Firmware UV-K1 / UV-K5 V3 (base F4HWN) | **remplacé par f4hwn-v6** (même matériel UV-K1/UV-K5 V3) — figé, conservé pour référence (`firmware/uv-k1-k5v3/`). Écran SARSAT validé sur matériel (affichage classique F4HWN) avant ce remplacement. |
| **Firmware f4hwn-v6 (base F4HWN v6, UV-K1/UV-K5 V3)** | **firmware principal pour le matériel UV-K1/UV-K5 V3** — remplace la V3 ci-dessus, vu la stabilité acquise. `firmware/f4hwn-v6/`. Écran de lancement **F+7** (menu d'overlay-apps F4HWN v6) avec quatre entrées intégrées en tête de liste : A APRS-RP, B SARSAT-RP, C SSTV-RP, D SONDE-RP — les emplacements d'overlay suivants gardent leur numéro physique. Aucun F1/F2 assignable ni combo fixe pour ces quatre écrans. Auto-ouverture de SARSAT/Sonde sur trame décodée. |

### Notes de terrain

- **Deux modifs obligatoires sur la C-Board Basic Version** (elle ne les a pas) :
  - **pont de polarisation 2× 22 kΩ** (3V3 → nœud ADC, nœud → GND) sur GP26,
    pour que l'ADC au repos lise à mi-échelle (`[lvl] dc` ~ milieu, pas ~0) —
    sans lui l'audio est écrêté demi-onde et rien ne décode. **Pas** 100 kΩ :
    l'impédance d'entrée dynamique de l'ADC RP2040 (~100 kΩ) fausserait la mesure.
  - **condensateur de liaison 1 µF** au lieu de 0,1 µF : avec R1‖R2 = 11 kΩ, un
    0,1 µF fait un passe-haut à ~145 Hz qui rogne le bas du spectre Manchester
    400 bps ; 1 µF → coupure ~14 Hz.

  Détails, tableau des valeurs et choix du condensateur dans
  [docs/hardware.md](docs/hardware.md).
- **Raccourcis clavier** (ouverture manuelle des écrans) : sur **les trois
  firmwares (V1, V3, f4hwn-v6)**, un point d'entrée unique — l'écran de
  lancement **F+7**, qui remplace les anciens raccourcis fixes F+0/F+5/F+8
  **et** le mécanisme F1/F2 assignable (menu `F1Shrt`/`F1Long`/`F2Shrt`/
  `F2Long`) : plus aucune autre façon d'ouvrir SARSAT, APRS, Sonde ou SSTV.
  F+7 liste les écrans disponibles par lettre (A, B, C, D sur V3 et
  f4hwn-v6 — V1 n'a pas SSTV, donc seulement A/B/C) ; **MENU** ouvre l'écran
  sélectionné, **HAUT/BAS** change la sélection, **EXIT** revient au menu
  puis quitte. L'écran SARSAT s'ouvre aussi tout seul, sur les trois
  firmwares, à l'arrivée d'une trame décodée.
- **Réglage du niveau audio** (une fois, mémorisé) : mets le **potentiomètre de
  volume de la radio au maximum** et laisse-le là ; accorde la radio sur une
  **fréquence UHF** où l'on entend le souffle FM (squelch ouvert) ; ouvre
  l'écran **SARSAT** (**F+7 → B**, sur les trois firmwares), appuie sur
  **`5`** pour passer en vue niveau, puis ajuste avec **HAUT /
  BAS** jusqu'à ce que le **souffle** amène la barre à mi-échelle (« reglage
  correct »). Une salve de balise, plus faible, décode alors sans y
  retoucher. Le curseur pilote le gain AF de la C-Board côté BK4819 (sauvé en
  EEPROM), indépendamment du potentiomètre.
- Pourquoi l'APRS marche sur la même carte avec le firmware de KD8CEC mais qu'ici
  la correction de polarisation a été nécessaire : l'AFSK ne demande que
  ton/timing et ignore l'écrêtage ; le slicer par corrélation d'amplitude
  bi-phase-L a besoin d'une forme d'onde bipolaire propre.
- **Notice d'utilisation** (destinée aux opérateurs, pas aux développeurs) :
  [docs/notice_utilisation.pdf](docs/notice_utilisation.pdf), générée depuis
  [docs/notice_utilisation.html](docs/notice_utilisation.html) (source
  éditable — `google-chrome --headless --disable-gpu --no-sandbox
  --no-pdf-header-footer --print-to-pdf=docs/notice_utilisation.pdf
  docs/notice_utilisation.html` pour la régénérer après modification).

## Compiler — firmware radio f4hwn-v6 (UV-K1 / UV-K5 V3)

```
cd firmware/f4hwn-v6
./compile-firmware.sh Labs \
  -DENABLE_FEAT_F4HWN_OVERLAY_APPS=ON \
  -DENABLE_APRS=ON -DENABLE_SARSAT=ON -DENABLE_SSTV=ON -DENABLE_SONDE=ON \
  -DENABLE_BYP_RAW_DEMODULATORS=ON \
  -DENABLE_SPECTRUM=OFF -DENABLE_FMRADIO=OFF \
  -DENABLE_FEAT_F4HWN_K5VIEWER=OFF -DENABLE_FEAT_F4HWN_RXTX_LOG=OFF \
  -DENABLE_FEAT_F4HWN_RXTX_LOG_K5VIEWER=OFF \
  -DENABLE_FEAT_F4HWN_RESCUE_OPS=OFF -DENABLE_AIRCOPY=OFF \
  -DENABLE_FEAT_F4HWN_EXT_FLASH_RW=ON \
  -DENABLE_FEAT_F4HWN_OVERLAY_BEAM=ON -DENABLE_FEAT_F4HWN_OVERLAY_INFO=ON \
  -DENABLE_SWD=OFF -DENABLE_FEAT_F4HWN_QRCODE=OFF
# -> build/Labs/f4hwn.labs.bin  (2026-11-01 : 118 840 o, 98,35 % de flash --
#    1 992 o de marge restante, voir plus bas pourquoi EXT_FLASH_RW/OVERLAY_BEAM/
#    OVERLAY_INFO sont requis, et pourquoi SWD/QRCODE sont coupés mais pas LOGO)
```

- Preset **`Labs`** et non `Fusion` depuis le 2026-11-01 : UV Studio n'affiche
  l'onglet **Apps** (installation des apps overlay) que si le firmware réussit
  DEUX sondes UART au démarrage de la connexion -- `apps` (commande `0x0730`,
  app slot info) ET `externalFlash` (commande `0x0738`, lecture brute) --
  code source : `uvstudio/js/flash.js`, `detectNavigationCapabilities()` /
  `applyNavigationCapabilities()` (dépôt `armel/armel.github.io`). Si l'une des
  deux échoue, UV Studio remet **les deux** capacités à `false` (sauf pour les
  firmwares `Service`/v7+, hors sujet ici) et affiche le badge "Labs only" --
  même si les apps overlay fonctionnent réellement sur la radio. D'où
  `ENABLE_FEAT_F4HWN_EXT_FLASH_RW=ON` obligatoire ci-dessus, pas seulement
  `EDITION_STRING=Labs` : sans lui, la sonde `externalFlash` (commandes UART
  `0x0738-0x073F`, dump/restore brut de la flash externe + factory reset côté
  UV Studio) échoue et désactive aussi l'onglet Apps par effet de bord, même si
  `ENABLE_FEAT_F4HWN_OVERLAY_APPS` (commandes `0x0730-0x0737`) est bien actif.
- `ENABLE_FEAT_F4HWN_OVERLAY_BEAM` / `ENABLE_FEAT_F4HWN_OVERLAY_INFO` : une fois
  l'onglet Apps débloqué, certaines apps du catalogue officiel (dépôt
  `armel/uv-k1-k5v3-firmware-custom`, `archive/apps/v<base>/`) restaient
  refusées au lancement sur la radio avec `APP_ERR_CAP` ("non supportée par ce
  firmware", `app_overlay.c:967` : `required_caps & ~APP_AVAILABLE_CAPS`).
  Vérifié en lisant l'en-tête réel (`required_caps`, offset 56) des `.app`
  officiels : `SystemInfo.app` et `SSTV.app` demandent `APP_CAP_SYSINFO`
  (0x08, dépend de `OVERLAY_INFO`), `Scan406.app` demande `APP_CAP_BEAM2`
  (0x10, dépend de `OVERLAY_BEAM`) -- tous deux activés ci-dessus pour ces
  trois apps (`EPIRB406.app`, lui, ne demande aucune capacité et marchait déjà).
  Coût mesuré des deux flags : ~836 o (118 476 → 119 312 o).
- `ENABLE_SWD` / `ENABLE_FEAT_F4HWN_QRCODE` : coupés le 2026-11-01 pour regagner
  de la marge flash (mesuré : 484 o pour QRCODE, SWD lui-même proche de 0 o car
  c'est surtout une config GPIO), sans perte fonctionnelle pour ce projet --
  sonde de debug SWD inutile sur un binaire de release, QR code de canal
  purement cosmétique.
- `ENABLE_FEAT_F4HWN_LOGO` / `LOGO_SAV` restent **`ON`**, volontairement :
  d'abord coupés par erreur en les croyant purement cosmétiques (logo de
  démarrage), puis rétablis -- le logo personnalisé uploadé via UV Studio
  (`LOGO_FLASH_ADDR = 0x011000`, flash externe) sert aussi d'image source à
  l'app SSTV du catalogue officiel pour transmettre une image personnalisée.
  Sans confusion avec le sélecteur photo/logo ADRASEC de notre propre écran
  SSTV résident (`App/app/sstv_tx.c`), qui lui n'a aucun rapport : deux images
  de test figées, compilées en dur côté RP2040 (`rp2040/src/sstv_master_image*.h`),
  sans lien avec `ENABLE_FEAT_F4HWN_LOGO`.
  D'autres extras F4HWN génériques coûtent plus cher mais touchent à de vrais
  conforts radio (scan Progress/Faster/RSSI/Subaudible : ~4 576 o ; menu
  catégorisé/Action Picker/Full Watch : ~3 420 o ; VOX/PMR/GMRS-FRS-MURS/Audio
  Scope : ~1 792 o ; Logo/Logo Save : ~1 048 o mais requis pour SSTV.app, voir
  ci-dessus) -- laissés actifs, à couper seulement si la marge redevient
  critique.
- `ENABLE_APRS / SARSAT / SSTV / SONDE` : les quatre écrans RP (`App/CMakeLists.txt`,
  `enable_feature`), **jamais activés par un preset** : sans ces `-D`, le menu
  F+7 existe mais sans A APRS-RP / B SARSAT-RP / C SSTV-RP / D SONDE-RP.
- `ENABLE_FEAT_F4HWN_OVERLAY_APPS` : le menu F+7 lui-même (sinon F+7 = VOX) ;
  exige `MULTIBOOT_OVERLAY`, présent dans `Fusion`/`Labs` mais pas dans `Custom`.
- `ENABLE_BYP_RAW_DEMODULATORS` : `MODULATION_RAW`, utilisé par l'écran Sonde, et
  `MODULATION_BYP`. **Depuis le 2026-10-30, BYP = profil « audio plat »** de
  l'écran Sonde et du mode hôte H28 : AF select 9 (REG_47 = 0x6942, seul moyen
  de couper le filtre de parole ~3 kHz du BK4819, les bits de REG_2B ne
  suffisent pas), FI large REG_43 = 0x7E2C, AFC coupée (rendue en quittant le
  mode). Un VFO ordinaire en BYP livre donc sur le jack l'audio brut qu'il faut
  aux décodeurs externes (UVLink, RP2040) sans ouvrir d'écran dédié. RAW reste
  le profil stock (REG_2B seul, AFC à plage minimale). Fichier :
  `App/driver/bk4829.c`, `BK4819_EnterBypass` / `BK4819_ExitBypass`.
- Extras coupés pour tenir dans les 118 Ko : spectre, radio FM broadcast,
  K5Viewer, journal RX/TX (`Fusion` complet + écrans RP déborde de 14 Ko).
  Docker requis ; sans TTY, envelopper dans `script -q -c "…" /dev/null`.

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

Il reste besoin de matériel RP2040 : le front-end ADC (pont de polarisation
2× 22 kΩ à ajouter + gain / seuils de détection de salve dans
`decoder_config.h`) et le lien série vers la radio. Le
générateur bi-phase-L synthétique de `test_slicer.c` n'est qu'un substitut
approximatif pour le stress test fixe-vs-double — les vrais enregistrements sont
la référence.

## Provenance & licence

Le code original de ce projet (patches firmware, modules RP2040 propres,
`docs/`, `tools/`, `rp2040/test/`) est sous **GPL‑3.0** — voir `LICENSE` et
`NOTICE`. Le dépôt était sous Apache‑2.0 jusqu'au 2026‑09‑11 ; il est repassé
en GPL‑3.0 spécifiquement pour pouvoir porter les décodeurs radiosonde
(DFM, M10/M20, vérification croisée du RS41) depuis les décodeurs C de
référence de `radiosonde_auto_rx`/`rs1729 RS` (GPL‑3.0) — voir `CREDITS.md`
pour l'attribution détaillée.

Le dépôt agrège aussi du code amont sous d'autres licences (toutes
compatibles GPL‑3.0 dans le sens absorption, jamais l'inverse) : **Apache‑2.0**
pour les trois firmwares Quansheng, **BSD‑3‑Clause** pour le démodulateur AFSK
porté de `pico_tnc`, **MIT** pour le décodeur SARSAT porté de
`moricef/Decode_sarsat_406_v1g_v2g` (Copyright (c) 2026 Fabrice Morel ; cœur
`dec406_v7` de **F4EHY**, 2020 — copie du `LICENSE` amont dans
`vendor/decode_sarsat/LICENSE.moricef-upstream`), et **GPL‑3.0** pour les
portions des décodeurs radiosonde portées depuis `radiosonde_auto_rx`/
`rs1729 RS` (auteur zilog80). **`CREDITS.md`** en fait l'inventaire complet.

> Des révisions amont antérieures du décodeur SARSAT portaient une mention
> *CC BY‑NC‑SA* dans l'en‑tête des fichiers ; les en‑têtes des fichiers portés
> ici sont alignés sur le `LICENSE` MIT amont. En cas de réutilisation,
> vérifie l'état courant de la licence sur le dépôt amont.
