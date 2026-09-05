# Firmware UV-K1 / UV-K5 V3 (base F4HWN) — écran SARSAT

Port de l'écran SARSAT du firmware UV-K5 V1 (`firmware/uv-k5v1-kd8cec/`) vers
la base **F4HWN** (`armel/uv-k1-k5v3-firmware-custom`, PY32F071 + BK4829),
qui couvre à la fois l'**UV-K1** nouvelle génération et l'**UV-K5 V3**
(même plateforme matérielle, même binaire).

La C-Board RP2040 et le protocole série (`docs/protocol.md`) sont **inchangés**
— c'est le même lien `AB CD … DC BA` à 38400 8N1, commandes `0x06Cx`. Aucune
modification côté `rp2040/` n'est nécessaire pour ce port.

## État

| Composant | État |
|---|---|
| Écran SARSAT (décodage 406 MHz affiché) | **validé sur matériel**, y compris signal faible — voir `bin/` et `patch/integration.md` (correctif AFC). Ouverture directe par **F+8**, comme sur le V1 (a remplacé rétroéclairage à la demande / inversion de fréquence sur cette touche). |
| Réglage de gain C-Board (potentiomètre volume au max) | fait, sur la vue niveau de l'écran SARSAT (touche `5`, UP/DOWN), partagé avec l'APRS. |
| Tracker APRS (TX/RX, GPS, canal dédié 170) | **compilé, 0 warning** — pas encore testé sur matériel. Ouverture directe par **F+5** (combinaison libre sur ce preset), comme sur le V1. Voir `patch/integration.md`. |
| Icône GPS en barre de statut | **compilé, 0 warning** — pas encore testé sur matériel. |
| Affichage « Icom » (façon V1) | **en cours, par étapes** — bandeau d'en-tête inversé + fréquence en gros chiffres + S-mètre recentré, actif en Main Only **et** en Dual Watch (portée limitée à la vue fréquence au repos pour l'instant, pas encore aux canaux mémoire/NOAA). Objectif final : parité complète avec le V1 (affichage/menu/décodage). Voir `patch/integration.md` et la Phase 6 du plan de suivi. |

## Compiler

```sh
./build.sh [TAG] [PRESET]   # TAG défaut v5.9.0, PRESET défaut Fusion
```

Aucun Docker nécessaire : le script clone `armel/uv-k1-k5v3-firmware-custom`
dans un répertoire temporaire, applique les points d'ancrage (`patch/*.c/.h`
+ quelques insertions `perl` dans `app.c`/`uart.c`/`CMakeLists.txt`), puis
compile avec `cmake --preset <PRESET>` + `arm-none-eabi-gcc` du `PATH`
(13.x testé). Résultat dans `bin/`.

## Flasher

Radio en **mode DFU** (bouton PTT + power, ou selon le modèle), puis :

- **UV Studio** (recommandé, dans le navigateur, aucune installation) :
  <https://armel.github.io/uvstudio/#flash> — charger le `.bin` de `bin/`.
  *Sauvegarde la calibration d'abord* (`#dump-calib`), comme recommandé par
  F4HWN lui-même.
- **ou** l'outil CLI amont : `python3 tools/serialtool/cli.py flash --port
  /dev/ttyUSBx <fichier.bin>` (voir le dépôt `armel/uv-k1-k5v3-firmware-custom`
  pour les options exactes — outil non redistribué ici, cloné par `build.sh`
  dans un répertoire temporaire donc pas conservé après coup ; le récupérer
  depuis l'amont si besoin en CLI).

## Voir aussi

- [`patch/integration.md`](patch/integration.md) — détail des points d'ancrage,
  différences avec le port V1, et ce qui reste à faire (APRS/GPS/canal 170).
- [`../uv-k5v1-kd8cec/`](../uv-k5v1-kd8cec/) — le firmware UV-K5 V1 (KD8CEC)
  dont ce port reprend l'écran SARSAT et le tracker APRS.
- [`../../docs/protocol.md`](../../docs/protocol.md) — protocole radio ↔ RP2040,
  commun aux deux firmwares.
