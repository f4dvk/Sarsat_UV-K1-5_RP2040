# Lien RP2040 ↔ radio — protocole applicatif SARSAT (brouillon, Phase 2)

Couche physique (d'après la C-Board `cboard_v032.uf2` de KD8CEC, analyse statique) :

| ligne | broche | débit | usage |
|------|-----|------|-----|
| lien radio | RP2040 **GP0 = UART0 TX**, **GP1 = UART0 RX** | **38400 8N1** | ce protocole |
| GPS | GP4/GP5 = UART1 | 9600 8N1 | NMEA, non utilisé par le firmware SARSAT |

On **n'utilise pas** le framing propriétaire C-Board de KD8CEC. Le débit est un
choix libre puisque les deux bouts sont à nous : les patches firmware SARSAT
pour l'UV-K1 et l'UV-K5 V1 gardent l'UART à **38400** (le défaut egzumer/F4HWN,
et ce qu'utilise l'outillage de `benshi-esp32-sim`). Le firmware C-Board
d'origine de KD8CEC utilise 57600 — sans objet ici puisqu'il est remplacé.

Transport = la trame UART Quansheng standard (egzumer / F4HWN
`App/app/uart.c`), **RP2040 → radio uniquement** pour l'instant :

```
AB CD | size:u16 LE | <inner> | crc16:u16 LE | DC BA
inner = [ID:u16 LE][data_size:u16 LE][data…]
inner + crc sont masqués par XOR avec la table Obfuscation de 16 octets
```

38400 8N1. Encodeur : `rp2040/src/quansheng_frame.c`.
Le firmware radio (Phase 3/4) ajoute un gestionnaire pour les ID ci-dessous dans
`UART_HandleCommand()` plus une petite app écran « SARSAT ».

| ID | Nom | Charge utile | Signification |
|----|------|---------|-------|
| `0x06CF` | `SARSAT_HELLO` | `proto_ver:u8` | keepalive, ~toutes les 5 s. La radio peut afficher un indicateur de lien. |
| `0x06C0` | `SARSAT_CLEAR` | — | efface le tampon d'écran SARSAT |
| `0x06C1` | `SARSAT_TEXT` | `line:u8, invert:u8, ascii[0..20]` | définit une ligne d'affichage (0 = haut). `invert` = rendu inversé. ASCII uniquement, ≤ 21 glyphes. |
| `0x06C2` | `SARSAT_BEACON` | struct compacte (ci-dessous) | résultat de décodage lisible par la machine ; la radio le met en forme elle-même. Alternative optionnelle à `0x06C1`. |
| `0x06C3` | `SARSAT_LEVEL` | `peak:u16,rms:u16,dc:u16,clip:u8,adcmin:u16,adcmax:u16,verdict:u8` LE | télémétrie de niveau audio pour l'écran de réglage de la radio, ~1/s, sans ACK |
| `0x06D0` | `APRS_CONFIG` | (réservé) radio → RP2040 : call / SSID / path / symbole | pour digipeat / ack — pas encore câblé. La config côté radio vit en EEPROM `0x1D00` (40 o ; **pas** `0x1D50`, que `SETTINGS_SaveSettings()` écrase). |
| `0x06D2` | `APRS_RXTEXT` | `line:u8, ascii[0..18]` | RP2040 → radio : une ligne d'un paquet APRS 144.8 MHz décodé, utilisée seulement quand le champ info n'a **pas** pu être parsé. `line = 0xFF` efface la vue RX ; `line = 0` est l'indicatif source, `1..3` le champ info enroulé. Sans ACK. |
| `0x06D3` | `APRS_RXINFO` | décodé structuré (ci-dessous) | RP2040 → radio : un paquet APRS parsé (symbole, lat/lon, cap/vitesse/altitude, distance+azimut vers l'opérateur, source, nom d'objet, chemin digipeater, texte commentaire/statut/message). La radio le rend façon Kenwood avec une icône symbole et une ligne « Direct » / « Via … ». Sans ACK. |
| `0x06D5` | `APRS_GPS` | `flags:u8, lat_e5:i32, lon_e5:i32, speed_kmh:u16, course_deg:u16, alt_m:i16, sats:u8` LE (16 o) | RP2040 → radio, ~toutes les 3 s : le fix d'un module GPS sur l'en-tête NMEA de la C-Board (UART1 GP5, 9600 8N1, `$GxRMC`/`$GxGGA`). `flags` bit0 = fix valide. Envoyé seulement une fois qu'un module a été vu. La radio l'utilise pour la balise quand le champ **Pos** de son menu APRS est sur **GPS** (sinon elle balise la lat/lon manuelle), et pilote un symbole GPS en barre haute (absent = pas de trames, clignotant = `flags` bit0 à 0, fixe = fix). Le RP2040 utilise aussi son propre fix pour la distance/azimut RX quand il est valide. Sans ACK. |

### Sélection du mode

La C-Board fait tourner le décodeur SARSAT **ou** APRS, jamais les deux, et
choisit d'après la fréquence RX dans la réponse `SARSAT_HELLO` : `144,0–148,0 MHz`
→ APRS (Bell-202 AFSK 1200 + AX.25, `rp2040/src/aprs_rx.c`, porté de JN1DFF
pico_tnc), tout le reste → SARSAT FGB. Le basculement prend effet en un
aller-retour HELLO (~1 s).

`SARSAT_TEXT` est le chemin primaire : toute la mise en forme vit dans le RP2040
(`sarsat_format_lines()` — un champ par ligne, chaînes longues coupées aux mots,
jusqu'à `SARSAT_MAX_LINES` = 14). La radio stocke les lignes et les **fait
défiler** (UV-K5 : HAUT/BAS ; UV-K1 : les touches latérales équivalentes),
7 lignes + un en-tête visibles à la fois. Un décodage pousse `CLEAR` puis les
lignes à la suite ; le RP2040 espace les trames de ~8 ms et la radio vide son
anneau UART dans une boucle `while` pour n'en perdre aucune. `SARSAT_BEACON`
est fourni pour une radio qui veut disposer les champs à sa façon.

### Charge utile `APRS_RXINFO` (little-endian)

```
u8   kind             0 autre, 1 position, 2 objet, 3 statut, 4 message, 5 télémétrie
u8   flags            bit0 has_pos, bit1 has_course/speed, bit2 has_alt, bit3 has_range
char sym_table        '/', '\' ou caractère overlay
char sym_code         code de symbole APRS
i32  lat_e5           latitude  × 1e5  (+ = N)
i32  lon_e5           longitude × 1e5  (+ = E)
u16  course_deg       0..359
u16  speed_kmh
i16  alt_m
u16  dist_hm          distance vers l'opérateur, en hectomètres (0 si flag à 0)
u16  bearing_deg      azimut de la cible depuis l'opérateur
char src[]            indicatif source terminé par NUL
char name[]           nom d'objet/item terminé par NUL ("" pour une position simple)
char via[]            terminé par NUL : les digipeaters qui ont réellement répété
                      la trame (bit H AX.25 posé), séparés par des virgules, alias
                      façon WIDEn/TRACEn retirés ("F8KCS-3,F8KCS-2"). "" = direct.
char text[]           texte commentaire / statut / message terminé par NUL
```

Distance et azimut sont calculés sur le RP2040 (approximation équirectangulaire)
depuis la position propre de l'opérateur, que la radio envoie dans sa réponse
`SARSAT_HELLO` (ci-dessous). Ils sont omis (flag à 0) tant que l'opérateur n'a
pas saisi de position dans l'écran de config APRS.

### Charge utile `SARSAT_BEACON` (little-endian)

```
u8   frame_bits        112 ou 144
u8   protocol           enum ProtocolType (dec406_v1g.h)
u8   is_test            1 = trame test/auto-test
u8   has_position
u16  country_code
i32  lat_1e4            latitude  × 1e4 (0 si pas de position)
i32  lon_1e4            longitude × 1e4
char hex_id[15]         ID de balise COSPAS 15-hex
char ident[24]          chaîne d'identification (tronquée)
```
Total 55 octets.

## Phase 2 — le sens radio → RP2040 (ACK / statut)

Actuellement RP2040 → radio est unidirectionnel. La Phase 2 ajoute une réponse
sur le même fil :

- **ACK** : la radio répond à chaque `0x06Cx` avec ID = `command | 0x8000`,
  charge utile `status:u8` (0 = OK). Permet au RP2040 de logger `[link] radio ACK`
  / détecter un lien mort ou mal câblé.
- **Statut** : la radio répond à `SARSAT_HELLO` (`0x06CF | 0x8000`) avec
  `vfo:u8, modulation:u8, rx_freq:u32 (unités de 10 Hz), sarsat_screen:u8,
  proto_ver:u8, my_lat_e5:i32, my_lon_e5:i32` (16 octets ; la position finale est
  0/absente sur les vieux builds radio — le RP2040 accepte aussi une réponse de
  8 octets). `my_lat/lon` est la position APRS manuelle de la radio, ou son fix
  GPS en direct quand le champ **Pos** du menu APRS est sur **GPS** et qu'un fix
  est à jour. `sarsat_screen` = **0 fermé / 1 vue décodage / 2 vue niveau** ; sur
  `2` le RP2040 bascule sur des fenêtres de capture courtes pour que la barre de
  niveau se rafraîchisse à ~7 Hz pour le réglage de gain AF (et arrête de décoder
  jusqu'à ce qu'il en sorte). La radio envoie aussi cette réponse **non
  sollicitée** à l'entrée/sortie de l'écran SARSAT et sur la bascule
  décodage↔niveau, pour que le changement prenne effet sans attendre le HELLO
  de 5 s. Le RP2040 alerte aussi si la radio n'est pas près de 406 MHz ou pas en
  FM, choisit le décodeur APRS vs SARSAT d'après la fréquence, et utilise la
  position de l'opérateur pour la distance/azimut de `APRS_RXINFO`.

Les deux bouts ont besoin d'un petit changement :
- firmware radio : appeler `SendReply()` depuis le gestionnaire `0x06Cx`.
- RP2040 : un **parseur** de trame (`quansheng_frame.c` est encode-seul
  aujourd'hui) + lire la réponse après `HELLO` dans `main.c`.

### Tester la Phase 2 sans radio

`tools/fake_radio.py` émule le bout radio. Câble un adaptateur USB-TTL 3,3 V sur
l'UART de la C-Board à la place du portatif (RX adaptateur ← RP2040 GP0, TX
adaptateur → GP1, GND commun), puis :

```
tools/fake_radio.py /dev/ttyUSB0                # décode les trames du RP2040
tools/fake_radio.py /dev/ttyUSB0 --ack          # + répond ACK à chaque 0x06Cx
tools/fake_radio.py /dev/ttyUSB0 --ack --status # + faux VFO/freq/mod sur HELLO
```

- le **mode décodage** fonctionne déjà et ne demande aucun changement firmware —
  il vérifie l'encodeur de trame du RP2040 (CRC-16/XMODEM, obfuscation, framing)
  et affiche chaque `CLEAR` / `TEXT L.. "..."` / `HELLO` exactement comme le
  parseur radio les voit. Son aller-retour build/parse est bit à bit contre
  `quansheng_frame.c`.
- **--ack / --status** exercent le chemin retour une fois le parseur RP2040 en place.

Tu peux aussi le pointer sur le vrai câble de programmation de la radio pour voir
ce que le firmware UV-K5 renvoie (rien pour l'instant, jusqu'à l'ajout de
`SendReply()`).
