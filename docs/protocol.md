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
| `0x06C2` | `SARSAT_BEACON` | struct compacte 29 o (ci-dessous) | résultat de décodage lisible par la machine, poussé après chaque décodage propre. La radio le met en cache pour son message APRS « Send SARSAT ». |
| `0x06C3` | `SARSAT_LEVEL` | `peak:u16,rms:u16,dc:u16,clip:u8,adcmin:u16,adcmax:u16,verdict:u8` LE | télémétrie de niveau audio pour l'écran de réglage de la radio, ~1/s, sans ACK |
| `0x06D0` | `APRS_CONFIG` | `call[6], ssid:u8, path:u8, sym_table:u8, sym_code:u8, digi_level:u8, flags:u8` (12 o) | radio → RP2040, sans ACK : poussé à chaque sauvegarde du menu APRS **et** à `APRS_Init()` (donc aussi après un reboot radio, le RP2040 n'ayant pas d'état persistant). `call`/`ssid` servent au digipeat « traçable » (voir plus bas), `path`/`sym_table`/`sym_code` sont gardés côté RP2040 mais inexploités pour l'instant. `digi_level` : 0 off, 1 répète WIDEn-N pour n=1, 2 aussi n=2, 3 aussi n=3 (cumulatif). `flags` bit 0 = **mode TNC KISS** (voir plus bas). La config côté radio vit en EEPROM `0x1D00` (56 o ; **pas** `0x1D50`, que `SETTINGS_SaveSettings()` écrase) sur le V1, adresse dédiée équivalente sur le K1/K5V3. |
| `0x06D2` | `APRS_RXTEXT` | `line:u8, ascii[0..18]` | RP2040 → radio : une ligne d'un paquet APRS 144.8 MHz décodé, utilisée seulement quand le champ info n'a **pas** pu être parsé. `line = 0xFF` efface la vue RX ; `line = 0` est l'indicatif source, `1..3` le champ info enroulé. Sans ACK. |
| `0x06D3` | `APRS_RXINFO` | décodé structuré (ci-dessous) | RP2040 → radio : un paquet APRS parsé (symbole, lat/lon, cap/vitesse/altitude, distance+azimut vers l'opérateur, source, nom d'objet, chemin digipeater, texte commentaire/statut/message). La radio le rend façon Kenwood avec une icône symbole et une ligne « Direct » / « Via … ». Sans ACK. |
| `0x06D5` | `APRS_GPS` | `flags:u8, lat_e5:i32, lon_e5:i32, speed_kmh:u16, course_deg:u16, alt_m:i16, sats:u8` LE (16 o) | RP2040 → radio, ~toutes les 3 s : le fix d'un module GPS sur l'en-tête NMEA de la C-Board (UART1 GP5, 9600 8N1, `$GxRMC`/`$GxGGA`). `flags` bit0 = fix valide. Envoyé seulement une fois qu'un module a été vu. La radio l'utilise pour la balise quand le champ **Pos** de son menu APRS est sur **GPS** (sinon elle balise la lat/lon manuelle), et pilote un symbole GPS en barre haute (absent = pas de trames, clignotant = `flags` bit0 à 0, fixe = fix). Le RP2040 utilise aussi son propre fix pour la distance/azimut RX quand il est valide. Sans ACK. |
| `0x06D6` | `APRS_DIGI` | trame AX.25 brute : `dst[7] src[7] digi[7]×n ctrl pid info`, **sans FCS** | RP2040 → radio, sans ACK : une trame que le RP2040 a décidé de digipeater (`aprs_digi.h` — WIDEn-N New-N-Paradigm, cf. section dédiée plus bas) et déjà mutée (SSID décrémenté / bit H posé). La radio recalcule le FCS et émet en AFSK sur le canal 170, exactement comme sa propre balise (mêmes conditions CSMA / écran SARSAT). |

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
u8   flags            bit0 has_pos, bit1 has_course/speed, bit2 has_alt, bit3 has_range,
                      bit4 ADRASEC (message "ADRASEC // Lat: .. // Lon: .." de
                      PCT_Report : lat_e5/lon_e5 = point demandé ; la radio ouvre
                      un écran collant coordonnées DMS + décimal, EXIT pour sortir)
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

### Digipeater WIDEn-N (`APRS_DIGI`)

Menu APRS (F+5) → champ **Digi** : `off` / `WIDE1` / `WIDE1+2` / `WIDE1+2+3`
(cumulatif — `WIDE1+2` répète aussi bien un alias `WIDE1-N` qu'un `WIDE2-N`).
Toute la décision vit côté RP2040 (`rp2040/src/aprs_digi.c`/`.h`, testée sur
hôte, `rp2040/test/host/test_aprs_digi`), qui a déjà l'adresse AX.25 sous la
main pour chaque trame décodée :

0. **Verrous anti-boucle** (si un indicatif est configuré) : la trame n'est
   **jamais** répétée si son adresse **source** est notre propre
   indicatif-SSID (on relaierait notre propre trame revenue par un autre
   chemin), ni si notre indicatif-SSID apparaît **déjà** quelque part dans
   la liste digi/via (utilisé ou non) — preuve que cette trame est déjà
   passée par nous une fois. Ce dernier contrôle ne périme jamais,
   contrairement à la suppression de doublon temporelle ci-dessous.
1. Parcourt les adresses digipeater/via de la trame (à partir de l'octet 14) ;
   celles déjà marquées répétées (bit H, affiché « WIDE1\* » sur un moniteur)
   sont **sautées**, jamais retouchées — seule la première adresse *non
   encore utilisée* est éligible (routage source AX.25 strictement
   positionnel, jamais de saut en avant dans le chemin).
2. Si cette adresse est un alias générique `WIDEn` (texte littéral du
   callsign, `n` = 1..3) avec `n` ≤ `digi_level` : décrémente son SSID (le
   « N » de `WIDEn-N`). L'alias **garde toujours son nom** (`WIDEn`) ; son
   propre bit H passe à 1 une fois le compteur à 0 (relais terminé pour cet
   alias), reste à 0 sinon (pour qu'un digipeater suivant puisse encore le
   satisfaire — le relais multi-sauts d'un `WIDE2-2`).
   Digipeat « traçable » si un indicatif est configuré côté radio (poussé
   par `0x06D0`) : une **nouvelle adresse est insérée** juste avant l'alias,
   portant `INDICATIF-SSID` bit H posé (c'est nous qui traitons ce saut) —
   la trame grandit de 7 o, à **chaque** saut, pas seulement le dernier.
   Recevoir `WIDE2-2` produit `INDICATIF-SSID*,WIDE2-1` après un saut,
   `INDICATIF1*,INDICATIF2*,WIDE2*` après qu'un second digipeater l'a
   terminé — un moniteur voit à la fois quelles stations ont relayé et
   comment l'alias générique a été consommé, à chaque étape. Repli sur un
   simple décrément en place (pas d'insertion, alias `WIDEn` seulement
   décrémenté/marqué) si aucun indicatif n'est configuré (`NOCALL`), ou si
   les 7 o supplémentaires ne tiendraient pas dans le tampon.
3. Sinon (adresse déjà utilisée en tête, indicatif explicite, alias hors
   plage, ou chemin déjà entièrement satisfait) : la trame n'est **pas**
   répétée.

Une trame acceptée est mise en file (1 emplacement) côté radio plutôt
qu'émise immédiatement, puis envoyée en AFSK dès que le canal est **vraiment**
libre — retentée à chaque tick (~10 ms) *et* à chaque itération de la boucle
du popup RX (qui bloquerait sinon `APRS_TimeSlice()` tout du long), avec un
petit délai de contenance (~300 ms) avant la première tentative pour laisser
le canal se stabiliser, et abandon après ~3 s si le canal reste occupé
(mieux vaut renoncer qu'émettre une répétition tardive). `0x06D6` arrive en
effet juste après la salve à répéter, radio quasi certainement encore en
réception à cet instant précis — une unique tentative immédiate perdait la
course CSMA presque systématiquement (remonté sur l'air). Le test « canal
occupé » utilise `g_SquelchLost` (porteuse présente), pas `gCurrentFunction`
(qui d'après `functions.h` désigne l'état « squelch fermé » avec
`FUNCTION_RECEIVE`, et se fige de toute façon pendant que la boucle du popup
bloque le tick principal).

**Anti-boucle temporelle** : en complément des verrous du point 0, une
suppression de doublon (`aprs_digi_seen_recently()`, ~30 s glissants, clé =
adresses dst+src + hachage du champ info) empêche de re-répéter une trame
déjà digipeatée récemment — utile quand un même poste
gère plusieurs niveaux WIDE à la fois (ex. `WIDE1+2`) et entend son propre
relais revenir via un autre digipeater avant d'avoir épuisé tous les sauts.

Aucune passerelle spécifique (aucune action sur un indicatif explicite en
tête de chemin) : seuls les alias `WIDEn` génériques déclenchent une
répétition, conformément au périmètre demandé.

### Accusé automatique des messages reçus

Tout message APRS **adressé à notre indicatif** (indicatif de base comparé,
SSID toléré) et **portant un numéro `{NN`** est accusé automatiquement — c'est
le comportement APRS standard. Fait **côté RP2040** : il construit
`::<expéditeur>:ackNN` (`aprs_build_ui()`, chemin `WIDE1-1`) et le remet à la
radio par `0x06D6 APRS_DIGI` (la radio ajoute le FCS, CSMA, émet sur le canal
170). Rien à faire côté firmware radio. Inactif en mode KISS (l'hôte gère
l'APRS).

### Message « report balise 121 MHz »

Menu APRS (F+5) → champ **`Send report`** : un message APRS pré-formaté vers un
destinataire fixe (champ **`To`** du menu, adressee APRS ≤ 9 caractères),
chemin fixe `WIDE1-1,WIDE2-2`, pour un compte-rendu de radiogoniométrie sur
balise de détresse 121,5 MHz. **Entièrement côté radio** — le RP2040 n'est pas
impliqué en émission ; l'accusé de réception est repéré dans le décodage
`APRS_RXINFO` (`0x06D3`) déjà poussé pour tout message reçu.

À l'activation : demande `Signal ?` (1 chiffre 0-9) puis, si > 0, `Direction ?`
(0-359) ; si 0, `Dir: KO` et pas de question direction. Le message émis est
`INDICATIF-SSID>APZSAR,WIDE1-1,WIDE2-2:` + info
`:DESTINATAIRE:Report Balise 121 MHz S: <n> Dir: <ddd|KO> <lat> <lon>{NN`
(chiffres bruts, `{NN` = numéro de message APRS, cycle 1..99). `<lat> <lon>` =
position résolue (fix GPS si mode GPS, sinon lat/lon manuel du menu) en
**degrés décimaux, 4 décimales, signées** (`-` = S / O) ; `0.0000 0.0000` si
aucune position n'est réglée.

La radio ré-émet **3 fois, 30 s d'intervalle** (sur ~90 s), puis **reste en
écoute de l'accusé pendant 5 min** avant de conclure. L'accusé
`:INDICATIF:ackNN` du destinataire (indicatif de base comparé, SSID toléré ;
numéro de message vérifié) est repéré même s'il arrive après la fenêtre de
ré-émission, y compris après un abandon. Les trames rentrantes sont traitées
aussi bien écran fermé (tick) qu'en restant dans le menu APRS (la boucle
bloquante sert l'UART à chaque itération). État sur la ligne `Send report` :
`Report TX n/3` → `Report wait ack` → `Report ACK OK` / `Report no ack`.
Sans C-Board branchée (rien pour entendre l'accusé), le verdict est
`no ack` au bout des 5 min.

### Mode TNC KISS

Menu APRS (F+5) → champ **`KISS TNC on/off`**. Quand il est actif :

- **Côté RP2040** : l'USB-CDC cesse d'être la console de debug (tous les
  `LOG()` deviennent muets) et devient un **flux KISS binaire** (SLIP :
  `FEND 0xC0`, `FESC 0xDB`, `TFEND 0xDC`, `TFESC 0xDD`). Le RP2040 arrête de
  décoder-pour-l'affichage et de digipeater : c'est un simple modem Bell-202.
  - RX : chaque trame AX.25 démodulée sort en **frame KISS data** (`FEND 0x00
    <trame échappée> FEND`) sur l'USB.
  - TX : une frame KISS data reçue de l'hôte est transmise à la radio en
    `APRS_DIGI` (`0x06D6`), telle quelle -- la radio ajoute le FCS et clé sur
    le canal 170, avec sa garde CSMA (`g_SquelchLost`). Un seul emplacement
    de file : un hôte qui rafale plusieurs trames en perd (APRS est lent).
  - Les commandes KISS non-data (`TXDELAY`, `P`, `SlotTime`, `TXtail`,
    `SetHardware`) sont acceptées et **ignorées** (la radio gère son keying).
- **Côté radio** : `APRS_TimeSlice()` n'exécute plus l'auto-balise ni le
  « report 121 MHz » ; `aprs_rx_arrived()` (popup) est inhibé. Reste actif :
  le fast-squelch APRS, le gain AF C-Board, et le relais `0x06D6 → APRS_TxFrame`
  (chemin digipeat réutilisé). L'écran APRS affiche `KISS TNC  host USB` en
  en-tête. La radio n'a pas besoin de garder un écran ouvert -- il suffit
  qu'un VFO soit sur 144.800 FM.
- **Transport** : bit 0 de l'octet `flags` (12ᵉ octet) de `APRS_CONFIG`
  (`0x06D0`) = KISS. Poussé à chaque changement du champ menu.
- **Half-duplex** : pendant une TX (~0,5-1 s) la RX est morte -- normal pour
  un TNC mono-radio. Un moniteur série lent qui gèle la pompe d'échantillons
  ferait perdre des trames RX (`PICO_STDIO_USB_STDOUT_TIMEOUT_US` limite ce
  risque).

### Charge utile `SARSAT_BEACON` (little-endian, compact, 29 octets)

Envoyé par le RP2040 après chaque décodage propre (en plus de `SARSAT_TEXT`).
La radio le met en cache (`APRS_NoteBeacon()`) pour son message APRS
**« Send SARSAT »** (voir plus bas).

```
off  type  champ            note
0    u8    frame_bits       112 ou 144
1    u8    protocol         enum ProtocolType (dec406_v1g.h)
2    u8    is_test          1 = trame test/auto-test/exercice
3    u8    has_position     1 = lat/lon valides
4    u16   country_code
6    i32   lat_e5           latitude  × 1e5, signé  (0 si has_position=0)
10   i32   lon_e5           longitude × 1e5, signé
14   char  hex_id[15]       ID balise COSPAS 15-hex, ASCII, zéro-complété
```
Total **29 octets**. (L'ancien projet de struct 55 o avec `ident[24]` /
`lat_1e4` n'a jamais été implémenté ; le format ci-dessus est celui du code.)

### Message APRS « Send SARSAT » (radio, sur validation opérateur)

Champ **`Send SARSAT`** du menu APRS (F+5), à côté de `Send report`. Sur
appui → confirmation « Send SARSAT ? » (A = envoyer / EXIT). Émet un message
APRS vers `gAprsCfg.msg_to` (le **même destinataire** que le report 121),
chemin `WIDE1-1,WIDE2-2`, avec un numéro de message → accusé automatique du
client destinataire, repéré via `0x06D3` (`APRS_MsgCheckAck()`, mêmes 3
ré-émissions / 30 s puis écoute 5 min que le report). État sur la ligne :
`Bcn #NN TX n/3` / `Bcn #NN wait ack` / `Bcn #NN ACK OK` / `Bcn #NN no ack`.

Corps du message :
```
:DEST     :SARSAT <hexID 15> <lat> <lon> c<pays>[ TEST]{NN
:DEST     :SARSAT <hexID 15> NOPOS c<pays>[ TEST]{NN        (sans position)
```
`<lat> <lon>` = degrés décimaux signés, 4 décimales (même format que le
report 121). Exemple : `:F4DVK    :SARSAT 1C72091A2B3FDFF 42.9544 1.3644 c227 TEST{03`.
Prérequis : indicatif ≠ NOCALL, `msg_to` renseigné, une balise en cache, et
la radio sur le canal APRS 170 (F+5 y bascule — voir `integration.md`).

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
