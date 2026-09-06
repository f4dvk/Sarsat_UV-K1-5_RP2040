# Firmware décodeur SARSAT + APRS pré-compilé (RP2040 / RP2350)

| fichier | carte | MCU | text / bss |
|---|---|---|---|
| `sarsat_rp2040-pico.uf2`  | Raspberry Pi Pico / RP2040-Zero | RP2040 (Cortex-M0+, sans FPU) | 77 Ko / 163 Ko |
| `sarsat_rp2040-pico2.uf2` | Raspberry Pi Pico 2 | RP2350 | 72 Ko / 163 Ko |

Les deux sont le **même firmware** (`../src/`), ne différant que par le cœur cible.

## Deux décodeurs, sélection automatique

La C-Board fait tourner **un seul** des deux décodeurs à la fois, choisi d'après
la fréquence RX que la radio rapporte dans sa réponse HELLO `0x06CF` :

- **~406 / 434 MHz** → COSPAS-SARSAT 1ʳᵉ génération (FGB / T.001), fenêtres 1,1 s.
- **144,0–148,0 MHz** → **APRS** : démod Bell-202 AFSK 1200 continu + dé-framing
  AX.25. 3 slicers parallèles (enveloppe biaisée ±3/8) + réparation FCS par
  bascule d'un seul bit (récupération façon Dire-Wolf — l'AX.25 n'a pas de FEC),
  puis **réparation croisée entre chaînes** (`aprs_try_fix_cross()`) si la
  réparation 1-bit échoue : les 3 chaînes décodent souvent la même salve en ne
  différant que sur quelques octets (trop pour 1 bit, mais une chaîne voisine
  a probablement le bon octet à ces positions) — chaque chaîne met en cache
  son dernier essai, et on teste les combinaisons "mon octet / celui du
  voisin" aux positions qui diffèrent (borné à 5 octets, sinon abandon),
  jusqu'à trouver celle qui passe le FCS et ressemble à une trame UI valide.
  Le champ info est ensuite parsé (non compressé / compressé / MIC-E / objet /
  item / statut / message) et poussé à la radio en décodé structuré `0x06D3` —
  symbole, lat/lon, cap/vitesse/altitude, distance + azimut vers l'opérateur
  (depuis la position dans la réponse HELLO de la radio) — pour que la radio le
  dessine façon Kenwood. Une trame dont le champ info n'est pas compris retombe
  sur du texte brut `0x06D2`. Un **limiteur FI** normalise l'entrée du
  discriminateur, donc un signal fort (même écrêté à l'ADC) démodule comme un
  signal nominal — plus de « fort = distordu = pas de décodage ». La ligne
  console `[aprs]` rapporte `clip=NN.N%` (butées de l'ADC) ; au-delà de ~2 %
  elle affiche `lower the radio AF gain`. Voir les lignes `[aprs]`
  (`M fixed` = trames récupérées par la bascule de bit).

Accorde le VFO de la radio et la C-Board suit en ~1 s (une ligne `mode APRS` /
`mode SARSAT` s'affiche au basculement).

## GPS (optionnel)

Un module GPS sur l'en-tête NMEA de la C-Board (**UART1 GP5, 9600 8N1**) est
parsé (`$GxRMC` + `$GxGGA`) et le fix est transmis à la radio en `0x06D5` toutes
les ~3 s. Dans le menu APRS de la radio (F+5), mets **Pos** sur **GPS** et la
balise utilise la position en direct (plus une extension cap/vitesse en
mouvement) ; laisse sur **manuel** pour baliser la lat/lon saisie. Sans module
connecté rien n'est envoyé et le comportement est inchangé. Le RP2040 utilise
aussi son propre fix pour la distance/azimut RX quand il est à jour. La barre
supérieure de la radio affiche un petit symbole GPS : **clignotant** pendant la
recherche, **fixe** au verrouillage, **rien** si aucun module n'est branché.
Le slicer de bits est en point fixe (int64) dans les deux, donc le RP2040
fonctionne sans FPU ; `-DSARSAT_SLICER_DOUBLE=ON` au moment du CMake bascule sur
le slicer double (ne vaut le coup que sur RP2350).

## Digipeater APRS (optionnel)

Menu APRS de la radio (F+5), champ **Digi** : `off` (défaut) / `WIDE1` /
`WIDE1+2` / `WIDE1+2+3`. La décision (`rp2040/src/aprs_digi.c`, testée sur
hôte) tourne sur chaque trame décodée : le premier alias `WIDEn` non encore
utilisé dans le chemin (une entrée déjà marquée « WIDE1\* » est sautée,
jamais retouchée) est décrémenté si `n` est dans le niveau choisi. L'alias
garde toujours son nom `WIDEn` ; son bit « répété » passe à 1 une fois le
compteur à 0, reste à 0 sinon (pour qu'un relais suivant satisfasse les
sauts restants d'un `WIDE2-2`). Si un indicatif est configuré côté radio
(**digipeat traçable**, poussé par `0x06D0`), une nouvelle adresse
`INDICATIF-SSID*` est **insérée** devant l'alias à chaque saut : `WIDE2-2`
reçu donne `INDICATIF-SSID*,WIDE2-1`, puis `IND1*,IND2*,WIDE2*` après un
2ᵉ digipeater. Repli sur un simple décrément si pas d'indicatif ou pas de
place. **Verrous anti-boucle** : jamais de répétition si la source de la
trame est notre propre indicatif, ni si notre indicatif figure déjà dans le
chemin digi (ne périme pas), en plus d'une suppression de doublon
temporelle (~30 s). Une trame retenue est mise en file côté radio (pas
émise tout de suite), petit délai de contenance ~300 ms, retentée à chaque
tick (test canal occupé = `g_SquelchLost`), abandon après ~3 s, puis émise
sur le canal 170 comme la propre balise de la radio. Détail complet dans
`docs/protocol.md` (section « Digipeater WIDEn-N »). **Non testé sur l'air.**

## Mode TNC KISS (optionnel)

Menu APRS de la radio (F+5), champ **KISS TNC** : `on` / `off` (défaut). En
`on`, la radio coupe son propre tracker (auto-balise, report 121, popup RX) et
la C-Board devient un **TNC KISS** sur son port série USB : l'hôte (Direwolf,
APRSdroid via USB-OTG, un client APRS-IS…) parle l'AX.25, la C-Board n'est que
le modem Bell-202.

- **RX** : chaque trame AX.25 décodée (front-end + 3 slicers + réparations,
  comme d'habitude) est encapsulée SLIP/KISS (`FEND 0x00 …data… FEND`) et
  poussée telle quelle sur l'USB-CDC — pas de parsing APRS, pas de `0x06D3`.
- **TX** : les trames KISS reçues de l'hôte (commande data uniquement ;
  TXDELAY/persist/… acceptées et ignorées) sont dé-slippées et envoyées à la
  radio en `0x06D6`, qui les émet sur le canal 170 via son chemin digipeat
  (FCS ajouté, garde CSMA `g_SquelchLost`). File à un seul emplacement : une
  rafale de trames rapprochées est tronquée. Half-duplex.
- **Log** : les lignes console `[…]` sont **tues** en mode KISS (elles
  pollueraient le flux binaire KISS sur le même port). Repasse `KISS TNC off`
  pour retrouver la console.

Codec : `../src/kiss.c`/`.h`, testé hôte `../test/host/test_kiss`. Détail dans
`docs/protocol.md` (section « Mode TNC KISS »). **Validé sur matériel
(2026-09-06) : RX et TX confirmés bout en bout avec `kissutil`.**

La C-Board de KD8CEC utilise une **RP2040-Zero**, donc `sarsat_rp2040-pico.uf2`
est celui à y flasher.

## Flash

Maintiens BOOTSEL enfoncé en branchant l'USB → un lecteur `RPI-RP2` apparaît →
copie le `.uf2` dessus. La carte redémarre sur le firmware.

## Console de debug (USB-CDC)

Ouvre le port série USB du RP2040 (`/dev/ttyACM0`, n'importe quel débit). Les
lignes sont étiquetées :

| étiquette | quand | signification |
|-----|------|---------|
| `[lvl]` | ~toutes les 3 s au repos | `peak` / `rms` de la fenêtre audio, min..max ADC brut, DC. **Règle le volume radio pour que `peak` lise ~4000-12000 sur une balise.** |
| `[burst]` | l'audio a franchi le seuil d'armement | le décodeur tourne sur cette fenêtre |
| `[slicer]` | après chaque salve | `144 bits: <hex>` (trame slicée brute, sync incluse) ou `no sync / no frame` |
| `[decode]` | après slicing | `BCH OK` + hex ID / pays / protocole / position / ident, ou `BCH uncorrectable`, ou (signal marginal) `BCH OK but hexID all-zero -- false positive, rejected` — le correcteur BCH peut, sur une fenêtre très bruitée, converger vers un mot de code valide mais dégénéré (ID tout à zéro, pays 0/Inconnu, position 0,0) ; comme aucune vraie balise COSPAS-SARSAT n'a un ID entièrement nul, ce cas est rejeté avant d'être poussé à la radio plutôt que d'afficher des infos erronées |
| `[tx]` | au décodage, et keepalive 5 s | `CLEAR`, `TEXT L0 "..."`, totaux d'octets, `HELLO` (avec compteur d'ack) |
| `[link]` | réponse radio, et battement 20 s | `link UP/DOWN`, `acks=N`, et une ligne `status:` au changement (VFO / modulation / fréquence RX / écran de la radio). Alerte seulement si la radio n'est pas en FM. Aucune restriction de fréquence — balises d'exercice 406 ou 434 MHz OK. |
| `[meter]` | tant que le mètre est actif (`m`) | par fenêtre : `rms/peak/dc/clip/adc` + une barre, pour régler le volume radio |
| `[aprs]` | mode APRS : sur chaque paquet + statut ~3 s | `#N SRC direct/via …` puis info enroulée ; un **message** affiche `#N SRC direct  to [ADRESSEE] : TEXTE` (utile pour voir à qui / quel `ackNN` un accusé du « report 121 » est adressé) ; statut = `CARRIER/idle  mod=FM  hdlc=N -> pkts=N (M fix) fcs_bad=N  clip=X.X%  cdt=N env=N` (`mod` = démod courante de la radio d'après sa réponse HELLO — doit lire `FM` ou `DSC` ; `cdt` = énergie de détection de porteuse, `env` = enveloppe du discriminateur) |
| `[aprs.rx]` | **provisoire** (`CFG_APRS_RX_DIAG`) : chaque candidat de trame HDLC non trivial | `chC RES len=L ui=U src=CALL \| <hex tête>` — `RES` = `OK`/`FIX`(1 bit réparé)/`BAD`(échec FCS)/`LNG`(débordement)/`DUP` (les fragments `SHT` sont comptés mais pas affichés). `ui=1` = la structure d'octets ressemble à une trame UI AX.25 valide. Lecture : `hdlc=0` en permanence → rien de démodulé (niveau/accord/squelch) ; `BAD` avec `ui=1` + `src` lisible mais la **longueur varie fortement pour la même station** → erreurs de bits corrompant le framing HDLC. Causes habituelles, dans l'ordre : le squelch radio qui **fait des bagots** en pleine salve (chaque ouverture/fermeture masque ~10–20 ms = 12–24 bits — utiliser l'option APRS `Squelch fast`, qui met désormais le délai de fermeture au max, et/ou baisser le `SQL` radio), pente de dé-emphase FM (`Demodu → DSC` + `W/N → Wide`), décalage d'accord. Mets `CFG_APRS_RX_DIAG 0` pour l'usage normal — les écritures console coûtent du temps de décodage. |
| `[aprs.raw]` | **provisoire** (`CFG_APRS_RX_DIAG`, commande `w`) | capture ADC brute one-shot de la prochaine salve, `begin … <hex 12 bits, 32 échantillons/ligne> … end`. Enregistre la console dans un fichier et décode-la hors-ligne avec `test/host/test_aprs_wav` pour une instrumentation bit par bit complète. |
| `[digi]` | niveau de digipeat reçu de la radio (`0x06D0`, au changement) ; chaque trame effectivement répétée | `level: off/WIDE1/WIDE1+2/WIDE1+2+3` ; `repeating #N (L bytes)` avant l'envoi `0x06D6` à la radio. |

### Console série

Tape dans le port série USB :

- **`m`** — bascule le MÈTRE de niveau. Regarde-le en tournant le volume radio :
  vise le **souffle** à `rms ~2000-3500`, `clip 0%`, `dc` proche du milieu
  d'échelle. Une salve de balise lit alors plus bas et décode.
- **`s`** — une ligne de mètre one-shot.
- **`w`** — (mode APRS, `CFG_APRS_RX_DIAG`) arme une capture ADC brute one-shot
  de la prochaine salve ; elle s'affiche en hex `[aprs.raw]` pour décodage
  hors-ligne.
- **`h`** — aide.

Mets `SARSAT_LOG 0` dans `src/decoder_config.h` pour taire le log ;
`CFG_TX_HEXDUMP 1` pour aussi dumper chaque trame UART + chaque `[link] ACK` en hex.

## Recompiler

```sh
export PICO_SDK_PATH=~/pico-sdk
cd ..            # rp2040/
cmake -S . -B build -DPICO_BOARD=pico     # ou pico2
cmake --build build
```

`sha256.txt` couvre les fichiers `.uf2` livrés ici.

> Pas encore testé sur matériel RP2040. Le chemin de décodage est validé sur
> hôte contre de vrais enregistrements de balises — voir `../test/host/` et le
> README du projet.
