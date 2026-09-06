# UV-K5 V1 (KD8CEC) — intégration de l'écran SARSAT

Base : source **KD8CEC `uvk5cec-0.3q`** (lignée egzumer, DP32G030 / BK4819).
Reproduire avec `./build.sh /chemin/vers/uvk5cec-0.3q` →
`bin/uvk5-cec-0.3q-sarsat.packed.bin`.

Toolchain : `arm-none-eabi-gcc` 13.2 (natif), `make`, `python3` + `crcmod`
(`pip install --user --break-system-packages crcmod`). Pas de Docker, pas de
J-Link — les étapes `openocd` du Makefile ne vivent que dans les cibles
`debug` / `flash`.

FLASH : **56 772 o de text + 16 o de data** sur 60 Kio (marge ~4,6 Kio) avec
`MAIN_SCREEN=moto`. La vue RX APRS rend un décodé structuré façon Kenwood avec
une icône symbole 16x16 ; `ENABLE_AUDIO_BAR` est désactivé dans `build.sh`
(l'overlay de barre micro TX est inutilisé ici) pour garder la marge.

## Écran VFO principal — `MAIN_SCREEN = stock | moto | id91`

`patch/ui_main.c.diff` place un `UI_DisplayMain()` alternatif (+ corps de
`DisplayRSSIBar()`) derrière la variable make `MAIN_SCREEN`. `build.sh` choisit
`moto`.

- **`stock`** — l'écran double-VFO egzumer/KD8CEC d'origine, totalement intact.
  Text **56 892 o** (le plus lourd — la fonction d'origine est grosse).
- **`moto`** — idiome MOTOTRBO DP/GP : en-tête inversé à coins arrondis (l.0) ;
  **fréquence dans une police condensée-grasse 24 px** (`MOTO_DIG`, ~450 o,
  générée par `tools/gen_bigdigits.py`) sur les l.1-3 ; ligne de détails centrée
  (l.4) ; S-mètre **antenne + 5 barres montantes** avec `-84 dBm S6` (l.5, corps
  alternatif de `DisplayRSSIBar`) ; l'autre VFO sur la l.6. Text **56 712 o**
  (−180 vs stock).
- **`id91`** — même disposition, mais réutilise des primitives déjà dans le
  firmware : `UI_DisplayFrequency` / `gFontBigDigits` (16 px, l.1-2) pour la
  fréquence et le **S-mètre segmenté horizontal stock** (`DrawLevelBar`) sur la
  l.4. Text **56 492 o** (**−400 vs stock, −220 vs moto**).

Helpers partagés `MainHeader` / `MainDetail` / `MainIdleVfo`. Les deux
redessins gèrent fréquence / canal MR (tous les `MDF_*`) / saisie de fréquence /
chaînes `VfoState` / plages de scan ; toutes les écritures restent dans
`gFrameBuffer[0..6]` ; pas de barre de touches en bas (l'UV-K5 n'en a pas sous
le LCD). Les trois compilent `-Wextra` propres. Pas encore testé sur matériel ;
`gDTMF_InputMode` (saisie DTMF manuelle) n'est pas dessiné en moto/id91.

> **Note :** la disposition `moto` a été retravaillée en plusieurs itérations
> terrain (voir l'historique git et le plan de projet) : en-tête en police 6 px
> dédiée `HDR6` avec tous les flags à position fixe (VFO, TX/RX, puissance,
> décalage, CTCSS, reverse, modulation), VFO inactif en 7 px sur la l.6, S-mètre
> centré à cheval sur les l.4/5.

## Ce que ça fait

La C-Board RP2040 décode les balises COSPAS-SARSAT 406 MHz 1ʳᵉ génération depuis
l'audio FM-RAW de la radio et pousse des lignes de texte formatées à la radio
sur la trame UART Quansheng standard. Ce firmware ajoute un écran pour les
afficher.

- **Auto-ouverture :** au premier `0x06C1` (SARSAT_TEXT) la radio bascule sur
  l'écran SARSAT.
- **Ouverture manuelle :** **F + 8** (remplace le raccourci F+8 « frequency
  reverse » quand `ENABLE_SARSAT=1`).

Mets le VFO sélectionné sur une fréquence de balise 406 MHz (406.025 / 406.028 /
406.037 MHz) avant d'ouvrir l'écran — le mode SARSAT reçoit juste *là*, il ne
scanne pas.
- **Défilement :** **HAUT / BAS** (le décodé est coupé aux mots en ~12-14 lignes
  courtes ; 6 lignes + un en-tête visibles — la zone de contenu du LCD fait
  7 lignes). L'en-tête montre `SARSAT n-m/total ^v`.
- **`5` :** bascule entre le texte de balise et l'écran de **niveau audio**
  (rms / peak / dc / clip / plage adc + barre + une ligne de verdict).
  **HAUT/BAS sur l'écran niveau règlent le gain AF BK4819** qui alimente la
  C-Board — mets le pot de volume au max et ajuste contre la barre rms ; sauvé
  en EEPROM. Le RP2040 envoie `0x06C3 SARSAT_LEVEL`. **Sur cet écran la C-Board
  bascule sur des fenêtres de capture courtes ~125 ms pour que la barre se
  rafraîchisse à ~7 Hz** (vs ~1 Hz sur la vue décodage) — la radio signale la
  vue niveau dans sa réponse HELLO (`screen=2`) et la ré-envoie immédiatement
  sur la bascule `5`, donc l'effet est immédiat. Le décodage est en pause tant
  que tu es sur cet écran. **Règle le souffle au milieu de la barre
  (`OK reglage correct`, dc à mi-échelle, clip 0 %) — ne le pousse PAS fort
  pour chasser la balise.** Le RP2040 détecte une salve *relativement* au
  plancher de souffle suivi (`CFG_BURST_QUIET_PCT`, une salve 406 MHz capturée
  lit 6–12 dB sous le souffle), donc il décode à n'importe quel gain propre.
  Le verdict `fort: APRS sature` est une mise en garde : le gain est assez haut
  pour qu'un paquet APRS 2 m fort écrête le tap audio partagé — baisse-le.
  L'écran SARSAT (ré)ouvre toujours sur la vue décodage, jamais sur la vue
  niveau.
- **Fermeture :** **EXIT**.

## Tracker APRS (`ENABLE_APRS`)

**F + 5** ouvre l'écran config / statut APRS (le slot spectrum libéré) :

HAUT/BAS déplacent la sélection dans les champs (Call, SSID, Path, Icon,
Interval, Popup, AF gain, Squelch, Light, Pos, Lat, Lon, Text) ; la l.0 est
l'en-tête, les l.1-6 une fenêtre défilante sur les champs (`^`/`v` dans l'en-tête
quand il y a plus au-dessus/en dessous). MENU entre en édition sur la ligne
sélectionnée, MENU à nouveau sauve, EXIT annule (recharge depuis l'EEPROM).
L'en-tête montre l'aide des touches contextuelle.

| champ | édition |
|-------|------|
| Call | HAUT/BAS changent le caractère, `*` déplace le curseur (6 caractères) |
| SSID | HAUT/BAS 0..15 |
| Path | HAUT/BAS none / WIDE1-1 / WIDE2-1 / WIDE1-1,WIDE2-1 |
| Icon | HAUT/BAS voiture / maison / camion / coureur / vélo / yacht / avion / wx / point |
| Interval | HAUT/BAS OFF / 30 / 60 / 120 / 300 / 600 / 900 / 1800 s (auto-balise) |
| Popup | HAUT/BAS OFF / 5 / 10 / 20 s — popup auto de la vue RX sur un paquet décodé, se referme après N s (une touche annule le minuteur) |
| AF gain | HAUT/BAS auto / 1..78 — un curseur combiné sur le BK4819 REG_48 AF Rx Gain-2 + gain DAC (~-52 dB à 1, stock à 78) ; il balaie d'abord Gain-2 63->8 (région linéaire), puis le gain DAC 15->0. Les deux sont ré-appliqués à chaque ouverture d'écran SARSAT / APRS et ~2x/s depuis le tick (`patch/radio.c.diff` empêche `RADIO_SetModulation()` de forcer le gain DAC au max, qui était le bug « plus fort après réouverture ») pour alimenter le tap C-Board. Mets le pot de volume au max et règle ça ; aussi ajustable sur l'écran niveau SARSAT contre la barre rms. `auto` = stock. Persiste via `gEeprom.DAC_GAIN` / `.VOLUME_GAIN` pour que `RADIO_SetupRegisters` le garde — pas de patch radio.c. **`auto` capture le gain du potentiomètre à l'ouverture de l'écran** (`APRS_ResyncAfGainKnob()`, appelée avant `APRS_ApplyAfGain()` dans `APP_RunSarsat()`/`APP_RunAprs()`) — avant ce correctif la capture ne se faisait qu'une seule fois par démarrage (première ouverture d'écran), restant figée même si le potentiomètre était tourné ensuite : `auto` reclampait alors silencieusement le gain bas, perçu comme une perte de sensibilité (bug trouvé sur le portage F4HWN, corrigé en miroir ici puisque le code était identique). |
| Squelch | HAUT/BAS bascule **`Squelch fast`** (défaut) / **`Squelch stock`**. `fast` met le **délai d'ouverture** squelch BK4819 (REG_4E bits 13:11) de 5 à 0 **seulement tant que le VFO RX est 144–148 MHz**, pour que le récepteur démute en quelques ms et que les premiers flags AX.25 d'un paquet ne soient pas perdus. Ré-assené à chaque tick de 10 ms (une reconfig VFO réécrit REG_4E). Hors bande = intact. Pour une perte vraiment nulle, mets le menu `SQL` de la radio à 0 (monitor). |
| Light | HAUT/BAS bascule **`Light on RX`** (stock — rétroéclairage à toute ouverture squelch) / **`Light on frame`**. Avec `Light on frame`, tant que le VFO RX est 144–148 MHz le rétroéclairage sur squelch RX est inhibé et l'écran s'allume **seulement quand la C-Board décode un paquet APRS** (`APRS_QuietBacklight()` conditionne l'appel `BACKLIGHT_ON_TR_RX` dans `APP_StartListening` ; `aprs_rx_arrived()` appelle `BACKLIGHT_TurnOn()`). Hors bande la radio se comporte normalement. |
| Lat / Lon | **saisie des chiffres** (2+5 pour lat, 3+5 pour lon, décimale placée auto), `*` bascule N/S resp. E/W ; ou HAUT/BAS pour des pas de ±0,001° quand rien n'est tapé. |

**5** = baliser maintenant. La config est sauvée en EEPROM **`0x1D00`** (40 o) —
*pas* `0x1D50` (`CEC_EEPROM_START1`) : `SETTINGS_SaveSettings()` y réécrit les
8 premiers octets avec les réglages CW / live-seek de KD8CEC à chaque changement
de menu, ce qui effaçait le magic APRS + l'indicatif (« la config APRS revient
par défaut »). `0x1D00` (`CEC_EEPROM_START2`) est dans la région des contacts
DTMF mais au-delà des 16 vrais contacts, protégé du reset usine, et inutilisé
par 0.3q.
`APRS_Init()` n'est pas sur le chemin de boot ; il tourne en lazy (lecture
EEPROM) la première fois que le tick de balise, l'écran ou une trame RX touche
la config.

L'écran garde le rétroéclairage allumé et rafraîchit le minuteur d'auto-verrou
clavier à chaque touche pendant qu'il tourne, et à la sortie re-sélectionne les
VFO (`RADIO_SelectVfos()` + `RADIO_SetupRegisters(true)`) et rafraîchit les deux
minuteurs — la boucle principale est bloquée tant que l'écran est up, donc sa
compta dual-watch / VFO et ses comptes-à-rebours rétroéclairage + verrou clavier
sont tous périmés au retour (c'était le bug « le VFO B disparaît / le clavier se
verrouille après la sortie »).

**Chemin TX :** `app/aprs.c` construit la trame UI AX.25 (`app/ax25.c` :
FCS-16/X.25, bit-stuffing, NRZI — testé sur hôte par
`rp2040/test/host/test_ax25`), puis clé la PA exactement comme
`FUNCTION_Transmit()` (`BK4819_DisableDTMF()` + `RADIO_SetTxParameters()` + LED
rouge) et génère de l'AFSK Bell-202 de la même façon que le firmware fait son
roger beep : `BK4819_EnterTxMute()` → `REG_70` tone1 (gain `AFSK_TONE_GAIN`,
défaut 64 — **baisse-le si le paquet est surmodulé**) → `BK4819_EnableTXLink()`
→ stabilisation → set `REG_71` et `BK4819_ExitTxMute()`, puis déroule le flux de
bits : `REG_71` est réécrit **seulement sur un vrai changement de tonalité**
(pour que les runs NRZI restent à phase continue) et chaque cellule de bit est
cadencée sur une **échéance SysTick libre** (`AFSK_TICKS_PER_BIT = 48 MHz / 1200`)
qui absorbe le coût ~40 µs de `BK4819_WriteRegister` et se corrige du jitter
d'IRQ — la moyenne reste exactement à 1200 baud. 40 flags d'ouverture,
3 de fermeture. ~0,6 s à l'antenne. **Aucun câblage micro/PTT** — la radio fait
tout, façon KD8CEC.
**Validé sur matériel** : les paquets émis décodent proprement sur un TNC APRS
externe.

> Deux bugs de première version, tous deux corrigés :
> - **`REG_30` à la main** sans `ENABLE_PA_GAIN` / `ENABLE_PLL_VCO` faisait
>   s'effondrer la PA alors que le GPIO PA-enable et la LED rouge restaient
>   actifs (« passe en émission, rien ne rayonne »). `BK4819_EnableTXLink()`
>   met `REG_30` correctement pour la DSP TX.
> - **`WriteRegister` par bit + `SYSTICK_DelayUs(820)`** faisait chaque cellule
>   à ~860 µs (3 % lent) et jittait — aucun TNC ne se verrouillait. L'horloge
>   à échéance plus les écritures `REG_71` seulement-au-changement l'ont corrigé.

> `APRS_Beacon()` clé le TX avec seulement un garde « pas déjà en émission /
> indicatif défini », il ne re-vérifie pas bande/lockout — mais voir ci-dessous,
> il ne balise plus sur le VFO courant de toute façon.

**Canal dédié 170 (« APRS »)** : comme le firmware CEC d'origine, la balise
n'émet **jamais** sur le VFO A/B actuellement affiché — elle recharge toujours
la config du **canal mémoire 170** (le dernier, `APRS_TX_CHANNEL` = index 169)
juste avant de baliser : fréquence, puissance, mode, largeur, décalage, CTCSS…
exactement ce que `RADIO_ConfigureChannel(..., VFO_CONFIGURE_RELOAD)` lit pour
n'importe quel canal mémoire. Ça permet de laisser VFO A/B sur autre chose (une
répétition, la surveillance SARSAT sur l'autre VFO…) tout en continuant de
baliser sur 144.800. Le canal 170 s'affiche partout comme **`APRS`** au lieu de
`CH170`/`MR 170` (nom EEPROM forcé par `APRS_Init()`). À la première utilisation
(canal encore vierge en EEPROM), il est pré-rempli à 144.800000 MHz, FM large,
puissance moyenne, sans CTCSS ; édite-le ensuite comme n'importe quel canal
mémoire (menu Channel, ou copie VFO→canal) pour changer où la balise part —
la config est relue à chaque salve, aucun redémarrage nécessaire. `APRS_Beacon()`
emprunte le slot du VFO TX le temps de la salve (sauvegarde/restauration
complète de `gEeprom.VfoInfo[]`/`ScreenChannel[]`/`MrChannel[]`), donc le VFO
affiché ne change jamais visiblement, y compris pendant la salve.
Ne balise **jamais** tant que l'écran SARSAT (F+8) est ouvert : ce dernier
bloque la boucle principale (donc `APRS_TimeSlice()`/l'auto-balise ne tourne
pas), et `APRS_Beacon()` a un garde explicite (`SARSAT_ScreenOpen()`) pour le
documenter/protéger un futur appelant.

> **Mémoires plafonnées à 170** (`misc.h.diff`, `MR_CHANNEL_LAST` 199 → 169) :
> comme le projet CEC d'origine, il n'y a plus que les canaux **1 à 170** —
> le 170 est bien le dernier, ce n'est plus un canal perdu au milieu de 200.
> Si un canal était déjà enregistré au-delà de 170 sur un ancien build, ses
> octets restent en EEPROM mais deviennent **inaccessibles depuis l'interface**
> (`IS_MR_CHANNEL()` ne va plus jusque-là). Effet de bord unique, au premier
> démarrage après le flash : si le VFO actif était en mode « canal fréquence »
> (bande VHF/UHF prédéfinie, pas un canal mémoire), son numéro logique interne
> a changé et il peut retomber sur une bande par défaut — resélectionne juste
> la fréquence/bande voulue une fois, rien n'est perdu. **Pas de canal mémoire
> normal (1-170) touché par ce changement, seuls les repères fréquence/NOAA
> internes se décalent.**

**RX :** la C-Board RP2040 démodule l'APRS 144.8 MHz (Bell-202 AFSK 1200 +
AX.25, `rp2040/src/aprs_rx.c` — 3 slicers biaisés parallèles + une réparation
FCS par bascule d'un seul bit pour la récupération d'erreur façon Dire-Wolf,
puisque l'AX.25 ne porte pas de FEC), **parse le champ info**
(`rp2040/src/aprs_parse.c` — non compressé / compressé / MIC-E / objet / item /
statut / message) et pousse un décodé structuré `0x06D3` : symbole, lat/lon,
cap/vitesse/altitude, et distance + azimut vers l'opérateur (calculés sur le
RP2040 depuis la position que la radio met dans sa réponse HELLO). La vue RX le
dessine façon Kenwood — une **icône symbole 16x16** (jeu façon aprs.fi incluant
le pylône relais `/r`), nom d'objet ou indicatif source + âge du paquet,
lat/lon, distance/azimut, cap/vitesse/altitude, puis le texte
commentaire/statut/message enroulé. Une trame dont le champ info n'est pas
compris retombe sur du texte brut `0x06D2` (indicatif source sur la ligne 0,
info enroulée en dessous). La vue ouvre directement sur l'affichage RX dès qu'un
paquet — structuré **ou** brut — a été décodé (`APRS_HasRx()`) ; avant elle
retombait sur le menu config après un décodé structuré parce que le tampon
texte brut était vide.
Sur un décodage la vue RX **s'ouvre en popup automatiquement** (selon le réglage
Popup : OFF / 5 / 10 / 20 s ; un cooldown de 5 s seulement si tu as appuyé sur
**EXIT** pour la fermer — une simple fermeture par minuteur laisse le paquet
suivant s'afficher tout de suite). F + 5 l'ouvre aussi. L'**auto-popup masque
la ligne de titre `APRS RX … *:cfg`** et décale le paquet d'une ligne vers le
haut pour un coup d'œil plus net ; le titre n'est montré que quand tu ouvres la
vue toi-même (F + 5). Dans la vue RX **`*`** → config, **EXIT** → ferme (retour
VFO). La RX reste live sous le popup pour que la C-Board continue à décoder —
un **paquet qui arrive sous le popup est affiché aussi et ré-arme le minuteur
d'auto-fermeture**. L'écran fait tourner un mini squelch (poll de l'IRQ SQUELCH
du BK4819, bascule le chemin HP + la LED RX) pour que le bruit inter-paquets ne
soit pas entendu pendant que la boucle principale est bloquée. La C-Board
bascule automatiquement en décodage APRS dès que le VFO RX de la radio est
144–148 MHz (d'après la réponse HELLO).

*Rien sur la vue RX ?* Vérifie la console USB du RP2040 : une ligne
`[aprs] mode APRS` doit apparaître (le HELLO de la radio doit rapporter
144–148 MHz), puis `[aprs] #N SRC>...` à chaque décodage. Pas de décodage =
problème de niveau / accord sur le tap C-Board, comme pour SARSAT. Le RP2040 a
un **limiteur FI** donc un signal *fort* ne distord plus le démod — mais
surveille le champ `clip=NN.N%` dans la ligne `[aprs]` : au-delà de ~2 % l'ADC
butte et il faut baisser le gain AF (`Squelch fast` + curseur AF gain, ou le
volume de la radio).

*La LED TX clignote pendant la RX ?* C'est l'**auto-balise** — mets
**Interval = OFF** pour les tests RX seuls. `APRS_Beacon()` / `APRS_TimeSlice()`
temporisent aussi tant que le canal est occupé (`FUNCTION_RECEIVE` /
`FUNCTION_MONITOR`), donc une balise ne clé plus par-dessus un paquet entrant.

### Budget flash

`build.sh` désactive `ENABLE_SPECTRUM`, `ENABLE_FMRADIO` (broadcast),
`ENABLE_VOX`, `ENABLE_FLASHLIGHT` et `ENABLE_AUDIO_BAR`. SARSAT + écran niveau +
TX APRS (canal dédié 170) + écran config + vue RX structurée avec icônes 16x16 +
`MAIN_SCREEN=moto` : **59 060 o sur 61 440, ~2,3 Ko libres** (`id91` −332 o,
`stock` −640 o). Même text avec ou sans le plafond à 170 canaux
(`misc.h.diff`) — seule la table `gMR_ChannelAttributes` rétrécit en bss.
- Le RP2040 envoie `CLEAR` + un `0x06C1` par ligne à ~8 ms d'écart ;
  `APP_RunSarsat()` vide avec
  `while (UART_IsCommandAvailable()) UART_HandleCommand();` pour qu'aucune ligne
  ne soit perdue par l'anneau UART de 256 octets.
- **L'entrée est gardée :** `APP_RunSarsat()` retourne immédiatement (laissant
  `gSarsatShowRequest` en attente pour le prochain tick de 10 ms) sauf si la
  radio est au repos sur `DISPLAY_MAIN`. Ouvrir cet écran bloquant par-dessus un
  menu laissait l'affichage et l'état VFO en vrac — écran n'importe quoi, bloqué
  en MR après un cycle d'alimentation.
- **La sortie est synchrone et complète.** L'écran peut être auto-ouvert depuis
  le tick de 10 ms, pas seulement par une touche, donc il ne peut pas s'appuyer
  sur la queue du gestionnaire de touches pour finir le nettoyage. Sur EXIT il :
  `FUNCTION_Select(FUNCTION_FOREGROUND)` (quitte `FUNCTION_MONITOR`, sinon la
  touche suivante « arrête juste le monitoring » et semble morte), restaure
  `gEeprom.RX_VFO`, **`RADIO_ConfigureChannel(0/1, VFO_CONFIGURE)`** (relit les
  deux VFO depuis l'EEPROM et *valide* `gEeprom.ScreenChannel` — une valeur
  parasite s'affichait comme **« M128, sans nom/freq » et bloquée**, parce que
  le tick n'a pas de queue de gestionnaire de touches pour lancer la reconfig),
  `RADIO_SelectVfos()` + `RADIO_SetupRegisters(true)`, `BACKLIGHT_TurnOn()` +
  `gKeyLockCountdown = 30`, `GUI_SelectNextDisplay(DISPLAY_MAIN)` **maintenant**,
  et efface `gSarsatShowRequest`.
- **Fréquence :** le VFO *actuellement sélectionné* (`gEeprom.TX_VFO`,
  `gTxVfo->pRX->Frequency`). À l'entrée il force `gRxVfo = gTxVfo` /
  `gEeprom.RX_VFO = gEeprom.TX_VFO` et ré-accorde ; à la sortie
  `RADIO_SelectVfos()` recalcule le vrai VFO RX (dual-watch / cross-band).
- **Squelch :** forcé ouvert. L'entrée appelle
  `APP_StartListening(FUNCTION_MONITOR)` (→ `AUDIO_AudioPathOn()`,
  `gEnableSpeaker = true`, squelch désactivé) pour que le tap C-Board entende
  toujours la RF. `gMonitor` est effacé à la sortie.
  *(Un simple `FUNCTION_Select(FUNCTION_MONITOR)` ne pose qu'un flag et n'ouvre
  pas le chemin audio — c'était le bug de première version.)*
- **Démodulation : `MODULATION_FM` (le discriminateur FM)**, FI WIDE, plus les
  bits 10/9/8 de REG_2B pour couper dé-emphase RX / HPF300 / LPF3k pour que les
  transitions bi-phase-L passent à plat. Appliqué **après** `APP_StartListening`
  (qui remet la modulation à celle du VFO).
  - **Pas** `MODULATION_RAW` / `BK4819_AF_BASEBAND1` : c'est une sortie brute
    façon BLU ; elle bat le fort porteur de balise 406 MHz contre la fréquence
    accordée en un **sifflement continu**. Le discriminateur transforme une
    petite erreur d'accord en une lente dérive DC à la place, que le slicer
    RP2040 supprime.
  - Si le souffle de l'AF plat est pire que ce que la platitude apporte,
    supprime l'écriture REG_2B dans `app/sarsat.c` → FM large simple.

  Le même profil discriminateur plat est maintenant aussi une **modulation
  sélectionnable par l'utilisateur, `DSC`**, dans le menu `Demodu`
  (`patch/radio.h.diff` ajoute `MODULATION_DISCRI`, `patch/radio.c.diff`
  pose/efface les bits de dé-emphase REG_2B dans `RADIO_SetModulation` — tout
  autre mode restaure le filtrage stock). `Demodu → DSC` (+ `W/N → Wide`) donne
  le profil SARSAT sur un canal normal, p. ex. pour alimenter la C-Board sur le
  canal APRS 144.8. L'écran SARSAT force toujours son propre FM+WIDE+REG2B quel
  que soit le `Demodu` du canal.

UART : inchangé. Le diviseur DualTachyon `UART1->BAUD = Frequency / 39053` donne
déjà ≈ 38400 8N1, ce qu'utilise le côté RP2040 (`CFG_RADIO_UART_BAUD 38400`).

## Commandes série gérées (ajoutées à `UART_HandleCommand`)

| ID | charge utile | action | réponse (`ID \| 0x8000`) |
|----|---------|--------|------------------------|
| `0x06C0` | — | efface le tampon de lignes + reset défilement | `{status:u8}` |
| `0x06C1` | `line:u8, invert:u8, ascii[0..20]` | définit une ligne ; invert → gras ; auto-ouverture | `{status:u8}` (1 = mauvais index de ligne) |
| `0x06C2` | (réservé, ignoré) | — | `{status:u8}` |
| `0x06CF` | `proto_ver:u8` | keepalive | `{vfo:u8, modulation:u8, rx_freq:u32 LE 10 Hz, screen_open:u8, proto:u8}` |

**Phase 2 (radio → RP2040) :** chaque `0x06Cx` reçoit une réponse via
`SendReply()` (rendue non-static dans `patch/app_uart.c.diff`). La réponse
`0x06CF` porte l'état live de la radio pour que le RP2040 puisse logger le lien
et alerter si la radio n'est pas en FM / pas près de 406 MHz. `modulation` est
rapportée comme FM tant que l'écran SARSAT est ouvert (il force FM), sinon la
modulation propre du VFO sélectionné.

Protocole complet : `../../docs/protocol.md`.

## La règle des 18 glyphes (pourquoi l'affichage corrompait l'EEPROM)

`UI_PrintStringSmall*` **ne tronque pas**. À la colonne `Start = 2` qu'utilisent
ces écrans, la police 6px de glyphe / pas 7px déborde la fin d'un
`gFrameBuffer[row]` (128 octets) après **18 glyphes** — les octets en trop
tombent dans la ligne *suivante*, et depuis la ligne du bas (6) ils tombent dans
**`gEeprom`**, que l'éditeur de liens place 2 octets après `gFrameBuffer`. Ça
corrompait `ScreenChannel` (« bloqué sur M128 »), `VFO_OPEN`, `KEY_LOCK`,
`DUAL_WATCH`… et s'affichait comme du n'importe quoi au début de la ligne du
dessous.

Toute chaîne envoyée à `UI_PrintStringSmall*` dans `sarsat.c` / `aprs.c` fait
maintenant `<= 18` glyphes : `SARSAT_LINE_CHARS` 21 → 18, la barre de l'écran
niveau 21 → 16, tous les en-têtes / chaînes de verdict raccourcis, le texte RX
APRS 18 et formaté compact (indicatif source + info enroulée, sans dst / chemin
digi). Le RP2040 coupe aux mots le décodé SARSAT et le paquet APRS à 18 aussi.

## Builds anciens connus-mauvais — récupération EEPROM

`gEeprom` est lié **2 octets après `gFrameBuffer`** (`gFrameBuffer[7][128]` suivi
immédiatement de `gEeprom[256]`). Un build ancien de ce fork écrivait du texte
en « ligne 7 » du LCD dans l'écran SARSAT (`SARSAT_VIS_ROWS` valait 7, et la
ligne de verdict de l'écran niveau était 7) — la zone à l'écran ne fait que
7 lignes (`gFrameBuffer[0..6]`), donc ces écritures tombaient **dans `gEeprom`**,
corrompant `ScreenChannel` / `VFO_OPEN` / `KEY_LOCK` / `DUAL_WATCH`…
Symptômes après un décodage + un cycle d'alimentation : écran n'importe quoi,
**bloqué en MR avec F+3 (VFO/MR) mort**, clavier verrouillé, réglages
APRS/autres remis à zéro.

Les builds actuels n'écrivent rien au-delà de la l.6, donc ils ne causent pas
de nouvelle corruption — mais **reflasher ne répare pas l'EEPROM**.
Récupération :

1. Flasher un build actuel.
2. Clavier verrouillé ? Appui long sur **`#`** (la touche F) pour déverrouiller.
3. **F + 3** → retour en mode VFO. (`patch/settings.c.diff` force aussi
   `VFO_OPEN = true` à chaque boot, puisque 0.3q n'a pas de menu pour ça — ça
   seul ré-autorise F+3.)
4. Corriger squelch / dual-watch / cross-band depuis le menu s'ils semblent
   mauvais.
5. Si toujours incohérent : **reset usine** du menu, puis reprogrammer les
   canaux.

## Changements par rapport à l'arbre d'origine

Nouveaux fichiers :
- `app/sarsat.c`, `app/sarsat.h`  (`patch/sarsat.{c,h}`)

Fichiers patchés (`patch/*.diff`, appliqués avec `patch -p0`) :

| fichier | changement |
|------|--------|
| `ceccommon.h` | chemins `#include "driver\x.h"` Windows → `driver/x.h` pour compiler avec GCC. *(bug de portabilité préexistant dans 0.3q, sans rapport avec SARSAT — appliqué par `build.sh` via `sed`, voir le diff pour référence.)* |
| `app/uart.c` | `#include "app/sarsat.h"` / `aprs.h` ; `case 0x06C0/C1/C2/C3/CF:` → `SARSAT_HandleUART()`, `case 0x06D2/D3/D5:` → `APRS_HandleUART()` ; **`SendReply()` dé-`static`ée** pour la Phase 2 |
| `app/app.c` | `#include "app/sarsat.h"` / `aprs.h` ; tâche 10 ms : `if (gSarsatShowRequest) APP_RunSarsat();`, `APRS_TimeSlice()` + `if (gAprsShowRequest) APP_RunAprs()` ; garde rétroéclairage `APP_StartListening` `&& !APRS_QuietBacklight()` ; inhibition `gSchedulePowerSave` `\|\| APRS_KeepAwake()` |
| `ui/status.c` | `#include "app/aprs.h"` ; un symbole GPS 5×5 dans la barre haute après l'indicateur VOX — absent (pas de GPS C-Board), clignotant (recherche) ou fixe (verrouillé), d'après `APRS_GpsState()` |
| `app/main.c` | `#include "app/sarsat.h"` / `aprs.h` ; F+8 → `APP_RunSarsat()`, F+5 → `APP_RunAprs()` |
| `radio.c` | `RADIO_SetModulation()` utilise `gEeprom.DAC_GAIN` au lieu de forcer le gain DAC AF à 0xF, pour que l'override de gain AF survive à un ré-accord FM |
| `settings.c` | `gEeprom.VFO_OPEN = true` inconditionnel dans `SETTINGS_InitEEPROM()` — 0.3q n'a pas de menu pour ça, et ça répare tout seul une radio dont `0x0E7F` a été écrasé par le bug de ligne-7 ancien (voir *récupération EEPROM* ci-dessus) |
| `Makefile` | `ENABLE_SARSAT ?= 1`, `ENABLE_APRS ?= 1` (+defines, +`app/sarsat.o` `app/aprs.o` `app/ax25.o`) ; `ENABLE_BYP_RAW_DEMODULATORS` défaut `0 → 1` ; `MAIN_SCREEN ?= stock` ; `VERSION_STRING = CEC3qSAR` |
| `radio.h` | `MODULATION_DISCRI` (le démod `DSC`) ajouté à l'enum, avant `MODULATION_UKNOWN` |
| `misc.h` | `MR_CHANNEL_LAST` 199 → **169** (200 canaux mémoire → **170**, comme le projet CEC d'origine) ; `FREQ_CHANNEL_FIRST/LAST` et `NOAA_CHANNEL_FIRST/LAST` décalés de -30 pour rester contigus (les adresses EEPROM réelles des canaux fréquence/NOAA ne bougent pas, seuls leurs numéros logiques changent) ; `gMR_ChannelAttributes[207]` → `[177]` (`FREQ_CHANNEL_LAST + 1`) |

`APP_RunSarsat()` est une boucle plein écran bloquante modelée sur
`APP_RunSpectrum()` (le firmware K5 ne fait tourner aucun watchdog matériel,
donc bloquer est sûr). Il pompe
`UART_IsCommandAvailable()/UART_HandleCommand()` lui-même pour que le texte
continue de se mettre à jour en direct tant que l'écran est ouvert.

## Non fait / notes

- Balises 2ᵉ génération (SGB) : hors périmètre (le RP2040 ne décode que le 1G).
- `0x06C2 SARSAT_BEACON` (struct binaire) est accepté mais ignoré ; le RP2040
  pilote actuellement tout via le texte `0x06C1`.
- Pas de nouvelle entrée de menu / réglage EEPROM pour SARSAT — la fonctionnalité
  est à la compilation (`ENABLE_SARSAT`) et ne demande aucune configuration.
- Pas encore testé sur matériel.

## Digipeater APRS WIDEn-N (2026-09-05)

Ajouté en miroir du port K1/K5V3 (même demande, faite pour les deux
firmwares) : menu APRS (F+5) champ **Digi** (Off/WIDE1/WIDE1+2/WIDE1+2+3),
`APRS_PushConfig()` enfin câblée (`0x06D0`, était un stub mort), refactor
`APRS_Beacon()` → `APRS_TxFrame()` partagée avec la nouvelle
`APRS_Digipeat()` (`0x06D6`, RP2040 → radio). La décision WIDEn-N elle-même
vit entièrement côté RP2040 (`rp2040/src/aprs_digi.c`/`.h`, partagé par les
deux firmwares) — voir `docs/protocol.md` (« Digipeater WIDEn-N ») et
`firmware/uv-k1-k5v3/patch/integration.md` pour le détail complet, identique
ici à un point près : `SendReply()` prend ici `(pReply, Size)` sans
paramètre `Port` (ce firmware n'a qu'un seul UART, contrairement au
K1/K5V3), et le dispatch du nouveau `0x06D6` a été ajouté à `app_uart.c.diff`
(hunk `@@ -615,10 +621,32 @@` → `+621,33`) au lieu d'un ancrage `build.sh`.

Build vert, 0 warning : `text 59368 o` (+272 o).

**Retour terrain : « pas de répétition de la trame »** -- même cause que le
K1/K5V3 (voir `firmware/uv-k1-k5v3/patch/integration.md` pour le détail) :
`0x06D6` arrive juste après la salve à répéter, radio quasi certainement
encore en réception, une seule tentative sans retry ; et le popup RX (qui
s'ouvre justement sur ce paquet) bloque `APRS_TimeSlice()`. Même correctif
mirroré ici : `APRS_Digipeat()` met la trame en file, `APRS_DigipeatTimeSlice()`
retente à chaque tick **et** à chaque itération de la boucle de
`APP_RunAprs()`, abandon après ~3 s. Build vert, 0 warning : `text 59652 o`
(+284 o).

**Confirmé fonctionnel sans popup.** Suite (détail complet dans
`firmware/uv-k1-k5v3/patch/integration.md`, décision partagée
`rp2040/src/aprs_digi.c`) :

- **Indicatif inséré à chaque saut** : après précision de l'utilisateur, le
  digipeat traçable insère `INDICATIF-SSID*` devant l'alias `WIDEn` (qui
  garde son nom, SSID décrémenté, bit H posé une fois à 0) à *chaque* saut,
  pas seulement le dernier -- `WIDE2-2` -> `INDICATIF-SSID*,WIDE2-1` ->
  `IND1*,IND2*,WIDE2*`. Uniforme WIDE1/2/3.
- **Deux verrous anti-boucle** : jamais de répétition si la source de la
  trame = notre indicatif, ni si notre indicatif figure déjà dans la liste
  digi (ne périme pas).
- **Petit délai de contenance** ~300 ms avant la 1ʳᵉ tentative d'émission.
- **Rétroéclairage "Light on frame" avec Popup off** : `APRS_QuietBacklight()`
  ne regardait pas `popup_s` -> le rétroéclairage stock restait supprimé
  même popups désactivés. Corrigé : exige maintenant `popup_s != 0`.
- **Non-répétition avec popup ouvert -- corrigé** : `APRS_CanTransmitNow()`
  testait `gCurrentFunction == FUNCTION_RECEIVE` (= « squelch fermé »
  d'après `functions.h`, mauvais état), et `gCurrentFunction` se fige
  pendant que la boucle du popup bloque le tick. Remplacé par un test
  direct sur `g_SquelchLost` (porteuse présente), tenu à jour en temps réel
  dans les deux contextes.

`test_aprs_digi` : 19 cas verts. Build vert, 0 warning : `text 59672 o`.
**Non testé sur l'air.**


## Message « report balise 121 MHz » (2026-09-06)

Demandé : un menu pour envoyer un message APRS canned à un destinataire fixe,
chemin fixe `WIDE1-1,WIDE2-2`, pour un compte-rendu de radiogoniométrie sur
balise de détresse 121,5 MHz. **FAIT, non testé matériel.** 100 % côté radio,
aucun changement RP2040 (l'accusé de réception passe par le décodage message
0x06D3 déjà en place).

- **`aprs_cfg_t` +16 o** (40 -> 56, 7 pages EEPROM) : `char msg_to[10]` =
  destinataire (adressee APRS, <= 9 car., ex. `F4DVK-7`) + `_rsv[6]` de
  bourrage (la boucle d'écriture EEPROM de `APRS_Save()` traite la struct par
  blocs de 8 o, donc la taille doit rester multiple de 8). `APRS_Init()`
  assainit `msg_to` (une EEPROM d'un build antérieur y a du 0xFF / des restes
  DTMF -> vidé si un octet n'est pas ` A-Z0-9-`).
- **Deux nouveaux champs menu** (F+5) : `To <call>` (édition caractère par
  caractère, jeu ` A-Z 0-9 -`) et `Send report` (montre l'état :
  `Send report` / `Report TX n/3` / `Report ACK OK` / `Report no ack`).
- **Assistant d'envoi** (MENU sur `Send report`) : demande `Signal ?` (1 chiffre
  0-9) ; si > 0, demande `Direction ?` (0-359, les chiffres qui feraient
  dépasser 359 sont refusés) ; si 0, pas de question direction et `Dir: KO`.
  `A` = envoyer, `EXIT` = annuler. Saisie sur le même schéma clavier que
  Lat/Lon.
- **Trame** : `INDICATIF-SSID>APZSAR,WIDE1-1,WIDE2-2:` + info
  `:DESTINATAIRE:Report Balise 121 MHz S: <n> Dir: <ddd|KO> <lat> <lon>{NN`.
  Chiffres bruts (pas de zéro de tête), « 121 MHz » littéral, `{NN` = numéro
  de message APRS (cycle 1..99). `<lat> <lon>` = position résolue
  (`APRS_MyPosition()` : fix GPS si mode GPS, sinon lat/lon manuel) en
  **degrés décimaux, 4 décimales, signées** (`-` = S / O), tronquées ; ajoutée
  telle quelle, `0.0000 0.0000` si aucune position n'est réglée. Construite
  avec `ax25_build_ui()` + émise par `APRS_TxFrame()` (partagés avec la
  balise), garde NOCALL + CSMA (`APRS_CanTransmitNow()`). Longueur pire cas
  ~68 o < `AX25_MAX_INFO` (80).
- **Accusé de réception** : `APRS_MsgTimeSlice()` (appelée du tick **et de la
  boucle `APP_RunAprs()`** -- donc les trames rentrantes sont bien traitées
  même en restant dans le menu APRS : la boucle appelle
  `UART_IsCommandAvailable()/UART_HandleCommand()` à chaque itération)
  ré-émet **3 fois**, **30 s** d'intervalle (`APRS_MSG_RETRY_10MS`, sur les
  ~90 premières s), puis **reste en écoute jusqu'à 5 min**
  (`APRS_MSG_WAIT_10MS`) avant de conclure `Report no ack`. Un vrai accusé
  APRS (digipeaté aller-retour, ou renvoyé par un i-gate) arrive
  couramment après la fenêtre de ré-émission -- la 1ʳᵉ version passait en
  `FAIL` à ~90 s et `APRS_MsgCheckAck()` ignorait alors l'accusé tardif
  (bug remonté : « le popup `ack01` arrive après avoir quitté le menu mais
  la ligne ne passe pas OK »). `APRS_MsgCheckAck()` accepte désormais aussi
  un accusé reçu **après** l'abandon (`FAIL -> ACK`), et compare l'**indicatif
  de base** de l'adressee (SSID toléré : certains clients accusent l'indicatif
  nu) + le numéro de message. `ackNN` -> `Report ACK OK`.
- **Numéro de message affiché** (2ᵉ remontée : « ack `dans les temps`, log
  `[aprs] #29 F4DVK direct ack01`, mais ne passe pas OK ») : `s_msg.seq`
  s'incrémente à **chaque** report envoyé (rebond 1..99) et les 3 ré-émissions
  d'une session partagent ce numéro. Après plusieurs essais, la radio attend
  p.ex. `ack03` -- un `ack01` tardif (le destinataire n'a accusé le tout
  premier essai que maintenant) ne correspond alors plus. La ligne `Send
  report` affiche désormais **`Rpt #NN TX n/3`** / `Rpt #NN wait ack` /
  `Rpt #NN ACK OK` / `Rpt #NN no ack` pour que l'opérateur voie quel `ackNN`
  attendre. `s_msg.seq` repart de 1 après un reboot radio. Rétroéclairage
  rallumé au verdict. Sans C-Board branchée le résultat est `no ack` au bout
  des 5 min.
- **RP2040** `main.c` : la ligne `[aprs]` d'un **message** affiche
  `#N SRC direct  to [ADRESSEE] : TEXTE` (au lieu du format position) pour
  voir à quel indicatif / `ackNN` un accusé est adressé. `.uf2` régénérés.
- **3ᵉ remontée** (« le ack n'est toujours pas pris en compte », log
  `[aprs] #3 F4DVK direct to [F4DVK-14 ] : ack01` -- l'adressee est bien
  `F4DVK-14`, correctement formaté). Trois durcissements de
  `APRS_MsgCheckAck()` :
  1. **numéro accepté `1..s_msg.seq`** (au lieu de `== s_msg.seq`) : les
     reports retestés portent le même texte S/Dir, un client lent qui
     ré-accuse `ack01` (ou dédoublonne nos ré-émissions) confirme quand même
     la réception ; seul un numéro **au-delà** de notre seq est rejeté.
  2. **indicatif comparé base à base, les deux côtés élagués** :
     `gAprsCfg.call` peut traîner une espace en fin (éditeur char-cycle) ->
     `strcmp("F4DVK", "F4DVK ")` échouait.
  3. **diagnostic visible** : `APRS_MsgCheckAck()` mémorise `last_ack_no`
     (tout `ackNN` vu pendant l'attente) ; si la ligne reste bloquée elle
     affiche `#03 wait  got a01` / `#03 no ack got a01` -> l'opérateur voit
     que l'accusé est bien arrivé mais n'a pas matché (et quel numéro).
- **Menu APRS ouvert manuellement : décodage en arrière-plan, pas de popup**
  (demandé après plusieurs allers-retours). Un paquet reçu pendant qu'on est
  dans l'écran config (F+5) **ne bascule pas** sur la vue RX -- seul un
  auto-popup (écran fermé -> tick l'ouvre sur une trame) le fait, avec son
  timeout `popup_s` habituel. Le décodage et la reconnaissance de l'accusé de
  report (`APRS_MsgCheckAck` depuis `APRS_HandleUART` dans la boucle)
  tournent quand même : la ligne `Send report` passe à `ACK OK` sans rien
  interrompre. `*` montre la dernière trame RX à la demande.
- **Audio de la trame dans le menu** (remontées successives : « pas d'audio »
  -> « léger souffle, aucun souffle FM » -> « squelch met plusieurs secondes
  à couper le souffle » -> « le squelch s'ouvre mais pas d'audio de la
  trame »). **Cause racine** : le `RADIO_SetupRegisters()` de KD8CEC
  **n'appelle jamais `RADIO_SetModulation()` / `BK4819_SetAF()`** (seul
  `APP_StartListening()` le fait) -- donc après un TX AFSK (qui met
  `BK4819_SetAF(AF_MUTE)` dans `APRS_SendTones()`), le BF du BK4819 reste
  **muté**, et la boucle principale qui le rétablirait est bloquée par
  `APP_RunAprs()`. `APRS_TxFrame()` appelle maintenant
  `RADIO_SetModulation(gRxVfo->Modulation)` après son `RADIO_SetupRegisters()`
  -- toutes les voies d'émission laissent le BF ré-ouvert.
  Ouverture **manuelle** de l'écran sur 144-148 MHz -> `RADIO_SelectVfos()` +
  `RADIO_SetupRegisters(true)` + **`RADIO_SetModulation(gRxVfo->Modulation)`**
  (dé-mute le BF) + `gEnableSpeaker` + `APRS_ApplySquelch()` (fast-squelch) +
  `APRS_ApplyAfGain()` (réglage opérateur **conservé**). Le HP démarre
  **coupé**, le mini-squelch de la boucle ne l'ouvre que sur une vraie
  porteuse (silence entre paquets, pas de souffle permanent), coupure
  différée ~2 s. `APP_StartListening()` (essayé) forçait le HP ouvert ->
  souffle continu jusqu'à une transition squelch, retiré. `#include
  "app/app.h"` laissé (inoffensif).
- Host : `rp2040/test/host/test_aprs_parse` -- cas `ack` ajouté (adressee
  9 car. + corps `ack07` extraits).

**Place flash récupérée** (le message + la position avaient laissé ~24 o de
marge) : `build.sh` désactive maintenant `ENABLE_COPY_CHAN_TO_VFO` (~110 o,
raccourci « copier le canal courant vers le VFO ») et passe
`MAIN_SCREEN=moto` -> **`id91`** (~340 o : même disposition icom, mais
réutilise la police gros chiffres stock + le S-mètre horizontal stock au
lieu des tables de police propres à `moto`).

**Écran `id91` -- 2 derniers digits de la fréquence en demi-hauteur** (sur
demande, comme le faisait `moto`) : `patch/ui_main.c.diff`, branche de rendu
fréquence id91 -- au lieu de `UI_DisplayFrequency(e, ...)` sur les 9 chiffres,
on tronque à `"DDD.DDD"` (gros chiffres stock 13 px, l.1-2) et on dessine les
2 derniers (100 Hz + 10 Hz) via `UI_PrintStringSmallNormal()` calé en x sur la
fin du bloc gros chiffres, l.2 (bas des gros chiffres). Aucune table de police
ajoutée -- que des helpers stock déjà liés. La saisie manuelle de fréquence
faisait déjà ça (`a + 6` en petit).

**Réception coupée dans le menu APRS -- corrigé** (remonté : « dès que je suis
dans le menu APRS, pas de réception : la LED s'allume, du souffle, mais la
trame ne passe pas »). Deux causes :
1. **Émission depuis la boucle du menu** (une ré-émission de report -- surtout
   depuis que la fenêtre d'accusé est passée à 5 min -- ou un digipeat)
   appelle `RADIO_SetupRegisters(true)` à la fin de `APRS_TxFrame()`, qui
   remet REG_4E au squelch du canal et re-force le gain AF. Le tick qui
   ré-applique normalement le fast-squelch / gain APRS (`APRS_TimeSlice()`)
   est bloqué tant que `APP_RunAprs()` est ouvert -> RX dégradée pour le
   reste de la session menu. **Fix** : `APRS_TxFrame()` ré-applique
   `APRS_ApplySquelch()` + `APRS_ApplyAfGain()` en fin -- toutes les voies
   d'émission laissent maintenant une RX APRS propre. En plus, la boucle de
   `APP_RunAprs()` appelle `APRS_ApplySquelch()` à chaque itération.
2. **Le mini-squelch de la boucle coupait l'audio entre paquets rapprochés.**
   `SQUELCH_FOUND` -> `AUDIO_AudioPathOff()` immédiat ; les trames APRS
   arrivent en rafale à une fraction de seconde d'écart, couper le flux vers
   la C-Board entre elles casse sa PLL de bit / le framing HDLC. **Fix** :
   coupure du HP **différée de ~2 s** (`msq_mute_at`) -- le flux reste
   continu sur les petits creux, le canal n'est silencé qu'après un vrai
   blanc.

Build V1 vert, 0 warning après ces lots (message + position, place
récupérée, fenêtre d'accusé, RX menu) : `text 61060 o` -- marge ~380 o.

**Encore de la place récupérée** (sur demande) : `build.sh` désactive aussi
`ENABLE_SCAN_RANGES` (~250 o, balayage par plage de fréquences -- le balayage
par canal mémoire reste). Avec `COPY_CHAN_TO_VFO` déjà off :
marge ~630 o. Après les durcissements ack + RX-menu + mise en réception FM
du menu : `text 60992 o`.

**Passe d'optimisation flash** (sur demande) : `APRS_Beacon()` et `APRS_MsgTx()`
partageaient le montage src/dst + `ax25_build_ui()` + `APRS_TxFrame()` -->
factorisé en `APRS_TxInfo()` ; le diagnostic `last_ack_no` (« got aNN » sur la
ligne `Send report`) retiré maintenant que l'accusé fonctionne ; `#include
"app/app.h"` retiré (plus de `APP_StartListening`). **`text 60820 o`** --
marge ~620 o. Le gros consommateur restant est `APP_RunAprs()` (~4 Ko : écran
bloquant interactif -- assistant d'envoi, saisie clavier, mini-squelch, mise
en réception) ; le réduire davantage = couper des fonctions. `moto` reste
dispo (`MAIN_SCREEN=moto`) si un futur build a de la place.


## Mode TNC KISS (2026-09-06)

Demandé : un mode KISS activable au menu, qui désactive les autres fonctions
(tracker, popup), l'hôte se connectant au RP2040. **FAIT et VALIDÉ SUR
MATÉRIEL (2026-09-06) : RX et TX confirmés bout en bout avec `kissutil`**
(trame RX décodée proprement côté hôte, trame TX émise par la radio et reçue
par un autre poste).

Le gros du travail est côté RP2040 (`rp2040/src/kiss.c`/`.h`, testé hôte
`rp2040/test/host/test_kiss` -- 12 cas verts) : codec SLIP/KISS + bascule de
mode. Côté radio, c'est minuscule (~170 o) parce que **la voie d'émission
réutilise entièrement la file digipeat**.

- **`aprs_cfg_t::opts` bit 6** (`APRS_OPT_KISS`, `0x40`) -- struct inchangée.
- **Champ menu `KISS TNC on/off`** (`F_KISS`, après `Send report`). Toggle
  immédiat -> `APRS_PushConfig()` (pas d'attente de la sauvegarde).
- **`APRS_PushConfig()`** : 12ᵉ octet de charge = `flags`, bit 0 = KISS
  (`b[2]` passe de 11 à 12).
- **`APRS_TimeSlice()`** : `if (kiss) return;` après `APRS_DigipeatTimeSlice()`
  -- garde le fast-squelch + le gain AF + le relais des trames (le chemin
  `0x06D6 -> APRS_Digipeat -> APRS_TxFrame`), coupe l'auto-balise, le
  « report 121 MHz » et l'animation du symbole GPS.
- **`aprs_rx_arrived()`** : `if (kiss) return;` en tête (ceinture-bretelles :
  le RP2040 n'envoie pas de `0x06D3`/`0x06D2` en KISS).
- **`draw_config()`** : en-tête `KISS TNC  host USB` quand actif.
- Pas besoin de garder un écran ouvert : le tick suffit. Un VFO sur 144.800 FM.

Build V1 vert, 0 warning : **`text 60988 o`** (+168 o, marge ~450 o). Le
dispatch `0x06D6` était déjà câblé (digipeater), rien à changer dans `build.sh`.
