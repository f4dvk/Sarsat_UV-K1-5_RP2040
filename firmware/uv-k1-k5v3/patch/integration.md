# UV-K1 / UV-K5 V3 (base F4HWN) — intégration de l'écran SARSAT

## Contexte

Demande : porter le firmware SARSAT/APRS vers l'UV-K1 et l'UV-K5 V3, sur la
base **F4HWN** (`armel/uv-k1-k5v3-firmware-custom`, tag `v5.9.0`), avec
**toutes les fonctions du firmware V1** (KD8CEC), l'affichage classique
pouvant rester celui de F4HWN (pas besoin de reporter les écrans `moto`/`id91`
du firmware V1).

Bonne nouvelle constatée en portant : ce dépôt F4HWN descend de la **même
lignée egzumer/DualTachyon** que le KD8CEC `uvk5cec-0.3q` utilisé pour le V1
— `VFO_Info_t`, `UI_PrintString*`, `RADIO_ConfigureChannel`,
`SETTINGS_SaveChannel(Name)`, `gFrameBuffer[FRAME_LINES][LCD_WIDTH]`,
`BK4819_*` (implémenté par `driver/bk4829.c` mais sous les **mêmes noms** que
`driver/bk4819.h`, donc portable sans changement de nom de fonction) sont
quasi identiques champ pour champ, fonction pour fonction. Le port de l'écran
SARSAT a été **mécanique** : aucune ré-écriture de logique, seulement
l'adaptation de l'API UART (qui prend ici un `Port` explicite,
`UART_PORT_UART` vs `UART_PORT_VCP`, l'USB étant un port UART séparé sur ce
firmware).

Le RP2040 (`rp2040/`) et le protocole (`docs/protocol.md`) sont **inchangés** —
ce sont eux qui restent fixes, chaque firmware radio implémente juste les
commandes `0x06Cx`/`0x06Dx` de son côté.

## Méthode de fork

Contrairement au firmware V1 (patch `diff -p0` sur un Makefile GNU classique),
F4HWN utilise **CMake + des presets** (`Fusion`, `Custom`, `Bandscope`...) avec
~80 flags `ENABLE_*`. `build.sh` suit donc le modèle déjà utilisé par
`benshi-esp32-sim/firmware/uv-k1-k5v3/build.sh` pour son propre "mode hôte" :
clone `git` frais + insertions `perl` sur des ancrages de texte stables
(`perl -0pi -e 's{...}{...}'`), pas de fichiers `.diff`. Chaque ancrage est
vérifié après coup (`grep -q ... || exit 1`) — un échec de correspondance est
fatal plutôt que de compiler un firmware à moitié câblé.

Compilation : **`cmake --preset <PRESET>` + `arm-none-eabi-gcc` du `PATH`**,
sans Docker (le script `compile-with-docker.sh` amont existe mais le
toolchain CMake (`cmake/gcc-arm-none-eabi.cmake`) ne demande que
`arm-none-eabi-gcc`/`cmake`/`ninja` en local — testé avec succès en
13.2.1, cmake 3.25, ninja).

## Fait : écran SARSAT

`patch/sarsat.c` / `patch/sarsat.h` — copie quasi verbatim du module du
firmware V1 (`firmware/uv-k5v1-kd8cec/patch/sarsat.c`), avec seulement :

- `SendReply(void*, uint16_t)` → `SendReply(uint32_t Port, void*, uint16_t)`
  (ce firmware a deux ports UART logiques, matériel + USB-CDC ; on parle
  toujours sur `UART_PORT_UART`, le port physique vers la C-Board) ;
- `UART_IsCommandAvailable()`/`UART_HandleCommand()` → même chose + `(UART_PORT_UART)` ;
- rien d'autre — `VFO_Info_t`, `RADIO_SetModulation(MODULATION_FM)`,
  `BK4819_SetFilterBandwidth`, `BK4819_REG_2B` (bypass dé-emphase/HPF/LPF pour
  laisser passer le bi-phase-L FGB à plat), `APP_StartListening`,
  `RADIO_ConfigureChannel`, `UI_PrintStringSmall*`, `gFrameBuffer` : identiques.

Câblage (`build.sh`, ancrages `perl`) :

| fichier | changement |
|------|--------|
| `App/app/app.c` | `#include "app/sarsat.h"` ; dans `APP_TimeSlice10ms()`, juste après le service `UART_PORT_UART`, `if (gSarsatShowRequest) { ...; APP_RunSarsat(); }` |
| `App/app/uart.c` | `#include "app/sarsat.h"` ; `SendReply()` dé-`static`ée ; `case SARSAT_CMD_CLEAR/TEXT/LEVEL/HELLO/BEACON:` → `SARSAT_HandleUART(pUART_Command->Header.ID, pUART_Command->Buffer + sizeof(Header_t), pUART_Command->Header.Size)` dans le `switch` de `UART_HandleCommand()` |
| `App/CMakeLists.txt` | `enable_feature(ENABLE_SARSAT app/sarsat.c)` juste après `ENABLE_UART_RW_BK_REGS` |
| `CMakePresets.json` | `"ENABLE_SARSAT": false` par défaut (activé explicitely par `build.sh` via `-DENABLE_SARSAT=ON`) |

**Ouverture manuelle de l'écran** : contrairement au V1 (raccourci fixe F+8),
F4HWN assigne les fonctions custom à des touches via le menu (`F1Shrt` /
`F1Long` / `F2Shrt` / `F2Long`) — nouvelle entrée `ACTION_OPT_SARSAT`
(`App/settings.h`, `App/app/action.c` : `[ACTION_OPT_SARSAT] = &APP_RunSarsat`,
`App/ui/menu.c` : `{"SARSAT", ACTION_OPT_SARSAT}` dans
`gSubMenu_SIDEFUNCTIONS`). Sur la radio : **MENU → F1Shrt (ou F1Long/F2Shrt/
F2Long) → choisir SARSAT**, puis la touche assignée ouvre l'écran à tout
moment. L'ouverture **automatique** sur trame décodée (`gSarsatShowRequest`,
hook `APP_TimeSlice10ms`) fonctionne aussi, indépendamment.

> **État final (voir la section « oui, F+8 sur UV-K1 » plus bas) :** comme
> sur le V1, l'écran SARSAT est maintenant sur **F+8** et l'écran APRS sur
> **F+5**, en dur (`case KEY_8` / `case KEY_5` de `App/app/main.c`).
> L'assignation par le menu reste possible en plus.

**Démodulation « DSC » (discriminateur à plat)** : déjà présente en amont sous
le nom **`MODULATION_RAW`** (`App/driver/bk4829.c:BK4819_EnterRaw()`, câblée
dans `RADIO_SetModulation()` `App/radio.c`), sous `ENABLE_BYP_RAW_DEMODULATORS`
— **éteint par défaut** sur le preset `Fusion`, activé par `build.sh`
(`-DENABLE_BYP_RAW_DEMODULATORS=ON`). Contrairement au nom, ce n'est **pas**
un mode baseband/IQ qui bat la porteuse : `BK4819_EnterRaw()` garde
`BK4819_SetAF(BK4819_AF_FM)` (le discriminateur) et débraye juste REG_2B
bits 10/9/8 (HPF300/LPF3K/dé-emphase RX). Pas besoin de dupliquer l'entrée ni
de la renommer : `*pMax` du menu Demodu suit `ARRAY_SIZE(gModulationStr)`,
donc `RAW` apparaît automatiquement dans le sélecteur de modulation dès que
le flag est actif — aucun patch `menu.c` nécessaire.

> **Précision (question "RAW est-il vraiment identique à DSC ?", suite à un
> retour terrain) : pas tout à fait, sur deux plans.**
> - **Registre** : `BK4819_EnterRaw()` (RAW, menu) et le profil interne de
>   l'écran SARSAT (`RADIO_SetModulation(MODULATION_FM)` + OR manuel de REG_2B
>   dans `sarsat.c`) partagent bien les mêmes bits REG_2B, mais **`RADIO_SetModulation()`
>   ne les traite pas dans la même branche** : RAW passe par le chemin
>   `MODULATION_BYP`/`MODULATION_RAW` (retour anticipé) qui écrit
>   **`REG_3D = 0x0000`** et **désactive l'AFC** ; le chemin FM normal
>   (utilisé par l'écran SARSAT) écrit **`REG_3D = 0x2AAB`** et **laisse l'AFC
>   active**. Le rôle exact de REG_3D n'est pas documenté même dans les
>   commentaires F4HWN eux-mêmes (« It is not clear why these specific
>   register values are used... nor what exactly they do »). Différence
>   réelle, mais pas forcément un défaut : l'AFC active pendant l'écran SARSAT
>   aide à suivre une dérive de fréquence sur un signal faible/lointain,
>   l'avoir désactivée en mode RAW "générique" (pensé pour alimenter un
>   appareil externe comme la C-Board) évite que la radio corrige la
>   fréquence dans le dos du décodeur externe. **L'écran SARSAT n'utilise
>   jamais le RAW du menu — il pose toujours son propre profil en interne,
>   donc ceci n'affecte pas le décodage SARSAT lui-même.**
> - **Protocole (la vraie cause du message `radio not in FM/DSC` vu sur le
>   terrain)** : `rp2040/src/main.c` a une table `mod_name[] = {"FM","AM",
>   "USB","BYP","RAW","DSC"}` et `MOD_IS_FM_LIKE(m) = (m==0 || m==5)` — écrite
>   pour l'énumération du firmware **V1**, où l'octet 4 ("RAW") est son mode
>   baseband/IQ qui bat la porteuse (à raison exclu de la liste FM-like) et
>   l'octet 5 ("DSC", `MODULATION_DISCRI`) est le mode discriminateur à plat
>   ajouté là-bas. **Le `MODULATION_RAW` de F4HWN est ce même profil
>   discriminateur à plat, mais il occupe l'octet 4 dans *son propre* enum**
>   — collision de valeur avec le "RAW" (différent, mauvais) du V1, d'où
>   l'avertissement erroné du RP2040 quand le canal est laissé sur `RAW` en
>   dehors de l'écran SARSAT. **Corrigé** : `SARSAT_ReplyStatus()`
>   (`patch/sarsat.c`) reformule l'octet transmis au RP2040 — `MODULATION_RAW`
>   local est envoyé comme **5 ("DSC")** sur le fil, pas sa valeur native 4,
>   pour correspondre à ce qu'il est réellement plutôt qu'à sa position dans
>   l'enum F4HWN. +4 o.

L'écran SARSAT pose ce même profil lui-même en interne (`sarsat.c`,
indépendant du réglage du canal) ; avoir `RAW` aussi dans le menu sert
surtout à alimenter la C-Board sur un canal normal (144.8 pour l'APRS, une
fois porté), comme sur le V1.

**Réglage de gain C-Board** (validé sur matériel, demandé après pour pouvoir
mettre le potentiomètre volume à fond) : `patch/afgain.c`/`.h`, nouveau
module autonome (pas de dépendance à `ENABLE_APRS`, contrairement au V1 où
c'était un champ de `aprs_cfg_t`) — `gAfGain` (0/hors 1..78 = auto : laisse
le potentiomètre piloter le gain ; 1..78 = niveau fixe, même calcul de
curseur que `APRS_ApplyAfGain()` du V1 : d'abord AF Rx Gain-2 63→8, puis le
gain DAC 15→0). `AFGAIN_Apply()` écrit `gEeprom.VOLUME_GAIN`/`.DAC_GAIN` puis
appelle `BK4819_SetRxAudioGain()` (fonction déjà présente en amont,
`App/driver/bk4829.c`, déjà utilisée par `RADIO_SetupRegisters()` juste après
`RADIO_SetModulation()` — donc pas besoin de patcher `RADIO_SetModulation()`
comme sur le V1 : appeler `AFGAIN_Apply()` **après** `RADIO_SetupRegisters()`/
`APP_StartListening()` dans `APP_RunSarsat()` suffit, on est le dernier à
écrire REG_48). Persisté dans l'EEPROM émulée (SPI NOR + table de mapping
`App/driver/eeprom_compat.c`) : nouvelle entrée réservant 8 o à `0x00A170`
(la queue non revendiquée du secteur "Settings", juste après le dernier bloc
mappé `0x00A160→0x00A170`) — même secteur physique que les réglages radio,
donc protégé du reset normal, effacé seulement par « reset ALL ». *Attention
avec cette table de mapping* : une adresse EEPROM non déclarée dans
`ADDR_MAPPINGS` ne pointe **nulle part** (écritures silencieusement perdues,
lectures = `0xFF`) — impossible d'improviser une adresse « probablement
libre » comme sur le V1, il faut une entrée `_MK_MAPPING` explicite.
**Interface** : reprend le mécanisme déjà présent dans `sarsat.c` (vue niveau,
touche `5` pour y basculer) — UP/DOWN y règlent maintenant `gAfGain` en
continu (barre rms + ligne « AF gain: N » comme retour), sauvé en EEPROM à la
sortie de la vue niveau ou de l'écran. C'est *le menu* demandé : pas besoin
d'un écran séparé tant que l'APRS (et son propre menu F+5) n'est pas porté.
Re-assertion ~2×/s pendant que l'écran est ouvert (une recalibration menu
ailleurs pourrait sinon remettre le gain stock).

> **Bug corrigé (retour terrain 2026-09-04) : perte de sensibilité dans
> l'écran SARSAT, reproductible même en « Main Only » (donc rien à voir avec
> le double-veille / VFO actif, piste initialement explorée puis écartée).**
> `s_stock_volgain` (le gain "stock" que le mode `auto` restaure) est capturé
> **une seule fois** dans la vie de la variable `static`, à la toute première
> ouverture de l'écran depuis le boot — puis reste figé à cette valeur pour
> le reste de la session, même si le potentiomètre volume est tourné
> **après coup**. Si cette première capture a eu lieu pot bas (essai
> antérieur, ou juste après un flash), `auto` re-clampe silencieusement le
> gain audio bas à **chaque** ouverture suivante, quoi que fasse le
> potentiomètre — perçu comme « il faut un signal plus fort ». Corrigé par
> `AFGAIN_ResyncKnob()` (nouvelle fonction, remet `s_stock_volgain` à
> "non capturé"), appelée juste avant le premier `AFGAIN_Apply()` à
> **chaque** ouverture de `APP_RunSarsat()` — `auto` capture donc la position
> réelle du potentiomètre à l'instant où l'écran s'ouvre, pas une valeur
> périmée. **Même bug corrigé en miroir sur le firmware V1**
> (`APRS_ResyncAfGainKnob()` dans `patch/aprs.c`, appelée dans
> `APP_RunSarsat()` et `APP_RunAprs()`) : le code source était identique
> (même `static uint8_t s_stock_volgain = 0xFF` jamais réinitialisé), donc
> le même défaut latent existait là aussi, juste pas encore remonté. +16 o
> (K1/K5V3) / +20 o (V1).

**Build vert, 0 warning**, preset `Fusion` (celui de `benshi-esp32-sim`, tous
les à-côtés F4HWN activés — c'est *l'affichage classique F4HWN* demandé,
aucun écran custom n'est redessiné) :

```
Memory region         Used Size  Region Size  %age Used
             RAM:       13184 B        16 KB     80.47%
           FLASH:      114944 B       118 KB     95.13%
```

**⚠️ Marge flash très courte (~4,9 %, ~5,6 Ko) avec le preset `Fusion` complet.**
`Fusion` embarque *tout* F4HWN (Game, FoxHunt, Beam, RxTxLog, K5Viewer,
Aircopy, FM broadcast, Spectrum...) — comme pour le V1 (où SPECTRUM/FMRADIO/
VOX/FLASHLIGHT avaient dû être coupés pour faire de la place à SARSAT+APRS),
il faudra probablement désactiver quelques extras F4HWN inutiles ici avant
d'ajouter l'APRS/GPS/canal 170 (ax25.c + aprs.c pesaient à eux seuls plusieurs
Ko sur le V1). Pistes, du plus sûr au plus radical : `-DENABLE_FEAT_F4HWN_GAME=OFF
-DENABLE_FEAT_F4HWN_FOXHUNT=OFF -DENABLE_FEAT_F4HWN_BEAM=OFF
-DENABLE_AIRCOPY=OFF -DENABLE_FEAT_F4HWN_K5VIEWER=OFF
-DENABLE_FEAT_F4HWN_RXTX_LOG=OFF`, ou passer sur un preset plus léger
(`Custom`/`Basic`) si `Fusion` ne suffit vraiment plus.

**Validé sur matériel** (écran SARSAT, retour utilisateur 2026-09-04). Le
réglage de gain C-Board ci-dessus a été ajouté après ce retour, sur la base
de ce même build validé.

> **Bug corrigé (retour terrain, même jour) : le squelch matériel n'était
> pas réellement forcé ouvert dans l'écran SARSAT.**
> Symptômes remontés : « je ne reçois plus ma trame dans le menu SARSAT »
> (même en Main Only, donc pas un problème de VFO), « ça décode en auto sur
> le VFO mais plus dans le menu SARSAT », et surtout : « sur signal faible je
> n'entends pas la trame, juste un "poc" » — la pièce qui a permis de
> confirmer le mécanisme.
>
> **Cause** : `RADIO_SetupRegisters()` (`App/radio.c`) appelle toujours
> `BK4819_SetupSquelch()` avec les seuils **normaux du canal**
> (`gRxVfo->SquelchOpen/CloseRSSI/Noise/GlitchThresh`), **sans aucun cas
> particulier pour `FUNCTION_MONITOR`**. Sur ce firmware, « monitor » ne
> change que la façon dont le **logiciel** réagit à l'interruption squelch
> trouvé/perdu (via la tâche de fond 10 ms, que cet écran bloquant ne fait
> pas tourner) — il ne reprogramme jamais les seuils **matériels** du
> BK4819. Le squelch du canal continue donc de couper l'AF exactement comme
> en réception normale : sur un signal fort il reste ouvert assez longtemps
> pour tout laisser passer (d'où « ça marche parfois »), sur un signal
> faible/marginal il s'entrouvre à peine (le fameux « poc ») au lieu de
> tenir toute une salve FGB (~520 ms) — décodage impossible côté RP2040,
> alors que la même salve s'entend très bien sur un VFO classique (où la
> gestion squelch normale, elle, continue de tourner).
>
> **Corrigé** : `APP_RunSarsat()` (`patch/sarsat.c`) rappelle
> `BK4819_SetupSquelch()` explicitement, juste après le profil REG_2B/WIDE,
> avec les valeurs **« toujours ouvert »** — les mêmes que celles que ce
> firmware utilise déjà lui-même pour son propre réglage **SQL = 0 (off)**
> (`RADIO_ConfigureSquelchAndOutputPower()`, `App/radio.c`) : RSSI open/close
> = 0, noise open/close = 127, glitch open/close = 255. Valeurs reprises
> telles quelles (pas de seuils inventés) puisqu'elles sont déjà le
> mécanisme « squelch désactivé » officiel de ce firmware. +24 o.
>
> **Corrigé en même temps (bug secondaire trouvé en creusant le même
> problème)** : le réglage de gain fixe (`gAfGain` 1..78) n'était appliqué
> que **pendant que l'écran SARSAT est ouvert** — `AFGAIN_Apply()` n'était
> appelée que depuis `APP_RunSarsat()`. Dès la fermeture de l'écran, le
> suivi normal du potentiomètre (qui continue de tourner en fond, contrairement
> à la boucle bloquante de l'écran) reprenait la main sur
> `gEeprom.VOLUME_GAIN`/`.DAC_GAIN` — le niveau calé sur la vue niveau ne
> décrivait donc pas ce qu'utilisait le décodage en fond (auto-popup), sans
> rapport avec ce que l'écran utilisait une fois rouvert. Nouvelle fonction
> `AFGAIN_TimeSlice()` (`patch/afgain.c`/`.h`), appelée sans condition
> (écran ouvert ou pas) depuis `APP_TimeSlice10ms()`, même cadence ~500 ms
> que le réglage interne à l'écran — un gain fixe reste maintenant actif
> partout, tout le temps. Sans effet en mode `auto`. +52 o cumulé.
>
> Build vert, 0 warning : `FLASH 115024/120832 o (95,19 %)`.
>
> **⚠️ Retour terrain sur ce correctif squelch : régression, ANNULÉ.**
> Testé sur l'air : silence quasi total dans l'écran SARSAT, y compris sur un
> signal fort qui décodait très bien avant ce changement (« le précédent
> build j'avais la trame sur signal fort, mais il fallait corriger les
> signaux faibles »), et le curseur AF gain poussé à fond (78) ne compensait
> même pas. Net pire que le problème d'origine. L'appel
> `BK4819_SetupSquelch(0, 0, 127, 127, 255, 255)` a donc été **retiré** de
> `APP_RunSarsat()` (`patch/sarsat.c`) — malgré la correspondance exacte avec
> le propre "SQL = 0" de ce firmware, quelque chose dans cette combinaison
> (peut-être une interaction avec le profil REG_2B/WIDE déjà en place, peut-
> être autre chose) casse plus qu'il ne répare sur ce matériel précis. Cause
> du « poc » sur signal faible toujours **non identifiée** — hypothèse de
> travail invalidée, à reprendre avec des chiffres `[lvl]`/`[burst]` frais
> plutôt que par déduction pure. Le réglage de gain (`AFGAIN_TimeSlice`,
> ci-dessus) est conservé — confirmé souhaité par l'utilisateur (« le gain
> doit agir en mode VFO pour détection sarsat et après en mode APRS »),
> indépendant du problème de squelch. Build vert après retrait :
> `FLASH 115000/120832 o (95,17 %)`.
>
> **Nouveau log terrain analysé** : deux constats distincts. 1) « infos
> erronées » sur signal faible = un faux positif BCH (`hexID`
> `000000000000000`, pays 0/Inconnu, position 0,0) — corrigé **côté RP2040**
> (`rp2040/src/main.c`, commun aux deux firmwares radio) : rejet explicite
> des décodages BCH-OK dont le hexID est entièrement à zéro (aucune vraie
> balise COSPAS-SARSAT n'a un ID nul). `.uf2` RP2040 régénérés. 2) « pas de
> trame » en écran SARSAT : dans ce log l'audio arrive en continu (peaks
> ~12-13k, pas de silence comme le « poc ») — cohérent avec le retrait du
> correctif squelch, semble déjà amélioré ; la fenêtre sans décodage
> ressemble à une absence réelle de salve plutôt qu'un bug.
>
> **Essai en cours, non confirmé** : figer l'AGC (`RADIO_SetupAGC(false,
> true)`, `patch/sarsat.c`) suite à « j'entends de temps en temps un bout de
> trame comme un gain automatique ou scan sur la sortie discri » — l'AGC
> adaptatif du BK4819 qui « pompe » en cours de salve expliquerait des
> bribes audio hachées plutôt qu'un ton continu ~520 ms. `RADIO_SetupAGC()`
> est un paramètre déjà documenté et utilisé ailleurs dans ce même firmware
> (pas des registres devinés comme l'essai squelch) — `disable=true` fige
> l'AGC du BK4819 sur un index fixe (3) au lieu du mode adaptatif. **À
> tester** ; si ça dégrade comme l'essai squelch, revenir en arrière de la
> même façon plutôt que d'empiler d'autres essais sans données fraîches.
> Build vert, +8 o : `FLASH 115008/120832 o (95,18 %)`.
>
> **⚠️ ANNULÉ (2ᵉ essai qui n'a pas aidé)** : mesure au banc (générateur de
> signal) juste après ce build — une trame à **-107 dBm, audible en VFO
> normal, ne l'était plus du tout en écran SARSAT ; une trame à -77 dBm
> l'était** — un écart de sensibilité de classe ~30 dB. Pas établi si cet
> écart préexistait (le filtre WIDE forcé coûte de la marge SNR à lui seul)
> ou a été introduit/aggravé par le gel de l'AGC sur l'index fixe 3 (peut-
> être un cran de gain pensé pour la marge sur signal fort, inadapté à un
> signal faible). `RADIO_SetupAGC(false, true)` **retiré** par prudence,
> plutôt que d'empiler une 3ᵉ hypothèse. Build vert après retrait :
> `FLASH 115000/120832 o (95,17 %)`.
>
> **Prochaine étape proposée à l'utilisateur** : comparer au banc, à un
> **même niveau dBm connu**, les lignes `[lvl]` du RP2040 en VFO normal et
> en écran SARSAT (le RP2040 loggue peak/rms/dc même quand rien n'est
> audible) — ça donnerait l'écart de gain exact en dB au lieu de continuer
> à deviner registre par registre.
>
> **✅ CONFIRMÉ SUR L'AIR (retour utilisateur, 3ᵉ essai)** : « c'était cela ».
> Symptôme initial : « trame audible juste après l'ouverture de l'écran,
> plus audible ensuite — AGC qui s'écarte ou AFC ? ». Cause :
> `RADIO_SetModulation(MODULATION_FM)` active l'AFC
> (`afcDisableRegSpec = (modulation != MODULATION_FM)`, faux pour FM = actif)
> — avec REG_2B débrayé (dé-emphase/HPF/LPF retirés), le discriminateur en
> sortie brute a un profil DC/bruit très différent de ce pour quoi l'AFC est
> réglée sur de l'audio FM normal filtré ; elle dérivait la fréquence en
> "suivant" ce bruit plutôt qu'un vrai décalage porteuse — exactement "marche
> à l'ouverture, dérive ensuite". **Correctif** : `BK4819_SetRegValue
> (afcDisableRegSpec, true)` ajouté après le profil REG_2B, dans
> `APP_RunSarsat()` — même `RegisterSpec` déjà documenté et utilisé par
> `RADIO_SetModulation()` lui-même, plus étroit que les deux essais
> précédents (ne touche ni le squelch ni le gain global) — c'est ce qui l'a
> distingué des deux tentatives ratées avant. Build vert, +24 o :
> `FLASH 115024/120832 o (95,19 %)`.
>
> **Pourquoi le V1 n'était pas affecté par le même symptôme, alors que son
> code est identique** (question posée après coup) : `App/radio.c` du V1 a
> **exactement** la même ligne `BK4819_SetRegValue(afcDisableRegSpec,
> modulation != MODULATION_FM)`, avec le **même** `RegisterSpec` (`{"AFC
> Disable", 0x73, 4, 1, 1}`) — l'AFC était donc **tout autant active** côté
> V1, jamais désactivée dans son écran SARSAT non plus. La différence n'est
> pas dans le code, elle est dans le silicium : V1 = DP32G030 + **BK4819**,
> UV-K1/K5V3 = PY32F071 + **BK4829** — deux puces différentes qui partagent
> une carte de registres compatible (même bit AFC Disable, d'où le code
> identique) mais pas forcément la même boucle d'AFC analogique en dessous.
> Avec le discriminateur mis à plat, il est plausible que la boucle AFC du
> BK4829 réagisse plus fort/vite à ce profil DC inhabituel que celle du
> BK4819, ou que la référence de fréquence de cette carte dérive un peu plus
> nativement. Hypothèse la plus probable, pas une certitude documentée —
> pas de doc silicium comparative disponible pour trancher définitivement.

## Fait : tracker APRS (2026-09-04)

Port complet de `firmware/uv-k5v1-kd8cec/patch/aprs.c`/`.h`/`ax25.c`/`.h` vers
ce firmware — `patch/aprs.c` (~950 lignes), `patch/aprs.h`, `patch/ax25.c`/`.h`
(copie verbatim, C pur sans dépendance radio, comme prévu au point 1
ci-dessus). Différences avec le V1, toutes documentées en tête de
`patch/aprs.c` :

- Pas de `SendReply()` — ce firmware appelle directement les fonctions UART ;
  `UART_IsCommandAvailable(Port)`/`UART_HandleCommand(Port)` prennent un
  `Port` explicite (`UART_PORT_UART`), comme pour `sarsat.c`.
- **Gain AF C-Board délégué à `app/afgain.h`** (le module autonome déjà écrit
  pour SARSAT) plutôt qu'un champ `af_gain` de `aprs_cfg_t` comme sur le V1 —
  `gAprsCfg.reserved` occupe l'octet laissé vacant pour ne pas changer la
  taille de la struct (toujours 40 o / 5 pages EEPROM de 8 o). `F_AFGAIN` du
  menu config lit/écrit `gAfGain`/`AFGAIN_Apply()` au lieu de
  `gAprsCfg.af_gain`/`APRS_ApplyAfGain()`.
- **`millis10()` est une vraie fonction**, pas un extern KD8CEC-spécifique :
  `App/scheduler.c` tenait déjà un compteur 10 ms privé
  (`gGlobalSysTickCounter`, incrémenté par le vrai `SysTick_Handler()`, donc
  qui avance même pendant les boucles bloquantes des écrans) — exposé par un
  nouveau getter `uint32_t millis10(void) { return gGlobalSysTickCounter; }`
  (`App/scheduler.c` + déclaration dans `App/scheduler.h`), même nom/sémantique
  que le V1 : `aprs.c` n'a besoin d'aucun changement d'appel.
- Adresse EEPROM dédiée : `0x00A178` (juste après les 8 o de `afgain.c` à
  `0x00A170`), 40 o réservés jusqu'à `0x00A1A0`, toujours dans la queue non
  revendiquée du secteur "Settings" — même schéma de `_MK_MAPPING` explicite
  que pour le gain AF (voir plus haut, rappel : une adresse non déclarée dans
  `ADDR_MAPPINGS` ne mène nulle part sur ce firmware).
- **API BK4819/BK4829 confirmée identique** pour toute la séquence TX AFSK :
  `BK4819_EnableTXLink`, `BK4819_DisableDTMF`, `BK4819_EnterTxMute`/
  `ExitTxMute`, `BK4819_SetAF`, `RADIO_SetTxParameters`,
  `BK4819_REG_70_ENABLE_TONE1`/`SHIFT_TONE1_TUNING_GAIN`, `BK4819_AF_MUTE`,
  `BK4819_REG_02_SQUELCH_LOST`/`FOUND`, `BK4819_REG_0C`,
  `BK4819_GPIO6_PIN2_GREEN`/`GPIO5_PIN1_RED`/`GPIO1_PIN29_PA_ENABLE`, et le
  layout de bits de `BK4819_SetupSquelch()` (REG_4E, délais ouverture/
  fermeture aux mêmes positions) — aucune adaptation registre nécessaire, la
  séquence TX/squelch rapide du V1 s'est portée telle quelle.
- `MR_CHANNELS_MAX = 1024` (`App/misc.h`) sur ce firmware — bien plus que les
  170 canaux plafonnés sur le V1 ; `APRS_TX_CHANNEL = 169` (canal 170) n'a
  besoin d'aucun plafonnement particulier ici, la plage native est déjà
  largement suffisante.

Câblage `build.sh` (mêmes principes que SARSAT : ancrages `perl`, réutilisant
le trick "`$&` en tête de remplacement" pour retomber sur les points
d'insertion déjà utilisés par SARSAT/afgain sans nouvel ancrage) :

| fichier | changement |
|------|--------|
| `App/app/app.c` | `#include "app/aprs.h"` ; `APRS_TimeSlice()` + popup auto (`gAprsShowRequest` → `APP_RunAprs()`) dans `APP_TimeSlice10ms()` ; `APRS_KeepAwake()` ajouté à la liste d'inhibition de `gSchedulePowerSave` (RX 144-148 MHz jamais coupé par l'éco batterie) ; `APRS_QuietBacklight()` dans la condition d'allumage du rétroéclairage sur RX (« light on frame ») |
| `App/app/uart.c` | `#include "app/aprs.h"` ; `case APRS_CMD_RXTEXT/RXINFO/GPS:` → `APRS_HandleUART(...)` |
| `App/settings.h` | `ACTION_OPT_APRS` dans l'enum |
| `App/app/action.c` | `#include "app/aprs.h"` ; `[ACTION_OPT_APRS] = &APP_RunAprs` |
| `App/ui/menu.c` | `{"APRS", ACTION_OPT_APRS}` dans `gSubMenu_SIDEFUNCTIONS` |
| `App/driver/eeprom_compat.c` | `_MK_MAPPING(0x00A178, 0x00A178, 0x00A1A0)` |
| `App/scheduler.h`/`.c` | `millis10()` (déclaration + définition) |
| `App/CMakeLists.txt` | `enable_feature(ENABLE_APRS app/aprs.c app/ax25.c)` |
| `CMakePresets.json` | `"ENABLE_APRS": false` par défaut (`-DENABLE_APRS=ON` dans `build.sh`) |

**Ouverture** : comme SARSAT, assignable via le menu (`F1Shrt`/`F1Long`/
`F2Shrt`/`F2Long` → `APRS`), plus l'auto-popup sur trame RX décodée.

**Canaux mémoire pré-remplis** (`APRS_SeedChannel()`, appelée par
`APRS_Init()`, même logique que le V1) — chacun écrit **seulement s'il est
encore vierge** en EEPROM, jamais par-dessus un canal utilisé :

| ch | nom | fréquence | modulation | largeur | puissance |
|---|---|---|---|---|---|
| 170 | `APRS` | 144.800 MHz | FM | large | **haute** |
| 1 | `SAREX` | 434.200 MHz | **RAW** (discri plat) | large | basse |
| 2 | `SARSAT` | 406.028 MHz | **RAW** | large | basse |

(406.028 MHz = bande SARSAT réelle : **réception uniquement**.) Le nom du
canal 170 est re-forcé à `"APRS"` à chaque boot.

**Marge flash dépassée avec le preset `Fusion` complet + SARSAT + APRS**
(constaté à la compilation : dépassement de ~2,3 Ko). `build.sh` désactive
donc `ENABLE_SPECTRUM` (`-DENABLE_SPECTRUM=OFF`) — l'extra F4HWN le plus lourd
et le moins lié à ce projet, même arbitrage que celui déjà fait sur le V1
(SPECTRUM/FMRADIO/VOX/FLASHLIGHT coupés là-bas).

**2026-09-09 — coupes supplémentaires** (le travail d'alignement registres RX
V1/V3 avait fait remonter le flash à 99,0 %). `build.sh` désactive maintenant
aussi, à la demande de l'utilisateur et par étapes :
`ENABLE_FEAT_F4HWN_GAME`, `ENABLE_FEAT_F4HWN_QRCODE`, `ENABLE_FEAT_F4HWN_LOGO`,
`ENABLE_FEAT_F4HWN_LOGO_SAV` (jeu, QR code, logo de boot), puis
`ENABLE_FEAT_F4HWN_K5VIEWER`, `ENABLE_FEAT_F4HWN_RXTX_LOG`,
`ENABLE_FEAT_F4HWN_RXTX_LOG_K5VIEWER` (visualiseur d'écran série, journal
d'activité RX/TX) — aucun rapport avec SARSAT/APRS.
→ `FLASH 109676/120832 o (90,77 %)`, `RAM 14400/16384 o (87,89 %)`,
soit **−~9,9 Ko flash / −~700 o RAM** par rapport au build à 99,0 %. Un build
d'essai « tous les extras non liés coupés » (FMRADIO, AIRCOPY, VOX, FoxHunt,
Beam, AudioScope, MenuCat, PMR/GMRS en plus) descend à
`FLASH 92 092 o (76,2 %)` / `RAM 84,2 %` — réserve disponible si besoin.

**Build vert, 0 warning**, preset `Fusion` (sans le spectre) :

```
Memory region         Used Size  Region Size  %age Used
             RAM:       14976 B        16 KB     91.41%
           FLASH:      114408 B       118 KB     94.68%
```

**⚠️ Non testé sur matériel** — c'est la toute première compilation de
`aprs.c`/`ax25.c` pour ce firmware ; le squelch rapide, l'AFSK TX, le popup
RX, le canal dédié et le GPS restent à valider sur l'air, exactement comme
chaque étape du portage SARSAT l'a été avant lui.

> **Bug corrigé (retour terrain, boîtier UV-K1) : sens des touches UP/DOWN
> inversé dans les vues gain AF (SARSAT) et config (APRS).**
> Remonté : « sur l'UV-K1, l'action des flèches est inversée. À droite ça
> descend, à gauche ça monte » — description en termes physiques, car sur le
> boîtier UV-K1 les deux touches latérales sont imprimées `◄`/`►` (gauche/
> droite) plutôt que `▲`/`▼` comme sur l'UV-K5, bien qu'électriquement ce
> soient toujours les mêmes codes `KEY_UP`/`KEY_DOWN` côté firmware.
>
> F4HWN a déjà un mécanisme prévu pour ça : le réglage **`SetNav`** du menu
> Service (`App/settings.h` : `gEeprom.SET_NAV`, bool) — la sous-chaîne du
> menu documente explicitement les deux boîtiers : `"LEFT\nRIGHT\nUV-K1"` /
> `"UP\nDOWN\nUV-K5(8)"` (`App/ui/menu.c`). Sur un « reset ALL »,
> `App/settings.c` remet `SET_NAV = false` **« for UV-K1 by default »** — et
> partout ailleurs dans ce firmware où UP/DOWN pilote une valeur ou un
> curseur (`App/app/main.c`, `menu.c`, `scanner.c`, `spectrum.c`,
> `aircopy.c`, `fm.c`), le code applique `if (!gEeprom.SET_NAV) Direction =
> -Direction;` avant de s'en servir — c'est la convention firmware-wide pour
> compenser l'étiquetage physiquement inversé du boîtier UV-K1.
>
> Nos écrans (vue gain AF de `sarsat.c`, écran config de `aprs.c`) lisaient
> `KEY_UP`/`KEY_DOWN` directement sans passer par cette convention — d'où
> l'inversion perçue uniquement sur UV-K1 (`SET_NAV` y vaut faux par défaut ;
> sur UV-K5 `SET_NAV` vaut vrai par défaut, donc rien ne changeait pour ce
> boîtier). **Corrigé** : `sarsat.c` calcule `d = (key==KEY_UP)?+1:-1` puis
> `if (!gEeprom.SET_NAV) d = -d;` avant de s'en servir (vue niveau ET défilement
> du texte décodé) ; `aprs.c` ajoute un petit `nav_key()` qui échange
> `KEY_UP`/`KEY_DOWN` dans la même condition, appliqué une fois en tête de la
> boucle de touches de l'écran config — couvre uniformément la sélection de
> champ, l'édition indicatif/texte caractère par caractère, la saisie
> lat/lon et le `field_step()` générique. Build vert, 0 warning,
> `FLASH 114444/120832 o (94,71 %)`.
>
> **Retour terrain suivant : toujours inversé** (« au moins pour les
> chiffres et les lettres », donc les champs numériques via `field_step()`
> et l'édition indicatif/texte via `cyc_char()`). Cause trouvée en
> creusant `App/settings.c` : **`gEeprom.SET_NAV` vaut `true` par défaut sur
> un flash normal**, pas `false`. Le commentaire « for UV-K1 by default »
> concerne uniquement le chemin de reset complet
> (`ENABLE_FEAT_F4HWN_RESCUE_OPS`, reset ALL explicite) ; le chemin de
> migration de version (à chaque flash d'un firmware avec un
> `VERSION_STRING_2` différent) a la ligne équivalente **commentée**
> (`//configByte[4] &= ~0x40;  // SET_NAV = 0`), et sur une EEPROM vierge le
> bit lu est à 1 (flash effacée = `0xFF`). Résultat : `!gEeprom.SET_NAV`
> vaut `false` par défaut → notre code ne fait **aucune** inversion, et le
> mapping brut `KEY_UP=+1` reste actif alors qu'il ne convient pas au
> boîtier UV-K1. Le correctif logiciel (`sarsat.c`/`aprs.c` ci-dessus) est
> correct dans son principe (même convention que tout le reste du firmware)
> mais **inopérant tant que `SetNav` n'est pas positionné manuellement**.
>
> **Pas un bug à corriger en dur** : `SetNav` est un choix utilisateur du
> menu Service, partagé par tout le firmware (menus, scanner, spectrum...),
> et ce même firmware (`Fusion`) sert **aussi** l'UV-K5 V3 (boutons UP/DOWN
> physiques normaux) — figer `SET_NAV=false` au boot casserait la
> navigation pour ces utilisateurs-là. La bonne réponse est de guider
> l'utilisateur UV-K1 vers **MENU → Service → SetNav → « UV-K1 »**
> (au lieu de « UV-K5(8) »), un réglage radio, pas un nouveau flash.
>
> **✅ Confirmé par l'utilisateur** : ce réglage résout l'inversion. Aucun
> changement de code nécessaire — le correctif `sarsat.c`/`aprs.c` était
> déjà correct, il ne faisait qu'attendre ce réglage utilisateur.

## Fait : icône GPS en barre de statut (2026-09-05)

Port de `firmware/uv-k5v1-kd8cec/patch/ui_status.c.diff` vers `App/ui/status.c` :
losange 5×5, même bitmap que le V1, même convention `APRS_GpsState()`
(0 aucun module / 1 recherche = clignote ~1,25 Hz via `millis10() % 80 < 40` /
2 verrouillé = fixe). Inséré juste après le bloc indicateur PTT (`ENABLE_FEAT_F4HWN`),
avant le `x = MAX(x1, 69u);` qui cadre le reste de la barre. `#include "app/aprs.h"`
sous `ENABLE_APRS`, `millis10()` déclarée en `extern` locale (scheduler.h n'est
pas inclus dans ce fichier). Build vert, 0 warning, +88 o :
`FLASH 114532/120832 o (94,79 %)`. **Non testé sur matériel.**

## Fait (étape 1/N) : écran principal « Icom » en mode Main Only (2026-09-05)

Objectif final : amener ce firmware au même niveau que le V1 (affichage, menu,
décodage), **par étapes vérifiables**. La feuille de route complète est
retracée dans l'historique git et les sections datées de ce fichier.

**Étape 1 (validée par l'utilisateur avant codage)** : ce firmware a déjà en
natif un mode très proche de l'Icom — `ENABLE_BIG_FREQ` (actif) affiche la
fréquence en gros chiffres, et le mode **Main Only**
(`gEeprom.DUAL_WATCH==OFF && gEeprom.CROSS_BAND_RX_TX==OFF`, togglable via
`ACTION_OPT_MAINONLY`, assignable comme SARSAT/APRS) affiche un seul VFO
actif. Zéro changement de code nécessaire — confirmé par l'utilisateur comme
bonne base visuelle.

**Étape 2 (ce commit) : bandeau d'en-tête inversé + réorganisation des lignes**,
le point identifié comme délicat avant de commencer (voir plus haut la
discussion sur la découverte que `App/ui/main.c` fait ~2500 lignes contre
~800 sur le V1, avec plein de fonctions imbriquées propres à F4HWN).

**Fichier repris en entier** (pas de patch perl comme pour les autres
fichiers stock) : `patch/ui_main.c`, copié tel quel depuis
`App/ui/main.c` amont puis édité directement — bien plus sûr qu'une série
de correctifs `perl -0pi` sur un fichier aussi gros et dense. `build.sh` le
copie par-dessus le clone (`cp "$HERE/patch/ui_main.c" App/ui/main.c`),
comme `sarsat.c`/`aprs.c`/`afgain.c`/`ax25.c`.

**Portée volontairement étroite de cette étape** (`icomMode`, calculé une
fois en tête de `UI_DisplayMain()`) : uniquement la vue VFO en fréquence
« au repos » — pas un canal mémoire/NOAA, pas de fréquence en cours de
saisie, pas d'appel/saisie DTMF, pas de scan actif, état VFO normal. Toute
autre situation retombe sur le rendu F4HWN d'origine, **strictement
inchangé** — zéro risque de régression sur ces chemins-là (canaux mémoire,
DTMF, scan, action picker, K5Viewer, alarme...).

**Ce que fait `icomMode`** :
- `MainHeaderIcom()` (nouvelle fonction, réutilise le petit fonte 6 px
  `HDR6`/`HdrPut()`/`HdrW()` du port V1 tel quel, C pur) dessine un bandeau
  **ligne 0 exclusivement**, inversé (fond noir texte blanc), coins
  arrondis : VFO/canal (ou « APRS » sur le canal 170), TX/RX, puissance
  (L/M/H), sens du décalage, CTCSS (T/CT), inversion (R), modulation +
  largeur — tout à position fixe (rien ne bouge quand un indicateur
  apparaît/disparaît), même principe que le V1.
- La fréquence (gros chiffres `ENABLE_BIG_FREQ`) descend d'une ligne
  (`line+1`/`line+2` au lieu de `line`/`line+1`) pour laisser la ligne 0 au
  bandeau — **la variable `line` elle-même n'est volontairement pas touchée**
  (elle sert aussi de base à une dizaine de tests `line == 0 ? A : B` sans
  rapport, dans tout le reste de la fonction ; la redéfinir aurait cassé ces
  positions ailleurs). Seuls les appels concernés (fréquence, ligne de
  détail) reçoivent une position explicite recalculée.
- Les icônes qui partageaient jusqu'ici la ligne 0 avec le haut des gros
  chiffres (fanion VFO, texte TX/RX, cadenas, symbole compandeur) sont
  supprimées en `icomMode` — l'info TX/RX/verrouillage est reprise dans le
  bandeau (verrouillage pas encore repris, voir « reste » plus bas).
- La « ligne de détail » (modulation/CTCSS/puissance/décalage/inversion/
  largeur, dessinée via une dizaine d'appels `GUI_DisplaySmallest(...,
  line==0?17:49,...)` déjà existants) est redirigée en bloc vers `y=25`
  (ligne 3) en `icomMode` — remplacement `sed`-like global de
  `line == 0 ? 17 : 49` → `icomMode ? 25 : (line == 0 ? 17 : 49)`, 11
  occurrences. **Limite connue** : seul le style d'affichage
  `gSetting_set_gui == false` (celui utilisant `GUI_DisplaySmallest`) est
  couvert ; l'autre style (`UI_PrintStringSmallNormal(..., LCD_WIDTH+N, ...)`,
  positions hors-écran dont le sens exact n'a pas été élucidé) n'est pas
  touché, donc pas garanti correct en `icomMode` si l'utilisateur a ce
  réglage sur `true`.
- Le S-mètre (`DisplayRSSIBar`, ligne 5 en Main Only) **n'est pas déplacé**
  à cette étape — reste où F4HWN le met déjà nativement (repositionnement
  prévu à l'étape 3 si besoin).
- Ligne 6 : le badge « VFO A/B » existant est remplacé par
  `MainIdleVfoIcom()` (nouvelle fonction, porte le `MainIdleVfo()` du V1) —
  un résumé une ligne du VFO **inactif** (fréquence + modulation, ou
  « M12 nom », ou « APRS ») — redondant avec le bandeau qui nomme déjà le
  VFO actif, donc remplacé plutôt que cumulé.
- **Bug latent trouvé en creusant `DisplayRSSIBar()`** : cette fonction fait
  clignoter le marqueur VFO (bitmap `BITMAP_VFO_Default`/`BITMAP_VFO_Empty`)
  directement sur la ligne `RxLine` (= ligne 0 en Main Only) via un appel
  **indépendant** de `UI_DisplayMain()` — `UI_MAIN_TimeSlice500ms()` rappelle
  `DisplayRSSIBar(true)` toutes les 500 ms tant que le RX est actif. Sans
  correctif, ce clignotement aurait effacé périodiquement le bandeau entre
  deux redessins complets. Corrigé par un nouveau booléen statique de
  portée fichier `s_icomMode` (mis à jour par `UI_DisplayMain()`, lu par
  `DisplayRSSIBar()`), qui désarme ce clignotement quand le bandeau est actif.

**Build vert, 0 warning** : `FLASH 116032/120832 o (96,03 %)`,
`RAM 15008/16384 o (91,60 %)` — marge restante ~4,8 Ko flash / ~1,3 Ko RAM,
suffisante pour cette étape mais à surveiller pour les suivantes (S-mètre,
extension aux canaux mémoire/NOAA/DTMF, menu).

> **Bug corrigé (retour terrain immédiat) : affichage VFO corrompu, fréquence
> et « SQ » visibles mais illisibles, texte qui se chevauche.**
> Cause : la « ligne de détail » (modulation/CT/puissance/décalage/inversion/
> largeur/SQL) existe en fait sous **deux styles parallèles** dans ce
> firmware, choisis par le réglage `gSetting_set_gui` (menu Service, bit 7
> de l'octet EEPROM correspondant — **vaut vrai par défaut sur une EEPROM
> vierge/jamais réglée**, bit à 1 quand la flash est effacée). Mon premier
> passage ne redirigeait que le style `gSetting_set_gui == false`
> (`GUI_DisplaySmallest`, adressage pixel direct) — documenté comme
> limitation connue, mais en pratique **c'est le style par défaut sur la
> plupart des radios qui n'a pas été couvert**. L'autre style
> (`gSetting_set_gui == true`, `UI_PrintStringSmallNormal(...,
> LCD_WIDTH + N, 0, line + 1)`) utilise un adressage délibérément débordant
> (`Start > 128` déborde du tableau 128 o de la ligne `line+1` vers le début
> de la ligne `line+2`) pour positionner ce texte — non redirigé, il
> continuait donc de tomber en `icomMode` sur la ligne 2, exactement là où
> vit désormais le bas des gros chiffres de fréquence : d'où le
> chevauchement illisible.
> **Corrigé** : les 16 appels `UI_PrintStringSmallNormal(..., LCD_WIDTH + N,
> 0, line + 1)` de la ligne de détail redirigés en bloc vers
> `(icomMode ? line + 2 : line + 1)` (même décalage d'une ligne que l'autre
> style), par substitution `perl` ciblée sur le motif exact plutôt qu'à la
> main (16 occurrences, mêmes risque d'oubli qu'à la main). Corrigé aussi au
> passage un débordement du même genre trouvé en relisant : le petit
> pictogramme « puissance utilisateur » (`BITMAP_PowerUser`, actif seulement
> si `OUTPUT_POWER_USER` est sélectionné) écrivait toujours 2 lignes plus
> bas que `line`, sans tenir compte d'`icomMode` — redirigé à 3 lignes plus
> bas dans ce cas, même logique.
> Build vert, 0 warning : `FLASH 116144/120832 o (96,12 %)`.

**Retour terrain suivant : « c'est mieux »** — VFO lisible. Deux points
notés :
- **Mode mémoire n'a pas le nouvel habillage** — attendu à cette étape :
  `icomMode` excluait délibérément `IS_MR_CHANNEL` (portée étroite de cette
  étape). **Étendu au canal mémoire le 2026-09-09** — voir la section
  « Habillage « Icom » étendu au canal mémoire » plus bas.
- **Bug corrigé : le badge de bande de fréquence (« F6 », coin haut-gauche)
  débordait de ~1-2 px dans le bandeau d'en-tête.** Cause :
  `UI_PrintStringSmallNormalInverse()` (utilisée pour ce badge) inverse non
  seulement sa propre ligne mais aussi **1 px de la ligne du dessus**
  (`gFrameBuffer[Line - 1][x] ^= 0x80;`, un filet de séparation visuelle
  entre deux VFO empilés en mode dual-VFO normal) — badge resté à
  `line + 1` (row 1) au lieu d'être redirigé comme le reste : `Line - 1`
  valait donc **row 0, le bandeau**, d'où le petit défaut visuel décrit.
  Corrigé : même redirection que le reste, `icomMode ? line + 2 : line + 1`
  — `Line - 1` devient alors row 1 (haut des gros chiffres à cet endroit
  précis, x=2, hors de la zone occupée par les chiffres eux-mêmes qui
  commencent à x=32 — comportement identique à avant le passage en icomMode,
  juste une ligne plus bas). Build vert : `FLASH 116156/120832 o (96,13 %)`.

> **Bug corrigé (retour terrain) : « quelques pixels entre les chiffres des
> fréquences » sur l'affichage du second VFO** (la ligne résumé
> `MainIdleVfoIcom()`, ligne 6). Cause : la lecture du pas de fréquence
> (« step size », `gSetting_set_gui == true`, donc actif par défaut — même
> réglage que le bug de la ligne de détail plus haut) écrit elle aussi sur
> la ligne 6, à `x=2` et `x=46` — exactement là où `MainIdleVfoIcom()`
> dessine la fréquence de l'autre VFO. Les deux se chevauchaient chiffre par
> chiffre, donnant l'impression de « pixels en trop » entre eux. **Corrigé** :
> ce bloc (déjà documenté comme non couvert par cette étape) est maintenant
> explicitement désactivé en `icomMode` (`&& !icomMode` ajouté à sa
> condition) — pas de perte réelle, le V1 n'affiche de toute façon jamais le
> pas de fréquence sur son écran principal. Build vert :
> `FLASH 116160/120832 o (96,13 %)`.

> **Bug corrigé (retour terrain) : la puissance affichée dans le bandeau et
> celle affichée sous la fréquence (ligne de détail) ne correspondaient
> pas.** Cause : `MainHeaderIcom()` réduisait `v->OUTPUT_POWER` à un simple
> `L/M/H` en supposant une échelle à 3 niveaux (0/1/2) comme sur le V1 — or
> ce firmware a une échelle à **8 niveaux**
> (`OUTPUT_POWER_USER=0, LOW1..LOW5=1..5, MID=6, HIGH=7`, `App/settings.h`),
> la même que celle déjà utilisée par la ligne de détail (`pwr_short[]` :
> "L1".."L5"/"M"/"H", avec `OUTPUT_POWER_USER` résolu via
> `gSetting_set_pwr`). Le bandeau indexait `pw[p<3?p:0]` sur la valeur
> brute 0-7 : par exemple `OUTPUT_POWER_MID` (6) tombait dans le cas
> `>= 3` et affichait « L » au lieu de « M ». **Corrigé** : le bandeau
> reprend maintenant exactement le même calcul que la ligne de détail
> (même tableau `pwr_short[]`, même résolution de `OUTPUT_POWER_USER`) —
> les deux affichages montrent désormais la même valeur. Le code puissance
> pouvant faire 2 caractères (« L1 ».."L5") au lieu d'1 seul, les champs
> suivants du bandeau (décalage, CTCSS, inversion, modulation) ont été
> décalés vers la droite pour ne pas se chevaucher (70→74, 78→86, 93→102,
> plancher modulation 103→112). Build vert : `FLASH 116204/120832 o (96,17 %)`.

> **Retour terrain suivant, deux demandes en une :**
> 1. **« Le signe RAW déborde de la barre inversée »** — corrigé. Le
>    « plancher » fixe (112) que je venais d'ajouter pour laisser la place
>    au code puissance sur 2 caractères était appliqué **tout le temps**,
>    même quand décalage/CTCSS/inversion n'étaient pas affichés — il
>    poussait donc la modulation plus loin que nécessaire dans le cas
>    courant (juste `RAW`, sans rien d'autre) : `HdrW("RAW")=23`,
>    `hx = 128-3-23 = 102`, mais le plancher fixe le remontait à 112, et
>    112+23=135 dépasse les 128 px de l'écran → texte tronqué à droite.
>    **Corrigé** : le plancher est maintenant calculé dynamiquement —
>    chaque champ optionnel (décalage, CTCSS, inversion) fait avancer un
>    curseur `floor_x` **seulement s'il est réellement affiché**, et la
>    modulation ne recule que d'autant que nécessaire. `RAW` seul revient
>    à la position naturelle (~102), largement dans l'écran ; les cas avec
>    plusieurs indicateurs actifs restent protégés du chevauchement.
> 2. **« Supprimer les infos sous le VFO principal, elles sont déjà dans le
>    bandeau »** — fait. Tout le bloc de la « ligne de détail »
>    (modulation/CT/puissance/décalage/inversion/largeur/SQL, ~290 lignes,
>    les deux styles `gSetting_set_gui`) est maintenant entouré d'un
>    `if (!icomMode) { ... }` : plus dessiné du tout quand le bandeau Icom
>    est actif, réutilisant simplement `MainHeaderIcom()` comme seule
>    source de cette information. Bonus : ça libère plus de flash que le
>    nouveau code du bandeau n'en consomme.
> 3. **Largeur (rien = wide, N = narrow)** : déjà correcte dans le bandeau
>    (`v->CHANNEL_BANDWIDTH == BANDWIDTH_NARROW ? " N" : ""`, accolée à la
>    modulation) — inchangée, confirmée présente maintenant que c'est la
>    seule source de cette info.
>
> Build vert, 0 warning, **flash en baisse** grâce à la suppression de la
> ligne de détail : `FLASH 116168/120832 o (96,14 %)` (contre 96,17 % juste
> avant).

> **Retour terrain suivant, deux ajustements fins :**
> 1. **Glyphe « 1 » de la police d'en-tête `HDR6`** : le pixel en haut à
>    droite du fanion manquait (`0x3E` au lieu de `0x3F` sur la 3ᵉ colonne —
>    la police est un bitmap colonne par colonne, bit0 = haut). Corrigé ici
>    **et dans `firmware/uv-k5v1-kd8cec/patch/ui_main.c.diff`** (même table,
>    copiée verbatim — même défaut latent là-bas, jamais remonté). V1 non
>    reconstruit/reflashé pour ce détail cosmétique, pas demandé.
> 2. **S-mètre recentré dans l'espace libre entre les deux VFO** (comme le
>    fait le port V1/moto). La suppression de l'ancienne ligne de détail a
>    libéré les lignes 3/4/5 entre la fréquence (lignes 1-2) et le résumé du
>    VFO inactif (ligne 6) — un blanc de 3 lignes (24 px) au lieu d'1 seule.
>    `DisplayRSSIBar()` (qui dessine le S-mètre sur **1** ligne de 8 px,
>    texte dBm + barre, confirmé en relisant la fonction en entier) restait
>    collé à la ligne 5 (juste au-dessus du résumé), comme en mode Main Only
>    classique. Corrigé : nouvelle branche `if (s_icomMode) line = 4;` — la
>    ligne du milieu des 3 disponibles, ce qui centre le S-mètre avec 8 px
>    d'air au-dessus et en dessous (le partage en 3 lignes de 8 px tombe
>    juste, pas besoin du décalage plus fin à cheval sur deux lignes qu'a dû
>    faire le port V1/moto sur un espace moins généreux). Le mode Main Only
>    *hors* icomMode (avec l'ancienne ligne de détail encore visible) garde
>    sa position ligne 5 d'origine, inchangée.
>
> Build vert, 0 warning : `FLASH 116176/120832 o (96,15 %)`.

## Fait (étape 5/N) : même habillage en Dual Watch (2026-09-05)

Demandé : « peux-tu modifier également le dual watch afin d'avoir le même
visuel ». Jusqu'ici `icomMode` exigeait `isMainOnly()` (Dual Watch désactivé)
— avec Dual Watch actif, l'écran retombait sur le double affichage F4HWN
d'origine (un VFO en haut, l'autre en bas), visuellement incohérent avec le
reste.

**Constat en reprenant le port V1** : son écran `moto`/`id91` n'a en fait
**jamais** de vrai « split écran » — `MainHeaderIcom()`/`MainIdleVfo()`
s'appliquent systématiquement, Dual Watch actif ou non ; le V1 affiche
toujours un VFO actif en grand + l'autre en résumé une ligne, quel que soit
le réglage Dual Watch. « Le même visuel » demandé est donc littéralement
**le comportement déjà en place ici pour Main Only, débarrassé de sa
condition `isMainOnly()`** — pas un nouveau layout à inventer.

**Fait** : `isMainOnly()` retiré des préconditions de `s_icomMode` (ne reste
que les préconditions de contenu : pas canal mémoire/NOAA, pas de saisie,
pas de DTMF, pas de scan, état VFO normal). Trois points de la boucle par-VFO
qui décidaient « une seule ligne de contenu, toujours ligne 0 » uniquement
sur `isMainOnly()` prennent maintenant `isMainOnly() || icomMode` :
- le choix de la ligne de base (`line = 0` au lieu de `0`/`4` selon le VFO) ;
- le `continue` qui saute le second VFO (celui qui n'est pas l'actif) ;
- l'aiguillage marqueur VFO / bandeau (même branche que Main Only).

Le dispatcher ligne 6 (`if (icomMode) MainIdleVfoIcom(...) else if
(isMainOnly() ...)`) n'a pas eu besoin de changer : il testait déjà
`icomMode` **avant** `isMainOnly()`, donc gérait déjà correctement ce
nouveau cas. `DisplayRSSIBar()` de même (`if (s_icomMode) line = 4;`,
indépendant d'`isMainOnly()` depuis l'étape précédente).

Conséquence attendue et voulue : en Dual Watch, l'écran bascule maintenant
entre « VFO A en grand, B en résumé » et « VFO B en grand, A en résumé » au
gré du balayage — exactement le comportement du V1, qui n'a jamais eu
d'affichage à deux VFO simultanés en pleine taille.

Build vert, 0 warning : `FLASH 116208/120832 o (96,17 %)`. **Non testé sur
matériel.**

## Fait (étape 6/N) : F+5 ouvre directement l'écran APRS (2026-09-05)

Demandé : « toujours pas possible de lancer à partir de F+8 SARSAT ou F+5
APRS ? » — jusqu'ici seule la voie indirecte (menu → F1Shrt/F1Long/F2Shrt/
F2Long → choisir SARSAT/APRS) existait sur ce firmware.

**F+5 → APRS : fait.** Dans `App/app/main.c` (`processFKeyFunction()`,
`case KEY_5:`), un appui **court** sur F+5 ne fait **rien** dans ce build
précis : la branche existante est `#ifdef ENABLE_NOAA ... #elif defined
(ENABLE_SPECTRUM) APP_RunSpectrum(); ... #endif`, et aucun des deux n'est
actif ici (`ENABLE_NOAA` jamais activé sur ce port, `ENABLE_SPECTRUM` coupé
pour faire de la place à SARSAT+APRS, voir plus haut) — F+5 court était donc
une combinaison **morte**, libre à récupérer sans rien casser. Nouvelle
branche `#elif defined(ENABLE_APRS) APP_RunAprs(); #endif` ajoutée à la
suite, avec la même priorité que l'existant (si un autre preset réactivait
NOAA ou Spectrum, ils garderaient la main sur F+5, pas de régression pour ce
cas-là). L'appui **long** sur F+5 (`toggle_chan_scanlist()`) est inchangé.

**F+8 → SARSAT : pas possible sans sacrifier une fonction existante.**
Contrairement à F+5, `case KEY_8:` fait déjà quelque chose de bien vivant
dans ce preset : appui court = `ACTION_BackLightOnDemand()` (rétroéclairage
à la demande), appui long = bascule `FrequencyReverse` (fonction radio
standard, utile par ex. pour écouter la fréquence de sortie d'un
répéteur). Aucun des dix F+chiffre (F+0 à F+9) n'est mort dans ce preset —
tous sont déjà pris par une fonction F4HWN active (FM, changement de bande,
permutation VFO, scanner, jeu Breakout/VOX, marche/arrêt, etc.) ; F+5 était
le seul creneau libre, par coïncidence de la combinaison `ENABLE_SPECTRUM`
déjà coupée. SARSAT reste donc accessible via l'assignation de touche
(menu → F1Shrt/F1Long/F2Shrt/F2Long → SARSAT) tant qu'un autre F+chiffre
n'est pas explicitement sacrifié à sa place — décision laissée à
l'utilisateur, pas prise unilatéralement ici.

Build vert, 0 warning, +4 o : `FLASH 116212/120832 o (96,18 %)`. **Non
testé sur matériel.**

**Suite (confirmée par l'utilisateur, comparaison avec le V1 d'abord) :
« oui, F+8 sur UV-K1 ».** Vérifié que F+7 n'était pas non plus libre sur le
V1 (`ENABLE_VOX=0` là-bas aussi → `case KEY_7` retombe sur
`toggle_chan_scanlist()`), et que le V1 avait lui-même sacrifié F+8
(uniquement l'inversion de fréquence) pour SARSAT — retenu comme référence
de parité. **Fait** : `case KEY_8` de `App/app/main.c` remplacé en entier
par `#ifdef ENABLE_SARSAT APP_RunSarsat(); #else <code d'origine> #endif`,
même schéma que le patch V1. Contrairement au V1, ce firmware faisait
**deux** choses sur F+8 (court : `ACTION_BackLightOnDemand()` ; long :
bascule `FrequencyReverse`) — les deux sacrifiées, sur confirmation
explicite, pour que la touche corresponde exactement à celle du V1. Build
vert, 0 warning, flash en légère baisse (code retiré) :
`FLASH 116188/120832 o (96,16 %)`. **Non testé sur matériel.**

## Fait (étape 7/N) : « Light on frame » ne s'allumait pas sur la trame (2026-09-05)

Remonté : « lors du décodage d'une trame APRS, le rétroéclairage ne s'allume
pas. Il s'allume et s'éteint quelques secondes après (peut-être au bout du
timeout popup) ». Confirmé par l'utilisateur : l'option **« Light on frame »**
(pas « Light on RX ») est bien celle activée dans le menu APRS.

**Analyse** : `aprs_rx_arrived()` (appelée à chaque trame décodée) allume le
rétroéclairage en un seul appel `BACKLIGHT_TurnOn()`. En théorie ça devrait
suffire — la boucle bloquante du popup gèle le compte à rebours normal du
rétroéclairage, donc l'allumage devrait tenir tout le popup. Log RP2040 fourni
(`[aprs] ... pkts=8`, une trame `F4DVK` décodée avec position) confirmant
qu'une trame est bien arrivée à ce moment-là, mais sans horodatage assez fin
pour prouver où exactement l'allumage se perdait. Retour terrain net après
correctif partiel : **le rétroéclairage ne s'allume qu'à la fermeture du
popup** — exactement le `BACKLIGHT_TurnOn()` inconditionnel déjà présent dans
le code de sortie de `APP_RunAprs()` (indépendant de « Light on frame »),
suivi ~1 s après par l'extinction programmée (`s_bl_off_10ms`). L'appel
ponctuel de `aprs_rx_arrived()`, lui, ne « tenait » manifestement pas —
cause exacte non identifiée avec certitude à la seule lecture du code
(aucune écriture concurrente trouvée qui l'annulerait), mais le symptôme
« un réglage one-shot qui ne tient pas » est un schéma déjà rencontré et
résolu deux fois dans ce projet (le gain AF figé, `AFGAIN_TimeSlice()`).

**Corrigé par le même principe, en réassertion plutôt qu'en un seul appel** :
- Nouvelle variable `s_bl_on_until_10ms` : arme une fenêtre de ~1,5 s à
  chaque trame décodée (`aprs_rx_arrived()`), en plus de l'appel immédiat
  existant.
- `APRS_TimeSlice()` (tick principal, ~10 ms, actif hors popup) réaffirme
  `BACKLIGHT_TurnOn()` tant que cette fenêtre n'est pas expirée — couvre la
  réception en fond sans popup (popup désactivé ou en cooldown).
- `APP_RunAprs()` (boucle du popup lui-même) réaffirme aussi
  `BACKLIGHT_TurnOn()` à **chaque itération** (~20 ms) tant que « Light on
  frame » est actif — couvre le cas où le popup est déjà ouvert.
Ainsi, quelle que soit la cause exacte de la perte du premier appel, le
rétroéclairage est maintenant réaffirmé assez souvent pour ne plus pouvoir
rester éteint entre la trame et la fermeture du popup.

**Même correctif porté sur le V1** (`firmware/uv-k5v1-kd8cec/patch/aprs.c`) :
code de `aprs.c` identique sur les deux firmwares, donc même risque latent
même si non remonté là-bas. Recompilé en natif pour vérifier : `text 59096 o
(+16 o)`, 0 warning (`-Wextra`).

Build K1/K5V3 vert, 0 warning : `FLASH 116240/120832 o (96,20 %)`. **Non
testé sur matériel — hypothèse de correction, à confirmer sur l'air.**

**Retour terrain : le correctif ci-dessus n'a rien changé** (« le problème
est identique »). Le même retour signalait aussi beaucoup moins de trames
décodées en FM qu'avec le V1, et des trames lisibles (`src=F4DVK`) rejetées
FCS-bad sur les 3 chaînes de décodage. Piste d'abord explorée (différence FM
BK4819/BK4829) puis **écartée par l'utilisateur lui-même** : « l'UV-K1 est
plus sensible aux perturbations HF » et « j'ai du bruit ambiant, quand je
m'écarte ça fonctionne » — donc un problème d'environnement RF au banc de
test pour le taux de décodage, indépendant du bug rétroéclairage.

**Cause racine trouvée** (indice décisif de l'utilisateur : « même en
appuyant sur une touche pendant le popup, aucun éclairage »). Si même un
appui clavier — qui appelle pourtant `BACKLIGHT_TurnOn()` sans condition à
la ligne concernée — n'allume rien, le bug n'est pas dans la logique
« Light on frame » mais dans `BACKLIGHT_TurnOn()` lui-même *dans ce contexte
précis*. Ce firmware (`ENABLE_FEAT_F4HWN`, `App/driver/backlight.c`) a un
pilote de rétroéclairage **très différent** de celui du V1 : `BACKLIGHT_
TurnOn()` ne fait qu'armer un fondu (cible + pas), et c'est
`BACKLIGHT_Update()` — appelée toutes les ~10 ms par le tick principal
(`App/app/app.c:1604`) — qui fait réellement progresser le PWM matériel vers
cette cible. Or `APP_RunAprs()` est une **boucle bloquante** (comme
`APP_RunSpectrum()`) : le tick principal ne tourne plus pendant qu'elle
s'exécute, donc `BACKLIGHT_Update()` n'est jamais appelée, et le
rétroéclairage reste figé à sa valeur d'avant l'ouverture — quel que soit le
nombre d'appels à `BACKLIGHT_TurnOn()` (trame décodée, appui touche, entrée
d'écran). Dès la sortie du popup, le tick principal reprend et rattrape
instantanément le fondu déjà armé — d'où le symptôme exact observé
(« s'allume dès que le popup s'arrête »). **Ce n'est pas une hypothèse** :
`App/app/foxhunt.c` et `App/app/spectrum.c` — deux autres écrans bloquants
de ce même firmware — ont déjà dû résoudre exactement ce problème, avec un
commentaire qui l'explique noir sur blanc (`FOXHUNT_TickDelay()` : « BACKLIGHT_
Update runs every 10 ms in APP_TimeSlice10ms; delaying the whole 50 ms tick
at once made it ~5x slower »). Le V1 (KD8CEC/DP32G030) n'a jamais eu ce bug :
son `BACKLIGHT_SetBrightness()` écrit le registre PWM de façon synchrone,
sans fondu ni tick — d'où sa fiabilité alors que le code `aprs.c` est
identique aux deux firmwares.

**Corrigé** : nouveau `APRS_TickDelay(ms)` (`patch/aprs.c`) et
`SARSAT_TickDelay(ms)` (`patch/sarsat.c`, même bug latent dans la boucle de
l'écran SARSAT, corrigé par précaution même si non spécifiquement remonté
— pas de fonctionnalité « light on decode » là-bas mais même risque qu'une
touche n'allume rien tant que l'écran est déjà ouvert) — même schéma que
`FOXHUNT_TickDelay()` : boucle de `SYSTEM_DelayMs(10)` + `BACKLIGHT_Update()`
par tranche de 10 ms, remplaçant les `SYSTEM_DelayMs(20)` des boucles
respectives (3 sites dans `aprs.c`, 1 dans `sarsat.c`). Build vert, 0
warning : `FLASH 116280/120832 o (96,23 %)`, `RAM 15008/16384 o (91,60 %)`.
`.bin` + `sha256.txt` régénérés. La cause est démontrée par le code lui-même
(même bug déjà corrigé ailleurs dans ce firmware), pas une hypothèse par
élimination comme les tentatives précédentes (réassertion `BACKLIGHT_
TurnOn()`, sans effet, laissée en place car inoffensive).

**Confirmé sur l'air : « fonctionne »** — nouveau retour, cette fois cosmétique :
« la fréquence de rafraîchissement de l'affichage [semble] faible, lors de
changement sur l'écran il y a un petit effet de fondu ». Cause : le premier
correctif espaçait les 16 appels `BACKLIGHT_Update()` nécessaires pour finir
un fondu sur ~160 ms (un appel toutes les 10 ms, calé sur la cadence du tick
principal) — ce qui rendait la **montée en luminosité** normalement discrète
(habituellement masquée ailleurs dans l'interface par le changement d'écran
qui l'accompagne) visible et perceptible comme un ralentissement, puisque
l'utilisateur regarde cet écran précis pendant tout le fondu. Corrigé :
`APRS_TickDelay()`/`SARSAT_TickDelay()` purgent maintenant les 16 appels
`BACKLIGHT_Update()` d'un coup, sans délai entre eux, avant la pause de 20 ms
— un fondu se termine toujours en ≤ 16 pas quelle que soit son amplitude
(`fadeStep` = écart/16, voir `BACKLIGHT_SetBrightness()`), donc l'enchaîner
d'un coup le rend instantané comme il l'était censé paraître, sans toucher
au comportement du fondu ailleurs dans l'interface (fonctions partagées
`BACKLIGHT_TurnOn()`/`BACKLIGHT_Update()` non modifiées, seul l'endroit d'où
elles sont pompées dans ces deux écrans bloquants change). Build vert, 0
warning : `FLASH 116276/120832 o (96,23 %)`. 
**Retour terrain : « toujours le même effet de fondu »**, avec la précision
« je ne sais pas depuis quand, je n'avais pas fait attention » — donc la
purge immédiate n'a rien changé au symptôme. Question posée pour trancher
sans deviner à l'aveugle : le même effet apparaît-il aussi ailleurs dans
l'interface (menus normaux, liste de canaux), hors SARSAT/APRS ? **Réponse :
oui, même effet ailleurs.** Conclusion : ce fondu est un comportement du
firmware F4HWN d'origine (probablement le même mécanisme de fondu de
rétroéclairage qu'on a dû réactiver ici, ou une caractéristique de l'écran
LCD), présent dans toute l'interface stock, sans rapport avec le code
SARSAT/APRS de ce projet — **hors périmètre, aucune action prévue ici**. Le
vrai bug visé par ce correctif (« aucun éclairage du tout pendant le popup,
même sur appui touche ») reste, lui, confirmé corrigé.

**Suite, sur demande explicite : « possibilité d'atténuer ce phénomène ? »**
Le fondu étant confirmé présent partout (hors périmètre SARSAT/APRS), la
correction touche ici le réglage global du firmware F4HWN, pas nos écrans.
`App/driver/backlight.c` : `fadeStep` divise l'écart de luminosité par 16
(`>> 4`), donc ~16 pas quel que soit le saut — à la cadence normale du tick
10 ms, un fondu complet prend ~160 ms. Raccourci à `>> 2` (4 pas, ~40 ms) :
diviseur réduit plutôt que fondu supprimé, pour garder une transition douce
sans le rendre perceptible comme un ralentissement. Patché via un nouvel
ancrage perl dans `build.sh` (pas de `#ifdef ENABLE_SARSAT`/`ENABLE_APRS` :
réglage global, indépendant de nos fonctionnalités), avec sa propre
vérification `grep -q`. Build vert, 0 warning, flash/RAM inchangés :
`FLASH 116276/120832 o (96,23 %)`. **Non testé sur l'air.**

**Retour terrain : « encore visible autant qu'avant »** — aucune amélioration
malgré le fondu 4x plus court. Test décisif proposé et confirmé : rétroéclairage
forcé sur **« toujours allumé »** (`BACKLIGHT_TIME`=61, `BACKLIGHT_TurnOn()` ne
peut alors plus rien changer) — **l'effet persiste identique**. Conclusion :
ce n'est pas le fondu de rétroéclairage du tout, mais probablement une
caractéristique physique de la dalle LCD elle-même (temps de transition des
pixels/segments, propre au panneau ST7565 STN/FSTN) — hors de portée d'un
correctif logiciel simple. Le raccourcissement du fondu (`>> 4` -> `>> 2`)
reste en place (inoffensif) mais n'a pas résolu ce symptôme précis.

**Question directe de l'utilisateur : « y a-t-il une vitesse de
rafraîchissement de l'affichage ? »** Oui, distincte du rétroéclairage et de
la dalle : la vitesse de la liaison SPI qui pousse le framebuffer vers le
contrôleur ST7565. `App/driver/st7565.c` : `LL_SPI_BAUDRATEPRESCALER_DIV64`,
soit ~750 kHz à 48 MHz cœur -- un rafraîchissement plein écran (7 lignes x
128 o) prend alors de l'ordre de 10-15 ms, assez pour un effet de balayage
visible ligne par ligne sur un changement d'écran (en plus, pas à la place,
de la caractéristique de dalle ci-dessus).

**Comparaison demandée avec le V1** : `uvk5cec-0.3q/driver/spi.c`
(`SPI_CR_SPR=2` = FPCLK/16, même horloge cœur 48 MHz) -> **~3 MHz**, sur le
même contrôleur ST7565 et la même dalle 128x64 (`LCD_WIDTH`/`FRAME_LINES`
identiques). Écart de **x4** entre les deux firmwares sur un matériel de la
même famille -- le V1 cumule en plus l'absence totale de fondu logiciel de
rétroéclairage (PWM écrit de façon synchrone, pas de fondu). Le rapprocher
n'est donc pas une valeur devinée : c'est aligner le K1/K5V3 sur une vitesse
déjà éprouvée ailleurs dans ce projet, sur le même écran.

**Fait, sur confirmation explicite** : `App/driver/st7565.c`,
`LL_SPI_BAUDRATEPRESCALER_DIV64` -> `LL_SPI_BAUDRATEPRESCALER_DIV16` (nouvel
ancrage perl dans `build.sh`, avec sa vérification `grep -q`) -- fait passer
un rafraîchissement plein écran d'environ 10-15 ms à ~2,5-4 ms. Build vert,
0 warning, flash/RAM inchangés : `FLASH 116276/120832 o (96,23 %)`. **Non
testé sur l'air** -- petit risque théorique de glitches d'affichage si le
câblage de la dalle a peu de marge électrique à cette vitesse (même si le
même rapport tourne sans souci sur le V1, matériel différent).

**Retour terrain : « l'effet est toujours présent »** — aucune amélioration
malgré la vitesse SPI x4. Avec ce résultat et le test "toujours allumé"
précédent (backlight écarté), **les deux hypothèses testées côté firmware
sont maintenant écartées** : ni le fondu logiciel de rétroéclairage, ni la
vitesse de transfert du framebuffer vers l'écran. Conclusion honnête : c'est
très vraisemblablement une caractéristique physique intrinsèque de la dalle
LCD elle-même (temps de réponse du cristal liquide du panneau ST7565
STN/FSTN), pas quelque chose qu'un réglage firmware peut corriger. Pas de
piste supplémentaire identifiée à ce stade -- **investigation close** plutôt
que de continuer à deviner des réglages sans preuve. Les deux tweaks
(fondu raccourci, SPI x4) restent en place par défaut (inoffensifs, légers
gains même sans résoudre ce symptôme précis) sauf demande contraire.

### Saisie manuelle de fréquence en mode icom (2026-09-05)

Remonté : « lorsque je rentre une fréquence manuellement, c'est l'ancien
affichage le temps de rentrer cette fréquence, après l'affichage revient en
mode icom ». Cause directe, documentée dès la première version d'`icomMode`
comme limitation volontaire de la 1ʳᵉ passe (`gInputBoxIndex == 0` dans la
porte de `s_icomMode`) : taper une fréquence désactivait entièrement
`icomMode` pour la durée de la saisie, retombant sur le rendu F4HWN
d'origine, avant de rebasculer en icom dès la validation.

Corrigé : la condition `gInputBoxIndex == 0` est retirée de la porte --
`IS_FREQ_CHANNEL(gEeprom.ScreenChannel[activeTxVFO])` suffit à garantir que
ceci ne s'applique jamais qu'à la saisie d'une *fréquence* (jamais un numéro
de canal mémoire, chemin distinct gated sur `IS_MR_CHANNEL`). La branche
« user entering a frequency » de la boucle par-VFO reçoit le même décalage
de ligne conditionnel à `icomMode` (`freqRow = icomMode ? line + 1 : line`)
que la branche d'affichage normale juste en dessous, qui l'avait déjà. Le
bandeau d'en-tête (VFO/puissance/modulation...) continue de s'afficher
normalement pendant la saisie ; seuls les chiffres de fréquence reflètent le
tampon de saisie en cours au lieu de la fréquence validée. Build vert, 0
warning : `FLASH 116280/120832 o (96,23 %)`. **Non testé sur l'air.**

### Habillage « Icom » étendu au canal mémoire (2026-09-09)

Remonté : « l'affichage n'est plus style icom si un canal mémoire est mis en
principal ». C'était la limitation volontaire de la 1ʳᵉ passe (voir plus haut :
« Mode mémoire n'a pas le nouvel habillage — attendu »). Levée maintenant, sur
le même modèle que l'écran principal du V1 (KD8CEC), qui habille déjà ses
canaux mémoire.

- Porte de `s_icomMode` : `IS_FREQ_CHANNEL(...)` devient
  `IS_FREQ_CHANNEL(...) || (IS_MR_CHANNEL(...) && gInputBoxIndex == 0)` — un
  canal mémoire actif passe en mode icom sauf pendant la saisie d'un numéro de
  canal (chemin distinct) ; les canaux NOAA restent sur le rendu F4HWN
  d'origine.
- `MainHeaderIcom()` : slot de gauche = `MR <n>` pour un canal mémoire (juste
  le numéro, comme le bandeau du V1), `APRS` pour le canal 170, `VFO A/B`
  sinon.
- Boucle par-VFO : les deux tests `IS_MR_CHANNEL(...)` du rendu canal (le
  badge numéro inversé ligne 1, et le gros bloc « scan-list + compander +
  switch `MDF_*` ») prennent `&& !icomMode`. En `icomMode` le canal est rendu
  exactement comme un VFO fréquence — gros chiffres de la fréquence du canal
  sur `line + 1` — quel que soit `CHANNEL_DISPLAY_MODE`. Le **nom** du canal
  (s'il y en a un) est écrit en petit sur la ligne 3, dans l'espace libre
  sous la fréquence.
- Le VFO inactif (`MainIdleVfoIcom()`, ligne 6) gérait déjà le cas canal
  (`A  M12 nom`) — inchangé.

Build vert, 0 warning : `FLASH 119456/120832 o (98,86 %)`, `RAM inchangée`
(+176 o flash). `.bin` + `sha256.txt` régénérés. **Non testé sur l'air.**

### (2026-09-09) Badge « F3 » retiré + fréquence centrée en mode icom

Demandé : « retirer le F3 à gauche de la fréquence principale et centrer
l'affichage (comme le v1) ».
- La branche `else if (IS_FREQ_CHANNEL(...))` (badge de bande inversé, coin
  haut-gauche) prend `&& !icomMode` → plus de « F3 » / « F6 » en mode icom.
- La fréquence en gros chiffres est maintenant **centrée** (bloc = gros
  chiffres + 2 petits digits de fin, `fx = (LCD_WIDTH - bw - 14) / 2`) comme
  l'écran icom du V1 — dans les 3 chemins (état stable, saisie de fréquence,
  fréquence > 1 GHz via `UI_PrintString` centré). `sprintf` maison
  (`"%u.%05u"`, sans le pad `%3u` de `UI_FormatFrequency`) pour qu'une
  fréquence < 100 MHz reste centrée aussi.
- **Scan** : inchangé — la porte de `s_icomMode` exige déjà
  `gScanStateDir == SCAN_OFF`, donc pendant un balayage l'écran repasse
  automatiquement au rendu F4HWN standard (bandeau non dessiné, S-mètre à sa
  position stock).

Build vert, 0 warning : `FLASH 109752/120832 o (90,83 %)`. `.bin` +
`sha256.txt` régénérés. **Non testé sur l'air.**

### Parité canaux avec le V1 (2026-09-05)

Demandé après un état des lieux comparatif complet entre les deux firmwares
(fait à la demande : « peux-tu me dire si nous avons porté toutes les
fonctionnalités ? »), deux écarts identifiés côté canaux :

**Label "APRS" en mode d'affichage `MDF_CHANNEL`** — le K1/K5V3 affichait
"CH-0170" au lieu de "APRS" quand ce style d'affichage précis (numéro de
canal brut) est sélectionné pour le VFO posé sur le canal APRS dédié (les
deux autres styles, nom et nom+fréquence, l'affichaient déjà correctement
via le nom EEPROM forcé). Même correctif que le V1 (`ui_main.c.diff`) :
`if (gEeprom.ScreenChannel[vfo_num] == APRS_TX_CHANNEL) strcpy(String,
"APRS"); else` devant le `sprintf(String, "CH-%04u", ...)`.

**Plafond mémoire à 170 canaux** — demandé pour reprendre la convention du
V1 (elle-même héritée du projet C-Board d'origine). Investigation avant
d'appliquer bêtement le même patch : sur ce firmware, `MR_CHANNELS_MAX`
(1024 par défaut) n'est pas qu'une borne d'énumération comme sur le V1 --
`FREQ_CHANNEL_FIRST/LAST` et `NOAA_CHANNEL_FIRST/LAST` en sont déjà dérivés
(un seul point de changement suffit, plus simple que le V1 qui devait
décaler 4 bornes littérales à la main), mais **deux autres fonctionnalités
supposent une divisibilité précise** :
- `SETTINGS_ResetTxLock()` (menu "reset verrouillage TX") découpait la
  mémoire canaux en 32 lots égaux d'octets -- correct seulement si
  `MR_CHANNELS_MAX` est un multiple de 32 (1024 l'est, 170 ne l'est pas :
  170/32 laisse un reste, ce qui aurait corrompu/sauté silencieusement des
  canaux près des limites de lot, un vrai bug mémoire, pas juste cosmétique).
- **Aircopy** (transfert de canaux radio-à-radio par audio) découpe en
  banques de 128 canaux (`AIRCOPY_NUM_BANKS = MR_CHANNELS_MAX / 128`,
  division entière) -- avec 170, une seule banque de 128 canaux, les 42
  suivants ne seraient jamais transférés.

Présenté à l'utilisateur avec 3 options (corriger les deux effets de bord,
ne pas plafonner, ou choisir une valeur sans effet de bord type 128) --
**« faire les deux correctifs »** confirmé. Fait :
1. `SETTINGS_ResetTxLock()` réécrite pour découper par **nombre de canaux**
   (32/lot, dernier lot partiel géré proprement) plutôt que par un nombre
   fixe de lots d'octets égaux -- correct pour n'importe quel
   `MR_CHANNELS_MAX`, y compris 170. Buffer de pile désormais à taille fixe
   (`CHANNELS_PER_BATCH * CHANNEL_SIZE` = 512 o, identique à avant pour le
   cas 1024, plus de VLA dimensionné à l'exécution).
2. `MR_CHANNELS_MAX` 1024 -> 170.
3. **Aircopy reste volontairement limité** à ses 128 premiers canaux
   (limitation documentée et acceptée -- fonctionnalité annexe sans rapport
   avec SARSAT/APRS, pas retouchée).

Build vert, 0 warning (reproduit sur 2 builds successifs) :
`FLASH 115636/120832 o (95,70 %)` (-632 o), `RAM 14784/16384 o (90,23 %)`
(-224 o, table d'attributs canaux plus petite). **Non testé sur l'air.**

### Réparation croisée entre chaînes de décodage (RP2040, code partagé)

Motivée par les trames `F4DVK` lisibles mais FCS-bad du retour ci-dessus :
les 3 chaînes de décodage parallèles (`aprs_rx.c`, biais -3/8, 0, +3/8 de
l'enveloppe) décodent souvent la même salve physique en ne différant que sur
quelques octets — trop pour la réparation existante (`aprs_try_fix()`, un
seul bit), mais une chaîne voisine a très probablement le bon octet à ces
positions. Nouvelle `aprs_try_fix_cross()` (`rp2040/src/aprs_rx.c`/`.h`) :
chaque chaîne met en cache son dernier essai (`aprs_rx_t::last_cand[]`) ;
quand la réparation 1-bit échoue, on cherche un candidat de même longueur
chez une chaîne sœur, récent (< ~50 ms), on relève les octets qui diffèrent
(abandon au-delà de 5, sinon trop de combinaisons/pas la même salve), et on
teste toutes les combinaisons "mon octet / le sien" à ces positions,
jusqu'à ce que l'une passe le FCS ET ressemble à une trame UI plausible.

Vérifié : nouveau test host `cross_fix_case()` (2 chaînes synthétiques,
erreurs sur des octets disjoints, un seul bit corrigible côté chaîne A
volontairement échoué pour prouver que seule la réparation croisée aboutit)
— `make check` intégralement vert. `pico`/`pico2` recompilés proprement (0
warning), `.uf2` + `sha256.txt` régénérés dans `rp2040/bin/`. **Non testé
sur l'air** — vient d'être écrit, indépendant du diagnostic RF ci-dessus
(utile quelle que soit l'origine des erreurs de bits).

### Reste à porter/vérifier

- Historique de trames RX, réparation d'erreur bit-à-bit AX.25 côté RP2040,
  chemin digi dans le popup, icônes 16×16, texte de balise éditable : tout
  ceci vit côté RP2040 (`rp2040/src/aprs_parse.c`, partagé) et dans
  `patch/aprs.c` lui-même — déjà repris tel quel du V1, rien de spécifique à
  refaire ici au-delà de la compilation qui vient de réussir.
- Unifier la structure de menu des deux firmwares pour une documentation
  commune : demandé par l'utilisateur, explicitement pour **après** la
  validation de ce port APRS — pas commencé.

## Notes de portage générales

- `App/driver/bk4819.c` (1868 lignes) existe dans le dépôt mais **n'est
  compilé nulle part** (`App/CMakeLists.txt` ne liste que
  `driver/bk4829.c`) — code mort/référence, ignorer.
- Le firmware F4HWN gère TX_VFO/RX_VFO, `VFO_CONFIGURE`/`VFO_CONFIGURE_RELOAD`,
  `RADIO_SelectVfos`/`RADIO_SetupRegisters`, `FUNCTION_MONITOR`/`FUNCTION_TRANSMIT`
  à l'identique du V1 — tout le savoir-faire acquis sur le V1 (emprunt du slot
  VFO pour baliser sur un canal fixe, garde anti-collision avec l'écran SARSAT,
  etc.) devrait se reporter sans surprise.
- Pas de `Makefile` : toute nouvelle option de compilation doit passer par
  `App/CMakeLists.txt` (`enable_feature(...)`) + `CMakePresets.json` (valeur
  par défaut), comme fait ici pour `ENABLE_SARSAT`.

## Digipeater APRS WIDEn-N (2026-09-05)

Demandé explicitement : « Est-il possible de créer un mode digipeater APRS ?
Menu APRS Digi : Off, WIDE1, WIDE2, WIDE3 [...] Bien sûr avec le fonctionnement
APRS, si une trame arrive avec WIDE1\*, elle n'est pas répétée. À faire sur
UV-K5 V1 et UV-K1 ». Fait sur les deux firmwares. Voir `docs/protocol.md`
(section « Digipeater WIDEn-N ») pour la spécification complète du protocole
et de l'algorithme -- résumé ici :

- **Décision entièrement côté RP2040** (`rp2040/src/aprs_digi.c`/`.h`, testé
  sur hôte, `rp2040/test/host/test_aprs_digi` : 11 cas, tous verts) : il a
  déjà l'adresse AX.25 sous la main pour chaque trame décodée, donc c'est
  lui qui décide quoi répéter et mute la trame en place, plutôt que dupliquer
  cette logique sur les deux firmwares radio.
- **Nouveau champ menu `Digi`** (F+5, cycle Off/WIDE1/WIDE1+2/WIDE1+2+3) :
  2 bits ajoutés à `aprs_cfg_t::opts` (bits 5:4, `APRS_OPT_DIGI_MASK`/
  `_SHIFT`) -- pas de changement de taille de la struct (toujours 40 o), les
  bits étaient libres des deux côtés (V1 et K1/K5V3 partagent le même layout
  `opts`).
- **`APRS_PushConfig()` enfin câblée** (`0x06D0`, radio -> RP2040) : c'était
  un stub mort depuis le début du projet APRS (« TX frame builder lives in
  uart.c; Phase: sent on next hello ») -- complétée avec `SendReply()`
  (même mécanisme que `SARSAT_ReplyStatus()`), et appelée à la fois sur
  sauvegarde du menu et depuis `APRS_Init()` (le RP2040 n'a pas d'état
  persistant, il lui faut la config à chaque redémarrage de l'un ou l'autre
  côté).
- **`APRS_Beacon()` refactorée** : le cœur "encoder + basculer sur le canal
  170 + clé + émettre + restaurer" en a été extrait en `APRS_TxFrame()`,
  partagée avec la nouvelle `APRS_Digipeat()` (`0x06D6`, RP2040 -> radio) qui
  prend une trame AX.25 brute déjà mutée par le RP2040, y ajoute le FCS
  (`ax25_fcs()`, déjà exposée) et l'émet à l'identique d'une balise. Gardes
  communes factorisées dans `APRS_CanTransmitNow()` (jamais en TX, jamais
  sur l'écran SARSAT, jamais canal occupé/CSMA) -- le digipeat n'exige PAS
  un indicatif configuré (`NOCALL`) contrairement à la balise : la trame
  répétée porte déjà l'identité de la station d'origine dans son propre
  champ source, ce poste ne fait que la relayer.
- **Anti-boucle** : `aprs_digi_seen_recently()` (cache ~12 entrées, fenêtre
  glissante 30 s, clé = dst+src+hachage info) empêche de re-répéter une
  trame déjà digipeatée récemment -- nécessaire dès qu'un même poste gère
  plusieurs niveaux WIDE (ex. `WIDE1+2`) et pourrait entendre son propre
  relais revenir via un autre digipeater avant d'avoir épuisé tous les sauts.
- **Dispatch UART** : `case APRS_CMD_DIGI` ajouté à la liste `switch` de
  `App/app/uart.c` (nouvel ancrage `build.sh`), même schéma que RXTEXT/
  RXINFO/GPS.

Build vert, 0 warning : `FLASH 115932/120832 o (95,94 %)`,
`RAM 14784/16384 o (90,23 %)` (RAM inchangée -- pas de nouveau champ EEPROM,
juste 2 bits dans un octet existant). `.bin` + `sha256.txt` régénérés. RP2040
(`pico`/`pico2`) recompilé avec `aprs_digi.c` (nouveau module, ajouté à
`CMakeLists.txt`), `main.c` (dispatch `CMD_APRS_CONFIG`, tentative de
digipeat sur chaque trame décodée), `decoder_config.h` (`CMD_APRS_DIGI`),
0 warning sur les deux cibles ; `.uf2` + `sha256.txt` régénérés.

**Retour terrain : « pas de répétition de la trame »**, log RP2040 fourni
montrant `[digi] repeating #12 (62 bytes)` (la décision et l'envoi `0x06D6`
ont bien eu lieu côté RP2040) mais rien émis sur l'air. Cause trouvée par
lecture du code, pas par essai-erreur : `0x06D6` arrive tout juste après la
salve qu'il doit répéter, au moment où la radio est quasi certainement
encore en réception/monitor (rémanence du squelch) -- `APRS_Digipeat()`
tentait une seule fois immédiatement (`APRS_CanTransmitNow()`), sans jamais
réessayer, contrairement à la balise (`APRS_Beacon()`, retentée toutes les
~3 s par `APRS_TimeSlice()` jusqu'à ce que le canal se libère). Un deuxième
problème latent, trouvé en creusant le premier : le popup RX (qui s'ouvre
justement sur le paquet qu'on veut répéter) bloque la boucle principale, et
donc `APRS_TimeSlice()` elle-même -- même avec une nouvelle tentative
correctement programmée, elle ne se serait jamais exécutée tant que le
popup reste ouvert.

Corrigé : `APRS_Digipeat()` **met en file** la trame (1 emplacement) au lieu
de l'émettre directement ; une nouvelle `APRS_DigipeatTimeSlice()`, appelée
à chaque tick de `APRS_TimeSlice()` **et** à chaque itération de la boucle
de `APP_RunAprs()` (même schéma que le réassert du rétroéclairage/AF gain
déjà utilisé dans ce projet pour contourner exactement ce même blocage),
retente tant que `APRS_CanTransmitNow()` refuse, avec un abandon après ~3 s
si le canal reste occupé (mieux vaut renoncer qu'émettre une répétition
tardive). Build vert, 0 warning : `FLASH 116044/120832 o (96,04 %)`,
`RAM 15040/16384 o (91,80 %)` (+256 o pour le tampon de la trame en attente).
`.bin` + `sha256.txt` régénérés.

**Confirmé fonctionnel sans popup** (« sans popup cela fonctionne »).
Deux demandes de suivi, faites ensemble :

1. **Indicatif dans la répétition** (« peux-tu intégrer l'indicatif ? »,
   puis « en fin de trame je crois qu'il faut faire INDICATIF-SSID,WIDE2\* ;
   même fonctionnement pour WIDE1 et WIDE3 »). Après une première version
   qui remplaçait l'alias par l'indicatif au dernier saut, l'utilisateur a
   précisé le comportement attendu -- celui des vrais digipeaters (Dire
   Wolf, matériel) : `aprs_digi_process()` (RP2040) **insère une nouvelle
   adresse `INDICATIF-SSID*` devant l'alias à CHAQUE saut**, jamais
   seulement le dernier ; l'alias garde toujours son nom `WIDEn`, seuls son
   SSID (décrémenté) et son bit H (posé une fois à 0) changent. `WIDE2-2`
   reçu -> `INDICATIF-SSID*,WIDE2-1` ; après qu'un 2ᵉ digipeater le termine
   -> `IND1*,IND2*,WIDE2*`. Uniforme pour WIDE1/2/3. Repli sur un simple
   décrément en place (pas d'insertion) sans indicatif configuré (`NOCALL`)
   ou si les 7 o supplémentaires ne tiennent pas. `my_call`/`my_ssid`
   poussés par `0x06D0`, jamais exploités côté RP2040 avant. Bug latent
   corrigé au passage : le décrément ne repositionnait pas les bits
   réservés `0x60`.
2. **Deux verrous anti-boucle** (« as-tu mis des verrous pour ne pas
   répéter sa propre trame [...] ni une trame déjà répétée [indicatif
   présent dans la liste des digi] ? »). Ajoutés dans `aprs_digi_process()`,
   testés avant tout le reste dès qu'un indicatif est configuré :
   - trame dont l'adresse **source** = notre indicatif-SSID -> jamais
     répétée (notre propre trame revenue par un autre chemin) ;
   - notre indicatif-SSID **déjà présent** quelque part dans la liste
     digi/via (utilisé ou non) -> jamais répétée (déjà passée par nous une
     fois). Ne périme jamais, contrairement à la suppression de doublon
     temporelle (~30 s) qui reste en place en complément.
3. **Petit délai avant la répétition** (« un petit délai entre la fin de
   trame et la répétition ? »). `APRS_DigipeatTimeSlice()` (les deux
   firmwares radio) attend `APRS_DIGI_MIN_DELAY_10MS` (~300 ms) depuis la
   mise en file avant la toute première tentative d'émission, même si le
   canal est déjà libre.

`test_aprs_digi` : 19 cas, tous verts (insertion WIDE1/2/3, relais suivant
qui termine la trace, replis, les 2 verrous + un contrôle de non-faux-
positif). Build vert, 0 warning partout. `.bin`/`.uf2` + `sha256.txt`
régénérés. **Non testé sur l'air.**

### Rétroéclairage "Light on frame" actif même avec Popup désactivé

Remonté : « quand popup est désactivé mais VFO sur APRS et menu APRS réglé
sur "Light on frame", l'éclairage s'allume uniquement sur détection de
trame -- il faudrait revenir en mode normal si popup off ». Confirmé dans
le code : `APRS_QuietBacklight()` ne regardait que le réglage "Light on
frame" (`APRS_OPT_BL_DECODE`) et la bande, jamais `popup_s` -- avec les
popups désactivés, le rétroéclairage stock sur simple ouverture squelch
restait donc supprimé en permanence dès qu'on est sur 144-148 MHz, sans
qu'aucun popup ne vienne jamais justifier cette sobriété. Corrigé : la
fonction exige maintenant aussi `popup_s != 0` -- popups désactivés =
comportement stock intégral, quel que soit le réglage "Light on frame".
Build vert sur les deux firmwares, 0 warning. `.bin` + `sha256.txt`
régénérés. **Non testé sur l'air.**

### Non-répétition avec popup ouvert -- correctif appliqué

Sur demande (« nous regarderons ensuite le problème [...] tu peux déjà
chercher et me faire une proposition », puis « tu peux en même temps faire
le correctif »). Cause trouvée par lecture de code : `APRS_CanTransmitNow()`
testait `gCurrentFunction == FUNCTION_RECEIVE || FUNCTION_MONITOR` pour
« canal occupé » -- mais `App/functions.h` documente `FUNCTION_RECEIVE`
comme « RX mode, **squelch closed** » (état de repos), et
`FUNCTION_INCOMING` (jamais testé) comme « receiving a signal (**squelch
open**) ». Le mauvais état était testé. Et de toute façon `gCurrentFunction`
n'est mis à jour que par le tick principal, que la boucle bloquante du
popup gèle : il restait figé sur sa valeur d'entrée (quasi toujours
« en réception », le popup s'ouvrant *sur* un paquet décodé) tant que le
popup était ouvert, quel que soit l'état réel du canal.

**Corrigé** (les deux firmwares) : le test « canal occupé » utilise
maintenant `g_SquelchLost` directement (vrai = porteuse présente). Cette
variable est tenue à jour en temps réel dans les deux contextes -- par le
tick principal normalement, par le mini-squelch de `APP_RunAprs()` pendant
le popup -- donc elle ne peut pas se figer. Build vert, 0 warning.
**Non testé sur l'air** (à confirmer, mais cause démontrée par les
commentaires de l'énumération et le blocage du tick).


### AFC désactivée sur la bande APRS (TEST, K1/K5V3 uniquement)

Remonté : « la première trame n'est pas décodée s'il n'y a pas eu de
réception récente ». Analyse (les deux firmwares comparés) :
- Le seul « économiseur » périodique est `FUNCTION_POWER_SAVE`/`gRxIdleMode`
  (= BATTERY_SAVE), déjà inhibé sur 144-148 MHz par `APRS_KeepAwake()`.
  Réserve : `aprs_on_band()` ne regarde que `gEeprom.RX_VFO` → en Dual Watch
  il peut évaluer le mauvais VFO. Le Dual Watch lui-même fait rater des
  salves (écoute à temps partiel + « parking » sur le VFO après une RX).
- **AFC** : `RADIO_SetModulation()` active l'AFC en `MODULATION_FM` sur les
  **deux** firmwares (REG_73 bit 4 = 0). Sur le K1/K5V3 elle est coupée en
  mode « RAW »/DSC (`BK4819_EnterRaw()`) ; sur le V1 elle reste active même
  en « DSC » (`MODULATION_DISCRI`, `radio.c.diff` la traite comme FM). En
  FM, canal au repos → l'AFC dérive sur le bruit → première salve décalée
  en fréquence → trame ratée ; l'AFC se recale ensuite. Même mécanisme que
  celui **confirmé sur l'air pour le SARSAT** (corrigé là par
  `BK4819_SetRegValue(afcDisableRegSpec, true)` dans `APP_RunSarsat()`).

**Essayé puis retiré (K1/K5V3 seul, V1 toujours intouché)** : une
`APRS_DisableAfc()` réassénée à chaque tick de `APRS_TimeSlice()` posait
REG_73 bit 4 (AFC disable) dès que le VFO RX était dans 144-148 MHz — même
schéma que `APRS_ApplySquelch()`, sans toucher squelch ni gain.

**Résultat sur l'air : aucune amélioration** (« pas mieux »). Deux logs
RP2040 successifs (RAW puis FM, `clip=0.0%`, gain AF calé, `env` stable
~400-480k) montrent des trames toujours lisibles (`src=F4DVK`, `F8BEC-9`,
`F8KCS-3`…) mais **toujours `BAD`** : 2-4 erreurs de bits dispersées par
trame, à des positions différentes sur les 3 chaînes de slicer, et sur
certains octets « chauds » les **trois** chaînes divergent (désaccord
triple = l'audio est réellement corrompu à cet instant, pas un artefact de
slicer). Les erreurs sont donc **dans le domaine RF** — le discriminateur
du K1/BK4829 est plus bruité que celui du V1/BK4819 pour les mêmes signaux
sur ce banc — et non une dérive de LO. Cohérent avec le constat de
l'utilisateur : « l'UV-K1 est plus sensible aux perturbations HF », « quand
je m'écarte ça fonctionne ». `APRS_DisableAfc()` **supprimée sur demande** ;
l'AFC repasse à son comportement `MODULATION_FM` d'origine (active), comme
le V1. Build vert, 0 warning : `FLASH 116048/120832 o (96,04 %)`. `.bin` +
`sha256.txt` régénérés.

**Épilogue (confirmé par l'utilisateur) : c'était un défaut matériel de son
montage C-Board.** Le même RP2040 avec les mêmes binaires décode normalement
sur un autre exemplaire. La chaîne de démodulation RP2040 (BPF, limiteur
dur, slicer à suivi de crêtes, LPF 13 taps, 3 slicers, PLL, HDLC, FCS) est
restée octet pour octet identique au commit initial ; le seul ajout
RP2040 depuis (`aprs_try_fix_cross()`) est purement additif (réparation
post-échec FCS). Le passage des 3 pads GP26/GP27/GP28 en mode analogique
(`CFG_ADC_SHORTED_MASK = 0x07`, inchangé depuis l'origine) est correct pour
le montage court-circuité — il **évite** que les buffers logiques de
GP27/GP28 chargent le nœud audio, il ne peut pas dégrader le décodage.
Rien à corriger côté firmware.


### Message « report balise 121 MHz » (2026-09-06)

Même fonction que sur le port V1 (voir `firmware/uv-k5v1-kd8cec/integration.md`
pour le détail), portée ici à l'identique. 100 % côté radio, aucun changement
RP2040.

- **`aprs_cfg_t` +16 o** (40 -> 56, 7 pages EEPROM) : `char msg_to[10]` +
  `_rsv[6]`. `build.sh` : le mapping `eeprom_compat.c` de la config APRS passe
  de `0x00A178..0x00A1A0` (40 o) à `0x00A178..0x00A1B0` (56 o) -- toujours dans
  la queue non revendiquée du secteur « Settings », aucune collision. Le
  `grep -q` de contrôle est mis à jour en conséquence. `APRS_Init()` assainit
  `msg_to` pour une EEPROM écrite par un build antérieur.
- Champs menu `To <call>` + `Send report`, assistant Signal/Direction,
  trame `INDICATIF>APZSAR,WIDE1-1,WIDE2-2:` + `:DEST:Report Balise 121 MHz
  S: n Dir: ddd|KO <lat> <lon>{NN`. Détail complet côté V1.
- `<lat> <lon>` = position résolue (`APRS_MyPosition()` : GPS si mode GPS +
  fix, sinon lat/lon manuel) en degrés décimaux, 4 décimales, signées,
  tronquées ; `0.0000 0.0000` si rien de réglé.
- **Accusé de réception** : 3 ré-émissions / 30 s puis **écoute jusqu'à
  5 min** (`APRS_MSG_WAIT_10MS`) ; `APRS_MsgCheckAck()` accepte un `ackNN`
  tardif, même après abandon (`FAIL -> ACK`), compare l'indicatif de base
  (SSID toléré). La ligne `Send report` affiche le **numéro de message** :
  `Rpt #NN TX n/3` -> `Rpt #NN wait ack` -> `Rpt #NN ACK OK` /
  `Rpt #NN no ack`, pour que l'opérateur voie quel `ackNN` attendre
  (`s_msg.seq` s'incrémente à chaque report, repart de 1 au reboot).
  Correctifs de deux remontées : (1) « popup `ackNN` après avoir quitté le
  menu, ne passe pas OK » -- les trames sont traitées dans le menu, la
  fenêtre d'accusé était trop courte ; (2) « ack dans les temps mais ne
  passe pas » -- numéro attendu > 1 après plusieurs essais. Voir le détail
  côté V1. RP2040 `main.c` : log message = `to [ADRESSEE] : TEXTE`.
- L'assistant réutilise `APRS_TickDelay()` (pas `SYSTEM_DelayMs`) dans sa
  boucle, comme le reste de cet écran (fondu rétroéclairage F4HWN).
- **Réception coupée dans le menu APRS -- corrigé** (« LED s'allume, du
  souffle, mais la trame ne passe pas ») : (1) `APRS_TxFrame()` ré-applique
  `APRS_ApplySquelch()` + `AFGAIN_Apply()` en fin (le tick, bloqué par
  `APP_RunAprs()`, ne le fait plus après une ré-émission de report / un
  digipeat) et la boucle appelle `APRS_ApplySquelch()` à chaque itération ;
  (2) le mini-squelch de la boucle diffère la coupure HP de **~2 s**
  (`msq_mute_at`) pour ne pas hacher le flux vers la C-Board entre paquets
  rapprochés. (3) à l'arrivée d'un paquet, la boucle bascule sur la **vue
  RX** même en ouverture manuelle du menu (le décodage tournait, l'écran
  config ne le montrait pas). Détail complet côté V1.
- **Ack toujours pas pris en compte -- durci** (log
  `to [F4DVK-14 ] : ack01`) : `APRS_MsgCheckAck()` accepte un numéro
  `1..s_msg.seq` (client lent qui ré-accuse `ack01`), compare l'indicatif
  base à base **les deux côtés élagués** (`gAprsCfg.call` peut traîner une
  espace), et mémorise `last_ack_no` -> la ligne bloquée affiche
  `#03 wait  got a01` / `#03 no ack got a01` comme diagnostic. Détail V1.
- **Audio de la trame dans le menu** (« pas d'audio » -> « aucun souffle FM »
  -> « squelch met plusieurs secondes à couper » -> « squelch s'ouvre mais
  pas d'audio »). Cause racine (surtout V1, cf. détail là-bas) : un TX AFSK
  laisse `BK4819_SetAF(AF_MUTE)` ; le `RADIO_SetupRegisters()` de ce firmware
  ré-appelle bien `RADIO_SetModulation()` mais on l'appelle explicitement
  aussi -- dans `APRS_TxFrame()` (fin) **et** à l'ouverture manuelle de
  l'écran : `RADIO_SelectVfos` + `RADIO_SetupRegisters(true)` +
  `RADIO_SetModulation(gRxVfo->Modulation)` + `gEnableSpeaker` +
  `APRS_ApplySquelch` + `AFGAIN_Apply` (réglage opérateur **conservé**). HP
  démarre **coupé**, le mini-squelch de la boucle l'ouvre sur une porteuse.
  `APP_StartListening()` retiré (forçait le HP ouvert). Détail V1.
- **Menu ouvert manuellement : décodage en arrière-plan, pas de popup**
  (demandé) : un paquet reçu pendant qu'on est dans l'écran config ne
  bascule pas sur la vue RX -- seul un auto-popup (écran fermé) le fait. Le
  décodage + la reconnaissance de l'accusé de report tournent quand même
  (ligne `Send report` -> `ACK OK` sans interrompre) ; `*` pour la dernière
  trame. Détail V1.
- **Passe d'optimisation flash** : `APRS_TxInfo()` factorise
  `APRS_Beacon()`/`APRS_MsgTx()`, diagnostic `last_ack_no` retiré, `#include
  "app/app.h"` retiré. **`FLASH 118088/120832 o (97,73 %)`** (−152 o).
- Build vert, 0 warning, `RAM 15072/16384 o`. `.bin` + `sha256.txt`
  régénérés. **Non testé matériel.**

## Mode TNC KISS (2026-09-06)

Miroir du V1 (détail complet là-bas). Le gros est côté RP2040
(`rp2040/src/kiss.c`/`.h`, codec SLIP/KISS, testé hôte
`rp2040/test/host/test_kiss`). Côté radio :

- **`aprs_cfg_t::opts` bit 6** (`APRS_OPT_KISS`, `0x40`) -- struct 56 o
  inchangée. Champ menu `KISS TNC on/off` (`F_KISS`, après `Send report`),
  toggle immédiat -> `APRS_PushConfig()`.
- **`APRS_PushConfig()`** : 12ᵉ octet de charge = `flags`, bit 0 = KISS
  (`SendReply(UART_PORT_UART, b, 16)`).
- **`APRS_TimeSlice()`** : `if (kiss) return;` juste après
  `APRS_DigipeatTimeSlice()` -- garde le fast-squelch, le gain AF
  (`AFGAIN_TimeSlice()`) et le relais des trames (`0x06D6 -> APRS_Digipeat`),
  coupe l'auto-balise / le report 121 / l'animation du symbole GPS.
- **`aprs_rx_arrived()`** : `if (kiss) return;` en tête.
- **`draw_config()`** : en-tête `KISS TNC  host USB` quand actif.

Build vert, 0 warning : **`FLASH 118224/120832 o (97,84 %)`** (+~150 o).
`.bin` + `sha256.txt` régénérés. La chaîne KISS (codec RP2040 + hôte) est
**validée sur matériel** (2026-09-06, `kissutil`, RX + TX bout en bout — cf.
détail V1) ; le port radio ici est un miroir verbatim du V1 (mêmes bits
`opts`, même gating `APRS_TimeSlice`).

## SmartBeaconing APRS (2026-09-06)

Miroir du V1 (barème et détail complet là-bas). Champ menu `Interval` (F+5) :
deux valeurs en plus, **`SB car`** (voiture) et **`SB foot`** (piéton) ;
cadence de balise variable selon la vitesse GPS + corner pegging tant qu'il y
a `Pos = GPS` + fix valide.

| | `SB car` | `SB foot` |
|---|---|---|
| bas / haut | 5 / 90 km/h | 2 / 8 km/h |
| lent / rapide | 1200 / 30 s | 600 / 90 s |
| virage mini / pente / temps mini | 25° / 255 / 25 s | 35° / 80 / 45 s |

**Repli sans fix GPS live** (`Pos = manual`, ou GPS en recherche) : balise à
intervalle fixe = la cadence lente du profil (1200 s / 600 s) sur la position
résolue. `manual` + lat/lon valides balise lentement ; `GPS` sans fix reste
silencieux.

`patch/aprs.c` : sentinelles `interval_s == 1|2`, table `s_sb_prof[2][7]`,
`APRS_SmartBeaconDue()`. `APRS_TimeSlice()` : SB live / repli fixe (cadence
lente) / période numérique. Build K1/K5V3 vert, 0 warning :
**`FLASH 118576/120832 o (98,13 %)`** (+352 o), `RAM 15104`. `.bin` +
`sha256.txt` régénérés. **Non testé matériel.**

## Canaux par défaut SAREX / SARSAT (2026-09-06)

`APRS_EnsureChannel()` -> `APRS_SeedChannel()` : canal 170 puissance par défaut
**moyenne -> haute** ; canaux 1 et 2 pré-remplis s'ils sont vierges —
`SAREX` 434.200 MHz RAW / `SARSAT` 406.028 MHz RAW, large + puissance basse
(`OUTPUT_POWER_LOW1`). Jamais par-dessus un canal utilisé. Build K1/K5V3 vert,
0 warning : `FLASH 118664/120832 o (98,21 %)` (+88). `.bin` + `sha256.txt`
régénérés. Non testé matériel.

## Écran ADRASEC + accusé automatique des messages (2026-09-07)

Identique au port V1 (voir `firmware/uv-k5v1-kd8cec/integration.md` et
`docs/protocol.md`) : l'accusé automatique est 100 % RP2040 (0 o ici), l'écran
ADRASEC collant est ajouté à `patch/aprs.c` (`s_adrasec` / `s_adr` /
`draw_adrasec()` / `view == 2` dans `APP_RunAprs()`, EXIT seul). Le F4HWN n'a
pas de drapeau `SMALL_BOLD` — inutile ici, la marge flash suffit. Build vert,
0 warning : **`FLASH 119280/120832 o (98,72 %)`**, RAM inchangée. `.bin` +
`sha256.txt` régénérés. **Non testé matériel.**

### (2026-09-07) Rafraîchissement du compteur report

`APP_RunAprs()` redessine la ligne « Send report » à chaque (ré)émission du
message report 121 MHz (suivi de `s_msg.tries`). La surbrillance grasse est
conservée telle quelle (le F4HWN a `gFontSmallBold`, pas de `SMALL_BOLD` à
couper). Build vert, 0 warning : `FLASH 119308/120832 o`.

### (2026-09-07) Icône « digi » sélectionnable dans le menu APRS

Demandé pour les deux firmwares. Une entrée `{ '/', '#', "digi" }` ajoutée à
la table `SYMS[]` de `patch/aprs.c` (après `dot`) : le champ **Icon** du menu
F+5 propose maintenant `digi` = l'étoile verte APRS du symbole digipeater
(`/#`). Le bitmap 16×16 correspondant (`ICON_BMP[10]`) et le mappage
`icon_index('/', '#') → 10` existaient déjà (utilisés pour afficher les digis
entendus dans le popup RX) — seul manquait le choix pour sa propre balise.
Aucun autre changement. Build vert, 0 warning : `FLASH 119280/120832 o`
(+12 o) ; V1 `text 61068 o` (inchangé, absorbé par l'alignement). `.bin` +
`sha256.txt` régénérés des deux côtés. **Non testé matériel.**

### (2026-09-09) Chaîne audio RX alignée sur le V1 pour SARSAT et APRS

Constat de l'opérateur : à antenne/signal comparables, **le V1 (UV‑K5 V1,
BK4819/KD8CEC) décode mieux — surtout l'APRS** — et il faut un gain AF fixe
différent (11 sur V1, 16 sur V3) pour le même niveau RMS lu par le RP2040.
Le code démod RP2040 est identique pour les deux radios : la différence est
uniquement dans l'audio que chaque firmware sort vers la C‑Board. Écarts
relevés dans la chaîne RX :

| | V1 (BK4819 / KD8CEC) | V3 (BK4829 / F4HWN) — avant |
|---|---|---|
| REG 0x54 / 0x55 (filtre/EQ AF) | jamais touché (défaut puce `0x9009`/`0x31A9`) | `RADIO_SetModulation()` les réécrit depuis « SetRxA » (défaut `FLAT` = `0x9009`/`0x3200`) |
| REG 0x43 (filtre FI/RF, « WIDE ») | `0x45A8` (pilote BK4819, fort 4 / faible 2) | `0x3028` (pilote BK4829, fort 3 / faible 0) — plus étroite, et se resserre plus sur signal faible |
| REG 0x7B (temps AGC) | `0x8420` (`InitAGC` profil FM) | `0x73DC` (`InitAGC` profil unique « stock BK4829 », AM+FM) |
| REG 0x49 (seuils RSSI AGC) | `0x2A38` (84/56, profil FM) | `0x2AB2` (85/50, profil unique) |
| REG 0x10..0x14 (tables de gain AGC) | tables BK4819 | tables BK4829 — **non alignées** (les crans LNA/PGA ne se correspondent pas 1:1) |
| REG 0x73 (AFC : plage + dynamique) | jamais écrit → défaut puce, seul le bit 4 (on/off) est basculé | init `0x4691` (→ `0x4681` en FM) |
| AFC on/off écran SARSAT | active | désactivée (correctif BK4829 confirmé air) |
| AFC on/off APRS | active (FM) | active (FM) — identique |
| Courbe gain AF | écriture REG_48 identique — l'écart 11/16 vient du niveau discri / AGC amont (silicium), pas d'un réglage |

**Non, l'AFC et l'AGC ne sont PAS à la même vitesse.** `BK4819_InitAGC()` de
ce firmware utilise **un seul** profil (« stock BK4829 », `REG_7B = 0x73DC`)
pour AM et FM ; le V1 a un profil FM dédié (`REG_7B = 0x8420`). REG_7B porte
les constantes de temps attaque/décroissance de l'AGC → vitesse différente.
Idem pour l'AFC : le V1 laisse REG_73 au défaut puce (seul le bit 4 bascule),
le V3 l'initialise à `0x4691` — plage et dynamique de correction de fréquence
différentes.

**⚠️ TOUT L'ALIGNEMENT REGISTRE A ÉTÉ ANNULÉ EN BLOC (2026-09-09).**

Les forçages successifs (REG 0x54/0x55 → `0x31A9`, REG 0x43 → `0x45A8`,
REG 0x7B → `0x8420` + REG 0x49 → `0x2A38`, REG 0x2B bits 10:8 débrayés,
REG 0x2A → `0x6600`, REG 0x4E bit 8 nettoyé), appliqués un à un dans
`APRS_ApplyRxAudio()` et l'écran SARSAT, ont fini par **tuer complètement le
décodage APRS** sur le matériel de l'opérateur. Cause exacte non isolée — trop
de variables changées à l'aveugle sur un chip (BK4829) sans doc de registres,
et en particulier forcer le profil AGC du BK4819 (REG 0x7B/0x49, calibrés
pour des tables de gain REG 0x10..14 *différentes*) était probablement le plus
dangereux.

**État après annulation :**
- `APRS_ApplyRxAudio()` → **corps vide** (no-op), gardée pour ne pas toucher
  aux 4 sites d'appel / `aprs.h`. La chaîne RX APRS du V3 est maintenant
  exactement celle de F4HWN stock (`RADIO_SetModulation` + `RADIO_SetupRegisters`),
  même philosophie que le V1 (qui ne force rien ici au-delà du délai
  d'ouverture squelch de `APRS_ApplySquelch()`).
- `APRS_ApplySquelch()` → masque revenu à **`~0x3E00`** (identique au V1,
  délai d'ouverture 0 / fermeture 3, rien d'autre).
- `patch/sarsat.c` → écritures `0x54/0x55` retirées ; le profil SetRxA
  (FLAT par défaut) tient, comme l'état validé sur l'air.
- **Conservé** (sûr, pur gain, aucune divergence par rapport au V1) :
  correctif battery-save (`APRS_KeepAwake()` sur les deux VFO + réveil forcé),
  et le garde-fou scramble/compander était dans `APRS_ApplyRxAudio()` — donc
  retiré aussi avec le reste ; à re-ajouter seul si un jour un canal APRS
  compandé pose problème.

**Le tableau comparatif ci-dessus reste valable comme référence** (écarts
réels entre BK4819 et BK4829), mais **aucun n'est plus corrigé en firmware**.
Pour ré-essayer : ré-ajouter **un seul** registre à la fois dans
`APRS_ApplyRxAudio()`, avec un test sur l'air entre chaque.

**« Un DSP qui amène de la latence ? »** Oui — les filtres audio de la puce
(REG 0x2B dé-emphase/HPF/LPF, REG 0x54/0x55) ont un retard de groupe
(sous-ms à ~1 ms), mais le BK4829 fait de toute façon plus de traitement
numérique interne (FI numérique, squelch numérique) que le BK4819 — pipeline
plus long, squelch plus lent, sortie discri plus faible/bruitée. **Non
corrigeable en firmware** — c'est du silicium.

Build vert, 0 warning : `FLASH 109576/120832 o (90,68 %)`,
`RAM 14400/16384 o`. `.bin` + `sha256.txt` régénérés. Le V1 garde uniquement
le correctif battery-save (sûr).

### (2026-09-09) Squelch APRS — REG_4E bit 8

Remonté : « le squelch me paraît plus long à s'ouvrir sur la V3 ». Le champ
*délai d'ouverture* de REG_4E est déjà mis à 0 par `APRS_ApplySquelch()` sur
les deux firmwares (code identique). Différence trouvée : le
`BK4819_SetupSquelch()` de **ce firmware (BK4829)** pose en plus **REG_4E
bit 8** (`(1u << 8)`, commenté « matches stock BK4829 ») ; le pilote BK4819
du V1 laisse ce bit à 0. `APRS_ApplySquelch()` ne le nettoyait pas (masque
`~0x3E00`, bits 13:9 seulement) → il restait à 1 sur le V3.

Masque élargi à `~0x3F00` (bits 13:8) — puis **ANNULÉ** avec le reste des
essais d'alignement RX quand l'opérateur a perdu tout décodage. `APRS_ApplySquelch()`
est revenu au masque **`~0x3E00`** (identique au V1 : délai d'ouverture 0 /
fermeture 3, rien d'autre).

**Rappel** : pour zéro perte de flags, le plus sûr reste `SQL = 0` (monitor)
dans le menu radio.

### (2026-09-09) Battery-save — « il faut une 1ʳᵉ trame ratée pour en recevoir une »

Symptôme classique du duty-cycle de l'économiseur de batterie
(`FUNCTION_POWER_SAVE` / `gRxIdleMode` → `BK4819_Sleep()` + GPIO RX bas
périodiquement). Une trame qui arrive pendant la sieste ne réveille le RX
qu'à moitié → la 1ʳᵉ est perdue, la suivante décode.

`APRS_KeepAwake()` (câblée dans la liste d'inhibition `gSchedulePowerSave` de
`app/app.c` depuis le début du port) existait déjà, mais ne testait que
`gEeprom.RX_VFO` — **en Dual Watch `RX_VFO` alterne**, donc l'économiseur
restait libre de se déclencher pendant la demi-période où le scanner est sur
l'autre VFO. Corrigé (deux firmwares) : `APRS_KeepAwake()` teste **les deux
VFO** (si l'un est dans 144–148 MHz → inhibition). En complément,
`APRS_TimeSlice()` fait `FUNCTION_Select(FUNCTION_FOREGROUND)` si on est déjà
en `FUNCTION_POWER_SAVE` sur la bande APRS (cas : radio déjà endormie au
moment d'arriver sur 144.8).

**Limite** : le Dual Watch écoute quand même 144.8 seulement ~50 % du temps —
pour une RX APRS fiable, utiliser **Main Only**. `BATSAVE` peut rester ON.
V3 `FLASH 109764/120832 o (90,84 %)` ; V1 `text 57988 o` (+72). `.bin` +
`sha256.txt` régénérés des deux côtés. **Non testé sur l'air.**

### (2026-09-09) F+5 bascule le RX sur le canal APRS 170 + « mode APRS » élargi

Demandé (pour que l'accusé du futur message « position balise SARSAT »
fonctionne) : à l'ouverture du menu APRS, la RX doit être sur 144.800, et les
traitements bande‑APRS doivent s'appliquer dès qu'on est sur le canal 170.

- **`APP_RunAprs()` (deux firmwares)** : à l'ouverture **manuelle** (F+5, pas
  auto‑popup), emprunte le slot `TX_VFO` pour le canal `APRS_TX_CHANNEL`
  (MR 170) pendant toute la durée du menu — même recette que `APRS_TxFrame()`
  (sauvegarde `VfoInfo`/`ScreenChannel`/`MrChannel`/`RX_VFO`,
  `RADIO_ConfigureChannel(VFO_CONFIGURE_RELOAD)`, restauration dans la séquence
  de sortie existante). `monitor` devient `!popup` (on retune toujours vers
  170 → RX FM live immédiatement, quel que soit la fréquence de départ :
  406 MHz SARSAT, un canal quelconque…). `RX_VFO` forcé sur le VFO actif
  (mono‑VFO le temps du menu). L'auto‑popup, lui, hérite de l'état RX comme
  avant (il s'est ouvert parce qu'une trame a décodé → radio déjà sur 144.8).
- **Reconnaissance « mode APRS » — précisée (demande utilisateur)** :
  helper `aprs_vfo_is_aprs(v)` (deux firmwares) :
  - **mode mémoire** : vrai **uniquement** sur le canal 170
    (`ScreenChannel[v] == APRS_TX_CHANNEL`). Aucun changement.
  - **mode VFO** : vrai **uniquement** si `freq_config_RX.Frequency` est
    **exactement égale** à la fréquence du canal 170 — **pas** toute la bande
    144–148 MHz (donc 145.500 simplex, etc. ne déclenchent rien).
  La fréquence du canal 170 est lue via `SETTINGS_FetchChannelFrequency()` et
  **mise en cache** (`s_aprs_ch_freq`, `aprs_refresh_ch_freq()` — rafraîchie
  par `APRS_Init()` et à chaque emprunt de canal par F+5) : `aprs_on_band()`
  tourne à chaque tick, on évite une lecture EEPROM répétée. Respecte un canal
  170 dont la fréquence a été éditée (ex. 144.390 en Amérique du Nord).
  `aprs_on_band()` = `aprs_vfo_is_aprs(RX_VFO)` ;
  `APRS_KeepAwake()` = `aprs_vfo_is_aprs(0) || aprs_vfo_is_aprs(1)` (les deux
  VFO, pour le Dual Watch).

Builds verts, 0 warning : V3 `FLASH 109940/120832 o (90,99 %)`,
`RAM inchangée` ; V1 `text 58168 o` (marge ~3,3 Ko). `.bin` + `sha256.txt`
régénérés des deux côtés. **Non testé sur l'air.**

### (2026-09-09) Message APRS « Send SARSAT » (position de balise décodée)

Demandé : relayer par **message APRS** (comme le report 121, même
destinataire `gAprsCfg.msg_to`, avec accusé) la position d'une balise SARSAT
décodée. Confirmation opérateur avant l'envoi.

- **RP2040** (`main.c`) : `radio_push_beacon()` émet `CMD_SARSAT_BEACON`
  (`0x06C2`, 29 o packé LE — cf. `docs/protocol.md`) après chaque décodage
  propre, en plus des lignes `0x06C1`.
- **Radio `sarsat.c`** : le handler `SARSAT_CMD_BEACON` (jusqu'ici un simple
  ACK) parse la charge et appelle `APRS_NoteBeacon()`.
- **Radio `aprs.c`** : cache `s_bcn` (hexID, lat/lon e5, pays, flags). La
  machine à états `s_msg` gagne un champ **`kind`** (`APRS_MSG_K_REPORT` /
  `_BEACON`) — un seul état, report 121 et balise mutuellement exclusifs.
  `APRS_MsgInfo()` branche sur `kind` :
  `:DEST     :SARSAT <hexID> <lat> <lon> c<pays>[ TEST]{NN`
  (ou `NOPOS` sans position ; lat/lon = degrés décimaux signés 4 déc., même
  format que le report). Réutilise `APRS_MsgTx/TimeSlice/CheckAck` tels quels
  (3 ré‑émissions / 30 s, écoute 5 min, `WIDE1-1,WIDE2-2`, accusé via
  `0x06D3`).
- **Menu F+5** : nouveau champ `F_SENDB` après `Send report` :
  `SARSAT: no bcn` / `Send SARSAT <4 hex>` / `Bcn #NN TX n/3` / `wait ack` /
  `ACK OK` / `no ack`. Appui → `s_wiz = 3` = écran de confirmation
  « Send SARSAT ? » + hexID + « with/no position » + « A = send / EXIT ».
  L'ack RX marche parce que F+5 met déjà la radio sur le canal 170 (voir
  section ci‑dessus).

Builds verts, 0 warning : V3 `FLASH 110688/120832 o (91,60 %)` (+748 o) ;
V1 `text 59220 o` (+1052, **marge ~2,2 Ko**) ; RP2040 pico `text 85476` /
pico2 `text 80060`. `.bin` / `.uf2` + `sha256.txt` régénérés partout,
`docs/protocol.md` à jour. `make check` vert. **Non testé sur l'air.**