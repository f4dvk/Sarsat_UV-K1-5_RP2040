#!/usr/bin/env bash
# Compile le firmware F4HWN (armel/uv-k1-k5v3-firmware-custom) avec l'écran
# SARSAT pour UV-K1 / UV-K5 V3 (PY32F071 + BK4829).
#
# Port du firmware UV-K5 V1 (KD8CEC) du même projet : même C-Board RP2040,
# même protocole série (docs/protocol.md), même écran SARSAT -- seule la
# glue côté radio change (API UART à Port explicite, macros CMake au lieu
# d'un Makefile). Le lien RP2040 <-> radio est inchangé, aucune modification
# côté rp2040/ n'est nécessaire.
#
# Prérequis : git, perl, cmake, ninja, arm-none-eabi-gcc (13.x testé).
# Pas besoin de Docker : le toolchain CMake pointe directement sur
# arm-none-eabi-gcc du PATH (cmake/gcc-arm-none-eabi.cmake).
#
# Usage :
#   ./build.sh [TAG] [PRESET]
#     TAG     tag ou commit amont (défaut : v5.9.0)
#     PRESET  preset CMake        (défaut : Fusion -- affichage F4HWN classique)
#
set -euo pipefail

UPSTREAM=https://github.com/armel/uv-k1-k5v3-firmware-custom
TAG=${1:-v5.9.0}
PRESET=${2:-Fusion}

HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

echo "== clone $UPSTREAM @ $TAG"
git clone --depth 1 --branch "$TAG" "$UPSTREAM" "$WORK/fw"
cd "$WORK/fw"
COMMIT=$(git rev-parse --short HEAD)

echo "== fichiers de l'écran SARSAT, du réglage de gain C-Board, du tracker APRS et de l'écran principal 'icom'"
cp "$HERE/patch/sarsat.c"  App/app/sarsat.c
cp "$HERE/patch/sarsat.h"  App/app/sarsat.h
cp "$HERE/patch/afgain.c"  App/app/afgain.c
cp "$HERE/patch/afgain.h"  App/app/afgain.h
cp "$HERE/patch/aprs.c"    App/app/aprs.c
cp "$HERE/patch/aprs.h"    App/app/aprs.h
cp "$HERE/patch/ax25.c"    App/app/ax25.c
cp "$HERE/patch/ax25.h"    App/app/ax25.h
cp "$HERE/patch/sonde.c"   App/app/sonde.c
cp "$HERE/patch/sonde.h"   App/app/sonde.h
cp "$HERE/patch/ui_main.c" App/ui/main.c

echo "== points d'ancrage"
# Chaque insertion est faite une seule fois (perl en mode "slurp", pas de /g)
# sur un clone neuf ; un échec d'ancrage est fatal (contrôle plus bas).

# App/frequencies.c : plafond RX réel du BK4829 (retour terrain : "souffle
# mais pas de RX à 1297 MHz sur l'UV-K1, fonctionne sur le V1"). Ce fichier est
# hérité tel quel du fork BK4819 (commentaire d'origine : "18MHz ~ 630MHz and
# 760MHz ~ 1300MHz"), jamais revalidé pour le BK4829. Fiche technique BK4829
# fournie par l'utilisateur : "Worldwide band: 18 MHz ~ 580 MHz, 760 MHz ~
# 1160 MHz" -- 3 constantes à corriger en conséquence (le V1/BK4819 garde ses
# valeurs d'origine, non concerné) :
#   - BX4819_band2_upper (plafond haut, sert de fait à BAND7_470MHz.upper en
#     ENABLE_WIDE_RX) : 1300 MHz -> 1160 MHz -- la cause directe du symptôme
#     remonté (1297 MHz tombait dans une zone où le récepteur BK4829 n'a plus
#     de front-end RF fonctionnel : souffle large-bande générique, jamais de
#     signal utile).
#   - BX4819_band1.upper (haut de la 1ʳᵉ bande, sert au "saut" automatique
#     hors du trou PLL entre les deux bandes de synthèse) : 630 -> 580 MHz --
#     au-delà, même symptôme latent (pas encore remonté) entre 580-630 MHz.
#   - BX4819_band2.lower (bas de la 2ᵉ bande, même mécanisme de saut) :
#     840 -> 760 MHz -- dans l'autre sens : le trou PLL actuel est trop large
#     et fait sauter par-dessus 760-840 MHz, qui est en fait de la bande
#     valide d'après la fiche technique.
perl -0pi -e 's/#define BX4819_band2_upper 130000000/#define BX4819_band2_upper 116000000  \/* BK4829 datasheet: worldwide band 760-1160 MHz (pas 1300 comme le BK4819) *\//' App/frequencies.c
perl -0pi -e 's/const freq_band_table_t BX4819_band1 = \{BX4819_band1_lower,  63000000\};/const freq_band_table_t BX4819_band1 = {BX4819_band1_lower, 58000000};  \/* BK4829: 18-580 MHz *\//' App/frequencies.c
perl -0pi -e 's/const freq_band_table_t BX4819_band2 = \{84000000, BX4819_band2_upper\};/const freq_band_table_t BX4819_band2 = {76000000, BX4819_band2_upper};  \/* BK4829: reouvre a 760 MHz *\//' App/frequencies.c

grep -q 'BX4819_band2_upper 116000000' App/frequencies.c || { echo "!! frequencies.c : plafond BK4829 1160 MHz"; exit 1; }
grep -q 'BX4819_band1_lower, 58000000' App/frequencies.c  || { echo "!! frequencies.c : bas de bande 580 MHz"; exit 1; }
grep -q '{76000000, BX4819_band2_upper}' App/frequencies.c || { echo "!! frequencies.c : haut de bande 760 MHz"; exit 1; }

# App/driver/bk4829.c : BK4819_SetFilterBandwidth() ignore totalement son
# parametre weak_no_different ("(void)weak_no_different;") -- sur ce driver,
# le preset WIDE (25 kHz) retrecit donc TOUJOURS le filtre RF sur un jugement
# signal-faible, meme pour un VFO normal FM/USB, contrairement a la V1
# (ENABLE_AM_FIX -> weak_no_different=true pour un VFO normal). Confirme sur
# l'air (2026-09-14, retour utilisateur) : le premier accrochage en FM Large
# sur V3 est nettement plus lent qu en RAW ou que sur la V1, plausible a la
# fois a cause de ceci et du cache AGC de RADIO_SetupAGC() (App/radio.c, non
# touche ici -- pas de bug reproductible identifie de ce cote la, juste un
# ecart d amorcage deja compense par le delai ajoute dans patch/sonde.c).
# Correctif minimal, additif : quand weak_no_different est vrai, le champ RF
# faible-signal (<11:9>) est elargi pour egaler le champ RF principal
# (<14:12>), desactivant le retrecissement automatique -- exactement la
# valeur 0x3628 deja utilisee a la main dans patch/sonde.c pour WIDE, ici
# generalisee au driver pour que TOUT appelant (pas seulement Sonde/SARSAT)
# puisse en beneficier. Quand weak_no_different est faux, val garde sa valeur
# stock d origine -- comportement inchange pour tout appelant qui ne l a pas
# encore demande.
perl -0pi -e 's/(            val = 0x3028;\n)/$1            if (weak_no_different) val = 0x3628;   \/* Sarsat_UV-K1-5_RP2040: champ RF faible-signal <11:9> elargi a 0b011 (= champ principal), pas de retrecissement automatique *\/\n/' App/driver/bk4829.c
perl -0pi -e 's/(            val = 0x4048;\n)/$1            if (weak_no_different) val = 0x4848;   \/* idem, champ faible-signal <11:9> elargi a 0b100 *\/\n/' App/driver/bk4829.c
perl -0pi -e 's/(            val = 0x2058;\n)/$1            if (weak_no_different) val = 0x2458;   \/* idem, champ faible-signal <11:9> elargi a 0b010 *\/\n/' App/driver/bk4829.c

grep -q 'if (weak_no_different) val = 0x3628;' App/driver/bk4829.c || { echo "!! bk4829.c : weak_no_different WIDE"; exit 1; }
grep -q 'if (weak_no_different) val = 0x4848;' App/driver/bk4829.c || { echo "!! bk4829.c : weak_no_different NARROW"; exit 1; }
grep -q 'if (weak_no_different) val = 0x2458;' App/driver/bk4829.c || { echo "!! bk4829.c : weak_no_different NARROWER"; exit 1; }

# App/radio.c : les deux seuls appels a BK4819_SetFilterBandwidth() pour un
# VFO FM/USB normal (hors AM, deja weak_no_different=true dans les deux cas)
# passaient "false" faute de ENABLE_AM_FIX defini pour ce build (V3 ne
# definit jamais cette macro, contrairement a V1) -- desormais que le driver
# ci-dessus honore vraiment ce parametre, les passer a "true" est ce qui fait
# reellement disparaitre le retrecissement pour un VFO normal, comme sur V1.
perl -0pi -e 's/(                #else\n)                    BK4819_SetFilterBandwidth\(Bandwidth, false\);\n(                #endif\n)/$1                    BK4819_SetFilterBandwidth(Bandwidth, true);   \/* Sarsat_UV-K1-5_RP2040: pas de retrecissement faible-signal pour un VFO FM\/USB normal, voir bk4829.c *\/\n$2/' App/radio.c
perl -0pi -e 's/(            #else\n)                BK4819_SetFilterBandwidth\(Bandwidth, false\);\n(            #endif\n)/$1                BK4819_SetFilterBandwidth(Bandwidth, true);   \/* idem *\/\n$2/' App/radio.c

grep -q 'BK4819_SetFilterBandwidth(Bandwidth, false)' App/radio.c && { echo "!! radio.c : un appel weak_no_different=false subsiste (attendu : 0)"; exit 1; }
[ "$(grep -c 'BK4819_SetFilterBandwidth(Bandwidth, true)' App/radio.c)" -ge 4 ] || { echo "!! radio.c : moins de 4 appels weak_no_different=true (2 AM deja presents + 2 patches)"; exit 1; }

# App/radio.c : RADIO_SetupAGC() met en cache son dernier (listeningAM,
# disable) dans un static et saute BK4819_InitAGC() -- une reecriture a chaud
# de toute la table de gain AGC -- si le meme couple est redemande. Ce cache
# est initialise a 0xFF sur ce firmware V3 : le tout premier appel FM/USB
# normal (non-AM, non-disable, donc couple 0) apres N IMPORTE QUEL etat
# different (AM, ou un TX/squelch-mute qui passe disable=true) tombe sur un
# cache-miss et declenche une vraie reecriture de table -- source plausible
# du temps de reaccrochage plus lent en FM Large observe sur V3 par rapport
# a la V1 (confirme sur l'air, 2026-09-14/15 -- meme mecanisme que celui deja
# neutralise pour l'ecran Sonde via un delai, voir patch/sonde.c). Sur la V1
# (uv-k5v1-kd8cec, radio.c d'origine, non touche par ce projet), ce meme
# static n'est PAS initialise explicitement et vaut donc 0 par defaut -- qui
# se trouve deja egal au premier couple FM/USB normal, donc
# BK4819_InitAGC() n'est quasiment jamais rappele en ecoute FM ordinaire la-
# bas. Sur confirmation explicite de l'utilisateur ("si la v1 est comme
# ceci, tu peux etendre le correctif") : aligne le seed V3 sur celui de la
# V1 (0 au lieu de 0xFF) pour que tout VFO classique beneficie du meme
# comportement, pas seulement l'ecran Sonde. Le delai de 300 ms dans
# patch/sonde.c est laisse en place (filet de securite bon marche pour le
# cas plus rare ou le cache a ete change par un AM/TX juste avant l'ouverture
# de l'ecran) plutot que retire.
perl -0pi -e 's/static uint8_t lastSettings = 0xFF;/static uint8_t lastSettings = 0;   \/* Sarsat_UV-K1-5_RP2040: aligne sur le comportement (non explicite) de la V1 -- voir commentaire ci-dessus *\//' App/radio.c

grep -q 'static uint8_t lastSettings = 0;' App/radio.c || { echo "!! radio.c : seed lastSettings AGC"; exit 1; }

# App/app/app.c #0 : include de sarsat.h + afgain.h + aprs.h + sonde.h
perl -0pi -e 's{#include "app/app.h"\n}{$&#ifdef ENABLE_SARSAT\n#include "app/sarsat.h"\n#include "app/afgain.h"\n#endif\n}' App/app/app.c
perl -0pi -e 's{#include "app/app.h"\n}{$&#ifdef ENABLE_APRS\n#include "app/aprs.h"\n#endif\n}' App/app/app.c
perl -0pi -e 's{#include "app/app.h"\n}{$&#ifdef ENABLE_SONDE\n#include "app/sonde.h"\n#endif\n}' App/app/app.c

# App/app/app.c #1 : APP_TimeSlice10ms(), juste après le service UART_PORT_UART
# -> AFGAIN_TimeSlice() en tache de fond (indep. de l'ecran, ~10 ms tick) pour
#    qu'un gain fixe reste actif meme hors ecran SARSAT ; puis ouvre l'ecran
#    SARSAT quand une trame 0x06C1 vient d'arriver (gSarsatShowRequest).
perl -0pi -e 's{        UART_HandleCommand\(UART_PORT_UART\);\n        // SCHEDULER_Enable\(\);\n    \}\n#endif\n}{$&\n#ifdef ENABLE_SARSAT\n    AFGAIN_TimeSlice();\n    if (gSarsatShowRequest) {\n        gSarsatShowRequest = false;\n        APP_RunSarsat();   /* auto-guarde ; se rearme si la radio est occupee */\n    }\n#endif\n}' App/app/app.c
# meme point d'ancrage : APRS_TimeSlice() (beacon auto, squelch rapide, gain
# fixe, icone GPS) + popup RX auto sur trame decodee (gAprsShowRequest).
perl -0pi -e 's{        UART_HandleCommand\(UART_PORT_UART\);\n        // SCHEDULER_Enable\(\);\n    \}\n#endif\n}{$&\n#ifdef ENABLE_APRS\n    APRS_TimeSlice();\n    if (gAprsShowRequest)\n        APP_RunAprs();     /* popup auto : ouvre en vue RX, auto-temporise/garde */\n#endif\n}' App/app/app.c
# meme point d'ancrage : ouvre l'ecran Sonde quand une trame 0x06E1 vient
# d'arriver (gSondeShowRequest) -- pas de TimeSlice() dedie (pas de beacon/
# squelch particulier a gerer hors ecran, contrairement a APRS).
perl -0pi -e 's{        UART_HandleCommand\(UART_PORT_UART\);\n        // SCHEDULER_Enable\(\);\n    \}\n#endif\n}{$&\n#ifdef ENABLE_SONDE\n    if (gSondeShowRequest) {\n        gSondeShowRequest = false;\n        APP_RunSonde();    /* auto-guarde ; se rearme si la radio est occupee */\n    }\n#endif\n}' App/app/app.c

# App/app/app.c #2 : inhibition de la veille batterie sur 144-148 MHz (sinon
# FUNCTION_POWER_SAVE coupe periodiquement le RX -> audio hache pour le C-Board).
perl -0pi -e 's/(            \|\| gScreenToDisplay != DISPLAY_MAIN\n)/$1#ifdef ENABLE_APRS\n            || APRS_KeepAwake()          \/\/ 144-148 MHz : garder le RX actif pour la C-Board\n#endif\n/' App/app/app.c

# App/app/app.c #3 : "light on frame" -- sur 144-148 MHz avec l'option activee,
# n'allumer le retroeclairage QUE sur un paquet decode, pas sur l'ouverture
# squelch normale entre les paquets.
perl -0pi -e 's/if \(gSetting_backlight_on_tx_rx & BACKLIGHT_ON_TR_RX\) \{\n        BACKLIGHT_TurnOn\(\);\n    \}/if ((gSetting_backlight_on_tx_rx & BACKLIGHT_ON_TR_RX)\n#ifdef ENABLE_APRS\n        \&\& !APRS_QuietBacklight()   \/\/ 144.8 APRS : lumiere seulement sur trame decodee\n#endif\n       ) {\n        BACKLIGHT_TurnOn();\n    }/' App/app/app.c

# App/app/uart.c : include
perl -0pi -e 's{#include "app/uart.h"\n}{$&#ifdef ENABLE_SARSAT\n#include "app/sarsat.h"\n#endif\n}' App/app/uart.c
perl -0pi -e 's{#include "app/uart.h"\n}{$&#ifdef ENABLE_APRS\n#include "app/aprs.h"\n#endif\n}' App/app/uart.c
perl -0pi -e 's{#include "app/uart.h"\n}{$&#ifdef ENABLE_SONDE\n#include "app/sonde.h"\n#endif\n}' App/app/uart.c

# App/app/uart.c : rendre SendReply() non-static (sarsat.c le reutilise)
perl -0pi -e 's/static void SendReply\(uint32_t Port, void \*pReply, uint16_t Size\)/void SendReply(uint32_t Port, void *pReply, uint16_t Size)/' App/app/uart.c

# App/app/uart.c : cases du switch (juste avant "    } // switch")
perl -0pi -e 's/\n    \} \/\/ switch/\n#ifdef ENABLE_SARSAT\n        case SARSAT_CMD_CLEAR:\n        case SARSAT_CMD_TEXT:\n        case SARSAT_CMD_LEVEL:\n        case SARSAT_CMD_HELLO:\n        case SARSAT_CMD_BEACON:\n            SARSAT_HandleUART(pUART_Command->Header.ID,\n                              pUART_Command->Buffer + sizeof(Header_t),\n                              pUART_Command->Header.Size);\n            break;\n#endif$&/' App/app/uart.c
perl -0pi -e 's/\n    \} \/\/ switch/\n#ifdef ENABLE_APRS\n        case APRS_CMD_RXTEXT:\n        case APRS_CMD_RXINFO:\n        case APRS_CMD_GPS:\n        case APRS_CMD_DIGI:\n            APRS_HandleUART(pUART_Command->Header.ID,\n                            pUART_Command->Buffer + sizeof(Header_t),\n                            pUART_Command->Header.Size);\n            break;\n#endif$&/' App/app/uart.c
perl -0pi -e 's/\n    \} \/\/ switch/\n#ifdef ENABLE_SONDE\n        case SONDE_CMD_CLEAR:\n        case SONDE_CMD_TEXT:\n            SONDE_HandleUART(pUART_Command->Header.ID,\n                             pUART_Command->Buffer + sizeof(Header_t),\n                             pUART_Command->Header.Size);\n            break;\n#endif$&/' App/app/uart.c

# App/settings.h : nouvelles entrées ACTION_OPT_SARSAT / ACTION_OPT_APRS /
# ACTION_OPT_SONDE (juste avant le sentinel LEN)
perl -0pi -e 's/    ACTION_OPT_LEN\n\};/#ifdef ENABLE_SARSAT\n    ACTION_OPT_SARSAT,\n#endif\n    ACTION_OPT_LEN\n};/' App/settings.h
perl -0pi -e 's/    ACTION_OPT_LEN\n\};/#ifdef ENABLE_APRS\n    ACTION_OPT_APRS,\n#endif\n    ACTION_OPT_LEN\n};/' App/settings.h
perl -0pi -e 's/    ACTION_OPT_LEN\n\};/#ifdef ENABLE_SONDE\n    ACTION_OPT_SONDE,\n#endif\n    ACTION_OPT_LEN\n};/' App/settings.h

# App/app/action.c : ouvrir l'écran SARSAT / APRS / Sonde depuis une touche
# assignable (F1/F2 court/long via le menu F4HWN standard "F1Shrt"/"F1Long"/
# "F2Shrt"/"F2Long" -> "SARSAT"/"APRS"/"SONDE"), en plus de l'auto-ouverture
# sur trame reçue. Pas de raccourci F+N dedie pour Sonde (contrairement a
# SARSAT/F+8 et APRS/F+5) : cette seule voie suffit et evite de sacrifier
# une autre combinaison de touche.
perl -0pi -e 's{#include "app/app.h"\n}{$&#ifdef ENABLE_SARSAT\n#include "app/sarsat.h"\n#endif\n}' App/app/action.c
perl -0pi -e 's{#include "app/app.h"\n}{$&#ifdef ENABLE_APRS\n#include "app/aprs.h"\n#endif\n}' App/app/action.c
perl -0pi -e 's{#include "app/app.h"\n}{$&#ifdef ENABLE_SONDE\n#include "app/sonde.h"\n#endif\n}' App/app/action.c
perl -0pi -e 's/\};\n\nstatic_assert\(ARRAY_SIZE\(action_opt_table\) == ACTION_OPT_LEN\);/#ifdef ENABLE_SARSAT\n    [ACTION_OPT_SARSAT] = &APP_RunSarsat,\n#endif\n$&/' App/app/action.c
perl -0pi -e 's/\};\n\nstatic_assert\(ARRAY_SIZE\(action_opt_table\) == ACTION_OPT_LEN\);/#ifdef ENABLE_APRS\n    [ACTION_OPT_APRS] = &APP_RunAprs,\n#endif\n$&/' App/app/action.c
perl -0pi -e 's/\};\n\nstatic_assert\(ARRAY_SIZE\(action_opt_table\) == ACTION_OPT_LEN\);/#ifdef ENABLE_SONDE\n    [ACTION_OPT_SONDE] = &APP_RunSonde,\n#endif\n$&/' App/app/action.c

# App/ui/menu.c : entrées "SARSAT" / "APRS" / "SONDE" dans la liste des
# fonctions assignables
perl -0pi -e 's/\};\n\nconst uint8_t gSubMenu_SIDEFUNCTIONS_size/#ifdef ENABLE_SARSAT\n    {"SARSAT",          ACTION_OPT_SARSAT},\n#endif\n$&/' App/ui/menu.c
perl -0pi -e 's/\};\n\nconst uint8_t gSubMenu_SIDEFUNCTIONS_size/#ifdef ENABLE_APRS\n    {"APRS",            ACTION_OPT_APRS},\n#endif\n$&/' App/ui/menu.c
perl -0pi -e 's/\};\n\nconst uint8_t gSubMenu_SIDEFUNCTIONS_size/#ifdef ENABLE_SONDE\n    {"SONDE",           ACTION_OPT_SONDE},\n#endif\n$&/' App/ui/menu.c

# App/driver/eeprom_compat.c : reserver de la place dans la queue non revendiquee
# du secteur "Settings" (0x00A170.. , juste apres "Settings Version" qui
# s'arrete a 0x00A170) : 8 o pour le reglage de gain AF C-Board (afgain.c),
# puis 56 o juste apres pour la config APRS (aprs.c, 7 pages EEPROM 8 o -- 40 o
# a l'origine, +16 o quand le champ "msg_to" du message report 121 a ete
# ajoute). Meme secteur physique que les reglages radio, donc protege du reset
# normal comme eux, efface seulement par "reset ALL".
perl -0pi -e 's/\n\};\n/\n    _MK_MAPPING(0x00A170, 0x00A170, 0x00A178),  \/\/ Sarsat_UV-K1-5_RP2040: gain AF C-Board (8 o)\n\};\n/' App/driver/eeprom_compat.c
perl -0pi -e 's/\n\};\n/\n    _MK_MAPPING(0x00A178, 0x00A178, 0x00A1B0),  \/\/ Sarsat_UV-K1-5_RP2040: config APRS (56 o)\n\};\n/' App/driver/eeprom_compat.c

# App/scheduler.h + .c : exposer millis10() (compteur 10 ms deja tenu par
# SysTick_Handler() dans gGlobalSysTickCounter, jusqu'ici prive a ce fichier) --
# meme nom/semantique que le millis10() du firmware V1 (KD8CEC), pour que
# aprs.c se porte sans aucun changement d'appel.
perl -0pi -e 's/#include "py32f0xx.h"\n/$&\nuint32_t millis10(void);\n/' App/scheduler.h
perl -0pi -e 's/static volatile uint32_t gGlobalSysTickCounter;\n/$&\nuint32_t millis10(void) { return gGlobalSysTickCounter; }\n/' App/scheduler.c

# App/driver/backlight.c : fondu de retroeclairage raccourci 16 -> 4 pas
# (~160 ms -> ~40 ms a la cadence normale du tick 10 ms). Remonte comme un
# "petit effet de fondu" visible a chaque changement d'ecran -- confirme par
# l'utilisateur present *partout* dans l'interface stock (menus, liste de
# canaux), pas specifique a SARSAT/APRS : reglage global du firmware F4HWN
# d'origine, pas conditionne par ENABLE_SARSAT/ENABLE_APRS. On raccourcit
# plutot que de le supprimer entierement (garde une transition, juste plus
# rapide qu'un flash brutal). Meme cadence de pas quelle que soit l'amplitude
# du saut de luminosite (fadeStep = ecart / diviseur).
perl -0pi -e 's/fadeStep = diff > 0 \? -\(-diff >> 4\) : diff >> 4;/fadeStep = diff > 0 ? -(-diff >> 2) : diff >> 2;/' App/driver/backlight.c

# App/driver/st7565.c : horloge SPI vers l'ecran LCD relevee DIV64 -> DIV16
# (48 MHz coeur -> ~750 kHz -> ~3 MHz). Meme rapport que le firmware V1
# (DP32G030, SPR=2 = FPCLK/16, meme horloge coeur 48 MHz -> ~3 MHz) sur le
# meme controleur ST7565 et la meme dalle 128x64 -- pas une valeur devinee,
# deja eprouvee sur ce projet via l'autre firmware. Un rafraichissement plein
# ecran (7 lignes x 128 o) passe d'environ 10-15 ms a ~2,5-4 ms, ce qui devrait
# reduire l'effet de "balayage" visible lors des changements d'ecran remonte
# par l'utilisateur (le fondu logiciel de retroeclairage et le "toujours
# allume" ayant deja ete ecartes comme cause).
perl -0pi -e 's/InitStruct\.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;/InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;/' App/driver/st7565.c

# App/settings.c : SETTINGS_ResetTxLock() (menu "reset verrouillage TX")
# decoupe la memoire canaux en 32 lots EGAUX d'octets -- correct seulement si
# MR_CHANNELS_MAX est un multiple de 32 (1024 l'est). Reecrit pour decouper
# par NOMBRE DE CANAUX (32/lot, dernier lot partiel gere), correct quel que
# soit MR_CHANNELS_MAX -- prealable indispensable avant de le plafonner a 170
# ci-dessous (170 n'est pas un multiple de 32 : l'ancien decoupage aurait
# corrompu/saute des canaux pres des limites de lot silencieusement).
perl -0pi -e 's/\Qvoid SETTINGS_ResetTxLock(void)
{
    \/\/ This is an expensive operation: full scan of all MR channels

    #define CHANNEL_SIZE               16
    #define TXLOCK_BYTE_OFFSET         12
    #define TXLOCK_BIT                 6
    #define SETTINGS_ResetTxLock_BATCH 32

    const uint32_t TotalBytes  = MR_CHANNELS_MAX * CHANNEL_SIZE;   \/\/ 1024 * 16 = 16 384
    const uint32_t BatchSize   = TotalBytes \/ SETTINGS_ResetTxLock_BATCH; \/\/ 16 384 \/ 32 = 512
    const uint32_t BatchChCnt  = BatchSize \/ CHANNEL_SIZE;         \/\/ 32 channels per batch

    uint8_t Buf[BatchSize];

    for (uint32_t batch = 0; batch < SETTINGS_ResetTxLock_BATCH; batch++)
    {
        uint32_t Offset = batch * BatchSize;

        PY25Q16_ReadBuffer(Offset, Buf, BatchSize);

        for (uint32_t ch = 0; ch < BatchChCnt; ch++)
        {
            uint32_t off = ch * CHANNEL_SIZE;
            Buf[off + TXLOCK_BYTE_OFFSET] |= (1 << TXLOCK_BIT);
        }

        PY25Q16_WriteBuffer(Offset, Buf, BatchSize, false);
    }

    RADIO_ConfigureChannel(0, VFO_CONFIGURE_RELOAD);
    RADIO_ConfigureChannel(1, VFO_CONFIGURE_RELOAD);

    #undef SETTINGS_ResetTxLock_BATCH
    #undef CHANNEL_SIZE
    #undef TXLOCK_BYTE_OFFSET
    #undef TXLOCK_BIT
}\E/void SETTINGS_ResetTxLock(void)
{
    \/\/ This is an expensive operation: full scan of all MR channels.
    \/\/ Batches by a fixed CHANNEL count rather than a fixed BYTE count split
    \/\/ into a fixed number of batches: the original math (TotalBytes \/ 32
    \/\/ batches) silently misaligned\/dropped channels whenever MR_CHANNELS_MAX
    \/\/ was not itself a multiple of 32 -- true for the stock 1024, no longer
    \/\/ true once Sarsat_UV-K1-5_RP2040 caps it at 170 for the dedicated APRS
    \/\/ channel. This form is correct for any MR_CHANNELS_MAX, including a
    \/\/ final partial batch.

    #define CHANNEL_SIZE       16
    #define TXLOCK_BYTE_OFFSET 12
    #define TXLOCK_BIT         6
    #define CHANNELS_PER_BATCH 32

    uint8_t Buf[CHANNELS_PER_BATCH * CHANNEL_SIZE];

    for (uint32_t first = 0; first < MR_CHANNELS_MAX; first += CHANNELS_PER_BATCH)
    {
        uint32_t Count = MR_CHANNELS_MAX - first;
        if (Count > CHANNELS_PER_BATCH) Count = CHANNELS_PER_BATCH;
        uint32_t Bytes  = Count * CHANNEL_SIZE;
        uint32_t Offset = first * CHANNEL_SIZE;

        PY25Q16_ReadBuffer(Offset, Buf, Bytes);

        for (uint32_t ch = 0; ch < Count; ch++)
        {
            uint32_t off = ch * CHANNEL_SIZE;
            Buf[off + TXLOCK_BYTE_OFFSET] |= (1 << TXLOCK_BIT);
        }

        PY25Q16_WriteBuffer(Offset, Buf, Bytes, false);
    }

    RADIO_ConfigureChannel(0, VFO_CONFIGURE_RELOAD);
    RADIO_ConfigureChannel(1, VFO_CONFIGURE_RELOAD);

    #undef CHANNELS_PER_BATCH
    #undef CHANNEL_SIZE
    #undef TXLOCK_BYTE_OFFSET
    #undef TXLOCK_BIT
}/' App/settings.c

# App/misc.h : plafond memoire 1024 -> 170 canaux, meme convention que le
# port V1 (canal APRS dedie = le dernier, pas perdu au milieu de centaines de
# slots inutilises). Contrairement au V1 (bornes d'enum litterales a decaler
# a la main), FREQ_CHANNEL_FIRST/LAST, NOAA_CHANNEL_FIRST/LAST et LAST_CHANNEL
# sont deja tous calcules A PARTIR de MR_CHANNELS_MAX ici -- un seul point de
# changement suffit. Aircopy (transfert radio-a-radio par audio) reste limite
# a ses 128 premiers canaux (AIRCOPY_NUM_BANKS = MR_CHANNELS_MAX / 128,
# division entiere -> 1 seule banque avec 170) : limitation documentee et
# acceptee, fonctionnalite annexe sans rapport avec SARSAT/APRS.
perl -0pi -e 's/#define MR_CHANNELS_MAX 1024/#define MR_CHANNELS_MAX 170 \/* Sarsat_UV-K1-5_RP2040: cap a 170, canal APRS dedie = le dernier (voir SETTINGS_ResetTxLock() plus haut, seul autre code sensible a MR_CHANNELS_MAX au-dela des bornes de cette enumeration). Aircopy reste limite a ses 128 premiers canaux (AIRCOPY_NUM_BANKS tronque). *\//' App/misc.h

# App/ui/status.c : icone GPS dans la barre de statut (losange 5x5 -- plein =
# fix verrouille, clignotant = module present mais en recherche, absent = pas
# de module GPS sur la C-Board). Meme convention que le port V1
# (patch/ui_status.c.diff) : APRS_GpsState() 0/1/2, meme bitmap.
perl -0pi -e 's/#include "ui\/status.h"\n/$&#ifdef ENABLE_APRS\n#include "app\/aprs.h"\n#endif\n/' App/ui/status.c
perl -0pi -e 's/(    x \+= sizeof\(gFontPttClassic\) \+ 3;\n#endif\n)/$1\n#ifdef ENABLE_APRS\n    \{\n        extern uint32_t millis10(void);\n        static const uint8_t BITMAP_GPS[5] = \{ 0x04, 0x0A, 0x15, 0x0A, 0x04 \};\n        const uint8_t gps = APRS_GpsState();\n        if (gps == 2 || (gps == 1 \&\& (millis10() % 80) < 40)) \{\n            memcpy(line + x, BITMAP_GPS, sizeof(BITMAP_GPS));\n            x1 = x + sizeof(BITMAP_GPS) + 1;\n        \}\n        if (gps) x += sizeof(BITMAP_GPS) + 3;\n    \}\n#endif\n/' App/ui/status.c

# App/app/main.c : F+5 ouvre directement l'ecran APRS (comme le V1), sur un
# appui court -- F+5 est mort dans ce build (case KEY_5 ne fait rien tant que
# ENABLE_NOAA et ENABLE_SPECTRUM sont tous deux desactives, ce qui est notre
# cas ici : ENABLE_SPECTRUM est coupe pour faire de la place a APRS, voir
# plus haut). Priorite NOAA/SPECTRUM > APRS conservee pour ne rien casser
# sur un autre preset qui activerait l'un des deux.
perl -0pi -e 's/#include "app\/generic.h"\n/$&#ifdef ENABLE_APRS\n#include "app\/aprs.h"\n#endif\n/' App/app/main.c
perl -0pi -e 's/(#elif defined\(ENABLE_SPECTRUM\)\n                APP_RunSpectrum\(\);\n                gRequestDisplayScreen = DISPLAY_MAIN;\n)#endif/$1#elif defined(ENABLE_APRS)\n                APP_RunAprs();\n#endif/' App/app/main.c
# F+8 ouvre directement l'ecran SARSAT, comme sur le V1 (App/app/main.c du
# V1 : "case KEY_8: #ifdef ENABLE_SARSAT APP_RunSarsat(); #else ... #endif").
# Contrairement a F+5, F+8 n'etait pas une combinaison morte ici -- appui
# court = ACTION_BackLightOnDemand(), appui long = bascule FrequencyReverse
# -- sacrifiees toutes les deux, sur demande explicite, pour la parite de
# touche avec le V1 (qui ne sacrifiait que l'inversion de frequence, seule
# chose que F+8 y faisait).
perl -0pi -e 's/#include "app\/generic.h"\n/$&#ifdef ENABLE_SARSAT\n#include "app\/sarsat.h"\n#endif\n/' App/app/main.c
perl -0pi -e 's/        case KEY_8:\n            if \(!beep\) \{\n                ACTION_BackLightOnDemand\(\); \n            \}\n            else \{\n                gTxVfo->FrequencyReverse = gTxVfo->FrequencyReverse == false;\n                gRequestSaveChannel = 1;\n            \}\n/        case KEY_8:\n#ifdef ENABLE_SARSAT\n            APP_RunSarsat();                 \/\/ F+8 : open the SARSAT screen (same as V1)\n            gRequestDisplayScreen = DISPLAY_MAIN;\n#else\n            if (!beep) {\n                ACTION_BackLightOnDemand(); \n            }\n            else {\n                gTxVfo->FrequencyReverse = gTxVfo->FrequencyReverse == false;\n                gRequestSaveChannel = 1;\n            }\n#endif\n/' App/app/main.c

# App/CMakeLists.txt : option + sources
perl -0pi -e 's/enable_feature\(ENABLE_UART_RW_BK_REGS\)\n/$&enable_feature(ENABLE_SARSAT\n    app\/sarsat.c\n    app\/afgain.c\n)\nenable_feature(ENABLE_APRS\n    app\/aprs.c\n    app\/ax25.c\n)\nenable_feature(ENABLE_SONDE\n    app\/sonde.c\n)\n/' App/CMakeLists.txt

# CMakePresets.json : defaut (off) dans chaque bloc de presets où ENABLE_UART_RW_BK_REGS
# apparaît (le fichier en a deux : un pour "configurePresets", un pour "buildPresets"
# ou similaire -- perl en mode /g pour couvrir les deux occurrences).
perl -0pi -e 's/( *)"ENABLE_UART_RW_BK_REGS": false,\n/$&$1"ENABLE_SARSAT": false,\n$1"ENABLE_APRS": false,\n$1"ENABLE_SONDE": false,\n/g' CMakePresets.json

echo "== controle"
grep -q 'app/sarsat.h'      App/app/app.c   || { echo "!! app.c : include sarsat"; exit 1; }
grep -q 'APP_RunSarsat'     App/app/app.c   || { echo "!! app.c : hook tick 10ms"; exit 1; }
grep -q 'AFGAIN_TimeSlice'  App/app/app.c   || { echo "!! app.c : hook AFGAIN_TimeSlice"; exit 1; }
grep -q 'app/aprs.h'        App/app/app.c   || { echo "!! app.c : include aprs"; exit 1; }
grep -q 'APRS_TimeSlice'    App/app/app.c   || { echo "!! app.c : hook APRS_TimeSlice"; exit 1; }
grep -q 'gAprsShowRequest'  App/app/app.c   || { echo "!! app.c : popup RX APRS"; exit 1; }
grep -q 'APRS_KeepAwake'    App/app/app.c   || { echo "!! app.c : inhibition veille"; exit 1; }
grep -q 'APRS_QuietBacklight' App/app/app.c || { echo "!! app.c : retroeclairage silencieux"; exit 1; }
grep -q 'app/sarsat.h'      App/app/uart.c  || { echo "!! uart.c : include sarsat"; exit 1; }
grep -q 'SARSAT_HandleUART' App/app/uart.c  || { echo "!! uart.c : dispatch sarsat"; exit 1; }
grep -q 'app/aprs.h'        App/app/uart.c  || { echo "!! uart.c : include aprs"; exit 1; }
grep -q 'APRS_HandleUART'   App/app/uart.c  || { echo "!! uart.c : dispatch aprs"; exit 1; }
grep -q 'APRS_CMD_DIGI'     App/app/uart.c  || { echo "!! uart.c : dispatch digipeat"; exit 1; }
grep -qE '^void SendReply\(' App/app/uart.c || { echo "!! uart.c : SendReply toujours static"; exit 1; }
grep -q 'ACTION_OPT_SARSAT' App/settings.h  || { echo "!! settings.h : enum sarsat"; exit 1; }
grep -q 'ACTION_OPT_APRS'   App/settings.h  || { echo "!! settings.h : enum aprs"; exit 1; }
grep -q 'app/sarsat.h'      App/app/action.c || { echo "!! action.c : include sarsat"; exit 1; }
grep -q 'ACTION_OPT_SARSAT.*APP_RunSarsat' App/app/action.c || { echo "!! action.c : table sarsat"; exit 1; }
grep -q 'app/aprs.h'        App/app/action.c || { echo "!! action.c : include aprs"; exit 1; }
grep -q 'ACTION_OPT_APRS.*APP_RunAprs' App/app/action.c || { echo "!! action.c : table aprs"; exit 1; }
grep -q 'ACTION_OPT_SARSAT' App/ui/menu.c   || { echo "!! menu.c : SIDEFUNCTIONS sarsat"; exit 1; }
grep -q 'ACTION_OPT_APRS'   App/ui/menu.c   || { echo "!! menu.c : SIDEFUNCTIONS aprs"; exit 1; }
grep -q 'ENABLE_SARSAT'     App/CMakeLists.txt || { echo "!! CMakeLists.txt : ENABLE_SARSAT"; exit 1; }
grep -q 'ENABLE_APRS'       App/CMakeLists.txt || { echo "!! CMakeLists.txt : ENABLE_APRS"; exit 1; }
grep -q 'ENABLE_SARSAT'     CMakePresets.json  || { echo "!! CMakePresets.json : ENABLE_SARSAT"; exit 1; }
grep -q 'ENABLE_APRS'       CMakePresets.json  || { echo "!! CMakePresets.json : ENABLE_APRS"; exit 1; }
grep -q 'app/afgain.c'      App/CMakeLists.txt || { echo "!! CMakeLists.txt : afgain.c"; exit 1; }
grep -q 'app/aprs.c'        App/CMakeLists.txt || { echo "!! CMakeLists.txt : aprs.c"; exit 1; }
grep -q 'app/ax25.c'        App/CMakeLists.txt || { echo "!! CMakeLists.txt : ax25.c"; exit 1; }
grep -q '0x00A170, 0x00A170, 0x00A178' App/driver/eeprom_compat.c || { echo "!! eeprom_compat.c : mapping gain AF"; exit 1; }
grep -q '0x00A178, 0x00A178, 0x00A1B0' App/driver/eeprom_compat.c || { echo "!! eeprom_compat.c : mapping config APRS"; exit 1; }
grep -qE '^uint32_t millis10\(void\);' App/scheduler.h || { echo "!! scheduler.h : millis10() declaration"; exit 1; }
grep -qE '^uint32_t millis10\(void\) \{ return gGlobalSysTickCounter; \}' App/scheduler.c || { echo "!! scheduler.c : millis10() definition"; exit 1; }
grep -q 'diff >> 2' App/driver/backlight.c || { echo "!! backlight.c : fondu raccourci"; exit 1; }
grep -q 'LL_SPI_BAUDRATEPRESCALER_DIV16' App/driver/st7565.c || { echo "!! st7565.c : horloge SPI ecran"; exit 1; }
grep -q 'CHANNELS_PER_BATCH' App/settings.c || { echo "!! settings.c : ResetTxLock par nombre de canaux"; exit 1; }
grep -qE '^#define MR_CHANNELS_MAX 170' App/misc.h || { echo "!! misc.h : plafond 170 canaux"; exit 1; }
grep -q 'APRS_TX_CHANNEL' App/ui/main.c || { echo "!! main.c : label APRS en mode MDF_CHANNEL"; exit 1; }
grep -q 'app/aprs.h'    App/ui/status.c || { echo "!! status.c : include aprs"; exit 1; }
grep -q 'APRS_GpsState' App/ui/status.c || { echo "!! status.c : icone GPS"; exit 1; }
grep -q 'MainHeaderIcom' App/ui/main.c  || { echo "!! main.c : ecran icom"; exit 1; }
grep -q 'app/aprs.h'  App/app/main.c    || { echo "!! app/main.c : include aprs"; exit 1; }
grep -q 'APP_RunAprs' App/app/main.c    || { echo "!! app/main.c : F+5 -> APRS"; exit 1; }
grep -q 'app/sarsat.h' App/app/main.c   || { echo "!! app/main.c : include sarsat"; exit 1; }
grep -q 'APP_RunSarsat' App/app/main.c  || { echo "!! app/main.c : F+8 -> SARSAT"; exit 1; }
grep -q 'app/sonde.h'   App/app/app.c   || { echo "!! app.c : include sonde"; exit 1; }
grep -q 'APP_RunSonde'  App/app/app.c   || { echo "!! app.c : hook tick 10ms sonde"; exit 1; }
grep -q 'app/sonde.h'   App/app/uart.c  || { echo "!! uart.c : include sonde"; exit 1; }
grep -q 'SONDE_HandleUART' App/app/uart.c || { echo "!! uart.c : dispatch sonde"; exit 1; }
grep -q 'ACTION_OPT_SONDE' App/settings.h || { echo "!! settings.h : enum sonde"; exit 1; }
grep -q 'app/sonde.h'   App/app/action.c || { echo "!! action.c : include sonde"; exit 1; }
grep -q 'ACTION_OPT_SONDE.*APP_RunSonde' App/app/action.c || { echo "!! action.c : table sonde"; exit 1; }
grep -q 'ACTION_OPT_SONDE' App/ui/menu.c || { echo "!! menu.c : SIDEFUNCTIONS sonde"; exit 1; }
grep -q 'ENABLE_SONDE'  App/CMakeLists.txt || { echo "!! CMakeLists.txt : ENABLE_SONDE"; exit 1; }
grep -q 'ENABLE_SONDE'  CMakePresets.json  || { echo "!! CMakePresets.json : ENABLE_SONDE"; exit 1; }
grep -q 'SONDE_ScreenOpen' App/app/sarsat.c || { echo "!! sarsat.c : fusion etat ecran sonde"; exit 1; }

echo "== build (preset=$PRESET, ENABLE_SARSAT=ON, ENABLE_APRS=ON, ENABLE_SONDE=ON, ENABLE_BYP_RAW_DEMODULATORS=ON)"
# ENABLE_BYP_RAW_DEMODULATORS : deja dans le code amont (App/driver/bk4829.c
# BK4819_EnterRaw(), cable dans RADIO_SetModulation() App/radio.c) mais eteint
# par defaut sur le preset Fusion. C'est le discriminateur FM a plat (REG_2B
# bits 10/9/8 debrayes, AF reste sur BK4819_AF_FM -- PAS un mode baseband/IQ
# qui battrait la porteuse) : le meme profil que le mode "DSC" ajoute au menu
# du firmware V1 (MODULATION_DISCRI), deja present ici sous le nom "RAW".
# L'ecran SARSAT le pose lui-meme en interne (sarsat.c) quel que soit ce
# reglage ; ceci l'expose en plus dans le menu Demodu de n'importe quel canal
# (utile notamment sur 144.8 pour l'APRS, une fois porte).

# ENABLE_SPECTRUM=OFF : le preset Fusion complet + SARSAT + APRS ne tient plus
# dans les 118 Ko flash / 16 Ko RAM du PY32F071 (depassement constate a la
# compilation, ~2,3 Ko de trop). L'analyseur de spectre est la fonctionnalite
# la plus lourde du preset et la moins liee au projet SARSAT/APRS -- meme
# arbitrage que celui deja fait sur le port V1 (SPECTRUM/FMRADIO/VOX/
# FLASHLIGHT coupes la-bas). Les autres extras F4HWN restent actifs.
#
# GAME / QRCODE / LOGO / LOGO_SAV / K5VIEWER / RXTX_LOG coupes (2026-09-09,
# demande utilisateur) : extras F4HWN sans rapport avec SARSAT/APRS, pour
# rendre de la marge flash (le build etait a 99,0 %). D'autres extras
# coupables au besoin : FMRADIO, AIRCOPY, VOX, FOXHUNT, BEAM, AUDIO_SCOPE,
# MENU_CAT, PMR/GMRS...
cmake --preset "$PRESET" -DENABLE_SARSAT=ON -DENABLE_APRS=ON -DENABLE_SONDE=ON -DENABLE_BYP_RAW_DEMODULATORS=ON \
    -DENABLE_SPECTRUM=OFF \
    -DENABLE_FEAT_F4HWN_GAME=OFF \
    -DENABLE_FEAT_F4HWN_QRCODE=OFF \
    -DENABLE_FEAT_F4HWN_LOGO=OFF \
    -DENABLE_FEAT_F4HWN_LOGO_SAV=OFF \
    -DENABLE_FEAT_F4HWN_K5VIEWER=OFF \
    -DENABLE_FEAT_F4HWN_RXTX_LOG=OFF \
    -DENABLE_FEAT_F4HWN_RXTX_LOG_K5VIEWER=OFF \
    -DVERSION_STRING_2="v${TAG#v}.SAR1" \
    > /tmp/uvk1k5v3-sarsat-cfg.log 2>&1 \
    || { echo "!! configure a échoué -- voir /tmp/uvk1k5v3-sarsat-cfg.log"; tail -40 /tmp/uvk1k5v3-sarsat-cfg.log; exit 1; }
cmake --build --preset "$PRESET" -j"$(nproc)"

BIN=$(find "build/$PRESET" -maxdepth 1 -name '*.bin' | head -n1)
[ -n "$BIN" ] || { echo "!! binaire introuvable dans build/$PRESET"; exit 1; }

OUT="$HERE/bin/uv-k1-k5v3-f4hwn-$(echo "$PRESET" | tr '[:upper:]' '[:lower:]')-sarsat-${TAG#v}-${COMMIT}.bin"
mkdir -p "$HERE/bin"
cp "$BIN" "$OUT"
echo "== OK -> ${OUT#"$HERE"/}"
sha256sum "$OUT"
