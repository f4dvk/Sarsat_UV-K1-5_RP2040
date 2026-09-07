# Câblage matériel — C-Board KD8CEC modifiée

> **BROUILLON — à relire, non encore vérifié sur matériel.**

Ce document décrit la variante utilisée par ce projet : la C-Board sert de
**coprocesseur d'écoute** (SARSAT RX + APRS RX/TX + GPS), elle n'injecte pas
d'audio micro et ne pilote pas le PTT (la balise APRS est générée par le
BK4819 de la radio, le SARSAT est RX seul).

**Alimentation : pas de régulateur.** Le RP2040 **et** le module GPS (GP-02
KIT) sont alimentés **directement par le 3,3 V que sort l'UV-K5 / UV-K1**
(pas de LM1117, pas de prise sur la batterie 8,4 V, pas de régulateur 5 V).
En contrepartie, **une modification des résistances internes de la radio
est nécessaire** : son rail 3,3 V est dimensionné pour ses propres besoins
et ne peut pas fournir tel quel les ~80-100 mA supplémentaires (RP2040
~25-50 mA + GPS jusqu'à ~50 mA à l'acquisition). Voir *Modification interne
de la radio* plus bas.

## Ce qui change par rapport au montage KD8CEC d'origine

| # | Modification | Pourquoi |
|---|---|---|
| 1 | **Ajouter un pont de polarisation** sur le nœud ADC : `R1 = 22 kΩ` du **3V3** au nœud, `R2 = 22 kΩ` du nœud à la **GND** (voir *impédance ADC* ci-dessous — **pas** 100 kΩ). | La *Basic Version* n'en a pas : l'entrée couplée flotte près d'un rail, l'audio est écrêté demi-onde par les 1N4148 et **rien ne décode** (log : `[lvl] dc` ~10-40 au lieu de ~2048, chaque salve lit `adc[0..4095]`). Le pont fixe le point de repos à **≈1,65 V** (mi-échelle de l'ADC 3,3 V). |
| 2 | **Remplacer C1 par `1 µF`** (au lieu de 0,1 µF). | Avec R1‖R2 = 11 kΩ, un 0,1 µF donne un passe-haut à ~145 Hz qui rogne le bas du spectre Manchester 400 bps du SARSAT ; `1 µF` → coupure ~14 Hz. Non-polarisé de préférence (film/MLCC) ; un tantale/électro. orienté « + » vers la radio convient aussi. |
| 3 | **Retirer la résistance série `20~100 Ω` et le haut-parleur externe.** | Ils servaient de charge factice / atténuateur pour KD8CEC. Ici la C-Board se contente de **surveiller** la ligne HP ; C1 bloque le DC et le pont fixe le niveau. Voir la mise en garde *audio radio* plus bas. |
| 4 | **Retirer le micro électret** (et son fil vers VCC). | Inutilisé : l'APRS TX passe par le générateur de tonalités du BK4819 de la radio, le SARSAT est RX seul. Le connecteur jack 2,5 mm reste — il porte la ligne série RX dont on a besoin. |
| 5 | **Retirer le régulateur (LM1117-5.0) et sa prise batterie 8,4 V.** | Le RP2040-Zero est alimenté directement sur son pad **3V3** par le 3,3 V de la radio ; son régulateur interne (XC6206) n'est **pas utilisé**. Le module GPS prend le même 3,3 V. Plus aucun 5 V sur la carte. |
| 6 | **Modifier les résistances internes de la radio** pour qu'elle puisse débiter ~80-100 mA de plus sur son 3,3 V. | Voir *Modification interne de la radio*. Sans ça : brown-out / reboot de la radio ou du RP2040 à l'acquisition GPS. |
| 7 | **Module GPS en 3,3 V.** | Le RP2040 n'est **pas tolérant 5 V** : un GPS alimenté en 5 V enverrait du 5 V sur GP5 → destruction. Alimenté en 3,3 V, son TX est compatible GP5. Le GP-02 KIT fonctionne en 3,3 V. |

## Schéma modifié

### Étage audio (le cœur de la modif)

```
        3V3 ────┬──────────────────┐
                │                  │
               R1 22k             D1 1N4148   (cathode vers 3V3)
                │                  │
  RADIO         │                  │
  HP/SPK ──┤├───┴────── nœud ADC ──┼──┬── GP26  (ADC0, échantillonné)
          C1 1µF        │          │  ├── GP27  ┐ pontés au nœud
                       R2 22k      │  └── GP28  ┘ (comme la C-Board d'origine)
                        │         D2 1N4148   (anode vers GND)
                        │          │
        GND ────────────┴──────────┘

  (résistance série 20~100 Ω et HP externe : RETIRÉS)
```

- Le pont **R1/R2** fixe le nœud à **3V3 / 2 ≈ 1,65 V**.
- **C1** bloque le DC de la sortie radio. `1 µF` + (R1‖R2 = 11 kΩ) → passe-haut
  à ~14 Hz (avec 0,1 µF ce serait ~145 Hz — trop haut pour le Manchester
  400 bps du SARSAT).
- **D1/D2** écrêtent le nœud à ~-0,6 V … ~3,9 V (protection ADC). En
  fonctionnement normal (±0,5 V autour de 1,65 V) ils ne conduisent pas.

### Série + GPS + alimentation

```
  RADIO série TX (jack 3,5 bague) ───────────► GP1   (UART0 RX)
  RADIO série RX (jack 2,5)       ◄─────────── GP0   (UART0 TX)
  RADIO MIC / PTT : micro électret RETIRÉ ; PTT non utilisé par le firmware

  RADIO 3,3 V (après mod résistances internes) ─┬─ RP2040-Zero pad 3V3  (PAS le pad 5V)
  RADIO GND ────────────────────────────────────┤   (régulateur interne XC6206 non utilisé)
                                                ├─ module GPS VCC  (3,3 V)
                                                └─ pont R1/R2 + D1/D2  (étage audio)

  (LM1117-5.0, C_in, C_out, prise batterie 8,4 V : RETIRÉS)

  MODULE GPS (GP-02 KIT) : VCC 3,3 V, GND, TX ──► GP5  (UART1 RX)   [RX du GPS non connecté]
```

**GP26 / GP27 / GP28** sont pontés ensemble sur le nœud (comme la C-Board
d'origine). Le firmware n'échantillonne que l'ADC0 mais met les 3 pads en mode
analogique (`CFG_ADC_SHORTED_MASK = 0x07` dans `decoder_config.h`).

La masse est commune radio ↔ C-Board via les jacks ; le 3,3 V d'alimentation
prend un fil supplémentaire depuis le point de la radio choisi ci-dessous.

## Impédance d'entrée de l'ADC — pourquoi R1/R2 = 22 kΩ et pas 100 kΩ

L'ADC du RP2040 a une **impédance d'entrée dynamique ≈ 100 kΩ** : pendant la
phase d'échantillonnage il tire un courant sur la source pour charger sa
capacité S/H. Si l'impédance de source (ici **R1‖R2**) est trop élevée
(> ~50 kΩ), cette charge ne se fait pas complètement — la mesure est faussée
(chute de tension) et l'erreur dépend de l'échantillon précédent → **distorsion
dépendante du signal**, pas un simple offset.

Donc : garder **R1‖R2 nettement sous 50 kΩ**.

| R1 = R2 | R1‖R2 (source ADC) | passe-haut avec C1 = 1 µF | verdict |
|---|---|---|---|
| 100 kΩ | 50 kΩ | 3 Hz | **à éviter** (limite haute de l'ADC) |
| 33 kΩ | 16,5 kΩ | 10 Hz | OK |
| **22 kΩ** | **11 kΩ** | **14 Hz** | **recommandé** |
| 15 kΩ | 7,5 kΩ | 21 Hz | OK (charge la sortie radio un peu plus) |

Consommation permanente du pont : 3,3 V / 44 kΩ ≈ **75 µA** — négligeable.
La sortie AF de la radio pilote 11 kΩ sans problème.

## Valeurs des composants

| réf | valeur | remarque |
|---|---|---|
| C1 | **1 µF** | condo de liaison. Non-polarisé de préférence (film / MLCC) ; un tantale ou électro. « + » vers la radio (côté haut, ~V_HP), « − » vers le nœud (~1,65 V) convient. |
| R1, R2 | **22 kΩ** | pont de polarisation 3V3 / GND — voir *impédance ADC* ci-dessus. |
| D1, D2 | 1N4148 | écrêteurs, conservés. |
| ~~LM1117-5.0, C_in, C_out~~ | — | **retirés** : plus de régulateur, alim directe 3,3 V depuis la radio. |
| C_dec | 10-100 µF + 100 nF | découplage au plus près du pad 3V3 de la RP2040-Zero (le régulateur retiré ne le fait plus). |

## Modification interne de la radio (alim 3,3 V)

Le 3,3 V que la radio met à disposition (broche d'accessoire / point interne)
passe par une **résistance série** (protection / limitation de courant) trop
élevée pour tirer les ~80-100 mA du RP2040 + GPS : la tension s'effondre à
l'acquisition GPS → brown-out de la radio ou du RP2040.

**Modification** : réduire (ou ponter) cette résistance série sur le rail
3,3 V. La valeur exacte et l'emplacement dépendent de la révision de la carte
radio — **à relever sur le PCB avant de souder** (typiquement une résistance
CMS de quelques ohms à quelques dizaines d'ohms sur le chemin
régulateur 3,3 V → point d'accessoire). Objectif : chute < ~0,1 V sous
~100 mA, soit une série ≤ ~1 Ω, ou pontée.

> ⚠️ Non encore relevé sur une carte réelle pour ce projet — compléter ici
> avec la référence exacte (ex. `R__` sur la carte UV-K5 rév. __) une fois
> la modif faite et validée.

Alternative si on ne veut pas ouvrir la radio : garder l'ancien montage
LM1117 depuis la batterie 8,4 V (voir historique git de ce fichier).

## Réglage / vérification

1. Sans signal, la console USB du RP2040 doit logger `[lvl] dc` **proche de
   2048** (mi-échelle 12 bits). Si `dc` ~10-40 : le pont R1/R2 n'est pas
   connecté ou une valeur est fausse.
2. **Niveau audio** — se règle une fois depuis la radio, pas au potentiomètre :
   - potentiomètre de volume **au maximum**, et on n'y touche plus ;
   - radio accordée sur une **fréquence UHF** avec le souffle FM audible
     (squelch ouvert) ;
   - écran **SARSAT** (**F+8**) → touche **`5`** (vue niveau) → **HAUT / BAS**
     jusqu'à ce que le **souffle** place la barre à mi-échelle. Le curseur agit
     sur le gain AF C-Board (BK4819 REG_48 + gain DAC), sauvé en EEPROM,
     indépendamment du potentiomètre.
   - Contre-vérification côté RP2040 : commande `m` (mètre) → souffle
     `rms ~2000-3500`, `clip 0 %`. Une salve de balise lit alors plus bas et
     décode.
3. GPS : sur `/dev/ttyACM0`, `[aprs] … mod=…` puis, une fois le fix acquis,
   `0x06D5` envoyé toutes les ~3 s et le symbole GPS de la barre haute passe de
   clignotant à fixe.

## Mises en garde

- **Audio de la radio.** Sur l'UV-K5, insérer une fiche dans le jack HP 3,5 mm
  **coupe le haut-parleur interne**. Sans HP externe, la radio devient
  **silencieuse**. Deux options :
  - accepter l'écoute silencieuse (cohérent avec un moniteur SARSAT/APRS, et
    avec l'option « Light on frame ») ;
  - **ou** prendre l'audio directement sur les cosses du HP interne (à
    l'intérieur de la radio) au lieu du jack — le HP interne continue alors de
    fonctionner. Dans ce cas, C1 se branche sur la cosse « + » du HP.
- **Tolérance 5 V.** Rien du RP2040 (GP0/GP1/GP5/GP26…) n'est tolérant 5 V —
  sans objet ici puisqu'il n'y a plus aucun 5 V, mais à garder en tête si on
  ajoute un accessoire.
- **Brown-out.** C'est **le** point de vigilance de ce montage : alim 3,3 V
  directe depuis la radio, sans régulateur tampon. La modif interne des
  résistances (ci-dessus) est **obligatoire** ; vérifier le 3,3 V à la sonde
  pendant une acquisition GPS (creux < 3,0 V = insuffisant). Découplage
  généreux (`C_dec`) au plus près du pad 3V3. Masse commune radio ↔ C-Board
  (via les jacks).
- **Autonomie.** Le RP2040 + GPS tirent en permanence sur la batterie de la
  radio (~50-100 mA) — à prendre en compte pour la durée d'écoute.

## Brochage série (rappel `docs/protocol.md`)

| RP2040 | ↔ | radio | jack |
|---|---|---|---|
| GP0 = UART0 TX | → | série RX | 2,5 mm |
| GP1 = UART0 RX | ← | série TX | 3,5 mm (bague) |
| GP5 = UART1 RX | ← | GPS TX | — |
| GP26 (+27/28) = ADC0 | ← | audio HP (via C1 + pont + clamp) | 3,5 mm |

38400 8N1 pour le lien radio, 9600 8N1 NMEA pour le GPS.
