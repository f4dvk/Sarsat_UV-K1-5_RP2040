/* See sstv_tx.h. K1/K5 V3 (PY32F071 + BK4829) only.
 *
 * branch SSTV_SSDV: replaces the abandoned IMGFSK/SSDV image link (see
 * decoder_config.h and git history) after its raw-FSK approach hit a wall
 * that survived every fix tried at the framing/register level (see
 * imgfsk_sync.c's history in git). SSTV is a much older, simpler protocol:
 * a single continuously-swept AUDIO TONE whose instantaneous frequency
 * encodes pixel luminance, with a self-resynchronising sync pulse before
 * each line -- no chip-level FIFO/correlator/scrambler involved at all, and
 * the RX side (RP2040) only needs an instantaneous-frequency estimate, not
 * bit-exact framing. TX stays internal to the BK4819/29, using its Tone1
 * generator (REG_70/71) -- already proven able to hold and sweep an
 * arbitrary audio frequency for CTCSS/DTMF/roger-beep use, repurposed here
 * for continuous per-pixel frequency updates instead of fixed tones.
 *
 * Register recipe for driving Tone1, since this repo doesn't vendor the
 * upstream BK4819/29 driver source (build.sh clones it fresh each build --
 * see that driver's own BK4819_PrepareToPlayTone()/BK4819_PlayTone()/
 * BK4819_EnterTxMute()/BK4819_ExitTxMute(), cross-checked against the
 * sibling GOGUFW-UV-K1-Messenger project's copy of the same driver, which
 * already proven this exact sequence for its own beep/DTMF features):
 *   REG_47 = 0x6342 : AF path = "BEEP" mode (0x6042 | (3 << 8), 3 =
 *                     BK4819_AF_BEEP in that driver's enum).
 *   REG_50 = 0xBB18 / 0x3B18 : TX mute / un-mute (BK4819_EnterTxMute() /
 *                     BK4819_ExitTxMute()'s own values).
 *   REG_70 = 0xE000 : Tone1 enable (bit 15) + tuning gain 96 (bits 14:8) --
 *                     BK4819_PrepareToPlayTone()'s own "normal" gain preset.
 *   REG_30 = 0x0302 : AF DAC (bit 9) + discriminator mode (bit 8) + TX DSP
 *                     (bit 1) enabled -- same three bits
 *                     BK4819_PrepareToPlayTone() sets, written 0 then this
 *                     value (same clear-then-set pattern that function and
 *                     BK4819_TurnsOffTones_TurnsOnRX() both use).
 *   REG_71 = freq(Hz) * 10.32444, XTAL 26 MHz (same formula as imgfsk_tx.c's
 *            REG_72, per the BK4819(V3) Application Note -- REG_71 is the
 *            analogous control word for Tone1 instead of Tone2/FSK),
 *            rewritten continuously while TX stays keyed to sweep the tone.
 *
 * Unlike IMGFSK, no FIFO, no preamble/sync-word registers, no scramble bit,
 * no packet length register -- the whole image is one continuous held tone,
 * timed entirely in software (SYSTICK_DelayUs(), already used elsewhere in
 * the upstream driver for sub-millisecond timing, e.g. I2C/BK4819 bit-bang
 * delays). ~110 s total for one Scottie 1 frame: a long blocking call --
 * no MCU watchdog to feed (imgfsk_tx.c's own multi-second blocking waits
 * already established that), BUT see the F4HWN auto-sleep timer note below,
 * which very much IS a real risk over a hold this long.
 *
 * ⚠️ (2026-09-26, retour terrain : coupures intermittentes en TX, QSSTV ne
 * decode pas) -- meme cause deja trouvee et corrigee pour IMGFSK RX (voir
 * git log, imgfsk_rx.c) : App/app/app.c (F4HWN, ENABLE_FEAT_F4HWN_SLEEP,
 * actif dans le preset "Fusion") a un minuteur de mise en veille ("Set
 * Off", gSetting_set_off, 1 min par defaut -- misc.c) qui, a expiration,
 * force gPowerSave_10ms=1 SANS CONDITION, declenchant BK4819_Sleep() --
 * coupant reellement l'audio/la porteuse, independamment de tout ce que ce
 * fichier configure. La ou IMGFSK_TimeSlice() etait rappelee a chaque tick
 * par la boucle principale (RX non bloquant) pour reecrire le compte a
 * rebours avant qu'il n'atteigne 0, SSTV_SendScottie1() est un SEUL appel
 * bloquant de ~110 s qui ne rend jamais la main a cette boucle -- il faut
 * donc reecrire ce compte a rebours nous-memes, directement dans la
 * primitive appelee le plus souvent (sstv_tone(), une fois toutes les
 * 432 us pendant le balayage), pour ne jamais le laisser atteindre 0.
 *
 * Protocol timing (VIS header + Scottie 1 scan-line structure) from the
 * SSTV Handbook (Martin Bruchanov OK2MNM, sstv-handbook.com, chapter 4) --
 * see this session's chat log for the exact tables cited.
 *
 * ⚠️ (2026-09-26) branch SSTV_SSDV, TX temps reel. L'image de test embarquee
 * ici (sstv_test_image.h, RGB332 80x64) plafonnait la qualite bien en
 * dessous de ce que la resolution native des modes SSTV permettrait --
 * cette radio n'a que ~118 Ko de flash au total (deja a 98%), la ou le
 * RP2040 en a 2 Mo. Le TX reste materiel (seul le BK4819/29 sait generer
 * le ton FM en continu), mais l'IMAGE elle-meme vient maintenant du
 * RP2040, en direct, pixel par pixel :
 *  - CMD_SSTV_START (0x06E2, voir rp2040/src/decoder_config.h pour la
 *    valeur canonique et la conception complete) est envoye au RP2040 des
 *    le tout debut de SSTV_SendScottie1(), ~1.2 s avant que le premier
 *    pixel ne soit reellement necessaire (le temps du VIS).
 *  - Le RP2040 repond en envoyant 245760 octets de luminance BRUTS (sans
 *    le framing Quansheng habituel) sur la meme liaison UART.
 *  - Cote radio, sstv_stream_next_byte() lit ces octets directement dans
 *    le tampon circulaire DMA de reception (App/driver/uart.c, deja
 *    existant pour tout l'UART, 256 o -- deux accesseurs ajoutes par
 *    build.sh : SSTV_UartDmaWritePos()/SSTV_UartDmaPeek()) plutot que de
 *    passer par le parseur de trames, pour ne pas perdre de temps CPU
 *    pendant la fenetre critique du balayage (432 us/pixel).
 * Debit UART porte a 230400 (build.sh, App/driver/uart.c) pour ce debit --
 * voir le commentaire de CFG_RADIO_UART_BAUD cote RP2040 pour le calcul de
 * marge (dimensionne pour PD-120, le mode SSTV le plus exigeant connu).
 *
 * ⚠️ (2026-09-26, retour terrain : decodage immediat avec un autre systeme
 * TX, mais aucun decodage ou une detection mauvaise avec le notre, meme
 * apres tous les correctifs de cadencement RP2040<->radio ci-dessus) --
 * AUTRE cause racine trouvee, cote radio cette fois, en lisant le vrai
 * depot amont (App/driver/systick.c, App/scheduler.c) : SysTick tourne a
 * la PRIORITE NVIC LA PLUS HAUTE possible (0) avec une interruption
 * periodique de 10 ms dont le gestionnaire (scheduler.c, minuteur TOT,
 * veille, CTCSS, dual watch...) fait un travail non trivial -- largement
 * assez pour preempter, une fois toutes les ~23 pixels (10 ms / 432 us),
 * n'importe quel SYSTICK_DelayUs() ou ecriture BK4819_WriteRegister() en
 * cours (protocole bit-bang, plusieurs micro-etapes). SSTV_CALL_OVERHEAD_US
 * ne corrige qu'une moyenne FIXE ; il ne peut rien contre cette gigue
 * PERIODIQUE, qui produit exactement le motif de bandes/franges observe.
 * Corrige en sortant le balayage image (pas le VIS, moins critique) de
 * SYSTICK_DelayUs() pour le faire piloter par TIM14 (Puya PY32F071) --
 * seul timer materiel totalement libre de ce firmware (seuls TIM6 --
 * voice.c, DAC -- et TIM7 -- backlight.c, PWM -- le sont), a la MEME
 * priorite que SysTick : les deux ne peuvent alors plus se preempter, la
 * gigue residuelle est bornee a la duree du gestionnaire SysTick lui-meme
 * (quelques us) au lieu de la duree d'un bit-bang interrompu en plein
 * milieu. Voir le bloc "TIM14" plus bas pour le detail du pipeline. */
#include <stdint.h>       /* app/uart.h expects this already included */
#include "app/sstv_tx.h"
#include "app/uart.h"     /* UART_PORT_UART, pour SendReply() */

#include "driver/bk4819.h"
#include "driver/eeprom.h"    /* EEPROM_ReadBuffer()/WriteBuffer() -- image sel persistence */
#include "driver/keyboard.h"  /* KEYBOARD_Poll()/KEY_EXIT -- abort a transmission in progress */
#include "driver/st7565.h"    /* ST7565_BlitFullScreen(), SSTV_ToggleImage()'s confirmation */
#include "driver/systick.h"
#include "driver/system.h"
#include "radio.h"
#include "ui/helper.h"        /* UI_DisplayClear()/UI_PrintStringSmallNormal(), idem */

#include "py32f0xx.h"        /* TIM14, TIM14_IRQn, NVIC_*, registre CMSIS */
#include "py32f071_ll_bus.h" /* LL_APB1_GRP2_*(TIM14) */
#include "py32f071_ll_tim.h" /* LL_TIM_* */

extern uint32_t SystemCoreClock;

/* Meme numero que rp2040/src/decoder_config.h -- pas de header partage
 * entre les deux firmwares (meme convention que CMD_SARSAT_HELLO et les
 * autres commandes de ce projet, chacune redefinie independamment des deux
 * cotes). */
#define CMD_SSTV_START            0x06E2u
#define CMD_SSTV_STATUS           0x06E3u
#define CMD_SSTV_IMAGE_SELECT     0x06E6u
#define CMD_SSTV_STOP             0x06E7u
#define SSTV_STREAM_MODE_SCOTTIE1 0u
#define SSTV_STREAM_MODE_MARTIN1  1u
#define SSTV_STREAM_MODE_PD90     2u
#define SSTV_STREAM_MODE_PD120    3u

extern void SendReply(uint32_t Port, void *pReply, uint16_t Size);
extern uint16_t SSTV_UartDmaWritePos(void);
extern uint8_t  SSTV_UartDmaPeek(uint16_t idx);

static void sstv_set_tone_hz(uint32_t freq_hz)
{
    /* freq * 10.32444, rounded -- same formula/rounding as imgfsk_tx.c's
     * REG_72 derivation, just applied to REG_71 (Tone1) instead. */
    BK4819_WriteRegister(BK4819_REG_71,
                          (uint16_t)((freq_hz * 103244u + 5000u) / 10000u));
}

#ifdef ENABLE_FEAT_F4HWN_SLEEP
extern uint16_t gSleepModeCountdown_500ms;
extern uint8_t  gSetting_set_off;
#endif

/* ⚠️ (2026-09-26, retour terrain : une transmission complete chronometree a
 * 2 min 12 = 132 s au lieu des ~110.5 s calcules, ecart ~21.5 s) -- ce
 * fichier ne comptait que le SYSTICK_DelayUs(duration_us) explicite de
 * chaque appel, jamais le cout REEL de BK4819_WriteRegister() lui-meme
 * (protocole bit-bang avec plusieurs SYSTICK_DelayUs(1) internes, chacun
 * probablement plus proche de quelques us que d'exactement 1 une fois le
 * cout d'appel de SYSTICK_DelayUs() inclus). Sur ~246800 appels a
 * sstv_tone() dans une image Scottie 1 complete (dont 245760 pixels a
 * 432 us -- l'ecrasante majorite), un cout FIXE d'environ 21.5s/246800 =
 * ~87 us par appel explique a lui seul tout l'ecart mesure (les quelques
 * appels plus longs -- VIS, sync, gap -- pesent trop peu dans le total
 * pour changer ce calcul). Compense en le retranchant de chaque duree
 * demandee ; valeur empirique.
 *
 * ⚠️ (2026-09-26) CONFIRME sur le terrain : apres ce correctif (et malgre
 * l'ajout depuis du deballage RGB332 dans sstv_pixel(), lui-meme ensuite
 * retire du chemin critique via table de correspondance, voir plus bas),
 * une transmission complete chronometree a ~1 min 50 = 110 s, quasiment
 * exactement les ~110.5 s calcules. Cadence correcte, valeur laissee
 * telle quelle. */
#define SSTV_CALL_OVERHEAD_US 87u

/* ⚠️ (2026-10-03, retour utilisateur : "remettre l'action de la touche exit
 * lors de l'envoi SSTV, comme F4HWN") -- SSTV_Send*() est un seul appel
 * bloquant de ~110 s (VIS + balayage), qui ne rend jamais la main a la
 * boucle principale pendant tout ce temps : rien ne pouvait auparavant
 * interrompre une emission en cours une fois lancee. KEYBOARD_Poll() n'est
 * appele qu'ICI, dans sstv_tone()/sstv_hw_tone() -- une fois par tonalite
 * VIS/sync/porche/gap (quelques dizaines d'appels sur toute une image), PAS
 * a chaque pixel -- pour ne pas reintroduire le genre de gigue de cadencement
 * deja chasse une fois dans ce fichier (voir le commentaire TIM14 plus bas) :
 * sstv_hw_tone_pixel() (appele, lui, une fois par pixel) ne relit que le
 * drapeau deja mis en cache (s_tx_abort), sans repoller le clavier. Latence
 * de detection resultante : au pire la duree d'une ligne complete (quelques
 * centaines de ms), largement suffisant pour une touche EXIT. Une fois
 * detecte, s_tx_abort reste vrai pour le reste de cette transmission : tout
 * appel ulterieur a ces trois fonctions devient un no-op immediat, ce qui
 * fait remonter tres vite (sans attendre la fin naturelle de l'image) le
 * controle a sstv_send_common(), dont le demontage (mute, PTT/PA off,
 * RADIO_SetupRegisters) s'execute de toute facon sans condition -- pas de
 * chemin special a ajouter pour l'abandon.
 *
 * ⚠️ (2026-10-03, retour terrain : "meme apres exit, il reste sur le
 * streaming" + log confirmant "stream done" n'arrivant qu'apres le delai
 * COMPLET normal de l'image, alors que la radio avait annule des le debut)
 * -- l'hypothese initiale ("le RP2040 continue inoffensivement en arriere-
 * plan, aucune consequence") etait FAUSSE : sstv_stream_scottie1()/martin1/
 * pd90/pd120 (RP2040, main.c) sont des boucles bloquantes cadencees en
 * temps reel qui ignoraient totalement cet abandon et continuaient a
 * ecrire des octets d'image BRUTS (non trames) sur la MEME liaison UART que
 * la radio, elle, avait deja rebasculee en fonctionnement normal (trames
 * AB CD...DC BA) -- source plausible du "repasse en emission par
 * intermittence" remonte par ailleurs. CMD_SSTV_STOP (decoder_config.h)
 * previent maintenant le RP2040 explicitement, des que EXIT est detecte --
 * voir son envoi juste en dessous. */
static volatile bool s_tx_abort;

static bool sstv_tx_exit_pressed(void)
{
    if (!s_tx_abort && KEYBOARD_Poll() == KEY_EXIT) {
        s_tx_abort = true;
        uint8_t buf[4] = {
            (uint8_t)(CMD_SSTV_STOP & 0xFFu), (uint8_t)(CMD_SSTV_STOP >> 8),
            0, 0,
        };
        SendReply(UART_PORT_UART, buf, sizeof buf);
    }
    return s_tx_abort;
}

/* Hold a tone for an exact duration -- the one primitive every VIS/sync/
 * scan-line segment below is built from. */
static void sstv_tone(uint32_t freq_hz, uint32_t duration_us)
{
    if (sstv_tx_exit_pressed())
        return;
#ifdef ENABLE_FEAT_F4HWN_SLEEP
    /* Keep the F4HWN auto-sleep timer from ever reaching 0 during our long
     * blocking hold -- see this file's header comment. Cheap (one write),
     * called far more often (every 432 us during the scan) than it needs
     * to be, but simplest/safest: no separate periodic-tick bookkeeping to
     * get wrong. */
    gSleepModeCountdown_500ms = (uint16_t)(gSetting_set_off * 120u);
#endif
    sstv_set_tone_hz(freq_hz);
    uint32_t d = (duration_us > SSTV_CALL_OVERHEAD_US)
                     ? duration_us - SSTV_CALL_OVERHEAD_US : 0u;
    SYSTICK_DelayUs(d);
}

/* ---- TIM14 : horloge materielle du balayage image ------------------------
 * Voir le commentaire d'en-tete du fichier pour la cause du probleme
 * (gigue de l'IRQ SysTick, prioritaire, toutes les ~10 ms). Utilise
 * uniquement par sstv_scan_channel()/sstv_send_scottie1_image() -- le VIS
 * reste sur sstv_tone()/SYSTICK_DelayUs() ci-dessus, sa duree (secondes)
 * rend une gigue de quelques us hors de propos.
 *
 * ⚠️ (2026-09-27) HISTORIQUE -- REFONTE. La toute premiere version pilotait
 * TIM14 avec un pipeline a UN SEUL CRAN : le thread devait attendre que
 * l'ISR consomme la valeur en cours avant de preparer la suivante, ce qui
 * suppose que le thread finit TOUJOURS son travail (lire un octet du flux
 * RP2040, calculer la frequence) largement avant l'echeance de l'intervalle
 * courant. Deux correctifs bases sur cette hypothese (reordonner wait/
 * fetch ; distinguer les retards sync/porche vs pixel) n'ont EU AUCUN EFFET
 * MESURABLE sur le terrain -- ni sur le compteur de diagnostic, ni sur la
 * forme d'onde reelle enregistree -- et le nouveau compteur separe a
 * revele que le retard est domine par le balayage PIXEL (12464/12682 pour
 * PD90, 34884/35721 pour Martin1 -- PIRE sur Martin1, qui calcule pourtant
 * MOINS par octet que PD90 : ni "specifique a PD90", ni "trop de calcul
 * par pixel"). Le seul facteur qui colle : Martin1 a un budget par pixel
 * plus court (457.6 us) que PD90 (532 us) et va plus mal -- symptome d'un
 * cout a peu pres FIXE quelque part dans la chaine RP2040->UART->radio
 * (granularite de sleep_us(), latence USB/CDC, IRQ SysTick qui vole
 * quelques us au thread...) qui pese proportionnellement plus lourd sur un
 * budget court, quel que soit le mode -- Scottie1 (432 us, budget encore
 * plus court) en souffre tres probablement aussi, juste masque par son
 * rendu RGB direct, plus tolerant au bruit qu'un flux chroma partage
 * (PD90) ou une structure a sync frequent (Martin1).
 *
 * Plutot que de continuer a exiger une synchronisation parfaite a CHAQUE
 * pixel (un seul cran d'avance), le thread peut maintenant travailler
 * plusieurs pixels A L'AVANCE dans une FILE CIRCULAIRE (s_queue,
 * SSTV_QUEUE_DEPTH cases) : il POUSSE une valeur (frequence + duree) des
 * qu'il en a une de prete, sans attendre que l'intervalle courant soit
 * termine, tant qu'il reste de la place dans la file. L'ISR, a chaque
 * echeance materielle, DEPILE la prochaine valeur si la file n'est pas
 * vide. Un ralentissement PONCTUEL du thread (le cout fixe evoque plus
 * haut, ou un octet du RP2040 qui met un peu plus de temps a arriver) est
 * alors absorbe par les valeurs DEJA EN FILE, au lieu de se traduire
 * immediatement par un retard materiel -- la file ne se vide que si le
 * thread prend du retard de facon SOUTENUE sur plusieurs pixels d'affilee,
 * pas a la moindre irregularite ponctuelle. */
#define SSTV_TIM_RETRY_US 20u
/* ⚠️ (2026-09-30) : porter cette file a 32 cases (tentative pour absorber
 * plus de gigue) a en realite AGGRAVE le probleme sur le terrain -- tim14
 * late (pixel) est passe de 3714 a 16166 pour une image PD120 tout aussi
 * degradee visuellement, sur le meme banc d'essai. Revenu a 16. Hypothese
 * pour expliquer ce sens contre-intuitif : une file plus profonde laisse le
 * thread "aspirer" un plus gros paquet d'octets deja en attente dans le
 * tampon UART DMA (256 o) des qu'un ralentissement libere de la place,
 * videant ce tampon-tampon plus vite -- moins de marge y reste ensuite pour
 * absorber le ralentissement SUIVANT, qui se traduit alors directement par
 * un sous-remplissage de la file. L'analyse fine de pd120_hs.wav (regression
 * lineaire des positions de synchro de paire sur toute la duree) montre par
 * ailleurs une erreur de frequence d'horloge negligeable (-0.06 %, donc PAS
 * un souci de quartz/RC) mais des ecarts locaux ponctuellement enormes
 * (jusqu'a ~260 ms sur une seule paire) : le probleme n'est pas une derive
 * lisse mais des BLOCAGES LOCALISES repetes, chacun coutant potentiellement
 * plusieurs dizaines de ms avant que le filet de securite adaptatif
 * (SSTV_STREAM_STUCK_THRESHOLD) ne se declenche -- voir ce mecanisme plus
 * bas, resserre en consequence. */
#define SSTV_QUEUE_DEPTH  64u   /* puissance de 2 -- masquage au lieu du modulo (test 2026-10-03 : 16 -> 64 pour absorber les retards RP2040) */

#define SSTV_TONE_KIND_NONPIXEL 0u
#define SSTV_TONE_KIND_PIXEL    1u

typedef struct {
    uint16_t reg71;
    uint16_t period_us;
    uint8_t  kind;
} sstv_queue_entry_t;

static volatile sstv_queue_entry_t s_queue[SSTV_QUEUE_DEPTH];
static volatile uint8_t  s_queue_head;   /* prochaine case a REMPLIR (thread) */
static volatile uint8_t  s_queue_tail;   /* prochaine case a VIDER (ISR) */
/* ⚠️ (2026-10-01, retrouve dans l'historique) : le tout premier PD120 (avant
 * la refonte en file, cf plus haut) tournait sur le pipeline ORIGINAL a un
 * seul cran d'avance (s_next_reg71/s_next_ready, SSTV_TIM_RETRY_US=200) et
 * donnait une image correcte (PD_120.png, 2026-09-26 -- seulement un peu
 * flou, limite par la basse resolution de la source d'alors, et un defaut
 * localise sur les ~90 premiers pixels, rien a voir avec la corruption de
 * chrominance generalisee vue depuis la reintroduction sur la file). La
 * file (16 cases) n'existait pas encore a l'epoque -- elle a ete ajoutee
 * PLUS TARD pour PD90/Martin1, et PD120 en a herite a sa reintroduction
 * sans qu'on reteste l'hypothese "un seul cran suffit, et vaut peut-etre
 * mieux, pour PD120". Coherent avec l'essai du 2026-09-30 : doubler la
 * profondeur (16->32) a AGGRAVE le sous-remplissage pour PD120 au lieu de
 * l'ameliorer. s_queue_mask (au lieu de la constante SSTV_QUEUE_DEPTH-1 en
 * dur) permet de retrouver ce comportement a un seul cran EXACTEMENT pour
 * PD120, sans dupliquer tout le pipeline ni toucher aux 3 autres modes
 * (Scottie1/Martin1/PD90), deja confirmes bons avec la file a 16. Masque a
 * 1 (pas 0) : un tampon circulaire a N cases n'offre que N-1 cases
 * UTILISABLES (il faut pouvoir distinguer plein de vide) -- masque=0
 * donnerait 0 case utilisable et bloquerait le thread indefiniment des le
 * premier push ; masque=1 (2 cases) donne exactement UNE case d'avance,
 * le comportement du pipeline original. Fixe par sstv_send_common() selon
 * stream_mode, avant que sstv_hw_init() ne (re)demarre la file. */
static uint8_t s_queue_mask = (uint8_t)(SSTV_QUEUE_DEPTH - 1u);
static volatile uint16_t s_last_pushed_period_us;   /* pour sstv_hw_tone_finish() */

/* Correction d'horloge radio (2026-10-04). L'oscillateur interne du PY32 fait
 * ~-0,5 % d'erreur sur les durees TIM14 (mesure : 430,3 ms au lieu de
 * 428,22 ms par ligne Scottie 1, horloge plus lente que nominal). La mesure
 * se fait SANS emission (SSTV_ClockStart/End) : le RP2040 chronometre 30 s
 * avec son quartz, la radio compte les cycles de son SysTick (meme oscillateur
 * que TIM14) pendant ce temps. Le facteur est stocke en EEPROM et applique a
 * chaque duree poussee dans la file. La valeur par defaut (-4860 ppm)
 * reproduit la correction mesuree a la main (432 us -> 430 us). */
#define SSTV_TRIM_EE_ADDR      0x0A1B8u
#define SSTV_TRIM_EE_MAGIC     0x5Au
#define SSTV_TRIM_DEFAULT_PPM  (-4860)
#define SSTV_TRIM_LIMIT_PPM    20000
static int16_t s_trim_ppm = SSTV_TRIM_DEFAULT_PPM;

static void sstv_trim_load(void)
{
    uint8_t buf[8];
    EEPROM_ReadBuffer(SSTV_TRIM_EE_ADDR, buf, sizeof(buf));
    if (buf[0] == SSTV_TRIM_EE_MAGIC)
        s_trim_ppm = (int16_t)(buf[1] | ((uint16_t)buf[2] << 8));
}

void SSTV_SetTrim(int16_t ppm)
{
    if (ppm > SSTV_TRIM_LIMIT_PPM)  ppm = SSTV_TRIM_LIMIT_PPM;
    if (ppm < -SSTV_TRIM_LIMIT_PPM) ppm = -SSTV_TRIM_LIMIT_PPM;
    s_trim_ppm = ppm;
    uint8_t buf[8] = { SSTV_TRIM_EE_MAGIC, (uint8_t)(ppm & 0xFF), (uint8_t)((uint16_t)ppm >> 8), 0, 0, 0, 0, 0 };
    EEPROM_WriteBuffer(SSTV_TRIM_EE_ADDR, buf);
}

int16_t SSTV_GetTrim(void)
{
    return s_trim_ppm;
}

/* Relit le trim stocke en EEPROM (sinon la RAM garde la valeur par defaut
 * tant qu'aucune emission n'a eu lieu depuis le demarrage). */
void SSTV_LoadTrim(void)
{
    sstv_trim_load();
}

static uint16_t sstv_trim_apply(uint16_t us)
{
    int32_t corrected = (int32_t)us + ((int32_t)us * s_trim_ppm) / 1000000;
    return (uint16_t)corrected;
}
static bool               s_hw_started;

/* ⚠️ (2026-09-27, retour terrain : repartition sync/porche vs pixel
 * incoherente d'un essai a l'autre depuis la refonte en file -- CAUSE :
 * s_pending_kind (une simple variable globale, mise a jour par le thread a
 * CHAQUE tentative de push) ne correspondait plus a l'entree responsable
 * d'une file trouvee vide par l'ISR, puisque le thread peut desormais
 * pousser plusieurs entrees d'avance sans lien direct avec ce que l'ISR
 * est en train de depiler. Corrige en rattachant le type a CHAQUE ENTREE
 * de la file (champ kind ci-dessus), et en se souvenant du type de la
 * DERNIERE entree reellement depilee (s_last_kind) -- "on etait en train
 * de faire X quand la file s'est retrouvee vide" est une attribution
 * fiable, contrairement a "le thread tentait de pousser X a cet instant
 * precis" qui ne veut plus dire grand-chose avec plusieurs cases
 * d'avance. */
static volatile uint8_t  s_last_kind;
static volatile uint16_t s_tim_late_nonpixel;
static volatile uint16_t s_tim_late_pixel;
/* Diagnostic (2026-10-03) : qui fait attendre le producteur -- octet UART
 * absent, ou file de pixels pleine. Rapportes dans CMD_SSTV_STATUS. */
static uint16_t s_cnt_uart_wait;
static uint16_t s_cnt_queue_full;
/* Mesure d'horloge (SSTV_ClockStart/End) : TIM14 tourne seul, sans
 * tonalite ni queue -- chaque debordement compte une periode de 10 ms. */
static volatile bool     s_clk_mode;
static volatile uint32_t s_clk_pops;
static uint32_t          s_clk_last_prog;   /* dernier temps programme, redonne si END est renvoye */

void TIM14_IRQHandler(void)
{
    if (!LL_TIM_IsActiveFlag_UPDATE(TIM14))
        return;
    LL_TIM_ClearFlag_UPDATE(TIM14);

    if (s_clk_mode) {          /* mesure d'horloge : rien d'autre a faire */
        s_clk_pops++;
        return;
    }

    if (s_queue_tail != s_queue_head) {
        uint16_t reg  = s_queue[s_queue_tail].reg71;
        uint16_t per  = s_queue[s_queue_tail].period_us;
        s_last_kind   = s_queue[s_queue_tail].kind;
        s_queue_tail  = (uint8_t)((s_queue_tail + 1u) & s_queue_mask);
        BK4819_WriteRegister(BK4819_REG_71, reg);
        LL_TIM_SetAutoReload(TIM14, (uint32_t)per - 1u);
    } else {
        /* File vide : le thread n'a rien pu preparer d'avance -- REG_71
         * garde naturellement sa derniere valeur (registre materiel, pas
         * touche ici), on ne fait que reessayer bientot. Attribue le
         * retard au type de la DERNIERE entree reellement appliquee. */
        LL_TIM_SetAutoReload(TIM14, SSTV_TIM_RETRY_US - 1u);
        if (s_last_kind == SSTV_TONE_KIND_PIXEL)
            s_tim_late_pixel++;
        else
            s_tim_late_nonpixel++;
    }
}

static void sstv_hw_init(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM14);
    LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_TIM14);
    LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_TIM14);

    LL_TIM_SetPrescaler(TIM14, (SystemCoreClock / 1000000u) - 1u);   /* 1 us/tick */
    LL_TIM_SetCounterMode(TIM14, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_UPDATE(TIM14);

    NVIC_SetPriority(TIM14_IRQn, 0);   /* == SysTick -- voir en-tete du fichier */
    NVIC_EnableIRQ(TIM14_IRQn);

    s_hw_started        = false;
    s_queue_head        = 0;
    s_queue_tail        = 0;
    s_last_kind         = SSTV_TONE_KIND_NONPIXEL;
    s_tim_late_nonpixel = 0;
    s_tim_late_pixel    = 0;
}

/* Pousse une valeur (frequence deja convertie + duree) dans la file, ou
 * l'applique directement s'il s'agit du tout premier appel de la
 * transmission (amorce le materiel, voir le commentaire de sstv_hw_tone()
 * plus bas). Bloque UNIQUEMENT si la file est deja pleine -- ce qui, avec
 * SSTV_QUEUE_DEPTH cases d'avance, ne devrait plus arriver en usage normal
 * (le thread a le temps de vider son avance pendant les intervalles
 * suivants). */
static void sstv_hw_queue_push(uint8_t kind, uint16_t reg71, uint16_t period_us)
{
    period_us = sstv_trim_apply(period_us);
    if (!s_hw_started) {
        /* ⚠️ (2026-10-01, retour terrain : plusieurs decodeurs independants
         * -- QSSTV, RPI_SSTV -- n'accrochent jamais sur PD120, alors que
         * HTCommander et l'appli "HT" y arrivent) -- course critique trouvee
         * ici : LL_TIM_EnableIT_UPDATE()+NVIC_EnableIRQ() sont deja actifs
         * depuis sstv_hw_init(), donc LL_TIM_GenerateEvent_UPDATE() (qui leve
         * UIF) peut faire prendre TIM14_IRQHandler() IMMEDIATEMENT, avant
         * meme la ligne LL_TIM_ClearFlag_UPDATE() suivante. Cet appel
         * parasite trouve la file VIDE (rien n'a encore ete pousse, meme pas
         * le porche qui suit toujours cette toute premiere synchro) et
         * ecrase l'ARR qu'on venait de regler (period_us-1, ex. 20000-1 pour
         * la synchro PD120) par SSTV_TIM_RETRY_US-1 (20 us) -- la toute
         * premiere tonalite de la transmission se retrouve coupee a presque
         * rien au lieu de tenir sa duree complete.
         *
         * ⚠️ (2026-10-01, suite) : d'abord corrige pour les 4 modes sans
         * distinction -- mais PD90 (jamais touche par ailleurs) est devenu
         * "de travers et non detectable par QSSTV" juste apres. Plutot que
         * de prouver que NVIC_DisableIRQ()/EnableIRQ() ici est reellement en
         * cause, applique la meme discipline que pour le masque de file et
         * le timeout : ce chemin d'amorçage est PARTAGE par les 4 modes, et
         * on a deja vu (seuil de blocage, profondeur de file) que toute
         * modification partagee, meme en apparence anodine, peut regresser
         * silencieusement Scottie1/Martin1/PD90 -- qui n'avaient pourtant
         * aucun symptome necessitant ce correctif. Restreint donc le
         * masquage IRQ au seul PD120 (s_queue_mask==1, cf plus haut) ; les 3
         * autres modes retrouvent exactement leur sequence d'origine,
         * jamais modifiee avant aujourd'hui. */
        if (s_queue_mask == 1u) {
            NVIC_DisableIRQ(TIM14_IRQn);
            BK4819_WriteRegister(BK4819_REG_71, reg71);
            LL_TIM_SetAutoReload(TIM14, (uint32_t)period_us - 1u);
            LL_TIM_GenerateEvent_UPDATE(TIM14);   /* force CNT=0, apply ARR now */
            LL_TIM_ClearFlag_UPDATE(TIM14);       /* ...but that's not a real elapsed interval */
            NVIC_EnableIRQ(TIM14_IRQn);
        } else {
            BK4819_WriteRegister(BK4819_REG_71, reg71);
            LL_TIM_SetAutoReload(TIM14, (uint32_t)period_us - 1u);
            LL_TIM_GenerateEvent_UPDATE(TIM14);   /* force CNT=0, apply ARR now */
            LL_TIM_ClearFlag_UPDATE(TIM14);       /* ...but that's not a real elapsed interval */
        }
        LL_TIM_EnableCounter(TIM14);
        s_hw_started = true;
        s_last_kind             = kind;
        s_last_pushed_period_us = period_us;
        return;
    }

    uint8_t next_head = (uint8_t)((s_queue_head + 1u) & s_queue_mask);
    if (next_head == s_queue_tail)
        s_cnt_queue_full++;
    while (next_head == s_queue_tail) { }   /* file pleine -- attendre une place */
    s_queue[s_queue_head].reg71     = reg71;
    s_queue[s_queue_head].period_us = period_us;
    s_queue[s_queue_head].kind      = kind;
    s_queue_head = next_head;               /* publie l'entree -- toujours en dernier */
    s_last_pushed_period_us = period_us;
}

/* Drop-in replacement for sstv_tone() during the image scan: same
 * signature, but the actual hold is timed by TIM14, not SYSTICK_DelayUs().
 * Must be followed by exactly one sstv_hw_tone_finish() call once the last
 * interval has been queued. Only safe for tones that do NOT depend on
 * stream data (sync/gap/porch) -- for per-pixel tones, use
 * sstv_hw_tone_pixel() instead. */
static void sstv_hw_tone(uint32_t freq_hz, uint32_t duration_us)
{
    /* One of the few places KEYBOARD_Poll() is actually called -- see
     * sstv_tx_exit_pressed()'s own comment. Called once per sync/gap/porch
     * tone (a handful of times per scan line), not per pixel. */
    if (sstv_tx_exit_pressed())
        return;
#ifdef ENABLE_FEAT_F4HWN_SLEEP
    gSleepModeCountdown_500ms = (uint16_t)(gSetting_set_off * 120u);
#endif
    uint16_t reg = (uint16_t)((freq_hz * 103244u + 5000u) / 10000u);
    sstv_hw_queue_push(SSTV_TONE_KIND_NONPIXEL, reg, (uint16_t)duration_us);
}

/* Call exactly once, right after the last sstv_hw_tone() of a transmission:
 * waits for the queue to fully drain (i.e. every pushed interval has
 * actually become the tone on the air), holds the LAST one for its own
 * duration (now counting down in hardware), then stops the clock. */
static void sstv_hw_tone_finish(void)
{
    while (s_queue_tail != s_queue_head) { }
    SYSTICK_DelayUs(s_last_pushed_period_us);
    LL_TIM_DisableCounter(TIM14);
    s_hw_started = false;
}

/* SSTV standard tones, shared by (almost) every mode's VIS header. */
#define SSTV_LEADER_HZ   1900u
#define SSTV_SYNC_HZ     1200u
#define SSTV_GAP_HZ      1500u   /* also the "black" luminance tone */
#define SSTV_BLACK_HZ    1500u
#define SSTV_WHITE_HZ    2300u
#define SSTV_VIS_BIT_US  30000u  /* 30 ms/bit, both mode bits and data bits */

/* Scottie 1: 320x256 RGB, VIS code 60 (0b0111100). Timings from the SSTV
 * Handbook's Scottie table: sync 9.0 ms, gap 1.5 ms, scan 138.240 ms/channel
 * (=> exactly 432 us/pixel over 320 pixels -- no rounding). */
#define SCOTTIE1_VIS_CODE   60u
#define SCOTTIE1_WIDTH      320
#define SCOTTIE1_HEIGHT     256
#define SCOTTIE1_SYNC_US    9000u
#define SCOTTIE1_GAP_US     1500u
/* 2026-10-03 : mesure sur s1.wav et s2.wav -- ligne reelle ~430.5 ms au lieu
 * de 428.22 ms, soit ~+0.5 % (~2.2 us/pixel). Horloge radio un peu lente ->
 * on programme 430 us pour obtenir ~432 us effectifs. Le RP2040 garde 432 us
 * comme reference de cadence (sstv_stream_scottie1). */
#define SCOTTIE1_PIXEL_US   432u   /* 138240 us / 320 : nominal, la correction d'horloge est appliquee par le trim (sstv_trim_apply) */

static void sstv_send_vis(uint8_t code)
{
    /* ⚠️ (2026-09-26, retour terrain : "decodage immediat" avec HTCommander
     * -- https://github.com/Ylianst/HTCommander, src/lib/sstv/encoder.dart
     * -- "aucun decodage" avec nous) -- BUG PROTOCOLAIRE trouve en comparant
     * a ce vrai encodeur SSTV tiers (_addVISHeader()) : la trame VIS
     * standard est leader-break-leader PUIS un bit de START (30 ms a
     * 1200 Hz, ci-dessous) AVANT les 7 bits de donnees -- un dixieme
     * element que nous n'avons jamais envoye, enchainant a tort le
     * deuxieme leader directement sur le premier bit de donnee. Un
     * decodeur strict qui attend cette trame complete pour caler ses
     * fenetres d'echantillonnage peut rejeter le VIS entierement ou lire
     * tous les bits suivants decales de 30 ms -- independamment de toute
     * gigue de cadencement par ailleurs, ca peut a soi seul expliquer
     * "aucun decodage". Notre propre decodeur Python (ce fil de discussion)
     * ne l'avait jamais remarque puisqu'il partage a tort la meme
     * hypothese erronee que ce code -- il se cale sur SA PROPRE emission,
     * pas sur la norme. */
    sstv_tone(SSTV_LEADER_HZ, 300000u);
    sstv_tone(SSTV_SYNC_HZ,   10000u);
    sstv_tone(SSTV_LEADER_HZ, 300000u);
    sstv_tone(SSTV_SYNC_HZ,   SSTV_VIS_BIT_US);   /* start bit -- manquait */

    int ones = 0;
    for (int i = 0; i < 7; i++) {
        int bit = (code >> i) & 1;   /* VIS bits go out LSB first */
        if (bit) ones++;
        sstv_tone(bit ? 1100u : 1300u, SSTV_VIS_BIT_US);
    }
    int parity = ones & 1;   /* even parity over the 7 data bits */
    sstv_tone(parity ? 1100u : 1300u, SSTV_VIS_BIT_US);

    sstv_tone(SSTV_SYNC_HZ, SSTV_VIS_BIT_US);   /* stop bit */
}

/* Read the next raw luminance byte the RP2040 is streaming, straight out of
 * the UART's own receive DMA ring buffer (see this file's header comment
 * and CMD_SSTV_START in rp2040/src/decoder_config.h) -- deliberately NOT
 * going through the normal Quansheng frame parser, which is both slower
 * than we can afford per pixel and unnecessary here (no ID/CRC needed for
 * a stream whose exact length both ends already agree on). Busy-waits for
 * the next byte if the RP2040 hasn't caught up yet -- at 230400 baud
 * (~4.3 us/byte) against a 432 us/pixel budget this should essentially
 * never actually block in practice, but if it does, it simply delays that
 * one pixel rather than desyncing anything (same self-correcting shape as
 * every other wait in this file).
 *
 * ⚠️ (2026-09-26, retour terrain : "reste en emission avec une tonalite" a
 * la fin de l'image, puis "je n'ai plus de decodage du tout") -- VRAIE
 * CAUSE trouvee cote RP2040 (voir sstv_stream_scottie1(), rp2040/src/
 * main.c) : celui-ci cadencait un debit PLAT de 432 us/octet sur tout le
 * flux, alors que nous ne lisons un octet ICI que pendant les segments de
 * balayage -- les gaps/sync entre canaux (13.5 ms de "temps mort" par
 * ligne, cote radio) faisaient prendre au RP2040 de l'avance a chaque
 * ligne, au point de deborder le tampon circulaire de 256 o bien avant la
 * fin de l'image (d'ou la derive lente observee sur un enregistrement
 * reel) puis de finir d'ecrire ~3.5 s avant que la radio n'ait besoin du
 * dernier octet (d'ou le blocage en fin d'emission). Corrige cote RP2040
 * en lui faisant suivre exactement le meme planning gap/sync/scan que la
 * boucle ci-dessous. Ce busy-wait garde neanmoins son garde-fou par
 * compteur d'iterations (meme genre que UART_TX_TIMEOUT_ITERATIONS,
 * App/driver/uart.c) en secours, jamais bloquant indefiniment sur un lien
 * serie -- et s_stream_timeout_count (ci-dessous) rapporte a chaque essai
 * combien de fois il s'est declenche, pour verifier sur le terrain que le
 * correctif RP2040 suffit (0 attendu) plutot que de le supposer. Sur
 * depassement, retourne 0 (noir) et avance quand meme -- un pixel deviendra
 * faux plutot que de bloquer toute la fin de l'emission. */
static uint16_t s_stream_read_pos;
static uint16_t s_stream_timeout_count;

/* ⚠️ (2026-09-27, retour terrain : PD90 "bon debut puis degradation
 * progressive", 630 uart timeout(s) contre 128 avant la refonte en file
 * d'attente TIM14) -- le seuil de 5 ms ci-dessous avait ete calibre pour
 * l'ANCIEN pipeline a un seul cran, ou le thread ne pouvait jamais prendre
 * plus d'un intervalle d'avance sur le materiel. Avec la file
 * (SSTV_QUEUE_DEPTH cases), le thread PEUT desormais prendre jusqu'a
 * QUEUE_DEPTH pixels d'avance sur le rythme reel du RP2040 -- ce qui est
 * voulu (absorbe les irregularites ponctuelles), mais signifie aussi qu'il
 * peut legitimement avoir besoin d'attendre jusqu'a QUEUE_DEPTH*PIXEL_US
 * pour l'octet suivant (~8.5 ms pour PD90 a 532 us/pixel avec 16 cases) --
 * plus que les 5 ms d'origine. Un abandon PREMATURE ici renvoie un pixel
 * FAUX (noir), une vraie corruption de contenu -- pire qu'un simple retard
 * que la file aurait pu absorber si on avait patiente un peu plus.
 * Remonte a 20 ms, confortablement au-dessus du pire cas avec la file
 * actuelle, tout en restant tres inferieur aux temps morts sync/porche
 * (1.5-22 ms) pour ne jamais masquer un vrai blocage. */
/* ⚠️ (2026-09-30, retour terrain : PD120 bloque en TX ~160-200 s de plus
 * apres que le RP2040 ait fini d'ecrire -- 3005 timeouts a 20 ms chacun
 * (60 s) sur ce seul compteur) -- si la radio a pris assez de retard pour
 * deborder le tampon circulaire DMA de 256 o, ou si le RP2040 a
 * litteralement fini d'envoyer, PLUS AUCUN octet ne peut plus jamais faire
 * progresser la lecture : chaque pixel RESTANT de la boucle (fixe, ex.
 * ~635000 pour PD120) attendrait alors betement le plein timeout de 20 ms,
 * un par un -- potentiellement des minutes. Filet de securite : au bout de
 * SSTV_STREAM_STUCK_THRESHOLD echecs CONSECUTIFS (bien plus qu'un aleas
 * ponctuel ne peut en causer), on suppose que la situation ne se debloquera
 * plus jamais d'elle-meme et on bascule sur un timeout bien plus court pour
 * le reste -- borne le pire cas a quelques dizaines de secondes au lieu de
 * potentiellement des minutes. Se reinitialise des qu'un octet arrive a
 * nouveau (retour a la normale si ce n'etait qu'un ralentissement
 * passager, pas un blocage definitif). */
/* ⚠️ (2026-09-30) : 20 ms x 5 consecutifs = 100 ms minimum perdus avant meme
 * que le filet de securite ne se declenche -- beaucoup trop lent une fois
 * qu'on sait (cf plus haut) que le probleme reel est fait de blocages
 * LOCALISES repetes, pas d'une derive continue. Mais 20 ms n'est pas un
 * choix arbitraire : c'est la marge legitime la plus large parmi les 4
 * modes (QUEUE_DEPTH*PIXEL_US = 16*532 = 8.5 ms pour PD90, le plus lent),
 * avec un facteur de securite -- PD120 (16*190 = 3 ms de marge legitime)
 * n'a PAS besoin d'un timeout aussi genereux, mais partageait la meme
 * constante que PD90. Rendu specifique par mode (s_stream_full_timeout_us,
 * fixe par sstv_send_common() selon stream_mode) : PD120 descend a 6 ms
 * (2x sa propre marge), Scottie1/Martin1/PD90 gardent 20 ms (aucun
 * changement, zero risque de regression la ou c'est deja confirme bon).
 *
 * ⚠️ (2026-10-01, retour terrain : PD90 "devenu de travers et non
 * detectable par QSSTV", alors que PD90 lui-meme n'a pas change) -- VRAIE
 * REGRESSION trouvee ici : le seuil ci-dessous (abaisse de 5 a 2) etait
 * PARTAGE par les 4 modes, pas specifique a PD120 comme le timeout
 * au-dessus. Pour PD90, une fois 2 timeouts consecutifs atteints (possible
 * meme en usage normal, sa marge legitime de 8.5 ms etant deja proche du
 * timeout de 20 ms), le mode "bloque" s'active et fait tomber le timeout a
 * 200 us -- tres en dessous des 8.5 ms que PD90 peut legitimement avoir
 * besoin d'attendre. PD90 se met alors a rater systematiquement ses octets
 * suivants (chaque octet manque, remet consecutive_timeouts au-dessus du
 * seuil, le mode "bloque" ne se desactive jamais) -- d'ou l'image qui part
 * de travers en cours de route. Rendu specifique par mode comme le timeout
 * : seul PD120 (qui a vraiment besoin d'une detection rapide, cf plus haut)
 * garde le seuil agressif de 2 ; Scottie1/Martin1/PD90 reviennent au seuil
 * d'origine de 5, jamais change pour eux avant cet essai. */
#define SSTV_STREAM_TIMEOUT_US_DEFAULT  20000u
#define SSTV_STREAM_TIMEOUT_US_PD120     6000u
#define SSTV_STREAM_TIMEOUT_STUCK_US      200u
#define SSTV_STREAM_STUCK_THRESHOLD_DEFAULT 5u
#define SSTV_STREAM_STUCK_THRESHOLD_PD120   2u
static uint16_t s_stream_consecutive_timeouts;
static uint16_t s_stream_full_timeout_us;
static uint16_t s_stream_stuck_threshold;

static uint8_t sstv_stream_next_byte(void)
{
    uint32_t timeout_us = (s_stream_consecutive_timeouts >= s_stream_stuck_threshold)
                               ? SSTV_STREAM_TIMEOUT_STUCK_US : s_stream_full_timeout_us;
    uint32_t timeout = timeout_us;
    if (SSTV_UartDmaWritePos() == s_stream_read_pos)
        s_cnt_uart_wait++;
    while (SSTV_UartDmaWritePos() == s_stream_read_pos) {
        if (--timeout == 0) {
            s_stream_timeout_count++;
            if (s_stream_consecutive_timeouts < 0xFFFFu)
                s_stream_consecutive_timeouts++;
            /* ⚠️ (2026-09-30) : BUG TROUVE ICI -- le commentaire au-dessus
             * de ce bloc dit "retourne 0 (noir) et avance quand meme", mais
             * s_stream_read_pos n'avancait jamais dans cette branche. Cet
             * octet reste donc "du a livrer" et sera relu au PROCHAIN
             * appel -- celui destine au pixel SUIVANT -- qui avance,
             * lui, l'index logique du pixel sans avancer la position reelle
             * dans le flux. Chaque timeout decalait ainsi DEFINITIVEMENT
             * l'image d'un octet par rapport au contenu reel (le decalage
             * ne se resorbe jamais tout seul) -- avec 4000-5000 timeouts
             * sur une image PD120, ca explique le bruit de couleur/les
             * stries qu'aucun reglage de timeout ou de profondeur de file
             * n'a jamais reussi a faire disparaitre : on ne changeait que
             * la frequence du probleme, jamais sa cause. Avance bien la
             * position maintenant, pour que le pixel SUIVANT lise le bon
             * octet -- un seul pixel devient noir (le comportement
             * documente depuis le debut), sans desynchroniser le reste. */
            s_stream_read_pos = (uint16_t)((s_stream_read_pos + 1) & 0xFFu);
            return 0;
        }
        SYSTICK_DelayUs(1);
    }
    s_stream_consecutive_timeouts = 0;
    uint8_t b = SSTV_UartDmaPeek(s_stream_read_pos);
    /* wrap at 256 (the DMA buffer's own size) -- SSTV_UartDmaWritePos()
     * only ever returns 0..255 too, the "buffer empty" test above needs
     * both sides in the same modular range or it can never match again
     * once read_pos would otherwise grow past it */
    s_stream_read_pos = (uint16_t)((s_stream_read_pos + 1) & 0xFFu);
    return b;
}

/* ⚠️ (2026-09-30, retour terrain : PD120 -- image de travers PUIS bloque en
 * TX avec une tonalite constante bien apres que le RP2040 ait fini
 * d'ecrire (log : "last reply" grimpe de 127s a 287s+), 3005 "uart
 * timeout(s)" a 20 ms chacun = 60 s de blocage a lui seul) -- a 190 us/
 * pixel (PD120), le budget est trop juste pour que la radio suive le
 * rythme de facon SOUTENUE (pas juste occasionnelle) : chaque pixel
 * derivait auparavant sstv_luma_hz() (une DIVISION) PUIS la formule
 * REG_71 (encore une DIVISION) -- or ce coeur Cortex-M0/M0+ n'a pas de
 * diviseur materiel, chaque "/" se traduit par un appel a une routine
 * logicielle de plusieurs dizaines de cycles. Deux divisions, des dizaines
 * de milliers de fois par image, pesent lourd sur un budget de
 * 190 us (9120 cycles a 48 MHz) une fois cumulees avec le reste (IRQ
 * SysTick, ecriture BK4819 dans TIM14_IRQHandler...) -- assez pour faire
 * deriver la radio en retard de facon PROGRESSIVE tout au long de
 * l'emission, jusqu'a deborder le tampon circulaire DMA de 256 o et
 * tomber dans le cas degenere ou le test d'egalite modulo-256 croit le
 * tampon "vide" alors qu'il est plein de donnees perimees -- catastrophe
 * une fois que le RP2040 a fini d'ecrire (plus aucune donnee ne peut plus
 * jamais faire progresser la lecture), chaque pixel restant de la boucle
 * (fixe, ~635000 pour PD120) attendant alors betement le plein timeout.
 * Corrige en PRECALCULANT les 256 valeurs REG_71 possibles (une seule fois,
 * a la compilation -- voir s_reg71_lut) : le chemin critique par pixel
 * devient une simple lecture de tableau, sans aucune division. */
static const uint16_t s_reg71_lut[256] = {
    15487,15518,15549,15580,15610,15641,15672,15703,15745,15776,15807,15838,15869,15900,15931,15972,
    16003,16034,16065,16096,16127,16158,16199,16230,16261,16292,16323,16354,16385,16416,16457,16488,
    16519,16550,16581,16612,16643,16684,16715,16746,16777,16808,16839,16870,16911,16942,16973,17004,
    17035,17066,17097,17139,17169,17200,17231,17262,17293,17324,17355,17397,17428,17459,17490,17521,
    17551,17582,17624,17655,17686,17717,17748,17779,17810,17851,17882,17913,17944,17975,18006,18037,
    18068,18109,18140,18171,18202,18233,18264,18295,18336,18367,18398,18429,18460,18491,18522,18563,
    18594,18625,18656,18687,18718,18749,18790,18821,18852,18883,18914,18945,18976,19007,19049,19079,
    19110,19141,19172,19203,19234,19276,19307,19338,19369,19400,19431,19461,19503,19534,19565,19596,
    19627,19658,19689,19720,19761,19792,19823,19854,19885,19916,19947,19988,20019,20050,20081,20112,
    20143,20174,20215,20246,20277,20308,20339,20370,20401,20442,20473,20504,20535,20566,20597,20628,
    20659,20700,20731,20762,20793,20824,20855,20886,20928,20959,20990,21020,21051,21082,21113,21155,
    21186,21217,21248,21279,21310,21341,21372,21413,21444,21475,21506,21537,21568,21599,21640,21671,
    21702,21733,21764,21795,21826,21867,21898,21929,21960,21991,22022,22053,22094,22125,22156,22187,
    22218,22249,22280,22311,22352,22383,22414,22445,22476,22507,22538,22579,22610,22641,22672,22703,
    22734,22765,22807,22838,22869,22900,22930,22961,22992,23023,23065,23096,23127,23158,23189,23220,
    23251,23292,23323,23354,23385,23416,23447,23478,23519,23550,23581,23612,23643,23674,23705,23746,
};   /* genere par : freq=1500+(v*800)//255 ; reg=(freq*103244+5000)//10000 -- identique a l'ancien calcul */

/* Per-pixel version of sstv_hw_tone(), used by every sstv_scan_channel*()
 * below -- fetches one stream byte and pushes its precomputed REG_71 word
 * (see s_reg71_lut above) into the TIM14 queue (see sstv_hw_queue_push()). */
static void sstv_hw_tone_pixel(uint32_t duration_us)
{
    /* Cached flag only, no KEYBOARD_Poll() here -- see sstv_tx_exit_pressed()
     * 's own comment (called once per PIXEL, far too often to poll the
     * keyboard itself without risking the kind of timing jitter this file
     * has already fought hard to eliminate). Also skips
     * sstv_stream_next_byte(), which can itself briefly block waiting on the
     * RP2040 -- no reason to pay that once a transmission is being
     * abandoned. */
    if (s_tx_abort)
        return;
#ifdef ENABLE_FEAT_F4HWN_SLEEP
    gSleepModeCountdown_500ms = (uint16_t)(gSetting_set_off * 120u);
#endif
    uint16_t reg = s_reg71_lut[sstv_stream_next_byte()];
    sstv_hw_queue_push(SSTV_TONE_KIND_PIXEL, reg, (uint16_t)duration_us);
}

static void sstv_scan_channel(void)
{
    for (int x = 0; x < SCOTTIE1_WIDTH; x++)
        sstv_hw_tone_pixel(SCOTTIE1_PIXEL_US);
}

/* Martin 1: 320x256 RGB, VIS code 44 (0b0101100). Timings from the SSTV
 * Handbook's Martin table (cross-checked against HTCommander's encoder.dart
 * encodeMartin() and CEC's MARTIN1_PIXEL_TIME): sync 4.862 ms, separator
 * 0.572 ms (before EACH of the three channels, unlike Scottie -- no mid-line
 * sync, one sync per line, always right before Green), scan 146.432 ms/
 * channel. Unlike Scottie1, there is no "first line only" quirk: every line
 * is identical (sync, sep, G, sep, B, sep, R).
 *
 * 146432 us / 320 px = 457.6 us/px -- NOT a whole number of microseconds,
 * unlike Scottie1's exact 432 us. TIM14's ARR is an integer tick count, so
 * a plain round() would systematically stretch or shrink every pixel by up
 * to 0.5 us -- individually tiny, but a CONSTANT bias across 320 pixels
 * would reproduce exactly the kind of cumulative-rate error (the "image
 * fortement de travers" bug, see TIM14 section above) already hunted down
 * once today. Fixed with a small fractional accumulator (a Bresenham-style
 * technique): hold 457 us on most pixels and 458 us on exactly 192 of the
 * 320 (146432 = 320*457 + 192*1, verified exact) -- the same technique is
 * mirrored on the RP2040 sender (main.c) so both ends stay bit-for-bit in
 * lockstep over the long run, not just on average. */
#define MARTIN1_VIS_CODE        44u
#define MARTIN1_WIDTH           320
#define MARTIN1_HEIGHT          256
#define MARTIN1_SYNC_US        4862u
#define MARTIN1_PORCH_US        572u
#define MARTIN1_PIXEL_BASE_US   457u
#define MARTIN1_PIXEL_INC_TENTHS 6u   /* 457 + 6/10 us average = 457.6 us */

static void sstv_scan_channel_martin1(void)
{
    uint32_t carry = 0;
    for (int x = 0; x < MARTIN1_WIDTH; x++) {
        carry += MARTIN1_PIXEL_INC_TENTHS;
        uint32_t extra = 0;
        if (carry >= 10u) { carry -= 10u; extra = 1u; }
        sstv_hw_tone_pixel(MARTIN1_PIXEL_BASE_US + extra);
    }
}

static void sstv_send_martin1_image(void)
{
    sstv_hw_init();
    for (int y = 0; y < MARTIN1_HEIGHT; y++) {
        sstv_hw_tone(SSTV_SYNC_HZ, MARTIN1_SYNC_US);
        sstv_hw_tone(SSTV_GAP_HZ, MARTIN1_PORCH_US);
        sstv_scan_channel_martin1();                    /* Green */
        sstv_hw_tone(SSTV_GAP_HZ, MARTIN1_PORCH_US);
        sstv_scan_channel_martin1();                    /* Blue */
        sstv_hw_tone(SSTV_GAP_HZ, MARTIN1_PORCH_US);
        sstv_scan_channel_martin1();                    /* Red */
    }
    sstv_hw_tone_finish();
}

/* PD90 (2026-09-26, remplace PD120) : PD120 (640x496, 190 us/pixel)
 * echouait au decodage sur TROIS decodeurs independants, dont QSSTV, alors
 * que Scottie1/Martin1 (432/457.6 us/pixel) fonctionnent -- suspicion forte
 * d'une limite materielle reelle du generateur Tone1/PLL du BK4819/29
 * (temps de stabilisation apres chaque changement de frequence, negligeable
 * a 432-457 us mais probablement pas a 190 us). Non pousse plus loin cote
 * diagnostic : bascule directement sur PD90, meme famille YUV420 mais
 * 532 us/pixel -- comparable aux deux modes deja eprouves, et reutilise
 * directement l'image 320x256 de Scottie1/Martin1 (pas besoin d'une image
 * source separee comme l'ancien PD120).
 *
 * VIS code 99. Timings depuis HTCommander's encoder.dart encodePaulDon('90'):
 * sync 20 ms, porch 2.08 ms (une seule fois par PAIRE de lignes, pas par
 * canal -- contrairement a Scottie/Martin, pas de separateur entre les
 * quatre sous-balayages), canal 170.24 ms => 170240/320 = 532 us/pixel
 * EXACTEMENT (pas besoin d'accumulateur fractionnaire, comme Scottie1).
 * Par paire : sync+porche, puis Y(ligne paire), V(moyenne), U(moyenne),
 * Y(ligne impaire) a la suite. Le RP2040 fait la conversion RGB->YUV (voir
 * sstv_stream_pd90(), main.c) -- la radio ne voit que des octets de niveau
 * 0..255 pour chaque canal, exactement comme Scottie1/Martin1, via la MEME
 * sstv_luma_hz(). */
#define PD90_VIS_CODE     99u
#define PD90_WIDTH        320
#define PD90_HEIGHT       256
#define PD90_SYNC_US    20000u
#define PD90_PORCH_US    2080u
#define PD90_PIXEL_US     532u

static void sstv_scan_channel_pd90(void)
{
    for (int x = 0; x < PD90_WIDTH; x++)
        sstv_hw_tone_pixel(PD90_PIXEL_US);
}

static void sstv_send_pd90_image(void)
{
    sstv_hw_init();
    for (int p = 0; p < (PD90_HEIGHT + 1) / 2; p++) {
        sstv_hw_tone(SSTV_SYNC_HZ, PD90_SYNC_US);
        sstv_hw_tone(SSTV_GAP_HZ, PD90_PORCH_US);
        sstv_scan_channel_pd90();   /* Y even */
        sstv_scan_channel_pd90();   /* V */
        sstv_scan_channel_pd90();   /* U */
        sstv_scan_channel_pd90();   /* Y odd */
    }
    sstv_hw_tone_finish();
}

/* PD120 (2026-09-27, RE-AJOUTE apres correctif de fond). Meme structure que
 * PD90 ci-dessus (VIS different, resolution 640x496, 190 us/pixel au lieu
 * de 532) -- voir decoder_config.h (rp2040) pour l'historique complet de
 * l'abandon puis du retour de ce mode. */
#define PD120_VIS_CODE    95u
#define PD120_WIDTH       640
#define PD120_HEIGHT      496
#define PD120_SYNC_US   20000u
#define PD120_PORCH_US   2080u
#define PD120_PIXEL_US    190u

static void sstv_scan_channel_pd120(void)
{
    for (int x = 0; x < PD120_WIDTH; x++)
        sstv_hw_tone_pixel(PD120_PIXEL_US);
}

static void sstv_send_pd120_image(void)
{
    sstv_hw_init();
    for (int p = 0; p < (PD120_HEIGHT + 1) / 2; p++) {
        sstv_hw_tone(SSTV_SYNC_HZ, PD120_SYNC_US);
        sstv_hw_tone(SSTV_GAP_HZ, PD120_PORCH_US);
        sstv_scan_channel_pd120();   /* Y even */
        sstv_scan_channel_pd120();   /* V */
        sstv_scan_channel_pd120();   /* U */
        sstv_scan_channel_pd120();   /* Y odd */
    }
    sstv_hw_tone_finish();
}

static void sstv_send_scottie1_image(void)
{
    /* ⚠️ (2026-09-26) SUPPRIME A NOUVEAU -- decision finale.
     * Historique : supprime une premiere fois sur la foi de HTCommander
     * (encodeur+decodeur sans ce sync) ; retabli sur la foi de CEC/uvk5cec
     * (cecsstv1.c, SSTV_SEND()), qui l'a EXACTEMENT avec le meme commentaire
     * ("STARTING SYNC PULSE (FIRST LINE ONLY)"), et dont le SSTVVISCode()
     * confirmait par ailleurs a raison le bit de start VIS.
     *
     * Mais retour terrain decisif : avec ce sync retabli, DEUX decodeurs
     * reels et independants (HTCommander ET un second logiciel HT) montrent
     * un defaut vertical a une position FIXE et IDENTIQUE sur les deux --
     * pas une derive progressive (ce qui ecarterait un probleme de
     * cadencement), mais un decalage constant, typique d'un decodeur qui
     * ancre son tout premier point de synchro a une position calculee
     * depuis la fin du VIS EN SUPPOSANT qu'il n'y a pas ce sync
     * supplementaire (voir rgb_modes.dart de HTCommander :
     * firstSyncPulseSeconds = syncPulseSeconds + 2*(separatorSeconds+
     * channelSeconds), qui ne compte que gap+G+gap+B+sync, sans les 9 ms
     * de plus qu'on envoie avant meme ce premier gap). CEC reste
     * documentairement correct, mais deux decodeurs reels indépendants et
     * largement utilises valent mieux qu'une seule reference textuelle :
     * la compatibilite pratique l'emporte. Supprime pour de bon. */
    sstv_hw_init();
    for (int y = 0; y < SCOTTIE1_HEIGHT; y++) {
        sstv_hw_tone(SSTV_GAP_HZ, SCOTTIE1_GAP_US);
        sstv_scan_channel();                            /* Green */
        sstv_hw_tone(SSTV_GAP_HZ, SCOTTIE1_GAP_US);
        sstv_scan_channel();                            /* Blue */
        sstv_hw_tone(SSTV_SYNC_HZ, SCOTTIE1_SYNC_US);
        sstv_hw_tone(SSTV_GAP_HZ, SCOTTIE1_GAP_US);
        sstv_scan_channel();                            /* Red */
    }
    sstv_hw_tone_finish();
}

/* 0 = photo, 1 = logo ADRASEC -- memes valeurs/sens que g_sstv_image_sel,
 * rp2040/src/main.c (CMD_SSTV_IMAGE_SELECT, decoder_config.h). N'affecte
 * que Scottie1/Martin1/PD90, qui partagent la meme image source 320x256
 * cote RP2040 ; PD120 (640x496, sa propre source) reste toujours sur la
 * photo -- voir le commentaire de sstv_image_pd120() cote RP2040 : le
 * Pico original n'a plus la place en flash pour un second logo a cette
 * resolution.
 *
 * ⚠️ (2026-10-03, retour terrain : "la selection ne reste pas apres
 * redemarrage (image par defaut)") -- g_sstv_image_sel cote RP2040
 * (main.c) n'est QUE de la RAM, jamais ecrite en flash : un reboot/coupure
 * du C-Board a lui seul (independant de celui de la radio) le fait
 * retomber a 0 (photo) sans que rien ne le resynchronise -- confirme par
 * recherche dans tout rp2040/src/ : aucun mecanisme de persistance flash
 * n'y existe du tout, ajouter un stockage flash+wear-leveling cote RP2040
 * pour UN SEUL bit serait disproportionne. Fixe des DEUX cotes a la fois :
 * (1) persistance EEPROM ICI, cote radio, meme recette qu'afgain.c
 * (gAfGain) -- survit a un reboot/coupure de la radio elle-meme ; (2)
 * sstv_send_common() (juste apres) renvoie desormais CMD_SSTV_IMAGE_SELECT
 * a CHAQUE emission (pas seulement au basculement), si bien que le RP2040
 * est reinforme de la bonne valeur a chaque TX, quel que soit son propre
 * historique de reboot -- il n'a donc plus besoin de memoire persistante a
 * lui, la radio la lui repete a chaque fois. */
#define SSTV_IMG_EE_ADDR   0x0A1B0u
#define SSTV_IMG_EE_MAGIC  0xA5u

static uint8_t s_image_sel;
static bool    s_image_sel_inited;

static void sstv_image_sel_init(void)
{
    if (s_image_sel_inited)
        return;
    uint8_t buf[8];
    EEPROM_ReadBuffer(SSTV_IMG_EE_ADDR, buf, sizeof(buf));
    s_image_sel = (buf[0] == SSTV_IMG_EE_MAGIC) ? buf[1] : 0;
    s_image_sel_inited = true;
}

static void sstv_send_image_sel(void)
{
    sstv_image_sel_init();
    uint8_t buf[5] = {
        (uint8_t)(CMD_SSTV_IMAGE_SELECT & 0xFFu), (uint8_t)(CMD_SSTV_IMAGE_SELECT >> 8),
        1, 0,
        s_image_sel,
    };
    SendReply(UART_PORT_UART, buf, sizeof buf);
}

void SSTV_ToggleImage(void)
{
    sstv_image_sel_init();
    s_image_sel = s_image_sel ? 0u : 1u;
    {
        uint8_t buf[8] = { SSTV_IMG_EE_MAGIC, s_image_sel, 0, 0, 0, 0, 0, 0 };
        EEPROM_WriteBuffer(SSTV_IMG_EE_ADDR, buf);
    }
    sstv_send_image_sel();

    /* Confirmation a l'ecran -- cet item de menu n'ouvre pas d'ecran dedie
     * (contrairement a APP_RunSstvRx), donc sans ca l'operateur n'a aucun
     * retour que le basculement a bien eu lieu. */
    UI_DisplayClear();
    UI_PrintStringSmallNormal(s_image_sel ? "Image: logo ADRASEC" : "Image: photo", 2, 0, 3);
    ST7565_BlitFullScreen();
    SYSTEM_DelayMs(600);
}

/* Shared by every mode: RP2040 handshake, key-up, VIS, image, teardown,
 * diagnostic report. Only the mode byte (for CMD_SSTV_START), the VIS code,
 * and the image-scan function differ between modes -- see SSTV_SendScottie1
 * ()/SSTV_SendMartin1() below, both thin wrappers around this. */
static void sstv_send_common(uint8_t stream_mode, uint8_t vis_code, void (*send_image)(void))
{
    s_tx_abort = false;   /* fresh transmission -- see sstv_tx_exit_pressed() */
    s_cnt_uart_wait = 0;
    s_cnt_queue_full = 0;
    sstv_trim_load();     /* hors chronometrage : avant la cle, pas pendant la file */

    /* Tell the RP2040 to start streaming pixel bytes NOW -- see this file's
     * header comment and CMD_SSTV_START (decoder_config.h) for the full
     * design. Sent first, before even keying up: the VIS header alone
     * takes ~1.2 s (610 ms leaders/break + 270 ms bits + our own 300 ms
     * settle below), comfortably more lead time than the RP2040 needs to
     * start filling the UART's own transmit path. s_stream_read_pos is
     * reset to the DMA buffer's CURRENT write position, not 0 -- anything
     * already sitting in that ring buffer from before this command is
     * unrelated traffic, not image data, and must not be read as such.
     *
     * CMD_SSTV_IMAGE_SELECT is sent FIRST, not after -- see its own comment
     * (sstv_send_image_sel()) : the RP2040 starts producing stream bytes as
     * soon as CMD_SSTV_START arrives, so the selector must already be known
     * to it by then, or the first image(s) after a reboot could briefly use
     * the wrong source. */
    sstv_send_image_sel();
    {
        /* [id_lo,id_hi,len_lo,len_hi,data...] -- same hand-built wire
         * format as sarsat.c's SARSAT_Reply(), SendReply() itself only
         * adds the outer AB CD/CRC/DC BA framing. */
        uint8_t buf[5] = {
            (uint8_t)(CMD_SSTV_START & 0xFFu), (uint8_t)(CMD_SSTV_START >> 8),
            1, 0,
            stream_mode,
        };
        SendReply(UART_PORT_UART, buf, sizeof buf);
    }
    s_stream_read_pos = SSTV_UartDmaWritePos();
    s_stream_timeout_count = 0;
    s_stream_consecutive_timeouts = 0;
    s_stream_full_timeout_us = (stream_mode == SSTV_STREAM_MODE_PD120)
                                   ? SSTV_STREAM_TIMEOUT_US_PD120 : SSTV_STREAM_TIMEOUT_US_DEFAULT;
    s_stream_stuck_threshold = (stream_mode == SSTV_STREAM_MODE_PD120)
                                   ? SSTV_STREAM_STUCK_THRESHOLD_PD120 : SSTV_STREAM_STUCK_THRESHOLD_DEFAULT;
    /* PD120 seul revient au pipeline a un seul cran d'avance (masque=1, 2
     * cases utiles) -- voir le commentaire pres de s_queue_mask plus haut.
     * sstv_hw_init() (appele par send_image() juste apres) ne touche pas a
     * ce masque, seulement a head/tail/compteurs -- le regler ICI, avant,
     * est donc suffisant. */
    /* TEST 2026-10-04 : PD90 sur la meme file courte que PD120 (2 entrees),
     * pour voir si le decalage de debut de ligne vient de l'avance de la file. */
    s_queue_mask = (stream_mode == SSTV_STREAM_MODE_PD120 || stream_mode == SSTV_STREAM_MODE_PD90)
                       ? 1u : (uint8_t)(SSTV_QUEUE_DEPTH - 1u);

    /* Key up on whatever channel/frequency is already selected -- same
     * model as imgfsk_tx.c's own test sender (roger-beep style, operator
     * tunes both ends to an agreed frequency first). */
    BK4819_DisableDTMF();
    RADIO_SetTxParameters();
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
    SYSTEM_DelayMs(20);
    BK4819_EnableTXLink();
    /* ⚠️ (2026-09-26, retour terrain : VIS "pas du tout reconnu" par
     * certains logiciels tiers, alors que QSSTV decode -- gain deja ecarte,
     * voir plus bas) -- 50 ms ne laissait peut-etre pas assez de temps a la
     * porteuse (PLL, PA) et au squelch/AGC du recepteur de se stabiliser
     * avant que le ton 1900 Hz du VIS ne commence : un decodeur strict qui
     * exige une detection propre sur toute la fenetre de calibration
     * (300 ms) peut echouer si son tout debut est encore du transitoire de
     * cle, la ou un decodeur plus tolerant (QSSTV) s'en accommode encore.
     * Porte a 300 ms, large marge avant tout signal utile -- n'affecte pas
     * la norme SSTV elle-meme (le VIS ne commence qu'apres). */
    SYSTEM_DelayMs(300);

    /* ⚠️ (2026-09-25, retour terrain : "un pic puis plus rien" -- porteuse
     * coupee) -- REG_30 n'est PLUS touche ici. BK4819_EnableTXLink()
     * (au-dessus) l'a deja ecrit avec un jeu de bits bien plus riche que le
     * seul trio audio (AF_DAC | DISC_MODE | TX_DSP) : il y ajoute
     * notamment ENABLE_PLL_VCO et ENABLE_PA_GAIN, sans lesquels il n'y a
     * tout simplement plus de porteuse RF -- seul le ton audio en bande de
     * base continuait a etre genere. Reecrire REG_30 ici avec seulement le
     * trio audio effacait ces bits et coupait donc l'emission, tout en
     * laissant le PTT/LED actifs (qui ne refletent que le GPIO, pas la
     * presence reelle de porteuse). Le trio audio necessaire au ton1 est de
     * toute facon deja inclus dans ce que EnableTXLink() ecrit -- rien a
     * ajouter. */
    /* ⚠️ (2026-09-26) TROUVE : "ecretage des aigues" en retour terrain, gain
     * REG_70 sans AUCUN effet audible teste a 28/96/120 -- tout ca vient de
     * copier le mode AF "BEEP" (REG_47, valeur tiree du depot externe
     * GOGUFW-UV-K1-Messenger) au lieu de regarder ce que fait DEJA ce
     * meme projet pour un autre TX a base de Tone1 : APRS (aprs.c,
     * APRS_SendTones(), fonctionnel et deja valide sur l'air). Celui-ci
     * utilise BK4819_SetAF(BK4819_AF_MUTE) -- PAS BEEP -- et un gain 64
     * (son propre commentaire : "roger beep uses 66, DTMF 65" -- valeurs
     * REELLES de CE firmware, pas d'un depot tiers potentiellement
     * different). Le mode BEEP achemine vraisemblablement le ton par un
     * circuit de conditionnement audio destine a l'ecoute locale (avec
     * pre-accentuation FM incluse, normale pour la voix mais qui surdose
     * les tons aigus d'un ton fixe comme du SSTV -- exactement le symptome
     * observe), la ou MUTE pilote Tone1 directement sans ce traitement.
     * Bascule sur le meme schema qu'APRS -- fonctions/constantes du driver
     * partage, pas de valeur hex devinee. */
    BK4819_EnterTxMute();
    BK4819_SetAF(BK4819_AF_MUTE);
    BK4819_WriteRegister(BK4819_REG_70,
        BK4819_REG_70_ENABLE_TONE1 |
        (64u << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));   /* == AFSK_TONE_GAIN, aprs.c */
    BK4819_ExitTxMute();   /* tone now on air */

    sstv_send_vis(vis_code);
    send_image();

    BK4819_EnterTxMute();
    BK4819_WriteRegister(BK4819_REG_70, 0x0000u);
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
    RADIO_SetupRegisters(true);   /* back to RX, same teardown as imgfsk_tx.c used */

    /* ⚠️ (2026-09-26, retour terrain : ce rapport n'arrivait jamais --
     * "aucune tonalite ne dure plus que prevu" cote radio (le TX se coupe
     * bien, juste quelques secondes de trop) mais le RP2040 ne recevait
     * jamais cette trame, et plus aucun trafic du tout par la suite ("link
     * DOWN", "last reply 110s ago") -- initialement envoye AVANT ce
     * teardown, donc encore keye/en TX au moment de l'appel. A la
     * difference de CMD_SSTV_START (envoye avant meme RADIO_SetTxParameters
     * (), radio encore dans son etat RX normal), envoyer ici pendant que le
     * TX est actif se heurte visiblement a un etat UART/BK4819 pas encore
     * fiable pour emettre une trame -- deplace apres RADIO_SetupRegisters()
     * ci-dessus, une fois la radio revenue dans l'etat RX normal ou tout le
     * reste du protocole (HELLO, ACKs) fonctionne deja sans probleme. */
    /* 2026-09-27: payload etendu a 6 octets -- s_tim_late_count (un seul
     * total) remplace par deux compteurs separes, non-pixel (sync/porche)
     * et pixel (lie au flux RP2040), voir le commentaire au-dessus de
     * TIM14_IRQHandler. */
    {
        uint8_t buf[14] = {
            (uint8_t)(CMD_SSTV_STATUS & 0xFFu), (uint8_t)(CMD_SSTV_STATUS >> 8),
            10, 0,
            (uint8_t)(s_stream_timeout_count & 0xFFu), (uint8_t)(s_stream_timeout_count >> 8),
            (uint8_t)(s_tim_late_nonpixel & 0xFFu), (uint8_t)(s_tim_late_nonpixel >> 8),
            (uint8_t)(s_tim_late_pixel & 0xFFu), (uint8_t)(s_tim_late_pixel >> 8),
            (uint8_t)(s_cnt_uart_wait & 0xFFu), (uint8_t)(s_cnt_uart_wait >> 8),
            (uint8_t)(s_cnt_queue_full & 0xFFu), (uint8_t)(s_cnt_queue_full >> 8),
        };
        SendReply(UART_PORT_UART, buf, sizeof buf);
    }

    /* Confirmation a l'ecran si l'operateur a coupe via EXIT -- sans ca,
     * rien ne distingue a l'ecran une emission interrompue d'une emission
     * qui est simplement allee a son terme. */
    if (s_tx_abort) {
        UI_DisplayClear();
        UI_PrintStringSmallNormal("SSTV TX annulee (EXIT)", 2, 0, 3);
        ST7565_BlitFullScreen();
        SYSTEM_DelayMs(600);
    }
}

void SSTV_SendScottie1(void)
{
    sstv_send_common(SSTV_STREAM_MODE_SCOTTIE1, SCOTTIE1_VIS_CODE, sstv_send_scottie1_image);
}

void SSTV_SendMartin1(void)
{
    sstv_send_common(SSTV_STREAM_MODE_MARTIN1, MARTIN1_VIS_CODE, sstv_send_martin1_image);
}

void SSTV_SendPD90(void)
{
    sstv_send_common(SSTV_STREAM_MODE_PD90, PD90_VIS_CODE, sstv_send_pd90_image);
}

void SSTV_SendPD120(void)
{
    sstv_send_common(SSTV_STREAM_MODE_PD120, PD120_VIS_CODE, sstv_send_pd120_image);
}


/* ---- Calibration horloge radio, SANS emission (2026-10-04) ----
 * La radio fait tourner TIM14 seul pendant que le RP2040 chronometre (quartz,
 * precis a ~20 ppm). Chaque debordement TIM14 = 10 000 us programmes : on
 * compte les debordements et on les compare au temps reel donne par le
 * RP2040. C'est exactement la base de temps des durees de pixels, donc le
 * trim corrige directement l'erreur qui fait penche les images. Aucune
 * ecriture BK4819 : pas d'emission. */
#define SSTV_CLK_PERIOD_US  10000u
#define SSTV_CMD_CLK_REPORT 0x06EDu     /* radio -> RP2040: {prog_us:u32} -- temps programme ecoule */
#define SSTV_CMD_CLK_ACK    0x06EEu     /* radio -> RP2040: {} -- timer demarre (accuse de reception) */

/* Le RP2040 mesure le temps reel entre DEUX accuses (un au demarrage, un
 * apres l'arret) : la latence de la boucle radio, identique dans les deux cas
 * a un petit bruit pres, s'annule. Pas de trim ici, c'est le RP2040 qui le
 * calcule et le renvoie (SSTV_CMD_TRIM). */
void SSTV_ClockStart(void)
{
    sstv_hw_init();
    LL_TIM_SetAutoReload(TIM14, SSTV_CLK_PERIOD_US - 1u);
    LL_TIM_ClearFlag_UPDATE(TIM14);
    s_clk_pops = 0;
    s_clk_mode = true;
    LL_TIM_EnableCounter(TIM14);
    uint8_t ack[4] = { (uint8_t)(SSTV_CMD_CLK_ACK & 0xFFu), (uint8_t)(SSTV_CMD_CLK_ACK >> 8), 0, 0 };
    SendReply(UART_PORT_UART, ack, sizeof ack);
}

void SSTV_ClockEnd(void)
{
    /* Idempotent : si le RP2040 renvoie END (trame perdue), on redonne le meme
     * resultat sans toucher au timer. */
    if (s_clk_mode) {
        LL_TIM_DisableCounter(TIM14);
        s_clk_mode = false;
        /* Temps programme = debordements * 10 ms + fraction en cours (CNT, en us). */
        s_clk_last_prog = s_clk_pops * SSTV_CLK_PERIOD_US + (uint32_t)LL_TIM_GetCounter(TIM14);
    }
    uint32_t prog = s_clk_last_prog;
    uint8_t buf[8] = {
        (uint8_t)(SSTV_CMD_CLK_REPORT & 0xFFu), (uint8_t)(SSTV_CMD_CLK_REPORT >> 8),
        4, 0,
        (uint8_t)(prog & 0xFFu), (uint8_t)((prog >> 8) & 0xFFu),
        (uint8_t)((prog >> 16) & 0xFFu), (uint8_t)((prog >> 24) & 0xFFu),
    };
    SendReply(UART_PORT_UART, buf, sizeof buf);
}
