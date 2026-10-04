/*
 * decoder_config.h — build-time configuration for the RP2040 SARSAT decoder.
 *
 * Target hardware: KD8CEC's "C-Board (DSP-Board) for UV-K5", *Basic Version*
 *   www.hamskey.com/2024/03/c-board-for-uv-k5.html
 *   - board            : RP2040-Zero (or a single-sided clone)
 *   - RX audio in       : radio speaker output (FM discriminator, flat AF) ->
 *                         1uF series cap (0.1uF stock -- swap it) -> node
 *                         clamped by two 1N4148 (to GND and to 3V3) -> GP26,
 *                         plus a 2x 22k bias divider (3V3 / GND) on the node
 *   - serial to radio   : GP0 = UART0 TX -> radio serial RX (2.5mm jack)
 *                         GP1 = UART0 RX <- radio serial TX (3.5mm jack ring)
 *   - power             : radio's ~3.3 V rail feeds the RP2040-Zero 3V3 pad
 *                         directly (mic-jack VCC). No on-board regulator used.
 *   - bias divider      : 2x 22k (3V3 -> GP26 node -> GND) MUST be added to the
 *                         Basic Version; without it the AC-coupled input floats
 *                         and the audio is half-wave clamped. See docs/hardware.md.
 *
 * Verified by static analysis of KD8CEC's stock C-Board image
 * (cboard_v032.uf2, "C-BOARD FOR UV-K5/CEC4 V0.32"):
 *   - UART0 on GP0/GP1 (GPIO func 2)                  = radio link
 *     (stock firmware: 57600 8N1; we use 38400, see below)
 *   - UART1 on GP4/GP5, baud 9600                     = GPS (NMEA), unused here
 *   - GP26 / ADC0, DMA-fed                            = audio in
 *   - GP2 / GP15 / GP22 / GP25 / GP27                 = misc I/O (buttons / LED
 *     / PTT for the monitoring+GPS variants), unused for SARSAT RX
 *   KD8CEC's own C-Board <-> CEC-firmware protocol is proprietary; we run our
 *   own 0x06Cx protocol on the same wires (see docs/protocol.md).
 *
 * Electrical caveats from KD8CEC's article, relevant here:
 *   - UV-K5 speaker audio approaches ~8 V at max volume; the RP2040 ADC is
 *     0..3.3 V. Keep the radio volume low. The 1N4148 pair is the only
 *     over-voltage protection.
 *   - Some UV-K5 units brown out the C-Board (insufficient jack current) — see
 *     the article's "point A / point B" resistor mod.
 *
 * SARSAT is RX-only: the C-Board's mic-audio (PWM) and PTT paths are unused.
 *
 * Board core is selected with -DPICO_BOARD=pico / pico2 at CMake time.
 */
#ifndef SARSAT_DECODER_CONFIG_H
#define SARSAT_DECODER_CONFIG_H

/* ---- audio input (C-Board mod: SPK -> 1uF -> 1N4148 clamp + 2x22k bias -> GP26) */
/*
 * IMPORTANT: the C-Board "Basic Version" has NO bias resistor on this node, so
 * the 0.1uF-coupled input floats near a rail and the audio gets half-wave
 * clamped by the 1N4148 to GND. Symptom in the log: idle ADC ~10-40 (not
 * ~2048) and every burst reads adc[0..4095]. Add a divider from 3V3 and GND to
 * the GP26 node -- 2x 22k (NOT 100k: the RP2040 ADC's ~100k dynamic input
 * impedance would skew the reading) -- so it sits at mid-scale (~1.65 V). Also
 * bump the coupling cap 0.1uF -> 1uF (R1||R2 = 11k -> ~14 Hz high-pass, keeps
 * the 400 bps Manchester low end). See docs/hardware.md. Nothing works without it.
 */
#define CFG_ADC_GPIO          26      /* ADC0 = the audio node                  */
#define CFG_ADC_CHANNEL       0
/* The CEC C-Board ties GP26/GP27/GP28 (ADC0/1/2) together onto the audio node.
 * We only sample ADC0, but the other two pads must be put in analog mode as
 * well (adc_gpio_init: pulls off + digital input buffer off) or their logic
 * input buffers load the node and draw shoot-through current as the audio
 * swings through the ~1.65 V logic threshold -> DC shift and distortion.
 * Bit i set here = also adc_gpio_init GP(26+i). */
#define CFG_ADC_SHORTED_MASK  0x07u    /* GP26 | GP27 | GP28                     */
#define CFG_SAMPLE_RATE_HZ    16000   /* 12000..48000; 16k -> 40 samples/bit   */
#define CFG_ADC_DC_CENTER     2048    /* 12-bit mid-scale; auto-tracked at run */
#define CFG_AUDIO_GAIN_SHIFT  3       /* (raw - dc) << shift -> slicer domain   */

/* ---- burst capture --------------------------------------------------- */
#define CFG_WINDOW_MS        1100     /* >= 2x a full FGB burst (~520 ms) so a  */
                                      /* whole frame always fits whatever its   */
                                      /* phase in the window                    */
#define CFG_BURST_PEAK_ON     3000    /* gained |sample| for the ABSOLUTE arm    */
                                      /* path (kept as a fallback). The main    */
                                      /* detector is now relative, see below.   */
#define CFG_DECODE_MAX_RMS    4000    /* absolute ceiling: never slice a window  */
                                      /* whose RMS is above this (it is hiss or  */
                                      /* clipping, not a captured carrier).     */
#define CFG_BURST_QUIET_PCT   60      /* RELATIVE burst detector. A 406 MHz FGB  */
                                      /* burst CAPTURES the FM RX, so its window */
                                      /* RMS collapses to well below the tracked */
                                      /* no-carrier hiss floor -- empirically    */
                                      /* 6-12 dB down (x2..x4). Slice a window   */
                                      /* when its RMS is below this percentage   */
                                      /* of the floor. This is independent of    */
                                      /* the absolute AF gain: tune the radio    */
                                      /* level screen to the *hiss* ("OK") and   */
                                      /* the quieter beacon still decodes, so    */
                                      /* the gain need not be pushed hot (which  */
                                      /* would clip a strong 2 m APRS packet on  */
                                      /* the shared audio tap). Raise toward 80  */
                                      /* if bursts are missed, lower toward 40   */
                                      /* if hiss slips through. The [lvl]/[burst]*/
                                      /* logs print rms as a %% of floorRMS.     */
#define CFG_REARM_MS          2000    /* min gap between two decode attempts    */
#define CFG_DEDUP_S          120      /* same hex ID within N s -> terse log    */

/* ---- audio conditioning ------------------------------------------- */
#define CFG_AUDIO_AGC         1       /* 1 = normalise each decoded window to    */
                                      /*     ~CFG_AGC_TARGET RMS before the      */
                                      /*     slicer (the quieted beacon audio is */
                                      /*     weak). 0 = raw.                     */
#define CFG_AGC_TARGET        6000

/* ---- serial link to the radio (C-Board: GP0/GP1 = UART0) ----------- */
#define CFG_RADIO_UART        uart0
#define CFG_RADIO_UART_TX_GPIO 0      /* -> radio serial RX (mic / 2.5mm jack)  */
#define CFG_RADIO_UART_RX_GPIO 1      /* <- radio serial TX (3.5mm jack ring)   */
/* ⚠️ (2026-09-26) branch SSTV_SSDV, temps reel : porte de 38400 a 230400.
 * Ce lien est un protocole 100% maison (docs/protocol.md : "le debit est un
 * choix libre puisque les deux bouts sont a nous"), donc rien ne le fige a
 * 38400 -- c'etait juste le defaut egzumer/F4HWN. Necessaire pour le
 * streaming SSTV temps reel (voir CMD_SSTV_START plus bas) : le pire cas
 * dimensionnant est PD-120 (640x480, 190 us/pixel, source : SSTV Handbook,
 * OK2MNM) qui demande ~5263 o/s en continu -- 38400 bauds ne donne que
 * ~3840 o/s (deja insuffisant), 230400 en donne ~23000, marge confortable
 * meme au-dela de PD-120. Les DEUX bouts (ici et App/driver/uart.c cote
 * radio, patche par build.sh) doivent changer ensemble. */
#define CFG_RADIO_UART_BAUD   230400

/* ---- SARSAT application command IDs (radio firmware, Phase 3/4) ----- */
#define CMD_SARSAT_CLEAR     0x06C0   /* no payload                              */
#define CMD_SARSAT_TEXT      0x06C1   /* {line:u8, invert:u8, ascii[...]}       */
#define CMD_SARSAT_BEACON    0x06C2   /* packed struct, see docs/protocol.md    */
#define CMD_SARSAT_LEVEL     0x06C3   /* {peak:u16, rms:u16, dc:u16, clip:u8,   */
                                      /* adc_min:u16, adc_max:u16, verdict:u8}  */
                                      /* LE. Feeds the radio level screen. No   */
                                      /* ACK (sent ~1/s).                       */
#define CMD_SARSAT_HELLO     0x06CF   /* {proto_ver:u8} keepalive               */

/* ---- APRS (RP2040 decodes 144.8 MHz packets, pushes text to the radio) --- */
#define CMD_APRS_CONFIG      0x06D0   /* radio -> RP2040: {call[6], ssid:u8,    */
                                      /*  path:u8, sym_table:u8, sym_code:u8,   */
                                      /*  digi_level:u8, flags:u8}.             */
                                      /*  digi_level: 0 off, 1 WIDE1-N, 2 also  */
                                      /*  WIDE2-N, 3 also WIDE3-N (cumulative). */
                                      /*  flags bit0 = KISS TNC mode (USB-CDC   */
                                      /*  becomes a binary KISS stream; the     */
                                      /*  RP2040 stops decoding-for-display and */
                                      /*  digipeating -- see kiss.h).           */
#define CMD_APRS_RXTEXT      0x06D2   /* RP2040 -> radio: {line:u8, ascii[...]} */
                                      /* line 0xFF = clear the RX view          */
#define CMD_APRS_RXINFO      0x06D3   /* RP2040 -> radio: structured decode     */
                                      /* (kind, symbol, lat/lon, cse/spd/alt,   */
                                      /*  dist/bearing, src, name, text)        */
#define CMD_APRS_GPS         0x06D5   /* RP2040 -> radio: GPS fix               */
                                      /* {flags:u8, lat_e5:i32, lon_e5:i32,     */
                                      /*  speed_kmh:u16, course:u16, alt:i16,   */
                                      /*  sats:u8}. flags bit0 = fix valid.     */
#define CMD_APRS_DIGI        0x06D6   /* RP2040 -> radio: raw AX.25 frame to    */
                                      /* digipeat -- dst[7] src[7] digi[7]*n    */
                                      /* ctrl pid info, NO FCS (the radio       */
                                      /* recomputes it and keys up on channel   */
                                      /* 170, same as its own beacon). Sent     */
                                      /* only when APRS_CONFIG's digi_level     */
                                      /* allowed repeating this frame (see      */
                                      /* aprs_digi.h) and it is not a recent    */
                                      /* duplicate -- OR, in KISS mode, for     */
                                      /* every frame the host asks to send.     */

/* ---- Radiosondes (RS41 full decode, M10/M20 full GPS decode) ----------- */
/* MODE_SONDE is a THIRD, explicitly-opened capture mode alongside SARSAT and
 * APRS (see main.c) -- it shares the 400-406 MHz band with SARSAT, so unlike
 * APRS (picked purely from the radio's RX frequency) it cannot be auto-
 * selected that way; the radio's SARSAT_HELLO reply's screen-state byte
 * (d[6]) carries a 3rd value (3 = "Sonde screen open") the RP2040 uses
 * instead, alongside the existing 0/1/2 (values 4/5 were IMGFSK, branch
 * SSTV_SSDV, abandoned -- see git history -- and are free for reuse, e.g.
 * by the SSTV mode that replaced it). Two parallel demod chains share
 * this one ADC stream: RS41 (+ the older header-only M10/M20 detector) at
 * 4800 baud, and the full M10/M20 GPS decoder at 9600 baud (see
 * sonde_sync.h / sonde_m10.h) -- DFM is not attempted (no confirmed sync
 * word yet, see sonde_sync.h). Continuous streaming capture (like APRS, not
 * a windowed burst capture like SARSAT): each sync hunter runs on every bit
 * regardless of frame boundaries, and this also happens to be the right
 * shape for DFM's continuous (non-bursty) transmission whenever it gets a
 * sync word to hunt for.
 *
 * 96 kHz (not 48 kHz): the first on-air test of the 9600 baud M10/M20 chain
 * at 48 kHz (5 samples/bit -- half RS41's margin) lost bit-lock partway
 * through real captures (visible as a clean run of bytes degrading into a
 * repeating FF FF FF... tail once the PLL drifted off a real, noisy
 * signal -- a differentially-decoded constant bit stream produces exactly
 * that pattern). 96 kHz gives M10/M20 the same 10 samples/bit margin RS41
 * already has proven on air, at the cost of tighter ADC ring timing (see
 * SONDE_RING_SAMPLES below, doubled to compensate).
 *
 * ⚠️ TEST (2026-09-11, diagnostic): reverted to 48 kHz here, on the same
 * hardware where 96 kHz has since shown a reproducible, hardware-level
 * flat-zero run (raw ADC pinned exactly at its floor for tens of ms,
 * confirmed via a sentinel-fill test on the RP2040's own raw 'y' capture
 * -- not a firmware bug, not a decode artifact, not tied to any BK4819
 * register this project controls, not tied to the received signal
 * content, not tied to power supply) -- present on two independently
 * built C-Boards, but absent from SARSAT (16 kHz) and APRS (13.2 kHz),
 * both of which share this exact same continuous free-running ADC+DMA
 * ring architecture, just at 6-7x lower sample rates. If the RP2040's ADC
 * has a small per-conversion glitch rate, running 6-7x more conversions
 * per second would produce proportionally more (or none, if it's a rate
 * threshold rather than a per-sample probability) -- this test isolates
 * that variable directly.
 *
 * IMPORTANT CAVEAT, re-read from this same comment's history above: the
 * 48 kHz "lost bit lock -> FF FF FF tail" symptom that justified the
 * original move to 96 kHz was diagnosed from DECODED BITS alone, before
 * this project had any way to look at the raw ADC values a capture was
 * actually built from (the 'y' command / sonde.raw dump did not exist
 * yet). A signal that flatlines to a constant DC value for a stretch
 * produces *exactly* that same "clean bytes degrading into a repeating
 * FF FF FF tail" signature once it goes through NRZI differential
 * decoding -- so that old 48 kHz failure may well have been this same
 * hardware-level flat-zero event the whole time, just not recognized as
 * such yet.
 *
 * TESTED and REVERTED (2026-09-11): ran this at 48 kHz on air. The
 * dropout still happened -- same order of magnitude in wall-clock ms on
 * the raw 'y' capture as every 96 kHz capture this session, if anything
 * covering a *larger* fraction of one M10/M20 capture window in chip
 * units that specific time (though that's a single data point, could be
 * natural variance in a real event rather than a rate effect). No
 * measurable benefit, and a real cost (half the oversampling margin per
 * chip) -- reverted to 96000 per this note's own criterion ("revert if
 * this doesn't change anything measurable"). Sample rate is not the
 * variable behind this dropout. */
#define CFG_SONDE_SAMPLE_RATE_HZ 96000
#define SONDE_RING_SAMPLES  16384        /* 2^14, ~170 ms @ 96 kHz -- same    */
                                         /* time margin as the previous 8192  */
                                         /* @ 48 kHz gave (doubled the rate,  */
                                         /* doubled the buffer)               */
#define SONDE_RING_BITS     14           /* log2(SONDE_RING_SAMPLES*2), must */
                                         /* survive the blocking radio_send  */
                                         /* burst on a decode (same rationale */
                                         /* as APRS_RING_BITS, aprs_rx.c)    */

#define CMD_SONDE_CLEAR      0x06E0u     /* no payload                       */
#define CMD_SONDE_TEXT       0x06E1u     /* {line:u8, invert:u8, ascii[...]} */
                                         /* -- same shape/line count as      */
                                         /* CMD_SARSAT_TEXT, own line buffer */
                                         /* on the radio side (app/sonde.c)  */

/* branch SSTV_SSDV : IMGFSK (transmission d'image SSDV sur porteuse FSK
 * brute du BK4819/29) ABANDONNEE (2026-09-25) apres deux architectures
 * essayees en vain :
 *  - v1 (radio -> RP2040, CMD_IMGFSK_RXPKT/RXDIAG, correlateur FSK materiel
 *    du BK4819/29, meme moteur que l'AirCopy stock) : ~30 commits de debug
 *    terrain, le correlateur s'arrete par intermittence de generer la
 *    moindre interruption apres quelques paquets, cause jamais elucidee
 *    malgre des registres identiques au bit pres a l'AirCopy prouve.
 *  - v2 (RX demodulation logicielle cote RP2040, comme APRS/Sonde, TX
 *    materiel inchange) : le correlateur RX n'etait plus en cause, mais
 *    apres avoir resolu successivement le mot de synchro, le scramble FSK,
 *    et l'alignement de trame (tous confirmes par capture reelle), le
 *    contenu utile ne correspondait jamais aux donnees reellement envoyees
 *    -- teste exhaustivement, y compris avec un motif de test trivial
 *    (compteur d'octets), sans trouver de transformation qui fonctionne.
 * Remplacee par du SSTV (TX materiel BK4819/29 par generation de tonalite
 * continue REG_70/71, RX logiciel cote RP2040 par estimation de frequence
 * instantanee) -- protocole plus simple et deja proche de l'architecture
 * audio/tons AFC deja prouvee par APRS.
 *
 * SSTV RX (rp2040/src/sstv_demod.h) : mode explicitement arme, meme
 * mecanisme que Sonde/l'ancien IMGFSK -- reutilise le meme octet
 * "screen_state" de CMD_SARSAT_HELLO (d[6]) avec la valeur 4, laissee
 * libre par le retrait d'IMGFSK (patch/sarsat.c cote radio,
 * SSTV_RxActive() -- une seule bascule menu, pas de choix de debit comme
 * l'ancien IMGFSK puisque Scottie 1 est pour l'instant le seul mode gere).
 * Frequence d'echantillonnage propre (48 kHz, pas les 13.2 kHz d'APRS) :
 * les tons SSTV montent a 2300 Hz, et l'estimateur de frequence a
 * passages par zero interpole a besoin de marge (voir sstv_demod.h) --
 * verifie empiriquement : a 13.2 kHz l'estimation oscillait de +/-75 Hz
 * (bien trop pour distinguer 256 niveaux de luminance sur une plage de
 * 800 Hz), reduit a moins de 2 Hz a 48 kHz avec en plus la correction de
 * la formule de periode (voir l'historique de sstv_demod.c). Buffer ADC
 * partage avec APRS (g_aprs_ring, jamais actifs en meme temps), comme
 * l'etait IMGFSK. */
#define CFG_SSTV_SAMPLE_RATE_HZ 48000

/* ⚠️ (2026-09-26) SSTV TX temps reel (branch SSTV_SSDV) : l'image de test
 * embarquee en flash radio (~118 Ko au total, deja a 98% -- voir
 * sstv_tx.c) plafonnait la qualite bien en dessous de ce que la RESOLUTION
 * du mode SSTV permettrait (320x256 pour Scottie 1, jusqu'a 640x480 pour
 * PD-120). Le RP2040 a 2 Mo de flash -- largement de quoi garder l'image
 * source en pleine qualite -- mais le TX reste materiel (BK4819/29, seul
 * capable de generer proprement le ton FM en continu). Solution : la
 * radio garde la generation VIS/sync/gap (fixe, deja prouvee) mais
 * demande au RP2040, au moment ou l'utilisateur declenche l'emission, de
 * lui STREAMER en direct un octet de luminance brut par pixel -- plus
 * aucune image embarquee cote radio.
 *
 * CMD_SSTV_START (radio -> RP2040, sans ACK, envoye au tout debut de
 * SSTV_SendScottie1() avant meme le VIS -- ~1.2 s d'avance avant que le
 * premier pixel ne soit reellement necessaire) : {mode:u8}. mode=0 =
 * Scottie 1 (320x256, ordre G-B-R par ligne, 245760 octets au total).
 * mode=1 = Martin 1 (2026-09-26, meme resolution/ordre d'octets, seul le
 * cadencement differe). Des valeurs suivantes pourront designer d'autres
 * modes (PD120...) sans changer la forme du message.
 *
 * Une fois CMD_SSTV_START recu, le RP2040 arrete tout autre trafic
 * SORTANT (HELLO periodique compris) et envoie les 245760 octets de
 * luminance EN BRUT sur la meme liaison UART, SANS le framing Quansheng
 * habituel (pas d'ID/taille/CRC) -- cote radio, un lecteur direct du
 * tampon circulaire DMA de reception (App/driver/uart.c, deja existant,
 * 256 o) consomme ces octets au rythme exact du pixel (voir sstv_tx.c),
 * sans repasser par le parseur de trames pour ne pas perdre de temps
 * pendant la fenetre critique. Debit necessaire : voir le commentaire de
 * CFG_RADIO_UART_BAUD plus haut. */
#define CMD_SSTV_START        0x06E2u    /* radio -> RP2040: {mode:u8} */
#define SSTV_STREAM_MODE_SCOTTIE1 0u
#define SSTV_STREAM_SCOTTIE1_WIDTH  320
#define SSTV_STREAM_SCOTTIE1_HEIGHT 256
#define SSTV_STREAM_SCOTTIE1_BYTES \
    (SSTV_STREAM_SCOTTIE1_WIDTH * SSTV_STREAM_SCOTTIE1_HEIGHT * 3u)

/* Martin 1 (2026-09-26) : meme resolution et meme ordre d'octets (G-B-R par
 * ligne) que Scottie 1 -- seul le cadencement gap/sync/pixel differe cote
 * radio (sstv_tx.c) et RP2040 (main.c, sstv_stream_martin1()). Reutilise
 * donc directement g_sstv_master_image (sstv_master_image.h), pas besoin
 * d'une deuxieme image source pour ce mode. */
#define SSTV_STREAM_MODE_MARTIN1  1u
#define SSTV_STREAM_MARTIN1_WIDTH  320
#define SSTV_STREAM_MARTIN1_HEIGHT 256
#define SSTV_STREAM_MARTIN1_BYTES \
    (SSTV_STREAM_MARTIN1_WIDTH * SSTV_STREAM_MARTIN1_HEIGHT * 3u)

/* PD90 (2026-09-26, remplace PD120 -- voir git log : PD120 (640x496,
 * 190 us/pixel) echouait au decodage sur TROIS decodeurs independants,
 * dont QSSTV, alors que Scottie1/Martin1 (432/457.6 us/pixel) fonctionnent
 * -- suspicion forte d'une limite materielle reelle du generateur Tone1/PLL
 * du BK4819/29 (temps de stabilisation apres chaque changement de
 * frequence, negligeable a 432-457 us mais probablement pas a 190 us). Pas
 * pousse plus loin cote diagnostic : bascule directement sur PD90, meme
 * famille YUV mais 532 us/pixel -- comparable aux deux modes deja
 * eprouves. Meme resolution que Scottie1/Martin1 (320x256) : reutilise
 * g_sstv_master_image directement, pas besoin d'image source separee
 * (contrairement a l'ancien PD120 et sa propre image 640x496, supprimee).
 * Codage YUV420 par PAIRE de lignes (Y de chaque ligne + U/V moyennes sur
 * la paire) -- pas le meme octet par pixel que Scottie1/Martin1 (RGB brut).
 * Le RP2040 convertit RGB->YUV a la volee pendant le streaming
 * (sstv_stream_pd90(), main.c) ; la radio ne voit que des octets de
 * "niveau" 0..255, exactement comme pour les deux autres modes
 * (sstv_luma_hz() s'applique identiquement a Y, U et V). */
#define SSTV_STREAM_MODE_PD90  2u
#define SSTV_STREAM_PD90_WIDTH  320
#define SSTV_STREAM_PD90_HEIGHT 256

/* PD120 (2026-09-27, RE-AJOUTE) -- avait ete abandonne au profit de PD90 le
 * 2026-09-26 (voir commentaire ci-dessus) parce que son debit de 190 us/
 * pixel revelait un vrai probleme dans le pipeline TIM14 cote radio (voir
 * sstv_tx.c : le thread ne parvenait pas a suivre le rythme de facon
 * fiable, quel que soit le mode -- Martin1 et PD90 en souffraient aussi,
 * juste moins visiblement). Ce probleme de fond a depuis ete corrige par
 * une refonte en file d'attente (queue) cote radio, qui absorbe les
 * irregularites ponctuelles au lieu d'exiger une synchronisation parfaite
 * a chaque pixel -- PD90 en beneficie deja sur le terrain, PD120 devrait
 * desormais fonctionner aussi. Meme famille YUV420 par paire de lignes que
 * PD90, mais 640x496 (image source separee, sstv_master_image_pd120.h,
 * reechantillonnee depuis la meme photo) et 190 us/pixel. */
#define SSTV_STREAM_MODE_PD120  3u
#define SSTV_STREAM_PD120_WIDTH  640
#define SSTV_STREAM_PD120_HEIGHT 496

/* CMD_SSTV_STATUS (radio -> RP2040, sans ACK, envoye juste apres la fin du
 * flux d'image, avant que la radio ne revienne en RX) : {timeout_count:u16,
 * tim_late_nonpixel:u16, tim_late_pixel:u16}, tous little-endian.
 *  - timeout_count : nombre de pixels ou sstv_stream_next_byte() (sstv_tx.c)
 *    a du abandonner l'attente d'un octet neuf du RP2040 (tampon DMA vide
 *    plus de ~5 ms) et rendre un pixel noir a la place.
 *  - tim_late_nonpixel / tim_late_pixel (2026-09-27, scinde en deux --
 *    voir sstv_tx.c, TIM14_IRQHandler -- suite a un retour terrain PD90/120
 *    ou le total unique ne permettait pas de savoir SI le retard venait du
 *    sync/porche (pas de dependance au flux) ou du balayage pixel (lie au
 *    flux RP2040), un correctif d'ordre wait/fetch n'ayant eu AUCUN effet
 *    mesure sur le total ni sur le signal reel) : nombre d'intervalles ou
 *    le thread radio n'avait pas fini de preparer la valeur suivante a
 *    temps pour l'interruption TIM14, respectivement hors balayage pixel
 *    (sync/porche/gap) et pendant le balayage pixel.
 * Idealement 0/0/0 en fonctionnement normal. */
#define CMD_SSTV_STATUS       0x06E3u    /* radio -> RP2040: {timeout_count:u16, tim_late_nonpixel:u16, tim_late_pixel:u16} */

/* ⚠️ (2026-10-03, retour terrain : "decode mais vu l'image envoyee, je ne
 * peux pas dire si c'est ok") -- l'unique image de test embarquee
 * (g_sstv_master_image, une vraie photo) ne permet pas de juger facilement
 * la qualite du decodage (pas de forme nette/connue a verifier). Ajoute une
 * SECONDE image de test, le logo ADRASEC (tools/sstv_gen_image.py, noir et
 * blanc pur -- le moindre artefact de decodage saute aux yeux, sans place
 * pour un jugement de nuance de gris), et un selecteur pour choisir entre
 * les deux : {index:u8}, 0 = photo (g_sstv_master_image), 1 = logo ADRASEC
 * (g_sstv_master_image_adrasec). Persiste cote RP2040 (g_sstv_image_sel,
 * main.c) jusqu'au prochain changement ou redemarrage -- pas besoin de le
 * re-signaler a chaque CMD_SSTV_START. */
#define CMD_SSTV_IMAGE_SELECT 0x06E6u    /* radio -> RP2040: {index:u8} */

/* ⚠️ (2026-10-03, retour terrain : "meme apres exit, il reste sur le
 * streaming" -- le log montre "stream done" n'arrivant qu'apres le DELAI
 * COMPLET normal d'une image (~110 s pour Scottie1), alors que la radio
 * avait annule des le debut) -- CAUSE : sstv_stream_scottie1()/martin1/
 * pd90/pd120 (main.c) sont des boucles bloquantes cadencees en temps reel
 * fixe (sstv_stream_wait(), voir son propre historique) qui ne savent PAS
 * que la touche EXIT, cote radio (sstv_tx.c), a deja coupe l'emission --
 * rien ne leur disait d'arreter plus tot. Pendant tout ce temps restant, le
 * RP2040 continue d'ecrire les octets d'image bruts (non trames) sur la
 * MEME liaison UART que la radio, elle, a deja rebasculee en fonctionnement
 * normal (trames AB CD...DC BA) -- source plausible du "repasse en
 * emission par intermittence" observe, en plus du simple gachis de temps
 * CPU/UART. CMD_SSTV_STOP permet a la radio de dire au RP2040 d'arreter
 * IMMEDIATEMENT, des que la touche EXIT est detectee (sstv_tx.c) -- les 4
 * boucles de streaming le verifient une fois par ligne (meme granularite
 * que le controle EXIT cote radio, pour ne pas perturber le cadencement
 * pixel deja fragile -- voir l'historique de sstv_stream_wait()). */
#define CMD_SSTV_STOP          0x06E7u    /* radio -> RP2040: {} (no payload) */
/* Calibration de l'horloge radio, SANS emission (2026-10-04) :
 *   CMD_SSTV_CLK_START : RP2040 -> radio {} -- la radio demarre son timer TIM14
 *                        et repond CMD_SSTV_CLK_ACK (accuse, sert a la mesure).
 *   CMD_SSTV_CLK_END   : RP2040 -> radio {} -- arret ; la radio repond
 *                        CMD_SSTV_CLK_REPORT {prog_us: u32 LE}.
 *   CMD_SSTV_TRIM      : RP2040 -> radio {trim_ppm: i16 LE} -- stocke en EEPROM.
 * Le RP2040 calcule le trim : prog / (temps reel entre les deux accuses) - 1. */
#define CMD_SSTV_CLK_START     0x06EBu
#define CMD_SSTV_CLK_END       0x06ECu
#define CMD_SSTV_TRIM          0x06E9u
#define CMD_SSTV_CLK_REPORT    0x06EDu
#define CMD_SSTV_CLK_ACK       0x06EEu
/* Menu SSTVCal de la radio (2026-10-04) :
 *   CMD_SSTV_CLK_REQ    : radio -> RP2040 {} -- lancer une mesure, renvoyer le resultat
 *   CMD_SSTV_CLK_RESULT : RP2040 -> radio {trim_ppm: i16 LE} -- proposition */
#define CMD_SSTV_CLK_REQ       0x06F0u
#define CMD_SSTV_CLK_RESULT    0x06EFu

/* ⚠️ (2026-10-03) SSTV RX en direct sur l'ecran radio (branch SSTV_SSDV) :
 * jusqu'ici le RP2040 decodait une image REELLEMENT recue (sstv_demod.c,
 * arme via SSTV_RxActive(), patch/sarsat.c) mais ne faisait qu'un hexdump
 * debug (sstv_service(), main.c) -- aucune image affichee cote radio. Ces
 * deux commandes RP2040 -> radio (sens oppose a CMD_SSTV_START/STATUS
 * ci-dessus, qui servent a l'EMISSION d'une image de test) permettent un
 * affichage PROGRESSIF, ligne d'ecran par ligne d'ecran, sur le LCD de la
 * radio pendant la reception -- meme principe que l'ecran SSTV de F4HWN
 * (uv-k1-k5v3-firmware-custom, feature_update_v6), mais le calcul
 * (reechantillonnage 320->128 colonnes, tramage) reste ici cote RP2040 (sa
 * marge CPU/RAM est bien plus confortable que celle, tres juste, de la
 * radio) -- la radio ne fait plus que dessiner les octets recus.
 *
 * 128x56, pas 128x64 : gFrameBuffer cote radio (driver/st7565.h) ne fait que
 * FRAME_LINES=7 pages (56 px) -- la 8e bande (page 0 du panneau physique)
 * est gStatusLine, la barre de statut (batterie/RSSI), rendue separement et
 * laissee intacte, comme pour tous les autres ecrans de ce projet (SARSAT/
 * APRS/Sonde). 256 lignes Scottie 1 / 56 lignes d'ecran n'est PAS un ratio
 * entier (gcd = 8 : A=32, B=7) -- contrairement a un ratio entier, le
 * nombre de lignes reelles par ligne d'ecran varie (4 ou 5) ; voir
 * sstv_rx_emit_row() (main.c), meme principe d'accumulateur que l'app SSTV
 * de F4HWN pour repartir ses propres lignes sur les 64 rangees d'ecran.
 *
 * CMD_SSTV_RX_CLEAR : envoyee une fois, au tout debut d'une nouvelle image
 * (line_ready_idx == 0, sstv_demod.c) -- efface l'ecran cote radio.
 * CMD_SSTV_RX_ROW : une ligne d'ECRAN (pas une ligne SSTV reelle) : les
 * lignes Scottie 1 reelles du groupe sont moyennees par colonne puis
 * reechantillonnees de 320 a 128 colonnes et tramees (Bayer 4x4) en 1 bit/
 * pixel -- {row:u8, bits[16]} (128 bits = 16 o, MSB en premier par octet =
 * colonne la plus a gauche). */
#define CMD_SSTV_RX_CLEAR      0x06E4u    /* RP2040 -> radio: {} (no payload) */
#define CMD_SSTV_RX_ROW        0x06E5u    /* RP2040 -> radio: {row:u8, bits[16]} */
#define SSTV_RX_SCREEN_WIDTH   128u
#define SSTV_RX_SCREEN_HEIGHT  56u     /* = gFrameBuffer's FRAME_LINES (7) * 8 px */
#define SSTV_RX_ROW_BYTES      (SSTV_RX_SCREEN_WIDTH / 8u)

/* ---- GPS (NMEA in on UART1, C-Board GPS header GP4/GP5) --------------- */
#define CFG_GPS_ENABLE        1
#define CFG_GPS_UART          uart1
#define CFG_GPS_UART_TX_GPIO  4       /* -> GPS RX (unused; module needs no TX) */
#define CFG_GPS_UART_RX_GPIO  5       /* <- GPS TX (NMEA)                       */
#define CFG_GPS_BAUD          9600
#define CFG_GPS_TX_MS         3000    /* push the fix to the radio this often   */
/* mode auto-selects from the radio's reported RX frequency (HELLO reply):     */
#define CFG_APRS_BAND_LO_10HZ  14400000u  /* 144.0 MHz : >= this -> APRS mode   */
#define CFG_APRS_BAND_HI_10HZ  14800000u  /* 148.0 MHz : <= this -> APRS mode   */

#define SARSAT_LVL_OK        0
#define SARSAT_LVL_CLIP      1
#define SARSAT_LVL_BIAS      2
#define SARSAT_LVL_HOT       3
#define SARSAT_LVL_LOW       4

#define SARSAT_LINK_PROTO_VER 1

/* ---- debug logging (USB-CDC console, 115200-agnostic) --------------- */
#define SARSAT_LOG           1       /* 0 = silent                             */
#define CFG_LEVEL_LOG_MS     3000    /* idle "audio level" report period (ms)  */
#define CFG_TX_HEXDUMP       0       /* 1 = also hex-dump every frame sent     */
#define CFG_APRS_RX_DIAG     1       /* 1 = provisional per-frame APRS RX log:  */
                                     /*     every HDLC candidate (OK / FIXED /  */
                                     /*     FCS-BAD / SHORT / DUP) with the     */
                                     /*     source call + a hex head, plus a    */
                                     /*     carrier/level line. Set 0 once the  */
                                     /*     decode issue is understood.         */

#endif /* SARSAT_DECODER_CONFIG_H */
