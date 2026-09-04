/*
 * audio_slicer.c — see audio_slicer.h.
 *
 * Algorithm and constants are those of F4EHY's dec406_v7 / moricef's
 * capture_trame(); only the arithmetic type and the I/O plumbing changed.
 */
#include "audio_slicer.h"
#include <string.h>

#define BAUDS        400
#define BITSYNC_ONES 15
#define FRAME_LONG   144
#define FRAME_SHORT  112

/* ------------------------------------------------------------------------- */
/* Fixed-point implementation (default; RP2040-safe).                         */
/*                                                                           */
/* Let M = 2*Nb. The upstream computes                                        */
/*     Ymoy = (1/M) * sum(Y)                                                  */
/*     Y1   = sum_i (Y[a_i]-Ymoy) * (Y[b_i]-Ymoy)          (i = 0..Nb-1)      */
/* Here we keep Ysum (int64) and evaluate the scaled quantity                 */
/*     Y1s  = sum_i (M*Y[a_i]-Ysum) * (M*Y[b_i]-Ysum)  ==  Y1 * M*M           */
/* Every threshold is scaled by the same constant M*M, so all comparisons     */
/* are identical to the float version (bar the float's own rounding).         */
/* ------------------------------------------------------------------------- */
int slicer_run_fixed(const int32_t *samples, int n, int rate, uint8_t *bits_out)
{
    const int Nb = rate / BAUDS;
    if (Nb < 12 || Nb > 120) return 0;
    const int M  = 2 * Nb;
    const int64_t MM = (int64_t)M * M;

    int32_t Y[2 * 120];
    memset(Y, 0, sizeof(Y));

    char s[160];
    for (int i = 0; i < 160; i++) s[i] = '-';

    int depart = 0, numBit = 0, cpte = 0, synchro = 0, Nb15;
    char etat = '-';
    int longueur_trame = FRAME_LONG;

    const int64_t coeff = 100;
    int64_t max_s   = (int64_t)10000 * MM;
    int64_t min_s   = -max_s;
    int64_t seuil1  = max_s / coeff;
    int64_t seuil0  = min_s / coeff;

    int k = 0;

    for (int idx = 0; idx < n && numBit < longueur_trame; idx++) {
        k = (k + 1) % M;
        Y[k] = samples[idx];

        int64_t Ysum = 0;
        for (int i = 0; i < M; i++) Ysum += Y[i];

        int64_t Y1s = 0;
        for (int i = 0; i < Nb; i++) {
            int a = (k + i) % M;
            int b = (k + i + Nb) % M;
            int64_t da = (int64_t)M * Y[a] - Ysum;
            int64_t db = (int64_t)M * Y[b] - Ysum;
            Y1s += da * db;
        }

        if (Y1s > max_s) { max_s = Y1s; seuil1 = max_s / coeff; }
        if (Y1s < min_s) { min_s = Y1s; seuil0 = min_s / coeff; }

        if (synchro == 0) {
            if (depart == 0) {
                if (Y1s > seuil1) depart = 1;
                cpte = 0;
            } else {
                cpte++;
                if (Y1s < seuil0) {
                    Nb15 = cpte / Nb;
                    if (Nb15 >= 12 && Nb15 <= 18) {
                        synchro = 1;
                        cpte = 0;
                        for (int i = 0; i < BITSYNC_ONES; i++) s[i] = '1';
                        numBit = BITSYNC_ONES;
                        etat = '0';
                    } else {
                        cpte = 0; depart = 0; synchro = 0;
                        etat = '-'; numBit = 0;
                    }
                }
            }
        } else {
            cpte++;

            if (s[24] == '0') longueur_trame = FRAME_SHORT;

            if (Y1s > seuil1) {
                if (etat == '0') {
                    etat = '1';
                    cpte -= Nb / 2;
                    while (cpte > 0 && numBit < longueur_trame) {
                        s[numBit] = (s[numBit - 1] == '1') ? '0' : '1';
                        numBit++;
                        cpte -= Nb;
                    }
                    cpte = 0;
                }
            } else if (Y1s < seuil0) {
                if (etat == '1') {
                    etat = '0';
                    cpte -= Nb / 2;
                    while (cpte > 0 && numBit < 149) {
                        s[numBit] = s[numBit - 1];
                        numBit++;
                        cpte -= Nb;
                    }
                    cpte = 0;
                }
            }
        }
    }

    if (numBit >= longueur_trame) {
        for (int i = 0; i < longueur_trame; i++)
            bits_out[i] = (s[i] == '1') ? 1 : 0;
        return longueur_trame;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Double reference — a faithful transcription of moricef capture_trame(),    */
/* kept for host-side A/B validation only.                                    */
/* ------------------------------------------------------------------------- */
int slicer_run_double(const int32_t *samples, int n, int rate, uint8_t *bits_out)
{
    const int Nb = rate / BAUDS;
    if (Nb < 12 || Nb > 120) return 0;
    const int M = 2 * Nb;

    double Y[2 * 120];
    for (int i = 0; i < M; i++) Y[i] = 0.0;

    char s[160];
    for (int i = 0; i < 160; i++) s[i] = '-';

    int depart = 0, numBit = 0, cpte = 0, synchro = 0, Nb15;
    char etat = '-';
    int longueur_trame = FRAME_LONG;

    const double coeff = 100.0;
    double max = 10e3, min = -10e3;
    double seuil1 = max / coeff, seuil0 = min / coeff;

    int k = 0;

    for (int idx = 0; idx < n && numBit < longueur_trame; idx++) {
        k = (k + 1) % M;
        Y[k] = (double)samples[idx];

        double Ymoy = 0.0;
        for (int i = 0; i < M; i++) Ymoy += Y[i];
        Ymoy /= M;

        double Y1 = 0.0;
        for (int i = 0; i < Nb; i++) {
            int j = (k + i + Nb) % M;
            Y1 += (Y[(k + i) % M] - Ymoy) * (Y[j] - Ymoy);
        }

        if (Y1 > max) { max = Y1; seuil1 = max / coeff; }
        if (Y1 < min) { min = Y1; seuil0 = min / coeff; }

        if (synchro == 0) {
            if (depart == 0) {
                if (Y1 > seuil1) depart = 1;
                cpte = 0;
            } else {
                cpte++;
                if (Y1 < seuil0) {
                    Nb15 = cpte / Nb;
                    if (Nb15 >= 12 && Nb15 <= 18) {
                        synchro = 1;
                        cpte = 0;
                        for (int i = 0; i < BITSYNC_ONES; i++) s[i] = '1';
                        numBit = BITSYNC_ONES;
                        etat = '0';
                    } else {
                        cpte = 0; depart = 0; synchro = 0;
                        etat = '-'; numBit = 0;
                    }
                }
            }
        } else {
            cpte++;
            if (s[24] == '0') longueur_trame = FRAME_SHORT;

            if (Y1 > seuil1) {
                if (etat == '0') {
                    etat = '1';
                    cpte -= Nb / 2;
                    while (cpte > 0 && numBit < longueur_trame) {
                        s[numBit] = (s[numBit - 1] == '1') ? '0' : '1';
                        numBit++;
                        cpte -= Nb;
                    }
                    cpte = 0;
                }
            } else if (Y1 < seuil0) {
                if (etat == '1') {
                    etat = '0';
                    cpte -= Nb / 2;
                    while (cpte > 0 && numBit < 149) {
                        s[numBit] = s[numBit - 1];
                        numBit++;
                        cpte -= Nb;
                    }
                    cpte = 0;
                }
            }
        }
    }

    if (numBit >= longueur_trame) {
        for (int i = 0; i < longueur_trame; i++)
            bits_out[i] = (s[i] == '1') ? 1 : 0;
        return longueur_trame;
    }
    return 0;
}

int slicer_run(const int32_t *samples, int n, int rate, uint8_t *bits_out)
{
#if defined(SARSAT_SLICER_DOUBLE)
    return slicer_run_double(samples, n, rate, bits_out);
#else
    return slicer_run_fixed(samples, n, rate, bits_out);
#endif
}
