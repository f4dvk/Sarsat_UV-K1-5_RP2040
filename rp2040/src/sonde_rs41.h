/*
 * sonde_rs41.h — Vaisala RS41 radiosonde frame layer (400-406 MHz, GFSK
 * 4800 baud). Part of the Sarsat_UV-K1-5_RP2040 radiosonde-hunting feature
 * (docs/protocol.md, "Radiosondes").
 *
 * Scope of this file: everything AFTER bit recovery -- byte-level frame
 * sync, de-whitening, per-block CRC, and the GPS (ECEF) block, plus a
 * standalone ECEF->geodetic (WGS84) conversion. The sample-level GFSK
 * demodulator (ADC samples -> bits) lives elsewhere (sonde_demod.*) and is
 * shared with the other radiosonde types where practical.
 *
 * Originally reimplemented from PUBLIC PROTOCOL DOCUMENTATION ONLY
 * (github.com/bazjo/RS41_Decoding and sigidwiki.com), before this project
 * relicensed from Apache-2.0 to GPL-3.0 on 2026-09-11 (explicit user
 * decision, see CREDITS.md) specifically to allow porting the DFM and
 * M10/M20 decoders from the reference C decoders in
 * github.com/projecthorus/radiosonde_auto_rx (GPL-3.0, author zilog80).
 * That gave the opportunity to CROSS-CHECK this file's earlier from-docs
 * guesses against `demod/mod/rs41mod.c`: the sync word, the 64-byte XOR
 * whitening mask, and the GPSPOS block's ECEF position/velocity field
 * layout all matched exactly (confirming the public write-up was right on
 * those points) -- but the CRC-16 variant guess (CRC-16/KERMIT) was WRONG;
 * the real one is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, MSB-first,
 * not reflected) -- see rs41_crc16()'s comment. This is almost certainly
 * why this project's one attempt to validate against a real RS41 capture
 * never got a block to CRC-check (see the radiosonde plan notes).
 *
 * ⚠️ Still not validated end-to-end against a real capture with the fixed
 * CRC in this session -- the sync/mask/GPS-layout cross-check above is a
 * strong signal this file is now correct, but an on-air pass (same spirit
 * as every other "not yet confirmed on air" feature in this project, see
 * integration.md's history) is the next real test.
 */
#ifndef SONDE_RS41_H
#define SONDE_RS41_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define RS41_MASK_LEN      64   /* whitening mask period, in bytes           */
#define RS41_SYNC_LEN       8   /* frame sync word length, in bytes          */
#define RS41_MAX_BLOCK_LEN 255  /* block "length" byte is 1 byte             */
#define RS41_GPSPOS_ID   0x7B   /* block ID: ECEF position + velocity        */

/* The 8-byte frame sync, RAW as transmitted (i.e. already XORed with the
 * whitening mask below) -- this is what a byte-level correlator should
 * search for directly in the received (still-scrambled) byte stream, before
 * paying the cost of de-whitening anything. Descrambled, this reads
 * 10 B6 CA 11 22 96 12 F8 (matches the RS41_MASK[0..7] XOR of the bytes
 * below -- the two independent sources cross-check). */
extern const uint8_t RS41_RAW_SYNC[RS41_SYNC_LEN];

/* 64-byte repeating XOR whitening mask (data starts "in phase" with mask[0]
 * right after RS41_RAW_SYNC, i.e. the byte immediately following the sync
 * word XORs with mask[0]). Public/well-known constant of the protocol
 * (published identically by multiple independent RS41 reverse-engineering
 * write-ups) -- not itself copyrightable code. */
extern const uint8_t RS41_MASK[RS41_MASK_LEN];

/* De-whiten `len` bytes in place. `mask_phase` is the whitening-mask index
 * (0..63) that applies to data[0] -- pass 0 for the byte right after the
 * sync word, or (running_offset % 64) for a later chunk of the same frame. */
void rs41_descramble(uint8_t *data, int len, int mask_phase);

/* Correlate `raw` (still scrambled, NOT bit-sliced audio -- already-recovered
 * bytes) against RS41_RAW_SYNC. Tolerates up to `max_bit_errors` mismatched
 * bits (a noisy front end will not give a byte-perfect sync every time).
 * Returns the byte offset of the sync word's first byte, or -1. */
int rs41_find_sync(const uint8_t *raw, int len, int max_bit_errors);

/* Block CRC-16. ⚠️ Variant not confirmed on air -- implemented as CRC-16/CCITT
 * (poly 0x1021), reflected in/out, init 0x0000 (the "KERMIT" parameterisation)
 * as the project's best guess; swap this one function if frames never
 * validate on real hardware. */
uint16_t rs41_crc16(const uint8_t *data, int len);

typedef struct {
    uint8_t        id;
    uint8_t        len;      /* length of `data`, excluding the 2-byte CRC   */
    const uint8_t *data;     /* points into the caller's (descrambled) frame */
} rs41_block_t;

/* Walk a de-whitened frame buffer (everything after the sync word) into
 * [id:u8, len:u8, data[len], crc16:u16 LE] blocks, stopping at the first CRC
 * mismatch or when too little data remains for another block header. Returns
 * the number of valid blocks found (<= max_blocks). */
int rs41_parse_blocks(const uint8_t *frame, int frame_len,
                      rs41_block_t *out, int max_blocks);

typedef struct {
    int32_t ecef_x_cm, ecef_y_cm, ecef_z_cm;
    int16_t ecef_vx_cms, ecef_vy_cms, ecef_vz_cms;
} rs41_gpspos_t;

/* Parse a 0x7B (GPSPOS) block's payload (18 bytes: 3x int32 LE position in
 * cm, 3x int16 LE velocity in cm/s -- per bazjo/RS41_Decoding's RS41-SGP
 * block table). Returns false if `len` is too short. */
bool rs41_parse_gpspos(const uint8_t *data, int len, rs41_gpspos_t *out);

/* WGS84 ECEF (metres) -> geodetic lat/lon (degrees x 1e5, same convention as
 * the rest of this project's APRS position fields) + altitude (metres).
 * Bowring's closed-form approximation, double precision (this runs once per
 * decoded frame, ~1/s -- soft-float cost is a non-issue at that rate).
 * Pure geometry, independent of anything RS41-specific -- fully
 * self-testable (round-trip against a forward geodetic->ECEF projection)
 * regardless of whether the frame-layer guesses above are ever confirmed. */
void rs41_ecef_to_geo(double x_m, double y_m, double z_m,
                     int32_t *lat_e5, int32_t *lon_e5, int32_t *alt_m);

#endif /* SONDE_RS41_H */
