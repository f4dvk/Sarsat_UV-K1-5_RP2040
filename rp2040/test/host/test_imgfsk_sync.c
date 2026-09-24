/* Host test for imgfsk_sync.c (branch SSTV_SSDV) -- same shape as
 * test_sonde_sync.c: feed a synthetic bitstream (preamble + sync word +
 * a known 256-byte packet, MSB-first per byte) and check the hunter locks
 * exactly once and hands back the packet bytes unchanged. */
#include "imgfsk_sync.h"

#include <stdio.h>
#include <string.h>

static int g_fail;

#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s\n", msg); g_fail = 1; } \
} while (0)

static void feed_byte(imgfsk_sync_t *s, uint8_t b, bool *locked, uint8_t *out_pkt, int *n_locks)
{
    for (int i = 7; i >= 0; i--) {
        if (imgfsk_sync_feed(s, (b >> i) & 1u)) {
            *locked = true;
            (*n_locks)++;
            memcpy(out_pkt, s->frame, IMGFSK_PACKET_SIZE);
        }
    }
}

int main(void)
{
    imgfsk_sync_t s;
    imgfsk_sync_init(&s);

    uint8_t pkt[IMGFSK_PACKET_SIZE];
    for (int i = 0; i < IMGFSK_PACKET_SIZE; i++)
        pkt[i] = (uint8_t)(0x55 + i * 37);   /* arbitrary but distinctive */

    bool locked = false;
    uint8_t captured[IMGFSK_PACKET_SIZE];
    int n_locks = 0;

    /* preamble: 7 bytes of 0xAA (must NOT trigger a lock) */
    for (int i = 0; i < 7; i++)
        feed_byte(&s, 0xAA, &locked, captured, &n_locks);
    CHECK(!locked, "preamble alone must not lock");

    /* sync word, MSB-first per byte (see imgfsk_sync.h's bit-order caveat) */
    feed_byte(&s, 0x85, &locked, captured, &n_locks);
    feed_byte(&s, 0xCF, &locked, captured, &n_locks);
    feed_byte(&s, 0xAB, &locked, captured, &n_locks);
    feed_byte(&s, 0x45, &locked, captured, &n_locks);
    CHECK(!locked, "sync word alone (before the packet) must not yet report a full packet");

    for (int i = 0; i < IMGFSK_PACKET_SIZE; i++)
        feed_byte(&s, pkt[i], &locked, captured, &n_locks);

    CHECK(locked, "expected exactly one packet lock after sync + 256 bytes");
    CHECK(n_locks == 1, "expected exactly one lock event, not more/less");
    CHECK(memcmp(captured, pkt, IMGFSK_PACKET_SIZE) == 0,
          "captured packet bytes must match the transmitted ones exactly");

    /* a second, independently-framed packet right after (matches how
     * patch/imgfsk_tx.c re-syncs per packet on the real link) */
    bool locked2 = false;
    uint8_t captured2[IMGFSK_PACKET_SIZE];
    int n_locks2 = 0;
    uint8_t pkt2[IMGFSK_PACKET_SIZE];
    for (int i = 0; i < IMGFSK_PACKET_SIZE; i++)
        pkt2[i] = (uint8_t)(0xC3 - i);

    for (int i = 0; i < 7; i++)
        feed_byte(&s, 0xAA, &locked2, captured2, &n_locks2);
    feed_byte(&s, 0x85, &locked2, captured2, &n_locks2);
    feed_byte(&s, 0xCF, &locked2, captured2, &n_locks2);
    feed_byte(&s, 0xAB, &locked2, captured2, &n_locks2);
    feed_byte(&s, 0x45, &locked2, captured2, &n_locks2);
    for (int i = 0; i < IMGFSK_PACKET_SIZE; i++)
        feed_byte(&s, pkt2[i], &locked2, captured2, &n_locks2);

    CHECK(locked2, "second, independently-framed packet must also lock");
    CHECK(memcmp(captured2, pkt2, IMGFSK_PACKET_SIZE) == 0,
          "second packet's bytes must match too");

    if (!g_fail)
        printf("OK: imgfsk_sync locks on sync+256B and captures bytes unchanged, "
               "twice in a row, no false lock on the preamble alone\n");
    return g_fail;
}
