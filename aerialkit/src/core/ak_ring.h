#ifndef AK_CORE_AK_RING_H
#define AK_CORE_AK_RING_H

#include <stdint.h>

/*
 * A byte ring buffer for interrupt-driven receive.
 *
 * An interrupt writes, the main loop reads, and neither waits for the other. A
 * full buffer drops the newest byte and counts it rather than blocking the
 * interrupt or overwriting data the reader has not seen yet - a dropped byte in
 * a CRSF stream is a frame the crc throws away, which is exactly the failure
 * that should be cheap.
 *
 * Kept free of hardware so it can be tested where a dropped byte is easy to
 * provoke, which on a board it is not.
 */

typedef struct {
    uint8_t  data[256];
    uint16_t head;    /* written by the producer */
    uint16_t tail;    /* read by the consumer */
    uint32_t dropped;
    uint32_t pushed;
} ak_ring_t;

void ak_ring_init(ak_ring_t *ring);

/* Producer side, safe from an interrupt: never blocks, never overwrites. */
void ak_ring_push(ak_ring_t *ring, uint8_t byte);

/* Consumer side: 1 and the byte when there was one, 0 when empty. */
int ak_ring_pop(ak_ring_t *ring, uint8_t *byte);

uint16_t ak_ring_count(const ak_ring_t *ring);

#endif /* AK_CORE_AK_RING_H */
