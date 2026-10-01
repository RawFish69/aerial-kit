#include "ak_ring.h"

void ak_ring_init(ak_ring_t *ring)
{
    ring->head = 0;
    ring->tail = 0;
    ring->dropped = 0;
    ring->pushed = 0;
}

void ak_ring_push(ak_ring_t *ring, uint8_t byte)
{
    uint16_t next = (uint16_t)((ring->head + 1u) % sizeof ring->data);
    if (next == ring->tail) {
        ring->dropped++;
        return;
    }
    ring->data[ring->head] = byte;
    ring->head = next;
    ring->pushed++;
}

int ak_ring_pop(ak_ring_t *ring, uint8_t *byte)
{
    if (ring->tail == ring->head) {
        return 0;
    }
    *byte = ring->data[ring->tail];
    ring->tail = (uint16_t)((ring->tail + 1u) % sizeof ring->data);
    return 1;
}

uint16_t ak_ring_count(const ak_ring_t *ring)
{
    uint16_t head = ring->head;
    uint16_t tail = ring->tail;
    return head >= tail ? (uint16_t)(head - tail)
                        : (uint16_t)(sizeof ring->data - tail + head);
}
