#ifndef AK_FLIGHT_DSHOT_TIMING_H
#define AK_FLIGHT_DSHOT_TIMING_H

#include <stdint.h>

#include "ak_types.h"

/*
 * Turning DShot frames into timer compare values.
 *
 * DShot is sent as one bit per timer period: the period *is* the bit time, and
 * the compare value decides whether that bit is a '0' or a '1'. The ESC reads
 * the duty cycle, so '1' is a long high time and '0' a short one.
 *
 * This is the pure part of that - a frame in, compare values out - which is why
 * it can be checked on a host. Putting the values into a register at the right
 * instant is the port's job, and that part needs a scope.
 *
 * The layout is what a timer DMA burst wants: four values per bit, one per
 * motor channel, interleaved so that one burst writes all four compare
 * registers. Two blank groups follow the frame, which hold every output low
 * between frames - an ESC sees the gap, not a stretched last bit.
 */

#define AK_DSHOT_BITS      16u
#define AK_DSHOT_GAP_GROUPS 2u
#define AK_DSHOT_GROUPS    (AK_DSHOT_BITS + AK_DSHOT_GAP_GROUPS) /* 18 */
#define AK_DSHOT_ENTRIES   (AK_DSHOT_GROUPS * AK_MAX_MOTORS)

/* The compare values for a '0' bit and a '1' bit, from the timer period and
 * the duty those bits are supposed to have.
 *
 * DShot says 37.5% for a '0' and 75% for a '1'. The reference implementation
 * this was checked against uses 35% and 70% of the period, and the difference
 * does not matter - what matters is that the two are far apart and both
 * unambiguous to an ESC, which samples around the middle of the bit. */
#define AK_DSHOT_DUTY_ZERO_NUM 35u /* percent */
#define AK_DSHOT_DUTY_ONE_NUM  70u

uint16_t ak_dshot_ccr_zero(uint16_t period);
uint16_t ak_dshot_ccr_one(uint16_t period);

/* Fills `entries` (AK_DSHOT_ENTRIES values) from a DShot frame per motor,
 * most significant bit first, then the blank groups. */
void ak_dshot_fill(uint16_t *entries, const uint16_t frames[AK_MAX_MOTORS],
                   uint16_t ccr_zero, uint16_t ccr_one);

/*
 * And the same frame for a transmitter that is told *when to change* rather
 * than how long to stay high: a list of levels and durations. The ESP32's RMT
 * is one, and the reason this is in the core rather than in that port is the
 * same as for the compare values above - the timing is arithmetic, and the
 * arithmetic can be checked on a host.
 *
 * Two items per bit: the high time (35% or 70% of the bit, the same duties the
 * timer encoder uses, so a scope sees the same waveform whichever target is
 * flying), then the low that finishes the bit. One more item follows, a low of
 * two bit times, which is the gap an ESC uses to see where one frame ends and
 * the next begins - the same two blank groups the timer path inserts.
 */
typedef struct {
    uint8_t  level; /* 1 for high, 0 for low */
    uint16_t ticks;
} ak_dshot_edge_t;

#define AK_DSHOT_EDGES (AK_DSHOT_BITS * 2u + 1u) /* 33 */

/* Fills `out` (AK_DSHOT_EDGES items). With no ticks per bit - a rate this
 * firmware does not speak, or a tick longer than the bit - every item is a
 * zero-length low, which transmits nothing rather than a wrong frame. */
void ak_dshot_edges(uint16_t frame, uint16_t ticks_per_bit,
                    ak_dshot_edge_t *out);

/* Whole ticks in one DShot bit for a transmitter whose tick is `tick_ns`
 * nanoseconds: DShot150 is 6667 ns a bit, 300 is 3333 and 600 is 1667, and a
 * tick that does not divide them is rounded to the nearest - 17 ticks of 100
 * ns for DShot600, which is 0.2% fast and inside what an ESC accepts. Returns
 * 0 for a rate DShot does not have. */
uint16_t ak_dshot_ticks_per_bit(uint32_t khz, uint32_t tick_ns);

#endif /* AK_FLIGHT_DSHOT_TIMING_H */
