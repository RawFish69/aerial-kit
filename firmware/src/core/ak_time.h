#ifndef AK_CORE_AK_TIME_H
#define AK_CORE_AK_TIME_H

#include <stdint.h>

/* Millisecond time base, from a board tick. Wraps at 2^32 ms (~49 days). */
void ak_time_init(void);
uint32_t ak_time_ms(void);

/* Cooperative delay: does not sleep the core, does not depend on anything but
 * the tick. Good enough until the scheduler milestone.
 *
 * It *gives up* rather than waiting for ever on a tick that is not moving, and
 * counts each time it does: a delay is the one wait in this firmware that had
 * no bound, and an unbounded wait at boot is a board that lights its LED and
 * then says nothing - which is what the F405 did on the bench on 2026-09-17.
 * `ak_delay_stalls()` is printed by the boot report when it is not zero. */
void ak_delay_ms(uint32_t ms);
uint32_t ak_delay_stalls(void);

/* The guard on its own, with the clock handed in, so a host test can drive a
 * tick that never moves without waiting for one. Returns the number of rounds
 * it took and sets *gave_up when the limit was reached first. */
unsigned ak_wait_for_ticks(uint32_t start, uint32_t ms, uint32_t limit,
                           uint32_t (*now)(void), int *gave_up);

#endif /* AK_CORE_AK_TIME_H */
