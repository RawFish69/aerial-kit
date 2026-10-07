#ifndef AK_CORE_AK_TIME_H
#define AK_CORE_AK_TIME_H

#include <stdint.h>

/* Millisecond time base, from a board tick. Wraps at 2^32 ms (~49 days). */
void ak_time_init(void);
uint32_t ak_time_ms(void);

/*
 * The same time base at microsecond resolution, for the scheduler.
 *
 * A millisecond tick cannot express the thing the fast loop is judged by: the
 * phase 1 acceptance test asks for a p99 *jitter* under five microseconds, and
 * one tick is two hundred times that. `ak_cycles()` is the other sub-
 * millisecond source in this tree and it is the wrong one here - it is 32 bits
 * at the core clock, so it wraps every 25.6 seconds at 168 MHz, and on the
 * ESP32 it is not a time base at all (`ak_cycles_per_us()` is zero there by
 * design, because that chip's clock moves). This is a *time* rather than a
 * cycle count: it is what the scheduler puts on a deadline, and every port can
 * answer it, including the one that cannot answer the cycle counter.
 *
 * Three properties, and the first is the one to keep in mind:
 *
 * - **It wraps every ~71.6 minutes**, which is far shorter than the millisecond
 *   clock's 49 days. Comparisons across the wrap are the same rule as
 *   everywhere else in this tree: unsigned subtraction, `(uint32_t)(a - b)`,
 *   never `<`. A deadline more than 71 minutes in the past or future is not
 *   representable, which is why the scheduler re-arms from *now* rather than
 *   accumulating a chain of deadlines that a wrap would fold together.
 * - **It is monotonic within a wrap**, and a caller may not assume it is
 *   *exact*: a port derives it from whatever counter it has, so the resolution
 *   is the port's. It is never coarser than the millisecond clock.
 * - **It carries the flight path, and it is still not a safety one.** Every IMU
 *   sample carries a microsecond stamp and the flight core derives its
 *   intervals from the stamps rather than from a nominal period, which is what
 *   lets the loop run at four kilohertz: a millisecond `dt` cannot express a
 *   250 us period at all, it can only be nought or one. What stays out of this
 *   clock is every *threshold*. No deadline, no timeout and no arm gate is
 *   expressed in microseconds, because 71.6 minutes of range is not enough for
 *   one and the millisecond clock is the one with the range - the receiver
 *   timeout, the arm hold and the configuration ABI's age bounds are all
 *   millisecond rules, and `ak_flight_step()` is *handed* the millisecond
 *   reading for them as a second argument rather than dividing this one down
 *   to it. The reason is the range difference stated above: `ak_time_us() /
 *   1000` is the millisecond reading only until this counter wraps, and past
 *   that it is the millisecond clock modulo 71.6 minutes - so a millisecond
 *   deadline computed from it fires every loop after the wrap. A caller reads
 *   both from the same counter and passes both.
 */
uint32_t ak_time_us(void);

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

/*
 * The cycle counter: the time base underneath the millisecond tick.
 *
 * A tick cannot resolve the work inside one iteration of a one-millisecond
 * loop, which is why doc 29 could say what the loop's period was and nothing
 * about its duration. This is the port's job because the register is: the M4
 * ports read DWT->CYCCNT, and the host build reads a model a test drives, so
 * `src/core/ak_perf.c` gets to do the arithmetic without knowing which.
 *
 * Three properties a caller has to know, and none of them is optional:
 *
 * - **It is free-running and 32 bits.** At 168 MHz it wraps every 25.6
 *   seconds. Two stamps are only comparable by unsigned subtraction,
 *   `(uint32_t)(now - then)`; comparing them with `<` is wrong across a wrap.
 * - **It may not be running.** `ak_clk_sysclk_hz()` is what turns cycles into
 *   microseconds, and a port that cannot say (0) leaves the durations
 *   unmeasured rather than guessed. `ak_cycles_per_us()` returning 0 is the
 *   signal for that, and ak_perf reports nothing rather than a fabricated rate.
 * - **It is a development instrument, not a safety one.** Nothing in the
 *   flight path may branch on it; it measures the control law, it does not
 *   take part in it.
 */
void     ak_cycles_init(void);
uint32_t ak_cycles(void);
uint32_t ak_cycles_per_us(void);

#endif /* AK_CORE_AK_TIME_H */
