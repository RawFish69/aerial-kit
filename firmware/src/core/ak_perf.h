#ifndef AK_CORE_AK_PERF_H
#define AK_CORE_AK_PERF_H

#include <stdint.h>

/*
 * Where the loop's time actually goes.
 *
 * doc 29 could say what the loop's *period* was and nothing about its
 * *duration*: `long_loops` and `max_loop_ms` come from the millisecond tick and
 * a tick cannot resolve the work inside one iteration. This is the other half -
 * a cycle counter, a stamp at each boundary, and the arithmetic that turns the
 * two into numbers a person can act on.
 *
 * The shape is a *sequence of phases*, not a set of begin/end pairs, and that
 * is the load-bearing decision. A loop is one pass through a fixed order of
 * work, so what has to be recorded is "the estimator began here", and the
 * profiler knows what that means: the section that was open has just ended.
 * Pairs of calls would instead let a caller open one bracket and forget it,
 * which on a one-millisecond loop is a section that appears to have run for
 * the rest of the mission - a wrong number that looks like a plausible one, and
 * the reason `ak_perf_loop_end` closes whatever is open rather than trusting a
 * caller to have done it.
 *
 * Three more things about the design are deliberate and are the ones to keep:
 *
 * 1. **Nothing here reads a register.** The counter is the port's
 *    (`ak_cycles()` in ak_time.h, DWT on the M4 ports, a driven model on the
 *    host), and this file only subtracts. Ground rule 7: `src/core/` stays
 *    MCU-free, which is what lets the host tests drive exact cycle counts
 *    through the same code the firmware runs.
 *
 * 2. **Every interval is an unsigned subtraction.** The counter is 32 bits and
 *    free-running: at 168 MHz it wraps every 25.6 seconds. Comparing two stamps
 *    (`a < b`) is wrong across a wrap; `(uint32_t)(a - b)` is right as long as
 *    no single interval is longer than 2^32 cycles, and that is the one
 *    condition stated rather than assumed - the tick is still what says a loop
 *    was late by a *millisecond*, and this only says what the work cost.
 *
 * 3. **The load figure is a lower bound and says so.** It counts the sections
 *    that were instrumented, and the firmware does work outside them (the
 *    console, the links, the navigation). A percentage that quietly omitted
 *    that would be the kind of claim doc 29 exists to prevent.
 *
 * The flight core is instrumented through this file without depending on it:
 * `ak_flight_phase_hook()` (ak_flight.h) takes a callback and the firmware
 * passes one that calls ak_perf_phase(). The core stays free of the profiler
 * for the same reason it stays free of the clock - it is linked into the
 * Python ABI's shared library, where none of this exists.
 */

/* The phases a loop is divided into, in the order the report prints them.
 * These are the five the roadmap's acceptance test names.
 *
 * AK_PERF_NONE is a real value and not a sentinel for a bug: it is the state
 * between sections, where the loop is doing work that is not attributed to any
 * of them - the console, the links, the navigation. Opening it closes the
 * previous section and charges the time to nobody, which is the honest answer
 * for work that has not been divided up. */
typedef enum {
    AK_PERF_NONE = 0,
    AK_PERF_IMU,
    AK_PERF_ESTIMATOR,
    AK_PERF_PID,
    AK_PERF_MIXER,
    AK_PERF_OUTPUT,
    AK_PERF_SECTIONS   /* one past the last: not a phase */
} ak_perf_section_t;

/* How many sections have a name: everything but AK_PERF_NONE, which is the
 * state *between* them and is not reported anywhere.
 *
 * This is the number the wire's AK_PROTO_PERF_SECTIONS copies, and a host test
 * asserts the two agree - so a section added here and forgotten there is a
 * failing build rather than a reply that is one field short. It is a define
 * rather than `AK_PERF_SECTIONS - 1` at each site so that the arithmetic lives
 * in one place and the enum stays free to gain a value before SECTIONS. */
#define AK_PERF_NAMED_SECTIONS ((unsigned)AK_PERF_SECTIONS - 1u)

const char *ak_perf_section_name(ak_perf_section_t section);

/*
 * The jitter histogram's resolution and span.
 *
 * Jitter is |period - nominal| in microseconds, one bin per microsecond up to
 * AK_PERF_JITTER_BINS - 1, with everything beyond that in the last bin and
 * counted separately. The range stops there because a 1 kHz loop with more than
 * 255 us of jitter is not being measured, it is being diagnosed, and
 * `period_max_us` is the field for that.
 *
 * The bins are 32-bit, which is 1 KB, and that is a deliberate size rather than
 * an accident: 16-bit bins would saturate after 65 535 loops, which is 65
 * seconds at 1 kHz, and a p99 that quietly stopped counting is worse than no
 * p99 at all.
 *
 * A cumulative histogram rather than a ring of recent samples, because "p99"
 * over a window nobody defined is not a number. This one covers every loop
 * since the last reset, which is a unit that can be stated.
 */
#define AK_PERF_JITTER_BINS 256

typedef struct {
    /* Whether the profiler has been started at all. A caller that reports
     * these numbers without checking this would print a window of zeros -
     * "the loop is costing nothing" - when the truth is that nothing is being
     * measured. Those are different sentences and only one of them is a reason
     * to stop looking. */
    int      started;
    uint32_t loops;             /* periods closed since reset */
    /* How many of those the port could actually measure, which is a different
     * number and is reported separately rather than quietly folded in.
     *
     * A port whose clock reads zero can still say a period closed - that needs
     * no clock - so `loops` keeps counting while every timed figure below stops
     * moving. Without this field a reader comparing `loops` against a period
     * would have no way to tell a profiler that was measuring from one that was
     * only counting, and the two look identical from the outside.
     *
     * On a port that can measure, the two are equal, which is the normal case
     * and the one the console prints. */
    uint32_t samples;
    uint32_t nominal_us;        /* the period the loop is trying to hold */

    uint32_t period_last_us;
    uint32_t period_min_us;
    uint32_t period_max_us;
    uint32_t late;              /* periods longer than nominal + slack */

    /* Across every loop since the last reset, from the histogram above. */
    uint16_t jitter_p50_us;
    uint16_t jitter_p99_us;
    uint16_t jitter_max_us;
    uint32_t jitter_over;       /* loops past the last bin: a *count*, not a clipped value */

    /* Work per section, averaged over every loop since reset. */
    uint32_t section_avg_ns[AK_PERF_SECTIONS];
    uint32_t section_max_us[AK_PERF_SECTIONS];

    /* Sum of the instrumented sections over the nominal slot, in per-mille.
     * A lower bound - see the header comment. */
    uint32_t load_permille;
} ak_perf_snapshot_t;

/* Zero every counter and start the cycle counter. Safe to call again. */
void ak_perf_init(void);

/*
 * Tell the profiler the period the loop is now trying to hold, in
 * microseconds - phase 1.4's `gyro_rate_hz` over `pid_denom`.
 *
 * Until 1.4 there was nothing to tell: the loop was a millisecond, the constant
 * said so, and `ak_perf_init` read it. Once the rate is a parameter, a
 * profiler still measuring jitter against one millisecond would report every
 * period of a four-kilohertz loop as late - a profiler that is wrong about the
 * aircraft's rate is worse than one that is not measuring, because its numbers
 * look like a finding.
 *
 * `load_permille` and the jitter are both derived from this, so it is the one
 * number to change. Zero is refused - "the loop is trying to hold zero
 * microseconds" is not a period - and leaves the caller's value alone.
 */
void ak_perf_set_nominal_us(uint32_t nominal_us);

/* The loop's own bracket. `start` is what ak_cycles() returned at the top.
 * Closing a period is what feeds the histogram, so a loop that is never closed
 * contributes nothing rather than contributing a zero. Opening a period closes
 * any section that was left open, so a period cannot begin inside one. */
void ak_perf_loop_begin(uint32_t start_cycles);

/* Move to a phase: the section that is open ends, and `phase` begins. Called
 * from the loop, and from inside the flight step through
 * `ak_flight_phase_hook()`.
 *
 * Ignored when no period is open, which is the rule that keeps a section from
 * being averaged into a loop it did not belong to. */
void ak_perf_phase(ak_perf_section_t phase);

/* Close the open section and the period. Safe to call when nothing is open. */
void ak_perf_loop_end(void);

/* What the console and the protocol read.
 *
 * The cycles are converted to microseconds with the port's clock *as it is at
 * this call*, so a port whose clock changed mid-window would have its earlier
 * cycles converted at the later rate. Neither port does that - the clock is
 * fixed at boot - and saying so is cheaper than guarding it. */
void ak_perf_snapshot(ak_perf_snapshot_t *out);

/* Forget the percentiles, the maxima and the averages without stopping the
 * counter: what a person does before a measurement they intend to quote.
 * Returns 0 and changes nothing if no loop has closed yet, because a reset that
 * reported a fresh window it did not have would be the same lie as a zeroed
 * maximum. */
int ak_perf_reset(void);

#endif /* AK_CORE_AK_PERF_H */
