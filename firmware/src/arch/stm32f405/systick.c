#include "arch.h"
#include "ak_time.h"

static volatile uint32_t tick_ms;

#ifdef AK_HOST_TICK
/* The fraction of the current millisecond, which nothing on a host produces by
 * itself: there is no counter here to count down, so a test moves this one.
 *
 * It is deliberately *not* the whole clock. The millisecond stays `tick_ms`,
 * incremented by SysTick_Handler and by ak_delay_ms exactly as it was before
 * this existed, so every reading taken before the microsecond clock arrived
 * still means what it meant - which is the property the tick test depends on:
 * five calls to the handler are five milliseconds, with or without a fraction
 * sitting beside them. */
static uint32_t host_sub_us;
#endif

void SysTick_Handler(void)
{
    tick_ms++;
}

void ak_arch_time_init(void)
{
    SYSTICK_LOAD = (ak_clk_sysclk_hz() / 1000u) - 1u;
    SYSTICK_VAL  = 0;
    SYSTICK_CTRL = SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT |
                   SYSTICK_CTRL_ENABLE;
}

uint32_t ak_arch_time_ms(void)
{
    return tick_ms;
}

/*
 * The microsecond clock, from the counter the tick is already made of.
 *
 * SysTick counts *down* from LOAD at the core clock, and the handler that
 * increments tick_ms is what fires when it reaches zero. So the counter holds
 * the fraction of the current millisecond that has elapsed, and the two reads -
 * the counter and the millisecond - are a two-step read with a rollover that
 * can land between them. The rollover is not rare: at a 1 kHz tick a caller
 * reading this every millisecond samples the whole window, and one that lands
 * inside the read would pair a fraction from the new millisecond with the
 * millisecond *before* it - a clock that goes backwards by almost a
 * millisecond, which a scheduler would read as a deadline it has already
 * missed.
 *
 * The guard is to read the counter either side of the millisecond and retry if
 * it went *up*, which is what a reload looks like when the counter counts down.
 * Only when no reload happened inside the window does `ms` belong to the same
 * millisecond as `val2`, and only then does the subtraction mean anything.
 *
 * The wrap at 2^32 us is ~71.6 minutes and is the caller's business - see
 * ak_time.h for the rule. Nothing here extends it; a deadline further away than
 * that is not representable and the scheduler re-arms from now rather than
 * trusting one.
 */
uint32_t ak_arch_time_us(void)
{
#ifdef AK_HOST_TICK
    /* Nothing on the host counts the fraction down, so it is a variable a test
     * moves. Same shape as the target: the millisecond plus the part of the
     * next one that has gone by. */
    return tick_ms * 1000u + host_sub_us;
#else
    uint32_t ms, val, val2;
    uint32_t per_us = ak_cycles_per_us();

    do {
        val  = SYSTICK_VAL;
        ms   = tick_ms;
        val2 = SYSTICK_VAL;
    } while (val2 > val);

    if (per_us == 0u) {
        /* No rate to convert cycles with. The millisecond is still a fact and
         * the fraction is not, so this answers what it knows - the same choice
         * ak_perf makes when a port cannot state a clock, and the reason this
         * never returns a number it did not measure. */
        return ms * 1000u;
    }
    return ms * 1000u + (SYSTICK_LOAD - val2) / per_us;
#endif
}

#ifdef AK_HOST_TICK
/* The host's clock, moved by the delay that is waiting on it - see the note in
 * src/core/time.c. Nothing else in the host build can advance this, which is
 * why the seam exists at all. */
void ak_host_tick_advance(uint32_t ms)
{
    tick_ms += ms;
}

/* And the finer step, which the host build had no way to ask for until the
 * scheduler needed one: a one-millisecond tick cannot move a five-microsecond
 * deadline. Separate from ak_host_tick_advance rather than a parameter on it so
 * that every existing caller keeps meaning exactly what it meant - a delay of
 * one millisecond is one tick and no fraction, not a tick plus whatever the
 * last caller left behind. */
void ak_host_tick_advance_us(uint32_t us)
{
    host_sub_us += us;
    tick_ms      += host_sub_us / 1000u;
    host_sub_us  %= 1000u;
}
#endif

/* The portable contract in core/time.h is satisfied here, by the port. */
void ak_time_init(void)
{
    ak_arch_time_init();
}

uint32_t ak_time_ms(void)
{
    return ak_arch_time_ms();
}

uint32_t ak_time_us(void)
{
    return ak_arch_time_us();
}
