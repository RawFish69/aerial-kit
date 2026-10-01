#include "ak_time.h"

/*
 * The wait that a delay runs, with the clock handed in.
 *
 * This is the one wait in the firmware that had no bound: every other one - the
 * console's UART, the flash controller, the SPI and I2C drivers, the USB core -
 * gives up and says so, because a board that stops inside its own boot is a
 * board nobody can diagnose. The F405 on the bench on 2026-09-17 is why: it put
 * its USB device on the bus and then said nothing, and one of the two things
 * that can look like that is a delay waiting on a SysTick that never arrives.
 *
 * `limit` is in rounds of the loop, not milliseconds: the whole point is to
 * decide when the clock *is not moving*, so the count has to come from
 * somewhere the clock cannot influence. Two million rounds is about a second on
 * a 168 MHz part - far longer than any real millisecond, and short enough that
 * a dead tick is a slow boot rather than a stopped board.
 *
 * Taking `now` as an argument is what makes both endings testable on a host
 * that has no SysTick running: `tests/test_arch.c` drives a clock that never
 * moves and one that counts.
 */
#define AK_DELAY_GUARD_ROUNDS 2000000u

static uint32_t delay_stalls;

unsigned ak_wait_for_ticks(uint32_t start, uint32_t ms, uint32_t limit,
                           uint32_t (*now)(void), int *gave_up)
{
    unsigned rounds = 0u;

    if (gave_up != 0) {
        *gave_up = 0;
    }
    while ((uint32_t)(now() - start) < ms) {
        if (++rounds >= limit) {
            if (gave_up != 0) {
                *gave_up = 1;
            }
            break;
        }
    }
    return rounds;
}

uint32_t ak_delay_stalls(void)
{
    return delay_stalls;
}

#ifdef AK_HOST_TICK
/*
 * The one place a host build needs a delay to mean something different.
 *
 * On a target this function waits for a tick that a timer raises. On the host
 * there is no timer - the tick in `src/arch/stm32f405/systick.c` is a variable
 * that a test steps by hand - so waiting for it is waiting for ever, which is
 * exactly what happened the first time a *port's own* bus driver was run on
 * the host: `ak_imu_open()` asks the bus for the datasheet's fifteen
 * milliseconds before it reads its who-am-i, and the test hung there.
 *
 * So on the host a delay is what advances the tick. It is a property of the
 * test environment and not of the firmware (the flag is on this object and the
 * systick object, and neither the simulator nor a target build sees it), and
 * it makes the delay honest in the only way a host can: the clock the driver
 * is waiting on moves by the amount it asked for.
 */
void ak_host_tick_advance(uint32_t ms);

void ak_delay_ms(uint32_t ms)
{
    ak_host_tick_advance(ms);
}
#else
void ak_delay_ms(uint32_t ms)
{
    int gave_up = 0;

    if (ms == 0u) {
        return;
    }
    (void)ak_wait_for_ticks(ak_time_ms(), ms, AK_DELAY_GUARD_ROUNDS,
                            ak_time_ms, &gave_up);
    if (gave_up) {
        /* The tick is not moving. Waiting longer cannot help; counted, so the
         * preflight can print how many waits gave up. */
        delay_stalls++;
    }
}
#endif
