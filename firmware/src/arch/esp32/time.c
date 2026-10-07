#include "esp.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Time, from esp_timer's microsecond counter.
 *
 * This file exists instead of the portable core/time.c, which busy-waits. On a
 * chip running FreeRTOS that would starve the idle task and trip the watchdog,
 * so the port provides both the clock and the delay - and core/time.c is left
 * out of the ESP32 build on purpose. It is the same contract either way.
 */

void ak_time_init(void)
{
    /* esp_timer is already running; nothing to start. */
}

uint32_t ak_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/*
 * The microsecond clock, which this port has had all along: esp_timer's
 * counter is microseconds and 64 bits, and the millisecond above is already a
 * division of it. This is the truncation to the contract's 32 bits, and it is
 * the one port where ak_time_us costs nothing new.
 *
 * It also makes a point about the cycle counter in this file: this chip can
 * answer a *time* in microseconds while answering *zero* for cycles per
 * microsecond, and the two are not in conflict. esp_timer is a clock the chip
 * maintains for exactly this; CCOUNT is the CPU clock, which moves. The
 * scheduler wants the former.
 */
uint32_t ak_time_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

void ak_delay_ms(uint32_t ms)
{
    /* One tick at a time so a long delay still feeds the scheduler. */
    while (ms >= 10u) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ms -= 10u;
    }
    if (ms > 0u) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

/*
 * The cycle counter, which this port does not have.
 *
 * The M4 ports read DWT->CYCCNT (src/arch/arm/cortex-m4/cycles.c) and get a
 * free-running count at a clock fixed at boot. The ESP32 has a count - Xtensa's
 * CCOUNT, which esp_cpu_get_cycle_count() reads - but **not at a fixed rate**:
 * it counts the CPU clock, and this chip's clock moves (DFS, and the light-sleep
 * entry that stops it altogether). `ak_cycles_per_us()` is a single number, and
 * a port whose rate changes cannot state one, so the honest answer here is the
 * one ak_time.h defines for exactly this case: zero.
 *
 * Zero is not a stub and it is not "not implemented yet". ak_perf.c reads it as
 * *"this port cannot convert cycles to microseconds"*, reports `samples` below
 * `loops`, and prints no durations - rather than dividing by a rate it does not
 * have and producing a plausible wrong number. The distinction is the same one
 * the whole profiler is built around, and it is why the port is allowed to
 * answer this way.
 *
 * A real reading here would mean a rate the loop can trust, which means pinning
 * the CPU clock for the duration of a measurement - a change to this port that
 * nobody has made and that no board here has been asked to justify.
 */
void ak_cycles_init(void)
{
    /* Nothing to start. */
}

uint32_t ak_cycles(void)
{
    return 0u;
}

uint32_t ak_cycles_per_us(void)
{
    return 0u;
}

uint32_t ak_delay_stalls(void)
{
    /* The portable delay learned to give up on a tick that never moves (see
     * src/core/time.c, and the F405 that stopped inside its own boot on
     * 2026-09-17). This port does not need the guard for the same reason it
     * does not use that file at all: its delay is FreeRTOS's, and a FreeRTOS
     * tick that has stopped is a board with nothing left to report. The count
     * exists so the console's preflight line - which is shared code - links and
     * says zero here. */
    return 0u;
}
