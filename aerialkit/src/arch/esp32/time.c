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
