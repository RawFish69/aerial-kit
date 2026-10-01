#ifndef AK_HOST_ESP32_SYSTEM_MODEL_H
#define AK_HOST_ESP32_SYSTEM_MODEL_H

/*
 * The modelled system layer: why the chip restarted, the reset itself, and the
 * two GPIO calls the LED makes. Compiled *instead of* IDF, so the real
 * `src/arch/esp32/fault.c`, `system.c` and `led.c` run against it.
 *
 * What it is: a reason the test sets, a restart the test can count, and a pin
 * table that records what the LED asked for and what level it wrote. What it is
 * not: a chip. There is no reset, no eFuse, no panic handler - IDF owns all of
 * that on the real thing, and the port's job is only to *ask* about it
 * afterwards, which is what the checks are about.
 */

#include <stdint.h>

#include "esp_system.h"

void host_esp_reset(void);

/* What the next `esp_reset_reason()` answers - the fact the port turns into a
 * fault record when it is one of the three ways a chip can crash. */
void host_esp_set_reset_reason(esp_reset_reason_t reason);

/* How many times `esp_restart()` was called. */
unsigned host_esp_restarts(void);

/* What the LED asked the GPIO driver for. `host_gpio_pin_config_count()` is
 * zero until `gpio_config()` is called at all. */
unsigned host_gpio_pin_config_count(void);
uint64_t host_gpio_pin_mask(void);
int host_gpio_mode(void);
int host_gpio_pull_up(void);
int host_gpio_pull_down(void);
int host_gpio_intr(void);

/* The level on one pin, as the last `gpio_set_level()` left it: 0, 1, or -1 for
 * a pin nothing has driven. */
int host_gpio_level(int gpio);

#endif /* AK_HOST_ESP32_SYSTEM_MODEL_H */
