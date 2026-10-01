#ifndef AK_HOST_IDF_GPIO_H
#define AK_HOST_IDF_GPIO_H

/*
 * The SPI bus file includes IDF's GPIO header for the pin types it names in
 * its configuration structs. Nothing in `src/arch/esp32/spi.c` calls a GPIO
 * function - the peripheral driver owns the pins once the bus is up.
 *
 * `src/arch/esp32/led.c` does call two of them, though: it configures one pin
 * as an output and writes a level to it. So this header carries the two
 * declarations and the config struct, in IDF's shape, and
 * `tests/host_esp32_system_model.c` answers them. The enums are IDF's values,
 * because the point of the model is to see what the port asked for.
 */

#include "driver/rmt_types.h"

typedef enum {
    GPIO_MODE_DISABLE = 0,
    GPIO_MODE_INPUT = 1,
    GPIO_MODE_OUTPUT = 2,
    GPIO_MODE_OUTPUT_OD = 3,
    GPIO_MODE_INPUT_OUTPUT_OD = 4,
    GPIO_MODE_INPUT_OUTPUT = 5,
} gpio_mode_t;

typedef enum {
    GPIO_PULLUP_DISABLE = 0,
    GPIO_PULLUP_ENABLE = 1,
} gpio_pullup_t;

typedef enum {
    GPIO_PULLDOWN_DISABLE = 0,
    GPIO_PULLDOWN_ENABLE = 1,
} gpio_pulldown_t;

typedef enum {
    GPIO_INTR_DISABLE = 0,
    GPIO_INTR_POSEDGE = 1,
    GPIO_INTR_NEGEDGE = 2,
    GPIO_INTR_ANYEDGE = 3,
    GPIO_INTR_LOW_LEVEL = 4,
    GPIO_INTR_HIGH_LEVEL = 5,
} gpio_int_type_t;

typedef struct {
    uint64_t pin_bit_mask;      /* one bit per pin, as IDF's is */
    gpio_mode_t mode;
    gpio_pullup_t pull_up_en;
    gpio_pulldown_t pull_down_en;
    gpio_int_type_t intr_type;
} gpio_config_t;

esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);

#endif /* AK_HOST_IDF_GPIO_H */
