#include "host_esp32_system_model.h"

#include "driver/gpio.h"

/*
 * See host_esp32_system_model.h. One pin table of the size a devkit's GPIO
 * numbering needs, and the three IDF calls the port makes into it.
 */

#define MODEL_PINS 40

static esp_reset_reason_t reason = ESP_RST_POWERON;
static unsigned restarts;

static unsigned pin_configs;
static uint64_t pin_mask;
static int pin_mode = -1;
static int pin_pull_up = -1;
static int pin_pull_down = -1;
static int pin_intr = -1;

static int levels[MODEL_PINS];

void host_esp_reset(void)
{
    reason = ESP_RST_POWERON;
    restarts = 0;
    pin_configs = 0;
    pin_mask = 0;
    pin_mode = pin_pull_up = pin_pull_down = pin_intr = -1;
    for (int i = 0; i < MODEL_PINS; i++) {
        levels[i] = -1;
    }
}

void host_esp_set_reset_reason(esp_reset_reason_t value) { reason = value; }

unsigned host_esp_restarts(void) { return restarts; }
unsigned host_gpio_pin_config_count(void) { return pin_configs; }
uint64_t host_gpio_pin_mask(void) { return pin_mask; }
int host_gpio_mode(void) { return pin_mode; }
int host_gpio_pull_up(void) { return pin_pull_up; }
int host_gpio_pull_down(void) { return pin_pull_down; }
int host_gpio_intr(void) { return pin_intr; }

int host_gpio_level(int gpio)
{
    return (gpio >= 0 && gpio < MODEL_PINS) ? levels[gpio] : -1;
}

esp_reset_reason_t esp_reset_reason(void) { return reason; }

void esp_restart(void) { restarts++; }

esp_err_t gpio_config(const gpio_config_t *config)
{
    if (config == 0) {
        return ESP_FAIL;
    }
    pin_configs++;
    pin_mask = config->pin_bit_mask;
    pin_mode = (int)config->mode;
    pin_pull_up = (int)config->pull_up_en;
    pin_pull_down = (int)config->pull_down_en;
    pin_intr = (int)config->intr_type;
    return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    int pin = (int)gpio_num;

    if (pin < 0 || pin >= MODEL_PINS) {
        return ESP_FAIL;
    }
    levels[pin] = level ? 1 : 0;
    return ESP_OK;
}
