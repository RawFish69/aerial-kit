#include "esp.h"

#include "driver/gpio.h"

static int led_gpio = -1;
static int led_lit;

void ak_esp_led_init(int gpio)
{
    led_gpio = gpio;
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << (uint32_t)gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&config);
    ak_esp_led_set(0);
}

void ak_esp_led_set(int on)
{
    led_lit = on ? 1 : 0;
    if (led_gpio >= 0) {
        gpio_set_level((gpio_num_t)led_gpio, on ? 1 : 0);
    }
}

int ak_esp_led_state(void)
{
    return led_lit;
}
