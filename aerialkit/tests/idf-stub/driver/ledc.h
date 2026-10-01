#ifndef AK_HOST_IDF_LEDC_H
#define AK_HOST_IDF_LEDC_H

#include <stdint.h>

#include "driver/rmt_types.h" /* esp_err_t, and the same notion of "opaque" */

/*
 * The LEDC surface `src/arch/esp32/output.c` uses for the servos: one timer at
 * 50 Hz with 16-bit duty, one channel per servo, and a duty written every
 * frame. IDF's own headers are the reference for the fields and the enum
 * names; the ESP32 build is what catches this file drifting from them.
 */

typedef enum {
    LEDC_LOW_SPEED_MODE = 0,
    LEDC_SPEED_MODE_MAX,
} ledc_mode_t;

typedef enum {
    LEDC_INTR_DISABLE = 0,
    LEDC_INTR_FADE_END,
    LEDC_INTR_MAX,
} ledc_intr_type_t;

typedef enum {
    LEDC_TIMER_0 = 0,
    LEDC_TIMER_1,
    LEDC_TIMER_MAX,
} ledc_timer_t;

typedef enum {
    LEDC_CHANNEL_0 = 0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_MAX,
} ledc_channel_t;

/* How wide this chip's duty counter is, as IDF's own soc_caps.h has it: the
 * port picks its servo resolution from this, and the ESP32's is 20 bits, so
 * the host build compiles the wider branch. The S3's is 14 and its build
 * compiles the other one. */
#define SOC_LEDC_TIMER_BIT_WIDTH 20

/* The resolution enum is sparse in IDF; only the 16-bit entry matters here. */
typedef enum {
    LEDC_TIMER_1_BIT = 1,
    LEDC_TIMER_16_BIT = 16,
} ledc_timer_bit_t;

typedef enum {
    LEDC_AUTO_CLK = 0,
    LEDC_USE_APB_CLK,
} ledc_clk_cfg_t;

typedef struct {
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t timer_num;
    uint32_t freq_hz;
    ledc_clk_cfg_t clk_cfg;
    bool deconfigure;
} ledc_timer_config_t;

typedef struct {
    int gpio_num;
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    ledc_intr_type_t intr_type;
    ledc_timer_t timer_sel;
    uint32_t duty;
    int hpoint;
    struct {
        unsigned int output_invert : 1;
    } flags;
} ledc_channel_config_t;

esp_err_t ledc_timer_config(const ledc_timer_config_t *timer_conf);
esp_err_t ledc_channel_config(const ledc_channel_config_t *ledc_conf);
esp_err_t ledc_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel,
                        uint32_t duty);
esp_err_t ledc_update_duty(ledc_mode_t speed_mode, ledc_channel_t channel);

#endif /* AK_HOST_IDF_LEDC_H */
