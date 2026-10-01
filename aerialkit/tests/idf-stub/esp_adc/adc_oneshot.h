#ifndef AK_HOST_IDF_ADC_ONESHOT_H
#define AK_HOST_IDF_ADC_ONESHOT_H

#include <stdint.h>

/* esp_err_t, ESP_OK and the rest of the one definition of them, which lives
 * with the RMT types because that was the first stub in this tree. */
#include "driver/rmt_types.h"

/*
 * The one-shot ADC surface `src/arch/esp32/adc.c` calls, declared the way
 * ESP-IDF declares it (`components/hal/include/hal/adc_types.h` and
 * `components/esp_adc/include/esp_adc/adc_oneshot.h` in the pinned checkout).
 *
 * The thing worth reading twice is the unit enum: `ADC_UNIT_1` is **0** and
 * `ADC_UNIT_2` is 1, where the chip's own pin names (and every board header
 * here) say ADC1 and ADC2. That off-by-one is the reason this stub exists at
 * all - it is a mistake a host can catch and no board can, because the
 * conversion itself is IDF's.
 */

typedef enum {
    ADC_UNIT_1, /* 0, and the unit GPIO32-39 belong to */
    ADC_UNIT_2, /* 1, which Wi-Fi owns while the radio is on */
} adc_unit_t;

typedef enum {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7,
    ADC_CHANNEL_8, ADC_CHANNEL_9,
} adc_channel_t;

typedef enum {
    ADC_ATTEN_DB_0 = 0,
    ADC_ATTEN_DB_2_5 = 1,
    ADC_ATTEN_DB_6 = 2,
    ADC_ATTEN_DB_12 = 3,
} adc_atten_t;

typedef enum {
    ADC_BITWIDTH_DEFAULT = 0, /* the widest this unit supports */
    ADC_BITWIDTH_9 = 9,
    ADC_BITWIDTH_10 = 10,
    ADC_BITWIDTH_11 = 11,
    ADC_BITWIDTH_12 = 12,
} adc_bitwidth_t;

typedef struct host_adc_unit *adc_oneshot_unit_handle_t;

typedef struct {
    adc_unit_t unit_id;
    int clk_src;  /* ADC_RTC_CLK_SRC_DEFAULT */
    int ulp_mode; /* ADC_ULP_MODE_DISABLE */
} adc_oneshot_unit_init_cfg_t;

typedef struct {
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
} adc_oneshot_chan_cfg_t;

esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *init_config,
                               adc_oneshot_unit_handle_t *ret_unit);
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle,
                                     adc_channel_t channel,
                                     const adc_oneshot_chan_cfg_t *config);
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle,
                           adc_channel_t chan, int *out_raw);

#endif /* AK_HOST_IDF_ADC_ONESHOT_H */
