#ifndef AK_HOST_IDF_ADC_CALI_SCHEME_H
#define AK_HOST_IDF_ADC_CALI_SCHEME_H

#include <stdint.h>

#include "driver/rmt_types.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"

/*
 * The original ESP32's calibration scheme: line fitting. IDF declares it in
 * `components/esp_adc/include/esp_adc/adc_cali_scheme.h`, and the enum is
 * IDF's own - `ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF` is what a chip
 * whose eFuse carries no calibration of its own answers, which is the case the
 * port reports differently from a calibrated chip.
 */

typedef enum {
    ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP = 0, /**< eFuse two-point */
    ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF = 1, /**< eFuse reference */
    ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF = 2, /**< the datasheet */
} adc_cali_line_fitting_efuse_val_t;

typedef struct {
    adc_unit_t unit_id;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
    /* Only on the original ESP32, which is the only chip this port targets so
     * far: the reference voltage to use when the eFuse has none. */
    uint32_t default_vref;
} adc_cali_line_fitting_config_t;

esp_err_t adc_cali_scheme_line_fitting_check_efuse(
    adc_cali_line_fitting_efuse_val_t *cali_val);
esp_err_t adc_cali_create_scheme_line_fitting(
    const adc_cali_line_fitting_config_t *config,
    adc_cali_handle_t *ret_handle);

/*
 * And the *other* scheme in the family: curve fitting, which is what every
 * chip after the original ESP32 uses - two points measured at the factory,
 * fitted, and stored per channel. IDF guards this behind
 * `ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED`, which is how a chip that has only
 * one of the two can even be compiled; the host declares both, because it
 * compiles the port twice - once for the ESP32 and once for the chips that
 * have this - and runs both.
 *
 * The difference that matters to the port is `chan`: a curve is per channel,
 * and a line is not.
 */
typedef struct {
    adc_unit_t unit_id;
    adc_channel_t chan;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
} adc_cali_curve_fitting_config_t;

esp_err_t adc_cali_create_scheme_curve_fitting(
    const adc_cali_curve_fitting_config_t *config,
    adc_cali_handle_t *ret_handle);

#endif /* AK_HOST_IDF_ADC_CALI_SCHEME_H */
