#ifndef AK_HOST_IDF_ADC_CALI_H
#define AK_HOST_IDF_ADC_CALI_H

#include "driver/rmt_types.h"

/*
 * The calibration surface: a handle and the one call that turns a raw reading
 * into millivolts, as IDF declares them in
 * `components/esp_adc/include/esp_adc/adc_cali.h`.
 */

typedef struct adc_cali_scheme_t *adc_cali_handle_t;

esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t handle, int raw,
                                  int *voltage);

#endif /* AK_HOST_IDF_ADC_CALI_H */
