#include "host_esp32_adc_model.h"

#include <string.h>

/*
 * See host_esp32_adc_model.h for what this is and what it is not. The one
 * piece of IDF's behaviour that is modelled rather than stubbed is the line:
 * `adc_cali_raw_to_voltage` interpolates between the two points the eFuse
 * carries, which is the arithmetic the port deliberately does not repeat.
 */

struct host_adc_unit {
    int unit;
};

struct adc_cali_scheme_t {
    int used;
};

static struct host_adc_unit unit_slot;
static struct adc_cali_scheme_t scheme_slot;

/* And the curve-fitting scheme's side: what the port asked it for, and the
 * switch for a chip whose eFuse carries no curve at all. */
static int curve_created;
static int curve_unit = -1;
static int curve_chan = -1;
static int curve_atten = -1;
static int curve_bitwidth = -1;
static int curve_refused;

static int efuse_mode; /* 0 = two points, 1 = reference, 2 = empty */
static int low_raw = 0, low_mv = 100;
static int high_raw = 4095, high_mv = 3300;
static int reference_mv = 1100;

static int raw_by_unit[2][10];
static int fail_unit, fail_channel, fail_read, fail_scheme;

static int seen_unit = -1, seen_channel = -1, seen_atten = -1, seen_width = -1;
static unsigned units_made, channels_configured, reads;
static unsigned default_vref;

void host_adc_reset(void)
{
    memset(&unit_slot, 0, sizeof unit_slot);
    memset(&scheme_slot, 0, sizeof scheme_slot);
    memset(raw_by_unit, 0, sizeof raw_by_unit);
    efuse_mode = 0;
    low_raw = 0;
    low_mv = 100;
    high_raw = 4095;
    high_mv = 3300;
    reference_mv = 1100;
    fail_unit = fail_channel = fail_read = fail_scheme = 0;
    seen_unit = seen_channel = seen_atten = seen_width = -1;
    units_made = channels_configured = reads = 0;
    default_vref = 0;
    curve_created = 0;
    curve_unit = -1;
    curve_chan = -1;
    curve_atten = -1;
    curve_bitwidth = -1;
    curve_refused = 0;
}

void host_adc_efuse_two_points(int a_raw, int a_mv, int b_raw, int b_mv)
{
    efuse_mode = 0;
    low_raw = a_raw;
    low_mv = a_mv;
    high_raw = b_raw;
    high_mv = b_mv;
}

void host_adc_efuse_reference(int mv)
{
    efuse_mode = 1;
    reference_mv = mv;
}

void host_adc_efuse_empty(void) { efuse_mode = 2; }

void host_adc_set_raw(int unit, int channel, int raw)
{
    if (unit < 0 || unit > 1 || channel < 0 || channel > 9) {
        return;
    }
    raw_by_unit[unit][channel] = raw;
}

void host_adc_fail_unit(int fail) { fail_unit = fail; }
void host_adc_fail_channel(int fail) { fail_channel = fail; }
void host_adc_fail_read(int fail) { fail_read = fail; }
void host_adc_fail_scheme(int fail) { fail_scheme = fail; }

int host_adc_unit(void) { return seen_unit; }
int host_adc_channel(void) { return seen_channel; }
int host_adc_atten(void) { return seen_atten; }
int host_adc_bitwidth(void) { return seen_width; }
unsigned host_adc_units_made(void) { return units_made; }
unsigned host_adc_channels_configured(void) { return channels_configured; }
unsigned host_adc_reads(void) { return reads; }
unsigned host_adc_default_vref(void) { return default_vref; }

esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *config,
                               adc_oneshot_unit_handle_t *out)
{
    if (fail_unit || config == 0 || out == 0) {
        return ESP_FAIL;
    }
    units_made++;
    seen_unit = (int)config->unit_id;
    unit_slot.unit = (int)config->unit_id;
    *out = &unit_slot;
    return ESP_OK;
}

esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle,
                                     adc_channel_t channel,
                                     const adc_oneshot_chan_cfg_t *config)
{
    if (fail_channel || handle == 0 || config == 0) {
        return ESP_FAIL;
    }
    channels_configured++;
    seen_channel = (int)channel;
    seen_atten = (int)config->atten;
    seen_width = (int)config->bitwidth;
    return ESP_OK;
}

esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle,
                           adc_channel_t channel, int *out_raw)
{
    /* Every ask is counted, including the ones that fail: a test asking "did
     * it get quieter after the first failure" needs the count of questions. */
    reads++;
    if (fail_read || handle == 0 || out_raw == 0) {
        return ESP_FAIL;
    }
    *out_raw = raw_by_unit[handle->unit][(int)channel];
    return ESP_OK;
}

esp_err_t adc_cali_scheme_line_fitting_check_efuse(
    adc_cali_line_fitting_efuse_val_t *value)
{
    if (value == 0) {
        return ESP_FAIL;
    }
    if (efuse_mode == 1) {
        *value = ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF;
    } else if (efuse_mode == 2) {
        *value = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
    } else {
        *value = ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP;
    }
    return ESP_OK;
}

/*
 * And the scheme every chip after the original ESP32 actually uses: a curve,
 * fitted to two factory points, and *per channel* - which is the field this
 * side of the model exists to check. A port that passed the unit where the
 * channel goes would calibrate a pin against another pin's curve, and only
 * these chips would ever notice.
 */
esp_err_t adc_cali_create_scheme_curve_fitting(
    const adc_cali_curve_fitting_config_t *config, adc_cali_handle_t *out)
{
    if (curve_refused || config == 0 || out == 0) {
        /* A chip whose eFuse carries no curve cannot be calibrated at all: IDF
         * refuses to create the scheme, and the port has to survive that - it
         * falls back to the datasheet's straight line and says so. */
        return ESP_FAIL;
    }
    curve_created = 1;
    curve_unit = (int)config->unit_id;
    curve_chan = (int)config->chan;
    curve_atten = (int)config->atten;
    curve_bitwidth = (int)config->bitwidth;
    scheme_slot.used = 1;
    *out = &scheme_slot;
    return ESP_OK;
}

int host_adc_curve_created(void) { return curve_created; }
int host_adc_curve_unit(void) { return curve_unit; }
int host_adc_curve_channel(void) { return curve_chan; }
int host_adc_curve_atten(void) { return curve_atten; }
int host_adc_curve_bitwidth(void) { return curve_bitwidth; }
void host_adc_set_curve_refused(int refused) { curve_refused = refused; }

esp_err_t adc_cali_create_scheme_line_fitting(
    const adc_cali_line_fitting_config_t *config, adc_cali_handle_t *out)
{
    if (fail_scheme || config == 0 || out == 0) {
        return ESP_FAIL;
    }
    default_vref = config->default_vref;
    scheme_slot.used = 1;
    *out = &scheme_slot;

    if (efuse_mode != 0) {
        /* A reference voltage rather than two measured points - and which
         * reference depends on the eFuse: its own when it carries one, and the
         * `default_vref` the caller passed when it carries none, which is
         * IDF's rule and the reason `adc.c` fills that field in whichever case
         * it is. The line runs from zero to the full scale a reference gives
         * at 12 dB attenuation, about 2.8 times it - 3100 mV from 1100 mV, the
         * same span src/arch/esp32/adc.c quotes from the datasheet. */
        int vref = efuse_mode == 1 ? reference_mv : (int)config->default_vref;
        low_raw = 0;
        low_mv = 0;
        high_raw = 4095;
        high_mv = (int)((long)vref * 31L / 11L);
    }
    return ESP_OK;
}

esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t handle, int raw,
                                  int *out_mv)
{
    if (handle == 0 || out_mv == 0 || high_raw == low_raw) {
        return ESP_FAIL;
    }
    /* The line through the two points, which is what "line fitting" means. */
    *out_mv = low_mv + (int)(((long)(raw - low_raw) * (high_mv - low_mv)) /
                             (high_raw - low_raw));
    return ESP_OK;
}
