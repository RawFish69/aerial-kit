#include "esp.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/*
 * The flight pack's ADC, on this chip.
 *
 * What is different here is not the arithmetic - the divider ratio, the cell
 * count and the thresholds all live in the core, in the same file the F405
 * uses - but the *unit of the answer*. The F405's ADC hands back counts and
 * the core converts them with a reference and a full scale it was told. This
 * chip's calibration is *line* fitting - the original ESP32 measures two
 * points into eFuse and fits a line through them, where the newer parts have a
 * multi-point curve - so the only honest conversion is the one IDF does, and
 * that is what comes out of here. The board's
 * battery read therefore returns volts and never sees a reference voltage,
 * which is the shape of the port working as intended: the core's contract is
 * "volts at your pin", and how a chip produces that is its own business.
 *
 * The attenuation is 12 dB, which is the widest this part has and about 3.1
 * volts of measured span with the curve-fitting calibration. That matters for
 * a divider rather than being a detail: a 10k/1k pair on a 6S pack lands at
 * 2.3 volts at the pin, and a narrower range would clip it.
 *
 * Nothing here has been on a board, and QEMU does not model this peripheral.
 * The emulator can say that the firmware runs with no divider fitted and says
 * so; it cannot say anything about a voltage.
 */

/* The two numbers the datasheet supplies when a chip's eFuse carries no
 * calibration of its own: the nominal reference and the span at the widest
 * attenuation. They are the *fallback*, and the report says when they are what
 * a reading came from. */
#define AK_ESP_ADC_DEFAULT_VREF_MV 1100
#define AK_ESP_ADC_FULL_SCALE_MV    3100

static int state; /* 0 = not tried, 1 = up, -1 = refused */
static adc_oneshot_unit_handle_t unit_handle;
static adc_cali_handle_t cali_handle;
static int cali_ok;
static int cali_default_vref; /* the eFuse had no reference voltage */
static int open_unit = -1;
static int open_channel = -1;

int ak_esp_adc_init(int unit, int channel)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { 0 };
    adc_oneshot_chan_cfg_t chan_cfg = { 0 };

    /*
     * The board names the unit the way the chip's pins do - "ADC1 channel 6"
     * for GPIO34 - and IDF's `adc_unit_t` counts from zero: `ADC_UNIT_1` is
     * **0**, `ADC_UNIT_2` is 1. So the translation is here, once, rather than
     * in a board header that would have to include IDF to say it: a board that
     * wrote 1 meaning ADC1 (which is what this port had) asks for the *second*
     * unit, and every pin on that unit is a different pin. Nothing on this
     * machine could see it - the conversion is IDF's and QEMU models no ADC -
     * so it is written down where the model can check it.
     *
     * A board asking for a unit this chip does not have is a board
     * misconfigured rather than a part that failed to come up, so it is
     * refused without latching `state`.
     */
    if (unit != 1 && unit != 2) {
        return -1;
    }
    if (state != 0) {
        /*
         * One converter per board in this port - the handle, the channel and
         * the calibration are all single - so a second init that asks for the
         * same converter is free and a second init that asks for a *different*
         * one is refused rather than silently answered with the first. (It did
         * the latter, which is how a board with two channels would have got
         * one channel's readings under the other's name.)
         */
        return (state == 1 && unit == open_unit && channel == open_channel)
                   ? 0
                   : -1;
    }
    unit_cfg.unit_id = (adc_unit_t)(unit - 1);
    if (adc_oneshot_new_unit(&unit_cfg, &unit_handle) != ESP_OK) {
        state = -1;
        return -1;
    }
    chan_cfg.atten = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_oneshot_config_channel(unit_handle, (adc_channel_t)channel,
                                   &chan_cfg) != ESP_OK) {
        state = -1;
        return -1;
    }

    /*
     * The calibration is what turns counts into volts. On this part it is
     * either a reference voltage or two measured points in eFuse, and a chip
     * whose eFuse carries neither is a chip that needs the datasheet's
     * nominal - the one case where IDF asks for `default_vref` rather than
     * reading it. Either way the answer is a voltage, and which of the three
     * it was is worth printing: "the calibration is the datasheet's" and "the
     * calibration is this chip's" are different claims about the same number.
     */
#if CONFIG_IDF_TARGET_ESP32
    /*
     * The original ESP32 calibrates with a *line*: a reference voltage and a
     * span, read from eFuse - and when the eFuse carries neither, IDF wants the
     * datasheet's nominal passed in, which is why this scheme has a
     * `default_vref` field at all. `check_efuse` is what says which of the two
     * a reading came from.
     */
    adc_cali_line_fitting_config_t cali_cfg = { 0 };
    adc_cali_line_fitting_efuse_val_t cali_val = 0;

    (void)adc_cali_scheme_line_fitting_check_efuse(&cali_val);
    cali_default_vref =
        cali_val == ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
    cali_cfg.unit_id = (adc_unit_t)(unit - 1);
    cali_cfg.atten = ADC_ATTEN_DB_12;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    cali_cfg.default_vref = AK_ESP_ADC_DEFAULT_VREF_MV;
    cali_ok = adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle) ==
              ESP_OK;
#else
    /*
     * Every other chip in this family calibrates with a *curve*: two points
     * measured at the factory, fitted, and stored per channel in eFuse. There
     * is no default reference to hand in - a chip whose eFuse carries no curve
     * cannot be calibrated at all, and the create fails, which leaves this port
     * on the datasheet's straight line and says so in its report. The rows are
     * the same four either way; what differs is which of the chip's own numbers
     * turns a reading into volts.
     *
     * The ESP32-S3 build is what found this: the line-fitting types do not
     * exist on that target, so the port had been written for one chip while
     * claiming to be the family's.
     */
    adc_cali_curve_fitting_config_t cali_cfg = { 0 };

    cali_cfg.unit_id = (adc_unit_t)(unit - 1);
    cali_cfg.chan = (adc_channel_t)channel;
    cali_cfg.atten = ADC_ATTEN_DB_12;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    cali_ok = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) ==
              ESP_OK;
    cali_default_vref = 0;
#endif

    open_unit = unit;
    open_channel = channel;
    state = 1;
    return 0;
}

int ak_esp_adc_mv(int unit, int channel)
{
    int raw = 0;
    int mv = 0;

    if (state != 1 || unit != open_unit || channel != open_channel) {
        return -1;
    }
    if (adc_oneshot_read(unit_handle, (adc_channel_t)channel, &raw) != ESP_OK) {
        return -1;
    }
    if (cali_ok && adc_cali_raw_to_voltage(cali_handle, raw, &mv) == ESP_OK) {
        return mv;
    }
    /* No calibration at all: the datasheet's straight line for the widest
     * attenuation, which is the same one the reference above names. */
    return (int)((float)raw * AK_ESP_ADC_FULL_SCALE_MV / 4095.0f);
}

void ak_esp_adc_report(ak_printf_fn out)
{
    if (state != 1) {
        out("adc:       the converter did not come up\n");
        return;
    }
    out("adc:       ADC%d channel %d, 12 dB attenuation, %s\n", open_unit,
        open_channel,
        !cali_ok ? "no calibration in eFuse: the datasheet's straight line"
                 : (cali_default_vref
                        ? "line fitting against the datasheet's reference"
                        : "line fitting against this chip's own eFuse"));
}

#ifdef AK_HOST_ADC
/*
 * The host model's tests drive this converter more than once in one process,
 * which a board never does: `state` is what makes the second init free, so
 * forgetting it is how the tests reach the paths after the first one -
 * a chip with the datasheet's reference instead of eFuse, a chip with no
 * calibration at all, a read that fails. Compiled only for the host build
 * (the Makefile passes -DAK_HOST_ADC on that object alone, the same seam the
 * F405's flash and I2C tests use), so the image never contains it.
 */
void ak_esp_adc_forget_for_host(void)
{
    state = 0;
    unit_handle = 0;
    cali_handle = 0;
    cali_ok = 0;
    cali_default_vref = 0;
    open_unit = -1;
    open_channel = -1;
}
#endif
