/*
 * The ESP32's battery converter, run on the host against a modelled IDF.
 *
 * This is the last file in `src/arch/esp32/` that had never been executed
 * anywhere: QEMU models no ADC, so the only thing that had ever happened to
 * `adc.c` was being compiled. What makes it worth a model is that this port
 * does *not* convert counts to volts - the chip's calibration is line fitting
 * on the original ESP32, so the conversion is IDF's, and the port's job is the
 * unit, the channel, the attenuation and which calibration the answer came
 * from.
 *
 * And that is where the bug was. A board header says "ADC1 channel 6" for
 * GPIO34, because that is what the chip's own pin table says; IDF's
 * `adc_unit_t` counts from zero, so `ADC_UNIT_1` is **0**. The port passed the
 * board's number straight through: a board asking for ADC1 got ADC_UNIT_2, a
 * unit whose channels are different pins and which the radio owns whenever
 * Wi-Fi is on. Nothing on this machine could see it - the conversion is IDF's
 * and no emulator here has an ADC - which is the case for a stand-in with the
 * real enum in it.
 *
 * What the model is not is a converter: the line's two points are the test's,
 * so every voltage below is a number this file chose. What is checked is the
 * port's side of the contract.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "esp.h"

#include "host_esp32_adc_model.h"
#include "tests.h"

#ifndef AK_HOST_ADC
#error "this file drives the port's host seam: build it with -DAK_HOST_ADC"
#endif
void ak_esp_adc_forget_for_host(void);

/*
 * And the same file compiled a second time, as a chip whose converter
 * calibrates with a *curve*: every ESP32 after the original one. The ESP32
 * build never compiles that branch and the S2/S3/C3 builds compile it without
 * a way to run it here, so the second compile - with its entry points renamed,
 * the way the two fault records are - is how those lines get executed. Two of
 * them are only in that branch and would otherwise be checked by nothing: the
 * unit translation it repeats, and the *channel*, which a curve has and a line
 * does not.
 */
int  ak_esp_adc_curve_init(int unit, int channel);
int  ak_esp_adc_curve_mv(int unit, int channel);
void ak_esp_adc_curve_report(ak_printf_fn out);
void ak_esp_adc_curve_forget_for_host(void);

static char report[512];

static int sink(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(report + strlen(report),
                            sizeof report - strlen(report), fmt, args);
    va_end(args);
    return written;
}

static void fresh(void)
{
    report[0] = '\0';
    host_adc_reset();
    ak_esp_adc_forget_for_host();
}

void test_arch_esp32_adc(void)
{
    const int unit = AK_BOARD_VBAT_UNIT;       /* 1: the board's ADC1 */
    const int channel = AK_BOARD_VBAT_CHANNEL; /* 6: GPIO34 */

    /* --- the converter the board names is the converter IDF is given ------ */

    fresh();
    expect("a board's converter comes up", ak_esp_adc_init(unit, channel) == 0);
    expect("and IDF is given the unit the board's pin belongs to",
           host_adc_unit() == ADC_UNIT_1);
    expect("which is not the number the board wrote down",
           host_adc_unit() != unit);
    expect("with the channel the board names", host_adc_channel() == channel);
    expect("at the widest attenuation, so a 6S divider is not clipped",
           host_adc_atten() == ADC_ATTEN_DB_12);
    expect("and the unit's default width", host_adc_bitwidth() ==
                                               ADC_BITWIDTH_DEFAULT);
    expect("one converter for one board", host_adc_units_made() == 1u);

    /* --- the volts are the chip's calibration, not this port's ----------- */

    /* Two points as an eFuse carries them: 100 mV at 0 counts and 3300 mV at
     * full scale. The line through them at 1000 counts is 881 mV, and the
     * datasheet's straight line (which the port uses only when there is no
     * calibration at all) would have said 757. */
    host_adc_efuse_two_points(0, 100, 4095, 3300);
    host_adc_set_raw(ADC_UNIT_1, channel, 1000);
    expect("a reading is the chip's own line, not the datasheet's",
           ak_esp_adc_mv(unit, channel) == 881);
    expect("and the port asks for the datasheet's reference only as a "
           "fallback", host_adc_default_vref() == 1100u);

    ak_esp_adc_report(sink);
    expect("the report says which unit and channel came up",
           strstr(report, "ADC1 channel 6") != 0);
    expect("and that the calibration is this chip's",
           strstr(report, "this chip's own eFuse") != 0);

    /* --- a unit or channel this port did not open is an error ------------- */

    expect("another unit is not a reading",
           ak_esp_adc_mv(2, channel) < 0);
    expect("another channel is not a reading",
           ak_esp_adc_mv(unit, channel + 1) < 0);
    expect("and the unit the board wrote is not how IDF counts",
           ak_esp_adc_mv(unit, channel) >= 0);

    /* --- one converter per board ----------------------------------------- */

    expect("asking for the same converter again is free",
           ak_esp_adc_init(unit, channel) == 0 && host_adc_units_made() == 1u);
    expect("and asking for a second one is refused rather than answered "
           "with the first",
           ak_esp_adc_init(2, 5) < 0 && host_adc_units_made() == 1u);
    expect("a unit this chip does not have is refused too",
           ak_esp_adc_init(3, 5) < 0);

    /* --- an eFuse with no calibration of its own ------------------------- */

    fresh();
    host_adc_efuse_empty();
    expect("a chip whose eFuse has nothing comes up anyway",
           ak_esp_adc_init(unit, channel) == 0);
    host_adc_set_raw(ADC_UNIT_1, channel, 4095);
    expect("on the datasheet's reference, which is IDF's line to draw",
           ak_esp_adc_mv(unit, channel) == 3100);
    report[0] = '\0';
    ak_esp_adc_report(sink);
    expect("and the report says the calibration is the datasheet's",
           strstr(report, "the datasheet's reference") != 0);

    /* --- and a part with no calibration at all --------------------------- */

    fresh();
    host_adc_fail_scheme(1);
    expect("a chip whose calibration will not build still comes up",
           ak_esp_adc_init(unit, channel) == 0);
    host_adc_set_raw(ADC_UNIT_1, channel, 1000);
    expect("and the port falls back to the datasheet's own straight line",
           ak_esp_adc_mv(unit, channel) == 757);
    report[0] = '\0';
    ak_esp_adc_report(sink);
    expect("which the report says out loud rather than calling it calibrated",
           strstr(report, "no calibration in eFuse") != 0);

    /* --- a converter that stops answering --------------------------------- */

    fresh();
    host_adc_fail_read(1);
    expect("a conversion that fails is an error, not a stale reading",
           ak_esp_adc_init(unit, channel) == 0 &&
               ak_esp_adc_mv(unit, channel) < 0);
    expect("and it stays an error on the next ask",
           ak_esp_adc_mv(unit, channel) < 0 && host_adc_reads() >= 2u);

    /* --- a converter that never comes up ---------------------------------- */

    fresh();
    host_adc_fail_unit(1);
    expect("a converter that cannot be made is refused",
           ak_esp_adc_init(unit, channel) < 0 && host_adc_units_made() == 0u);
    expect("and it is not retried behind the caller's back",
           ak_esp_adc_init(unit, channel) < 0 && host_adc_units_made() == 0u);
    report[0] = '\0';
    ak_esp_adc_report(sink);
    expect("and the report says the converter did not come up",
           strstr(report, "did not come up") != 0);
    expect("with no reading to hand out", ak_esp_adc_mv(unit, channel) < 0);

    /* --- and the same port as a chip whose calibration is a curve --------- */

    /*
     * The other half of the family: the chips after the original ESP32 fit a
     * curve through two factory points, per channel. The port's branch for
     * that is compiled by the S2/S3/C3 builds and executed by nothing on this
     * machine, so this drives the second compile of the file directly.
     */
    host_adc_reset();
    host_adc_set_raw(ADC_UNIT_1, channel, 2048); /* the model counts from zero */
    ak_esp_adc_curve_forget_for_host();

    expect("a chip with a curve in its eFuse comes up",
           ak_esp_adc_curve_init(unit, channel) == 0);
    expect("and it is the curve scheme that is created",
           host_adc_curve_created() == 1);
    expect("in IDF's unit numbering, which the board's number is not",
           host_adc_curve_unit() == ADC_UNIT_1 && host_adc_curve_unit() != unit);
    /* The one field a line does not have: a curve is measured per channel, so
     * a port that passed the unit here would calibrate the pin against another
     * pin's curve - and only these chips would ever notice. */
    expect("with the channel the curve belongs to",
           host_adc_curve_channel() == channel);
    expect("at the widest attenuation and the default width",
           host_adc_curve_atten() == ADC_ATTEN_DB_12 &&
               host_adc_curve_bitwidth() == ADC_BITWIDTH_DEFAULT);
    expect("and a reading comes out of it",
           ak_esp_adc_curve_mv(unit, channel) > 0);

    /* A chip whose eFuse carries no curve cannot be calibrated at all: IDF
     * refuses the scheme, and the port has to fall back to the datasheet's
     * straight line and say which it used. */
    host_adc_reset();
    host_adc_set_raw(ADC_UNIT_1, channel, 2048);
    host_adc_set_curve_refused(1);
    ak_esp_adc_curve_forget_for_host();
    report[0] = '\0';

    expect("a chip with no curve in eFuse still opens",
           ak_esp_adc_curve_init(unit, channel) == 0 &&
               host_adc_curve_created() == 0);
    expect("and still hands out a voltage",
           ak_esp_adc_curve_mv(unit, channel) > 0);
    ak_esp_adc_curve_report(sink);
    expect("and its report says that calibration is the datasheet's",
           strstr(report, "datasheet") != 0);
}
