#ifndef AK_HOST_ESP32_ADC_MODEL_H
#define AK_HOST_ESP32_ADC_MODEL_H

/*
 * The modelled ADC, and the IDF calls `src/arch/esp32/adc.c` makes. Compiled
 * *instead of* IDF, so the real port file runs against it.
 *
 * What it is: a one-shot converter that hands back whatever raw reading the
 * test put in it for a unit and a channel, and a line-fitting calibration that
 * turns a reading into millivolts through two points - which is what line
 * fitting *is* on this chip, and the reason the port never converts counts to
 * volts itself.
 *
 * What it is not: a converter. There is no attenuation curve, no noise, no
 * sample time, and no eFuse: the two points are the ones the test sets, and a
 * chip whose eFuse has nothing in it is a flag rather than a fuse. Every
 * number this model produces is therefore a number the test chose - what it
 * checks is that the *port* asked for the right unit, channel and attenuation,
 * used the calibration it was given, reported which calibration that was, and
 * turned failures into errors rather than readings.
 */

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

/* Back to the state a chip comes out of reset in: no handle, no reading, the
 * chip's own two-point calibration, nothing failing. */
void host_adc_reset(void);

/* The eFuse, as the three cases the port distinguishes between: two measured
 * points, a reference voltage, or nothing at all. */
void host_adc_efuse_two_points(int low_raw, int low_mv, int high_raw,
                               int high_mv);
void host_adc_efuse_reference(int reference_mv);
void host_adc_efuse_empty(void);

/* The reading the converter hands back for one unit and channel, and the
 * switches for the three IDF calls the port makes. */
void host_adc_set_raw(int unit, int channel, int raw);
void host_adc_fail_unit(int fail);
void host_adc_fail_channel(int fail);
void host_adc_fail_read(int fail);
void host_adc_fail_scheme(int fail);

/* What the port asked IDF for. The unit is IDF's own enum, so the check can
 * tell ADC_UNIT_1 (0) from ADC_UNIT_2 (1) - which is the whole reason this
 * model exists: a board header that says "ADC1 channel 6" is asking for the
 * unit GPIO34 belongs to, and IDF counts that unit from zero. */
int host_adc_unit(void);
int host_adc_channel(void);
int host_adc_atten(void);
int host_adc_bitwidth(void);
unsigned host_adc_units_made(void);
unsigned host_adc_channels_configured(void);
unsigned host_adc_reads(void);
/* The reference the port handed the calibration scheme; 0 when it was not
 * asked for one because the eFuse had a calibration of its own. */
unsigned host_adc_default_vref(void);

/*
 * The curve-fitting scheme, which every chip after the original ESP32 uses.
 * What the port asked it for is the check that matters: a curve is per
 * *channel*, so a port that passed the unit where the channel goes would
 * calibrate against another pin's calibration - and only these chips notice.
 */
int  host_adc_curve_created(void);
int  host_adc_curve_unit(void);
int  host_adc_curve_channel(void);
int  host_adc_curve_atten(void);
int  host_adc_curve_bitwidth(void);
/* A chip whose eFuse has no curve: IDF refuses to create the scheme. */
void host_adc_set_curve_refused(int refused);

#endif /* AK_HOST_ESP32_ADC_MODEL_H */
