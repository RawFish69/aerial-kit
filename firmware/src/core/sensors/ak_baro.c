#include "ak_baro.h"

/*
 * The part of a barometer that is the same for every part.
 *
 * A driver is a name, a who-am-i and two functions; a board asks for "a
 * barometer on this bus" rather than for a part number it has to know in
 * advance. Same shape as the IMU, for the same reason: the probe happens before
 * anything is believed.
 */

extern const ak_baro_driver_t ak_baro_dps310;
extern const ak_baro_driver_t ak_baro_spl06;
extern const ak_baro_driver_t ak_baro_bmp280;
extern const ak_baro_driver_t ak_baro_bme280;
extern const ak_baro_driver_t ak_baro_bmp388;
extern const ak_baro_driver_t ak_baro_bmp390;

const ak_baro_driver_t *const ak_baro_drivers[] = {
    &ak_baro_dps310,
    &ak_baro_spl06,
    &ak_baro_bmp280,
    &ak_baro_bme280,
    &ak_baro_bmp388,
    &ak_baro_bmp390,
    0,
};

const ak_baro_driver_t *ak_baro_detect(const ak_bus_t *bus,
                                       uint8_t *whoami_seen)
{
    if (bus == 0) {
        return 0;
    }

    for (const ak_baro_driver_t *const *slot = ak_baro_drivers; *slot != 0;
         slot++) {
        const ak_baro_driver_t *driver = *slot;
        uint8_t value = 0;

        /* A bus that cannot be read at all - nothing wired, the wrong pins, the
         * wrong chip select - is not a barometer that is missing: there is
         * nothing to ask. */
        if (ak_bus_read(bus, driver->whoami_reg, &value, 1) != 0) {
            return 0;
        }
        if (whoami_seen != 0) {
            *whoami_seen = value;
        }
        if (value == driver->whoami_value) {
            return driver;
        }
    }
    return 0;
}

int ak_baro_open(ak_baro_t *baro, const ak_bus_t *bus, ak_printf_fn out)
{
    uint8_t whoami = 0;
    const ak_baro_driver_t *driver;

    baro->bus = bus;
    baro->driver = 0;
    baro->samples = 0;
    baro->errors = 0;
    baro->present = 0;

    driver = ak_baro_detect(bus, &whoami);
    if (driver == 0) {
        if (out != 0) {
            out("baro:      none (who-am-i 0x%02x)\n", whoami);
        }
        return -1;
    }

    if (driver->init(bus, out) != 0) {
        if (out != 0) {
            out("baro:      %s did not configure\n", driver->name);
        }
        return -1;
    }

    baro->driver = driver;
    baro->present = 1;
    return 0;
}

int ak_baro_read(ak_baro_t *baro, ak_baro_sample_t *sample)
{
    int got;

    sample->valid = 0;
    if (!baro->present) {
        return -1;
    }

    got = baro->driver->read(baro->bus, sample);
    if (got < 0) {
        baro->errors++;
        sample->valid = 0;
        return -1;
    }
    if (got > 0) {
        baro->samples++;
    }
    return got;
}

/*
 * Height above a reference pressure.
 *
 * The standard atmosphere says
 *
 *   h = 44330 * (1 - (p / p0) ^ 0.190295)
 *
 * and this firmware does not link libm, so there is no pow() to call. What it
 * does have is the binomial series for (1 - e)^a with e = 1 - p/p0, which is
 * small near the ground and is the only place this is used:
 *
 *   1 - (1 - e)^a = a e + 0.0770521 e^2 + 0.0464755 e^3 + 0.0326457 e^4
 *                      + 0.0248736 e^5 + ...
 *
 * The terms are all the same sign for 0 < a < 1, and they get small quickly
 * *because* e is small: a thousand metres is e = 0.113. Five terms are within
 * two centimetres at that height and far better below it, and the host test
 * checks the whole curve against libm rather than trusting that sentence.
 *
 * Above e = 0.35 - about 4.5 km up, which is not a height this flies at - the
 * series is clamped rather than allowed to diverge: a wrong number that stays
 * inside the range of the arithmetic is better than an infinity in a control
 * loop.
 */
float ak_baro_altitude_m(float pressure_pa, float reference_pa)
{
    if (pressure_pa <= 0.0f || reference_pa <= 0.0f) {
        return 0.0f;
    }

    float e = 1.0f - pressure_pa / reference_pa;
    if (e > 0.35f) {
        e = 0.35f;
    } else if (e < -0.35f) {
        e = -0.35f;
    }

    float e2 = e * e;
    float e3 = e2 * e;
    float e4 = e3 * e;
    float e5 = e4 * e;

    float series = 0.190295f * e + 0.0770521f * e2 + 0.0464755f * e3 +
                   0.0326457f * e4 + 0.0248736f * e5;
    return 44330.0f * series;
}
