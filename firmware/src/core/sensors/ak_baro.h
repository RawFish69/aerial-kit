#ifndef AK_SENSORS_AK_BARO_H
#define AK_SENSORS_AK_BARO_H

#include "ak_bus.h"
#include "ak_console.h"

/*
 * Barometers behind one interface, the way inertial sensors are.
 *
 * A barometer is the one sensor that measures *height* directly, and height is
 * the axis this aircraft flies worst: a wing holding altitude on GPS alone is
 * holding it on a measurement that lags seconds behind and wanders a few metres
 * while it catches up. Pressure responds immediately and is repeatable to a few
 * tens of centimetres over a flight - not absolute, because the weather moves,
 * which is why what matters is the *change* since take-off.
 *
 * So this interface hands out pressure in pascals and a temperature to
 * compensate it with, and the arithmetic that turns those into a height lives
 * next to the arithmetic that turns a gyro into an angle: in the core, where a
 * host test can hold it against the standard atmosphere.
 */

typedef struct {
    float    pressure_pa;
    float    temperature_c;
    uint32_t time_ms;
    int      valid;
} ak_baro_sample_t;

typedef struct {
    const char *name;
    uint8_t     whoami_reg;
    uint8_t     whoami_value;
    int (*init)(const ak_bus_t *bus, ak_printf_fn out);
    int (*read)(const ak_bus_t *bus, ak_baro_sample_t *sample);
} ak_baro_driver_t;

typedef struct {
    const ak_bus_t         *bus;
    const ak_baro_driver_t *driver;
    uint32_t                samples;
    uint32_t                errors;
    int                     present;
} ak_baro_t;

/* Every driver this build knows about, ending with a null entry - and the
 * drivers themselves, so a test can say which one it expected rather than only
 * that something answered. */
extern const ak_baro_driver_t ak_baro_dps310;
extern const ak_baro_driver_t ak_baro_spl06;
extern const ak_baro_driver_t ak_baro_bmp280;
extern const ak_baro_driver_t ak_baro_bme280;
extern const ak_baro_driver_t ak_baro_bmp388;
extern const ak_baro_driver_t ak_baro_bmp390;

extern const ak_baro_driver_t *const ak_baro_drivers[];

/* The first driver whose who-am-i answers, or null - the same probe-first rule
 * the IMU uses, because a barometer read as the wrong part is a plausible
 * altitude in the wrong units. */
const ak_baro_driver_t *ak_baro_detect(const ak_bus_t *bus,
                                       uint8_t *whoami_seen);

int ak_baro_open(ak_baro_t *baro, const ak_bus_t *bus, ak_printf_fn out);

/* One measurement, when the part has one. Returns 1 and fills `sample` when
 * there was a new one, 0 when there was not, and negative on a bus error. */
int ak_baro_read(ak_baro_t *baro, ak_baro_sample_t *sample);

/*
 * Height above a reference pressure, by the international standard atmosphere:
 *
 *   h = 44330 * (1 - (p / p0) ^ 0.190295)
 *
 * The reference is the pressure where the aircraft is standing, not sea level:
 * what a flight controller wants is how far it has moved since take-off, and
 * that is also what makes the number immune to the weather changing.
 */
float ak_baro_altitude_m(float pressure_pa, float reference_pa);

#endif /* AK_SENSORS_AK_BARO_H */
