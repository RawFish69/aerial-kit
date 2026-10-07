#ifndef AK_SENSORS_AK_IMU_H
#define AK_SENSORS_AK_IMU_H

#include "ak_bus.h"
#include "ak_console.h"
#include "ak_types.h"

/*
 * Inertial sensors behind one interface.
 *
 * A driver is a name, a who-am-i, and two functions - the parts that differ
 * between an ICM-42688-P and an MPU-6000. Everything else is shared, and a
 * board asks for "an IMU on this bus" rather than for a part number it has to
 * know in advance.
 *
 * Probe is the first test and the cheapest one: every supported part answers a
 * who-am-i register, so a driver that is wrong about its part fails at `open`
 * instead of producing plausible numbers from the wrong registers.
 */

typedef struct {
    const char *name;
    uint8_t     whoami_reg;
    uint8_t     whoami_value;
    int (*init)(const ak_bus_t *bus, ak_printf_fn out);
    int (*read)(const ak_bus_t *bus, ak_imu_sample_t *sample);
    /*
     * Turn the gyro's data-ready output on interrupt pin 1 on, or off.
     *
     * `init` already leaves the part routing data-ready to INT1 - all four
     * drivers here do, because on the boards these parts were written against
     * that line is wired and the interrupt is the point of it. This exists for
     * the other case: a board whose IMU has no INT pad, where the part would be
     * driving a trace that goes nowhere. Nothing is wrong with that
     * electrically - it is an output into an open pad - but "the firmware
     * raises an interrupt it is not listening to" is a thing a person reading a
     * board should be able to see and turn off, and a board that says it has no
     * INT pin should be able to make the part match.
     *
     * Null on a part with no such output. Returns 0 when the part took the
     * change and negative when the bus refused it, so a caller that is turning
     * the interrupt *off* for a board with no pin can treat a failure as
     * cosmetic and a caller that is turning it on cannot.
     *
     * The gyro's output data rate is deliberately not an argument, and now that
     * phase 1.4 has landed it is worth saying why it lives in its own hook
     * instead. Routing data-ready to INT1 and choosing how often that line
     * pulses are two acts: the first is a board's wiring question and the second
     * is a rate, and a board that turns the interrupt off has not thereby
     * changed the rate the part samples at. Folding the rate in would have made
     * `configure_drdy(bus, 0)` mean either "stop telling me" or "stop sampling",
     * and the off path's whole safety argument is that it undoes exactly one
     * load-bearing bit.
     */
    int (*configure_drdy)(const ak_bus_t *bus, int enable);

    /*
     * Set the rate the part samples at, in Hz, and answer the rate it took.
     *
     * The answer is what the part was actually programmed to and not an echo of
     * the request, because every part here can only take the rates its own
     * register's table offers. The rule the drivers follow is *the fastest rate
     * that part supports which is not above the one asked for*: an aircraft
     * told to run at 3 kHz and given 4 kHz would be running the loop faster
     * than the frame it was configured for, and one given nothing at all would
     * keep the rate `init` programmed, which is the silent answer this exists
     * to replace.
     *
     * Return 0 when this part has no rate this driver can set, or when the
     * request is below every rate the part offers - in both cases the part is
     * left exactly as it was, and a caller that gets 0 must not read it as
     * "0 Hz". `out` may be null and a driver may use it to say something the
     * single number cannot: the BMI270's gyro can run at 3200 Hz and its
     * accelerometer tops out at 1600, and that is a sentence rather than a
     * mismatch to be quietly rounded away.
     *
     * Null on a driver whose part cannot be re-rated.
     *
     * Whether the accelerometer moves too is per part and each driver's own
     * comment says which it is: on the MPU family one divider drives both
     * sensors, on the ICM the two configuration registers take the same ODR
     * field, on a BMI270 the accelerometer stops at 1600 Hz whatever the gyro
     * does, and on the LSM6DSO the accelerometer is deliberately left at the
     * eighth of the gyro's rate the reference implementation puts it at. What
     * the number this returns always describes is the gyro, because that is
     * what the control loop and `gyro_rate_hz` are about.
     */
    uint32_t (*set_rate)(const ak_bus_t *bus, uint32_t hz, ak_printf_fn out);
} ak_imu_driver_t;

typedef struct {
    const ak_bus_t        *bus;
    const ak_imu_driver_t *driver;
    uint32_t               samples;
    uint32_t               errors;
    int                    present;
    /*
     * The rate the part is programmed to, in Hz, as the driver answered it -
     * zero until a driver has stated one.
     *
     * Zero is "not stated" and not "stopped". A part `init` has configured but
     * that no driver can re-rate is still sampling at whatever `init` chose,
     * and the honest answer for it is that this firmware cannot say which -
     * which is a different sentence from "zero hertz", and the console's report
     * prints it as such.
     */
    uint32_t               rate_hz;
} ak_imu_t;

/* Every driver this build knows about, ending with a null entry. Pointers
 * rather than a copy, because a C static initialiser cannot be built from
 * another object's value - and because a driver is one instance, not a value
 * to be duplicated. */
extern const ak_imu_driver_t *const ak_imu_drivers[];

/* Reads the who-am-i register of each known driver and returns the first match,
 * or null. A bus that cannot be read at all - nothing wired, or the wrong pins
 * - returns null rather than a guess. */
const ak_imu_driver_t *ak_imu_detect(const ak_bus_t *bus, uint8_t *whoami_seen);

/* Detect and configure. Returns 0 when a sensor answered and took its
 * configuration, negative otherwise. `out` may be null; a driver prints what it
 * found there, because "which part is on this board" is a question a bench
 * session asks. */
int ak_imu_open(ak_imu_t *imu, const ak_bus_t *bus, ak_printf_fn out);

/*
 * Why the last `ak_imu_open()` went the way it did, kept so that it can be
 * asked again.
 *
 * The open prints its verdict once, at boot, and on this board that print is
 * before the host can attach - so the sentence that says whether nothing is on
 * the bus or the wrong thing is, has been the one line a bench session cannot
 * get back. `imu` and the preflight then report only that there is no sensor,
 * which is the same answer for a missing part, a miswired one and a part this
 * build has no driver for - three different repairs.
 *
 * The four outcomes are kept apart on purpose. NOBODY means no address
 * acknowledged, which is wiring or a part that is not there; UNKNOWN_PART means
 * something answered and it is not one of `ak_imu_drivers`, which is a different
 * fault with a different fix; NO_CONFIG means the right part answered and would
 * not take its configuration, which is marginal wiring rather than a missing
 * part. A report that collapsed them would be no better than `none fitted`.
 */
typedef enum {
    AK_IMU_OK = 0,       /* a driver matched and configured */
    AK_IMU_NOBODY,       /* nothing answered on the bus */
    AK_IMU_UNKNOWN_PART, /* something answered, and it is not a known part */
    AK_IMU_NO_CONFIG     /* the right part answered and would not configure */
} ak_imu_result_t;

/* The outcome of the last `ak_imu_open()`. Before one has run, AK_IMU_NOBODY is
 * what it says, because that is the truth about a board on which no open has
 * happened - the same thing an open that found nothing would leave. */
ak_imu_result_t ak_imu_last_result(void);

/* The who-am-i byte the last open read and did not recognise, valid only when
 * `ak_imu_last_result()` is AK_IMU_UNKNOWN_PART or AK_IMU_NO_CONFIG - 0
 * otherwise, which is also what "nothing answered" reads as. */
uint8_t ak_imu_last_whoami(void);

/* One sample, scaled to rad/s and g. Returns 0 on success and sets
 * sample->valid; on failure the sample keeps valid = 0, which is what the
 * flight core treats as "no attitude". */
int ak_imu_read(ak_imu_t *imu, ak_imu_sample_t *sample);

/*
 * Ask the part for `hz`, and answer what it took.
 *
 * The return is the part's new rate when it took one, and 0 when it did not -
 * because the driver has no setter, because the request is below every rate the
 * part offers, or because the bus refused the write. A caller cannot tell those
 * three apart from the number, which is why this also prints the sentence: on a
 * bench the question is "why is my aircraft at 1 kHz when I set 8", and the
 * answer is a part, a table or a wire.
 *
 * `imu->rate_hz` is updated to the new rate when one was taken and left alone
 * when none was, so it never goes backwards to zero on a failed write.
 */
uint32_t ak_imu_set_rate(ak_imu_t *imu, uint32_t hz, ak_printf_fn out);

#endif /* AK_SENSORS_AK_IMU_H */
