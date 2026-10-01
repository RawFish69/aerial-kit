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
} ak_imu_driver_t;

typedef struct {
    const ak_bus_t        *bus;
    const ak_imu_driver_t *driver;
    uint32_t               samples;
    uint32_t               errors;
    int                    present;
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

#endif /* AK_SENSORS_AK_IMU_H */
