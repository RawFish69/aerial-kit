#include "ak_imu.h"

/*
 * The parts this build knows about, in the order they are probed.
 *
 * A board asks for "an IMU on this bus" and gets whichever of these answers,
 * which is the whole point of the probe: a board that changes sensor between
 * revisions still boots, and a driver that is wrong about its part fails here
 * instead of producing plausible numbers from the wrong registers.
 */
extern const ak_imu_driver_t ak_imu_icm42688;
extern const ak_imu_driver_t ak_imu_icm42605;
extern const ak_imu_driver_t ak_imu_mpu6000;
extern const ak_imu_driver_t ak_imu_mpu6500;
extern const ak_imu_driver_t ak_imu_mpu9250;
extern const ak_imu_driver_t ak_imu_bmi270;
extern const ak_imu_driver_t ak_imu_lsm6dso;
extern const ak_imu_driver_t ak_imu_bno055;

const ak_imu_driver_t *const ak_imu_drivers[] = {
    &ak_imu_icm42688,
    &ak_imu_icm42605,
    &ak_imu_bmi270,
    /*
     * Before the LSM6DSO, and on purpose. The LSM6DSO's who-am-i is register
     * 0x0F, and on a BNO055 that is the high byte of the magnetometer's x axis:
     * after a warm reset of the board (which does not reset a breakout on the
     * Qwiic cable) the part is still streaming, and one reading in 256 of that
     * byte is 0x6C. Probed in the other order, a BNO055 would now and then boot
     * as an LSM6DSO and be configured through registers it does not have. The
     * BNO055's own who-am-i is register 0x00, a reserved register on an
     * LSM6DSO, and the BMI270 above it answers 0x24 there rather than 0xA0.
     */
    &ak_imu_bno055,
    &ak_imu_lsm6dso,
    &ak_imu_mpu6000,
    &ak_imu_mpu6500,
    &ak_imu_mpu9250,
    0,
};

const ak_imu_driver_t *ak_imu_detect(const ak_bus_t *bus, uint8_t *whoami_seen)
{
    if (whoami_seen != 0) {
        *whoami_seen = 0;
    }
    for (const ak_imu_driver_t *const *slot = ak_imu_drivers; *slot != 0;
         slot++) {
        const ak_imu_driver_t *driver = *slot;
        uint8_t value = 0;
        if (ak_bus_read(bus, driver->whoami_reg, &value, 1) != 0) {
            continue;
        }
        if (value == driver->whoami_value) {
            return driver;
        }
        if (whoami_seen != 0 && *whoami_seen == 0) {
            *whoami_seen = value;
        }
    }
    return 0;
}

/*
 * The last open's verdict, kept for a console that is asked afterwards. Both
 * start at "nothing answered", which is what is true of a board where no open
 * has run - see the note on `ak_imu_last_result()`.
 */
static ak_imu_result_t last_result = AK_IMU_NOBODY;
static uint8_t         last_whoami;

ak_imu_result_t ak_imu_last_result(void)
{
    return last_result;
}

uint8_t ak_imu_last_whoami(void)
{
    return last_whoami;
}

int ak_imu_open(ak_imu_t *imu, const ak_bus_t *bus, ak_printf_fn out)
{
    imu->bus = bus;
    imu->samples = 0;
    imu->errors = 0;
    imu->present = 0;
    imu->rate_hz = 0;
    uint8_t whoami = 0;
    imu->driver = ak_imu_detect(bus, &whoami);
    last_whoami = whoami;

    if (imu->driver == 0) {
        last_result = whoami != 0 ? AK_IMU_UNKNOWN_PART : AK_IMU_NOBODY;
        if (out != 0) {
            if (whoami != 0) {
                /* Something answered and it is not a part this build knows.
                 * Saying so saves checking the wiring twice. */
                out("imu:       answered 0x%02x, which is not a known part\n",
                    whoami);
            } else {
                out("imu:       nothing answered on the bus\n");
            }
        }
        return -1;
    }

    if (imu->driver->init(bus, out) != 0) {
        last_result = AK_IMU_NO_CONFIG;
        if (out != 0) {
            out("imu:       %s answered but would not configure\n",
                imu->driver->name);
        }
        return -1;
    }

    imu->present = 1;
    last_result = AK_IMU_OK;
    if (out != 0) {
        const uint8_t address = ak_bus_address(bus);
        if (address != 0u) {
            out("imu:       %s at 0x%02x\n", imu->driver->name, address);
        } else {
            out("imu:       %s\n", imu->driver->name);
        }
    }
    return 0;
}

uint32_t ak_imu_set_rate(ak_imu_t *imu, uint32_t hz, ak_printf_fn out)
{
    if (imu == 0 || !imu->present || imu->driver == 0) {
        return 0;
    }
    if (imu->driver->set_rate == 0) {
        if (out != 0) {
            /*
             * Not "whatever it powered up with". A driver with no rate hook
             * still configures its part at init - the BNO055's writes its
             * accelerometer and gyro bandwidth registers there - so the rate
             * is one the driver *chose* and then cannot state, which is a
             * different thing from the part's power-on default and a different
             * thing from a rate that is zero. Corrected 2026-10-06, on the day
             * the BNO055 became the only driver taking this branch: the old
             * sentence described a part that never had a register written.
             */
            out("imu:       %s: this driver cannot set the rate, so the part "
                "is left at the rate this driver's init chose for it\n",
                imu->driver->name);
        }
        return 0;
    }

    const uint32_t took = imu->driver->set_rate(imu->bus, hz, out);

    if (took == 0) {
        if (out != 0) {
            out("imu:       %s: %u Hz is not a rate this part can take; the "
                "rate is unchanged at %u Hz\n",
                imu->driver->name, hz, imu->rate_hz);
        }
        return 0;
    }

    imu->rate_hz = took;
    /* Said even when the part took exactly what was asked for. The number a
     * person needs is not "the request was accepted" but "the part is at this
     * rate", and on the parts whose tables do not contain every value those are
     * two different numbers often enough to print every time. */
    if (out != 0) {
        out("imu:       %u Hz requested, part at %u Hz\n", hz, took);
    }
    return took;
}

int ak_imu_read(ak_imu_t *imu, ak_imu_sample_t *sample)
{
    sample->valid = 0;

    if (!imu->present || imu->driver == 0) {
        return -1;
    }
    if (imu->driver->read(imu->bus, sample) != 0) {
        imu->errors++;
        return -1;
    }

    imu->samples++;
    sample->valid = 1;
    return 0;
}
