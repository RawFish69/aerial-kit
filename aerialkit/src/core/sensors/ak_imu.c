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

const ak_imu_driver_t *const ak_imu_drivers[] = {
    &ak_imu_icm42688,
    &ak_imu_icm42605,
    &ak_imu_bmi270,
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
        out("imu:       %s\n", imu->driver->name);
    }
    return 0;
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
