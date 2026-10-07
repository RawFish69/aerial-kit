#ifndef AK_SENSORS_AK_IMU_BNO055_H
#define AK_SENSORS_AK_IMU_BNO055_H

#include "ak_bus.h"
#include "ak_imu.h"

/*
 * The parts of the BNO055 that are not "an IMU".
 *
 * The flight core sees this part through ak_imu_driver_t like every other one:
 * a who-am-i, an init and a read of gyro and accelerometer. What it does not
 * see is the reason anyone fits a BNO055 - the Bosch fusion processor on the
 * same die, its calibration state and its power-on self test. Those are here,
 * as plain reads on the same bus, for the console and for a bench session to
 * ask; nothing in the control loop calls them.
 *
 * Why the loop does not fly the part's own attitude is written at the top of
 * ak_imu_bno055.c. The short version: the fusion output is 100 Hz and the
 * part's fusion modes lock the gyro's range and filter, while the raw
 * accelerometer-magnetometer-gyro mode this driver uses gives the firmware's
 * own estimator its sensors at the rate it runs.
 */

/*
 * CALIB_STAT, split. Each field is 0 (uncalibrated) to 3 (fully calibrated).
 *
 * These are the **fusion processor's** calibration states, and they only move
 * while the part is in a fusion mode. In the AMG mode this driver leaves the
 * part in, the fusion processor is idle and all four read 0 - which is the
 * truth about the fusion processor and says nothing about the raw sensors the
 * firmware reads. The firmware calibrates those itself (ak_gyro_cal.c,
 * ak_accel_cal.c). The console prints this caveat beside the numbers rather
 * than letting four zeros read as four faults.
 */
typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} ak_bno055_calib_t;

/*
 * ST_RESULT, split: the power-on self test the part runs on itself at every
 * boot. 1 means that block passed. Unlike the calibration states this *is*
 * meaningful in any mode - it is the one register that says the accelerometer,
 * magnetometer and gyro on the die are alive before a single sample is read.
 */
typedef struct {
    uint8_t accel;
    uint8_t mag;
    uint8_t gyro;
    uint8_t mcu;
} ak_bno055_selftest_t;

/* The driver itself, so the console can ask "is the part on this bus the
 * BNO055" by identity rather than by comparing names. */
extern const ak_imu_driver_t ak_imu_bno055;

/* 0 on a read, negative when the bus refused it. */
int ak_bno055_read_calib(const ak_bus_t *bus, ak_bno055_calib_t *calib);
int ak_bno055_read_selftest(const ak_bus_t *bus, ak_bno055_selftest_t *st);

/*
 * SYS_STATUS (0 idle, 1 system error, 5 fusion running, 6 running without
 * fusion - the value this driver expects) and SYS_ERR (0 when status is not 1).
 * Both raw, because the datasheet's table of them is the reference and a
 * paraphrase of it would be a second table to keep in step.
 */
int ak_bno055_read_status(const ak_bus_t *bus, uint8_t *status, uint8_t *error);

/* The operating mode the part reports it is in (OPR_MODE's low nibble). */
int ak_bno055_read_mode(const ak_bus_t *bus, uint8_t *mode);

/*
 * How long this part took to agree it had changed mode, in milliseconds, on the
 * last switch init made - 0 if none has been made yet.
 *
 * Datasheet table 3-6 gives that time as a typical, and a typical is not a
 * bound: the driver polls the register until the part agrees rather than
 * sleeping for the table's number. This is what that poll cost, kept so the
 * console can print a measurement of the fitted part instead of the table's
 * figure, and so a board whose part is slower than the table says so.
 */
unsigned ak_bno055_last_switch_ms(void);

/* The mode init leaves the part in: accelerometer, magnetometer and gyro, raw,
 * no fusion. Exposed so a test and the console can compare against it. */
#define AK_BNO055_MODE_CONFIG 0x00u
#define AK_BNO055_MODE_AMG    0x07u

/* The two SYS_STATUS values this driver waits on: idle, which is what the part
 * reports sitting in CONFIG with nothing running and is how init knows its
 * power-on self test is over, and running without fusion, which is what AMG
 * produces. */
#define AK_BNO055_STATUS_IDLE      0x00u
#define AK_BNO055_STATUS_NO_FUSION 0x06u

#endif /* AK_SENSORS_AK_IMU_BNO055_H */
