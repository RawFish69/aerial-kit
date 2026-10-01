/*
 * The bias a gyro has when the aircraft is armed.
 *
 * Every MEMS gyro carries an offset, it moves with temperature, and the
 * control loop cannot tell it from a slow rotation - so it integrates into a
 * growing attitude error. A bench calibration measures it at whatever
 * temperature the bench was; the flight happens at another one, and arming is
 * the one moment in a flight where "the gyro should read zero" is both true
 * and known.
 *
 * What is checked here is the mechanism, with numbers rather than a claim: a
 * part carrying a bias the stored calibration does not know about, a heading
 * that walks away because of it, and the arm-time measurement removing it -
 * and the same measurement *refusing* to move a good bias when the aircraft
 * is moving, which is the case that would make it worse than useless.
 */

#include <math.h>
#include <stdio.h>

#include "ak_estimator.h"
#include "ak_gyro_cal.h"
#include "ak_math.h"
#include "tests.h"

/* A sample the way the flight loop sees one: the aligned gyro, with the part's
 * own offset in it and (optionally) a roll rate, which is what "the aircraft
 * is moving" looks like from a gyro. */
static ak_imu_sample_t gyro_sample(float bias_dps, float rate_dps,
                                   uint32_t time_ms)
{
    ak_imu_sample_t imu;

    imu.gyro[0] = ak_deg2rad(rate_dps);
    imu.gyro[1] = 0.0f;
    imu.gyro[2] = ak_deg2rad(bias_dps);
    imu.accel[0] = 0.0f;
    imu.accel[1] = 0.0f;
    imu.accel[2] = 1.0f;
    imu.time_ms = time_ms;
    imu.valid = 1;
    return imu;
}

/* What a stationary aircraft's heading does over `seconds`, with a gyro that
 * has `bias_dps` in it: the estimator is integrated exactly as the flight loop
 * integrates it, at 1 kHz. Returns the drift in degrees. */
static float heading_drift_deg(const ak_gyro_cal_t *cal, float bias_dps,
                               float seconds)
{
    ak_estimator_t est;
    float start;

    ak_estimator_init(&est, 0.5f);
    start = est.yaw;
    for (int i = 0; i < (int)(seconds * 1000.0f); i++) {
        ak_imu_sample_t imu = gyro_sample(bias_dps, 0.0f, (uint32_t)i);

        ak_gyro_cal_apply(cal, &imu);
        ak_estimator_update(&est, &imu, 0.001f);
    }
    return ak_rad2deg(est.yaw - start);
}

void test_gyro_arm_calibration(void)
{
    ak_gyro_cal_t cal;
    ak_imu_sample_t imu;
    float bias_dps[3];
    float drift;

    /*
     * A part with five degrees a second in it that nobody has calibrated, over
     * twenty seconds of a stationary aircraft: a hundred degrees of heading
     * with nothing whatever moving.
     */
    ak_gyro_cal_init(&cal, 200u, 0.05f, 0.35f);
    drift = heading_drift_deg(&cal, 5.0f, 20.0f);
    printf("        uncalibrated: %.0f deg of heading drift in 20 s\n",
           (double)drift);
    expect("a gyro bias walks a stationary aircraft's heading away",
           drift > 90.0f && drift < 110.0f);

    /*
     * And the arm-time measurement: two hundred milliseconds of a still
     * aircraft, which is what arming gives - the switch is held for half a
     * second with the throttle down.
     */
    ak_gyro_cal_start(&cal);
    for (int i = 0; i < 200; i++) {
        imu = gyro_sample(5.0f, 0.0f, (uint32_t)i);
        (void)ak_gyro_cal_feed(&cal, &imu);
    }
    expect("the measurement finishes in the time arming gives it",
           cal.done == 1 && cal.samples == 200u);
    ak_gyro_cal_bias_dps(&cal, bias_dps);
    expect("and it finds the bias the part is carrying",
           fabsf(bias_dps[2] - 5.0f) < 0.1f);

    drift = heading_drift_deg(&cal, 5.0f, 20.0f);
    printf("        after the arm-time measurement: %.1f deg in 20 s\n",
           (double)drift);
    expect("so the same twenty seconds leave the heading where it was",
           fabsf(drift) < 1.0f);

    /*
     * The case that would make this worse than useless: an aircraft that is
     * *moving* when a measurement starts - a switch flicked in the air, or a
     * pilot carrying it. Every sample is rejected, nothing is written, and the
     * bias it was calibrated with stays where it was.
     */
    ak_gyro_cal_start(&cal);
    for (int i = 0; i < 200; i++) {
        imu = gyro_sample(5.0f, 60.0f, (uint32_t)i); /* rolling at 60 dps */
        (void)ak_gyro_cal_feed(&cal, &imu);
    }
    ak_gyro_cal_bias_dps(&cal, bias_dps);
    expect("and a measurement taken while the aircraft is moving changes "
           "nothing",
           cal.done == 0 && cal.samples == 0u && cal.rejected == 200u &&
               fabsf(bias_dps[2] - 5.0f) < 0.1f);
}
