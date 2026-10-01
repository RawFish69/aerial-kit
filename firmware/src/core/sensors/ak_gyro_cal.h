#ifndef AK_SENSORS_AK_GYRO_CAL_H
#define AK_SENSORS_AK_GYRO_CAL_H

#include "ak_types.h"

/*
 * Gyro bias: the offset a rate sensor reports when it is not moving.
 *
 * Every MEMS gyro has one, it drifts with temperature, and the control loop
 * cannot tell it from a slow rotation - so it integrates into a growing
 * attitude error until the aircraft is quietly holding an angle nobody asked
 * for. Measuring it is a matter of sitting still for a fraction of a second.
 *
 * The accumulator stores the *residual* - what the gyro reports after the
 * current compensation is removed - so calibrating twice converges instead of
 * doubling the correction. Samples that show the aircraft moving are rejected
 * rather than averaged in: a calibration done while somebody leans on the desk
 * is worse than no calibration, because it is wrong in a way that looks
 * deliberate.
 *
 * "Moving" is judged by *steadiness*, not by the size of the rate: a sample
 * counts as still when it sits close to the mean of the samples already
 * accepted. The obvious test - "the rate must be near zero" - cannot work,
 * because the offset being measured *is* a rate: a part with five degrees a
 * second in it fails a three-degree test on every sample and is never
 * calibrated at all. Betaflight makes the same distinction by refusing to
 * finish a calibration while the readings are still changing.
 */

typedef struct {
    float    bias[3];      /* rad/s, subtracted from every sample */
    float    sum[3];       /* residual accumulated so far */
    float    accel_sum[3]; /* the direction the aircraft was held in */
    uint32_t wanted;
    uint32_t samples;
    /* rad/s a sample may differ from the running mean before it counts as
     * "the aircraft was moving" - see the note above the struct. */
    float    max_rate;
    /* And the largest rate that could be a part's offset at all: a gyro more
     * than this far out is not a part, it is a plot. It is a sanity bound
     * rather than the still test, and the two are different questions. */
    float    max_bias;
    uint32_t rejected;
    int      running;
    int      done;
} ak_gyro_cal_t;

void ak_gyro_cal_init(ak_gyro_cal_t *cal, uint32_t wanted, float max_rate,
                      float max_bias);

/* Starts a calibration. The existing bias stays in place and is refined. */
void ak_gyro_cal_start(ak_gyro_cal_t *cal);

/* One sample. Returns 1 when this sample completed the calibration. */
int ak_gyro_cal_feed(ak_gyro_cal_t *cal, const ak_imu_sample_t *sample);

/* Removes the bias in place. Safe before any calibration: it does nothing. */
void ak_gyro_cal_apply(const ak_gyro_cal_t *cal, ak_imu_sample_t *sample);

/* Bias in degrees per second, for the parameter table and the console. */
void ak_gyro_cal_bias_dps(const ak_gyro_cal_t *cal, float bias_dps[3]);
void ak_gyro_cal_set_bias_dps(ak_gyro_cal_t *cal, const float bias_dps[3]);

#endif /* AK_SENSORS_AK_GYRO_CAL_H */
