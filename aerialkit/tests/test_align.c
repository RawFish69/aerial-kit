/*
 * Board alignment and gyro bias.
 *
 * Both are wrong in a way that looks like something else: an unaligned board
 * flies as if the tuning were bad, and an uncalibrated gyro drifts into an
 * attitude nobody commanded. They are also both arithmetic, so they are
 * checked here rather than argued about.
 */

#include <math.h>
#include <stdint.h>

#include "ak_align.h"
#include "ak_gyro_cal.h"
#include "ak_math.h"
#include "tests.h"

static ak_imu_sample_t sample_with(float ax, float ay, float az, float gx,
                                   float gy, float gz)
{
    ak_imu_sample_t sample;
    sample.accel[0] = ax;
    sample.accel[1] = ay;
    sample.accel[2] = az;
    sample.gyro[0] = gx;
    sample.gyro[1] = gy;
    sample.gyro[2] = gz;
    sample.time_ms = 0;
    sample.valid = 1;
    return sample;
}

static int near3(const float v[3], float x, float y, float z)
{
    return fabsf(v[0] - x) < 1e-4f && fabsf(v[1] - y) < 1e-4f &&
           fabsf(v[2] - z) < 1e-4f;
}

static void test_alignment(void)
{
    ak_align_t align;
    ak_imu_sample_t sample;

    /* Zero angles is the identity: a board mounted square must not be quietly
     * rotated by the code that is supposed to leave it alone. */
    ak_align_set(&align, 0.0f, 0.0f, 0.0f);
    sample = sample_with(0.1f, 0.2f, 1.0f, 0.3f, 0.4f, 0.5f);
    ak_align_apply(&align, &sample);
    expect("no alignment angles means no rotation",
           near3(sample.accel, 0.1f, 0.2f, 1.0f) &&
           near3(sample.gyro, 0.3f, 0.4f, 0.5f));

    /* Yaw 90 degrees: gravity measured along the sensor's x is gravity along
     * the airframe's y. */
    ak_align_set(&align, 0.0f, 0.0f, 90.0f);
    sample = sample_with(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f);
    ak_align_apply(&align, &sample);
    expect("90 degrees of yaw moves x onto y", near3(sample.accel, 0.0f, 1.0f, 0.0f));
    expect("and leaves the rotation about z alone",
           fabsf(sample.gyro[2] - 0.5f) < 1e-4f);

    /* Roll 90 degrees: the sensor's z becomes the airframe's -y. */
    ak_align_set(&align, 90.0f, 0.0f, 0.0f);
    sample = sample_with(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    ak_align_apply(&align, &sample);
    expect("90 degrees of roll moves z onto -y",
           near3(sample.accel, 0.0f, -1.0f, 0.0f));

    /* 180 degrees of yaw is what somebody mounting a board backwards gets,
     * which is common enough to test. */
    ak_align_set(&align, 0.0f, 0.0f, 180.0f);
    sample = sample_with(1.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f);
    ak_align_apply(&align, &sample);
    expect("180 degrees of yaw turns the board around",
           near3(sample.accel, -1.0f, -0.5f, 0.0f));

    /* A rotation matrix has to stay orthonormal whatever the angles are. This
     * is the check that catches a sign or an order mistake in the closed form
     * the matrix is written out as. */
    ak_align_set(&align, 30.0f, 20.0f, 10.0f);
    int orthonormal = 1;
    for (int row = 0; row < 3; row++) {
        float length = 0.0f;
        for (int col = 0; col < 3; col++) {
            length += align.matrix[row][col] * align.matrix[row][col];
        }
        orthonormal = orthonormal && fabsf(length - 1.0f) < 1e-3f;
    }
    for (int a = 0; a < 3; a++) {
        for (int b = a + 1; b < 3; b++) {
            float dot = 0.0f;
            for (int col = 0; col < 3; col++) {
                dot += align.matrix[a][col] * align.matrix[b][col];
            }
            orthonormal = orthonormal && fabsf(dot) < 1e-3f;
        }
    }
    expect("the rotation matrix stays orthonormal", orthonormal);
}

static void test_gyro_calibration(void)
{
    ak_gyro_cal_t cal;
    ak_imu_sample_t sample;

    /* A gyro reporting 0.02 rad/s at rest, which is 1.15 dps. */
    ak_gyro_cal_init(&cal, 100, 0.05f, 0.35f);
    ak_gyro_cal_start(&cal);

    int finished = 0;
    for (int i = 0; i < 100; i++) {
        sample = sample_with(0.0f, 0.0f, 1.0f, 0.02f, -0.01f, 0.005f);
        finished = ak_gyro_cal_feed(&cal, &sample) || finished;
    }
    expect("the calibration finishes after the samples it asked for",
           finished == 1 && cal.done == 1);
    expect("and the bias is what the gyro was reporting",
           fabsf(cal.bias[0] - 0.02f) < 1e-5f &&
           fabsf(cal.bias[1] + 0.01f) < 1e-5f &&
           fabsf(cal.bias[2] - 0.005f) < 1e-5f);

    sample = sample_with(0.0f, 0.0f, 1.0f, 0.02f, -0.01f, 0.005f);
    ak_gyro_cal_apply(&cal, &sample);
    expect("applying it leaves a stationary gyro at zero",
           near3(sample.gyro, 0.0f, 0.0f, 0.0f));

    /* A calibration done while somebody is moving the aircraft is worse than
     * none, so those samples are refused rather than averaged in. */
    ak_gyro_cal_start(&cal);
    for (int i = 0; i < 50; i++) {
        sample = sample_with(0.0f, 0.0f, 1.0f, 2.0f, 0.0f, 0.0f);
        (void)ak_gyro_cal_feed(&cal, &sample);
    }
    expect("samples that show movement are rejected",
           cal.samples == 0 && cal.rejected == 50 && cal.done == 0);

    /* Calibrating twice refines rather than doubles the correction. */
    float zero_dps[3] = { 0.0f, 0.0f, 0.0f };
    ak_gyro_cal_set_bias_dps(&cal, zero_dps);
    ak_gyro_cal_start(&cal);
    for (int i = 0; i < 100; i++) {
        sample = sample_with(0.0f, 0.0f, 1.0f, 0.02f, 0.0f, 0.0f);
        (void)ak_gyro_cal_feed(&cal, &sample);
    }
    float first_pass = cal.bias[0];

    ak_gyro_cal_start(&cal);
    for (int i = 0; i < 100; i++) {
        sample = sample_with(0.0f, 0.0f, 1.0f, 0.02f, 0.0f, 0.0f);
        (void)ak_gyro_cal_feed(&cal, &sample);
    }
    expect("a second calibration does not double the bias",
           fabsf(cal.bias[0] - first_pass) < 1e-5f);

    /* The parameter table stores degrees per second, so the conversion has to
     * survive the round trip. */
    float dps[3] = { 1.5f, -0.25f, 0.0f };
    ak_gyro_cal_set_bias_dps(&cal, dps);
    ak_gyro_cal_bias_dps(&cal, dps);
    expect("bias survives a round trip through degrees per second",
           fabsf(dps[0] - 1.5f) < 1e-3f && fabsf(dps[1] + 0.25f) < 1e-3f);
}

void test_align_and_calibration(void)
{
    test_alignment();
    test_gyro_calibration();
}
