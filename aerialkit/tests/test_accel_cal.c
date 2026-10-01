/*
 * Accelerometer calibration, six positions.
 *
 * The arithmetic is easy to get wrong in a way that is invisible: a sign, a
 * scale that is really 1/scale, a bias applied after the scale instead of
 * before. So this does not check the formula - it puts a *known* part on six
 * faces, gives the calibration the readings that part would produce, and asks
 * whether what comes out is the part's real error. If the correction is wrong
 * in any of those ways, the recovered bias or scale is wrong in the same way.
 */

#include <math.h>

#include "ak_accel_cal.h"
#include "tests.h"

/* A part with this bias and this scale, in the airframe's axes. */
#define PART_BIAS_X  0.032f
#define PART_BIAS_Y -0.018f
#define PART_BIAS_Z  0.045f
#define PART_SCALE_X 0.985f
#define PART_SCALE_Y 1.021f
#define PART_SCALE_Z 0.994f

/* What that part reads with `axis` pointing up (sign +1) or down (sign -1). */
static void part_reading(int axis, float sign, float out[3])
{
    const float bias[3] = { PART_BIAS_X, PART_BIAS_Y, PART_BIAS_Z };
    const float scale[3] = { PART_SCALE_X, PART_SCALE_Y, PART_SCALE_Z };

    for (int i = 0; i < 3; i++) {
        out[i] = bias[i];
    }
    out[axis] += sign / scale[axis];
}

static void feed_face(ak_accel_cal_t *cal, int face, int axis, float sign,
                      uint32_t samples)
{
    ak_accel_cal_begin(cal, face);
    for (uint32_t i = 0; i < samples; i++) {
        ak_imu_sample_t sample;
        for (int j = 0; j < 3; j++) {
            sample.gyro[j] = 0.0f;
        }
        part_reading(axis, sign, sample.accel);
        sample.time_ms = i;
        sample.valid = 1;
        ak_accel_cal_feed(cal, &sample);
    }
}

static void test_six_positions(void)
{
    ak_accel_cal_t cal;
    ak_accel_cal_init(&cal, 16u, 0.1f);

    expect("nothing is known before a face is measured",
           !ak_accel_cal_complete(&cal));

    feed_face(&cal, AK_ACCEL_LEVEL, 2, 1.0f, 16u);
    feed_face(&cal, AK_ACCEL_INVERTED, 2, -1.0f, 16u);
    feed_face(&cal, AK_ACCEL_NOSE_UP, 0, 1.0f, 16u);
    feed_face(&cal, AK_ACCEL_NOSE_DOWN, 0, -1.0f, 16u);
    expect("four faces is not six", !ak_accel_cal_complete(&cal));

    feed_face(&cal, AK_ACCEL_LEFT_DOWN, 1, 1.0f, 16u);
    feed_face(&cal, AK_ACCEL_RIGHT_DOWN, 1, -1.0f, 16u);
    expect("six faces is complete", ak_accel_cal_complete(&cal));
    expect("and the correction can be worked out", ak_accel_cal_finish(&cal));

    float bias[3];
    float scale[3];
    ak_accel_cal_get(&cal, bias, scale);

    expect("the measured bias is the part's bias",
           fabsf(bias[0] - PART_BIAS_X) < 0.001f &&
           fabsf(bias[1] - PART_BIAS_Y) < 0.001f &&
           fabsf(bias[2] - PART_BIAS_Z) < 0.001f);
    expect("and the measured scale is the part's scale",
           fabsf(scale[0] - PART_SCALE_X) < 0.001f &&
           fabsf(scale[1] - PART_SCALE_Y) < 0.001f &&
           fabsf(scale[2] - PART_SCALE_Z) < 0.001f);
}

static void test_the_correction_is_the_inverse(void)
{
    ak_accel_cal_t cal;
    ak_accel_cal_init(&cal, 8u, 0.1f);

    feed_face(&cal, AK_ACCEL_LEVEL, 2, 1.0f, 8u);
    feed_face(&cal, AK_ACCEL_INVERTED, 2, -1.0f, 8u);
    feed_face(&cal, AK_ACCEL_NOSE_UP, 0, 1.0f, 8u);
    feed_face(&cal, AK_ACCEL_NOSE_DOWN, 0, -1.0f, 8u);
    feed_face(&cal, AK_ACCEL_LEFT_DOWN, 1, 1.0f, 8u);
    feed_face(&cal, AK_ACCEL_RIGHT_DOWN, 1, -1.0f, 8u);
    (void)ak_accel_cal_finish(&cal);

    /* The whole point: what the part says, corrected, is what is true. Level
     * should read one g up and nothing sideways; on its nose, one g forward. */
    for (int axis = 0; axis < 3; axis++) {
        ak_imu_sample_t sample;
        part_reading(axis, 1.0f, sample.accel);
        ak_accel_cal_apply(&cal, &sample);
        expect("a corrected axis-up reading is one g",
               fabsf(sample.accel[axis] - 1.0f) < 0.002f);
        for (int other = 0; other < 3; other++) {
            if (other != axis) {
                expect("and the other axes read nothing",
                       fabsf(sample.accel[other]) < 0.002f);
            }
        }
    }
}

static void test_moving_samples_are_refused(void)
{
    ak_accel_cal_t cal;
    ak_accel_cal_init(&cal, 4u, 0.1f);
    ak_accel_cal_begin(&cal, AK_ACCEL_LEVEL);

    ak_imu_sample_t sample;
    part_reading(2, 1.0f, sample.accel);
    sample.gyro[0] = 0.5f; /* somebody is turning it */
    sample.gyro[1] = 0.0f;
    sample.gyro[2] = 0.0f;
    sample.valid = 1;
    sample.time_ms = 0;
    expect("a sample taken while moving is refused",
           ak_accel_cal_feed(&cal, &sample) == 0 && cal.samples[AK_ACCEL_LEVEL] == 0);
    expect("and counted", cal.rejected == 1);

    /* An accelerometer reading a third of a g is mid-swing, not a face. */
    sample.gyro[0] = 0.0f;
    sample.accel[0] = 0.0f;
    sample.accel[1] = 0.0f;
    sample.accel[2] = 0.3f;
    expect("nor is one that is not reading gravity",
           ak_accel_cal_feed(&cal, &sample) == 0 && cal.rejected == 2);

    /* And an invalid sample is not a measurement at all. */
    sample.valid = 0;
    sample.accel[2] = 1.0f;
    expect("nor is an invalid one",
           ak_accel_cal_feed(&cal, &sample) == 0 && cal.rejected == 2);
}

static void test_a_bad_face_is_refused(void)
{
    ak_accel_cal_t cal;
    ak_accel_cal_init(&cal, 4u, 0.1f);

    /* Every face measured, but the aircraft was not level on two of them: the
     * pair of readings is not two g apart, which is the check that stands
     * between this and a correction nobody should believe. */
    ak_accel_cal_begin(&cal, AK_ACCEL_LEVEL);
    ak_imu_sample_t sample;
    for (int i = 0; i < 4; i++) {
        part_reading(2, 1.0f, sample.accel);
        sample.gyro[0] = sample.gyro[1] = sample.gyro[2] = 0.0f;
        sample.valid = 1;
        sample.time_ms = 0;
        (void)ak_accel_cal_feed(&cal, &sample);
    }
    feed_face(&cal, AK_ACCEL_INVERTED, 2, -1.0f, 4u);
    feed_face(&cal, AK_ACCEL_NOSE_UP, 0, 1.0f, 4u);
    feed_face(&cal, AK_ACCEL_NOSE_DOWN, 0, -1.0f, 4u);
    feed_face(&cal, AK_ACCEL_LEFT_DOWN, 1, 1.0f, 4u);

    /* The last one is propped up at forty-five degrees instead of being put on
     * its side, so that axis only reads about 0.7 g. */
    ak_accel_cal_begin(&cal, AK_ACCEL_RIGHT_DOWN);
    for (int i = 0; i < 4; i++) {
        sample.accel[0] = 0.0f;
        sample.accel[1] = -0.7071f;
        sample.accel[2] = 0.7071f;
        sample.gyro[0] = sample.gyro[1] = sample.gyro[2] = 0.0f;
        sample.valid = 1;
        sample.time_ms = 0;
        (void)ak_accel_cal_feed(&cal, &sample);
    }

    expect("all six faces are measured", ak_accel_cal_complete(&cal));
    expect("but a face that was not level refuses the whole thing",
           ak_accel_cal_finish(&cal) == 0);

    float bias[3];
    float scale[3];
    ak_accel_cal_get(&cal, bias, scale);
    expect("and the correction that was in force is left alone",
           bias[0] == 0.0f && scale[0] == 1.0f);
}

static void test_a_face_can_be_redone(void)
{
    ak_accel_cal_t cal;
    ak_accel_cal_init(&cal, 4u, 0.1f);

    feed_face(&cal, AK_ACCEL_NOSE_DOWN, 0, -1.0f, 4u);
    expect("a face with enough samples has them",
           ak_accel_cal_have(&cal, AK_ACCEL_NOSE_DOWN));

    /* Starting the same face again throws the old samples away rather than
     * averaging the two attempts, which would mix a wrong face into a right
     * one. */
    ak_accel_cal_begin(&cal, AK_ACCEL_NOSE_DOWN);
    expect("starting it again starts from nothing",
           cal.samples[AK_ACCEL_NOSE_DOWN] == 0 &&
           !ak_accel_cal_have(&cal, AK_ACCEL_NOSE_DOWN));
}

void test_accel_calibration(void)
{
    test_six_positions();
    test_the_correction_is_the_inverse();
    test_moving_samples_are_refused();
    test_a_bad_face_is_refused();
    test_a_face_can_be_redone();
}
