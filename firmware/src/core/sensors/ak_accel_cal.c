#include "ak_accel_cal.h"

#include "ak_math.h"

/* The two faces that read each axis, the way up and the way down. */
static const struct {
    int axis;
    int up;
    int down;
} axes[3] = {
    { 0, AK_ACCEL_NOSE_UP, AK_ACCEL_NOSE_DOWN },
    { 1, AK_ACCEL_LEFT_DOWN, AK_ACCEL_RIGHT_DOWN },
    { 2, AK_ACCEL_LEVEL, AK_ACCEL_INVERTED },
};

/* A face that is not level, or a part with a wildly wrong scale, shows up as a
 * pair of readings that are not two g apart. Real accelerometers are inside a
 * few percent of that; this is the line between "calibrate it" and "believe
 * nothing you are told". */
#define AK_ACCEL_MIN_SPAN_G 1.95f
#define AK_ACCEL_MAX_SPAN_G 2.05f

void ak_accel_cal_init(ak_accel_cal_t *cal, uint32_t wanted, float max_rate)
{
    for (int i = 0; i < 3; i++) {
        cal->bias[i] = 0.0f;
        cal->scale[i] = 1.0f;
    }
    cal->wanted = wanted;
    cal->max_rate = max_rate;
    ak_accel_cal_reset(cal);
}

void ak_accel_cal_set(ak_accel_cal_t *cal, const float bias[3],
                      const float scale[3])
{
    for (int i = 0; i < 3; i++) {
        cal->bias[i] = bias[i];
        /* A scale of zero would erase an axis, and one loaded from a
         * corrupted parameter is not a calibration. */
        cal->scale[i] = (scale[i] > 0.5f && scale[i] < 2.0f) ? scale[i] : 1.0f;
    }
}

void ak_accel_cal_get(const ak_accel_cal_t *cal, float bias[3], float scale[3])
{
    for (int i = 0; i < 3; i++) {
        bias[i] = cal->bias[i];
        scale[i] = cal->scale[i];
    }
}

void ak_accel_cal_reset(ak_accel_cal_t *cal)
{
    for (int f = 0; f < AK_ACCEL_FACES; f++) {
        cal->samples[f] = 0;
        for (int i = 0; i < 3; i++) {
            cal->sum[f][i] = 0.0f;
        }
    }
    cal->rejected = 0;
    cal->face = -1;
}

int ak_accel_cal_begin(ak_accel_cal_t *cal, int face)
{
    if (face < 0 || face >= AK_ACCEL_FACES) {
        return 0;
    }
    cal->face = face;
    cal->samples[face] = 0;
    for (int i = 0; i < 3; i++) {
        cal->sum[face][i] = 0.0f;
    }
    return 1;
}

int ak_accel_cal_feed(ak_accel_cal_t *cal, const ak_imu_sample_t *sample)
{
    if (cal->face < 0 || !sample->valid) {
        return 0;
    }

    /* Still means still: a rate sensor reading motion, or a gravity vector
     * that is not about a g, is somebody handling the aircraft, and a face
     * measured while it was picked up is a correction that will tilt it. */
    float rate = ak_absf(sample->gyro[0]) + ak_absf(sample->gyro[1]) +
                 ak_absf(sample->gyro[2]);
    float magnitude = ak_sqrtf(sample->accel[0] * sample->accel[0] +
                               sample->accel[1] * sample->accel[1] +
                               sample->accel[2] * sample->accel[2]);
    if (rate > cal->max_rate || magnitude < 0.7f || magnitude > 1.3f) {
        cal->rejected++;
        return 0;
    }

    for (int i = 0; i < 3; i++) {
        cal->sum[cal->face][i] += sample->accel[i];
    }
    cal->samples[cal->face]++;

    if (cal->samples[cal->face] < cal->wanted) {
        return 0;
    }

    cal->face = -1;
    return 1;
}

int ak_accel_cal_have(const ak_accel_cal_t *cal, int face)
{
    if (face < 0 || face >= AK_ACCEL_FACES) {
        return 0;
    }
    return cal->samples[face] >= cal->wanted &&
           cal->wanted > 0u;
}

int ak_accel_cal_complete(const ak_accel_cal_t *cal)
{
    for (int f = 0; f < AK_ACCEL_FACES; f++) {
        if (!ak_accel_cal_have(cal, f)) {
            return 0;
        }
    }
    return 1;
}

int ak_accel_cal_finish(ak_accel_cal_t *cal)
{
    if (!ak_accel_cal_complete(cal)) {
        return 0;
    }

    float bias[3];
    float scale[3];

    for (int a = 0; a < 3; a++) {
        int axis = axes[a].axis;
        float up = cal->sum[axes[a].up][axis] / (float)cal->samples[axes[a].up];
        float down = cal->sum[axes[a].down][axis] /
                     (float)cal->samples[axes[a].down];
        float span = up - down;

        if (span < AK_ACCEL_MIN_SPAN_G || span > AK_ACCEL_MAX_SPAN_G) {
            return 0;
        }

        bias[axis] = (up + down) * 0.5f;
        scale[axis] = 2.0f / span;
    }

    ak_accel_cal_set(cal, bias, scale);
    return 1;
}

void ak_accel_cal_apply(const ak_accel_cal_t *cal, ak_imu_sample_t *sample)
{
    for (int i = 0; i < 3; i++) {
        sample->accel[i] = (sample->accel[i] - cal->bias[i]) * cal->scale[i];
    }
}

const char *ak_accel_cal_face_name(int face)
{
    switch (face) {
    case AK_ACCEL_LEVEL:
        return "level";
    case AK_ACCEL_INVERTED:
        return "inverted";
    case AK_ACCEL_NOSE_DOWN:
        return "nose down";
    case AK_ACCEL_NOSE_UP:
        return "nose up";
    case AK_ACCEL_RIGHT_DOWN:
        return "right side down";
    case AK_ACCEL_LEFT_DOWN:
        return "left side down";
    default:
        return "?";
    }
}
