#include "ak_gyro_cal.h"

#include "ak_math.h"

/* How far the gravity vector may swing between samples before this counts as
 * "the aircraft was turning": two degrees, which is a tenth of a second of a
 * twenty-degree-a-second roll. */
#define AK_GYRO_CAL_ACCEL_COS 0.9994f

void ak_gyro_cal_init(ak_gyro_cal_t *cal, uint32_t wanted, float max_rate,
                      float max_bias)
{
    cal->bias[0] = 0.0f;
    cal->bias[1] = 0.0f;
    cal->bias[2] = 0.0f;
    cal->wanted = wanted;
    cal->max_rate = max_rate;
    cal->max_bias = max_bias;
    cal->samples = 0;
    cal->rejected = 0;
    cal->running = 0;
    cal->done = 0;
}

void ak_gyro_cal_start(ak_gyro_cal_t *cal)
{
    cal->sum[0] = 0.0f;
    cal->sum[1] = 0.0f;
    cal->sum[2] = 0.0f;
    cal->accel_sum[0] = 0.0f;
    cal->accel_sum[1] = 0.0f;
    cal->accel_sum[2] = 0.0f;
    cal->samples = 0;
    cal->rejected = 0;
    cal->running = 1;
    cal->done = 0;
}

void ak_gyro_cal_apply(const ak_gyro_cal_t *cal, ak_imu_sample_t *sample)
{
    for (int i = 0; i < 3; i++) {
        sample->gyro[i] -= cal->bias[i];
    }
}

int ak_gyro_cal_feed(ak_gyro_cal_t *cal, const ak_imu_sample_t *sample)
{
    if (!cal->running) {
        return 0;
    }
    /* A sample the driver could not read is not a sample. The caller's buffer
     * still holds the previous one - already aligned and bias-corrected - and
     * feeding it again subtracted the bias twice and counted towards `wanted`.
     * The accelerometer's calibration has always asked this. */
    if (!sample->valid) {
        return 0;
    }

    /* Compensated first, so the accumulator holds the residual and a second
     * calibration refines rather than doubles the correction. */
    float residual[3];
    float mean[3];
    float deviation = 0.0f;
    float magnitude = 0.0f;
    for (int i = 0; i < 3; i++) {
        residual[i] = sample->gyro[i] - cal->bias[i];
        magnitude += residual[i] * residual[i];
    }
    magnitude = ak_sqrtf(magnitude);

    /* Not a bias at all: a gyro reading this far from zero is either a part
     * that is broken or an aircraft that is being flown. */
    if (magnitude > cal->max_bias) {
        cal->rejected++;
        return 0;
    }

    /*
     * Is this a sample of a *still* aircraft?
     *
     * The first version of this asked whether the rate was small, and that is
     * the wrong question: the thing being measured - the part's own offset -
     * is a rate, so a gyro with five degrees a second in it failed a
     * three-degree test on every single sample and was never calibrated at
     * all. (Measured: 2198 samples, 2198 rejected, and the heading walked 109
     * degrees while the aircraft was parked.)
     *
     * What is true of a still aircraft is that its readings do not *change*:
     * they sit within a few tens of a degree a second of wherever the bias
     * puts them. So the test is against the running mean of the samples
     * already accepted, which is where the bias is converging, and a sample
     * that jumps away from it is a sample taken while something was moving.
     * That is also what the reference implementations do - Betaflight refuses
     * to finish a gyro calibration while the readings are still changing.
     */
    if (cal->samples > 0u) {
        float accel_dot = 0.0f;
        float accel_mean[3];
        float accel_now[3];
        float norm = 0.0f;

        for (int i = 0; i < 3; i++) {
            float delta;

            mean[i] = cal->sum[i] / (float)cal->samples;
            delta = residual[i] - mean[i];
            if (delta < 0.0f) {
                delta = -delta;
            }
            if (delta > deviation) {
                deviation = delta;
            }
        }
        if (deviation > cal->max_rate) {
            cal->rejected++;
            return 0;
        }

        /*
         * And the other half of the same question, from the other sensor: a
         * gyro reading that *is* steady can still be an aircraft turning at a
         * constant rate, and a turning aircraft's gravity vector moves. So the
         * accelerometer is asked too - the direction it points has to be where
         * it was pointing - which is what catches a roll or a pitch that the
         * gyro cannot tell from an offset.
         *
         * What neither of them catches is a steady rotation about the vertical,
         * where gravity does not move at all. There is no measurement on this
         * aircraft that separates that from a yaw offset; what makes it
         * acceptable is that nobody spins an aircraft on the ground, and
         * `calibrate gyro` exists for the case where somebody is not sure.
         */
        for (int i = 0; i < 3; i++) {
            accel_mean[i] = cal->accel_sum[i] / (float)cal->samples;
            accel_now[i] = sample->accel[i];
            accel_dot += accel_mean[i] * accel_now[i];
            norm += accel_mean[i] * accel_mean[i];
        }
        norm = ak_sqrtf(norm) *
               ak_sqrtf(accel_now[0] * accel_now[0] +
                        accel_now[1] * accel_now[1] +
                        accel_now[2] * accel_now[2]);
        if (norm > 0.001f && accel_dot / norm < AK_GYRO_CAL_ACCEL_COS) {
            cal->rejected++;
            return 0;
        }
    }

    for (int i = 0; i < 3; i++) {
        cal->sum[i] += residual[i];
        cal->accel_sum[i] += sample->accel[i];
    }
    cal->samples++;

    if (cal->samples >= cal->wanted) {
        for (int i = 0; i < 3; i++) {
            cal->bias[i] += cal->sum[i] / (float)cal->samples;
        }
        cal->running = 0;
        cal->done = 1;
        return 1;
    }
    return 0;
}

void ak_gyro_cal_bias_dps(const ak_gyro_cal_t *cal, float bias_dps[3])
{
    for (int i = 0; i < 3; i++) {
        bias_dps[i] = ak_rad2deg(cal->bias[i]);
    }
}

void ak_gyro_cal_set_bias_dps(ak_gyro_cal_t *cal, const float bias_dps[3])
{
    for (int i = 0; i < 3; i++) {
        cal->bias[i] = ak_deg2rad(bias_dps[i]);
    }
}
