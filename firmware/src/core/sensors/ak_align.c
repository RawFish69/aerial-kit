#include "ak_align.h"

#include "ak_math.h"

void ak_align_set(ak_align_t *align, float roll_deg, float pitch_deg,
                  float yaw_deg)
{
    align->roll_deg = roll_deg;
    align->pitch_deg = pitch_deg;
    align->yaw_deg = yaw_deg;

    float cr = ak_cosf(ak_deg2rad(roll_deg));
    float sr = ak_sinf(ak_deg2rad(roll_deg));
    float cp = ak_cosf(ak_deg2rad(pitch_deg));
    float sp = ak_sinf(ak_deg2rad(pitch_deg));
    float cy = ak_cosf(ak_deg2rad(yaw_deg));
    float sy = ak_sinf(ak_deg2rad(yaw_deg));

    /* Roll, then pitch, then yaw: R = Rz * Ry * Rx, written out because the
     * order is not interchangeable and a matrix library here would only hide
     * which one it is. */
    align->matrix[0][0] = cy * cp;
    align->matrix[0][1] = cy * sp * sr - sy * cr;
    align->matrix[0][2] = cy * sp * cr + sy * sr;

    align->matrix[1][0] = sy * cp;
    align->matrix[1][1] = sy * sp * sr + cy * cr;
    align->matrix[1][2] = sy * sp * cr - cy * sr;

    align->matrix[2][0] = -sp;
    align->matrix[2][1] = cp * sr;
    align->matrix[2][2] = cp * cr;
}

static void rotate(const ak_align_t *align, float vector[3])
{
    float x = vector[0];
    float y = vector[1];
    float z = vector[2];

    vector[0] = align->matrix[0][0] * x + align->matrix[0][1] * y +
                align->matrix[0][2] * z;
    vector[1] = align->matrix[1][0] * x + align->matrix[1][1] * y +
                align->matrix[1][2] * z;
    vector[2] = align->matrix[2][0] * x + align->matrix[2][1] * y +
                align->matrix[2][2] * z;
}

void ak_align_apply(const ak_align_t *align, ak_imu_sample_t *sample)
{
    rotate(align, sample->accel);
    rotate(align, sample->gyro);
}
