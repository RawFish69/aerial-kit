#ifndef AK_SENSORS_AK_ALIGN_H
#define AK_SENSORS_AK_ALIGN_H

#include "ak_types.h"

/*
 * Board alignment: turning what the sensor measured into what the airframe
 * means.
 *
 * No real flight controller has its inertial sensor bolted on square. The part
 * is rotated somewhere between "close enough" and "90 degrees in two axes", and
 * the firmware has to know, because a roll axis that is really the pitch axis
 * is a crash that looks like a tuning problem.
 *
 * The three angles are the rotation that takes a vector in the sensor's frame
 * and expresses it in the airframe's: apply roll about x, then pitch about y,
 * then yaw about z, in that order, which is the order written down here because
 * the order of three rotations is part of the answer.
 */

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float matrix[3][3];
} ak_align_t;

void ak_align_set(ak_align_t *align, float roll_deg, float pitch_deg,
                  float yaw_deg);

/* Rotates accel and gyro in place. Leave the angles at zero and this is the
 * identity, which is why it is safe to call unconditionally. */
void ak_align_apply(const ak_align_t *align, ak_imu_sample_t *sample);

#endif /* AK_SENSORS_AK_ALIGN_H */
