#ifndef AK_SENSORS_AK_ACCEL_CAL_H
#define AK_SENSORS_AK_ACCEL_CAL_H

#include "ak_types.h"

/*
 * Accelerometer calibration, six positions.
 *
 * A gyro bias is a constant that grows an attitude error over time; an
 * accelerometer error is an attitude error that is there immediately, and it is
 * the one the angle loop will fly the aircraft to. A part that reads 1.02 g
 * with its axis up and -0.97 g with it down is 20 mg off centre and half a
 * percent off scale, which is about 1.2 degrees of tilt nobody asked for - on
 * every axis at once, in a way that looks exactly like a badly levelled bench.
 *
 * Six positions measure both at once, because each axis is read pointing up and
 * pointing down:
 *
 *   bias  = (up + down) / 2
 *   scale = 2 / (up - down)
 *
 * The arithmetic is trivial. What is not is refusing to do it: samples taken
 * while the aircraft is moving, or on a face that is not actually level, give a
 * correction that is worse than none because it looks deliberate. So a face
 * only accumulates still samples, and the whole thing refuses to produce
 * numbers if a face is missing or a pair of readings is not roughly 2 g apart.
 *
 * The faces are the *airframe's*, not the sensor's, and the correction is
 * applied after alignment - so what the pilot is told is "nose down", not
 * "sensor x negative", and a board mounted at an angle does not need a
 * different set of instructions.
 */

typedef enum {
    AK_ACCEL_LEVEL = 0,      /* upright, +z up            */
    AK_ACCEL_INVERTED,       /* upside down               */
    AK_ACCEL_NOSE_DOWN,
    AK_ACCEL_NOSE_UP,
    AK_ACCEL_RIGHT_DOWN,
    AK_ACCEL_LEFT_DOWN,
    AK_ACCEL_FACES,
} ak_accel_face_t;

typedef struct {
    float    bias[3];              /* g, subtracted from every sample */
    float    scale[3];             /* multiplied in after the bias */
    float    sum[AK_ACCEL_FACES][3];
    uint32_t samples[AK_ACCEL_FACES];
    uint32_t wanted;               /* samples per face */
    uint32_t rejected;
    float    max_rate;             /* rad/s above which a sample is moving */
    int      face;                 /* the face being sampled, or -1 */
} ak_accel_cal_t;

void ak_accel_cal_init(ak_accel_cal_t *cal, uint32_t wanted, float max_rate);

/* The correction, for the parameter table and for restoring it at boot. */
void ak_accel_cal_set(ak_accel_cal_t *cal, const float bias[3],
                      const float scale[3]);
void ak_accel_cal_get(const ak_accel_cal_t *cal, float bias[3], float scale[3]);

/* Throws away the measurements and keeps the correction that is in force. */
void ak_accel_cal_reset(ak_accel_cal_t *cal);

/* Starts sampling one face. Returns 0 for a face number that does not exist. */
int ak_accel_cal_begin(ak_accel_cal_t *cal, int face);

/* One sample. Returns 1 when this sample completed the face being sampled. */
int ak_accel_cal_feed(ak_accel_cal_t *cal, const ak_imu_sample_t *sample);

/* Which faces have enough samples, for a report. */
int ak_accel_cal_have(const ak_accel_cal_t *cal, int face);
int ak_accel_cal_complete(const ak_accel_cal_t *cal);

/* Works out the correction from the six faces. Returns 0 and leaves the
 * existing correction alone if a face is missing or the readings are not a
 * plausible gravity: a calibration nobody should trust is worse than none. */
int ak_accel_cal_finish(ak_accel_cal_t *cal);

/* Removes the bias and applies the scale, in place. Does nothing before a
 * calibration. */
void ak_accel_cal_apply(const ak_accel_cal_t *cal, ak_imu_sample_t *sample);

const char *ak_accel_cal_face_name(int face);

#endif /* AK_SENSORS_AK_ACCEL_CAL_H */
