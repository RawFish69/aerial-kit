#ifndef AK_FLIGHT_TYPES_H
#define AK_FLIGHT_TYPES_H

#include <stdint.h>

/*
 * The data that crosses between flight-core modules. Nothing hardware-shaped
 * appears here: a board hands in samples, a board takes out commands.
 *
 * Frames and units, fixed once so that a sign error is a bug and not a
 * convention:
 *
 *   body frame    x forward, y right, z down
 *   gyro          rad/s about x, y, z
 *   accel         g about x, y, z (z reads +1 with the board level and the
 *                 right way up, because z points down)
 *   sticks        roll/pitch/yaw -1..1, throttle 0..1
 *   motors        0..1 (0 = stopped)
 *   servos        -1..1 (0 = centre)
 *   angles        radians, rates rad/s
 */

#define AK_MAX_MOTORS   4
#define AK_MAX_SERVOS   2
#define AK_RC_CHANNELS  8

/* Channel order for the default layout: AETR, then aux. */
enum {
    AK_RC_ROLL = 0,
    AK_RC_PITCH = 1,
    AK_RC_THROTTLE = 2,
    AK_RC_YAW = 3,
    AK_RC_MODE = 4,
    AK_RC_ARM = 5,
};

typedef struct {
    float    gyro[3];  /* rad/s */
    float    accel[3]; /* g */
    uint32_t time_ms;
    int      valid;
} ak_imu_sample_t;

typedef struct {
    uint16_t channel[AK_RC_CHANNELS];
    uint32_t last_update_ms;
    int      valid;
} ak_rc_input_t;

typedef struct {
    float motor[AK_MAX_MOTORS]; /* 0..1 */
    float servo[AK_MAX_SERVOS]; /* -1..1 */
} ak_outputs_t;

#endif /* AK_FLIGHT_TYPES_H */
