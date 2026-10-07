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
    /*
     * When the sensor's reading was *taken*, in microseconds, from
     * ak_time_us().
     *
     * This was `time_ms` until phase 1.5 and the change is the milestone, not
     * tidying. The estimator's dt is the interval between two of these, so the
     * resolution of this field *is* the resolution of the integration: at the
     * one-kilohertz loop the fleet flies today a millisecond timestamp is
     * exactly right, and at the eight-kilohertz gyro phase 1.4 is aimed at it
     * is worse than useless - consecutive samples land in the same millisecond,
     * the interval reads zero or one, and the estimate is quantised to a 100%
     * error on every other sample. Nothing about that failure is visible in a
     * millisecond log, which is why it has to be fixed before the rate goes up
     * rather than after.
     *
     * It is the same clock as ak_time_ms() and not a second one: every port's
     * ak_arch_time_us() returns `ms * 1000 + fraction` from the one counter
     * (see ak_time.h), so a port cannot hand out two clocks that disagree about
     * what time it is. It does **not** follow that the millisecond reading can
     * be recovered from this one by division - it cannot, past the 71.6-minute
     * wrap - which is why ak_flight_step() takes both readings rather than one.
     * See ak_time.h and docs/29-timing.md.
     */
    uint32_t time_us;
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
