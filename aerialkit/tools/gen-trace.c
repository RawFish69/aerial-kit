/*
 * gen-trace - write a synthetic input trace for replay.
 *
 *   build-host/aerialkit-gen-trace > docs/evidence/trace-hover.csv
 *
 * 250 Hz, three seconds, one aircraft: disarmed and level, then the arm switch
 * held on with the throttle down, then throttle up with a roll input, then
 * hands off. It exercises the parts of the loop that are supposed to be dull -
 * the arming hold, the motor response, the mixer's roll rows - and it is the
 * input side of the evidence file, so anyone can regenerate it and compare.
 *
 * Angle mode with a roll command, not rate mode: the point of this trace is to
 * show the loop responding to an attitude error, which needs no gyro input to
 * be believable.
 */

#include <math.h>
#include <stdio.h>

#define TRACE_HZ 250
#define TRACE_SECONDS 3

int main(void)
{
    const float dt = 1.0f / (float)TRACE_HZ;
    const int steps = TRACE_HZ * TRACE_SECONDS;

    printf("t_ms,gyro_x,gyro_y,gyro_z,ax,ay,az,roll,pitch,yaw,throttle,arm,mode\n");

    for (int i = 0; i < steps; i++) {
        float t = (float)i * dt;

        float roll_command = 0.0f;
        float throttle = 0.0f;
        int arm = 0;

        if (t >= 1.0f && t < 1.6f) {
            arm = 1;             /* switch on, throttle down: the hold runs */
        } else if (t >= 1.6f && t < 2.6f) {
            arm = 1;
            throttle = 0.5f;
            roll_command = 0.4f; /* about 14 degrees of commanded roll */
        } else if (t >= 2.6f) {
            arm = 1;
            throttle = 0.5f;
            roll_command = 0.0f;
        }

        /* The aircraft does not move in this trace: the accelerometer reports
         * gravity the whole time and the gyro reports nothing. That is enough
         * to show the loop's response, and it is honest about what it is - a
         * scripted input, not a flight. */
        printf("%u,0,0,0,0,0,1,%.4f,0,0,%.4f,%d,1\n",
               (unsigned)(i * 1000 / TRACE_HZ), (double)roll_command,
               (double)throttle, arm);
    }

    return 0;
}
