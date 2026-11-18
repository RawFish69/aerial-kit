/*
 * Flat C API over the AK flight core, for software-in-the-loop on a host.
 *
 * Compiled together with the *unmodified* firmware sources under
 * firmware/src/core (flight, filter, sensors) by build.py. Nothing here
 * re-implements control: every output comes from ak_flight_step(), the
 * function the firmware's own loop calls.
 *
 * Units and frames are the firmware's: body FRD, gyro rad/s, accelerometer
 * in g with +1 on z when level and still, RC in receiver counts, motors 0..1,
 * servos -1..1, airframe 0 = quad-X, 1 = elevon wing.
 */

#include <stdlib.h>
#include <string.h>

#include "ak_flight.h"
#include "ak_mixer.h"
#include "ak_rc.h"

typedef struct {
    ak_flight_t flight;
} akfc_t;

int akfc_abi_version(void) { return 2; }
unsigned akfc_rc_channels(void) { return AK_RC_CHANNELS; }
unsigned akfc_max_motors(void) { return AK_MAX_MOTORS; }
unsigned akfc_max_servos(void) { return AK_MAX_SERVOS; }

akfc_t *akfc_create(int airframe, unsigned motors, unsigned servos, unsigned loop_period_us)
{
    akfc_t *fc = (akfc_t *)calloc(1, sizeof(akfc_t));
    if (fc == NULL) {
        return NULL;
    }
    if (airframe == 1) {
        ak_flight_init(&fc->flight, &ak_mixer_elevon_wing);
        fc->flight.airframe = 1u;
    } else {
        ak_flight_init(&fc->flight, &ak_mixer_quad_x);
        fc->flight.airframe = 0u;
    }
    ak_flight_apply_airframe(&fc->flight);
    ak_flight_set_board_outputs(&fc->flight, motors, servos);
    if (loop_period_us > 0u) {
        ak_flight_set_loop_period_us(&fc->flight, loop_period_us);
    }
    return fc;
}

void akfc_destroy(akfc_t *fc) { free(fc); }

/* RC layout: 0-based roll, pitch, throttle, yaw, mode, arm; then min, mid, max. */
void akfc_rc_layout(const akfc_t *fc, unsigned out[9])
{
    const ak_rc_config_t *c = &fc->flight.rc_cfg;
    out[0] = AK_RC_ROLL;
    out[1] = AK_RC_PITCH;
    out[2] = AK_RC_THROTTLE;
    out[3] = AK_RC_YAW;
    out[4] = c->mode_channel - 1u;
    out[5] = c->arm_channel - 1u;
    out[6] = c->min;
    out[7] = c->mid;
    out[8] = c->max;
}

/*
 * One firmware loop iteration. rc_ms is when the receiver frame arrived;
 * pass rc_valid = 0 to model a receiver that has gone quiet.
 */
void akfc_step(akfc_t *fc, const float gyro[3], const float accel[3],
               const uint16_t *channels, int rc_valid, uint32_t rc_ms,
               uint32_t now_us, uint32_t now_ms, float *motors, float *servos)
{
    ak_imu_sample_t imu;
    memset(&imu, 0, sizeof imu);
    for (int i = 0; i < 3; i++) {
        imu.gyro[i] = gyro[i];
        imu.accel[i] = accel[i];
    }
    imu.time_us = now_us;
    imu.valid = 1;

    static ak_rc_input_t rc; /* per-call scratch; the core keeps its own state */
    memset(&rc, 0, sizeof rc);
    for (unsigned i = 0; i < AK_RC_CHANNELS; i++) {
        rc.channel[i] = channels[i];
    }
    rc.last_update_ms = rc_ms;
    rc.valid = rc_valid;

    ak_flight_step(&fc->flight, &imu, &rc, now_us, now_ms);

    const ak_outputs_t *o = ak_flight_outputs(&fc->flight);
    for (unsigned i = 0; i < AK_MAX_MOTORS; i++) {
        motors[i] = o->motor[i];
    }
    for (unsigned i = 0; i < AK_MAX_SERVOS; i++) {
        servos[i] = o->servo[i];
    }
}

int akfc_state(const akfc_t *fc) { return (int)ak_flight_state(&fc->flight); }
const char *akfc_state_name(const akfc_t *fc) { return ak_flight_state_name(ak_flight_state(&fc->flight)); }

/* The firmware estimator's attitude, rad (its own convention, FRD). */
void akfc_attitude(const akfc_t *fc, float out[3])
{
    out[0] = fc->flight.est.roll;
    out[1] = fc->flight.est.pitch;
    out[2] = fc->flight.est.yaw;
}

/* What the core made of the sticks this loop: roll, pitch, yaw, throttle, angle_mode, arm_request. */
void akfc_command(const akfc_t *fc, float out[6])
{
    const ak_rc_command_t *c = &fc->flight.cmd;
    out[0] = c->roll;
    out[1] = c->pitch;
    out[2] = c->yaw;
    out[3] = c->throttle;
    out[4] = (float)c->angle_mode;
    out[5] = (float)c->arm_request;
}

/* Firmware limits a radio or teleop has to match:
 * max_tilt_deg, max_rate_dps, throttle_low, arm_hold_ms, arm_threshold, mode_threshold. */
void akfc_config(const akfc_t *fc, float out[6])
{
    out[0] = fc->flight.cfg.max_tilt_deg;
    out[1] = fc->flight.cfg.max_rate_dps;
    out[2] = fc->flight.cfg.throttle_low;
    out[3] = (float)fc->flight.cfg.arm_hold_ms;
    out[4] = (float)fc->flight.rc_cfg.arm_threshold;
    out[5] = (float)fc->flight.rc_cfg.mode_threshold;
}
