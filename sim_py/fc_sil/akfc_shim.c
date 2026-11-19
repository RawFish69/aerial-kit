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
 * servos -1..1 (trailing edge up positive), airframe = the firmware's id.
 */

#include <stdlib.h>
#include <string.h>

#include "ak_flight.h"
#include "ak_math.h"
#include "ak_mixer.h"
#include "ak_nav.h"
#include "ak_rc.h"

typedef struct {
    ak_flight_t flight;
    /* The navigator, for velocity hold: run the way main.c's nav_update()
     * runs it (see akfc_step_nav). */
    ak_nav_t nav;
    uint32_t nav_last_ms;
    int nav_have_last;
} akfc_t;

int akfc_abi_version(void) { return 3; }
unsigned akfc_rc_channels(void) { return AK_RC_CHANNELS; }
unsigned akfc_max_motors(void) { return AK_MAX_MOTORS; }
unsigned akfc_max_servos(void) { return AK_MAX_SERVOS; }

akfc_t *akfc_create(int airframe, unsigned motors, unsigned servos, unsigned loop_period_us)
{
    akfc_t *fc = (akfc_t *)calloc(1, sizeof(akfc_t));
    if (fc == NULL) {
        return NULL;
    }
    /* Any firmware airframe id (ak_mixer_for_airframe): 0 quad-X,
     * 1 twin-motor elevon wing, 7 single-motor elevon wing, ... */
    ak_flight_init(&fc->flight, &ak_mixer_quad_x);
    fc->flight.airframe = (uint32_t)(airframe < 0 ? 0 : airframe);
    ak_flight_apply_airframe(&fc->flight);
    if (fc->flight.mixer == 0) {
        free(fc);
        return NULL;
    }
    ak_flight_set_board_outputs(&fc->flight, motors, servos);
    if (loop_period_us > 0u) {
        ak_flight_set_loop_period_us(&fc->flight, loop_period_us);
    }
    ak_nav_init(&fc->nav);
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

/* 1-based receiver channel that selects velocity hold (0 = none). */
void akfc_set_velocity_channel(akfc_t *fc, unsigned channel)
{
    fc->flight.rc_cfg.velocity_channel = channel;
}

/*
 * One loop as main.c runs it with a navigator: link update, velocity hold
 * (main.c's nav_update(), the velocity-hold branch, reading the command the
 * core decoded last loop), the core's step, then GPS-course heading aiding.
 * The measurements are what the aircraft's sensors would give it: world
 * north/east velocity (fused), altitude above sea level, and a GPS ground
 * track (speed and course) for heading alignment; gps_valid = 0 withholds
 * all of them.
 */
void akfc_step_nav(akfc_t *fc, const float gyro[3], const float accel[3],
                   const uint16_t *channels, int rc_valid, uint32_t rc_ms,
                   uint32_t now_us, uint32_t now_ms,
                   int gps_valid, float north_m_s, float east_m_s, float alt_msl_m,
                   float *motors, float *servos)
{
    ak_imu_sample_t imu;
    ak_rc_input_t rc;
    ak_flight_t *f = &fc->flight;

    memset(&imu, 0, sizeof imu);
    for (int i = 0; i < 3; i++) {
        imu.gyro[i] = gyro[i];
        imu.accel[i] = accel[i];
    }
    imu.time_us = now_us;
    imu.valid = 1;
    memset(&rc, 0, sizeof rc);
    for (unsigned i = 0; i < AK_RC_CHANNELS; i++) {
        rc.channel[i] = channels[i];
    }
    rc.last_update_ms = rc_ms;
    rc.valid = rc_valid;

    (void)ak_flight_link_update(f, &rc, now_ms, 0);
    {
        int state = ak_flight_state(f);
        int flying = state == AK_FLIGHT_ARMED || state == AK_FLIGHT_MANAGED ||
                     state == AK_FLIGHT_RTH || state == AK_FLIGHT_DESCEND;
        ak_nav_input_t in;
        ak_rc_command_t nav_command;

        memset(&in, 0, sizeof in);
        in.alt_mm = (int32_t)(alt_msl_m * 1000.0f);
        in.gps_valid = gps_valid;
        in.velocity_valid = gps_valid;
        in.north_m_s = north_m_s;
        in.east_m_s = east_m_s;
        in.yaw_mrad = (int32_t)(f->est.yaw * 1000.0f);
        in.yaw_aligned = f->est.track_aligned;
        in.max_tilt_rad = ak_deg2rad(f->cfg.max_tilt_deg);
        in.dt_s = fc->nav_have_last ? (float)(now_ms - fc->nav_last_ms) / 1000.0f : 0.0f;
        fc->nav_last_ms = now_ms;
        fc->nav_have_last = 1;
        memset(&nav_command, 0, sizeof nav_command);

        if (f->cmd.velocity_mode && flying && ak_flight_link_live(f)) {
            ak_nav_velocity_setpoint_t v;
            float yaw = f->est.yaw;
            float right = f->cmd.roll * fc->nav.quad_speed;
            float forward = -f->cmd.pitch * fc->nav.quad_speed;

            v.north_mps = forward * ak_cosf(yaw) - right * ak_sinf(yaw);
            v.east_mps = forward * ak_sinf(yaw) + right * ak_cosf(yaw);
            v.climb_mps = (f->cmd.throttle - 0.5f) * 2.0f * AK_NAV_QUAD_MAX_CLIMB_MPS;
            v.yaw_rate_rps = f->cmd.yaw * AK_NAV_QUAD_MAX_YAW_RATE_RPS;
            ak_nav_velocity_set(&fc->nav, &v);
            if (ak_nav_velocity_step(&fc->nav, &in, &nav_command)) {
                ak_flight_set_guidance(f, &nav_command);
                ak_flight_set_managed(f, 1);
            } else {
                ak_flight_set_guidance(f, 0);
                ak_flight_set_managed(f, 0);
            }
        } else {
            ak_nav_velocity_stop(&fc->nav);
            ak_flight_set_guidance(f, 0);
            ak_flight_set_managed(f, 0);
        }
    }
    ak_flight_step(f, &imu, &rc, now_us, now_ms);
    if (gps_valid) {
        float speed = ak_sqrtf(north_m_s * north_m_s + east_m_s * east_m_s);
        ak_estimator_aid_heading(&f->est, ak_atan2f(east_m_s, north_m_s), speed,
                                 (float)ak_flight_last_loop_us(f) / 1000000.0f);
    }
    {
        const ak_outputs_t *o = ak_flight_outputs(f);
        for (unsigned i = 0; i < AK_MAX_MOTORS; i++) {
            motors[i] = o->motor[i];
        }
        for (unsigned i = 0; i < AK_MAX_SERVOS; i++) {
            servos[i] = o->servo[i];
        }
    }
}

int akfc_velocity_mode(const akfc_t *fc) { return fc->flight.cmd.velocity_mode; }
int akfc_track_aligned(const akfc_t *fc) { return fc->flight.est.track_aligned; }

/* The navigator's hover throttle (parameter rth_cruise): per airframe. */
void akfc_set_nav_cruise(akfc_t *fc, float throttle) { fc->nav.cruise = throttle; }

/* The zero-climb trim the vertical loop has learned, for host regressions. */
float akfc_nav_climb_integral(const akfc_t *fc) { return fc->nav.climb_i; }
