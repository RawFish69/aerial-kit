/*
 * Host tests for the flight core.
 *
 * These run on the development machine, where there is a libm, a debugger and
 * no risk of a prop turning. They check the things a bench cannot easily
 * provoke - a sensor going quiet, a failsafe latch, our own atan2 against a
 * reference - and then call the same selftest the board prints at boot.
 *
 *   make test
 */

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ak_estimator.h"
#include "ak_crsf.h"
#include "ak_flight.h"
#include "ak_math.h"
#include "ak_mixer.h"
#include "ak_output.h"
#include "ak_pid.h"
#include "ak_selftest.h"
#include "tests.h"

/* Shared with the other test files, which is why they are not static. */
int failures;
int checks;

void expect(const char *name, int passed)
{
    checks++;
    if (passed) {
        printf("  ok       %s\n", name);
    } else {
        printf("  FAILED   %s\n", name);
        failures++;
    }
}

int expect_failures(void)
{
    return failures;
}

static int printf_shim(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vprintf(fmt, ap);
    va_end(ap);
    return n;
}

static void test_math_against_libm(void)
{
    float worst_sqrt = 0.0f;
    for (int i = 1; i < 2000; i++) {
        float x = (float)i * 0.37f;
        float want = sqrtf(x);
        float got = ak_sqrtf(x);
        float rel = fabsf(got - want) / want;
        if (rel > worst_sqrt) {
            worst_sqrt = rel;
        }
    }
    printf("  sqrt     worst relative error vs libm: %.3e\n", (double)worst_sqrt);
    expect("ak_sqrtf within 1e-5 relative of libm", worst_sqrt < 1e-5f);

    float worst_atan = 0.0f;
    for (int iy = -40; iy <= 40; iy++) {
        for (int ix = -40; ix <= 40; ix++) {
            float y = (float)iy * 0.25f;
            float x = (float)ix * 0.25f;
            if (x == 0.0f && y == 0.0f) {
                continue;
            }
            float err = fabsf(ak_atan2f(y, x) - atan2f(y, x));
            if (err > worst_atan) {
                worst_atan = err;
            }
        }
    }
    printf("  atan2    worst absolute error vs libm: %.3e rad\n", (double)worst_atan);
    expect("ak_atan2f within 1e-4 rad of libm", worst_atan < 1e-4f);

    /* Sine and cosine, which board alignment needs. Checked over enough turns
     * that a mistake in the range reduction shows up. */
    float worst_trig = 0.0f;
    for (int i = -3000; i <= 3000; i++) {
        float x = (float)i * 0.01f;
        float err_sin = fabsf(ak_sinf(x) - sinf(x));
        float err_cos = fabsf(ak_cosf(x) - cosf(x));
        if (err_sin > worst_trig) {
            worst_trig = err_sin;
        }
        if (err_cos > worst_trig) {
            worst_trig = err_cos;
        }
    }
    printf("  sin/cos  worst absolute error vs libm: %.3e over -30..30 rad\n",
           (double)worst_trig);
    expect("ak_sinf and ak_cosf within 1e-5 of libm", worst_trig < 1e-5f);

    /*
     * The wrap the log and the attitude telemetry are written through.
     *
     * Two things are being checked, and the second is the one that matters:
     * that it is the true remainder - the same heading one whole turn earlier
     * or later - and that it *always* comes back inside the circle, because
     * what it feeds is a float-to-int16 conversion and an angle outside the
     * circle is a conversion that is undefined rather than merely wrong. That
     * is how this was found: the sanitizer stopped the three-hundred-second
     * session inside the log writer, at a yaw of 3276.85 degrees.
     */
    float worst_wrap = 0.0f;
    for (int i = -100000; i <= 100000; i++) {
        float x = (float)i * 0.01f; /* -1000..1000 rad: 159 turns either way */
        float got = ak_wrap_pi(x);
        float err = fabsf(got - remainderf(x, 6.28318531f));
        if (err > AK_PI) {
            err = fabsf(err - 6.28318531f); /* the same heading, from the far side */
        }
        if (err > worst_wrap) {
            worst_wrap = err;
        }
        if (!(got >= -AK_PI && got <= AK_PI)) {
            worst_wrap = 1.0e9f; /* out of the circle: not "wrong", undefined */
        }
    }
    printf("  wrap     worst error vs libm remainder: %.3e rad over -1000..1000\n",
           (double)worst_wrap);
    expect("ak_wrap_pi is the true remainder", worst_wrap < 1.0e-4f);

    expect("and a whole turn later is the same heading",
           fabsf(ak_wrap_pi(ak_deg2rad(370.0f)) - ak_deg2rad(10.0f)) < 0.001f &&
               fabsf(ak_wrap_pi(ak_deg2rad(-350.0f)) - ak_deg2rad(10.0f)) < 0.001f);

    /* The attitude fields are what a caller converts with, and the first of
     * these is the sanitizer's number exactly: 3276.85 degrees, nine turns
     * into a flight, which reached the log as a 32768 that its int16 could not
     * hold. As a heading it is 36.85 degrees - and it is the same heading one
     * whole turn earlier and one whole turn later, which is the property a
     * continuous angle cannot have and a field this narrow needs. */
    expect("a yaw nine turns into a flight is the heading it really is",
           ak_attitude_ddeg(ak_deg2rad(3276.85f)) == 368 &&
               ak_attitude_ddeg(ak_deg2rad(36.85f)) == 368 &&
               ak_attitude_ddeg(ak_deg2rad(396.85f)) == 368);
    expect("and the field is the tenth of a degree its header claims",
           ak_attitude_ddeg(0.0f) == 0 &&
               ak_attitude_ddeg(AK_PI) == 1800 &&
               ak_attitude_ddeg(-AK_PI) == -1800);

    /* Not angles, so not answers: NaN and the infinities are what a sensor
     * that has gone wrong produces, and both of those are undefined to
     * convert. Zero is a heading the aircraft is not claiming to have. */
    expect("an angle that is not a number wraps to nothing, not to UB",
           ak_wrap_pi(NAN) == 0.0f && ak_wrap_pi(INFINITY) == 0.0f &&
               ak_wrap_pi(-INFINITY) == 0.0f && ak_wrap_pi(1.0e30f) == 0.0f);
}

static void test_pid(void)
{
    ak_pid_t pid;

    /* Proportional only: output is kp * error, immediately. */
    ak_pid_init(&pid, 2.0f, 0.0f, 0.0f, 1.0f, 10.0f, 0.0f);
    float out = ak_pid_update(&pid, 1.0f, 0.0f, 0.001f);
    expect("pid proportional gain is exact", fabsf(out - 2.0f) < 1e-6f);

    /* The integral is clamped, so holding an unreachable setpoint cannot build
     * up a term that takes seconds to unwind. */
    ak_pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.25f, 5.0f, 0.0f);
    for (int i = 0; i < 5000; i++) {
        out = ak_pid_update(&pid, 1.0f, 0.0f, 0.001f);
    }
    expect("pid integral clamps", fabsf(pid.integral) <= 0.25f + 1e-6f);
    expect("pid output clamps", fabsf(out) <= 5.0f + 1e-6f);

    /* Derivative on measurement: a setpoint step with the measurement held
     * still must not produce a derivative spike. */
    ak_pid_init(&pid, 1.0f, 0.0f, 5.0f, 0.0f, 100.0f, 0.0f);
    ak_pid_update(&pid, 1.0f, 0.0f, 0.001f);
    out = ak_pid_update(&pid, 1.0f, 0.0f, 0.001f);
    expect("pid derivative ignores setpoint steps", fabsf(out - 1.0f) < 1e-5f);

    /* A moving measurement does produce one, with the right sign. */
    ak_pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.0f, 100.0f, 0.0f);
    ak_pid_update(&pid, 0.0f, 0.0f, 0.001f);
    out = ak_pid_update(&pid, 0.0f, 0.1f, 0.001f);
    expect("pid derivative opposes the measurement's motion", out < 0.0f);
}

static void test_estimator(void)
{
    ak_estimator_t est;
    ak_imu_sample_t imu = { .valid = 1 };

    /* Held at 20 degrees of roll with a matching accelerometer: the estimate
     * has to end up there, not at zero and not at twice it. */
    ak_estimator_init(&est, 0.5f);
    float want = ak_deg2rad(20.0f);
    for (int i = 1; i <= 3000; i++) {
        imu.accel[0] = 0.0f;
        imu.accel[1] = sinf(want);
        imu.accel[2] = cosf(want);
        imu.gyro[0] = 0.0f;
        imu.gyro[1] = 0.0f;
        imu.gyro[2] = 0.0f;
        imu.time_ms = (uint32_t)i;
        ak_estimator_update(&est, &imu, 0.001f);
    }
    expect("estimator tracks a held 20 degree roll", fabsf(est.roll - want) < 0.01f);
    expect("estimator reports converged", est.converged != 0);

    /* Accelerometer readings that are not gravity are ignored, so a hard
     * manoeuvre does not yank the attitude estimate around. */
    ak_estimator_init(&est, 0.5f);
    float before = est.roll;
    for (int i = 1; i <= 500; i++) {
        imu.accel[0] = 0.0f;
        imu.accel[1] = 3.0f; /* 3 g sideways: not gravity */
        imu.accel[2] = 0.4f;
        imu.time_ms = (uint32_t)i;
        ak_estimator_update(&est, &imu, 0.001f);
    }
    expect("estimator ignores accelerometer readings far from 1 g",
           fabsf(est.roll - before) < 0.01f);

    /*
     * Yaw from the ground track. This is what makes a magnetometer-less
     * aircraft able to hold a position: the estimate starts at whatever
     * direction the aircraft happened to be pointing when it was switched on,
     * and the GPS course is the only measurement of the world frame there is.
     *
     * Four things are worth pinning: it converges, it does not move when the
     * aircraft is not moving, it takes the short way round the circle, and it
     * does not claim to be aligned before it has actually been corrected.
     */
    ak_estimator_init(&est, 0.5f);
    est.yaw = ak_deg2rad(120.0f); /* a yaw that means nothing yet */
    for (int i = 0; i < 200; i++) {
        ak_estimator_aid_heading(&est, ak_deg2rad(10.0f), 0.5f, 0.01f);
    }
    expect("a stationary aircraft does not get its yaw moved by a course",
           fabsf(est.yaw - ak_deg2rad(120.0f)) < 0.001f &&
               est.track_aligned == 0);

    for (int i = 0; i < 3000; i++) {
        ak_estimator_aid_heading(&est, ak_deg2rad(10.0f), 8.0f, 0.01f);
    }
    expect("a moving one is pulled onto the track it is making",
           fabsf(est.yaw - ak_deg2rad(10.0f)) < 0.02f && /* 1 degree, 30 s */
               est.track_aligned == 1);

    /* Across north: a course of 5 degrees and a yaw of 355 are ten degrees
     * apart, not 350, and the estimate has to move the short way. */
    ak_estimator_init(&est, 0.5f);
    est.yaw = ak_deg2rad(355.0f);
    for (int i = 0; i < 3000; i++) {
        ak_estimator_aid_heading(&est, ak_deg2rad(5.0f), 8.0f, 0.01f);
    }
    expect("and it goes round the short way across north",
           fabsf(est.yaw - ak_deg2rad(365.0f)) < 0.05f);

    /*
     * And what the caller feeds it: the module reports motion over the
     * *ground*, and the track the aircraft made through the air is that vector
     * minus the wind. The three cases that matter are the ones the raw ground
     * track gets wrong.
     */
    {
        float speed = 0.0f;
        float course = 0.0f;

        /* Drifting with a five-metre-a-second wind, from the west, with the
         * nose anywhere: the ground vector is the wind and nothing else, so
         * there is no air track at all - which is the answer that matters,
         * because a yaw aligned to *this* is a yaw aligned to the weather. */
        ak_estimator_air_track(5000, 9000000 /* due east */, 0.0f, -5.0f,
                               &speed, &course);
        expect("drifting in wind leaves no track through the air",
               speed < 0.05f);

        /* Flying north at six metres a second through the air with five of
         * tailwind from the south: the ground vector is the two added. */
        ak_estimator_air_track(7810 /* the length of (6, 5) */, 3980000,
                               0.0f, -5.0f, &speed, &course);
        expect("flying through a crosswind reports the way the nose is going",
               fabsf(speed - 6.0f) < 0.05f && fabsf(course) < 0.02f);

        /* And with no wind at all it is the module's own vector, untouched. */
        ak_estimator_air_track(4000, 9000000, 0.0f, 0.0f, &speed, &course);
        expect("in still air the track is simply the ground track",
               fabsf(speed - 4.0f) < 0.01f &&
                   fabsf(course - 1.5707963f) < 0.01f);
    }
}

static ak_imu_sample_t imu_at(float roll, float pitch, uint32_t time_ms)
{
    ak_imu_sample_t imu;
    imu.gyro[0] = 0.0f;
    imu.gyro[1] = 0.0f;
    imu.gyro[2] = 0.0f;
    imu.accel[0] = -sinf(pitch);
    imu.accel[1] = sinf(roll);
    imu.accel[2] = cosf(roll) * cosf(pitch);
    imu.time_ms = time_ms;
    imu.valid = 1;
    return imu;
}

static ak_rc_input_t rc_at(float roll, float throttle, int arm, int angle_mode,
                           uint32_t time_ms)
{
    ak_rc_config_t cfg;
    ak_rc_default_config(&cfg);

    ak_rc_input_t rc;
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        rc.channel[i] = (uint16_t)cfg.mid;
    }
    rc.channel[AK_RC_ROLL] = roll >= 0.0f
        ? (uint16_t)((float)cfg.mid + roll * (float)(cfg.max - cfg.mid))
        : (uint16_t)((float)cfg.mid + roll * (float)(cfg.mid - cfg.min));
    rc.channel[AK_RC_THROTTLE] =
        (uint16_t)((float)cfg.min + throttle * (float)(cfg.max - cfg.min));
    rc.channel[AK_RC_ARM] = (uint16_t)(arm ? cfg.max : cfg.min);
    rc.channel[AK_RC_MODE] = (uint16_t)(angle_mode ? cfg.max : cfg.min);
    rc.last_update_ms = time_ms;
    rc.valid = 1;
    return rc;
}

/*
 * The gyro filter, and the cutoff in its name.
 *
 * `gyro_lpf_hz` is a first-order low pass on the gyro, applied once per step
 * before the estimator and the three rate loops see a rate - and 0 by default,
 * because a filter in the *proportional* path costs phase and this aircraft's
 * gains are not tuned (`ak_flight.c` has the closed-loop check that said so,
 * and the reference's own number, 250, is the first thing to try once a log
 * shows the noise it would remove).
 *
 * What is checked here is that the parameter means the frequency it names,
 * which is the property the *previous* coefficient did not have: the first
 * version of `ak_lpf_alpha` was `w*dt/(1+w*dt)`, and at a kilohertz loop that
 * put a "250 Hz" filter's -3 dB at 163 Hz. The check drives the loop with a
 * sine and reads `flight->gyro`, so it measures the filter through the code
 * that uses it rather than the arithmetic beside it.
 */
static float gyro_response(float hz, float lpf_hz)
{
    ak_flight_t flight;
    flight_with_outputs(&flight, &ak_mixer_quad_x);
    flight.cfg.gyro_lpf_hz = lpf_hz;

    const float dt = 1.0f / 1000.0f;   /* the fleet's loop rate */
    float peak = 0.0f;
    uint32_t t = 0;

    for (int i = 0; i < 2000; i++) {
        t++;
        ak_imu_sample_t imu = imu_at(0.0f, 0.0f, t);
        /* One rad/s of sine, so the peak that comes back *is* the gain. */
        imu.gyro[0] = sinf(6.283185307f * hz * (float)i * dt);
        ak_rc_input_t rc = rc_at(0.0f, 0.0f, 0, 1, t);
        ak_flight_step(&flight, &imu, &rc, t);
        if (i > 500 && fabsf(flight.gyro[0]) > peak) {
            peak = fabsf(flight.gyro[0]);
        }
    }
    return peak;
}

static void test_the_gyro_filter(void)
{
    expect("a slow oscillation goes through the gyro filter unharmed",
           gyro_response(20.0f, 250.0f) > 0.95f);
    expect("its -3 dB point is the frequency the parameter names",
           fabsf(gyro_response(250.0f, 250.0f) - 0.7071f) < 0.05f);
    expect("and a rate the gyro's noise lives at is attenuated",
           gyro_response(400.0f, 250.0f) < 0.65f);

    /* And with the parameter at zero it is the identity - sample for sample,
     * not "about right": a disabled filter that still smoothed would be a
     * filter nobody could switch off. */
    ak_flight_t flight;
    flight_with_outputs(&flight, &ak_mixer_quad_x);
    flight.cfg.gyro_lpf_hz = 0.0f;
    int identical = 1;
    for (int i = 1; i <= 20; i++) {
        ak_imu_sample_t imu = imu_at(0.0f, 0.0f, (uint32_t)i);
        imu.gyro[0] = 0.1f * (float)i;
        ak_rc_input_t rc = rc_at(0.0f, 0.0f, 0, 1, (uint32_t)i);
        ak_flight_step(&flight, &imu, &rc, (uint32_t)i);
        if (flight.gyro[0] != imu.gyro[0]) {
            identical = 0;
        }
    }
    expect("while the filter is the identity when the parameter is zero",
           identical);
}

static void test_flight_angle_mode(void)
{
    ak_flight_t flight;
    flight_with_outputs(&flight, &ak_mixer_quad_x);
    uint32_t t = 0;

    /* Level, disarmed: nothing moves. */
    for (int i = 0; i < 200; i++) {
        t++;
        ak_imu_sample_t imu = imu_at(0.0f, 0.0f, t);
        ak_rc_input_t rc = rc_at(0.0f, 0.0f, 0, 1, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    const ak_outputs_t *o = ak_flight_outputs(&flight);
    expect("disarmed: no motor turns", o->motor[0] == 0.0f && o->motor[1] == 0.0f &&
                                        o->motor[2] == 0.0f && o->motor[3] == 0.0f);

    /* Arm. */
    for (int i = 0; i < 800; i++) {
        t++;
        ak_imu_sample_t imu = imu_at(0.0f, 0.0f, t);
        ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("armed after the hold", ak_flight_state(&flight) == AK_FLIGHT_ARMED);

    /* Angle mode, held at 15 degrees of roll, sticks centred: the controller
     * has to command a negative roll torque to bring it back to level, which
     * in this mixer means the right-hand motors go up. */
    for (int i = 0; i < 200; i++) {
        t++;
        ak_imu_sample_t imu = imu_at(ak_deg2rad(15.0f), 0.0f, t);
        ak_rc_input_t rc = rc_at(0.0f, 0.4f, 1, 1, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    o = ak_flight_outputs(&flight);
    printf("  roll torque %.3f, motors %.3f %.3f %.3f %.3f\n",
           (double)flight.torque[0], (double)o->motor[0], (double)o->motor[1],
           (double)o->motor[2], (double)o->motor[3]);
    expect("angle mode pushes back against an uncommanded roll",
           flight.torque[0] < 0.0f);
    expect("and the mixer leans the other way",
           o->motor[0] > o->motor[2] && o->motor[1] > o->motor[3]);

    /* Sensor loss is a failsafe, even with the receiver still talking. */
    ak_imu_sample_t bad = imu_at(0.0f, 0.0f, ++t);
    bad.valid = 0;
    ak_rc_input_t rc = rc_at(0.0f, 0.4f, 1, 1, t);
    ak_flight_step(&flight, &bad, &rc, t);
    expect("loss of the IMU stops the motors",
           ak_flight_state(&flight) == AK_FLIGHT_FAILSAFE &&
           flight.out.motor[0] == 0.0f);
}

/* The same channels with the mode switch thrown the other way: rate mode, which
 * is what a pilot flies a quadrotor with by hand. `rc_at` holds the roll stick
 * and the mode channel, and this sets the other two sticks on top of it. */
static ak_rc_input_t rc_mode(float roll, float pitch, float yaw, float throttle,
                             int arm, int angle_mode, uint32_t time_ms)
{
    ak_rc_config_t cfg;
    ak_rc_default_config(&cfg);

    ak_rc_input_t rc = rc_at(roll, throttle, arm, angle_mode, time_ms);
    /* A stick is set the way the receiver reports it: centred is the middle
     * count, and the two ends are the two ends. */
    rc.channel[AK_RC_PITCH] =
        pitch >= 0.0f
            ? (uint16_t)((float)cfg.mid + pitch * (float)(cfg.max - cfg.mid))
            : (uint16_t)((float)cfg.mid + pitch * (float)(cfg.mid - cfg.min));
    rc.channel[AK_RC_YAW] =
        yaw >= 0.0f
            ? (uint16_t)((float)cfg.mid + yaw * (float)(cfg.max - cfg.mid))
            : (uint16_t)((float)cfg.mid + yaw * (float)(cfg.mid - cfg.min));
    return rc;
}

/*
 * Rate mode, which no test had ever flown.
 *
 * The mode channel selects one of two flight laws, and until this was written
 * every test passed `angle_mode` as 1 and every simulator session held the
 * mode channel high: the branch a pilot actually flies a quadrotor with - the
 * stick asks for a rotation *rate*, and nothing levels the aircraft - had no
 * evidence beyond the code compiling.
 *
 * What it should do is what the stick says: a full roll stick asks for the
 * configured rate and the mixer rolls the aircraft that way; centring the
 * stick asks for *zero* rate and leaves the attitude alone, which is the whole
 * difference from angle mode; and the pitch and yaw sticks do the same on their
 * own axes, in the direction they are pushed.
 */
static void test_flight_rate_mode(void)
{
    ak_flight_t flight;
    ak_imu_sample_t imu;
    ak_rc_input_t rc;
    flight_with_outputs(&flight, &ak_mixer_quad_x);
    uint32_t t = 0;

    /* Armed with the mode switch low, to show the law does not change what
     * arming means. */
    for (int i = 0; i < 1000; i++) {
        t++;
        imu = imu_at(0.0f, 0.0f, t);
        rc = rc_mode(0, 0, 0, 0.0f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("it arms in rate mode too",
           ak_flight_state(&flight) == AK_FLIGHT_ARMED);

    /* A full right roll stick: the rate the parameter asks for, and a torque
     * that rolls right. */
    for (int i = 0; i < 50; i++) {
        t++;
        imu = imu_at(0.0f, 0.0f, t);
        rc = rc_mode(1.0f, 0.0f, 0.0f, 0.5f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    float max_rate = ak_deg2rad(flight.cfg.max_rate_dps);
    expect("a full roll stick asks for the full rate",
           flight.rate_setpoint[0] > max_rate * 0.99f &&
               flight.rate_setpoint[0] < max_rate * 1.01f);
    expect("and the mixer rolls it that way", flight.torque[0] > 0.0f);

    /* Centred, with the aircraft *tilted*: the commanded rate is zero. That is
     * the difference between the two laws in the sentence a pilot would use -
     * angle mode with a centred stick asks for the rate that would level the
     * aircraft (the test above measures it pushing back), and rate mode asks
     * for nothing at all. The attitude does not enter this law anywhere else
     * either: it is a rate error and nothing more. */
    flight.est.roll = ak_deg2rad(20.0f);
    for (int i = 0; i < 50; i++) {
        t++;
        imu = imu_at(ak_deg2rad(20.0f), 0.0f, t);
        rc = rc_mode(0, 0, 0, 0.5f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("centring the stick asks for no rotation at all",
           ak_absf(flight.rate_setpoint[0]) < 0.001f);

    /* And a rotation nobody asked for is damped: the sticks are centred and
     * the gyro says the aircraft is rolling, so the loop's only business is to
     * stop it. (What it does *not* do is care where the wings are - the check
     * above, with a tilted aircraft and a zero command, is that half.) */
    for (int i = 0; i < 100; i++) {
        t++;
        imu = imu_at(ak_deg2rad(20.0f), 0.0f, t);
        imu.gyro[0] = ak_deg2rad(100.0f);
        rc = rc_mode(0, 0, 0, 0.5f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("and a rotation nobody asked for is damped",
           flight.torque[0] < 0.0f);

    /* Forward stick: nose down, which is a negative pitch rate. */
    for (int i = 0; i < 50; i++) {
        t++;
        imu = imu_at(0.0f, 0.0f, t);
        rc = rc_mode(0, -1.0f, 0, 0.5f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("a forward pitch stick asks for a nose-down rate",
           flight.rate_setpoint[1] < -max_rate * 0.99f);
    expect("and the mixer pitches it down", flight.torque[1] < 0.0f);

    /* Right stick on the rudder: a right yaw rate. */
    for (int i = 0; i < 50; i++) {
        t++;
        imu = imu_at(0.0f, 0.0f, t);
        rc = rc_mode(0, 0, 1.0f, 0.5f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    float yaw_rate = ak_deg2rad(flight.cfg.yaw_rate_dps);
    expect("a right yaw stick asks for a right yaw rate",
           flight.rate_setpoint[2] > yaw_rate * 0.5f);
    expect("and the mixer yaws it right", flight.torque[2] > 0.0f);
}

/*
 * The single-motor wing, flown - the airframe whose yaw column is empty because
 * there is nothing on the aircraft to fill it.
 *
 * `tests/test_mix.c` checks the table. This checks the *loop*, and the two are
 * not the same check, because the mixer already guarantees the output: the yaw
 * column is zero, so the torque is multiplied by zero whatever the control law
 * asks for. A flight core that ran the yaw PID anyway would produce **byte for
 * byte the same motor and servo pulses as one that does not** - every output
 * this file could look at would agree.
 *
 * What differs is the PID's state. A loop handed a demand it can never satisfy
 * does not return zero: it returns a discarded torque and keeps the error it
 * accumulated producing it, so a pilot holding rudder winds up an integral that
 * is invisible while the axis stays unauthorised. That is why these assertions
 * read the setpoint, the torque and the integral rather than the outputs, and
 * why the twin-motor wing is flown beside it: without that comparison, "no yaw
 * torque" cannot be told apart from a stick that was never moved.
 */
static void test_single_motor_wing_flies_without_yaw(void)
{
    ak_flight_t flight;
    ak_imu_sample_t imu;
    ak_rc_input_t rc;
    uint32_t t = 0;

    /* Selected the way the firmware selects it: by the parameter, not by
     * handing `ak_flight_init` a table it would then override. */
    ak_flight_init(&flight, &ak_mixer_quad_x);
    flight.airframe = 7u;
    ak_flight_apply_airframe(&flight);
    ak_flight_set_board_outputs(&flight, 1u, 2u);

    for (int i = 0; i < 1000; i++) {
        t++;
        imu = imu_at(0.0f, 0.0f, t);
        rc = rc_mode(0, 0, 0, 0.0f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("the single-motor wing arms",
           ak_flight_state(&flight) == AK_FLIGHT_ARMED);
    expect("and it flies the one-motor table, not the quadrotor's",
           flight.mixer == &ak_mixer_elevon_wing_single);

    /* Full right rudder, held for two seconds. On the twin this asks for a yaw
     * rate and gets one; here there is no actuator to ask. */
    for (int i = 0; i < 2000; i++) {
        t++;
        imu = imu_at(0.0f, 0.0f, t);
        rc = rc_mode(0, 0, 1.0f, 0.5f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("a rudderless wing is asked for no yaw rate at all",
           flight.rate_setpoint[2] == 0.0f);
    expect("and asks for no yaw torque, however long the stick is held",
           flight.torque[2] == 0.0f);
    expect("and the yaw loop has not wound up against nothing",
           flight.rate[2].integral == 0.0f);

    /* The two axes it does have still answer, which is what stops this test
     * passing for a control law that had stopped responding altogether. */
    for (int i = 0; i < 200; i++) {
        t++;
        imu = imu_at(0.0f, 0.0f, t);
        rc = rc_mode(1.0f, 0.0f, 1.0f, 0.5f, 1, 0, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("a right roll stick still rolls it", flight.torque[0] > 0.0f);
    expect("and the rudder held beside it is still inert",
           flight.torque[2] == 0.0f && flight.rate[2].integral == 0.0f);

    /* The same held rudder on the airframe this one is a copy of. The stick
     * reaches the loop and the loop has an answer, so the silence above is this
     * aircraft's and not the test's. */
    {
        ak_flight_t twin;
        uint32_t tt = 0;

        ak_flight_init(&twin, &ak_mixer_quad_x);
        twin.airframe = 1u;
        ak_flight_apply_airframe(&twin);
        ak_flight_set_board_outputs(&twin, 2u, 2u);
        for (int i = 0; i < 1000; i++) {
            tt++;
            imu = imu_at(0.0f, 0.0f, tt);
            rc = rc_mode(0, 0, 0, 0.0f, 1, 0, tt);
            ak_flight_step(&twin, &imu, &rc, tt);
        }
        for (int i = 0; i < 200; i++) {
            tt++;
            imu = imu_at(0.0f, 0.0f, tt);
            rc = rc_mode(0, 0, 1.0f, 0.5f, 1, 0, tt);
            ak_flight_step(&twin, &imu, &rc, tt);
        }
        expect("the same rudder on the twin-motor wing is asked for, and yaws",
               twin.rate_setpoint[2] > 0.0f && twin.torque[2] > 0.0f);
    }
}

/*
 * The arming gates: the ones that were there, and the one that was not.
 *
 * An aircraft that is not the right way up must not arm. Betaflight refuses
 * with ARMING_DISABLED_ANGLE (fc/core.c, against its own small_angle) and INAV
 * calls the same condition ARMING_DISABLED_NOT_LEVEL (fc/fc_core.c); both
 * default to 25 degrees. AerialKit had the switch, the throttle and the
 * estimate and no tilt check at all, which on a bench means a board whose
 * alignment is wrong - or one somebody is holding on its side - arms and then
 * flies the correction.
 *
 * Every case here asks the same function the flight core and the console ask,
 * because a second copy of a gate list is a second answer.
 */
static void test_flight_arming_gates(void)
{
    ak_flight_t flight;
    ak_rc_command_t cmd;
    float detail = 0.0f;

    /* No receiver at all: the first gate, and the one the bare bench has. */
    flight_with_outputs(&flight, &ak_mixer_quad_x);
    {
        ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, 0);
        ak_rc_decode(&rc, &flight.rc_cfg, &cmd);
    }
    expect("no link: the arm check says so",
           ak_flight_arm_check(&flight, &cmd, &detail) == AK_ARM_NO_LINK);

    /*
     * The gate that is not about the aircraft at all: the board's outputs
     * against the airframe's mix.
     *
     * This is the wing's own board - the one whose two servos took the two
     * motors TMR2 was driving - with a quadrotor's mix selected. Every other
     * gate would pass: the link is up, the switch is on, the throttle is down
     * and the estimate is level. It must not arm, and it must say both numbers,
     * because the person reading it is looking at a firmware build and not at a
     * stick.
     */
    {
        uint32_t t = 0;
        ak_flight_t f;

        ak_flight_init(&f, &ak_mixer_quad_x);
        /* Two motors and two servos: the wing's own board, which cannot drive a
         * quadrotor's four. */
        ak_flight_set_board_outputs(&f, 2u, 2u);
        for (int i = 0; i < 800; i++) {
            t++;
            ak_imu_sample_t imu = imu_at(0.0f, 0.0f, t);
            ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, t);
            ak_flight_step(&f, &imu, &rc, t);
        }
        cmd = f.cmd;
        expect("a quadrotor's mix on a board with two motors does not arm",
               ak_flight_state(&f) == AK_FLIGHT_DISARMED);
        expect("and the reason is the outputs, not the aircraft",
               ak_flight_arm_check(&f, &cmd, &detail) == AK_ARM_OUTPUTS);
        expect("with both numbers in the flight core for the message",
               f.needed_motors == 4u && f.needed_servos == 0u &&
                   f.board_motors == 2u && f.board_servos == 2u);
    }

    /* And the same board with the mix it was built for: the gate is a
     * comparison, not a prohibition. */
    {
        uint32_t t = 0;
        ak_flight_t f;

        /* The airframe is the parameter that decides which mixer flies, so the
         * wing is selected the way the firmware selects it - not by handing
         * `ak_flight_init` a table it would then override. */
        ak_flight_init(&f, &ak_mixer_quad_x);
        f.airframe = 1u;
        ak_flight_apply_airframe(&f);
        ak_flight_set_board_outputs(&f, 2u, 2u);
        for (int i = 0; i < 800; i++) {
            t++;
            ak_imu_sample_t imu = imu_at(0.0f, 0.0f, t);
            ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, t);
            ak_flight_step(&f, &imu, &rc, t);
        }
        expect("and the wing's mix on that board arms",
               ak_flight_state(&f) == AK_FLIGHT_ARMED);
    }

    /*
     * And a board that has not said: the counts are zero and the answer is a
     * refusal rather than an assumption. A gate a board can skip by staying
     * quiet is not a gate.
     */
    {
        ak_flight_t f;

        ak_flight_init(&f, &ak_mixer_quad_x);
        expect("a board that has not said what it drives does not arm",
               ak_flight_arm_check(&f, &cmd, &detail) == AK_ARM_OUTPUTS);
    }

    /* Level, switch on, throttle down: ready, and armed after the hold. */
    {
        uint32_t t = 0;
        ak_flight_t f;
        flight_with_outputs(&f, &ak_mixer_quad_x);
        for (int i = 0; i < 800; i++) {
            t++;
            ak_imu_sample_t imu = imu_at(0.0f, 0.0f, t);
            ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, t);
            ak_flight_step(&f, &imu, &rc, t);
        }
        expect("level: it arms", ak_flight_state(&f) == AK_FLIGHT_ARMED);
    }

    /* The gate that was missing: 40 degrees of roll, which passes every other
     * gate - the estimate is converged, the throttle is down, the switch is on.
     * It must not arm, and it must be able to say how far out it is. */
    {
        uint32_t t = 0;
        ak_flight_t f;
        flight_with_outputs(&f, &ak_mixer_quad_x);
        for (int i = 0; i < 3000; i++) {
            t++;
            ak_imu_sample_t imu = imu_at(ak_deg2rad(40.0f), 0.0f, t);
            ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, t);
            ak_flight_step(&f, &imu, &rc, t);
        }
        cmd = f.cmd;
        expect("40 degrees of roll: it does not arm",
               ak_flight_state(&f) == AK_FLIGHT_DISARMED);
        expect("and the reason is the tilt",
               ak_flight_arm_check(&f, &cmd, &detail) == AK_ARM_NOT_LEVEL);
        expect("and the number behind it is the tilt it read",
               detail > 39.0f && detail < 41.0f);
        printf("  a tilted aircraft: %d degrees from level, against a limit "
               "of %d\n",
               (int)(detail + 0.5f), (int)(f.cfg.arm_max_tilt_deg + 0.5f));
    }

    /*
     * And the hold that gate starts is a wall-clock delay, not a wait for the
     * filter. `converged` is fifty accelerometer updates - a count, with no
     * term for how wrong the estimate still is - so a 20 degree aircraft, well
     * inside the tilt limit, is cleared to arm while the angle between the
     * estimate's body-up and the accelerometer is still more than ten degrees.
     * The number that matters is the innovation at the millisecond the aircraft
     * actually arms, because that is the estimate the aircraft is flying on. It
     * has to be read from the state machine and not from ak_flight_arm_check():
     * that call is the predicate *beside* the gate, so a check built on it
     * measures the shipped gate no matter what the arming loop does.
     */
    {
        uint32_t t = 0;
        ak_flight_t f;
        flight_with_outputs(&f, &ak_mixer_quad_x);
        int ready_ms = -1;
        float innov_at_ready = -1.0f;
        for (int i = 0; i < 3000; i++) {
            t++;
            ak_imu_sample_t imu = imu_at(ak_deg2rad(20.0f), 0.0f, t);
            ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, t);
            ak_flight_step(&f, &imu, &rc, t);
            cmd = f.cmd;
            if (ready_ms < 0 && ak_flight_state(&f) == AK_FLIGHT_ARMED) {
                ready_ms = (int)t;
                innov_at_ready = ak_estimator_innovation_deg(&f.est, imu.accel);
            }
        }
        printf("  a 20 degree aircraft: it arms at %d ms with the estimate "
               "%.1f degrees out\n",
               ready_ms, (double)innov_at_ready);
        expect("20 degrees of roll, inside the tilt limit: it arms with the "
               "estimate under 2 degrees out", innov_at_ready <= 2.0f);
        expect("and the same aircraft still arms",
               ak_flight_state(&f) == AK_FLIGHT_ARMED);
    }

    /* The same aircraft with the check turned off, the way Betaflight's
     * small_angle of 180 turns it off: now it arms, which is what makes the
     * case above a gate rather than a permanent refusal. */
    {
        uint32_t t = 0;
        ak_flight_t f;
        flight_with_outputs(&f, &ak_mixer_quad_x);
        f.cfg.arm_max_tilt_deg = 180.0f;
        for (int i = 0; i < 3000; i++) {
            t++;
            ak_imu_sample_t imu = imu_at(ak_deg2rad(40.0f), 0.0f, t);
            ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, t);
            ak_flight_step(&f, &imu, &rc, t);
        }
        expect("the same tilt with the check off: it arms",
               ak_flight_state(&f) == AK_FLIGHT_ARMED);
    }

    /* And a gate that was already there still speaks in the same voice. */
    {
        uint32_t t = 0;
        ak_flight_t f;
        flight_with_outputs(&f, &ak_mixer_quad_x);
        for (int i = 0; i < 300; i++) {
            t++;
            ak_imu_sample_t imu = imu_at(0.0f, 0.0f, t);
            ak_rc_input_t rc = rc_at(0.0f, 0.5f, 1, 1, t);
            ak_flight_step(&f, &imu, &rc, t);
        }
        cmd = f.cmd;
        expect("throttle up: the check names the throttle",
               ak_flight_arm_check(&f, &cmd, &detail) == AK_ARM_THROTTLE);
        expect("and the number behind it is the stick",
               detail > 0.45f && detail < 0.55f);
    }
}

/* Pack 16 channels the way the receiver does, so the decoder is checked
 * against frames built from the wire description rather than from itself. */
static uint8_t pack_crsf(uint8_t *frame, const uint16_t channels[AK_CRSF_CHANNELS],
                         uint8_t address, uint8_t type)
{
    uint8_t payload[22];
    for (int i = 0; i < 22; i++) {
        payload[i] = 0;
    }
    /* Sixteen eleven-bit channels are twenty-two bytes exactly, and the last
     * one ends on the last bit of the last byte - so a byte-wise unpack (which
     * is what this was) writes a third byte past the end of the array for the
     * final channel. It wrote zero there every time, which is why nothing ever
     * noticed; running the suite under -fsanitize=undefined is what did. Bit by
     * bit cannot leave the payload: bit + 10 is 175 for the last channel. */
    for (int n = 0; n < AK_CRSF_CHANNELS; n++) {
        unsigned bit = (unsigned)n * 11u;
        uint32_t value = (uint32_t)channels[n] & 0x7FFu;
        for (unsigned b = 0; b < 11u; b++) {
            if ((value >> b) & 1u) {
                payload[(bit + b) / 8u] |= (uint8_t)(1u << ((bit + b) % 8u));
            }
        }
    }

    frame[0] = address;
    frame[1] = 2 + 22; /* type + payload + crc */
    frame[2] = type;
    for (int i = 0; i < 22; i++) {
        frame[3 + i] = payload[i];
    }
    frame[25] = ak_crsf_crc8(&frame[2], 23);
    return 26;
}

static void test_crsf(void)
{
    uint16_t channels[AK_CRSF_CHANNELS];
    uint8_t frame[AK_CRSF_MAX_FRAME];

    /* The hand-checkable packing cases first: all ones packs to all 0xFF, and
     * channel n starts at bit 11n. */
    uint8_t payload[22];
    for (int i = 0; i < 16; i++) {
        channels[i] = 0x7FF;
    }
    pack_crsf(frame, channels, 0xC8, AK_CRSF_TYPE_RC_CHANNELS);
    for (int i = 0; i < 22; i++) {
        payload[i] = frame[3 + i];
    }
    int all_ones = 1;
    for (int i = 0; i < 22; i++) {
        all_ones = all_ones && payload[i] == 0xFF;
    }
    expect("all 16 channels at full scale pack to 0xFF", all_ones);

    for (int i = 0; i < 16; i++) {
        channels[i] = 0;
    }
    channels[0] = 1;
    pack_crsf(frame, channels, 0xC8, AK_CRSF_TYPE_RC_CHANNELS);
    expect("channel 0 is the low 11 bits", frame[3] == 0x01 && frame[4] == 0x00);

    channels[0] = 0;
    channels[1] = 1;
    pack_crsf(frame, channels, 0xC8, AK_CRSF_TYPE_RC_CHANNELS);
    expect("channel 1 starts at bit 11", frame[4] == 0x08 && frame[5] == 0x00);

    /* A real frame, fed one byte at a time the way a UART delivers it. */
    ak_crsf_t crsf;
    ak_crsf_init(&crsf);
    ak_rc_input_t rc;
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        rc.channel[i] = 0;
    }
    rc.valid = 0;

    for (int i = 0; i < 16; i++) {
        channels[i] = (uint16_t)(992 + i);
    }
    uint8_t length = pack_crsf(frame, channels, 0xC8, AK_CRSF_TYPE_RC_CHANNELS);
    int decoded = 0;
    for (uint8_t i = 0; i < length; i++) {
        decoded = ak_crsf_feed(&crsf, frame[i], &rc, 1000) || decoded;
    }
    expect("a clean frame decodes", decoded == 1 && rc.valid == 1);
    expect("channels come back in order",
           rc.channel[0] == 992 && rc.channel[1] == 993 && rc.channel[7] == 999);

    /* A corrupted byte must not decode, and must not stop the next frame. */
    uint32_t before = crsf.crc_errors;
    frame[10] ^= 0x40;
    for (uint8_t i = 0; i < length; i++) {
        ak_crsf_feed(&crsf, frame[i], &rc, 1100);
    }
    expect("a corrupted frame is rejected", crsf.crc_errors == before + 1);

    pack_crsf(frame, channels, 0xC8, AK_CRSF_TYPE_RC_CHANNELS);
    decoded = 0;
    for (uint8_t i = 0; i < length; i++) {
        decoded = ak_crsf_feed(&crsf, frame[i], &rc, 1200) || decoded;
    }
    expect("the parser recovers after a bad frame", decoded == 1);

    /* A truncated frame followed by noise then a good frame: the parser has to
     * resynchronise rather than stay stuck mid-frame. */
    for (uint8_t i = 0; i < 6; i++) {
        ak_crsf_feed(&crsf, frame[i], &rc, 1300);
    }
    for (int i = 0; i < 12; i++) {
        ak_crsf_feed(&crsf, (uint8_t)(0x91 + i), &rc, 1300);
    }
    decoded = 0;
    for (uint8_t i = 0; i < length; i++) {
        decoded = ak_crsf_feed(&crsf, frame[i], &rc, 1400) || decoded;
    }
    expect("a truncated frame does not wedge the parser", decoded == 1);

    /* A valid frame of a type we do not use yet: counted, not decoded. */
    pack_crsf(frame, channels, 0xC8, 0x14 /* link statistics */);
    decoded = 0;
    for (uint8_t i = 0; i < length; i++) {
        decoded = ak_crsf_feed(&crsf, frame[i], &rc, 1500) || decoded;
    }
    expect("a non-RC frame is accepted but not decoded", decoded == 0);

    /* Our own crc against a published check value: crc8/dvb-s2 of "123456789"
     * is 0xBC. */
    const uint8_t check[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    expect("crc8/dvb-s2 check value", ak_crsf_crc8(check, sizeof check) == 0xBC);
}

static void test_output_encoding(void)
{
    /* Hand-derived vectors, not round-tripped through the implementation. The
     * crc covers (throttle << 1) | telemetry:
     *   throttle 0,    no telemetry: value 0x000 -> crc 0x0 -> 0x0000
     *   throttle 2047, telemetry:    value 0xFFF -> crc 0xF -> 0xFFFF
     *   throttle 1000, no telemetry: value 0x7D0 -> crc 0xA -> 0x7D0A
     *   throttle 1000, telemetry:    value 0x7D1 -> crc 0xB -> 0x7D1B */
    expect("dshot disarmed frame is 0x0000", ak_dshot_pack(0, 0) == 0x0000);
    expect("dshot full throttle with telemetry is 0xFFFF",
           ak_dshot_pack(2047, 1) == 0xFFFF);
    expect("dshot throttle 1000 packs to 0x7D0A",
           ak_dshot_pack(1000, 0) == 0x7D0A);
    expect("dshot throttle 1000 with telemetry packs to 0x7D1B",
           ak_dshot_pack(1000, 1) == 0x7D1B);

    /* The disarmed band is the interesting part of the mapping: 0 means off,
     * and anything above it has to clear the reserved 1..47. */
    expect("motor 0 is the disarmed command, not minimum thrust",
           ak_dshot_from_motor(0.0f) == 0);
    expect("motor 1.0 is full throttle",
           ak_dshot_from_motor(1.0f) == AK_DSHOT_MAX_THROTTLE);
    expect("the smallest nonzero motor value clears the reserved band",
           ak_dshot_from_motor(0.001f) >= AK_DSHOT_MIN_THROTTLE);

    expect("servo centre is 1500 us", ak_servo_pulse_us(0.0f, 1000, 1500, 2000) == 1500);
    expect("servo limits are 1000 and 2000 us",
           ak_servo_pulse_us(-1.0f, 1000, 1500, 2000) == 1000 &&
           ak_servo_pulse_us(1.0f, 1000, 1500, 2000) == 2000);
    expect("a servo command beyond full travel clamps",
           ak_servo_pulse_us(3.0f, 1000, 1500, 2000) == 2000);

    /* One whole frame, the way a board would hand it to its timers. */
    ak_outputs_t out;
    ak_output_frame_t frame;
    ak_mixer_apply(&ak_mixer_elevon_wing, 0.5f, 0.2f, 0.1f, 0.0f, &out);
    ak_output_encode(&out, 0, 0, &frame);
    expect("the wing's elevons come out as opposite pulses",
           frame.servo_us[0] > 1500 && frame.servo_us[1] < 1500);
    expect("the wing's motors come out as DShot frames",
           frame.dshot[0] != 0 && frame.dshot[1] != 0);

    /*
     * And the plumbing a pilot adds on top, which is the half that decides
     * whether a wing turns: a reversed linkage, a centre that is not 1500, and
     * how far the linkage actually moves.
     */
    {
        ak_servo_trim_t trims[AK_MAX_SERVOS];

        ak_servo_trim_defaults(trims, AK_MAX_SERVOS);
        expect("the default linkage is the neutral one",
               trims[0].reversed == 0u && trims[0].trim_us == 0.0f &&
                   trims[0].travel_us == AK_SERVO_DEFAULT_TRAVEL_US);
        expect("and it encodes the way it did before there were trims",
               ak_servo_pulse_trimmed(0.0f, &trims[0]) == 1500u &&
                   ak_servo_pulse_trimmed(1.0f, &trims[0]) == 2000u &&
                   ak_servo_pulse_trimmed(-1.0f, &trims[0]) == 1000u);

        /* Reversed: the same command, the other way round - which is what a
         * servo arm on the other side of the surface does. */
        trims[1].reversed = 1u;
        expect("a reversed servo answers the other way",
               ak_servo_pulse_trimmed(1.0f, &trims[1]) == 1000u &&
                   ak_servo_pulse_trimmed(-1.0f, &trims[1]) == 2000u);
        expect("and it is still centred when the command is centred",
               ak_servo_pulse_trimmed(0.0f, &trims[1]) == 1500u);

        /* A trim moves the centre and nothing else. */
        trims[1].reversed = 0u;
        trims[1].trim_us = -30.0f;
        expect("a trim moves the centre, not the travel",
               ak_servo_pulse_trimmed(0.0f, &trims[1]) == 1470u &&
                   ak_servo_pulse_trimmed(1.0f, &trims[1]) == 1970u);

        /* And the travel is what makes two elevons deflect by the same amount:
         * a shorter linkage moves further, so it is given fewer microseconds. */
        trims[1].trim_us = 0.0f;
        trims[1].travel_us = 400u;
        expect("a shorter travel deflects less for the same stick",
               ak_servo_pulse_trimmed(1.0f, &trims[1]) == 1900u);

        /* And a parameter or an arithmetic mistake cannot drive a servo into
         * its stop: the mapping clamps at what the hardware survives. The
         * trim moves the whole range with it - 1500 + 200 +/- 900 is 800 to
         * 2600, and the top end is not a pulse any servo wants. */
        trims[1].travel_us = 900u;
        trims[1].trim_us = (float)AK_SERVO_MAX_TRIM_US;
        expect("a travel and a trim together cannot exceed what a servo survives",
               ak_servo_pulse_trimmed(1.0f, &trims[1]) ==
                   AK_SERVO_HARD_MAX_US &&
                   ak_servo_pulse_trimmed(-1.0f, &trims[1]) == 800u);

        /* And in a frame, which is where the flight loop reads it. */
        ak_servo_trim_defaults(trims, AK_MAX_SERVOS);
        trims[1].reversed = 1u;
        ak_output_encode(&out, trims, 0, &frame);
        expect("the frame carries the trim: a reversed elevon is the other way",
               frame.servo_us[0] > 1500 && frame.servo_us[1] > 1500);
    }

    ak_mixer_apply(&ak_mixer_quad_x, 0.0f, 0.0f, 0.0f, 0.0f, &out);
    ak_output_encode(&out, 0, 0, &frame);
    expect("zero throttle encodes as four disarmed frames",
           frame.dshot[0] == 0 && frame.dshot[1] == 0 &&
           frame.dshot[2] == 0 && frame.dshot[3] == 0);
}

/* A closed loop against a toy plant: the only test here that can tell a
 * stable loop from an unstable one. The plant is one number per axis - torque
 * becomes rate through a lag - which is crude, but a controller that cannot
 * hold 15 degrees of roll against it has no business on an aircraft. */
static void test_flight_closed_loop(void)
{
    ak_flight_t flight;
    flight_with_outputs(&flight, &ak_mixer_quad_x);

    const float dt = 1.0f / 250.0f;
    const float target = ak_deg2rad(15.0f);
    const float rate_per_torque = 10.0f; /* rad/s per unit of torque */
    const float actuator_tau = 0.03f;    /* s */
    const float alpha = dt / (actuator_tau + dt);

    float roll = 0.0f;
    float roll_rate = 0.0f;
    float torque_lag = 0.0f;
    uint32_t t = 0;

    /* Arm first: switch on, throttle down, estimator converged. The clock and
     * the plant share dt here, so the arm hold is measured the same way the
     * board would measure it. */
    for (int i = 0; i < 300; i++) {
        t += (uint32_t)(dt * 1000.0f);
        ak_imu_sample_t imu = imu_at(roll, 0.0f, t);
        imu.gyro[0] = roll_rate;
        ak_rc_input_t rc = rc_at(0.0f, 0.0f, 1, 1, t);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("closed loop: armed", ak_flight_state(&flight) == AK_FLIGHT_ARMED);

    /* Then command a roll and let the plant move. */
    float command = target / ak_deg2rad(flight.cfg.max_tilt_deg);
    for (int i = 0; i < 750; i++) {
        t += (uint32_t)(dt * 1000.0f);
        ak_imu_sample_t imu = imu_at(roll, 0.0f, t);
        imu.gyro[0] = roll_rate;
        ak_rc_input_t rc = rc_at(command, 0.5f, 1, 1, t);
        ak_flight_step(&flight, &imu, &rc, t);

        torque_lag += alpha * (flight.torque[0] - torque_lag);
        roll_rate = torque_lag * rate_per_torque;
        roll += roll_rate * dt;
    }

    printf("  closed loop: commanded %.1f deg, reached %.1f deg, rate %.2f rad/s\n",
           (double)ak_rad2deg(target), (double)ak_rad2deg(roll),
           (double)roll_rate);
    expect("closed loop: angle mode reaches the commanded roll",
           fabsf(roll - target) < ak_deg2rad(3.0f));
    expect("closed loop: and settles rather than oscillating",
           fabsf(roll_rate) < 0.5f);
}

/*
 * The register model, and the tests that run against it.
 *
 * `test_arch()` and `test_arch_at32()` map a register block at
 * `PRIVATE_BASE 0xE000E000`, because that is where the part puts it and a model
 * somewhere else would be a model of a different part. Under AddressSanitizer
 * that address is inside the *shadow gap* - the 24 TB ASan reserves for the
 * shadow of addresses no 64-bit program has - and ASan refuses the mapping
 * unless `ASAN_OPTIONS=protect_shadow_gap=0`.
 *
 * That option is not narrow. With it on, ASan maps the gap as ordinary zeroed
 * memory, so its shadow reads back unpoisoned and an access *anywhere* in the
 * gap is permitted; measured, a write to `0xA0000000` (mapped by nothing) and a
 * write to `0x9bff9c00` (the shadow of the modelled page) both succeed, where
 * without it both are reported as `unknown-crash`. Everything outside the gap
 * reports identically either way, which is why the option was acceptable as a
 * first move - but "acceptable until the runner can split them" is a different
 * claim from "correct", and this is the split.
 *
 * **The group is six tests, and "two files" was the first answer.** What
 * belongs in it is not "the tests that map the block" but "the tests that run
 * against the model": the fault record reads its status registers out of the
 * private page, and all three board tests drive a whole boot sequence against
 * the peripheral one. Run with only the two arch files, those three did not
 * fail *loudly* - they asked whether an earlier test had mapped the pages, were
 * told no, and returned, reporting less than half the checks they were written
 * to make (measured: 1273 checks and 2 failed, against 2054 for the whole suite
 * as it stood then, and the board sections at zero). That is the defect this
 * split found, and it is fixed in those files: each now establishes the model
 * it needs.
 *
 * So the binary takes two arguments, and `make test` passes neither of them:
 *
 *   (no argument)     the whole suite in one process, exactly as before
 *   --only-registers  the six tests that run against the model, and nothing
 *                     else
 *   --no-registers    everything except those six
 *
 * **The runner and the wiring are both in this tree.** The selector came first,
 * as the half that makes closing the residue a build change rather than a code
 * change, and the `sanitize` target then took it up: it runs `--no-registers`
 * with the gap **protected** and `--only-registers` with it conceded - the two
 * invocations are thirty lines apart in the `SAN_ENV` block. This paragraph said
 * the wiring "is not in this tree" for as long as the split has been in it,
 * which is the standing hazard of a comment that describes a build: nothing
 * compiles it, and the reader who trusts it does not look.
 *
 * The check that the split is real is that the three runs agree with each
 * other: measured on this tree, 885 checks with `--only-registers` and 1384
 * with `--no-registers`, against 2269 for the whole run, zero failed in all
 * three, and 885 + 1384 = 2269. That is a measurement and not a formality - it
 * did not hold when this was first tried, which is how the defect above was
 * found rather than argued about.
 *
 * The numbers move whenever a test joins the group, and that is the point of
 * writing them down: this pair is the 2026-09-29 measurement, and it was
 * 786/1336/2122 when the split was made. `tests/test_board_feather.c` is the
 * most recent mover and it moved only the register half: one more board on a
 * part already in the group, which maps the register pages like the other two,
 * so all 54 of its checks run here and the suite went 2,215 (the reading in trap
 * 204) to 2,269.
 */
static int only_registers;
static int no_registers;

/*
 * The six tests that run against the register model, in one place.
 *
 * They are contiguous in the run and written once, so `--only-registers` and
 * the region `--no-registers` leaves out cannot drift apart. A selector that
 * ran a different set from the one the guard names would be the same class of
 * mistake as the guard was, one level up.
 *
 * `test_boot()` used to sit in the middle of these. It runs against the core
 * and not against a register block, so it moved out; and it could only move
 * once the five had stopped depending on each other's leftovers, which is what
 * the precondition fix in them was for.
 */
static void the_register_model(void)
{
    printf("\nthe port itself, running against a register block\n");
    test_arch();

    printf("\nthe fault record, which is read only when the board stops\n");
    test_fault();

    printf("\nthe board's own file: the boot sequence and the wiring\n");
    test_board_f405();

    printf("\nand a second board on the same part, whose header differs\n");
    test_board_feather();

    printf("\nthe third target's clock, which is not the F405's\n");
    test_arch_at32();

    printf("\nand the wing's board file, which is a different board\n");
    test_board_ghf435();
}

/* The whole suite. `--no-registers` leaves the five model tests out of it. */
static int run_all(void)
{
    printf("aerialkit host tests\n");

    printf("\nmath\n");
    test_math_against_libm();

    printf("\npid\n");
    test_pid();

    printf("\nestimator\n");
    test_estimator();
    test_the_gyro_filter();

    printf("\nflight loop\n");
    test_flight_angle_mode();
    test_flight_rate_mode();
    test_single_motor_wing_flies_without_yaw();
    test_flight_arming_gates();

    printf("\nclosed loop against a toy plant\n");
    test_flight_closed_loop();

    printf("\ncrsf receive\n");
    test_crsf();

    printf("\noutput encoding\n");
    test_output_encoding();

    printf("\ntext parsing and formatting\n");
    test_text();

    printf("\nparameters and console\n");
    test_params_and_cli();

    printf("\ndshot bit timing\n");
    test_dshot_timing();

    printf("\nport register encodings\n");
    test_register_encodings();

    if (!no_registers) {
        the_register_model();
    }

    printf("\nand where that boot has got to, for a board with no console\n");
    test_boot();

    printf("\nthe second target's outputs, which had never run anywhere\n");
    test_arch_esp32_output();

    printf("\nand its two sensor buses, against modelled devices\n");
    test_arch_esp32_bus();

    printf("\nand its two uarts, where the receiver and the gps are\n");
    test_arch_esp32_uart();

    printf("\nand its network, in the Wi-Fi build QEMU cannot run\n");
    test_arch_esp32_net();

    printf("\nand its pack's converter, which is the last file on that "
           "target a host had never run\n");
    test_arch_esp32_adc();

    printf("\nand how that target says why it restarted, and blinks\n");
    test_arch_esp32_fault();

    printf("\ni2c timing\n");
    test_i2c_timing();

    printf("\nreceive path\n");
    test_receive_path();

    printf("\ncrsf telemetry out\n");
    test_crsf_telemetry();

    printf("\nimu driver against a fake bus\n");
    test_imu();

    printf("\nthe bmi270 and its configuration file\n");
    test_bmi270();

    printf("\nthe lsm6dso, the one imu on the bus that is not spi\n");
    test_lsm6dso();

    printf("\nboard alignment and gyro bias\n");
    test_align_and_calibration();

    printf("\nmixer authority\n");
    test_mixer_authority();

    printf("\nmixer parity with the reference implementations\n");
    test_mixer_parity();

    printf("\nbarometer\n");
    test_barometer();

    printf("\nthe other barometer\n");
    test_bmp280();
    test_bmp388();

    printf("\nheight above the take-off point\n");
    test_altitude();

    printf("\nthe hand launch\n");
    test_launch();

    printf("\nhow far the ground is\n");
    test_rangefinder();

    printf("\nthe gyro's bias, measured at arm\n");
    test_gyro_arm_calibration();

    printf("\nthe flight pack\n");
    test_battery();

    printf("\nblackbox\n");
    test_blackbox();

    printf("\ngps\n");
    test_gps();

    printf("\nnavigation\n");
    test_navigation();

    printf("\ngps configuration\n");
    test_gps_config();

    printf("\nconfig protocol\n");
    test_protocol();

    printf("\nreceiver calibration\n");
    test_rc_calibration();

    printf("\nsbus, the other receiver in the drawer\n");
    test_sbus();

    printf("\naccelerometer calibration\n");
    test_accel_calibration();

    printf("\nselftest (the same one the board prints at boot)\n");
    int rc_selftest = ak_flight_selftest(printf_shim);
    expect("ak_flight_selftest passes", rc_selftest == 0);

    printf("\n%d checks, %d failed\n", checks, failures);
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--only-registers") == 0) {
            only_registers = 1;
        } else if (strcmp(argv[i], "--no-registers") == 0) {
            no_registers = 1;
        } else {
            /* Refused rather than ignored: a mistyped selector that ran the
             * whole suite would look like a pass. */
            fprintf(stderr,
                    "unknown argument: %s\n"
                    "usage: %s [--only-registers | --no-registers]\n",
                    argv[i], argv[0]);
            return 2;
        }
    }
    if (only_registers && no_registers) {
        fprintf(stderr,
                "--only-registers and --no-registers together select nothing\n");
        return 2;
    }

    if (only_registers) {
        printf("aerialkit host tests, the register model alone\n");
        the_register_model();
        printf("\n%d checks, %d failed\n", checks, failures);
        return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    return run_all();
}
