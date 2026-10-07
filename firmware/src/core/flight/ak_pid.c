#include "ak_pid.h"

#include "ak_math.h"

void ak_pid_init(ak_pid_t *pid, float kp, float ki, float kd,
                 float i_limit, float out_limit, float d_lpf1_hz)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->out_limit = out_limit;

    pid->d_lpf1_hz = d_lpf1_hz;
    pid->d_lpf2_hz = 0.0f;
    /* No interval yet, so no coefficients: the first update builds them for
     * the dt it is actually called with. A `d_dt` of zero would be a real
     * interval and would let a wrong guess stand. */
    pid->d_dt = -1.0f;
    ak_filter_pt1_init(&pid->d_lpf1, d_lpf1_hz, 0.0f);
    ak_filter_pt1_init(&pid->d_lpf2, 0.0f, 0.0f);

    ak_pid_reset(pid);
}

void ak_pid_reset(ak_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filtered = 0.0f;
    pid->have_prev = 0;
    pid->p_term = 0.0f;
    ak_filter_pt1_reset(&pid->d_lpf1);
    ak_filter_pt1_reset(&pid->d_lpf2);
}

void ak_pid_set_dterm_lpf(ak_pid_t *pid, float lpf1_hz, float lpf2_hz, float dt)
{
    if (lpf1_hz == pid->d_lpf1_hz && lpf2_hz == pid->d_lpf2_hz &&
        dt == pid->d_dt) {
        return;
    }
    ak_filter_pt1_set(&pid->d_lpf1, lpf1_hz, dt);
    ak_filter_pt1_set(&pid->d_lpf2, lpf2_hz, dt);
    pid->d_lpf1_hz = lpf1_hz;
    pid->d_lpf2_hz = lpf2_hz;
    pid->d_dt = dt;
}

float ak_pid_update(ak_pid_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    pid->p_term = pid->kp * error;

    /*
     * Conditional integration: the integral moves only while the loop still has
     * output left to spend on it.
     *
     * The clamp below bounds the integral, and that bound is not the same
     * number as the output's. At the defaults - rate_i_limit 0.30 against
     * torque_limit 0.60 - it is half of full torque, and the two are set in
     * different places by different arguments: one is "how much steady-state
     * error should this loop be willing to trim out", the other is "how much of
     * the mixer is this axis allowed". Nothing made them agree, and nothing
     * could have, because the right value of the first depends on the second.
     *
     * What that costs is not a tuning matter, it is stored energy. A wing held
     * nose-up on the ground while armed, in angle mode, hands this loop a
     * setpoint of several radians per second against a gyro reading zero,
     * because the angle loop is looking at an attitude error that the hand
     * holding the aircraft is not going to let it correct. The proportional
     * term alone is already past the output limit; the integral then runs to its
     * own limit underneath it, and it is still there, full and saturated, at the
     * instant of the throw. Measured: 0.30 stored after one second, and 0.30
     * after twenty.
     *
     * The fix is the standard one and it is a property rather than a number:
     * **the integral can never grow past the point where the controller is
     * already doing everything it can.** With it, the two limits stop having to
     * agree, because the output's limit now bounds the integral too.
     *
     * The test is on the proportional term plus the candidate rather than the
     * full output, so the derivative term is not consulted. That is deliberate
     * and it errs one way: the derivative opposes, so `p + i - d` can be inside
     * the limit while `p + i` is outside, and this refuses to integrate in that
     * case even though it could have. The cost is a step of integration forgone
     * on a transient; the alternative is an anti-windup rule whose behaviour
     * depends on the D-term chain's state, which is a much harder thing to
     * reason about than one step of lost integration.
     *
     * Note which way this is *not* symmetric, because it is the case the loop
     * is for: an error the loop can actually reach still integrates exactly as
     * it did, to exactly the same value and at exactly the same rate. Backing
     * off a saturated axis - the integral unwinding - is likewise untouched.
     * Only the accumulation further into a limit that is already reached is
     * refused.
     */
    float candidate = ak_clampf(pid->integral + pid->ki * error * dt,
                                -pid->i_limit, pid->i_limit);
    if (!((pid->p_term + candidate > pid->out_limit && error > 0.0f) ||
          (pid->p_term + candidate < -pid->out_limit && error < 0.0f))) {
        pid->integral = candidate;
    }

    float d_term = 0.0f;
    if (pid->kd != 0.0f) {
        float rate = 0.0f;
        if (pid->have_prev) {
            rate = (measurement - pid->prev_meas) / dt;
        }
        /* Built for the interval this update is running at, whether or not
         * anybody moved the cutoff. Safe in any call order: a PID used on its
         * own gets its static coefficients on the first update, and one the
         * flight loop moves gets the numbers it was just given. */
        ak_pid_set_dterm_lpf(pid, pid->d_lpf1_hz, pid->d_lpf2_hz, dt);

        pid->d_filtered = ak_filter_pt1_apply(&pid->d_lpf1, rate);
        pid->d_filtered = ak_filter_pt1_apply(&pid->d_lpf2, pid->d_filtered);
        d_term = pid->kd * pid->d_filtered;
    }
    pid->prev_meas = measurement;
    pid->have_prev = 1;

    pid->p_term = pid->kp * error;
    float output = pid->p_term + pid->integral - d_term;
    return ak_clampf(output, -pid->out_limit, pid->out_limit);
}
