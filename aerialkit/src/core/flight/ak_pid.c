#include "ak_pid.h"

#include "ak_math.h"

void ak_pid_init(ak_pid_t *pid, float kp, float ki, float kd,
                 float i_limit, float out_limit, float d_cutoff_hz)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->out_limit = out_limit;
    pid->d_cutoff_hz = d_cutoff_hz;
    ak_pid_reset(pid);
}

void ak_pid_reset(ak_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filtered = 0.0f;
    pid->have_prev = 0;
}

float ak_pid_update(ak_pid_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;

    pid->integral = ak_clampf(pid->integral + pid->ki * error * dt,
                              -pid->i_limit, pid->i_limit);

    float d_term = 0.0f;
    if (pid->kd != 0.0f) {
        float rate = 0.0f;
        if (pid->have_prev) {
            rate = (measurement - pid->prev_meas) / dt;
        }
        float alpha = ak_lpf_alpha(pid->d_cutoff_hz, dt);
        pid->d_filtered += alpha * (rate - pid->d_filtered);
        d_term = pid->kd * pid->d_filtered;
    }
    pid->prev_meas = measurement;
    pid->have_prev = 1;

    float output = pid->kp * error + pid->integral - d_term;
    return ak_clampf(output, -pid->out_limit, pid->out_limit);
}
