#ifndef AK_FLIGHT_PID_H
#define AK_FLIGHT_PID_H

/*
 * PID with the two details that matter in a rate loop:
 *
 *  - the derivative acts on the measurement, not on the error, so a stick step
 *    does not produce a derivative spike (the setpoint kick that shows up as a
 *    twitch on a fast airframe);
 *  - the integral is clamped and the output is clamped after the sum, so a
 *    held-out condition cannot wind up the term that is meant to fix it.
 *
 * No feed-forward, no gain scheduling, no autotune: those belong to the tuning
 * milestone, which needs an airframe to tune against.
 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_limit;     /* absolute clamp on the integral term */
    float out_limit;   /* absolute clamp on the output */
    float d_cutoff_hz; /* 0 disables the derivative low pass */
    float integral;
    float prev_meas;
    float d_filtered;
    int   have_prev;
} ak_pid_t;

void  ak_pid_init(ak_pid_t *pid, float kp, float ki, float kd,
                  float i_limit, float out_limit, float d_cutoff_hz);
void  ak_pid_reset(ak_pid_t *pid);
float ak_pid_update(ak_pid_t *pid, float setpoint, float measurement, float dt);

#endif /* AK_FLIGHT_PID_H */
