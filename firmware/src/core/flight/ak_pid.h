#ifndef AK_FLIGHT_PID_H
#define AK_FLIGHT_PID_H

#include "../filter/ak_filter.h"

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
 *
 * The derivative is filtered by the D-term chain from roadmap 2.2: two PT1
 * sections, the first with a cutoff that can follow the throttle. It is the
 * same shape as the gyro chain and it is here rather than in the flight loop
 * because the derivative is computed here - the raw difference of two samples
 * divided by dt is the noisiest signal on the aircraft, and it is noise the P
 * and I terms never see because they act on the error rather than on its
 * difference.
 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_limit;     /* absolute clamp on the integral term */
    float out_limit;   /* absolute clamp on the output */

    /* The D-term chain's coefficients, and what they are currently built for.
     * `d_lpf1_hz` is betaflight's `dterm_lpf1_static_hz` unless the caller has
     * moved it with `ak_pid_set_dterm_lpf` to follow the throttle. `d_lpf2_hz`
     * is `dterm_lpf2_static_hz` and is never dynamic - the reference applies
     * the throttle curve to each chain's *first* section only, and this is that
     * shape rather than an approximation of it. `d_dt` is the sample interval
     * the coefficients were built for; a change of loop rate or of the
     * integration plan's subdivision changes it, so it is compared and not
     * assumed. */
    ak_filter_pt1_t d_lpf1;
    ak_filter_pt1_t d_lpf2;
    float d_lpf1_hz;
    float d_lpf2_hz;
    float d_dt;

    float integral;
    float prev_meas;
    float d_filtered;
    int   have_prev;
    /* The proportional term the last update produced, `kp * error`, kept for
     * the blackbox (roadmap 4.1): the integral and the filtered derivative are
     * state and were always readable, while P was a local and could not be
     * logged. Written by every update and cleared by a reset; nothing in the
     * control law reads it back. */
    float p_term;
} ak_pid_t;

/* `d_lpf1_hz` is the D-term chain's first section: 0 disables it. This is the
 * static configuration, and the one a PID used on its own keeps. */
void  ak_pid_init(ak_pid_t *pid, float kp, float ki, float kd,
                  float i_limit, float out_limit, float d_lpf1_hz);

/* Clears the integrator and the derivative's state. The chain's coefficients
 * are kept: a reset throws away what the aircraft has measured, not what
 * somebody configured, and re-deriving them would put a filter back at its
 * default cutoff behind a caller's back. */
void  ak_pid_reset(ak_pid_t *pid);

/*
 * Move the D-term chain's two cutoffs, keeping its state - which is what a
 * dynamic low pass does when the throttle changes, and what this is for.
 *
 * The flight loop calls this once per integration piece with the first cutoff
 * resolved from the throttle (see `ak_flight_dyn_cutoff`). Calling it with the
 * same numbers is free: the coefficients are only rebuilt when one of them or
 * the sample interval actually moved, and `ak_pid_update` does that check too,
 * so a PID that nobody moves still gets coefficients built for its own dt.
 */
void  ak_pid_set_dterm_lpf(ak_pid_t *pid, float lpf1_hz, float lpf2_hz, float dt);

float ak_pid_update(ak_pid_t *pid, float setpoint, float measurement, float dt);

#endif /* AK_FLIGHT_PID_H */
