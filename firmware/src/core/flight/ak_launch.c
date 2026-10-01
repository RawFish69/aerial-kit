#include "ak_launch.h"

#include "ak_math.h"

void ak_launch_init(ak_launch_t *launch)
{
    /*
     * The reference's numbers, and the reason for each is in the header: the
     * throttle and the climb angle are what INAV launches a wing with, the
     * timeout is its own limit on the manoeuvre, and the abort fraction is the
     * one this firmware already takes a mission back with.
     *
     * The climb is the one that is airframe. 18 degrees is INAV's default for
     * an aircraft that flies at ten to twenty metres a second on the wing; a
     * heavier wing, or a calm day with no wind to launch into, wants less. It is
     * a parameter for that reason, and nothing here has flown.
     */
    launch->cfg.throttle = 0.70f;
    launch->cfg.climb_deg = 18.0f;
    launch->cfg.abort_stick = 0.15f;
    launch->cfg.timeout_ms = 5000u;
    launch->active = 0;
    launch->started_ms = 0;
    launch->ended_because = AK_LAUNCH_REASON_NONE;
}

void ak_launch_start(ak_launch_t *launch, uint32_t now_ms)
{
    if (launch->active) {
        return;
    }
    launch->active = 1;
    launch->started_ms = now_ms;
    launch->ended_because = AK_LAUNCH_REASON_NONE;
}

void ak_launch_stop(ak_launch_t *launch, ak_launch_reason_t why)
{
    if (launch->active) {
        launch->ended_because = (int)why;
    }
    launch->active = 0;
}

int ak_launch_active(const ak_launch_t *launch)
{
    return launch->active;
}

int ak_launch_step(ak_launch_t *launch, const ak_rc_command_t *pilot,
                   float max_tilt_rad, uint32_t now_ms, ak_rc_command_t *out)
{
    if (!launch->active || pilot == 0 || out == 0) {
        return 0;
    }

    /*
     * The pilot's sticks end it, and this comes first: an aircraft being thrown
     * by somebody whose other hand is on the transmitter is the whole case this
     * exists for, and a stick that has moved means the pilot is flying it.
     * Yaw counts as much as pitch and roll here - a hand on the rudder is a
     * hand on the aircraft.
     */
    float abort = launch->cfg.abort_stick;
    if (ak_absf(pilot->roll) > abort || ak_absf(pilot->pitch) > abort ||
        ak_absf(pilot->yaw) > abort) {
        ak_launch_stop(launch, AK_LAUNCH_REASON_STICK);
        return 0;
    }

    if ((uint32_t)(now_ms - launch->started_ms) >= launch->cfg.timeout_ms) {
        ak_launch_stop(launch, AK_LAUNCH_REASON_TIMEOUT);
        return 0;
    }

    /*
     * The climb as a fraction of the travel the core scales by: the caller
     * passes the same limit the angle loop divides its stick input by, so "18
     * degrees" here is 18 degrees of attitude out of the loop. A climb angle
     * larger than the aircraft's own tilt limit is clamped rather than
     * commanded and then clipped by a second piece of code.
     */
    float climb_rad = ak_deg2rad(launch->cfg.climb_deg);
    float fraction = max_tilt_rad > 0.0f ? climb_rad / max_tilt_rad : 1.0f;

    out->roll = 0.0f;
    out->pitch = ak_clampf(fraction, 0.0f, 1.0f);
    out->yaw = 0.0f;
    out->throttle = ak_clampf(launch->cfg.throttle, 0.0f, 1.0f);
    /* Angle mode whatever the pilot's mode switch says: the launch is an
     * attitude to hold, and a rate command in the same two seconds would be an
     * aircraft rotating until somebody stops it. */
    out->angle_mode = 1;
    out->arm_request = pilot->arm_request;
    return 1;
}

const char *ak_launch_reason_name(ak_launch_reason_t why)
{
    switch (why) {
    case AK_LAUNCH_REASON_STICK:
        return "a stick";
    case AK_LAUNCH_REASON_TIMEOUT:
        return "the time";
    case AK_LAUNCH_REASON_OFF:
        return "the switch";
    case AK_LAUNCH_REASON_NONE:
    default:
        return "nothing";
    }
}
