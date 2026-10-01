#ifndef AK_FLIGHT_LAUNCH_H
#define AK_FLIGHT_LAUNCH_H

#include "ak_rc.h"

/*
 * A hand launch, for the airframe that has to be thrown.
 *
 * Every other mode in this firmware can wait for the pilot to be holding the
 * sticks. A launch cannot: one hand is on the wing and the other is on the
 * transmitter, and the two seconds after the throw are the two seconds when an
 * aircraft with a stalled wing and no altitude needs the climb it was not given.
 * INAV has the same mode, entered from a switch and flown by a twelve-state
 * machine (navigation/navigation_fw_launch.c); what is here is the half of it
 * that matters on a first flight, and the numbers are its numbers:
 *
 *   - the throttle held during the launch is the reference's 70 per cent
 *     (`nav_fw_launch_thr` 1700 of 1000..2000),
 *   - the attitude is its climb angle, 18 degrees (`nav_fw_launch_climb_angle`,
 *     "attitude of model, not climb slope"),
 *   - it ends after its timeout, 5 seconds (`nav_fw_launch_timeout`), or when
 *     the pilot moves a stick - which is INAV's abort deadband
 *     (`nav_fw_launch_land_abort_deadband`, 100 of 820 counts, and this uses
 *     the fifteenth of travel a mission is taken back with, for the same
 *     reason: one fraction of travel that means "the pilot is flying").
 *
 * What is deliberately *not* here yet is the half that makes INAV's version
 * automatic: it watches the accelerometer and the GPS for the throw itself, so
 * the motors stay at idle until the aircraft is moving. AerialKit's simulator
 * has no throw in it - its wing's speed follows its throttle, with no
 * acceleration transient and no hand - so a detection threshold here would be a
 * number nobody could measure. The switch is the detection: the pilot arms with
 * the throttle down, raises the launch switch (the motors spool up and the
 * climb attitude is held), throws, and takes it back with a stick or lets the
 * timeout end it. See docs/24-launch.md.
 *
 * The guidance it produces is an angle-mode command: a fixed pitch attitude and
 * a fixed throttle, whatever the pilot's mode switch says, because the point is
 * a climb the pilot is not holding.
 */

typedef struct {
    float throttle;      /* the throttle the launch flies at, 0..1 */
    float climb_deg;     /* the pitch attitude it holds, degrees nose up */
    float abort_stick;   /* a stick past this fraction of travel ends it */
    uint32_t timeout_ms; /* and so does this, from the moment it started */
} ak_launch_config_t;

typedef struct {
    ak_launch_config_t cfg;
    int      active;
    uint32_t started_ms;
    /* What the last one ended with, for the console. Zero until one has. */
    int      ended_because;
} ak_launch_t;

/* Why a launch stopped flying the aircraft. */
typedef enum {
    AK_LAUNCH_REASON_NONE = 0,
    AK_LAUNCH_REASON_STICK,   /* the pilot moved a stick: it is theirs */
    AK_LAUNCH_REASON_TIMEOUT, /* it ran its course */
    AK_LAUNCH_REASON_OFF,     /* the switch went low, or the aircraft disarmed */
} ak_launch_reason_t;

void ak_launch_init(ak_launch_t *launch);

/* The switch went high while armed, or low. Starting one that is already active
 * does not restart its clock: the timeout is a limit on the manoeuvre, not a
 * thing a flapping switch can reset. */
void ak_launch_start(ak_launch_t *launch, uint32_t now_ms);
void ak_launch_stop(ak_launch_t *launch, ak_launch_reason_t why);

int ak_launch_active(const ak_launch_t *launch);

/*
 * One step. Fills `out` with the command the aircraft is being flown with and
 * returns 1 while the launch is flying it; returns 0 once it has ended, having
 * stopped itself, with the reason in `launch->ended_because`.
 *
 * `max_tilt_rad` is the core's own angle-mode limit: the climb is an attitude
 * in degrees and the command the core reads is a fraction of that travel, so
 * the two have to come from the same place or the climb is not the number the
 * pilot set.
 */
int ak_launch_step(ak_launch_t *launch, const ak_rc_command_t *pilot,
                   float max_tilt_rad, uint32_t now_ms, ak_rc_command_t *out);

const char *ak_launch_reason_name(ak_launch_reason_t why);

#endif /* AK_FLIGHT_LAUNCH_H */
