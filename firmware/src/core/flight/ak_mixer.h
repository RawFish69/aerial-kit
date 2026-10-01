#ifndef AK_FLIGHT_MIXER_H
#define AK_FLIGHT_MIXER_H

#include "ak_types.h"

/*
 * A mixer is a table, not a function.
 *
 * Each output row says what kind of output it is and how much throttle, roll,
 * pitch and yaw it gets. Adding an airframe is adding a table; the control code
 * never learns what an elevon is. It is also the only place a sign lives, so a
 * reversed rudder is one number in one row.
 *
 * Columns, in order: throttle, roll, pitch, yaw.
 */

typedef enum {
    AK_OUT_MOTOR = 0,
    AK_OUT_SERVO = 1,
} ak_output_kind_t;

#define AK_MIXER_AXES 4
/*
 * A table has four columns, and the first of them is the throttle - which is
 * not one of the axes a control law has a loop for. So `AK_MIXER_AXES` is the
 * *column* count and the three control axes come after the throttle: roll is
 * column 1, pitch 2, yaw 3.
 *
 * The names are here because confusing the two is silent rather than loud.
 * Every table in ak_mixer.c has a non-zero pitch column, so asking about yaw
 * by reading `coeff[row][2]` answers "yes" on a wing with no rudder; and
 * reading column 0 for the roll question answers "yes" always, because every
 * row carries throttle. `tests/test_mix.c` caught exactly that on the first run
 * of `ak_mixer_has_axis`, which until then was reading the pitch column to
 * answer a question about yaw.
 */
#define AK_MIXER_COL_THROTTLE 0u
#define AK_MIXER_COL_ROLL     1u
#define AK_MIXER_COL_PITCH    2u
#define AK_MIXER_COL_YAW      3u
/* How many axes a control law has - roll, pitch, yaw. The torque, setpoint and
 * gyro arrays are this long, and it is what `ak_mixer_has_axis` takes. */
#define AK_MIXER_CONTROL_AXES 3u
#define AK_MIXER_ROWS (AK_MAX_MOTORS + AK_MAX_SERVOS)

typedef struct {
    const char *name;
    uint8_t     count;
    uint8_t     kind[AK_MIXER_ROWS];
    float       coeff[AK_MIXER_ROWS][AK_MIXER_AXES];
    float       offset[AK_MIXER_ROWS];
    /* The lowest a motor may turn while armed. A quad wants its motors
     * spinning at idle - yaw control is differential thrust, and stopped
     * motors have none - while a fixed wing wants them stopped, because the
     * throttle stick is a throttle there, not a safety. */
    float       motor_idle;
    /*
     * And what the *failsafe* needs to know: whether this aircraft can stop.
     *
     * A quadrotor that loses its link stops, because stopping is what it does
     * - it is held up by thrust and there is nothing to gain by holding. A
     * fixed wing cannot: with the motor stopped and the servos centred it is a
     * brick with wings, and the faster it is going the worse the arrival. So
     * this flag is what sends a wing down in a circle instead (see the
     * descend state in ak_flight.c) and it belongs to the airframe, not to the
     * failsafe.
     */
    uint8_t     fixed_wing;
} ak_mixer_t;

/* Fills out->motor[] and out->servo[] completely, so a shorter airframe cannot
 * inherit the previous frame's values in the slots it does not use. */
void ak_mixer_apply(const ak_mixer_t *mixer, float throttle, float roll,
                    float pitch, float yaw, ak_outputs_t *out);

/* The same, with the axes scaled per output kind: what the authority limit
 * needs, and what a mixer test wants to poke at. */
void ak_mixer_apply_scaled(const ak_mixer_t *mixer, float throttle, float roll,
                           float pitch, float yaw, float motor_scale,
                           float servo_scale, ak_outputs_t *out);

/* The mix an armed aircraft actually flies: the torque the control loop asked
 * for, reduced as far as it has to be to fit between the motor idle floor and
 * full throttle, and no further.
 *
 * This is the difference between a mixer and a flight controller. Clamping each
 * motor on its own - which is what a bare mixer does - throws away the
 * requested thrust and hands back a different aircraft: at full throttle a roll
 * command becomes asymmetric thrust, and at idle it becomes no yaw authority at
 * all. Reducing the *differential* keeps the thrust the pilot asked for and
 * gives up only authority, which is the trade a pilot expects. */
void ak_mixer_apply_limited(const ak_mixer_t *mixer, float throttle,
                            const float torque[3], ak_outputs_t *out);

/* Quadrotor, X layout. Motor rows are, in this order:
 *   0 rear right, 1 front right, 2 rear left, 3 front left
 * Signs: roll right lowers the right pair, nose-up raises the *front* pair -
 * more thrust at the back of an aircraft pushes the nose *down*, which is why
 * this table's pitch column is the negative of Betaflight's - and yaw right
 * raises the pair this table assumes turns counter-clockwise. Which motor
 * actually turns which way is set by props and ESC wiring, so the yaw column is
 * the one to check first on a new build - with the props off.
 *
 * This comment said "nose-up raises the rear pair" until 2026-09-19, and the
 * table agreed with it, and both were wrong about the aircraft:
 * tests/test_mixer_parity.c states the convention in physics for that reason. */
extern const ak_mixer_t ak_mixer_quad_x;

/* Twin-motor flying wing with elevons, the twin-wings airframe:
 *   0 left motor, 1 right motor, servo 0 left elevon, servo 1 right elevon.
 * Throttle is the mean of the two motors and yaw is differential thrust.
 * Elevon signs assume the usual convention (positive pitch = trailing edge up,
 * positive roll = right roll). Both are provisional until a servo is on the
 * bench: no servos have been connected to this firmware. */
extern const ak_mixer_t ak_mixer_elevon_wing;

/* Single-motor flying wing with elevons: the same aircraft as the twin above
 * with one motor instead of two, and **no rudder**.
 *   0 motor, servo 0 left elevon, servo 1 right elevon.
 *
 * This is the airframe whose yaw column is all zeros, and that column is a
 * statement rather than a placeholder: there is no actuator on this aircraft
 * that can produce yaw torque. The twin above yaws by differential thrust; with
 * one motor that mechanism is gone, and nothing takes its place. Course turns
 * are made by banking and pulling, which is what the elevons are for.
 *
 * A zero column is therefore load-bearing in a way the other tables' zeros are
 * not, and the control law has to *read* it rather than multiply by it: a yaw
 * loop left running against this airframe integrates an error it can never
 * reduce and asks for a torque nothing can deliver. `ak_mixer_has_axis` is how
 * it asks, and ak_flight.c holds such an axis inert.
 *
 * Elevon signs are the twin's, for the same reason and with the same caveat:
 * provisional until a servo is on the bench. No servos have been connected to
 * this firmware, and no aircraft has flown. */
extern const ak_mixer_t ak_mixer_elevon_wing_single;

/*
 * Which axes this airframe can actually move.
 *
 * `axis` is 0 roll, 1 pitch, 2 yaw. True when at least one output row carries a
 * non-zero coefficient on that axis - which is a question about the *mixer*,
 * because the mixer is the only thing in this firmware that knows what is
 * bolted to the aircraft.
 *
 * It exists because a zero column in a table and "this axis has no actuator"
 * are the same fact, and only one of them can be checked. The control law asks
 * this so that a wing with no rudder does not fly a yaw controller that is
 * arguing with nobody (docs/04-flight-core.md).
 */
int ak_mixer_has_axis(const ak_mixer_t *mixer, unsigned axis);

/*
 * And the rest of the multirotor layouts, all four-motors-or-fewer and all
 * taken from the same reference table quad-X above comes from. Each is the
 * same four controls against the same four axes arranged for a different
 * frame, which is a difference of motor numbering and yaw direction rather
 * than of control law - and the one thing about a mixer that no bench can
 * check with the props off.
 */
/* Motor rows: 0 front left, 1 front right, 2 rear right, 3 rear left. */
extern const ak_mixer_t ak_mixer_quad_x_1234;
/* Motor rows: 0 rear, 1 right, 2 left, 3 front. */
extern const ak_mixer_t ak_mixer_quad_p;
/* Motor rows: 0 rear top, 1 front right, 2 rear bottom, 3 front left. */
extern const ak_mixer_t ak_mixer_y4;
/* Motor rows: 0 rear right, 1 front right, 2 rear left, 3 front left. */
extern const ak_mixer_t ak_mixer_vtail4;
/* Motor rows 0..2 rear, right, left; servo 0 is the tail rotor's tilt. */
extern const ak_mixer_t ak_mixer_tri;

/* How many airframe numbers there are: the `airframe` parameter's range is
 * 0..AK_MIXER_AIRFRAMES-1, and the list above is that range in order. */
#define AK_MIXER_AIRFRAMES 8u

/*
 * The mix an airframe flies, by the number the `airframe` parameter uses: 0 is
 * a quad-X, 1 is an elevon wing, 2 is a quad-X in the 1234 motor numbering, 3
 * is a quad plus, 4 is a Y4, 5 is a V-tail quad, 6 is a tricopter and 7 is a
 * single-motor elevon wing. Anything outside the range is a quad-X: a saved
 * configuration from a build that had fewer frames must fly something rather
 * than nothing.
 *
 * These numbers are in flash. A configuration saved by any build of this
 * firmware means the same aircraft in every later one, so a new airframe takes
 * the next number rather than a place in the list where it would read better -
 * 7 is the single-motor wing and not, say, 2, because 2 is already a quad-X in
 * somebody's saved configuration. The list order and the numbering are the same
 * thing here and neither may be sorted.
 *
 * One function rather than the same ternary in two places, because the
 * airframe chooses *two* things - the mix here and the navigator's return
 * profile in ak_nav.c - and the failure of setting one and not the other is a
 * wing flying a quadrotor's mix. The preflight asks this and the navigator's
 * profile whether they still agree.
 */
const ak_mixer_t *ak_mixer_for_airframe(uint32_t airframe);

#endif /* AK_FLIGHT_MIXER_H */
