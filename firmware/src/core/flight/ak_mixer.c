#include "ak_mixer.h"

#include "ak_math.h"

void ak_mixer_apply_scaled(const ak_mixer_t *mixer, float throttle, float roll,
                           float pitch, float yaw, float motor_scale,
                           float servo_scale, ak_outputs_t *out)
{
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        out->motor[i] = 0.0f;
    }
    for (int i = 0; i < AK_MAX_SERVOS; i++) {
        out->servo[i] = 0.0f;
    }

    /* Rows are consumed in order within each kind: the nth motor row is motor
     * n, the nth servo row is servo n. Indexing by row position instead wasted
     * every servo row of a table that puts its servos first - which is exactly
     * what the elevon-wing table does, and what the selftest caught. */
    uint8_t motor = 0;
    uint8_t servo = 0;

    for (uint8_t row = 0; row < mixer->count; row++) {
        float scale = mixer->kind[row] == AK_OUT_SERVO ? servo_scale
                                                       : motor_scale;
        float value = mixer->offset[row] +
                      mixer->coeff[row][0] * throttle +
                      (mixer->coeff[row][1] * roll + mixer->coeff[row][2] * pitch +
                       mixer->coeff[row][3] * yaw) * scale;

        if (mixer->kind[row] == AK_OUT_SERVO) {
            value = ak_clampf(value, -1.0f, 1.0f);
            if (servo < AK_MAX_SERVOS) {
                out->servo[servo++] = value;
            }
        } else {
            value = ak_clampf(value, 0.0f, 1.0f);
            if (motor < AK_MAX_MOTORS) {
                out->motor[motor++] = value;
            }
        }
    }
}

void ak_mixer_apply(const ak_mixer_t *mixer, float throttle, float roll,
                    float pitch, float yaw, ak_outputs_t *out)
{
    ak_mixer_apply_scaled(mixer, throttle, roll, pitch, yaw, 1.0f, 1.0f, out);
}

/* How far the differential has to be turned down for every motor to stay
 * between the idle floor and full throttle. */
static float authority_scale(float base, float min_differential,
                             float max_differential, float idle)
{
    float scale = 1.0f;

    if (max_differential > 0.0f && base + max_differential > 1.0f) {
        float limit = (1.0f - base) / max_differential;
        if (limit < scale) {
            scale = limit;
        }
    }
    if (min_differential < 0.0f && base + min_differential < idle) {
        float limit = (base - idle) / -min_differential;
        if (limit < scale) {
            scale = limit;
        }
    }
    return ak_clampf(scale, 0.0f, 1.0f);
}

void ak_mixer_apply_limited(const ak_mixer_t *mixer, float throttle,
                            const float torque[3], ak_outputs_t *out)
{
    /* While armed, the motors never stop: the floor is the airframe's, and the
     * throttle never goes below it or above full. */
    float base = ak_clampf(throttle, mixer->motor_idle, 1.0f);

    float min_differential = 0.0f;
    float max_differential = 0.0f;

    for (uint8_t row = 0; row < mixer->count; row++) {
        if (mixer->kind[row] == AK_OUT_SERVO) {
            continue;
        }
        float differential = mixer->coeff[row][1] * torque[0] +
                             mixer->coeff[row][2] * torque[1] +
                             mixer->coeff[row][3] * torque[2];
        if (differential < min_differential) {
            min_differential = differential;
        }
        if (differential > max_differential) {
            max_differential = differential;
        }
    }

    float scale = authority_scale(base, min_differential, max_differential,
                                  mixer->motor_idle);
    ak_mixer_apply_scaled(mixer, base, torque[0], torque[1], torque[2], scale,
                          1.0f, out);
}

/*
 * Rows: RR, FR, RL, FL, which is the order every reference implementation in
 * this workspace uses and the order a person's motor numbering follows.
 *
 * The yaw column is the one that cannot be reasoned out on paper: which
 * diagonal spins which way is decided by the props and the ESC wiring, and a
 * table built for the other convention yaws the aircraft backwards on the
 * first flight. This one is the common convention - the counter-clockwise pair
 * is front-right and rear-left - and `make test` checks it against the pinned
 * upstreams rather than against anybody's memory. The authority limit cannot
 * catch it: a quad with the yaw column inverted flies perfectly, and turns the
 * wrong way.
 */
const ak_mixer_t ak_mixer_quad_x = {
    .name = "quad-x",
    .count = 4,
    .kind = {
        AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR,
        AK_OUT_SERVO, AK_OUT_SERVO,
    },
    /*
     * throttle, roll, pitch, yaw.
     *
     * The pitch column is the *negative* of Betaflight's QUADX table, and the
     * difference is a sign convention rather than a disagreement about
     * aircraft. This firmware's pitch is positive when the nose is *up*: its
     * estimator computes it that way (`pitch = atan2(-ax, ...)`, which is what
     * the accelerometer of a nose-up aircraft reads) and its elevon mix uses
     * it that way (up-elevon, nose up). Betaflight's is positive nose *down* -
     * see `attitude.values.pitch` and the "negative is backwards" comment in
     * its imu.c - so copying its table verbatim inverted this axis, and an
     * inverted pitch axis is not a stick-feel problem: the angle loop drove the
     * nose one way while the motors pushed it the other, which the simulator
     * showed as a 1,861 degree runaway the first time its quad plant modelled
     * thrust honestly (rear motors up, nose down).
     *
     * `set pitch` from a receiver is the same sign in both firmwares (channel
     * high is positive), so the *physical* behaviour is identical: stick
     * forward, nose down, rear motors faster. What differs is which way the
     * number in the middle points, and every table in this file has to agree
     * with the estimator and with each other. tests/test_mixer_parity.c checks
     * both - the numbers against Betaflight's, with this one convention
     * accounted for, and the physical direction, which is what a sign flip
     * gets wrong.
     */
    .coeff = {
        /* throttle, roll,  pitch, yaw */
        { 1.0f, -1.0f, -1.0f, -1.0f }, /* rear right  */
        { 1.0f, -1.0f,  1.0f,  1.0f }, /* front right */
        { 1.0f,  1.0f, -1.0f,  1.0f }, /* rear left   */
        { 1.0f,  1.0f,  1.0f, -1.0f }, /* front left  */
    },
    .offset = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .motor_idle = 0.05f,
};

/* Rows: left motor, right motor, left elevon, right elevon. */
const ak_mixer_t ak_mixer_elevon_wing = {
    .name = "elevon-wing",
    .count = 4,
    .kind = {
        AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_SERVO, AK_OUT_SERVO,
    },
    .coeff = {
        /* throttle, roll,  pitch, yaw */
        { 1.0f,  0.0f,  0.0f,  1.0f }, /* left motor  */
        { 1.0f,  0.0f,  0.0f, -1.0f }, /* right motor */
        { 0.0f, -1.0f,  1.0f,  0.0f }, /* left elevon */
        { 0.0f,  1.0f,  1.0f,  0.0f }, /* right elevon */
    },
    .offset = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .motor_idle = 0.0f, /* a wing's motors may stop: the throttle is a throttle */
    .fixed_wing = 1u,   /* and it cannot stop in the air: see the descend state */
};

/*
 * The same wing with one motor and no rudder. Rows: motor, left elevon, right
 * elevon.
 *
 * The twin above controls yaw with differential thrust: its two motor rows
 * carry opposite signs in the yaw column. One motor is one row, so that column
 * is zero - not because yaw does not matter, but because **there is nothing on
 * this aircraft that can produce it**. The airframe is a wing; a wing turns by
 * banking and pulling, and both of those are elevons.
 *
 * So this is the table where a zero *means* something, and the difference
 * matters. Every other zero here is "this output is not used for that axis" -
 * the quad-X's motor rows carry no roll from the throttle column and that is
 * unremarkable. This one is "this aircraft cannot do that", and a control law
 * that cannot tell the two apart will fly a yaw loop all the way to the ground:
 * the loop integrates the heading error it is being handed, the mixer multiplies
 * the result by zero and returns zero torque, and the aircraft never turns, so
 * the error never shrinks and the integral never stops growing. The torque is
 * discarded but the *state* is not, and that state is what `ak_mixer_has_axis`
 * exists to let the flight core see (ak_flight.c, control_armed).
 *
 * The elevon rows are byte-for-byte the twin's. That is deliberate: the same
 * airframe with the same control surfaces should fly the same elevons, and a
 * wing that behaved differently depending on how many motors it had would be a
 * bug with a plausible-looking table behind it.
 *
 * Signs are provisional in exactly the way the twin's are - see the header -
 * and the yaw column is not: there is no sign to get wrong.
 */
const ak_mixer_t ak_mixer_elevon_wing_single = {
    .name = "elevon-wing-single",
    .count = 3,
    .kind = {
        AK_OUT_MOTOR, AK_OUT_SERVO, AK_OUT_SERVO,
    },
    .coeff = {
        /* throttle, roll,  pitch, yaw */
        { 1.0f,  0.0f,  0.0f,  0.0f }, /* motor       */
        { 0.0f, -1.0f,  1.0f,  0.0f }, /* left elevon */
        { 0.0f,  1.0f,  1.0f,  0.0f }, /* right elevon */
    },
    .offset = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .motor_idle = 0.0f,
    .fixed_wing = 1u,
};

/*
 * The rest of the multirotor frames.
 *
 * Same source as quad-X above - Betaflight's `mixer_init.c` at the pinned
 * revision, transcribed with its pitch sign accounted for - and each one is a
 * *layout*: the same four controls against the same four axes, arranged for a
 * different frame. What differs between them is which corner is motor 1 and
 * which way the yaw column points, which is exactly the thing a person gets
 * wrong by hand and the thing a reference table settles.
 *
 * What is *not* here is any frame with more than four motors. `AK_MAX_MOTORS`
 * is four, so the hexacopters, octocopters and Y6 in the same reference table
 * wait on the output count, the DShot channels, the blackbox record's motor
 * field and the telemetry frame growing together - a change of its own,
 * deliberately not smuggled in behind a table. Four motors is the goal's
 * airframe; this is the list to work down when a hex arrives.
 */

/* Rows: front left, front right, rear right, rear left. The same aircraft as
 * quad-X with the motors numbered the way plenty of boards are wired - motor 1
 * at the front left - which is why it is a frame of its own rather than a
 * footnote: the props go on differently. */
const ak_mixer_t ak_mixer_quad_x_1234 = {
    .name = "quad-x-1234",
    .count = 4,
    .kind = {
        AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR,
    },
    .coeff = {
        /* throttle, roll,  pitch, yaw */
        { 1.0f,  1.0f,  1.0f, -1.0f }, /* front left  */
        { 1.0f, -1.0f,  1.0f,  1.0f }, /* front right */
        { 1.0f, -1.0f, -1.0f, -1.0f }, /* rear right  */
        { 1.0f,  1.0f, -1.0f,  1.0f }, /* rear left   */
    },
    .offset = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .motor_idle = 0.05f,
};

/* Rows: rear, right, left, front - the four arms at the compass points. */
const ak_mixer_t ak_mixer_quad_p = {
    .name = "quad-p",
    .count = 4,
    .kind = {
        AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR,
    },
    .coeff = {
        /* throttle, roll,  pitch, yaw */
        { 1.0f,  0.0f, -1.0f, -1.0f }, /* rear  */
        { 1.0f, -1.0f,  0.0f,  1.0f }, /* right */
        { 1.0f,  1.0f,  0.0f,  1.0f }, /* left  */
        { 1.0f,  0.0f,  1.0f, -1.0f }, /* front */
    },
    .offset = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .motor_idle = 0.05f,
};

/* Rows: rear top, front right, rear bottom, front left. Two motors up and two
 * down on the same arms, so roll comes from the pair that is left-right and
 * pitch from the pair that is fore-aft - which is why this table has zeros
 * where the others have numbers. */
const ak_mixer_t ak_mixer_y4 = {
    .name = "y4",
    .count = 4,
    .kind = {
        AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR,
    },
    .coeff = {
        /* throttle, roll,  pitch, yaw */
        { 1.0f,  0.0f, -1.0f, -1.0f }, /* rear top    */
        { 1.0f, -1.0f,  1.0f,  0.0f }, /* front right */
        { 1.0f,  0.0f, -1.0f,  1.0f }, /* rear bottom */
        { 1.0f,  1.0f,  1.0f,  0.0f }, /* front left  */
    },
    .offset = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .motor_idle = 0.05f,
};

/* Rows: rear right, front right, rear left, front left, with the rear arms
 * swept into a V. The unequal numbers are the reference's, not a rounding: the
 * swept arms give the rear motors less authority in pitch and more in yaw, and
 * the table says so rather than pretending a V-tail is an X. */
const ak_mixer_t ak_mixer_vtail4 = {
    .name = "vtail4",
    .count = 4,
    .kind = {
        AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR,
    },
    .coeff = {
        /* throttle, roll,  pitch, yaw */
        { 1.0f, -0.58f, -0.58f,  1.0f }, /* rear right  */
        { 1.0f, -0.46f,  0.39f, -0.5f }, /* front right */
        { 1.0f,  0.58f, -0.58f, -1.0f }, /* rear left   */
        { 1.0f,  0.46f,  0.39f,  0.5f }, /* front left  */
    },
    .offset = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .motor_idle = 0.05f,
};

/*
 * Rows: rear, right, left, then the tail servo.
 *
 * The reference's tricopter table has three motors and no yaw column at all:
 * yaw is the *servo* that tilts the tail rotor, and Betaflight carries it in
 * the servo mixer rather than in the motor mixer. This firmware has one table
 * for both - a row says whether it drives a motor or a servo - so the tail is
 * the fourth row here, and the yaw column reaches it and nothing else. How far
 * it moves is the board's own `servo1_travel_us`: the travel is a property of
 * the linkage, not of the frame.
 */
const ak_mixer_t ak_mixer_tri = {
    .name = "tri",
    .count = 4,
    .kind = {
        AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_MOTOR, AK_OUT_SERVO,
    },
    .coeff = {
        /* throttle, roll,  pitch, yaw */
        { 1.0f,  0.0f, -1.333333f, 0.0f }, /* rear       */
        { 1.0f, -1.0f,  0.666667f, 0.0f }, /* right      */
        { 1.0f,  1.0f,  0.666667f, 0.0f }, /* left       */
        { 0.0f,  0.0f,  0.0f,      1.0f }, /* tail servo */
    },
    .offset = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .motor_idle = 0.05f,
};

const ak_mixer_t *ak_mixer_for_airframe(uint32_t airframe)
{
    switch (airframe) {
    case 1u:
        return &ak_mixer_elevon_wing;
    case 2u:
        return &ak_mixer_quad_x_1234;
    case 3u:
        return &ak_mixer_quad_p;
    case 4u:
        return &ak_mixer_y4;
    case 5u:
        return &ak_mixer_vtail4;
    case 6u:
        return &ak_mixer_tri;
    case 7u:
        return &ak_mixer_elevon_wing_single;
    default:
        return &ak_mixer_quad_x;
    }
}

int ak_mixer_has_axis(const ak_mixer_t *mixer, unsigned axis)
{
    if (mixer == 0 || axis >= AK_MIXER_CONTROL_AXES) {
        /* Not a question about an axis. 0 is roll and 2 is yaw; the throttle is
         * not one of the three, and answering "yes" for it would let a caller
         * treat the throttle as a control axis. */
        return 0;
    }
    /* `axis` counts the three control axes and the table's columns start with
     * the throttle, so roll is column 1 rather than column 0. Off by one here
     * is silent: every table has a non-zero pitch column, so reading the pitch
     * column for the yaw question says a rudderless wing can yaw. */
    unsigned col = axis + AK_MIXER_COL_ROLL;

    for (uint8_t row = 0u; row < mixer->count; row++) {
        if (mixer->coeff[row][col] != 0.0f) {
            return 1;
        }
    }
    return 0;
}
