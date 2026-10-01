/*
 * Mixer parity, against the reference implementations.
 *
 * The mixer is the one place in a flight controller where a wrong number is
 * invisible until it is airborne: a quad with its yaw column inverted flies
 * perfectly and turns the wrong way, and a quad with two rows swapped pirouettes
 * on take-off. Every sign in it is a claim about a physical aircraft, and the
 * only thing that can check a claim like that without a propeller is the
 * implementation somebody else already flew.
 *
 * So this compares ours against theirs two ways, and they catch different
 * mistakes:
 *
 *   - the tables, coefficient by coefficient, which is exact and would catch a
 *     swapped row, a flipped sign or a different motor order; and
 *   - the arithmetic, over a grid of inputs, which is what would catch the
 *     table being right while the code that consumes it is not.
 *
 * What it cannot check is the part that is not in either table: which
 * physical motor is on which corner is the airframe's business, and the
 * yaw axis depends on which way the props are mounted. The reference simply
 * says which convention the rest of the world uses, and matching it is the
 * point - a person who has built a quad before wires it the way every other
 * flight controller expects.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ak_mixer.h"
#include "mixer_reference.h"
#include "tests.h"

/* Our rows in the reference's order, for the tables that have a reference. */
static void test_quad_x_table_matches(void)
{
    static const char *names[MIXER_REFERENCE_ROWS] = {
        "rear right", "front right", "rear left", "front left",
    };

    expect("the quad-X mixer has one row per reference row",
           ak_mixer_quad_x.count >= MIXER_REFERENCE_ROWS);

    int mismatches = 0;
    for (int row = 0; row < MIXER_REFERENCE_ROWS; row++) {
        const float *ours = ak_mixer_quad_x.coeff[row];
        const mixer_reference_row_t *theirs = &mixer_reference_quad_x[row];

        if (fabsf(ours[0] - theirs->throttle) > 0.0001f ||
            fabsf(ours[1] - theirs->roll) > 0.0001f ||
            fabsf(ours[2] -
                  theirs->pitch * MIXER_REFERENCE_PITCH_SIGN) > 0.0001f ||
            fabsf(ours[3] - theirs->yaw) > 0.0001f) {
            printf("        %s: ours %.2f %.2f %.2f %.2f, reference "
                   "%.2f %.2f %.2f %.2f\n",
                   names[row], (double)ours[0], (double)ours[1],
                   (double)ours[2], (double)ours[3],
                   (double)theirs->throttle, (double)theirs->roll,
                   (double)theirs->pitch, (double)theirs->yaw);
            mismatches++;
        }
    }

    expect("every quad-X row matches the reference, in the reference's order, "
           "with its pitch convention accounted for",
           mismatches == 0);
    expect("and every row is a motor",
           ak_mixer_quad_x.kind[0] == AK_OUT_MOTOR &&
           ak_mixer_quad_x.kind[1] == AK_OUT_MOTOR &&
           ak_mixer_quad_x.kind[2] == AK_OUT_MOTOR &&
           ak_mixer_quad_x.kind[3] == AK_OUT_MOTOR);
}

/*
 * The same table through the code that flies it: for inputs that stay inside
 * the output range, our mixer has to produce the reference's numbers exactly.
 * The authority limit and the idle floor are ours and are not part of the
 * comparison - this is the mix underneath them.
 */
static void test_quad_x_arithmetic_matches(void)
{
    /* throttle, roll, pitch, yaw */
    static const float inputs[][4] = {
        { 0.00f,  0.00f,  0.00f,  0.00f },
        { 0.50f,  0.00f,  0.00f,  0.00f },
        { 0.50f,  0.20f,  0.00f,  0.00f },
        { 0.50f, -0.20f,  0.00f,  0.00f },
        { 0.50f,  0.00f,  0.20f,  0.00f },
        { 0.50f,  0.00f, -0.20f,  0.00f },
        { 0.50f,  0.00f,  0.00f,  0.20f },
        { 0.50f,  0.00f,  0.00f, -0.20f },
        { 0.40f,  0.10f,  0.10f,  0.10f },
        { 0.40f, -0.10f, -0.10f, -0.10f },
        { 0.60f,  0.05f, -0.05f,  0.05f },
    };

    int mismatches = 0;
    for (unsigned i = 0; i < sizeof inputs / sizeof inputs[0]; i++) {
        float throttle = inputs[i][0];
        float roll = inputs[i][1];
        float pitch = inputs[i][2];
        float yaw = inputs[i][3];

        ak_outputs_t out;
        ak_mixer_apply(&ak_mixer_quad_x, throttle, roll, pitch, yaw, &out);

        for (int row = 0; row < MIXER_REFERENCE_ROWS; row++) {
            float want = mixer_reference_apply(
                &mixer_reference_quad_x[row], throttle, roll,
                pitch * MIXER_REFERENCE_PITCH_SIGN, yaw);
            /* Only where neither implementation would clamp. */
            if (want < 0.0f || want > 1.0f) {
                continue;
            }
            if (fabsf(out.motor[row] - want) > 0.0001f) {
                printf("        input %u row %d: ours %.3f, reference %.3f\n",
                       i, row, (double)out.motor[row], (double)want);
                mismatches++;
            }
        }
    }

    expect("the quad-X mix produces the reference's numbers", mismatches == 0);
}

/*
 * The conventions themselves, stated as the checks a person would make on a
 * bench with the props off. These are the sentences that matter if the table
 * ever gets edited: what "roll right" means, what "nose up" means, and which
 * pair speeds up for a yaw.
 */
static void test_the_conventions_are_the_usual_ones(void)
{
    ak_outputs_t out;

    ak_mixer_apply(&ak_mixer_quad_x, 0.5f, 0.0f, 0.0f, 0.0f, &out);
    expect("a quad at half throttle is symmetric",
           fabsf(out.motor[0] - out.motor[2]) < 0.0001f &&
           fabsf(out.motor[0] - out.motor[1]) < 0.0001f);

    /* Roll right lowers the right pair. Rows are RR, FR, RL, FL. */
    ak_mixer_apply(&ak_mixer_quad_x, 0.5f, 1.0f, 0.0f, 0.0f, &out);
    expect("roll right drops the right pair",
           out.motor[0] < out.motor[2] && out.motor[1] < out.motor[3]);

    /*
     * Nose up raises the *front* pair - which is the check that was wrong here
     * for as long as the table was a verbatim copy of Betaflight's, whose pitch
     * axis points the other way. The old text said "nose up raises the rear
     * pair" and passed, because the table and the sentence agreed with each
     * other and neither agreed with an aircraft: more thrust at the back of an
     * aircraft pushes the nose *down*. The simulator found it the first time
     * its quad plant modelled thrust honestly, as a 1,861 degree runaway.
     */
    ak_mixer_apply(&ak_mixer_quad_x, 0.5f, 0.0f, 1.0f, 0.0f, &out);
    expect("nose up raises the front pair, which is what pitches a nose up",
           out.motor[1] > out.motor[0] && out.motor[3] > out.motor[2]);

    /* And the same thing in the other direction, because a sign error is
     * symmetric: the stick forward (negative, which is what the estimator
     * calls nose down) has to speed the rear pair up. */
    ak_mixer_apply(&ak_mixer_quad_x, 0.5f, 0.0f, -1.0f, 0.0f, &out);
    expect("and a nose-down command raises the rear pair",
           out.motor[0] > out.motor[1] && out.motor[2] > out.motor[3]);

    /* Yaw right speeds up the counter-clockwise pair, which in the convention
     * both reference implementations use is front-right and rear-left. */
    ak_mixer_apply(&ak_mixer_quad_x, 0.5f, 0.0f, 0.0f, 1.0f, &out);
    expect("yaw right speeds up front-right and rear-left",
           out.motor[1] > out.motor[0] && out.motor[2] > out.motor[3]);

    /* And the reference says the same thing, without our code in the way. */
    expect("which is what the reference says too",
           mixer_reference_apply(&mixer_reference_quad_x[1], 0.5f, 0.0f, 0.0f,
                                 1.0f) >
               mixer_reference_apply(&mixer_reference_quad_x[0], 0.5f, 0.0f,
                                     0.0f, 1.0f));
}

/*
 * The wing's reference table is the aircraft's own mixer.
 *
 * A twin-motor flying wing with elevons is this project's airframe and neither
 * upstream ships that table - but the aircraft *flies* one today, and its saved
 * configuration is in this repository
 * (`projects/twin-wings/ghf435-inav/inav-preset-label-set.txt`, the session
 * that ends in `save`):
 *
 *     mmix 0  1.000  0.000  0.000  0.500    mmix 1  1.000  0.000  0.000 -0.500
 *     smix 0 1 0  50 0 -1   smix 1 1 1  50 0 -1
 *     smix 2 2 0 -50 0 -1   smix 3 2 1  50 0 -1
 *
 * INAV's `mmix`/`smix` numbers are percentages, and its table is this one with
 * **half** the coefficients on every control axis - a normalisation, not a
 * difference in the aircraft: this firmware's convention is "full-scale
 * coefficient, clamped by `torque_limit`", INAV's is "half-scale coefficient,
 * clamped by its ±500 output". The two conventions have to stay in step, which
 * is what the last check here is for. So what the wing's checks do is:
 *
 * 1. its own conventions, which its pages claim - elevons opposite for roll,
 *    together for pitch, thrust for yaw;
 * 2. the same sign convention as the quad on the axes they share;
 * 3. and the aircraft's own numbers, up to the two conventions the pages name
 *    (the scale, and which way round the servo horns are - INAV's `-1` reverse
 *    flag versus ours).
 */
static void test_wing_conventions(void)
{
    ak_outputs_t out;

    ak_mixer_apply(&ak_mixer_elevon_wing, 0.5f, 0.0f, 0.0f, 0.0f, &out);
    expect("a wing at half throttle has level elevons and even thrust",
           fabsf(out.servo[0]) < 0.0001f && fabsf(out.servo[1]) < 0.0001f &&
           fabsf(out.motor[0] - out.motor[1]) < 0.0001f);

    ak_mixer_apply(&ak_mixer_elevon_wing, 0.5f, 0.1f, 0.0f, 0.0f, &out);
    expect("roll right moves the elevons opposite ways",
           out.servo[0] > 0.0f && out.servo[1] < 0.0f);

    ak_mixer_apply(&ak_mixer_elevon_wing, 0.5f, 0.0f, 0.1f, 0.0f, &out);
    expect("nose up moves both elevons the same way",
           out.servo[0] > 0.0f && out.servo[1] > 0.0f);

    /* The same sign convention as the quad: yaw right means nose right, which
     * for differential thrust means the left motor pushes harder. */
    ak_mixer_apply(&ak_mixer_elevon_wing, 0.5f, 0.0f, 0.0f, 1.0f, &out);
    expect("yaw right pushes the left motor harder",
           out.motor[0] > out.motor[1]);

    /*
     * And against the aircraft's own mixer, row by row and axis by axis. The
     * captured table, as fractions, with INAV's servo-reverse flags already
     * applied to the signs:
     *
     *   motor 1   throttle 1.0, roll 0,    pitch 0,    yaw +0.5
     *   motor 2   throttle 1.0, roll 0,    pitch 0,    yaw -0.5
     *   servo 1   throttle 0,   roll -0.5, pitch -0.5, yaw 0
     *   servo 2   throttle 0,   roll +0.5, pitch -0.5, yaw 0
     *
     * Ours is that table times two, with the servos negated - and the two
     * conventions are the ones the comment above names. What this check stops
     * is one of them being changed alone: halving the coefficients here without
     * meaning to, or flipping a servo sign, is a *different aircraft*, and it
     * would otherwise be invisible because both tables look plausible.
     */
    static const float captured[4][4] = {
        { 1.0f,  0.0f,  0.0f,  0.5f },
        { 1.0f,  0.0f,  0.0f, -0.5f },
        { 0.0f, -0.5f, -0.5f,  0.0f },
        { 0.0f,  0.5f, -0.5f,  0.0f },
    };
    int same = 1;
    for (unsigned row = 0; row < 4u; row++) {
        /*
         * Two conventions between the tables, and only the control axes have
         * both of them:
         *
         * * the **scale**: INAV's coefficients are half ours on roll, pitch and
         *   yaw (its `torque`-equivalent is the ±500 clamp, ours is
         *   `torque_limit`), and this one's throttle coefficient is 1.0 - the
         *   mean of the motors - where a doubled one would be twice the
         *   throttle at full stick;
         * * the **servo direction**: INAV's `-1` reverse flag is our sign, and
         *   a horn goes on the way the builder mounts it.
         */
        int servo = ak_mixer_elevon_wing.kind[row] == AK_OUT_SERVO;
        for (unsigned axis = 0; axis < 4u; axis++) {
            float sign = axis == 0 ? 1.0f : (servo ? -2.0f : 2.0f);
            if (fabsf(ak_mixer_elevon_wing.coeff[row][axis] -
                      sign * captured[row][axis]) > 0.0001f) {
                same = 0;
            }
        }
    }
    expect("and it is the aircraft's own mixer up to its scale and its servo "
           "direction", same);
}

/*
 * Every frame that has a reference, against it: the same four numbers in the
 * same row order, with the reference's pitch convention accounted for. Row
 * order is the part of a mixer that nobody can check by reading their own
 * aircraft - a table with the right numbers in the wrong order takes off and
 * pirouettes - so it is compared to the table every other flight controller
 * ships.
 */
static void test_every_frame_matches_its_reference(void)
{
    static const char *const names_x[] = {
        "rear right", "front right", "rear left", "front left",
    };
    static const char *const names_x1234[] = {
        "front left", "front right", "rear right", "rear left",
    };
    static const char *const names_p[] = { "rear", "right", "left", "front" };
    static const char *const names_y4[] = {
        "rear top", "front right", "rear bottom", "front left",
    };
    static const char *const names_vtail4[] = {
        "rear right", "front right", "rear left", "front left",
    };
    static const char *const names_tri[] = { "rear", "right", "left" };

    static const struct {
        const char *frame;
        const ak_mixer_t *mixer;
        const mixer_reference_row_t *reference;
        unsigned rows;
        const char *const *names;
    } cases[] = {
        { "quad-x", &ak_mixer_quad_x, mixer_reference_quad_x, 4u, names_x },
        { "quad-x-1234", &ak_mixer_quad_x_1234, mixer_reference_quad_x_1234,
          4u, names_x1234 },
        { "quad-p", &ak_mixer_quad_p, mixer_reference_quad_p, 4u, names_p },
        { "y4", &ak_mixer_y4, mixer_reference_y4, 4u, names_y4 },
        { "vtail4", &ak_mixer_vtail4, mixer_reference_vtail4, 4u, names_vtail4 },
        /* Three motor rows: the tail's tilt is the reference's *servo* mixer,
         * and this firmware's fourth row - checked below. */
        { "tri", &ak_mixer_tri, mixer_reference_tri, 3u, names_tri },
    };

    for (unsigned c = 0; c < sizeof cases / sizeof cases[0]; c++) {
        char what[96];
        int mismatches = 0;

        for (unsigned row = 0; row < cases[c].rows; row++) {
            const float *ours = cases[c].mixer->coeff[row];
            const mixer_reference_row_t *theirs = &cases[c].reference[row];

            if (fabsf(ours[0] - theirs->throttle) > 0.0001f ||
                fabsf(ours[1] - theirs->roll) > 0.0001f ||
                fabsf(ours[2] -
                      theirs->pitch * MIXER_REFERENCE_PITCH_SIGN) > 0.0001f ||
                fabsf(ours[3] - theirs->yaw) > 0.0001f) {
                printf("        %s/%s: ours %.3f %.3f %.3f %.3f, reference "
                       "%.3f %.3f %.3f %.3f\n",
                       cases[c].frame, cases[c].names[row], (double)ours[0],
                       (double)ours[1], (double)ours[2], (double)ours[3],
                       (double)theirs->throttle, (double)theirs->roll,
                       (double)theirs->pitch, (double)theirs->yaw);
                mismatches++;
            }
        }

        snprintf(what, sizeof what,
                 "the %s mixer matches the reference row for row, in its order",
                 cases[c].frame);
        expect(what, mismatches == 0);
    }
}

/*
 * And that every frame in the table is one this firmware could fly: within the
 * output limits, every motor row actually throttled, and no two frames sharing
 * a name (the console and the preflight print the name, and two frames called
 * the same thing is a person configuring the wrong aircraft).
 */
static void test_every_frame_is_well_formed(void)
{
    int bad_counts = 0;
    int bad_throttle = 0;
    int bad_names = 0;

    for (uint32_t n = 0; n < AK_MIXER_AIRFRAMES; n++) {
        const ak_mixer_t *m = ak_mixer_for_airframe(n);
        unsigned motors = 0u;
        unsigned servos = 0u;

        for (uint8_t row = 0u; row < m->count; row++) {
            if (m->kind[row] == AK_OUT_SERVO) {
                servos++;
            } else {
                motors++;
                if (m->coeff[row][0] != 1.0f) {
                    bad_throttle++;
                }
            }
        }

        if (m->count > AK_MAX_MOTORS + AK_MAX_SERVOS || motors == 0u ||
            motors > AK_MAX_MOTORS || servos > AK_MAX_SERVOS) {
            bad_counts++;
        }
        if (m->name == 0 || m->name[0] == '\0') {
            bad_names++;
        }
        for (uint32_t other = n + 1u; other < AK_MIXER_AIRFRAMES; other++) {
            const ak_mixer_t *o = ak_mixer_for_airframe(other);

            if (m->name != 0 && o->name != 0 && strcmp(m->name, o->name) == 0) {
                bad_names++;
            }
        }
    }

    expect("every airframe fits this firmware's output limits", bad_counts == 0);
    expect("and every motor row is a throttled one", bad_throttle == 0);
    expect("and no two airframes share a name", bad_names == 0);
}

/*
 * The 1234 frame is the *same aircraft* as quad-X with the motors numbered
 * differently - that is what it is for - so the same command has to move the
 * same physical corner. A table with the right numbers in the wrong order
 * passes every check above; it does not pass this one.
 */
static void test_the_1234_frame_is_the_same_aircraft(void)
{
    /* quad-X rows are RR, FR, RL, FL; 1234's are FL, FR, RR, RL. */
    static const unsigned corner_of_1234[4] = { 3u, 1u, 0u, 2u };
    int mismatches = 0;

    for (int step = 0; step < 9; step++) {
        float throttle = 0.4f;
        float roll = (float)(step % 3 - 1) * 0.15f;
        float pitch = (float)((step / 3) % 3 - 1) * 0.15f;
        float yaw = (float)((step * 2) % 3 - 1) * 0.15f;
        ak_outputs_t a;
        ak_outputs_t b;

        ak_mixer_apply(&ak_mixer_quad_x, throttle, roll, pitch, yaw, &a);
        ak_mixer_apply(&ak_mixer_quad_x_1234, throttle, roll, pitch, yaw, &b);

        for (int row = 0; row < 4; row++) {
            if (fabsf(a.motor[corner_of_1234[row]] - b.motor[row]) > 0.0001f) {
                mismatches++;
            }
        }
    }

    expect("the 1234 frame is the same aircraft with the motors renumbered",
           mismatches == 0);
}

/*
 * And the tricopter steers with its tail: the reference does that with its
 * servo mixer, this firmware with a row, and a table where the yaw column had
 * leaked onto the motors would still pass every coefficient check above.
 */
static void test_the_tricopter_steers_with_its_tail(void)
{
    ak_outputs_t out;

    ak_mixer_apply(&ak_mixer_tri, 0.5f, 0.0f, 0.0f, 1.0f, &out);
    expect("yaw moves the tricopter's tail servo, and no motor",
           out.servo[0] > 0.1f &&
               fabsf(out.motor[0] - out.motor[1]) < 0.0001f &&
               fabsf(out.motor[0] - out.motor[2]) < 0.0001f);

    ak_mixer_apply(&ak_mixer_tri, 0.5f, 0.0f, 0.0f, -1.0f, &out);
    expect("and the other way for yaw the other way", out.servo[0] < -0.1f);
}

void test_mixer_parity(void)
{
    test_quad_x_table_matches();
    test_quad_x_arithmetic_matches();
    test_the_conventions_are_the_usual_ones();
    test_wing_conventions();
    test_every_frame_matches_its_reference();
    test_every_frame_is_well_formed();
    test_the_1234_frame_is_the_same_aircraft();
    test_the_tricopter_steers_with_its_tail();
}
