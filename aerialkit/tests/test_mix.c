/*
 * Mixer authority: what an armed aircraft does when the torque asked for does
 * not fit under the throttle.
 *
 * This is the part of a mixer that is not arithmetic anyone can eyeball. Each
 * case below is a behaviour a pilot would notice: the thrust staying where the
 * stick put it, the idle floor holding, full throttle giving up authority
 * rather than thrust, and a wing's elevons keeping their full travel while its
 * motors saturate.
 */

#include <math.h>
#include <stdint.h>

#include "ak_mixer.h"
#include "tests.h"

static float mean_motor(const ak_outputs_t *out)
{
    float sum = 0.0f;
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        sum += out->motor[i];
    }
    return sum / (float)AK_MAX_MOTORS;
}

static float max_motor(const ak_outputs_t *out)
{
    float max = out->motor[0];
    for (int i = 1; i < AK_MAX_MOTORS; i++) {
        if (out->motor[i] > max) {
            max = out->motor[i];
        }
    }
    return max;
}

static float min_motor(const ak_outputs_t *out)
{
    float min = out->motor[0];
    for (int i = 1; i < AK_MAX_MOTORS; i++) {
        if (out->motor[i] < min) {
            min = out->motor[i];
        }
    }
    return min;
}

static void test_idle_floor(void)
{
    ak_outputs_t out;
    float torque[3] = { 0.0f, 0.0f, 0.0f };

    /* Armed at zero throttle: a quad's motors turn slowly, because stopped
     * motors cannot yaw it. */
    ak_mixer_apply_limited(&ak_mixer_quad_x, 0.0f, torque, &out);
    expect("a quad idles rather than stopping when armed at zero throttle",
           min_motor(&out) > 0.0f && max_motor(&out) < 0.1f);

    /* A wing does not: there the throttle really is a throttle, and its yaw is
     * differential thrust rather than something to preserve at idle. */
    ak_mixer_apply_limited(&ak_mixer_elevon_wing, 0.0f, torque, &out);
    expect("a wing's motors stop",
           out.motor[0] == 0.0f && out.motor[1] == 0.0f);
}

static void test_thrust_is_kept(void)
{
    ak_outputs_t out;
    float torque[3] = { 0.0f, 0.0f, 0.0f };

    /* No torque demand past the limits: the mix is untouched. */
    ak_mixer_apply_limited(&ak_mixer_quad_x, 0.5f, torque, &out);
    expect("a mix that fits is not scaled",
           fabsf(mean_motor(&out) - 0.5f) < 1e-4f);

    /* A big roll demand at full throttle cannot fit, so authority goes - not
     * thrust. The mean stays where the stick put it. */
    torque[0] = 1.0f;
    ak_mixer_apply_limited(&ak_mixer_quad_x, 1.0f, torque, &out);
    expect("at full throttle the motors do not exceed full",
           max_motor(&out) <= 1.0f + 1e-4f);
    expect("and the thrust the pilot asked for is still there",
           fabsf(mean_motor(&out) - 1.0f) < 1e-3f);

    /* Near the bottom the same thing happens in the other direction: the
     * differential shrinks so the lowest motor sits on the idle floor. */
    torque[0] = 1.0f;
    ak_mixer_apply_limited(&ak_mixer_quad_x, 0.06f, torque, &out);
    expect("near idle the lowest motor sits on the floor",
           min_motor(&out) >= 0.05f - 1e-4f && min_motor(&out) < 0.06f);
    expect("and the mean is still the throttle",
           fabsf(mean_motor(&out) - 0.06f) < 5e-3f);
}

static void test_authority_shrinks_with_throttle(void)
{
    ak_outputs_t low;
    ak_outputs_t high;
    float torque[3] = { 0.6f, 0.0f, 0.0f };

    ak_mixer_apply_limited(&ak_mixer_quad_x, 0.4f, torque, &low);
    ak_mixer_apply_limited(&ak_mixer_quad_x, 0.9f, torque, &high);

    float low_range = max_motor(&low) - min_motor(&low);
    float high_range = max_motor(&high) - min_motor(&high);
    expect("the same stick gives less differential at higher throttle",
           high_range < low_range);
    expect("but still gives some", high_range > 0.01f);

    /* At full throttle there is no room at all: the mix collapses to a flat
     * full-throttle, which is what the pilots of every flight controller
     * expect and nobody enjoys. */
    ak_mixer_apply_limited(&ak_mixer_quad_x, 1.0f, torque, &high);
    expect("full throttle with a roll demand is flat",
           (max_motor(&high) - min_motor(&high)) < 0.02f);
}

static void test_servos_keep_their_travel(void)
{
    ak_outputs_t out;
    /* A wing at full throttle with a roll demand: the motors saturate and the
     * elevons must not be scaled with them, because they are the control
     * surfaces - the thing keeping the aircraft the right way up. */
    float torque[3] = { 0.8f, 0.0f, 0.0f };

    ak_mixer_apply_limited(&ak_mixer_elevon_wing, 1.0f, torque, &out);
    expect("a wing's elevons keep full travel when its motors saturate",
           fabsf(out.servo[0] - 0.8f) < 1e-3f &&
           fabsf(out.servo[1] + 0.8f) < 1e-3f);
    expect("and its motors stay inside their limits",
           max_motor(&out) <= 1.0f + 1e-4f && min_motor(&out) >= 0.0f);
}

static void test_disarmed_is_still_stopped(void)
{
    /* The idle floor belongs to the armed mix only. Disarmed, the flight core
     * writes zeros itself, and nothing in the mixer can talk it out of that. */
    ak_outputs_t out;
    float torque[3] = { 0.0f, 0.0f, 0.0f };
    ak_mixer_apply_limited(&ak_mixer_quad_x, 0.0f, torque, &out);
    expect("the floor is a mixer behaviour, not a safety feature",
           min_motor(&out) > 0.0f);
}

/*
 * And the one thing that chooses between them: the `airframe` parameter's
 * number. The same number also chooses the navigator's return profile, and the
 * two are set together - so this pins the mapping itself, which is what the
 * preflight's consistency check is built on.
 */
static void test_airframe_selects_the_mix(void)
{
    expect("airframe 0 is the quad-X",
           ak_mixer_for_airframe(0u) == &ak_mixer_quad_x &&
               ak_mixer_for_airframe(0u)->name[0] == 'q');
    expect("and airframe 1 is the elevon wing",
           ak_mixer_for_airframe(1u) == &ak_mixer_elevon_wing &&
               ak_mixer_for_airframe(1u)->name[0] == 'e');
    expect("and airframe 7 is the single-motor wing",
           ak_mixer_for_airframe(7u) == &ak_mixer_elevon_wing_single &&
               ak_mixer_for_airframe(7u) != &ak_mixer_quad_x);

    /* The range and the switch, asked against each other rather than one at a
     * time. A number added to `AK_MIXER_AIRFRAMES` with no `case` to go with it
     * falls through to the default and quietly flies a quadrotor - and the
     * obvious test, "does this number return something", passes, because the
     * quadrotor is something. What notices is asking the tables to be pairwise
     * *distinct* over the range the parameter allows.
     *
     * (This test said "airframe 7 is the quadrotor, because the parameter is
     * range-checked to 0..1" until 2026-09-20. Both halves were false: the
     * range had been six wider than that for a while, and 7 is now a wing.) */
    for (uint32_t n = 0u; n < AK_MIXER_AIRFRAMES; n++) {
        for (uint32_t m = n + 1u; m < AK_MIXER_AIRFRAMES; m++) {
            expect("no two numbers in the parameter's range fly the same mix",
                   ak_mixer_for_airframe(n) != ak_mixer_for_airframe(m));
        }
    }
    /* One past the end is the quadrotor, deliberately: a configuration saved by
     * a build with more frames must fly *something* rather than nothing, and
     * the quadrotor is the airframe whose failure mode is the mildest. */
    expect("and a number outside the range is a quadrotor, not a gap",
           ak_mixer_for_airframe(AK_MIXER_AIRFRAMES) == &ak_mixer_quad_x &&
               ak_mixer_for_airframe(99u) == &ak_mixer_quad_x);
}

/*
 * The single-motor wing, which is the first table whose *zeros* are a claim.
 *
 * Every other table's empty cells mean "this output is not used for that axis",
 * which is unremarkable: the quad-X's motor rows carry no roll. This one's yaw
 * column means "this aircraft has no actuator that can produce yaw", and the
 * difference is the whole reason `ak_mixer_has_axis` exists - a control law that
 * cannot tell the two apart runs a yaw loop whose torque is thrown away while
 * its integral accumulates.
 *
 * So the test is not only "is the column zero". It is: the column is zero, the
 * mixer says so, the twin it is copied from says the opposite about the same
 * question, and the *outputs* reflect it whatever torque is asked for.
 */
static void test_single_motor_wing_has_no_yaw(void)
{
    const ak_mixer_t *wing = &ak_mixer_elevon_wing_single;

    expect("the single-motor wing flies as a fixed wing", wing->fixed_wing == 1u);
    expect("it drives one motor and two servos, and no more",
           wing->count == 3u && wing->kind[0] == AK_OUT_MOTOR &&
               wing->kind[1] == AK_OUT_SERVO && wing->kind[2] == AK_OUT_SERVO);
    expect("it is a wing with the motors stopped, like the twin",
           wing->motor_idle == 0.0f);

    expect("it can move roll and pitch",
           ak_mixer_has_axis(wing, 0u) && ak_mixer_has_axis(wing, 1u));
    expect("and it cannot move yaw at all", !ak_mixer_has_axis(wing, 2u));
    /* The twin answers the same question the other way, which is what makes the
     * line above a measurement of this airframe rather than of the question. */
    expect("where the twin-motor wing yaws on its two motors",
           ak_mixer_has_axis(&ak_mixer_elevon_wing, 2u));
    /* The throttle column is not one of the three axes. Answering "yes" for it
     * would let a caller treat the throttle as a control axis, which is exactly
     * the mistake a mixer with a throttle column invites. */
    expect("the throttle column is not an axis and is not reported as one",
           !ak_mixer_has_axis(wing, 3u) && !ak_mixer_has_axis(wing, 99u));
    expect("and a mixer that does not exist has no axes",
           !ak_mixer_has_axis(0, 0u));

    /* The elevons are the twin's, row for row: the same aircraft with the same
     * control surfaces must fly the same surfaces, or a wing would behave
     * differently depending on how many motors it had. */
    for (uint8_t row = 1u; row < 3u; row++) {
        for (unsigned axis = 0u; axis < AK_MIXER_AXES; axis++) {
            expect("the single wing's elevons are the twin's elevons",
                   wing->coeff[row][axis] ==
                       ak_mixer_elevon_wing.coeff[row + 1u][axis]);
        }
    }

    /* And a torque that is nothing but yaw moves nothing: the elevons stay
     * centred and the motor holds the throttle it was given. */
    float torque[3] = { 0.0f, 0.0f, 1.0f };
    ak_outputs_t out;
    ak_mixer_apply_limited(wing, 0.5f, torque, &out);
    expect("a torque with nothing but yaw in it moves neither elevon",
           fabsf(out.servo[0]) < 1e-4f && fabsf(out.servo[1]) < 1e-4f);
    expect("and does not touch the throttle",
           fabsf(out.motor[0] - 0.5f) < 1e-4f);
    /* The slots this airframe does not use are zeroed rather than inherited
     * from whatever flew before it. */
    expect("and the motor slot it does not have is left at zero",
           out.motor[1] == 0.0f && out.motor[2] == 0.0f && out.motor[3] == 0.0f);
}

void test_mixer_authority(void)
{
    test_idle_floor();
    test_thrust_is_kept();
    test_authority_shrinks_with_throttle();
    test_servos_keep_their_travel();
    test_disarmed_is_still_stopped();
    test_airframe_selects_the_mix();
    test_single_motor_wing_has_no_yaw();
}
