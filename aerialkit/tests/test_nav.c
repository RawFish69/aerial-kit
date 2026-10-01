/*
 * Navigation: the arithmetic that decides which way home is, and what the
 * flight core does when the pilot's link is gone.
 *
 * The course error is the one worth staring at. Wrapping a heading difference
 * wrongly turns a ten degree error into a 350 degree one - the aircraft turns
 * away from home and flies until it runs out of battery - and it only shows up
 * when the two headings are on opposite sides of north. So that boundary is
 * tested on both sides, at 180, and at exactly zero.
 */

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ak_flight.h"
#include "ak_mixer.h"
#include "ak_nav.h"
#include "tests.h"

/* Defined further down, beside the other flight-core fixtures. */
static ak_rc_input_t rc_frame(uint32_t now_ms, int arm);

static ak_imu_sample_t level_imu(uint32_t time_ms)
{
    ak_imu_sample_t imu;
    imu.gyro[0] = 0.0f;
    imu.gyro[1] = 0.0f;
    imu.gyro[2] = 0.0f;
    imu.accel[0] = 0.0f;
    imu.accel[1] = 0.0f;
    imu.accel[2] = 1.0f;
    imu.time_ms = time_ms;
    imu.valid = 1;
    return imu;
}

static void test_distance_and_bearing(void)
{
    int32_t distance = 0;
    int32_t bearing = 0;

    /* A hundredth of a degree of latitude is about 1111 metres, and north is
     * zero by definition. */
    ak_nav_distance_bearing(520000000, 40000000, 520100000, 40000000, &distance,
                            &bearing);
    expect("a hundredth of a degree north is about 1111 m",
           distance > 1105 && distance < 1117);
    expect("and it is due north", bearing > -30 && bearing < 30);

    /* Due east at the equator is ninety. */
    ak_nav_distance_bearing(0, 0, 0, 100000, &distance, &bearing);
    expect("due east is ninety degrees", bearing > 8990 && bearing < 9010);

    /* And due south is 180 (or -180: both mean the same thing, so the test
     * accepts either sign). */
    ak_nav_distance_bearing(520100000, 40000000, 520000000, 40000000, &distance,
                            &bearing);
    expect("due south is half a turn", abs(bearing) > 17990);

    /* Longitude shrinks with latitude: the same tenth of a degree is narrower
     * at 60 north than at the equator, by about half. */
    int32_t at_equator = 0;
    int32_t at_sixty = 0;
    ak_nav_distance_bearing(0, 0, 0, 1000000, &at_equator, 0);
    ak_nav_distance_bearing(600000000, 0, 600000000, 1000000, &at_sixty, 0);
    expect("a degree of longitude is shorter away from the equator",
           at_sixty < at_equator / 2 + 100 && at_sixty > at_equator / 2 - 100);
}

static void test_course_error(void)
{
    /* Wanting north while flying east is a ninety degree turn to the left. */
    expect("a quarter turn shows as ninety degrees",
           fabsf(ak_nav_course_error_e2(0, 9000000) + 90.0f) < 0.01f);

    /* The wrap: 350 wanted, 10 flying, is twenty degrees left, not 340 right. */
    expect("across north, the short way round is taken",
           fabsf(ak_nav_course_error_e2(35000, 1000000) + 20.0f) < 0.01f);
    expect("and the other way round",
           fabsf(ak_nav_course_error_e2(1000, 35000000) - 20.0f) < 0.01f);

    /* Exactly opposite: either turn is the same, and it must not be reported as
     * zero or as 360. */
    float opposite = ak_nav_course_error_e2(0, 18000000);
    expect("dead opposite is a half turn, not a full one",
           fabsf(opposite) > 179.9f && fabsf(opposite) <= 180.1f);

    /* 123.45 degrees, in the two units the two sides use: hundredths for a
     * bearing this firmware computed, hundred-thousandths for a course a
     * receiver reported. */
    expect("flying the right way is no error at all",
           fabsf(ak_nav_course_error_e2(12345, 12345000)) < 0.01f);
}

/*
 * The wing profile's input, from the four numbers its tests care about. The
 * yaw fields stay zero: the wing steers on the GPS course and never looks at
 * them - the quadrotor's profile is the one that needs a heading, and it has
 * its own tests below.
 */
static int nav_step(ak_nav_t *nav, int32_t lat_e7, int32_t lon_e7,
                    int32_t alt_mm, int32_t course_e5, ak_rc_command_t *out)
{
    ak_nav_input_t in;
    memset(&in, 0, sizeof in);

    in.lat_e7 = lat_e7;
    in.lon_e7 = lon_e7;
    in.alt_mm = alt_mm;
    in.gps_valid = 1; /* these are navigation steps: they have a fix */
    in.course_e5 = course_e5;
    in.max_tilt_rad = 0.6109f; /* 35 degrees, the flight core's own default */

    return ak_nav_step(nav, &in, out);
}

static void test_guidance(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;

    ak_nav_init(&nav);
    expect("there is nothing to return to before a home is set",
           ak_nav_engage(&nav, 100000) == 0);

    /* Home is 1 km north of the aircraft. */
    ak_nav_set_home(&nav, 521000000, 40000000, 0);
    expect("engaging captures the altitude to hold",
           ak_nav_engage(&nav, 120000) == 1 && nav.active == 1 &&
           nav.hold_alt_mm == 120000);

    /* Flying east: home is to the left, so the command is a left roll. */
    expect("a wing heading away from home is steered toward it",
           nav_step(&nav, 520000000, 40000000, 120000, 9000000, &out) == 1);
    expect("which is a left roll", out.roll < -0.5f);
    expect("at cruise throttle", out.throttle > 0.5f && out.throttle < 0.6f);
    expect("with no rudder, because a wing steers with its elevons",
           fabsf(out.yaw) < 0.001f);

    /* Below the altitude being held: pitch up. Above it: pitch down. The
     * magnitudes are pinned too, because the sign is what the old version of
     * this got right while being a thousand times too strong - the altitude
     * error is handed over in millimetres and the gain is per metre, and only
     * the scale tells the two apart. */
    (void)nav_step(&nav, 520000000, 40000000, 100000, 0, &out);
    expect("below the hold altitude climbs", out.pitch > 0.0f);
    expect("twenty metres below the hold is a fifth of a pitch command",
           fabsf(out.pitch - 0.20f) < 0.001f);
    (void)nav_step(&nav, 520000000, 40000000, 140000, 0, &out);
    expect("above it descends", out.pitch < 0.0f);
    expect("and twenty metres above is minus a fifth",
           fabsf(out.pitch + 0.20f) < 0.001f);

    /* A metre off must not saturate anything: that is the bug above, in one
     * line. */
    (void)nav_step(&nav, 520000000, 40000000, 119000, 0, &out);
    expect("a metre of altitude error is a hundredth of a pitch command",
           fabsf(out.pitch - 0.01f) < 0.001f);

    /* Arrived: stop steering at a point and circle instead. */
    ak_nav_set_home(&nav, 520000000, 40000000, 0);
    (void)nav_step(&nav, 520000001, 40000000, 120000, 0, &out);
    expect("arriving turns the return into a circle",
           fabsf(out.roll - nav.loiter_roll) < 0.001f);

    ak_nav_disengage(&nav);
    expect("a disengaged navigator says nothing",
           nav_step(&nav, 520000000, 40000000, 120000, 0, &out) == 0);
}

/*
 * The quadrotor's return, flown.
 *
 * The wing's profile can be tested one step at a time, because a roll command
 * means the same thing whatever the aircraft is doing. A quadrotor's cannot be:
 * the profile commands an *acceleration*, so whether a tilt is the right tilt
 * depends on two things no single step shows - which way the airframe is
 * pointing, and how fast it is already travelling. So this flies a point mass
 * through the profile: the same code the firmware runs, with the simplest
 * physics that can disagree with it, and the checks are what the aircraft did
 * rather than what one command said.
 */
static const float NAV_TEST_DT = 0.02f;      /* 50 Hz */
static const float NAV_TEST_G = 9.80665f;
#define NAV_TEST_HOVER_THROTTLE 0.55f        /* the profile's cruise default */
#define NAV_TEST_THROTTLE_ACCEL 20.0f        /* m/s^2 per unit of throttle */
#define NAV_TEST_M_PER_DEG 111320.0f

typedef struct {
    float north_m, east_m;   /* where it is, from home */
    float alt_m;
    float vn, ve;            /* how fast, in the world frame */
    float climb_m_s;
    float yaw_rad;
    int   yaw_aligned;
} nav_test_quad_t;

static void nav_test_input(const nav_test_quad_t *q, ak_nav_input_t *in)
{
    float speed = sqrtf(q->vn * q->vn + q->ve * q->ve);

    memset(in, 0, sizeof *in);
    in->lat_e7 = 521000000 + (int32_t)(q->north_m / NAV_TEST_M_PER_DEG * 1e7f);
    in->lon_e7 = 40000000 + (int32_t)(q->east_m / (NAV_TEST_M_PER_DEG * 0.6f) *
                                      1e7f);
    in->alt_mm = 20000 + (int32_t)(q->alt_m * 1000.0f);
    in->gps_valid = 1; /* the return below is flown with a fix */
    in->speed_mm_s = (int32_t)(speed * 1000.0f);
    in->course_e5 = (int32_t)(atan2f(q->ve, q->vn) * 57.29578f * 100000.0f);
    in->yaw_mrad = (int32_t)(q->yaw_rad * 1000.0f);
    in->yaw_aligned = q->yaw_aligned;
    in->max_tilt_rad = 0.6109f; /* 35 degrees */
    in->dt_s = NAV_TEST_DT;
}

/* One step of the simplest quad that can be flown by a tilt: the commanded
 * angle is reached, and it becomes an acceleration. */
static void nav_test_advance(nav_test_quad_t *q, const ak_rc_command_t *out)
{
    float a_forward = -out->pitch * 0.6109f * NAV_TEST_G;
    float a_right = out->roll * 0.6109f * NAV_TEST_G;
    float an = a_forward * cosf(q->yaw_rad) - a_right * sinf(q->yaw_rad);
    float ae = a_forward * sinf(q->yaw_rad) + a_right * cosf(q->yaw_rad);

    q->vn += an * NAV_TEST_DT;
    q->ve += ae * NAV_TEST_DT;
    q->north_m += q->vn * NAV_TEST_DT;
    q->east_m += q->ve * NAV_TEST_DT;

    q->climb_m_s += (out->throttle - NAV_TEST_HOVER_THROTTLE) *
                    NAV_TEST_THROTTLE_ACCEL * NAV_TEST_DT;
    q->alt_m += q->climb_m_s * NAV_TEST_DT;
}

static void test_quad_return(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;
    ak_nav_input_t in;
    nav_test_quad_t quad;
    float start_m = 300.0f;
    float closest_m = 1.0e9f;
    float far_m = 0.0f;
    int moved_early = 0;
    int arrived_at = 0;
    int steps = 0;
    char name[96];
    float alt_min_m = 1.0e9f;
    float alt_max_m = -1.0e9f;

    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000); /* home is the ground */
    expect("the quadrotor's return engages like the wing's",
           ak_nav_engage(&nav, 70000) == 1 && nav.hold_alt_mm == 70000);

    /* 300 m north of home, pointing east, 50 m below the altitude it was
     * asked to hold, standing still. */
    quad.north_m = start_m;
    quad.east_m = 0.0f;
    quad.alt_m = 20.0f;
    quad.vn = quad.ve = quad.climb_m_s = 0.0f;
    quad.yaw_rad = 1.57079633f;
    quad.yaw_aligned = 1;

    nav_test_input(&quad, &in);
    (void)ak_nav_step(&nav, &in, &out);
    expect("a quad well below the return altitude climbs before it steers",
           out.throttle > 0.6f && fabsf(out.roll) < 0.01f &&
               fabsf(out.pitch) < 0.01f);

    for (steps = 0; steps < 6000; steps++) { /* 120 s at 50 Hz */
        float distance;

        nav_test_input(&quad, &in);
        if (ak_nav_step(&nav, &in, &out) == 0) {
            break;
        }
        nav_test_advance(&quad, &out);

        distance = sqrtf(quad.north_m * quad.north_m +
                         quad.east_m * quad.east_m);
        if (distance < closest_m) {
            closest_m = distance;
        }
        if (distance > far_m && distance < start_m + 200.0f) {
            far_m = distance; /* it never wanders off before coming back */
        }
        /* The first time it moves, it should already be at the altitude it was
         * asked to hold: climbing first is the safe part and the reason the
         * profile does it that way. */
        if (!moved_early && distance < start_m - 2.0f) {
            moved_early = 1;
            expect("and it is at the return altitude when it starts to move",
                   quad.alt_m > 47.0f);
        }
        if (arrived_at == 0 && distance < 60.0f) {
            arrived_at = steps;
        }
        /* The last twenty seconds, for what "holding" means: a band rather
         * than one sample, because a controller that is converging and one
         * that is oscillating both pass a single reading. */
        if (steps > 5000) {
            if (quad.alt_m < alt_min_m) {
                alt_min_m = quad.alt_m;
            }
            if (quad.alt_m > alt_max_m) {
                alt_max_m = quad.alt_m;
            }
        }
    }

    snprintf(name, sizeof name,
             "the quadrotor comes home: %.0f m of 300, hovering %.1f m up",
             (double)closest_m, (double)quad.alt_m);
    expect(name, closest_m < 60.0f);
    expect("it arrives without wandering off first", far_m < 380.0f);
    snprintf(name, sizeof name,
             "and then it descends to its hover height: %.1f to %.1f m above "
             "home, asked for %.1f",
             (double)alt_min_m, (double)alt_max_m,
             (double)nav.quad_hover_m);
    expect(name, alt_min_m > nav.quad_hover_m - 3.0f &&
                     alt_max_m < nav.quad_hover_m + 3.0f);
    expect("holding there instead of drifting away",
           fabsf(quad.north_m) < 30.0f && fabsf(quad.east_m) < 30.0f &&
               fabsf(quad.vn) < 2.0f && fabsf(quad.ve) < 2.0f);

    /*
     * And without a heading worth using it must not *steer*: the safety
     * property - a world-frame error turned into a body-frame tilt with a yaw
     * that is 90 degrees out flies the aircraft confidently in the wrong
     * direction - and the reason the estimator has to earn one.
     *
     * What it does instead is make a track to earn one from: straight ahead,
     * no roll and no yaw command, until the module's course means something.
     * That is a change of behaviour and it is deliberate - the version of this
     * that held still instead could never start at all in still air, because
     * the only heading measurement on the aircraft needs motion to exist (see
     * test_alignment_nudge below).
     */
    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    ak_nav_engage(&nav, 70000);
    quad.north_m = start_m;
    quad.east_m = 0.0f;
    quad.alt_m = 20.0f;
    quad.vn = quad.ve = quad.climb_m_s = 0.0f;
    quad.yaw_rad = 1.57079633f;
    quad.yaw_aligned = 0;
    float max_roll_seen = 0.0f;
    for (steps = 0; steps < 2000; steps++) {
        nav_test_input(&quad, &in);
        (void)ak_nav_step(&nav, &in, &out);
        if (fabsf(out.roll) > max_roll_seen) {
            max_roll_seen = fabsf(out.roll);
        }
        nav_test_advance(&quad, &out);
    }
    expect("with no aligned heading it never steers, and earns one straight "
           "ahead instead",
           max_roll_seen < 0.01f && fabsf(quad.north_m - start_m) < 2.0f &&
               quad.east_m > 1.0f && quad.alt_m > 48.0f);
}

/*
 * The quadrotor's descent rate, on a barometer that reports the way the real
 * one does.
 *
 * The vertical loop is a rate loop: the throttle follows the difference
 * between the climb rate the profile asks for and the one the aircraft has.
 * That makes the *rate* a control input, and the obvious way to measure it -
 * the height this pass minus the height last pass - is wrong on a barometer
 * that arrives at 32 Hz in whole millimetres: between two passes of a 1 kHz
 * loop the height is unchanged six passes out of seven and then jumps by
 * 33 mm, which reads as "not descending" for six passes and "descending at 33
 * metres a second" for one. With the throttle railed down on the quiet passes
 * and a single millisecond of correction on the loud one, the loop flew a
 * return that dived: measured in the simulator, it believed it was not
 * descending while the aircraft came down at 9.9 metres a second, reached the
 * ground from its hover height, and climbed back.
 *
 * So this flies the last part of a return against a barometer that is
 * quantised exactly that way and checks what the aircraft did, not what the
 * loop meant: a descent that is near the rate it was asked for, and a hover
 * height it stops at rather than falls through.
 */
static void test_quad_descent_rate(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;
    ak_nav_input_t in;
    const float dt = 0.001f;          /* the flight loop's 1 kHz */
    const float baro_step_m = 0.0332f; /* what one pressure count is worth */
    const float hover_m = 2.5f;
    float alt_m = 23.0f;              /* above home, the return altitude */
    float vz = 0.0f;
    float reported_m = 23.0f;
    float min_alt_m = 1.0e9f;
    float fastest_m_s = 0.0f;
    int   baro_age_ms = 0;
    int   descending_seen = 0;

    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    nav.quad_hover_m = hover_m;
    ak_nav_engage(&nav, 20000 + (int32_t)(alt_m * 1000.0f));

    for (int i = 0; i < 40000; i++) { /* forty seconds */
        float accel;

        /* The barometer: 32 Hz, and a value that can only be a whole number
         * of its own pressure counts. */
        if (++baro_age_ms >= 31) {
            baro_age_ms = 0;
            reported_m = floorf(alt_m / baro_step_m + 0.5f) * baro_step_m;
        }

        memset(&in, 0, sizeof in);
        /* Over home: the return's own arrival has happened, so the profile is
         * flying the descent rather than the translation. */
        in.lat_e7 = 521000000;
        in.lon_e7 = 40000000;
        in.alt_mm = 20000 + (int32_t)(reported_m * 1000.0f);
        in.gps_valid = 1; /* a descent, not a hold: the fix is there */
        in.speed_mm_s = 0;
        in.course_e5 = 0;
        in.yaw_mrad = 0;
        in.yaw_aligned = 1;
        in.max_tilt_rad = 0.6109f;
        in.dt_s = dt;

        if (ak_nav_step(&nav, &in, &out) == 0) {
            break;
        }

        /* The vertical part of the simulator's quad: the collective against
         * gravity, with drag. The point of the test is the loop, so the plant
         * is the simplest one that can disagree with it. */
        accel = 9.81f * (out.throttle / 0.55f - 1.0f) - 0.5f * vz;
        vz += accel * dt;
        alt_m += vz * dt;
        if (alt_m <= 0.0f) {
            alt_m = 0.0f;
            if (vz < 0.0f) {
                vz = 0.0f;
            }
        }
        if (nav.descending) {
            descending_seen = 1;
        }
        if (descending_seen) {
            if (vz < fastest_m_s) {
                fastest_m_s = vz;
            }
            /* Only until the landing begins. The profile is *supposed* to
             * leave the hover height and go to the ground once it has settled
             * there; what this measures is whether it reaches the hover at
             * all, or dives through it on the way down. */
            if (!nav.landing && alt_m < min_alt_m) {
                min_alt_m = alt_m;
            }
        }
    }

    expect("the descent happens at all", descending_seen && min_alt_m < 4.0f);
    {
        char name[128];

        snprintf(name, sizeof name,
                 "and it is the descent it was asked for: %.1f m/s at its "
                 "fastest, against a cap of 3.0",
                 (double)-fastest_m_s);
        expect(name, fastest_m_s > -4.0f);
        snprintf(name, sizeof name,
                 "and it stops at the hover height instead of going through "
                 "it: %.2f m above home, asked for %.1f",
                 (double)min_alt_m, (double)hover_m);
        expect(name, min_alt_m > hover_m - 1.0f);
    }
}

/*
 * Flying with no position fix.
 *
 * The link is gone, the navigator is flying, and the module that says where
 * the aircraft is stops answering. Giving up hands the flight core a lost link
 * with nobody flying, and the correct thing for a lost link is to stop the
 * motors - which in the air is not a state to hand an aircraft to. So the
 * claim is a narrow one: with no fix, what comes out of the navigator depends
 * on the gyro and the barometer, and on nothing else. The position in the
 * input is stale, and the check that proves it is ignored is that the command
 * is the same as it would be from anywhere.
 */
/*
 * The wing's standing pitch, and the altitude it was costing.
 *
 * A wing sinks, so holding an altitude takes a standing nose-up - and a
 * proportional law can only get that from the altitude *error*, which means it
 * holds an altitude below the one it was given. Measured in the simulator: the
 * return held 14 m where it was told to hold 15, and a fence that asked it to
 * climb to a 30 m floor settled at 26. This is that in one axis, against a
 * plant with a sink and a climb per unit of pitch - and it is an A/B, because
 * a claim about an integral that is not measured against the loop without one
 * is a claim about nothing.
 */
static float wing_altitude_run(int with_integral)
{
    ak_nav_t nav;
    ak_nav_input_t in;
    ak_rc_command_t out;
    const float dt = NAV_TEST_DT;
    const float climb_per_pitch = 6.0f; /* m/s of climb per unit of pitch */
    const float sink_m_s = 0.3f;
    float deficit_m = 0.0f; /* metres below the altitude it was told to hold */

    ak_nav_init(&nav);
    if (!with_integral) {
        nav.alt_ki = 0.0f;
    }
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_WING);
    ak_nav_set_home(&nav, 521000000, 40000000, 0);
    (void)ak_nav_engage(&nav, 100000); /* hold 100 m */

    memset(&in, 0, sizeof in);
    in.lat_e7 = 521000000 + 1000000; /* a kilometre north: still returning */
    in.lon_e7 = 40000000;
    in.gps_valid = 1;
    in.course_e5 = 18000000; /* flying south, home is north */
    in.max_tilt_rad = 0.6109f;
    in.dt_s = dt;

    for (int i = 0; i < 6000; i++) { /* 120 s at 50 Hz */
        float climb_m_s;

        in.alt_mm = 100000 - (int32_t)(deficit_m * 1000.0f);
        (void)ak_nav_step(&nav, &in, &out);
        climb_m_s = climb_per_pitch * out.pitch - sink_m_s;
        deficit_m -= climb_m_s * dt;
    }
    return deficit_m;
}

static void test_wing_altitude_droop(void)
{
    float with = wing_altitude_run(1);
    float without = wing_altitude_run(0);

    printf("        altitude deficit: with the integral %.2f m, without %.2f m\n",
           (double)with, (double)without);

    /* The proportional law settles where 0.01 of pitch per metre of error
     * supplies the 0.05 of pitch it takes to stop the sink: five metres. */
    expect("a proportional altitude loop holds below the altitude it was given",
           without > 4.0f && without < 6.0f);
    expect("and the standing pitch it learns removes that",
           fabsf(with) < 0.5f);
    expect("which is what makes it an integral and not a longer gain",
           with < without - 3.0f);
}

/*
 * Earning a heading.
 *
 * There is no compass on this aircraft, and the only measurement of which way
 * the nose points is the track it makes through the air - so a quadrotor that
 * has to fly somewhere, and is not moving, does not know which way it is
 * facing and cannot start. That is not a corner case: it is every mission
 * started from a hover in still air, and the simulator says so - the aircraft
 * sat at 221 m from its first waypoint for two minutes with the yaw estimate
 * frozen 105 degrees from the nose.
 *
 * So the navigator makes a track to learn from: a shove straight ahead that
 * steers nothing (there is no heading to steer with yet), stops once the
 * module's course means something, and gives up rather than flying on if it
 * never does.
 */
static void test_alignment_nudge(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;
    ak_nav_input_t in;

    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);

    memset(&in, 0, sizeof in);
    in.lat_e7 = 521000000 + 2000000; /* 222 m north of home */
    in.lon_e7 = 40000000;
    in.alt_mm = 40000;               /* at the altitude it was told to hold */
    in.gps_valid = 1;
    in.yaw_aligned = 0;              /* it does not know which way it points */
    in.speed_mm_s = 0;               /* because it is not moving */
    in.max_tilt_rad = 0.6109f;
    in.dt_s = NAV_TEST_DT;

    (void)ak_nav_step(&nav, &in, &out);
    expect("an aircraft that does not know which way it points shoves forward",
           out.pitch < -0.2f && fabsf(out.roll) < 0.01f);
    expect("and does not steer with a heading it has not got",
           fabsf(out.yaw) < 0.001f);

    /* Moving: the shove stops, because the module now has a course for the
     * estimator to align to, and steering before that is steering blind. */
    in.speed_mm_s = 4000;
    (void)ak_nav_step(&nav, &in, &out);
    expect("and it stops shoving once it has a track to learn from",
           fabsf(out.pitch) < 0.01f && fabsf(out.roll) < 0.01f);

    /* It climbs before it shoves: a return that earns a heading with the
     * ground close is a return that earns it into the ground. */
    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);
    in.speed_mm_s = 0;
    in.alt_mm = 30000; /* ten metres below the altitude it is holding */
    (void)ak_nav_step(&nav, &in, &out);
    expect("and it climbs first, before any of this",
           fabsf(out.pitch) < 0.01f && out.throttle > NAV_TEST_HOVER_THROTTLE);

    /* And the cap: an aircraft that never earns one holds rather than flying
     * on into whatever is in front of it. */
    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);
    in.alt_mm = 40000;
    in.speed_mm_s = 0;
    for (int i = 0; i < 700; i++) { /* fourteen seconds at 50 Hz */
        (void)ak_nav_step(&nav, &in, &out);
    }
    expect("it gives up rather than flying away when it cannot earn a heading",
           nav.nudge_gave_up == 1 && fabsf(out.pitch) < 0.01f);
}

/*
 * The vertical loop, and the rate it was asked for.
 *
 * The quadrotor's height is flown by a *rate* loop: the profile decides how
 * fast the aircraft should be climbing and the throttle follows the difference
 * between that and the rate the height estimate says it has. A proportional
 * loop of that shape settles where the plant needs the throttle the error
 * happens to supply, which is not where it was asked to be - and how far short
 * is a property of the *plant*, which is why this is measured against one
 * rather than asserted about the gain.
 *
 * The plant below is the simulator's quad in one axis: a throttle offset from
 * the hover throttle is an acceleration, and the rate it reaches is limited by
 * drag. A standing 0.026 of throttle buys about 0.27 m/s of descent - which is
 * the number the recorded gps-loss landing shows, 72.9 seconds to come down
 * twenty metres while the profile was asking for 0.6 m/s all the way.
 *
 * So the test is an A/B: the same descent flown twice, with the integral and
 * without it, and the check is that the aircraft flies the rate it was told to
 * *and* that the old law does not.
 */
static float vertical_rate_run(int with_integral, float *final_alt_m,
                               ak_nav_t *nav_state)
{
    ak_nav_t nav;
    ak_nav_input_t in;
    ak_rc_command_t out;
    const float dt = NAV_TEST_DT;
    const float drag_per_s = 2.0f;
    float alt_m = 20.0f;      /* above home */
    float climb_m_s = 0.0f;
    float from_alt = 0.0f;
    const float from_s = 12.0f;
    int   have_from = 0;
    float rate = 0.0f;
    int   steps = (int)(24.0f / dt);

    ak_nav_init(&nav);
    nav.quad_alt_ki = with_integral ? 0.05f : 0.0f;
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);
    nav.hold_land_s = 1u; /* come down where it is, one second from now */

    memset(&in, 0, sizeof in);
    in.lat_e7 = 521000000;
    in.lon_e7 = 40000000;
    in.gps_valid = 0;
    in.range_fitted = 1;
    in.max_tilt_rad = 0.6109f;
    in.dt_s = dt;

    for (int i = 0; i < steps; i++) {
        float t = (float)i * dt;

        in.alt_mm = 20000 + (int32_t)(alt_m * 1000.0f);
        (void)ak_nav_step(&nav, &in, &out);
        climb_m_s += ((out.throttle - NAV_TEST_HOVER_THROTTLE) *
                          NAV_TEST_THROTTLE_ACCEL -
                      drag_per_s * climb_m_s) *
                     dt;
        alt_m += climb_m_s * dt;

        /* Measured over ten seconds once the descent is established, so the
         * number is the rate it flies rather than the transient it started
         * with. */
        if (t >= from_s && t <= 22.0f) {
            if (!have_from) {
                from_alt = alt_m;
                have_from = 1;
            } else {
                rate = (alt_m - from_alt) / (t - from_s);
            }
        }
    }

    if (final_alt_m != 0) {
        *final_alt_m = alt_m;
    }
    if (nav_state != 0) {
        *nav_state = nav;
    }
    return rate;
}

static void test_vertical_rate_loop(void)
{
    float with;
    float without;
    float final_m = 0.0f;

    /* The commanded descent is AK_QUAD_LAND_CLIMB, 0.6 m/s: a descent in place
     * is capped there for the whole of it. */
    with = vertical_rate_run(1, &final_m, 0);
    without = vertical_rate_run(0, 0, 0);

    printf("        descent rate: with the integral %.2f m/s, without %.2f, "
           "commanded 0.60\n", (double)-with, (double)-without);

    expect("the descent flies the rate the profile asked for",
           with < -0.45f && with > -0.75f);
    expect("and a proportional loop alone does not, which is why it is there",
           without > -0.45f);
    expect("so the integral is what makes the two numbers differ",
           with < without - 0.15f);
    expect("and it is still flying rather than through the floor",
           final_m > 1.0f && final_m < 12.0f);
}

/*
 * And the bound on what it may learn: a standing throttle worth no more than a
 * one-metre-a-second rate error, and therefore *nothing* at all while the
 * profile is asking for a hover.
 *
 * Both halves matter. An integral that keeps growing while the aircraft cannot
 * do what it was asked hands back a throttle nobody asked for the moment the
 * aircraft can; an integral that is allowed to sit on a *fixed* bound does the
 * same thing more slowly - which is how the first version of this carried an
 * aircraft through its hover height and into the ground.
 */
static void test_vertical_integral_is_bounded(void)
{
    ak_nav_t nav;
    ak_nav_input_t in;
    ak_rc_command_t out;
    float worst = 0.0f;
    int   pinned = 0;

    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);
    nav.hold_land_s = 1u;

    memset(&in, 0, sizeof in);
    in.lat_e7 = 521000000;
    in.lon_e7 = 40000000;
    in.gps_valid = 0;
    in.range_fitted = 1;
    in.max_tilt_rad = 0.6109f;
    in.dt_s = NAV_TEST_DT;
    /* The aircraft never moves: the height it is asked about is the height it
     * keeps, while the profile asks it down. */
    in.alt_mm = 40000;

    for (int i = 0; i < 2000; i++) {
        (void)ak_nav_step(&nav, &in, &out);
        if (fabsf(nav.climb_i) > worst) {
            worst = fabsf(nav.climb_i);
        }
        if (out.throttle <= 0.0f || out.throttle >= 1.0f) {
            pinned = 1;
        }
    }

    /* 0.6 m/s of commanded descent: the bound is 0.12 of throttle per m/s of
     * it, which is 0.072. */
    expect("the standing throttle is bounded by what the rate is worth",
           worst <= 0.0721f);
    expect("and the throttle it asks for is always a throttle", !pinned);
    expect("and the aircraft is asked to come down rather than to hang",
           nav.hold_landing == 1 && out.throttle < NAV_TEST_HOVER_THROTTLE);
    expect("with the bound reached, not exceeded",
           fabsf(nav.climb_i) > 0.03f);

    /*
     * And now the profile is asking for a hover: the aircraft is at the height
     * the return settles at, over home, with a fix. There is no rate to learn,
     * so the standing throttle it learned coming down has to go - and the
     * aircraft holds the hover on the proportional term alone, which is what
     * every flight before this one did.
     */
    in.gps_valid = 1;
    in.lat_e7 = 521000000;
    in.lon_e7 = 40000000;
    in.alt_mm = 20000 + (int32_t)(nav.quad_hover_m * 1000.0f);
    for (int i = 0; i < 75; i++) {   /* a second and a half: still hovering */
        (void)ak_nav_step(&nav, &in, &out);
    }
    expect("and the return is in the hover phase it was asked about",
           nav.descending == 1 && nav.landing == 0);
    expect("a hover has nothing to learn, so the standing throttle goes",
           fabsf(nav.climb_i) < 0.002f);
}

/*
 * The ground below.
 *
 * Everything the navigator has been able to measure until now is an estimate
 * of a *change*: a barometer anchored once on the ground, or a GPS altitude
 * that wanders by metres. The last metre of a descent is the one place where
 * that is not good enough, because the number being flown to and the error in
 * the instrument are the same size - and a navigator that believes it has
 * arrived while it is still a metre up either hovers there until the pack is
 * flat or stops its motors in the air.
 *
 * A rangefinder measures the distance directly, so these are the two
 * behaviours it buys, in the units the navigator works in: the descent keeps
 * going while the *part* says there is ground left below, and a quadrotor that
 * has lost its position can choose to come down where it is - which it may
 * only do with something measuring the ground, because the alternative is a
 * descent flown on the instrument that has already failed.
 */
static void test_rangefinder_descent(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;
    ak_nav_input_t in;
    nav_test_quad_t quad;
    float with_range;
    float without_range;

    /* --- the descent the estimate cannot finish --------------------------- */

    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000); /* the ground, in MSL */
    expect("the quadrotor's return engages", ak_nav_engage(&nav, 40000) == 1);

    quad.north_m = 0.0f;
    quad.east_m = 0.0f;
    quad.alt_m = 20.0f;
    quad.vn = quad.ve = quad.climb_m_s = 0.0f;
    quad.yaw_rad = 0.0f;
    quad.yaw_aligned = 1;

    /* Fly the return down to the hover height and let it settle there, which
     * is what puts the profile into its landing phase. */
    for (int i = 0; i < 4000 && !nav.landing; i++) {
        nav_test_input(&quad, &in);
        if (ak_nav_step(&nav, &in, &out) == 0) {
            break;
        }
        nav_test_advance(&quad, &out);
    }
    expect("and reaches the landing phase over home",
           nav.landing == 1 && nav.descending == 1);

    /*
     * Now the disagreement that matters, with the estimate held at the ground
     * and the part reading 1.2 m. The altitude is fed in unchanged for a
     * second first, so the climb-rate window has settled and the two
     * commands below differ by the *height* and nothing else.
     */
    memset(&in, 0, sizeof in);
    in.lat_e7 = 521000000;
    in.lon_e7 = 40000000;
    in.alt_mm = 20000; /* the estimate says: level with the ground */
    in.gps_valid = 1;
    in.yaw_aligned = 1;
    in.max_tilt_rad = 0.6109f;
    in.dt_s = NAV_TEST_DT;
    for (int i = 0; i < 50; i++) {
        (void)ak_nav_step(&nav, &in, &out);
    }
    without_range = out.throttle;
    expect("the estimate alone stops asking it down at the ground",
           without_range > NAV_TEST_HOVER_THROTTLE - 0.02f);

    in.agl_valid = 1;
    in.agl_mm = 1200;
    (void)ak_nav_step(&nav, &in, &out);
    with_range = out.throttle;
    expect("but a part saying there is still 1.2 m to go keeps it coming down",
           with_range < without_range - 0.02f);
    /* And it is the *landing* descent rather than the return's dive: the
     * throttle is below a hover, and well above what asking for the three
     * metres a second of the return would take (with this plant that is about
     * 0.23, against 0.55 of a hover). The altitude in this test never moves,
     * so the loop is pushing as hard as it is allowed to. */
    expect("and it is the descent it was already allowed to fly, not a dive",
           with_range < NAV_TEST_HOVER_THROTTLE - 0.01f &&
               with_range > NAV_TEST_HOVER_THROTTLE - 0.15f);

    /* The same reading with the landing not begun is not the navigator's
     * business: a return that is still flying home stays where it was told
     * to. */
    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);
    in.agl_valid = 1;
    in.agl_mm = 1200;
    (void)ak_nav_step(&nav, &in, &out);
    expect("a part reading the ground does not change a return that is still "
           "climbing or flying home",
           out.throttle > NAV_TEST_HOVER_THROTTLE);

    /* --- and coming down with nowhere to aim ------------------------------ */

    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);
    nav.hold_land_s = 5u;

    memset(&in, 0, sizeof in);
    in.lat_e7 = 521000000;
    in.lon_e7 = 40000000;
    in.alt_mm = 40000; /* holding the altitude it engaged at */
    in.gps_valid = 0;  /* and it has lost the fix */
    in.range_fitted = 1;
    in.max_tilt_rad = 0.6109f;
    in.dt_s = NAV_TEST_DT;

    /* Four seconds of holding: not yet. */
    for (int i = 0; i < 200; i++) {
        (void)ak_nav_step(&nav, &in, &out);
    }
    expect("a quadrotor holding with a part fitted waits for the pilot's "
           "delay", nav.hold_landing == 0 && out.throttle > 0.5f);

    for (int i = 0; i < 100; i++) {
        (void)ak_nav_step(&nav, &in, &out);
    }
    expect("and then comes down where it is",
           nav.hold_landing == 1 && out.throttle < NAV_TEST_HOVER_THROTTLE);
    /* The landing's own rate and not the return's dive, with the same two
     * bounds as the descent above: below a hover, and well above the three
     * metres a second the return would ask for. */
    expect("at the speed a landing is flown at, not the return's",
           out.throttle > NAV_TEST_HOVER_THROTTLE - 0.15f);
    expect("and it is still not steering: there is nowhere to steer to",
           fabsf(out.roll) < 0.01f && fabsf(out.pitch) < 0.01f);

    /*
     * And the same pilot setting with no part to measure the ground: the hold
     * goes on being a hold. This is the negative control for the whole
     * feature - the descent is only allowed because something is measuring
     * the ground, not because somebody asked for it.
     */
    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);
    nav.hold_land_s = 5u;
    in.range_fitted = 0;
    for (int i = 0; i < 600; i++) {
        (void)ak_nav_step(&nav, &in, &out);
    }
    expect("with no rangefinder the same setting still only holds",
           nav.hold_landing == 0 &&
               fabsf(out.throttle - NAV_TEST_HOVER_THROTTLE) < 0.02f);
    expect("and nothing has moved the aircraft down", in.alt_mm == 40000);

    /* The wing has no such manoeuvre at all: it cannot land without a
     * position, and where it would land is not somewhere anybody chose. */
    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_WING);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    (void)ak_nav_engage(&nav, 40000);
    nav.hold_land_s = 5u;
    in.gps_valid = 0;
    in.range_fitted = 1;
    for (int i = 0; i < 600; i++) {
        (void)ak_nav_step(&nav, &in, &out);
    }
    expect("and a wing with no fix still flies on rather than coming down",
           nav.hold_landing == 0 && out.throttle > 0.3f);
}

static void test_gps_loss_holds(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;
    ak_nav_input_t in;

    /* The wing: 1 km from home, 20 m below the altitude it was told to hold,
     * with the fix gone. */
    ak_nav_init(&nav);
    ak_nav_set_home(&nav, 521000000, 40000000, 0);
    ak_nav_engage(&nav, 120000);

    memset(&in, 0, sizeof in);
    in.lat_e7 = 520000000;
    in.lon_e7 = 40000000;
    in.alt_mm = 100000;
    in.course_e5 = 9000000; /* flying east, and home is north */
    in.max_tilt_rad = 0.6109f;
    in.gps_valid = 0;

    expect("a navigator with no fix keeps flying rather than giving up",
           ak_nav_step(&nav, &in, &out) == 1 && nav.active == 1);
    expect("and it holds its altitude instead of chasing a stale position",
           out.pitch > 0.05f && fabsf(out.roll) < 0.01f &&
               fabsf(out.yaw) < 0.001f);
    expect("and the console can see how long it has been holding",
           nav.hold_steps == 1u);

    /* The same input *with* a fix steers, which is the flag doing the work. */
    in.gps_valid = 1;
    (void)ak_nav_step(&nav, &in, &out);
    expect("and the same input with a fix steers home", out.roll < -0.5f);

    /* The quadrotor holds its height with throttle, and levels its attitude:
     * a hover that drifts, because there is nothing to hold a position
     * against, rather than a descent because the navigation stopped. */
    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    ak_nav_engage(&nav, 40000);

    in.alt_mm = 30000; /* 10 m below what it was told to hold */
    in.gps_valid = 0;
    expect("a quadrotor with no fix flies level and climbs back to its hold",
           ak_nav_step(&nav, &in, &out) == 1 && out.throttle > 0.6f &&
               fabsf(out.roll) < 0.01f && fabsf(out.pitch) < 0.01f);
}

/*
 * A mission is the same steering with a target that moves when it is reached.
 * The reason it is worth its own tests is that the interesting behaviour is all
 * at the ends: what happens before the first waypoint, between two, and after
 * the last one, where the aircraft has to end up circling rather than hunting
 * for the next thing forever.
 */
static void test_mission(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;

    ak_nav_init(&nav);

    expect("a mission with no waypoints will not start",
           ak_nav_start_mission(&nav, 100000) == 0);

    ak_nav_set_home(&nav, 520000000, 40000000, 0);
    ak_nav_set_waypoint(&nav, 0, 521000000, 40000000); /* 1.1 km north */
    ak_nav_set_waypoint(&nav, 1, 521000000, 401000000); /* then east */
    ak_nav_set_waypoint_count(&nav, 2);

    expect("starting one captures the altitude and points at the first",
           ak_nav_start_mission(&nav, 150000) == 1 &&
           ak_nav_mission_active(&nav) && ak_nav_waypoint_index(&nav) == 0 &&
           nav.hold_alt_mm == 150000);

    /* Flying east toward the first waypoint, which is north: roll left. */
    expect("it steers at the first waypoint",
           nav_step(&nav, 520000000, 40000000, 150000, 9000000, &out) == 1);
    expect("which is a left roll", out.roll < -0.5f);

    /* Arriving at it moves to the next one rather than stopping. */
    (void)nav_step(&nav, 521000000, 40000000, 150000, 0, &out);
    expect("reaching a waypoint advances to the next",
           ak_nav_waypoint_index(&nav) == 1 && nav.waypoints_reached == 1);

    /* The last one is not advanced past: the aircraft circles there, which is
     * the same thing a return does when it arrives. */
    (void)nav_step(&nav, 521000000, 401000000, 150000, 0, &out);
    expect("and reaching the last one starts the circle",
           fabsf(out.roll - nav.loiter_roll) < 0.001f);
    expect("with both arrivals counted, and the list still on the last one",
           ak_nav_waypoint_index(&nav) == 1 && nav.waypoints_reached == 2);

    /* Circling does not go on counting: an aircraft at a waypoint is there
     * once, and a counter that climbs is a counter that cannot be read. */
    for (int i = 0; i < 100; i++) {
        (void)nav_step(&nav, 521000000, 401000000, 150000, 0, &out);
    }
    expect("and circling does not count as arriving again",
           nav.waypoints_reached == 2);

    /* Nor does leaving the radius and coming back. A wing circling a waypoint
     * stays inside it, but a quadrotor holding station drifts in and out of a
     * fifteen metre circle, and "counted once each" has to mean once. Measured
     * in the loop before this was fixed: two waypoints, three arrivals. */
    (void)nav_step(&nav, 522000000, 402000000, 150000, 0, &out);
    (void)nav_step(&nav, 521000000, 401000000, 150000, 0, &out);
    expect("and leaving the radius and coming back does not either",
           nav.waypoints_reached == 2);

    /* It keeps circling rather than reporting the mission over: the aircraft
     * has nowhere else to be, and a navigator that stops guiding mid-air is a
     * navigator that has just handed back a wing that is not being flown. */
    for (int i = 0; i < 50; i++) {
        (void)nav_step(&nav, 521000000, 401000000, 150000, 0, &out);
    }
    expect("and it goes on guiding", fabsf(out.roll - nav.loiter_roll) < 0.001f);

    ak_nav_stop_mission(&nav);
    expect("stopping it says nothing at all",
           !ak_nav_mission_active(&nav) &&
           nav_step(&nav, 521000000, 401000000, 150000, 0, &out) == 0);

    /* A count past the end of the list is not a reason to fly off the end of
     * an array. */
    ak_nav_set_waypoint_count(&nav, 99);
    expect("the list is as long as it can be, and no longer",
           nav.waypoint_count == AK_NAV_WAYPOINTS);
    ak_nav_set_waypoint_count(&nav, -3);
    expect("and a negative count is none", nav.waypoint_count == 0);
}

/*
 * The same waypoint list, on the quadrotor.
 *
 * Two things are the airframe's answer rather than the list's, and both were
 * found by flying the mission on the quadrotor in the simulator: how close
 * counts as arriving (a wing can hold a circle of tens of metres, a quadrotor
 * can stop at a point) and whether arriving means *coming down*. A waypoint is
 * a place to be, not ground to land on, and the altitude a quadrotor's descent
 * aims at is the altitude home was captured at - so a descent at a waypoint
 * would be a landing somewhere else, at this aircraft's home ground level.
 */
static void test_quad_mission(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;
    ak_nav_input_t in;
    nav_test_quad_t quad;

    memset(&quad, 0, sizeof quad);
    quad.north_m = 1070.0f; /* 30 m short of a waypoint 1.1 km north */
    quad.east_m = 0.0f;
    quad.alt_m = 30.0f;     /* at the altitude the mission started at */
    quad.yaw_rad = 1.57079633f;
    quad.yaw_aligned = 1;

    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    /* A waypoint 1.1 km north: the aircraft is 30 m short of it, and 30 m is
     * inside a wing's arrival radius and outside a quadrotor's. */
    ak_nav_set_waypoint(&nav, 0, 521098814, 40000000);
    ak_nav_set_waypoint_count(&nav, 1);
    ak_nav_start_mission(&nav, 20000 + 30000);

    nav_test_input(&quad, &in);
    (void)ak_nav_step(&nav, &in, &out);
    expect("30 m from a waypoint is not arriving, for a quadrotor",
           nav.waypoints_reached == 0u);
    expect("and it is not coming down, because a waypoint is not home",
           nav.descending == 0);

    /* Within its own radius it counts the arrival - and the descent stays
     * where it belongs, which is the half of this that would land the
     * aircraft on somebody's garden. */
    quad.north_m = 1090.0f;
    nav_test_input(&quad, &in);
    (void)ak_nav_step(&nav, &in, &out);
    expect("inside its own radius the arrival counts",
           nav.waypoints_reached == 1u);
    expect("and it goes on holding its altitude over the waypoint",
           nav.descending == 0 && !nav.landing);

    /* Arriving *home* is what opens the descent, and that is a return. */
    ak_nav_init(&nav);
    ak_nav_set_profile(&nav, AK_NAV_PROFILE_QUAD);
    ak_nav_set_home(&nav, 521000000, 40000000, 20000);
    ak_nav_engage(&nav, 20000 + 30000);
    quad.north_m = 10.0f;
    quad.alt_m = 30.0f;
    nav_test_input(&quad, &in);
    (void)ak_nav_step(&nav, &in, &out);
    expect("but arriving home is, which is the return's own manoeuvre",
           nav.descending == 1);
}

/*
 * The floor and the fence: the two things between a return and a return that
 * arrives somewhere it should not have.
 */
static void test_floor_and_fence(void)
{
    ak_nav_t nav;
    ak_rc_command_t out;

    ak_nav_init(&nav);
    ak_nav_set_home(&nav, 520000000, 40000000, 0);
    (void)ak_nav_engage(&nav, 120000);          /* captured 120 m */

    expect("with no floor, the altitude it captured is the altitude it holds",
           ak_nav_hold_altitude(&nav) == 120000);

    /* A floor above the captured altitude wins: a return that descends into
     * the hill beside it is a return that arrives somewhere else. */
    nav.min_alt_mm = 200000;
    expect("a floor above the capture is what it holds",
           ak_nav_hold_altitude(&nav) == 200000);
    (void)nav_step(&nav, 520000000, 40000000, 120000, 0, &out);
    expect("and below the floor it climbs", out.pitch > 0.0f);

    nav.min_alt_mm = 100000;
    expect("a floor below the capture changes nothing",
           ak_nav_hold_altitude(&nav) == 120000);

    /* The fence is off until somebody turns it on. */
    expect("a fence that is off never complains",
           ak_nav_outside_fence(&nav, 530000000, 40000000) == 0);

    nav.fence_enabled = 1;
    expect("an enabled fence with no radius is still off",
           ak_nav_outside_fence(&nav, 530000000, 40000000) == 0);

    nav.fence_m = 500.0f;
    expect("inside the fence is inside",
           ak_nav_outside_fence(&nav, 520000000, 40000000) == 0);
    expect("five ten-thousandths of a degree is about 5.5 km",
           ak_nav_distance_home(&nav, 520500000, 40000000) > 5500 &&
           ak_nav_distance_home(&nav, 520500000, 40000000) < 5600);
    expect("which is well outside a 500 m fence",
           ak_nav_outside_fence(&nav, 520500000, 40000000) == 1);
    expect("and 300 m out is inside it",
           ak_nav_outside_fence(&nav, 520002695, 40000000) == 0);

    /* Without a home there is nothing to be outside of. */
    ak_nav_init(&nav);
    nav.fence_enabled = 1;
    nav.fence_m = 100.0f;
    expect("no home means no fence",
           ak_nav_outside_fence(&nav, 530000000, 40000000) == 0 &&
           ak_nav_distance_home(&nav, 530000000, 40000000) < 0);
}

/*
 * The lid: the fence's third side.
 *
 * One number doing two jobs, and both are needed. It caps the altitude the
 * navigator will hold, so a return that engaged above it comes down instead of
 * flying home high - and it is a *trigger*, because a pilot flying by hand
 * through it is the same problem as a pilot flying by hand through the radius:
 * somebody else has to take the aircraft. Both are measured here against a home
 * at 120 m, which is the whole point of expressing it above home rather than on
 * the absolute altitude: the pilot is setting a limit on how high they are
 * flying, not on what the barometer reads.
 */
static void test_fence_ceiling(void)
{
    ak_nav_t nav;

    ak_nav_init(&nav);
    ak_nav_set_home(&nav, 520000000, 40000000, 120000);

    expect("with no ceiling the fence says nothing about altitude",
           ak_nav_above_ceiling(&nav, 500000) == 0);
    nav.fence_enabled = 1;
    expect("an enabled fence with no ceiling still says nothing",
           ak_nav_above_ceiling(&nav, 500000) == 0);

    nav.fence_ceiling_m = 60.0f; /* 180 m above sea level, at this home */
    expect("below the ceiling is below",
           ak_nav_above_ceiling(&nav, 179999) == 0);
    expect("exactly at it is still under it",
           ak_nav_above_ceiling(&nav, 180000) == 0);
    expect("and one millimetre over is over it",
           ak_nav_above_ceiling(&nav, 180001) == 1);

    /* A return that engaged above the ceiling holds the ceiling, not the
     * altitude it happened to be at. */
    (void)ak_nav_engage(&nav, 180000);
    nav.fence_ceiling_m = 40.0f; /* 160 m above sea level */
    expect("a return engaged above the ceiling holds below it",
           ak_nav_hold_altitude(&nav) == 150000);

    /* And the room comes off the margin: a floor five metres under the lid
     * leaves five metres of margin, so the floor is still what it holds. And a
     * floor well under the lid leaves the margin at its own ten metres. */
    nav.min_alt_mm = 155000;
    expect("a floor close under the ceiling is still the floor",
           ak_nav_hold_altitude(&nav) == 155000);
    nav.min_alt_mm = 140000;
    expect("a floor with room under it leaves the margin its ten metres",
           ak_nav_hold_altitude(&nav) == 150000);

    /* A floor *above* the ceiling, which is a pilot asking for something
     * impossible: the ceiling is the one that holds, because a ceiling is
     * airspace and a floor is terrain the aircraft can fly over. */
    nav.min_alt_mm = 200000;
    expect("a floor above the ceiling does not win",
           ak_nav_hold_altitude(&nav) == 160000);

    /* A lid so low that the margin would put the hold underground: the ground
     * is the floor of the last resort, not a descent through it. */
    nav.min_alt_mm = 0;
    nav.fence_ceiling_m = 5.0f; /* 125 m above sea level, home is at 120 m */
    expect("a lid under the margin holds the ground, not below it",
           ak_nav_hold_altitude(&nav) == 120000);

    /* The fence is one switch, and the ceiling is part of it: with the fence
     * off, a floor above where the lid was is the floor again. */
    nav.min_alt_mm = 200000;
    nav.fence_enabled = 0;
    expect("with the fence off there is no ceiling to be over",
           ak_nav_above_ceiling(&nav, 500000) == 0);
    expect("nor a ceiling to hold under",
           ak_nav_hold_altitude(&nav) == 200000);
}

/*
 * Managed flight: the pilot hands over on purpose, with the link still up.
 *
 * Everything that matters here is about who is in charge at a given moment. A
 * navigator that flew while the pilot was flying would be a hazard, and a
 * navigator that stopped flying the moment the link dropped would have been
 * the sort of failsafe nobody wants: the aircraft is already being flown, and
 * a mission that is going well should carry on.
 */
static void test_managed_flight(void)
{
    ak_flight_t flight;
    ak_rc_command_t guidance;
    ak_nav_t nav;
    uint32_t t = 0;

    flight_with_outputs(&flight, &ak_mixer_elevon_wing);

    /* Full-forward roll on the guidance, so "the navigator is flying" is
     * visible in the outputs: the sticks are centred throughout. */
    guidance.roll = 1.0f;
    guidance.pitch = 0.0f;
    guidance.yaw = 0.0f;
    guidance.throttle = 0.55f;
    guidance.angle_mode = 1;
    guidance.arm_request = 1;

    for (int i = 0; i < 900; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("armed with the sticks centred",
           ak_flight_state(&flight) == AK_FLIGHT_ARMED);

    /* Guidance without permission does nothing: that is the rule that keeps a
     * navigator from fighting a pilot. */
    ak_flight_set_guidance(&flight, &guidance);
    for (int i = 0; i < 50; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("guidance alone does not take over a live link",
           ak_flight_state(&flight) == AK_FLIGHT_ARMED);

    /* Hand over. */
    ak_flight_set_managed(&flight, 1);
    for (int i = 0; i < 200; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("handing over puts the aircraft on mission",
           ak_flight_state(&flight) == AK_FLIGHT_MANAGED &&
           ak_flight_managed(&flight));
    expect("and the navigator is the one flying it",
           ak_flight_outputs(&flight)->motor[0] > 0.0f);

    /* The link goes. The mission carries on, because the navigator was already
     * flying - this is not a failsafe, it is a flight that lost its radio. */
    for (int i = 0; i < 100; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t + 1000);
    }
    expect("losing the link does not end a mission",
           ak_flight_state(&flight) == AK_FLIGHT_MANAGED);

    /* The pilot comes back and asks for the aircraft. */
    for (int i = 0; i < 10; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    ak_flight_set_managed(&flight, 0);
    for (int i = 0; i < 10; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("taking it back puts the sticks in charge",
           ak_flight_state(&flight) == AK_FLIGHT_ARMED &&
           !ak_flight_managed(&flight));

    /* And the arm switch still wins over all of it. */
    ak_flight_set_managed(&flight, 1);
    for (int i = 0; i < 50; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("on mission again", ak_flight_state(&flight) == AK_FLIGHT_MANAGED);
    for (int i = 0; i < 10; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 0); /* arm switch off */
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("but the arm switch disarms from a mission too",
           ak_flight_state(&flight) == AK_FLIGHT_DISARMED);
    expect("and disarming lets go of the navigator entirely",
           !ak_flight_managed(&flight));

    (void)nav;
}

static ak_rc_input_t rc_frame(uint32_t now_ms, int arm)
{
    ak_rc_input_t rc;
    ak_rc_config_t cfg;
    ak_rc_default_config(&cfg);
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        rc.channel[i] = (uint16_t)cfg.mid;
    }
    rc.channel[AK_RC_THROTTLE] = (uint16_t)cfg.min;
    rc.channel[AK_RC_ARM] = (uint16_t)(arm ? cfg.max : cfg.min);
    rc.last_update_ms = now_ms;
    rc.valid = 1;
    return rc;
}

static void test_flight_core_takes_guidance(void)
{
    ak_flight_t flight;
    flight_with_outputs(&flight, &ak_mixer_elevon_wing);
    ak_flight_set_guidance(&flight, 0);

    uint32_t t = 0;
    /* Arm it with a live link, throttle down, for longer than the hold. */
    for (int i = 0; i < 900; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("armed with a live link", ak_flight_state(&flight) == AK_FLIGHT_ARMED);

    /* Now the link goes, with guidance available: the aircraft must keep
     * flying, not stop. */
    ak_rc_command_t guidance;
    ak_nav_t nav;
    ak_nav_init(&nav);
    ak_nav_set_home(&nav, 521000000, 40000000, 0);
    (void)ak_nav_engage(&nav, 120000);
    (void)nav_step(&nav, 520000000, 40000000, 120000, 0, &guidance);
    ak_flight_set_guidance(&flight, &guidance);

    for (int i = 0; i < 100; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t + 1000); /* stale: the link is gone */
    }
    expect("with guidance, losing the link returns rather than stops",
           ak_flight_state(&flight) == AK_FLIGHT_RTH);
    expect("and the motors keep running",
           ak_flight_outputs(&flight)->motor[0] > 0.0f);

    /* The pilot comes back: control returns to the sticks. */
    for (int i = 0; i < 10; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("a returning link hands control back to the pilot",
           ak_flight_state(&flight) == AK_FLIGHT_ARMED);

    /* Without guidance the same loss stops the aircraft, which is the
     * behaviour to keep until a return has been tested. */
    ak_flight_set_guidance(&flight, 0);
    for (int i = 0; i < 50; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 1);
        ak_flight_step(&flight, &imu, &rc, t + 5000);
    }
    expect("without guidance the aircraft stops",
           ak_flight_state(&flight) == AK_FLIGHT_FAILSAFE &&
           ak_flight_outputs(&flight)->motor[0] == 0.0f);

    /* And guidance cannot arm a disarmed aircraft: a navigator decides where to
     * fly, never whether to be armed. */
    ak_flight_t fresh;
    flight_with_outputs(&fresh, &ak_mixer_elevon_wing);
    ak_flight_set_guidance(&fresh, &guidance);
    for (int i = 0; i < 300; i++) {
        t++;
        ak_imu_sample_t imu = level_imu(t);
        ak_rc_input_t rc = rc_frame(t, 0);
        ak_flight_step(&fresh, &imu, &rc, t + 9000);
    }
    expect("guidance cannot arm a disarmed aircraft",
           ak_flight_state(&fresh) == AK_FLIGHT_DISARMED &&
           ak_flight_outputs(&fresh)->motor[0] == 0.0f);
}

/* The state a board reports before anything has been armed. On the ESP32 -
 * which has no inertial sensor yet - this is the state it boots into, and the
 * first version of the port reported "failsafe" from its first loop iteration,
 * which reads as an aircraft in trouble rather than one that has never flown. */
static void test_disarmed_is_not_a_failsafe(void)
{
    ak_flight_t flight;
    flight_with_outputs(&flight, &ak_mixer_quad_x);
    ak_flight_set_guidance(&flight, 0);

    /* No IMU, no receiver: a board with nothing plugged into it. */
    ak_imu_sample_t imu = level_imu(1);
    imu.valid = 0;
    ak_rc_input_t rc = rc_frame(1, 0);
    rc.valid = 0;

    for (uint32_t t = 1; t <= 200; t++) {
        imu.time_ms = t;
        ak_flight_step(&flight, &imu, &rc, t);
    }
    expect("a disarmed aircraft with no sensors stays disarmed",
           ak_flight_state(&flight) == AK_FLIGHT_DISARMED);
    expect("and its motors stay stopped",
           ak_flight_outputs(&flight)->motor[0] == 0.0f);

    /*
     * And a navigator's guidance does not change that. This is the rule the
     * bring-up checklist's return-to-home test rests on - a disarmed aircraft
     * is disarmed whatever the navigator would like it to do - which is why
     * that step has to arm the aircraft first, props off, before it can expect
     * a single output to move.
     */
    ak_flight_t hinted;
    ak_rc_command_t nav_command;
    flight_with_outputs(&hinted, &ak_mixer_quad_x);
    memset(&nav_command, 0, sizeof nav_command);
    nav_command.angle_mode = 1;
    nav_command.throttle = 0.6f;
    nav_command.pitch = 0.3f;
    ak_flight_set_guidance(&hinted, &nav_command);
    ak_flight_set_managed(&hinted, 1);
    for (uint32_t t = 1; t <= 200; t++) {
        ak_imu_sample_t good = level_imu(t);
        ak_rc_input_t link = rc_frame(t, 0); /* receiver up, arm switch low */
        ak_flight_step(&hinted, &good, &link, t);
    }
    expect("and it ignores a navigator asking for a climb",
           ak_flight_state(&hinted) == AK_FLIGHT_DISARMED &&
           ak_flight_outputs(&hinted)->motor[0] == 0.0f);

    /* The other half: armed, with the link gone and a navigator flying, the
     * motors do turn - which is what the bench watcher is looking for. */
    ak_flight_t manned;
    flight_with_outputs(&manned, &ak_mixer_quad_x);
    uint32_t tt = 0;
    for (int i = 0; i < 900; i++) {
        tt++;
        ak_imu_sample_t good = level_imu(tt);
        ak_rc_input_t link = rc_frame(tt, 1);
        ak_flight_step(&manned, &good, &link, tt);
    }
    ak_flight_set_guidance(&manned, &nav_command);
    ak_flight_set_managed(&manned, 0); /* a return, not the pilot's request */
    {
        ak_imu_sample_t good = level_imu(++tt);
        ak_rc_input_t gone = rc_frame(tt, 1);

        gone.valid = 0; /* the transmitter went off */
        ak_flight_step(&manned, &good, &gone, tt);
    }
    expect("and an armed one with a lost link does fly the guidance",
           ak_flight_state(&manned) == AK_FLIGHT_RTH &&
           ak_flight_outputs(&manned)->motor[0] > 0.1f);

    /* The same board once it has been armed and then loses the IMU is a
     * different matter: that is a failsafe, and it latches. */
    ak_flight_t armed;
    flight_with_outputs(&armed, &ak_mixer_quad_x);
    uint32_t t = 0;
    for (int i = 0; i < 900; i++) {
        t++;
        ak_imu_sample_t good = level_imu(t);
        ak_rc_input_t link = rc_frame(t, 1);
        ak_flight_step(&armed, &good, &link, t);
    }
    expect("and once armed", ak_flight_state(&armed) == AK_FLIGHT_ARMED);

    ak_imu_sample_t gone = level_imu(++t);
    gone.valid = 0;
    ak_rc_input_t link = rc_frame(t, 1);
    ak_flight_step(&armed, &gone, &link, t);
    expect("losing the IMU while armed is a failsafe",
           ak_flight_state(&armed) == AK_FLIGHT_FAILSAFE &&
           ak_flight_outputs(&armed)->motor[0] == 0.0f);

    /* And it *stays* one when the sensor comes back: a failsafe that clears
     * itself the moment the fault does is a failsafe that starts flying again
     * on its own. What clears this is the arm switch going off. */
    ak_imu_sample_t back = level_imu(++t);
    ak_rc_input_t still_armed = rc_frame(t, 1);
    for (int i = 0; i < 200; i++) {
        back = level_imu(++t);
        still_armed = rc_frame(t, 1);
        ak_flight_step(&armed, &back, &still_armed, t);
    }
    expect("and it stays one after the sensor comes back",
           ak_flight_state(&armed) == AK_FLIGHT_FAILSAFE &&
           ak_flight_outputs(&armed)->motor[0] == 0.0f);
}

void test_navigation(void)
{
    test_distance_and_bearing();
    test_course_error();
    test_guidance();
    test_quad_return();
    test_wing_altitude_droop();
    test_alignment_nudge();
    test_quad_descent_rate();
    test_vertical_rate_loop();
    test_vertical_integral_is_bounded();
    test_rangefinder_descent();
    test_gps_loss_holds();
    test_mission();
    test_quad_mission();
    test_floor_and_fence();
    test_fence_ceiling();
    test_managed_flight();
    test_flight_core_takes_guidance();
    test_disarmed_is_not_a_failsafe();
}
