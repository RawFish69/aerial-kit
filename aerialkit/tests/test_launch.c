/*
 * The hand launch.
 *
 * The manoeuvre is small enough to hold in one hand: a throttle, an attitude
 * and a clock, and three ways out (a stick, the clock, the switch). What is
 * worth testing here is not that it climbs - that is one multiply - but the
 * *edges*, because every one of them is a way for a mode that flies an aircraft
 * for two seconds to be wrong in the air: a stick at exactly the abort fraction
 * is the difference between a launch that ends when the pilot takes it back and
 * one that keeps flying an aircraft somebody is already flying; a clock that
 * restarts is a launch that outlives its own timeout; and a command whose climb
 * is a fraction of the wrong travel is a climb nobody asked for.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ak_launch.h"
#include "ak_math.h"
#include "tests.h"

static ak_rc_command_t sticks(float roll, float pitch, float yaw)
{
    ak_rc_command_t cmd;

    cmd.roll = roll;
    cmd.pitch = pitch;
    cmd.yaw = yaw;
    cmd.throttle = 0.0f; /* the pilot's, which the launch overrides */
    cmd.angle_mode = 1;
    cmd.arm_request = 1;
    return cmd;
}

void test_launch(void)
{
    const float max_tilt = ak_deg2rad(35.0f);
    ak_launch_t launch;
    ak_rc_command_t pilot = sticks(0.0f, 0.0f, 0.0f);
    ak_rc_command_t out;

    ak_launch_init(&launch);
    expect("a launch that was never started flies nothing",
           !ak_launch_active(&launch) &&
               ak_launch_step(&launch, &pilot, max_tilt, 0u, &out) == 0);

    /* And the numbers it would fly with are the reference's: INAV launches a
     * wing at 70 per cent and an 18 degree climb attitude, for five seconds. */
    expect("the throttle is the reference's 70 per cent",
           ak_absf(launch.cfg.throttle - 0.70f) < 0.001f);
    expect("the climb is the reference's 18 degrees",
           ak_absf(launch.cfg.climb_deg - 18.0f) < 0.001f);
    expect("the timeout is the reference's five seconds",
           launch.cfg.timeout_ms == 5000u);
    expect("and the abort fraction is a fifteenth of travel",
           ak_absf(launch.cfg.abort_stick - 0.15f) < 0.001f);

    ak_launch_start(&launch, 0u);
    expect("a started launch is flying", ak_launch_active(&launch));
    expect("and it produces a command",
           ak_launch_step(&launch, &pilot, max_tilt, 1u, &out) == 1);
    expect("the throttle is the launch's, not the pilot's",
           ak_absf(out.throttle - 0.70f) < 0.001f);
    /* 18 degrees of attitude, as a fraction of the 35 degree travel the angle
     * loop scales a stick by: the two have to be the same travel or the climb
     * is not the number the pilot set. */
    expect("the climb is the attitude over the loop's own tilt limit",
           ak_absf(out.pitch - ak_deg2rad(18.0f) / max_tilt) < 0.001f);
    expect("the wings are level and the nose is not yawed",
           out.roll == 0.0f && out.yaw == 0.0f);
    expect("and it is an attitude, whatever mode the pilot's switch says",
           out.angle_mode == 1);

    /* A climb angle the aircraft's tilt limit cannot express is clamped rather
     * than clipped twice: the command is what the core reads, and the core
     * clamps it again. */
    launch.cfg.climb_deg = 60.0f;
    (void)ak_launch_step(&launch, &pilot, ak_deg2rad(10.0f), 2u, &out);
    expect("a climb past the tilt limit asks for all of it",
           ak_absf(out.pitch - 1.0f) < 0.001f);
    launch.cfg.climb_deg = 18.0f;

    /* A stick at exactly the abort fraction is not a stick: the threshold is
     * "past", so a receiver's jitter around it does not end a launch. */
    pilot = sticks(0.15f, 0.0f, 0.0f);
    expect("a stick at the fraction is still the launch's",
           ak_launch_step(&launch, &pilot, max_tilt, 3u, &out) == 1);
    pilot = sticks(0.16f, 0.0f, 0.0f);
    expect("a stick past it hands the aircraft back",
           ak_launch_step(&launch, &pilot, max_tilt, 4u, &out) == 0 &&
               !ak_launch_active(&launch));
    expect("and the reason is the stick",
           launch.ended_because == AK_LAUNCH_REASON_STICK);
    expect("and it says so in words",
           strcmp(ak_launch_reason_name(AK_LAUNCH_REASON_STICK), "a stick") == 0);
    expect("a launch that has ended flies nothing, even with the switch up",
           ak_launch_step(&launch, &pilot, max_tilt, 5u, &out) == 0);

    /* Yaw and pitch abort it too: a hand on any of the three sticks is a hand
     * on the aircraft. */
    pilot = sticks(0.0f, -0.4f, 0.0f);
    ak_launch_start(&launch, 0u);
    expect("a pitch stick hands it back",
           ak_launch_step(&launch, &pilot, max_tilt, 10u, &out) == 0);
    pilot = sticks(0.0f, 0.0f, 0.3f);
    ak_launch_start(&launch, 0u);
    expect("and so does the rudder",
           ak_launch_step(&launch, &pilot, max_tilt, 10u, &out) == 0);

    /* The clock: the timeout is measured from the start, it is the boundary
     * that decides, and starting one that is already running does not reset it.
     * That last one is the point - a flapping switch must not buy an aircraft
     * another five seconds of somebody else flying it. */
    pilot = sticks(0.0f, 0.0f, 0.0f);
    ak_launch_start(&launch, 1000u);
    expect("a launch one millisecond short of its timeout is still flying",
           ak_launch_step(&launch, &pilot, max_tilt, 5999u, &out) == 1);
    expect("and at its timeout it is not",
           ak_launch_step(&launch, &pilot, max_tilt, 6000u, &out) == 0);
    expect("and the reason is the time",
           launch.ended_because == AK_LAUNCH_REASON_TIMEOUT);

    ak_launch_start(&launch, 1000u);
    ak_launch_start(&launch, 4000u); /* the switch flicked off and on again */
    expect("a restart does not restart the clock",
           ak_launch_step(&launch, &pilot, max_tilt, 6000u, &out) == 0);

    /* And the caller can stop it, which is what the switch going down, the arm
     * switch going down and a fence all do. */
    ak_launch_start(&launch, 0u);
    ak_launch_stop(&launch, AK_LAUNCH_REASON_OFF);
    expect("a stopped launch is not flying",
           !ak_launch_active(&launch) &&
               launch.ended_because == AK_LAUNCH_REASON_OFF);

    /* A launch with a zero timeout is the pilot's own: it ends at the first
     * step, which is the honest way to configure "no timer". */
    ak_launch_init(&launch);
    launch.cfg.timeout_ms = 0u;
    ak_launch_start(&launch, 100u);
    expect("a zero timeout ends it at the first step",
           ak_launch_step(&launch, &pilot, max_tilt, 100u, &out) == 0);

    printf("  launch: %d degrees at %d per cent, %u ms, abort past %.2f\n",
           (int)(18.0f + 0.5f), 70, 5000u, 0.15);
}
