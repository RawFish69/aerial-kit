#ifndef AK_FLIGHT_AK_NAV_H
#define AK_FLIGHT_AK_NAV_H

#include <stdint.h>

#include "ak_rc.h"

/*
 * Navigation, for the fixed wing: where home is, which way it is, and what to
 * do about it.
 *
 * The output is deliberately the same shape the RC decoder produces - sticks
 * and a throttle - because that is the interface the control loop and the mixer
 * already understand. A navigator that reached into the flight loop would have
 * to be re-tested against every airframe; one that produces stick positions
 * only has to be right about where home is.
 *
 * There are two profiles, because the two airframes return in different ways.
 *
 * The **fixed wing** holds a cruise throttle, steers with roll toward home,
 * holds altitude with pitch, and circles when it arrives.
 *
 * The **quadrotor** climbs to the return altitude, translates in the body
 * frame (a tilt is an acceleration for a quad, not a turn), and settles into a
 * hover above home. That needs something a wing does not: the aircraft's
 * heading *relative to north*, because a world-frame position error has to be
 * turned into body-frame tilts. Without a magnetometer that comes from the GPS
 * ground track (ak_estimator_aid_heading), which is why the profile asks
 * whether the yaw has been aligned and holds still until it has - a tilt
 * computed with a yaw that is 90 degrees out flies the aircraft confidently in
 * the wrong direction.
 *
 * The profile is the only thing that changes: the output is still sticks and a
 * throttle, so the control loop and the mixer never learn which one is flying.
 */

/* How many places a mission can name. Small on purpose: a list that fits on a
 * console screen is a list a person can check, and the memory is better spent
 * on the flight loop. */
#define AK_NAV_WAYPOINTS 4

/* The climb rate the quadrotor's vertical loop flies on is measured over a
 * window rather than between two consecutive steps, and the window is eight
 * samples 25 ms apart - a sixth of a second: long enough to see several
 * barometer updates, short enough to leave the loop its phase margin. The
 * arithmetic behind the number is in ak_nav.c, quad_climb_sample(). */
#define AK_NAV_RATE_SAMPLES 8
#define AK_NAV_RATE_STEP_MS 25u

/*
 * The longest interval one guidance step will integrate over, in milliseconds.
 *
 * The navigator is a guidance loop: it is handed the time since its last step
 * and uses it as the timestep of every integrator it owns. That is right for a
 * late step and wrong for a loop that did not run, and the two arrive here as
 * the same number. `main.c` freezes the interval while the navigator is
 * disengaged - a GPS outage returns early from `nav_update()` before the
 * interval is computed - so a return that engages at the moment the fix comes
 * back is handed the whole outage as one step.
 *
 * A step that long is not a bigger correction, it is a wrong one. Measured on
 * the host, a wing circling two hundred millimetres below the altitude it is
 * holding, `alt_ki` 0.1, with twenty 20 ms steps behind it: one step of `dt_s`
 * 5.0 took `alt_i` from 0.0076 to 0.1076, and the commanded pitch with it from
 * 0.0096 to 0.1096. A 20 ms step is worth 0.0004 of that accumulator, so one
 * step arrived carrying two hundred and fifty steps' worth of integrator.
 * Bounded, the same step is worth 0.005 - twelve and a half of them.
 *
 * 250 ms is past "this control step ran late" and into "this loop did not run".
 * It is anchored in the rate window above rather than picked: 175 ms is the
 * window the quadrotor's climb rate is measured over, and a step longer than
 * the window is a step whose altitude change no window can hold.
 *
 * What is bounded is one step, not the flight: the surplus is counted in
 * `dt_clipped_ms` and reported at the console, which is the flight core's own
 * rule for a gap it could not use (see AK_FLIGHT_MAX_CATCHUP: "what the loop
 * could not use, it reports"). The navigator does not catch up by running the
 * loop several times, because its output is one set of sticks per step and the
 * intermediate answers reach nobody.
 */
#define AK_NAV_MAX_DT_MS 250u
#define AK_NAV_MAX_DT_S 0.25f

typedef enum {
    AK_NAV_PROFILE_WING = 0,
    AK_NAV_PROFILE_QUAD = 1,
} ak_nav_profile_t;

/* What the navigator is told each step. A struct rather than six arguments,
 * because every one of them is a measurement with its own units and the
 * call sites are easier to read this way. */
typedef struct {
    int32_t lat_e7, lon_e7;
    int32_t alt_mm;      /* above mean sea level */
    /* Whether the position above is a *fix* or the last one the module sent.
     * A navigator that is already flying keeps flying when this goes false -
     * see ak_nav_step() - because the alternative is stopping the motors over
     * whatever is underneath it. */
    int     gps_valid;
    int32_t course_e5;   /* the gps ground course, degrees * 1e5 */
    int32_t speed_mm_s;  /* the gps ground speed */
    int32_t yaw_mrad;    /* the estimator's heading */
    int     yaw_aligned; /* ... aligned to the ground track */
    /* What a full stick asks the angle loop for, so the quadrotor's profile
     * can command a *tilt* rather than an arbitrary number of sticks: the
     * flight core owns that number (max_tilt_deg), and this is it, in radians. */
    float   max_tilt_rad;
    /*
     * How far the ground is below, from a rangefinder, when there is one.
     *
     * `agl_valid` is that number, and `range_fitted` is a different fact: a
     * part that is out of range is answering "nothing within two metres of
     * me", which is what a descent onto an unknown spot wants to hear while it
     * is still high - so the navigator will start a landing in place with a
     * part it cannot yet read, and fly the last of it on the number.
     */
    int32_t agl_mm;
    int     agl_valid;
    int     range_fitted;
    /* Seconds since the last step. The quadrotor's profile differentiates the
     * altitude to get a climb rate - the GPS gives a ground speed but nothing
     * vertical, so the rate has to come from the height itself - and a
     * derivative without a timestep is a number with no units. */
    float   dt_s;
} ak_nav_input_t;

typedef struct {
    /* Guidance gains, in the units a person would tune them in. */
    float course_kp;   /* roll per degree of course error */
    float alt_kp;      /* pitch per metre of altitude error */
    /*
     * And the standing pitch the wing needs, learned. A proportional altitude
     * loop holds an altitude *below* the one it was given: the pitch that
     * stops the sink has to come from somewhere and the error is the only
     * place it can come from. The quadrotor's vertical loop had the same shape
     * and got the same fix - hold_altitude() in ak_nav.c has the numbers, the
     * bound and what the bound is for.
     */
    float alt_ki;      /* pitch per metre-second of altitude error */
    float alt_i;       /* the standing pitch learned so far */
    float cruise;      /* throttle to hold while returning */
    float loiter_roll; /* bank angle for the circle at home, as a stick fraction */
    float arrive_m;    /* how close counts as arrived */
    /* A floor under the altitude the navigator will hold. Zero means none:
     * a return that descends through the hill you were flying beside is a
     * return that arrives somewhere else. */
    int32_t min_alt_mm;
    /* How far from home the aircraft may be, and whether that is enforced. A
     * fence is a return that triggers itself, so it is a behaviour somebody
     * turns on rather than one that is on. */
    float   fence_m;
    /*
     * And how high above home it may go, in metres. Zero is no lid. The fence
     * is drawn around home - the same place the radius is measured from - and
     * this is the third side of the box: the radius keeps it in, the ceiling
     * keeps it under, and `min_alt_mm` keeps it off the hill. Crossable in
     * either direction, so it is both a trigger (the navigator takes the
     * aircraft, like the radius) and a cap on the altitude it will hold.
     */
    float   fence_ceiling_m;
    int     fence_enabled;

    /* Which airframe's return this is, and the quadrotor's own numbers. The
     * wing's are above; these mean nothing to it. */
    ak_nav_profile_t profile;
    float   quad_kp;        /* (m/s) of commanded speed per metre of error */
    float   quad_speed;     /* m/s, the most it will ask for */
    float   quad_arrive_m;  /* metres from home where it starts coming down */
    float   quad_hover_m;   /* metres above home where the quad settles */
    /* The vertical loop's integral gain: how fast the standing throttle it
     * needs to fly a rate is learned. See quad_throttle() - a proportional
     * loop settles short of the rate it was asked for, and that shortfall is
     * the whole difference between the descent the profile commands and the
     * one the aircraft flies. */
    float   quad_alt_ki;    /* throttle per (m/s) of rate error per second */
    float   climb_i;        /* the standing throttle learned so far */
    /* What the position loop has learned about the wind: a standing addition
     * to the velocity it asks for, so that holding a station does not need a
     * standing position error to lean against the drift. See quad_hold(). */
    float   hold_n_m_s;
    float   hold_e_m_s;
    int32_t home_alt_mm;    /* the altitude home was captured at */
    /* The climb rate the vertical loop flies on, measured over a window
     * rather than between two consecutive steps: see quad_climb_sample(). */
    int32_t  rate_alt_mm[AK_NAV_RATE_SAMPLES];
    /* And when each of those samples was taken, on a clock the navigator keeps
     * itself. The window's *length* has to come from the samples that are
     * actually in it: taking it from the nominal spacing instead assumes the
     * caller's step is the spacing, and a caller that runs at 20 ms would get
     * a rate 1.6 times the truth - which the loop would then fly to. See
     * quad_climb_sample(). */
    uint32_t rate_time_ms[AK_NAV_RATE_SAMPLES];
    uint32_t rate_clock_ms;
    unsigned rate_next;
    unsigned rate_count;
    unsigned rate_age_ms;
    float    climb_m_s;
    /* Steps flown holding with no position fix, which is what the console
     * reports when a return has lost its GPS: a number rather than a flag,
     * because "for how long" is the question a person asks next. */
    uint32_t hold_steps;
    /* The descent has begun and does not un-begin: a return that has arrived
     * once and then drifts out of the arrival radius must not climb back to
     * the cruising altitude to come the last few metres again. */
    int     descending;
    /* The hover is over and the last part of the return is on: down to the
     * ground, slowly, and then the flight core disarms. A return that hovers
     * until the battery dies is not a return - the aircraft is still in the
     * air with nobody flying it - and the plan's own words for this manoeuvre
     * are climb, translate, descend, land. */
    int     landing;
    /*
     * A landing with no position fix: the same descent, onto whatever happens
     * to be below the aircraft, after it has held station for as long as the
     * pilot allowed. It is a separate flag from `landing` because it is a
     * different manoeuvre with a different justification - see hold_step() -
     * and because a fix that comes back in the middle of one hands the
     * aircraft back to the return it started as.
     */
    int     hold_landing;
    /* Seconds of holding with no fix before that begins, and how long it has
     * been holding. Zero seconds means never, which is the default: it is a
     * behaviour somebody turns on, like the fence. */
    uint32_t hold_land_s;
    uint32_t no_fix_ms;
    uint32_t hover_ms;
    /*
     * Milliseconds of interval the guidance loop was handed but would not
     * integrate, because a single step that long is a wrong correction rather
     * than a bigger one. See AK_NAV_MAX_DT_MS. A count rather than a flag, and
     * for the same reason `hold_steps` is one: "how much" is what a person
     * asks next, and a navigator that quietly shortened a step would be
     * telling a story about a loop that ran when it did not.
     */
    uint32_t dt_clipped_ms;
    /* The last course a fix reported while the aircraft was moving. A hovering
     * aircraft has no direction of travel and its module reports noise, so the
     * course the turn is planned from is the last one that meant something -
     * the yaw estimate carries it in between, on the gyro. */
    int32_t last_course_e5;
    /* The yaw estimate at the moment that course was measured. Between fixes
     * of a course that means something, the gyro is what carries it: the
     * difference between now and then is how far the aircraft has turned. */
    int32_t course_yaw_mrad;

    int32_t home_lat_e7;
    int32_t home_lon_e7;
    int32_t hold_alt_mm;   /* altitude captured when the return was engaged */
    int     have_home;
    int     active;
    uint32_t steps;
    /*
     * The nudge, and how long it has been trying: a quadrotor with no
     * magnetometer cannot know which way its nose points until it moves, so
     * when it has to go somewhere and does not know, it *makes* a track to
     * learn from. See quad_step().
     */
    uint32_t nudge_ms;
    int      nudge_gave_up;

    /* A mission: places to go, in order. The aircraft flies to each in turn and
     * circles at the last one - so a mission whose last waypoint is home is a
     * mission that ends at home. */
    int32_t waypoint_lat_e7[AK_NAV_WAYPOINTS];
    int32_t waypoint_lon_e7[AK_NAV_WAYPOINTS];
    int     waypoint_count;
    int     waypoint_index;  /* which one is being flown now */
    int     mission;         /* 1 while a mission is being flown */
    uint32_t waypoints_reached; /* arrivals, counted once each */
    /* Which waypoint that counter has already counted, so an aircraft that
     * holds station or circles *on* the arrival radius cannot count the same
     * one twice. -1 is "none yet". */
    int     last_counted_index;
    int     at_target;          /* already inside the arrival radius */
} ak_nav_t;

void ak_nav_init(ak_nav_t *nav);

/* Home, and the altitude it was captured at - which is the ground, because
 * home is only ever set while the aircraft is standing on it. The quadrotor's
 * return descends to `quad_hover_m` above that; the wing ignores it. */
void ak_nav_set_home(ak_nav_t *nav, int32_t lat_e7, int32_t lon_e7,
                     int32_t alt_mm);

void ak_nav_set_profile(ak_nav_t *nav, ak_nav_profile_t profile);

/* Engage: capture the altitude to hold and start steering. Returns 0 if there
 * is no home to return to. */
int ak_nav_engage(ak_nav_t *nav, int32_t current_alt_mm);

void ak_nav_disengage(ak_nav_t *nav);

/* The mission list. Setting one is just storing it; the flight starts when the
 * pilot says so. */
void ak_nav_set_waypoint(ak_nav_t *nav, int index, int32_t lat_e7,
                         int32_t lon_e7);
void ak_nav_set_waypoint_count(ak_nav_t *nav, int count);

/* Start flying them, holding `current_alt_mm` the way a return does. Returns 0
 * with no waypoints to fly, or with nowhere to have come from. */
int ak_nav_start_mission(ak_nav_t *nav, int32_t current_alt_mm);

/* Stop, wherever it is: the pilot has taken the aircraft back. */
void ak_nav_stop_mission(ak_nav_t *nav);

int ak_nav_mission_active(const ak_nav_t *nav);

/* Which waypoint is being flown, or -1 when they have all been reached. */
int ak_nav_waypoint_index(const ak_nav_t *nav);

/* How far the aircraft is from home, in metres, or -1 if there is no home to
 * measure from. */
int32_t ak_nav_distance_home(const ak_nav_t *nav, int32_t lat_e7,
                             int32_t lon_e7);

/* Outside the fence: 1 when the aircraft is further from home than the fence
 * allows, 0 when it is inside, the fence is off, or there is no home. */
int ak_nav_outside_fence(const ak_nav_t *nav, int32_t lat_e7, int32_t lon_e7);

/* Above the fence: the same question about the other side of the box, against
 * the altitude over home. 0 when the fence is off, no ceiling is set, or the
 * aircraft is under it. */
int ak_nav_above_ceiling(const ak_nav_t *nav, int32_t alt_mm);

/* The altitude the navigator will hold: the one captured when it engaged, or
 * the floor, whichever is higher. */
int32_t ak_nav_hold_altitude(const ak_nav_t *nav);

/* Distance in metres and bearing in hundredths of a degree, 0 being north.
 * Equirectangular: a metre or so of error at a kilometre, which is far below
 * what a loiter radius of a few tens of metres cares about. */
void ak_nav_distance_bearing(int32_t from_lat_e7, int32_t from_lon_e7,
                             int32_t to_lat_e7, int32_t to_lon_e7,
                             int32_t *distance_m, int32_t *bearing_e2);

/* The shortest signed difference between two courses, in degrees, wrapped to
 * -180..180. The wrap is the whole reason this is a function: getting it wrong
 * turns a ten degree error into a 350 degree one and the aircraft away from
 * home. */
float ak_nav_course_error_e2(int32_t wanted_e2, int32_t actual_e5);

/* One step of guidance. Takes the fix, which the caller has already checked is
 * valid, and produces what the sticks would have said. Returns 1 while
 * returning, 0 when it is not engaged. */
int ak_nav_step(ak_nav_t *nav, const ak_nav_input_t *in, ak_rc_command_t *out);

#endif /* AK_FLIGHT_AK_NAV_H */
