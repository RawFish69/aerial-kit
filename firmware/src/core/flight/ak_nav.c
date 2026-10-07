#include "ak_nav.h"

#include <string.h>

#include "ak_math.h"

#define AK_NAV_EARTH_RADIUS_M 6371000.0f

/* The vertical loop's default integral gain, in throttle per (m/s)-second of
 * climb-rate error. The reasoning is beside the loop itself, in
 * quad_throttle(). */
#define AK_QUAD_ALT_VEL_KI_DEFAULT 0.10f
/* What a metre a second of commanded rate is allowed to be worth in standing
 * throttle. A little above the proportional gain, so a steady rate can be held
 * on the integral alone with the error back at zero. */
#define AK_QUAD_CLIMB_I_PER_M_S 0.12f

void ak_nav_init(ak_nav_t *nav)
{
    /* Every field, including the ones a later change adds: a navigator that
     * starts with whatever was on the stack is a navigator that flies
     * whatever was on the stack. */
    memset(nav, 0, sizeof *nav);
    nav->course_kp = 0.025f;  /* 40 degrees of error is a full roll command */
    nav->alt_kp = 0.01f;      /* 100 metres of error is a full pitch command */
    nav->alt_ki = 0.0008f;    /* 20 m of error for 20 s is 0.32 of pitch */
    nav->alt_i = 0.0f;
    nav->cruise = 0.55f;
    nav->loiter_roll = 0.35f;
    nav->arrive_m = 60.0f;
    nav->min_alt_mm = 0;
    nav->fence_m = 0.0f;
    nav->fence_ceiling_m = 0.0f;
    nav->fence_enabled = 0;
    nav->profile = AK_NAV_PROFILE_WING;
    nav->quad_kp = 0.5f;     /* 10 m of error asks for 5 m/s */
    nav->quad_speed = 6.0f;  /* m/s, the most it will ask for */
    /* Close enough that the descent finishes over the ground it took off from:
     * see the note on the quadrotor's arrival in quad_step(). */
    nav->quad_arrive_m = 15.0f;
    nav->quad_hover_m = 2.0f;
    nav->quad_alt_ki = AK_QUAD_ALT_VEL_KI_DEFAULT;
    nav->climb_i = 0.0f;
    nav->home_alt_mm = 0;
    nav->rate_next = 0;
    nav->rate_count = 0;
    nav->rate_age_ms = 0;
    nav->climb_m_s = 0.0f;
    for (int i = 0; i < AK_NAV_RATE_SAMPLES; i++) {
        nav->rate_alt_mm[i] = 0;
        nav->rate_time_ms[i] = 0;
    }
    nav->rate_clock_ms = 0;
    nav->hold_n_m_s = 0.0f;
    nav->hold_e_m_s = 0.0f;
    nav->hold_steps = 0;
    nav->descending = 0;
    nav->landing = 0;
    nav->hold_landing = 0;
    nav->no_fix_ms = 0;
    /* And the standing throttle the last return learned is not this one's:
     * the aircraft it was learned on may have been holding nothing at all. */
    nav->climb_i = 0.0f;
    nav->hover_ms = 0;
    nav->last_course_e5 = 0;
    nav->course_yaw_mrad = 0;
    nav->home_lat_e7 = 0;
    nav->home_lon_e7 = 0;
    nav->hold_alt_mm = 0;
    nav->have_home = 0;
    nav->active = 0;
    nav->steps = 0;
    for (int i = 0; i < AK_NAV_WAYPOINTS; i++) {
        nav->waypoint_lat_e7[i] = 0;
        nav->waypoint_lon_e7[i] = 0;
    }
    nav->waypoint_count = 0;
    nav->waypoint_index = 0;
    nav->mission = 0;
    nav->waypoints_reached = 0;
    nav->last_counted_index = -1;
    nav->at_target = 0;
}

void ak_nav_set_profile(ak_nav_t *nav, ak_nav_profile_t profile)
{
    nav->profile = profile;
}

void ak_nav_set_home(ak_nav_t *nav, int32_t lat_e7, int32_t lon_e7,
                     int32_t alt_mm)
{
    nav->home_lat_e7 = lat_e7;
    nav->home_lon_e7 = lon_e7;
    /* The altitude home was set at is the ground the quadrotor's return
     * descends *to* - it is captured while the aircraft is standing on it,
     * which is the only moment anybody knows where the ground is. */
    nav->home_alt_mm = alt_mm;
    nav->have_home = 1;
}

int ak_nav_engage(ak_nav_t *nav, int32_t current_alt_mm)
{
    if (!nav->have_home) {
        return 0;
    }
    nav->hold_alt_mm = current_alt_mm;
    /* The window starts empty, so the first sixth of a second of the return
     * cannot be flown on a rate measured before it began. */
    nav->rate_next = 0;
    nav->rate_count = 0;
    nav->rate_age_ms = 0;
    nav->climb_m_s = 0.0f;
    /* And the wind the last return learned is not this return's wind. */
    nav->hold_n_m_s = 0.0f;
    nav->hold_e_m_s = 0.0f;
    nav->hold_steps = 0;
    nav->descending = 0;
    nav->landing = 0;
    nav->hold_landing = 0;
    nav->no_fix_ms = 0;
    nav->nudge_ms = 0;
    nav->nudge_gave_up = 0;
    /* And the standing pitch the last flight learned is not this one's: a wing
     * that learned it carrying a heavy pack, or in a thermal, hands it back. */
    nav->alt_i = 0.0f;
    nav->hover_ms = 0;
    nav->active = 1;
    nav->steps = 0;
    return 1;
}

void ak_nav_disengage(ak_nav_t *nav)
{
    nav->active = 0;
    /* A landing in place is a descent the navigator was flying, so handing the
     * aircraft back ends it: if the navigator is engaged again it starts from
     * the top, including the wait before anything comes down. */
    nav->hold_landing = 0;
    nav->no_fix_ms = 0;
    nav->climb_i = 0.0f;
    nav->nudge_ms = 0;
    nav->nudge_gave_up = 0;
    nav->alt_i = 0.0f;
}

void ak_nav_set_waypoint(ak_nav_t *nav, int index, int32_t lat_e7,
                         int32_t lon_e7)
{
    if (index < 0 || index >= AK_NAV_WAYPOINTS) {
        return;
    }
    nav->waypoint_lat_e7[index] = lat_e7;
    nav->waypoint_lon_e7[index] = lon_e7;
}

void ak_nav_set_waypoint_count(ak_nav_t *nav, int count)
{
    if (count < 0) {
        count = 0;
    }
    if (count > AK_NAV_WAYPOINTS) {
        count = AK_NAV_WAYPOINTS;
    }
    nav->waypoint_count = count;
}

int ak_nav_start_mission(ak_nav_t *nav, int32_t current_alt_mm)
{
    if (nav->waypoint_count <= 0 || !nav->have_home) {
        return 0;
    }
    /* Same capture as a return: the altitude the pilot was flying at is the
     * altitude the navigator holds, because a mission is not a reason to
     * change height. */
    nav->hold_alt_mm = current_alt_mm;
    nav->active = 1;
    nav->mission = 1;
    nav->waypoint_index = 0;
    nav->waypoints_reached = 0;
    nav->last_counted_index = -1;
    nav->at_target = 0;
    nav->steps = 0;
    return 1;
}

void ak_nav_stop_mission(ak_nav_t *nav)
{
    nav->mission = 0;
    nav->active = 0;
    nav->waypoint_index = 0;
}

int ak_nav_mission_active(const ak_nav_t *nav)
{
    return nav->mission && nav->active;
}

int ak_nav_waypoint_index(const ak_nav_t *nav)
{
    return nav->mission && nav->waypoint_index < nav->waypoint_count
               ? nav->waypoint_index
               : -1;
}

void ak_nav_distance_bearing(int32_t from_lat_e7, int32_t from_lon_e7,
                             int32_t to_lat_e7, int32_t to_lon_e7,
                             int32_t *distance_m, int32_t *bearing_e2)
{
    /*
     * The longitude difference is taken the short way round, and in 64 bits.
     *
     * Both halves are needed and neither is enough. Longitude is carried in
     * units of 1e-7 degrees, so a coordinate reaches 1.8e9 and the difference
     * between two of them reaches 3.6e9 - which is past INT32_MAX, so the
     * subtraction itself is the bug, before any wrap is considered. Wrapping
     * it into -180..180 degrees afterwards is what makes the answer the
     * *shorter* of the two ways round: two points either side of the
     * antimeridian are metres apart, not most of the planet apart.
     *
     * Latitude does not wrap and cannot overflow: its difference is at most
     * 1.8e9, which fits.
     */
    int64_t dlon_e7 = (int64_t)to_lon_e7 - (int64_t)from_lon_e7;
    if (dlon_e7 > 1800000000LL) {
        dlon_e7 -= 3600000000LL;
    } else if (dlon_e7 < -1800000000LL) {
        dlon_e7 += 3600000000LL;
    }

    float dlat_rad = ak_deg2rad((float)(to_lat_e7 - from_lat_e7) * 1e-7f);
    float dlon_rad = ak_deg2rad((float)dlon_e7 * 1e-7f);
    float mean_lat_rad =
        ak_deg2rad((float)((from_lat_e7 + to_lat_e7) / 2) * 1e-7f);

    float north = dlat_rad * AK_NAV_EARTH_RADIUS_M;
    float east = dlon_rad * ak_cosf(mean_lat_rad) * AK_NAV_EARTH_RADIUS_M;

    if (distance_m != 0) {
        *distance_m = (int32_t)(ak_sqrtf(north * north + east * east) + 0.5f);
    }
    if (bearing_e2 != 0) {
        /* atan2(east, north): zero is north and east is ninety, which is what a
         * compass bearing means. */
        *bearing_e2 = (int32_t)(ak_rad2deg(ak_atan2f(east, north)) * 100.0f);
    }
}

float ak_nav_course_error_e2(int32_t wanted_e2, int32_t actual_e5)
{
    float wanted_deg = (float)wanted_e2 * 0.01f;
    float actual_deg = (float)actual_e5 * 0.00001f;
    float difference_rad = ak_deg2rad(wanted_deg - actual_deg);

    /* Wrapping through sin and cos rather than with branches: atan2 of the
     * unit vector is the angle in -180..180, and it cannot be got wrong at the
     * boundary the way a pair of if statements can. */
    return ak_rad2deg(ak_atan2f(ak_sinf(difference_rad), ak_cosf(difference_rad)));
}

/*
 * The altitude error arrives in millimetres and alt_kp is per metre, which is
 * what its name, its help text and the parameter table all say it is.
 *
 * Multiplying the millimetres by it - which is what this did - made the gain a
 * thousand times too high: a hundred millimetres of error was a full pitch
 * command, so the altitude loop was not a loop at all but a bang-bang
 * controller, and the return-to-home pitched between full nose-up and full
 * nose-down once a second for the whole flight. Nothing caught it because the
 * unit test only asked which way the pitch went.
 */
/*
 * The wing's height, as a pitch: the error asks for a nose-up, and the
 * standing part of it is *learned*.
 *
 * A proportional law alone holds an altitude below the one it was given,
 * because an aircraft sinks and the pitch that stops the sink has to come from
 * somewhere - and the only place a proportional law can get it is the error
 * itself. Measured in the simulator: the wing's return held 14 m where it was
 * told to hold 15, and a fence that asked it to climb to a 30 m floor settled
 * at 26. Both are the same standing error, and both are what the integral
 * removes: the error goes to zero and the standing pitch stays.
 *
 * The bound is a fifth of full pitch, and the integration stops while the
 * output is pinned and the error is still pushing it - the aircraft being held
 * up by the end of the stick is not the aircraft that needs more pitch. Those
 * are the same two rules the quadrotor's vertical loop needed, for the same
 * reasons, and the wing's version is slower because a wing's height is a
 * phugoid: an integral fast enough to trim a quadrotor would pump a wing's
 * slow oscillation instead of damping it.
 */
#define AK_WING_ALT_I_MAX 0.20f

/* How far under a fence ceiling the navigator holds: see ak_nav_hold_altitude()
 * for what happens without it. INAV's geozone default, in metres. */
#define AK_NAV_CEILING_MARGIN_M 10.0f

static float hold_altitude(ak_nav_t *nav, int32_t alt_mm, float dt_s)
{
    float error_m =
        (float)(ak_nav_hold_altitude(nav) - alt_mm) / 1000.0f;
    float i_next = nav->alt_i;
    float out;

    if (dt_s > 0.0f) {
        i_next = ak_clampf(i_next + nav->alt_ki * error_m * dt_s,
                           -AK_WING_ALT_I_MAX, AK_WING_ALT_I_MAX);
    }
    out = error_m * nav->alt_kp + i_next;

    if ((out > 1.0f && error_m > 0.0f) || (out < -1.0f && error_m < 0.0f)) {
        i_next = nav->alt_i;
        out = error_m * nav->alt_kp + i_next;
    }
    nav->alt_i = i_next;
    return ak_clampf(out, -1.0f, 1.0f);
}

int32_t ak_nav_hold_altitude(const ak_nav_t *nav)
{
    int32_t alt_mm = nav->hold_alt_mm > nav->min_alt_mm ? nav->hold_alt_mm
                                                       : nav->min_alt_mm;
    /*
     * And the lid, under the floor in this ordering because the ceiling is the
     * one that is not negotiable: a floor is terrain the aircraft can fly over,
     * a ceiling is airspace it may not be in. A pilot who has set the two
     * across each other - a floor above the ceiling - has asked for something
     * impossible, and the answer that keeps the aircraft where it is allowed to
     * be is to hold the lower number.
     *
     * The height it holds is the ceiling *less a margin*, and that is not
     * decoration. Holding exactly the number the trigger fires at is a knife
     * edge: the navigator takes the aircraft over the line, holds it there, and
     * every metre the loop spends above its own target is another breach - take
     * over, hand back, take over, at the rate the vertical loop oscillates.
     * Measured in the fence session before this was here: the pilot and the
     * navigator swapped control four times in twenty seconds, which is the
     * hazard the fence's own comment warns about for a ring smaller than a
     * wing's turn. INAV's geozones carry the same idea as
     * `geozone_safe_altitude_distance` - "vertical distance that must be
     * maintained to the upper and lower limits of the zone", ten metres by
     * default - and this is that distance.
     */
    if (nav->fence_enabled && nav->fence_ceiling_m > 0.0f && nav->have_home) {
        int32_t ceiling_mm =
            nav->home_alt_mm + (int32_t)(nav->fence_ceiling_m * 1000.0f);
        int32_t margin_mm = (int32_t)(AK_NAV_CEILING_MARGIN_M * 1000.0f);
        /*
         * The margin comes out of the room the pilot has left between the floor
         * and the ceiling, so a floor close under a ceiling is still the floor:
         * at a floor five metres under the lid the margin is five metres, and
         * the aircraft holds the floor. A pilot who has put the floor *above*
         * the ceiling has asked for something impossible - there is no room at
         * all - and the ceiling wins, because that is the one it is not allowed
         * to cross.
         */
        int32_t room_mm = ceiling_mm - nav->min_alt_mm;
        if (margin_mm > room_mm) {
            margin_mm = room_mm;
        }
        if (margin_mm < 0) {
            margin_mm = 0;
        }
        if (alt_mm > ceiling_mm - margin_mm) {
            alt_mm = ceiling_mm - margin_mm;
        }
        /* And never below the ground it took off from: a lid so low that the
         * margin would put the hold underground is a lid the aircraft cannot be
         * under, and the answer to that is the ground rather than a descent
         * through it. */
        if (alt_mm < nav->home_alt_mm) {
            alt_mm = nav->home_alt_mm;
        }
    }
    return alt_mm;
}

int32_t ak_nav_distance_home(const ak_nav_t *nav, int32_t lat_e7,
                             int32_t lon_e7)
{
    if (!nav->have_home) {
        return -1;
    }
    int32_t distance_m = 0;
    ak_nav_distance_bearing(lat_e7, lon_e7, nav->home_lat_e7, nav->home_lon_e7,
                            &distance_m, 0);
    return distance_m;
}

int ak_nav_outside_fence(const ak_nav_t *nav, int32_t lat_e7, int32_t lon_e7)
{
    if (!nav->fence_enabled || nav->fence_m <= 0.0f || !nav->have_home) {
        return 0;
    }
    int32_t distance_m = ak_nav_distance_home(nav, lat_e7, lon_e7);
    return distance_m > (int32_t)nav->fence_m;
}

/*
 * The fence's other side, and the reason it is not just a cap on the altitude
 * the navigator holds: a cap only matters to an aircraft the navigator is
 * already flying. A pilot flying by hand through the ceiling is the same
 * problem as a pilot flying by hand through the radius - somebody else has to
 * take the aircraft - so this is checked the way the radius is, against the
 * home altitude, and it triggers the same return.
 */
int ak_nav_above_ceiling(const ak_nav_t *nav, int32_t alt_mm)
{
    if (!nav->fence_enabled || nav->fence_ceiling_m <= 0.0f || !nav->have_home) {
        return 0;
    }
    int32_t ceiling_mm =
        nav->home_alt_mm + (int32_t)(nav->fence_ceiling_m * 1000.0f);
    return alt_mm > ceiling_mm;
}

/* --- the fixed wing's profile ---------------------------------------------
 *
 * Roll steers toward home on the course error; pitch holds the altitude that
 * was captured when the return was engaged. When it arrives the caller
 * overrides the roll with the loiter bank, because a wing cannot stop.
 */
static void wing_step(ak_nav_t *nav, const ak_nav_input_t *in,
                      int32_t bearing_e2, ak_rc_command_t *out)
{
    float course_error_deg = ak_nav_course_error_e2(bearing_e2, in->course_e5);

    out->roll = ak_clampf(course_error_deg * nav->course_kp, -1.0f, 1.0f);
    out->pitch = hold_altitude(nav, in->alt_mm, in->dt_s);
    out->yaw = 0.0f;
    out->throttle = nav->cruise;
    out->angle_mode = 1;
    out->arm_request = 0; /* a navigator does not decide whether to be armed */
}

/* --- the quadrotor's profile ----------------------------------------------
 *
 * Climb, translate, settle. Three things are different from the wing, and each
 * of them is a property of the airframe rather than a preference:
 *
 * - **A tilt is an acceleration.** A quad has no elevator, so the profile
 *   commands a *velocity* and lets the tilt follow from the difference between
 *   that and the velocity the GPS reports. The damping in that difference is
 *   the half that makes it stable: a position-only loop flies at the target and
 *   past it, and then the aircraft is somewhere else with the error pointing
 *   back the way it came.
 * - **The throttle is the vertical control.** The wing holds height with
 *   pitch; a quad holds it with throttle, around the hover throttle the pilot
 *   configured (`cruise`). Descending to the hover height above home is the
 *   same law with a different target, so there is no separate descent mode to
 *   get wrong.
 * - **The yaw has to mean something.** A world-frame position error becomes a
 *   body-frame tilt through the heading, and this aircraft has no magnetometer:
 *   the heading comes from the GPS ground track
 *   (ak_estimator_aid_heading), which is only good while the aircraft is
 *   moving. Until that has happened the profile climbs and holds - the part of
 *   the return that is safe without a heading - rather than translating
 *   confidently in the wrong direction.
 *
 * The last two metres of the descent are deliberately the pilot's: the return
 * descends to `quad_hover_m` above the ground home was set on and holds there.
 * A landing the aircraft cannot see - no rangefinder, no landing detector, and
 * a barometer that measures the weather - is worth less than a hover somebody
 * can take over from, and auto-land is the next step rather than this one.
 */
#define AK_QUAD_CLIMB_KP 0.4f  /* (m/s) of commanded climb per metre of error */
#define AK_QUAD_CLIMB_MAX 3.0f /* m/s: the fastest it will ask to go up or down */
#define AK_QUAD_ALT_VEL_KP 0.08f /* throttle per (m/s) of climb-rate error */
/*
 * The standing throttle the vertical loop learns, and the most it may learn.
 *
 * The loop above it is proportional, and a proportional loop settles *where the
 * plant needs the throttle the error happens to supply* - not where it was
 * asked to be. Measured in the loop, that is not a small difference: the
 * descent an aircraft with no position flies settles at about **0.3 m/s where
 * the profile asks for 0.6**, because the throttle it takes to hold six tenths
 * of a metre a second down is twice what a 0.3 m/s rate error asks for. The
 * fix is the same one the position loop already has: integrate the error that
 * is left and let the integral hold it.
 *
 * What it may hold is bounded *by what it is being asked for*, and that is the
 * second thing this loop taught: a fixed bound is a bound the loop can sit on.
 * With a quarter of a hover throttle allowed, the integral wound up during the
 * fast part of a return - three metres a second down from altitude - and then
 * took seconds to unwind when the profile changed its mind, carrying the
 * aircraft straight through the hover height it was supposed to settle at and
 * into the ground. The tape is the evidence: 3.1, 2.7, 2.4, 2.1, 1.6, 1.0,
 * 0.3, with no hover in it at all.
 *
 * So the standing throttle may never be worth more than the rate being asked
 * for, at `AK_QUAD_CLIMB_I_PER_M_S` of throttle per m/s. A hover is
 * then a state with nothing to learn - the throttle that holds a hover is the
 * hover throttle, and the integral is brought back to zero the moment the
 * demand for a *rate* goes away - and a fast descent may learn what a fast
 * descent needs and must give it back when the demand falls. The proportional
 * term pays for the rest, with a temporary error it forgets.
 *
 * The integration also stops while the output is pinned at an end *and the
 * error is pushing it further* - an aircraft held up by the end of the stick is
 * not an aircraft held up by a rate it has not reached, and an integral that
 * learns otherwise hands back a throttle nobody asked for.
 */
#define AK_QUAD_VEL_KP 2.0f    /* (m/s^2) per (m/s) of velocity error */
#define AK_QUAD_MAX_ACCEL 5.0f /* m/s^2, about 27 degrees of tilt */
/* The wind the position loop learns, as a standing addition to the velocity it
 * asks for: how fast it learns, and the most it may ask for. */
#define AK_QUAD_HOLD_KI 0.05f  /* (m/s) per metre-second of position error */
#define AK_QUAD_HOLD_MAX 3.0f  /* m/s - half of quad_speed, near enough */
#define AK_QUAD_CLIMB_BAND_MM 2000 /* start moving within this of the return altitude */
#define AK_QUAD_COURSE_MIN_MS 1.0f /* below this the module's course means nothing */
/*
 * The nudge that earns a heading.
 *
 * Nothing on this aircraft measures which way the nose points. The only
 * heading there is comes from the track the aircraft makes through the air,
 * and a stationary aircraft does not make one - so a quadrotor told to fly to
 * a waypoint from a standstill in still air has *no way to start*: the
 * navigator will not translate without a heading, and it cannot earn a heading
 * without translating. Measured in the simulator: `mission quad calm` hovered
 * on the spot at 221 m from its first waypoint for two minutes, with the yaw
 * estimate frozen 105 degrees from the nose.
 *
 * So when it needs a heading and does not have one, the navigator makes a
 * track: a quarter of a stick of forward tilt, held until the module's course
 * means something - 3.5 m/s, above the gate the estimator uses - and then
 * thrown away in favour of the steering. The aircraft ends up a few metres
 * from where it started, which is the price of knowing which way it is facing,
 * and the cap is there because a nudge that never earns a heading is an
 * aircraft flying away from its mission for ever: after the cap it holds and
 * says so.
 */
#define AK_QUAD_NUDGE_TILT 0.35f
#define AK_QUAD_NUDGE_SPEED 3.0f
#define AK_QUAD_NUDGE_MS 8000u
#define AK_QUAD_G 9.80665f
/* The hover, before the landing: how close to the hover height counts as being
 * there, how long it has to stay there, and how fast the last descent goes. */
#define AK_QUAD_HOVER_BAND_MM 1000
#define AK_QUAD_HOVER_MS 2000u
#define AK_QUAD_LAND_CLIMB 0.6f /* m/s, capping the descent near the ground */
#define AK_QUAD_LAND_BAND_MM 3000 /* below this the cap applies */
/* The yaw rate asked for, as a fraction of full yaw stick per degree of course
 * error: 50 degrees out is a full-stick turn. */
#define AK_QUAD_YAW_KP 0.02f
/* The most of the yaw stick the navigator will use, and the reason is in the
 * comment at the course law: a heading measured from the track has to be
 * earned slowly enough for the track to mean the nose. */
#define AK_QUAD_YAW_LIMIT 0.25f

static int32_t quad_target_altitude(const ak_nav_t *nav)
{
    if (nav->landing) {
        return nav->home_alt_mm; /* the ground the take-off was from */
    }
    if (nav->descending) {
        return nav->home_alt_mm + (int32_t)(nav->quad_hover_m * 1000.0f);
    }
    return ak_nav_hold_altitude(nav);
}

/*
 * The throttle that flies the climb rate the profile asked for.
 *
 * One function because the two places that fly a quadrotor vertically - the
 * return's descent and the hold it does when it has lost its position - must
 * not drift apart: the integral is state, and two copies of a stateful loop
 * are two aircraft.
 */
static float quad_throttle(ak_nav_t *nav, float desired_climb, float dt_s)
{
    float error = desired_climb - nav->climb_m_s;
    float i_next = nav->climb_i + nav->quad_alt_ki * error * dt_s;
    float i_limit = AK_QUAD_CLIMB_I_PER_M_S *
                    (desired_climb < 0.0f ? -desired_climb : desired_climb);
    float out;

    i_next = ak_clampf(i_next, -i_limit, i_limit);
    out = nav->cruise + AK_QUAD_ALT_VEL_KP * error + i_next;

    /* Conditional integration: the integral is left where it is while the
     * throttle is pinned and the error is pushing it further into the end. */
    if ((out > 1.0f && error > 0.0f) || (out < 0.0f && error < 0.0f)) {
        i_next = nav->climb_i;
        out = nav->cruise + AK_QUAD_ALT_VEL_KP * error + i_next;
    }
    nav->climb_i = i_next;
    return ak_clampf(out, 0.0f, 1.0f);
}

/*
 * How fast the aircraft is climbing, over a window rather than between two
 * steps.
 *
 * The vertical loop is a rate loop - the throttle follows the *difference*
 * between the climb rate it asks for and the one it has - so a rate that is
 * wrong is not a small error, it is the wrong command. And the obvious way to
 * get one is badly wrong on this aircraft: the height arrives from the
 * barometer at about 32 Hz as an integer number of millimetres, so between
 * two consecutive passes of a 1 kHz loop it is *unchanged* six passes out of
 * seven and then jumps by 33 mm.
 *
 * Differencing that gives an aircraft that is not climbing, not climbing, not
 * climbing, and then climbing at 33 metres a second. The loop cannot average
 * that out, because its response is clamped: on the six quiet passes the
 * commanded rate is compared against zero and the throttle is railed down,
 * and the one loud pass is one millisecond long. Measured in the simulator
 * with the return at its despatch altitude, the loop *believed it was not
 * descending* while the aircraft descended at 9.9 metres a second: the
 * commanded three metres a second was never reached because the feedback
 * always said "you are not going down yet", so it went down harder. That is
 * the dive the landing tape shows reaching the ground.
 *
 * Eight samples 25 ms apart make a 175 ms window: five or six barometer
 * updates, so the staircase averages into a rate with a 0.19 m/s quantum, and
 * a lag of under a tenth of a second - a fraction of the loop's own time
 * constant, which is what keeps the phase margin. The window is why the
 * measurement is in the navigator rather than in the barometer driver: it is
 * a control input, not a sensor reading.
 */
static void quad_climb_sample(ak_nav_t *nav, int32_t alt_mm, float dt_s)
{
    unsigned oldest;
    float seconds;

    if (dt_s <= 0.0f) {
        return;
    }
    nav->rate_age_ms += (unsigned)(dt_s * 1000.0f + 0.5f);
    nav->rate_clock_ms += (unsigned)(dt_s * 1000.0f + 0.5f);
    if (nav->rate_count > 0u && nav->rate_age_ms < AK_NAV_RATE_STEP_MS) {
        return;
    }
    nav->rate_age_ms = 0;
    nav->rate_alt_mm[nav->rate_next] = alt_mm;
    nav->rate_time_ms[nav->rate_next] = nav->rate_clock_ms;
    nav->rate_next = (nav->rate_next + 1u) % AK_NAV_RATE_SAMPLES;
    if (nav->rate_count < AK_NAV_RATE_SAMPLES) {
        nav->rate_count++;
    }
    if (nav->rate_count < 2u) {
        return;
    }

    oldest = (nav->rate_next + AK_NAV_RATE_SAMPLES - nav->rate_count) %
             AK_NAV_RATE_SAMPLES;
    seconds = (float)(nav->rate_clock_ms - nav->rate_time_ms[oldest]) /
              1000.0f;
    if (seconds <= 0.0f) {
        return;
    }
    nav->climb_m_s = (float)(alt_mm - nav->rate_alt_mm[oldest]) / 1000.0f /
                     seconds;
}

static void quad_step(ak_nav_t *nav, const ak_nav_input_t *in,
                      int32_t distance_m, int32_t bearing_e2, int settle,
                      ak_rc_command_t *out)
{
    float alt_error_m;

    /*
     * `settle` is the quadrotor's own arrival, `quad_arrive_m` from home, and
     * it is deliberately not the wing's `rth_arrive_m`.
     *
     * The descent is the part of a return that has to happen *somewhere*, and
     * the somewhere is under the aircraft. A wing's arrival radius is the
     * radius of the circle it can hold while it waits, so it is tens of
     * metres: sixty is a comfortable orbit. A quadrotor's arrival is a
     * vertical descent, so its radius is how far from home it can still be
     * descending - and starting that descent at a wing's sixty metres is
     * *not* a flight-quality preference. Measured in the loop before this
     * was a number of its own: the descent is capped at three metres a second
     * and home was sixty metres away, so the aircraft had seven seconds of
     * height against twenty-one seconds of distance, reached the ground
     * thirty-eight metres downwind of home, and flew the last of the way
     * there with a few tenths of a metre of altitude under it. On a real
     * airframe that is a drag across whatever lies between the two points.
     *
     * The gate does not have to be where the aircraft stops, because the
     * horizontal loop never stops flying it home: arriving changes the
     * *altitude* target (there is ground under it now) and the velocity loop
     * keeps asking for zero velocity over the point. Start the descent close
     * enough that it finishes over the ground the take-off was from.
     */
    if (settle) {
        nav->descending = 1;
    }

    /*
     * The hover first, then the landing. Two seconds at the hover height is
     * what "settled" means, and it is deliberately a decision the navigator
     * makes rather than something a pilot has to be there for.
     */
    if (nav->descending && !nav->landing) {
        int32_t hover_mm = nav->home_alt_mm +
                           (int32_t)(nav->quad_hover_m * 1000.0f);
        int32_t off = in->alt_mm - hover_mm;

        if (off < 0) {
            off = -off;
        }
        if (off <= AK_QUAD_HOVER_BAND_MM && in->dt_s > 0.0f) {
            nav->hover_ms += (uint32_t)(in->dt_s * 1000.0f);
            if (nav->hover_ms >= AK_QUAD_HOVER_MS) {
                nav->landing = 1;
            }
        } else {
            /* Not where it was told to be: the clock starts again rather than
             * accumulating a hover that never happened. */
            nav->hover_ms = 0;
        }
    }

    /*
     * Which height the loop flies on, for the last part of a landing.
     *
     * The barometric estimate is what the whole return is flown on, and it is
     * the wrong instrument for the last metre: it is a pressure difference
     * anchored once on the ground, so a leak of a few metres over a flight is
     * invisible to it and is exactly the size of the distance being flown. A
     * rangefinder pointed down measures that distance directly, so once the
     * landing has begun and there is a reading, the target is the ground and
     * the height is the measurement of the ground: the loop keeps asking the
     * aircraft down until the part says it has arrived, whatever the estimate
     * believes. The rate the loop damps with is still the barometer's, because
     * a differentiation of a noisy short-range part is a worse rate than the
     * one the window already produces.
     *
     * Where the two disagree, this is the *landing* and not the range that
     * decides anything: the flight core will not stop the motors until the
     * part and the estimate agree (see landing_detector in main.c), so a part
     * reading the roof of a car cannot end the flight - and a descent that is
     * carried a metre too low by the same part is a descent onto the ground it
     * was aiming at.
     */
    int32_t measured_alt = in->alt_mm;
    if (nav->landing && in->agl_valid) {
        measured_alt = nav->home_alt_mm + in->agl_mm;
    }

    alt_error_m = (float)(quad_target_altitude(nav) - measured_alt) / 1000.0f;
    /* The vertical loop is a *rate* loop, not a throttle-from-error one: an
     * error straight into throttle is a spring with nothing to damp it, and
     * the flight this was written against oscillated 40 metres either side of
     * the altitude it was asked to hold. The rate comes from the height, since
     * the GPS gives a ground speed and nothing vertical. */
    float desired_climb = ak_clampf(AK_QUAD_CLIMB_KP * alt_error_m,
                                    -AK_QUAD_CLIMB_MAX, AK_QUAD_CLIMB_MAX);

    /* The last few metres are slow: a return is allowed to be unhurried, and
     * three metres a second into the ground is not a landing, it is an
     * arrival. The approach to the hover height above it is left to the
     * proportional law, which is measured not to overshoot it: the tape has
     * the aircraft reaching 3.0 m above home, asked for 2.5, on the way in. */
    if (nav->landing &&
        (in->alt_mm - nav->home_alt_mm) < AK_QUAD_LAND_BAND_MM) {
        desired_climb = ak_clampf(desired_climb, -AK_QUAD_LAND_CLIMB, 0.0f);
    }

    out->roll = 0.0f;
    out->pitch = 0.0f;
    out->yaw = 0.0f;
    out->angle_mode = 1;
    out->arm_request = 0;
    /* The rate the loop flies on is measured over a window, not between this
     * step and the last one: see quad_climb_sample(). */
    quad_climb_sample(nav, in->alt_mm, in->dt_s);
    out->throttle = quad_throttle(nav, desired_climb, in->dt_s);

    /*
     * Which way the aircraft is travelling, and which way it should be.
     *
     * The heading comes from the GPS ground track - the direction the aircraft
     * is *travelling*, which equals the way it is pointing only while it is
     * flying forwards. So the return also *steers*, with the same course law
     * the wing uses, aimed by the yaw rate: the nose turns toward home, the
     * tilt follows the nose because the tilt is computed in the body frame,
     * the velocity follows the tilt, and the track follows the velocity. The
     * aircraft ends up flying at the target nose-first.
     *
     * That convergence is what makes the track a heading measurement the
     * estimator can trust, and it is not optional: a version of this that only
     * translated - pointing the nose wherever it happened to point - made a
     * track at right angles to the nose, the estimator aligned its yaw to that
     * track, and the next tilt was computed in a frame that had just been
     * turned underneath it. The flown scenario showed that as an aircraft
     * spinning on the spot while travelling away from home at nine metres a
     * second, which no amount of arithmetic on a bench had shown.
     */
    {
        int32_t course_e5 = in->course_e5;
        float course_error_deg;

        if ((float)in->speed_mm_s / 1000.0f >= AK_QUAD_COURSE_MIN_MS) {
            nav->last_course_e5 = course_e5;
            nav->course_yaw_mrad = in->yaw_mrad;
        } else if (nav->last_course_e5 != 0) {
            /* Hovering, so the module has no course to report. What the
             * aircraft is pointing at now is where it was pointing when the
             * course was measured, plus however far it has turned since - and
             * the gyro knows that, which is the one thing it is good at. */
            int32_t turned_mrad = in->yaw_mrad - nav->course_yaw_mrad;

            course_e5 = nav->last_course_e5 +
                        (int32_t)((float)turned_mrad * 5729.578f);
        }

        course_error_deg = ak_nav_course_error_e2(bearing_e2, course_e5);
        out->yaw = ak_clampf(course_error_deg * AK_QUAD_YAW_KP, -1.0f, 1.0f);
        /* Gently. The only heading this aircraft has is the track it is
         * making, so a hard turn is a turn whose heading measurement lags
         * behind the nose by the turn rate times the time the airframe takes
         * to change velocity - and the tilt is computed in that lagging frame.
         * Measured: turning at the full stick rate, the estimate ended up 160
         * degrees from the nose and the aircraft flew a circle two hundred
         * metres across, away from where it was going. */
        out->yaw = ak_clampf(out->yaw, -AK_QUAD_YAW_LIMIT, AK_QUAD_YAW_LIMIT);
    }

    /* Climbing first: a return that steers while it is still low is a return
     * that steers into the ground.
     */
    if (in->alt_mm < quad_target_altitude(nav) - AK_QUAD_CLIMB_BAND_MM) {
        return;
    }

    /*
     * And then a heading worth using - or one worth earning.
     *
     * The aircraft cannot translate without knowing which way its nose points,
     * and it cannot know that without moving (see the nudge above). While it
     * is already moving, the module has a course and the estimator is pulling
     * the yaw onto it: the right thing to do is to stop and let that converge,
     * which is what the first half of this does. While it is *not* moving,
     * nothing will ever converge, and the second half makes a track to learn
     * from.
     *
     * Note what is *not* here: an arrival does not stop the horizontal loop.
     * Stopping the command the moment the aircraft is inside the arrival
     * radius leaves it coasting - it has velocity and nothing is asking for
     * any - so it drifts out of the radius, gets a command back, and flies a
     * limit cycle around home forever. Arriving changes the *altitude* target
     * (there is ground under it now); the velocity loop keeps asking for zero
     * velocity, which is what holding a station is. */
    if (!in->yaw_aligned) {
        float ground_m_s = (float)in->speed_mm_s / 1000.0f;

        /* Straight, whether it is shoving or coasting out the shove. The
         * course law above is aimed by a heading nobody has, so a yaw command
         * here is a turn toward somewhere the aircraft cannot know - and while
         * the heading is being earned it is worse than useless, because a
         * turning aircraft's track is not its nose and the alignment then
         * chases a target that moves with it. Measured: the shove reached
         * three metres a second, the steering took over before the two seconds
         * of track the estimator needs had passed, and the estimate ended up
         * seventy degrees out with the aircraft wandering. */
        out->yaw = 0.0f;

        if (ground_m_s < AK_QUAD_NUDGE_SPEED && nav->nudge_ms < AK_QUAD_NUDGE_MS) {
            /* The shove is a command and the timer is an accumulation, so only
             * the timer is gated on there having been an interval. The two were
             * one condition, and the first step of a return has no interval by
             * construction - see nav_dt() - which would have postponed the
             * shove by a step for no reason the manoeuvre has. */
            if (in->dt_s > 0.0f) {
                nav->nudge_ms += (uint32_t)(in->dt_s * 1000.0f + 0.5f);
            }
            out->pitch = -AK_QUAD_NUDGE_TILT;
        } else if (nav->nudge_ms >= AK_QUAD_NUDGE_MS) {
            /* It has tried, and the aircraft is not learning which way it
             * points: hold where it is rather than flying on into whatever
             * comes next. The console says so. */
            nav->nudge_gave_up = 1;
        }
        return;
    }

    {
        float yaw_rad = (float)in->yaw_mrad / 1000.0f;
        float bearing_rad = ak_deg2rad((float)bearing_e2 / 100.0f);
        float north_m = (float)distance_m * ak_cosf(bearing_rad);
        float east_m = (float)distance_m * ak_sinf(bearing_rad);
        float speed_m_s = (float)in->speed_mm_s / 1000.0f;
        float vn = 0.0f;
        float ve = 0.0f;
        float tilt = in->max_tilt_rad > 0.001f ? in->max_tilt_rad : 0.5f;
        float vd_n;
        float vd_e;
        float an;
        float ae;
        float a_forward;
        float a_right;

        /*
         * The wind, learned.
         *
         * A tilt accelerates the aircraft through the *air*; the GPS reports
         * it over the *ground*. Holding a station in a steady wind therefore
         * needs a standing tilt, and a proportional loop can only get one
         * from a standing position error: with the gains here, five metres a
         * second of wind costs about two and a half metres of offset, which
         * is the difference between landing on the pad and landing beside it.
         *
         * So the loop integrates its own position error into a standing
         * addition to the velocity it asks for - the average difference
         * between motion through the air and motion over the ground, which is
         * the wind - and the error it needs to hold station goes to nothing
         * as that addition learns. It is conditioned on not being saturated:
         * on the way home the loop is asking for all the speed it has, the
         * error is large and *staying* large is what a constant wind does, so
         * integrating there would wind the term up to its limit and hand the
         * arrival a correction larger than the wind.
         */
        if (in->dt_s > 0.0f) {
            float want_n = nav->quad_kp * north_m + nav->hold_n_m_s;
            float want_e = nav->quad_kp * east_m + nav->hold_e_m_s;

            if (want_n < nav->quad_speed && want_n > -nav->quad_speed &&
                want_e < nav->quad_speed && want_e > -nav->quad_speed) {
                nav->hold_n_m_s = ak_clampf(
                    nav->hold_n_m_s + AK_QUAD_HOLD_KI * north_m * in->dt_s,
                    -AK_QUAD_HOLD_MAX, AK_QUAD_HOLD_MAX);
                nav->hold_e_m_s = ak_clampf(
                    nav->hold_e_m_s + AK_QUAD_HOLD_KI * east_m * in->dt_s,
                    -AK_QUAD_HOLD_MAX, AK_QUAD_HOLD_MAX);
            }
        }
        vd_n = ak_clampf(nav->quad_kp * north_m + nav->hold_n_m_s,
                         -nav->quad_speed, nav->quad_speed);
        vd_e = ak_clampf(nav->quad_kp * east_m + nav->hold_e_m_s,
                         -nav->quad_speed, nav->quad_speed);

        if (speed_m_s >= AK_QUAD_COURSE_MIN_MS) {
            float course_rad = ak_deg2rad((float)in->course_e5 / 100000.0f);

            vn = speed_m_s * ak_cosf(course_rad);
            ve = speed_m_s * ak_sinf(course_rad);
        }

        an = ak_clampf(AK_QUAD_VEL_KP * (vd_n - vn), -AK_QUAD_MAX_ACCEL,
                       AK_QUAD_MAX_ACCEL);
        ae = ak_clampf(AK_QUAD_VEL_KP * (vd_e - ve), -AK_QUAD_MAX_ACCEL,
                       AK_QUAD_MAX_ACCEL);

        /* The body frame: the nose points along (cos yaw, sin yaw) in
         * (north, east), because both the yaw estimate and the module's course
         * are measured from north toward east. The right-hand axis is that
         * vector turned a quarter turn clockwise. */
        a_forward = an * ak_cosf(yaw_rad) + ae * ak_sinf(yaw_rad);
        a_right = -an * ak_sinf(yaw_rad) + ae * ak_cosf(yaw_rad);

        /* A tilt is an acceleration: the horizontal part of the thrust is g
         * times the angle, near enough for the angles this flies at. A
         * positive pitch command is nose *up*, which accelerates backwards, so
         * forward carries a minus; a positive roll command is right side down,
         * which accelerates right. Dividing by the angle a full stick asks for
         * puts the command in the units the angle loop and the mixers speak. */
        out->pitch = ak_clampf(-(a_forward / AK_QUAD_G) / tilt, -1.0f, 1.0f);
        out->roll = ak_clampf((a_right / AK_QUAD_G) / tilt, -1.0f, 1.0f);
    }
}

/*
 * Flying with no position fix.
 *
 * A return exists because the link is gone, and the one measurement it cannot
 * do without is the position - so the worst thing this firmware can be handed
 * is a return whose GPS module stops answering. What it did before this was
 * written was to give up: the navigator disengaged, the flight core saw a
 * lost link with nobody flying, and did the correct thing for a lost link,
 * which is stop the motors. Measured in the loop, the aircraft fell out of
 * twenty metres onto the ground it had been flying home to.
 *
 * The alternative is not navigation, because there is nothing to navigate
 * with: it is *holding*, with the two measurements that still work. The
 * attitude estimate is the gyro and the accelerometer, and it is unaffected;
 * the height is the barometer, and it is unaffected. So the aircraft flies
 * level and holds the altitude it was told to hold, which for the wing means
 * straight on at cruise and for the quadrotor means a hover that drifts with
 * the wind. Both are better than a deliberate motor stop in the air, and both
 * are things the pilot can see: the mode stays RTH or `on autopilot`, and the
 * console counts the steps it has been holding for.
 *
 * A wing in that state flies until the battery is gone, and that is the honest
 * answer for one: it cannot land without a position, and where it would land
 * is not somewhere anybody chose. A quadrotor can, and that is what
 * `hold_land_s` is for: after that many seconds of holding with no fix, *and
 * only with a rangefinder fitted that is answering*, it comes down where it is
 * - a slow descent onto whatever is underneath, flown on the part for the last
 * two metres and stopped by the same rule that stops any other landing. The
 * alternative it replaces is a hover until the pack is flat, which ends with
 * the aircraft on the ground anyway, without anybody's hand on the throttle
 * and without knowing where.
 *
 * It is off by default and it is the pilot's switch, for the reason the fence
 * and the battery return are: it takes an aircraft that is still flying and
 * commits it to coming down. The rangefinder requirement is not decoration
 * either - without something measuring the ground the descent would be flown
 * blind on an estimate that is the thing that failed, so without a part this
 * is a hover, exactly as before.
 */
static void hold_step(ak_nav_t *nav, const ak_nav_input_t *in,
                      ak_rc_command_t *out)
{
    nav->hold_steps++;

    out->roll = 0.0f;
    out->yaw = 0.0f;
    out->angle_mode = 1;
    out->arm_request = 0; /* a navigator does not decide whether to be armed */

    if (nav->profile == AK_NAV_PROFILE_QUAD) {
        /* The quadrotor's vertical control is its throttle, and the loop it
         * already has - a rate loop around the hover throttle - works on the
         * barometer alone. */
        int32_t target_alt = ak_nav_hold_altitude(nav);
        int32_t measured_alt = in->alt_mm;
        float alt_error_m;
        float desired_climb;

        if (in->dt_s > 0.0f) {
            nav->no_fix_ms += (uint32_t)(in->dt_s * 1000.0f + 0.5f);
        }
        if (!nav->hold_landing && nav->hold_land_s > 0u && in->range_fitted &&
            nav->no_fix_ms >= nav->hold_land_s * 1000u) {
            nav->hold_landing = 1;
        }
        if (nav->hold_landing) {
            /* Down onto the ground below. The target is the ground the
             * take-off was from - and where the part is reading, the height
             * measuring it is the part, so the error is "however far the
             * ground still is" whatever the terrain does underneath. */
            target_alt = nav->home_alt_mm;
            if (in->agl_valid) {
                measured_alt = nav->home_alt_mm + in->agl_mm;
            }
        }

        alt_error_m = (float)(target_alt - measured_alt) / 1000.0f;
        desired_climb =
            ak_clampf(AK_QUAD_CLIMB_KP * alt_error_m,
                      nav->hold_landing ? -AK_QUAD_LAND_CLIMB
                                        : -AK_QUAD_CLIMB_MAX,
                      AK_QUAD_CLIMB_MAX);

        quad_climb_sample(nav, in->alt_mm, in->dt_s);
        out->pitch = 0.0f;
        out->throttle = quad_throttle(nav, desired_climb, in->dt_s);
        return;
    }

    /* The wing holds height with pitch and flies on at cruise throttle: no
     * roll, which is as straight as it can hold without a course to fly. */
    out->pitch = hold_altitude(nav, in->alt_mm, in->dt_s);
    out->throttle = nav->cruise;
}

/*
 * The interval this step is allowed to have, which is not always the interval
 * the caller reported. See AK_NAV_MAX_DT_MS for what the bound is and why.
 *
 * Two things are taken out of the caller's number here, and both of them are
 * intervals the navigator did not fly:
 *
 *   - The first step of an engagement is worth nothing, whatever the caller
 *     says. The caller's interval is measured from *its* last step, and the
 *     navigator was disengaged for all of it - `main.c` returns before it
 *     computes the interval while there is no fix and nobody is coming home,
 *     so the clock it is handed spans the whole time the pilot was flying.
 *     There is no predecessor in this engagement, so there is no interval:
 *     the same answer the flight core gives a sample with no predecessor, and
 *     for the same reason - inventing one would be a claim about a step that
 *     did not happen. `steps` is zeroed by ak_nav_engage() and not by anything
 *     else, which is what makes it the right thing to ask.
 *
 *   - Anything past the bound. What is left over is *counted* rather than
 *     quietly dropped, so the console can say a loop ran short instead of
 *     showing a guidance loop that is simply behaving oddly.
 *
 * A zero, a negative, or a NaN all come out as zero rather than being passed
 * on. The negative and the NaN are not intervals either, and the NaN is the
 * one that used to reach the arithmetic: every comparison against it is false,
 * so `quad_climb_sample`'s `dt_s <= 0.0f` guard let it through to a cast.
 */
static float nav_dt(ak_nav_t *nav, float dt_s)
{
    if (!(dt_s > 0.0f)) {
        return 0.0f;
    }
    if (nav->steps == 0u) {
        return 0.0f;
    }
    if (dt_s > AK_NAV_MAX_DT_S) {
        nav->dt_clipped_ms += (uint32_t)((dt_s - AK_NAV_MAX_DT_S) * 1000.0f +
                                         0.5f);
        return AK_NAV_MAX_DT_S;
    }
    return dt_s;
}

int ak_nav_step(ak_nav_t *nav, const ak_nav_input_t *in, ak_rc_command_t *out)
{
    ak_nav_input_t bounded;
    int32_t target_lat;
    int32_t target_lon;
    int32_t distance_m = 0;
    int32_t bearing_e2 = 0;
    int flying_to_waypoint;
    int arrived;

    if (!nav->active || !nav->have_home) {
        return 0;
    }

    /*
     * Every integrator below reads `in->dt_s`, and there are a lot of them -
     * the wing's altitude loop, the quadrotor's throttle loop, its climb-rate
     * window and its station-holding wind - so the interval is bounded once,
     * here, on a copy, rather than trusted at each of them. `nav_dt()` asks
     * `nav->steps`, so it runs before the step is counted.
     */
    bounded = *in;
    bounded.dt_s = nav_dt(nav, in->dt_s);
    in = &bounded;

    nav->steps++;

    /* No fix, and already flying: hold rather than stop. Everything below
     * this needs a position, so this is a different manoeuvre, not a special
     * case of one. */
    if (!in->gps_valid) {
        hold_step(nav, in, out);
        return 1;
    }

    /* Where it is going: the waypoint it is on, or home. A mission is the same
     * steering with a target that moves when it is reached, which is the whole
     * reason the return was written as guidance and not as a manoeuvre - and it
     * is why both profiles take a target rather than knowing about home. */
    target_lat = nav->home_lat_e7;
    target_lon = nav->home_lon_e7;
    flying_to_waypoint = nav->mission && nav->waypoint_index >= 0 &&
                         nav->waypoint_index < nav->waypoint_count;
    if (flying_to_waypoint) {
        target_lat = nav->waypoint_lat_e7[nav->waypoint_index];
        target_lon = nav->waypoint_lon_e7[nav->waypoint_index];
    }

    ak_nav_distance_bearing(in->lat_e7, in->lon_e7, target_lat, target_lon,
                            &distance_m, &bearing_e2);

    /* Close enough. On a mission that means "next one", and past the end of
     * the list it means the same thing a return does: stop trying to reach a
     * point and stay there - the wing circles it, the quad settles over it.
     *
     * The arrival is counted on the way in rather than every step: an aircraft
     * circling a waypoint is at it once, and a counter that climbs while it
     * circles is a counter nobody can use to tell whether a mission worked.
     *
     * How close that is, is the airframe's answer, and it is the same one for
     * a waypoint as for home: a wing's arrival radius is the circle it can
     * hold, tens of metres, and a quadrotor's is the distance from which it
     * can still stop at a point. Measured on the same waypoint list: the wing
     * passes 31 m from its first waypoint, and the quadrotor with the wing's
     * sixty metres "arrived" 45 m out and cut the corner, with its own fifteen
     * it arrives 2 m out. */
    arrived = distance_m <= (int32_t)(nav->profile == AK_NAV_PROFILE_QUAD
                                          ? nav->quad_arrive_m
                                          : nav->arrive_m);

    if (nav->profile == AK_NAV_PROFILE_QUAD) {
        /*
         * The quadrotor comes down over home, so arriving *home* is what opens
         * the gate the descent runs through - and a waypoint is not home.
         *
         * That is not a detail: the altitude it descends to is the altitude
         * home was captured at, so a quadrotor that treated a waypoint as an
         * arrival would come down onto whatever is under the waypoint at
         * home's ground level, and a mission that ended at a waypoint would
         * end with a landing nobody asked for. A waypoint is a point to arrive
         * at; the last one is where it holds station until somebody says
         * otherwise. See quad_step().
         */
        int settle = arrived && !flying_to_waypoint;

        quad_step(nav, in, distance_m, bearing_e2, settle, out);
    } else {
        wing_step(nav, in, bearing_e2, out);
    }

    /* Counted once per waypoint. The rule used to be "on every step that
     * arrives, having not been arrived the step before", which is fine for an
     * aircraft that passes through - and wrong for one that *holds station on
     * the radius*, where drifting a metre out and back counts the same
     * waypoint again. Measured with the quadrotor hovering over its last
     * waypoint: two waypoints, three arrivals. */
    if (nav->mission && flying_to_waypoint && arrived && !nav->at_target &&
        nav->waypoint_index != nav->last_counted_index) {
        nav->waypoints_reached++;
        nav->last_counted_index = nav->waypoint_index;
    }
    nav->at_target = arrived;

    if (flying_to_waypoint && nav->waypoint_index + 1 < nav->waypoint_count) {
        if (arrived) {
            nav->waypoint_index++;
            nav->at_target = 0; /* the next one is somewhere else */
        }
    } else if (arrived && nav->profile == AK_NAV_PROFILE_WING) {
        /* The last waypoint, or home: a wing cannot stop, so it circles. The
         * quadrotor's arrival is the descent its own profile is already
         * flying - there is nothing here to override.
         *
         * **Only the roll.** `wing_step()` above has already computed this
         * step's pitch, from this step's altitude and interval - so computing
         * it again here was not a second opinion, it was a second *step* of
         * the same loop: a circling wing advanced `alt_i` twice per control
         * step and flew its altitude I-gain at twice the gain `alt_ki` names.
         *
         * And the second call's answer is the one that reached the output,
         * because its return value is what this branch assigned - so the
         * commanded pitch came from an integrator that had just been stepped
         * twice as well. Measured on a wing two metres low at 50 Hz, twenty
         * loiter steps, `alt_ki` 0.1: `alt_i` 0.080 with this line gone and
         * 0.160 with it, and the commanded pitch 0.100 against 0.180.
         *
         * The accumulator is the part that carries: `alt_i` is state, and the
         * extra 0.08 is still there at the start of the next step.
         */
        out->roll = nav->loiter_roll;
    }

    return 1;
}
