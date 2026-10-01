/*
 * AerialKit Firmware - the entry point every platform shares.
 *
 * What runs today is a boot path, a flight core with no sensors behind it, and
 * a console. The flight loop is driven with an invalid sensor sample on
 * purpose: with no IMU driver, the honest state for the aircraft to be in is
 * failsafe, and the console says so.
 */

#include "ak_board.h"

#include <string.h>

#include "ak_main.h"
#include "ak_boot.h"
#include "ak_cli.h"
#include "ak_console.h"
#include "ak_fault.h"
#include "ak_flight.h"
#include "ak_launch.h"
#include "ak_mixer.h"
#include "ak_params.h"
#include "ak_selftest.h"
#include "ak_rc_receiver.h"
#include "ak_crsf_telemetry.h"
#include "ak_imu.h"
#include "ak_baro.h"
#include "ak_rangefinder.h"
#include "ak_altitude.h"
#include "ak_battery.h"
#include "ak_align.h"
#include "ak_gyro_cal.h"
#include "ak_accel_cal.h"
#include "ak_log.h"
#include "ak_flashlog.h"
#include "ak_gps.h"
#include "ak_gps_config.h"
#include "ak_nav.h"
#include "ak_proto.h"
#include "ak_console_link.h"
#include "ak_math.h"
#include "ak_text.h"
#include "ak_time.h"
#include "ak_version.h"

#define AK_BOOT_BLINKS     3
#define AK_LED_PERIOD_MS   500u
/* AK_FLIGHT_LOOP_MS is the core's, from ak_flight.h, not this file's. The
 * flight core's control law runs once per ak_flight_step() and is told the
 * interval between calls, so the gate below and the constant the core uses have
 * to be the same number; two #defines with one value each is how they stop
 * being the same number. */
#define AK_HEARTBEAT_MS    10000u
/* AK_DRAIN_QUOTA is the core's, from ak_main.h, not this file's - the
 * simulator's flood session counts the bytes a pass takes and compares them
 * against it, and a check against a copy of a constant is a check against the
 * wrong number. See the comment on the console drain for what it does. */
#define AK_CAL_SAMPLES     500u
/* rad/s a sample may differ from the running mean before the aircraft counts
 * as moving - about three degrees a second, which is a quiet hand. */
#define AK_CAL_MAX_STILL   0.05f
/* And the largest rate that could be a part's offset at all: about twenty
 * degrees a second, the spec limit of the parts this flies. Beyond it the
 * answer is not a calibration but a different sensor. */
#define AK_CAL_MAX_BIAS    0.35f
#define AK_LOG_EVERY       4u    /* 250 Hz out of a 1 kHz loop: 1.5 s of memory */
/* And the long log every tenth of those: 25 Hz, fifteen seconds of history. */
#define AK_LONG_EVERY      10u
/* And the flash log every fifth of a second. A record is twelve words
 * programmed one at a time, so this is the rate at which the writing fits in
 * a 1 kHz loop without the loop noticing; the region is five 128 KB sectors
 * since the configuration took the sixth for a second bank, so it holds a bit
 * under an hour at this rate. */
#define AK_FLASH_EVERY     200u

static ak_flight_t flight;
static ak_param_t  param_items[AK_PARAMS_MAX];
static ak_params_t params;
static ak_cli_t    cli;

/* Defined with the rest of the parameter plumbing, and called from the mission
 * command, which writes waypoints through the table rather than around it. */
static void parameters_changed(void);

static uint32_t    dshot_khz = 300u;
/* Which protocol the receiver speaks. Both are parsed; this says which one the
 * bytes arriving on the receiver's port are. */
static uint32_t    rc_protocol = AK_RC_PROTOCOL_CRSF;
static ak_rc_receiver_t receiver;
/* What the handset is told, and how often it has been asked. The frames go out
 * of the receiver's own transmit line, which is why a board whose protocol is
 * SBUS sends nothing at all: SBUS has no return path. */
static ak_crsf_tlm_t crsf_tlm;
static uint32_t      next_crsf_ms;
static uint32_t      crsf_pings_answered;
static ak_imu_t          imu;
static ak_imu_sample_t   imu_sample;
static int               imu_ok;
static ak_baro_t         baro;
static ak_baro_sample_t  baro_sample;
static int               baro_ok;
/* The pressure where the aircraft was standing, captured while disarmed. What
 * a barometer is good for is the change since take-off: the absolute pressure
 * is the weather. */
static float             baro_reference_pa;
static int               baro_have_reference;
static uint32_t          baro_samples;
static uint32_t          baro_next_ms;
/* How many reads in a row have failed. Kept as a count rather than a flag so
 * the console can print how long the part has been silent for. */
static uint32_t          baro_fails;
#define AK_BARO_PERIOD_MS 30u  /* the part measures at 32 Hz and no faster */
/*
 * The rangefinder: how far the ground is, when the ground is close enough for
 * the answer to matter. A board without one is a board that lands the way it
 * did before there was one - see landing_detector() - so nothing below assumes
 * it is there.
 */
static ak_rangefinder_t  range;
static int               range_ok;
/* Reads in a row that failed, so the console can say how long the part has
 * been silent for rather than only that it is. */
static uint32_t          range_fails;
/* How close to the ground counts as *on* it, and how far the two height
 * measurements may disagree before neither is believed. Both are parameters;
 * these are the values a person would start with. */
static uint32_t          range_land_mm = 400u;
static float             range_agree_m = 3.0f;
/* The flight pack, and how often it is worth asking. Ten times a second is
 * four times faster than the filter this feeds needs, and slow enough that a
 * 23 microsecond conversion is not a line worth counting. */
static ak_battery_t      battery;
static int               battery_ready;
static uint32_t          battery_next_ms;
static uint32_t          battery_last_ms;
#define AK_BATTERY_PERIOD_MS 100u
/* The height the aircraft actually flies on: the barometer's changes, the
 * GPS's absolute reference, and a leak between them. */
static ak_altitude_t     altitude;
static uint32_t          gps_last_alt_time_ms;
static ak_align_t        align;
static ak_gyro_cal_t     gyro_cal;
/*
 * The biasing the flight loop does for itself: a gyro's offset moves with
 * temperature and with the part, so the number somebody measured on a cold
 * bench is not necessarily the number the aircraft is flying with. Arming is
 * the one moment the aircraft is certainly still and certainly about to fly,
 * so that is when the bias is measured again - see the flight loop.
 */
static ak_flight_state_t last_flight_state;
static ak_accel_cal_t    accel_cal;
static ak_rc_cal_t       rc_cal;
static ak_log_t          blackbox;
/* The long log: the same records, a coarser rate, and - on a board with
 * retained RAM - the one that is still there after the reset it is explaining.
 * The fast ring holds 1.5 seconds at 250 Hz for a bench session; this one
 * holds fifteen seconds at 25 Hz for what happened. */
static ak_log_t         *longlog;
static ak_log_t          longlog_fallback;
static int               longlog_retained;
static int               longlog_kept;
/* The third log, and the only one that survives losing the battery: the same
 * records, written into flash at a rate the loop can afford. It stops when it
 * runs out of erased sectors and is given more room on the ground - see
 * ak_flashlog.h, which is where that decision is argued. */
static ak_flashlog_t     flashlog;
static int               flashlog_state;
static ak_gps_t          gps;
static uint32_t          gps_fixes;
static ak_nav_t          nav;
static ak_rc_command_t   nav_command;
static uint32_t          rth_enable; /* off until somebody tests it */
static uint32_t          rth_engagements;
static uint32_t          fence_enable;
static uint32_t          fence_engagements;
/* The pack going flat is the one return trigger that is neither about where the
 * aircraft is nor about whether anybody is talking to it: the link is up, the
 * pilot is flying, and the energy is running out. Off until somebody asks for
 * it, for the same reason the fence is: it takes an aircraft off a pilot. */
static uint32_t          battery_rth;
static uint32_t          battery_returns;
static int               battery_return;   /* latched while it is flying home */
static float             rth_min_alt_m;
/* The mission: a list of places to go, held in the parameter table so a
 * configurator can set it and `save` keeps it, and copied into the navigator
 * whenever anything changes. */
static float             wp_lat_deg[AK_NAV_WAYPOINTS];
static float             wp_lon_deg[AK_NAV_WAYPOINTS];
static uint32_t          wp_count;
static uint32_t          mission_requested;
/*
 * The mission switch has a *level* and an *edge*, and the difference is the
 * whole of a bug that was here: a mission starts when the pilot asks for one -
 * the switch going on, or `mission start` typed at the console - and not merely
 * because the switch is on. The level alone meant that anything which cancelled
 * a mission (a stick, a fence, a pack that has gone critical) was undone on the
 * next pass, because the switch was still on and the navigator saw a fresh
 * request. `mission_console` is the console's own request, which is sticky
 * until it is stopped, and `mission_edge` is "somebody has asked, and the
 * navigator has not started flying it yet".
 */
static int               mission_console;
static int               mission_edge;
static int               mission_switch_on;
/* Which receiver channel is the mission switch, 1-based, or 0 for none. A
 * pilot flying an aircraft should not need a laptop to start a mission, and a
 * switch is also the fastest way to stop one. */
static uint32_t          mission_channel;
static uint32_t          missions_started;
static uint32_t          missions_cancelled;

/*
 * The launch switch, and the aircraft it flies.
 *
 * A *level*, where the mission's switch is an edge, and the difference is which
 * hand is free: a mission is a mode somebody selects and leaves selected, and a
 * launch is two seconds with one hand on the wing and one on the transmitter.
 * The switch that has to be held is the switch that does not need a second
 * flick. The launch module has the manoeuvre - see ak_launch.h - and this is
 * the switch, the words, and the rule that says when it may fly.
 */
static ak_launch_t       launch;
static ak_rc_command_t   launch_command;
static uint32_t          launch_channel;
static uint32_t          launch_timeout_s;
static int               launch_switch_on;
/* "Somebody has asked for a launch, and it has not flown one yet." The switch
 * is a *level* on the transmitter and an *edge* here, and the difference is a
 * bug the simulator found on the first run of it: a launch that ended on its
 * timeout left the switch up, so the next pass started another one, and the
 * aircraft was launched again every five seconds for as long as the switch was
 * held. One request, one launch; cycling the switch asks for another. */
static int               launch_edge;

/* Home is wherever the aircraft was when it last had a fix and was not flying:
 * the take-off point, captured without anyone having to ask. `home` on the
 * console sets it by hand, which is what a bench session wants. */
static int32_t altitude_msl_mm(void);

static int nav_home_set(ak_printf_fn out)
{
    if (!gps.have_fix) {
        out("home: no fix yet, so there is nowhere to remember\n");
        return -1;
    }
    ak_nav_set_home(&nav, gps.fix.lat_e7, gps.fix.lon_e7,
                    altitude_msl_mm());
    out("home: %d.%07d, %d.%07d\n", nav.home_lat_e7 / 10000000,
        nav.home_lat_e7 % 10000000, nav.home_lon_e7 / 10000000,
        nav.home_lon_e7 % 10000000);
    return 0;
}

/* Ask the module to speak UBX and to send NAV-PVT.
 *
 * A u-blox out of the box speaks NMEA at 9600, which this parser cannot read
 * - so a perfectly good module looks like noise until it is asked. The ask is
 * repeated a few times with a pause: a module that is still booting when we
 * first talk will not hear it, and sending the same frame four times costs
 * nothing next to a flight that starts with no GPS. */
static uint32_t gps_config_sends;
static uint32_t gps_config_next_ms;

static void gps_configure(uint32_t now)
{
    if (gps_fixes > 0 || gps_config_sends >= 4u) {
        return; /* it is talking, or it is not going to */
    }
    if ((int32_t)(now - gps_config_next_ms) < 0) {
        return;
    }
    gps_config_next_ms = now + 2000u;

    uint8_t frame[AK_GPS_VALSET_MAX_BYTES];
    unsigned length = ak_gps_config_frame(AK_GPS_MEASUREMENT_MS_DEFAULT, frame,
                                          sizeof frame);
    if (length > 0 && ak_board_gps_send((const char *)frame, length) == 0) {
        gps_config_sends++;
    }
}

static void nav_home_clear(void)
{
    nav.have_home = 0;
}

/*
 * The output test: driving the pins so somebody can look at them.
 *
 * Every other path to the outputs goes through the flight core, which only
 * writes them when it is armed - and arming needs a sensor, a receiver and a
 * converged attitude estimate, so a board on a bench with none of that cannot
 * be made to move a pin at all. That is the wrong way round for the one check a
 * scope exists for.
 *
 * So this walks the outputs: one at a time, a motor to a low idle and back, a
 * servo to half travel each way, then round again. The point is not the value,
 * it is *which* output moves when - which is how M1..M4 and the two servos get
 * identified on a new board, with the props off.
 *
 * Three rules, and they are the safety of it: it refuses to start while armed,
 * it stops by itself if the aircraft stops being disarmed, and it ends with
 * everything at zero rather than wherever the sweep happened to be.
 */
#define AK_OUTPUT_TEST_SLOT_MS 1500u
#define AK_OUTPUT_TEST_MOTOR    0.15f /* enough to turn a motor, not to hurt */
#define AK_OUTPUT_TEST_SERVO    0.5f  /* half travel each way */

static int      output_test_active;
static uint32_t output_test_started_ms;

/* The pilot's plumbing for the servos, which the parameter table points
 * straight at: three numbers per servo, and the defaults are the neutral
 * linkage. See ak_output.h for what each one is for - the reversal is the one
 * that decides whether a wing can turn. */
static ak_servo_trim_t servo_trim[AK_MAX_SERVOS];

static void output_test_outputs(uint32_t now, ak_outputs_t *out)
{
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        out->motor[i] = 0.0f;
    }
    for (int i = 0; i < AK_MAX_SERVOS; i++) {
        out->servo[i] = 0.0f;
    }

    unsigned slots = AK_MAX_MOTORS + AK_MAX_SERVOS;
    uint32_t elapsed = now - output_test_started_ms;
    unsigned slot = (unsigned)(elapsed / AK_OUTPUT_TEST_SLOT_MS);
    uint32_t within = elapsed % AK_OUTPUT_TEST_SLOT_MS;

    /* Round and round until somebody stops it: a person walking to the bench
     * with a scope should not have to catch the one pass. */
    if (slot >= slots) {
        output_test_started_ms = now;
        slot = 0;
        within = 0;
    }

    /* Up, hold, down - a triangle, so the value is never ambiguous. */
    float phase = (float)within / (float)AK_OUTPUT_TEST_SLOT_MS * 3.0f;
    float shape = phase < 1.0f ? phase : (phase < 2.0f ? 1.0f : 3.0f - phase);

    if (slot < AK_MAX_MOTORS) {
        out->motor[slot] = shape * AK_OUTPUT_TEST_MOTOR;
    } else {
        out->servo[slot - AK_MAX_MOTORS] = shape * AK_OUTPUT_TEST_SERVO;
    }
}

/*
 * The height the navigator flies on, in the units it wants: millimetres above
 * mean sea level.
 *
 * The estimate is a height above the take-off point, so it is anchored by
 * adding the GPS altitude that was captured on the ground. A board with no
 * barometer gets the GPS altitude itself, exactly as it always did - the
 * estimator degenerates to the one sensor it has.
 */
static int32_t altitude_msl_mm(void)
{
    if (ak_altitude_have_reference(&altitude) && baro_ok) {
        return (int32_t)((altitude.reference_msl_m +
                          ak_altitude_height_m(&altitude)) *
                         1000.0f);
    }
    return gps.fix.alt_msl_mm;
}

/* Has the pilot moved a stick? A mission is a mode the pilot can take back at
 * any moment, and the fastest, most obvious way to take anything back in an
 * aircraft is to move the controls. The threshold is a fraction of full travel:
 * small enough that anybody who means it crosses it, large enough that a
 * receiver's own jitter does not. */
static int pilot_moved_the_sticks(void)
{
    ak_rc_command_t cmd;
    if (!ak_rc_decode(&receiver.channels, &flight.rc_cfg, &cmd)) {
        return 0;
    }
    const float threshold = 0.15f;
    return ak_absf(cmd.roll) > threshold || ak_absf(cmd.pitch) > threshold ||
           ak_absf(cmd.yaw) > threshold;
}

/* What the navigator is doing, for the console and for the blackbox flags. */
static void nav_update(uint32_t now)
{
    ak_nav_input_t in;

    /*
     * The mission switch, if there is one, is the pilot's control: a mode
     * reachable from the transmitter has to be a mode the transmitter can end,
     * and its *going on* is what asks for a mission - not its being on, which
     * would restart one that something else had just cancelled.
     */
    if (mission_channel >= 1u && mission_channel <= AK_RC_CHANNELS) {
        uint16_t counts = receiver.channels.channel[mission_channel - 1u];
        int on = receiver.channels.valid &&
                 counts > flight.rc_cfg.arm_threshold;

        if (on && !mission_switch_on) {
            mission_edge = 1;
        }
        mission_switch_on = on;
    } else {
        mission_switch_on = 0;
    }
    mission_requested = (mission_switch_on || mission_console) ? 1u : 0u;

    int state = ak_flight_state(&flight);
    int flying = state == AK_FLIGHT_ARMED || state == AK_FLIGHT_MANAGED ||
                 state == AK_FLIGHT_RTH || state == AK_FLIGHT_DESCEND;

    /*
     * The launch switch, read the way the mission's is: the pilot's control,
     * and its *going up* is the request. A switch left up is not a standing
     * order to keep launching the aircraft, which is what the first version of
     * this did - see launch_edge.
     */
    {
        int on = 0;

        if (launch_channel >= 1u && launch_channel <= AK_RC_CHANNELS) {
            uint16_t counts = receiver.channels.channel[launch_channel - 1u];

            on = receiver.channels.valid &&
                 counts > flight.rc_cfg.arm_threshold;
        }
        /*
         * The edge only counts while the pilot is flying the aircraft: a switch
         * flipped on the bench, before arming, is not a launch waiting to
         * happen the moment somebody arms a wing in their hands. Arming first
         * and throwing the switch is INAV's own order too.
         */
        if (on && !launch_switch_on && state == AK_FLIGHT_ARMED) {
            launch_edge = 1;
        }
        launch_switch_on = on;
    }

    /*
     * A launch starts when somebody has asked for one, the pilot is flying the
     * aircraft - not a mission or a return, because the aircraft has to be in
     * somebody's hand - and the attitude it will hold is an attitude that has
     * been measured. It ends the moment the switch goes down or the aircraft
     * stops being the pilot's, which is the same rule the arm switch follows
     * everywhere else and what makes this a mode nobody can leave running.
     */
    if (launch_edge && !ak_launch_active(&launch) &&
        state == AK_FLIGHT_ARMED && flight.est.converged) {
        ak_launch_start(&launch, now);
        launch_edge = 0;
        ak_console_printf("launch: %d degrees of climb at %d per cent - a "
                          "stick gives it back\r\n",
                          (int)(launch.cfg.climb_deg + 0.5f),
                          (int)(launch.cfg.throttle * 100.0f + 0.5f));
    } else if (ak_launch_active(&launch) &&
               (!launch_switch_on ||
                (state != AK_FLIGHT_ARMED && state != AK_FLIGHT_MANAGED))) {
        ak_launch_stop(&launch, AK_LAUNCH_REASON_OFF);
        ak_console_printf("launch: over - %s\r\n",
                          launch_switch_on ? ak_flight_state_name(state)
                                           : "the switch");
    }

    /*
     * A fix that has gone stale stops a return from *starting* - without a
     * position there is nothing to come home to - but it does not stop one
     * that is already flying. Giving up means the flight core sees a lost
     * link with nobody flying and stops the motors, which in the air is not a
     * safe state to hand an aircraft to; the navigator holds instead, level
     * and at the altitude it was told to hold, on the two measurements that
     * still work. See hold_step() in ak_nav.c.
     */
    int gps_ok = ak_gps_fix_valid(&gps, now, 2000u);

    /* The launch is the one thing here that needs no position at all, so a fix
     * that has gone quiet does not end it: see the step below, which is after
     * the fence and the pack have had their say. */
    if (!gps_ok && !(nav.active && flying) && !ak_launch_active(&launch)) {
        ak_nav_disengage(&nav);
        ak_flight_set_guidance(&flight, 0);
        ak_flight_set_managed(&flight, 0);
        return;
    }

    /* What the navigator is told, in one place: a measurement added later is
     * added here rather than at every call. */
    memset(&in, 0, sizeof in);
    in.lat_e7 = gps.fix.lat_e7;
    in.lon_e7 = gps.fix.lon_e7;
    in.alt_mm = altitude_msl_mm();
    in.gps_valid = gps_ok;
    in.course_e5 = gps.fix.course_e5;
    in.speed_mm_s = gps.fix.speed_mm_s;
    /* The heading, and whether it means anything yet: the estimator aligns
     * yaw to the GPS ground track while the aircraft is moving (see
     * ak_estimator_aid_heading), and the quadrotor's return will not translate
     * until it has. */
    in.yaw_mrad = (int32_t)(flight.est.yaw * 1000.0f);
    in.yaw_aligned = flight.est.track_aligned;
    in.max_tilt_rad = ak_deg2rad(flight.cfg.max_tilt_deg);
    /*
     * The ground below, when a rangefinder is measuring it. These three are
     * the navigator's whole knowledge of the ground plane: the distance, the
     * freshness of that distance, and whether a part is answering at all -
     * which out of range is not the same as absent, and is what a descent onto
     * an unknown spot wants to know while it is still high.
     */
    in.agl_valid = range_ok &&
                   ak_rangefinder_valid(&range, now, AK_RANGE_STALE_MS);
    in.agl_mm = in.agl_valid ? range.distance_mm : AK_RANGE_NONE;
    in.range_fitted = range_ok &&
                      (range.samples > 0u || range.out_of_range > 0u);
    /* The navigator differentiates altitude for the quadrotor's climb rate, so
     * it needs the time between steps - which is the flight loop's period, not
     * the nominal one: a loop that ran late is a step that covers more time. */
    {
        static uint32_t nav_last_ms;
        static int nav_have_last;
        /* The loop's own timestamp, not a fresh clock read: the simulator
         * charges for a clock read that comes with no board call behind it (it
         * is modelling a spin-wait), so reading the clock for the fun of it
         * moves the aircraft's sense of time. */

        in.dt_s = nav_have_last ? (float)(now - nav_last_ms) / 1000.0f : 0.0f;
        nav_last_ms = now;
        nav_have_last = 1;
    }

    /*
     * The pack, which is the other trigger that takes an aircraft off a pilot
     * whose link is up. Two things about it are deliberate.
     *
     * It *latches*, and the sticks do not cancel it. A pack sags under load and
     * comes back when the load comes off, and a return that followed the
     * voltage back up would hand the aircraft to a pilot who is watching a
     * flat battery - so once it has started it runs until the flight ends:
     * the aircraft lands, or the arm switch goes off. That is deliberately
     * *not* the rule the mission uses (moving a stick gives a mission back),
     * because the pilot here is already holding a stick: the pack went flat
     * while they were flying it, and "the sticks are off centre" is a
     * description of the flight they were already making, not an instruction
     * to take it back. The scenario sags a pack, brings it back to 12 volts
     * mid-return, and checks that the aircraft does not change its mind.
     *
     * It is the *pilot's* switch, like the fence: `battery_rth` is 0 by
     * default, because a firmware that takes the aircraft off a pilot the
     * first time a cell dips is a firmware nobody will fly. With it off, a
     * critical pack is what it always was here - measured, reported on the
     * console, sent to the handset - and nothing else.
     */
    if (battery_return && (!flying || !battery_rth)) {
        /* The flight is over, or the pilot has turned the behaviour off. */
        battery_return = 0;
    }
    if (battery_rth && !battery_return && flying && gps_ok &&
        battery.state == AK_BATTERY_CRITICAL) {
        battery_return = 1;
        battery_returns++;
        ak_console_printf("battery: %d.%02d V a cell - bringing it home\r\n",
                          (int)battery.volts_per_cell,
                          (int)(battery.volts_per_cell * 100.0f) % 100);
    }

    /*
     * A fence is a return that triggers itself, and it is the one thing here
     * that takes an aircraft away from a pilot whose link is up - so it is a
     * behaviour somebody turns on, and it stops asking once the aircraft is
     * back inside. Enabling it is opting into the return it performs.
     */
    int outside = ak_nav_outside_fence(&nav, gps.fix.lat_e7, gps.fix.lon_e7);
    /*
     * The fence has two ways out of it now, and they are the same kind of
     * event: leaving the ring, and going over the lid. Both are somebody
     * else's decision to make - the navigator's - so both are folded into the
     * one `fence` the rest of this function reasons about, and the console
     * says which one it was where it says anything at all.
     */
    int over_ceiling = ak_nav_above_ceiling(&nav, altitude_msl_mm());
    int fence_breach = outside || over_ceiling;

    /*
     * And a mission is a mode somebody asked for, while a dead pack and a
     * fence the aircraft has left are *reasons* - so they come off a mission
     * too. The version of this that checked the mission first flew the whole
     * waypoint list with a pack that had gone critical and never entered the
     * return: two safety behaviours, one of them documented, and a mode that
     * silently switched both off.
     */
    if (fence_breach || battery_return) {
        /*
         * A failsafe outranks a mission, and *bypasses* it rather than
         * cancelling it: the switch is still on, so a mission block that ran
         * anyway would take the guidance away and hand the aircraft back to
         * sticks that are centred with the throttle up - measured, with the
         * pack going critical under a mission: the aircraft stopped its
         * mission, flew on with nobody flying it and climbed to 27 m, 533 m
         * from home, instead of coming back.
         */
        if (nav.mission) {
            ak_nav_stop_mission(&nav);
            missions_cancelled++;
            ak_console_printf("mission: stopped - %s\r\n",
                              fence_breach ? "the fence" : "the pack");
        }
        /* And the console's request goes with it: the pilot has just been told
         * the mission is over, and a mission that resumes as soon as the
         * aircraft is back inside the fence is a mission that leaves it again.
         * The switch is the pilot's, and cycling it asks for a new one. */
        mission_console = 0;
        /*
         * And a launch, which a failsafe outranks for the same reason and with
         * the same rule: the aircraft has somewhere else to be, so the climb it
         * was given stops being the important thing about it. It is stopped
         * rather than left running, because a launch that outlived a failsafe
         * would be a mode that switched one off - which is the bug this block
         * exists because of.
         */
        if (ak_launch_active(&launch)) {
            ak_launch_stop(&launch, AK_LAUNCH_REASON_OFF);
            ak_console_printf("launch: over - %s\r\n",
                              fence_breach ? "the fence" : "the pack");
        }
    } else if (mission_requested) {
        /*
         * A mission, which is a mode the pilot asked for: it flies with the
         * link up or down, because the navigator is not taking over from
         * anybody - it was handed the aircraft. Moving a stick gives it back.
         */
        if (nav.mission && pilot_moved_the_sticks()) {
            ak_nav_stop_mission(&nav);
            mission_console = 0;
            missions_cancelled++;
            ak_flight_set_guidance(&flight, 0);
            ak_flight_set_managed(&flight, 0);
            return;
        }
        if (!ak_nav_mission_active(&nav)) {
            if (!mission_edge) {
                /* Somebody is asking for a mission that has already been
                 * cancelled, and asking with a switch that has been on since
                 * before: the switch has to be cycled, or `mission start`
                 * typed, before the navigator will fly it again. */
                ak_flight_set_guidance(&flight, 0);
                ak_flight_set_managed(&flight, 0);
                return;
            }
            if (flying && ak_nav_start_mission(&nav, altitude_msl_mm())) {
                missions_started++;
                mission_edge = 0;
            } else {
                ak_flight_set_guidance(&flight, 0);
                ak_flight_set_managed(&flight, 0);
                return;
            }
        }
        if (ak_nav_step(&nav, &in, &nav_command)) {
            ak_flight_set_guidance(&flight, &nav_command);
            ak_flight_set_managed(&flight, 1);
        } else {
            ak_flight_set_guidance(&flight, 0);
            ak_flight_set_managed(&flight, 0);
        }
        return;
    }

    /*
     * The launch, which is a mode the pilot asked for like a mission and is
     * flown the same way - guidance handed to the core with `managed` set, so
     * the navigator is flying an aircraft whose link is up because it was told
     * to. It comes after the failsafes above (so a launch can be taken away by
     * one) and after the mission (so a mission in the air is not interrupted by
     * a switch somebody leaned on).
     *
     * The command it produces is angle mode whatever the pilot's mode switch
     * says: a launch is an attitude to hold, and a rate command in the same two
     * seconds would be an aircraft rotating until somebody stops it.
     */
    if (ak_launch_active(&launch)) {
        ak_rc_command_t out;

        if (ak_launch_step(&launch, &flight.cmd,
                           ak_deg2rad(flight.cfg.max_tilt_deg), now, &out)) {
            launch_command = out;
            ak_flight_set_guidance(&flight, &launch_command);
            ak_flight_set_managed(&flight, 1);
        } else {
            ak_flight_set_guidance(&flight, 0);
            ak_flight_set_managed(&flight, 0);
            ak_console_printf("launch: over - %s\r\n",
                              ak_launch_reason_name(
                                  (ak_launch_reason_t)launch.ended_because));
        }
        return;
    }

    /* The mission was let go: tell the navigator, so the return below starts
     * from nothing rather than from the last waypoint. */
    if (nav.mission) {
        ak_nav_stop_mission(&nav);
    }

    if (!rth_enable && !fence_breach && !battery_return) {
        ak_nav_disengage(&nav);
        ak_flight_set_guidance(&flight, 0);
        ak_flight_set_managed(&flight, 0);
        return;
    }

    if (!nav.active) {
        /* A lost link with a return enabled, a fence the aircraft has flown
         * through, or a pack that is done: either way it is going home. */
        int asked = fence_breach || battery_return ||
                    (!ak_flight_link_live(&flight) && rth_enable);
        if (asked && flying && ak_nav_engage(&nav, altitude_msl_mm())) {
            if (outside) {
                fence_engagements++;
                ak_console_printf("fence: %d m from home, bringing it back\r\n",
                                  (int)ak_nav_distance_home(
                                      &nav, gps.fix.lat_e7, gps.fix.lon_e7));
            } else if (over_ceiling) {
                /* The lid, and the number a pilot needs is how high the
                 * aircraft is - the radius message above says how far out it
                 * is for the same reason. */
                fence_engagements++;
                ak_console_printf(
                    "fence: %d m up, bringing it back\r\n",
                    (int)((altitude_msl_mm() - nav.home_alt_mm) / 1000));
            } else if (battery_return) {
                /* The message was printed when the pack went critical, which
                 * is the moment a person needs to hear about it. */
            } else {
                rth_engagements++;
            }
        } else {
            ak_flight_set_guidance(&flight, 0);
            ak_flight_set_managed(&flight, 0);
            return;
        }
    } else if (!fence_breach && !battery_return &&
               ak_flight_link_live(&flight)) {
        /* The pilot is back. The flight core hands control back on the same
         * rule, and the navigator has to let go on it too: a navigator that
         * stayed engaged would still be holding the altitude it captured on
         * the way out the next time the link went, and the console would go on
         * saying "engaged" while somebody else flew the aircraft. */
        ak_nav_disengage(&nav);
        ak_flight_set_guidance(&flight, 0);
        ak_flight_set_managed(&flight, 0);
        return;
    }

    if (ak_nav_step(&nav, &in, &nav_command)) {
        ak_flight_set_guidance(&flight, &nav_command);
        /* Inside the fence this is a failsafe and the flight core decides who
         * is flying. Outside it, the navigator is flying a link that is still
         * up, and that has to be said in the one way the core understands. */
        ak_flight_set_managed(&flight,
                              (fence_breach || battery_return) ? 1 : 0);
    } else {
        ak_flight_set_guidance(&flight, 0);
        ak_flight_set_managed(&flight, 0);
    }
}

/*
 * The mission, at the console.
 *
 * Waypoints live in the parameter table - so `set wp0_lat 52.1` and `save` work
 * on them like anything else, and a configurator over the protocol can write
 * them without knowing that a mission exists. `mission add` is a convenience
 * for the person at the bench rather than a second kind of storage: it looks up
 * the next slot and sets the same two parameters.
 */
static void mission_report(ak_printf_fn out)
{
    char lat[24];
    char lon[24];
    int flying_at = ak_nav_waypoint_index(&nav);

    out("mission:   %u waypoint%s, %s\n", (unsigned)wp_count,
        wp_count == 1u ? "" : "s",
        ak_nav_mission_active(&nav) ? "flying" : "not flying");

    for (int i = 0; i < (int)wp_count && i < AK_NAV_WAYPOINTS; i++) {
        ak_format_fixed(wp_lat_deg[i], 7, lat, sizeof lat);
        ak_format_fixed(wp_lon_deg[i], 7, lon, sizeof lon);
        out("  [%d] %s, %s%s\n", i, lat, lon,
            i == flying_at ? "  <- flying here" : "");
    }
    if (ak_nav_mission_active(&nav)) {
        out("  %u reached, holding %d mm\n", nav.waypoints_reached,
            nav.hold_alt_mm);
    }
    if (missions_started > 0u || missions_cancelled > 0u) {
        out("  %u started, %u taken back\n", (unsigned)missions_started,
            (unsigned)missions_cancelled);
    }
    if (mission_channel >= 1u) {
        out("  switch on channel %u\n", (unsigned)mission_channel);
    }
}

static int mission_add(ak_printf_fn out, const char *lat_text,
                       const char *lon_text)
{
    float lat = 0.0f;
    float lon = 0.0f;

    if (!ak_parse_float(lat_text, &lat) || !ak_parse_float(lon_text, &lon) ||
        lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f) {
        out("mission: '%s, %s' is not a position\n", lat_text, lon_text);
        return -1;
    }

    unsigned index = wp_count;
    char name[AK_PARAM_NAME_MAX];
    char text[24];
    char message[64];

    ak_strlcpy(name, "wp", sizeof name);
    unsigned used = ak_strlen(name);
    used += ak_format_uint(index, 0, &name[used], sizeof name - used);
    ak_strlcpy(&name[used], "_lat", sizeof name - used);

    ak_format_fixed(lat, 7, text, sizeof text);
    message[0] = '\0';
    if (ak_params_set(&params, name, text, message, sizeof message) != 0) {
        out("mission: %s\n", message);
        return -1;
    }

    name[used] = '\0';
    ak_strlcpy(&name[used], "_lon", sizeof name - used);
    ak_format_fixed(lon, 7, text, sizeof text);
    message[0] = '\0';
    if (ak_params_set(&params, name, text, message, sizeof message) != 0) {
        out("mission: %s\n", message);
        return -1;
    }

    ak_format_uint(index + 1u, 0, text, sizeof text);
    message[0] = '\0';
    if (ak_params_set(&params, "wp_count", text, message, sizeof message) != 0) {
        out("mission: %s\n", message);
        return -1;
    }

    /* The parameters point straight at the fields the navigator is told about,
     * but the navigator is told through apply_parameters - so say it now rather
     * than waiting for somebody to type `set`. */
    parameters_changed();

    ak_format_fixed(wp_lat_deg[index], 7, text, sizeof text);
    out("mission: waypoint %u is %s", index, text);
    ak_format_fixed(wp_lon_deg[index], 7, text, sizeof text);
    out(", %s - 'save' keeps it\n", text);
    return 0;
}

static int mission_command(ak_printf_fn out, int argc, const char *const *argv)
{
    const char *what = argc > 1 ? argv[1] : 0;

    if (what == 0 || ak_str_eq(what, "list")) {
        mission_report(out);
        return 0;
    }

    if (ak_str_eq(what, "start")) {
        if (wp_count == 0u) {
            out("mission: nothing to fly - 'mission add <lat> <lon>' first\n");
            return -1;
        }
        mission_console = 1;
        mission_edge = 1;
        out("mission: flying %u waypoint%s; move a stick to take it back\n",
            (unsigned)wp_count, wp_count == 1u ? "" : "s");
        return 0;
    }

    if (ak_str_eq(what, "stop")) {
        mission_console = 0;
        mission_edge = 0;
        ak_nav_stop_mission(&nav);
        out("mission: stopping\n");
        return 0;
    }

    if (ak_str_eq(what, "add")) {
        if (argc < 4) {
            out("mission: add <lat> <lon>, in degrees\n");
            return -1;
        }
        if (wp_count >= AK_NAV_WAYPOINTS) {
            out("mission: the list holds %u and it is full\n",
                (unsigned)AK_NAV_WAYPOINTS);
            return -1;
        }
        return mission_add(out, argv[2], argv[3]);
    }

    out("mission: what? 'list', 'add <lat> <lon>', 'start', 'stop'\n");
    return -1;
}

static void gps_report(ak_printf_fn out)
{
    ak_gps_report(&gps, out);
    out("uart:      %u bytes dropped by the receive buffer\n",
        ak_board_gps_dropped());
    out("fixes:     %u, valid now: %s\n", gps_fixes,
        ak_gps_fix_valid(&gps, ak_time_ms(), 2000u) ? "yes" : "no");
    out("configure:  %u attempts\n", gps_config_sends);

    if (nav.have_home && gps.have_fix) {
        int32_t distance_m = 0;
        int32_t bearing_e2 = 0;
        ak_nav_distance_bearing(gps.fix.lat_e7, gps.fix.lon_e7, nav.home_lat_e7,
                                nav.home_lon_e7, &distance_m, &bearing_e2);
        out("home:      %d m away, bearing %d.%02d deg\n", distance_m,
            bearing_e2 / 100, bearing_e2 % 100);
        out("return:    %s, %u engagements, holding %d mm",
            nav.active ? "engaged" : "not engaged", rth_engagements,
            ak_nav_hold_altitude(&nav));
        if (nav.min_alt_mm > 0) {
            out(" (floor %d mm)", nav.min_alt_mm);
        }
        /* Which manoeuvre that is, because the two airframes do not return the
         * same way and the console is where a person checks which one is
         * armed. */
        out(" (%s profile)\n",
            nav.profile == AK_NAV_PROFILE_QUAD ? "quadrotor: climb, translate, "
                                                 "settle"
                                               : "fixed wing: bank and circle");
        if (nav.hold_steps > 0u) {
            if (nav.hold_landing) {
                out("held:      %u steps with no fix, and coming down where it "
                    "is - %s\n", nav.hold_steps,
                    range_ok ? "the ground is being measured"
                             : "with nothing measuring the ground");
            } else {
                out("held:      %u steps with no fix, %s - level, altitude "
                    "held\n", nav.hold_steps,
                    ak_gps_fix_valid(&gps, ak_time_ms(), 2000u)
                        ? "and the fix is back"
                        : "still holding");
            }
        }
        if (fence_enable) {
            int outside = ak_nav_outside_fence(&nav, gps.fix.lat_e7,
                                               gps.fix.lon_e7);
            int above = ak_nav_above_ceiling(&nav, altitude_msl_mm());

            out("fence:     %d m, %s\n", (int)nav.fence_m,
                outside ? "outside" : "inside");
            if (nav.fence_ceiling_m > 0.0f) {
                out("ceiling:   %d m above home, %s\n",
                    (int)nav.fence_ceiling_m, above ? "over it" : "under it");
            }
            out("fence:     %u trigger%s\n", (unsigned)fence_engagements,
                fence_engagements == 1u ? "" : "s");
        }
    } else {
        out("home:      not set yet\n");
    }
    out("link:      %s, returning %s\n",
        ak_flight_link_live(&flight) ? "up" : "gone",
        rth_enable ? "enabled" : "disabled");
}

/* One record per loop iteration. Integers in fixed units, because the console
 * formatter has no floating point, and a log that needs floating point to read
 * is a log that is hard to read in the one place it matters. */
static void log_iteration(uint32_t now)
{
    const ak_outputs_t *out = ak_flight_outputs(&flight);
    ak_rc_command_t cmd;
    int decoded = ak_rc_decode(&receiver.channels, &flight.rc_cfg, &cmd);

    ak_log_record_t record;
    record.time_ms = now;

    for (int i = 0; i < 3; i++) {
        record.gyro[i] = (int16_t)(ak_rad2deg(imu_sample.gyro[i]) * 10.0f);
        record.accel[i] = (int16_t)(imu_sample.accel[i] * 1000.0f);
    }
    /* Tenths of a degree, wrapped: the estimate runs on unwrapped and this
     * field is periodic, so the two are not the same number once the aircraft
     * has turned a few times. See ak_attitude_ddeg(). */
    record.attitude[0] = ak_attitude_ddeg(flight.est.roll);
    record.attitude[1] = ak_attitude_ddeg(flight.est.pitch);
    record.yaw = ak_attitude_ddeg(flight.est.yaw);
    /* The height the navigator flies on, above the take-off reference: the
     * number a landing, a return, or the landing rule is judged by. */
    record.alt_mm = (int32_t)(ak_altitude_height_m(&altitude) * 1000.0f);

    record.stick[0] = decoded ? (int16_t)(cmd.roll * 1000.0f) : 0;
    record.stick[1] = decoded ? (int16_t)(cmd.pitch * 1000.0f) : 0;
    record.stick[2] = decoded ? (int16_t)(cmd.yaw * 1000.0f) : 0;
    record.stick[3] = decoded ? (int16_t)(cmd.throttle * 1000.0f) : 0;

    for (int i = 0; i < 3; i++) {
        record.torque[i] = (int8_t)(flight.torque[i] * 100.0f);
    }
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        float motor = out->motor[i] * 254.0f + 0.5f;
        record.motor[i] = motor > 254.0f ? 254u : (uint8_t)motor;
    }

    record.state = (uint8_t)ak_flight_state(&flight);
    record.flags = (uint8_t)((decoded ? AK_LOG_RC_LIVE : 0) |
                             (imu_sample.valid ? AK_LOG_IMU_VALID : 0));
    /*
     * Where it was. The position is logged whether or not a *fix* is behind
     * it - the last one the module sent is still where the aircraft was a
     * moment ago, and after a crash that is the search area - and the flag
     * says which of the two this record holds, because a position from a
     * module that has stopped answering is a place the aircraft *was*.
     */
    record.lat_e7 = gps.fix.lat_e7;
    record.lon_e7 = gps.fix.lon_e7;
    if (ak_gps_fix_valid(&gps, now, 2000u)) {
        record.flags |= AK_LOG_GPS_VALID;
    }

    ak_log_push(&blackbox, &record);
    /* Every tenth of the fast log's records: the long one is about what
     * happened, not about what a tuning pass needs. */
    if ((flight.steps % (AK_LOG_EVERY * AK_LONG_EVERY)) == 0u) {
        ak_log_push(longlog, &record);
    }
    /* And a fifth of that into flash, which is the log that will still be
     * there when the aircraft is picked up out of the field. */
    if ((flight.steps % AK_FLASH_EVERY) == 0u) {
        (void)ak_flashlog_push(&flashlog, &record);
    }
}

static void log_dump(ak_printf_fn out)
{
    ak_log_dump(&blackbox, out);
}

static void log_reset(void)
{
    ak_log_reset(&blackbox);
}

static void longlog_dump(ak_printf_fn out)
{
    ak_log_dump(longlog, out);
}

static void longlog_reset(void)
{
    ak_log_reset(longlog);
}

static void flashlog_dump(ak_printf_fn out)
{
    (void)ak_flashlog_dump(&flashlog, out);
}

static void flashlog_reset(void)
{
    (void)ak_flashlog_clear(&flashlog);
}

/*
 * Does this machine live up to what the firmware believes about it?
 *
 * There are faults a build cannot catch and a host test cannot see, because the
 * port code is not in the host build: a console pointed at the wrong pin, a
 * timer that never started, a tick that does not tick, a configuration sector
 * that is damaged. From the outside they all look the same - the board does
 * nothing - and from the inside they are cheap to check once, at boot.
 *
 * The count is the number of things that mean the firmware is wrong about
 * itself, not the number of parts missing: a board with no IMU is a board with
 * no IMU, and saying so is the point, but it is not a fault.
 */
/*
 * Whether this aircraft would arm, and if not, what is in the way.
 *
 * The flight core decides - ak_flight_arm_check, one list of gates - and this
 * turns the answer into the console's words, because the core does no I/O. It
 * is asked from three places: `status` while disarmed, the end of the preflight
 * report, and the moment a pilot throws the arm switch and the aircraft does
 * not respond. That last one is the reason this exists at all: the firmware
 * used to answer an arm it would not honour with silence, and "it just will
 * not arm" is a bench hour that a single sentence pays for.
 */
static void arm_line(ak_printf_fn out, const char *prefix)
{
    float detail = 0.0f;
    ak_rc_command_t cmd = flight.cmd; /* the last decoded frame, as it stands */
    ak_arm_block_t block = ak_flight_arm_check(&flight, &cmd, &detail);

    out("%s", prefix);
    switch (block) {
    case AK_ARM_OK:
        out("ready\n");
        break;
    case AK_ARM_OUTPUTS:
        /* The one refusal a pilot cannot do anything about from the sticks, so
         * it says both numbers: what the mix needs and what the board has. */
        if (!flight.board_outputs_known) {
            out("refused - the board has not said what its outputs are\n");
        } else {
            out("refused - this airframe's mix needs %u motor%s and %u servo%s, "
                "and the board drives %u and %u\n",
                (unsigned)flight.needed_motors,
                flight.needed_motors == 1u ? "" : "s",
                (unsigned)flight.needed_servos,
                flight.needed_servos == 1u ? "" : "s",
                (unsigned)flight.board_motors, (unsigned)flight.board_servos);
        }
        break;
    case AK_ARM_NO_LINK:
        /* "No receiver" and "a receiver that has stopped" are the same gate and
         * very different bench problems, so the count that tells them apart is
         * printed with the answer. */
        if (receiver.frames == 0u) {
            out("refused - no receiver has spoken\n");
        } else {
            out("refused - the receiver has gone quiet (%u frames)\n",
                receiver.frames);
        }
        break;
    case AK_ARM_FAILSAFE:
        out("refused - a failsafe is latched: the arm switch off, then on\n");
        break;
    case AK_ARM_NOT_REQUESTED:
        out("refused - the arm switch is off\n");
        break;
    case AK_ARM_THROTTLE:
        out("refused - the throttle is at %d per cent, and arming wants %d\n",
            (int)(detail * 100.0f + 0.5f),
            (int)(flight.cfg.throttle_low * 100.0f + 0.5f));
        break;
    case AK_ARM_NOT_CONVERGED:
        out("refused - the attitude estimate has not seen the accelerometer\n");
        break;
    case AK_ARM_NOT_LEVEL:
        out("refused - the aircraft is %d degrees from level, and arming "
            "wants %d\n",
            (int)(detail + 0.5f), (int)(flight.cfg.arm_max_tilt_deg + 0.5f));
        break;
    default:
        out("refused\n");
        break;
    }
}

/* Is the aircraft on the ground, for the one command that needs to know
 * (`save`, whose erase stalls the loop). See `ak_cli_io_t.disarmed`. */
static int cli_disarmed(void)
{
    return ak_flight_state(&flight) == AK_FLIGHT_DISARMED;
}

/* The shape `status` prints. */
static void arm_report(ak_printf_fn out)
{
    arm_line(out, "arm:       ");
}

/*
 * Say it once per attempt, in the loop.
 *
 * Once per *attempt* rather than once per pass, because this is a message for a
 * person holding a transmitter, and a line every millisecond is not a message.
 * The rule is "the switch is asking and the answer is different from the one
 * somebody was last given", so a pilot who fixes the throttle and is then told
 * about the tilt hears both, and a pilot who has already been told to lower the
 * throttle is not told again.
 */
static void arm_announce(void)
{
    static ak_arm_block_t last = AK_ARM_OK;
    float detail = 0.0f;

    /*
     * Only while it is disarmed, which is the only state the gates are about:
     * once the aircraft is flying, a throttle that is up is not a refusal, and
     * a console that says "refused" about an aircraft that is armed is a
     * console whose words cannot be trusted.
     */
    if (ak_flight_state(&flight) != AK_FLIGHT_DISARMED) {
        last = AK_ARM_OK;
        return;
    }
    if (!flight.cmd.arm_request) {
        /* The switch went off: the next attempt is a new attempt. */
        last = AK_ARM_OK;
        return;
    }

    ak_arm_block_t block = ak_flight_arm_check(&flight, &flight.cmd, &detail);

    if (block != AK_ARM_OK && block != last) {
        arm_line(ak_console_printf, "arm:       ");
    }
    last = block;
}

/* Why there is no inertial sensor, so that the question can be asked after the
 * boot - the open's own line is printed once, before a host can attach, and on
 * the F405 that line has never been recoverable. See ak_imu_last_result().
 *
 * `label` is the column the caller is printing in, because the same sentence is
 * wanted under `imu` and under the preflight and the two do not share a prefix.
 */
static void imu_absence(ak_printf_fn out, const char *label)
{
    switch (ak_imu_last_result()) {
    case AK_IMU_UNKNOWN_PART:
        out("%sanswered 0x%02x, which is not a known part\n", label,
            ak_imu_last_whoami());
        break;
    case AK_IMU_NO_CONFIG:
        out("%sa known part answered but would not configure\n", label);
        break;
    case AK_IMU_NOBODY:
    default:
        out("%snothing answered on the bus\n", label);
        break;
    }
}

static int preflight_run(ak_printf_fn out, int verbose)
{
    int problems = 0;

    if (ak_board_console_attached_port() != ak_board_console_port()) {
        problems++;
        if (verbose) {
            out("FAIL  console is on 0x%08x, but the board's console is 0x%08x\n",
                ak_board_console_attached_port(), ak_board_console_port());
        }
    } else if (verbose) {
        out("ok    console attached to the board's own port\n");
    }

    if (!ak_board_clock_ok()) {
        problems++;
        if (verbose) {
            /* No vendor's initials in the message: HSE/HSI are the STM32's
             * names for the crystal and the internal clock, and this part's are
             * HEXT and HICK. The board's own clock summary says which of its
             * sources is in use; this line says what went wrong. */
            out("FAIL  the clock is not on its crystal: the board fell back to "
                "the internal clock\n");
        }
    } else if (verbose) {
        out("ok    clock on the crystal, system %u MHz, apb1 %u MHz\n",
            ak_board_clock_sysclk_hz() / 1000000u,
            ak_board_clock_apb1_hz() / 1000000u);
    }

    /* The tick has to advance while we stand here, or every delay, timeout and
     * failsafe in this firmware is measuring nothing. */
    uint32_t t0 = ak_time_ms();
    uint32_t spins = 0;
    while (ak_time_ms() == t0 && spins < 5000000u) {
        spins++;
    }
    if (ak_time_ms() == t0) {
        problems++;
        if (verbose) {
            out("FAIL  the millisecond tick is not running (%u delay%s have "
                "given up waiting for it)\n",
                ak_delay_stalls(), ak_delay_stalls() == 1u ? "" : "s");
        }
    } else if (verbose) {
        out("ok    tick advanced after %u spins\n", spins);
    }

    if (!ak_board_output_ready()) {
        if (verbose) {
            /* A fact about the board rather than about the aircraft, and the
             * one that makes the mix-versus-board line below fail: the timers
             * this board's outputs need are not up, so it states nothing it
             * can drive. */
            out("--    outputs: none on this board - the timers did not come "
                "up\n");
        }
    } else if (ak_board_output_dshot_period() == 0 ||
        ak_board_output_dshot_hz() != dshot_khz * 1000u) {
        problems++;
        if (verbose) {
            out("FAIL  outputs are at %u kHz, %u asked for\n",
                ak_board_output_dshot_hz() / 1000u, dshot_khz);
        }
    } else if (verbose) {
        out("ok    dshot %u kHz, ARR %u, ccr %u/%u, %u frames sent\n",
            ak_board_output_dshot_hz() / 1000u, ak_board_output_dshot_period(),
            ak_board_output_ccr_zero(), ak_board_output_ccr_one(),
            ak_board_output_frames_sent());
    }

    /* Big enough for the largest record the firmware can write, which is what
     * makes this a question whose answer can be trusted: a board reads the
     * whole record or reports a failure, so a buffer smaller than a real
     * configuration turns "present and intact" into "damaged". That is not
     * hypothetical - with 64 bytes here, both real targets answered a saved
     * 1191-byte configuration with a checksum failure on every boot, and the
     * simulator hid it by answering "nothing stored" instead. */
    char config_text[AK_PARAMS_TEXT_MAX + 1];
    int config_length = ak_board_config_read(config_text, sizeof config_text - 1u);
    if (config_length < 0) {
        problems++;
        if (verbose) {
            out("FAIL  the saved configuration is damaged (a bad length, or a "
                "checksum mismatch)\n");
        }
    } else if (verbose) {
        out("ok    saved configuration: %s\n",
            config_length > 0 ? "present and intact" : "none stored");
    }

    if (ak_fault_present()) {
        problems++;
        if (verbose) {
            out("FAIL  a fault is recorded: pc 0x%08x cfsr 0x%08x\n",
                ak_fault.pc, ak_fault.cfsr);
        }
    } else if (verbose) {
        out("ok    no fault recorded since power on\n");
    }

    /*
     * What the aircraft *is*, and what it will therefore do.
     *
     * The `airframe` parameter chooses two things: the mix in the flight core
     * and the return profile in the navigator. They are set together, in one
     * function, so a disagreement should be impossible - which is exactly why
     * it is worth a check: a wing flying a quadrotor's mix, or returning the
     * way a quadrotor does, is a crash with a plausible number in it, and the
     * interface that sets them is the kind that grows a second caller.
     */
    {
        const ak_mixer_t *wanted = ak_mixer_for_airframe(flight.airframe);
        /* The *mix* says which return it needs - a multirotor's or a fixed
         * wing's - rather than the airframe number being tested against a
         * literal here and another one where the navigator is set up. The two
         * disagreed the moment there was a second fixed wing in the table. */
        ak_nav_profile_t wanted_profile = wanted->fixed_wing
                                              ? AK_NAV_PROFILE_WING
                                              : AK_NAV_PROFILE_QUAD;

        if (flight.mixer != wanted || nav.profile != wanted_profile) {
            problems++;
            if (verbose) {
                out("FAIL  airframe %u: flying the %s mixer with a %s return\n",
                    (unsigned)flight.airframe,
                    flight.mixer != 0 ? flight.mixer->name : "no",
                    nav.profile == AK_NAV_PROFILE_QUAD ? "quadrotor"
                                                       : "fixed wing");
            }
        } else if (verbose) {
            out("ok    airframe %u: the %s mix and a %s return\n",
                (unsigned)flight.airframe, wanted->name,
                wanted_profile == AK_NAV_PROFILE_QUAD ? "quadrotor"
                                                      : "fixed wing");
        }
    }

    /*
     * And whether this board can drive what that airframe asks for. The mixer
     * and the outputs are two facts from two places - a parameter and a build -
     * and they meet in exactly one place: this comparison. A wing's mix on a
     * board with no servos is the failure this exists for, and it does not need
     * the aircraft in the air to be a crash: the arming gate refuses, so the
     * first symptom is a refusal with both numbers in it rather than a wing that
     * will not turn.
     */
    {
        int fits = flight.board_outputs_known &&
                   flight.board_motors >= flight.needed_motors &&
                   flight.board_servos >= flight.needed_servos;

        if (!fits) {
            problems++;
            if (verbose) {
                if (!flight.board_outputs_known) {
                    out("FAIL  the board has not said what its outputs are\n");
                } else {
                    out("FAIL  this mix needs %u motors and %u servos; the "
                        "board drives %u and %u\n",
                        (unsigned)flight.needed_motors,
                        (unsigned)flight.needed_servos,
                        (unsigned)flight.board_motors,
                        (unsigned)flight.board_servos);
                }
            }
        } else if (verbose) {
            out("ok    outputs: %u motors, %u servos on this board, and the mix "
                "needs %u and %u\n",
                (unsigned)flight.board_motors, (unsigned)flight.board_servos,
                (unsigned)flight.needed_motors,
                (unsigned)flight.needed_servos);
        }
    }

    /* And the gyro's offset, because the flight is flown on it: either the
     * aircraft measured one for itself at power-up, or the number in the
     * parameters is the one being flown with - which is a different claim. */
    if (verbose) {
        if (gyro_cal.done) {
            float dps[3];

            ak_gyro_cal_bias_dps(&gyro_cal, dps);
            out("--    gyro bias: measured (%u samples): %d %d %d mdps\n",
                gyro_cal.samples, (int)(dps[0] * 1000.0f),
                (int)(dps[1] * 1000.0f), (int)(dps[2] * 1000.0f));
        } else {
            out("--    gyro bias: not measured yet - the stored bias stands\n");
        }
    }

    if (verbose) {
        /* Present or absent, these are facts rather than faults. */
        out("--    imu: %s\n", imu_ok ? imu.driver->name : "none fitted");
        if (!imu_ok) {
            imu_absence(out, "--    imu: ");
        }
        out("--    baro: %s\n",
            baro_ok ? baro.driver->name : "none fitted");
        /* The rangefinder is a fact rather than a fault for the same reason
         * the other sensor lines are: a board without one lands on the
         * barometer, which is what this firmware did before there was one. */
        if (!range_ok) {
            out("--    rangefinder: none fitted\n");
        } else if (range.distance_mm >= 0) {
            out("--    rangefinder: %s, the ground is %d.%02d m below\n",
                range.driver->name, (int)(range.distance_mm / 1000),
                (int)((range.distance_mm % 1000) / 10));
        } else {
            out("--    rangefinder: %s, nothing in range\n",
                range.driver->name);
        }
        out("--    altitude: %s\n",
            !baro_ok ? "gps only"
                     : (altitude.have_gps_reference ? "barometer, gps anchored"
                                                    : "barometer, no gps anchor yet"));
        /* A fact rather than a fault, for the same reason the sensor lines
         * are: the count above it is the number of things that mean the
         * firmware is wrong about *itself*, and a flat pack is not one of
         * those. It is worth a line anyway - a pilot reading this before a
         * flight wants to see the number that says whether to bother. */
        if (!battery_ready) {
            out("--    battery: none fitted on this board\n");
        } else if (battery.samples == 0u) {
            out("--    battery: no reading yet\n");
        } else if (battery.state == AK_BATTERY_ABSENT) {
            out("--    battery: nothing connected\n");
        } else {
            out("--    battery: %uS, %d.%02d V, %d.%02d V a cell - %s\n",
                battery.cells, (int)battery.volts,
                (int)(battery.volts * 100.0f) % 100,
                (int)battery.volts_per_cell,
                (int)(battery.volts_per_cell * 100.0f) % 100,
                ak_battery_state_name(battery.state));
        }
        out("--    receiver: %u bytes, %u frames\n", receiver.bytes,
            receiver.frames);
        out("--    gps: %u bytes, %u nav-pvt, %u configure attempts\n", gps.bytes,
            gps.fix_messages, gps_config_sends);
        /*
         * What the fix is, and whether the return the pilot may be trusting
         * would actually work - which is two facts and not one: a fix can be
         * arriving and no good to navigate on, and home is captured from the
         * first fix the aircraft saw on the ground, so an aircraft that armed
         * before it had one has nothing to come back to. A pilot who reads
         * `rth_enable 1` and takes off is trusting a return that does not
         * exist in either of those cases, and the console is where they find
         * out.
         */
        if (!gps.have_fix) {
            out("--    fix: none yet\n");
        } else {
            const char *why = gps.fix.fix_type < AK_GPS_FIX_3D
                                  ? "no height (a 2D fix)"
                              : gps.fix.satellites < gps.min_sats
                                  ? "too few satellites"
                                  : 0;

            out("--    fix: type %u, %u satellites, hacc %u mm - %s\n",
                gps.fix.fix_type, gps.fix.satellites, gps.fix.hacc_mm,
                why == 0 ? "usable" : "not usable by the navigator");
            if (why != 0) {
                out("--          %s\n", why);
            }
        }
        if (!rth_enable) {
            out("--    return: off - a lost link stops the aircraft\n");
        } else if (!nav.have_home) {
            out("--    return: enabled, but no home yet - a lost link would "
                "stop it\n");
        } else if (!ak_gps_fix_valid(&gps, ak_time_ms(), 2000u)) {
            out("--    return: enabled, but the fix is not usable - a lost "
                "link would stop it\n");
        } else {
            out("--    return: ready, home set, %u satellites\n",
                gps.fix.satellites);
        }
        out("--    blackbox: %u records%s\n", blackbox.count,
            blackbox.overwritten > 0 ? " (wrapped)" : "");
        out("--    long log: %u records in %s%s\n", longlog->count,
            longlog_retained ? "retained RAM" : "ordinary RAM",
            longlog_kept ? ", kept from the run before" : "");
        /* The one that is still there after the battery comes out, and - when
         * it has run out of erased sectors - the one that is not recording. */
        if (flashlog_state != AK_FLASHLOG_OK) {
            out("--    flash log: none on this board\n");
        } else {
            out("--    flash log: %u records%s\n",
                ak_flashlog_count(&flashlog),
                flashlog.stopped ? ", full - clears a sector on the ground" : "");
        }
        out("--    link %s, %s\n",
            ak_flight_link_live(&flight) ? "up" : "gone",
            ak_flight_state_name(ak_flight_state(&flight)));
        /* A hand launch is somebody's two hands and a throw, so whether the
         * switch is set at all is worth reading before the aircraft is picked
         * up rather than after the motors stay quiet. */
        if (launch_channel >= 1u && launch_channel <= AK_RC_CHANNELS) {
            out("--    launch: channel %u, %d per cent, %d degrees of climb, "
                "%u s\n",
                (unsigned)launch_channel, (int)(launch.cfg.throttle * 100.0f +
                                                0.5f),
                (int)(launch.cfg.climb_deg + 0.5f), (unsigned)launch_timeout_s);
        } else {
            out("--    launch: off - no channel\n");
        }
        /* And whether the switch, on this aircraft, would actually arm it:
         * the gates are the flight core's, the sentence is this file's, and
         * the two cannot disagree because there is one function behind both. */
        arm_line(out, "--    arm:       ");
    }

    return problems;
}

static int preflight(ak_printf_fn out)
{
    int problems = preflight_run(out, 1);
    out("preflight: %s\n",
        problems == 0 ? "the machine is what the firmware thinks it is"
                      : "SOMETHING IS WRONG - read the FAIL lines above");
    return problems;
}

/* --- the config protocol -------------------------------------------------
 *
 * It shares the console's port, which is what the sync byte is for: while the
 * parser is idle, a byte that is not a frame sync belongs to the console. A
 * tool talking the protocol and a person typing therefore coexist without
 * either knowing about the other, and a tool that sends something malformed
 * hands the port back as soon as its frame is abandoned.
 */
static ak_proto_t    proto;
/* A second parser for the board's network, which is a different link with its
 * own client on it. Sharing one would let two clients' bytes interleave into
 * one frame, which no amount of crc catching makes acceptable. */
static ak_proto_t    net_proto;
static ak_proto_io_t proto_io;
static uint32_t      telemetry_sent;
static uint32_t      next_telemetry_ms;
static int           net_was_connected;

static void proto_status(void *ctx, ak_proto_status_t *out)
{
    (void)ctx;
    const ak_outputs_t *outputs = ak_flight_outputs(&flight);

    out->flight_state = (uint8_t)ak_flight_state(&flight);
    out->link_live = (uint8_t)(ak_flight_link_live(&flight) ? 1 : 0);
    out->gps_fix_type = gps.have_fix ? gps.fix.fix_type : 0;
    out->gps_satellites = gps.have_fix ? gps.fix.satellites : 0;
    /* The same unit the log carries, by the same function, so a heading read
     * off the console and a heading read out of a log cannot come out
     * differently. */
    out->roll_ddeg = ak_attitude_ddeg(flight.est.roll);
    out->pitch_ddeg = ak_attitude_ddeg(flight.est.pitch);
    out->yaw_ddeg = ak_attitude_ddeg(flight.est.yaw);
    out->lat_e7 = gps.have_fix ? gps.fix.lat_e7 : 0;
    out->lon_e7 = gps.have_fix ? gps.fix.lon_e7 : 0;
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        float motor = outputs->motor[i] * 254.0f + 0.5f;
        out->motor[i] = motor > 254.0f ? 254u : (uint8_t)motor;
    }
}

/* The same path the console's `save` takes, so the two cannot drift. */
static int save_parameters(void)
{
    char text[AK_PARAMS_TEXT_MAX];

    /* The same call the console's `save` makes, so the two cannot disagree -
     * which is what they were before B3: two copies of store, bounds, write
     * and mark, agreeing by inspection rather than by construction. All three
     * refusals come back as -1 because this is the protocol's answer and the
     * protocol has one status byte for "no"; the console, which has a screen,
     * tells them apart. (A wire client that wants to know *why* wants a status
     * code that says so, which is a wire change and belongs with the
     * cross-repository contract rather than here.) */
    return ak_params_save(&params, text, sizeof text, ak_board_config_write,
                          ak_flight_config_writable(&flight)) < 0 ? -1 : 0;
}

static int proto_save(void *ctx)
{
    (void)ctx;
    return save_parameters();
}

/* Whether the protocol may take a configuration write right now.
 *
 * The same predicate `save_parameters()` hands to the storage, and the same one
 * the reload paths ask, asked here for the same reason: three routes that each
 * decided for themselves when a write is safe would be three answers, and the
 * one arriving over a socket is the one with nobody in the room.
 *
 * It is asked per request. A session on the network link outlives an arming -
 * that is the ordinary case for a ground station - so an answer latched when
 * the client connected would permit a `set` on an aircraft that armed a minute
 * later, and the client's own display, which follows the heartbeats, would say
 * armed while the board accepted the write. */
static int proto_writable(void *ctx)
{
    (void)ctx;
    return ak_flight_config_writable(&flight) ? 1 : 0;
}

/* The receiver, as RC_CHANNELS asks for it.
 *
 * The same two things the console's `rc` prints - the receiver's own view of the
 * link, and the sticks - decoded with `flight.rc_cfg`, which is the
 * configuration the aircraft is flying. A tab that decoded the counts itself
 * from `rc_mid` and `rc_deadband` would be a second implementation of
 * `centred()` in another language, and the two would disagree at the deadband
 * edge with the screen and the airframe each believing its own.
 *
 * `dropped` is the one number here that the receiver cannot know: it is the
 * UART receive buffer's, so it comes from the board and is filled in alongside
 * rather than inside. */
static void proto_rc_state(void *ctx, ak_proto_rc_t *out)
{
    (void)ctx;
    const ak_rc_receiver_t *rx = &receiver;
    uint32_t protocol = ak_rc_receiver_protocol(rx);

    out->protocol = (uint8_t)protocol;
    out->count = (uint8_t)AK_RC_CHANNELS;
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        out->raw[i] = (uint16_t)rx->channels.channel[i];
    }

    out->bytes = rx->bytes;
    out->frames = rx->frames;
    out->crc_errors = rx->crsf.crc_errors;
    out->rejected = protocol == AK_RC_PROTOCOL_SBUS ? rx->sbus.rejected
                                                   : rx->crsf.rejected;
    out->lost = rx->sbus.lost_frames;
    out->failsafe_frames = rx->sbus.failsafe_frames;
    out->dropped = ak_board_rc_dropped();

    if (rx->channels.valid) {
        out->flags |= AK_PROTO_RC_LINK;
    }
    if (protocol == AK_RC_PROTOCOL_SBUS && rx->sbus.failsafe) {
        out->flags |= AK_PROTO_RC_FAILSAFE;
    }
    if (protocol == AK_RC_PROTOCOL_CRSF) {
        out->flags |= AK_PROTO_RC_TELEMETRY;
    }
    if (protocol == AK_RC_PROTOCOL_SBUS && !ak_board_rc_inverted()) {
        out->flags |= AK_PROTO_RC_NO_INVERTER;
    }

    ak_rc_command_t cmd;
    if (!ak_rc_decode(&rx->channels, &flight.rc_cfg, &cmd)) {
        /* No frame, or one the decode would not use - an all-zero frame, or
         * counts that cannot be a stick. The sticks stay zero and the flag
         * stays clear, and those are different claims: zeroed sticks *with*
         * this bit set mean a handset with its sticks centred. */
        return;
    }

    out->flags |= AK_PROTO_RC_DECODED;
    out->sticks[0] = (int16_t)(cmd.roll * 1000.0f);
    out->sticks[1] = (int16_t)(cmd.pitch * 1000.0f);
    out->sticks[2] = (int16_t)(cmd.yaw * 1000.0f);
    out->sticks[3] = (int16_t)(cmd.throttle * 1000.0f);
    if (cmd.arm_request) {
        out->switches |= AK_PROTO_RC_ARM_ON;
    }
    if (cmd.angle_mode) {
        out->switches |= AK_PROTO_RC_ANGLE;
    }
}

static void link_report(ak_printf_fn out)
{
    ak_printf_fn p = out;
    p("links:     %u console, %u net, %u rc, %u gps bytes in one pass at most\n",
      ak_main_drain_max(AK_LINK_CONSOLE), ak_main_drain_max(AK_LINK_NET),
      ak_main_drain_max(AK_LINK_RC), ak_main_drain_max(AK_LINK_GPS));
}

static void proto_report(ak_printf_fn out)
{
    out("console link:\n");
    ak_proto_report(&proto, out);
    if (ak_board_net_ready()) {
        out("network link:\n");
        ak_proto_report(&net_proto, out);
        out("%u telemetry frames sent\n", telemetry_sent);
    }
    ak_board_net_report(out);
}

/* The blackbox as the protocol sees it: a count, and the i-th oldest record as
 * fixed-width bytes. The record is encoded rather than memcpy'd, because a C
 * struct's padding is a compiler's opinion and a wire format cannot be. */
/*
 * The three logs over the protocol, as one interface: a tool asks for a source
 * and then reads records by index exactly as it does for the fast ring. Two of
 * them are rings in RAM and one is out in flash, and the difference is this
 * function rather than the client's problem.
 */
static int32_t proto_log_count(void *ctx, uint8_t source)
{
    (void)ctx;

    switch (source) {
    case AK_PROTO_LOG_FAST:
        return blackbox.count;
    case AK_PROTO_LOG_LONG:
        return longlog->count;
    case AK_PROTO_LOG_FLASH:
        /* -1 says "this device has no log in flash", which is a different
         * answer from an empty one and the one the ESP32 gives. */
        return flashlog_state == AK_FLASHLOG_OK
                   ? (int32_t)ak_flashlog_count(&flashlog)
                   : -1;
    default:
        return -1;
    }
}

static int proto_log_source_present(uint8_t source)
{
    switch (source) {
    case AK_PROTO_LOG_FAST:
    case AK_PROTO_LOG_LONG:
        return 1;
    case AK_PROTO_LOG_FLASH:
        return flashlog_state == AK_FLASHLOG_OK;
    default:
        return 0;
    }
}

static unsigned proto_log_record(void *ctx, uint8_t source, uint16_t index,
                                 uint8_t *out, unsigned capacity)
{
    (void)ctx;
    ak_log_record_t record;

    if (!proto_log_source_present(source)) {
        return 0;
    }
    if (source == AK_PROTO_LOG_FLASH) {
        /* A slot the power cut in half is refused here rather than sent as
         * whatever its bytes happen to spell: the client counts it as missing
         * and carries on, which is what the console dump does too. */
        if (ak_flashlog_record_at(&flashlog, index, &record) != 1) {
            return 0;
        }
    } else {
        const ak_log_t *ring = source == AK_PROTO_LOG_LONG ? longlog : &blackbox;

        if (!ak_log_get(ring, index, &record)) {
            return 0;
        }
    }
    return ak_log_encode_record(&record, out, capacity);
}

/* The values live here, the parameter table points at them, and one callback
 * pushes them into the code that uses them - so a `set`, a `load` and a
 * calibration all take the same path to the same place. */
static float align_roll_deg;
static float align_pitch_deg;
static float align_yaw_deg;
static float gyro_bias_dps[3];
static float accel_bias_g[3];
static float accel_scale[3];

static void apply_parameters(void)
{
    ak_align_set(&align, align_roll_deg, align_pitch_deg, align_yaw_deg);
    ak_gyro_cal_set_bias_dps(&gyro_cal, gyro_bias_dps);
    ak_accel_cal_set(&accel_cal, accel_bias_g, accel_scale);
    /* The waypoints are degrees in the table, because that is what a person
     * types and what a configurator shows; the navigator keeps them in the
     * protocol's own 1e-7 units, because that is what the GPS speaks and a
     * round trip through degrees is a metre nobody needs to lose. */
    for (int i = 0; i < AK_NAV_WAYPOINTS; i++) {
        ak_nav_set_waypoint(&nav, i, (int32_t)(wp_lat_deg[i] * 10000000.0f),
                            (int32_t)(wp_lon_deg[i] * 10000000.0f));
    }
    ak_nav_set_waypoint_count(&nav, (int)wp_count);
    nav.min_alt_mm = (int32_t)(rth_min_alt_m * 1000.0f);
    nav.fence_enabled = fence_enable ? 1 : 0;
    /* Seconds in the parameter, milliseconds in the manoeuvre, and the clock
     * the launch is timed against is the same one the console reads. */
    launch.cfg.timeout_ms = launch_timeout_s * 1000u;
    /* Which return the navigator flies is the airframe's answer, not a
     * parameter of its own: a quadrotor climbs, translates and settles, and a
     * wing banks and circles, and asking the pilot to keep two selectors in
     * step is asking for the one time they are not. */
    ak_nav_set_profile(&nav, ak_mixer_for_airframe(flight.airframe)->fixed_wing
                                ? AK_NAV_PROFILE_WING
                                : AK_NAV_PROFILE_QUAD);
}

/*
 * The sensors, as SENSOR_INFO asks for them.
 *
 * This is `imu_report`, `baro_report`, `rangefinder_report`, `battery_report`
 * and `gps_report` seen from the other end, and it has the same obligation they
 * do: the first question a person asks a sensor readout is "is there one", and
 * a struct of zeros cannot answer it. So each arm sets `present` from the same
 * flag its console report branches on - imu_ok, baro_ok, range_ok,
 * battery_ready, gps.have_fix - and when that flag is clear it fills in the
 * *reason* rather than leaving a reading of zero behind.
 *
 * Every number is taken from the driver's own state, not recomputed. The
 * temperature is converted to hundredths of a degree and the voltages to
 * centivolts, which is a change of unit and not of authority; a client that
 * wanted to do its own arithmetic could, and would be wrong in the same places
 * the console would be, because there is one implementation and this is it.
 */
static void sensor_name(char *dst, const char *name)
{
    unsigned i = 0;
    while (i < AK_PROTO_SENSOR_NAME - 1u && name[i] != '\0') {
        dst[i] = name[i];
        i++;
    }
    while (i < AK_PROTO_SENSOR_NAME) {
        dst[i] = '\0';
        i++;
    }
}

static void proto_sensor_state(void *ctx, uint8_t topic, ak_proto_sensor_t *out)
{
    (void)ctx;

    switch (topic) {
    case AK_PROTO_SENSOR_IMU: {
        ak_proto_imu_t *imu_out = &out->as.imu;
        if (!imu_ok) {
            /* Which of the three ways it is missing. The console distinguishes
             * them - "nothing answered on the bus" is a wiring job and
             * "answered 0xNN, which is not a known part" is a driver job - and
             * a client that collapsed them would send somebody to the wrong
             * one. */
            switch (ak_imu_last_result()) {
            case AK_IMU_UNKNOWN_PART:
                imu_out->absent_reason = AK_PROTO_IMU_ABSENT_UNKNOWN_PART;
                imu_out->whoami = ak_imu_last_whoami();
                break;
            case AK_IMU_NO_CONFIG:
                imu_out->absent_reason = AK_PROTO_IMU_ABSENT_NO_CONFIG;
                break;
            case AK_IMU_NOBODY:
            case AK_IMU_OK:
            default:
                imu_out->absent_reason = AK_PROTO_IMU_ABSENT_NOBODY;
                break;
            }
            return;
        }
        out->present = 1;
        sensor_name(imu_out->driver, imu.driver->name);
        for (unsigned i = 0; i < 3; i++) {
            imu_out->accel[i] = (int16_t)(imu_sample.accel[i] * 1000.0f);
            imu_out->gyro[i] = (int16_t)(imu_sample.gyro[i] * 1000.0f);
            imu_out->gyro_bias[i] = (int16_t)(gyro_bias_dps[i] * 1000.0f);
        }
        imu_out->align[0] = (int16_t)align_roll_deg;
        imu_out->align[1] = (int16_t)align_pitch_deg;
        imu_out->align[2] = (int16_t)align_yaw_deg;
        imu_out->samples = imu.samples;
        imu_out->errors = imu.errors;
        return;
    }

    case AK_PROTO_SENSOR_BARO: {
        ak_proto_baro_t *baro_out = &out->as.baro;
        if (!baro_ok) {
            return;
        }
        out->present = 1;
        sensor_name(baro_out->driver, baro.driver->name);
        baro_out->pressure_pa = (int32_t)baro_sample.pressure_pa;
        baro_out->temperature_cdeg = (int16_t)(baro_sample.temperature_c * 100.0f);
        baro_out->have_reference = baro_have_reference ? 1u : 0u;
        baro_out->reference_pa = (int32_t)baro_reference_pa;
        /* Both heights in centimetres, from the same helpers the console prints
         * - including the "not captured yet" case, where the console prints
         * nothing at all and this sends zero *with have_reference clear*, which
         * is the distinction the flag is for. */
        baro_out->height_cm =
            baro_have_reference
                ? (int32_t)(ak_baro_altitude_m(baro_sample.pressure_pa,
                                               baro_reference_pa) *
                            100.0f)
                : 0;
        baro_out->have_gps_reference = altitude.have_gps_reference ? 1u : 0u;
        baro_out->fused_cm = (int32_t)(ak_altitude_height_m(&altitude) * 100.0f);
        baro_out->samples = baro.samples;
        baro_out->errors = baro.errors;
        baro_out->fails = baro_fails;
        baro_out->baro_samples = altitude.baro_samples;
        baro_out->gps_samples = altitude.gps_samples;
        return;
    }

    case AK_PROTO_SENSOR_RANGE: {
        ak_proto_range_t *range_out = &out->as.range;
        if (!range_ok) {
            return;
        }
        out->present = 1;
        sensor_name(range_out->driver, range.driver->name);
        range_out->address = range.driver->address;
        /* The driver keeps the range as a signed millimetre count and the wire
         * carries it unsigned, so the cast is checked rather than assumed: a
         * part whose reach did not fit sixteen bits would arrive as a small
         * number, and 65535 is the answer that cannot be mistaken for one. */
        range_out->max_mm = range.driver->max_mm < 0 ||
                                    range.driver->max_mm > 65535
                                ? (uint16_t)65535
                                : (uint16_t)range.driver->max_mm;
        /* Negative is "nothing in range", and it is sent as it is rather than
         * clamped to zero: zero is a wall against the lens, which is a
         * different thing to be looking at. */
        range_out->distance_mm = range.distance_mm;
        range_out->age_ms = range.last_ms != 0u ? ak_time_ms() - range.last_ms : 0u;
        range_out->samples = range.samples;
        range_out->out_of_range = range.out_of_range;
        range_out->rejected = range.rejected;
        range_out->faults = range.faults;
        range_out->fails = range_fails;
        range_out->land_mm = range_land_mm;
        range_out->agree_cm = (uint16_t)(range_agree_m * 100.0f);
        return;
    }

    case AK_PROTO_SENSOR_BATTERY: {
        ak_proto_battery_t *battery_out = &out->as.battery;
        if (!battery_ready) {
            /* No divider on this board at all - which is a fact about the
             * hardware and not a reading, so it is `present` staying clear
             * rather than a pack at zero volts. */
            return;
        }
        out->present = 1;
        battery_out->ready = 1;
        battery_out->have_reading = battery.samples > 0u ? 1u : 0u;
        battery_out->state = (uint8_t)battery.state;
        battery_out->cells = (uint8_t)battery.cells;
        battery_out->volts_cv = (uint16_t)(battery.volts * 100.0f + 0.5f);
        battery_out->per_cell_cv = (uint16_t)(battery.volts_per_cell * 100.0f + 0.5f);
        battery_out->pin_mv = battery.pin_volts < 0.0f
                                  ? (int16_t)-1
                                  : (int16_t)(battery.pin_volts * 1000.0f + 0.5f);
        battery_out->ratio_milli =
            (uint16_t)(battery.divider_ratio * 1000.0f + 0.5f);
        battery_out->rth = battery_rth ? 1u : 0u;
        battery_out->warn_cell_mv =
            (uint16_t)(battery.warn_cell_v * 1000.0f + 0.5f);
        battery_out->critical_cell_mv =
            (uint16_t)(battery.critical_cell_v * 1000.0f + 0.5f);
        /* Samples and returns are the console's `readings:` and `return:`
         * lines. `rejected` is the same number on both, so a client that shows
         * the pack's voltage also shows how much of it was thrown away. */
        battery_out->samples = battery.samples;
        battery_out->rejected = battery.rejected;
        battery_out->returns = battery_returns;
        return;
    }

    case AK_PROTO_SENSOR_GPS: {
        ak_proto_gps_t *gps_out = &out->as.gps;
        /* Always present: the GPS UART is part of every board in this tree, so
         * the absence to report here is "no fix yet", which the have_fix byte
         * says - and it is a different sentence from "no receiver", which the
         * counters distinguish because a receiver that never frames and one
         * that frames nothing usable leave different numbers behind. */
        out->present = 1;
        gps_out->have_fix = gps.have_fix ? 1u : 0u;
        gps_out->fix_type = gps.fix.fix_type;
        gps_out->fix_ok = gps.fix.fix_ok ? 1u : 0u;
        gps_out->satellites = gps.fix.satellites;
        gps_out->valid_now = ak_gps_fix_valid(&gps, ak_time_ms(), 2000u) ? 1u : 0u;
        gps_out->lat_e7 = gps.fix.lat_e7;
        gps_out->lon_e7 = gps.fix.lon_e7;
        gps_out->alt_msl_mm = gps.fix.alt_msl_mm;
        gps_out->speed_mm_s = gps.fix.speed_mm_s;
        gps_out->course_e5 = gps.fix.course_e5;
        gps_out->have_home = nav.have_home ? 1u : 0u;
        gps_out->home_lat_e7 = nav.home_lat_e7;
        gps_out->home_lon_e7 = nav.home_lon_e7;
        gps_out->home_distance_m = -1;
        gps_out->home_bearing_cdeg = 0;
        if (nav.have_home && gps.have_fix) {
            int32_t distance_m = 0;
            int32_t bearing_e2 = 0;
            ak_nav_distance_bearing(gps.fix.lat_e7, gps.fix.lon_e7,
                                    nav.home_lat_e7, nav.home_lon_e7,
                                    &distance_m, &bearing_e2);
            gps_out->home_distance_m = distance_m;
            gps_out->home_bearing_cdeg = bearing_e2;
        }
        gps_out->returning = nav.active ? 1u : 0u;
        gps_out->rth_enabled = rth_enable ? 1u : 0u;
        gps_out->fixes = gps_fixes;
        gps_out->dropped = ak_board_gps_dropped();
        gps_out->config_sends = gps_config_sends;
        return;
    }

    default:
        /* Unreachable: the dispatch bounds the topic before calling. Left
         * returning with `present` clear rather than guessing at a body. */
        return;
    }
}

/* Measure what the gyro reports while the aircraft is not moving. The result
 * goes into the parameter table rather than straight into the calibration, so
 * `params` shows it, `save` keeps it, and it is range-checked like any other
 * value. */
/* `calibrate` measures what "still" looks like; `calibrate rc` measures what
 * "centred" looks like. They are the same idea - learn a baseline from a
 * stationarity the pilot provides - and they belong behind one command. */
static int calibrate_rc(ak_printf_fn out)
{
    if (ak_flight_state(&flight) != AK_FLIGHT_DISARMED) {
        out("calibrate rc: refusing - the aircraft is not disarmed\n");
        return -1;
    }

    ak_rc_cal_init(&rc_cal, 50u);
    ak_rc_cal_start(&rc_cal);
    for (unsigned i = 0; i < 200u && !rc_cal.done; i++) {
        if (receiver.channels.valid) {
            ak_rc_cal_feed(&rc_cal, &receiver.channels);
        }
        ak_delay_ms(5);
    }

    if (!rc_cal.done) {
        out("calibrate rc: no frames from the receiver\n");
        return -1;
    }

    int32_t offset[AK_RC_CHANNELS];
    if (!ak_rc_cal_apply(&rc_cal, &flight.rc_cfg, offset)) {
        out("calibrate rc: nothing to apply\n");
        return -1;
    }

    /* Push the new centre into the parameter table, so it is visible to
     * `params`, kept by `save`, and range-checked like any other value. */
    char text[24];
    char message[48];
    ak_format_uint(flight.rc_cfg.mid, 0, text, sizeof text);
    message[0] = '\0';
    if (ak_params_set(&params, "rc_mid", text, message, sizeof message) != 0) {
        out("calibrate rc: %s\n", message);
        return -1;
    }

    out("rc:        %u frames, centre %u (was off by %d, %d, %d on roll, "
        "pitch, yaw)\n",
        rc_cal.samples, flight.rc_cfg.mid, offset[AK_RC_ROLL],
        offset[AK_RC_PITCH], offset[AK_RC_YAW]);
    out("calibrate rc: done - 'save' keeps it\n");
    return 0;
}

/*
 * Six positions, one command each. The pilot puts the aircraft on a face and
 * says which one, the firmware samples it, and the arithmetic waits until all
 * six are in. Doing it a face at a time is not politeness: a calibration that
 * has to be done in one go is one nobody finishes, and a missing face is
 * worse than a wrong one because it is invisible.
 */
static int calibrate_accel(ak_printf_fn out, const char *face_text)
{
    /*
     * The same rule the gyro, the receiver and the pack calibrations follow,
     * and the one this was missing: a calibration is for an aircraft on a
     * bench with its props off. Two reasons, and the second is the one that
     * made this a defect rather than a tidiness: holding an armed aircraft in
     * six attitudes is a hand near a live throttle, and the last face writes
     * `accel_bias_*` and `accel_scale_*` - which the flight loop applies to the
     * next sample, so an aircraft in the air would have its attitude estimate
     * changed under it. The bench session for exactly this found it: the three
     * other calibrations refused while armed, and this one went ahead.
     */
    if (ak_flight_state(&flight) != AK_FLIGHT_DISARMED) {
        out("calibrate accel: refusing - the aircraft is not disarmed\n");
        return -1;
    }
    if (face_text == 0) {
        out("calibrate accel: which face? [0] level, [1] inverted, "
            "[2] nose down,\n");
        out("                 [3] nose up, [4] right side down, "
            "[5] left side down\n");
        return -1;
    }

    /* One digit, so the parse is a digit test rather than a call into the
     * parameter table's number parsing. */
    if (face_text[0] < '0' || face_text[0] > '9' || face_text[1] != '\0' ||
        (face_text[0] - '0') >= AK_ACCEL_FACES) {
        out("calibrate accel: '%s' is not a face\n", face_text);
        return -1;
    }
    int face = face_text[0] - '0';

    if (!ak_accel_cal_begin(&accel_cal, face)) {
        out("calibrate accel: cannot start that face\n");
        return -1;
    }
    out("calibrate accel: hold the aircraft %s, not moving...\n",
        ak_accel_cal_face_name(face));

    for (unsigned i = 0; i < AK_CAL_SAMPLES * 6u; i++) {
        ak_imu_sample_t sample;
        if (ak_imu_read(&imu, &sample) == 0) {
            ak_align_apply(&align, &sample);
            if (ak_accel_cal_feed(&accel_cal, &sample)) {
                break;
            }
        }
        ak_delay_ms(1);
    }

    if (!ak_accel_cal_have(&accel_cal, face)) {
        out("calibrate accel: not enough still samples (%u of %u, %u "
            "rejected) - the aircraft is moving, or the sensor stopped "
            "answering\n",
            accel_cal.samples[face], accel_cal.wanted, accel_cal.rejected);
        return -1;
    }

    for (int f = 0; f < AK_ACCEL_FACES; f++) {
        out("  [%d] %-16s %s\n", f, ak_accel_cal_face_name(f),
            ak_accel_cal_have(&accel_cal, f) ? "measured" : "-");
    }

    int remaining = 0;
    for (int f = 0; f < AK_ACCEL_FACES; f++) {
        if (!ak_accel_cal_have(&accel_cal, f)) {
            remaining++;
        }
    }
    if (remaining > 0) {
        out("calibrate accel: %s done - %d face%s to go\n",
            ak_accel_cal_face_name(face), remaining, remaining == 1 ? "" : "s");
        return 0;
    }

    if (!ak_accel_cal_finish(&accel_cal)) {
        out("calibrate accel: the six readings are not a plausible gravity - "
            "one of the faces was not flat. Start again from face 0.\n");
        ak_accel_cal_reset(&accel_cal);
        return -1;
    }

    static const char *bias_names[3] = { "accel_bias_x", "accel_bias_y",
                                         "accel_bias_z" };
    static const char *scale_names[3] = { "accel_scale_x", "accel_scale_y",
                                          "accel_scale_z" };
    float bias[3];
    float scale[3];
    char message[64];
    char bias_text[3][16];
    char scale_text[3][16];

    ak_accel_cal_get(&accel_cal, bias, scale);
    for (int i = 0; i < 3; i++) {
        ak_format_fixed(bias[i], 4, bias_text[i], sizeof bias_text[i]);
        message[0] = '\0';
        if (ak_params_set(&params, bias_names[i], bias_text[i], message,
                          sizeof message) != 0) {
            out("calibrate accel: %s\n", message);
            return -1;
        }
        ak_format_fixed(scale[i], 4, scale_text[i], sizeof scale_text[i]);
        message[0] = '\0';
        if (ak_params_set(&params, scale_names[i], scale_text[i], message,
                          sizeof message) != 0) {
            out("calibrate accel: %s\n", message);
            return -1;
        }
    }

    /* The console formatter has no floating point, which is why these went
     * through ak_format_fixed above - the first version printed "%g %g %g" at
     * itself and the simulator read the literal back. */
    out("accel bias:  %s %s %s g\n", bias_text[0], bias_text[1], bias_text[2]);
    out("accel scale: %s %s %s\n", scale_text[0], scale_text[1],
        scale_text[2]);
    out("calibrate accel: done - 'save' keeps it across a reboot\n");
    return 0;
}

/*
 * The pack's divider, measured rather than computed by hand.
 *
 * `vbat_ratio` is how many volts at the pack one volt at the ADC pin is worth,
 * and it is a property of two resistors on the board: the board file says what
 * its own divider is and the parameter says what the firmware believes, which
 * on a board whose divider nobody has measured is the *other* board's number.
 * Getting it wrong is not cosmetic - the pack estimate is what decides when the
 * aircraft comes home, so a ratio out by a factor of two reads a half-empty pack
 * as a healthy one, and the first symptom is a wing still flying at 2.5 volts a
 * cell.
 *
 * The arithmetic is one division, and the input a person can actually produce is
 * the other half of it: a multimeter across the pack. So the command takes that
 * number, reads the pin, and does the division - the same shape as
 * `calibrate rc`, which takes what the sticks are doing rather than asking
 * somebody to work out four ranges.
 */
static int calibrate_vbat(ak_printf_fn out, const char *volts_text)
{
    if (ak_flight_state(&flight) != AK_FLIGHT_DISARMED) {
        out("calibrate vbat: refusing - the aircraft is not disarmed\n");
        return -1;
    }
    if (!battery_ready) {
        out("calibrate vbat: this board has no way to measure the pack\n");
        return -1;
    }

    float measured = 0.0f;
    if (volts_text == 0 || !ak_parse_float(volts_text, &measured)) {
        out("calibrate vbat: how many volts? 'calibrate vbat 12.60'\n");
        return -1;
    }
    if (measured < 1.0f || measured > 60.0f) {
        char text[16];
        ak_format_fixed(measured, 2, text, sizeof text);
        out("calibrate vbat: %s V is not a pack this aircraft flies\n", text);
        return -1;
    }

    /* Several conversions rather than one, because the answer is a ratio and
     * one count at the pin is worth eleven at the pack on a 10k/1k: eight of
     * them average the ADC's own noise down to well inside a tenth of a cell. */
    float pin = 0.0f;
    unsigned taken = 0u;

    for (unsigned i = 0; i < 8u; i++) {
        float volts = ak_board_battery_pin_volts();
        if (volts >= 0.0f) {
            pin += volts;
            taken++;
        }
    }
    if (taken == 0u) {
        out("calibrate vbat: the pin reading never finished - the ADC, not the "
            "pack\n");
        return -1;
    }
    pin /= (float)taken;

    /* A pin reading almost nothing is a divider with no pack on it, and
     * dividing by it would produce a ratio made of noise. */
    if (pin < 0.05f) {
        char text[16];
        ak_format_fixed(pin * 1000.0f, 1, text, sizeof text);
        out("calibrate vbat: the pin reads %s mV - connect the pack\n", text);
        return -1;
    }

    float ratio = measured / pin;
    char ratio_text[16];
    char volts_text_fixed[16];
    char pin_text[16];
    char message[64];

    ak_format_fixed(ratio, 3, ratio_text, sizeof ratio_text);
    ak_format_fixed(measured, 2, volts_text_fixed, sizeof volts_text_fixed);
    ak_format_fixed(pin, 3, pin_text, sizeof pin_text);

    /* And it goes in through the parameter table, not around it: the range
     * check, the changed count and the saved record are the table's, and a
     * calibration that wrote the variable directly would be a second way to set
     * a parameter. */
    message[0] = '\0';
    if (ak_params_set(&params, "vbat_ratio", ratio_text, message,
                      sizeof message) != 0) {
        out("calibrate vbat: the table refused %s (%s)\n", ratio_text,
            message[0] != '\0' ? message : "no reason given");
        return -1;
    }

    out("calibrate vbat: %s V at the pack over %s V at the pin is a ratio of "
        "%s\n", volts_text_fixed, pin_text, ratio_text);
    out("                'save' keeps it, and the pack now reads %s V\n",
        volts_text_fixed);
    return 0;
}

static int calibrate(ak_printf_fn out, int argc, const char *const *argv)
{
    const char *what = argc > 1 ? argv[1] : 0;

    if (what != 0 && ak_str_eq(what, "rc")) {
        return calibrate_rc(out);
    }
    if (what != 0 && ak_str_eq(what, "accel")) {
        return calibrate_accel(out, argc > 2 ? argv[2] : 0);
    }
    if (what != 0 && ak_str_eq(what, "vbat")) {
        return calibrate_vbat(out, argc > 2 ? argv[2] : 0);
    }
    if (what != 0 && !ak_str_eq(what, "gyro")) {
        out("calibrate: what? 'gyro', 'rc', 'accel <face>', or "
            "'vbat <pack volts>'\n");
        return -1;
    }

    if (!imu_ok) {
        out("calibrate: no inertial sensor\n");
        return -1;
    }
    if (ak_flight_state(&flight) != AK_FLIGHT_DISARMED) {
        out("calibrate: refusing - the aircraft is not disarmed\n");
        return -1;
    }

    ak_gyro_cal_start(&gyro_cal);
    for (unsigned i = 0; i < AK_CAL_SAMPLES * 4u && !gyro_cal.done; i++) {
        ak_imu_sample_t sample;
        if (ak_imu_read(&imu, &sample) == 0) {
            ak_align_apply(&align, &sample);
            (void)ak_gyro_cal_feed(&gyro_cal, &sample);
        }
        ak_delay_ms(1);
    }

    if (!gyro_cal.done) {
        out("calibrate: failed with %u good samples and %u rejected - the "
            "aircraft is moving, or the sensor stopped answering\n",
            gyro_cal.samples, gyro_cal.rejected);
        return -1;
    }

    static const char *names[3] = { "gyro_bias_roll", "gyro_bias_pitch",
                                    "gyro_bias_yaw" };
    char text[3][24];
    char message[64];
    float bias_dps[3];

    ak_gyro_cal_bias_dps(&gyro_cal, bias_dps);
    for (int i = 0; i < 3; i++) {
        ak_format_fixed(bias_dps[i], 3, text[i], sizeof text[i]);
        message[0] = '\0';
        if (ak_params_set(&params, names[i], text[i], message, sizeof message) != 0) {
            out("calibrate: %s\n", message);
            return -1;
        }
    }

    out("gyro bias: roll %s, pitch %s, yaw %s dps (%u samples, %u rejected)\n",
        text[0], text[1], text[2], gyro_cal.samples, gyro_cal.rejected);
    out("calibrate: done - 'save' keeps it across a reboot\n");
    return 0;
}

/*
 * `output` reports what the board's timers are doing; `output test` drives
 * them, one at a time, so that "the motor pads are on the right pins" and "the
 * servo horn moves the way the mixer thinks" are things somebody can watch
 * rather than things this firmware asserts.
 */
static int output_command(ak_printf_fn out, int argc, const char *const *argv)
{
    const char *what = argc > 1 ? argv[1] : 0;

    if (what != 0 && ak_str_eq(what, "test")) {
        if (argc > 2 && ak_str_eq(argv[2], "stop")) {
            output_test_active = 0;
            out("output test: stopped, everything at zero\n");
            return 0;
        }
        if (ak_flight_state(&flight) != AK_FLIGHT_DISARMED) {
            out("output test: refusing - the aircraft is not disarmed\n");
            return -1;
        }
        if (!ak_board_output_ready()) {
            out("output test: this board has no outputs\n");
            return -1;
        }
        output_test_active = 1;
        output_test_started_ms = ak_time_ms();
        out("output test: motors to %d%%, servos to half travel, one at a "
            "time - props off\n",
            (int)(AK_OUTPUT_TEST_MOTOR * 100.0f));
        out("             it repeats until 'output test stop'; the frame "
            "counts are above\n");
        return 0;
    }

    ak_board_output_report(out);
    /* And the servos' own plumbing, which is the half of this that a pilot sets
     * and the reason to read this command before a first flight: a reversed
     * linkage that nobody noticed is a wing that does not turn. */
    for (unsigned i = 0; i < AK_MAX_SERVOS; i++) {
        char trim[16];

        ak_format_fixed(servo_trim[i].trim_us, 0, trim, sizeof trim);
        out("servo %u:   %s, trim %s us, %u us at full stick\n",
            i + 1u, servo_trim[i].reversed ? "reversed" : "normal", trim,
            (unsigned)servo_trim[i].travel_us);
    }
    out("test:      %s\n", output_test_active ? "running" : "not running");
    return 0;
}

static void imu_report(ak_printf_fn out)
{
    if (!imu_ok) {
        out("imu:       none - the flight core stays in failsafe\n");
        imu_absence(out, "imu:       ");
        return;
    }
    out("imu:       %s, %u samples, %u errors\n", imu.driver->name,
        imu.samples, imu.errors);
    out("sample:    accel %d %d %d per-mille of g\n",
        (int)(imu_sample.accel[0] * 1000.0f), (int)(imu_sample.accel[1] * 1000.0f),
        (int)(imu_sample.accel[2] * 1000.0f));
    out("           gyro %d %d %d mrad/s\n", (int)(imu_sample.gyro[0] * 1000.0f),
        (int)(imu_sample.gyro[1] * 1000.0f), (int)(imu_sample.gyro[2] * 1000.0f));
    out("alignment: roll %d, pitch %d, yaw %d degrees\n",
        (int)align_roll_deg, (int)align_pitch_deg, (int)align_yaw_deg);
    out("gyro bias: %d %d %d mdps\n", (int)(gyro_bias_dps[0] * 1000.0f),
        (int)(gyro_bias_dps[1] * 1000.0f), (int)(gyro_bias_dps[2] * 1000.0f));
}

/*
 * The rangefinder, as `range` prints it: which part, how far the ground is,
 * and the counters that say whether to believe it.
 *
 * The counters are the point of the command. "1.20 m" on its own is a number;
 * a number next to "3 readings thrown away as impossible and 12 reads with no
 * answer" is a measurement somebody can decide about - which is the same
 * reason the receiver prints framing errors and the barometer prints failed
 * reads.
 */
static void rangefinder_report(ak_printf_fn out)
{
    if (!range_ok) {
        out("range:     none fitted - the last metres are the barometer's "
            "and the landing rule is the old one\n");
        return;
    }

    out("range:     %s at 0x%02x, up to %d.%02d m\n", range.driver->name,
        (unsigned)range.driver->address, (int)(range.driver->max_mm / 1000),
        (int)((range.driver->max_mm % 1000) / 10));
    if (range.distance_mm >= 0) {
        out("ground:    %d.%03d m below, measured %u ms ago\n",
            (int)(range.distance_mm / 1000),
            (int)(range.distance_mm % 1000), (unsigned)(ak_time_ms() -
                                                        range.last_ms));
    } else {
        out("ground:    nothing in range\n");
    }
    out("readings:  %u kept, %u out of range, %u impossible, %u failed\n",
        range.samples, range.out_of_range, range.rejected, range.faults);
    if (range_fails > 0u) {
        out("           %u reads in a row with no answer\n", range_fails);
    }
    /* What the landing rule does with it, in the words the rule uses - the
     * two numbers a person would change if a landing is refused or early. */
    out("landing:   the ground counts as reached within %u mm, and only while "
        "the two heights agree within %d.%01d m\n", (unsigned)range_land_mm,
        (int)(range_agree_m), (int)((range_agree_m * 10.0f)) % 10);
}

/*
 * The barometer, as `baro` prints it: pressure, the temperature it is
 * compensated with, and the height above where the aircraft was standing.
 *
 * The reference is captured while disarmed, and it is the whole reason a
 * barometer is worth carrying: the absolute pressure is today's weather, and
 * the *change* since take-off is the aircraft's altitude to a few tens of
 * centimetres. A number in metres above the bench is a number somebody can
 * check by lifting the aircraft a metre.
 */
static void baro_report(ak_printf_fn out)
{
    if (!baro_ok) {
        out("baro:      none fitted - altitude comes from the gps alone\n");
        return;
    }

    out("baro:      %s, %u samples, %u errors\n", baro.driver->name,
        baro.samples, baro.errors);
    out("pressure:  %d pa, temperature %d.%01d c\n",
        (int)baro_sample.pressure_pa, (int)baro_sample.temperature_c,
        (int)(baro_sample.temperature_c * 10.0f) % 10);
    if (baro_have_reference) {
        out("reference: %d pa, captured on the ground\n",
            (int)baro_reference_pa);
        out("height:    %d.%02d m above the reference, from the barometer\n",
            (int)ak_baro_altitude_m(baro_sample.pressure_pa, baro_reference_pa),
            (int)(ak_baro_altitude_m(baro_sample.pressure_pa,
                                     baro_reference_pa) *
                      100.0f) %
                100);
    } else {
        out("reference: not captured yet (arm once with the aircraft still)\n");
    }

    /* What the aircraft actually flies on: the barometer's changes, the GPS's
     * absolute reference, and the leak between them. */
    out("fused:     %d.%02d m above the reference%s\n",
        (int)ak_altitude_height_m(&altitude),
        (int)(ak_altitude_height_m(&altitude) * 100.0f) % 100,
        altitude.have_gps_reference ? ", gps correcting the drift"
                                    : ", no gps reference yet");
    out("           %u baro, %u gps samples\n", altitude.baro_samples,
        altitude.gps_samples);
}

/*
 * The flight pack, as `battery` prints it: what the ADC measured, what the
 * divider makes of it, and the only number that decides anything - volts per
 * cell.
 *
 * The state comes first because it is the answer. Everything below it is the
 * working: the ratio to check against a multimeter, the thresholds to check
 * against a preference, and the board's own raw counts, because two numbers
 * that have been through the same arithmetic cannot disagree and the raw one
 * can.
 */
static void battery_report(ak_printf_fn out)
{
    if (!battery_ready) {
        /* The board says what it has, the way it does for a sensor it has no
         * socket for - and this is the honest answer for a board whose
         * divider is designed in and not soldered in. The raw line below it
         * still shows the conversion, which is the part that is checkable
         * without a pack. */
        out("battery:   none fitted on this board\n");
        ak_board_battery_report(out);
        return;
    }
    if (battery.samples == 0u) {
        out("battery:   no reading yet\n");
    } else if (battery.state == AK_BATTERY_ABSENT) {
        out("battery:   nothing connected (%d.%03d V at the pin)\n",
            (int)battery.pin_volts,
            (int)(battery.pin_volts * 1000.0f) % 1000);
    } else {
        out("battery:   %uS, %d.%02d V pack, %d.%02d V a cell - %s\n",
            battery.cells, (int)battery.volts,
            (int)(battery.volts * 100.0f) % 100, (int)battery.volts_per_cell,
            (int)(battery.volts_per_cell * 100.0f) % 100,
            ak_battery_state_name(battery.state));
    }
    out("divider:   %d.%03d pack volts per volt at the pin (vbat_ratio)\n",
        (int)battery.divider_ratio,
        (int)(battery.divider_ratio * 1000.0f) % 1000);
    out("return:    %s when a cell is under %d.%03d V, %u engagement%s\n",
        battery_rth ? "comes home" : "stays with the pilot",
        (int)battery.critical_cell_v,
        (int)(battery.critical_cell_v * 1000.0f) % 1000,
        (unsigned)battery_returns, battery_returns == 1u ? "" : "s");
    /* "under", because that is the comparison: exactly on the threshold is not
     * past it, and the words should be the ones the code uses. */
    out("thresholds: low under %d.%03d V a cell, critical under %d.%03d V\n",
        (int)battery.warn_cell_v, (int)(battery.warn_cell_v * 1000.0f) % 1000,
        (int)battery.critical_cell_v,
        (int)(battery.critical_cell_v * 1000.0f) % 1000);
    out("readings:  %u taken, %u rejected as implausible\n", battery.samples,
        battery.rejected);
    ak_board_battery_report(out);
}

/*
 * Has it landed?
 *
 * The one thing this firmware does that a mistake cannot undo is stop the
 * motors while the aircraft is in the air, so the question is answered from
 * what is *known* rather than from a height: the navigator is in the last part
 * of its descent, the aircraft is below the hover height it settled at - and
 * the height has stopped falling, for a second and a half, while the navigator
 * is still asking it to fall.
 *
 * The absolute height is deliberately not the evidence. The estimate is a
 * filtered barometer anchored by the GPS, and at a hundred metres above sea
 * level the simulation has it wobbling by several metres: a rule that wanted
 * "within a metre of the ground" never fired at all, and one that wanted
 * "within five" would fire on a gust. What cannot be wrong is that a descent
 * which has stopped, under a navigator still commanding it down, means
 * something is holding the aircraft up - and at the end of a descent that
 * something is the ground.
 *
 * A rangefinder changes what "known" means, and it is used first when there is
 * one: a part looking at the ground says how far away it is, which is a
 * measurement of the question rather than an inference from the one next to
 * it. Two guards come with it, and they are the whole reason it is safe to
 * prefer it:
 *
 * - **The reading has to hold.** Half a second of continuous ground contact,
 *   which at the part's 10 Hz is several readings and longer than any
 *   transient a light or a prop blade can produce.
 * - **It has to agree with the height estimate.** A lidar looking down at a
 *   hedge, a slope, or the roof of the car it flew over reports a metre and a
 *   half while the aircraft is twenty metres up, and it reports it steadily -
 *   so time alone cannot catch that. A disagreement of more than a few metres
 *   between the two height measurements means *neither* is believed and the
 *   aircraft keeps flying, which is the safe half of not knowing.
 *
 * With a rangefinder *disagreeing*, the barometric rule below is not allowed
 * to fire either. That is the case the rangefinder was added for: the
 * navigator's height estimate leaking by more than the descent it is asking
 * for makes a landing look like a stall, and the old rule would stop the
 * motors a metre and a half up. A fitted part that says "not yet" gets the
 * last word; a part that is absent, dead or out of range leaves the rule
 * exactly as it was.
 */
/* Two seconds of window, and the aircraft must have come down less than half
 * a metre in it: the navigator is asking for six tenths of a metre a second
 * down there, so a descent that is happening moves more than a metre in a
 * window and a landing moves almost nothing. Two windows in a row, so a single
 * quiet patch in a gusty estimate is not enough. */
#define AK_LAND_WINDOW_MS 2000u
#define AK_LAND_FELL_MM 600
#define AK_LAND_STILL_MS 4000u
/*
 * And the rangefinder's own version of the same rule.
 *
 * "Close to the ground" is not on its own enough to stop the motors, and the
 * reason is physical: an aircraft rests on its *gear*, so a part under the
 * fuselage does not read zero once it has landed - it reads however tall the
 * legs are, and that is a number this firmware has no business guessing. What
 * is true is the same thing the barometric rule is built on, one instrument
 * nearer: a descent that has *stopped* while the navigator is still asking for
 * it was stopped by something, and at the end of a descent that something is
 * the ground. So the part has to say the ground is within `range_land_mm` - a
 * gear-height of slack, not a measurement of the gear - *and* stop changing.
 *
 * Half a second at the part's 10 Hz is five readings, and a hundred
 * millimetres is a fifth of the half-metre a second the navigator commands
 * down there: a descent that is still happening moves three times that in a
 * window, and a landing, which moves nothing but noise, is nowhere near it.
 */
#define AK_LAND_RANGE_MS 500u
#define AK_LAND_RANGE_FELL_MM 100
#define AK_LAND_RANGE_STILL_MS 500u

/*
 * Is the height the navigator flies on still being *measured*?
 *
 * The estimate is a barometer anchored by a GPS, so it is live while either of
 * those is answering: the barometer at 32 Hz, the GPS at a few hertz. A
 * component that has stopped - a part that died, a bus that came loose, a
 * module that lost its sky - leaves the estimate perfectly still, and "still"
 * is what a landing looks like.
 */
static int altitude_live(uint32_t now)
{
    if (baro_ok && baro_fails < 2u) {
        return 1;
    }
    return ak_gps_fix_valid(&gps, now, 2000u);
}

static int landing_detector(uint32_t now)
{
    static int      have_mark;
    static uint32_t mark_ms;
    static int32_t  mark_alt_mm;
    static uint32_t still_ms;
    static int      have_ground_mark;
    static uint32_t ground_mark_ms;
    static int32_t  ground_mark_mm;
    static uint32_t ground_still_ms;
    int32_t         alt_mm = altitude_msl_mm();
    int32_t         agl_mm = -1;

    /*
     * What the rangefinder has to say, if it is saying anything: a reading
     * that is fresh enough to be a measurement of now, and that has not been
     * thrown away by the service for being impossible.
     */
    if (range_ok && ak_rangefinder_valid(&range, now, AK_RANGE_STALE_MS)) {
        agl_mm = range.distance_mm;
    }

    /* Not under a navigator that is landing, or nowhere to land relative to:
     * there is no question to answer. */
    if (!(nav.landing || nav.hold_landing) || !nav.have_home) {
        have_mark = 0;
        still_ms = 0;
        have_ground_mark = 0;
        ground_still_ms = 0;
        return 0;
    }

    if (agl_mm >= 0) {
        int32_t height_mm = alt_mm - nav.home_alt_mm;
        int32_t disagreement =
            agl_mm > height_mm ? agl_mm - height_mm : height_mm - agl_mm;

        /* This is the rangefinder's question while it has an answer, so the
         * barometric window starts again rather than running underneath it. */
        have_mark = 0;
        still_ms = 0;

        /*
         * Either the ground is not there yet, or the two measurements do not
         * agree about where the aircraft is. Both mean "not landed", and the
         * second one is a veto on the rule below rather than a fallback to it:
         * the part preferred for the landing is the one that has an opinion,
         * and a part that is reading a hedge from twenty metres up is exactly
         * the reading that must not stop the motors.
         */
        if (agl_mm > (int32_t)range_land_mm ||
            disagreement > (int32_t)(range_agree_m * 1000.0f)) {
            have_ground_mark = 0;
            ground_still_ms = 0;
            return 0;
        }

        /* Close, and agreeing: is it still coming down? */
        if (!have_ground_mark || (uint32_t)(now - ground_mark_ms) >=
                                     AK_LAND_RANGE_MS) {
            if (have_ground_mark) {
                int32_t fell_mm = ground_mark_mm - agl_mm;

                if (fell_mm < AK_LAND_RANGE_FELL_MM) {
                    ground_still_ms += (uint32_t)(now - ground_mark_ms);
                } else {
                    ground_still_ms = 0;
                }
            }
            ground_mark_mm = agl_mm;
            ground_mark_ms = now;
            have_ground_mark = 1;
        }
        return ground_still_ms >= AK_LAND_RANGE_STILL_MS;
    }
    have_ground_mark = 0;
    ground_still_ms = 0;

    /*
     * Past here the evidence is the estimate, and an estimate that is not
     * being measured stalls perfectly. So the height has to be live for the
     * window to count: either the barometer is answering, or the fix is - the
     * two sources the estimate is built from - and if neither is, this
     * refuses to say anything at all. (A rangefinder that *is* answering never
     * gets here: its verdict was taken above, on evidence of its own.)
     * Measured before this was added, with the barometer failing as a return's
     * descent began: the navigator's height froze at twenty metres, the
     * landing rule never fired (it needs the hover band first), and the
     * aircraft sat on the ground with its motors still running.
     */
    if (!altitude_live(now)) {
        have_mark = 0;
        still_ms = 0;
        return 0;
    }

    if (!have_mark || (uint32_t)(now - mark_ms) >= AK_LAND_WINDOW_MS) {
        if (have_mark) {
            int32_t fell_mm = mark_alt_mm - alt_mm;

            if (fell_mm < AK_LAND_FELL_MM) {
                still_ms += (uint32_t)(now - mark_ms);
            } else {
                still_ms = 0;
            }
        }
        mark_alt_mm = alt_mm;
        mark_ms = now;
        have_mark = 1;
    }
    return still_ms >= AK_LAND_STILL_MS;
}

/*
 * Telemetry out: what the handset is told.
 *
 * The frames are built in the flight core's own units and converted at the
 * wire in ak_crsf_telemetry.c; what this does is answer the one question that
 * file cannot - what the aircraft's state is right now - and put the bytes on
 * the receiver's transmit line. Two of the numbers are worth a note:
 *
 * - The battery percentage is an estimate from the pack's voltage: a straight
 *   line from the ceiling a cell can be detected at down to the critical
 *   threshold the pilot configured. The voltage rides in the same frame, so a
 *   handset shows both and the pilot can see how rough the estimate is.
 * - "Link live" is part of the mode, because an aircraft in failsafe and one
 *   whose pilot has handed it over are the two cases a handset has to be able
 *   to tell apart from the ground.
 */
static int telemetry_percent(void)
{
    float full = battery.cell_detect_v;
    float empty = battery.critical_cell_v;
    float percent;

    if (full <= empty || battery.state == AK_BATTERY_ABSENT) {
        return 0;
    }
    percent = (battery.volts_per_cell - empty) * 100.0f / (full - empty);
    if (percent < 0.0f) {
        percent = 0.0f;
    }
    if (percent > 100.0f) {
        percent = 100.0f;
    }
    return (int)(percent + 0.5f);
}

static void telemetry_input(ak_crsf_telemetry_t *in, uint32_t now_ms)
{
    ak_flight_state_t state = ak_flight_state(&flight);

    memset(in, 0, sizeof *in);

    if (battery_ready && battery.samples > 0u &&
        battery.state != AK_BATTERY_ABSENT) {
        in->volts_valid = 1;
        in->volts = battery.volts;
        in->percent_valid = 1;
        in->percent = telemetry_percent();
    }
    if (imu_ok) {
        in->attitude_valid = 1;
        in->roll_rad = flight.est.roll;
        in->pitch_rad = flight.est.pitch;
        in->yaw_rad = flight.est.yaw;
    }
    if (ak_gps_fix_valid(&gps, now_ms, 2000u)) {
        in->fix_valid = 1;
        in->lat_e7 = gps.fix.lat_e7;
        in->lon_e7 = gps.fix.lon_e7;
        in->alt_m = (float)gps.fix.alt_msl_mm / 1000.0f;
        in->speed_mm_s = (float)gps.fix.speed_mm_s;
        in->course_deg = (float)gps.fix.course_e5 / 100000.0f;
        in->satellites = gps.fix.satellites;
    }
    in->flight_mode = ak_crsf_flight_mode(
        state != AK_FLIGHT_DISARMED,
        /* Circling down is a failsafe as far as the handset is concerned: what
         * went away is the pilot's link, and if it comes back for a moment the
         * last thing the handset should name is a mode that sounds normal. */
        state == AK_FLIGHT_FAILSAFE || state == AK_FLIGHT_DESCEND,
        state == AK_FLIGHT_RTH, state == AK_FLIGHT_MANAGED,
        flight.link_live && flight.cmd.angle_mode);
}

/* Called from the flight loop, on the telemetry tick. The device ping is a
 * question and is answered at once; the rest are the scheduled frames, one per
 * tick at most. */
static void telemetry_service(void)
{
    uint8_t frame[AK_CRSF_TLM_MAX_FRAME];
    ak_crsf_telemetry_t in;
    uint32_t now = ak_time_ms();
    unsigned len;

    /* SBUS has no return path, and a board that cannot speak says so by
     * returning an error - which is a fact about the wiring, not a fault. */
    if (ak_rc_receiver_protocol(&receiver) != AK_RC_PROTOCOL_CRSF) {
        return;
    }
    if (receiver.crsf.pings != crsf_pings_answered) {
        crsf_pings_answered = receiver.crsf.pings;
        len = ak_crsf_tlm_device_info(&crsf_tlm, AK_PRODUCT_STR, frame,
                                      sizeof frame);
        if (len > 0u) {
            (void)ak_board_rc_send((const char *)frame, len);
        }
    }

    /* One clock read per tick, and the same number for everything in it: the
     * simulator charges a clock read that comes with no board call behind it
     * (it is modelling a spin-wait), so a loop that reads the clock for the
     * fun of it moves the aircraft's sense of time - see
     * docs/18-software-in-the-loop.md. */
    telemetry_input(&in, now);
    len = ak_crsf_tlm_next(&crsf_tlm, &in, now, frame, sizeof frame);
    if (len > 0u) {
        (void)ak_board_rc_send((const char *)frame, len);
    }
}

/* The receiver, as `rc` prints it: the core's view of the link, plus the one
 * number only the port knows - how many bytes the receive buffer had to drop. */
static void rc_report(ak_printf_fn out)
{
    /* The flight core's own configuration, so the sticks printed here are the
     * sticks the aircraft is flying. It used to be the build-time defaults
     * inside the report, which diverge from these the moment somebody runs
     * `calibrate rc`. */
    ak_rc_receiver_report(&receiver, &flight.rc_cfg, out);
    if (ak_rc_receiver_protocol(&receiver) == AK_RC_PROTOCOL_SBUS &&
        !ak_board_rc_inverted()) {
        /* The one thing about SBUS that is hardware rather than protocol. A
         * receiver on the wrong side of this looks exactly like a receiver on
         * the wrong baud rate, so it is worth saying which one it is before
         * somebody spends an afternoon on the baud rate. */
        /* The board's *own* file, named by the build rather than by hand: this
         * message used to send the reader to the F405's header whichever board
         * was running, and with three of them that is a wasted walk to a bench
         * with the wrong file. */
        out("warning:   SBUS idles low and this board has no inverter in front\n"
            "           of the receiver pin, so the port will read framing\n"
            "           errors - see src/boards/%s/board.h\n", AK_BOARD_STR);
    }
    out("uart:      %u bytes dropped by the receive buffer\n",
        ak_board_rc_dropped());
    /* The other direction, which the console is the only place to see: a
     * handset shows the frames that arrive, and this shows the ones that
     * left. `output test` has the same shape for the motors. */
    if (ak_rc_receiver_protocol(&receiver) == AK_RC_PROTOCOL_CRSF) {
        out("telemetry: %u frames out, %u device replies\n", crsf_tlm.frames,
            crsf_tlm.device_infos);
    } else {
        out("telemetry: none - sbus has no return path\n");
    }
}

/* Called by the console after any parameter changes. Four of them reach
 * hardware or arithmetic: the airframe (which mixer), the rate loop's gains
 * (which `ak_flight_apply_config` copies into the three `ak_pid_t`s), the
 * DShot rate, and the alignment and bias. */
static void parameters_changed(void)
{
    ak_flight_apply_config(&flight);
    apply_parameters();
    ak_board_output_set_rate(dshot_khz);
    /* Both halves of the receiver: the parser, and the board's line settings.
     * They are one setting from the pilot's side and two from the port's. */
    ak_rc_receiver_set_protocol(&receiver, rc_protocol);
    ak_board_rc_set_protocol(rc_protocol);
}

static const ak_cli_io_t cli_io = {
    .out = ak_console_printf,
    .now_ms = ak_time_ms,
    .config_read = ak_board_config_read,
    .config_write = ak_board_config_write,
    .reboot = ak_board_reboot,
    .bootloader = ak_board_enter_bootloader,
    .output_report = output_command,
    .rc_report = rc_report,
    .imu_report = imu_report,
    .baro_report = baro_report,
    .range_report = rangefinder_report,
    .battery_report = battery_report,
    .spi_test = ak_board_spi_loopback,
    .calibrate = calibrate,
    .log_dump = log_dump,
    .log_reset = log_reset,
    .longlog_dump = longlog_dump,
    .longlog_reset = longlog_reset,
    .flashlog_dump = flashlog_dump,
    .flashlog_reset = flashlog_reset,
    .gps_report = gps_report,
    .home_set = nav_home_set,
    .home_clear = nav_home_clear,
    .mission = mission_command,
    .preflight = preflight,
    .arm_report = arm_report,
    /* The one predicate: `save` writes flash, and the write stalls the loop. */
    .disarmed = cli_disarmed,
    .proto_report = proto_report,
    .link_report = link_report,
    .on_change = parameters_changed,
};

static void banner(void)
{
    /* The text lives in ak_cli.c so that the `banner` command prints the same
     * bytes this does. It is printed here before a host can be attached, which
     * is why the command exists at all: see ak_banner_print(). */
    ak_banner_print(ak_console_printf);
}

/* A fault that reset the board is the first thing worth knowing about, so it is
 * printed before anything else that could go wrong. */
static void fault_report(void)
{
    if (!ak_fault_present()) {
        ak_console_write("faults: none recorded\n");
        return;
    }

    ak_console_printf("fault: %u recorded, last pc 0x%08x lr 0x%08x psr 0x%08x\n",
                      ak_fault.count, ak_fault.pc, ak_fault.lr, ak_fault.psr);
    ak_console_printf("       cfsr 0x%08x hfsr 0x%08x mmfar 0x%08x bfar 0x%08x\n",
                      ak_fault.cfsr, ak_fault.hfsr, ak_fault.mmfar, ak_fault.bfar);
    ak_console_printf("       r0 0x%08x r1 0x%08x r2 0x%08x r3 0x%08x r12 0x%08x\n",
                      ak_fault.r0, ak_fault.r1, ak_fault.r2, ak_fault.r3,
                      ak_fault.r12);
    /* The frame's address, because the words above are a copy: the originals
     * are still on the stack, and this is where to look for them. */
    ak_console_printf("       frame 0x%08x (the stacked registers are here)\n",
                      ak_fault.frame);
    ak_fault_clear();
}

static void selftest(void)
{
    ak_console_write("selftest:\n");
    int failed = ak_flight_selftest(ak_console_printf);
    ak_console_printf("selftest: %s\n\n", failed ? "FAILED" : "passed");
}

#if AK_BOOT_USB_PUMP_MS
/*
 * Drive the console's polled door for a while, before the banner.
 *
 * The long version of why is beside the flag in ak_board.h. The short one: a
 * board whose console door is polled is silent and deaf on that wire until the
 * main loop starts polling it, because the main loop is the only caller. From
 * the host's side that is indistinguishable from a board that faulted - and it
 * hides the case where the board actually did, since the bytes announcing the
 * fault queue up in a ring that a stopped CPU never drains.
 *
 * This runs after the millisecond tick exists, so the window is measured with
 * the firmware's own clock and not with a guess about how long a poll takes.
 * A counted loop would have to guess exactly that, and a guess wrong by an
 * order of magnitude turns a five-second instrument into an eighty-second one.
 */
static void console_pump(uint32_t ms)
{
    uint32_t start = ak_time_ms();
    char     byte;

    /* Unsigned subtraction, so this is correct across the tick's wrap without
     * knowing where in the count the boot happened to land. */
    while ((uint32_t)(ak_time_ms() - start) < ms) {
        (void)ak_board_console_poll_rx(&byte);
    }
}
#endif

#if AK_USB_TRACE
/*
 * Blink the USB trace out of the status pin, forever.
 *
 * Why the numbers are the numbers, and why an LED is the instrument at all, is
 * beside the flag in ak_board.h. This is the reading half: it takes the
 * counters once, at ten seconds, and then blinks them in a fixed order until
 * somebody pulls the power.
 *
 * Ten seconds is the host's budget, not ours. Every boot this bench has
 * recorded ends with the host giving up after four attempts at a device
 * descriptor, which takes a couple of seconds - so sampling at ten is sampling
 * after the conversation is over, and the counters are the whole of it rather
 * than the first request of it.
 *
 * The order, and it is the order because the first number is the hypothesis and
 * the other three are what is left if it is wrong:
 *
 *   1. the system clock, in units of 20 MHz (8 = the 168 MHz a good 12 or 8 MHz
 *      crystal gives; 1 = the 16 MHz HSI fallback, or anything else the clock
 *      layer could not lift)
 *   2. bus resets the core saw
 *   3. setup packets handed to the stack
 *   4. control transfers the stack started
 *
 * Read as the boot tell-tale is read: N blinks, three rounds, 120 ms on and
 * 160 ms off - so a group is 840*N + 2100 ms, and the rounds are what tells a
 * miscount from a number. Zero is not "no blinks" - nobody can count a silence
 * - so it is one 1500 ms flash, which no group above zero contains. The groups
 * are separated by a 2.5 s dark pause (4 s before it starts over), so a person
 * can tell where one ends without a stopwatch.
 *
 * One pass and then it returns, because the lamp is no longer the only reader:
 * the same numbers have just been written to flash, and the caller follows this
 * with a hand-over to the ROM bootloader, which puts them on the wire where a
 * host can fetch them. The pass is here for the person at the bench, not for
 * the record - which is why one is enough, and why the numbers are sampled
 * once and blinked from the sample rather than read live. Ten seconds in, the
 * counters are already settled: the host gave up on the device descriptor a
 * couple of seconds after the pull-up.
 */
static ak_boot_beat_t trace_beats[AK_BOOT_BEATS_MAX];

static void trace_gap(uint32_t ms)
{
    ak_board_led_set(0);
    ak_delay_ms(ms);
}

static void trace_blink(unsigned value)
{
    if (value == 0u) {
        ak_board_led_set(1);
        ak_delay_ms(1500u);
        ak_board_led_set(0);
        return;
    }

    /* The plan tops out at thirteen, and a group longer than that is not
     * readable anyway - but a clamped number that still looks like a number is
     * worse than no number, so anything past the count is blinked as the
     * maximum, and the counters this is pointed at cannot reach it: a
     * sixteen-bit setup count at ten seconds is a bus that has gone mad. */
    if (value > (unsigned)AK_BOOT_STAGE_COUNT) {
        value = (unsigned)AK_BOOT_STAGE_COUNT;
    }

    int n = ak_boot_blink_plan((int)value, trace_beats, AK_BOOT_BEATS_MAX);
    if (n > 0) {
        ak_boot_run(trace_beats, n, ak_board_led_set, ak_delay_ms);
    }
}

static void usb_trace_report(void)
{
    ak_usb_trace_t t;
    unsigned       mhz20;

    ak_usb_trace_get(&t);

    /* 168 MHz is 8.4 units of 20 and 16 MHz is 0.8, so the division is what
     * separates the two clocks the clock layer can actually land on - and the
     * clamp is what keeps the fallback from reading as zero. */
    mhz20 = (unsigned)(t.sysclk_hz / 20000000u);
    if (mhz20 < 1u) {
        mhz20 = 1u;
    }

    trace_blink(mhz20);
    trace_gap(2500u);
    trace_blink(t.resets);
    trace_gap(2500u);
    trace_blink(t.setups);
    trace_gap(2500u);
    trace_blink(t.sends);
    trace_gap(4000u);
}
#endif
/*
 * The most bytes one pass has taken off any one link.
 *
 * The quota's claim is about a pass, so the count has to be made where a pass
 * is defined - here. The simulator cannot make it: its clock and its pass
 * boundary are the same event (the console poll that found nothing), so a link
 * busy enough to matter is exactly a link that stops the boundary from being
 * marked, and the count it produced was bytes-between-clock-ticks wearing the
 * name bytes-per-pass. It read 200000 for a console the sim had fed 200000
 * bytes, whether or not the firmware was bounded.
 *
 * This one cannot miss a pass: it is incremented by the same call that takes
 * the byte, so a byte taken without a count is not expressible. What it can
 * do is read zero if the counting itself is removed, which is why the
 * simulator's check requires both that it is not zero and that it is within
 * the quota (trap 61).
 */
static uint32_t drain_max_bytes[AK_LINK_COUNT];

static int drain_count(ak_link_t link, unsigned *drained)
{
    if (++*drained > drain_max_bytes[link]) {
        drain_max_bytes[link] = *drained;
    }
    return *drained >= AK_DRAIN_QUOTA;
}

uint32_t ak_main_drain_max(ak_link_t link)
{
    return link < AK_LINK_COUNT ? drain_max_bytes[link] : 0u;
}

int ak_firmware_main(void)
{
    ak_board_init();
    ak_time_init();
    ak_boot_mark(AK_BOOT_TICK);
#if AK_BOOT_USB_PUMP_MS
    console_pump(AK_BOOT_USB_PUMP_MS);
#endif
    banner();
    ak_boot_mark(AK_BOOT_BANNER);
    fault_report();
    selftest();
    ak_boot_mark(AK_BOOT_SELFTEST);

    ak_flight_init(&flight, &ak_mixer_quad_x);
    /*
     * And what this board is built into, which is the board's answer and not the
     * core's: it has to land here, between the init above and the table below,
     * because the table's defaults are whatever the fields hold when it is
     * registered - see the comment on ak_flight_param_table.
     *
     * A board whose header cannot drive the default airframe's mix refuses to
     * arm and says so in `preflight` with both numbers in it. That is the right
     * refusal, but it is a refusal nobody can clear on a board whose pads do not
     * change, so the board names its own aircraft instead of the core naming
     * everyone's.
     */
    flight.airframe = ak_board_default_airframe();
    ak_flight_apply_airframe(&flight);
    unsigned count = ak_flight_param_table(&flight, param_items, AK_PARAMS_MAX);
    dshot_khz = 300u;
    count = ak_params_add_u32(param_items, count, "dshot_khz",
                              "150, 300 or 600 (applies immediately)",
                              &dshot_khz, 150u, 600u, AK_PARAM_GROUP_OUTPUTS);
    /* The receiver's protocol, which is a property of the receiver rather than
     * of the aircraft: an ELRS link speaks CRSF, most receivers a person buys
     * speak SBUS, and the two do not run at the same line settings. The number
     * is the protocol's index rather than "0 or 1" so the console's help says
     * which is which. */
    count = ak_params_add_u32(param_items, count, "rc_protocol",
                              "0 crsf, 1 sbus (applies immediately)",
                              &rc_protocol, 0u, 1u, AK_PARAM_GROUP_RECEIVER);
    /*
     * The battery, as four numbers.
     *
     * `vbat_ratio` is the one that has to be right for any of the others to
     * mean anything: it is pack volts per volt at the ADC pin, so a 10k/1k
     * divider is 11.0. It is a parameter rather than a board constant because
     * it is the number a multimeter is used to correct - measure the pack,
     * compare with what `battery` prints, and scale the ratio by the error.
     *
     * `vbat_cells` at 0 counts the cells from the voltage, which is what a
     * pack plugged in by hand wants; a number overrides it for the pack whose
     * count the arithmetic gets wrong.
     */
    ak_battery_init(&battery);
    /*
     * The servos' own plumbing, one set of three per output: reversed for a
     * linkage that goes the other way, a centre trim, and how far the linkage
     * moves at full stick. A quadrotor never touches them - it has no servos -
     * and a wing does not fly until they are right, which is what the output
     * report is for.
     */
    ak_servo_trim_defaults(servo_trim, AK_MAX_SERVOS);
    for (unsigned i = 0; i < AK_MAX_SERVOS; i++) {
        static const char *const names[AK_MAX_SERVOS][3] = {
            { "servo1_reverse", "servo1_trim_us", "servo1_travel_us" },
            { "servo2_reverse", "servo2_trim_us", "servo2_travel_us" },
        };

        count = ak_params_add_u32(param_items, count, names[i][0],
                                  "1 when this servo's linkage moves the "
                                  "surface the other way",
                                  &servo_trim[i].reversed, 0u, 1u, AK_PARAM_GROUP_OUTPUTS);
        count = ak_params_add_float(param_items, count, names[i][1],
                                    "centre, in microseconds from 1500",
                                    &servo_trim[i].trim_us, 0,
                                    (float)-AK_SERVO_MAX_TRIM_US,
                                    (float)AK_SERVO_MAX_TRIM_US, AK_PARAM_GROUP_OUTPUTS);
        count = ak_params_add_u32(param_items, count, names[i][2],
                                  "how far the linkage moves at full stick, "
                                  "in microseconds",
                                  &servo_trim[i].travel_us,
                                  AK_SERVO_MIN_TRAVEL_US,
                                  AK_SERVO_MAX_TRAVEL_US, AK_PARAM_GROUP_OUTPUTS);
    }
    count = ak_params_add_float(param_items, count, "vbat_ratio",
                                "pack volts per volt at the ADC pin (10k/1k = 11)",
                                &battery.divider_ratio, 3, 1.0f, 50.0f, AK_PARAM_GROUP_POWER);
    count = ak_params_add_u32(param_items, count, "vbat_cells",
                              "cells, or 0 to count them from the voltage",
                              &battery.cells_override, 0u, 8u, AK_PARAM_GROUP_POWER);
    count = ak_params_add_float(param_items, count, "vbat_warn_cell",
                                "volts a cell that says the pack is low",
                                &battery.warn_cell_v, 3, 1.0f, 5.0f, AK_PARAM_GROUP_POWER);
    count = ak_params_add_float(param_items, count, "vbat_min_cell",
                                "volts a cell that says the pack is done",
                                &battery.critical_cell_v, 3, 1.0f, 5.0f, AK_PARAM_GROUP_POWER);
    count = ak_params_add_u32(param_items, count, "battery_rth",
                              "1 brings it home when the pack is done",
                              &battery_rth, 0u, 1u, AK_PARAM_GROUP_POWER);
    align_roll_deg = 0.0f;
    align_pitch_deg = 0.0f;
    align_yaw_deg = 0.0f;
    count = ak_params_add_float(param_items, count, "align_roll_deg",
                                "mounting rotation, board to airframe",
                                &align_roll_deg, 1, -180.0f, 180.0f, AK_PARAM_GROUP_SENSORS);
    count = ak_params_add_float(param_items, count, "align_pitch_deg",
                                "mounting rotation, board to airframe",
                                &align_pitch_deg, 1, -180.0f, 180.0f, AK_PARAM_GROUP_SENSORS);
    count = ak_params_add_float(param_items, count, "align_yaw_deg",
                                "mounting rotation, board to airframe",
                                &align_yaw_deg, 1, -180.0f, 180.0f, AK_PARAM_GROUP_SENSORS);
    for (int i = 0; i < 3; i++) {
        gyro_bias_dps[i] = 0.0f;
    }
    count = ak_params_add_float(param_items, count, "gyro_bias_roll",
                                "measured by 'calibrate', dps",
                                &gyro_bias_dps[0], 3, -20.0f, 20.0f, AK_PARAM_GROUP_SENSORS);
    count = ak_params_add_float(param_items, count, "gyro_bias_pitch",
                                "measured by 'calibrate', dps",
                                &gyro_bias_dps[1], 3, -20.0f, 20.0f, AK_PARAM_GROUP_SENSORS);
    count = ak_params_add_float(param_items, count, "gyro_bias_yaw",
                                "measured by 'calibrate', dps",
                                &gyro_bias_dps[2], 3, -20.0f, 20.0f, AK_PARAM_GROUP_SENSORS);
    /* Six positions, as six pairs. The bias is in g and the scale is
     * dimensionless - a part that reads 2% low on an axis gets 1.02 there,
     * which is what makes 'accel_scale_y' readable as a number. */
    static const char *bias_names[3] = { "accel_bias_x", "accel_bias_y",
                                         "accel_bias_z" };
    static const char *scale_names[3] = { "accel_scale_x", "accel_scale_y",
                                          "accel_scale_z" };
    for (int i = 0; i < 3; i++) {
        accel_bias_g[i] = 0.0f;
        accel_scale[i] = 1.0f;
        count = ak_params_add_float(param_items, count, bias_names[i],
                                    "measured by 'calibrate accel', g",
                                    &accel_bias_g[i], 4, -1.0f, 1.0f, AK_PARAM_GROUP_SENSORS);
        count = ak_params_add_float(param_items, count, scale_names[i],
                                    "measured by 'calibrate accel'",
                                    &accel_scale[i], 4, 0.5f, 1.5f, AK_PARAM_GROUP_SENSORS);
    }
    ak_gyro_cal_init(&gyro_cal, AK_CAL_SAMPLES, AK_CAL_MAX_STILL,
                     AK_CAL_MAX_BIAS);
    /* The accelerometer's calibration asks the same question of the same
     * aircraft - is it being held still - so it uses the same threshold. */
    ak_accel_cal_init(&accel_cal, AK_CAL_SAMPLES, AK_CAL_MAX_STILL);
    /* And the measurement the flight loop runs for itself starts here, at
     * power-up: the aircraft is on the ground and still for as long as it
     * takes 200 samples to arrive, and the bias it measures is the one the
     * part has at the temperature of the day rather than the bench's. */
    ak_gyro_cal_start(&gyro_cal);
    ak_log_init(&blackbox);
    ak_log_set_decimation(&blackbox, AK_LOG_EVERY);

    /* The long log: retained memory if the board has any, ordinary memory if
     * not. `resumed` is what the boot report and the preflight say about it -
     * a log from last time is worth knowing about before the aircraft is
     * picked up, and a board that cannot keep one should say so rather than
     * quietly hand back an empty ring. */
    unsigned retained_bytes = 0;
    longlog = ak_board_retained_ram(&retained_bytes);
    if (longlog != 0 && retained_bytes >= sizeof *longlog) {
        longlog_retained = 1;
    } else {
        longlog_retained = 0;
        longlog = &longlog_fallback;
    }
    longlog_kept = ak_log_resume(longlog);
    ak_log_set_decimation(longlog, AK_LOG_EVERY * AK_LONG_EVERY);

    /* And the third log. It comes back with whatever the flash still holds
     * from every run that ever wrote to it, which is the whole point of it -
     * and it is also why the boot report says how much there is. */
    flashlog_state = ak_flashlog_resume(&flashlog, ak_board_log_store());

    ak_gps_init(&gps);
    ak_board_gps_init();
    ak_nav_init(&nav);
    ak_launch_init(&launch);
    ak_proto_init(&proto);
    ak_proto_init(&net_proto);
    /* The console serves requests and does not stream: it is the wire a person
     * types at, and frames arriving among their keystrokes is a console nobody
     * can use. The network link is the one that pushes, which is why the
     * subscribe command answers 0 here and a rate there. */
    net_proto.can_stream = 1u;
    proto_io.params = &params;
    /* What this build can answer, and nothing it cannot.
     *
     * APPLIES_ON_WRITE is set because the next line wires `on_change` to
     * `parameters_changed`, which is the act the bit names - so it is a
     * reading of this file rather than a plan.
     *
     * GATES_ON_ARMED is set, and it is set by the line below that wires
     * `writable` into the protocol. Every write route on this wire - `set`,
     * `save`, `default` - consults it at the moment of the request, and it is
     * the same `ak_flight_config_writable` predicate the console's `save` and
     * the airborne reload paths use. That predicate is why the bit is a reading
     * of this file rather than an intention: the guard existed before this
     * milestone, on every route except the one that arrives over a socket.
     *
     * PARAM_DEFAULT is set because ak_proto.c has a `case` for it and that case
     * consults the same gate, which is what makes it safe to advertise: the
     * opcode that can wipe a configuration is the one that most needs the
     * refusal, and it would be worse to have it than not to have it if it did
     * not.
     *
     * PARAM_INFO is set because ak_proto.c has a `case` for it and for
     * PARAM_HELP, and that is a reading of this build rather than a plan. The
     * two travel under one bit deliberately: they are one answer in two shapes -
     * what a parameter is, and the prose beside it - and a client that could
     * page the table but not fetch a row's help would be drawing a form with no
     * explanation of it. A board with one and not the other is not a thing worth
     * being able to say. */
    proto_io.features = AK_PROTO_FEATURE_APPLIES_ON_WRITE |
                        AK_PROTO_FEATURE_PARAM_INFO |
                        AK_PROTO_FEATURE_PARAM_DEFAULT |
                        AK_PROTO_FEATURE_GATES_ON_ARMED |
                        AK_PROTO_FEATURE_RC_CHANNELS |
                        AK_PROTO_FEATURE_SENSOR_INFO;
    proto_io.status = proto_status;
    /* The same callback the console gets, set from the same function, so a
     * parameter set over either link reaches the aircraft identically. Until
     * this line existed, a wire `set` changed the table and nothing else. */
    proto_io.on_change = parameters_changed;
    proto_io.save = proto_save;
    /* The armed-state policy's answer, from the same predicate the console's
     * `save` passes down to the storage. The protocol asks it before every
     * write rather than the board pushing the state at connect, because a link
     * that stays up across an arming is the ordinary case and a gate latched at
     * connect is a gate that is wrong one second later. */
    proto_io.writable = proto_writable;
    /* Set on every board, because every board header defines the `ak_board_rc_`
     * family and this main.c always has a receiver. A board whose receiver port
     * has nothing plugged into it answers with LINK clear and counts of zero,
     * which is true; the "this board has no receiver input" answer is reserved
     * for a build that genuinely lacks one and leaves this null. */
    proto_io.rc_state = proto_rc_state;
    /* Set on every board too, and the same argument: the five topics are this
     * main.c's five sensor reports, and every board in this tree builds those.
     * A board that fitted nothing answers `present` clear with the reason the
     * console prints, which is its own true answer - the null callback is
     * reserved for a build with no sensor reporting at all. */
    proto_io.sensor_state = proto_sensor_state;
    proto_io.log_count = proto_log_count;
    proto_io.log_record = proto_log_record;
    proto_io.ctx = 0;
    rth_enable = 0;
    count = ak_params_add_u32(param_items, count, "rth_enable",
                              "0 to stop on a lost link, 1 to come home",
                              &rth_enable, 0u, 1u, AK_PARAM_GROUP_NAVIGATION);
    /* What the navigator will believe. INAV's floor and INAV's range, because
     * the reason for it is a property of GPS receivers rather than of this
     * firmware: low satellite counts come with very inaccurate positions. */
    count = ak_params_add_u32(param_items, count, "gps_min_sats",
                              "satellites a fix needs to be navigated on",
                              &gps.min_sats, 5u, 10u, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "rth_cruise",
                                "throttle to hold while returning",
                                &nav.cruise, 2, 0.0f, 1.0f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "rth_course_kp",
                                "roll per degree of course error",
                                &nav.course_kp, 4, 0.0f, 0.2f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "rth_alt_kp",
                                "pitch per metre of altitude error",
                                &nav.alt_kp, 4, 0.0f, 0.1f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "rth_alt_ki",
                                "pitch per metre-second of altitude error (the standing pitch)",
                                &nav.alt_ki, 5, 0.0f, 0.01f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "rth_loiter_roll",
                                "bank held while circling at home",
                                &nav.loiter_roll, 2, 0.0f, 1.0f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "rth_arrive_m",
                                "how close counts as arrived",
                                &nav.arrive_m, 0, 10.0f, 1000.0f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "rth_min_alt_m",
                                "never hold below this while returning (0 = off)",
                                &rth_min_alt_m, 0, 0.0f, 500.0f, AK_PARAM_GROUP_NAVIGATION);
    /* The quadrotor's return is a different manoeuvre, so it has its own three
     * numbers: how hard it leans on the position error, how fast it is allowed
     * to come home, and how high above the ground home was set it settles. The
     * profile itself follows the airframe - a quad-X and an elevon wing cannot
     * share one - which `set airframe` is what selects. */
    count = ak_params_add_float(param_items, count, "quad_return_kp",
                                "speed asked for per metre from home, m/s per m",
                                &nav.quad_kp, 3, 0.05f, 2.0f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "quad_return_speed",
                                "fastest the return will ask for, m/s",
                                &nav.quad_speed, 1, 1.0f, 20.0f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "quad_arrive_m",
                                "how close home before the return descends",
                                &nav.quad_arrive_m, 0, 3.0f, 200.0f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "quad_hover_m",
                                "metres above home the quadrotor settles at",
                                &nav.quad_hover_m, 1, 0.5f, 30.0f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "quad_alt_ki",
                                "standing throttle learned per (m/s)-second of climb-rate error",
                                &nav.quad_alt_ki, 3, 0.0f, 0.5f, AK_PARAM_GROUP_NAVIGATION);
    /*
     * The rangefinder, as three numbers.
     *
     * `range_land_mm` is how close the ground has to be before the aircraft
     * counts as on it, and `range_agree_m` is how far the part and the height
     * estimate may disagree before neither is believed - the guard against a
     * part looking at a hedge, a slope or the roof of a car from twenty metres
     * up. `quad_hold_land_s` is the one that changes what the aircraft *does*:
     * seconds of holding with no position fix before a quadrotor comes down
     * where it is. Zero means it never does, and the hold goes on being a
     * hold, which is the behaviour this firmware had before any of this.
     */
    count = ak_params_add_u32(param_items, count, "range_land_mm",
                              "how close the ground counts as landed, mm",
                              &range_land_mm, 0u, 2000u, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "range_agree_m",
                                "how far the rangefinder and the height may disagree",
                                &range_agree_m, 1, 0.5f, 20.0f, AK_PARAM_GROUP_SENSORS);
    count = ak_params_add_u32(param_items, count, "quad_hold_land_s",
                              "seconds of holding with no fix before a quadrotor lands in place (0 = never)",
                              &nav.hold_land_s, 0u, 600u, AK_PARAM_GROUP_NAVIGATION);
    fence_enable = 0;
    count = ak_params_add_u32(param_items, count, "fence_enable",
                              "1 brings it home if it leaves the fence",
                              &fence_enable, 0u, 1u, AK_PARAM_GROUP_FAILSAFE);
    count = ak_params_add_float(param_items, count, "fence_radius_m",
                                "how far from home it may go (0 = off)",
                                &nav.fence_m, 0, 0.0f, 5000.0f, AK_PARAM_GROUP_FAILSAFE);
    count = ak_params_add_float(param_items, count, "fence_ceiling_m",
                                "how high above home it may go (0 = off)",
                                &nav.fence_ceiling_m, 0, 0.0f, 5000.0f, AK_PARAM_GROUP_FAILSAFE);

    /* The mission, as parameters: a waypoint is two numbers that must stay
     * together, and the table is where numbers that survive a reboot live.
     * Degrees rather than the protocol's 1e-7 units, because these are typed
     * and read by people and by configurators, and apply_parameters converts
     * once. Seven decimals is about a centimetre, which is far below what the
     * receiver is good for. */
    static const char *lat_name[AK_NAV_WAYPOINTS] = {
        "wp0_lat", "wp1_lat", "wp2_lat", "wp3_lat",
    };
    static const char *lon_name[AK_NAV_WAYPOINTS] = {
        "wp0_lon", "wp1_lon", "wp2_lon", "wp3_lon",
    };
    for (int i = 0; i < AK_NAV_WAYPOINTS; i++) {
        wp_lat_deg[i] = 0.0f;
        wp_lon_deg[i] = 0.0f;
        count = ak_params_add_float(param_items, count, lat_name[i],
                                    "mission waypoint, degrees",
                                    &wp_lat_deg[i], 7, -90.0f, 90.0f, AK_PARAM_GROUP_NAVIGATION);
        count = ak_params_add_float(param_items, count, lon_name[i],
                                    "mission waypoint, degrees",
                                    &wp_lon_deg[i], 7, -180.0f, 180.0f, AK_PARAM_GROUP_NAVIGATION);
    }
    wp_count = 0;
    count = ak_params_add_u32(param_items, count, "wp_count",
                              "how many waypoints to fly, in order",
                              &wp_count, 0u, AK_NAV_WAYPOINTS, AK_PARAM_GROUP_NAVIGATION);
    mission_channel = 0;
    count = ak_params_add_u32(param_items, count, "mission_channel",
                              "RC channel for the mission switch (0 = none)",
                              &mission_channel, 0u, AK_RC_CHANNELS, AK_PARAM_GROUP_NAVIGATION);
    /*
     * The launch, which is configuration rather than a manoeuvre: the channel
     * that selects it, and the three numbers the manoeuvre is (the numbers
     * themselves are INAV's defaults - see ak_launch.h - and unlike theirs this
     * one is flown by a switch rather than by a throw detector, which is
     * written down where the module is).
     */
    launch_channel = 0;
    count = ak_params_add_u32(param_items, count, "launch_channel",
                              "RC channel for the hand-launch switch (0 = none)",
                              &launch_channel, 0u, AK_RC_CHANNELS, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "launch_throttle",
                                "throttle held during a launch",
                                &launch.cfg.throttle, 2, 0.0f, 1.0f, AK_PARAM_GROUP_NAVIGATION);
    count = ak_params_add_float(param_items, count, "launch_climb_deg",
                                "climb attitude held during a launch, degrees",
                                &launch.cfg.climb_deg, 0, 0.0f, 45.0f, AK_PARAM_GROUP_NAVIGATION);
    /* Seconds in the parameter and milliseconds in the manoeuvre, and the
     * default is the module's own rather than a second copy of it: registering
     * this at zero is what made the first build of it end its launch on the
     * first pass. */
    launch_timeout_s = launch.cfg.timeout_ms / 1000u;
    count = ak_params_add_u32(param_items, count, "launch_timeout_s",
                              "seconds a launch may fly the aircraft",
                              &launch_timeout_s, 0u, 60u, AK_PARAM_GROUP_NAVIGATION);
    /* And whatever the board itself has to offer. On the ESP32 that is the
     * Wi-Fi credentials, which is the difference between a network that is
     * compiled in and one the aircraft was told; on either of the two
     * microcontrollers it is nothing, and an empty hook is the honest shape of
     * that. */
    count = ak_board_param_table(param_items, count);

    /* And whether any of that was dropped. A table that is one parameter too
     * small is a setting that does not exist, and the only way to notice used
     * to be to miss it - which is how the table sat full at 64 while the ESP32
     * grew Wi-Fi credentials that were never there. */
    if (ak_params_overflow() > 0u) {
        ak_console_printf("params: %u parameter%s did not fit the table "
                          "(%u slots) - raise AK_PARAMS_MAX\r\n",
                          ak_params_overflow(),
                          ak_params_overflow() == 1u ? "" : "s",
                          (unsigned)AK_PARAMS_MAX);
    }

    /* The same, for a name too long to be read back out of a saved record.
     * Printed separately because the consequence is different and so is the
     * cure: the parameter is *absent* rather than misconfigured, and the thing
     * to change is the name, not a table size. */
    if (ak_params_long_name() > 0u) {
        ak_console_printf("params: %u name%s longer than %u characters were "
                          "refused - they could never be loaded back\r\n",
                          ak_params_long_name(),
                          ak_params_long_name() == 1u ? " is" : "s are",
                          (unsigned)AK_PARAM_NAME_MAX);
    }

    ak_params_init(&params, param_items, count);
    ak_cli_init(&cli, &cli_io, &params, &flight);
    apply_parameters();
    ak_boot_mark(AK_BOOT_PARAMETERS);
    ak_board_output_init();
    /* And what the board just brought up, handed to the flight core: the mix
     * the selected airframe needs and the outputs this board has are compared
     * before anything can arm. It is said here, once, right after the outputs
     * exist - a board that never says leaves the aircraft unable to arm, which
     * is the safe direction for a number nobody has stated. */
    if (ak_board_output_ready()) {
        unsigned motors = 0u;
        unsigned servos = 0u;

        ak_board_output_shape(&motors, &servos);
        ak_flight_set_board_outputs(&flight, motors, servos);
    } else {
        /*
         * The board's timers did not come up, so it states nothing about what
         * it can drive - and the arming gate refuses with both numbers in the
         * sentence rather than an aircraft that arms and writes DShot frames
         * into channels that do not exist. The preflight says which half is
         * missing, and a board in this state has one job: say so.
         *
         * This is asked *after* `ak_board_output_init()` and before anything
         * else touches the outputs, and the ESP32's answer is what made it
         * necessary: it used to call its outputs ready as soon as the servo
         * timer existed, which is a timer a quadrotor does not use.
         */
        ak_flight_set_board_outputs(&flight, 0u, 0u);
    }
    ak_boot_mark(AK_BOOT_OUTPUTS);
    ak_board_rc_init();
    /* Both parsers exist from the start; which one sees the bytes is a
     * parameter, and the board has to be told as well because CRSF and SBUS do
     * not run at the same baud rate, parity or stop bits. */
    ak_rc_receiver_init(&receiver);
    ak_rc_receiver_set_protocol(&receiver, rc_protocol);
    ak_board_rc_set_protocol(rc_protocol);
    ak_crsf_tlm_init(&crsf_tlm);
    next_crsf_ms = ak_time_ms();
    /* The flight pack. setup() configures the pin and the ADC; whether the
     * board has both is the board's answer, not an assumption - and a board
     * that cannot measure one says so rather than reporting zero volts, which
     * would read as an empty battery. */
    ak_board_battery_init();
    battery_ready = ak_board_battery_ready();
    battery_last_ms = ak_time_ms();
    battery_next_ms = battery_last_ms;
    ak_boot_mark(AK_BOOT_BATTERY);
    ak_board_imu_init();
    /* A board without a sensor bus is a port in progress, not a fault: the
     * flight core stays in failsafe and the preflight report says why. */
    imu_ok = ak_board_imu_bus() != 0 &&
             ak_imu_open(&imu, ak_board_imu_bus(), ak_console_printf) == 0;
    ak_boot_mark(AK_BOOT_IMU);
    /* And the barometer, if there is one. A board without one still flies: the
     * gps gives altitude, badly, and the console says which of the two this
     * machine has. */
    ak_altitude_init(&altitude);
    baro_ok = ak_board_baro_bus() != 0 &&
              ak_baro_open(&baro, ak_board_baro_bus(), ak_console_printf) == 0;
    baro_next_ms = ak_time_ms();
    /* And the rangefinder, on its own address on the same bus. A board with
     * neither of these still flies: the console says which of the three
     * measurements is missing and the landing rule falls back to the one it
     * had before there was a rangefinder. */
    range_ok = ak_board_range_bus() != 0 &&
               ak_rangefinder_open(&range, ak_board_range_bus(),
                                   ak_console_printf) == 0;
    ak_boot_mark(AK_BOOT_ALTITUDE);

    ak_console_write("console: type 'help'\r\n");
    (void)ak_cli_run(&cli, "load"); /* says what it found, either way */

    /* And now the board's network, which waits for that load on purpose: what
     * it joins - or whether it carries its own network - is in the parameters
     * that were just read, and a radio started earlier would be one that came
     * up on the previous build's network or on none. */
    ak_board_net_start();

    int boot_problems = preflight_run(ak_console_printf, 0);
    if (boot_problems > 0) {
        ak_console_printf("preflight: %d problem%s - type 'preflight'\r\n",
                          boot_problems, boot_problems == 1 ? "" : "s");
    } else {
        ak_console_write("preflight: the machine is what the firmware thinks it is\r\n");
    }
    ak_boot_mark(AK_BOOT_PREFLIGHT);
    /* A log from the run before is the first thing anybody should know about
     * after an unexplained reset, and it is worth one line at boot rather than
     * a command somebody has to remember to type. */
    if (longlog_kept && longlog->count > 0u) {
        ak_log_record_t newest;
        unsigned last_ms = 0;
        if (ak_log_get(longlog, (uint16_t)(longlog->count - 1u), &newest)) {
            last_ms = newest.time_ms;
        }
        /* The timestamps are the *previous* run's uptime, so this says how far
         * into that run the log reaches - which is the useful half of "when
         * did this happen" when the clock has just restarted. */
        ak_console_printf("long log: %u records from the run before, to %u ms "
                          "into it - 'log long'\r\n",
                          longlog->count, last_ms);
    }
    /* And the log that does not care whether this was a reset or a walk back
     * from the crash: it is all the runs, and the only one worth reading after
     * the battery came out. */
    if (flashlog_state == AK_FLASHLOG_OK && ak_flashlog_count(&flashlog) > 0u) {
        ak_console_printf("flash log: %u records from every run - 'log flash'\r\n",
                          ak_flashlog_count(&flashlog));
    } else if (flashlog_state == AK_FLASHLOG_NEEDS_ERASE) {
        ak_console_write("flash log: no room - will erase a sector on the "
                         "ground\r\n");
    }
    ak_console_write("boot: ok\r\n");

    for (int i = 0; i < AK_BOOT_BLINKS; i++) {
        ak_board_led_set(1);
        ak_delay_ms(80);
        ak_board_led_set(0);
        ak_delay_ms(120);
    }

    /* The console is up, the boot report is out, and the board is listening:
     * this is the line that says so, and the one a script waits for before it
     * types anything. */
    ak_cli_prompt(&cli);

    uint32_t now = ak_time_ms();
    uint32_t next_led = now + AK_LED_PERIOD_MS;
    uint32_t next_loop = now + AK_FLIGHT_LOOP_MS;
    uint32_t next_heartbeat = now + AK_HEARTBEAT_MS;

#if AK_USB_TRACE
    /* One shot, at ten seconds - see the note on usb_trace_report(). It sits
     * here rather than in the boot path because the counters it reads only mean
     * anything once the loop has been polling the port, and the first ten
     * seconds are when the host does its four attempts. */
    int trace_reported = 0;
#endif

    for (;;) {
        char byte;

#if AK_USB_TRACE
        if (!trace_reported && ak_time_ms() >= 10000u) {
            trace_reported = 1;

            /* Written down first, and its result not checked: a board that
             * cannot save still has a lamp, and one that saved still has to be
             * read. A host that finds no record reads the page itself and sees
             * why - all-blank is a save that never happened, and anything else
             * is a page that has moved. See ak_board_trace_save(). */
            (void)ak_board_trace_save();

            /* And hand the part to the ROM, now that the ROM can be reached
             * without a hand on the board: the host reads the record and
             * flashes the next image, and a diagnostic loop no longer costs a
             * BOOT0 press per turn. The lamp report is the fallback when the
             * board cannot get there - ak_arch_bootloader() never returns on
             * success, so reaching the line below means the hand-over did not
             * happen.
             *
             * This call was taken out on 2026-09-27, when the hand-over was a
             * branch from the running firmware and did not work, and put back
             * by ad766dd with the reset-and-remap path that ak_arch_bootloader()
             * uses now. The comment that used to sit here saying it was gone
             * from this path was left behind by that removal and contradicted
             * the call above it; it is deleted rather than kept, because
             * "no current image attempts it" is false at HEAD.
             *
             * The console's `dfu` command reaches the ROM through the same
             * function - measured on the bench board on 2026-09-30, twice. This
             * path is not measured, and neither measurement separates the
             * reset-and-remap from BOOT0 having been left high. See
             * docs/05-bringup.md 6c. */
        }
#endif

        /*
         * How much of a link one pass may take before the rest of the loop
         * runs, which is the "quota communications" half of B4.
         *
         * The two drains below are driven by the board's receive FIFO, and
         * nothing in them knows or cares how long they have been going: a
         * client that streams without pausing - a telemetry flood, a console
         * being pasted into, a link that has gone noisy - keeps
         * `ak_board_*_poll_rx()` returning true, and the pass never reaches the
         * control gate at the bottom. The aircraft then flies at whatever rate
         * the flood permits, which is the one failure mode where the thing
         * taking the time is not the thing that matters.
         *
         * A quota rather than a deadline check, because the body between here
         * and the gate is bounded: at most this many bytes off each link, then
         * the period-gated slow reads, then the gate. So the control loop is
         * reached at least every AK_DRAIN_QUOTA bytes per link, whatever
         * arrives, and a byte left in the FIFO is picked up on the next pass
         * microseconds later. A command answered a microsecond later is a
         * command answered; an iteration missed is an aircraft that did not
         * fly.
         *
         * The cost of a flood is therefore visible where it should be: the
         * core's own `long_loops` and `max_loop_ms`, which `status` prints.
         */
        unsigned drained = 0u;

        while (ak_board_console_poll_rx(&byte)) {
            /*
             * The binary protocol and the human console share this port, and
             * ak_console_link_feed is now the whole of the rule for which of
             * them a byte belongs to.
             *
             * It used to be written out here - "an idle parser and a byte that
             * is not a frame sync means somebody is typing" - and that reading
             * of it was wrong twice over, in ways this file cannot show:
             *
             *   - `ak_proto_idle` asked whether the parser was between frames,
             *     which is not the same question as whether the wire is free.
             *     A frame that stops part way leaves the parser non-idle until
             *     the next byte arrives and the gap expires, and a text byte
             *     sent in that window was swallowed by the protocol path: the
             *     first character of a command typed after half a frame simply
             *     disappeared.
             *   - The tail of a frame abandoned for a bad length arrived at an
             *     idle parser, so it was typed at the console: its printable
             *     bytes into the line buffer, and any 0x0D or 0x0A among them
             *     into a prompt nobody asked for - which is what a script reads
             *     to mean "the last command has finished".
             *
             * The rule lives in ak_console_link.c with those two comments
             * beside it, and tests/test_proto.c drives this same function, so
             * the next person to change it does not have to find it here first.
             */
            uint8_t response[AK_PROTO_FRAME_MAX];
            unsigned written = ak_console_link_feed(&proto, &proto_io, &cli,
                                                    (uint8_t)byte, ak_time_ms(),
                                                    response, sizeof response);
            if (written > 0) {
                ak_console_write_raw((const char *)response, written);
            }
            /* Both halves of the link count. This used to `continue` down the
             * typing path, which skipped the increment - so the quota bounded
             * the binary protocol and left unbounded the one case its own
             * comment names, a person pasting at the console. The simulator
             * measured 200000 bytes taken in a single pass (trap 60). */
            if (drain_count(AK_LINK_CONSOLE, &drained)) {
                break;      /* the rest of the FIFO waits for the next pass */
            }
        }

        /* The board's network, if it has one: the same protocol, a different
         * link. The console is not on this one, so a byte that is not a frame
         * sync is noise from a client that has lost the plot and is dropped
         * rather than fed to a command line nobody is typing at. */
        /* A link that has just been connected starts from nothing: the
         * half-frame the last client left behind is not this one's, and
         * neither is the telemetry stream it asked for. Without this a client
         * that reconnects is sent a stream it never subscribed to, before its
         * first request has been answered - which the client in this
         * repository reported as a reply to the wrong command, and it was
         * right. */
        {
            int connected_now = ak_board_net_connected();
            if (connected_now && !net_was_connected) {
                ak_proto_init(&net_proto);
                /* A fresh parser for a new client, and this link is still the
                 * one that streams - see where the console's is set up. */
                net_proto.can_stream = 1u;
            }
            net_was_connected = connected_now;
        }
        drained = 0u;
        while (ak_board_net_poll_rx(&byte)) {
            uint8_t response[AK_PROTO_FRAME_MAX];
            unsigned written = ak_proto_feed(&net_proto, &proto_io,
                                             (uint8_t)byte, ak_time_ms(),
                                             response, sizeof response);
            if (written > 0) {
                ak_board_net_write((const char *)response, written);
            }
            if (drain_count(AK_LINK_NET, &drained)) {
                break;      /* the same quota, for the same reason */
            }
        }

        /* Telemetry, for a client that asked for a stream. The rate is per
         * link and lives in that link's parser, and the frame is built from
         * the same status the STATUS reply carries - one source, so a stream
         * and a poll can never disagree about what the aircraft is doing. */
        if (net_proto.telemetry_hz > 0u && ak_board_net_connected()) {
            if ((int32_t)(ak_time_ms() - next_telemetry_ms) >= 0) {
                uint8_t frame[AK_PROTO_FRAME_MAX];
                unsigned length = ak_proto_telemetry_frame(&proto_io,
                                                           ak_time_ms(), frame,
                                                           sizeof frame);
                if (length > 0) {
                    ak_board_net_write((const char *)frame, length);
                    telemetry_sent++;
                }
                next_telemetry_ms = ak_time_ms() +
                                    (1000u / net_proto.telemetry_hz);
            }
        }

        /* The receiver and the GPS get the same quota. Their byte rates are
         * bounded by their own hardware rather than by a client, but "bounded
         * by the part" is an assumption about a part that may be faulty or
         * unplugged and babbling, which is exactly the assumption the two
         * links above are not allowed to make either. A receiver cut off
         * mid-frame is a framing error it already counts and reports. */
        uint8_t rc_byte;
        drained = 0u;
        while (ak_board_rc_poll(&rc_byte)) {
            (void)ak_rc_receiver_feed(&receiver, rc_byte, ak_time_ms());
            if (drain_count(AK_LINK_RC, &drained)) {
                break;
            }
        }

        uint8_t gps_byte;
        drained = 0u;
        while (ak_board_gps_poll(&gps_byte)) {
            if (ak_gps_feed(&gps, gps_byte, ak_time_ms())) {
                float msl_m = (float)gps.fix.alt_msl_mm / 1000.0f;
                uint32_t stamp_ms = ak_time_ms();

                gps_fixes++;
                /*
                 * A fix is a new altitude, and the altitude estimate wants it
                 * with the time since the last one: the GPS is the slow,
                 * absolute half of the answer and its correction is a rate,
                 * not a step.
                 */
                if (ak_flight_state(&flight) == AK_FLIGHT_DISARMED &&
                    !altitude.have_gps_reference) {
                    ak_altitude_set_gps_reference(&altitude, msl_m);
                } else if (altitude.have_gps_reference &&
                           gps_last_alt_time_ms != 0u) {
                    ak_altitude_gps(
                        &altitude, msl_m,
                        (float)(stamp_ms - gps_last_alt_time_ms) / 1000.0f);
                }
                gps_last_alt_time_ms = stamp_ms;
            }
            if (drain_count(AK_LINK_GPS, &drained)) {
                break;
            }
        }
        gps_configure(ak_time_ms());

        /*
         * The flight pack, at a rate that is about the filter rather than the
         * hardware: a conversion is 23 microseconds and the answer moves on a
         * scale of half a second. Sampling it before anybody asks means the
         * first `battery` at the console already has an answer, and the first
         * reading is the estimate rather than a step towards one.
         */
        if (battery_ready && (int32_t)(now - battery_next_ms) >= 0) {
            uint32_t stamp = ak_time_ms();
            float dt_s = (float)(stamp - battery_last_ms) / 1000.0f;

            battery_next_ms = now + AK_BATTERY_PERIOD_MS;
            battery_last_ms = stamp;
            ak_battery_sample(&battery, ak_board_battery_pin_volts(), dt_s);
        }

        now = ak_time_ms();

        if ((int32_t)(now - next_led) >= 0) {
            next_led += AK_LED_PERIOD_MS;
            ak_board_led_toggle();
        }

        if ((int32_t)(now - next_loop) >= 0) {
            next_loop += AK_FLIGHT_LOOP_MS;
            /* With a sensor on the bus this is a real attitude estimate and
             * the aircraft can arm; without one every sample is invalid and
             * the core holds it in failsafe. Both are the same code path. */
            if (imu_ok) {
                (void)ak_imu_read(&imu, &imu_sample);
                /* What the sensor measured, expressed in the airframe's terms
                 * and with the offset and the scale the six-position
                 * calibration measured taken out of it. */
                ak_align_apply(&align, &imu_sample);
                /*
                 * A calibration runs while the aircraft is *disarmed*: that is
                 * when it is certainly on the ground and certainly not being
                 * flown, and it is where the bias a part has at the
                 * temperature of the day is measured. It is fed the aligned
                 * sample before the bias is taken out, because the accumulator
                 * holds the residual - see ak_gyro_cal.h.
                 *
                 * The threshold is what makes this safe: three degrees a
                 * second, so an aircraft being carried, or one rolling forward
                 * on a launch, has its samples refused rather than averaged in
                 * as if they were the part's offset. (The first version of
                 * this measured at *arm*, with the old twenty-degree
                 * threshold, and a wing's take-off roll was written into the
                 * bias: the fence session, which arms and launches, ended up
                 * fighting its own yaw axis with 13.6 deg/s of elevator.)
                 */
                if (gyro_cal.running &&
                    ak_flight_state(&flight) == AK_FLIGHT_DISARMED) {
                    float dps[3];

                    if (ak_gyro_cal_feed(&gyro_cal, &imu_sample)) {
                        ak_gyro_cal_bias_dps(&gyro_cal, dps);
                        ak_console_printf(
                            "gyro: bias measured while disarmed: %d %d %d "
                            "mdps (%u samples, %u rejected)\r\n",
                            (int)(dps[0] * 1000.0f), (int)(dps[1] * 1000.0f),
                            (int)(dps[2] * 1000.0f), gyro_cal.samples,
                            gyro_cal.rejected);
                    }
                }
                ak_gyro_cal_apply(&gyro_cal, &imu_sample);
                ak_accel_cal_apply(&accel_cal, &imu_sample);
            } else {
                imu_sample.valid = 0;
            }
            imu_sample.time_ms = now;

            /* Home is the take-off point, and the only moment it can be
             * captured without asking is while the aircraft is on the ground
             * with a fix - so that is when it happens. */
            if (!nav.have_home && ak_gps_fix_valid(&gps, now, 2000u) &&
                ak_flight_state(&flight) == AK_FLIGHT_DISARMED) {
                ak_nav_set_home(&nav, gps.fix.lat_e7, gps.fix.lon_e7,
                                altitude_msl_mm());
                ak_console_printf("home: from the first fix while disarmed, "
                                  "%d.%07d, %d.%07d\n",
                                  nav.home_lat_e7 / 10000000,
                                  nav.home_lat_e7 % 10000000,
                                  nav.home_lon_e7 / 10000000,
                                  nav.home_lon_e7 % 10000000);
            }

            /* Decide what the navigator wants before the flight core decides
             * what to do with it - but sense the link first, so both of them
             * are looking at the same answer. The navigator used to read the
             * link a step late, which meant it only ever saw the link go at
             * the same moment the core latched the failsafe, and the return
             * never engaged. */
            (void)ak_flight_link_update(&flight, &receiver.channels, now, 0);
            nav_update(now);
            /* Only while a navigator is flying it: a pilot on the sticks is
             * never disarmed by a barometer. */
            {
                int state = ak_flight_state(&flight);

                flight.landed =
                    (state == AK_FLIGHT_RTH || state == AK_FLIGHT_MANAGED ||
                     state == AK_FLIGHT_DESCEND) &&
                    landing_detector(now);
            }
            ak_flight_step(&flight, &imu_sample, &receiver.channels, now);
            /* And if a pilot is asking to arm and the answer is no, say why -
             * once per attempt, not once per pass. */
            arm_announce();

            /*
             * And once the aircraft is armed, stop measuring.
             *
             * The measurement itself runs while *disarmed*, which is when the
             * aircraft is certainly on the ground and certainly not being
             * flown; the moment the switch comes up the aircraft may be
             * rolling, being carried to a launch, or in the air, and nothing
             * that is moving may be written into a bias. If the measurement
             * never completed - a power-up that was followed straight by an
             * arm, an aircraft that was handled the whole time - the stored
             * bias stands and the console is told, once.
             *
             * It is one measurement per power-up, and a second one after the
             * flight was tried and taken out: the aircraft that has just
             * landed is still settling, and a half second of *steady* reading
             * from a rolling airframe is exactly the shape of a bias. Measured
             * that way, a session with five degrees a second of real bias came
             * back with a second measurement of 4575, 5770 and 5032 mdps on
             * the three axes and a heading that walked 15 degrees while
             * parked. The moment to measure is when somebody has just put the
             * aircraft down and switched it on.
             */
            {
                ak_flight_state_t state_now = ak_flight_state(&flight);

                if (state_now != AK_FLIGHT_DISARMED && gyro_cal.running) {
                    ak_console_printf(
                        "gyro: no bias measured before arming (%u samples, %u "
                        "rejected as moving); the stored bias stands\r\n",
                        gyro_cal.samples, gyro_cal.rejected);
                    gyro_cal.running = 0;
                }
                last_flight_state = state_now;
            }

            /*
             * The one heading measurement this aircraft has, and it only works
             * while it is moving: the track it is making *through the air*.
             *
             * The module reports motion over the ground, and the difference
             * between that and the way the aircraft is pointing is the crab
             * angle - which in wind is tens of degrees and, at a hover, is
             * *all* of it. So the wind is subtracted first, and the wind is
             * the navigator's own standing term: the velocity it has learned
             * to fly at to hold station, which is exactly the wind, in the
             * world frame. Until the navigator has learned one the correction
             * is zero and this is the ground track, which is what every flight
             * in this repository before it used.
             *
             * It is applied after the step, so the yaw it corrects is the one
             * this pass just integrated, and the step it is told about is the
             * loop's own period - the correction has a time constant of
             * seconds, so being a millisecond out about it is not a thing that
             * matters. A fix that has gone stale is not used: a course the
             * module is no longer measuring is a course somebody is guessing.
             */
            if (ak_gps_fix_valid(&gps, now, 2000u)) {
                float air_speed_m_s;
                float air_course_rad;

                ak_estimator_air_track(gps.fix.speed_mm_s, gps.fix.course_e5,
                                       nav.hold_n_m_s, nav.hold_e_m_s,
                                       &air_speed_m_s, &air_course_rad);
                ak_estimator_aid_heading(&flight.est, air_course_rad,
                                         air_speed_m_s,
                                         (float)AK_FLIGHT_LOOP_MS / 1000.0f);
            }


            /*
             * The turn the wing is already making, so its yaw loop damps
             * instead of fighting: g tan(bank) over the speed, which is what a
             * banked turning aircraft does. The speed is the GPS's - the
             * ground speed, which in wind is not the airspeed, and is the
             * measurement this aircraft has.
             *
             * A quadrotor gets zero: its yaw has nothing to do with its bank,
             * and telling it otherwise would have it yaw every time it leaned
             * to translate.
             *
             * **`g tan(bank)/V` is a rate of change of *heading*, and this is
             * fed to a loop whose feedback is the gyro's z axis - a rate about
             * the body's own vertical.** Those are the same number only while
             * the aircraft is level. In a level coordinated turn the two are
             * related by `heading' = r / (cos(pitch) cos(roll))`, so the body
             * rate the loop should be damping is `heading' * cos(pitch) *
             * cos(roll)`, and it is that factor which is applied here.
             *
             * This was invisible until the estimator and the plant were both
             * repaired, for the same reason the plant's own gyro was wrong in
             * the same direction as the old estimator: the simulator reported
             * a heading rate as if it were a body rate, and this feedforward
             * was a heading rate. The two errors cancelled exactly, at every
             * bank angle, in every session.
             *
             * Measured, on the fence session's `the motors barely had to fight
             * the turn` check, whose threshold is 10 deg/s of differential yaw:
             *
             *     old estimator, old plant,
             *       gyro in the same wrong frame    6.8 deg/s   passes, and
             *                                                   passes because
             *                                                   the two errors
             *                                                   cancel
             *     old estimator, honest gyro       10.7 deg/s   FAILS
             *     everything repaired, this
             *       factor taken back out          10.8 deg/s   FAILS
             *     everything repaired               6.2 deg/s   passes
             *
             * The third row is an ablation, not a recollection: the same tree
             * as the fourth with this factor alone reverted, 41 of 42 sessions
             * passing and this one check red. So the factor is not a refinement
             * of a session that was passing anyway - with the frames fixed
             * everywhere else it is the entire difference, and a fence session
             * that passed at 6.8 before passed for the reason this commit
             * exists to remove.
             */
            flight.yaw_rate_ff = 0.0f;
            if (flight.airframe == 1u &&
                ak_gps_fix_valid(&gps, now, 2000u)) {
                float speed_m_s = (float)gps.fix.speed_mm_s / 1000.0f;

                if (speed_m_s > 3.0f) {
                    float roll = flight.est.roll;
                    float tan_roll = ak_sinf(roll) / ak_cosf(roll);
                    float heading_rate =
                        ak_clampf(9.80665f * tan_roll / speed_m_s, -1.0f, 1.0f);

                    flight.yaw_rate_ff = heading_rate *
                                         ak_cosf(flight.est.pitch) *
                                         ak_cosf(roll);
                }
            }

            /* Every fourth iteration. The aircraft does not change
             * meaningfully in a millisecond, and the memory is better spent on
             * time than on resolution. */
            if ((flight.steps % AK_LOG_EVERY) == 0u) {
                log_iteration(now);
            }

            /*
             * The flash log's one decision: give it room, but only while the
             * aircraft is on the ground. Erasing a sector stops the CPU for
             * about a second, and a second of no control loop is a crash - so
             * the log stops instead, and this is where it is allowed to catch
             * up. Disarmed is the only state that means that: an aircraft in
             * failsafe may still be flying.
             */
            (void)ak_flashlog_service(&flashlog,
                                      ak_flight_state(&flight) ==
                                          AK_FLIGHT_DISARMED);

            /* And what the handset is told, at its own much slower rate: the
             * pilot reads a battery voltage, not a control loop. */
            if ((int32_t)(now - next_crsf_ms) >= 0) {
                next_crsf_ms = now + AK_CRSF_TLM_TICK_MS;
                telemetry_service();
            }

            /* The same outputs the flight core just decided on, in the form
             * the timers and the ESC's see. With no sensors the core is in
             * failsafe, so this is a stream of disarmed DShot frames and
             * centred servos - which is exactly what a scope should show on a
             * board that has not been armed.
             *
             * `output test` takes the place of the flight core here, and only
             * here: it is the one thing that writes outputs without arming,
             * and it stops the moment the aircraft is not disarmed. */
            static ak_output_frame_t frame;
            if (output_test_active) {
                if (ak_flight_state(&flight) != AK_FLIGHT_DISARMED) {
                    output_test_active = 0;
                    ak_console_write("output test: stopped - the aircraft is "
                                     "not disarmed\r\n");
                } else {
                    ak_outputs_t sweep;
                    output_test_outputs(now, &sweep);
                    ak_output_encode(&sweep, servo_trim, 0, &frame);
                }
            }
            if (!output_test_active) {
                ak_output_encode(ak_flight_outputs(&flight), servo_trim, 0,
                                 &frame);
            }
            ak_board_output_write(&frame);
        }

        /*
         * The two reads that block, and they are here - after the control step
         * and after the outputs have been written - rather than above it, which
         * is the "move slow I2C outside critical iteration" half of B4.
         *
         * Both are I2C transactions: the barometer's conversion takes
         * milliseconds to be ready and the rangefinder's read waits on a part
         * that may not answer at all. Neither is fast and neither can be made
         * fast from here, so the question is only what waits for them. Above
         * the gate, everything did: the attitude one pass old was computed,
         * then a millisecond or more was spent on a pressure, and only then was
         * the aircraft flown. Below it, the aircraft is flown first with the
         * newest attitude there is and the outputs go out, and the slow parts
         * are read on whatever time is left over. They are period-gated - the
         * barometer at 32 Hz, the rangefinder by its own service - so nothing
         * here runs on most passes.
         *
         * This does not make the loop faster. `max_loop_ms` in the core's
         * timing counters will still show the pass where a barometer took two
         * milliseconds, because a single-threaded loop cannot read a slow part
         * without waiting for it. What it changes is *which* work is late:
         * a height estimate a millisecond stale rather than a control output,
         * and an aircraft that has already been flown before anything slow is
         * asked for. Making the reads themselves non-blocking - the part
         * signalling a data-ready pin, or the conversion started and collected
         * a pass later - is the next step, and it is a change to the sensor
         * services rather than to this loop.
         */
        now = ak_time_ms();

        /*
         * The barometer, polled at the rate it measures at. Its reference is
         * captured while the aircraft is disarmed and has not moved - the
         * pressure on the ground - because everything a barometer is used for
         * here is a *change*: the weather moves the absolute number and the
         * height does not.
         */
        if (baro_ok && (int32_t)(now - baro_next_ms) >= 0) {
            ak_baro_sample_t fresh;
            int got;

            baro_next_ms = now + AK_BARO_PERIOD_MS;
            got = ak_baro_read(&baro, &fresh);
            if (got > 0) {
                baro_fails = 0;
                fresh.time_ms = now;
                baro_sample = fresh;
                baro_samples++;
                if (!baro_have_reference &&
                    ak_flight_state(&flight) == AK_FLIGHT_DISARMED) {
                    baro_reference_pa = fresh.pressure_pa;
                    baro_have_reference = 1;
                    ak_altitude_set_baro_reference(&altitude,
                                                   fresh.pressure_pa);
                }
                ak_altitude_baro(&altitude, fresh.pressure_pa);
            } else if (baro_fails < 255u) {
                /*
                 * A barometer that was answering and has stopped. Two samples
                 * in a row is 60 ms at 32 Hz - long enough that a single
                 * missed conversion is not a failure, short enough that the
                 * height does not freeze for long: from here the estimate
                 * comes from the GPS, which is what a board with no barometer
                 * at all does, and the next good sample takes it back.
                 */
                baro_fails++;
                if (baro_fails == 2u) {
                    ak_altitude_baro_lost(&altitude);
                    ak_console_printf("baro: %u reads failed in a row - the "
                                      "height is coming from the gps\r\n",
                                      baro_fails);
                }
            }
        }

        /*
         * The rangefinder, asked every pass and answered at its own rate: the
         * service owns the period, so this is a comparison rather than a
         * second timer, and the flight loop never has to know how fast the
         * part is. A part that stops answering is said once and then only
         * counted - the same rule the receiver's framing errors get, and for
         * the same reason: a line repeated a thousand times a second is a
         * console nobody can read in the situation where somebody is reading
         * it. The distance is *not* kept when the part goes quiet: a landing
         * may not be decided on a number nothing is measuring any more.
         */
        if (range_ok) {
            int32_t mm = 0;
            int got = ak_rangefinder_read(&range, now, &mm);

            if (got == AK_RANGE_FAULT) {
                if (range_fails == 0u) {
                    ak_console_printf("rangefinder: no answer - the ground is "
                                      "not being measured\r\n");
                }
                if (range_fails < 255u) {
                    range_fails++;
                }
            } else if (got != AK_RANGE_IDLE) {
                /* The part answered - with a distance or with "nothing in
                 * range" - so it is not quiet any more. A pass where it was
                 * not asked is *not* an answer, and treating it as one is what
                 * made this pair of lines flip once per poll while a part was
                 * dead, with the count of failed reads never rising above one.
                 * The simulation found it: see the `rangefindfail` session. */
                if (range_fails > 0u) {
                    ak_console_printf("rangefinder: answering again after %u "
                                      "failed %s\r\n", range_fails,
                                      range_fails == 1u ? "read" : "reads");
                }
                range_fails = 0u;
            }
        }

        if ((int32_t)(now - next_heartbeat) >= 0) {
            next_heartbeat += AK_HEARTBEAT_MS;
            ak_console_printf("alive: %u ms, %u loops\n", now, flight.steps);
        }
    }
}
