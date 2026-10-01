/*
 * fw_sim - the whole firmware, against a simulated aircraft.
 *
 *   build-host/aerialkit-fw-sim [seconds]
 *
 * Everything else in this repository tests a module. This runs `main()`: the
 * console, the CLI, the config protocol, the sensor drivers, the flight core
 * and the navigation code, wired to each other exactly as they are on a board,
 * with the board contract implemented in this file instead of in silicon.
 *
 * That contract is the same seam the ESP32 port uses. What the simulation
 * provides: virtual time advanced by the firmware's own delays, synthetic CRSF
 * frames packed the way a receiver packs them, synthetic NAV-PVT frames packed
 * the way a u-blox packs them, a register file that answers like an
 * ICM-42688-P, and an airframe model that turns the mixer's output back into
 * motion. Every parser, driver and control law in the path is the real one.
 *
 * The scenario flies one aircraft: arm, take off, roll, lose the radio link,
 * and let the navigator bring it home while somebody types at the console.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>

#include "ak_board.h"
#include "ak_fault.h"
#include "ak_accel_cal.h"
#include "ak_console.h"
#include "attitude_step.h"
#include "ak_crsf.h"
#include "ak_crsf_telemetry.h"
#include "ak_rc_receiver.h"
#include "ak_sbus.h"
#include "ak_fault.h"
#include "ak_gps.h"
#include "ak_log.h"
#include "ak_main.h"
#include "ak_math.h"
#include "ak_rc.h"
#include "ak_ring.h"
#include "ak_time.h"
#include "ak_text.h"
#include "ak_version.h"
#include "ubx_reference.h"

#define SIM_HZ 1000u

static uint32_t virtual_ms;
static uint32_t run_seconds = 20u;

void ak_time_init(void) {}

/*
 * The simulated clock.
 *
 * What costs time on a board is *waiting*, and in this firmware waiting is
 * reading the clock in a loop. So a clock read with no other board call since
 * the last one is a spin, and costs a microsecond - a clock read plus a pass
 * around a 160 MHz core. One pass of the main loop costs a whole millisecond,
 * which is what AK_FLIGHT_LOOP_MS says a pass is worth, and is charged once
 * per iteration by sim_step().
 *
 * The first version charged a millisecond for *every* spin read, and the loop
 * reads the clock four times a pass, so the firmware's own sense of time ran at
 * roughly twice real speed. That was not cosmetic: the receiver and the GPS
 * scheduled themselves with `virtual_ms % period`, and a clock that moves in
 * jumps of two lands on an exact multiple of twenty about one time in twenty.
 * The frames were never lost - two thirds of them were never sent.
 */
#define SIM_SPIN_US 1u

static uint32_t spin_us;      /* the sub-millisecond part of the clock */
static int clock_activity;    /* a board call happened since the last clock read */

/*
 * The board that lies, and what each lie is for.
 *
 * `preflight_run()` exists to answer "is this board what the code thinks it
 * is", and every one of its FAIL branches is a bench finding: a crystal that
 * never started, a saved configuration whose checksum is wrong, a flight
 * controller that crashed before this boot, a board with fewer outputs than
 * the mix needs. None of them had ever been *executed* on this machine - the
 * branches were compiled and never reached, which is the state that let an
 * unreachable Wi-Fi half sit in the ESP32's port for days. A check nobody can
 * trip is a check nobody can trust, so the simulator's board can be wrong on
 * purpose.
 *
 * The session arms one lie at a time and types `preflight` after each; the
 * checks look for that failure's own line, and for the clean line once every
 * lie is taken back - because a check that cannot pass is not a check either.
 * Two of the lies are facts about boot rather than switches a running command
 * can read (`sim_no_outputs`, `sim_two_motors`), so those two are session
 * words and they close half of the arming gate with them.
 */
static int sim_lie_console_port;
static int sim_lie_clock;
static int sim_lie_tick;
static uint32_t sim_lie_tick_ms;
static int sim_lie_dshot;
static int sim_lie_config;
static int sim_lie_config_write;
static int sim_no_outputs;
static int sim_two_motors;

/* The plant's heartbeat and the senders are described further down, but the
 * clock needs to know how long a pass of the firmware's loop is. */
static uint32_t n_sim_step;

uint32_t ak_time_ms(void)
{
    /* The millisecond tick, stopped. Nothing advances: the preflight's own
     * spin is bounded (five million reads and then it gives up), so this is a
     * board that cannot measure anything rather than a simulator that hangs,
     * and the session clears the lie as soon as the command has answered. */
    if (sim_lie_tick) {
        return sim_lie_tick_ms;
    }
    if (!clock_activity) {
        spin_us += SIM_SPIN_US;
        if (spin_us >= 1000u) {
            spin_us = 0;
            virtual_ms++;
        }
    }
    clock_activity = 0;
    return virtual_ms;
}

/* --- the airframe --------------------------------------------------------- */

enum {
    SIM_QUAD = 0,
    SIM_WING = 1,
    SIM_BENCH = 2,
    SIM_MISSION = 3,
    SIM_FENCE = 4,
    /* Nobody is flying this one: the console is the pilot, and the commands
     * come from stdin. It is how the bench runner (tools/bench_check.py) is
     * tested without a board, and how a person types at the firmware on a
     * machine that is not an aircraft. */
    SIM_CONSOLE = 5,
    /* The quadrotor's return, flown: out on a stick, the link goes, and the
     * navigator has to bring it home and put it in a hover - which needs the
     * plant to have motion at all, and needs the yaw to mean something once it
     * does. */
    SIM_QUAD_RTH = 6,
    /* The same return with the GPS module going quiet halfway home: the one
     * measurement a return cannot do without is the one that fails, with the
     * link already down. What the aircraft does next is the scenario. */
    SIM_GPS_LOST = 7,
    /* The pack goes flat while somebody is flying it, and - if it is asked to -
     * the aircraft takes itself home. See battery_step(). */
    SIM_BATTERY = 8,
    /* The sticks in *rate* mode, which is the other flight law the mode
     * channel selects and the one a pilot flies a quadrotor with by hand. */
    SIM_RATE = 9,
    /* The gyro stops answering in flight: the one sensor whose loss the flight
     * core treats as a reason to stop, and - until this - a path with no
     * evidence beyond a host test. See noimu_step(). */
    SIM_NO_IMU = 10,
    /* The same return, the same GPS loss, and a pilot who has said what to do
     * about it: with a rangefinder fitted and `quad_hold_land_s` set, the
     * aircraft stops holding and comes down where it is. See gpslost_step()
     * and the checks at the end of a run. */
    SIM_GPS_LOST_LAND = 11,
    /* An aircraft held at an angle with the arm switch thrown: the gate that
     * was missing. Nothing flies here - the aircraft is in somebody's hand on
     * a bench, which is where a tilted arm happens - and what the run is about
     * is what the console says and whether the motors stay off. See
     * tilt_step(). */
    SIM_TILT = 12,
    /* A wing thrown by hand: the switch, the spool-up, the climb attitude the
     * launch holds, and the stick that takes it back. The plant has no throw in
     * it - a wing's speed here follows its throttle - so what this flies is the
     * half of a launch the firmware does, which is the half a switch can start.
     * See launch_seq(). */
    SIM_LAUNCH = 13,
    /* Not a flight: a board that stopped. The record a fault leaves behind is
     * planted in the RAM startup does not clear *before* the firmware boots,
     * so what this session is about is the three lines the next boot prints -
     * the ones somebody at a bench reads and feeds to `addr2line`. The stage
     * is planted with values that are odd on purpose: the check is that the
     * record reaches the console at all, and with its own numbers. */
    SIM_FAULT = 14,
    /* The board that lies: nothing flies and nothing is crashed, and the
     * session is a conversation with `preflight` in which one board fact is
     * wrong at a time. Every FAIL line that command can print is a bench
     * finding, and this is the only place on this machine where they are
     * reached - see the lie flags above and badboard_step(). */
    SIM_BADBOARD = 15,
    /* A GPS module that is on the bus and not talking this parser's language:
     * a u-blox out of the box speaks NMEA at 9600, and the firmware's answer is
     * to send it a configuration frame, four times, and then give up and fly
     * without it. Nothing else in this file reaches `gps_configure()`: every
     * other session's module is talking from the first second, so the path
     * exists for a bench case no session had. */
    SIM_GPS_QUIET = 16,
    /* An aircraft armed the instant the firmware would let it: the arm switch
     * is already up when the battery goes in, so the bias the firmware
     * measures for itself has not been measured yet. That is a window of
     * nearly half a second that any person who plugs a battery in with the
     * switch left on will find, and what the firmware does about it is the
     * whole of this session - see armnow_step(). */
    SIM_ARM_NOW = 17,
    /* `output test` is the one command that writes the outputs without
     * arming, and it gives them back the moment somebody arms: the interlock
     * that keeps a bench sweep from fighting the pilot who has just taken
     * control. See testarm_step(). */
    SIM_TEST_ARM = 18,
    /* The bench's mistakes, one console command at a time: the calibrations
     * tried in the wrong state, and the sentences that come back. It is the
     * session that found `calibrate accel` had no "not disarmed" guard where
     * the other three have one. See refuse_step(). */
    SIM_REFUSE = 19,
};

/*
 * The mission scenario: a wing that is told where to go while somebody is
 * still holding the transmitter.
 *
 * What it is for is the thing that is new: a navigator flying an aircraft whose
 * pilot's link is up, because the pilot handed it over. That is a different
 * rule from the return-to-home, where the navigator only flies because nobody
 * else can, and the two are one `if` apart in the flight core.
 */
static const float mission_wp_m[2][2] = {
    { 220.0f,   0.0f }, /* 220 m north of home */
    { 220.0f, 275.0f }, /* then 275 m east of that */
};

static int   mission_stage;
/* The mission with a pack that goes flat while it is flying: takes precedence
 * over the waypoints, which is the whole question this run asks. */
static int   sim_mission_flatpack;
static int   sim_mission_sagged;
static float mission_start_alt_m;
static float mission_min_m[2];
static float mission_end_m[2];
static float mission_end_alt_m;
static int   mission_switch_on;
static int   mission_finished;
/* And, for the long runs, the switch is left on: the aircraft holds the
 * station (a quadrotor) or circles it (a wing) for the rest of the session,
 * which is the drift question no short session can ask. */
static int   sim_mission_hold;
/* And the run where the *pilot* takes the mission back with a stick while the
 * switch stays on - the one ending no session had run, and the state that has
 * its own branch in main.c: still asking, already cancelled. */
static int   sim_mission_takeback;
/* The session where the rangefinder stops answering on the way down and comes
 * back before the aircraft is on the ground: which step of that the scenario
 * is on. */
static int   quadrth_range_stage;
/* Which airframe flies the mission: the wing circles where it arrives and the
 * quadrotor hovers there, and both are flown by the same waypoint list and the
 * same switch. */
static int   mission_airframe = 1;

static uint32_t sim_gps_stop_ms;   /* zero: the module never goes quiet */
/* The line being typed lives here rather than on the stack: it is handed to the
 * console poll, which reads it long after mission_type_waypoint has returned. */
static char  mission_line[64];
/* The fence's own, for the one command whose argument is a number the scenario
 * has to work out: the ceiling, set above wherever the aircraft happens to be. */
static char  fence_line[48];

/*
 * The fence scenario: a pilot flies out with the link up, and the aircraft
 * comes back because it left somewhere it was told to stay.
 *
 * That is the one automatic behaviour here that takes an aircraft away from
 * somebody who is still holding the transmitter, so the check is not "did it
 * come back" alone - it is also that the floor it was given is the altitude it
 * climbed to on the way, and that the pilot got the aircraft back once it was
 * inside again.
 */
/*
 * The fence is 250 m, and that is not an arbitrary number: a wing holding a
 * circle at the pilot's stick has a turn radius, and a fence smaller than that
 * radius is a fence the aircraft leaves again on the far side of its own
 * turn - the navigator takes over, hands back, takes over. The first version
 * of this scenario used 150 m with a 12-degree turn and did exactly that, which
 * is worth knowing about a real aircraft as much as a simulated one.
 */
#define FENCE_RADIUS_M 250.0f
/* Sea level in the simulation, because the floor is an altitude and the GPS
 * reports one; this is 30 m above the take-off point. */
#define FENCE_FLOOR_MSL_M 150.0f
#define FENCE_FLOOR_ABOVE_HOME_M (FENCE_FLOOR_MSL_M - (SIM_HOME_ALT_MM / 1000.0f))
static int   fence_stage;
static float fence_min_after_m;
static float fence_end_m;
static uint32_t fence_next_status_ms;
/* The lid, set above the aircraft's own altitude once the pilot has it back:
 * what it was, when the breach fired, and the highest the aircraft got after
 * that - the number that says the climb stopped. */
static int      fence_ceiling_m;
static uint32_t fence_ceiling_fired_ms;
static float    fence_max_after_m;
static float    fence_ceiling_end_m;
/* Whether the "where does the fence think it is" report has been asked for
 * yet, in the phase above the lid. */
static int      fence_gps_over;
/* Where the aircraft was when the radius return handed it back: the floor test
 * is about the *return*, so it is measured there rather than at the end of a
 * run that now goes on to test the ceiling afterwards. */
static float    fence_alt_at_handover_m;

static int sim_airframe = SIM_QUAD;

/* Which plant - and therefore which take-off and which set of stick meanings -
 * a session flies. The two airframes are not the same aircraft, and a scenario
 * that got this wrong would be measuring its own mistake: the first version of
 * the quadrotor mission ran the wing's plant with the quadrotor's profile, and
 * the aircraft never left the ground. */
static int sim_flies_a_quad(void)
{
    if (sim_airframe == SIM_MISSION) {
        return mission_airframe == 0;
    }
    return sim_airframe == SIM_QUAD || sim_airframe == SIM_QUAD_RTH ||
           sim_airframe == SIM_GPS_LOST || sim_airframe == SIM_BATTERY ||
           sim_airframe == SIM_RATE || sim_airframe == SIM_NO_IMU ||
           sim_airframe == SIM_GPS_LOST_LAND ||
           sim_airframe == SIM_ARM_NOW || sim_airframe == SIM_TEST_ARM;
}

/* Set by --console: the console's bytes come from stdin rather than from a
 * scenario's script. */
static int console_stdin;

/*
 * The flight pack the simulator is holding, and the divider it is behind.
 *
 * The quadrotor's pack gives up on cue: a *step* to 9.75 volts at ten and a
 * half seconds, which is a 3S at 3.25 volts a cell. A step rather than a
 * discharge curve, on purpose - what this is for is the arithmetic that turns
 * a pin voltage into a cell count and a level, all the way through the loop
 * that flies the aircraft, and a battery model would be a second thing that
 * could be wrong. The other airframes keep a healthy 3S.
 */
#define SIM_PACK_DIVIDER 11.0f /* the board's 10k/1k pair */
#define SIM_PACK_SAG_MS  10500u
#define SIM_PACK_SAG_V   9.75f
static float sim_pack_v = 12.0f;
static int   sim_pack_sagged;
/* Whether a sag has come *back* again: the battery scenario sags one and
 * recovers it, because a pack that recovers must not hand a return back
 * mid-flight. */
static int      sim_pack_recovered;
/* The mode channel, which selects angle or rate mode. It is a stick position a
 * pilot can reach in the air rather than a build option, so a scenario that
 * means to fly rate mode has to throw the switch. */
static int      sim_rate_mode;
/* A barometer that stops answering: zero is never, otherwise the simulated
 * time it goes quiet. The return scenario sets it partway down, which is the
 * moment the aircraft is leaning on it hardest. */
static uint32_t sim_baro_stop_ms;
static int      sim_baro_fail_on_descent;
static int      sim_range_fail_on_descent;
/* The gyro stops answering: zero is never, otherwise the simulated time it
 * goes quiet, and `sim_imu_back_ms` is when it answers again. The scenario
 * uses both, because a failsafe that un-latches by itself is not a failsafe. */
static uint32_t sim_imu_stop_ms;
/* The offset a real part carries when it has warmed up since somebody
 * calibrated it: the session that asks about the arm-time calibration sets
 * this, and it is added to every axis. */
static float    sim_gyro_bias_dps;
static uint32_t sim_imu_back_ms;
/* The satellite count the module reports, and whether a scenario degrades it
 * partway through a return: four satellites and a 3D fix is the case INAV's
 * floor exists for, and the frames keep arriving throughout. */
static unsigned sim_gps_sats = 11u;

/* What the firmware asked the module to become. The configuration frames are
 * the one thing this bus carries *out*, and a session whose module is not
 * talking is the only one that sends any. */
static uint8_t  sim_gps_cfg[192];
static unsigned sim_gps_cfg_len;
static unsigned sim_gps_cfg_sends;
static int      sim_badfix_on_return;
static uint32_t sim_gps_bad_ms;
static uint32_t sim_gps_good_ms;
/* A fix that is *wrong* rather than degraded: the position jumps, the fix type
 * and the satellite count stay perfect, and the frames keep coming. Nothing in
 * the firmware's fix-quality gate can see this one, which is the point of
 * measuring it. Zero is never. */
static uint32_t sim_gps_jump_at_ms;
static uint32_t sim_gps_jump_until_ms;
static float    sim_gps_jump_north_m;
static int      sim_gps_jump_on_return;
/*
 * Sensor noise, off unless a session asks for it with `noisy`.
 *
 * Everything in this simulator has been ideal: the gyro reported the plant's
 * exact rate, the accelerometer the exact gravity vector, the barometer the
 * exact pressure (quantised, which is the one real error the tape ever had) and
 * the GPS the exact position. A real part is not that good, and the loops that
 * fly on these numbers have never been asked whether they tolerate one.
 *
 * The numbers are the datasheet's, converted to one-sigma at the rate this
 * firmware samples at:
 *
 *   ICM-42688P gyro    2.8 mdps/sqrt(Hz)  ->  0.06 deg/s at 1 kHz
 *   ICM-42688P accel   70 ug/sqrt(Hz)     ->  0.0016 g   at 1 kHz
 *   DPS310 barometer   ~0.5 Pa            ->  4 cm of height
 *   GPS horizontal     HACC 1.5 m         ->  1.5 m one-sigma
 *
 * The generator is a seeded LCG with an Irwin-Hall sum on top, so the same
 * session produces the same tape on every host: a noise run that cannot be
 * re-run is a noise run nobody can compare against.
 */
static int      sim_noisy;
static uint32_t sim_rng = 0x13579BDFu;
static int      sim_rng_used;

static float sim_rand(void)
{
    /* Three uniforms, sum minus one and a half: mean zero, sigma 0.5, bounded
     * at +-1.5, which is close enough to Gaussian and has no libm in it. */
    float sum = 0.0f;

    for (int i = 0; i < 3; i++) {
        sim_rng = sim_rng * 1664525u + 1013904223u;
        sum += (float)(sim_rng >> 8) / 16777216.0f;
    }
    return sum - 1.5f;
}

static float sim_noise(float one_sigma)
{
    if (!sim_noisy) {
        return 0.0f;
    }
    sim_rng_used = 1;
    return sim_rand() * 2.0f * one_sigma; /* sigma 0.5 each way */
}
/* The link loss that is a receiver in its own failsafe rather than a silent
 * one: see the STEP_YAW transition in the scenario. */
static int      sim_rx_failsafe_on_link_loss;

/*
 * The wing with nothing to bring it home: RTH is off, which is what a board
 * ships with, so when the receiver comes out there is no navigator and no
 * pilot - and a fixed wing that stops flying is a brick. The owner's rule for
 * that case is "land as it circles down", so this session watches exactly
 * that: see the winglost block in the report.
 */
static int      sim_wing_lost;

/* Ends the run and prints the checks: called by a scenario that has seen what
 * it was waiting for - the lost wing reaching the ground - as well as by the
 * clock running out. Declared here because the scenario runs before the
 * reporting half of this file is defined. */
static void sim_finish(void);

/*
 * The bench mode puts the aircraft in whatever attitude the scenario says,
 * which is how a person calibrates an accelerometer: hold it on a face, say
 * which face, wait. The gyro reads zero while it is held, because it is being
 * held still, and the accelerometer can be given a known error so that the
 * calibration has something to find and the check has something to compare
 * with.
 */
static int   sim_held;
static float sim_hold_roll_deg;
static float sim_hold_pitch_deg;
/* And whether the hand holding it is a hand: see plant_step(). */
static int   sim_handled;

/* The part the bench mode is holding: what it reads is
 * bias + true / scale, so a correct calibration recovers both numbers. */
static const float sim_part_bias[3] = { 0.030f, -0.020f, 0.040f };
static const float sim_part_scale[3] = { 0.980f, 1.020f, 1.005f };

static float plant_roll_rate;
static float plant_pitch_rate;
static float plant_roll;
static float plant_pitch;
static float plant_yaw_rate_dps;
static ak_outputs_t plant_outputs;

/*
 * The wind.
 *
 * Every session in this file flew in still air, which is the one thing a real
 * flight almost never has - and it is not a detail, because it is the
 * difference between where an aircraft is *pointing* and where it is *going*.
 * A wing pointed north in a crosswind travels north-east: its heading and its
 * ground track differ by the crab angle, and a GPS reports the track. That is
 * exactly the assumption the estimator's ground-track yaw alignment rests on
 * (ak_estimator_aid_heading), so a plant with no wind could not test it.
 *
 * Air out of the west at five metres a second: a breezy afternoon, and well
 * inside what either of these aircraft is meant to fly in. Which sessions get
 * it is not arbitrary, and the ones that do not say why - see where it is set
 * in main().
 */
static float sim_wind_n_m_s;
static float sim_wind_e_m_s;
#define SIM_WIND_BREEZE_E 5.0f

/* Where the aircraft is, for the airframe that can be somewhere. A quad in
 * this simulation flies a point mass too now: a tilt is an acceleration, the
 * collective is height, and the two prop pairs fight over the yaw. It used to
 * hold its position and its heading, which meant the quadrotor's own return
 * could not be flown here at all - a profile whose whole job is to *go*
 * somewhere is untestable on an aircraft that cannot. */
static float plant_north_m;
static float plant_east_m;
static float plant_alt_m;
static float plant_heading_deg;
static float plant_airspeed_m_s;
/* The airframe's velocity over the ground, which is what a GPS reports the
 * course of. For the wing in still air that is the way it is pointing; for a
 * quad flying sideways it is not, and that difference is the whole reason the
 * estimator aligns its yaw to the ground track. */
static float plant_vn;
static float plant_ve;
static float plant_vz;
/* The air-relative half of the quadrotor's velocity: a tilt changes how the
 * aircraft moves through the *air*, and the ground sees that plus the wind. */
static float plant_air_vn;
static float plant_air_ve;

/* The collective that holds a quad up, and the two props that spin
 * counter-clockwise (seen from above): the front-right and the rear-left,
 * which is the pair every flight controller speeds up to yaw right. Fitting
 * the props the other way round is a real thing to do and makes the yaw axis
 * run backwards - which is why the bench checklist has a line for it. */
#define SIM_QUAD_HOVER 0.55f
/*
 * Drag, which this plant did not have - and that is what made the wind case
 * look like a control-tuning problem when it was a model problem.
 *
 * With no drag a tilt sets an *acceleration* and a throttle sets a *climb
 * acceleration*, so both loops are double integrators: a hover throttle holds
 * whatever climb rate the aircraft happens to have, and a lean holds whatever
 * speed. Real aircraft do not behave that way - a tilt sets a *speed* that the
 * drag balances, and a throttle above hover sets a *climb rate* that it
 * balances - and both the firmware's loops and any pilot's expectations are
 * built on that. Airframes settle in about two seconds here, which is what the
 * numbers in the airframe's own notes look like.
 */
#define SIM_QUAD_DRAG_PER_S 0.5f
/* Degrees a second of yaw per unit of differential thrust between the two
 * motors: a full differential is a firm rudder input, not a pirouette. */
#define SIM_WING_YAW_PER_DIFFERENTIAL 40.0f

/* The largest differential the motors have been asked for, and when: the
 * number that says whether the yaw loop is flying the aircraft or fighting it. */
static float differential_peak_dps;
static uint32_t differential_peak_ms;

/* The largest angle between where the wing was pointing and where it was
 * going: the wind's signature, and zero in still air. */
static float crab_peak_deg;

static float sim_quad_yaw_torque(void)
{
    return (plant_outputs.motor[1] + plant_outputs.motor[2] -
            plant_outputs.motor[0] - plant_outputs.motor[3]) * 0.5f;
}

/*
 * Where the aircraft ends up pointing, from how fast it is turning.
 *
 * plant_roll_rate, plant_pitch_rate and plant_yaw_rate_dps are *body* rates:
 * rotation about the aircraft's own three axes. A moment produces them, a gyro
 * reports them, and they are not the rates at which the roll, pitch and yaw
 * angles change. Level, the two triples are the same three numbers; banked or
 * pitched they are not, and this file treated them as the same for as long as
 * it has existed - as did ak_estimator.c, which added gyro[0]*dt to roll. Both
 * being wrong the same way is exactly why no session ever showed it.
 *
 * Attitude is Euler angles in the ZYX order the rest of the repository uses -
 * roll about the body x axis, then pitch about the new y, then yaw about the
 * new z - so the angles move as
 *
 *     roll_dot  = p + q sin(roll) tan(pitch) + r cos(roll) tan(pitch)
 *     pitch_dot = q cos(roll) - r sin(roll)
 *     yaw_dot   = (q sin(roll) + r cos(roll)) / cos(pitch)
 *
 * and this is that relation, integrated one step from the rates the two
 * airframes produce.
 *
 * sin(roll) and cos(roll) are read here rather than passed in because the
 * caller has already changed nothing else; cos(pitch) is floored away from
 * zero. Neither airframe flies with its nose vertical, and a scenario that has
 * gone wrong should produce a wrong number rather than a division by zero that
 * spreads a NaN through every session after it.
 *
 * The arithmetic is in tools/attitude_step.c now, with the reasoning above it.
 * It moved for one reason: while it was here, the only check that it *was* the
 * kinematics was a scratch harness in a job's temporary directory, because
 * `static` plus six file-static globals is not reachable from a second
 * translation unit. The plant's attitude step is what every other measurement
 * in the suite is taken against, so it can be the one part nobody re-runs.
 */
static void sim_attitude_step(float dt)
{
    ak_euler_t att;

    att.roll = plant_roll;
    att.pitch = plant_pitch;
    att.heading_deg = plant_heading_deg;
    ak_zyx_step(&att, plant_roll_rate, plant_pitch_rate, plant_yaw_rate_dps, dt);
    plant_roll = att.roll;
    plant_pitch = att.pitch;
    plant_heading_deg = att.heading_deg;
}

/*
 * The other direction, for the one case where the attitude is the input: an
 * aircraft somebody is holding. There the angles and their rates are given and
 * the gyro still reports body rates, so the three rates the caller has are
 * Euler rates and these are their ZYX inverse.
 */
static void sim_euler_rates_to_body(float roll_dot, float pitch_dot,
                                    float yaw_dot)
{
    float sr = ak_sinf(plant_roll), cr = ak_cosf(plant_roll);
    float sp = ak_sinf(plant_pitch), cp = ak_cosf(plant_pitch);

    plant_roll_rate = roll_dot - yaw_dot * sp;
    plant_pitch_rate = pitch_dot * cr + yaw_dot * cp * sr;
    plant_yaw_rate_dps = ak_rad2deg(-pitch_dot * sr + yaw_dot * cp * cr);
}

static void plant_step_quad(float dt)
{
    /* Torque becomes rate through a first-order lag: an airframe does not
     * change its rotation instantly. Crude, and enough to tell a stable loop
     * from an unstable one. */
    const float tau = 0.05f;
    float change = dt / (tau + dt);
    float torque_roll = (plant_outputs.motor[2] + plant_outputs.motor[3] -
                         plant_outputs.motor[0] - plant_outputs.motor[1]) * 0.5f;
    /* The pitch torque is the *front* pair minus the *rear* pair, and that is
     * physics rather than a convention: more thrust at the back of the
     * aircraft puts the nose down. The first version of this model had it the
     * other way round - rear thrust pitched the nose *up* - which made a plant
     * that agreed with an inverted mixer instead of disagreeing with it, and
     * an inverted pitch axis is what the firmware had (see ak_mixer.c). The
     * model is only worth having if it models the aircraft. */
    float torque_pitch = (plant_outputs.motor[1] + plant_outputs.motor[3] -
                          plant_outputs.motor[0] - plant_outputs.motor[2]) * 0.5f;

    plant_roll_rate += change * (torque_roll * 20.0f - plant_roll_rate);
    plant_pitch_rate += change * (torque_pitch * 20.0f - plant_pitch_rate);

    /* Yaw: the two diagonal pairs fight, and the pair that is spinning
     * counter-clockwise wins the nose to the right. */
    plant_yaw_rate_dps += change * (ak_rad2deg(sim_quad_yaw_torque() * 8.0f) -
                                    plant_yaw_rate_dps);

    /* Those three are rates about the aircraft's own axes, so the attitude
     * follows from all three at once rather than each angle from its
     * like-named rate. */
    sim_attitude_step(dt);

    /* Height: the collective against gravity, with the motors lagging the
     * command the way they lag the others. The ground is a floor - it does not
     * let the aircraft through - and the aircraft sits on it until the
     * collective is above hover, which is what makes a take-off a take-off. */
    float collective = (plant_outputs.motor[0] + plant_outputs.motor[1] +
                        plant_outputs.motor[2] + plant_outputs.motor[3]) * 0.25f;
    float climb = 9.81f * (collective / SIM_QUAD_HOVER - 1.0f);

    if (sim_held) {
        climb = 0.0f; /* somebody is holding it: the bench scenario */
    }
    plant_vz += (climb - SIM_QUAD_DRAG_PER_S * plant_vz) * dt;
    plant_alt_m += plant_vz * dt;
    if (plant_alt_m <= 0.0f) {
        plant_alt_m = 0.0f;
        if (plant_vz < 0.0f) {
            plant_vz = 0.0f;
        }
        /* On the ground, sliding stops: without this the velocity a landing
         * touched down with is still there in the model at the end of the run,
         * which reads as an aircraft doing thirty metres a second while parked
         * with its motors off. */
        plant_vn *= 0.9f;
        plant_ve *= 0.9f;
    }

    /* And where the tilt takes it. The horizontal part of the thrust is g times
     * the tilt angle, near enough for the angles this flies at; the tilt is
     * expressed in the body's own frame, so the heading is what turns it into
     * north and east. */
    if (!sim_held) {
        float heading = ak_deg2rad(plant_heading_deg);
        float a_forward = -9.81f * ak_sinf(plant_pitch);
        float a_right = 9.81f * ak_sinf(plant_roll);

        plant_air_vn += (a_forward * ak_cosf(heading) -
                         a_right * ak_sinf(heading) -
                         SIM_QUAD_DRAG_PER_S * plant_air_vn) * dt;
        plant_air_ve += (a_forward * ak_sinf(heading) +
                         a_right * ak_cosf(heading) -
                         SIM_QUAD_DRAG_PER_S * plant_air_ve) * dt;
        /* The ground sees the air-relative motion plus the wind, and the GPS
         * reports the ground - which is the whole point of modelling it. */
        plant_vn = plant_air_vn + sim_wind_n_m_s;
        plant_ve = plant_air_ve + sim_wind_e_m_s;
        /* Parked, friction beats the wind: an aircraft standing on its own
         * skids does not drift downwind, and a plant that lets it take off from
         * somewhere other than where it was standing is a plant that moves the
         * home the firmware captured. */
        if (plant_alt_m <= 0.0f) {
            plant_air_vn *= 0.9f;
            plant_air_ve *= 0.9f;
            plant_vn = 0.0f;
            plant_ve = 0.0f;
        }
        plant_north_m += plant_vn * dt;
        plant_east_m += plant_ve * dt;
    }
}

static void plant_step_wing(float dt)
{
    /* An elevon is a control surface, not a motor: it deflects, the aircraft
     * rotates, and the rotation lags the deflection the same way a quad's
     * rotation lags its thrust. The rate gains are picked so the loops this
     * repository has (an angle loop at 6 rad/s per radian into a rate loop at
     * 0.25 of full authority) are stable against it - a plant that is easy to
     * fly proves nothing, and one that cannot be flown proves less. */
    const float tau = 0.09f;
    float change = dt / (tau + dt);

    /* Left elevon gets +roll and +pitch, right gets -roll and +pitch: that is
     * the wing's mixer table, and reading it here is deliberate - a wrong sign
     * in the table has to fly the wrong way in the simulation rather than be
     * modelled away. */
    float roll_command = (plant_outputs.servo[0] - plant_outputs.servo[1]) * 0.5f;
    float pitch_command = (plant_outputs.servo[0] + plant_outputs.servo[1]) * 0.5f;

    /* Both are body rates - the elevon deflects, the aircraft rotates about
     * its own roll and pitch axes - and the attitude follows from them and the
     * yaw rate together, at the foot of this function. */
    plant_roll_rate += change * (roll_command * 8.0f - plant_roll_rate);
    plant_pitch_rate += change * (pitch_command * 5.0f - plant_pitch_rate);

    /* Point mass. The throttle sets the speed, the bank turns the velocity
     * vector - the coordinated turn a wing flies when it is not slipping - the
     * pitch climbs, and the speed carries it. No stall, no sideslip, no wind,
     * no drag curve, and the yaw axis is ignored because a wing with no rudder
     * has almost none. */
    float throttle = (plant_outputs.motor[0] + plant_outputs.motor[1]) * 0.5f;
    /* No throttle, no airspeed, and no airspeed means no flying: the aircraft
     * sits on the ground until somebody opens the throttle, which is also what
     * makes the position the firmware captures as home the place it is actually
     * sitting. A wing that taxied off the spot while disarmed would put home
     * somewhere else and call it a bug in the navigation. */
    float want_speed = throttle < 0.05f ? 0.0f : 6.0f + 16.0f * throttle;
    plant_airspeed_m_s += (want_speed - plant_airspeed_m_s) * change;

    float airspeed = ak_clampf(plant_airspeed_m_s, 0.0f, 40.0f);
    float bank = ak_clampf(plant_roll, -1.4f, 1.4f);
    float tan_bank = ak_sinf(bank) / ak_cosf(bank);
    float turn_rate_dps =
        airspeed > 1.0f ? ak_rad2deg(9.81f / airspeed * tan_bank) : 0.0f;

    /*
     * And the differential thrust, which this model did not have. A twin-motor
     * wing with no rudder yaws with its motors, and that is the *only* yaw
     * control it has - a model that leaves it out (this one did, until the
     * return's motors were noticed sitting pegged at opposite ends of their
     * range all the way round every turn) makes a yaw axis that cannot be
     * wrong. The sign is the mixer's: yaw right pushes the left motor harder,
     * and yaw right is the nose going right.
     */
    {
        float differential = (plant_outputs.motor[0] - plant_outputs.motor[1]) *
                             SIM_WING_YAW_PER_DIFFERENTIAL;
        float magnitude = differential < 0.0f ? -differential : differential;

        /*
         * turn_rate_dps is how fast the *nose comes round* and the
         * differential is a moment about the aircraft's own yaw axis, and the
         * plant's third state is the second of those. So the first has to be
         * turned into it rather than added to it: inverting the yaw row of the
         * ZYX relation, the nose comes round at turn_rate_dps only if the body
         * yaw rate is
         *
         *     r = (yaw_dot cos(pitch) - q sin(roll)) / cos(roll)
         *
         * which is the heading rate less whatever the pitch rate contributes
         * to it. Level and unpitched the two are the same number, which is why
         * this read as a heading rate for so long. cos(roll) is floored for
         * the same reason cos(pitch) is in sim_attitude_step(): a wing on its
         * side is not a case either of these aircraft is meant to fly.
         */
        float yaw_dot = ak_deg2rad(turn_rate_dps + differential);
        float sr = ak_sinf(plant_roll), cr = ak_cosf(plant_roll);
        float cp = ak_cosf(plant_pitch);

        if (cr > -0.05f && cr < 0.05f) {
            cr = cr < 0.0f ? -0.05f : 0.05f;
        }
        plant_yaw_rate_dps =
            ak_rad2deg((yaw_dot * cp - plant_pitch_rate * sr) / cr);

        if (magnitude > differential_peak_dps) {
            differential_peak_dps = magnitude;
            differential_peak_ms = virtual_ms;
        }
    }
    sim_attitude_step(dt);

    float heading = ak_deg2rad(plant_heading_deg);
    /* The airspeed vector is along the heading; the ground sees that plus the
     * wind. A wing pointed north in a crosswind therefore *travels* north-east,
     * and the course the GPS reports is the track it is making, not the way it
     * is pointing. */
    plant_vn = airspeed * ak_cosf(heading) + sim_wind_n_m_s;
    plant_ve = airspeed * ak_sinf(heading) + sim_wind_e_m_s;
    /* Standing on the ground with the motor off, the wind does not taxi it:
     * friction wins, which is what keeps the home the firmware captures on the
     * spot the aircraft was actually standing on. */
    if (plant_alt_m <= 0.0f && airspeed < 0.5f) {
        plant_vn = 0.0f;
        plant_ve = 0.0f;
    }
    plant_north_m += plant_vn * dt;
    plant_east_m += plant_ve * dt;
    plant_alt_m += airspeed * ak_sinf(plant_pitch) * dt;

    /* How far the nose is from the track it is making: the crab angle, which
     * is a measurement of the wind rather than of this model. */
    if (plant_ve != 0.0f || plant_vn != 0.0f) {
        float track_deg = atan2f(plant_ve, plant_vn) * 57.29578f;
        float crab = track_deg - plant_heading_deg;

        while (crab > 180.0f) {
            crab -= 360.0f;
        }
        while (crab < -180.0f) {
            crab += 360.0f;
        }
        if (crab < 0.0f) {
            crab = -crab;
        }
        if (crab > crab_peak_deg) {
            crab_peak_deg = crab;
        }
    }
}

static void plant_step(float dt)
{
    if (!sim_flies_a_quad()) {
        /* The navigating scenarios fly the same wing: it is the airframe with
         * somewhere to go. */
        plant_step_wing(dt);
    } else {
        plant_step_quad(dt);
    }
}

/* --- the IMU, as a register file the real driver reads --------------------- */

static uint8_t imu_regs[256];

static void imu_refresh(void)
{
    if (sim_held) {
        float t = (float)virtual_ms * 0.001f;

        if (sim_handled) {
            /*
             * A hand, not a bench vice.
             *
             * The gyro bias the firmware measures for itself needs five
             * hundred samples that agree with each other, and an aircraft
             * somebody is holding never gives it five hundred in a row - which
             * is why it measures at power-up, on the ground, before anybody
             * picks the aircraft up. This models the other case: a person
             * holding it, with the small rotation a hand cannot help, while
             * the calibration runs.
             *
             * The angles stay inside the arming gate's tilt limit on purpose.
             * A hand-held aircraft that is level enough to arm is exactly the
             * case that reaches the firmware's "no bias measured before
             * arming" line, and a scenario that rolled it past the limit would
             * be testing the tilt gate instead.
             */
            float w = 6.2831853f * 0.9f;

            plant_roll = ak_deg2rad(sim_hold_roll_deg +
                                    12.0f * ak_sinf(w * t));
            plant_pitch = ak_deg2rad(sim_hold_pitch_deg +
                                     8.0f * ak_sinf(w * 1.7f * t + 0.7f));
            /* Those two angles are *given* here, so what a hand imparts is the
             * rate they move at - an Euler rate - and the body rates the gyro
             * reports are its ZYX inverse, below. */
            sim_euler_rates_to_body(
                ak_deg2rad(12.0f * w) * ak_cosf(w * t),
                ak_deg2rad(8.0f * 1.7f * w) * ak_cosf(w * 1.7f * t + 0.7f),
                ak_deg2rad(25.0f * ak_sinf(w * 0.5f * t + 1.1f)));
        } else {
            /* Held still on a face: the attitude is whatever the scenario
             * says, and nothing is rotating. */
            plant_roll = ak_deg2rad(sim_hold_roll_deg);
            plant_pitch = ak_deg2rad(sim_hold_pitch_deg);
            sim_euler_rates_to_body(0.0f, 0.0f, 0.0f);
        }
    }

    float ax = -ak_sinf(plant_pitch);
    float ay = ak_sinf(plant_roll);
    float az = ak_cosf(plant_roll) * ak_cosf(plant_pitch);

    /* A real part measures this through its own noise. */
    ax += sim_noise(0.0016f);
    ay += sim_noise(0.0016f);
    az += sim_noise(0.0016f);

    if (sim_airframe == SIM_BENCH) {
        /* A real part is not this good, and a calibration that has nothing to
         * find proves nothing. */
        ax = sim_part_bias[0] + ax / sim_part_scale[0];
        ay = sim_part_bias[1] + ay / sim_part_scale[1];
        az = sim_part_bias[2] + az / sim_part_scale[2];
    }

    int16_t accel[3] = {
        (int16_t)(ax * 2048.0f),
        (int16_t)(ay * 2048.0f),
        (int16_t)(az * 2048.0f),
    };
    /*
     * The gyro reports body rates, and the plant's three rate states are body
     * rates, so this is a read and not a conversion. It was a conversion for
     * as long as the plant integrated Euler angles - the honest direction of
     * the ZYX relation, applied to a plant in the wrong frame - and this is
     * where that stops: there is one frame now and both sides use it.
     *
     * What it cost while there were two: the firmware and the plant were wrong
     * in the same direction, so the loop agreed with itself and no session
     * could see it. Repairing the estimator removed the agreement and the
     * simulator could not accept the repair in any configuration - the
     * measurement is in AERIAL-KIT-GOAL-PROGRESS.md under B2, and the four
     * rows of that table are the reason this file's plant changed before the
     * estimator repair landed.
     */
    int16_t gyro[3] = {
        (int16_t)((ak_rad2deg(plant_roll_rate) + sim_noise(0.06f) +
                   sim_gyro_bias_dps) * 16.4f),
        (int16_t)((ak_rad2deg(plant_pitch_rate) + sim_noise(0.06f) +
                   sim_gyro_bias_dps) * 16.4f),
        (int16_t)((plant_yaw_rate_dps + sim_noise(0.06f) +
                   sim_gyro_bias_dps) * 16.4f),
    };
    for (int i = 0; i < 3; i++) {
        imu_regs[0x1F + i * 2] = (uint8_t)((uint16_t)accel[i] >> 8);
        imu_regs[0x20 + i * 2] = (uint8_t)((uint16_t)accel[i] & 0xFFu);
        imu_regs[0x25 + i * 2] = (uint8_t)((uint16_t)gyro[i] >> 8);
        imu_regs[0x26 + i * 2] = (uint8_t)((uint16_t)gyro[i] & 0xFFu);
    }
}

static int sim_bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    /* A gyro that has stopped answering: the bus read fails, the driver counts
     * an error, and the sample the flight loop gets is not valid. */
    if (sim_imu_stop_ms != 0u && virtual_ms >= sim_imu_stop_ms &&
        (sim_imu_back_ms == 0u || virtual_ms < sim_imu_back_ms)) {
        return -1;
    }
    if (reg == 0x1F) {
        imu_refresh();
    }
    for (unsigned i = 0; i < len; i++) {
        buf[i] = imu_regs[(reg + i) & 0xFFu];
    }
    return 0;
}

static int sim_bus_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    imu_regs[reg] = value;
    return 0;
}

static void sim_bus_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    ak_delay_ms(ms);
}

static const ak_bus_t sim_bus = {
    .read = sim_bus_read,
    .write = sim_bus_write,
    .delay_ms = sim_bus_delay,
    .ctx = 0,
};

/* --- the barometer, as a second register file ------------------------------ */

/*
 * A DPS310 that answers with the pressure the standard atmosphere gives at
 * whatever height the plant is flying at.
 *
 * Its coefficients are deliberately simple - the pressure is linear in the
 * reading - because a real part's coefficients are a datasheet footnote and
 * the two things this simulator is here to exercise are the driver's path
 * (registers, two's-complement decode, scaling, compensation) and the
 * arithmetic that turns pressure into height. The pressure is the exact
 * standard atmosphere computed with libm, which the firmware cannot have, and
 * the height it reads back is the firmware's own series: feeding one into the
 * other is the check.
 */
static uint8_t baro_regs[256];

#define SIM_BARO_C10   100000.0f  /* pressure per unit of the scaled reading */
#define SIM_BARO_C0    2000.0f    /* temperature offset and slope, in the part's
                                   * own arbitrary units */
#define SIM_BARO_C1    1000.0f
#define SIM_BARO_K     253952.0f  /* the scale for sixteen-times oversampling */
#define SIM_SEA_LEVEL_PA 101325.0f

static void baro_refresh(void)
{
    /* The plant's height above its own take-off point, then the pressure the
     * standard atmosphere has at that height. */
    double ratio = 1.0 - (double)plant_alt_m / 44330.0;
    if (ratio < 0.5) {
        ratio = 0.5;
    }
    float pressure = (float)((double)SIM_SEA_LEVEL_PA * pow(ratio, 5.25500));
    float temperature_c = 20.0f;

    /* Four centimetres of height, which at sea level is about half a pascal -
     * and the part's own quantum is 0.39 Pa, so this is the noise of a real
     * part *underneath* the staircase the firmware already has to live with. */
    if (sim_noisy) {
        pressure += sim_noise(0.5f);
    }

    /* Invert the part's own model to get the readings an honest sensor would
     * report: P = (Praw / K) * c10 and T = c0*0.5 + c1*(Traw / K). */
    int32_t p_raw = (int32_t)(pressure / SIM_BARO_C10 * SIM_BARO_K);
    int32_t t_raw = (int32_t)((temperature_c - SIM_BARO_C0 * 0.5f) /
                              SIM_BARO_C1 * SIM_BARO_K);

    baro_regs[0x00] = (uint8_t)((uint32_t)p_raw >> 16);
    baro_regs[0x01] = (uint8_t)(((uint32_t)p_raw >> 8) & 0xFFu);
    baro_regs[0x02] = (uint8_t)((uint32_t)p_raw & 0xFFu);
    baro_regs[0x03] = (uint8_t)((uint32_t)t_raw >> 16);
    baro_regs[0x04] = (uint8_t)(((uint32_t)t_raw >> 8) & 0xFFu);
    baro_regs[0x05] = (uint8_t)((uint32_t)t_raw & 0xFFu);
    /* Coefficient and sensor ready, and a pressure result waiting, every time
     * it is asked: the part measures at 32 Hz and the firmware polls at 33. */
    baro_regs[0x08] |= 0xF0u;
}

static int baro_bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    /* A part that has stopped answering: the bus read fails, which is what a
     * dead barometer or a broken joint looks like from the driver's side. */
    if (sim_baro_stop_ms != 0u && virtual_ms >= sim_baro_stop_ms) {
        return -1;
    }
    if (reg == 0x00) {
        baro_refresh();
    }
    for (unsigned i = 0; i < len; i++) {
        buf[i] = baro_regs[(reg + i) & 0xFFu];
    }
    return 0;
}

static int baro_bus_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    baro_regs[reg] = value;
    return 0;
}

static const ak_bus_t baro_bus = {
    .read = baro_bus_read,
    .write = baro_bus_write,
    .delay_ms = sim_bus_delay,
    .ctx = 0,
};

/* --- the rangefinder, as a third register file -----------------------------
 *
 * A TOF10120 that answers with the plant's own height above the ground below
 * it, which is the one measurement the aircraft cannot get from anything else
 * it carries: the barometer measures a change since take-off and the GPS an
 * absolute height that wanders, and both of them are the same instrument at
 * the last metre as they were at a hundred.
 *
 * It is deliberately an *honest* part: it reports the truth, it reports
 * nothing when there is nothing within two metres of it, and it is as noisy as
 * a real one when the session asks for noise. The filter, the period and the
 * decision are all in the firmware, so what this can be wrong about is the
 * part, not the arithmetic.
 *
 * Whether it is fitted at all is a session word: the aircraft this firmware
 * has to keep working on is also the one without one.
 */
static uint8_t  range_regs[256];
static int      sim_range_fitted = 1;
static uint32_t sim_range_stop_ms;
/* A *bare* board: what a devkit out of the bag is, and what the ESP32
 * devkits here actually are - an inertial sensor, a receiver and a GPS, and
 * none of the parts a flight controller's board carries soldered on. The
 * firmware's claim is that such a board still flies, on the gps's altitude
 * where the barometer would have been and on the barometer where the
 * rangefinder would have been; `quadrth bare` is where that is flown. */
static int      sim_baro_fitted = 1;
static int      sim_vbat_fitted = 1;
/* Whether this run is the *bare board* one, so the scenario knows to ask the
 * console for the preflight report before anything flies. */
static int      sim_bare;
static int      sim_bare_report_done;
/* The launch session's other ending: the *pack* goes critical while the launch
 * is still flying the aircraft, which is the failsafe the launch has to give
 * way to - a launch that outlived a failsafe would be a mode that switched one
 * off, which is the bug that block in main.c exists because of. */
static int      sim_launch_pack;
static int      sim_launch_sagged;
static int      sim_launch_status_done;
/* A receiver that never says anything at all - not a lost link, but a
 * receiver that is not there: the state `calibrate rc` has its own sentence
 * for. */
static int      sim_rc_quiet;

#define SIM_RANGE_MAX_MM 2000

static void range_refresh(void)
{
    double mm = (double)plant_alt_m * 1000.0;

    if (mm < 0.0) {
        mm = 0.0;
    }
    if (sim_noisy) {
        /* A real part's noise at this range, and the same seeded generator the
         * rest of the sensors use, so a noisy run is reproducible. */
        mm += (double)sim_noise(40.0f);
        if (mm < 0.0) {
            mm = 0.0;
        }
    }
    /* Past the end of its range the part reports a number that is not a
     * distance, which is what the driver's rule reads as "nothing there". */
    if (mm >= (double)SIM_RANGE_MAX_MM) {
        mm = (double)SIM_RANGE_MAX_MM + 1000.0;
    }
    range_regs[0x00] = (uint8_t)(((uint32_t)mm >> 8) & 0xFFu);
    range_regs[0x01] = (uint8_t)((uint32_t)mm & 0xFFu);
}

static int range_bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    if (sim_range_stop_ms != 0u && virtual_ms >= sim_range_stop_ms) {
        return -1;
    }
    if (reg == 0x00) {
        range_refresh();
    }
    for (unsigned i = 0; i < len; i++) {
        buf[i] = range_regs[(reg + i) & 0xFFu];
    }
    return 0;
}

static int range_bus_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    range_regs[reg] = value;
    return 0;
}

static const ak_bus_t range_bus = {
    .read = range_bus_read,
    .write = range_bus_write,
    .delay_ms = sim_bus_delay,
    .ctx = 0,
};

/* --- the receiver --------------------------------------------------------- */

static ak_ring_t rc_ring;

/*
 * The telemetry the firmware sends back up the receiver's wire, decoded here
 * the way the receiver and the handset decode it.
 *
 * A check that says "bytes went out" proves almost nothing: the interesting
 * question is whether the handset would show the aircraft that is actually
 * flying - the pack it is holding, the attitude the plant has, the position the
 * GPS is reporting, the mode the flight core is in. So this is a parser, not a
 * buffer: sync byte, length, type, crc, and then the fields of the frames this
 * firmware sends.
 */
typedef struct {
    uint8_t  frame[64];
    unsigned held;
    unsigned expected;
    uint32_t frames;
    uint32_t crc_errors;
    uint32_t battery_frames, attitude_frames, gps_frames, mode_frames;
    int      have_battery;
    float    volts;
    int      percent;
    int      have_attitude;
    float    roll_rad, pitch_rad, yaw_rad;
    int      have_fix;
    int32_t  lat_e7, lon_e7;
    float    alt_m, speed_mm_s, course_deg;
    unsigned satellites;
    char     mode[16];
    int      have_mode;
    unsigned modes_seen; /* one bit per distinct mode string, for the checks */
    /* The answer to a device ping: a handset asks who is on the link before it
     * shows anything at all, and this is what came back. */
    uint32_t device_info_frames;
    uint8_t  info_dest, info_origin, info_parameters, info_version;
    char     info_name[40];
} sim_tlm_t;

static sim_tlm_t rc_tlm;

static uint16_t tlm_get_u16(const uint8_t *at)
{
    return (uint16_t)(((uint16_t)at[0] << 8) | at[1]);
}

static uint32_t tlm_get_u32(const uint8_t *at)
{
    return ((uint32_t)at[0] << 24) | ((uint32_t)at[1] << 16) |
           ((uint32_t)at[2] << 8) | at[3];
}

static void tlm_mode_seen(const char *text)
{
    static const char *names[] = { "DISARM", "ANGLE", "ACRO", "RTH", "!FS!" };

    for (unsigned i = 0; i < sizeof names / sizeof names[0]; i++) {
        if (strcmp(text, names[i]) == 0) {
            rc_tlm.modes_seen |= 1u << i;
        }
    }
}

static void tlm_decode(unsigned type, const uint8_t *payload, unsigned len)
{
    switch (type) {
    case AK_CRSF_TYPE_BATTERY:
        if (len < 8u) {
            return;
        }
        rc_tlm.battery_frames++;
        rc_tlm.have_battery = 1;
        rc_tlm.volts = (float)tlm_get_u16(payload) / 10.0f;
        rc_tlm.percent = payload[7];
        break;
    case AK_CRSF_TYPE_ATTITUDE:
        if (len < 6u) {
            return;
        }
        rc_tlm.attitude_frames++;
        rc_tlm.have_attitude = 1;
        rc_tlm.pitch_rad = (float)(int16_t)tlm_get_u16(&payload[0]) / 10000.0f;
        rc_tlm.roll_rad = (float)(int16_t)tlm_get_u16(&payload[2]) / 10000.0f;
        rc_tlm.yaw_rad = (float)(int16_t)tlm_get_u16(&payload[4]) / 10000.0f;
        break;
    case AK_CRSF_TYPE_GPS:
        if (len < 15u) {
            return;
        }
        rc_tlm.gps_frames++;
        rc_tlm.have_fix = 1;
        rc_tlm.lat_e7 = (int32_t)tlm_get_u32(&payload[0]);
        rc_tlm.lon_e7 = (int32_t)tlm_get_u32(&payload[4]);
        rc_tlm.speed_mm_s = (float)tlm_get_u16(&payload[8]) * 1000.0f / 36.0f;
        rc_tlm.course_deg = (float)tlm_get_u16(&payload[10]) / 10.0f;
        rc_tlm.alt_m = (float)tlm_get_u16(&payload[12]) - 1000.0f;
        rc_tlm.satellites = payload[14];
        break;
    case AK_CRSF_TYPE_FLIGHT_MODE: {
        unsigned n = 0;

        rc_tlm.mode_frames++;
        while (n < len && payload[n] != 0u && n + 1u < sizeof rc_tlm.mode) {
            rc_tlm.mode[n] = (char)payload[n];
            n++;
        }
        rc_tlm.mode[n] = '\0';
        rc_tlm.have_mode = 1;
        tlm_mode_seen(rc_tlm.mode);
        break;
    }
    case AK_CRSF_TYPE_DEVICE_INFO: {
        /*
         * The answer to a ping, decoded the way a handset decodes it:
         * [dest][origin][name, zero terminated][twelve bytes of serial number]
         * [how many parameters it offers over this protocol][version]. That
         * layout is Betaflight's `crsfFrameDeviceInfo` (upstream/
         * betaflight-2026.6.1, src/main/telemetry/crsf.c), which is what a
         * handset has actually been built against.
         */
        unsigned n = 0;

        if (len < 17u) {
            return;
        }
        rc_tlm.device_info_frames++;
        rc_tlm.info_dest = payload[0];
        rc_tlm.info_origin = payload[1];
        while (2u + n + 1u < len && payload[2u + n] != 0u &&
               n + 1u < sizeof rc_tlm.info_name) {
            rc_tlm.info_name[n] = (char)payload[2u + n];
            n++;
        }
        rc_tlm.info_name[n] = '\0';
        rc_tlm.info_parameters = payload[len - 2u];
        rc_tlm.info_version = payload[len - 1u];
        break;
    }
    default:
        return;
    }
}

/* One byte out of the firmware's transmit path, parsed as the receiver would. */
static void tlm_feed(uint8_t byte)
{
    if (rc_tlm.held == 0) {
        if (byte == AK_CRSF_ADDRESS_FLIGHT_CONTROLLER) {
            rc_tlm.frame[0] = byte;
            rc_tlm.held = 1;
            rc_tlm.expected = 0;
        }
        return;
    }

    rc_tlm.frame[rc_tlm.held++] = byte;
    if (rc_tlm.held == 2) {
        unsigned len = rc_tlm.frame[1];
        if (len < 2u || len + 2u > sizeof rc_tlm.frame) {
            rc_tlm.held = 0;
            return;
        }
        rc_tlm.expected = len + 2u;
        return;
    }
    if (rc_tlm.expected == 0u || rc_tlm.held < rc_tlm.expected) {
        return;
    }

    {
        unsigned len = rc_tlm.frame[1];
        uint8_t crc = rc_tlm.frame[rc_tlm.expected - 1u];
        uint8_t want = ak_crsf_crc8(&rc_tlm.frame[2], (uint8_t)(len - 1u));

        rc_tlm.held = 0;
        rc_tlm.expected = 0;
        if (crc != want) {
            rc_tlm.crc_errors++;
            return; /* a frame the handset would drop */
        }
        rc_tlm.frames++;
        tlm_decode(rc_tlm.frame[2], &rc_tlm.frame[3],
                   (len - 2u)); /* type + payload + crc, minus type and crc */
    }
}

static uint16_t rc_channels[AK_CRSF_CHANNELS];
static uint32_t rc_frames;
/* The receiver is *alive and in its own failsafe*: it keeps sending frames,
 * with the flag up (where the protocol has one) and the channels at whatever
 * failsafe was configured. See rc_send_sbus and the scenario's STEP_YAW. */
static int rc_failsafe;
static uint32_t rc_frames_at_failsafe;
static uint32_t rc_bytes_offered;
static uint32_t rc_bytes_taken;

/*
 * Which protocol the simulated receiver is speaking. The firmware sets it
 * through the board contract - the same call a real board gets when somebody
 * changes the parameter - so the whole path is exercised: parameter, board,
 * parser, flight core. The default is CRSF, which is what the board boots at.
 */
static uint32_t sim_rc_protocol = AK_RC_PROTOCOL_CRSF;

static void rc_send_crsf(void)
{
    uint8_t frame[26];
    uint8_t payload[22];
    memset(payload, 0, sizeof payload);

    for (int n = 0; n < AK_CRSF_CHANNELS; n++) {
        unsigned bit = (unsigned)n * 11u;
        unsigned at = bit / 8u;
        unsigned shift = bit % 8u;
        uint32_t value = (uint32_t)rc_channels[n] & 0x7FFu;
        uint32_t window = value << shift;

        /* Three bytes at most, and the last channel's eleven bits only reach
         * the second: the loop used to write the third regardless, which is a
         * byte past the end of the payload for channel fifteen. It landed on
         * something that was overwritten a line later, so nothing ever showed,
         * and the address sanitizer found it in about four seconds. */
        payload[at] |= (uint8_t)(window & 0xFFu);
        if (at + 1u < sizeof payload) {
            payload[at + 1u] |= (uint8_t)((window >> 8) & 0xFFu);
        }
        if (at + 2u < sizeof payload) {
            payload[at + 2u] |= (uint8_t)((window >> 16) & 0xFFu);
        }
    }

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = AK_CRSF_TYPE_RC_CHANNELS;
    memcpy(&frame[3], payload, sizeof payload);
    frame[25] = ak_crsf_crc8(&frame[2], 23);

    for (uint8_t i = 0; i < sizeof frame; i++) {
        ak_ring_push(&rc_ring, frame[i]);
    }
    rc_bytes_offered += sizeof frame;
    rc_frames++;
}

/*
 * The handset asking who is on the link: a device ping (0x28), which the
 * receiver forwards to the flight controller on the wire the channel frames
 * arrive on.
 *
 * The payload is the pair of addresses - destination then origin - so the
 * frame is six bytes, and the firmware answers it with a device-info frame
 * once per ping (src/core/main.c's telemetry_service). A handset sends this
 * before it shows anything at all, which is why it is worth a check of its
 * own: a flight controller that does not answer it is a flight controller the
 * radio's own display says nothing about.
 */
static uint32_t crsf_pings_sent;

static void rc_send_crsf_ping(void)
{
    uint8_t frame[6];

    frame[0] = AK_CRSF_ADDRESS_FLIGHT_CONTROLLER; /* to the flight controller */
    frame[1] = 4u;                                /* type, two addresses, crc */
    frame[2] = AK_CRSF_TYPE_DEVICE_PING;
    frame[3] = AK_CRSF_ADDRESS_FLIGHT_CONTROLLER; /* destination */
    frame[4] = AK_CRSF_ADDRESS_RADIO_TRANSMITTER; /* origin: the handset */
    frame[5] = ak_crsf_crc8(&frame[2], 3u);

    for (uint8_t i = 0; i < sizeof frame; i++) {
        ak_ring_push(&rc_ring, frame[i]);
    }
    rc_bytes_offered += sizeof frame;
    crsf_pings_sent++;
}

/*
 * The same channels, in the other protocol. Twenty-five bytes: a 0x0F header,
 * twenty-two bytes of channels packed little-endian eleven bits at a time, the
 * flags byte, and a 0x00 footer.
 *
 * Built here rather than by calling the firmware's packer, for the same reason
 * the CRSF frame above is: two independent implementations that agree are
 * evidence, and a simulator that reuses the parser's own packing can only ever
 * prove the parser agrees with itself.
 */
static void rc_send_sbus(void)
{
    uint8_t frame[AK_SBUS_FRAME];
    memset(frame, 0, sizeof frame);

    frame[0] = 0x0Fu;
    /* The flags byte: bit 2 is lost frames, bit 3 is the receiver's own
     * failsafe - "I am alive and I have lost my transmitter". */
    frame[AK_SBUS_FRAME - 2u] = rc_failsafe ? AK_SBUS_FLAG_FAILSAFE : 0u;
    frame[AK_SBUS_FRAME - 1u] = 0x00u;

    for (unsigned n = 0; n < AK_SBUS_CHANNELS; n++) {
        unsigned bit = n * 11u;
        unsigned at = 1u + bit / 8u;
        unsigned shift = bit % 8u;
        uint32_t window = ((uint32_t)rc_channels[n] & 0x7FFu) << shift;

        /* Never the last three bytes: they are the flags and the footer, and a
         * write of zero into the footer is a frame nothing will accept. */
        frame[at] |= (uint8_t)(window & 0xFFu);
        if (at + 1u < AK_SBUS_FRAME - 2u) {
            frame[at + 1u] |= (uint8_t)((window >> 8) & 0xFFu);
        }
        if (at + 2u < AK_SBUS_FRAME - 2u) {
            frame[at + 2u] |= (uint8_t)((window >> 16) & 0xFFu);
        }
    }

    for (unsigned i = 0; i < sizeof frame; i++) {
        ak_ring_push(&rc_ring, frame[i]);
    }
    rc_bytes_offered += sizeof frame;
    rc_frames++;
}

static void rc_send(void)
{
    if (sim_rc_quiet) {
        return;                 /* a receiver that is not on the aircraft */
    }
    if (sim_rc_protocol == AK_RC_PROTOCOL_SBUS) {
        rc_send_sbus();
    } else {
        rc_send_crsf();
    }
}

/* --- the GPS -------------------------------------------------------------- */

static ak_ring_t gps_ring;

/* Where the module's origin is - the point the aircraft is sitting on when the
 * firmware captures home, and the point the straight-line metres of the plant
 * are measured from. 52.1234567 N, 4.9876543 E, and an altitude that is the
 * same number in millimetres because the navigation compares altitudes. */
#define SIM_HOME_LAT_E7  521234567
#define SIM_HOME_LON_E7  49876543
#define SIM_HOME_ALT_MM  120000

/* Metres per degree of latitude and of longitude, at the origin. The longitude
 * number is what makes a metre east a different number of degrees from a metre
 * north, which is the mistake a flat-earth model makes. */
#define SIM_M_PER_DEG_LAT 111320.0f
static float sim_m_per_deg_lon(void)
{
    return SIM_M_PER_DEG_LAT * ak_cosf(ak_deg2rad((float)SIM_HOME_LAT_E7 * 1e-7f));
}

static void gps_send(void)
{
    uint8_t frame[8 + AK_GPS_UBX_NAV_PVT_LENGTH];
    uint8_t payload[AK_GPS_UBX_NAV_PVT_LENGTH];
    memset(payload, 0, sizeof payload);

    /* Built from the layout INAV's own ubx_nav_pvt describes, not from the
     * parser's constants: a wrong constant has to fail a check here rather than
     * be copied into the simulator and pass. */
    ak_ubx_nav_pvt_t pvt;
    memset(&pvt, 0, sizeof pvt);
    pvt.fix_type = AK_GPS_FIX_3D;
    pvt.flags = 0x01;
    pvt.satellites = (uint8_t)sim_gps_sats;
    /* The plant's position, in the units the module reports it in. A quad that
     * has not moved reports the origin, which is exactly what the first fix
     * while disarmed has to be for home to be the take-off point. */
    pvt.latitude = SIM_HOME_LAT_E7 +
                   (int32_t)(plant_north_m / SIM_M_PER_DEG_LAT * 1e7f);
    pvt.longitude = SIM_HOME_LON_E7 +
                    (int32_t)(plant_east_m / sim_m_per_deg_lon() * 1e7f);
    /* And the one lie a receiver can tell that no flag admits to: a position
     * that is simply wrong, with the fix type, the satellite count and the
     * accuracy figures all exactly as they were. */
    if (sim_gps_jump_until_ms != 0u && virtual_ms >= sim_gps_jump_at_ms &&
        virtual_ms < sim_gps_jump_until_ms) {
        /* North-south, which is across the path home rather than along it: a
         * glitch that points the same way the aircraft is already going is a
         * glitch the saturated velocity command hides. */
        pvt.latitude +=
            (int32_t)(sim_gps_jump_north_m / SIM_M_PER_DEG_LAT * 1e7f);
    }
    pvt.altitude_ellipsoid =
        SIM_HOME_ALT_MM + (int32_t)(plant_alt_m * 1000.0f);
    pvt.altitude_msl = pvt.altitude_ellipsoid;
    pvt.horizontal_accuracy = 1200;
    /* A good fix is a metre and a half wide, and this one is *noisy*: the
     * position moves by that much from frame to frame, which is what the
     * navigator's position loop has to eat. */
    if (sim_noisy) {
        pvt.latitude +=
            (int32_t)(sim_noise(1.5f) / SIM_M_PER_DEG_LAT * 1e7f);
        pvt.longitude +=
            (int32_t)(sim_noise(1.5f) / sim_m_per_deg_lon() * 1e7f);
    }
    /*
     * What the module measures is how the aircraft is *travelling*: the
     * direction and length of its path over the ground, which for a quad flying
     * sideways is not the direction it is pointing. That is not a detail -
     * it is the only heading measurement a magnetometer-less aircraft has
     * (ak_estimator_aid_heading), so a simulator that reported the nose
     * direction here would be handing the firmware a measurement no GPS makes.
     * Standing still it has no direction of travel and reports the last one it
     * had, which is what a module does.
     */
    {
        float speed_m_s = sqrtf(plant_vn * plant_vn + plant_ve * plant_ve);

        pvt.speed_2d = (int32_t)(speed_m_s * 1000.0f);
        if (speed_m_s > 0.5f) {
            pvt.heading_2d =
                (int32_t)(atan2f(plant_ve, plant_vn) * 57.29578f * 100000.0f);
        } else {
            pvt.heading_2d = (int32_t)(plant_heading_deg * 100000.0f);
        }
        /* And the course it reports is jittery too, by about a degree - which
         * matters because the quadrotor's yaw comes from it. */
        if (sim_noisy && speed_m_s > 0.5f) {
            pvt.heading_2d += (int32_t)(sim_noise(1.0f) * 100000.0f);
        }
    }
    pvt.position_dop = 123;
    memcpy(payload, &pvt, sizeof pvt);

    frame[0] = AK_GPS_UBX_SYNC1;
    frame[1] = AK_GPS_UBX_SYNC2;
    frame[2] = 0x01;
    frame[3] = 0x07;
    frame[4] = (uint8_t)AK_GPS_UBX_NAV_PVT_LENGTH;
    frame[5] = 0;
    memcpy(&frame[6], payload, sizeof payload);
    uint8_t a;
    uint8_t b;
    ak_gps_ubx_checksum(&frame[2],
                        (uint16_t)(4u + AK_GPS_UBX_NAV_PVT_LENGTH), &a, &b);
    frame[6 + AK_GPS_UBX_NAV_PVT_LENGTH] = a;
    frame[7 + AK_GPS_UBX_NAV_PVT_LENGTH] = b;

    for (unsigned i = 0; i < sizeof frame; i++) {
        ak_ring_push(&gps_ring, frame[i]);
    }
}

/* --- the scenario --------------------------------------------------------- */

typedef enum {
    STEP_IDLE = 0,
    STEP_ARM,
    STEP_TAKE_OFF,
    STEP_PITCH,
    STEP_ROLL,
    STEP_YAW,
    STEP_LOSE_LINK,
    STEP_RECONNECT,
    STEP_END,
} step_t;

static step_t step;
static uint32_t step_since;
static float rc_roll;
static float rc_pitch;
static float rc_yaw;
static float rc_throttle;
static int rc_arm;
/* Channel 8: the launch switch, when a scenario asks for one. */
static int rc_launch;
static int rc_link_lost;

/* What the aircraft did, measured at the only place the simulator can honestly
 * measure it: the numbers the mixer handed to the timers. */
static uint32_t link_lost_ms;
/* The height it was at when the receiver came out, for the session that
 * watches a wing with no navigator come down: what "it descended" has to be
 * measured against. */
static float alt_at_link_loss_m;
static uint32_t motors_first_ms;
static uint32_t motors_last_ms;
static float motor_peak;
static float roll_at_link_loss_deg;
static float roll_at_test_deg;
/* Three seconds after the pilot asks for a left roll, rather than the last
 * sample of the run: how long the return took to get home decides how much of
 * the run is left for the pilot, and with a noisy fix the return takes longer
 * (33.5 s against 22.6 s), which left the pilot's phase outside a 40-second
 * session and failed a check that had nothing wrong with it. */
static float roll_after_reconnect_deg = 1.0e9f;
static uint32_t reconnect_ms;
static float pitch_at_test_deg;
static float heading_before_yaw_deg;
static float yaw_turned_deg;
/* A small stick: the yaw rate at full stick is a racing quad's 350 degrees a
 * second, which would have the aircraft round three times before the check
 * looked. */
#define SIM_YAW_STICK 0.15f
static float servo_peak;

/* The barometer's own answer, and where the airframe was when it gave it. The
 * console tee is the only place both are visible at once, which is why the
 * capture below fills these in. */
static float baro_reported_m;
static float fused_reported_m;
static float baro_truth_m;
static int   baro_reported;
static int   fused_reported;

/* The output test, as the timers saw it: which outputs moved, and whether
 * everything came back to zero afterwards. */
static int      output_test_running;
static int      output_test_ran;
static unsigned output_test_motors;
static unsigned output_test_servos;
static float    output_test_after_peak;
/* And what each motor was left at, so a check can tell "the aircraft's own
 * symmetric idle" from "the sweep, still going". */
static float    output_test_after_motor[AK_MAX_MOTORS];
/* Frames written, and the count when the test was stopped: the sample taken in
 * the same loop pass as the typed command still holds the frame from before it,
 * so the check starts at the first frame written *after* the stop. */
static uint32_t output_frames;
static uint32_t output_stop_frame;

/* And for the wing, where it went: distance from the take-off point, which is
 * what "return to home" is a claim about. */
static float distance_m;
static float distance_max_m;
static float distance_min_after_loss_m;
static float distance_at_end_m;
static float alt_at_link_loss_m;
static float alt_at_end_m;
static float roll_at_end_deg;
static float loiter_max_m;

static float distance_from_home_m(void)
{
    return (float)sqrt((double)(plant_north_m * plant_north_m +
                                plant_east_m * plant_east_m));
}

/* The radio and the GPS run on their own clocks, as they do on a bench, and
 * both are started when the firmware reaches its main loop rather than while
 * it is still booting: a transmitter talking into a receive ring that nothing
 * is draining yet drops frames, and a drop during the boot blink says nothing
 * about the code under test. */
#define SIM_RC_PERIOD_MS  20u   /* 50 Hz, the rate CRSF sends channel frames at */
#define SIM_GPS_PERIOD_MS 200u  /* 5 Hz, a plain UBX NAV-PVT rate */

static int loop_started;
static uint32_t next_rc_ms;
static uint32_t next_gps_ms;
static uint32_t next_trace_ms;
static uint32_t gps_frames_sent;

/* The loop's first pass is where the radio's and the GPS's deadlines are set
 * from. Both the scripted modes and the console mode come through here, so
 * they start their clocks the same way. */
static void note_loop_started(void)
{
    if (!loop_started) {
        loop_started = 1;
        next_rc_ms = virtual_ms;
        next_gps_ms = virtual_ms;
        next_trace_ms = virtual_ms;
    }
}

/* The stick the scenario is holding, as a fraction of full travel, and what the
 * angle loop should therefore be chasing: max_tilt_deg times the stick. */
#define SIM_ROLL_STICK 0.5f

/* How far over the aircraft is held in the `tilt` session. Sixty degrees is
 * past every reasonable limit (25 is the reference default) and is an angle a
 * person holding a quadrotor on a bench reaches without trying. */
#define TILT_HELD_DEG 60.0f

/* When the aircraft is put down level in that session. The check that no output
 * ever came out of the tilted phase is a comparison against this, so the number
 * lives in one place rather than in the timeline and the check both. */
#define TILT_LEVEL_MS 10000u
#define SIM_MAX_TILT_DEG 35.0f

/* Defined next to the console tee, far below, and read here. */
static int console_said(const char *needle);
static int console_count(const char *needle);
static int console_pair(const char *needle, unsigned *first, unsigned *second);
static int console_number(const char *name, double *out);
static int console_accel(int out[3]);

/*
 * The bench scenario: six positions, one command each, the way a person does
 * it. It cannot be driven by a clock the way the others are - each `calibrate
 * accel` blocks the firmware inside the CLI handler until the face is
 * measured, and the console poll is not running - so it is driven by what the
 * firmware prints: send a face, wait for the report that names it, send the
 * next. A five-second deadline per face stops a failure from hanging the run
 * instead of failing it.
 */
static const struct {
    float roll_deg;
    float pitch_deg;
} bench_faces[AK_ACCEL_FACES] = {
    {   0.0f,   0.0f }, /* level           */
    { 180.0f,   0.0f }, /* inverted        */
    {   0.0f,  90.0f }, /* nose down       */
    {   0.0f, -90.0f }, /* nose up         */
    { -90.0f,   0.0f }, /* right side down */
    {  90.0f,   0.0f }, /* left side down  */
};

/*
 * A line the simulator types itself, for the scenarios that have to work out
 * what to say: the bench, whose next command depends on what came back, and the
 * mission, whose waypoints are a conversion of the metres the plant flies in.
 * The time-driven console script below cannot express either.
 */
static const char *typed_line;
static unsigned typed_at;

static char bench_sentinel[48];     /* what the firmware prints when done */
static char bench_line[32];
static int bench_face;              /* the next face to send */
static int bench_after;             /* what to type once the faces are in */
static uint32_t bench_wait_since;
static uint32_t bench_test_started_ms;

static void bench_say(const char *line)
{
    typed_line = line;
    typed_at = 0;
}

static void bench_step(void)
{
    if (!loop_started || typed_line != 0) {
        return;
    }

    /* Anything already printed and still waiting: wait for it. */
    if (bench_sentinel[0] != '\0') {
        if (!console_said(bench_sentinel) &&
            (uint32_t)(virtual_ms - bench_wait_since) < 5000u) {
            return;
        }
        bench_sentinel[0] = '\0';
    }

    if (bench_face < AK_ACCEL_FACES) {
        int face = bench_face++;
        sim_held = 1;
        sim_hold_roll_deg = bench_faces[face].roll_deg;
        sim_hold_pitch_deg = bench_faces[face].pitch_deg;

        printf("sim: %6u ms  holding the aircraft %s\n", virtual_ms,
               ak_accel_cal_face_name(face));

        if (face == AK_ACCEL_FACES - 1) {
            snprintf(bench_sentinel, sizeof bench_sentinel, "accel bias:");
        } else {
            snprintf(bench_sentinel, sizeof bench_sentinel,
                     "calibrate accel: %s done", ak_accel_cal_face_name(face));
        }
        bench_wait_since = virtual_ms;
        snprintf(bench_line, sizeof bench_line, "calibrate accel %d\n", face);
        bench_say(bench_line);
        return;
    }

    /* All six are in. Level again, then ask the firmware what it thinks the
     * sensor is reading now - the correction is only worth having if it shows
     * up in the sample the flight loop uses. */
    if (bench_after == 0) {
        sim_hold_roll_deg = 0.0f;
        sim_hold_pitch_deg = 0.0f;
        bench_after = 1;
        return;
    }
    if (bench_after == 1) {
        bench_after = 2;
        bench_say("imu\n");
        return;
    }
    if (bench_after == 2) {
        bench_after = 3;
        bench_say("params\n");
        return;
    }
    if (bench_after == 3) {
        bench_after = 4;
        /* The pack's divider, the way the checklist says to do it: a
         * multimeter's number and one command. The simulator's own pack is
         * 12.0 V behind an 11:1 divider, so a person saying 12.60 gets a ratio
         * of 11.55 - and the point of the check is that the division happens in
         * the firmware rather than in somebody's head. */
        snprintf(bench_sentinel, sizeof bench_sentinel, "calibrate vbat:");
        bench_wait_since = virtual_ms;
        bench_say("calibrate vbat 12.60\n");
        return;
    }
    if (bench_after == 4) {
        bench_after = 5;
        /* The table again, because the calibration just changed a parameter and
         * the checks read the *console's* dump rather than the firmware's
         * variable: a value that is right in RAM and wrong on the console is
         * still a value nobody can read. */
        bench_say("params\n");
        return;
    }
    if (bench_after == 5) {
        bench_after = 6;
        bench_say("log long clear\n");
        return;
    }
    /* And the output test, which is the one thing on this bench that makes a
     * pin move: a person with a scope watches each output in turn. The
     * simulator watches the same frames, from the board side of them. */
    if (bench_after == 6) {
        bench_after = 7;
        bench_test_started_ms = virtual_ms;
        bench_say("output test\n");
        return;
    }
    if (bench_after == 7 && virtual_ms > bench_test_started_ms + 9500u) {
        bench_after = 8;
        bench_say("output test stop\n");
    }
}

/*
 * The mission's own timeline. The waypoints are typed rather than poked in,
 * because the console path - parse, range check, parameter table, navigator -
 * is the one a person uses and therefore the one that has to work.
 */
static void mission_type_waypoint(int index)
{
    char lat[16];
    char lon[16];

    float lat_deg = (float)SIM_HOME_LAT_E7 * 1e-7f +
                    mission_wp_m[index][0] / SIM_M_PER_DEG_LAT;
    float lon_deg = (float)SIM_HOME_LON_E7 * 1e-7f +
                    mission_wp_m[index][1] / sim_m_per_deg_lon();

    ak_format_fixed(lat_deg, 7, lat, sizeof lat);
    ak_format_fixed(lon_deg, 7, lon, sizeof lon);
    snprintf(mission_line, sizeof mission_line, "mission add %s %s\n", lat, lon);
    printf("sim: %6u ms  typed: mission add %s %s\n", virtual_ms, lat, lon);
    typed_line = mission_line;
    typed_at = 0;
}

/* How far the aircraft is from a waypoint, in the metres the plant flies in. */
static float mission_distance_m(int index)
{
    float north = plant_north_m - mission_wp_m[index][0];
    float east = plant_east_m - mission_wp_m[index][1];
    return (float)sqrt((double)(north * north + east * east));
}

static void mission_step(void)
{
    if (!loop_started || typed_line != 0 || sim_airframe != SIM_MISSION) {
        return;
    }

    /* A tape of the mission's own geometry, once a second: how far each
     * waypoint still is, which is the number every check at the end of a run
     * is about. The generic tape prints attitude and motors; a mission that is
     * flying the wrong way, or circling a waypoint it cannot close on, is
     * visible here and nowhere else. */
    {
        static uint32_t mission_tape_ms;

        if ((int32_t)(virtual_ms - mission_tape_ms) >= 0) {
            mission_tape_ms = virtual_ms + 1000u;
            printf("sim: %6u ms  %.0f m to wp0, %.0f m to wp1, %.0f m up, "
                   "nose %.0f deg, the firmware believes %.0f deg\n",
                   virtual_ms, (double)mission_distance_m(0),
                   (double)mission_distance_m(1), (double)plant_alt_m,
                   (double)plant_heading_deg,
                   rc_tlm.have_attitude ? (double)ak_rad2deg(rc_tlm.yaw_rad)
                                        : -999.0);
        }
    }

    /*
     * When the transmitter switch goes on, and then everything after it,
     * measured from there rather than from the boot: a quadrotor has to be
     * *climbed* before a mission that starts on the ground is a mission at
     * all, and the wing climbs out on its own nose-up.
     */
    uint32_t t0 = mission_airframe == 0 ? 14000u : 6000u;
    /*
     * In still air a quadrotor has to earn its heading before it can fly
     * anywhere: it has no compass and the only heading there is comes from the
     * track it makes, so the navigator's first few seconds are a straight
     * shove (see quad_step). That is ten seconds the wind used to save it -
     * with a breeze the drift alone gave the estimator a track - so the two
     * reports and the switch-off move back by the same amount, and the run is
     * given the extra time rather than the mission being cut short.
     *
     * It is 27 s rather than 15 because the attitude propagation was repaired
     * (ak_estimator.c, `est_propagate`): the mission now takes longer to fly,
     * and the allowance is what has to give, not the mission.
     *
     * Measured, by bisecting this constant against `mission quad calm` and
     * reading the `counted both arrivals` check:
     *
     *     estimator          least extra time that passes
     *     Euler-rate step     9 s   (6 s of the old 15 s was slack)
     *     quaternion step    21 s
     *
     * So the same 6 s of slack costs 27 s. Two numbers matter here and the
     * second one is the point: 21 s is the requirement, and landing on it
     * exactly would leave a session that passes because nothing moved. The
     * check is a mission the aircraft flies, not a race it wins, so it keeps
     * the margin it had. Where the time goes is heading, not attitude: in that
     * mission both estimators are up to 140 deg out in yaw - there is no
     * compass and the track is the only heading there is - while the repaired
     * one tracks the plant's roll to 1-2 deg where the old one is 7-25 deg out.
     */
    uint32_t calm_extra =
        (mission_airframe == 0 && sim_wind_n_m_s == 0.0f &&
         sim_wind_e_m_s == 0.0f)
            ? 15000u
            : 0u;
    /* And how long the mission itself takes: the same list, flown at a
     * quadrotor's six metres a second instead of a wing's ten, over the same
     * five hundred metres. */
    uint32_t t_status = t0 + (mission_airframe == 0 ? 30000u : 19000u);
    uint32_t t_report1 = t0 + (mission_airframe == 0 ? 50000u : 24000u);
    uint32_t t_report2 =
        t0 + calm_extra + (mission_airframe == 0 ? 95000u : 44000u);
    uint32_t t_off = t0 + calm_extra + (mission_airframe == 0 ? 100000u : 49000u);

    /* The airframe first: `airframe` selects the mixer *and* the navigator's
     * profile, so this is also what decides whether the aircraft circles a
     * waypoint or hovers over it. */
    if (mission_stage == 0 && virtual_ms > 1000u) {
        mission_stage = 1;
        /* The pack's own reason to come home is off by default, so the run
         * that is about it turns it on - and it is typed before the airframe
         * because the airframe line is the one the checks already expect
         * first in the transcript. */
        typed_line = sim_mission_flatpack
                         ? (mission_airframe == 0
                                ? "set battery_rth 1\nset airframe 0\n"
                                : "set battery_rth 1\nset airframe 1\n")
                         : (mission_airframe == 0 ? "set airframe 0\n"
                                                  : "set airframe 1\n");
        typed_at = 0;
        return;
    }
    if (mission_stage == 1 && virtual_ms > 4000u) {
        mission_stage = 2;
        mission_type_waypoint(0);
        return;
    }
    if (mission_stage == 2 && virtual_ms > 4500u) {
        mission_stage = 3;
        typed_line = "set mission_channel 7\n";
        typed_at = 0;
        return;
    }
    if (mission_stage == 3 && virtual_ms > 5500u) {
        mission_stage = 4;
        mission_type_waypoint(1);
        return;
    }
    if (mission_stage == 4 && virtual_ms > t0) {
        mission_stage = 5;
        mission_start_alt_m = plant_alt_m;
        printf("sim: %6u ms  mission switch on, at %.0f m\n", virtual_ms,
               (double)plant_alt_m);
        /* The transmitter, not the console: a mode a pilot can select in the
         * air is the point of having one. */
        mission_switch_on = 1;
        return;
    }
    /*
     * And, if this is the run about it, the pack going critical while the
     * mission is flying: the pilot asked for the waypoints, and a dead pack is
     * a reason to stop flying them. See the checks at the end of the run.
     *
     * Two seconds after the `status` the mission's own timeline types at
     * t0 + 30 s, so that report reads as the mission it was: on autopilot,
     * flying the list, with the switch on.
     */
    if (sim_mission_flatpack && !sim_mission_sagged &&
        virtual_ms > t0 + 32000u) {
        sim_mission_sagged = 1;
        sim_pack_v = SIM_PACK_SAG_V;
        printf("sim: %6u ms  the pack goes critical at %.1f m out\n",
               virtual_ms, (double)distance_from_home_m());
        return;
    }
    /* One `status` mid-mission: the flight core's own account of who is flying
     * is the thing this scenario exists to check. */
    if (mission_stage == 5 && virtual_ms > t_status) {
        mission_stage = 6;
        typed_line = "status\n";
        typed_at = 0;
        return;
    }
    /* Two reports while it flies, so the transcript shows the arrivals being
     * counted rather than only the end state. */
    if (mission_stage == 6 && virtual_ms > t_report1) {
        mission_stage = 7;
        typed_line = "mission\n";
        typed_at = 0;
        return;
    }
    if (mission_stage == 7 && virtual_ms > t_report2) {
        mission_stage = 8;
        typed_line = "mission\n";
        typed_at = 0;
        return;
    }
    /*
     * And the way a mission ends that no session had run: the pilot takes it
     * back with a *stick*, with the switch still on.
     *
     * That is the rule the mission shares with the launch - a stick is the
     * fastest way for a person to stop something - and it leaves the switch in
     * the one state that has its own branch in main.c: still asking, with the
     * mission already stopped, where the answer must be "cycle it" rather than
     * a mission that starts itself again. Two seconds of stick, then centred,
     * and the reports the checks read.
     */
    if (sim_mission_takeback &&
        mission_stage == 8 && virtual_ms > t_off - 5000u) {
        mission_stage = 9;
        printf("sim: %6u ms  the pilot moves a stick, with the switch on\n",
               virtual_ms);
        rc_roll = 0.35f;
        return;
    }
    if (sim_mission_takeback && mission_stage == 9 &&
        virtual_ms > t_off - 3000u) {
        mission_stage = 10;
        printf("sim: %6u ms  the stick comes back to centre\n", virtual_ms);
        rc_roll = 0.0f;
        return;
    }
    if (sim_mission_takeback && mission_stage == 10 &&
        virtual_ms > t_off - 1500u) {
        mission_stage = 11;
        typed_line = "mission\n";
        typed_at = 0;
        return;
    }
    /*
     * And the switch, which is the half of a mode that matters: a pilot who
     * can start something from the transmitter has to be able to stop it from
     * there too. Every run reaches this - the plain one and the flat pack with
     * the mission already over, and the take-back run whose mission a stick
     * ended (stage 11) - except the hold runs, which are the two sessions
     * about what happens when nobody lets go.
     */
    if ((mission_stage == 8 || mission_stage == 11) && virtual_ms > t_off) {
        if (sim_mission_hold) {
            /* The long run: the switch stays on. Everything after this is the
             * aircraft flying the mission it was given, which is what a
             * session about holding is about - the alternative, letting go
             * here, leaves the last four minutes flying on whatever the
             * script's final stick positions were, and that is a property of
             * the script rather than of the firmware. Frozen numbers are not
             * taken either: the checks at the end read the *live* distance and
             * altitude. */
            mission_stage = 20;
            printf("sim: %6u ms  the mission is left on - holding from here\n",
                   virtual_ms);
            return;
        }
        mission_stage = 12;
        printf("sim: %6u ms  mission switch off\n", virtual_ms);
        mission_switch_on = 0;
        /* Freeze what the mission did here: everything after this is the
         * pilot, and a check about the mission should not be measuring the
         * flying that comes after it. */
        mission_end_m[0] = mission_distance_m(0);
        mission_end_m[1] = mission_distance_m(1);
        mission_end_alt_m = plant_alt_m;
        mission_finished = 1;
        return;
    }
    if (mission_stage == 12 && virtual_ms > t_off + 2000u) {
        mission_stage = 13;
        typed_line = "status\n";
        typed_at = 0;
        return;
    }
    /* And, in the take-back run, one more reading of the same counters after
     * the switch has gone off: what makes "the switch still on did not start it
     * again" a check rather than a single sample. */
    if (sim_mission_takeback && mission_stage == 13 &&
        virtual_ms > t_off + 4000u) {
        mission_stage = 14;
        typed_line = "mission\n";
        typed_at = 0;
    }
}

/*
 * The quadrotor's return, flown rather than unit-tested.
 *
 * The wing's return is checked in this file already, and the quadrotor's was
 * checked only by a host test with a point mass - because this simulator's quad
 * had no motion to be returned from. Now it has: a tilt accelerates it, the
 * collective holds it up, and the yaw torque turns it, so the profile that
 * climbs, translates and settles can be watched doing exactly that.
 *
 * The scenario is built to need the yaw: the aircraft starts pointing *east*
 * while the estimator's yaw starts at zero, so the body-frame translation the
 * profile commands is wrong by ninety degrees until the ground-track
 * alignment has had time to work. If that alignment never happens, the quad
 * flies confidently away from home instead of to it, which is what the checks
 * would say.
 */
static int quadrth_stage;
static uint32_t quadrth_since;
static float quadrth_out_m;
static float quadrth_out_alt_m;
static float quadrth_engage_yaw_error_deg;
static float quadrth_closest_m;
static int quadrth_yaw_checked;
static uint32_t quadrth_arrived_ms;
static uint32_t quadrth_out_ms;
static uint32_t quadrth_link_back_ms;
static float quadrth_hover_alt_m;
static float quadrth_hover_alt_later_m;
static float quadrth_arrived_m;
static float quadrth_motor_stop_alt_m;
static float quadrth_motor_stop_m;
static float quadrth_descend_m;
static int   quadrth_have_descend;
static float quadrth_hold_sum_m;
static unsigned quadrth_hold_samples;
static uint32_t quadrth_hold_next_ms;
/* What the flight did while the *fix* was bad rather than gone: whether the
 * motors kept turning, and how well it held the one thing it could still
 * measure. */
static float badfix_min_collective = 1.0e9f;
static float badfix_alt_low_m = 1.0e9f;
static float badfix_alt_high_m = -1.0e9f;
static int   badfix_reported;
static int   badfix_preflight_report;
static int   quadrth_motors_stopped;
static int   quadrth_restarted;
static uint32_t quadrth_trace_next_ms;
/* The heading the firmware believes, sampled twice while the aircraft is
 * parked and the receiver is back: the difference between the two is drift,
 * and drift with nothing moving is a gyro bias the arm-time calibration did
 * not remove. See the check at the end of a run. */
static int   gyro_drift_sampled;
static float gyro_drift_yaw_a;
/* The landing-in-place session's own tape: what happened to an aircraft that
 * came down with no position reference at all. */
static int   gpslost_land_stopped;
static float gpslost_land_stop_alt_m;
static float gpslost_land_stop_m;
static float gpslost_land_min_alt_m = 1.0e9f;
static float gpslost_land_min_collective = 1.0e9f;
static int   gpslost_land_said_landing;
static uint32_t gpslost_land_status_ms;

static void quadrth_step(void)
{
    uint32_t elapsed = virtual_ms - quadrth_since;

    if (!loop_started) {
        return;
    }


    if (quadrth_stage == 0 && elapsed > 1000u) {
        quadrth_stage = 1;
        typed_line = "set rth_enable 1\n";
        typed_at = 0;
        return;
    }
    /* The bare board's run asks the console what it thinks it is before
     * anything flies, because that is the report a person reads on a devkit
     * out of the bag - and because those lines are the ones that say which
     * parts are not there. */
    if (sim_bare && quadrth_stage == 1 && !sim_bare_report_done &&
        elapsed > 1800u) {
        sim_bare_report_done = 1;
        /* The preflight, and then the two reports a person types on a board
         * with nothing soldered to it: "is there a barometer? is there a
         * pack?" - both of which have their own lines, and both of which say
         * what the aircraft does without them. */
        typed_line = "preflight\nbaro\nbattery\n";
        typed_at = 0;
        return;
    }
    if (quadrth_stage == 1 && elapsed > 2500u) {
        /* The switch first, with the throttle still at the bottom: arming
         * needs the throttle down, and a scenario that opens both at once never
         * arms at all - which is how the first version of this sat on the
         * ground for fifty seconds with the motors at zero. */
        quadrth_stage = 2;
        printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    if (quadrth_stage == 2 && elapsed > 4000u) {
        quadrth_stage = 3;
        printf("sim: %6u ms  throttle up, climbing\n", virtual_ms);
        /* And the handset asks who is on the link while the aircraft is
         * climbing: twice, so that "one answer per ping" is a check rather
         * than a hope - a firmware that answered on every telemetry tick would
         * flood the link it is flying on. */
        rc_send_crsf_ping();
        rc_throttle = 0.58f;
        return;
    }
    if (quadrth_stage == 3 && elapsed > 7000u) {
        quadrth_stage = 4;
        printf("sim: %6u ms  nose down, flying out east\n", virtual_ms);
        rc_send_crsf_ping();
        rc_pitch = -0.30f;
        return;
    }
    if (quadrth_stage == 4 && elapsed > 22000u) {
        float speed = sqrtf(plant_vn * plant_vn + plant_ve * plant_ve);

        /* Out far enough to have somewhere to come back from, and moving for
         * long enough that the ground-track alignment has had time to work -
         * which is the thing this scenario exists to check. */
        quadrth_stage = 5;
        quadrth_out_m = distance_from_home_m();
        quadrth_out_alt_m = plant_alt_m;
        quadrth_out_ms = virtual_ms;
        rc_pitch = 0.0f;
        rc_link_lost = 1;
        printf("sim: %6u ms  receiver unplugged, %.0f m out at %.1f m, "
               "%.0f m/s\n", virtual_ms, (double)quadrth_out_m,
               (double)plant_alt_m, (double)speed);

        /* How far the yaw estimate is from the track it is actually making.
         * This is the measurement that makes the return possible, and it is
         * the firmware's own number: it arrives in the telemetry attitude
         * frame, decoded above. */
        if (speed > 1.0f) {
            float track_deg = atan2f(plant_ve, plant_vn) * 57.29578f;
            float yaw_deg = ak_rad2deg(rc_tlm.yaw_rad);
            float error = track_deg - yaw_deg;

            while (error > 180.0f) {
                error -= 360.0f;
            }
            while (error < -180.0f) {
                error += 360.0f;
            }
            quadrth_engage_yaw_error_deg = error;
            quadrth_yaw_checked = 1;
        }
        return;
    }
    if (quadrth_stage >= 5 && quadrth_stage <= 8) {
        float distance = distance_from_home_m();

        if (distance < quadrth_closest_m) {
            quadrth_closest_m = distance;
        }

        /* Where it is, once a second, from the link going to the end. The
         * claim this session makes is *where* the return puts the aircraft
         * down, and that is a path rather than a point: the checks read the
         * end of it, and this is the rest of the tape - the descent, how far
         * downwind it reaches the ground, and the last of the distance home
         * with almost no altitude under it. See docs/14-navigation.md. */
        if ((int32_t)(virtual_ms - quadrth_trace_next_ms) >= 0) {
            quadrth_trace_next_ms = virtual_ms + 1000u;
            printf("sim: %6u ms  %.1f m from home, %.1f m up, mode %s\n",
                   virtual_ms, (double)distance, (double)plant_alt_m,
                   rc_tlm.have_mode ? rc_tlm.mode : "?");
        }

        /* Ask the firmware what it thinks the return is doing, a couple of
         * seconds after it engaged, the way the wing's session does: the
         * console's own report is the evidence, not the simulator's
         * assumption. */
        if (quadrth_stage == 5 && (uint32_t)(virtual_ms - quadrth_out_ms) > 4000u) {
            quadrth_stage = 6;
            typed_line = "gps\n";
            typed_at = 0;
            return;
        }

        /* Where the descent starts, which is the thing the return's own
         * arrival radius is *for*: the aircraft is brought back at the
         * altitude it was holding and only then comes down, so the distance
         * at the moment it first drops out of that altitude is the number
         * that says whether the return ends over the pad or somewhere
         * downwind of it. */
        if (!quadrth_have_descend &&
            plant_alt_m < quadrth_out_alt_m - 3.0f) {
            quadrth_have_descend = 1;
            quadrth_descend_m = distance;
            printf("sim: %6u ms  started coming down %.1f m from home, "
                   "%.1f m up\n",
                   virtual_ms, (double)distance, (double)plant_alt_m);
        }

        /* And, if this run is the one about it, the barometer giving up as the
         * descent begins: from here the aircraft is leaning on the height for
         * the only thing this firmware stops its own motors for. */
    if (sim_baro_fail_on_descent && sim_baro_stop_ms == 0u &&
            quadrth_have_descend) {
            sim_baro_stop_ms = virtual_ms;
            printf("sim: %6u ms  the barometer stops answering, %.1f m up, "
                   "%.1f m from home\n",
                   virtual_ms, (double)plant_alt_m, (double)distance);
        }

        /* And the rangefinder giving up at the same moment, for the session
         * that asks what happens when the part the landing rule prefers is the
         * one that dies: the barometric rule has to still be there. */
        if (sim_range_fail_on_descent && quadrth_range_stage == 0 &&
            sim_range_stop_ms == 0u && quadrth_have_descend) {
            sim_range_stop_ms = virtual_ms;
            printf("sim: %6u ms  the rangefinder stops answering, %.1f m up, "
                   "%.1f m from home\n",
                   virtual_ms, (double)plant_alt_m, (double)distance);
        }

        /*
         * And the same part coming *back*, which is the other half of a
         * sensor failure and the half no scenario had: a wire that was loose,
         * a part that browned out, a connector somebody reseated. The
         * firmware's answer is a sentence rather than a state - "answering
         * again after N failed reads" - and the landing rule has to go back to
         * using the ground once the ground is being measured again.
         *
         * While it is quiet the scenario asks the console what it thinks,
         * because that report (`range`) is what a person reads in the air: the
         * part, the failing count, and which rule the landing will use.
         */
        if (sim_range_fail_on_descent && sim_range_stop_ms != 0u &&
            quadrth_range_stage == 0 &&
            (uint32_t)(virtual_ms - sim_range_stop_ms) > 1000u) {
            quadrth_range_stage = 1;
            typed_line = "range\n";
            typed_at = 0;
            return;
        }
        if (sim_range_fail_on_descent && quadrth_range_stage == 1 &&
            (uint32_t)(virtual_ms - sim_range_stop_ms) > 4000u) {
            quadrth_range_stage = 2;
            printf("sim: %6u ms  the rangefinder answers again, %.1f m up\n",
                   virtual_ms, (double)plant_alt_m);
            sim_range_stop_ms = 0u;
            return;
        }

        /*
         * And, if this run is the one about it, the fix going *bad* rather than
         * away: the frames keep arriving at five a second, with a 3D fix the
         * receiver calls ok, and four satellites. Twenty seconds into the
         * return, and back to eleven twenty seconds later, so the check can be
         * that a bad fix pauses the navigation without ending the flight.
         */
        if (sim_badfix_on_return && sim_gps_bad_ms == 0u &&
            quadrth_stage >= 6 && (uint32_t)(virtual_ms - quadrth_out_ms) > 20000u) {
            sim_gps_bad_ms = virtual_ms;
            sim_gps_good_ms = virtual_ms + 20000u;
            sim_gps_sats = 4u;
            printf("sim: %6u ms  the fix degrades to 4 satellites, %.1f m up, "
                   "%.1f m from home\n",
                   virtual_ms, (double)plant_alt_m, (double)distance);
        } else if (sim_badfix_on_return && sim_gps_good_ms != 0u &&
                   virtual_ms >= sim_gps_good_ms && sim_gps_sats != 11u) {
            sim_gps_sats = 11u;
            printf("sim: %6u ms  the fix is good again, 11 satellites, %.1f m "
                   "up, %.1f m from home\n",
                   virtual_ms, (double)plant_alt_m, (double)distance);
        }

        /*
         * And the fix that lies: forty-five seconds into the return - on the
         * way home, at the altitude it is holding - the position jumps fifty
         * metres east for two seconds. The type, the satellites and the
         * accuracy figures are untouched, so this is the one kind of bad fix
         * the navigator has no way to refuse.
         */
        if (sim_gps_jump_on_return && sim_gps_jump_at_ms == 0u &&
            quadrth_stage >= 6 && distance < 20.0f) {
            sim_gps_jump_at_ms = virtual_ms;
            sim_gps_jump_until_ms = virtual_ms + 2000u;
            sim_gps_jump_north_m = 50.0f;
            printf("sim: %6u ms  the fix jumps 50 m north for two seconds, "
                   "%.1f m from home, %.1f m up\n",
                   virtual_ms, (double)distance, (double)plant_alt_m);
        }

        /* While it is bad: what the aircraft does, and asking the console what
         * the firmware makes of the fix - the numbers on the tape are the
         * simulator's, and this is the firmware's own answer. */
        if (sim_badfix_on_return && sim_gps_sats < 6u) {
            float collective = (plant_outputs.motor[0] + plant_outputs.motor[1] +
                                plant_outputs.motor[2] +
                                plant_outputs.motor[3]) *
                               0.25f;

            if (collective < badfix_min_collective) {
                badfix_min_collective = collective;
            }
            if (plant_alt_m < badfix_alt_low_m) {
                badfix_alt_low_m = plant_alt_m;
            }
            if (plant_alt_m > badfix_alt_high_m) {
                badfix_alt_high_m = plant_alt_m;
            }
            if (!badfix_reported && sim_gps_bad_ms != 0u &&
                virtual_ms > sim_gps_bad_ms + 5000u) {
                badfix_reported = 1;
                typed_line = "gps\n";
                typed_at = 0;
                return;
            }
            /* And the checklist's own view of it: what the preflight says about
             * a return the pilot has asked for and cannot have. */
            if (badfix_reported && !badfix_preflight_report &&
                sim_gps_bad_ms != 0u && virtual_ms > sim_gps_bad_ms + 9000u) {
                badfix_preflight_report = 1;
                typed_line = "preflight\n";
                typed_at = 0;
                return;
            }
        }

        /* Arrived means it has actually come back *and come down*: not "it
         * happens to be inside the arrival radius at the moment the link
         * went", which is what the first version of this checked, and not
         * "it is inside the radius" either, which is what the second one did
         * - with the descent gated on the quadrotor's own fifteen metres, the
         * aircraft crosses twenty-five metres of its own accord while still
         * at the altitude it is holding, and a trigger there would stop
         * watching before the part this session exists to see. */
        if (quadrth_stage == 6 && distance < 20.0f && plant_alt_m < 3.5f) {
            quadrth_arrived_ms = virtual_ms;
            quadrth_hover_alt_m = plant_alt_m;
            quadrth_arrived_m = distance;
            printf("sim: %6u ms  first down at the hover height: %.0f m out, "
                   "%.1f m up\n",
                   virtual_ms, (double)distance, (double)plant_alt_m);
            quadrth_stage = 7;
            return;
        }
        /* A few seconds later, still there: the difference between passing
         * through the hover height on the way down and holding it. */
        if (quadrth_stage == 7 &&
            (uint32_t)(virtual_ms - quadrth_arrived_ms) > 4000u) {
            quadrth_hover_alt_later_m = plant_alt_m;
            quadrth_stage = 8;
        }

        /* What "holding a station" is worth, sampled while it is doing it:
         * how far from home the aircraft sits once it is down at the hover
         * height, ten times a second until its motors stop. A proportional
         * position loop needs a standing error to lean against a standing
         * wind; the loop's wind term is what removes it, and this is the
         * measurement that says whether it did. */
        if (quadrth_stage >= 7 && !quadrth_motors_stopped &&
            (int32_t)(virtual_ms - quadrth_hold_next_ms) >= 0) {
            quadrth_hold_next_ms = virtual_ms + 100u;
            quadrth_hold_sum_m += distance;
            quadrth_hold_samples++;
        }
    }

    /* The motors going quiet is the landing rule firing: the navigator was
     * flying, and the height had stopped changing at the ground. The altitude
     * it happens at is the whole safety argument, so it is recorded. */
    if (!quadrth_motors_stopped && quadrth_stage >= 5) {
        float collective = (plant_outputs.motor[0] + plant_outputs.motor[1] +
                            plant_outputs.motor[2] + plant_outputs.motor[3]) *
                           0.25f;

        if (collective < 0.06f) {
            quadrth_motors_stopped = 1;
            quadrth_motor_stop_alt_m = plant_alt_m;
            quadrth_motor_stop_m = distance_from_home_m();
            printf("sim: %6u ms  motors stopped at %.1f m above home, "
                   "%.1f m from it\n",
                   virtual_ms, (double)plant_alt_m,
                   (double)quadrth_motor_stop_m);
        } else if (quadrth_motors_stopped && collective > 0.2f &&
                   !quadrth_restarted) {
            quadrth_restarted = 1;
            printf("sim: %6u ms  motors running again at %.1f m, mode %s\n",
                   virtual_ms, (double)plant_alt_m,
                   rc_tlm.have_mode ? rc_tlm.mode : "?");
        }
    }

    /* Settled, *landed*, and only then does the receiver come back - so the
     * question the last check asks is whether an aircraft that has landed
     * stays landed, which is the behaviour a return that ends in a landing
     * needs and a hover-forever return never had.
     *
     * The trigger is the motors going quiet rather than a second on the clock.
     * It used to be fourteen seconds after the hover began, which was longer
     * than that return took to land: with the descent gated on the quadrotor's
     * own arrival radius the return is a slower manoeuvre, the receiver came
     * back while the aircraft was still coming down, and the session failed
     * three checks for a reason that had nothing to do with the firmware - the
     * pilot took over a return that had not finished. Waiting for the stop is
     * also the honest version of what the comment above says. */
    if (quadrth_stage == 8 && quadrth_motors_stopped &&
        (uint32_t)(virtual_ms - quadrth_arrived_ms) > 10000u) {
        quadrth_stage = 9;
        quadrth_link_back_ms = virtual_ms;
        printf("sim: %6u ms  receiver back - has the pilot got it?\n",
               virtual_ms);
        rc_link_lost = 0;
        /* A pilot who has just taken the aircraft back holds a hover rather
         * than the throttle they were climbing out with. */
        rc_throttle = 0.55f;
        return;
    }
    /* A second later, so the question is asked after the link is live rather
     * than in the same millisecond it came back: a link is live when a *fresh*
     * frame has arrived, and a status typed before that answers "returning
     * home" for the one reason that matters least. */
    if (quadrth_stage == 9 &&
        (uint32_t)(virtual_ms - quadrth_link_back_ms) > 1000u) {
        quadrth_stage = 10;
        typed_line = "status\n";
        typed_at = 0;
    }

    /* And last, the blackbox: the record grew a yaw and a height for this
     * session's sake, and the check is that the *firmware* fills them - which
     * is a different claim from the layout tests, and the reason to ask after
     * a flight rather than after a bench run on the ground. */
    if (quadrth_stage == 10 &&
        (uint32_t)(virtual_ms - quadrth_link_back_ms) > 4000u) {
        quadrth_stage = 11;
        /* The *flash* log, which is the one that holds the whole flight: the
         * fast ring is a second and a half, so after a landing it holds a
         * second and a half of a parked aircraft. */
        typed_line = "log flash\n";
        typed_at = 0;
    }

    /*
     * And, once the receiver is back and the aircraft is parked, the first of
     * the two heading samples the check at the end of the run reads: the
     * difference between this and the last telemetry frame is how far the
     * firmware's heading walked while nothing at all was moving.
     */
    if (sim_gyro_bias_dps != 0.0f && !gyro_drift_sampled &&
        rc_tlm.have_attitude && quadrth_stage >= 10 &&
        (uint32_t)(virtual_ms - quadrth_link_back_ms) > 1000u) {
        gyro_drift_sampled = 1;
        gyro_drift_yaw_a = rc_tlm.yaw_rad;
    }
}

/*
 * The GPS module goes quiet on the way home.
 *
 * A return is a manoeuvre that exists because the link is gone, and the one
 * measurement it cannot do without is the position. So this is the worst
 * combination the firmware can be handed: nobody is flying it, and the sensor
 * that knows where it is stops answering. What happens next is the scenario,
 * and the answer before this was written was *nothing good* - the navigator
 * gave up, the flight core saw a lost link with no guidance and did the
 * correct thing for a lost link, which is stop the motors, and a quadrotor
 * with its motors stopped is a falling object. The checks are the shape of
 * the answer this firmware should give instead: keep flying, hold what it can
 * still measure, and say so.
 */
static int   gpslost_stage;
static int   gpslost_cut;
static float gpslost_alt_at_cut_m;
static float gpslost_min_alt_m = 1.0e9f;
static float gpslost_min_collective = 1.0e9f;
static uint32_t gpslost_tape_ms;

static void gpslost_step(void)
{
    if (!loop_started) {
        return;
    }

    if (gpslost_stage == 0 && virtual_ms > 1000u) {
        gpslost_stage = 1;
        /* The landing variant is the same flight with one thing configured
         * differently, so it is typed rather than compiled: a pilot who wants
         * the aircraft to come down when it has no position to fly home to
         * says so, and one who does not gets the hold. */
        typed_line = sim_airframe == SIM_GPS_LOST_LAND
                         ? "set rth_enable 1\nset quad_hold_land_s 8\n"
                         : "set rth_enable 1\n";
        typed_at = 0;
        return;
    }
    if (gpslost_stage == 1 && virtual_ms > 2500u) {
        gpslost_stage = 2;
        printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    if (gpslost_stage == 2 && virtual_ms > 4000u) {
        gpslost_stage = 3;
        printf("sim: %6u ms  throttle up, climbing\n", virtual_ms);
        rc_throttle = 0.58f;
        return;
    }
    if (gpslost_stage == 3 && virtual_ms > 7000u) {
        gpslost_stage = 4;
        printf("sim: %6u ms  nose down, flying out east\n", virtual_ms);
        rc_pitch = -0.30f;
        return;
    }
    if (gpslost_stage == 4 && virtual_ms > 20000u) {
        gpslost_stage = 5;
        rc_pitch = 0.0f;
        rc_link_lost = 1;
        printf("sim: %6u ms  receiver unplugged, %.0f m out at %.1f m up\n",
               virtual_ms, (double)distance_from_home_m(),
               (double)plant_alt_m);
        return;
    }
    if (gpslost_stage == 5 && virtual_ms > 31000u) {
        /* What the console says *before* the module goes quiet, so the two
         * reports are the same question asked either side of the event. */
        gpslost_stage = 6;
        typed_line = "gps\n";
        typed_at = 0;
        return;
    }
    if (gpslost_stage == 6 && virtual_ms > 32000u) {
        gpslost_stage = 7;
        sim_gps_stop_ms = virtual_ms;
        gpslost_cut = 1;
        gpslost_alt_at_cut_m = plant_alt_m;
        printf("sim: %6u ms  the gps module goes quiet, %.0f m from home at "
               "%.1f m up, mode %s\n",
               virtual_ms, (double)distance_from_home_m(),
               (double)plant_alt_m, rc_tlm.have_mode ? rc_tlm.mode : "?");
        return;
    }
    if (gpslost_stage >= 7) {
        float collective = (plant_outputs.motor[0] + plant_outputs.motor[1] +
                            plant_outputs.motor[2] + plant_outputs.motor[3]) *
                           0.25f;

        if (plant_alt_m < gpslost_min_alt_m) {
            gpslost_min_alt_m = plant_alt_m;
        }
        if (collective < gpslost_min_collective) {
            gpslost_min_collective = collective;
        }
        if ((int32_t)(virtual_ms - gpslost_tape_ms) >= 0) {
            gpslost_tape_ms = virtual_ms + 1000u;
            printf("sim: %6u ms  %.1f m from home, %.1f m up, mode %s\n",
                   virtual_ms, (double)distance_from_home_m(),
                   (double)plant_alt_m, rc_tlm.have_mode ? rc_tlm.mode : "?");
        }
        if (gpslost_stage == 7 && virtual_ms > 42000u) {
            gpslost_stage = 8;
            typed_line = "gps\n";
            typed_at = 0;
        }

        /*
         * The landing-in-place session watches the same plant, and asks a
         * different question: it was told to come down where it is, so did it?
         * The motors stopping is the flight core's landing rule - the one
         * thing this firmware does that a mistake cannot undo - so the height
         * that happens at is the number the whole feature is judged on.
         */
        if (sim_airframe == SIM_GPS_LOST_LAND) {
            if (plant_alt_m < gpslost_land_min_alt_m) {
                gpslost_land_min_alt_m = plant_alt_m;
            }
            /* Before the motors stop, because the check below is about the
             * *descent* being flown rather than fallen - and after they stop
             * the collective is zero by definition, which would read as a
             * fall. */
            if (collective >= 0.06f &&
                collective < gpslost_land_min_collective) {
                gpslost_land_min_collective = collective;
            }
            /* What the console says about it, typed once the descent has been
             * running long enough to be reported rather than started. */
            if (!gpslost_land_said_landing && virtual_ms > 50000u) {
                gpslost_land_said_landing = 1;
                typed_line = "status\n";
                typed_at = 0;
            }
            if (!gpslost_land_stopped && collective < 0.06f) {
                gpslost_land_stopped = 1;
                gpslost_land_stop_alt_m = plant_alt_m;
                gpslost_land_stop_m = distance_from_home_m();
                printf("sim: %6u ms  the motors stop: %.2f m up, %.0f m from "
                       "home, %u ms after the module went quiet\n",
                       virtual_ms, (double)plant_alt_m,
                       (double)gpslost_land_stop_m,
                       (unsigned)(virtual_ms - sim_gps_stop_ms));
            }
            /* And once more a few seconds after the motors stop, so the check
             * can read the state the aircraft ended in rather than the state
             * it was in while it was coming down. */
            if (gpslost_land_stopped && gpslost_land_status_ms == 0u &&
                virtual_ms > 5000u) {
                gpslost_land_status_ms = 1u;
                typed_line = "status\n";
                typed_at = 0;
            }
        }
    }
}

/*
 * The pack goes flat with somebody flying it.
 *
 * This is the one return trigger that is not about where the aircraft is or
 * whether anybody is still talking to it: the link is up, the pilot is flying,
 * and the energy is running out. Every flight controller that means to bring
 * aircraft home has an answer for it, and this one had none - the pack state
 * was measured, filtered, reported on the console and sent to the handset, and
 * *nothing acted on it*. So a quadrotor whose cells fell below the critical
 * threshold went on flying until they could not hold it up.
 *
 * The scenario also sags the pack back to a healthy voltage partway through
 * the return, because that is what a pack does under load and it is the case
 * that separates a latch from a level test: a return that follows the voltage
 * back up hands the aircraft to a pilot who is watching a flat battery.
 */
static int   battery_stage;
static float battery_alt_at_sag_m;

static void battery_step(void)
{
    if (!loop_started) {
        return;
    }

    if (battery_stage == 0 && virtual_ms > 1000u) {
        battery_stage = 1;
        typed_line = "set battery_rth 1\n";
        typed_at = 0;
        return;
    }
    if (battery_stage == 1 && virtual_ms > 2500u) {
        battery_stage = 2;
        printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    if (battery_stage == 2 && virtual_ms > 4000u) {
        battery_stage = 3;
        printf("sim: %6u ms  throttle up, climbing out\n", virtual_ms);
        rc_throttle = 0.68f;
        return;
    }
    if (battery_stage == 3 && virtual_ms > 7000u) {
        battery_stage = 4;
        printf("sim: %6u ms  nose down, flying out east\n", virtual_ms);
        rc_pitch = -0.30f;
        return;
    }
    if (battery_stage == 4 && virtual_ms > 15000u) {
        battery_stage = 5;
        sim_pack_v = SIM_PACK_SAG_V;
        battery_alt_at_sag_m = plant_alt_m;
        printf("sim: %6u ms  the pack sags to %.2f V, %.2f V a cell, %.0f m "
               "out at %.1f m up\n",
               virtual_ms, (double)sim_pack_v, (double)(sim_pack_v / 3.0f),
               (double)distance_from_home_m(), (double)plant_alt_m);
        return;
    }
    if (battery_stage == 5 && virtual_ms > 25000u) {
        battery_stage = 6;
        sim_pack_v = 12.0f;
        sim_pack_recovered = 1;
        printf("sim: %6u ms  the pack recovers to %.2f V under no load\n",
               virtual_ms, (double)sim_pack_v);
        return;
    }
    /* What the console says while the aircraft is coming home. */
    if (battery_stage == 6 && virtual_ms > 30000u) {
        battery_stage = 7;
        typed_line = "gps\n";
        typed_at = 0;
        return;
    }
    if (battery_stage == 7 && virtual_ms > 100000u) {
        battery_stage = 8;
        typed_line = "status\n";
        typed_at = 0;
    }
}

/*
 * The sticks in rate mode.
 *
 * The mode channel selects one of two flight laws, and until this was written
 * only one of them had ever been flown: every host test passed angle mode, and
 * this simulator held the mode channel high for every session it ran. So the
 * branch a pilot actually flies a quadrotor with by hand - the rate loop,
 * straight from the stick to a rotation rate with nothing levelling the
 * aircraft - had no evidence at all beyond the code compiling.
 *
 * What the scenario asks is what a pilot would: a full roll stick for two
 * seconds should produce a roll rate near the one the parameter asks for, in
 * the right direction; centring the stick should *stop* the rotation; the
 * pitch and yaw sticks should do the same on their axes; and the aircraft
 * should stay where the stick left it rather than levelling, because levelling
 * is the other mode's job.
 */
static int   rate_stage;
/* What the scenario asks `max_rate_dps` to be, so the check is against the
 * parameter the session set rather than the compiled-in one - which is also
 * how the parameter's own path is exercised. */
#define SIM_RATE_MAX_DPS 400.0f
static float rate_roll_peak_dps;
static float rate_pitch_peak_dps;
static float rate_yaw_peak_dps;
static float rate_roll_after_centre_dps;
static float rate_roll_angle_after_centre_deg;

/*
 * The gyro stops answering in flight.
 *
 * This is the one sensor the flight core refuses to fly without, and the rule
 * it applies is the blunt one: no attitude is not a state to fly in, so the
 * motors stop and the state latches until the arm switch is cycled. That rule
 * has been in the core since the beginning and host-tested since shortly after
 * - "losing the IMU while armed is a failsafe" - and it had never been flown,
 * which is the gap this closes: the wiring between a failing bus read and a
 * stopped motor is exactly where an integration bug lives.
 *
 * The part comes *back* partway through, because the interesting half of a
 * failsafe is what it does when the fault clears.
 */
static int   noimu_stage;
static uint32_t noimu_ms;
static uint32_t noimu_motors_last_ms;
static float noimu_alt_at_fail_m;

static void noimu_step(void)
{
    if (!loop_started) {
        return;
    }

    if (noimu_stage == 0 && virtual_ms > 1000u) {
        noimu_stage = 1;
        printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    if (noimu_stage == 1 && virtual_ms > 3000u) {
        noimu_stage = 2;
        printf("sim: %6u ms  throttle up, climbing out\n", virtual_ms);
        rc_throttle = 0.64f;
        return;
    }
    if (noimu_stage == 2 && virtual_ms > 6000u) {
        noimu_stage = 3;
        printf("sim: %6u ms  nose down, flying out east\n", virtual_ms);
        rc_pitch = -0.25f;
        return;
    }
    if (noimu_stage == 3 && virtual_ms > 12000u) {
        noimu_stage = 4;
        sim_imu_stop_ms = virtual_ms;
        noimu_ms = virtual_ms;
        noimu_alt_at_fail_m = plant_alt_m;
        printf("sim: %6u ms  the gyro stops answering, %.1f m up\n",
               virtual_ms, (double)plant_alt_m);
        /* The sticks stay where they were: whatever the pilot was doing, the
         * aircraft is the one that has to do something about it. */
        return;
    }
    if (noimu_stage == 4) {
        if (motors_last_ms >= noimu_ms) {
            noimu_motors_last_ms = motors_last_ms;
        }
        if (virtual_ms > 15000u) {
            noimu_stage = 5;
            sim_imu_back_ms = virtual_ms;
            printf("sim: %6u ms  the gyro answers again\n", virtual_ms);
        }
        return;
    }
    if (noimu_stage == 5 && virtual_ms > 17000u) {
        noimu_stage = 6;
        typed_line = "status\n";
        typed_at = 0;
    }
}

/*
 * The aircraft somebody is holding at an angle, with the arm switch thrown.
 *
 * This is the scenario for the gate that was missing: nothing here checks
 * whether a tilted aircraft *would* fly badly, because the question is whether
 * it is allowed to start. The plant is held 60 degrees over, the switch goes
 * on, and what the run is about is three things - the motors never turn, the
 * console says which gate refused it and how far out the aircraft is, and the
 * same switch arms the moment the aircraft is put level, which is what makes
 * the refusal a gate rather than a different bug.
 *
 * It is a bench scenario with a receiver, because the gates that need a
 * transmitter - the switch and the sticks - are exactly the ones a bare board
 * cannot exercise. See ak_flight_arm_check() and `arm_line()` in main.c.
 */
static int tilt_stage;
static float tilt_roll_at_refusal_deg;

/*
 * A hand launch, which is the one manoeuvre in this firmware that exists
 * because a pilot's hands are full: one is on the wing and the other is on the
 * transmitter, and neither can hold a climb stick.
 *
 * What the simulator can fly is the half of it the firmware does. There is no
 * throw in this plant - a wing's speed here follows its throttle, with no
 * acceleration transient and no hand - so the *detection* half of INAV's launch
 * (its twelve-state machine watching the accelerometer and the GPS for the
 * throw) is deliberately not implemented and not tested here: a threshold for
 * an event this model cannot produce is a number nobody could measure. The
 * switch is the detection. What is measured is what happens after it: the
 * motors spool to the launch throttle, the aircraft flies the climb attitude it
 * was given and leaves the ground, and a stick takes it back.
 */
static int   launch_stage;
static uint32_t launch_started_ms;
static float launch_pitch_at_handover_deg;
static float launch_alt_at_handover_m;

static void launch_seq(void)
{
    if (!loop_started) {
        return;
    }

    if (launch_stage == 0 && virtual_ms > 600u) {
        launch_stage = 1;
        /* The pack's run asks for a return when the pack is done, which is what
         * makes "the pack went critical" a *reason* to stop flying rather than
         * only a number on the console. */
        typed_line = sim_launch_pack ? "set battery_rth 1\nset airframe 1\n"
                                     : "set airframe 1\n";
        typed_at = 0;
        return;
    }
    if (launch_stage == 1 && virtual_ms > 1200u) {
        launch_stage = 2;
        typed_line = "set launch_channel 8\n";
        typed_at = 0;
        return;
    }
    /* Arm first, with the throttle down and the launch switch still off: the
     * arming gates are the same for a wing as for anything else, and the launch
     * is a mode on top of an aircraft that is already armed. */
    if (launch_stage == 2 && virtual_ms > 2000u) {
        launch_stage = 3;
        printf("sim: %6u ms  arm switch on, throttle down, launch switch off\n",
               virtual_ms);
        rc_arm = 1;
        return;
    }
    if (launch_stage == 3 && virtual_ms > 3000u) {
        launch_stage = 4;
        launch_started_ms = virtual_ms;
        printf("sim: %6u ms  launch switch on - the aircraft is in a hand\n",
               virtual_ms);
        rc_launch = 1;
        return;
    }
    /*
     * The pack going critical while the launch is still flying the aircraft:
     * a *reason* to stop flying, where the launch is a *mode* somebody asked
     * for. main.c's rule is that the reason wins and the launch stops - and
     * the version of that code which did not is why this scenario exists.
     */
    if (sim_launch_pack && launch_stage >= 4 && !sim_launch_sagged &&
        virtual_ms > 4500u) {
        sim_launch_sagged = 1;
        sim_pack_v = SIM_PACK_SAG_V;
        printf("sim: %6u ms  the pack goes critical, %.0f m up, mid-launch\n",
               virtual_ms, (double)plant_alt_m);
        return;
    }
    /* And the pack's run asks the flight core what it is doing, a few seconds
     * into the return the pack asked for. */
    if (sim_launch_pack && sim_launch_sagged && !sim_launch_status_done &&
        virtual_ms > 12000u) {
        sim_launch_status_done = 1;
        typed_line = "status\n";
        typed_at = 0;
        return;
    }
    /* One status while it is flying the launch: the firmware's own account of
     * who is flying is the thing worth reading, and it is not the same as the
     * pilot's. */
    if (launch_stage == 4 && virtual_ms > 5000u) {
        launch_stage = 5;
        typed_line = "status\n";
        typed_at = 0;
        return;
    }
    /* And the stick, which is what takes it back: a pilot whose other hand is
     * now on the transmitter, with the throttle they are going to fly on. */
    if (launch_stage == 5 && virtual_ms > 7000u) {
        launch_stage = 6;
        launch_pitch_at_handover_deg = ak_rad2deg(plant_pitch);
        launch_alt_at_handover_m = plant_alt_m;
        printf("sim: %6u ms  the pilot takes it: %.0f m up, nose %.0f deg\n",
               virtual_ms, (double)plant_alt_m,
               (double)ak_rad2deg(plant_pitch));
        rc_pitch = 0.2f;
        rc_throttle = 0.6f;
        return;
    }
    if (launch_stage == 6 && virtual_ms > 8000u) {
        launch_stage = 7;
        typed_line = "status\n";
        typed_at = 0;
        return;
    }
    /*
     * And the other way out: the switch itself.
     *
     * The stick is one way to take a launch back and the timeout is another;
     * the third is the switch that started it going *down*, which is the one a
     * pilot reaches for when the throw has gone wrong and the hand is already
     * on the transmitter. So the switch is cycled here - off, then on again,
     * which is a second launch - and then thrown down while that launch is
     * still flying the aircraft.
     */
    if (launch_stage == 7 && virtual_ms > 9000u) {
        if (sim_launch_pack) {
            /* This run's story is the pack's; the second launch and the
             * switch ending are the other scenario's. */
            return;
        }
        launch_stage = 8;
        printf("sim: %6u ms  the launch switch goes off, and on again\n",
               virtual_ms);
        rc_launch = 0;
        /* Hands off the sticks: the pilot has been flying on a pitch stick
         * since the handover, and a launch ends the moment a stick moves - so
         * a second launch started with this stick still held would be over
         * before the switch could end it. */
        rc_pitch = 0.0f;
        rc_roll = 0.0f;
        return;
    }
    if (launch_stage == 8 && virtual_ms > 10000u) {
        launch_stage = 9;
        printf("sim: %6u ms  a second launch, back in the hand\n", virtual_ms);
        rc_launch = 1;
        return;
    }
    if (launch_stage == 9 && virtual_ms > 12000u) {
        launch_stage = 10;
        printf("sim: %6u ms  the switch comes down under it\n", virtual_ms);
        rc_launch = 0;
        return;
    }
    if (launch_stage == 10 && virtual_ms > 13000u) {
        launch_stage = 11;
        typed_line = "status\n";
        typed_at = 0;
    }
}

static void tilt_step(void)
{
    if (!loop_started) {
        return;
    }

    if (tilt_stage == 0 && virtual_ms > 600u) {
        tilt_stage = 1;
        /* Held over since before the first sample: this is an aircraft in
         * somebody's hand, not one that rolled there, so the estimate has
         * never seen anything but this angle. */
        printf("sim: %6u ms  the aircraft is held at %.0f degrees of roll\n",
               virtual_ms, (double)TILT_HELD_DEG);
        return;
    }
    /* Late enough that the estimate has converged and settled on the angle it
     * is held at, so that the refusal is about the tilt rather than about a
     * filter that is still coming up. */
    if (tilt_stage == 1 && virtual_ms > 4000u) {
        tilt_stage = 2;
        tilt_roll_at_refusal_deg = ak_rad2deg(plant_roll);
        printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    if (tilt_stage == 2 && virtual_ms > 6500u) {
        tilt_stage = 3;
        typed_line = "status\n";
        typed_at = 0;
        return;
    }
    if (tilt_stage == 3 && virtual_ms > 9000u) {
        tilt_stage = 4;
        printf("sim: %6u ms  the switch goes off\n", virtual_ms);
        rc_arm = 0;
        return;
    }
    if (tilt_stage == 4 && virtual_ms > TILT_LEVEL_MS) {
        tilt_stage = 5;
        printf("sim: %6u ms  put down level\n", virtual_ms);
        sim_hold_roll_deg = 0.0f;
        return;
    }
    if (tilt_stage == 5 && virtual_ms > 12000u) {
        tilt_stage = 6;
        printf("sim: %6u ms  arm switch on again, throttle down\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    if (tilt_stage == 6 && virtual_ms > 14000u) {
        tilt_stage = 7;
        printf("sim: %6u ms  throttle up, still held\n", virtual_ms);
        rc_throttle = 0.55f;
        return;
    }
    if (tilt_stage == 7 && virtual_ms > 17000u) {
        tilt_stage = 8;
        typed_line = "status\n";
        typed_at = 0;
    }
}

/*
 * An aircraft somebody was holding when it was armed.
 *
 * The gyro bias the firmware measures for itself needs five hundred samples of
 * a *still* aircraft - the samples have to agree with each other, which is
 * what makes an average of them a bias rather than a manoeuvre. An aircraft
 * held in a hand never gives it five hundred in a row, so the calibration is
 * still running when the aircraft arms, and the firmware says so and abandons
 * it: the stored bias stands.
 *
 * The first version of this session assumed the other way in - arm the board
 * faster than the calibration can finish - and the simulator said no: the
 * arming gate holds the switch for `arm_hold_ms` (500 by default) *after* the
 * estimate converges (fifty accelerometer updates, about fifty milliseconds),
 * so a still aircraft always finishes the measurement first. The margin is
 * fifty milliseconds, which is worth knowing before anybody shortens that hold
 * or slows the imu down; the way in that is wide open is a hand.
 *
 * So the scenario is a person's hand: the aircraft wobbles (inside the arming
 * gate's tilt limit, which is what makes it armable at all), the switch is
 * thrown while it is still being held, and the firmware arms it and says what
 * it did not measure. Then it is put down and flown, because an aircraft that
 * is told "the stored bias stands" is an aircraft that is expected to fly.
 */
static int armnow_stage;
static uint32_t armnow_armed_ms;
static float armnow_alt_at_arm_m;

static void armnow_step(void)
{
    if (!loop_started) {
        return;
    }

    if (armnow_stage == 0 && virtual_ms > 500u) {
        armnow_stage = 1;
        printf("sim: %6u ms  somebody is holding the aircraft, and it is "
               "moving\n",
               virtual_ms);
        return;
    }
    if (armnow_stage == 1 && virtual_ms > 2500u) {
        armnow_stage = 2;
        printf("sim: %6u ms  arm switch on, still in the hand\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    if (armnow_stage == 2 && virtual_ms > 3400u) {
        armnow_stage = 3;
        typed_line = "status\n";
        typed_at = 0;
        return;
    }
    if (armnow_stage == 3 && console_said("state:     armed")) {
        armnow_stage = 4;
        armnow_armed_ms = virtual_ms;
        armnow_alt_at_arm_m = plant_alt_m;
        printf("sim: %6u ms  armed with the bias measurement still running\n",
               virtual_ms);
        return;
    }
    if (armnow_stage == 4 && virtual_ms > 4800u) {
        armnow_stage = 5;
        printf("sim: %6u ms  put down on the bench, level and still\n",
               virtual_ms);
        sim_handled = 0;
        sim_hold_roll_deg = 0.0f;
        sim_hold_pitch_deg = 0.0f;
        return;
    }
    if (armnow_stage == 5 && virtual_ms > 5300u) {
        armnow_stage = 6;
        printf("sim: %6u ms  throttle up, flying on the bias it has\n",
               virtual_ms);
        sim_held = 0;
        rc_throttle = 0.58f;
    }
}

/*
 * The output test, and the arm switch.
 *
 * `output test` is the one thing in the firmware that writes the outputs
 * without arming: a sweep of each motor and each servo, one at a time, for
 * somebody checking that the right thing moves. It refuses to start on an
 * armed aircraft - that half is checked in the quadrotor's session - and this
 * is the other half, which no session had run: it stops the moment the
 * aircraft arms, because the pilot who has just taken control is not to share
 * the outputs with a test.
 *
 * The scenario is the bench one: start the sweep, and throw the arm switch
 * while it is running. What makes it a check rather than a hope is that the
 * sweep was *moving something* before it stopped - a test that never started
 * would also never be stopped - and that nothing moved afterwards.
 */
static int testarm_stage;

/*
 * The bench's mistakes: a person doing the calibrations in the wrong order.
 *
 * Every one of them has a sentence of its own, and each sentence is a
 * diagnosis - so what this session is for is the words, in the state that
 * produces them: four calibrations tried while the aircraft is *armed* (which
 * is the refusal that keeps a hand away from a live throttle, and which
 * `calibrate accel` turned out not to have), a pack that is not a pack, a
 * divider with nothing on it, a face that is not a face, a receiver that is
 * not there, and a gyro calibration tried while somebody is moving the
 * aircraft.
 */
static int refuse_stage;

static void refuse_step(void)
{
    if (!loop_started) {
        return;
    }

    if (refuse_stage == 0 && sim_rc_quiet) {
        /* With no receiver there is nothing to arm *on*: the four armed
         * refusals need a link, and this run is about the sentence a
         * calibration gets when the receiver has never spoken. So it goes
         * straight to the disarmed cases. */
        refuse_stage = 2;
        return;
    }
    if (refuse_stage == 0 && virtual_ms > 800u) {
        refuse_stage = 1;
        printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    /* Four calibrations while armed, and the other command that can only be
     * about a bench: `save` writes flash, and this board's erase stalls the
     * loop for about a second - which in the air stops the DShot frames and
     * lets the ESCs' own failsafe time out. Each refuses in its own words, and
     * the accel calibration is the one that used to go ahead. */
    if (refuse_stage == 1 && virtual_ms > 2400u) {
        refuse_stage = 2;
        typed_line = "calibrate\ncalibrate rc\ncalibrate vbat 12.6\n"
                     "calibrate accel 0\nsave\n";
        typed_at = 0;
        return;
    }
    /* Put it down: the rest of the session is what a person sees when the
     * aircraft is disarmed but something else is wrong. */
    if (refuse_stage == 2 && virtual_ms > 3600u) {
        refuse_stage = 3;
        printf("sim: %6u ms  arm switch off, and the pack unplugged\n",
               virtual_ms);
        rc_arm = 0;
        sim_pack_v = 0.0f;      /* the divider, with nothing on it */
        return;
    }
    if (refuse_stage == 3 && virtual_ms > 4800u) {
        refuse_stage = 4;
        typed_line = sim_rc_quiet
                         ? "calibrate vbat 99\ncalibrate vbat 12.6\n"
                           "calibrate accel 9\ncalibrate rc\n"
                         : "calibrate vbat 99\ncalibrate vbat 12.6\n"
                           "calibrate accel 9\n";
        typed_at = 0;
        return;
    }
    /* And the one a moving aircraft produces: somebody picks it up in the
     * middle of a gyro calibration and it refuses to write a bias. */
    if (refuse_stage == 4 && virtual_ms > 7000u) {
        refuse_stage = 5;
        printf("sim: %6u ms  somebody picks the aircraft up\n", virtual_ms);
        sim_handled = 1;
        typed_line = "calibrate\n";
        typed_at = 0;
        return;
    }
    if (refuse_stage == 5 && virtual_ms > 10500u) {
        refuse_stage = 6;
        sim_handled = 0;
        typed_line = "status\n";
        typed_at = 0;
    }
    /* And the bench mistake that looks like a success: the board's flash will
     * not take the record. What a person reads has to say the save did not
     * happen, and what `params` says afterwards has to agree with it - a save
     * that failed silently is an aircraft flying yesterday's settings, which
     * is the one thing a parameter table exists to prevent. */
    if (refuse_stage == 6 && virtual_ms > 12000u) {
        refuse_stage = 7;
        sim_lie_config_write = 1;
        typed_line = "set mission_channel 7\nsave\nparams\n";
        typed_at = 0;
    }
    /* And the other half of the same check: with the board honest again, the
     * same command succeeds - or "it refused" would be all this session could
     * show, and a firmware that refused every save would pass it. */
    if (refuse_stage == 7 && virtual_ms > 14200u) {
        refuse_stage = 8;
        sim_lie_config_write = 0;
        typed_line = "save\n";
        typed_at = 0;
    }
}

static void testarm_step(void)
{
    if (!loop_started) {
        return;
    }

    if (testarm_stage == 0 && virtual_ms > 700u) {
        testarm_stage = 1;
        typed_line = "output test\n";
        typed_at = 0;
        return;
    }
    if (testarm_stage == 1 && virtual_ms > 2000u) {
        testarm_stage = 2;
        printf("sim: %6u ms  arm switch on, with the sweep running\n",
               virtual_ms);
        rc_arm = 1;
        return;
    }
    if (testarm_stage == 2 && console_said("output test: stopped")) {
        testarm_stage = 3;
        printf("sim: %6u ms  the test gave the outputs back\n", virtual_ms);
        typed_line = "status\n";
        typed_at = 0;
    }
}

static void rate_step(void)
{
    if (!loop_started) {
        return;
    }

    if (rate_stage == 0 && virtual_ms > 600u) {
        rate_stage = 1;
        typed_line = "set max_rate_dps 400\n";
        typed_at = 0;
        return;
    }
    if (rate_stage == 1 && virtual_ms > 1200u) {
        rate_stage = 2;
        sim_rate_mode = 1;   /* throw the mode switch before arming */
        printf("sim: %6u ms  mode channel low: rate mode\n", virtual_ms);
        return;
    }
    if (rate_stage == 2 && virtual_ms > 1600u) {
        rate_stage = 3;
        printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
        rc_arm = 1;
        return;
    }
    if (rate_stage == 3 && virtual_ms > 3000u) {
        rate_stage = 4;
        printf("sim: %6u ms  throttle up to hover\n", virtual_ms);
        rc_throttle = 0.56f;
        return;
    }
    if (rate_stage == 4 && virtual_ms > 6000u) {
        rate_stage = 5;
        printf("sim: %6u ms  full right roll stick\n", virtual_ms);
        rc_roll = 1.0f;
        return;
    }
    if (rate_stage == 5) {
        float dps = ak_rad2deg(plant_roll_rate);

        if (dps > rate_roll_peak_dps) {
            rate_roll_peak_dps = dps;
        }
        if (virtual_ms > 8000u) {
            rate_stage = 6;
            printf("sim: %6u ms  stick centred, at %.0f deg/s and %.0f deg\n",
                   virtual_ms, (double)dps,
                   (double)ak_rad2deg(plant_roll));
            rc_roll = 0.0f;
        }
        return;
    }
    if (rate_stage == 6) {
        /* One second later: the rotation should have stopped, and the angle
         * should be wherever the stick left it. */
        if (virtual_ms > 9000u && rate_roll_after_centre_dps == 0.0f) {
            rate_roll_after_centre_dps = ak_rad2deg(plant_roll_rate);
            rate_roll_angle_after_centre_deg = ak_rad2deg(plant_roll);
        }
        if (virtual_ms > 10000u) {
            rate_stage = 7;
            printf("sim: %6u ms  full nose-down pitch stick\n", virtual_ms);
            rc_pitch = -1.0f;
        }
        return;
    }
    if (rate_stage == 7) {
        float dps = ak_rad2deg(plant_pitch_rate);

        if (dps < rate_pitch_peak_dps) {
            rate_pitch_peak_dps = dps; /* nose down is negative */
        }
        if (virtual_ms > 12000u) {
            rate_stage = 8;
            printf("sim: %6u ms  stick centred, full right yaw stick\n",
                   virtual_ms);
            rc_pitch = 0.0f;
            rc_yaw = 1.0f;
        }
        return;
    }
    if (rate_stage == 8) {
        if (plant_yaw_rate_dps > rate_yaw_peak_dps) {
            rate_yaw_peak_dps = plant_yaw_rate_dps;
        }
        if (virtual_ms > 14000u) {
            rate_stage = 9;
            rc_yaw = 0.0f;
        }
    }
}

static void fence_step(void)
{
    if (!loop_started || typed_line != 0 || sim_airframe != SIM_FENCE) {
        return;
    }

    if (fence_stage == 0 && virtual_ms > 1000u) {
        fence_stage = 1;
        typed_line = "set airframe 1\n";
        typed_at = 0;
        return;
    }
    if (fence_stage == 1 && virtual_ms > 2000u) {
        fence_stage = 2;
        typed_line = "set fence_enable 1\n";
        typed_at = 0;
        return;
    }
    if (fence_stage == 2 && virtual_ms > 2500u) {
        fence_stage = 3;
        typed_line = "set fence_radius_m 250\n";
        typed_at = 0;
        return;
    }
    if (fence_stage == 3 && virtual_ms > 3500u) {
        fence_stage = 4;
        typed_line = "set rth_min_alt_m 150\n";
        typed_at = 0;
        return;
    }

    /* The trigger is what the firmware says about it, not what the simulator
     * thinks: a fence that fires without saying so is a fence nobody can
     * debug. */
    if (fence_stage == 4 && console_said("fence:")) {
        fence_stage = 5;
        printf("sim: %6u ms  the fence has it, %.0f m out\n", virtual_ms,
               (double)distance_from_home_m());
        /* The pilot stops pushing out and holds a circle inside the fence. */
        rc_pitch = 0.0f;
        rc_roll = 0.35f;
        typed_line = "status\n";
        typed_at = 0;
        return;
    }
    if (fence_stage == 5 && virtual_ms > 45000u) {
        /* When the navigator hands the aircraft back depends on where in the
         * turn it was when the fence fired, and that is a knife edge: a
         * handful of simulated milliseconds moves the trigger to the other
         * side of "pointing at home" and the return comes round the other way.
         * So this asks until the answer changes rather than once at a fixed
         * second - the check is that the pilot gets it back, not that they get
         * it back by second forty-five. (It was a fixed second until a
         * telemetry tick in the flight loop shifted the clock and this
         * session's verdict with it.) */
        if (!console_said("state:     armed") &&
            (int32_t)(virtual_ms - fence_next_status_ms) >= 0) {
            fence_next_status_ms = virtual_ms + 2000u;
            typed_line = "status\n";
            typed_at = 0;
        } else if (console_said("state:     armed")) {
            fence_stage = 6;
            fence_alt_at_handover_m = plant_alt_m;
        }
    }

    /*
     * And the lid, which is the third side of the same fence.
     *
     * The pilot has the aircraft back and is circling inside the ring, so this
     * is the case a ceiling exists for: somebody flying by hand who does not
     * notice the height. It goes on *above* the aircraft - so the trigger is
     * the climb and not the setting - and what has to happen is what happened
     * when it left the ring.
     */
    if (fence_stage == 6 && virtual_ms > 46000u) {
        fence_stage = 7;
        fence_ceiling_m = (int)(plant_alt_m + 15.0f);
        snprintf(fence_line, sizeof fence_line, "set fence_ceiling_m %d\n",
                 fence_ceiling_m);
        printf("sim: %6u ms  a ceiling goes on at %d m above home\n",
               virtual_ms, fence_ceiling_m);
        typed_line = fence_line;
        typed_at = 0;
        return;
    }
    if (fence_stage == 7 && console_said("fence_ceiling_m = ")) {
        fence_stage = 8;
        printf("sim: %6u ms  climbing for the ceiling, %.0f m above home\n",
               virtual_ms, (double)plant_alt_m);
        rc_pitch = 0.35f;
        rc_throttle = 0.75f;
        return;
    }
    if (fence_stage == 8 && console_said("m up, bringing it back")) {
        fence_stage = 9;
        fence_ceiling_fired_ms = virtual_ms;
        fence_max_after_m = plant_alt_m;
        printf("sim: %6u ms  the ceiling has it, %.0f m above home\n",
               virtual_ms, (double)plant_alt_m);
        /* The navigator is flying now: the pilot levels off and lets it. */
        rc_pitch = 0.0f;
        rc_roll = 0.0f;
        return;
    }
    if (fence_stage == 9) {
        if (plant_alt_m > fence_max_after_m) {
            fence_max_after_m = plant_alt_m;
        }
        /* Two `gps` commands, because the report has two sides to it: one
         * while the navigator is bringing the aircraft down from above the
         * lid, and one after it is under it again. Nothing had ever asked this
         * firmware where the fence thought the aircraft was - the fence
         * itself was checked, the sentence a person reads about it was not. */
        if (!fence_gps_over && virtual_ms > fence_ceiling_fired_ms + 300u) {
            fence_gps_over = 1;
            typed_line = "gps\n";
            typed_at = 0;
            return;
        }
        if (virtual_ms > fence_ceiling_fired_ms + 8000u) {
            fence_stage = 10;
            typed_line = "status\n";
            typed_at = 0;
        }
        return;
    }
    /* And the second report, back under the lid: the run is seventy seconds
     * and the ceiling fires near fifty-five of them, so this is the last
     * question there is time to ask. */
    if (fence_stage == 10 && virtual_ms > fence_ceiling_fired_ms + 11000u) {
        fence_stage = 11;
        typed_line = "gps\n";
        typed_at = 0;
    }
}

/*
 * The board that lies, one fact at a time.
 *
 * The schedule and the script are one thing: this arms the lie, and the
 * console script types `preflight` three hundred milliseconds later, so every
 * answer in the transcript belongs to exactly one lie and the checks can look
 * for the line that lie is supposed to produce. The first and last stages
 * arm nothing, and that is the other half of the test: the command has to
 * come back clean when the board is honest, or "it complained" would be the
 * only thing this session could show.
 */
static int badboard_stage = -1;

static void badboard_arm(int stage)
{
    sim_lie_console_port = 0;
    sim_lie_clock = 0;
    sim_lie_config = 0;
    sim_lie_config_write = 0;
    sim_lie_dshot = 0;
    sim_lie_tick = 0;
    /* The fault record is not a flag the board reads back - it is the record
     * itself, which is why taking the lie back means clearing it. */
    ak_fault.magic = 0;
    ak_fault.count = 0;

    switch (stage) {
    case 1:
        sim_lie_console_port = 1;
        break;
    case 2:
        sim_lie_clock = 1;
        break;
    case 3:
        sim_lie_config = 1;
        break;
    case 4:
        /* Planted the way the crash would leave it, with a pc that is nobody's
         * real address and is on purpose: the check is that the line carries
         * the record's own number rather than a plausible one. */
        ak_fault.magic = AK_FAULT_MAGIC;
        ak_fault.count = 1;
        ak_fault.pc = 0xDEAD0F00u;
        ak_fault.cfsr = 0x00000082u;
        break;
    case 5:
        sim_lie_dshot = 1;
        break;
    case 6:
        sim_lie_tick_ms = ak_time_ms();
        sim_lie_tick = 1;
        break;
    default:
        break;
    }
}

/* Eight stages of half a second: a clean one, the six lies, and a clean one to
 * finish on. */
#define BADBOARD_FIRST_MS 600u
#define BADBOARD_STAGE_MS 500u
#define BADBOARD_STAGES   8

static void badboard_step(void)
{
    int stage;

    /* The two boot-time variants type one command and nothing else: their lie
     * was set before the firmware booted, and the stages below would only take
     * it back - including the DShot rate the `noout` run lies about, which is
     * the whole point of that check being an absent line. */
    if (sim_no_outputs || sim_two_motors) {
        return;
    }
    if (virtual_ms < BADBOARD_FIRST_MS) {
        return;
    }
    stage = (int)((virtual_ms - BADBOARD_FIRST_MS) / BADBOARD_STAGE_MS);
    if (stage >= BADBOARD_STAGES) {
        stage = BADBOARD_STAGES - 1;
    }
    if (stage != badboard_stage) {
        badboard_stage = stage;
        badboard_arm(stage);
    }
}

static void scenario(void)
{
    uint32_t elapsed = virtual_ms - step_since;

    if (sim_airframe == SIM_CONSOLE) {
        return; /* the pilot is a keyboard, and it is not on a clock */
    }
    if (sim_airframe == SIM_FAULT) {
        /* Nothing is flown here: the board that stopped is judged by what its
         * next boot printed, and a scripted arm-and-roll in the middle of that
         * would only put lines in the transcript that mean nothing. */
        return;
    }
    if (sim_airframe == SIM_BADBOARD) {
        /* Also nothing flown: this session is a person with a console and a
         * board that is wrong, and the lie has to be in place before the
         * command that reads it. */
        badboard_step();
        return;
    }
    if (sim_airframe == SIM_GPS_QUIET) {
        /* Nothing flown: the aircraft is on the ground with a module that is
         * not talking, which is exactly when somebody reads `gps`. Without
         * this the session fell through to the quadrotor's own arm-and-roll
         * script - the tail of this function was written when every session
         * that was not named was a quad - and the transcript showed an
         * aircraft flying itself, which is not what "the module is quiet"
         * means. */
        return;
    }
    if (sim_airframe == SIM_QUAD_RTH) {
        quadrth_step();
        return;
    }
    if (sim_airframe == SIM_GPS_LOST || sim_airframe == SIM_GPS_LOST_LAND) {
        gpslost_step();
        return;
    }
    if (sim_airframe == SIM_BATTERY) {
        battery_step();
        return;
    }
    if (sim_airframe == SIM_RATE) {
        rate_step();
        return;
    }
    if (sim_airframe == SIM_NO_IMU) {
        noimu_step();
        return;
    }
    if (sim_airframe == SIM_TILT) {
        tilt_step();
        return;
    }
    if (sim_airframe == SIM_ARM_NOW) {
        armnow_step();
        return;
    }
    if (sim_airframe == SIM_TEST_ARM) {
        testarm_step();
        return;
    }
    if (sim_airframe == SIM_REFUSE) {
        refuse_step();
        return;
    }
    if (sim_airframe == SIM_LAUNCH) {
        launch_seq();
        return;
    }

    /* The quadrotor's pack sags on cue; see SIM_PACK_SAG_V. The wing, the
     * mission and the fence keep theirs, so the healthy path is on the tape
     * too. */
    if (sim_airframe == SIM_QUAD && !sim_pack_sagged &&
        virtual_ms >= SIM_PACK_SAG_MS) {
        sim_pack_sagged = 1;
        sim_pack_v = SIM_PACK_SAG_V;
        printf("sim: %6u ms  the pack sags to %.2f V, a 3S at %.2f V a cell\n",
               virtual_ms, (double)sim_pack_v,
               (double)(sim_pack_v / 3.0f));
    }

    /* The bench does not fly and the mission has its own timeline below; both
     * drive themselves rather than following the timed script. */
    if (sim_airframe == SIM_BENCH) {
        return;
    }
    if (sim_airframe == SIM_MISSION) {
        if (step == STEP_IDLE && elapsed > 1500u) {
            printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
            rc_arm = 1;
            step = STEP_ARM;
            step_since = virtual_ms;
        } else if (step == STEP_ARM && elapsed > 1500u) {
            printf("sim: %6u ms  throttle up, climbing out\n", virtual_ms);
            rc_throttle = 0.55f;
            rc_pitch = 0.15f;
            step = STEP_TAKE_OFF;
            step_since = virtual_ms;
        }
        return;
    }
    /* The pilot's roll, three seconds after the pilot asked for it. This sits
     * here rather than in the phase machine because the phase is over by then:
     * the command is what matters, and the aircraft's answer to it. */
    if (reconnect_ms != 0u && roll_after_reconnect_deg > 1.0e8f &&
        (uint32_t)(virtual_ms - reconnect_ms) >= 3000u) {
        roll_after_reconnect_deg = ak_rad2deg(plant_roll);
    }


    if (sim_airframe == SIM_FENCE) {
        if (step == STEP_IDLE && elapsed > 1500u) {
            printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
            rc_arm = 1;
            step = STEP_ARM;
            step_since = virtual_ms;
        } else if (step == STEP_ARM && elapsed > 1500u) {
            printf("sim: %6u ms  throttle up, flying out north\n", virtual_ms);
            rc_throttle = 0.55f;
            rc_pitch = 0.10f;
            step = STEP_TAKE_OFF;
            step_since = virtual_ms;
        }
        return;
    }

    switch (step) {
    case STEP_IDLE:
        if (elapsed > 1500u) {
            printf("sim: %6u ms  arm switch on, throttle down\n", virtual_ms);
            rc_arm = 1;
            step = STEP_ARM;
            step_since = virtual_ms;
        }
        break;
    case STEP_ARM:
        if (elapsed > 1500u) {
            rc_throttle = 0.55f;
            if (sim_airframe == SIM_MISSION && sim_flies_a_quad()) {
                /* A quadrotor climbs on collective: a hover throttle is a
                 * hover, and a mission that has to be flown from the air needs
                 * more than that to leave the ground. */
                rc_throttle = 0.68f;
                printf("sim: %6u ms  throttle up, climbing out\n", virtual_ms);
            } else if (sim_airframe == SIM_WING ||
                       sim_airframe == SIM_MISSION ||
                       sim_airframe == SIM_FENCE) {
                /* Some nose-up, so the aircraft is not at ground level when the
                 * navigator captures the altitude it is going to hold. */
                rc_pitch = 0.20f;
                printf("sim: %6u ms  throttle up, climb out\n", virtual_ms);
            } else {
                printf("sim: %6u ms  throttle up to hover\n", virtual_ms);
            }
            step = STEP_TAKE_OFF;
            step_since = virtual_ms;
        }
        break;
    case STEP_TAKE_OFF:
        if (sim_airframe == SIM_WING && elapsed > 8000u) {
            printf("sim: %6u ms  receiver unplugged, %.0f m out\n", virtual_ms,
                   (double)distance_m);
            rc_link_lost = 1;
            link_lost_ms = virtual_ms;
            alt_at_link_loss_m = plant_alt_m;
            step = STEP_LOSE_LINK;
            step_since = virtual_ms;
        } else if (sim_airframe != SIM_WING && elapsed > 2000u) {
            /* The pitch axis first, because it is the one every other flight
             * controller has had to get right and the one this firmware had
             * inverted: the stick pushed forward must put the nose *down*. */
            printf("sim: %6u ms  pitch nose down, half stick\n", virtual_ms);
            rc_pitch = -SIM_ROLL_STICK;
            step = STEP_PITCH;
            step_since = virtual_ms;
        }
        break;
    case STEP_PITCH:
        if (elapsed > 2000u) {
            printf("sim: %6u ms  pitch held at %.1f deg, roll right\n",
                   virtual_ms, (double)ak_rad2deg(plant_pitch));
            pitch_at_test_deg = ak_rad2deg(plant_pitch);
            rc_pitch = 0.0f;
            rc_roll = SIM_ROLL_STICK;
            step = STEP_ROLL;
            step_since = virtual_ms;
        }
        break;
    case STEP_ROLL:
        if (elapsed > 2000u) {
            printf("sim: %6u ms  yaw right, a little stick\n", virtual_ms);
            roll_at_test_deg = ak_rad2deg(plant_roll);
            rc_roll = 0.0f;
            rc_yaw = SIM_YAW_STICK;
            heading_before_yaw_deg = plant_heading_deg;
            step = STEP_YAW;
            step_since = virtual_ms;
        }
        break;
    case STEP_YAW:
        if (elapsed > 2000u) {
            /*
             * The turn, the short way round: the plant's heading is 0..360, so
             * a turn that crosses north comes out as a subtraction that is
             * 360 degrees wrong unless it is wrapped. It was not, and a run
             * with sensor noise happened to cross north - the check called a
             * right turn of 93 degrees a left turn of 266, which is the sort
             * of arithmetic this repository keeps finding in its own
             * instruments rather than its aircraft.
             */
            float turned = plant_heading_deg - heading_before_yaw_deg;

            while (turned > 180.0f) {
                turned -= 360.0f;
            }
            while (turned < -180.0f) {
                turned += 360.0f;
            }
            yaw_turned_deg = turned;
            printf("sim: %6u ms  receiver unplugged\n", virtual_ms);
            rc_yaw = 0.0f;
            if (sim_rx_failsafe_on_link_loss) {
                /*
                 * The other way a link is lost: the transmitter is off, the
                 * receiver is alive, and it goes on sending frames with its
                 * own failsafe flag set. The channels are the *last* ones the
                 * pilot held - throttle up, hovering - which is the version of
                 * this that looks exactly like a good command, and the reason
                 * the flag is worth having at all.
                 */
                rc_failsafe = 1;
                rc_frames_at_failsafe = rc_frames;
                printf("sim: %6u ms  receiver in its own failsafe: still "
                       "sending, throttle %.2f, sticks centred\n",
                       virtual_ms, (double)rc_throttle);
            } else {
                rc_link_lost = 1;
            }
            link_lost_ms = virtual_ms;
            roll_at_link_loss_deg = ak_rad2deg(plant_roll);
            alt_at_link_loss_m = plant_alt_m;
            step = STEP_LOSE_LINK;
            step_since = virtual_ms;
        }
        break;
    case STEP_LOSE_LINK:
        if (sim_wing_lost) {
            /* The wing with no navigator is coming down in its circle, and the
             * ground is what ends it. The plant's height is not floored for a
             * wing - the quadrotor's vertical model is the one with a ground
             * under it - so this ends when the descent has happened rather than
             * when an altimeter reads zero: twelve metres of it, from the
             * height the receiver came out at. */
            if (plant_alt_m <= alt_at_link_loss_m - 12.0f) {
                printf("sim: %6u ms  %.0f m lower at %.0f m - the circle came down\n",
                       virtual_ms, (double)(alt_at_link_loss_m - plant_alt_m),
                       (double)plant_alt_m);
                /* And this *ends* the session: the plant has no ground under a
                 * wing, so a run that carried on would fly the aircraft
                 * through it and report a descent measured from below the
                 * field. The numbers the checks read are the ones at
                 * touchdown. */
                sim_finish();
            }
        } else if (sim_airframe == SIM_WING) {
            /* The wing waits for the arrival however long it takes; a timeout
             * here would end the scenario a mile before the thing it is
             * watching. (It did, for one revision: the quad's six-second
             * fall-through ran on the wing too and the flight carried on with
             * nothing driving it.) */
            if (distance_m < 60.0f) {
                printf("sim: %6u ms  home again, %.0f m out - loitering\n",
                       virtual_ms, (double)distance_m);
                step = STEP_RECONNECT;
                step_since = virtual_ms;
            }
        } else if (elapsed > 6000u) {
            step = STEP_END;
            step_since = virtual_ms;
        }
        break;
    case STEP_RECONNECT:
        /* The receiver comes back, and the pilot asks for something the
         * navigator would never do: a roll to the left, away from the circle
         * it is holding. If the aircraft banks left, the pilot has it.
         *
         * Long enough first that the circle is a circle: the navigation's
         * loiter is only worth watching if it has been round once. */
        if (elapsed > 8000u) {
            printf("sim: %6u ms  receiver back, left roll - has the pilot got it?\n",
                   virtual_ms);
            rc_link_lost = 0;
            rc_pitch = 0.0f;
            rc_roll = -0.5f;
            reconnect_ms = virtual_ms;
            step = STEP_END;
            step_since = virtual_ms;
        }
        break;
    default:
        break;
    }
}

static void rc_refresh(void);
static void gps_send(void);
static void report_log_columns(void); /* reads the console's own log dump */
static int console_home_near(void);   /* with a noisy fix, within 5 m of home */

/*
 * The scenario's verdict.
 *
 * `make sil` is a demonstration; this is the part that can fail. Every check is
 * something the board contract can see for itself - the rate the loop turned
 * at, whether the bytes the radio sent came out of the parser, whether the
 * mixer's numbers moved the airframe, and whether losing the radio stopped it -
 * so a regression in the flight core, the parser or the wiring between them
 * exits non-zero instead of printing a transcript nobody reads.
 */
static int failures;

static void report(int ok, const char *what, const char *detail)
{
    printf("sim: %-4s %-42s %s\n", ok ? "ok" : "FAIL", what, detail);
    if (!ok) {
        failures++;
    }
}

static void sim_finish(void)
{
    char detail[96];

    if (sim_airframe == SIM_CONSOLE) {
        /* No scenario ran, so there is no scenario to judge: what this session
         * is worth is whatever the person - or the bench runner - at the
         * console made of it. */
        printf("\nsim: ---- console session, %u ms, %u passes ----\n",
               virtual_ms, n_sim_step);
        exit(0);
    }

    if (sim_airframe == SIM_FAULT) {
        /*
         * The boot after a crash, which is the only place a fault record is
         * ever read. Nothing flew, so none of the flying checks below apply -
         * this block is the whole judgement: the three lines, whole, because
         * the person reading them is going to copy the pc into `addr2line`,
         * and a line that prints the wrong field is a bench session spent
         * guessing.
         */
        printf("\nsim: ---- the boot after a crash, %u ms ----\n", virtual_ms);

        report(console_said("fault: 2 recorded, last pc 0x08001b42 "
                            "lr 0x08000a55 psr 0x61000000"),
               "the fault reaches the console with the count and the pc",
               "fault: 2 recorded, last pc 0x08001b42 lr 0x08000a55 "
               "psr 0x61000000");
        report(console_said("       cfsr 0x00000082 hfsr 0x40000000 "
                            "mmfar 0x00000000 bfar 0x00000000"),
               "and with the fault status registers",
               "the line the record's cfsr and hfsr are read from");
        report(console_said("       r0 0x11111111 r1 0x22222222 "
                            "r2 0x33333333 r3 0x44444444 r12 0x55555555"),
               "and the registers the faulting code was using",
               "the line the record's r0-r3 and r12 are read from");
        report(console_said("       frame 0x2001ffc0 (the stacked registers "
                            "are here)"),
               "and where those registers still are, for a stack walk",
               "the line that says where the frame was");
        report(!ak_fault_present(),
               "and the boot consumes it, so the next thing that asks is not "
               "told about the same crash twice",
               "ak_fault_present() is false after the boot's report");
        report(console_said("preflight: the machine is what the firmware "
                            "thinks it is"),
               "which the preflight, running after the report, agrees with",
               "the record was consumed before the preflight looked");

        printf("sim: %s (%d check%s failed)\n",
               failures == 0 ? "PASS" : "FAIL", failures,
               failures == 1 ? "" : "s");
        exit(failures == 0 ? 0 : 1);
    }

    if (sim_airframe == SIM_BADBOARD) {
        /*
         * The board that lies, judged by whether the *command* said the right
         * thing about each specific wrongness. This is the one session on this
         * machine that reaches the preflight's FAIL branches at all, so what
         * it protects is the line a person reads first when a fresh board does
         * not behave: each of these strings is a diagnosis, and a diagnosis
         * that cannot be produced is worse than none because the bench goes
         * looking for the fault somewhere else.
         */
        printf("\nsim: ---- the board that lies, %u ms ----\n", virtual_ms);

        if (sim_no_outputs) {
            /* A port whose timers never came up. What is pinned here is the
             * whole chain, because this is the one board fault that ends in an
             * armed aircraft rather than a refusal if any link of it is
             * missing: the preflight calls it a fact, does not then ask a
             * timer that is not running what rate it is running at, the boot
             * says there is a problem before anybody types anything, and the
             * mix-versus-board line and the arming gate both refuse with the
             * two numbers. The session lies about the rate as well, so the
             * absent line is a check too.
             *
             * (The preflight's other outputs branch, "the board has not said
             * what its outputs are", is unreachable while main.c states the
             * shape at boot. With the ready check above, a board that is not
             * ready states *zero* rather than nothing - which is the branch
             * that produces the numbers in the refusal below.) */
            report(console_said("--    outputs: none on this board"),
                   "a board whose outputs never came up says so",
                   "--    outputs: none on this board");
            report(!console_said("FAIL  outputs are at"),
                   "and the rate is not asked of a timer that is not running",
                   "the rate check was skipped, though the rate was lied about");
            report(console_said("preflight: 1 problem - type 'preflight'"),
                   "and the boot says there is a problem before anybody types",
                   "preflight: 1 problem - type 'preflight'");
            report(console_said("FAIL  this mix needs 4 motors and 0 servos; "
                                "the board drives 0 and 0"),
                   "and the mix is refused with both numbers in it",
                   "FAIL  this mix needs 4 motors and 0 servos; the board "
                   "drives 0 and 0");
            report(console_said("refused - this airframe's mix needs 4 motors "
                                "and 0 servos, and the board drives 0 and 0"),
                   "so the arming gate refuses too, which is the whole point",
                   "the arm line, with the numbers a person can act on");
        } else if (sim_two_motors) {
            /* Two motors and two servos - the ESP32-C3's shape - against a
             * quadrotor's mix. Both numbers are in the sentence, which is what
             * makes it a diagnosis rather than a refusal. */
            report(console_said("FAIL  this mix needs 4 motors and 0 servos; "
                                "the board drives 2 and 2"),
                   "a two-motor board is told the mix does not fit it",
                   "FAIL  this mix needs 4 motors and 0 servos; the board "
                   "drives 2 and 2");
            report(console_said("refused - this airframe's mix needs 4 motors "
                                "and 0 servos, and the board drives 2 and 2"),
                   "and the arming gate refuses on the same two numbers",
                   "the arm line, which is the sentence the pilot reads");
            report(console_count("preflight: SOMETHING IS WRONG") == 1,
                   "and that is the only thing wrong with it",
                   "one preflight was typed, so the other lines are absent");
        } else {
            report(console_said("FAIL  console is on 0x40004800, but the "
                                "board's console is 0x00000001"),
                   "a console on a port the board does not have is caught",
                   "FAIL  console is on 0x40004800, but the board's console "
                   "is 0x00000001");
            report(console_said("FAIL  the clock is not on its crystal: the "
                                "board fell back to the internal clock"),
                   "a crystal that never started is caught",
                   "FAIL  the clock is not on its crystal");
            report(console_said("FAIL  the saved configuration is damaged"),
                   "a configuration whose record will not read is caught",
                   "FAIL  the saved configuration is damaged (a bad length, "
                   "or a checksum mismatch)");
            report(console_said("FAIL  a fault is recorded: pc 0xdead0f00 "
                                "cfsr 0x00000082"),
                   "a crash from the run before is caught, with its own pc",
                   "FAIL  a fault is recorded: pc 0xdead0f00 cfsr 0x00000082");
            report(console_said("FAIL  outputs are at 302 kHz, 300 asked for"),
                   "timers running at the wrong rate are caught",
                   "FAIL  outputs are at 302 kHz, 300 asked for");
            report(console_said("FAIL  the millisecond tick is not running"),
                   "a dead millisecond tick is caught rather than hung on",
                   "FAIL  the millisecond tick is not running (0 delays have "
                   "given up waiting for it)");
            /* And the other half: with every lie taken back it passes, twice -
             * once before the first lie and once after the last. The boot's
             * own clean line is the third. */
            report(console_count("preflight: the machine is what the firmware "
                                 "thinks it is") == 3,
                   "and an honest board passes the same command",
                   "the clean line three times: at boot, and after each clean "
                   "run");
            report(console_count("preflight: SOMETHING IS WRONG") == 6,
                   "one wrong board fact, one refusal - and no more",
                   "six SOMETHING IS WRONG lines for six lies");
        }

        printf("sim: %s (%d check%s failed)\n",
               failures == 0 ? "PASS" : "FAIL", failures,
               failures == 1 ? "" : "s");
        exit(failures == 0 ? 0 : 1);
    }

    if (sim_airframe == SIM_GPS_QUIET) {
        /*
         * A module that never answers, judged by what the firmware did about
         * it: the four configuration frames it sends - and the fact that it
         * sends four and no more - what is *in* them, and that the aircraft
         * still knows it has no fix. The frame's content is the part worth
         * pinning: a configuration that asks a u-blox for the wrong rate is a
         * module that answers at the wrong rate, which reads as a fix that
         * lags the aircraft.
         */
        printf("\nsim: ---- a gps that never speaks, %u ms ----\n", virtual_ms);

        report(sim_gps_cfg_sends == 4u,
               "the module is asked to configure itself four times, and no more",
               "four VALSET frames on the wire, then it gives up");
        report(console_said("4 attempts"),
               "and the console's own report says so",
               "the `gps` command's configure line");

        /* The frame: UBX sync, class CFG (0x06), id VALSET (0x8A). */
        report(sim_gps_cfg_len > 10u && sim_gps_cfg[0] == 0xB5u &&
                   sim_gps_cfg[1] == 0x62u && sim_gps_cfg[2] == 0x06u &&
                   sim_gps_cfg[3] == 0x8Au,
               "and what it sends is a UBX configuration frame",
               "sync B5 62, class CFG 06, id VALSET 8A");

        /* Walk the key/value pairs and read the two that decide whether a real
         * module answers at all: the measurement rate, and NAV-PVT on the
         * port. */
        {
            unsigned at = 6u + 4u; /* sync, class, id, length, then the header */
            unsigned rate_ms = 0u;
            unsigned nav_pvt = 0u;

            while (at + 4u < sim_gps_cfg_len) {
                uint32_t key = (uint32_t)sim_gps_cfg[at] |
                               ((uint32_t)sim_gps_cfg[at + 1u] << 8) |
                               ((uint32_t)sim_gps_cfg[at + 2u] << 16) |
                               ((uint32_t)sim_gps_cfg[at + 3u] << 24);

                /* The two rate keys carry a 16-bit value and the message keys
                 * a single byte, which is a fact about the key's type rather
                 * than about this frame: a walk that assumed one size read
                 * every later key off by one byte, which is exactly the bug the
                 * check below would have missed if it only looked for the
                 * first key. */
                if (key == 0x30210001u || key == 0x30210002u) {
                    if (key == 0x30210001u) { /* CFG-RATE-MEAS */
                        rate_ms = (unsigned)sim_gps_cfg[at + 4u] |
                                  ((unsigned)sim_gps_cfg[at + 5u] << 8);
                    }
                    at += 6u;
                } else if (key == 0x20910007u) { /* MSGOUT-NAV-PVT-UART1 */
                    nav_pvt = sim_gps_cfg[at + 4u];
                    at += 5u;
                } else {
                    at += 5u; /* one-byte values: the messages switched off */
                }
            }

            report(rate_ms == 200u,
                   "asking for a measurement every 200 ms, which is 5 Hz",
                   "CFG-RATE-MEAS = 200");
            report(nav_pvt == 1u,
                   "and for NAV-PVT once per solution on the port",
                   "MSGOUT-NAV-PVT-UART1 = 1");
        }

        report(console_said("valid now: no"),
               "and the aircraft knows it has no fix",
               "the `gps` command's fix line");
        report(console_said("0 nav-pvt"),
               "with the parser having read nothing off the wire",
               "the preflight's gps line");

        printf("sim: %s (%d check%s failed)\n",
               failures == 0 ? "PASS" : "FAIL", failures,
               failures == 1 ? "" : "s");
        exit(failures == 0 ? 0 : 1);
    }

    if (sim_airframe == SIM_QUAD_RTH) {
        /*
         * The quadrotor's return, judged the way the wing's is: from where the
         * aircraft went and what the firmware said about it, not from what a
         * command was supposed to mean.
         */
        snprintf(detail, sizeof detail, "%.0f m out at its furthest",
                 (double)distance_max_m);
        /* Over forty metres in either air: the point of the check is that the
         * aircraft had somewhere to come back from, and the outbound leg is
         * flown rather than blown, so the calm run (which has 46 m of its own
         * airspeed and no wind behind it) gets there too. */
        report(distance_max_m > 40.0f, "the quadrotor flew away from home",
               detail);

        snprintf(detail, sizeof detail,
                 "the estimator's yaw was %.0f deg from the track it was "
                 "making when the link went",
                 (double)quadrth_engage_yaw_error_deg);
        report(quadrth_yaw_checked &&
                   ak_absf(quadrth_engage_yaw_error_deg) < 25.0f,
               "and its yaw estimate had found the ground track by then",
               detail);

        int engaged = console_said("return:    engaged");
        report(engaged, "the navigator says the return engaged",
               engaged ? "the console reports it engaged"
                       : "the return never engaged");

        /*
         * And, when the session gave the gyro a warmed-up bias, what the
         * heading did about it.
         *
         * The aircraft spends the end of this flight parked with its motors
         * off and nothing to align to - the GPS-track aid needs motion, and
         * there is none - so a bias the arm-time calibration did not remove
         * shows up as a heading that walks away while the aircraft is still.
         * The console's own `status` is asked, rather than the telemetry: the
         * receiver is unplugged for part of this flight, and a number the
         * firmware printed over the console is evidence that does not depend
         * on a radio link.
         */
        if (sim_gyro_bias_dps != 0.0f) {
            /* How far the firmware's heading walked between the two samples
             * taken while the aircraft was parked with its motors off: with
             * nothing moving and no track to align to, the only thing that
             * moves a gyro-integrated heading is the bias the part is carrying
             * that its calibration did not know about. */
            float error =
                ak_rad2deg(rc_tlm.yaw_rad - gyro_drift_yaw_a);

            while (error > 180.0f) {
                error -= 360.0f;
            }
            while (error < -180.0f) {
                error += 360.0f;
            }
            snprintf(detail, sizeof detail,
                     "%.0f dps of warmed-up bias; the heading walked %.0f deg "
                     "while parked", (double)sim_gyro_bias_dps,
                     (double)error);
            report(gyro_drift_sampled && rc_tlm.have_attitude &&
                       ak_absf(error) < 5.0f,
                   "and a gyro that warmed up since the bench does not walk "
                   "the heading away", detail);
        }

        snprintf(detail, sizeof detail,
                 "closest approach %.0f m of %.0f m out; %.0f m out when it "
                 "first reached the hover height",
                 (double)quadrth_closest_m, (double)quadrth_out_m,
                 (double)quadrth_arrived_m);
        report(quadrth_closest_m < 25.0f,
               "and brought it back over the ground it took off from", detail);

        snprintf(detail, sizeof detail,
                 "at %.1f m above home on arrival, %.1f m four seconds later",
                 (double)quadrth_hover_alt_m,
                 (double)quadrth_hover_alt_later_m);
        report(quadrth_hover_alt_m < 5.0f && quadrth_hover_alt_later_m > 1.0f &&
                   quadrth_hover_alt_later_m < 5.0f,
               "then descended to the hover height it was told to hold",
               detail);

        /* And *where* it came down, which is what the return's own arrival
         * radius buys: the descent has to start over the pad, not at the
         * wing-sized radius the return was brought home inside. */
        snprintf(detail, sizeof detail,
                 "the descent started %.1f m from home at %.1f m up",
                 (double)quadrth_descend_m, (double)quadrth_out_alt_m);
        report(quadrth_have_descend && quadrth_descend_m < 20.0f,
               "and it came down over the ground it took off from", detail);

        /* And stayed there in the wind, which is the difference between a
         * landing on the pad and a landing beside it. */
        snprintf(detail, sizeof detail,
                 "averaged %.1f m from home over %u samples of the settle",
                 quadrth_hold_samples > 0u
                     ? (double)(quadrth_hold_sum_m /
                                (float)quadrth_hold_samples)
                     : 0.0,
                 quadrth_hold_samples);
        report(quadrth_hold_samples > 20u &&
                   quadrth_hold_sum_m / (float)quadrth_hold_samples < 2.0f,
               "and it held its station over home while it settled", detail);

        report_log_columns();

        /*
         * And the version where the fix is *bad* rather than gone: the frames
         * keep arriving, with a 3D fix the receiver calls ok and four
         * satellites. The navigator must not fly that - and the flight must not
         * end either: it holds what it can still measure and picks the return
         * up when the fix is worth using again.
         */
        if (sim_badfix_on_return) {
            snprintf(detail, sizeof detail,
                     "lowest collective while the fix was bad: %.2f, of a "
                     "0.55 hover",
                     (double)badfix_min_collective);
            report(sim_gps_bad_ms > 0u && badfix_min_collective > 0.2f,
                   "a fix the navigator would not use did not end the flight",
                   detail);
            snprintf(detail, sizeof detail,
                     "%.1f to %.1f m up while it was bad, against the 22 m it "
                     "was holding",
                     (double)badfix_alt_low_m, (double)badfix_alt_high_m);
            report(sim_gps_bad_ms > 0u && badfix_alt_low_m > 15.0f &&
                       badfix_alt_high_m < 30.0f,
                   "and it held the altitude it could still measure", detail);
            int said_why = console_said("usable:    no - 4 satellites");
            report(said_why, "and the console said why it would not use it",
                   said_why ? "the console reports the fix as not usable, and "
                              "how many satellites the navigator wants"
                            : "the console never explained the fix");
            int preflight_said = console_said("return: enabled, but the fix is "
                                              "not usable");
            report(preflight_said,
                   "and the preflight told the pilot a return would not work",
                   preflight_said
                       ? "the checklist's fact says the return is enabled and "
                         "would not happen"
                       : "the preflight did not say the return was impossible");
        }

        /*
         * And the fix that *lies*: same type, same satellites, same accuracy
         * figures, position fifty metres out for two seconds. No gate can
         * refuse that one - the only defence is that the loop is closed - so
         * what is checked is the cost: the return pauses while it believes the
         * lie and picks itself up afterwards, and the aircraft still lands
         * where it took off.
         */
        if (sim_gps_jump_on_return) {
            snprintf(detail, sizeof detail,
                     "closest approach %.0f m, motors stopped %.0f m from home",
                     (double)quadrth_closest_m, (double)quadrth_motor_stop_m);
            report(sim_gps_jump_at_ms > 0u && quadrth_motors_stopped &&
                       quadrth_motor_stop_m < 10.0f,
                   "a fix that lied about where it was did not lose the return",
                   detail);
        }

        /*
         * And the run where the part the landing rule *prefers* is the one
         * that fails: the rangefinder stops answering as the descent starts
         * and comes back four seconds later. Three things are asked, and they
         * are the three a person would ask of it - was I told, once and not
         * once a millisecond; did the console's own report agree while it was
         * quiet; and did the aircraft still land, on the rule that does not
         * need the ground.
         */
        if (sim_range_fail_on_descent) {
            int said = console_count("rangefinder: no answer");
            int counted = console_said("reads in a row with no answer");
            int again = console_said("rangefinder: answering again after");

            snprintf(detail, sizeof detail,
                     "said %d time%s, report carried the count: %s, said it "
                     "came back: %s", said, said == 1 ? "" : "s",
                     counted ? "yes" : "no", again ? "yes" : "no");
            report(said == 1 && counted && again,
                   "a rangefinder that went quiet and came back is said once "
                   "each way, and counted while it was away", detail);

            snprintf(detail, sizeof detail,
                     "on the ground at %.1f m, motors stopped %.1f m up, %.0f "
                     "m from home", (double)plant_alt_m,
                     (double)quadrth_motor_stop_alt_m,
                     (double)quadrth_motor_stop_m);
            report(quadrth_motors_stopped && plant_alt_m < 0.5f &&
                       quadrth_motor_stop_alt_m < 1.5f,
                   "and the return still landed it, on the rule that does not "
                   "need the ground", detail);
        }

        /*
         * And the board a devkit out of the bag is - no barometer, no
         * rangefinder, no pack divider - which is the shape of every ESP32
         * this project has, and the reason every part of this firmware has a
         * fallback. What is asked is two things: that the machine says what it
         * is not, on the lines a person reads first, and that it *flies*
         * anyway - the return engages, comes home and lands on the gps's
         * altitude where the barometer would have been and on the barometer
         * where the rangefinder would have been.
         */
        if (sim_bare) {
            int imu = console_said("--    imu: icm42688p");
            int baro = console_said("--    baro: none fitted");
            int range = console_said("--    rangefinder: none fitted");
            int altitude = console_said("--    altitude: gps only");
            int battery = console_said("--    battery: none fitted on this "
                                       "board");
            /* And the two commands a person would type next, each of which has
             * its own sentence for a part that is not there. */
            int baro_cmd = console_said("baro:      none fitted - altitude "
                                        "comes from the gps alone");
            int battery_cmd = console_said("battery:   none fitted on this "
                                           "board");

            snprintf(detail, sizeof detail,
                     "imu %s, baro %s, rangefinder %s, altitude %s, battery "
                     "%s, and the two reports %s %s", imu ? "yes" : "no",
                     baro ? "yes" : "no", range ? "yes" : "no",
                     altitude ? "yes" : "no", battery ? "yes" : "no",
                     baro_cmd ? "yes" : "no", battery_cmd ? "yes" : "no");
            report(imu && baro && range && altitude && battery && baro_cmd &&
                       battery_cmd,
                   "a bare board's preflight names every part that is not on "
                   "it", detail);

            snprintf(detail, sizeof detail,
                     "on the ground at %.1f m, motors stopped %.1f m up, %.0f "
                     "m from home", (double)plant_alt_m,
                     (double)quadrth_motor_stop_alt_m,
                     (double)quadrth_motor_stop_m);
            report(quadrth_motors_stopped && plant_alt_m < 0.5f &&
                       quadrth_motor_stop_m < 15.0f,
                   "and the same board flew the return and landed on what it "
                   "has, which is the gps and the inertial sensor", detail);
        }

        /* The landing: the part the milestone's own words end with, and the
         * only place this firmware stops its own motors. */
        snprintf(detail, sizeof detail,
                 "on the ground at %.1f m, motors stopped at %.1f m",
                 (double)plant_alt_m, (double)quadrth_motor_stop_alt_m);
        snprintf(detail, sizeof detail,
                 "on the ground at %.1f m, motors stopped at %.1f m, %.0f m "
                 "from home",
                 (double)plant_alt_m, (double)quadrth_motor_stop_alt_m,
                 (double)quadrth_motor_stop_m);
        report(plant_alt_m < 0.5f && quadrth_motors_stopped &&
                   quadrth_motor_stop_alt_m < 1.0f &&
                   quadrth_motor_stop_m < 10.0f,
               "and it landed and stopped its motors there", detail);

        snprintf(detail, sizeof detail, "the later status says %s",
                 console_said("state:     disarmed") ? "disarmed"
                                                     : "something else");
        report(console_said("state:     disarmed"),
               "and stayed down - disarmed - when the receiver came back",
               detail);

        /* The mode the handset would have shown, from the telemetry the
         * firmware actually sent. */
        report((rc_tlm.modes_seen & (1u << 3)) != 0u,
               "and the flight mode it sent said RTH",
               rc_tlm.have_mode ? rc_tlm.mode : "no mode frame arrived");

        /*
         * And the question a handset asks before its display will show
         * anything at all: who is on the link. Two pings went up the wire
         * while the aircraft was climbing, and there have to be exactly two
         * answers - a flight controller that answered on every telemetry tick
         * would spend the link it is flying on saying the same thing again.
         */
        snprintf(detail, sizeof detail,
                 "%u ping%s up the wire, %u answer%s back, name \"%s\"",
                 crsf_pings_sent, crsf_pings_sent == 1u ? "" : "s",
                 rc_tlm.device_info_frames,
                 rc_tlm.device_info_frames == 1u ? "" : "s",
                 rc_tlm.info_name);
        report(crsf_pings_sent == 2u && rc_tlm.device_info_frames == 2u,
               "the handset asked who is on the link, and got one answer per "
               "ping", detail);

        snprintf(detail, sizeof detail,
                 "\"%s\", to 0x%02x from 0x%02x, %u parameters, version %u",
                 rc_tlm.info_name, rc_tlm.info_dest, rc_tlm.info_origin,
                 rc_tlm.info_parameters, rc_tlm.info_version);
        report(strcmp(rc_tlm.info_name, AK_PRODUCT_STR) == 0 &&
                   rc_tlm.info_dest == AK_CRSF_ADDRESS_RADIO_TRANSMITTER &&
                   rc_tlm.info_origin ==
                       AK_CRSF_ADDRESS_FLIGHT_CONTROLLER &&
                   rc_tlm.info_parameters == 0u &&
                   rc_tlm.info_version == 1u,
               "and the answer is the device-info frame a handset expects",
               detail);

    }

    if (sim_airframe == SIM_GPS_LOST) {
        /*
         * The measurement the firmware cannot do without, gone, with nobody
         * flying: the return must not turn into a motor stop. These read what
         * the plant did and what the firmware said, in that order.
         */
        int engaged = console_said("return:    engaged");
        int said_no_fix = console_said("valid now: no");

        report(engaged, "the return was flying when the gps went quiet",
               engaged ? "the console says the return was engaged"
                       : "the return had given up before the cut");
        report(gpslost_cut && said_no_fix,
               "and the console says the fix is gone",
               said_no_fix ? "the console reports no valid fix"
                           : "the console never said the fix was gone");
        report(console_said("with no fix, still holding"),
               "and that it is holding rather than navigating",
               console_said("with no fix, still holding")
                   ? "the console counts the steps it has been holding for"
                   : "the console never reported a hold");
        snprintf(detail, sizeof detail,
                 "lowest collective after the cut was %.2f, of hover 0.55",
                 (double)gpslost_min_collective);
        report(gpslost_cut && gpslost_min_collective > 0.2f,
               "but the motors kept turning", detail);
        snprintf(detail, sizeof detail,
                 "%.1f m up at the cut, %.1f m at its lowest after it, "
                 "%.1f m at the end",
                 (double)gpslost_alt_at_cut_m, (double)gpslost_min_alt_m,
                 (double)plant_alt_m);
        report(gpslost_cut && gpslost_min_alt_m > gpslost_alt_at_cut_m - 8.0f,
               "and it did not fall out of the sky", detail);
        snprintf(detail, sizeof detail,
                 "%.1f m up at the cut, %.1f m at the end",
                 (double)gpslost_alt_at_cut_m, (double)plant_alt_m);
        report(gpslost_cut &&
                   ak_absf(plant_alt_m - gpslost_alt_at_cut_m) < 8.0f,
               "and it was still holding its height at the end", detail);
        report((rc_tlm.modes_seen & (1u << 3)) != 0u,
               "and the handset was still being told it was returning",
               rc_tlm.have_mode ? rc_tlm.mode : "no mode frame arrived");
    }

    if (sim_airframe == SIM_GPS_LOST_LAND) {
        /*
         * The same loss of the only measurement a return can navigate on, with
         * a pilot who has said what to do about it: come down where you are,
         * with the rangefinder flying the last of it.
         *
         * What the checks are looking for is the difference between *landing*
         * and *falling*: the aircraft has to end up on the ground (it does not
         * know where home is any more), it has to get there at a descent rate
         * this firmware commands rather than one gravity chose, and the motors
         * have to stop because the ground was measured and not because a
         * timer ran out. The console line the navigator prints while it is
         * doing it is the evidence that it was the hold-descent and not
         * something else.
         */
        /*
         * And the negative control, which is the same session with the part
         * taken off the aircraft: the pilot's setting alone must not bring it
         * down. Without something measuring the ground the descent would be
         * flown on the estimate that has already failed, so the aircraft
         * holds - which is the behaviour this firmware had before any of this
         * existed, and the reason the rangefinder is a requirement rather than
         * a nicety.
         *
         * The two halves are branches rather than an early return, because
         * this function ends by *exiting* the simulator: a `return` here skips
         * that and the loop calls it again, which is a check printed forever
         * and a log file nobody wants to measure. (Learned the expensive way:
         * thirty gigabytes of it.)
         */
        if (!sim_range_fitted) {
            snprintf(detail, sizeof detail,
                     "%.1f m at the cut, %.1f m at the end, motors turning "
                     "%s", (double)gpslost_alt_at_cut_m, (double)plant_alt_m,
                     gpslost_land_stopped ? "no" : "yes");
            report(!gpslost_land_stopped && plant_alt_m > 10.0f,
                   "with no rangefinder it held instead of landing", detail);
            snprintf(detail, sizeof detail,
                     "the lowest collective after the fix went was %.2f, of a "
                     "0.55 hover", (double)gpslost_min_collective);
            report(gpslost_min_collective > 0.2f,
                   "and the motors kept turning", detail);
            report(console_said("still holding"),
                   "and the console says it is a hold, not a descent",
                   console_said("still holding")
                       ? "the status report names the hold"
                       : "the console never reported a hold");
        } else {
            snprintf(detail, sizeof detail,
                     "on the ground at %.2f m, motors stopped at %.2f m, %.0f "
                     "m from home", (double)plant_alt_m,
                     (double)gpslost_land_stop_alt_m,
                     (double)gpslost_land_stop_m);
            report(gpslost_land_stopped && plant_alt_m < 0.5f &&
                       gpslost_land_stop_alt_m < 1.0f,
                   "the aircraft came down and stopped on the ground", detail);
            snprintf(detail, sizeof detail,
                     "the lowest collective after the fix went was %.2f, of a "
                     "0.55 hover", (double)gpslost_land_min_collective);
            report(gpslost_land_min_collective > 0.2f,
                   "and it came down rather than falling", detail);
            snprintf(detail, sizeof detail,
                     "%.1f m at the cut, %.1f m at its lowest, %.1f m at the "
                     "end", (double)gpslost_alt_at_cut_m,
                     (double)gpslost_land_min_alt_m, (double)plant_alt_m);
            report(gpslost_cut && gpslost_land_min_alt_m > -1.0f,
                   "with nothing under it but wind and the ground it chose",
                   detail);
            report(console_said("coming down where it is"),
                   "and the console says it was the descent with no position",
                   console_said("coming down where it is")
                       ? "the status report names the manoeuvre"
                       : "the console never reported the descent");
            snprintf(detail, sizeof detail, "the last status says %s",
                     console_said("state:     disarmed") ? "disarmed"
                                                         : "something else");
            report(console_said("state:     disarmed"),
                   "and it finished the flight disarmed", detail);
            report(console_said("held:") && console_said("steps with no fix"),
                   "on the same counter the hold always used",
                   "the navigator counts the steps it has been holding for");
        }
    }

    if (sim_airframe == SIM_BATTERY) {
        /*
         * The pack went flat under a pilot who was still flying it, and came
         * back up again a few seconds later. What the checks want is the shape
         * of the answer: the aircraft took itself home, it said why, it did
         * not hand back a return because the voltage recovered, and it ended
         * on the ground.
         */
        int engaged = console_said("return:    engaged");
        int said_why = console_said("bringing it home");

        report(battery_stage >= 5,
               "the pack went below the critical threshold in flight",
               battery_stage >= 5 ? "the simulator sagged it to 3.25 V a cell"
                                  : "the pack never sagged");
        report(said_why, "and the console said the aircraft was taking itself home",
               said_why ? "the console says it is bringing it home"
                        : "nothing on the console mentioned the pack");
        report(engaged && battery_stage >= 7,
               "and the navigator was flying it with the link still up",
               engaged ? "the console reports the return engaged"
                       : "the return never engaged");
        snprintf(detail, sizeof detail,
                 "%.2f V at the end, %.1f m from home",
                 (double)sim_pack_v, (double)distance_from_home_m());
        report(sim_pack_recovered && console_said("state:     disarmed"),
               "and a pack that recovered did not cancel the return",
               sim_pack_recovered
                   ? "the pack was back to 12.00 V before it landed"
                   : "the pack never recovered");
        snprintf(detail, sizeof detail,
                 "on the ground at %.1f m, %.0f m from home",
                 (double)plant_alt_m, (double)distance_from_home_m());
        report(plant_alt_m < 0.5f && (rc_tlm.modes_seen & (1u << 3)) != 0u,
               "and it was brought home and landed", detail);
    }

    if (sim_airframe == SIM_RATE) {
        /*
         * Rate mode, against what the parameters ask for: a full stick is
         * `max_rate_dps` of rotation, `yaw_rate_dps` on the rudder axis, the
         * direction is the stick's, and centring the stick stops the rotation
         * - in either rotational direction, and without levelling the
         * aircraft, which is the other mode's job.
         */
        snprintf(detail, sizeof detail,
                 "%.0f deg/s asked for, %.0f reached, %.0f a second after "
                 "the stick was centred",
                 (double)SIM_RATE_MAX_DPS,
                 (double)rate_roll_peak_dps,
                 (double)rate_roll_after_centre_dps);
        report(rate_roll_peak_dps > SIM_RATE_MAX_DPS * 0.7f &&
                   rate_roll_peak_dps < SIM_RATE_MAX_DPS * 1.3f,
               "a full roll stick in rate mode gives the rate it asked for",
               detail);
        report(rate_roll_peak_dps > 0.0f && rate_pitch_peak_dps < 0.0f &&
                   rate_yaw_peak_dps > 0.0f,
               "and every stick turns the aircraft the way it is pushed",
               "right roll, nose-down pitch and right yaw all came out "
               "positive-when-asked");
        snprintf(detail, sizeof detail,
                 "%.0f deg of roll while the stick was held, %.0f deg/s a "
                 "second after centring",
                 (double)rate_roll_angle_after_centre_deg,
                 (double)rate_roll_after_centre_dps);
        report(ak_absf(rate_roll_after_centre_dps) < 60.0f,
               "and centring the stick stops the rotation", detail);
        report(rate_roll_angle_after_centre_deg > 90.0f,
               "and the aircraft stays where the stick left it", detail);
    }

    if (sim_airframe == SIM_TILT) {
        /*
         * An aircraft somebody is holding at an angle, with the arm switch
         * thrown: the gate that was missing. Three things are asked, and the
         * first two are two-sided on purpose - the console has to say *how
         * far* out the aircraft is and agree with the plant, and the outputs
         * have to stay at zero while it is held over - because "it did not
         * arm" alone is also what a firmware that can no longer arm at all
         * would say.
         */
        double said_deg = 0.0;
        int said = console_number("refused - the aircraft is", &said_deg);
        float said_tilt_deg = said ? (float)said_deg : 0.0f;
        float off_by = said_tilt_deg - tilt_roll_at_refusal_deg;

        snprintf(detail, sizeof detail,
                 "the console says %d degrees from level, the plant is at %d",
                 said ? (int)(said_deg + 0.5) : -1,
                 (int)(tilt_roll_at_refusal_deg + 0.5f));
        report(said && ak_absf(off_by) < 2.0f,
               "a tilted aircraft will not arm, and the console says why",
               detail);

        snprintf(detail, sizeof detail,
                 "the first output came at %u ms, and it was put level at "
                 "%u ms", motors_first_ms, (unsigned)TILT_LEVEL_MS);
        report(motors_first_ms > TILT_LEVEL_MS,
               "and nothing turned while it was held over", detail);

        snprintf(detail, sizeof detail, "asked after the switch, on the same "
                 "aircraft: %s",
                 console_said("state:     armed") ? "armed" : "still disarmed");
        report(console_said("state:     disarmed") &&
                   console_said("state:     armed") && motors_last_ms > 0u,
               "and the same switch arms it once it is level", detail);
    }

    if (sim_airframe == SIM_REFUSE) {
        /*
         * The bench's mistakes, judged by the sentences they produce. Each of
         * these is a diagnosis a person reads at a bench, and a diagnosis that
         * cannot be produced sends somebody looking for the fault somewhere
         * else - the reasoning the preflight's failure lines got the same
         * treatment for.
         *
         * The first check is the one that found a defect rather than a
         * missing line: four calibrations tried while the aircraft was armed,
         * and only three of them refusing.
         */
        if (!sim_rc_quiet) {
            int armed_refusals = console_count("refusing - the aircraft is "
                                               "not disarmed");
            snprintf(detail, sizeof detail,
                     "%d of the four calibrations refused while armed",
                     armed_refusals);
            report(armed_refusals == 4,
                   "every calibration refuses to run on an armed aircraft, the "
                   "accelerometer's included", detail);

            /*
             * And the command that is not a calibration but has the same rule
             * for the same reason: `save` writes flash, and this board's erase
             * stalls the loop for about a second. In the air that is not a slow
             * loop - the DShot frames stop and the ESCs time out - so it
             * refuses, in its own words, and the disarmed save later in the
             * session is what keeps this from passing on a firmware that simply
             * refused every save.
             */
            int armed_save = console_said("save: refusing - this writes flash, "
                                          "and an armed aircraft is not a bench");
            report(armed_save,
                   "and a save on an armed aircraft is refused, because the "
                   "write stalls the loop",
                   armed_save ? "the console says why, and writes nothing"
                              : "the save ran on an armed aircraft");
        }

        int pack = console_said("calibrate vbat: 99.00 V is not a pack this "
                                "aircraft flies");
        int nopack = console_said("the pin reads 0.0 mV - connect the pack");
        snprintf(detail, sizeof detail,
                 "a pack of 99 V: %s; a divider with nothing on it: %s",
                 pack ? "refused" : "accepted", nopack ? "said" : "not said");
        report(pack && nopack,
               "a pack that is not a pack, and a divider with nothing on it, "
               "are two different sentences", detail);

        int face = console_said("calibrate accel: '9' is not a face");
        report(face, "a face that is not a face says so, with the digit",
               face ? "the console names the face it was given"
                    : "no answer about the face");

        if (sim_rc_quiet) {
            int quiet = console_said("calibrate rc: no frames from the "
                                     "receiver");
            report(quiet,
                   "and a calibration with a receiver that has never spoken "
                   "says so", quiet ? "the console says there are no frames"
                                    : "the console said something else");
        }

        int moving = console_said("calibrate: failed with") &&
                     console_said("the aircraft is moving, or the sensor "
                                  "stopped answering");
        report(moving,
               "a gyro calibration somebody interrupts by picking the "
               "aircraft up refuses to write a bias",
               moving ? "the console says how many samples it had and how many "
                        "it rejected"
                      : "the console reported no failure");

        report(console_said("state:     disarmed"),
               "and none of it armed anything",
               console_said("state:     disarmed")
                   ? "the last status says disarmed"
                   : "the last status says something else");

        /*
         * And the last mistake of the session, which is the one that would
         * otherwise be invisible: a board whose flash refuses the record. Two
         * things have to line up - the console has to say the save did not
         * happen, and the parameters' changed count has to still be there
         * afterwards - because a firmware that said "saved", or marked the
         * table saved anyway, would leave an aircraft flying the settings it
         * had before with nobody told. Then the same command on an honest
         * board saves, which is what keeps the check from passing on a
         * firmware that simply refuses every save.
         */
        int refused = console_count("save: the board refused the write") == 1;
        int unsaved = console_said("1 changed since the last save");
        report(refused,
               "a board whose flash refuses the record does not get a save "
               "reported as one",
               refused ? "the console says the board refused the write"
                       : "the save was not refused in words");
        report(unsaved,
               "and the parameter that was set is still counted as unsaved",
               unsaved ? "params still says one change since the last save"
                       : "the changed count did not survive the failed save");
        int honest = console_said("saved ");
        report(honest,
               "while the same command on a board that takes the write does "
               "save it",
               honest ? "the second save wrote the record and said so"
                      : "the second save did not report a record written");

        printf("sim: %s (%d check%s failed)\n", failures == 0 ? "PASS" : "FAIL",
               failures, failures == 1 ? "" : "s");
        exit(failures == 0 ? 0 : 1);
    }

    if (sim_airframe == SIM_TEST_ARM) {
        /*
         * The bench sweep that met the arm switch. Both halves are two-sided,
         * because "the test stopped" is also what a test that never started
         * would say: the sweep has to have moved a motor first, and the
         * sentence that says why it stopped has to be the firmware's own.
         */
        int stopped = console_count("output test: stopped - the aircraft is "
                                    "not disarmed");

        snprintf(detail, sizeof detail,
                 "the sweep moved motor%s %u of 4, and stopping it was said "
                 "%d time%s",
                 output_test_motors == 0x0Fu ? "s" : "",
                 (unsigned)(output_test_motors == 0x0Fu
                                ? 4u
                                : (output_test_motors ? 1u : 0u)),
                 stopped, stopped == 1 ? "" : "s");
        report(output_test_motors != 0u && stopped == 1,
               "an output test that is running is stopped by the arm switch, "
               "and says so once", detail);

        /*
         * What is left after the stop is the *aircraft's* output, not the
         * test's: an armed quadrotor idles its four motors together (the
         * mixer's `motor_idle`, five per cent), where the sweep drives one
         * output at a time up to fifteen. So the check is the shape of it -
         * the largest of them is under what the test drives at all, and the
         * four are within a whisker of each other - rather than zero, which
         * would be asking an armed aircraft to keep its ESCs quiet.
         */
        float lowest = output_test_after_motor[0];
        float highest = output_test_after_motor[0];
        for (int i = 1; i < AK_MAX_MOTORS; i++) {
            if (output_test_after_motor[i] < lowest) {
                lowest = output_test_after_motor[i];
            }
            if (output_test_after_motor[i] > highest) {
                highest = output_test_after_motor[i];
            }
        }
        snprintf(detail, sizeof detail,
                 "after the stop the four motors read %.3f %.3f %.3f %.3f "
                 "(the sweep drives one to 0.150)",
                 (double)output_test_after_motor[0],
                 (double)output_test_after_motor[1],
                 (double)output_test_after_motor[2],
                 (double)output_test_after_motor[3]);
        report(output_stop_frame != 0u && highest < 0.12f &&
                   highest - lowest < 0.03f,
               "and it gave the outputs back rather than fighting the pilot",
               detail);

        report(console_said("state:     armed"),
               "and the aircraft is the thing that has them",
               console_said("state:     armed")
                   ? "the status typed after the stop says armed"
                   : "the status says something else");
    }

    if (sim_airframe == SIM_ARM_NOW) {
        /*
         * The aircraft that armed before the bias was measured. Two of these
         * are two-sided: the message has to be there *once*, and the normal
         * "bias measured while disarmed" line has to be absent - without that
         * second half, a session that quietly took the long way round (arming
         * after half a second) would pass the first and prove nothing at all.
         */
        int told = console_count("gyro: no bias measured before arming");
        int measured = console_said("gyro: bias measured while disarmed");

        /*
         * And the numbers out of the firmware's own sentence: how many
         * samples it managed to gather, and how many it threw away as moving.
         * The second is the proof that the hand was doing what this scenario
         * says it was - without it, a session that quietly held the aircraft
         * still would pass the check above and prove nothing.
         */
        unsigned samples = 0u, rejected = 0u;
        (void)console_pair("gyro: no bias measured before arming (", &samples,
                           &rejected);

        snprintf(detail, sizeof detail,
                 "told %d time%s: %u samples, %u rejected as moving, and the "
                 "measurement line %s", told, told == 1 ? "" : "s", samples,
                 rejected, measured ? "is there too" : "is not");
        report(told == 1 && !measured && rejected > 0u && samples < 500u,
               "an aircraft armed before the bias was measured is told once, "
               "and was not told the other thing", detail);

        report(console_said("state:     armed"),
               "and it did arm, which is what makes this a window rather "
               "than a refusal",
               console_said("state:     armed")
                   ? "the status the scenario typed says armed"
                   : "the status says something else");

        snprintf(detail, sizeof detail,
                 "armed at %u ms, motors up to %.2f, %.1f m above where it "
                 "was when it armed",
                 armnow_armed_ms, (double)motor_peak,
                 (double)(plant_alt_m - armnow_alt_at_arm_m));
        report(motor_peak > 0.05f &&
                   plant_alt_m - armnow_alt_at_arm_m > 2.0f,
               "and it flew on the bias it had, rather than refusing to go "
               "anywhere on a bias it did not", detail);
    }

    if (sim_airframe == SIM_NO_IMU) {
        /*
         * The gyro stops answering, in flight, with the sticks where the pilot
         * left them. What the flight core promises is narrow and absolute: no
         * attitude is not a state to fly in, so the motors stop - and the state
         * latches, because a failsafe that un-latches the moment the fault
         * clears is a failsafe that flies again on its own.
         */
        snprintf(detail, sizeof detail,
                 "%.1f m up at the failure, motors last turning %u ms later",
                 (double)noimu_alt_at_fail_m,
                 noimu_motors_last_ms > noimu_ms
                     ? noimu_motors_last_ms - noimu_ms
                     : 0u);
        report(noimu_ms > 0u && noimu_motors_last_ms >= noimu_ms &&
                   noimu_motors_last_ms - noimu_ms <= 100u,
               "the gyro going quiet in flight stopped the motors", detail);
        report(noimu_alt_at_fail_m > 5.0f,
               "and it was in the air when it happened",
               "the plant was climbing at the moment the part went quiet");
        int latched = console_said("state:     failsafe");
        report(latched, "and the state latched as a failsafe",
               latched ? "the console, asked after the gyro came back, still "
                         "says failsafe"
                       : "the console does not say failsafe");
    }

    distance_at_end_m = distance_from_home_m();
    alt_at_end_m = plant_alt_m;
    roll_at_end_deg = ak_rad2deg(plant_roll);

    printf("\nsim: ---- %u s of simulated flight ----\n", run_seconds);
    printf("sim: %u ms, %u passes of the main loop\n", virtual_ms, n_sim_step);

    if (sim_airframe == SIM_BENCH) {
        /*
         * First, where the boot got to - the one question a board with no
         * console has to answer, and the reason the marks exist. Every stage
         * is marked by the code that does the work: four in the board file,
         * nine here in main(). A stage that stopped being marked is a
         * tell-tale image that quietly blinks the wrong number, which is worse
         * than no tell-tale at all, so the boot is walked and read here rather
         * than only on the board that needs it.
         */
        snprintf(detail, sizeof detail, "the last stage marked was %d, %s",
                 ak_boot_stage_reached(),
                 ak_boot_stage_name(ak_boot_stage_reached()));
        report(ak_boot_stage_reached() == AK_BOOT_PREFLIGHT,
               "the boot reached all thirteen of the stages it has", detail);

        /* Every line below is the firmware's own, read back out of the
         * console: this says the six commands went through the CLI, the face
         * mapping, the arithmetic, the parameter table and the sample the
         * flight loop uses, not just the arithmetic in a test. */
        int measured = console_said("accel bias:");
        report(measured, "six faces produced a correction",
               measured ? "the console reports a bias and a scale"
                        : "no calibration came out of the six faces");

        static const char *bias_names[3] = { "accel_bias_x", "accel_bias_y",
                                             "accel_bias_z" };
        static const char *scale_names[3] = { "accel_scale_x", "accel_scale_y",
                                              "accel_scale_z" };
        int bias_ok = 1;
        int scale_ok = 1;
        double worst_bias = 0.0;
        double worst_scale = 0.0;

        for (int i = 0; i < 3; i++) {
            double value = 0.0;
            if (!console_number(bias_names[i], &value)) {
                bias_ok = 0;
                continue;
            }
            /* A count of an int16 at 2048 per g is half a milligram, and the
             * arithmetic on six of them is worth a couple more. */
            double error = value - (double)sim_part_bias[i];
            if (error < 0.0) {
                error = -error;
            }
            if (error > worst_bias) {
                worst_bias = error;
            }
            if (error > 0.002) {
                bias_ok = 0;
            }

            if (!console_number(scale_names[i], &value)) {
                scale_ok = 0;
                continue;
            }
            error = value - (double)sim_part_scale[i];
            if (error < 0.0) {
                error = -error;
            }
            if (error > worst_scale) {
                worst_scale = error;
            }
            if (error > 0.002) {
                scale_ok = 0;
            }
        }

        snprintf(detail, sizeof detail, "worst of the three is %.4f g",
                 worst_bias);
        report(bias_ok, "the bias it measured is the part's bias", detail);
        snprintf(detail, sizeof detail, "worst of the three is %.4f",
                worst_scale);
        report(scale_ok, "the scale it measured is the part's scale", detail);

        /* And the pack's divider, from a multimeter's number: the simulator's
         * pack is 12.0 V behind an 11:1 pair, and the calibration is told
         * 12.60 - so the ratio it writes has to be 12.60 over what the pin
         * actually reads, and the parameter has to come back as that number
         * rather than as the board file's default. */
        int vbat_said = console_said("calibrate vbat:");
        double ratio = 0.0;
        /* What the ratio should be: the number the multimeter was said to show,
         * over what the pin actually reads - which is the pack behind this
         * board's divider, 12.0 over 11. */
        double pin = (double)sim_pack_v / (double)SIM_PACK_DIVIDER;
        double want = 12.60 / pin;

        /* Read out of the calibration's *own* line rather than out of the
         * parameter dump: the dump has this parameter in it twice by now (once
         * before the calibration and once after), and a reader that takes the
         * first one is reading the value the calibration replaced. */
        report(vbat_said && console_number("ratio of", &ratio),
               "a multimeter reading calibrates the pack's divider",
               "");
        snprintf(detail, sizeof detail, "vbat_ratio %.3f against %.3f",
                 ratio, want);
        report(ratio > want - 0.05 && ratio < want + 0.05,
               "and the ratio it wrote is the pack over the pin", detail);

        /* And the correction reaches the sample the flight loop flies on, which
         * is the only place it matters. A per-mille either side is the sensor's
         * own resolution, not the arithmetic's. */
        int sample[3];
        int read = console_accel(sample);
        int level = read && sample[0] == 0 && sample[1] == 0 &&
                    sample[2] >= 998 && sample[2] <= 1001;
        if (read) {
            snprintf(detail, sizeof detail,
                     "the flight loop's own sample reads %d %d %d per-mille",
                     sample[0], sample[1], sample[2]);
        } else {
            snprintf(detail, sizeof detail, "no sample in the console");
        }
        report(level, "the corrected sensor reads level and nothing else",
               detail);

        /* And the long log is wired to a command: the one that survives a
         * reset has to be clearable, or a bench session starts with somebody
         * else's flight in it. */
        int cleared = console_said("long log cleared");
        report(cleared, "the long log answers to a command",
               cleared ? "the console cleared it"
                       : "nothing answered 'log long clear'");

        /* And the output test, which is how a person finds out which pad is
         * which motor with a scope instead of with props on. The firmware
         * drives; the simulator watches the same frames. */
        snprintf(detail, sizeof detail,
                 "motors 0x%x of 0x%x, servos 0x%x of 0x%x", output_test_motors,
                 0x0Fu, output_test_servos, 0x03u);
        report(output_test_ran && output_test_motors == 0x0Fu &&
                   output_test_servos == 0x03u,
               "the output test moved every motor and every servo", detail);

        snprintf(detail, sizeof detail, "largest output after the stop: %.3f",
                 (double)output_test_after_peak);
        report(output_test_after_peak == 0.0f,
               "and left everything at zero when it stopped", detail);

        printf("sim: %s (%d check%s failed)\n", failures == 0 ? "PASS" : "FAIL",
               failures, failures == 1 ? "" : "s");
        exit(failures == 0 ? 0 : 1);
    }

    if (sim_airframe == SIM_LAUNCH) {
        /*
         * A wing thrown by hand. The checks are the three things a launch
         * promises and the one it must not do: the numbers it says it will fly,
         * the aircraft actually leaving the ground on the attitude it was
         * given, the stick taking it back - and, in the middle of it, the
         * firmware's own account of who was flying.
         */

        /*
         * This run's ending is not the stick's, so it is judged on its own
         * before the checks that are about the stick are made.
         */
        /*
         * And the failsafe, which is a *reason* rather than a way out the pilot
         * chose: the same scenario with the pack going critical mid-launch.
         * Two things are asked, and the second is the one the code comment
         * above that block is about - the launch stops *and* the return that
         * the pack asked for is the thing flying the aircraft, rather than a
         * launch that outlived the reason to stop.
         */
        if (sim_launch_pack) {
            int over = console_said("launch: over - the pack");
            /* What the navigator's state is called depends on which reason
             * asked for the return: a lost link is `returning home`, a pack is
             * `on autopilot` - both are the navigator flying the aircraft. */
            int flying_it = console_said("state:     on autopilot") ||
                            console_said("state:     returning home");

            snprintf(detail, sizeof detail,
                     "the launch was over: %s, and the status after it says the "
                     "navigator has it: %s", over ? "yes" : "no",
                     flying_it ? "yes" : "no");
            report(over && flying_it,
                   "a pack that goes critical mid-launch stops the launch and "
                   "hands the aircraft to the return", detail);

            snprintf(detail, sizeof detail,
                     "%d m from home and %d m up when the status was typed",
                     (int)distance_from_home_m(), (int)plant_alt_m);
            report(distance_from_home_m() < 250.0f,
                   "and the return that the pack asked for is flying it home",
                   detail);

            printf("sim: %s (%d check%s failed)\n",
                   failures == 0 ? "PASS" : "FAIL", failures,
                   failures == 1 ? "" : "s");
            exit(failures == 0 ? 0 : 1);
        }


        int said = console_said("launch: 18 degrees of climb at 70 per cent");

        report(said,
               "the switch started a launch, and it said what it would fly",
               said ? "the console names the climb and the throttle"
                    : "the console never mentions a launch");

        {
            /* The motors, as the plant saw them: the launch's throttle and not
             * the pilot's, who has not touched the stick yet. */
            snprintf(detail, sizeof detail,
                     "the launch asks for %.2f and the largest frame the mixer "
                     "produced was %.2f", 0.70, (double)motor_peak);
            report(motor_peak > 0.65f && motor_peak < 0.80f,
                   "and the motors went to the launch throttle", detail);
        }

        {
            float wanted = 18.0f;

            snprintf(detail, sizeof detail,
                     "%.0f degrees of nose-up when the pilot took it, against "
                     "the %.0f it was given",
                     (double)launch_pitch_at_handover_deg, (double)wanted);
            report(ak_absf(launch_pitch_at_handover_deg - wanted) < 6.0f,
                   "and it flew the attitude it was given", detail);
        }

        snprintf(detail, sizeof detail, "%.0f m up when the pilot took it",
                 (double)launch_alt_at_handover_m);
        report(launch_alt_at_handover_m > 2.0f,
               "and it left the ground while the launch was flying it", detail);

        report(console_said("state:     on autopilot"),
               "the status while it was flying says the launch had it",
               console_said("state:     on autopilot")
                   ? "the mid-launch status says on autopilot"
                   : "the mid-launch status does not say on autopilot");

        int handed = console_said("launch: over - a stick");
        report(handed && console_said("state:     armed"),
               "and the stick gave it back to the pilot",
               handed ? "the console says the stick ended it, and the later "
                        "status says armed"
                      : "the console never says the stick ended the launch");

        /*
         * And the third way out, which no session had run: the switch that
         * started the launch going *down* under it. What is asked is two-sided
         * against the check above - two launches, matching the two different
         * endings - because "the switch ended it" is also true of a launch
         * that never started.
         */
        int by_switch = console_said("launch: over - the switch");
        int starts = console_count("degrees of climb at 70 per cent");
        snprintf(detail, sizeof detail,
                 "%d launches started, and the ending was the switch: %s",
                 starts, by_switch ? "yes" : "no");
        report(starts == 2 && by_switch && console_said("state:     armed"),
               "and the switch under a launch gives it back as well", detail);

        printf("sim: %s (%d check%s failed)\n", failures == 0 ? "PASS" : "FAIL",
               failures, failures == 1 ? "" : "s");
        exit(failures == 0 ? 0 : 1);
    }

    if (sim_airframe == SIM_FENCE) {
        int noticed = console_said("fence:");
        report(noticed, "the fence noticed the aircraft leaving",
               noticed ? "the console says the fence brought it back"
                       : "the console never mentions the fence");

        /* And the yaw axis, which is differential thrust on this airframe: how
         * hard the motors were asked to fight the turn. The yaw loop used to
         * hold a yaw *rate* of zero while the bank turned the aircraft, which
         * pegged one motor and idled the other for the whole return - and once
         * the plant modelled what differential thrust does, the aircraft never
         * came back at all. Coordinating the setpoint with the bank leaves the
         * authority where it belongs. */
        snprintf(detail, sizeof detail,
                 "the largest was %.1f deg/s of yaw, at %u ms, in %.0f m/s of "
                 "wind the coordination cannot see",
                 (double)differential_peak_dps, differential_peak_ms,
                 (double)sim_wind_e_m_s);
        report(differential_peak_dps < 10.0f,
               "the motors barely had to fight the turn", detail);

        snprintf(detail, sizeof detail, "closest approach %.0f m (fence %.0f m)",
                 (double)fence_min_after_m, (double)FENCE_RADIUS_M);
        report(fence_min_after_m > 0.0f && fence_min_after_m < FENCE_RADIUS_M,
               "and it brought it back inside", detail);

        int autopilot = console_said("state:     on autopilot");
        report(autopilot, "the flight core says the navigator was flying",
               autopilot ? "status says on autopilot"
                         : "status never says on autopilot");

        /* And gave it back: the last status, typed after the aircraft was
         * inside again, says the pilot has it. */
        int back = console_said("state:     armed");
        report(back, "and the pilot has it back now",
               back ? "the later status says armed"
                    : "the pilot never gets it back");

        snprintf(detail, sizeof detail,
                 "floor %.0f m above home, %.0f m up when it handed over",
                 (double)FENCE_FLOOR_ABOVE_HOME_M,
                 (double)fence_alt_at_handover_m);
        report(ak_absf(fence_alt_at_handover_m - FENCE_FLOOR_ABOVE_HOME_M) <
                   15.0f,
               "and it climbed to the floor it was given on the way", detail);

        snprintf(detail, sizeof detail,
                 "%.0f m from home when the pilot took it back (fence %.0f m)",
                 (double)fence_end_m, (double)FENCE_RADIUS_M);
        report(fence_end_m < FENCE_RADIUS_M,
               "and it was inside the ring when it handed over", detail);

        /*
         * And the ceiling, which is the same fence seen from below: the pilot
         * has the aircraft back and climbs through a lid set fifteen metres
         * above where it was flying. Two things are asked, and neither is the
         * other: the navigator took it (the console says so, with the height),
         * and the climb *stopped* - measured as the highest the plant got
         * after the breach against the ceiling, because a lid that is noticed
         * after the aircraft has gone another fifty metres up is a lid that
         * did not do anything.
         */
        int ceiling = console_said("m up, bringing it back");
        snprintf(detail, sizeof detail,
                 "ceiling %d m, highest afterwards %.0f m", fence_ceiling_m,
                 (double)fence_max_after_m);
        report(ceiling && fence_ceiling_m > 0 &&
                   fence_max_after_m < (float)fence_ceiling_m + 25.0f,
               "the ceiling took the aircraft off the pilot as well", detail);

        snprintf(detail, sizeof detail,
                 "%.0f m above home and %.0f m from home at the end, with the "
                 "ceiling at %d",
                 (double)plant_alt_m, (double)fence_ceiling_end_m,
                 fence_ceiling_m);
        report(fence_ceiling_m > 0 &&
                   plant_alt_m < (float)fence_ceiling_m + 5.0f &&
                   fence_ceiling_end_m < FENCE_RADIUS_M,
               "and it came back down under it", detail);

        /*
         * And what the console says when somebody asks the fence where the
         * aircraft is: the report a pilot reads in the air, both sides of it.
         * Two `gps` commands were typed - one above the lid and one back under
         * it - and the sentences have to be there, with the ring's own number
         * and the count of how many times the fence has taken the aircraft.
         */
        int above = console_said("m above home, over it");
        int below = console_said("m above home, under it");
        char ring[32];
        snprintf(ring, sizeof ring, "%d m, inside", (int)FENCE_RADIUS_M);
        int inside = console_said(ring);
        /* The count itself is not pinned: the ring took the aircraft twice on
         * the way out in this run (it was outside on two passes) and the lid
         * once, and that is a fact about the geometry rather than about the
         * firmware. What is checked is that the line is there, with a number
         * in it, once per report - which is what a person reads. */
        int triggers = console_count(" triggers") >= 2;
        snprintf(detail, sizeof detail,
                 "over the lid: %s, under it: %s, inside the ring: %s, "
                 "triggers counted: %s",
                 above ? "yes" : "no", below ? "yes" : "no",
                 inside ? "yes" : "no", triggers ? "yes" : "no");
        report(above && below && inside && triggers,
               "and the fence report says which side of the ring and of the "
               "lid the aircraft is on, and how many times it has fired",
               detail);

        printf("sim: %s (%d check%s failed)\n", failures == 0 ? "PASS" : "FAIL",
               failures, failures == 1 ? "" : "s");
        exit(failures == 0 ? 0 : 1);
    }

    if (sim_airframe == SIM_MISSION) {
        /* A mission, judged at the only place it can be: where the aircraft
         * went, and what the firmware said while it was going there. How close
         * "arriving" is, is the airframe's answer - see ak_nav_step - so the
         * threshold is too, with a few metres of slack for a discrete sample
         * of a turning aircraft. */
        float arrive_m = mission_airframe == 0 ? 15.0f : 60.0f;

        /*
         * And the run where the pack goes critical under the mission, which is
         * judged differently on purpose: the waypoints are *not* the answer
         * any more. The pilot asked for them; a dead pack is a reason to stop
         * flying them, and the aircraft has to come home instead.
         */
        if (sim_mission_flatpack) {
            float home_m = distance_from_home_m();

            /*
             * Two things at once, because either alone is weak: the console
             * says the mission is over, *and* the aircraft never went on to
             * the second waypoint. The console line on its own can be printed
             * after a mission that finished by itself - the switch goes off
             * and the mission is stopped - and the geometry on its own says
             * nothing about who ended it.
             *
             * The distances go in as integers: a `%f` here has no bounded
             * width, and gcc says so - the format truncation warning that
             * this replaces was the size of the buffer talking, not the
             * message.
             */
            snprintf(detail, sizeof detail,
                     "%s, and it came no closer than %d m to waypoint 2",
                     console_said("mission: stopped - the pack")
                         ? "the console stopped the mission"
                         : "the console said nothing",
                     (int)mission_min_m[1]);
            report(console_said("mission: stopped - the pack") &&
                       mission_min_m[1] > 50.0f,
                   "the pack took the aircraft off the mission",
                   detail);
            report(console_said("bringing it home"),
                   "and the console says why",
                   console_said("battery:") && console_said("bringing it home")
                       ? "the battery report is in the transcript"
                       : "no battery return was reported");
            /*
             * And the half that is a mode rather than a trigger: the switch is
             * still on, it is on for the next seventy seconds of this run, and
             * the mission must not come back. The navigator's own count is the
             * evidence - a restart would be a second `started` and a second
             * `taken back` on the report it types under the still-on switch.
             */
            report(console_said("1 started, 1 taken back"),
                   "and the switch still on did not restart it",
                   console_said("1 started, 1 taken back")
                       ? "the mid-flight report counts one mission, taken back"
                       : "the mission was started or taken back more than once");
            snprintf(detail, sizeof detail,
                     "%d m from home at the end, on the ground at %d m, %s",
                     (int)home_m, (int)plant_alt_m,
                     console_said("state:     disarmed") ? "disarmed"
                                                         : "still armed");
            report(home_m < 30.0f && plant_alt_m < 0.5f &&
                       console_said("state:     disarmed"),
                   "and the return brought it home and landed it", detail);

            printf("sim: %s (%d check%s failed)\n",
                   failures == 0 ? "PASS" : "FAIL", failures,
                   failures == 1 ? "" : "s");
            exit(failures == 0 ? 0 : 1);
        }

        /*
         * And the run where the *pilot* ends the mission: a stick, with the
         * switch still on. Two things are asked and the second is the one no
         * session had run - the mission must not come back by itself, because
         * the switch has been on the whole time and the pilot has already said
         * no once.
         */
        if (sim_mission_takeback) {
            int taken = console_said("1 started, 1 taken back");
            int armed = console_said("state:     armed");

            snprintf(detail, sizeof detail,
                     "the report says one mission taken back: %s, and the "
                     "status after it says armed: %s",
                     taken ? "yes" : "no", armed ? "yes" : "no");
            report(taken && armed,
                   "a stick took the mission back and gave the aircraft to "
                   "the pilot", detail);
            /* And the switch that was on through all of it did not start it
             * again: the same report, typed four seconds later, still counts
             * one. */
            report(console_count("1 started, 1 taken back") >= 2 &&
                       !console_said("2 started"),
                   "and the switch still on did not start it again",
                   console_said("2 started")
                       ? "the mission restarted under a switch that was never cycled"
                       : "both reports count one mission, taken back once");

            printf("sim: %s (%d check%s failed)\n",
                   failures == 0 ? "PASS" : "FAIL", failures,
                   failures == 1 ? "" : "s");
            exit(failures == 0 ? 0 : 1);
        }

        snprintf(detail, sizeof detail,
                 "closest approach %.0f m (arrive %.0f m)",
                 (double)mission_min_m[0], (double)arrive_m);
        report(mission_min_m[0] < arrive_m + 5.0f,
               "the mission reached its first waypoint", detail);

        snprintf(detail, sizeof detail, "closest approach %.0f m",
                 (double)mission_min_m[1]);
        report(mission_min_m[1] < arrive_m + 5.0f, "and then the second", detail);

        int counted = console_said("2 reached");
        /* And never three. The check used to be "does the console say two",
         * which a counter that goes 2 then 3 passes on the way past - and the
         * wing's own recorded transcript said exactly that, three arrivals for
         * two waypoints, while this check passed. */
        int overcounted = console_said("3 reached");
        report(counted && !overcounted, "the navigator counted both arrivals",
               overcounted ? "the console says three arrivals for two waypoints"
               : counted   ? "the console says two reached"
                           : "the console does not say two were reached");

        int on_mission = console_said("state:     on autopilot");
        report(on_mission, "the flight core says the navigator is flying",
               on_mission ? "status says on autopilot"
                          : "status never says on autopilot");

        if (sim_mission_hold) {
            /*
             * The long run: the mission has been on for the whole session, so
             * what is checked is where the aircraft *is* at the end of it
             * rather than what it did on the way. That is the drift question
             * the short sessions cannot ask - a position loop with a slow walk
             * in it, or an altitude that loses a metre a minute, passes every
             * check a seventy-five-second run has.
             *
             * The bounds are the ones "holding" already means here: inside the
             * arrival radius the navigator itself uses (fifteen metres for the
             * quadrotor, sixty for the wing), and the same twenty-five metres
             * of altitude the short mission allows.
             */
            float held_m = mission_distance_m(1);
            snprintf(detail, sizeof detail,
                     "%d m from the last waypoint after %u s, %s",
                     (int)held_m, virtual_ms / 1000u,
                     mission_airframe == 0 ? "still holding station there"
                                           : "still circling it");
            report(held_m < arrive_m,
                   "it is still at the last waypoint minutes later", detail);

            snprintf(detail, sizeof detail,
                     "started at %.0f m, still at %.0f m",
                     (double)mission_start_alt_m, (double)plant_alt_m);
            report(ak_absf(plant_alt_m - mission_start_alt_m) < 25.0f,
                   "and at the altitude it has held all along", detail);

            report(console_said("state:     on autopilot"),
                   "with the mission still flying it",
                   "the status says on autopilot");
        } else {
            /* And the switch that started it stopped it: the last status,
             * typed after the switch went off, says the pilot has the
             * aircraft. */
            int released = console_said("state:     armed");
            report(released, "and the switch that started it ends it",
                   released ? "the later status says armed"
                            : "the switch does not give it back");

            snprintf(detail, sizeof detail,
                     "%.0f m from the last waypoint, still %s",
                     (double)mission_end_m[1],
                     mission_airframe == 0 ? "holding station there"
                                           : "circling");
            report(mission_end_m[1] < 150.0f,
                   "and it stayed at the last one instead of flying on",
                   detail);

            snprintf(detail, sizeof detail,
                     "started at %.0f m, ended at %.0f m",
                     (double)mission_start_alt_m, (double)mission_end_alt_m);
            report(ak_absf(mission_end_alt_m - mission_start_alt_m) < 25.0f,
                   "holding the altitude it had when the mission started",
                   detail);
        }

        printf("sim: %s (%d check%s failed)\n", failures == 0 ? "PASS" : "FAIL",
               failures, failures == 1 ? "" : "s");
        exit(failures == 0 ? 0 : 1);
    }

    /* AK_FLIGHT_LOOP_MS is 1, so a pass that takes appreciably longer than a
     * millisecond of simulated time means the clock and the loop disagree -
     * which is exactly the bug that stopped the aircraft arming. */
    float ms_per_pass = (float)virtual_ms / (float)n_sim_step;
    snprintf(detail, sizeof detail, "%.4f ms per pass over %u passes",
             (double)ms_per_pass, n_sim_step);
    report(ms_per_pass > 0.95f && ms_per_pass < 1.05f,
           "one pass of the loop is one millisecond", detail);

    snprintf(detail, sizeof detail, "%u of %u bytes taken, ring dropped %u",
             rc_bytes_taken, rc_bytes_offered, ak_board_rc_dropped());
    report(rc_bytes_offered > 0u && rc_bytes_taken == rc_bytes_offered &&
               ak_board_rc_dropped() == 0u,
           "every radio byte reached the parser", detail);

    snprintf(detail, sizeof detail, "%u frames of %u bytes each",
             rc_frames, (unsigned)(rc_bytes_offered / (rc_frames ? rc_frames : 1u)));
    report(rc_frames >= 2u * run_seconds, "the radio sent frames throughout",
           detail);

    /*
     * The flight pack, end to end and inside the loop that flies the aircraft:
     * a pin voltage, the board's divider, the core's cell count and thresholds,
     * and the console's own words. The quadrotor's pack is pulled from 12.0 to
     * 9.75 volts at ten and a half seconds, so both answers are on the tape -
     * the same pack before and after.
     */
    if (sim_airframe == SIM_QUAD) {
        int counted = console_said("3S,");
        int healthy = console_said("V a cell - ok");

        snprintf(detail, sizeof detail, "12.00 V reads as %s and %s", 
                 counted ? "three cells" : "not three cells",
                 healthy ? "ok" : "not ok");
        report(counted && healthy,
               "the pack it is holding is the pack it reports", detail);

        int sagged = console_said("V a cell - critical");
        snprintf(detail, sizeof detail,
                 "9.75 V, three cells, 3.25 V a cell: reported %s",
                 sagged ? "critical" : "as something else");
        report(sagged, "and says so when the same pack sags", detail);
    }

    snprintf(detail, sizeof detail, "%u gps frames, %u bytes dropped",
             gps_frames_sent, ak_board_gps_dropped());
    report(gps_frames_sent > 0u && ak_board_gps_dropped() == 0u,
           "the gps stream reached the parser", detail);

    /* The position the simulated module broadcast, as the firmware's own boot
     * line reports where home is. This is the check that catches a NAV-PVT
     * field being read from the wrong offset, which is a bug this repository
     * had: the frame decodes either way, and only the numbers show it. */
    char home[48];
    snprintf(home, sizeof home, "%d.%07d, %d.%07d", SIM_HOME_LAT_E7 / 10000000,
             SIM_HOME_LAT_E7 % 10000000, SIM_HOME_LON_E7 / 10000000,
             SIM_HOME_LON_E7 % 10000000);
    int said_home = console_said(home);

    /*
     * With `noisy` the module's first fix is a metre or two from where the
     * aircraft is standing, so the exact string is not there to find - and the
     * question the check is really asking ("is the position the firmware
     * decoded the position the module broadcast?") is answered by a tolerance
     * four times the noise instead. */
    snprintf(detail, sizeof detail, "looking for \"%s\" in the console: %s", home,
             said_home ? "found" : "not found");
    if (sim_noisy && !said_home) {
        said_home = console_home_near();
        if (said_home) {
            snprintf(detail, sizeof detail,
                     "the fix is noisy, and the home it captured is within "
                     "five metres of where it was standing");
        }
    }
    report(said_home,
           "the position the gps sent is the position the firmware has", detail);

    snprintf(detail, sizeof detail, "motors turning %u..%u ms, up to %.2f",
             motors_first_ms, motors_last_ms, (double)motor_peak);
    /* Not for the session whose aircraft is on the bench with its outputs
     * being swept: it arms, and the point of it is that it never takes off. */
    if (sim_airframe != SIM_TEST_ARM) {
        report(motor_peak > 0.2f && motors_last_ms > motors_first_ms,
               "the aircraft armed and the mixer drove the motors", detail);
    }

    if (sim_wing_lost) {
        /*
         * The owner's rule for a wing whose pilot is gone and which has no
         * navigator: land as it circles down. Not the quadrotor's answer -
         * stop - because a wing that stops flying has no control at all.
         *
         * Two things are watched, and the second is what makes this a landing
         * rather than a fall: that it comes down, and that it is still being
         * *flown* on the way there - a bank it holds, a nose a little down, and
         * the motors turning, because that is what keeps air over the elevons.
         * A wing that arrived with its servos centred and its motors stopped
         * would be somewhere in the field with nothing having flown it.
         */
        report(console_said("state:     circling down"),
               "the firmware says what it is doing: circling down",
               "the status the console printed after the receiver came out");

        snprintf(detail, sizeof detail,
                 "%.0f m down from the %.0f m it was at when the receiver came out",
                 (double)(alt_at_link_loss_m - plant_alt_m),
                 (double)alt_at_link_loss_m);
        report(alt_at_link_loss_m - plant_alt_m > 10.0f,
               "and the lost wing came down", detail);

        snprintf(detail, sizeof detail, "bank %.0f deg, pitch %.0f deg",
                 (double)ak_rad2deg(plant_roll),
                 (double)ak_rad2deg(plant_pitch));
        report(fabsf(ak_rad2deg(plant_roll)) > 15.0f &&
                   ak_rad2deg(plant_pitch) < -2.0f,
               "holding the bank and the nose-down it was told to hold", detail);

        snprintf(detail, sizeof detail, "motors %.2f %.2f",
                 (double)plant_outputs.motor[0],
                 (double)plant_outputs.motor[1]);
        report(plant_outputs.motor[0] + plant_outputs.motor[1] > 0.05f,
               "with the motors still turning, which is what keeps the "
               "elevons flying", detail);
    }

    if (sim_airframe == SIM_WING) {
        /* The wing is the airframe with somewhere to go: the pilot flies it
         * out under throttle, the receiver comes out, and the navigator has to
         * turn it round and bring it back. Nothing else in this repository
         * closes that loop. */
        snprintf(detail, sizeof detail, "%.0f m out at its furthest",
                 (double)distance_max_m);
        report(distance_max_m > 80.0f,
               "the wing flew away from the take-off point", detail);

        snprintf(detail, sizeof detail, "closest approach %.0f m (arrive 60 m)",
                 (double)distance_min_after_loss_m);
        if (!sim_wing_lost) { /* the return, not the descent */
            report(distance_min_after_loss_m > 0.0f &&
                       distance_min_after_loss_m < 60.0f,
                   "return-to-home brought it back inside the arrival radius",
                   detail);
        }

        /* And it did that in wind, which is a different aircraft to fly: the
         * nose and the track it is making are up to a crab angle apart, the
         * steering is done on the track because that is what a GPS reports,
         * and the whole scenario was still air before this. */
        snprintf(detail, sizeof detail,
                 "the nose was up to %.0f deg off the track it was making, "
                 "in %.0f m/s of wind out of the west",
                 (double)crab_peak_deg, (double)sim_wind_e_m_s);
        if (!sim_wing_lost) { /* the return, not the descent */
            report(crab_peak_deg > 10.0f,
                   "and it flew that home in wind, nose off its own track", detail);
        }

        snprintf(detail, sizeof detail,
                 "circled between %.0f m and %.0f m before the pilot took over",
                 (double)distance_min_after_loss_m, (double)loiter_max_m);
        if (!sim_wing_lost) { /* the return, not the descent */
            report(loiter_max_m < 120.0f,
                   "and it circled there instead of wandering off", detail);
        }

        snprintf(detail, sizeof detail,
                 "held %.0f m, ended at %.0f m (limit 25 m)",
                 (double)alt_at_link_loss_m, (double)alt_at_end_m);
        if (!sim_wing_lost) { /* the return, not the descent */
            report(ak_absf(alt_at_end_m - alt_at_link_loss_m) < 25.0f,
                   "the altitude it captured is the altitude it kept", detail);
        }

        snprintf(detail, sizeof detail, "elevons moved to %.2f of full travel",
                 (double)servo_peak);
        report(servo_peak > 0.1f, "the mixer drove the elevons", detail);

        /* The firmware's own report of the return, not the simulator's. The
         * navigator had never engaged at all - it read the link a step behind
         * the flight core and only ever saw it go at the moment the core
         * latched the failsafe - and this is the line that says so. */
        int said_engaged = console_said("return:    engaged");
        if (!sim_wing_lost) { /* the return, not the descent */
            report(said_engaged, "the navigator says it took over",
                   said_engaged ? "the console reports the return engaged"
                                : "the console never reports it engaged");
        }

        /* And gives it back. The navigator flies a right-hand circle; the pilot
         * asks for a left roll, which no part of the return would ever do. The
         * aircraft banking left is the pilot having control, and nothing else
         * in this repository checks that. */
        snprintf(detail, sizeof detail,
                 "asked for a left roll, %.1f deg three seconds later",
                 (double)roll_after_reconnect_deg);
        if (!sim_wing_lost) { /* the return, not the descent */
            report(roll_after_reconnect_deg < -5.0f,
                   "the pilot got it back when the receiver did", detail);
        }

        /* And the barometer, which is the one sensor that measures height
         * directly: what it said, against what the airframe was doing when it
         * said it. */
        snprintf(detail, sizeof detail,
                 "the barometer says %.1f m, the airframe is at %.1f m",
                 (double)baro_reported_m, (double)baro_truth_m);
        if (!sim_wing_lost) { /* the return, not the descent */
            report(baro_reported &&
                       ak_absf(baro_reported_m - baro_truth_m) < 2.0f,
                   "the barometer reads the height the aircraft is flying at",
                   detail);
        }

        /* And the height the navigator flies on, which is the barometer with
         * the gps's absolute reference leaked into it - the whole chain from a
         * standard atmosphere to a control loop's altitude input. */
        snprintf(detail, sizeof detail,
                 "the fused height is %.1f m, the airframe is at %.1f m",
                 (double)fused_reported_m, (double)baro_truth_m);
        report(ak_absf(fused_reported_m - baro_truth_m) < 2.0f,
               "and the height the navigator flies on is that height", detail);

        /*
         * And what the handset was told. The frames are decoded on their way
         * out of the firmware (ak_board_rc_send above), so every check here is
         * the receiver's and the handset's view of the flight: the pack the
         * airframe is holding, the attitude it has, the position the gps is
         * reporting, and which mode the flight core was in. A telemetry that
         * sent plausible nonsense would pass a "bytes went out" check and fail
         * every one of these.
         */
        report(rc_tlm.frames > 0u && rc_tlm.crc_errors == 0u,
               "the handset's telemetry arrived intact",
               rc_tlm.frames ? "every frame passed the crc the receiver checks"
                             : "no telemetry frames arrived");

        snprintf(detail, sizeof detail,
                 "the handset reads %.2f V, the pack is %.2f V",
                 (double)rc_tlm.volts, (double)sim_pack_v);
        report(rc_tlm.have_battery && rc_tlm.battery_frames > 0u &&
                   ak_absf(rc_tlm.volts - sim_pack_v) < 0.2f,
               "and the battery frame carries the pack it is flying",
               detail);

        snprintf(detail, sizeof detail,
                 "the handset reads %.1f deg, the airframe is at %.1f deg",
                 (double)ak_rad2deg(rc_tlm.roll_rad),
                 (double)ak_rad2deg(plant_roll));
        report(rc_tlm.have_attitude &&
                   ak_absf(rc_tlm.roll_rad - plant_roll) < 0.15f,
               "and the attitude frame carries the attitude it has", detail);

        {
            /* The fix the module is sending, as the simulator built it: the
             * same arithmetic gps_send() uses, so this is a comparison against
             * the source of the fix rather than against itself. */
            int32_t want_lat = SIM_HOME_LAT_E7 +
                               (int32_t)(plant_north_m / SIM_M_PER_DEG_LAT *
                                         1e7f);
            int32_t want_lon = SIM_HOME_LON_E7 +
                               (int32_t)(plant_east_m / sim_m_per_deg_lon() *
                                         1e7f);
            /* A degree of latitude is 111 km, so 3000e-7 deg is about 33 m -
             * wide enough for the two frames a second the fix arrives at, and
             * narrow enough that a wrong position fails. */
            int close = rc_tlm.have_fix && rc_tlm.gps_frames > 0u &&
                        (rc_tlm.lat_e7 - want_lat < 3000) &&
                        (want_lat - rc_tlm.lat_e7 < 3000) &&
                        (rc_tlm.lon_e7 - want_lon < 3000) &&
                        (want_lon - rc_tlm.lon_e7 < 3000);

            snprintf(detail, sizeof detail,
                     "the handset reads %d.%07d, %d.%07d, %u satellites",
                     (int)(rc_tlm.lat_e7 / 10000000),
                     (int)(rc_tlm.lat_e7 % 10000000),
                     (int)(rc_tlm.lon_e7 / 10000000),
                     (int)(rc_tlm.lon_e7 % 10000000),
                     rc_tlm.satellites);
            report(close && rc_tlm.satellites == 11u,
                   "and the gps frame carries the fix the module sent", detail);
        }

        /* The mode the handset shows: angle mode while the pilot is flying it,
         * and the return once the link goes. Both are things a pilot looks at,
         * and neither is in any other check. */
        if (!sim_wing_lost) { /* the return, not the descent */
            report((rc_tlm.modes_seen & (1u << 1)) != 0u &&
                       (rc_tlm.modes_seen & (1u << 3)) != 0u,
                   "and the flight mode said ANGLE and then RTH",
                   rc_tlm.have_mode ? rc_tlm.mode
                                    : "no flight mode frame arrived");
        }
    } else if (sim_airframe != SIM_QUAD_RTH &&
               sim_airframe != SIM_GPS_LOST &&
               sim_airframe != SIM_GPS_LOST_LAND &&
               sim_airframe != SIM_BATTERY &&
               sim_airframe != SIM_RATE &&
               sim_airframe != SIM_TILT &&
               sim_airframe != SIM_NO_IMU &&
               sim_airframe != SIM_ARM_NOW &&
               sim_airframe != SIM_TEST_ARM) {
        /* The stick-and-motor checks belong to the quad's own session; the
         * return scenarios judge themselves with what the aircraft did - and
         * the gps-loss one never commands a stick after the link goes, the
         * battery one is a return, and the rate one is the *other* flight law
         * and has its own checks about what the sticks did. */
        snprintf(detail, sizeof detail, "commanded %.1f deg, reached %.1f deg",
                 (double)(SIM_ROLL_STICK * SIM_MAX_TILT_DEG),
                 (double)roll_at_test_deg);
        report(ak_absf(roll_at_test_deg -
                       SIM_ROLL_STICK * SIM_MAX_TILT_DEG) < 2.0f,
               "the angle loop reached the commanded roll", detail);

        /* And the pitch axis, which is the one this firmware had inverted: the
         * stick forward (negative) has to put the nose down, which is a
         * *negative* pitch in this aircraft's own units. The simulator's plant
         * models the thrust of a quad now - rear motors up, nose down - so a
         * mixer row with the wrong sign diverges here in a second rather than
         * on the first flight. */
        snprintf(detail, sizeof detail,
                 "commanded %.1f deg nose down, reached %.1f deg",
                 (double)(-SIM_ROLL_STICK * SIM_MAX_TILT_DEG),
                 (double)pitch_at_test_deg);
        report(ak_absf(pitch_at_test_deg +
                       SIM_ROLL_STICK * SIM_MAX_TILT_DEG) < 2.0f,
               "and the nose went down when the stick went forward", detail);

        /* And yaw, which is the axis no simulation can settle on its own:
         * which diagonal of a quad spins which way is chosen by the props and
         * the ESCs. The plant assumes the layout every flight controller
         * assumes - front-right and rear-left counter-clockwise - so this check
         * is "the firmware's yaw column agrees with the usual props", and the
         * bench has the last word. */
        snprintf(detail, sizeof detail,
                 "%.1f deg of nose-right for %.0f deg a second asked",
                 (double)yaw_turned_deg, (double)(SIM_YAW_STICK * 350.0f));
        report(yaw_turned_deg > 10.0f && yaw_turned_deg < 200.0f,
               "yaw right turned the nose right, the usual props assumed",
               detail);

        /* And `output test` refuses to drive the pins of an armed aircraft -
         * the one rule that makes it safe to have a command that writes the
         * outputs without the flight core. */
        int refused = console_said("output test: refusing");
        report(refused, "the output test refuses while armed",
               refused ? "the console says it refused"
                       : "the output test did not refuse");

        if (link_lost_ms == 0u) {
            snprintf(detail, sizeof detail, "the link was never lost");
        } else {
            snprintf(detail, sizeof detail,
                     "%u ms after the link went (limit 400)",
                     motors_last_ms - link_lost_ms);
        }
        report(link_lost_ms > 0u && motors_last_ms >= link_lost_ms &&
                   motors_last_ms - link_lost_ms <= 400u,
               "losing the radio stopped the motors", detail);

        /*
         * And the version of that where the receiver is *alive*: the
         * transmitter went off, the receiver went on sending, and its frames
         * carry the last stick positions - a hovering throttle, centred
         * controls - with only its own failsafe flag to say anything is wrong.
         * A firmware that read those channels as a command would fly the
         * aircraft on, which is exactly what the flag is for.
         */
        if (sim_rx_failsafe_on_link_loss) {
            snprintf(detail, sizeof detail,
                     "%u frames carrying throttle %.2f after the transmitter "
                     "went off, motors stopped %u ms later",
                     (unsigned)(rc_frames - rc_frames_at_failsafe),
                     (double)rc_throttle,
                     link_lost_ms > 0u ? motors_last_ms - link_lost_ms : 0u);
            report(rc_frames > rc_frames_at_failsafe + 10u &&
                       motors_last_ms > link_lost_ms &&
                       motors_last_ms - link_lost_ms <= 400u,
                   "and a receiver inside its own failsafe did not fly it",
                   detail);
        }

        /*
         * Which receiver. This airframe's is an SBUS one: the console was told
         * at 200 ms, the firmware told the board, the board's line settings
         * changed and the parser that saw the bytes changed with them - and
         * the aircraft then armed and flew on those frames. A firmware that
         * read the wrong protocol would have counted a stream of framing
         * errors and never armed, so the two checks above are also the proof
         * that this one is not a lie.
         */
        int sbus_frames = console_said("receiver:  sbus,");
        snprintf(detail, sizeof detail, "the simulator's receiver spoke %s",
                 sim_rc_protocol == AK_RC_PROTOCOL_SBUS ? "sbus" : "crsf");
        report(sim_rc_protocol == AK_RC_PROTOCOL_SBUS && sbus_frames,
               "and the receiver it was told to read is the one it read",
               detail);
    }

    printf("sim: %s (%d check%s failed)\n", failures == 0 ? "PASS" : "FAIL",
           failures, failures == 1 ? "" : "s");
    exit(failures == 0 ? 0 : 1);
}

/* One simulated millisecond: the airframe moves, the radio and the GPS send
 * what they were going to send, and the scenario advances. Called from the
 * console poll, which the firmware makes exactly once per pass of the main
 * loop, and from ak_delay_ms for the waits before the loop starts. */
static void sim_step(void)
{
    n_sim_step++;
    virtual_ms++;
    clock_activity = 1;
    plant_step(1.0f / (float)SIM_HZ);

    /* What the mixer said, one millisecond later - the same numbers a scope on
     * the motor pads would show, and the only place a simulator can see whether
     * the aircraft is being flown or merely talked about. */
    int turning = 0;
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        if (plant_outputs.motor[i] > 0.01f) {
            turning = 1;
        }
        if (plant_outputs.motor[i] > motor_peak) {
            motor_peak = plant_outputs.motor[i];
        }
    }
    for (int i = 0; i < AK_MAX_SERVOS; i++) {
        if (ak_absf(plant_outputs.servo[i]) > servo_peak) {
            servo_peak = ak_absf(plant_outputs.servo[i]);
        }
    }

    /* What the output test moved, and what was left afterwards. */
    if (output_test_running) {
        for (int i = 0; i < AK_MAX_MOTORS; i++) {
            if (plant_outputs.motor[i] > 0.01f) {
                output_test_motors |= 1u << (unsigned)i;
            }
        }
        for (int i = 0; i < AK_MAX_SERVOS; i++) {
            if (ak_absf(plant_outputs.servo[i]) > 0.05f) {
                output_test_servos |= 1u << (unsigned)i;
            }
        }
    } else if (output_test_ran && output_frames > output_stop_frame) {
        for (int i = 0; i < AK_MAX_MOTORS; i++) {
            if (plant_outputs.motor[i] > output_test_after_peak) {
                output_test_after_peak = plant_outputs.motor[i];
            }
            if (plant_outputs.motor[i] > output_test_after_motor[i]) {
                output_test_after_motor[i] = plant_outputs.motor[i];
            }
        }
        for (int i = 0; i < AK_MAX_SERVOS; i++) {
            if (ak_absf(plant_outputs.servo[i]) > output_test_after_peak) {
                output_test_after_peak = ak_absf(plant_outputs.servo[i]);
            }
        }
    }

    distance_m = distance_from_home_m();
    if (distance_m > distance_max_m) {
        distance_max_m = distance_m;
    }
    if (step == STEP_RECONNECT && distance_m > loiter_max_m) {
        loiter_max_m = distance_m;
    }
    if (rc_link_lost && (distance_min_after_loss_m == 0.0f ||
                         distance_m < distance_min_after_loss_m)) {
        distance_min_after_loss_m = distance_m;
    }
    if (turning) {
        motors_last_ms = virtual_ms;
        if (motors_first_ms == 0u) {
            motors_first_ms = virtual_ms;
        }
    }

    if (loop_started) {
        /* Deadlines, not `virtual_ms % period`. The clock advances by whatever
         * the spin accounting says it should, so an equality test can miss its
         * slot entirely; a deadline can only be missed by being late. */
        if ((int32_t)(virtual_ms - next_trace_ms) >= 0) {
            next_trace_ms += 1000u;
            if (sim_airframe == SIM_CONSOLE) {
                /* Nothing to narrate: the console is the pilot. */
            } else if (sim_airframe == SIM_WING) {
                printf("sim: %6u ms  roll %6.1f  pitch %6.1f deg  alt %4.0f m  "
                       "%4.0f m out  course %5.1f  motors %.2f %.2f\n",
                       virtual_ms, (double)ak_rad2deg(plant_roll),
                       (double)ak_rad2deg(plant_pitch), (double)plant_alt_m,
                       (double)distance_m, (double)plant_heading_deg,
                       (double)plant_outputs.motor[0],
                       (double)plant_outputs.motor[1]);
            } else {
                printf("sim: %6u ms  roll %6.1f deg  pitch %6.1f deg  "
                       "motors %.2f %.2f %.2f %.2f\n",
                       virtual_ms, (double)ak_rad2deg(plant_roll),
                       (double)ak_rad2deg(plant_pitch),
                       (double)plant_outputs.motor[0],
                       (double)plant_outputs.motor[1],
                       (double)plant_outputs.motor[2],
                       (double)plant_outputs.motor[3]);
            }
        }
        if ((int32_t)(virtual_ms - next_rc_ms) >= 0) {
            next_rc_ms = (virtual_ms - next_rc_ms > SIM_RC_PERIOD_MS)
                             ? virtual_ms + SIM_RC_PERIOD_MS
                             : next_rc_ms + SIM_RC_PERIOD_MS;
            if (!rc_link_lost) {
                rc_refresh();
            }
        }
        if ((int32_t)(virtual_ms - next_gps_ms) >= 0) {
            next_gps_ms = (virtual_ms - next_gps_ms > SIM_GPS_PERIOD_MS)
                              ? virtual_ms + SIM_GPS_PERIOD_MS
                              : next_gps_ms + SIM_GPS_PERIOD_MS;
            /* A module that stops answering mid-flight: the frames simply
             * stop, which is what a receiver that loses its sky does. */
            if (sim_gps_stop_ms == 0u || virtual_ms < sim_gps_stop_ms) {
                gps_send();
                gps_frames_sent++;
            }
        }
    }
    if (sim_airframe == SIM_BENCH) {
        bench_step();
    }
    if (sim_airframe == SIM_MISSION) {
        mission_min_m[0] = mission_min_m[0] == 0.0f
                               ? mission_distance_m(0)
                               : (mission_distance_m(0) < mission_min_m[0]
                                      ? mission_distance_m(0)
                                      : mission_min_m[0]);
        mission_min_m[1] = mission_min_m[1] == 0.0f
                               ? mission_distance_m(1)
                               : (mission_distance_m(1) < mission_min_m[1]
                                      ? mission_distance_m(1)
                                      : mission_min_m[1]);
        if (!mission_finished) {
            mission_end_m[0] = mission_distance_m(0);
            mission_end_m[1] = mission_distance_m(1);
        }
        mission_step();
    }
    if (sim_airframe == SIM_FENCE) {
        /* Everything after the trigger is the return, so the distance is
         * measured from the moment the firmware says the fence fired. */
        if (console_said("fence:")) {
            float home_m = distance_from_home_m();
            fence_min_after_m = fence_min_after_m == 0.0f
                                    ? home_m
                                    : (home_m < fence_min_after_m
                                           ? home_m
                                           : fence_min_after_m);
        }
        /*
         * And "where it ended up" is the end of *this* test, not the end of the
         * run: the ceiling phase below starts after the pilot has the aircraft
         * back, and a distance carried through it would be measuring the
         * pilot's own circling afterwards. The check is about the return, so it
         * is frozen when the return hands over (fence_stage 6).
         */
        if (fence_stage < 6) {
            fence_end_m = distance_from_home_m();
        }
        if (fence_ceiling_fired_ms != 0u) {
            fence_ceiling_end_m = distance_from_home_m();
        }
        fence_step();
    }
    scenario();

    /* A console session has no stopwatch: it ends when the other end hangs up
     * (or when the person at the keyboard has had enough). Everything else
     * runs for the number of seconds it was given. */
    if (run_seconds != 0u && virtual_ms >= run_seconds * 1000u) {
        sim_finish();
    }
}

static void rc_refresh(void)
{
    ak_rc_config_t cfg;
    ak_rc_default_config(&cfg);

    for (int i = 0; i < AK_CRSF_CHANNELS; i++) {
        rc_channels[i] = (uint16_t)cfg.mid;
    }
    /* AETR, the order the firmware expects. */
    rc_channels[AK_RC_ROLL] = (uint16_t)((float)cfg.mid + rc_roll * 819.0f);
    rc_channels[AK_RC_PITCH] = (uint16_t)((float)cfg.mid + rc_pitch * 819.0f);
    rc_channels[AK_RC_YAW] = (uint16_t)((float)cfg.mid + rc_yaw * 819.0f);
    rc_channels[AK_RC_THROTTLE] =
        (uint16_t)((float)cfg.min + rc_throttle * (float)(cfg.max - cfg.min));
    rc_channels[AK_RC_ARM] = rc_arm ? (uint16_t)cfg.max : (uint16_t)cfg.min;
    /* The mode channel: high is angle mode, low is rate mode. Which one is in
     * force is a stick position a pilot can reach in the air, so a scenario
     * that means to fly rate mode has to throw the switch. */
    rc_channels[AK_RC_MODE] = (uint16_t)(sim_rate_mode ? cfg.min : cfg.max);
    /* Channel 7, the first of the two spare aux channels: the mission switch,
     * when a scenario asks for one. */
    rc_channels[6] = mission_switch_on ? (uint16_t)cfg.max : (uint16_t)cfg.min;
    /* Channel 8, the second of the two spare aux channels: the hand-launch
     * switch, when a scenario asks for one. */
    rc_channels[7] = rc_launch ? (uint16_t)cfg.max : (uint16_t)cfg.min;

    rc_send();
}

/* --- the board contract --------------------------------------------------- */

static char clock_summary[] = "simulated, 160 MHz";
/* The same capacity the real record has (AK_PARAMS_TEXT_MAX), so a session
 * that saves a full table is testing the thing the aircraft does. It was 1024
 * - the size the record had before the parameter table outgrew it - and a
 * save of a real table would have failed here while working on a board. */
static char config_text[AK_PARAMS_TEXT_MAX];
static int config_length;

void ak_board_init(void)
{
    imu_regs[0x75] = 0x47; /* WHO_AM_I: an ICM-42688-P answers here */
    imu_regs[0x4D] = 0xC0;
    /* And a DPS310 beside it on its own chip select, with the coefficients the
     * refresh inverts: c10 alone, so the pressure is linear in the reading. */
    baro_regs[0x0D] = 0x10;               /* WHO_AM_I */
    baro_regs[0x10] = 0x7D;               /* c0 = 2000: 0x7D0 */
    baro_regs[0x11] = 0x03;               /* and c1 = 1000: 0x3E8 */
    baro_regs[0x12] = 0xE8;
    baro_regs[0x13] = 0x00;               /* c00: 0 */
    baro_regs[0x14] = 0x00;
    baro_regs[0x15] = 0x01;               /* c10 = 100000: 0x186A0 */
    baro_regs[0x16] = 0x86;
    baro_regs[0x17] = 0xA0;
    baro_regs[0x18] = 0x00;               /* c01 and c11: 0 */
    baro_regs[0x19] = 0x00;
    baro_regs[0x1A] = 0x00;
    baro_regs[0x1B] = 0x00;
    baro_regs[0x1C] = 0x00;               /* c20, c21, c30: 0 */
    baro_regs[0x1D] = 0x00;
    baro_regs[0x1E] = 0x00;
    baro_regs[0x1F] = 0x00;
    /* And the rangefinder on the same wires at its own address: the address
     * register is what its driver reads back as "something is there". */
    range_regs[0x0F] = 0x52;
    baro_regs[0x20] = 0x00;
    baro_regs[0x21] = 0x00;
    baro_regs[0x28] = 0x00;
    /* Coefficient and sensor ready from the first read, as a part that came out
     * of reset a while ago would be. */
    baro_regs[0x08] = 0xF0;
}

const char *ak_board_name(void) { return "SIM / software in the loop"; }
/* Quad-X, the core's own default: the simulator is not a board and has no
 * header to have an opinion about. A scenario that wants a wing sets the
 * parameter, which is what a person would do on a real one. */
uint32_t ak_board_default_airframe(void) { return 0u; }
void ak_board_led_set(int on) { (void)on; }
void ak_board_led_toggle(void) {}
int ak_board_led_state(void) { return 0; }
const char *ak_board_clock_summary(void) { return clock_summary; }
int ak_board_clock_ok(void) { return sim_lie_clock ? 0 : 1; }
uint32_t ak_board_clock_sysclk_hz(void) { return 160000000u; }
uint32_t ak_board_clock_apb1_hz(void) { return 160000000u; }
uint32_t ak_board_console_port(void) { return 1u; }
/* A console on a port the board does not have: the one preflight finding that
 * a person cannot see by looking at the aircraft, because it is a wiring
 * mistake inside the firmware. */
uint32_t ak_board_console_attached_port(void)
{
    return sim_lie_console_port ? 0x40004800u : 1u;
}
void ak_board_reboot(void) {}
/* No ROM bootloader in a simulator, and no hand on a board either: `dfu` says
 * so, which is the message the CLIs of the two parts that have one never
 * reach. */
int ak_board_enter_bootloader(void) { return 0; }

int ak_board_config_read(void *buf, uint32_t len)
{
    /* A record whose length does not add up, or whose checksum is wrong: the
     * board cannot tell the difference, and neither can this. */
    if (sim_lie_config) {
        return -1;
    }
    if (config_length == 0) {
        return 0; /* nothing stored is not a failure */
    }
    if ((uint32_t)config_length + 1u > len) {
        /* A caller whose buffer is too small gets the failure every real board
         * gives it. Returning 0 here - "nothing stored" - is what kept a core
         * bug invisible: the preflight asked with a 64-byte buffer and the
         * simulator, alone among the three implementations, said there was no
         * configuration at all. */
        return -1;
    }
    memcpy(buf, config_text, (size_t)config_length);
    ((char *)buf)[config_length] = '\0';
    return config_length;
}

int ak_board_config_write(const void *buf, uint32_t len)
{
    /* `sim_lie_config_write`: a board whose flash will not take the record -
     * a controller that never finishes, a sector that will not unlock. The
     * console's answer to that is the sentence this lie exists to produce: a
     * save that failed is not a save, and the parameters' changed count has to
     * stay where it was. */
    if (sim_lie_config_write) {
        return -1;
    }
    if (len >= sizeof config_text) {
        return -1;
    }
    memcpy(config_text, buf, len);
    config_text[len] = '\0';
    config_length = (int)len;
    return 0;
}

void ak_board_output_init(void) {}
/* `noout`: the port came up with no timer channels at all - a board whose
 * outputs never answered, which is a fact about boot rather than a command. */
int ak_board_output_ready(void) { return sim_no_outputs ? 0 : 1; }
/* The simulator's board drives four motors and two servos, the same shape the
 * firmware's own boards answer with - so both airframes' mixes fit it and the
 * SIL sessions arm. A board that answered less would be a session that could not
 * arm, which is the point of the check rather than a problem with it. */
void ak_board_output_shape(unsigned *motors, unsigned *servos)
{
    /* `twomotor`: the ESP32-C3's shape - two RMT transmit channels - which a
     * quadrotor's mix does not fit. The arming gate refuses with both numbers
     * in the sentence, and the preflight says the same thing before anybody
     * arms anything. */
    *motors = sim_two_motors ? 2u : 4u;
    *servos = 2u;
}

void ak_board_output_write(const ak_output_frame_t *frame)
{
    clock_activity = 1;
    output_frames++;
    /* The mixer's decision, fed back into the airframe. This is the join that
     * makes it a loop rather than a test: the numbers a board would send to its
     * timers are the numbers the model flies on. */
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        /* The DShot frame carries the 11-bit throttle in bits 15..5. */
        uint16_t throttle = (uint16_t)(frame->dshot[i] >> 5);
        plant_outputs.motor[i] = (float)throttle / 2047.0f;
    }
    for (int i = 0; i < AK_MAX_SERVOS; i++) {
        /* And a servo comes back out of its pulse width, so the model flies on
         * what a servo horn would actually do - including the encoder's
         * clamping, which a wing with a reversed or over-driven elevon would
         * otherwise not see. */
        plant_outputs.servo[i] =
            ((float)frame->servo_us[i] - (float)AK_SERVO_CENTER_US) /
            ((float)(AK_SERVO_MAX_US - AK_SERVO_CENTER_US));
    }
}

void ak_board_output_set_rate(uint32_t khz) { (void)khz; }
void ak_board_output_report(ak_printf_fn out) { out("outputs:   simulated\n"); }
uint32_t ak_board_output_dshot_hz(void)
{
    /* Two kilohertz away from what was asked for: the timers are running, at
     * the wrong rate, which is a real one when a prescaler is wrong. */
    return sim_lie_dshot ? 302000u : 300000u;
}
uint32_t ak_board_output_dshot_period(void) { return 279u; }
uint32_t ak_board_output_ccr_zero(void) { return 98u; }
uint32_t ak_board_output_ccr_one(void) { return 196u; }
uint32_t ak_board_output_frames_sent(void) { return virtual_ms; }

void ak_board_rc_init(void) { ak_ring_init(&rc_ring); }

/* What the firmware says the receiver is speaking, which is what this
 * simulator's receiver then speaks. A board that ignored this and kept sending
 * one protocol while the parser read the other is exactly the bug the check on
 * the far side is looking for. */
void ak_board_rc_set_protocol(uint32_t protocol) { sim_rc_protocol = protocol; }

/* The simulated wire is a logic-level one: whatever the receiver drives is
 * what the port reads, so SBUS's inversion is already handled by whatever is
 * between them - here, nothing. */
int ak_board_rc_inverted(void) { return 1; }

/* Telemetry out: the firmware's frames, parsed on the way past the way the
 * receiver and the handset would parse them. */
int ak_board_rc_send(const char *data, unsigned len)
{
    for (unsigned i = 0; i < len; i++) {
        tlm_feed((uint8_t)data[i]);
    }
    return 0;
}

int ak_board_rc_poll(uint8_t *byte)
{
    /* This is real board activity: the loop is doing work, not standing on the
     * clock, so the next clock read is not a spin. */
    clock_activity = 1;
    if (!ak_ring_pop(&rc_ring, byte)) {
        return 0;
    }
    rc_bytes_taken++;
    return 1;
}
uint32_t ak_board_rc_dropped(void) { return rc_ring.dropped; }

void ak_board_gps_init(void) { ak_ring_init(&gps_ring); }
int ak_board_gps_poll(uint8_t *byte)
{
    clock_activity = 1;
    return ak_ring_pop(&gps_ring, byte);
}
uint32_t ak_board_gps_dropped(void) { return gps_ring.dropped; }
int ak_board_gps_send(const char *data, unsigned len)
{
    if (len <= sizeof sim_gps_cfg) {
        memcpy(sim_gps_cfg, data, len);
        sim_gps_cfg_len = len;
    }
    sim_gps_cfg_sends++;
    return 0; /* the frame is on the wire as far as the driver can tell */
}

void ak_board_imu_init(void) {}
const ak_bus_t *ak_board_imu_bus(void) { return &sim_bus; }
const ak_bus_t *ak_board_baro_bus(void)
{
    return sim_baro_fitted ? &baro_bus : 0;
}
const ak_bus_t *ak_board_range_bus(void)
{
    return sim_range_fitted ? &range_bus : 0;
}
void ak_board_spi_loopback(ak_printf_fn out) { out("spi:       simulated\n"); }

/*
 * The flight pack, in volts at the pin - which is what the board contract says
 * a board measures, and the whole of what it says. The divider's ratio, the
 * cell count and the thresholds are the flight core's; a board that reported
 * pack volts here instead would have taken the arithmetic with it, and the two
 * targets would drift apart one number at a time.
 */
void ak_board_battery_init(void) {}
int ak_board_battery_ready(void) { return sim_vbat_fitted ? 1 : 0; }
float ak_board_battery_pin_volts(void) { return sim_pack_v / SIM_PACK_DIVIDER; }
void ak_board_battery_report(ak_printf_fn out)
{
    float pin = sim_pack_v / SIM_PACK_DIVIDER;

    out("adc:       simulated: %d.%03d V at the pin behind a %d:1 divider\n",
        (int)pin, (int)(pin * 1000.0f) % 1000, (int)SIM_PACK_DIVIDER);
}

/*
 * No network in the simulation.
 *
 * The ESP32's socket lives in a FreeRTOS task and the bytes reach the flight
 * loop through a stream buffer; none of that exists here, and the routing it
 * feeds - a second protocol parser, and telemetry for whoever asked for it - is
 * exercised on the target it was written for, under QEMU, by a client on a
 * real socket. What this file does not have is the two ends of the wire.
 */
int ak_board_net_ready(void) { return 0; }
int ak_board_net_connected(void) { return 0; }
int ak_board_net_poll_rx(char *byte) { (void)byte; return 0; }
void ak_board_net_write(const char *data, unsigned len) { (void)data; (void)len; }
void ak_board_net_report(ak_printf_fn out) { out("net:       simulated, none\n"); }
void ak_board_net_start(void) {}
unsigned ak_board_param_table(ak_param_t *items, unsigned count)
{
    (void)items;
    return count;
}

/*
 * Retained RAM, simulated by ordinary memory.
 *
 * A host process starts with zeroed memory, which is exactly what a board
 * with a *dead* battery looks like on a cold boot - so the simulator runs the
 * same path the firmware takes when there is nothing to resume, including the
 * check that rejects uninitialised RAM rather than reading it as a log. What it
 * cannot do is have a previous run's log in there: that needs a process that
 * outlives its own reset, which is the unit test's job with a filled buffer.
 */
static ak_log_t sim_retained;

void *ak_board_retained_ram(unsigned *bytes)
{
    *bytes = sizeof sim_retained;
    return &sim_retained;
}

/*
 * The simulated board's flash log, and why it is ordinary memory: the flight
 * loop should run the same logging code the board runs, and `make proto-test`
 * should be able to pull the log over the protocol the way a tool on the bench
 * pulls a real one. What it does not model is the part that needs silicon -
 * erase timing, the busy flag, wear - and it is generous with room (four
 * sectors) so that no session has to erase anything.
 *
 * The flash log's own logic against a store it cannot see is in
 * tests/test_log.c, and against the real board's region and the modelled
 * controller in tests/test_arch.c.
 */
#define SIM_LOG_SECTORS 4u
#define SIM_LOG_SECTOR_BYTES 8192u

static uint8_t sim_log_flash[SIM_LOG_SECTORS][SIM_LOG_SECTOR_BYTES];
static ak_flashlog_region_t sim_log_regions[SIM_LOG_SECTORS];
static int sim_log_ready;

/* Flat, like the part: see the note in tools/akproto_sim.c. */
static uint8_t *const sim_log_bytes = &sim_log_flash[0][0];

static uint32_t sim_log_read(uint32_t address)
{
    uint32_t offset = address - (uint32_t)(uintptr_t)sim_log_bytes;
    uint32_t word = 0u;

    if ((offset + 4u) > sizeof sim_log_flash) {
        return 0xFFFFFFFFu;
    }
    for (unsigned i = 0u; i < 4u; i++) {
        word |= (uint32_t)sim_log_bytes[offset + i] << (8u * i);
    }
    return word;
}

static int sim_log_erase(unsigned index)
{
    if (index >= SIM_LOG_SECTORS) {
        return -1;
    }
    memset(sim_log_flash[index], 0xFF, SIM_LOG_SECTOR_BYTES);
    return 0;
}

/* Word by word, and only into words that are still erased, because that is the
 * rule the real part enforces and a double that ignored it would let a bug
 * through that flash would refuse. */
static int sim_log_write(uint32_t address, const void *bytes, unsigned len)
{
    const uint8_t *source = bytes;
    uint32_t offset = address - (uint32_t)(uintptr_t)sim_log_bytes;

    if ((offset + len) > sizeof sim_log_flash) {
        return -1;
    }
    for (unsigned i = 0u; i < len; i++) {
        uint8_t *cell = &sim_log_bytes[offset + i];

        if (*cell != 0xFFu && *cell != source[i]) {
            return -1;
        }
        *cell = (uint8_t)(*cell & source[i]);
    }
    return 0;
}

static const ak_flashlog_store_t sim_log_store = {
    .regions = sim_log_regions,
    .count = SIM_LOG_SECTORS,
    .read = sim_log_read,
    .read_block = 0, /* the simulator's flash is memory, like the F405's */
    .erase = sim_log_erase,
    .write = sim_log_write,
};

const ak_flashlog_store_t *ak_board_log_store(void)
{
    if (!sim_log_ready) {
        for (unsigned i = 0u; i < SIM_LOG_SECTORS; i++) {
            sim_log_regions[i].base =
                (uint32_t)(uintptr_t)&sim_log_flash[i][0];
            sim_log_regions[i].bytes = SIM_LOG_SECTOR_BYTES;
            (void)sim_log_erase(i);
        }
        sim_log_ready = 1;
    }
    return &sim_log_store;
}

/*
 * The firmware's output goes to stdout, and somebody types at it while it
 * flies - so the CLI runs against a live flight controller rather than against
 * a mock. The transcript then carries two views of the same aircraft at the
 * same timestamps: the simulator's (the airframe's attitude and the mixer's
 * numbers) and the firmware's own (what `status` thinks its state is). Where
 * those two disagree is where a bug is.
 */
typedef struct {
    uint32_t at_ms;
    const char *text;
} console_line_t;

/*
 * The wing is set up by typing at the console, not by reaching into the
 * firmware: `set airframe 1` is the same command a person would use on a bench,
 * and it goes through the parameter table, the range check and the hook that
 * re-resolves the mixer. `set rth_enable 1` is the switch that is off by
 * default because an untested automatic behaviour is worse than a stop - and
 * this is the thing that tests it.
 */
static const console_line_t console_script_quad[] = {
    /* The receiver is an SBUS one on this airframe. Typed rather than assumed,
     * because that is what a person does on a bench - and it puts the
     * parameter, the board's line settings, the parser and the flight core on
     * one path from a console command to a turning motor. */
    { 200u, "set rc_protocol 1\n" },
    { 2500u, "status\n" },
    { 5500u, "status\n" },
    { 7500u, "rc\n" },
    { 8500u, "battery\n" },
    { 10500u, "status\n" },
    { 12500u, "gps\n" },
    { 13500u, "battery\n" },
    { 14500u, "status\n" },
    { 15500u, "output test\n" },
    { 16500u, "preflight\n" },
    { 18500u, "status\n" },
};

static const console_line_t console_script_wing[] = {
    { 700u, "set airframe 1\n" },
    { 1500u, "set rth_enable 1\n" },
    { 4000u, "status\n" },
    { 9500u, "baro\n" },
    { 9000u, "status\n" },
    { 13000u, "status\n" },
    { 17000u, "gps\n" },
    { 21000u, "status\n" },
    { 25000u, "rc\n" },
    { 29000u, "status\n" },
    { 33000u, "preflight\n" },
    { 37000u, "status\n" },
};

/*
 * The wing with no navigator, for the session that loses its link with nothing
 * to bring it home: the *only* thing the console has to do is make the
 * firmware a wing, because RTH off is the default a board ships with. The
 * status line afterwards is the firmware's own view of what it is doing when
 * the receiver has come out.
 */
static const console_line_t console_script_wing_lost[] = {
    { 700u, "set airframe 1\n" },
    { 14000u, "status\n" },
};

/*
 * The board that lies, typed at the console.
 *
 * One `preflight` three hundred milliseconds after each stage of
 * badboard_step() arms a lie, and one before the first and after the last, so
 * the same command is on the tape both refusing and passing. Nothing else is
 * typed: this session is not about the aircraft.
 */
static const console_line_t console_script_badboard[] = {
    { 900u,  "preflight\n" },
    { 1400u, "preflight\n" },
    { 1900u, "preflight\n" },
    { 2400u, "preflight\n" },
    { 2900u, "preflight\n" },
    { 3400u, "preflight\n" },
    { 3900u, "preflight\n" },
    { 4400u, "preflight\n" },
};

/* The two boot-time lies: the board has no outputs at all, or it drives two
 * motors - the ESP32-C3's shape - which a quadrotor's mix does not fit. One
 * command, and the boot has already said what the board is. */
static const console_line_t console_script_badboot[] = {
    { 900u, "preflight\n" },
};

/* A module that is there and not talking: ask it what it sees, wait long enough
 * for all four configuration attempts, and ask again. */
static const console_line_t console_script_gps_quiet[] = {
    { 1200u, "gps\n" },
    { 2000u, "preflight\n" },
    { 9000u, "gps\n" },
};

static const console_line_t *console_script = console_script_quad;
static unsigned console_lines = sizeof console_script_quad / sizeof console_script_quad[0];
static unsigned console_line;
static unsigned console_char;

int ak_board_console_poll_rx(char *byte)
{
    clock_activity = 1;

    /* Console mode: the bytes come from whoever is typing, which is what makes
     * the simulator a stand-in for a board on a bench rather than a recording
     * of one. Non-blocking, like a UART's receive register: the loop has an
     * aircraft to fly whether or not somebody is typing. */
    if (console_stdin) {
        ssize_t got = read(STDIN_FILENO, byte, 1);
        /* The same accounting the scripted modes get: the radio and the GPS
         * start when the firmware's loop does, not while it is still booting. */
        note_loop_started();
        sim_step();
        if (got == 0) {
            /* The other end hung up. Nothing here should spin on a closed
             * pipe, so the session ends the way the timed ones do. */
            sim_finish();
        }
        return got == 1 ? 1 : 0;
    }

    /* The bench scenario types its own lines, one at a time, and only after
     * what it typed last has been answered. */
    if (typed_line != 0) {
        if (typed_line[typed_at] != '\0') {
            *byte = typed_line[typed_at++];
            return 1;
        }
        typed_line = 0;
        typed_at = 0;
    }

    if (console_line < console_lines &&
        (int32_t)(virtual_ms - console_script[console_line].at_ms) >= 0) {
        const char *text = console_script[console_line].text;
        if (text[console_char] != '\0') {
            *byte = text[console_char++];
            return 1;
        }
        console_line++;
        console_char = 0;
    }

    /* This is the one board call the firmware makes exactly once per loop
     * iteration when nothing is arriving, which makes it the place where a
     * simulated millisecond belongs. Advancing time in the output path was the
     * first attempt, and it deadlocked: the loop only writes an output after
     * its clock has moved, and its clock only moved in the output path. */
    note_loop_started();
    sim_step();
    return 0;
}

/*
 * The console, tee'd.
 *
 * What the firmware prints is its own testimony: `status` says what state it
 * thinks it is in, and the boot line says where it thinks home is. The
 * simulator reads that back, which turns some seam-level checks into
 * end-to-end ones - "the bytes arrived" becomes "the position arrived".
 *
 * The capture stops when it is full rather than wrapping. Everything worth
 * reading is printed in the first second, and a buffer that silently loses its
 * beginning would make a check fail for the wrong reason.
 */
/*
 * Everything the console printed, for the checks to read.
 *
 * It was 16 KB, and that is smaller than one `log flash` dump of a full
 * flight - fifteen kilobytes - before the boot report, the selftest, the
 * session's own tape and every report the scenario typed are counted. The
 * capture filled, stopped appending, and the check that reads the log's
 * columns back found a hundred and fifty records of a parked aircraft instead
 * of a flight: an instrument limit that read as a firmware fault, which is the
 * third time this project has had one of those.
 */
static char console_capture[131072];
static unsigned console_capture_len;

static int console_said(const char *needle)
{
    return strstr(console_capture, needle) != 0;
}

/* How many times a line came out, for the checks that have to tell "the answer
 * came back clean" from "the boot's own clean answer is in there somewhere".
 * The badboard session types the same command eight times. */
static int console_count(const char *needle)
{
    int count = 0;
    const char *at = console_capture;
    size_t length = strlen(needle);

    if (length == 0u) {
        return 0;
    }
    while ((at = strstr(at, needle)) != 0) {
        count++;
        at += length;
    }
    return count;
}

/*
 * The two numbers in a printed line like "(%u samples, %u rejected as moving)",
 * for the checks that want to read back what the firmware measured rather than
 * only that it said something. Returns 0 when the needle, the comma or either
 * number is not there.
 */
static int console_pair(const char *needle, unsigned *first, unsigned *second)
{
    const char *at = strstr(console_capture, needle);
    char *end = 0;

    if (at == 0) {
        return 0;
    }
    at += strlen(needle);
    unsigned a = (unsigned)strtoul(at, &end, 10);
    if (end == at) {
        return 0;
    }
    /* The two numbers are not always adjacent: the firmware's own line says
     * "(%u samples, %u rejected as moving)", and the word between them is
     * what a person reads. */
    const char *comma = strchr(end, ',');
    if (comma == 0) {
        return 0;
    }
    const char *second_at = comma + 1;
    unsigned b = (unsigned)strtoul(second_at, &end, 10);
    if (end == second_at) {
        return 0;
    }
    *first = a;
    *second = b;
    return 1;
}

/* The number the console printed after a parameter's name, for checks that
 * have to allow for the sensor's own resolution. Returns 0 if the name is not
 * in the capture at all. */
static int console_number(const char *name, double *out)
{
    char needle[40];
    snprintf(needle, sizeof needle, "%s ", name);

    const char *at = strstr(console_capture, needle);
    if (at == 0) {
        return 0;
    }
    at += strlen(needle);
    while (*at == ' ') {
        at++;
    }
    char *end = 0;
    double value = strtod(at, &end);
    if (end == at) {
        return 0;
    }
    *out = value;
    return 1;
}

/* The three accelerometer readings the `imu` command printed, in per-mille of
 * g. These are what the flight loop is actually using. */
static int console_accel(int out[3])
{
    const char *at = strstr(console_capture, "sample:    accel ");
    if (at == 0) {
        return 0;
    }
    at += strlen("sample:    accel ");
    for (int i = 0; i < 3; i++) {
        char *end = 0;
        out[i] = (int)strtol(at, &end, 10);
        if (end == at) {
            return 0;
        }
        at = end;
    }
    return 1;
}

/*
 * The blackbox, read the way a person reads it after a flight: the columns for
 * the height and the yaw, in as much of the dump as the console capture holds
 * (the first sixteen kilobytes of it, which is the take-off end of the flight).
 * The layout tests prove the record *has* the fields; this proves the firmware
 * fills them with the flight - a log whose height column is zero from take-off
 * to landing is a log that cannot answer the question a bad landing asks.
 */
/*
 * Where the module put home, when the fix it came from was noisy. The exact
 * string is not there to find in that case, so this reads the coordinates out
 * of the boot line and asks whether they are within five metres of where the
 * aircraft was standing - which is four times the noise the session asked for.
 */
static int console_home_near(void)
{
    const char *at = strstr(console_capture,
                            "from the first fix while disarmed, ");
    const char *p;
    char *stop = 0;
    double lat;
    double lon;
    long lat_e7;
    long lon_e7;

    if (at == 0) {
        return 0;
    }
    p = at + strlen("from the first fix while disarmed, ");
    lat = strtod(p, &stop);
    if (stop == 0 || *stop != ',') {
        return 0;
    }
    lon = strtod(stop + 1, &stop);
    lat_e7 = (long)(lat * 1e7);
    lon_e7 = (long)(lon * 1e7);
    return labs(lat_e7 - SIM_HOME_LAT_E7) < 500 &&
           labs(lon_e7 - SIM_HOME_LON_E7) < 500;
}

static void report_log_columns(void)
{
    char detail[128];
    const char *rows = strstr(console_capture, "\ntime_ms,gyro_x");
    float low_mm = 1.0e9f;
    float high_mm = -1.0e9f;
    float yaw_span_deg = 0.0f;
    float yaw_low = 1.0e9f;
    float yaw_high = -1.0e9f;
    /*
     * And where the last record says it was, against where the plant says it
     * is: the log's own answer to "where did it come down", checked against
     * the aircraft rather than against itself.
     */
    double last_lat_e7 = 0.0;
    double last_lon_e7 = 0.0;
    int    have_position = 0;
    unsigned rows_seen = 0;

    if (rows != 0) {
        const char *line = strchr(rows + 1, '\n');

        while (line != 0 && *line != '\0') {
            /* time, 3 gyro, 3 accel, roll, pitch, yaw, alt, ... */
            char buf[512];
            const char *end = strchr(line + 1, '\n');
            unsigned len = end == 0 ? (unsigned)strlen(line + 1)
                                    : (unsigned)(end - (line + 1));
            unsigned field = 0;

            if (len >= sizeof buf || len == 0u) {
                break;
            }
            memcpy(buf, line + 1, len);
            buf[len] = '\0';
            for (const char *p = buf; *p != '\0'; p++) {
                if (*p == ',') {
                    field++;
                } else if (field == 9 || field == 10 || field == 24 ||
                           field == 25) {
                    char *stop = 0;
                    float value = (float)strtod(p, &stop);

                    if (field == 9) {
                        if (value < yaw_low) {
                            yaw_low = value;
                        }
                        if (value > yaw_high) {
                            yaw_high = value;
                        }
                    } else if (field == 10) {
                        if (value < low_mm) {
                            low_mm = value;
                        }
                        if (value > high_mm) {
                            high_mm = value;
                        }
                    } else if (field == 24) {
                        last_lat_e7 = value;
                        have_position = 1;
                    } else {
                        last_lon_e7 = value;
                    }
                    p = stop - 1;
                }
            }
            rows_seen++;
            line = end;
        }
        yaw_span_deg = (yaw_high - yaw_low) / 10.0f;
    }

    /*
     * How far the last record's position is from where the plant says the
     * aircraft is, in metres, both ways.
     */
    double dlat_m = ((last_lat_e7 - (double)SIM_HOME_LAT_E7) / 1e7) *
                    (double)SIM_M_PER_DEG_LAT;
    double dlon_m = ((last_lon_e7 - (double)SIM_HOME_LON_E7) / 1e7) *
                    (double)sim_m_per_deg_lon();
    double position_error_m =
        sqrt((dlat_m - (double)plant_north_m) * (dlat_m - (double)plant_north_m) +
             (dlon_m - (double)plant_east_m) * (dlon_m - (double)plant_east_m));

    snprintf(detail, sizeof detail,
             "%u records, height %.1f to %.1f m, yaw span %.0f deg, last "
             "position %.0f m from where the aircraft is",
             rows_seen, (double)(low_mm / 1000.0f), (double)(high_mm / 1000.0f),
             (double)yaw_span_deg, (double)position_error_m);
    /*
     * The height has to have *been* somewhere the aircraft was - 15 m up on a
     * flight that holds 22 - and the yaw has to have moved with the flight.
     * Not that the low end is the ground: the capture holds the first sixteen
     * kilobytes of a dump that is fifty, so which part of a flight is visible
     * depends on how long the flight was - the hundred-second return shows its
     * take-off (0.0 m up) and the hundred-and-seventy-second one shows only its
     * hold (23.0 m). What the check is for is that the columns are *filled*,
     * and the layout tests are what say they are in the right place.
     */
    report(rows_seen > 100u && high_mm > 15000.0f && yaw_span_deg > 45.0f,
           "and the blackbox carries the height and the yaw of the flight",
           detail);
    /*
     * And the position - the field the log grew for the question that comes
     * after a crash, "where is it". The last record's position has to be where
     * the plant says the aircraft is, not merely a plausible latitude: a log
     * that records the *home* position on every row, or zeroes, would pass
     * every other check in this file.
     */
    report(have_position && position_error_m < 20.0,
           "and where the flight ended, from the last record in it", detail);
}

void ak_console_write_raw(const char *data, unsigned len)
{
    fwrite(data, 1, len, stdout);

    for (unsigned i = 0; i < len; i++) {
        if (console_capture_len + 1u < sizeof console_capture) {
            console_capture[console_capture_len++] = data[i];
            console_capture[console_capture_len] = '\0';
        }
    }

    /* The barometer's report, taken as it arrives: what the firmware says the
     * height is, and what the airframe is actually doing at that moment. The
     * number is only read once the line has ended, so a half-written one is
     * not mistaken for an answer. */
    if (!baro_reported) {
        const char *at = strstr(console_capture, "height:    ");
        if (at != 0) {
            char *end = 0;
            float value = (float)strtod(at + strlen("height:    "), &end);
            if (end != 0 && *end == ' ') {
                baro_reported_m = value;
                baro_truth_m = plant_alt_m;
                baro_reported = 1;
            }
        }
    }

    /* And the fused height, on its own flag: it is the one the aircraft
     * actually flies on - the barometer's changes with the gps correcting its
     * drift - and it arrives further down the same report. */
    if (!fused_reported) {
        const char *fused_at = strstr(console_capture, "fused:     ");
        if (fused_at != 0) {
            char *fused_end = 0;
            float value = (float)strtod(fused_at + strlen("fused:     "),
                                        &fused_end);
            if (fused_end != 0 && *fused_end == ' ') {
                fused_reported_m = value;
                fused_reported = 1;
            }
        }
    }

    /* The output test, from the two lines the firmware prints about it: the
     * frames in between are what the timers saw. */
    if (!output_test_running && strstr(console_capture,
                                       "output test: motors to") != 0) {
        output_test_running = 1;
        output_test_ran = 1;
    }
    if (output_test_running &&
        strstr(console_capture, "output test: stopped") != 0) {
        output_test_running = 0;
        output_stop_frame = output_frames;
    }
}

/* --- driving time --------------------------------------------------------- */

void ak_delay_ms(uint32_t ms)
{
    /* Delays happen at boot and during calibration; the flight loop never comes
     * through here, because it polls. */
    for (uint32_t i = 0; i < ms; i++) {
        sim_step();
    }
}

uint32_t ak_delay_stalls(void)
{
    /* The simulator's clock is the loop it just stepped, so a delay cannot
     * stall here: the count is always zero, and the preflight's line about it
     * never has anything to say. */
    return 0u;
}

int main(int argc, char **argv)
{
    if (argc > 1) {
        run_seconds = (uint32_t)atoi(argv[1]);
    }
    if (argc > 2) {
        /*
         * Which sessions fly in wind.
         *
         * Every flying session does, since the plant learned about wind - the
         * quadrotor's return included. The wing's is the one the wind explains
         * best: its steering is done on the ground course the GPS reports,
         * which is a track rather than a heading, and a crosswind is exactly
         * what tells those two apart (its nose ends up to 86 degrees off the
         * track it is making).
         *
         * The quadrotor's return used to *appear* to fail in wind - altitude
         * hunting by seventeen metres, stopping short of home - and the first
         * explanation for it (a lean stealing thrust from the vertical loop
         * through the mixer's authority limit) was wrong. The plant had no
         * drag: a tilt set an acceleration and a hover throttle set a climb
         * rate that nothing opposed, so both of the firmware's loops were
         * being asked to stabilise a spacecraft. With drag in the plant the
         * same return lands in the same wind.
         *
         * A word after the session name - `calm` - takes the wind out, so the
         * same scenario can be flown in still air and the two compared:
         *
         *   build-host/aerialkit-fw-sim 100 quadrth calm
         *
         * That is how the calm numbers in docs/14-navigation.md were
         * measured, and it is worth doing rather than assuming: the
         * quadrotor's return learns the wind so that holding a station over
         * home does not need a standing position error, and a term that
         * learns a wind has to do nothing when there is not one.
         */
        sim_wind_e_m_s = 0.0f;
        if (strcmp(argv[2], "console") == 0) {
            /* The bench runner's mode: no scenario, no scripted typing, and
             * the console on stdin. The loop still flies, still models the
             * sensors and still sends radio and GPS frames - it is a board on
             * a bench with somebody at the console. */
            sim_airframe = SIM_CONSOLE;
            console_lines = 0;
            console_stdin = 1;
            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            if (flags >= 0) {
                (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
            }
            /* Line buffered, because this mode is a conversation rather than a
             * recording: a pipe's default block buffering holds the whole boot
             * report until four kilobytes have accumulated or the process
             * exits, and a console that answers only at the end is not a
             * console - it is a file that will be printed later. */
            (void)setvbuf(stdout, 0, _IOLBF, 0);
        } else if (strcmp(argv[2], "quadrth") == 0) {
            /* The quadrotor's return, flown. It starts pointing east with the
             * estimator's yaw at zero, so the return needs the ground-track
             * alignment to have worked before the aircraft can translate
             * anywhere sensible. */
            sim_airframe = SIM_QUAD_RTH;
            console_lines = 0;
            plant_heading_deg = 90.0f;
            quadrth_closest_m = 1.0e9f;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
        } else if (strcmp(argv[2], "gpslost") == 0) {
            /* The same return, with the GPS module going quiet on the way
             * home: see gpslost_step(). */
            sim_airframe = SIM_GPS_LOST;
            console_lines = 0;
            plant_heading_deg = 90.0f;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
        } else if (strcmp(argv[2], "gpslostland") == 0) {
            /* The same flight with `quad_hold_land_s` set and a rangefinder
             * fitted: the hold becomes a descent onto whatever is below it,
             * which is the one thing a quadrotor with no position can still
             * do - and the reason the rangefinder was added. */
            sim_airframe = SIM_GPS_LOST_LAND;
            console_lines = 0;
            plant_heading_deg = 90.0f;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
        } else if (strcmp(argv[2], "battery") == 0) {
            /* The pack goes flat under a pilot who is still flying it: see
             * battery_step(). */
            sim_airframe = SIM_BATTERY;
            console_lines = 0;
            plant_heading_deg = 90.0f;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
        } else if (strcmp(argv[2], "rate") == 0) {
            /* Rate mode, with the sticks: see rate_step(). */
            sim_airframe = SIM_RATE;
            console_lines = 0;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
        } else if (strcmp(argv[2], "noimu") == 0) {
            /* The gyro stops answering in flight: see noimu_step(). */
            sim_airframe = SIM_NO_IMU;
            console_lines = 0;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
        } else if (strcmp(argv[2], "tilt") == 0) {
            /* An aircraft held at an angle with the arm switch thrown, and
             * what the firmware does about it: see tilt_step(). Held from
             * before the first sample, so the estimate has only ever seen this
             * angle and the refusal cannot be about a filter still coming up. */
            sim_airframe = SIM_TILT;
            console_lines = 0;
            sim_held = 1;
            sim_hold_roll_deg = TILT_HELD_DEG;
            sim_hold_pitch_deg = 0.0f;
        } else if (strcmp(argv[2], "launch") == 0) {
            /* A wing thrown by hand, with a switch for the throw this plant
             * cannot model: see launch_seq(). */
            sim_airframe = SIM_LAUNCH;
            console_lines = 0;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
        } else if (strcmp(argv[2], "wing") == 0) {
            sim_airframe = SIM_WING;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
            console_script = console_script_wing;
            console_lines =
                sizeof console_script_wing / sizeof console_script_wing[0];
        } else if (strcmp(argv[2], "winglost") == 0) {
            /* The same wing with nothing to bring it home: no navigator, so
             * when the receiver comes out the rule the owner asked for is the
             * one that applies - land as it circles down. Same plant as the
             * wing above, different script and different checks. */
            sim_airframe = SIM_WING;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
            sim_wing_lost = 1;
            console_script = console_script_wing_lost;
            console_lines = sizeof console_script_wing_lost /
                            sizeof console_script_wing_lost[0];
        } else if (strcmp(argv[2], "bench") == 0) {
            /* No console script: the six positions are typed by bench_step(),
             * which has to watch what comes back before it types the next. */
            sim_airframe = SIM_BENCH;
            console_lines = 0;
        } else if (strcmp(argv[2], "fault") == 0) {
            /* A board that stopped and left a record: no flight, no script -
            * the boot prints what the last one left behind. */
            sim_airframe = SIM_FAULT;
            console_lines = 0;
        } else if (strcmp(argv[2], "gpsquiet") == 0) {
            /* A GPS module that never speaks: nothing but the four
             * configuration frames go out, and the firmware flies without a
             * fix - which is the state a person is in when the module is
             * wired, powered, and still speaking NMEA. */
            sim_airframe = SIM_GPS_QUIET;
            sim_gps_stop_ms = 1u; /* silent from the first millisecond */
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
            console_script = console_script_gps_quiet;
            console_lines = sizeof console_script_gps_quiet /
                            sizeof console_script_gps_quiet[0];
        } else if (strcmp(argv[2], "badboard") == 0) {
            /* The board that lies: nothing flies, and the run is eight
             * `preflight` commands with one wrong board fact between each.
             * With `noout` or `twomotor` it is instead one command against a
             * board whose outputs are wrong in the way boot decides - see
             * badboard_step(). */
            sim_airframe = SIM_BADBOARD;
            console_script = console_script_badboard;
            console_lines =
                sizeof console_script_badboard / sizeof console_script_badboard[0];
        } else if (strcmp(argv[2], "testarm") == 0) {
            /* The sweep, and the arm switch thrown while it runs. See
             * testarm_step(). The aircraft is *held* - on a bench, props off,
             * which is what the command is for - because a free quadrotor with
             * one motor at fifteen per cent would roll over, and rightly: this
             * is the bench interlock, not a flight. */
            sim_airframe = SIM_TEST_ARM;
            sim_held = 1;
            sim_hold_roll_deg = 0.0f;
            sim_hold_pitch_deg = 0.0f;
            console_lines = 0;
        } else if (strcmp(argv[2], "refuse") == 0) {
            /* The bench's mistakes: the calibrations tried in the wrong state,
             * and the sentences that come back. See refuse_step(). The
             * aircraft is *held* - it is a bench, and one of the cases is
             * somebody picking it up mid-calibration. */
            sim_airframe = SIM_REFUSE;
            sim_held = 1;
            console_lines = 0;
        } else if (strcmp(argv[2], "armnow") == 0) {
            /* Somebody is holding the aircraft, and keeps holding it while
             * the arm switch goes up: the gyro bias never gets its five
             * hundred still samples, so the aircraft arms without one. See
             * armnow_step() and plant_step()'s `sim_handled`. */
            sim_airframe = SIM_ARM_NOW;
            sim_held = 1;
            sim_handled = 1;
            sim_hold_roll_deg = 0.0f;
            sim_hold_pitch_deg = 0.0f;
            console_lines = 0;
        } else if (strcmp(argv[2], "mission") == 0) {
            /* The same: this one works out its own waypoints. */
            sim_airframe = SIM_MISSION;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
            console_lines = 0;
            /* `mission quad` flies the same waypoints on the quadrotor: the
             * same list, the same switch, a different profile - which is the
             * point of the profile being the airframe's answer. */
            if (argc > 3 && strcmp(argv[3], "quad") == 0) {
                mission_airframe = 0;
                plant_heading_deg = 90.0f;
            }
        } else if (strcmp(argv[2], "fence") == 0) {
            sim_airframe = SIM_FENCE;
            sim_wind_e_m_s = SIM_WIND_BREEZE_E;
            console_lines = 0;
        } else if (strcmp(argv[2], "quad") != 0) {
            fprintf(stderr,
                    "usage: %s [seconds] "
                    "[quad|wing|bench|mission|fence|console|quadrth|badboard|"
                    "armnow|testarm] "
                    "[calm|noout|twomotor]\n",
                    argv[0]);
            return 2;
        }

        /* Still air, for the comparison the note above describes. Any word
         * after the session name: the mission's airframe is one of them, so
         * this looks at all of them rather than at argv[3]. */
        for (int i = 3; i < argc; i++) {
            if (strcmp(argv[i], "calm") == 0) {
                sim_wind_n_m_s = 0.0f;
                sim_wind_e_m_s = 0.0f;
            }
            /* `hold`: a mission session that does not let go of the mission,
             * so the aircraft is still holding (or circling) the last waypoint
             * at the end of a run long enough for drift to show. See the
             * mission checks. */
            if (strcmp(argv[i], "hold") == 0) {
                sim_mission_hold = 1;
            }
            /* `takeback`: the pilot ends the mission with a stick while the
             * switch stays on, which is the ending main.c has a branch of its
             * own for - the switch already asking, and the answer being "cycle
             * it" rather than a mission that starts itself again. */
            if (strcmp(argv[i], "takeback") == 0) {
                sim_mission_takeback = 1;
            }
            /* `nobaro`: the barometer stops answering partway down the
             * return. See the quadrth scenario, which is the one that sets
             * the moment. */
            if (strcmp(argv[i], "nobaro") == 0) {
                sim_baro_fail_on_descent = 1;
            }
            /* `rangefail`: the rangefinder stops answering as the return's
             * descent begins, which is the case the barometric rule has to
             * still cover. */
            if (strcmp(argv[i], "rangefail") == 0) {
                sim_range_fail_on_descent = 1;
            }
            /* `norange`: no rangefinder on this aircraft at all. The landing
             * rule then falls back to the barometer, which is what every
             * flight in this repository did before there was a part to
             * prefer. */
            if (strcmp(argv[i], "norange") == 0) {
                sim_range_fitted = 0;
            }
            /* `bare`: the board a devkit out of the bag is - no barometer, no
             * rangefinder, no pack divider, and an inertial sensor, a receiver
             * and a GPS. The claim this word exists to test is the firmware's
             * own: such a board still flies. See the quadrth checks. */
            if (strcmp(argv[i], "bare") == 0) {
                sim_baro_fitted = 0;
                sim_vbat_fitted = 0;
                sim_range_fitted = 0;
                sim_bare = 1;
            }
            /* `rxfailsafe`: the transmitter goes off and the receiver keeps
             * sending, inside its own failsafe. See STEP_YAW. */
            /* `pack`: the launch session's other ending - the pack goes
             * critical while the launch is flying the aircraft, which is the
             * failsafe the launch has to give way to. */
            if (strcmp(argv[i], "pack") == 0) {
                sim_launch_pack = 1;
            }
            /* `rcquiet`: no receiver frames at all, which is a different
             * thing from a lost link. See `refuse_step()`. */
            if (strcmp(argv[i], "rcquiet") == 0) {
                sim_rc_quiet = 1;
            }
            if (strcmp(argv[i], "rxfailsafe") == 0) {
                sim_rx_failsafe_on_link_loss = 1;
            }
            /* `badfix`: the fix degrades to four satellites partway home. */
            if (strcmp(argv[i], "badfix") == 0) {
                sim_badfix_on_return = 1;
            }
            /* `gpsjump`: one fix that is simply wrong, with every flag intact. */
            if (strcmp(argv[i], "gpsjump") == 0) {
                sim_gps_jump_on_return = 1;
            }
            /* `noisy`: the sensors as a real part leaves them. Works for any
             * session, because the point is what noise does to the flight. */
            if (strcmp(argv[i], "noisy") == 0) {
                sim_noisy = 1;
            }
            /* `gyrobias`: the part has warmed up since it was calibrated, so
             * every axis reads two degrees a second when the aircraft is
             * still. What the flight does about that is the session's
             * question: see the arm-time calibration in main.c. */
            if (strcmp(argv[i], "gyrobias") == 0) {
                sim_gyro_bias_dps = 5.0f;
            }
            /* `flatpack`: the mission is flying and the pack goes critical
             * under it. A pilot asked for the waypoints; a dead pack is a
             * reason to stop flying them. */
            if (strcmp(argv[i], "flatpack") == 0) {
                sim_mission_flatpack = 1;
            }
            /* `noout`: this board has not said what its outputs are - a port
             * whose timers never came up. `twomotor`: it drives two motors and
             * two servos, which is the ESP32-C3's shape, so a quadrotor's mix
             * does not fit it. Both are facts boot decides rather than
             * switches a running command can read, so both are words, and each
             * turns the session into the single `preflight` that reads them. */
            if (strcmp(argv[i], "noout") == 0) {
                sim_no_outputs = 1;
                /* And the rate is wrong as well, so the check that it is never
                 * asked has something to be absent about. */
                sim_lie_dshot = 1;
                console_script = console_script_badboot;
                console_lines =
                    sizeof console_script_badboot / sizeof console_script_badboot[0];
            }
            if (strcmp(argv[i], "twomotor") == 0) {
                sim_two_motors = 1;
                console_script = console_script_badboot;
                console_lines =
                    sizeof console_script_badboot / sizeof console_script_badboot[0];
            }
        }
    }
    if (run_seconds == 0u) {
        switch (sim_airframe) {
        case SIM_CONSOLE:
            /* No limit: the console is the pilot, and it says when it is done
             * by hanging up. */
            run_seconds = 0u;
            break;
        case SIM_QUAD_RTH:
            /* Long enough to fly out, come home, settle, land and hand back.
             * The descent starts over home rather than at the wing's arrival
             * radius now, so the return is a longer manoeuvre than a dive:
             * ten seconds of it are spent coming down gently over the pad,
             * and the outbound leg is flown rather than blown - see the
             * scenario, which flies at four metres a second of its own. */
            run_seconds = 100u;
            break;
        case SIM_GPS_LOST:
            /* Out, the link gone, the fix gone, and long enough after it that
             * "what did the aircraft do for the rest of the flight" has an
             * answer. */
            run_seconds = 70u;
            break;
        case SIM_GPS_LOST_LAND:
            /* The same flight, plus the wait before the descent starts and
             * twenty-something metres of it at six tenths of a metre a
             * second. */
            run_seconds = 130u;
            break;
        case SIM_BATTERY:
            /* Out, the pack flat, home, down - and long enough to see that a
             * pack that recovers does not cancel the return. */
            run_seconds = 110u;
            break;
        case SIM_RATE:
            /* Long enough for three sticks' worth of rates. */
            run_seconds = 18u;
            break;
        case SIM_NO_IMU:
            /* Long enough to fly, lose the gyro, and get it back. */
            run_seconds = 20u;
            break;
        case SIM_WING:
            run_seconds = 40u;
            break;
        case SIM_BENCH:
            /* Long enough for the six faces, the reports, and a full round of
             * the output test afterwards. */
            run_seconds = 16u;
            break;
        case SIM_ARM_NOW:
            /* Arm, be told, climb, and be asked how it is - the whole
             * scenario is over in a few seconds, and the last status wants a
             * settled aircraft rather than a rising one. */
            run_seconds = 10u;
            break;
        case SIM_TEST_ARM:
            /* The sweep starts at 700 ms, the switch is thrown at 2 s, and
             * the aircraft arms half a second after that. */
            run_seconds = 6u;
            break;
        case SIM_MISSION:
            run_seconds = 75u;
            break;
        case SIM_FENCE:
            run_seconds = 60u;
            break;
        case SIM_FAULT:
            /* Long enough to boot and print the record, and no longer: there
             * is nothing flying here. */
            run_seconds = 3u;
            break;
        case SIM_BADBOARD:
            /* Eight stages of half a second, the boot before them and the last
             * answer after; the two boot-time variants type once. */
            run_seconds = (sim_no_outputs || sim_two_motors) ? 2u : 6u;
            break;
        default:
            run_seconds = 20u;
            break;
        }
    }

    if (sim_airframe == SIM_FAULT) {
        /*
         * The board that stopped. This is the record `src/arch/arm/cortex-m4/
         * fault.c` would have left in RAM startup does not clear, planted
         * here because the point of the session is the boot *after* the
         * crash: the one that reads the record out. The numbers are the ones
         * in docs/05-bringup.md's example and the checks below look for them,
         * so a change to the format is a failing check rather than a page
         * that quietly stops matching what the firmware prints.
         */
        ak_fault.magic = AK_FAULT_MAGIC;
        ak_fault.count = 2;
        ak_fault.pc = 0x08001B42u;
        ak_fault.lr = 0x08000A55u;
        ak_fault.psr = 0x61000000u;
        ak_fault.cfsr = 0x00000082u;
        ak_fault.hfsr = 0x40000000u;
        ak_fault.mmfar = 0x00000000u;
        ak_fault.bfar = 0x00000000u;
        ak_fault.r0 = 0x11111111u;
        ak_fault.r1 = 0x22222222u;
        ak_fault.r2 = 0x33333333u;
        ak_fault.r3 = 0x44444444u;
        ak_fault.r12 = 0x55555555u;
        ak_fault.frame = 0x2001FFC0u;
    }
    return ak_firmware_main();
}
