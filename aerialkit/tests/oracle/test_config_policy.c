/*
 * B3's baseline: how a configuration change reaches the aircraft, and who is
 * allowed to make one.
 *
 * The assessment's B3 asks for one service behind every route into the
 * configuration - console, protocol, defaults, load, save - and for an
 * armed-state policy on top of it. Before any of that is written, this file
 * measures what the routes do *now*, because "the routes are not unified" is
 * an adjective and this repository does not land adjectives.
 *
 * Three claims, each measured rather than asserted:
 *
 *   1. A parameter set over the wire does not reach the aircraft. The console
 *      calls `apply_change` after every `set`; the protocol's PARAM_SET calls
 *      `ak_params_set` and stops. `parameters_changed()` in main.c is what
 *      turns a parameter into behaviour - the airframe, the whole table, the
 *      DShot timer's rate, and the receiver's protocol on both the parser and
 *      the board's line - and none of it runs for a wire change. The parameter
 *      table moves and the hardware does not, which is the worst pair for an
 *      aircraft: `params` reports the new value while the timer runs the old.
 *
 *   2. Neither route consults armed state before persisting. `ak_cli.c`'s
 *      command_save and `main.c`'s save_parameters both write the board's
 *      flash with no reference to the flight state at all, and the protocol's
 *      PARAM_SAVE handler reaches `io->save` without looking at the status it
 *      is perfectly capable of asking for.
 *
 *   3. The four change families the assessment names by hand - airframe, RC,
 *      calibration, derived waypoint - take effect through *both* routes. This
 *      is B3's second half, and it is a different question from 1: 1 asks
 *      "does the route tell the aircraft at all", and this asks "does the
 *      aircraft actually change, the same way, whichever door was used". A
 *      counter cannot answer it. Each family is driven to a real effect through
 *      a real core call and read back through that core's own getter, so a
 *      route that fired its callback and applied nothing is a failure here
 *      where it would have been a pass above.
 *
 * This began as a baseline in the same shape as
 * tests/oracle/test_estimator_oracle.c: it asserts the behaviour B3 is supposed
 * to produce, so it FAILED against the tree as it stood. It lives in
 * tests/oracle/ rather than tests/ because `TEST_OBJS` globs every C file in
 * the tests directory into the main test binary and a second main() would
 * collide with it. B3.1 and B3.2 have landed, so it is now a hard stage of
 * ci.sh (`config-policy`), which is what makes it a gate rather than a record;
 * and section 5, the four families, was added after that with the same
 * discipline - it was measured against the unrepaired routing first.
 *
 * Section 3b is milestone 4's addition, and it is here rather than in
 * tests/test_proto.c because it is the same question this file was written to
 * ask. test_proto.c proves the protocol refuses a write while armed; this file
 * proves *this device's* arms race is closed - that the gate is bound to the
 * real ak_flight_config_writable() through the real dispatch, that a refusal
 * carries a status a person can read rather than silence, and that the refusal
 * left the value alone. The gate's own logic belongs to the unit suite; the
 * wiring belongs here. Unbinding it (`io.writable = 0`) turns six checks red and
 * leaves "the wire refuses to persist while armed" green, because the save
 * mirror still catches that one - which is exactly why the two are counted
 * separately rather than one being taken for the other.
 *
 * What is real here and what is a stand-in, stated plainly, because the
 * difference matters to anyone reading the numbers:
 *
 *   - Real: ak_params, ak_cli (init, run, the whole `set` and `save` path),
 *     ak_proto (init, feed, the whole PARAM_SET and PARAM_SAVE path), and
 *     ak_flight (init, step, the arm hold, ak_flight_state, the airframe's
 *     mixer resolution). For section 5, the effect side is real too -
 *     ak_rc_receiver_set_protocol, ak_gyro_cal_set_bias_dps and
 *     ak_nav_set_waypoint are the calls main.c makes - and every one of the
 *     four families is read back through that core's own getter rather than
 *     through a variable the probe kept a copy of.
 *   - Stand-in: the board seams, of which there are now four. `config_write`
 *     and `io.save` are counters rather than flash, because this runs on a host
 *     with no config sector; `ak_board_output_set_rate` and
 *     `ak_board_rc_set_protocol` are counters rather than a timer and a UART,
 *     because a host has neither. All four count *that they were called and
 *     with what* - which is the entire question - and none adds policy of its
 *     own, so none can be the reason a change is refused or applied.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "ak_cli.h"
#include "ak_flight.h"
#include "ak_gyro_cal.h"
#include "ak_mixer.h"
#include "ak_nav.h"
#include "ak_params.h"
#include "ak_proto.h"
#include "ak_rc.h"
#include "ak_rc_receiver.h"

/* --- the measurements ---------------------------------------------------- */

static int failures;

static void expect(const char *name, int passed)
{
    printf("  %-6s %s\n", passed ? "ok" : "FAILED", name);
    if (!passed) {
        failures++;
    }
}

/* --- the board seams, and what they saw ---------------------------------- */

/* Every call to the board's config write, with the flight state at the moment
 * of the call. The state is read through a pointer the probe sets to its own
 * flight, which is how "was this written while armed?" becomes a number
 * instead of a reading of the source. */
static ak_flight_t *watched_flight;
static unsigned     cli_writes;
static int          cli_write_while_armed;
static unsigned     wire_saves;
static int          wire_save_while_armed;

/*
 * The protocol's own gate, which milestone 4 added as `ak_proto_io_t.writable`.
 *
 * This is a *third* policy site and it is not the same one as counter_wire_save
 * below. That one mirrors main.c's save_parameters - the route that reaches
 * flash - and it catches a board whose protocol has no gate at all, which is
 * every board built before milestone 4 and every device with no aircraft to be
 * armed. This one is the gate the protocol itself consults before it will even
 * look at a request, so it covers PARAM_SET and PARAM_DEFAULT as well, which
 * have no storage route to mirror.
 *
 * Both are counted rather than one being trusted: a request refused by the gate
 * never reaches the storage route, so the mirror's counter stays at zero and
 * would report "refused" for a reason it did not produce. The two counters
 * disagreeing is how that becomes visible.
 */
static unsigned wire_gate_asks;
static unsigned wire_gate_refusals;

static int wire_writable(void *ctx)
{
    (void)ctx;
    wire_gate_asks++;
    if (watched_flight == 0 || !ak_flight_config_writable(watched_flight)) {
        wire_gate_refusals++;
        return 0;
    }
    return 1;
}

static int counter_config_write(const void *buf, uint32_t len)
{
    (void)buf;
    (void)len;
    cli_writes++;
    if (watched_flight != 0 &&
        ak_flight_state(watched_flight) == AK_FLIGHT_ARMED) {
        cli_write_while_armed++;
    }
    return 0;
}

/*
 * main.c's save_parameters, mirrored. It is a mirror because main.c is the
 * firmware's entry point and is not linked into this binary - so this is the
 * one place in the probe where the code under test is copied rather than
 * called, and it is stated here rather than buried.
 *
 * The copy is faithful in the way that matters: it asks the same
 * ak_flight_config_writable() the real one asks, and it is guarded by the same
 * ak_params_save() writable argument. A mirror that had drifted would show up
 * below as a control that never fires, which is what the second control is
 * for.
 */
static int counter_wire_save(void *ctx)
{
    (void)ctx;
    if (watched_flight == 0 || !ak_flight_config_writable(watched_flight)) {
        return -1;
    }
    wire_saves++;
    if (ak_flight_state(watched_flight) == AK_FLIGHT_ARMED) {
        wire_save_while_armed++;
    }
    return 0;
}

/* --- the table, the console and the wire, over ONE params object --------- */

static ak_param_t    items[AK_PARAMS_MAX];
static ak_params_t   params;
static ak_flight_t   flight;
static ak_cli_t      cli;
static ak_proto_t    proto;
static ak_proto_io_t io;
static uint32_t      dshot_khz = 300u;

/*
 * The four families the assessment names, bound to real storage and real cores
 * exactly as main.c binds them.
 *
 * `airframe` is deliberately not declared here: it is not main.c's parameter,
 * ak_flight_param_table() registers it against flight.airframe, which is why
 * setup() calls that function rather than adding one by hand. A probe that
 * declared its own `airframe` would be measuring its own table.
 */
static uint32_t         rc_protocol = AK_RC_PROTOCOL_CRSF;
static ak_rc_receiver_t receiver;
static float            gyro_bias_dps[3];
static ak_gyro_cal_t    gyro_cal;
static float            wp_lat_deg[AK_NAV_WAYPOINTS];
static float            wp_lon_deg[AK_NAV_WAYPOINTS];
static uint32_t         wp_count;
static ak_nav_t         nav;

/* --- the two board seams section 5 adds ---------------------------------- */

/*
 * ak_board_output_set_rate and ak_board_rc_set_protocol are the two effects
 * main.c's parameters_changed() performs that a host cannot: one needs a timer,
 * the other a UART. Counting them is the same trade as config_write above, and
 * it is what keeps the DShot rate and the receiver's *line* settings in the
 * measurement at all - without them, "takes effect identically" would quietly
 * come to mean "on the parts a host can run".
 */
static unsigned output_rate_sets;
static uint32_t output_rate_last;
static unsigned board_rc_sets;
static uint32_t board_rc_last;

/* --- the change callback, which is the thing under test ------------------ */

static unsigned applies;

/*
 * main.c's parameters_changed(), mirrored.
 *
 * It is a mirror for the same reason counter_wire_save is: main.c is the
 * firmware's entry point, is not linked into this binary, and the function is
 * static. What matters is that the *effects* are the real ones - every call
 * below is a call main.c makes, against the same core - so section 5's
 * assertions are about the aircraft changing rather than about a callback
 * counter incrementing.
 *
 * The counter stays because section 1 needs it. "The wire told the aircraft" is
 * a question about the callback being reached at all, and an effect-based
 * assertion cannot tell "never reached" from "reached and had no effect" - the
 * first is the B3 defect and the second would be a different one.
 */
static void apply_like_main(void)
{
    applies++;

    ak_flight_apply_airframe(&flight);

    /* The receiver, both halves: the parser here, the board's line below. */
    ak_rc_receiver_set_protocol(&receiver, rc_protocol);
    board_rc_sets++;
    board_rc_last = rc_protocol;

    ak_gyro_cal_set_bias_dps(&gyro_cal, gyro_bias_dps);

    /* Degrees in the table, 1e-7 units in the navigator - main.c's conversion,
     * and here for main.c's reason: the GPS speaks 1e-7 and a round trip
     * through degrees is a metre nobody needs to lose. */
    for (int i = 0; i < AK_NAV_WAYPOINTS; i++) {
        ak_nav_set_waypoint(&nav, i, (int32_t)(wp_lat_deg[i] * 10000000.0f),
                            (int32_t)(wp_lon_deg[i] * 10000000.0f));
    }
    ak_nav_set_waypoint_count(&nav, (int)wp_count);

    output_rate_sets++;
    output_rate_last = dshot_khz;
}

/*
 * The console's screen. The CLI prints on the paths this file measures - a
 * `save` that worked says "saved N bytes", and the refusal B3 adds will say
 * why - so a null sink would not just lose the evidence, it would crash on it.
 * The first few lines are shown indented with the report; the rest are counted
 * so a runaway path cannot bury the numbers.
 */
static unsigned cli_lines;

static int cli_out(const char *fmt, ...)
{
    va_list args;

    if (cli_lines < 6u) {
        printf("      console | ");
        va_start(args, fmt);
        (void)vprintf(fmt, args);
        va_end(args);
    }
    cli_lines++;
    return 0;
}

static void fill_status(void *ctx, ak_proto_status_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    /* The protocol *is* told the flight state - this callback is required and
     * the real main.c fills it from the same ak_flight_state() the console
     * would ask. Whether the save path looks at it is the question. */
    out->flight_state = (uint8_t)ak_flight_state(&flight);
    out->link_live = 1u;
}

static void setup(void)
{
    unsigned n = 0;

    /* The flight core first, because `airframe` is one of *its* parameters and
     * is bound to flight.airframe by this call. The core's own comment asks for
     * it straight after init: whatever the fields hold then is what `defaults`
     * puts back. */
    ak_flight_init(&flight, &ak_mixer_quad_x);
    n = ak_flight_param_table(&flight, items, AK_PARAMS_MAX);

    n = ak_params_add_u32(items, n, "dshot_khz", "the output protocol's rate",
                          &dshot_khz, 150u, 600u, AK_PARAM_GROUP_OUTPUTS);
    n = ak_params_add_u32(items, n, "rc_protocol", "0 crsf, 1 sbus",
                          &rc_protocol, 0u, 1u, AK_PARAM_GROUP_RECEIVER);
    n = ak_params_add_float(items, n, "gyro_bias_roll", "deg/s",
                            &gyro_bias_dps[0], 3, -20.0f, 20.0f, AK_PARAM_GROUP_SENSORS);
    n = ak_params_add_float(items, n, "gyro_bias_pitch", "deg/s",
                            &gyro_bias_dps[1], 3, -20.0f, 20.0f, AK_PARAM_GROUP_SENSORS);
    n = ak_params_add_float(items, n, "gyro_bias_yaw", "deg/s",
                            &gyro_bias_dps[2], 3, -20.0f, 20.0f, AK_PARAM_GROUP_SENSORS);
    /* NAVIGATION, because that is what the board files this under: main.c
     * registers the mission's waypoints in a loop whose group is the mission's.
     * This file is a model of the real table, so a heading that disagreed with
     * the board's would be a second answer to the same question - which is the
     * exact failure the group field was added to remove. */
    n = ak_params_add_float(items, n, "wp_lat_1", "degrees",
                            &wp_lat_deg[0], 6, -90.0f, 90.0f,
                            AK_PARAM_GROUP_NAVIGATION);
    n = ak_params_add_u32(items, n, "wp_count", "waypoints in the mission",
                          &wp_count, 0u, AK_NAV_WAYPOINTS, AK_PARAM_GROUP_NAVIGATION);
    ak_params_init(&params, items, n);

    ak_flight_set_board_outputs(&flight, 4u, 2u);
    watched_flight = &flight;

    ak_rc_receiver_init(&receiver);
    ak_gyro_cal_init(&gyro_cal, 400u, 5.0f, 20.0f);
    ak_nav_init(&nav);

    ak_cli_io_t cli_io;
    memset(&cli_io, 0, sizeof cli_io);
    cli_io.out = cli_out;
    cli_io.config_write = counter_config_write;
    cli_io.on_change = apply_like_main;
    ak_cli_init(&cli, &cli_io, &params, &flight);

    memset(&io, 0, sizeof io);
    io.params = &params;
    io.status = fill_status;
    /* The same callback the console gets. main.c wires this from the same
     * `parameters_changed` (see its proto_io initialisation); a probe that
     * left it null would be measuring a device that has nothing to re-apply,
     * which is not the device under test. */
    io.on_change = apply_like_main;
    io.save = counter_wire_save;
    /* main.c's proto_writable(), mirrored for the same reason counter_wire_save
     * is: main.c is not linked here. It asks the same
     * ak_flight_config_writable() the real one asks. */
    io.writable = wire_writable;
    io.ctx = 0;
    ak_proto_init(&proto);
}

/* Console `set`, through the real CLI. */
static void console_set(const char *line)
{
    (void)ak_cli_run(&cli, line);
}

/*
 * The last response this file received, and how long it was.
 *
 * Kept rather than discarded because milestone 4 made the *status byte* the
 * answer to the question this probe asks: a refusal that is indistinguishable
 * from silence would let a person keep clicking a button that does nothing.
 * The payload begins at byte 5 - after sync, sync, version and the command with
 * its response bit - which is the firmware's own frame layout, not a guess.
 */
static uint8_t  wire_response[AK_PROTO_FRAME_MAX];
static unsigned wire_response_len;

static int wire_response_status(void)
{
    return wire_response_len >= 6u ? (int)wire_response[5] : -1;
}

/* One protocol request, byte by byte, through the real parser. */
static void wire_command(uint8_t command, const uint8_t *args, unsigned n)
{
    uint8_t frame[AK_PROTO_FRAME_MAX];
    uint8_t *response = wire_response;
    unsigned at = 5;

    wire_response_len = 0;

    frame[0] = AK_PROTO_SYNC1;
    frame[1] = AK_PROTO_SYNC2;
    frame[2] = AK_PROTO_VERSION;
    frame[3] = command;
    for (unsigned i = 0; i < n; i++) {
        frame[at++] = args[i];
    }
    frame[4] = (uint8_t)(at - 5u);
    uint16_t crc = ak_proto_crc16(&frame[2], at - 2u);
    frame[at++] = (uint8_t)(crc & 0xFFu);
    frame[at++] = (uint8_t)(crc >> 8);

    for (unsigned i = 0; i < at; i++) {
        unsigned wrote = ak_proto_feed(&proto, &io, frame[i], 1000u, response,
                                       sizeof wire_response);
        if (wrote > 0u) {
            wire_response_len = wrote;
        }
    }
}

/*
 * The wire addresses a parameter by its index in the table, not by name, so
 * section 5's four families are reached by looking each name up. Looking it up
 * rather than hard-coding an index matters: an index copied from a listing is a
 * number that silently starts meaning a different parameter the next time the
 * table grows, and the probe would go on measuring something.
 */
static int param_index(const char *name)
{
    ak_param_t *found = ak_params_find(&params, name);

    return found == 0 ? -1 : (int)(found - items);
}

static void wire_set_named(const char *name, const char *value)
{
    uint8_t args[2 + 24];
    int index = param_index(name);
    unsigned len = (unsigned)strlen(value);

    if (index < 0 || len > sizeof args - 2u) {
        printf("      probe  | no usable parameter named %s\n", name);
        return;
    }
    args[0] = (uint8_t)index;
    for (unsigned i = 0; i < len; i++) {
        args[1 + i] = (uint8_t)value[i];
    }
    wire_command(AK_PROTO_CMD_PARAM_SET, args, 1u + len);
}

static void wire_set(const char *value)
{
    wire_set_named("dshot_khz", value);
}

/* --- arming -------------------------------------------------------------- */

/*
 * Level, still, arm channel high, long enough for the hold. The same shape
 * test_aerialkit.c uses; the builders are local because the ones there are
 * static to that file.
 */
static void arm_the_aircraft(void)
{
    ak_rc_config_t cfg;
    ak_rc_default_config(&cfg);

    for (uint32_t t = 1; t <= 900u; t++) {
        ak_imu_sample_t imu;
        ak_rc_input_t rc;

        imu.gyro[0] = 0.0f; imu.gyro[1] = 0.0f; imu.gyro[2] = 0.0f;
        imu.accel[0] = 0.0f; imu.accel[1] = 0.0f; imu.accel[2] = 1.0f;
        imu.time_ms = t;
        imu.valid = 1;

        for (int i = 0; i < AK_RC_CHANNELS; i++) {
            rc.channel[i] = (uint16_t)cfg.mid;
        }
        rc.channel[AK_RC_THROTTLE] = (uint16_t)cfg.min;
        rc.channel[AK_RC_ARM] = (uint16_t)cfg.max;
        rc.channel[AK_RC_MODE] = (uint16_t)cfg.max;
        rc.last_update_ms = t;
        rc.valid = 1;

        ak_flight_step(&flight, &imu, &rc, t);
    }
}

/* --- the probe ----------------------------------------------------------- */

int main(void)
{
    printf("B3 baseline: the configuration routes, and the armed-state policy\n");

    /* 1. The console applies. This half has to work, or the second half is
     *    measuring a command that never did anything in the first place. */
    setup();
    console_set("set dshot_khz 600");
    printf("\nconsole `set dshot_khz 600`\n");
    printf("  value now        %u\n", (unsigned)dshot_khz);
    printf("  apply callback   %u call(s)\n", applies);
    expect("the console's set changed the parameter", dshot_khz == 600u);
    expect("and the console told the aircraft", applies == 1u);

    /* 2. The wire does not. Both halves are asserted: that the table moved,
     *    and that the aircraft was not told. The first alone would be
     *    satisfied by a build where the command works; the second alone would
     *    also be satisfied by one where it fails. Only together do they say
     *    "this took effect everywhere except the aeroplane". */
    unsigned before = applies;
    wire_set("450");
    printf("\nwire PARAM_SET dshot_khz=450\n");
    printf("  value now        %u\n", (unsigned)dshot_khz);
    printf("  apply callback   %u call(s), was %u\n", applies, before);
    expect("the wire's set changed the parameter", dshot_khz == 450u);
    expect("and the wire told the aircraft", applies > before);

    /* 3. Neither route refuses a save while armed. */
    arm_the_aircraft();
    printf("\narmed state        %s\n", ak_flight_state_name(ak_flight_state(&flight)));
    expect("the aircraft is armed for this measurement",
           ak_flight_state(&flight) == AK_FLIGHT_ARMED);

    console_set("save");
    printf("\nconsole `save` while armed\n");
    printf("  board writes     %u (%u while armed)\n", cli_writes,
           cli_write_while_armed);
    expect("the console refuses to persist while armed", cli_writes == 0u);

    /* What the protocol was told, at the moment it decided to save. The
     * handler is handed a callback that answers exactly this and does not call
     * it, which is the difference between "could not know" and "did not ask". */
    ak_proto_status_t seen;
    fill_status(0, &seen);
    wire_command(AK_PROTO_CMD_PARAM_SAVE, 0, 0);
    printf("\nwire PARAM_SAVE while armed\n");
    printf("  status offered   flight_state %u (%s)\n", (unsigned)seen.flight_state,
           ak_flight_state_name((ak_flight_state_t)seen.flight_state));
    printf("  board saves      %u (%u while armed)\n", wire_saves,
           wire_save_while_armed);
    printf("  gate asked        %u (%u refusal(s)), status %d\n", wire_gate_asks,
           wire_gate_refusals, wire_response_status());
    expect("the wire refuses to persist while armed", wire_saves == 0u);

    /*
     * 3b. The gate milestone 4 added, measured on the two commands the storage
     *     route cannot cover.
     *
     *     The check above is satisfied by the *mirror* in counter_wire_save,
     *     which is the route to flash. PARAM_SET has no storage route at all -
     *     it moves the running table - so nothing before milestone 4 stood
     *     between an armed aircraft and a wire write. These three assertions are
     *     what that gap closing looks like from the wire: the request is
     *     refused, the refusal is *stated* rather than silent, and the value did
     *     not move, because a refused write that half-landed would be worse than
     *     one that was allowed.
     *
     *     The gate is also asserted to have been *asked*. "No write happened"
     *     and "a request never parsed" are the same number, and the second is
     *     the failure this file has to be able to tell from the first.
     */
    uint32_t armed_dshot = dshot_khz;
    unsigned asks_before = wire_gate_asks;
    wire_set("600");
    printf("\nwire PARAM_SET while armed\n");
    printf("  value now        %u (was %u)\n", (unsigned)dshot_khz,
           (unsigned)armed_dshot);
    printf("  gate asked        %u (%u refusal(s)), status %d\n", wire_gate_asks,
           wire_gate_refusals, wire_response_status());
    expect("the gate was asked before the set was answered",
           wire_gate_asks > asks_before);
    expect("the wire refuses a set while armed, and says so",
           wire_response_status() == AK_PROTO_WRITE_REFUSED_ARMED);
    expect("and the refused set left the value alone", dshot_khz == armed_dshot);

    /* The reset, which rewrites the whole table and is the largest write this
     * wire can ask for. It must be gated for the same reason a set is: a gate
     * that covered the small write and not the sweeping one would be a gate in
     * name only. */
    console_set("set dshot_khz 600");
    uint32_t before_reset = dshot_khz;
    wire_command(AK_PROTO_CMD_PARAM_DEFAULT, (const uint8_t *)"\x02", 1u);
    printf("\nwire PARAM_DEFAULT (all) while armed\n");
    printf("  value now        %u (was %u)\n", (unsigned)dshot_khz,
           (unsigned)before_reset);
    printf("  status           %d\n", wire_response_status());
    expect("the wire refuses a reset while armed, and says so",
           wire_response_status() == AK_PROTO_WRITE_REFUSED_ARMED);
    expect("and nothing was put back", dshot_khz == before_reset);

    /* 4. The control, and the reason this file is worth reading without it.
     *
     *    Every check above is of the form "this did not happen". A counter
     *    that never fires - a seam that was never wired, a command that never
     *    parsed, an aircraft that was never really armed - satisfies all of
     *    them and measures nothing. So the same save, disarmed, has to reach
     *    the board. This is the estimator oracle's control in a different
     *    costume: on its own, "2047 checks, 0 failed" cannot tell the baseline
     *    tree from the repaired one, and on their own these zeros cannot tell
     *    a policy from a dead counter.
     */
    cli_writes = 0;
    cli_write_while_armed = 0;
    setup();                        /* a fresh flight, never armed */
    console_set("save");
    printf("\ncontrol: console `save` while disarmed\n");
    printf("  state            %s\n",
           ak_flight_state_name(ak_flight_state(&flight)));
    printf("  board writes     %u (%u while armed)\n", cli_writes,
           cli_write_while_armed);
    expect("disarmed, the same save does reach the board", cli_writes == 1u);
    expect("and the control was not armed either", cli_write_while_armed == 0u);

    /* And the same control for the wire, because the wire's seam is the one
     * this file mirrors rather than calls - a mirror that had drifted would
     * leave the row above passing for the wrong reason. */
    wire_saves = 0;
    wire_save_while_armed = 0;
    wire_command(AK_PROTO_CMD_PARAM_SAVE, 0, 0);
    printf("\ncontrol: wire PARAM_SAVE while disarmed\n");
    printf("  board saves      %u (%u while armed)\n", wire_saves,
           wire_save_while_armed);
    expect("disarmed, the wire's save does reach the board", wire_saves == 1u);

    /* And the control for the gate itself, for the same reason and against the
     * same failure. Every gate check above is "this did not happen"; a gate that
     * answered 0 to everything - a seam wired backwards, a flight object that
     * was never disarmed - satisfies all of them and protects nothing. So the
     * identical set, on this fresh disarmed aircraft, has to land, and the gate
     * has to have been asked on the way. */
    wire_gate_asks = 0;
    wire_gate_refusals = 0;
    wire_set("450");
    printf("\ncontrol: wire PARAM_SET while disarmed\n");
    printf("  value now        %u\n", (unsigned)dshot_khz);
    printf("  gate asked        %u (%u refusal(s)), status %d\n", wire_gate_asks,
           wire_gate_refusals, wire_response_status());
    expect("disarmed, the same set is accepted", dshot_khz == 450u);
    expect("and the gate allowed it rather than never being asked",
           wire_gate_asks == 1u && wire_gate_refusals == 0u);

    /*
     * 5. The four families the assessment names by hand - airframe, RC,
     *    calibration, derived waypoint - taking effect through *both* doors.
     *
     *    This is a different question from section 1. Section 1 asked whether a
     *    route tells the aircraft at all and answered with a counter. Here a
     *    counter would be satisfied by a callback that applied nothing, so the
     *    assertion is the aircraft's own state, read through the core's own
     *    getter: the mixer the flight core resolved, the protocol the receiver
     *    holds, the bias the calibration carries, the waypoint the navigator
     *    would fly to.
     *
     *    Two different values, one per door, which is what makes the second
     *    measurement evidence rather than repetition: a dead wire leaves the
     *    console's value in place and fails, and a live one leaves its own. The
     *    pairs are asserted distinct at the end, once, rather than assumed four
     *    times - two equal values would make all four wire assertions pass on a
     *    door that never opened.
     *
     *    All four are measured disarmed, which is the state the assessment asks
     *    about and the state a bench session is in.
     */
    setup();                        /* a fresh flight, never armed */
    printf("\nthe four families, disarmed: console then wire, a different value each\n");

    console_set("set airframe 1");
    unsigned console_airframe = flight.airframe;
    const ak_mixer_t *console_mixer = flight.mixer;
    printf("\nconsole `set airframe 1`\n");
    printf("  flight.airframe  %u\n", (unsigned)console_airframe);
    printf("  mixer            %s\n", console_mixer->name);
    expect("the console changed the airframe the core flies",
           console_airframe == 1u && console_mixer == ak_mixer_for_airframe(1u));

    wire_set_named("airframe", "0");
    printf("wire PARAM_SET airframe=0\n");
    printf("  flight.airframe  %u\n", (unsigned)flight.airframe);
    printf("  mixer            %s\n", flight.mixer->name);
    expect("and the wire changed it too, back to the quad",
           flight.airframe == 0u && flight.mixer == ak_mixer_for_airframe(0u));
    /*
     * This one is deliberately about the two *values* and not about the mix,
     * and it is the one assertion here that passed against the unrepaired wire.
     * That is the whole argument for the assertions above it: `airframe` is
     * bound straight to flight.airframe, so a wire write moves the parameter
     * whether or not anything re-applies it - `flight.airframe == 0u` alone
     * would have been green on the tree where the aircraft was still flying the
     * wing's mix. Only the mixer says whether the aircraft changed. The three
     * families below keep their own copy inside their core, so for them the
     * value check fails too, and the effect check is the second opinion rather
     * than the only one.
     */
    expect("the two airframe values were different ones",
           console_airframe != flight.airframe);

    console_set("set rc_protocol 1");
    unsigned console_rc = ak_rc_receiver_protocol(&receiver);
    unsigned console_line = board_rc_last;
    printf("\nconsole `set rc_protocol 1`\n");
    printf("  receiver holds   %u\n", console_rc);
    printf("  board's line     %u\n", console_line);
    expect("the console changed the protocol the parser speaks",
           console_rc == AK_RC_PROTOCOL_SBUS);
    expect("and the board's line settings with it", console_line == 1u);

    wire_set_named("rc_protocol", "0");
    printf("wire PARAM_SET rc_protocol=0\n");
    printf("  receiver holds   %u\n", (unsigned)ak_rc_receiver_protocol(&receiver));
    printf("  board's line     %u\n", (unsigned)board_rc_last);
    expect("and the wire changed both halves too",
           ak_rc_receiver_protocol(&receiver) == AK_RC_PROTOCOL_CRSF &&
           board_rc_last == 0u);
    expect("the two RC values were different ones",
           console_rc != ak_rc_receiver_protocol(&receiver));

    float bias[3];

    console_set("set gyro_bias_roll 0.5");
    ak_gyro_cal_bias_dps(&gyro_cal, bias);
    float console_bias = bias[0];
    printf("\nconsole `set gyro_bias_roll 0.5`\n");
    printf("  gyro bias roll   %.3f\n", (double)console_bias);
    expect("the console changed the bias the calibration applies",
           fabsf(console_bias - 0.5f) < 1e-4f);

    wire_set_named("gyro_bias_roll", "-0.25");
    ak_gyro_cal_bias_dps(&gyro_cal, bias);
    printf("wire PARAM_SET gyro_bias_roll=-0.25\n");
    printf("  gyro bias roll   %.3f\n", (double)bias[0]);
    expect("and the wire changed it too", fabsf(bias[0] + 0.25f) < 1e-4f);
    expect("the two bias values were different ones",
           fabsf(console_bias - bias[0]) > 1e-4f);

    console_set("set wp_lat_1 47.5");
    int32_t console_wp = nav.waypoint_lat_e7[0];
    printf("\nconsole `set wp_lat_1 47.5`\n");
    printf("  nav waypoint 1   %d (1e-7 deg)\n", console_wp);
    expect("the console changed the waypoint the navigator would fly to",
           console_wp == 475000000);

    wire_set_named("wp_lat_1", "48.25");
    printf("wire PARAM_SET wp_lat_1=48.25\n");
    printf("  nav waypoint 1   %d (1e-7 deg)\n", nav.waypoint_lat_e7[0]);
    expect("and the wire changed it too", nav.waypoint_lat_e7[0] == 482500000);
    expect("the two waypoint values were different ones",
           console_wp != nav.waypoint_lat_e7[0]);

    printf("\n%d measurement(s) failed\n", failures);
    printf("%s\n", failures == 0
           ? "every configuration route applies, and none persists in the air"
           : "a route is not reaching the aircraft, or one is persisting armed");
    return failures == 0 ? 0 : 1;
}
