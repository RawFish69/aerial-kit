#include "ak_cli.h"

#include "ak_board.h"
#include "ak_fault.h"
#include "ak_text.h"
#include "ak_version.h"

#define AK_CLI_MAX_ARGS 4

/*
 * The prompt.
 *
 * Two things need it. A person: without it, a board that is listening and a
 * board that has stopped look exactly alike, and the first thing anybody does
 * on a new board is type at it. And a script: it is the only thing the firmware
 * prints that means "the last command has finished and the console is idle",
 * which is what tools/bench_check.py watches for when it drives a real board
 * (docs/05-bringup.md has shown the prompt in its transcripts all along, which
 * is how the omission was noticed).
 */
void ak_cli_prompt(ak_cli_t *cli)
{
    cli->io.out("ak> ");
}

void ak_cli_forget(ak_cli_t *cli)
{
    /* Only if there is something to forget. The common case is a port with no
     * half-typed line on it - a client that only ever speaks the protocol, a
     * person who pressed return - and printing a newline at every binary byte
     * would put a break in the middle of whatever the console was saying. */
    if (cli->len > 0u || cli->overflow != 0u) {
        cli->io.out("\r\n");
    }
    cli->len = 0;
    cli->overflow = 0;
}

void ak_cli_init(ak_cli_t *cli, const ak_cli_io_t *io, ak_params_t *params,
                 ak_flight_t *flight)
{
    cli->io = *io;
    cli->params = params;
    cli->flight = flight;
    cli->len = 0;
    cli->overflow = 0;
    cli->commands = 0;
    cli->line[0] = '\0';
}

/* A parameter has moved: let the board act on it. The console does not know
 * what any parameter means, which is why this is a callback rather than a list
 * of things to call. */
static void apply_change(ak_cli_t *cli)
{
    if (cli->io.on_change != 0) {
        cli->io.on_change();
    }
}

static void print_help(ak_cli_t *cli)
{
    cli->io.out("commands:\n");
    cli->io.out("  help                 this\n");
    cli->io.out("  banner               the boot banner, again - it is printed\n");
    cli->io.out("                       before a host can attach, so this is the\n");
    cli->io.out("                       only way to read it back\n");
    cli->io.out("  version              firmware, board, revision, build stamp\n");
    cli->io.out("  status               state, uptime, attitude, outputs\n");
    cli->io.out("  params               every parameter and its current value\n");
    cli->io.out("  get <name>           one parameter\n");
    cli->io.out("  set <name> <value>   change one (range-checked, not clamped)\n");
    cli->io.out("  defaults             back to the built-in values\n");
    cli->io.out("  save                 write the parameters to the board\n");
    cli->io.out("  load                 read them back\n");
    cli->io.out("  clear                wipe the screen\n");
    cli->io.out("  reboot               reset the board\n");
    cli->io.out("  dfu                  hand the part to its ROM bootloader (reflash\n");
    cli->io.out("                       without a jumper), where the board has one\n");
    cli->io.out("  output               motors, servos, timers, frame counts\n");
    cli->io.out("  output test [stop]   walk each output so a scope can see it\n");
    cli->io.out("  rc                   receiver counters, raw channels, sticks\n");
    cli->io.out("  imu                  the inertial sensor, if there is one\n");
    cli->io.out("  baro                 pressure, temperature and height above the bench\n");
    cli->io.out("  range                how far the ground is, if a part is fitted\n");
    cli->io.out("  battery              the flight pack, and the pin it is measured on\n");
    cli->io.out("  spi                  sensor bus loopback (jumper MOSI to MISO)\n");
    cli->io.out("  calibrate [gyro]     gyro bias, aircraft still and disarmed\n");
    cli->io.out("  calibrate rc         receiver centres, sticks centred\n");
    cli->io.out("  calibrate vbat <V>   the pack's divider, against a multimeter\n");
    cli->io.out("  calibrate accel <0-5>  accelerometer, one face per command:\n");
    cli->io.out("                       0 level, 1 inverted, 2 nose down,\n");
    cli->io.out("                       3 nose up, 4 right down, 5 left down\n");
    cli->io.out("  mission [list]       the waypoints and whether they are flown\n");
    cli->io.out("  mission add LAT LON  add one, in degrees\n");
    cli->io.out("  mission start|stop   fly them, or give the aircraft back\n");
    cli->io.out("  log [reset]          dump the blackbox, or empty it\n");
    cli->io.out("  log long [clear]     the long log (survives a reset)\n");
    cli->io.out("  log flash [clear]    the log in flash (survives the battery)\n");
    cli->io.out("  gps                  position, speed, satellites\n");
    cli->io.out("  home [clear]         remember this spot, or forget it\n");
    cli->io.out("  preflight            check the board against what the firmware believes\n");
    cli->io.out("  proto                config-protocol counters (the binary port)\n");
    cli->io.out("\n");
    cli->io.out("Everything here has been tested on a host and in the simulator.\n");
    cli->io.out("None of it has been on an aircraft: nothing has turned a motor,\n");
    cli->io.out("moved a servo, or read a real sensor. See docs/05-bringup.md.\n");
}

void ak_banner_print(ak_printf_fn out)
{
    out("\r\n");
    out("AerialKit Firmware\r\n");
    out("  product:  %s\r\n", AK_PRODUCT_STR);
    out("  board:    %s\r\n", ak_board_name());
    out("  rev:      %s\r\n", AK_REV_STR);
    out("  built:    %s\r\n", AK_STAMP_STR);
    out("  clocks:   %s\r\n", ak_board_clock_summary());
    out("  state:    no sensor drivers - the flight loop runs in failsafe\r\n");
    out("\r\n");
}

static void print_version(ak_cli_t *cli)
{
    cli->io.out("product:  %s\n", AK_PRODUCT_STR);
    cli->io.out("board:    %s\n", ak_board_name());
    cli->io.out("rev:      %s\n", AK_REV_STR);
    cli->io.out("built:    %s\n", AK_STAMP_STR);
}

static void print_status(ak_cli_t *cli)
{
    ak_flight_t *flight = cli->flight;
    ak_estimator_t *est = &flight->est;
    const ak_outputs_t *out = ak_flight_outputs(flight);

    cli->io.out("state:     %s\n",
                ak_flight_state_name(ak_flight_state(flight)));
    if (cli->io.now_ms != 0) {
        cli->io.out("uptime:    %u ms\n", cli->io.now_ms());
    }
    cli->io.out("loops:     %u\n", flight->steps);
    /*
     * What the loop's own timing did, which is the half of B4 that makes the
     * other half worth having.
     *
     * These four numbers are the answer to "why did it fly badly", and before
     * them there was no answer at all: a bus that stalls every few hundred
     * milliseconds, a gyro that has stopped advancing its clock, and a loop
     * that is simply being called too slowly all produce the same symptom from
     * the pilot's seat and were indistinguishable from the ground. They are
     * printed on one line because they are read together - the first two count
     * sensor intervals the loop had to catch up on, the next two count time it
     * could not use, and `loops` above is what makes them percentages.
     */
    cli->io.out("timing:    %u gaps, %u catchup pieces, %u dropped ms, "
                "%u unusable\n",
                flight->timing.gap_steps, flight->timing.catchup_steps,
                flight->timing.dropped_ms, flight->timing.unusable);
    cli->io.out("timing:    %u duplicate samples, %u clock resets, "
                "%u long loops, longest %u ms\n",
                flight->timing.duplicates, flight->timing.clock_resets,
                flight->timing.long_loops, flight->timing.max_loop_ms);
    if (cli->io.link_report != 0) {
        cli->io.link_report(cli->io.out);
    }
    cli->io.out("attitude:  roll %d mrad, pitch %d mrad, yaw %d mrad\n",
                (int)(est->roll * 1000.0f), (int)(est->pitch * 1000.0f),
                (int)(est->yaw * 1000.0f));
    cli->io.out("attitude:  %s\n", est->converged ? "converged" : "not converged");
    cli->io.out("motors:    %d %d %d %d per-mille\n",
                (int)(out->motor[0] * 1000.0f), (int)(out->motor[1] * 1000.0f),
                (int)(out->motor[2] * 1000.0f), (int)(out->motor[3] * 1000.0f));
    cli->io.out("servos:    %d %d per-mille\n", (int)(out->servo[0] * 1000.0f),
                (int)(out->servo[1] * 1000.0f));
    /* Why it is not armed, while it is not armed: asked before a pilot throws
     * a switch rather than after, and the same answer the switch would get. */
    if (ak_flight_state(flight) == AK_FLIGHT_DISARMED &&
        cli->io.arm_report != 0) {
        cli->io.arm_report(cli->io.out);
    }
    cli->io.out("params:    %u changed since the last save\n", cli->params->changed);
    /*
     * Which configuration this is, as opposed to how far it has moved since the
     * last save. The line above answers "has anything changed"; this one answers
     * "changed from what", and it is the one an experiment result has to be
     * filed under - two aircraft with different gains both report zero above.
     *
     * It is over the effective table, so it is the same number after a fresh
     * boot from the same record, and it does not move when only a secret
     * changes: see ak_params_hash for why that trade is the right way round.
     */
    cli->io.out("identity:  0x%08x\n", ak_params_hash(cli->params));
    /* What the configuration on the board did to *this* build's table, which
     * is the question after a flash: a parameter that was removed, and one
     * that has just been added, are both invisible otherwise. */
    if (cli->params->load_have_report) {
        ak_params_load_report_t report;

        ak_params_load_report(cli->params, &report);
        cli->io.out("config:    %u of %u parameters from the saved record",
                    report.applied, report.total);
        if (report.unknown > 0u) {
            cli->io.out(", %u gone (%s)", report.unknown, report.unknown_name);
        }
        if (report.unmentioned > 0u) {
            cli->io.out(", %u new (%s)", report.unmentioned,
                        report.unmentioned_name);
        }
        cli->io.out("\n");
    }
    if (ak_fault_present()) {
        cli->io.out("fault:     %u recorded, pc 0x%08x cfsr 0x%08x\n",
                    ak_fault.count, ak_fault.pc, ak_fault.cfsr);
    } else {
        cli->io.out("fault:     none recorded\n");
    }
}

static int command_save(ak_cli_t *cli)
{
    /*
     * A save is a bench command, and the erase inside it is a stall.
     *
     * The F405's configuration lives in one 128 KB flash sector: erasing it
     * stalls every instruction fetch for about a second, because the code is in
     * the same bank. A second in the air is not a slow loop - the DShot frames
     * stop, the ESCs' own failsafe times out, and a quadrotor comes down. The
     * four calibrations have refused to run on an armed aircraft since the bench
     * session that found the accelerometer's missing its guard; this is the same
     * rule for the command that writes flash, and it was missing for the same
     * reason: everybody who typed it was on a bench.
     */
    if (cli->io.disarmed != 0 && !cli->io.disarmed()) {
        cli->io.out("save: refusing - this writes flash, and an armed aircraft "
                    "is not a bench\n");
        return 1;
    }
    if (cli->io.config_write == 0) {
        cli->io.out("no configuration storage on this board\n");
        return 1;
    }
    /* One call, and the same one the protocol's save goes through. This
     * function used to hold its own copy of the sequence - store, bounds,
     * write, mark - and so did main.c's save_parameters. Two copies of a
     * safety rule is one copy too many; see ak_params_save(). */
    int length = ak_params_save(cli->params, cli->config, sizeof cli->config,
                                cli->io.config_write,
                                ak_flight_config_writable(cli->flight));
    /* Not while it is flying. The answer comes from the flight core and is
     * passed in, so this route cannot persist without having asked - and the
     * reason is on ak_flight_config_writable. */
    if (length == AK_PARAMS_SAVE_BLOCKED) {
        cli->io.out("save: refused while %s - the aircraft is not on the "
                    "bench\n", ak_flight_state_name(ak_flight_state(cli->flight)));
        return 1;
    }
    /* A table that does not fit the record is refused rather than saved in
     * part: loading a prefix back would leave every parameter after the cut
     * holding whatever the aircraft happens to have. */
    if (length == AK_PARAMS_SAVE_TOO_BIG) {
        cli->io.out("save: the table does not fit the %u-byte configuration "
                    "record (AK_PARAMS_TEXT_MAX)\n",
                    (unsigned)sizeof cli->config);
        return 1;
    }
    if (length < 0) {
        cli->io.out("save: the board refused the write\n");
        return 1;
    }
    cli->io.out("saved %d bytes\n", length);
    return 0;
}

/* Everything after the first two words of the line - which is where the value
 * of `set <name> <value>` starts, spaces and all. */
static const char *value_after_name(const char *line, const char *name)
{
    (void)name;

    const char *p = ak_skip_spaces(line);
    for (int word = 0; word < 2; word++) {
        if (*p == '\0') {
            return 0;
        }
        while (*p != '\0' && *p != ' ') {
            p++;
        }
        p = ak_skip_spaces(p);
    }
    return *p == '\0' ? 0 : p;
}

static int command_load(ak_cli_t *cli)
{
    if (cli->io.config_read == 0) {
        cli->io.out("no configuration storage on this board\n");
        return 1;
    }
    int length = cli->io.config_read(cli->config, sizeof cli->config - 1u);
    if (length <= 0) {
        cli->io.out("no saved configuration\n");
        return 1;
    }
    cli->config[length] = '\0';

    char msg[64];
    msg[0] = '\0';
    if (ak_params_deserialize(cli->params, cli->config, msg, sizeof msg) != 0) {
        cli->io.out("load failed: %s\n", msg);
        return 1;
    }
    ak_params_mark_saved(cli->params);
    apply_change(cli);
    /*
     * What it did, not just that it happened. The record was written by a
     * build that may have had a different table - that is what an upgrade is -
     * and until this was printed, a parameter that no longer existed was
     * skipped without a word and one that had just been added silently kept
     * its compiled-in value. Both are worth a line, and the *names* are the
     * important half: a count says something happened.
     */
    ak_params_load_report_t report;
    ak_params_load_report(cli->params, &report);
    cli->io.out("loaded %d bytes: %u of %u parameters", length, report.applied,
                report.total);
    if (report.unknown > 0u) {
        cli->io.out(", %u gone (%s)", report.unknown, report.unknown_name);
    }
    if (report.unmentioned > 0u) {
        cli->io.out(", %u new (%s)", report.unmentioned,
                    report.unmentioned_name);
    }
    cli->io.out("\n");
    return 0;
}

int ak_cli_run(ak_cli_t *cli, const char *line)
{
    char words[AK_CLI_MAX_ARGS][AK_CLI_LINE_MAX / 2];
    unsigned argc = 0;
    const char *p = ak_skip_spaces(line);

    /* Split into words, terminated in place. A word longer than the buffer is
     * cut rather than overrunning; the parameter range check catches the rest. */
    while (*p != '\0' && argc < AK_CLI_MAX_ARGS) {
        unsigned n = 0;
        while (*p != '\0' && *p != ' ' && n + 1u < sizeof words[0]) {
            words[argc][n++] = *p++;
        }
        words[argc][n] = '\0';
        argc++;
        while (*p == ' ') {
            p++;
        }
    }

    cli->commands++;
    if (argc == 0) {
        return 0;
    }

    const char *cmd = words[0];

    if (ak_str_eq(cmd, "help")) {
        print_help(cli);
        return 0;
    }
    if (ak_str_eq(cmd, "banner")) {
        ak_banner_print(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "version")) {
        print_version(cli);
        return 0;
    }
    if (ak_str_eq(cmd, "status")) {
        print_status(cli);
        return 0;
    }
    if (ak_str_eq(cmd, "params")) {
        ak_params_dump(cli->params, cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "output")) {
        if (cli->io.output_report == 0) {
            cli->io.out("this board has no outputs\n");
            return 1;
        }
        const char *argv[AK_CLI_MAX_ARGS];
        for (unsigned i = 0; i < argc; i++) {
            argv[i] = words[i];
        }
        return cli->io.output_report(cli->io.out, (int)argc, argv) == 0 ? 0 : 1;
    }
    if (ak_str_eq(cmd, "rc")) {
        if (cli->io.rc_report == 0) {
            cli->io.out("this board has no receiver input\n");
            return 1;
        }
        cli->io.rc_report(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "imu")) {
        if (cli->io.imu_report == 0) {
            cli->io.out("this board has no inertial sensor\n");
            return 1;
        }
        cli->io.imu_report(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "baro")) {
        if (cli->io.baro_report == 0) {
            cli->io.out("this board has no barometer\n");
            return 1;
        }
        cli->io.baro_report(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "range")) {
        if (cli->io.range_report == 0) {
            cli->io.out("this board has no rangefinder\n");
            return 1;
        }
        cli->io.range_report(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "battery")) {
        if (cli->io.battery_report == 0) {
            cli->io.out("this board has no way to measure a battery\n");
            return 1;
        }
        cli->io.battery_report(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "spi")) {
        if (cli->io.spi_test == 0) {
            cli->io.out("this board has no sensor bus\n");
            return 1;
        }
        cli->io.spi_test(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "calibrate")) {
        if (cli->io.calibrate == 0) {
            cli->io.out("this board has nothing to calibrate\n");
            return 1;
        }
        const char *argv[AK_CLI_MAX_ARGS];
        for (unsigned i = 0; i < argc; i++) {
            argv[i] = words[i];
        }
        return cli->io.calibrate(cli->io.out, (int)argc, argv) == 0 ? 0 : 1;
    }
    if (ak_str_eq(cmd, "mission")) {
        if (cli->io.mission == 0) {
            cli->io.out("this firmware has no navigator\n");
            return 1;
        }
        const char *argv[AK_CLI_MAX_ARGS];
        for (unsigned i = 0; i < argc; i++) {
            argv[i] = words[i];
        }
        return cli->io.mission(cli->io.out, (int)argc, argv) == 0 ? 0 : 1;
    }
    if (ak_str_eq(cmd, "log")) {
        if (cli->io.log_dump == 0) {
            cli->io.out("this board has no blackbox\n");
            return 1;
        }
        if (argc >= 2 && ak_str_eq(words[1], "reset")) {
            if (cli->io.log_reset != 0) {
                cli->io.log_reset();
            }
            cli->io.out("log cleared\n");
            return 0;
        }
        /* The long log: the same records at a tenth of the rate, and - on a
         * board with retained RAM - the one that is still there after a
         * reset. `log` is for a tuning pass; this is for what happened. */
        if (argc >= 2 && ak_str_eq(words[1], "long")) {
            if (cli->io.longlog_dump == 0) {
                cli->io.out("this board has no long log\n");
                return 1;
            }
            if (argc >= 3 && ak_str_eq(words[2], "clear")) {
                if (cli->io.longlog_reset != 0) {
                    cli->io.longlog_reset();
                }
                cli->io.out("long log cleared\n");
                return 0;
            }
            cli->io.longlog_dump(cli->io.out);
            return 0;
        }
        /* The third log: the one in flash, which is the only one that is still
         * there when the battery is not. It is slow and it is short at this
         * rate, and both of those are the point. */
        if (argc >= 2 && ak_str_eq(words[1], "flash")) {
            if (cli->io.flashlog_dump == 0) {
                cli->io.out("this board has no flash log\n");
                return 1;
            }
            if (argc >= 3 && ak_str_eq(words[2], "clear")) {
                if (cli->io.flashlog_reset != 0) {
                    cli->io.flashlog_reset();
                }
                cli->io.out("flash log cleared\n");
                return 0;
            }
            cli->io.flashlog_dump(cli->io.out);
            return 0;
        }
        cli->io.log_dump(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "gps")) {
        if (cli->io.gps_report == 0) {
            cli->io.out("this board has no gps\n");
            return 1;
        }
        cli->io.gps_report(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "home")) {
        if (cli->io.home_set == 0) {
            cli->io.out("this board has no navigator\n");
            return 1;
        }
        if (argc >= 2 && ak_str_eq(words[1], "clear")) {
            if (cli->io.home_clear != 0) {
                cli->io.home_clear();
            }
            cli->io.out("home forgotten\n");
            return 0;
        }
        return cli->io.home_set(cli->io.out) == 0 ? 0 : 1;
    }
    if (ak_str_eq(cmd, "preflight")) {
        if (cli->io.preflight == 0) {
            cli->io.out("this board cannot check itself\n");
            return 1;
        }
        return cli->io.preflight(cli->io.out) == 0 ? 0 : 1;
    }
    if (ak_str_eq(cmd, "proto")) {
        if (cli->io.proto_report == 0) {
            cli->io.out("this board speaks no protocol\n");
            return 1;
        }
        cli->io.proto_report(cli->io.out);
        return 0;
    }
    if (ak_str_eq(cmd, "get")) {
        if (argc < 2) {
            cli->io.out("usage: get <name>\n");
            return 1;
        }
        ak_param_t *item = ak_params_find(cli->params, words[1]);
        if (item == 0) {
            cli->io.out("unknown parameter: %s\n", words[1]);
            return 1;
        }
        char value[AK_PARAM_VALUE_MAX];
        ak_params_get_text(item, value, sizeof value);
        cli->io.out("%s = %s  (%s)\n", item->name, value, item->help);
        return 0;
    }
    if (ak_str_eq(cmd, "set")) {
        if (argc < 3) {
            cli->io.out("usage: set <name> <value>\n");
            return 1;
        }
        /* The value is the rest of the line rather than one word: a Wi-Fi
         * password may have a space in it, and so may a name somebody types.
         * A number with a stray second word fails to parse, which is the
         * better outcome anyway. */
        const char *text = value_after_name(line, words[1]);
        char msg[64];
        msg[0] = '\0';
        if (text == 0 || ak_params_set(cli->params, words[1], text, msg,
                                       sizeof msg) != 0) {
            cli->io.out("%s\n", msg);
            return 1;
        }
        ak_param_t *item = ak_params_find(cli->params, words[1]);
        char value[AK_PARAM_VALUE_MAX];
        ak_params_get_text(item, value, sizeof value);
        apply_change(cli);
        cli->io.out("%s = %s\n", item->name, value);
        return 0;
    }
    if (ak_str_eq(cmd, "defaults")) {
        ak_params_reset(cli->params);
        apply_change(cli);
        cli->io.out("parameters back to their built-in values\n");
        return 0;
    }
    if (ak_str_eq(cmd, "save")) {
        return command_save(cli);
    }
    if (ak_str_eq(cmd, "load")) {
        return command_load(cli);
    }
    if (ak_str_eq(cmd, "clear")) {
        cli->io.out("\n\n\n");
        return 0;
    }
    if (ak_str_eq(cmd, "reboot")) {
        if (cli->io.reboot == 0) {
            cli->io.out("reboot is not wired on this board\n");
            return 1;
        }
        cli->io.out("rebooting\n");
        cli->io.reboot();
        return 0;
    }
    if (ak_str_eq(cmd, "dfu")) {
        /*
         * The way back in to flash something else. The call does not return on
         * a board that can do it - the ROM takes the part over - so the only
         * reachable path here is the one where it could not, and the message
         * says so rather than leaving somebody to wonder whether it worked.
         *
         * The console is the thing to type this at, and on the boards this
         * firmware targets the console is a USB port that is about to stop
         * being a console: after the hand-over, the host sees the ROM's own
         * device instead, and on this family leaving that needs the cable
         * pulled and put back.
         */
        if (cli->io.bootloader == 0 || cli->io.bootloader() == 0) {
            cli->io.out("dfu:       this board cannot enter its rom bootloader "
                        "from software\n");
            return 1;
        }
        return 0;
    }

    cli->io.out("unknown command: %s (try 'help')\n", cmd);
    return 1;
}

void ak_cli_feed(ak_cli_t *cli, char c)
{
    if (c == '\r' || c == '\n') {
        cli->io.out("\r\n");
        if (cli->overflow) {
            cli->io.out("line too long, discarded\r\n");
        } else if (cli->len > 0) {
            cli->line[cli->len] = '\0';
            ak_cli_run(cli, cli->line);
        }
        cli->len = 0;
        cli->overflow = 0;
        ak_cli_prompt(cli);
        return;
    }

    if (c == '\b' || c == 127) {
        if (cli->len > 0) {
            cli->len--;
            cli->io.out("\b \b");
        }
        return;
    }

    if (c < ' ' || c > '~') {
        return;
    }

    if (cli->len + 1u >= sizeof cli->line) {
        cli->overflow = 1;
        return;
    }

    cli->line[cli->len++] = c;
    cli->io.out("%c", c);
}
