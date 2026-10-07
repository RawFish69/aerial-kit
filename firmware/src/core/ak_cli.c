#include "ak_cli.h"

#include "ak_board.h"
#include "ak_dshot_gcr.h"
#include "ak_dshot_capture.h"
#include "ak_dshot_edt.h"
#include "ak_fault.h"
#include "ak_perf.h"
#include "ak_sched.h"
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

/*
 * Where the loop's time goes.
 *
 * `status` prints the loop's *period* - how long between iterations, which the
 * millisecond tick can answer - and nothing about its *duration*, which it
 * cannot: a tick that fires every millisecond cannot resolve what happened
 * inside one. This is the other half, and it comes from the cycle counter the
 * port runs (ak_perf.h).
 *
 * Four things are printed, in the order a person reads them:
 *
 *   - the window: how many loops closed, and how many of those the port could
 *     actually time. On a port whose clock reads zero the second number stops
 *     while the first keeps going, and printing only one of them would make a
 *     profiler that is not measuring look exactly like one that is.
 *   - the period, against the nominal the loop is trying to hold.
 *   - the jitter, which is the distance from that nominal in *either*
 *     direction: a loop that came back early is as interesting as one that came
 *     back late, and reporting only the late half would hide a clock running
 *     fast.
 *   - the sections, and the load they add up to.
 *
 * The load is a *lower bound* and the last line says so, because the firmware
 * does work outside the instrumented sections - the console, the links, the
 * navigation - and a percentage that quietly omitted it would be the kind of
 * claim this file exists to prevent.
 *
 * `perf reset` forgets the window without stopping anything, which is what a
 * person does before a measurement they intend to quote.
 */
static int command_perf(ak_cli_t *cli, char words[][AK_CLI_LINE_MAX / 2],
                        unsigned argc)
{
    ak_perf_snapshot_t s;
    unsigned i;

    if (argc > 1u) {
        if (!ak_str_eq(words[1], "reset")) {
            cli->io.out("perf:      unknown argument '%s' (perf [reset])\n",
                        words[1]);
            return 1;
        }
        if (!ak_perf_reset()) {
            cli->io.out("perf:      nothing to forget - no loop has closed "
                        "since this board came up\n");
            return 1;
        }
        cli->io.out("perf:      window cleared; the counter keeps running\n");
        return 0;
    }

    ak_perf_snapshot(&s);
    if (!s.started) {
        /* Not a window of zeros. A build with no profiler in it has nothing to
         * report, and "the loop costs nothing" is the one thing that number
         * would look like. */
        cli->io.out("perf:      no profiler in this build\n");
        return 1;
    }

    cli->io.out("perf:      %u loops closed, %u of them measured\n", s.loops,
                s.samples);
    cli->io.out("period:    %u us nominal, last %u, min %u, max %u, %u late\n",
                s.nominal_us, s.period_last_us, s.period_min_us,
                s.period_max_us, s.late);
    cli->io.out("jitter:    p50 %u us, p99 %u us, max %u us (%u past the "
                "%u us bin)\n",
                (unsigned)s.jitter_p50_us, (unsigned)s.jitter_p99_us,
                (unsigned)s.jitter_max_us, s.jitter_over,
                AK_PERF_JITTER_BINS - 1u);
    cli->io.out("sections:  per loop, averaged over the window; then the "
                "longest single run\n");
    for (i = (unsigned)AK_PERF_NONE + 1u; i < (unsigned)AK_PERF_SECTIONS;
         i++) {
        unsigned avg_ns = s.section_avg_ns[i];

        cli->io.out("  %-9s  %u.%03u us   max %u us\n",
                    ak_perf_section_name((ak_perf_section_t)i), avg_ns / 1000u,
                    avg_ns % 1000u, s.section_max_us[i]);
    }
    cli->io.out("load:      %u.%u%% of the %u us slot, instrumented sections "
                "only\n",
                s.load_permille / 10u, s.load_permille % 10u, s.nominal_us);
    return 0;
}

/*
 * A bidirectional DShot reply, read back off the wire by hand.
 *
 * This is the bench tool for phase 3.1's other half. The port that captures the
 * ESC's twenty-one levels - input capture, DMA, where the frame starts, how
 * many samples a level lasts - cannot be tested without an ESC that answers, and
 * there is none on this bench. What *can* be done without one is this: read the
 * levels a capture gives into the console and get the number out, which is how
 * a real capture will be checked against the ESC's own figure when there is one
 * to check. Until then it is also the only way to see the decoder work on
 * anything but the host tests' own frames.
 *
 * There are two ways in, and the second is the one a real capture arrives by.
 * The levels are typed as a string, one character for each level in the order
 * they arrived, `_ . , :` allowed as separators so a person can paste a capture
 * in groups of five:
 *
 *     dshot 1_10101_01111_11101_11011
 *
 * and an optional pole count, which is what turns eRPM into the number a
 * tachometer would read:
 *
 *     dshot 1_10101_01111_11101_11011 14
 *
 * `dshot runs` takes the same reply as the *run lengths* a capture actually
 * gives - how many samples the line held each level, at three samples to the
 * bit - which is the shape ak_dshot_capture_slice() wants and the shape a
 * logic analyser or a DMA buffer produces:
 *
 *     dshot runs 9_14_5_2_5_2_8_2_2_2_2_2_2_2_8_2_2_5_8_2_2_2
 *
 * The two are not the same command twice. The levels path starts *after* the
 * capture half and checks the decoder; the runs path starts where the port
 * starts and checks the whole of phase 3.1, printing the levels it worked out
 * so that a person can see where a disagreement is. It is also the only way to
 * reach the module's refusals - a line that never went low, a capture too short
 * to be a reply - without an ESC to produce them.
 *
 * A leading high run is the first thing a real capture has, so the run list may
 * begin with a `^` marker: the first run is then high, and the module skips it
 * as the line idling before the ESC pulled it. Without the marker the first run
 * is low, which is the shape a port gives after it has found the frame's start.
 *
 * Without a pole count there is no mechanical rpm to print, and the command says
 * so rather than assuming the fourteen poles an aircraft motor usually has: the
 * acceptance criterion is "matching a tachometer within 2%", and a figure that
 * came from an assumed pole count could not be compared with one.
 *
 * A refused frame is described rather than only rejected - the four symbols are
 * printed with the bad one marked, and the fold's value is printed against the
 * 0xf that would have made it good - because "the decoder said no" is not
 * enough to tell a mis-sliced capture from a bad table.
 */
/*
 * The first line of both dshot readouts: the four symbols as they came off the
 * wire, and the frame's own verdict on them. Shared, because there is one
 * decode and two readouts of what it returned - the eRPM above and the
 * extended-telemetry one below - and a second copy of this would be a second
 * place for the refusals to drift apart. Zero when the frame decoded, and
 * `*word` is then the twelve-bit value; one when it did not, and the reason has
 * been printed.
 */
static int dshot_open(ak_cli_t *cli, const uint8_t *bits,
                      ak_dshot_gcr_frame_t *frame, uint16_t *word)
{
    ak_dshot_gcr_t verdict;
    unsigned i;

    cli->io.out("dshot:     ");
    ak_dshot_gcr_read(bits, frame);
    for (i = 0u; i < 4u; i++) {
        if (frame->nibbles[i] == AK_DSHOT_GCR_BAD) {
            cli->io.out("-- ");
        } else {
            cli->io.out("%x ", frame->nibbles[i]);
        }
    }

    verdict = ak_dshot_gcr_decode(bits, word);
    if (verdict == AK_DSHOT_GCR_BAD_SYMBOL) {
        cli->io.out("- group %u is not one of the sixteen codes\n", frame->bad + 1u);
        cli->io.out("dshot:     a group of five levels that is not a code "
                    "means the frame was sliced at the wrong level, or the "
                    "capture is not a reply\n");
        return 1;
    }
    if (verdict == AK_DSHOT_GCR_BAD_CRC) {
        cli->io.out("- word 0x%x, fold 0x%x, and a reply has to fold to 0x%x\n",
                    frame->word, frame->fold, AK_DSHOT_GCR_FOLD_OK);
        cli->io.out("dshot:     four legal symbols and a fold that is not it: "
                    "the frame is refused\n");
        return 1;
    }

    cli->io.out("- word 0x%x, fold 0x%x\n", frame->word, frame->fold);
    return 0;
}

/*
 * The levels of a reply as a person types them, shared by both readouts. The
 * strictness is the same as `dshot runs`': a level that is not 0 or 1, and a
 * frame that is not twenty-one levels long, are refused here rather than handed
 * on to become a plausible wrong number. Zero on success.
 */
static int dshot_parse_levels(ak_cli_t *cli, const char *spec, uint8_t *bits)
{
    unsigned levels = 0u;
    unsigned i;

    for (i = 0u; spec[i] != '\0'; i++) {
        char c = spec[i];

        if (c == '0' || c == '1') {
            if (levels >= AK_DSHOT_GCR_BITS) {
                cli->io.out("dshot:     %u levels typed, and a reply is %u: "
                            "the rest of the line is not a pole count\n",
                            levels + 1u, AK_DSHOT_GCR_BITS);
                return 1;
            }
            bits[levels] = (uint8_t)(c - '0');
            levels++;
        } else if (c != '_' && c != '.' && c != ',' && c != ':') {
            cli->io.out("dshot:     '%c' is not a level: use 0 and 1, and "
                        "_ . , : as separators\n", c);
            return 1;
        }
    }

    if (levels != AK_DSHOT_GCR_BITS) {
        cli->io.out("dshot:     %u levels read, and a reply is %u: a short "
                    "frame is not a frame\n", levels, AK_DSHOT_GCR_BITS);
        return 1;
    }

    return 0;
}

static int dshot_report(ak_cli_t *cli, const uint8_t *bits, uint32_t poles,
                        int have_poles)
{
    ak_dshot_gcr_frame_t frame;
    uint16_t value = 0u;

    if (dshot_open(cli, bits, &frame, &value)) {
        return 1;
    }

    if (value == 0x0FFFu) {
        cli->io.out("value:     0x%x - the esc says the motor is not turning\n",
                    value);
        return 0;
    }

    uint32_t period = ak_dshot_period_us(value);
    uint32_t erpm = 0u;

    if (ak_dshot_erpm_from_value(value, &erpm) != AK_DSHOT_ERPM_TURNING) {
        /* A frame that passed the fold and carries no period: a mantissa of
         * zero, which the reference reads as invalid rather than as a speed.
         * The other way into this - a period slower than the arithmetic can
         * express - cannot happen from twenty-one levels, because the field is
         * twelve bits and its widest period is 65408 us, which is 900 eRPM; the
         * module refuses anything wider as "not a reply" before the arithmetic
         * sees it, so this command has no branch for it and needs none. */
        cli->io.out("value:     0x%x - the esc sent no period: a mantissa of "
                    "zero is not a speed\n", value);
        return 1;
    }

    cli->io.out("value:     0x%x - a period of %u us per electrical "
                "revolution\n", value, period);
    cli->io.out("erpm:      %u\n", erpm);

    if (have_poles) {
        cli->io.out("rpm:       %u, on %u poles\n", ak_dshot_rpm(erpm, poles),
                    poles);
    } else {
        cli->io.out("rpm:       not printed: name the motor's poles "
                    "(a pole count after the levels)\n");
    }
    return 0;
}

/*
 * `dshot edt`: the same twenty-one levels under the extended-telemetry law.
 *
 * Two readouts and not one, which is the whole point of the command. Whether a
 * reply is a period or one of the seven other things is not visible in the bits
 * - it is a property of what that motor was asked for and what it has sent
 * before (ak_dshot_edt.h's decision 7) - and a console has no motor and no
 * history to consult. So this prints what the reply is with extended telemetry
 * in play, and beside it what the same twelve bits are with it off, which is
 * what `dshot` itself answers. A readout that showed only one of the two would
 * teach that the field has one meaning, and the two are one bit apart.
 */
static int dshot_report_edt(ak_cli_t *cli, const uint8_t *bits, uint32_t poles,
                            int have_poles)
{
    ak_dshot_gcr_frame_t frame;
    ak_dshot_edt_value_t value;
    ak_dshot_edt_status_t status;
    uint32_t erpm = 0u;
    uint16_t word = 0u;
    unsigned field;

    if (dshot_open(cli, bits, &frame, &word)) {
        return 1;
    }

    field = (unsigned)((word & AK_DSHOT_EDT_TYPE_MASK) >> AK_DSHOT_EDT_TYPE_SHIFT);
    status = ak_dshot_edt_decode(word, 1, &value);

    if (status == AK_DSHOT_EDT_STOPPED) {
        cli->io.out("edt:       the esc says the motor is not turning (0x%x) - "
                    "a period field either way\n", word);
        return 0;
    }
    if (status == AK_DSHOT_EDT_BAD_ERPM) {
        cli->io.out("edt:       0x%x has the marker clear and no period in it: "
                    "a mantissa of zero is not a speed\n", word);
        return 1;
    }

    cli->io.out("edt:       %s, from field %u of the four bits 11..8\n",
                ak_dshot_edt_type_name(value.type), field);

    if (value.type == AK_DSHOT_EDT_ERPM) {
        cli->io.out("value:     %u erpm, and the same answer with extended "
                    "telemetry off, because this reply carries the marker\n",
                    value.erpm);
        if (have_poles) {
            cli->io.out("rpm:       %u, on %u poles\n",
                        ak_dshot_rpm(value.erpm, poles), poles);
        }
        return 0;
    }

    cli->io.out("payload:   0x%x, the low eight bits\n", value.payload);

    switch (value.type) {
    case AK_DSHOT_EDT_TEMPERATURE:
        cli->io.out("value:     %u celsius\n", value.celsius);
        break;
    case AK_DSHOT_EDT_VOLTAGE:
        cli->io.out("value:     %u mv, at %u mv a count\n",
                    value.millivolts, AK_DSHOT_EDT_MV_PER_COUNT);
        break;
    case AK_DSHOT_EDT_CURRENT:
        cli->io.out("value:     %u ma, at %u ma a count\n",
                    value.milliamps, AK_DSHOT_EDT_MA_PER_COUNT);
        break;
    default:
        cli->io.out("value:     none this firmware can state: %s is the esc's "
                    "own number, and what it means is in the esc's "
                    "documentation and not in these bits\n",
                    ak_dshot_edt_type_name(value.type));
        break;
    }

    /* And the other reading of the same twelve bits, which is what makes this
     * field worth two decisions rather than one. */
    if (ak_dshot_erpm_from_value(word, &erpm) == AK_DSHOT_ERPM_TURNING) {
        cli->io.out("erpm:      the same bits with extended telemetry off are a "
                    "period: %u erpm\n", erpm);
        if (have_poles) {
            cli->io.out("rpm:       %u, on %u poles\n",
                        ak_dshot_rpm(erpm, poles), poles);
        }
    } else {
        cli->io.out("erpm:      the same bits with extended telemetry off are "
                    "not a period at all, so the two readings do not even "
                    "overlap\n");
    }

    return 0;
}

/*
 * `dshot runs`: the run lengths of a capture, to the twenty-one levels, to the
 * readout above.
 *
 * The parse is deliberately strict about two things a port cannot produce and a
 * person can type: a run of no samples, and more runs than a capture can hold
 * (AK_DSHOT_CAPTURE_RUNS_MAX). Neither is a reply, and the module's own refusals
 * are about the *bits* a run list carries rather than about its shape, so a
 * list that is malformed in shape is refused here instead of being handed on to
 * become a plausible wrong number.
 */
static int command_dshot_runs(ak_cli_t *cli, const char *spec, uint32_t poles,
                              int have_poles)
{
    ak_dshot_run_t runs[AK_DSHOT_CAPTURE_RUNS_MAX];
    uint8_t bits[AK_DSHOT_GCR_BITS];
    unsigned count = 0u;
    unsigned level = 0u;
    const char *p = spec;

    if (*p == '^') {
        level = 1u;
        p++;
    }

    while (*p != '\0') {
        unsigned samples = 0u;
        unsigned digits = 0u;

        while (*p >= '0' && *p <= '9') {
            samples = samples * 10u + (unsigned)(*p - '0');
            digits++;
            if (samples > 0xFFFFu) {
                cli->io.out("dshot:     a run longer than %u samples: the "
                            "capture is not a reply\n", 0xFFFFu);
                return 1;
            }
            p++;
        }

        if (digits == 0u) {
            cli->io.out("dshot:     runs are sample counts: '%c' is not one\n",
                        *p);
            return 1;
        }
        if (samples == 0u) {
            cli->io.out("dshot:     a run of no samples is not a run: the "
                        "line cannot change level and change back\n");
            return 1;
        }
        if (count >= AK_DSHOT_CAPTURE_RUNS_MAX) {
            cli->io.out("dshot:     more than %u runs: a reply of twenty-one "
                        "bits needs at most twenty-one, and a capture this "
                        "long is not one\n", AK_DSHOT_CAPTURE_RUNS_MAX);
            return 1;
        }

        runs[count].level = (uint8_t)level;
        runs[count].samples = (uint16_t)samples;
        count++;
        level ^= 1u;

        if (*p == '\0') {
            break;
        }
        if (*p != '_' && *p != '.' && *p != ',' && *p != ':') {
            cli->io.out("dshot:     '%c' is not a separator: use _ . , : "
                        "between run lengths\n", *p);
            return 1;
        }
        p++;
    }

    if (count == 0u) {
        cli->io.out("dshot:     no runs typed\n");
        return 1;
    }

    ak_dshot_capture_t verdict = ak_dshot_capture_slice(runs, count, bits);

    if (verdict == AK_DSHOT_CAPTURE_NO_LOW) {
        cli->io.out("dshot:     the line never went low in this capture, so no "
                    "esc answered the frame: there is no reply in it to read\n");
        return 1;
    }
    if (verdict == AK_DSHOT_CAPTURE_TOO_FEW_BITS) {
        cli->io.out("dshot:     the runs carry fewer than %u bits, and a reply "
                    "is %u: either the capture stopped early or no esc sent "
                    "one\n", AK_DSHOT_CAPTURE_MIN_BITS, AK_DSHOT_GCR_BITS);
        return 1;
    }
    if (verdict == AK_DSHOT_CAPTURE_TOO_MANY_BITS) {
        cli->io.out("dshot:     a run would carry more bits than a reply "
                    "holds: the capture is longer than one frame\n");
        return 1;
    }

    /* Printed grouped as the wire sends it - the start bit and then four
     * symbols - so that this line can be typed straight back into the levels
     * form with the separators added, and the two paths can be held against
     * each other on the same capture. */
    cli->io.out("levels:    ");
    for (unsigned i = 0u; i < AK_DSHOT_GCR_BITS; i++) {
        if (i == 1u || i == 6u || i == 11u || i == 16u) {
            cli->io.out(" ");
        }
        cli->io.out("%u", (unsigned)bits[i]);
    }
    cli->io.out("\n");

    return dshot_report(cli, bits, poles, have_poles);
}

static int command_dshot(ak_cli_t *cli, char words[][AK_CLI_LINE_MAX / 2],
                         unsigned argc)
{
    uint8_t bits[AK_DSHOT_GCR_BITS];
    uint32_t poles = 0u;
    int have_poles = 0;

    if (argc < 2u) {
        cli->io.out("dshot:     a reply is %u levels, as 0 and 1 in the order "
                    "they arrived\n", AK_DSHOT_GCR_BITS);
        cli->io.out("dshot:     dshot <levels> [poles], for example "
                    "dshot 1_10101_01111_11101_11011 14\n");
        cli->io.out("dshot:     or dshot edt <levels> [poles], the same reply "
                    "read for extended telemetry: temperature, voltage, "
                    "current, or a byte the esc defines\n");
        cli->io.out("dshot:     or dshot runs <lengths> [poles], the same reply "
                    "as the samples each level held, for example "
                    "dshot runs 9_14_5_2_5\n");
        return 1;
    }

    if (ak_str_eq(words[1], "edt")) {
        /*
         * Three words or four: the sub-verb, the levels, and an optional pole
         * count. There is no fifth case to refuse here even though the shape
         * has room for one, because ak_cli_run() splits a line into at most
         * AK_CLI_MAX_ARGS words - so `dshot edt <levels> 14 7` arrives as
         * `dshot edt <levels> 14` and the 7 is not seen by anything. That is
         * the splitter's behaviour and not this command's, and it is written
         * down rather than guarded against so that the next reader does not
         * add an `argc > 4` branch that can never run.
         */
        if (argc < 3u) {
            cli->io.out("dshot:     edt takes the same reply dshot does, and an "
                        "optional pole count (dshot edt <levels> [poles])\n");
            return 1;
        }
        if (argc == 4u) {
            if (!ak_parse_uint(words[3], &poles)) {
                cli->io.out("dshot:     '%s' is not a pole count\n", words[3]);
                return 1;
            }
            have_poles = 1;
        }
        if (dshot_parse_levels(cli, words[2], bits)) {
            return 1;
        }
        return dshot_report_edt(cli, bits, poles, have_poles);
    }

    if (ak_str_eq(words[1], "runs")) {
        if (argc < 3u || argc > 4u) {
            cli->io.out("dshot:     runs takes a run list and an optional pole "
                        "count (dshot runs <lengths> [poles])\n");
            return 1;
        }
        if (argc == 4u) {
            if (!ak_parse_uint(words[3], &poles)) {
                cli->io.out("dshot:     '%s' is not a pole count\n", words[3]);
                return 1;
            }
            have_poles = 1;
        }
        return command_dshot_runs(cli, words[2], poles, have_poles);
    }

    if (argc > 3u) {
        cli->io.out("dshot:     too many arguments (dshot <levels> [poles])\n");
        return 1;
    }

    if (dshot_parse_levels(cli, words[1], bits)) {
        return 1;
    }

    if (argc == 3u) {
        if (!ak_parse_uint(words[2], &poles)) {
            cli->io.out("dshot:     '%s' is not a pole count\n", words[2]);
            return 1;
        }
        have_poles = 1;
    }

    return dshot_report(cli, bits, poles, have_poles);
}

/*
 * The task table, read back.
 *
 * `ak_sched.c` counts four things per task and none of them is worth anything a
 * person cannot read off the board: a period nothing compares against, a
 * `missed` nobody can see, an overrun that shows up only as an aircraft that
 * behaves oddly. The host tests prove the accounting is right; this is what
 * makes it *evidence*, which is a different claim - the numbers on a bench board
 * under a real load are the ones phase 1 is judged on.
 *
 * The units are microseconds throughout, because that is what the deadlines are
 * in and a millisecond column would hide the whole quantity being measured. A
 * task that has never run prints its period and zeros, and that is a reading:
 * zero runs is not the same as no such task, and `ak_sched_stats` keeps the two
 * apart for exactly this report.
 */
static int command_tasks(ak_cli_t *cli)
{
    int count = ak_sched_count();
    int i;

    if (count <= 0) {
        /* Not "no scheduler in this build" - the object is always linked. An
         * empty table is an empty table: nothing has registered a task, which
         * on a board is `ak_firmware_main` not having reached its table yet. */
        cli->io.out("tasks:     nothing has registered a task\n");
        return 1;
    }

    cli->io.out("tasks:     %d in the table; slack is a twentieth of a task's "
                "own period\n", count);
    for (i = 0; i < count; i++) {
        ak_sched_stats_t s;
        const char *name;

        if (ak_sched_stats(i, &s) != 0) {
            continue;
        }
        name = ak_sched_name(i);
        /* The slack is printed per row rather than once at the top because
         * since phase 1.4 it belongs to the task's period, not to the table. */
        cli->io.out("  %-9s  %u us  slack %u us  %u runs, %u late "
                    "(worst %u us), %u missed\n",
                    name != 0 ? name : "?", s.period_us,
                    ak_sched_slack_us(s.period_us), s.runs, s.late,
                    s.late_max_us, s.missed);
        cli->io.out("             %u overran its period, last %u us, "
                    "longest %u us\n",
                    s.overrun, s.last_us, s.max_us);
    }
    return 0;
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
    cli->io.out("  perf [reset]         where the loop's time goes, in microseconds\n");
    cli->io.out("  dshot <21 levels>    decode a bidirectional reply: the levels\n");
    cli->io.out("    [poles]            an ESC sent as 0 and 1, in order\n");
    cli->io.out("  dshot edt <levels>   the same reply read for extended\n");
    cli->io.out("    [poles]            telemetry: temperature, voltage,\n");
    cli->io.out("                       current, or a byte the esc defines\n");
    cli->io.out("  dshot runs <runs>    the same reply as the run lengths a\n");
    cli->io.out("    [poles]            capture gives, in samples, ^ first if\n");
    cli->io.out("                       the run before the reply is in it\n");
    cli->io.out("  tasks                the task table: periods, lateness, misses\n");
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
    /* The two time counters are microseconds in the core and milliseconds on
     * this line, and the division is here rather than in the core so that the
     * accumulator does not lose a millisecond off every counted event. The
     * labels are unchanged: a person comparing two logs should be reading the
     * same units they always were. */
    cli->io.out("timing:    %u gaps, %u catchup pieces, %u dropped ms, "
                "%u unusable\n",
                flight->timing.gap_steps, flight->timing.catchup_steps,
                (unsigned)(flight->timing.dropped_us / 1000u),
                flight->timing.unusable);
    cli->io.out("timing:    %u duplicate samples, %u clock resets, "
                "%u long loops, longest %u ms\n",
                flight->timing.duplicates, flight->timing.clock_resets,
                flight->timing.long_loops,
                (unsigned)(flight->timing.max_loop_us / 1000u));
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
        if (report.renamed > 0u) {
            /* Neither gone nor new: a parameter this build moved to a new
             * name, whose value came across. Worth saying out loud, because
             * the person's next `save` will write the new spelling. */
            cli->io.out(", %u renamed (%s)", report.renamed,
                        report.renamed_name);
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
        /* Everything else in the record was loaded, so it is applied: the
         * table and the flight core must not disagree. Not marked saved -
         * the table now differs from the record by the refused value. */
        apply_change(cli);
        cli->io.out("load: refused %s; every other value was loaded\n", msg);
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
    if (report.renamed > 0u) {
        cli->io.out(", %u renamed (%s)", report.renamed, report.renamed_name);
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
    if (ak_str_eq(cmd, "perf")) {
        return command_perf(cli, words, argc);
    }
    if (ak_str_eq(cmd, "dshot")) {
        return command_dshot(cli, words, argc);
    }
    if (ak_str_eq(cmd, "tasks")) {
        return command_tasks(cli);
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
    if (ak_str_eq(cmd, "set") || ak_str_eq(cmd, "defaults") ||
        ak_str_eq(cmd, "load")) {
        /* The same rule the protocol's PARAM_SET has always had: a parameter
         * change rebuilds the controller (the PIDs re-initialise, `airframe`
         * swaps the mixer), and an armed aircraft is not a bench. Until
         * 2026-10-06 only `save` asked. */
        if ((cli->io.disarmed != 0 && !cli->io.disarmed()) ||
            (cli->flight != 0 && !ak_flight_config_writable(cli->flight))) {
            cli->io.out("%s: refusing - the aircraft is %s, and a parameter "
                        "change rebuilds the controller\n", cmd,
                        cli->flight != 0
                            ? ak_flight_state_name(ak_flight_state(cli->flight))
                            : "armed");
            return 1;
        }
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
        /* Read back *after* the change has been applied rather than before it.
         * The two differ for a parameter the hardware corrects - the rate
         * parameters are written back to what the part and the outputs actually
         * took - and reading first printed `dshot_khz = 600` on the line
         * directly under "dshot_khz is now 300", because apply_change() is what
         * runs the correction. The sentence this prints is a statement about
         * the table, so it has to be read from the table once nothing else is
         * going to move it. */
        ak_param_t *item = ak_params_find(cli->params, words[1]);
        char value[AK_PARAM_VALUE_MAX];
        apply_change(cli);
        ak_params_get_text(item, value, sizeof value);
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
