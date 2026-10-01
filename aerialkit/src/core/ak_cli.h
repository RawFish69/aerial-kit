#ifndef AK_CORE_AK_CLI_H
#define AK_CORE_AK_CLI_H

#include <stdint.h>

#include "ak_console.h"
#include "ak_flight.h"
#include "ak_params.h"

/*
 * The console the aircraft is talked to through.
 *
 * Everything it touches arrives through `ak_cli_io_t`: the output, the clock,
 * and the three board operations (read saved configuration, write saved
 * configuration, reset). That is what lets the whole command set be tested on
 * the host with a capturing printf and a fake flash, and it is also why the CLI
 * never has to know what a UART is.
 *
 * It is deliberately not a protocol. A person with a serial adapter is the
 * intended caller; the machine-readable version is the configurator's job, and
 * it will speak to the same parameter table.
 */

#define AK_CLI_LINE_MAX 96

/*
 * The banner, printed to anything that takes printf.
 *
 * One function and not two, because there were two and they disagreed: the
 * boot banner printed `ak_board_name()` - `AERIALKIT_F405 / WeAct
 * STM32F405RGT6` - while `version` printed the bare `AK_BOARD_STR`.
 *
 * **Neither of them names the hardware**, which the first version of this
 * comment got wrong: both are compile-time strings taken from the selected
 * board file, so both name what the image was built for. `ak_board_name()` is
 * only *longer*, and a longer string reads more like a reading - so printing it
 * in `version` made the hazard worse rather than better. Reading a build fact
 * as a board fact is what trap 209 cost a bench session; the `board:` line's
 * one measured field is the crystal beside it.
 *
 * It is here, and not beside the boot path that calls it first, because the
 * second caller is the console command below: the boot banner is emitted before
 * a host can possibly be attached, so the only way to read it back is to ask
 * for it again.
 */
void ak_banner_print(ak_printf_fn out);

typedef struct {
    ak_printf_fn out;
    uint32_t (*now_ms)(void);
    int (*config_read)(void *buf, uint32_t len);   /* bytes read, or -1 */
    int (*config_write)(const void *buf, uint32_t len); /* 0 on success */
    void (*reboot)(void);
    /* Hand the part to its ROM bootloader: 0 on a board with no software way
     * in, and on one that has it the call does not return - the ROM takes the
     * part over. The "dfu" command is the only caller. */
    int  (*bootloader)(void);
    /* What the outputs are doing, and - with "test" - driving them so somebody
     * with a scope can look. The arguments arrive as typed, argv[0] being the
     * command itself, the same way the calibrations get theirs. */
    int  (*output_report)(ak_printf_fn out, int argc, const char *const *argv);
    void (*rc_report)(ak_printf_fn out);     /* what the receiver is saying */
    void (*imu_report)(ak_printf_fn out);    /* what the inertial sensor says */
    void (*baro_report)(ak_printf_fn out);   /* what the barometer says */
    void (*range_report)(ak_printf_fn out);  /* what the rangefinder says */
    void (*battery_report)(ak_printf_fn out); /* what the flight pack is doing */
    void (*spi_test)(ak_printf_fn out);      /* check the sensor bus wiring */
    /* Measure a baseline the pilot provides: what "still" looks like ("gyro"
     * or no argument), what "centred" looks like ("rc"), or which way up the
     * aircraft is ("accel <face>"). The arguments arrive the way they were
     * typed, with argv[0] the command itself. */
    int  (*calibrate)(ak_printf_fn out, int argc, const char *const *argv);
    /* The mission: report it, add a waypoint, start or stop flying it. */
    int  (*mission)(ak_printf_fn out, int argc, const char *const *argv);
    void (*log_dump)(ak_printf_fn out);      /* the blackbox, as CSV */
    void (*log_reset)(void);
    /* And the long one, which is the same records at a tenth of the rate and
     * survives a reset where the board has retained RAM. */
    void (*longlog_dump)(ak_printf_fn out);
    void (*longlog_reset)(void);
    /* And the third one, in flash: the records that outlive the battery, at a
     * rate the loop can afford. */
    void (*flashlog_dump)(ak_printf_fn out);
    void (*flashlog_reset)(void);
    void (*gps_report)(ak_printf_fn out);    /* the position, if there is one */
    int  (*home_set)(ak_printf_fn out);      /* take the current fix as home */
    void (*home_clear)(void);
    int  (*preflight)(ak_printf_fn out);     /* is this machine what it thinks? */
    /*
     * Whether this aircraft would arm, and if not, what is in the way. It is a
     * callback rather than something this file decides because the gates are
     * the flight core's and the words are the board's console's; `status` asks
     * it while disarmed, which is the one moment the answer is a question.
     */
    void (*arm_report)(ak_printf_fn out);
    /*
     * Is the aircraft disarmed? The one *predicate* the console needs, because
     * `save` writes flash and this board's erase stalls the CPU - and the loop
     * with it - for about a second, which is why the same rule the four
     * calibrations follow applies here: a command that can only be about an
     * aircraft on a bench refuses to run one that is flying.
     */
    int  (*disarmed)(void);
    void (*proto_report)(ak_printf_fn out);  /* config-protocol counters */
    /*
     * What each link cost the loop: the most bytes any one of them has taken
     * in a single pass. A callback for the same reason the one above is -
     * the quota belongs to the loop and the words belong to whoever owns the
     * console - and printed in `status` because it is the other half of the
     * timing numbers: those say the loop was late, this says what made it.
     */
    void (*link_report)(ak_printf_fn out);
    void (*on_change)(void);                 /* a parameter moved */
} ak_cli_io_t;

typedef struct {
    ak_cli_io_t   io;
    ak_params_t  *params;
    ak_flight_t  *flight;
    char          line[AK_CLI_LINE_MAX];
    /* One more than the longest record: the load path is handed the record's
     * length by the board and writes a terminator after it. */
    char          config[AK_PARAMS_TEXT_MAX + 1];
    unsigned      len;
    unsigned      overflow;
    uint32_t      commands;
} ak_cli_t;

void ak_cli_init(ak_cli_t *cli, const ak_cli_io_t *io, ak_params_t *params,
                 ak_flight_t *flight);

/* Print the console's prompt, "ak> ". The console prints it after every line
 * it runs, and the firmware prints the first one when the boot report is done
 * - so anything watching the port can tell "idle and listening" from "gone".
 */
void ak_cli_prompt(ak_cli_t *cli);

/* Throw away a half-typed line without running it, ending it on screen if
 * anything of it was echoed.
 *
 * The console shares its port with the binary protocol, and `ak_cli_feed`
 * cannot tell the two apart by looking at a byte: a payload or checksum byte
 * that happens to land in the printable range is a character as far as this
 * file is concerned, and one that happens to be CR or LF is the end of a line.
 * So binary traffic that reaches this console does two things, and both of them
 * are the same complaint from the far end - a command typed next is not
 * answered, and the output has a prompt in it in a place no command put one.
 * The second is worse than it looks: the prompt is what a script reads to mean
 * "the last command has finished" (see ak_cli_prompt above), so a prompt nobody
 * asked for is a script that stops reading in the middle of an answer.
 *
 * The prompt is deliberately not reprinted here. The caller is about to put
 * binary on the wire, and "ak> " in the middle of a frame is one more thing for
 * the client on the other end to have to parse around.
 */
void ak_cli_forget(ak_cli_t *cli);

/* One received byte. Printable characters are echoed, backspace edits, and a
 * newline runs the line. A line that overflows is thrown away rather than
 * half-executed. */
void ak_cli_feed(ak_cli_t *cli, char c);

/* Runs one line as if it had been typed. 0 on success, 1 on an unknown command
 * or a bad argument. Exposed so the tests can drive it without a shell. */
int ak_cli_run(ak_cli_t *cli, const char *line);

#endif /* AK_CORE_AK_CLI_H */
