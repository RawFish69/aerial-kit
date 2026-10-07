#ifndef AK_CORE_AK_PROTO_H
#define AK_CORE_AK_PROTO_H

#include <stdint.h>

#include "ak_console.h"
#include "ak_params.h"

/*
 * The machine-readable side of the console.
 *
 * The console is for a person and the protocol is for a program: the NAS web
 * console, a companion computer on the aircraft, a test script. They share the
 * same UART, distinguished by the frame's first byte, and they share the same
 * parameter table - which is the point of having one table.
 *
 *   AA 55  version  command  length  payload...  crc16(lo)  crc16(hi)
 *
 * The crc covers the version, command, length and payload - everything except
 * the sync pair and the crc itself, which is the only arrangement where a
 * corrupted length cannot be hidden by the bytes that follow it.
 *
 * Commands are small and their replies carry the same command with the top bit
 * set, so a reply to command 1 is 0x81: one field to check instead of a
 * correlation table.
 *
 * Byte-fed, like every other parser here, because that is the shape a UART
 * gives you - and because it makes a truncated frame a nothing rather than a
 * wrong answer.
 */

#define AK_PROTO_SYNC1 0xAAu
#define AK_PROTO_SYNC2 0x55u
#define AK_PROTO_VERSION 1u
#define AK_PROTO_MAX_PAYLOAD 96u
#define AK_PROTO_FRAME_MAX (6u + AK_PROTO_MAX_PAYLOAD + 2u)
#define AK_PROTO_RESPONSE_BIT 0x80u
#define AK_PROTO_GAP_MS 50u

enum {
    AK_PROTO_CMD_HELLO = 0x01,
    AK_PROTO_CMD_PARAM_GET = 0x02,
    AK_PROTO_CMD_PARAM_SET = 0x03,
    AK_PROTO_CMD_PARAM_SAVE = 0x04,
    AK_PROTO_CMD_STATUS = 0x05,
    AK_PROTO_CMD_LOG_INFO = 0x06,
    AK_PROTO_CMD_LOG_GET = 0x07,
    /* Ask for a telemetry stream at this rate in Hz, 0 to stop. The reply
     * acks the rate that was accepted. */
    AK_PROTO_CMD_TELEMETRY = 0x08,
    /* Which log the next LOG_INFO and LOG_GET are about. A device has three
     * answers to "what happened": the fast ring in RAM, the long one that
     * survives a reset, and the one in flash that survives the battery. The
     * reply says which one is now selected and how many records it holds, so a
     * tool needs one round trip rather than two.
     *
     * LOG_GET's index is *within the selected log*, oldest first, which is
     * what makes the three of them the same interface from a client's side. */
    AK_PROTO_CMD_LOG_SOURCE = 0x09,
    /* What a parameter *is*, as opposed to what it holds: its type, its group,
     * its bounds and its default. PARAM_GET has always answered the value and
     * the console has always printed all of this beside it, but the wire
     * carried none of it - so every client either showed a value with no range
     * or invented one, and a screen that guessed a group from a name prefix is
     * what the `group` field was added to stop.
     *
     * Paged, because an entry is a name and three numbers spelled out and the
     * frame is 96 bytes. `carried` in the reply is what was actually written,
     * so a page that ran out of room says so rather than reporting a short
     * list as a complete one. */
    AK_PROTO_CMD_PARAM_INFO = 0x0A,
    /* The long prose beside a parameter, fetched per row and on demand. It is a
     * separate command because it is the one field nobody needs for all ninety
     * at once, and because it is the only one with no useful bound on its
     * length - so it is walked by offset rather than paged by count, and no
     * part of it is ever silently cut off. */
    AK_PROTO_CMD_PARAM_HELP = 0x0B,
    /* Put parameters back to the defaults this build was compiled with.
     *
     * `mode` selects what is reset: 1 is one parameter, named by the `index`
     * that follows; 2 is the whole table. **A bare request is refused** - an
     * empty frame meaning "wipe every parameter" is a booby trap, and the one
     * command on this wire that a truncated frame could turn into a factory
     * reset is not a command to give a default to. Mode 0 is not "all"; it is
     * a request that named nothing.
     *
     * Unlike the console's `defaults`, this refuses while the aircraft is
     * armed - see `ak_proto_io_t::writable`. The two disagree on purpose and
     * docs/16-protocol.md says why. */
    AK_PROTO_CMD_PARAM_DEFAULT = 0x0C,
    /* What the receiver is hearing, right now: the counts as they arrived, the
     * sticks those counts mean, and enough counters to tell "no receiver" from
     * "a receiver saying nonsense".
     *
     * Polled, never streamed, and that is a decision rather than an omission.
     * The console link carries no stream at all (`can_stream`), and the one tab
     * a person most wants while holding a transmitter is exactly the one that
     * should work on the cable. A client that wants it repeatedly asks
     * repeatedly; at the rates a handset moves, that costs nothing.
     *
     * **The firmware decodes the sticks.** `rc_min`, `rc_mid`, `rc_max` and
     * `rc_deadband` are parameters, and an app that recomputed roll and pitch
     * from them would be a second authority on the one number that decides
     * whether the aircraft is being commanded - two implementations of
     * `centred()` in two languages, disagreeing at the deadband edge, with the
     * screen and the airframe each believing its own. The counts are sent too,
     * because a count out of range is the first sign of a receiver on the wrong
     * baud rate, but they are sent as evidence and not as an input.
     *
     * A request has no arguments. There is nothing to name: this is what the
     * receiver is doing, not a query about a channel. */
    AK_PROTO_CMD_RC_CHANNELS = 0x0D,
    /* What the board is *hearing* rather than what it is doing: the IMU, the
     * barometer, the rangefinder, the flight pack and the GPS, each in the
     * terms its own driver reports it.
     *
     * Named by topic rather than sent as one frame, because the five have
     * nothing in common but the question. A combined reply would be either
     * larger than the frame or missing whichever sensor the board happens to
     * carry, and a client that has to ask about a board with no rangefinder
     * every time it wants the battery is a client that reads four fifths
     * nothing.
     *
     * **The absent case is the reason this opcode is shaped the way it is.**
     * The console has always answered "none fitted" in words, and a wire that
     * answered a struct of zeros instead would be saying "this board has a
     * barometer and it reads zero pressure" - which is not a missing sensor,
     * it is a sensor that has failed, and the two call for different afternoons
     * of work. So `present` is a byte on every topic and the body is *absent*,
     * not zeroed, when it is clear: a body that is not there cannot be mistaken
     * for a reading of zero.
     *
     * A board with no sensor reporting at all leaves `sensor_state` null, and
     * every topic then answers AK_PROTO_SENSOR_NO_SUCH - which is true of it,
     * and is why there is no separate "this board has no sensors" status to
     * get wrong. */
    AK_PROTO_CMD_SENSOR_INFO = 0x0E,
    /* A range of one log, pushed rather than asked for one record at a time.
     *
     * `LOG_GET` works and is the command a tool should reach for first, but it
     * costs a round trip per record and the records are 87 bytes: reading the
     * flash log - 5 460 of them on the F405 - is 5 460 round trips, and on a
     * 115200 cable that is twenty minutes of asking. Batching is not available
     * to fix it: two records do not fit in a 96-byte frame, and `LOG_GET`
     * carries no sequence number, so a client cannot reassemble a batch it
     * cannot see the shape of.
     *
     * So the request names a range and a rate, the reply acks what was
     * accepted, and the records arrive as frames with **no response bit** -
     * the same mechanism `TELEMETRY` uses, and gated by the same `can_stream`,
     * because a console link is a person's wire and a stream there arrives
     * between their keystrokes.
     *
     * **Every frame names its own index and its own source**, which is the
     * one place this departs from `LOG_GET`'s reply shape and is a deliberate
     * departure: a reply does not need to say what it is answering, and a push
     * does, because nothing above it ties the frame to a request. It is what
     * lets a client count holes without a second command: a record the board
     * will not give - a slot the power cut in half - arrives as a frame at
     * that index with status `HOLE` and no body, rather than as a gap in a
     * sequence the client is inferring.
     *
     * **And it ends out loud.** The last frame carries status `DONE`. A
     * stream that simply stopped would be indistinguishable from a link that
     * died, and "the read finished" and "the board went away" call for
     * different next actions. */
    AK_PROTO_CMD_LOG_STREAM = 0x11,
    /* What this board's outputs *are*, as the bench instrument needs them: one
     * descriptor per output, in the board's own numbering, carrying the
     * plumbing a person sets rather than the value being driven.
     *
     * **The value is deliberately not here.** What moves an output is the
     * flight core or the test below, and a client that could read "motor 2 is
     * at 40%" would be reading a number that belongs to whichever of those ran
     * last. What is here is the part that is a property of the aircraft: which
     * outputs exist, and for a servo - the one output with a linkage behind it
     * - whether it is reversed, where its centre is, and how far it travels.
     * The console has printed that list since it had an `output` command; this
     * is the same list with the prose taken off, which is why the two cannot
     * disagree about an aircraft.
     *
     * **No test verb here, ever.** This opcode is a read. The `output test`
     * half of the console's command is the next number along, with its own
     * gate and its own argument. Folding them together would put a verb that
     * moves a motor behind the same number a client polls to draw a table, and
     * no client polls something that can spin a propeller.
     *
     * A request has no arguments: the largest board in this tree has four
     * motors and two servos, and one frame holds thirteen descriptors. A board
     * that one day holds more gets a `first` argument then, which is a change
     * an old client survives - it sends an empty frame and reads the page it
     * has always read. */
    AK_PROTO_CMD_OUTPUT_INFO = 0x0F,
    /* Drive one output, for as long as somebody is holding a button.
     *
     * **This overturns a recorded refusal, deliberately.** The rule has been
     * that nothing on this wire sets an output to a value of your choosing;
     * the console's `output test` - the command that walks M1..M4 and the two
     * servos one at a time so a scope can say which pad is which - has never
     * been reachable from a socket, and `docs/06-console.md` names that
     * omission as one of the two things arming from a console would need. The
     * owner took the opposite decision for a **typed, disarmed-only,
     * hold-to-run** form, and the properties below are what makes that safe
     * rather than merely intended. They are written down here because the next
     * reader will otherwise find the old refusal and "fix" this.
     *
     * **Refused unless the aircraft is disarmed, checked at the moment of each
     * command rather than at connect.** A gate checked once is a gate that is
     * wrong a second later, and arming can happen between two frames.
     * `io->writable` is the same predicate the console's `save`, the airborne
     * routes and the configuration writes use; this opcode adds no second
     * opinion about what "safe to write" means.
     *
     * **One output at a time, named in the request and echoed in the reply.**
     * There is no "all motors" form and there will not be one: the console's
     * sweep is safe because it is a bench tool on a cable, and this is on a
     * socket. A client that wants to walk the outputs walks them.
     *
     * **`level` is a percentage of a cap the firmware owns**, and the cap is
     * sent in the reply rather than being a number the client knows. A client
     * that asks for more than the cap is given the cap and told so, rather than
     * being refused or obeyed - the same shape as the log stream's clamped
     * rate, and for the same reason: the reply is what will happen, not an
     * echo of what was asked.
     *
     * **It times out.** A hold that stops arriving is a client that has gone
     * away - a closed tab, a link that died - and the output returns to zero on
     * its own after `AK_PROTO_OUTPUT_TEST_MAX_MS`. The reply carries
     * `remaining_ms` so a screen reads the deadline from the board's number
     * rather than inventing one, which is the metadata walk's discipline
     * applied to a clock.
     *
     * **`remaining_ms` is the whole window, not a countdown, and a screen must
     * not draw it as one.** The request *is* the renewal, so every reply to a
     * renewal restarts the window: a client under a held button is told 500 ms
     * over and over and the number never falls. A configurator that printed
     * `untilMs - now` as a countdown therefore showed a hold ticking towards a
     * stop that was not coming - measured against its own once-a-second page
     * clock it read "another 1313 ms" about this 500 ms window. The honest
     * sentence is the window itself ("stops 500 ms after the last request"), and
     * the deadline is for the client to *compare* against, not to display.
     *
     * **`op` is 0 for start-or-hold and 1 for stop**, so a client that has lost
     * track of the state can always send a stop without knowing whether one is
     * running. A stop that was refused because nothing was running would be a
     * stop a client could not send, which is the wrong way round for the one
     * verb whose whole job is to be available. */
    AK_PROTO_CMD_OUTPUT_TEST = 0x10,
    /* The board's own preflight checklist, the same one the console's
     * `preflight` command prints, page by page.
     *
     * **A read, and the most valuable one on this wire.** Everything else a
     * client can ask is about the configuration or the sensors; this is the
     * only opcode that answers "is this machine what the firmware thinks it
     * is", which is the question a person asks before a first flight and the
     * one they cannot answer by looking at the aircraft. The console has had
     * it since it had the command; the sentences are the console's, and that
     * is deliberate - see below.
     *
     * **One checklist, two renderings.** The console prints these lines; this
     * opcode sends them. They are built once, by one function, into one record
     * that both paths read - not written twice and kept in step by hand. The
     * protocol therefore cannot report a machine that is cleaner than the one
     * the console describes, which is the failure a second implementation
     * would have on the day somebody added a check to one of them.
     *
     * **`detail` is the console's sentence verbatim, name and all.** A client
     * that renders `name` as a row heading and `detail` underneath it shows
     * the same words a person reading the console sees, and a firmware that
     * reworded one without the other would fail a host test rather than
     * shipping two vocabularies for one fact. The `name` is the console's own
     * leading word where it had one (`gyro bias`, `saved configuration`) and a
     * short token where the sentence never named its subject (`mix`, `tick`),
     * because a client keys rows on it and a key has to be stable across
     * wordings.
     *
     * **`verdict` is three-valued, and the third value is the point.** `FAIL`
     * is a check that did not pass and counts towards the summary. `PASS` is a
     * check that did. `FACT` is a line that is neither - the gyro bias, the
     * fitted sensors, the battery, the arming gate's answer - and a client that
     * rendered those as passes would be claiming the firmware verified
     * something it only observed. The console has drawn that distinction with
     * `--` since it had the command; this is the same distinction with a name.
     *
     * **Paged by request, not pushed.** A request is `index(u8), offset(u16
     * LE)`; the reply carries the line's `name`, its `verdict`, the whole
     * sentence's length and one part of it. Walking `offset` is how a sentence
     * longer than a frame arrives whole - the same offset walk PARAM_HELP uses,
     * and for the same reason: an entry that cannot fit must not be silently
     * cut, because half a sentence about a fault reads as a different fault.
     *
     * **The record is rebuilt when `index` is zero**, which is the first
     * request of a walk and only that one. A checklist read over eight pages
     * takes seconds during which a fault can arrive or a battery can be
     * plugged in; rebuilding per page would describe eight different machines
     * in one report. The rebuild is on the client's first request rather than
     * on a timer, so two clients cannot see each other's snapshot and neither
     * can see a stale one. */
    AK_PROTO_CMD_PREFLIGHT = 0x12,
    /* The calibrations the console already has, behind one opcode with a verb.
     *
     * `calibrate gyro`, `calibrate rc`, `calibrate vbat` and `calibrate accel`
     * have been on the console since it had one, and every one of them measures
     * the aircraft rather than setting it: what the gyro reads when nothing is
     * moving, what the receiver calls centred, what the pack's divider actually
     * is, and which way up the board is. None of it was reachable over the
     * wire, which is why no configurator could offer a calibration, and it is
     * the last of the console's bench commands to be carried across.
     *
     * **It does not block, and that is the design rather than an optimisation.**
     * The console's version of each of these sits in a loop - a thousand
     * milliseconds for the receiver, up to three seconds for the six
     * accelerometer faces - because the console is a human wire and the person
     * watching it is the one who asked for the measurement. This opcode is
     * dispatched from the flight loop, and a calibration that held that loop
     * for three seconds would stop the stabiliser, the telemetry and the
     * failsafe timer for exactly as long as a person was willing to hold an
     * aircraft on its side. So the board owns the session: a verb starts one,
     * the flight loop advances it a sample at a time, and the STATUS verb reads
     * it. The precedent is already in main.c - the gyro bias measured while
     * disarmed is fed from the loop and not from whatever asked for it.
     *
     * **Every reply is a reading of the session, not an acknowledgement.** The
     * verb that ran, whether a session is live, how many samples it has taken,
     * how many it refused for moving, and the measurement so far. That is
     * MISSION's shape and MISSION's reason - a client that has just sent a verb
     * should be able to draw the board's own answer without a second round trip
     * that can fail on its own - and it is the only shape that lets a wizard
     * show progress, since the alternative is a client guessing at a duration
     * the board never promised. `samples` and `rejected` are the two numbers
     * the plan asked for by name: a calibration that quietly averaged in
     * samples taken while the aircraft was moving is worse than one that failed,
     * so the refusals are reported rather than hidden.
     *
     * **Every verb here ends in a parameter write, so `io->writable` gates all
     * of them** - `gyro_bias_*`, `rc_mid`, `vbat_ratio`, `accel_bias_*` and
     * `accel_scale_*` are ordinary table entries, range-checked and kept by
     * `save` like any other. That is not the write gate wearing a different hat;
     * it is the same gate, and the console already refuses all four while armed
     * for the reason main.c's accel comment gives: holding an armed aircraft
     * through six attitudes is a hand near a live throttle.
     *
     * **Whether the pilot meant it is not a refusal here.** The six faces are
     * six commands and each one is one click from a wrong bias, so the app puts
     * a step in front of each - but that is the confirmation MISSION's comment
     * describes, and it belongs in front of the button, where a person is. The
     * firmware's refusals are the ones only the aircraft knows: armed, already
     * running, nothing to calibrate, or not enough still samples. */
    AK_PROTO_CMD_CALIBRATE = 0x13,
    /* The mission: the list of places to go, and the verbs that start and stop
     * going to them.
     *
     * **The first opcode here whose verb changes what the aircraft will do in
     * the air.** Everything before it changes what the firmware remembers (a
     * parameter, a save), reads a fact (a sensor, a checklist, a log), or moves
     * one pad on a bench (OUTPUT_TEST, which cannot outlive a link). `start`
     * asks the navigator to fly the aircraft to a list of positions, and it is
     * the nearest thing on this wire to an irreversible act - so it is the one
     * opcode whose reply is a *state* rather than an acknowledgement, and the
     * one the app puts a confirmation in front of.
     *
     * **Waypoints are not here, and that is the design.** They are parameters -
     * `wp0_lat`, `wp0_lon`, ... and `wp_count` - so they are already on this
     * wire through PARAM_SET, they are range-checked by the table, they appear
     * in a backup, and `save` keeps them. The console's `mission add` is a
     * convenience that writes those same two parameters rather than a second
     * kind of storage. An `add` verb here would be a second way to write them,
     * and the two would disagree the first time one grew a bound the other did
     * not.
     *
     * **`requested` and `active` are two different facts and both are sent.**
     * The console's `mission start` does not start anything: it sets a request,
     * and the flight loop starts the mission when the aircraft is armed and
     * flying. A reply that carried only `active` would show a client "not
     * flying" immediately after a start that succeeded, and a reply that
     * carried only `requested` would make the app's sentence the console's -
     * which says "flying" about an aircraft sitting on a bench. Neither is what
     * happened; the two fields are.
     *
     * **Every reply carries the whole state, on every verb.** A client that
     * has just sent `start` gets the mission as the board now holds it, rather
     * than an echo of what it asked for or a second round trip that could fail
     * on its own. It is what makes a screen that cannot show a stale panel, and
     * it is why the reply is one shape for all five verbs.
     *
     * **The refusals are the ones the firmware owns, and only those.** An empty
     * list has nothing to fly and a home cannot be set from a fix the navigator
     * will not use - both are answers about this aircraft that a client cannot
     * compute. Whether the pilot meant it is not one of them: that is the
     * confirmation, and it belongs in front of the button, where a person is. */
    AK_PROTO_CMD_MISSION = 0x14,
    /*
     * Where the loop's time went, as a read.
     *
     * The one opcode here that changes nothing on the aircraft: it asks the
     * board what its own control loop cost, and the answer is the same window
     * the console's `perf` prints. Status is the *measurement's* status, not a
     * refusal - there is nothing to refuse - and the two values are "here is the
     * window" and "this build has no profiler in it". The second is a real
     * answer and not an error: a firmware built without one is a firmware whose
     * loop cost nobody is asking about, and a client must be able to tell that
     * from a board whose loop is genuinely costing nothing.
     *
     * **`loops` and `samples` are both on the wire, deliberately.** A port whose
     * clock reads zero can still count that a period closed, so the two diverge
     * exactly when the port cannot measure - and a client comparing a period
     * against a loop count has no way to see that from one number. Sending both
     * makes "measuring" and "only counting" a reading rather than an inference.
     *
     * The section figures are averaged over the window, per *loop* rather than
     * per time the section ran, because that is the unit a budget is spent in:
     * a section that runs on one loop in ten costs a tenth of the number here
     * on every loop. The load is the sum of those against the nominal slot and
     * is a **lower bound** - the console, the links and the navigation are not
     * instrumented - which the reply's own name for the field says.
     */
    AK_PROTO_CMD_PERF = 0x15,
    /*
     * How fast each motor is turning, as a read.
     *
     * The board has been able to hear an ESC since `ak_dshot_capture.c` and turn
     * a reply into an eRPM since `ak_dshot_gcr.c`; what none of that had was a
     * way off the board. `ak_proto_status_t` carries `motor[4]`, and every byte
     * of it is the output this firmware *asked* for - a command, never a
     * reading - so a client watching a motor stop and a client watching a motor
     * the board has no idea about drew the same four bars.
     *
     * **A new opcode and not a field appended to STATUS.** The version byte
     * moves only when an existing byte changes meaning, and appending changes
     * the meaning of the *length* under every client written before it. A new
     * command is a new command: an old client never sends it and an old board
     * answers 0x7F, which is already a thing this protocol says.
     *
     * **Polled and never streamed**, for RC_CHANNELS' reason exactly: the
     * console link cannot stream at all, and a board on a cable is precisely
     * where somebody is holding a motor in their hand.
     *
     * The status byte carries the two answers that must not be confused - see
     * AK_PROTO_MOTOR_* below - and per motor the flags carry the rest.
     *
     * **eRPM and RPM are both here, and so is the pole count they were divided
     * by.** eRPM is the ESC's own number: it is arithmetic-free, it is what the
     * notch bank in `ak_rpm_filter` consumes, and it is the only figure a board
     * with no pole count can honestly send. RPM is the number a person wants and
     * it is `ak_dshot_rpm(erpm, poles)` - a division by a constant the *board*
     * has to be told (`motor_poles`), because the console's `dshot` command
     * makes the same demand and refuses to assume the fourteen poles an
     * aircraft motor usually has. Sending both, plus the pole count used, is
     * what keeps the app from doing arithmetic on a number whose meaning it
     * cannot check: a client shows eRPM always, shows RPM when the board says
     * it knows the poles, and otherwise prints the console's own sentence.
     *
     * Temperature, voltage and current are the extended-telemetry fields
     * `ak_dshot_edt.c` already decodes. They are measurements and never
     * verdicts: which of them is worth colouring red is the app's business.
     * There is deliberately **no motor-health field here yet** - that module
     * decides a verdict from a window of samples, and a verdict is a separate
     * question from a reading, with its own argument to make.
     */
    AK_PROTO_CMD_MOTOR_TELEMETRY = 0x16,
};

/* Whether a preflight line is a check that failed, a check that passed, or a
 * fact the firmware observed without judging it. See the opcode above for why
 * there are three and not two. */
enum {
    AK_PROTO_PREFLIGHT_VERDICT_FAIL = 0,
    AK_PROTO_PREFLIGHT_VERDICT_PASS = 1,
    AK_PROTO_PREFLIGHT_VERDICT_FACT = 2,
};

/* The status byte on PREFLIGHT.
 *
 * `NONE` and `NO_INDEX` are different answers and both are worth being able to
 * give: the first is a board whose firmware has no checklist at all - a board
 * built without the run, or a device that is not an aircraft - and the second
 * is a client that has walked past the end. A board that answered `NONE` for
 * both would leave the client unable to tell a firmware it should upgrade from
 * its own arithmetic. */
enum {
    AK_PROTO_PREFLIGHT_OK = 0,
    AK_PROTO_PREFLIGHT_NO_INDEX = 1,
    AK_PROTO_PREFLIGHT_NONE = 2,
};

/* The most lines a board may report, and a bound rather than a copy of any
 * one board's count: the largest checklist in this tree is well under it, and
 * a build that grew past it fails a host test rather than truncating a
 * checklist - which is the one thing this opcode must never do, since the
 * missing line could be the failing one. */
#define AK_PROTO_PREFLIGHT_MAX 32u

/* The longest name a line may carry, and the longest sentence. Both are
 * checked by a host test against the real checklist rather than trusted, so a
 * sentence that outgrew the buffer fails the build's tests rather than being
 * cut on a wire whose whole purpose is saying what is true. */
#define AK_PROTO_PREFLIGHT_NAME_MAX 24u
#define AK_PROTO_PREFLIGHT_DETAIL_MAX 120u

/* One line of the checklist, as the firmware builds it and as PREFLIGHT sends
 * it. `detail` points into the firmware's own record and is NUL-terminated;
 * the wire copy is length-prefixed instead, because a name and a sentence are
 * not the place to be spending a byte on a terminator. */
typedef struct {
    const char *name;
    uint8_t     verdict;
    const char *detail;
} ak_preflight_line_t;

/* What MISSION is asked to do. `STATUS` is the only read; the rest are the
 * console's verbs, one number each.
 *
 * There is no `add` and no `clear` here: the list is the parameter table's (see
 * the opcode), and a client adds a waypoint by writing `wp<N>_lat` and
 * `wp<N>_lon` and then `wp_count` through PARAM_SET, where the range check, the
 * changed count and the saved record already are. */
#define AK_PROTO_MISSION_STATUS     0u
#define AK_PROTO_MISSION_START      1u
#define AK_PROTO_MISSION_STOP       2u
#define AK_PROTO_MISSION_HOME_SET   3u
#define AK_PROTO_MISSION_HOME_CLEAR 4u

/* The status byte on MISSION.
 *
 * Five refusals, and each is a fact the firmware holds rather than an opinion
 * about intent. `NO_WAYPOINTS` and `NO_FIX` are answers a client cannot compute
 * - a client can count `wp_count` itself, but only the board knows whether the
 * navigator would accept those positions, and after a PARAM_SET race the two
 * would differ. `NO_NAV` is for a build with no navigator at all, which is the
 * same shape as PREFLIGHT's `NONE`: a client that got it should say the board
 * cannot do this, not show an empty mission. */
enum {
    AK_PROTO_MISSION_OK = 0,
    AK_PROTO_MISSION_NO_VERB = 1,      /* refused: no such op */
    AK_PROTO_MISSION_NO_NAV = 2,       /* refused: this board has no navigator */
    AK_PROTO_MISSION_NO_WAYPOINTS = 3, /* refused: nothing to fly */
    AK_PROTO_MISSION_NO_FIX = 4,       /* refused: home needs a usable fix */
};

/* The mission, as a caller fills it in and as every MISSION reply carries it.
 *
 * `index` is the waypoint being flown and is `AK_PROTO_MISSION_NO_INDEX` when
 * the navigator is not flying one - a value out of range for a four-waypoint
 * list, because "not flying a waypoint" and "flying waypoint zero" are the two
 * ends of a mission and a client that showed them the same way would draw the
 * first waypoint as the one in progress. */
#define AK_PROTO_MISSION_NO_INDEX 0xFFu

typedef struct {
    uint8_t  active;      /* the navigator is flying a mission now */
    uint8_t  requested;   /* a mission has been asked for, and not taken back */
    uint8_t  count;       /* waypoints in the list */
    uint8_t  index;       /* the waypoint being flown, or NO_INDEX */
    uint8_t  channel;     /* the mission switch's channel, or 0 for none */
    uint16_t reached;     /* waypoints reached on the running mission */
    uint16_t started;     /* missions started since power on */
    uint16_t cancelled;   /* missions taken back since power on */
    int32_t  hold_alt_mm; /* the altitude a running mission holds */
} ak_proto_mission_t;

/* What CALIBRATE is asked to do. `STATUS` is the only read; the rest start a
 * session, or end one.
 *
 * These are the console's four calibrations and its verbs, one number each, and
 * the mapping is deliberately the obvious one: a client that knows what
 * `calibrate accel 3` does knows what ACCEL with face 3 does, because it is the
 * same measurement reaching the same parameter names.
 *
 * `ABORT` exists because a session here can outlive the command that started
 * it. The console's version has no need of it - a person waiting for a blocked
 * loop can only wait - but a wizard whose user has just picked the aircraft up
 * needs to be able to stop the sampling without powering the board down, and a
 * session that could only be ended by finishing would refuse the next start
 * with BUSY forever. */
#define AK_PROTO_CALIBRATE_STATUS 0u
#define AK_PROTO_CALIBRATE_GYRO   1u
#define AK_PROTO_CALIBRATE_RC     2u
#define AK_PROTO_CALIBRATE_ACCEL  3u
#define AK_PROTO_CALIBRATE_VBAT   4u
#define AK_PROTO_CALIBRATE_ABORT  5u

/* The status byte on CALIBRATE.
 *
 * Every one of these is a fact the aircraft holds rather than an opinion about
 * what the pilot meant, which is the same test MISSION's refusals are held to.
 *
 * `NOTHING` and `NO_SAMPLES` are the pair worth keeping apart, and they are the
 * pair the console already keeps apart. The first is a fact about the *build* -
 * `calibrate vbat` on a board with no pack divider, `calibrate rc` on a board
 * with no receiver port - and no amount of trying will change it. The second is
 * a fact about the *aircraft*: the hardware is there and it would not hold
 * still, or nothing arrived on the wire. A client that showed them the same way
 * would send a person to look at a connector that was never fitted.
 *
 * **The comments below are paraphrases.** The wire carries the status byte and
 * nothing more, so the sentence a client shows is the client's: the ten
 * sentences themselves are `docs/16-protocol.md`'s status table and
 * `tools/akproto.py`'s `CALIBRATE_STATUS_NAMES`, character for character, with
 * the web configurator carrying a third copy that a test holds to both. These
 * are here to keep the enum readable, and one of them was wrong for a while —
 * `IMPLAUSIBLE` said "the six faces are not a gravity", which is only the
 * accelerometer's way in; `main.c` also returns it when a gyro or an RC
 * calibration's numbers will not go into the parameter table. */
enum {
    AK_PROTO_CALIBRATE_OK = 0,
    AK_PROTO_CALIBRATE_NO_VERB = 1,      /* refused: no such verb */
    AK_PROTO_CALIBRATE_ARMED = 2,        /* refused: the aircraft is armed */
    AK_PROTO_CALIBRATE_BUSY = 3,         /* refused: a session is already running */
    AK_PROTO_CALIBRATE_NOTHING = 4,      /* refused: nothing on this board to calibrate */
    AK_PROTO_CALIBRATE_NO_SAMPLES = 5,   /* ran, and did not get enough still samples */
    AK_PROTO_CALIBRATE_IMPLAUSIBLE = 6,  /* ran, and the result is not plausible */
    AK_PROTO_CALIBRATE_IDLE = 7,         /* refused: nothing to abort */
    AK_PROTO_CALIBRATE_NO_FACE = 8,      /* refused: no such accelerometer face */
    AK_PROTO_CALIBRATE_BAD_VALUE = 9,    /* refused: the number given is not a pack */
};

/* How many numbers a measurement carries, and it is the accelerometer's six
 * that set it: three of bias and three of scale, because that is the one
 * calibration here that measures a scale as well as an offset. The other three
 * use a prefix of it and zero the rest, which is a real zero - "this
 * measurement has no fourth number" - and not a placeholder.
 *
 * The meaning of the slots is fixed by the verb that filled them, and the verb
 * is in the reply beside them, so there is nothing to infer. Every slot is
 * fixed-point, for the reason the whole protocol is: a float on the wire is a
 * byte order and a rounding mode agreed by two compilers.
 *
 *   GYRO   [0..2] bias, millidegrees per second
 *   RC     [0]    the centre that was applied, microseconds
 *          [1..3] how far roll, pitch and yaw were off it, microseconds
 *   VBAT   [0]    the ratio measured, times 1e6
 *   ACCEL  [0..2] bias, micro-g
 *          [3..5] scale, times 1e6
 */
#define AK_PROTO_CALIBRATE_RESULT 6u

/* The step byte when no face is being sampled: out of range for all six, so a
 * client cannot read "not sampling" as "sampling face zero" - the same trick
 * AK_PROTO_MISSION_NO_INDEX plays, and for the same reason. */
#define AK_PROTO_CALIBRATE_NO_STEP 0xFFu

/* The verb byte when there is no session to name, which is a board that has
 * never calibrated anything. Out of range for all six for NO_STEP's reason, and
 * it is the one that would bite hardest: the verb is what tells a client how to
 * read the six result slots, and a fresh board reading zero would be saying
 * "these are the status verb's numbers" beside six zeros that are not a
 * measurement of anything. `0xFF` is already the byte this opcode's dispatch
 * answers when a client named no verb, so it is the same "not a verb" a caller
 * has met in a refusal. */
#define AK_PROTO_CALIBRATE_NO_SESSION 0xFFu

/* The calibration, as a caller fills it in and as every CALIBRATE reply carries
 * it. See the opcode for why the reply is this and not an acknowledgement.
 *
 * `faces` is a bitmask of the six accelerometer faces already measured, and it
 * is the one field here that describes a *previous* command as much as this
 * one: the six-face flow is six commands that accumulate, so a wizard's whole
 * state is this byte plus the step. It is meaningful only for ACCEL and reads
 * zero for the other three, which is true rather than a placeholder - a gyro
 * calibration has no faces. */
typedef struct {
    uint8_t  active;   /* a session is sampling now */
    uint8_t  verb;     /* which one, as the AK_PROTO_CALIBRATE_* numbers */
    uint8_t  step;     /* the face being sampled, or NO_STEP */
    uint8_t  faces;    /* bitmask of the faces measured so far */
    uint32_t samples;  /* samples taken toward this session */
    uint32_t rejected; /* samples refused for moving, or for a bad frame */
    int32_t  result[AK_PROTO_CALIBRATE_RESULT];
} ak_proto_calibration_t;

/* The status byte on PERF.
 *
 * `NONE` is not an error and not an empty window: it is a *build* with no
 * profiler in it, which is a fact about the firmware rather than about the
 * aircraft. The distinction matters for the same reason it does everywhere else
 * in this file - a client that rendered a window of zeros as "the loop costs
 * nothing" would be reporting the opposite of what the board just said. */
enum {
    AK_PROTO_PERF_OK = 0,
    AK_PROTO_PERF_NONE = 1,
};

/* The sections a PERF reply carries, in the order they are sent.
 *
 * A copy of AK_PERF_NAMED_SECTIONS rather than an include of ak_perf.h,
 * because this header is the wire's description and the profiler's enum is an
 * implementation of it - and a host test asserts the two agree, so a section
 * added to the profiler and forgotten here is a failing build rather than a
 * reply that is one field short.
 *
 * Five and not six: the profiler's enum also holds AK_PERF_NONE, the state
 * between sections, which is work that has not been divided up and so has no
 * name and no number to send. */
#define AK_PROTO_PERF_SECTIONS 5u

/* The profiler's window, as PERF sends it. Every field is the console `perf`
 * command's own number, in the units the wire carries them.
 *
 * The section averages are in **tenths of a microsecond**, which is the one
 * place this struct changes units: a nanosecond figure does not fit a u16 for
 * any section that matters, and a whole microsecond would hide the difference
 * between a loop that is comfortable and one that is nearly out of slot. The
 * maxima stay whole microseconds, matching the console, and the load is
 * per-mille of the nominal slot, because "12.3 %" and "123 per-mille" are the
 * same number and only one of them needs a decimal point on a wire. */
typedef struct {
    uint32_t loops;    /* periods closed since the window began */
    uint32_t samples;  /* how many of those the port could time */
    uint16_t nominal_us;
    uint32_t period_last_us;
    uint32_t period_min_us;
    uint32_t period_max_us;
    uint32_t late;
    uint16_t jitter_p50_us;
    uint16_t jitter_p99_us;
    uint16_t jitter_max_us;
    uint32_t jitter_over;
    uint16_t section_avg_us_x10[AK_PROTO_PERF_SECTIONS];
    uint16_t section_max_us[AK_PROTO_PERF_SECTIONS];
    uint16_t load_permille;
} ak_proto_perf_t;

/* The status byte on MOTOR_TELEMETRY.
 *
 * The same two answers RC_CHANNELS distinguishes, about a different port.
 * `NONE` is this build having no way to hear an ESC at all - compiled without a
 * bidirectional DShot path, or a board with no motor outputs to hear - and its
 * reply is one byte and stops, because everything after it would be a claim
 * about hardware this board does not have, and a frame of zeros reads on a
 * screen as four stopped motors rather than as no telemetry path.
 *
 * A board that *has* the path and has heard nothing yet answers `OK` with every
 * motor's MEASURED flag clear. That is a different state of a different
 * aircraft and it gets a different sentence. */
enum {
    AK_PROTO_MOTOR_OK = 0,
    AK_PROTO_MOTOR_NONE = 1,
};

/* The most motors a reply can carry, and a bound on the firmware's own count
 * rather than a copy of it, exactly as AK_PROTO_RC_MAX is: AK_MAX_MOTORS is the
 * flight core's number and this is the wire's, so a board that grew a motor
 * could keep a client working by sending the first AK_PROTO_MOTOR_MAX of them.
 * A client that needs to know whether it was shown all of them asks
 * OUTPUT_INFO, whose motor count is the board's own. */
#define AK_PROTO_MOTOR_MAX 4u

/* Facts about one motor, as bits. Every one of them says that a measurement
 * happened; not one of them is a judgement, and none is ever a warning. */
#define AK_PROTO_MOTOR_FLAG_MEASURED (1u << 0)
/* The `rpm` field is a number rather than a placeholder. Set only when MEASURED
 * is set *and* the board was told the motor's pole count: an eRPM with no pole
 * count is a true reading of something that is not a speed, and a screen that
 * divided by an assumed fourteen would be inventing the figure it shows. */
#define AK_PROTO_MOTOR_FLAG_RPM (1u << 1)
#define AK_PROTO_MOTOR_FLAG_TEMPERATURE (1u << 2)
#define AK_PROTO_MOTOR_FLAG_VOLTAGE (1u << 3)
#define AK_PROTO_MOTOR_FLAG_CURRENT (1u << 4)

/* One motor's telemetry, as MOTOR_TELEMETRY sends it.
 *
 * Every field beside the flags is meaningful only when its flag is set. "A
 * temperature of zero" and "no temperature has been heard" are the same byte
 * otherwise, and a screen that drew them the same way would be reporting an ESC
 * that is cold when the board has never heard from it at all - the rule the
 * receiver's DECODED bit and the sensor topics' `present` byte already follow.
 *
 * `packets` and `invalid` are the quality window's two counts over the window
 * `ak_dshot_edt.h` defines (ten buckets of AK_DSHOT_EDT_BUCKET_MS each). Zero
 * packets is a true answer and not an absence: the window is a window, MEASURED
 * is how a client tells "nothing has been heard" from "nothing has been heard
 * lately", and the ratio is the ERPM CRC rate the roadmap's acceptance clause
 * asks to be under 1 %. */
typedef struct {
    uint8_t  flags;
    uint32_t erpm;
    uint32_t rpm;             /* meaningful when FLAG_RPM is set */
    uint8_t  temperature;
    uint8_t  max_temperature; /* the session's maximum, as ak_dshot_edt.h holds it */
    uint16_t millivolts;
    uint16_t milliamps;
    uint16_t packets;
    uint16_t invalid;
} ak_proto_motor_t;

/* The whole reply's body: how many motors follow, and the pole count they were
 * converted with.
 *
 * `poles` is carried rather than inferred because it is the one number that
 * decides whether `rpm` means anything, and a client that had to guess it would
 * be a second authority on a question the board has already answered. Zero
 * means the board does not know - which is the state of every board in this
 * tree today, because no `motor_poles` parameter exists yet - and then FLAG_RPM
 * is clear on every motor and `rpm` is the placeholder the flags say it is. */
typedef struct {
    uint8_t count;   /* how many entries the board filled, at most AK_PROTO_MOTOR_MAX */
    uint8_t poles;   /* 0 = this board does not know */
    ak_proto_motor_t motor[AK_PROTO_MOTOR_MAX];
} ak_proto_motor_telemetry_t;

/* The status byte on RC_CHANNELS.
 *
 * The second value is the one worth having. A board with no receiver port could
 * answer a frame of zeros, and that frame would be indistinguishable from a
 * receiver that is plugged in and has never framed - which is a wiring fault at
 * the other end of the cable, a different afternoon's work, and the thing the
 * console already distinguishes by having no `rc_report` at all. */
enum {
    AK_PROTO_RC_OK = 0,   /* this board has a receiver */
    AK_PROTO_RC_NONE = 1, /* this board has no receiver input */
};

/* The most channels a reply can carry, and a bound on the firmware's own count
 * rather than a copy of it: AK_RC_CHANNELS is the flight core's number and this
 * is the wire's, so a firmware that grew its receiver could keep the same
 * client working by sending the first AK_PROTO_RC_MAX of them. */
#define AK_PROTO_RC_MAX 16u

/* Facts about the receiver, as bits. Facts, not warnings: which of them is
 * worth colouring red is the app's business, and a firmware that decided would
 * be pre-rendering a screen it cannot see.
 *
 * The four counters that follow the switches are per protocol, and the ones
 * that do not apply read zero - which is true rather than a placeholder, since
 * an SBUS frame has no CRC to fail and a CRSF frame cannot be flagged
 * failsafe by a receiver that has no such flag. */
#define AK_PROTO_RC_LINK        (1u << 0) /* a channel frame has arrived */
#define AK_PROTO_RC_FAILSAFE    (1u << 1) /* the receiver's own failsafe is set */
/* The four sticks below are this frame's, decoded by the firmware. Clear when
 * there is no frame, and clear when there is one it could not use - an
 * all-zero frame, or counts that cannot be a stick. The sticks are zeroed in
 * that case, so this bit is the only thing that tells "not decoded" from
 * "centred", and the two must never be shown the same way. */
#define AK_PROTO_RC_DECODED     (1u << 2)
/* This board's receiver pin has no inverter in front of it. Only meaningful on
 * SBUS, where it is the difference between a working link and an afternoon
 * spent on the baud rate - the same warning the console prints. */
#define AK_PROTO_RC_NO_INVERTER (1u << 3)
/* There is a return path to the handset. CRSF has one; SBUS does not, and a
 * client that offered telemetry settings on a board without one would be
 * offering a control that cannot act. */
#define AK_PROTO_RC_TELEMETRY   (1u << 4)

#define AK_PROTO_RC_ARM_ON  (1u << 0)
#define AK_PROTO_RC_ANGLE   (1u << 1)

/* The receiver, as a caller fills it in. Spelled out here for the same reason
 * ak_proto_status_t is: the protocol has no opinion about where the numbers
 * come from, and a board with a receiver on a different bus fills the same
 * struct. */
typedef struct {
    uint8_t  flags;
    uint8_t  protocol; /* 0 CRSF, 1 SBUS - the same numbers as ak_rc_protocol_t */
    uint8_t  count;    /* how many of `raw` are meaningful */
    uint8_t  switches; /* AK_PROTO_RC_ARM_ON | AK_PROTO_RC_ANGLE */
    uint16_t raw[AK_PROTO_RC_MAX];
    /* Per-mille, in the order the console prints them: roll, pitch, yaw,
     * throttle. Roll/pitch/yaw are -1000..1000 and throttle is 0..1000. */
    int16_t  sticks[4];
    /* Counters, named as the receiver names them so there is no translation to
     * get wrong. See the flag comment for which are zero on which protocol. */
    uint32_t bytes;
    uint32_t frames;
    uint32_t crc_errors;
    uint32_t rejected;
    uint32_t lost;
    uint32_t failsafe_frames;
    uint32_t dropped; /* the UART receive buffer's drops, which only the board knows */
} ak_proto_rc_t;

/*
 * The sensors, as SENSOR_INFO asks for them.
 *
 * Five topics, numbered rather than named, so a client added to before the
 * firmware is older than the firmware is asked for a topic it does not have
 * and gets a clean refusal rather than a misread frame. The numbers are the
 * order the console's own bring-up checklist prints them in, which is the order
 * of how much is lost without them.
 */
#define AK_PROTO_SENSOR_IMU     0u
#define AK_PROTO_SENSOR_BARO    1u
#define AK_PROTO_SENSOR_RANGE   2u
#define AK_PROTO_SENSOR_BATTERY 3u
#define AK_PROTO_SENSOR_GPS     4u
#define AK_PROTO_SENSOR_TOPICS  5u

/* How long a driver's name may be, including the terminating zero. Twelve
 * because the longest in this tree is "lsm6dso" - the field is fixed so the
 * body's length does not depend on which part answered, and a fixed field means
 * a bound that has to be stated. A name that does not fit is cut, and the
 * template that defines the drivers has a check that fails the build if one
 * ever would be. */
#define AK_PROTO_SENSOR_NAME 12u

/*
 * The status byte on SENSOR_INFO.
 *
 * Two values, and the split between them is not "found" and "not found". NO_SUCH
 * is a statement about the *build*: this firmware does not answer for that
 * topic, either because the topic does not exist or because the board wired no
 * sensor reporting at all. A topic that exists and has nothing fitted is
 * AK_PROTO_SENSOR_OK with `present` clear - the board knows the question and
 * its answer is that there is no part there, which is a different sentence from
 * "I do not know what you are asking", and a client that showed them the same
 * way would send somebody looking for a driver instead of a socket. */
enum {
    AK_PROTO_SENSOR_OK = 0,      /* this build knows the topic */
    AK_PROTO_SENSOR_NO_SUCH = 1, /* no such topic on this board */
};

/* Least significant of the four reasons a board can have no IMU, as bits, so a
 * client can show the console's sentence without a table of its own. The values
 * do not match ak_imu_result_t's and are not meant to: that enum is the
 * driver layer's and this is the wire's, and pinning one to the other would
 * make renumbering either a protocol change. */
#define AK_PROTO_IMU_ABSENT_NOBODY        1u /* nothing answered on the bus */
#define AK_PROTO_IMU_ABSENT_UNKNOWN_PART  2u /* something answered, and it is not known */
#define AK_PROTO_IMU_ABSENT_NO_CONFIG     3u /* the right part answered and would not configure */

/* Each body is a fixed layout, written field by field in the dispatch, so the
 * structs below are how a caller hands the numbers over rather than a
 * description of the bytes. Sizes are counted in append_* calls, not in
 * sizeof - padding is the compiler's business and the wire has none. */

typedef struct {
    char     driver[AK_PROTO_SENSOR_NAME]; /* empty when absent_reason is set */
    uint8_t  absent_reason;    /* 0, or one of AK_PROTO_IMU_ABSENT_* */
    uint8_t  whoami;           /* what the part answered to its identity register */
    int16_t  accel[3];         /* per-mille of g */
    int16_t  gyro[3];          /* milliradians per second */
    int16_t  align[3];         /* degrees, as the aircraft's axes were set */
    int16_t  gyro_bias[3];     /* milli-degrees per second, as `calibrate gyro` left it */
    uint32_t samples;
    uint32_t errors;
} ak_proto_imu_t;

typedef struct {
    char     driver[AK_PROTO_SENSOR_NAME];
    int32_t  pressure_pa;
    int16_t  temperature_cdeg; /* hundredths of a degree */
    uint8_t  have_reference;   /* the ground pressure was captured */
    int32_t  reference_pa;
    int32_t  height_cm;        /* above the reference, from the barometer alone */
    uint8_t  have_gps_reference;
    int32_t  fused_cm;         /* what the aircraft actually flies on */
    uint32_t samples;
    uint32_t errors;
    uint32_t fails;            /* reads in a row with no answer */
    uint32_t baro_samples;     /* into the altitude filter */
    uint32_t gps_samples;
} ak_proto_baro_t;

typedef struct {
    char     driver[AK_PROTO_SENSOR_NAME];
    uint8_t  address;          /* the I2C address the part answered at */
    uint16_t max_mm;
    int32_t  distance_mm;      /* negative means nothing in range */
    uint32_t age_ms;           /* since that measurement, or since the last try */
    uint32_t samples;
    uint32_t out_of_range;
    uint32_t rejected;         /* impossible readings thrown away */
    uint32_t faults;           /* reads the part did not answer */
    uint32_t fails;            /* in a row */
    uint32_t land_mm;          /* what counts as reached */
    uint16_t agree_cm;         /* how far baro and range may differ */
} ak_proto_range_t;

typedef struct {
    uint8_t  ready;            /* this board has a divider at all */
    uint8_t  have_reading;     /* as opposed to a divider with nothing on it yet */
    uint8_t  state;            /* ak_battery_state_t, as the flight code numbers it */
    uint8_t  cells;
    uint16_t volts_cv;         /* the pack, in centivolts */
    uint16_t per_cell_cv;
    int16_t  pin_mv;           /* millivolts at the ADC pin; negative if none */
    uint16_t ratio_milli;      /* vbat_ratio, thousandths */
    uint8_t  rth;              /* the aircraft comes home on a low cell */
    uint16_t warn_cell_mv;
    uint16_t critical_cell_mv;
    uint32_t samples;
    uint32_t rejected;
    uint32_t returns;          /* how many times it has been called home */
} ak_proto_battery_t;

typedef struct {
    uint8_t  have_fix;         /* the receiver has produced a position */
    uint8_t  fix_type;         /* u-blox's own numbering, untranslated */
    uint8_t  fix_ok;           /* the receiver's usable bit */
    uint8_t  satellites;
    uint8_t  valid_now;        /* and it is recent, by the same age rule the console uses */
    int32_t  lat_e7;
    int32_t  lon_e7;
    int32_t  alt_msl_mm;
    int32_t  speed_mm_s;       /* ground speed */
    int32_t  course_e5;        /* degrees * 1e5 */
    uint8_t  have_home;
    int32_t  home_lat_e7;
    int32_t  home_lon_e7;
    int32_t  home_distance_m;  /* negative when there is no home or no fix */
    int32_t  home_bearing_cdeg;
    uint8_t  returning;        /* the navigator is flying it home */
    uint8_t  rth_enabled;
    uint32_t fixes;
    uint32_t dropped;          /* bytes lost in the UART receive buffer */
    uint32_t config_sends;     /* attempts to configure the receiver */
} ak_proto_gps_t;

/* One topic's answer, as a caller fills it in.
 *
 * The five bodies are a union because exactly one of them is meaningful per
 * request, and the topic byte says which - so this is not a struct a client
 * reads whole, it is a way for the callback to hand over the right one without
 * five callbacks that would each need their own null check in io_t. */
typedef struct {
    uint8_t present; /* 1 when a part is fitted and this body was filled */
    uint8_t topic;
    union {
        ak_proto_imu_t     imu;
        ak_proto_baro_t    baro;
        ak_proto_range_t   range;
        ak_proto_battery_t battery;
        ak_proto_gps_t     gps;
    } as;
} ak_proto_sensor_t;

/*
 * One output, as OUTPUT_INFO reports it.
 *
 * Two kinds, numbered rather than named, for the reason the sensor topics are:
 * a client written before a third kind exists reads a kind it does not know and
 * can say so, rather than misreading a frame. The kinds are the two the frame
 * encoder already has - `ak_output_frame_t` is a list of DShot words and a list
 * of servo microseconds, and there is nothing else on any board in this tree.
 *
 * **The three servo fields are meaningless on a motor and are written as zeros
 * there.** That is not the "absent versus reading zero" distinction the sensor
 * reply is built around: a motor has no linkage, so there is no fact being
 * suppressed, and `kind` is on every entry precisely so that a client knows
 * which three fields to read. A servo entry's zeros would mean something; a
 * motor entry's do not.
 *
 * `trim_us` is signed and is the only signed field here: the table's numbers
 * are u32 and float, and a servo whose neutral is below the frame's centre
 * needs a negative trim. It is the reversal and the travel that decide whether
 * a wing can turn, and the console prints all three for that reason.
 */
#define AK_PROTO_OUTPUT_MOTOR 0u
#define AK_PROTO_OUTPUT_SERVO 1u

/* Seven bytes an entry, counted in append_* calls rather than in sizeof -
 * padding is the compiler's business and the wire has none. */
typedef struct {
    uint8_t  kind;
    uint8_t  index;     /* this output's number within its own kind, 0-based */
    uint8_t  reversed;  /* servos: the linkage moves the other way from the stick */
    int16_t  trim_us;   /* servos: added to the centre, signed */
    uint16_t travel_us; /* servos: how far the linkage moves at full stick */
} ak_proto_output_t;

/* The status byte on OUTPUT_INFO.
 *
 * NONE is a fact about the board - the same sentence `output test` prints when
 * it refuses on a device with nothing to drive - and it is not the same as an
 * empty list. A board with a list that came back empty is a board that is not
 * ready or a board with a bug, and reporting it as OK with a count of zero
 * would have a client draw an aircraft with no motors; NONE at least says the
 * honest thing, which is that there is no list to draw. */
enum {
    AK_PROTO_OUTPUT_INFO_OK = 0,
    AK_PROTO_OUTPUT_INFO_NONE = 1,    /* no list: nothing to drive, or not ready */
    AK_PROTO_OUTPUT_INFO_TOO_MANY = 2,/* more outputs than one frame carries */
};

/* The most outputs this reply will ever carry, and the array the dispatch
 * builds one in.
 *
 * It is a bound on the *protocol*, not on any board: a board with more outputs
 * than this gets AK_PROTO_OUTPUT_INFO_TOO_MANY rather than a page of them,
 * because a short list drawn as the whole aircraft is a client missing a servo
 * it will then go looking for in the wiring. A board that passes it is a board
 * this opcode grows a `first` argument for - a change an old client survives,
 * because it sends the empty frame it has always sent.
 *
 * The number is the arithmetic, not a round one that seemed roomy. The reply is
 * five bytes of header and seven bytes an entry, so the largest list that fits
 * one frame is (96 - 5) / 7, which is 13. This was 16 for one revision, chosen
 * because the largest board here has six outputs and sixteen is comfortably
 * past six - and 5 + 16 * 7 is 117, so the largest legal list this opcode could
 * be asked for overflowed AK_PROTO_MAX_PAYLOAD by 21 bytes. `append_u8` stops
 * writing at the capacity it is given but keeps counting, so the frame still
 * went out, with a length byte describing bytes that were never written. The
 * check in tests/test_proto.c that reads the maximum list's length back is what
 * caught it; the comment here claimed the fit without multiplying. */
#define AK_PROTO_OUTPUT_MAX ((AK_PROTO_MAX_PAYLOAD - 5u) / 7u)

/* The `op` byte on OUTPUT_TEST, and the status it answers with.
 *
 * STOP is a value rather than a separate command because the two share every
 * other argument and every gate, and a client that has lost track of the state
 * must be able to send one without first asking. */
#define AK_PROTO_OUTPUT_TEST_HOLD 0u /* drive it, or keep driving it */
#define AK_PROTO_OUTPUT_TEST_STOP 1u

enum {
    AK_PROTO_OUTPUT_TEST_OK = 0,       /* the output is being driven */
    AK_PROTO_OUTPUT_TEST_STOPPED = 1,  /* the stop was carried out */
    AK_PROTO_OUTPUT_TEST_NO_OUTPUT = 2,/* no output of that kind and number */
    AK_PROTO_OUTPUT_TEST_ARMED = 3,    /* refused: the aircraft is not disarmed */
    AK_PROTO_OUTPUT_TEST_NO_BOARD = 4, /* refused: this board drives no outputs */
    AK_PROTO_OUTPUT_TEST_NO_OP = 5,    /* refused: no such `op` */
};

/* What OUTPUT_TEST replies with, and how the board hands it over.
 *
 * `level_pct` is the clamped number rather than the requested one, so a client
 * that asked for 100 and got 15 renders a button labelled with what the board
 * is doing. `remaining_ms` is the board's countdown and not a duration the
 * client works out from its own clock: the whole value of a hold that expires
 * is that the expiry is the board's, and a screen counting from the moment it
 * pressed would be a second clock disagreeing with the one that stops the
 * motor. */
typedef struct {
    uint8_t  level_pct;
    uint16_t remaining_ms;
} ak_proto_output_test_t;

/*
 * How long a hold survives without being renewed, and how hard it is allowed to
 * drive.
 *
 * Both are firmware constants and not arguments, which is the whole of their
 * value: a client cannot ask for a longer hold or a bigger number, so a client
 * that is wrong about either cannot be wrong in a way that moves a propeller.
 * The hold is renewed by the client sending the same request again - a screen
 * that is holding a button sends it at a fraction of this - so the timeout only
 * ever fires for a client that has stopped talking, which is the case it is
 * there for.
 *
 * The cap is a percentage of full range. Fifteen is the console's own number
 * for the motor sweep (`AK_OUTPUT_TEST_MOTOR`, "enough to turn a motor, not to
 * hurt") and it is the right one to copy: a value that spins a propeller on the
 * bench is a value that needs the props off, and the number that identifies
 * which pad is which is much smaller than the number that tests whether it
 * pulls.
 */
#define AK_PROTO_OUTPUT_TEST_MAX_MS 500u
#define AK_PROTO_OUTPUT_TEST_MAX_PCT 15u

/* The status byte on PARAM_SET, PARAM_SAVE and PARAM_DEFAULT.
 *
 * They share one vocabulary because they are the same kind of answer - "did
 * this write happen, and if not, what stopped it" - and a client that has to
 * learn three of them will get one wrong. `AK_PROTO_WRITE_REFUSED_ARMED` is
 * the one added by the write gate: it means the board is not in a position to
 * be reconfigured, which is a different fact from "the value was out of range"
 * and from "the board has nowhere to save", and a screen that showed either of
 * those for it would be telling a person to try the wrong thing. */
enum {
    AK_PROTO_WRITE_OK = 0,
    AK_PROTO_WRITE_NO_SUCH = 1,      /* no such parameter */
    AK_PROTO_WRITE_REJECTED = 2,     /* the table refused the value */
    AK_PROTO_WRITE_NO_STORAGE = 3,   /* nowhere to save, or nothing to save to */
    AK_PROTO_WRITE_REFUSED_ARMED = 5,
    /* 4 is "the board refused the write" on PARAM_SAVE and is left where it
     * is - it is the storage's own error, not a policy. Renumbering it would
     * change what an existing client reads off an existing opcode, which is
     * the one thing AK_PROTO_VERSION exists to prevent. */
    AK_PROTO_WRITE_STORAGE_ERROR = 4,
};

/* The three sources, and what a device that has none of them says. */
#define AK_PROTO_LOG_FAST  0u
#define AK_PROTO_LOG_LONG  1u
#define AK_PROTO_LOG_FLASH 2u
#define AK_PROTO_LOG_MAX   2u

/* The most a telemetry stream can be asked for. A flight controller's own loop
 * runs at a kilohertz and none of this is worth that: the aircraft does not
 * change meaningfully in 20 ms, and a stream faster than the link is a stream
 * that fills a socket buffer and then delays the config that shares it. */
#define AK_PROTO_TELEMETRY_MAX_HZ 50u

/* The most a log stream can be asked for, and the same reasoning as the
 * telemetry ceiling above with one addition: a stream that filled a socket
 * buffer would delay the configuration commands sharing that socket, and a log
 * read is the one long-running thing this protocol can be asked to do. Fifty a
 * second is the fastest a record fits beside everything else on a network link;
 * the whole flash log at that rate is a little over two minutes, which is a read
 * worth starting rather than a read worth abandoning.
 *
 * That ceiling was set when a record was 51 bytes in a 60-byte slot and the
 * F405's log was five sectors, 10 920 records. The region gave a sector to the
 * image before roadmap 2.4 and 2.4 grew the record to 66 bytes in an 80-byte
 * slot, so the record is a third larger and the log 6 552 where it was 8 736
 * going into this milestone; roadmap 4.1 grew it again, to 87 bytes in a
 * 96-byte slot and 5 460 records. A bigger record at the same rate is a heavier
 * stream than the one the ceiling was argued against, so 50 is no longer a
 * measured ceiling but a carried-over one - re-measuring it needs a bench board
 * on a network link, which this bench did not have. Nothing here claims it
 * was. */
#define AK_PROTO_LOG_STREAM_MAX_HZ 50u

/* What a streamed frame is carrying.
 *
 * The hole is the reason there are three values rather than one. A record the
 * board will not produce - a flash slot the power cut in half, which
 * `ak_flashlog_record_at` refuses rather than decoding - is a fact about the
 * log, and a stream that skipped it would leave the client drawing a line
 * between the records either side of a gap it never learned about. It arrives
 * as a frame at that index with no body, so the client counts what it was told
 * about instead of inferring a sequence.
 *
 * `DONE` is the other half of the same argument in the other direction: a
 * stream that stopped silently is a link failure and a finished read wearing
 * the same appearance, and only one of them means the data is complete. */
enum {
    AK_PROTO_LOG_STREAM_RECORD = 0, /* a record follows the header */
    AK_PROTO_LOG_STREAM_HOLE = 1,   /* this index was refused; no body */
    AK_PROTO_LOG_STREAM_DONE = 2,   /* the range is finished; no body */
};

/*
 * What a board can answer, as a word HELLO carries.
 *
 * A client's problem is not "which protocol version is this" - that is one
 * byte and it is in every frame - it is "does this build answer the command I
 * am about to send". Until this word existed the only way to find out was to
 * send it and wait out a timeout, once per command, on a link that carries a
 * person's configuration.
 *
 * A bit means the command exists in this build *and* does what
 * docs/16-protocol.md says it does. It is set when that is true and not
 * before: a board that set AK_PROTO_FEATURE_OUTPUT_TEST because this file
 * defines the constant, while its dispatch had no case for it, would have a
 * client send a frame and wait for an answer that never comes - which is the
 * exact failure the word exists to prevent. Defining a constant here is how
 * the vocabulary is written down; setting it in `ak_proto_io_t::features` is
 * how a board says it speaks it, and the two are deliberately different acts.
 *
 * The word is u32 and there are 12 bits spoken for, which leaves room to grow
 * without ever having to move AK_PROTO_VERSION: an old client reads the bits
 * it knows and ignores the rest, and an old board sends a HELLO that ends
 * before the field - so a client that reads no capability word must report
 * `absent`, never `no capabilities`. Those are different claims and only one
 * of them is true of a board that predates this.
 */
#define AK_PROTO_FEATURE_PARAM_INFO (1u << 0)
#define AK_PROTO_FEATURE_PARAM_DEFAULT (1u << 1)
/* A successful set makes the board re-apply its configuration, not merely
 * change the table. True of any build whose `on_change` is wired up. */
#define AK_PROTO_FEATURE_APPLIES_ON_WRITE (1u << 2)
/* Every write path - set, save, default, output test - refuses while the
 * aircraft is armed. Not the same claim as "save is gated": a board can gate
 * one route and not the others, and one that did would be worse than either,
 * because the guard would depend on which button was pressed. */
#define AK_PROTO_FEATURE_GATES_ON_ARMED (1u << 3)
#define AK_PROTO_FEATURE_RC_CHANNELS (1u << 4)
#define AK_PROTO_FEATURE_SENSOR_INFO (1u << 5)
#define AK_PROTO_FEATURE_OUTPUT_INFO (1u << 6)
#define AK_PROTO_FEATURE_OUTPUT_TEST (1u << 7)
#define AK_PROTO_FEATURE_LOG_STREAM (1u << 8)
#define AK_PROTO_FEATURE_PREFLIGHT (1u << 9)
#define AK_PROTO_FEATURE_CALIBRATE (1u << 10)
#define AK_PROTO_FEATURE_MISSION (1u << 11)
/* The build has a profiler and answers PERF with a window rather than with
 * "none". A board that set this while its `perf` callback was null would be
 * advertising a reading it cannot take, which is the same failure the whole
 * word exists to prevent - see the note above the first bit. */
#define AK_PROTO_FEATURE_PERF (1u << 12)
/* The build can hear its ESCs and answers MOTOR_TELEMETRY with a speed per
 * motor, telling a motor it cannot hear from one it heard turning at zero.
 *
 * **Not set by any board in this tree yet, and that is the honest half of this
 * milestone.** The decode is written and host-tested (`ak_dshot_gcr.c`,
 * `ak_dshot_edt.c`) and the codec above is written and host-tested, but the
 * *capture* half - the port's input capture, its DMA buffer, where a frame
 * begins - is not written for any board, and `ak_dshot_edt_t` is still owned by
 * nothing. A board that set this bit would be advertising a reading it cannot
 * take, which is the one thing the word exists to prevent, so no board sets it
 * and every board answers MOTOR_TELEMETRY with AK_PROTO_MOTOR_NONE. The
 * vocabulary is written down; the bit waits for the port, as bit 10 waited for
 * the calibration opcode. */
#define AK_PROTO_FEATURE_MOTOR_TELEMETRY (1u << 13)

/* What a caller has to be able to say about itself. Kept as a struct filled by
 * the caller rather than a pile of getters, so the protocol has no opinion
 * about where the numbers come from. */
typedef struct {
    uint8_t  flight_state;
    uint8_t  link_live;
    uint8_t  gps_fix_type;
    uint8_t  gps_satellites;
    int16_t  roll_ddeg;
    int16_t  pitch_ddeg;
    int16_t  yaw_ddeg;
    int32_t  lat_e7;
    int32_t  lon_e7;
    uint8_t  motor[4];
} ak_proto_status_t;

typedef struct {
    ak_params_t *params;
    /* The AK_PROTO_FEATURE_* bits this build answers for, sent in HELLO.
     *
     * A field rather than a callback, and that is the whole design: the bit
     * word is a property of the compiled firmware, not a reading of the
     * hardware, so there is nothing to ask at run time and nothing that could
     * answer differently between two calls. A callback here would invite
     * exactly the mistake this word exists to remove - a board that reports a
     * capability it is not currently in a position to exercise.
     *
     * Zero is a real value - a board with no optional command on it - and it
     * is not the same as the field being absent, which is what an old board's
     * HELLO says by ending early. */
    uint32_t features;
    /* Fills `out` with the current flight state. Required. */
    void (*status)(void *ctx, ak_proto_status_t *out);
    /* A parameter has moved: let the board act on it.
     *
     * The console has had this since it had a `set` - ak_cli.h's on_change -
     * and the protocol did not, so `set` over the wire changed the table and
     * stopped there. The table is not the aircraft: `parameters_changed()` in
     * main.c is what turns a parameter into behaviour (the airframe, the whole
     * table, the DShot timer's rate, and the receiver's protocol on both the
     * parser and the board's line), and a client that set `dshot_khz` over the
     * wire got a `params` reply reporting the new value while the timer ran the
     * old one. A reported value that the hardware does not have is the worst
     * pair available on an aircraft.
     *
     * The same signature as the console's, and for the same reason: it is the
     * same act, and the two links must not be able to disagree about when it
     * happens.
     *
     * Optional, like `save` - a device with nothing to re-apply leaves it
     * null, and the protocol changes the table exactly as it used to. */
    void (*on_change)(void);
    /* Persists the parameters. Optional - a board with no storage returns
     * negative and the protocol says so rather than pretending. */
    int (*save)(void *ctx);
    /* Whether this board will accept a configuration write *now*: nonzero for
     * yes. Optional, and null means "asked no such question" - a device with no
     * aircraft to be armed is not a device that is permanently disarmed, and
     * the two must not be confused.
     *
     * Every write route consults this - set, save, default, and the output test
     * when it lands - at the moment of the request rather than once at connect,
     * because a link that stays up across an arming is the normal case and a
     * gate checked at connect is a gate that is wrong a second later.
     *
     * It returns a *policy* answer, not a reason. The board decides what makes
     * a configuration write unsafe - main.c passes ak_flight_config_writable(),
     * the same predicate the console's `save` and the airborne routes use - and
     * the protocol makes no attempt to infer it from `status`, which reports
     * the flight state for display and is not a gate.
     *
     * This is what AK_PROTO_FEATURE_GATES_ON_ARMED names. A board that sets
     * that bit while leaving this null would be advertising a guard it does not
     * have, which is why main.c sets the two together or not at all. */
    int (*writable)(void *ctx);
    /* The receiver, as RC_CHANNELS asks for it. Optional in the same way
     * `log_count` is, and null means "this board has no receiver input" - a
     * fact about the board that the reply says outright rather than expressing
     * as a frame of zeros, because a frame of zeros is also what a receiver
     * that is plugged in and silent looks like.
     *
     * A callback and not a field, unlike `features`, for the reason the two are
     * different: the capability word is a property of the compiled firmware and
     * cannot change while it runs, and this changes every time a handset moves.
     *
     * Filled per request, with the counters included, so that a client polling
     * this opcode needs no second command to find out whether the receiver is
     * healthy - the whole point of polling something at the rate a person can
     * move a stick. */
    void (*rc_state)(void *ctx, ak_proto_rc_t *out);
    /* One sensor, as SENSOR_INFO asks for it.
     *
     * Optional in the same way `rc_state` is, and null means "this board
     * reports no sensors" - which every topic answers AK_PROTO_SENSOR_NO_SUCH
     * for. That is deliberately the same answer as an out-of-range topic, and
     * deliberately not the same as a topic that exists with nothing fitted:
     * the first is a fact about the build and the second is a fact about the
     * aircraft, and only the second is a reason to go and look at a socket.
     *
     * Called only for topics this header defines, so the callback's own default
     * case is unreachable from the wire and a topic added here without an arm
     * there is a build-time gap rather than a client's misread frame. It is
     * still asked to leave `present` clear rather than to trust that, because
     * the cost of being wrong is a client believing a zeroed struct.
     *
     * A callback and not a field, for `rc_state`'s reason: half of these
     * change while the board runs. Filled per request rather than cached,
     * because a cache here would be a second copy of the driver's state and
     * the whole value of this opcode is that it is the driver's own numbers. */
    void (*sensor_state)(void *ctx, uint8_t topic, ak_proto_sensor_t *out);
    /* The logs, if there are any: how many records `source` holds, and the
     * i-th oldest of that source as the firmware's own record bytes. Optional
     * in the same way - and a device with one log answers for one source and
     * says "no" for the others, which is what the reply's status is for.
     *
     * `log_count` returns -1 for a source this device does not have, which is
     * how a client tells "no such log" from "an empty one": the second answer
     * is zero, and both are worth being able to say. */
    int32_t (*log_count)(void *ctx, uint8_t source);
    unsigned (*log_record)(void *ctx, uint8_t source, uint16_t index,
                           uint8_t *out, unsigned capacity);
    /* This board's outputs, as OUTPUT_INFO asks for them.
     *
     * **The return is the board's total, not the number written.** It writes at
     * most `capacity` descriptors - the dispatch passes AK_PROTO_OUTPUT_MAX -
     * and returns how many the aircraft has, which may be more. That split is
     * what lets the dispatch refuse with TOO_MANY instead of quietly showing a
     * page as the whole aircraft, and it is the one place in this reply where
     * "how many are there" and "how many are here" are different questions.
     *
     * Optional in the same way `rc_state` is, and null means "this board drives
     * no outputs" - which the reply says outright rather than expressing as an
     * empty list, because an empty list is also what a board with a wiring bug
     * would send. A board that has outputs and returns zero is saying something
     * different from a board that has none, and the two status values are there
     * so a client can tell. */
    unsigned (*outputs)(void *ctx, ak_proto_output_t *out, unsigned capacity);
    /* Drive one output, or stop driving. Returns one of
     * AK_PROTO_OUTPUT_TEST_*, and fills `out` when it drives.
     *
     * Null means the same thing `outputs` being null means - this board drives
     * no outputs - and the reply is the same refusal, so a board cannot be in
     * the state of listing outputs it will not drive or driving outputs it will
     * not list. main.c sets the two together or not at all.
     *
     * **The gate is the callback's, not the dispatch's.** This is the one
     * opcode in the protocol whose safety depends on a fact about the aircraft
     * that changes while it runs, and the answer has to be read at the moment
     * of the command - so the callback asks `ak_flight_state` itself rather
     * than the dispatch asking `io->writable` first and passing the verdict
     * down. `io->writable` is the same predicate inside, which is why the two
     * routes cannot disagree about what "safe to write" means.
     *
     * **The two arguments are the two halves of a split, not two spells of the
     * same check.** `op` is a *wire* value - this header defines it and the
     * dispatch refuses anything else before calling - so the callback sees only
     * HOLD or STOP and branches on them rather than validating them. `kind` and
     * `index` are *facts about the aircraft*, which the protocol has no way to
     * know, so the callback is the only thing that can refuse them. A second
     * check on either side would be a second policy. */
    int (*output_test)(void *ctx, uint8_t op, uint8_t kind, uint8_t index,
                       uint8_t level_pct, ak_proto_output_test_t *out);
    /* This board's preflight checklist, as PREFLIGHT pages it.
     *
     * Fills `*lines` with a pointer to the board's own record and `*count`
     * with how many lines it holds, and returns nonzero. Returns zero - and
     * leaves both alone - when this board has no checklist, which the reply
     * says outright rather than expressing as an empty one.
     *
     * **A pointer to the board's record, not a copy into a caller's buffer**,
     * and that is the one place in this interface where the protocol reads the
     * firmware's own memory. It is right here because the record *is* the
     * console's: main.c builds it once and prints it, and a copy per request
     * would be a second record that could be built at a different moment from
     * the one the console describes. The pointer is valid until the next call
     * with `first == 0` and must not be held across one.
     *
     * `first` is the index the client asked for. Zero means "start a walk",
     * and a walk is the unit: the board rebuilds the record for it, so every
     * page of one walk describes one moment. A nonzero `first` reads the
     * record the last walk built, which is why a client that jumps straight to
     * index 5 gets whatever the previous walk left rather than a fresh one -
     * an answer it can detect, since a walk of pages that never began is not a
     * thing a client does by accident. */
    int (*preflight)(void *ctx, unsigned first, const ak_preflight_line_t **lines,
                     unsigned *count);
    /* The mission, as MISSION asks about it and as its verbs act on it.
     *
     * Returns one of AK_PROTO_MISSION_*, and **fills `out` on every path it is
     * called, including the ones that refuse** - the whole state is what a
     * reply carries, and a client that asked to start an empty mission should
     * come back with `NO_WAYPOINTS` *and* `count == 0`, so it can say why in
     * the board's own terms rather than a sentence of its own.
     *
     * `op` is a *wire* value, so the dispatch refuses anything outside the five
     * before calling; the callback branches on it rather than validating it. A
     * board that has no navigator leaves this null and the dispatch answers
     * `NO_NAV`, which is the same split OUTPUT_INFO and its `outputs` callback
     * use - null means "this board does not do this", and it is a different
     * answer from "the list is empty".
     *
     * **Nothing here is gated on the aircraft being disarmed, deliberately.**
     * `stop` exists to be available at any moment including in the air - it is
     * the verb whose whole job is to be reachable - and `start` is a request
     * the flight loop honours only when it is already flying, so an armed board
     * that received it has changed nothing yet. The gate the other opcodes have
     * is a gate on *writing configuration*, and this does not write any. */
    int (*mission)(void *ctx, uint8_t op, ak_proto_mission_t *out);
    /* The calibrations, as CALIBRATE starts, reads and ends them.
     *
     * Returns one of AK_PROTO_CALIBRATE_*, and **fills `out` on every path it
     * is called, including the ones that refuse** - the session as the board
     * now holds it is what a reply carries, so a client that asked to start a
     * gyro calibration on an armed aircraft should come back with `ARMED` *and*
     * a session saying nothing is running, rather than a header with a caller's
     * stack behind it. The dispatch zeroes it first for that reason.
     *
     * `verb` is a *wire* value, so the dispatch refuses anything outside the six
     * before calling and the callback branches on it rather than validating it -
     * the same split `mission` and `output_test` use. `face` and `mv` are the
     * two arguments the verbs take: the accelerometer face for ACCEL, the
     * measured pack voltage in millivolts for VBAT, and neither is read by a
     * verb that does not take it. The dispatch does not range-check either,
     * because both ranges are the aircraft's - the accelerometer's face count
     * and the pack's plausible voltage - and a second check in the dispatch
     * would be a second policy that could disagree with the first.
     *
     * **The gate is asked here as well as in the dispatch, and both are
     * wanted.** The dispatch checks `io->writable` so that a refusal is a
     * policy with a status a client can render, exactly as PARAM_SAVE does; the
     * callback checks again so the guard does not depend on the request having
     * come through this file. Neither is redundant with the *session's* own
     * check, which is the third one and the only one that matters in the air:
     * a calibration that is running when the aircraft arms must stop, and that
     * is a fact about the aircraft changing after the command, which no gate at
     * dispatch time can see. main.c's gyro calibration has checked exactly that
     * from the loop since before this opcode existed.
     *
     * Null means "this board has nothing to calibrate", which the reply says
     * outright as `NOTHING` rather than as a session that never starts - the
     * same split OUTPUT_INFO's null callback makes, and a different answer from
     * a board whose accelerometer would not hold still. */
    int (*calibrate)(void *ctx, uint8_t verb, uint8_t face, uint32_t mv,
                     ak_proto_calibration_t *out);
    /* The profiler's window, as PERF asks for it.
     *
     * Returns nonzero when there is a window to report, and fills `out` on that
     * path. Returns zero with `out` untouched when this build has nothing to
     * measure, which the reply carries as AK_PROTO_PERF_NONE - and null means
     * the same thing at build time, so a board without a profiler need not
     * define a callback that always says no.
     *
     * **A callback and not a field**, unlike `features`: the window changes
     * every loop, and a field here would be a copy of the profiler's state that
     * a board would have to remember to refresh. It is the profiler's own
     * numbers or nothing - the same argument `sensor_state` carries.
     *
     * This is what AK_PROTO_FEATURE_PERF names, and the two are set together or
     * not at all, for the reason the feature word's own comment gives. */
    int (*perf)(void *ctx, ak_proto_perf_t *out);
    /* What each motor is doing, as MOTOR_TELEMETRY asks for it.
     *
     * Returns nonzero when this board has a way to hear its ESCs, and fills
     * `out` on that path - including the case where it has heard nothing yet,
     * which is a filled struct with every MEASURED flag clear and not a refusal.
     * Returns zero with `out` untouched when this build has no such path, which
     * the reply carries as AK_PROTO_MOTOR_NONE; null means the same thing at
     * build time, so a board without the path need not define a callback that
     * always says no.
     *
     * **A callback and not a field**, for `perf`'s reason: the window moves
     * every time a reply arrives from an ESC, and a field here would be a copy
     * of the telemetry's state that a board would have to remember to refresh.
     *
     * This is what AK_PROTO_FEATURE_MOTOR_TELEMETRY names, and the two are set
     * together or not at all. Every board in this tree currently sets neither,
     * so every one of them answers NONE - see the bit's own comment. */
    int (*motor_telemetry)(void *ctx, ak_proto_motor_telemetry_t *out);
    void *ctx;
} ak_proto_io_t;

typedef struct {
    uint8_t  state;
    uint8_t  buffer[AK_PROTO_FRAME_MAX];
    uint16_t held;
    uint16_t expected;
    uint16_t crc_received;
    uint32_t last_byte_ms;

    /* Per link, because the protocol parser is: one of these is the console
     * and one is the network, and they subscribe separately. */
    uint8_t  telemetry_hz;
    /* Whether this link can push frames at a client without being asked. The
     * console cannot and should not: it is a shared, human-facing wire, and a
     * stream on it would arrive between a person's keystrokes. A network link
     * can. The subscribe command answers 0 on a link that cannot - a rate a
     * client will never receive is worse than being told no, and "the answer is
     * the rate that will actually be sent" is what that command has always
     * promised. */
    uint8_t  can_stream;
    /* And per link for the same reason: two clients asking about different
     * logs should not move each other's pointer. */
    uint8_t  log_source;

    /* The log stream, if this link has one running. Four fields rather than a
     * pointer to a cursor object because a link has at most one, and because
     * `ak_proto_init` clearing the struct is then all "a new client subscribes
     * to nothing" needs to be true - the same reason `telemetry_hz` lives
     * here.
     *
     * `log_stream_index` is the *next* record to send, so the range is
     * half-open: [index, end). It is advanced by the frame builder and not by
     * the caller, so a frame that could not be built does not silently skip
     * the record it was for. */
    uint8_t  log_stream_hz;
    uint8_t  log_stream_source;
    uint16_t log_stream_index;
    uint16_t log_stream_end;

    uint32_t frames;
    uint32_t responses;
    uint32_t bad_crc;
    uint32_t bad_length;
    uint32_t unknown_commands;
    uint32_t bytes;
} ak_proto_t;

void ak_proto_init(ak_proto_t *proto);

/* A telemetry frame, built to be pushed rather than sent in reply to anything:
 * the command byte has no response bit, which is how a client tells a stream
 * from its own answers on the same connection. Returns the frame length, or 0
 * if the buffer is too small. */
unsigned ak_proto_telemetry_frame(const ak_proto_io_t *io, uint32_t now_ms,
                                  uint8_t *out, unsigned capacity);

/* One frame of a log stream, and it advances the stream by one index.
 *
 * Returns the frame length, or 0 when it has nothing it can send: no stream is
 * running, or the buffer is too small to hold a record whole. It never returns
 * 0 because the range ended - the end is a frame carrying `DONE`, not a
 * silence, because a stream that stopped without saying so would look exactly
 * like a link that died. A caller that treated 0 as "the read finished" would
 * be reading the one distinction this command exists to make.
 *
 * **Emitting `DONE` clears `log_stream_hz`**, so the caller must take the rate
 * it wants to schedule by *before* calling - a caller that read the rate
 * afterwards would divide by zero on the last frame of every stream.
 *
 * A record the board will not produce is not skipped: that index gets a frame
 * carrying `HOLE` and no body. What this function will not do is invent a
 * record to keep the sequence smooth. */
unsigned ak_proto_log_stream_frame(ak_proto_t *proto, const ak_proto_io_t *io,
                                   uint8_t *out, unsigned capacity);

/* One byte. Returns the length of a response written into `response` when this
 * byte completed a frame that has one, and 0 otherwise. */
unsigned ak_proto_feed(ak_proto_t *proto, const ak_proto_io_t *io, uint8_t byte,
                       uint32_t now_ms, uint8_t *response, unsigned capacity);

uint16_t ak_proto_crc16(const uint8_t *data, unsigned length);

/* True when the parser would read the *next* byte as the start of a frame:
 * between frames, and not part way through one that has gone quiet.
 *
 * `state == STATE_SYNC1` alone does not answer that, and taking it for the
 * answer is a bug with two halves. A frame that stops part way is abandoned
 * only when the next byte arrives and the gap since the last one is longer than
 * AK_PROTO_GAP_MS - so in between, the parser reports non-idle while being, in
 * every way that matters, finished with the frame. A caller that trusts the raw
 * state sends that next byte down the protocol's path, where it is discarded:
 * the first character of a command typed after a half-frame disappears, and
 * "version" arrives as "ersion".
 *
 * The gap is the same test ak_proto_feed makes before it abandons a frame, so
 * the two cannot disagree about whose byte this is. That matters more than it
 * sounds: if the caller decides the byte is the console's, ak_proto_feed is
 * never called for it, so the parser's own idea of the gap does not advance -
 * the two would drift apart on exactly the byte they disagree about. */
int ak_proto_idle_at(const ak_proto_t *proto, uint32_t now_ms);

void ak_proto_report(const ak_proto_t *proto, ak_printf_fn out);

#endif /* AK_CORE_AK_PROTO_H */
