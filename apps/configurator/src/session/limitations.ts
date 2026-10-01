import type { Capabilities } from '../protocol/features';
import { Feature, has } from '../protocol/features';
import type { Limitation } from './types';

/**
 * What this firmware cannot do over the config protocol, said plainly.
 *
 * Every entry here is a claim about the source, so every entry names the file
 * and the symbol that establishes it. A limitation without a citation is a
 * rumour, and a configurator is exactly the place where a rumour about a flight
 * controller turns into somebody's afternoon.
 *
 * These are shown in the workspace rather than hidden behind a tooltip. The
 * goal they serve is specific: a write the firmware cannot fully apply must
 * ship with a concrete explanation, not a disabled button and no reason.
 */

export interface LimitationFacts {
  /** The board answered `param save` with "nowhere to save". */
  readonly noPersistence: boolean;
  /** The board agreed to a non-zero telemetry rate but has sent no frame. */
  readonly streamPromisedNotDelivered: boolean;
  /** The board's capability word, or `null` when its `hello` ended before the
   *  field. Null is not zero: it means the board cannot say, which is a
   *  different sentence from "it says no". */
  readonly capabilities: Capabilities;
}

/*
 * This was `f4-writes-are-echoed-only`, and the name was a claim the source no
 * longer supports. It said the config protocol had no post-change callback —
 * "ak_proto_io_t has no on_change field to fire" — and that a write over the
 * wire therefore reached the table and stopped there.
 *
 * That is false, and was made false deliberately: `ak_proto_io_t.on_change`
 * exists, `ak_proto.c` fires it on a successful set, and `main.c` wires it to
 * `parameters_changed`, which is the same call the console's `set` makes. The
 * field's own comment in `ak_proto.h` says why it was added — "the table is not
 * the aircraft".
 *
 * So the limitation is not that the firmware fails to re-apply. It is that the
 * *reply* does not say whether it did. What comes back is a status byte and the
 * table's own message; nothing on the wire distinguishes a board that rebuilt
 * its configuration from one that only moved a number, and a client cannot tell
 * one firmware revision from another. Naming the mechanism was how the old entry
 * went wrong — the mechanism changed underneath it. What is left is an
 * observation about the bytes, which is what a limitation should be.
 */
const NO_APPLICATION_CONFIRMATION: Limitation = {
  id: 'no-application-confirmation',
  severity: 'qualifies-writes',
  summary: 'the board confirms the value is in its table, not that the aircraft rebuilt for it',
  detail:
    'This board reported a capability word and does not claim ' +
    'AK_PROTO_FEATURE_APPLIES_ON_WRITE, so its own answer is that a set moves ' +
    'the parameter table and nothing else. The reply is a status byte and the ' +
    'table\'s own message (ak_proto.c, AK_PROTO_CMD_PARAM_SET) either way, and ' +
    'neither carries an application fact. This app reports what it can ' +
    'establish (echoed) and marks application "not established" rather than ' +
    'reading it out of a byte that does not hold it.',
  citations: [
    'aerialkit/src/core/ak_proto.h:ak_proto_io_t',
    'aerialkit/src/core/ak_proto.h:AK_PROTO_FEATURE_APPLIES_ON_WRITE',
    'aerialkit/src/core/ak_proto.c:AK_PROTO_CMD_PARAM_SET',
  ],
};

/*
 * The same limitation on a board that cannot answer the question at all.
 *
 * The two cases used to be one entry that asserted the weaker of the two
 * ("nothing on the wire says whether it re-applied") on *every* connection,
 * including connections to boards where the stronger thing was true. Now that
 * HELLO carries the capability word they are separable, and they must stay
 * separate: "it says it does not re-apply" and "it cannot say" are different
 * facts about the aircraft, and a single wording for both would be this app
 * making the same class of mistake it was built to catch.
 */
const NO_APPLICATION_CONFIRMATION_UNKNOWN: Limitation = {
  id: 'no-application-confirmation',
  severity: 'qualifies-writes',
  summary: 'the board confirms the value is in its table, not that the aircraft rebuilt for it',
  detail:
    'This board\'s hello reply ended before the capability word, so it predates ' +
    'AK_PROTO_FEATURE_APPLIES_ON_WRITE (ak_proto.h) and cannot say whether a ' +
    'set re-applies the configuration or only moves a number. Both are possible ' +
    'on the revisions this reply fits, so this app reports the thing it can ' +
    'establish (echoed) and marks application "not established" rather than ' +
    'guessing which revision answered.',
  citations: [
    'aerialkit/src/core/ak_proto.h:ak_proto_io_t',
    'aerialkit/src/core/ak_proto.h:AK_PROTO_FEATURE_APPLIES_ON_WRITE',
    'aerialkit/src/core/ak_proto.c:AK_PROTO_CMD_PARAM_SET',
  ],
};

/*
 * This was `firmware-has-no-armed-guard`, and it was half wrong in a way worth
 * recording, because the half that was right is the half that matters.
 *
 * It said neither PARAM_SET nor PARAM_SAVE checks the flight state. PARAM_SAVE
 * does, though not in the handler: `main.c`'s `save_parameters` passes
 * `ak_flight_config_writable(&flight)` into `ak_params_save`, and that returns
 * true only for AK_FLIGHT_DISARMED — the argument exists precisely so a route
 * cannot save without having asked. So a save while armed is refused.
 *
 * PARAM_SET has no such check, and that is the real asymmetry: a set while
 * armed is accepted, moves the running configuration, and is not persisted. The
 * entry below is only about the half that is true.
 *
 * It is now conditional on the board's own answer. GATES_ON_ARMED is a claim
 * the board makes about its own writes, and a board reporting the bit is
 * saying the asymmetry is closed. Setting that bit is a bigger promise than it
 * looks — it covers set, save *and* default — so a firmware that guards some of
 * them and not others must leave it clear, which is what main.c does today
 * (deliberately: see the comment on `proto_io.features`). This entry therefore
 * does not appear on a board that reports the bit, and does appear on one that
 * reports the word without it.
 */
const NO_ARMED_GUARD_ON_SET: Limitation = {
  id: 'no-armed-guard-on-set',
  severity: 'note',
  summary: 'the board would take a parameter value while armed; this app will not send one',
  detail:
    'This board reported a capability word and does not claim ' +
    'AK_PROTO_FEATURE_GATES_ON_ARMED, so its own answer is that a set is not ' +
    'checked against the flight state. Saving is guarded and setting is not: ' +
    'save_parameters (main.c) passes ak_flight_config_writable() into ' +
    'ak_params_save, and that function returns true only when the aircraft is ' +
    'disarmed — it is refused otherwise because the write erases a flash sector ' +
    'with the core stopped, which an aircraft in the air cannot afford. A set ' +
    'has no such deadline, which is why it has no such guard, and why this is a ' +
    'note rather than a block. The gate you are seeing is in this app: a ' +
    'different client would not have it.',
  citations: [
    'aerialkit/src/core/ak_proto.h:AK_PROTO_FEATURE_GATES_ON_ARMED',
    'aerialkit/src/core/main.c:save_parameters',
    'aerialkit/src/core/flight/ak_flight.c:ak_flight_config_writable',
  ],
};

/** The same, on a board whose hello ended before the capability word. */
const NO_ARMED_GUARD_ON_SET_UNKNOWN: Limitation = {
  id: 'no-armed-guard-on-set',
  severity: 'note',
  summary: 'the board would take a parameter value while armed; this app will not send one',
  detail:
    'This board\'s hello reply ended before the capability word, so it predates ' +
    'AK_PROTO_FEATURE_GATES_ON_ARMED (ak_proto.h) and cannot say whether a set ' +
    'is checked against the flight state — a revision where it is and a ' +
    'revision where it is not produce the same bytes here. Saving is guarded on ' +
    'every revision that has the argument at all: save_parameters (main.c) ' +
    'passes ak_flight_config_writable() into ak_params_save, which returns true ' +
    'only when the aircraft is disarmed. Because a set might not be checked, ' +
    'this app gates it, and the gate you are seeing is in this app: a different ' +
    'client would not have it.',
  citations: [
    'aerialkit/src/core/ak_proto.h:AK_PROTO_FEATURE_GATES_ON_ARMED',
    'aerialkit/src/core/main.c:save_parameters',
    'aerialkit/src/core/flight/ak_flight.c:ak_flight_config_writable',
  ],
};

/*
 * Milestone 3's entry, and the one that would have been easiest to write as a
 * half-truth.
 *
 * Until 2026-09-30 this app held a build-time snapshot of the firmware's
 * parameter table in `src/firmware/parameter-table.json` and showed its ranges,
 * decimal counts and help sentences beside whatever board answered. It was 32
 * rows against a 92-parameter board, and the missing row was not visible,
 * because every consumer joined the file to the board's reply *by name* — a name
 * the file lacked produced a row with no range, which reads exactly like a
 * parameter that has none.
 *
 * The file is a fixture for the demo board now and nothing else: a real board's
 * description arrives over `param info` and `param help` or it does not arrive.
 * So the honest sentence for a board that cannot describe itself is not "the
 * ranges you see came from a file" — nothing is shown at all. The rows are
 * still the board's own, read over `param get`: names, indexes and values are
 * real. It is the *description* that is missing, and a person tuning an
 * aircraft is owed the difference between "this parameter has no stated range"
 * and "this app cannot see one".
 */
const RANGES_FROM_A_BUILD_TIME_READ: Limitation = {
  id: 'ranges-from-a-build-time-read',
  severity: 'note',
  summary: 'no range, decimal count or help sentence is shown for any parameter',
  detail:
    'This board reported a capability word and does not claim ' +
    'AK_PROTO_FEATURE_PARAM_INFO, so its own answer is that it cannot describe ' +
    'its parameters. The protocol carries `param info` and `param help` ' +
    '(ak_proto.c, AK_PROTO_CMD_PARAM_INFO and AK_PROTO_CMD_PARAM_HELP) and this ' +
    'board does not answer them. Every name, index and value below is the ' +
    'board\'s own, read over `param get`; what is missing is the description — ' +
    'so each row says "no description" rather than showing a range from ' +
    'anywhere else. This app holds no fallback table, deliberately: a file of ' +
    'ranges shown beside a board that did not send them is a second authority, ' +
    'and it was wrong for a whole release.',
  citations: [
    'aerialkit/src/core/ak_proto.h:AK_PROTO_FEATURE_PARAM_INFO',
    'aerialkit/src/core/ak_proto.c:AK_PROTO_CMD_PARAM_INFO',
    'aerialkit/src/core/ak_proto.c:AK_PROTO_CMD_PARAM_HELP',
  ],
};

/** The same, on a board whose hello ended before the capability word. */
const RANGES_FROM_A_BUILD_TIME_READ_UNKNOWN: Limitation = {
  id: 'ranges-from-a-build-time-read',
  severity: 'note',
  summary: 'no range, decimal count or help sentence is shown for any parameter',
  detail:
    'This board\'s hello reply ended before the capability word, so it predates ' +
    'AK_PROTO_FEATURE_PARAM_INFO (ak_proto.h) and cannot say whether it answers ' +
    '`param info`. This app asked anyway, once, and read the answer rather than ' +
    'assuming: the reply was the protocol\'s unknown-command byte, 0x7f ' +
    '(ak_proto.c, the default case). That is the board saying it has no such ' +
    'command, which is why every row here is named and valued but not described. ' +
    'A reflash to a firmware that answers it fills all of this in.',
  citations: [
    'aerialkit/src/core/ak_proto.h:AK_PROTO_FEATURE_PARAM_INFO',
    'aerialkit/src/core/ak_proto.c:AK_PROTO_CMD_PARAM_INFO',
    'aerialkit/src/core/ak_proto.c:AK_PROTO_CMD_PARAM_HELP',
  ],
};

const ARMED_STATE_GATES_WRITES: Limitation = {
  id: 'armed-state-gates-writes',
  severity: 'blocks-writes',
  summary: 'writes need a disarmed aircraft and a fresh reading of its state',
  detail:
    'The armed state comes from the flight_state byte of the most recent ' +
    'STATUS or telemetry frame (ak_proto.h, ak_proto_status_t; the numbering ' +
    'is ak_flight.h\'s — 0 disarmed, 1 armed, 2..5 failsafe, returning home, ' +
    'on autopilot, circling down, and everything from 2 up counts as armed ' +
    'here because the aircraft is flying itself). When that reading goes ' +
    'stale the state becomes unknown, and unknown is not disarmed: a board ' +
    'that armed since the last frame would look identical to one that had ' +
    'not, so no write is enabled until a fresh frame arrives.',
  citations: [
    'aerialkit/src/core/ak_proto.h:ak_proto_status_t',
    'aerialkit/src/core/flight/ak_flight.h:ak_flight_state_t',
    'aerialkit/src/core/flight/ak_flight.h:AK_FLIGHT_DISARMED',
  ],
};

const NO_PERSISTENCE: Limitation = {
  id: 'no-persistence',
  severity: 'blocks-writes',
  summary: 'this board has nowhere to save, so nothing survives a power cycle',
  detail:
    'The board answered `param save` with "nowhere to save" (status 3). ' +
    'Values can still be written into the running table, but they are gone ' +
    'when the board loses power. A device with no config flash says this ' +
    'rather than reporting a successful save.',
  citations: [
    'aerialkit/src/core/ak_params.h:ak_params_save',
    'aerialkit/src/core/ak_params.c:ak_params_save',
  ],
};

/*
 * This was `CONSOLE_CANNOT_STREAM`, id `console-link-cannot-stream`, until
 * 2026-09-20, and the name was wrong in a way that mattered: it described a
 * mechanism (a console link that cannot stream) rather than the observation
 * (a link that promised a rate and sent nothing). A console link *refuses* -
 * it answers 0 by design, and this app reports that as a refusal, not as this
 * limitation. So the case that reaches here is the opposite one: a link that
 * streams answered with a non-zero rate, and the frames did not arrive.
 */
const STREAM_PROMISED_NOT_DELIVERED: Limitation = {
  id: 'stream-promised-not-delivered',
  severity: 'note',
  summary: 'this link agreed to a telemetry rate and has sent nothing',
  detail:
    'The board answered a telemetry subscribe with a non-zero rate and has ' +
    'not pushed a frame since. That non-zero answer is the whole reason to ' +
    'expect one, and it is what makes this a promise rather than a refusal: ' +
    'on this firmware a console link answers 0 by design, because it is the ' +
    'wire a person types at and frames arriving among their keystrokes is a ' +
    'console nobody can use (ak_proto.h, can_stream - main.c sets it on the ' +
    'network link and not on the console). So a non-zero rate came from a ' +
    'link that streams, and what is missing is the frames. Subscribed is not ' +
    'the same fact as frames arriving, so the values below come from polling ' +
    'STATUS.',
  citations: [
    'aerialkit/src/core/ak_proto.h:can_stream',
    'aerialkit/src/core/ak_proto.c:AK_PROTO_CMD_TELEMETRY',
    'aerialkit/src/core/main.c:can_stream',
  ],
};

/*
 * The five entries above were unconditional, and the two that describe a
 * firmware *capability* were unconditional in a way that had gone false: they
 * asserted, on every connection, the weakest thing that could be said about any
 * revision. They are readings now. A board that reports APPLIES_ON_WRITE does
 * not get told it might not re-apply; a board that reports GATES_ON_ARMED does
 * not get told its writes are unguarded. A board that reports *neither the bit
 * nor the word* still gets both — one wording for "it says no", another for
 * "it cannot say" — because collapsing those two is the mistake this file
 * exists to prevent.
 *
 * The third is milestone 3's: a board that cannot describe itself gets no
 * ranges, and says why. It is the entry that replaced a build-time snapshot of
 * the firmware's own table, which is how an off-by-one lived in this app for a
 * release without anyone seeing it.
 */
export function limitationsFor(facts: LimitationFacts): Limitation[] {
  const out: Limitation[] = [ARMED_STATE_GATES_WRITES];
  if (facts.noPersistence) out.push(NO_PERSISTENCE);
  if (facts.streamPromisedNotDelivered) out.push(STREAM_PROMISED_NOT_DELIVERED);
  if (!has(facts.capabilities, Feature.APPLIES_ON_WRITE)) {
    out.push(
      facts.capabilities === null ? NO_APPLICATION_CONFIRMATION_UNKNOWN : NO_APPLICATION_CONFIRMATION,
    );
  }
  if (!has(facts.capabilities, Feature.GATES_ON_ARMED)) {
    out.push(
      facts.capabilities === null ? NO_ARMED_GUARD_ON_SET_UNKNOWN : NO_ARMED_GUARD_ON_SET,
    );
  }
  if (!has(facts.capabilities, Feature.PARAM_INFO)) {
    out.push(
      facts.capabilities === null
        ? RANGES_FROM_A_BUILD_TIME_READ_UNKNOWN
        : RANGES_FROM_A_BUILD_TIME_READ,
    );
  }
  return out;
}

export const ALL_LIMITATIONS = {
  ARMED_STATE_GATES_WRITES,
  NO_APPLICATION_CONFIRMATION,
  NO_APPLICATION_CONFIRMATION_UNKNOWN,
  NO_ARMED_GUARD_ON_SET,
  NO_ARMED_GUARD_ON_SET_UNKNOWN,
  NO_PERSISTENCE,
  RANGES_FROM_A_BUILD_TIME_READ,
  RANGES_FROM_A_BUILD_TIME_READ_UNKNOWN,
  STREAM_PROMISED_NOT_DELIVERED,
};
