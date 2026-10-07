/**
 * The wire, as the firmware defines it in `src/core/ak_proto.h`.
 *
 * These numbers are duplicated rather than generated. The firmware is C and the
 * app is TypeScript, and a code generator between them would be a third thing to
 * keep correct; the protocol is small enough that the honest trade is a short
 * file with a comment pointing at its source, plus the golden vectors in
 * `tests/fixtures` that catch it when the two drift.
 */

export const SYNC1 = 0xaa;
export const SYNC2 = 0x55;

/** `AK_PROTO_VERSION`. A frame carrying anything else is not ours to read. */
export const PROTOCOL_VERSION = 1;

/** The bit a reply sets on the command it answers. */
export const RESPONSE_BIT = 0x80;

/** `AK_PROTO_MAX_PAYLOAD` — the largest payload the firmware will emit. */
export const MAX_PAYLOAD = 96;

/** `AK_PROTO_FRAME_MAX` — sync pair, version, command, length, payload, crc. */
export const MAX_FRAME = 6 + MAX_PAYLOAD + 2;

/** `AK_PROTO_GAP_MS` — how long a partial frame may stall before it is dropped. */
export const GAP_MS = 50;

/**
 * The client's own stall bound, which is *not* the firmware's. The firmware
 * measures silence between bytes on its UART; this side measures the time
 * between two calls of a JavaScript data handler, and a frame the board sent
 * contiguously can arrive split across two of them with a render or a layout
 * in between. At 50 ms a busy main thread dropped valid telemetry and log
 * frames as "stalled" (found by the 2026-10-06 audit). A quarter second is
 * past any render and still an order of magnitude inside a request's timeout.
 */
export const CLIENT_GAP_MS = 250;

/** `AK_PROTO_TELEMETRY_MAX_HZ`. Asking for more is answered, not refused. */
export const TELEMETRY_MAX_HZ = 50;

/**
 * `AK_PROTO_LOG_STREAM_MAX_HZ`. The same rule as telemetry's: a request for
 * more comes back with what will actually be sent, not a refusal.
 *
 * It is 50 for the same reason telemetry is, and it is worth saying which
 * reason that is, because a log stream and a telemetry stream are not the same
 * traffic. A record is 87 bytes and a status frame is 26, so the log stream
 * costs about three times what telemetry does at the same rate. What keeps both
 * honest is that they share the link with the config channel: a stream that
 * filled the socket buffer would delay the reply to a `param set`, which is
 * the failure a configurator is least able to explain.
 */
export const LOG_STREAM_MAX_HZ = 50;

export const enum Command {
  HELLO = 0x01,
  PARAM_GET = 0x02,
  PARAM_SET = 0x03,
  PARAM_SAVE = 0x04,
  STATUS = 0x05,
  LOG_INFO = 0x06,
  LOG_GET = 0x07,
  TELEMETRY = 0x08,
  LOG_SOURCE = 0x09,
  PARAM_INFO = 0x0a,
  PARAM_HELP = 0x0b,
  PARAM_DEFAULT = 0x0c,
  RC_CHANNELS = 0x0d,
  SENSOR_INFO = 0x0e,
  OUTPUT_INFO = 0x0f,
  OUTPUT_TEST = 0x10,
  LOG_STREAM = 0x11,
  PREFLIGHT = 0x12,
  CALIBRATE = 0x13,
  MISSION = 0x14,
  /** The loop profiler's window, as the console's `perf` prints it. A read. */
  PERF = 0x15,
  MOTOR_TELEMETRY = 0x16,
}

/**
 * A pushed log frame's status byte, from `AK_PROTO_LOG_STREAM_*`.
 *
 * `DONE` is the one that matters most: it is a frame rather than a silence, so
 * a reader can tell a range that finished from a link that died. `HOLE` is the
 * other one — a record the board refused, named by index, which is a fact the
 * reader has rather than a gap it has to infer from a count.
 */
export const enum LogStreamStatus {
  RECORD = 0,
  HOLE = 1,
  DONE = 2,
}

/**
 * `RC_CHANNELS`'s status byte.
 *
 * `NONE` is worth its own value rather than a frame of zeros: a board with no
 * receiver port and a receiver that is plugged in and has never framed would
 * otherwise be the same reply, and they are a firmware fact and a wiring fault
 * respectively.
 */
export const enum RcStatus {
  OK = 0,
  /** This board has no receiver input. The reply is one byte and stops. */
  NONE = 1,
}

/** The facts about a receiver, from `AK_PROTO_RC_*` in `ak_proto.h`. */
export const enum RcFlag {
  /** A channel frame has arrived. */
  LINK = 1 << 0,
  /** The receiver's own failsafe is set: it has lost its transmitter. */
  FAILSAFE = 1 << 1,
  /**
   * The four sticks in this reply are this frame's, decoded by the firmware.
   *
   * **The flag that keeps two replies apart whose sticks are both all zero**: a
   * frame the decode would not use, and a handset sitting centred. Same bytes,
   * different meanings, and drawing them the same way would tell a person their
   * sticks are centred when the board has no idea.
   */
  DECODED = 1 << 2,
  /** This board's receiver pin has no inverter in front of it. Only meaningful
   *  on SBUS, where it explains a link that reads as framing errors. */
  NO_INVERTER = 1 << 3,
  /** There is a return path to the handset. CRSF has one; SBUS does not. */
  TELEMETRY = 1 << 4,
}

export const enum RcSwitch {
  ARM_ON = 1 << 0,
  ANGLE = 1 << 1,
}

/** `AK_PROTO_RC_MAX` — the most channels a reply can carry. */
export const RC_MAX_CHANNELS = 16;

/**
 * `ak_rc_protocol_t`. The wire sends the number and the names live in
 * `ak_rc_protocol_name()` (`ak_rc_receiver.c`), which a client cannot call —
 * the same rule the parameter groups follow, and for the same reason: a number
 * this app cannot resolve must render as unknown, never as a neighbour's name.
 */
export const RC_PROTOCOLS: Record<number, string> = {
  0: 'CRSF',
  1: 'SBUS',
};

/** The four sticks, in the order the reply carries them. */
export const RC_STICKS = ['roll', 'pitch', 'yaw', 'throttle'] as const;

/**
 * `MOTOR_TELEMETRY`'s status byte.
 *
 * The same two answers `RcStatus` distinguishes, about a different port. `NONE`
 * is this build having no way to hear an ESC, and its reply is one byte and
 * stops — because a header and four entries of zeros would read on this screen
 * as four motors that are stopped, which is a claim about hardware the board
 * does not have.
 *
 * The middle answer has no value of its own: status `OK` with every motor's
 * `MEASURED` flag clear is "the board has the path and has heard nothing yet".
 */
export const enum MotorStatus {
  OK = 0,
  /** This build has no motor telemetry path. The reply is one byte and stops. */
  NONE = 1,
}

/** The facts about one motor, from `AK_PROTO_MOTOR_FLAG_*` in `ak_proto.h`. */
export const enum MotorFlag {
  /**
   * The ESC answered: `erpm` is a reading rather than a placeholder.
   *
   * **The flag that keeps the three replies apart.** No telemetry path, no
   * telemetry heard, and a motor that reported zero are all "0" on a screen,
   * and only this bit says which of them a row is showing.
   */
  MEASURED = 1 << 0,
  /**
   * `rpm` is a number rather than a placeholder.
   *
   * Set only when `MEASURED` is set *and* the board was told the motor's pole
   * count. An eRPM with no pole count is a true reading of something that is
   * not a speed, and this app does not divide it by a guessed fourteen to make
   * it look like one — the console's `dshot` command refuses for the same
   * reason.
   */
  RPM = 1 << 1,
  /** `temperature` and `maxTemperature` are readings. */
  TEMPERATURE = 1 << 2,
  VOLTAGE = 1 << 3,
  CURRENT = 1 << 4,
}

/**
 * `PERF`'s status byte. `NONE` is a build with no profiler in it — a fact about
 * the firmware, not a loop that costs nothing — and its reply is one byte.
 */
export const enum PerfStatus {
  OK = 0,
  NONE = 1,
}

/** `AK_PROTO_PERF_SECTIONS` — the named loop sections a PERF reply carries. */
export const PERF_SECTIONS = 5;

/** `AK_PROTO_MOTOR_MAX` — the most motors a reply can carry. */
export const MOTOR_MAX = 4;

/**
 * One entry's size on the wire, from `ak_proto_motor_t`: flags (1), eRPM and rpm
 * (4 each), temperature and its session maximum (1 each), millivolts and
 * milliamps (2 each), the window's packets and invalid (2 each).
 */
export const MOTOR_ENTRY_BYTES = 19;

/** The flag bits, by name, in the order `ak_proto.h` declares them. Facts
 *  rather than warnings: which of them is worth colouring red is this screen's
 *  business and not the firmware's. */
export const MOTOR_FLAGS: ReadonlyArray<{ readonly bit: MotorFlag; readonly name: string }> = [
  { bit: MotorFlag.MEASURED, name: 'measured' },
  { bit: MotorFlag.RPM, name: 'rpm' },
  { bit: MotorFlag.TEMPERATURE, name: 'temperature' },
  { bit: MotorFlag.VOLTAGE, name: 'voltage' },
  { bit: MotorFlag.CURRENT, name: 'current' },
];

/** One bit of a flags byte. */
export function flagSet(bits: number, flag: number): boolean {
  return (bits & flag) !== 0;
}

/**
 * `SENSOR_INFO`'s status byte.
 *
 * `NO_SUCH` is a claim about the *build*: this firmware does not answer for that
 * topic at all. It is deliberately not the same thing as a reply that says
 * `present: 0`, which is a claim about the *aircraft* — the build knows the
 * question and this board has nothing fitted. Collapsing them would send
 * somebody looking for a driver when the part is simply not soldered in, which
 * is the same rule `RcStatus.NONE` follows one level down.
 */
export const enum SensorStatus {
  OK = 0,
  NO_SUCH = 1,
}

/** The topics `SENSOR_INFO` answers about, from `AK_PROTO_SENSOR_*`. */
export const enum Sensor {
  IMU = 0,
  BARO = 1,
  RANGE = 2,
  BATTERY = 3,
  GPS = 4,
}

/** `AK_PROTO_SENSOR_TOPICS` — the number of topics, and one past the last. */
export const SENSOR_TOPICS = 5;

/** `AK_PROTO_SENSOR_NAME` — the fixed width of a body's driver-name field. */
export const SENSOR_NAME_LENGTH = 12;

/** The body each topic carries, counted from `ak_proto.h` the way the firmware
 *  counts it. A reply of any other length is refused rather than read. */
export const SENSOR_BODY_LENGTH: Readonly<Record<Sensor, number>> = {
  [Sensor.IMU]: 12 + 1 + 1 + 6 + 6 + 6 + 6 + 4 + 4,
  [Sensor.BARO]: 12 + 4 + 2 + 1 + 4 + 4 + 1 + 4 + 4 + 4 + 4 + 4 + 4,
  [Sensor.RANGE]: 12 + 1 + 2 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 2,
  [Sensor.BATTERY]: 1 + 1 + 1 + 1 + 2 + 2 + 2 + 2 + 1 + 2 + 2 + 4 + 4 + 4,
  [Sensor.GPS]: 1 + 1 + 1 + 1 + 1 + 4 + 4 + 4 + 4 + 4 + 1 + 4 + 4 + 4 + 4 + 1 + 1 + 4 + 4 + 4,
};

/** The topics in the order a screen should show them, with the name the
 *  firmware's own console command uses for each. */
export const SENSOR_TOPICS_ORDERED: ReadonlyArray<{
  readonly topic: Sensor;
  readonly name: string;
  readonly command: string;
}> = [
  { topic: Sensor.IMU, name: 'imu', command: 'imu' },
  { topic: Sensor.BARO, name: 'baro', command: 'baro' },
  { topic: Sensor.RANGE, name: 'range', command: 'range' },
  { topic: Sensor.BATTERY, name: 'battery', command: 'battery' },
  { topic: Sensor.GPS, name: 'gps', command: 'gps' },
];

/**
 * `OUTPUT_INFO`'s status byte.
 *
 * `NONE` is a claim about the *board*: there is no list. Either it has nothing
 * to drive or it is not ready to say so, and neither is an aircraft with zero
 * motors — which is why the firmware answers this rather than a well-formed
 * page with a count of zero, and why this app must not draw the two the same
 * way. `TOO_MANY` is a refusal rather than a page: the board has more outputs
 * than one frame carries.
 */
export const enum OutputInfoStatus {
  OK = 0,
  NONE = 1,
  TOO_MANY = 2,
}

/**
 * The kinds of output, from `AK_PROTO_OUTPUT_*`.
 *
 * A number this app cannot name renders as unknown, the rule the parameter
 * groups, the receiver protocols and the GPS fix types all follow: the wire
 * carries a number, the names live in the firmware, and guessing a neighbour's
 * name would be inventing a fact about the aircraft.
 */
export const enum OutputKind {
  MOTOR = 0,
  SERVO = 1,
}

export const OUTPUT_KIND_NAMES: Readonly<Record<number, string>> = {
  [OutputKind.MOTOR]: 'motor',
  [OutputKind.SERVO]: 'servo',
};

/**
 * Seven bytes a descriptor, counted from `ak_proto.h` the way the firmware
 * counts it in `ak_proto.c` — `kind, index, reversed, trim (i16), travel (u16)`.
 *
 * Not `sizeof`: the C struct has padding and the wire does not, which is the
 * whole reason the firmware's own comment says the seven is counted in
 * `append_*` calls. A reply whose length disagrees with its own count is
 * refused rather than read, because reading on would take the next entry's kind
 * byte for this one's index.
 */
export const OUTPUT_ENTRY_BYTES = 7;

/** `AK_PROTO_OUTPUT_MAX` — the most descriptors one reply carries. The firmware
 *  refuses a board with more rather than paging it, and the number is
 *  `(AK_PROTO_MAX_PAYLOAD - 5) / 7`, which is where the 13 comes from. */
export const OUTPUT_MAX = Math.floor((MAX_PAYLOAD - 5) / OUTPUT_ENTRY_BYTES);

/**
 * The `op` byte on `OUTPUT_TEST`.
 *
 * `STOP` is a value rather than a second command because the two share every
 * other argument and every gate, and a client that has lost track of the state
 * must be able to send one without asking first. It is also answered *before*
 * every other check — a stop arrives even while the aircraft is armed — which
 * is the property that makes it the safe thing to send when unsure.
 */
export const enum OutputTestOp {
  HOLD = 0,
  STOP = 1,
}

/** `OUTPUT_TEST`'s status byte, from `AK_PROTO_OUTPUT_TEST_*`. */
export const enum OutputTestStatus {
  OK = 0,
  STOPPED = 1,
  NO_OUTPUT = 2,
  ARMED = 3,
  NO_BOARD = 4,
  NO_OP = 5,
}

/**
 * What each status means, in the firmware's own terms.
 *
 * `STOPPED` is deliberately not phrased as a refusal: a stop is a success, and
 * the four that are refusals each name what did not happen. A screen that wrote
 * "failed" for all five would be reporting a policy decision the board made
 * (ARMED) as a fault.
 */
export const OUTPUT_TEST_STATUS_TEXT: Readonly<Record<OutputTestStatus, string>> = {
  [OutputTestStatus.OK]: 'holding',
  [OutputTestStatus.STOPPED]: 'stopped',
  [OutputTestStatus.NO_OUTPUT]: 'this board has no output with that number',
  [OutputTestStatus.ARMED]: 'refused: the aircraft is armed',
  [OutputTestStatus.NO_BOARD]: 'this board drives no outputs',
  [OutputTestStatus.NO_OP]: 'the request named no verb',
};

/**
 * `AK_PROTO_OUTPUT_TEST_MAX_MS` and `AK_PROTO_OUTPUT_TEST_MAX_PCT`.
 *
 * **Both are the firmware's constants, not the client's arguments.** The run
 * time is enforced on the board and reported back as `remaining_ms`, so the
 * panel counts down from the board's number rather than its own; the level is
 * clamped on the board, so asking for 200 comes back as the cap. These two
 * copies are here to *render* the ceiling and to sanity-check a reply, and a
 * client that used them to decide what it was allowed to ask for would still be
 * asking — the board is the one that answers.
 */
export const OUTPUT_TEST_MAX_MS = 500;
export const OUTPUT_TEST_MAX_PCT = 15;

/**
 * Why a board has no IMU, as `AK_PROTO_IMU_ABSENT_*` numbers them.
 *
 * Not `ak_imu_result_t`'s values: that enum is the driver layer's and this is
 * the wire's, and pinning one to the other would make renumbering either a
 * protocol change. A number this app cannot name renders as unknown.
 */
export const IMU_ABSENT: Readonly<Record<number, string>> = {
  1: 'nothing answered on the bus',
  2: 'something answered, and it is not a known part',
  3: 'the right part answered and would not configure',
};

/** `ak_battery_state_t`, by the name the console prints. */
export const BATTERY_STATES: Readonly<Record<number, string>> = {
  0: 'absent',
  1: 'ok',
  2: 'warn',
  3: 'critical',
};

/**
 * `ak_gps_fix_t` — **the receiver's own numbering, deliberately untranslated.**
 *
 * `ak_gps.h` says so in as many words, and it is why these names are u-blox's
 * rather than MAVLink's: MAVLink's `GPS_FIX_TYPE` numbers the same ideas
 * differently (its 3 is a 3D fix and so is this one, but its 1 is "no fix"
 * where this one's 1 is dead reckoning), so a table borrowed from the other
 * adapter would name every fix type wrong in a way that looks plausible.
 *
 * A number this app cannot name renders as unknown, the same rule the parameter
 * groups and the receiver protocols follow.
 */
export const GPS_FIX_TYPES: Readonly<Record<number, string>> = {
  0: 'no fix',
  1: 'dead reckoning only',
  2: '2D fix',
  3: '3D fix',
  4: '3D fix with dead reckoning',
  5: 'time only, no position',
};

/**
 * The byte a board answers any command it does not implement with.
 *
 * From `ak_proto.c`'s `default:` case: the reply is correlated to the request —
 * it carries the request's own command byte with the response bit — and its
 * payload is this one byte. So a board that predates `PARAM_INFO` does not go
 * silent; it says this, which is a readable answer and not a timeout.
 */
export const UNKNOWN_COMMAND = 0x7f;

/** `AK_PARAM_*` from `ak_params.h` — what an entry says it is. */
export const enum ParamType {
  FLOAT = 0,
  U32 = 1,
  TEXT = 2,
}

export const PARAM_TYPE_NAMES: Record<number, string> = {
  [ParamType.FLOAT]: 'float',
  [ParamType.U32]: 'u32',
  [ParamType.TEXT]: 'text',
};

/**
 * `ak_param_group_t`, from `ak_params.h`.
 *
 * **A client carries a copy of this, exactly as it carries a copy of the
 * command numbers.** The wire sends the group as one byte and nothing else: the
 * names live in `ak_param_group_name()` (`ak_params.c`), which a client cannot
 * call. The protocol's own client does the same thing for the same reason
 * (`tools/akproto.py`, `PARAM_GROUPS`), and states the rule that makes it
 * safe — a number this app does not know renders as unknown, *never* as a
 * neighbour's name.
 *
 * The alternative was to ask the board for the strings, which would mean a
 * second opcode and a paging walk to serve thirteen words that change once a
 * year. A group heading is a reading aid; a wrong one is worse than none, and
 * the "unknown" rendering is what keeps that true.
 */
export const PARAM_GROUPS: Record<number, string> = {
  0: 'none',
  1: 'rates',
  2: 'angle',
  3: 'arming',
  4: 'receiver',
  5: 'airframe',
  6: 'outputs',
  7: 'power',
  8: 'failsafe',
  9: 'navigation',
  10: 'sensors',
  11: 'network',
  12: 'timing',
};

/** What a `param info` page says about itself. */
/**
 * A group number as a heading, or `null` for one this app has no name for.
 *
 * The wire carries a *number* and every client carries its own copy of the
 * names — the same rule `tools/akproto.py` documents on the firmware's side.
 * `null` is a real answer and must render as unknown: a number that resolved to
 * a neighbour's name would be this app inventing a heading, and a person would
 * have no way to tell.
 */
export function groupHeading(group: number): string | null {
  const name = PARAM_GROUPS[group];
  if (name === undefined) return null;
  return name.charAt(0).toUpperCase() + name.slice(1);
}

export const enum InfoStatus {
  /** The page carries entries. */
  OK = 0,
  /** The request named no index. The reply is one byte and stops there. */
  NO_INDEX = 1,
  /**
   * The row at this index exists and does not fit one frame at any size. **Not
   * the end of the table** — a client that read it as one would stop early and
   * never learn the row exists. The walk steps over it and records the hole.
   */
  TOO_BIG = 2,
}

/**
 * The status byte of a write, in the firmware's own vocabulary (`SET_STATUS` in
 * `tools/akproto.py`, from the enum in `ak_proto.c`).
 *
 * `REFUSED` is the interesting one: it is the only case where the board also
 * sends a sentence, and that sentence is the only place a range is ever stated.
 */
export const enum SetStatus {
  OK = 0,
  NO_SUCH_PARAMETER = 1,
  VALUE_REFUSED = 2,
  NOWHERE_TO_SAVE = 3,
  /**
   * The board's storage took the request and the *write* failed — a flash
   * sector that would not erase. This is the storage's own error and it has
   * been number 4 since before there was a status 5, which is why it keeps the
   * number even though it now reads out of order beside `REFUSED_ARMED`.
   * Renumbering it would change what an existing client reads off an existing
   * opcode. See the enum in `ak_proto.h`, which makes the same argument.
   */
  STORAGE_ERROR = 4,
  /**
   * Refused because the aircraft is armed — the firmware's own gate, not this
   * app's. It arrives on a board whose capability word claims
   * `GATES_ON_ARMED`, and it is the one refusal a *different* client would also
   * have got.
   *
   * It is 5 and not 4 because 4 was already spoken for, and a status byte whose
   * meaning depends on which firmware answered is worse than a gap in the
   * numbering.
   */
  REFUSED_ARMED = 5,
}

export function setStatusName(status: number): string {
  switch (status) {
    case SetStatus.OK:
      return 'accepted';
    case SetStatus.NO_SUCH_PARAMETER:
      return 'no such parameter';
    case SetStatus.VALUE_REFUSED:
      return 'value refused';
    case SetStatus.NOWHERE_TO_SAVE:
      return 'nowhere to save';
    case SetStatus.STORAGE_ERROR:
      return 'the board could not write its storage';
    case SetStatus.REFUSED_ARMED:
      return 'refused: the aircraft is armed';
    default:
      return `unknown status ${status}`;
  }
}

/**
 * The flight-state byte, by name.
 *
 * `AK_FLIGHT_DISARMED` is 0 through `AK_FLIGHT_DESCEND` at 5, from
 * `ak_flight.h`. Live and the blackbox both decode this byte, and a second copy
 * of the list is how the two would come to disagree about what the same byte
 * means — a disagreement nothing would catch, because both would render.
 */
export const FLIGHT_STATE_NAMES = [
  'disarmed',
  'armed',
  'failsafe',
  'return to home',
  'autopilot',
  'descend',
] as const;

/** The three logs a device may have. Which ones exist is the device's answer. */
export const enum LogSource {
  FAST = 0,
  LONG = 1,
  FLASH = 2,
}

export const LOG_SOURCE_NAMES: Record<LogSource, string> = {
  [LogSource.FAST]: 'fast',
  [LogSource.LONG]: 'long',
  [LogSource.FLASH]: 'flash',
};

export const LOG_SOURCE_DESCRIPTIONS: Record<LogSource, string> = {
  [LogSource.FAST]: 'the fast ring in RAM, 250 Hz for about 1.5 s',
  [LogSource.LONG]: 'the long ring, 25 Hz for about 15 s, survives a reset',
  // The one description here whose duration is not something the wire can
  // establish, and it says where the number came from instead of just stating
  // it. `log info` answers with how many records the ring is *holding*, never
  // with how many it can hold — the capacity is a property of the part's
  // sectors, and the only place it exists is the board file. The figure is
  // four 128 KB sectors (`src/boards/FEATHER_F405/board.c`) at 1365 slots each
  // after the header (`src/core/ak_flashlog.h`, asserted at
  // `tests/test_arch.c`), and 5 460 records at 5 Hz is 18 minutes. It said
  // "about an hour" until 2026-10-01, which was the geometry two record
  // growths ago, 10 920 until 2026-10-03, when the image took a sector, and
  // 8 736 until 2026-10-04, when the record grew to 80 bytes to carry the
  // filtering fields (roadmap 2.4), and 6552 until 2026-10-05, when it grew to
  // 96 to carry the controller's (roadmap 4.1).
  [LogSource.FLASH]:
    'the log in flash, 5 Hz, survives the battery — the F405’s four 128 KB sectors hold 5 460 records, about 18 minutes; that is the board file’s arithmetic, not something the board reports',
};

/**
 * `PREFLIGHT`'s status byte, from `AK_PROTO_PREFLIGHT_*`.
 *
 * `NONE` and `NO_INDEX` are two different facts and are kept apart for the same
 * reason `SensorStatus.NO_SUCH` and `present: 0` are: `NONE` says this board has
 * no checklist to serve — a build without one, or a device that is not an
 * aircraft — and `NO_INDEX` says the walk went off the end of a checklist that
 * exists. Rendering the second as the first would hide a bug in this app's own
 * loop behind a statement about the board.
 *
 * **There is no `OK`-with-nothing case.** A line with an empty sentence is a
 * page with `OK`, a stated total of zero and a stated length of zero, which is
 * a reading rather than an absence.
 */
export const enum PreflightStatus {
  OK = 0,
  NO_INDEX = 1,
  NONE = 2,
}

/**
 * A line's verdict, from `AK_PROTO_PREFLIGHT_VERDICT_*`.
 *
 * These are the console's own three columns, and the app draws them with the
 * console's own words rather than inventing a fourth: `FAIL` is a fault to act
 * on, `PASS` is a check that ran and was happy, and `FACT` is a reading that is
 * neither — the gyro bias, the log counts, which part is fitted. A build that
 * made every FACT a PASS would be claiming to have checked things it only
 * measured.
 */
export const enum PreflightVerdict {
  FAIL = 0,
  PASS = 1,
  FACT = 2,
}

/**
 * The console's own left-hand column for each verdict, spaces and all.
 *
 * Copied rather than derived, because these six-character strings are what
 * `main.c`'s `preflight_marker` returns and `tools/akproto_firmware_check.py`
 * asserts the console's lines equal this plus the sentence. A client that
 * padded them differently would be a second rendering of one checklist — which
 * is the thing the design exists to prevent.
 */
export const PREFLIGHT_MARKERS: Readonly<Record<PreflightVerdict, string>> = {
  [PreflightVerdict.FAIL]: 'FAIL  ',
  [PreflightVerdict.PASS]: 'ok    ',
  [PreflightVerdict.FACT]: '--    ',
};

/** The word for each verdict on a tab, where there is room for one. */
export const PREFLIGHT_VERDICT_NAMES: Readonly<Record<PreflightVerdict, string>> = {
  [PreflightVerdict.FAIL]: 'FAIL',
  [PreflightVerdict.PASS]: 'ok',
  [PreflightVerdict.FACT]: 'reading',
};

/** `AK_PROTO_PREFLIGHT_NAME_MAX` — the widest name a page will carry. */
export const PREFLIGHT_NAME_MAX = 24;

/** `AK_PROTO_PREFLIGHT_MAX` — the most lines one checklist can hold. */
export const PREFLIGHT_MAX_LINES = 32;

/**
 * `CALIBRATE`'s six verbs, from `AK_PROTO_CALIBRATE_*` in `ak_proto.h`.
 *
 * These are the console's own four calibrations plus the read and the abort,
 * one number each, and the mapping is deliberately the obvious one: a person
 * who knows what `calibrate accel 3` does at the console knows what `ACCEL`
 * with face 3 does here, because it is the same measurement writing the same
 * parameter names.
 *
 * `ABORT` exists because a session started over this wire can outlive the frame
 * that started it. The console's version has no need of it — a person waiting
 * out a blocked loop can only wait — but a wizard whose user has just picked the
 * aircraft up must be able to stop the sampling without powering the board down,
 * and a session that could only be ended by finishing would answer `BUSY`
 * forever.
 */
export const enum CalibrateVerb {
  /** The only read: what the running session is doing, if there is one. */
  STATUS = 0,
  GYRO = 1,
  RC = 2,
  ACCEL = 3,
  VBAT = 4,
  ABORT = 5,
}

/** The verbs by the name the firmware's own console uses for each. The two must
 *  not call one verb two things, so this is copied from `akproto.py`'s table
 *  rather than spelled again here. */
export const CALIBRATE_VERB_NAMES: Readonly<Record<number, string>> = {
  [CalibrateVerb.STATUS]: 'status',
  [CalibrateVerb.GYRO]: 'gyro',
  [CalibrateVerb.RC]: 'rc',
  [CalibrateVerb.ACCEL]: 'accel',
  [CalibrateVerb.VBAT]: 'vbat',
  [CalibrateVerb.ABORT]: 'abort',
};

/**
 * `CALIBRATE`'s status byte, from `AK_PROTO_CALIBRATE_*`.
 *
 * Every one is a fact the aircraft holds rather than an opinion about what a
 * pilot meant, which is the test MISSION's refusals are held to. The pair worth
 * keeping apart is `NOTHING` and `NO_SAMPLES`: the first is a fact about the
 * *build* — `vbat` on a board with no pack divider, `rc` on a board with no
 * receiver port — and no amount of trying will change it; the second is a fact
 * about the *aircraft* — the hardware is there and it would not hold still, or
 * nothing arrived on the wire. Showing them the same way sends a person to look
 * at a connector that was never fitted.
 *
 * `NO_VERB` is the one refusal that is about the *request* rather than the
 * aircraft: the frame named no verb, or one this build does not have, or a
 * `VBAT` with no number behind it. The reply echoes the offending byte rather
 * than correcting it, so `0xFF` is "the frame did not say" and any other value
 * is "you asked for something this firmware does not implement".
 */
export const enum CalibrateStatus {
  OK = 0,
  NO_VERB = 1,
  /** Refused: the aircraft is armed. This is the write gate, on this opcode. */
  ARMED = 2,
  /** Refused: a session is already running. */
  BUSY = 3,
  /** Refused: this board has nothing to calibrate. A statement about the build. */
  NOTHING = 4,
  /** Ran, and did not get enough still samples. A statement about the aircraft. */
  NO_SAMPLES = 5,
  /** Ran, and the result is not one this aircraft will accept. */
  IMPLAUSIBLE = 6,
  /** Refused: there was nothing running to abort. */
  IDLE = 7,
  /** Refused: no such accelerometer face. */
  NO_FACE = 8,
  /** Refused: the voltage given is not a pack this aircraft flies. */
  BAD_VALUE = 9,
}

/**
 * What each status means, in the firmware's own terms.
 *
 * The refusals each name what did not happen, in the phrasing the client's own
 * error text and the console's sentences use, so a person reading the panel and
 * a person reading `akproto.py`'s output meet the same words. `OK` is "done"
 * rather than "accepted": the reply is a *reading* of the session, so the panel
 * draws the session rather than this word.
 *
 * **Three records of these ten sentences exist and all three are the same
 * text**: this table, `tools/akproto.py`'s `CALIBRATE_STATUS_NAMES`, and the
 * status table in `docs/16-protocol.md`. `tests/calibration.test.ts` reads the
 * other two off disk and fails on any disagreement, which is not hypothetical —
 * before that test existed this table said `ok` where the console says `done`,
 * and both the client and the document had dropped the `refused:` prefix from
 * `NO_VERB` that every other refusal carries.
 */
export const CALIBRATE_STATUS_TEXT: Readonly<Record<CalibrateStatus, string>> = {
  [CalibrateStatus.OK]: 'done',
  [CalibrateStatus.NO_VERB]: 'refused: no such verb',
  [CalibrateStatus.ARMED]: 'refused: the aircraft is armed',
  [CalibrateStatus.BUSY]: 'refused: a session is already running',
  [CalibrateStatus.NOTHING]: 'refused: nothing on this board to calibrate',
  [CalibrateStatus.NO_SAMPLES]: 'ran, and did not get enough still samples',
  [CalibrateStatus.IMPLAUSIBLE]:
    'ran, and the measurement is not one this aircraft will accept',
  [CalibrateStatus.IDLE]: 'refused: nothing to abort',
  [CalibrateStatus.NO_FACE]: 'refused: no such accelerometer face',
  [CalibrateStatus.BAD_VALUE]:
    'refused: the voltage given is not a pack this aircraft flies',
};

/**
 * `AK_PROTO_CALIBRATE_RESULT` — how many numbers a measurement carries.
 *
 * Six, and it is the accelerometer that sets it: three of bias and three of
 * scale, because that is the one calibration here that measures a scale as well
 * as an offset. The other three use a prefix of it and zero the rest, which is a
 * real zero — "this measurement has no fourth number" — and not a placeholder.
 */
export const CALIBRATE_RESULT = 6;

/**
 * What the six result slots mean, fixed by the verb that filled them.
 *
 * **The verb is in the reply beside them, so there is nothing to infer** — and
 * that is why this table is keyed by verb rather than being one list. A client
 * handed three gyro biases labelled with the accel scheme would print numbers
 * under names that describe a different measurement, and both readings are
 * plausible enough that nobody would catch it.
 *
 * The strings are `tools/akproto.py`'s `CALIBRATE_RESULT_MEANING` character for
 * character, so the panel and the command line name one number one way.
 */
export const CALIBRATE_RESULT_NAMES: Readonly<Record<number, readonly string[]>> = {
  [CalibrateVerb.GYRO]: ['bias roll (mdps)', 'bias pitch (mdps)', 'bias yaw (mdps)'],
  [CalibrateVerb.RC]: [
    'centre (us)',
    'roll off centre (us)',
    'pitch off centre (us)',
    'yaw off centre (us)',
  ],
  [CalibrateVerb.VBAT]: ['ratio (x1e6)'],
  [CalibrateVerb.ACCEL]: [
    'bias x (ug)',
    'bias y (ug)',
    'bias z (ug)',
    'scale x (x1e6)',
    'scale y (x1e6)',
    'scale z (x1e6)',
  ],
};

/**
 * `AK_PROTO_CALIBRATE_NO_STEP` — the `step` byte when no face is being sampled.
 *
 * Out of range for all six faces, so a client cannot read "not sampling" as
 * "sampling face zero" — the same trick `MISSION_NO_INDEX` plays, and for the
 * same reason: face zero is a real face and the two ends of the six-face flow
 * must not draw the same way.
 */
export const CALIBRATE_NO_STEP = 0xff;

/**
 * `AK_PROTO_CALIBRATE_NO_SESSION` — the `verb` byte when there is no session to
 * name, which is a board that has never calibrated anything.
 *
 * Out of range for all six for `NO_STEP`'s reason, and it is the one that would
 * bite hardest: the verb is what tells a client how to read the six result
 * slots, and a fresh board reading zero would be saying "these are the `status`
 * verb's numbers" beside six zeros that are not a measurement of anything.
 * `0xFF` is already the byte the dispatch answers when a client named no verb,
 * so it is the same "not a verb" a caller has already met in a refusal.
 */
export const CALIBRATE_NO_SESSION = 0xff;

/** `AK_PROTO_CALIBRATE_FACES` — the six accelerometer faces, one bit each. */
export const CALIBRATE_FACES = 6;

/** All six bits, so a wizard can count how much of the flow is done. */
export const CALIBRATE_ALL_FACES = 0x3f;

/**
 * `MISSION`'s five verbs, from `AK_PROTO_MISSION_*` in `ak_proto.h`.
 *
 * `STATUS` is the only read; the other four are the console's own verbs, one
 * number each. **There is deliberately no `add` and no `clear`.** The waypoints
 * are parameters — `wp0_lat`, `wp0_lon` and `wp_count` — so they are already on
 * this wire through `0x03`, they are range-checked by the table that owns them,
 * they appear in a backup, and `save` keeps them. A verb here that wrote them
 * would be a second way to write them, and the two would disagree the first time
 * one of them grew a bound the other did not.
 */
export const enum MissionVerb {
  STATUS = 0,
  START = 1,
  STOP = 2,
  HOME_SET = 3,
  HOME_CLEAR = 4,
}

/** The verbs by the name the firmware's own console uses for each. Used for a
 *  client's error text and for the panel's buttons, so the two cannot call one
 *  verb two things. */
export const MISSION_VERB_NAMES: Readonly<Record<number, string>> = {
  [MissionVerb.STATUS]: 'status',
  [MissionVerb.START]: 'start',
  [MissionVerb.STOP]: 'stop',
  [MissionVerb.HOME_SET]: 'home set',
  [MissionVerb.HOME_CLEAR]: 'home clear',
};

/**
 * `MISSION`'s status byte, from `AK_PROTO_MISSION_*`.
 *
 * Four refusals, and each is a fact rather than an opinion about intent.
 * `NO_NAV` is the odd one out and it is kept apart from `NO_WAYPOINTS` for the
 * same reason `PreflightStatus.NONE` is kept apart from a checklist of zero
 * lines: it is a statement about the *build* — there is no navigation module at
 * all — and a client that drew it as an empty list would send somebody looking
 * for waypoints to add when the thing missing is a module.
 *
 * **Neither is `NO_WAYPOINTS` an error.** It is the aircraft's answer about the
 * list it holds, and it is the answer `start` gives on a board whose table says
 * `wp_count` is zero.
 */
export const enum MissionStatus {
  OK = 0,
  /**
   * Refused: the frame named no verb at all, or one this build does not have.
   *
   * The reply **echoes** the offending verb rather than correcting it, so a
   * reader can tell the two apart: `op` of `0xFF` is "the frame did not say",
   * any other out-of-range value is "you asked for a verb this firmware does not
   * implement".
   */
  NO_VERB = 1,
  /** Refused: this board has no navigator at all. A statement about the build. */
  NO_NAV = 2,
  /** Refused: there is nothing to fly. A statement about the parameter table. */
  NO_WAYPOINTS = 3,
  /** Refused: home cannot be set from a position the navigator would not use. */
  NO_FIX = 4,
}

/**
 * What each status means, in the firmware's own terms.
 *
 * `OK` is "done" rather than "accepted": every verb answers with the aircraft's
 * whole state, so the acknowledgement and the reading arrive together and the
 * panel draws the state rather than this word. The four refusals each name what
 * did not happen, because "failed" would report a policy the aircraft made as a
 * fault — the same rule `OUTPUT_TEST_STATUS_TEXT` follows.
 */
export const MISSION_STATUS_TEXT: Readonly<Record<MissionStatus, string>> = {
  [MissionStatus.OK]: 'done',
  [MissionStatus.NO_VERB]: 'refused: the request named no verb this build has',
  [MissionStatus.NO_NAV]: 'refused: this board has no navigator',
  [MissionStatus.NO_WAYPOINTS]: 'refused: there is nothing to fly',
  [MissionStatus.NO_FIX]: 'refused: home needs a usable position fix',
};

/**
 * `AK_PROTO_MISSION_NO_INDEX` — the `index` byte when the navigator is flying no
 * waypoint.
 *
 * Deliberately out of range for a waypoint list: "not flying a waypoint" and
 * "flying waypoint zero" are the two ends of a mission, and a client that drew
 * them the same way would mark the first waypoint as the one in progress.
 */
export const MISSION_NO_INDEX = 0xff;
