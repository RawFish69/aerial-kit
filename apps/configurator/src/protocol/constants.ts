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

/** `AK_PROTO_TELEMETRY_MAX_HZ`. Asking for more is answered, not refused. */
export const TELEMETRY_MAX_HZ = 50;

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
  [LogSource.FLASH]: 'the log in flash, 5 Hz for about an hour, survives the battery',
};
