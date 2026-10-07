import {
  BATTERY_STATES,
  CALIBRATE_ALL_FACES,
  CALIBRATE_NO_SESSION,
  CALIBRATE_NO_STEP,
  CALIBRATE_RESULT,
  CALIBRATE_RESULT_NAMES,
  CALIBRATE_VERB_NAMES,
  CalibrateStatus,
  CalibrateVerb,
  IMU_ABSENT,
  InfoStatus,
  LogSource,
  LogStreamStatus,
  MISSION_NO_INDEX,
  MISSION_VERB_NAMES,
  MOTOR_ENTRY_BYTES,
  MOTOR_FLAGS,
  MOTOR_MAX,
  PERF_SECTIONS,
  PerfStatus,
  MotorFlag,
  MotorStatus,
  MissionStatus,
  MissionVerb,
  OUTPUT_ENTRY_BYTES,
  OUTPUT_KIND_NAMES,
  OUTPUT_TEST_STATUS_TEXT,
  OUTPUT_TEST_MAX_MS,
  OUTPUT_TEST_MAX_PCT,
  OutputInfoStatus,
  OutputKind,
  OutputTestOp,
  OutputTestStatus,
  PARAM_GROUPS,
  PREFLIGHT_NAME_MAX,
  PreflightStatus,
  PreflightVerdict,
  PARAM_TYPE_NAMES,
  ParamType,
  RC_MAX_CHANNELS,
  RC_PROTOCOLS,
  RC_STICKS,
  RcStatus,
  SENSOR_BODY_LENGTH,
  SENSOR_NAME_LENGTH,
  SENSOR_TOPICS_ORDERED,
  Sensor,
  SensorStatus,
  SetStatus,
  UNKNOWN_COMMAND,
} from './constants';

/**
 * Payload decoders, one per reply, written against the layout in
 * `firmware/docs/16-protocol.md` and cross-checked against the firmware's own
 * `tools/akproto.py` in the tests.
 *
 * Two rules run through all of them:
 *
 *  - **A short payload is an error, never a default.** A zero read out of a
 *    truncated frame is a plausible-looking wrong answer, and a configurator
 *    that shows "latitude 0" for "the board did not say" is a configurator that
 *    puts an aircraft in the Atlantic.
 *  - **Absent is not zero.** Where the protocol has a way to say "this device
 *    does not have that" — a log source, for instance — it is carried through
 *    as null rather than collapsed into an empty result.
 */
export class ProtocolError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'ProtocolError';
  }
}

class Reader {
  private at = 0;
  constructor(private readonly data: Uint8Array) {}

  get remaining(): number {
    return this.data.length - this.at;
  }

  need(count: number, what: string): void {
    if (this.remaining < count) {
      throw new ProtocolError(
        `${what}: needed ${count} bytes, the reply carried ${this.remaining}`,
      );
    }
  }

  u8(what: string): number {
    this.need(1, what);
    return this.data[this.at++]!;
  }

  u16(what: string): number {
    this.need(2, what);
    const value = this.data[this.at]! | (this.data[this.at + 1]! << 8);
    this.at += 2;
    return value;
  }

  i16(what: string): number {
    const value = this.u16(what);
    return value & 0x8000 ? value - 0x10000 : value;
  }

  u32(what: string): number {
    this.need(4, what);
    const value =
      (this.data[this.at]! |
        (this.data[this.at + 1]! << 8) |
        (this.data[this.at + 2]! << 16) |
        (this.data[this.at + 3]! << 24)) >>>
      0;
    this.at += 4;
    return value;
  }

  i32(what: string): number {
    const value = this.u32(what);
    return value > 0x7fffffff ? value - 0x100000000 : value;
  }

  /** A NUL-terminated string. The terminator is required: a reply that runs out
   *  of bytes mid-name is a truncated frame, not a short name. */
  cstring(what: string): string {
    const end = this.data.indexOf(0, this.at);
    if (end < 0) throw new ProtocolError(`${what}: no terminator in the reply`);
    const text = new TextDecoder().decode(this.data.subarray(this.at, end));
    this.at = end + 1;
    return text;
  }

  bytes(count: number, what: string): Uint8Array {
    this.need(count, what);
    const out = this.data.slice(this.at, this.at + count);
    this.at += count;
    return out;
  }

  /**
   * Exactly `count` bytes, read as text.
   *
   * Not `cstring` and not a variant of it: a length-prefixed string carries no
   * terminator, so looking for one would read past the field into whatever
   * follows it — the sentence's length, in `preflight`'s case, which is a byte
   * that is often zero and would make a name look one byte longer than it is.
   * The two are different wire shapes and neither is a special case of the
   * other.
   */
  ascii(count: number, what: string): string {
    return new TextDecoder().decode(this.bytes(count, what));
  }
}

export interface Hello {
  /** The protocol version the *board* answered with, which may not be ours. */
  readonly protocolVersion: number;
  readonly product: string;
  readonly parameterCount: number;
  /** How many parameters differ from what is saved in flash, as text. */
  readonly changedSinceSaved: string;
  /**
   * The board's `AK_PROTO_FEATURE_*` word, or **null when the reply ended
   * before the field**, which means this firmware predates it.
   *
   * Null is not zero. Zero says "this board has none of the optional
   * commands"; null says "this board cannot tell you". Both are useful and
   * only one of them is about the board's abilities, so they are kept apart
   * here and everywhere downstream.
   */
  readonly features: number | null;
  /** `ak_params_hash` of the board's table, or null when the field is absent.
   *  Two boards reporting the same hash hold the same parameter table, which
   *  is what makes a backup from one meaningful on the other. */
  readonly configHash: number | null;
}

export function parseHello(payload: Uint8Array): Hello {
  const reader = new Reader(payload);
  const protocolVersion = reader.u8('hello: protocol version');
  const product = reader.cstring('hello: product name');
  const parameterCount = reader.u16('hello: parameter count');
  const changedSinceSaved = reader.cstring('hello: changed count');

  // The two fields below were appended to a reply that used to end here, which
  // is what let them be added without moving the protocol version. So: no
  // trailing bytes is an old board and gets two nulls, and a *partial* pair is
  // a truncated frame rather than a board that half-supports something.
  if (reader.remaining === 0) {
    return { protocolVersion, product, parameterCount, changedSinceSaved, features: null, configHash: null };
  }
  if (reader.remaining < 8) {
    throw new ProtocolError(
      `hello: the capability word and hash are 8 bytes together, and the reply carried ${reader.remaining}`,
    );
  }
  const features = reader.u32('hello: features');
  const configHash = reader.u32('hello: config hash');
  return { protocolVersion, product, parameterCount, changedSinceSaved, features, configHash };
}

export interface ParameterValue {
  readonly index: number;
  readonly name: string;
  /** The value exactly as the board printed it. The protocol has no typed
   *  fields, so this is a string here too — parsing it into a number would be
   *  this app inventing a type the board never stated. */
  readonly value: string;
}

/** `PARAM_GET`'s reply: status, then the name and value when it succeeded. */
export function parseParamGet(payload: Uint8Array, index: number): ParameterValue | null {
  const reader = new Reader(payload);
  const status = reader.u8('param get: status');
  if (status !== 0) return null;
  const name = reader.cstring('param get: name');
  const value = reader.cstring('param get: value');
  return { index, name, value };
}

export interface ParamSetReply {
  readonly status: SetStatus;
  /** The parameter table's own words — "out of range 0.000..3.000" — or empty.
   *  This is the *only* place the protocol ever states a range. */
  readonly message: string;
}

/** `status, then the board's own words`. Shared by the three commands that
 *  answer this way — set, save and default — because they are one shape, and
 *  three copies of it would be three places for the trailing-message rule to
 *  drift. `what` only names the command in an error. */
function parseStatusMessage(payload: Uint8Array, what: string): ParamSetReply {
  const reader = new Reader(payload);
  const status = reader.u8(`${what}: status`) as SetStatus;
  // An older board sends nothing after the status. That reads as an empty
  // message, which is the honest answer rather than an error.
  const message = reader.remaining > 0 ? reader.cstring(`${what}: message`) : '';
  return { status, message };
}

export function parseParamSet(payload: Uint8Array): ParamSetReply {
  return parseStatusMessage(payload, 'param set');
}

/**
 * `PARAM_DEFAULT`'s reply: the same `status, message` pair as a set, and the
 * same type, because it is the same answer to a different question.
 *
 * What is *not* shared is the reading of it. A status of `VALUE_REFUSED` here
 * carries sentences a person has to act on — "name what to reset: 1 <index>, or
 * 2 for all" — so the caller must show the message rather than only the status
 * name. `setStatusName(VALUE_REFUSED)` is "value refused", which for this
 * command would be a true sentence that tells nobody anything.
 */
export function parseParamDefault(payload: Uint8Array): ParamSetReply {
  return parseStatusMessage(payload, 'param default');
}

/**
 * What the receiver is hearing, from `rc channels`.
 *
 * **The sticks are the firmware's.** `rc_min`, `rc_mid`, `rc_max` and
 * `rc_deadband` are parameters, and this app decoding the raw counts itself
 * would be a second implementation of the firmware's `centred()` in another
 * language — disagreeing at the deadband edge, with the screen and the airframe
 * each believing its own. The counts are carried here as evidence (a count out
 * of range is the first sign of a receiver on the wrong baud rate) and never as
 * an input.
 */
export interface RcState {
  readonly status: RcStatus;
  readonly flags: number;
  /** The wire's number, kept as well as the name, so a protocol this app does
   *  not know renders as unknown rather than as nothing. */
  readonly protocol: number;
  /** `null` when this app has no name for that number. */
  readonly protocolName: string | null;
  readonly channels: readonly number[];
  /**
   * Per-mille, in `RC_STICKS` order. **Zeroed when `RcFlag.DECODED` is clear**,
   * which is why the flag and not these numbers is what a caller reads first:
   * all-zero sticks with the flag set are a centred handset, and all-zero sticks
   * with it clear are a board that has no idea where the sticks are.
   */
  readonly sticks: readonly number[];
  readonly switches: number;
  readonly bytes: number;
  readonly frames: number;
  /** CRSF only: an SBUS frame has no CRC to fail, so this is 0 there. */
  readonly crcErrors: number;
  readonly rejected: number;
  /** SBUS only: frames that arrived flagged with lost frames. */
  readonly lost: number;
  /** SBUS only: frames that arrived with the receiver's own failsafe set. */
  readonly failsafeFrames: number;
  /** Bytes the UART receive buffer dropped — the board's number, not the
   *  receiver's, and the one counter only the port knows. */
  readonly dropped: number;
}

const NO_RECEIVER: Omit<RcState, 'status'> = {
  flags: 0,
  protocol: 0,
  protocolName: null,
  channels: [],
  sticks: [],
  switches: 0,
  bytes: 0,
  frames: 0,
  crcErrors: 0,
  rejected: 0,
  lost: 0,
  failsafeFrames: 0,
  dropped: 0,
};

export function parseRcChannels(payload: Uint8Array): RcState {
  const reader = new Reader(payload);
  // Read as the raw byte rather than as `RcStatus`: 0x7F is not a member of that
  // enum, and the *point* of the check below is a value the firmware's own
  // `RcStatus` cannot express. Casting first would make the comparison a type
  // error, which is the type system correctly saying these are two different
  // kinds of answer.
  const raw = reader.u8('rc channels: status');

  if (raw === UNKNOWN_COMMAND) {
    // A board that predates this command still answers — with 0x7F, correlated
    // to the request. It gets its own sentence rather than falling into the
    // `!== OK` branch below, and the difference is not pedantry: that branch
    // means "this board has no receiver input", so a build without the command
    // would be reported to a person as a *hardware* fact about a board that may
    // have a receiver plugged into it right now. Same rule as `param info`, and
    // the same reason.
    throw new ProtocolError(
      'rc channels: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `rc channels`',
    );
  }

  const status = raw as RcStatus;
  if (status !== RcStatus.OK) {
    // One byte and nothing else. Every field below would be a claim about a
    // receiver this board does not have, and zeros would read as a receiver
    // with a dead link rather than as no receiver port at all.
    return { status, ...NO_RECEIVER };
  }

  const flags = reader.u8('rc channels: flags');
  const protocol = reader.u8('rc channels: protocol');
  const count = reader.u8('rc channels: count');
  // The firmware clamps this to what it actually wrote. A count past the
  // protocol's bound is a frame that has gone wrong somewhere, and reading on
  // would take the sticks for channels.
  if (count > RC_MAX_CHANNELS) {
    throw new ProtocolError(
      `rc channels: the reply says ${count} channels, and the protocol allows at most ${RC_MAX_CHANNELS}`,
    );
  }

  const channels: number[] = [];
  for (let i = 0; i < count; i += 1) {
    channels.push(reader.u16(`rc channels: channel ${i} (of ${count})`));
  }

  const sticks: number[] = [];
  for (const name of RC_STICKS) {
    sticks.push(reader.i16(`rc channels: ${name}`));
  }

  const switches = reader.u8('rc channels: switches');
  // The seven counters. Named as the receiver names them, so nothing here is a
  // translation that could go wrong.
  const bytes = reader.u32('rc channels: bytes');
  const frames = reader.u32('rc channels: frames');
  const crcErrors = reader.u32('rc channels: crc errors');
  const rejected = reader.u32('rc channels: rejected');
  const lost = reader.u32('rc channels: lost');
  const failsafeFrames = reader.u32('rc channels: failsafe frames');
  const dropped = reader.u32('rc channels: dropped');

  return {
    status,
    flags,
    protocol,
    protocolName: RC_PROTOCOLS[protocol] ?? null,
    channels,
    sticks,
    switches,
    bytes,
    frames,
    crcErrors,
    rejected,
    lost,
    failsafeFrames,
    dropped,
  };
}

/**
 * How fast one motor is turning, from `motor telemetry`.
 *
 * Every field beside the flags is `null` when its flag is clear, and that is
 * the whole point: "a temperature of zero" and "no temperature has been heard"
 * are the same byte otherwise, and a screen that drew them the same way would
 * report an ESC that is cold when the board has never heard from it at all.
 *
 * `rpm` is `null` unless `MotorFlag.RPM` is set, which is only when the board
 * measured this motor *and* was told its pole count. **This app does not
 * compute it**, for the reason the receiver's sticks are not recomputed here:
 * `erpm / (poles / 2)` looks like arithmetic that cannot go wrong, and the one
 * number it depends on is the one no board in this tree knows, so a plausible
 * divide-by-fourteen would be a figure this app invented and labelled as the
 * motor's speed.
 */
export interface MotorReading {
  readonly index: number;
  readonly flags: number;
  readonly flagNames: readonly string[];
  readonly measured: boolean;
  readonly erpm: number;
  readonly rpm: number | null;
  readonly temperature: number | null;
  readonly maxTemperature: number | null;
  readonly millivolts: number | null;
  readonly milliamps: number | null;
  /** The quality window's two counts. Zero packets is a true answer and not an
   *  absence: the window is a window, and `measured` is how a client tells
   *  "nothing has ever been heard" from "nothing has been heard lately". */
  readonly packets: number;
  readonly invalid: number;
}

/**
 * What the board can hear from its ESCs, from `motor telemetry`.
 *
 * **Three answers that collapse to the same row of zeroes on a screen**, and
 * the status and the flags are what keep them apart — the discipline
 * `RcFlag.DECODED` and `SENSOR_INFO`'s `present` byte already follow:
 *
 *  - `status: NONE` — this build has no telemetry path. A fact about the
 *    firmware, and `motors` is empty rather than a list of zeroes.
 *  - `status: OK` with every `measured` clear — the board has the path and has
 *    heard nothing yet. A fact about the aircraft, and the thing to do about it
 *    is look at the ESC's signal wire.
 *  - `status: OK` with `measured` on a motor whose `erpm` is 0 — an ESC that
 *    answered and said it is not turning. The only one of the three that is a
 *    reading of a motor at rest.
 */
export interface MotorTelemetryState {
  readonly status: MotorStatus;
  readonly count: number;
  /** The pole count the board divided `rpm` by. `0` means it does not know,
   *  and then no motor's `rpm` flag is set. */
  readonly poles: number;
  readonly motors: readonly MotorReading[];
}

const NO_TELEMETRY: Omit<MotorTelemetryState, 'status'> = {
  count: 0,
  poles: 0,
  motors: [],
};

export function parseMotorTelemetry(payload: Uint8Array): MotorTelemetryState {
  const reader = new Reader(payload);
  const raw = reader.u8('motor telemetry: status');

  if (raw === UNKNOWN_COMMAND) {
    throw new ProtocolError(
      'motor telemetry: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `motor telemetry`',
    );
  }
  if (raw !== MotorStatus.OK && raw !== MotorStatus.NONE) {
    throw new ProtocolError(
      `motor telemetry: the board answered with status ${raw}, which is not one of the two ` +
        'the firmware defines',
    );
  }

  const status = raw as MotorStatus;
  if (status !== MotorStatus.OK) {
    /* One byte and no more, which is the firmware's own rule. Anything after it
     * would be a claim about hardware this board does not have, and reading it
     * would be this app believing a stream of bytes it has decided to believe
     * nothing about. */
    if (reader.remaining !== 0) {
      throw new ProtocolError(
        `motor telemetry: a board with no telemetry path answers one byte, and this reply ` +
          `carried ${payload.length}`,
      );
    }
    return { status, ...NO_TELEMETRY };
  }

  const count = reader.u8('motor telemetry: count');
  if (count > MOTOR_MAX) {
    throw new ProtocolError(
      `motor telemetry: the reply says ${count} motors, and the protocol allows at most ` +
        `${MOTOR_MAX}`,
    );
  }
  const poles = reader.u8('motor telemetry: poles');

  const motors: MotorReading[] = [];
  for (let index = 0; index < count; index += 1) {
    const at = `motor telemetry: motor ${index} (of ${count})`;
    const flags = reader.u8(`${at} flags`);
    const erpm = reader.u32(`${at} eRPM`);
    const rpm = reader.u32(`${at} rpm`);
    const temperature = reader.u8(`${at} temperature`);
    const maxTemperature = reader.u8(`${at} session maximum`);
    const millivolts = reader.u16(`${at} millivolts`);
    const milliamps = reader.u16(`${at} milliamps`);
    const packets = reader.u16(`${at} packets`);
    const invalid = reader.u16(`${at} invalid`);

    motors.push({
      index,
      flags,
      flagNames: MOTOR_FLAGS.filter((entry) => (flags & entry.bit) !== 0).map(
        (entry) => entry.name,
      ),
      measured: (flags & MotorFlag.MEASURED) !== 0,
      erpm,
      rpm: flags & MotorFlag.RPM ? rpm : null,
      temperature: flags & MotorFlag.TEMPERATURE ? temperature : null,
      maxTemperature: flags & MotorFlag.TEMPERATURE ? maxTemperature : null,
      millivolts: flags & MotorFlag.VOLTAGE ? millivolts : null,
      milliamps: flags & MotorFlag.CURRENT ? milliamps : null,
      packets,
      invalid,
    });
  }

  if (reader.remaining !== 0) {
    throw new ProtocolError(
      `motor telemetry: ${count} motor(s) take ${payload.length - reader.remaining} bytes, ` +
        `and the reply carried ${payload.length}`,
    );
  }
  return { status, count, poles, motors };
}

/**
 * The loop profiler's window, from `PERF`, in the units the wire carries:
 * section averages in tenths of a microsecond, maxima in whole microseconds,
 * load in per-mille of the nominal slot (a lower bound — not every task is
 * instrumented). `status: NONE` is a build without a profiler: the firmware
 * still sends the whole window, zeroed, and those zeros are not readings.
 */
export interface PerfWindow {
  readonly status: PerfStatus;
  readonly loops: number;
  readonly samples: number;
  readonly nominalUs: number;
  readonly periodLastUs: number;
  readonly periodMinUs: number;
  readonly periodMaxUs: number;
  readonly late: number;
  readonly jitterP50Us: number;
  readonly jitterP99Us: number;
  readonly jitterMaxUs: number;
  readonly jitterOver: number;
  readonly sectionAvgX10: readonly number[];
  readonly sectionMaxUs: readonly number[];
  readonly loadPermille: number;
}

export function parsePerf(payload: Uint8Array): PerfWindow {
  const reader = new Reader(payload);
  const raw = reader.u8('perf: status');
  if (raw === UNKNOWN_COMMAND) {
    throw new ProtocolError('perf: the board answered 0x7F — this build predates `perf`');
  }
  if (raw !== PerfStatus.OK && raw !== PerfStatus.NONE) {
    throw new ProtocolError(`perf: the board answered with status ${raw}, which the firmware does not define`);
  }
  const window: PerfWindow = {
    status: raw as PerfStatus,
    loops: reader.u32('perf: loops'),
    samples: reader.u32('perf: samples'),
    nominalUs: reader.u16('perf: nominal period'),
    periodLastUs: reader.u32('perf: last period'),
    periodMinUs: reader.u32('perf: minimum period'),
    periodMaxUs: reader.u32('perf: maximum period'),
    late: reader.u32('perf: late periods'),
    jitterP50Us: reader.u16('perf: jitter p50'),
    jitterP99Us: reader.u16('perf: jitter p99'),
    jitterMaxUs: reader.u16('perf: jitter max'),
    jitterOver: reader.u32('perf: jitter over budget'),
    sectionAvgX10: Array.from({ length: PERF_SECTIONS }, (_, i) => reader.u16(`perf: section ${i} average`)),
    sectionMaxUs: Array.from({ length: PERF_SECTIONS }, (_, i) => reader.u16(`perf: section ${i} maximum`)),
    loadPermille: reader.u16('perf: load'),
  };
  if (reader.remaining !== 0) {
    throw new ProtocolError(`perf: the window is ${payload.length - reader.remaining} bytes, and the reply carried ${payload.length}`);
  }
  return window;
}

/**
 * What the board is *hearing* rather than what it is doing, from
 * `sensor info`.
 *
 * **Three answers, and a screen that shows them the same way is wrong about
 * two of them:**
 *
 *  - `status: NO_SUCH` — this build does not answer for that topic at all.
 *    A fact about the firmware, and the thing to do about it is update it.
 *  - `status: OK`, `present: false` — the build knows the question and this
 *    board has nothing fitted. A fact about the aircraft, and the thing to do
 *    about it is look at the socket.
 *  - `status: OK`, `present: true` — a reading, and a zero in it is a zero.
 *
 * The wire keeps them apart by leaving the body *off* rather than sending one
 * of zeros, because a body of zeros is a sensor reading zero pressure, which is
 * a sensor that has failed. That is the same rule `RcFlag.DECODED` follows, and
 * for the same reason: one set of bytes, two meanings.
 */
export interface SensorAnswer {
  readonly status: SensorStatus;
  readonly topic: number;
  /** `null` when this app has no name for that topic number. */
  readonly topicName: string | null;
  readonly present: boolean;
  /** `null` on both of the first two answers, and never a body of zeros. */
  readonly body: SensorBody | null;
}

/** `imu`: which part, how it is aligned, and what it last read. */
export interface ImuBody {
  readonly topic: Sensor.IMU;
  readonly driver: string;
  /** `AK_PROTO_IMU_ABSENT_*`. Only meaningful when this driver is fitted but
   *  would not come up — a board with no IMU at all never gets this far. */
  readonly absent: number;
  readonly absentReason: string | null;
  readonly whoami: number;
  /** Per-mille of g, which is the unit the console prints and the log stores. */
  readonly accel: readonly number[];
  /** Milliradians per second. */
  readonly gyro: readonly number[];
  /** Degrees, from the `align_*_deg` parameters. */
  readonly align: readonly number[];
  /** Milli-degrees per second, from the gyro calibration. */
  readonly gyroBias: readonly number[];
  readonly samples: number;
  readonly errors: number;
}

/** `baro`: pressure, and the height above where the aircraft was standing.
 *
 *  The reference is the whole reason to carry a barometer — the absolute
 *  pressure is today's weather and the *change* since take-off is the altitude
 *  — so `haveReference` is carried beside the number rather than folded into
 *  it. Without a reference the height is meaningless rather than zero. */
export interface BaroBody {
  readonly topic: Sensor.BARO;
  readonly driver: string;
  readonly pressurePa: number;
  readonly temperatureC: number;
  readonly haveReference: boolean;
  readonly referencePa: number;
  readonly heightCm: number;
  readonly haveGpsReference: boolean;
  readonly fusedCm: number;
  readonly samples: number;
  readonly errors: number;
  readonly fails: number;
  readonly baroSamples: number;
  readonly gpsSamples: number;
}

/** `range`: how far the ground is, and the counters that say whether to
 *  believe it. `distanceMm` is negative for "nothing in range" — a wall against
 *  the lens is 0, and the two must not be drawn the same way. */
export interface RangeBody {
  readonly topic: Sensor.RANGE;
  readonly driver: string;
  readonly address: number;
  readonly maxMm: number;
  readonly distanceMm: number;
  readonly ageMs: number;
  readonly samples: number;
  readonly outOfRange: number;
  readonly rejected: number;
  readonly faults: number;
  readonly fails: number;
  readonly landMm: number;
  readonly agreeCm: number;
}

/** `battery`: the pack, and the only number that decides anything — volts a
 *  cell. `pinMv` is negative for "the board had no reading", which is a
 *  different thing from 0 mV at the pin. */
export interface BatteryBody {
  readonly topic: Sensor.BATTERY;
  readonly ready: boolean;
  readonly haveReading: boolean;
  readonly state: number;
  readonly stateName: string | null;
  readonly cells: number;
  readonly volts: number;
  readonly voltsPerCell: number;
  readonly pinMv: number;
  readonly ratio: number;
  readonly rth: boolean;
  readonly warnCellV: number;
  readonly criticalCellV: number;
  readonly samples: number;
  readonly rejected: number;
  readonly returns: number;
}

/** `gps`: the fix, where it is, and the way home. Read `haveFix` before the
 *  position: 0, 0 with no fix is the Gulf of Guinea, and a map that plotted it
 *  would be putting an aircraft there. */
export interface GpsBody {
  readonly topic: Sensor.GPS;
  readonly haveFix: boolean;
  readonly fixType: number;
  readonly fixOk: boolean;
  readonly satellites: number;
  readonly validNow: boolean;
  readonly lat: number;
  readonly lon: number;
  readonly altMslMm: number;
  readonly speedMmS: number;
  readonly courseDeg: number;
  readonly haveHome: boolean;
  readonly homeLat: number;
  readonly homeLon: number;
  /** Metres, or −1 when there is nowhere to measure to. */
  readonly homeDistanceM: number;
  readonly homeBearingDeg: number;
  readonly returning: boolean;
  readonly rthEnabled: boolean;
  readonly fixes: number;
  readonly dropped: number;
  readonly configSends: number;
}

export type SensorBody = ImuBody | BaroBody | RangeBody | BatteryBody | GpsBody;

/** A body's driver name: `SENSOR_NAME_LENGTH` bytes, NUL-padded, and **not**
 *  required to carry a terminator** — a name that exactly fills the field is a
 *  legal name. `Reader.cstring` requires one, so reading a name with it would
 *  refuse the one part whose name happens to be twelve characters long. */
function fixedName(reader: Reader, what: string): string {
  const raw = reader.bytes(SENSOR_NAME_LENGTH, what);
  const end = raw.indexOf(0);
  const name = raw.subarray(0, end === -1 ? raw.length : end);
  return new TextDecoder().decode(name);
}

function sensorBody(reader: Reader, topic: Sensor): SensorBody {
  switch (topic) {
    case Sensor.IMU: {
      const driver = fixedName(reader, 'sensor info: imu driver');
      const absent = reader.u8('sensor info: imu absent reason');
      return {
        topic,
        driver,
        absent,
        absentReason: IMU_ABSENT[absent] ?? null,
        whoami: reader.u8('sensor info: imu whoami'),
        accel: [reader.i16('sensor info: imu accel x'), reader.i16('sensor info: imu accel y'), reader.i16('sensor info: imu accel z')],
        gyro: [reader.i16('sensor info: imu gyro x'), reader.i16('sensor info: imu gyro y'), reader.i16('sensor info: imu gyro z')],
        align: [reader.i16('sensor info: imu align roll'), reader.i16('sensor info: imu align pitch'), reader.i16('sensor info: imu align yaw')],
        gyroBias: [reader.i16('sensor info: imu bias x'), reader.i16('sensor info: imu bias y'), reader.i16('sensor info: imu bias z')],
        samples: reader.u32('sensor info: imu samples'),
        errors: reader.u32('sensor info: imu errors'),
      };
    }
    case Sensor.BARO: {
      const driver = fixedName(reader, 'sensor info: baro driver');
      return {
        topic,
        driver,
        pressurePa: reader.i32('sensor info: baro pressure'),
        temperatureC: reader.i16('sensor info: baro temperature') / 100,
        haveReference: reader.u8('sensor info: baro have reference') !== 0,
        referencePa: reader.i32('sensor info: baro reference'),
        heightCm: reader.i32('sensor info: baro height'),
        haveGpsReference: reader.u8('sensor info: baro have gps reference') !== 0,
        fusedCm: reader.i32('sensor info: baro fused height'),
        samples: reader.u32('sensor info: baro samples'),
        errors: reader.u32('sensor info: baro errors'),
        fails: reader.u32('sensor info: baro fails'),
        baroSamples: reader.u32('sensor info: baro altitude samples'),
        gpsSamples: reader.u32('sensor info: baro gps samples'),
      };
    }
    case Sensor.RANGE: {
      const driver = fixedName(reader, 'sensor info: range driver');
      const address = reader.u8('sensor info: range address');
      return {
        topic,
        driver,
        address,
        maxMm: reader.u16('sensor info: range max'),
        // Signed, and left signed: negative is "nothing in range".
        distanceMm: reader.i32('sensor info: range distance'),
        ageMs: reader.u32('sensor info: range age'),
        samples: reader.u32('sensor info: range samples'),
        outOfRange: reader.u32('sensor info: range out of range'),
        rejected: reader.u32('sensor info: range rejected'),
        faults: reader.u32('sensor info: range faults'),
        fails: reader.u32('sensor info: range fails'),
        landMm: reader.u32('sensor info: range land'),
        agreeCm: reader.u16('sensor info: range agree'),
      };
    }
    case Sensor.BATTERY: {
      const ready = reader.u8('sensor info: battery ready') !== 0;
      const haveReading = reader.u8('sensor info: battery have reading') !== 0;
      const state = reader.u8('sensor info: battery state');
      return {
        topic,
        ready,
        haveReading,
        state,
        stateName: BATTERY_STATES[state] ?? null,
        cells: reader.u8('sensor info: battery cells'),
        volts: reader.u16('sensor info: battery volts') / 100,
        voltsPerCell: reader.u16('sensor info: battery volts per cell') / 100,
        // Signed: negative is "no reading at the pin", not 0 mV.
        pinMv: reader.i16('sensor info: battery pin'),
        ratio: reader.u16('sensor info: battery ratio') / 1000,
        rth: reader.u8('sensor info: battery rth') !== 0,
        warnCellV: reader.u16('sensor info: battery warn') / 1000,
        criticalCellV: reader.u16('sensor info: battery critical') / 1000,
        samples: reader.u32('sensor info: battery samples'),
        rejected: reader.u32('sensor info: battery rejected'),
        returns: reader.u32('sensor info: battery returns'),
      };
    }
    case Sensor.GPS: {
      const haveFix = reader.u8('sensor info: gps have fix') !== 0;
      const fixType = reader.u8('sensor info: gps fix type');
      const fixOk = reader.u8('sensor info: gps fix ok') !== 0;
      const satellites = reader.u8('sensor info: gps satellites');
      const validNow = reader.u8('sensor info: gps valid now') !== 0;
      const lat = reader.i32('sensor info: gps latitude') / 1e7;
      const lon = reader.i32('sensor info: gps longitude') / 1e7;
      const altMslMm = reader.i32('sensor info: gps altitude');
      const speedMmS = reader.i32('sensor info: gps speed');
      const courseDeg = reader.i32('sensor info: gps course') / 1e5;
      const haveHome = reader.u8('sensor info: gps have home') !== 0;
      const homeLat = reader.i32('sensor info: gps home latitude') / 1e7;
      const homeLon = reader.i32('sensor info: gps home longitude') / 1e7;
      const homeDistanceM = reader.i32('sensor info: gps home distance');
      const homeBearingDeg = reader.i32('sensor info: gps home bearing') / 100;
      return {
        topic,
        haveFix,
        fixType,
        fixOk,
        satellites,
        validNow,
        lat,
        lon,
        altMslMm,
        speedMmS,
        courseDeg,
        haveHome,
        homeLat,
        homeLon,
        homeDistanceM,
        homeBearingDeg,
        returning: reader.u8('sensor info: gps returning') !== 0,
        rthEnabled: reader.u8('sensor info: gps rth enabled') !== 0,
        fixes: reader.u32('sensor info: gps fixes'),
        dropped: reader.u32('sensor info: gps dropped'),
        configSends: reader.u32('sensor info: gps config sends'),
      };
    }
    default:
      // Unreachable: `parseSensorInfo` checks the topic against
      // `SENSOR_BODY_LENGTH` before calling. Thrown rather than returned as a
      // made-up body, because the alternative to "no such topic" is inventing
      // a reading for it.
      throw new ProtocolError(`sensor info: no body is defined for topic ${topic}`);
  }
}

export function parseSensorInfo(payload: Uint8Array): SensorAnswer {
  const reader = new Reader(payload);
  // Read as the raw byte rather than as `SensorStatus`: 0x7F is not a member of
  // that enum, and the *point* of the check below is a value the firmware's own
  // status enum cannot express. Same rule as `rc channels`.
  const raw = reader.u8('sensor info: status');

  if (raw === UNKNOWN_COMMAND) {
    // A board that predates this command still answers, with 0x7F. It gets its
    // own sentence rather than falling into the `NO_SUCH` branch, and the
    // difference is not pedantry: `NO_SUCH` is "this build knows the question
    // and has no sensor there", so a build without the command would be
    // reported to a person as a *hardware* fact about a board that may have the
    // part soldered in right now.
    throw new ProtocolError(
      'sensor info: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `sensor info`',
    );
  }

  const status = raw as SensorStatus;
  if (status !== SensorStatus.OK && status !== SensorStatus.NO_SUCH) {
    throw new ProtocolError(`sensor info: status ${status} is not one this protocol defines`);
  }

  const topic = reader.u8('sensor info: topic');
  const topicName = SENSOR_TOPICS_ORDERED.find((entry) => entry.topic === topic)?.name ?? null;
  const present = reader.u8('sensor info: present') !== 0;

  if (status === SensorStatus.NO_SUCH) {
    // Three bytes and no more. A tail here is a frame this app does not
    // understand rather than an answer with extra in it.
    if (reader.remaining !== 0) {
      throw new ProtocolError(
        `sensor info: a refused topic answers three bytes, and this reply carried ${reader.remaining} more`,
      );
    }
    return { status, topic, topicName, present: false, body: null };
  }

  if (!present) {
    // **The absent case, and the reason the opcode is shaped this way.** A body
    // of zeros would say "this board has a barometer and it reads zero
    // pressure", which is a failed sensor rather than a missing one.
    if (reader.remaining !== 0) {
      throw new ProtocolError(
        `sensor info: an absent sensor carries no body — that is what tells it from a ` +
          `sensor reading zero — and this reply has ${reader.remaining} bytes after the ` +
          `topic and present`,
      );
    }
    return { status, topic, topicName, present: false, body: null };
  }

  const expected = SENSOR_BODY_LENGTH[topic as Sensor];
  if (expected === undefined) {
    throw new ProtocolError(`sensor info: no body is defined for topic ${topic}`);
  }
  if (reader.remaining !== expected) {
    throw new ProtocolError(
      `sensor info: topic ${topic} carries a ${expected}-byte body and this one is ` +
        `${reader.remaining} — a client that read on would take the next field's bytes ` +
        `for this one's`,
    );
  }
  const body = sensorBody(reader, topic as Sensor);
  if (reader.remaining !== 0) {
    throw new ProtocolError(`sensor info: ${reader.remaining} bytes left over after the body`);
  }
  return { status, topic, topicName, present: true, body };
}

/**
 * One pad the board drives, from `output info`.
 *
 * The three linkage fields are on the wire for every entry — the firmware
 * copies them from a struct field by field, so a motor's are written as zeros
 * rather than left out. They are `null` here for a motor, because a motor has
 * no linkage to reverse and no neutral to trim, and a screen that drew a
 * neutral for one would be drawing a fact about a servo it does not have.
 */
export interface OutputDescriptor {
  readonly kind: number;
  /** `null` when this app has no name for that kind number. */
  readonly kindName: string | null;
  /** This output's number *within its own kind*, 0-based — the number an
   *  `output test` names, and not an index into this list. */
  readonly index: number;
  /** Servos only: the linkage moves the other way from the stick. */
  readonly reversed: boolean;
  /** Servos only: microseconds added to the centre, signed. */
  readonly trimUs: number | null;
  /** Servos only: microseconds of travel at full stick. */
  readonly travelUs: number | null;
}

/**
 * What the board drives, from `output info`.
 *
 * **Three answers, and two of them would draw the same blank panel:**
 *
 *  - `status: NONE` — there is no list. Either this board has nothing to drive
 *    or it is not ready to say; both are "no outputs", and neither is an
 *    aircraft with zero motors. The firmware answers this rather than a
 *    well-formed page with a count of zero, because that page would be a
 *    sentence saying "this aircraft has no motors" and no board that flies can
 *    say it.
 *  - `status: TOO_MANY` — the board has more outputs than one frame carries, so
 *    it refused rather than paged. A short list drawn as the whole aircraft is a
 *    client missing a servo it would then go looking for in the wiring.
 *  - `status: OK` — here is the list, and `outputs.length` is its count.
 *
 * `capPct` comes back on every path, including the two that carry no
 * descriptors. It is the *firmware's* ceiling on an output test rather than
 * anything about this board, and a client that only learned it from a board
 * with outputs would have no ceiling to draw on a board without.
 */
export interface OutputList {
  readonly status: OutputInfoStatus;
  readonly count: number;
  readonly motors: number;
  readonly servos: number;
  readonly capPct: number;
  readonly outputs: readonly OutputDescriptor[];
}

export function parseOutputInfo(payload: Uint8Array): OutputList {
  const reader = new Reader(payload);
  // Raw, not cast: 0x7F is not a member of the status enum and the point of the
  // check is a value that enum cannot express. Same rule as `rc channels`.
  const raw = reader.u8('output info: status');

  if (raw === UNKNOWN_COMMAND) {
    throw new ProtocolError(
      'output info: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `output info`',
    );
  }

  const status = raw as OutputInfoStatus;
  if (
    status !== OutputInfoStatus.OK &&
    status !== OutputInfoStatus.NONE &&
    status !== OutputInfoStatus.TOO_MANY
  ) {
    throw new ProtocolError(`output info: status ${status} is not one this protocol defines`);
  }

  const count = reader.u8('output info: count');
  const motors = reader.u8('output info: motors');
  const servos = reader.u8('output info: servos');
  const capPct = reader.u8('output info: cap');

  if (status !== OutputInfoStatus.OK) {
    // No list at all, so a body behind this header is a frame this app does not
    // understand rather than an answer with extra in it. And a count on either
    // of these two would be a list the status says is not there.
    if (reader.remaining !== 0) {
      throw new ProtocolError(
        `output info: status ${status} carries no list, and this reply has ` +
          `${reader.remaining} bytes behind the header`,
      );
    }
    if (count !== 0 || motors !== 0 || servos !== 0) {
      throw new ProtocolError(
        `output info: status ${status} carries no list, and this reply claims ${count} of them`,
      );
    }
    return { status, count: 0, motors: 0, servos: 0, capPct, outputs: [] };
  }

  const expected = count * OUTPUT_ENTRY_BYTES;
  if (reader.remaining !== expected) {
    throw new ProtocolError(
      `output info: ${count} outputs means ${expected} bytes of descriptors and this reply ` +
        `has ${reader.remaining} — reading on would take the next entry's kind byte for ` +
        `this one's index`,
    );
  }
  if (motors + servos !== count) {
    // The split is what lets a screen say "4 motors, 2 servos" without counting
    // descriptors, so a split that does not add up means one of the two numbers
    // was written from something other than the list.
    throw new ProtocolError(
      `output info: the header says ${motors} motors and ${servos} servos and ` +
        `${count} descriptors follow`,
    );
  }

  const outputs: OutputDescriptor[] = [];
  for (let i = 0; i < count; i++) {
    const kind = reader.u8(`output info: entry ${i} kind`);
    const index = reader.u8(`output info: entry ${i} index`);
    const reversed = reader.u8(`output info: entry ${i} reversed`) !== 0;
    const trimUs = reader.i16(`output info: entry ${i} trim`);
    const travelUs = reader.u16(`output info: entry ${i} travel`);
    const isServo = kind === OutputKind.SERVO;
    outputs.push({
      kind,
      kindName: OUTPUT_KIND_NAMES[kind] ?? null,
      index,
      reversed,
      trimUs: isServo ? trimUs : null,
      travelUs: isServo ? travelUs : null,
    });
  }
  return { status, count, motors, servos, capPct, outputs };
}

/**
 * What one `output test` command did, from the reply.
 *
 * **The echo is the point.** The reply names the op, the kind, the index and
 * the percentage actually driven on every path, and `parseOutputTest` checks the
 * three it was given against what was sent. A client holding a screen of four
 * outputs has to know which answer it is holding, and one that has lost track of
 * the state can send a stop without asking first — which is why that check
 * matters more here than anywhere else in this protocol.
 *
 * Two properties a caller acts on:
 *
 *  - `levelPct` is what *will* be driven, not what was asked for. Asking for
 *    200 is neither an error nor obeyed; it comes back as the firmware's cap,
 *    and a countdown driven from the request rather than from this number would
 *    be counting something the board never agreed to.
 *  - `STOPPED` is a success rather than a refusal. Every other status means
 *    nothing was driven, and `ARMED` in particular is a policy the board
 *    applied rather than a fault.
 */
export interface OutputTestAnswer {
  readonly status: OutputTestStatus;
  readonly statusText: string;
  readonly op: number;
  /** `null` when this app has no name for that op number. */
  readonly opName: string | null;
  readonly kind: number;
  /** `null` for the `0xFF` a short request is answered with — deliberately not
   *  a kind, so "you did not say which" does not read as output zero. */
  readonly kindName: string | null;
  readonly index: number;
  readonly levelPct: number;
  readonly remainingMs: number;
}

/** The request an `output test` frame carries: `op, kind, index, level`. */
export interface OutputTestRequest {
  readonly op: OutputTestOp;
  readonly kind: number;
  readonly index: number;
  readonly levelPct: number;
}

export function parseOutputTest(
  payload: Uint8Array,
  sent?: OutputTestRequest,
): OutputTestAnswer {
  const reader = new Reader(payload);
  const raw = reader.u8('output test: status');

  if (raw === UNKNOWN_COMMAND) {
    throw new ProtocolError(
      'output test: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `output test`, and reading it as a ' +
        'refusal would report a policy decision that was never made',
    );
  }

  const status = raw as OutputTestStatus;
  if (OUTPUT_TEST_STATUS_TEXT[status] === undefined) {
    throw new ProtocolError(`output test: status ${status} is not one this protocol defines`);
  }

  const op = reader.u8('output test: op');
  const kind = reader.u8('output test: kind');
  const index = reader.u8('output test: index');
  const levelPct = reader.u8('output test: level');
  const remainingMs = reader.u16('output test: remaining');
  if (reader.remaining !== 0) {
    throw new ProtocolError(`output test: ${reader.remaining} bytes left over after the reply`);
  }

  // Both of these are the firmware's own clamp, so a reply outside them is a
  // firmware that broke its own promise. Checked rather than trusted for the
  // reason every length here is: a level of 200 drawn as a slider position is a
  // plausible-looking wrong answer, and this app's job is to be unable to show
  // one. `STOPPED` and the refusals drive nothing, so they carry zeroes and are
  // not about a level at all.
  if (status === OutputTestStatus.OK) {
    if (levelPct > OUTPUT_TEST_MAX_PCT) {
      throw new ProtocolError(
        `output test: the reply says it will drive ${levelPct}% and this firmware's cap is ` +
          `${OUTPUT_TEST_MAX_PCT}%`,
      );
    }
    if (remainingMs > OUTPUT_TEST_MAX_MS) {
      throw new ProtocolError(
        `output test: the reply says it will run for ${remainingMs} ms and this firmware's ` +
          `cap is ${OUTPUT_TEST_MAX_MS} ms`,
      );
    }
  }

  if (sent !== undefined && (op !== sent.op || kind !== sent.kind || index !== sent.index)) {
    throw new ProtocolError(
      `output test: asked about kind ${sent.kind} output ${sent.index} with op ${sent.op} and ` +
        `the reply names kind ${kind} output ${index} with op ${op} — the answer belongs to ` +
        `another request`,
    );
  }

  return {
    status,
    statusText: OUTPUT_TEST_STATUS_TEXT[status],
    op,
    opName:
      op === OutputTestOp.HOLD ? 'hold' : op === OutputTestOp.STOP ? 'stop' : null,
    kind,
    kindName: OUTPUT_KIND_NAMES[kind] ?? null,
    index,
    levelPct,
    remainingMs,
  };
}

/**
 * One row of the table's own description of itself, from a `param info` page.
 *
 * **Everything here came off the wire.** That is the whole difference between
 * this and what it replaces: the app used to hold a build-time snapshot of
 * `ak_flight.c` and show its ranges beside whatever board answered, which is a
 * second authority and was 32 rows stale against a 92-parameter board. A row
 * with no `BoardMeta` now means the board did not describe it, and the panel
 * says so rather than reaching for a file.
 */
export interface BoardMeta {
  readonly name: string;
  /** The wire's number, kept as well as the name so a row can say "type 7"
   *  rather than nothing when the app does not know the shape. */
  readonly type: number;
  readonly typeName: string | null;
  readonly group: number;
  /** The group's name, or **null when this app has no name for that number** —
   *  a client carries its own copy of `ak_param_group_t` (see
   *  `protocol/constants.ts`), and a number it cannot resolve must not be
   *  rendered as a neighbour's name. */
  readonly groupName: string | null;
  readonly decimals: number;
  readonly flags: number;
  /** `AK_PARAM_SECRET`. */
  readonly secret: boolean;
  /** The bounds as the board spelled them. Empty for a text parameter, which
   *  has no numeric range. Kept as text and parsed separately, because the
   *  protocol has no typed fields anywhere and this app does not invent one. */
  readonly min: string;
  readonly max: string;
  /** Text parameters only: how many characters the row may hold. Null for a
   *  numeric one, whose bound is a range instead. */
  readonly maxLen: number | null;
  readonly default: string;
  /**
   * The row's prose, from `param help`. **Null until it has been fetched** —
   * the help walk runs after the table is usable rather than inside the connect
   * walk, so a row renders before its sentence arrives. Null here means "not
   * read yet", never "the board has none": an empty help string is the board
   * saying it has none, and the two are kept apart.
   */
  readonly help: string | null;
}

export interface ParamInfoPage {
  readonly status: InfoStatus;
  /** The index the page is about, echoed by the board. */
  readonly first: number;
  readonly entries: readonly BoardMeta[];
}

/**
 * One `param info` page. `askedFrom` is the index the request named, and the
 * page has to agree with it: a page built from the wrong offset would join onto
 * the previous one and produce a table that is *right about every parameter and
 * wrong about every index*, which is the shape of mistake this walk exists not
 * to make.
 */
export function parseParamInfoPage(payload: Uint8Array, askedFrom: number): ParamInfoPage {
  const reader = new Reader(payload);
  const status = reader.u8('param info: status');

  if (status === UNKNOWN_COMMAND) {
    // A board that does not implement the command still answers — with 0x7F,
    // correlated to the request. Saying so is the point: this is a board that
    // predates `param info`, not a board that went silent.
    throw new ProtocolError(
      'param info: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `param info`',
    );
  }
  if (status === InfoStatus.NO_INDEX) {
    // One byte, and the reply stops there. This app never sends an index-less
    // request, so reaching here means the board read the frame differently.
    return { status: InfoStatus.NO_INDEX, first: askedFrom, entries: [] };
  }
  if (status !== InfoStatus.OK && status !== InfoStatus.TOO_BIG) {
    throw new ProtocolError(
      `param info: the board answered status ${status}, which this build does not define`,
    );
  }

  const first = reader.u8('param info: first index');
  const carried = reader.u8('param info: entries');
  if (first !== (askedFrom & 0xff)) {
    throw new ProtocolError(
      `param info: asked from ${askedFrom} and the page says ${first}`,
    );
  }

  const entries: BoardMeta[] = [];
  for (let i = 0; i < carried; i++) {
    entries.push(parseParamEntry(reader, entries.length));
  }
  if (reader.remaining !== 0) {
    throw new ProtocolError(
      `param info: the page carried ${carried} entries and ${reader.remaining} ` +
        'byte(s) over — the entries and the frame disagree',
    );
  }
  return { status: status as InfoStatus, first, entries };
}

function parseParamEntry(reader: Reader, at: number): BoardMeta {
  const name = reader.cstring(`param info: entry ${at} name`);
  const type = reader.u8(`param info: entry ${at} type`);
  const group = reader.u8(`param info: entry ${at} group`);
  const decimals = reader.u8(`param info: entry ${at} decimals`);
  const flags = reader.u8(`param info: entry ${at} flags`);

  let min = '';
  let max = '';
  let maxLen: number | null = null;
  if (type === ParamType.TEXT) {
    // A text row is bounded by how many characters it holds, in the byte a
    // numeric row would begin its minimum in. The entry's own `type` is what
    // tells the two shapes apart, so neither the client nor the firmware needs
    // a version to know which one it is reading.
    maxLen = reader.u8(`param info: entry ${at} max length`);
  } else if (type === ParamType.FLOAT || type === ParamType.U32) {
    min = reader.cstring(`param info: entry ${at} minimum`);
    max = reader.cstring(`param info: entry ${at} maximum`);
  } else {
    // A shape this app cannot read is an error, not a guess: the next field
    // would be read as whatever this build expected it to be, and a plausible
    // wrong number is the failure mode this file is written against.
    throw new ProtocolError(
      `param info: entry ${at} (${name}) says type ${type}, which this build does ` +
        'not know the shape of',
    );
  }
  const value = reader.cstring(`param info: entry ${at} default`);

  return {
    name,
    type,
    typeName: PARAM_TYPE_NAMES[type] ?? null,
    group,
    groupName: PARAM_GROUPS[group] ?? null,
    decimals,
    flags,
    secret: (flags & 0x01) !== 0,
    min,
    max,
    maxLen,
    default: value,
    help: null,
  };
}

/**
 * One slice of a row's help text, walked by offset.
 *
 * Walked rather than paged, which is what makes a help string longer than a
 * frame an ordinary case: the client is done when it has `total` bytes. The
 * reply echoes the offset it answered from, and that echo is checked — a slice
 * joined at the wrong offset produces prose that reads perfectly and says
 * something the firmware never wrote.
 */
export interface ParamHelpSlice {
  readonly index: number;
  readonly offset: number;
  readonly total: number;
  /**
   * The bytes, not a decoded string.
   *
   * `offset` is a *byte* offset on the wire, so a slice that decoded here and
   * advanced by `text.length` would walk the wrong distance the first time a
   * help string held a character wider than one byte — and the result would
   * read as prose. The caller joins the bytes and decodes once.
   */
  readonly bytes: Uint8Array;
}

export function parseParamHelpSlice(payload: Uint8Array, askedFrom: number): ParamHelpSlice {
  const reader = new Reader(payload);
  const status = reader.u8('param help: status');
  if (status === UNKNOWN_COMMAND) {
    throw new ProtocolError(
      'param help: the board answered 0x7F — this build predates `param help`',
    );
  }
  if (status !== 0) {
    throw new ProtocolError(`param help: the board refused index ${askedFrom} with status ${status}`);
  }
  const index = reader.u8('param help: index');
  const offset = reader.u16('param help: offset');
  const total = reader.u16('param help: total');
  const length = reader.u8('param help: length');
  const bytes = reader.bytes(length, 'param help: text');
  if (index !== (askedFrom & 0xff)) {
    throw new ProtocolError(`param help: asked about ${askedFrom} and the reply says ${index}`);
  }
  if (reader.remaining !== 0) {
    throw new ProtocolError(
      `param help: the reply declared ${length} byte(s) of text and carried ` +
        `${length + reader.remaining}`,
    );
  }
  return { index, offset, total, bytes };
}

/** A numeric bound the board spelled, or null when this app will not evaluate
 *  it. `null` is a real answer: the panel then shows no bound rather than a
 *  guessed one, and says which of the two it is. */
export function boundFromText(text: string): number | null {
  const trimmed = text.trim();
  if (trimmed === '') return null;
  const value = Number(trimmed);
  return Number.isFinite(value) ? value : null;
}

export interface Status {
  readonly flightState: number;
  readonly linkLive: number;
  readonly gpsFixType: number;
  readonly gpsSatellites: number;
  readonly rollDeg: number;
  readonly pitchDeg: number;
  readonly yawDeg: number;
  readonly lat: number;
  readonly lon: number;
  /** Four outputs. 0..254, as the log record stores them. */
  readonly motors: readonly number[];
}

export function parseStatus(payload: Uint8Array): Status {
  const reader = new Reader(payload);
  const flightState = reader.u8('status: flight state');
  const linkLive = reader.u8('status: link');
  const gpsFixType = reader.u8('status: fix type');
  const gpsSatellites = reader.u8('status: satellites');
  const rollDeg = reader.i16('status: roll') / 10;
  const pitchDeg = reader.i16('status: pitch') / 10;
  const yawDeg = reader.i16('status: yaw') / 10;
  const lat = reader.i32('status: latitude') / 1e7;
  const lon = reader.i32('status: longitude') / 1e7;
  const motors = Array.from(reader.bytes(4, 'status: motors'));
  return {
    flightState,
    linkLive,
    gpsFixType,
    gpsSatellites,
    rollDeg,
    pitchDeg,
    yawDeg,
    lat,
    lon,
    motors,
  };
}

export interface Telemetry extends Status {
  readonly uptimeMs: number;
}

/** A pushed telemetry frame: the status body with an uptime in front. */
export function parseTelemetry(payload: Uint8Array): Telemetry {
  const reader = new Reader(payload);
  const uptimeMs = reader.u32('telemetry: uptime');
  const rest = payload.subarray(4);
  return { uptimeMs, ...parseStatus(rest) };
}

export interface LogSourceReply {
  readonly status: number;
  readonly source: LogSource | null;
  /** Records in the selected log, or null when the device refused the source —
   *  "no such log" is a different answer from "an empty one", and the firmware
   *  spends a status byte to keep them apart. */
  readonly count: number | null;
}

export function parseLogSource(payload: Uint8Array): LogSourceReply {
  const reader = new Reader(payload);
  const status = reader.u8('log source: status');
  const source = reader.u8('log source: source');
  const count = reader.u16('log source: count');
  if (status !== 0) return { status, source: null, count: null };
  return { status, source: source as LogSource, count };
}

/**
 * How many records the selected log holds.
 *
 * **Two bytes and no status byte**, which is the whole of what this reply can
 * say: the firmware writes `count > 0 ? count : 0`, so an absent ring, an
 * absent log reader and an empty ring all arrive here as the same zero. There
 * is deliberately no `LogInfoReply` object for that reason — a `{status, count}`
 * shape would invent a status the wire does not carry, and one of the three
 * zeros would end up rendering as a fact about hardware. Where the difference
 * matters, it is `LOG_SOURCE`'s answer that must be read, not this one's.
 */
export function parseLogInfo(payload: Uint8Array): number {
  return new Reader(payload).u16('log info: count');
}

/**
 * What a stream request was answered with: the range and the rate that will
 * actually be sent, after the board clamped both.
 *
 * `accepted` is not "the request was well formed" — a refusal and an accepted
 * request that will send nothing are different facts and arrive as different
 * status bytes. `count === 0` is the second one, and it is the same answer
 * whether the range named no records or the rate was zero, which is deliberate
 * on the firmware's side: a client's job is to notice no stream is coming.
 */
export interface LogStreamReply {
  readonly status: number;
  readonly source: LogSource | null;
  readonly first: number | null;
  readonly count: number | null;
  readonly hz: number | null;
}

/**
 * The answer to a stream request: `status, source, first, count, rate`.
 *
 * A refusal carries the source it refused and four zeros, and this returns
 * `null` for the four rather than passing zeroes on as facts — a client that
 * read `first: 0, count: 0` out of a refusal would draw an empty range where
 * the board said "there is no such log here".
 */
export function parseLogStreamReply(payload: Uint8Array): LogStreamReply {
  const reader = new Reader(payload);
  const status = reader.u8('log stream: status');
  const source = reader.u8('log stream: source');
  const first = reader.u16('log stream: first');
  const count = reader.u16('log stream: count');
  const hz = reader.u8('log stream: rate');
  if (status !== 0) return { status, source: null, first: null, count: null, hz: null };
  return { status, source: source as LogSource, first, count, hz };
}

/**
 * One pushed frame of a log stream.
 *
 * The `index` is the record's own, read off the frame — **never a counter kept
 * by the reader.** A stream that hit a hole would leave a reader counting from
 * zero plotting the records after it against the wrong timestamps, and doing it
 * without saying anything.
 *
 * `record` is null for `HOLE` and `DONE`, and the three statuses are kept
 * apart rather than collapsed into "no record": a hole is a record the board
 * would not produce, and `DONE` is the end of a range. Rendering them the same
 * way would turn "this range finished" and "this range lost three records"
 * into the same picture.
 */
export interface LogStreamFrame {
  readonly status: LogStreamStatus;
  readonly source: LogSource;
  readonly index: number;
  readonly record: LogRecord | null;
}

export function parseLogStreamFrame(payload: Uint8Array): LogStreamFrame {
  const reader = new Reader(payload);
  const status = reader.u8('log stream frame: status');
  const source = reader.u8('log stream frame: source');
  const index = reader.u16('log stream frame: index');
  if (status !== LogStreamStatus.RECORD) {
    return { status: status as LogStreamStatus, source: source as LogSource, index, record: null };
  }
  const body = reader.bytes(logRecordBytes(reader.remaining, 'log stream frame: record'), 'log stream frame: record');
  // The record decoder wants `LOG_GET`'s shape — a status byte and then the
  // record — and a pushed frame carries its status three fields earlier. So the
  // byte is put back in front rather than a second decoder written: two
  // decoders for one record layout is exactly how the two come to disagree,
  // and the length-driven version test above has to be in one place for the
  // same reason.
  const framed = new Uint8Array(1 + body.length);
  framed.set(body, 1);
  return {
    status: LogStreamStatus.RECORD,
    source: source as LogSource,
    index,
    record: parseLogRecord(framed, index),
  };
}

export interface LogRecord {
  /** Index within the *selected* log, oldest first. */
  readonly index: number;
  readonly timeMs: number;
  readonly gyro: readonly [number, number, number];
  readonly accel: readonly [number, number, number];
  readonly rollDeg: number;
  readonly pitchDeg: number;
  readonly yawDeg: number;
  readonly altMm: number;
  readonly stick: readonly [number, number, number, number];
  readonly torque: readonly [number, number, number];
  readonly motors: readonly [number, number, number, number];
  readonly state: number;
  readonly flags: number;
  readonly lat: number;
  readonly lon: number;
  /**
   * The same quantity as `gyro` in the same unit, after the notch bank and both
   * low-passes — the reading the controller actually flew on. `null` on a
   * record written before roadmap 2.4, which is a *different claim* from a
   * filtered reading of zero: one says the firmware did not write the field,
   * the other that it wrote a zero into it.
   */
  readonly gyroFiltered: readonly [number, number, number] | null;
  /** Where the notches are sitting, whole Hz, or null on an older record.
   *  Per axis the first engaged slot's centre, and zero *within* an axis means
   *  no measurement has engaged a notch there — a slot with nothing behind it
   *  is bypassed, so it has no centre to report. */
  readonly notchHz: readonly [number, number, number] | null;
  /** How many slots a measurement has engaged per axis, or null when the
   *  record predates the field. */
  readonly notchEngaged: readonly [number, number, number] | null;
  /**
   * Version 4's columns (roadmap 4.1), each null on an older record for the
   * same reason the filtering ones are. `timeUs` is the loop's own microsecond
   * clock, which wraps every 71.6 minutes and is what a per-sample interval is
   * read from; `timeMs` is the one to sort by.
   */
  readonly timeUs: number | null;
  /** What the rate loop was asked for, per axis, in 0.1 dps. */
  readonly rateSetpoint: readonly [number, number, number] | null;
  /** The rate loop's terms per axis, in torque percent: P + I - D is the
   *  torque before its clamp, and ±127 means the term saturated. */
  readonly pidP: readonly [number, number, number] | null;
  readonly pidI: readonly [number, number, number] | null;
  readonly pidD: readonly [number, number, number] | null;
  /** Pack voltage in mV, or null when the record predates the field *or* the
   *  board had no reading (flag 0x10 clear) — the firmware writes 0 there, and
   *  0 mV is not a pack. */
  readonly vbatMv: number | null;
}

/** Version 2's record, which is version 3's first 51 bytes. */
const LOG_RECORD_V2_BYTES = 51;
/** Version 3's: the above plus the filtering trio (roadmap 2.4). */
const LOG_RECORD_V3_BYTES = 66;
/** Version 4's: the above plus the controller's columns (roadmap 4.1). */
const LOG_RECORD_BYTES = 87;
/** `AK_LOG_VBAT_VALID`: the record's `vbatMv` is a reading. */
export const LOG_FLAG_VBAT_VALID = 0x10;
/** `AK_LOG_ANGLE_MODE`: the pilot had angle mode selected. */
export const LOG_FLAG_ANGLE_MODE = 0x08;

/**
 * How many bytes this record is, from how many are actually here.
 *
 * The record carries no version of its own — the ring's header does, and the
 * header is not on this wire — so the frame's length is the only statement
 * about which layout arrived. Roadmap 2.4 appended fifteen bytes and 4.1
 * twenty-one more, so a board running 4.1 answers 87, one before it 66, and one
 * before 2.4 51, and the difference is *reported* (the fields the board did not
 * write come back null) rather than papered over with zeros a person would
 * plot.
 *
 * Anything else is a truncation: 52 bytes is not a record, it is a record with
 * three bytes missing, and reading it as 51 would report a decode that worked.
 */
function logRecordBytes(available: number, what: string): number {
  if (available >= LOG_RECORD_BYTES) return LOG_RECORD_BYTES;
  if (available === LOG_RECORD_V3_BYTES) return LOG_RECORD_V3_BYTES;
  if (available === LOG_RECORD_V2_BYTES) return LOG_RECORD_V2_BYTES;
  throw new ProtocolError(
    `${what}: a record is ${LOG_RECORD_BYTES} bytes, ${LOG_RECORD_V3_BYTES} from a ` +
      `firmware that predates the controller fields, or ${LOG_RECORD_V2_BYTES} from one ` +
      `that predates the filtering fields; the reply left ${available}`,
  );
}

/** The 87-byte record from `ak_log.h`, field for field. The layout has grown
 *  five times and every growth changed this decoder *and* the firmware's, on
 *  purpose: offsets that are wrong on one side produce a CSV that looks right.
 *
 *  Grown at the *end* since version 2, so an older board's shorter record
 *  decodes as a correct prefix and the fields it lacks come back null. */
export function parseLogRecord(payload: Uint8Array, index: number): LogRecord | null {
  const reader = new Reader(payload);
  const status = reader.u8('log get: status');
  if (status !== 0) return null;
  const body = reader.bytes(logRecordBytes(reader.remaining, 'log get: record'), 'log get: record');
  return decodeLogRecord(body, index);
}

function decodeLogRecord(body: Uint8Array, index: number): LogRecord {
  const view = new DataView(body.buffer, body.byteOffset, body.byteLength);
  const v3 = body.length >= LOG_RECORD_V3_BYTES;
  const v4 = body.length >= LOG_RECORD_BYTES;
  const flags = body[42]!;
  const i8x3 = (at: number): [number, number, number] => [
    view.getInt8(at),
    view.getInt8(at + 1),
    view.getInt8(at + 2),
  ];

  return {
    index,
    timeMs: view.getUint32(0, true),
    gyro: [view.getInt16(4, true), view.getInt16(6, true), view.getInt16(8, true)],
    accel: [view.getInt16(10, true), view.getInt16(12, true), view.getInt16(14, true)],
    rollDeg: view.getInt16(16, true) / 10,
    pitchDeg: view.getInt16(18, true) / 10,
    yawDeg: view.getInt16(20, true) / 10,
    altMm: view.getInt32(22, true),
    stick: [
      view.getInt16(26, true),
      view.getInt16(28, true),
      view.getInt16(30, true),
      view.getInt16(32, true),
    ],
    torque: [view.getInt8(34), view.getInt8(35), view.getInt8(36)],
    motors: [body[37]!, body[38]!, body[39]!, body[40]!],
    state: body[41]!,
    flags: body[42]!,
    lat: view.getInt32(43, true) / 1e7,
    lon: view.getInt32(47, true) / 1e7,
    gyroFiltered: v3
      ? [view.getInt16(51, true), view.getInt16(53, true), view.getInt16(55, true)]
      : null,
    notchHz: v3 ? [view.getUint16(57, true), view.getUint16(59, true), view.getUint16(61, true)] : null,
    notchEngaged: v3 ? [body[63]!, body[64]!, body[65]!] : null,
    timeUs: v4 ? view.getUint32(66, true) : null,
    rateSetpoint: v4
      ? [view.getInt16(70, true), view.getInt16(72, true), view.getInt16(74, true)]
      : null,
    pidP: v4 ? i8x3(76) : null,
    pidI: v4 ? i8x3(79) : null,
    pidD: v4 ? i8x3(82) : null,
    vbatMv: v4 && (flags & LOG_FLAG_VBAT_VALID) !== 0 ? view.getUint16(85, true) : null,
  };
}

/**
 * One page of one line of the board's checklist.
 *
 * The shape is `param help`'s with an index in front of it, and it exists for
 * the same reason: a sentence that does not fit a frame is walked by offset
 * rather than cut, and `sentenceLength` is what tells the reader it has the
 * whole thing. It is the *sentence's* length and not this page's — a reader that
 * compared the page against the frame's capacity would stop one byte short of
 * the end and show a fault without its cause.
 */
export interface PreflightPage {
  readonly status: PreflightStatus;
  /** The line that was asked for. `null` on `NONE`, where there is no
   *  checklist to have a line of. */
  readonly index: number | null;
  /** How many lines the board has — on the refusal too, so a reader that walked
   *  off the end can see where the end was instead of guessing. */
  readonly count: number;
  /** `null` on both of the whole-payload answers. */
  readonly verdict: PreflightVerdict | null;
  readonly name: string | null;
  /** The sentence's whole length, not this page's. */
  readonly sentenceLength: number;
  readonly detail: string;
}

export function parsePreflightPage(payload: Uint8Array): PreflightPage {
  const reader = new Reader(payload);
  // Read as the raw byte rather than as `PreflightStatus`: 0x7F is not a member
  // of that enum and is not meant to be — it is the firmware's answer for a
  // command a build does not have. Folding it into `NONE` would report "this
  // board has no checklist" about a board that may have one and simply predates
  // the opcode.
  const raw = reader.u8('preflight: status');

  if (raw === UNKNOWN_COMMAND) {
    throw new ProtocolError(
      'preflight: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `preflight`',
    );
  }

  const status = raw as PreflightStatus;
  if (status === PreflightStatus.NONE) {
    // One byte and no more. Everything this page would have carried — a count, a
    // name, a sentence — would be a claim about a checklist that does not exist.
    // Same rule as `rc channels` and `output info`, and checked rather than
    // trusted.
    if (reader.remaining !== 0) {
      throw new ProtocolError(
        `preflight: a board with no checklist answers one byte, and this reply carried ` +
          `${reader.remaining} more`,
      );
    }
    return {
      status,
      index: null,
      count: 0,
      verdict: null,
      name: null,
      sentenceLength: 0,
      detail: '',
    };
  }

  const index = reader.u8('preflight: index');
  const count = reader.u8('preflight: count');

  if (status === PreflightStatus.NO_INDEX) {
    // The index and the count, and nothing else. The index is the one that was
    // *asked for* rather than a failure to report it — the firmware echoes it —
    // and 0xFF is its answer when the request named none, which is out of range
    // for both fields so it cannot be read as "the board has 255 lines".
    if (reader.remaining !== 0) {
      throw new ProtocolError(
        `preflight: a refusal answers three bytes, and this reply carried ` +
          `${reader.remaining} more`,
      );
    }
    return {
      status,
      index,
      count,
      verdict: null,
      name: null,
      sentenceLength: 0,
      detail: '',
    };
  }

  if (status !== PreflightStatus.OK) {
    throw new ProtocolError(`preflight: status ${status} is not one this protocol defines`);
  }

  const verdict = reader.u8('preflight: verdict') as PreflightVerdict;
  if (verdict !== PreflightVerdict.FAIL && verdict !== PreflightVerdict.PASS &&
      verdict !== PreflightVerdict.FACT) {
    throw new ProtocolError(
      `preflight: verdict ${verdict} is not one this protocol defines — the three are ` +
        `FAIL, PASS and FACT`,
    );
  }

  const nameLength = reader.u8('preflight: name length');
  if (nameLength > PREFLIGHT_NAME_MAX) {
    throw new ProtocolError(
      `preflight: a name of ${nameLength} bytes is longer than the protocol's field ` +
        `(${PREFLIGHT_NAME_MAX})`,
    );
  }
  const name = reader.ascii(nameLength, 'preflight: name');
  const sentenceLength = reader.u16('preflight: sentence length');
  const detailLength = reader.u8('preflight: page length');
  const detail = reader.ascii(detailLength, 'preflight: sentence');

  if (reader.remaining !== 0) {
    throw new ProtocolError(
      `preflight: the page carried ${reader.remaining} bytes past the sentence it declared`,
    );
  }
  // The page can never be longer than the sentence it belongs to. A page that
  // claimed to would make `offset + detail.length` overshoot `sentenceLength`
  // and a walker would either loop or stop early; both are silent.
  if (detail.length > sentenceLength) {
    throw new ProtocolError(
      `preflight: line ${index} carries ${detail.length} bytes of a sentence it says is ` +
        `${sentenceLength} long`,
    );
  }

  return { status, index, count, verdict, name, sentenceLength, detail };
}

/**
 * The mission, as one `MISSION` reply carries it.
 *
 * **Every verb answers with this whole shape**, not with an acknowledgement: the
 * state is the aircraft's answer about itself *after* the verb ran, so a client
 * that has just sent `start` reads the effect of its own frame without a second
 * round trip that could fail on its own. That ordering is a property of the
 * firmware — `main.c` fills the state at the end of `proto_mission` — and this
 * decoder cannot check it. What it can check is the bytes.
 *
 * **`requested` and `active` are two different facts, and conflating them is the
 * bug the pair exists to prevent.** `requested` is "a mission has been asked for
 * and not taken back"; `active` is "the navigator is flying one now". The
 * console's `start` does not start anything — it sets a request the flight loop
 * honours on a later pass, once the aircraft is armed and flying — so a reply
 * carrying only `active` would tell a client its own button had done nothing,
 * and one carrying only `requested` would let it draw a mission in progress
 * about an aircraft sitting on a bench.
 */
export interface MissionState {
  readonly status: MissionStatus;
  /**
   * The verb the reply echoes, **never corrected by the firmware**. `0xFF` is
   * "the frame did not say"; any other value out of range is "you asked for a
   * verb this build does not have". Both are `NO_VERB`, and the raw byte is
   * kept because it is what tells the two apart.
   */
  readonly op: number;
  /** `op` resolved to a name, or `null` for either of the two cases above. */
  readonly verb: MissionVerb | null;
  /** The navigator is flying a mission now. Never derived from `requested`. */
  readonly active: boolean;
  /** A mission has been asked for and not taken back. */
  readonly requested: boolean;
  /**
   * Waypoints in the list, from the board's `wp_count`.
   *
   * **255 is "255 or more"**: the field is one byte and the firmware clamps
   * rather than refusing, so a table with 300 waypoints reports 255.
   */
  readonly count: number;
  /**
   * The waypoint being flown, or **null when the navigator is flying none** —
   * the wire's `0xFF`, kept out of the numbering because waypoint zero is a real
   * waypoint and the two ends of a mission must not draw the same way.
   */
  readonly index: number | null;
  /**
   * The mission switch's channel, or **null when no switch is configured** — the
   * wire's zero. Absent is not zero here: the parameter behind this field is
   * one-based with a range starting at 1, so zero is the *absence* of a switch
   * rather than a channel number.
   */
  readonly channel: number | null;
  readonly reached: number;
  readonly started: number;
  readonly cancelled: number;
  /**
   * The altitude a running mission holds, in mm, and zero when none is running.
   * The field behind it is left wherever the last mission put it, and a number
   * nobody is holding is not a fact about now.
   */
  readonly holdAltMm: number;
}

export function parseMission(payload: Uint8Array): MissionState {
  const reader = new Reader(payload);
  // Read as the raw byte rather than as `MissionStatus`: 0x7F is not a member of
  // that enum and is not meant to be. It is the one-byte answer the firmware
  // gives a command a build does not implement, and folding it into a status
  // would report "this board has no navigator" about a board that may have one
  // and simply predates the opcode.
  const raw = reader.u8('mission: status');

  if (raw === UNKNOWN_COMMAND) {
    throw new ProtocolError(
      'mission: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `mission`',
    );
  }
  if (raw > MissionStatus.NO_FIX) {
    throw new ProtocolError(`mission: status ${raw} is not one this protocol defines`);
  }

  // Every mission reply is the same seventeen bytes whatever the verb and
  // whatever the status, so a short one is a truncated frame rather than a
  // shorter answer. Checked once with the frame's own size in the sentence,
  // rather than field by field where the message would be about one integer.
  reader.need(16, 'mission: the reply');

  const status = raw as MissionStatus;
  const op = reader.u8('mission: op');
  const active = reader.u8('mission: active') !== 0;
  const requested = reader.u8('mission: requested') !== 0;
  const count = reader.u8('mission: count');
  const index = reader.u8('mission: index');
  const channel = reader.u8('mission: channel');
  const reached = reader.u16('mission: reached');
  const started = reader.u16('mission: started');
  const cancelled = reader.u16('mission: cancelled');
  const holdAltMm = reader.i32('mission: hold altitude');

  if (reader.remaining !== 0) {
    throw new ProtocolError(
      `mission: every mission reply is 17 bytes and this one carried ${17 + reader.remaining}`,
    );
  }

  return {
    status,
    op,
    verb: MISSION_VERB_NAMES[op] === undefined ? null : (op as MissionVerb),
    active,
    requested,
    count,
    index: index === MISSION_NO_INDEX ? null : index,
    channel: channel === 0 ? null : channel,
    reached,
    started,
    cancelled,
    holdAltMm,
  };
}

/**
 * The calibration session, as one `CALIBRATE` reply carries it.
 *
 * **Every verb answers with this whole shape**, not with an acknowledgement:
 * the state is a *reading* of the session, so `calibrate gyro` comes back with
 * the session it started and the wizard draws progress from the same frame it
 * began with, without a second round trip that could fail on its own. That is
 * the whole reason `ABORT` and `STATUS` can both be answered while a session is
 * running — the loop keeps turning because no verb blocks it.
 *
 * **`verb` is the *session's*, not an echo of the request.** This is the one
 * place the two differ and it is the field's entire purpose: the verb beside the
 * six result slots is what says how to read them, so a `status` poll during a
 * gyro calibration reports `GYRO`, and asked for `abort` on a board that has
 * never calibrated anything it reports `NO_SESSION`. A client that echoed its
 * own request would be reading the one byte it is meant to learn from. The
 * refusals that are about the *request* — `NO_VERB` — keep their echo, because
 * there the byte names the input the dispatch could not use.
 *
 * **`NO_SESSION` and a verb of `STATUS` are different claims.** `STATUS` is a
 * session that exists and is idle; `NO_SESSION` (`0xFF`) is a board that has
 * never had one. Both come with six zero results, and rendering them the same
 * way would be this app inventing a measurement out of an absence.
 */
export interface CalibrationState {
  readonly status: CalibrateStatus;
  /**
   * The session's verb, resolved — or `null` for the two "not a verb" cases,
   * `NO_SESSION` and an out-of-range echo.
   */
  readonly verb: CalibrateVerb | null;
  /** The raw verb byte, kept because `NO_SESSION` and an out-of-range echo are
   *  two different "not a verb" a caller may need to tell apart. */
  readonly verbRaw: number;
  /** A session is sampling now. */
  readonly active: boolean;
  /**
   * The accelerometer face being sampled, or **null when none is** — the wire's
   * `NO_STEP`, kept out of the numbering because face zero is a real face and
   * "not sampling" must not draw as "sampling face zero".
   */
  readonly step: number | null;
  /** The bitmask of faces already measured: meaningful only for `ACCEL`, and
   *  zero for the other three because a gyro calibration has no faces. */
  readonly faces: number;
  /** How many faces of the six are in, for the wizard's progress. */
  readonly facesDone: number;
  /** Samples taken toward this session. */
  readonly samples: number;
  /** Samples refused — for moving, or for a bad frame. */
  readonly rejected: number;
  /**
   * The six result slots, fixed-point and signed.
   *
   * Their meaning is fixed by `verb`, and the names come from
   * `CALIBRATE_RESULT_NAMES` — a prefix of the six for the three that carry
   * fewer. Slots past the measurement are real zeros, not placeholders.
   */
  readonly results: readonly number[];
  /** `CALIBRATE_RESULT_NAMES[verb]`, or an empty list when there is no session
   *  to name one for. */
  readonly resultNames: readonly string[];
}

export function parseCalibration(payload: Uint8Array): CalibrationState {
  const reader = new Reader(payload);
  const raw = reader.u8('calibrate: status');

  if (raw === UNKNOWN_COMMAND) {
    throw new ProtocolError(
      'calibrate: the board answered 0x7F, the status this firmware gives a command ' +
        'it does not implement — this build predates `calibrate`',
    );
  }
  if (raw > CalibrateStatus.BAD_VALUE) {
    throw new ProtocolError(`calibrate: status ${raw} is not one this protocol defines`);
  }

  // Five bytes of header, then the two counts, then the six results: 5 + 8 + 24
  // = 37, and it is the same thirty-seven whatever the verb and whatever the
  // status. A short one is a truncated frame rather than a shorter answer, so it
  // is checked once with the frame's own size in the sentence.
  reader.need(36, 'calibrate: the reply');

  const status = raw as CalibrateStatus;
  const verbRaw = reader.u8('calibrate: verb');
  const active = reader.u8('calibrate: active') !== 0;
  const step = reader.u8('calibrate: step');
  const faces = reader.u8('calibrate: faces');
  const samples = reader.u32('calibrate: samples');
  const rejected = reader.u32('calibrate: rejected');

  const results: number[] = [];
  for (let i = 0; i < CALIBRATE_RESULT; i += 1) {
    results.push(reader.i32(`calibrate: result ${i}`));
  }

  if (reader.remaining !== 0) {
    throw new ProtocolError(
      `calibrate: every calibrate reply is 37 bytes and this one carried ${
        37 + reader.remaining
      }`,
    );
  }

  // Resolved against the *verb* table, not the results table, and the
  // difference is the two verbs that carry no numbers: `STATUS` names a session
  // that exists and is idle, and `ABORT` names the session an abort just ended.
  // Reading the verb off `CALIBRATE_RESULT_NAMES` would report both as no
  // session at all — the one confusion this parser exists to prevent.
  const verb = CALIBRATE_VERB_NAMES[verbRaw] === undefined ? null : (verbRaw as CalibrateVerb);
  const done = popcount(faces & CALIBRATE_ALL_FACES);

  return {
    status,
    verb,
    verbRaw,
    active,
    step: step === CALIBRATE_NO_STEP ? null : step,
    faces,
    facesDone: done,
    samples,
    rejected,
    results,
    resultNames: CALIBRATE_RESULT_NAMES[verbRaw] ?? [],
  };
}

/** How many of a six-bit mask are set. A helper rather than an inline reduce so
 *  the wizard's "3 of 6 faces" and the parser's `facesDone` cannot count
 *  differently. */
function popcount(mask: number): number {
  let count = 0;
  for (let bit = 0; bit < 8; bit += 1) {
    if ((mask & (1 << bit)) !== 0) count += 1;
  }
  return count;
}
