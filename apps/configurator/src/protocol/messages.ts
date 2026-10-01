import {
  BATTERY_STATES,
  IMU_ABSENT,
  InfoStatus,
  LogSource,
  PARAM_GROUPS,
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
  const uptimeMs = reader.i32('telemetry: uptime');
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
}

/** The 51-byte record from `ak_log.h`, field for field. The layout has grown
 *  three times and every growth changed this decoder *and* the firmware's, on
 *  purpose: offsets that are wrong on one side produce a CSV that looks right. */
export function parseLogRecord(payload: Uint8Array, index: number): LogRecord | null {
  const reader = new Reader(payload);
  const status = reader.u8('log get: status');
  if (status !== 0) return null;
  const body = reader.bytes(51, 'log get: record');
  const view = new DataView(body.buffer, body.byteOffset, body.byteLength);

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
  };
}
