import {
  Command,
  InfoStatus,
  MAX_PAYLOAD,
  ParamType,
  RcFlag,
  RcStatus,
  RcSwitch,
  SENSOR_NAME_LENGTH,
  SENSOR_TOPICS,
  Sensor,
  SensorStatus,
  SetStatus,
  TELEMETRY_MAX_HZ,
  UNKNOWN_COMMAND,
} from '../protocol/constants';
import { Feature } from '../protocol/features';
import { buildFrame, FrameDecoder, isResponse, type Frame } from '../protocol/frame';
import { paramsHash } from '../protocol/params-hash';

/**
 * A synthetic AerialKit that answers the real protocol.
 *
 * This exists so the app can be used, tested and demonstrated with no board, no
 * simulator and no companion process — from a phone, from a static host, from a
 * browser with no Web Serial. It is a *board simulator*, not a mock of the UI:
 * it produces frames the same decoder parses and answers commands the same
 * client sends, so the whole path from a click to a rendered number is real.
 *
 * The one thing it must never do is pass for hardware. Two rules hold that
 * line, and both are enforced by tests rather than by intention:
 *
 *  1. **It says so on the wire.** Its product string is `aerialkit-demo`, not
 *     the `aerialkit-f405` the capture recorded — the file it is built from
 *     carries both, so the difference is checkable rather than remembered.
 *     Anything that identifies the board therefore carries the word "demo"
 *     without the UI having to add it.
 *  2. **It is disarmed on boot**, where the firmware's simulator boots armed.
 *     The write gate is the most important behaviour in this app and a demo
 *     that cannot show it closing is a demo that hides the feature. The UI
 *     offers a switch for it, labelled as a demo control, because AerialKit has
 *     no "arm me" command over the config link and inventing one here would
 *     teach a person a command that does not exist.
 *
 * Everything else mirrors the device, including the parts that are inconvenient
 * — `param save` succeeds and clears the changed count, because that is what
 * the board this is standing in for does. Whether anything survives a reload is
 * a fact about the *transport*, and it is stated there, once, in the identity
 * strip. A demo that lied at the protocol layer to make a point at the identity
 * layer would break the app's own tests in the process.
 */

export interface DemoParameter {
  readonly name: string;
  readonly value: string;
  /** What `defaults` would restore. The demo board starts here. */
  readonly default?: string;
  readonly help?: string;
  readonly type?: 'float' | 'u32';
  readonly decimals?: number;
  readonly min?: number | null;
  readonly max?: number | null;
  /** `AK_PARAM_GROUP_*`, the number the board serves over `param info` and a
   *  client renders under its own name for. Omitted means 0 (`none`), which the
   *  app shows as "unknown group" rather than filing under a neighbour. */
  readonly group?: number;
}

export interface DemoBoardOptions {
  readonly parameters?: readonly DemoParameter[];
  readonly product?: string;
  /**
   * The `AK_PROTO_FEATURE_*` word this board reports, or **`null` to answer a
   * `hello` that ends before the field** — the shape a board predating it
   * sends.
   *
   * The two are kept apart on purpose, and the option exists so the "old
   * firmware" path can be driven against a real peer rather than against a
   * mock of one: `null` here produces the byte sequence an old board produces,
   * and everything downstream of it — the parser, the session, the tab rail —
   * has to cope. Omitted means the default below, which mirrors `main.c`.
   */
  readonly features?: number | null;
  /** `ak_params_hash` of this board's table. Omitted means it is computed from
   *  the table, the same way the firmware computes it. */
  readonly configHash?: number;
  /** When false, `param save` answers "nowhere to save". The default mirrors a
   *  board with storage; tests drive the refusal with this. */
  readonly canSave?: boolean;
  /** How many records each log holds. Index 0 fast, 1 long, 2 flash. A `null`
   *  entry means this device does not have that log at all, which the board
   *  reports as a refusal rather than as an empty log — two different answers. */
  readonly logCounts?: readonly (number | null)[];
  /**
   * What this board's receiver input is: `crsf`, `sbus`, or **`null` for a
   * board with no receiver port at all**, which answers `rc channels` with
   * `AK_PROTO_RC_NONE` — one byte, and nothing after it.
   *
   * That third case is not decoration. "No receiver port" and "a receiver that
   * has never framed" are different faults with different fixes, and a
   * simulator that could only produce the second would leave the first
   * untestable against a real peer.
   */
  readonly receiver?: 'crsf' | 'sbus' | null;
  /**
   * Which sensors this board has fitted, by topic.
   *
   * Omitted means all five are present, which is what `aerialkit-fw-sim` has.
   * `false` for a topic makes the board answer `present: 0` with no body — the
   * middle of the three answers, and the one worth being able to drive: a board
   * that knows the question and has nothing in the socket is a different thing
   * from a firmware that has never heard of the topic at all, and telling a
   * person it is the wrong one sends them to update firmware that is current.
   */
  readonly sensors?: Partial<Record<Sensor, boolean>>;
}

const LOG_RECORD_BYTES = 51;
const DEFAULT_LOG_COUNTS: readonly (number | null)[] = [512, 4096, 2048];

/**
 * Where this simulated board is standing: the same numbers `statusBody`
 * reports, named once so the GPS sensor body can report the same place. Two
 * panels of one app disagreeing about the aircraft's position would be the app
 * contradicting itself.
 */
const DEMO_LAT_E7 = 51.5e7;
const DEMO_LON_E7 = -0.12e7;

/** The BMI270's `WHO_AM_I`, which is what `ak_imu_bmi270.c` checks for. A
 *  plausible part number rather than a zero, because a demo that reported
 *  `whoami: 0` would be showing a board whose IMU did not identify itself. */
const DEMO_IMU_WHOAMI = 0x24;

/** The rate `main.c` samples the IMU at, which is the rate the console's own
 *  `imu ... samples` counter climbs at. */
const DEMO_IMU_HZ = 1000;

/** Standard sea-level pressure, which is the `reference` this board took when
 *  it powered up standing on a bench. */
const DEMO_BARO_SEA_PA = 101325;

/** A 4S pack's thresholds, from `ak_flight.c`'s defaults. */
const DEMO_BATTERY_WARN_MV = 3500;
const DEMO_BATTERY_CRITICAL_MV = 3300;
/** The divider's ratio and what the pin behind it reads, which is what the
 *  console's `battery` command prints as "1.090 V at the pin behind a 11:1
 *  divider". */
const DEMO_BATTERY_RATIO = 11;
const DEMO_BATTERY_PIN_MV = 1090;

/**
 * What this simulator claims, by default.
 *
 * `APPLIES_ON_WRITE | PARAM_INFO | PARAM_DEFAULT | GATES_ON_ARMED`, which is
 * exactly what `main.c` sets and for the same reason: it is a statement about
 * the build, not about the aircraft.
 *
 * **The gate bit was absent here until milestone 4, and its arrival is the
 * whole point of that milestone.** It used to be absent because a real board
 * did not set it either — `save` was armed-gated through `main.c`'s
 * `save_parameters` and `set` was not checked at all — so a simulator that
 * claimed it would have hidden the one limitation the app most needs to be able
 * to show. The firmware now consults `ak_proto_io_t.writable` on every write
 * route, so the bit is a true statement about the build and this board makes it.
 * What the app shows on a real board therefore changes with the firmware, which
 * is the correct behaviour and is why the limitation entry for it is a reading
 * of this word rather than a paragraph somebody has to remember to delete.
 *
 * The word is also this board's own statement about which commands it answers,
 * and this file holds itself to it: with the word absent (`features: null`) or
 * with a bit clear, that command falls through to the default case and is
 * answered `0x7F`, exactly as a firmware built without it does. A simulator that
 * answered them anyway would make the app's "old board" path untestable against
 * a real peer, which is the only way that path is worth testing.
 */
const DEMO_FEATURES =
  Feature.APPLIES_ON_WRITE |
  Feature.PARAM_INFO |
  Feature.PARAM_DEFAULT |
  Feature.GATES_ON_ARMED |
  Feature.RC_CHANNELS |
  Feature.SENSOR_INFO;

/**
 * A receiver frame every 20 ms, which is what CRSF and SBUS both do at their
 * usual rates. The counters below are derived from elapsed time against this
 * rather than incremented per poll: this board is polled at ten a second and
 * its receiver frames at fifty, and a counter that moved by one per *poll*
 * would tell a person their receiver was running at the polling rate.
 */
const DEMO_RC_FRAME_MS = 20;

/** The firmware's channel count (`AK_RC_CHANNELS` in `ak_types.h`). */
const DEMO_RC_CHANNELS = 8;

/** The firmware's default calibration (`ak_rc_default_config`), which is what
 *  the demo board decodes against. A real board's may have been moved by
 *  `calibrate rc`; this one has not. */
const DEMO_RC_MIN = 172;
const DEMO_RC_MID = 992;
const DEMO_RC_MAX = 1811;
const DEMO_RC_DEADBAND = 0.02;
const DEMO_RC_ARM_THRESHOLD = 1300;
const DEMO_RC_MODE_THRESHOLD = 1300;

export class DemoBoard {
  private readonly parameters: DemoParameter[];
  /**
   * What `param default` restores, captured once at construction.
   *
   * A copy rather than reading `DemoParameter.default` at reset time, because
   * the two are not the same list: `default` is optional and a parameter that
   * does not state one defaults to *the value this board booted with*, which is
   * a fact about a moment that has passed by the time anybody asks. Reading
   * `parameters[i].value` in the handler would restore "the default" to
   * "whatever it is now", which is a reset that resets nothing — the exact
   * shape of bug this file's `default` field exists to avoid.
   */
  private readonly defaults: readonly string[];
  private readonly product: string;
  private readonly canSave: boolean;
  private readonly logCounts: readonly (number | null)[];
  /** `null` is a real setting here: it makes this board answer a `hello` that
   *  ends before the capability word, which is what an old board sends. */
  private readonly features: number | null;
  private readonly fixedHash: number | null;
  private readonly receiver: 'crsf' | 'sbus' | null;
  /** Which topics this board has no sensor for. Absent means fitted. */
  private readonly sensors: Partial<Record<Sensor, boolean>>;
  /** When this board's receiver stopped framing, or `null` while it is. */
  private rcLostAtMs: number | null = null;
  private readonly decoder = new FrameDecoder();
  private readonly started = Date.now();

  /** Flight state as the wire numbers it: 0 disarmed, 1 armed. */
  private flightState = 0;
  private telemetryHz = 0;
  private nextTelemetryAt = 0;
  private logSource = 0;
  private changed = 0;

  readonly stats = { commands: 0, refused: 0, unknown: 0 };

  constructor(options: DemoBoardOptions = {}) {
    this.parameters = [...(options.parameters ?? [])];
    this.defaults = this.parameters.map((item) => item.default ?? item.value);
    this.product = options.product ?? 'aerialkit-demo';
    this.canSave = options.canSave ?? true;
    this.logCounts = options.logCounts ?? DEFAULT_LOG_COUNTS;
    // `=== undefined`, not `??`: `null` means "answer as a board that predates
    // the capability word", and a null-coalescing default would quietly turn
    // that back into a board that reports one.
    this.features = options.features === undefined ? DEMO_FEATURES : options.features;
    this.fixedHash = options.configHash ?? null;
    this.receiver = options.receiver === undefined ? 'crsf' : options.receiver;
    this.sensors = options.sensors ?? {};
  }

  /** The hash this board reports. Computed from the *live* table, as the
   *  firmware computes it: a `param set` moves it, which is what makes it a
   *  statement about the configuration rather than about the build. */
  get configHash(): number {
    return this.fixedHash ?? paramsHash(this.parameters.map((p) => ({ name: p.name, value: p.value })));
  }

  get isArmed(): boolean {
    return this.flightState === 1;
  }

  setArmed(armed: boolean): void {
    this.flightState = armed ? 1 : 0;
  }

  /**
   * What this board holds for a row, or `null` for an index it does not have.
   *
   * A read accessor rather than a test reaching into `parameters`, because the
   * question a test asks — "did that write land, and did the reset move it
   * back" — is the same question `param get` answers over the wire, and it
   * should be answerable without building a frame. It is genuinely read-only:
   * a caller that wants to change a value has to send the command, which is
   * what keeps the protocol the only way in.
   */
  parameterValue(index: number): string | null {
    return this.parameters[index]?.value ?? null;
  }

  get parameterCount(): number {
    return this.parameters.length;
  }

  /** How many writes have landed since the last save. `hello` reports it, which
   *  is how a client knows there is something unsaved to lose. */
  get changedCount(): number {
    return this.changed;
  }

  /** Bytes to send back, if the board has anything to say. */
  feed(chunk: Uint8Array): Uint8Array | null {
    const { frames } = this.decoder.push(chunk);
    let out: Uint8Array | null = null;
    for (const frame of frames) {
      const reply = this.answer(frame);
      if (reply === null) continue;
      out = out === null ? reply : concat(out, reply);
    }
    return out;
  }

  /** A telemetry frame if one is due. The board decides, and a rate of zero
   *  means never. */
  poll(nowMs: number): Uint8Array | null {
    if (this.telemetryHz === 0 || nowMs < this.nextTelemetryAt) return null;
    this.nextTelemetryAt = nowMs + Math.round(1000 / this.telemetryHz);
    return buildFrame(Command.TELEMETRY, this.telemetryBody_(nowMs - this.started));
  }

  /**
   * The pushed frame: a status body with an uptime in front of it, which is
   * what `ak_proto_telemetry_frame()` builds (ak_proto.c:145). The command byte
   * carries *no* response bit, because nothing asked for this frame — that is
   * the whole mechanism by which a client tells a stream from its own replies.
   */
  private telemetryBody_(uptimeMs: number): Uint8Array {
    const body = this.statusBody(uptimeMs);
    const out = new Uint8Array(4 + body.length);
    new DataView(out.buffer).setInt32(0, Math.round(uptimeMs), true);
    out.set(body, 4);
    return out;
  }

  private answer(frame: Frame): Uint8Array | null {
    // A request carries the bare command and a reply carries it with the top
    // bit set. A board does not answer a reply.
    if (isResponse(frame.command)) return null;

    const command = (frame.command & 0x7f) as Command;
    this.stats.commands++;

    switch (command) {
      case Command.HELLO:
        return this.reply(command, this.helloBody());
      case Command.PARAM_GET:
        return this.reply(command, this.paramGetBody(frame.payload));
      case Command.PARAM_SET:
        return this.reply(command, this.paramSetBody(frame.payload));
      case Command.PARAM_SAVE:
        return this.reply(command, this.paramSaveBody());
      case Command.STATUS:
        return this.reply(command, this.statusBody(Date.now() - this.started));
      case Command.LOG_INFO:
        return this.reply(command, this.logInfoBody());
      case Command.LOG_GET:
        return this.reply(command, this.logGetBody(frame.payload));
      case Command.LOG_SOURCE:
        return this.reply(command, this.logSourceBody(frame.payload));
      case Command.TELEMETRY:
        return this.reply(command, this.telemetryBody(frame.payload));
      case Command.PARAM_INFO:
        if (!this.has(Feature.PARAM_INFO)) break;
        return this.reply(command, this.paramInfoBody(frame.payload));
      case Command.PARAM_HELP:
        if (!this.has(Feature.PARAM_INFO)) break;
        return this.reply(command, this.paramHelpBody(frame.payload));
      case Command.PARAM_DEFAULT:
        if (!this.has(Feature.PARAM_DEFAULT)) break;
        return this.reply(command, this.paramDefaultBody(frame.payload));
      case Command.RC_CHANNELS:
        if (!this.has(Feature.RC_CHANNELS)) break;
        return this.reply(command, this.rcBody());
      case Command.SENSOR_INFO:
        if (!this.has(Feature.SENSOR_INFO)) break;
        return this.reply(command, this.sensorBody(frame.payload));
      default:
        break;
    }

    // The firmware's `default:` case, and the reason it is worth modelling
    // rather than returning nothing: a board that does not implement a command
    // still *answers*. The reply carries this board's one byte, correlated to
    // the request's own command byte, so an old board costs a client one round
    // trip and produces a sentence. Returning null here would instead model a
    // two-second timeout — a different failure with a different cause, and the
    // one the app would then have to guess about.
    this.stats.unknown++;
    return this.reply(command, new Uint8Array([UNKNOWN_COMMAND]));
  }

  /** Whether this board's capability word covers a feature. `null` — a board
   *  that predates the word — covers nothing, which is what makes it the "old
   *  firmware" peer rather than a board with a word and no bits. */
  private has(feature: Feature): boolean {
    return this.features !== null && (this.features & feature) !== 0;
  }

  private reply(command: Command, payload: Uint8Array): Uint8Array {
    // The response bit is the whole correlation scheme, so it is set in one
    // place and never by a caller.
    return buildFrame(command | 0x80, payload);
  }

  // ---- bodies -------------------------------------------------------------

  private helloBody(): Uint8Array {
    // version, product, count, changed — the order and the *types* the firmware
    // writes (ak_proto.c:188). The count is a u16 and the changed count is a
    // NUL-terminated string, because the firmware formats it with
    // ak_format_uint rather than storing it as a number. Writing it as two raw
    // bytes here would produce a reply the firmware never sends and a parser
    // that reads the wrong thing — which is what happened the first time.
    const name = cstring(this.product);
    const changed = cstring(String(this.changed));
    // The capability word and the hash are appended *after* `changed`, never
    // interleaved — the same layout ak_proto.c writes, and the reason the
    // firmware could add them without moving AK_PROTO_VERSION.
    const tail = this.features === null ? 0 : 8;
    const out = new Uint8Array(1 + name.length + 2 + changed.length + tail);
    out[0] = 1;
    out.set(name, 1);
    const at = 1 + name.length;
    out[at] = this.parameters.length & 0xff;
    out[at + 1] = (this.parameters.length >> 8) & 0xff;
    out.set(changed, at + 2);
    if (this.features === null) {
      // A board that predates the field: the reply ends here, and the app must
      // report `null` — "this firmware predates the capability word" — rather
      // than zero, which would be the different claim "no capabilities".
      return out;
    }
    const end = at + 2 + changed.length;
    writeU32le(out, end, this.features);
    writeU32le(out, end + 4, this.configHash);
    return out;
  }

  private paramGetBody(args: Uint8Array): Uint8Array {
    const index = args.length >= 1 ? args[0]! : 0xff;
    const item = this.parameters[index];
    if (item === undefined) return new Uint8Array([1]);
    return concat(concat(new Uint8Array([0]), cstring(item.name)), cstring(item.value));
  }

  /**
   * The armed refusal, or `null` when this board may be written to.
   *
   * **Conditional on the capability bit, and that is the file's own rule rather
   * than a convenience.** A board's gate is a fact about the build, and
   * `GATES_ON_ARMED` is that build's statement of it — so a simulator asked to
   * model a board that does not report the bit has to model a board without the
   * gate, or the app's "this board does not guard its writes" path becomes
   * untestable against a real peer. The alternative is a demo board that
   * guards unconditionally and a limitation entry nothing can produce, which is
   * a reading with only one possible value.
   *
   * It is checked before the index on every route, because that ordering is a
   * decision about a person rather than about the protocol: a client told "no
   * such parameter" while armed would reasonably conclude that naming a real
   * one would have worked.
   */
  private armedRefusal(): Uint8Array | null {
    if (!this.has(Feature.GATES_ON_ARMED) || !this.isArmed) return null;
    this.stats.refused++;
    return refusal(SetStatus.REFUSED_ARMED, 'refused: the aircraft is armed');
  }

  private paramSetBody(args: Uint8Array): Uint8Array {
    const armed = this.armedRefusal();
    if (armed !== null) return armed;

    if (args.length < 2 || args[0]! >= this.parameters.length) {
      this.stats.refused++;
      return refusal(SetStatus.NO_SUCH_PARAMETER, 'no such parameter');
    }

    const index = args[0]!;
    const item = this.parameters[index]!;
    const text = new TextDecoder().decode(args.subarray(1));

    // The armed check is above, in `armedRefusal()`, and it used to live here.
    // Until milestone 4 it was *ahead* of the firmware rather than with it — the
    // comment in this spot said so, because `ak_proto.c` checked the parameter
    // index and not the flight state, and only the save path was gated. The
    // firmware now consults `ak_proto_io_t.writable` at the moment of every
    // request and `main.c` wires it to `ak_flight_config_writable()`, so this is
    // the same rule in both places rather than a simulator modelling the
    // firmware as it ought to be.
    //
    // It remains unreachable through the app: the session refuses a write on the
    // near side while the aircraft is armed or its state is unknown, so these
    // bytes never arrive from a click. It is reachable from a test, which is
    // what makes this board the peer that proves the far-side gate exists — that
    // a client-side refusal is not the only thing standing between an armed
    // aircraft and a write.
    const problem = check(item, text);
    if (problem !== null) {
      this.stats.refused++;
      return refusal(SetStatus.VALUE_REFUSED, problem);
    }

    this.parameters[index] = { ...item, value: format(item, text) };
    this.changed++;
    return refusal(SetStatus.OK, '');
  }

  private paramSaveBody(): Uint8Array {
    const armed = this.armedRefusal();
    if (armed !== null) return armed;
    if (!this.canSave) return new Uint8Array([SetStatus.NOWHERE_TO_SAVE]);
    this.changed = 0;
    return new Uint8Array([SetStatus.OK]);
  }

  /**
   * Put parameters back to the values this build started with.
   *
   * `mode` 1 is one row, named by the index that follows; 2 is the whole table.
   * **A bare request is refused**, and that is the one design decision in this
   * handler worth defending: an empty frame is what a truncated one looks like,
   * and "the frame lost its payload" must not be the same bytes as "wipe every
   * parameter". There is no default in the other direction either — mode 0 is
   * not "all", it is a request that named nothing.
   *
   * The order is the firmware's: the gate, then what was asked, then the
   * argument. Checking the index before the mode would answer "no such
   * parameter" to a frame that never named a parameter.
   */
  private paramDefaultBody(args: Uint8Array): Uint8Array {
    const armed = this.armedRefusal();
    if (armed !== null) return armed;
    if (args.length < 1) {
      this.stats.refused++;
      return refusal(SetStatus.VALUE_REFUSED, 'name what to reset: 1 <index>, or 2 for all');
    }

    const mode = args[0]!;
    if (mode === 1) {
      if (args.length < 2 || args[1]! >= this.parameters.length) {
        this.stats.refused++;
        return refusal(SetStatus.NO_SUCH_PARAMETER, 'no such parameter');
      }
      const index = args[1]!;
      this.parameters[index] = { ...this.parameters[index]!, value: this.defaults[index]! };
      this.changed++;
      return refusal(SetStatus.OK, '');
    }
    if (mode === 2) {
      for (let i = 0; i < this.parameters.length; i++) {
        this.parameters[i] = { ...this.parameters[i]!, value: this.defaults[i]! };
      }
      this.changed++;
      return refusal(SetStatus.OK, '');
    }
    this.stats.refused++;
    return refusal(SetStatus.VALUE_REFUSED, 'mode is 1 (one) or 2 (all)');
  }

  /**
   * One page of this board's description of its own table.
   *
   * Built by measuring an entry before writing it, exactly as `ak_proto.c` does,
   * so a page that runs out of room stops *between* entries and reports how many
   * it wrote. Appending and hoping would cut an entry in half — and half an
   * entry reads as the next entry's bytes, which is a right value against a
   * wrong name.
   */
  private paramInfoBody(args: Uint8Array): Uint8Array {
    if (args.length < 1) {
      // The request named no index. A page built from an assumed zero would be
      // indistinguishable from a complete table, so the protocol answers this
      // shape with one byte and no header, and this is it.
      return new Uint8Array([InfoStatus.NO_INDEX]);
    }

    const first = args[0]!;
    const entries: Uint8Array[] = [];
    let size = 3; // status, first, carried
    let status: InfoStatus = InfoStatus.OK;

    for (let index = first; index < this.parameters.length; index++) {
      const entry = infoEntry(this.parameters[index]!);
      if (size + entry.length > MAX_PAYLOAD) {
        // A page that ran out of room is fine. A *single* entry that cannot fit
        // one frame is a different answer and has to be said out loud: a client
        // that reads `carried == 0` as "the end" would stop early and never
        // learn the entry exists, and one that re-asks from the same index would
        // ask forever. There is no third answer, because an entry cannot be
        // split.
        if (entries.length === 0) status = InfoStatus.TOO_BIG;
        break;
      }
      entries.push(entry);
      size += entry.length;
    }

    return concatAll([new Uint8Array([status, first, entries.length]), ...entries]);
  }

  /**
   * One slice of a row's help text, walked by byte offset.
   *
   * Walked rather than paged, which is what makes "the text was longer than one
   * frame" an ordinary case rather than a failure: the client is done when it
   * has `total` bytes, and an offset past the end is an empty tail.
   */
  private paramHelpBody(args: Uint8Array): Uint8Array {
    if (args.length < 3 || args[0]! >= this.parameters.length) {
      // As the firmware does: a request this shape cannot be answered is
      // refused with a status, not answered as an empty string. Empty and
      // absent are different facts and the app renders them differently.
      return new Uint8Array([1]);
    }
    const index = args[0]!;
    const bytes = new TextEncoder().encode(this.parameters[index]!.help ?? '');
    const total = bytes.length;

    let offset = args[1]! | (args[2]! << 8);
    if (offset > total) offset = total;

    // Seven header bytes, so the room for text is what the header does not use.
    const room = MAX_PAYLOAD - 7;
    let part = total - offset;
    if (part > room) part = room;
    if (part > 0xff) part = 0xff;

    return concatAll([
      new Uint8Array([
        0,
        index,
        offset & 0xff,
        (offset >> 8) & 0xff,
        total & 0xff,
        (total >> 8) & 0xff,
        part,
      ]),
      bytes.subarray(offset, offset + part),
    ]);
  }

  /**
   * The receiver, as this board sees it.
   *
   * **This function is standing in for the firmware, not for the app**, and the
   * difference matters for what it is allowed to do. It decodes the sticks from
   * its own raw counts against its own calibration — which is exactly what
   * `ak_rc_decode()` does in `ak_rc_receiver.c` and exactly what the *app* is
   * forbidden to do. A client that recomputed the sticks would be a second
   * implementation of the firmware's `centred()` in another language, and the
   * two would disagree at the deadband edge with the screen and the airframe
   * each believing its own. `mirrorCentred` below exists here for that reason
   * and this file states so; nothing under `src/session` or `src/ui` calls it.
   *
   * The board is *disarmed* on boot like every other part of this simulator,
   * and its arm channel follows its own flight state rather than being an
   * independent knob. A demo that could show the switch high while reporting
   * disarmed would be modelling a state the firmware does not sit in.
   */
  private rcBody(): Uint8Array {
    if (this.receiver === null) {
      // One byte and nothing after it: this board has no receiver port. Every
      // field the other reply carries would be a claim about hardware this
      // board does not have, and a frame of zeros would read as a receiver with
      // a dead link — a different fault with a different fix.
      return new Uint8Array([RcStatus.NONE]);
    }

    const frames = this.rcFrames();
    const out = new Uint8Array(4 + DEMO_RC_CHANNELS * 2 + 8 + 1 + 28);
    let at = 0;

    if (this.rcLostAtMs !== null) {
      // A receiver that has stopped framing. The sticks stay zero and the
      // DECODED bit stays clear, which is the whole point of that bit: an
      // all-zero stick reading from a board that has no idea where the sticks
      // are is not the same byte sequence's meaning as a handset sitting
      // centred, and only the flag can tell a person which one they have.
      out[at++] = RcStatus.OK;
      out[at++] = 0;
      out[at++] = this.receiver === 'crsf' ? 0 : 1;
      out[at++] = DEMO_RC_CHANNELS;
      at += DEMO_RC_CHANNELS * 2;
      at += 8;
      out[at++] = 0;
      this.writeRcCounters(out, at, frames);
      return out;
    }

    const raw = this.rcRawCounts(frames);
    const flags =
      RcFlag.LINK |
      RcFlag.DECODED |
      (this.receiver === 'crsf' ? RcFlag.TELEMETRY : 0) |
      (this.receiver === 'sbus' ? RcFlag.NO_INVERTER : 0);

    out[at++] = RcStatus.OK;
    out[at++] = flags;
    out[at++] = this.receiver === 'crsf' ? 0 : 1;
    out[at++] = DEMO_RC_CHANNELS;
    for (const count of raw) {
      out[at++] = count & 0xff;
      out[at++] = (count >> 8) & 0xff;
    }

    const decoded = this.rcDecoded(raw);
    for (const value of decoded.sticks) {
      // Per-mille, and the sign is why this is a signed write rather than a
      // magnitude and a direction. The firmware multiplies by 1000 and casts;
      // a value of -1000 has to survive as one.
      const scaled = Math.round(value * 1000);
      out[at++] = scaled & 0xff;
      out[at++] = (scaled >> 8) & 0xff;
    }

    let switches = 0;
    if (decoded.armRequest) switches |= RcSwitch.ARM_ON;
    if (decoded.angleMode) switches |= RcSwitch.ANGLE;
    out[at++] = switches;
    this.writeRcCounters(out, at, frames);
    return out;
  }

  /**
   * The seven counters, in the firmware's own order and at the offset the
   * header and channels left off at.
   *
   * Written one call each rather than as a run of zero bytes, so that the
   * sequence here can be read against `case AK_PROTO_CMD_RC_CHANNELS` in
   * `ak_proto.c` line by line. A single `at += 20` would be one byte short of
   * the five counters it stood for and would silently write off the end of a
   * `Uint8Array`, which is the kind of defect that shows up as a client reading
   * one field too far rather than as a failure.
   */
  private writeRcCounters(out: Uint8Array, at: number, frames: number): void {
    const bytes = frames * (this.receiver === 'crsf' ? 26 : 25);
    const counters = [
      bytes,
      frames,
      0, // crc errors: a simulated receiver's frames are all well-formed
      0, // rejected
      0, // lost, SBUS only
      0, // failsafe frames, SBUS only
      0, // dropped by the UART's receive buffer, which this board does not have
    ];
    for (const value of counters) {
      writeU32le(out, at, value);
      at += 4;
    }
  }

  /** Frames this board's receiver has delivered, from elapsed time rather than
   *  from how often it has been asked. Frozen at the moment it stopped. */
  private rcFrames(): number {
    const since = (this.rcLostAtMs ?? Date.now()) - this.started;
    return Math.max(0, Math.floor(since / DEMO_RC_FRAME_MS));
  }

  /**
   * The counts a receiver would be sending, as a function of time.
   *
   * Synthetic and regular on purpose, like the log records: a sweep a person
   * can see move, in the count range CRSF and SBUS actually use (172..1811 with
   * 992 in the middle, which is what the flight core is calibrated for). Throttle
   * sits at the bottom because a throttle stick at rest is at the bottom, and
   * the arm channel follows this board's own flight state.
   */
  private rcRawCounts(frames: number): number[] {
    const phase = frames / 50; // one sweep every few seconds
    const swing = 420;
    const raw = new Array<number>(DEMO_RC_CHANNELS).fill(DEMO_RC_MID);
    raw[0] = Math.round(DEMO_RC_MID + Math.sin(phase) * swing);
    raw[1] = Math.round(DEMO_RC_MID + Math.cos(phase) * swing);
    raw[2] = DEMO_RC_MIN; // throttle, at the bottom
    raw[3] = Math.round(DEMO_RC_MID + Math.sin(phase / 3) * swing);
    raw[4] = DEMO_RC_MIN; // mode channel low: rate mode, not angle
    raw[5] = this.isArmed ? DEMO_RC_MAX : DEMO_RC_MIN; // the arm channel
    return raw;
  }

  /** What this board's own decoder makes of those counts. */
  private rcDecoded(raw: readonly number[]): {
    sticks: readonly number[];
    armRequest: boolean;
    angleMode: boolean;
  } {
    const sticks = [
      mirrorCentred(raw[0]!, DEMO_RC_DEADBAND),
      mirrorCentred(raw[1]!, DEMO_RC_DEADBAND),
      mirrorCentred(raw[3]!, DEMO_RC_DEADBAND),
      // Throttle is bottom-to-top, not centre-out, and the firmware maps it
      // against min..max rather than mid..max. Getting this wrong is the
      // difference between "the throttle bar is at zero" and "the throttle bar
      // is at halfway" on a board nobody has touched.
      clamp01((raw[2]! - DEMO_RC_MIN) / (DEMO_RC_MAX - DEMO_RC_MIN)),
    ];
    return {
      sticks,
      armRequest: raw[5]! > DEMO_RC_ARM_THRESHOLD,
      angleMode: raw[4]! > DEMO_RC_MODE_THRESHOLD,
    };
  }

  /**
   * Stop or resume this board's receiver framing.
   *
   * Off is a receiver that is powered but not heard — the state a person is in
   * when they have bound the wrong model or left the transmitter off, and the
   * one where "all zeros" and "centred" must not look alike on screen.
   */
  setReceiverAlive(alive: boolean): void {
    this.rcLostAtMs = alive ? null : Date.now();
  }

  /**
   * One sensor's body, or the two-or-three-byte answer that stands in its place.
   *
   * **Three answers, and this method produces all three**, because a simulator
   * that only ever produced the third would leave the app's two other paths
   * tested against nothing but hand-built fixtures:
   *
   *  - a topic this app has no name for, or one past the five — `NO_SUCH, topic`
   *    and three bytes, which is what a firmware that never heard of the topic
   *    says;
   *  - a topic this board genuinely does not have — `OK, topic, present: 0` and
   *    still nothing after it. `sensors: { gps: false }` is how a test asks for
   *    it, and it is the case the whole opcode is shaped around: an absent
   *    sensor carries *no body*, so that "not fitted" and "fitted and reading
   *    zero" cannot be the same bytes;
   *  - the reading.
   *
   * The counters are derived from elapsed time rather than incremented per
   * poll, exactly as the receiver's are and for the same reason: this board is
   * polled at whatever rate the app polls at, and a counter that moved by one
   * per poll would tell a person their IMU was running at the UI's refresh rate.
   */
  private sensorBody(payload: Uint8Array): Uint8Array {
    const topic = payload.length > 0 ? payload[0]! : 0xff;
    const seconds = (Date.now() - this.started) / 1000;

    // A topic this board has no sensor for, in the firmware's own terms. The
    // firmware refuses a topic outside its enum *and* a device with no driver
    // for a topic it does answer — the first is a fact about the build and
    // arrives here, and `present: 0` below is the second. The comparison is the
    // firmware's own (`topic >= AK_PROTO_SENSOR_TOPICS`), which is why this is
    // a range check and not a membership test: a *hole* in the middle of the
    // numbering would be a topic the build claims and has no body for.
    const known = topic < SENSOR_TOPICS;
    if (!known) return new Uint8Array([SensorStatus.NO_SUCH, topic & 0xff, 0]);

    if (this.sensors[topic as Sensor] === false) {
      return new Uint8Array([SensorStatus.OK, topic & 0xff, 0]);
    }

    const body = new Body();
    switch (topic) {
      case Sensor.IMU:
        body.name('icm42688p');
        body.u8(0); // no absence: this board's IMU came up
        body.u8(DEMO_IMU_WHOAMI);
        // Per-mille of g, and standing still is one g on z. A little wobble so
        // the tab shows something moving that is not a lie about the aircraft:
        // this board is on a bench and being nudged.
        body.i16(Math.round(Math.sin(seconds) * 12));
        body.i16(Math.round(Math.cos(seconds * 1.3) * 12));
        body.i16(1000);
        for (let axis = 0; axis < 3; axis++) {
          body.i16(Math.round(Math.sin(seconds * (0.7 + axis * 0.2)) * 4));
        }
        // Alignment and bias at zero: this board has never been calibrated and
        // every `align_*_deg` is zero, which is what the demo table says too.
        for (let i = 0; i < 6; i++) body.i16(0);
        body.u32(Math.round(seconds * DEMO_IMU_HZ));
        body.u32(0);
        break;
      case Sensor.BARO:
        body.name('bmp388');
        body.i32(Math.round(DEMO_BARO_SEA_PA + Math.sin(seconds / 20) * 40));
        body.i16(2150); // 21.50 C, the bench temperature
        body.u8(1);
        body.i32(DEMO_BARO_SEA_PA);
        body.i32(Math.round(Math.sin(seconds / 20) * 34));
        body.u8(0); // no GPS reference: this board is not flying
        body.i32(Math.round(Math.sin(seconds / 20) * 34));
        body.u32(Math.round(seconds * 25));
        body.u32(0);
        body.u32(0);
        body.u32(Math.round(seconds * 25));
        body.u32(Math.round(seconds * 5));
        break;
      case Sensor.RANGE:
        body.name('tof10120');
        body.u8(0x52); // the address its driver claims
        body.u16(4000);
        // A bench reading that sweeps, and negative for "nothing there" on the
        // way back — the two have to be drawn differently and this is the only
        // place a demo can show that.
        {
          const sweep = (seconds * 30) % 200;
          body.i32(sweep < 150 ? Math.round(180 + sweep * 12) : -1);
        }
        body.u32(3);
        body.u32(Math.round(seconds * 50));
        body.u32(Math.round(seconds * 50 * 0.2));
        body.u32(0);
        body.u32(0);
        body.u32(0);
        body.u32(180); // the height it measured while standing
        body.u16(2);
        break;
      case Sensor.BATTERY:
        // A 4S pack, sagging very slowly. Cell volts are what decides anything,
        // and they are written in millivolts like the firmware's.
        {
          const cellMv = Math.round(3820 - Math.sin(seconds / 60) * 40);
          body.u8(1); // ready
          body.u8(1); // a reading came back
          body.u8(cellMv > DEMO_BATTERY_WARN_MV ? 1 : 2);
          body.u8(4); // cells
          body.u16(cellMv * 4);
          body.u16(cellMv);
          body.i16(DEMO_BATTERY_PIN_MV);
          body.u16(Math.round(DEMO_BATTERY_RATIO * 1000));
          body.u8(0); // not returning
          body.u16(DEMO_BATTERY_WARN_MV);
          body.u16(DEMO_BATTERY_CRITICAL_MV);
          body.u32(Math.round(seconds * 10));
          body.u32(0);
          body.u32(Math.round(seconds * 10));
        }
        break;
      case Sensor.GPS:
        // **The same position `statusBody` reports**, deliberately: two panels
        // of one app disagreeing about where the aircraft is would be the app
        // contradicting itself, and the Sensors tab is not a second opinion.
        body.u8(1); // have fix
        body.u8(3); // a 3D fix
        body.u8(1); // fix ok
        body.u8(11); // satellites
        body.u8(1); // valid now
        body.i32(Math.round(DEMO_LAT_E7));
        body.i32(Math.round(DEMO_LON_E7));
        body.i32(35_000); // 35 m above the ellipsoid
        body.i32(0); // not moving
        body.i32(0); // course, undefined at rest
        // Home is where it is, because this board has never been anywhere. A
        // distance of 0 is the honest number for an aircraft that has not taken
        // off; inventing a home a hundred metres away would put a marker on a
        // map for a flight that did not happen.
        body.u8(1);
        body.i32(Math.round(DEMO_LAT_E7));
        body.i32(Math.round(DEMO_LON_E7));
        body.i32(0);
        body.i32(0);
        body.u8(0); // not returning
        body.u8(0); // RTH not enabled
        body.u32(Math.round(seconds)); // one fix a second
        body.u32(0);
        body.u32(Math.round(seconds / 5));
        break;
      default:
        // Unreachable: `known` above covers exactly these five.
        return new Uint8Array([SensorStatus.NO_SUCH, topic & 0xff, 0]);
    }

    // **The header goes on here, and only here.** The two refusals above are
    // whole replies of three bytes; this arm is the one that carries a body, and
    // the `present` byte is what tells a client the bytes after it are a reading
    // rather than the next field. Assembling it in the same expression as the
    // body is deliberate: a header written earlier and a body returned later is
    // how one of the two gets forgotten, and forgetting this one produces a
    // reply that reads as a status byte made of the driver name's first letter.
    return concat(new Uint8Array([SensorStatus.OK, topic & 0xff, 1]), body.done());
  }

  private logInfoBody(): Uint8Array {
    const count = this.countFor(this.logSource);
    return new Uint8Array([count & 0xff, (count >> 8) & 0xff]);
  }

  private logGetBody(args: Uint8Array): Uint8Array {
    const index = args.length >= 2 ? args[0]! | (args[1]! << 8) : 0xffff;
    if (index >= this.countFor(this.logSource)) return new Uint8Array([1]);
    return concat(new Uint8Array([0]), this.logRecord(index));
  }

  private logSourceBody(args: Uint8Array): Uint8Array {
    const wanted = args.length >= 1 ? args[0]! : 0;
    // A device without a log says it does not have it, and says which source is
    // still selected. Answering "zero records" would be a different statement
    // and a worse one: it reads as an empty log rather than an absent one.
    if (wanted >= this.logCounts.length || this.logCounts[wanted] === null) {
      return new Uint8Array([1, this.logSource, 0, 0]);
    }
    this.logSource = wanted;
    const count = this.countFor(wanted);
    return new Uint8Array([0, wanted, count & 0xff, (count >> 8) & 0xff]);
  }

  private telemetryBody(args: Uint8Array): Uint8Array {
    const wanted = Math.min(args.length >= 1 ? args[0]! : 0, TELEMETRY_MAX_HZ);
    this.telemetryHz = wanted;
    this.nextTelemetryAt = 0;
    return new Uint8Array([wanted]);
  }

  private countFor(source: number): number {
    const count = this.logCounts[source];
    return typeof count === 'number' ? count : 0;
  }

  private logRecord(index: number): Uint8Array {
    const body = new Uint8Array(LOG_RECORD_BYTES);
    const view = new DataView(body.buffer);
    // Synthetic and regular on purpose: a record that looked like a real flight
    // would be a record someone could mistake for one.
    const phase = index / 24;
    view.setUint32(0, index * 20, true); // 50 Hz, the fast log's rate
    view.setInt16(4, Math.round(Math.sin(phase) * 120), true);
    view.setInt16(6, Math.round(Math.cos(phase) * 90), true);
    view.setInt16(8, Math.round(Math.sin(phase / 3) * 40), true);
    view.setInt16(10, Math.round(Math.sin(phase / 2) * 30), true);
    view.setInt16(12, Math.round(Math.cos(phase / 2) * 20), true);
    view.setInt16(14, 1000, true);
    view.setInt16(16, Math.round(Math.sin(phase) * 150), true);
    view.setInt16(18, Math.round(Math.cos(phase) * 110), true);
    view.setInt16(20, 0, true);
    view.setInt32(22, Math.round(Math.sin(phase / 8) * 12000), true);
    view.setInt16(26, 1500, true);
    view.setInt16(28, 1500, true);
    view.setInt16(30, 1500, true);
    view.setInt16(32, index < 40 ? 1000 : 1600, true);
    body[34] = Math.round(Math.sin(phase) * 100) & 0xff;
    body[35] = Math.round(Math.cos(phase) * 100) & 0xff;
    body[36] = 0;
    for (let motor = 0; motor < 4; motor++) {
      body[37 + motor] = index < 40 ? 0 : 120 + motor * 4;
    }
    body[41] = index < 40 ? 0 : 1;
    body[42] = 0;
    view.setInt32(43, Math.round(51.5e7), true);
    view.setInt32(47, Math.round(-0.12e7), true);
    return body;
  }

  private statusBody(uptimeMs: number): Uint8Array {
    const out = new Uint8Array(22);
    const view = new DataView(out.buffer);
    const seconds = uptimeMs / 1000;
    out[0] = this.flightState;
    out[1] = 1; // the link this frame arrived on is by definition live
    out[2] = 3; // a 3D fix
    out[3] = 11;
    view.setInt16(4, Math.round(Math.sin(seconds / 3) * 80), true);
    view.setInt16(6, Math.round(Math.cos(seconds / 4) * 50), true);
    view.setInt16(8, Math.round(((seconds * 12) % 360) * 10), true);
    view.setInt32(10, Math.round(DEMO_LAT_E7), true);
    view.setInt32(14, Math.round(DEMO_LON_E7), true);
    for (let motor = 0; motor < 4; motor++) {
      out[18 + motor] = this.isArmed ? 128 + motor * 6 : 0;
    }
    return out;
  }
}

/**
 * Whether the table would take this text, and if not, why — in the table's own
 * voice. The firmware's message is `out of range 0.000..2.000`, and this keeps
 * that shape so a person sees the same sentence from the demo, the simulator
 * and a board.
 */
function check(item: DemoParameter, text: string): string | null {
  const type = item.type ?? 'float';
  if (type === 'u32') {
    if (!/^\d+$/.test(text.trim())) return `"${text}" is not a whole number`;
  } else if (!Number.isFinite(Number(text))) {
    return `"${text}" is not a number`;
  }

  const value = Number(text);
  const { min, max } = item;
  if (typeof min === 'number' && value < min) return outOfRange(item);
  if (typeof max === 'number' && value > max) return outOfRange(item);
  return null;
}

function outOfRange(item: DemoParameter): string {
  const decimals = item.decimals ?? 3;
  const min = (item.min ?? 0).toFixed(decimals);
  const max = (item.max ?? 0).toFixed(decimals);
  // The firmware writes `0.000..2.000` with no spaces; matched exactly so the
  // app's tests can compare a demo refusal against a captured one.
  return `out of range ${min}..${max}`;
}

function format(item: DemoParameter, text: string): string {
  const decimals = item.decimals ?? 3;
  if ((item.type ?? 'float') === 'u32') return String(Math.trunc(Number(text)));
  return Number(text).toFixed(decimals);
}

/**
 * One `param info` entry, in the byte order `ak_proto.c` writes it.
 *
 * `type` comes first and the shape of the bounds follows from it, which is why
 * a text parameter and a numeric one can share an entry layout at all: the
 * reader knows which of the two it is holding before it has to know how long the
 * next field is.
 */
function infoEntry(item: DemoParameter): Uint8Array {
  const type = (item.type ?? 'float') === 'u32' ? ParamType.U32 : ParamType.FLOAT;
  const decimals = type === ParamType.U32 ? 0 : (item.decimals ?? 3);
  const parts: Uint8Array[] = [
    cstring(item.name),
    // type, group, decimals, flags. The demo board has no secret parameters and
    // no `AK_PARAM_SEEN` — those are statements a running firmware makes about
    // its own storage, and this board has none.
    new Uint8Array([type, item.group ?? 0, decimals, 0]),
  ];
  // A text parameter would put its `max_len` in the byte a numeric one begins
  // its minimum in; `DemoParameter.type` is only ever `float` or `u32`, so this
  // board never writes that shape. It is named here rather than silently
  // omitted because the layout is the protocol's and a reader comparing this
  // function against `case AK_PROTO_CMD_PARAM_INFO` in `ak_proto.c` should find
  // the difference stated rather than have to work it out.
  parts.push(cstring(boundText(item.min, decimals)));
  parts.push(cstring(boundText(item.max, decimals)));
  parts.push(cstring(item.default ?? item.value));
  return concatAll(parts);
}

/**
 * A bound as the firmware would print it, or the empty string.
 *
 * Empty is the honest answer for a parameter with no bound this file can state:
 * `boundFromText` reads it as `null` and the panel then shows no range, which is
 * a different thing from a range of zero.
 */
function boundText(value: number | null | undefined, decimals: number): string {
  if (typeof value !== 'number' || !Number.isFinite(value)) return '';
  return value.toFixed(decimals);
}

/**
 * A centre-out stick reading, arithmetically identical to the firmware's
 * `centred()` in `ak_rc.c`.
 *
 * **This is the demo board being the firmware, and it is written out here so
 * that no part of the app can be tempted to do it.** The firmware owns the
 * decode because `rc_min`, `rc_mid`, `rc_max` and `rc_deadband` are
 * *parameters*: a board that has been through `calibrate rc` holds different
 * numbers from these, and a client recomputing the sticks from a calibration it
 * read once — or from its own idea of the defaults — would show a person sticks
 * the aircraft is not flying. The reply carries the decode for exactly that
 * reason, and `src/session/aerialkit.ts` only ever renders it.
 *
 * The deadband is the part that bites: the firmware compares the *scaled* value
 * against it and snaps to zero rather than scaling the deadband into counts, so
 * a near-centre count reads as exactly zero here and not as a small number that
 * happens to look like noise.
 */
function mirrorCentred(raw: number, deadband: number): number {
  let value: number;
  if (raw >= DEMO_RC_MID) {
    const span = DEMO_RC_MAX - DEMO_RC_MID;
    value = span > 0 ? (raw - DEMO_RC_MID) / span : 0;
  } else {
    const span = DEMO_RC_MID - DEMO_RC_MIN;
    value = span > 0 ? -((DEMO_RC_MID - raw) / span) : 0;
  }
  value = Math.max(-1, Math.min(1, value));
  return Math.abs(value) < deadband ? 0 : value;
}

function clamp01(value: number): number {
  return Math.max(0, Math.min(1, value));
}

function refusal(status: SetStatus, message: string): Uint8Array {
  return concat(new Uint8Array([status]), cstring(message));
}

function concatAll(parts: readonly Uint8Array[]): Uint8Array {
  const out = new Uint8Array(parts.reduce((n, part) => n + part.length, 0));
  let at = 0;
  for (const part of parts) {
    out.set(part, at);
    at += part.length;
  }
  return out;
}

function cstring(text: string): Uint8Array {
  const body = new TextEncoder().encode(text);
  const out = new Uint8Array(body.length + 1);
  out.set(body, 0);
  return out;
}

/** Little-endian u32 at a known offset. The firmware appends these with
 *  `append_u32le` (`ak_proto.c`); the byte order is a property of the protocol,
 *  not of the host, so it is written out rather than taken from a `DataView`
 *  whose default is big-endian. */
function writeU32le(out: Uint8Array, at: number, value: number): void {
  out[at] = value & 0xff;
  out[at + 1] = (value >>> 8) & 0xff;
  out[at + 2] = (value >>> 16) & 0xff;
  out[at + 3] = (value >>> 24) & 0xff;
}

/**
 * A sensor body under construction.
 *
 * A builder rather than an array and a running offset, because the two mistakes
 * this file can make are both invisible under the offset style: writing a field
 * at the wrong offset overwrites the field before it, and a `Uint8Array` write
 * past the end is *silently dropped* rather than thrown. Under a builder a
 * missing field shortens the body, which the client refuses by length, and
 * there is no place to get an offset wrong.
 *
 * The wording of each method matches the `append_*` function in `ak_proto.c`
 * that produces the same bytes, so a body here can be read against the firmware
 * line by line.
 */
class Body {
  private readonly parts: Uint8Array[] = [];

  private le(bytes: number, value: number, signed: boolean): this {
    const out = new Uint8Array(bytes);
    const view = new DataView(out.buffer);
    const at = 0;
    if (bytes === 2) {
      if (signed) view.setInt16(at, value, true);
      else view.setUint16(at, value & 0xffff, true);
    } else if (signed) {
      view.setInt32(at, value | 0, true);
    } else {
      view.setUint32(at, value >>> 0, true);
    }
    this.parts.push(out);
    return this;
  }

  u8(value: number): this {
    this.parts.push(new Uint8Array([value & 0xff]));
    return this;
  }

  u16(value: number): this {
    return this.le(2, value, false);
  }

  i16(value: number): this {
    return this.le(2, value, true);
  }

  u32(value: number): this {
    return this.le(4, value, false);
  }

  i32(value: number): this {
    return this.le(4, value, true);
  }

  /** `append_name`: exactly `SENSOR_NAME_LENGTH` bytes, NUL-padded, and **cut
   *  rather than refused** if the name is too long — the field has no room to
   *  say "too long" in, which is why the firmware's own `append_name` does the
   *  same. A name that exactly fills the field carries no terminator, which is
   *  a legal name and is why the client reads it fixed-width. */
  name(text: string): this {
    const bytes = new TextEncoder().encode(text).subarray(0, SENSOR_NAME_LENGTH);
    const out = new Uint8Array(SENSOR_NAME_LENGTH);
    out.set(bytes, 0);
    this.parts.push(out);
    return this;
  }

  done(): Uint8Array {
    let total = 0;
    for (const part of this.parts) total += part.length;
    const out = new Uint8Array(total);
    let at = 0;
    for (const part of this.parts) {
      out.set(part, at);
      at += part.length;
    }
    return out;
  }
}

function concat(a: Uint8Array, b: Uint8Array): Uint8Array {
  const out = new Uint8Array(a.length + b.length);
  out.set(a, 0);
  out.set(b, a.length);
  return out;
}
