import {
  CALIBRATE_VERB_NAMES,
  CalibrateVerb,
  Command,
  InfoStatus,
  LOG_STREAM_MAX_HZ,
  MISSION_VERB_NAMES,
  MissionVerb,
  TELEMETRY_MAX_HZ,
} from './constants';
import { buildFrame, baseCommand, FrameDecoder, isResponse, type Frame } from './frame';
import {
  parseCalibration,
  parseHello,
  parseLogInfo,
  parseLogRecord,
  parseLogSource,
  parseLogStreamFrame,
  parseLogStreamReply,
  parseMission,
  parseMotorTelemetry,
  parsePerf,
  parseOutputInfo,
  parseOutputTest,
  parseParamGet,
  parseParamHelpSlice,
  parseParamInfoPage,
  parseParamSet,
  parsePreflightPage,
  parseRcChannels,
  parseSensorInfo,
  parseStatus,
  parseTelemetry,
  ProtocolError,
  type BoardMeta,
  type CalibrationState,
  type Hello,
  type LogRecord,
  type LogSourceReply,
  type LogStreamFrame,
  type LogStreamReply,
  type MissionState,
  type MotorTelemetryState,
  type PerfWindow,
  type OutputList,
  type OutputTestAnswer,
  type OutputTestRequest,
  type ParamInfoPage,
  type ParamSetReply,
  parseParamDefault,
  type PreflightPage,
  type ParameterValue,
  type RcState,
  type SensorAnswer,
  type Status,
  type Telemetry,
} from './messages';

/**
 * One row of the table's own description, at the index it belongs to.
 *
 * `meta` is null only when the board said it could not describe that row, and
 * `unavailable` then says so in the board's terms. There is deliberately no
 * third case where the app supplies something from a file.
 */
export interface ParamDescriptor {
  readonly index: number;
  readonly meta: BoardMeta | null;
  readonly unavailable: string | null;
}

/** A byte pipe. Serial, a socket, a demo loop — the client cannot tell. */
export interface ByteLink {
  write(bytes: Uint8Array): void | Promise<void>;
  /** Returns an unsubscribe function. */
  onData(handler: (chunk: Uint8Array) => void): () => void;
  onClose(handler: (reason: string) => void): () => void;
  onError(handler: (message: string) => void): () => void;
}

export class TimeoutError extends Error {
  constructor(command: string, ms: number) {
    super(`the board did not answer ${command} within ${ms} ms`);
    this.name = 'TimeoutError';
  }
}

export class LinkClosedError extends Error {
  constructor() {
    super('the connection closed before the board answered');
    this.name = 'LinkClosedError';
  }
}

/**
 * A dropped reply is not the same as a corrupt one, and the difference matters
 * to a person: a timeout means the board is busy, absent or not this firmware,
 * while a correlation error means two things are talking past each other.
 */
export class CorrelationError extends Error {
  constructor(
    readonly got: number,
    readonly wanted: number,
  ) {
    super(
      `the board answered command 0x${got.toString(16)}, which is not what was asked ` +
        `(0x${wanted.toString(16)})`,
    );
    this.name = 'CorrelationError';
  }
}

export interface ClientOptions {
  /** How long to wait for a reply. A board on a real UART can be slow to boot. */
  readonly timeoutMs?: number;
  /** Called for every pushed telemetry frame. */
  readonly onTelemetry?: (frame: Telemetry) => void;
  /**
   * Called for every pushed log-stream frame, `DONE` included.
   *
   * The end of a range arrives here rather than through a promise, because it
   * is not the answer to anything: a stream is started once and then runs, and
   * the frame that says it has finished is the last of the pushed frames rather
   * than a reply that was waiting. A client that expected a resolving promise
   * would have to invent a timeout for a stream that ended normally.
   */
  readonly onLogFrame?: (frame: LogStreamFrame) => void;
  /** Called when the link drops, whatever the reason. */
  readonly onClosed?: (reason: string) => void;
  /** Called for frames the decoder refused, so a lossy link is visible rather
   *  than mysterious. */
  readonly onDecodeIssue?: (issue: string) => void;
}

interface Queued {
  readonly command: Command;
  readonly payload: Uint8Array;
  /** Built when the request is made, so a request that cannot be framed (a
   *  value too long for a payload) fails then, not inside pump() after it has
   *  taken the line and a timer - which used to hold every request behind it
   *  for a full timeout and throw into the transport's data handler. */
  readonly frame: Uint8Array;
  readonly resolve: (payload: Uint8Array) => void;
  readonly reject: (error: Error) => void;
  readonly label: string;
}

/**
 * The protocol, from the client's side.
 *
 * Requests are serialized: one in flight at a time, the rest queued. The wire
 * correlates a reply to its request by the command byte and nothing else, so two
 * outstanding requests for the same command could not be told apart. Making that
 * impossible is cheaper than making it usually work.
 *
 * Telemetry frames are the one thing that may arrive unasked. They are routed to
 * `onTelemetry` and never mistaken for a reply — a push carries the command
 * byte *without* the response bit, which is exactly why the firmware spends that
 * bit on it.
 */
export class AerialKitClient {
  private readonly decoder = new FrameDecoder();
  private readonly timeoutMs: number;
  private readonly queue: Queued[] = [];
  private current: Queued | null = null;
  private timer: ReturnType<typeof setTimeout> | null = null;
  /**
   * After a timeout, a quiet moment before the next request goes out, and the
   * command whose reply may still be on its way. Replies carry no sequence
   * number, only the command byte, so a reply that arrived just after its
   * request timed out used to be taken as the answer to whatever was sent
   * next: a `param set` refused by the board recorded as accepted because the
   * previous set's late "OK" answered it, or an unrelated request rejected
   * with a correlation error. A late reply in the window is dropped.
   */
  private settle: ReturnType<typeof setTimeout> | null = null;
  private lateCommand: Command | null = null;
  private closedReason: string | null = null;
  private readonly disposers: Array<() => void> = [];

  /** Diagnostics, counted rather than guessed at. */
  readonly stats = { frames: 0, telemetry: 0, logFrames: 0, issues: 0, writes: 0 };

  constructor(
    private readonly link: ByteLink,
    private readonly options: ClientOptions = {},
  ) {
    this.timeoutMs = options.timeoutMs ?? 2000;
    this.disposers.push(link.onData((chunk) => this.feed(chunk)));
    this.disposers.push(link.onClose((reason) => this.drop(reason)));
    this.disposers.push(
      link.onError((message) => {
        this.options.onDecodeIssue?.(message);
      }),
    );
  }

  close(): void {
    for (const dispose of this.disposers) dispose();
    this.disposers.length = 0;
    this.drop('closed by the client');
  }

  private drop(reason: string): void {
    if (this.closedReason !== null) return;
    this.closedReason = reason;
    if (this.timer !== null) clearTimeout(this.timer);
    this.timer = null;
    if (this.settle !== null) clearTimeout(this.settle);
    this.settle = null;
    this.current?.reject(new LinkClosedError());
    this.current = null;
    while (this.queue.length > 0) this.queue.shift()!.reject(new LinkClosedError());
    this.options.onClosed?.(reason);
  }

  get isClosed(): boolean {
    return this.closedReason !== null;
  }

  get closeReason(): string | null {
    return this.closedReason;
  }

  private feed(chunk: Uint8Array): void {
    if (this.closedReason !== null) return;
    const { frames, issues } = this.decoder.push(chunk);
    for (const issue of issues) {
      this.stats.issues++;
      this.options.onDecodeIssue?.(describeIssue(issue));
    }
    for (const frame of frames) this.dispatch(frame);
  }

  private dispatch(frame: Frame): void {
    this.stats.frames++;
    const command = baseCommand(frame.command);

    if (command === Command.TELEMETRY && !isResponse(frame.command)) {
      this.stats.telemetry++;
      try {
        this.options.onTelemetry?.(parseTelemetry(frame.payload));
      } catch (error) {
        this.stats.issues++;
        this.options.onDecodeIssue?.(String((error as Error).message));
      }
      return;
    }

    if (command === Command.LOG_STREAM && !isResponse(frame.command)) {
      this.stats.logFrames++;
      try {
        this.options.onLogFrame?.(parseLogStreamFrame(frame.payload));
      } catch (error) {
        this.stats.issues++;
        this.options.onDecodeIssue?.(String((error as Error).message));
      }
      return;
    }

    const current = this.current;
    if (current === null && this.lateCommand === command) {
      // The reply to a request that already timed out: it answers nothing
      // that is waiting, so it is dropped rather than misread.
      this.lateCommand = null;
      this.stats.issues++;
      this.options.onDecodeIssue?.(
        `a late reply to 0x${command.toString(16)}, after its request timed out, was dropped`,
      );
      return;
    }
    if (current === null) {
      this.stats.issues++;
      this.options.onDecodeIssue?.(
        `an unasked-for reply to 0x${command.toString(16)} arrived and was dropped`,
      );
      return;
    }
    if (command !== current.command) {
      const error = new CorrelationError(command, current.command);
      this.finish();
      current.reject(error);
      return;
    }
    const { resolve } = current;
    this.finish();
    resolve(frame.payload);
  }

  private finish(): void {
    if (this.timer !== null) clearTimeout(this.timer);
    this.timer = null;
    this.current = null;
    this.pump();
  }

  private pump(): void {
    if (this.current !== null || this.settle !== null || this.closedReason !== null) return;
    const next = this.queue.shift();
    if (next === undefined) return;
    this.current = next;
    this.timer = setTimeout(() => {
      const pending = this.current;
      if (pending !== next) return;
      this.timer = null;
      this.current = null;
      this.lateCommand = next.command;
      // A fifth of the timeout, at most a quarter second: long enough for a
      // reply that was merely slow, short enough not to be felt.
      this.settle = setTimeout(() => {
        this.settle = null;
        this.lateCommand = null;
        this.pump();
      }, Math.min(250, this.timeoutMs / 5));
      next.reject(new TimeoutError(next.label, this.timeoutMs));
    }, this.timeoutMs);
    void Promise.resolve(this.link.write(next.frame)).catch(
      (error: unknown) => {
        if (this.current !== next) return;
        this.finish();
        next.reject(error instanceof Error ? error : new Error(String(error)));
      },
    );
  }

  private request(command: Command, payload: Uint8Array, label: string): Promise<Uint8Array> {
    if (this.closedReason !== null) return Promise.reject(new LinkClosedError());
    this.stats.writes++;
    let frame: Uint8Array;
    try {
      frame = buildFrame(command, payload);
    } catch (error) {
      return Promise.reject(error instanceof Error ? error : new Error(String(error)));
    }
    return new Promise((resolve, reject) => {
      this.queue.push({ command, payload, frame, resolve, reject, label });
      this.pump();
    });
  }

  // ---- the commands -------------------------------------------------------

  async hello(): Promise<Hello> {
    return parseHello(await this.request(Command.HELLO, new Uint8Array(0), 'hello'));
  }

  async status(): Promise<Status> {
    return parseStatus(await this.request(Command.STATUS, new Uint8Array(0), 'status'));
  }

  /**
   * One parameter. `null` means the board said there is no such index — which
   * the enumeration path treats as the end of the table, and the single-read
   * path treats as an error.
   */
  async paramGet(index: number): Promise<ParameterValue | null> {
    if (index < 0 || index > 255) throw new RangeError('parameter index is one byte');
    return parseParamGet(
      await this.request(Command.PARAM_GET, new Uint8Array([index]), `param get ${index}`),
      index,
    );
  }

  /**
   * The whole table, read index by index.
   *
   * `hello` says how many there are, but the loop stops on the first refusal as
   * well, so a board whose count and table disagree — which is exactly what a
   * half-finished firmware upgrade looks like — produces the parameters that
   * exist rather than a hole.
   */
  async paramList(count: number, onProgress?: (done: number, total: number) => void): Promise<ParameterValue[]> {
    const out: ParameterValue[] = [];
    for (let index = 0; index < count; index++) {
      const item = await this.paramGet(index);
      if (item === null) break;
      out.push(item);
      onProgress?.(index + 1, count);
    }
    return out;
  }

  /**
   * One page of the table's description of itself.
   *
   * A board that does not implement the command answers `0x7F`, correlated to
   * the request, and `parseParamInfoPage` raises for it — so asking an old board
   * costs one round trip and produces a sentence, not a two-second timeout.
   */
  async paramInfo(first: number): Promise<ParamInfoPage> {
    if (first < 0 || first > 255) throw new RangeError('parameter index is one byte');
    return parseParamInfoPage(
      await this.request(Command.PARAM_INFO, new Uint8Array([first]), `param info ${first}`),
      first,
    );
  }

  /**
   * The whole table's metadata, walked page by page.
   *
   * The walk ends when the board says the page is empty — `carried == 0` with
   * status 0 — and **steps over** a status-2 row rather than stopping at it. A
   * row that does not fit one frame is a hole to be reported, not the end of
   * the table: reading it as the end is how a 92-parameter board renders as
   * ninety-one and nothing anywhere says so.
   *
   * `count` comes from `hello`, and the loop is bounded by it as well, so a
   * board whose pages never advance cannot spin here.
   */
  async paramInfoList(
    count: number,
    onProgress?: (done: number, total: number) => void,
  ): Promise<ParamDescriptor[]> {
    const out: ParamDescriptor[] = [];
    let index = 0;
    while (index < count) {
      const page = await this.paramInfo(index);
      if (page.status === InfoStatus.NO_INDEX) {
        // The board answered a question this app did not ask, which means it
        // read the frame as something else. Continuing would build a table out
        // of a conversation that is not happening.
        throw new ProtocolError(
          'param info: the board answered "no index was named" to a request that named one',
        );
      }
      if (page.status === InfoStatus.TOO_BIG) {
        out.push({
          index,
          meta: null,
          unavailable: 'the board says this entry is too large to fit one frame',
        });
        index += 1;
        onProgress?.(index, count);
        continue;
      }
      if (page.entries.length === 0) break;
      for (const entry of page.entries) {
        out.push({ index, meta: entry, unavailable: null });
        index += 1;
      }
      onProgress?.(index, count);
    }
    return out;
  }

  /**
   * One row's help text, walked by offset until the whole thing has arrived.
   *
   * Walked rather than paged, which is what makes a help string longer than a
   * frame an ordinary case. The reply echoes the offset it answered from and
   * `parseParamHelpSlice` checks it, so slices cannot be joined at the wrong
   * place — prose joined at a wrong offset reads perfectly.
   */
  async paramHelp(index: number): Promise<string> {
    if (index < 0 || index > 255) throw new RangeError('parameter index is one byte');
    const parts: Uint8Array[] = [];
    let offset = 0;
    let total = 0;
    for (;;) {
      const payload = new Uint8Array(3);
      payload[0] = index;
      payload[1] = offset & 0xff;
      payload[2] = (offset >> 8) & 0xff;
      const slice = parseParamHelpSlice(
        await this.request(Command.PARAM_HELP, payload, `param help ${index}`),
        index,
      );
      if (slice.offset !== offset) {
        throw new ProtocolError(
          `param help ${index}: asked from byte ${offset} and the board answered from ${slice.offset}`,
        );
      }
      total = slice.total;
      if (slice.bytes.length === 0) {
        if (offset < total) {
          throw new ProtocolError(
            `param help ${index}: the board stopped at ${offset} of ${total} with nothing to send`,
          );
        }
        break;
      }
      parts.push(slice.bytes);
      offset += slice.bytes.length;
      if (offset >= total) break;
    }
    const joined = new Uint8Array(parts.reduce((n, part) => n + part.length, 0));
    let at = 0;
    for (const part of parts) {
      joined.set(part, at);
      at += part.length;
    }
    return new TextDecoder().decode(joined);
  }

  /**
   * One page of one line of the board's checklist.
   *
   * The caller names the line *and* the byte to start from, rather than this
   * method walking a line on its own the way `paramHelp` does. That is
   * deliberate and it is about the rebuild: the firmware builds the checklist on
   * an `index` of zero and reads the same record for every other index, so a
   * walk has to keep its first request at offset zero or line twenty would
   * answer from a checklist built for a different question. Exposing the page
   * lets the session own that rule in one place, where the count and the
   * sentences can be kept consistent with each other.
   */
  async preflightPage(index: number, offset: number): Promise<PreflightPage> {
    if (index < 0 || index > 255) throw new RangeError('preflight line is one byte');
    if (offset < 0 || offset > 0xffff) throw new RangeError('preflight offset is two bytes');
    const payload = new Uint8Array(3);
    payload[0] = index;
    payload[1] = offset & 0xff;
    payload[2] = (offset >> 8) & 0xff;
    return parsePreflightPage(
      await this.request(Command.PREFLIGHT, payload, `preflight line ${index}`),
    );
  }

  /**
   * One mission verb, and the aircraft's whole mission state back.
   *
   * **The reply is the state, not an acknowledgement**, and it is the state
   * *after* the verb ran — so a caller that sends `start` reads the effect of its
   * own frame here and needs no second request. See `MissionState` for the two
   * facts (`requested` and `active`) that keep a start that has been asked for
   * apart from a mission that is flying.
   *
   * **A refusal is an answer, not a thrown error.** "There is nothing to fly" and
   * "home needs a usable fix" are things the aircraft says about itself, and they
   * arrive with the same seventeen bytes as an acceptance, so the caller reads
   * them from `status` rather than from a catch. Only three things throw: a link
   * that is closed, a reply that never came, and a frame this decoder refuses.
   *
   * A verb this protocol does not define is refused *here*, without a frame: the
   * firmware would answer it, but the only thing that answer could say is that
   * this app asked for something it already knew was not there.
   */
  async mission(verb: MissionVerb): Promise<MissionState> {
    const name = MISSION_VERB_NAMES[verb];
    if (name === undefined) {
      throw new RangeError(`mission: ${verb} is not a verb this protocol defines`);
    }
    return parseMission(
      await this.request(Command.MISSION, new Uint8Array([verb]), `mission ${name}`),
    );
  }

  /**
   * One calibration verb, and the session's whole state back.
   *
   * **The reply is a reading, not an acknowledgement** — the same shape as
   * `mission`, and here it is what makes a wizard possible: `status` and the
   * verb that started a session are answered from one frame, so polling a
   * running calibration is `calibrate(STATUS)` and the progress arrives in the
   * `samples`/`rejected` counts rather than through a second round trip. The
   * flight loop keeps turning throughout, which is why none of these blocks.
   *
   * **A refusal is an answer, not a thrown error.** `ARMED`, `BUSY`,
   * `NO_SAMPLES` and `IMPLAUSIBLE` are things the aircraft says about itself and
   * arrive with the same thirty-seven bytes as an acceptance, so the caller
   * reads them from `status`. Only three things throw: a closed link, a reply
   * that never came, and a frame this decoder refuses.
   *
   * **The arguments are refused here, without a frame, when this app can already
   * see they are wrong.** A verb outside the six is not a thing the firmware
   * could do anything useful with, and `0xFF` would ask it to calibrate a pack
   * divider against nothing at all — the exact default `ak_proto.c` refuses on
   * the board's side, and there is no reason to spend a frame learning what this
   * file already knows. A face outside `0..5` is left to the board: it is the
   * board's six faces, and `NO_FACE` is its sentence for a seven-sided one.
   */
  async calibrate(verb: CalibrateVerb, face = 0, mv = 0): Promise<CalibrationState> {
    const name = CALIBRATE_VERB_NAMES[verb];
    if (name === undefined) {
      throw new RangeError(`calibrate: ${verb} is not a verb this protocol defines`);
    }
    if (!Number.isInteger(face) || face < 0 || face > 255) {
      throw new RangeError('calibrate: the face is one byte');
    }

    let payload: Uint8Array;
    if (verb === CalibrateVerb.VBAT) {
      if (!Number.isInteger(mv) || mv < 0 || mv > 0xffffffff) {
        throw new RangeError('calibrate: the pack voltage is four bytes of millivolts');
      }
      payload = new Uint8Array(5);
      payload[0] = verb;
      const view = new DataView(payload.buffer);
      view.setUint32(1, mv, true);
    } else if (verb === CalibrateVerb.ACCEL) {
      payload = new Uint8Array([verb, face]);
    } else {
      payload = new Uint8Array([verb]);
    }

    return parseCalibration(
      await this.request(Command.CALIBRATE, payload, `calibrate ${name}`),
    );
  }

  async paramSet(index: number, value: string): Promise<ParamSetReply> {
    const text = new TextEncoder().encode(value);
    const payload = new Uint8Array(1 + text.length);
    payload[0] = index;
    payload.set(text, 1);
    return parseParamSet(
      await this.request(Command.PARAM_SET, payload, `param set ${index}`),
    );
  }

  async paramSave(): Promise<ParamSetReply> {
    return parseParamSet(await this.request(Command.PARAM_SAVE, new Uint8Array(0), 'param save'));
  }

  /**
   * Put parameters back to the values this build was compiled with.
   *
   * `index` is required for mode 1 and *must be omitted* for mode 2 — not
   * defaulted to 0. A caller that wants "all" and passes index 0 as a harmless
   * filler would be sending a frame this app never means to send, and the
   * firmware reads the two modes by length. The signature makes the two calls
   * different calls rather than the same call with a flag, which is how the
   * mode ends up in the type instead of in a comment at the call site.
   */
  async paramDefault(index: number): Promise<ParamSetReply>;
  async paramDefault(index: null): Promise<ParamSetReply>;
  async paramDefault(index: number | null): Promise<ParamSetReply> {
    const payload = index === null ? new Uint8Array([2]) : new Uint8Array([1, index]);
    const what = index === null ? 'param default all' : `param default ${index}`;
    return parseParamDefault(await this.request(Command.PARAM_DEFAULT, payload, what));
  }

  /**
   * What the receiver is hearing, right now.
   *
   * Polled rather than streamed, deliberately: the console link cannot stream
   * at all, and the one tab a person most wants while holding a transmitter is
   * exactly the one that should work on the cable. A caller that wants it
   * continuously calls this continuously.
   *
   * No arguments, and none to give: this is what the receiver is doing, not a
   * question about a channel.
   */
  async rcChannels(): Promise<RcState> {
    return parseRcChannels(
      await this.request(Command.RC_CHANNELS, new Uint8Array(0), 'rc channels'),
    );
  }

  /**
   * How fast the board can hear each motor turning, if it can hear them.
   *
   * Polled rather than streamed, for the same reason `rcChannels` is: the
   * console link cannot stream, and this is a number a person wants while
   * standing at the aircraft rather than one to plot.
   *
   * The request is the empty frame, and the reply is one of three answers —
   * see `MotorTelemetryState`. **No arguments, and deliberately none to give**:
   * a pole count would be a request to compute RPM, and the board reports its
   * own or reports that it cannot.
   */
  /** The loop profiler's window. A read; it changes nothing on the board. */
  async perf(): Promise<PerfWindow> {
    return parsePerf(await this.request(Command.PERF, new Uint8Array(0), 'perf'));
  }

  async motorTelemetry(): Promise<MotorTelemetryState> {
    return parseMotorTelemetry(
      await this.request(Command.MOTOR_TELEMETRY, new Uint8Array(0), 'motor telemetry'),
    );
  }

  /**
   * What one of the board's sensors is reading, or why it is not.
   *
   * One topic per call rather than "tell me about everything", because the
   * interesting answer is per-sensor and a combined reply would need a length
   * per section to stay inside 96 bytes — which is the shape `PARAM_INFO`'s
   * paging already has, and here it buys nothing. A caller that wants the set
   * calls this once per topic.
   *
   * The reply is not a reading that may be absent; it is one of **three**
   * answers, and `SensorAnswer` carries them apart. See the interface.
   */
  async sensorInfo(topic: number): Promise<SensorAnswer> {
    return parseSensorInfo(
      await this.request(Command.SENSOR_INFO, new Uint8Array([topic]), `sensor info ${topic}`),
    );
  }

  /**
   * What the board drives, and how many pads actually reach a header.
   *
   * The request is the empty frame — this is a read with no argument, so there
   * is no malformed request to refuse and the only thing the reply can say is
   * one of three things about the board. A board with more outputs than one
   * frame carries refuses rather than paging, and `OutputList` carries that
   * apart from "this board drives nothing".
   */
  async outputInfo(): Promise<OutputList> {
    return parseOutputInfo(await this.request(Command.OUTPUT_INFO, new Uint8Array(0), 'output info'));
  }

  /**
   * Hold one output at a level, or stop.
   *
   * **The only command in this client that makes an aircraft do something.** It
   * is one output at a time because the wire has no other form, and `levelPct`
   * is a request rather than an instruction: the board clamps it to its own cap
   * and answers with the percentage it will actually drive.
   *
   * The reply is checked against what was sent. That check is worth more here
   * than anywhere else in this file: a client holding a screen of four outputs
   * has to know which answer it is holding, and one that has lost track can
   * send the stop without asking first.
   */
  async outputTest(request: OutputTestRequest): Promise<OutputTestAnswer> {
    const payload = new Uint8Array([
      request.op,
      request.kind,
      request.index,
      request.levelPct & 0xff,
    ]);
    const what =
      `output test op ${request.op} kind ${request.kind} ${request.index} ` +
      `at ${request.levelPct}%`;
    return parseOutputTest(await this.request(Command.OUTPUT_TEST, payload, what), request);
  }

  /**
   * Which log the next `logInfo` and `logGet` are about. A source this device
   * does not have is refused with a non-zero status, and that is returned as
   * such rather than as a log with no records.
   */
  async selectLog(source: number): Promise<LogSourceReply> {
    return parseLogSource(
      await this.request(Command.LOG_SOURCE, new Uint8Array([source]), `log source ${source}`),
    );
  }

  /**
   * How many records the *currently selected* log holds, right now.
   *
   * This is the refresh, not the selection: `selectLog` is what decides which
   * log that is, and this asks the same question again later without changing
   * anything — a ring that has gone on recording since the last answer holds
   * more than it did, and a person who has just read a hundred records wants to
   * know whether there are now a hundred and one.
   *
   * **The reply carries no status byte, so this cannot report an absent log.**
   * `ak_proto.c` answers `count > 0 ? count : 0`, which collapses "this device
   * has no log reader", "this device does not have that ring" and "the ring is
   * empty" into one zero. That collapse is why `selectLog` exists and why this
   * method must never be the first thing asked: a caller that took a zero from
   * here as a fact about hardware would report an absent log as an empty one.
   * Select first; refresh after.
   */
  async logInfo(): Promise<number> {
    return parseLogInfo(await this.request(Command.LOG_INFO, new Uint8Array(0), 'log info'));
  }

  async logGet(index: number): Promise<LogRecord | null> {
    const payload = new Uint8Array(2);
    payload[0] = index & 0xff;
    payload[1] = (index >> 8) & 0xff;
    return parseLogRecord(
      await this.request(Command.LOG_GET, payload, `log get ${index}`),
      index,
    );
  }

  /**
   * Ask for a telemetry stream and return the rate the firmware agreed to.
   *
   * Zero is an answer, not an error: the console is a wire a person types at and
   * the firmware refuses to push frames into it. A client that treated zero as a
   * failure would report a working board as broken.
   */
  async subscribe(hz: number): Promise<number> {
    const wanted = Math.max(0, Math.min(TELEMETRY_MAX_HZ, Math.round(hz)));
    const reply = await this.request(
      Command.TELEMETRY,
      new Uint8Array([wanted]),
      'telemetry subscribe',
    );
    if (reply.length < 1) throw new ProtocolError('telemetry: the reply carried no rate');
    return reply[0]!;
  }

  /**
   * Ask for a range of a log to be pushed, and get back what will actually be
   * sent.
   *
   * The reply is not the answer to the whole transaction — the records arrive
   * afterwards, at `onLogFrame`, ending with a frame carrying `DONE`. So a
   * caller reads `hz === 0` here as "no stream is coming" and stops waiting,
   * rather than as an error: it is the console link's answer and the answer to
   * a range with nothing in it alike.
   *
   * A zero rate is sent as a *stop*, which is how a running stream is ended
   * early — the firmware treats it the same way it treats a request with
   * nothing to send.
   */
  async startLogStream(
    source: number,
    first: number,
    count: number,
    hz: number,
  ): Promise<LogStreamReply> {
    const wanted = Math.max(0, Math.min(LOG_STREAM_MAX_HZ, Math.round(hz)));
    const payload = new Uint8Array(6);
    payload[0] = source & 0xff;
    payload[1] = first & 0xff;
    payload[2] = (first >> 8) & 0xff;
    payload[3] = count & 0xff;
    payload[4] = (count >> 8) & 0xff;
    payload[5] = wanted;
    return parseLogStreamReply(
      await this.request(Command.LOG_STREAM, payload, `log stream ${source}`),
    );
  }
}

function describeIssue(issue: ReturnType<FrameDecoder['take']>['issues'][number]): string {
  switch (issue.kind) {
    case 'checksum':
      return `a frame failed its checksum (computed 0x${issue.expected.toString(16)}, the wire said 0x${issue.received.toString(16)})`;
    case 'length':
      return `a frame declared a length of ${issue.declared} bytes and was refused`;
    case 'version':
      return `a frame carried protocol version ${issue.version}, which this build does not speak`;
    case 'stalled':
      return `a frame stalled half-received (${issue.held} bytes) and was dropped`;
  }
}
