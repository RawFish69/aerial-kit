import { Command, InfoStatus, TELEMETRY_MAX_HZ } from './constants';
import { buildFrame, baseCommand, FrameDecoder, isResponse, type Frame } from './frame';
import {
  parseHello,
  parseLogRecord,
  parseLogSource,
  parseParamGet,
  parseParamHelpSlice,
  parseParamInfoPage,
  parseParamSet,
  parseRcChannels,
  parseSensorInfo,
  parseStatus,
  parseTelemetry,
  ProtocolError,
  type BoardMeta,
  type Hello,
  type LogRecord,
  type LogSourceReply,
  type ParamInfoPage,
  type ParamSetReply,
  parseParamDefault,
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
  /** Called when the link drops, whatever the reason. */
  readonly onClosed?: (reason: string) => void;
  /** Called for frames the decoder refused, so a lossy link is visible rather
   *  than mysterious. */
  readonly onDecodeIssue?: (issue: string) => void;
}

interface Queued {
  readonly command: Command;
  readonly payload: Uint8Array;
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
  private closedReason: string | null = null;
  private readonly disposers: Array<() => void> = [];

  /** Diagnostics, counted rather than guessed at. */
  readonly stats = { frames: 0, telemetry: 0, issues: 0, writes: 0 };

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

    const current = this.current;
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
    if (this.current !== null || this.closedReason !== null) return;
    const next = this.queue.shift();
    if (next === undefined) return;
    this.current = next;
    this.timer = setTimeout(() => {
      const pending = this.current;
      if (pending !== next) return;
      this.finish();
      next.reject(new TimeoutError(next.label, this.timeoutMs));
    }, this.timeoutMs);
    void Promise.resolve(this.link.write(buildFrame(next.command, next.payload))).catch(
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
    return new Promise((resolve, reject) => {
      this.queue.push({ command, payload, resolve, reject, label });
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
   * Which log the next `logInfo` and `logGet` are about. A source this device
   * does not have is refused with a non-zero status, and that is returned as
   * such rather than as a log with no records.
   */
  async selectLog(source: number): Promise<LogSourceReply> {
    return parseLogSource(
      await this.request(Command.LOG_SOURCE, new Uint8Array([source]), `log source ${source}`),
    );
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
