/**
 * MSP — the protocol Betaflight and INAV speak, from the browser's side.
 *
 * This app has one protocol of its own and then has to talk to boards that are
 * not ours. There is no single protocol to be compatible with: AerialKit speaks
 * its own, Betaflight and INAV speak MSP, ArduPilot and PX4 speak MAVLink.
 * This file knows MSP.
 *
 * The bytes are Betaflight's own implementation, not a summary of it —
 * `src/main/msp/msp_serial.c` for the framing, `msp_protocol.h` and `msp.c` for
 * the commands and their payload layouts — and every decoder here is tested
 * against frames captured from this repository's own Betaflight stand-in
 * (`tools/capture-msp-fixtures.py`), which is itself written from the same
 * source.
 *
 * **Read-only, deliberately.** Parameter *writes* to somebody else's firmware
 * are how a tool crashes an aircraft. Nothing in this file has a write command,
 * and the interface says so rather than offering a button that does nothing.
 *
 * Three differences from our own protocol are worth knowing before reading on:
 *
 *  - **A reply carries the same command it answers.** There is no response bit
 *    and no sequence number, so two requests for the same command outstanding
 *    at once could not be told apart. Requests are serialised for the same
 *    reason ours are.
 *  - **Two framings.** MSP v1 is `$M<` with an XOR checksum and an 8-bit
 *    command; MSP v2 is `$X<` with CRC-8/DVB-S2 and a 16-bit function. The
 *    settings commands only exist in v2, because 0x3010 does not fit in a byte.
 *  - **There is no "list the settings" command.** `MSP2_CLI_SETTING` takes a
 *    *name* and answers the text `name = value`. A ground station that shows a
 *    table of them ships the list of names itself. That is the per-firmware
 *    parameter model, in the protocol's own words.
 */

// ---- framing ---------------------------------------------------------------

export const FRAME_REQUEST = 0x244d3c; // "$M<"
export const FRAME_REPLY = 0x244d3e; // "$M>"
export const FRAME_REQUEST_V2 = 0x24583c; // "$X<"
export const FRAME_REPLY_V2 = 0x24583e; // "$X>"
export const FRAME_REPLY_V2_ERROR = 0x245821; // "$X!" — a reply that is a refusal

/** A v1 size byte is a byte, so a payload is at most 255 and the frame at most
 *  259. Used to bound what a stream of noise can make this allocate. */
export const MSP_MAX_PAYLOAD_V1 = 255;

/** Commands, from `msp_protocol.h`. The numbers are the protocol. */
export enum MspCommand {
  API_VERSION = 1,
  FC_VARIANT = 2,
  FC_VERSION = 3,
  BOARD_INFO = 4,
  STATUS = 101,
  MOTOR = 104,
  RC = 105,
  RAW_GPS = 106,
  ATTITUDE = 108,
  ANALOG = 110,
}

/**
 * The two commands a Betaflight board's *settings* answer on, from
 * `msp_protocol_v2_betaflight.h`. They live up here rather than with the
 * commands above because a v2 function is 16 bits wide and these are two of
 * the three thousandths.
 */
export const MSP2_CLI_SETTING = 0x3010;
export const MSP2_CLI_SETTING_INFO = 0x3011;

/** `crc8_calc` — poly 0xD5, init 0. The checksum of every v2 frame. */
export function crc8DvbS2(bytes: Uint8Array): number {
  let crc = 0;
  for (const byte of bytes) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) {
      crc = crc & 0x80 ? ((crc << 1) ^ 0xd5) & 0xff : (crc << 1) & 0xff;
    }
  }
  return crc;
}

/** The XOR MSP v1 defines: size, command, every payload byte. */
function checksumV1(command: number, payload: Uint8Array): number {
  let value = (payload.length + 1) & 0xff;
  value ^= command & 0xff;
  for (const byte of payload) value ^= byte;
  return value;
}

export function buildRequest(command: number, payload: Uint8Array = new Uint8Array(0)): Uint8Array {
  if (payload.length > MSP_MAX_PAYLOAD_V1) {
    throw new RangeError(`a payload of ${payload.length} bytes does not fit an MSP v1 frame`);
  }
  const out = new Uint8Array(6 + payload.length);
  out[0] = 0x24;
  out[1] = 0x4d;
  out[2] = 0x3c;
  out[3] = payload.length + 1;
  out[4] = command & 0xff;
  out.set(payload, 5);
  out[5 + payload.length] = checksumV1(command, payload);
  return out;
}

export function buildRequestV2(function_: number, payload: Uint8Array = new Uint8Array(0)): Uint8Array {
  const header = new Uint8Array(5);
  header[0] = 0; // flags
  header[1] = function_ & 0xff;
  header[2] = (function_ >> 8) & 0xff;
  header[3] = payload.length & 0xff;
  header[4] = (payload.length >> 8) & 0xff;

  const out = new Uint8Array(8 + payload.length + 1);
  out[0] = 0x24;
  out[1] = 0x58;
  out[2] = 0x3c;
  out.set(header, 3);
  out.set(payload, 8);
  const crcOver = new Uint8Array(header.length + payload.length);
  crcOver.set(header, 0);
  crcOver.set(payload, header.length);
  out[8 + payload.length] = crc8DvbS2(crcOver);
  return out;
}

// ---- decoding --------------------------------------------------------------

export interface MspFrame {
  /** 1 or 2. Which framing carried it. */
  readonly version: 1 | 2;
  /** The command (v1) or function (v2). A reply carries the one it answers. */
  readonly command: number;
  /** True for `$X!`: the board answered, and the answer was no. */
  readonly refused: boolean;
  readonly payload: Uint8Array;
}

export type MspIssue =
  | { readonly kind: 'checksum'; readonly version: 1 | 2 }
  | { readonly kind: 'truncated'; readonly version: 1 | 2 };

export interface MspDecodeResult {
  readonly frames: MspFrame[];
  readonly issues: MspIssue[];
}

/**
 * A byte-fed MSP decoder for both framings.
 *
 * Fed whatever the transport produced, in whatever sizes it chose. It never
 * throws: a bad frame on a live link is a statistic, not an exception, and a
 * client that gave up on the first corrupted byte would be unusable on a real
 * UART — which is exactly the reasoning our own decoder uses.
 */
export class MspDecoder {
  private buffer: number[] = [];
  private pending: MspFrame[] = [];
  private problems: MspIssue[] = [];

  push(chunk: Uint8Array): MspDecodeResult {
    for (const byte of chunk) this.buffer.push(byte);
    this.scan();
    const frames = this.pending;
    const issues = this.problems;
    this.pending = [];
    this.problems = [];
    return { frames, issues };
  }

  /** Anything held half-received. A caller deciding "has this board gone quiet"
   *  wants to know there are bytes stuck rather than none at all. */
  get held(): number {
    return this.buffer.length;
  }

  reset(): void {
    this.buffer.length = 0;
  }

  private scan(): void {
    for (;;) {
      // Find a magic. Every byte that is not the start of a frame is skipped,
      // which is what a board's own serial parser does with a stream somebody
      // typed at — and what this must do, because detection sends a question
      // at a board that speaks something else entirely.
      //
      // `$M` and `$X` are told apart by their **second** byte. Their third
      // does not do it: `$M>` and `$X>` both end in `>`, and 0x3e on its own
      // says nothing about which framing is talking.
      let start = -1;
      let v2 = false;
      for (let i = 0; i + 2 < this.buffer.length; i++) {
        if (this.buffer[i] !== 0x24) continue;
        const second = this.buffer[i + 1];
        const third = this.buffer[i + 2];
        if (second === 0x4d && (third === 0x3c || third === 0x3e)) {
          start = i;
          v2 = false;
          break;
        }
        if (second === 0x58 && (third === 0x3c || third === 0x3e || third === 0x21)) {
          start = i;
          v2 = true;
          break;
        }
      }
      if (start < 0) {
        // Keep the last two bytes: they may be the beginning of a magic whose
        // third byte has not arrived yet.
        if (this.buffer.length > 2) this.buffer.splice(0, this.buffer.length - 2);
        return;
      }
      if (start > 0) this.buffer.splice(0, start);

      const consumed = v2 ? this.tryV2() : this.tryV1();
      if (consumed === 0) return; // need more bytes
      if (consumed < 0) {
        // Refused: drop one byte and resynchronise rather than stalling here.
        this.buffer.shift();
      }
    }
  }

  /** Returns bytes consumed, 0 for "need more", or -1 for "this one is bad". */
  private tryV1(): number {
    if (this.buffer.length < 6) return 0;
    const size = this.buffer[3]!;
    const total = 3 + 1 + size + 1;
    if (this.buffer.length < total) return 0;
    const command = this.buffer[4]!;
    const payload = Uint8Array.from(this.buffer.slice(5, 5 + size - 1));
    const received = this.buffer[5 + size - 1]!;
    if (checksumV1(command, payload) !== received) {
      this.problems.push({ kind: 'checksum', version: 1 });
      return -1;
    }
    this.buffer.splice(0, total);
    // v1 has no error frame: `$M!` is not a thing Betaflight sends. A v1 board
    // that will not answer says so by saying nothing, which is what the client's
    // deadline is for. Only v2 has a refusal to report.
    this.pending.push({ version: 1, command, refused: false, payload });
    return total;
  }

  private tryV2(): number {
    if (this.buffer.length < 9) return 0;
    const refused = this.buffer[2] === 0x21;
    const function_ = this.buffer[4]! | (this.buffer[5]! << 8);
    const size = this.buffer[6]! | (this.buffer[7]! << 8);
    const total = 3 + 5 + size + 1;
    if (this.buffer.length < total) return 0;
    const payload = Uint8Array.from(this.buffer.slice(8, 8 + size));
    const received = this.buffer[8 + size]!;
    const crcOver = Uint8Array.from(this.buffer.slice(3, 8 + size));
    if (crc8DvbS2(crcOver) !== received) {
      this.problems.push({ kind: 'checksum', version: 2 });
      return -1;
    }
    this.buffer.splice(0, total);
    this.pending.push({ version: 2, command: function_, refused, payload });
    return total;
  }
}

// ---- messages --------------------------------------------------------------

export class MspProtocolError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'MspProtocolError';
  }
}

/**
 * The board answered, and the answer was no.
 *
 * MSP says that with the frame's third byte: `$X!` is a reply carrying an error
 * rather than a result. A client that treated it as data would show a person an
 * empty answer where the board said "no such name" — and "this board does not
 * have that setting" is a fact, while "" is a bug report.
 */
export class MspRefusal extends Error {
  constructor(readonly command: number) {
    super(`the board refused 0x${command.toString(16)}`);
    this.name = 'MspRefusal';
  }
}

/** A reader that refuses to guess. A short payload is an error, never a zero:
 *  a zero latitude is a place in the Atlantic, and "this board does not say" is
 *  a different answer from "this board says zero". */
class Reader {
  private at = 0;
  constructor(
    private readonly bytes: Uint8Array,
    private readonly what: string,
  ) {}

  private need(count: number): void {
    if (this.at + count > this.bytes.length) {
      throw new MspProtocolError(
        `${this.what}: wanted ${count} more bytes at offset ${this.at}, and the payload is ${this.bytes.length}`,
      );
    }
  }

  u8(): number {
    this.need(1);
    return this.bytes[this.at++]!;
  }

  u16(): number {
    this.need(2);
    const value = this.bytes[this.at]! | (this.bytes[this.at + 1]! << 8);
    this.at += 2;
    return value;
  }

  i16(): number {
    const value = this.u16();
    return value >= 0x8000 ? value - 0x10000 : value;
  }

  i32(): number {
    this.need(4);
    const view = new DataView(this.bytes.buffer, this.bytes.byteOffset + this.at, 4);
    this.at += 4;
    return view.getInt32(0, true);
  }

  /** A length byte and that many characters — `sbufWritePString`. */
  pstring(): string {
    const length = this.u8();
    this.need(length);
    const text = new TextDecoder().decode(this.bytes.subarray(this.at, this.at + length));
    this.at += length;
    return text;
  }

  rest(): Uint8Array {
    const out = this.bytes.subarray(this.at);
    this.at = this.bytes.length;
    return out;
  }

  get done(): boolean {
    return this.at >= this.bytes.length;
  }
}

export interface MspApiVersion {
  readonly protocolVersion: number;
  readonly apiMajor: number;
  readonly apiMinor: number;
}

export function parseApiVersion(payload: Uint8Array): MspApiVersion {
  const reader = new Reader(payload, 'MSP_API_VERSION');
  return {
    protocolVersion: reader.u8(),
    apiMajor: reader.u8(),
    apiMinor: reader.u8(),
  };
}

export interface MspFcVariant {
  /** `BTFL`, `INAV`, or something this app has never heard of. */
  readonly variant: string;
  readonly known: boolean;
}

export function parseFcVariant(payload: Uint8Array): MspFcVariant {
  const variant = new TextDecoder().decode(payload).replace(/\0+$/, '');
  return { variant, known: variant === 'BTFL' || variant === 'INAV' };
}

export interface MspFcVersion {
  readonly version: string;
}

export function parseFcVersion(payload: Uint8Array): MspFcVersion {
  const reader = new Reader(payload, 'MSP_FC_VERSION');
  reader.u8(); // year since 2000
  reader.u8();
  reader.u8();
  return { version: reader.done ? '' : reader.pstring() };
}

export interface MspBoardInfo {
  readonly boardIdentifier: string;
  readonly targetName: string;
  readonly boardName: string;
  readonly manufacturer: string;
}

export function parseBoardInfo(payload: Uint8Array): MspBoardInfo {
  if (payload.length < 4) throw new MspProtocolError('MSP_BOARD_INFO: shorter than its identifier');
  const identifier = new TextDecoder().decode(payload.subarray(0, 4)).replace(/\0+$/, '');
  // Four identifier bytes, then a u16 hardware revision, then board type and
  // capabilities, and only then the three strings.
  const rest = new Reader(payload.subarray(8), 'MSP_BOARD_INFO');
  return {
    boardIdentifier: identifier,
    targetName: rest.pstring(),
    boardName: rest.pstring(),
    manufacturer: rest.pstring(),
  };
}

export interface MspStatus {
  readonly cycleTimeUs: number;
  readonly i2cErrors: number;
  /** Which sensors the board reports having. A missing one is not a fault, it
   *  is a board that does not carry it. */
  readonly sensors: {
    readonly accel: boolean;
    readonly baro: boolean;
    readonly mag: boolean;
    readonly gps: boolean;
    readonly gyro: boolean;
  };
  /** True when the ARM box is active. This is the board's own answer, and the
   *  only one this app will act on. */
  readonly armed: boolean;
  readonly modeFlags: number;
  readonly profile: number;
}

export function parseStatus(payload: Uint8Array): MspStatus {
  const reader = new Reader(payload, 'MSP_STATUS');
  const cycleTimeUs = reader.u16();
  const i2cErrors = reader.u16();
  const sensors = reader.u16();
  const modeFlags = reader.i32();
  const profile = reader.u8();
  return {
    cycleTimeUs,
    i2cErrors,
    sensors: {
      accel: (sensors & (1 << 0)) !== 0,
      baro: (sensors & (1 << 1)) !== 0,
      mag: (sensors & (1 << 2)) !== 0,
      gps: (sensors & (1 << 3)) !== 0,
      gyro: (sensors & (1 << 5)) !== 0,
    },
    armed: (modeFlags & 1) !== 0,
    modeFlags,
    profile,
  };
}

export interface MspAttitude {
  readonly rollDeg: number;
  readonly pitchDeg: number;
  readonly yawDeg: number;
}

export function parseAttitude(payload: Uint8Array): MspAttitude {
  const reader = new Reader(payload, 'MSP_ATTITUDE');
  return {
    rollDeg: reader.i16() / 10,
    pitchDeg: reader.i16() / 10,
    yawDeg: reader.i16() / 10,
  };
}

export interface MspAnalog {
  readonly volts: number;
  readonly mahDrawn: number;
  readonly rssi: number;
  readonly amps: number;
}

export function parseAnalog(payload: Uint8Array): MspAnalog {
  const reader = new Reader(payload, 'MSP_ANALOG');
  const vbat = reader.u8();
  const mahDrawn = reader.u16();
  const rssi = reader.u16();
  const amps = reader.i16();
  // The grown frame carries a second, finer voltage. A board that stops after
  // the first is an older board, not a broken one.
  const fine = reader.done ? null : reader.u16();
  return { volts: (fine ?? vbat * 10) / 10, mahDrawn, rssi, amps };
}

export interface MspRawGps {
  readonly fixType: number;
  readonly satellites: number;
  readonly lat: number;
  readonly lon: number;
  readonly altitudeM: number;
  readonly speedCmS: number;
  readonly courseDeg: number;
}

export function parseRawGps(payload: Uint8Array): MspRawGps {
  const reader = new Reader(payload, 'MSP_RAW_GPS');
  return {
    fixType: reader.u8(),
    satellites: reader.u8(),
    lat: reader.i32(),
    lon: reader.i32(),
    altitudeM: reader.u16(),
    speedCmS: reader.u16(),
    courseDeg: reader.i16() / 10,
  };
}

export function parseMotor(payload: Uint8Array): readonly number[] {
  const reader = new Reader(payload, 'MSP_MOTOR');
  const out: number[] = [];
  while (!reader.done) out.push(reader.u16());
  return out;
}

export function parseRc(payload: Uint8Array): readonly number[] {
  const reader = new Reader(payload, 'MSP_RC');
  const out: number[] = [];
  while (!reader.done) out.push(reader.u16());
  return out;
}

/**
 * `MSP2_CLI_SETTING` — ask a setting by name, get back `name = value`.
 *
 * There is no "list them" request in the protocol, which is why this takes a
 * name and why the interface has a box to type one in.
 */
export function parseSetting(payload: Uint8Array): { readonly name: string; readonly value: string } {
  const text = new TextDecoder().decode(payload).replace(/\0+$/, '');
  const at = text.indexOf('=');
  if (at < 0) return { name: text.trim(), value: '' };
  return { name: text.slice(0, at).trim(), value: text.slice(at + 1).trim() };
}

export interface MspSettingInfo {
  /** How long the whole description is. A board answers it in windows, so this
   *  is what says whether the window just received is the last one. */
  readonly totalLength: number;
  readonly window: string;
  readonly fields: Readonly<Record<string, string>>;
}

export function parseSettingInfo(payload: Uint8Array): MspSettingInfo {
  const reader = new Reader(payload, 'MSP2_CLI_SETTING_INFO');
  const totalLength = reader.u16();
  const window = new TextDecoder().decode(reader.rest());
  const fields: Record<string, string> = {};
  for (const line of window.split('\n')) {
    const at = line.indexOf('=');
    if (at > 0) fields[line.slice(0, at).trim()] = line.slice(at + 1).trim();
  }
  return { totalLength, window, fields };
}

// ---- the client ------------------------------------------------------------

/** The byte pipe an MSP client needs. The same shape our own client takes, so
 *  a transport never has to know which protocol is about to speak through it —
 *  which is what makes firmware detection possible at all. */
export interface MspLink {
  write(bytes: Uint8Array): void | Promise<void>;
  onData(handler: (chunk: Uint8Array) => void): () => void;
  onClose(handler: (reason: string) => void): () => void;
  onError(handler: (message: string) => void): () => void;
}

/**
 * The board did not answer.
 *
 * This is the *expected* outcome of the first question asked of a board that
 * turns out to speak something else, which is why it is an ordinary result and
 * not an exception a caller has to guard against to detect firmware.
 */
export class MspTimeoutError extends Error {
  constructor(command: number, ms: number) {
    super(`the board did not answer 0x${command.toString(16)} within ${ms} ms`);
    this.name = 'MspTimeoutError';
  }
}

export class MspLinkClosedError extends Error {
  constructor() {
    super('the connection closed before the board answered');
    this.name = 'MspLinkClosedError';
  }
}

export interface MspClientOptions {
  /** How long to wait. A board on a real UART can be slow to boot, and an MSP
   *  board asked a question it does not know answers with silence rather than
   *  with an error — so this is the only thing that ends the wait. */
  readonly timeoutMs?: number;
  /** Frames the board sent that nobody asked for. MSP has no telemetry push, so
   *  these are worth counting rather than discarding quietly. */
  readonly onUnsolicited?: (frame: MspFrame) => void;
  /** Frames the decoder refused, so a lossy cable is visible rather than
   *  mysterious. */
  readonly onIssue?: (issue: string) => void;
  readonly onClosed?: (reason: string) => void;
}

interface Queued {
  readonly version: 1 | 2;
  readonly command: number;
  readonly frame: Uint8Array;
  readonly resolve: (payload: Uint8Array) => void;
  readonly reject: (error: Error) => void;
}

/**
 * MSP, from the client's side.
 *
 * Serialised for the same reason our own client is, but for a sharper one: MSP
 * has no response bit *and* no sequence number, so a reply is tied to its
 * request by the command number alone. Two outstanding requests for the same
 * command would be indistinguishable, and there is no way to detect that after
 * the fact. One at a time is the only correct implementation.
 *
 * A refusal is delivered as `MspRefusal` rather than as an empty payload. The
 * board said no, and "no such setting" is an answer a person needs to see.
 */
export class MspClient {
  private readonly decoder = new MspDecoder();
  private readonly queue: Queued[] = [];
  private inFlight: Queued | null = null;
  private timer: ReturnType<typeof setTimeout> | null = null;
  private closed = false;
  private readonly unsubscribes: Array<() => void> = [];
  private readonly timeoutMs: number;

  /** Counted rather than guessed at, so a lossy cable is a number on a screen. */
  readonly stats = { frames: 0, issues: 0 };

  constructor(
    private readonly link: MspLink,
    private readonly options: MspClientOptions = {},
  ) {
    this.timeoutMs = options.timeoutMs ?? 1000;
    this.unsubscribes.push(link.onData((chunk) => this.onData(chunk)));
    this.unsubscribes.push(
      link.onClose((reason) => {
        this.closed = true;
        this.failInFlight(new MspLinkClosedError());
        this.options.onClosed?.(reason);
      }),
    );
    this.unsubscribes.push(link.onError((message) => this.options.onIssue?.(message)));
  }

  /** v1: the state commands. */
  request(command: number, payload: Uint8Array = new Uint8Array(0)): Promise<Uint8Array> {
    return this.enqueue(1, command, buildRequest(command, payload));
  }

  /** v2: the settings commands, which do not fit in a v1 command byte. */
  requestV2(function_: number, payload: Uint8Array = new Uint8Array(0)): Promise<Uint8Array> {
    return this.enqueue(2, function_, buildRequestV2(function_, payload));
  }

  /**
   * Reads the board's identity: what it is, what release, what board.
   *
   * Every one of these is a read. This method exists so the *first* thing this
   * app does with a board it has not met is look at it, which is the order the
   * design asks for and the order that cannot damage anything.
   */
  async identify(): Promise<MspIdentity> {
    const api = parseApiVersion(await this.request(MspCommand.API_VERSION));
    const variant = parseFcVariant(await this.request(MspCommand.FC_VARIANT));
    const release = await this.request(MspCommand.FC_VERSION)
      .then(parseFcVersion)
      .catch(() => ({ version: '' }));
    // A board that will not describe itself is still a board with an identity.
    // The identifier is a question mark rather than blank, so the screen says
    // "not established" instead of drawing an empty box.
    const board = await this.request(MspCommand.BOARD_INFO)
      .then(parseBoardInfo)
      .catch(() => null);
    return { api, variant, release, board };
  }

  close(): void {
    this.closed = true;
    this.failInFlight(new MspLinkClosedError());
    for (const off of this.unsubscribes) off();
    this.unsubscribes.length = 0;
  }

  private enqueue(version: 1 | 2, command: number, frame: Uint8Array): Promise<Uint8Array> {
    if (this.closed) return Promise.reject(new MspLinkClosedError());
    return new Promise<Uint8Array>((resolve, reject) => {
      this.queue.push({ version, command, frame, resolve, reject });
      this.pump();
    });
  }

  private pump(): void {
    if (this.closed || this.inFlight !== null || this.queue.length === 0) return;
    const next = this.queue.shift()!;
    this.inFlight = next;
    // Started before the write, not after: a write that throws must still leave
    // the queue in a state the next caller can use.
    this.timer = setTimeout(() => {
      const stalled = this.inFlight;
      this.inFlight = null;
      this.timer = null;
      stalled?.reject(new MspTimeoutError(stalled.command, this.timeoutMs));
      this.pump();
    }, this.timeoutMs);
    try {
      void this.link.write(next.frame);
    } catch (error) {
      this.settle();
      next.reject(error instanceof Error ? error : new Error(String(error)));
      this.pump();
    }
  }

  private settle(): void {
    if (this.timer !== null) clearTimeout(this.timer);
    this.timer = null;
    this.inFlight = null;
  }

  private failInFlight(error: Error): void {
    const stalled = this.inFlight;
    this.settle();
    stalled?.reject(error);
    while (this.queue.length > 0) this.queue.shift()!.reject(error);
  }

  private onData(chunk: Uint8Array): void {
    if (this.closed) return;
    const { frames, issues } = this.decoder.push(chunk);
    this.stats.frames += frames.length;
    this.stats.issues += issues.length;
    for (const issue of issues) {
      this.options.onIssue?.(
        issue.kind === 'checksum'
          ? `an MSP v${issue.version} frame failed its checksum`
          : `an MSP v${issue.version} frame was cut short`,
      );
    }
    for (const frame of frames) {
      const outstanding = this.inFlight;
      if (outstanding === null || outstanding.command !== frame.command) {
        // Nobody asked, or somebody asked something else. Counted, never
        // mistaken for the answer to the question that is outstanding.
        this.options.onUnsolicited?.(frame);
        continue;
      }
      this.settle();
      if (frame.refused) outstanding.reject(new MspRefusal(frame.command));
      else outstanding.resolve(frame.payload);
      this.pump();
    }
  }
}

export interface MspIdentity {
  readonly api: MspApiVersion;
  readonly variant: MspFcVariant;
  readonly release: MspFcVersion;
  /** Null when the board would not describe itself — a question mark, not an
   *  empty board name. */
  readonly board: MspBoardInfo | null;
}
