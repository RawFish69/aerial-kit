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
 * **This file used to be read-only on principle, and now it can write.** The
 * overturn is deliberate and its reasoning is worth keeping, because the
 * argument that produced the old rule has not gone away.
 *
 * The old rule was: *parameter writes to somebody else's firmware are how a
 * tool crashes an aircraft.* That is still true, and it is why the write added
 * here is not a general one. What changed is that refusing to write turned out
 * to cost more than it saved — a person with a Betaflight board and a laptop
 * has a configurator already, and the honest choices were to write parameters
 * carefully or to be the one tool that cannot. So the write is added with the
 * four properties that make it safe, each of which is a test that fails if
 * removed:
 *
 *  1. **It is gated on the board's own armed state**, read from the status
 *     frame, and a state that has gone stale counts as unknown and not as
 *     disarmed. This is the *foreign* board's armed bit, a different fact from
 *      our own board's, and the two are never conflated.
 *  2. **It is reported as applied only after the board agrees.** Betaflight
 *     confirms a set in the reply itself, so that reply *is* the read-back;
 *     INAV answers an empty ACK, so the read-back is a second read and the
 *     write stands as *unconfirmed* until it lands. A write that cannot be
 *     confirmed is drawn as unconfirmed, never as done.
 *  3. **Persisting is a separate act.** A set changes the running
 *     configuration; `MSP_EEPROM_WRITE` is what makes it survive a battery
 *     change, and this app will not do it on the back of a write.
 *  4. **There is no general command builder.** The message ids that can leave
 *     this file are still a closed, named set.
 *
 * The one thing the old rule was right about and this keeps: **nothing here
 * arms, disarms, or sets an output.** There is no command for any of them, and
 * that is a property of the file rather than a policy of the interface.
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

/**
 * INAV's settings commands, from its own `msp_protocol_v2_common.h`.
 *
 * **These are not Betaflight's, and this app used to send Betaflight's to
 * INAV.** `inav-9.1.0` has no `msp_protocol_v2_betaflight.h` at all and no
 * case for `0x3010` anywhere under `src/main/`, so a setting read from an INAV
 * board was answered with silence, timed out, and was reported as *the board
 * does not have a setting by that name* — a wrong answer given confidently,
 * which is the worst kind this project has a rule about. The pinned sources
 * settle it in one grep; assuming the two firmwares share a command is what
 * went wrong, so the two are now separate constants selected by the variant
 * the board reported rather than by what this file hoped.
 */
export const MSP2_COMMON_SETTING = 0x1003;
export const MSP2_COMMON_SET_SETTING = 0x1004;
export const MSP2_COMMON_SETTING_INFO = 0x1007;

/**
 * `MSP_EEPROM_WRITE` (250) — the separate act that makes a changed setting
 * survive losing the battery.
 *
 * It is 250 in **both** firmwares (`msp_protocol.h` in each), and it is the
 * only one of the three commands they do agree on. Both refuse it while armed —
 * `betaflight-2026.6.1/src/main/msp/msp.c:3578` and
 * `inav-9.1.0/src/main/fc/fc_msp.c:2852` each open with an arming check and
 * return `MSP_RESULT_ERROR` — so the foreign board enforces its own gate here
 * and this app does not have to be the only thing standing between a person and
 * a flash write on an armed aircraft. It still asks first, because a round trip
 * saved is not the point; being told *why* is.
 */
export const MSP_EEPROM_WRITE = 250;

/** Which firmware's settings commands to speak. Not a guess: `FC_VARIANT`
 *  answers `BTFL` or `INAV` and this is that answer, narrowed. */
export type SettingFamily = 'betaflight' | 'inav';

export interface SettingCommands {
  readonly family: SettingFamily;
  /** Ask for one setting by name. */
  readonly read: number;
  /** Change one. */
  readonly write: number;
  /** The description behind a name. */
  readonly info: number;
  /**
   * How a value crosses the wire, which is the whole of the disagreement.
   *
   * `text` is Betaflight's: the payload is the CLI line `name = value` and the
   * reply is the same line with the value the board now holds — **the reply to
   * a write is the read-back**, so a write needs no second round trip.
   *
   * `typed` is INAV's: the value goes as its native binary width, chosen from
   * the type the *description* reports, and the reply is an empty ACK that says
   * nothing about the value. A read-back there is a second `read`, and until it
   * comes back the write is *unconfirmed* — which is a different thing from
   * failed and has to be drawn as one.
   */
  readonly encoding: 'text' | 'typed';
}

/**
 * The commands for the firmware the board said it was, or null for one this app
 * cannot read settings from.
 *
 * Null rather than a default, because defaulting is what produced the wrong
 * answer above: a board whose settings commands this app does not know is a
 * board to say so about, not a board to guess at.
 */
export function settingCommandsFor(variant: MspFcVariant): SettingCommands | null {
  if (variant.variant === 'BTFL') {
    return {
      family: 'betaflight',
      read: MSP2_CLI_SETTING,
      write: MSP2_CLI_SETTING,
      info: MSP2_CLI_SETTING_INFO,
      encoding: 'text',
    };
  }
  if (variant.variant === 'INAV') {
    return {
      family: 'inav',
      read: MSP2_COMMON_SETTING,
      write: MSP2_COMMON_SET_SETTING,
      info: MSP2_COMMON_SETTING_INFO,
      encoding: 'typed',
    };
  }
  return null;
}

/**
 * INAV's value types, `inav-9.1.0/src/main/fc/settings.h:21-29`.
 *
 * **The byte on the wire is masked.** `SETTING_INFO` sends `setting->type`
 * whole, and that field packs the section into bits 3-5 and the mode into bits
 * 6-7 (`SETTING_TYPE_OFFSET`/`_SECTION_OFFSET`/`_MODE_OFFSET`), so a profile
 * setting arrives as `0x08 | VAR_UINT8` and a decoder that switched on the raw
 * byte would call the commonest setting on the board an unknown type. The
 * firmware masks with `SETTING_TYPE_MASK` before switching
 * (`settings.h:73`); this does the same, and a test feeds it `0x28` to prove it.
 */
export enum InavSettingType {
  UINT8 = 0,
  INT8 = 1,
  UINT16 = 2,
  INT16 = 3,
  UINT32 = 4,
  FLOAT = 5,
  STRING = 6,
}

export const SETTING_TYPE_MASK = 0x07;
export const SETTING_SECTION_MASK = 0x38;
export const SETTING_MODE_MASK = 0xc0;

/** How many bytes a value of this type occupies, or null for one this app will
 *  not encode — `STRING` is a pointer into the board's own storage and writing
 *  one is not a thing to do from here. */
export function inavValueSize(type: InavSettingType): number | null {
  switch (type) {
    case InavSettingType.UINT8:
    case InavSettingType.INT8:
      return 1;
    case InavSettingType.UINT16:
    case InavSettingType.INT16:
      return 2;
    case InavSettingType.UINT32:
    case InavSettingType.FLOAT:
      return 4;
    case InavSettingType.STRING:
      return null;
  }
}

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

/** The XOR MSP v1 defines: size, command, every payload byte. The size is the
 *  payload's length alone - Betaflight's `mspHeaderV1_t` is `{size, cmd}` and
 *  `dataSize = size` - which this file, aerialkit's Python client and its fake
 *  board all had as length + 1 until 2026-10-06: consistent with each other,
 *  so every fixture agreed, and with no real board. */
function checksumV1(command: number, payload: Uint8Array): number {
  let value = payload.length & 0xff;
  value ^= command & 0xff;
  for (const byte of payload) value ^= byte;
  return value;
}

/** The largest v2 payload this decoder will wait for. */
export const MSP_MAX_PAYLOAD_V2 = 4096;

export function buildRequest(command: number, payload: Uint8Array = new Uint8Array(0)): Uint8Array {
  if (payload.length > MSP_MAX_PAYLOAD_V1) {
    throw new RangeError(`a payload of ${payload.length} bytes does not fit an MSP v1 frame`);
  }
  const out = new Uint8Array(6 + payload.length);
  out[0] = 0x24;
  out[1] = 0x4d;
  out[2] = 0x3c;
  out[3] = payload.length;
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
  | { readonly kind: 'truncated'; readonly version: 1 | 2 }
  /** A header whose size no board sends: corrupted, and skipped. */
  | { readonly kind: 'oversize'; readonly version: 2; readonly size: number };

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
      // says nothing about which framing is talking. `!` is the third byte of
      // an *error* frame in both, which is why it appears twice below.
      let start = -1;
      let v2 = false;
      for (let i = 0; i + 2 < this.buffer.length; i++) {
        if (this.buffer[i] !== 0x24) continue;
        const second = this.buffer[i + 1];
        const third = this.buffer[i + 2];
        if (second === 0x4d && (third === 0x3c || third === 0x3e || third === 0x21)) {
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
    // Marker (3), size, command, `size` payload bytes, checksum.
    const total = 3 + 1 + 1 + size + 1;
    if (this.buffer.length < total) return 0;
    const command = this.buffer[4]!;
    const payload = Uint8Array.from(this.buffer.slice(5, 5 + size));
    const received = this.buffer[5 + size]!;
    if (checksumV1(command, payload) !== received) {
      this.problems.push({ kind: 'checksum', version: 1 });
      return -1;
    }
    const refused = this.buffer[2] === 0x21;
    this.buffer.splice(0, total);
    // **v1 does have an error frame, and this file said it did not.** The claim
    // was that a v1 board which will not answer says so by saying nothing. That
    // is wrong: Betaflight picks the third byte from `packet->result` in
    // `msp_serial.c:329`, so a refused v1 command comes back as `$M!` with the
    // original command and an empty payload. Reading every `$M` as `$M>` meant a
    // refusal was decoded as a *successful answer with no data* — the worst
    // possible reading, because the caller then reports the command as having
    // been accepted. Found by capturing a real `MSP_EEPROM_WRITE` refusal from
    // the stand-in and watching this decoder drop the frame on the floor.
    this.pending.push({ version: 1, command, refused, payload });
    return total;
  }

  private tryV2(): number {
    if (this.buffer.length < 9) return 0;
    const refused = this.buffer[2] === 0x21;
    const function_ = this.buffer[4]! | (this.buffer[5]! << 8);
    const size = this.buffer[6]! | (this.buffer[7]! << 8);
    // A size no board sends is a corrupted header, not a frame to wait for:
    // trusting it made the decoder buffer up to 64 KiB of every later frame
    // before giving up. Betaflight's and INAV's receive buffers are a few
    // hundred bytes; 4 KiB is well past any reply either sends.
    if (size > MSP_MAX_PAYLOAD_V2) {
      this.problems.push({ kind: 'oversize', version: 2, size });
      return -1;
    }
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

  u32(): number {
    this.need(4);
    const view = new DataView(this.bytes.buffer, this.bytes.byteOffset + this.at, 4);
    this.at += 4;
    return view.getUint32(0, true);
  }

  i32(): number {
    this.need(4);
    const view = new DataView(this.bytes.buffer, this.bytes.byteOffset + this.at, 4);
    this.at += 4;
    return view.getInt32(0, true);
  }

  /** A NUL-terminated string, which is how INAV writes a setting's name
   *  (`sbufWriteDataSafe(dst, name_buf, strlen(name_buf) + 1)`). Not
   *  `pstring` — a length byte and a NUL are different wire formats, and
   *  reading one as the other gives a name with a stray byte on the front. */
  cstring(): string {
    const end = this.bytes.indexOf(0, this.at);
    if (end < 0) {
      throw new MspProtocolError(`${this.what}: a name that is never terminated`);
    }
    const text = new TextDecoder().decode(this.bytes.subarray(this.at, end));
    this.at = end + 1;
    return text;
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

/**
 * INAV's description of a setting, `mspSettingInfoCommand`
 * (`inav-9.1.0/src/main/fc/fc_msp.c:3970`), which is a **binary struct** and not
 * the text Betaflight answers with. Same command number in neither case — INAV's
 * is 0x1007 and Betaflight's is 0x3011 — so the two never had to agree, and they
 * do not: one is `key=value` lines in an output buffer, the other is this.
 *
 * The layout, in order, from the source: the name NUL-terminated, then `pgn`
 * (u16), then `type`, `section` and `mode` (u8 each), then `min` (i32) and `max`
 * (u32), then the absolute index (u16), and finally **two bytes that are always
 * present** — the current profile and the profile count for a profile-based
 * setting, or two zeroes for a master one. Those last two are the reason a
 * decoder cannot stop at the index: the board promises them so a client can
 * assume a fixed length, and a reader that treated them as absent would be
 * reading a shorter struct than the board wrote.
 */
export interface InavSettingInfo {
  readonly name: string;
  readonly pgn: number;
  /** The masked value type — see `InavSettingType` for why it is masked. */
  readonly type: InavSettingType;
  /** The raw byte, kept so a caller can report the section and mode too. */
  readonly typeByte: number;
  readonly section: number;
  readonly mode: number;
  readonly min: number;
  readonly max: number;
  readonly index: number;
  readonly profile: number;
  readonly profileCount: number;
}

export function parseInavSettingInfo(payload: Uint8Array): InavSettingInfo {
  const reader = new Reader(payload, 'MSP2_COMMON_SETTING_INFO');
  const name = reader.cstring();
  const pgn = reader.u16();
  const typeByte = reader.u8();
  const section = reader.u8();
  const mode = reader.u8();
  const min = reader.i32();
  const max = reader.u32();
  const index = reader.u16();
  const profile = reader.u8();
  const profileCount = reader.u8();
  return {
    name,
    pgn,
    type: (typeByte & SETTING_TYPE_MASK) as InavSettingType,
    typeByte,
    section,
    mode,
    min,
    max,
    index,
    profile,
    profileCount,
  };
}

/**
 * An INAV value as its native bytes, or a sentence saying why it will not go.
 *
 * The number is parsed strictly and then range-checked against the min and max
 * the *board* reported, because INAV's own `mspSetSettingCommand` does exactly
 * that and rejects the write if it is out of range — so sending one is a
 * guaranteed refusal, and refusing it here is the same answer one round trip
 * earlier with a better sentence. It is not a second authority: the bounds are
 * the board's, and the board's copy of them is still what decides.
 *
 * Floats go as IEEE-754 little-endian, which is what `sbufReadF32Safe` reads.
 * An integer type is **not** rounded or truncated to fit — a value that does not
 * survive the round trip is refused rather than silently altered, because a
 * parameter that came back different from what was typed is how a person learns
 * not to trust the panel.
 */
export function encodeInavValue(
  type: InavSettingType,
  text: string,
  bounds: { readonly min: number; readonly max: number },
): { readonly bytes: Uint8Array } | { readonly refuses: string } {
  const size = inavValueSize(type);
  if (size === null) {
    return { refuses: 'a string setting is a pointer into the board\'s own storage and is not written from here' };
  }
  const trimmed = text.trim();
  const value = Number(trimmed);
  if (trimmed === '' || !Number.isFinite(value)) {
    return { refuses: `"${trimmed}" is not a number` };
  }
  if (value < bounds.min || value > bounds.max) {
    return {
      refuses: `the board says this setting runs from ${bounds.min} to ${bounds.max}, so ${trimmed} is out of range`,
    };
  }
  const view = new DataView(new ArrayBuffer(size));
  switch (type) {
    case InavSettingType.UINT8:
      view.setUint8(0, value);
      break;
    case InavSettingType.INT8:
      view.setInt8(0, value);
      break;
    case InavSettingType.UINT16:
      view.setUint16(0, value, true);
      break;
    case InavSettingType.INT16:
      view.setInt16(0, value, true);
      break;
    case InavSettingType.UINT32:
      view.setUint32(0, value, true);
      break;
    case InavSettingType.FLOAT:
      view.setFloat32(0, value, true);
      break;
    default:
      return { refuses: 'this app does not encode that setting type' };
  }
  // The check the encode is really for: a value that did not survive being
  // written at this width is not the value that was asked for.
  const back = decodeInavValue(type, new Uint8Array(view.buffer));
  if (back === null || Math.abs(back - value) > Math.abs(value) * 1e-6 + 1e-9) {
    return { refuses: `${trimmed} does not survive being written as a ${InavSettingType[type].toLowerCase()}` };
  }
  return { bytes: new Uint8Array(view.buffer) };
}

/** The number an INAV value reads as, or null for a type this app does not
 *  decode. The inverse of the encode above, and deliberately total: it is used
 *  to read back what the board holds, where a wrong answer is worse than none. */
export function decodeInavValue(type: InavSettingType, bytes: Uint8Array): number | null {
  if (bytes.length < (inavValueSize(type) ?? Infinity)) return null;
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  switch (type) {
    case InavSettingType.UINT8:
      return view.getUint8(0);
    case InavSettingType.INT8:
      return view.getInt8(0);
    case InavSettingType.UINT16:
      return view.getUint16(0, true);
    case InavSettingType.INT16:
      return view.getInt16(0, true);
    case InavSettingType.UINT32:
      return view.getUint32(0, true);
    case InavSettingType.FLOAT:
      return view.getFloat32(0, true);
    case InavSettingType.STRING:
      return null;
  }
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
    const failed = (error: unknown) => {
      if (this.inFlight !== next) return;
      this.settle();
      next.reject(error instanceof Error ? error : new Error(String(error)));
      this.pump();
    };
    try {
      // A write that fails *asynchronously* (a serial port gone between two
      // requests) rejected a promise nobody held: an unhandled rejection, and
      // the request then sat out its whole timeout instead of failing at once.
      void Promise.resolve(this.link.write(next.frame)).catch(failed);
    } catch (error) {
      failed(error);
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
          : issue.kind === 'oversize'
            ? `an MSP v2 header claimed ${issue.size} bytes, which no board sends, and was skipped`
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
