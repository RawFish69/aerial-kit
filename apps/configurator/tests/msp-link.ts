import fixtures from './fixtures/msp.json';
import {
  MspCommand,
  MSP2_CLI_SETTING,
  MSP2_CLI_SETTING_INFO,
  MSP2_COMMON_SETTING,
  MSP2_COMMON_SETTING_INFO,
  MSP2_COMMON_SET_SETTING,
  MSP_EEPROM_WRITE,
  type MspLink,
} from '../src/protocol/msp';
import type { Transport } from '../src/transport/types';

/**
 * The captured frames, and a board that answers from them.
 *
 * Not a `.test.ts`, so the runner does not collect it: this is shared between
 * the tests about the protocol and the ones about *detecting* which protocol is
 * on the other end, and both want the same board — the one whose bytes came out
 * of this repository's Betaflight stand-in.
 *
 * The framing below is read longhand rather than through the decoder it is
 * meant to be testing. A test that pulls its payload out with the code under
 * test proves only that the code agrees with itself; a second reader means a
 * mistake in either one shows up as a checksum or an offset that does not match.
 */
export type Variant = 'betaflight' | 'inav';

export interface Capture {
  readonly note: string;
  readonly request: string;
  readonly reply: string;
}

export function captured(variant: Variant, key: string): Capture {
  const all = fixtures[variant] as unknown as Record<string, Capture>;
  const one = all[key];
  if (one === undefined) throw new Error(`no captured frame ${variant}/${key}`);
  return one;
}

export const keys = (variant: Variant): string[] =>
  Object.keys(fixtures[variant] as object).filter((key) => !key.startsWith('_'));

export function bytes(hex: string): Uint8Array {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(hex.substr(i * 2, 2), 16);
  return out;
}

export function hex(value: Uint8Array): string {
  return Array.from(value, (byte) => byte.toString(16).padStart(2, '0')).join('');
}

/** The payload, walked by hand. v1 is `$M>` + size + command + payload + xor;
 *  v2 is `$X>` + flags + function(2) + size(2) + payload + crc. */
export function payloadOf(hexFrame: string): Uint8Array {
  const all = bytes(hexFrame);
  const v2 = all[1] === 0x58;
  const start = v2 ? 8 : 5;
  // v1's size is the payload's length (it was read as length + 1 until
  // 2026-10-06, the same mistake as the decoder this checks).
  const size = v2 ? all[6]! | (all[7]! << 8) : all[3]!;
  return all.subarray(start, start + size);
}

/**
 * The setting the fixtures carry, and the range its *own description* states.
 *
 * Read out of `setting_info_known` rather than written down a second time, so
 * the harness cannot accept a value the board's description says is out of
 * range. A hardcoded 1000..2000 here would be a third opinion about a fact the
 * fixture already contains, and the day the stand-in's table changes the
 * harness would go on enforcing the old range — which is the failure mode this
 * whole file exists to avoid.
 */
const SETTING = 'failsafe_throttle';

/** The value a captured read carries, in the layout that variant uses. */
function readValue(variant: Variant, key: string): number {
  const payload = payloadOf(captured(variant, key).reply);
  if (variant === 'inav') {
    const at = payload.indexOf(0) + 1;
    return payload[at]! | (payload[at + 1]! << 8);
  }
  return Number(/=\s*(-?\d+)/.exec(new TextDecoder().decode(payload))?.[1]);
}

interface BoardFacts {
  readonly min: number;
  readonly max: number;
  /** What the board holds *after* the captured write. */
  readonly written: number;
}

function boardFacts(variant: Variant): BoardFacts {
  const info = payloadOf(captured(variant, 'setting_info_known').reply);
  const written = readValue(variant, 'setting_read_after_write');
  if (variant === 'inav') {
    // name\0, pgn u16, type u8, section u8, mode u8, min i32, max u32, ...
    const at = info.indexOf(0) + 1 + 2 + 3;
    const view = new DataView(info.buffer, info.byteOffset + at);
    return { min: view.getInt32(0, true), max: view.getUint32(4, true), written };
  }
  const text = new TextDecoder().decode(info.subarray(2));
  return {
    min: Number(/min=(-?\d+)/.exec(text)?.[1]),
    max: Number(/max=(-?\d+)/.exec(text)?.[1]),
    written,
  };
}

const inRange = (value: number, range: { min: number; max: number }): boolean =>
  Number.isFinite(value) && value >= range.min && value <= range.max;

const stripNul = (text: string): string => text.replace(/\0.*$/s, '');

/**
 * A link that answers from the captured frames, so a client is driven by bytes
 * a board really sent rather than by a loopback that agrees with it.
 */
export class FixtureLink implements MspLink {
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();
  readonly written: Uint8Array[] = [];
  closed = false;
  /** Set to model a board that speaks something else entirely: writes are
   *  recorded and then ignored. */
  silent = false;
  /** Whether the board says it is armed. The board is the authority, so the
   *  tests set this and then *read* it back through `STATUS` rather than
   *  handing the session a boolean — a session that believed a caller instead
   *  of the board would pass either way. */
  armed = false;

  /**
   * A board that answers a write without taking it.
   *
   * Not a hypothetical: it is the case the four-fact rule exists for. A board
   * that ACKs and keeps its old value is what an INAV write looks like from a
   * client that trusts the ACK, and it is the only way to reach the app's
   * disagreement path now that both stand-ins refuse out-of-range values
   * rather than clamping them.
   */
  ignoresWrites = false;

  /** What the board was last told to hold, and only for the value the
   *  read-back fixture was captured at — see below. Not named `written`: that
   *  is the array of frames this board was *sent*, and shadowing it cost a
   *  confusing round of failures the first time. */
  private held: number | null = null;

  /** This variant's own stated range for `SETTING`, and the value its write
   *  was captured at — the two families report the first in different layouts,
   *  so neither is assumed. */
  private readonly facts: BoardFacts;

  constructor(private readonly variant: Variant = 'betaflight') {
    this.facts = boardFacts(variant);
  }

  private get range(): { min: number; max: number } {
    return this.facts;
  }

  /**
   * The key for a *read*, which depends on whether the board has been written
   * to — because a board's answer to `what do you hold` depends on that.
   *
   * Served only for the one value the read-back was captured at. Any other
   * value falls back to the pre-write frame, and that is deliberate: the
   * alternative is arithmetic on a captured reply in the test harness, which
   * would make this file a second implementation of the board and put the
   * harness's opinion between the app and the bytes. A test that writes 1300
   * and reads must not be told it holds 1200 by a harness that decided so.
   */
  private readKey(): string {
    return this.held === this.facts.written ? 'setting_read_after_write' : 'setting_known';
  }

  private noteWrite(value: number): void {
    if (!this.ignoresWrites) this.held = value;
  }

  private answerFor(command: number, v2: boolean, frame: Uint8Array): Uint8Array | null {
    const wanted = v2 ? this.v2Key(command, frame) : this.v1Key(command, frame);
    if (wanted === null) return null;
    return bytes(captured(this.variant, wanted).reply);
  }

  private v1Key(command: number, frame: Uint8Array): string | null {
    // The two commands whose answer depends on the arming flags rather than on
    // what was asked. Both firmwares refuse `MSP_EEPROM_WRITE` while armed, and
    // a `STATUS` that still said disarmed would make the gate untestable.
    if (command === MspCommand.STATUS) return this.armed ? 'status_armed' : 'status';
    if (command === MSP_EEPROM_WRITE) {
      return this.armed ? 'eeprom_write_refused' : 'eeprom_write';
    }
    const names: Record<number, string> = {
      [MspCommand.API_VERSION]: 'api_version',
      [MspCommand.FC_VARIANT]: 'fc_variant',
      [MspCommand.FC_VERSION]: 'fc_version',
      [MspCommand.BOARD_INFO]: 'board_info',
      [MspCommand.ATTITUDE]: 'attitude',
      [MspCommand.MOTOR]: 'motor',
      [MspCommand.RC]: 'rc',
      [MspCommand.RAW_GPS]: 'raw_gps',
      [MspCommand.ANALOG]: 'analog',
    };
    return names[command] ?? null;
  }

  /**
   * A settings answer depends on the *name* asked for, not only on the command.
   *
   * The stand-in has two settings and refuses anything else, so keying this on
   * the command alone would answer `no_such_setting_at_all` with the value of
   * `failsafe_throttle` — and the refusal, which is the thing under test, would
   * never be exercised.
   *
   * The two families are keyed separately because they are separate protocols:
   * Betaflight reads and writes on the *same* command number and separates them
   * by whether the payload has an `=` in it, while INAV has a command for each
   * and crosses the value as bytes. A single key table would have to pretend
   * those were the same thing, which is exactly the mistake the app made.
   */
  private v2Key(command: number, frame: Uint8Array): string | null {
    const size = frame[6]! | (frame[7]! << 8);
    const payload = frame.subarray(8, 8 + size);
    return this.variant === 'inav'
      ? this.inavKey(command, payload)
      : this.betaflightKey(command, payload);
  }

  private betaflightKey(command: number, payload: Uint8Array): string | null {
    const text = new TextDecoder().decode(payload);
    if (command === MSP2_CLI_SETTING_INFO) {
      return stripNul(text) === SETTING ? 'setting_info_known' : null;
    }
    if (command !== MSP2_CLI_SETTING) return null;
    const eq = text.indexOf('=');
    // No `=`: a read. This is the only way to tell the two apart on this
    // firmware, and a harness that guessed from the command would answer a
    // *write* with the value the board held before it.
    if (eq < 0) return stripNul(text) === SETTING ? this.readKey() : 'setting_absent';
    const name = text.slice(0, eq).trim();
    if (name !== SETTING) return 'setting_absent';
    const value = Number(text.slice(eq + 1).trim());
    if (!inRange(value, this.range)) return 'setting_write_refused';
    this.noteWrite(value);
    return 'setting_write';
  }

  private inavKey(command: number, payload: Uint8Array): string | null {
    const end = payload.indexOf(0);
    const name = new TextDecoder().decode(payload.subarray(0, end < 0 ? payload.length : end));
    if (command === MSP2_COMMON_SETTING_INFO) {
      return name === SETTING ? 'setting_info_known' : null;
    }
    if (command === MSP2_COMMON_SETTING) {
      return name === SETTING ? this.readKey() : 'setting_absent';
    }
    if (command !== MSP2_COMMON_SET_SETTING) return null;
    if (name !== SETTING) return 'setting_absent';
    // The value is the setting's own width, little-endian, straight after the
    // NUL — `failsafe_throttle` is a uint16 in the pinned release.
    const value = payload[end + 1]! | (payload[end + 2]! << 8);
    if (!inRange(value, this.range)) return 'setting_write_refused';
    this.noteWrite(value);
    return 'setting_write';
  }

  write(frame: Uint8Array): void {
    if (this.closed) return;
    this.written.push(frame);
    if (this.silent) return;
    // The command, read straight out of the frame the way a board's own parser
    // reads it. v2's is 16 bits, which v1 has no room for. And `$M` and `$X`
    // are told apart by the second byte — their third is `>` for both.
    const v2 = frame[1] === 0x58;
    const command = v2 ? frame[4]! | (frame[5]! << 8) : frame[4]!;
    const reply = this.answerFor(command, v2, frame);
    if (reply === null) return; // a command the board does not know: silence
    // Saves are counted here rather than by the caller, so the number is what
    // the *board* was told to do. A test that counted its own calls would pass
    // on a session that never sent the frame.
    if (command === MSP_EEPROM_WRITE && !v2 && this.armed === false) this.saves += 1;
    queueMicrotask(() => {
      if (this.closed) return;
      for (const handler of this.dataHandlers) handler(reply);
    });
  }

  /** How many times the board was actually asked to write its flash. */
  saves = 0;

  /** Nothing comes back at all — a board that speaks another protocol. */
  ignore(): void {
    this.written.length = 0;
    this.silent = true;
  }

  deliver(chunk: Uint8Array): void {
    queueMicrotask(() => {
      if (this.closed) return;
      for (const handler of this.dataHandlers) handler(chunk);
    });
  }

  hangUp(reason = 'the cable was pulled'): void {
    this.closed = true;
    for (const handler of this.closeHandlers) handler(reason);
  }

  onData(handler: (chunk: Uint8Array) => void): () => void {
    this.dataHandlers.add(handler);
    return () => this.dataHandlers.delete(handler);
  }
  onClose(handler: (reason: string) => void): () => void {
    this.closeHandlers.add(handler);
    return () => this.closeHandlers.delete(handler);
  }
  onError(handler: (message: string) => void): () => void {
    this.errorHandlers.add(handler);
    return () => this.errorHandlers.delete(handler);
  }
}

/** `FixtureLink` wearing the `Transport` a session — or a page — needs. */
export class FixtureTransport implements Transport {
  readonly info = {
    kind: 'serial' as const,
    label: 'USB serial',
    detail: 'a Betaflight stand-in on the other end',
  };
  private open_ = false;
  readonly link: FixtureLink;

  constructor(variant: Variant = 'betaflight') {
    this.link = new FixtureLink(variant);
  }

  /** Make the board say it is armed. Nothing is told to the session: the next
   *  `STATUS` poll is how it finds out, which is the only way a real board
   *  would ever tell it. */
  arm(): void {
    this.link.armed = true;
  }

  /** Every frame the board was sent, so a test can count what was *not* sent.
   *  The gate before a write is only worth anything if it is before the write,
   *  and that is a fact about this array rather than about a sentence. */
  get written(): Uint8Array[] {
    return this.link.written;
  }

  /** How many times the board was actually asked to write its flash. */
  savedCount(): number {
    return this.link.saves;
  }

  get isOpen(): boolean {
    return this.open_;
  }
  async open(): Promise<void> {
    this.open_ = true;
  }
  async close(): Promise<void> {
    this.open_ = false;
  }
  write(bytes: Uint8Array): void {
    this.link.write(bytes);
  }
  onData(handler: (chunk: Uint8Array) => void): () => void {
    return this.link.onData(handler);
  }
  onClose(handler: (reason: string) => void): () => void {
    return this.link.onClose(handler);
  }
  onError(handler: (message: string) => void): () => void {
    return this.link.onError(handler);
  }
}
