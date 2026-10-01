import fixtures from './fixtures/msp.json';
import { MspCommand, MSP2_CLI_SETTING, MSP2_CLI_SETTING_INFO, type MspLink } from '../src/protocol/msp';
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
  const size = v2 ? all[6]! | (all[7]! << 8) : all[3]! - 1;
  return all.subarray(start, start + size);
}

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

  constructor(private readonly variant: Variant = 'betaflight') {}

  private answerFor(command: number, v2: boolean, frame: Uint8Array): Uint8Array | null {
    const wanted = v2 ? this.v2Key(command, frame) : this.v1Key(command);
    if (wanted === null) return null;
    return bytes(captured(this.variant, wanted).reply);
  }

  private v1Key(command: number): string | null {
    const names: Record<number, string> = {
      [MspCommand.API_VERSION]: 'api_version',
      [MspCommand.FC_VARIANT]: 'fc_variant',
      [MspCommand.FC_VERSION]: 'fc_version',
      [MspCommand.BOARD_INFO]: 'board_info',
      [MspCommand.STATUS]: 'status',
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
   */
  private v2Key(command: number, frame: Uint8Array): string | null {
    const size = frame[6]! | (frame[7]! << 8);
    const asked = new TextDecoder()
      .decode(frame.subarray(8, 8 + size))
      .replace(/\0.*$/s, '');
    if (command === MSP2_CLI_SETTING_INFO) {
      return asked === 'failsafe_throttle' ? 'setting_info_known' : null;
    }
    if (command === MSP2_CLI_SETTING) {
      return asked === 'failsafe_throttle' ? 'setting_known' : 'setting_absent';
    }
    return null;
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
    queueMicrotask(() => {
      if (this.closed) return;
      for (const handler of this.dataHandlers) handler(reply);
    });
  }

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
  readonly link = new FixtureLink();

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
