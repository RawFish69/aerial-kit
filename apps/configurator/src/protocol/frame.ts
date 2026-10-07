import { crc16 } from './crc16';
import {
  CLIENT_GAP_MS,
  MAX_FRAME,
  MAX_PAYLOAD,
  PROTOCOL_VERSION,
  RESPONSE_BIT,
  SYNC1,
  SYNC2,
} from './constants';

/**
 * One decoded frame.
 *
 * `payload` is a copy, not a view into the decoder's buffer. That is deliberate:
 * a view would be overwritten by the next frame, and a caller holding "the
 * parameter table" while a telemetry stream runs would watch it change under
 * it. The cost is one small allocation per frame, which at 50 Hz is not a cost.
 */
export interface Frame {
  readonly version: number;
  readonly command: number;
  readonly payload: Uint8Array;
}

/** Something the decoder refused. Never thrown: a bad frame on a live link is
 *  a statistic, not an exception, and a client that gives up on the first
 *  corrupted byte is a client that cannot be used on a real UART. */
export type DecodeIssue =
  | { readonly kind: 'checksum'; readonly expected: number; readonly received: number }
  | { readonly kind: 'length'; readonly declared: number }
  | { readonly kind: 'version'; readonly version: number }
  | { readonly kind: 'stalled'; readonly held: number };

export interface DecodeResult {
  readonly frames: Frame[];
  readonly issues: DecodeIssue[];
}

const enum State {
  Sync1,
  Sync2,
  Body,
  CrcLo,
  CrcHi,
}

export function isResponse(command: number): boolean {
  return (command & RESPONSE_BIT) !== 0;
}

/** The command with the response bit cleared — what to compare against. */
export function baseCommand(command: number): number {
  return command & ~RESPONSE_BIT & 0xff;
}

/**
 * A byte-fed frame decoder, the browser's half of `ak_proto_feed()`.
 *
 * It is fed whatever the transport produced, in whatever sizes the transport
 * chose — one byte, half a frame, three frames at once. Nothing about the
 * output depends on the chunking, which is the property the tests spend most of
 * their time on.
 *
 * Bounded by construction: the body buffer is `MAX_FRAME` bytes and a declared
 * length beyond `MAX_PAYLOAD` is refused before anything is written, so a
 * hostile or broken peer cannot grow this. The firmware's own parser is bounded
 * the same way.
 */
export class FrameDecoder {
  private state: State = State.Sync1;
  private readonly body = new Uint8Array(MAX_FRAME);
  private held = 0;
  private expected = 3;
  private lastByteMs = 0;

  /** Frames and issues produced since the last call to `take()`. */
  private frames: Frame[] = [];
  private issues: DecodeIssue[] = [];

  reset(): void {
    this.state = State.Sync1;
    this.held = 0;
    this.expected = 3;
    this.frames = [];
    this.issues = [];
  }

  /** Drops a partial frame that has stalled for longer than the wire allows.
   *  The firmware abandons a frame after `AK_PROTO_GAP_MS` of silence and hands
   *  the port back to the console; a client that kept waiting would treat the
   *  next reply as the tail of this one. Returns true if something was dropped. */
  expireStalled(nowMs: number): boolean {
    if (this.state === State.Sync1 || this.held === 0) return false;
    if (nowMs - this.lastByteMs <= CLIENT_GAP_MS) return false;
    this.issues.push({ kind: 'stalled', held: this.held });
    this.state = State.Sync1;
    this.held = 0;
    this.expected = 3;
    return true;
  }

  push(bytes: Uint8Array, nowMs: number = Date.now()): DecodeResult {
    this.expireStalled(nowMs);
    for (let i = 0; i < bytes.length; i++) {
      this.lastByteMs = nowMs;
      this.feed(bytes[i]!);
    }
    return this.take();
  }

  take(): DecodeResult {
    const result = { frames: this.frames, issues: this.issues };
    this.frames = [];
    this.issues = [];
    return result;
  }

  private fail(issue: DecodeIssue): void {
    this.issues.push(issue);
    this.state = State.Sync1;
    this.held = 0;
    this.expected = 3;
  }

  private feed(value: number): void {
    switch (this.state) {
      case State.Sync1:
        if (value === SYNC1) this.state = State.Sync2;
        return;

      case State.Sync2:
        if (value === SYNC2) {
          this.state = State.Body;
          this.held = 0;
          this.expected = 3;
        } else {
          // A byte that is not the second sync is not necessarily garbage: it
          // may be the first sync of the frame that follows a stray 0xAA. The
          // firmware's parser restarts from nothing here, which loses a frame
          // on the 0xAA 0xAA 0x55 sequence; re-testing the byte costs one
          // comparison and does not.
          this.state = value === SYNC1 ? State.Sync2 : State.Sync1;
        }
        return;

      case State.Body: {
        if (this.held >= this.body.length) {
          this.fail({ kind: 'length', declared: this.held });
          return;
        }
        this.body[this.held++] = value;

        // version, command, length
        if (this.held === 3) {
          const declared = this.body[2]!;
          if (declared > MAX_PAYLOAD) {
            this.fail({ kind: 'length', declared });
            return;
          }
          this.expected = 3 + declared;
        }
        if (this.held >= this.expected) this.state = State.CrcLo;
        return;
      }

      case State.CrcLo:
        if (this.held >= this.body.length) {
          this.fail({ kind: 'length', declared: this.held });
          return;
        }
        this.body[this.held++] = value;
        this.state = State.CrcHi;
        return;

      case State.CrcHi: {
        if (this.held + 1 > this.body.length) {
          this.fail({ kind: 'length', declared: this.held });
          return;
        }
        this.body[this.held++] = value;

        const expectedCrc = crc16(this.body.subarray(0, this.expected));
        const received = this.body[this.expected]! | (this.body[this.expected + 1]! << 8);

        const version = this.body[0]!;
        const command = this.body[1]!;
        const problem =
          expectedCrc !== received
            ? ({ kind: 'checksum', expected: expectedCrc, received } as const)
            : version !== PROTOCOL_VERSION
              ? ({ kind: 'version', version } as const)
              : null;

        if (problem) {
          this.fail(problem);
          return;
        }

        this.frames.push({
          version,
          command,
          payload: this.body.slice(3, this.expected),
        });
        this.state = State.Sync1;
        this.held = 0;
        this.expected = 3;
        return;
      }
    }
  }
}

/**
 * Builds a request frame. The CRC covers the version, command, length and
 * payload — everything but the sync pair and the CRC itself, which is the only
 * arrangement where a corrupted *length* cannot be hidden by the bytes after
 * it.
 */
export function buildFrame(command: number, payload: Uint8Array = new Uint8Array(0)): Uint8Array {
  if (payload.length > MAX_PAYLOAD) {
    throw new RangeError(`payload of ${payload.length} exceeds the ${MAX_PAYLOAD}-byte maximum`);
  }
  const body = new Uint8Array(3 + payload.length);
  body[0] = PROTOCOL_VERSION;
  body[1] = command & 0xff;
  body[2] = payload.length;
  body.set(payload, 3);

  const crc = crc16(body);
  const frame = new Uint8Array(body.length + 4);
  frame[0] = SYNC1;
  frame[1] = SYNC2;
  frame.set(body, 2);
  frame[frame.length - 2] = crc & 0xff;
  frame[frame.length - 1] = (crc >> 8) & 0xff;
  return frame;
}
