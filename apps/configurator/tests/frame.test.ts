import { describe, expect, it } from 'vitest';
import { Command, MAX_PAYLOAD, PROTOCOL_VERSION } from '../src/protocol/constants';
import { buildFrame, FrameDecoder, isResponse, baseCommand } from '../src/protocol/frame';
import { crc16 } from '../src/protocol/crc16';
import fixture from './fixtures/aerialkit.json';

/**
 * The decoder is fed whatever the transport produced, so the properties that
 * matter are about *chunking*: the same bytes in one piece, one byte at a time,
 * or split through the middle of the CRC must produce the same frames. Almost
 * every test here is that idea with a different split.
 */

type Decoded = ReturnType<FrameDecoder['take']>;

/** `push()` returns what it produced and clears the decoder, so the results are
 *  collected as they come rather than read out afterwards. */
function framesFrom(chunks: Uint8Array[], nowMs = 0): Decoded {
  const decoder = new FrameDecoder();
  const all: Decoded = { frames: [], issues: [] };
  for (const chunk of chunks) {
    const result = decoder.push(chunk, nowMs);
    all.frames.push(...result.frames);
    all.issues.push(...result.issues);
  }
  return all;
}

function bytesOf(hex: string): Uint8Array {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(hex.slice(i * 2, i * 2 + 2), 16);
  return out;
}

const HELLO = bytesOf(fixture.frames.hello.reply);
const STATUS = bytesOf(fixture.frames.status.reply);

describe('the decoder against real captured frames', () => {
  it('decodes a frame the firmware actually sent', () => {
    const { frames, issues } = framesFrom([HELLO]);
    expect(issues).toEqual([]);
    expect(frames).toHaveLength(1);
    expect(frames[0]!.command).toBe(Command.HELLO | 0x80);
    expect(isResponse(frames[0]!.command)).toBe(true);
    expect(baseCommand(frames[0]!.command)).toBe(Command.HELLO);
  });

  it('produces the same frames however the bytes are chunked', () => {
    const whole = framesFrom([HELLO]);
    const byOne = framesFrom(Array.from(HELLO, (byte) => new Uint8Array([byte])));
    // Splits chosen to land inside the header, the payload and the CRC.
    const split = framesFrom([HELLO.slice(0, 1), HELLO.slice(1, 5), HELLO.slice(5, HELLO.length - 1), HELLO.slice(HELLO.length - 1)]);
    for (const other of [byOne, split]) {
      expect(other.frames).toEqual(whole.frames);
      expect(other.issues).toEqual([]);
    }
  });

  it('decodes two frames that arrived in one read', () => {
    const both = new Uint8Array(HELLO.length + STATUS.length);
    both.set(HELLO, 0);
    both.set(STATUS, HELLO.length);
    const { frames, issues } = framesFrom([both]);
    expect(issues).toEqual([]);
    expect(frames.map((frame) => frame.command)).toEqual([
      Command.HELLO | 0x80,
      Command.STATUS | 0x80,
    ]);
  });

  it('gives the payload as a copy, so the next frame cannot rewrite it', () => {
    const decoder = new FrameDecoder();
    const first = decoder.push(HELLO, 0).frames[0]!;
    const before = Uint8Array.from(first.payload);
    decoder.push(STATUS, 0);
    expect(first.payload).toEqual(before);
  });
});

describe('what the decoder refuses', () => {
  it('refuses a frame whose checksum is wrong, and says both numbers', () => {
    const broken = Uint8Array.from(HELLO);
    broken[broken.length - 1]! ^= 0xff;
    const { frames, issues } = framesFrom([broken]);
    expect(frames).toEqual([]);
    expect(issues).toHaveLength(1);
    expect(issues[0]!.kind).toBe('checksum');
  });

  it('refuses a declared length beyond the protocol maximum', () => {
    // version, command, length — then nothing. A length of 200 is refused as
    // soon as it is read, without waiting for 200 bytes that will not come.
    const decoder = new FrameDecoder();
    const { frames, issues } = decoder.push(
      new Uint8Array([0xaa, 0x55, PROTOCOL_VERSION, Command.STATUS, 200]),
      0,
    );
    expect(frames).toEqual([]);
    expect(issues[0]).toEqual({ kind: 'length', declared: 200 });
  });

  it('accepts a declared length of exactly the maximum', () => {
    const frame = buildFrame(Command.STATUS, new Uint8Array(MAX_PAYLOAD));
    const { frames, issues } = framesFrom([frame]);
    expect(issues).toEqual([]);
    expect(frames).toHaveLength(1);
    expect(frames[0]!.payload).toHaveLength(MAX_PAYLOAD);
  });

  it('refuses a payload one byte past the maximum before it is built', () => {
    expect(() => buildFrame(Command.STATUS, new Uint8Array(MAX_PAYLOAD + 1))).toThrow(RangeError);
  });

  it('refuses a frame from a protocol version it does not speak', () => {
    const body = new Uint8Array([PROTOCOL_VERSION + 1, Command.STATUS, 0]);
    const crc = crc16(body);
    const frame = new Uint8Array([0xaa, 0x55, ...body, crc & 0xff, crc >> 8]);
    const { frames, issues } = framesFrom([frame]);
    expect(frames).toEqual([]);
    expect(issues[0]).toEqual({ kind: 'version', version: PROTOCOL_VERSION + 1 });
  });

  it('drops a half-received frame that has stalled', () => {
    const decoder = new FrameDecoder();
    decoder.push(HELLO.slice(0, 4), 1000);
    expect(decoder.expireStalled(1000 + 49)).toBe(false); // inside the gap
    expect(decoder.expireStalled(1000 + 51)).toBe(true); // past it
    const { frames, issues } = decoder.take();
    expect(frames).toEqual([]);
    expect(issues[0]).toEqual({ kind: 'stalled', held: 2 });
  });

  it('resynchronises on the 0xAA 0xAA 0x55 sequence', () => {
    // The firmware's parser restarts from nothing on the second byte here and
    // loses the frame; re-testing the byte costs one comparison and does not.
    const bytes = new Uint8Array([0xaa, ...HELLO]);
    const { frames, issues } = framesFrom([bytes]);
    expect(issues).toEqual([]);
    expect(frames).toHaveLength(1);
    expect(baseCommand(frames[0]!.command)).toBe(Command.HELLO);
  });

  it('recovers a good frame that follows a corrupted one', () => {
    const broken = Uint8Array.from(HELLO);
    broken[6]! ^= 0xff;
    const both = new Uint8Array(broken.length + STATUS.length);
    both.set(broken, 0);
    both.set(STATUS, broken.length);
    const { frames, issues } = framesFrom([both]);
    expect(issues).toHaveLength(1);
    expect(frames).toHaveLength(1);
    expect(baseCommand(frames[0]!.command)).toBe(Command.STATUS);
  });
});

describe('the checksum itself', () => {
  it('matches the CRC the firmware put on the wire', () => {
    // The captured frame's last two bytes are the firmware's answer. Computing
    // it here from the same bytes is what proves the polynomial, the init value
    // and the byte order all agree with `ak_proto_crc16`.
    const body = HELLO.subarray(2, HELLO.length - 2);
    const expected = crc16(body);
    expect(expected & 0xff).toBe(HELLO[HELLO.length - 2]);
    expect(expected >> 8).toBe(HELLO[HELLO.length - 1]);
  });

  it('is CRC-16/CCITT-FALSE: the standard check value', () => {
    // "123456789" is the canonical input; 0x29B1 is the canonical answer.
    expect(crc16(new TextEncoder().encode('123456789'))).toBe(0x29b1);
  });
});
