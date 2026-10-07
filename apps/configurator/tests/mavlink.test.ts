import { describe, expect, it } from 'vitest';

import {
  CRC_EXTRA,
  MAVLINK_MESSAGES,
  MavProtocolError,
  MavlinkClient,
  MavlinkDecoder,
  MAV_CMD_SET_MESSAGE_INTERVAL,
  MAV_MODE_FLAG_SAFETY_ARMED,
  bytesBeyondTable,
  crcAccumulate,
  decode,
  deriveCrcExtra,
  heartbeatOf,
  mavLayout,
  num,
  str,
  type MavLink,
  type MavlinkFrame,
} from '../src/protocol/mavlink';
import fixture from './fixtures/mavlink.json';

/**
 * The MAVLink adapter against frames it did not produce.
 *
 * Two oracles, and they are different in kind, which is the point:
 *
 *  - **`frames`** were sent by `firmware/tools/mavlink_fake_vehicle.py`, whose
 *    frames are built by **pymavlink** — the reference implementation. Feeding
 *    those bytes to this app's decoder is a real test: everything has to agree,
 *    the framing, the checksum, the CRC_EXTRA, the field offsets and the
 *    zero-truncation rule.
 *  - **`packed`** were built by pymavlink directly, for the three messages the
 *    fake vehicle never sends. `tools/capture-mavlink-fixtures.py` makes its own
 *    longhand parser validate every one of them before writing the file, so a
 *    frame in there has already been accepted by two independent readings of
 *    the wire format.
 *
 * What is *not* an oracle here is the `crc_extra` block: it was transcribed by
 * hand into the capture script, so checking this app's derivation against it
 * catches a typo but proves nothing on its own. The frames are what prove it —
 * a wrong CRC_EXTRA makes every checksum fail, and the decode tests would go
 * red rather than green.
 */

interface CapturedFrame {
  readonly msgid: number;
  readonly crc_extra: number;
  readonly frame_hex: string;
  readonly payload_hex: string;
  readonly seq: number;
  readonly source: string;
}

const frames = fixture.frames as unknown as Record<string, CapturedFrame>;
const packed = fixture.packed as unknown as Record<string, CapturedFrame[]>;
const expect_ = fixture.expect as unknown as Record<string, Record<string, unknown>>;

function bytes(hex: string): Uint8Array {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(hex.slice(i * 2, i * 2 + 2), 16);
  return out;
}

/** Decodes one captured frame, asserting nothing else came with it. */
function oneFrame(hex: string): MavlinkFrame {
  const { frames: got, issues } = new MavlinkDecoder().push(bytes(hex));
  expect(issues).toEqual([]);
  expect(got).toHaveLength(1);
  return got[0]!;
}

/** Every captured frame, from both sources, with the name it was filed under. */
function allCaptured(): Array<{ name: string; frame: CapturedFrame }> {
  const out: Array<{ name: string; frame: CapturedFrame }> = [];
  for (const [name, frame] of Object.entries(frames)) out.push({ name, frame });
  for (const [name, list] of Object.entries(packed)) {
    for (const frame of list) out.push({ name, frame });
  }
  return out;
}

describe('the message table', () => {
  it('derives the published CRC_EXTRA for every message it carries', () => {
    // Every byte of this byte is a function of the message's name, its field
    // types, its field names, its array lengths and which fields are
    // extensions. A single wrong character anywhere changes it - so a table
    // that reproduces all fifteen has been checked field by field, by a
    // constant that was published before this file existed.
    //
    // 23 is `PARAM_SET`, added when this app grew a parameter write. Its 168
    // came back from the derivation on the first run, which is the check
    // working: the entry was transcribed from `common.xml` and the constant
    // says whether the transcription was right.
    const published: Record<number, number> = {
      0: 50, 1: 124, 21: 159, 22: 220, 23: 168, 24: 24, 30: 39, 33: 104,
      36: 222, 65: 118, 74: 20, 76: 152, 77: 143, 148: 178, 253: 83,
    };
    for (const def of MAVLINK_MESSAGES) {
      expect(`${def.name}=${deriveCrcExtra(def)}`).toBe(`${def.name}=${published[def.id]}`);
    }
    expect(MAVLINK_MESSAGES).toHaveLength(Object.keys(published).length);
  });

  it('agrees with the CRC_EXTRA transcribed in the capture script', () => {
    for (const [id, extra] of Object.entries(fixture.crc_extra as Record<string, number>)) {
      expect(CRC_EXTRA.get(Number(id))).toBe(extra);
    }
  });

  it('excludes extension fields from the CRC, and includes them in the layout', () => {
    // The property that makes extensions extensions. Getting this flag wrong
    // is invisible in the offsets - the fields are still there and still in the
    // right place - and shows up only as a checksum that never validates, which
    // is why it is asserted directly rather than left to the frame tests.
    const statustext = MAVLINK_MESSAGES.find((def) => def.id === 253)!;
    const withExtension = deriveCrcExtra({
      ...statustext,
      fields: statustext.fields.map((field) =>
        field.name === 'id' ? { ...field, ext: false } : field,
      ),
    });
    expect(withExtension).not.toBe(83);
    // ...and it is still in the layout at its own offset.
    const layout = mavLayout(253)!;
    expect(layout.length).toBe(54);
    expect(layout.fields.find((field) => field.name === 'id')!.offset).toBe(51);
  });

  it('sorts non-extension fields by size and appends extensions unsorted', () => {
    // PARAM_VALUE is the message that makes the rule visible: a 16-byte char
    // array sorts as *one* byte, so it lands after two uint16_ts.
    const paramValue = mavLayout(22)!;
    expect(paramValue.fields.map((field) => `${field.name}@${field.offset}`)).toEqual([
      'param_value@0', 'param_count@4', 'param_index@6', 'param_id@8', 'param_type@24',
    ]);
    // SERVO_OUTPUT_RAW is the other shape: `port` is an ordinary uint8_t and is
    // sorted before the extension servos, so it sits *between* servo8 and
    // servo9 - a layout nobody would write down on purpose.
    const servo = mavLayout(36)!;
    expect(servo.fields.map((field) => field.name).slice(9, 12))
      .toEqual(['port', 'servo9_raw', 'servo10_raw']);
    expect(servo.length).toBe(37);
    // And the one whose field order is declaration order unchanged.
    expect(mavLayout(253)!.fields.map((field) => field.name))
      .toEqual(['severity', 'text', 'id', 'chunk_seq']);
  });

  it('computes the wire lengths the dialect defines', () => {
    const lengths: Record<number, number> = {
      0: 9, 1: 31, 21: 2, 22: 25, 24: 52, 30: 28, 33: 28, 36: 37,
      65: 42, 74: 20, 76: 33, 77: 10, 148: 78, 253: 54,
    };
    for (const [id, length] of Object.entries(lengths)) {
      expect(`${id}:${mavLayout(Number(id))!.length}`).toBe(`${id}:${length}`);
    }
  });
});

describe('frames the vehicle actually sent', () => {
  it('captured every message this app claims to read', () => {
    const have = new Set([...Object.keys(frames), ...Object.keys(packed)]);
    for (const name of [
      'HEARTBEAT', 'SYS_STATUS', 'PARAM_VALUE', 'GPS_RAW_INT', 'ATTITUDE',
      'GLOBAL_POSITION_INT', 'SERVO_OUTPUT_RAW', 'RC_CHANNELS', 'VFR_HUD',
      'STATUSTEXT', 'COMMAND_ACK', 'AUTOPILOT_VERSION',
    ]) {
      expect(have).toContain(name);
    }
  });

  it('validates the checksum of every frame in the fixture', () => {
    // This is the load-bearing one. A frame is accepted only if this app's
    // framing, its CRC-16/MCRF4XX and its derived CRC_EXTRA all match the
    // reference implementation's, for every message in the table.
    for (const { name, frame } of allCaptured()) {
      const { frames: got, issues } = new MavlinkDecoder().push(bytes(frame.frame_hex));
      expect(`${name}:${issues.length}`).toBe(`${name}:0`);
      expect(`${name}:${got.length}`).toBe(`${name}:1`);
      expect(`${name}:${got[0]!.msgid}`).toBe(`${name}:${frame.msgid}`);
      expect(`${name}:${got[0]!.name}`).toBe(`${name}:${name}`);
    }
  });

  it('decodes every payload to the values the capture recorded', () => {
    for (const [name, wanted] of Object.entries(expect_)) {
      const frame = name === 'STATUSTEXT_truncated'
        ? packed.STATUSTEXT![1]!
        : frames[name] ?? packed[name]?.[0];
      expect(frame, `no captured frame for ${name}`).toBeDefined();
      const decoded = decode(oneFrame(frame!.frame_hex));
      for (const [field, value] of Object.entries(wanted)) {
        expect(`${name}.${field}=${JSON.stringify(decoded[field])}`)
          .toBe(`${name}.${field}=${JSON.stringify(value)}`);
      }
    }
  });

  it('reads a parameter name out of its fixed-width char array', () => {
    const decoded = decode(oneFrame(frames.PARAM_VALUE!.frame_hex));
    // 16 bytes, NUL-padded. A decoder that kept the padding would draw a name
    // with invisible characters in it.
    expect(str(decoded, 'param_id')).toBe(expect_.PARAM_VALUE!.param_id);
    expect(str(decoded, 'param_id')).not.toContain('\0');
    expect(num(decoded, 'param_type')).toBe(9); // MAV_PARAM_TYPE_REAL32
  });

  it('reads the multi-byte arrays whole', () => {
    const version = decode(oneFrame(packed.AUTOPILOT_VERSION![0]!.frame_hex));
    expect(num(version, 'capabilities')).toBe(1);
    expect(num(version, 'uid')).toBe(0x15161718191a1b1c);
    expect(str(version, 'flight_custom_version')).toBe('1,2,3,4,5,6,7,8');
    expect(str(version, 'os_custom_version')).toBe('17,18,19,20,21,22,23,24');
  });
});

describe('MAVLink v2 truncation', () => {
  it('takes the length byte as the payload length, not the struct size', () => {
    const frame = oneFrame(frames.SERVO_OUTPUT_RAW!.frame_hex);
    expect(frame.version).toBe(2);
    expect(frame.structLength).toBe(37);
    // The vehicle sent twelve bytes of a thirty-seven byte message, because
    // every field after servo4 is zero. That is a normal frame.
    expect(frame.payload.length).toBe(12);
    const decoded = decode(frame);
    expect(num(decoded, 'servo1_raw')).toBeGreaterThan(0);
    expect(num(decoded, 'servo5_raw')).toBe(0);
    expect(num(decoded, 'servo16_raw')).toBe(0);
  });

  it('pads a frame truncated to two bytes of a fifty-four byte message', () => {
    const frame = oneFrame(packed.STATUSTEXT![1]!.frame_hex);
    expect(frame.structLength).toBe(54);
    expect(frame.payload.length).toBe(2);
    const decoded = decode(frame);
    expect(str(decoded, 'text')).toBe('x');
    expect(num(decoded, 'severity')).toBe(4);
    // The zero-valued extension fields were stripped, and reading them back as
    // zero is exactly what the sender meant. A decoder that required a full
    // struct would refuse a perfectly ordinary frame.
    expect(num(decoded, 'id')).toBe(0);
    expect(num(decoded, 'chunk_seq')).toBe(0);
  });

  it('reads a payload longer than its own definition from the prefix', () => {
    // **This test used to assert the opposite**, and the opposite was wrong.
    // It required `decode` to throw `MavProtocolError` on a payload longer than
    // the message it claimed to be — which reads as strictness and is in fact
    // the app refusing a legal frame. MAVLink grows a message by appending
    // fields after `<extensions/>`, the appended fields go on the wire after
    // every ordinary one, and they are **excluded from the CRC_EXTRA**; that
    // exclusion is the whole mechanism by which a dialect can add a field
    // without invalidating every ground station already in the field. So a
    // checksum that validates has already proved the *base* fields agree, and a
    // length past the table is a newer peer rather than a corrupt frame.
    //
    // Found on `SYS_STATUS`: this app carried the 31-byte pre-extension message
    // and PX4 1.17.0 sends 43, so a real vehicle's battery voltage, current and
    // sensor health were dropped on arrival — one warning line per frame, for as
    // long as the link was up. `dialect.test.ts` drives that exact frame.
    const frame = oneFrame(frames.HEARTBEAT!.frame_hex);
    const payload = new Uint8Array(frame.payload.length + 4);
    payload.set(frame.payload, 0);
    payload.set([0xaa, 0xbb, 0xcc, 0xdd], frame.payload.length);
    const longer: MavlinkFrame = { ...frame, payload };

    expect(bytesBeyondTable(longer)).toBe(4);
    // And what it cannot name changes nothing about what it can.
    expect(decode(longer)).toEqual(decode(frame));
  });

  it('counts no bytes past the table for a frame that fits it', () => {
    // The other half: `bytesBeyondTable` is a measurement, not an assumption.
    // A frame this app can describe in full reports zero, so nothing is
    // reported to a person about a peer that is not ahead of them.
    const frame = oneFrame(frames.HEARTBEAT!.frame_hex);
    expect(bytesBeyondTable(frame)).toBe(0);
    // v2's zero-truncation makes a *short* payload, which is not a negative
    // count of bytes past the end and must not be reported as one.
    const short: MavlinkFrame = {
      ...frame,
      payload: frame.payload.subarray(0, 4),
    };
    expect(bytesBeyondTable(short)).toBe(0);
  });
});

describe('what a damaged or unfamiliar link does', () => {
  it('refuses a frame whose checksum does not match', () => {
    const corrupt = bytes(frames.HEARTBEAT!.frame_hex);
    corrupt[corrupt.length - 1]! ^= 0xff;
    const { frames: got, issues } = new MavlinkDecoder().push(corrupt);
    expect(got).toEqual([]);
    expect(issues).toEqual([{ kind: 'checksum', msgid: 0 }]);
  });

  it('reports a message it has no definition for as unread, not as parsed', () => {
    // A v2 frame for message 4200, which this app does not carry. Without a
    // CRC_EXTRA its checksum cannot be verified and its length cannot be
    // trusted, so the honest answer is "unread" - counted and reported, never
    // guessed at.
    const body = Uint8Array.of(4, 0, 0, 1, 1, 1, 0x68, 0x10, 0, 1, 2, 3, 4);
    const out = new Uint8Array(body.length + 3);
    out[0] = 0xfd;
    out.set(body, 1);
    const { frames: got, issues } = new MavlinkDecoder().push(out);
    expect(got).toEqual([]);
    expect(issues).toEqual([{ kind: 'unknown-message', msgid: 4200 }]);
  });

  it('still finds a known message either side of an unknown one', () => {
    // The resynchronisation has to survive a stream that is mostly messages
    // this app cannot read, or a vehicle streaming an unfamiliar dialect would
    // look like a vehicle that had gone quiet.
    const unknown = Uint8Array.of(0xfd, 4, 0, 0, 1, 1, 1, 0x68, 0x10, 0, 1, 2, 3, 4, 0, 0);
    const stream = new Uint8Array(unknown.length * 2 + bytes(frames.HEARTBEAT!.frame_hex).length);
    stream.set(unknown, 0);
    stream.set(bytes(frames.HEARTBEAT!.frame_hex), unknown.length);
    stream.set(unknown, unknown.length + bytes(frames.HEARTBEAT!.frame_hex).length);
    const { frames: got, issues } = new MavlinkDecoder().push(stream);
    expect(got.map((frame) => frame.name)).toEqual(['HEARTBEAT']);
    expect(issues.filter((issue) => issue.kind === 'unknown-message')).toHaveLength(2);
  });

  it('holds a half-received frame rather than discarding it', () => {
    const whole = bytes(frames.HEARTBEAT!.frame_hex);
    const decoder = new MavlinkDecoder();
    expect(decoder.push(whole.subarray(0, 5)).frames).toEqual([]);
    expect(decoder.held).toBe(5);
    expect(decoder.push(whole.subarray(5)).frames.map((frame) => frame.name)).toEqual(['HEARTBEAT']);
  });
});

describe('MAVLink v1', () => {
  it('reads a v1 frame, which no fixture carries', () => {
    // Every captured frame is v2, so this path is covered only here - and it is
    // covered with this test's own framing rather than the module's, written
    // out longhand from the wire format the way the capture script is. An
    // implementation that only ever exchanged bytes with itself would agree
    // with itself about a wrong header just as happily.
    // Wire order for HEARTBEAT is custom_mode, type, autopilot, base_mode,
    // system_status, mavlink_version — custom_mode is the u32 and sorts first.
    // A quadrotor (2) on ArduPilot (3), armed (base_mode bit 7).
    const payload = Uint8Array.of(2, 0, 0, 0, 2, 3, 128, 4, 3);
    const body = Uint8Array.of(payload.length, 7, 1, 1, 0, ...payload);
    let crc = 0xffff;
    const acc = (c: number, byte: number): number => {
      let tmp = (byte & 0xff) ^ (c & 0xff);
      tmp = (tmp ^ (tmp << 4)) & 0xff;
      return ((c >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
    };
    for (const byte of body) crc = acc(crc, byte);
    crc = acc(crc, 50); // the HEARTBEAT CRC_EXTRA
    const frame = Uint8Array.of(0xfe, ...body, crc & 0xff, (crc >> 8) & 0xff);

    const { frames: got, issues } = new MavlinkDecoder().push(frame);
    expect(issues).toEqual([]);
    expect(got).toHaveLength(1);
    expect(got[0]!.version).toBe(1);
    expect(got[0]!.systemId).toBe(1);
    expect(got[0]!.seq).toBe(7);
    expect(num(decode(got[0]!), 'custom_mode')).toBe(2);
    expect(heartbeatOf(got[0]!).armed).toBe(true);
    expect(heartbeatOf(got[0]!).typeName).toBe('quadrotor');
    expect(heartbeatOf(got[0]!).autopilotName).toBe('ArduPilot');
  });
});

// ---- the client ------------------------------------------------------------

class Wire implements MavLink {
  readonly written: Uint8Array[] = [];
  private readonly handlers = new Set<(chunk: Uint8Array) => void>();

  write(bytes: Uint8Array): void {
    this.written.push(bytes);
  }

  onData(handler: (chunk: Uint8Array) => void): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  onClose(): () => void {
    return () => {};
  }

  onError(): () => void {
    return () => {};
  }

  deliver(chunk: Uint8Array): void {
    for (const handler of this.handlers) handler(chunk);
  }

  /** Every message id that reached the wire. */
  get msgids(): number[] {
    return this.written.map((frame) => new MavlinkDecoder().push(frame).frames[0]!.msgid);
  }
}

describe('the client that may send exactly two things', () => {
  it('cannot write anything until the vehicle has spoken', () => {
    const client = new MavlinkClient(new Wire());
    // A MAVLink request is addressed to the vehicle's own system id, and the
    // only way to learn it is to listen. This is why MAVLink is detected by
    // listening and never by probing.
    expect(() => client.requestParameterList()).toThrow(/before hearing a heartbeat/);
    client.close();
  });

  it('addresses its requests to the system the heartbeat came from', () => {
    const wire = new Wire();
    const client = new MavlinkClient(wire);
    wire.deliver(bytes(frames.HEARTBEAT!.frame_hex));
    client.requestParameterList();

    const sent = new MavlinkDecoder().push(wire.written[0]!).frames[0]!;
    const heartbeat = heartbeatOf(oneFrame(frames.HEARTBEAT!.frame_hex));
    expect(sent.name).toBe('PARAM_REQUEST_LIST');
    expect(num(decode(sent), 'target_system')).toBe(heartbeat.systemId);
    expect(num(decode(sent), 'target_component')).toBe(heartbeat.componentId);
    expect(sent.version).toBe(2);
    client.close();
  });

  it('sends only PARAM_REQUEST_LIST and COMMAND_LONG, ever', () => {
    // The safety property, measured rather than asserted in a comment. MAVLink's
    // COMMAND_LONG is the same message that carries arm, takeoff and
    // DO_SET_HOME, and this app has to be unable to send those. Driving the
    // whole write surface and looking at what left is the only way to know.
    const wire = new Wire();
    const client = new MavlinkClient(wire);
    wire.deliver(bytes(frames.HEARTBEAT!.frame_hex));
    client.requestParameterList();
    client.requestMessageInterval(30, 4);
    client.requestMessageInterval(33, 2);

    expect(wire.msgids).toEqual([21, 76, 76]);
    for (const frame of wire.written) {
      const decoded = decode(new MavlinkDecoder().push(frame).frames[0]!);
      if (frame[0] === 0xfd) {
        const msgid = frame[7]! | (frame[8]! << 8) | (frame[9]! << 16);
        if (msgid === 76) {
          // Every COMMAND_LONG this app can build carries this and only this.
          expect(num(decoded, 'command')).toBe(MAV_CMD_SET_MESSAGE_INTERVAL);
        }
      }
    }
    client.close();
  });

  it('asks for a stream rate in the units the message takes', () => {
    const wire = new Wire();
    const client = new MavlinkClient(wire);
    wire.deliver(bytes(frames.HEARTBEAT!.frame_hex));
    client.requestMessageInterval(30, 4);

    const decoded = decode(new MavlinkDecoder().push(wire.written[0]!).frames[0]!);
    // param1 is the *message id* and param2 the interval in microseconds. A
    // first draft that puts zero in param1 asks the vehicle for HEARTBEAT
    // instead, which looks like it worked.
    expect(num(decoded, 'param1')).toBe(30);
    expect(num(decoded, 'param2')).toBe(250000);
    client.close();
  });

  it('refuses to ask for a message it could not read back', () => {
    const wire = new Wire();
    const client = new MavlinkClient(wire);
    wire.deliver(bytes(frames.HEARTBEAT!.frame_hex));
    expect(() => client.requestMessageInterval(4200, 4)).toThrow(/no definition/);
    expect(() => client.requestMessageInterval(30, 0)).toThrow(RangeError);
    expect(() => client.requestMessageInterval(30, 100000)).toThrow(RangeError);
    client.close();
  });

  it('answers in the dialect the vehicle was heard speaking', () => {
    // Replying in v2 to a vehicle that spoke v1 is assuming a capability that
    // was never demonstrated.
    const wire = new Wire();
    const client = new MavlinkClient(wire);
    const payload = Uint8Array.of(0, 2, 0, 0, 0, 0, 0, 0, 3);
    const body = Uint8Array.of(payload.length, 0, 1, 1, 0, ...payload);
    let crc = 0xffff;
    const acc = (c: number, byte: number): number => {
      let tmp = (byte & 0xff) ^ (c & 0xff);
      tmp = (tmp ^ (tmp << 4)) & 0xff;
      return ((c >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
    };
    for (const byte of body) crc = acc(crc, byte);
    crc = acc(crc, 50);
    wire.deliver(Uint8Array.of(0xfe, ...body, crc & 0xff, (crc >> 8) & 0xff));

    client.requestParameterList();
    expect(wire.written[0]![0]).toBe(0xfe);
    client.close();
  });

  it('reports the armed state from the heartbeat, and nothing else', () => {
    const disarmed = heartbeatOf(oneFrame(frames.HEARTBEAT!.frame_hex));
    expect(disarmed.armed).toBe((disarmed.baseMode & MAV_MODE_FLAG_SAFETY_ARMED) !== 0);
    expect(disarmed.autopilotName).toContain('ArduPilot');
    expect(disarmed.mavlinkVersion).toBe(3);
  });
});

describe('the checksum primitive', () => {
  it('reproduces the checksum pymavlink put on a real frame', () => {
    // Taken from the fixture rather than typed from memory: a hand-written body
    // is a second guess at the same bytes, and one of them will be wrong.
    const frame = bytes(frames.HEARTBEAT!.frame_hex);
    const length = frame[1]!;
    const body = frame.subarray(1, 10 + length);
    const embedded = frame[10 + length]! | (frame[11 + length]! << 8);
    let crc = 0xffff;
    for (const byte of body) crc = crcAccumulate(crc, byte);
    expect(crcAccumulate(crc, 50)).toBe(embedded);
    expect(embedded).toBe(0x67d6);
  });

  it('counts the extra once, not twice', () => {
    // The mistake this function's shape prevents: appending the CRC_EXTRA to
    // the buffer *and* passing it here. It is the natural way to write it and it
    // produces a checksum no vehicle agrees with, which on a live link is
    // indistinguishable from a bad cable.
    const frame = bytes(frames.HEARTBEAT!.frame_hex);
    const length = frame[1]!;
    const body = frame.subarray(1, 10 + length);
    const embedded = frame[10 + length]! | (frame[11 + length]! << 8);
    const withExtraAppended = Uint8Array.of(...body, 50);
    let crc = 0xffff;
    for (const byte of withExtraAppended) crc = crcAccumulate(crc, byte);
    expect(crcAccumulate(crc, 50)).not.toBe(embedded);
  });
});
