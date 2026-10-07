import {
  CRC_EXTRA,
  frameChecksum,
  mavLayout,
  MavlinkDecoder,
  paramValueOf,
} from '../src/protocol/mavlink';
import type { Transport } from '../src/transport/types';
import fixture from './fixtures/mavlink.json';

/**
 * A vehicle that talks, and only talks.
 *
 * Shared between the tests about MAVLink itself and the ones about *drawing the
 * right conclusion* from what arrives on a serial port — both want the same
 * vehicle, and it should be the one whose bytes came out of this repository's
 * own ArduPilot stand-in rather than a second implementation of MAVLink written
 * by the test.
 *
 * So the frames below are the captured ones, byte for byte. Where a test needs
 * a variation the capture does not contain — a vehicle that is *armed*, a
 * parameter table with more than one row — the captured frame is **patched in
 * place** and its checksum recomputed, rather than a new frame being assembled
 * from a field layout. Patching is the smaller assumption: the payload stays the
 * length the reference implementation made it, and the only thing that moves is
 * the field under test.
 *
 * The recomputation does use this app's `frameChecksum`. That is a real
 * circularity and it is worth naming: it means these helpers cannot catch a
 * wrong checksum. It does not need to — `mavlink.test.ts` reproduces `0x67d6`
 * for a captured heartbeat body from first principles, and every fixture frame
 * here is decoded by a decoder that would reject a wrong one.
 */

interface CapturedFrame {
  readonly msgid: number;
  readonly frame_hex: string;
  readonly source: string;
}

const frames = fixture.frames as unknown as Record<string, CapturedFrame>;
const packed = fixture.packed as unknown as Record<string, CapturedFrame[]>;

/** A captured frame by name, from either block — the nine the vehicle streamed
 *  and the four pymavlink built for the messages it never sends. */
function captured(name: string): CapturedFrame {
  const streamed = frames[name];
  if (streamed !== undefined) return streamed;
  const built = packed[name]?.[0];
  if (built !== undefined) return built;
  throw new Error(`no captured frame ${name}`);
}

export function bytes(hex: string): Uint8Array {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(hex.substr(i * 2, 2), 16);
  return out;
}

export function hex(value: Uint8Array): string {
  return Array.from(value, (byte) => byte.toString(16).padStart(2, '0')).join('');
}

/** A frame from the `packed` block specifically.
 *
 *  `captured()` prefers the streamed copy, and for `COMMAND_ACK` the streamed
 *  copy is two bytes long: v2 truncated the trailing zeros, so the frame the
 *  vehicle actually sent carries the command id and *no result field at all*.
 *  A test that needs to vary the result has to start from the packed frame,
 *  which pymavlink built at full length. */
export function packedCaptured(name: string): CapturedFrame {
  const built = packed[name]?.[0];
  if (built === undefined) throw new Error(`no packed frame ${name}`);
  return built;
}

/** A captured frame, as it arrived. */
export function capturedFrame(name: string): Uint8Array {
  return bytes(captured(name).frame_hex);
}

/** A captured frame with fields overwritten and its checksum made true again.
 *
 *  `header` patches the framing bytes rather than the payload — the system id
 *  lives there, and so does the sequence number. They are inside the checksum's
 *  range (the body runs from the length byte), so they are recomputed over just
 *  the same as a field is. */
export function patchedFrame(
  name: string,
  patch: Readonly<Record<string, number | string>>,
  headerPatch: { readonly systemId?: number; readonly componentId?: number; readonly seq?: number } = {},
): Uint8Array {
  return patchCaptured(captured(name), patch, headerPatch);
}

/** The same, starting from the `packed` frame rather than the streamed one —
 *  which is the only way to reach a field v2 truncation removed from the frame
 *  the vehicle actually sent. */
export function patchedPackedFrame(
  name: string,
  patch: Readonly<Record<string, number | string>>,
  headerPatch: { readonly systemId?: number; readonly componentId?: number; readonly seq?: number } = {},
): Uint8Array {
  return patchCaptured(packedCaptured(name), patch, headerPatch);
}

function patchCaptured(
  base: CapturedFrame,
  patch: Readonly<Record<string, number | string>>,
  headerPatch: { readonly systemId?: number; readonly componentId?: number; readonly seq?: number },
): Uint8Array {
  const frame = bytes(base.frame_hex);
  const msgid = base.msgid;
  const layout = mavLayout(msgid);
  if (layout === undefined) throw new Error(`no layout for message ${msgid}`);

  const v2 = frame[0] === 0xfd;
  const header = v2 ? 10 : 6;
  const payload = frame.slice(header, frame.length - 2);

  for (const [field, value] of Object.entries(patch)) {
    const where = layout.fields.find((item) => item.name === field);
    if (where === undefined) throw new Error(`${layout.name} has no field ${field}`);
    const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
    if (typeof value === 'string') {
      // A char array, NUL-padded to its full width the way a vehicle writes one.
      for (let i = 0; i < where.size; i++) payload[where.offset + i] = 0;
      const text = value.slice(0, where.size);
      for (let i = 0; i < text.length; i++) payload[where.offset + i] = text.charCodeAt(i);
      continue;
    }
    switch (where.type) {
      case 'uint8_t': view.setUint8(where.offset, value); break;
      case 'int8_t': view.setInt8(where.offset, value); break;
      case 'uint16_t': view.setUint16(where.offset, value, true); break;
      case 'int16_t': view.setInt16(where.offset, value, true); break;
      case 'uint32_t': view.setUint32(where.offset, value, true); break;
      case 'int32_t': view.setInt32(where.offset, value, true); break;
      case 'uint64_t': view.setBigUint64(where.offset, BigInt(value), true); break;
      case 'float': view.setFloat32(where.offset, value, true); break;
      case 'double': view.setFloat64(where.offset, value, true); break;
      default: throw new Error(`patching a ${where.type} is not something this helper does`);
    }
  }

  const out = new Uint8Array(frame.length);
  out.set(frame.slice(0, header), 0);
  out.set(payload, header);
  // v2 is `fd len incompat compat seq sysid compid msgid[3]`; v1 has no
  // incompat/compat and a one-byte message id.
  if (headerPatch.seq !== undefined) out[4] = headerPatch.seq;
  if (headerPatch.systemId !== undefined) out[v2 ? 5 : 3] = headerPatch.systemId;
  if (headerPatch.componentId !== undefined) out[v2 ? 6 : 4] = headerPatch.componentId;
  // Everything from the length byte to the last payload byte, then the
  // message's own CRC_EXTRA once, folded into the running value.
  const body = out.slice(1, out.length - 2);
  const crc = frameChecksum(body, CRC_EXTRA.get(msgid)!);
  out[out.length - 2] = crc & 0xff;
  out[out.length - 1] = (crc >> 8) & 0xff;

  // Read it back before handing it out. A patched frame that its own decoder
  // rejects would make a failing test look like a bug in the thing under test.
  const check = new MavlinkDecoder().push(out);
  if (check.frames.length !== 1) {
    throw new Error(`the patched ${layout.name} frame was rejected by the decoder`);
  }
  return out;
}

/** The vehicle, at the other end of a wire a test owns. */
export class MavFixtureVehicle {
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();
  readonly written: Uint8Array[] = [];
  closed = false;

  /** What the vehicle is streaming. Names into the fixture. */
  stream = ['HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'SYS_STATUS', 'VFR_HUD', 'RC_CHANNELS', 'SERVO_OUTPUT_RAW'];

  /** The parameter table it answers `PARAM_REQUEST_LIST` with. Its own module
   *  answer says there are forty and sends one; that is worth a test of its
   *  own, so this starts empty and a test fills it in. */
  parameters: Array<{ id: string; value: number; type: number }> = [];

  /** Set false to model a vehicle that will not enumerate its parameters. */
  answersParameterList = true;

  /** What the vehicle *says* it has, when that differs from what it sends. The
   *  captured `PARAM_VALUE` claims forty and the stand-in sends one, which is
   *  the shape of a half-read table and worth being able to reproduce. */
  declaredParameterCount: number | null = null;

  /** How the vehicle is armed, as the heartbeat says. */
  armed = false;

  /** What the vehicle answers a `COMMAND_LONG` with.
   *
   *  `0` is the captured ack — the reference implementation's own answer to
   *  `MAV_CMD_SET_MESSAGE_INTERVAL`, which is what a real autopilot sends when
   *  it agrees. `3` is `MAV_RESULT_UNSUPPORTED`, which is what it sends when it
   *  does not: a vehicle that has the message but not that command, or a build
   *  with the stream-rate path compiled out. Both are ordinary answers and the
   *  caller cannot tell them apart from the *absence* of an ack, which is why
   *  the distinction has to be read out of the payload. */
  commandAckResult = 0;

  /** Set false to model a vehicle that takes a `COMMAND_LONG` and answers
   *  nothing at all — which is not the same as refusing it, and is the third
   *  thing a request can come to. */
  answersCommandLong = true;

  /** Whether the vehicle answers a `PARAM_SET` with a `PARAM_VALUE` at all.
   *
   *  False models the case MAVLink cannot distinguish from agreement by
   *  silence alone: the frame went out and nothing came back, so the write
   *  stands unresolved. That is a real outcome on a lossy link and the app has
   *  to be able to say so rather than assume the best. */
  answersParamSet = true;

  /** Whether the vehicle *takes* the value it was sent.
   *
   *  True is an ordinary writable parameter. **False is the shape ArduPilot
   *  actually uses for a read-only one**: `GCS_Param.cpp` does not error, it
   *  sends the parameter's own current value back as an ordinary `PARAM_VALUE`,
   *  and a client that only looked for "did a value come back" would record
   *  that as agreement. So this knob is what makes the read-back test able to
   *  fail — set false and an app that trusted the mere arrival of a
   *  `PARAM_VALUE` reports a write that never happened. */
  acceptsParamSet = true;

  /** The last frame `say()` put on the wire, so a test can mangle a copy of it. */
  lastSaid: Uint8Array | null = null;

  get probeWrites(): Uint8Array[] {
    return this.written;
  }

  /** One frame, delivered as a vehicle would — on its own schedule, not in
   *  answer to anything. */
  say(
    name: string,
    patch?: Readonly<Record<string, number | string>>,
    headerPatch?: { readonly systemId?: number; readonly componentId?: number },
  ): void {
    const frame =
      patch === undefined && headerPatch === undefined
        ? capturedFrame(name)
        : patchedFrame(name, patch ?? {}, headerPatch ?? {});
    this.sayBytes(frame);
  }

  /** Bytes straight onto the wire, for frames no capture contains. */
  sayBytes(frame: Uint8Array): void {
    this.lastSaid = frame;
    this.deliver(frame);
  }

  /**
   * Heartbeats on a period, the way a vehicle actually does.
   *
   * A test that hands detection a heartbeat before it is listening has tested
   * nothing: detection's whole job is to notice a vehicle that announces itself
   * on its own schedule. This is what makes that real, and it returns its own
   * stop function so a test cannot leave a timer running into the next one.
   */
  announceEvery(intervalMs = 50): () => void {
    const timer = setInterval(() => this.sayArmed(this.armed), intervalMs);
    return () => clearInterval(timer);
  }

  sayArmed(armed: boolean): void {
    // base_mode bit 7 is MAV_MODE_FLAG_SAFETY_ARMED.
    this.armed = armed;
    this.say('HEARTBEAT', { base_mode: armed ? 128 : 0 });
  }

  private deliver(frame: Uint8Array): void {
    queueMicrotask(() => {
      if (this.closed) return;
      for (const handler of this.dataHandlers) handler(frame);
    });
  }

  write(frame: Uint8Array): void {
    if (this.closed) return;
    this.written.push(frame);
    const decoded = new MavlinkDecoder().push(frame);
    const first = decoded.frames[0];
    if (first === undefined) return;

    if (first.msgid === 21 && this.answersParameterList) {
      // One `PARAM_VALUE` per index, in order, the way a vehicle drips them out.
      const declared = this.declaredParameterCount ?? this.parameters.length;
      for (let index = 0; index < this.parameters.length; index++) {
        const parameter = this.parameters[index]!;
        this.deliver(
          patchedFrame('PARAM_VALUE', {
            param_id: parameter.id,
            param_value: parameter.value,
            param_type: parameter.type,
            param_count: declared,
            param_index: index,
          }),
        );
      }
    }

    if (first.msgid === 23 && this.answersParamSet) {
      // `PARAM_SET`. The vehicle echoes a `PARAM_VALUE` — and that is the whole
      // of MAVLink's answer, because there is no separate ack message. Which
      // value comes back is the vehicle's decision, and this is where a real
      // autopilot's two behaviours live: it takes the value, or it keeps its
      // own. ArduPilot does the second for a read-only parameter, silently.
      const sent = paramValueOf(first);
      const held = this.parameters.find((parameter) => parameter.id === sent.paramId);
      const value = this.acceptsParamSet ? sent.paramValue : (held?.value ?? sent.paramValue);
      if (this.acceptsParamSet && held !== undefined) held.value = value;
      this.deliver(
        patchedFrame('PARAM_VALUE', {
          param_id: sent.paramId,
          param_value: value,
          param_type: sent.paramType,
          param_count: this.parameters.length,
          param_index: this.parameters.findIndex((parameter) => parameter.id === sent.paramId),
        }),
      );
    }

    if (first.msgid === 76 && this.answersCommandLong) {
      // `COMMAND_LONG`. This app only ever sends one command,
      // `MAV_CMD_SET_MESSAGE_INTERVAL`, so the answer is the ack for 511.
      if (this.commandAckResult === 0) {
        // A vehicle that agrees sends the captured ack, byte for byte: it is
        // the reference implementation's own answer and v2 has trimmed its
        // trailing zeros, which is exactly what a real one looks like.
        this.say('COMMAND_ACK');
      } else {
        // A vehicle that refuses has to say so, so the result byte exists and
        // the frame is the full-length one pymavlink built.
        this.sayBytes(patchedPackedFrame('COMMAND_ACK', { result: this.commandAckResult }));
      }
    }
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

/** The vehicle wearing the `Transport` a session — or a page — needs. */
export class MavFixtureTransport implements Transport {
  readonly info = {
    kind: 'serial' as const,
    label: 'USB serial',
    detail: 'an ArduPilot stand-in on the other end',
  };
  private open_ = false;
  readonly vehicle = new MavFixtureVehicle();

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
    this.vehicle.write(bytes);
  }
  onData(handler: (chunk: Uint8Array) => void): () => void {
    return this.vehicle.onData(handler);
  }
  onClose(handler: (reason: string) => void): () => void {
    return this.vehicle.onClose(handler);
  }
  onError(handler: (message: string) => void): () => void {
    return this.vehicle.onError(handler);
  }
}
