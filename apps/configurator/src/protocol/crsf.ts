/**
 * CRSF, the wire an ExpressLRS radio module speaks.
 *
 * The host side of `aerial_kit.comms.crsf` and of the AK firmware's
 * `ak_crsf*.c`, checked against the same vectors (`protocols/vectors/crsf.json`).
 * Layouts follow ExpressLRS 4.1.0: `src/include/crsf_protocol.h` for frames and
 * `src/lib/CrsfProtocol/CRSFEndpoint.cpp` for the parameter ("Lua") menu.
 *
 *   [address] [length = type + payload + crc] [type] [payload...] [crc8 dvb-s2]
 *
 * Extended frames (type >= 0x28) start their payload with destination and
 * origin addresses. Telemetry fields are big-endian.
 */

export const CRSF_SYNC = 0xc8;
export const CRSF_MAX_FRAME = 64;

export const Address = {
  BROADCAST: 0x00,
  USB: 0x10,
  FLIGHT_CONTROLLER: 0xc8,
  RADIO_TRANSMITTER: 0xea,
  CRSF_RECEIVER: 0xec,
  CRSF_TRANSMITTER: 0xee,
  ELRS_LUA: 0xef,
} as const;

export const FrameType = {
  GPS: 0x02,
  VARIO: 0x07,
  BATTERY: 0x08,
  BARO_ALTITUDE: 0x09,
  LINK_STATISTICS: 0x14,
  RC_CHANNELS_PACKED: 0x16,
  ATTITUDE: 0x1e,
  FLIGHT_MODE: 0x21,
  DEVICE_PING: 0x28,
  DEVICE_INFO: 0x29,
  PARAMETER_SETTINGS_ENTRY: 0x2b,
  PARAMETER_READ: 0x2c,
  PARAMETER_WRITE: 0x2d,
  ELRS_STATUS: 0x2e,
  COMMAND: 0x32,
  MSP_REQ: 0x7a,
  MSP_RESP: 0x7b,
  MSP_WRITE: 0x7c,
} as const;

/** Addresses a frame may start with (ELRS's handset parser accepts all three). */
const SYNC_BYTES = new Set<number>([Address.FLIGHT_CONTROLLER, Address.RADIO_TRANSMITTER, Address.CRSF_TRANSMITTER]);

export const CHANNEL_MIN = 172;
export const CHANNEL_MID = 992;
export const CHANNEL_MAX = 1811;

/** A byte read that is 0 past the end: frames are length-checked before their fields are read. */
const u8 = (a: Uint8Array | readonly number[], i: number): number => a[i] ?? 0;

export function crc8DvbS2(bytes: Uint8Array): number {
  let crc = 0;
  for (const b of bytes) {
    crc ^= b;
    for (let i = 0; i < 8; i++) crc = crc & 0x80 ? ((crc << 1) ^ 0xd5) & 0xff : (crc << 1) & 0xff;
  }
  return crc;
}

export interface CrsfFrame {
  readonly address: number;
  readonly type: number;
  readonly payload: Uint8Array;
}

export function isExtended(type: number): boolean {
  return type >= 0x28 && type <= 0x96;
}

export function encodeFrame(type: number, payload: Uint8Array, address: number = CRSF_SYNC): Uint8Array {
  if (payload.length > CRSF_MAX_FRAME - 4) throw new RangeError(`CRSF payload too long: ${payload.length}`);
  const out = new Uint8Array(payload.length + 4);
  out[0] = address;
  out[1] = payload.length + 2;
  out[2] = type;
  out.set(payload, 3);
  out[out.length - 1] = crc8DvbS2(out.subarray(2, out.length - 1));
  return out;
}

/** An extended frame: payload = [destination, origin, ...body]. */
export function encodeExtended(type: number, destination: number, origin: number, body: Uint8Array = new Uint8Array(),
  address: number = CRSF_SYNC): Uint8Array {
  const payload = new Uint8Array(body.length + 2);
  payload[0] = destination;
  payload[1] = origin;
  payload.set(body, 2);
  return encodeFrame(type, payload, address);
}

/** Byte-fed: a truncated or corrupted frame is a nothing, never a wrong answer. */
export class CrsfDecoder {
  private buf: number[] = [];
  crcErrors = 0;

  push(chunk: Uint8Array): CrsfFrame[] {
    const out: CrsfFrame[] = [];
    for (const b of chunk) this.buf.push(b);
    for (;;) {
      while (this.buf.length > 0 && !SYNC_BYTES.has(this.buf[0]!)) this.buf.shift();
      if (this.buf.length < 2) break;
      const len = this.buf[1]!;
      if (len < 2 || len > CRSF_MAX_FRAME - 2) {
        this.buf.shift();
        continue;
      }
      if (this.buf.length < len + 2) break;
      const raw = Uint8Array.from(this.buf.slice(0, len + 2));
      if (crc8DvbS2(raw.subarray(2, raw.length - 1)) !== u8(raw, raw.length - 1)) {
        this.crcErrors++;
        this.buf.shift();
        continue;
      }
      this.buf.splice(0, len + 2);
      out.push({ address: u8(raw, 0), type: u8(raw, 2), payload: raw.slice(3, raw.length - 1) });
    }
    return out;
  }
}

// ----------------------------------------------------------------- channels

export function packChannels(channels: readonly number[]): Uint8Array {
  if (channels.length !== 16) throw new RangeError('CRSF carries exactly 16 channels');
  const out = new Uint8Array(22);
  let bit = 0;
  for (const raw of channels) {
    const v = Math.max(0, Math.min(2047, Math.round(raw)));
    for (let i = 0; i < 11; i++, bit++) if (v & (1 << i)) out[bit >> 3] = u8(out, bit >> 3) | (1 << (bit & 7));
  }
  return out;
}

export function unpackChannels(payload: Uint8Array): number[] {
  if (payload.length < 22) throw new RangeError('short RC channels payload');
  const out: number[] = [];
  let bit = 0;
  for (let c = 0; c < 16; c++) {
    let v = 0;
    for (let i = 0; i < 11; i++, bit++) if (u8(payload, bit >> 3) & (1 << (bit & 7))) v |= 1 << i;
    out.push(v);
  }
  return out;
}

// ---------------------------------------------------------------- telemetry

export interface LinkStatistics {
  readonly uplinkRssi1Dbm: number;
  readonly uplinkRssi2Dbm: number;
  readonly uplinkLinkQuality: number;
  readonly uplinkSnrDb: number;
  readonly activeAntenna: number;
  readonly rfMode: number;
  readonly uplinkTxPower: number;
  readonly downlinkRssiDbm: number;
  readonly downlinkLinkQuality: number;
  readonly downlinkSnrDb: number;
}

export type Telemetry =
  | { kind: 'link'; value: LinkStatistics }
  | { kind: 'attitude'; pitchRad: number; rollRad: number; yawRad: number }
  | { kind: 'battery'; voltageV: number; currentA: number; capacityMah: number; remainingPercent: number }
  | { kind: 'gps'; latitudeDeg: number; longitudeDeg: number; groundspeedMps: number; headingDeg: number;
      altitudeM: number; satellites: number }
  | { kind: 'barometer'; altitudeM: number; verticalSpeedMps: number }
  | { kind: 'flightMode'; mode: string };

const i8 = (b: number) => (b > 127 ? b - 256 : b);

export function cString(bytes: Uint8Array, at: number): [string, number] {
  let end = at;
  while (end < bytes.length && bytes[end] !== 0) end++;
  return [new TextDecoder().decode(bytes.subarray(at, end)), end + 1];
}

export function decodeTelemetry(frame: CrsfFrame): Telemetry | null {
  const p = frame.payload;
  const v = new DataView(p.buffer, p.byteOffset, p.byteLength);
  switch (frame.type) {
    case FrameType.LINK_STATISTICS:
      if (p.length < 10) return null;
      return { kind: 'link', value: {
        uplinkRssi1Dbm: -u8(p, 0), uplinkRssi2Dbm: -u8(p, 1), uplinkLinkQuality: u8(p, 2), uplinkSnrDb: i8(u8(p, 3)),
        activeAntenna: u8(p, 4), rfMode: u8(p, 5), uplinkTxPower: u8(p, 6), downlinkRssiDbm: -u8(p, 7),
        downlinkLinkQuality: u8(p, 8), downlinkSnrDb: i8(u8(p, 9)) } };
    case FrameType.ATTITUDE:   // pitch, roll, yaw; 1e-4 rad
      if (p.length < 6) return null;
      return { kind: 'attitude', pitchRad: v.getInt16(0) / 1e4, rollRad: v.getInt16(2) / 1e4, yawRad: v.getInt16(4) / 1e4 };
    case FrameType.BATTERY:
      if (p.length < 8) return null;
      return { kind: 'battery', voltageV: v.getUint16(0) / 10, currentA: v.getUint16(2) / 10,
        capacityMah: (u8(p, 4) << 16) | (u8(p, 5) << 8) | u8(p, 6), remainingPercent: u8(p, 7) };
    case FrameType.GPS:
      if (p.length < 15) return null;
      return { kind: 'gps', latitudeDeg: v.getInt32(0) / 1e7, longitudeDeg: v.getInt32(4) / 1e7,
        groundspeedMps: v.getUint16(8) / 36, headingDeg: v.getUint16(10) / 100,
        altitudeM: v.getUint16(12) - 1000, satellites: u8(p, 14) };
    case FrameType.BARO_ALTITUDE: {
      if (p.length < 4) return null;
      const raw = v.getUint16(0);
      const altitudeM = raw & 0x8000 ? raw & 0x7fff : (raw - 10000) / 10;
      return { kind: 'barometer', altitudeM, verticalSpeedMps: v.getInt16(2) / 100 };
    }
    case FrameType.FLIGHT_MODE:
      return { kind: 'flightMode', mode: cString(p, 0)[0] };
    default:
      return null;
  }
}

// ------------------------------------------------- device and parameter menu

export interface DeviceInfo {
  readonly destination: number;
  readonly origin: number;
  readonly name: string;
  readonly serial: number;
  readonly hardwareVersion: number;
  readonly softwareVersion: number;
  readonly parameterCount: number;
  readonly parameterVersion: number;
}

/** `[dest][origin][name\0][serial u32][hw u32][sw u32][field count][param version]`, big-endian. */
export function decodeDeviceInfo(frame: CrsfFrame): DeviceInfo | null {
  if (frame.type !== FrameType.DEVICE_INFO || frame.payload.length < 3) return null;
  const p = frame.payload;
  const [name, at] = cString(p, 2);
  if (p.length < at + 14) return null;
  const v = new DataView(p.buffer, p.byteOffset, p.byteLength);
  return { destination: u8(p, 0), origin: u8(p, 1), name, serial: v.getUint32(at), hardwareVersion: v.getUint32(at + 4),
    softwareVersion: v.getUint32(at + 8), parameterCount: u8(p, at + 12), parameterVersion: u8(p, at + 13) };
}

export const ParamType = {
  UINT8: 0, INT8: 1, UINT16: 2, INT16: 3, UINT32: 4, INT32: 5, FLOAT: 8,
  TEXT_SELECTION: 9, STRING: 10, FOLDER: 11, INFO: 12, COMMAND: 13, OUT_OF_RANGE: 127,
} as const;

/** Command steps (ELRS `lcs*`). */
export const CommandStep = { IDLE: 0, CLICK: 1, EXECUTING: 2, ASK_CONFIRM: 3, CONFIRMED: 4, CANCEL: 5, QUERY: 6 } as const;

export type Parameter =
  | { kind: 'selection'; id: number; parent: number; name: string; hidden: boolean; options: string[];
      value: number; min: number; max: number; units: string }
  | { kind: 'number'; id: number; parent: number; name: string; hidden: boolean; type: number;
      value: number; min: number; max: number; units: string }
  | { kind: 'folder'; id: number; parent: number; name: string; hidden: boolean; children: number[] }
  | { kind: 'info' | 'string'; id: number; parent: number; name: string; hidden: boolean; value: string }
  | { kind: 'command'; id: number; parent: number; name: string; hidden: boolean; step: number;
      timeoutMs: number; info: string }
  | { kind: 'unsupported'; id: number; parent: number; name: string; hidden: boolean; type: number };

/** One PARAMETER_SETTINGS_ENTRY chunk: `[dest][origin][field id][chunks remaining][data...]`. */
export interface ParameterChunk {
  readonly origin: number;
  readonly field: number;
  readonly chunksRemaining: number;
  readonly data: Uint8Array;
}

export function decodeParameterChunk(frame: CrsfFrame): ParameterChunk | null {
  if (frame.type !== FrameType.PARAMETER_SETTINGS_ENTRY || frame.payload.length < 4) return null;
  const p = frame.payload;
  return { origin: u8(p, 1), field: u8(p, 2), chunksRemaining: u8(p, 3), data: p.slice(4) };
}

/**
 * The reassembled entry body (chunk data concatenated):
 * `[parent][type | 0x80 hidden][name\0][type-specific...]`, as
 * `CRSFEndpoint::sendParameter` writes it.
 */
export function decodeParameter(id: number, body: Uint8Array): Parameter {
  const parent = u8(body, 0);
  const hidden = (u8(body, 1) & 0x80) !== 0;
  const type = u8(body, 1) & 0x7f;
  const [name, at] = cString(body, 2);
  const base = { id, parent, name, hidden };
  switch (type) {
    case ParamType.TEXT_SELECTION: {
      const [options, next] = cString(body, at);
      return { kind: 'selection', ...base, options: options.split(';'), value: u8(body, next),
        min: u8(body, next + 1), max: u8(body, next + 2), units: cString(body, next + 4)[0] };
    }
    case ParamType.UINT8:
    case ParamType.INT8: {
      const r = (b: number) => (type === ParamType.INT8 ? i8(b) : b);
      return { kind: 'number', ...base, type, value: r(u8(body, at)), min: r(u8(body, at + 1)), max: r(u8(body, at + 2)),
        units: cString(body, at + 4)[0] };
    }
    case ParamType.UINT16:
    case ParamType.INT16: {
      // ExpressLRS memcpy()s its native (little-endian) struct here, although its
      // PARAMETER_WRITE reads the value big-endian. Read what it sends.
      const v = new DataView(body.buffer, body.byteOffset + at, 6);
      const r = (o: number) => (type === ParamType.INT16 ? v.getInt16(o, true) : v.getUint16(o, true));
      return { kind: 'number', ...base, type, value: r(0), min: r(2), max: r(4), units: cString(body, at + 8)[0] };
    }
    case ParamType.FOLDER: {
      // The folder repeats its (possibly dynamic) name, then child ids to 0xff.
      const [, next] = cString(body, 2);
      const children: number[] = [];
      for (let i = next; i < body.length && u8(body, i) !== 0xff; i++) children.push(u8(body, i));
      return { kind: 'folder', ...base, children };
    }
    case ParamType.INFO:
    case ParamType.STRING:
      return { kind: type === ParamType.INFO ? 'info' : 'string', ...base, value: cString(body, at)[0] };
    case ParamType.COMMAND:
      return { kind: 'command', ...base, step: u8(body, at), timeoutMs: u8(body, at + 1) * 10, info: cString(body, at + 2)[0] };
    default:
      return { kind: 'unsupported', ...base, type };
  }
}

export function encodeDevicePing(origin: number = Address.ELRS_LUA): Uint8Array {
  return encodeExtended(FrameType.DEVICE_PING, Address.BROADCAST, origin);
}

export function encodeParameterRead(destination: number, field: number, chunk: number,
  origin: number = Address.ELRS_LUA): Uint8Array {
  return encodeExtended(FrameType.PARAMETER_READ, destination, origin, Uint8Array.of(field, chunk));
}

/** The value bytes ExpressLRS's PARAMETER_WRITE handler reads (big-endian for wider types). */
export function parameterValueBytes(type: number, value: number): Uint8Array {
  switch (type) {
    case ParamType.UINT16:
    case ParamType.INT16:
      return Uint8Array.of((value >> 8) & 0xff, value & 0xff);
    case ParamType.UINT32:
    case ParamType.INT32:
      return Uint8Array.of((value >>> 24) & 0xff, (value >>> 16) & 0xff, (value >>> 8) & 0xff, value & 0xff);
    default:
      return Uint8Array.of(value & 0xff);
  }
}

export function encodeParameterWrite(destination: number, field: number, valueBytes: Uint8Array,
  origin: number = Address.ELRS_LUA): Uint8Array {
  const body = new Uint8Array(1 + valueBytes.length);
  body[0] = field;
  body.set(valueBytes, 1);
  return encodeExtended(FrameType.PARAMETER_WRITE, destination, origin, body);
}

export const MSP_ELRS_RXTX_CONFIG = 0x2d;
export const RxtxConfigSubcommand = {
  UID: 0x00,
  BIND_PHRASE: 0x01,
  MODEL_ID: 0x0a,
} as const;

export const CRSF_COMMAND_SUBCMD_RX = 0x10;
export const CRSF_COMMAND_SUBCMD_RX_BIND = 0x01;

/** The direct bind command used by `aerial-kit-link ... radio bind`. */
export function encodeCrsfBindCommand(destination: number, origin: number = Address.ELRS_LUA): Uint8Array {
  return encodeExtended(FrameType.COMMAND, destination, origin, Uint8Array.of(
    CRSF_COMMAND_SUBCMD_RX,
    CRSF_COMMAND_SUBCMD_RX_BIND,
  ));
}

/** ExpressLRS's MSP-v2 request over CRSF (`CRSFRouter::SetMspV2Request`). */
export function encodeMspV2Request(destination: number, payload: Uint8Array,
  functionCode = MSP_ELRS_RXTX_CONFIG, origin: number = Address.ELRS_LUA): Uint8Array {
  const body = Uint8Array.from([
    0x50, 0, functionCode & 0xff, (functionCode >> 8) & 0xff,
    payload.length & 0xff, (payload.length >> 8) & 0xff, ...payload,
  ]);
  const framed = new Uint8Array(body.length + 1);
  framed.set(body);
  framed[body.length] = crc8DvbS2(body.subarray(1));
  return encodeExtended(FrameType.MSP_REQ, destination, origin, framed);
}

/** ExpressLRS's MSP-v1 write envelope over CRSF (`CRSFRouter::AddMspMessage`). */
export function encodeMspWrite(destination: number, payload: Uint8Array,
  functionCode = MSP_ELRS_RXTX_CONFIG, origin: number = Address.ELRS_LUA): Uint8Array {
  const body = Uint8Array.from([
    0x30, payload.length, functionCode & 0xff, ...payload,
  ]);
  const framed = new Uint8Array(body.length + 1);
  framed.set(body);
  framed[body.length] = crc8DvbS2(body.subarray(1));
  return encodeExtended(FrameType.MSP_WRITE, destination, origin, framed);
}

export interface MspResponse {
  readonly functionCode: number;
  readonly payload: Uint8Array;
}

/** Decode the MSP-v1 response envelope ExpressLRS's router emits. */
export function decodeMspResponse(frame: CrsfFrame): MspResponse | null {
  if (frame.type !== FrameType.MSP_RESP || frame.payload.length < 6) return null;
  const msp = frame.payload.subarray(2);
  if (msp[0] !== 0x30) return null;
  const length = u8(msp, 1);
  if (msp.length !== length + 4) return null;
  if (crc8DvbS2(msp.subarray(1, msp.length - 1)) !== u8(msp, msp.length - 1)) return null;
  return { functionCode: u8(msp, 2), payload: msp.slice(3, msp.length - 1) };
}

export function encodeRcChannels(channels: readonly number[], address: number = CRSF_SYNC): Uint8Array {
  return encodeFrame(FrameType.RC_CHANNELS_PACKED, packChannels(channels), address);
}

export function hex(bytes: Uint8Array): string {
  return Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join('');
}

export function fromHex(text: string): Uint8Array {
  const out = new Uint8Array(text.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(text.slice(2 * i, 2 * i + 2), 16);
  return out;
}
