import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import {
  Address,
  CrsfDecoder,
  type CrsfFrame,
  FrameType,
  ParamType,
  decodeDeviceInfo,
  decodeParameter,
  decodeParameterChunk,
  decodeTelemetry,
  encodeDevicePing,
  encodeExtended,
  encodeFrame,
  encodeParameterRead,
  encodeParameterWrite,
  fromHex,
  hex,
  packChannels,
  parameterValueBytes,
  unpackChannels,
} from '../src/protocol/crsf';

/** The vectors the Python library and the firmware are checked against too. */
interface Case {
  name: string;
  address: number;
  frame_type: number;
  payload_hex: string;
  raw_hex: string;
  decoded: Record<string, unknown>;
}
const VECTORS = JSON.parse(
  readFileSync(resolve(__dirname, '../../../protocols/vectors/crsf.json'), 'utf8'),
) as { protocol: string; cases: Case[] };
const byName = (n: string) => VECTORS.cases.find((c) => c.name === n)!;
/** The one frame in some bytes (a test failure if there is not exactly one). */
function one(bytes: Uint8Array): CrsfFrame {
  const frames = new CrsfDecoder().push(bytes);
  expect(frames).toHaveLength(1);
  return frames[0]!;
}

describe('CRSF against the shared vectors', () => {
  it('reads the CRSF vector file', () => {
    expect(VECTORS.protocol).toBe('crsf');
    expect(VECTORS.cases.length).toBeGreaterThanOrEqual(17);
  });

  it.each(VECTORS.cases.map((c) => [c.name, c] as const))('%s: parses and re-encodes byte for byte', (_n, c) => {
    const f = one(fromHex(c.raw_hex));
    expect(f.address).toBe(c.address);
    expect(f.type).toBe(c.frame_type);
    expect(hex(f.payload)).toBe(c.payload_hex);
    expect(hex(encodeFrame(c.frame_type, fromHex(c.payload_hex), c.address))).toBe(c.raw_hex);
  });

  it('RC channels pack to the vector bytes and back', () => {
    for (const n of ['rc_channels_centre', 'rc_channels_extremes']) {
      const c = byName(n);
      const channels = c.decoded.channels as number[];
      expect(hex(packChannels(channels))).toBe(c.payload_hex);
      expect(unpackChannels(fromHex(c.payload_hex))).toEqual(channels);
    }
  });

  it('telemetry decodes to the vector meaning', () => {
    const t = (n: string) => decodeTelemetry(one(fromHex(byName(n).raw_hex)));
    const att = t('attitude');
    expect(att).toMatchObject({ kind: 'attitude' });
    if (att?.kind === 'attitude') {
      expect(att.pitchRad).toBeCloseTo(0.1, 4);
      expect(att.rollRad).toBeCloseTo(-0.5, 4);
      expect(att.yawRad).toBeCloseTo(1.2, 4);
    }
    expect(t('battery')).toEqual({ kind: 'battery', voltageV: 16.8, currentA: 12.3, capacityMah: 1500, remainingPercent: 88 });
    // GPS by the CRSF / ExpressLRS / AK-firmware scales (degrees * 1e7, km/h * 10).
    const gps = t('gps');
    if (gps?.kind !== 'gps') throw new Error('gps');
    expect(gps.latitudeDeg).toBeCloseTo(4.73977, 6);
    expect(gps.groundspeedMps).toBeCloseTo(100 / 36, 6);
    expect([gps.headingDeg, gps.altitudeM, gps.satellites]).toEqual([90, 450, 12]);
    const baro = t('barometer');
    if (baro?.kind !== 'barometer') throw new Error('baro');
    expect(baro.altitudeM).toBeCloseTo(123.4, 3);
    expect(baro.verticalSpeedMps).toBeCloseTo(-2.5, 3);
    expect(t('flight_mode')).toEqual({ kind: 'flightMode', mode: 'ANGLE' });
    expect(t('link_statistics')).toEqual({ kind: 'link', value: {
      uplinkRssi1Dbm: -90, uplinkRssi2Dbm: -92, uplinkLinkQuality: 88, uplinkSnrDb: 7, activeAntenna: 1, rfMode: 2,
      uplinkTxPower: 3, downlinkRssiDbm: -80, downlinkLinkQuality: 70, downlinkSnrDb: 4 } });
  });

  // KNOWN VECTOR ERROR (filed in change-requests.md): the GPS case stores 47 397 700 and 100, which
  // are 4.73977 deg and 2.78 m/s at the CRSF scales every implementation uses, but labels them
  // 47.3977 deg and 10 m/s. Flips to passing when the vector is regenerated; then drop `.fails`.
  it.fails('the GPS vector label matches its bytes', () => {
    const gps = decodeTelemetry(one(fromHex(byName('gps').raw_hex)));
    if (gps?.kind !== 'gps') throw new Error('gps');
    expect(gps.latitudeDeg).toBeCloseTo(byName('gps').decoded.latitude_deg as number, 6);
    expect(gps.groundspeedMps).toBeCloseTo(byName('gps').decoded.groundspeed_mps as number, 1);
  });

  // KNOWN VECTOR ERROR (filed): parameter_read stores [0, 1] after the addresses and labels it
  // field 1, chunk 0; ExpressLRS orders field then chunk, so those bytes are field 0, chunk 1.
  it.fails('the parameter_read vector label matches its bytes', () => {
    const f = one(fromHex(byName('parameter_read').raw_hex));
    expect([f.payload[2], f.payload[3]]).toEqual([byName('parameter_read').decoded.field, byName('parameter_read').decoded.chunk]);
  });

  // KNOWN VECTOR ERROR (filed): parameter_write stores [0, 1, 2] after the addresses (a leading
  // byte ExpressLRS does not read) for "field 1, value 2"; ExpressLRS reads [field][value].
  it.fails('the parameter_write vector is laid out as ExpressLRS reads it', () => {
    const f = one(fromHex(byName('parameter_write').raw_hex));
    expect([f.payload[2], f.payload[3]]).toEqual([1, 2]);
  });

  it('device ping, info, parameter read/write and command match', () => {
    // The vector pads the ping with two zero bytes ExpressLRS's own Lua does not send
    // ({0x00, 0xEA}); compare what it means, and encode what ExpressLRS sends.
    const ping = one(fromHex(byName('device_ping').raw_hex));
    expect([ping.type, ping.payload[0], ping.payload[1]])
      .toEqual([FrameType.DEVICE_PING, Address.RADIO_TRANSMITTER, Address.FLIGHT_CONTROLLER]);
    const ours = one(encodeDevicePing(Address.RADIO_TRANSMITTER));
    expect([ours.type, ...ours.payload]).toEqual([FrameType.DEVICE_PING, Address.BROADCAST, Address.RADIO_TRANSMITTER]);
    const info = decodeDeviceInfo(one(fromHex(byName('device_info').raw_hex)));
    expect(info).toMatchObject({ name: 'aerialkit-f405', parameterCount: 0, parameterVersion: 1,
      destination: Address.RADIO_TRANSMITTER, origin: Address.FLIGHT_CONTROLLER });
    // ExpressLRS's Lua sends {device, handset, fieldId, fieldChunk}: field first.
    const read = one(encodeParameterRead(Address.RADIO_TRANSMITTER, 1, 0,
      Address.FLIGHT_CONTROLLER));
    expect([...read.payload]).toEqual([Address.RADIO_TRANSMITTER, Address.FLIGHT_CONTROLLER, 1, 0]);
    // ExpressLRS reads PARAMETER_WRITE as [field][value...] after the addresses.
    const write = one(encodeParameterWrite(Address.RADIO_TRANSMITTER, 1,
      parameterValueBytes(ParamType.UINT8, 2), Address.FLIGHT_CONTROLLER));
    expect([...write.payload]).toEqual([Address.RADIO_TRANSMITTER, Address.FLIGHT_CONTROLLER, 1, 2]);
    expect(hex(encodeExtended(FrameType.COMMAND, Address.RADIO_TRANSMITTER, Address.FLIGHT_CONTROLLER,
      Uint8Array.of(0x10, 0x01)))).toBe(byName('command').raw_hex);
  });
});

/** An entry body exactly as CRSFEndpoint::sendParameter lays it out (ExpressLRS 4.1.0). */
function entry(parent: number, type: number, name: string, tail: number[]): Uint8Array {
  return Uint8Array.from([parent, type, ...new TextEncoder().encode(name), 0, ...tail]);
}
const z = (s: string) => [...new TextEncoder().encode(s), 0];

describe('ExpressLRS parameter menu entries', () => {
  it('text selection: options, value, min, max, default, units', () => {
    const p = decodeParameter(1, entry(0, ParamType.TEXT_SELECTION, 'Packet Rate',
      [...z('50Hz;150Hz;250Hz;500Hz'), 2, 0, 3, 0, ...z('')]));
    expect(p).toEqual({ kind: 'selection', id: 1, parent: 0, name: 'Packet Rate', hidden: false,
      options: ['50Hz', '150Hz', '250Hz', '500Hz'], value: 2, min: 0, max: 3, units: '' });
  });

  it('hidden flag, command, folder, info and uint8', () => {
    expect(decodeParameter(9, entry(0, ParamType.COMMAND | 0x80, 'Bind', [0, 200, ...z('')])))
      .toEqual({ kind: 'command', id: 9, parent: 0, name: 'Bind', hidden: true, step: 0, timeoutMs: 2000, info: '' });
    // A folder's (possibly dynamic) name is written over the name slot, then its children to 0xff.
    expect(decodeParameter(4, entry(0, ParamType.FOLDER, 'TX Power (25mW)', [5, 6, 0xff])))
      .toMatchObject({ kind: 'folder', name: 'TX Power (25mW)', children: [5, 6] });
    expect(decodeParameter(12, entry(0, ParamType.INFO, 'Version', z('4.1.0 AK'))))
      .toMatchObject({ kind: 'info', value: '4.1.0 AK' });
    expect(decodeParameter(7, entry(4, ParamType.UINT8, 'Fan Thresh', [3, 0, 10, 0, ...z('s')])))
      .toMatchObject({ kind: 'number', parent: 4, value: 3, min: 0, max: 10, units: 's' });
  });

  it('a chunked entry reassembles from PARAMETER_SETTINGS_ENTRY frames', () => {
    const body = entry(0, ParamType.TEXT_SELECTION, 'Telem Ratio', [...z('Std;Off;1:128;1:64;1:32;1:16;1:8;1:4;1:2;Race'), 0, 0, 9, 0, ...z('')]);
    const max = 20;
    const parts: Uint8Array[] = [];
    for (let i = 0; i < body.length; i += max) parts.push(body.slice(i, i + max));
    const decoder = new CrsfDecoder();
    const data: number[] = [];
    parts.forEach((part, k) => {
      const frame = encodeExtended(FrameType.PARAMETER_SETTINGS_ENTRY, Address.ELRS_LUA, Address.RADIO_TRANSMITTER,
        Uint8Array.from([3, parts.length - 1 - k, ...part]), Address.RADIO_TRANSMITTER);
      const chunk = decodeParameterChunk(decoder.push(frame)[0]!)!;
      expect(chunk.chunksRemaining).toBe(parts.length - 1 - k);
      data.push(...chunk.data);
    });
    expect(decodeParameter(3, Uint8Array.from(data))).toMatchObject({ kind: 'selection', name: 'Telem Ratio' });
  });
});

describe('the decoder', () => {
  it('survives garbage, split frames and bad checksums', () => {
    const good = encodeDevicePing();
    const bad = good.slice();
    bad[bad.length - 1] = (bad[bad.length - 1] ?? 0) ^ 0xff;
    const d = new CrsfDecoder();
    expect(d.push(Uint8Array.of(0x00, 0x55, 0x99))).toEqual([]);
    expect(d.push(bad)).toEqual([]);
    expect(d.crcErrors).toBe(1);
    expect(d.push(good.subarray(0, 3))).toEqual([]);
    expect(d.push(good.subarray(3))).toHaveLength(1);
  });
});
