import { describe, expect, it } from 'vitest';
import { Command, MotorStatus, PerfStatus } from '../src/protocol/constants';
import { Feature, featureNames, has } from '../src/protocol/features';
import { parseMotorTelemetry, parsePerf, parseStatus, parseTelemetry, ProtocolError } from '../src/protocol/messages';
import { AerialKitClient } from '../src/protocol/client';
import { buildFrame } from '../src/protocol/frame';
import { ManualLink } from './helpers';

function perfReply(status = 0): Uint8Array {
  // ak_proto.h wire order, with distinct values to catch shifted fields.
  const bytes = new Uint8Array(59);
  const view = new DataView(bytes.buffer);
  bytes[0] = status;
  view.setUint32(1, 0xf1234567, true);
  view.setUint32(5, 23, true);
  view.setUint16(9, 1000, true);
  [1001, 999, 1020, 7].forEach((v, i) => view.setUint32(11 + 4 * i, v, true));
  [2, 18, 20].forEach((v, i) => view.setUint16(27 + 2 * i, v, true));
  view.setUint32(33, 3, true);
  [11, 22, 33, 44, 55].forEach((v, i) => view.setUint16(37 + 2 * i, v, true));
  [6, 7, 8, 9, 10].forEach((v, i) => view.setUint16(47 + 2 * i, v, true));
  view.setUint16(57, 123, true);
  return bytes;
}

describe('current runtime protocol compatibility', () => {
  it('names new capabilities without enabling an absent or clear bit', () => {
    expect(featureNames((1 << 12) | (1 << 13))).toEqual(['PERF', 'MOTOR_TELEMETRY']);
    expect(has(null, Feature.PERF)).toBe(false);
    expect(has(0, Feature.MOTOR_TELEMETRY)).toBe(false);
    expect(has(1 << 12, Feature.PERF)).toBe(true);
  });

  it('decodes the complete PERF window in firmware units', () => {
    expect(parsePerf(perfReply())).toEqual({
      status: PerfStatus.OK, loops: 0xf1234567, samples: 23, nominalUs: 1000,
      periodLastUs: 1001, periodMinUs: 999, periodMaxUs: 1020, late: 7,
      jitterP50Us: 2, jitterP99Us: 18, jitterMaxUs: 20, jitterOver: 3,
      sectionAvgX10: [11, 22, 33, 44, 55], sectionMaxUs: [6, 7, 8, 9, 10],
      loadPermille: 123,
    });
    expect(parsePerf(perfReply(1)).status).toBe(PerfStatus.NONE);
  });

  it('refuses missing, truncated, oversized and unknown PERF replies', () => {
    for (const bytes of [new Uint8Array(), new Uint8Array([0x7f]),
      perfReply().slice(0, 58), new Uint8Array(60), perfReply(2)]) {
      expect(() => parsePerf(bytes)).toThrow(ProtocolError);
    }
  });

  it('requests PERF through the correlated read-only client', async () => {
    const link = new ManualLink();
    const client = new AerialKitClient(link);
    try {
      const result = client.perf();
      expect(link.written[0]![3]).toBe(Command.PERF);
      link.deliver(buildFrame(Command.PERF | 0x80, perfReply()));
      expect((await result).samples).toBe(23);
    } finally { client.close(); }
  });

  it('preserves unsigned telemetry uptime and signed status coordinates', () => {
    const bytes = new Uint8Array(26);
    const view = new DataView(bytes.buffer);
    view.setUint32(0, 0xf1234567, true);
    view.setInt16(8, -123, true);
    view.setInt32(14, -345678901, true);
    expect(parseTelemetry(bytes).uptimeMs).toBe(0xf1234567);
    expect(parseTelemetry(bytes).rollDeg).toBe(-12.3);
    expect(parseStatus(bytes.slice(4)).lat).toBe(-34.5678901);
    expect(() => parseStatus(new Uint8Array(21))).toThrow(ProtocolError);
  });

  it('distinguishes no ESC path, unheard motors, and measured zero RPM', () => {
    expect(parseMotorTelemetry(new Uint8Array([1]))).toEqual({
      status: MotorStatus.NONE, count: 0, poles: 0, motors: [],
    });
    const bytes = new Uint8Array(22);
    bytes[1] = 1;
    let motor = parseMotorTelemetry(bytes).motors[0]!;
    expect(motor.measured).toBe(false);
    expect(motor.rpm).toBeNull();
    bytes[2] = 14;
    bytes[3] = 3;
    motor = parseMotorTelemetry(bytes).motors[0]!;
    expect(motor.measured).toBe(true);
    expect(motor.rpm).toBe(0);
    expect(motor.temperature).toBeNull();
  });

  it('reads eRPM and electrical values only with their validity flags', () => {
    const bytes = new Uint8Array(22);
    bytes.set([0, 1, 14, 31]);
    const view = new DataView(bytes.buffer);
    view.setUint32(4, 14000, true);
    view.setUint32(8, 2000, true);
    bytes[12] = 35; bytes[13] = 40;
    [12000, 2500, 100, 2].forEach((v, i) => view.setUint16(14 + 2 * i, v, true));
    expect(parseMotorTelemetry(bytes).motors[0]).toMatchObject({
      measured: true, erpm: 14000, rpm: 2000, temperature: 35,
      maxTemperature: 40, millivolts: 12000, milliamps: 2500, packets: 100, invalid: 2,
    });
    bytes[3] = 1;
    expect(parseMotorTelemetry(bytes).motors[0]).toMatchObject({
      erpm: 14000, rpm: null, temperature: null, millivolts: null, milliamps: null,
    });
  });

  it('rejects malformed motor telemetry rather than fabricating readings', () => {
    for (const bytes of [new Uint8Array(), new Uint8Array([1, 0]),
      new Uint8Array([0, 5, 14]), new Uint8Array([0x7f]),
      new Uint8Array([0, 1, 14]), new Uint8Array(23)]) {
      expect(() => parseMotorTelemetry(bytes)).toThrow(ProtocolError);
    }
  });
});
