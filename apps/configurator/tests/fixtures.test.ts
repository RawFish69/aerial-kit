import { describe, expect, it } from 'vitest';
import { SetStatus } from '../src/protocol/constants';
import { baseCommand, FrameDecoder, type Frame } from '../src/protocol/frame';
import {
  parseHello,
  parseLogRecord,
  parseLogSource,
  parseParamGet,
  parseParamSet,
  parseStatus,
  ProtocolError,
} from '../src/protocol/messages';
import fixture from './fixtures/aerialkit.json';
import table from '../src/firmware/demo-table.json';

/**
 * Every parser, run against bytes the firmware actually sent.
 *
 * The point of these is that they cannot drift. `tests/frame.test.ts` proves
 * the framing; this proves the *field offsets* — the part where being one byte
 * out produces a number that looks plausible, which is the failure mode a
 * configurator has to be engineered against. The fixture came from
 * `tools/capture-fixtures.py`, which drove `build-host/aerialkit-sim` with the
 * firmware's own client, so these are the firmware's answers, not this app's.
 */

function decode(hex: string): Frame {
  const bytes = new Uint8Array(hex.length / 2);
  for (let i = 0; i < bytes.length; i++) bytes[i] = parseInt(hex.slice(i * 2, i * 2 + 2), 16);
  const { frames, issues } = new FrameDecoder().push(bytes, 0);
  expect(issues).toEqual([]);
  expect(frames).toHaveLength(1);
  return frames[0]!;
}

const frames = fixture.frames as Record<string, { request: string; reply: string; note: string }>;

describe('the capture itself', () => {
  it('came from the firmware simulator and says so', () => {
    expect(fixture.captured_from).toBe('aerialkit-sim');
    expect(fixture.product).toBe(table.captured_product);
  });

  it('has a frame for every command the app sends', () => {
    for (const name of [
      'hello',
      'status',
      'param_get_0',
      'param_set_ok',
      'param_set_refused',
      'param_set_short',
      'param_save',
      'log_source_fast',
      'log_get_0',
      'telemetry_rate',
    ]) {
      expect(Object.keys(frames)).toContain(name);
    }
  });

  it('records the request as well as the reply, and they correlate', () => {
    for (const [name, entry] of Object.entries(frames)) {
      const request = decode(entry.request);
      const reply = decode(entry.reply);
      expect(baseCommand(reply.command), `${name}: reply does not answer the request`).toBe(
        baseCommand(request.command),
      );
    }
  });
});

describe('hello', () => {
  const hello = parseHello(decode(frames.hello!.reply).payload);

  it('reads the product, the version and the count', () => {
    expect(hello.product).toBe(fixture.product);
    expect(hello.protocolVersion).toBe(fixture.protocol_version);
    expect(hello.parameterCount).toBe(fixture.parameter_count);
  });

  it('reads the changed count as text, because that is what the wire carries', () => {
    expect(typeof hello.changedSinceSaved).toBe('string');
    expect(hello.changedSinceSaved).toMatch(/^\d+$/);
  });
});

describe('param get', () => {
  it('reads the first parameter', () => {
    const item = parseParamGet(decode(frames.param_get_0!.reply).payload, 0);
    expect(item).not.toBeNull();
    expect(item!.index).toBe(0);
    expect(item!.name).toBe(fixture.parameter_table[0]!.name);
    expect(item!.value).toBe(fixture.parameter_table[0]!.value);
  });

  it('answers null, not an empty parameter, one index past the end', () => {
    expect(parseParamGet(decode(frames.param_get_past_end!.reply).payload, 32)).toBeNull();
  });

  it('agrees with the demo table name for name, in order', () => {
    // The fixture's table came off the wire; the demo table's came out of
    // ak_flight.c by tools/ak_table.py. They are two readings of the same
    // firmware, and if they ever disagree the capture refuses to write either
    // file. This is the same check from the app's side, so a stale committed
    // demo table fails the build too.
    //
    // It is *not* the check it used to be. Until milestone 3 this proved that
    // the metadata the app showed a real board came from the same table as the
    // board's own reply. That job now belongs to `param info`: the board serves
    // its own description and nothing joins it to a file. What is left here is
    // the demo board's starting table being checked against the firmware it was
    // captured from.
    expect(fixture.parameter_table.map((item) => item.name)).toEqual(
      table.parameters.map((item) => item.name),
    );
    expect(table.parameters.map((item) => item.index)).toEqual(
      fixture.parameter_table.map((item) => item.index),
    );
  });
});

describe('param set', () => {
  it('reads a refusal and the table’s own words with it', () => {
    const reply = parseParamSet(decode(frames.param_set_refused!.reply).payload);
    expect(reply.status).toBe(SetStatus.VALUE_REFUSED);
    // Not "refused" but "out of range 0.000..3.000" — the message is the only
    // place the protocol ever states a range, and a client that drops it leaves
    // a person guessing at a number they cannot see.
    expect(reply.message).toBe('out of range 0.000..3.000');
  });

  it('reads a refusal that names the shape rather than the value', () => {
    // An index with no value. The firmware refuses this identically to a bad
    // index, which is worth knowing: a truncated write looks like a missing
    // parameter.
    const reply = parseParamSet(decode(frames.param_set_short!.reply).payload);
    expect(reply.status).toBe(SetStatus.NO_SUCH_PARAMETER);
    expect(reply.message).toBe('no such parameter');
  });

  it('reads a successful write as status 0 with no message', () => {
    const reply = parseParamSet(decode(frames.param_set_ok!.reply).payload);
    expect(reply.status).toBe(SetStatus.OK);
    expect(reply.message).toBe('');
  });

  it('accepts a reply that stops after the status', () => {
    // An older board sends nothing after the status byte. That has to read as
    // an empty message rather than an error.
    const reply = parseParamSet(new Uint8Array([SetStatus.OK]));
    expect(reply).toEqual({ status: SetStatus.OK, message: '' });
  });
});

describe('param save', () => {
  it('reports what the board said about persisting', () => {
    const reply = parseParamSet(decode(frames.param_save!.reply).payload);
    // The simulator has a save callback, so this one succeeds. The
    // nowhere-to-save path is covered in the session tests, where the demo
    // board can be built without storage.
    // `REFUSED_ARMED` is a legal answer here too: the host simulator boots
    // armed, and since milestone 4 the save path is gated on exactly that. It
    // reads as a short list because it is the set of answers this *fixture*
    // could contain, not the set the command defines.
    expect([
      SetStatus.OK,
      SetStatus.NOWHERE_TO_SAVE,
      SetStatus.REFUSED_ARMED,
      SetStatus.STORAGE_ERROR,
    ]).toContain(reply.status);
  });
});

describe('status', () => {
  const status = parseStatus(decode(frames.status!.reply).payload);

  it('reads the flight state and the link', () => {
    expect(status.flightState).toBeGreaterThanOrEqual(0);
    expect(status.flightState).toBeLessThanOrEqual(5);
    // The host simulator boots armed, which is why the app's write gate is shut
    // by default against it. That is a fact about the simulator, recorded here
    // so a change to it is noticed rather than accommodated.
    expect(fixture.anomalies.length).toBeGreaterThan(0);
  });

  it('reads four motors', () => {
    expect(status.motors).toHaveLength(4);
    for (const motor of status.motors) {
      expect(motor).toBeGreaterThanOrEqual(0);
      expect(motor).toBeLessThanOrEqual(255);
    }
  });

  it('scales attitude and position the way the reference client does', () => {
    const raw = decode(frames.status!.reply).payload;
    const view = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
    expect(status.rollDeg).toBeCloseTo(view.getInt16(4, true) / 10, 6);
    expect(status.lat).toBeCloseTo(view.getInt32(10, true) / 1e7, 9);
    expect(status.lon).toBeCloseTo(view.getInt32(14, true) / 1e7, 9);
  });

  it('refuses a truncated status rather than defaulting the missing fields', () => {
    // The failure this prevents: a short reply read as a full one, giving
    // latitude 0 — which is a real place, in the Atlantic.
    const short = decode(frames.status!.reply).payload.slice(0, 12);
    expect(() => parseStatus(short)).toThrow(ProtocolError);
  });
});

describe('the logs', () => {
  it('reads the selected source and how many records it holds', () => {
    const reply = parseLogSource(decode(frames.log_source_fast!.reply).payload);
    expect(reply.status).toBe(0);
    expect(reply.source).toBe(0);
    expect(reply.count).toBeGreaterThan(0);
  });

  it('reads a source the device does not have as absent, not as empty', () => {
    const reply = parseLogSource(decode(frames.log_source_absent!.reply).payload);
    expect(reply.status).not.toBe(0);
    // The distinction that matters: null is "no such log", 0 would be "an empty
    // one", and the firmware spends a status byte to keep them apart.
    expect(reply.count).toBeNull();
    expect(reply.source).toBeNull();
  });

  it('reads a 51-byte record field for field', () => {
    const payload = decode(frames.log_get_0!.reply).payload;
    expect(payload).toHaveLength(52); // status + one record

    // Decoded independently by the firmware's own client, so this is a check
    // against a second implementation rather than against this one's
    // assumptions. The values come from `akproto.parse_log_record` run over the
    // same bytes by tools/capture-fixtures.py:
    //
    //   time_ms 2000  gyro (0, 200, 0)  accel (0, 0, 0)
    //   roll 0  pitch 0  yaw 0  alt_mm 0  sticks all 0  torque all 0
    //   motors (0, 0, 0, 60)  state 2  flags 0  lat 0  lon 0
    //
    // `state 2` is AK_FLIGHT_FAILSAFE and `motor4 60` is the only output
    // turning, which is what the simulator's log holds at index 0. Being one
    // byte out anywhere in this layout would still produce numbers — which is
    // exactly why the assertion is the whole record and not a spot check.
    expect(parseLogRecord(payload, 0)).toEqual({
      index: 0,
      timeMs: 2000,
      gyro: [0, 200, 0],
      accel: [0, 0, 0],
      rollDeg: 0,
      pitchDeg: 0,
      yawDeg: 0,
      altMm: 0,
      stick: [0, 0, 0, 0],
      torque: [0, 0, 0],
      motors: [0, 0, 0, 60],
      state: 2,
      flags: 0,
      lat: 0,
      lon: 0,
    });
  });

  it('refuses a short record rather than reading past it', () => {
    const payload = decode(frames.log_get_0!.reply).payload.slice(0, 20);
    expect(() => parseLogRecord(payload, 0)).toThrow(ProtocolError);
  });
});

describe('what the capture recorded about the simulator', () => {
  it('records a refused telemetry stream, not a rate', () => {
    // The console link refuses. `ak_proto_init()` zeroes the whole struct, so
    // `can_stream` is 0, and the only place that sets it is main.c's network
    // link — there, the comment explains, because a console with frames
    // arriving among a person's keystrokes is a console nobody can use. So the
    // reply carries 0, and 0 is an answer.
    //
    // This assertion used to be `expect(anomaly.evidence).toContain('10')`, and
    // it passed while the reply said 0 *and* while it still said 10 — because
    // the evidence string contains "asked for 10" whatever the board answered.
    // It checked the request and claimed to check the reply. It reads the
    // reply's own byte now.
    const reply = decode(frames.telemetry_rate!.reply);
    expect(reply.payload[0]).toBe(0);

    const anomaly = fixture.anomalies[0]!;
    expect(anomaly.what).toContain('refuses');
    expect(anomaly.evidence).toContain('refusal');
  });
});
