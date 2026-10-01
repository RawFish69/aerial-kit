import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import {
  BATTERY_STATES,
  SENSOR_BODY_LENGTH,
  SENSOR_NAME_LENGTH,
  SENSOR_TOPICS,
  Sensor,
  SensorStatus,
} from '../src/protocol/constants';
import { parseSensorInfo, ProtocolError } from '../src/protocol/messages';
import { AerialKitSession } from '../src/session/aerialkit';
import { BoardLink, FakeClock, LinkTransport, makeBoard, settle } from './helpers';
import type { DemoBoard } from '../src/transport/demo-board';

/**
 * The sensors, from the wire up.
 *
 * **One opcode, three answers, and telling them apart is what this file is
 * for:**
 *
 *  - `AK_PROTO_SENSOR_NO_SUCH` — the build does not answer for that topic at
 *    all. A fact about the firmware, and the fix is an update.
 *  - `OK` with `present` clear — the build knows the question and this board
 *    has nothing fitted. A fact about the aircraft, and the fix is a socket.
 *  - `OK` with `present` set — a reading, and a zero in it is a zero.
 *
 * The firmware keeps the last two apart by leaving the body *off* rather than
 * sending a body of zeros, because a body of zeros is a sensor reading zero,
 * which is a sensor that has failed. A client that read on would collapse them;
 * a client that invented a body for the absent case would too.
 *
 * And a fourth that is not a sensor fact at all: `0x7F`, the byte this firmware
 * answers a command it does not implement with. A build that predates
 * `sensor info` must not be reported to a person as a board whose parts are
 * missing.
 */

const STALE_AFTER = 2000;
const SENSOR_POLL_MS = 1000;

interface Built {
  session: AerialKitSession;
  board: DemoBoard;
  link: BoardLink;
  clock: FakeClock;
}

async function connect(
  options: {
    features?: number | null;
    sensors?: Partial<Record<Sensor, boolean>>;
  } = {},
): Promise<Built> {
  const clock = new FakeClock();
  const board = makeBoard({
    ...(options.features === undefined ? {} : { features: options.features }),
    ...(options.sensors === undefined ? {} : { sensors: options.sensors }),
  });
  const link = new BoardLink(board);
  const session = new AerialKitSession(new LinkTransport(link), {
    now: clock.now,
    staleAfterMs: STALE_AFTER,
    // The status poll never fires on its own here. The sensor round does, and a
    // test moves it with `advanceTimersByTime` — that interval is the *page's*
    // timer rather than this board's clock, so the two move by different calls.
    pollMs: 1_000_000,
    sensorPollMs: SENSOR_POLL_MS,
  });
  await session.open();
  await session.refresh();
  return { session, board, link, clock };
}

/** The demo board counts samples against the wall clock, which under
 *  `vi.useFakeTimers()` is the fake one. Moving it is how a test gives this
 *  board sensors that have actually been running. */
function runBoardFor(ms: number): void {
  vi.setSystemTime(Date.now() + ms);
}

beforeEach(() => {
  vi.useFakeTimers();
});

afterEach(() => {
  vi.useRealTimers();
});

// ---- hand-built payloads ---------------------------------------------------
//
// Hand-built is the point: the parser's job is field offsets, and a payload
// produced by the same code under test would agree with it about a wrong
// offset. Each builder below mirrors the `append_*` call sequence in
// `ak_proto.c`'s `append_sensor_body` and nothing else.

class Wire {
  private readonly bytes: number[] = [];
  u8(value: number): this {
    this.bytes.push(value & 0xff);
    return this;
  }
  u16(value: number): this {
    return this.u8(value).u8(value >> 8);
  }
  i16(value: number): this {
    return this.u16(value < 0 ? value + 0x10000 : value);
  }
  u32(value: number): this {
    return this.u8(value).u8(value >> 8).u8(value >> 16).u8(value >> 24);
  }
  i32(value: number): this {
    return this.u32(value < 0 ? value + 0x100000000 : value);
  }
  /** `append_name`: fixed width, NUL-padded, cut rather than refused. */
  name(text: string): this {
    for (let i = 0; i < SENSOR_NAME_LENGTH; i++) this.u8(i < text.length ? text.charCodeAt(i) : 0);
    return this;
  }
  done(): Uint8Array {
    return new Uint8Array(this.bytes);
  }
}

function imuPayload(name = 'bmi270'): Uint8Array {
  const at = new Wire().name(name).u8(0).u8(0x24);
  for (let i = 0; i < 12; i++) at.i16(i * 10);
  return at.u32(1234).u32(7).done();
}

function baroPayload(): Uint8Array {
  return new Wire()
    .name('bmp388')
    .i32(101325)
    .i16(2150)
    .u8(1)
    .i32(101000)
    .i32(2750)
    .u8(0)
    .i32(2700)
    .u32(500)
    .u32(1)
    .u32(2)
    .u32(500)
    .u32(100)
    .done();
}

function rangePayload(): Uint8Array {
  return new Wire()
    .name('tof10120')
    .u8(0x52)
    .u16(4000)
    .i32(-1)
    .u32(12)
    .u32(900)
    .u32(180)
    .u32(3)
    .u32(1)
    .u32(0)
    .u32(1800)
    .u16(2)
    .done();
}

function batteryPayload(): Uint8Array {
  return new Wire()
    .u8(1)
    .u8(1)
    .u8(2)
    .u8(4)
    .u16(1512)
    .u16(378)
    .i16(1090)
    .u16(11000)
    .u8(0)
    .u16(3500)
    .u16(3300)
    .u32(600)
    .u32(4)
    .u32(9)
    .done();
}

function gpsPayload(): Uint8Array {
  return new Wire()
    .u8(1)
    .u8(3)
    .u8(1)
    .u8(11)
    .u8(1)
    .i32(515000000)
    .i32(-1200000)
    .i32(35000)
    .i32(1234)
    .i32(18000000)
    .u8(1)
    .i32(515000000)
    .i32(-1200000)
    .i32(120)
    .i32(4500)
    .u8(0)
    .u8(1)
    .u32(90)
    .u32(2)
    .u32(18)
    .done();
}

const BUILDERS: readonly { topic: Sensor; name: string; build: () => Uint8Array }[] = [
  { topic: Sensor.IMU, name: 'imu', build: imuPayload },
  { topic: Sensor.BARO, name: 'baro', build: baroPayload },
  { topic: Sensor.RANGE, name: 'range', build: rangePayload },
  { topic: Sensor.BATTERY, name: 'battery', build: batteryPayload },
  { topic: Sensor.GPS, name: 'gps', build: gpsPayload },
];

/** A reply, framed the way the firmware frames one: status, topic, present. */
function reply(topic: number, body: Uint8Array | null, status = SensorStatus.OK): Uint8Array {
  const out = new Uint8Array(3 + (body?.length ?? 0));
  out[0] = status;
  out[1] = topic;
  out[2] = body === null ? 0 : 1;
  if (body !== null) out.set(body, 3);
  return out;
}

describe('the body, field by field', () => {
  for (const { topic, name, build } of BUILDERS) {
    it(`reads the ${name} body at exactly the length the header says`, () => {
      const body = build();
      // The table in `constants.ts` is the app's copy of the firmware's
      // lengths, and this is the assertion that keeps the copy honest: a body
      // builder here that drifted from it would fail before the parser saw it.
      expect(body.length).toBe(SENSOR_BODY_LENGTH[topic]);
      expect(parseSensorInfo(reply(topic, body)).body).not.toBeNull();
    });
  }

  it('reads the imu body at the offset the wire puts it at', () => {
    const answer = parseSensorInfo(reply(Sensor.IMU, imuPayload()));
    expect(answer.status).toBe(SensorStatus.OK);
    expect(answer.topic).toBe(Sensor.IMU);
    expect(answer.topicName).toBe('imu');
    expect(answer.present).toBe(true);
    const body = answer.body;
    if (body === null || body.topic !== Sensor.IMU) throw new Error('not an imu body');
    // Twelve i16s after the name, reason and whoami: accel, gyro, align, bias,
    // each written as i*10 by the builder above and in that order.
    expect(body.driver).toBe('bmi270');
    expect(body.absent).toBe(0);
    expect(body.absentReason).toBeNull();
    expect(body.whoami).toBe(0x24);
    expect(body.accel).toEqual([0, 10, 20]);
    expect(body.gyro).toEqual([30, 40, 50]);
    expect(body.align).toEqual([60, 70, 80]);
    expect(body.gyroBias).toEqual([90, 100, 110]);
    expect(body.samples).toBe(1234);
    expect(body.errors).toBe(7);
  });

  it('reads the baro body, and keeps the reference beside the pressure', () => {
    const answer = parseSensorInfo(reply(Sensor.BARO, baroPayload()));
    const body = answer.body;
    if (body === null || body.topic !== Sensor.BARO) throw new Error('not a baro body');
    expect(body.driver).toBe('bmp388');
    expect(body.pressurePa).toBe(101325);
    // Hundredths of a degree on the wire, degrees here: the app's convention,
    // and the same one `parseStatus` uses for its own tenths.
    expect(body.temperatureC).toBeCloseTo(21.5, 6);
    expect(body.haveReference).toBe(true);
    expect(body.referencePa).toBe(101000);
    expect(body.heightCm).toBe(2750);
    expect(body.haveGpsReference).toBe(false);
    expect(body.fusedCm).toBe(2700);
    expect(body.samples).toBe(500);
    expect(body.errors).toBe(1);
    expect(body.fails).toBe(2);
    expect(body.baroSamples).toBe(500);
    expect(body.gpsSamples).toBe(100);
  });

  it('keeps a negative rangefinder distance signed, because negative is a reading', () => {
    const answer = parseSensorInfo(reply(Sensor.RANGE, rangePayload()));
    const body = answer.body;
    if (body === null || body.topic !== Sensor.RANGE) throw new Error('not a range body');
    expect(body.driver).toBe('tof10120');
    expect(body.address).toBe(0x52);
    expect(body.maxMm).toBe(4000);
    // The whole reason this field is i32 rather than u32: "nothing in range"
    // and "a wall against the lens" are 0 and a number, and only a negative
    // can say the first without lying about the second.
    expect(body.distanceMm).toBe(-1);
    expect(body.ageMs).toBe(12);
    expect(body.samples).toBe(900);
    expect(body.outOfRange).toBe(180);
    expect(body.landMm).toBe(1800);
    expect(body.agreeCm).toBe(2);
  });

  it('converts the battery to the units the rest of the app uses', () => {
    const answer = parseSensorInfo(reply(Sensor.BATTERY, batteryPayload()));
    const body = answer.body;
    if (body === null || body.topic !== Sensor.BATTERY) throw new Error('not a battery body');
    expect(body.ready).toBe(true);
    expect(body.haveReading).toBe(true);
    expect(body.state).toBe(2);
    expect(body.stateName).toBe(BATTERY_STATES[2]);
    expect(body.cells).toBe(4);
    // Centivolts on the wire, volts on screen; millivolts on the wire for the
    // thresholds, volts here. Both are the app's convention and both are the
    // number a person reads off a pack.
    expect(body.volts).toBeCloseTo(15.12, 6);
    expect(body.voltsPerCell).toBeCloseTo(3.78, 6);
    expect(body.pinMv).toBe(1090);
    expect(body.ratio).toBeCloseTo(11, 6);
    expect(body.warnCellV).toBeCloseTo(3.5, 6);
    expect(body.criticalCellV).toBeCloseTo(3.3, 6);
    expect(body.returns).toBe(9);
  });

  it('reads haveFix before the position, and converts the position like Status', () => {
    const answer = parseSensorInfo(reply(Sensor.GPS, gpsPayload()));
    const body = answer.body;
    if (body === null || body.topic !== Sensor.GPS) throw new Error('not a gps body');
    expect(body.haveFix).toBe(true);
    expect(body.fixType).toBe(3);
    expect(body.fixOk).toBe(true);
    expect(body.satellites).toBe(11);
    // Ten-millionths of a degree on the wire, degrees here — the same division
    // `parseStatus` does, so the two panels cannot disagree about a position.
    expect(body.lat).toBeCloseTo(51.5, 6);
    expect(body.lon).toBeCloseTo(-0.12, 6);
    expect(body.altMslMm).toBe(35000);
    expect(body.courseDeg).toBeCloseTo(180, 6);
    expect(body.haveHome).toBe(true);
    expect(body.homeDistanceM).toBe(120);
    expect(body.homeBearingDeg).toBeCloseTo(45, 6);
    expect(body.rthEnabled).toBe(true);
    expect(body.fixes).toBe(90);
    expect(body.dropped).toBe(2);
    expect(body.configSends).toBe(18);
  });

  it('reads a driver name that exactly fills the field, which has no terminator', () => {
    // Twelve characters is a legal name and leaves no room for a NUL. A reader
    // that required a terminator — `Reader.cstring` does — would refuse the one
    // name that happens to be exactly the field's width.
    const answer = parseSensorInfo(reply(Sensor.IMU, imuPayload('twelvechars!')));
    const body = answer.body;
    if (body === null || body.topic !== Sensor.IMU) throw new Error('not an imu body');
    expect(body.driver).toBe('twelvechars!');
  });
});

describe('the three answers, kept apart', () => {
  it('refuses a topic as a fact about the build, and carries no body with it', () => {
    const answer = parseSensorInfo(new Uint8Array([SensorStatus.NO_SUCH, Sensor.BARO, 0]));
    expect(answer.status).toBe(SensorStatus.NO_SUCH);
    expect(answer.topic).toBe(Sensor.BARO);
    expect(answer.topicName).toBe('baro');
    expect(answer.present).toBe(false);
    expect(answer.body).toBeNull();
  });

  it('answers present: no with no body, which is the whole point of the shape', () => {
    // The bytes a board with nothing in the socket sends. A body of zeros would
    // be a *different* answer — a barometer reading zero pressure — and this is
    // the assertion that the parser does not turn one into the other.
    const answer = parseSensorInfo(reply(Sensor.BARO, null));
    expect(answer.status).toBe(SensorStatus.OK);
    expect(answer.present).toBe(false);
    expect(answer.body).toBeNull();
  });

  it('refuses an absent sensor that carries a body anyway', () => {
    // Reading on here is how the absent case and the reading case become the
    // same thing downstream, which is exactly the collapse the firmware's
    // shape exists to prevent.
    const out = new Uint8Array(3 + 52);
    out[0] = SensorStatus.OK;
    out[1] = Sensor.BARO;
    out[2] = 0;
    expect(() => parseSensorInfo(out)).toThrow(/reading zero/);
  });

  it('gives 0x7F its own sentence rather than reading it as a missing sensor', () => {
    // The distinction is not pedantry: `NO_SUCH` is "this build knows the
    // question and has no driver", so reading 0x7F as `NO_SUCH` would report a
    // *hardware* fact about a board that may have the part soldered in.
    let thrown: unknown = null;
    try {
      parseSensorInfo(new Uint8Array([0x7f, 0, 0]));
    } catch (error) {
      thrown = error;
    }
    expect(thrown).toBeInstanceOf(ProtocolError);
    expect((thrown as ProtocolError).message).toMatch(/predates `sensor info`/);
  });

  it('refuses a status outside the pair', () => {
    expect(() => parseSensorInfo(new Uint8Array([9, Sensor.IMU, 0]))).toThrow(/status 9/);
  });

  it('refuses a body of the wrong length rather than reading on into the next field', () => {
    const short = new Uint8Array(2 + 55);
    short[0] = SensorStatus.OK;
    short[1] = Sensor.GPS;
    short[2] = 1;
    expect(() => parseSensorInfo(short)).toThrow(/next field's bytes/);

    const long = new Uint8Array(3 + 47);
    long[0] = SensorStatus.OK;
    long[1] = Sensor.IMU;
    long[2] = 1;
    expect(() => parseSensorInfo(long)).toThrow(/next field's bytes/);
  });

  it('names a topic this app has no name for, rather than borrowing a neighbour', () => {
    const answer = parseSensorInfo(new Uint8Array([SensorStatus.NO_SUCH, 61, 0]));
    expect(answer.topic).toBe(61);
    expect(answer.topicName).toBeNull();
  });

  it('refuses a reply too short to carry its own status', () => {
    expect(() => parseSensorInfo(new Uint8Array(0))).toThrow(/status/);
  });
});

describe('the session polls them, and only while someone is looking', () => {
  it('asks for nothing until a view says it is watching', async () => {
    const { session } = await connect();
    expect(session.snapshot.sensors.watching).toBe(false);
    expect(session.snapshot.sensors.rounds).toBe(0);
    await vi.advanceTimersByTimeAsync(SENSOR_POLL_MS * 3);
    expect(session.snapshot.sensors.rounds).toBe(0);
    // And nothing has been asked for, which the counters on the link show.
    expect(session.snapshot.sensors.atMs).toBeNull();
  });

  it('asks immediately on true, then once a round', async () => {
    const { session } = await connect();
    session.watchSensors(true);
    await settle(40);
    // Immediately, not after an interval: a tab that opened onto five sections
    // saying "not read yet" for a second would look like a fault.
    expect(session.snapshot.sensors.rounds).toBe(1);
    expect(session.snapshot.sensors.watching).toBe(true);

    runBoardFor(5_000);
    await vi.advanceTimersByTimeAsync(SENSOR_POLL_MS);
    await settle(40);
    expect(session.snapshot.sensors.rounds).toBe(2);
  });

  it('stops when the view goes away', async () => {
    const { session } = await connect();
    session.watchSensors(true);
    await settle(40);
    const rounds = session.snapshot.sensors.rounds;
    session.watchSensors(false);
    await vi.advanceTimersByTimeAsync(SENSOR_POLL_MS * 4);
    expect(session.snapshot.sensors.rounds).toBe(rounds);
    expect(session.snapshot.sensors.watching).toBe(false);
  });

  it('does not ask a board whose capability word does not claim the opcode', async () => {
    // A board with no word at all — the "old firmware" peer. Five questions a
    // round would earn five 0x7F answers, and the tab that would show them is
    // disabled with the reason already on it.
    const { session } = await connect({ features: null });
    session.watchSensors(true);
    await settle(40);
    expect(session.snapshot.sensors.watching).toBe(false);
    expect(session.snapshot.sensors.rounds).toBe(0);
  });

  it('holds all five topics from one round, so the panel is never half-updated', async () => {
    const { session } = await connect();
    session.watchSensors(true);
    await settle(40);
    const answers = session.snapshot.sensors.answers;
    expect(Object.keys(answers).map(Number).sort((a, b) => a - b)).toEqual(
      Array.from({ length: SENSOR_TOPICS }, (_, topic) => topic),
    );
    // One arrival time for the round, which is the age the panel shows. Five
    // topics published as they arrived would give the picture five ages.
    expect(session.snapshot.sensors.atMs).not.toBeNull();
  });

  it('keeps a non-zero reading as a reading and an absent sensor as an absence', async () => {
    const { session } = await connect({ sensors: { [Sensor.BARO]: false } });
    session.watchSensors(true);
    await settle(40);
    const baro = session.snapshot.sensors.answers[Sensor.BARO];
    expect(baro?.status).toBe(SensorStatus.OK);
    expect(baro?.present).toBe(false);
    expect(baro?.body).toBeNull();
    // The IMU on the same board is a reading, and it is not zero: the two
    // answers travel in the same round and this is the check that they are not
    // being flattened into one another on the way through the session.
    const imu = session.snapshot.sensors.answers[Sensor.IMU];
    expect(imu?.present).toBe(true);
    if (imu?.body === null || imu?.body.topic !== Sensor.IMU) throw new Error('not an imu body');
    expect(imu.body.driver).toBe('icm42688p');
    expect(imu.body.accel[2]).toBe(1000);
  });

  it('clears the readings when the link closes, because a voltage is read as now', async () => {
    const { session, link } = await connect();
    session.watchSensors(true);
    await settle(40);
    expect(Object.keys(session.snapshot.sensors.answers).length).toBe(SENSOR_TOPICS);

    link.hangUp('the cable was pulled');
    await settle(40);
    // A driver name and a pack voltage left on screen after the port closed is
    // the app describing an aircraft it is no longer hearing — the same lie as
    // a frozen bar chart, and worse: somebody would go looking for a sensor the
    // *next* board does not have.
    expect(Object.keys(session.snapshot.sensors.answers)).toEqual([]);
    expect(session.snapshot.sensors.atMs).toBeNull();
    expect(session.snapshot.sensors.watching).toBe(false);
  });

  it('reports an old board in the board\'s own terms, once, and keeps reading nothing', async () => {
    // A board with the capability word and the bit clear: it answers `hello`
    // with a word, so the session asks, and the board answers 0x7F. The panel
    // is where that sentence goes; here the assertion is that the failure is
    // recorded once and does not become an event per round.
    const { session } = await connect({ features: 0 });
    session.watchSensors(true);
    await settle(40);
    // The word covers nothing, so this session does not poll at all — the same
    // path as the absent word, and deliberately: both mean "this build does not
    // answer", which is a fact the rail's reason already states.
    expect(session.snapshot.sensors.watching).toBe(false);
    const errors = session.snapshot.events.filter((event) => event.text.includes('sensor'));
    expect(errors).toEqual([]);
  });
});
