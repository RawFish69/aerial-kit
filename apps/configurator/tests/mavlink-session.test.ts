import { describe, expect, it, vi } from 'vitest';

import { detectFirmware } from '../src/protocol/detect';
import { MavlinkDecoder } from '../src/protocol/mavlink';
import { MAV_LIMITATIONS, MavBoard } from '../src/session/mavlink';
import { FakeClock, ManualLink } from './helpers';
import { MavFixtureTransport, MavFixtureVehicle } from './mavlink-link';

/**
 * The MAVLink board: what it reads, and what it cannot do.
 *
 * The vehicle on the other end is the captured one — see `mavlink-link.ts` for
 * why the frames are the reference implementation's rather than this test's.
 * What is under test here is the *session*: that opening the link is passive,
 * that a parameter table is read rather than guessed, and that the class has no
 * way to express any of the writes MAVLink would allow it.
 */

/** A board opened against a vehicle that has already announced itself, the way
 *  detection leaves things: the heartbeat was heard before the board existed. */
function lastEvent(board: MavBoard) {
  const events = board.snapshot.events;
  return events[events.length - 1];
}

function lastAck(board: MavBoard) {
  const acks = board.snapshot.acks;
  return acks[acks.length - 1];
}

async function opened(
  transport = new MavFixtureTransport(),
): Promise<{ board: MavBoard; vehicle: MavFixtureVehicle; transport: MavFixtureTransport }> {
  const vehicle = transport.vehicle;
  vehicle.say('HEARTBEAT');
  const board = new MavBoard(transport, { linkAlreadyOpen: true });
  await board.open();
  return { board, vehicle, transport };
}

describe('recognising a MAVLink vehicle from the port alone', () => {
  it('names the vehicle, and says it is read-only', async () => {
    const vehicle = new MavFixtureVehicle();
    // Announcing on its own schedule, starting after detection has begun. A
    // heartbeat handed over before then would prove nothing: noticing a vehicle
    // that talks unasked is the whole of this step.
    const stop = vehicle.announceEvery(40);
    try {
      const detection = await detectFirmware(vehicle, { probeMs: 20, listenMs: 300 });

      expect(detection.family).toBe('mavlink');
      expect(detection.detail).toContain('ArduPilot');
      expect(detection.detail).toContain('quadrotor');
      expect(detection.detail).toContain('system 1');
      expect(detection.detail).toContain('MAVLink v2');
      expect(detection.detail).toContain('read-only');
      expect(detection.mavlink?.heartbeat?.systemId).toBe(1);
      expect(detection.mavlink?.frames).toBeGreaterThan(0);
    } finally {
      stop();
    }
  });

  it('hands the bytes it heard to the board, so nothing waits for a second heartbeat', async () => {
    const vehicle = new MavFixtureVehicle();
    const stop = vehicle.announceEvery(40);
    try {
      const detection = await detectFirmware(vehicle, { probeMs: 20, listenMs: 300 });
      vehicle.say('ATTITUDE');
      await Promise.resolve();
      const heard = [...detection.mavlink!.heard];

      // A fresh transport with nothing on it: the board is built only from the
      // bytes detection kept, so everything it knows it knows from those.
      const board = new MavBoard(new MavFixtureTransport(), {
        linkAlreadyOpen: true,
        replay: heard,
      });
      await board.open();

      expect(board.snapshot.phase).toBe('ready');
      expect(board.snapshot.identity?.systemId).toBe(1);
      // The board's frame count is what a fresh decoder finds in the same
      // bytes — so `replay` decoded them rather than being handed a summary to
      // believe.
      const fresh = new MavlinkDecoder();
      let expected = 0;
      for (const chunk of heard) expected += fresh.push(chunk).frames.length;
      expect(expected).toBeGreaterThan(0);
      expect(board.snapshot.counts.frames).toBe(expected);
    } finally {
      stop();
    }
  });

  it('reports a truncated MAVLink header as a talking link, not as a vehicle', async () => {
    vi.useFakeTimers();
    try {
      const link = new ManualLink();
      // A MAVLink magic and a length and then nothing — the shape of a stray
      // byte in console noise. Three bytes cannot satisfy a checksum that needs
      // a per-message constant, so this is "something is talking", not a PX4.
      setTimeout(() => link.deliver(Uint8Array.of(0xfd, 0x09, 0x00)), 100);
      const pending = detectFirmware(link, { probeMs: 20, listenMs: 60 });
      await vi.advanceTimersByTimeAsync(500);
      const heard = await pending;

      expect(heard.family).toBe('unrecognised');
      expect(heard.firstByte).toBe(0xfd);
      expect(heard.detail).toContain('no MAVLink frame on it passed its checksum');
    } finally {
      vi.useRealTimers();
    }
  });

  it('reports a vehicle that streams without ever announcing itself', async () => {
    vi.useFakeTimers();
    try {
      const vehicle = new MavFixtureVehicle();
      // After both AerialKit hellos and the MSP probe (3 x 20 ms), inside the listen.
      setTimeout(() => vehicle.say('ATTITUDE'), 80);
      const pending = detectFirmware(vehicle, { probeMs: 20, listenMs: 200 });
      await vi.advanceTimersByTimeAsync(600);
      const detection = await pending;

      // Checksum-valid frames arrived, so the link is MAVLink. No heartbeat, so
      // there is no system id and nothing could be addressed — said outright
      // rather than rounded to "recognised".
      expect(detection.family).toBe('mavlink');
      expect(detection.mavlink?.heartbeat).toBeNull();
      expect(detection.detail).toContain('none of them was a heartbeat');
    } finally {
      vi.useRealTimers();
    }
  });
});

describe('a vehicle, listened to', () => {
  it('sends nothing at all until the vehicle has spoken', async () => {
    const transport = new MavFixtureTransport();
    const board = new MavBoard(transport, { linkAlreadyOpen: true });
    // No heartbeat yet, so `open()` is waiting. Nothing has gone out.
    const opening = board.open();
    expect(transport.vehicle.written).toEqual([]);
    expect(board.snapshot.phase).toBe('listening');

    transport.vehicle.say('HEARTBEAT');
    await opening;
    expect(transport.vehicle.written).toEqual([]);
    expect(board.snapshot.counts.sent).toBe(0);
  });

  it('reads the identity and the armed state from the heartbeat', async () => {
    const { board, vehicle } = await opened();
    vehicle.sayArmed(true);
    await Promise.resolve();

    const identity = board.snapshot.identity!;
    expect(identity.autopilotName).toBe('ArduPilot');
    expect(identity.typeName).toBe('quadrotor');
    // The framing that carried it, not the `mavlink_version` field — which is 3
    // on every autopilot alive and says nothing about this connection.
    expect(identity.framing).toBe(2);
    expect(identity.mavlinkVersion).toBe(3);
    expect(board.snapshot.live.armed).toBe('armed');
    expect(board.snapshot.live.stale).toBe(false);
    expect(lastEvent(board)?.text).toContain('ARMED');
  });

  it('keeps the telemetry the vehicle streams unasked', async () => {
    const { board, vehicle } = await opened();
    for (const name of [
      'ATTITUDE', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'SYS_STATUS',
      'VFR_HUD', 'RC_CHANNELS', 'SERVO_OUTPUT_RAW',
    ]) {
      vehicle.say(name);
    }
    await Promise.resolve();

    const live = board.snapshot.live;
    expect(live.attitude?.yawDeg).toBeCloseTo(270.99999501924856, 9);
    expect(live.position?.latDeg).toBeCloseTo(52.1234567, 7);
    expect(live.position?.hdgDeg).toBeCloseTo(271, 6);
    expect(live.sysStatus?.voltageV).toBeCloseTo(12.6, 6);
    // 78% is what the vehicle said. A negative would mean "I do not know", and
    // this board must not turn that into a number.
    expect(live.sysStatus?.batteryRemainingPct).toBe(78);
    expect(live.rc?.chancount).toBe(8);
    expect(live.servos?.servos[0]).toBe(1500);
    expect(board.snapshot.counts.frames).toBeGreaterThanOrEqual(8);
    expect(board.snapshot.counts.issues).toBe(0);
  });

  it('names what the vehicle is streaming', async () => {
    const { board, vehicle } = await opened();
    vehicle.say('ATTITUDE');
    vehicle.say('ATTITUDE');
    await Promise.resolve();

    const seen = board.snapshot.seen;
    expect(seen.find((item) => item.msgid === 30)).toMatchObject({ name: 'ATTITUDE', count: 2 });
    expect(seen.find((item) => item.msgid === 0)?.name).toBe('HEARTBEAT');
    // Nothing was counted as unread: every frame the captured vehicle sends is
    // one this app carries a definition for.
    expect(board.snapshot.counts.unread).toBe(0);
  });

  it('counts a message it cannot verify rather than parsing a guess', async () => {
    const { board, vehicle } = await opened();
    // `V2_EXTENSION` (248) is not in this app's table. Its checksum needs a
    // CRC_EXTRA that is not on the wire, so a real one cannot be told from
    // noise — and this app does not pretend otherwise. The id is overwritten on
    // a copy of a captured frame, which also invalidates that frame's checksum;
    // that is the point. An unknown id is never decoded, so the checksum that
    // would have caught the edit is never consulted.
    vehicle.say('COMMAND_ACK');
    const mangled = Uint8Array.from(vehicle.lastSaid!);
    mangled[7] = 248; // the low byte of the 24-bit message id
    vehicle.sayBytes(mangled);
    await Promise.resolve();

    expect(board.snapshot.counts.unread).toBe(1);
    // Counted as an issue and *not* as a frame: it was never a message. The two
    // frames are the heartbeat and the ack; the mangled copy is neither.
    expect(board.snapshot.counts.issues).toBe(1);
    expect(board.snapshot.counts.frames).toBe(2);
    // And the refusal is on the record, once, rather than as a stream of them.
    expect(lastEvent(board)?.level).toBe('warn');
    expect(lastEvent(board)?.text).toContain('no definition for (id 248)');
  });

  it('lets the armed state go stale rather than reading as disarmed', async () => {
    const clock = new FakeClock();
    const transport = new MavFixtureTransport();
    const board = new MavBoard(transport, { linkAlreadyOpen: true, now: clock.now });
    transport.vehicle.sayArmed(true);
    await board.open();
    expect(board.snapshot.live.armed).toBe('armed');

    clock.advance(3000);
    board.age();
    expect(board.snapshot.live.stale).toBe(true);
    expect(board.snapshot.live.armed).toBe('unknown');
  });

  it('follows one vehicle and refuses to average two', async () => {
    const { board, vehicle } = await opened();
    vehicle.say('HEARTBEAT', { base_mode: 0 }, { systemId: 7 });
    await Promise.resolve();

    expect(board.snapshot.identity?.systemId).toBe(1);
    expect(lastEvent(board)?.level).toBe('warn');
    expect(lastEvent(board)?.text).toContain('system 7');
  });
});

describe('reading a parameter table', () => {
  it('asks once, and takes the table the vehicle sends back', async () => {
    const { board, vehicle } = await opened();
    vehicle.parameters = [
      { id: 'RATE_RLL_P', value: 0.5, type: 9 },
      { id: 'FENCE_ENABLE', value: 1, type: 2 },
      { id: 'BATT_CAPACITY', value: 5200, type: 9 },
    ];
    await board.readParameters(2000);

    expect(board.snapshot.parameters.map((p) => p.id)).toEqual([
      'RATE_RLL_P', 'FENCE_ENABLE', 'BATT_CAPACITY',
    ]);
    expect(board.snapshot.parameters[2]?.value).toBe(5200);
    expect(board.snapshot.reading).toBeNull();
  });

  it('addresses the request to the vehicle that spoke', async () => {
    const { board, transport, vehicle } = await opened();
    vehicle.parameters = [{ id: 'RATE_RLL_P', value: 0.5, type: 9 }];
    await board.readParameters(2000);

    // One request. The header's system id is this app's own — 255, the
    // conventional ground-station id — and the *target* is the first byte of
    // the payload, which is the id the heartbeat gave. There is no other way to
    // learn it, which is why nothing may be sent before a heartbeat.
    const request = transport.vehicle.written[0]!;
    expect(transport.vehicle.written).toHaveLength(1);
    expect(request[5]).toBe(255);
    expect(request[10]).toBe(1);
  });

  it('says so when the vehicle lists more than it sends', async () => {
    const { board, vehicle } = await opened();
    vehicle.parameters = [{ id: 'RATE_RLL_P', value: 0.5, type: 9 }];
    // The captured frame's own answer: forty declared, one sent.
    vehicle.declaredParameterCount = 40;
    await board.readParameters(2000);

    expect(board.snapshot.parameters).toHaveLength(1);
    expect(lastEvent(board)?.text).toBe('the vehicle listed 40 parameters and sent 1');
  });

  it('asks for a stream rate with the command the reference implementation agrees on', async () => {
    const { board, transport } = await opened();
    board.requestStream(30, 4);
    await Promise.resolve();

    expect(transport.vehicle.written).toHaveLength(1);
    expect(board.snapshot.counts.sent).toBe(1);
    // And the ack the vehicle sent back — the captured `COMMAND_ACK`, which is
    // the reference implementation's answer to `MAV_CMD_SET_MESSAGE_INTERVAL`
    // — is kept and named.
    expect(lastAck(board)).toMatchObject({ command: 511, result: 0 });
    expect(lastAck(board)?.resultName).toBe('accepted');
  });

  it('refuses a rate the vehicle could not honour', async () => {
    const { board } = await opened();
    board.requestStream(30, 0);
    expect(board.snapshot.counts.sent).toBe(0);
    expect(lastEvent(board)?.level).toBe('warn');
  });
});

/**
 * A `COMMAND_ACK` is not agreement. It carries a result, and a vehicle that
 * understands `COMMAND_LONG` and refuses the command inside it answers with
 * `MAV_RESULT_UNSUPPORTED` — a well-formed ack that means the opposite of the
 * one the fixture carries.
 *
 * This is the failure mode a configurator is most likely to have, because the
 * ack's *arrival* is the thing that is easy to notice and its *payload* is the
 * thing that has to be read. An app that logs "asked for message 30 at 4 Hz"
 * when it sent the request, and never reconciles the answer, reports success
 * for a telemetry rate the vehicle never agreed to change — and the person
 * watching sees a rate that did not move and an app that says it did.
 */
describe('the answer to a command, which is not always yes', () => {
  it('says the vehicle refused, rather than that the request was made', async () => {
    const { board, vehicle } = await opened();
    vehicle.commandAckResult = 3; // MAV_RESULT_UNSUPPORTED
    board.requestStream(30, 4);
    await Promise.resolve();
    await Promise.resolve();

    expect(lastAck(board)).toMatchObject({ command: 511, result: 3 });
    expect(lastAck(board)?.resultName).toBe('unsupported');
    expect(lastEvent(board)?.level).toBe('warn');
    expect(lastEvent(board)?.text).toContain('refused');
  });

  it('says the vehicle agreed, when it did', async () => {
    const { board } = await opened();
    board.requestStream(30, 4);
    await Promise.resolve();
    await Promise.resolve();

    expect(lastAck(board)?.resultName).toBe('accepted');
    expect(lastEvent(board)?.level).toBe('info');
    expect(lastEvent(board)?.text).toContain('message 30');
  });

  it('does not report agreement for a vehicle that never answered', async () => {
    const { board, vehicle } = await opened();
    // A vehicle that takes the frame and says nothing. Silence is not consent:
    // it is an unanswered request, and the app has to be able to say which of
    // the three it is looking at. `pending` is the honest word for it — the
    // request has not been refused, and it has not been granted.
    vehicle.answersCommandLong = false;
    board.requestStream(30, 4);
    await Promise.resolve();
    await Promise.resolve();

    expect(board.snapshot.acks).toHaveLength(0);
    expect(lastEvent(board)?.text).not.toContain('accepted');
    expect(board.snapshot.streamRequests[0]).toMatchObject({
      msgid: 30, hz: 4, outcome: 'pending', result: null,
    });
  });

  it('marks a superseded request as never answered, not as refused', async () => {
    const { board, vehicle } = await opened();
    vehicle.answersCommandLong = false;
    board.requestStream(30, 4);
    await Promise.resolve();
    board.requestStream(33, 10);
    await Promise.resolve();
    await Promise.resolve();

    // A second request for the same command takes over the answer slot, so the
    // first will never be answered. That is a different fact from a refusal and
    // from a grant, and the row has to say so.
    expect(board.snapshot.streamRequests.map((request) => request.outcome)).toEqual([
      'unanswered',
      'pending',
    ]);
  });

  it('keeps the outcome of each request, not just the last', async () => {
    const { board, vehicle } = await opened();
    board.requestStream(30, 4);
    await Promise.resolve();
    await Promise.resolve();
    vehicle.commandAckResult = 3;
    board.requestStream(33, 10);
    await Promise.resolve();
    await Promise.resolve();

    expect(board.snapshot.streamRequests.map((request) => request.outcome)).toEqual([
      'accepted',
      'unsupported',
    ]);
  });
});

describe('what this board deliberately cannot do', () => {
  it('has no method that could arm, fly or write to the vehicle', async () => {
    const { board } = await opened();

    // The safety property is the type, not a flag: there is nothing to call.
    for (const forbidden of [
      'arm', 'disarm', 'setMode', 'setParameter', 'writeParameter', 'save',
      'uploadMission', 'takeoff', 'setHome', 'send', 'command', 'write',
    ]) {
      expect(Object.getPrototypeOf(board), forbidden).not.toHaveProperty(forbidden);
    }

    // And the whole surface, listed. This is deliberately exhaustive: adding
    // anything to this class has to be a deliberate act that edits this list in
    // the same commit, so a write cannot arrive quietly.
    const everything = Object.getOwnPropertyNames(Object.getPrototypeOf(board))
      .filter((name) => name !== 'constructor')
      .sort();
    expect(everything).toEqual([
      'age', 'close', 'counts', 'log', 'onCommandAck', 'onFrame', 'onHeartbeat',
      'onIssue', 'open', 'patch', 'readParameters', 'requestStream', 'seenList',
      'snapshot', 'subscribe',
    ]);
  });

  it('states its limits, starting with the one that matters', () => {
    const all = MAV_LIMITATIONS.join(' ');
    expect(MAV_LIMITATIONS[0]).toContain('Read-only');
    expect(all).toContain('PARAM_SET');
    expect(all).toContain('no way to arm');
    expect(all).toContain('goes stale after 2 s');
    // Every claim about hardware has to be a confession, not a boast.
    expect(all).toContain('has been run against a physical vehicle');
  });
});
