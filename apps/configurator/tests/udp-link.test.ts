// @vitest-environment node
import { describe, expect, it } from 'vitest';

import { MavBoard } from '../src/session/mavlink';
import { MavFixtureVehicle, capturedFrame } from './mavlink-link';
import { FixtureVehicleSocket, UdpLink } from './udp-link';

/**
 * The UDP test transport, run against a socket rather than described.
 *
 * `tests/sitl.test.ts` is the file that answers assessment task 10's *"genuine
 * PX4/ArduPlane SITL semantic tests"*, and it does not run in CI: it needs an
 * autopilot binary built from a different repository. The risk that creates is
 * specific and worth naming — a harness nobody has run is a harness whose first
 * execution is the one it was written for, and its own history in this ledger
 * says a test is only evidence once it has been run against something.
 *
 * So this file runs the transport half of it, always, against the only vehicle
 * this repository can produce without external code: its own fixture. What it
 * establishes is that the plumbing works — that a `MavBoard` reaches `ready`
 * over a real socket, that the reply address is learned rather than configured,
 * and that the drop-before-spoken rule holds on the wire.
 *
 * **What it deliberately does not establish is anything about ArduPilot.** The
 * fixture is this repository's own stand-in, so it cannot surprise anybody, and
 * the last test below asserts the one thing that makes that concrete: the
 * harness's "fixed wing" claim is a real discriminator, because this vehicle
 * fails it.
 *
 * Everything here is loopback on an ephemeral port, so it collides with nothing
 * and needs no network.
 */

const GROUND = { host: '127.0.0.1', port: 0 };

async function wired(): Promise<{
  link: UdpLink;
  vehicle: MavFixtureVehicle;
  socket: FixtureVehicleSocket;
  stop: () => void;
}> {
  const link = new UdpLink(GROUND);
  await link.open();
  const vehicle = new MavFixtureVehicle();
  // The vehicle connects to the port the ground station actually landed on,
  // which is why `UdpLink.boundPort` exists.
  const socket = new FixtureVehicleSocket(vehicle, {
    host: '127.0.0.1',
    port: link.boundPort,
  });
  await socket.open();
  const stop = (): void => {
    socket.close();
    link.close();
  };
  return { link, vehicle, socket, stop };
}

describe('a MAVLink vehicle on a real socket', () => {
  it('is heard before it is spoken to, and the reply address is learned from the wire', async () => {
    const { link, vehicle, stop } = await wired();
    try {
      // The vehicle speaks first, on its own schedule, from an ephemeral port.
      // Nothing has been written, so nothing can have been learned yet.
      expect(link.learnedPeer).toBeNull();
      vehicle.sayArmed(false);
      await new Promise((resolve) => setTimeout(resolve, 100));

      expect(link.received.length).toBeGreaterThan(0);
      // The peer is the vehicle's *source* port, not the port we bound. A
      // transport that sent replies to its own bound port would have a peer of
      // `ground.port` here, and every reply would be posted back to itself.
      expect(link.learnedPeer).not.toBeNull();
      expect(link.learnedPeer?.port).not.toBe(link.boundPort);
      expect(link.learnedPeer?.address).toBe('127.0.0.1');
      // And nothing was dropped getting to this point: the app stayed quiet
      // until there was somewhere to be quiet *to*.
      expect(link.dropped).toBe(0);

      // **Learning the address and using it are two different things**, and the
      // assertions above only establish the first. The mutation that caught the
      // gap — send to `this.local` instead of `this.peer` — left every line
      // above true, because the peer is still recorded; only a round trip shows
      // where the bytes actually went. So: write a frame the vehicle recognises
      // and require it to arrive.
      const before = vehicle.written.length;
      link.write(capturedFrame('COMMAND_ACK'));
      await new Promise((resolve) => setTimeout(resolve, 100));
      expect(vehicle.written.length).toBe(before + 1);
    } finally {
      stop();
    }
  });

  it('drops a write that has nobody to go to, and counts it', async () => {
    // The structural half of the safety property. A `write` before the vehicle
    // has spoken has no learned address, so it is dropped — and counted, so the
    // harness reports a number rather than timing out.
    const { link, stop } = await wired();
    try {
      link.write(Uint8Array.of(0xfd, 0x00, 0x00));
      expect(link.dropped).toBe(1);
      expect(link.received).toEqual([]);
    } finally {
      stop();
    }
  });

  it('opens a MavBoard to ready over the socket, and back again', async () => {
    const { link, vehicle, stop } = await wired();
    try {
      const stopBeating = vehicle.announceEvery(30);
      const board = new MavBoard(
        {
          info: { kind: 'bridge', label: 'loopback', detail: 'a socket in this test' },
          open: async () => {},
          close: async () => link.close(),
          write: (bytes) => link.write(bytes),
          onData: (handler) => link.onData(handler),
          onClose: (handler) => link.onClose(handler),
          onError: (handler) => link.onError(handler),
        },
        { heartbeatMs: 4000, linkAlreadyOpen: true },
      );
      try {
        await board.open();
        expect(board.snapshot.phase).toBe('ready');
        expect(board.snapshot.identity?.systemId).toBe(1);
        // A conversation over a socket: the request goes out, the ack comes
        // back, and the outcome is recorded against the request. This is the
        // same path the SITL file asserts against ArduPilot.
        board.requestStream(30, 4);
        await new Promise((resolve) => setTimeout(resolve, 200));
        expect(board.snapshot.streamRequests[0]?.outcome).toBe('accepted');
        expect(vehicle.written.length).toBeGreaterThan(0);
      } finally {
        stopBeating();
        await board.close();
      }
    } finally {
      stop();
    }
  });

  it('reaches a parameter table over the socket', async () => {
    const { link, vehicle, stop } = await wired();
    try {
      vehicle.parameters = [
        { id: 'RATE_RLL_P', value: 0.5, type: 9 },
        { id: 'BATT_CAPACITY', value: 5200, type: 9 },
      ];
      const stopBeating = vehicle.announceEvery(30);
      const board = new MavBoard(
        {
          info: { kind: 'bridge', label: 'loopback', detail: 'a socket in this test' },
          open: async () => {},
          close: async () => link.close(),
          write: (bytes) => link.write(bytes),
          onData: (handler) => link.onData(handler),
          onClose: (handler) => link.onClose(handler),
          onError: (handler) => link.onError(handler),
        },
        { heartbeatMs: 4000, linkAlreadyOpen: true },
      );
      try {
        await board.open();
        await board.readParameters(3000);
        expect(board.snapshot.parameters.map((parameter) => parameter.id)).toEqual([
          'RATE_RLL_P',
          'BATT_CAPACITY',
        ]);
      } finally {
        stopBeating();
        await board.close();
      }
    } finally {
      stop();
    }
  });

  it('would tell a fixed wing from a quadrotor, which is what the SITL file asserts', async () => {
    // The one thing this file can say about the SITL harness's assertions. The
    // fixture is an ArduCopter, so `typeName` is `quadrotor` — and
    // `sitl.test.ts` asserts `fixed wing`. That assertion is therefore a real
    // discriminator rather than something every vehicle satisfies, and the
    // proof is here, in a test that runs.
    const { link, vehicle, stop } = await wired();
    try {
      const stopBeating = vehicle.announceEvery(30);
      const board = new MavBoard(
        {
          info: { kind: 'bridge', label: 'loopback', detail: 'a socket in this test' },
          open: async () => {},
          close: async () => link.close(),
          write: (bytes) => link.write(bytes),
          onData: (handler) => link.onData(handler),
          onClose: (handler) => link.onClose(handler),
          onError: (handler) => link.onError(handler),
        },
        { heartbeatMs: 4000, linkAlreadyOpen: true },
      );
      try {
        await board.open();
        expect(board.snapshot.identity?.autopilotName).toBe('ArduPilot');
        expect(board.snapshot.identity?.typeName).toBe('quadrotor');
        expect(board.snapshot.identity?.typeName).not.toBe('fixed wing');
      } finally {
        stopBeating();
        await board.close();
      }
    } finally {
      stop();
    }
  });
});
