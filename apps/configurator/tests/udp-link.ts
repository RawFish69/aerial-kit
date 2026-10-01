import { createSocket, type Socket } from 'node:dgram';

import type { MavLink } from '../src/protocol/mavlink';
import type { MavFixtureVehicle } from './mavlink-link';

/**
 * A real UDP socket, dressed as the byte pipe a MAVLink client needs.
 *
 * This exists because `tests/sitl.test.ts` needs one and `src/` must not have
 * one: the app's transports are Web Serial and the companion bridge, because
 * those are what a browser has, and nothing ships a UDP MAVLink transport on
 * this evidence. Living in `tests/` alongside `mavlink-link.ts` is what keeps
 * that true — it is the same arrangement the fixture vehicle already has.
 *
 * **The reply address is learned, not configured**, and that is the one
 * non-obvious thing here. ArduPilot's SITL `udp` link is a *connected* client
 * socket: it `connect()`s to the ground station's address and sends from an
 * ephemeral port of its own choosing. So `send(to the configured address)` from
 * here would post every reply straight back to this socket. The address a reply
 * has to reach is the source of the last datagram that arrived, which is only
 * known once the vehicle has spoken — the same condition the app imposes on
 * itself, arrived at for an entirely different reason.
 *
 * A write before then is dropped and counted rather than sent into the void, so
 * a harness bug reads as a number instead of as a timeout. `dropped === 0` is
 * the assertion that says the app never tried to talk first.
 */
export interface Endpoint {
  readonly host: string;
  readonly port: number;
}

export class UdpLink implements MavLink {
  private socket: Socket | null = null;
  private peer: { readonly port: number; readonly address: string } | null = null;
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();

  /** Every datagram, in arrival order, for tests that want to look again. */
  readonly received: Uint8Array[] = [];
  /** Writes refused because the vehicle had not spoken yet. */
  dropped = 0;

  constructor(private readonly local: Endpoint) {}

  /** The port actually bound. Meaningful with `port: 0`, which is how the
   *  loopback self-test avoids colliding with anything else on the machine. */
  get boundPort(): number {
    return this.socket?.address().port ?? 0;
  }

  get learnedPeer(): { readonly port: number; readonly address: string } | null {
    return this.peer;
  }

  async open(): Promise<void> {
    const socket = createSocket({ type: 'udp4', reuseAddr: true });
    this.socket = socket;
    socket.on('message', (data, from) => {
      this.peer = { port: from.port, address: from.address };
      const chunk = new Uint8Array(data);
      this.received.push(chunk);
      for (const handler of this.dataHandlers) handler(chunk);
    });
    socket.on('error', (error: Error) => {
      for (const handler of this.errorHandlers) handler(error.message);
    });
    socket.on('close', () => {
      for (const handler of this.closeHandlers) handler('the socket closed');
    });
    await new Promise<void>((resolve, reject) => {
      socket.once('error', reject);
      socket.bind(this.local.port, this.local.host, () => resolve());
    });
  }

  write(bytes: Uint8Array): void {
    if (this.socket === null) return;
    if (this.peer === null) {
      this.dropped += 1;
      return;
    }
    this.socket.send(Buffer.from(bytes), this.peer.port, this.peer.address);
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

  close(): void {
    this.socket?.close();
    this.socket = null;
  }
}

/**
 * This repository's own fixture vehicle, put on a socket.
 *
 * It exists so `tests/udp-link.test.ts` can run the whole UDP path — the
 * learned reply address, the drop-before-spoken rule, a `MavBoard` reaching
 * `ready` over a socket — without an autopilot binary. That matters more than
 * it sounds: a test file written but never executed is not evidence of anything,
 * and the alternative was shipping a harness whose first contact with a socket
 * would be the run it was written for.
 *
 * **It is a connected client, like the SITL.** The socket `connect()`s to the
 * ground station up front, so it can speak before it is spoken to and sends from
 * an ephemeral port. Modelling that faithfully is the point: a fixture that
 * merely replied to the last datagram would let a broken `UdpLink` pass, because
 * the two would agree on the wrong address.
 *
 * What it cannot do is stand in for a real autopilot. It is `tests/mavlink-link.ts`
 * — this repository's own words — so it cannot surprise anybody, and every
 * semantic assertion in `sitl.test.ts` is a claim about ArduPilot that this
 * cannot support.
 */
export class FixtureVehicleSocket {
  private socket: Socket | null = null;

  constructor(
    private readonly vehicle: MavFixtureVehicle,
    private readonly ground: Endpoint,
  ) {}

  async open(): Promise<void> {
    const socket = createSocket({ type: 'udp4' });
    this.socket = socket;
    socket.on('message', (data) => {
      // What the app sent, delivered the way the fixture expects to receive it.
      this.vehicle.write(new Uint8Array(data));
    });
    await new Promise<void>((resolve, reject) => {
      socket.once('error', reject);
      socket.connect(this.ground.port, this.ground.host, () => resolve());
    });
    // The vehicle's own output goes onto the wire, whenever it decides to speak.
    this.vehicle.onData((frame) => {
      socket.send(Buffer.from(frame));
    });
  }

  close(): void {
    this.socket?.close();
    this.socket = null;
  }
}
