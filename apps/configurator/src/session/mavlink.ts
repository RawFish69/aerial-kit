import type { Transport, TransportInfo } from '../transport/types';
import {
  MAV_CMD_SET_MESSAGE_INTERVAL,
  MavResult,
  MavlinkClient,
  MavProtocolError,
  attitudeOf,
  autopilotVersionOf,
  commandAckOf,
  globalPositionOf,
  gpsOf,
  heartbeatOf,
  mavLayout,
  paramValueOf,
  rcChannelsOf,
  servoOutputOf,
  statusTextOf,
  sysStatusOf,
  vfrHudOf,
  type MavAttitude,
  type MavAutopilotVersion,
  type MavCommandAck,
  type MavGlobalPosition,
  type MavGps,
  type MavHeartbeat,
  type MavIssue,
  type MavParamValue,
  type MavRcChannels,
  type MavServoOutput,
  type MavStatusText,
  type MavSysStatus,
  type MavVfrHud,
  type MavlinkFrame,
} from '../protocol/mavlink';
import type { ArmedState } from './types';

/**
 * A vehicle, listened to — and read-only by construction.
 *
 * The third foreign board, and the one with the most ways to hurt somebody. MSP
 * has no write this app would want; MAVLink has all of them. `COMMAND_LONG`
 * carries `MAV_CMD_COMPONENT_ARM_DISARM`, `MAV_CMD_NAV_TAKEOFF` and
 * `MAV_CMD_DO_SET_HOME`, `PARAM_SET` writes a parameter the running autopilot
 * acts on, and a mission upload can fly an aircraft to a place it was not going.
 *
 * So this class has no method that could express any of them. That is the same
 * property `MspBoard` has and it is enforced the same way — by the type, not by
 * a flag a caller could forget. What it can do is exactly:
 *
 *   - **listen.** The vehicle talks on its own; this class keeps what it says.
 *   - **`readParameters()`** — `PARAM_REQUEST_LIST`, which unlike MSP really
 *     does enumerate a parameter table.
 *   - **`requestStream()`** — ask the vehicle to change a telemetry rate. This
 *     is a write, it is stated as one, and it changes what the vehicle *says*,
 *     never what it does.
 *
 * And what it deliberately does not have, which is worth saying out loud
 * because a reader will otherwise assume it is an oversight: **there is no
 * parameter writing.** MAVLink has `PARAM_SET` and ArduPilot refuses one while
 * armed. This app does not send it. A parameter write to a running autopilot is
 * a flight action, and nothing here has been qualified to take one.
 */

/** The same bound the other two families use. A heartbeat that stopped arriving
 *  is not a vehicle that disarmed. */
const STALE_MS = 2000;

export interface MavLive {
  readonly heartbeat: MavHeartbeat | null;
  readonly attitude: MavAttitude | null;
  readonly position: MavGlobalPosition | null;
  readonly gps: MavGps | null;
  readonly sysStatus: MavSysStatus | null;
  readonly vfrHud: MavVfrHud | null;
  readonly rc: MavRcChannels | null;
  readonly servos: MavServoOutput | null;
  /** The vehicle's own messages, newest last, bounded. */
  readonly status: readonly MavStatusText[];
  readonly armed: ArmedState;
  readonly lastSeenMs: number | null;
  readonly stale: boolean;
}

/** One parameter, as the vehicle reported it. */
export interface MavParameter {
  readonly id: string;
  readonly value: number;
  readonly type: number;
  readonly index: number;
  readonly atMs: number;
}

export interface MavEvent {
  readonly atMs: number;
  readonly level: 'info' | 'warn' | 'error';
  readonly text: string;
}

/** A message the vehicle is streaming, whether or not this app can read it. */
export interface MavSeenMessage {
  readonly msgid: number;
  readonly name: string | null;
  readonly count: number;
}

/**
 * A stream-rate request, and what the vehicle said about it.
 *
 * A `COMMAND_ACK` is not agreement: it carries a `MAV_RESULT`, and a vehicle
 * that understands `COMMAND_LONG` and refuses the command inside it answers
 * `unsupported` with every appearance of having answered. So the request and
 * the answer are kept together, and `outcome` is the only place the two meet.
 *
 * The three ways a request can end are deliberately distinct:
 *
 *  - `pending` — sent, and nothing has come back yet.
 *  - a `MAV_RESULT` name — an ack arrived. `accepted` is the only one of these
 *    that means the vehicle agreed to change what it sends.
 *  - `unanswered` — a later request replaced this one and no ack ever arrived
 *    for it. That is not a refusal, and showing it as one would be a different
 *    lie from the one this replaces.
 */
export interface MavStreamRequest {
  readonly msgid: number;
  readonly hz: number;
  /** The command it was sent as. Always `MAV_CMD_SET_MESSAGE_INTERVAL`, since
   *  that is the only one this app can put on the wire. */
  readonly command: number;
  readonly sentAtMs: number;
  readonly outcome: string;
  /** The raw `MAV_RESULT` code, once one arrived. */
  readonly result: number | null;
}

export interface MavSnapshot {
  readonly phase: 'closed' | 'opening' | 'listening' | 'ready' | 'failed';
  readonly failure: string | null;
  readonly transport: TransportInfo;
  readonly identity: MavHeartbeat | null;
  /** From `AUTOPILOT_VERSION`, when the vehicle sends one. Absent is normal:
   *  not every autopilot streams it and none of them streams it unasked. */
  readonly firmware: MavAutopilotVersion | null;
  readonly live: MavLive;
  readonly parameters: readonly MavParameter[];
  readonly reading: { readonly done: number; readonly total: number } | null;
  readonly acks: readonly MavCommandAck[];
  readonly streamRequests: readonly MavStreamRequest[];
  readonly events: readonly MavEvent[];
  readonly seen: readonly MavSeenMessage[];
  readonly counts: {
    readonly frames: number;
    readonly issues: number;
    readonly unread: number;
    readonly signed: number;
    readonly sent: number;
  };
}

/**
 * What this connection cannot establish, stated rather than implied.
 *
 * Same discipline as the other two workspaces. The first line is the one that
 * matters most, because MAVLink is the protocol where a configurator *could*
 * do almost anything.
 */
export const MAV_LIMITATIONS: readonly string[] = [
  'Read-only. This app sends exactly two messages — a parameter-list request and a telemetry-rate request — and has no way to arm, change a flight mode, upload a mission or write a parameter. MAVLink can do all four; this app does not.',
  'MAVLink has PARAM_SET and ArduPilot refuses one while armed. This app never sends it: a parameter write to a running autopilot is a flight action, and nothing here has been qualified to take one.',
  'A stream-rate request is a write to the vehicle. It changes what the vehicle reports, never how it flies.',
  'The armed state is the heartbeat\'s own word and goes stale after 2 s. A stale state is shown as unknown, never as disarmed.',
  'This app carries definitions for twelve messages. Anything else the vehicle streams is counted and named as unread rather than guessed at — MAVLink\'s checksum needs a per-message constant that is not on the wire, so a message this app does not carry cannot be verified and will not be parsed.',
  'Nothing here has been run against a physical vehicle. It is verified against the repository\'s own ArduPilot stand-in and against frames built by pymavlink, the reference implementation.',
];

export class MavBoard {
  private readonly client: MavlinkClient;
  private readonly listeners = new Set<() => void>();
  private state: MavSnapshot;
  private readonly clock: () => number;
  private readonly linkAlreadyOpen: boolean;
  private lastArmedAtMs: number | null = null;
  private status: MavStatusText[] = [];
  private acks: MavCommandAck[] = [];
  private streamRequests: MavStreamRequest[] = [];
  /** Parameters by name, so a second pass that answers out of order cannot
   *  duplicate a row. */
  private readonly byId = new Map<string, MavParameter>();
  private paramCount = 0;
  private reading = false;
  /** The system id this connection is following, fixed by the first heartbeat.
   *  A second vehicle on the same wire is reported, not merged. */
  private systemId: number | null = null;
  /** Issue kinds already reported, so a loose cable does not produce a thousand
   *  identical lines. */
  private readonly issuesSeen = new Set<MavIssue['kind']>();

  constructor(
    private readonly transport: Transport,
    options: {
      readonly now?: () => number;
      readonly heartbeatMs?: number;
      /** The link is already open — detection had to listen for the heartbeat
       *  before this class could be chosen. See `AerialKitSessionOptions`. */
      readonly linkAlreadyOpen?: boolean;
      /** Bytes detection already heard, fed through the same decoder rather
       *  than trusted as a summary. Without this the board waits up to a whole
       *  heartbeat period for the vehicle to come round again. */
      readonly replay?: readonly Uint8Array[];
    } = {},
  ) {
    this.clock = options.now ?? (() => Date.now());
    this.linkAlreadyOpen = options.linkAlreadyOpen ?? false;
    this.state = {
      phase: 'closed',
      failure: null,
      transport: transport.info,
      identity: null,
      firmware: null,
      live: {
        heartbeat: null, attitude: null, position: null, gps: null,
        sysStatus: null, vfrHud: null, rc: null, servos: null,
        status: [], armed: 'unknown', lastSeenMs: null, stale: true,
      },
      parameters: [],
      reading: null,
      acks: [],
      streamRequests: [],
      events: [],
      seen: [],
      counts: { frames: 0, issues: 0, unread: 0, signed: 0, sent: 0 },
    };
    this.client = new MavlinkClient(transport, {
      heartbeatMs: options.heartbeatMs ?? 3000,
    });
    this.client.onFrame((frame) => this.onFrame(frame));
    // A refused frame is not a frame, so `onFrame` never sees it. Without this
    // the counters would sit still exactly when the link went bad.
    this.client.onIssue((issue) => this.onIssue(issue));
    if (options.replay !== undefined) this.client.replay(options.replay);
  }

  get snapshot(): MavSnapshot {
    return this.state;
  }

  subscribe(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  /**
   * Opens the link and waits to be spoken to.
   *
   * There is no probe and nothing is transmitted. A vehicle heartbeats unasked,
   * and until one arrives this app does not know the system id a request would
   * have to be addressed to — so the only correct way to start a MAVLink
   * conversation is to listen for it.
   */
  async open(): Promise<void> {
    this.patch({ phase: 'opening', failure: null });
    try {
      if (!this.linkAlreadyOpen) await this.transport.open();
      this.patch({ phase: 'listening' });
      const heartbeat = await this.client.waitForHeartbeat();
      this.onHeartbeat(heartbeat);
      this.patch({ phase: 'ready' });
      this.log(
        'info',
        `${heartbeat.autopilotName} on a ${heartbeat.typeName}, system ${heartbeat.systemId} — ` +
          `MAVLink v${heartbeat.framing}, ${heartbeat.armed ? 'ARMED' : 'disarmed'}`,
      );
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.patch({ phase: 'failed', failure: message });
      this.log('error', message);
    }
  }

  /**
   * Reads the parameter table.
   *
   * `PARAM_REQUEST_LIST` is answered one parameter at a time, and a vehicle
   * with six hundred of them answers six hundred times. So this resolves when
   * the count the vehicle itself reported has been reached, when nothing new
   * has arrived for a while, or when the deadline passes — and reports which of
   * those happened rather than claiming a complete table either way.
   */
  async readParameters(timeoutMs = 4000): Promise<void> {
    if (this.state.phase !== 'ready') return;
    this.byId.clear();
    this.paramCount = 0;
    this.reading = true;
    this.patch({ parameters: [], reading: { done: 0, total: 0 } });
    try {
      this.client.requestParameterList();
      this.patch({ counts: { ...this.counts(), sent: this.client.stats.sent } });
    } catch (error) {
      this.reading = false;
      this.patch({ reading: null });
      this.log('warn', error instanceof Error ? error.message : String(error));
      return;
    }

    const deadline = this.clock() + timeoutMs;
    let lastCount = 0;
    let quietSince = this.clock();
    while (this.clock() < deadline) {
      await new Promise((resolve) => setTimeout(resolve, 50));
      if (this.byId.size !== lastCount) {
        lastCount = this.byId.size;
        quietSince = this.clock();
        continue;
      }
      // The vehicle said how many there are, and it has now sent them all.
      if (this.paramCount > 0 && this.byId.size >= this.paramCount) break;
      // Or it has stopped answering. Half a table is worth showing and worth
      // labelling as half, which is what `reading` carries.
      if (this.byId.size > 0 && this.clock() - quietSince > 1000) break;
    }

    this.reading = false;
    this.patch({ reading: null });
    const done = this.byId.size;
    if (this.paramCount > 0 && done < this.paramCount) {
      this.log('warn', `the vehicle listed ${this.paramCount} parameters and sent ${done}`);
    } else {
      this.log('info', `read ${done} parameters`);
    }
  }

  /**
   * Asks the vehicle to stream one message at a rate. A write, and named one.
   *
   * The log line says the request was *sent*, not that it was granted, because
   * those are different facts and only the vehicle knows the second one. What
   * it says about that arrives as a `COMMAND_ACK` and is recorded against this
   * request in `streamRequests` — see `onCommandAck`.
   */
  requestStream(msgid: number, hz: number): void {
    try {
      this.client.requestMessageInterval(msgid, hz);
    } catch (error) {
      this.log('warn', error instanceof Error ? error.message : String(error));
      return;
    }
    // A second request supersedes the first: whatever the vehicle answers next
    // is an answer to this one. The earlier request is marked as never answered
    // rather than left looking as though it is still waiting, and rather than
    // being quietly deleted — an unanswered request is a fact about the link.
    this.streamRequests = this.streamRequests.map((request) =>
      request.outcome === 'pending' ? { ...request, outcome: 'unanswered' } : request,
    );
    this.streamRequests = [
      ...this.streamRequests,
      {
        msgid,
        hz,
        command: MAV_CMD_SET_MESSAGE_INTERVAL,
        sentAtMs: this.clock(),
        outcome: 'pending',
        result: null,
      },
    ].slice(-20);
    this.patch({
      counts: { ...this.counts(), sent: this.client.stats.sent },
      streamRequests: this.streamRequests,
    });
    this.log('info', `asked for message ${msgid} at ${hz} Hz`);
  }

  /**
   * The vehicle's answer to the one command this app sends.
   *
   * The ack is filed in `acks` either way, because a table of what the vehicle
   * said is worth having. But it is also *read*, which is the part that was
   * missing: `MAV_RESULT` 0 is the only result that means the vehicle agreed,
   * and anything else is the vehicle declining a request this app had already
   * written down as made. Leaving that unsaid is how a configurator reports a
   * telemetry rate that never changed.
   *
   * `in progress` is the one non-zero result that is not a refusal: the vehicle
   * has taken the command and has not finished with it, so the request stays
   * pending and a later ack decides it.
   */
  private onCommandAck(ack: MavCommandAck): void {
    this.acks = [...this.acks, ack].slice(-20);
    if (ack.command !== MAV_CMD_SET_MESSAGE_INTERVAL) {
      this.patch({ acks: this.acks });
      return;
    }

    // The newest request still waiting. Nothing correlates an ack to a request
    // on the wire, but the command id does, and requests for the same command
    // are answered in the order they were sent.
    let index = -1;
    for (let i = this.streamRequests.length - 1; i >= 0; i--) {
      if (this.streamRequests[i]!.outcome === 'pending') {
        index = i;
        break;
      }
    }
    if (index < 0) {
      this.patch({ acks: this.acks });
      return;
    }

    const request = this.streamRequests[index]!;
    const inProgress = ack.result === MavResult.IN_PROGRESS;
    const outcome = inProgress ? 'pending' : ack.resultName;
    this.streamRequests = this.streamRequests.map((item, at) =>
      at === index ? { ...item, outcome, result: ack.result } : item,
    );
    this.patch({ acks: this.acks, streamRequests: this.streamRequests });

    if (inProgress) return;
    if (outcome === 'accepted') {
      this.log('info', `the vehicle accepted the request for message ${request.msgid} at ${request.hz} Hz`);
    } else {
      this.log(
        'warn',
        `the vehicle refused the request for message ${request.msgid} at ${request.hz} Hz: ` +
          `${outcome} (MAV_RESULT ${ack.result})`,
      );
    }
  }

  /** Ages the armed state against the clock, so "stale" is a fact about time
   *  rather than about when a frame happened to arrive. */
  age(): void {
    if (this.lastArmedAtMs === null) return;
    const elapsed = this.clock() - this.lastArmedAtMs;
    const stale = elapsed > STALE_MS;
    const armed: ArmedState = stale ? 'unknown' : this.state.live.armed;
    if (
      stale === this.state.live.stale &&
      armed === this.state.live.armed &&
      this.state.live.lastSeenMs !== null
    ) {
      this.patch({ live: { ...this.state.live, lastSeenMs: elapsed } });
      return;
    }
    this.patch({ live: { ...this.state.live, lastSeenMs: elapsed, stale, armed } });
  }

  async close(): Promise<void> {
    this.client.close();
    await this.transport.close();
  }

  private counts(): MavSnapshot['counts'] {
    const stats = this.client.stats;
    return {
      frames: stats.frames,
      issues: stats.issues,
      unread: stats.unreadMessages,
      signed: stats.signed,
      sent: stats.sent,
    };
  }

  /** Returns false when the heartbeat came from a system this connection is
   *  not following, so the caller can decline to act on it. */
  private onHeartbeat(heartbeat: MavHeartbeat): boolean {
    if (this.systemId === null) {
      this.systemId = heartbeat.systemId;
      this.patch({ identity: heartbeat });
      return true;
    }
    if (heartbeat.systemId !== this.systemId) {
      // Two vehicles on one link. The screen would otherwise average them.
      this.log(
        'warn',
        `ignoring a heartbeat from system ${heartbeat.systemId}: this connection is following ` +
          `system ${this.systemId}, and two vehicles on one link cannot be told apart`,
      );
      return false;
    }
    this.patch({ identity: heartbeat });
    return true;
  }

  private onFrame(frame: MavlinkFrame): void {
    const counts = this.counts();
    const at = this.clock();

    if (frame.msgid === 0) {
      const heartbeat = heartbeatOf(frame);
      // A heartbeat from another system is a second vehicle on the wire.
      // `onHeartbeat` reports it and says no; this board follows one aircraft.
      if (!this.onHeartbeat(heartbeat)) {
        this.patch({ counts, seen: this.seenList() });
        return;
      }
      this.lastArmedAtMs = at;
      const was = this.state.live.armed;
      const now: ArmedState = heartbeat.armed ? 'armed' : 'disarmed';
      // A change of armed state is the single most consequential thing this
      // screen shows, so it gets a line of its own. Not on the first
      // heartbeat, which `open()` already announced.
      if (was !== 'unknown' && was !== now) {
        this.log('info', `the vehicle is now ${now.toUpperCase()}`);
      }
      this.patch({
        live: {
          ...this.state.live,
          heartbeat,
          armed: now,
          lastSeenMs: 0,
          stale: false,
        },
        counts,
        seen: this.seenList(),
      });
      return;
    }

    // Everything else is data, and a frame from another system's stream is not
    // this aircraft's.
    if (this.systemId !== null && frame.systemId !== this.systemId) {
      this.patch({ counts, seen: this.seenList() });
      return;
    }

    try {
      switch (frame.msgid) {
        case 30:
          this.patch({ live: { ...this.state.live, attitude: attitudeOf(frame) } });
          break;
        case 33:
          this.patch({ live: { ...this.state.live, position: globalPositionOf(frame) } });
          break;
        case 24:
          this.patch({ live: { ...this.state.live, gps: gpsOf(frame) } });
          break;
        case 1:
          this.patch({ live: { ...this.state.live, sysStatus: sysStatusOf(frame) } });
          break;
        case 74:
          this.patch({ live: { ...this.state.live, vfrHud: vfrHudOf(frame) } });
          break;
        case 65:
          this.patch({ live: { ...this.state.live, rc: rcChannelsOf(frame) } });
          break;
        case 36:
          this.patch({ live: { ...this.state.live, servos: servoOutputOf(frame) } });
          break;
        case 22: {
          const parameter = paramValueOf(frame);
          // An empty name is not a parameter. A vehicle that has not yet loaded
          // its table answers with one, and a row called "" would be a lie about
          // what it is called.
          if (parameter.paramId !== '') {
            this.paramCount = parameter.paramCount;
            this.byId.set(parameter.paramId, {
              id: parameter.paramId,
              value: parameter.paramValue,
              type: parameter.paramType,
              index: parameter.paramIndex,
              atMs: at,
            });
            this.patch({
              parameters: [...this.byId.values()].sort((a, b) => a.index - b.index),
              reading: this.reading
                ? { done: this.byId.size, total: parameter.paramCount }
                : this.state.reading,
            });
          }
          break;
        }
        case 253: {
          const text = statusTextOf(frame);
          this.status = [...this.status, text].slice(-100);
          this.patch({ live: { ...this.state.live, status: this.status } });
          break;
        }
        case 77:
          this.onCommandAck(commandAckOf(frame));
          break;
        case 148:
          this.patch({ firmware: autopilotVersionOf(frame) });
          break;
        default:
          break;
      }
    } catch (error) {
      // A message this app carries a definition for but could not read. That is
      // a bug here rather than a fault in the vehicle, and it is worth a line
      // rather than a crash.
      if (error instanceof MavProtocolError) this.log('warn', error.message);
      else throw error;
    }
    this.patch({ counts, seen: this.seenList() });
  }

  /** A frame the decoder refused.
   *
   *  Said once per kind rather than once per frame: a cable that has come loose
   *  produces thousands of checksum failures, and a thousand identical lines
   *  would bury the one that mattered. The running count is on the screen.
   */
  private onIssue(issue: MavIssue): void {
    this.patch({ counts: this.counts(), seen: this.seenList() });
    if (this.issuesSeen.has(issue.kind)) return;
    this.issuesSeen.add(issue.kind);
    const what =
      issue.kind === 'unknown-message'
        ? `a message this app has no definition for (id ${issue.msgid}) — its checksum cannot be checked, so it will not be read`
        : issue.kind === 'checksum'
          ? `a frame whose checksum did not match (message ${issue.msgid}) — a cable problem or a different protocol on the wire`
          : 'a frame with a flag this app does not implement, which it cannot parse';
    this.log('warn', `the vehicle sent ${what}`);
  }

  private seenList(): MavSeenMessage[] {    const out: MavSeenMessage[] = [];
    for (const [msgid, count] of this.client.seen) {
      out.push({ msgid, name: mavLayout(msgid)?.name ?? null, count });
    }
    return out.sort((a, b) => b.count - a.count);
  }

  private log(level: MavEvent['level'], text: string): void {
    this.patch({ events: [...this.state.events, { atMs: this.clock(), level, text }].slice(-100) });
  }

  private patch(change: Partial<MavSnapshot>): void {
    this.state = { ...this.state, ...change };
    for (const listener of this.listeners) listener();
  }
}
