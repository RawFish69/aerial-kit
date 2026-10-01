import type { Transport, TransportInfo } from '../transport/types';
import {
  MspClient,
  MspCommand,
  MSP2_CLI_SETTING,
  MSP2_CLI_SETTING_INFO,
  MspRefusal,
  parseAnalog,
  parseAttitude,
  parseRawGps,
  parseSetting,
  parseSettingInfo,
  parseStatus,
  type MspAnalog,
  type MspAttitude,
  type MspIdentity,
  type MspRawGps,
  type MspSettingInfo,
  type MspStatus,
} from '../protocol/msp';
import type { ArmedState } from './types';

/**
 * A board that is not ours, read-only.
 *
 * This is deliberately **not** the `Session` the AerialKit workspace talks to.
 * That interface has `edit`, `write`, `save` and `stream` on it, and a foreign
 * board supports none of them: MSP has no save, no telemetry subscription, and
 * this app will not write a parameter whose semantics it does not know. An
 * implementation that threw from four methods would be a `Session` in name
 * only, and the first caller to forget would find out at runtime.
 *
 * So a foreign board gets its own, smaller vocabulary — one that has no way to
 * express a write. The type is the safety property.
 *
 * What it *does* establish, before anything else: what the board is. Every call
 * here is a read, and the reads are real — the decoders under them are pinned
 * against frames captured from Betaflight's own layouts.
 */

/** Armed state goes stale on the same bound as our own board's. A screen that
 *  keeps saying "disarmed" about a stale frame is the failure mode this exists
 *  to prevent; unknown is not disarmed. */
const STALE_MS = 2000;

export interface ForeignLive {
  readonly status: MspStatus | null;
  readonly attitude: MspAttitude | null;
  readonly analog: MspAnalog | null;
  readonly gps: MspRawGps | null;
  readonly armed: ArmedState;
  readonly lastSeenMs: number | null;
  readonly stale: boolean;
}

/**
 * One setting, as the board answered.
 *
 * `value` is null and `refusal` is set when the board said no. Those are kept
 * apart because they are different facts: a board that has no setting called
 * that is not a board whose setting is empty.
 */
export interface ForeignSetting {
  readonly name: string;
  readonly value: string | null;
  readonly refusal: string | null;
  readonly info: MspSettingInfo | null;
  readonly atMs: number;
}

export interface ForeignEvent {
  readonly atMs: number;
  readonly level: 'info' | 'warn' | 'error';
  readonly text: string;
}

export interface ForeignSnapshot {
  readonly phase: 'closed' | 'opening' | 'reading' | 'ready' | 'failed';
  readonly failure: string | null;
  readonly transport: TransportInfo;
  readonly identity: MspIdentity | null;
  readonly live: ForeignLive;
  readonly settings: readonly ForeignSetting[];
  readonly events: readonly ForeignEvent[];
  readonly counts: { readonly frames: number; readonly issues: number };
}

/**
 * The facts this connection cannot establish, stated rather than implied.
 *
 * Same discipline as the AerialKit workspace's list: if the interface is going
 * to draw a value, it also has to be able to say what it is not.
 */
export const FOREIGN_LIMITATIONS: readonly string[] = [
  'Read-only. This app writes no parameter to a firmware whose semantics it does not know, and MSP has no command here that would.',
  'There is no "list the settings" command in MSP. A setting is read by typing its name; a ground station showing a table ships the list of names itself.',
  'MSP carries no parameter ranges, help text or decimals. Where a value has a unit, the unit is the board\'s, not this app\'s.',
  'The armed state is read from the status frame and goes stale after 2 s. A stale state is shown as unknown, never as disarmed.',
  'MAVLink is not implemented. A vehicle that heartbeats on its own is reported as unrecognised rather than guessed at.',
  'Nothing here has been run against a physical board. It is verified against frames captured from this repository\'s own Betaflight stand-in.',
];

export class MspBoard {
  private readonly client: MspClient;
  private readonly listeners = new Set<() => void>();
  private state: ForeignSnapshot;
  private clock: () => number;
  private lastArmedAtMs: number | null = null;
  private readonly linkAlreadyOpen: boolean;

  constructor(
    private readonly transport: Transport,
    options: {
      readonly now?: () => number;
      readonly timeoutMs?: number;
      /** The link is already open — detection had to ask the board what it was
       *  before this class could be chosen. See `AerialKitSessionOptions`. */
      readonly linkAlreadyOpen?: boolean;
    } = {},
  ) {
    this.clock = options.now ?? (() => Date.now());
    this.linkAlreadyOpen = options.linkAlreadyOpen ?? false;
    this.state = {
      phase: 'closed',
      failure: null,
      transport: transport.info,
      identity: null,
      live: {
        status: null,
        attitude: null,
        analog: null,
        gps: null,
        armed: 'unknown',
        lastSeenMs: null,
        stale: true,
      },
      settings: [],
      events: [],
      counts: { frames: 0, issues: 0 },
    };
    this.client = new MspClient(transport, {
      timeoutMs: options.timeoutMs ?? 1000,
      onIssue: (issue) => this.log('warn', issue),
      // MSP has no telemetry push, so a frame nobody asked for is worth
      // counting rather than discarding quietly. A board that emits them is a
      // board doing something this app does not understand yet.
      onUnsolicited: () => this.patch({ counts: { ...this.state.counts, frames: this.client.stats.frames } }),
      onClosed: (reason) => {
        this.patch({ phase: 'closed' });
        this.log('warn', `the link closed: ${reason}`);
      },
    });
  }

  get snapshot(): ForeignSnapshot {
    return this.state;
  }

  subscribe(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  async open(): Promise<void> {
    this.patch({ phase: 'opening', failure: null });
    try {
      if (!this.linkAlreadyOpen) await this.transport.open();
      this.patch({ phase: 'reading' });
      const identity = await this.client.identify();
      this.patch({ identity });
      this.log(
        'info',
        `${identity.variant.variant} ${identity.release.version || '(release not reported)'} — ` +
          `${identity.board?.boardName ?? '?'} (${identity.board?.targetName ?? '?'})`,
      );
      await this.poll();
      this.patch({ phase: 'ready' });
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.patch({ phase: 'failed', failure: message });
      this.log('error', message);
    }
  }

  /**
   * One round of the live reads.
   *
   * Sequential, because MSP ties a reply to its request by the command number
   * alone and two outstanding requests could not be told apart. One field that
   * fails does not abandon the others: a board that answers status but not GPS
   * is a board with no GPS, and that is worth showing.
   */
  async poll(): Promise<void> {
    const status = await this.read(MspCommand.STATUS, parseStatus);
    const attitude = await this.read(MspCommand.ATTITUDE, parseAttitude);
    const analog = await this.read(MspCommand.ANALOG, parseAnalog);
    const gps = await this.read(MspCommand.RAW_GPS, parseRawGps);

    const at = this.clock();
    if (status !== null) this.lastArmedAtMs = at;
    this.patch({
      live: {
        status: status ?? this.state.live.status,
        attitude: attitude ?? this.state.live.attitude,
        analog: analog ?? this.state.live.analog,
        gps: gps ?? this.state.live.gps,
        armed: status === null ? this.state.live.armed : status.armed ? 'armed' : 'disarmed',
        lastSeenMs: this.lastArmedAtMs === null ? null : at - this.lastArmedAtMs,
        stale: this.lastArmedAtMs === null || at - this.lastArmedAtMs > STALE_MS,
      },
      counts: { frames: this.client.stats.frames, issues: this.client.stats.issues },
    });
    // A board that answered nothing at all has stopped being a connection,
    // whatever the socket says.
    if (status === null && attitude === null && analog === null && gps === null) {
      this.patch({
        live: { ...this.state.live, armed: 'unknown', stale: true },
      });
    }
  }

  /** Ages the armed state against the clock, so "stale" is a fact about time
   *  rather than about when a frame happened to arrive. */
  age(): void {
    if (this.lastArmedAtMs === null) return;
    const elapsed = this.clock() - this.lastArmedAtMs;
    const stale = elapsed > STALE_MS;
    if (stale === this.state.live.stale && this.state.live.lastSeenMs !== null) {
      this.patch({ live: { ...this.state.live, lastSeenMs: elapsed } });
      return;
    }
    this.patch({
      live: { ...this.state.live, lastSeenMs: elapsed, stale, armed: stale ? 'unknown' : this.state.live.armed },
    });
  }

  /**
   * Reads one setting by name.
   *
   * The name is not a nicety of this interface — it is the protocol. There is
   * no request that lists them, so this is the only way to read one, and the
   * board's answer is either `name = value` or a refusal.
   */
  async lookup(name: string): Promise<void> {
    const trimmed = name.trim();
    if (trimmed === '') return;
    const atMs = this.clock();
    try {
      const payload = await this.client.requestV2(MSP2_CLI_SETTING, encode(trimmed));
      const { value } = parseSetting(payload);
      this.record({ name: trimmed, value, refusal: null, info: null, atMs });
    } catch (error) {
      if (error instanceof MspRefusal) {
        // The board said no. That is an answer, and it is recorded as one.
        this.record({
          name: trimmed,
          value: null,
          refusal: 'the board does not have a setting by that name',
          info: null,
          atMs,
        });
        return;
      }
      this.record({
        name: trimmed,
        value: null,
        refusal: error instanceof Error ? error.message : String(error),
        info: null,
        atMs,
      });
    }
  }

  /** The description behind a name: pgn, type, min, max, default. Delivered in
   *  windows by the board, so this asks once and reports what came back. */
  async describe(name: string): Promise<void> {
    const trimmed = name.trim();
    if (trimmed === '') return;
    try {
      const payload = await this.client.requestV2(MSP2_CLI_SETTING_INFO, encode(`${trimmed}\0`));
      const info = parseSettingInfo(payload);
      const existing = this.state.settings.find((item) => item.name === trimmed);
      this.record({
        name: trimmed,
        value: existing?.value ?? null,
        refusal: null,
        info,
        atMs: this.clock(),
      });
    } catch (error) {
      this.log('warn', `no description for ${trimmed}: ${error instanceof Error ? error.message : String(error)}`);
    }
  }

  async close(): Promise<void> {
    this.client.close();
    await this.transport.close();
  }

  private async read<T>(command: number, parse: (payload: Uint8Array) => T): Promise<T | null> {
    try {
      return parse(await this.client.request(command));
    } catch {
      // Silence from one command is not a failure of the connection. MSP
      // answers a command it does not know with nothing at all, so this is the
      // ordinary way a board says "I do not have that".
      return null;
    }
  }

  private record(setting: ForeignSetting): void {
    const others = this.state.settings.filter((item) => item.name !== setting.name);
    this.patch({ settings: [...others, setting].sort((a, b) => a.name.localeCompare(b.name)) });
  }

  private log(level: ForeignEvent['level'], text: string): void {
    this.patch({ events: [...this.state.events, { atMs: this.clock(), level, text }].slice(-100) });
  }

  private patch(change: Partial<ForeignSnapshot>): void {
    this.state = { ...this.state, ...change };
    for (const listener of this.listeners) listener();
  }
}

function encode(text: string): Uint8Array {
  return new TextEncoder().encode(text);
}
