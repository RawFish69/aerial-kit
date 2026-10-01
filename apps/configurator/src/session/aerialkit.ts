import { Feature, has, reasonFor } from '../protocol/features';
import { SENSOR_TOPICS_ORDERED, SetStatus, setStatusName } from '../protocol/constants';
import {
  AerialKitClient,
  CorrelationError,
  LinkClosedError,
  TimeoutError,
  type ParamDescriptor,
} from '../protocol/client';
import type { ParameterValue, Status, Telemetry } from '../protocol/messages';
import { ProtocolError } from '../protocol/messages';
import { DEMO_PRODUCT, type Transport } from '../transport/types';
import { limitationsFor } from './limitations';
import type {
  ArmedState,
  Identity,
  Limitation,
  ParameterRow,
  Permission,
  SensorAnswer,
  Session,
  SessionEvent,
  SessionSnapshot,
  WriteRecord,
} from './types';

/**
 * The armed state is read from frames, so it is only ever as fresh as the last
 * one. A board that armed since would look exactly like one that had not, so
 * past this bound the state is `unknown` and unknown is not disarmed.
 */
export const DEFAULT_STALE_AFTER_MS = 2000;

/** How often STATUS is asked for when no telemetry is arriving. */
export const DEFAULT_POLL_MS = 500;

/**
 * How often `rc channels` is asked for while a view is watching.
 *
 * Faster than the status poll because the two answer different questions: a
 * status frame says whether the aircraft is armed, which does not need to be
 * smooth, and this one is a person moving a stick and watching a bar. Ten a
 * second is smooth enough to see and slow enough that a 115200 link still has
 * room for the parameter reads happening beside it.
 */
export const DEFAULT_RC_POLL_MS = 100;

/**
 * How often the sensor round is asked for while a view is watching.
 *
 * Much slower than the receiver, because the two are not the same kind of
 * reading. A sensor round is **five** round trips rather than one, and what it
 * returns is mostly things that do not change — a driver name, an alignment, a
 * set of pack thresholds. The numbers that do move are a pressure and a pack
 * voltage, which a person reads as "16.4 volts" and not as a trace. Once a
 * second is faster than anybody reads those and slow enough that the five
 * exchanges do not crowd the parameter reads sharing the link.
 */
export const DEFAULT_SENSOR_POLL_MS = 1000;

/** The status poll while a view draws the attitude live: 20 Hz, which is what
 *  a 3D model needs to move rather than jump, and one round trip at a time. */
export const DEFAULT_ATTITUDE_POLL_MS = 50;

/** How long a non-zero telemetry agreement may go without a frame before the
 *  app says so. Long enough that a slow board is not accused. */
export const STREAM_GRACE_MS = 1500;

const MAX_EVENTS = 200;

export interface AerialKitSessionOptions {
  readonly now?: () => number;
  readonly staleAfterMs?: number;
  readonly pollMs?: number;
  /** How often `rc channels` is polled while something is watching. */
  readonly rcPollMs?: number;
  /** How often the sensor round is polled while something is watching. */
  readonly sensorPollMs?: number;
  /** The status poll's period while a view watches the attitude. */
  readonly attitudePollMs?: number;
  readonly streamGraceMs?: number;
  /**
   * Whether to read every row's help text after the table is usable.
   *
   * On by default, and off in tests that are about something else and should
   * not have to account for ninety-two extra round trips. It is *not* a way to
   * turn metadata off: the `param info` walk is part of `refresh()` and always
   * runs when the board claims the capability.
   */
  readonly readHelp?: boolean;
  /**
   * The link is already open.
   *
   * Set when something had to talk to the board *before* deciding what board it
   * was — which is exactly what firmware detection is. Opening an already-open
   * port twice is a hazard on a real device, so rather than hope every
   * transport is idempotent, the caller says so.
   */
  readonly linkAlreadyOpen?: boolean;
}

/**
 * AerialKit, from the configurator's side.
 *
 * The whole class exists to keep four facts apart that a simpler client would
 * merge: what was requested, what the board echoed, what the running firmware
 * applied, and what will survive a power cycle. The class is written so that a
 * write cannot quietly read as "done" — `WriteRecord.echoed` is set only from a
 * status byte, `applied` is three-valued with `null` meaning "this firmware
 * cannot establish it", and every construction site that leaves `applied` at
 * something other than `true` also supplies `notAppliedBecause`. So the one
 * claim the wire supports is the one claim the interface shows.
 */
export class AerialKitSession implements Session {
  private readonly now: () => number;
  private readonly staleAfterMs: number;
  private readonly pollMs: number;
  private readonly rcPollMs: number;
  private readonly attitudePollMs: number;
  /** Whether a view is drawing the attitude, which speeds the status poll up. */
  private attitudeWatchers = 0;
  /** One status exchange at a time: at 20 Hz a slow answer would otherwise
   *  stack requests in the client's queue behind it. */
  private pollBusy = false;
  private readonly sensorPollMs: number;
  private readonly streamGraceMs: number;
  private readonly linkAlreadyOpen: boolean;
  private readonly readHelp: boolean;
  /** Guards the help pass against a second `refresh()` starting one while the
   *  first is still walking, which on a slow link is easy to trigger by
   *  reconnecting. */
  private helpPassRunning = false;

  private client: AerialKitClient | null = null;
  private pollTimer: ReturnType<typeof setInterval> | null = null;
  private rcTimer: ReturnType<typeof setInterval> | null = null;
  private sensorTimer: ReturnType<typeof setInterval> | null = null;
  private streamGraceTimer: ReturnType<typeof setTimeout> | null = null;
  private readonly listeners = new Set<() => void>();

  private state: SessionSnapshot;
  /** The live snapshot object. React reads this by identity, so it is replaced
   *  rather than mutated — a mutated snapshot is a render that never happens. */
  private rows: ParameterRow[] = [];
  private events: SessionEvent[] = [];
  private writesInFlight = false;

  constructor(
    private readonly transport: Transport,
    options: AerialKitSessionOptions = {},
  ) {
    this.now = options.now ?? (() => Date.now());
    this.staleAfterMs = options.staleAfterMs ?? DEFAULT_STALE_AFTER_MS;
    this.pollMs = options.pollMs ?? DEFAULT_POLL_MS;
    this.rcPollMs = options.rcPollMs ?? DEFAULT_RC_POLL_MS;
    this.attitudePollMs = options.attitudePollMs ?? DEFAULT_ATTITUDE_POLL_MS;
    this.sensorPollMs = options.sensorPollMs ?? DEFAULT_SENSOR_POLL_MS;
    this.streamGraceMs = options.streamGraceMs ?? STREAM_GRACE_MS;
    this.readHelp = options.readHelp ?? true;
    this.linkAlreadyOpen = options.linkAlreadyOpen ?? false;
    this.state = this.blank();
  }

  // ---- the snapshot -------------------------------------------------------

  private blank(): SessionSnapshot {
    return {
      phase: 'closed',
      failure: null,
      openedAtMs: null,
      transport: this.transport.info,
      identity: null,
      parameters: [],
      readError: null,
      readProgress: null,
      live: { status: null, telemetry: null, armed: 'unknown', lastSeenMs: null, stale: true },
      telemetry: { agreedHz: null, received: 0, lastFrameMs: null },
      rc: { state: null, error: null, atMs: null, watching: false, polls: 0, failed: 0 },
      sensors: { answers: {}, error: null, atMs: null, watching: false, rounds: 0, failed: 0 },
      unsaved: null,
      permission: {
        allowed: false,
        reason: 'not connected — there is nothing to write to yet',
      },
      limitations: [],
      events: [],
      counts: { frames: 0, telemetry: 0, issues: 0, writes: 0 },
    };
  }

  get snapshot(): SessionSnapshot {
    return this.state;
  }

  subscribe(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  /** Replaces state without telling anyone. Used to get a new live view into
   *  place *before* anything derived from it is computed — the write
   *  permission, the limitation list. Computing those from the previous state
   *  is a bug that reads as a correct answer, which is the worst kind. */
  private set(patch: Partial<SessionSnapshot>): void {
    this.state = { ...this.state, ...patch };
  }

  private publish(patch: Partial<SessionSnapshot>): void {
    this.set(patch);
    for (const listener of this.listeners) listener();
  }

  /** Republish everything that follows from the current state. Called after
   *  `set()`, never instead of it. */
  private publishDerived(): void {
    this.publish({
      permission: this.permission(),
      counts: this.counts(),
      limitations: this.limitations(),
      events: this.events,
    });
  }

  /** Appends to the log *and* publishes it. An event that only lands on the
   *  next unrelated publish is an event that can be lost, and the log is where
   *  a person goes when something has already gone wrong. */
  private event(level: SessionEvent['level'], text: string): void {
    this.events = [...this.events, { atMs: this.now(), level, text }].slice(-MAX_EVENTS);
    this.publish({ events: this.events });
  }

  // ---- connecting ---------------------------------------------------------

  async open(): Promise<void> {
    if (this.client !== null) return;
    this.publish({ phase: 'opening', failure: null });
    this.event('info', `opening ${this.transport.info.label} — ${this.transport.info.detail}`);

    try {
      if (!this.linkAlreadyOpen) await this.transport.open();
    } catch (error) {
      this.fail(message(error));
      return;
    }

    const client = new AerialKitClient(this.transport, {
      onTelemetry: (frame) => this.onTelemetry(frame),
      onClosed: (reason) => this.onClosed(reason),
      onDecodeIssue: (issue) => {
        this.event('warn', issue);
        this.publish({ counts: this.counts() });
      },
    });
    this.client = client;
    this.publish({ phase: 'identifying', openedAtMs: this.now() });
    this.startPolling();
  }

  private fail(text: string): void {
    this.event('error', text);
    this.publish({ phase: 'failed', failure: text, permission: deny(text) });
  }

  private counts() {
    const stats = this.client?.stats;
    return {
      frames: stats?.frames ?? 0,
      telemetry: stats?.telemetry ?? 0,
      issues: stats?.issues ?? 0,
      writes: stats?.writes ?? 0,
    };
  }

  private onClosed(reason: string): void {
    this.stopTimers();
    this.event('warn', `the link closed: ${reason}`);
    // The armed state is not merely old now — there is no link to refresh it
    // from, so it is unknown and stays unknown.
    this.set({
      live: { ...this.state.live, armed: 'unknown', stale: true },
      // The channel list goes the same way, and for a stronger reason: a bar
      // chart is read as *now* in a way a label is not. A frozen set of sticks
      // on a closed port would say a person is still holding a transmitter.
      rc: { ...this.state.rc, state: null, atMs: null, watching: false, error: reason },
      // The sensors go with them, and this one is not a matter of taste: a
      // voltage is read as the pack in front of you and a driver name as the
      // hardware fitted. Keeping either on screen after the port closed would
      // have the app describe an aircraft it is no longer hearing, which is the
      // same lie as a frozen bar chart with a stronger consequence — somebody
      // might go looking for a sensor that the *next* board does not have.
      sensors: { ...this.state.sensors, answers: {}, atMs: null, watching: false, error: reason },
    });
    this.publish({
      phase: 'failed',
      failure: reason,
      permission: deny('the link is closed, so nothing can be sent'),
      counts: this.counts(),
      limitations: this.limitations(),
    });
  }

  private onTelemetry(frame: Telemetry): void {
    const at = this.now();
    this.lastFrameAt = at;
    this.streamStarved = false;
    if (this.streamGraceTimer !== null) {
      clearTimeout(this.streamGraceTimer);
      this.streamGraceTimer = null;
    }
    this.set({
      live: {
        ...this.state.live,
        status: frame,
        telemetry: frame,
        armed: armedFrom(frame),
        lastSeenMs: 0,
        stale: false,
      },
      telemetry: {
        ...this.state.telemetry,
        received: this.state.telemetry.received + 1,
        lastFrameMs: at,
      },
    });
    this.publishDerived();
  }

  private limitations(): Limitation[] {
    return limitationsFor({
      noPersistence: this.noPersistence,
      // Not "agreed to a rate and nothing yet" — a board is allowed a moment to
      // start. Only after the grace has passed with nothing arriving is that a
      // fact worth telling someone.
      streamPromisedNotDelivered: this.streamStarved,
      // Read off the identity rather than kept in a field of its own, so there
      // is one copy of the board's answer. Before `hello` returns there is no
      // identity, and `null` is the right reading then too: this app does not
      // yet know what the board supports, which is what null means.
      capabilities: this.state.identity?.features ?? null,
    });
  }

  private noPersistence = false;
  private streamStarved = false;

  // ---- reading ------------------------------------------------------------

  async refresh(): Promise<void> {
    const client = this.client;
    if (client === null) return;
    this.publish({ phase: this.state.identity === null ? 'identifying' : 'reading' });

    let identity: Identity;
    let unsaved: number | null;
    try {
      const hello = await client.hello();
      identity = {
        product: hello.product,
        protocolVersion: hello.protocolVersion,
        parameterCount: hello.parameterCount,
        changedSinceSaved: hello.changedSinceSaved,
        features: hello.features,
        configHash: hello.configHash,
        isDemo: hello.product === DEMO_PRODUCT,
      };
      // The board's own count of what differs from flash, which is the only
      // trustworthy starting point: a page that assumed zero would tell someone
      // their board had nothing unsaved when it had.
      const changed = Number(hello.changedSinceSaved);
      unsaved =
        hello.changedSinceSaved.trim() !== '' && Number.isFinite(changed) ? changed : null;
    } catch (error) {
      this.fail(`the board did not answer hello: ${message(error)}`);
      return;
    }

    this.event(
      identity.isDemo ? 'warn' : 'info',
      identity.isDemo
        ? `identified as ${identity.product} — a simulated board in this page, not hardware`
        : `identified as ${identity.product}, protocol ${identity.protocolVersion}, ` +
          `${identity.parameterCount} parameters`,
    );
    this.publish({ identity, phase: 'reading', unsaved });

    const read: ParameterValue[] = [];
    try {
      const items = await client.paramList(identity.parameterCount, (done, total) => {
        this.publish({ readProgress: { done, total } });
      });
      read.push(...items);
    } catch (error) {
      this.publish({
        readError: `the table stopped after ${read.length} of ${identity.parameterCount}: ${message(error)}`,
      });
      this.event('error', `reading the parameter table failed: ${message(error)}`);
    }

    // Keyed by the board's own index, and nothing is joined by name. A name the
    // board's two walks disagree about is a fact about the board and is
    // reported as one, below — it must not silently borrow its neighbour's
    // range, which is what the previous name-keyed lookup did.
    this.rows = read.map((item) => ({
      index: item.index,
      name: item.name,
      boardValue: item.value,
      edited: null,
      meta: null,
      metaUnavailable: null,
      write: null,
      changedFromBoot: false,
    }));

    this.event('info', `read ${this.rows.length} of ${identity.parameterCount} parameters`);
    // Ready before the descriptions arrive. The values are what a person came
    // for and they are all here; the ranges and groupings are a second pass, so
    // that a slow board shows its table rather than an empty one.
    this.publish({
      parameters: this.rows,
      phase: 'ready',
      readProgress: null,
      readError: this.state.readError,
      permission: this.permission(),
      limitations: this.limitations(),
    });

    await this.readMetadata(client, identity);
    // Deliberately not awaited: help is prose, the table is already usable, and
    // ninety-two more round trips should not hold up the end of `refresh()`.
    if (this.readHelp) void this.readHelpText(client);

    await this.pollOnce();
  }

  /**
   * The board's description of each row, from `param info`.
   *
   * This is a reading, not a lookup, and every way it can fail produces a
   * sentence on the rows rather than a silent gap:
   *
   *  - the board has no `PARAM_INFO` bit, or no capability word at all, and the
   *    rows say which;
   *  - a row the board said is too large for one frame gets the board's reason;
   *  - a row the board's *two* walks disagree about — described at an index that
   *    the value walk gave a different name — is marked as a disagreement, which
   *    is the one failure a name-keyed join could never have shown.
   *
   * Nothing here falls back to a file. The build-time snapshot this replaces was
   * 32 rows against a 92-parameter board, and its staleness was invisible
   * *because* the join was by name and a missing name looked like a parameter
   * with no range.
   */
  private async readMetadata(client: AerialKitClient, identity: Identity): Promise<void> {
    const served = await this.fetchMetadata(client, identity);
    if ('why' in served) {
      const why = served.why;
      this.rows = this.rows.map((row) => ({ ...row, meta: null, metaUnavailable: why }));
      this.event('warn', `parameter descriptions are not available: ${why}`);
      this.publish({ parameters: this.rows, limitations: this.limitations() });
      return;
    }

    const byIndex = new Map<number, ParamDescriptor>();
    for (const descriptor of served.descriptors) byIndex.set(descriptor.index, descriptor);

    this.rows = this.rows.map((row) => {
      const descriptor = byIndex.get(row.index);
      if (descriptor === undefined) {
        return {
          ...row,
          meta: null,
          metaUnavailable:
            `the board described ${served.descriptors.length} parameters and said ` +
            'nothing about this one',
        };
      }
      if (descriptor.meta !== null && descriptor.meta.name !== row.name) {
        // Both walks came off the same board, so a disagreement is the board
        // contradicting itself rather than the app misreading it. Show the
        // board's value-walk name (that is the one the writes address) and say
        // what the description walk called it.
        return {
          ...row,
          meta: null,
          metaUnavailable:
            `the board's description of index ${row.index} is named ` +
            `\`${descriptor.meta.name}\` and its value is named \`${row.name}\` — ` +
            'the two walks disagree, so this app will not put one row\'s range beside the other\'s name',
        };
      }
      return { ...row, meta: descriptor.meta, metaUnavailable: descriptor.unavailable };
    });

    const described = this.rows.filter((row) => row.meta !== null).length;
    this.event(
      described === this.rows.length ? 'info' : 'warn',
      `the board described ${described} of ${this.rows.length} parameters`,
    );
    this.publish({
      parameters: this.rows,
      readProgress: null,
      limitations: this.limitations(),
    });
  }

  private async fetchMetadata(
    client: AerialKitClient,
    identity: Identity,
  ): Promise<{ descriptors: readonly ParamDescriptor[] } | { why: string }> {
    const capabilities = identity.features;
    if (capabilities !== null && !has(capabilities, Feature.PARAM_INFO)) {
      // The board answered the capability word and left this bit clear, which is
      // an explicit "no". Asking anyway would be asking a question that has been
      // answered.
      const why = reasonFor(capabilities, Feature.PARAM_INFO, 'param info');
      if (why !== null) return { why };
    }
    // `capabilities === null` reaches here: the reply ended before the word, so
    // nothing on the wire says whether this board answers `param info`. One
    // round trip settles it — a board that predates the command replies 0x7F,
    // which is a readable answer and not a timeout — and a measured "it answered
    // 0x7F" is worth more to a person than "this app cannot tell" repeated
    // ninety-two times.
    try {
      const descriptors = await client.paramInfoList(identity.parameterCount, (done, total) => {
        this.publish({ readProgress: { done, total } });
      });
      return { descriptors };
    } catch (error) {
      const heard =
        capabilities === null
          ? "this board's `hello` carried no capability word, so this app asked `param info` " +
            'once to find out'
          : 'this board claims `param info` in its capability word, so this app asked';
      return { why: `${heard}, and ${message(error)}` };
    }
  }

  /**
   * Every row's prose, one `param help` walk each.
   *
   * One row failing does not stop the pass — a board that refuses one index has
   * not stopped having help for the others — but it does get recorded, on the
   * event log and by leaving that row's `help` at null, which the panel renders
   * as "not read" rather than as "this board has none".
   */
  private async readHelpText(client: AerialKitClient): Promise<void> {
    if (this.helpPassRunning) return;
    this.helpPassRunning = true;
    let failed = 0;
    try {
      for (const row of this.rows) {
        if (row.meta === null) continue;
        if (client.isClosed) return;
        try {
          const help = await client.paramHelp(row.index);
          this.replaceRow(row.index, (current) =>
            current.meta === null ? current : { ...current, meta: { ...current.meta, help } },
          );
        } catch (error) {
          failed++;
          const why = message(error);
          this.replaceRow(row.index, (current) =>
            current.meta === null
              ? current
              : { ...current, meta: { ...current.meta, help: null }, metaUnavailable: why },
          );
        }
      }
      if (failed > 0) {
        this.event('warn', `help text did not arrive for ${failed} parameter(s)`);
      }
    } finally {
      this.helpPassRunning = false;
    }
  }

  async reread(index: number): Promise<void> {
    const client = this.client;
    if (client === null) return;
    try {
      const item = await client.paramGet(index);
      if (item === null) {
        this.event('warn', `the board no longer has a parameter at index ${index}`);
        return;
      }
      this.replaceRow(index, (row) => ({
        ...row,
        boardValue: item.value,
        edited: null,
        changedFromBoot: false,
      }));
    } catch (error) {
      this.event('error', `re-reading ${index} failed: ${message(error)}`);
    }
  }

  private replaceRow(index: number, update: (row: ParameterRow) => ParameterRow): void {
    this.rows = this.rows.map((row) => (row.index === index ? update(row) : row));
    this.publish({ parameters: this.rows, counts: this.counts() });
  }

  // ---- writing ------------------------------------------------------------

  private permission(): Permission {
    if (this.client === null || this.client.isClosed) {
      return deny('the link is closed, so nothing can be sent');
    }
    if (this.state.phase !== 'ready' && this.state.phase !== 'reading') {
      return deny('the parameter table has not been read yet');
    }
    const { armed, stale, lastSeenMs } = this.state.live;
    if (lastSeenMs === null && armed === 'unknown') {
      return deny(
        'no frame carrying the armed state has arrived yet. A board that is ' +
          'armed and one that is not look identical until one does.',
      );
    }
    if (stale) {
      return deny(
        `the armed state is ${lastSeenMs ?? 0} ms old and this app stops trusting ` +
          `it at ${this.staleAfterMs} ms — the aircraft may have armed since.`,
      );
    }
    if (armed === 'armed') {
      return {
        allowed: false,
        reason:
          'the board says it is armed. This app does not write to an armed ' +
          'aircraft; disarm it and the write will be enabled.',
      };
    }
    if (armed === 'unknown') {
      return deny('the board reported a flight state this build does not recognise');
    }
    return {
      allowed: true,
      reason: 'the board reports disarmed, read within the last ' + this.staleAfterMs + ' ms',
    };
  }

  edit(index: number, value: string): void {
    this.replaceRow(index, (row) => ({ ...row, edited: value }));
  }

  revert(index: number): void {
    this.replaceRow(index, (row) => ({ ...row, edited: null }));
  }

  async write(index: number): Promise<void> {
    const row = this.rows.find((item) => item.index === index);
    if (row === undefined || row.edited === null) return;

    const permission = this.permission();
    const requested = row.edited;

    // The gate is checked here rather than in the view, so a keyboard shortcut,
    // a test and a button all pass through the same door.
    if (!permission.allowed) {
      this.recordRefusedLocally(index, requested, permission.reason);
      return;
    }

    await this.send(index, requested);
  }

  private recordRefusedLocally(index: number, requested: string, reason: string): void {
    this.event('warn', `refused locally: ${reason}`);
    this.replaceRow(index, (row) => ({
      ...row,
      write: {
        requested,
        echoed: false,
        applied: null,
        // The same status the *board* now answers with, used here for the same
        // refusal on the near side of the wire. It used to be a locally
        // invented `BOARD_REFUSED_WRITE` — a wire status this app made up to
        // describe something that never reached the wire, which meant a reader
        // could not tell a refusal that came back from the board from one this
        // app took it upon itself to make. Both are now the firmware's own
        // status 5, and `notAppliedBecause` is what says which side said it.
        status: SetStatus.REFUSED_ARMED,
        message: reason,
        atMs: this.now(),
        notAppliedBecause: 'it was never sent',
      },
    }));
  }

  private async send(index: number, requested: string): Promise<void> {
    const client = this.client;
    if (client === null) return;
    this.writesInFlight = true;
    try {
      const reply = await client.paramSet(index, requested);
      const echoed = reply.status === SetStatus.OK;

      // The parameter's own row, re-read rather than assumed. The board's
      // answer to `set` carries no value, so the only way to know what it now
      // holds is to ask — and for a float the board rounds and prints, so the
      // value that comes back may not be the text that went out.
      let boardValue: string | null = null;
      if (echoed) {
        const item = await client.paramGet(index);
        boardValue = item?.value ?? null;
      }

      // Application is a *capability* here, not an inference from the reply
      // byte. The reply carries no application field on any revision — it is a
      // status and a sentence — so what this reads is the board's own statement
      // in HELLO about whether a successful set re-applies the configuration.
      // Three-valued and all three reachable:
      //
      //   true   echoed, and the board claims APPLIES_ON_WRITE
      //   false  the board refused it, so nothing was applied
      //   null   echoed, and the board cannot say — no word, or the bit clear
      //
      // `null` must never be spelled `true`. A board whose HELLO ended before
      // the capability word is not a board that re-applies, and a board that
      // reports the word without the bit is one whose own answer is "a set
      // moves the table and nothing else".
      const applies = has(this.state.identity?.features ?? null, Feature.APPLIES_ON_WRITE);
      const applied: boolean | null = !echoed ? false : applies ? true : null;

      const record: WriteRecord = {
        requested,
        // Set only here, and only from a status byte. This is the one line in
        // the app that decides whether a write may be called echoed.
        echoed,
        applied,
        status: reply.status,
        message: reply.message,
        atMs: this.now(),
        // Absent exactly when the board established it. `notAppliedBecause`
        // describes why this app is *not* calling it applied, so a write that
        // was applied has nothing to explain — and a write that was not needs
        // the sentence whether it is null or false. Spread rather than set to
        // `undefined`, so the property is absent rather than present-and-empty
        // and a caller reading `'notAppliedBecause' in record` is not misled.
        ...(applied === true
          ? {}
          : { notAppliedBecause: echoed ? NOT_APPLIED : 'the board refused it' }),
      };

      this.event(
        echoed ? 'info' : 'warn',
        echoed
          ? `${this.rows[index]?.name ?? index}: board now holds ${boardValue ?? '?'} (echoed; application not established)`
          : `${this.rows[index]?.name ?? index}: refused — ${setStatusName(reply.status)}` +
              (reply.message !== '' ? `: ${reply.message}` : ''),
      );

      this.replaceRow(index, (row) => ({
        ...row,
        boardValue: boardValue ?? row.boardValue,
        edited: echoed ? null : row.edited,
        write: record,
        // A write that landed is by definition a change the board has not been
        // told to keep.
        changedFromBoot: echoed ? true : row.changedFromBoot,
      }));

      if (echoed) {
        const unsaved = (this.state.unsaved ?? 0) + 1;
        this.publish({ unsaved });
      }
      this.publish({ permission: this.permission(), counts: this.counts() });
    } catch (error) {
      this.event('error', `writing ${index} failed: ${message(error)}`);
      this.replaceRow(index, (row) => ({
        ...row,
        write: {
          requested,
          echoed: false,
          applied: null,
          status: SetStatus.NO_SUCH_PARAMETER,
          message: message(error),
          atMs: this.now(),
          notAppliedBecause: 'the exchange did not complete',
        },
      }));
    } finally {
      this.writesInFlight = false;
    }
  }

  /**
   * Sends every staged value, and keeps going past a refusal.
   *
   * Parameters are independent — a bad rate gain is not a reason to leave a
   * bad receiver calibration unsent — so one refusal does not stop the rest.
   * What *does* stop it is the gate closing: a telemetry frame may arrive
   * mid-run saying the aircraft just armed, and that is a reason to stop
   * sending anything at all.
   */
  async writeAll(): Promise<{ written: number; refused: number; stopped: boolean }> {
    const staged = this.rows.filter((row) => row.edited !== null);
    let written = 0;
    let refused = 0;
    for (const row of staged) {
      if (!this.permission().allowed) {
        this.event('warn', `stopped before ${row.name}: the write gate closed mid-run`);
        return { written, refused, stopped: true };
      }
      await this.send(row.index, row.edited!);
      const record = this.rows.find((item) => item.index === row.index)?.write;
      if (record?.echoed === true) written++;
      else refused++;
    }
    this.event('info', `${written} written, ${refused} refused`);
    return { written, refused, stopped: false };
  }

  async save(): Promise<void> {
    const client = this.client;
    if (client === null) return;
    try {
      const reply = await client.paramSave();
      if (reply.status === SetStatus.OK) {
        this.noPersistence = false;
        this.event('info', 'the board saved its parameters');
        this.publish({ unsaved: 0, limitations: this.limitations() });
        return;
      }
      if (reply.status === SetStatus.NOWHERE_TO_SAVE) {
        this.noPersistence = true;
        this.event('warn', 'the board has nowhere to save — this does not survive a power cycle');
        this.publish({ limitations: this.limitations() });
        return;
      }
      if (reply.status === SetStatus.REFUSED_ARMED) {
        // The board's own gate, not this app's. It arrives when the aircraft
        // armed since this app's last reading of its state — a race the
        // near-side gate narrows and cannot close, which is exactly why the
        // far side needs its own.
        this.event('warn', 'the board refused to save: the aircraft is armed');
        return;
      }
      this.event('warn', `the board refused to save: ${setStatusName(reply.status)}`);
    } catch (error) {
      this.event('error', `saving failed: ${message(error)}`);
    }
  }

  /**
   * Put parameters back to the values this build was compiled with.
   *
   * `index === null` resets the whole table. Both forms are refused by this
   * app's near-side gate while the aircraft is armed, and both are refused
   * again by the board — the request is a *write*, and it is the same
   * `writable` gate on the far side that answers a set.
   *
   * **This is the one action in the app that changes many parameters at once,
   * and it is why the reply is not thrown away.** After a successful reset the
   * rows are re-read rather than recomputed: the board's defaults are the
   * board's, and a client that filled the table in from its own idea of what
   * the defaults are would be a second authority for the same numbers.
   *
   * What is deliberately *not* here is a reset that then saves. A reset lands
   * in the running table and in the unsaved count like any other write; making
   * it survive a power cycle is a separate act with its own button, and the
   * two must not be one click.
   */
  async resetParameters(index: number | null): Promise<boolean> {
    const client = this.client;
    if (client === null) return false;

    const permission = this.permission();
    if (!permission.allowed) {
      this.event('warn', `refused locally: ${permission.reason}`);
      return false;
    }

    try {
      // Branched rather than passed through, because the client's two modes
      // are two different calls and not one call with a flag. A `number | null`
      // argument would not compile there, deliberately.
      const reply =
        index === null ? await client.paramDefault(null) : await client.paramDefault(index);
      if (reply.status !== SetStatus.OK) {
        this.event(
          'warn',
          `the board would not reset: ${setStatusName(reply.status)}` +
            (reply.message !== '' ? `: ${reply.message}` : ''),
        );
        return false;
      }
      this.event(
        'info',
        index === null
          ? 'the board put every parameter back to its build defaults'
          : `the board put parameter ${index} back to its build default`,
      );
      // Re-read rather than assume. `refresh` walks the table with `param get`
      // and publishes, so the rows, the unsaved count and the hash all come
      // from the board's own answer to "what do you hold now".
      await this.refresh();
      return true;
    } catch (error) {
      this.event('error', `resetting failed: ${message(error)}`);
      return false;
    }
  }

  // ---- streaming ----------------------------------------------------------

  async stream(hz: number): Promise<number> {
    const client = this.client;
    if (client === null) return 0;
    try {
      const agreed = await client.subscribe(hz);
      this.event(
        'info',
        agreed === 0
          ? `the board will not stream on this link (asked for ${hz} Hz)`
          : `the board agreed to ${agreed} Hz`,
      );
      this.set({
        telemetry: {
          agreedHz: agreed,
          received: this.state.telemetry.received,
          lastFrameMs: this.state.telemetry.lastFrameMs,
        },
      });

      // An agreement is not a stream. If none arrives inside the grace, say so
      // rather than leaving a person waiting on a link that will never carry
      // one — which is exactly what the host simulator does.
      if (this.streamGraceTimer !== null) clearTimeout(this.streamGraceTimer);
      this.streamStarved = false;
      if (agreed > 0) {
        this.streamGraceTimer = setTimeout(() => {
          this.streamGraceTimer = null;
          this.streamStarved = true;
          this.event(
            'warn',
            `agreed to ${agreed} Hz and no frame has arrived in ${this.streamGraceMs} ms — ` +
              'subscribed is not the same fact as receiving',
          );
          this.publishDerived();
        }, this.streamGraceMs);
      }
      this.publishDerived();
      return agreed;
    } catch (error) {
      this.event('error', `the telemetry request failed: ${message(error)}`);
      return 0;
    }
  }

  // ---- the receiver -------------------------------------------------------

  /**
   * Start or stop polling `rc channels`.
   *
   * The board is not asked for this until a view says it is looking, and that
   * is a decision about the link rather than about the tabs. Every rc poll is a
   * full round trip on a wire that also carries the parameter table and the
   * status poll; a page that polled it always would make a person's parameter
   * writes slower for a chart nobody has open.
   *
   * Asking immediately on `true` matters for the same reason the status poll
   * runs at connect: a tab that opened onto a blank chart for a tenth of a
   * second and then filled would look like a receiver fault.
   */
  watchRc(on: boolean): void {
    if (this.state.rc.watching === on) return;
    this.set({ rc: { ...this.state.rc, watching: on } });

    if (!on) {
      if (this.rcTimer !== null) clearInterval(this.rcTimer);
      this.rcTimer = null;
      this.publish({ rc: this.state.rc });
      return;
    }

    this.publish({ rc: this.state.rc });
    void this.refreshRc();
    if (this.rcTimer !== null) clearInterval(this.rcTimer);
    this.rcTimer = setInterval(() => {
      void this.refreshRc();
    }, this.rcPollMs);
  }

  /**
   * One `rc channels` exchange.
   *
   * **The failure sentence is kept beside the last reading rather than
   * replacing it, and that is the honest shape.** A board that answers nine
   * polls in ten has a receiver worth looking at; replacing its channels with
   * an error every time one times out would make a working link look broken,
   * and dropping the error would make a broken link look quiet. So the reading
   * keeps its own arrival time and the panel puts the age next to it, and the
   * error is a separate field the panel shows *above* it.
   *
   * A refusal is not retried differently from a success: unlike telemetry
   * there is no agreement to starve, and a board that stops answering is
   * exactly what the age is for.
   */
  async refreshRc(): Promise<void> {
    const client = this.client;
    if (client === null || client.isClosed) return;
    // The same rule the status poll follows: a write in flight is a person
    // waiting, and a rc poll queued ahead of it in the client's own queue would
    // only make the write look slower than it is.
    if (this.writesInFlight) return;
    try {
      const state = await client.rcChannels();
      this.set({
        rc: {
          ...this.state.rc,
          state,
          error: null,
          atMs: this.now(),
          polls: this.state.rc.polls + 1,
          failed: 0,
        },
      });
      // Published through the derived path, not with `publish({ rc })`: the rc
      // reading does not move the permission, but it is one more frame on the
      // link and the counters are what a person checks when the page feels
      // slow.
      this.publishDerived();
    } catch (error) {
      // Not an event per failure. At ten polls a second an unreachable board
      // would fill the log in under a minute and bury whatever else went
      // wrong; the count and the sentence on the panel are where this belongs,
      // and one event is emitted when failures *start*, so the log still has
      // the moment it began.
      const first = this.state.rc.failed === 0;
      this.set({ rc: { ...this.state.rc, error: message(error), failed: this.state.rc.failed + 1 } });
      if (first) this.event('warn', `reading the receiver failed: ${message(error)}`);
      this.publishDerived();
    }
  }

  // ---- the sensors --------------------------------------------------------

  /**
   * Start or stop polling `sensor info`. The same shape as `watchRc`, and the
   * same two reasons: nothing is asked for on a tab nobody has open, and
   * turning it on asks immediately so the tab does not open onto emptiness.
   *
   * **A board whose capability word does not cover the opcode is not asked at
   * all.** The tab is unavailable with that reason written on it, and a poll
   * that would earn five `0x7F` answers a second is five round trips spent
   * learning nothing. The check is the same `features` reading the tab rail
   * uses, so the two cannot disagree about whether this board answers.
   */
  watchSensors(on: boolean): void {
    if (this.state.sensors.watching === on) return;
    const watching = on && this.answersSensors();
    this.set({ sensors: { ...this.state.sensors, watching } });

    if (!watching) {
      if (this.sensorTimer !== null) clearInterval(this.sensorTimer);
      this.sensorTimer = null;
      this.publish({ sensors: this.state.sensors });
      return;
    }

    this.publish({ sensors: this.state.sensors });
    void this.refreshSensors();
    if (this.sensorTimer !== null) clearInterval(this.sensorTimer);
    this.sensorTimer = setInterval(() => {
      void this.refreshSensors();
    }, this.sensorPollMs);
  }

  /** Whether this board's capability word claims `sensor info`. An old board —
   *  one that sent no word at all — does not, which is a fact about the bytes
   *  rather than an inference. */
  private answersSensors(): boolean {
    const features = this.state.identity?.features;
    if (features === null || features === undefined) return false;
    return (features & Feature.SENSOR_INFO) !== 0;
  }

  /**
   * One pass over every sensor topic.
   *
   * **A round is a round: the answers are swapped in together, or none of them
   * are.** Asking five topics one at a time and publishing each as it arrives
   * would draw a panel whose sections are a few hundred milliseconds apart,
   * which on a slow link is a visible stagger and reads as a fault. The set
   * goes in at once and carries one arrival time, so the age on the panel is
   * the age of the picture rather than of its oldest corner.
   *
   * The previous round is kept on failure, like the receiver's, and for the
   * same reason: a board that answers nine rounds in ten has sensors worth
   * looking at. `atMs` is what says the reading is old; the error sits above it.
   */
  async refreshSensors(): Promise<void> {
    const client = this.client;
    if (client === null || client.isClosed) return;
    if (this.writesInFlight) return;
    try {
      const answers: Record<number, SensorAnswer> = {};
      for (const { topic } of SENSOR_TOPICS_ORDERED) {
        answers[topic] = await client.sensorInfo(topic);
      }
      this.set({
        sensors: {
          ...this.state.sensors,
          answers,
          error: null,
          atMs: this.now(),
          rounds: this.state.sensors.rounds + 1,
          failed: 0,
        },
      });
      this.publishDerived();
    } catch (error) {
      // One event when failures start, never one per attempt: at a round a
      // second this would fill the log inside a minute on a board that has
      // gone away, burying whatever else went wrong.
      const first = this.state.sensors.failed === 0;
      this.set({
        sensors: { ...this.state.sensors, error: message(error), failed: this.state.sensors.failed + 1 },
      });
      if (first) this.event('warn', `reading the sensors failed: ${message(error)}`);
      this.publishDerived();
    }
  }

  // ---- polling ------------------------------------------------------------

  private startPolling(): void {
    if (this.pollTimer !== null) return;
    const period = this.attitudeWatchers > 0 ? this.attitudePollMs : this.pollMs;
    this.pollTimer = setInterval(() => {
      void this.pollOnce();
    }, period);
  }

  /**
   * A view is (or stops) drawing the attitude live. The status poll runs at
   * `attitudePollMs` while any view watches and falls back to `pollMs` when the
   * last one stops, so a page with no 3D view on screen costs the link what it
   * always did. Counted rather than a flag, because two views may watch at once.
   */
  watchAttitude(on: boolean): void {
    const before = this.attitudeWatchers > 0;
    this.attitudeWatchers = Math.max(0, this.attitudeWatchers + (on ? 1 : -1));
    const after = this.attitudeWatchers > 0;
    if (before === after || this.pollTimer === null) return;
    clearInterval(this.pollTimer);
    this.pollTimer = null;
    this.startPolling();
  }

  private stopTimers(): void {
    if (this.pollTimer !== null) clearInterval(this.pollTimer);
    this.pollTimer = null;
    if (this.rcTimer !== null) clearInterval(this.rcTimer);
    this.rcTimer = null;
    if (this.sensorTimer !== null) clearInterval(this.sensorTimer);
    this.sensorTimer = null;
    if (this.streamGraceTimer !== null) clearTimeout(this.streamGraceTimer);
    this.streamGraceTimer = null;
  }

  /**
   * One STATUS exchange.
   *
   * Polling rather than trusting a stream, because an AerialKit console link
   * cannot stream and the host simulator does not. Skipped while a write is in
   * flight: the client serialises requests anyway, and a status frame queued
   * behind a write would only make the write look slower than it is.
   */
  private async pollOnce(): Promise<void> {
    const client = this.client;
    if (client === null || client.isClosed || this.writesInFlight || this.pollBusy) return;
    this.pollBusy = true;
    try {
      const status = await client.status();
      this.lastFrameAt = this.now();
      this.set({
        live: { ...this.state.live, status, armed: armedFrom(status), lastSeenMs: 0, stale: false },
      });
      this.publishDerived();
    } catch (error) {
      // A poll that times out is not worth an error line every 500 ms; the
      // staleness clock is what reports it, once it matters.
      if (!(error instanceof TimeoutError)) {
        this.event('warn', `status poll failed: ${message(error)}`);
      }
    } finally {
      this.pollBusy = false;
    }
  }

  /**
   * Ages the armed state, and is called on a tick by the view.
   *
   * This is what makes `unknown` a real state rather than a missing one. A
   * board that armed a second ago and a board that did not are indistinguishable
   * until a frame says which, so past the bound the app stops claiming to know —
   * and a shut gate is the consequence, not a warning beside an open one.
   */
  age(): void {
    const last = this.lastFrameAt;
    if (last === null) return;
    const age = this.now() - last;
    const stale = age > this.staleAfterMs;
    const shown = this.state.live.lastSeenMs ?? -1;
    // Republish on a change of state, and otherwise about four times a second
    // so the age on screen keeps moving. A tick that published every 500 ms
    // regardless would re-render the whole table for nothing.
    if (stale === this.state.live.stale && Math.abs(age - shown) < 250) return;
    this.set({
      live: {
        ...this.state.live,
        lastSeenMs: age,
        stale,
        armed: stale ? 'unknown' : this.state.live.armed,
      },
    });
    this.publishDerived();
  }

  /** When the last frame carrying armed state arrived. */
  private lastFrameAt: number | null = null;

  async close(): Promise<void> {
    this.stopTimers();
    this.client?.close();
    this.client = null;
    await this.transport.close();
    this.event('info', 'closed');
    const blank = this.blank();
    this.rows = [];
    this.state = { ...blank, transport: this.transport.info, events: this.events };
    for (const listener of this.listeners) listener();
  }
}

/**
 * The sentence attached to a write the board accepted but did **not** claim to
 * apply. It is a constant on purpose: this is the app's single most important
 * claim, and it should be identical everywhere it appears.
 *
 * **This is no longer attached to every accepted write, and that is milestone
 * 4's whole point.** It used to be, because there was no way to tell one
 * firmware revision from another: the reply carries a status and a sentence and
 * no application field on any revision, so the app said "not established" on
 * every connection and was right to. HELLO now carries a capability word, and
 * `AK_PROTO_FEATURE_APPLIES_ON_WRITE` is the board stating that a successful
 * set does re-apply. A board that claims it gets `applied: true` and no
 * sentence; a board that cannot say still gets this one.
 *
 * The sentence itself has not changed, because what it describes has not: it
 * used to say the opposite — that the config protocol "does not run the
 * console's post-change callback (F4)" — and that was false. The callback
 * exists (`ak_proto_io_t.on_change`), the protocol fires it on a successful
 * set, and `main.c` wires it to `parameters_changed`, which is the same call
 * the console's `set` makes.
 *
 * The wording is deliberately weaker than "the board applied it" and
 * deliberately stronger than "it did not": `parameters_changed()` re-applies
 * the whole configuration — airframe, navigation profile, output rate,
 * receiver protocol — not one narrow thing, and saying so is what makes the
 * sentence checkable against main.c.
 */
export const NOT_APPLIED =
  'the board put it in its parameter table, which is all this reply can confirm. ' +
  'AerialKit re-applies its configuration on a successful set — the protocol ' +
  'fires ak_proto_io_t.on_change, which main.c wires to parameters_changed() — but ' +
  'this board did not claim AK_PROTO_FEATURE_APPLIES_ON_WRITE, so its own answer ' +
  'is that it may not have. Nothing in the reply reports whether that ran, and a ' +
  'client cannot tell one firmware revision from another by its bytes. So the ' +
  'board may have re-applied it; that it did is not established here.';

function armedFrom(status: Status): ArmedState {
  // ak_flight.h: AK_FLIGHT_DISARMED is 0, AK_FLIGHT_ARMED is 1, and 2..5 are
  // failsafe, returning home, on autopilot and circling down. Anything in the
  // second group is not disarmed *and* is not simply armed — the aircraft is
  // flying itself, and a write is no more appropriate there.
  if (status.flightState === 0) return 'disarmed';
  if (status.flightState === 1) return 'armed';
  if (status.flightState >= 2 && status.flightState <= 5) return 'armed';
  return 'unknown';
}

function deny(reason: string): Permission {
  return { allowed: false, reason };
}

function message(error: unknown): string {
  if (error instanceof TimeoutError) return error.message;
  if (error instanceof LinkClosedError) return error.message;
  if (error instanceof CorrelationError) return error.message;
  if (error instanceof ProtocolError) return error.message;
  if (error instanceof Error) return error.message;
  return String(error);
}
