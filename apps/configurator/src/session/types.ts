import type { SetStatus } from '../protocol/constants';
import type { BoardMeta, ParameterValue, RcState, SensorAnswer, Status, Telemetry } from '../protocol/messages';
import type { TransportInfo } from '../transport/types';

/**
 * The vocabulary the whole app is built on, and the reason it exists.
 *
 * A configurator's easiest lie is to treat "the board said ok" as "the aircraft
 * changed". They are four different facts and a person tuning a machine needs
 * to be able to tell them apart:
 *
 *   requested  — what the person typed. Theirs; nothing has happened yet.
 *   echoed     — the board accepted the value into its parameter table. The
 *                wire said so. This is the *only* one a write can establish
 *                on its own, and it is what a naive client would call "done".
 *   applied    — the running firmware has actually rebuilt whatever depends on
 *                the value. The firmware does do this: ak_proto.c fires
 *                ak_proto_io_t's on_change on a successful set, and main.c wires
 *                it to parameters_changed(), which is the same call the console's
 *                `set` makes. What the wire does *not* carry is any word saying
 *                so — the reply is a status byte and the table's own message, and
 *                a client cannot tell one firmware revision from another. So this
 *                app reports `applied: null`, meaning "not established", rather
 *                than reading it out of a byte that does not hold it. A board that
 *                can say so is what the capability word is for.
 *   persisted  — the value will survive losing power. Only `param save` can
 *                establish this, and a board with nowhere to save says so.
 *
 * The `Limitation` list is the same idea turned into something the interface
 * can show. Where the firmware cannot establish a safe or complete write, the
 * connection says so in a sentence naming the source file, rather than shipping
 * a write button and a shrug.
 */

export type ArmedState = 'armed' | 'disarmed' | 'unknown';

/**
 * Why a write is or is not permitted right now.
 *
 * `reason` is always populated, including when the answer is yes — a person
 * about to change a flight controller is owed the reason it was allowed, not
 * only the reason it was not.
 */
export interface Permission {
  readonly allowed: boolean;
  readonly reason: string;
}

export type LimitationId =
  | 'armed-state-gates-writes'
  | 'no-armed-guard-on-set'
  | 'no-application-confirmation'
  | 'no-persistence'
  | 'ranges-from-a-build-time-read'
  | 'stream-promised-not-delivered';

export interface Limitation {
  readonly id: LimitationId;
  readonly severity: 'blocks-writes' | 'qualifies-writes' | 'note';
  /** One line, for the strip at the top of the workspace. */
  readonly summary: string;
  /** The paragraph behind it, with the file and symbol that establish it. */
  readonly detail: string;
  /**
   * Where the claim comes from, as `<path>:<symbol>`.
   *
   * Structured rather than left inside the prose because the prose is what rots.
   * An entry here said the protocol had no `on_change` field long after the
   * field was added, and nothing failed — a paragraph cannot be checked. These
   * can: `tools/check-citations.py` resolves every one of them against the
   * firmware tree and fails when a symbol stops existing.
   *
   * A symbol, not a line number, for the same reason: the line moves on the next
   * edit, the name does not.
   */
  readonly citations: readonly string[];
}

export interface Identity {
  /** What the board calls itself. `aerialkit-f405` is hardware;
   *  `aerialkit-demo` is this page pretending, and the string says so. */
  readonly product: string;
  readonly protocolVersion: number;
  readonly parameterCount: number;
  /** How many parameters differ from what is in flash, the board's own count. */
  readonly changedSinceSaved: string;
  /** The `AK_PROTO_FEATURE_*` word the board reported, or **`null` when its
   *  `hello` ended before the field** — a board predating it. Null and zero are
   *  different claims and must not be collapsed: zero says "this firmware
   *  answers no optional commands", null says "this firmware cannot say". */
  readonly features: number | null;
  /** `ak_params_hash` of the board's live table, or null for a board that does
   *  not send it. This is what a saved configuration is filed under. */
  readonly configHash: number | null;
  readonly isDemo: boolean;
}

/**
 * Firmware-side metadata, **as the board served it**.
 *
 * This was `ParameterMeta`, and it came out of `src/firmware/parameter-table.json`
 * — a build-time snapshot of `ak_flight.c` that had gone stale by sixty rows and
 * was joined to the board's reply `by name`. That join is what made the staleness
 * survivable and therefore invisible: a name the file did not have produced a row
 * with no range, which reads exactly like a parameter that has none.
 *
 * It is `BoardMeta` now, it comes off the wire over `param info`, and there is no
 * fallback to a file. One source that is sometimes absent is honest; two that
 * both always answer is how the off-by-one shipped.
 */
export type { BoardMeta };

/** One write's whole story, kept so the interface can show what happened rather
 *  than only the latest state. */
export interface WriteRecord {
  readonly requested: string;
  /** True when the board answered status 0. The value is in its table. */
  readonly echoed: boolean;
  /**
   * Whether the running firmware rebuilt what depends on the value.
   *
   * Three-valued on purpose, and `null` is the honest one today: the firmware
   * does re-apply (ak_proto_io_t.on_change), but the reply carries no field
   * saying so and a client cannot tell which revision answered. A boolean here
   * would force this app to guess in one direction or the other, and both
   * guesses are wrong — `false` would deny a thing that happens, `true` would
   * claim a thing the wire never said.
   */
  readonly applied: boolean | null;
  readonly status: SetStatus;
  readonly message: string;
  readonly atMs: number;
  /** Why this app is not calling it applied, in a sentence. Required whenever
   *  `applied` is not `true` — which is what stops a write quietly reading as
   *  "done" — and absent when a board has established that it applied. */
  readonly notAppliedBecause?: string;
}

export interface ParameterRow {
  /** The board's own index, and the key everything joins on. Never the name:
   *  a row whose name this app cannot find is a row it has no metadata for,
   *  which is a different thing from a row pointing at its neighbour. */
  readonly index: number;
  /** What the board calls it, from the board's reply. */
  readonly name: string;
  /** What the board last said. `null` before the table has been read. */
  readonly boardValue: string | null;
  /** What the person has typed and not yet sent. `null` means "unchanged". */
  readonly edited: string | null;
  /** The board's description of this row, or **null when it did not give one** —
   *  a board whose firmware predates `param info`, or a row it said it could not
   *  fit. `unavailable` below then says which, in the board's terms. */
  readonly meta: BoardMeta | null;
  /**
   * Why part of this row's description is missing, in the board's terms.
   *
   * It covers both halves of the description, because the two fail for the same
   * kind of reason and a person wants the sentence either way: absent metadata
   * (`meta` is null — the board has no `param info`, or said this entry is too
   * large for one frame), and metadata whose help text did not arrive.
   *
   * `null` alongside a `null` meta means nothing has tried to read it yet. That
   * is not the same as the board having nothing to say, and the panel renders
   * the two differently.
   */
  readonly metaUnavailable: string | null;
  readonly write: WriteRecord | null;
  /** Set when the value on the board differs from what the board reported at
   *  connect. A person who has forgotten which box they changed needs this. */
  readonly changedFromBoot: boolean;
}

export interface LiveView {
  readonly status: Status | null;
  readonly telemetry: Telemetry | null;
  readonly armed: ArmedState;
  /** Milliseconds since the last frame that carried armed state, or null. */
  readonly lastSeenMs: number | null;
  /** True when `lastSeenMs` has passed the staleness bound: the aircraft may
   *  have armed since. Unknown is not disarmed. */
  readonly stale: boolean;
}

export interface TelemetryView {
  /** What the board agreed to when asked, or null if never asked. */
  readonly agreedHz: number | null;
  /** Frames actually received. Zero after a non-zero agreement is a fact worth
   *  showing — it is what the host simulator does. */
  readonly received: number;
  readonly lastFrameMs: number | null;
}

/**
 * The receiver, as of the last `rc channels` poll.
 *
 * This is a *polled* view and is stored as a series of one-off readings rather
 * than as a stream, because that is what the opcode is: the console link cannot
 * stream, and the receiver tab is the one a person wants while holding a
 * transmitter, so polling is what makes it work on the cable they already have.
 *
 * Two fields carry the honesty of it:
 *
 *  - `atMs` is when the last answer arrived, so a frozen bar chart can be read
 *    as frozen. A receiver view with no age on it is a picture claiming to be
 *    live, which is the same lie as an armed banner that never goes stale.
 *  - `watching` is whether this session is polling at all. Nothing is asked for
 *    on a tab nobody has open — the rc poll is one more exchange on a link that
 *    also carries the parameter table, and a page that quietly doubled its own
 *    traffic would make the parameter reads look slow for no visible reason.
 */
export interface RcView {
  /** The board's answer, or `null` before the first one. Cleared when the link
   *  closes: a channel list from a closed port is a reading of a receiver this
   *  app is no longer hearing. */
  readonly state: RcState | null;
  /** Why the last poll did not produce an answer, in the board's terms. */
  readonly error: string | null;
  /** When `state` arrived, or `null` if nothing ever has. */
  readonly atMs: number | null;
  readonly watching: boolean;
  /** Answers received on this connection. */
  readonly polls: number;
  /** Polls that failed since the last answer, so a link dropping every other
   *  exchange is visible rather than merely slow. */
  readonly failed: number;
}

/**
 * The board's sensors, as of the last `sensor info` round.
 *
 * **Held per topic rather than as one reading**, because that is what the opcode
 * is: one question per sensor, each with its own answer, and each answer one of
 * three things rather than a number. A topic with no entry here has not been
 * asked yet, which is *not* the same as an absent sensor and must not be drawn
 * as one — the same distinction `SensorStatus` carries on the wire, one level
 * up.
 *
 * Polled while a view is watching, and slower than the receiver: a driver name
 * and a set of thresholds do not change, and the numbers that do change are
 * pressures and voltages. Polling five topics at the receiver's ten a second
 * would put fifty extra round trips a second onto a link that also carries the
 * parameter table, to move numbers a person cannot read that fast.
 */
export interface SensorView {
  /** The last answer for each topic that has one, keyed by topic number. */
  readonly answers: Readonly<Record<number, SensorAnswer>>;
  /** Why the last round produced no answer, in the board's terms. */
  readonly error: string | null;
  /** When the answers last all arrived, or `null` if none ever has. */
  readonly atMs: number | null;
  readonly watching: boolean;
  /** Complete rounds that finished. One round is one question per topic. */
  readonly rounds: number;
  /** Rounds that failed since the last complete one. */
  readonly failed: number;
}

export interface SessionEvent {
  readonly atMs: number;
  readonly level: 'info' | 'warn' | 'error';
  readonly text: string;
}

export type Phase = 'closed' | 'opening' | 'identifying' | 'reading' | 'ready' | 'failed';

export interface SessionSnapshot {
  readonly phase: Phase;
  readonly failure: string | null;
  readonly openedAtMs: number | null;
  readonly transport: TransportInfo;
  readonly identity: Identity | null;
  readonly parameters: readonly ParameterRow[];
  readonly readError: string | null;
  readonly readProgress: { readonly done: number; readonly total: number } | null;
  readonly live: LiveView;
  readonly telemetry: TelemetryView;
  readonly rc: RcView;
  readonly sensors: SensorView;
  readonly unsaved: number | null;
  readonly permission: Permission;
  readonly limitations: readonly Limitation[];
  readonly events: readonly SessionEvent[];
  readonly counts: {
    readonly frames: number;
    readonly telemetry: number;
    readonly issues: number;
    readonly writes: number;
  };
}

/**
 * What the interface talks to. One implementation per firmware family; this is
 * the seam that lets a second family arrive without the views learning about
 * it.
 */
export interface Session {
  readonly snapshot: SessionSnapshot;
  subscribe(listener: () => void): () => void;
  /** Opens the transport and starts polling. For Web Serial this must be
   *  reached from a user gesture, so the caller opens from a click. */
  open(): Promise<void>;
  /** Ages the armed state against the clock. Driven by the view's tick, so that
   *  "stale" is a fact about time rather than about when a frame happened to
   *  arrive. */
  age(): void;
  /** Reads the identity, then the whole parameter table. */
  refresh(): Promise<void>;
  /** One parameter, re-read from the board. */
  reread(index: number): Promise<void>;
  /** Stages a value locally. Nothing is sent. */
  edit(index: number, value: string): void;
  /** Discards a staged value. */
  revert(index: number): void;
  /** Sends one staged value. Refused locally when the gate is shut. */
  write(index: number): Promise<void>;
  /** Sends every staged value. Keeps going past a refusal — parameters are
   *  independent — but stops if the write gate closes mid-run. */
  writeAll(): Promise<{ written: number; refused: number; stopped: boolean }>;
  /** Writes the parameter table to the board's storage. */
  save(): Promise<void>;
  /** Puts one parameter, or all of them, back to the value this build was
   *  compiled with — `null` for all. Lands in the running table only; saving is
   *  a separate act. Refused locally when the gate is shut. */
  resetParameters(index: number | null): Promise<boolean>;
  /** Asks for a telemetry stream; returns the rate the board agreed to. */
  stream(hz: number): Promise<number>;
  /**
   * Whether to poll `rc channels`. Idempotent, and the reason it takes a
   * boolean rather than being two methods is that a caller unmounting and a
   * caller never having started both mean "stop", and a view cannot always tell
   * which of the two it is.
   *
   * Turning it on asks immediately, so a tab opening onto a blank chart fills
   * on the first round trip rather than after a poll interval.
   */
  watchRc(on: boolean): void;
  /** One `rc channels` exchange, for a caller that wants one now — a person
   *  pressing "read it again", or a test that does not want to wait out a
   *  timer. */
  refreshRc(): Promise<void>;
  /**
   * Whether to poll `sensor info`. The same shape as `watchRc` and for the same
   * reasons: idempotent, asked immediately on `true`, and off on a tab nobody
   * has open.
   *
   * On a board whose capability word does not cover the opcode this does
   * nothing — there is nothing to ask, and the tab that turns it on is
   * unavailable with that reason on it.
   */
  watchSensors(on: boolean): void;
  /** Poll the status fast while a view draws the attitude live. */
  watchAttitude(on: boolean): void;
  /** One pass over every sensor topic, for "read them again" or a test. */
  refreshSensors(): Promise<void>;
  close(): Promise<void>;
}

export type { ParameterValue, RcState, SensorAnswer, Status, Telemetry };
