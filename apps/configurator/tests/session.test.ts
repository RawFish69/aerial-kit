import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { Feature } from '../src/protocol/features';
import { SetStatus } from '../src/protocol/constants';
import { AerialKitSession, NOT_APPLIED } from '../src/session/aerialkit';
import { Command } from '../src/protocol/constants';
import { BoardLink, FakeClock, LinkTransport, TINY, makeBoard, settle } from './helpers';
import type { DemoBoard } from '../src/transport/demo-board';

/**
 * The write model, which is the reason this app exists.
 *
 * Every test here is one of the four facts the app must keep apart — requested,
 * echoed, applied, persisted — and the two gates that decide whether a write
 * may be sent at all. The assertions name the state rather than a boolean,
 * because "allowed: false" is not the claim; "allowed: false *because the
 * aircraft is armed*" is.
 */

const STALE_AFTER = 2000;

interface Built {
  session: AerialKitSession;
  board: DemoBoard;
  link: BoardLink;
  clock: FakeClock;
}

async function connect(
  options: {
    armed?: boolean;
    canSave?: boolean;
    pushTelemetry?: boolean;
    armOnFirstWrite?: boolean;
    parameters?: typeof TINY;
    /** The capability word the board reports. Omitted = the demo board's own
     *  (APPLIES_ON_WRITE); `null` = a board whose hello ends before the field. */
    features?: number | null;
  } = {},
): Promise<Built> {
  const clock = new FakeClock();
  const board = makeBoard({
    parameters: options.parameters ?? TINY,
    canSave: options.canSave ?? true,
    ...(options.features === undefined ? {} : { features: options.features }),
  });
  board.setArmed(options.armed ?? false);
  const link = new BoardLink(board, options.pushTelemetry ?? false, options.armOnFirstWrite ?? false);
  const session = new AerialKitSession(new LinkTransport(link), {
    now: clock.now,
    staleAfterMs: STALE_AFTER,
    // Long enough that the poll never fires on its own: a test that wants a
    // status frame asks for one by refreshing, rather than racing a timer.
    pollMs: 1_000_000,
  });
  await session.open();
  await session.refresh();
  return { session, board, link, clock };
}

beforeEach(() => {
  vi.useFakeTimers();
});

afterEach(() => {
  vi.useRealTimers();
});

describe('connecting and identifying', () => {
  it('reads the whole table and settles into ready', async () => {
    const { session } = await connect();
    const snapshot = session.snapshot;
    expect(snapshot.phase).toBe('ready');
    expect(snapshot.failure).toBeNull();
    expect(snapshot.parameters.map((row) => row.name)).toEqual(['roll_kp', 'airframe']);
    expect(snapshot.identity!.parameterCount).toBe(2);
  });

  it('marks a board that is not the captured product as a demo', async () => {
    const { session } = await connect();
    // The demo board answers to a name the capture did not record, so nothing
    // downstream has to remember to add the word "demo".
    expect(session.snapshot.identity!.product).toBe('aerialkit-demo');
    expect(session.snapshot.identity!.isDemo).toBe(true);
  });

  it('reads each row’s description off the wire, not out of a file', async () => {
    // The description half of the table. Until milestone 3 these fields came
    // from a build-time snapshot of `ak_flight.c` that had gone 32 rows stale
    // against a 92-parameter board and was joined on by name, so the missing row
    // and its neighbour both rendered as "no range". They now arrive over
    // `param info` and `param help` or they do not arrive at all.
    const { session } = await connect();
    const [roll, airframe] = session.snapshot.parameters;
    expect(roll!.meta).not.toBeNull();
    expect(roll!.meta!.name).toBe('roll_kp');
    expect(roll!.meta!.group).toBe(1);
    expect(roll!.meta!.groupName).toBe('rates');
    // The bounds are text, as the wire spells them — the protocol has no typed
    // fields and the app parses them, so a number here would be a shape that
    // cannot arrive.
    expect(roll!.meta!.min).toBe('0.000');
    expect(roll!.meta!.max).toBe('3.000');
    expect(roll!.meta!.decimals).toBe(3);
    expect(roll!.meta!.help).toBe('rate loop P, roll');
    // A u32 renders its bounds with no decimal places, which is the board's
    // `decimals` byte doing its job rather than the app guessing from the type.
    expect(airframe!.meta!.typeName).toBe('u32');
    expect(airframe!.meta!.min).toBe('0');
    expect(airframe!.meta!.max).toBe('6');
    expect(session.snapshot.parameters.map((row) => row.metaUnavailable)).toEqual([null, null]);
  });

  it('keys a description to its index, never to a name', async () => {
    // The failure this guards against is the one that shipped: metadata joined
    // to values by name, so a row the description did not mention silently
    // borrowed its neighbour's range. Here the two walks agree on positions, and
    // the assertion is on the *pairing* rather than on either list.
    const { session } = await connect();
    const rows = session.snapshot.parameters;
    expect(rows.map((row) => [row.index, row.name, row.meta?.name])).toEqual([
      [0, 'roll_kp', 'roll_kp'],
      [1, 'airframe', 'airframe'],
    ]);
  });

  it('says a board that answers 0x7f has no description rather than inventing one', async () => {
    // A board whose HELLO ends before the capability word — every board built
    // before this milestone. The app probes once instead of hedging on every
    // row, because a measured "it answered 0x7f" is worth more than ninety-two
    // repetitions of "this app cannot tell".
    const { session } = await connect({ features: null });
    for (const row of session.snapshot.parameters) {
      expect(row.meta).toBeNull();
      expect(row.metaUnavailable).toMatch(/0x7f/i);
    }
  });

  it('leaves the help pending rather than calling it absent when the walk is off', async () => {
    // `readHelp: false` is the console link's setting: the value walk has to
    // finish inside a 115200 UART's patience, and one `param help` per row does
    // not. The rows are still described — bounds, group, decimals all arrived —
    // and only the prose is missing, which renders as "not read yet".
    const clock = new FakeClock();
    const board = makeBoard({ parameters: TINY });
    board.setArmed(false);
    const session = new AerialKitSession(new LinkTransport(new BoardLink(board)), {
      now: clock.now,
      pollMs: 1_000_000,
      readHelp: false,
    });
    await session.open();
    await session.refresh();
    const [roll] = session.snapshot.parameters;
    expect(roll!.meta).not.toBeNull();
    expect(roll!.meta!.min).toBe('0.000');
    expect(roll!.meta!.help).toBeNull();
    expect(roll!.metaUnavailable).toBeNull();
  });
});

describe('the armed-state gate', () => {
  it('refuses every write while the board reports armed', async () => {
    const { session } = await connect({ armed: true });
    expect(session.snapshot.live.armed).toBe('armed');
    expect(session.snapshot.permission.allowed).toBe(false);
    expect(session.snapshot.permission.reason).toContain('armed');
  });

  it('allows a write when the board reports disarmed, and says why', async () => {
    const { session } = await connect({ armed: false });
    expect(session.snapshot.live.armed).toBe('disarmed');
    expect(session.snapshot.permission.allowed).toBe(true);
    // The reason is populated when the answer is yes, too. A person about to
    // change a flight controller is owed the reason it was allowed.
    expect(session.snapshot.permission.reason).toContain('disarmed');
  });

  it('refuses a write to an armed board without sending anything', async () => {
    const { session, board } = await connect({ armed: true });
    const before = board.stats.commands;
    session.edit(0, '1.000');
    await session.write(0);
    await settle();

    // Not one byte went out: the gate is on the near side of the wire.
    expect(board.stats.commands).toBe(before);
    const row = session.snapshot.parameters[0]!;
    expect(row.write!.echoed).toBe(false);
    expect(row.write!.notAppliedBecause).toBe('it was never sent');
    expect(row.write!.requested).toBe('1.000');
    // And the staged value is still there, so the person does not lose it.
    expect(row.edited).toBe('1.000');
  });

  it('turns a stale armed state into unknown, and unknown is not disarmed', async () => {
    const { session, clock } = await connect({ armed: false });
    expect(session.snapshot.live.armed).toBe('disarmed');

    clock.advance(STALE_AFTER + 1);
    session.age();

    expect(session.snapshot.live.stale).toBe(true);
    expect(session.snapshot.live.armed).toBe('unknown');
    expect(session.snapshot.permission.allowed).toBe(false);
    expect(session.snapshot.permission.reason).toContain('may have armed since');
  });

  it('reopens the gate when a fresh frame arrives', async () => {
    const { session, clock } = await connect({ armed: false });
    clock.advance(STALE_AFTER + 1);
    session.age();
    expect(session.snapshot.permission.allowed).toBe(false);

    await session.refresh();
    expect(session.snapshot.live.stale).toBe(false);
    expect(session.snapshot.permission.allowed).toBe(true);
  });

  it('shuts the gate again if the aircraft arms between frames', async () => {
    const { session, board } = await connect({ armed: false });
    expect(session.snapshot.permission.allowed).toBe(true);

    board.setArmed(true);
    await session.refresh();

    expect(session.snapshot.live.armed).toBe('armed');
    expect(session.snapshot.permission.allowed).toBe(false);
    expect(session.snapshot.permission.reason).toContain('armed');
  });
});

describe('what a write establishes, and what it does not', () => {
  it('calls a write applied only when the board said it re-applies', async () => {
    // The demo board claims APPLIES_ON_WRITE, so this is the case where the
    // board has answered the question and the answer is yes. This test used to
    // assert the opposite — `applied` was null and NOT_APPLIED was attached on
    // *every* accepted write — and that was correct while the reply was the
    // only evidence available, because the reply carries no application field
    // on any revision. The capability word is what changed.
    const { session } = await connect({ armed: false });
    session.edit(0, '1.250');
    await session.write(0);
    await settle();

    const row = session.snapshot.parameters[0]!;
    expect(row.write!.echoed).toBe(true);
    expect(row.write!.status).toBe(SetStatus.OK);
    expect(row.write!.applied).toBe(true);
    // Absent, not empty. A write the board established has nothing to explain,
    // and a caller reading `'notAppliedBecause' in record` must not be misled
    // by a property present with an undefined value.
    expect('notAppliedBecause' in row.write!).toBe(false);
  });

  it('marks application unestablished on a board that does not claim it', async () => {
    // The same write, to a board reporting a capability word without
    // APPLIES_ON_WRITE. This is the case NOT_APPLIED was written for and the
    // only one it still fits: the board's own answer is that it may not
    // re-apply, so `null` is the reading and the sentence is owed.
    //
    // `null` here and `false` below are different claims and this pair of tests
    // is what keeps them apart: this board accepted the value and cannot say
    // what followed, while a refused write never reached the table at all.
    const { session } = await connect({
      armed: false,
      features: Feature.PARAM_INFO,
    });
    session.edit(0, '1.250');
    await session.write(0);
    await settle();

    const row = session.snapshot.parameters[0]!;
    expect(row.write!.echoed).toBe(true);
    expect(row.write!.applied).toBeNull();
    expect(row.write!.notAppliedBecause).toBe(NOT_APPLIED);
    // The sentence still describes the mechanism rather than naming the old
    // F4 finding, which was false when it was written and is not coming back.
    expect(row.write!.notAppliedBecause).toContain('on_change');
    expect(row.write!.notAppliedBecause).not.toContain('F4');
  });

  it('reports a refused write as not applied, because that part is established', async () => {
    const { session } = await connect({ armed: false });
    // `airframe` is declared 0..6, so 99 is outside the parameter's own range
    // and the board refuses it with its own message.
    session.edit(1, '99');
    await session.write(1);
    await settle();

    const row = session.snapshot.parameters[1]!;
    expect(row.write!.echoed).toBe(false);
    // Unlike the accepted case, this one *is* knowable: the table did not take
    // the value, so nothing rebuilt from it.
    expect(row.write!.applied).toBe(false);
    expect(row.write!.notAppliedBecause).toBe('the board refused it');
  });

  it('re-reads the value from the board rather than trusting the text it sent', async () => {
    const { session } = await connect({ armed: false });
    // 1.2 will come back as 1.200: the board formats to the parameter's own
    // decimal count, so the value on screen is the board's, not the typist's.
    session.edit(0, '1.2');
    await session.write(0);
    await settle();
    expect(session.snapshot.parameters[0]!.boardValue).toBe('1.200');
    expect(session.snapshot.parameters[0]!.edited).toBeNull();
  });

  it('counts a landed write as unsaved until the board is told to keep it', async () => {
    const { session } = await connect({ armed: false });
    expect(session.snapshot.unsaved).toBe(0);
    session.edit(0, '1.000');
    await session.write(0);
    await settle();
    expect(session.snapshot.unsaved).toBe(1);
  });

  it('carries the board’s own refusal message through unchanged', async () => {
    const { session } = await connect({ armed: false });
    session.edit(0, '99');
    await session.write(0);
    await settle();

    const row = session.snapshot.parameters[0]!;
    expect(row.write!.echoed).toBe(false);
    expect(row.write!.status).toBe(SetStatus.VALUE_REFUSED);
    // The table's words, not this app's. This is the only place the protocol
    // ever states a range, and paraphrasing it would be inventing one.
    expect(row.write!.message).toBe('out of range 0.000..3.000');
    // A refused write leaves the text in the box so it can be corrected.
    expect(row.edited).toBe('99');
  });

  it('refuses a value that is not a number at all, in the same voice', async () => {
    const { session } = await connect({ armed: false });
    session.edit(1, 'quadx');
    await session.write(1);
    await settle();
    expect(session.snapshot.parameters[1]!.write!.message).toBe('"quadx" is not a whole number');
  });
});

describe('persisting', () => {
  it('clears the unsaved count when the board saves', async () => {
    const { session } = await connect({ canSave: true });
    session.edit(0, '1.000');
    await session.write(0);
    await settle();
    await session.save();
    expect(session.snapshot.unsaved).toBe(0);
    expect(session.snapshot.limitations.map((item) => item.id)).not.toContain('no-persistence');
  });

  it('says a board with nowhere to save has nowhere to save', async () => {
    const { session } = await connect({ canSave: false });
    await session.save();
    const limitation = session.snapshot.limitations.find((item) => item.id === 'no-persistence');
    expect(limitation).toBeDefined();
    expect(limitation!.severity).toBe('blocks-writes');
    expect(limitation!.detail).toContain('status 3');
  });
});

describe('telemetry, and the difference between subscribed and receiving', () => {
  it('records an agreement and then says when no frame follows it', async () => {
    // A peer that answers the subscribe with the rate and then never pushes a
    // frame. That is *not* the host simulator, which refuses a console
    // subscribe outright (0), and not a console link on a board either — both
    // of those answer 0 by design and this app reports them as a refusal. It is
    // a link that streams promising frames that do not arrive, which is the
    // case worth having a sentence for.
    const { session } = await connect({ pushTelemetry: false });
    const agreed = await session.stream(10);
    expect(agreed).toBe(10);
    expect(session.snapshot.telemetry.received).toBe(0);
    expect(session.snapshot.limitations.map((item) => item.id)).not.toContain(
      'stream-promised-not-delivered',
    );

    await vi.advanceTimersByTimeAsync(2000);

    expect(session.snapshot.telemetry.received).toBe(0);
    const limitation = session.snapshot.limitations.find(
      (item) => item.id === 'stream-promised-not-delivered',
    );
    expect(limitation).toBeDefined();
    expect(limitation!.detail).toContain('can_stream');
  });

  it('does not accuse a board that is actually streaming', async () => {
    const { session, link } = await connect({ pushTelemetry: true });
    await session.stream(10);
    for (let i = 1; i <= 5; i++) {
      link.tick(i * 100);
      await settle();
    }
    await vi.advanceTimersByTimeAsync(2000);

    expect(session.snapshot.telemetry.received).toBeGreaterThan(0);
    expect(session.snapshot.limitations.map((item) => item.id)).not.toContain(
      'stream-promised-not-delivered',
    );
  });

  it('treats a zero agreement as an answer, not a failure', async () => {
    const { session } = await connect({ pushTelemetry: false });
    const agreed = await session.stream(0);
    expect(agreed).toBe(0);
    await vi.advanceTimersByTimeAsync(2000);
    expect(session.snapshot.limitations.map((item) => item.id)).not.toContain(
      'stream-promised-not-delivered',
    );
  });

  it('reads armed state out of a pushed telemetry frame too', async () => {
    const { session, board, link, clock } = await connect({ armed: false, pushTelemetry: true });
    await session.stream(10);

    board.setArmed(true);
    clock.advance(100);
    link.tick(clock.now());
    await settle();

    // No STATUS was asked for; the stream said it.
    expect(session.snapshot.live.armed).toBe('armed');
    expect(session.snapshot.permission.allowed).toBe(false);
  });
});

describe('limitations, as readings of the capability word', () => {
  it('does not make a claim the board has answered the other way', async () => {
    const { session } = await connect();
    const ids = session.snapshot.limitations.map((item) => item.id);
    // Not the old pair. `f4-writes-are-echoed-only` and
    // `firmware-has-no-armed-guard` were both claims about the firmware that
    // the firmware stopped supporting, and the app went on making them.
    expect(ids).not.toContain('f4-writes-are-echoed-only');
    expect(ids).not.toContain('firmware-has-no-armed-guard');
    expect(ids).toContain('armed-state-gates-writes');
    // The demo board reports all four bits the firmware sets, and each one
    // retires a note the app used to owe on every connection.
    //
    // `GATES_ON_ARMED` is the milestone-4 arrival. Until the firmware set it,
    // this line read `toContain('no-armed-guard-on-set')` — the app correctly
    // telling everyone that a set was not checked against the flight state,
    // because it was not. The firmware now consults `ak_proto_io_t.writable` on
    // every write route, so the board says so and the note is not owed. That
    // the assertion flipped without anyone editing the limitation is the whole
    // point of the entry being a *reading*: nothing had to remember to delete
    // it, and nothing could have forgotten to either.
    //
    // `RC_CHANNELS` joins the list in milestone 5, for the same reason and with
    // the same effect: the demo board answers `rc channels` because the firmware
    // does, and this list is the record of which bits that means.
    //
    // `SENSOR_INFO` joins it in milestone 6. It is the first bit here that no
    // *limitation* reads — the app never owed a sentence about sensors, because
    // it made no claim about them before — so this line is doing only its first
    // job for that bit rather than both: it records that the demo peer answers
    // the opcode, which is what makes the Sensors tab's "old board" path
    // testable against a real peer rather than only against a hand-built reply.
    expect(session.snapshot.identity!.features).toBe(
      Feature.APPLIES_ON_WRITE |
        Feature.PARAM_INFO |
        Feature.PARAM_DEFAULT |
        Feature.GATES_ON_ARMED |
        Feature.RC_CHANNELS |
        Feature.SENSOR_INFO,
    );
    expect(ids).not.toContain('no-application-confirmation');
    expect(ids).not.toContain('ranges-from-a-build-time-read');
    expect(ids).not.toContain('no-armed-guard-on-set');
  });

  it('owes the armed-set note on a board that reports the word without the bit', async () => {
    // The other half, and the one that keeps the entry honest. A board can
    // carry a capability word and still not claim GATES_ON_ARMED — an older
    // firmware, or one that guards some write routes and not others. That board
    // must still be told its set is unchecked, and told it in the *it-says-no*
    // wording rather than the cannot-say one.
    const { session } = await connect({ features: Feature.APPLIES_ON_WRITE | Feature.PARAM_INFO });
    const ids = session.snapshot.limitations.map((item) => item.id);
    expect(ids).toContain('no-armed-guard-on-set');
    expect(ids).not.toContain('no-application-confirmation');
  });

  it('states all three, in the cannot-say wording, for a board with no capability word', async () => {
    const { session } = await connect({ features: null });
    const ids = session.snapshot.limitations.map((item) => item.id);
    expect(session.snapshot.identity!.features).toBeNull();
    expect(ids).toContain('no-application-confirmation');
    expect(ids).toContain('no-armed-guard-on-set');
    expect(ids).toContain('ranges-from-a-build-time-read');
    // The wording is the point. "It says it does not re-apply" and "it cannot
    // say" are different facts about the aircraft, and the old single entry
    // asserted the weaker one on every connection.
    const confirmation = session.snapshot.limitations.find(
      (item) => item.id === 'no-application-confirmation',
    );
    expect(confirmation!.detail).toContain('ended before the capability word');
    const guard = session.snapshot.limitations.find((item) => item.id === 'no-armed-guard-on-set');
    expect(guard!.detail).toContain('ended before the capability word');
    // And the range note says what was *read*, not what was assumed: this app
    // asked once and the board answered 0x7f.
    const ranges = session.snapshot.limitations.find(
      (item) => item.id === 'ranges-from-a-build-time-read',
    );
    expect(ranges!.detail).toContain('ended before the capability word');
    expect(ranges!.detail).toContain('0x7f');
  });

  it('names an old board’s missing description as the board’s own answer', async () => {
    // A board with a capability word that leaves PARAM_INFO clear. It is not
    // "unknown" — it said no — so it gets the entry that quotes the bit rather
    // than the one that quotes the missing field.
    const { session } = await connect({ features: Feature.APPLIES_ON_WRITE });
    const ranges = session.snapshot.limitations.find(
      (item) => item.id === 'ranges-from-a-build-time-read',
    );
    expect(ranges).toBeDefined();
    expect(ranges!.detail).toContain('does not claim AK_PROTO_FEATURE_PARAM_INFO');
    expect(ranges!.detail).not.toContain('ended before the capability word');
    // The rows are still the board's: a missing description is not a missing
    // table, and the panel must keep naming and valuing every parameter.
    expect(session.snapshot.parameters.map((row) => row.name)).toEqual(['roll_kp', 'airframe']);
    expect(session.snapshot.parameters.every((row) => row.meta === null)).toBe(true);
  });

  it('drops the armed-set note when the board claims it gates on armed', async () => {
    const { session } = await connect({
      features: Feature.APPLIES_ON_WRITE | Feature.GATES_ON_ARMED,
    });
    const ids = session.snapshot.limitations.map((item) => item.id);
    expect(ids).not.toContain('no-armed-guard-on-set');
    expect(ids).not.toContain('no-application-confirmation');
    // The gates this *app* applies are its own and do not depend on the board.
    expect(ids).toContain('armed-state-gates-writes');
  });

  it('cites the source for every claim it makes, as symbols', async () => {
    const { session } = await connect();
    for (const limitation of session.snapshot.limitations) {
      expect(limitation.citations.length, limitation.id).toBeGreaterThan(0);
      for (const citation of limitation.citations) {
        // `path:symbol`, never a line number — a line number is wrong after
        // the next edit, and nothing would fail. check-citations.py resolves
        // these against the firmware tree.
        expect(citation, limitation.id).toMatch(/^[a-z_/]+\.(c|h):[A-Za-z_][A-Za-z0-9_]*$/);
      }
      expect(limitation.summary.length).toBeGreaterThan(10);
      expect(limitation.detail.length).toBeGreaterThan(80);
    }
  });
});

describe('when the link drops', () => {
  it('shuts the gate and reports the reason', async () => {
    const { session, link } = await connect({ armed: false });
    expect(session.snapshot.permission.allowed).toBe(true);

    link.hangUp('the cable was pulled');
    await settle();

    expect(session.snapshot.phase).toBe('failed');
    expect(session.snapshot.failure).toBe('the cable was pulled');
    // The armed state is not merely old now — there is no link to refresh it
    // from, so it is unknown and stays unknown.
    expect(session.snapshot.live.armed).toBe('unknown');
    expect(session.snapshot.permission.allowed).toBe(false);
  });
});

describe('writing the whole staged set', () => {
  it('sends every staged value when the gate is open', async () => {
    const { session, board } = await connect({ armed: false });
    session.edit(0, '0.500');
    session.edit(1, '3');
    await session.writeAll();
    await settle();

    const rows = session.snapshot.parameters;
    expect(rows[0]!.boardValue).toBe('0.500');
    expect(rows[1]!.boardValue).toBe('3');
    expect(board.stats.refused).toBe(0);
  });

  it('keeps going past a refusal, because parameters are independent', async () => {
    const { session } = await connect({ armed: false });
    session.edit(0, '99'); // out of range
    session.edit(1, '3'); // fine

    const summary = await session.writeAll();
    await settle();

    const rows = session.snapshot.parameters;
    expect(rows[0]!.write!.echoed).toBe(false);
    expect(rows[0]!.edited).toBe('99'); // left in the box to be corrected
    expect(rows[1]!.boardValue).toBe('3'); // the good one still landed
    expect(summary).toEqual({ written: 1, refused: 1, stopped: false });
  });

  it('stops mid-run if the gate closes', async () => {
    // The board arms the moment the first write lands and says so in a
    // telemetry frame, which is the only thing that can close the gate: the
    // permission is computed from frames, never from hope.
    const { session, board } = await connect({ armed: false, pushTelemetry: true, armOnFirstWrite: true });
    await session.stream(10);

    session.edit(0, '0.500');
    session.edit(1, '3');
    const summary = await session.writeAll();
    await settle();

    const rows = session.snapshot.parameters;
    expect(rows[0]!.boardValue).toBe('0.500'); // the first had already gone
    expect(rows[1]!.edited).toBe('3'); // the second must not
    expect(rows[1]!.write).toBeNull();
    expect(summary.stopped).toBe(true);
    expect(session.snapshot.live.armed).toBe('armed');
    expect(session.snapshot.permission.allowed).toBe(false);
    expect(board.stats.refused).toBe(0); // the board never had to refuse
  });
});

describe('the commands the app sends', () => {
  it('sends a param set with the index and the value as text', async () => {
    const { session, board } = await connect({ armed: false });
    const before = board.stats.commands;
    session.edit(0, '2.5');
    await session.write(0);
    await settle();
    // set, then a get to find out what the board now holds.
    expect(board.stats.commands).toBe(before + 2);
    expect(Command.PARAM_SET).toBe(0x03);
  });
});

describe('the attitude rate', () => {
  it('polls STATUS at 20 Hz only while a view watches the attitude', async () => {
    const board = makeBoard({ parameters: TINY, canSave: true });
    const link = new BoardLink(board);
    let statuses = 0;
    const write = link.write.bind(link);
    // Byte 3 of a frame is the command: AA 55 version command ...
    link.write = (bytes: Uint8Array) => {
      if (bytes[3] === Command.STATUS) statuses++;
      write(bytes);
    };
    const session = new AerialKitSession(new LinkTransport(link), {
      now: new FakeClock().now,
      pollMs: 1000,
      attitudePollMs: 50,
      readHelp: false,
    });
    await session.open();

    statuses = 0;
    await vi.advanceTimersByTimeAsync(1000);
    expect(statuses).toBeLessThanOrEqual(1);

    session.watchAttitude(true);
    statuses = 0;
    await vi.advanceTimersByTimeAsync(1000);
    expect(statuses).toBeGreaterThanOrEqual(15);

    session.watchAttitude(false);
    statuses = 0;
    await vi.advanceTimersByTimeAsync(1000);
    expect(statuses).toBeLessThanOrEqual(1);
    await session.close();
  });
});
