import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { RC_MAX_CHANNELS, RcFlag, RcStatus, RcSwitch } from '../src/protocol/constants';
import { parseRcChannels, ProtocolError } from '../src/protocol/messages';
import { AerialKitClient } from '../src/protocol/client';
import { AerialKitSession } from '../src/session/aerialkit';
import { BoardLink, FakeClock, LinkTransport, makeBoard, settle } from './helpers';
import type { DemoBoard } from '../src/transport/demo-board';

/**
 * The receiver, from the wire up.
 *
 * Three replies that look alike and mean different things, and keeping them
 * apart is what this file is for:
 *
 *  - `AK_PROTO_RC_NONE` — one byte. This board has no receiver port. A fact
 *    about the hardware.
 *  - A full frame with the link flag clear and the sticks at zero — a receiver
 *    port with a receiver on it that has never framed. A wiring or binding
 *    fault.
 *  - A full frame with `DECODED` set and the sticks at zero — a handset sitting
 *    centred. Neither fault.
 *
 * And a fourth that is not a receiver fact at all: `0x7F`, the status this
 * firmware gives a command it does not implement. A build that predates
 * `rc channels` must not be reported to a person as a board with no receiver.
 */

const STALE_AFTER = 2000;
const RC_POLL_MS = 100;

interface Built {
  session: AerialKitSession;
  board: DemoBoard;
  link: BoardLink;
  clock: FakeClock;
}

async function connect(
  options: {
    features?: number | null;
    receiver?: 'crsf' | 'sbus' | null;
  } = {},
): Promise<Built> {
  const clock = new FakeClock();
  const board = makeBoard({
    ...(options.features === undefined ? {} : { features: options.features }),
    ...(options.receiver === undefined ? {} : { receiver: options.receiver }),
  });
  const link = new BoardLink(board);
  const session = new AerialKitSession(new LinkTransport(link), {
    now: clock.now,
    staleAfterMs: STALE_AFTER,
    // The status poll never fires on its own here. The rc poll does, at the
    // interval the shipped default uses, and a test moves it with
    // `advanceTimersByTime` — the interval is the *page's* timer, not this
    // board's clock, so the two are advanced by different calls.
    pollMs: 1_000_000,
    rcPollMs: RC_POLL_MS,
  });
  await session.open();
  await session.refresh();
  return { session, board, link, clock };
}

/** The demo board counts receiver frames against the wall clock, which under
 *  `vi.useFakeTimers()` is the fake one. Moving it is how a test gives this
 *  board a receiver that has actually been running. */
function runBoardFor(ms: number): void {
  vi.setSystemTime(Date.now() + ms);
}

let clients: AerialKitClient[] = [];

function clientOn(board: DemoBoard): AerialKitClient {
  const client = new AerialKitClient(new BoardLink(board));
  clients.push(client);
  return client;
}

beforeEach(() => {
  vi.useFakeTimers();
});

afterEach(() => {
  for (const client of clients) client.close();
  clients = [];
  vi.useRealTimers();
});

/** One `rc channels` payload, written by hand. A hand-built frame is the point:
 *  the parser's job is field offsets, and a payload produced by the same code
 *  under test would agree with it about a wrong offset. */
function payload(options: {
  status?: number;
  flags?: number;
  protocol?: number;
  channels: readonly number[];
  sticks: readonly number[];
  switches?: number;
  counters?: readonly number[];
}): Uint8Array {
  const counters = options.counters ?? [0, 0, 0, 0, 0, 0, 0];
  const out = new Uint8Array(4 + options.channels.length * 2 + 8 + 1 + 28);
  let at = 0;
  out[at++] = options.status ?? RcStatus.OK;
  out[at++] = options.flags ?? 0;
  out[at++] = options.protocol ?? 0;
  out[at++] = options.channels.length;
  for (const count of options.channels) {
    out[at++] = count & 0xff;
    out[at++] = (count >> 8) & 0xff;
  }
  for (const value of options.sticks) {
    const signed = value < 0 ? value + 0x10000 : value;
    out[at++] = signed & 0xff;
    out[at++] = (signed >> 8) & 0xff;
  }
  out[at++] = options.switches ?? 0;
  for (const counter of counters) {
    out[at++] = counter & 0xff;
    out[at++] = (counter >> 8) & 0xff;
    out[at++] = (counter >> 16) & 0xff;
    out[at++] = (counter >> 24) & 0xff;
  }
  return out;
}

describe('the reply, field by field', () => {
  it('reads every field at the offset the wire puts it at', () => {
    const parsed = parseRcChannels(
      payload({
        flags: RcFlag.LINK | RcFlag.DECODED | RcFlag.TELEMETRY,
        protocol: 0,
        channels: [172, 992, 1811],
        sticks: [0, -1000, 250, 1000],
        switches: RcSwitch.ARM_ON,
        // Seven distinct values, so a counter read from its neighbour's bytes
        // is a failing assertion rather than a coincidence.
        counters: [1, 2, 3, 4, 5, 6, 7],
      }),
    );

    expect(parsed.status).toBe(RcStatus.OK);
    expect(parsed.protocolName).toBe('CRSF');
    expect(parsed.channels).toEqual([172, 992, 1811]);
    // Signed, and -1000 is the value that says so: written as a magnitude and
    // a direction it would survive as +1000, which is full stick the other way.
    expect(parsed.sticks).toEqual([0, -1000, 250, 1000]);
    expect(parsed.switches).toBe(RcSwitch.ARM_ON);
    expect([parsed.bytes, parsed.frames, parsed.crcErrors, parsed.rejected]).toEqual([1, 2, 3, 4]);
    expect([parsed.lost, parsed.failsafeFrames, parsed.dropped]).toEqual([5, 6, 7]);
  });

  it('names a protocol it knows, and says so rather than guessing at one it does not', () => {
    const known = parseRcChannels(payload({ protocol: 1, channels: [1], sticks: [0, 0, 0, 0] }));
    expect(known.protocolName).toBe('SBUS');

    // The number is kept as well as the name, so a protocol this app has never
    // heard of renders as "the board says 9" rather than as nothing at all —
    // the same rule the parameter groups follow.
    const unknown = parseRcChannels(payload({ protocol: 9, channels: [1], sticks: [0, 0, 0, 0] }));
    expect(unknown.protocolName).toBeNull();
    expect(unknown.protocol).toBe(9);
  });

  it('refuses a channel count past what the protocol allows', () => {
    // Reading on would take the sticks for channels, and the four stick values
    // for counters. A count the firmware cannot have written is a broken frame,
    // not a frame with more channels in it.
    const tooMany = payload({
      channels: new Array<number>(RC_MAX_CHANNELS + 1).fill(992),
      sticks: [0, 0, 0, 0],
    });
    expect(() => parseRcChannels(tooMany)).toThrow(/at most 16/);
  });
});

describe('no receiver, and a receiver that has never framed', () => {
  it('stops at the status byte when the board has no receiver port', () => {
    const parsed = parseRcChannels(new Uint8Array([RcStatus.NONE]));

    expect(parsed.status).toBe(RcStatus.NONE);
    // Every field is empty rather than zero, because zero is a reading: a
    // receiver with zero frames and zero channels is a receiver that is not
    // working, and this board has nowhere to plug one in.
    expect(parsed.channels).toEqual([]);
    expect(parsed.sticks).toEqual([]);
    expect(parsed.flags).toBe(0);
    expect(parsed.protocolName).toBeNull();
    expect(parsed.frames).toBe(0);
  });

  it('gets that answer from the demo board when it is configured without one', async () => {
    const { session } = await connect({ receiver: null });
    await session.refreshRc();
    expect(session.snapshot.rc.state?.status).toBe(RcStatus.NONE);
  });

  it('reports a receiver that has stopped framing as a frame, not as no receiver', async () => {
    const { session, board } = await connect({ receiver: 'crsf' });
    runBoardFor(1_000);
    await session.refreshRc();
    const live = session.snapshot.rc.state!;
    expect(live.status).toBe(RcStatus.OK);
    expect(live.frames).toBeGreaterThan(0);
    expect(live.flags & RcFlag.DECODED).not.toBe(0);

    board.setReceiverAlive(false);
    await session.refreshRc();
    const lost = session.snapshot.rc.state!;

    // Still a full frame — so still `OK` — and the two facts that say what
    // happened are the flags, not the numbers.
    expect(lost.status).toBe(RcStatus.OK);
    expect(lost.channels.length).toBeGreaterThan(0);
    expect(lost.flags & RcFlag.LINK).toBe(0);
    expect(lost.flags & RcFlag.DECODED).toBe(0);
    // Zeroed, and *only* the flag makes these zeroes mean "no idea" rather than
    // "centred". Same bytes, different meaning, which is the whole reason the
    // bit exists.
    expect(lost.sticks).toEqual([0, 0, 0, 0]);
    // The counters are the ones it had reached, frozen rather than reset: a
    // receiver that stopped framing did not stop having framed.
    expect(lost.frames).toBeGreaterThan(0);
  });
});

describe('an old board', () => {
  it('answers 0x7F, and that is told apart from a board with no receiver', async () => {
    // `features: null` is a board whose hello ends before the capability word.
    // It answers `rc channels` the way the firmware answers any command it does
    // not have: correlated, with one byte, 0x7F.
    const client = clientOn(makeBoard({ features: null }));
    await expect(client.rcChannels()).rejects.toThrow(ProtocolError);
    await expect(client.rcChannels()).rejects.toThrow(/predates `rc channels`/);
  });

  it('surfaces it as an error, not as a reading about hardware', async () => {
    const { session } = await connect({ features: null });
    await session.refreshRc();

    const rc = session.snapshot.rc;
    // Null, not a `NONE` state. A board that does not implement the command has
    // said nothing about whether it has a receiver, and reporting that byte as
    // "no receiver input" would be this app turning "I do not know" into a
    // hardware fact — with a receiver possibly plugged in at that moment.
    expect(rc.state).toBeNull();
    expect(rc.error).toMatch(/0x7F/);
    expect(rc.failed).toBe(1);
  });
});

describe('the session polls it, and only while someone is looking', () => {
  it('asks immediately when watching starts, then on the interval', async () => {
    const { session } = await connect({ receiver: 'crsf' });
    expect(session.snapshot.rc.watching).toBe(false);
    expect(session.snapshot.rc.polls).toBe(0);

    session.watchRc(true);
    expect(session.snapshot.rc.watching).toBe(true);
    // The first ask is immediate, so a tab opening onto a blank chart fills on
    // the first round trip rather than after a poll interval.
    await settle();
    expect(session.snapshot.rc.polls).toBe(1);
    expect(session.snapshot.rc.state).not.toBeNull();

    vi.advanceTimersByTime(RC_POLL_MS);
    await settle();
    expect(session.snapshot.rc.polls).toBe(2);

    vi.advanceTimersByTime(RC_POLL_MS * 3);
    await settle();
    expect(session.snapshot.rc.polls).toBe(5);
  });

  it('stops when watching stops, and keeps the last reading', async () => {
    const { session } = await connect({ receiver: 'crsf' });
    session.watchRc(true);
    await settle();
    const polls = session.snapshot.rc.polls;

    session.watchRc(false);
    expect(session.snapshot.rc.watching).toBe(false);
    vi.advanceTimersByTime(RC_POLL_MS * 10);
    await settle();
    // Not one more. The rc poll is a second exchange on a link that also
    // carries the parameter table, and a page that went on asking for a tab
    // nobody has open would make the parameter reads look slow for no visible
    // reason.
    expect(session.snapshot.rc.polls).toBe(polls);
    // The reading stays, with its age, rather than being blanked: what a person
    // sees when they come back is the last answer and how old it is.
    expect(session.snapshot.rc.state).not.toBeNull();
    expect(session.snapshot.rc.atMs).not.toBeNull();
  });

  it('is idempotent, so a view cannot double its own polling', async () => {
    const { session } = await connect({ receiver: 'crsf' });
    session.watchRc(true);
    session.watchRc(true);
    session.watchRc(true);
    await settle();
    vi.advanceTimersByTime(RC_POLL_MS);
    await settle();
    // One immediate ask plus one interval. Three overlapping intervals would
    // make this 4 or more.
    expect(session.snapshot.rc.polls).toBe(2);
  });

  it('clears the reading when the link closes, and says why', async () => {
    const { session, link } = await connect({ receiver: 'crsf' });
    session.watchRc(true);
    await settle();
    expect(session.snapshot.rc.state).not.toBeNull();

    link.hangUp('the cable was pulled');
    await settle();
    // A channel list from a closed port is a reading of a receiver this app is
    // no longer hearing. A bar chart is read as *now* in a way a label is not.
    expect(session.snapshot.rc.state).toBeNull();
    expect(session.snapshot.rc.watching).toBe(false);
    expect(session.snapshot.rc.error).toMatch(/cable was pulled/);
  });

  it('counts failed polls, and writes only the first of a run to the log', async () => {
    // A board that answers `rc channels` with 0x7F: the link stays up and every
    // poll fails, which is the shape an unreachable or too-old board has. The
    // reading this checks is the *counting*, not a receiver fact.
    const { session } = await connect({ features: null });
    session.watchRc(true);
    await settle();
    const logged = session.snapshot.events.length;
    expect(session.snapshot.rc.failed).toBe(1);

    vi.advanceTimersByTime(RC_POLL_MS * 5);
    await settle();
    // Every failed poll is counted, so a link dropping every other exchange is
    // visible rather than merely slow.
    expect(session.snapshot.rc.failed).toBeGreaterThan(1);
    // But only the first of the run is written down: at ten polls a second an
    // unreachable board would otherwise bury the log it is written into.
    expect(session.snapshot.events.length).toBe(logged);
  });
});
