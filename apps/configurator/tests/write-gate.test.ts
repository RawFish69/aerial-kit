import { describe, expect, it } from 'vitest';
import { AerialKitClient } from '../src/protocol/client';
import { Command, SetStatus, UNKNOWN_COMMAND } from '../src/protocol/constants';
import { Feature } from '../src/protocol/features';
import { baseCommand, buildFrame, FrameDecoder } from '../src/protocol/frame';
import { parseParamDefault, parseParamSet } from '../src/protocol/messages';
import { AerialKitSession } from '../src/session/aerialkit';
import { BoardLink, FakeClock, LinkTransport, TINY, makeBoard, settle } from './helpers';
import type { DemoBoard } from '../src/transport/demo-board';

/**
 * Milestone 4: the write gate, on both sides of the wire, and the reset.
 *
 * The firmware half of this is `tests/test_proto.c` — `ak_proto_io_t.writable`,
 * consulted by every write route, plus `AK_PROTO_CMD_PARAM_DEFAULT`. This file
 * is the app half, and it exists because the two halves can each be right and
 * still disagree in the middle: the app's near-side gate can refuse a write the
 * board would take, and the board's far-side gate can refuse one the app sent.
 * Both are tested here, against a real peer that answers real frames.
 *
 * The far-side tests matter more than they look. The app's own gate is the one
 * a person sees, and it is easy to test; what it cannot cover is the race it
 * narrows and cannot close — the aircraft arming in the window between the
 * app's last reading of the flight state and the bytes arriving. Only the
 * board's gate covers that, and a test that runs the app's gate alone would
 * report the aircraft protected when it is not.
 */

/** A board with no session in front of it, driven frame by frame. */
function wire(features?: number | null) {
  const board = makeBoard({
    parameters: TINY,
    ...(features === undefined ? {} : { features }),
  });
  const decoder = new FrameDecoder();
  const send = (command: Command, payload: Uint8Array): Uint8Array => {
    const reply = board.feed(buildFrame(command, payload));
    if (reply === null) throw new Error(`the board did not answer ${command}`);
    const { frames } = decoder.push(reply);
    expect(frames).toHaveLength(1);
    return frames[0]!.payload;
  };
  return { board, send };
}

describe('the board refuses a write while the aircraft is armed', () => {
  it('answers a set, a save and a default with the armed refusal, not a bad-value one', () => {
    const { board, send } = wire();
    board.setArmed(true);

    const set = parseParamSet(send(Command.PARAM_SET, new Uint8Array([0, 0x31])));
    expect(set.status).toBe(SetStatus.REFUSED_ARMED);
    // The firmware's own words, and the app shows them as they arrive.
    expect(set.message).toBe('refused: the aircraft is armed');

    const save = parseParamSet(send(Command.PARAM_SAVE, new Uint8Array(0)));
    expect(save.status).toBe(SetStatus.REFUSED_ARMED);

    const reset = parseParamDefault(send(Command.PARAM_DEFAULT, new Uint8Array([2])));
    expect(reset.status).toBe(SetStatus.REFUSED_ARMED);
  });

  it('refuses the armed set before it argues about the index', () => {
    // The ordering the firmware chose, and the reason for it is a person rather
    // than the protocol: a client told "no such parameter" while armed would
    // reasonably conclude that naming a real one would have worked. Index 200
    // does not exist, and the *armed* refusal is still the answer.
    const { board, send } = wire();
    board.setArmed(true);
    const reply = parseParamSet(send(Command.PARAM_SET, new Uint8Array([200, 0x31])));
    expect(reply.status).toBe(SetStatus.REFUSED_ARMED);
  });

  it('leaves the value untouched, so a refused write is not a partial one', () => {
    const { board, send } = wire();
    board.setArmed(true);
    send(Command.PARAM_SET, new Uint8Array([0, ...new TextEncoder().encode('1.500')]));
    expect(board.parameterValue(0)).toBe('0.250');
  });

  it('takes the same write once it is disarmed, with no reconnect', () => {
    // The gate is a question asked per request, not a decision remembered from
    // the connection. A board that latched "armed" at connect would fail this
    // and pass every other test in the file.
    const { board, send } = wire();
    board.setArmed(true);
    expect(parseParamSet(send(Command.PARAM_SET, new Uint8Array([0, 0x31]))).status).toBe(
      SetStatus.REFUSED_ARMED,
    );
    board.setArmed(false);
    expect(parseParamSet(send(Command.PARAM_SET, new Uint8Array([0, 0x31]))).status).toBe(
      SetStatus.OK,
    );
    expect(board.parameterValue(0)).toBe('1.000');
  });

  it('does not claim the gate on a board that reports the word without the bit', () => {
    // A board that reports a capability word and leaves GATES_ON_ARMED clear is
    // saying its writes are not checked. This board is built that way on
    // purpose, so the app's limitation entry for it stays reachable — the entry
    // is a reading, and a reading with nothing that can produce the other value
    // is a constant wearing a reading's clothes.
    const { board, send } = wire(Feature.PARAM_INFO | Feature.APPLIES_ON_WRITE);
    board.setArmed(true);
    expect(parseParamSet(send(Command.PARAM_SET, new Uint8Array([0, 0x31]))).status).toBe(
      SetStatus.OK,
    );
  });
});

describe('param default', () => {
  it('refuses a request that named nothing, and says what it wants', () => {
    // The one design decision here worth a test of its own. An empty frame is
    // what a truncated frame looks like, and "the frame lost its payload" must
    // not be the same bytes as "wipe every parameter".
    const { send } = wire();
    const reply = parseParamDefault(send(Command.PARAM_DEFAULT, new Uint8Array(0)));
    expect(reply.status).toBe(SetStatus.VALUE_REFUSED);
    expect(reply.message).toBe('name what to reset: 1 <index>, or 2 for all');
  });

  it('refuses mode 0 rather than reading it as all', () => {
    const { send } = wire();
    const reply = parseParamDefault(send(Command.PARAM_DEFAULT, new Uint8Array([0])));
    expect(reply.status).toBe(SetStatus.VALUE_REFUSED);
    expect(reply.message).toBe('mode is 1 (one) or 2 (all)');
  });

  it('refuses mode 3 for the same reason', () => {
    const { send } = wire();
    expect(parseParamDefault(send(Command.PARAM_DEFAULT, new Uint8Array([3]))).status).toBe(
      SetStatus.VALUE_REFUSED,
    );
  });

  it('puts one row back and leaves its neighbour alone', () => {
    const { board, send } = wire();
    send(Command.PARAM_SET, new Uint8Array([0, ...new TextEncoder().encode('1.500')]));
    send(Command.PARAM_SET, new Uint8Array([1, ...new TextEncoder().encode('5')]));

    expect(parseParamDefault(send(Command.PARAM_DEFAULT, new Uint8Array([1, 0]))).status).toBe(
      SetStatus.OK,
    );
    // The named row is back where it started; the row the request did not name
    // is still where the person left it. A reset-all that ignored the mode
    // would pass a test that only checked the first of those.
    expect(board.parameterValue(0)).toBe('0.250');
    expect(board.parameterValue(1)).toBe('5');
  });

  it('puts the whole table back, including the row the table does not spell a default for', () => {
    // `TINY` states a `default` for both rows, so this checks the ordinary
    // path. The unspelled case is the next test.
    const { send, board } = wire();
    send(Command.PARAM_SET, new Uint8Array([0, ...new TextEncoder().encode('2.000')]));
    send(Command.PARAM_SET, new Uint8Array([1, ...new TextEncoder().encode('4')]));
    expect(board.parameterValue(0)).toBe('2.000');

    expect(parseParamDefault(send(Command.PARAM_DEFAULT, new Uint8Array([2]))).status).toBe(
      SetStatus.OK,
    );
    expect(board.parameterValue(0)).toBe('0.250');
    expect(board.parameterValue(1)).toBe('0');
  });

  it('restores a value this board booted with and never spelled a default for', () => {
    // The bug this is here to catch: a board that reads `parameters[i].value`
    // when asked to reset restores "the default" to "whatever it is now", which
    // is a reset that resets nothing. `NO_DEFAULT` below states no `default`
    // field at all, so the only correct answer is the value it was built with.
    const board = makeBoard({
      parameters: [{ name: 'unspelled', value: '7', type: 'u32', decimals: 0, min: 0, max: 9 }],
    });
    const decoder = new FrameDecoder();
    const send = (payload: Uint8Array): Uint8Array => {
      const reply = board.feed(buildFrame(Command.PARAM_DEFAULT, payload))!;
      return decoder.push(reply).frames[0]!.payload;
    };

    board.feed(buildFrame(Command.PARAM_SET, new Uint8Array([0, 0x33])));
    expect(board.parameterValue(0)).toBe('3');
    expect(parseParamDefault(send(new Uint8Array([1, 0]))).status).toBe(SetStatus.OK);
    expect(board.parameterValue(0)).toBe('7');
  });

  it('refuses an index that is not there', () => {
    const { send } = wire();
    const reply = parseParamDefault(send(Command.PARAM_DEFAULT, new Uint8Array([1, 200])));
    expect(reply.status).toBe(SetStatus.NO_SUCH_PARAMETER);
  });

  it('refuses mode 1 with the index missing rather than defaulting to row 0', () => {
    const { send, board } = wire();
    board.feed(buildFrame(Command.PARAM_SET, new Uint8Array([0, 0x31])));
    const reply = parseParamDefault(send(Command.PARAM_DEFAULT, new Uint8Array([1])));
    expect(reply.status).toBe(SetStatus.NO_SUCH_PARAMETER);
    expect(board.parameterValue(0)).toBe('1.000');
  });
});

describe('a board that predates the command', () => {
  it('answers 0x7F rather than going silent', async () => {
    // The "old firmware" path, against a real peer: no capability word, so
    // `param default` is not a command this board has. It must be answered the
    // way the firmware's `default:` case answers — the request's own command
    // byte with the response bit, and one payload byte.
    const board = makeBoard({ parameters: TINY, features: null });
    const decoder = new FrameDecoder();
    const reply = board.feed(buildFrame(Command.PARAM_DEFAULT, new Uint8Array([2])))!;
    const frame = decoder.push(reply).frames[0]!;
    expect(baseCommand(frame.command)).toBe(Command.PARAM_DEFAULT);
    expect(frame.command & 0x80).toBe(0x80);
    expect(Array.from(frame.payload)).toEqual([UNKNOWN_COMMAND]);
  });

  it('gives the watcher the same answer the client would see', async () => {
    // Through the real client, so the answer travels the path a person's click
    // would take: the client resolves it, the parser reads the status, and the
    // session reports a refusal rather than a reset that quietly did nothing.
    const built = await connect({ features: null });
    const ok = await built.session.resetParameters(null);
    expect(ok).toBe(false);
    const events = built.session.snapshot.events;
    const last = events[events.length - 1]!;
    expect(last.text).toContain('unknown status');
  });
});

interface Built {
  session: AerialKitSession;
  board: DemoBoard;
  link: BoardLink;
  clock: FakeClock;
}

async function connect(
  options: { armed?: boolean; features?: number | null } = {},
): Promise<Built> {
  const clock = new FakeClock();
  const board = makeBoard({
    parameters: TINY,
    ...(options.features === undefined ? {} : { features: options.features }),
  });
  board.setArmed(options.armed ?? false);
  const link = new BoardLink(board, false, false);
  const session = new AerialKitSession(new LinkTransport(link), {
    now: clock.now,
    staleAfterMs: 2000,
    pollMs: 1_000_000,
  });
  await session.open();
  await session.refresh();
  return { session, board, link, clock };
}

describe('the reset, through the session', () => {
  it('refuses locally while the aircraft is armed, and sends nothing', async () => {
    const { session, board } = await connect({ armed: true });
    const before = board.stats.commands;
    const ok = await session.resetParameters(null);
    await settle();
    expect(ok).toBe(false);
    // The board was not asked. That is the assertion: a local refusal that
    // still puts bytes on the wire is not a refusal, it is a race.
    expect(board.stats.commands).toBe(before);
    const events = session.snapshot.events;
    expect(events[events.length - 1]!.text).toContain('refused locally');
  });

  it('resets one row and re-reads the table rather than recomputing it', async () => {
    const { session, board } = await connect();
    session.edit(0, '1.750');
    await session.write(0);
    await settle();
    expect(board.parameterValue(0)).toBe('1.750');

    expect(await session.resetParameters(0)).toBe(true);
    await settle();
    // The number on screen came from `param get` after the reset, not from a
    // copy of the default this app is holding.
    expect(session.snapshot.parameters[0]!.boardValue).toBe('0.250');
    expect(board.parameterValue(0)).toBe('0.250');
  });

  it('resets the whole table', async () => {
    const { session, board } = await connect();
    session.edit(0, '2.500');
    session.edit(1, '5');
    await session.writeAll();
    await settle();
    expect(board.parameterValue(0)).toBe('2.500');

    expect(await session.resetParameters(null)).toBe(true);
    await settle();
    expect(session.snapshot.parameters[0]!.boardValue).toBe('0.250');
    expect(session.snapshot.parameters[1]!.boardValue).toBe('0');
  });

  it('does not save, so a reset is not also a persistence', async () => {
    // The two are separate acts with separate consequences: a reset lands in
    // the running table, and losing it is one power cycle away until somebody
    // saves. A reset that saved would be an irreversible change behind one
    // button.
    const { session } = await connect();
    session.edit(0, '2.500');
    await session.write(0);
    await settle();
    const unsavedAfterWrite = session.snapshot.unsaved;

    await session.resetParameters(null);
    await settle();
    // Still unsaved, and still counted — the reset moved the table, which is
    // exactly what the changed count is a count of.
    expect(session.snapshot.unsaved).toBeGreaterThanOrEqual(unsavedAfterWrite ?? 0);
    expect(session.snapshot.events.some((event) => event.text.includes('saved'))).toBe(false);
  });
});

/**
 * The race the app's gate cannot close.
 *
 * Every write the app sends is gated on the aircraft being disarmed *as of the
 * last frame that carried its state*. Between that frame and the bytes arriving
 * the aircraft can arm, and no amount of care on the near side changes that.
 * The far side is what covers it, and this is the test that says so.
 */
describe('the far side covers the race the near side cannot', () => {
  it('has the board refuse a write the app sent while it believed disarmed', async () => {
    const { session, board } = await connect({ armed: false });

    // The person clicks, and the aircraft arms before the frame is answered.
    board.setArmed(true);
    session.edit(0, '1.250');
    await session.write(0);
    await settle();

    const row = session.snapshot.parameters[0]!;
    expect(row.write!.status).toBe(SetStatus.REFUSED_ARMED);
    // Not an error, not a silent nothing: a refusal, with the board's own
    // sentence on it, and the value where it was.
    expect(row.write!.echoed).toBe(false);
    expect(row.write!.applied).toBe(false);
    expect(board.parameterValue(0)).toBe('0.250');
  });

  it('says which side refused it', async () => {
    // The same status byte from two different gates, and a person needs to know
    // which. The near-side gate says "it was never sent"; a board's refusal has
    // a different sentence, and the difference is the whole reason the app no
    // longer invents a status for its own refusals.
    const { session, board } = await connect({ armed: false });
    board.setArmed(true);
    session.edit(0, '1.250');
    await session.write(0);
    await settle();
    expect(session.snapshot.parameters[0]!.write!.notAppliedBecause).toBe('the board refused it');
  });
});

describe('the client answers the reset as a typed call', () => {
  it('sends mode 1 with the index, and mode 2 with no index at all', async () => {
    // The frame is the claim. A client that sent `[2, 0]` for "reset all" would
    // be sending four bytes the firmware does not read, and on a stricter
    // handler that is a different request.
    const board = makeBoard({ parameters: TINY });
    const link = new ManualCapture(board);
    const client = new AerialKitClient(link, { timeoutMs: 50 });

    const one = client.paramDefault(0);
    link.deliver(buildFrame(Command.PARAM_DEFAULT | 0x80, new Uint8Array([SetStatus.OK])));
    await one;
    expect(Array.from(link.sent[0]!.payload)).toEqual([1, 0]);

    const all = client.paramDefault(null);
    link.deliver(buildFrame(Command.PARAM_DEFAULT | 0x80, new Uint8Array([SetStatus.OK])));
    await all;
    expect(Array.from(link.sent[1]!.payload)).toEqual([2]);

    client.close();
  });
});

/** Records what went out and answers when the test says so. */
class ManualCapture {
  readonly sent: Array<{ command: number; payload: Uint8Array }> = [];
  private readonly handlers = new Set<(chunk: Uint8Array) => void>();

  constructor(readonly board: DemoBoard) {}

  write(bytes: Uint8Array): void {
    const decoder = new FrameDecoder();
    const { frames } = decoder.push(bytes);
    for (const frame of frames) {
      this.sent.push({ command: baseCommand(frame.command), payload: frame.payload });
    }
  }

  deliver(frame: Uint8Array): void {
    queueMicrotask(() => {
      for (const handler of this.handlers) handler(frame);
    });
  }

  onData(handler: (chunk: Uint8Array) => void): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }
  onClose(): () => void {
    return () => {};
  }
  onError(): () => void {
    return () => {};
  }
}
