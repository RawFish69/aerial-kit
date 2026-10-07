import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { Command } from '../src/protocol/constants';
import {
  AerialKitClient,
  CorrelationError,
  LinkClosedError,
  TimeoutError,
  type ClientOptions,
} from '../src/protocol/client';
import { buildFrame } from '../src/protocol/frame';
import { BoardLink, ManualLink, makeBoard, realParameters, settle } from './helpers';

/**
 * The client, against a board that answers like the firmware.
 *
 * The behaviours worth pinning are the ones a real serial link produces and a
 * loopback never does: a reply that never comes, a reply to something else, a
 * link that dies mid-request, and a pushed frame arriving between a request and
 * its answer. A client that only ever met a perfect board would pass every test
 * and fail the first bench session.
 */

let clients: AerialKitClient[] = [];

function clientOn(link: BoardLink | ManualLink, options: ClientOptions = {}): AerialKitClient {
  const client = new AerialKitClient(link, options);
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

describe('requesting', () => {
  it('reads hello from a board that answers', async () => {
    const client = clientOn(new BoardLink(makeBoard()));
    const hello = await client.hello();
    expect(hello.product).toBe('aerialkit-demo');
    // The demo board's fixture table, which is the firmware's own table as
    // `capture-fixtures.py` read it off the wire. This number was 32 while the
    // fixture was a hand-copied snapshot missing `arm_accel_lpf_hz`, and nothing
    // failed — the file was internally consistent and every consumer joined it
    // to the board by name. `npm run verify:drift` is what catches it now; this
    // is the second, cruder alarm. Read from the fixture rather than written
    // down, so a table that grows does not leave this asserting the old one —
    // it was a literal 33 when the firmware's chain parameters made it 39.
    expect(hello.parameterCount).toBe(realParameters().length);
  });

  it('serialises requests, because the wire correlates by command byte alone', async () => {
    const client = clientOn(new BoardLink(makeBoard()));

    // Three at once. Two PARAM_GETs outstanding together would be
    // indistinguishable on this wire — the reply carries the command byte and
    // nothing else — so the client does not allow it. Each still gets its own
    // answer, which is what makes the queueing correct rather than merely safe.
    const [hello, first, second] = await Promise.all([
      client.hello(),
      client.paramGet(0),
      client.paramGet(1),
    ]);
    expect(hello.product).toBe('aerialkit-demo');
    expect(first!.index).toBe(0);
    expect(second!.index).toBe(1);
    expect(first!.name).not.toBe(second!.name);
  });

  it('counts what it has sent and received', async () => {
    const client = clientOn(new BoardLink(makeBoard()));
    await client.hello();
    await client.status();
    expect(client.stats.frames).toBe(2);
    expect(client.stats.writes).toBe(2);
    expect(client.stats.issues).toBe(0);
  });
});

describe('when the board does not answer', () => {
  it('gives up with a timeout naming the wait, and moves on to the next request', async () => {
    const link = new ManualLink();
    const client = clientOn(link, { timeoutMs: 200 });
    const caught = client.hello().catch((error: unknown) => error);
    await vi.advanceTimersByTimeAsync(250);

    const error = await caught;
    expect(error).toBeInstanceOf(TimeoutError);
    expect((error as Error).message).toContain('200 ms');
    expect(link.written).toHaveLength(1);

    // The queue is not wedged by the loss: the next request goes out.
    void client.status().catch(() => undefined);
    await settle();
    expect(link.written).toHaveLength(2);
  });

  it('refuses a reply to a different command rather than accepting it', async () => {
    // A board that answers the wrong thing is worse than one that is silent:
    // the bytes parse, and the numbers are real numbers, just not for this
    // question. Accepting them would put a plausible wrong value on screen.
    const link = new ManualLink();
    const client = clientOn(link, { timeoutMs: 10_000 });
    const caught = client.hello().catch((error: unknown) => error);
    await settle();

    link.deliver(buildFrame(Command.STATUS | 0x80, new Uint8Array(22)));
    await settle();

    expect(await caught).toBeInstanceOf(CorrelationError);
  });
});

describe('when the link drops', () => {
  it('rejects everything outstanding rather than leaving it hanging', async () => {
    const link = new BoardLink(makeBoard());
    const client = clientOn(link);
    await client.hello();

    link.hangUp('the cable was pulled');
    await settle();

    expect(client.isClosed).toBe(true);
    expect(client.closeReason).toBe('the cable was pulled');
    await expect(client.status()).rejects.toBeInstanceOf(LinkClosedError);
  });

  it('reports the drop once, to the client that asked', async () => {
    const onClosed = vi.fn();
    const link = new BoardLink(makeBoard());
    clientOn(link, { onClosed });
    link.hangUp('gone');
    link.hangUp('gone again');
    expect(onClosed).toHaveBeenCalledTimes(1);
  });

  it('rejects a request that was already in flight', async () => {
    const link = new ManualLink();
    const client = clientOn(link, { timeoutMs: 10_000 });
    const caught = client.hello().catch((error: unknown) => error);
    await settle();

    link.hangUp('unplugged mid-request');
    await settle();

    expect(await caught).toBeInstanceOf(LinkClosedError);
    expect(client.closeReason).toBe('unplugged mid-request');
  });
});

describe('telemetry', () => {
  it('routes a pushed frame to the telemetry handler, not to a pending request', async () => {
    const link = new BoardLink(makeBoard(), true);
    const seen: number[] = [];
    const client = clientOn(link, { onTelemetry: (frame) => seen.push(frame.uptimeMs) });

    await client.subscribe(10);
    await settle();
    link.tick(Date.now());
    await settle();

    expect(seen).toHaveLength(1);
    expect(client.stats.telemetry).toBe(1);
    // A push carries the command byte without the response bit. Had the client
    // matched on the bare byte it would have read this as the answer to
    // whatever was outstanding, and a 33-byte status body is not that.
    expect(client.stats.issues).toBe(0);
  });

  it('returns the rate the board agreed to, and zero is an answer', async () => {
    const client = clientOn(new BoardLink(makeBoard()));
    expect(await client.subscribe(10)).toBe(10);
    expect(await client.subscribe(0)).toBe(0);
    // Past the maximum, the board's answer is what counts, not the request.
    expect(await client.subscribe(999)).toBe(50);
  });

  it('drops an unasked-for reply and counts it rather than stalling', async () => {
    const issues: string[] = [];
    const link = new BoardLink(makeBoard());
    const client = clientOn(link, { onDecodeIssue: (text) => issues.push(text) });
    await client.hello();

    link.inject(buildFrame(Command.STATUS | 0x80, new Uint8Array(22)));
    await settle();

    expect(issues.some((text) => text.includes('unasked-for'))).toBe(true);
    expect(client.stats.issues).toBe(1);
    // And the client still works afterwards.
    await expect(client.status()).resolves.toBeDefined();
  });
});
