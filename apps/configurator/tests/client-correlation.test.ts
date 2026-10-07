import { afterEach, describe, expect, it, vi } from 'vitest';

import { AerialKitClient } from '../src/protocol/client';
import { crc16 } from '../src/protocol/crc16';

/*
 * Replies carry no sequence number, only the command byte - so a reply that
 * arrives just after its request timed out must not be read as the answer to
 * the next request. Found by the 2026-10-06 audit: a `param set` the board
 * refused was recorded as accepted, because the previous set's late "OK"
 * answered it, and an unrelated request was rejected with a correlation error.
 *
 * The client's answer is a quiet window after a timeout (a fifth of it, at
 * most 250 ms) in which nothing is sent and a reply to the timed-out command
 * is dropped. A reply later than that is indistinguishable from the answer to
 * the next request of the same command - the protocol has no sequence number -
 * which is a limit of the wire, not of this code.
 */

function reply(command: number, payload: number[]): Uint8Array {
  const body = new Uint8Array([1, command | 0x80, payload.length, ...payload]);
  const c = crc16(body);
  return new Uint8Array([0xaa, 0x55, ...body, c & 0xff, c >> 8]);
}

class Link {
  private readonly handlers = new Set<(chunk: Uint8Array) => void>();
  readonly writes: Uint8Array[] = [];
  write(bytes: Uint8Array): void {
    this.writes.push(bytes);
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
  deliver(bytes: Uint8Array): void {
    for (const handler of this.handlers) handler(bytes);
  }
}

afterEach(() => {
  vi.useRealTimers();
});

describe('a reply that arrives after its request timed out', () => {
  it('is dropped, and does not answer the next request', async () => {
    vi.useFakeTimers();
    const link = new Link();
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const client = new AerialKitClient(link as any, { timeoutMs: 100 });
    const first = client.paramSet(10, '0.5').catch((error: Error) => error);
    // Past the timeout (100 ms) and inside the quiet that follows it (20 ms).
    await vi.advanceTimersByTimeAsync(105);
    expect(((await first) as Error).name).toBe("TimeoutError");

    const second = client.paramSet(11, '9999');
    await Promise.resolve();
    // The first set's late "OK" arrives while the second is still queued.
    link.deliver(reply(0x03, [0, 0]));
    await vi.advanceTimersByTimeAsync(50);
    expect(link.writes).toHaveLength(2); // the second request went out after the quiet
    // And the board's own answer to the second: refused.
    link.deliver(reply(0x03, [2, ...new TextEncoder().encode('out of range'), 0]));
    expect((await second).status).toBe(2);
  });

  it('does not reject an unrelated request either', async () => {
    vi.useFakeTimers();
    const link = new Link();
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const client = new AerialKitClient(link as any, { timeoutMs: 100 });
    const first = client.status().catch((error: Error) => error);
    await vi.advanceTimersByTimeAsync(105);
    expect(((await first) as Error).name).toBe("TimeoutError");

    const save = client.paramSave();
    await Promise.resolve();
    link.deliver(reply(0x05, new Array(22).fill(0))); // the status, late
    await vi.advanceTimersByTimeAsync(50);
    link.deliver(reply(0x04, [0]));
    expect((await save).status).toBe(0);
  });
});

describe('a request that cannot be framed', () => {
  it('fails at once and does not hold up the ones behind it', async () => {
    const link = new Link();
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const client = new AerialKitClient(link as any, { timeoutMs: 2000 });
    const tooLong = client.paramSet(0, 'x'.repeat(200));
    await expect(tooLong).rejects.toBeInstanceOf(RangeError);
    const status = client.status();
    await Promise.resolve();
    expect(link.writes).toHaveLength(1); // the status went straight out
    link.deliver(reply(0x05, new Array(22).fill(0)));
    await expect(status).resolves.toBeDefined();
  });
});
