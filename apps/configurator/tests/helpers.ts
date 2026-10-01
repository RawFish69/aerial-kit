import type { ByteLink } from '../src/protocol/client';
import table from '../src/firmware/demo-table.json';
import { DemoBoard, type DemoBoardOptions, type DemoParameter } from '../src/transport/demo-board';
import type { Transport } from '../src/transport/types';

/**
 * A link to a demo board with no timers in it.
 *
 * The session under test is driven to a known state rather than waiting for
 * one, so a test says what it is checking instead of how long it waited. The
 * reply is delivered on a microtask rather than synchronously: a write that
 * answers inside its own `write()` call would never exercise the client's
 * queue, its timeout or its correlation, and those are where the bugs are.
 */
export class BoardLink implements ByteLink {
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();
  closed = false;

  constructor(
    readonly board: DemoBoard,
    /** Whether the board is ever given the chance to push telemetry. False
     *  models the host simulator, which agrees to a rate and then sends
     *  nothing because `akproto_sim.c` never builds a telemetry frame. */
    private readonly pushTelemetry = false,
    /**
     * Arms the board the moment the first write lands, and pushes a telemetry
     * frame saying so on the next turn.
     *
     * This is how "the aircraft armed while you were writing" is tested without
     * a race: the gate is closed by a *frame*, so a test that wants it closed
     * has to produce one, at a moment it chooses.
     */
    private readonly armOnFirstWrite = false,
  ) {}

  private armedByWrite = false;

  write(bytes: Uint8Array): void {
    if (this.closed) return;
    const reply = this.board.feed(bytes);
    queueMicrotask(() => {
      if (this.closed || reply === null) return;
      for (const handler of this.dataHandlers) handler(reply);
      // A reply to PARAM_SET carries the response bit: 0x03 | 0x80.
      if (this.armOnFirstWrite && !this.armedByWrite && reply[3] === (0x03 | 0x80)) {
        this.armedByWrite = true;
        this.board.setArmed(true);
        const frame = this.board.poll(Date.now());
        if (frame === null) return;
        queueMicrotask(() => {
          if (this.closed) return;
          for (const handler of this.dataHandlers) handler(frame);
        });
      }
    });
  }

  /** One telemetry frame, if the board has one due. Only does anything when the
   *  link was built with `pushTelemetry`. */
  tick(nowMs: number): void {
    if (!this.pushTelemetry || this.closed) return;
    const frame = this.board.poll(nowMs);
    if (frame === null) return;
    queueMicrotask(() => {
      if (this.closed) return;
      for (const handler of this.dataHandlers) handler(frame);
    });
  }

  hangUp(reason = 'the cable was pulled'): void {
    this.closed = true;
    for (const handler of this.closeHandlers) handler(reason);
  }

  /** Put bytes on the wire that the client did not ask for. Used to check that
   *  an unsolicited reply is counted and dropped rather than mistaken for the
   *  answer to whatever is outstanding. */
  inject(chunk: Uint8Array): void {
    if (this.closed) return;
    queueMicrotask(() => {
      if (this.closed) return;
      for (const handler of this.dataHandlers) handler(chunk);
    });
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
}

/**
 * A link where the test decides what, if anything, comes back.
 *
 * `BoardLink` answers, which is right for the session tests and wrong for the
 * ones about a board that is silent, slow or answering the wrong question.
 * Here nothing is sent until the test says so, and the bytes that were written
 * are kept so a test can assert on what went out.
 */
export class ManualLink implements ByteLink {
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();
  readonly written: Uint8Array[] = [];
  closed = false;

  write(bytes: Uint8Array): void {
    this.written.push(bytes);
  }

  /** Hand a frame to the client as if it had just arrived. */
  deliver(frame: Uint8Array): void {
    queueMicrotask(() => {
      if (this.closed) return;
      for (const handler of this.dataHandlers) handler(frame);
    });
  }

  /** The far end goes away — unplugged, or the process behind it died. */
  hangUp(reason = 'the cable was pulled'): void {
    this.closed = true;
    for (const handler of this.closeHandlers) handler(reason);
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
}

/** A `BoardLink` dressed as a transport, so a session can be pointed at one. */
export class LinkTransport implements Transport {
  readonly info = { kind: 'demo' as const, label: 'test board', detail: 'a board in this process' };
  private open_ = false;

  constructor(readonly link: BoardLink) {}

  get isOpen(): boolean {
    return this.open_;
  }

  async open(): Promise<void> {
    this.open_ = true;
  }

  async close(): Promise<void> {
    this.open_ = false;
    this.link.hangUp('closed by the test');
  }

  write(bytes: Uint8Array): void {
    this.link.write(bytes);
  }

  onData(handler: (chunk: Uint8Array) => void): () => void {
    return this.link.onData(handler);
  }

  onClose(handler: (reason: string) => void): () => void {
    return this.link.onClose(handler);
  }

  onError(handler: (message: string) => void): () => void {
    return this.link.onError(handler);
  }
}

/** The demo table as the demo board's parameters.
 *
 * The same rows `demoParameters()` hands the shipped demo board, mapped the same
 * way — including `group`, which is what the group headings on the Parameters
 * panel are drawn from. A helper that dropped it would give a test table that
 * renders under one "not described" heading and pass anyway, which is the shape
 * of the bug this milestone is about. */
export function realParameters(): DemoParameter[] {
  return (table.parameters as Array<DemoParameter & { default?: string }>).map((item) => ({
    name: item.name,
    value: item.default ?? item.value ?? '0',
    help: item.help,
    type: item.type,
    decimals: item.decimals,
    min: item.min,
    max: item.max,
    group: item.group,
  }));
}

/** A short table, for tests that are about behaviour rather than the firmware. */
export const TINY: DemoParameter[] = [
  { name: 'roll_kp', value: '0.250', default: '0.250', help: 'rate loop P, roll', type: 'float', decimals: 3, min: 0, max: 3, group: 1 },
  { name: 'airframe', value: '0', default: '0', help: 'which mixer', type: 'u32', decimals: 0, min: 0, max: 6, group: 5 },
];

export function makeBoard(options: DemoBoardOptions = {}): DemoBoard {
  return new DemoBoard({ parameters: realParameters(), product: table.demo_product, ...options });
}

/** Let every queued microtask run. One `await` is not enough: the client
 *  resolves, then the session publishes, then the view would render. */
export async function settle(turns = 8): Promise<void> {
  for (let i = 0; i < turns; i++) await Promise.resolve();
}

/** A clock a test can move, so "stale" is decided rather than waited for. */
export class FakeClock {
  constructor(private ms = 1_000_000) {}
  now = (): number => this.ms;
  advance(delta: number): void {
    this.ms += delta;
  }
}
