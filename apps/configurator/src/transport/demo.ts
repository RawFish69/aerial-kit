import table from '../firmware/demo-table.json';
import { DemoBoard, type DemoParameter } from './demo-board';
import type { Transport, TransportInfo } from './types';

export interface DemoOptions {
  /** Override the table. Tests use a short one. */
  readonly parameters?: readonly DemoParameter[];
  /** How often the board is given the chance to push telemetry, in ms. Working
   *  at 60 Hz means a 50 Hz stream is not visibly quantised to the poll. */
  readonly pollMs?: number;
}

/**
 * The demo board's starting table.
 *
 * This file is a *fixture*, not a board's metadata, and the distinction is the
 * whole of milestone 3. These rows are handed to `DemoBoard`, which then serves
 * them over `param info` and `param help` like any other board — so what the app
 * renders came off the wire either way. Nothing here is ever attached to a board
 * that did not send it, which is what the previous design did and how a 32-row
 * file sat beside a 92-parameter board without anything failing.
 *
 * The names, groups, ranges, decimal places and boot values are real; the
 * product string the board answers with is not, and that is the safeguard.
 */
export function demoParameters(): DemoParameter[] {
  return (table.parameters as DemoParameter[]).map((item) => ({
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

/**
 * The demo board, as a transport.
 *
 * Bytes written go into the board and the answer comes back on the next tick,
 * so the client above it cannot tell this from a cable. The one difference it
 * can see is in the identity the board reports, which is the point.
 */
export class DemoTransport implements Transport {
  readonly info: TransportInfo = {
    kind: 'demo',
    label: 'Demo board',
    detail: 'a simulated board inside this page — no hardware, no cable, nothing saved to disk',
  };

  readonly board: DemoBoard;
  private readonly pollMs: number;
  private timer: ReturnType<typeof setInterval> | null = null;
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();
  private open_ = false;

  constructor(options: DemoOptions = {}) {
    this.board = new DemoBoard({
      parameters: options.parameters ?? demoParameters(),
      product: table.demo_product,
    });
    this.pollMs = options.pollMs ?? 16;
  }

  get isOpen(): boolean {
    return this.open_;
  }

  async open(): Promise<void> {
    if (this.open_) return;
    this.open_ = true;
    this.timer = setInterval(() => {
      const frame = this.board.poll(Date.now());
      if (frame !== null) this.emit(frame);
    }, this.pollMs);
  }

  private emit(bytes: Uint8Array): void {
    for (const handler of this.dataHandlers) handler(bytes);
  }

  async write(bytes: Uint8Array): Promise<void> {
    if (!this.open_) {
      for (const handler of this.errorHandlers) handler('the demo board is not open');
      return;
    }
    // The write is answered on a later tick rather than synchronously, so the
    // client's timeout, queue and correlation paths are exercised here exactly
    // as they are over a cable. A demo that answered instantly would hide every
    // bug that lives in the gap.
    await Promise.resolve();
    const reply = this.board.feed(bytes);
    if (reply !== null) this.emit(reply);
  }

  async close(): Promise<void> {
    if (!this.open_) return;
    this.open_ = false;
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
    for (const handler of this.closeHandlers) handler('the demo board was closed');
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
