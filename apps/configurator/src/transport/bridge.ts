import { TransportError, type Transport, type TransportInfo } from './types';

/** The companion's default. Configurable, because a second board means a second
 *  bridge and there is no reason to make people rebuild the page for it. */
export const DEFAULT_BRIDGE_URL = 'ws://127.0.0.1:8787/ak';

export interface BridgeOptions {
  readonly url?: string;
  /** How long to wait for the socket to open before giving up. */
  readonly openTimeoutMs?: number;
}

/**
 * A companion process on the same machine, reached over a loopback WebSocket.
 *
 * A browser cannot open a TCP socket and cannot spawn a process, so the two
 * things a person most often wants — talk to the firmware's own simulator, or
 * to a board already claimed by something else — need a helper. That is all the
 * bridge is: bytes in, bytes out. It has no knowledge of the protocol and must
 * never grow any, because the moment it starts interpreting frames there are two
 * implementations of the protocol and one of them is always behind.
 *
 * The address defaults to loopback and the transport refuses anything else. A
 * bridge on another host would mean a page on the public internet holding an
 * unauthenticated byte pipe into someone's flight controller, and "it is only on
 * my LAN" is the sentence that precedes every one of those incidents.
 */
export class BridgeTransport implements Transport {
  readonly info: TransportInfo;
  private socket: WebSocket | null = null;
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();
  private closedReported = false;

  constructor(private readonly options: BridgeOptions = {}) {
    const url = options.url ?? DEFAULT_BRIDGE_URL;
    this.info = { kind: 'bridge', label: 'Local bridge', detail: url };
    if (!isLoopback(url)) {
      throw new TransportError(
        `the bridge must be on this machine; ${url} is not. The bridge is an ` +
          'unauthenticated byte pipe — it does not belong on a network.',
      );
    }
  }

  static defaultUrl(): string {
    return DEFAULT_BRIDGE_URL;
  }

  get isOpen(): boolean {
    return this.socket !== null && this.socket.readyState === WebSocket.OPEN;
  }

  async open(): Promise<void> {
    const url = this.options.url ?? DEFAULT_BRIDGE_URL;
    const timeout = this.options.openTimeoutMs ?? 3000;
    this.closedReported = false;

    const socket = new WebSocket(url);
    socket.binaryType = 'arraybuffer';

    await new Promise<void>((resolve, reject) => {
      const timer = setTimeout(() => {
        socket.close();
        reject(
          new TransportError(
            `nothing answered at ${url} within ${timeout} ms. Start the bridge ` +
              '(npm run bridge, in apps/configurator) and try again.',
          ),
        );
      }, timeout);

      socket.onopen = () => {
        clearTimeout(timer);
        resolve();
      };
      socket.onerror = () => {
        clearTimeout(timer);
        reject(
          new TransportError(
            `could not reach ${url}. Start the bridge (npm run bridge) and try again.`,
          ),
        );
      };
    });

    this.socket = socket;
    socket.onmessage = (event: MessageEvent) => {
      const data: unknown = event.data;
      let bytes: Uint8Array | null = null;
      if (data instanceof ArrayBuffer) bytes = new Uint8Array(data);
      else if (data instanceof Uint8Array) bytes = data;
      if (bytes === null || bytes.length === 0) return;
      for (const handler of this.dataHandlers) handler(bytes);
    };
    socket.onclose = (event: CloseEvent) => {
      this.socket = null;
      this.reportClose(
        event.reason !== '' ? event.reason : `the bridge closed the socket (code ${event.code})`,
      );
    };
    socket.onerror = () => {
      this.reportError('the bridge socket reported an error');
    };
  }

  private reportClose(reason: string): void {
    if (this.closedReported) return;
    this.closedReported = true;
    for (const handler of this.closeHandlers) handler(reason);
  }

  private reportError(message: string): void {
    for (const handler of this.errorHandlers) handler(message);
  }

  async write(bytes: Uint8Array): Promise<void> {
    if (this.socket === null || this.socket.readyState !== WebSocket.OPEN) {
      throw new TransportError('the bridge is not connected');
    }
    // A copy: the socket may send after the caller has reused its buffer, and a
    // frame that mutates between here and the wire is a checksum failure that
    // looks like a firmware bug.
    this.socket.send(bytes.slice());
  }

  async close(): Promise<void> {
    const socket = this.socket;
    this.socket = null;
    if (socket === null) return;
    socket.onmessage = null;
    socket.onerror = null;
    socket.onclose = () => this.reportClose('closed by this page');
    try {
      socket.close(1000, 'closed by this page');
    } catch {
      /* already closing */
    }
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

/** Loopback only. `localhost` is included because it is the same machine, and
 *  `[::1]` because that is what localhost resolves to on a dual-stack box. */
function isLoopback(url: string): boolean {
  let host: string;
  try {
    host = new URL(url).hostname;
  } catch {
    return false;
  }
  return host === '127.0.0.1' || host === 'localhost' || host === '::1' || host === '[::1]';
}
