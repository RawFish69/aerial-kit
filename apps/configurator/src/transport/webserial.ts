import type { ByteLink } from '../protocol/client';
import { TransportError, type Transport, type TransportInfo } from './types';

/** The firmware's console UART runs at 115200. */
export const DEFAULT_BAUD = 115200;

/** The USB ids of an AerialKit console (ST's VCP ids, which the firmware uses). */
export const AERIALKIT_USB = { usbVendorId: 0x0483, usbProductId: 0x5740 } as const;

interface SerialLike {
  open(options: { baudRate: number }): Promise<void>;
  getInfo?(): { usbVendorId?: number; usbProductId?: number };
  close(): Promise<void>;
  readable: ReadableStream<Uint8Array> | null;
  writable: WritableStream<Uint8Array> | null;
}

function isPresent(value: unknown): boolean {
  return typeof value === 'object' && value !== null;
}

/**
 * A board on a USB cable, through the browser's own Web Serial.
 *
 * This is the only transport with no companion software in front of it, and it
 * is also the most restricted one. Two of its restrictions are structural and
 * are handled here rather than in the UI:
 *
 *  - **`requestPort()` needs a user gesture.** A browser will refuse it from a
 *    timer or a mount effect, so `open()` must be reached from a click. The
 *    caller does that; this class does not try to work around it.
 *  - **A page can only reach ports a person has chosen.** There is no
 *    enumeration of "all boards on this machine" and there will not be: the
 *    chooser *is* the permission boundary. So "connect" always shows a chooser
 *    the first time, and the page never learns about a port the person did not
 *    hand it.
 *
 * The page persists nothing. The browser does keep a grant once a person has
 * chosen a port (and a managed policy can grant one up front), so `open()`
 * reuses a granted AerialKit port instead of showing the chooser every time -
 * still only from a click on Connect, so nothing opens because a page loaded.
 * `choose: true` forces the chooser for a person who wants a different port.
 */
export class WebSerialTransport implements Transport {
  readonly info: TransportInfo = {
    kind: 'serial',
    label: 'USB serial',
    detail: `Web Serial at ${DEFAULT_BAUD} baud`,
  };

  private port: SerialLike | null = null;

  /**
   * Set when Chrome reported the port lost within a second of opening it.
   *
   * On Linux that is almost always a tty left with VMIN=0 by the last program
   * that used it (pyserial does this): Chrome inherits it, its second read
   * returns zero bytes and it calls that a lost device, though the board never
   * left the bus. Measured on the Y520, 2026-10-01.
   */
  earlyLoss: string | null = null;
  private openedAt = 0;

  /** Show the chooser even when a granted AerialKit port exists. */
  constructor(private readonly choose = false) {}
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
  private reading = false;
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();

  static supported(): boolean {
    return typeof navigator !== 'undefined' && 'serial' in navigator;
  }

  get isOpen(): boolean {
    return this.reader !== null;
  }

  async open(): Promise<void> {
    const serial = (navigator as unknown as { serial?: { requestPort(): Promise<SerialLike> } })
      .serial;
    if (serial === undefined || !isPresent(serial)) {
      throw new TransportError(
        'this browser has no Web Serial. Chrome, Edge and Opera on a desktop have it; ' +
          'Firefox and Safari do not, and no phone browser does.',
      );
    }

    let port: SerialLike;
    try {
      const granted = this.choose ? [] : await grantedPorts(serial);
      const ours = granted.find((candidate) => {
        const info = candidate.getInfo?.() ?? {};
        return info.usbVendorId === AERIALKIT_USB.usbVendorId && info.usbProductId === AERIALKIT_USB.usbProductId;
      });
      port = ours ?? (await serial.requestPort());
    } catch (error) {
      // A dismissed chooser rejects with NotFoundError, which is a person
      // changing their mind and not a failure worth a red banner.
      const name = (error as { name?: string } | null)?.name;
      if (name === 'NotFoundError') throw new TransportError('no port was chosen');
      throw new TransportError(`the browser refused the port chooser: ${String(error)}`);
    }

    try {
      await port.open({ baudRate: DEFAULT_BAUD });
    } catch (error) {
      // On Windows a COM port has one owner at a time, and this is what the
      // browser says when someone else is it.
      throw new TransportError(
        'The port is in use by another program. Close anything else connected to the board ' +
          '(another configurator, a serial monitor, a Python script), or unplug and replug it, ' +
          `then connect again. (${String(error)})`,
      );
    }

    this.port = port;
    this.writer = port.writable?.getWriter() ?? null;
    if (port.readable === null) {
      throw new TransportError('the port opened with nothing to read from');
    }
    this.reader = port.readable.getReader();
    this.openedAt = Date.now();
    this.earlyLoss = null;
    void this.readLoop();
  }

  private async readLoop(): Promise<void> {
    this.reading = true;
    try {
      for (;;) {
        const { value, done } = await this.reader!.read();
        if (done) break;
        if (value !== undefined && value.length > 0) {
          for (const handler of this.dataHandlers) handler(value);
        }
      }
      this.announceClose('the board stopped sending');
    } catch (error) {
      // An unplugged board lands here. It is the normal way a session ends, so
      // it is reported as a disconnect and not as an exception.
      if (this.reading && Date.now() - this.openedAt < 1000) {
        this.earlyLoss =
          'Chrome lost the port a moment after opening it, while the board stayed connected. On Linux this is ' +
          'a port left with VMIN=0 by the last program that used it (a pyserial script, a serial monitor). Run ' +
          '`stty -F /dev/ttyACM0 min 1` and connect again.';
      }
      if (this.reading) this.announceClose(`the port failed: ${String(error)}`);
    } finally {
      this.reading = false;
    }
  }

  private announceClose(reason: string): void {
    for (const handler of this.closeHandlers) handler(reason);
  }

  async write(bytes: Uint8Array): Promise<void> {
    if (this.writer === null) throw new TransportError('the port is not open');
    try {
      await this.writer.write(bytes);
    } catch (error) {
      this.announceClose(`the write failed: ${String(error)}`);
      throw new TransportError(`the write failed: ${String(error)}`);
    }
  }

  async close(): Promise<void> {
    this.reading = false;
    try {
      await this.reader?.cancel();
    } catch {
      /* cancelling a reader that already ended is not a problem */
    }
    try {
      this.reader?.releaseLock();
    } catch {
      /* released twice */
    }
    this.reader = null;
    try {
      this.writer?.releaseLock();
    } catch {
      /* released twice */
    }
    this.writer = null;
    try {
      await this.port?.close();
    } catch {
      /* already gone */
    }
    this.port = null;
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

async function grantedPorts(serial: unknown): Promise<SerialLike[]> {
  const getPorts = (serial as { getPorts?: () => Promise<SerialLike[]> }).getPorts;
  if (typeof getPorts !== 'function') return [];
  try {
    return await getPorts.call(serial);
  } catch {
    return [];
  }
}

/** Exported for the type of a `ByteLink` built on something else. */
export type { ByteLink };
