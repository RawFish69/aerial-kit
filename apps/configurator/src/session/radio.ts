import type { ByteLink } from '../protocol/client';
import {
  Address,
  CommandStep,
  CrsfDecoder,
  FrameType,
  decodeDeviceInfo,
  decodeParameter,
  decodeParameterChunk,
  decodeTelemetry,
  encodeDevicePing,
  encodeParameterRead,
  encodeParameterWrite,
  parameterValueBytes,
  type CrsfFrame,
  type DeviceInfo,
  type LinkStatistics,
  type Parameter,
  type Telemetry,
} from '../protocol/crsf';

/**
 * A radio module (ExpressLRS transmitter) on its handset port, the way the
 * radio's own Lua script talks to it: ping, read the device's menu field by
 * field (each field may arrive in chunks), write a value, run a command.
 *
 * Separate from the flight-controller session on purpose: a radio module is
 * a different device on a different port at a different rate (400000 baud),
 * and nothing here can reach a flight controller.
 *
 * Writes are refused until `allowWrites` is set: the same "nothing is written
 * until you send it" rule the rest of this app follows.
 */

export interface RadioSnapshot {
  readonly device: DeviceInfo | null;
  readonly parameters: ReadonlyMap<number, Parameter>;
  readonly loading: { readonly done: number; readonly total: number } | null;
  readonly link: LinkStatistics | null;
  readonly linkAgeMs: number | null;
  readonly telemetry: Readonly<Partial<Record<Telemetry['kind'], Telemetry>>>;
  readonly error: string | null;
}

export class RadioWriteRefused extends Error {
  constructor() {
    super('changes to the radio are off: turn on "Allow changes" first');
    this.name = 'RadioWriteRefused';
  }
}

interface Waiter {
  readonly match: (frame: CrsfFrame) => boolean;
  readonly resolve: (frame: CrsfFrame) => void;
}

export class RadioSession {
  allowWrites = false;
  private readonly decoder = new CrsfDecoder();
  private readonly waiters = new Set<Waiter>();
  private readonly listeners = new Set<(s: RadioSnapshot) => void>();
  private readonly unsubscribe: Array<() => void> = [];
  private device: DeviceInfo | null = null;
  private readonly parameters = new Map<number, Parameter>();
  private loading: RadioSnapshot['loading'] = null;
  private link: LinkStatistics | null = null;
  private linkAt = 0;
  private readonly telemetry: Partial<Record<Telemetry['kind'], Telemetry>> = {};
  private error: string | null = null;

  constructor(private readonly linkPort: ByteLink, private readonly timeoutMs = 600,
    private readonly now: () => number = () => Date.now()) {
    this.unsubscribe.push(linkPort.onData((chunk) => this.onData(chunk)));
    this.unsubscribe.push(linkPort.onError((message) => this.fail(message)));
  }

  subscribe(listener: (s: RadioSnapshot) => void): () => void {
    this.listeners.add(listener);
    listener(this.snapshot());
    return () => this.listeners.delete(listener);
  }

  snapshot(): RadioSnapshot {
    return {
      device: this.device,
      parameters: new Map(this.parameters),
      loading: this.loading,
      link: this.link,
      linkAgeMs: this.link === null ? null : this.now() - this.linkAt,
      telemetry: { ...this.telemetry },
      error: this.error,
    };
  }

  private changed(): void {
    const s = this.snapshot();
    for (const l of this.listeners) l(s);
  }

  private fail(message: string): void {
    this.error = message;
    this.changed();
  }

  private onData(chunk: Uint8Array): void {
    let touched = false;
    for (const frame of this.decoder.push(chunk)) {
      for (const w of [...this.waiters]) {
        if (w.match(frame)) {
          this.waiters.delete(w);
          w.resolve(frame);
        }
      }
      const t = decodeTelemetry(frame);
      if (t !== null) {
        if (t.kind === 'link') {
          this.link = t.value;
          this.linkAt = this.now();
        } else {
          this.telemetry[t.kind] = t;
        }
        touched = true;
      }
    }
    if (touched) this.changed();
  }

  private request(bytes: Uint8Array, match: (f: CrsfFrame) => boolean, what: string): Promise<CrsfFrame> {
    return new Promise((resolve, reject) => {
      const waiter: Waiter = { match, resolve: (f) => { clearTimeout(timer); resolve(f); } };
      const timer = setTimeout(() => {
        this.waiters.delete(waiter);
        reject(new Error(`the radio did not answer ${what} within ${this.timeoutMs} ms`));
      }, this.timeoutMs);
      this.waiters.add(waiter);
      void Promise.resolve(this.linkPort.write(bytes)).catch((e: unknown) => {
        clearTimeout(timer);
        this.waiters.delete(waiter);
        reject(e instanceof Error ? e : new Error(String(e)));
      });
    });
  }

  private async retry<T>(attempts: number, fn: () => Promise<T>): Promise<T> {
    let last: unknown;
    for (let i = 0; i < attempts; i++) {
      try {
        return await fn();
      } catch (e) {
        last = e;
      }
    }
    throw last;
  }

  /** Ping until a module answers with its device info. */
  async identify(attempts = 3): Promise<DeviceInfo> {
    const frame = await this.retry(attempts, () => this.request(encodeDevicePing(),
      (f) => f.type === FrameType.DEVICE_INFO && decodeDeviceInfo(f)?.origin === Address.RADIO_TRANSMITTER,
      'a device ping'));
    this.device = decodeDeviceInfo(frame);
    this.error = null;
    this.changed();
    return this.device!;
  }

  /** One field, all of its chunks. */
  async readParameter(field: number): Promise<Parameter> {
    const data: number[] = [];
    for (let chunk = 0; ; chunk++) {
      const frame = await this.retry(3, () => this.request(
        encodeParameterRead(Address.RADIO_TRANSMITTER, field, chunk),
        (f) => decodeParameterChunk(f)?.field === field, `field ${field} chunk ${chunk}`));
      const part = decodeParameterChunk(frame)!;
      data.push(...part.data);
      if (part.chunksRemaining === 0) break;
      if (chunk > 32) throw new Error(`field ${field} never finished`);
    }
    const p = decodeParameter(field, Uint8Array.from(data));
    this.parameters.set(field, p);
    this.changed();
    return p;
  }

  /** The whole menu: fields 1..parameterCount, in order. */
  async readAll(): Promise<ReadonlyMap<number, Parameter>> {
    const device = this.device ?? (await this.identify());
    const total = device.parameterCount;
    for (let field = 1; field <= total; field++) {
      this.loading = { done: field - 1, total };
      this.changed();
      try {
        await this.readParameter(field);
      } catch (e) {
        this.loading = null;
        this.fail(e instanceof Error ? e.message : String(e));
        throw e;
      }
    }
    this.loading = null;
    this.changed();
    return this.parameters;
  }

  /** Write a selection or number, then read the field back to show what the module now holds. */
  async write(field: number, value: number): Promise<Parameter> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    const p = this.parameters.get(field);
    if (p === undefined || (p.kind !== 'selection' && p.kind !== 'number')) {
      throw new Error(`field ${field} is not a value that can be set`);
    }
    if (value < p.min || value > p.max) throw new RangeError(`${p.name}: ${value} is outside ${p.min}..${p.max}`);
    const type = p.kind === 'number' ? p.type : 9;
    await this.linkPort.write(encodeParameterWrite(Address.RADIO_TRANSMITTER, field, parameterValueBytes(type, value)));
    return this.readParameter(field);
  }

  /** Run a command (e.g. Bind): click, then poll its status until it is idle again. */
  async runCommand(field: number, maxPolls = 20): Promise<Parameter> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    const p = this.parameters.get(field);
    if (p?.kind !== 'command') throw new Error(`field ${field} is not a command`);
    const step = async (s: number) => this.request(
      encodeParameterWrite(Address.RADIO_TRANSMITTER, field, Uint8Array.of(s)),
      (f) => decodeParameterChunk(f)?.field === field, `${p.name}`);
    let frame = await step(CommandStep.CLICK);
    for (let i = 0; i < maxPolls; i++) {
      const c = decodeParameterChunk(frame)!;
      const now = decodeParameter(field, c.data);
      this.parameters.set(field, now);
      this.changed();
      if (now.kind !== 'command' || now.step === CommandStep.IDLE) return now;
      if (now.step === CommandStep.ASK_CONFIRM) return now;     // the UI asks the person, then confirm()
      frame = await step(CommandStep.QUERY);
    }
    throw new Error(`${p.name} did not finish`);
  }

  async confirm(field: number): Promise<Parameter> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    await this.linkPort.write(encodeParameterWrite(Address.RADIO_TRANSMITTER, field, Uint8Array.of(CommandStep.CONFIRMED)));
    return this.readParameter(field);
  }

  dispose(): void {
    for (const u of this.unsubscribe) u();
    this.waiters.clear();
    this.listeners.clear();
  }
}
