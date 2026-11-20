import {
  Address,
  CommandStep,
  CrsfDecoder,
  FrameType,
  MSP_ELRS_RXTX_CONFIG,
  ParamType,
  RxtxConfigSubcommand,
  encodeExtended,
  encodeFrame,
  encodeRcChannels,
  unpackChannels,
  type CrsfFrame,
} from '../protocol/crsf';
import { bindingUid } from '../protocol/elrs-binding';
import type { Transport, TransportInfo } from './types';

/**
 * A simulated ExpressLRS transmitter module inside the page.
 *
 * A protocol peer, not a mock of the UI: it answers CRSF device pings, serves
 * its parameter menu as chunked PARAMETER_SETTINGS_ENTRY frames laid out the
 * way ExpressLRS 4.1.0's CRSFEndpoint lays them out, applies writes, runs a
 * bind command through its steps, and sends link statistics while its
 * pretend receiver is connected. Its name says "demo" so it cannot pass for
 * a module on a cable.
 */

const z = (s: string) => [...new TextEncoder().encode(s), 0];

type Entry =
  | { id: number; parent: number; type: typeof ParamType.TEXT_SELECTION; name: string; options: string[]; value: number }
  | { id: number; parent: number; type: typeof ParamType.FOLDER; name: string }
  | { id: number; parent: number; type: typeof ParamType.INFO; name: string; value: string }
  | { id: number; parent: number; type: typeof ParamType.COMMAND; name: string; step: number; info: string };

export const DEMO_RADIO_NAME = 'AK ELRS TX (demo)';
export const DEMO_RADIO_RX_NAME = 'AK ELRS RX (demo)';

function demoMenu(role: 'tx' | 'rx'): Entry[] {
  if (role === 'rx') {
    return [
      { id: 1, parent: 0, type: ParamType.TEXT_SELECTION, name: 'Bind Storage', options: ['Persistent', 'Volatile', 'Returnable', 'Administered'], value: 0 },
      { id: 2, parent: 0, type: ParamType.COMMAND, name: 'Enter Bind Mode', step: CommandStep.IDLE, info: '' },
      { id: 3, parent: 0, type: ParamType.INFO, name: 'Version', value: '4.1.0 AK-A3 RX' },
    ];
  }
  return [
    { id: 1, parent: 0, type: ParamType.TEXT_SELECTION, name: 'Packet Rate', options: ['50Hz', '100Hz', '150Hz', '250Hz', '333Hz Full', '500Hz'], value: 2 },
    { id: 2, parent: 0, type: ParamType.TEXT_SELECTION, name: 'Telem Ratio', options: ['Std', 'Off', '1:128', '1:64', '1:32', '1:16', '1:8', '1:4', '1:2', 'Race'], value: 0 },
    { id: 3, parent: 0, type: ParamType.TEXT_SELECTION, name: 'Switch Mode', options: ['Wide', 'Hybrid'], value: 1 },
    { id: 4, parent: 0, type: ParamType.FOLDER, name: 'TX Power' },
    { id: 5, parent: 4, type: ParamType.TEXT_SELECTION, name: 'Max Power', options: ['10', '25', '50', '100', '250'], value: 3 },
    { id: 6, parent: 4, type: ParamType.TEXT_SELECTION, name: 'Dynamic', options: ['Off', 'Dyn', 'AUX9', 'AUX10'], value: 1 },
    { id: 7, parent: 0, type: ParamType.TEXT_SELECTION, name: 'Model Match', options: ['Off', 'On'], value: 0 },
    { id: 8, parent: 0, type: ParamType.COMMAND, name: 'Bind', step: CommandStep.IDLE, info: '' },
    { id: 9, parent: 0, type: ParamType.INFO, name: 'Version', value: '4.1.0 AK-A3' },
    { id: 10, parent: 0, type: ParamType.FOLDER, name: 'WiFi Connectivity' },
    { id: 11, parent: 10, type: ParamType.COMMAND, name: 'Enable WiFi', step: CommandStep.IDLE, info: '' },
    { id: 12, parent: 10, type: ParamType.COMMAND, name: 'Enable Rx WiFi', step: CommandStep.IDLE, info: '' },
  ];
}

function body(e: Entry, menu: Entry[]): number[] {
  switch (e.type) {
    case ParamType.TEXT_SELECTION:
      return [e.parent, e.type, ...z(e.name), ...z(e.options.join(';')), e.value, 0, e.options.length - 1, 0, ...z(e.name === 'Max Power' ? 'mW' : '')];
    case ParamType.FOLDER:
      return [e.parent, e.type, ...z(e.name), ...menu.filter((c) => c.parent === e.id).map((c) => c.id), 0xff];
    case ParamType.INFO:
      return [e.parent, e.type, ...z(e.name), ...z(e.value)];
    case ParamType.COMMAND:
      return [e.parent, e.type, ...z(e.name), e.step, 200, ...z(e.info)];
  }
}

export interface DemoRadioOptions {
  readonly role?: 'tx' | 'rx';
  /** Largest chunk of entry data per frame; small enough to make the menu arrive in pieces. */
  readonly chunkMax?: number;
  readonly linkStatsMs?: number;
  /** Drop this many UID reads before answering, to exercise reconnect/retry. */
  readonly uidFailures?: number;
}

export class DemoRadio {
  readonly role: 'tx' | 'rx';
  readonly menu: Entry[];
  receiverConnected = true;
  uid: Uint8Array = Uint8Array.of(0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc);
  directBindRequests = 0;
  readonly directBindDestinations: number[] = [];
  moduleWifiRequested = false;
  receiverWifiRequested = false;
  private readonly decoder = new CrsfDecoder();
  private readonly chunkMax: number;
  private uidFailuresRemaining: number;
  lastRcChannels: number[] | null = null;

  constructor(options: DemoRadioOptions = {}) {
    this.role = options.role ?? 'tx';
    this.menu = demoMenu(this.role);
    this.chunkMax = options.chunkMax ?? 20;
    this.uidFailuresRemaining = options.uidFailures ?? 0;
  }

  feed(bytes: Uint8Array): Uint8Array[] {
    const out: Uint8Array[] = [];
    for (const frame of this.decoder.push(bytes)) {
      if (frame.type === FrameType.RC_CHANNELS_PACKED && frame.payload.length >= 22) {
        this.lastRcChannels = unpackChannels(frame.payload);
        continue;
      }
      out.push(...this.answer(frame));
    }
    return out;
  }

  private ext(type: number, to: number, data: number[], origin = this.origin): Uint8Array {
    return encodeExtended(type, to, origin, Uint8Array.from(data), Address.RADIO_TRANSMITTER);
  }

  private get origin(): number {
    return this.role === 'rx' ? Address.CRSF_RECEIVER : Address.CRSF_TRANSMITTER;
  }

  private answer(frame: CrsfFrame): Uint8Array[] {
    if (frame.type < FrameType.DEVICE_PING || frame.payload.length < 2) return [];
    const p = frame.payload;
    const to = p[0] ?? 0;
    const from = p[1] ?? Address.ELRS_LUA;
    if (to !== Address.BROADCAST && to !== Address.CRSF_TRANSMITTER &&
      to !== Address.CRSF_RECEIVER && to !== Address.RADIO_TRANSMITTER) return [];
    switch (frame.type) {
      case FrameType.DEVICE_PING:
        return [this.ext(FrameType.DEVICE_INFO, from, [
          ...z(this.role === 'rx' ? DEMO_RADIO_RX_NAME : DEMO_RADIO_NAME),
          0x45, 0x4c, 0x52, 0x53, 0, 0, 0, 0, 0, 4, 1, 0, this.menu.length, 0])];
      case FrameType.COMMAND:
        if (p[2] === 0x10 && p[3] === 0x01) {
          this.directBindRequests++;
          this.directBindDestinations.push(to);
          this.receiverConnected = false;
        }
        return [];
      case FrameType.PARAMETER_READ:
        return this.entryChunk(from, p[2] ?? 0, p[3] ?? 0);
      case FrameType.PARAMETER_WRITE: {
        const e = this.menu.find((m) => m.id === p[2]);
        const v = p[3] ?? 0;
        if (e === undefined) return [];
        if (e.type === ParamType.TEXT_SELECTION && v < e.options.length) e.value = v;
        if (e.type === ParamType.COMMAND) this.command(e, v);
        return e.type === ParamType.COMMAND ? this.entryChunk(from, e.id, 0) : [];
      }
      case FrameType.MSP_REQ:
        if (p[4] === MSP_ELRS_RXTX_CONFIG && p[5] === 0 && p[6] === 1 &&
          p[8] === RxtxConfigSubcommand.UID) {
          if (this.uidFailuresRemaining > 0) {
            this.uidFailuresRemaining--;
            return [];
          }
          return [this.mspResponse(from, Uint8Array.from([RxtxConfigSubcommand.UID, ...this.uid]))];
        }
        return [];
      case FrameType.MSP_WRITE:
        if (p[4] === MSP_ELRS_RXTX_CONFIG) {
          if (p[5] === RxtxConfigSubcommand.UID && p.length >= 12) {
            this.uid = p.slice(6, 12);
          } else if (p[5] === RxtxConfigSubcommand.BIND_PHRASE) {
            const phrase = new TextDecoder().decode(p.slice(6, -1));
            this.uid = bindingUid(phrase);
          }
        }
        return [];
      default:
        return [];
    }
  }

  private mspResponse(to: number, payload: Uint8Array): Uint8Array {
    const body = new Uint8Array(payload.length + 4);
    body[0] = 0x30;
    body[1] = payload.length;
    body[2] = MSP_ELRS_RXTX_CONFIG;
    body.set(payload, 3);
    let innerCrc = 0;
    for (const byte of body.subarray(1, body.length - 1)) innerCrc ^= byte;
    body[body.length - 1] = innerCrc;
    return this.ext(FrameType.MSP_RESP, to, [...body]);
  }

  private command(e: Extract<Entry, { type: typeof ParamType.COMMAND }>, step: number): void {
    if (step === CommandStep.CLICK) {
      if (e.name === 'Enable WiFi') {
        e.step = this.receiverConnected ? CommandStep.ASK_CONFIRM : CommandStep.EXECUTING;
        e.info = this.receiverConnected ? 'Enter WiFi Update?' : 'WiFi Running...';
      } else if (e.name === 'Enable Rx WiFi') {
        e.step = CommandStep.EXECUTING;
        e.info = 'Receiver WiFi Running...';
        this.receiverWifiRequested = true;
      } else {
        e.step = CommandStep.EXECUTING;
        e.info = 'Binding...';
        this.receiverConnected = false;
      }
    } else if (step === CommandStep.CONFIRMED && e.step === CommandStep.ASK_CONFIRM && e.name === 'Enable WiFi') {
      e.step = CommandStep.EXECUTING;
      e.info = 'WiFi Running...';
      this.moduleWifiRequested = true;
      this.receiverConnected = false;
    } else if (step === CommandStep.QUERY && e.step === CommandStep.EXECUTING) {
      e.step = CommandStep.IDLE;
      if (e.name === 'Bind') {
        e.info = 'Bound';
        this.receiverConnected = true;
      }
    } else if (step === CommandStep.CANCEL) {
      e.step = CommandStep.IDLE;
      e.info = '';
    }
  }

  private entryChunk(to: number, field: number, chunk: number): Uint8Array[] {
    const e = this.menu.find((m) => m.id === field);
    if (e === undefined) return [];
    const data = body(e, this.menu);
    const count = Math.max(1, Math.ceil(data.length / this.chunkMax));
    if (chunk >= count) return [];
    const part = data.slice(chunk * this.chunkMax, (chunk + 1) * this.chunkMax);
    return [this.ext(FrameType.PARAMETER_SETTINGS_ENTRY, to, [field, count - 1 - chunk, ...part])];
  }

  /** What the module sends unasked: link statistics while its receiver is connected. */
  linkStatistics(nowMs: number): Uint8Array | null {
    if (!this.receiverConnected) return null;
    const wobble = Math.round(3 * Math.sin(nowMs / 700));
    const rate = this.role === 'tx' ? (this.menu[0] as { value: number }).value : 2;
    const power = this.role === 'tx'
      ? [10, 25, 50, 100, 250].indexOf(Number(
        (this.menu[4] as { options: string[]; value: number }).options[
          (this.menu[4] as { value: number }).value
        ],
      ))
      : 0;
    return encodeFrame(FrameType.LINK_STATISTICS, Uint8Array.of(
      62 - wobble, 64 - wobble, 100, 9, 0, rate, Math.max(power, 0) + 1,
      58 + wobble, 100, 8), Address.RADIO_TRANSMITTER);
  }

  rcChannels(): Uint8Array | null {
    if (this.role !== 'rx' || !this.receiverConnected) return null;
    return encodeRcChannels([
      992, 992, 172, 992, 992, 172, 992, 992,
      992, 992, 992, 992, 992, 992, 992, 992,
    ]);
  }
}

export class DemoRadioTransport implements Transport {
  readonly info: TransportInfo = {
    kind: 'demo',
    label: 'Demo radio',
    detail: 'a simulated ExpressLRS module inside this page — no hardware, no RF, nothing saved',
  };

  readonly radio: DemoRadio;
  private timer: ReturnType<typeof setInterval> | null = null;
  private rcTimer: ReturnType<typeof setInterval> | null = null;
  private readonly dataHandlers = new Set<(chunk: Uint8Array) => void>();
  private readonly closeHandlers = new Set<(reason: string) => void>();
  private readonly errorHandlers = new Set<(message: string) => void>();
  private open_ = false;
  reconnects = 0;
  private readonly linkStatsMs: number;

  constructor(options: DemoRadioOptions = {}) {
    this.radio = new DemoRadio(options);
    this.linkStatsMs = options.linkStatsMs ?? 200;
  }

  get isOpen(): boolean {
    return this.open_;
  }

  async open(): Promise<void> {
    if (this.open_) return;
    this.open_ = true;
    this.timer = setInterval(() => {
      const frame = this.radio.linkStatistics(Date.now());
      if (frame !== null) this.emit(frame);
    }, this.linkStatsMs);
    this.rcTimer = setInterval(() => {
      const frame = this.radio.rcChannels();
      if (frame !== null) this.emit(frame);
    }, 7);
  }

  private emit(bytes: Uint8Array): void {
    for (const handler of this.dataHandlers) handler(bytes);
  }

  async write(bytes: Uint8Array): Promise<void> {
    if (!this.open_) {
      for (const handler of this.errorHandlers) handler('the demo radio is not open');
      return;
    }
    // Answered on a later tick, as a cable would, so timeouts and correlation are exercised.
    await Promise.resolve();
    for (const reply of this.radio.feed(bytes)) this.emit(reply);
  }

  async reconnect(): Promise<void> {
    this.reconnects++;
    if (!this.open_) await this.open();
  }

  async close(): Promise<void> {
    if (!this.open_) return;
    this.open_ = false;
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
    if (this.rcTimer !== null) clearInterval(this.rcTimer);
    this.rcTimer = null;
    for (const handler of this.closeHandlers) handler('the demo radio was closed');
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
