import type { ByteLink } from '../protocol/client';
import {
  Address,
  CommandStep,
  CrsfDecoder,
  FrameType,
  decodeDeviceInfo,
  decodeParameter,
  decodeParameterChunk,
  decodeMspResponse,
  decodeTelemetry,
  encodeDefaultTelemetryFrames,
  encodeCrsfBindCommand,
  encodeDevicePing,
  encodeMspV2Request,
  encodeMspWrite,
  encodeParameterRead,
  encodeParameterWrite,
  encodeRcChannels,
  MSP_ELRS_RXTX_CONFIG,
  RxtxConfigSubcommand,
  parameterValueBytes,
  CHANNEL_MAX,
  CHANNEL_MID,
  CHANNEL_MIN,
  unpackChannels,
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
  readonly role: 'tx' | 'rx' | 'unknown';
  readonly uid: Uint8Array | null;
  readonly bench: RadioBenchSnapshot;
  readonly error: string | null;
}

export type BenchRequirementStatus = 'PASS' | 'FAIL' | 'NOT TESTED' | 'SENT / NOT VERIFIED';

export interface RadioBenchSnapshot {
  readonly running: boolean;
  readonly mode: 'idle' | 'drive' | 'capture';
  readonly startedAtMs: number | null;
  readonly endsAtMs: number | null;
  readonly requestedHz: number;
  readonly observedHz: number | null;
  readonly framesSent: number;
  readonly rcFramesCaptured: number;
  readonly channels: number[] | null;
  readonly frameCounts: Readonly<Record<string, number>>;
  readonly crcErrors: number;
  readonly resyncs: number;
  readonly framingErrors: number;
  readonly linkStatistics: number;
  readonly telemetryKinds: readonly string[];
  readonly telemetryInjected: number;
  readonly safeDisarmSent: boolean;
  readonly requirements: Readonly<Record<string, BenchRequirementStatus>>;
  readonly result: 'IDLE' | 'RUNNING' | 'PASS' | 'FAIL' | 'NOT TESTED';
  readonly summary: string;
  readonly error: string | null;
}

type Mutable<T> = {
  -readonly [P in keyof T]: T[P] extends readonly (infer U)[]
    ? U[]
    : T[P] extends Readonly<Record<string, infer V>>
      ? Record<string, V>
      : T[P];
};

type MutableBench = Mutable<RadioBenchSnapshot>;

interface BenchCounters {
  readonly frames: number;
  readonly bytes: number;
  readonly crcErrors: number;
  readonly resyncs: number;
  readonly framingErrors: number;
}

const BENCH_FRAME_NAMES: Readonly<Record<number, string>> = {
  [FrameType.GPS]: 'gps',
  [FrameType.BATTERY]: 'battery',
  [FrameType.BARO_ALTITUDE]: 'barometer',
  [FrameType.LINK_STATISTICS]: 'link_statistics',
  [FrameType.RC_CHANNELS_PACKED]: 'rc_channels',
  [FrameType.ATTITUDE]: 'attitude',
  [FrameType.FLIGHT_MODE]: 'flight_mode',
  [FrameType.DEVICE_INFO]: 'device_info',
};

function idleBench(): MutableBench {
  return {
    running: false,
    mode: 'idle',
    startedAtMs: null,
    endsAtMs: null,
    requestedHz: 150,
    observedHz: null,
    framesSent: 0,
    rcFramesCaptured: 0,
    channels: null,
    frameCounts: {},
    crcErrors: 0,
    resyncs: 0,
    framingErrors: 0,
    linkStatistics: 0,
    telemetryKinds: [],
    telemetryInjected: 0,
    safeDisarmSent: false,
    requirements: {},
    result: 'IDLE',
    summary: 'No bench operation has run.',
    error: null,
  };
}

function neutralChannels(): number[] {
  const channels = Array<number>(16).fill(CHANNEL_MID);
  channels[2] = CHANNEL_MIN; // throttle
  channels[5] = CHANNEL_MIN; // arm
  return channels;
}

function driveChannels(pattern: 'disarmed' | 'arm-test' | 'channel-test',
  elapsedSec: number, armHoldSec: number): number[] {
  const channels = neutralChannels();
  if (pattern === 'disarmed') return channels;
  channels[4] = CHANNEL_MAX; // angle mode while armed
  channels[5] = CHANNEL_MAX; // arm
  if (elapsedSec < armHoldSec || pattern === 'arm-test') return channels;
  const phase = (elapsedSec - armHoldSec) % 8;
  channels[2] = CHANNEL_MIN + Math.round((CHANNEL_MAX - CHANNEL_MIN) * 0.1);
  if (phase < 2) channels[0] = CHANNEL_MID - 240;
  else if (phase < 4) channels[0] = CHANNEL_MID + 240;
  else if (phase < 6) channels[1] = CHANNEL_MID - 240;
  else channels[1] = CHANNEL_MID + 240;
  return channels;
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
  private targetAddress: number = Address.RADIO_TRANSMITTER;
  private role: 'tx' | 'rx' | 'unknown' = 'unknown';
  private uid: Uint8Array | null = null;
  private error: string | null = null;
  private benchState: MutableBench = idleBench();
  private benchBaseline: BenchCounters = { frames: 0, bytes: 0, crcErrors: 0, resyncs: 0, framingErrors: 0 };
  private benchTimer: ReturnType<typeof setTimeout> | null = null;
  private benchStartedAt = 0;
  private benchDeadline = 0;
  private benchPattern: 'disarmed' | 'arm-test' | 'channel-test' = 'disarmed';
  private benchArmHoldSec = 0.75;
  private benchDriveArmed = false;
  private benchLastPublish = 0;
  private benchWriting = false;

  constructor(private readonly linkPort: ByteLink, private readonly timeoutMs = 600,
    private readonly now: () => number = () => Date.now()) {
    this.unsubscribe.push(linkPort.onData((chunk) => this.onData(chunk)));
    this.unsubscribe.push(linkPort.onError((message) => this.fail(message)));
    this.unsubscribe.push(linkPort.onClose((reason) => {
      void this.stopBench(`disconnect: ${reason}`);
    }));
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
      role: this.role,
      uid: this.uid === null ? null : Uint8Array.from(this.uid),
      bench: {
        ...this.benchState,
        frameCounts: { ...this.benchState.frameCounts },
        requirements: { ...this.benchState.requirements },
        telemetryKinds: [...this.benchState.telemetryKinds],
        channels: this.benchState.channels === null ? null : [...this.benchState.channels],
      },
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
      if (this.benchState.running) {
        const name = BENCH_FRAME_NAMES[frame.type] ?? `0x${frame.type.toString(16).padStart(2, '0')}`;
        this.benchState.frameCounts[name] = (this.benchState.frameCounts[name] ?? 0) + 1;
        if (frame.type === FrameType.RC_CHANNELS_PACKED && frame.payload.length >= 22) {
          this.benchState.rcFramesCaptured++;
          this.benchState.channels = unpackChannels(frame.payload);
        }
        if (frame.type === FrameType.LINK_STATISTICS) this.benchState.linkStatistics++;
      }
      if (t !== null) {
        if (t.kind === 'link') {
          this.link = t.value;
          this.linkAt = this.now();
        } else {
          this.telemetry[t.kind] = t;
        }
        if (this.benchState.running && !this.benchState.telemetryKinds.includes(t.kind)) {
          this.benchState.telemetryKinds.push(t.kind);
        }
        touched = true;
      }
    }
    if (this.benchState.running) {
      this.syncBenchCounters();
      if (this.now() - this.benchLastPublish >= 100) {
        this.benchLastPublish = this.now();
        touched = true;
      }
    }
    if (touched) this.changed();
  }

  private counters(): BenchCounters {
    return {
      frames: this.decoder.frames,
      bytes: this.decoder.bytesReceived,
      crcErrors: this.decoder.crcErrors,
      resyncs: this.decoder.resyncs,
      framingErrors: this.decoder.framingErrors,
    };
  }

  private syncBenchCounters(): void {
    const current = this.counters();
    this.benchState.crcErrors = current.crcErrors - this.benchBaseline.crcErrors;
    this.benchState.resyncs = current.resyncs - this.benchBaseline.resyncs;
    this.benchState.framingErrors = current.framingErrors - this.benchBaseline.framingErrors;
  }

  private resetBench(mode: 'drive' | 'capture', durationSec: number, hz: number): void {
    const now = this.now();
    this.benchState = {
      ...idleBench(),
      running: true,
      mode,
      startedAtMs: now,
      endsAtMs: now + Math.max(0.1, durationSec) * 1000,
      requestedHz: hz,
      result: 'RUNNING',
      summary: mode === 'drive' ? 'Continuous drive is running.' : 'Listening for receiver CRSF frames.',
    };
    this.benchStartedAt = now;
    this.benchDeadline = this.benchState.endsAtMs!;
    this.benchBaseline = this.counters();
    this.benchDriveArmed = false;
    this.benchLastPublish = 0;
  }

  private clearBenchTimer(): void {
    if (this.benchTimer !== null) clearTimeout(this.benchTimer);
    this.benchTimer = null;
  }

  private async writeSafeDisarm(): Promise<boolean> {
    try {
      const safe = neutralChannels();
      if (this.benchDriveArmed) {
        safe[5] = CHANNEL_MAX;
        await this.linkPort.write(encodeRcChannels(safe));
        await new Promise((resolve) => setTimeout(resolve, 20));
      }
      safe[5] = CHANNEL_MIN;
      await this.linkPort.write(encodeRcChannels(safe));
      this.benchState.safeDisarmSent = true;
      return true;
    } catch (error) {
      this.benchState.safeDisarmSent = false;
      this.benchState.error = error instanceof Error ? error.message : String(error);
      return false;
    }
  }

  /** Start a bounded TX drive. The caller must have enabled writes and shown a bench confirmation. */
  async startBenchDrive(options: {
    readonly hz?: number;
    readonly durationSec?: number;
    readonly pattern?: 'disarmed' | 'arm-test' | 'channel-test';
    readonly armHoldSec?: number;
  } = {}): Promise<void> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    const hz = Math.max(1, Math.min(500, options.hz ?? 150));
    const durationSec = Math.max(0.1, Math.min(300, options.durationSec ?? 10));
    const pattern = options.pattern ?? 'disarmed';
    const armHoldSec = Math.max(0.55, options.armHoldSec ?? 0.75);
    if (pattern !== 'disarmed' && durationSec <= armHoldSec) {
      throw new Error('drive duration must exceed the arm-hold interval');
    }
    await this.stopBench('replaced by a new drive');
    this.benchPattern = pattern;
    this.benchArmHoldSec = armHoldSec;
    this.resetBench('drive', durationSec, hz);
    this.changed();
    const periodMs = 1000 / hz;
    const tick = async () => {
      if (!this.benchState.running || this.benchWriting) return;
      const now = this.now();
      if (now >= this.benchDeadline) {
        await this.stopBench('drive timeout');
        return;
      }
      this.benchWriting = true;
      try {
        const elapsed = (now - this.benchStartedAt) / 1000;
        const armed = pattern !== 'disarmed';
        this.benchDriveArmed = armed;
        await this.linkPort.write(encodeRcChannels(driveChannels(pattern, elapsed, armHoldSec)));
        this.benchState.framesSent++;
        this.syncBenchCounters();
      } catch (error) {
        this.benchState.error = error instanceof Error ? error.message : String(error);
        await this.stopBench('drive write failed');
        return;
      } finally {
        this.benchWriting = false;
      }
      if (this.benchState.running) {
        this.benchTimer = setTimeout(() => { void tick(); }, periodMs);
        if (this.now() - this.benchLastPublish >= 100) {
          this.benchLastPublish = this.now();
          this.changed();
        }
      }
    };
    await tick();
  }

  /** Start a bounded RX capture, optionally sending one validated telemetry sequence. */
  async startBenchCapture(options: {
    readonly durationSec?: number;
    readonly injectTelemetry?: boolean;
  } = {}): Promise<void> {
    if (options.injectTelemetry && !this.allowWrites) throw new RadioWriteRefused();
    const durationSec = Math.max(0.1, Math.min(300, options.durationSec ?? 10));
    await this.stopBench('replaced by a new capture');
    this.resetBench('capture', durationSec, 0);
    if (options.injectTelemetry) await this.injectBenchTelemetry();
    this.benchTimer = setTimeout(() => { void this.stopBench('capture timeout'); }, durationSec * 1000);
    this.changed();
  }

  /** Send a bounded representative telemetry sequence. Sent is not verification. */
  async injectBenchTelemetry(): Promise<void> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    const frames = encodeDefaultTelemetryFrames();
    for (const frame of frames) {
      await this.linkPort.write(frame);
      this.benchState.telemetryInjected++;
      await new Promise((resolve) => setTimeout(resolve, 20));
    }
    this.benchState.requirements.telemetry_injection = 'SENT / NOT VERIFIED';
    this.changed();
  }

  async stopBench(reason = 'stopped'): Promise<void> {
    const wasRunning = this.benchState.running;
    const mode = this.benchState.mode;
    this.clearBenchTimer();
    if (mode === 'drive' && (wasRunning || this.benchDriveArmed)) {
      await this.writeSafeDisarm();
    }
    if (wasRunning) {
      this.syncBenchCounters();
      this.benchState.running = false;
      this.benchState.endsAtMs = this.now();
      if (mode === 'drive') {
        this.benchState.requirements.rc_drive = this.benchState.framesSent > 0 ? 'PASS' : 'FAIL';
        this.benchState.observedHz = this.benchState.framesSent /
          Math.max(0.001, (this.now() - this.benchStartedAt) / 1000);
        this.benchState.requirements.link_statistics =
          this.benchState.linkStatistics > 0 ? 'PASS' : 'NOT TESTED';
        this.benchState.requirements.telemetry =
          this.benchState.telemetryKinds.length > 0 ? 'PASS' : 'NOT TESTED';
        this.benchState.requirements.rx_rc_capture = 'NOT TESTED';
        this.benchState.requirements.rf_link = 'NOT TESTED';
        this.benchState.result = this.benchState.framesSent > 0 ? 'PASS' : 'FAIL';
        this.benchState.summary =
          `TX drive ${reason}: ${this.benchState.framesSent} frames sent; ` +
          `safe disarm ${this.benchState.safeDisarmSent ? 'confirmed' : 'not confirmed'}. RF link remains NOT TESTED here.`;
      } else {
        this.benchState.requirements.rx_rc_capture =
          this.benchState.rcFramesCaptured > 0 ? 'PASS' : 'FAIL';
        this.benchState.observedHz = this.benchState.rcFramesCaptured /
          Math.max(0.001, (this.now() - this.benchStartedAt) / 1000);
        this.benchState.requirements.link_statistics =
          this.benchState.linkStatistics > 0 ? 'PASS' : 'NOT TESTED';
        this.benchState.requirements.rf_link =
          this.benchState.rcFramesCaptured > 0 && this.benchState.linkStatistics > 0
            ? 'PASS'
            : 'NOT TESTED';
        this.benchState.result =
          this.benchState.rcFramesCaptured > 0 ? 'PASS' : 'FAIL';
        this.benchState.summary =
          `RX capture ${reason}: ${this.benchState.rcFramesCaptured} RC frames; ` +
          `${this.benchState.linkStatistics} link-statistics frames.`;
      }
      this.changed();
    }
  }

  benchEvidence(): Record<string, unknown> {
    const s = this.snapshot().bench;
    return {
      openedAtMs: this.now(),
      role: this.role,
      device: this.device,
      bench: s,
      link: this.link,
      telemetry: this.telemetry,
      decoder: this.counters(),
    };
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
      (f) => {
        const info = decodeDeviceInfo(f);
        return f.type === FrameType.DEVICE_INFO &&
          (info?.origin === Address.CRSF_TRANSMITTER ||
            info?.origin === Address.CRSF_RECEIVER ||
            info?.origin === Address.RADIO_TRANSMITTER);
      },
      'a device ping'));
    this.device = decodeDeviceInfo(frame);
    const origin = this.device?.origin;
    this.targetAddress = origin === Address.CRSF_RECEIVER
      ? Address.CRSF_RECEIVER
      : origin === Address.CRSF_TRANSMITTER || origin === Address.RADIO_TRANSMITTER
        ? Address.CRSF_TRANSMITTER
        : Address.RADIO_TRANSMITTER;
    this.role = this.targetAddress === Address.CRSF_RECEIVER ? 'rx' : 'tx';
    this.error = null;
    this.changed();
    return this.device!;
  }

  /** One field, all of its chunks. */
  async readParameter(field: number): Promise<Parameter> {
    const frame = await this.retry(3, () => this.request(
      encodeParameterRead(this.targetAddress, field, 0),
      (f) => decodeParameterChunk(f)?.field === field, `field ${field} chunk 0`));
    const p = await this.parameterFromReply(field, frame);
    this.parameters.set(field, p);
    this.changed();
    return p;
  }

  /** Reassemble a parameter reply that may continue across later chunks. */
  private async parameterFromReply(field: number, first: CrsfFrame): Promise<Parameter> {
    const initial = decodeParameterChunk(first);
    if (initial === null || initial.field !== field) {
      throw new Error(`field ${field} returned a malformed parameter reply`);
    }
    const data = [...initial.data];
    let remaining = initial.chunksRemaining;
    for (let chunk = 1; remaining > 0; chunk++) {
      if (chunk > 32) throw new Error(`field ${field} never finished`);
      const frame = await this.retry(3, () => this.request(
        encodeParameterRead(this.targetAddress, field, chunk),
        (f) => decodeParameterChunk(f)?.field === field, `field ${field} chunk ${chunk}`));
      const part = decodeParameterChunk(frame)!;
      data.push(...part.data);
      remaining = part.chunksRemaining;
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
    await this.linkPort.write(encodeParameterWrite(this.targetAddress, field, parameterValueBytes(type, value)));
    return this.readParameter(field);
  }

  /**
   * Run a command: click, then poll it until it is idle.
   *
   * Some commands ask the module itself for confirmation. `acceptPrompt`
   * sends that confirmation only after the caller has explicitly confirmed
   * the action with the person operating the configurator.
   */
  async runCommand(field: number, maxPolls = 20, acceptPrompt = false): Promise<Parameter> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    const p = this.parameters.get(field);
    if (p?.kind !== 'command') throw new Error(`field ${field} is not a command`);
    const step = async (s: number) => this.request(
      encodeParameterWrite(this.targetAddress, field, Uint8Array.of(s)),
      (f) => decodeParameterChunk(f)?.field === field, `${p.name}`);
    let frame = await step(CommandStep.CLICK);
    for (let i = 0; i < maxPolls; i++) {
      const now = await this.parameterFromReply(field, frame);
      this.parameters.set(field, now);
      this.changed();
      if (now.kind !== 'command' || now.step === CommandStep.IDLE) return now;
      if (now.step === CommandStep.ASK_CONFIRM) {
        if (!acceptPrompt) return now;     // the generic UI asks the person, then calls confirm()
        frame = await step(CommandStep.CONFIRMED);
        const accepted = await this.parameterFromReply(field, frame);
        this.parameters.set(field, accepted);
        this.changed();
        return accepted;
      }
      frame = await step(CommandStep.QUERY);
    }
    throw new Error(`${p.name} did not finish`);
  }

  async confirm(field: number): Promise<Parameter> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    await this.linkPort.write(encodeParameterWrite(this.targetAddress, field, Uint8Array.of(CommandStep.CONFIRMED)));
    return this.readParameter(field);
  }

  /** Read the UID through ExpressLRS's MSP RXTX configuration channel. */
  async readUid(): Promise<Uint8Array> {
    const frame = await this.request(
      encodeMspV2Request(this.targetAddress, Uint8Array.of(RxtxConfigSubcommand.UID)),
      (f) => {
        const response = decodeMspResponse(f);
        return response?.functionCode === MSP_ELRS_RXTX_CONFIG &&
          response.payload[0] === RxtxConfigSubcommand.UID;
      },
      'the module UID',
    );
    const response = decodeMspResponse(frame);
    if (response === null || response.payload.length !== 7) {
      throw new Error('the module returned a malformed UID');
    }
    this.uid = response.payload.slice(1);
    this.changed();
    return Uint8Array.from(this.uid);
  }

  /**
   * Set a binding phrase. ExpressLRS stores the MD5-derived UID, not the
   * phrase, and sends no completion reply; reconnect and read the UID to
   * verify it.
   */
  async setBindingPhrase(phrase: string): Promise<void> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    const bytes = new TextEncoder().encode(phrase);
    await this.linkPort.write(encodeMspWrite(
      this.targetAddress,
      Uint8Array.from([RxtxConfigSubcommand.BIND_PHRASE, ...bytes]),
    ));
    this.uid = null;
    this.changed();
  }

  /** Set a complete six-byte UID directly. */
  async setUid(uid: Uint8Array): Promise<void> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    if (uid.length !== 6) throw new RangeError('an ExpressLRS UID must be six bytes');
    await this.linkPort.write(encodeMspWrite(
      this.targetAddress,
      Uint8Array.from([RxtxConfigSubcommand.UID, ...uid]),
    ));
    this.uid = null;
    this.changed();
  }

  private command(names: readonly string[]): Parameter {
    const wanted = new Set(names);
    const found = [...this.parameters.values()].find(
      (p) => p.kind === 'command' && wanted.has(p.name),
    );
    if (found === undefined) {
      throw new Error(`this module does not expose ${names.join(' or ')}`);
    }
    return found;
  }

  /**
   * Send ExpressLRS's direct CRSF bind command (COMMAND 0x10/0x01).
   *
   * The command has no completion reply. A display changing to binding mode
   * proves only that the request was sent; peer acquisition needs RF evidence.
   */
  async bind(): Promise<void> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    await this.linkPort.write(encodeCrsfBindCommand(this.targetAddress));
  }

  /** Enable the module's native Wi-Fi, when the detected module exposes it. */
  async enableModuleWifi(): Promise<Parameter> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    return this.runCommand(this.command(['Enable WiFi']).id, 20, true);
  }

  /** Ask a connected TX to put the receiver into Wi-Fi update mode. */
  async enableReceiverWifi(): Promise<Parameter> {
    if (!this.allowWrites) throw new RadioWriteRefused();
    return this.runCommand(this.command(['Enable Rx WiFi']).id);
  }

  async dispose(): Promise<void> {
    await this.stopBench('session disposed');
    for (const u of this.unsubscribe) u();
    this.waiters.clear();
    this.listeners.clear();
  }
}
