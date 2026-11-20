import { Button, Callout, HTMLSelect, InputGroup, ProgressBar, Switch, Tag } from '@blueprintjs/core';
import { useCallback, useEffect, useRef, useState } from 'react';
import { CommandStep, type Parameter } from '../protocol/crsf';
import { bindingUid, isUnboundUid, parseUid, uidText } from '../protocol/elrs-binding';
import { RadioSession, type BenchRequirementStatus, type RadioSnapshot } from '../session/radio';
import { DemoRadioTransport } from '../transport/demo-radio';
import type { Transport } from '../transport/types';
import { WebSerialTransport } from '../transport/webserial';
import { Panel } from './panels';

/** Logical native-USB CRSF rates for the Aerial Kit Rev A3 endpoints. */
export const RADIO_BAUD = 400000;
export const RADIO_RX_BAUD = 420000;

type RadioKind = 'serial-tx' | 'serial-rx' | 'demo' | 'demo-rx';
interface PhraseEvidence {
  readonly phrase: string;
  readonly tx: string | null;
  readonly rx: string | null;
}

const RF_MODES: Record<number, string> = { 0: '4 Hz', 1: '50 Hz', 2: '150 Hz', 3: '250 Hz', 4: '500 Hz' };
const MANAGED_RADIO_COMMANDS = new Set(['Bind', 'Enter Bind Mode', 'Enable WiFi', 'Enable Rx WiFi']);

function version(v: number): string {
  return `${(v >> 16) & 0xff}.${(v >> 8) & 0xff}.${v & 0xff}`;
}

function paramsFor(snapshot: RadioSnapshot | null): Parameter[] {
  return snapshot === null ? [] : [...snapshot.parameters.values()];
}

/**
 * The radio side of the configurator: a radio module (ExpressLRS TX or RX)
 * is its own device on its own port, so it has its own connection and page,
 * separate from the flight controller's.
 */
export function RadioApp({ serialAvailable }: { serialAvailable: boolean }) {
  const [kind, setKind] = useState<RadioKind>(serialAvailable ? 'serial-tx' : 'demo');
  const [session, setSession] = useState<RadioSession | null>(null);
  const [snapshot, setSnapshot] = useState<RadioSnapshot | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [allow, setAllow] = useState(false);
  const [phrase, setPhrase] = useState('');
  const [uidInput, setUidInput] = useState('');
  const [confirmAction, setConfirmAction] = useState<'bind' | 'phrase' | 'uid' | 'wifi' | 'rx-wifi' | null>(null);
  const [bindingMessage, setBindingMessage] = useState<string | null>(null);
  const [phraseEvidence, setPhraseEvidence] = useState<PhraseEvidence | null>(null);
  const [benchPattern, setBenchPattern] = useState<'disarmed' | 'arm-test' | 'channel-test'>('disarmed');
  const [benchHz, setBenchHz] = useState(150);
  const [benchDriveSec, setBenchDriveSec] = useState(10);
  const [benchCaptureSec, setBenchCaptureSec] = useState(10);
  const [benchArmConfirmed, setBenchArmConfirmed] = useState(false);
  const [benchInject, setBenchInject] = useState(true);
  const transport = useRef<Transport | null>(null);
  const [, tick] = useState(0);

  useEffect(() => {
    const t = setInterval(() => tick((n) => n + 1), 500);   // link age ticks over while idle
    return () => clearInterval(t);
  }, []);

  const disconnect = useCallback(async () => {
    await session?.dispose();
    setSession(null);
    setSnapshot(null);
    setAllow(false);
    await transport.current?.close();
    transport.current = null;
  }, [session]);

  useEffect(() => {
    const onBeforeUnload = () => { void session?.stopBench('navigation'); };
    window.addEventListener('beforeunload', onBeforeUnload);
    return () => {
      window.removeEventListener('beforeunload', onBeforeUnload);
      void session?.stopBench('navigation');
    };
  }, [session]);

  const connect = useCallback(async () => {
    setBusy(true);
    setError(null);
    try {
      const t: Transport = kind === 'demo' || kind === 'demo-rx'
        ? new DemoRadioTransport(kind === 'demo-rx' ? { role: 'rx' } : {})
        : new WebSerialTransport(true, kind === 'serial-rx' ? RADIO_RX_BAUD : RADIO_BAUD);
      await t.open();
      transport.current = t;
      const s = new RadioSession(t, kind === 'demo' || kind === 'demo-rx' ? 300 : 800);
      s.subscribe(setSnapshot);
      setSession(s);
      await s.identify();
      await s.readAll();
      try {
        await s.readUid();
      } catch {
        // RXTX configuration is optional; the menu remains usable when a
        // module build does not answer the UID query.
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  }, [kind]);

  useEffect(() => {
    if (session !== null) session.allowWrites = allow;
  }, [allow, session]);

  const act = async (fn: () => Promise<unknown>) => {
    setBusy(true);
    setError(null);
    try {
      await fn();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const command = (names: readonly string[]) => paramsFor(snapshot).find(
    (p) => p.kind === 'command' && names.includes(p.name),
  );

  const confirmed = async () => {
    if (session === null || confirmAction === null) return;
    const action = confirmAction;
    setConfirmAction(null);
    setBusy(true);
    setError(null);
    try {
      if (action === 'bind') {
        await session.bind();
        setBindingMessage(
          'CRSF bind request sent to this endpoint only. Keep the other endpoint in bind mode during the same window; a sent command is not proof of binding.',
        );
      } else if (action === 'phrase') {
        const actual = await session.setBindingPhrase(phrase);
        const actualText = uidText(actual);
        const role = session.snapshot().role;
        setPhraseEvidence((previous) => {
          const base = previous?.phrase === phrase ? previous : { phrase, tx: null, rx: null };
          return {
            ...base,
            ...(role === 'tx' ? { tx: actualText } : {}),
            ...(role === 'rx' ? { rx: actualText } : {}),
          };
        });
        setBindingMessage(
          `Verified ${role === 'unknown' ? 'module' : role.toUpperCase()} phrase readback: ${actualText}. ` +
          'Connect the other endpoint and set the same phrase before a two-board RF test.',
        );
        setPhrase('');
      } else if (action === 'uid') {
        const actual = await session.setUid(parseUid(uidInput));
        setBindingMessage(`Verified direct UID readback: ${uidText(actual)}.`);
        setUidInput('');
      } else if (action === 'wifi') {
        const result = await session.enableModuleWifi();
        setBindingMessage(`Wi-Fi request sent${result.kind === 'command' && result.info ? `: ${result.info}` : '; Wi-Fi availability is not confirmed'}`);
      } else {
        const result = await session.enableReceiverWifi();
        setBindingMessage(`Receiver Wi-Fi request sent${result.kind === 'command' && result.info ? `: ${result.info}` : '; receiver Wi-Fi is not confirmed'}`);
      }
    } catch (e) {
      setBindingMessage(`Request may have reached the module, but no confirmation was established: ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      setBusy(false);
    }
  };

  if (session === null) {
    return (
      <main className="page radio-page">
        <Panel title="Radio module" note="ExpressLRS TX or RX, configured over CRSF">
          <p>
            A radio module is a separate device from the flight controller: it has its own port and its
            own settings (packet rate, telemetry ratio, power, binding). This page talks to it the way a
            radio's ExpressLRS script does.
          </p>
          <p className="bp5-text-muted">
            The Aerial Kit ELRS USB TX Rev A3 uses its native USB-CDC endpoint: logical {RADIO_BAUD} baud for
            a TX image and {RADIO_RX_BAUD} for an RX image. USB CDC may ignore the requested rate. GPIO4 is
            sampled at boot: low selects TX and high selects RX. Verify the electrical switch position; do not
            rely on the printed label or change it while powered.
          </p>
          {error !== null && <Callout intent="danger">{error}</Callout>}
          <div className="connectbar">
            <HTMLSelect aria-label="Radio connection" value={kind} disabled={busy}
              onChange={(e) => setKind(e.currentTarget.value as RadioKind)}>
              <option value="serial-tx" disabled={!serialAvailable}>
                USB serial TX ({RADIO_BAUD} baud){serialAvailable ? '' : ' (not available in this browser)'}
              </option>
              <option value="serial-rx" disabled={!serialAvailable}>
                USB serial RX ({RADIO_RX_BAUD} baud){serialAvailable ? '' : ' (not available in this browser)'}
              </option>
              <option value="demo">Demo radio</option>
              <option value="demo-rx">Demo RX radio</option>
            </HTMLSelect>
            <Button intent="primary" icon="link" loading={busy}
              disabled={busy || (!kind.startsWith('demo') && !serialAvailable)}
              onClick={() => void connect()}>
              Connect radio
            </Button>
          </div>
        </Panel>
      </main>
    );
  }

  const s = snapshot;
  const params = s === null ? [] : [...s.parameters.values()];
  const top = params.filter((p) => p.parent === 0 && !p.hidden);
  const childrenOf = (id: number) => params.filter((p) => p.parent === id && !p.hidden);
  const fresh = s?.link !== null && (s?.linkAgeMs ?? Infinity) < 1500;
  const bindCommand = command(['Bind', 'Enter Bind Mode']);
  const moduleWifi = command(['Enable WiFi']);
  const receiverWifi = command(['Enable Rx WiFi']);
  const currentUid = s?.uid === null || s?.uid === undefined
    ? 'not read'
    : isUnboundUid(s.uid) ? 'unbound (all zero UID)' : uidText(s.uid);
  const phrasePreview = phrase === '' ? 'unbound (all zero UID)' : uidText(bindingUid(phrase));
  const roleName = s?.role === 'rx' ? 'RX receiver' : s?.role === 'tx' ? 'TX transmitter' : 'radio module';
  const phrasePairReady =
    phraseEvidence?.tx !== null &&
    phraseEvidence?.rx !== null &&
    phraseEvidence?.tx === phraseEvidence?.rx;

  const row = (p: Parameter) => {
    switch (p.kind) {
      case 'selection':
        return (
          <div className="radio-row" key={p.id}>
            <label htmlFor={`radio-p${p.id}`}>{p.name}</label>
            <HTMLSelect id={`radio-p${p.id}`} value={p.value} disabled={!allow || busy}
              onChange={(e) => void act(() => session.write(p.id, Number(e.currentTarget.value)))}>
              {p.options.map((o, i) => (i >= p.min && i <= p.max ? <option key={i} value={i}>{o}{p.units}</option> : null))}
            </HTMLSelect>
          </div>
        );
      case 'number':
        return (
          <div className="radio-row" key={p.id}>
            <span>{p.name}</span>
            <span>{p.value}{p.units} <span className="bp5-text-muted">({p.min}..{p.max})</span></span>
          </div>
        );
      case 'command':
        if (MANAGED_RADIO_COMMANDS.has(p.name)) {
          return (
            <div className="radio-row" key={p.id}>
              <span>{p.name}</span>
              <span className="bp5-text-muted">managed in Binding and Wi-Fi above</span>
            </div>
          );
        }
        return (
          <div className="radio-row" key={p.id}>
            <span>{p.name}{p.info !== '' && <span className="bp5-text-muted"> — {p.info}</span>}</span>
            {p.step === CommandStep.ASK_CONFIRM ? (
              <Button small intent="warning" disabled={!allow || busy} onClick={() => void act(() => session.confirm(p.id))}>
                Confirm
              </Button>
            ) : (
              <Button small disabled={!allow || busy} onClick={() => void act(() => session.runCommand(p.id))}>
                Run
              </Button>
            )}
          </div>
        );
      case 'info':
      case 'string':
        return (
          <div className="radio-row" key={p.id}><span>{p.name}</span><span>{p.value}</span></div>
        );
      case 'folder':
        return (
          <fieldset className="radio-folder" key={p.id}>
            <legend>{p.name}</legend>
            {childrenOf(p.id).map(row)}
          </fieldset>
        );
      default:
        return (
          <div className="radio-row" key={p.id}><span>{p.name}</span><span className="bp5-text-muted">not shown here</span></div>
        );
    }
  };

  return (
    <main className="page radio-page">
      {error !== null && <Callout intent="danger">{error}</Callout>}
      <Panel title={s?.device?.name ?? 'Radio module'}
        note={s?.device ? `firmware ${version(s.device.softwareVersion)} · ${s.device.parameterCount} settings` : 'identifying…'}
        actions={<Button icon="offline" onClick={() => void disconnect()}>Disconnect</Button>}>
        <div className="radio-link" aria-label="Link">
          {fresh && s?.link ? (
            <>
              <Tag intent={s.link.uplinkLinkQuality > 70 ? 'success' : 'warning'}>LQ {s.link.uplinkLinkQuality}%</Tag>
              <span>RSSI {s.link.uplinkRssi1Dbm} / {s.link.uplinkRssi2Dbm} dBm</span>
              <span>SNR {s.link.uplinkSnrDb} dB</span>
              <span>rate {RF_MODES[s.link.rfMode] ?? `mode ${s.link.rfMode}`}</span>
              <span>downlink LQ {s.link.downlinkLinkQuality}%</span>
            </>
          ) : (
            <Tag minimal>{s?.link ? `no link for ${Math.round((s.linkAgeMs ?? 0) / 1000)} s` : 'no receiver connected'}</Tag>
          )}
        </div>
      </Panel>
      <Panel title="Binding and Wi-Fi" note={`${roleName} · these actions change the radio identity or stop the current link`}>
        {bindingMessage !== null && <Callout intent="warning">{bindingMessage}</Callout>}
        <div className="radio-row binding-row">
          <span>Current module UID</span>
          <span><code>{currentUid}</code></span>
        </div>
        {s?.uidError !== null && s?.uidError !== undefined && (
          <Callout intent="warning">
            {s.uidError} Check the endpoint, cable and decoded frame counters before retrying.
          </Callout>
        )}
        <div className="radio-row binding-row">
          <Button small icon="refresh" disabled={busy} onClick={() => void act(async () => {
            const uid = await session.readUid();
            setBindingMessage(`Read ${uidText(uid)} from the connected endpoint.`);
          })}>
            Read UID
          </Button>
          <span className="bp5-text-muted">The phrase is not stored; the module stores the six-byte UID derived from it.</span>
        </div>
        <div className="radio-row binding-row">
          <label htmlFor="binding-phrase">Binding phrase</label>
          <InputGroup id="binding-phrase" value={phrase} disabled={!allow || busy}
            onChange={(e) => setPhrase(e.currentTarget.value)}
            placeholder="shared phrase for both endpoints" fill />
        </div>
        <div className="radio-row binding-row">
          <span>Derived UID</span>
          <code>{phrasePreview}</code>
        </div>
        <div className="radio-row binding-row">
          <Button small disabled={!allow || busy || phrase === ''} onClick={() => setConfirmAction('phrase')}>
            Set phrase
          </Button>
          <span className="bp5-text-muted">Writes the MD5-derived UID and reboots the module. Readback is required.</span>
        </div>
        <p className="small muted">
          Set the phrase on RX first, then TX (or the reverse), reconnecting each endpoint independently.
          Matching phrase configuration is safer than an unbounded bind window. RF testing stays gated until
          both readbacks below are present and identical.
        </p>
        <div className="radio-row binding-row">
          <span>Pair evidence</span>
          <span>
            TX <code>{phraseEvidence?.tx ?? 'not verified'}</code>
            {' · '}RX <code>{phraseEvidence?.rx ?? 'not verified'}</code>
          </span>
        </div>
        {phrasePairReady && (
          <Callout intent="success">
            TX and RX phrase-derived UIDs match. Reconnect both boards, confirm both UIDs again, then run the
            two-board RF test.
          </Callout>
        )}
        <div className="radio-row binding-row">
          <label htmlFor="binding-uid">Direct UID</label>
          <InputGroup id="binding-uid" value={uidInput} disabled={!allow || busy}
            onChange={(e) => setUidInput(e.currentTarget.value)}
            placeholder="AA:BB:CC:DD:EE:FF" fill />
        </div>
        <div className="radio-row binding-row">
          <Button small disabled={!allow || busy || uidInput === ''} onClick={() => setConfirmAction('uid')}>
            Set UID
          </Button>
          <span className="bp5-text-muted">Six hexadecimal bytes. There is no phrase stored to recover later.</span>
        </div>
        <div className="radio-row binding-row">
          <Button small intent="warning" disabled={!allow || busy || s?.role === 'unknown'}
            onClick={() => setConfirmAction('bind')}>
            {s?.role === 'rx' ? 'Enter Bind Mode' : 'Start binding mode'}
          </Button>
          <span className="bp5-text-muted">
            Sends CRSF COMMAND 0x10/0x01. {bindCommand === undefined
              ? 'This parameter menu does not advertise a bind entry, but the stock command remains available.'
              : `The menu advertises ${bindCommand.name}.`}
            {' '}This is separate from setting a phrase or UID. RX must show binding-mode breathing; start TX
            while RX is still in that state, then require LINK_STATISTICS from both endpoints.
          </span>
        </div>
        {confirmAction !== null && (
          <Callout intent="danger" title={
            confirmAction === 'bind'
              ? 'Confirm bind request'
              : confirmAction === 'wifi' || confirmAction === 'rx-wifi'
                ? 'Confirm Wi-Fi request'
                : 'Confirm radio identity change'
          }>
            <p>
              {confirmAction === 'bind'
                ? 'Enter explicit binding mode now? The RF link may drop while the module searches for a peer.'
                : confirmAction === 'wifi' || confirmAction === 'rx-wifi'
                  ? 'Enable Wi-Fi now? The radio link may stop while the module enters its update mode.'
                  : 'Write this identity now? The module will reboot and the current link will drop.'}
            </p>
            <Button small intent="danger" disabled={busy} onClick={() => void confirmed()}>
              Confirm
            </Button>{' '}
            <Button small disabled={busy} onClick={() => setConfirmAction(null)}>Cancel</Button>
          </Callout>
        )}
        <div className="radio-row binding-row">
          {moduleWifi !== undefined && (
            <Button small disabled={!allow || busy} onClick={() => setConfirmAction('wifi')}>
              Enable module Wi-Fi
            </Button>
          )}
          {receiverWifi !== undefined && (
            <Button small disabled={!allow || busy} onClick={() => setConfirmAction('rx-wifi')}>
              Enable receiver Wi-Fi
            </Button>
          )}
          {moduleWifi === undefined && receiverWifi === undefined && (
            <span className="bp5-text-muted">This module did not advertise a Wi-Fi command.</span>
          )}
        </div>
        {(moduleWifi !== undefined || receiverWifi !== undefined) && (
          <p className="small muted">
            The module's native page is not embedded here. Join <code>ExpressLRS TX</code> or
            {' '}<code>ExpressLRS RX</code> with password <code>expresslrs</code>, then open
            {' '}<a href="http://10.0.0.1/" target="_blank" rel="noreferrer">http://10.0.0.1/</a>.
            The SSID depends on the role image.
          </p>
        )}
      </Panel>
      <Panel title="Settings" note="read from the module"
        actions={<Switch checked={allow} label="Allow changes" onChange={(e) => setAllow(e.currentTarget.checked)} />}>
        {s?.loading && <ProgressBar value={s.loading.total ? s.loading.done / s.loading.total : 0} />}
        {!allow && <p className="bp5-text-muted">Read-only. Turn on “Allow changes” to set values or run Bind.</p>}
        <div className="radio-menu">{top.map(row)}</div>
      </Panel>
      <Panel title="Bench" note="bounded native-USB CRSF drive, capture and evidence">
        <Callout intent="primary">
          Color identifies the role: TX is blue and RX is green. This Bench page never treats a sent frame as RF
          proof. LINK_STATISTICS and receiver RC frames must be observed before the RF link can pass.
        </Callout>
        {s?.role === 'tx' && (
          <div className="bench-controls">
            <label>
              Stream rate
              <InputGroup type="number" value={String(benchHz)} min={1} max={500}
                onChange={(e) => setBenchHz(Math.max(1, Math.min(500, Number(e.currentTarget.value))))} />
            </label>
            <label>
              Duration seconds
              <InputGroup type="number" value={String(benchDriveSec)} min={1} max={300}
                onChange={(e) => setBenchDriveSec(Math.max(1, Math.min(300, Number(e.currentTarget.value))))} />
            </label>
            <label>
              Pattern
              <HTMLSelect aria-label="Pattern" value={benchPattern}
                onChange={(e) => {
                  setBenchPattern(e.currentTarget.value as typeof benchPattern);
                  setBenchArmConfirmed(false);
                }}>
                <option value="disarmed">Disarmed centered</option>
                <option value="arm-test">Arm hold then low throttle</option>
                <option value="channel-test">Arm hold then channel test</option>
              </HTMLSelect>
            </label>
            {benchPattern !== 'disarmed' && (
              <Switch checked={benchArmConfirmed}
                label="I confirm the controlled arm sequence and safe area"
                onChange={(e) => setBenchArmConfirmed(e.currentTarget.checked)} />
            )}
            <div className="bench-buttons">
              <Button intent="primary"
                disabled={!allow || busy || s?.bench.running || (benchPattern !== 'disarmed' && !benchArmConfirmed)}
                onClick={() => void act(() => session.startBenchDrive({
                  hz: benchHz,
                  durationSec: benchDriveSec,
                  pattern: benchPattern,
                  armHoldSec: 0.75,
                }))}>
                Start continuous drive
              </Button>
              <Button intent="danger" disabled={!s?.bench.running}
                onClick={() => void act(() => session.stopBench('operator stop'))}>
                Stop and disarm
              </Button>
            </div>
          </div>
        )}
        {s?.role === 'rx' && (
          <div className="bench-controls">
            <label>
              Capture seconds
              <InputGroup type="number" value={String(benchCaptureSec)} min={1} max={300}
                onChange={(e) => setBenchCaptureSec(Math.max(1, Math.min(300, Number(e.currentTarget.value))))} />
            </label>
            <Switch checked={benchInject} label="Inject representative attitude, battery, GPS and barometer telemetry"
              onChange={(e) => setBenchInject(e.currentTarget.checked)} />
            <div className="bench-buttons">
              <Button intent="primary" disabled={!allow || busy || s?.bench.running}
                onClick={() => void act(() => session.startBenchCapture({
                  durationSec: benchCaptureSec,
                  injectTelemetry: benchInject,
                }))}>
                Start capture
              </Button>
              <Button disabled={!allow || busy} onClick={() => void act(() => session.injectBenchTelemetry())}>
                Inject telemetry
              </Button>
              <Button intent="danger" disabled={!s?.bench.running}
                onClick={() => void act(() => session.stopBench('operator stop'))}>
                Stop capture
              </Button>
            </div>
          </div>
        )}
        {s?.role === 'unknown' && <p className="bp5-text-muted">Identify the radio before starting a Bench operation.</p>}
        {s?.bench && (
          <div className="bench-evidence" aria-label="Bench evidence">
            <div className="bench-buttons">
              <Tag intent={s.bench.result === 'PASS' ? 'success' : s.bench.result === 'FAIL' ? 'danger' : 'none'}>
                {s.bench.result}
              </Tag>
              <Button small icon="download" disabled={s.bench.running}
                onClick={() => {
                  const blob = new Blob([JSON.stringify(session.benchEvidence(), null, 2)], { type: 'application/json' });
                  const url = URL.createObjectURL(blob);
                  const anchor = document.createElement('a');
                  anchor.href = url;
                  anchor.download = `aerialkit-radio-bench-${Date.now()}.json`;
                  anchor.click();
                  URL.revokeObjectURL(url);
                }}>
                Download evidence
              </Button>
            </div>
            <p>{s.bench.summary}</p>
            <div className="bench-counters">
              <span>sent {s.bench.framesSent}</span>
              <span>rate {s.bench.observedHz === null ? '—' : `${s.bench.observedHz.toFixed(1)} Hz`}</span>
              <span>RC captured {s.bench.rcFramesCaptured}</span>
              <span>link stats {s.bench.linkStatistics}</span>
              <span>CRC {s.bench.crcErrors}</span>
              <span>resyncs {s.bench.resyncs}</span>
              <span>framing {s.bench.framingErrors}</span>
              <span>injected {s.bench.telemetryInjected}</span>
            </div>
            {s.bench.channels !== null && (
              <div className="bench-channels" aria-label="Captured channels">
                {s.bench.channels.map((value, index) => <span key={index}>CH{index + 1} {value}</span>)}
              </div>
            )}
            {Object.keys(s.bench.frameCounts).length > 0 && (
              <div className="bench-counters">
                {Object.entries(s.bench.frameCounts).map(([name, count]) => <span key={name}>{name} {count}</span>)}
              </div>
            )}
            {Object.keys(s.telemetry).length > 0 && (
              <div className="bench-telemetry" aria-label="Bench telemetry">
                {Object.entries(s.telemetry).map(([kind, value]) => (
                  <code key={kind}>{kind}: {JSON.stringify(value)}</code>
                ))}
              </div>
            )}
            <div className="bench-requirements">
              {Object.entries(s.bench.requirements).map(([name, status]) => (
                <span key={name}>{name}: <RequirementTag status={status} /></span>
              ))}
            </div>
            {s.bench.error !== null && <Callout intent="danger">{s.bench.error}</Callout>}
          </div>
        )}
      </Panel>
    </main>
  );
}

function RequirementTag({ status }: { status: BenchRequirementStatus }) {
  return <Tag minimal intent={
    status === 'PASS' ? 'success'
      : status === 'FAIL' ? 'danger'
        : status === 'SENT / NOT VERIFIED' ? 'warning'
          : 'none'
  }>{status}</Tag>;
}
