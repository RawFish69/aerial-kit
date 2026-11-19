import { Button, Callout, HTMLSelect, ProgressBar, Switch, Tag } from '@blueprintjs/core';
import { useCallback, useEffect, useRef, useState } from 'react';
import { CommandStep, type Parameter } from '../protocol/crsf';
import { RadioSession, type RadioSnapshot } from '../session/radio';
import { DemoRadioTransport } from '../transport/demo-radio';
import type { Transport } from '../transport/types';
import { WebSerialTransport } from '../transport/webserial';
import { Panel } from './panels';

/** An ExpressLRS module's handset port (UART0 on the Aerial Kit Rev A3, J3) runs CRSF at this rate. */
export const RADIO_BAUD = 400000;

type RadioKind = 'serial' | 'demo';

const RF_MODES: Record<number, string> = { 0: '4 Hz', 1: '50 Hz', 2: '150 Hz', 3: '250 Hz', 4: '500 Hz' };

function version(v: number): string {
  return `${(v >> 16) & 0xff}.${(v >> 8) & 0xff}.${v & 0xff}`;
}

/**
 * The radio side of the configurator: a radio module (ExpressLRS transmitter)
 * is its own device on its own port, so it has its own connection and page,
 * separate from the flight controller's.
 */
export function RadioApp({ serialAvailable }: { serialAvailable: boolean }) {
  const [kind, setKind] = useState<RadioKind>(serialAvailable ? 'serial' : 'demo');
  const [session, setSession] = useState<RadioSession | null>(null);
  const [snapshot, setSnapshot] = useState<RadioSnapshot | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [allow, setAllow] = useState(false);
  const transport = useRef<Transport | null>(null);
  const [, tick] = useState(0);

  useEffect(() => {
    const t = setInterval(() => tick((n) => n + 1), 500);   // link age ticks over while idle
    return () => clearInterval(t);
  }, []);

  const disconnect = useCallback(async () => {
    session?.dispose();
    setSession(null);
    setSnapshot(null);
    setAllow(false);
    await transport.current?.close();
    transport.current = null;
  }, [session]);

  const connect = useCallback(async () => {
    setBusy(true);
    setError(null);
    try {
      const t: Transport = kind === 'demo' ? new DemoRadioTransport() : new WebSerialTransport(true, RADIO_BAUD);
      await t.open();
      transport.current = t;
      const s = new RadioSession(t, kind === 'demo' ? 300 : 800);
      s.subscribe(setSnapshot);
      setSession(s);
      await s.identify();
      await s.readAll();
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

  if (session === null) {
    return (
      <main className="page radio-page">
        <Panel title="Radio module" note="ExpressLRS transmitter, configured over CRSF">
          <p>
            A radio module is a separate device from the flight controller: it has its own port and its
            own settings (packet rate, telemetry ratio, power, binding). This page talks to it the way a
            radio's ExpressLRS script does.
          </p>
          <p className="bp5-text-muted">
            The Aerial Kit ELRS USB TX Rev A3 takes CRSF on UART0 (J3: pin 4 board RX, pin 3 board TX, pin 2 GND)
            through a 3.3 V USB-UART adapter at {RADIO_BAUD} baud. Its own USB-C port is for flashing and does not
            carry CRSF in ExpressLRS 4.1.0.
          </p>
          {error !== null && <Callout intent="danger">{error}</Callout>}
          <div className="connectbar">
            <HTMLSelect aria-label="Radio connection" value={kind} disabled={busy}
              onChange={(e) => setKind(e.currentTarget.value as RadioKind)}>
              <option value="serial" disabled={!serialAvailable}>
                USB serial ({RADIO_BAUD} baud){serialAvailable ? '' : ' (not available in this browser)'}
              </option>
              <option value="demo">Demo radio</option>
            </HTMLSelect>
            <Button intent="primary" icon="link" loading={busy} disabled={busy || (kind === 'serial' && !serialAvailable)}
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
      <Panel title="Settings" note="read from the module"
        actions={<Switch checked={allow} label="Allow changes" onChange={(e) => setAllow(e.currentTarget.checked)} />}>
        {s?.loading && <ProgressBar value={s.loading.total ? s.loading.done / s.loading.total : 0} />}
        {!allow && <p className="bp5-text-muted">Read-only. Turn on “Allow changes” to set values or run Bind.</p>}
        <div className="radio-menu">{top.map(row)}</div>
      </Panel>
    </main>
  );
}
