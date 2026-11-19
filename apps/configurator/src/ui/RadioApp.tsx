import { Button, Callout, HTMLSelect, InputGroup, ProgressBar, Switch, Tag } from '@blueprintjs/core';
import { useCallback, useEffect, useRef, useState } from 'react';
import { CommandStep, type Parameter } from '../protocol/crsf';
import { bindingUid, isUnboundUid, parseUid, uidText } from '../protocol/elrs-binding';
import { RadioSession, type RadioSnapshot } from '../session/radio';
import { DemoRadioTransport } from '../transport/demo-radio';
import type { Transport } from '../transport/types';
import { WebSerialTransport } from '../transport/webserial';
import { Panel } from './panels';

/** An ExpressLRS module's handset port (UART0 on the Aerial Kit Rev A3, J3) runs CRSF at this rate. */
export const RADIO_BAUD = 400000;
export const RADIO_RX_BAUD = 420000;

type RadioKind = 'serial-tx' | 'serial-rx' | 'demo';

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
      const t: Transport = kind === 'demo'
        ? new DemoRadioTransport()
        : new WebSerialTransport(true, kind === 'serial-rx' ? RADIO_RX_BAUD : RADIO_BAUD);
      await t.open();
      transport.current = t;
      const s = new RadioSession(t, kind === 'demo' ? 300 : 800);
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
        setBindingMessage('CRSF bind request sent. This command has no completion reply; confirm peer acquisition from both endpoints and their link statistics.');
      } else if (action === 'phrase') {
        await session.setBindingPhrase(phrase);
        setBindingMessage('Phrase write sent. The module should reboot; reconnect and read its UID to verify the derived identity.');
        setPhrase('');
      } else if (action === 'uid') {
        await session.setUid(parseUid(uidInput));
        setBindingMessage('UID write sent. The module should reboot; reconnect and read its UID to verify it.');
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
            The Aerial Kit ELRS USB TX Rev A3 takes CRSF on UART0 (J3: pin 4 board RX, pin 3 board TX, pin 2 GND)
            through a 3.3 V USB-UART adapter: {RADIO_BAUD} baud for a TX image, {RADIO_RX_BAUD} for an RX image.
            Set SW2 to match the image and verify continuity; ExpressLRS does not select TX/RX from GPIO4 at
            runtime, and this connection choice only selects the baud. The board's USB-C port is for flashing and
            the native ExpressLRS Wi-Fi workflow, not stock CRSF configuration.
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
            </HTMLSelect>
            <Button intent="primary" icon="link" loading={busy}
              disabled={busy || (kind !== 'demo' && !serialAvailable)}
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
        <div className="radio-row binding-row">
          <Button small icon="refresh" disabled={busy} onClick={() => void act(() => session.readUid())}>
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
            {' '}This is separate from setting a phrase or UID.
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
    </main>
  );
}
