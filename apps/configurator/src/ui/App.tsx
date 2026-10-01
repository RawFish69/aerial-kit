import { Button, Card, FormGroup, H2, InputGroup, Navbar, RadioCard, Tag, type Intent } from '@blueprintjs/core';
import { Notice } from './panels';
import { useCallback, useEffect, useMemo, useState } from 'react';
import { detectFirmware, type Detection } from '../protocol/detect';
import { AerialKitSession } from '../session/aerialkit';
import { MspBoard } from '../session/foreign';
import { MavBoard } from '../session/mavlink';
import type { Session } from '../session/types';
import { BridgeTransport, DEFAULT_BRIDGE_URL } from '../transport/bridge';
import { DemoTransport } from '../transport/demo';
import { WebSerialTransport } from '../transport/webserial';
import { TransportError, type Transport, type TransportKind } from '../transport/types';
import { ArmedBanner, IdentityBadge, Limitations, Panel, type ParameterActions } from './panels';
import { ForeignWorkspace } from './foreign';
import { MavWorkspace } from './mavlink';
import { TabRail } from './TabRail';
import { useAging, useMavAging, useSession } from './useSession';

/**
 * The workspace.
 *
 * One board, one session, one page. The order of the page is the order of the
 * questions a person actually has when they plug a flight controller in: what
 * am I talking to, is it armed, what can this link not do, what does it say
 * right now, and only then — what would you like to change.
 *
 * Connecting is always a click. For Web Serial that is a hard requirement of
 * the browser, and for the others it is the right shape anyway: this app never
 * opens a device because a page loaded.
 */

const KINDS: ReadonlyArray<{ kind: TransportKind; label: string; blurb: string }> = [
  {
    kind: 'demo',
    label: 'Demo board',
    blurb: 'A simulated board inside this page. Nothing is plugged in and nothing is saved.',
  },
  {
    kind: 'serial',
    label: 'USB serial',
    blurb: 'A board on a USB cable, straight from the browser. Chrome, Edge or Opera.',
  },
  {
    kind: 'bridge',
    label: 'Local bridge',
    blurb: 'The firmware simulator, or a port another program holds, through the bridge helper.',
  },
];

export function App() {
  const [kind, setKind] = useState<TransportKind>(() => WebSerialTransport.supported() ? 'serial' : 'demo');
  const [reviewEdits, setReviewEdits] = useState(0);
  const [bridgeUrl, setBridgeUrl] = useState(DEFAULT_BRIDGE_URL);
  const [session, setSession] = useState<Session | null>(null);
  /** A board that is not ours. Mutually exclusive with `session`. */
  const [foreign, setForeign] = useState<MspBoard | null>(null);
  /** A vehicle that is not ours. Mutually exclusive with both. */
  const [mav, setMav] = useState<MavBoard | null>(null);
  const [detection, setDetection] = useState<Detection | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [streamHz, setStreamHz] = useState(10);
  const [nowMs, setNowMs] = useState(() => Date.now());

  const snapshot = useSession(session);
  useAging(session);
  // A vehicle's armed state expires the same way ours does, so it is aged by the
  // same clock rather than left frozen on the last heartbeat.
  useMavAging(mav);

  // A clock for the "open for" field. Once a second is plenty for something a
  // person reads, and it keeps the page from re-rendering at frame rate.
  useEffect(() => {
    const timer = setInterval(() => setNowMs(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);

  const serialAvailable = useMemo(() => WebSerialTransport.supported(), []);

  const disconnect = useCallback(async () => {
    const current = session;
    const other = foreign;
    const vehicle = mav;
    setSession(null);
    setForeign(null);
    setMav(null);
    setDetection(null);
    setReviewEdits(0);
    if (current !== null) await current.close();
    if (other !== null) await other.close();
    if (vehicle !== null) await vehicle.close();
  }, [foreign, mav, session]);

  const connect = useCallback(async (choosePort = false) => {
    setError(null);
    if (session !== null || foreign !== null || mav !== null) await disconnect();

    let transport: Transport;
    try {
      transport =
        kind === 'serial'
          ? new WebSerialTransport(choosePort)
          : kind === 'bridge'
            ? new BridgeTransport({ url: bridgeUrl })
            : new DemoTransport();
    } catch (problem) {
      setError(describe(problem));
      return;
    }

    setBusy(true);
    try {
      // The demo board is ours by construction, so it is not interrogated. On a
      // real port the question is worth its cost: the same cable can carry an
      // AerialKit board, a Betaflight board or a vehicle, and which one it is
      // decides both what this app may send and what it may promise.
      if (kind === 'demo') {
        const next = new AerialKitSession(transport);
        setSession(next);
        await next.open();
        await next.refresh();
        return;
      }

      await transport.open();
      const found = await detectFirmware(transport);
      setDetection(found);

      if (found.family === 'aerialkit') {
        const next = new AerialKitSession(transport, { linkAlreadyOpen: true });
        setSession(next);
        await next.open();
        await next.refresh();
        return;
      }

      if (found.family === 'msp') {
        const board = new MspBoard(transport, { linkAlreadyOpen: true });
        setForeign(board);
        await board.open();
        return;
      }

      if (found.family === 'mavlink') {
        // The bytes detection already heard are handed over rather than
        // discarded: a vehicle heartbeats on a period of its own, and waiting
        // for the next one would be a second of a dead-looking page for no
        // reason. They go through the same decoder, not a summary of it.
        const board = new MavBoard(transport, {
          linkAlreadyOpen: true,
          replay: found.mavlink?.heard ?? [],
        });
        setMav(board);
        await board.open();
        board.age();
        return;
      }

      // Nothing recognised. The link is closed rather than left open, and the
      // sentence says what was heard and what was not — a person whose cable is
      // fine but whose firmware is unknown is owed that difference.
      await transport.close();
      const lost = transport instanceof WebSerialTransport ? transport.earlyLoss : null;
      if (lost !== null) {
        setError(lost);
        return;
      }
      setError(
        `${found.detail}. Nothing was sent that could change the board.` +
          (found.family === 'silent' && kind === 'serial'
            ? ' Check that the port you picked is the board (“AerialKit F405”, or ttyACM on Linux). ' +
              'On Linux, ModemManager can hold a new USB serial port for a few seconds after it appears — ' +
              'wait and connect again. For persistent failures, see the USB recovery guide: ' +
              'https://github.com/RawFish69/aerial-kit/blob/main/docs/web-configurator.md#usb-recovery-on-linux'
            : ''),
      );
    } catch (problem) {
      setError(describe(problem));
    } finally {
      setBusy(false);
    }
  }, [bridgeUrl, disconnect, foreign, kind, mav, session]);

  const actions: ParameterActions = useMemo(
    () => ({
      edit: (index, value) => session?.edit(index, value),
      revert: (index) => session?.revert(index),
      reread: (index) => void session?.reread(index),
      write: (index) => void session?.write(index),
      writeAll: () => void session?.writeAll(),
      reset: (index) => void session?.resetParameters(index),
    }),
    [session],
  );

  // Stable identities, because the Receiver panel starts and stops its own
  // polling from an effect keyed on `onWatchRc`: a fresh function each render
  // would have the panel stop and restart the poll it just started.
  const onWatchRc = useCallback((on: boolean) => session?.watchRc(on), [session]);
  const onReadRc = useCallback(() => void session?.refreshRc(), [session]);
  const onWatchSensors = useCallback((on: boolean) => session?.watchSensors(on), [session]);
  const onReadSensors = useCallback(() => void session?.refreshSensors(), [session]);
  const onWatchAttitude = useCallback((on: boolean) => session?.watchAttitude(on), [session]);

  const phase = snapshot?.phase ?? 'closed';
  const connected = mav !== null || foreign !== null || (session !== null && snapshot !== null);
  const unsaved = snapshot?.unsaved ?? null;
  const staged = snapshot?.parameters.filter((row) => row.edited !== null).length ?? 0;
  const saveReason = staged > 0
    ? 'Send or discard the staged edits first. Saving writes only the values already on the board.'
    : snapshot?.permission.allowed ? null : snapshot?.permission.reason ?? 'Connect a board first.';

  return (
    <div className={`app ${connected ? 'is-connected' : 'is-idle'}`}>
      <Navbar className="topbar">
        <Navbar.Group className="brand">
          <BrandMark />
          <Navbar.Heading>Aerial Kit configurator</Navbar.Heading>
        </Navbar.Group>

        {session !== null && snapshot !== null && mav === null && foreign === null && (
          <>
            <div className="status">
              {/* Always on screen, on every tab: which board this is decides
                  what every other number on the page means. */}
              {snapshot.identity !== null && <IdentityBadge identity={snapshot.identity} />}
              <Tag minimal intent={phaseIntent(phase)}>{phase}</Tag>
              {snapshot.readProgress !== null && phase === 'reading' && (
                <span className="small muted">
                  reading parameter {snapshot.readProgress.done} of {snapshot.readProgress.total}
                </span>
              )}
              {snapshot.failure !== null && <span className="small">{snapshot.failure}</span>}
            </div>
            <ArmedBanner live={snapshot.live} permission={snapshot.permission} />
            <div className="actions">
              <Button minimal icon="refresh" onClick={() => void session.refresh()} disabled={busy || phase === 'failed'}>
                Re-read table
              </Button>
              <Button
                icon="floppy-disk"
                intent={unsaved !== null && unsaved > 0 ? 'primary' : 'none'}
                onClick={() => void session.save()}
                disabled={busy || saveReason !== null}
                title={saveReason ?? 'Write the running parameters to flash'}
              >
                Save to flash{unsaved !== null && unsaved > 0 ? ` (${unsaved})` : ''}
              </Button>
              <Button minimal icon="offline" onClick={() => void disconnect()}>Disconnect</Button>
            </div>
          </>
        )}

        {mav !== null && (
          <>
            <div className="status">
              <Tag minimal intent={phaseIntent(mav.snapshot.phase)}>{mav.snapshot.phase}</Tag>
              {detection !== null && <span className="small muted">{detection.detail}</span>}
            </div>
            <div className="actions">
              <Button onClick={() => void mav.readParameters()} disabled={busy || mav.snapshot.phase !== 'ready'}>
                Read parameters
              </Button>
              <Button minimal icon="offline" onClick={() => void disconnect()}>Disconnect</Button>
            </div>
          </>
        )}

        {foreign !== null && (
          <>
            <div className="status">
              <Tag minimal intent={phaseIntent(foreign.snapshot.phase)}>{foreign.snapshot.phase}</Tag>
              {detection !== null && <span className="small muted">{detection.detail}</span>}
            </div>
            <div className="actions">
              <Button onClick={() => void foreign.poll()}>Re-read</Button>
              <Button minimal icon="offline" onClick={() => void disconnect()}>Disconnect</Button>
            </div>
          </>
        )}
      </Navbar>

      {session !== null && snapshot !== null && (
        <div className="changebar" aria-label="Configuration changes">
          <div className="change-stage">
            <span className={`stage-number ${staged > 0 ? 'pending' : ''}`} aria-hidden="true">1</span>
            <div>
              <strong>{staged > 0 ? `${staged} staged edit${staged === 1 ? '' : 's'}` : 'No staged edits'}</strong>
              <span>{staged > 0 ? "Send or discard edits before saving." : "Edits stay in this page until sent."}</span>
            </div>
          </div>
          <div className="change-stage">
            <span className={`stage-number ${unsaved !== null && unsaved > 0 ? 'pending' : ''}`} aria-hidden="true">2</span>
            <div>
              <strong>{unsaved === null ? 'Flash state not reported' : unsaved > 0 ? `${unsaved} board change${unsaved === 1 ? '' : 's'} not in flash` : 'No board changes to save'}</strong>
              <span>{snapshot.identity?.isDemo ? 'Demo values reset when disconnected.' : 'Save board values to keep them after power off.'}</span>
            </div>
          </div>
          {staged > 0 && (
            <Button minimal intent="primary" icon="list" onClick={() => setReviewEdits((value) => value + 1)}>
              Review edits ({staged})
            </Button>
          )}
        </div>
      )}

      {error !== null && connected && <Notice intent="danger" className="page-notice">{error}</Notice>}

      {mav !== null ? (
        <main className="page">
          <MavWorkspace board={mav} />
        </main>
      ) : foreign !== null ? (
        <main className="page">
          <ForeignWorkspace board={foreign} />
        </main>
      ) : session === null || snapshot === null ? (
        <main className="connect-page">
          <ConnectForm
            kind={kind}
            setKind={setKind}
            bridgeUrl={bridgeUrl}
            setBridgeUrl={setBridgeUrl}
            serialAvailable={serialAvailable}
            busy={busy}
            onConnect={(choose) => void connect(choose)}
            error={error}
          />
        </main>
      ) : (
        <TabRail
          snapshot={snapshot}
          actions={actions}
          busy={busy}
          streamHz={streamHz}
          setStreamHz={setStreamHz}
          onStream={(hz) => void session.stream(hz)}
          nowMs={nowMs}
          reviewEdits={reviewEdits}
          onReviewEditsConsumed={() => setReviewEdits(0)}
          onRefresh={() => void session.refresh()}
          onSave={() => void session.save()}
          onWatchRc={onWatchRc}
          onReadRc={onReadRc}
          onWatchSensors={onWatchSensors}
          onReadSensors={onReadSensors}
          onWatchAttitude={onWatchAttitude}
          footer={
            // Not a tab: a limitation is owed on every connection, including
            // the ones where nothing is being changed.
            <Panel title="What this app cannot establish" note="stated on every connection, not only when something goes wrong">
              <Limitations items={snapshot.limitations} permission={snapshot.permission} />
            </Panel>
          }
        />
      )}
    </div>
  );
}

function phaseIntent(phase: string): Intent {
  return phase === 'ready' ? 'success' : phase === 'failed' ? 'danger' : 'warning';
}

function BrandMark() {
  return (
    <img
      className="brand-mark"
      src={`${import.meta.env.BASE_URL}brand/aerialkit-rotor-a.svg`}
      width={32}
      height={32}
      alt=""
      aria-hidden="true"
    />
  );
}

function ConnectForm({
  kind,
  setKind,
  bridgeUrl,
  setBridgeUrl,
  serialAvailable,
  busy,
  onConnect,
  error,
}: {
  kind: TransportKind;
  setKind: (kind: TransportKind) => void;
  bridgeUrl: string;
  setBridgeUrl: (url: string) => void;
  serialAvailable: boolean;
  busy: boolean;
  onConnect: (choosePort: boolean) => void;
  error: string | null;
}) {
  const unavailable = kind === 'serial' && !serialAvailable;

  return (
    <Card className="connect-card" elevation={2} aria-labelledby="connect-title">
      <H2 id="connect-title">Connect your board</H2>
      <p className="bp5-text-muted">
        Choose a connection to read your flight controller. To try the interface without hardware, open the demo.
      </p>

      <div className="choices" role="radiogroup" aria-label="Talk to">
        {[KINDS[1]!, KINDS[0]!, KINDS[2]!].map((entry) => {
          const disabled = entry.kind === 'serial' && !serialAvailable;
          return (
            <RadioCard
              key={entry.kind}
              className="choice"
              inputProps={{ name: "transport", value: entry.kind }}
              checked={kind === entry.kind}
              disabled={disabled}
              onChange={() => setKind(entry.kind)}
              alignIndicator="left"
            >
              <div>
                <strong>
                  {entry.label}
                  {disabled ? ' (not available in this browser)' : ''}
                </strong>
                <div className="bp5-text-muted">{entry.blurb}</div>
              </div>
            </RadioCard>
          );
        })}
      </div>

      {kind === 'bridge' && (
        <FormGroup label="Bridge address" labelFor="url">
          <InputGroup id="url" value={bridgeUrl} spellCheck={false} onChange={(event) => setBridgeUrl(event.target.value)} />
        </FormGroup>
      )}

      {unavailable && (
        <Notice intent="none">
          This browser has no Web Serial, so a cable cannot be used from this page. Chrome, Edge and Opera have it;
          Safari and Firefox do not. The demo board and the local bridge work anywhere.
        </Notice>
      )}

      <div className="connect-actions">
        <Button intent="primary" large icon="link" onClick={() => onConnect(false)} disabled={busy || unavailable} loading={busy}>
          {kind === 'demo' ? 'Explore demo' : kind === 'bridge' ? 'Connect bridge' : 'Connect via USB'}
        </Button>
        {kind === 'serial' && serialAvailable && (
          <Button minimal onClick={() => onConnect(true)} disabled={busy}>
            Choose a different port
          </Button>
        )}
      </div>

      {error !== null && <Notice intent="danger">{error}</Notice>}
    </Card>
  );
}

function describe(problem: unknown): string {
  if (problem instanceof TransportError) return problem.message;
  if (problem instanceof Error) return problem.message;
  return String(problem);
}
