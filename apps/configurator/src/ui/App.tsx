import { Button, HTMLSelect, InputGroup, Menu, MenuDivider, MenuItem, Navbar, Tag, type Intent } from '@blueprintjs/core';
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
import { applyTheme, loadTheme, THEMES, type Theme } from './theme';
import { Desktop, Notepad, Taskbar, TitleBar, useWindowDrag, type WindowState } from './xp';

/**
 * The workspace.
 *
 * One board, one session, one page. The connection controls sit in the top bar
 * in every state, the way a bench tool's port picker does: a person always
 * connects and disconnects from the same place.
 *
 * Connecting is always a click. For Web Serial that is a hard requirement of
 * the browser, and for the others it is the right shape anyway: this app never
 * opens a device because a page loaded.
 */

const REPO = 'https://github.com/RawFish69/aerial-kit';
const CONTACT = 'dev@nori.fish';
const LINKS = {
  guide: `${REPO}/blob/main/docs/web-configurator.md`,
  firmware: `${REPO}/tree/main/firmware`,
  source: REPO,
  issues: `${REPO}/issues`,
  contact: CONTACT,
};

const KINDS: ReadonlyArray<{ kind: TransportKind; label: string }> = [
  { kind: 'serial', label: 'USB serial' },
  { kind: 'demo', label: 'Demo board' },
  { kind: 'bridge', label: 'Local bridge' },
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
  const [theme, setTheme] = useState<Theme>(loadTheme);
  useEffect(() => applyTheme(theme), [theme]);
  /** Classic only: the window's state. Minimised keeps the board connected. */
  const [win, setWin] = useState<WindowState>('maximized');
  const [balloon, setBalloon] = useState<{ title: string; text: string } | null>(null);
  const [aboutOpen, setAboutOpen] = useState(false);
  // A restored window can be moved by its title bar; maximised, it cannot.
  const move = useWindowDrag(win === 'restored');
  const openAbout = useCallback(() => setAboutOpen(true), []);
  const closeAbout = useCallback(() => setAboutOpen(false), []);

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

  const connect = useCallback(async (choosePort = false, via: TransportKind = kind) => {
    setError(null);
    if (session !== null || foreign !== null || mav !== null) await disconnect();

    let transport: Transport;
    try {
      transport =
        via === 'serial'
          ? new WebSerialTransport(choosePort)
          : via === 'bridge'
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
      if (via === 'demo') {
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
          (found.family === 'silent' && via === 'serial'
            ? ' Check that the port you picked is the board (“AerialKit F405”, or ttyACM on Linux). ' +
              'On Linux, ModemManager can hold a new USB serial port for a few seconds after it appears — ' +
              'wait and connect again.'
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
  const open = connected || session !== null;
  const unsaved = snapshot?.unsaved ?? null;
  const staged = snapshot?.parameters.filter((row) => row.edited !== null).length ?? 0;
  const saveReason = staged > 0
    ? 'Send or discard the staged edits first. Saving writes only the values already on the board.'
    : snapshot?.permission.allowed ? null : snapshot?.permission.reason ?? 'Connect a board first.';

  const product = snapshot?.identity?.product ?? null;
  // Classic's tray balloon, once per board, the way new hardware announced itself.
  useEffect(() => {
    if (product === null) return;
    setBalloon({ title: 'Found New Hardware', text: `${product} is connected and ready to use.` });
    const timer = setTimeout(() => setBalloon(null), 7000);
    return () => clearTimeout(timer);
  }, [product]);

  const openDemo = () => {
    setKind('demo');
    void connect(false, 'demo');
  };
  const windowTitle = `Aerial Kit Configurator${product !== null ? ` - ${product}` : ''}`;
  // Classic and OG share the window, the menus and the taskbar.
  const classic = theme !== 'modern';
  const links = LINKS;

  return (
    <>
    {classic && <Desktop win={win} onOpen={() => setWin('maximized')} onAbout={openAbout} />}
    <div
      className={`app ${connected ? 'is-connected' : 'is-idle'}${classic ? ` xp-window xp-${win}` : ''}`}
      style={classic ? move.style : undefined}
    >
      {classic && (
        <TitleBar
          title={windowTitle}
          win={win}
          setWin={setWin}
          onClose={() => {
            void disconnect();
            setWin('closed');
          }}
          theme={theme}
          setTheme={setTheme}
          links={links}
          onAbout={openAbout}
          dragHandlers={move.handlers}
          file={
            <Menu>
              {!open && <MenuItem icon="link" text="Connect" onClick={() => void connect()} disabled={busy} />}
              {!open && (
                <MenuItem icon="lab-test" text="Open demo board" onClick={openDemo} disabled={busy} />
              )}
              {session !== null && (
                <MenuItem icon="refresh" text="Re-read table" onClick={() => void session.refresh()} disabled={busy || phase === 'failed'} />
              )}
              {session !== null && (
                <MenuItem icon="floppy-disk" text="Save to flash" onClick={() => void session.save()} disabled={busy || saveReason !== null} />
              )}
              {open && <MenuDivider />}
              {open && <MenuItem icon="offline" text="Disconnect" onClick={() => void disconnect()} />}
            </Menu>
          }
        />
      )}
      <Navbar className="topbar">
        <Navbar.Group className="brand">
          <BrandMark />
          <Navbar.Heading>Aerial Kit <span className="brand-product">Configurator</span></Navbar.Heading>
        </Navbar.Group>

        {!open && (
          <ConnectBar
            kind={kind}
            setKind={setKind}
            bridgeUrl={bridgeUrl}
            setBridgeUrl={setBridgeUrl}
            serialAvailable={serialAvailable}
            busy={busy}
            onConnect={(choose) => void connect(choose)}
          />
        )}

        {session !== null && snapshot !== null && mav === null && foreign === null && (
          <>
            <div className="status">
              {/* Always on screen, on every tab: which board this is decides
                  what every other number on the page means. */}
              {snapshot.identity !== null && <IdentityBadge identity={snapshot.identity} />}
              <ArmedBanner live={snapshot.live} permission={snapshot.permission} />
            </div>
            <div className="actions">
              {staged > 0 && (
                <Button minimal intent="warning" icon="edit" onClick={() => setReviewEdits((value) => value + 1)}>
                  Review edits ({staged})
                </Button>
              )}
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
              <Button icon="offline" onClick={() => void disconnect()}>Disconnect</Button>
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
              <Button icon="offline" onClick={() => void disconnect()}>Disconnect</Button>
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
              <Button icon="offline" onClick={() => void disconnect()}>Disconnect</Button>
            </div>
          </>
        )}
        {theme === 'modern' && <HTMLSelect
          className="theme-picker"
          minimal
          aria-label="Theme"
          value={theme}
          onChange={(event) => setTheme(event.currentTarget.value as Theme)}
        >
          {THEMES.map((entry) => <option key={entry.id} value={entry.id}>{entry.label}</option>)}
        </HTMLSelect>}
        {theme === 'modern' && <nav className="resource-links" aria-label="Project resources">
          <a href={LINKS.guide} target="_blank" rel="noreferrer">Docs</a>
          <a href={LINKS.source} target="_blank" rel="noreferrer">GitHub</a>
        </nav>}
      </Navbar>

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
        <StartPage
          serialAvailable={serialAvailable}
          busy={busy}
          error={error}
          onDemo={openDemo}
        />
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
            <Panel title="Connection limits">
              <Limitations items={snapshot.limitations} permission={snapshot.permission} />
            </Panel>
          }
        />
      )}

      {session !== null && snapshot !== null && (
        <footer className="statusbar" aria-label="Connection status">
          <span>{snapshot.transport.label}</span>
          <span className={`phase phase-${phase}`}>
            {phase === 'reading' && snapshot.readProgress !== null
              ? `reading ${snapshot.readProgress.done}/${snapshot.readProgress.total}`
              : phase}
          </span>
          {snapshot.identity !== null && <span>protocol v{snapshot.identity.protocolVersion}</span>}
          {snapshot.identity !== null && <span>{snapshot.identity.parameterCount} parameters</span>}
          <span className={staged > 0 ? 'pending' : undefined}>
            {staged === 0 ? 'no staged edits' : `${staged} staged edit${staged === 1 ? '' : 's'}`}
          </span>
          <span className={unsaved !== null && unsaved > 0 ? 'pending' : undefined}>
            {unsaved === null ? 'flash state unknown' : unsaved > 0 ? `${unsaved} not saved to flash` : 'flash in sync'}
          </span>
          {snapshot.failure !== null && <span className="failure">{snapshot.failure}</span>}
        </footer>
      )}
    </div>
    {classic && (
      <Taskbar
        win={win}
        setWin={setWin}
        title={windowTitle}
        connected={connected}
        linkLabel={snapshot?.transport.label ?? ''}
        theme={theme}
        setTheme={setTheme}
        onDemo={() => {
          setWin('maximized');
          openDemo();
        }}
        onDisconnect={() => void disconnect()}
        links={links}
        balloon={balloon}
        onBalloonClose={() => setBalloon(null)}
        onAbout={openAbout}
      />
    )}
    {classic && aboutOpen && <Notepad onClose={closeAbout} />}
    </>
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
      width={28}
      height={28}
      alt=""
      aria-hidden="true"
    />
  );
}

function ConnectBar({
  kind,
  setKind,
  bridgeUrl,
  setBridgeUrl,
  serialAvailable,
  busy,
  onConnect,
}: {
  kind: TransportKind;
  setKind: (kind: TransportKind) => void;
  bridgeUrl: string;
  setBridgeUrl: (url: string) => void;
  serialAvailable: boolean;
  busy: boolean;
  onConnect: (choosePort: boolean) => void;
}) {
  const unavailable = kind === 'serial' && !serialAvailable;
  return (
    <div className="connectbar">
      <HTMLSelect
        aria-label="Connection"
        value={kind}
        onChange={(event) => setKind(event.currentTarget.value as TransportKind)}
        disabled={busy}
      >
        {KINDS.map((entry) => (
          <option key={entry.kind} value={entry.kind} disabled={entry.kind === 'serial' && !serialAvailable}>
            {entry.label}
            {entry.kind === 'serial' && !serialAvailable ? ' (not available in this browser)' : ''}
          </option>
        ))}
      </HTMLSelect>
      {kind === 'bridge' && (
        <InputGroup
          aria-label="Bridge address"
          className="bridge-url"
          value={bridgeUrl}
          spellCheck={false}
          onChange={(event) => setBridgeUrl(event.target.value)}
        />
      )}
      {kind === 'serial' && serialAvailable && (
        <Button minimal onClick={() => onConnect(true)} disabled={busy} title="Pick a different serial port">
          Choose port
        </Button>
      )}
      <Button intent="primary" icon="link" onClick={() => onConnect(false)} disabled={busy || unavailable} loading={busy}>
        Connect
      </Button>
    </div>
  );
}

/** What a person sees before anything is connected: what to do, and where the rest lives. */
function StartPage({
  serialAvailable,
  busy,
  error,
  onDemo,
}: {
  serialAvailable: boolean;
  busy: boolean;
  error: string | null;
  onDemo: () => void;
}) {
  return (
    <main className="start">
      <div className="start-body">
        <div className="window-caption" aria-hidden="true">Getting started</div>
        {error !== null && <Notice intent="danger">{error}</Notice>}
        {!serialAvailable && (
          <Notice intent="warning">
            This browser has no Web Serial, so it cannot open a USB port. Use Chrome, Edge or Opera on a desktop,
            or open the demo board to look around.
          </Notice>
        )}

        <section>
          <h2>Connect a flight controller</h2>
          <ol className="start-steps">
            <li>Flash the Aerial Kit firmware to your board. STM32 and ESP32 boards are supported, with more coming soon.</li>
            <li>Plug the board in over USB. Props off.</li>
            <li>Pick <strong>USB serial</strong> above and press <strong>Connect</strong>, then choose the board's port.</li>
          </ol>
          <p className="muted">
            Nothing is written until you send it, and nothing is kept after power-off until you save to flash.
            Writes are refused while the board reports armed.
          </p>
          <Button onClick={onDemo} disabled={busy} icon="lab-test">Open demo</Button>
          <span className="muted small start-demo-note">A simulated board in this page. No hardware needed.</span>
        </section>

        <section>
          <h2>Resources</h2>
          <ul className="start-links">
            <li><a href={LINKS.guide} target="_blank" rel="noreferrer">Configurator guide</a><span>connecting, Linux USB permissions, the local bridge</span></li>
            <li><a href={LINKS.firmware} target="_blank" rel="noreferrer">Firmware</a><span>source, build and flash instructions</span></li>
            <li><a href={LINKS.issues} target="_blank" rel="noreferrer">Report a problem</a><span>GitHub issues</span></li>
          </ul>
        </section>

        <section>
          <h2>Contact</h2>
          <p>
            Developer email: <a href={`mailto:${CONTACT}`}>{CONTACT}</a>
          </p>
        </section>
      </div>
    </main>
  );
}

function describe(problem: unknown): string {
  if (problem instanceof TransportError) return problem.message;
  if (problem instanceof Error) return problem.message;
  return String(problem);
}
