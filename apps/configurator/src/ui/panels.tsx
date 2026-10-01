import { Button, Callout, Card, HTMLTable, Tag, type Intent } from '@blueprintjs/core';
import { Fragment, useEffect, type ReactNode } from 'react';
import {
  BATTERY_STATES,
  flagSet,
  GPS_FIX_TYPES,
  groupHeading,
  RC_PROTOCOLS,
  RC_STICKS,
  RcFlag,
  RcStatus,
  RcSwitch,
  SENSOR_TOPICS_ORDERED,
  Sensor,
  SensorStatus,
} from '../protocol/constants';
import { boundFromText, type RcState, type Status } from '../protocol/messages';
import type {
  BaroBody,
  BatteryBody,
  GpsBody,
  ImuBody,
  RangeBody,
  SensorAnswer,
  SensorBody,
} from '../protocol/messages';
import type {
  Identity,
  Limitation,
  LiveView,
  ParameterRow,
  Permission,
  RcView,
  SensorView,
  SessionEvent,
  SessionSnapshot,
  TelemetryView,
} from '../session/types';
import type { TransportInfo } from '../transport/types';

/*
 * The panels.
 *
 * One rule runs through all of them: a number is never shown without the thing
 * that produced it. "0.250" is not a fact about an aircraft; "the board said
 * 0.250, in reply to a read at 03:12:04" is. Every panel that shows state also
 * shows where the state came from, and every panel that shows a limit shows the
 * file it was read out of.
 */

export function Panel({
  title,
  note,
  children,
  actions,
}: {
  title: string;
  note?: ReactNode;
  children: ReactNode;
  actions?: ReactNode;
}) {
  return (
    <Card className="panel" compact>
      <header>
        <h2 className="bp5-heading">{title}</h2>
        <div className="row" style={{ gap: 10 }}>
          {note !== undefined && <span className="note bp5-text-muted">{note}</span>}
          {actions}
        </div>
      </header>
      <div className="body">{children}</div>
    </Card>
  );
}

/** A notice, as a Blueprint callout. `none` is informational. */
export function Notice({
  intent,
  className,
  style,
  role,
  children,
}: {
  intent: 'danger' | 'warning' | 'none';
  className?: string;
  style?: React.CSSProperties;
  role?: string;
  children: ReactNode;
}) {
  return (
    <Callout
      intent={intent === 'none' ? 'primary' : intent}
      compact
      className={`notice ${className ?? ''}`}
      style={style}
      role={role}
    >
      {children}
    </Callout>
  );
}

/**
 * Whether the aircraft is armed, and why this app believes it.
 *
 * The three states are given three different treatments on purpose. "Unknown"
 * is a hatched bar in the unknown palette, not a quiet grey: a grey that reads
 * as "probably fine" is the most dangerous thing this page could draw, because
 * the write gate is closed in that state and a person who thinks it is open
 * will spend the next minute wondering why the button does nothing.
 */
export function ArmedBanner({ live, permission }: { live: LiveView; permission: Permission }) {
  const cls =
    live.armed === 'armed' ? 'is-armed' : live.armed === 'disarmed' ? 'is-disarmed' : 'is-unknown';
  const word =
    live.armed === 'armed'
      ? 'Armed'
      : live.armed === 'disarmed'
        ? 'Disarmed'
        : 'Armed state unknown';

  const intent: Intent = live.armed === 'armed' ? 'danger' : live.armed === 'disarmed' ? 'success' : 'warning';
  return (
    <div className={`armed ${cls}`} title={permission.reason}>
      <Tag large intent={intent} icon={live.armed === 'disarmed' ? 'tick-circle' : 'warning-sign'}>
        <strong>{word}</strong>
      </Tag>
      <div className="armed-text">
        <span className="implication">
          {live.armed === 'unknown'
            ? 'this app does not write to an aircraft it cannot account for'
            : live.armed === 'armed'
              ? 'no write will be sent'
              : 'writes are permitted'}
        </span>
        <span className="because bp5-text-muted">{permission.reason}</span>
      </div>
    </div>
  );
}

/** What the app cannot establish, stated every time the connection is open. */
export function Limitations({ items, permission }: { items: readonly Limitation[]; permission?: Permission }) {
  if (items.length === 0) {
    return <p className="muted">No limitations recorded for this connection.</p>;
  }
  return (
    <div className="limits">
      {/*
        Keyed by the id *and* the detail, for the reason the parameter table
        above is keyed by a chapter's group and its first row's index. An id is
        not unique here by design: three of these limitations have two wordings,
        one for a board that says no and one for a board that cannot say, and
        both carry the same id because they are the same limitation. The
        generator picks one of the two, so a connection never renders both — but
        a component that takes a list owes the list a key that is unique among
        its siblings whatever the caller passes, and the two wordings differ in
        the detail. `tests/ui.test.tsx` renders every wording at once and React
        warned about it; production React would not have.
      */}
      {items.map((item) => (
        <Callout
          className={`limit ${item.id === 'armed-state-gates-writes' && permission?.allowed ? 'note' : item.severity}`}
          key={`${item.id}\u0000${item.detail}`}
          compact
          intent={item.id === 'armed-state-gates-writes' && permission?.allowed ? 'none' : item.severity === 'blocks-writes' ? 'danger' : item.severity === 'qualifies-writes' ? 'warning' : 'none'}
        >
          <div className="summary">{item.summary}</div>
          <div className="detail">{item.detail}</div>
          {/*
            The citations are shown rather than left in the prose, because the
            point of having them structured is that a reader can go and look.
            They are `path:symbol`, so they stay right when the file moves on —
            the prose above them is what rots.
          */}
          <ul className="cites">
            {item.citations.map((citation) => (
              <li className="mono" key={citation}>
                {citation}
              </li>
            ))}
          </ul>
        </Callout>
      ))}
    </div>
  );
}

/** Who is on the other end. The word "demo" comes off the wire. */
/**
 * What the board calls itself, as a badge.
 *
 * Its own component because it appears twice: in the header, on every tab, and
 * in the Identity panel. The header copy is the one that matters — "simulated
 * — not hardware" is a safety fact, and a page that keeps it behind a tab is a
 * page where someone can spend an afternoon tuning a board that is not there.
 */
export function IdentityBadge({ identity }: { identity: Identity }) {
  return (
    <Tag large minimal intent={identity.isDemo ? 'warning' : 'success'} icon={identity.isDemo ? 'lab-test' : 'tick'}>
      {identity.isDemo ? (
        <>
          <span>{identity.product}</span>
          <span className="kind"> simulated — not hardware</span>
        </>
      ) : (
        <span>{identity.product}</span>
      )}
    </Tag>
  );
}

export function IdentityPanel({
  identity,
  transport,
  phase,
  openedAtMs,
  nowMs,
}: {
  identity: Identity | null;
  transport: TransportInfo;
  phase: string;
  openedAtMs: number | null;
  nowMs: number;
}) {
  return (
    <Panel title="Connection" note={phase}>
      {identity === null ? (
        <p className="muted">No identity yet — the board has not answered `hello`.</p>
      ) : (
        <>
          <div className="row" style={{ marginBottom: 12 }}>
            <IdentityBadge identity={identity} />
          </div>
          <dl className="kv">
            <dt>transport</dt>
            <dd>{transport.label}</dd>
            <dt>carrying</dt>
            <dd>{transport.detail}</dd>
            <dt>protocol</dt>
            <dd>v{identity.protocolVersion}</dd>
            <dt>parameters</dt>
            <dd>{identity.parameterCount}</dd>
            <dt>changed</dt>
            <dd>{identity.changedSinceSaved}</dd>
            <dt>open for</dt>
            <dd>{openedAtMs === null ? '—' : formatDuration(nowMs - openedAtMs)}</dd>
          </dl>
        </>
      )}
    </Panel>
  );
}

const FLIGHT_STATES = ['disarmed', 'armed', 'failsafe', 'return to home', 'autopilot', 'descend'];

export function LivePanel({ live, telemetry }: { live: LiveView; telemetry: TelemetryView }) {
  const status: Status | null = live.telemetry ?? live.status;
  const source =
    live.telemetry !== null
      ? 'a pushed telemetry frame'
      : live.status !== null
        ? 'a status reply this app asked for'
        : null;

  return (
    <Panel
      title="Live"
      note={
        telemetry.agreedHz === null
          ? 'not streaming'
          : telemetry.agreedHz === 0
            ? 'the board declined to stream'
            : `${telemetry.received} frames at up to ${telemetry.agreedHz} Hz`
      }
    >
      {status === null ? (
        <p className="muted">Nothing has reported yet.</p>
      ) : (
        <>
          <dl className="kv">
            <dt>flight state</dt>
            <dd>
              {FLIGHT_STATES[status.flightState] ?? `unrecognised (${status.flightState})`}
            </dd>
            <dt>link</dt>
            <dd>{status.linkLive === 0 ? 'down, as the board sees it' : 'live'}</dd>
            <dt>attitude</dt>
            <dd>
              {status.rollDeg.toFixed(1)}° / {status.pitchDeg.toFixed(1)}° /{' '}
              {status.yawDeg.toFixed(1)}°
            </dd>
            <dt>gps</dt>
            <dd>
              {status.gpsFixType === 0 ? 'no fix' : `fix type ${status.gpsFixType}`},{' '}
              {status.gpsSatellites} satellites
            </dd>
            <dt>position</dt>
            <dd>
              {(status.lat / 1e7).toFixed(6)}, {(status.lon / 1e7).toFixed(6)}
            </dd>
            <dt>outputs</dt>
            <dd>{status.motors.join(' · ')}</dd>
          </dl>
          <p className="small muted" style={{ marginTop: 10 }}>
            from {source}
            {live.lastSeenMs === null ? '' : `, ${live.lastSeenMs} ms ago`}
          </p>
        </>
      )}
    </Panel>
  );
}

/**
 * The receiver, as the board hears it.
 *
 * Four things this panel deliberately does **not** do, each because the
 * firmware is the authority on it and this app is not:
 *
 *  - **It does not decode the sticks.** The board sends them already decoded,
 *    against its own `rc_min`, `rc_mid`, `rc_max` and `rc_deadband` — which are
 *    parameters, and which `calibrate rc` moves. A client that worked them out
 *    from the raw counts would be a second implementation of the firmware's
 *    `centred()` in another language, agreeing with the aircraft everywhere
 *    except at the deadband edge and at every stick a centre calibration
 *    shifted. So the raw counts and the decoded sticks are shown side by side,
 *    labelled, and this app never converts one into the other.
 *  - **It does not scale the channel bars by the calibration**, for the same
 *    reason one step further on: a bar drawn against a calibration is a decode,
 *    and this reply does not carry one.
 *  - **It does not name which raw channel is which stick.** `AK_RC_ROLL` is 0
 *    and `AK_RC_THROTTLE` is 2 in `ak_types.h`, and none of that is on the
 *    wire — the reply carries counts in the receiver's order and four sticks
 *    the firmware has already picked out. Numbering the bars and naming the
 *    sticks is what the bytes support; matching them up would be this app
 *    asserting a table it read in a header file.
 *  - **It does not poll when nobody is looking.** `onWatch` is turned on by
 *    this component being mounted and off by it being unmounted, which is the
 *    tab being open. The rc poll is a full round trip that shares a wire with
 *    the parameter table, and a page that asked for it on every tab would make
 *    a person's writes slower for a chart they cannot see.
 */
export function ReceiverPanel({
  rc,
  nowMs,
  busy,
  onWatch,
  onRead,
}: {
  rc: RcView;
  nowMs: number;
  busy: boolean;
  onWatch: (on: boolean) => void;
  onRead: () => void;
}) {
  useEffect(() => {
    onWatch(true);
    return () => onWatch(false);
    // `onWatch` is memoised by the caller, so this runs on mount and unmount
    // and not on every poll — a dependency that changed each render would stop
    // and restart the polling it is here to start.
  }, [onWatch]);

  const state = rc.state;
  const age = rc.atMs === null ? null : Math.max(0, nowMs - rc.atMs);

  return (
    <Panel
      title="Receiver"
      note={
        state === null
          ? rc.watching
            ? 'asking the board'
            : 'not reading the receiver'
          : `${rc.polls} reading${rc.polls === 1 ? '' : 's'}, the last ${formatDuration(age ?? 0)} ago`
      }
      actions={
        <Button minimal onClick={onRead} disabled={busy}>
          Read it again
        </Button>
      }
    >
      {rc.error !== null && (
        <Notice intent="danger" style={{ marginBottom: 12 }}>
          {rc.error}
          {rc.failed > 1 && ` — ${rc.failed} readings in a row have failed.`}
        </Notice>
      )}

      {state === null ? (
        <p className="muted">
          {rc.error === null
            ? 'No reading yet. The board has not answered `rc channels`.'
            : // Not "nothing arrived": an answer that this app refused — a
              // board reporting 0x7F for a command it does not have, say — did
              // arrive, and the sentence above is where it is written down.
              'There is no receiver state to show. What the board said is above.'}
        </p>
      ) : state.status !== RcStatus.OK ? (
        <NoReceiverInput />
      ) : (
        <RcReport state={state} age={age} />
      )}
    </Panel>
  );
}

/**
 * The board's own answer that it has no receiver.
 *
 * Quoted rather than paraphrased, because this is one of the two replies a
 * person is most likely to meet and the difference between them is the whole
 * reason the status byte exists: a board with no receiver port says this, and a
 * board whose receiver has never framed answers a full frame of zeros with the
 * link flag clear. The first is a firmware fact about the hardware; the second
 * is a wiring or binding fault. Drawing them alike would send someone looking
 * for a receiver on a board that has nowhere to plug one in.
 */
function NoReceiverInput() {
  return (
    <>
      <p>
        <strong>This board has no receiver input.</strong>
      </p>
      <p className="small muted" style={{ marginTop: 8 }}>
        The answer to <span className="mono">rc channels</span> was one byte — the status
        value the firmware calls <span className="mono">AK_PROTO_RC_NONE</span> — and it stopped
        there. That is a statement about the board rather than about a receiver: a board that
        has a port with nothing plugged into it answers with a full frame, every channel at
        zero and the link flag clear, and this panel draws that differently.
      </p>
    </>
  );
}

function RcReport({ state, age }: { state: RcState; age: number | null }) {
  const linked = flagSet(state.flags, RcFlag.LINK);
  const decoded = flagSet(state.flags, RcFlag.DECODED);
  const sbus = state.protocol === 1;

  return (
    <>
      <dl className="kv">
        <dt>protocol</dt>
        <dd>
          {state.protocolName ?? (
            <span className="unknown">
              the board says {state.protocol}, which this app has no name for
            </span>
          )}
        </dd>
        <dt>frames</dt>
        <dd>
          {linked
            ? 'a channel frame has arrived'
            : 'no channel frame has arrived since the board powered up'}
        </dd>
        {flagSet(state.flags, RcFlag.FAILSAFE) && (
          <>
            <dt>failsafe</dt>
            <dd>
              <strong>the receiver says it has lost its transmitter</strong>
            </dd>
          </>
        )}
        <dt>return path</dt>
        <dd>
          {flagSet(state.flags, RcFlag.TELEMETRY)
            ? 'the handset can be sent telemetry'
            : 'none — this protocol has no return path to the handset'}
        </dd>
        {sbus && (
          <>
            <dt>inverter</dt>
            <dd>
              {flagSet(state.flags, RcFlag.NO_INVERTER)
                ? "this board's receiver pin has no inverter in front of it, so an SBUS receiver wired straight to it reads as framing errors"
                : 'the receiver pin is inverted, as an SBUS receiver expects'}
            </dd>
          </>
        )}
        <dt>read</dt>
        <dd>{age === null ? '—' : `${age} ms ago`}</dd>
      </dl>

      <h3 className="sub-head">Channels</h3>
      <div className="rc-channels">
        {state.channels.map((count, index) => (
          <div className="rc-channel" key={index}>
            <span className="head">
              <span>channel {index + 1}</span>
              <span className="n">{count}</span>
            </span>
            <span className="rc-track">
              <span
                className="fill"
                style={{ left: 0, width: `${Math.max(0, Math.min(100, (count / 2047) * 100))}%` }}
              />
            </span>
          </div>
        ))}
      </div>
      <p className="small muted" style={{ marginTop: 8, marginBottom: 14 }}>
        The counts are the receiver's own, in the order it sends them, and the bars are drawn
        against 0…2047 — the whole of an 11-bit count field. They are <em>not</em> scaled by the
        receiver's calibration, which the board holds in <span className="mono">rc_min</span>,{' '}
        <span className="mono">rc_mid</span> and <span className="mono">rc_max</span> and does not
        send here. This panel does not name which channel is which stick: that mapping lives in
        the firmware's <span className="mono">ak_types.h</span> and is not on the wire.
      </p>

      <h3 className="sub-head">Sticks, as the board decoded them</h3>
      {!decoded ? (
        <p className="muted">
          The board has not decoded these — it sent a frame it would not use, or none at all. All
          four values below are zero, and they mean &ldquo;this board does not know where the
          sticks are&rdquo; rather than &ldquo;the sticks are centred&rdquo;. The two are the same
          bytes and the link flag above is what tells them apart.
        </p>
      ) : (
        <>
          <div className="rc-sticks">
            {RC_STICKS.map((name, index) => {
              const perMille = state.sticks[index] ?? 0;
              return (
                <div className="rc-channel" key={name}>
                  <span className="head">
                    <span>{name}</span>
                    <span className="n">{(perMille / 1000).toFixed(3)}</span>
                  </span>
                  <span className="rc-track">
                    <span className="centre" style={{ left: '50%' }} />
                    <span
                      className="fill"
                      style={spanFor(perMille)}
                    />
                  </span>
                </div>
              );
            })}
          </div>
          <p className="small muted" style={{ marginTop: 8, marginBottom: 14 }}>
            Roll, pitch and yaw run -1.000 to 1.000 with zero at the centre; throttle runs 0.000 to
            1.000. The board sends these in thousandths, already decoded against its own{' '}
            <span className="mono">rc_min</span>, <span className="mono">rc_mid</span>,{' '}
            <span className="mono">rc_max</span> and <span className="mono">rc_deadband</span> — the
            same numbers the flight core uses, which is why this app shows what came back instead
            of working them out from the counts above. A stick that reads exactly 0.000 is inside
            the deadband, not necessarily perfectly centred.
          </p>
        </>
      )}

      <dl className="kv">
        <dt>arm switch</dt>
        <dd>
          {flagSet(state.switches, RcSwitch.ARM_ON)
            ? 'the board reads this channel as asking to arm'
            : 'not asking to arm'}
        </dd>
        <dt>mode switch</dt>
        <dd>
          {flagSet(state.switches, RcSwitch.ANGLE)
            ? 'above the mode threshold: angle mode'
            : 'below the mode threshold: rate mode'}
        </dd>
      </dl>

      <h3 className="sub-head">Counters, since the board powered up</h3>
      <HTMLTable compact striped className="data-table">
        <tbody>
          <tr>
            <td className="name">bytes</td>
            <td>{state.bytes}</td>
            <td className="muted">everything the port has taken in</td>
          </tr>
          <tr>
            <td className="name">frames</td>
            <td>{state.frames}</td>
            <td className="muted">channel frames the receiver delivered</td>
          </tr>
          <tr>
            <td className="name">crc errors</td>
            <td>{state.crcErrors}</td>
            <td className="muted">
              CRSF only — an SBUS frame carries no checksum, so this is 0 there and not a
              measurement
            </td>
          </tr>
          <tr>
            <td className="name">rejected</td>
            <td>{state.rejected}</td>
            <td className="muted">frames the decoder would not use</td>
          </tr>
          <tr>
            <td className="name">lost</td>
            <td>{state.lost}</td>
            <td className="muted">SBUS only: frames flagged lost by the receiver</td>
          </tr>
          <tr>
            <td className="name">failsafe frames</td>
            <td>{state.failsafeFrames}</td>
            <td className="muted">SBUS only: frames arriving with the failsafe bit set</td>
          </tr>
          <tr>
            <td className="name">dropped</td>
            <td>{state.dropped}</td>
            <td className="muted">
              bytes the board's own UART buffer threw away — the port's number, not the
              receiver's
            </td>
          </tr>
        </tbody>
      </HTMLTable>
      <p className="small muted" style={{ marginTop: 10 }}>
        Each counter is named as its receiver names it, and a counter that is 0 because a protocol
        has no such thing says so above rather than reading as a clean measurement. Centring the
        sticks — writing a new <span className="mono">rc_mid</span> — is the console's{' '}
        <span className="mono">calibrate rc</span> (<span className="mono">ak_rc_cal_apply</span>,
        in <span className="mono">aerialkit/src/core/flight/ak_rc.c</span>); this app has no
        calibration page yet, and the Receiver tab is where you would notice you need one.
      </p>
    </>
  );
}

/** A centred bar for a stick in thousandths: negative fills left of the tick,
 *  positive right of it, zero fills nothing at all. */
function spanFor(perMille: number): { left: string; width: string } {
  const clamped = Math.max(-1000, Math.min(1000, perMille));
  const half = clamped / 2000; // -0.5 .. 0.5
  const width = Math.abs(half) * 100;
  return half < 0
    ? { left: `${50 + half * 100}%`, width: `${width}%` }
    : { left: '50%', width: `${width}%` };
}

/**
 * The sensors.
 *
 * **Three answers per sensor, drawn three ways, and that is the whole point of
 * this panel.** The wire refuses to collapse them and a screen that did would
 * be undoing the firmware's work:
 *
 *  - *this build does not answer for that topic* — a fact about the firmware,
 *    and the thing to do about it is update it;
 *  - *this board has none fitted* — a fact about the aircraft, and the thing to
 *    do about it is look at the socket;
 *  - *a reading*, and a zero in it is a zero.
 *
 * A fourth case belongs to this app rather than to the board: a topic that has
 * not been read yet. That is drawn as "not read", never as "absent", because
 * those are the two states a person most needs to tell apart and the one this
 * panel is most able to confuse.
 */
export function SensorsPanel({
  sensors,
  nowMs,
  busy,
  onWatch,
  onRead,
}: {
  sensors: SensorView;
  nowMs: number;
  busy: boolean;
  onWatch: (on: boolean) => void;
  onRead: () => void;
}) {
  useEffect(() => {
    onWatch(true);
    return () => onWatch(false);
    // Memoised by the caller, as the receiver's is: a dependency that changed
    // each render would stop and restart the polling it exists to start.
  }, [onWatch]);

  const age = sensors.atMs === null ? null : Math.max(0, nowMs - sensors.atMs);
  const read = SENSOR_TOPICS_ORDERED.filter(({ topic }) => sensors.answers[topic] !== undefined);

  return (
    <Panel
      title="Sensors"
      note={
        sensors.atMs === null
          ? sensors.watching
            ? 'asking the board'
            : 'not reading the sensors'
          : `${sensors.rounds} round${sensors.rounds === 1 ? '' : 's'}, the last ${formatDuration(age ?? 0)} ago`
      }
      actions={
        <Button minimal onClick={onRead} disabled={busy}>
          Read them again
        </Button>
      }
    >
      {sensors.error !== null && (
        <Notice intent="danger" style={{ marginBottom: 12 }}>
          {sensors.error}
          {sensors.failed > 1 && ` — ${sensors.failed} rounds in a row have failed.`}
        </Notice>
      )}

      {read.length === 0 ? (
        <p className="muted">
          {sensors.error === null
            ? 'Nothing read yet. The board has not answered `sensor info`.'
            : // An answer that arrived and was refused — a board reporting 0x7F,
              // say — is not the same as silence, and the sentence above is
              // where that answer is written down.
              'There is no sensor reading to show. What the board said is above.'}
        </p>
      ) : (
        SENSOR_TOPICS_ORDERED.map(({ topic, name }) => {
          const answer = sensors.answers[topic];
          if (answer === undefined) {
            // Asked for over the wire, so it is in the list, but this round did
            // not reach it. Named rather than omitted: a panel that silently
            // dropped a section would look like a board with four sensors.
            return (
              <section className="sensor" key={topic}>
                <h3 className="sub-head">
                  {name} <span className="muted small">— not read in the last round</span>
                </h3>
              </section>
            );
          }
          return <SensorSection key={topic} name={name} answer={answer} />;
        })
      )}
    </Panel>
  );
}

/** One topic, in whichever of the three states the board answered with. */
function SensorSection({ name, answer }: { name: string; answer: SensorAnswer }) {
  const body = answer.body;

  if (answer.status === SensorStatus.NO_SUCH) {
    return (
      <section className="sensor">
        <h3 className="sub-head">{name}</h3>
        <p>
          <strong>This firmware does not answer for {name}.</strong>
        </p>
        <p className="small muted" style={{ marginTop: 8 }}>
          The reply was the status this protocol calls{' '}
          <span className="mono">AK_PROTO_SENSOR_NO_SUCH</span>, which is a statement about the
          build and not about this aircraft. A build with the driver compiled in would answer the
          same question even with nothing soldered in — it would say{' '}
          <em>present: no</em>, which this panel draws differently. The fix for this one is a
          firmware update; going looking at the socket would not help.
        </p>
      </section>
    );
  }

  if (body === null) {
    return (
      <section className="sensor">
        <h3 className="sub-head">{name}</h3>
        <p>
          <strong>This board has no {name} fitted.</strong>
        </p>
        <p className="small muted" style={{ marginTop: 8 }}>
          The board knows the question and answered <span className="mono">present: no</span> with
          no reading after it. The firmware sends no body at all in this case rather than a body
          of zeros, and that is deliberate: a body of zeros would say this board has a sensor
          reading zero, which is a sensor that has failed. The two are different faults with
          different fixes.
        </p>
      </section>
    );
  }

  const driver = 'driver' in body ? body.driver : '';
  return (
    <section className="sensor">
      <h3 className="sub-head">
        {name} <span className="muted small">— {driver === '' ? 'no driver name' : driver}</span>
      </h3>
      <SensorFields body={body} />
    </section>
  );
}

// ---- one renderer per topic ------------------------------------------------
//
// Kept as separate components rather than one function full of switches: the
// body is a tagged union, so narrowing it once per component is what lets
// TypeScript check that each field is read off the body that actually has it.
// A single reader that took the union and tested fields would compile while
// showing an IMU's sample count under the barometer.

function SensorFields({ body }: { body: SensorBody }) {
  switch (body.topic) {
    case Sensor.IMU:
      return <ImuFields body={body} />;
    case Sensor.BARO:
      return <BaroFields body={body} />;
    case Sensor.RANGE:
      return <RangeFields body={body} />;
    case Sensor.BATTERY:
      return <BatteryFields body={body} />;
    case Sensor.GPS:
      return <GpsFields body={body} />;
  }
}

function ImuFields({ body }: { body: ImuBody }) {
  return (
    <>
      {body.absentReason !== null && (
        <Notice intent="warning" style={{ marginBottom: 12 }}>
          <strong>The board has an IMU driver and its part did not come up:</strong>{' '}
          {body.absentReason}. The numbers below are the last ones it read, or zeros if it never
          read any.
        </Notice>
      )}
      <dl className="kv">
        <dt>part</dt>
        <dd>
          {body.driver}
          {body.whoami !== 0 && (
            <span className="muted small"> — identified itself as 0x{body.whoami.toString(16)}</span>
          )}
        </dd>
        <dt>acceleration</dt>
        <dd>
          {body.accel.map((v, i) => `${fmt(v / 1000, 3)} g${'xyz'[i]}`).join(', ')}
          <span className="muted small"> — the last sample, in the board's own frame</span>
        </dd>
        <dt>rotation</dt>
        <dd>
          {body.gyro.map((v, i) => `${fmt(v / 1000, 3)} rad/s${'xyz'[i]}`).join(', ')}
        </dd>
        <dt>alignment</dt>
        <dd>
          {body.align.every((v) => v === 0)
            ? 'none — the board is not told to rotate this part'
            : body.align.map((v, i) => `${fmt(v, 1)}°${['roll', 'pitch', 'yaw'][i]}`).join(', ')}
        </dd>
        <dt>gyro bias</dt>
        <dd>
          {body.gyroBias.every((v) => v === 0)
            ? 'zero — either it was calibrated to zero or it has never been calibrated'
            : body.gyroBias.map((v) => `${fmt(v / 1000, 4)}`).join(', ') + ' rad/s'}
        </dd>
        <dt>samples</dt>
        <dd>
          {body.samples}
          <span className="muted small"> since power-up</span>
        </dd>
        <dt>errors</dt>
        <dd className={body.errors > 0 ? 'bad' : undefined}>{body.errors}</dd>
      </dl>
      <p className="small muted" style={{ marginBottom: 4 }}>
        Acceleration is in thousandths of a g and rotation in thousandths of a radian per second,
        which is the unit the board stores its log records in. A board standing still reads one g
        on whichever axis points down.
      </p>
    </>
  );
}

function BaroFields({ body }: { body: BaroBody }) {
  return (
    <>
      <dl className="kv">
        <dt>part</dt>
        <dd>{body.driver}</dd>
        <dt>pressure</dt>
        <dd>
          {fmt(body.pressurePa / 100, 2)} hPa
          <span className="muted small"> — {body.pressurePa} Pa</span>
        </dd>
        <dt>temperature</dt>
        <dd>{fmt(body.temperatureC, 2)} °C</dd>
        <dt>reference</dt>
        <dd>
          {body.haveReference
            ? `${fmt(body.referencePa / 100, 2)} hPa — the pressure the board took as the ground`
            : 'none — the board has not been told what the ground reads, so the height below is a difference from nothing'}
        </dd>
        <dt>height</dt>
        <dd>
          {body.haveReference ? (
            `${fmt(body.heightCm / 100, 2)} m`
          ) : (
            <span className="unknown">{body.heightCm} cm, which means nothing without a reference</span>
          )}
        </dd>
        <dt>fused height</dt>
        <dd>
          {fmt(body.fusedCm / 100, 2)} m
          {body.haveGpsReference && <span className="muted small"> — the GPS has since re-set the ground</span>}
        </dd>
        <dt>samples</dt>
        <dd>
          {body.samples} <span className="muted small">since power-up, {body.errors} bad, {body.fails} failed reads</span>
        </dd>
        <dt>fused from</dt>
        <dd>
          {body.baroSamples} barometer and {body.gpsSamples} GPS samples
        </dd>
      </dl>
      <p className="small muted" style={{ marginBottom: 4 }}>
        <strong>The reference is the whole reason to carry a barometer.</strong> The absolute
        pressure is today's weather; the <em>change</em> since take-off is the altitude. The board
        takes the reference when it arms or when the GPS settles, and this panel shows the height
        as a difference from it — which is a different number from the pressure, and the reason
        both are on screen.
      </p>
    </>
  );
}

function RangeFields({ body }: { body: RangeBody }) {
  const none = body.distanceMm < 0;
  return (
    <>
      <dl className="kv">
        <dt>part</dt>
        <dd>
          {body.driver}
          <span className="muted small"> — at address 0x{body.address.toString(16)}</span>
        </dd>
        <dt>distance</dt>
        <dd className={none ? 'muted' : undefined}>
          {none ? (
            <>
              nothing in range <span className="muted small">(the board sent {body.distanceMm})</span>
            </>
          ) : (
            `${fmt(body.distanceMm, 0)} mm`
          )}
        </dd>
        <dt>range</dt>
        <dd>{body.maxMm} mm at most</dd>
        <dt>age</dt>
        <dd>
          {body.ageMs} ms
          <span className="muted small"> since the reading the distance came from</span>
        </dd>
        <dt>ground</dt>
        <dd>
          {fmt(body.landMm, 0)} mm
          <span className="muted small"> — the height it measured while standing, which is the
            zero for a landing</span>
        </dd>
        <dt>agreement</dt>
        <dd>
          {fmt(body.agreeCm / 100, 2)} m
          <span className="muted small"> — the height at which the rangefinder and the barometer
            last agreed, which is what the landing detector waits for</span>
        </dd>
      </dl>
      <h4 className="sub-head">Counters, since the board powered up</h4>
      <HTMLTable compact striped className="data-table">
        <tbody>
          <tr>
            <td className="name">readings</td>
            <td>{body.samples}</td>
            <td className="muted">taken from the part</td>
          </tr>
          <tr>
            <td className="name">out of range</td>
            <td>{body.outOfRange}</td>
            <td className="muted">
              the part answered but had nothing to report — the ground is usually too far away
            </td>
          </tr>
          <tr>
            <td className="name">rejected</td>
            <td className={body.rejected > 0 ? 'bad' : undefined}>{body.rejected}</td>
            <td className="muted">readings the driver would not use</td>
          </tr>
          <tr>
            <td className="name">faults</td>
            <td className={body.faults > 0 ? 'bad' : undefined}>{body.faults}</td>
            <td className="muted">the bus or the part misbehaved</td>
          </tr>
          <tr>
            <td className="name">failed</td>
            <td className={body.fails > 0 ? 'bad' : undefined}>{body.fails}</td>
            <td className="muted">reads that came back with nothing</td>
          </tr>
        </tbody>
      </HTMLTable>
    </>
  );
}

function BatteryFields({ body }: { body: BatteryBody }) {
  return (
    <>
      <dl className="kv">
        <dt>state</dt>
        <dd>
          {body.stateName === null ? (
            <span className="unknown">the board says {body.state}, which this app has no name for</span>
          ) : (
            body.stateName
          )}
          {!body.ready && <span className="muted small"> — the board is not using this for anything yet</span>}
        </dd>
        <dt>pack</dt>
        <dd>
          {fmt(body.volts, 2)} V
          <span className="muted small"> across {body.cells} cell{body.cells === 1 ? '' : 's'}</span>
        </dd>
        <dt>per cell</dt>
        <dd>
          {body.haveReading ? (
            <strong>{fmt(body.voltsPerCell, 2)} V</strong>
          ) : (
            <span className="unknown">no reading — the board has not measured the pack</span>
          )}
          <span className="muted small">
            {' '}
            — the only number that decides anything, and the one the thresholds below are compared
            against
          </span>
        </dd>
        <dt>at the pin</dt>
        <dd>
          {body.pinMv < 0
            ? 'the board has no reading at the pin'
            : `${body.pinMv} mV behind a ${fmt(body.ratio, 2)}:1 divider`}
        </dd>
        <dt>thresholds</dt>
        <dd>
          warns at {fmt(body.warnCellV, 2)} V and calls it critical at {fmt(body.criticalCellV, 2)} V
          per cell
        </dd>
        <dt>return to home</dt>
        <dd>
          {body.rth
            ? 'the board is configured to bring the aircraft back on a low pack'
            : 'not configured to act on the pack state'}
        </dd>
        <dt>samples</dt>
        <dd>
          {body.samples} <span className="muted small">since power-up, {body.rejected} rejected, {body.returns} state changes</span>
        </dd>
      </dl>
      <p className="small muted" style={{ marginBottom: 4 }}>
        The per-cell number is the board's own division, not this app's: the firmware already knows
        the cell count and applies the thresholds to that figure. This panel does not work it out
        from the pack voltage, because on a pack that is nearly empty the two would differ in the
        second decimal and the thresholds are set in the second decimal.
      </p>
    </>
  );
}

function GpsFields({ body }: { body: GpsBody }) {
  return (
    <>
      <dl className="kv">
        <dt>fix</dt>
        <dd>
          {!body.haveFix ? (
            <span className="unknown">none — the board has not got a position</span>
          ) : (
            <>
              {GPS_FIX_TYPES[body.fixType] ?? (
                <span className="unknown">
                  the receiver says fix type {body.fixType}, which this app has no name for
                </span>
              )}
              <span className="muted small">
                {' '}
                — {body.satellites} satellites{body.fixOk ? ', usable' : ', not yet usable'}
              </span>
            </>
          )}
        </dd>
        {/*
          The position is shown as a dash rather than as 0, 0 when there is no
          fix, and that is not a cosmetic choice: 0, 0 is a real place in the
          Gulf of Guinea, and this app has a doctrine about putting an aircraft
          there. See `parseSensorInfo`'s note on reading `haveFix` first.
        */}
        <dt>position</dt>
        <dd>
          {body.haveFix ? (
            <span className="mono">
              {body.lat.toFixed(7)}, {body.lon.toFixed(7)}
            </span>
          ) : (
            <span className="muted">— (no fix; the board sent no position, and 0, 0 is not it)</span>
          )}
        </dd>
        <dt>altitude</dt>
        <dd>
          {body.haveFix ? `${fmt(body.altMslMm / 1000, 1)} m above the ellipsoid` : '—'}
        </dd>
        <dt>speed</dt>
        <dd>{body.haveFix ? `${fmt(body.speedMmS / 1000, 2)} m/s` : '—'}</dd>
        <dt>course</dt>
        <dd>
          {body.haveFix && body.speedMmS > 0
            ? `${fmt(body.courseDeg, 1)}°`
            : '— (a course is meaningless at a standstill)'}
        </dd>
        <dt>home</dt>
        <dd>
          {!body.haveHome ? (
            'not set — the board has nowhere to return to'
          ) : body.homeDistanceM < 0 ? (
            `set at ${body.homeLat.toFixed(7)}, ${body.homeLon.toFixed(7)}; the distance to it cannot be worked out`
          ) : (
            <>
              {fmt(body.homeDistanceM, 0)} m away at {fmt(body.homeBearingDeg, 0)}°
              <span className="muted small"> — from {body.homeLat.toFixed(7)}, {body.homeLon.toFixed(7)}</span>
            </>
          )}
        </dd>
        <dt>returning</dt>
        <dd>
          {body.returning
            ? 'the board is flying the aircraft home'
            : body.rthEnabled
              ? 'not returning, though the board is allowed to'
              : 'not returning, and not allowed to'}
        </dd>
      </dl>
      <h4 className="sub-head">Counters, since the board powered up</h4>
      <HTMLTable compact striped className="data-table">
        <tbody>
          <tr>
            <td className="name">fixes</td>
            <td>{body.fixes}</td>
            <td className="muted">received{body.validNow ? '; the last one was usable' : '; no usable one at present'}</td>
          </tr>
          <tr>
            <td className="name">dropped</td>
            <td className={body.dropped > 0 ? 'bad' : undefined}>{body.dropped}</td>
            <td className="muted">that the receiver handed over with a bad checksum</td>
          </tr>
          <tr>
            <td className="name">configuration</td>
            <td>{body.configSends}</td>
            <td className="muted">
              messages sent to the receiver — it is configured from this board rather than by a
              separate tool
            </td>
          </tr>
        </tbody>
      </HTMLTable>
      <p className="small muted" style={{ marginBottom: 4 }}>
        The position is the receiver's, converted from the wire's ten-millionths of a degree into
        degrees. Nothing here is a map: this app makes no network requests, so what it can honestly
        draw is a grid and the aircraft's own numbers, not somebody's tiles.
      </p>
    </>
  );
}

/** One row's four facts, kept apart. */
function Trace({ row }: { row: ParameterRow }) {
  const write = row.write;
  if (write === null) return null;
  return (
    <div className="trace">
      <span className="fact">
        <span className="label">requested</span>
        <span className="mono">{write.requested}</span>
      </span>
      <span className="fact">
        <span className="label">echoed</span>
        <span>
          {write.echoed
            ? 'yes — the board put it in its table'
            : `no — ${write.message === '' ? `status ${write.status}` : write.message}`}
        </span>
      </span>
      {/*
       * Three-valued, and the middle value is the common one. `null` is not a
       * missing answer — it is the answer: this firmware re-applies on a
       * successful set but the reply carries no field reporting it, so the
       * fact is unestablished rather than false. Rendering it as "no" would
       * deny something that happens; rendering it as "yes" would claim
       * something no byte said.
       */}
      <span className="fact">
        <span className="label">applied</span>
        <span>
          {write.applied === true
            ? 'yes — the board reported it re-applied'
            : write.applied === false
              ? `no — ${write.notAppliedBecause ?? 'the write did not land'}`
              : 'not established'}
        </span>
      </span>
      {write.applied === false ? null : <span className="why">{write.notAppliedBecause}</span>}
    </div>
  );
}

export interface ParameterActions {
  edit(index: number, value: string): void;
  revert(index: number): void;
  write(index: number): void;
  reread(index: number): void;
  /** Sends every staged value. A separate verb rather than a sentinel index,
   *  because it is a different operation: it keeps going past a refusal. */
  writeAll(): void;
  /**
   * Puts a row back to the value the board's build was compiled with — or the
   * whole table, when `index` is null.
   *
   * `null` rather than a sentinel index like -1, and that is the wire's shape
   * rather than a style choice: mode 2 carries *no index at all*, so a caller
   * meaning "all" must have a way to say nothing, and -1 would be a number the
   * client would then have to remember not to send.
   */
  reset(index: number | null): void;
}

export function ParametersPanel({
  rows,
  allRows,
  permission,
  unsaved,
  busy,
  resetReason,
  actions,
}: {
  rows: readonly ParameterRow[];
  /** The whole table when `rows` is a filtered view of it, so "Send all
   *  staged" still counts staged rows the filter hides. */
  allRows?: readonly ParameterRow[];
  permission: Permission;
  unsaved: number | null;
  busy: boolean;
  /** Null when this board answers `param default`; otherwise why not. A board
   *  that predates the opcode gets a disabled control naming the opcode, not a
   *  button that fails when pressed. */
  resetReason: string | null;
  actions: ParameterActions;
}) {
  if (rows.length === 0) {
    return (
      <Panel title="Parameters">
        <p className="muted">The table has not been read yet.</p>
      </Panel>
    );
  }

  return (
    <Panel
      title="Parameters"
      note={
        unsaved === null
          ? `${(allRows ?? rows).length} read`
          : `${(allRows ?? rows).length} read · the board reports ${unsaved} not yet in flash`
      }
      actions={
        <>
          {/* The one control on this page that moves many numbers at once, and
              it is deliberately not `primary`: Send all staged is what a person
              came here to do, and a build default is what they do when they have
              lost their way. It is also not destructive of anything unsaved —
              it writes to the running table like any other write, and `Reset`
              does not save. */}
          <Button
            minimal
            onClick={() => actions.reset(null)}
            disabled={busy || !permission.allowed || resetReason !== null}
            title={resetReason ?? (permission.allowed ? undefined : permission.reason)}
          >
            Reset all to build defaults
          </Button>
          <Button
            intent="primary"
            onClick={() => actions.writeAll()}
            disabled={busy || !permission.allowed || !(allRows ?? rows).some((row) => row.edited !== null)}
            title={permission.allowed ? undefined : permission.reason}
          >
            Send all staged
          </Button>
        </>
      }
    >
      <HTMLTable compact striped className="params">
        <colgroup>
          <col style={{ width: 192 }} />
          <col className="col-notes" />
          <col style={{ width: 134 }} />
          <col style={{ width: 116 }} />
          <col style={{ width: 96 }} />
          <col style={{ width: 136 }} />
        </colgroup>
        <thead>
          <tr>
            <th>Parameter</th>
            <th className="notes">What it is</th>
            <th>Range</th>
            <th>On the board</th>
            <th>New value</th>
            <th>Write</th>
          </tr>
        </thead>
        <tbody>
          {groupRows(rows).map((segment) => (
            <Fragment key={segment.key}>
              <tr className="group">
                <td colSpan={6}>{segment.label}</td>
              </tr>
              {segment.rows.map((row) => (
                <ParameterLine
                  key={row.index}
                  row={row}
                  permission={permission}
                  busy={busy}
                  resetReason={resetReason}
                  actions={actions}
                />
              ))}
            </Fragment>
          ))}
        </tbody>
      </HTMLTable>
    </Panel>
  );
}

/**
 * The flat parameter list, cut into chapters for reading.
 *
 * The chapter is now the board's own `group` byte, served over `param info`. It
 * used to be derived here from name prefixes — `name.startsWith('rate_')`, plus
 * a list of exceptions — and that was a screen asserting a structure the board
 * never stated. It recognised about thirty names out of ninety-two, and the ones
 * it did recognise it filed by luck: `arm_accel_lpf_hz` landed under "Arming"
 * because it happens to start with `arm_`, not because anything said it belonged
 * there, and every parameter the heuristic had never seen got "Other".
 *
 * The wire order is still the truth and is never changed — `groupRows` only
 * decides where a heading is drawn between already-adjacent rows, so a board
 * whose table order differs simply gets repeated headings, never reordered data.
 *
 * **Which is why a chapter is keyed by its group *and* its first row's index,
 * and not by its group alone.** This build's own table revisits groups — the
 * group bytes run 1, 1, …, 1, 3, 1, 2, 2, …, 4, 12, 8, 8, 8, 5, 4, 4, 4, 4 — so
 * `group:1` begins a second chapter fourteen rows down from the first. A React
 * key is only unique among its siblings, and two chapters sharing one made React
 * match the wrong pair and leave the leftovers in the DOM: the table grew by a
 * chapter on every re-read. That is not a cosmetic failure. It is what the
 * browser check found at milestone 4, and what it looked like from outside was a
 * `Reset all to build defaults` that the board answered `OK`, that the session
 * then re-read correctly, and that left the screen showing the *previous* table's
 * rows — a correct reset that appeared to do nothing, because the row on screen
 * was a stale copy. The row index is unique by the board's own numbering and
 * stable across a re-read, so a chapter keeps its identity and its inputs keep
 * their focus.
 */
function groupRows(
  rows: readonly ParameterRow[],
): Array<{ key: string; label: string; rows: ParameterRow[] }> {
  const out: Array<{ group: string; key: string; label: string; rows: ParameterRow[] }> = [];
  for (const row of rows) {
    const { key: group, label } = groupLabel(row);
    const last = out[out.length - 1];
    if (last !== undefined && last.group === group) {
      last.rows.push(row);
    } else {
      out.push({ group, key: `${group}#${row.index}`, label, rows: [row] });
    }
  }
  return out;
}

/**
 * What to write above a row, and the identity to group by.
 *
 * Three cases, and they are kept apart on purpose because collapsing them is how
 * a heading becomes a guess. A described row gets its group's name. A row whose
 * group number this app has no name for gets the *number*, never a neighbour's
 * name. A row with no metadata at all says that, because "the board did not
 * describe this" and "the board says group none" are different facts.
 *
 * The key is the group number rather than the label, so two adjacent rows the
 * app cannot name still group together instead of starting a heading each.
 */
function groupLabel(row: ParameterRow): { key: string; label: string } {
  if (row.meta === null) {
    return { key: 'undescribed', label: 'Not described by the board' };
  }
  const heading = groupHeading(row.meta.group);
  if (heading === null) {
    return {
      key: `group:${row.meta.group}`,
      label: `Group ${row.meta.group} — this app has no name for that number`,
    };
  }
  return { key: `group:${row.meta.group}`, label: heading };
}

/**
 * Why no range is on the row, in the board's terms where there is one.
 *
 * `metaUnavailable` carries the board's own reason — no `param info` in this
 * build, an entry too large for one frame, a help walk that failed — and it is
 * preferred over anything this app could say, because it is a reading rather
 * than an inference. The rest are the cases where the board described the row
 * but the *bounds* are this app's problem: text the board spelled that this
 * parser will not evaluate, or a text parameter, which has a length rather than
 * a range.
 */
function whyNoRange(row: ParameterRow, low: number | null, high: number | null): string {
  if (row.meta === null) {
    return row.metaUnavailable === null
      ? ' — this app has not read this row’s description yet'
      : ` — ${row.metaUnavailable}`;
  }
  if (row.meta.maxLen !== null) {
    return ` — a text parameter, so it is bounded by ${row.meta.maxLen} characters rather than by a range`;
  }
  if (row.meta.min.trim() === '' && row.meta.max.trim() === '') {
    // The board answered for this row and stated no numeric bound. Kept apart
    // from the case below, which is a board that *did* state bounds and spelled
    // one in a way this parser will not read: the two would otherwise share a
    // sentence that is only true of the second.
    return ' — the board described this row and stated no numeric bound for it';
  }
  if (low === null || high === null) {
    return (
      ' — the board spelled a bound this app will not evaluate, so it shows none ' +
      `rather than a guess (min “${row.meta.min}”, max “${row.meta.max}”)`
    );
  }
  return '';
}

function ParameterLine({
  row,
  permission,
  busy,
  resetReason,
  actions,
}: {
  row: ParameterRow;
  permission: Permission;
  busy: boolean;
  /** Null when this board can be reset; otherwise *why not*, in the board's
   *  own terms. The same reading the tab rail uses, one level down. */
  resetReason: string | null;
  actions: ParameterActions;
}) {
  const staged = row.edited !== null;
  // A range is shown only when both bounds came off the wire *and* this app is
  // willing to evaluate both. One bound is not a range, and half of one invites
  // a person to assume the other half. The bounds arrive as the board spelled
  // them — the protocol has no typed fields anywhere and this app does not
  // invent one — so the text is parsed here and nowhere else.
  const meta = row.meta;
  const low = meta === null ? null : boundFromText(meta.min);
  const high = meta === null ? null : boundFromText(meta.max);
  const bounds =
    low !== null && high !== null && meta !== null
      ? `${fmt(low, meta.decimals)} … ${fmt(high, meta.decimals)}`
      : null;

  return (
    <Fragment>
      <tr className={staged ? 'edited' : undefined}>
        <td className="name">
          <div className="name">{row.name}</div>
        </td>
        <td className="notes">
          {meta !== null && meta.help !== null && (
            <div className="help" title={meta.help}>
              {meta.help}
            </div>
          )}
          {meta !== null && meta.help === null && (
            // `help: null` is "not read yet", never "the board has none" — an
            // empty help string is the board saying it has none, and the two
            // render differently on purpose.
            <div className="help muted">help not read</div>
          )}
        </td>
        <td className="range">
          {bounds ?? (meta === null ? 'no description' : 'no range stated')}
          {bounds === null && (
            <div className="why">{whyNoRange(row, low, high)}</div>
          )}
        </td>
        <td className="board">
          <span className="value">{row.boardValue ?? '—'}</span>
          {row.changedFromBoot && <span className="changed">changed</span>}
        </td>
        <td className="value">
          <div className="entry">
            <input
              className="bp5-input bp5-small"
              type="text"
              inputMode="decimal"
              aria-label={`${row.name}, new value`}
              value={row.edited ?? ''}
              placeholder={row.boardValue ?? ''}
              disabled={busy}
              onChange={(event) => actions.edit(row.index, event.target.value)}
              onKeyDown={(event) => {
                if (event.key === 'Enter') actions.write(row.index);
                if (event.key === 'Escape') actions.revert(row.index);
              }}
            />
          </div>
        </td>
        <td className="actions">
          {/* "Read" is a clean-state verb: it re-reads the board value, which is
              still visible one column over. Once a value is staged the row has
              only two jobs — send it or throw it away — so the read button steps
              aside and returns after either happens. Three verbs side by side
              here (Read · Discard · Send) measured 175px, wider than any column
              the table can afford without starving "What it is". */}
          {!staged && (
            <>
              <Button
                minimal small
                onClick={() => actions.reread(row.index)}
                disabled={busy}
                title="Read it back from the board"
              >
                Read
              </Button>
              {/* Deliberately in the *clean* cell and not beside Discard/Send.
                  A reset is not a way to finish editing a value — it discards
                  what the board holds, not what you typed — and pairing it with
                  Send would put a build default one mis-click from a change the
                  person was about to make. Two buttons in this cell either way,
                  which is what the column can afford. */}
              <Button
                minimal small
                onClick={() => actions.reset(row.index)}
                disabled={busy || !permission.allowed || resetReason !== null}
                title={
                  resetReason ??
                  (permission.allowed
                    ? 'Put this parameter back to the value this build was compiled with'
                    : permission.reason)
                }
              >
                Reset
              </Button>
            </>
          )}
          {staged && (
            <Button
              minimal small
              onClick={() => actions.revert(row.index)}
              disabled={busy}
              title="Discard what you typed"
            >
              Discard
            </Button>
          )}
          {staged && (
            <Button
              small
              onClick={() => actions.write(row.index)}
              disabled={busy || !permission.allowed}
              title={permission.allowed ? undefined : permission.reason}
            >
              Send
            </Button>
          )}
        </td>
      </tr>
      {row.write !== null && (
        <tr className="detail">
          <td colSpan={6}>
            <Trace row={row} />
          </td>
        </tr>
      )}
    </Fragment>
  );
}

export function DiagnosticsPanel({
  snapshot,
  events,
}: {
  snapshot: SessionSnapshot;
  events: readonly SessionEvent[];
}) {
  const { counts } = snapshot;
  return (
    <Panel title="Diagnostics" note={`${events.length} events`}>
      <div className="counters">
        <Counter n={counts.frames} k="frames" />
        <Counter n={counts.writes} k="sent" />
        <Counter n={counts.telemetry} k="pushed" />
        <Counter n={counts.issues} k="refused" bad={counts.issues > 0} />
      </div>
      <hr className="rule" />
      {events.length === 0 ? (
        <p className="muted small">Nothing has happened yet.</p>
      ) : (
        <ul className="events">
          {events.slice(0, 60).map((event, index) => (
            <li key={`${event.atMs}-${index}`} className={event.level}>
              <span className="at">{formatClock(event.atMs)}</span>
              <span className="text">{event.text}</span>
            </li>
          ))}
        </ul>
      )}
    </Panel>
  );
}

function Counter({ n, k, bad = false }: { n: number; k: string; bad?: boolean }) {
  return (
    <div className="counter">
      <div className="n" style={bad ? { color: 'var(--danger-ink)' } : undefined}>
        {n}
      </div>
      <div className="k">{k}</div>
    </div>
  );
}

// ---- formatting -----------------------------------------------------------

export function fmt(value: number, decimals: number): string {
  return value.toFixed(decimals);
}

function formatDuration(ms: number): string {
  const seconds = Math.max(0, Math.round(ms / 1000));
  if (seconds < 60) return `${seconds}s`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m ${seconds % 60}s`;
  return `${Math.floor(minutes / 60)}h ${minutes % 60}m`;
}

function formatClock(ms: number): string {
  const date = new Date(ms);
  const pad = (n: number) => String(n).padStart(2, '0');
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
}
