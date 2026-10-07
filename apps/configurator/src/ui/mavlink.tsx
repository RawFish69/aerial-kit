import { Button, HTMLTable } from '@blueprintjs/core';
import { useCallback, useState, useSyncExternalStore } from 'react';

import { MAV_LIMITATIONS, type MavBoard, type MavSnapshot } from '../session/mavlink';
import { Panel, fmt } from './panels';

/**
 * A vehicle that is not ours.
 *
 * The third workspace, and the one where what is *missing* matters most. A
 * MAVLink vehicle will accept an arm command, a mode change, a mission upload
 * and a parameter write, and this page offers none of them — not as disabled
 * buttons, which would suggest they are one permission away, but as nothing at
 * all. What is left is a page that watches: what the vehicle says it is, what
 * it is streaming, what it is doing, and what its parameter table contains.
 *
 * The armed banner is the same three states as the other two pages, in the same
 * colours, for the same reason. A vehicle whose heartbeat stopped arriving 30
 * seconds ago is not a disarmed vehicle, and this page refuses to draw it as
 * one.
 */

function useMav(board: MavBoard | null): MavSnapshot | null {
  const subscribe = useCallback(
    (listener: () => void) => (board === null ? () => {} : board.subscribe(listener)),
    [board],
  );
  const get = useCallback(() => (board === null ? null : board.snapshot), [board]);
  return useSyncExternalStore(subscribe, get, get);
}

/** Message ids this page offers to ask for at a higher rate, and why those. */
const RATES: ReadonlyArray<{ msgid: number; label: string }> = [
  { msgid: 30, label: 'Attitude' },
  { msgid: 33, label: 'Position' },
  { msgid: 74, label: 'Airspeed and heading' },
  { msgid: 65, label: 'RC channels' },
];

export function MavWorkspace({ board }: { board: MavBoard }) {
  const snapshot = useMav(board);
  const [busy, setBusy] = useState(false);
  const [hz, setHz] = useState(4);

  if (snapshot === null) return null;
  const { identity, live, firmware } = snapshot;

  const cls =
    live.armed === 'armed' ? 'is-armed' : live.armed === 'disarmed' ? 'is-disarmed' : 'is-unknown';
  const word =
    live.armed === 'armed'
      ? 'Armed'
      : live.armed === 'disarmed'
        ? 'Disarmed'
        : 'Armed state unknown';

  const read = async (): Promise<void> => {
    setBusy(true);
    try {
      await board.readParameters();
    } finally {
      setBusy(false);
    }
  };

  const unread = snapshot.seen.filter((item) => item.name === null);

  return (
    <>
      <div className={`armed ${cls}`}>
        <div className="headline">
          <span>{word}</span>
          <span className="implication">
            {live.lastSeenMs === null
              ? 'no heartbeat has arrived yet'
              : `from a heartbeat ${Math.round(live.lastSeenMs / 1000)} s ago`}
          </span>
        </div>
        <p className="because">
          {live.stale || live.lastSeenMs === null
            ? 'The heartbeat that carries this state has stopped arriving, so the vehicle’s present state is unknown. It is shown as unknown rather than as disarmed — the last thing it said is not the last thing that happened.'
            : 'Read from the heartbeat the vehicle sends on its own. This app has no way to arm or disarm it, which is why nothing here is gated on this state.'}
        </p>
      </div>

      <Panel title="What this is" note="from the vehicle’s own heartbeat, heard not asked for">
        {identity === null ? (
          <p className="muted">the vehicle has not announced itself</p>
        ) : (
          <dl className="kv">
            <Fact label="autopilot">{identity.autopilotName}</Fact>
            <Fact label="airframe">{identity.typeName}</Fact>
            <Fact label="system">
              {identity.systemId}.{identity.componentId}
            </Fact>
            <Fact label="MAVLink">
              v{identity.framing} on the wire — the `mavlink_version` field reads{' '}
              {identity.mavlinkVersion}, which is a protocol constant and not a fact about this link
            </Fact>
            <Fact label="mode">custom mode {identity.customMode}, base mode {identity.baseMode}</Fact>
            <Fact label="status">state {identity.systemStatus}</Fact>
            {firmware !== null && (
              <>
                <Fact label="flight stack">
                  {firmware.flightSwVersion || '?'} (middleware{' '}
                  {firmware.middlewareSwVersion || '?'}, OS {firmware.osSwVersion || '?'})
                </Fact>
                <Fact label="board">
                  vendor {firmware.vendorId}, product {firmware.productId}, board{' '}
                  {firmware.boardVersion || '?'}
                </Fact>
                <Fact label="uid">{firmware.uid || '?'}</Fact>
              </>
            )}
          </dl>
        )}
        {firmware === null && (
          <p className="muted small">The vehicle has not sent AUTOPILOT_VERSION.</p>
        )}
      </Panel>

      <Panel title="Live" note="the vehicle streams this; nothing above asked for it">
        <dl className="kv">
          <Fact label="attitude">
            {live.attitude === null
              ? '?'
              : `${fmt(live.attitude.rollDeg, 1)}° roll, ${fmt(live.attitude.pitchDeg, 1)}° pitch, ${fmt(live.attitude.yawDeg, 1)}° yaw`}
          </Fact>
          <Fact label="position">
            {live.position === null
              ? '?'
              : `${fmt(live.position.latDeg, 7)}, ${fmt(live.position.lonDeg, 7)} at ${fmt(live.position.altM, 1)} m`}
          </Fact>
          <Fact label="heading">
            {live.position === null || live.position.hdgDeg === null
              ? 'the vehicle did not say'
              : `${fmt(live.position.hdgDeg, 0)}°`}
          </Fact>
          <Fact label="velocity">
            {live.position === null
              ? '?'
              : `${fmt(live.position.vxCms / 100, 1)}, ${fmt(live.position.vyCms / 100, 1)}, ${fmt(live.position.vzCms / 100, 1)} m/s`}
          </Fact>
          <Fact label="airspeed">
            {live.vfrHud === null
              ? '?'
              : `${fmt(live.vfrHud.airspeedMs, 1)} m/s air, ${fmt(live.vfrHud.groundspeedMs, 1)} m/s ground, throttle ${fmt(live.vfrHud.throttlePct, 0)}%`}
          </Fact>
          <Fact label="battery">
            {live.sysStatus === null
              ? '?'
              : `${fmt(live.sysStatus.voltageV, 2)} V, ${fmt(live.sysStatus.currentA, 1)} A${
                  live.sysStatus.batteryRemainingPct === null
                    ? ', remaining not reported'
                    : `, ${live.sysStatus.batteryRemainingPct}% remaining`
                }`}
          </Fact>
          <Fact label="GPS">
            {live.gps === null
              ? '?'
              : `${live.gps.fixName}, ${live.gps.satellitesVisible} satellites`}
          </Fact>
          <Fact label="RC">
            {live.rc === null ? '?' : `${live.rc.chancount} channels, rssi ${live.rc.rssi}`}
          </Fact>
          <Fact label="servos">
            {live.servos === null
              ? '?'
              : live.servos.servos.slice(0, 8).map((value) => `${value}`).join('  ')}
          </Fact>
        </dl>
        <p className="muted small">? = not sent by the vehicle. A heading of 0xffff means no heading, not north.</p>
      </Panel>

      <Panel
        title="Parameters"
        note="read from the vehicle, never assumed"
        actions={
          <Button type="button" onClick={() => void read()} disabled={busy || snapshot.phase !== 'ready'}>
            {busy ? 'Reading…' : 'Read the table'}
          </Button>
        }
      >
        {snapshot.parameters.length === 0 ? (
          <p className="muted small">Nothing read yet.</p>
        ) : (
          <HTMLTable compact striped className="data-table">
            <tbody>
              {snapshot.parameters.map((parameter) => (
                <tr key={parameter.id}>
                  <td className="name">{parameter.id}</td>
                  <td>{fmt(parameter.value, 6)}</td>
                  <td className="muted small">
                    index {parameter.index} of {snapshot.parameters.length}, type {parameter.type}
                  </td>
                </tr>
              ))}
            </tbody>
          </HTMLTable>
        )}
      </Panel>

      <Panel
        title="Telemetry rates"
        note="the only thing this app sends a vehicle that changes anything"
      >
        <div className="row">
          <label className="small muted">
            rate
            <input
              type="number"
              min={1}
              max={50}
              value={hz}
              aria-label="stream rate in hertz"
              onChange={(event) => setHz(Number(event.target.value))}
              style={{ width: 70, marginLeft: 8 }}
            />
            Hz
          </label>
          {RATES.map((rate) => (
            <Button
              key={rate.msgid}
              type="button"
              minimal
              disabled={snapshot.phase !== 'ready'}
              onClick={() => board.requestStream(rate.msgid, hz)}
            >
              {rate.label} at {hz} Hz
            </Button>
          ))}
        </div>
        <p className="muted small">Changes only how often the vehicle reports. Rate 0 is refused.</p>
        {snapshot.acks.length > 0 && (
          <HTMLTable compact striped className="data-table">
            <tbody>
              {snapshot.acks.map((ack, index) => (
                <tr key={`${ack.command}-${index}`}>
                  <td className="name">command {ack.command}</td>
                  <td>{ack.resultName}</td>
                  <td className="muted small">progress {ack.progress}%</td>
                </tr>
              ))}
            </tbody>
          </HTMLTable>
        )}
      </Panel>

      <Panel title="What the vehicle is saying" note="counted, including what this app cannot read">
        {snapshot.seen.length === 0 ? (
          <p className="muted">nothing heard yet</p>
        ) : (
          <HTMLTable compact striped className="data-table">
            <tbody>
              {snapshot.seen.map((item) => (
                <tr key={item.msgid}>
                  <td className="name">{item.name ?? `message ${item.msgid}`}</td>
                  <td>{item.count}</td>
                  <td className="muted small">
                    {item.name === null ? (
                      <span className="unknown">
                        no definition in this app — its checksum cannot be checked, so it is never
                        parsed
                      </span>
                    ) : (
                      ''
                    )}
                  </td>
                </tr>
              ))}
            </tbody>
          </HTMLTable>
        )}
        <p className="muted small">
          {unread.length === 0
            ? `${snapshot.counts.frames} frames read, ${snapshot.counts.issues} refused.`
            : `${unread.length} message type(s) have no definition here and are counted, not decoded.`}
          {snapshot.counts.signed > 0 && ` ${snapshot.counts.signed} frames were signed.`}
        </p>
      </Panel>

      <Panel title="What this connection cannot do" note="stated, not implied">
        <ul className="list-plain">
          {MAV_LIMITATIONS.map((line) => (
            <li key={line}>{line}</li>
          ))}
        </ul>
      </Panel>

      <Panel title="Trace" note="what happened, in order">
        <ol className="list-plain">
          {snapshot.events.map((event, index) => (
            <li key={`${event.atMs}-${index}`} className={event.level}>
              {event.text}
            </li>
          ))}
        </ol>
      </Panel>
    </>
  );
}

function Fact({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <>
      <dt>{label}</dt>
      <dd>{children}</dd>
    </>
  );
}
