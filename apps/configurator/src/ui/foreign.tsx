import { Button, HTMLTable } from '@blueprintjs/core';
import { useCallback, useState, useSyncExternalStore } from 'react';

import { FOREIGN_LIMITATIONS, type ForeignSnapshot, type MspBoard } from '../session/foreign';
import { Panel, fmt } from './panels';

/**
 * A board that is not ours.
 *
 * It looks different from the AerialKit workspace on purpose, and not only
 * because there is less of it. The AerialKit page is built around writing — the
 * four facts, the gate, the staged values. This page has no write on it at all,
 * because MSP has no command here that would be safe to send to a firmware this
 * app cannot reason about. A read-only board shown in a writing interface would
 * be the wrong shape twice over: buttons that do nothing, and a person left to
 * guess which of them are the dead ones.
 *
 * The armed banner is the same component and the same three states as our own
 * page. That is the one thing that must not differ: a board whose arming state
 * is unknown has to look unknown here too.
 */

function useForeign(board: MspBoard | null): ForeignSnapshot | null {
  const subscribe = useCallback(
    (listener: () => void) => (board === null ? () => {} : board.subscribe(listener)),
    [board],
  );
  const get = useCallback(() => (board === null ? null : board.snapshot), [board]);
  return useSyncExternalStore(subscribe, get, get);
}

export function ForeignWorkspace({ board }: { board: MspBoard }) {
  const snapshot = useForeign(board);
  const [name, setName] = useState('');
  const [busy, setBusy] = useState(false);

  if (snapshot === null) return null;
  const { identity, live } = snapshot;

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
      await board.lookup(name);
      await board.describe(name);
    } finally {
      setBusy(false);
    }
  };

  return (
    <>
      <div className={`armed ${cls}`}>
        <div className="headline">
          <span>{word}</span>
          <span className="implication">
            {live.lastSeenMs === null
              ? 'no status frame has arrived yet'
              : `from a status frame ${Math.round(live.lastSeenMs / 1000)} s ago`}
          </span>
        </div>
        <p className="because">
          {live.stale || live.lastSeenMs === null
            ? 'Nothing here is written to this board, so this state gates nothing. It is shown because a person reading the numbers above is entitled to know how old they are.'
            : 'Read from the board’s own status frame. This app sends this board nothing that could change it.'}
        </p>
      </div>

      <Panel
        title="What this is"
        note="read from the board, never assumed"
      >
        {identity === null ? (
          <p className="muted">the board did not describe itself</p>
        ) : (
          <dl className="kv">
            <Fact label="firmware">
              {identity.variant.known
                ? identity.variant.variant
                : `${identity.variant.variant} — a firmware this app has no name table for`}
            </Fact>
            <Fact label="release">{identity.release.version || '?'}</Fact>
            <Fact label="board">
              {identity.board === null
                ? '?'
                : `${identity.board.boardName} (${identity.board.targetName})`}
            </Fact>
            <Fact label="identifier">{identity.board?.boardIdentifier ?? '?'}</Fact>
            <Fact label="manufacturer">{identity.board?.manufacturer ?? '?'}</Fact>
            <Fact label="MSP">
              {identity.api.apiMajor}.{identity.api.apiMinor} — protocol version{' '}
              {identity.api.protocolVersion}
            </Fact>
          </dl>
        )}
      </Panel>

      <Panel title="Live" note="the reads this app is allowed to make">
        <dl className="kv">
          <Fact label="attitude">
            {live.attitude === null
              ? '?'
              : `${fmt(live.attitude.rollDeg, 1)}° roll, ${fmt(live.attitude.pitchDeg, 1)}° pitch, ${fmt(live.attitude.yawDeg, 1)}° yaw`}
          </Fact>
          <Fact label="battery">
            {live.analog === null
              ? '?'
              : `${fmt(live.analog.volts, 1)} V, ${live.analog.mahDrawn} mAh drawn`}
          </Fact>
          <Fact label="position">
            {live.gps === null || live.gps.fixType === 0
              ? 'no fix'
              : `${live.gps.satellites} satellites, ${live.gps.lat / 1e7}, ${live.gps.lon / 1e7}`}
          </Fact>
          <Fact label="sensors">
            {live.status === null
              ? '?'
              : Object.entries(live.status.sensors)
                  .filter(([, present]) => present)
                  .map(([sensor]) => sensor)
                  .join(', ') || 'none reported'}
          </Fact>
          <Fact label="loop">{live.status === null ? '?' : `${live.status.cycleTimeUs} µs`}</Fact>
          <Fact label="frames read">{snapshot.counts.frames}</Fact>
        </dl>
        <p className="muted small">
          A field this board does not carry shows a question mark. It never shows a zero — a
          latitude of zero is a real place in the Atlantic, and "the board did not say" is a
          different answer.
        </p>
      </Panel>

      <Panel
        title="Settings"
        note="by name, because MSP has no request that lists them"
      >
        <div className="row">
          <input
            type="text"
            value={name}
            placeholder="failsafe_throttle"
            aria-label="setting name"
            onChange={(event) => setName(event.target.value)}
            onKeyDown={(event) => {
              if (event.key === 'Enter') void read();
            }}
          />
          <Button type="button" onClick={() => void read()} disabled={busy || name.trim() === ''}>
            {busy ? 'Reading…' : 'Read'}
          </Button>
        </div>
        {snapshot.settings.length === 0 ? (
          <p className="muted small">
            Nothing read yet. Type a name and ask. This is not a limitation of the page — there is
            genuinely no command in MSP that returns the list, which is why every ground station
            ships its own name table per release.
          </p>
        ) : (
          <HTMLTable compact striped className="data-table">
            <tbody>
              {snapshot.settings.map((setting) => (
                <tr key={setting.name}>
                  <td className="name">{setting.name}</td>
                  <td>
                    {setting.refusal !== null ? (
                      <span className="unknown">{setting.refusal}</span>
                    ) : setting.value === '' || setting.value === null ? (
                      <span className="unknown">the board did not give a value</span>
                    ) : (
                      setting.value
                    )}
                  </td>
                  <td className="muted small">
                    {setting.info === null
                      ? ''
                      : (['pgn', 'type', 'min', 'max', 'default'] as const)
                          .filter((key) => setting.info?.fields[key] !== undefined)
                          .map((key) => `${key}=${setting.info?.fields[key]}`)
                          .join('  ')}
                  </td>
                </tr>
              ))}
            </tbody>
          </HTMLTable>
        )}
      </Panel>

      <Panel title="What this connection cannot do" note="stated, not implied">
        <ul className="list-plain">
          {FOREIGN_LIMITATIONS.map((line) => (
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
