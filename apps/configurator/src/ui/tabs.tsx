import { Button } from '@blueprintjs/core';
import type { ReactNode } from 'react';
import { Feature, reasonFor } from '../protocol/features';
import type { SessionSnapshot } from '../session/types';
import { AttitudeView } from './views/Attitude';
import { OutputsView, Sticks } from './views/Controls';
import { ParametersView } from './views/Parameters';
import {
  DiagnosticsPanel,
  IdentityPanel,
  LivePanel,
  Panel,
  ReceiverPanel,
  SensorsPanel,
  type ParameterActions,
} from './panels';

/**
 * The rail.
 *
 * Before this file the workspace was three `if` branches in `App.tsx` and no
 * registry at all: every panel existed whether or not anything could back it,
 * and a person looking at a bare page had no way to tell "your firmware is
 * older than this app" from "the app forgot to render something".
 *
 * **The rule this file exists for: a tab that cannot be backed is still in the
 * rail, disabled, carrying the reason.** The reason names the opcode, not the
 * bit — a person reading "does not answer `rc channels`" knows what to search
 * for in the firmware; a person reading "missing bit 4" does not.
 *
 * There are two ways a tab can be unusable and they are different facts, so
 * they get different sentences:
 *
 *  - **The board cannot.** Its capability word says so, or it predates the
 *    word. That is a reading, and `reasonFor` is the one place that knows the
 *    difference between an absent word and an absent bit.
 *  - **This app cannot yet.** The opcode is specified in
 *    `firmware/docs/16-protocol.md` and this view was never written. A tab
 *    enabled by a board capability whose view does not exist would be worse
 *    than a disabled one: it would open onto nothing and read as a firmware
 *    fault.
 *
 * Both are reported through the same field so the rail has one way to say "not
 * now". A tab is listed only when at least one of the two is the honest
 * answer, and the sentence says which.
 */

export type Family = 'aerialkit' | 'msp' | 'mavlink';

/** Everything a tab's view may read. Passed whole rather than in pieces, so a
 *  new tab does not need a new prop threaded through `App`. */
export interface WorkspaceProps {
  readonly snapshot: SessionSnapshot;
  readonly actions: ParameterActions;
  readonly busy: boolean;
  readonly streamHz: number;
  readonly setStreamHz: (hz: number) => void;
  /** Asks the board for the stream. Separate from `setStreamHz`, which only
   *  moves the picker — a view that ran the request from the setter would
   *  subscribe on every keystroke of a number field. */
  readonly onStream: (hz: number) => void;
  readonly nowMs: number;
  /** A user request to open and review staged parameter edits. */
  readonly reviewEdits?: number;
  readonly onReviewEditsConsumed?: () => void;
  readonly onRefresh: () => void;
  readonly onSave: () => void;
  /**
   * Whether the Receiver tab should be polling `rc channels`.
   *
   * On the props rather than inside the panel because the panel is not allowed
   * to own a session, and because the caller is the only thing that knows when
   * the tab is on screen. Memoised by `App`: a new function identity each
   * render would have the panel stop and restart its own polling.
   */
  readonly onWatchRc: (on: boolean) => void;
  /** One `rc channels` exchange now, for the panel's own button. */
  readonly onReadRc: () => void;
  /**
   * Whether the Sensors tab should be polling `sensor info`. Same contract as
   * `onWatchRc`: on the props, because the panel may not own a session and only
   * the caller knows whether the tab is on screen.
   */
  readonly onWatchSensors: (on: boolean) => void;
  /** One full round over every sensor topic now. */
  readonly onReadSensors: () => void;
  /** Whether a view is drawing the attitude live (speeds up the status poll).
   *  Memoised by `App`, for the same reason as `onWatchRc`. */
  readonly onWatchAttitude: (on: boolean) => void;
}

/** The board's `airframe` parameter, once the table has been read. */
export function airframeOf(snapshot: SessionSnapshot): number | null {
  const row = snapshot.parameters.find((candidate) => candidate.name === 'airframe');
  const value = row?.boardValue == null ? NaN : Number(row.boardValue);
  return Number.isInteger(value) ? value : null;
}

export interface Tab {
  readonly id: string;
  readonly label: string;
  /** Null when the tab can be opened; otherwise *why it cannot*, as a
   *  sentence. Never a bare absence. */
  readonly unavailable?: (snapshot: SessionSnapshot) => string | null;
  readonly families: readonly Family[];
  readonly render: (props: WorkspaceProps) => ReactNode;
}

/**
 * A tab this app has not written the view for.
 *
 * The opcode is named on purpose, because the useful thing for a reader who
 * wants the tab is to know that the wire side is specified and the missing
 * half is here. The board's own answer is appended when it has one, so a
 * person with old firmware is not told to wait for an app release that would
 * not help them.
 */
function notBuilt(opcode: string, feature?: Feature) {
  return (snapshot: SessionSnapshot): string => {
    const base = `this app has no view for \`${opcode}\` yet - see firmware/docs/16-protocol.md for the protocol`;
    if (feature === undefined) return base;
    const board = reasonFor(snapshot.identity?.features ?? null, feature, opcode);
    // `reasonFor` returns null when the board *does* answer it. Then the app is
    // the only half missing, and saying so is the whole difference between
    // this tab and a broken one.
    return board === null ? base : `${base}. ${board}`;
  };
}

export const TABS: readonly Tab[] = [
  {
    id: 'attitude',
    label: 'Attitude',
    families: ['aerialkit'],
    // Opens for any board: STATUS is in every protocol version, so there is no
    // capability to wait for. Without an IMU the board reports level, and the
    // Sensors tab is where that is explained.
    render: ({ snapshot, onWatchAttitude }) => (
      <AttitudeView live={snapshot.live} airframe={airframeOf(snapshot)} onWatch={onWatchAttitude} />
    ),
  },
  {
    id: 'live',
    label: 'Live',
    families: ['aerialkit'],
    render: ({ snapshot, busy, streamHz, setStreamHz, onStream }) => (
      <div className="stack" style={{ marginTop: 0 }}>
        <LivePanel live={snapshot.live} telemetry={snapshot.telemetry} />
        <Panel title="Stream" note={snapshot.transport.label}>
          <div className="row">
            <label htmlFor="hz">rate</label>
            <select
              id="hz"
              style={{ width: 'auto' }}
              value={streamHz}
              onChange={(event) => setStreamHz(Number(event.target.value))}
            >
              {[0, 1, 5, 10, 20, 50].map((hz) => (
                <option key={hz} value={hz}>
                  {hz === 0 ? 'off' : `${hz} Hz`}
                </option>
              ))}
            </select>
            <Button onClick={() => onStream(streamHz)} disabled={busy || snapshot.phase === 'failed'}>
              Apply
            </Button>
          </div>
          <p className="small muted" style={{ marginTop: 10 }}>
            {snapshot.telemetry.agreedHz === null
              ? 'Off. Pick a rate and apply to start telemetry.'
              : snapshot.telemetry.agreedHz === 0
                ? 'The board declined to stream on this link (the console link cannot).'
                : `Streaming at ${snapshot.telemetry.agreedHz} Hz · ${snapshot.telemetry.received} frames received.`}
          </p>
        </Panel>
      </div>
    ),
  },
  {
    id: 'parameters',
    label: 'Parameters',
    families: ['aerialkit'],
    render: ({ snapshot, actions, busy, reviewEdits, onReviewEditsConsumed }) => <ParametersView snapshot={snapshot} actions={actions} busy={busy} reviewEdits={reviewEdits} onReviewEditsConsumed={onReviewEditsConsumed} />,
  },
  {
    id: 'identity',
    label: 'Identity',
    families: ['aerialkit'],
    render: ({ snapshot, nowMs }) => (
      <IdentityPanel
        identity={snapshot.identity}
        transport={snapshot.transport}
        phase={snapshot.phase}
        openedAtMs={snapshot.openedAtMs}
        nowMs={nowMs}
      />
    ),
  },
  {
    id: 'diagnostics',
    label: 'Diagnostics',
    families: ['aerialkit'],
    render: ({ snapshot }) => (
      <DiagnosticsPanel snapshot={snapshot} events={snapshot.events} />
    ),
  },

  // ---- written later, in the order they are planned ----------------------
  //
  // Each names its opcode. Nothing appears here that this app has a view for:
  // a disabled tab is a statement about a missing half, and a `render` that
  // returned null would make that statement false.
  {
    id: 'receiver',
    label: 'Receiver',
    families: ['aerialkit'],
    // No `unavailable` field, deliberately: this tab opens for any board. The
    // board is still asked, because a firmware that predates the capability
    // word answers `rc channels` with `0x7F` and the panel says so in the
    // board's own terms rather than this rail guessing on its behalf — the same
    // rule the parameter table's metadata walk follows. (`tools/check-tabs.py`
    // reads an `unavailable` field as "this tab does not open", so absent is
    // the only way to say this one does.)
    render: ({ snapshot, busy, nowMs, onWatchRc, onReadRc }) => (
      <div className="stack" style={{ marginTop: 0 }}>
      <Sticks rc={snapshot.rc} />
      <ReceiverPanel
        rc={snapshot.rc}
        nowMs={nowMs}
        busy={busy}
        onWatch={onWatchRc}
        onRead={onReadRc}
      />
      </div>
    ),
  },
  {
    id: 'sensors',
    label: 'Sensors',
    families: ['aerialkit'],
    // Opens for any board, like the Receiver, and for the same reason: the
    // board is asked and the panel reports its answer in the board's own terms.
    // A firmware that predates the capability word answers `sensor info` with
    // `0x7F`, and the panel says *that* — which is the useful sentence, and one
    // this rail could not produce without guessing on the board's behalf.
    //
    // `App` does not poll a board whose word does not claim the bit (see
    // `watchSensors`), so an old board costs nothing here beyond the first
    // answer that says so.
    render: ({ snapshot, busy, nowMs, onWatchSensors, onReadSensors }) => (
      <SensorsPanel
        sensors={snapshot.sensors}
        nowMs={nowMs}
        busy={busy}
        onWatch={onWatchSensors}
        onRead={onReadSensors}
      />
    ),
  },
  {
    id: 'motors',
    label: 'Motors',
    families: ['aerialkit'],
    // The live outputs come from STATUS, which every board answers. Testing a
    // motor needs `output info` and a motor-test opcode that do not exist yet;
    // the view says so instead of offering a button.
    render: ({ snapshot, onWatchAttitude }) => (
      <OutputsView live={snapshot.live} airframe={airframeOf(snapshot)} onWatch={onWatchAttitude} />
    ),
  },
  {
    id: 'blackbox',
    label: 'Blackbox',
    families: ['aerialkit'],
    unavailable: () => 'this app has no log viewer yet — `log get` already carries the records, so this one needs no firmware change',
    render: () => null,
  },
  {
    id: 'backup',
    label: 'Backup',
    families: ['aerialkit'],
    unavailable: () =>
      'back up and restore of the parameter table are in Parameters (a JSON file); replaying the firmware’s own `name=value` lines is not built yet',
    render: () => null,
  },
  {
    id: 'preflight',
    label: 'Preflight',
    families: ['aerialkit'],
    unavailable: notBuilt('preflight', Feature.PREFLIGHT),
    render: () => null,
  },
  {
    id: 'mission',
    label: 'Mission',
    families: ['aerialkit'],
    unavailable: notBuilt('mission', Feature.MISSION),
    render: () => null,
  },
  {
    id: 'firmware',
    label: 'Firmware',
    families: ['aerialkit'],
    unavailable: () => 'this app has no firmware panel yet — it will show what the board reported and why flashing is refused, and nothing more',
    render: () => null,
  },
];

/** The tabs for one family. `msp` and `mavlink` have no rail of their own
 *  yet; their workspaces are drawn whole. */
export function tabsFor(family: Family): readonly Tab[] {
  return TABS.filter((tab) => tab.families.includes(family));
}

/** Whether a tab can be opened, and the reason it cannot. */
export function tabState(tab: Tab, snapshot: SessionSnapshot): { readonly open: boolean; readonly why: string | null } {
  const why = tab.unavailable?.(snapshot) ?? null;
  return { open: why === null, why };
}
