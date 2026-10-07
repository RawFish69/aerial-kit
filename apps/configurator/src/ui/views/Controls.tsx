import { useEffect } from 'react';
import { RcFlag, flagSet } from '../../protocol/constants';
import type { LiveView, RcView } from '../../session/types';
import { AIRFRAME_NAMES } from './airframes';

/**
 * The two gimbals, drawn as a handset shows them (Mode 2: throttle and yaw on
 * the left stick, pitch and roll on the right). Positions are the firmware's
 * decoded sticks, per-mille — not the raw channels — so what moves here is
 * what the flight core would act on.
 */
export function Sticks({ rc }: { rc: RcView }) {
  const state = rc.state;
  const decoded = state !== null && flagSet(state.flags, RcFlag.DECODED);
  const [roll = 0, pitch = 0, yaw = 0, throttle = 0] = decoded ? state!.sticks : [];
  // Throttle is 0..1000; the gimbal's vertical axis is centred, so map it.
  const throttleY = throttle / 500 - 1;

  return (
    <div className="sticks">
      <Gimbal label="Throttle and yaw" x={yaw / 1000} y={throttleY} live={decoded} />
      <Gimbal label="Pitch and roll" x={roll / 1000} y={pitch / 1000} live={decoded} />
      {!decoded && (
        <p className="muted small sticks-note">
          {state === null
            ? 'Waiting for the board to answer.'
            : 'The board has not decoded a frame from a receiver, so there are no stick positions to draw.'}
        </p>
      )}
    </div>
  );
}

function Gimbal({ label, x, y, live }: { label: string; x: number; y: number; live: boolean }) {
  const size = 150;
  const c = size / 2;
  const px = c + Math.max(-1, Math.min(1, x)) * (c - 12);
  const py = c - Math.max(-1, Math.min(1, y)) * (c - 12);
  return (
    <figure className="gimbal" aria-label={`${label}: ${live ? `x ${(x * 100).toFixed(0)}%, y ${(y * 100).toFixed(0)}%` : 'no input'}`}>
      <svg viewBox={`0 0 ${size} ${size}`} width={size} height={size}>
        <rect x="1" y="1" width={size - 2} height={size - 2} rx="10" fill="var(--sunken)" stroke="var(--rule)" />
        <line x1={c} x2={c} y1="10" y2={size - 10} stroke="var(--rule)" />
        <line y1={c} y2={c} x1="10" x2={size - 10} stroke="var(--rule)" />
        {live && (
          <>
            <line x1={c} y1={c} x2={px} y2={py} stroke="var(--select)" strokeOpacity="0.4" strokeWidth="2" />
            <circle cx={px} cy={py} r="9" fill="var(--select)" />
          </>
        )}
      </svg>
      <figcaption>{label}</figcaption>
    </figure>
  );
}

/**
 * The outputs as the board reports them in STATUS, laid out where they sit on
 * the airframe. Read-only: driving a motor from this page needs the firmware's
 * `output info` / motor-test opcodes, which do not exist yet, and a configurator
 * that spun props from a guess is how fingers get cut.
 */
export function OutputsView({
  live,
  airframe,
  onWatch,
}: {
  live: LiveView;
  airframe: number | null;
  onWatch: (on: boolean) => void;
}) {
  useEffect(() => {
    onWatch(true);
    return () => onWatch(false);
  }, [onWatch]);

  const motors = live.status?.motors ?? [];
  const layout = motorLayout(airframe, motors.length);
  const armed = live.armed === 'armed';

  return (
    <div className="outputs">
      <section className="outputs-map" aria-label="Motor outputs on the airframe, seen from above">
        <svg viewBox="-130 -130 260 260" width="100%">
          <path d="M 0 -118 l -9 16 h 18 z" fill="var(--caution)" />
          <text x="0" y="-92" textAnchor="middle" className="map-label">front</text>
          {layout.map(({ x, y }, i) => {
            const level = motorLevel(motors[i] ?? 0);
            const r = 30;
            const circumference = 2 * Math.PI * r;
            return (
              <g key={i} transform={`translate(${x} ${y})`}>
                <line x1={-x} y1={-y} x2="0" y2="0" stroke="var(--rule-strong)" strokeWidth="6" />
                <circle r={r} fill="var(--surface)" stroke="var(--rule)" strokeWidth="6" />
                <circle
                  r={r}
                  fill="none"
                  stroke={armed ? 'var(--warning)' : 'var(--select)'}
                  strokeWidth="6"
                  strokeDasharray={`${level * circumference} ${circumference}`}
                  transform="rotate(-90)"
                />
                <text y="-2" textAnchor="middle" className="map-num">{Math.round(level * 100)}%</text>
                <text y="14" textAnchor="middle" className="map-label">M{i + 1}</text>
              </g>
            );
          })}
          <rect x="-20" y="-20" width="40" height="40" rx="6" fill="var(--raised)" stroke="var(--rule-strong)" />
        </svg>
      </section>

      <section className="outputs-bars" aria-label="Motor outputs as bars">
        {motors.length === 0 ? (
          <p className="muted">The board has not reported its outputs yet.</p>
        ) : (
          motors.map((value, i) => (
            <div className="vbar" key={i}>
              <div className="vbar-track">
                <div className={`vbar-fill ${armed ? 'armed' : ''}`} style={{ height: `${motorLevel(value) * 100}%` }} />
              </div>
              <span className="vbar-value">{Math.round(motorLevel(value) * 100)}%</span>
              <span className="vbar-label">Motor {i + 1}</span>
            </div>
          ))
        )}
        <div className="outputs-note">
          <p>
            <strong>{airframe === null ? 'Airframe not read yet' : AIRFRAME_NAMES[airframe] ?? `Airframe ${airframe}`}</strong>
            <span className="muted"> · motor outputs</span>
          </p>
          <p className="muted small">Motor test is not in the configurator yet. Use the board's console, props off.</p>
        </div>
      </section>
    </div>
  );
}

/** Top-down motor positions in the firmware's mixer order. */
function motorLayout(airframe: number | null, count: number): ReadonlyArray<{ x: number; y: number }> {
  const d = 78;
  switch (airframe) {
    case 1:
      return [{ x: -60, y: 70 }, { x: 60, y: 70 }].slice(0, Math.max(count, 2));
    case 7:
      return [{ x: 0, y: 80 }];
    case 2:
      return [{ x: -d, y: -d }, { x: d, y: -d }, { x: d, y: d }, { x: -d, y: d }];
    case 3:
      return [{ x: 0, y: 95 }, { x: 95, y: 0 }, { x: -95, y: 0 }, { x: 0, y: -95 }];
    case 6:
      return [{ x: 0, y: 95 }, { x: d, y: -60 }, { x: -d, y: -60 }];
    default:
      return [{ x: d, y: d }, { x: d, y: -d }, { x: -d, y: d }, { x: -d, y: -d }];
  }
}

/** STATUS carries each motor output as one byte, 0 to 254 (main.c scales by 254). */
function motorLevel(value: number): number {
  return Math.max(0, Math.min(254, value)) / 254;
}
