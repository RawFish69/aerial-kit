import { Button, Tag } from '@blueprintjs/core';
import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import type { LiveView } from '../../session/types';
import { buildAirframe } from './airframes';

/**
 * The attitude, drawn three ways: the airframe in 3D, a horizon and heading
 * instrument, and the numbers. All three come from the same STATUS reading —
 * the view never integrates or predicts, it only eases the model toward the
 * last angles the board reported, so a frozen board looks frozen.
 *
 * While this view is on screen the session polls STATUS at 20 Hz
 * (`watchAttitude`); it drops back to its normal rate when the view closes.
 */

const DEG = Math.PI / 180;

/** jsdom (the unit tests' browser) has no canvas or WebGL and logs an error for
 *  every attempt; the views draw nothing there instead. */
const NO_CANVAS = typeof navigator !== 'undefined' && /jsdom/i.test(navigator.userAgent);
const TRACE_SECONDS = 10;

interface Sample {
  readonly t: number;
  readonly roll: number;
  readonly pitch: number;
  readonly yaw: number;
}

export function AttitudeView({
  live,
  airframe,
  onWatch,
}: {
  live: LiveView;
  airframe: number | null;
  onWatch: (on: boolean) => void;
}) {
  const status = live.status;
  const [yawZero, setYawZero] = useState(0);
  const [followHeading, setFollowHeading] = useState(false);

  useEffect(() => {
    onWatch(true);
    return () => onWatch(false);
  }, [onWatch]);

  const roll = status?.rollDeg ?? 0;
  const pitch = status?.pitchDeg ?? 0;
  const yaw = wrap360((status?.yawDeg ?? 0) - yawZero);

  // The trace keeps its own history: a ring of the readings this view saw.
  const trace = useRef<Sample[]>([]);
  const lastStatus = useRef<unknown>(null);
  if (status !== null && status !== lastStatus.current) {
    lastStatus.current = status;
    const now = performance.now() / 1000;
    trace.current.push({ t: now, roll, pitch, yaw: wrap180(yaw) });
    while (trace.current.length > 0 && now - trace.current[0]!.t > TRACE_SECONDS) trace.current.shift();
  }

  const motors = status?.motors ?? [];

  return (
    <div className="attitude">
      <div className="viewport">
        <Scene
          roll={roll}
          pitch={pitch}
          yaw={followHeading ? 0 : yaw}
          airframe={airframe}
          motors={motors}
        />
        <div className="viewport-hud">
          <Tag intent={live.stale || status === null ? 'warning' : 'success'} minimal round>
            {status === null ? 'No attitude yet' : live.stale ? 'Board stopped answering' : 'Live, 20 Hz'}
          </Tag>
        </div>
        <div className="viewport-tools">
          <Button type="button" icon="locate" onClick={() => setYawZero(status?.yawDeg ?? 0)} disabled={status === null}>
            Zero heading
          </Button>
          <Button type="button" icon="compass" active={followHeading} aria-pressed={followHeading} onClick={() => setFollowHeading((on) => !on)}>
            {followHeading ? 'Showing roll and pitch only' : 'Hide heading'}
          </Button>
        </div>
      </div>

      <aside className="instruments">
        <Horizon roll={roll} pitch={pitch} heading={yaw} />
        <div className="readouts">
          <Readout label="Roll" value={roll} unit="°" hint="right wing down is positive" />
          <Readout label="Pitch" value={pitch} unit="°" hint="nose up is positive" />
          <Readout label="Heading" value={yaw} unit="°" hint={yawZero === 0 ? 'as the board reports it' : 'relative to the zero you set'} />
        </div>
        <TraceChart samples={trace.current} />
      </aside>
    </div>
  );
}

function Readout({ label, value, unit, hint }: { label: string; value: number; unit: string; hint: string }) {
  return (
    <div className="readout" title={hint}>
      <span className="readout-label">{label}</span>
      <span className="readout-value">
        {value.toFixed(1)}
        <small>{unit}</small>
      </span>
    </div>
  );
}

/** The 3D viewport. Owns its renderer; reads the latest angles from a ref so a
 *  new reading never re-creates the scene. */
function Scene({
  roll,
  pitch,
  yaw,
  airframe,
  motors,
}: {
  roll: number;
  pitch: number;
  yaw: number;
  airframe: number | null;
  motors: readonly number[];
}) {
  const host = useRef<HTMLDivElement>(null);
  const target = useRef({ roll, pitch, yaw, motors });
  target.current = { roll, pitch, yaw, motors };

  useEffect(() => {
    const element = host.current;
    if (element === null || NO_CANVAS) return;

    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    } catch {
      element.textContent = 'This browser cannot draw 3D (no WebGL). The instrument beside this still shows the attitude.';
      return;
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    element.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(38, 1, 0.1, 100);
    camera.position.set(1.9, 1.45, 2.5);
    camera.lookAt(0, 0, 0);

    scene.add(new THREE.HemisphereLight(0xffffff, 0x252a31, 1.2));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(3, 5, 2);
    scene.add(key);
    // A rim light from behind, so dark carbon still has an edge.
    const rim = new THREE.DirectionalLight(0x8abbff, 1.2);
    rim.position.set(-3, 2, -4);
    scene.add(rim);

    const grid = new THREE.GridHelper(8, 16, 0x5f6b7c, 0x383e47);
    grid.position.y = -0.9;
    scene.add(grid);

    // A fixed north arrow on the ground, so heading reads against something.
    const north = new THREE.Mesh(
      new THREE.ConeGeometry(0.08, 0.3, 3),
      new THREE.MeshBasicMaterial({ color: 0xbd6bbd }),
    );
    north.rotation.x = -Math.PI / 2;
    north.position.set(0, -0.88, -3.2);
    scene.add(north);

    const model = buildAirframe(airframe);
    const pivot = new THREE.Group();
    pivot.add(model.group);
    scene.add(pivot);

    // Frame the model by its size: a wing is twice as wide as a quad, and one
    // fixed camera cannot suit both.
    const radius = new THREE.Box3().setFromObject(model.group).getSize(new THREE.Vector3()).length() / 2;
    camera.position.set(1.9, 1.45, 2.5).normalize().multiplyScalar(radius * 2.6);
    camera.lookAt(0, 0, 0);

    const current = new THREE.Quaternion();
    const wanted = new THREE.Quaternion();
    const euler = new THREE.Euler(0, 0, 0, 'YXZ');
    let frame = 0;
    let last = performance.now();

    const resize = () => {
      const { clientWidth: w, clientHeight: h } = element;
      if (w === 0 || h === 0) return;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(element);
    resize();

    const reduceMotion = window.matchMedia?.('(prefers-reduced-motion: reduce)').matches ?? false;

    const tick = (now: number) => {
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      const t = target.current;
      // Body to three.js: yaw clockwise from above is -Y, pitch nose up is +X,
      // roll right wing down is -Z.
      euler.set(t.pitch * DEG, -t.yaw * DEG, -t.roll * DEG, 'YXZ');
      wanted.setFromEuler(euler);
      current.slerp(wanted, reduceMotion ? 1 : 1 - Math.exp(-dt * 18));
      pivot.quaternion.copy(current);

      model.props.forEach((prop, i) => {
        const level = Math.max(0, Math.min(1000, t.motors[i] ?? 0)) / 1000;
        if (level > 0 && !reduceMotion) prop.rotateY((i % 2 === 0 ? 1 : -1) * level * dt * 60);
      });

      renderer.render(scene, camera);
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);

    return () => {
      cancelAnimationFrame(frame);
      observer.disconnect();
      renderer.dispose();
      scene.traverse((object) => {
        const mesh = object as THREE.Mesh;
        mesh.geometry?.dispose();
        const mat = mesh.material as THREE.Material | THREE.Material[] | undefined;
        if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
        else mat?.dispose();
      });
      element.removeChild(renderer.domElement);
    };
  }, [airframe]);

  return <div className="scene" ref={host} aria-label="3D view of the airframe at the board's reported attitude" role="img" />;
}

/** A primary-flight-display style horizon with a roll scale and a heading strip. */
function Horizon({ roll, pitch, heading }: { roll: number; pitch: number; heading: number }) {
  const size = 220;
  const r = size / 2;
  const pxPerDeg = 2.4;
  const ladder = [-30, -20, -10, 10, 20, 30];
  const ticks = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60];

  return (
    <figure className="horizon" aria-label={`Horizon: roll ${roll.toFixed(0)} degrees, pitch ${pitch.toFixed(0)} degrees, heading ${heading.toFixed(0)}`}>
      <svg viewBox={`0 0 ${size} ${size + 34}`} width="100%">
        <defs>
          <clipPath id="ak-horizon-clip">
            <circle cx={r} cy={r} r={r - 14} />
          </clipPath>
        </defs>
        <g clipPath="url(#ak-horizon-clip)">
          <g transform={`rotate(${-roll} ${r} ${r}) translate(0 ${pitch * pxPerDeg})`}>
            <rect x={-size} y={-size * 2 + r} width={size * 3} height={size * 2} fill="var(--sky)" />
            <rect x={-size} y={r} width={size * 3} height={size * 2} fill="var(--ground)" />
            <line x1={-size} x2={size * 2} y1={r} y2={r} stroke="var(--ink)" strokeWidth="1.5" />
            {ladder.map((d) => (
              <g key={d}>
                <line
                  x1={r - (Math.abs(d) % 20 === 0 ? 30 : 18)}
                  x2={r + (Math.abs(d) % 20 === 0 ? 30 : 18)}
                  y1={r - d * pxPerDeg}
                  y2={r - d * pxPerDeg}
                  stroke="var(--ink)"
                  strokeWidth="1"
                  opacity="0.8"
                />
                <text x={r + 36} y={r - d * pxPerDeg + 4} className="horizon-num">
                  {Math.abs(d)}
                </text>
              </g>
            ))}
          </g>
        </g>
        <circle cx={r} cy={r} r={r - 14} fill="none" stroke="var(--rule-strong)" strokeWidth="1.5" />
        {ticks.map((d) => {
          const a = (d - 90) * DEG;
          const inner = d % 30 === 0 ? r - 26 : r - 21;
          return (
            <line
              key={d}
              x1={r + Math.cos(a) * inner}
              y1={r + Math.sin(a) * inner}
              x2={r + Math.cos(a) * (r - 14)}
              y2={r + Math.sin(a) * (r - 14)}
              stroke="var(--ink-soft)"
              strokeWidth={d === 0 ? 2.5 : 1.2}
            />
          );
        })}
        <g transform={`rotate(${-roll} ${r} ${r})`}>
          <path d={`M ${r} ${30} l -6 -10 h 12 z`} fill="var(--target)" />
        </g>
        {/* the fixed aircraft symbol */}
        <path d={`M ${r - 52} ${r} h 32 l 8 8 l 8 -8 h 32`} fill="none" stroke="var(--select)" strokeWidth="3.5" strokeLinejoin="round" />
        <circle cx={r} cy={r} r="2.5" fill="var(--select)" />

        <HeadingStrip y={size + 6} width={size} heading={heading} />
      </svg>
    </figure>
  );
}

function HeadingStrip({ y, width, heading }: { y: number; width: number; heading: number }) {
  const span = 90;
  const px = width / span;
  const marks: number[] = [];
  const first = Math.ceil((heading - span / 2) / 10) * 10;
  for (let d = first; d <= heading + span / 2; d += 10) marks.push(d);
  const label = (d: number) => {
    const w = wrap360(d);
    return ({ 0: 'N', 90: 'E', 180: 'S', 270: 'W' } as Record<number, string>)[w] ?? String(w / 10);
  };
  return (
    <g>
      <rect x="0" y={y} width={width} height="26" rx="3" fill="var(--sunken)" stroke="var(--rule)" />
      {marks.map((d) => {
        const x = width / 2 + (d - heading) * px;
        return (
          <g key={d}>
            <line x1={x} x2={x} y1={y} y2={y + (d % 30 === 0 ? 9 : 5)} stroke="var(--ink-soft)" />
            {d % 30 === 0 && (
              <text x={x} y={y + 21} textAnchor="middle" className="horizon-num">
                {label(d)}
              </text>
            )}
          </g>
        );
      })}
      <path d={`M ${width / 2} ${y} l -5 -6 h 10 z`} fill="var(--select)" />
    </g>
  );
}

/** Roll, pitch and heading over the last ten seconds. */
function TraceChart({ samples }: { samples: readonly Sample[] }) {
  const canvas = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const element = canvas.current;
    if (NO_CANVAS) return;
    const context = element?.getContext('2d');
    if (!element || !context) return;
    const ratio = Math.min(window.devicePixelRatio, 2);
    const w = element.clientWidth;
    const h = element.clientHeight;
    element.width = w * ratio;
    element.height = h * ratio;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, w, h);

    const css = getComputedStyle(element);
    const color = (name: string) => css.getPropertyValue(name).trim() || '#888';
    context.strokeStyle = color('--rule');
    context.lineWidth = 1;
    for (const d of [-90, -45, 0, 45, 90]) {
      const y = h / 2 - (d / 180) * h;
      context.beginPath();
      context.moveTo(0, y);
      context.lineTo(w, y);
      context.stroke();
    }
    if (samples.length < 2) return;
    const end = samples[samples.length - 1]!.t;
    const series: Array<[keyof Sample, string]> = [
      ['roll', color('--select')],
      ['pitch', color('--target')],
      ['yaw', color('--caution')],
    ];
    for (const [key, stroke] of series) {
      context.strokeStyle = stroke;
      context.lineWidth = 1.6;
      context.beginPath();
      samples.forEach((s, i) => {
        const x = w - ((end - s.t) / TRACE_SECONDS) * w;
        const y = h / 2 - ((s[key] as number) / 180) * h;
        if (i === 0) context.moveTo(x, y);
        else context.lineTo(x, y);
      });
      context.stroke();
    }
  });

  return (
    <figure className="att-trace">
      <figcaption>
        <span className="key select">roll</span>
        <span className="key target">pitch</span>
        <span className="key caution">heading</span>
        <span className="muted">last {TRACE_SECONDS} s, ±90°</span>
      </figcaption>
      <canvas ref={canvas} aria-label="Roll, pitch and heading over the last ten seconds" />
    </figure>
  );
}

function wrap360(deg: number): number {
  return ((deg % 360) + 360) % 360;
}

function wrap180(deg: number): number {
  const w = wrap360(deg);
  return w > 180 ? w - 360 : w;
}
