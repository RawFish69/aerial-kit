import { Button, Callout, Checkbox, FileInput, HTMLSelect, HTMLTable, ProgressBar } from '@blueprintjs/core';
import { useMemo, useState } from 'react';
import {
  FLASH_TARGETS,
  flash,
  planFlash,
  type FlashProgress,
  type FlashTargetId,
  type ImageFile,
  type Loader,
} from '../firmware/flash';
import { Panel } from './panels';

const hexOffset = (n: number) => `0x${n.toString(16).padStart(5, '0')}`;

/** A chosen file's bytes (FileReader where `Blob.arrayBuffer` is missing). */
function readBytes(f: File): Promise<Uint8Array> {
  if (typeof f.arrayBuffer === 'function') return f.arrayBuffer().then((b) => new Uint8Array(b));
  return new Promise((resolve, reject) => {
    const r = new FileReader();
    r.onload = () => resolve(new Uint8Array(r.result as ArrayBuffer));
    r.onerror = () => reject(r.error ?? new Error(`could not read ${f.name}`));
    r.readAsArrayBuffer(f);
  });
}

/**
 * Choose what to flash, check it, flash it. The images are the build's own
 * files (`.pio/build/<env>/`); where each goes comes from its partition table.
 * `makeLoader` is the ROM loader (esptool-js over Web Serial by default).
 */
export function FlashApp({ serialAvailable, makeLoader }: {
  serialAvailable: boolean;
  makeLoader?: () => Promise<Loader>;
}) {
  const [targetId, setTargetId] = useState<FlashTargetId>('radio-diagnostic');
  const [files, setFiles] = useState<ImageFile[]>([]);
  const [acknowledged, setAcknowledged] = useState(false);
  const [progress, setProgress] = useState<FlashProgress | null>(null);
  const [busy, setBusy] = useState(false);
  const [result, setResult] = useState<{ ok: boolean; text: string } | null>(null);
  const target = FLASH_TARGETS.find((t) => t.id === targetId)!;
  const plan = useMemo(() => (files.length > 0 ? planFlash(files) : null), [files]);
  const ready = plan !== null && plan.problems.length === 0 && acknowledged && !busy;

  const pick = async (list: FileList | null) => {
    setResult(null);
    if (list === null) return;
    setFiles(await Promise.all([...list].map(async (f) => ({ name: f.name, bytes: await readBytes(f) }))));
  };

  const run = async () => {
    if (plan === null) return;
    setBusy(true);
    setResult(null);
    try {
      const loader = await (makeLoader ?? (async () => (await import('../firmware/esptool')).esptoolLoader()))();
      await flash(target, plan, loader, setProgress);
      setResult({ ok: true, text: `${target.label} written and the board reset.` });
    } catch (e) {
      setResult({ ok: false, text: e instanceof Error ? e.message : String(e) });
    } finally {
      setBusy(false);
      setProgress(null);
    }
  };

  return (
    <main className="page flash-page">
      <Panel title="Flash firmware" note="ESP32-S3 boards, over Web Serial">
        <p>
          Choose what this board should run, then give the files the build wrote
          (<code>.pio/build/&lt;env&gt;/</code>: bootloader.bin, partitions.bin, boot_app0.bin, firmware.bin).
          Each file is checked and placed where its partition table says before anything is written.
        </p>
        <div className="flash-row">
          <label htmlFor="flash-target">Firmware</label>
          <HTMLSelect id="flash-target" value={targetId} disabled={busy}
            onChange={(e) => { setTargetId(e.currentTarget.value as FlashTargetId); setAcknowledged(false); }}>
            {FLASH_TARGETS.map((t) => <option key={t.id} value={t.id}>{t.label}</option>)}
          </HTMLSelect>
        </div>
        <p className="bp5-text-muted">
          {target.board} · {target.chip} · build <code>{target.buildEnv}</code>
        </p>
        <FileInput inputProps={{ multiple: true, accept: '.bin', 'aria-label': 'Image files' }} disabled={busy}
          text={files.length > 0 ? files.map((f) => f.name).join(', ') : 'Choose the image files…'}
          onInputChange={(e) => void pick((e.target as HTMLInputElement).files)} />
      </Panel>

      {plan !== null && (
        <Panel title="What will be written" note={plan.problems.length > 0 ? 'refused' : 'checked'}>
          {plan.problems.length > 0 && (
            <Callout intent="danger" title="Not flashing">
              <ul>{plan.problems.map((p) => <li key={p}>{p}</li>)}</ul>
            </Callout>
          )}
          <HTMLTable compact striped aria-label="Flash plan">
            <thead><tr><th>file</th><th>offset</th><th>size</th></tr></thead>
            <tbody>
              {plan.segments.map((s) => (
                <tr key={s.role}><td>{s.name}</td><td><code>{hexOffset(s.offset)}</code></td><td>{s.bytes.length.toLocaleString()} bytes</td></tr>
              ))}
            </tbody>
          </HTMLTable>
          <Callout intent="warning">{target.note}</Callout>
          <Checkbox checked={acknowledged} disabled={busy} label="I have read this and the board is ready"
            onChange={(e) => setAcknowledged(e.currentTarget.checked)} />
          {!serialAvailable && makeLoader === undefined && (
            <Callout intent="danger">This browser has no Web Serial. Chrome, Edge and Opera on a desktop have it.</Callout>
          )}
          <Button intent="primary" icon="flash" disabled={!ready || (!serialAvailable && makeLoader === undefined)}
            loading={busy} onClick={() => void run()}>
            Flash {target.label}
          </Button>
          {progress !== null && (
            <div className="flash-progress">
              <span>{progress.name}</span>
              <ProgressBar value={progress.fraction} />
            </div>
          )}
          {result !== null && <Callout intent={result.ok ? 'success' : 'danger'}>{result.text}</Callout>}
          <p className="bp5-text-muted">
            The board must be in its ROM bootloader: hold BOOT while plugging it in. Checked against the
            repository's build outputs and a simulated loader; not yet run against a board from this page.
          </p>
        </Panel>
      )}
    </main>
  );
}
