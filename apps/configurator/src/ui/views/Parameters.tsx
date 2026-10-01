import { Button, InputGroup, SegmentedControl } from '@blueprintjs/core';
import { Notice } from '../panels';
import { useEffect, useMemo, useRef, useState } from 'react';
import { Feature, reasonFor } from '../../protocol/features';
import type { SessionSnapshot } from '../../session/types';
import { ParametersPanel, type ParameterActions } from '../panels';

type Filter = 'all' | 'changed' | 'staged';

/**
 * The parameter table with the tools around it: find a row by name or by what
 * it does, narrow to what differs from the build defaults or what is staged,
 * and keep a copy of the board's values in a file.
 *
 * Restoring from a file only **stages** values. Nothing reaches the board until
 * a person presses "Send all staged", and the write gate and range checks apply
 * to a restored value exactly as to a typed one.
 */
export function ParametersView({
  snapshot,
  actions,
  busy,
  reviewEdits,
  onReviewEditsConsumed,
}: {
  snapshot: SessionSnapshot;
  actions: ParameterActions;
  busy: boolean;
  reviewEdits?: number;
  onReviewEditsConsumed?: () => void;
}) {
  const [query, setQuery] = useState('');
  const [filter, setFilter] = useState<Filter>('all');
  const [group, setGroup] = useState('all');
  useEffect(() => {
    if (reviewEdits) {
      setFilter('staged'); setQuery(''); setGroup('all');
      onReviewEditsConsumed?.();
    }
  }, [reviewEdits]);
  const [restoreNote, setRestoreNote] = useState<string | null>(null);
  const file = useRef<HTMLInputElement>(null);
  const rows = snapshot.parameters;

  const counts = useMemo(
    () => ({
      changed: rows.filter(differsFromDefault).length,
      staged: rows.filter((row) => row.edited !== null).length,
    }),
    [rows],
  );

  const groups = useMemo(() => Array.from(new Set(rows.map((row) => row.meta?.groupName ?? 'Undescribed'))).sort(), [rows]);
  const visible = useMemo(() => {
    const q = query.trim().toLowerCase();
    return rows.filter((row) => {
      if (group !== 'all' && (row.meta?.groupName ?? 'Undescribed') !== group) return false;
      if (filter === 'changed' && !differsFromDefault(row)) return false;
      if (filter === 'staged' && row.edited === null) return false;
      if (q === '') return true;
      return (
        row.name.toLowerCase().includes(q) ||
        (row.meta?.help ?? '').toLowerCase().includes(q) ||
        (row.meta?.groupName ?? '').toLowerCase().includes(q)
      );
    });
  }, [rows, query, filter, group]);

  const backup = () => {
    const values: Record<string, string> = {};
    for (const row of rows) if (row.boardValue !== null && !row.meta?.secret) values[row.name] = row.boardValue;
    const document = {
      format: 'aerialkit-parameters/1',
      product: snapshot.identity?.product ?? null,
      configHash: snapshot.identity?.configHash ?? null,
      savedAt: new Date().toISOString(),
      parameters: values,
    };
    const blob = new Blob([JSON.stringify(document, null, 2)], { type: 'application/json' });
    const link = window.document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = `${document.product ?? 'aerialkit'}-parameters-${document.savedAt.slice(0, 19).replace(/[:T]/g, '-')}.json`;
    link.click();
    URL.revokeObjectURL(link.href);
  };

  const restore = async (picked: File) => {
    try {
      const parsed = JSON.parse(await picked.text()) as { parameters?: Record<string, unknown> };
      const values = parsed.parameters;
      if (values === undefined || typeof values !== 'object' || values === null) {
        setRestoreNote('That file has no "parameters" object, so nothing was staged.');
        return;
      }
      let staged = 0;
      const unknown: string[] = [];
      for (const [name, value] of Object.entries(values)) {
        const row = rows.find((candidate) => candidate.name === name);
        if (row === undefined) {
          unknown.push(name);
          continue;
        }
        const text = String(value);
        if (text !== row.boardValue) {
          actions.edit(row.index, text);
          staged++;
        }
      }
      setFilter(staged > 0 ? 'staged' : 'all');
      setRestoreNote(
        `${staged} value${staged === 1 ? '' : 's'} staged from ${picked.name}; nothing is sent until you press “Send all staged”.` +
          (unknown.length > 0 ? ` ${unknown.length} name${unknown.length === 1 ? '' : 's'} this board does not have: ${unknown.slice(0, 6).join(', ')}${unknown.length > 6 ? '…' : ''}.` : ''),
      );
    } catch (error) {
      setRestoreNote(`That file is not a parameter backup (${String(error)}). Nothing was staged.`);
    }
  };

  return (
    <div className="params-view">
      <div className="params-tools">
        <InputGroup
          type="search"
          className="search"
          leftIcon="search"
          placeholder={`Search ${rows.length} parameters by name or description`}
          value={query}
          onChange={(event) => setQuery(event.target.value)}
          aria-label="Search parameters"
        />
        <label className="group-picker">
          <span className="small muted">Group</span>
          <select className="bp5-input" aria-label="Parameter group" value={group} onChange={(event) => setGroup(event.target.value)}>
            <option value="all">All groups</option>
            {groups.map((name) => <option key={name} value={name}>{name}</option>)}
          </select>
        </label>
        <SegmentedControl
          aria-label="Show"
          value={filter}
          onValueChange={(value) => setFilter(value as Filter)}
          options={[
            { label: 'All', value: 'all' },
            { label: `Not default (${counts.changed})`, value: 'changed' },
            { label: `Staged (${counts.staged})`, value: 'staged' },
          ]}
        />
        <span className="spacer" />
        <Button type="button" icon="download" onClick={backup} disabled={rows.length === 0}>
          Back up to file
        </Button>
        <Button type="button" icon="upload" onClick={() => file.current?.click()} disabled={rows.length === 0 || busy}>
          Restore from file
        </Button>
        <input
          ref={file}
          type="file"
          accept="application/json,.json"
          hidden
          onChange={(event) => {
            const picked = event.target.files?.[0];
            if (picked) void restore(picked);
            event.target.value = '';
          }}
        />
      </div>

      <div className="params-summary">
        <span className="small muted">Showing {visible.length} of {rows.length} parameters</span>
        {counts.staged > 0 && (
          <Button minimal small icon="undo" disabled={busy} onClick={() => {
            for (const row of rows) if (row.edited !== null) actions.revert(row.index);
          }}>Discard all staged</Button>
        )}
      </div>

      {restoreNote !== null && (
        <Notice intent="none" role="status">
          {restoreNote}
        </Notice>
      )}

      {visible.length === 0 && rows.length > 0 ? (
        <p className="empty">
          No parameter matches{query !== '' ? ` “${query}”` : ''}
          {group !== 'all' ? ` in ${group}` : ''}
          {filter !== 'all' ? ` among the ${filter === 'changed' ? 'values that differ from the defaults' : 'staged values'}` : ''}.{' '}
          <Button type="button" minimal small intent="primary" onClick={() => { setQuery(''); setFilter('all'); setGroup('all'); }}>
            Show all
          </Button>
        </p>
      ) : (
        <ParametersPanel
          rows={visible}
          allRows={rows}
          permission={snapshot.permission}
          unsaved={snapshot.unsaved}
          busy={busy}
          resetReason={reasonFor(snapshot.identity?.features ?? null, Feature.PARAM_DEFAULT, 'param default')}
          actions={actions}
        />
      )}
    </div>
  );
}

function differsFromDefault(row: SessionSnapshot['parameters'][number]): boolean {
  if (row.meta === null || row.boardValue === null) return false;
  const a = Number(row.boardValue);
  const b = Number(row.meta.default);
  if (Number.isFinite(a) && Number.isFinite(b)) return a !== b;
  return row.boardValue !== row.meta.default;
}
