import { Button, type IconName } from '@blueprintjs/core';
import { Notice, Panel } from './panels';
import { useEffect, useState, type ReactNode } from 'react';
import { tabState, tabsFor, type Tab, type WorkspaceProps } from './tabs';

/**
 * The sidebar.
 *
 * Only the sections this board can back are listed. The ones it cannot are not
 * hidden from the person, only from the rail: the Identity section lists each
 * with the reason, which names the opcode — "does not answer `preflight`" is
 * something to search the firmware for; a greyed-out button is not.
 */
export function TabRail(props: WorkspaceProps & { readonly initialTab?: string; readonly footer?: ReactNode }) {
  const tabs = tabsFor('aerialkit');
  const [selected, setSelected] = useState<string>(props.initialTab ?? tabs[0]!.id);

  useEffect(() => {
    if (props.reviewEdits) setSelected('parameters');
  }, [props.reviewEdits]);

  const chosen = tabs.find((tab) => tab.id === selected) ?? tabs[0]!;
  const state = tabState(chosen, props.snapshot);
  const openTabs = tabs.filter((tab) => tabState(tab, props.snapshot).open);
  const closedTabs = tabs.filter((tab) => !tabState(tab, props.snapshot).open);

  return (
    <div className="workspace">
      <nav className="rail" aria-label="Workspace sections">
        <div className="rail-box">
        <div className="rail-title" aria-hidden="true">Flight controller</div>
        {openTabs.map((tab) => (
          <Button
            key={tab.id}
            type="button"
            minimal
            fill
            alignText="left"
            className="tab"
            icon={TAB_ICONS[tab.id] ?? 'square'}
            active={tab.id === chosen.id}
            aria-current={tab.id === chosen.id ? 'page' : undefined}
            onClick={() => setSelected(tab.id)}
          >
            {tab.label}
          </Button>
        ))}
        </div>
      </nav>

      <main className="tabbody">
        <header className="section-heading"><h1>{chosen.label}</h1></header>
        {/*
          A tab that was open can close when the *board* changes under it. Handled
          rather than assumed away, because the alternative is a blank body.
        */}
        {state.open ? (
          <>{chosen.render(props)}</>
        ) : (
          <Notice intent="none">
            <strong>{chosen.label} is not available.</strong> {state.why}
          </Notice>
        )}
        {chosen.id === 'identity' && closedTabs.length > 0 && (
          <Panel title="Not yet available">
            <dl className="kv unavailable">
              {closedTabs.map((tab) => (
                <div key={tab.id}>
                  <dt>{tab.label}</dt>
                  <dd>{tabState(tab, props.snapshot).why}</dd>
                </div>
              ))}
            </dl>
          </Panel>
        )}
        {chosen.id === 'identity' && props.footer !== undefined && <div className="tabfooter">{props.footer}</div>}
      </main>
    </div>
  );
}

/** Blueprint icons for the sections; purely decorative. */
const TAB_ICONS: Record<string, IconName> = {
  attitude: 'airplane',
  live: 'pulse',
  parameters: 'settings',
  identity: 'id-number',
  diagnostics: 'diagnosis',
  receiver: 'satellite',
  sensors: 'compass',
  motors: 'cog',
  blackbox: 'database',
  backup: 'archive',
  preflight: 'endorsed',
  mission: 'route',
  firmware: 'build',
};

/** The tabs in the order they are listed, for tests and for the capability
 *  matrix `docs/BUILD-AND-DEPLOY.md` carries. */
export function tabLabels(family: 'aerialkit' | 'msp' | 'mavlink' = 'aerialkit'): readonly string[] {
  return tabsFor(family).map((tab) => tab.label);
}

export type { Tab };
