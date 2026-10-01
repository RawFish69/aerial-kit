import { Button, type IconName } from '@blueprintjs/core';
import { Notice } from './panels';
import { useEffect, useState, type ReactNode } from 'react';
import { tabState, tabsFor, type Tab, type WorkspaceProps } from './tabs';

/**
 * The sidebar, and the one thing about it worth stating.
 *
 * A section that cannot be opened is **shown**, disabled, with its reason as the
 * button's title *and* as a line under the body. Both, because a `title` is
 * invisible to a keyboard, to a touch screen and to anyone who does not think
 * to hover, and the reason is the most useful sentence on the page.
 *
 * The reason names an opcode. `reasonFor` is the one place that decides
 * between "this firmware does not answer that" and "this firmware cannot say",
 * and this view does not re-derive it.
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

  const button = (tab: Tab) => {
    const item = tabState(tab, props.snapshot);
    return (
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
        disabled={!item.open}
        title={item.why ?? undefined}
        onClick={() => setSelected(tab.id)}
      >
        {tab.label}
      </Button>
    );
  };

  return (
    <div className="workspace">
      <nav className="rail" aria-label="Workspace sections">
        {openTabs.map(button)}
        {closedTabs.length > 0 && (
          <>
            <div className="rail-heading bp5-text-muted" aria-hidden="true">Unavailable</div>
            {closedTabs.map(button)}
            <details className="unavailable-sections">
              <summary>Why unavailable?</summary>
              <dl>{closedTabs.map((tab) => (
                <div key={tab.id}>
                  <dt>{tab.label}</dt>
                  <dd>{tabState(tab, props.snapshot).why}</dd>
                </div>
              ))}</dl>
            </details>
          </>
        )}
      </nav>

      <main className="tabbody">
        <header className="section-heading"><h1>{chosen.label}</h1></header>
        {/*
          Unreachable through the rail — a disabled button cannot be clicked —
          but reachable when the *board* changes under a tab that is already
          open. Handled rather than assumed away, because the alternative is a
          blank body.
        */}
        {state.open ? (
          <>{chosen.render(props)}</>
        ) : (
          <Notice intent="none">
            <strong>{chosen.label} is not available.</strong> {state.why}
          </Notice>
        )}
        {props.footer !== undefined && <div className="tabfooter">{props.footer}</div>}
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
