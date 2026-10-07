import { Icon, Menu, MenuDivider, MenuItem, Popover } from '@blueprintjs/core';
import { useEffect, useRef, useState, type PointerEvent as ReactPointerEvent, type ReactNode } from 'react';
import { THEMES, type Theme } from './theme';

/**
 * Classic's desktop: the window frame with working caption buttons, a menu
 * bar, a taskbar with a Start menu and a tray, and the tray's balloon. Every
 * control here does something real — minimise keeps the board connected,
 * close disconnects — so none of it is decoration that ignores a click.
 */

export type WindowState = 'maximized' | 'restored' | 'minimized' | 'closed';

export interface Links {
  readonly guide: string;
  readonly firmware: string;
  readonly source: string;
  readonly issues: string;
  readonly contact: string;
}

const LOGO = `${import.meta.env.BASE_URL}brand/aerialkit-rotor-a.svg`;

function Logo({ size }: { size: number }) {
  return <img className="xp-logo" src={LOGO} width={size} height={size} alt="" aria-hidden="true" />;
}

/**
 * Moving a window by its title bar. Returns the offset to apply as a
 * transform and the handlers for the title bar. The window is kept where its
 * title bar can still be grabbed: at least 120 px of it stays on screen, and
 * it never goes above the top or under the taskbar.
 */
export function useWindowDrag(enabled = true) {
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const drag = useRef<{ x: number; y: number; ox: number; oy: number; rect: DOMRect } | null>(null);

  const handlers = {
    onPointerDown: (event: ReactPointerEvent<HTMLElement>) => {
      if (!enabled || event.button !== 0) return;
      if ((event.target as HTMLElement).closest('button') !== null) return;
      const windowElement = event.currentTarget.closest('.xp-window, .xp-notepad');
      if (windowElement === null) return;
      drag.current = {
        x: event.clientX,
        y: event.clientY,
        ox: offset.x,
        oy: offset.y,
        rect: windowElement.getBoundingClientRect(),
      };
      event.currentTarget.setPointerCapture(event.pointerId);
    },
    onPointerMove: (event: ReactPointerEvent<HTMLElement>) => {
      const start = drag.current;
      if (start === null) return;
      const keep = 120;
      const taskbar = 34;
      let dx = event.clientX - start.x;
      let dy = event.clientY - start.y;
      dx = Math.max(keep - start.rect.right, Math.min(window.innerWidth - keep - start.rect.left, dx));
      dy = Math.max(-start.rect.top, Math.min(window.innerHeight - taskbar - 32 - start.rect.top, dy));
      setOffset({ x: start.ox + dx, y: start.oy + dy });
    },
    onPointerUp: () => {
      drag.current = null;
    },
    onPointerCancel: () => {
      drag.current = null;
    },
  };

  return {
    offset,
    reset: () => setOffset({ x: 0, y: 0 }),
    style: enabled && (offset.x !== 0 || offset.y !== 0) ? { transform: `translate(${offset.x}px, ${offset.y}px)` } : undefined,
    handlers,
  };
}

export function TitleBar({
  title,
  win,
  setWin,
  onClose,
  theme,
  setTheme,
  file,
  links,
  onAbout,
  dragHandlers,
}: {
  title: string;
  win: WindowState;
  setWin: (win: WindowState) => void;
  onClose: () => void;
  theme: Theme;
  setTheme: (theme: Theme) => void;
  file: JSX.Element;
  links: Links;
  onAbout: () => void;
  /** Title-bar handlers from `useWindowDrag`, so a restored window can be moved. */
  dragHandlers?: ReturnType<typeof useWindowDrag>['handlers'];
}) {
  const maximized = win === 'maximized';
  const toggle = () => setWin(maximized ? 'restored' : 'maximized');
  const menu = (label: string, content: JSX.Element) => (
    <Popover content={content} placement="bottom-start" minimal popoverClassName="xp-menu-popover">
      <button type="button" className="xp-menu-title">{label}</button>
    </Popover>
  );
  return (
    <div className="xp-chrome">
      <div className={`xp-titlebar ${maximized ? '' : 'is-draggable'}`} onDoubleClick={toggle} {...dragHandlers}>
        <Logo size={18} />
        <span className="xp-title">{title}</span>
        <div className="xp-caption-buttons">
          <button type="button" className="xp-cap xp-cap-min" aria-label="Minimize" title="Minimize" onClick={() => setWin('minimized')}>
            <svg viewBox="0 0 11 11" aria-hidden="true"><rect x="1" y="8" width="6" height="2" /></svg>
          </button>
          <button type="button" className="xp-cap xp-cap-max" aria-label={maximized ? 'Restore down' : 'Maximize'} title={maximized ? 'Restore Down' : 'Maximize'} onClick={toggle}>
            {maximized ? (
              <svg viewBox="0 0 11 11" aria-hidden="true">
                <path d="M3 1h7v6H8V3H3z" />
                <path d="M1 4h7v6H1z M2 6v3h5V6z" fillRule="evenodd" />
              </svg>
            ) : (
              <svg viewBox="0 0 11 11" aria-hidden="true"><path d="M1 1h9v9H1z M2 3v6h7V3z" fillRule="evenodd" /></svg>
            )}
          </button>
          <button type="button" className="xp-cap xp-cap-close" aria-label="Close" title="Close" onClick={onClose}>
            <svg viewBox="0 0 11 11" aria-hidden="true"><path d="M1.5 1.5l8 8M9.5 1.5l-8 8" strokeWidth="2" stroke="currentColor" /></svg>
          </button>
        </div>
      </div>
      <div className="xp-menubar" role="menubar" aria-label="Menu">
        {menu('File', file)}
        {menu(
          'View',
          <Menu>
            {THEMES.map((entry) => (
              <MenuItem key={entry.id} icon={entry.id === theme ? 'tick' : 'blank'} text={`${entry.label} theme`} onClick={() => setTheme(entry.id)} />
            ))}
            <MenuDivider />
            <MenuItem icon={maximized ? 'minimize' : 'maximize'} text={maximized ? 'Restore window' : 'Maximize window'} onClick={toggle} />
          </Menu>,
        )}
        {menu('Help', <HelpMenu links={links} onAbout={onAbout} />)}
      </div>
    </div>
  );
}

function HelpMenu({ links, onAbout }: { links: Links; onAbout: () => void }) {
  return (
    <Menu>
      <MenuItem icon="document" text="About Aerial Kit" onClick={onAbout} />
      <MenuDivider />
      <MenuItem icon="manual" text="Configurator guide" href={links.guide} target="_blank" />
      <MenuItem icon="code" text="Firmware source" href={links.firmware} target="_blank" />
      <MenuItem icon="git-repo" text="Aerial Kit on GitHub" href={links.source} target="_blank" />
      <MenuDivider />
      <MenuItem icon="issue" text="Report a problem" href={links.issues} target="_blank" />
      <MenuItem icon="envelope" text={`Developer email: ${links.contact}`} href={`mailto:${links.contact}`} />
    </Menu>
  );
}

/** The desktop behind a restored, minimised or closed window. */
export function Desktop({ win, onOpen, onAbout }: { win: WindowState; onOpen: () => void; onAbout: () => void }) {
  const [selected, setSelected] = useState<string | null>(null);
  const icon = (id: string, label: string, art: ReactNode, open: () => void) => (
    <button
      type="button"
      className={`xp-desktop-icon ${selected === id ? 'is-selected' : ''}`}
      onClick={(event) => {
        event.stopPropagation();
        setSelected(id);
      }}
      onDoubleClick={open}
      onKeyDown={(event) => {
        if (event.key === 'Enter') open();
      }}
      title="Double-click to open"
    >
      {art}
      <span>{label}</span>
    </button>
  );
  return (
    <div className="xp-desktop" aria-hidden={win === 'maximized'} onClick={() => setSelected(null)}>
      <div className="xp-desktop-icons">
        {icon('app', 'Aerial Kit Configurator', <Logo size={40} />, onOpen)}
        {icon('about', 'About Aerial Kit.txt', <TextFileIcon />, onAbout)}
      </div>
    </div>
  );
}

export function Taskbar({
  win,
  setWin,
  title,
  connected,
  linkLabel,
  theme,
  setTheme,
  onDemo,
  onDisconnect,
  links,
  balloon,
  onBalloonClose,
  onAbout,
}: {
  win: WindowState;
  setWin: (win: WindowState) => void;
  title: string;
  connected: boolean;
  linkLabel: string;
  theme: Theme;
  setTheme: (theme: Theme) => void;
  onDemo: () => void;
  onDisconnect: () => void;
  links: Links;
  balloon: { title: string; text: string } | null;
  onBalloonClose: () => void;
  onAbout: () => void;
}) {
  const [startOpen, setStartOpen] = useState(false);
  const [clock, setClock] = useState(() => new Date());
  const startRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const timer = setInterval(() => setClock(new Date()), 15_000);
    return () => clearInterval(timer);
  }, []);

  // Clicking anywhere outside the Start menu closes it, as it always did.
  useEffect(() => {
    if (!startOpen) return;
    const close = (event: MouseEvent) => {
      if (startRef.current !== null && !startRef.current.contains(event.target as Node)) setStartOpen(false);
    };
    const escape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') setStartOpen(false);
    };
    document.addEventListener('mousedown', close);
    document.addEventListener('keydown', escape);
    return () => {
      document.removeEventListener('mousedown', close);
      document.removeEventListener('keydown', escape);
    };
  }, [startOpen]);

  const visible = win === 'maximized' || win === 'restored';
  const nextTheme = THEMES[(THEMES.findIndex((entry) => entry.id === theme) + 1) % THEMES.length]!;
  const run = (action: () => void) => () => {
    setStartOpen(false);
    action();
  };
  const open = () => setWin(win === 'closed' || win === 'minimized' ? 'maximized' : win);

  return (
    <div className="xp-taskbar" ref={startRef}>
      <button type="button" className={`xp-start ${startOpen ? 'is-open' : ''}`} onClick={() => setStartOpen((value) => !value)} aria-expanded={startOpen} aria-haspopup="menu">
        <Logo size={20} />
        <span className="xp-start-label">start</span>
      </button>

      {startOpen && (
        <div className="xp-startmenu" role="menu" aria-label="Start menu">
          <div className="xp-startmenu-head">
            <Logo size={44} />
            <span>Aerial Kit</span>
          </div>
          <div className="xp-startmenu-body">
            <ul className="xp-startmenu-left">
              <StartItem icon="application" strong label="Aerial Kit Configurator" hint="Open the configurator" onClick={run(open)} />
              <StartItem icon="lab-test" strong label="Demo board" hint="A simulated board, no hardware" onClick={run(onDemo)} />
              <li className="xp-startmenu-sep" role="separator" />
              <StartItem icon="document" label="About Aerial Kit" onClick={run(onAbout)} />
              <StartItem icon="manual" label="Configurator guide" href={links.guide} />
            </ul>
            <ul className="xp-startmenu-right">
              <StartItem icon="code" label="Firmware" href={links.firmware} />
              <StartItem icon="git-repo" label="GitHub" href={links.source} />
              <li className="xp-startmenu-sep" role="separator" />
              <StartItem icon="issue" label="Report a problem" href={links.issues} />
              <StartItem icon="envelope" label="Contact" href={`mailto:${links.contact}`} />
            </ul>
          </div>
          <div className="xp-startmenu-foot">
            <button type="button" onClick={run(() => setTheme(nextTheme.id))}>
              <span className="xp-foot-icon xp-foot-theme"><Icon icon="contrast" size={14} /></span>
              {nextTheme.label} theme
            </button>
            <button type="button" onClick={run(onDisconnect)} disabled={!connected}>
              <span className="xp-foot-icon xp-foot-off"><Icon icon="power" size={14} /></span>
              Disconnect
            </button>
          </div>
        </div>
      )}

      <div className="xp-tasks">
        {win !== 'closed' && (
          <button
            type="button"
            className={`xp-task ${visible ? 'is-active' : ''}`}
            onClick={() => setWin(visible ? 'minimized' : 'maximized')}
            title={title}
          >
            <Logo size={16} />
            <span>{title}</span>
          </button>
        )}
      </div>

      <div className="xp-tray">
        {balloon !== null && (
          <div className="xp-balloon" role="status">
            <button type="button" className="xp-balloon-close" aria-label="Close notification" onClick={onBalloonClose}>×</button>
            <strong><Icon icon="info-sign" size={14} /> {balloon.title}</strong>
            <span>{balloon.text}</span>
          </div>
        )}
        <span className={`xp-tray-icon ${connected ? 'is-on' : ''}`} title={connected ? `Connected: ${linkLabel}` : 'No board connected'}>
          <Icon icon={connected ? 'link' : 'offline'} size={14} />
        </span>
        <span className="xp-clock">{clock.toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' })}</span>
      </div>
    </div>
  );
}

function StartItem({
  icon,
  label,
  hint,
  strong,
  href,
  onClick,
}: {
  icon: Parameters<typeof Icon>[0]['icon'];
  label: string;
  hint?: string;
  strong?: boolean;
  href?: string;
  onClick?: () => void;
}) {
  const body: ReactNode = (
    <>
      <span className="xp-start-icon"><Icon icon={icon} size={strong ? 22 : 16} /></span>
      <span className="xp-start-text">
        <span className={strong ? 'is-strong' : undefined}>{label}</span>
        {hint !== undefined && <small>{hint}</small>}
      </span>
    </>
  );
  return (
    <li role="none">
      {href !== undefined ? (
        <a role="menuitem" href={href} target={href.startsWith('mailto:') ? undefined : '_blank'} rel="noreferrer">{body}</a>
      ) : (
        <button role="menuitem" type="button" onClick={onClick}>{body}</button>
      )}
    </li>
  );
}

const ABOUT_TEXT = `About Aerial Kit
================

Aerial Kit is an open-source flight controller project: firmware for
small drones, a Python toolkit for simulation and control, and this
configurator.

The configurator runs entirely in your browser. Plug a board in over
USB, press Connect, and you can:

  - watch attitude, receiver, sensors and motor outputs live
  - read, edit and save the flight parameters
  - back up a configuration to a file and restore it

Nothing is written to the board until you send it, nothing is kept
after power-off until you save to flash, and writes are refused while
the board reports armed. Props off on the bench.

Boards: the STM32 and ESP32 families, with more coming soon.
No board? File > Open demo board.

Source, firmware and docs:
  https://github.com/RawFish69/aerial-kit

Found a problem?
  https://github.com/RawFish69/aerial-kit/issues

Developer email: dev@nori.fish

--
Made by RawFish69 · https://nori.fish
`;

/**
 * About Aerial Kit.txt, opened the way a text file on this desktop would be:
 * a small Notepad window that can be dragged by its title bar and closed.
 */
export function Notepad({ onClose }: { onClose: () => void }) {
  const move = useWindowDrag();

  useEffect(() => {
    const escape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') onClose();
    };
    document.addEventListener('keydown', escape);
    return () => document.removeEventListener('keydown', escape);
  }, [onClose]);

  return (
    <div
      className="xp-notepad"
      role="dialog"
      aria-label="About Aerial Kit.txt"
      style={move.style}
    >
      <div className="xp-titlebar is-draggable" {...move.handlers}>
        <span className="xp-notepad-icon" aria-hidden="true" />
        <span className="xp-title">About Aerial Kit.txt - Notepad</span>
        <div className="xp-caption-buttons">
          <button type="button" className="xp-cap xp-cap-close" aria-label="Close" title="Close" onClick={onClose}>
            <svg viewBox="0 0 11 11" aria-hidden="true"><path d="M1.5 1.5l8 8M9.5 1.5l-8 8" strokeWidth="2" stroke="currentColor" /></svg>
          </button>
        </div>
      </div>
      <div className="xp-menubar" aria-hidden="true">
        <span className="xp-menu-title is-static">File</span>
        <span className="xp-menu-title is-static">Edit</span>
        <span className="xp-menu-title is-static">Format</span>
        <span className="xp-menu-title is-static">View</span>
        <span className="xp-menu-title is-static">Help</span>
      </div>
      <textarea className="xp-notepad-text" readOnly value={ABOUT_TEXT} spellCheck={false} aria-label="About Aerial Kit" />
    </div>
  );
}

/** A plain text-file icon for the desktop. */
function TextFileIcon() {
  return (
    <svg className="xp-file-icon" width="40" height="40" viewBox="0 0 32 32" aria-hidden="true">
      <path d="M7 2h13l6 6v22H7z" fill="#fff" stroke="#5a5a5a" />
      <path d="M20 2v6h6" fill="#e6e6e6" stroke="#5a5a5a" />
      {[12, 15, 18, 21, 24].map((y) => (
        <path key={y} d={`M10 ${y}h${y === 24 ? 8 : 13}`} stroke="#4a6fa5" strokeWidth="1.4" />
      ))}
    </svg>
  );
}
