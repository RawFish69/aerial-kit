/**
 * The three looks: Modern (Blueprint's dark theme), Classic (Windows XP style)
 * and OG (Windows 95/98 style). OG is drawn as Classic's window with its own
 * surfaces, so the page carries both names: `data-theme="classic og"`. The choice is a per-browser convenience, so it lives in
 * localStorage and the page works the same when storage is unavailable.
 */

export type Theme = 'modern' | 'classic' | 'og';

const KEY = 'aerialkit.theme';

export const THEMES: ReadonlyArray<{ readonly id: Theme; readonly label: string }> = [
  { id: 'modern', label: 'Modern' },
  { id: 'classic', label: 'Classic' },
  { id: 'og', label: 'OG' },
];

export function loadTheme(): Theme {
  try {
    const stored = localStorage.getItem(KEY);
    return stored === 'classic' || stored === 'og' ? stored : 'modern';
  } catch {
    return 'modern';
  }
}

export function applyTheme(theme: Theme): void {
  document.documentElement.dataset.theme = theme === 'og' ? 'classic og' : theme;
  document.documentElement.style.colorScheme = theme === 'modern' ? 'dark' : 'light';
  // Blueprint's dark class on body, so portals (popovers, toasts) follow it.
  document.body.classList.toggle('bp5-dark', theme === 'modern');
  try {
    localStorage.setItem(KEY, theme);
  } catch {
    // Private windows and blocked storage: the theme still applies for this visit.
  }
}
