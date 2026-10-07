import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { App } from './ui/App';
import '@blueprintjs/core/lib/css/blueprint.css';
import './styles.css';
import { applyTheme, loadTheme } from './ui/theme';

applyTheme(loadTheme());

const host = document.getElementById('root');
if (host === null) {
  throw new Error('the page has no #root element to mount into');
}

// StrictMode double-invokes effects in development, which is exactly what this
// app wants: the session opens a port and starts a timer, and a component that
// survives being mounted twice is a component that will survive a re-render.
createRoot(host).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
