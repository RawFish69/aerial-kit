# AerialKit branding

![AerialKit original Rotor A on light and dark surfaces](../public/brand/aerialkit-selected.svg)

The selected logo is **original 01: Rotor A**, chosen on 2026-10-01. A swept-wing
letter A surrounds a three-blade propeller. The original geometry is retained;
the later refinement was declined. This identity is shared by the flight-controller
firmware and web configurator.

## Assets

| Asset | Use |
| --- | --- |
| [Blue mark](../public/brand/aerialkit-rotor-a.svg) | Configurator header and favicon |
| [White mark](../public/brand/aerialkit-rotor-a-white.svg) | Dark backgrounds and one-colour reproduction |
| [Dark mark](../public/brand/aerialkit-rotor-a-ink.svg) | Light backgrounds and one-colour reproduction |
| [Light-background wordmark](../public/brand/aerialkit-rotor-a-logo-light.svg) | Documents and light surfaces |
| [Dark-background wordmark](../public/brand/aerialkit-rotor-a-logo-dark.svg) | Dark surfaces |

These are editable SVGs. Marks use a 64 × 64 viewBox. Horizontal wordmarks use
the Segoe UI / Helvetica / Arial system sans-serif stack, with editable text.

## Use in the app

- Header: 32 × 32 px, followed by the existing AerialKit configurator title.
- Favicon: the same original Rotor A; 16, 24 and 32 px were visually reviewed.
- Accent: Blueprint blue `#4c90f0`. Monochrome: `#f6f7f9` or `#1c2127`.
- Keep proportions intact and leave at least 8 px between the header mark and text.
- Use solid fills. Do not add glow, shadows or gradients.

`App.tsx` builds the header asset URL from Vite's `BASE_URL`; Vite rewrites the
favicon URL in `index.html`. Both therefore follow `AK_BASE` for subpath builds.

Earlier concepts remain as separate exploration files. `aerialkit-mark*.svg`,
`aerialkit-wing-rotor*.svg`, and `aerialkit-rotor-a-v2*.svg` are not the selected
logo and are not referenced by the app.
