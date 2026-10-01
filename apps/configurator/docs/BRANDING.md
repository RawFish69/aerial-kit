# Configurator branding

The app uses the original 01 Rotor A [SVG](../public/brand/aerialkit-rotor-a.svg), blue `#4c90f0`, at 32 px in the header and as its favicon. The displayed name is **Aerial Kit**, with a space.

The header asset uses `import.meta.env.BASE_URL`; Vite rewrites the favicon URL for the build base. This keeps both working when the app is served at a subpath.

See the repository [brand guide](../../../docs/branding.md).
