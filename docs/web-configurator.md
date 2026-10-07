# Aerial Kit web configurator

[App source](../apps/configurator) · [Full build/deployment guide](../apps/configurator/docs/BUILD-AND-DEPLOY.md) · [Firmware](flight-controller-firmware.md)

A standalone React/TypeScript app for board identity, attitude, live diagnostics and parameter configuration. It detects supported firmware and exposes capabilities reported by the connected board. The built app is static files; the optional local bridge is a separate helper.

## Open the configurator

[Launch Aerial Kit Configurator](https://rawfish69.github.io/aerial-kit/) · [Download source](https://github.com/RawFish69/aerial-kit/archive/refs/heads/main.zip)

No installation is needed for the hosted app. Choose **Demo board**, then **Explore demo** to try it without hardware. Readings are simulated and edits reset on disconnect.

For a physical board:

1. Build and flash the correct [firmware profile](flight-controller-firmware.md). The public profiles are Feather F405 and ESP32DEV; a profile does not establish that external sensors or wiring work.
2. Open the app in a desktop browser that supports Web Serial, such as Chrome or Edge, and plug in a USB data cable.
3. Close other applications using the serial port. Select **USB serial**, click **Connect via USB**, and choose the board in the browser's permission dialog.
4. Confirm the board identity. Inspect attitude and sensor readings before editing parameters.

The page requests a device only after you click Connect. It does not flash firmware automatically. A board in ROM DFU mode must first be flashed and booted into the firmware's USB console. Safari and Firefox users can explore the demo or use a supported browser for USB.

**Local bridge:** GitHub Pages serves HTTPS. Browsers can block an insecure `ws://` bridge from that page. Run the app locally with the loopback helper, or configure a trusted secure `wss://` endpoint using the [bridge guide](../apps/configurator/docs/BUILD-AND-DEPLOY.md). GitHub Pages hosts static assets and cannot run the bridge or firmware builds.

## Run locally

Install Node.js 20+ and npm, then from the repository root:

```sh
cd apps/configurator
npm ci
npm run dev
```

Open the URL printed by Vite. Choose **USB serial** in a browser with Web Serial support, **Demo board** to explore without hardware, or **Local bridge** to connect through the helper. USB permission is granted through the browser's device chooser. See the [transport capability matrix](../apps/configurator/docs/BUILD-AND-DEPLOY.md) for supported browsers and firmware.

## USB recovery on Linux

If the board is silent, confirm that the selected port belongs to the board, close other serial clients and retry after a few seconds. ModemManager may probe a newly attached serial port and consume replies while the configurator is connecting. Check that your user has permission to open the serial device.

If ModemManager repeatedly claims the AerialKit USB console, a rule scoped to its USB ID can exclude it from modem probing:

```sh
sudo tee /etc/udev/rules.d/99-aerialkit.rules >/dev/null <<'EOF'
# AerialKit USB console (0483:5740) is not a modem.
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules
```

Reconnect the board after reloading the rule, then use the browser's USB device chooser again. This changes host device handling; it does not flash the board or grant browser permission automatically. The firmware's USB console uses `0483:5740`; check the actual device ID before applying this rule to a different board.

## Configuration workflow

1. Connect and check the board identity, armed state and limitations.
2. Open Parameters; search by name/help or filter by the board's own group.
3. Edit values. These are staged in the app and have not reached the board.
4. Use **Review edits** and send staged values, or **Discard all staged**.
5. **Save to flash** persists the board's values. It is disabled while edits are still staged, so sending and saving remain distinct.

Writes are gated by armed state and freshness. An echoed value does not prove that the running controller applied it; the UI states what the protocol can establish. Backup/restore excludes secret values, and restoring stages changes for review.

## Build and check

```sh
npm test
npm run build
AK_BASE=/aerial-kit/ npm run build
npm run verify:deploy
```

`dist/` is the static site. `AK_BASE` sets an optional hosting subpath and also covers logo/favicon URLs. Live SITL tests require the simulator specified by `AERIALKIT_SITL`; without it those cases are skipped. The drift checks compare source citations, demo metadata and MAVLink dialect generation against `../../firmware/`. The public tree does not include the large C host simulator; Local bridge and live SITL workflows require an externally built simulator or a real serial bridge.

## GitHub Pages deployment

[The Pages workflow](../.github/workflows/configurator-pages.yml) tests and builds the app on relevant pushes to `main`; pull requests build without deploying. It publishes `apps/configurator/dist` with `AK_BASE=/aerial-kit/`, including the logo and favicon beneath that path.

In repository **Settings → Pages**, use **GitHub Actions** as the source. Run **Configurator and GitHub Pages** manually when enabling it for the first time. The deployment job needs `pages: write`, `id-token: write`, and the `github-pages` environment. A successful deployment exposes the app at the launch link above. See [GitHub's custom workflow guide](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages) for repository settings.

## Connect Python to firmware

The [read-only Python example](../examples/firmware/) reads board identity and attitude over USB or a firmware TCP endpoint, then converts the firmware's NED/FRD attitude into the Python stack's ENU/FLU quaternion convention. It uses the same protocol client as the firmware tools. It is a telemetry example, not a flight-control loop.

## Brand and licence

The app uses the selected original **01 Rotor A** symbol. [Brand guide](branding.md) includes the SVG assets and usage details. The configurator is GPL-3.0-or-later; see [its licence](../apps/configurator/LICENSE) and component notices.
