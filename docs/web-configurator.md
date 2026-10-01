# AerialKit web configurator

[App source](../apps/configurator) · [Full build/deployment guide](../apps/configurator/docs/BUILD-AND-DEPLOY.md) · [Firmware](flight-controller-firmware.md)

A standalone React/TypeScript app for board identity, attitude, live diagnostics and parameter configuration. It detects supported firmware and exposes capabilities reported by the connected board. The built app is static files; the optional local bridge is a separate helper.

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
AK_BASE=/aerialkit/configurator/ npm run build
npm run verify:deploy
```

`dist/` is the static site. `AK_BASE` sets an optional hosting subpath and also covers logo/favicon URLs. Live SITL tests require the simulator specified by `AERIALKIT_SITL`; without it those cases are skipped. The drift checks compare source citations, demo metadata and MAVLink dialect generation against `../../firmware/`. The public tree does not include the large C host simulator; Local bridge and live SITL workflows require an externally built simulator or a real serial bridge.

Publishing source in this repo does not deploy a hosted configurator. See the deployment guide for serving it over HTTPS and running the loopback bridge.

## Brand and licence

The app uses the selected original **01 Rotor A** symbol. [Brand guide](branding.md) includes the SVG assets and usage details. The configurator is GPL-3.0-or-later; see [its licence](../apps/configurator/LICENSE) and component notices.
