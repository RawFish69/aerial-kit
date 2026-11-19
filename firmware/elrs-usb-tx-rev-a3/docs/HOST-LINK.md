# Rev A3 host link

This describes how a host talks to the Aerial Kit ELRS USB TX Rev A3 using
the pinned ExpressLRS 4.1.0 target.

## USB-only bench transport

The preferred bench connection is the ESP32-S3 native USB-CDC port. It works
for both roles and does not require a USB-UART adapter or Wi-Fi:

- TX: host sends CRSF `RC_CHANNELS_PACKED` to the board. The bridge feeds the
  normal ExpressLRS scheduler and returns telemetry, link statistics and device
  information over the same CDC stream.
- RX: the board outputs CRSF `RC_CHANNELS_PACKED` as the flight-controller
  endpoint. Host telemetry written to CDC is forwarded over RF through the
  normal telemetry path.
- Nominal logical rates remain **400000 baud for the TX host side** and
  **420000 baud for the RX FC side**. USB CDC ignores the selected baud rate;
  the nominal rate is documented for CRSF tooling and configuration.
- Native USB ROM download remains separate. Hold BOOT and pulse RESET if the
  application does not enter download mode automatically.

Two boards enumerate as two distinct `/dev/ttyACM*` devices.

Open native USB without asserting DTR or RTS. The Python `SerialTransport`
does this by default. If another tool resets the board while opening the port,
the ROM banner and bootloader `load:` text may precede CRSF. Resynchronize on
sync byte `0xC8`; do not parse that text as CRSF. Arduino core logging is
disabled in these target images so post-boot serial output remains CRSF.

## Optional UART bench transport

Use a **3.3 V USB-UART adapter** connected to UART0 on J3:

| Host signal | J3 pin | ESP32-S3 signal |
| --- | ---: | --- |
| Host TX to board RX | J3.4 | GPIO44 / U0RXD |
| Host RX from board TX | J3.3 | GPIO43 / U0TXD |
| Ground | J3.2 | GND |

Use full-duplex mode: `serial_rx` and `serial_tx` are different GPIOs. Do not
tie them together. Leave J3.1 (3V3) and J3.7 (USB VBUS) disconnected when the
board is powered by USB.

The normal handset CRSF rate is **400000 baud, 8N1, non-inverted**. The pinned
ExpressLRS `CRSFHandset.cpp` starts autobaud detection at 400000 and only cycles
through its handset baud table. The 420000 rate is the receiver-to-flight-
controller link, not the host-to-TX link.

The UART path remains available for existing J3 installations. It is not
required for the USB-only bench.

Native USB on GPIO19/20 supports:

- ROM flashing.
- ExpressLRS configuration and firmware-update workflows.
- CRSF RC frames and telemetry through the Aerial Kit native USB bridge.

## TX and RX roles

ExpressLRS still builds separate TX and RX applications:

- TX target: `AERIALKIT_ELRS_USB_TX_REV_A3_via_UART`.
- RX target: `AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART`.

The normal Rev A3 artifact contains both applications. On every reset, a small
bootloader selector samples GPIO4 and boots:

- GPIO4 low: the active TX slot (`tx_0` at `0x10000` or `tx_1` at `0x2b0000`);
- GPIO4 high: the active RX slot (`rx_0` at `0x450000` or `rx_1` at `0x590000`).

The active slot for each role is stored in two independently erasable
`role_state` records at `0x6d0000`. If the selected image is invalid, the
bootloader falls back only to the other slot for the same role. It never boots
the opposite RF role.

The factory image uses the QIO bootloader that matches the `qio_qspi` SDK
libraries used by both applications. The assembler derives the selector hook
and flash-map symbols from that bootloader ELF and refuses to build if
regenerating the unpatched ELF does not exactly match PlatformIO's
`bootloader.bin`.

This is a boot-time choice, not a live runtime switch. Change SW2 and reboot;
do not change roles with the board powered and RF active. Verify the switch by
continuity because the first prototype read low while its physical label
indicated RX.

Each application checks the same pin again before radio setup. If the switch
changed or the bootloader somehow selected the other application, both FEM
enables remain low and the application stops instead of opening the wrong RF
role.

The ExpressLRS Wi-Fi update path writes to the inactive slot for the running
role and commits that slot after a successful update. A full USB/ROM factory
flash is still required for the initial installation or recovery.

The factory image includes an empty LittleFS filesystem in the `spiffs`
partition at `0x6d2000`. This avoids first-boot mount/format failures caused by
stale filesystem contents from a different partition layout.

The RX image also outputs receiver frames on UART0 at the receiver-to-FC rate,
normally 420000 baud, unless configuration changes it. The USB CDC endpoint
operates in parallel.

## Firmware overlay

The board overlay is stored under `patches/` and pins ExpressLRS 4.1.0 at
commit `a9d4a9cb5b5687c4c9d7e9e7fbdf44ad93651da6`. It:

- supplies the Rev A3 target JSON;
- selects the ESP32-S3 2.4 GHz TX or RX environment;
- enables the SX1280 DC/DC path;
- maps TXEN/RXEN, SPI, USB and LED pins;
- limits the WS2812 to 26/255;
- adds two update slots per role and two role-state records;
- preformats LittleFS for the full factory image;
- bridges CRSF over native USB-CDC in both roles;
- emits a one-green-pulse TX or two-blue-pulse RX boot heartbeat;
- samples GPIO4 again before radio setup and fails safe on a role mismatch.
- derives the QIO bootloader and selector addresses from the actual build
  instead of pairing a DIO bootloader with QIO libraries;
- enters a visible red fault loop if target hardware metadata is unavailable.

Both applications and their combined flash image build in this repository.
No hardware was flashed or opened as part of this host-link milestone.

## Host integration decision

The host-side implementation will provide:

- a pure-Python CRSF encoder/decoder independent of ROS;
- a selectable serial transport for a real UART and an in-memory transport
  for tests/SIL;
- a ROS radio backend that keeps the existing `/uav/backend/*` contract;
- a separate simulated ELRS link node for loss, latency and packet-rate tests.
