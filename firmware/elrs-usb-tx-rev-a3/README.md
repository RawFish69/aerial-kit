# ELRS USB TX Rev A3 firmware

Firmware support for the Aerial Kit ELRS USB TX Rev A3: an ESP32-S3-MINI-1U,
SX1280 and RFX2401C 2.4 GHz transceiver. The paired hardware sources are in
[`../../hardware/overlays/elrs-usb-tx/rev-a3/`](../../hardware/overlays/elrs-usb-tx/rev-a3/).

## Firmware families

Three images are provided:

| Image | Purpose | RF state |
| --- | --- | --- |
| `diagnostic` | Board bring-up and non-RF subsystem test | TXEN and RXEN held low |
| `tx` | ExpressLRS 4.1.0 transmitter | RFX2401C controlled by ELRS |
| `rx` | ExpressLRS 4.1.0 receiver | RFX2401C controlled by ELRS |

ExpressLRS TX and RX are separate builds. The physical SW2 selection does not
switch a single image at runtime.

## Hardware mapping

| Signal | GPIO |
| --- | ---: |
| UART0 RX / TX | 44 / 43 |
| SX1280 NSS / MOSI / SCK / MISO | 10 / 11 / 12 / 13 |
| SX1280 RESET / BUSY / DIO1 | 14 / 9 / 8 |
| RFX2401C TXEN / RXEN | 6 / 7 |
| Role select | 4 |
| WS2812 data | 21 |
| I2C SDA / SCL | 34 / 48 |

The firmware limits the WS2812 to 26/255, approximately 10% brightness.
The SX1280 power table is `[-18, -14, -10, -6, +5]` dBm and defaults to the
lowest entry. The +5 dBm maximum is the documented input limit of the
RFX2401C, not a measured board output.

## Build

PlatformIO and the ESP32 Arduino 2.0.17 framework are required.

The build scripts use `ELRS_USB_TX_CACHE` for the local Arduino framework and
ESP32-S3 toolchain. On this development machine the existing cache is at
`~/Projects/elrs-usb-tx/toolchain`; set the variable explicitly if the cache
is elsewhere.

```sh
./scripts/bootstrap.sh
./scripts/build.sh diagnostic
./scripts/build.sh tx
./scripts/build.sh rx
```

`bootstrap.sh` creates an ignored `.vendor/` tree with ExpressLRS 4.1.0 and
the pinned ExpressLRS Targets database, then applies this project's small
overlay. No upstream source is vendored into Git.

## Flash

```sh
./scripts/flash.sh diagnostic /dev/ttyACM0
./scripts/flash.sh tx /dev/ttyACM0
./scripts/flash.sh rx /dev/ttyACM0
```

The diagnostic image should be flashed first with the antenna disconnected.
It verifies boot, USB, UART, SX1280 SPI and firmware version, BUSY/DIO1 idle
states, FEM enable control, role and strap inputs, I2C bus presence, J5 GPIO
states and the WS2812.

The diagnostic image can assert each FEM enable for an electrical pin test,
but it never commands the SX1280 to transmit. It is safe on an open RF
connector.

Before running any ELRS TX/RX image, attach a suitable 2.4 GHz antenna or a
50 ohm terminated RF path. Actual PA output, receiver sensitivity, link
acquisition and range cannot be validated without one.

## Current bench record

The 2026-11-17 diagnostic run passed ESP32-S3 boot, native USB, UART0,
SX1280 SPI, WS2812, FEM control and shutdown tests. The SX1280 returned
firmware version `0xA9B7`. No remote I2C device was present.

GPIO4 read low while the switch was physically labelled RX. That conflicts
with the design note (`low=TX`, `high=RX`), so the switch orientation must be
checked by continuity before relying on its label.

## Licensing

The ExpressLRS and target source trees remain under their upstream licences
and are not vendored here. The hardware sources were supplied by the project
owner; no hardware licence has been asserted yet. Choose and document one
before public distribution.
