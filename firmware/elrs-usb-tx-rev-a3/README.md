# ELRS USB TX Rev A3 firmware

Firmware support for the Aerial Kit ELRS USB TX Rev A3: an ESP32-S3-MINI-1U,
SX1280 and RFX2401C 2.4 GHz transceiver. The paired hardware sources are in
[`../../hardware/overlays/elrs-usb-tx/rev-a3/`](../../hardware/overlays/elrs-usb-tx/rev-a3/).

## Firmware families

The normal installation is one dual-role image. GPIO4 is sampled on every
reset: low starts the ExpressLRS TX application and high starts the RX
application. Moving SW2 and rebooting changes roles without reflashing.

| Image | Purpose | RF state |
| --- | --- | --- |
| `diagnostic` | Board bring-up and non-RF subsystem test | TXEN and RXEN held low |
| `dual-role` | One 8 MB flash image containing the boot selector, TX and RX | RFX2401C controlled by the selected ELRS role |
| `tx` | Optional role-only development image | RFX2401C controlled by ELRS |
| `rx` | Optional role-only development image | RFX2401C controlled by ELRS |

The TX and RX applications remain separate ExpressLRS builds. The flash has
two update slots per role:

| Partition | Offset | Purpose |
| --- | ---: | --- |
| `tx_0` | `0x10000` | initial TX application |
| `tx_1` | `0x2b0000` | TX Wi-Fi OTA target |
| `rx_0` | `0x450000` | initial RX application |
| `rx_1` | `0x590000` | RX Wi-Fi OTA target |
| `role_state` | `0x6d0000` | active TX and RX slot records |
| `spiffs` | `0x6d2000` | preformatted LittleFS |

Only a small bootloader selector is custom. It runs before partition selection
on every boot, reads the role-state records, and falls back only to the other
slot for the same role. Each application also checks GPIO4 at startup and
holds both FEM enables low if it was booted in the wrong role.

Wi-Fi OTA updates the inactive slot for the running role and records the new
slot only after `Update.end()` succeeds. Use the full factory artifact for the
initial install and recovery.

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
./scripts/build.sh
./scripts/build.sh diagnostic
```

`./scripts/build.sh` (or `./scripts/build.sh dual`) builds both ExpressLRS
applications, creates the role-state records and an empty LittleFS image, and
produces:

- `dist/aerialkit-rev-a3-elrs-dual-role.bin`, the single full-flash image;
- `dist/aerialkit-rev-a3-elrs-dual-role.json`, role mapping, offsets and hashes.

The role-only `tx` and `rx` modes remain available for isolated development.

`bootstrap.sh` creates an ignored `.vendor/` tree with ExpressLRS 4.1.0 and
the pinned ExpressLRS Targets database, then applies this project's small
overlay. No upstream source is vendored into Git.

## Flash

```sh
./scripts/flash.sh /dev/ttyACM0
./scripts/flash.sh diagnostic /dev/ttyACM0
./scripts/flash.sh tx /dev/ttyACM0
./scripts/flash.sh rx /dev/ttyACM0
```

With no mode, `flash.sh` writes the factory artifact at offset `0x0`. This is
a normal USB/ROM flash, not an OTA update. It replaces the partition table,
NVS, initial role slots, role state and LittleFS. The unused per-role OTA slots
are left erased.

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
checked by continuity before relying on its label. The dual-role mapping uses
the electrical design, not the printed label.

## Licensing

The ExpressLRS and target source trees remain under their upstream licences
and are not vendored here. The hardware sources were supplied by the project
owner; no hardware licence has been asserted yet. Choose and document one
before public distribution.
