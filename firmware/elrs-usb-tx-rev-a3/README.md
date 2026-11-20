# ELRS USB TX Rev A3 firmware

Firmware support for the Aerial Kit ELRS USB TX Rev A3: an ESP32-S3-MINI-1U,
SX1280 and RFX2401C 2.4 GHz transceiver. The paired hardware sources are in
[`../../hardware/overlays/elrs-usb-tx/rev-a3/`](../../hardware/overlays/elrs-usb-tx/rev-a3/).

## Firmware families

The normal installation is one dual-role image. GPIO4 is sampled on every
reset: high starts the ExpressLRS TX application and low starts the RX
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

The factory image uses `bootloader_qio_80m.elf`, matching the applications'
`qio_qspi` SDK libraries. The assembler reads the bootloader symbols instead of
assuming fixed DIO addresses and requires the unpatched ELF conversion to
match PlatformIO's own `bootloader.bin` byte-for-byte.

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

## LED state

TX is always blue. RX is always green. State is shown only by pulse or
breathing pattern. Dual-role builds compile out the upstream `devRGB.cpp` LED
animations and the `esp32rgb.cpp` I2S WS2812 driver entirely. One IDF RMT
channel on GPIO21 (`expresslrs-overlay/src/aerialkit_ws2812.cpp`) drives the
LED with WS2812B-2020 timing: T0H 300 ns, T0L 950 ns, T1H 750 ns, T1L 600 ns,
and a 300 µs low latch after every frame. The classic 0.4/0.8 µs WS2812
timing used by Arduino `neopixelWrite()` is outside the WS2812B-2020 limits.
The only colors the LED code can produce are the role color, red and amber.

LED self-test: sending `C8 08 7F <addr> EA 41 4B 4C 54 <crc>` (address `0xEE`
on TX, `0xEC` on RX) shows pure red, green and blue for one second each
through the operational driver:

```sh
python3 tools/usb_crsf_bench.py /dev/ttyACM0 --baud 420000 \
  --rate 0 --duration 4 --led-selftest 0xEC --write
```

The six-order physical channel diagnostic uses `41 4B 4C 50` (`AKLP`) in the
same frame. It sends each known logical color for one second under every
possible wire order and prints `AK-LEDPERM` labels. See
`docs/evidence/rev-a3-led-permutation-diagnostic-2026-11-19.md`.

| State | TX | RX |
| --- | --- | --- |
| Boot identity | One 350 ms pulse | Two 350 ms pulses |
| Bind | Breathing, 3 s cycle | Breathing, 3 s cycle |
| Searching / no RF peer | Breathing, 1 s cycle | Breathing, 1 s cycle |
| Host/FC present, RF disconnected | Two 100 ms pulses/s | Two 100 ms pulses/s |
| RF connected | Steady | Steady |
| No sufficient serial input | One 100 ms pulse every 2 s | One 100 ms pulse every 2 s |
| Radio/SX1280 fault | Fast red, 100 ms on/off | Fast red, 100 ms on/off |
| Role/model mismatch | Three amber pulses, 1 s pause | Three amber pulses, 1 s pause |

Breathing stays in the role color, updates every 20 ms, and fades from
approximately 2% to 10% brightness. Bind persists for at least one complete
three-second cycle after it is observed.

The native USB CDC path is a CRSF transport in both roles. TX accepts
`RC_CHANNELS_PACKED` from the host and starts the normal ExpressLRS scheduling
path. RX emits `RC_CHANNELS_PACKED` to the host and accepts host telemetry.
The nominal rates are 400000 baud for TX-host and 420000 baud for RX-FC; USB
CDC ignores the requested baud rate. Two boards appear as distinct
`/dev/ttyACM*` devices.

Opening the port can reset the board. The ESP32-S3 USB Serial/JTAG resets
the chip while RTS is asserted and DTR is not. Linux asserts both lines on
`open()`, and pyserial then clears DTR before RTS, so "deassert both before
open" briefly presents exactly that reset state. Clear RTS first, then DTR;
`tools/usb_crsf_bench.py` does this. Resynchronize on a CRSF sync byte after
any reset or boot text.

After setup each application writes one ASCII line to native USB, for
example:

```text
AK-BOOT role=tx slot=tx_0 gpio4=high expect_gpio4=high reset=poweron(1) elf=1a2b3c4d led=rmt-ok led_failures=0 led_error=0x0 ws2812_t0h=300ns t1h=750ns latch=300us
```

It reports the compiled role, the running partition, GPIO4 at boot, the reset
reason (`brownout`, `panic`, `task_wdt` etc.), the ELF hash prefix, and LED
driver health from checked IDF RMT calls (`led_error` is the last `esp_err_t`). ASCII never contains `0xC8`, so CRSF parsers skip it.

Bench probe (TX: 150 Hz RC, a valid `C8 04 28 00 EA 54` ping each second,
concurrent reader, timed writes):

```sh
python3 tools/usb_crsf_bench.py /dev/ttyACM0 --duration 10
python3 tools/usb_crsf_bench.py /dev/ttyACM0 --rate 0 --telemetry   # RX
```

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

The 2026-11-17 diagnostic run passed ESP32-S3 boot, native USB, UART0, SX1280
SPI, WS2812, FEM control and shutdown tests. The SX1280 returned firmware
version `0xA9B7`. No remote I2C device was present.

The printed switch labels are authoritative. The final dual-role selector
therefore maps high GPIO4 to TX and low GPIO4 to RX. Both applications enforce
the same mapping before RF setup and hold both FEM enables low on disagreement.

On 2026-11-19, a hardware A/B test showed that the earlier `7210…` factory image
paired a DIO bootloader with QIO-built applications. The corrected QIO image
removed the panic and LittleFS failures. The final GRB color artifact is:

```text
size: 8331264 bytes
sha256: 7c6714425e4a2981ef787267df89627838e61b155e288536e66cf465faeada63
```

The operational wire order is the bench-confirmed G-R-B mapping. Role and
pattern logic are unchanged. Records are in `releases/7c671442/`; the final
hardware gate is one blue TX boot pulse at GPIO4 high, two green RX boot pulses
at GPIO4 low, then blue-only TX and green-only RX state patterns.

## Licensing

The ExpressLRS and target source trees remain under their upstream licences
and are not vendored here. The hardware sources were supplied by the project
owner; no hardware licence has been asserted yet. Choose and document one
before public distribution.
