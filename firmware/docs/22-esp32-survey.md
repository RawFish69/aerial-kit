> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - what already exists for ESP32 flight control

The goal asks for this survey **before** any of the ESP32 milestone is written:

> Survey what already exists for ESP32 flight control (Espressif's ESP-Drone is
> one place to start) before writing any of it.

This is that survey, written 2026-09-16. It says what was looked at, at which
revision, what each thing is good for, and what it leaves for us. It is a
reading list and a set of decisions, not a plan: the plan is
[01-plan.md](01-plan.md) M6, and what AerialKit's own ESP32 port already does is
[17-esp32-port.md](17-esp32-port.md).

**Nothing was copied.** No source was read line by line for this page, nothing
moved into this repository, and [03-attribution.md](03-attribution.md) is
unchanged by it. Two of the four projects below are GPL-3.0 and AerialKit is
GPL-3.0, so copying is *permissible* with attribution - but the working rule in
this repository is read, understand, reimplement, and the licence only decides
what is allowed if that ever changes.

## What was looked at, and how

| Source | Where | Revision | Licence |
| --- | --- | --- | --- |
| Betaflight's ESP32 platform | `upstream/betaflight-2026.6.1/src/platform/ESP32/` | `6dbc4218` (pinned in this workspace) | GPL-3.0 |
| INAV | `upstream/inav-9.1.0/` | `e519b69` (pinned) | GPL-3.0 |
| ArduPilot's ESP32 HAL | `github.com/ArduPilot/ardupilot`, `libraries/AP_HAL_ESP32` | `b2b1b3d2` (master, 2026-09-16) | GPL-3.0 |
| ESP-Drone | `github.com/espressif/esp-drone`, README | `db0f6562` (master, 2026-06-04) | GPL-3.0 |
| Project-level search | GitHub search API, `esp32 flight controller`, sorted by stars | 2026-09-16 | mixed |

ArduPilot and PX4 are pinned in `upstream/revisions.json` but are **not checked
out** on this machine, so what is said about ArduPilot below comes from its
source tree read through the GitHub API, not from a local checkout; PX4 has no
ESP32 platform at all (`platforms/` is `common, nuttx, posix, qurt, ros2`), which
is worth knowing because it means the aircraft-grade RTOS-and-HAL approach does
not cover this part.

INAV is in the same position from the other side: **zero ESP32 files**. Of the
four families this project reads, two have nothing for this chip.

## The three that do

### Betaflight's ESP32 platform - the closest thing to our own problem

An in-tree checkout, 73 files under `src/platform/ESP32/`, covering four chips:
ESP32, ESP32-S3, ESP32-P4, ESP32-C5, with targets (`ESP32WROOM`, `ESP32S3`,
`ESP32P4`, `ESP32C5`). It is the most directly useful reference here because it
is the *same shape of port*: a flight controller that is otherwise written for
STM32, with a platform directory that supplies only what the chip has to.

What it settles, with the files that settle it:

| Question | Betaflight's answer |
| --- | --- |
| How does the firmware get built? | ESP-IDF, hydrated as a **submodule** and pinned by revision (`src/platform/ESP32/mk/ESP32.mk`: `PLATFORM_SDK := esp_idf`, `ESP_IDF_DIR = $(LIB_MODULES_DIR)/esp-idf`) - the same choice this project made in [00-decisions.md](00-decisions.md) #6, made independently |
| What does the board get flashed with? | a **merged image**: IDF's bootloader + a partition table + the app, made with `esptool elf2image` and a merge step, flashed at `0x0` rather than as a flat binary at the app offset |
| Where do parameters live? | their own partition: `partitions.csv` reserves `config` (`data, 0x40`, 64 KB at `0x210000`) which `config_flash.c` maps and writes directly - not NVS, and not IDF's blob API |
| What does the console run on? | UART and USB-serial-JTAG (`serial_uart_esp32.c`, `serial_usb_vcp_esp32.c`). **Their ESP32 port has no Wi-Fi in it at all**: no source file under `src/platform/ESP32/` mentions `esp_wifi` or `esp_netif`, and the only file there that contains the string "wifi" is a prebuilt IDF bootloader blob |
| What is not done? | DMA: `dma_stub_esp32.c` is what ESP32, ESP32-P4 and ESP32-C5 build, and only ESP32-S3 uses the real `dma_esp32.c` (`mk/ESP32*.mk` says which) |

The last line of that table is the important one for us: the most mature
flight-controller project that has an ESP32 port **did not put the link on
Wi-Fi**. The goal's M6 asks for Wi-Fi config and telemetry because that is the
reason this target exists for *us*, so their port answers the platform
questions and leaves ours untouched - and it is worth saying plainly that
choosing Wi-Fi is choosing the part nobody in the reference set ships as their
primary link.

### ArduPilot's ESP32 HAL - the one that did put MAVLink on Wi-Fi

`libraries/AP_HAL_ESP32`, with its own README (488 lines of build and wiring
notes), boards `esp32buzz` and `esp32diy`, also built on IDF as a submodule
(`Tools/scripts/esp32_get_idf.sh`, `modules/esp_idf`).

What it has, from its own README's progress list: a Wi-Fi driver that comes up
as an **access point** (`192.168.4.1:5760`, Mission Planner or mavproxy
connect to it), MAVLink on console/USB, and MAVLink over Wi-Fi as **either TCP
or UDP chosen at compile time**; GPS on UART1; RC in through the RMT peripheral
or a software reader (`RmtSigReader`, `SoftSigReaderInt`); RC output through
RMT; a compass on I2C; a GY-91 example; SD card; parameters in a `storage`
partition.

And what it says about the hard parts - the most useful lines in the whole
survey, because they are the failure modes we would otherwise rediscover:

- "**slow but functional wifi implementation** for tcp & udp";
- "UDP mavlink over wifi does not automatically stream to client/s when they
  connect to wifi AP";
- "Fix parameters loading in wifi both udp and tcp (**slow and not reliable**)";
- and, on the aircraft side: "we only have 6 channels of output due to pin
  limitations".

Two conclusions for AerialKit. First, **parameters over the link are the part
that goes wrong**: our config protocol already has a checksummed, line-oriented
parameter dump, and the ESP32 port must not gain a second, half-working path
for the same thing. Second, **the AP is the sane default**: a board that is its
own access point with a known address is configuration-free at the field, and
joining somebody else's network is the case that needs stored credentials.

### ESP-Drone - the product shape, from Espressif

Espressif's own ESP32 drone, with the README fetched rather than remembered:
**GPL-3.0**, "the main code is ported from **Crazyflie**", builds against
**ESP-IDF release/v5.0**, runs on ESP32 / ESP32-S2 / ESP32-S3, is controlled by
a phone app or a gamepad **over Wi-Fi**, and offers stabilize, height-hold and
position-hold modes with extension boards. There are first-party iOS and
Android apps in separate repositories, and it speaks enough of Crazyflie's
protocol that `cfclient` can drive it.

That is the closest thing to a *product* answer in this space, and it is worth
reading for the shape of it: a fixed AP, an app that finds the aircraft, modes
selected from the app, and a documented hardware reference. It is also the
wrong shape for this project in one specific way - it is Crazyflie's
architecture underneath, so its estimator, mixer and parameter system are not
ours, and porting it would be adopting a different flight stack rather than
adding a target.

### What the search turned up, unread

From a GitHub search for `esp32 flight controller` sorted by stars, listed here
as *candidates* and explicitly not as references this page has read: `esp-fc`
(MIT, about 830 stars), `madflight` (MIT, about 490 - ESP32, RP2040 and STM32 in
one codebase), `ESP32-Flight-controller-` (MIT, Arduino-based), `cortex` (MIT,
ESP32-S3 PID stabilisation), `ESP_Pilot` (GPL-3.0), `Open-ESPilot`
(CERN-OHL-W-2.0, and a hardware licence rather than a software one).

The permissive ones matter for exactly one reason: if AerialKit ever *does* want
to take code rather than reimplement it, MIT is compatible with our GPL-3.0 and
the attribution entry is short. Until that day they are a reading list, and none
of them has been read for this page.

## What this survey decides

1. **ESP-IDF stays, as a pinned submodule** - three independent ports agree
   (Betaflight, ArduPilot, ESP-Drone), and the alternative is writing an app
   image format and bringing up the flash cache by hand.
2. **The AP is the default link, and joining a network is the configured
   case.** Today AerialKit's SSID and password are compile-time Kconfig values
   (`ports/esp32/main/Kconfig.projbuild`, `CONFIG_AK_NET_WIFI_SSID`): the same
   firmware needs a different build per network, which is not configuration.
   Runtime Wi-Fi configuration, stored where it survives a reflash, is the M6
   gap this survey leaves open.
   **Done 2026-09-16**: the credentials are parameters in the one table
   (`wifi_ssid`, `wifi_pass` - a secret, `wifi_ap_ssid`, `wifi_mode`), the
   access point is refused without a password of eight characters or more
   rather than coming up open, and `save` keeps them across a reflash. What it
   cost is worth reading as well:
   [17-esp32-port.md](17-esp32-port.md) has the text parameter type, the table
   that was silently full at 64, and the configuration record that was already
   111 bytes too small.
3. **The ESP32 gets a partition table of its own**, with a reserved region for
   parameters and (later) the blackbox - Betaflight's `config` partition and
   ArduPilot's `storage` partition are the same pattern, and AerialKit's F405
   layout already has the equivalent ([02-hardware.md](02-hardware.md)).
   **Done 2026-09-16**: `ports/esp32/partitions.csv` reserves a 704 KB
   `aerialkit` region for the log, the configuration stays in IDF's `nvs`, and
   the same `src/core/ak_flashlog.c` that writes the F405's log writes this one
   through `esp_partition_*`. The check pulls its records over the network in
   QEMU ([17-esp32-port.md](17-esp32-port.md)).
4. **Telemetry keeps speaking AerialKit's protocol**, over TCP as it does now,
   with UDP as a later option. What is *not* to be repeated is ArduPilot's
   known failure: a stream that does not start when a client connects, and
   parameter loading that is "slow and not reliable" over the link.
5. **RC in and out on this chip is an RMT problem** (ArduPilot's
   `RmtSigReader`/`RCOutput`), which matters for M3 when the target stops being
   a platform exercise and starts driving ESCs.
6. **Nothing from any of them is copied**, and the licences allow it if that
   changes: GPL-3.0 for Betaflight, ArduPilot and ESP-Drone, MIT for most of the
   community projects. Any future borrowing goes in
   [03-attribution.md](03-attribution.md) with project, file, revision, what
   was taken, and licence.

What this page cannot decide, because it is not a software question: **which
ESP32 board AerialKit targets**. Nothing here has been run on hardware - the
port's evidence is a QEMU boot ([17-esp32-port.md](17-esp32-port.md)) - and the
board is still uninventoried in [02-hardware.md](02-hardware.md).
