# AK Firmware

The custom C flight controller for **Feather F405** and **ESP32DEV**. This directory contains the runtime source and board build files. Older radio/link projects are in [legacy/](legacy/README.md).

## Feather F405

Install GNU Make, Python 3 and ARM GNU Toolchain. Put `arm-none-eabi-*` on PATH, then:

```sh
cd firmware
make
make check
```

The default image is `build/feather/aerialkit-feather-f405.bin`. Set `CROSS=/path/to/toolchain/bin/arm-none-eabi-` if needed; image checks also require that toolchain directory on PATH. `EXTRA_CFLAGS` selects fitted-hardware options and diagnostic instruments. Build a different variant in a separate OUT directory.

Feather F405 includes its pin map and BNO055/LSM6DSO support. Check the physical sensor address, I2C wiring and board diagnostics before relying on it. The firmware has not been flown.

## ESP32DEV

Install ESP-IDF and source its `export.sh`, then:

```sh
cd firmware/ports/esp32
idf.py set-target esp32
idf.py build
```

This project accepts only `AK_BOARD=ESP32DEV`. The default network profile supports the emulated Ethernet environment; use `sdkconfig.wifi` for a physical devkit's Wi-Fi build. See the [firmware guide](../docs/flight-controller-firmware.md) for both invocations.

See [current runtime and protocol notes](docs/35-public-runtime-sync.md) and the
[Feather pin map](docs/34-feather-pinmap.md).

## Configure and verify

- [Web configurator](../apps/configurator/README.md)
- [Firmware guide and target status](../docs/flight-controller-firmware.md)
- [Source](src/) - portable core, STM32F405/ESP32 ports and two board profiles
- [Historical design/evidence](docs/) - records from the full verification tree

The large C host-test and simulator harnesses, AT32F435 port, other ESP32 variants and WeAct board profile are excluded from this public component. Their original source remains in the development workspace and Git history. The public Makefile builds/checks the Feather image; it does not offer the former `make host`, `make test` or all-target CI commands. Python protocol clients remain available under `tools/`; simulator-based tools require an externally built simulator.

## Licence

AK firmware is GPL-3.0-or-later; see [LICENSE](LICENSE). The archived projects in `legacy/` retain their existing licensing.
