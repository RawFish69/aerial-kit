# AerialKit Firmware

![AerialKit](../apps/configurator/public/brand/aerialkit-selected.svg)

A custom C flight controller with a portable control core, a host simulator, and STM32F405, ESP32 and AT32F435 board ports. Supports multirotor and fixed-wing control; see the [design](docs/01-plan.md) and [board definitions](src/boards/).

**Bench status:** STM32F405 USB enumeration and configurator communication have been observed. Feather F405 has its own board definition and LSM6DSO support. This firmware has not been flown; output waveforms, wiring, failsafe and vehicle tuning still require hardware validation. Build checks are not flight qualification. See [board status](../docs/flight-controller-firmware.md).

## Build

Needs GNU Make, Python 3, a native C compiler for host tests, and ARM GNU Toolchain for ARM images. Put `arm-none-eabi-*` on PATH or set `CROSS=/path/to/toolchain/bin/arm-none-eabi-`.

```sh
git clone https://github.com/RawFish69/aerial-kit.git
cd aerial-kit/aerialkit
make BOARD=AERIALKIT_F405 OUT=build/f405
make BOARD=AERIALKIT_F405 OUT=build/f405 check
make BOARD=FEATHER_F405 PRODUCT=aerialkit-feather-f405 OUT=build/feather
make BOARD=FEATHER_F405 PRODUCT=aerialkit-feather-f405 OUT=build/feather check
make test
make boards-check
```

For AT32F435 use `BOARD=AERIALKIT_GHF435 ARCH=at32f435 PART=at32f435rg PRODUCT=aerialkit-ghf435` with a separate output directory. ESP32 ports build through ESP-IDF; they do not use the ARM Makefile invocation.

## Configure

The [web configurator](../apps/configurator/README.md) lives in this repository alongside the firmware. It connects over USB serial, or to the host simulator through the local bridge. Its in-browser demo needs no board.

## Documentation

- [Firmware quick start, hardware status and verification](../docs/flight-controller-firmware.md)
- [Architecture](docs/01-plan.md) · [Decisions and licence](docs/00-decisions.md)
- [Build, test stages and image checks](docs/28-build.md)
- [STM32F405 bring-up](docs/05-bringup.md) · [AT32F435 bring-up](docs/26-ghf435-bringup.md)
- [Configuration protocol](docs/27-configurator.md) · [Board inventory](contract/)
- [Bench evidence](docs/evidence/) — historical records, not a claim that every port has run on hardware.

The component path is `aerialkit/`; the independent Python package is `aerial_kit/`.

## Licence

GPL-3.0-or-later. See [LICENSE](LICENSE) and [licensing decisions](docs/00-decisions.md).
