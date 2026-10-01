# AK firmware

[Source and quick start](../firmware/README.md) - [Web configurator](web-configurator.md)

AK firmware is the custom flight-controller runtime in `firmware/`. This public tree includes two profiles:

| Target | Build | Hardware status |
| --- | --- | --- |
| Feather F405 (`FEATHER_F405`) | ARM GNU Toolchain + Make | Pin map and LSM6DSO support; verify the physical sensor/address/wiring on your board |
| ESP32 devkit (`ESP32DEV`) | ESP-IDF | ESP32 runtime port; physical devkit validation remains separate from recorded emulator results |

The C runtime includes control, estimation, flight-state handling, parameters and the configuration protocol. Only the STM32F405 and ESP32 architecture ports are included. The older PlatformIO projects live in [firmware/legacy/](../firmware/legacy/README.md).

## Feather F405

Install Make, Python 3 and ARM GNU Toolchain, with `arm-none-eabi-*` on PATH:

```sh
cd firmware
make
make check
```

The default is `FEATHER_F405`, part `stm32f405rg`. Images are written to `build/feather/`. An absolute `CROSS` prefix can select the compiler; add its directory to PATH for symbol checks. Use separate output directories for compile-time variants.

## ESP32

Install ESP-IDF and activate it by sourcing its `export.sh`:

```sh
cd firmware/ports/esp32
idf.py set-target esp32
idf.py build
```

The default configuration uses emulated Ethernet. For a physical devkit's Wi-Fi build, from the same directory:

```sh
idf.py -B build-wifi -DSDKCONFIG=build-wifi/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.wifi" build
```

Configure credentials through the parameter interface. Never commit credentials into a board definition or sdkconfig. This repository does not include the C3, S2, S3 or AT32F435 profiles.

## Configuration and evidence

Use the [web configurator](web-configurator.md) for identity, attitude, sensor diagnostics and parameters. Stage edits, send them to the board, then save to flash. The demo transport needs no hardware.

Image checks verify layout and symbols. They do not verify I2C, output waveforms, servo/ESC wiring or failsafe. No flight qualification is claimed. Historical core/register tests and host simulations were run against the full development tree; that larger C harness is not included here. Simulator-based Python tools and optional app SITL tests can use an externally built simulator.

## Licence

The runtime is [GPL-3.0-or-later](../firmware/LICENSE), separate from the root MIT-licensed Python/ROS package and existing archived link projects.
