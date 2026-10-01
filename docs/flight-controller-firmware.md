# AerialKit flight-controller firmware

[Firmware source](../aerialkit/) · [Web configurator](web-configurator.md) · [Build reference](../aerialkit/docs/28-build.md)

AerialKit is the custom C flight-controller implementation in this repository. Its portable core covers control, estimation, parameters, configuration protocol and flight-state handling. Hardware ports and board definitions live under `aerialkit/src/arch/` and `aerialkit/src/boards/`.

## Board support and evidence

| Board / port | Current evidence | Remaining work |
| --- | --- | --- |
| WeAct STM32F405 (`AERIALKIT_F405`) | USB enumeration and configurator communication recorded on the bench; 12 MHz crystal accounted for by the clock measurement | Real vehicle wiring, outputs and failsafe validation |
| Feather STM32F405 (`FEATHER_F405`) | Dedicated pin map and LSM6DSO driver; I2C address/ACK/bus-recovery fixes present in source | Confirm the physical sensor/address/wiring on each board; no flight qualification |
| GHF435 / AT32F435 (`AERIALKIT_GHF435`) | Build and image checks; host register tests | AerialKit image not hardware validated on this target |
| ESP32 / C3 / S2 / S3 | ESP-IDF ports and simulator/QEMU evidence where recorded | Physical board and flight validation per target |

These statements describe existing bench records and source support. They do not establish that every board is a ready-to-fly flight controller. No AerialKit flight is claimed.

## ARM quick start

Install GNU Make, Python 3 and ARM GNU Toolchain. For host tests install a native C compiler. From the repository root:

```sh
cd aerialkit
make BOARD=AERIALKIT_F405 OUT=build/f405
make BOARD=AERIALKIT_F405 OUT=build/f405 check
make BOARD=FEATHER_F405 PRODUCT=aerialkit-feather-f405 OUT=build/feather
make BOARD=FEATHER_F405 PRODUCT=aerialkit-feather-f405 OUT=build/feather check
make test
make boards-check
```

If the toolchain is outside PATH, pass `CROSS=/path/to/toolchain/bin/arm-none-eabi-` to Make and add that directory to PATH for image checks. Images and ELF files remain in the chosen output directory. Use separate directories for different boards.

Use the [F405 bring-up guide](../aerialkit/docs/05-bringup.md) before flashing. A firmware build is not a reason to overwrite a board's existing image. Read the trace/evidence before any flash that may erase it.

For ESP32, follow the [ESP-IDF build reference](../aerialkit/docs/28-build.md). For other ARM parts, set BOARD, ARCH, PART and PRODUCT together as listed there.

## Configure and verify

Start the [web configurator](web-configurator.md), connect via USB serial, check identity and sensor diagnostics, then stage parameter edits. Sending values and saving to flash are separate operations. The app's demo transport is useful for learning this flow without hardware.

Host tests exercise the portable core and register harness; image checks validate layout and symbols. Physical I2C, PWM/DShot, servo/BEC/ESC wiring and failsafe still need board-level checks. Historical results are in [firmware evidence](../aerialkit/docs/evidence/).

## Licence

This component is GPL-3.0-or-later, under [aerialkit/LICENSE](../aerialkit/LICENSE). It is separate from the root MIT-licensed Python/ROS package.
