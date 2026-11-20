# Ubuntu bring-up · ELRS USB TX Rev A3

## Identify and power the board

Match the AERIAL KIT / REV A3 silkscreen, ESP32-S3-MINI-1U module and layout. Check for assembly shorts with power removed. Use a known-good USB data cable and current-limited initial power. Check fused VBUS, main 3V3 (nominal 3.32 V), 3V3_RADIO and 3V3_PA. Inspect EN at TP1 and IO0 at TP2 if startup fails. Keep RF off during USB bring-up. See [hardware reference](HARDWARE.md).

## Enter ESP32-S3 ROM download mode

1. Hold SW1 **BOOT** to ground GPIO0.
2. While holding BOOT, plug in USB-C or momentarily ground TP1 (**EN/RST**) on an already powered board, then release TP1.
3. Release BOOT after reset. GPIO46 is pulled low by the board. To return to normal boot, reset with BOOT released or power cycle.

Compare `lsusb` and `/dev/serial/by-id/` before and after connecting. Native USB Serial/JTAG commonly appears as `/dev/ttyACM*`. Use the actual port. VM and WSL sessions need USB passthrough. On a permissions error, inspect the port's owner and group (commonly `dialout` on Ubuntu). [Espressif boot mode](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html) · [USB Serial/JTAG](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/usb-serial-jtag-console.html).

## Identify and back up before writing

Use esptool 5 syntax below. In a Python virtual environment, install with `python -m pip install "esptool>=5,<6"` if needed. Replace the example PORT with the actual device. Pick a unique backup filename.

```bash
PORT=/dev/ttyACM0
python -m esptool --chip esp32s3 --port "$PORT" flash-id
python -m esptool --chip esp32s3 --port "$PORT" get-security-info
python -m esptool --chip esp32s3 --port "$PORT" read-flash 0 ALL rev-a3-preflash.bin
sha256sum rev-a3-preflash.bin
```

Record the esptool version, chip revision, flash size and security state. Expect 8 MiB (8,388,608 bytes) from the BOM. Check the backup size and keep it local since flash may contain credentials. If the backup or security checks fail, diagnose before any write or erase. Encryption and Secure Download Mode can affect reads and recovery. [Esptool commands and flash protection](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/basic-commands.html).

## Recover through UART0

With USB supplying board power, use a **3.3 V logic** adapter. Wire J3.2 GND to adapter GND, J3.3 (board TX GPIO43) to adapter RX, and J3.4 (board RX GPIO44) to adapter TX. Leave J3.1 and J3.7 disconnected. Enter ROM using SW1 and TP1 as above and substitute the adapter's actual port (often `/dev/ttyUSB*`). Start at 115200 baud. If UART works and USB is missing, check the cable, connector, R4/R5 and USB assembly. [Espressif troubleshooting](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/troubleshooting.html).

## Select or build Rev A3 firmware

- This board calls for a custom ESP32-S3 + SX1280 + RFX2401C hardware target. The schematic proposes unified ExpressLRS ESP32-S3 2.4 GHz TX and RX bases. Check target names and JSON keys against the selected firmware and Targets commits. Populate the pins from the [ESP32 pin map](data/esp32-pin-map.csv). The schematic calls for SX1280 DC/DC mode, correct FEM TX/RX enable control and a validated power table.
- SW2 supplies a logic level on GPIO4. The current firmware ships one
  full-flash artifact with a boot-time selector: GPIO4 high starts TX and GPIO4
  low starts RX. Define the host-facing USB protocol separately; USB ROM
  flashing is independent of application USB data.
- Local `aerial-kit` commit `2bb8039295648437b6e4fc5fabced02c36a2aa09` contains `firmware/legacy/elrs/` for ESP32-C3, with different GPIOs and limited interoperability according to its README. `Integrated-UAV-Stack` commit `049d66e062bd837ee837e403cc1bf0603a48070a` also has older `ELRS/` environments for other hardware. Use these as reference code only. A verified Rev A3 firmware build or binary was not found in the sources inspected.
- Start with a board-specific USB/UART smoke test that holds both FEM enables low. Verify boot and USB/UART stability before bringing up SPI, BUSY, IRQ, SW2 and LED. For a selected image, use the build system's flash arguments and offsets. Record the build commit, command, artifact hashes and boot logs. Power cycle with BOOT released and verify normal boot before RF tests.

[ExpressLRS custom hardware guide](https://github.com/ExpressLRS/Docs/blob/master/docs/hardware/special-targets/diy-tx.md) · [ExpressLRS Targets](https://github.com/ExpressLRS/targets) · [ExpressLRS firmware](https://github.com/ExpressLRS/ExpressLRS).
