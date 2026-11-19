# Rev A3 QIO bootloader fix — 2026-11-19

## Finding

The four-slot factory image assembled through 2026-11-19 paired a DIO
bootloader with applications linked against the ESP32-S3 `qio_qspi` SDK.
PlatformIO intentionally rewrites the application image header from QIO to DIO
for ESP32-S3 Arduino builds, so the header alone did not expose the mismatch.

The failure appeared after boot:

- flash reads returned incorrect data;
- `ESP.getSketchSize()` could not find the appended target layout;
- LittleFS reported a corrupted directory pair and could not format it;
- TX remained dark with no native USB output;
- RX dereferenced an uninitialized serial endpoint and entered a panic loop.

## Hardware A/B evidence

The bench operator flashed an unversioned diagnostic image containing the same
TX and RX application bytes with only the bootloader changed from
`bootloader_dio_80m.elf` to a selector-patched
`bootloader_qio_80m.elf`. The resulting image booted without invalid-image,
LittleFS, or panic messages, and the WS2812 initialized.

That diagnostic image is not a release and must not be used.

## Permanent changes

- The assembler reads the SDK memory type from the application map and selects
  `bootloader_qio_80m.elf` for `qio_qspi`.
- Bootloader hook and flash-map symbols are read from the selected ELF with
  `nm`; selector addresses are no longer fixed to DIO values.
- The chosen unpatched ELF is converted with the same esptool and flags as the
  PlatformIO build and must be byte-identical to `bootloader.bin`.
- The manifest records the bootloader ELF, SDK memory type, application header
  mode, image mode and selector-patched bootloader hash.
- Both applications check fixed Rev A3 role/FEM pins before target options are
  loaded. A mismatch holds TXEN and RXEN low and enters a visible red fault
  pattern.
- Missing hardware configuration now enters the same visible fault loop and
  prints `AK-FAULT hardware layout not loaded`; it no longer crashes on a null
  serial endpoint.
- The role heartbeat runs before target options and before the normal LED
  driver: one blue pulse for TX and two green pulses for RX at 26/255.
- The normal LED driver checks I2S calls and falls back to RMT if I2S cannot
  initialize or write.
- Arduino core logging is disabled for these target environments so text does
  not interleave with CRSF after boot.
- Bootstrap inputs are content-stamped. A changed patch or overlay resets the
  ignored ExpressLRS and target vendor trees before applying the stack.

## Validation

- TX and RX ExpressLRS builds: passed.
- Native USB/CRSF and LED tests: 9 passed.
- Role, selector, trailer and factory-image tests: 8 passed.
- Source bootloader byte comparison: passed.
- Factory image assembler: passed.
- No board was powered, flashed, opened, or commanded during this code work.

## Release artifact

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: c1668c40f87a94452bafbaca281b11799a6e2a187a6553e04866b3eba80524d4
flash offset: 0x0
bootloader: bootloader_qio_80m.elf
SDK memory type: qio_qspi
```

Hardware acceptance is still required. The RX/TX role heartbeat colors, CRSF
enumeration, device information, RC direction, and RF link statistics must be
recorded from this exact image before calling the board bench-validated.
