# Rev A3 RMT LED final artifact — 2026-11-19

## Result

The factory image was rebuilt from the isolated final source tree after replacing
the dual-role I2S WS2812 path with a checked IDF RMT driver.

```text
path: /home/cy/Projects/aerial-kit-firmware-final/firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 5fbc25c9d23b32b33859a7ae02fce5f69aee718cb60c217850d241fc45317838
timestamp: 2026-11-19 07:03:42 -0800
flash offset: 0x0
bootloader: bootloader_qio_80m.elf
bootloader SDK type: qio_qspi
application header flash mode: dio
```

The manifest and `SHA256SUMS` agree with that identity.

A fresh rebuild from the final source produced this identity after the previous
uncommitted candidate at `532f348b...`. Comparing the two complete images found
68 changed bytes: the two appended options blocks differed only in ExpressLRS's
per-build `flash-discriminator`, and the remaining bytes were `mklittlefs`
metadata. The executable TX and RX image slices were byte-identical. This
verifies that `5fbc25c9...` corresponds to the current source and that the
earlier candidate differed only in build-generated metadata.

## Source correspondence

The candidate previously observed at 5e9d6c61... was not frozen. A concurrent
bench worktree rebuild changed it to 5576f902..., and the shared source then
advanced to a dedicated RMT WS2812 driver. The shared worktree was left
unmodified. Its latest firmware changes were copied into the isolated branch
and rebuilt from current `main`.

The final build supersedes all earlier candidate hashes. It also fixes the
dual-role build command to compile both PlatformIO environments in one
invocation; separate invocations removed the first environment's build output.

Final application hashes:

```text
TX fb33ccf0ab7bb4c4af2fb1538aeb77965d9a01efba946fe7091a328a43295ce7
RX 1d842176aa52c79d1c336616846632667ccae83012f64bde3048700048bf8d5c
```

Both application maps resolve `qio_qspi`. Both application images declare DIO
in their image headers while running with the QIO bootloader.

## LED implementation

- Dual-role builds compile out upstream `devRGB.cpp` animations and
  `esp32rgb.cpp` I2S output.
- One IDF RMT driver on GPIO21 is shared by boot identity, fault loops, normal
  state display and the explicit LED self-test.
- The driver checks every IDF return, reads back APB source/divider, includes an
  explicit low latch period and reports failures.
- WS2812 waveform: T0H 300 ns, T0L 950 ns, T1H 750 ns, T1L 600 ns, 300 us
  latch. The wire word is GRB after the 26/255 brightness cap.
- TX is blue and RX is green. Bind uses three-second role-color breathing;
  searching uses one-second role-color breathing; connected is steady.
- Radio fault is fast red; model/role mismatch is three amber pulses then a
  pause.
- An explicit CRSF `0x7F` bench frame with `AKLT` starts one-second pure red,
  green and blue self-test phases through the same production driver.

## Native USB review

- TX drains native USB every loop, accepts valid RC_CHANNELS_PACKED frames,
  starts the normal handset scheduler, returns telemetry through the router and
  declares no-handset after 500 ms without RC.
- RX runs an additional `SerialCRSF` endpoint on the same native USB port. It
  emits 26-byte RC frames, parses host telemetry and queues it through the
  normal RF telemetry path.
- The standalone USB probe opens with a deassertion order that avoids the
  ESP32-S3 USB-JTAG reset combination, requires `--write` for RC/telemetry/
  self-test/reset operations and sends a throttle-low then arm-low sequence on
  exit.
- The earlier 15-frame stall remains a hardware observation to reproduce. The
  final image emits `AK-BOOT ... reset=<reason> ... led=...` after startup so a
  reset, brownout, watchdog and LED error are distinguishable on the USB stream.

## Validation

- Isolated TX and RX PlatformIO builds: passed.
- Native ExpressLRS tests: 103 passed, including all 9 native USB/CRSF and LED
  state tests.
- Python firmware/LED/tool tests after the final build: 23 passed.
- Linked TX and RX ELFs: no upstream HSV, rainbow, brightness fade, ESP32 I2S
  LED or `WS281B` symbols; required RMT/boot/state symbols present.
- QIO bootloader regenerated source matches PlatformIO bootloader byte-for-byte.
- Manifest, `SHA256SUMS`, role layout and application hashes verified.

No hardware was flashed, reset, opened or commanded during this work.
