# Rev A3 WS2812 output path and native-USB bench probe — 2026-11-19

Current status: resolved. Candidate `19c7b3ce…` was superseded after the
six-order hardware diagnostic confirmed physical G-R-B. The current final
artifact, including the labeled-role inversion, is in
`rev-a3-role-inverted-final-2026-11-19.md`; the sections below are historical.

## Superseded candidate: `5e9d6c61…` (hardware FAIL, LED)

```text
sha256: 5e9d6c6165bd2698239931da49f2bb1a7043b5847eb1ca764d8e72f6212ebb5e
TX app: 54d418a94bf3bb5da4f51632310c9968380848b62d8e55257c607884b320385f
RX app: a31b86091d6bbff9824df8cf8a1260be199679e65f6a3309fd20288322936a97
```

This candidate removed the upstream I2S WS2812 driver and LED animations from
the dual-role build. The linked ELFs contained no `HsvToRgb`, `blinkyUpdate`,
`hueFadeLED`, `ESP32LedDriver` or `i2s_write`. The LED was driven by the
Arduino RMT wrapper.

Bench, RX position, fresh boot:

```text
AK-BOOT role=rx slot=rx_0 gpio4=high reset=unknown elf=9ee8a276 led=rmt-ok led_failures=0
product=AK USB RX A3 origin=0xEC destination=0xC8 firmware=4.1.0
```

Role, slot, application and LED state were correct, but the LED showed
changing colors from across the palette. Do not flash this hash again.

The `5e9d6c61…` ELFs were lost when a native-env `pio test` cleaned the build
directory. The factory image and manifest are intact; their app, bootloader and
LittleFS hashes were re-verified.

### Cause

1. **Waveform out of specification.** Both the upstream I2S driver and the
   Arduino `neopixelWrite()` timing (copied into `5e9d6c61…`) send classic
   WS2812 bits: T0H 400 ns, T1H 800 ns, T0L 800 ns, T1L 400 ns, 1.2 µs per
   bit. LED1 is a WS2812B-2020. The majority of its V6 datasheet copies give
   T0H 220–380 ns, T0L/T1H/T1L 580 ns–1 µs, at least 1.25 µs per bit and a
   reset above 280 µs. One copy gives T1L 220–420 ns. A 400 ns zero-bit high
   time sits at the part's decision threshold, so zero bits are misread. The
   diagnostic image only sent the constant value 26 (`00011010`); the state
   machine's breathing sweeps every value, so misreads change color
   continuously. This also accounts for the earlier I2S "blinking rainbow"
   (the state machine was already active in that build; its app code matched
   HEAD byte for byte).
2. **False health report.** Arduino `rmtWriteBlocking()` ignores the
   `rmt_write_items()` result and always returns `true`, so `led=rmt-ok
   led_failures=0` reflected no real check.

LED1 is powered from 3V3 (schematic nets `3V3 → LED1 VDD`, `IO21 → R15 →
LED1 DI`). A 3.3 V data level into a 5 V LED is therefore not the cause.
3.3 V is still below the WS2812B-2020 datasheet supply range, which reduces
timing margin.

## Fix: candidate `5576f902…`

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 5576f9027d569abe0d6d2f90595a8d4940328ca3ccc2f94f04991995ae39f749
TX app (tx_0): f17d9981d460c20d95e01991a2390cbebf261639ae76daa0e5c53e89f257e32f
RX app (rx_0): eb40be8614c0d5091967ece0a1821a86c6fed6fe2a460810549fe8fe75f27d80
TX ELF: 8581ebc3c172ebdbaf4b621bc6c2991ace91f44766cfc9ca8e895be738236656
RX ELF: fd10729d3ceefa488156f7d7bdde5e9076d748f9b78db61e82c8535627c47f3b
bootloader: bootloader_qio_80m.elf, SDK qio_qspi, app header dio
build inputs (path-relative sha256 of patches, overlay, partitions, targets,
apply_targets.py): ea3e9ec76de282ebb69d7652d35ffcbe3433d09819bc2e622d49598d7457aa90
```

The manifest and checksums are versioned in
`firmware/elrs-usb-tx-rev-a3/releases/5576f902/`. The 8 MB binary is not
committed; check the file the bench flashes against `SHA256SUMS`. To recompute
the build-input hash from a checkout:

```sh
cd firmware/elrs-usb-tx-rev-a3
find patches expresslrs-overlay partitions targets tools/apply_targets.py -type f -print0 \
  | sort -z | xargs -0 sha256sum | sha256sum
```

Status: superseded by `19c7b3ce…` (see below).

### Changes

- `expresslrs-overlay/src/aerialkit_ws2812.cpp`: one WS2812 driver for the
  boot identity, fault loops, the state-machine device and the self-test.
  It uses the IDF RMT driver on channel 3 directly, and checks the result of
  `rmt_config`, `rmt_driver_install`, `rmt_write_items` and
  `rmt_wait_tx_done`. It reads back the APB source and divider, and is
  serialised by a mutex. The Arduino `rmt*` wrapper is no longer used.
- Timing at 25 ns per tick: T0H 300 ns, T0L 950 ns, T1H 750 ns, T1L 600 ns
  (1.25/1.35 µs bits). Every frame ends with an explicit 300 µs low latch
  item, so frames never depend on the idle gap.
- `AK-BOOT` reports `led=rmt-ok|rmt-fail`, the real failure count and last
  `esp_err_t`, the numeric reset reason and the waveform constants.
- LED self-test: the CRSF frame `C8 08 7F <own addr> EA 'A' 'K' 'L' 'T' <crc>`
  (own address 0xEE on TX, 0xEC on RX) shows pure red, then green, then blue
  for one second each through the operational driver. It prints
  `AK-LEDTEST …` lines. The local endpoint consumes the frame, which is never
  forwarded over RF. `tools/usb_crsf_bench.py --led-selftest 0xEC` sends it.
- State and color decisions are unchanged.

### Hardware result for `5576f902…` (board 1)

- Flashed at `0x0`, `Hash of data verified`. Boot:
  `AK-BOOT role=rx slot=rx_0 gpio4=high reset=unknown(0) elf=fd10729d led=rmt-ok led_failures=0 led_error=0x0`.
- The garbled colors are gone. The LED is stable and the patterns are
  correct: RX shows the host-present/RF-disconnected double blink, and TX
  shows its correct pattern.
- Green and blue are swapped: RX shows blue and TX shows green. Red is
  correct. The self-test ran three times with `AK-LEDTEST done led=ok`.
- Opening the port with RTS cleared before DTR did not reboot the board,
  which confirms the reset-on-open mechanism below.

## Attempted channel order fix: candidate `19c7b3ce…`

The hypothesis was that LED1 clocks bytes in blue, red, green order rather
than the datasheet G-R-B order. The operational table was changed to B-R-G in
`aerialkit_led_state_machine.h`
(`AERIALKIT_WS2812_WIRE_FIRST/SECOND/THIRD` = blue, red, green). Role logic,
patterns, timing and the waveform are unchanged.

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 19c7b3ce0d7094ba1af049c6374cdc9afbc040ea737eb10d4617d1e0d554f05f
TX app (tx_0): 2b5e6e4db431ceba5b02cb91ad9603b38f38e63db53ccfe6e84eb48daaabebd3
RX app (rx_0): f40f11489ce167a0e2a72ffdeef40cf9f4d2cee945a4fe20e9c57dc2ddf1310d
TX ELF: 61326a1d476a66e36f11a8d87a111f9929b20327c5ab83897dbccf6853745792
RX ELF: d28b7656e11d82930046b1ee5c094e1df888595f45644009ed8884df32f14ac1
bootloader: bootloader_qio_80m.elf, SDK qio_qspi, app header dio
build inputs (path-relative): 50eb100a3d2c9f467878ab2cc56cc2bbfeaaf493c55229adbee528ec2d71d511
```

Records are in `firmware/elrs-usb-tx-rev-a3/releases/19c7b3ce/`. Host tests
cover the wire order: blue is byte 1, red byte 2 and green byte 3; TX blue is
`0x1A0000`, RX green is `0x00001A`, and the role sweep is single-channel.
14/14 firmware host tests pass, including the ELF check against this build.
The native suite passes 9/9.

This hypothesis did not survive hardware testing. RX still displayed blue, so
the candidate cannot be used for final state colors. The current diagnostic
keeps that operational table unchanged while testing all six orders through an
explicit temporary path.

## Native-USB findings

- The ESP32-S3 USB Serial/JTAG resets the chip while RTS is asserted and DTR
  is not. Linux asserts both on `open()`. pyserial then clears DTR before
  RTS, so `dtr=False, rts=False` before open still pulses reset (observed:
  each open rebooted the board). `tools/usb_crsf_bench.py` clears RTS first,
  then DTR. A change request is filed for `aerial_kit.comms.SerialTransport`.
- The HWCDC receive ISR always drains the USB OUT FIFO and drops bytes when
  its 256-byte queue is full; it never back-pressures the host. A host write
  that blocks after about 15 frames therefore means USB was not being
  serviced at all (reset, crash or disabled CDC interrupt), not slow parsing.
  The `AK-BOOT` reset reason distinguishes these. The probe reads
  concurrently and timestamps the first write stall.

## Tests

- `tests/native_led/test_led_output.cpp` (host g++, via
  `tests/test_led_output.py`) checks:
  - exact wire words (5576f902 used G-R-B; 19c7b3ce uses B-R-G: TX `0x1A0000`, RX `0x00001A`);
  - every item duration against the WS2812B-2020 limits;
  - the 25-item frame, with a latch longer than 280 µs and no early end
    marker;
  - the exact green bit sequence;
  - a 12 s sweep of every role pattern showing no non-role channel and the
    brightness cap;
  - pure red fault and fixed amber mismatch;
  - boot pulse counts (TX 1, RX 2);
  - bind 3 s and searching 1 s breathing, and steady connected;
  - self-test colors, timing and frame matching.
- `tests/test_led_output.py` also checks the patch and source wiring. When
  build outputs exist, it checks that the ELFs have no I2S, HSV, rainbow,
  fade or Arduino RMT-wrapper symbols.
- `tests/test_usb_crsf_bench.py` checks:
  - the valid ping `C8 04 28 00 EA 54`;
  - the 26-byte RC frame;
  - CRSF resynchronisation across ROM, AK-BOOT and garbage bytes, and
    bad-CRC recovery;
  - DEVICE_INFO decoding;
  - the self-test frame;
  - that the port-open sequence never presents RTS=1 with DTR=0.

Test results for `5576f902…`: 14/14 firmware host tests pass, including the
ELF symbol check against this build's ELFs. The native ExpressLRS suite
(`pio test -e native -f test_usb_transport`) passes 9/9.

`neopixelWrite` and `rmtWriteBlocking` are still linked, from the Arduino
core's `digitalWrite()`/`pinMode()` handling of the virtual pin `RGB_BUILTIN`
(97). No dual-role code uses that pin, and the source checks forbid direct use.

## Historical bench steps (`19c7b3ce…`)

1. (Use `19c7b3ce…`, not `5576f902…`.) Verify the file hash, then flash at `0x0` (USB/ROM) and confirm
   `Hash of data verified`.
2. Read `AK-BOOT`: check `led=rmt-ok led_failures=0 led_error=0x0`, role,
   slot and reset reason.
3. Run the self-test: `--led-selftest 0xEC` (RX) or `0xEE` (TX). Each of red,
   green and blue must be a pure color for one second. If not, the waveform
   is still wrong, so do not evaluate state patterns.
4. Only then check the role patterns: RX green-only, TX blue-only, bind vs
   searching breathing speed, steady when connected.

These steps are retained as history. Use the all-six-order diagnostic and
record its complete result matrix before changing the operational mapping.
