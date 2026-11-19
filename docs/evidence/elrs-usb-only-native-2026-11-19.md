# Rev A3 native-USB-only ELRS bench — 2026-11-19

The bench has no J3 USB-UART adapter and Wi-Fi is unavailable. The Rev A3
firmware now carries CRSF over the native ESP32-S3 USB-CDC endpoint in both
roles while retaining the UART path.

## Behavior

- TX host to board: CRSF `RC_CHANNELS_PACKED` frames are parsed into the normal
  ExpressLRS TX module endpoint and start the handset scheduling path. A
  500 ms absence of host RC frames returns the TX to no-Crossfire state.
- TX board to host: RF telemetry, link statistics and device messages continue
  through the existing CRSF router and native USB connector.
- RX board to host: the second `SerialCRSF` endpoint writes
  `RC_CHANNELS_PACKED` frames to native USB.
- RX host to board: telemetry frames are parsed by that endpoint and forwarded
  over RF through the normal receiver telemetry path.
- Logical rates are 400000 baud for TX-host and 420000 baud for RX-FC. USB CDC
  ignores baud requests; the nominal values identify the CRSF endpoint.

## LED diagnostics

Before target options are loaded, both FEM enables are driven low and the
WS2812 emits a short role heartbeat over RMT:

- TX: one blue pulse.
- RX: two green pulses.

The existing disconnected, bind and connected patterns follow. The global
WS2812 brightness remains 26/255 (about 10%).

A role mismatch or missing hardware layout holds both FEM enables low and
enters a slow red fault pattern. Hardware-layout failure also prints
`AK-FAULT hardware layout not loaded` periodically on native USB.

## Bootloader correction

The first factory image paired a DIO bootloader with applications linked
against the QIO SDK. It produced invalid flash reads, LittleFS failures and an
RX panic loop. The assembler now derives the SDK memory type from each
application map, selects `bootloader_qio_80m.elf`, derives all selector symbols
from that ELF, and requires the unpatched generated bootloader to match
PlatformIO's `bootloader.bin` byte-for-byte.

## Validation

- ExpressLRS 4.1.0 TX build completed with the native USB bridge.
- ExpressLRS 4.1.0 RX build completed with the native USB bridge.
- Native USB/CRSF tests: 9 passed (RC input, telemetry output routing,
  telemetry input, device information, heartbeat/role logic, LED priorities,
  color locking, pulse timing and serial guards).
- Role/selector/factory-image tests: 8 passed, including independent QIO
  selection, a deliberate bootloader-mode mismatch, CRSF framing and appended
  target trailers.
- Factory artifact: 8,331,264 bytes.
- SHA-256:
  `c1668c40f87a94452bafbaca281b11799a6e2a187a6553e04866b3eba80524d4`.
- No physical board was flashed or powered during this code change.

The earlier `7210…` image is invalid and must not be flashed. Hardware
acceptance remains gated on the `c166…` image and the USB-only bench checklist.
