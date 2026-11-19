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

The WS2812 initializes a short role heartbeat before the normal ExpressLRS
state machine:

- TX: one green pulse.
- RX: two blue pulses.

The existing disconnected, bind and connected patterns follow. The global
WS2812 brightness remains 26/255 (about 10%).

## Validation

- ExpressLRS 4.1.0 TX build completed with the native USB bridge.
- ExpressLRS 4.1.0 RX build completed with the native USB bridge.
- Native USB transport tests: 3 passed (RC input, telemetry output routing,
  telemetry input).
- Role/factory-image tests: 5 passed, including CRSF channel framing.
- Factory artifact: 8,331,264 bytes.
- SHA-256:
  `7210b9ff0600c2458e3fc3f429c0bbb6a9d282f1ddbe5ad4bd9c0bcb23a6cca7`.
- No physical board was flashed or powered during this code change.

The hardware run is gated on the replacement artifact hash and the USB-only
bench checklist.
