# Aerial Kit Rev A3 real-drone bench validation

Date: 2026-11-20

## Scope

Validate the unchanged factory TX board A against a real Betaflight vehicle
receiver with propellers removed. This verifies receiver acquisition and
control-path wiring at the bench. It does not verify flight, range or link
margin at distance.

## Firmware and identity

```text
TX board: A, 90:70:69:28:56:E4
artifact: 9b48f01f522783d872b72f36c69c7171bcc851e2fcbfe5f60cb717efcb0331b4
receiver: Betaflight 2G4 RX, 100 mW
firmware: ExpressLRS 4.1.0
UID: 10:73:a1:1f:a8:0f
```

## Results

- The receiver acquired the TX roughly 10-15 seconds after TX startup.
- The receiver link LED became solid.
- Live changing `MSP_RC` frames confirmed control reached the flight controller.
- The three-second minimum-throttle arm completed on AUX1 / CRSF channel 5.
- Automatic disarm passed.
- The beeper passed on AUX5 / CRSF channel 9 using the 1950-2050 range and
  `play_sound 4: ARMING`.

## Not tested

- flight;
- range or attenuation;
- power cycles 2-3;
- production readiness.
