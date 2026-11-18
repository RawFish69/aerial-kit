# Direct ELRS / CRSF bench checklist — 2026-11-17

This is the ordered bench handoff for the Aerial Kit ELRS USB TX Rev A3 and
the direct CRSF ROS backend. Nothing in this checklist was performed during
the overnight run: no serial port, radio or flight controller was opened, and
no RF was transmitted. Completing it would establish a bench-tested link, not
flight readiness.

## Stop conditions

- Remove all propellers. Keep the aircraft restrained until electrical and
  failsafe checks are complete.
- Fit a suitable 2.4 GHz antenna or a 50 ohm terminated RF load before powering
  any TX or RX image. The diagnostic image may run on an open connector because
  it keeps both FEM enables low and never commands the SX1280 to transmit.
- Start at the lowest configured RF power. Increase it only after the conducted
  or terminated path has been checked.
- Stop if the role switch does not select the image and hardware role you
  intend to test.

## 1. Confirm the role switch by continuity

1. Power the board down and disconnect USB and the host adapter.
2. Check continuity from GPIO4 to ground and to 3.3 V in both switch positions.
   Record which position is electrically low and high.
3. Match low to the TX image/design role and high to the RX image/design role.
   Do not trust the printed `TX`/`RX` label: the 2026-11-17 diagnostic read low
   while the prototype label indicated RX.
4. Label the verified positions before applying power.

Expected result: exactly one role and one image agree in each switch position.

## 2. Verify the supplied images

From `firmware/elrs-usb-tx-rev-a3/`:

```bash
sha256sum -c dist/SHA256SUMS
```

Expected result: all three recorded images verify. Rebuild and repeat rather
than flashing a checksum failure.

## 3. Run the non-RF diagnostic

With the RF connector open and no TX/RX image active:

```bash
./scripts/flash.sh diagnostic /dev/ttyACM0
```

Open native USB serial and copy the diagnostic report. Expected minimum:

```text
DIAG usb=ok uart0_tx=43 uart0_rx=44
DIAG sx1280 spi=ok version=0xA9B7 busy=0 dio1=0
DIAG fem txen=0 rxen=0 final=shutdown
```

Record the GPIO4 role reading, FEM readback, strap values and any remote I2C
devices. Do not proceed if the SX1280 SPI check fails or either FEM enable
cannot be returned low.

## 4. Select and flash each role

For the transmitter board:

1. Attach an antenna or 50 ohm load.
2. Set the verified low GPIO4 position.
3. Flash the transmitter image:

   ```bash
   ./scripts/flash.sh tx /dev/ttyACM0
   ```

For the receiver board:

1. Attach an antenna or 50 ohm load.
2. Set the verified high GPIO4 position.
3. Flash the receiver image:

   ```bash
   ./scripts/flash.sh rx /dev/ttyACM0
   ```

Expected result: the diagnostic image is replaced by the matching ExpressLRS
application, and native USB can still be used for configuration.

## 5. Configure and bind

Open each board's ExpressLRS configuration page over native USB. Configure both
boards with:

- the same binding phrase;
- the same regulatory domain, packet rate (initially 150 Hz) and telemetry
  ratio;
- the lowest available transmitter power for the first powered RF test.

Power the receiver first, then the transmitter, with a load on both RF ports.
Expected result: the receiver acquires the transmitter and reports link
statistics. A failure to bind is not diagnosed by raising power; check role,
domain, binding phrase and image first.

## 6. Test the host UART without a flight controller

Use a 3.3 V USB-UART adapter:

| Host | J3 |
| --- | --- |
| Adapter TX | J3.4 / GPIO44 |
| Adapter RX | J3.3 / GPIO43 |
| Ground | J3.2 / GND |

Do not connect J3.1 or J3.7 while the board is USB-powered. Run the backend only
after confirming the adapter is 3.3 V:

```bash
source ros2_ws/install/setup.bash
ros2 run hw_bridge crsf_radio_backend_node --ros-args \
  -p transport:=serial \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baud:=400000 \
  -p airframe_name:=fpv_quad_5in
```

Keep `/uav/backend/enable` false and inspect the bytes on the UART or a protocol
analyzer. Expected result: valid 26-byte `RC_CHANNELS_PACKED` frames at the
configured rate, disarmed channel 6, minimum channel 3 throttle, and no serial
open/write errors. `/uav/radio/status` should report a stale telemetry link when
no receiver telemetry is connected.

Then drive an enable rising edge through the command stack. Expected result:
channel 6 (arm) goes high while channel 5 (mode) is high and channel 3 remains
at 172 for `arm_hold_sec` (1.0 s by default), followed by normal AETR sticks.
Force throttle above `throttle_low` before the hold completes and confirm the
FC/receiver does not arm. Disable must drive channel 6 and throttle low
immediately.

Unplug the USB-UART adapter while frames are running. Expected result:
`/uav/radio/status` reports the disconnect, the node retries without exiting, and
a reconnect begins another full arm hold at low throttle. It must not resume an
armed handover from the previous connection.

## 7. Restore the firmware HIL harness before FC-in-the-loop

This stage is currently blocked. The public firmware tree builds only the ARM
Feather F405 image; the Makefile has no host/SIL target. The historical notes
refer to these files and targets, but they are not present in this checkout:

- the host peripheral tests and models under `firmware/tests/` (`tests/` exists
  in this public tree, but only fixture data is retained);
- `firmware/tools/ak_control.h` and the shared control library it describes
  (`tools/akcontrol.py` is present, but cannot load the missing library);
- `make control-check`, `make control-experiment` and `make sil`;
- the `aerialkit-fw-sim` executable;
- the MuJoCo `ActuatorPlant`/IMU adapter used to exchange motor and sensor data
  with the firmware control core.

Do not invent a replacement in this checklist. Restore the retained development
tree or implement and test those host targets first. Then the intended HIL test
is: firmware mixer outputs drive the simulated motors, simulated IMU feeds the
firmware, start with hover, then small stick steps, and compare the plant's
attitude/position with a serial transcript. This overnight run did not perform
that test.

## 8. FC bench test with props off

1. Configure the FC receiver UART for CRSF at 420000 baud, 8N1, non-inverted.
2. Confirm AETR on channels 1-4, mode on channel 5 and arm on channel 6.
   AerialKit uses a 1300-count threshold. A Betaflight setup needs its AUX
   mapping changed to this order.
3. Connect the receiver's CRSF output to the FC and keep the FC outputs
   disconnected from ESCs if the board permits it; otherwise keep props removed
   and the aircraft restrained.
4. Arm and disarm repeatedly. Expected result: the FC changes armed state only
   while channel 6 requests arm and its own arming gates pass.
5. Move roll, pitch, yaw and throttle one at a time. Expected result: only the
   corresponding output or commanded attitude changes, with the documented
   sign. Confirm mode changes on channel 5. At yaw 90 degrees (nose north),
   command a world-north velocity demand and confirm it appears as
   body-forward motion, not body-right.
6. Compare handset/FC telemetry with independent instruments. Pitch must not be
   swapped with roll; battery volts/current and GPS latitude/longitude/speed
   must match their real magnitudes rather than byte-swapped nonsense.
7. Configure the FC failsafe before testing it. It must not hold the last command.
   For the AerialKit firmware, a quadrotor stops its motors and latches failsafe;
   a fixed wing circles down as documented in `docs/04-flight-core.md`. For
   another FC, use its controlled-land or motor-stop policy and record the exact
   settings.
8. While armed with outputs at a safe low level, remove transmitter RF or
   disconnect the CRSF signal. Expected result at the FC: a stale receiver/link
   indication followed by the configured failsafe action. The aircraft must not
   re-arm when the link returns; cycle the arm switch through off and on.
9. Repeat the link-loss test from a fresh arm cycle. Copy the FC console, UART
   counters, failsafe state and output/ESC signal evidence into a dated record.

Stop after the props-off FC and failsafe checks. This checklist does not
authorize a flight or claim that the link, tuning or airframe is flight-ready.
