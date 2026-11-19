# Review: overnight ELRS / CRSF radio work (Codex, 2026-11-17)

Reviewer: agent. Scope: commits `ce2c5ae`, `f8310cd`, `952b981` on `overnight-2026-11-17`
(`aerial_kit/comms/crsf.py`, `hw_bridge/crsf_*`, `sim_elrs_link_node.py`, `HOST-LINK.md`,
bench checklist). References used: ExpressLRS 4.1.0 source in
`~/Projects/elrs-usb-tx/ExpressLRS` (`src/include/crsf_protocol.h`,
`src/lib/Handset/CRSFHandset.cpp`) and the AK firmware in `firmware/src/core/flight/`.

Verdict: good structure and the simulated-radio plumbing works, but three defects would stop
the real link at the bench. Fix before any hardware step.

## Blocking

### B1. Telemetry decode is wrong for real devices (endianness and field order)
CRSF multi-byte telemetry fields are big-endian, and the attitude payload is
**pitch, roll, yaw** (`crsf_sensor_attitude_s` in ELRS; `attitude_frame()` in
`firmware/src/core/flight/ak_crsf_telemetry.c` writes the same). `crsf.py` encodes and
decodes ATTITUDE, BATTERY_SENSOR and GPS little-endian, and attitude as roll, pitch, yaw.
Its tests only round-trip its own encoder, so they pass either way.

Decoding real-format frames with the current code:

| Truth | Decoded |
|---|---|
| pitch 0.10, roll -0.50, yaw 1.20 rad | roll -0.614, pitch 3.096, yaw -0.815 |
| 16.8 V, 12.3 A, 1500 mAh | 4300.8 V, 3148.8 A, 14 419 200 mAh |
| 47.3977, 8.5440, 10 m/s, 90 deg, 450 m | -147.11, 1.19, 739.6 m/s, 102.75 deg, 42 525 m |

The decoded attitude and GPS feed `/uav/hw/imu` and `/uav/hw/gps`, i.e. the state estimator.
Fix: big-endian (`>`), attitude order pitch/roll/yaw; add tests with frames built
independently (byte literals taken from the ELRS struct layout or from the AK firmware's
encoder output), not from `crsf.py`'s own `make_*` functions.

### B2. Host baud rate 420000 is not a rate the ELRS TX module accepts from a handset
`CRSFHandset.cpp`: `TxToHandsetBauds[] = {400000, 115200, 5250000, 3750000, 1870000, 921600,
2250000}`; autobaud cycles only through these. 420000 is the receiver-to-FC rate, not the
handset-to-TX rate, and is ~5 % off 400000, outside UART tolerance, so the module would
never sync. `HOST-LINK.md`, `docs/HARDWARE.md`, the node default `serial_baud: 420000`
and the bench checklist (line ~122) all say 420000. Fix: 400000 default for the host
(optionally 921600/1870000 if the USB-UART adapter supports them); keep 420000 only for
the RX-to-FC UART (checklist line ~156 is correct).

### B3. The backend cannot arm a real flight controller
On the enable rising edge the very next frame has arm=high together with throttle from
`velocity_to_rc` (hover, 0.5). Betaflight refuses to arm unless throttle is low at the
arm transition. The AK firmware requires the arm switch on **with throttle below
`throttle_low` for `arm_hold_ms`** (`ak_flight.h`; its self-test holds 700 ms). Neither
would arm. The SIL could not catch this because `crsf_sim_receiver_node` treats the arm
channel as "enabled" directly.

Fix: an explicit arm sequence: on enable, hold arm=high and throttle at minimum for a
configurable `arm_hold_sec` (default > the FC's hold, e.g. 1.0 s), then release to the
controller; on disable, throttle minimum then arm low. Make the SIL receiver model the FC
arming rule (low throttle for the hold, otherwise stay disarmed) and test that arming
fails if throttle is high.

### B4. Arm and mode channels are swapped relative to the AK firmware
`firmware/src/core/flight/ak_types.h` / `ak_rc.c`: `AK_RC_MODE = 4`, `AK_RC_ARM = 5`
(0-based), i.e. **ch5 = mode, ch6 = arm** by default, thresholds 1300, counts 172/992/1811.
`crsf_radio.yaml` and `CrsfChannelMap` default to arm = ch5, mode = ch6. Against the AK
firmware the backend would toggle flight mode when it means to arm, and never arm. Make the
defaults match the AK firmware (and say in the checklist that a Betaflight setup needs its
AUX mapping set to the same channels).

## Should fix

- **S1. No serial reconnect** (the report claims one). `SerialTransport` has none; a USB
  unplug raises out of the timer and kills the node. The TX then stops receiving frames
  and the FC goes to failsafe, which is safe, but it should be deliberate: catch the
  error, report status, retry opening, never send stale frames after reconnect without a
  fresh disarmed/arm sequence.
- **S2. The SIL proves transport, not flight-controller semantics.** It decodes sticks back
  into a *velocity* command for the MuJoCo plant's velocity loop; a real FC in Angle mode
  maps sticks to *angles*. With `vxy_feedback`, the decoded command is the velocity
  *error* (`kv*ex/kv`), not the demand. Results (0 %/10 % loss, link cut) are valid for
  latency, loss and timeout handling only. Say so in the evidence, or decode sticks into
  `MultirotorManualCommand` in angle mode (the plant supports it) so stick semantics are
  exercised.
- **S3. Frame convention is hidden by yaw ~ 0.** `velocity_to_rc` takes body-frame demand
  (x right, y forward, the Gazebo convention); `sim_fast` sends world-frame twists and the
  SIL ran with heading control off, where the two coincide. Before hardware, state which
  frame `/uav/backend/cmd_twist` is in for this backend and rotate world -> body with the
  measured yaw if needed; add a test at yaw = 90 deg.
- **S4. `crsf_radio.yaml` turns `vxy_feedback` on**, while the docs keep GPS-derived
  feedback opt-in until verified on the vehicle. Default it off for hardware.

## Fine as is (checked)

- RC frame address 0xC8: ELRS's handset parser accepts 0xEE or 0xC8 (`CRSFHandset.cpp`).
- Full-duplex, non-inverted on separate UART0 pins: matches `halfDuplex = (TX pin == RX pin)`.
- Frame length/CRC (DVB-S2 over type+payload), 11-bit channel packing, link-statistics layout,
  parser resync on bad CRC.
- Disabled state sends arm low and minimum throttle; command timeout holds neutral hover.

## Re-review of fixes (commits 40eb014, 96cf796), agent review, 2026-11-17

Verdict: **B1-B4 and S1-S4 accepted.** hw_bridge 38 passed, test_crsf + test_fc_sil 23 passed (ROS_DOMAIN_ID 88).

Independent check against the real firmware: `BackendArmSequencer` -> `CrsfChannelMap.encode` ->
`pack_channels`/`unpack_channels` (the wire format) -> the unmodified AK flight core
(`sim_py/fc_sil`), at 1 kHz with a level IMU:

| Sequencer config | Throttle count during hold | Firmware result |
|---|---|---|
| node config (`arm_throttle: 0.0`), enable at 0.5 s | 172 | **armed at 1.0 s** (0.5 s FW hold) |
| dataclass default `throttle_min=0.05` | 254 | never arms |

The shipped node passes `arm_throttle` (0.0), so the real path arms. Telemetry field layouts
match ELRS `crsf_protocol.h` (big-endian; attitude pitch, roll, yaw; GPS `>iiHHHB`).

Nits (non-blocking):
- N1. `BackendArmSequencer.throttle_min` defaults to 0.05. That encodes to 254 counts, i.e.
  0.05003 normalised, above the firmware's `throttle > throttle_low (0.05)` refusal. Default
  it to 0.0 so that a bare `BackendArmSequencer()` arms a real FC.
- N2. `ReceiverArmingGate` compares the float 0.05 against 0.05 before any count
  quantisation, which is why N1 was invisible to the SIL. Quantise through counts like the
  FC does, or add the end-to-end test above (encode -> counts -> `sim_py.fc_sil.AkFlightCore`
  -> assert armed) to `hw_bridge` tests, skipped when no C compiler is available.
