# Radio review remediation — 2026-11-17

Scope: findings in `agent-review-radio-2026-11-17.md`. All findings were
confirmed against the pinned ELRS 4.1.0 and AerialKit firmware sources. None
was disputed.

## Verified source of truth

- ExpressLRS `src/include/crsf_protocol.h` lines 273-280 define the battery
  payload, lines 352-361 define GPS, and lines 363-369 define attitude as
  `pitch, roll, yaw`, all with big-endian multi-byte fields.
- ExpressLRS `src/lib/Handset/CRSFHandset.cpp` line 42 defines the handset baud
  table with `400000` first. The receiver-to-FC rate is a separate link.
- AerialKit `src/core/flight/ak_crsf_telemetry.c` lines 129-183 writes the
  attitude, battery and GPS fields with the same byte order.
- AerialKit `src/core/flight/ak_flight.h` lines 165-173 define
  `throttle_low`, `arm_hold_ms` and `rc_timeout_ms`; `ak_flight.c` lines
  1682-1690 implement the throttle-low held arm transition.

## Findings

### B1 — telemetry format: fixed

ATTITUDE, BATTERY_SENSOR and GPS now use big-endian fields. ATTITUDE is encoded
and decoded as pitch, roll, yaw.

Tests use wire literals rather than `crsf.py` encoders:

- AK firmware encoder output:
  `c8081e03e8ec782ee084` for pitch 0.10, roll -0.50, yaw 1.20.
- Independent ELRS layout for battery:
  `ea0a0800a8007b0005dc58d1`.
- Independent ELRS layout for GPS:
  `ea11021c4050a80517b6000168232805aa0cbd`.

### B2 — host baud: fixed

Host/handset transport now defaults to 400000 baud in the codec, ROS node and
YAML. `HOST-LINK.md`, hardware/software documentation and the host-UART bench
command say 400000. Only the receiver-to-FC UART remains 420000.

### B3 — arming sequence: fixed

The backend holds arm high with throttle at CRSF minimum 172 for
`arm_hold_sec` (1.0 s default), then hands over controller sticks. Disable
immediately sends arm low and minimum throttle. The simulated receiver models
the FC gate: arm high with throttle at or below `throttle_low` must persist for
its own `arm_hold_sec` (0.5 s default). Tests prove high throttle does not arm
and low throttle does after the hold.

An intermediate run exposed a quantization edge at normalized 0.05; the final
`arm_throttle` is the actual CRSF low endpoint, so the FC does not see 0.05003
from a 254-count channel value.

### S1 — serial reconnect: fixed

The node catches serial open/read/write failures, reports them on
`/uav/radio/status`, closes and retries the port, resets its parser, and resets
the arm sequence. Tests verify reconnect reopens at 400000 and puts the arm
state back into the disarmed hold.

### S2 — simulated stick semantics: fixed with a stated limit

The receiver publishes `uav_msgs/MultirotorManualCommand` on
`/uav/radio/decoded_manual`. The SIL launch exercises the plant's VELOCITY manual
mode with world-to-body stick conversion.

Limit: this does not prove a real FC's ANGLE-mode response, nor the tuning of
the hardware-default angle stick map. Those remain props-off bench items.

### S3 — command frame: fixed

`command_frame` is an explicit `world|body` setting, defaulting to `world` in
the current bringup. World ENU demand is rotated into controller right/forward
axes using measured yaw. A 90-degree heading test verifies north demand becomes
forward stick, not right stick.

### S4 — hardware velocity feedback: fixed

`crsf_radio.yaml` sets `vxy_feedback: false`. The SIL uses manual velocity sticks
and therefore does not depend on that hardware feedback path.

### B4 — arm/mode channel order: fixed

A concurrent review update correctly found that AerialKit's zero-based
`AK_RC_MODE=4` and `AK_RC_ARM=5` make wire channels 5 and 6 the mode and arm
switches respectively. Defaults now use mode CH5 and arm CH6, a 1300-count
threshold, and channel 5 high for Angle mode. The bench checklist states that a
Betaflight AUX map must be changed to the same assignment.

## End-to-end SIL results

All runs used `ROS_DOMAIN_ID=61`, MuJoCo, 150 Hz and 8 ms configured one-way
latency.

| Run | Result | Closest goal | Max altitude | Samples |
| --- | --- | ---: | ---: | ---: |
| FPV 0% loss | `Demo: disarm` | 0.41 m | 4.48 m | 6,241 |
| FPV 10% loss | `Demo: disarm` | 0.04 m | 4.65 m | 7,699 |
| FPV mid-flight cut | receiver disabled, landed | n/a | 3.11 m after cut | 6,175 |

The link cut was published at 2.02 m. The receiver disabled and the plant
returned to 0.38 m with enable false.

Evidence:

- `radio-sil-fpv-0pct-final-20261117-164925.json`
- `radio-sil-fpv-10pct-final-20261117-165305.json`
- `radio-sil-fpv-link-loss-final-20261117-165728.json`

Every run was followed by a process check showing no radio SIL launch or node
remaining.

## Final test gate

Run with `ROS_DOMAIN_ID=61` on the combined worktree:

- Python: **541 passed, 1 expected failure, 89 subtests passed**.
- ROS: **216 passed** (three existing non-failing warnings).

The extra Python expected-failure case belongs to the concurrent
firmware-in-the-loop work, not to this radio change set.
