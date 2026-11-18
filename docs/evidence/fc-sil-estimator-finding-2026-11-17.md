# Finding: AK estimator under-reads sustained tilt on a multirotor (angle mode over-banks)

Found by: Claude, firmware-in-the-loop (`sim_py/fc_sil`, unmodified `firmware/src/core`), 2026-11-17.
Owner: Codex (firmware). Simulation only; no hardware claim either way.

## What happens
`ak_estimator_update()` corrects attitude toward the accelerometer with
`tau = 0.5 s` (`ak_flight.c: ak_estimator_init(&flight->est, 0.5f)`) whenever
`0.7 g < |a| < 1.3 g` (`AK_ACCEL_TRUST_LOW/HIGH`). On a multirotor the
accelerometer measures the specific force, which in steady flight is mostly
**thrust, along body z**, not gravity. While banked and holding altitude,
`|a| = 1/cos(bank)` is 1.25 g at 37 deg, inside the gate. The correction therefore
pulls the estimate back toward level. Angle mode then sees "not banked enough" and
keeps rolling.

## Numbers (FPV 5" quad, angle mode, roll stick 0.5 held, throttle ~hover)

| t after stick | true roll | firmware `est.roll` |
|---|---|---|
| 0.5 s | 23.3 deg | 12.7 deg |
| 1.0 s | 37.5 deg | 12.9 deg |
| 1.5 s | 51.0 deg | 13.2 deg |
| 2.0 s | 63.6 deg | 13.5 deg |

Control experiment: the same run with an accelerometer that reads *gravity only*
(not physical) gives estimate = truth = 17.5 deg, holding steady. So the control law,
mixer and signs are right, and the estimator's accelerometer trust is the cause.
Adding body drag to the sim (0.08 -> 0.6 N/(m/s)) shrinks the error only slightly
(estimate reaches ~23 deg at t = 3 s). Real rotor drag helps somewhat more than the sim
shows, but it does not remove the effect.

In the keyboard session (`fc-sil-teleop-fpv-session.json`), holding the roll key in
angle mode reached 67 deg (configured limit is about 35 deg), and holding pitch built
up to 17 m/s, then flew into the ground.

## Suggested fix directions (firmware owner's call)
- Gate accelerometer correction much tighter for multirotors (e.g. `| |a| - 1 g | < 0.05`)
  and/or only when body rates are low; or
- a much longer time constant for the quad airframe (Betaflight-style: seconds, not 0.5 s); or
- remove the expected thrust term (`a_corrected = a - T/m * z_body`) when a thrust estimate exists.

## Regression test waiting for the fix
`sim_py/tests/test_fc_sil.py::test_known_gap_estimator_tracks_sustained_tilt_with_a_physical_accelerometer`
is marked `expectedFailure`. When the estimator is fixed it will start passing
(pytest then reports XPASS); remove the marker. The library rebuilds automatically
when any firmware source changes (content hash).
