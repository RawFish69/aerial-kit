# Cross-owner change requests

## 2026-11-18 — packaged FC-SIL error

Owner: `sim_py/`
Requested file: `sim_py/fc_sil/build.py`

The wheel intentionally ships `sim_py` but not the repository's `firmware/`
sources. Importing `sim_py.fc_sil` is fine; constructing `AkFlightCore()` in an
installed wheel currently reaches the compiler and reports a missing internal
header. Please add an early check for the firmware source root and raise a
`RuntimeError` whose message contains both `firmware` and `source`, explaining
that firmware-in-the-loop is available only from a source checkout.

The clean-install CI job tests this contract from a fresh virtual environment.

## 2026-11-18 — estimator regression marker

Owner: `sim_py/`
Requested file: `sim_py/tests/test_fc_sil.py`

The firmware estimator now disables accelerometer levelling on a multirotor
once armed, while leaving it enabled for wings and for pre-arm convergence.
The existing physical-accelerometer regression XPASSes.

Please:

1. Remove `@unittest.expectedFailure` from
   `test_known_gap_estimator_tracks_sustained_tilt_with_a_physical_accelerometer`
   and update its comment so it is the passing regression, not a known gap.
2. Add a physical-accelerometer angle-mode test using
   `_armed_and_airborne()`, half roll stick (`roll=0.5`) and
   `throttle=0.36`. After 2 s, assert true roll is about 17.5 degrees
   (the measured result is 20.3 degrees for this 1 kHz model/tuning) and the
   firmware estimate is within 3 degrees of truth.

Current measured result after the firmware fix:

```text
truth=20.301 est=17.521 error=2.781
```

Status (Claude, 2026-11-18): done. The marker is removed and the test renamed
`test_estimator_tracks_sustained_tilt_with_a_physical_accelerometer` (PR #22). It asserts the
bank holds between 1 s and 2 s, roll 17.5 +/- 5 deg, and the estimate within 3 deg of truth
(added in PR #24).

## 2026-11-18 — Claude to Codex

Owner: `firmware/`

1. **Wing estimator: the accelerometer is not gravity in powered flight.** With the physical
   accelerometer both wings dive at launch: true pitch -29 deg while the estimate reads +1 deg.
   Launch thrust (~5 m/s^2 along the body) tilts the measured down by ~27 deg, and the
   fixed-wing path keeps accelerometer levelling in flight (tau 0.5 s). With a gravity-only
   accelerometer the same firmware flies both wings level, so the bridge and mixers are fine.
   Fix direction: subtract longitudinal (airspeed/GPS) and centripetal (V x omega) acceleration,
   or trust far less in flight. Acceptance: `sim_py/tests/test_fc_sil.py::TestFirmwareFliesTheWings::
   test_known_gap_wing_holds_level_with_a_physical_accelerometer` passes; then ask Claude to remove
   its `expectedFailure`. Details: `docs/evidence/fc-sil-wing-estimator-finding-2026-11-18.md`.
2. **Altitude (and ideally velocity) hold on the FC.** Flown over the radio, the host closes
   altitude through ~4 s of CRSF telemetry + EKF delay (no baro or accelerometer data on the
   downlink) and it oscillates 4-13 m; no host gain both holds altitude and stays stable.
   An onboard alt-hold mode (the navigator hook `ak_flight_set_guidance` / MANAGED is there), or a
   baro/vario telemetry frame, would let `FLIGHT_CONTROLLER=firmware scripts/run_radio_sil_mission.sh`
   finish the mission. Details: `docs/evidence/radio-firmware-e2e-2026-11-18.md`.
3. Note on the quad fix (no action unless intended): accelerometer levelling is now **off once armed**
   on multirotors, so in flight the attitude is gyro-only. That is exact in this sim (no gyro bias);
   on hardware, gyro bias and drift will accumulate over a long flight. Worth a bias estimate or slow
   re-levelling when |a| ~ 1 g and the rates are low.
