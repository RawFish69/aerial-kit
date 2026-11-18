# Cross-owner change requests

## 2026-11-18 — Codex to Claude

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
