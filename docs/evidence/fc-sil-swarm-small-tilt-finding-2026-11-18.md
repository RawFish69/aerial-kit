# Finding: below the 7 deg gate the quad estimator still under-reads tilt

Found by: agent, firmware swarm (`sim_py.fc_sil.swarm.FirmwareSwarm`, unmodified firmware at
`main` with PR #26), 2026-11-18. Owner: Codex (firmware). Simulation only.

The quad fix (PR #17) stops accelerometer levelling once armed and tilted beyond 7 deg. Below 7
deg it still levels on the accelerometer, and while the quad accelerates that reads thrust
direction, not gravity. Gentle flight lives entirely below 7 deg: swarm formations, the radio
mission, any autonomous cruise.

One `swarm_quad`, angle mode, a 1 m/s velocity step through the companion law:

| t | true pitch | firmware estimate | velocity |
|---|---|---|---|
| 0.4 s | 5.8 deg | 2.3 deg | 0.22 |
| 0.8 s | 7.7 deg | 1.6 deg | 0.66 |
| 1.6 s | 3.4 deg | 0.2 deg | 1.30 (overshoot) |

Commanded about 3.9 deg (stick 0.11 x 35 deg). The firmware flies about twice the angle it is asked
for, so the outer velocity and position loops overshoot and ring.

| | velocity step peak | 4-drone circle error after 10 s |
|---|---|---|
| gravity-only accelerometer | 0.76 m/s (no overshoot) | 0.22 m |
| physical accelerometer | 1.32 m/s | 3.27 m |

Probably also behind the horizontal zig-zag left in the radio mission
(`radio-firmware-e2e-2026-11-18.md`).

Fix direction: the same as above 7 deg, applied at all tilts once armed (gyro-carried attitude, as
the wing now does), plus slow re-levelling only when |a| ~ 1 g, the rates are low and the
commanded acceleration is small; or subtract the thrust model.
Acceptance: `sim_py/tests/test_fc_sil_swarm.py::test_known_gap_formation_with_a_physical_accelerometer`
passes (then the sim agent removes its `expectedFailure`), and `test_fc_sil.py` stays green.
