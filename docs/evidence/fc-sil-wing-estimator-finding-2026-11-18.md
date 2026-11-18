# Finding: the AK estimator misreads pitch on a powered flying wing (angle mode dives it in)

Found by: Claude, firmware-in-the-loop (`sim_py.fc_sil.WingFirmwareInTheLoop`, unmodified
`firmware/src/core` at `main` after the quad estimator fix), 2026-11-18. Owner: Codex (firmware).
Simulation only.

## Setup
Twin wing (firmware airframe 1, `ak_mixer_elevon_wing`) and single wing (airframe 7,
`ak_mixer_elevon_wing_single`) in MuJoCo. Armed in the hand (still, level IMU), throttle 0.6,
thrown level at 15 m/s from 60 m, angle mode, sticks centred. Motor u -> thrust T_max u^2,
servo -1..1 -> +/-25 deg elevon, trailing edge up positive.

## Result
| | t = 1.5 s | 3 s | 4.5 s | 6 s |
|---|---|---|---|---|
| twin, physical accelerometer: true pitch / firmware pitch | -19.1 / +2.6 | -28.9 / +0.6 | -26.7 / -1.6 | hits the ground |
| single, physical: true / firmware | -15.2 / +2.0 | -23.6 / +0.2 | -18.1 / -0.8 | -9.4 / -0.4, z 20 m |
| twin, gravity-only accelerometer | -0.1 / -0.1 | +0.5 / +0.5 | +0.4 / +0.4 | +0.3 / +0.3, z 61 m |
| single, gravity-only | -1.1 / -1.1 | +0.2 / +0.2 | +0.4 / +0.4 | +0.3 / +0.3, z 59 m |

With an accelerometer that reads gravity only, the firmware flies both wings level, with small
elevon trims. So the bridge, mixer rows and signs are right. With the physical accelerometer
it believes it is level while diving at -20 to -29 deg.

## Why
`ak_flight.c` keeps accelerometer correction in flight for fixed wings at tau 0.5 s ("A fixed
wing keeps the accelerometer correction in flight"). A powered wing's accelerometer reads
lift + thrust - drag, not gravity. Just after launch about 7 N of thrust on about 1 kg gives
about 5 m/s^2 along the body, tilting the measured "down" about 27 deg nose-up. The estimate
follows it within half a second, angle mode levels that false horizon, and the nose goes down.
As it dives and accelerates the error persists. The same happens in any sustained acceleration
or turn (centripetal).

## Suggested fix directions (firmware owner's call)
- Subtract the known accelerations before using the accelerometer as "down": longitudinal from
  airspeed or GPS ground-speed rate, centripetal from `V x omega` (ArduPlane/INAV do this).
- And/or trust it far less in flight: a long tau, and only when |a| is near 1 g and the rates are low.
- Re-run `sim_py/tests/test_fc_sil.py::TestFirmwareFliesTheWings`. The
  `test_known_gap_wing_holds_level_with_a_physical_accelerometer` test is `expectedFailure`;
  remove the marker when it passes.

## Also verified on the wings (tests pass)
- Arming in the hand; it refuses while the switch is held from power-up. In the air it will not
  start the arm hold, which is correct.
- Stick signs: roll right gives right wing down; pitch stick forward gives nose down (RC convention).
- Lost link: state "circling down" with a 30 deg bank, power kept on (all of it on one motor of the
  twin, its yaw), about 4 m/s descent, about 15 deg/s turn.
