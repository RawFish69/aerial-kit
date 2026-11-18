# Radio end to end with the real AK firmware (Claude, 2026-11-18)

Simulation only. Path under test, every stage real code except the link and the plant:

    mission executor -> crsf_radio_backend_node (arm sequence, sticks, CRSF framing)
      -> sim_elrs_link_node (150 Hz, 8 ms, loss %) -> receiver bytes
      -> multirotor adapter, flight_controller:=firmware, firmware_rc_input:=crsf
         (CRSF parser -> unmodified AK flight core at 1 kHz, frames delivered at their arrival ms)
      -> MuJoCo fpv_quad_5in;  telemetry back over CRSF -> hw_state_estimator (EKF) -> host

Run: `FLIGHT_CONTROLLER=firmware scripts/run_radio_sil_mission.sh fpv_quad_5in <loss%>`,
or `ros2 launch sim_fast bringup.launch.py sim_backend:=mujoco radio_sim:=true flight_controller:=firmware start_demo:=true`.

## What works (tested)
`ros2_ws/src/hw_bridge/test/test_radio_firmware_e2e.py` (no ROS; same code path):
- Arming through the wire: host holds arm high + 172 counts for 1.0 s; the firmware arms after its 0.5 s hold.
- Take-off and velocity hold: 2 m/s climb -> 1.96 m/s; 3 m/s forward -> 2.9 m/s at ~2 deg tilt, with 10 % frame loss.
- Link cut: the firmware fails safe on its own receiver timeout (motors 0, latched); restored link stays latched.
- Disable -> firmware disarms.
- Stable with a 1 s lag on the host's velocity estimate; the old gains flip the quad there (regression test both ways).

ROS mission: arms, takes off, flies, never flips or runs away: max tilt 28.9 deg at 0 % loss, 31.5 deg at 10 % loss
(`radio-sil-fpv-firmware-loss{0.0,10.0}-20261118-*.{json,log}`).

## What does not work yet: completing the autonomous mission
The bringup demo mission (~200 m) does **not** reach the goal with the firmware flying. The quad oscillates
(position +/- 6 m, altitude 3-10 m, ~9 s period) and the 240 s mission timeout ends it 180 m short.
`flight_controller:=sim` completes the same mission (Codex's runs: closest approach 0.41 m).

Causes, measured:
1. **The host closes the velocity and altitude loops over telemetry.** The AK firmware has no velocity or
   altitude mode, so `crsf_radio_backend_node` turns velocity demands into angle sticks and throttle with
   feedback from `/uav/backend/odom`. That odometry comes from CRSF attitude + GPS + baro through the EKF,
   with no accelerometer, and it trails the truth by about 1-2 s (e.g. truth +6.2 m/s, estimate +1.4 m/s).
   With a 2 s lag no host gain is both stable and holds altitude (sweep in the transcript: high gains
   oscillate, low gains sag into the ground).
2. **The firmware estimator gap** (`fc-sil-estimator-finding-2026-11-17.md`) biases tilt: with an ideal
   accelerometer the same loop settles at 0.76 m/s for a 1.1 m/s demand, with the physical one at 1.75 m/s.

Gains were re-derived from thrust physics for firmware mode (`sim_py.fc_sil.radio_stick_settings`):
`kz = hover/(2g) * 2/s`, `kv_xy = 0.4/s / g`, throttle 0.3x..2x hover thrust, tilt-compensated hover,
the FC's own 35 deg tilt limit. The simulator-mode gains (kz 0.2, kv_xy 0.5) first climbed to 1.5 km,
then flipped the quad.

## Recommendation (firmware owner)
Autonomous flight over this link needs **altitude and velocity hold on the FC**, where the IMU is: the
AK core already has a navigator hook (`ak_flight_set_guidance`, `AK_FLIGHT_MANAGED`). Then the host sends
velocity setpoints (or an alt-hold throttle) instead of closing loops through a 1-2 s telemetry lag.
Alternatively add accelerometer data to the downlink. Fix the estimator first; re-run this mission after.

## Update: after the firmware estimator fix (PR #17), same day
The estimator gap is fixed (`test_fc_sil` known-gap test now passes and is a regression test:
true 20.3 deg vs estimate 17.5 deg, holding). Gains re-tuned on the fixed firmware: speed 0.2 /s,
climb 4 /s (`radio_stick_settings` defaults). Bench loop (no ROS): tilt <= 16 deg, altitude within
+/-0.7 m with a 1-2 s velocity-estimate lag.

ROS mission (`radio-sil-fpv-firmware-fixed-loss0.0-*.json`): horizontal flight is now calm
(max tilt 6.2 deg, max 2 m/s) but **altitude oscillates 4-13 m with a ~9 s period** and the
mission still ends ~190 m short. The host's vertical-speed estimate is close to half a cycle
out of phase with the truth (truth -6.9 m/s while the estimate reads -0.8), i.e. ~4 s of
effective delay: over CRSF the EKF has no baro or accelerometer, so its vertical channel leans on
GPS altitude. A softer climb gain (1 /s) sank into the ground and flipped; no host gain both holds
altitude and stays stable through that delay.

So the remaining blocker is architectural, as above: **altitude hold on the FC** (it has the
baro/IMU at 1 kHz), or put a baro/vario frame in the downlink so the host estimate is usable.
