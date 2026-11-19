# Sim agent changes on 2026-11-17 (uncommitted, for Codex to commit)

Please commit these as their own commit(s), separate from the radio-link work.
Suggested message: "sim: fix MuJoCo divergence after crashes; guard against silent resets".

Code
- `aerial_kit/airframes/multirotor_specs/mjcf.py`: integrator RK4
- `aerial_kit/airframes/twin_wing/mjcf.py`, `aerial_kit/airframes/single_wing/mjcf.py`: integrator implicit
- `sim_py/backends/mujoco_guard.py` (new): `step_guarded`, `SimulationDiverged`
- `sim_py/backends/mujoco_{multirotor,fixedwing,single_wing}_backend.py`: use `step_guarded`
- `ros2_ws/src/sim_bridge/sim_bridge/{fixedwing,multirotor}_backend_adapter_node.py`: freeze on divergence

Tests
- `sim_py/tests/test_mujoco_multirotor.py` (TestCrashRobustness), `sim_py/tests/test_mujoco_single_wing.py` (TestWingTumbleStability)
- `ros2_ws/src/sim_bridge/test/test_multirotor_adapter.py` (freeze test)

Docs / evidence
- `docs/MULTIROTOR_MUJOCO.md` (Crashes limitation)
- `docs/evidence/mujoco-divergence-2026-11-17.txt`, `docs/evidence/multirotor-gazebo-*-demo.{log,json}`, this file

Verified: sim_py 526 passed (includes Codex's new tests at the time); sim_bridge + teleop + sim_fast ROS tests 107+ passed; asset checks pass.
Note: the RK4 switch changes multirotor numerics slightly; SIL results recorded before it are still valid
for the guard but should be re-run if you need bit-exact comparisons.

Note on MUJOCO_LOG.TXT: the warnings logged at the repo root on 2026-11-17
were from the agent's deliberate reproductions (07:03-07:19) and from the guard
test forcing a blow-up at t=0 on every suite run. The test now runs that step
in a temporary directory; the stray log was deleted. Consider adding
`MUJOCO_LOG.TXT` to `.gitignore`.

## Second batch: firmware-in-the-loop (separate commit)

Suggested message: "sim: fly the unmodified AK flight core in MuJoCo (firmware-in-the-loop)".
Nothing under `firmware/` was changed; the harness only reads it.

Code
- `sim_py/fc_sil/akfc_shim.c` (new): flat C API over `ak_flight_step`
- `sim_py/fc_sil/build.py` (new): builds `sim_py/fc_sil/build/libakfc.so` from firmware/src/core (hash-cached)
- `sim_py/fc_sil/__init__.py` (new): ctypes binding `AkFlightCore`, `FirmwareInTheLoop`, `FirmwareRadio`, IMU conversion
- `sim_py/fc_sil/.gitignore` (new): ignores `build/`
- `ros2_ws/src/sim_bridge/sim_bridge/multirotor_backend_adapter_node.py`: `flight_controller:=sim|firmware`, `firmware_max_thrust_ratio`
- `ros2_ws/src/sim_fast/launch/multirotor_mujoco_teleop.launch.py`: `flight_controller` argument

Tests
- `sim_py/tests/test_fc_sil.py` (new): 11 pass + 1 expectedFailure (the estimator finding)
- `ros2_ws/src/sim_bridge/test/test_multirotor_adapter.py`: firmware mode + non-quad refusal (sim_bridge 57 passed)

Docs / evidence
- `docs/SOFTWARE_GUIDE.md`: "Flying the AK firmware itself" section
- `docs/evidence/fc-sil-estimator-finding-2026-11-17.md` (for the firmware owner)
- `docs/evidence/fc-sil-teleop-fpv-session.{log,json}`: live keyboard session, ROS_DOMAIN_ID 88
- `docs/evidence/agent-review-radio-2026-11-17.md`: added B4 (arm/mode channel swap vs AK firmware)
