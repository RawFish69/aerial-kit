# Claude changes on 2026-11-17 (uncommitted, for Codex to commit)

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
were from Claude's deliberate reproductions (07:03-07:19) and from the guard
test forcing a blow-up at t=0 on every suite run. The test now runs that step
in a temporary directory; the stray log was deleted. Consider adding
`MUJOCO_LOG.TXT` to `.gitignore`.
