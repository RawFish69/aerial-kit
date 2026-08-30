# sim_py Information

## What `sim_py` is

`sim_py` is the standalone Python simulation stack in this repo.

It is organized as a lightweight framework with pluggable components:

- `Planner`: builds waypoints from start to goal
- `Controller`: computes control targets from state + waypoint
- `DynamicsBackend`: advances simulation state over time
- `Airframe`: capabilities, actuator allocation, and trim for one vehicle family

Core orchestration is in `sim_py/core/runner.py`. The datatypes, interfaces, and registry
that used to live under `sim_py/core/` now live in the standalone `aerial_kit` package
at the repo root (no ROS, no matplotlib), so `hw_bridge` and other ROS nodes can eventually
import the same control code `sim_py` uses. `sim_py/core/{types,interfaces,registry}.py`
are thin re-export shims kept for backward compatibility -- import from `aerial_kit`
directly in new code.

### Airframe scope

`Airframe` is a registry-plugged type like the other three: `quad`/`hex`/`octo`
(`MultirotorAirframe`, config-only mixer geometry) and `twin_wing` (`TwinWingAirframe`,
allocation/trim) are registered in `aerial_kit.registry.AIRFRAMES`. Select one with
`--airframe` or `vehicle.airframe` in `sim_config.yaml`. `PointMassBackend` has no actuator
concept and ignores which airframe is selected -- it still integrates `accel_cmd` directly --
so today airframe selection changes what `airframe.allocate()` computes (exercised every
step for validation) but not the simulated trajectory. A controller/airframe `CommandKind`
mismatch (e.g. `pid` + `twin_wing`) fails at startup with a readable error rather than
silently misbehaving.

`FixedWingBackend` (`sim_py/backends/fixedwing_backend.py`, registered as `fixedwing`) is a
real 6-DOF rigid body + hand-rolled aero model (linear lift curve blended into a flat-plate
model past stall, drag polar, pitch/roll/yaw moment derivatives, wind as a world-frame term
so airspeed != groundspeed -- see `aerial_kit/dynamics/fixed_wing.py`). Aero/inertia coefficients
are a plausible default set for a small flying wing, not a fitted model of any real airframe
(`simulation.fixedwing.*` in `sim_config.yaml`) -- real, measured CAD data for the user's
actual airframe exists in `plans/04-twin-wing-control/REAL_AIRFRAME_SPEC.md` but is not yet
integrated.

`FixedWingL1TECSController` (`aerial_kit/controllers/fixed_wing.py`, registered as `l1_tecs`,
`command_kind=AIRSPEED_NAV`) closes the loop: L1 lateral guidance
(`aerial_kit/guidance/l1.py`, point-target variant since `Controller.compute()` only
gets one target waypoint, not a path segment) picks a bank command, TECS-lite
(`aerial_kit/guidance/tecs.py`) picks throttle + pitch from airspeed/altitude error, and
an attitude PID converts those into a `Wrench` for `TwinWingAirframe.allocate()`. `runner.py`
now dispatches on `airframe.capabilities.command_kind`: ACCEL keeps driving `accel_cmd`
exactly as before (bit-identical, verified via the existing quad/hex/octo tests), AIRSPEED_NAV
pulls a `Wrench` from `ControlTarget.metadata["wrench"]`, allocates it, and hands the backend
`ControlTarget.metadata["actuator_cmd"]` instead. `--airframe twin_wing --backend fixedwing
--controller l1_tecs` is therefore wired end-to-end, though `run_simulation()` always starts
missions at zero velocity (no runway/launch model), so it's exercised directly at cruise trim
in tests rather than through a full CLI mission -- see
`sim_py/tests/test_fixed_wing_guidance.py` (including a 30 s straight-and-level stability
check) and `test_runner_integration.py`'s dispatch smoke test. Full coordinated turns
(bank-to-turn-rate coupling, not just this model's weak weathervaning aero) remain open --
see `plans/PROGRESS.md`.

## Quick usage

From repo root:

```bash
./scripts/setup_sim_py_venv.sh
source sim_py/.venv/bin/activate
python -m sim_py.run_sim
```

Useful options:

```bash
# Switch controller
python -m sim_py.run_sim --controller lqr

# Switch airframe (config-only mixer for hex/octo; twin_wing needs a matching controller)
python -m sim_py.run_sim --airframe hex

# Override terrain
python -m sim_py.run_sim --terrain mountains

# Override runtime
python -m sim_py.run_sim --sim-time 120 --dt 0.01

# Optional RotorPy backend
./scripts/setup_sim_py_venv.sh --with-rotorpy
source sim_py/.venv/bin/activate
python -m sim_py.run_sim --backend rotorpy

# RotorPy + mountains terrain
python -m sim_py.run_sim --backend rotorpy --terrain mountains

# RotorPy + mountains + LQR controller
python -m sim_py.run_sim --backend rotorpy --terrain mountains --controller lqr
```

Headless environments:

```bash
MPLBACKEND=Agg python -m sim_py.run_sim --backend pointmass
```

## Main config

Primary config file: `sim_py/sim_config.yaml`

Key sections:

- `path.*`: planner config and start/goal settings
- `controller.*`: controller selection and gains
- `visual.*`: plotting and terrain visual scaling
- `simulation.backend`: `pointmass` (default) or `rotorpy`
- `simulation.seed`: optional deterministic random seed
- `simulation.rotorpy.*`: optional RotorPy backend settings

CLI flags override config values where applicable.

## Architecture map

Entry + app:

- `sim_py/run_sim.py` (compat entrypoint)
- `sim_py/app/cli.py`
- `sim_py/app/main.py`

`aerial_kit` is the standalone, publishable package (no ROS, no matplotlib) -- it holds
everything reusable by anything that isn't `sim_py`'s own orchestration/plotting:

- `aerial_kit/types.py` (`SimState`, `ControlTarget`, `Capabilities`, `Wrench`, `CommandKind`, ...)
- `aerial_kit/interfaces.py` (`Planner`, `Controller`, `DynamicsBackend`)
- `aerial_kit/registry.py` (`PLANNERS`/`CONTROLLERS`/`BACKENDS`/`AIRFRAMES`)
- `aerial_kit/airframes/` (`base.py`, `multirotor.py`, `fixed_wing.py`)
- `aerial_kit/dynamics/` (`pointmass.py`, `multirotor.py`, `fixed_wing.py` -- pure numpy 6-DOF/point-mass models)
- `aerial_kit/controllers/` (`position.py` position-level pid/lqr/mpc math, `basic.py`
  `Controller` subclasses, `fixed_wing.py` `FixedWingL1TECSController`)
- `aerial_kit/guidance/` (`l1.py`, `tecs.py`)

Core orchestration (sim_py-specific, thin shims into `aerial_kit` for `core.{types,interfaces,registry}`):

- `sim_py/core/types.py`, `interfaces.py`, `registry.py` (re-export shims)
- `sim_py/core/config.py`
- `sim_py/core/runner.py`

Built-in plugins that stay in `sim_py` (matplotlib-adjacent, ROS-adjacent, or external-optional-dependency):

- `sim_py/planners/basic.py`, `dubins.py` (steering geometry, not yet wired into a planner)
- `sim_py/backends/pointmass_backend.py`, `fixedwing_backend.py` (thin `DynamicsBackend`
  wrappers around `aerial_kit.dynamics.*`)
- `sim_py/backends/rotorpy_backend.py` (external optional dependency, not in `aerial_kit`)

## Built-in registry keys

Planners:

- `straight`
- `astar`
- `rrt`
- `rrtstar` (aliases: `rrt*`, `rrt_star`)
- `dubins` (`DubinsPlanner`, curvature-feasible single start->goal path; reads
  `path.turn_radius_m`, which `runner.py` populates from the selected airframe's
  `Capabilities.min_turn_radius_m` when set)

Controllers:

- `pid`
- `lqr`
- `mpc`
- `l1_tecs` (`FixedWingL1TECSController`, AIRSPEED_NAV -- pairs with `twin_wing`/`fixedwing`)

Backends:

- `pointmass`
- `rotorpy`
- `fixedwing` (`FixedWingBackend` -- see Airframe scope above for the zero-velocity-launch caveat)

Airframes:

- `quad`, `hex`, `octo` (`MultirotorAirframe`, config-only mixer)
- `twin_wing` (`TwinWingAirframe`, allocation/trim; pairs with `fixedwing` + `l1_tecs`)

## How to add new plugins

1. Implement one interface from `aerial_kit/interfaces.py` (or subclass
   `aerial_kit.airframes.base.Airframe` for a new airframe).
2. Register it in `aerial_kit/registry.py`.
3. Keep config namespaced:
- planner params under `path.*`
- controller params under `controller.<name>.*`
- backend params under `simulation.<backend_name>.*`
- airframe choice under `vehicle.airframe`

No runner changes are required for normal planner/controller/backend/airframe additions.
