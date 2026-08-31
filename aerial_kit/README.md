# aerial-kit

Control stack for aerial robots. The core has no ROS or Matplotlib dependency;
simulation and visualization are available as optional extras.

For firmware, hardware integration, and the full ROS 2 / simulation stack, visit the repo:
[github.com/RawFish69/aerial-kit](https://github.com/RawFish69/aerial-kit).

## What's in it

- `aerial_kit.types` - `SimState`, `ControlTarget`, `Capabilities`, `Wrench`,
  `CommandKind`, `Waypoint`
- `aerial_kit.interfaces` - `Controller`, `DynamicsBackend`, `Planner` ABCs
- `aerial_kit.registry` - a pluggable component registry
  (`register_airframe`/`register_controller`/etc., `create_*` factories)
- `aerial_kit.airframes` - `Airframe` ABC, `MultirotorAirframe` (mixer-driven quad/hex/
  octo), `TwinWingAirframe` (elevon + differential-thrust allocation, trim)
- `aerial_kit.dynamics` - 6-DOF multirotor dynamics, point-mass dynamics, and a
  hand-rolled 6-DOF fixed-wing model with a flat-plate-blended lift curve, drag polar,
  and moment derivatives
- `aerial_kit.controllers` - PID/LQR/MPC position controllers, and
  `FixedWingL1TECSController` (L1 lateral guidance + TECS-lite longitudinal control +
  coordinated-turn attitude PID)
- `aerial_kit.guidance` - `l1_bank_command`, `tecs_command` as standalone functions

## Quick start

```python
from aerial_kit.registry import register_builtin_components, create_airframe, create_controller

register_builtin_components()
airframe = create_airframe("quad")
controller = create_controller("pid")
print(airframe.capabilities)
```

`register_builtin_components()` here registers only what lives inside `aerial_kit`
itself (airframes, controllers) - it has no ROS or matplotlib dependency and does not
know about any host application's own dynamics backends or planners. A host application
(like `sim_py` in the parent repo) registers its own backends/planners into the same
registry alongside this.

## Simulation and visualization

Install the optional simulator and Matplotlib viewer:

```bash
python -m pip install "aerial-kit[sim]"
```

Run the bundled default quadrotor scenario:

```bash
aerial-kit-sim
aerial-kit-sim --example fixed-wing
aerial-kit-sim --no-show --save result.png
```

### Quadrotor teleop

Launch real-time keyboard teleoperation with one command:

```bash
aerial-kit-teleop
```

| Key | Action |
| --- | --- |
| `W` / `S` or `Up` / `Down` | forward / backward |
| `A` / `D` or `Left` / `Right` | strafe left / right |
| `Space` / `Shift` | climb / descend |
| `Q` / `E` | yaw left / right |
| `X` | neutralize all commands |
| `P` | pause / resume |
| `C` | toggle follow / world camera |
| `-` / `=` | zoom the follow camera out / in |
| `H` | hide the on-screen help |
| `Esc` | exit |

Controls are body-relative, so yawing with `Q`/`E` changes where `W` takes you.
Click the plot window first; the HUD shows `NO FOCUS` when keystrokes are not
reaching it, and held keys are released whenever focus is lost.

The equivalent simulator command is `aerial-kit-sim --teleop`. Without installing
the package, run `python -m aerial_kit.sim.teleop` from the repository root, or
`python examples/quadrotor/teleop.py` from anywhere (which is also what an IDE
Run button does). Teleop needs an interactive Matplotlib backend and fails with
an explanatory message if the active backend can only write files.

The public Python API accepts a YAML path, a mapping, or a normalized config:

```python
from aerial_kit.sim import load_config, run_simulation
from aerial_kit.visualization import plot_simulation

config = load_config("examples/quadrotor/config.yaml")
result = run_simulation(config)
print(f"goal error: {result.distance_to_goal:.2f} m")
plot_simulation(result)
```

Complete configurable examples are included in the repository:

- `examples/quadrotor`: quad airframe, PID control, and native multirotor dynamics.
- `examples/fixed_wing`: twin-motor flying wing, Dubins planning, native 6-DOF
  dynamics, and L1/TECS control.

The fixed-wing example starts at cruise airspeed to represent a hand launch. A
fixed wing cannot be initialized at zero velocity like a hovering multirotor.

## Status

Early and actively developed. Be clear-eyed about what's actually verified:

- **Multirotor (quad/hex/octo airframes, PID/LQR/MPC): verified, working.** This is the
  path flying on real hardware in the parent project.
- **Twin-motor wing (`TwinWingAirframe`, `FixedWingL1TECSController`, L1/TECS guidance):
  under development, simulation-only.** Not flown on any hardware. Its 6-DOF aero model
  uses plausible placeholder coefficients, not a fitted model of any specific real
  airframe, and its guidance gains are validated only in the specific simulated scenarios
  its own test suite covers - treat it as a research/simulation component, not something
  to fly as-is.

## License

MIT
