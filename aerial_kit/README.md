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
- `aerial_kit.controllers` - PID/LQR/MPC position controllers; `ConstrainedMPC` (QP MPC
  with acceleration and speed limits inside the optimisation, solved by a small ADMM
  `BoxQP`); `MPPI` (vectorised, warm-started sampling controller with sphere-obstacle and
  floor costs); `PathReference` (horizon references along a waypoint polyline);
  attitude-level `GeometricController` (SE(3) tracking with flatness feedforward) and
  `NMPCController` (iLQR nonlinear MPC on thrust and body rates), both producing a wrench;
  and `FixedWingL1TECSController` (L1 lateral guidance + TECS-lite longitudinal control +
  coordinated-turn attitude PID)
- `aerial_kit.guidance` - `l1_bank_command`, `tecs_command` as standalone functions
- `aerial_kit.trajectory` - `MinSnapTrajectory`: piecewise minimum-snap through waypoints with
  speed/acceleration/jerk limits and optimised segment times, queryable for position through
  snap and as a `FlatReference`
- `aerial_kit.estimation` - `InsEkf`: loosely coupled INS Kalman filter (GPS, baro, optional
  accelerometer; bias states, outlier gating); `SimulatedIns`: simulated GPS, baro and IMU
  feeding it, so a simulated controller can fly on the estimate (`simulation.estimator.mode: ekf`)

## Quick start

```python
import numpy as np

from aerial_kit import (
    SimState,
    Waypoint,
    create_airframe,
    create_controller,
    register_builtin_components,
)

register_builtin_components()
airframe = create_airframe("quad")
controller = create_controller("pid")
state = SimState(position=np.zeros(3), velocity=np.zeros(3))
waypoint = Waypoint(position=np.array([1.0, 0.0, 1.0]))
target = controller.compute(state, waypoint, {})
assert target.accel_cmd.shape == (3,)
print(airframe.capabilities.command_kind, target.accel_cmd)
```

The top-level exports, `aerial_kit.interfaces`, `aerial_kit.types` and
`aerial_kit.registry` are the typed public contract and are checked in CI with
strict function signatures. The distribution ships a PEP 561 `partial` marker:
specialized numerical internals remain importable but are not claimed as a
fully typed API surface.

`register_builtin_components()` here registers only what lives inside `aerial_kit`
itself (airframes, controllers) - it has no ROS or matplotlib dependency and does not
know about any host application's own dynamics backends or planners. A host application
(like `sim_py` in the parent repo) registers its own backends/planners into the same
registry alongside this.

## Flight-controller CLI

The `ak` executable auto-detects an Aerial Kit controller (AKP) or a
Betaflight controller (MSP) on a USB serial port:

```bash
ak fc status --port /dev/ttyACM0
ak fc rc --port /dev/ttyACM0
ak fc params --port /dev/ttyACM0 --name failsafe_throttle
ak fc get failsafe_throttle --port /dev/ttyACM0
ak fc set failsafe_throttle 1200 --port /dev/ttyACM0 --write
ak fc save --port /dev/ttyACM0 --write
ak fc preflight --port /dev/ttyACM0 --protocol betaflight --json fc-preflight.json
ak fc rc --port /dev/ttyACM0 --protocol betaflight --json fc-rc.json
```

Use `--protocol akp` or `--protocol betaflight` to skip auto-detection. Reads
are safe by default. Configuration writes require `--write`, are refused while
the board reports itself armed, and are never retried after a refusal. Aerial
Kit parameters can be enumerated; Betaflight settings are addressed by name
because MSP has no enumerate-all command.

`ak fc status` treats a successful Betaflight MSP response as proof that the
flight controller is reachable, not proof that its receiver is linked.
Betaflight prints `link=unknown`; take a fresh `ak fc rc` observation and decode
the arming-disable flags with `ak fc preflight`. Failsafe or receiver-failsafe
flags make the preflight command exit nonzero.

For bounded live keyboard control through the CRSF transmitter:

```bash
ak radio teleop --port /dev/ttyACM0 --profile betaflight \
  --duration 30 --write --confirm-arm
```

The stream starts disarmed. Press Enter to arm after the low-throttle hold;
A/D, W/S and Q/E step roll, pitch and yaw, R/F change throttle, C centres the
sticks, and X panic-disarms and exits. Every exit path attempts a disarmed
frame. The no-argument `ak teleop` command remains the simulator viewer.

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

### Teleop

Launch real-time keyboard teleoperation with one command:

```bash
aerial-kit-teleop                          # quadrotor
aerial-kit-teleop --airframe fixed-wing    # twin-wing
```

**Quadrotor** — drone-style controls:

| Key | Action |
| --- | --- |
| `W` / `S` or `Up` / `Down` | forward / backward |
| `A` / `D` or `Left` / `Right` | strafe left / right |
| `Space` / `Shift` | climb / descend |
| `Q` / `E` | yaw left / right |

**Fixed wing** — RC-plane-style controls (no rudder, so `Q`/`E` biases
differential thrust rather than yawing directly):

| Key | Action |
| --- | --- |
| `W` / `S` or `Up` / `Down` | pitch: dive / climb |
| `A` / `D` or `Left` / `Right` | bank: turn right / left |
| `Space` / `Shift` | throttle up / down |
| `Q` / `E` | differential-thrust yaw nudge |

Shared: `X` neutralize, `P` pause, `C` toggle follow/world camera, `-`/`=`
zoom, `H` hide help, `Esc` exit.

Controls are body-relative, so yawing with `Q`/`E` changes where `W` takes you.
Click the plot window first; the HUD shows `NO FOCUS` when keystrokes are not
reaching it, and held keys are released whenever focus is lost. The follow
camera is a third-person chase view that stays behind the vehicle and turns
with its heading.

The equivalent simulator commands are `aerial-kit-sim --teleop` and
`aerial-kit-sim --example fixed-wing --teleop`. Without installing the
package, run `python -m aerial_kit.sim.teleop` (add `--airframe fixed-wing`
for the wing) from the repository root, or `python examples/quadrotor/teleop.py`
/ `python examples/fixed_wing/teleop.py` from anywhere (which is also what an
IDE Run button does). Teleop needs an interactive Matplotlib backend and fails
with an explanatory message if the active backend can only write files.

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
