# aerial-kit

A control stack for aerial robots covering multirotors and fixed-wing aircraft: airframe
capabilities, actuator allocation/trim, 6-DOF and point-mass dynamics models, controllers
(PID/LQR/MPC, and an L1 + TECS fixed-wing autopilot), and lateral/longitudinal guidance
laws -- all pure numpy/scipy, no ROS and no matplotlib dependency.

It's the `aerial_kit` package inside the
[aerial-kit](https://github.com/RawFish69/aerial-kit) repo, a
quadcopter-to-general-aerial-robotics control stack -- there, it's the shared layer
imported by both a standalone Python simulator and ROS 2 flight nodes so they don't
duplicate control code.

## What's in it

- `aerial_kit.types` -- `SimState`, `ControlTarget`, `Capabilities`, `Wrench`,
  `CommandKind`, `Waypoint`
- `aerial_kit.interfaces` -- `Controller`, `DynamicsBackend`, `Planner` ABCs
- `aerial_kit.registry` -- a pluggable component registry
  (`register_airframe`/`register_controller`/etc., `create_*` factories)
- `aerial_kit.airframes` -- `Airframe` ABC, `MultirotorAirframe` (mixer-driven quad/hex/
  octo), `TwinWingAirframe` (elevon + differential-thrust allocation, trim)
- `aerial_kit.dynamics` -- 6-DOF multirotor dynamics, point-mass dynamics, and a
  hand-rolled 6-DOF fixed-wing model with a flat-plate-blended lift curve, drag polar,
  and moment derivatives
- `aerial_kit.controllers` -- PID/LQR/MPC position controllers, and
  `FixedWingL1TECSController` (L1 lateral guidance + TECS-lite longitudinal control +
  coordinated-turn attitude PID)
- `aerial_kit.guidance` -- `l1_bank_command`, `tecs_command` as standalone functions

## Quick start

```python
from aerial_kit.registry import register_builtin_components, create_airframe, create_controller

register_builtin_components()
airframe = create_airframe("quad")
controller = create_controller("pid")
print(airframe.capabilities)
```

`register_builtin_components()` here registers only what lives inside `aerial_kit`
itself (airframes, controllers) -- it has no ROS or matplotlib dependency and does not
know about any host application's own dynamics backends or planners. A host application
(like `sim_py` in the parent repo) registers its own backends/planners into the same
registry alongside this.

## Status

Early and actively developed. Be clear-eyed about what's actually verified:

- **Multirotor (quad/hex/octo airframes, PID/LQR/MPC): verified, working.** This is the
  path flying on real hardware in the parent project.
- **Twin-motor wing (`TwinWingAirframe`, `FixedWingL1TECSController`, L1/TECS guidance):
  under development, simulation-only.** Not flown on any hardware. Its 6-DOF aero model
  uses plausible placeholder coefficients, not a fitted model of any specific real
  airframe, and its guidance gains are validated only in the specific simulated scenarios
  its own test suite covers -- treat it as a research/simulation component, not something
  to fly as-is.

## License

MIT
