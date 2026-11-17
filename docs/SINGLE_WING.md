# Single-motor flying-wing model

This is a research model of a small swept flying wing with one centreline
pusher motor, two elevons and no rudder. It is sized for roughly 1.1 kg and
1.2 m span. It is an analytic 6-DOF model for control and mission work, not a
validated model of a physical aircraft.

Every dimensional, mass, aerodynamic and propulsion number in
`aerial_kit/airframes/single_wing/single_wing.yaml` is a **research default,
not a measurement or fitted coefficient**. The YAML is the single source of
truth. `spec.py` derives the planform and constructs `SingleWingParams`; tests
fail if the dataclass defaults or consumers drift from it.

The operational defaults are a 14 m/s cruise, 9 m/s minimum, 24 m/s maximum,
3 m/s maximum climb rate and 45 degree maximum bank. The 9 m/s minimum is
deliberately above the approximately 8.8 m/s stall speed of this default lift
model; it is not a hard structural or control law.

## Conventions

- Body axes are FRD: x forward, y right, z down.
- World axes are ENU. `attitude_quat` maps body FRD vectors to world ENU.
- Actuators are in physical order
  `[thrust_n, elevon_left_rad, elevon_right_rad]`.
- Elevons are trailing-edge-up positive.
- Equal positive elevons pitch nose-up.
- Positive differential, `elevon_right - elevon_left`, rolls right.
- There is no yaw actuator. A requested yaw moment is dropped; turns must be
  coordinated by bank, with yaw coming only from aerodynamic weathervaning.

FRD vectors and the quaternion convention match the twin-wing model. The
`Wrench` handed to `SingleWingAirframe.allocate()` uses Aerial Kit's existing
wing-controller convention: `moment_body[0]` and `moment_body[1]` are
roll/pitch demand magnitudes interpreted by the allocator, while
`moment_body[2]` has no corresponding actuator.

## What is modelled

### Rigid body and aerodynamics

`SingleWingDynamics` integrates a 6-DOF rigid body with forward Euler and the
same body-rate clamp as `FixedWingDynamics.step()`. Lift, drag and the
post-stall flat-plate blend are imported from `dynamics/fixed_wing.py`; they
are not copied.

The model includes angle-of-attack lift, quadratic induced drag, sideslip side
force and weathervaning, roll damping, pitch damping, yaw damping, elevon pitch
authority and elevon roll authority. The planform, mass and inertia come from
the YAML. Aileron/elevon commands are physical left and right trailing-edge-up
angles.

### Propulsion effects

All four effects have YAML feature switches:

| Effect | Switch | Model |
|---|---|---|
| Motor reaction | `motor_reaction_torque` | `-spin * Kq * thrust` about the thrust axis |
| Thrust lag | `thrust_lag` | First-order response with `thrust_tau_s` |
| Pusher wash | `prop_wash` | Small effective elevon roll/pitch authority gain at low dynamic pressure and high thrust |
| Thrust-line offset | `thrust_offset_moment` | `r_motor x F_thrust`, including its pitch moment |

The prop-wash factor is intentionally small. A pusher propeller sits behind
the elevons, so most slipstream has already passed the wing; the term represents
an effective low-speed authority allowance rather than a detailed slipstream
map. It scales only the elevon moment terms and does not add another lift model.

Motor and propeller inertias are retained in the spec for a later shaft-state
model. This analytic model uses the first-order thrust lag directly and does
not integrate a separate rotor angular velocity; the stored inertias therefore
do not contribute a second reaction-torque transient here.

## Trim

`compute_trim(airspeed)` solves the pre-stall level-flight force balance using
the actual nonlinear `lift_coefficient()` function, not a separate linear
approximation. It then solves:

- common elevon angle for the pitch moment, including thrust-line offset;
- elevon differential for motor reaction roll torque;
- collective thrust for drag and the tilted lift/thrust force balance.

At 14 m/s the research-default model gives:

| Quantity | Value |
|---|---:|
| Thrust | 1.1310 N |
| Angle of attack | 2.2215 deg |
| Lift coefficient | 0.3730 |
| Common elevon | +2.2402 deg |
| Left elevon | +2.1383 deg |
| Right elevon | +2.3421 deg |
| Prop-wash authority scale | 1.00314 |

The small right/left difference cancels the clockwise-from-behind propeller's
reaction torque. Trim assumes zero body rates, zero sideslip, no wind and a
rigid, undamaged aircraft.

## Allocation

`SingleWingAirframe.allocate()` clamps thrust to the motor range, then keeps
pitch demand first. Roll receives only the elevon travel left after pitch, and
its base includes reaction-torque compensation. This is saturation by priority,
not independent clipping followed by an illegal mixer result.

The 3-actuator command is:

```text
elevon_left  = common_pitch - roll
elevon_right = common_pitch + roll
```

## Run the example

From the repository root:

```bash
sim_py/.venv/bin/python examples/single_wing/run.py --no-show
sim_py/.venv/bin/python examples/single_wing/run.py --save single-wing.png
```

The example starts airborne at 14 m/s, plans a Dubins `L-S-L` path from
`[0, 0, 100]` to `[220, 60, 100]`, follows its lookahead samples with the shared
L1/TECS attitude law and integrates every actuator command through
`SingleWingDynamics`. A fixed wing cannot hover, so it never starts from zero
airspeed.

The current research-default run reaches the 12 m acceptance radius with about
2.8 m final goal error, 3.8 m peak cross-track error and 14.2 m/s final
airspeed.

## Tests

```bash
sim_py/.venv/bin/python -m pytest -q sim_py/tests
```

The single-wing tests cover YAML/parameter drift, a 30-second balanced trim,
elevon signs, spin-direction reaction torque and its roll trim, pitch-over-roll
saturation, propulsion switches, air-relative wind, full-stick numerical
robustness and a closed-loop Dubins/L1 mission.

## Approximations and limitations

- No aerodynamic coefficient is fitted or measured. Planform geometry does not
  directly generate the coefficients.
- No spanwise flow, finite-wing lift distribution, washout, dihedral, fin,
  fuselage or propeller wake geometry is resolved.
- Elevons change moments but not lift, drag or hinge load. Servo rate, servo
  dynamics, blow-back and mechanical compliance are absent.
- Thrust is a lagged command. There is no motor electrical model, battery sag,
  advance ratio, propeller thrust curve or shaft-speed integration.
- Motor reaction uses a constant torque-per-thrust coefficient. The model does
  not integrate rotor inertia or spin-up angular acceleration.
- Pusher wash is a coarse, deliberately weak authority scale.
- No landing gear, launch, touchdown, ground contact or crash model exists.
- The controller flies on the true state in the example; no estimator is used.
- Forward Euler at 10-20 ms is adequate for the tested mission but is not a
  high-fidelity or real-time certification model.
- There is no Gazebo or hardware validation in this model (MuJoCo and ROS 2:
  see "MuJoCo and ROS integration" below).

## MuJoCo and ROS integration

Added after the analytic model, in the same pattern as the twin wing:

- **Registered:**
  - backends `singlewing` (analytic) and `mujoco_singlewing`;
  - airframe `single_wing`;
  - controller `single_wing_l1_tecs`.

  The generic runner flies `examples/single_wing/config.yaml` on either
  backend. Both backends supply the spec's trim thrust to the L1/TECS
  controller (`controller_defaults`).
- **`aerial_kit/airframes/single_wing/mjcf.py`.** Wing, pod, fins and
  elevons come from the twin-wing procedural generator, through a planform
  view of `single_wing.yaml`; plus a centreline pusher motor pod, bell,
  spinner and prop.
  - Elevons are servo hinges (`articulation.elevon_servo`), and the prop
    hinge spins about +x in the spec's direction.
  - The composite mass and inertia equal the spec exactly.
  - The values only MuJoCo needs (servo, rotor mass, prop visual) are in the
    `articulation` block.
- **`sim_py/backends/mujoco_single_wing_backend.py`.** MuJoCo integrates the
  free body, the elevon servos and the prop shaft. `SingleWingDynamics`
  supplies the loads, evaluated on the measured hinge angles and the lagged
  thrust.
- **ROS:** `fixedwing_backend_adapter_node` takes `airframe:=single_wing`,
  and so do both launch files:

  ```bash
  ros2 launch sim_fast fixed_wing_mujoco_demo.launch.py airframe:=single_wing
  ros2 launch sim_fast fixed_wing_mujoco_teleop.launch.py airframe:=single_wing
  ```

  - Manual `ActuatorCommand`: the single motor's thrust is
    `thrust_left_n + thrust_right_n`. The teleop sends half its trim on each.
  - Q/E do nothing, because there is no yaw actuator.
  - Allocation goes through `SingleWingAirframe`, which adds its own trim
    and motor-reaction compensation.
- **Measured:** the MuJoCo + EKF + 3/1 m/s wind ROS mission completed, with
  a final distance of 9.20 m, altitude within 2.73 m of cruise and no NaNs.

## Shared registrations (merged)

These registrations were merged into the shared registries on 2026-11-16:

```python
# aerial_kit/airframes/__init__.py
from .single_wing_airframe import SingleWingAirframe
# add "SingleWingAirframe" to __all__

# aerial_kit/registry.py
from .airframes.single_wing_airframe import SingleWingAirframe
from .controllers.single_wing import SingleWingL1TECSController
register_airframe("single_wing", SingleWingAirframe)
register_controller("single_wing_l1_tecs", SingleWingL1TECSController)

# sim_py/core/registry.py
from sim_py.backends.single_wing_backend import SingleWingBackend
register_backend("singlewing", SingleWingBackend)
```
