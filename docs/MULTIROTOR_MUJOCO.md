# MuJoCo multirotors: FPV quad, micro LR quad, research hex and octo

One shared spec system and one MuJoCo plant for every multirotor. The
airframes:

| name | role | rotors | mass | TWR | source |
|---|---|---|---|---|---|
| `fpv_quad_5in` | FPV freestyle | 4, stretched-X, 5.1" tri-blade | 0.65 kg | 8.8 | research defaults |
| `micro_lr_quad` | FPV micro long-range | 4, 3" tri-blade | 0.38 kg | 3.0 | the `lr_drone` CAD (URDF + STLs) |
| `research_hex` | research | 6, hexa-X, 15" | 3.2 kg | 4.8 | research defaults |
| `research_octo` | research | 8, octo-X, 15" | 5.5 kg | 3.3 | research defaults |

The swarm quad is planned and not built yet. It will be one more YAML in
`aerial_kit/airframes/multirotor_specs/`, with no code change needed.

## Single source of truth

`aerial_kit/airframes/multirotor_specs/<name>.yaml` (loader `spec.py`) holds:
- mass and inertia;
- rotor positions and spins;
- per-motor max thrust, motor lag and yaw-torque coefficient;
- prop geometry, rotor inertia and display-rate shaft speed;
- linear drag;
- control bandwidths and manual-mode limits;
- geometry style and dimensions, camera, materials, collision primitives.

From it:

- `MultirotorAllocator`: the mixer, built from the actual rotor positions and
  spins (not an ideal polygon). Saturation keeps the collective and scales
  the moment demand down uniformly, preserving its direction.
- `mjcf.py`: the MuJoCo model (procedural meshes inline; CAD STLs by path).
- `controller_defaults()` on the backend: mass, composite inertia and the
  cascade bandwidths for the runner. An explicit config value still wins.

The micro quad reads its mass, inertia, rotor hub positions and meshes from
`ros2_ws/src/sim_gazebo/models/lr_drone_urdf/`. The CAD axes (+y forward, +x
right) are rotated into body FLU and re-centred on the composite CG. Two
additions are documented in its YAML:

- **Battery payload (assumed).** The CAD has no battery, which puts the CG
  1.7 cm ahead of the rotor centroid. An assumed 95 g pack on top, towards
  the rear, balances it to within 1 mm. Change it to your build.
- **Front-left prop mesh offset.** `lr_drone_alignment.yaml` gives
  `Prop_CW_2` a zero mesh offset, but that STL is geometrically identical to
  `Prop_CW` (hub at x = +0.092 m in mesh coordinates). The spec overrides it
  so the prop sits on its motor. The Gazebo `lr_drone` model still reads the
  alignment file unchanged, so its front-left prop is probably drawn off-hub.

## Conventions

- Body **FLU, z up** (x forward, y left), origin at the composite CG; world
  ENU. This matches `SimState`, `Wrench` and the cascade. The firmware's
  `ActuatorPlant` is FRD/NED, and `quad_x_seam` owns that conversion.
- Rotor `spin` is as seen from above. A CCW rotor's reaction torque on the
  body is clockwise (−z).
- Quads use the firmware/Betaflight quad-X row order (rear-right,
  front-right, rear-left, front-left) and "props-in" spin, which is the
  firmware's `QUAD_X_SPIN`. A test pins this.

## The MuJoCo model

```
body (freejoint)              frame / hub, arms, battery, gear, camera, collision primitives
  <rotor>_thrust_site         per rotor, on the ROOT body at the hub
  <rotor>_rotor (hinge +z)    bell, hub, blades, painted tips -- display spin
  [payload bodies]            CAD-style payload masses (welded)
```

- **Rotor force and yaw torque are MuJoCo actuators.** Each rotor has a
  `general` actuator on its site, with `gear = [0 0 1 0 0 k_yaw]`, so one
  control in newtons gives thrust along body +z and the matching reaction
  yaw torque. MuJoCo integrates them.
- **Propulsion model** (`propulsion.model`; all four bundled airframes use `rpm`):
  - `rpm`: each MuJoCo substep, the backend integrates
    - motor speed, with a first-order lag on *speed*;
    - thrust = `T_max · ω²`, so thrust rises slower and falls faster than a
      thrust lag would;
    - the battery (`battery` block): voltage = open-circuit(SoC) − I·R,
      where current comes from momentum-theory rotor power ÷ figure of
      merit ÷ drive efficiency, plus avionics. Available speed scales with
      voltage.

    It then sets the actuator thrust.
  - `lagged_thrust`: thrust follows the command through MuJoCo's own
    first-order filter. This is the firmware `ActuatorPlant` model the
    cross-check test pins.
- **Battery-sag compensation** (`voltage_compensation`, default on) models
  the flight controller's vbat compensation. Commanded thrust is delivered
  until the motor saturates, so sag shows up as a **lower thrust ceiling**
  (tested: at 10 % charge, full throttle gives < 85 % of the full-pack
  thrust). Off, the same throttle droops (tested).
- **Ground effect** (`propulsion.ground_effect`): Cheeseman–Bennett,
  `T_IGE/T_OGE = 1/(1 − (R/4h)²)`, per rotor by hub height, clamped at
  h = R/2.
- **Hover (research defaults):**

  | airframe | hover current | endurance (80 % of pack) |
  |---|---|---|
  | FPV | 3.9 A | ≈ 14 min |
  | micro | 4.9 A | ≈ 6 min |
  | hex | 14 A | ≈ 33 min |
  | octo | 28 A | ≈ 28 min |

  These are momentum-theory estimates, not measured endurance.
- Overrides without editing a spec, under `simulation.mujoco_multirotor`:
  `propulsion_model`, `ground_effect`, `battery_soc`, `voltage_compensation`.
- **Display spin.** A velocity actuator spins each rotor at
  `visual_max_speed * sqrt(T / T_max)` (FPV/micro 120 rad/s, research
  80 rad/s, about 1/30 of real). The rotor keeps its real inertia, so
  gyroscopic precession is under-represented by that ratio. Spin-up reaction
  torque on the frame is real.
- **Drag and wind:** linear body-axis drag on the air-relative velocity,
  applied at the CG as an external force (the same model as `ActuatorPlant`).
- **Ground:** collision primitives only: FPV box plus motor spheres, research
  hub plus skids, micro CAD box. Visual meshes never collide.
- **Cameras:** `chase` (tracks the CG) and the on-board `fpv` /
  `payload_cam`, which sits just ahead of its lens.

## Checks (tests)

- Mass and composite inertia in MuJoCo equal the spec to 1e-8.
- Thrust acts on the frame, never on the free rotor shaft.
- Open-loop hover holds within 2 cm for 2 s on every airframe.
- Motor lag reaches 63 % at τ.
- Props spin in their spec directions.
- Roll, pitch and yaw moments give rates of the right sign on the right axis.
- CCW rotors react clockwise.
- Idle on the ground rests; full thrust takes off.
- The cascade flies every airframe to a waypoint in wind, to within 0.3 m.
- **Against the firmware's `ActuatorPlant`:** same FPV airframe, same
  per-motor pulses on roll, pitch and yaw over 1.5 s. Agreement is 2.4 mm in
  position, 0.3 mm/s in velocity and 0.03 rad/s in rates, with the aircraft
  having moved 1.6 m at 2.7 m/s.

## Python runner

```bash
sim_py/.venv/bin/python examples/quadrotor/run.py --config examples/quadrotor/mujoco.yaml --no-show
```

`simulation.backend: mujoco_multirotor`, with `simulation.mujoco_multirotor.airframe`
set to one of the four names. The cascade controller gets its mass, inertia
and bandwidths from the spec.

## ROS

**Autonomous**, the same stack and contract as the point-mass sim:

```bash
ros2 launch sim_fast bringup.launch.py start_demo:=true sim_backend:=mujoco airframe:=research_hex
```

`sim_backend` defaults to `pointmass`, so the original behaviour is
unchanged. `viewer:=true` opens the MuJoCo viewer; `wind_x_mps` and
`wind_y_mps` set the wind.

**Keyboard teleop:**

```bash
ros2 launch sim_fast multirotor_mujoco_teleop.launch.py airframe:=fpv_quad_5in [mode:=angle|velocity|acro]
```

Click the "Aerial Kit multirotor teleop" window, then press Enter to arm.

| Key | velocity mode | angle mode | acro mode |
|---|---|---|---|
| W / S | fly forward / back | pitch forward / back | pitch rate |
| A / D | fly left / right | roll left / right | roll rate |
| Space / Shift | climb / descend | climb / descend (rate) | raw throttle (small steps) |
| Q / E | yaw left / right | yaw rate | yaw rate |

M cycles the mode, X re-centres the sticks, P pauses (centred sticks),
Backspace disarms, and Esc (or closing the window) exits.

The default mode is `angle` for FPV airframes and `velocity` for research
airframes. Message: `uav_msgs/MultirotorManualCommand` on
`/uav/backend/manual` (mode plus normalised sticks).

**Control source, each 10 ms tick:**

1. enabled, with a fresh (≤ 0.3 s), active, finite manual command → manual;
2. enabled, with a fresh `cmd_twist` (≤ 0.5 s) → velocity control;
3. enabled, with every command stale → **hold** (brake to a hover; link-loss
   behaviour);
4. disabled, below 0.5 m above its resting height → **idle** (motors off);
5. disabled, higher up → **auto-land** at 0.7 m/s, then idle.

A real flight controller stops its motors on disarm. The simulator refuses to
drop an aircraft from height.

Non-finite twists or sticks are refused, and they cancel the previous
command rather than waiting out the timeout. All three manual modes, and
`cmd_twist`, end in a z-up wrench that goes through the airframe's
allocator. MuJoCo integrates the rotor actuators; nothing sets the pose.

## Limitations / sim-to-real gaps

- **Rotor model:** static thrust ∝ ω² with a lagged motor speed, voltage
  sag and ground effect.
  - No advance-ratio (forward-flight) thrust loss, blade flapping, inflow or
    vortex ring.
  - Thermal, Peukert and non-linear open-circuit-voltage effects are not
    modelled.
  - Battery and propeller constants are research defaults.
- **Inertia:** the procedural airframes' inertias are research defaults. The
  micro quad's comes from CAD plus an assumed battery.
- **Drag** is linear; there are no aerodynamic moments.
- **Gyroscopic coupling** is reduced by the display-rate shaft.
- **Climb and speed loops are proportional:** against drag they settle a
  few percent short of the commanded rate (e.g. 7.45 m/s for an 8 m/s climb
  on the FPV quad).
- **Angle mode holds vertical speed** (throttle = climb rate), because a
  raw-throttle angle mode on an 8.8:1 thrust-to-weight quad is not flyable
  from on/off keys (measured: 2 s of Space reached 90 m/s). Acro keeps raw
  throttle.
- **Manual modes** are simulator flight modes built on the cascade, not a
  port of Betaflight or PX4 rate/angle controllers. There is no stick expo or
  rates curve beyond a linear map.
- **Mission tracker:** `bringup.launch.py` sets the `mpc`/`mppi` trackers'
  `velocity_loop_tau_s` per backend. Point mass uses 1/1.5 s; MuJoCo uses
  `1/velocity_kp` of the chosen airframe. Measured: the research hex with
  `mission_tracker:=mpc` finished the demo 0.01 m from the goal.
- **Gazebo** models for these airframes are not generated yet (`lr_drone`
  keeps its existing hand-made model).
- No hardware readiness is claimed.

## Gazebo visual

Each bundled multirotor now has a generated, static Gazebo model:

```text
ros2_ws/src/sim_gazebo/models/fpv_quad_5in_visual/
ros2_ws/src/sim_gazebo/models/micro_lr_quad_visual/
ros2_ws/src/sim_gazebo/models/research_hex_visual/
ros2_ws/src/sim_gazebo/models/research_octo_visual/
```

The models are generated from the same YAML specs and
`MultirotorGeometry.parts()` as MuJoCo. Procedural parts are exported as OBJ
meshes with one shared MTL per airframe. `micro_lr_quad` does not copy its CAD
files: frame and prop visuals reference the existing STLs through
`model://lr_drone_urdf/meshes/...`, using the mesh origins and rotations from
the spec, including the corrected `Prop_CW_2` offset.

Body coordinates are FLU throughout. Root geometry is on the static `body`
link, while each rotor's local parts sit on a static link at that rotor's
origin. The model has no collisions, inertials or dynamics: Gazebo provides
the scene and camera, while MuJoCo remains the authoritative plant.

Generate or verify the assets:

```bash
python3 scripts/generate_multirotor_assets.py
python3 scripts/generate_multirotor_assets.py --check
python3 scripts/generate_multirotor_assets.py --airframe research_hex
```

Run the live visualisation with any bundled airframe:

```bash
ros2 launch sim_gazebo multirotor_gazebo_demo.launch.py \
  headless:=true start_demo:=true airframe:=research_hex
```

`multirotor_gazebo_demo.launch.py` starts
`sim_fast/bringup.launch.py` with `sim_backend:=mujoco`, starts Gazebo with
`multirotor_demo.sdf`, spawns `model://<airframe>_visual`, bridges
`/world/uav_multirotor/set_pose`, and runs `gazebo_multirotor_pose_node`.
That node subscribes to `/uav/backend/odom` and sends one pose service request
at a time at 20 Hz.

This visual has the same part set as the MuJoCo mesh geometry, but it does not
animate rotor spin or articulate anything. Pose following is kinematic and
has no sensor, contact or aerodynamic feedback into Gazebo. The model is a
review and scene-visualisation asset, not a second Gazebo flight plant.
