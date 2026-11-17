# Twin-wing airframe model

The twin-motor flying wing has one description,
`aerial_kit/airframes/twin_wing/twin_wing.yaml`. Everything else is generated
from it or reads it:

| Consumer | How |
|---|---|
| MuJoCo model | `aerial_kit.airframes.twin_wing.mjcf.twin_wing_mjcf()` builds the MJCF at runtime (inline meshes, no files) |
| Gazebo visual | `scripts/generate_twin_wing_assets.py` writes `ros2_ws/src/sim_gazebo/models/twin_wing_visual/` (SDF + OBJ + MTL) |
| Aero model | `TwinWingSpec.fixed_wing_params()` fills mass, inertia, area, span, chord and motor separation of `FixedWingParams` |
| ROS adapter | loads the spec (or `airframe_spec:=...`) and passes those numbers to both the analytic and MuJoCo plants |

Drift guards:

- `generate_twin_wing_assets.py --check` (run by `sim_gazebo` tests) fails if the checked-in Gazebo files differ from the YAML.
- `sim_py/tests/test_twin_wing_model.py` fails if `FixedWingParams` / `TwinWingAirframe` defaults, `aerial_kit/sim/defaults/fixed_wing.yaml` or `examples/fixed_wing/mujoco.yaml` disagree with the YAML.
- A ROS test checks that the MuJoCo mesh geoms and the Gazebo visuals are the same set of parts.

To change the aircraft: edit the YAML, run
`python3 scripts/generate_twin_wing_assets.py`, run the tests.

## Frame and layout

Body FRD (x forward, y right, z down), origin at the reference CG, which is
also the aero moment reference. World is ENU; the free joint's quaternion is
body-FRD → world-ENU, as in `SimState.attitude_quat`.

- Planform: 1.40 m span, 0.345 m root / 0.155 m tip chord (S = 0.35 m², mean
  chord 0.25 m, matching the aero reference), 28° leading-edge sweep, 2°
  visual washout, reflexed thick-section airfoil. Geometric static margin
  (quarter-MAC estimate) ≈ 10.7 % MAC.
- Equipment pod faired into the centre section, hatch, nose camera window.
- Wingtip fins (0.13 m tall, 38° sweep).
- Two tractor nacelles at y = ±0.30 m, 0.23 m two-blade props with hubs,
  spinners and outrunner bells.
- Elevons from y = 0.22 to 0.64 m, 25 % local chord, on a swept hinge line;
  3 mm hinge slot, visible hinge rod ends, control horns.
- Colours: light top / dark bottom, orange moving surfaces, red port / green
  starboard nav lights, yellow prop tips.

## Body tree (MuJoCo)

```
body  (freejoint)            wing halves, pod, fins, nacelles, hinge rods, nav lights
├── left_elevon   hinge      swept hinge axis, ±27° stop, servo ±25°
├── right_elevon  hinge
├── left_prop     hinge +x   spin −1 (CCW from behind)
└── right_prop    hinge +x   spin +1 (CW from behind; inboard blades rise)
```

Mass is split so the composite is exactly the spec: 1.5 kg, CG at the origin,
inertia diag(0.05, 0.03, 0.06) kg·m² about it at neutral elevons (tested to
1e-7). Elevons are 25 g each and rotors 30 g each; the root body gets the
remainder with the parallel-axis terms removed.

## What is what

| Part | Status |
|---|---|
| Free-body motion | **Physically integrated** by MuJoCo (2 ms substeps, implicitfast) |
| Lift, drag, thrust, aero moments | **Model**: `FixedWingDynamics` coefficients, unchanged; research defaults, **not fitted** to this geometry or any real aircraft |
| Elevon hinges | **Articulated and integrated**: position servo (kp 35, kv 2.5), 8 rad/s slew limit, joint damping, armature, mechanical stops, surface mass. Aero is fed the *measured* hinge angles |
| Elevon hinge aerodynamic moment | **Not modelled** (servo load is inertia/gravity only) |
| Prop shafts | **Articulated and integrated**: velocity-controlled motor whose time constant equals the thrust lag; reaction torque and gyroscopic coupling act on the airframe |
| Prop shaft speed | **Visual-rate**: full thrust = 150 rad/s instead of a real ~1200 rad/s so blades read at 60 Hz. Shaft inertia is scaled up by the same ratio so angular momentum (gyroscopic effect) matches the real rotor |
| Thrust | First-order lag on the command (`thrust_tau_s`), as before; not derived from shaft speed |
| Prop aerodynamic torque | **Not modelled** (counter-rotation cancels it to first order) |
| Wing/fin/pod meshes | **Visual only**; no force comes from them |
| Collision | Box + two capsules (group 3), ground contact only |
| Mass split, child inertias | **Approximate** (plate and rotor estimates); composite matches the spec exactly |
| Gazebo model | **Kinematic visual**: same meshes, pose-driven from Telemetry, elevons at neutral, props static |

## Elevon sign convention

Physical hinges are trailing-edge-up positive on both sides. Aerial Kit's
actuator vector is physically ordered:

```
[thrust_left, thrust_right, elevon_left, elevon_right]
```

It pitches up for positive `elevon_left + elevon_right` and rolls right for
positive `elevon_right - elevon_left`. The allocator, fixed-wing dynamics,
teleop mixer and MuJoCo hinge mapping all use this convention. The YAML records
`elevons.command_index: {left: 2, right: 3}`, and tests check that a right-roll
command raises the physical right trailing edge and produces a positive roll
moment.

This now matches `docs/FIRMWARE_WING_CONTROL.md`:
`elevon_left = pitch - roll`, `elevon_right = pitch + roll`.

## Regenerating and reviewing

```bash
python3 scripts/generate_twin_wing_assets.py          # Gazebo SDF/OBJ/MTL
python3 scripts/generate_twin_wing_assets.py --check  # drift check
python3 scripts/generate_twin_wing_assets.py --mjcf /tmp/twin_wing.xml
sim_py/.venv/bin/python scripts/render_twin_wing.py --flight   # MuJoCo images
python3 scripts/render_gazebo_twin_wing.py                     # Gazebo images (ROS sourced)
```

`python -m mujoco.viewer --mjcf=/tmp/twin_wing.xml` opens the model on its own;
drag the joint sliders to see the hinges and shafts move.

## Keyboard teleop

See `docs/TWIN_WING_TELEOP.md`: `ros2 launch sim_fast fixed_wing_mujoco_teleop.launch.py`
flies this model by keyboard through `uav_msgs/ActuatorCommand` and the same
servo/prop/aero path.

## Remaining simulation-to-real gaps

- Aero coefficients are research defaults; no wind-tunnel, CFD or flight-log fit.
- No spanwise loading: washout, dihedral, fins and the pod have no aerodynamic effect.
- Elevons have no hinge-moment aerodynamics, so servo load and blow-back are absent.
- Thrust is a lagged command, not a prop/motor/battery model; no advance-ratio loss.
- Prop shaft speed is scaled for display (angular momentum preserved).
- Mass distribution of child parts is estimated, not weighed.
- Gazebo shows neutral elevons and static props.
- Nothing here establishes hardware readiness.
