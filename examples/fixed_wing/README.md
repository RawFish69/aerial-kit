# Twin-motor fixed-wing example

This example uses the `twin_wing` airframe, native 6-DOF fixed-wing dynamics,
Dubins planning, and the L1/TECS controller. The wing has two motors and two
elevons; yaw authority comes from differential thrust.

From the repository root:

```bash
python -m pip install -e ".[sim]"
python examples/fixed_wing/run.py
```

Run headlessly and save the result:

```bash
python examples/fixed_wing/run.py --no-show --save fixed-wing-result.png
```

## Teleop (real-time keyboard control)

```bash
python examples/fixed_wing/teleop.py
```

or from the repository root, `python -m aerial_kit.sim.teleop --airframe fixed-wing`
(the plain `aerial-kit-teleop --airframe fixed-wing` console-script form needs
the package actually pip-installed and its Scripts directory on `PATH`; the
`python -m`/script forms above need neither).

RC-plane-style controls, not the quadrotor's drone-style ones -- the wing has
no rudder, so `Q`/`E` biases differential thrust rather than yawing directly:

| Key | Action |
| --- | --- |
| `W` / `S` or `Up` / `Down` | pitch: dive / climb |
| `A` / `D` or `Left` / `Right` | bank: turn right / left |
| `Space` / `Shift` | throttle up / down |
| `Q` / `E` | differential-thrust yaw nudge |
| `X` | neutralize all commands |
| `P` | pause / resume |
| `C` | toggle follow / world camera |
| `-` / `=` | zoom the follow camera out / in |
| `H` | hide the on-screen help |
| `Esc` | exit |

The follow camera is a third-person chase view: it stays behind the aircraft
and turns with its heading rather than watching from a fixed compass bearing.
This airframe's aero model is a small-angle approximation with no rate
limiter of its own; a sustained extreme input (e.g. holding full aileron for
several seconds) can drive it into a regime the model was never fit for. The
engine detects a diverging flight and freezes rather than crashing the
window -- the HUD reports `CRASHED` when this happens, and Esc still exits
cleanly.

Important parameters in `config.yaml` include:

- `initial_state.velocity_mps` and `heading_deg`: hand-launch/cruise initial
  conditions. Do not initialize this airframe at zero airspeed.
- `simulation.fixedwing.*`: mass, geometry, inertia, and wind.
- `controller.l1_tecs.cruise_airspeed_mps`: requested cruise speed.
- `controller.l1_tecs.l1_distance_m`: lateral-guidance look-ahead distance.
- `controller.l1_tecs.tecs_*`: speed and altitude control gains.
- `path.turn_radius_m`: optional override; when omitted it comes from the
  selected airframe's capabilities.

The built-in aerodynamic coefficients are plausible research defaults, not a
fitted model of a particular aircraft. Fit these values before using results to
make real-airframe design or tuning decisions.
