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
