# Controller benchmarks

Date: 2026-11-18

Command:

```bash
python -m aerial_kit.benchmarks --output docs/evidence/controller-benchmarks.md
```

The same 12 s hold/step/gust and 15 s figure-eight references are used for
every horizontal simulator family. Position controllers fly
`PointMassDynamics`; wrench controllers fly the motor-level quad-X
`ActuatorPlant`; wing controllers fly their existing twin/single-wing 6-DOF
models. Because a fixed wing cannot hover, its "hover hold" reference advances
at 15 m/s, and its step/gust references preserve the same lateral offsets.
Effort is mean acceleration command for the point-mass family, mean normalized
motor command for quad-X, and mean absolute actuator command for wings. These
are research-default models, not aircraft-specific tuning claims.

| Controller | Family | Scenario | RMS error (m) | Final error (m) | Effort | Finite | Note |
| --- | --- | --- | ---: | ---: | ---: | :---: | --- |
| pid | accel | hover_hold | 0.000 | 0.000 | 0.000 | yes |  |
| pid | accel | step | 1.541 | 0.001 | 0.329 | yes |  |
| pid | accel | figure8 | 3.076 | 2.772 | 0.894 | yes |  |
| pid | accel | wind_gust | 2.100 | 2.780 | 1.853 | yes |  |
| lqr | accel | hover_hold | 0.000 | 0.000 | 0.000 | yes |  |
| lqr | accel | step | 1.300 | 0.000 | 0.563 | yes |  |
| lqr | accel | figure8 | 1.821 | 1.346 | 1.049 | yes |  |
| lqr | accel | wind_gust | 0.636 | 0.791 | 1.869 | yes |  |
| mpc | accel | hover_hold | 0.000 | 0.000 | 0.000 | yes |  |
| mpc | accel | step | 2.069 | 0.443 | 0.282 | yes |  |
| mpc | accel | figure8 | 5.246 | 6.287 | 0.919 | yes |  |
| mpc | accel | wind_gust | 8.510 | 12.173 | 1.839 | yes |  |
| mppi | accel | hover_hold | 0.320 | 0.346 | 0.079 | yes |  |
| mppi | accel | step | 1.572 | 0.342 | 0.983 | yes |  |
| mppi | accel | figure8 | 1.523 | 1.178 | 1.201 | yes |  |
| mppi | accel | wind_gust | 18.907 | 46.285 | 1.060 | yes |  |
| constrained_mpc | accel | hover_hold | 0.000 | 0.000 | 0.000 | yes |  |
| constrained_mpc | accel | step | 60.295 | 143.014 | 3.334 | yes |  |
| constrained_mpc | accel | figure8 | 1.703 | 1.226 | 1.071 | yes |  |
| constrained_mpc | accel | wind_gust | 31.376 | 77.145 | 0.000 | yes |  |
| warm_mppi | accel | hover_hold | 1.409 | 2.964 | 0.059 | yes |  |
| warm_mppi | accel | step | 58.204 | 138.216 | 3.245 | yes |  |
| warm_mppi | accel | figure8 | 1.560 | 1.146 | 1.869 | yes |  |
| warm_mppi | accel | wind_gust | 30.636 | 75.547 | 0.059 | yes |  |
| cascade | wrench | hover_hold | 0.060 | 0.000 | 0.410 | yes |  |
| cascade | wrench | step | 1.632 | 0.003 | 0.411 | yes |  |
| cascade | wrench | figure8 | 3.197 | 2.700 | 0.412 | yes |  |
| cascade | wrench | wind_gust | 0.251 | 0.312 | 0.410 | yes |  |
| geometric | wrench | hover_hold | 0.011 | 0.000 | 0.410 | yes |  |
| geometric | wrench | step | 1.261 | 0.000 | 0.419 | yes |  |
| geometric | wrench | figure8 | 1.431 | 1.012 | 0.413 | yes |  |
| geometric | wrench | wind_gust | 0.035 | 0.040 | 0.410 | yes |  |
| nmpc | wrench | hover_hold | 0.007 | 0.000 | 0.410 | yes |  |
| nmpc | wrench | step | 1.183 | 0.000 | 0.425 | yes |  |
| nmpc | wrench | figure8 | 1.391 | 0.954 | 0.413 | yes |  |
| nmpc | wrench | wind_gust | 0.042 | 0.050 | 0.410 | yes |  |
| l1_tecs | wing | hover_hold | 19.802 | 36.326 | 0.795 | yes |  |
| l1_tecs | wing | step | 21.855 | 44.967 | 0.848 | yes |  |
| l1_tecs | wing | figure8 | 126.448 | 126.829 | 0.802 | yes |  |
| l1_tecs | wing | wind_gust | 28.657 | 55.673 | 0.861 | yes |  |
| single_wing_l1_tecs | wing | hover_hold | 14.117 | 26.687 | 0.679 | yes |  |
| single_wing_l1_tecs | wing | step | 19.753 | 42.286 | 0.735 | yes |  |
| single_wing_l1_tecs | wing | figure8 | 119.431 | 117.305 | 0.694 | yes |  |
| single_wing_l1_tecs | wing | wind_gust | 22.392 | 44.894 | 0.739 | yes |  |

The table is diagnostic rather than a ranking. The large constrained-MPC and
warm-MPPI step/gust errors are retained rather than hidden; both controllers
are designed around a horizon reference and need follow-up comparison through
that interface. The wing figure-eight errors expose the minimum-turn and
reference-path limits of the current research defaults. No hardware tuning is
implied.
