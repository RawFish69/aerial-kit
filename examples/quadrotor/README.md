# Quadrotor example

This example flies a quadrotor from a start point to a goal using PID position
control and the native attitude-aware multirotor dynamics backend.

From the repository root:

```bash
python -m pip install -e ".[sim]"
python examples/quadrotor/run.py
```

For interactive keyboard flight, use the short launcher:

```bash
aerial-kit-teleop
```

If Python's user Scripts directory is not on your `PATH`, the console script
above will not resolve; run `python -m aerial_kit.sim.teleop` from the repository
root instead, which needs no install.

In VS Code, you can instead open `examples/quadrotor/teleop.py` and click
**Run Python File**. That script bootstraps `sys.path` itself, so it works from
any working directory.

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

Click the plot window first. Controls are body-relative, so `Q`/`E` changes
where `W` takes you. The HUD shows `NO FOCUS` when keystrokes are not reaching
the window, and held keys are released on focus loss.

The follow camera is a third-person chase view: it stays behind the vehicle
and turns with its heading rather than watching from a fixed compass bearing.

The [fixed-wing example](../fixed_wing) has its own teleop with different,
plane-appropriate controls (pitch/bank/throttle rather than strafe/climb) --
see `examples/fixed_wing/README.md`.

Run headlessly and save the result:

```bash
python examples/quadrotor/run.py --no-show --save quadrotor-result.png
```

Edit `config.yaml` to customize the run. Useful parameters are:

- `simulation.duration` and `simulation.dt`: runtime and integration timestep.
- `simulation.multirotor.mass`, `kv_drag`, and attitude-loop gains: vehicle dynamics.
- `controller.pid.kp` and `kd`: position and velocity feedback gains.
- `path.start_relative_*` and `end_relative_*`: start and goal as fractions of
  the bundled 120 x 120 x 60 metre world.
- `visual.path_color` and `path_linewidth`: plot appearance.
- `controller.teleop.*`: teleop accelerations, velocity damping, speed limits and
  yaw rate.
- `visual.teleop_*`: follow-camera radius, quad display scale, ground grid
  spacing and trail width.

The script prints the final position, goal error, and collision count and opens
the 3D trajectory viewer unless `--no-show` is supplied.
