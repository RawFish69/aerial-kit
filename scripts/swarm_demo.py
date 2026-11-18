#!/usr/bin/env python3
"""Fly an N-drone formation mission in one MuJoCo scene; print a summary, render stills.

    python3 scripts/swarm_demo.py --n 12 [--airframe swarm_quad] [--out docs/evidence/swarm-demo]

Mission: take off from a grid to 2 m, fly a circle at 3 m, a line at 2.5 m,
back to a grid at 2 m, land. Writes ``summary.json`` and, if offscreen rendering works, PNG stills
of each formation.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from sim_py.swarm import SwarmFormationSim, circle_formation, grid_formation, line_formation  # noqa: E402


def still(sim, path: Path, width=1280, height=720, close_up: bool = False):
    try:
        import mujoco
        from PIL import Image
    except Exception:
        return None
    sw = sim.swarm
    try:
        renderer = mujoco.Renderer(sw.model, height, width)
    except Exception as exc:  # no GL available
        print(f"(no render: {exc})")
        return None
    p = sw.positions()
    cam = mujoco.MjvCamera()
    if close_up:   # the drone nearest the camera side and its neighbours
        k = int(np.argmin(p[:, 0] + p[:, 1]))
        cam.lookat[:] = p[k]
        cam.distance, cam.azimuth, cam.elevation = 1.4, 225.0, -20.0
    else:
        cam.lookat[:] = [p[:, 0].mean(), p[:, 1].mean(), p[:, 2].mean()]
        spread = float(np.ptp(p[:, :2], axis=0).max()) if len(p) > 1 else 1.0
        cam.distance = max(2.5, 0.85 * spread + 1.5)
        cam.azimuth, cam.elevation = 135.0, -42.0
    renderer.update_scene(sw.data, cam)
    Image.fromarray(renderer.render()).save(path)
    renderer.close()
    return path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--airframe", default="swarm_quad")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    sim = SwarmFormationSim(n=args.n, airframe=args.airframe)
    summary = {"n": args.n, "airframe": args.airframe, "phases": {}}

    def phase(name, targets=None):
        err = sim.formation_error_m(targets) if targets is not None else None
        summary["phases"][name] = {
            "t_s": round(sim.swarm.t, 2),
            "max_slot_error_m": None if err is None else round(err, 3),
            "z_m": [round(float(z), 2) for z in sim.swarm.positions()[:, 2]],
        }
        if args.out:
            still(sim, args.out / f"{name}.png")
            if name in ("circle", "grid_on_ground"):
                still(sim, args.out / f"{name}_close.png", close_up=True)

    phase("grid_on_ground")
    sim.takeoff(2.0, 4.0)
    phase("takeoff")
    circle = sim.fly_formation(circle_formation(args.n, radius_m=max(2.0, 0.5 * args.n), z_m=3.0), 10.0)
    phase("circle", circle)
    line = sim.fly_formation(line_formation(args.n, 1.2, 2.5, heading_rad=0.5), 12.0)
    phase("line", line)
    grid = sim.fly_formation(grid_formation(args.n, 1.5, 2.0), 10.0)
    phase("grid", grid)
    sim.land(8.0)
    phase("landed")
    summary.update(
        min_separation_m=round(sim.min_separation_seen, 3),
        drone_contacts=sorted(sim.contacts_seen),
        all_finite=bool(np.all(np.isfinite(sim.swarm.data.qpos))),
        landed_disarmed=not any(sim.armed),
        sim_time_s=round(sim.swarm.t, 2),
        wall_time_s=round(time.time() - t0, 1),
    )
    text = json.dumps(summary, indent=2)
    print(text)
    if args.out:
        (args.out / "summary.json").write_text(text + "\n")
    ok = (summary["all_finite"] and summary["landed_disarmed"] and not summary["drone_contacts"]
          and all(p["max_slot_error_m"] is None or p["max_slot_error_m"] < 0.3 for p in summary["phases"].values()))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
