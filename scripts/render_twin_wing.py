#!/usr/bin/env python3
"""Render review images of the generated MuJoCo twin wing.

Static poses use the generated MJCF directly; ``--flight`` also flies the
MuJoCo backend under the shared controller and renders chase frames with the
elevons and props in whatever state the physics left them.

    sim_py/.venv/bin/python scripts/render_twin_wing.py --out docs/evidence/twin-wing-renders

Needs an OpenGL context (a display, or MUJOCO_GL=egl / osmesa).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from aerial_kit.airframes.twin_wing import load_twin_wing_spec  # noqa: E402
from aerial_kit.airframes.twin_wing.mjcf import twin_wing_mjcf  # noqa: E402
from aerial_kit.dynamics.fixed_wing import level_attitude_quat  # noqa: E402

VIEWS = {
    # name: (azimuth, elevation, distance)
    "front_three_quarter": (215.0, -22.0, 2.1),
    "rear_three_quarter": (35.0, -25.0, 2.1),
    "top": (180.0, -89.0, 2.0),
    "bottom": (180.0, 60.0, 2.0),
    "front": (180.0, -6.0, 1.9),
    "left_side": (270.0, -4.0, 1.9),
}


def _save(img: np.ndarray, path: Path) -> None:
    try:
        from PIL import Image

        Image.fromarray(img).save(path)
    except Exception:
        import matplotlib.pyplot as plt

        plt.imsave(path, img)


def _set_joints(model, data, mujoco, **angles) -> None:
    for name, value in angles.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = value


def render_static(out: Path, width: int, height: int) -> list[Path]:
    import mujoco

    model = mujoco.MjModel.from_xml_string(twin_wing_mjcf(load_twin_wing_spec()))
    data = mujoco.MjData(model)
    data.qpos[:3] = [0.0, 0.0, 3.0]
    data.qpos[3:7] = level_attitude_quat(0.0)
    renderer = mujoco.Renderer(model, height, width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, 3.0]
    written = []

    poses = {
        "neutral": {},
        # Positive = trailing edge up on both physical surfaces.
        "pitch_up": {"left_elevon_joint": 0.35, "right_elevon_joint": 0.35},
        # Right roll: right TE up, left TE down.
        "roll_right": {"left_elevon_joint": -0.35, "right_elevon_joint": 0.35},
    }
    for pose_name, angles in poses.items():
        data.qpos[7:] = 0.0
        _set_joints(model, data, mujoco, left_prop_joint=0.4, right_prop_joint=-0.4, **angles)
        mujoco.mj_forward(model, data)
        views = VIEWS if pose_name == "neutral" else {
            "rear_three_quarter": VIEWS["rear_three_quarter"],
            "left_side": (250.0, -8.0, 1.4),
            # Close-ups on each surface from behind and slightly outboard.
            "right_elevon_closeup": (20.0, -10.0, 0.75, [-0.22, -0.43, 3.0]),
            "left_elevon_closeup": (-20.0, -10.0, 0.75, [-0.22, 0.43, 3.0]),
        }
        for view, spec_view in views.items():
            az, el, dist = spec_view[:3]
            # World ENU: body +y (right wing) is world -y at heading 0.
            cam.lookat[:] = spec_view[3] if len(spec_view) > 3 else [0.0, 0.0, 3.0]
            cam.azimuth, cam.elevation, cam.distance = az, el, dist
            renderer.update_scene(data, cam)
            path = out / f"mujoco_{pose_name}_{view}.png"
            _save(renderer.render(), path)
            written.append(path)
    renderer.close()
    return written


def render_flight(out: Path, width: int, height: int, seconds: float) -> list[Path]:
    import mujoco

    from aerial_kit.airframes.fixed_wing import TwinWingAirframe
    from aerial_kit.controllers.fixed_wing import airspeed_nav_wrench
    from aerial_kit.dynamics.fixed_wing import FixedWingDynamics
    from aerial_kit.types import ControlTarget, SimState
    from sim_py.backends.mujoco_fixedwing_backend import MujocoFixedWingBackend

    spec = load_twin_wing_spec()
    params = spec.fixed_wing_params()
    backend = MujocoFixedWingBackend()
    backend.reset(
        SimState(position=np.array([0.0, 0.0, 30.0]), velocity=np.array([15.0, 0.0, 0.0]), t=0.0,
                 attitude_quat=level_attitude_quat(0.0), body_rates=np.zeros(3)),
        {}, {"simulation": {"mujoco": {"timestep": 0.002}}})
    trim = FixedWingDynamics(params).compute_trim(15.0)
    airframe = TwinWingAirframe(mass_kg=params.mass_kg)
    cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": 15.0, "trim_throttle_n": float(trim[0])}}}
    renderer = mujoco.Renderer(backend.model, height, width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_BODY, "body")
    cam.distance, cam.azimuth, cam.elevation = 2.4, 200.0, -15.0
    written = []
    steps = int(seconds / 0.01)
    for k in range(steps):
        t = k * 0.01
        bank = math.radians(30.0) if 1.0 < t < 3.0 else 0.0
        state = backend.state()
        wrench = airspeed_nav_wrench(state, 15.0, bank, 0.0, cfg)
        wrench.moment_body[1] += float(trim[2])
        cmd = airframe.allocate(wrench, state)
        backend.step(ControlTarget(accel_cmd=np.zeros(3), metadata={"actuator_cmd": cmd}), 0.01)
        if k in (int(0.8 / 0.01), int(1.25 / 0.01), int(2.5 / 0.01)):
            renderer.update_scene(backend.data, cam)
            path = out / f"mujoco_flight_chase_t{t:.2f}.png"
            _save(renderer.render(), path)
            written.append(path)
        if k == int(1.25 / 0.01):
            renderer.update_scene(backend.data, "chase")
            path = out / "mujoco_flight_chase_camera.png"
            _save(renderer.render(), path)
            written.append(path)
    renderer.close()
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=REPO / "docs" / "evidence" / "twin-wing-renders")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--flight", action="store_true", help="also render chase frames in flight")
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    written = render_static(args.out, args.width, args.height)
    if args.flight:
        written += render_flight(args.out, args.width, args.height, args.seconds)
    for path in written:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
