#!/usr/bin/env python3
"""Render review images of the generated MuJoCo multirotor airframes.

    sim_py/.venv/bin/python scripts/render_multirotor.py [--airframe fpv_quad_5in ...]
        [--out docs/evidence/multirotor-renders]

Static views at hover with rotors at a visible angle, plus the on-board
camera (FPV / payload) view. Needs an OpenGL context.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from aerial_kit.airframes.multirotor_specs import available_airframes, load_multirotor_spec  # noqa: E402
from aerial_kit.airframes.multirotor_specs.mjcf import multirotor_mjcf  # noqa: E402


def _save(img, path: Path) -> None:
    from PIL import Image

    Image.fromarray(img).save(path)


def render(name: str, out: Path, width: int, height: int) -> list[Path]:
    import mujoco

    spec = load_multirotor_spec(name)
    model = mujoco.MjModel.from_xml_string(multirotor_mjcf(spec))
    data = mujoco.MjData(model)
    span = 2.0 * (spec.arm_radius_m() + spec.prop_radius_m)
    z = max(1.5, 3.0 * span)
    data.qpos[2] = z
    for k, rotor in enumerate(spec.rotors):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{rotor.name}_rotor_joint")
        data.qpos[model.jnt_qposadr[jid]] = 0.35 + 0.6 * k
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height, width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, z]
    views = {
        "front_three_quarter": (200.0, -22.0, 1.9 * span),
        "rear_three_quarter": (25.0, -25.0, 1.9 * span),
        "top": (180.0, -89.0, 2.0 * span),
        "side": (270.0, -5.0, 1.8 * span),
    }
    written = []
    for view, (az, el, dist) in views.items():
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        renderer.update_scene(data, cam)
        path = out / f"{name}_{view}.png"
        _save(renderer.render(), path)
        written.append(path)
    if spec.camera:
        data.qpos[2] = 1.5 if spec.role == "fpv" else 3.0
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, spec.camera["name"])
        path = out / f"{name}_onboard_{spec.camera['name']}.png"
        _save(renderer.render(), path)
        written.append(path)
    renderer.close()
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--airframe", nargs="*", default=None)
    parser.add_argument("--out", type=Path, default=REPO / "docs" / "evidence" / "multirotor-renders")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for name in args.airframe or available_airframes():
        for path in render(name, args.out, args.width, args.height):
            print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
