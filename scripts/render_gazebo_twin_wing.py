#!/usr/bin/env python3
"""Capture review images of the generated Gazebo twin-wing visual.

Builds a throwaway world (temp dir) with the twin_wing_visual model posed
level and a few saving camera sensors, runs ``gz sim -s`` headless with
ogre2 for a few seconds, and copies one frame per camera to ``--out``.
The shipped world is not modified.

    source ~/.local/share/ros/jazzy/setup.bash
    python3 scripts/render_gazebo_twin_wing.py --out docs/evidence/twin-wing-renders
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
MODELS = REPO / "ros2_ws" / "src" / "sim_gazebo" / "models"

# name: camera position (world ENU) looking at the aircraft at (0, 0, 2).
CAMERAS = {
    "front_three_quarter": (1.6, 1.3, 2.9),
    "rear_three_quarter": (-1.7, -1.2, 2.9),
    "top": (0.0, 0.0, 4.4),
    "below": (0.4, -0.5, 0.25),
}


def _look_at_rpy(eye, target):
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    return 0.0, pitch, yaw


def _world(width: int, height: int, frames_dir: Path) -> str:
    cams = []
    for name, eye in CAMERAS.items():
        r, p, y = _look_at_rpy(eye, (0.0, 0.0, 2.0))
        if name == "top":
            p, y = math.pi / 2 - 1e-3, math.pi
        cams.append(f"""
    <model name="cam_{name}">
      <static>true</static>
      <pose>{eye[0]} {eye[1]} {eye[2]} {r} {p} {y}</pose>
      <link name="link">
        <sensor name="cam_{name}" type="camera">
          <update_rate>2</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>1.0</horizontal_fov>
            <image><width>{width}</width><height>{height}</height></image>
            <clip><near>0.05</near><far>200</far></clip>
            <save enabled="true"><path>{frames_dir / name}</path></save>
          </camera>
        </sensor>
      </link>
    </model>""")
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="twin_wing_review">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.70 0.80 0.92 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.50 0.56 0.47 1</ambient><diffuse>0.55 0.62 0.52 1</diffuse></material>
        </visual>
      </link>
    </model>
    <include>
      <name>twin_wing</name>
      <!-- Level flight, heading +x: body FRD needs a 180 deg roll into ENU. -->
      <pose>0 0 2 {math.pi} 0 0</pose>
      <uri>model://twin_wing_visual</uri>
    </include>{''.join(cams)}
  </world>
</sdf>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=REPO / "docs" / "evidence" / "twin-wing-renders")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--headless-rendering", action="store_true",
                        help="use EGL instead of the X display")
    args = parser.parse_args()
    if shutil.which("gz") is None:
        print("gz not found; source the ROS/Gazebo setup first")
        return 1
    args.out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="twin_wing_gz_") as tmp:
        tmp = Path(tmp)
        frames = tmp / "frames"
        frames.mkdir()
        world = tmp / "review.sdf"
        world.write_text(_world(args.width, args.height, frames), encoding="utf-8")
        env = dict(os.environ)
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
            [str(MODELS), env.get("GZ_SIM_RESOURCE_PATH", "")])
        cmd = ["gz", "sim", "-s", "-r", str(world)]
        if args.headless_rendering:
            cmd.insert(3, "--headless-rendering")
        server = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
        try:
            # Camera sensors only render (and save) while someone subscribes.
            deadline = time.monotonic() + args.seconds
            for name in CAMERAS:
                topic = (f"/world/twin_wing_review/model/cam_{name}/link/link/"
                         f"sensor/cam_{name}/image")
                remaining = max(deadline - time.monotonic(), 3.0)
                try:
                    subprocess.run(["gz", "topic", "-e", "-n", "3", "-t", topic], env=env,
                                   timeout=remaining, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
                except subprocess.TimeoutExpired:
                    pass
        finally:
            server.terminate()
            try:
                server.wait(timeout=10)
            except subprocess.TimeoutExpired:
                server.kill()
        written = 0
        for name in CAMERAS:
            shots = sorted((frames / name).glob("*.png")) if (frames / name).is_dir() else []
            if not shots:
                print(f"no frame from camera {name}")
                continue
            dest = args.out / f"gazebo_{name}.png"
            shutil.copy(shots[-1], dest)
            print(dest)
            written += 1
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())
