#!/usr/bin/env python3
"""Convert read_board.py JSON lines into Aerial Kit body-to-world quaternions.

Firmware: FRD body, NED world, Euler angles in degrees.
Python example: FLU body, ENU world, quaternion order [w, x, y, z].
No position or velocity is invented from an attitude sample.
"""
import json
import sys
import numpy as np
from aerial_kit.dynamics.rotations import rotmat_to_quat

NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FLU_TO_FRD = np.diag([1., -1., -1.])

def orientation(sample):
    roll, pitch, yaw = np.deg2rad([sample[k] for k in ("roll_deg", "pitch_deg", "yaw_deg")])
    if not np.all(np.isfinite([roll, pitch, yaw])):
        raise ValueError("attitude must contain finite angles")
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rotmat_to_quat(NED_TO_ENU @ rz @ ry @ rx @ FLU_TO_FRD)

def main():
    for line in sys.stdin:
        if line.strip():
            sample = json.loads(line)
            print(json.dumps({"sample": sample.get("sample"), "quaternion_wxyz": orientation(sample).tolist(), "world_frame": "ENU", "body_frame": "FLU"}))

if __name__ == "__main__":
    main()
