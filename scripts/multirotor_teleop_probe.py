#!/usr/bin/env python3
"""Drive a live multirotor teleop session with real X key events and measure it.

Run while ``multirotor_mujoco_teleop.launch.py`` is up (ROS sourced, X display,
python-xlib importable). Sends XTEST key events to the Tk teleop window and
records /uav/backend/manual and /uav/backend/odom per phase; prints JSON.
Automates the key path and the measured response, not the visual judgement.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from uav_msgs.msg import MultirotorManualCommand
from Xlib import display

sys.path.insert(0, str(Path(__file__).resolve().parent))
from teleop_session_probe import find_window, focus, key  # noqa: E402

TITLE = 'Aerial Kit multirotor teleop'

# (phase, keysym or None, hold s, settle s)
SCRIPT = [
    ('disarmed_on_ground', None, 0.0, 2.0),
    ('arm', 'Return', 0.0, 1.0),
    ('climb_Space', 'space', 2.0, 0.0),
    ('settle', None, 0.0, 2.0),
    ('forward_W', 'w', 1.0, 0.0),
    ('recentre_X', 'x', 0.0, 2.0),
    ('right_D', 'd', 1.0, 0.0),
    ('recentre_X2', 'x', 0.0, 2.0),
    ('yaw_right_E', 'e', 1.0, 0.0),
    ('yaw_left_Q', 'q', 1.0, 0.0),
    ('settle2', None, 0.0, 1.5),
    ('mode_M', 'm', 0.0, 0.5),
    ('mode2_right_D', 'd', 0.15, 0.0),
    ('mode2_left_A', 'a', 0.15, 0.0),
    ('recentre_X3', 'x', 0.0, 2.0),
    ('pause_P', 'p', 0.0, 0.0),
    ('paused_W_held', 'w', 1.0, 0.0),
    ('unpause_P', 'p', 0.0, 1.0),
    ('disarm_Backspace', 'BackSpace', 0.0, 4.0),
    ('rearm', 'Return', 0.0, 1.0),
    ('focus_loss', '__unfocus__', 0.0, 4.0),
    ('exit_Esc', '__refocus_exit__', 0.0, 2.0),
]


class Rec(Node):
    def __init__(self):
        super().__init__('multirotor_teleop_probe')
        self.phase = 'startup'
        self.cmd, self.odom = [], []
        self.create_subscription(MultirotorManualCommand, '/uav/backend/manual', self._c, 50)
        self.create_subscription(Odometry, '/uav/backend/odom', self._o, 50)

    def _c(self, m):
        self.cmd.append((self.phase, m.active, m.mode, m.roll, m.pitch, m.yaw, m.throttle))

    def _o(self, m):
        p, v, w, q = m.pose.pose.position, m.twist.twist.linear, m.twist.twist.angular, m.pose.pose.orientation
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2)))
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
        self.odom.append((self.phase, p.x, p.y, p.z, v.x, v.y, v.z, w.z, roll, yaw))


def mean(xs):
    return round(sum(xs) / len(xs), 3) if xs else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--shots', default='')
    a = ap.parse_args()
    d = display.Display()
    rclpy.init()
    rec = Rec()

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(rec, timeout_sec=0.01)

    win = None
    for _ in range(150):
        win = find_window(d, lambda n: n == TITLE, wm_class='Tk')
        if win:
            break
        spin(0.2)
    if win is None:
        print('teleop window not found')
        return 1
    spin(3.0)   # let the viewer open, then take focus back
    focus(d, win)
    spin(1.0)
    viewer = find_window(d, lambda n: n.startswith('MuJoCo'))

    def shot(name):
        if not a.shots or viewer is None:
            return
        try:
            from PIL import ImageGrab

            g = viewer.get_geometry()
            pos = viewer.translate_coords(d.screen().root, 0, 0)
            Path(a.shots).mkdir(parents=True, exist_ok=True)
            ImageGrab.grab(bbox=(-pos.x, -pos.y, -pos.x + g.width, -pos.y + g.height)).save(
                Path(a.shots) / f'{name}.png')
        except Exception as exc:
            print(f'screenshot {name} failed: {exc}')

    for phase, ks, hold, settle in SCRIPT:
        rec.phase = phase
        if ks == '__unfocus__':
            focus(d, d.screen().root)
        elif ks == '__refocus_exit__':
            focus(d, win)
            spin(0.3)
            key(d, 'Escape', True)
            key(d, 'Escape', False)
        elif ks:
            key(d, ks, True)
            spin(hold if hold > 0 else 0.05)
            if phase in ('climb_Space', 'forward_W', 'right_D', 'mode2_right_D'):
                shot(phase)
            key(d, ks, False)
        spin(max(settle, 0.05))
    out = {}
    for phase, *_ in SCRIPT:
        c = [x for x in rec.cmd if x[0] == phase]
        o = [x for x in rec.odom if x[0] == phase]
        out[phase] = {
            'active_frac': mean([float(x[1]) for x in c]),
            'mode': sorted({int(x[2]) for x in c}),
            'sticks_rpy_thr': [mean([x[i] for x in c]) for i in (3, 4, 5, 6)],
            'z_end': round(o[-1][3], 2) if o else None,
            'v_end': [round(o[-1][i], 2) for i in (4, 5, 6)] if o else None,
            'mean_yaw_rate': mean([x[7] for x in o]),
            'max_abs_roll_deg': round(max(abs(x[8]) for x in o), 1) if o else None,
            'yaw_end_deg': round(o[-1][9], 1) if o else None,
        }
    out['_odom_finite'] = all(all(math.isfinite(v) for v in x[1:]) for x in rec.odom)
    out['_odom_samples'] = len(rec.odom)
    text = json.dumps(out, indent=1)
    Path(a.out).write_text(text + '\n')
    print(text)
    rec.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
