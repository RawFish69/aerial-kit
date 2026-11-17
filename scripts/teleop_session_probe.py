#!/usr/bin/env python3
"""Drive a live twin-wing teleop session with real X key events and measure it.

Starts nothing itself: run it while ``fixed_wing_mujoco_teleop.launch.py``
is up (ROS sourced, X display available). It finds the teleop Tk window,
sends key presses/releases through the XTEST extension (the same path a
physical keyboard takes into the X server), records
``/uav/backend/cmd_actuators`` and ``/uav/backend/telemetry_raw``, and prints
a per-phase JSON summary: control source, commanded actuators, mean body
rates. Optional screenshots of the MuJoCo viewer window are saved per phase.

This automates the key path and the measured response. Whether the motion
*looks* right in the viewer is still a human judgement.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from uav_msgs.msg import ActuatorCommand, Telemetry
from Xlib import X, XK, display
from Xlib.ext import xtest

TELEOP_TITLE = 'Aerial Kit twin-wing teleop'
VIEWER_TITLE_PREFIX = 'MuJoCo'

# (phase name, keysym to hold or tap, hold seconds, settle seconds after).
# Open-loop actuator control does not level itself: each deflection is
# followed by the opposite one, the way a pilot flies it, so the aircraft
# stays airborne for the whole script.
SCRIPT = [
    ('disarmed_start', None, 0.0, 3.0),
    ('arm', 'Return', 0.0, 1.0),
    ('trim_hold', None, 0.0, 1.5),
    ('bank_right_D', 'd', 0.35, 0.0),
    ('counter_A', 'a', 0.35, 0.0),
    ('recentre_X', 'x', 0.0, 1.5),
    ('bank_left_A', 'a', 0.35, 0.0),
    ('counter_D', 'd', 0.35, 0.0),
    ('recentre_X2', 'x', 0.0, 1.5),
    ('nose_up_S', 's', 0.3, 0.0),
    ('counter_W', 'w', 0.3, 0.0),
    ('recentre_X3', 'x', 0.0, 1.5),
    ('nose_down_W', 'w', 0.3, 0.0),
    ('counter_S', 's', 0.3, 0.0),
    ('recentre_X4', 'x', 0.0, 1.5),
    ('throttle_up_Space', 'space', 1.5, 0.0),
    ('throttle_down_Shift', 'Shift_L', 0.8, 0.0),
    ('trim_after_throttle', None, 0.0, 1.0),
    ('yaw_Q', 'q', 1.0, 0.0),
    ('yaw_E', 'e', 1.0, 0.0),
    ('pause_P', 'p', 0.0, 0.0),
    ('paused_D_held', 'd', 1.0, 0.0),
    ('unpause_P', 'p', 0.0, 1.0),
    ('disarm_Backspace', 'BackSpace', 0.0, 3.0),
    ('rearm', 'Return', 0.0, 1.0),
    ('focus_loss', '__unfocus__', 0.0, 4.0),
    ('refocus_rearm', '__refocus_arm__', 0.0, 1.5),
    ('exit_Esc', 'Escape', 0.0, 2.0),
]


class Recorder(Node):
    def __init__(self):
        super().__init__('teleop_session_probe')
        self.phase = 'startup'
        self.cmds, self.tel = [], []
        self.create_subscription(ActuatorCommand, '/uav/backend/cmd_actuators', self._cmd, 50)
        self.create_subscription(Telemetry, '/uav/backend/telemetry_raw', self._tel, 50)

    def _cmd(self, m):
        self.cmds.append((self.phase, m.active, m.thrust_left_n, m.thrust_right_n,
                          m.elevon_left_rad, m.elevon_right_rad))

    def _tel(self, m):
        self.tel.append((self.phase, m.status_text.split(': ')[-1], m.twist.angular.x,
                         m.twist.angular.y, m.twist.angular.z, m.pose.position.z,
                         m.airspeed_mps))


def find_window(d, predicate, wm_class=None):
    """Find a *client* window by title.

    Window-manager frames (e.g. mutter-x11-frames) carry the same title;
    keys sent to a frame never reach the application, so match WM_CLASS too.
    """
    stack = [d.screen().root]
    while stack:
        w = stack.pop()
        try:
            name = w.get_wm_name()
            cls = w.get_wm_class() or ()
            if name and predicate(str(name)) and (wm_class is None or wm_class in cls):
                return w
            stack.extend(w.query_tree().children)
        except Exception:
            continue
    return None


def key(d, keysym, down):
    code = d.keysym_to_keycode(XK.string_to_keysym(keysym))
    xtest.fake_input(d, X.KeyPress if down else X.KeyRelease, code)
    d.sync()


def focus(d, win):
    win.raise_window()
    win.set_input_focus(X.RevertToParent, X.CurrentTime)
    d.sync()


def mean(xs):
    return round(sum(xs) / len(xs), 4) if xs else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--shots', type=Path, default=None)
    args = parser.parse_args()

    d = display.Display()
    rclpy.init()
    rec = Recorder()

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(rec, timeout_sec=0.01)

    teleop = None
    for _ in range(100):
        teleop = find_window(d, lambda n: n == TELEOP_TITLE, wm_class='Tk')
        if teleop is not None:
            break
        spin(0.2)
    if teleop is None:
        print('teleop window not found')
        return 1
    viewer = None
    for _ in range(50):
        viewer = find_window(d, lambda n: n.startswith(VIEWER_TITLE_PREFIX) and 'teleop' not in n)
        if viewer is not None and 'mutter' not in str(viewer.get_wm_class() or ''):
            break
        spin(0.2)
    focus(d, teleop)
    spin(1.0)

    def shot(name):
        if args.shots is None or viewer is None:
            return
        try:
            from PIL import ImageGrab

            g = viewer.get_geometry()
            pos = viewer.translate_coords(d.screen().root, 0, 0)
            x, y = -pos.x, -pos.y
            ImageGrab.grab(bbox=(x, y, x + g.width, y + g.height), xdisplay=None).save(
                args.shots / f'teleop_{name}.png')
        except Exception as exc:
            print(f'screenshot {name} failed: {exc}')

    args.shots and args.shots.mkdir(parents=True, exist_ok=True)
    for phase, keysym, hold, settle in SCRIPT:
        rec.phase = phase
        if keysym == '__unfocus__':
            target = viewer or d.screen().root
            focus(d, target)
        elif keysym == '__refocus_arm__':
            focus(d, teleop)
            spin(0.3)
            key(d, 'Return', True)
            key(d, 'Return', False)
        elif keysym is not None:
            key(d, keysym, True)
            if hold > 0.0:
                spin(hold * 0.6)
                if phase in ('bank_right_D', 'bank_left_A', 'nose_up_S', 'throttle_up_Space'):
                    shot(phase)
                spin(hold * 0.4)
            key(d, keysym, False)
        spin(max(settle, 0.05))

    summary = {}
    for phase, *_ in SCRIPT:
        cmds = [c for c in rec.cmds if c[0] == phase]
        tel = [t for t in rec.tel if t[0] == phase]
        summary[phase] = {
            'cmd_samples': len(cmds),
            'active_fraction': mean([float(c[1]) for c in cmds]),
            'thrust_L_R': [mean([c[2] for c in cmds]), mean([c[3] for c in cmds])],
            'elevon_L_R_rad': [mean([c[4] for c in cmds]), mean([c[5] for c in cmds])],
            'sources': sorted({t[1] for t in tel}),
            'mean_pqr': [mean([t[2] for t in tel]), mean([t[3] for t in tel]),
                         mean([t[4] for t in tel])],
            'z_end': round(tel[-1][5], 2) if tel else None,
            'airspeed_end': round(tel[-1][6], 2) if tel else None,
        }
    finite = all(all(v == v for v in t[2:]) for t in rec.tel)
    summary['_all_telemetry_finite'] = finite
    summary['_telemetry_samples'] = len(rec.tel)
    text = json.dumps(summary, indent=1)
    args.out.write_text(text + '\n', encoding='utf-8')
    print(text)
    rec.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
