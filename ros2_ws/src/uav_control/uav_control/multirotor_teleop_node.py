"""Keyboard teleoperation for the ROS MuJoCo multirotors (quad / hex / octo).

Same Tk key window, key repeat filter and safety behaviour as the twin-wing
teleop (``fixed_wing_teleop_node``); key-to-axis handling is the shared
``sim_py.teleop`` code. Publishes:

    /uav/backend/enable   std_msgs/Bool
    /uav/backend/manual   uav_msgs/MultirotorManualCommand   (mode + sticks)

and nothing else. The plant's flight mode does the control (velocity /
angle / acro); this node only expresses stick positions.

Keys: Enter arm, Backspace disarm, W/S forward/back (pitch), A/D left/right
(roll), Space/Shift up/down (throttle or climb), Q/E yaw left/right,
M cycle mode, X re-centre, P pause (centred sticks), Esc disarm and exit.
"""

from __future__ import annotations

import math
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path

for _parent in Path(__file__).resolve().parents:
    if (_parent / 'aerial_kit').is_dir() and (_parent / 'pyproject.toml').is_file():
        if str(_parent) not in sys.path:
            sys.path.insert(0, str(_parent))
        break

from sim_py.teleop.commands import axes_from_keys  # noqa: E402
from sim_py.teleop.input_state import KeyboardState  # noqa: E402

from uav_control.fixed_wing_teleop_node import (  # noqa: E402
    ARM_TOKENS,
    DISARM_TOKENS,
    EXIT_TOKENS,
    run_window,
)

MODES = ('velocity', 'angle', 'acro')
MODE_KEYS = frozenset({'m'})
WINDOW_TITLE = 'Aerial Kit multirotor teleop'
HELP_LINES = (
    'Enter      arm / enable manual control',
    'Backspace  disarm (plant holds position, idles on the ground)',
    'W / S      forward / back   (pitch)',
    'A / D      left / right     (roll)',
    'Space      up   (climb; acro: throttle, held)',
    'Shift      down (descend; acro: throttle, held)',
    'Q / E      yaw left / right',
    'M          cycle mode: velocity -> angle -> acro',
    'X          re-centre all sticks',
    'P          pause: centred sticks (hover / hold)',
    'C          toggle viewer: chase / on-board (FPV) camera',
    'Esc        disarm and exit',
    '',
    'velocity: sticks = speed, centred = hold position',
    'angle:    sticks = tilt, centred = level, holds altitude',
    'acro:     sticks = rotation rate, raw throttle; nothing self-levels',
    'Keys act only while this window has focus.',
)


def default_mode(role: str) -> str:
    return 'angle' if role == 'fpv' else 'velocity'


@dataclass(frozen=True)
class MultirotorTeleopOutput:
    active: bool
    mode: str
    roll: float
    pitch: float
    yaw: float
    throttle: float
    paused: bool
    focused: bool


class MultirotorTeleopController:
    """Arm state, mode and latched keys -> stick positions. No Tk, no ROS."""

    def __init__(self, mode: str = 'velocity', throttle_step: float = 0.35,
                 unfocused_disarm_sec: float = 3.0, acro_throttle_step: float = 0.06):
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}, got {mode!r}')
        if not 0.0 < throttle_step <= 0.5:
            raise ValueError('throttle_step must be in (0, 0.5]')
        self.mode = mode
        self.throttle_step = float(throttle_step)
        # Acro throttle is the raw collective: on a high-TWR quad even a small
        # step above hover is a strong climb, so keys move it gently.
        self.acro_throttle_step = float(acro_throttle_step)
        self.keys = KeyboardState()
        self.armed = False
        self.exit_requested = False
        self.unfocused_disarm_sec = float(unfocused_disarm_sec)
        self._unfocused_since = None
        self.events: list[str] = []

    def press(self, token):
        if token is None:
            return
        if token in ARM_TOKENS:
            if not self.armed:
                self.events.append(f'armed ({self.mode} mode)')
            self.armed = True
            return
        if token in DISARM_TOKENS:
            self.disarm('disarmed (Backspace)')
            return
        if token in EXIT_TOKENS:
            self.disarm('disarmed (Esc)')
            self.exit_requested = True
            return
        if token in MODE_KEYS:
            self.mode = MODES[(MODES.index(self.mode) + 1) % len(MODES)]
            self.keys.neutralize()
            self.events.append(f'mode: {self.mode}')
            return
        self.keys.press(token)

    def release(self, token):
        if token is not None:
            self.keys.release(token)

    def disarm(self, reason='disarmed'):
        if self.armed:
            self.events.append(reason)
        self.armed = False
        self.keys.held_keys.clear()

    def request_exit(self):
        self.disarm('disarmed (window closed)')
        self.exit_requested = True

    def set_focused(self, focused, now):
        was = self.keys.focused
        self.keys.set_focused(focused)
        if was and not focused:
            self._unfocused_since = now
            if self.armed:
                self.events.append('focus lost: sticks centred')
        elif focused:
            self._unfocused_since = None

    def update(self, now):
        if (self.armed and self._unfocused_since is not None
                and now - self._unfocused_since >= self.unfocused_disarm_sec):
            self.disarm('disarmed: window unfocused too long')

    def output(self) -> MultirotorTeleopOutput:
        centred = self.keys.paused or not self.keys.focused or not self.armed
        if centred:
            roll = pitch = yaw = 0.0
            throttle = 0.5
        else:
            axes = axes_from_keys(self.keys)
            pitch, roll = float(axes.forward), float(axes.right)
            # sim_py's yaw axis is +1 for Q (yaw left); the stick is +1 for nose right.
            yaw = -float(axes.yaw)
            step = self.acro_throttle_step if self.mode == 'acro' else self.throttle_step
            throttle = 0.5 + step * float(axes.up)
        return MultirotorTeleopOutput(self.armed, self.mode, roll, pitch, yaw, throttle,
                                      self.keys.paused, self.keys.focused)

    def drain_events(self):
        events, self.events = self.events, []
        return events


def manual_message(msg_type, output: MultirotorTeleopOutput, sequence_id: int, stamp=None):
    msg = msg_type()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = 'body_flu'
    msg.sequence_id = int(sequence_id) & 0xFFFFFFFF
    msg.mode = {'velocity': msg_type.MODE_VELOCITY, 'angle': msg_type.MODE_ANGLE,
                'acro': msg_type.MODE_ACRO}[output.mode]
    msg.roll, msg.pitch, msg.yaw, msg.throttle = (float(output.roll), float(output.pitch),
                                                  float(output.yaw), float(output.throttle))
    msg.active = bool(output.active)
    return msg


def format_status(output: MultirotorTeleopOutput) -> str:
    state = f'ARMED - {output.mode.upper()}' if output.active else 'DISARMED - plant holds / idles'
    if output.active and output.paused:
        state += ' (PAUSED)'
    if output.active and not output.focused:
        state += ' (UNFOCUSED)'
    return (f'{state}\n'
            f'roll {output.roll:+.2f}  pitch {output.pitch:+.2f}  yaw {output.yaw:+.2f}\n'
            f'throttle {output.throttle:.2f}  (0.50 = hover / hold)')


def _make_node():
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from uav_msgs.msg import MultirotorManualCommand

    from aerial_kit.airframes.multirotor_specs import load_multirotor_spec

    class MultirotorTeleopNode(Node):
        def __init__(self):
            super().__init__('multirotor_teleop_node')
            self.declare_parameter('manual_topic', '/uav/backend/manual')
            self.declare_parameter('enable_topic', '/uav/backend/enable')
            self.declare_parameter('publish_rate_hz', 50.0)
            self.declare_parameter('airframe', 'fpv_quad_5in')
            self.declare_parameter('mode', 'auto')
            self.declare_parameter('throttle_step', 0.35)
            self.declare_parameter('unfocused_disarm_sec', 3.0)
            self.declare_parameter('viewer_camera_topic', '/uav/sim/viewer_camera')
            rate = float(self.get_parameter('publish_rate_hz').value)
            if not math.isfinite(rate) or rate <= 0.0:
                raise ValueError(f'publish_rate_hz must be positive, got {rate!r}')
            self.period = 1.0 / rate
            spec = load_multirotor_spec(str(self.get_parameter('airframe').value))
            mode = str(self.get_parameter('mode').value).lower()
            if mode == 'auto':
                mode = default_mode(spec.role)
            self.controller = MultirotorTeleopController(
                mode, float(self.get_parameter('throttle_step').value),
                float(self.get_parameter('unfocused_disarm_sec').value))
            self._msg = MultirotorManualCommand
            self._bool = Bool
            self.cmd_pub = self.create_publisher(
                MultirotorManualCommand, self.get_parameter('manual_topic').value, 10)
            self.enable_pub = self.create_publisher(Bool, self.get_parameter('enable_topic').value, 10)
            self.camera_pub = self.create_publisher(
                String, self.get_parameter('viewer_camera_topic').value, 10)
            self._camera_requests = 0
            self.sequence = 0
            self._last_enable = None
            self._last_enable_time = 0.0
            self.get_logger().info(
                f'multirotor teleop ready for {spec.name} in {mode} mode; '
                'press Enter in the teleop window to arm')

        def publish_once(self, now=None):
            now = time.monotonic() if now is None else now
            self.controller.update(now)
            output = self.controller.output()
            self.sequence += 1
            self.cmd_pub.publish(manual_message(self._msg, output, self.sequence,
                                                self.get_clock().now().to_msg()))
            if output.active != self._last_enable or now - self._last_enable_time > 0.5:
                self.enable_pub.publish(self._bool(data=output.active))
                self._last_enable = output.active
                self._last_enable_time = now
            for event in self.controller.drain_events():
                self.get_logger().info(event)
            requests = self.controller.keys.camera_toggle_requests
            while self._camera_requests < requests:
                self._camera_requests += 1
                self.camera_pub.publish(String(data='next'))
            return output

        def publish_disarm(self, repeats=3):
            self.controller.disarm('disarmed (shutdown)')
            for _ in range(repeats):
                self.publish_once()

    return MultirotorTeleopNode()


def main(args=None) -> int:
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    try:
        import tkinter as tk

        root = tk.Tk()
    except Exception as exc:
        print(f'multirotor_teleop_node: cannot open the Tk key window ({exc}). '
              'Teleop needs a display; nothing was armed. Exiting.', file=sys.stderr)
        return 1

    stop_flag = {'stop': False}

    def _stop(signum, frame):
        stop_flag['stop'] = True

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    node = None
    try:
        node = _make_node()
        run_window(root, node, rclpy, stop_flag, title=WINDOW_TITLE, help_lines=HELP_LINES,
                   format_status=format_status, geometry='470x450')
    except Exception as exc:
        print(f'multirotor_teleop_node: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            try:
                node.publish_disarm()
            except Exception:
                pass
            node.get_logger().info('teleop exited; manual control released')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            root.destroy()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
