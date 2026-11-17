"""Keyboard teleoperation for the ROS twin wing (actuator level).

A small Tk window captures keys; key handling and the key-to-actuator mapping
are the shared, Matplotlib-free ``sim_py.teleop.input_state`` /
``sim_py.teleop.commands`` code, so control signs live in exactly one place.
The node publishes:

    /uav/backend/enable        std_msgs/Bool             armed state
    /uav/backend/cmd_actuators uav_msgs/ActuatorCommand  physical L/R order

and nothing else: no mission, no AirspeedNav. The backend adapter owns
precedence, timeout and clamping; this node only expresses the pilot's
intent. Trim comes from the same airframe spec and aero model the plant uses.

Keys: Enter arm, Backspace disarm, W/S nose down/up, A/D bank left/right,
Space/Shift throttle up/down, Q/E differential-thrust yaw, X re-centre,
P pause (trim), Esc disarm and exit.
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

import numpy as np  # noqa: E402

from sim_py.teleop.commands import (  # noqa: E402
    FixedWingTeleopTuning,
    fixedwing_command_from_keys,
    neutral_fixedwing_command,
)
from sim_py.teleop.input_state import KeyboardState  # noqa: E402

ARM_TOKENS = frozenset({'return', 'kp_enter'})
DISARM_TOKENS = frozenset({'backspace'})
EXIT_TOKENS = frozenset({'escape'})

#: Tk keysym (lowercased) -> the token names ``normalize_key`` understands.
_TK_ALIASES = {
    'shift_l': 'shift',
    'shift_r': 'shift',
    'control_l': 'ctrl',
    'control_r': 'ctrl',
    'alt_l': 'alt',
    'alt_r': 'alt',
    'kp_enter': 'return',
}

WINDOW_TITLE = 'Aerial Kit twin-wing teleop'
HELP_LINES = (
    'Enter      arm / enable manual control',
    'Backspace  disarm (plant flies its safe cruise circle)',
    'W / S      nose down / nose up',
    'A / D      bank left / right',
    'Space      throttle up (held)',
    'Shift      throttle down (held)',
    'Q / E      differential-thrust yaw left / right',
    'X          re-centre all controls to trim',
    'P          pause: hold trim while paused',
    'C          toggle viewer: chase / on-board (nose) camera',
    'Esc        disarm and exit',
    '',
    'Keys act only while this window has focus.',
    'Losing focus re-centres to trim; staying unfocused disarms.',
)


def tk_key_token(keysym: str | None) -> str | None:
    """Map a Tk ``event.keysym`` to a lowercase key token.

    Letters are lowercased so shift+W still reads as ``w`` (shift itself is
    tracked by its own press/release events).
    """
    if not keysym:
        return None
    lowered = str(keysym).lower()
    return _TK_ALIASES.get(lowered, lowered)


def tuning_from_trim(
    trim_thrust_n: float,
    trim_elevon_rad: float,
    throttle_range_n: float = 3.0,
    elevon_max_rad: float = math.radians(10.0),
    yaw_diff_n: float = 0.6,
) -> FixedWingTeleopTuning:
    """Stick tuning centred on the plant's own level-flight trim."""
    return FixedWingTeleopTuning(
        throttle_trim_n=float(trim_thrust_n),
        throttle_range_n=float(throttle_range_n),
        elevon_max_rad=float(elevon_max_rad),
        yaw_diff_n=float(yaw_diff_n),
        trim_elevon_rad=float(trim_elevon_rad),
    )


def plant_trim(airframe_spec: str = '', cruise_airspeed_mps: float = 15.0,
               airframe: str = 'twin_wing'):
    """(thrust per motor slot, common elevon) level trim from the spec + aero model.

    For the single-motor wing the ActuatorCommand's two thrust slots add up
    to its one motor, so each slot carries half the trim thrust.
    """
    if airframe == 'single_wing':
        from aerial_kit.airframes.single_wing import load_single_wing_spec
        from aerial_kit.dynamics.single_wing import SingleWingDynamics

        spec = load_single_wing_spec(airframe_spec or None)
        trim = SingleWingDynamics(spec.dynamics_params()).compute_trim_detailed(cruise_airspeed_mps)
        return 0.5 * float(trim['thrust_n']), float(trim['elevon_common_rad'])
    from aerial_kit.airframes.twin_wing import load_twin_wing_spec
    from aerial_kit.dynamics.fixed_wing import FixedWingDynamics

    spec = load_twin_wing_spec(airframe_spec or None)
    trim = FixedWingDynamics(spec.fixed_wing_params()).compute_trim(cruise_airspeed_mps)
    return float(trim[0]), float(trim[2])


@dataclass(frozen=True)
class TeleopOutput:
    active: bool
    actuator: np.ndarray       # [thrust_L, thrust_R, elevon_L, elevon_R]
    paused: bool
    focused: bool


class TeleopController:
    """Arm state + latched keys -> actuator command. No Tk, no ROS."""

    def __init__(self, tuning: FixedWingTeleopTuning, unfocused_disarm_sec: float = 3.0):
        self.tuning = tuning
        self.keys = KeyboardState()
        self.armed = False
        self.exit_requested = False
        self.unfocused_disarm_sec = float(unfocused_disarm_sec)
        self._unfocused_since: float | None = None
        self.events: list[str] = []

    def press(self, token: str | None) -> None:
        if token is None:
            return
        if token in ARM_TOKENS:
            if not self.armed:
                self.events.append('armed')
            self.armed = True
            return
        if token in DISARM_TOKENS:
            self.disarm('disarmed (Backspace)')
            return
        if token in EXIT_TOKENS:
            self.disarm('disarmed (Esc)')
            self.exit_requested = True
            self.keys.press(token)
            return
        self.keys.press(token)

    def release(self, token: str | None) -> None:
        if token is not None:
            self.keys.release(token)

    def disarm(self, reason: str = 'disarmed') -> None:
        if self.armed:
            self.events.append(reason)
        self.armed = False
        self.keys.held_keys.clear()

    def request_exit(self) -> None:
        self.disarm('disarmed (window closed)')
        self.exit_requested = True

    def set_focused(self, focused: bool, now: float) -> None:
        was = self.keys.focused
        self.keys.set_focused(focused)
        if was and not focused:
            self._unfocused_since = now
            if self.armed:
                self.events.append('focus lost: controls re-centred to trim')
        elif focused:
            self._unfocused_since = None

    def update(self, now: float) -> None:
        """Time-based safety: a long focus loss hands control back to the plant."""
        if (self.armed and self._unfocused_since is not None
                and now - self._unfocused_since >= self.unfocused_disarm_sec):
            self.disarm('disarmed: window unfocused too long')

    def output(self) -> TeleopOutput:
        if self.keys.paused or not self.keys.focused or not self.armed:
            cmd = neutral_fixedwing_command(self.tuning)
        else:
            cmd = fixedwing_command_from_keys(self.keys, tuning=self.tuning)
        return TeleopOutput(
            active=self.armed,
            actuator=np.asarray(cmd.actuator_cmd, dtype=float).copy(),
            paused=self.keys.paused,
            focused=self.keys.focused,
        )

    def drain_events(self) -> list[str]:
        events, self.events = self.events, []
        return events


class KeyRepeatFilter:
    """Hide X11 auto-repeat from the latched key state.

    Holding a key under X11 produces a stream of release/press pairs. A
    release is therefore deferred by ``delay_s`` and cancelled if the same
    key is pressed again in that window; repeated presses of a key that is
    still down are dropped (so P/Enter do not toggle repeatedly). Nothing
    depends on repeats arriving: a held key stays latched from its first
    press until its real release.
    """

    def __init__(self, schedule, cancel, delay_s: float = 0.04):
        self._schedule = schedule      # (delay_s, callback) -> handle
        self._cancel = cancel          # handle -> None
        self.delay_s = float(delay_s)
        self.down: set[str] = set()
        self._pending: dict[str, object] = {}

    def press(self, token: str) -> bool:
        """True if this is a genuine new press the controller should see."""
        handle = self._pending.pop(token, None)
        if handle is not None:
            self._cancel(handle)
        if token in self.down:
            return False
        self.down.add(token)
        return True

    def release(self, token: str, on_release) -> None:
        old = self._pending.pop(token, None)
        if old is not None:
            self._cancel(old)

        def fire():
            self._pending.pop(token, None)
            self.down.discard(token)
            on_release(token)

        self._pending[token] = self._schedule(self.delay_s, fire)

    def clear(self) -> None:
        for handle in self._pending.values():
            self._cancel(handle)
        self._pending.clear()
        self.down.clear()


def actuator_message(msg_type, output: TeleopOutput, sequence_id: int, stamp=None):
    msg = msg_type()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = 'body_frd'
    msg.sequence_id = int(sequence_id) & 0xFFFFFFFF
    msg.thrust_left_n = float(output.actuator[0])
    msg.thrust_right_n = float(output.actuator[1])
    msg.elevon_left_rad = float(output.actuator[2])
    msg.elevon_right_rad = float(output.actuator[3])
    msg.active = bool(output.active)
    return msg


def _make_node(controller_factory=None):
    """Build the ROS node (rclpy must already be initialised)."""
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from uav_msgs.msg import ActuatorCommand

    class FixedWingTeleopNode(Node):
        def __init__(self) -> None:
            super().__init__('fixed_wing_teleop_node')
            self.declare_parameter('actuator_topic', '/uav/backend/cmd_actuators')
            self.declare_parameter('enable_topic', '/uav/backend/enable')
            self.declare_parameter('publish_rate_hz', 50.0)
            self.declare_parameter('cruise_airspeed_mps', 15.0)
            self.declare_parameter('airframe_spec', '')
            self.declare_parameter('airframe', 'twin_wing')
            self.declare_parameter('throttle_range_n', 3.0)
            self.declare_parameter('elevon_max_deg', 10.0)
            self.declare_parameter('yaw_diff_n', 0.6)
            self.declare_parameter('unfocused_disarm_sec', 3.0)
            self.declare_parameter('viewer_camera_topic', '/uav/sim/viewer_camera')

            rate = float(self.get_parameter('publish_rate_hz').value)
            if not math.isfinite(rate) or rate <= 0.0:
                raise ValueError(f'publish_rate_hz must be positive, got {rate!r}')
            self.period = 1.0 / rate
            airframe = str(self.get_parameter('airframe').value)
            if airframe not in ('twin_wing', 'single_wing'):
                raise ValueError(f"airframe must be 'twin_wing' or 'single_wing', got {airframe!r}")
            thrust, elevon = plant_trim(
                str(self.get_parameter('airframe_spec').value),
                float(self.get_parameter('cruise_airspeed_mps').value), airframe)
            single = airframe == 'single_wing'
            tuning = tuning_from_trim(
                thrust, elevon,
                # Per thrust slot; the single wing adds the two slots.
                throttle_range_n=float(self.get_parameter('throttle_range_n').value) * (0.5 if single else 1.0),
                elevon_max_rad=math.radians(float(self.get_parameter('elevon_max_deg').value)),
                # One motor: Q/E would only move thrust between the slots of
                # the same motor, so they do nothing on the single wing.
                yaw_diff_n=0.0 if single else float(self.get_parameter('yaw_diff_n').value))
            self.controller = (controller_factory or TeleopController)(
                tuning, float(self.get_parameter('unfocused_disarm_sec').value))
            self.cmd_pub = self.create_publisher(
                ActuatorCommand, self.get_parameter('actuator_topic').value, 10)
            self.enable_pub = self.create_publisher(
                Bool, self.get_parameter('enable_topic').value, 10)
            self.camera_pub = self.create_publisher(
                String, self.get_parameter('viewer_camera_topic').value, 10)
            self._camera_requests = 0
            self._msg_type = ActuatorCommand
            self._bool = Bool
            self.sequence = 0
            self._last_enable = None
            self._last_enable_time = 0.0
            self.get_logger().info(
                f'twin-wing teleop ready: trim {thrust:.2f} N/motor, '
                f'{math.degrees(elevon):.2f} deg elevon; press Enter in the '
                'teleop window to arm')

        def publish_once(self, now: float | None = None) -> TeleopOutput:
            now = time.monotonic() if now is None else now
            self.controller.update(now)
            output = self.controller.output()
            self.sequence += 1
            self.cmd_pub.publish(actuator_message(
                self._msg_type, output, self.sequence,
                self.get_clock().now().to_msg()))
            # Enable on change, plus a 2 Hz heartbeat so a late subscriber
            # still learns the state.
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

        def publish_disarm(self, repeats: int = 3) -> None:
            self.controller.disarm('disarmed (shutdown)')
            for _ in range(repeats):
                self.publish_once()

    return FixedWingTeleopNode()


def _format_status(output: TeleopOutput) -> str:
    state = 'ARMED - MANUAL' if output.active else 'DISARMED - plant autopilot / safe circle'
    if output.active and output.paused:
        state = 'ARMED - PAUSED (trim)'
    if output.active and not output.focused:
        state = 'ARMED - UNFOCUSED (trim)'
    t_l, t_r, e_l, e_r = output.actuator
    return (f'{state}\n'
            f'thrust L/R  {t_l:5.2f} / {t_r:5.2f} N\n'
            f'elevon L/R  {math.degrees(e_l):+6.2f} / {math.degrees(e_r):+6.2f} deg (TE up +)')


def run_window(root, node, rclpy_module, stop_flag, title=WINDOW_TITLE,
               help_lines=HELP_LINES, format_status=None, geometry='460x400') -> None:
    """Wire Tk events to the controller and drive publishing from Tk's loop.

    Shared by the wing and multirotor teleops: ``node`` needs ``controller``
    (press/release/set_focused/request_exit/exit_requested), ``period`` and
    ``publish_once()``; ``format_status(output)`` renders the status line.
    """
    import tkinter as tk

    format_status = format_status or _format_status
    controller = node.controller
    root.title(title)
    root.geometry(geometry)
    tk.Label(root, text='\n'.join(help_lines), justify='left',
             font=('TkFixedFont', 10)).pack(padx=10, pady=(10, 4), anchor='w')
    status = tk.Label(root, text='', justify='left', font=('TkFixedFont', 11, 'bold'))
    status.pack(padx=10, pady=4, anchor='w')

    repeat = KeyRepeatFilter(
        schedule=lambda delay, cb: root.after(int(delay * 1000), cb),
        cancel=root.after_cancel)

    def on_press(event):
        token = tk_key_token(event.keysym)
        if token is not None and repeat.press(token):
            controller.press(token)

    def on_release(event):
        token = tk_key_token(event.keysym)
        if token is not None:
            repeat.release(token, controller.release)

    def on_focus(focused):
        def handler(event):
            if event.widget is root:
                if not focused:
                    repeat.clear()
                controller.set_focused(focused, time.monotonic())
        return handler

    root.bind('<KeyPress>', on_press)
    root.bind('<KeyRelease>', on_release)
    root.bind('<FocusIn>', on_focus(True))
    root.bind('<FocusOut>', on_focus(False))
    root.protocol('WM_DELETE_WINDOW', controller.request_exit)

    period_ms = max(int(node.period * 1000), 5)

    def tick():
        if stop_flag['stop'] or not rclpy_module.ok():
            controller.request_exit()
        if controller.exit_requested:
            root.quit()
            return
        rclpy_module.spin_once(node, timeout_sec=0.0)
        output = node.publish_once()
        status.config(text=format_status(output),
                      fg=('#b02020' if output.active else '#205020'))
        root.after(period_ms, tick)

    def reclaim_focus():
        # The MuJoCo viewer usually maps a moment after this window and takes
        # keyboard focus. Ask for it back once; the window manager may still
        # refuse (focus-stealing prevention), in which case click the window.
        root.lift()
        root.focus_force()

    root.after(period_ms, tick)
    root.focus_force()
    root.after(3000, reclaim_focus)
    root.mainloop()


def main(args=None) -> int:
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    # The window comes first: if there is no display nothing is published,
    # so the plant never sees an armed manual source.
    try:
        import tkinter as tk

        root = tk.Tk()
    except Exception as exc:
        print(f'fixed_wing_teleop_node: cannot open the Tk key window ({exc}). '
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
        run_window(root, node, rclpy, stop_flag)
    except Exception as exc:
        print(f'fixed_wing_teleop_node: {exc}', file=sys.stderr)
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
