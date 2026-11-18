"""Firmware-in-the-loop: the AK flight core flies the MuJoCo airframes.

:class:`AkFlightCore` is a ctypes binding over ``akfc_shim.c`` and the
unmodified firmware sources (built by :mod:`sim_py.fc_sil.build`).
:class:`FirmwareInTheLoop` closes the loop at the firmware's 1 kHz rate:

    MuJoCo state -> IMU sample (FRD, rad/s, g) ---> ak_flight_step()
    sticks -> receiver counts ---------------------^        |
    MuJoCo rotors <- thrust = T_max * u^2 <- motor[i] (0..1) <-+

The control law, mixer, estimator, arming and failsafe are all the
firmware's. Motor order is the firmware's quad-X order, which the
multirotor specs use (pinned by ``test_mujoco_multirotor``).
"""

from __future__ import annotations

import ctypes
import math
from dataclasses import dataclass

import numpy as np

from aerial_kit.types import ControlTarget, SimState

from .build import build

GRAVITY = 9.81
_lib = None


def _load():
    global _lib
    if _lib is not None:
        return _lib
    lib = ctypes.CDLL(str(build()))
    f32p = ctypes.POINTER(ctypes.c_float)
    lib.akfc_abi_version.restype = ctypes.c_int
    lib.akfc_create.restype = ctypes.c_void_p
    lib.akfc_create.argtypes = [ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
    lib.akfc_destroy.argtypes = [ctypes.c_void_p]
    lib.akfc_rc_layout.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)]
    lib.akfc_step.argtypes = [ctypes.c_void_p, f32p, f32p, ctypes.POINTER(ctypes.c_uint16),
                              ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                              f32p, f32p]
    lib.akfc_state.argtypes = [ctypes.c_void_p]
    lib.akfc_state_name.argtypes = [ctypes.c_void_p]
    lib.akfc_state_name.restype = ctypes.c_char_p
    lib.akfc_attitude.argtypes = [ctypes.c_void_p, f32p]
    lib.akfc_command.argtypes = [ctypes.c_void_p, f32p]
    lib.akfc_config.argtypes = [ctypes.c_void_p, f32p]
    for name in ("akfc_rc_channels", "akfc_max_motors", "akfc_max_servos"):
        getattr(lib, name).restype = ctypes.c_uint
    if lib.akfc_abi_version() != 2:
        raise RuntimeError("akfc shim ABI mismatch; rebuild with build.py --force")
    _lib = lib
    return lib


@dataclass(frozen=True)
class RcLayout:
    roll: int
    pitch: int
    throttle: int
    yaw: int
    mode: int
    arm: int
    min: int
    mid: int
    max: int


class AkFlightCore:
    """One instance of the firmware's flight state machine."""

    QUAD_X, ELEVON_WING = 0, 1

    def __init__(self, airframe: int = 0, motors: int = 4, servos: int = 2, loop_period_us: int = 1000):
        self._lib = _load()
        self.n_channels = int(self._lib.akfc_rc_channels())
        self.n_motors = int(self._lib.akfc_max_motors())
        self.n_servos = int(self._lib.akfc_max_servos())
        if motors > self.n_motors:
            raise ValueError(f"the AK firmware drives at most {self.n_motors} motors")
        self._h = self._lib.akfc_create(int(airframe), motors, servos, loop_period_us)
        if not self._h:
            raise MemoryError("akfc_create failed")
        raw = (ctypes.c_uint * 9)()
        self._lib.akfc_rc_layout(self._h, raw)
        self.layout = RcLayout(*[int(v) for v in raw])
        self._gyro = (ctypes.c_float * 3)()
        self._accel = (ctypes.c_float * 3)()
        self._ch = (ctypes.c_uint16 * self.n_channels)()
        self._motor = (ctypes.c_float * self.n_motors)()
        self._servo = (ctypes.c_float * self.n_servos)()

    def close(self) -> None:
        if getattr(self, "_h", None):
            self._lib.akfc_destroy(self._h)
            self._h = None

    def __del__(self):  # pragma: no cover - interpreter shutdown order
        try:
            self.close()
        except Exception:
            pass

    def step(self, gyro_frd, accel_g, channels, rc_valid: bool, rc_ms: int, now_us: int, now_ms: int):
        for i in range(3):
            self._gyro[i] = float(gyro_frd[i])
            self._accel[i] = float(accel_g[i])
        for i in range(self.n_channels):
            self._ch[i] = int(channels[i])
        self._lib.akfc_step(self._h, self._gyro, self._accel, self._ch, int(bool(rc_valid)),
                            int(rc_ms) & 0xFFFFFFFF, int(now_us) & 0xFFFFFFFF, int(now_ms) & 0xFFFFFFFF,
                            self._motor, self._servo)
        return np.array(self._motor[:], dtype=float), np.array(self._servo[:], dtype=float)

    @property
    def state(self) -> str:
        return self._lib.akfc_state_name(self._h).decode()

    def attitude(self) -> np.ndarray:
        out = (ctypes.c_float * 3)()
        self._lib.akfc_attitude(self._h, out)
        return np.array(out[:], dtype=float)

    def config(self) -> dict:
        """The firmware's own limits, read from its config struct."""
        out = (ctypes.c_float * 6)()
        self._lib.akfc_config(self._h, out)
        return dict(max_tilt_deg=out[0], max_rate_dps=out[1], throttle_low=out[2],
                    arm_hold_s=out[3] / 1000.0, arm_threshold=int(out[4]), mode_threshold=int(out[5]))

    def command(self) -> dict:
        out = (ctypes.c_float * 6)()
        self._lib.akfc_command(self._h, out)
        return dict(roll=out[0], pitch=out[1], yaw=out[2], throttle=out[3],
                    angle_mode=bool(out[4]), arm_request=bool(out[5]))

    def channels_from_sticks(self, roll: float, pitch: float, yaw: float, throttle: float,
                             arm: bool, angle_mode: bool = True) -> list[int]:
        """Receiver counts for normalised sticks (roll/pitch/yaw -1..1, throttle 0..1)."""
        L = self.layout

        def stick(v: float) -> int:
            v = max(-1.0, min(1.0, float(v)))
            return round(L.mid + v * ((L.max - L.mid) if v >= 0 else (L.mid - L.min)))

        ch = [L.mid] * self.n_channels
        ch[L.roll], ch[L.pitch], ch[L.yaw] = stick(roll), stick(pitch), stick(yaw)
        ch[L.throttle] = round(L.min + max(0.0, min(1.0, float(throttle))) * (L.max - L.min))
        ch[L.arm] = L.max if arm else L.min
        ch[L.mode] = L.max if angle_mode else L.min
        return ch


def flu_to_firmware_imu(rates_flu, accel_world, rot_body_to_world):
    """Gyro (rad/s) and accelerometer (g) in the firmware's FRD convention.

    The accelerometer reports the specific force *negated* (``+1 g`` on z
    when level and still), as ``docs/31-contract.md`` C1 and
    ``ActuatorPlant.sense`` define it.
    """
    p, q, r = np.asarray(rates_flu, dtype=float)
    gyro = np.array([p, -q, -r])
    f_world = np.asarray(accel_world, dtype=float) - np.array([0.0, 0.0, -GRAVITY])
    fx, fy, fz = np.asarray(rot_body_to_world, dtype=float).T @ f_world   # FLU specific force
    accel = np.array([-fx, fy, fz]) / GRAVITY                             # = -f_FRD / g
    return gyro, accel


class FirmwareInTheLoop:
    """The AK firmware flying a MuJoCo multirotor (quad-X) at 1 kHz."""

    def __init__(self, airframe: str = "fpv_quad_5in", start_z: float | None = None,
                 wind_mps=(0.0, 0.0, 0.0), loop_hz: float = 1000.0, battery_soc: float = 1.0):
        from aerial_kit.dynamics.rotations import quat_to_rotmat
        from sim_py.backends.mujoco_multirotor_backend import MujocoMultirotorBackend

        self._rotmat = quat_to_rotmat
        self.backend = MujocoMultirotorBackend()
        on_ground = start_z is None
        self.backend.reset(
            SimState(position=np.array([0.0, 0.0, 0.4 if on_ground else float(start_z)]),
                     velocity=np.zeros(3), t=0.0, attitude_quat=np.array([1.0, 0.0, 0.0, 0.0]),
                     body_rates=np.zeros(3)), {},
            {"simulation": {"mujoco_multirotor": {
                "airframe": airframe, "wind_mps": list(wind_mps),
                "start_motors": "idle" if on_ground else "hover", "battery_soc": battery_soc,
                # A real ESC is not battery-compensated: the firmware sees the sag.
                "voltage_compensation": False}}})
        spec = self.backend.spec
        if spec.n_rotors != 4:
            raise ValueError(f"{airframe}: the AK firmware's quad-X mixer drives 4 rotors, "
                             f"this airframe has {spec.n_rotors}")
        self.spec = spec
        self.fc = AkFlightCore(AkFlightCore.QUAD_X, motors=4, servos=0,
                               loop_period_us=int(round(1e6 / loop_hz)))
        self.dt = 1.0 / loop_hz
        self.t = 0.0
        self.sticks = dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, arm=False, angle_mode=True)
        self.rc_alive = True
        self._last_rc_ms = 0
        self._raw_channels: list[int] | None = None   # set by receive_frame()
        self._v_prev = np.asarray(self.backend.state().velocity, dtype=float).copy()
        self.motors = np.zeros(4)
        if on_ground:   # settle on the gear before the estimator starts trusting it
            for _ in range(int(1.0 / self.dt)):
                self.backend.step(ControlTarget(accel_cmd=np.zeros(3),
                                                metadata={"rotor_thrust_n": np.zeros(4)}), self.dt)
            self._v_prev = np.asarray(self.backend.state().velocity, dtype=float).copy()

    def set_sticks(self, **kwargs) -> None:
        unknown = set(kwargs) - set(self.sticks)
        if unknown:
            raise ValueError(f"unknown stick(s): {sorted(unknown)}")
        self.sticks.update(kwargs)

    def imu(self):
        s = self.backend.state()
        v = np.asarray(s.velocity, dtype=float)
        accel_world = (v - self._v_prev) / self.dt
        return flu_to_firmware_imu(s.body_rates, accel_world, self._rotmat(s.attitude_quat)), v

    def step(self, seconds: float) -> None:
        n = max(1, int(round(seconds / self.dt)))
        t_max = self.spec.max_thrust_n
        for _ in range(n):
            (gyro, accel), v = self.imu()
            self._v_prev = v
            now_us = int(round(self.t * 1e6))
            now_ms = now_us // 1000
            if self._raw_channels is not None:
                # Receiver mode: the channels and their arrival time come from
                # CRSF frames; the firmware decides on its own when they are stale.
                ch = self._raw_channels
            else:
                if self.rc_alive:
                    self._last_rc_ms = now_ms
                ch = self.fc.channels_from_sticks(**self.sticks)
            self.motors, _ = self.fc.step(gyro, accel, ch, self.rc_alive, self._last_rc_ms, now_us, now_ms)
            thrust = t_max * np.clip(self.motors[:4], 0.0, 1.0) ** 2   # throttle -> speed -> thrust
            self.backend.step(ControlTarget(accel_cmd=np.zeros(3),
                                            metadata={"rotor_thrust_n": thrust}), self.dt)
            self.t += self.dt

    def receive_frame(self, channels) -> None:
        """An RC frame arrived from the receiver now (CRSF channel counts).

        Switches this loop to receiver mode for good: from now on only frames
        feed the firmware, as on a real FC wired to an ELRS receiver.
        """
        ch = [int(c) for c in list(channels)[:self.fc.n_channels]]
        if len(ch) < self.fc.n_channels:
            raise ValueError(f"need {self.fc.n_channels} channels, got {len(ch)}")
        self._raw_channels = ch
        self.rc_alive = True
        self._last_rc_ms = int(round(self.t * 1e6)) // 1000

    @property
    def state(self) -> SimState:
        return self.backend.state()

    def close(self) -> None:
        self.fc.close()


class FirmwareRadio:
    """What a pilot's radio would send the firmware, from teleop-style inputs.

    * Arming is the real sequence: the arm switch goes on with the throttle at
      minimum and stays there for ``arm_hold_s`` before the sticks pass
      through (the firmware needs throttle low for its arm hold).
    * ``throttle`` in the teleop convention (0.5 = hover) is mapped in
      *thrust*: 0 is motors at idle, centre is hover, 1 is
      ``max_thrust_ratio`` x hover thrust (default 2, i.e. +1 g). Mapping
      the stick to full throttle makes a 5:1 FPV quad unflyable from a
      keyboard; the firmware has no altitude hold, so climb rate is the
      pilot's job, as on the real aircraft.
    * ``mode`` 'angle' and 'acro' select the firmware's mode channel; the
      simulator's 'velocity' mode is not a firmware feature and flies as angle.
    """

    def __init__(self, hover_throttle: float, arm_hold_s: float = 0.8, max_thrust_ratio: float = 2.0):
        if not 0.0 < hover_throttle < 1.0:
            raise ValueError("hover_throttle must be in (0, 1)")
        if max_thrust_ratio < 1.0:
            raise ValueError("max_thrust_ratio must be >= 1")
        self.hover = float(hover_throttle)
        self.max_thrust_ratio = float(max_thrust_ratio)
        self.arm_hold_s = float(arm_hold_s)
        self._armed_since: float | None = None

    def firmware_throttle(self, stick: float) -> float:
        t = min(max(float(stick), 0.0), 1.0)
        if t <= 0.5:
            ratio = t / 0.5                                   # 0 .. hover thrust
        else:
            ratio = 1.0 + (self.max_thrust_ratio - 1.0) * (t - 0.5) / 0.5
        return min(1.0, self.hover * math.sqrt(ratio))        # thrust = T_max * u^2

    def sticks(self, now_s: float, arm: bool, mode: str = "angle",
               roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
               throttle: float = 0.5) -> dict:
        if not arm:
            self._armed_since = None
            return dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, arm=False, angle_mode=True)
        if self._armed_since is None:
            self._armed_since = float(now_s)
        angle = mode != "acro"
        if now_s - self._armed_since < self.arm_hold_s:
            return dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, arm=True, angle_mode=angle)
        return dict(roll=roll, pitch=pitch, yaw=yaw, throttle=self.firmware_throttle(throttle),
                    arm=True, angle_mode=angle)


def hover_throttle(spec) -> float:
    """Throttle (0..1) that hovers this airframe: thrust = T_max * u^2."""
    return float(math.sqrt(float(np.mean(spec.hover_thrusts())) / spec.max_thrust_n))


def radio_stick_settings(spec, fc: AkFlightCore | None = None, climb_gain_per_s: float = 2.0,
                         speed_gain_per_s: float = 0.4, thrust_range=(0.3, 2.0)) -> dict:
    """Host radio settings that fly this airframe on the AK firmware in angle mode.

    The host's vertical loop sends throttle = hover + kz * (vz_demand - vz).
    With thrust = T_max u^2 a throttle step du gives about 2 du / hover g
    of vertical acceleration, so ``kz = hover / (2 g) * climb_gain_per_s``
    makes the loop a first-order climb-rate controller with that bandwidth.
    The throttle range is clipped to ``thrust_range`` x hover thrust so up
    and down authority stay comparable (a 9:1 quad otherwise has +8 g / -1 g
    and any lag in the velocity estimate ratchets it upward).

    Horizontally, a tilt theta gives g tan(theta) of acceleration, so
    ``kv_xy = speed_gain_per_s / g`` rad per m/s of error. The defaults
    (2 /s climb, 0.4 /s speed) stay stable with a 1 s lag on the host's
    velocity estimate, about what CRSF telemetry plus the 1 Hz baro/GPS
    filter give; the original kv_xy = 0.5 rad/(m/s) flips the quad there.
    """
    own = fc is None
    fc = fc or AkFlightCore()
    try:
        cfg = fc.config()
    finally:
        if own:
            fc.close()
    hover = hover_throttle(spec)
    lo, hi = thrust_range
    return {
        "stick_mapping": "angle",
        "max_tilt_deg": float(cfg["max_tilt_deg"]),
        "hover_throttle": hover,
        "kz": hover / (2.0 * GRAVITY) * float(climb_gain_per_s),
        "kv_xy": float(speed_gain_per_s) / GRAVITY,
        "tilt_compensation": True,
        "throttle_min": max(float(cfg["throttle_low"]) + 0.01, hover * math.sqrt(lo)),
        "throttle_max": min(0.95, hover * math.sqrt(hi)),
    }
