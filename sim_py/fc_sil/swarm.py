"""A swarm in which every drone runs its own copy of the AK firmware.

Each vehicle of a :class:`sim_py.backends.mujoco_swarm.MujocoSwarm` gets an
:class:`AkFlightCore`: its IMU from that vehicle's MuJoCo state, its motor
outputs to that vehicle's rotors, all at the firmware's 1 kHz. On top, each
drone has what a real swarm member would carry, an onboard companion that turns a
world-frame velocity setpoint into angle-mode sticks (the firmware has no
velocity mode) using its own, un-delayed state, the role a companion
computer on an MSP/CRSF link plays.

    fs = FirmwareSwarm(n=4)
    fs.arm()
    fs.fly_velocity(lambda p: v_world_per_drone, seconds=5.0)
"""

from __future__ import annotations

import math

import numpy as np

from aerial_kit.dynamics.rotations import quat_to_rotmat
from aerial_kit.types import ControlTarget

from . import AkFlightCore, GRAVITY, flu_to_firmware_imu, hover_throttle


def velocity_to_angle_sticks(v_world, v_cmd_world, yaw_rad: float, hover: float, max_tilt_deg: float,
                             speed_gain_per_s: float = 0.8, climb_gain_per_s: float = 3.0,
                             throttle_range=(0.05, 0.95)) -> dict:
    """Companion law: velocity error -> tilt (angle sticks) and tilt-compensated throttle.

    A tilt theta gives g tan(theta) of horizontal acceleration, so the tilt for
    an acceleration ``k * e`` is ``atan(k e / g)``; throttle is hover thrust
    divided by cos(tilt), plus ``hover / (2 g) * k_z * e_z`` (thrust ~ u^2).
    Sticks follow the RC convention: roll right +, pitch forward + (nose down).
    """
    e = np.asarray(v_cmd_world, dtype=float) - np.asarray(v_world, dtype=float)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    fwd = c * e[0] + s * e[1]
    right = s * e[0] - c * e[1]
    lim = math.radians(max_tilt_deg)
    pitch = max(-lim, min(lim, math.atan(speed_gain_per_s * fwd / GRAVITY)))
    roll = max(-lim, min(lim, math.atan(speed_gain_per_s * right / GRAVITY)))
    tilt_comp = 1.0 / math.sqrt(max(math.cos(roll) * math.cos(pitch), 0.25))
    throttle = hover * tilt_comp + hover / (2.0 * GRAVITY) * climb_gain_per_s * e[2]
    lo, hi = throttle_range
    return dict(roll=roll / lim, pitch=pitch / lim, yaw=0.0, throttle=float(min(hi, max(lo, throttle))))


class FirmwareSwarm:
    """N quad-X drones, each flown by its own unmodified AK flight core."""

    def __init__(self, n: int = 4, airframe: str = "swarm_quad", spacing_m: float = 1.5, loop_hz: float = 1000.0,
                 accel_model: str = "physical"):
        from sim_py.backends.mujoco_swarm import MujocoSwarm

        if accel_model not in ("physical", "gravity"):
            raise ValueError("accel_model must be 'physical' or 'gravity' (test instrument: reads gravity only)")
        self.accel_model = accel_model

        self.swarm = MujocoSwarm(n=n, airframe=airframe, spacing_m=spacing_m,
                                 settings={"voltage_compensation": False})
        spec = self.swarm.spec
        if spec.n_rotors != 4:
            raise ValueError(f"{airframe}: the AK firmware's quad-X mixer drives 4 rotors, not {spec.n_rotors}")
        self.n = self.swarm.n
        self.fcs = [AkFlightCore(AkFlightCore.QUAD_X, motors=4, servos=0, loop_period_us=int(round(1e6 / loop_hz)))
                    for _ in range(self.n)]
        self.max_tilt_deg = float(self.fcs[0].config()["max_tilt_deg"])
        self.hover = hover_throttle(spec)
        self.t_max = spec.max_thrust_n
        self.dt = 1.0 / loop_hz
        self.t = 0.0
        self.sticks = [dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, arm=False, angle_mode=True)
                       for _ in range(self.n)]
        self.rc_alive = [True] * self.n
        self._last_rc_ms = [0] * self.n
        self._v_prev = np.array([s.velocity for s in self.swarm.states()], dtype=float)
        self.motors = np.zeros((self.n, 4))
        self.min_separation_seen = math.inf
        self.contacts_seen: set[tuple[int, int]] = set()

    @property
    def states(self):
        return self.swarm.states()

    @property
    def fc_states(self) -> list[str]:
        return [fc.state for fc in self.fcs]

    def step(self, seconds: float) -> None:
        for _ in range(max(1, int(round(seconds / self.dt)))):
            now_us = int(round(self.t * 1e6))
            now_ms = now_us // 1000
            states = self.swarm.states()
            targets = []
            for i, (fc, s) in enumerate(zip(self.fcs, states)):
                v = np.asarray(s.velocity, dtype=float)
                a_world = (v - self._v_prev[i]) / self.dt if self.accel_model == "physical" else np.zeros(3)
                gyro, accel = flu_to_firmware_imu(s.body_rates, a_world, quat_to_rotmat(s.attitude_quat))
                self._v_prev[i] = v
                if self.rc_alive[i]:
                    self._last_rc_ms[i] = now_ms
                motors, _ = fc.step(gyro, accel, fc.channels_from_sticks(**self.sticks[i]),
                                    self.rc_alive[i], self._last_rc_ms[i], now_us, now_ms)
                self.motors[i] = motors[:4]
                targets.append(ControlTarget(accel_cmd=np.zeros(3), metadata={
                    "rotor_thrust_n": self.t_max * np.clip(motors[:4], 0.0, 1.0) ** 2}))
            self.swarm.step(targets, self.dt)
            self.t += self.dt
        self.min_separation_seen = min(self.min_separation_seen, self.swarm.min_separation_m())
        self.contacts_seen.update(self.swarm.drone_contacts())

    def arm(self) -> None:
        """The real sequence on every drone: switch off, then on with throttle at minimum."""
        for st in self.sticks:
            st.update(arm=False, throttle=0.0, roll=0.0, pitch=0.0)
        self.step(0.3)
        for st in self.sticks:
            st.update(arm=True)
        self.step(0.8)

    def disarm(self) -> None:
        for st in self.sticks:
            st.update(arm=False, throttle=0.0)
        self.step(0.1)

    def fly_velocity(self, velocity_fn, seconds: float, period_s: float = 0.02) -> None:
        """Every ``period_s`` the companions read state and set sticks for ``velocity_fn(positions)``."""
        for _ in range(max(1, int(round(seconds / period_s)))):
            states = self.swarm.states()
            v_cmd = np.asarray(velocity_fn(np.array([s.position for s in states])), dtype=float)
            for i, s in enumerate(states):
                r = quat_to_rotmat(s.attitude_quat)
                yaw = math.atan2(r[1, 0], r[0, 0])
                self.sticks[i].update(velocity_to_angle_sticks(
                    s.velocity, v_cmd[i], yaw, self.hover, self.max_tilt_deg))
            self.step(period_s)

    def close(self) -> None:
        for fc in self.fcs:
            fc.close()
