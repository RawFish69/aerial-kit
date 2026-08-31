"""Native attitude-aware multirotor dynamics backend."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from aerial_kit.dynamics.multirotor import DynamicsParams, UAVDynamics
from aerial_kit.interfaces import DynamicsBackend
from aerial_kit.types import ControlTarget, SimState


def _quat_to_rpy(quat: np.ndarray) -> tuple[float, float, float]:
    """Standard body-to-world quaternion to roll, pitch, yaw."""
    w, x, y, z = np.asarray(quat, dtype=float)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)


def _wrap_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _euler_rates_to_body_rates(
    euler_rates: np.ndarray, *, roll: float, pitch: float
) -> np.ndarray:
    """Convert ZYX Euler-angle rates into body rates at the given attitude."""
    roll_rate, pitch_rate, yaw_rate = np.asarray(euler_rates, dtype=float).reshape(3)
    sin_roll, cos_roll = np.sin(roll), np.cos(roll)
    sin_pitch, cos_pitch = np.sin(pitch), np.cos(pitch)
    return np.array(
        [
            roll_rate - yaw_rate * sin_pitch,
            pitch_rate * cos_roll + yaw_rate * cos_pitch * sin_roll,
            -pitch_rate * sin_roll + yaw_rate * cos_pitch * cos_roll,
        ],
        dtype=float,
    )


#: ``ControlTarget.metadata`` key holding a commanded body yaw rate in rad/s.
#: Positive is a left (counter-clockwise seen from above) turn.
YAW_RATE_METADATA_KEY = "yaw_rate_cmd"


class MultirotorBackend(DynamicsBackend):
    """Map world-frame acceleration targets into thrust and body-rate commands.

    The tilt needed to produce a commanded acceleration depends on where the
    airframe is currently pointing, so the desired thrust direction is rotated
    into the vehicle's yaw-aligned frame before roll/pitch are extracted. That
    makes a commanded yaw rate (``metadata["yaw_rate_cmd"]``) a real degree of
    freedom rather than something the attitude loop fights against.

    Heading is tracked against an integrated setpoint rather than left open
    loop. Roll and pitch corrections are Euler-angle rates, and feeding those
    straight to the body-rate loop leaks into yaw whenever the airframe is
    tilted, so an uncommanded quadrotor would slowly spin. The Euler rates are
    converted to body rates and the heading hold removes any residual drift.
    """

    def __init__(self) -> None:
        self._dyn: UAVDynamics | None = None
        self._t = 0.0
        self._attitude_kp = 5.0
        self._yaw_kp = 4.0
        self._max_body_rate = 4.0
        self._yaw_setpoint = 0.0

    def reset(
        self,
        initial_state: SimState,
        world: Mapping[str, Any],
        cfg: Mapping[str, Any],
    ) -> None:
        del world
        sim_cfg = dict(cfg.get("simulation", {}) or {})
        mr_cfg = dict(sim_cfg.get("multirotor", {}) or {})
        param_fields = set(DynamicsParams.__dataclass_fields__)
        params = DynamicsParams(**{key: value for key, value in mr_cfg.items() if key in param_fields})
        self._attitude_kp = float(mr_cfg.get("attitude_kp", 5.0))
        self._yaw_kp = float(mr_cfg.get("yaw_kp", 4.0))
        self._max_body_rate = float(mr_cfg.get("max_body_rate_rps", 4.0))

        self._dyn = UAVDynamics(params)
        self._dyn.position = np.asarray(initial_state.position, dtype=float).copy()
        self._dyn.velocity = np.asarray(initial_state.velocity, dtype=float).copy()
        if initial_state.attitude_quat is not None:
            self._dyn.attitude_quat = np.asarray(initial_state.attitude_quat, dtype=float).copy()
        if initial_state.body_rates is not None:
            self._dyn.body_rates = np.asarray(initial_state.body_rates, dtype=float).copy()
        self._t = float(initial_state.t)
        self._yaw_setpoint = _quat_to_rpy(self._dyn.attitude_quat)[2]

    def step(self, control_target: ControlTarget, dt: float) -> None:
        if self._dyn is None:
            raise RuntimeError("MultirotorBackend.reset() must be called before step().")

        acceleration = np.asarray(control_target.accel_cmd, dtype=float).reshape(3)
        desired_specific_force = acceleration + np.array([0.0, 0.0, self._dyn.params.gravity])
        force_norm = float(np.linalg.norm(desired_specific_force))
        if force_norm < 1e-9:
            desired_specific_force = np.array([0.0, 0.0, self._dyn.params.gravity])
            force_norm = self._dyn.params.gravity
        body_z = desired_specific_force / force_norm

        roll, pitch, yaw = _quat_to_rpy(self._dyn.attitude_quat)
        # De-rotate the desired thrust axis by the current heading so roll/pitch
        # targets are expressed in the airframe's own forward/right frame.
        cos_yaw = float(np.cos(yaw))
        sin_yaw = float(np.sin(yaw))
        heading_frame_z = np.array(
            [
                cos_yaw * body_z[0] + sin_yaw * body_z[1],
                -sin_yaw * body_z[0] + cos_yaw * body_z[1],
                body_z[2],
            ]
        )
        roll_des = float(np.arcsin(np.clip(-heading_frame_z[1], -1.0, 1.0)))
        pitch_des = float(np.arctan2(heading_frame_z[0], heading_frame_z[2]))

        yaw_rate_cmd = float(control_target.metadata.get(YAW_RATE_METADATA_KEY, 0.0) or 0.0)
        self._yaw_setpoint = _wrap_pi(self._yaw_setpoint + yaw_rate_cmd * float(dt))
        euler_rates = np.array(
            [
                self._attitude_kp * _wrap_pi(roll_des - roll),
                self._attitude_kp * _wrap_pi(pitch_des - pitch),
                yaw_rate_cmd + self._yaw_kp * _wrap_pi(self._yaw_setpoint - yaw),
            ]
        )
        rate_cmd = np.clip(
            _euler_rates_to_body_rates(euler_rates, roll=roll, pitch=pitch),
            -self._max_body_rate,
            self._max_body_rate,
        )
        thrust_cmd = force_norm / max(self._dyn.params.gravity, 1e-9)
        self._dyn.set_command(rate_cmd, thrust_cmd)
        self._dyn.step(float(dt))
        self._t += float(dt)

    def state(self) -> SimState:
        if self._dyn is None:
            raise RuntimeError("MultirotorBackend.reset() must be called before state().")
        return SimState(
            position=self._dyn.position.copy(),
            velocity=self._dyn.velocity.copy(),
            attitude_quat=self._dyn.attitude_quat.copy(),
            body_rates=self._dyn.body_rates.copy(),
            t=self._t,
        )

    def visualization_command(self) -> tuple[float, np.ndarray]:
        """Collective thrust (1 = hover) and FRD body-rate command.

        Used by the teleop viewer to colour each propeller. Missing after
        ``reset`` is treated as hover with zero rates.
        """
        if self._dyn is None:
            return 1.0, np.zeros(3, dtype=float)
        return float(self._dyn.cmd_thrust), np.asarray(self._dyn.cmd_body_rates, dtype=float).copy()

    def apply_constraints(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
        terrain: Any | None,
        terrain_clearance: float,
    ) -> None:
        if self._dyn is None:
            raise RuntimeError("MultirotorBackend.reset() must be called before apply_constraints().")
        self._dyn.position = np.clip(self._dyn.position, min_bounds, max_bounds)
        if terrain is not None and hasattr(terrain, "height_at"):
            ground = float(terrain.height_at(self._dyn.position[0], self._dyn.position[1]))
            self._dyn.position[2] = max(self._dyn.position[2], ground + terrain_clearance)
