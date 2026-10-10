"""The INS EKF in a simulation loop: simulated sensors in, an estimated state out.

A controller in the simulator used to be handed the plant's true state. On an
aircraft it gets an estimate: GPS at a few hertz with metres of noise, a baro,
and an accelerometer with a bias. :class:`SimulatedIns` stands between the two.
Each call to :meth:`SimulatedIns.update` takes the true state, synthesises what
those sensors would have reported since the last call, runs them through
:class:`~aerial_kit.estimation.ins_ekf.InsEkf`, and returns a state whose
position and velocity are the filter's.

What the sensors are made of
----------------------------
Accelerometer
    Specific force in z-up body axes (REP-145, the filter's
    ``"body_specific_force"`` mode), built from the change in true velocity
    over the step: ``f = R^T ((v_k - v_{k-1}) / dt + g e3)``. That is the
    average specific force over the interval - what an integrating IMU's
    delta-velocity reports - so the filter's prediction across the step is
    exact apart from the noise and bias added here. It needs no plant
    internals, so it works on every backend.
GPS
    True position plus white noise, horizontal and vertical separately, at
    ``gps_rate_hz``.
Baro
    True altitude plus a constant offset plus white noise, at ``baro_rate_hz``.

Attitude and body rates are passed through from the truth: the filter takes
attitude as an input (see its module docstring - the flight controller's AHRS
provides it), so this is the same division of labour the hardware has. A
backend with no attitude (the point mass) is treated as level.

Every draw comes from one generator seeded by ``seed``, so a run is repeatable.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields, replace
from typing import Any, Mapping

import numpy as np

from ..dynamics.rotations import quat_to_rotmat
from ..types import SimState
from .ins_ekf import InsConfig, InsEkf

_LEVEL = np.array([1.0, 0.0, 0.0, 0.0])


@dataclass
class SensorConfig:
    """What the simulated sensors report, and how badly. SI units, ENU."""

    gps_rate_hz: float = 5.0
    gps_sigma_xy_m: float = 1.0
    gps_sigma_z_m: float = 2.0
    baro_rate_hz: float = 20.0
    baro_sigma_m: float = 0.3
    baro_offset_m: float = 0.0
    # White noise on each accelerometer sample, m/s^2, one sigma.
    accel_sigma_mps2: float = 0.1
    # Constant accelerometer bias, z-up body axes, m/s^2. The filter estimates it.
    accel_bias_mps2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    seed: int = 0

    def __post_init__(self) -> None:
        for name in ("gps_rate_hz", "baro_rate_hz"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be > 0")
        for name in ("gps_sigma_xy_m", "gps_sigma_z_m", "baro_sigma_m", "accel_sigma_mps2"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be >= 0")
        bias = np.asarray(self.accel_bias_mps2, dtype=float)
        if bias.shape != (3,):
            raise ValueError("accel_bias_mps2 must have 3 components")
        self.accel_bias_mps2 = tuple(float(b) for b in bias)


@dataclass
class SimulatedIns:
    """Simulated GPS, baro and IMU feeding an :class:`InsEkf`.

    ``ekf`` defaults to a filter whose measurement noise matches the sensors'
    (a filter that is told the truth about its sensors is the baseline; a
    mismatched one is an experiment, and is what passing ``ekf`` is for).
    """

    sensors: SensorConfig = field(default_factory=SensorConfig)
    ekf: InsConfig | None = None

    def __post_init__(self) -> None:
        if self.ekf is None:
            s = self.sensors
            self.ekf = InsConfig(
                accel_mode="body_specific_force",
                gps_sigma_xy=max(s.gps_sigma_xy_m, 1e-3),
                gps_sigma_z=max(s.gps_sigma_z_m, 1e-3),
                baro_sigma=max(s.baro_sigma_m, 1e-3),
            )
        elif self.ekf.accel_mode != "body_specific_force":
            raise ValueError("the simulated accelerometer reports body specific force; use that accel_mode")
        self.filter = InsEkf(self.ekf)
        self._rng = np.random.default_rng(self.sensors.seed)
        self._prev: SimState | None = None
        self._next_gps = 0.0
        self._next_baro = 0.0
        self.gps_updates = 0
        self.baro_updates = 0

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "SimulatedIns":
        """Build from a mapping: sensor fields at the top, filter fields under ``ekf``."""
        cfg = dict(cfg or {})
        ekf_cfg = cfg.pop("ekf", None)
        cfg.pop("mode", None)
        known = {f.name for f in fields(SensorConfig)}
        unknown = sorted(set(cfg) - known)
        if unknown:
            raise ValueError(f"unknown estimator setting(s) {unknown}; known are {sorted(known | {'mode', 'ekf'})}")
        sensors = SensorConfig(**cfg)
        ekf = None
        if ekf_cfg is not None:
            ekf_cfg = dict(ekf_cfg)
            ekf_known = {f.name for f in fields(InsConfig)}
            unknown = sorted(set(ekf_cfg) - ekf_known)
            if unknown:
                raise ValueError(f"unknown estimator.ekf setting(s) {unknown}; known are {sorted(ekf_known)}")
            ekf_cfg.setdefault("accel_mode", "body_specific_force")
            defaults = cls(sensors=sensors).ekf
            ekf = replace(defaults, **ekf_cfg)
        return cls(sensors=sensors, ekf=ekf)

    def reset(self, truth: SimState) -> SimState:
        """Start at the true state: the home point is known when the aircraft arms.

        The baro offset keeps the filter's prior uncertainty; GPS altitude makes
        it observable, which is what ``baro_offset_m`` is there to exercise.
        """
        self.filter.reset(
            np.asarray(truth.position, dtype=float),
            float(truth.t),
            velocity=np.asarray(truth.velocity, dtype=float),
            position_sigma=0.1,
        )
        self._prev = truth
        self._next_gps = float(truth.t) + 1.0 / self.sensors.gps_rate_hz
        self._next_baro = float(truth.t) + 1.0 / self.sensors.baro_rate_hz
        return self._estimate(truth)

    def update(self, truth: SimState) -> SimState:
        """Advance to ``truth.t`` and return the estimated state."""
        if self._prev is None:
            return self.reset(truth)
        s = self.sensors
        t = float(truth.t)
        dt = t - float(self._prev.t)
        if dt <= 0.0:
            return self._estimate(truth)

        quat = self._quat(truth)
        R = quat_to_rotmat(quat)
        a_world = (np.asarray(truth.velocity, dtype=float) - np.asarray(self._prev.velocity, dtype=float)) / dt
        f_body = R.T @ (a_world + np.array([0.0, 0.0, self.filter.cfg.gravity]))
        f_body = f_body + np.asarray(s.accel_bias_mps2) + self._rng.normal(0.0, s.accel_sigma_mps2, size=3)
        self.filter.predict(t, accel=f_body, attitude_quat=quat)

        position = np.asarray(truth.position, dtype=float)
        if t + 1e-9 >= self._next_gps:
            noise = self._rng.normal(0.0, 1.0, size=3) * np.array([s.gps_sigma_xy_m, s.gps_sigma_xy_m, s.gps_sigma_z_m])
            self.filter.update_gps(position + noise)
            self.gps_updates += 1
            self._next_gps += 1.0 / s.gps_rate_hz
            if self._next_gps <= t:  # a step longer than the GPS period
                self._next_gps = t + 1.0 / s.gps_rate_hz
        if t + 1e-9 >= self._next_baro:
            self.filter.update_baro(position[2] + s.baro_offset_m + self._rng.normal(0.0, s.baro_sigma_m))
            self.baro_updates += 1
            self._next_baro += 1.0 / s.baro_rate_hz
            if self._next_baro <= t:
                self._next_baro = t + 1.0 / s.baro_rate_hz

        self._prev = truth
        return self._estimate(truth)

    @staticmethod
    def _quat(state: SimState) -> np.ndarray:
        if state.attitude_quat is None:
            return _LEVEL.copy()
        q = np.asarray(state.attitude_quat, dtype=float).reshape(4)
        return q / np.linalg.norm(q)

    def _estimate(self, truth: SimState) -> SimState:
        return SimState(
            position=self.filter.position,
            velocity=self.filter.velocity,
            t=float(truth.t),
            attitude_quat=truth.attitude_quat,
            body_rates=truth.body_rates,
        )


__all__ = ["SensorConfig", "SimulatedIns"]
