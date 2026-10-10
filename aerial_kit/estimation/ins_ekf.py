"""Loosely coupled INS Kalman filter: position, velocity and sensor biases.

What it estimates, and what it takes as given
---------------------------------------------
State (10): ``[p (3), v (3), b_acc (3), b_baro (1)]`` - world ENU position and
velocity, the accelerometer's bias in body axes, and the barometer's offset
from the GPS-referenced altitude.

Attitude is an *input*, not a state. Every flight controller this stack talks to
(Betaflight, PX4, ArduPilot) already runs an AHRS on gyro, accelerometer and
magnetometer at kHz rates; re-estimating attitude here from a 50 Hz telemetry
stream would be worse at it. What the FC does *not* do over a CRSF link is fuse
GPS and baro into a position and velocity the mission loop can close on - that
is this filter's job.

Accelerometer modes
-------------------
The ROS contract ``/uav/hw/imu`` (``sensor_msgs/Imu``) does not pin down what a
given source puts in ``linear_acceleration``, so it is a setting:

``"body_specific_force"``
    REP-145: specific force in body axes, gravity included - an IMU at rest
    reads ``+g`` on its up axis. ``a_world = R (f - b_acc) - g e3``. The
    accelerometer bias is estimated.
``"world_linear"``
    Gravity already removed and already rotated to world axes (what
    ``hw_bridge``'s ``fake_fc_sim`` used to publish). ``a_world = f``; no bias.
``"none"``
    No usable acceleration (Betaflight's CRSF telemetry carries attitude, not
    accelerometer). The prediction is a constant-velocity model with white
    acceleration noise. Treating an all-zeros field as a measurement would read
    as free fall, which is why this is the default.

Measurements: GPS position (horizontal and vertical with separate noise), baro
altitude. Each update is gated by its Mahalanobis distance against a chi-square
threshold, so a GPS jump or a baro spike is rejected rather than followed.

The model is linear in the state given the attitude, so the predict and update
equations are the Kalman filter's exactly; "EKF" is about the attitude entering
through ``R`` and is the name people search for.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..dynamics.rotations import quat_to_rotmat

G = 9.81
NX = 10
ACCEL_MODES = ("body_specific_force", "world_linear", "none")

# 99.9 % chi-square quantiles for 1, 2 and 3 degrees of freedom.
_CHI2_999 = {1: 10.83, 2: 13.82, 3: 16.27}


@dataclass
class InsConfig:
    accel_mode: str = "none"
    # Process noise (continuous-time spectral densities)
    # m/s^2/sqrt(Hz). With an IMU it is the accelerometer's noise density
    # (vibration included); in "none" mode it is the acceleration the model
    # does not know about, which is far larger. None picks per mode.
    accel_noise: float | None = None
    accel_bias_walk: float = 0.01  # m/s^2/sqrt(s)
    baro_bias_walk: float = 0.05  # m/sqrt(s)
    # Measurement noise (1-sigma)
    gps_sigma_xy: float = 1.5
    gps_sigma_z: float = 3.0
    baro_sigma: float = 0.3
    # Initial uncertainty
    init_pos_sigma: float = 5.0
    init_vel_sigma: float = 1.0
    init_bias_sigma: float = 0.3
    init_baro_bias_sigma: float = 2.0
    gate: bool = True
    gravity: float = G
    # Velocity output: horizontal velocity is only as good as the GPS rate in
    # "none" mode; this bounds how stale a prediction is allowed to get.
    max_predict_dt: float = 0.5

    def __post_init__(self) -> None:
        if self.accel_mode not in ACCEL_MODES:
            raise ValueError(f"accel_mode must be one of {ACCEL_MODES}, got {self.accel_mode!r}")
        if self.accel_noise is None:
            self.accel_noise = 1.0 if self.accel_mode == "none" else 0.1
        for name in ("accel_noise", "gps_sigma_xy", "gps_sigma_z", "baro_sigma",
                     "init_pos_sigma", "init_vel_sigma"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be > 0")
        for name in ("accel_bias_walk", "baro_bias_walk", "init_bias_sigma", "init_baro_bias_sigma"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be >= 0")


@dataclass
class UpdateResult:
    accepted: bool
    mahalanobis2: float
    innovation: np.ndarray = field(default_factory=lambda: np.zeros(0))


class InsEkf:
    """See the module docstring. Call :meth:`predict` then any updates, in time order."""

    def __init__(self, config: InsConfig | None = None) -> None:
        self.cfg = config or InsConfig()
        self.x = np.zeros(NX)
        self.P = np.diag(
            [self.cfg.init_pos_sigma**2] * 3
            + [self.cfg.init_vel_sigma**2] * 3
            + [self.cfg.init_bias_sigma**2] * 3
            + [self.cfg.init_baro_bias_sigma**2]
        )
        self.t: Optional[float] = None
        self.initialised = False
        self.rejected = {"gps": 0, "baro": 0}

    # -- state access ---------------------------------------------------------
    @property
    def position(self) -> np.ndarray:
        return self.x[0:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:6].copy()

    @property
    def accel_bias(self) -> np.ndarray:
        return self.x[6:9].copy()

    @property
    def baro_bias(self) -> float:
        return float(self.x[9])

    def position_sigma(self) -> np.ndarray:
        return np.sqrt(np.diag(self.P)[0:3])

    def velocity_sigma(self) -> np.ndarray:
        return np.sqrt(np.diag(self.P)[3:6])

    def reset(
        self,
        position: np.ndarray,
        t: float,
        velocity: np.ndarray | None = None,
        position_sigma: float | None = None,
        baro_bias_sigma: float | None = None,
    ) -> None:
        """Start from a known position (e.g. the home point at arming).

        ``position_sigma`` overrides the initial position uncertainty. A home
        that defines the origin is known exactly, and leaving it at the
        default lets an unobservable split - altitude against baro offset,
        when there is no GPS altitude - drift towards the prior instead.
        ``baro_bias_sigma`` likewise, for a baro zeroed at the same moment.
        """
        self.__init__(self.cfg)
        self.x[0:3] = np.asarray(position, dtype=float).reshape(3)
        if position_sigma is not None:
            self.P[0:3, 0:3] = (float(position_sigma) ** 2) * np.eye(3)
        if baro_bias_sigma is not None:
            self.P[9, 9] = float(baro_bias_sigma) ** 2
        if velocity is not None:
            self.x[3:6] = np.asarray(velocity, dtype=float).reshape(3)
        self.t = float(t)
        self.initialised = True

    # -- prediction -------------------------------------------------------------
    def predict(self, t: float, accel: np.ndarray | None = None, attitude_quat: np.ndarray | None = None) -> None:
        """Propagate to time ``t``.

        ``accel`` is the IMU's ``linear_acceleration`` and ``attitude_quat`` the
        FC's attitude (w-first, body to world, z-up body); how ``accel`` is read
        depends on ``accel_mode``. In ``"none"`` mode both are ignored.
        """
        t = float(t)
        if self.t is None:
            self.t = t
            return
        dt = t - self.t
        if dt <= 0.0:
            return  # out of order or duplicate: nothing to propagate
        self.t = t
        while dt > 0.0:
            h = min(dt, self.cfg.max_predict_dt)
            self._predict_step(h, accel, attitude_quat)
            dt -= h

    def _predict_step(self, dt: float, accel, attitude_quat) -> None:
        cfg = self.cfg
        F = np.eye(NX)
        F[0:3, 3:6] = dt * np.eye(3)
        a_world = np.zeros(3)
        if cfg.accel_mode == "body_specific_force" and accel is not None and attitude_quat is not None:
            R = quat_to_rotmat(np.asarray(attitude_quat, dtype=float) / np.linalg.norm(attitude_quat))
            f = np.asarray(accel, dtype=float).reshape(3)
            a_world = R @ (f - self.x[6:9]) - np.array([0.0, 0.0, cfg.gravity])
            # d(a_world)/d(b_acc) = -R
            F[0:3, 6:9] = -0.5 * dt * dt * R
            F[3:6, 6:9] = -dt * R
        elif cfg.accel_mode == "world_linear" and accel is not None:
            a_world = np.asarray(accel, dtype=float).reshape(3)

        self.x[0:3] += self.x[3:6] * dt + 0.5 * a_world * dt * dt
        self.x[3:6] += a_world * dt

        # Discrete process noise: white acceleration on (p, v) (the standard
        # piecewise-continuous form), random walks on the biases.
        q = cfg.accel_noise**2
        Q = np.zeros((NX, NX))
        blk = q * np.array([[dt**3 / 3.0, dt**2 / 2.0], [dt**2 / 2.0, dt]])
        for i in range(3):
            Q[np.ix_([i, 3 + i], [i, 3 + i])] = blk
        Q[6:9, 6:9] = (cfg.accel_bias_walk**2) * dt * np.eye(3)
        Q[9, 9] = (cfg.baro_bias_walk**2) * dt
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    # -- updates ----------------------------------------------------------------
    def update_gps(self, position: np.ndarray, use_altitude: bool = True) -> UpdateResult:
        """A GPS fix as ENU position relative to the origin (``hw_bridge.geo``).

        ``use_altitude=False`` updates east and north only - for receivers whose
        altitude is not worth fusing - rather than inventing a vertical value.
        """
        z = np.asarray(position, dtype=float).reshape(3)
        if not self.initialised:
            self.x[0:3] = z if use_altitude else np.array([z[0], z[1], self.x[2]])
            self.initialised = True
            return UpdateResult(True, 0.0, np.zeros(3))
        rows = 3 if use_altitude else 2
        H = np.zeros((rows, NX))
        H[:, 0:rows] = np.eye(rows)
        Rn = np.diag(([self.cfg.gps_sigma_xy**2] * 2 + [self.cfg.gps_sigma_z**2])[:rows])
        return self._update(z[:rows], H, Rn, "gps")

    def update_baro(self, altitude: float) -> UpdateResult:
        """Baro altitude relative to the same zero as the GPS altitude (e.g. home)."""
        z = np.array([float(altitude)])
        H = np.zeros((1, NX))
        H[0, 2] = 1.0
        H[0, 9] = 1.0
        return self._update(z, H, np.array([[self.cfg.baro_sigma**2]]), "baro")

    def _update(self, z, H, Rn, name) -> UpdateResult:
        y = z - H @ self.x
        S = H @ self.P @ H.T + Rn
        S_inv = np.linalg.inv(S)
        d2 = float(y @ S_inv @ y)
        if self.cfg.gate and d2 > _CHI2_999[len(z)]:
            self.rejected[name] += 1
            return UpdateResult(False, d2, y)
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        I_KH = np.eye(NX) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rn @ K.T  # Joseph form: stays symmetric PSD
        return UpdateResult(True, d2, y)


__all__ = ["ACCEL_MODES", "InsConfig", "InsEkf", "UpdateResult"]
