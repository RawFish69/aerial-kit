"""Simple UAV dynamics model with RK4 integration (multirotor-shaped).

Inspired by the ROS2 sim_dyn DynamicsNode but kept lighter-weight for offline
simulation and visualization: thrust along body +Z, linear drag, first-order
body-rate tracking to a commanded rate.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class DynamicsParams:
    sim_rate: float = 200.0
    mass: float = 1.0
    # Thrust command value that corresponds to hover.
    # With c0=0 and c1=1, hover_thrust=1.0 => thrust = mass * gravity.
    hover_thrust: float = 1.0
    c0: float = 0.0
    c1: float = 1.0
    kv_drag: float = 0.1
    gravity: float = 9.81


class UAVDynamics:
    """
    Continuous-time UAV dynamics with RK4 step integration.

    State:
      position (ENU): [x, y, z]
      velocity (ENU): [vx, vy, vz]
      attitude quaternion: [w, x, y, z]
      body rates (FRD): [p, q, r]
    """

    def __init__(self, params: DynamicsParams | None = None):
        self.params = params or DynamicsParams()

        self.position = np.array([0.0, 0.0, 1.0], dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.attitude_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.body_rates = np.zeros(3, dtype=float)

        # Command (can be set externally each step)
        self.cmd_body_rates = np.zeros(3, dtype=float)
        self.cmd_thrust = self.params.hover_thrust

    @property
    def state(self) -> dict:
        return {
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
            "attitude_quat": self.attitude_quat.copy(),
            "body_rates": self.body_rates.copy(),
        }

    def set_command(self, body_rates: np.ndarray, thrust: float) -> None:
        self.cmd_body_rates = np.asarray(body_rates, dtype=float)
        self.cmd_thrust = float(thrust)

    def step(self, dt: float) -> None:
        """Advance the state by one time step using RK4."""
        k1 = self._dynamics(self.position, self.velocity, self.attitude_quat, self.body_rates)

        pos2 = self.position + 0.5 * dt * k1["vel"]
        vel2 = self.velocity + 0.5 * dt * k1["acc"]
        quat2 = self._integrate_quaternion(self.attitude_quat, k1["omega_body"], 0.5 * dt)
        omega2 = self.body_rates + 0.5 * dt * k1["alpha"]
        k2 = self._dynamics(pos2, vel2, quat2, omega2)

        pos3 = self.position + 0.5 * dt * k2["vel"]
        vel3 = self.velocity + 0.5 * dt * k2["acc"]
        quat3 = self._integrate_quaternion(self.attitude_quat, k2["omega_body"], 0.5 * dt)
        omega3 = self.body_rates + 0.5 * dt * k2["alpha"]
        k3 = self._dynamics(pos3, vel3, quat3, omega3)

        pos4 = self.position + dt * k3["vel"]
        vel4 = self.velocity + dt * k3["acc"]
        quat4 = self._integrate_quaternion(self.attitude_quat, k3["omega_body"], dt)
        omega4 = self.body_rates + dt * k3["alpha"]
        k4 = self._dynamics(pos4, vel4, quat4, omega4)

        self.position += (dt / 6.0) * (k1["vel"] + 2 * k2["vel"] + 2 * k3["vel"] + k4["vel"])
        self.velocity += (dt / 6.0) * (k1["acc"] + 2 * k2["acc"] + 2 * k3["acc"] + k4["acc"])
        self.body_rates += (dt / 6.0) * (k1["alpha"] + 2 * k2["alpha"] + 2 * k3["alpha"] + k4["alpha"])

        omega_avg = (
            k1["omega_body"] + 2 * k2["omega_body"] + 2 * k3["omega_body"] + k4["omega_body"]
        ) / 6.0
        self.attitude_quat = self._integrate_quaternion(self.attitude_quat, omega_avg, dt)
        self.attitude_quat /= np.linalg.norm(self.attitude_quat)

    def _dynamics(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        quat: np.ndarray,
        omega_body: np.ndarray,
    ) -> dict:
        """
        Compute dynamics derivatives.

        We use a simple model similar to the ROS2 DynamicsNode:
        - Thrust along body z-axis, mapped to ENU
        - Linear drag
        - First-order body rate tracking to commanded rates
        """
        p = self.params

        thrust_mag = p.mass * p.gravity * (p.c0 + p.c1 * self.cmd_thrust)

        # Convert quaternion [w, x, y, z] to rotation matrix (ENU).
        w, x, y, z = quat
        R = np.array(
            [
                [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
            ]
        )

        # Thrust along body +Z in body frame => [0, 0, thrust_mag]
        thrust_world = R @ np.array([0.0, 0.0, thrust_mag])

        gravity_force = np.array([0.0, 0.0, -p.mass * p.gravity])
        drag_force = -p.kv_drag * vel
        acc = (thrust_world + gravity_force + drag_force) / p.mass

        tau = 0.1
        alpha = (self.cmd_body_rates - omega_body) / tau

        return {
            "vel": vel,
            "acc": acc,
            "omega_body": omega_body,
            "alpha": alpha,
        }

    @staticmethod
    def _integrate_quaternion(quat: np.ndarray, omega_body: np.ndarray, dt: float) -> np.ndarray:
        """Integrate quaternion with body rates."""
        w, x, y, z = quat
        p, q_rate, r = omega_body

        dq = 0.5 * np.array(
            [
                -x * p - y * q_rate - z * r,
                w * p + y * r - z * q_rate,
                w * q_rate - x * r + z * p,
                w * r + x * q_rate - y * p,
            ]
        )

        return quat + dt * dq
