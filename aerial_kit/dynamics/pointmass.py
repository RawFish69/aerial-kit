"""Simple 3D point-mass dynamics: x_dot = v, v_dot = a_cmd - kv*v.

Used for high-level controller comparison (PID/LQR/MPC) without modeling
attitude.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PointMassParams:
    mass: float = 1.0
    kv_drag: float = 0.1


class PointMassDynamics:
    def __init__(self, params: PointMassParams | None = None):
        self.params = params or PointMassParams()
        self.position = np.array([0.0, 0.0, 1.0], dtype=float)
        self.velocity = np.zeros(3, dtype=float)

    @property
    def state(self) -> dict:
        return {
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
        }

    def step(self, acc_cmd: np.ndarray, dt: float) -> None:
        acc_cmd = np.asarray(acc_cmd, dtype=float)
        drag = -self.params.kv_drag * self.velocity
        acc = acc_cmd + drag
        self.velocity += acc * dt
        self.position += self.velocity * dt
