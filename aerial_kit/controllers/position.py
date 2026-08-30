"""Position-level controllers for the standalone simulator.

These operate on a simple point-mass model:
  x_dot = v, v_dot = a_cmd
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_are


def pid_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    kp: float,
    kd: float,
) -> np.ndarray:
    """Compute acceleration command using simple PD on position."""
    pos = np.asarray(pos, dtype=float)
    vel = np.asarray(vel, dtype=float)
    target = np.asarray(target, dtype=float)

    pos_error = target - pos
    vel_error = -vel
    acc_cmd = kp * pos_error + kd * vel_error
    return acc_cmd


def lqr_gain_double_integrator(q_pos: float, q_vel: float, r_acc: float) -> np.ndarray:
    """Compute continuous-time LQR gain for a 1D double integrator."""
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    Q = np.diag([q_pos, q_vel])
    R = np.array([[r_acc]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K


def lqr_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    q_pos: float = 10.0,
    q_vel: float = 2.0,
    r_acc: float = 1.0,
) -> np.ndarray:
    """LQR state-feedback for each axis of a double integrator."""
    pos = np.asarray(pos, dtype=float)
    vel = np.asarray(vel, dtype=float)
    target = np.asarray(target, dtype=float)

    x_err = np.stack([pos - target, vel], axis=-1)
    K = lqr_gain_double_integrator(q_pos, q_vel, r_acc)
    acc_cmd = -np.einsum("ij,bj->bi", K, x_err).squeeze(-1)
    return acc_cmd


def mpc_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    q_pos: float,
    q_vel: float,
    r_acc: float,
) -> np.ndarray:
    """Very lightweight MPC-like controller."""
    return lqr_position_control(pos, vel, target, q_pos=q_pos, q_vel=q_vel, r_acc=r_acc)


__all__ = [
    "pid_position_control",
    "lqr_gain_double_integrator",
    "lqr_position_control",
    "mpc_position_control",
]
