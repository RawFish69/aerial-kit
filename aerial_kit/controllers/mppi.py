"""Warm-started, vectorised MPPI for a point-mass (double-integrator) model.

``mppi_position_control`` in :mod:`.position` is a stateless function: it
samples around *zero* every call, loops over samples in Python, and has nothing
to say about obstacles. Each of those limits it in a way that matters once the
controller is in a loop:

* Sampling around zero throws the previous plan away. MPPI is an iterative
  method that happens to run one iteration per control step - its quality comes
  from refining the *same* nominal sequence step after step, which
  :class:`MPPI` keeps and shifts.
* A Python loop over 300 samples x 12 steps is the whole runtime. Here the
  rollouts are a pair of cumulative sums over a ``(K, N, 3)`` array.
* Its strength over a QP-based MPC is that the cost can be anything you can
  evaluate on a batch of trajectories: non-convex obstacle terms, a floor,
  a speed limit as a soft penalty. :class:`MPPI` takes obstacles - spheres,
  vertical cylinders (trees) and axis-aligned boxes, all as signed-distance
  functions - and a minimum altitude, which a linear MPC cannot express.

The update follows Williams et al. (2017): rollouts ``U_k = clip(U_nom + eps_k)``,
weights ``w_k ~ exp(-(S_k - min S) / lambda)``, ``U_nom <- sum w_k U_k``.

``lambda`` is relative by default: ``temperature * (median S - min S)``. An
absolute ``lambda`` only suits one cost scale, and these costs scale with the
weights, the horizon and the distance to the reference. At the first absolute
default (1.0) the effective sample size was 2 of 514: the "average" was the
single best noisy rollout, and the command was 2.6x rougher than it needed to
be. ``temperature_mode="absolute"`` keeps the textbook form. Two
extra rollouts are always included - the unperturbed nominal and a "brake"
sequence of zero acceleration - so a bad batch of noise can never make the
update worse than both of them.

Same model, units and frame as :mod:`.mpc`: world frame, z up, the acceleration
is a demand without gravity.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np

from .reference import HorizonReference, constant_reference


@dataclass(frozen=True)
class SphereObstacle:
    """A sphere the trajectory must stay out of (plus the controller's margin)."""

    center: tuple[float, float, float]
    radius: float

    def sdf(self, points: np.ndarray) -> np.ndarray:
        """Signed distance from ``points`` (..., 3): negative inside."""
        return np.linalg.norm(points - np.asarray(self.center, dtype=float), axis=-1) - float(self.radius)


@dataclass(frozen=True)
class CylinderObstacle:
    """A vertical cylinder - a tree trunk - from ``z_min`` to ``z_max``."""

    center_xy: tuple[float, float]
    radius: float
    z_min: float
    z_max: float

    def sdf(self, points: np.ndarray) -> np.ndarray:
        d_h = np.linalg.norm(points[..., :2] - np.asarray(self.center_xy, dtype=float), axis=-1) - float(self.radius)
        d_v = np.maximum(float(self.z_min) - points[..., 2], points[..., 2] - float(self.z_max))
        outside = np.sqrt(np.maximum(d_h, 0.0) ** 2 + np.maximum(d_v, 0.0) ** 2)
        return outside + np.minimum(np.maximum(d_h, d_v), 0.0)


@dataclass(frozen=True)
class BoxObstacle:
    """An axis-aligned box: ``center`` and ``half_size`` (half the edge lengths)."""

    center: tuple[float, float, float]
    half_size: tuple[float, float, float]

    def sdf(self, points: np.ndarray) -> np.ndarray:
        q = np.abs(points - np.asarray(self.center, dtype=float)) - np.asarray(self.half_size, dtype=float)
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
        return outside + np.minimum(np.max(q, axis=-1), 0.0)


@dataclass
class MPPISolution:
    """Result of one MPPI update."""

    accel: np.ndarray  # (3,) first acceleration of the updated nominal plan
    accel_sequence: np.ndarray  # (N, 3)
    predicted_positions: np.ndarray  # (N, 3), steps 1..N of the nominal plan
    predicted_velocities: np.ndarray  # (N, 3)
    min_cost: float
    effective_samples: float  # 1 / sum(w^2): how many rollouts carried the update
    metadata: dict = field(default_factory=dict)


def rollout(
    position: np.ndarray, velocity: np.ndarray, accels: np.ndarray, dt: float
) -> tuple[np.ndarray, np.ndarray]:
    """Exact double-integrator rollouts.

    ``accels`` is ``(..., N, 3)``; returns positions and velocities at steps
    ``1..N`` with the same leading shape.
    """
    v0 = np.asarray(velocity, dtype=float)
    p0 = np.asarray(position, dtype=float)
    vel = v0 + dt * np.cumsum(accels, axis=-2)
    vel_prev = np.concatenate([np.broadcast_to(v0, accels[..., :1, :].shape), vel[..., :-1, :]], axis=-2)
    pos = p0 + np.cumsum(vel_prev * dt + 0.5 * dt * dt * accels, axis=-2)
    return pos, vel


class MPPI:
    """Model predictive path integral controller with a persistent nominal plan."""

    def __init__(
        self,
        *,
        dt: float = 0.1,
        horizon: int = 20,
        samples: int = 512,
        temperature: float = 0.1,
        temperature_mode: str = "relative",
        noise_std: float | Sequence[float] = 2.0,
        q_pos: float = 8.0,
        q_vel: float = 1.0,
        q_terminal: float = 20.0,
        r_acc: float = 0.05,
        max_accel_xy: float = 6.0,
        max_accel_z: float = 4.0,
        max_speed: float | None = None,
        speed_penalty: float = 50.0,
        obstacles: Iterable = (),
        obstacle_margin: float = 0.5,
        obstacle_penalty: float = 1e4,
        min_altitude: float | None = None,
        altitude_penalty: float = 1e3,
        seed: int | None = None,
    ) -> None:
        self.dt = max(float(dt), 1e-4)
        self.horizon = max(int(horizon), 1)
        self.samples = max(int(samples), 1)
        if temperature <= 0.0:
            raise ValueError("temperature must be positive")
        self.temperature = float(temperature)
        self.temperature_mode = str(temperature_mode).lower()
        if self.temperature_mode not in {"absolute", "relative"}:
            raise ValueError("temperature_mode must be 'absolute' or 'relative'")
        std = np.broadcast_to(np.asarray(noise_std, dtype=float), (3,)).copy()
        if np.any(std < 0.0):
            raise ValueError("noise_std must be non-negative")
        self.noise_std = std
        self.q_pos, self.q_vel = float(q_pos), float(q_vel)
        self.q_terminal, self.r_acc = float(q_terminal), float(r_acc)
        if max_accel_xy <= 0.0 or max_accel_z <= 0.0:
            raise ValueError("acceleration limits must be positive")
        self.accel_limit = np.array([max_accel_xy, max_accel_xy, max_accel_z], dtype=float)
        self.max_speed = None if max_speed is None else float(max_speed)
        self.speed_penalty = float(speed_penalty)
        self.obstacles = list(obstacles)
        self.obstacle_margin = float(obstacle_margin)
        self.obstacle_penalty = float(obstacle_penalty)
        self.min_altitude = None if min_altitude is None else float(min_altitude)
        self.altitude_penalty = float(altitude_penalty)
        self._rng = np.random.default_rng(seed)
        self._nominal = np.zeros((self.horizon, 3))

    @property
    def nominal(self) -> np.ndarray:
        return self._nominal.copy()

    def reset(self, seed: int | None = None) -> None:
        """Zero the nominal plan; optionally reseed the noise."""
        self._nominal = np.zeros((self.horizon, 3))
        if seed is not None:
            self._rng = np.random.default_rng(seed)

    def trajectory_costs(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        accels: np.ndarray,
        reference: HorizonReference,
        obstacles: Sequence | None = None,
    ) -> np.ndarray:
        """Cost of each rollout. Arrays are ``(K, N, 3)``; returns ``(K,)``."""
        ref_p = reference.positions[None]
        ref_v = reference.velocities[None]
        e_p = positions - ref_p
        e_v = velocities - ref_v
        cost = self.q_pos * np.sum(e_p[:, :-1] ** 2, axis=(1, 2))
        cost += self.q_vel * np.sum(e_v ** 2, axis=(1, 2))
        cost += self.q_terminal * np.sum(e_p[:, -1] ** 2, axis=-1)
        cost += self.r_acc * np.sum(accels ** 2, axis=(1, 2))

        if self.max_speed is not None:
            excess = np.maximum(np.linalg.norm(velocities, axis=-1) - self.max_speed, 0.0)
            cost += self.speed_penalty * np.sum(excess ** 2, axis=1)
        if self.min_altitude is not None:
            below = np.maximum(self.min_altitude - positions[..., 2], 0.0)
            cost += self.altitude_penalty * np.sum(below ** 2, axis=1)

        obs = self.obstacles if obstacles is None else list(obstacles)
        for o in obs:
            # Every obstacle shape is a signed distance; "inside" is within the
            # controller's margin of the surface.
            depth = np.maximum(self.obstacle_margin - o.sdf(positions), 0.0)
            # A hard per-step penalty for being inside, plus a smooth term so
            # rollouts that are less deep inside are still ranked above others.
            cost += self.obstacle_penalty * np.sum((depth > 0.0) + depth ** 2, axis=1)
        return cost

    def solve(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        reference: HorizonReference | np.ndarray,
        *,
        obstacles: Sequence | None = None,
    ) -> MPPISolution:
        """Run one MPPI update from ``(position, velocity)`` and shift the plan."""
        p0 = np.asarray(position, dtype=float).reshape(3)
        v0 = np.asarray(velocity, dtype=float).reshape(3)
        if not (np.all(np.isfinite(p0)) and np.all(np.isfinite(v0))):
            raise ValueError("MPPI received a non-finite state")
        N, K = self.horizon, self.samples
        if not isinstance(reference, HorizonReference):
            reference = constant_reference(np.asarray(reference, dtype=float), N)
        if reference.positions.shape != (N, 3) or reference.velocities.shape != (N, 3):
            raise ValueError(f"reference must have {N} steps")

        lim = self.accel_limit
        noise = self._rng.standard_normal((K, N, 3)) * self.noise_std
        U = np.clip(self._nominal[None] + noise, -lim, lim)
        U = np.concatenate([U, self._nominal[None], np.zeros((1, N, 3))], axis=0)

        pos, vel = rollout(p0, v0, U, self.dt)
        costs = self.trajectory_costs(pos, vel, U, reference, obstacles)

        min_cost = float(np.min(costs))
        lam = self.temperature
        if self.temperature_mode == "relative":
            lam *= max(float(np.median(costs)) - min_cost, 1e-9)
        w = np.exp(-(costs - min_cost) / lam)
        w /= float(np.sum(w))
        self._nominal = np.clip(np.tensordot(w, U, axes=1), -lim, lim)

        plan = self._nominal.copy()
        plan_p, plan_v = rollout(p0, v0, plan, self.dt)
        # Shift for the next call: the first step is being applied now.
        self._nominal = np.vstack([plan[1:], plan[-1:]])

        return MPPISolution(
            accel=plan[0].copy(),
            accel_sequence=plan,
            predicted_positions=plan_p,
            predicted_velocities=plan_v,
            min_cost=min_cost,
            effective_samples=float(1.0 / np.sum(w ** 2)),
            metadata={"samples": K + 2},
        )


Obstacle = SphereObstacle | CylinderObstacle | BoxObstacle

__all__ = ["BoxObstacle", "CylinderObstacle", "MPPI", "MPPISolution", "Obstacle", "SphereObstacle", "rollout"]
