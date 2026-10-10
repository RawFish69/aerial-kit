"""Horizon references sampled from a waypoint polyline.

A predictive controller wants to know where the aircraft should be at each of
the next ``N`` steps, not just where the next waypoint is. Handing it the
waypoint alone makes it a point stabiliser: it plans to *arrive* at every
corner, braking into each one. Handing it a reference that moves along the path
at cruise speed makes it a tracker, and lets it cut the corner it can see
coming.

:class:`PathReference` turns a polyline into that reference. It is the same
code on both sides of the repo: the standalone simulator uses it, and so does
the ROS 2 ``uav_control`` tracker.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class HorizonReference:
    """Positions and velocities for steps ``1..N`` of a horizon, world frame."""

    positions: np.ndarray  # (N, 3)
    velocities: np.ndarray  # (N, 3)
    progress_m: float  # arc length at the current position
    remaining_m: float  # arc length from the current position to the end


class PathReference:
    """Arc-length parameterised polyline with a speed profile that stops at the end.

    ``decel_mps2`` shapes the approach to the final point: the reference speed
    is ``min(cruise, sqrt(2 * decel * remaining))``, so it arrives at the last
    waypoint with zero velocity instead of overshooting it.
    """

    def __init__(self, waypoints: np.ndarray, *, decel_mps2: float = 1.5) -> None:
        pts = np.asarray(waypoints, dtype=float).reshape(-1, 3)
        if pts.shape[0] == 0:
            raise ValueError("a path needs at least one waypoint")
        if not np.all(np.isfinite(pts)):
            raise ValueError("path waypoints must be finite")
        # Drop repeated points: a zero-length segment has no direction.
        keep = [0]
        for i in range(1, pts.shape[0]):
            if np.linalg.norm(pts[i] - pts[keep[-1]]) > 1e-9:
                keep.append(i)
        self.points = pts[keep]
        seg = np.diff(self.points, axis=0)
        self.segment_lengths = np.linalg.norm(seg, axis=1)
        self.cumulative = np.concatenate([[0.0], np.cumsum(self.segment_lengths)])
        self.length = float(self.cumulative[-1])
        self.decel_mps2 = max(float(decel_mps2), 1e-3)

    def point_at(self, s: float) -> np.ndarray:
        """Position at arc length ``s`` (clamped to the path)."""
        return self._interp(np.array([s]))[0][0]

    def _interp(self, s: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Positions and unit tangents at arc lengths ``s``."""
        s = np.clip(np.asarray(s, dtype=float), 0.0, self.length)
        if self.points.shape[0] == 1:
            return np.repeat(self.points, s.size, axis=0), np.zeros((s.size, 3))
        idx = np.clip(np.searchsorted(self.cumulative, s, side="right") - 1, 0, len(self.segment_lengths) - 1)
        start = self.points[idx]
        seg = self.points[idx + 1] - start
        seg_len = self.segment_lengths[idx][:, None]
        frac = (s - self.cumulative[idx])[:, None] / seg_len
        return start + frac * seg, seg / seg_len

    def project(self, position: np.ndarray, *, s_min: float = 0.0, s_max: float | None = None) -> float:
        """Arc length of the closest point on the path to ``position``.

        ``s_min``/``s_max`` restrict the search window; a tracker passes its
        last progress as ``s_min`` so a path that doubles back on itself does
        not make the reference jump backwards.
        """
        p = np.asarray(position, dtype=float).reshape(3)
        s_hi = self.length if s_max is None else min(float(s_max), self.length)
        s_lo = max(0.0, min(float(s_min), s_hi))
        if self.points.shape[0] == 1 or s_hi - s_lo <= 1e-12:
            return s_lo

        best_s, best_d = s_lo, np.inf
        for i, L in enumerate(self.segment_lengths):
            a, b = self.cumulative[i], self.cumulative[i + 1]
            if b < s_lo or a > s_hi:
                continue
            d = (self.points[i + 1] - self.points[i]) / L
            t = float(np.dot(p - self.points[i], d))
            t = min(max(t, s_lo - a, 0.0), min(s_hi - a, L))
            dist = float(np.linalg.norm(self.points[i] + t * d - p))
            if dist < best_d - 1e-12:
                best_s, best_d = a + t, dist
        return best_s

    def speed_at(self, s: np.ndarray, cruise_mps: float) -> np.ndarray:
        remaining = np.maximum(self.length - np.asarray(s, dtype=float), 0.0)
        return np.minimum(float(cruise_mps), np.sqrt(2.0 * self.decel_mps2 * remaining))

    def sample(self, s0: float, *, cruise_mps: float, dt: float, horizon: int) -> HorizonReference:
        """March ``horizon`` steps of ``dt`` along the path from arc length ``s0``."""
        horizon = max(int(horizon), 1)
        dt = max(float(dt), 1e-4)
        s = float(np.clip(s0, 0.0, self.length))
        s_steps = np.empty(horizon)
        for k in range(horizon):
            # Integrate the speed profile rather than assume a constant speed,
            # so the reference itself decelerates into the final point.
            v = float(self.speed_at(np.array([s]), cruise_mps)[0])
            s = min(s + v * dt, self.length)
            s_steps[k] = s
        positions, tangents = self._interp(s_steps)
        velocities = tangents * self.speed_at(s_steps, cruise_mps)[:, None]
        return HorizonReference(positions, velocities, float(s0), self.length - float(s0))


def constant_reference(target: np.ndarray, horizon: int) -> HorizonReference:
    """Hold-at-``target`` reference: every step at the target, at rest."""
    t = np.asarray(target, dtype=float).reshape(1, 3)
    return HorizonReference(np.repeat(t, horizon, axis=0), np.zeros((horizon, 3)), 0.0, 0.0)


__all__ = ["HorizonReference", "PathReference", "constant_reference"]
