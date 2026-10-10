"""Piecewise minimum-snap trajectories through waypoints (Mellinger & Kumar, 2011).

:func:`aerial_kit.controllers.minimum_snap_trajectory` fits *one* polynomial
through all the waypoints and returns samples. That is fine for a smooth curve
to look at; it is not something a tracking controller can fly, because it does
not start or stop at rest, its length is limited by one polynomial's order, and
it cannot be asked "where should I be at t = 3.7 s, how fast, accelerating how
hard, with what jerk?".

:class:`MinSnapTrajectory` can. One 7th-order polynomial per segment, with

* rest at both ends (velocity, acceleration and jerk zero, unless given),
* the waypoints hit exactly, and velocity, acceleration, jerk and snap
  continuous through them,
* the integrated squared snap minimised over what is left - two degrees of
  freedom per interior waypoint and axis,
* segment durations first allocated from one trapezoidal speed profile over
  the whole path, then redistributed to minimise the snap cost at fixed total
  time (:meth:`MinSnapTrajectory.optimize_segment_times`), then the whole
  trajectory rescaled uniformly until the tightest of the speed, acceleration
  and jerk limits is just met - sped up as often as slowed down.

It is smooth, not time-optimal: minimising snap prefers gentle ramps, so a
long straight flight spends longer speeding up and slowing down than a
bang-bang profile would (121 m at 3 m/s: about 60 s against 43 s). Uniform scaling is exact: stretching every
  duration by ``s`` keeps the same optimal path and divides the k-th derivative
  by ``s**k``.

Each segment's polynomial is in normalised time ``tau = (t - t_i) / T_i`` in
``[0, 1]``, which keeps the linear system well conditioned for segments of very
different lengths (the reason the global fit next door moved to a Chebyshev
basis).

:meth:`MinSnapTrajectory.flat_reference` turns it into the
:class:`~aerial_kit.controllers.geometric.FlatReference` the geometric
controller and the NMPC track, jerk included, with yaw fixed or along the
direction of travel.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import factorial
from typing import Callable

import numpy as np

ORDER = 7
NC = ORDER + 1  # coefficients per segment
CONTINUITY = 4  # derivatives continuous through interior waypoints: vel, acc, jerk, snap


def _deriv_coeffs(r: int) -> np.ndarray:
    """``k! / (k - r)!`` for k = 0..7 (zero where k < r)."""
    return np.array([factorial(k) / factorial(k - r) if k >= r else 0.0 for k in range(NC)])


def _basis_row(tau: float, r: int) -> np.ndarray:
    """Row ``a`` with ``a . c = d^r p / d tau^r`` at ``tau``."""
    d = _deriv_coeffs(r)
    out = np.zeros(NC)
    for k in range(r, NC):
        out[k] = d[k] * tau ** (k - r)
    return out


def _snap_cost_tau() -> np.ndarray:
    """``Q`` with ``c^T Q c = int_0^1 (d^4 p / d tau^4)^2 d tau``."""
    d = _deriv_coeffs(4)
    Q = np.zeros((NC, NC))
    for k in range(4, NC):
        for m in range(4, NC):
            Q[k, m] = d[k] * d[m] / (k + m - 7)
    return Q


_Q_TAU = _snap_cost_tau()


def path_trapezoid_durations(distances: np.ndarray, v_max: float, a_max: float) -> np.ndarray:
    """Segment durations from *one* trapezoidal speed profile over the whole path.

    Each segment gets the time that profile spends on it: accelerate from
    rest, cruise at ``v_max``, decelerate to rest at the end. Allocating every
    segment as its own rest-to-rest trapezoid instead makes the minimum-snap
    speed rise and dip inside every segment, so the peak meets ``v_max`` while
    the average is half of it - the first version of this flew a straight
    121 m path in 81 s at a 3 m/s limit.
    """
    d = np.asarray(distances, dtype=float)
    total = float(d.sum())
    s_ramp = min(v_max * v_max / (2.0 * a_max), total / 2.0)
    v_peak = np.sqrt(2.0 * a_max * s_ramp)
    t_ramp = v_peak / a_max
    t_cruise = (total - 2.0 * s_ramp) / v_max if v_peak >= v_max - 1e-12 else 0.0

    def time_at(s: float) -> float:
        if s <= s_ramp:
            return np.sqrt(2.0 * s / a_max)
        if s <= total - s_ramp:
            return t_ramp + (s - s_ramp) / v_peak
        rem = total - s
        return 2.0 * t_ramp + t_cruise - np.sqrt(2.0 * max(rem, 0.0) / a_max)

    cum = np.concatenate([[0.0], np.cumsum(d)])
    times = np.array([time_at(float(x)) for x in cum])
    return np.maximum(np.diff(times), 1e-3)


def trapezoid_time(distance: float, v_max: float, a_max: float) -> float:
    """Rest-to-rest time over ``distance`` with speed and acceleration limits."""
    if distance <= 0.0:
        return 0.0
    if distance > v_max * v_max / a_max:
        return distance / v_max + v_max / a_max
    return 2.0 * np.sqrt(distance / a_max)


@dataclass
class Limits:
    v_max: float = 3.0
    a_max: float = 3.0
    j_max: float | None = 10.0


class MinSnapTrajectory:
    """Piecewise 7th-order minimum-snap trajectory, queryable at any time."""

    def __init__(
        self,
        waypoints: np.ndarray,
        durations: np.ndarray | None = None,
        *,
        limits: Limits | None = None,
        start_derivatives: np.ndarray | None = None,
        end_derivatives: np.ndarray | None = None,
        t0: float = 0.0,
        optimize_times: bool = True,
    ) -> None:
        W = np.asarray(waypoints, dtype=float)
        if W.ndim != 2 or W.shape[1] != 3 or W.shape[0] < 2:
            raise ValueError("waypoints must be (M >= 2, 3)")
        if not np.all(np.isfinite(W)):
            raise ValueError("waypoints must be finite")
        # A repeated waypoint is a zero-length segment: no duration fits it.
        keep = [0] + [i for i in range(1, len(W)) if np.linalg.norm(W[i] - W[i - 1]) > 1e-9]
        self.waypoints = W[keep]
        if len(self.waypoints) < 2:
            raise ValueError("need at least two distinct waypoints")
        self.limits = limits or Limits()
        if self.limits.v_max <= 0.0 or self.limits.a_max <= 0.0:
            raise ValueError("v_max and a_max must be > 0")
        self.start_derivatives = np.zeros((3, 3)) if start_derivatives is None else np.asarray(start_derivatives, dtype=float).reshape(3, 3)
        self.end_derivatives = np.zeros((3, 3)) if end_derivatives is None else np.asarray(end_derivatives, dtype=float).reshape(3, 3)
        self.t0 = float(t0)

        n_seg = len(self.waypoints) - 1
        if durations is None:
            dist = np.linalg.norm(np.diff(self.waypoints, axis=0), axis=1)
            self._solve(path_trapezoid_durations(dist, self.limits.v_max, self.limits.a_max))
            if optimize_times and n_seg > 1:
                self.optimize_segment_times()
            self.scale_to_limits()
        else:
            T = np.asarray(durations, dtype=float).reshape(-1)
            if T.shape != (n_seg,) or np.any(T <= 0.0):
                raise ValueError(f"durations must be {n_seg} positive values")
            self._solve(T)

    # -- construction ---------------------------------------------------------
    def _solve(self, durations: np.ndarray) -> None:
        """Equality-constrained QP per axis, solved through its KKT system."""
        self.durations = np.asarray(durations, dtype=float).copy()
        self.times = self.t0 + np.concatenate([[0.0], np.cumsum(self.durations)])
        n_seg = len(self.durations)
        n = n_seg * NC
        H = np.zeros((n, n))
        for i, T in enumerate(self.durations):
            # Snap in real time: d^4/dt^4 = T^-4 d^4/dtau^4, dt = T dtau.
            H[i * NC:(i + 1) * NC, i * NC:(i + 1) * NC] = _Q_TAU / T**7

        T = self.durations
        # The constraint rows are the same for x, y and z; only the values
        # differ, so each row is built once with a tag that says where its
        # value comes from.
        specs = []  # (row parts, (kind, data))
        # Start: position and 3 derivatives.
        for r in range(4):
            specs.append(([(0, _basis_row(0.0, r) / T[0] ** r)], ("start", r)))
        # End.
        for r in range(4):
            specs.append(([(n_seg - 1, _basis_row(1.0, r) / T[-1] ** r)], ("end", r)))
        # Interior: position on both sides, continuity of derivatives 1..4.
        for j in range(1, n_seg):
            specs.append(([(j - 1, _basis_row(1.0, 0))], ("wp", j)))
            specs.append(([(j, _basis_row(0.0, 0))], ("wp", j)))
            for r in range(1, CONTINUITY + 1):
                specs.append(([(j - 1, _basis_row(1.0, r) / T[j - 1] ** r), (j, -_basis_row(0.0, r) / T[j] ** r)], ("zero", r)))

        A = np.zeros((len(specs), n))
        for k, (parts, _) in enumerate(specs):
            for seg, vec in parts:
                A[k, seg * NC:(seg + 1) * NC] += vec
        m = A.shape[0]
        KKT = np.zeros((n + m, n + m))
        KKT[:n, :n] = 2.0 * H
        KKT[:n, n:] = A.T
        KKT[n:, :n] = A
        coeffs = np.zeros((n_seg, NC, 3))
        for axis in range(3):
            b = np.zeros(m)
            for k, (_, (kind, data)) in enumerate(specs):
                if kind == "start":
                    b[k] = self.waypoints[0, axis] if data == 0 else self.start_derivatives[data - 1, axis]
                elif kind == "end":
                    b[k] = self.waypoints[-1, axis] if data == 0 else self.end_derivatives[data - 1, axis]
                elif kind == "wp":
                    b[k] = self.waypoints[data, axis]
            sol = np.linalg.solve(KKT, np.concatenate([np.zeros(n), b]))
            coeffs[:, :, axis] = sol[:n].reshape(n_seg, NC)
        self.coeffs = coeffs  # (segment, k, axis), normalised time

    def snap_cost(self) -> float:
        """Integrated squared snap over the whole trajectory, all axes."""
        return float(sum(
            np.einsum("ka,kl,la->", self.coeffs[i], _Q_TAU / T**7, self.coeffs[i])
            for i, T in enumerate(self.durations)
        ))

    def optimize_segment_times(self, iterations: int = 25) -> None:
        """Redistribute time between segments to minimise snap, total time fixed.

        With every waypoint time fixed by an allocation rule, the start and
        end transients ring through the spline as a slowly decaying speed
        oscillation: on a straight 121 m path the speed swung between 1 and
        2.9 m/s, and the uniform rescale that holds the peak at the limit then
        made the whole flight slow. Moving time between segments to lower the
        total snap (Richter, Bry & Roy, 2013) removes the ringing; the total
        is then set by :meth:`scale_to_limits`. Projected gradient descent on
        log-durations, gradient by finite differences.
        """
        total = float(self.durations.sum())
        log_t = np.log(self.durations)

        def cost_at(lt: np.ndarray) -> float:
            T = np.exp(lt)
            self._solve(T * (total / T.sum()))
            return self.snap_cost()

        J = cost_at(log_t)
        step = 0.2
        h = 1e-4
        for _ in range(iterations):
            grad = np.empty_like(log_t)
            for i in range(log_t.size):
                e = np.zeros_like(log_t)
                e[i] = h
                grad[i] = (cost_at(log_t + e) - J) / h
            grad -= grad.mean()  # stay on the fixed-total-time surface
            norm = float(np.linalg.norm(grad))
            if norm < 1e-12:
                break
            improved = False
            while step > 1e-4:
                cand = log_t - step * grad / norm
                Jc = cost_at(cand)
                if Jc < J:
                    log_t, J, step, improved = cand, Jc, step * 1.5, True
                    break
                step *= 0.5
            if not improved:
                break
        cost_at(log_t)

    def max_derivatives(self, samples_per_segment: int = 60) -> tuple[float, float, float]:
        """Peak speed, acceleration and jerk along the trajectory (sampled)."""
        ts = np.concatenate([
            np.linspace(a, b, samples_per_segment, endpoint=False) for a, b in zip(self.times[:-1], self.times[1:])
        ] + [[self.times[-1]]])
        v = np.linalg.norm(self.evaluate(ts, 1), axis=1).max()
        a = np.linalg.norm(self.evaluate(ts, 2), axis=1).max()
        j = np.linalg.norm(self.evaluate(ts, 3), axis=1).max()
        return float(v), float(a), float(j)

    def scale_to_limits(self) -> float:
        """Rescale every duration so the tightest limit is just met; returns the factor.

        A factor below one speeds the trajectory up. The peaks are sampled, so
        it is applied twice, the second time correcting what sampling missed.
        """
        total = 1.0
        lim = self.limits
        for _ in range(2):
            v, a, j = self.max_derivatives()
            factor = max(v / lim.v_max, np.sqrt(a / lim.a_max), np.cbrt(j / lim.j_max) if lim.j_max else 0.0)
            if factor <= 0.0 or abs(factor - 1.0) < 1e-3:
                break
            factor *= 1.0005  # land just inside the limit, not on it
            self._solve(self.durations * factor)
            total *= factor
        return total

    # -- evaluation -------------------------------------------------------------
    @property
    def duration(self) -> float:
        return float(self.times[-1] - self.times[0])

    def evaluate(self, t, derivative: int = 0) -> np.ndarray:
        """The ``derivative``-th time derivative at ``t`` (scalar or array); clamped to the ends."""
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        t_c = np.clip(t_arr, self.times[0], self.times[-1])
        seg = np.clip(np.searchsorted(self.times, t_c, side="right") - 1, 0, len(self.durations) - 1)
        T = self.durations[seg]
        tau = (t_c - self.times[seg]) / T
        d = _deriv_coeffs(derivative)
        out = np.zeros((t_arr.size, 3))
        for k in range(derivative, NC):
            out += (d[k] * tau ** (k - derivative))[:, None] * self.coeffs[seg, k, :]
        out /= (T ** derivative)[:, None]
        # Past either end the trajectory holds its end state: at rest, unless
        # end derivatives were given, which then are not extrapolated.
        if derivative > 0:
            outside = (t_arr < self.times[0]) | (t_arr > self.times[-1])
            out[outside] = 0.0
        return out[0] if np.ndim(t) == 0 else out

    def flat_reference(self, t: float, yaw: float | str | None = None):
        """A :class:`FlatReference` at ``t``.

        ``yaw`` is a fixed heading in radians, ``"velocity"`` to face the
        direction of travel (heading held while nearly stopped), or ``None`` to
        leave the heading to the controller.
        """
        from ..controllers.geometric import FlatReference

        p, v, a, j = (self.evaluate(t, r) for r in range(4))
        yaw_val, yaw_rate = None, 0.0
        if isinstance(yaw, str):
            if yaw != "velocity":
                raise ValueError("yaw must be a number, 'velocity' or None")
            speed2 = v[0] ** 2 + v[1] ** 2
            if speed2 > 0.04:
                yaw_val = float(np.arctan2(v[1], v[0]))
                yaw_rate = float((v[0] * a[1] - v[1] * a[0]) / speed2)
            else:
                # Hold the last meaningful heading near standstill.
                yaw_val = self._held_yaw(t)
        elif yaw is not None:
            yaw_val = float(yaw)
        return FlatReference(position=p, velocity=v, acceleration=a, jerk=j, yaw=yaw_val, yaw_rate=yaw_rate)

    def _held_yaw(self, t: float) -> float | None:
        ts = np.linspace(self.times[0], self.times[-1], 400)
        v = self.evaluate(ts, 1)
        moving = (v[:, 0] ** 2 + v[:, 1] ** 2) > 0.04
        if not moving.any():
            return None
        idx = np.nonzero(moving)[0]
        before = idx[ts[idx] <= t]
        k = before[-1] if before.size else idx[0]
        return float(np.arctan2(v[k, 1], v[k, 0]))

    def as_reference(self, yaw: float | str | None = None) -> Callable[[float], object]:
        """``t -> FlatReference``, the form the attitude-level controllers take."""
        return lambda t: self.flat_reference(t, yaw)


def thin_waypoints(points: np.ndarray, min_spacing: float = 3.0, max_turn_deg: float = 20.0) -> np.ndarray:
    """Drop planner waypoints that add nothing: closer than ``min_spacing`` to
    the last kept one and turning less than ``max_turn_deg``. Grid planners
    return a point every cell; a minimum-snap fit through all of them is many
    tiny segments that each have to start and end their curvature."""
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(P) <= 2:
        return P
    kept = [P[0]]
    for i in range(1, len(P) - 1):
        d_prev = P[i] - kept[-1]
        d_next = P[i + 1] - P[i]
        dist = float(np.linalg.norm(d_prev))
        if dist < 1e-9:
            continue
        cosang = float(np.dot(d_prev, d_next) / (dist * max(np.linalg.norm(d_next), 1e-9)))
        turn = np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))
        if dist >= min_spacing and turn >= max_turn_deg:
            kept.append(P[i])
        elif dist >= 4.0 * min_spacing:
            kept.append(P[i])
    kept.append(P[-1])
    return np.array(kept)


__all__ = ["Limits", "MinSnapTrajectory", "path_trapezoid_durations", "thin_waypoints", "trapezoid_time"]
