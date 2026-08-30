"""L1 lateral guidance: bank-angle command toward a target point.

Classic L1 guidance (Park, Deyst & How) steers along a *path segment* using a
lookahead point L1 ahead on that segment. This codebase's ``Controller``
interface (``compute(state, target_waypoint, cfg)``) only hands a controller a
single target point per step, not the segment it lies on, so this is the
point-target variant -- closer to pure pursuit than full L1: the lookahead
vector is simply the vector from the vehicle to the target, capped implicitly
by whatever ``l1_distance_m`` is configured to. It reduces to the textbook
formula exactly when the vehicle is on the path and the target IS the L1
lookahead point.
"""

from __future__ import annotations

import numpy as np


def l1_bank_command(
    position_xy: np.ndarray,
    velocity_xy: np.ndarray,
    target_xy: np.ndarray,
    airspeed_mps: float,
    l1_distance_m: float,
    max_bank_rad: float,
    gravity_mps2: float = 9.81,
) -> float:
    """Bank command (rad) to steer the ground track toward ``target_xy``.

    ``eta`` is the signed angle from the current track direction to the
    line-of-sight toward the target (right-handed, world XY plane, positive
    = target to the left of track). The standard L1 lateral acceleration law
    ``2*V^2/L1*sin(eta)`` gives a desired acceleration positive-left; a
    coordinated turn produces lateral acceleration ``g*tan(phi)`` toward the
    *right* for a positive (standard aviation) bank angle, so achieving a
    positive-left acceleration needs the *negated* bank: ``atan(-a_cmd/g)``.
    Clamped to ``max_bank_rad``.
    """
    to_target = np.asarray(target_xy, dtype=float) - np.asarray(position_xy, dtype=float)
    dist = float(np.linalg.norm(to_target))
    if dist < 1e-6 or airspeed_mps < 1e-3:
        return 0.0

    speed = float(np.linalg.norm(velocity_xy))
    track_dir = np.asarray(velocity_xy, dtype=float) / speed if speed > 1e-3 else to_target / dist

    eta = float(
        np.arctan2(
            track_dir[0] * to_target[1] - track_dir[1] * to_target[0],
            track_dir[0] * to_target[0] + track_dir[1] * to_target[1],
        )
    )
    l1_distance_m = max(float(l1_distance_m), 1e-3)
    a_cmd = 2.0 * airspeed_mps**2 / l1_distance_m * np.sin(eta)
    phi_cmd = float(np.arctan2(-a_cmd, gravity_mps2))
    return float(np.clip(phi_cmd, -abs(max_bank_rad), abs(max_bank_rad)))
