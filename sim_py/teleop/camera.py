"""Camera control for the teleop viewer.

A world-sized view makes metre-scale motion invisible, so the default is a
follow camera with a small local radius. The world view remains available for
orientation within the map.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

FOLLOW = "follow"
WORLD = "world"


@dataclass
class CameraController:
    """Owns the 3D axes limits and viewing angles."""

    world_bounds: np.ndarray  # (2, 3): [min_xyz, max_xyz]
    radius: float = 12.0
    min_radius: float = 3.0
    max_radius: float = 60.0
    elevation_deg: float = 22.0
    azimuth_deg: float = -60.0
    #: World view looks down on the whole map so terrain does not hide the aircraft.
    world_elevation_deg: float = 34.0
    world_azimuth_deg: float | None = None
    mode: str = FOLLOW
    #: Third-person chase camera: in FOLLOW mode, ``azimuth_deg`` is recomputed
    #: from the vehicle's own heading every frame (when :meth:`apply` is given
    #: one) instead of staying fixed in world space -- the camera stays behind
    #: the vehicle and turns with it, like a game's behind-the-vehicle camera,
    #: rather than watching from a stationary compass bearing. ``apply`` still
    #: falls back to the static ``azimuth_deg`` whenever no heading is passed
    #: (world mode, or a caller -- like a static README-still capture -- that
    #: wants to fix the angle itself), so this is backward compatible.
    chase: bool = True
    #: Degrees added to the vehicle's heading to place the camera. Empirically
    #: calibrated (see sim_py/tests/test_teleop.py's chase-camera tests):
    #: ``azim = heading`` puts the camera in *front* of the vehicle looking
    #: back at its nose, so directly behind -- as if piloting it -- is
    #: ``heading + 180`` exactly.
    chase_azimuth_offset_deg: float = 180.0
    #: Axes3D leaves wide margins by default; this fills the canvas so the
    #: aircraft is large enough to read its attitude. Distinct from
    #: :meth:`zoom`, which changes how much world the view covers.
    canvas_fill: float = 1.45

    def __post_init__(self) -> None:
        self.world_bounds = np.asarray(self.world_bounds, dtype=float).reshape(2, 3)
        self.radius = float(np.clip(self.radius, self.min_radius, self.max_radius))

    def toggle_mode(self) -> str:
        self.mode = WORLD if self.mode == FOLLOW else FOLLOW
        return self.mode

    def zoom(self, steps: int) -> None:
        """Change the follow radius by ``steps`` multiplicative notches."""
        if not steps:
            return
        factor = 1.25 ** int(steps)
        self.radius = float(np.clip(self.radius * factor, self.min_radius, self.max_radius))

    def follow_limits(self, position: np.ndarray) -> np.ndarray:
        """Cube of side ``2 * radius`` centred on the vehicle, clipped to the world.

        The cube keeps an equal aspect on all three axes so attitude is not
        visually sheared, and it slides instead of shrinking near a boundary.
        """
        position = np.asarray(position, dtype=float).reshape(3)
        lower_world, upper_world = self.world_bounds
        lower = position - self.radius
        upper = position + self.radius
        for axis in range(3):
            span = upper[axis] - lower[axis]
            world_span = upper_world[axis] - lower_world[axis]
            if span >= world_span:
                centre = 0.5 * (lower_world[axis] + upper_world[axis])
                lower[axis] = centre - span / 2.0
                upper[axis] = centre + span / 2.0
                continue
            if lower[axis] < lower_world[axis]:
                upper[axis] += lower_world[axis] - lower[axis]
                lower[axis] = lower_world[axis]
            elif upper[axis] > upper_world[axis]:
                lower[axis] -= upper[axis] - upper_world[axis]
                upper[axis] = upper_world[axis]
        return np.vstack([lower, upper])

    def world_limits(self) -> np.ndarray:
        return self.world_bounds.copy()

    def limits_for(self, position: np.ndarray) -> np.ndarray:
        if self.mode == FOLLOW:
            return self.follow_limits(position)
        return self.world_limits()

    def apply(self, ax: Any, position: np.ndarray, yaw: float | None = None) -> None:
        """Position the axes' limits and viewing angle for this frame.

        ``yaw`` (radians) is the vehicle's current heading. When given, FOLLOW
        mode with ``chase`` enabled recomputes ``azimuth_deg`` from it every
        call, so the camera stays behind the vehicle as it turns instead of
        watching from a fixed compass bearing. Omit it (as a static-still
        capture script does after setting ``azimuth_deg`` itself, or as WORLD
        mode always does) to keep the azimuth exactly as last set.
        """
        limits = self.limits_for(position)
        lower, upper = limits
        ax.set_xlim(float(lower[0]), float(upper[0]))
        ax.set_ylim(float(lower[1]), float(upper[1]))
        ax.set_zlim(float(lower[2]), float(upper[2]))
        span = np.maximum(upper - lower, 1e-6)
        ax.set_box_aspect(tuple(span / float(span[0])), zoom=self.canvas_fill)
        if self.mode == FOLLOW and self.chase and yaw is not None:
            self.azimuth_deg = float(np.degrees(yaw)) + self.chase_azimuth_offset_deg
        if self.mode == WORLD:
            elev = self.world_elevation_deg
            azim = self.azimuth_deg if self.world_azimuth_deg is None else self.world_azimuth_deg
        else:
            elev = self.elevation_deg
            azim = self.azimuth_deg
        ax.view_init(elev=elev, azim=azim)

    def describe(self) -> str:
        if self.mode == FOLLOW:
            chase_tag = " chase" if self.chase else ""
            return f"follow r={self.radius:.0f}m{chase_tag}"
        return "world"


__all__ = ["FOLLOW", "WORLD", "CameraController"]
