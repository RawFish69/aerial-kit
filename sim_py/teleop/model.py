"""Code-native X-configuration quadrotor visual model.

Geometry is defined once in the body frame and transformed by the simulated
attitude quaternion, so roll, pitch and yaw shown on screen are always the
vehicle's real orientation rather than a display-only heading variable.

Proportions and the front-red / rear-blue convention mirror the Three.js quad
in ``tools/drone_visualizer.py``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Body-frame axes are ENU-like: +X forward, +Y left, +Z up.
MOTOR_LAYOUT: tuple[tuple[float, float], ...] = (
    (1.0, 1.0),  # front-left
    (1.0, -1.0),  # front-right
    (-1.0, -1.0),  # rear-right
    (-1.0, 1.0),  # rear-left
)
MOTOR_LABELS: tuple[str, ...] = ("front-left", "front-right", "rear-right", "rear-left")
MOTOR_IS_FRONT: tuple[bool, ...] = tuple(layout[0] > 0 for layout in MOTOR_LAYOUT)

FRONT_COLOR = "#ff3b30"
REAR_COLOR = "#2f7bff"
FRAME_COLOR = "#3a3a44"
BODY_COLOR = "#d8d8e0"
PROP_COLOR = "#9fb4c7"
NOSE_COLOR = "#ffd60a"

#: Colour stops for each propeller: 0% throttle is grey, then yellow → orange → red at 100%.
THRUST_COLOR_STOPS: tuple[tuple[float, tuple[int, int, int]], ...] = (
    (0.00, (86, 88, 96)),
    (0.10, (255, 228, 72)),
    (0.38, (255, 196, 48)),
    (0.58, (255, 128, 28)),
    (0.78, (255, 56, 22)),
    (1.00, (216, 12, 24)),
)


def quat_to_rotation_matrix(quat: np.ndarray | None) -> np.ndarray:
    """Body-to-world rotation for a ``[w, x, y, z]`` quaternion."""
    if quat is None:
        return np.eye(3, dtype=float)
    q = np.asarray(quat, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        return np.eye(3, dtype=float)
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_to_euler_rpy(quat: np.ndarray | None) -> tuple[float, float, float]:
    """Roll, pitch and yaw in radians for a ``[w, x, y, z]`` quaternion."""
    if quat is None:
        return 0.0, 0.0, 0.0
    q = np.asarray(quat, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        return 0.0, 0.0, 0.0
    w, x, y, z = q / norm
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)


def yaw_to_quat(yaw: float) -> np.ndarray:
    """Level attitude quaternion at heading ``yaw``."""
    half = 0.5 * float(yaw)
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=float)


def transform_points(points: np.ndarray, rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    """Rotate body-frame points into the world and translate to ``position``."""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    return pts @ np.asarray(rotation, dtype=float).T + np.asarray(position, dtype=float).reshape(3)


@dataclass(frozen=True)
class QuadGeometry:
    """Body-frame geometry of an X-configuration quadrotor.

    ``scale`` is a pure display multiplier. A ~1 m airframe is only a few
    pixels wide in a 25 m field of view, so the drawn model is exaggerated to
    keep attitude readable; physics is unaffected.
    """

    arm_length: float = 0.9
    scale: float = 3.0
    body_length_ratio: float = 0.55
    body_width_ratio: float = 0.5
    body_height_ratio: float = 0.18
    motor_height_ratio: float = 0.16
    prop_radius_ratio: float = 0.42
    nose_length_ratio: float = 0.55
    prop_points: int = 17

    @property
    def span(self) -> float:
        """Motor-to-motor distance across the diagonal, in drawn metres."""
        return 2.0 * self.arm_length * self.scale

    def _arm_offset(self) -> float:
        return self.arm_length * self.scale / np.sqrt(2.0)

    def motor_positions_body(self) -> np.ndarray:
        """``(4, 3)`` motor hub locations, ordered as :data:`MOTOR_LABELS`."""
        offset = self._arm_offset()
        return np.array(
            [[sign_x * offset, sign_y * offset, 0.0] for sign_x, sign_y in MOTOR_LAYOUT],
            dtype=float,
        )

    def arm_segments_body(self) -> list[np.ndarray]:
        """One line per arm, from the frame centre out to each motor."""
        return [
            np.array([[0.0, 0.0, 0.0], motor], dtype=float)
            for motor in self.motor_positions_body()
        ]

    def body_segments_body(self) -> list[np.ndarray]:
        """Closed top and bottom outlines of the central fuselage box."""
        half_x = self.arm_length * self.scale * self.body_length_ratio / 2.0
        half_y = self.arm_length * self.scale * self.body_width_ratio / 2.0
        half_z = self.arm_length * self.scale * self.body_height_ratio / 2.0
        corners = np.array(
            [
                [half_x, half_y],
                [half_x, -half_y],
                [-half_x, -half_y],
                [-half_x, half_y],
                [half_x, half_y],
            ],
            dtype=float,
        )
        segments: list[np.ndarray] = []
        for z in (half_z, -half_z):
            segments.append(np.column_stack([corners, np.full(len(corners), z)]))
        for corner in corners[:-1]:
            segments.append(
                np.array([[corner[0], corner[1], -half_z], [corner[0], corner[1], half_z]], dtype=float)
            )
        return segments

    def motor_housing_segments_body(self) -> list[np.ndarray]:
        """Vertical stub at each motor, drawn thick to read as a housing."""
        height = self.arm_length * self.scale * self.motor_height_ratio
        return [
            np.array([[m[0], m[1], m[2]], [m[0], m[1], m[2] + height]], dtype=float)
            for m in self.motor_positions_body()
        ]

    def propeller_polylines_body(self) -> list[np.ndarray]:
        """A disc outline above each motor."""
        radius = self.arm_length * self.scale * self.prop_radius_ratio
        height = self.arm_length * self.scale * self.motor_height_ratio
        angles = np.linspace(0.0, 2.0 * np.pi, self.prop_points)
        circle = np.column_stack([radius * np.cos(angles), radius * np.sin(angles), np.zeros_like(angles)])
        return [circle + np.array([m[0], m[1], m[2] + height]) for m in self.motor_positions_body()]

    def propeller_blade_segments_body(self) -> list[np.ndarray]:
        """Two crossed blades inside each propeller disc."""
        radius = self.arm_length * self.scale * self.prop_radius_ratio
        height = self.arm_length * self.scale * self.motor_height_ratio
        blades: list[np.ndarray] = []
        for motor in self.motor_positions_body():
            hub = np.array([motor[0], motor[1], motor[2] + height], dtype=float)
            for angle in (0.0, np.pi / 2.0):
                tip = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0], dtype=float)
                blades.append(np.array([hub - tip, hub + tip], dtype=float))
        return blades

    def nose_segments_body(self) -> list[np.ndarray]:
        """Forward-direction indicator: a spike with a small arrowhead."""
        length = self.arm_length * self.scale * (1.0 + self.nose_length_ratio)
        tip = np.array([length, 0.0, 0.0], dtype=float)
        base = np.array([self.arm_length * self.scale * self.body_length_ratio / 2.0, 0.0, 0.0])
        barb = self.arm_length * self.scale * 0.22
        return [
            np.array([base, tip], dtype=float),
            np.array([tip, tip + np.array([-barb, barb * 0.6, 0.0])], dtype=float),
            np.array([tip, tip + np.array([-barb, -barb * 0.6, 0.0])], dtype=float),
        ]

    def body_axis_segments_body(self) -> list[np.ndarray]:
        """Body X/Y/Z reference axes, each one span long."""
        length = self.arm_length * self.scale * 1.1
        return [
            np.array([[0.0, 0.0, 0.0], [length, 0.0, 0.0]], dtype=float),
            np.array([[0.0, 0.0, 0.0], [0.0, length, 0.0]], dtype=float),
            np.array([[0.0, 0.0, 0.0], [0.0, 0.0, length]], dtype=float),
        ]

    def motor_colors(self) -> list[str]:
        return [FRONT_COLOR if is_front else REAR_COLOR for is_front in MOTOR_IS_FRONT]


@dataclass(frozen=True)
class WingGeometry:
    """Body-frame twin-wing drawing, exaggerated so attitude reads in a local view.

    +X is nose-forward, +Y left, +Z up. Physics is unaffected.
    """

    fuselage_length: float = 6.8
    wingspan: float = 10.6
    chord: float = 1.45
    motor_span_ratio: float = 0.42

    def segments_body(self) -> list[np.ndarray]:
        """Fuselage, main wing and twin tractor props — flying-wing layout, no tail hook."""
        half = 0.5 * self.fuselage_length
        wing_x = 0.08 * half
        wing_y = 0.5 * self.wingspan
        motor_y = wing_y * self.motor_span_ratio
        segs: list[np.ndarray] = [
            np.array([[-half, 0.0, 0.0], [half, 0.0, 0.0]], dtype=float),
            np.array([[wing_x, -wing_y, 0.0], [wing_x, wing_y, 0.0]], dtype=float),
        ]
        radius = 0.42 * self.chord
        angles = np.linspace(0.0, 2.0 * np.pi, 17)
        for sign in (-1.0, 1.0):
            hub = np.array([wing_x, sign * motor_y, 0.0], dtype=float)
            # Disc sits on the wing (XZ plane) so a chase view reads it as a circle.
            disc = np.column_stack(
                [radius * np.cos(angles), np.zeros_like(angles), radius * np.sin(angles)]
            )
            segs.append(disc + hub)
            for angle in (0.0, np.pi / 2.0):
                tip = np.array([radius * np.cos(angle), 0.0, radius * np.sin(angle)], dtype=float)
                segs.append(np.array([hub - tip, hub + tip], dtype=float))
        return segs

    def segment_styles(self, motor_thrust: np.ndarray | None = None) -> tuple[list[str], list[float]]:
        """Colours and linewidths matching :meth:`segments_body`."""
        fuse, wing = "#d8d8e0", "#c5c9d4"
        colors = [fuse, wing]
        widths = [3.6, 5.6]
        thrusts = np.array([0.5, 0.5], dtype=float)
        if motor_thrust is not None:
            thrusts = np.clip(np.asarray(motor_thrust, dtype=float).reshape(-1)[:2], 0.0, 1.0)
            if thrusts.size < 2:
                thrusts = np.array([0.5, 0.5], dtype=float)
        for frac in thrusts.tolist():
            tint = thrust_to_color(float(frac))
            colors.extend([tint, tint, tint])
            widths.extend([1.4, 2.2, 2.2])
        return colors, widths


def rotor_thrust_fractions(
    *,
    collective: float,
    body_rate_cmd: np.ndarray | None = None,
    max_body_rate: float = 4.0,
    mix: float = 0.58,
) -> np.ndarray:
    """Per-rotor thrust in ``[0, 1]``, visual motor order FL / FR / RR / RL.

    The native multirotor backend integrates a single collective thrust plus
    body-rate commands, so this reconstructs the X-mixer a real quad would run
    to produce that wrench. ``collective`` is 1 at hover. Body rates are FRD:
    +p right-wing-down, +q nose-down, +r yaw-right.
    """
    hover = 0.5 * float(np.clip(collective, 0.0, 2.0))
    rates = np.zeros(3, dtype=float) if body_rate_cmd is None else np.asarray(body_rate_cmd, dtype=float).reshape(3)
    scale = max(float(max_body_rate), 1e-6)
    roll, pitch, yaw = (np.clip(rates / scale, -1.0, 1.0) * float(mix)).tolist()
    return np.clip(
        np.array(
            [
                hover + roll + pitch - yaw,  # front-left
                hover - roll + pitch + yaw,  # front-right
                hover - roll - pitch - yaw,  # rear-right
                hover + roll - pitch + yaw,  # rear-left
            ],
            dtype=float,
        ),
        0.0,
        1.0,
    )


def thrust_to_color(frac: float) -> str:
    """Map a 0–1 thrust fraction onto :data:`THRUST_COLOR_STOPS`."""
    t = float(np.clip(frac, 0.0, 1.0))
    for (t0, c0), (t1, c1) in zip(THRUST_COLOR_STOPS, THRUST_COLOR_STOPS[1:]):
        if t <= t1:
            span = t1 - t0
            u = 0.0 if span < 1e-9 else (t - t0) / span
            rgb = tuple(int(round(a + u * (b - a))) for a, b in zip(c0, c1))
            return f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"
    r, g, b = THRUST_COLOR_STOPS[-1][1]
    return f"#{r:02x}{g:02x}{b:02x}"


__all__ = [
    "BODY_COLOR",
    "FRAME_COLOR",
    "FRONT_COLOR",
    "MOTOR_IS_FRONT",
    "MOTOR_LABELS",
    "MOTOR_LAYOUT",
    "NOSE_COLOR",
    "PROP_COLOR",
    "QuadGeometry",
    "REAR_COLOR",
    "WingGeometry",
    "THRUST_COLOR_STOPS",
    "quat_to_euler_rpy",
    "quat_to_rotation_matrix",
    "rotor_thrust_fractions",
    "thrust_to_color",
    "transform_points",
    "yaw_to_quat",
]
