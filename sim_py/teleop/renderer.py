"""Matplotlib renderer for the teleop viewer.

All moving geometry lives in a handful of ``Line3DCollection`` artists whose
segments are recomputed from the vehicle's attitude quaternion each frame. That
keeps the artist count low enough to hold a real-time frame rate in a 3D axes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from aerial_kit.types import SimState

from .camera import CameraController
from .commands import TeleopCommand, describe_axes
from .input_state import CONTROL_HELP_BAR
from .model import (
    BODY_COLOR,
    NOSE_COLOR,
    PROP_COLOR,
    QuadGeometry,
    quat_to_euler_rpy,
    quat_to_rotation_matrix,
    thrust_to_color,
    transform_points,
)
from .world import TeleopWorld

BACKGROUND = "#10131a"
PANEL = "#171b25"
TEXT_COLOR = "#e6e9f0"
GRID_COLOR = "#2b3242"
COLLISION_COLOR = "#ff2d55"
TRAIL_COLOR = "#2f8fa8"
VELOCITY_COLOR = "#ffd60a"
PLANNED_COLOR = "#ff9f43"
GOAL_COLOR = "#ff6b9d"


@dataclass
class HudInfo:
    """Everything the HUD prints that is not part of :class:`SimState`."""

    frames: int = 0
    frame_rate: float = 0.0
    real_time_factor: float = 0.0
    sim_steps: int = 0
    backend_name: str = ""
    paused: bool = False
    focused: bool = True
    colliding: bool = False
    camera: str = ""
    neutralized: bool = False
    motor_thrust: np.ndarray | None = None


class QuadArtist:
    """The quadrotor itself: frame, arms, motors, propellers and nose."""

    def __init__(self, ax: Any, geometry: QuadGeometry, *, show_body_axes: bool = True) -> None:
        self.geometry = geometry
        self._show_body_axes = show_body_axes

        motor_colors = geometry.motor_colors()
        parts: list[np.ndarray] = []
        colors: list[str] = []
        widths: list[float] = []
        motor_segments: list[list[int]] = [[] for _ in range(4)]

        def _add(segment: np.ndarray, color: str, width: float, *, motor: int | None = None) -> None:
            if motor is not None:
                motor_segments[motor].append(len(parts))
            parts.append(segment)
            colors.append(color)
            widths.append(width)

        for i, (arm, color) in enumerate(zip(geometry.arm_segments_body(), motor_colors)):
            _add(arm, color, 3.0)
        for segment in geometry.body_segments_body():
            _add(segment, BODY_COLOR, 1.6)
        for i, housing in enumerate(geometry.motor_housing_segments_body()):
            _add(housing, motor_colors[i], 7.0)
        for i, disc in enumerate(geometry.propeller_polylines_body()):
            _add(disc, PROP_COLOR, 1.4, motor=i)
        for i, blade in enumerate(geometry.propeller_blade_segments_body()):
            _add(blade, PROP_COLOR, 2.2, motor=i // 2)
        for spike in geometry.nose_segments_body():
            _add(spike, NOSE_COLOR, 2.6)

        self._base_colors = colors
        self._thrust_by_motor: tuple[tuple[int, ...], ...] = tuple(tuple(idx) for idx in motor_segments)
        # Tests and HUD colouring still look up propeller discs via this alias.
        self._prop_by_motor = self._thrust_by_motor
        # One flat point array keeps the per-frame transform to a single matmul.
        self._split_indices = np.cumsum([len(part) for part in parts])[:-1]
        self._body_points = np.vstack(parts)
        #: Most recent world-frame polylines. ``Line3DCollection`` keeps its 3D
        #: segments private and only exposes the projected 2D ones after a
        #: draw, so the renderer keeps its own copy for inspection and tests.
        self.world_segments: list[np.ndarray] = list(parts)
        self.segment_count = len(parts)
        self.display_colors = list(colors)

        self.frame = Line3DCollection(parts, colors=colors, linewidths=widths, zorder=12)
        ax.add_collection3d(self.frame)

        axis_parts = geometry.body_axis_segments_body()
        self._axis_split = np.cumsum([len(part) for part in axis_parts])[:-1]
        self._axis_points = np.vstack(axis_parts)
        self.body_axes = Line3DCollection(
            axis_parts,
            colors=["#ff6b6b", "#5ef38c", "#7cc4ff"],
            linewidths=1.2,
            alpha=0.85 if show_body_axes else 0.0,
            zorder=11,
        )
        self.body_axes.set_visible(show_body_axes)
        ax.add_collection3d(self.body_axes)

    def motor_positions_world(
        self, position: np.ndarray, attitude_quat: np.ndarray | None
    ) -> np.ndarray:
        """``(4, 3)`` world-frame motor hubs for the given pose."""
        return transform_points(
            self.geometry.motor_positions_body(), quat_to_rotation_matrix(attitude_quat), position
        )

    def update(
        self,
        position: np.ndarray,
        attitude_quat: np.ndarray | None,
        *,
        colliding: bool,
        motor_thrust: np.ndarray | None = None,
    ) -> None:
        rotation = quat_to_rotation_matrix(attitude_quat)
        world_points = transform_points(self._body_points, rotation, position)
        self.world_segments = np.split(world_points, self._split_indices)
        self.frame.set_segments(self.world_segments)
        if colliding:
            self.display_colors = [COLLISION_COLOR] * len(self._base_colors)
            self.frame.set_color(self.display_colors)
        else:
            colors = list(self._base_colors)
            if motor_thrust is not None:
                thrusts = np.asarray(motor_thrust, dtype=float).reshape(-1)
                for motor_i, indices in enumerate(self._thrust_by_motor):
                    frac = float(thrusts[motor_i]) if motor_i < thrusts.size else 0.5
                    tint = thrust_to_color(frac)
                    for index in indices:
                        colors[index] = tint
            self.frame.set_color(colors)
            self.display_colors = colors
        if self._show_body_axes:
            axis_points = transform_points(self._axis_points, rotation, position)
            self.body_axes.set_segments(np.split(axis_points, self._axis_split))


class TeleopRenderer:
    """Owns the figure, the static scene, the quad artist and the HUD."""

    def __init__(
        self,
        *,
        world: TeleopWorld,
        camera: CameraController,
        geometry: QuadGeometry,
        backend_name: str,
        visual_cfg: dict[str, Any] | None = None,
    ) -> None:
        import matplotlib.pyplot as plt

        visual_cfg = dict(visual_cfg or {})
        self.world = world
        self.camera = camera
        self.geometry = geometry
        self.backend_name = backend_name

        self.fig = plt.figure(figsize=(11.0, 7.5), facecolor=BACKGROUND)
        self.fig.canvas.manager.set_window_title("aerial-kit quadrotor teleop")
        self.ax = self.fig.add_subplot(111, projection="3d", facecolor=BACKGROUND)
        self._style_axes()
        self._draw_static_scene(visual_cfg)

        start = np.asarray(world.start_position, dtype=float)
        # The trail is deliberately thinner and dimmer than the airframe; at the
        # plotting widths used for static path plots it visually swamps the
        # vehicle in a close follow view.
        self.trail, = self.ax.plot(
            [start[0]], [start[1]], [start[2]],
            color=str(visual_cfg.get("teleop_trail_color", TRAIL_COLOR)),
            linewidth=float(visual_cfg.get("teleop_trail_linewidth", 1.8)),
            alpha=float(visual_cfg.get("teleop_trail_alpha", 0.85)),
            zorder=8,
        )
        self.velocity_arrow, = self.ax.plot(
            [start[0], start[0]], [start[1], start[1]], [start[2], start[2]],
            color=VELOCITY_COLOR,
            linewidth=2.0,
            alpha=0.9,
            zorder=10,
        )
        self.ground_marker, = self.ax.plot(
            [start[0]], [start[1]], [0.0],
            marker="x", color="#6b7488", markersize=6, linestyle="none", zorder=7,
        )
        self.quad = QuadArtist(
            self.ax,
            geometry,
            show_body_axes=bool(visual_cfg.get("teleop_show_body_axes", True)),
        )

        # Overlay text is rasterized on every frame, so the HUD is kept compact
        # and the help is one toggleable line -- together they were costing more
        # draw time than all of the 3D geometry combined.
        self.hud = self.fig.text(
            0.010, 0.988, "",
            fontsize=9.5, family="monospace", color=TEXT_COLOR,
            va="top", ha="left",
            bbox={"facecolor": PANEL, "alpha": 0.9, "edgecolor": GRID_COLOR, "pad": 4.0},
            zorder=20,
        )
        self.help_text = self.fig.text(
            0.5, 0.012, CONTROL_HELP_BAR,
            fontsize=8.5, family="monospace", color="#8f99ad",
            va="bottom", ha="center", zorder=20,
        )
        # Leave room below the axes for the axis labels the canvas fill pushes
        # outward, and for the help bar underneath them.
        self.fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.10)

    def _style_axes(self) -> None:
        from matplotlib.ticker import MaxNLocator

        ax = self.ax
        # No axis labels: the canvas fill pushes the 3D box past the axes bbox,
        # where labels collide with the help bar. Tick numbers stay for scale
        # and the HUD reports position in metres.
        ax.tick_params(colors="#8b93a7", labelsize=7)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_pane_color((0.06, 0.07, 0.10, 1.0))
            axis._axinfo["grid"]["color"] = GRID_COLOR
            axis._axinfo["grid"]["linewidth"] = 0.5
            # Every tick label is re-rasterized each frame; four per axis is
            # enough to read position and keeps the frame rate up.
            axis.set_major_locator(MaxNLocator(4))
        ax.grid(True)

    def _draw_static_scene(self, visual_cfg: dict[str, Any]) -> None:
        """Draw ground grid, terrain and obstacles once.

        Static geometry is still re-projected on every draw, so the mesh is
        deliberately coarse to protect the frame rate.
        """
        ax = self.ax
        world = self.world
        span_x = float(world.space_dim[0])
        span_y = float(world.space_dim[1])

        spacing = float(visual_cfg.get("teleop_grid_spacing_m", 10.0))
        grid: list[np.ndarray] = []
        for x in np.arange(0.0, span_x + spacing * 0.5, spacing):
            grid.append(np.array([[x, 0.0, 0.0], [x, span_y, 0.0]], dtype=float))
        for y in np.arange(0.0, span_y + spacing * 0.5, spacing):
            grid.append(np.array([[0.0, y, 0.0], [span_x, y, 0.0]], dtype=float))
        if grid:
            ax.add_collection3d(
                Line3DCollection(grid, colors=GRID_COLOR, linewidths=0.7, alpha=0.7, zorder=1)
            )

        if world.terrain is not None and hasattr(world.terrain, "heights"):
            terrain = world.terrain
            stride_x = max(1, terrain.xs.size // 24)
            stride_y = max(1, terrain.ys.size // 24)
            xs = terrain.xs[::stride_x]
            ys = terrain.ys[::stride_y]
            heights = terrain.heights[::stride_x, ::stride_y]
            mesh_x, mesh_y = np.meshgrid(xs, ys, indexing="ij")
            ax.plot_surface(
                mesh_x, mesh_y, heights,
                cmap=str(visual_cfg.get("terrain_cmap", "terrain")),
                linewidth=0.0,
                antialiased=False,
                alpha=float(visual_cfg.get("terrain_alpha", 0.55)),
                zorder=2,
            )

        trunks: list[np.ndarray] = []
        rocks: list[np.ndarray] = []
        for obstacle in world.obstacles:
            if hasattr(obstacle, "heights"):
                continue
            center = np.asarray(obstacle.center, dtype=float)
            if hasattr(obstacle, "radius"):
                height = float(getattr(obstacle, "height", 2.0))
                trunks.append(
                    np.array(
                        [[center[0], center[1], center[2]], [center[0], center[1], center[2] + height]],
                        dtype=float,
                    )
                )
            elif hasattr(obstacle, "size"):
                rocks.append(center)
        if trunks:
            ax.add_collection3d(
                Line3DCollection(
                    trunks,
                    colors="#3f8f52",
                    linewidths=float(visual_cfg.get("tree_linewidth", 3.5)),
                    alpha=float(visual_cfg.get("tree_alpha", 0.9)),
                    zorder=3,
                )
            )
        if rocks:
            rock_array = np.vstack(rocks)
            ax.scatter(
                rock_array[:, 0], rock_array[:, 1], rock_array[:, 2],
                color="#7d8492", s=18, alpha=0.8, zorder=3,
            )

        planned = world.planned_waypoints
        show_plan = bool(visual_cfg.get("teleop_show_plan", False))
        if show_plan and planned is not None and len(planned) >= 2:
            # One line, no waypoint crumbs — those read as extra vehicles.
            lifted = np.asarray(planned, dtype=float).copy()
            lifted[:, 2] += 0.6
            ax.plot(
                lifted[:, 0], lifted[:, 1], lifted[:, 2],
                color=PLANNED_COLOR,
                linestyle="-",
                linewidth=float(visual_cfg.get("planned_linewidth", 1.8)),
                alpha=0.85,
                zorder=6,
            )

    def update(
        self,
        state: SimState,
        command: TeleopCommand,
        trail: np.ndarray,
        info: HudInfo,
    ) -> None:
        position = np.asarray(state.position, dtype=float).reshape(3)
        velocity = np.asarray(state.velocity, dtype=float).reshape(3)

        self.quad.update(
            position,
            state.attitude_quat,
            colliding=info.colliding,
            motor_thrust=info.motor_thrust,
        )
        if trail.shape[0] >= 2:
            self.trail.set_data_3d(trail[:, 0], trail[:, 1], trail[:, 2])

        # Scale the velocity arrow to the view so it stays readable at any zoom.
        arrow_scale = max(0.8, self.camera.radius / 12.0)
        tip = position + velocity * 0.18 * arrow_scale
        self.velocity_arrow.set_data_3d(
            [position[0], tip[0]], [position[1], tip[1]], [position[2], tip[2]]
        )
        ground_z = self.world.ground_height(position[0], position[1])
        self.ground_marker.set_data_3d([position[0]], [position[1]], [ground_z])

        self.camera.apply(self.ax, position)
        self.hud.set_text(self._hud_text(state, command, info))

    def _hud_text(self, state: SimState, command: TeleopCommand, info: HudInfo) -> str:
        position = np.asarray(state.position, dtype=float).reshape(3)
        velocity = np.asarray(state.velocity, dtype=float).reshape(3)
        roll, pitch, yaw = quat_to_euler_rpy(state.attitude_quat)
        speed = float(np.linalg.norm(velocity))

        if not info.focused:
            status = "NO FOCUS - click the window"
        elif info.paused:
            status = "PAUSED"
        else:
            status = "RUNNING"
        if info.neutralized:
            status += " | NEUTRALIZED"

        lines = [
            f"t {state.t:8.2f}s  frame {info.frames:>6d}  step {info.sim_steps:>7d}",
            f"{info.frame_rate:5.1f} fps   real-time x{info.real_time_factor:4.2f}",
            f"pos {position[0]:7.1f} {position[1]:7.1f} {position[2]:6.1f} m",
            f"vel {velocity[0]:7.2f} {velocity[1]:7.2f} {velocity[2]:6.2f}  |v| {speed:5.2f}",
            f"rpy {np.degrees(roll):7.1f} {np.degrees(pitch):7.1f} {np.degrees(yaw):6.1f} deg",
            f"cmd {describe_axes(command.axes)} yr{command.yaw_rate_cmd:+5.2f}",
            _thrust_hud_line(info.motor_thrust),
            f"{info.backend_name or self.backend_name}"
            f"{(' | ' + self.world.planner_type) if self.world.planner_type else ''}"
            f" | {info.camera or self.camera.describe()}"
            f" | collision {'YES' if info.colliding else 'no'}",
            status,
        ]
        return "\n".join(lines)

    def set_help_visible(self, visible: bool) -> None:
        self.help_text.set_visible(bool(visible))


def _thrust_hud_line(motor_thrust: np.ndarray | None) -> str:
    if motor_thrust is None:
        return "thr  --  --  --  --   FL FR RR RL"
    frac = np.asarray(motor_thrust, dtype=float).reshape(-1)
    pct = [int(round(100.0 * float(frac[i]))) if i < frac.size else 0 for i in range(4)]
    return f"thr FL{pct[0]:3d} FR{pct[1]:3d} RR{pct[2]:3d} RL{pct[3]:3d} %"


__all__ = ["HudInfo", "QuadArtist", "TeleopRenderer"]
