"""
3D visualization utilities for the standalone simulator.

Uses matplotlib's 3D plotting to show:
- Terrain obstacles
- UAV trajectory
"""

from __future__ import annotations

from typing import Iterable, List, Mapping, Any

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from aerial_kit.sim.terrain import BoxObstacle, CylinderObstacle, HeightFieldTerrain


def _quat_wxyz_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] into a 3x3 rotation matrix."""
    q = np.asarray(quat_wxyz, dtype=float).reshape(4)
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def plot_simulation(
    trajectory: np.ndarray,
    obstacles: Iterable[CylinderObstacle | BoxObstacle],
    space_dim: np.ndarray | None = None,
    terrain_type: str | None = None,
    visual_cfg: Mapping[str, Any] | None = None,
    planned_waypoints: np.ndarray | None = None,
    goal_position: np.ndarray | None = None,
    planner_type: str | None = None,
    attitude_quats: np.ndarray | None = None,
    backend_name: str | None = None,
    show: bool = True,
) -> plt.Figure:
    """
    Plot UAV trajectory and terrain in 3D.

    Args:
        trajectory: array of shape (N, 3) with ENU positions.
        obstacles: iterable of CylinderObstacle or BoxObstacle.
        space_dim: optional [x, y, z] for axis limits.
        show: whether to call plt.show() at the end.
    """
    traj = np.asarray(trajectory, dtype=float)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Use visual config or defaults
    if visual_cfg is None:
        visual_cfg = {}

    # Higher contrast defaults to keep path visible through terrain
    path_linewidth = float(visual_cfg.get("path_linewidth", 2.0))
    path_color = str(visual_cfg.get("path_color", "deepskyblue"))
    marker_size = float(visual_cfg.get("marker_size", 40.0))
    tree_linewidth = float(visual_cfg.get("tree_linewidth", 4.0))
    tree_alpha = float(visual_cfg.get("tree_alpha", 0.9))
    tree_radius_ref = float(visual_cfg.get("tree_radius_ref", 1.0))
    planned_linewidth = float(visual_cfg.get("planned_linewidth", 2.0))
    planned_alpha = float(visual_cfg.get("planned_alpha", 0.8))
    terrain_alpha = float(visual_cfg.get("terrain_alpha", 0.45))
    terrain_cmap = str(visual_cfg.get("terrain_cmap", "terrain"))
    base_span = 100.0
    if space_dim is not None:
        space_dim_arr = np.asarray(space_dim, dtype=float).reshape(3)
        base_span = float(max(space_dim_arr[0], space_dim_arr[1], 1.0))
    auto_quad_len = max(1.2, 0.02 * base_span)
    quad_arm_length = float(visual_cfg.get("quad_arm_length", auto_quad_len))
    attitude_axis_scale = float(visual_cfg.get("attitude_axis_scale", quad_arm_length * 0.6))
    quad_max_frames = int(visual_cfg.get("quad_max_frames", 35))

    # Plot trajectory
    ax.plot(
        traj[:, 0],
        traj[:, 1],
        traj[:, 2],
        color=path_color,
        linewidth=path_linewidth,
        label="UAV path",
        zorder=5,
    )
    ax.scatter(
        traj[0, 0],
        traj[0, 1],
        traj[0, 2],
        color="cyan",
        s=marker_size,
        label="Start",
        zorder=4,
    )
    # Plot goal position (target)
    if goal_position is not None:
        goal_position = np.asarray(goal_position, dtype=float)
        ax.scatter(
            goal_position[0],
            goal_position[1],
            goal_position[2],
            color="magenta",
            s=marker_size,
            label="Goal",
            zorder=4,
        )

    # Plot planned waypoints if provided (for debugging path planning)
    if planned_waypoints is not None and len(planned_waypoints) > 0:
        planned_waypoints = np.asarray(planned_waypoints, dtype=float)
        if len(planned_waypoints) > 2:
            # Draw planned path as orange dashed line
            ax.plot(
                planned_waypoints[:, 0],
                planned_waypoints[:, 1],
                planned_waypoints[:, 2],
                color="orange",
                linestyle="--",
                linewidth=planned_linewidth,
                alpha=planned_alpha,
                label="Planned path",
                zorder=4,
            )
            # Mark waypoints as small orange dots
            ax.scatter(
                planned_waypoints[:, 0],
                planned_waypoints[:, 1],
                planned_waypoints[:, 2],
                color="orange",
                s=20,
                alpha=0.5,
                zorder=4,
            )

    # Optional aircraft pose overlay from quaternion trajectory.
    if attitude_quats is not None:
        is_fixed_wing = str(backend_name).lower() == "fixedwing"
        pose_label = "Fixed-wing pose" if is_fixed_wing else "Multirotor pose"
        body_half_length = quad_arm_length * (0.9 if is_fixed_wing else 0.5)
        wing_half_span = quad_arm_length * (1.3 if is_fixed_wing else 0.5)
        q_traj = np.asarray(attitude_quats, dtype=float)
        if q_traj.ndim == 2 and q_traj.shape[1] == 4:
            n = min(len(traj), len(q_traj))
            if n > 0:
                stride = max(1, n // max(1, quad_max_frames))
                first_label = True
                for i in range(0, n, stride):
                    q = q_traj[i]
                    if not np.all(np.isfinite(q)):
                        continue
                    p = traj[i]
                    R = _quat_wxyz_to_rotmat(q)

                    # Body axes in world frame.
                    ex = R[:, 0]
                    ey = R[:, 1]
                    ez = R[:, 2]

                    # A long fuselage + broad wing for fixed wing, or equal
                    # crossed arms for a multirotor.
                    arm_x0 = p - ex * body_half_length
                    arm_x1 = p + ex * body_half_length
                    arm_y0 = p - ey * wing_half_span
                    arm_y1 = p + ey * wing_half_span

                    ax.plot(
                        [arm_x0[0], arm_x1[0]],
                        [arm_x0[1], arm_x1[1]],
                        [arm_x0[2], arm_x1[2]],
                        color="black",
                        linewidth=2.4,
                        alpha=0.95,
                        label=pose_label if first_label else None,
                        zorder=6,
                    )
                    ax.plot(
                        [arm_y0[0], arm_y1[0]],
                        [arm_y0[1], arm_y1[1]],
                        [arm_y0[2], arm_y1[2]],
                        color="black",
                        linewidth=2.4,
                        alpha=0.95,
                        zorder=6,
                    )

                    # Draw body-frame axes (X red, Y green, Z blue).
                    ax.quiver(
                        p[0], p[1], p[2],
                        ex[0], ex[1], ex[2],
                        length=attitude_axis_scale,
                        color="red",
                        alpha=0.7,
                        linewidth=1.0,
                    )
                    ax.quiver(
                        p[0], p[1], p[2],
                        ey[0], ey[1], ey[2],
                        length=attitude_axis_scale,
                        color="lime",
                        alpha=0.7,
                        linewidth=1.0,
                    )
                    ax.quiver(
                        p[0], p[1], p[2],
                        ez[0], ez[1], ez[2],
                        length=attitude_axis_scale,
                        color="dodgerblue",
                        alpha=0.7,
                        linewidth=1.0,
                    )
                    first_label = False

                # Always highlight the final aircraft pose.
                qf = q_traj[n - 1]
                if np.all(np.isfinite(qf)):
                    pf = traj[n - 1]
                    Rf = _quat_wxyz_to_rotmat(qf)
                    exf = Rf[:, 0]
                    eyf = Rf[:, 1]
                    ezf = Rf[:, 2]

                    ax.plot(
                        [pf[0] - exf[0] * body_half_length * 1.4, pf[0] + exf[0] * body_half_length * 1.4],
                        [pf[1] - exf[1] * body_half_length * 1.4, pf[1] + exf[1] * body_half_length * 1.4],
                        [pf[2] - exf[2] * body_half_length * 1.4, pf[2] + exf[2] * body_half_length * 1.4],
                        color="yellow",
                        linewidth=3.0,
                        alpha=1.0,
                        label="Final fixed wing" if is_fixed_wing else "Final multirotor",
                        zorder=8,
                    )
                    ax.plot(
                        [pf[0] - eyf[0] * wing_half_span * 1.4, pf[0] + eyf[0] * wing_half_span * 1.4],
                        [pf[1] - eyf[1] * wing_half_span * 1.4, pf[1] + eyf[1] * wing_half_span * 1.4],
                        [pf[2] - eyf[2] * wing_half_span * 1.4, pf[2] + eyf[2] * wing_half_span * 1.4],
                        color="yellow",
                        linewidth=3.0,
                        alpha=1.0,
                        zorder=8,
                    )
                    ax.quiver(
                        pf[0], pf[1], pf[2],
                        exf[0], exf[1], exf[2],
                        length=attitude_axis_scale * 1.8,
                        color="red",
                        alpha=1.0,
                        linewidth=2.0,
                    )
                    ax.quiver(
                        pf[0], pf[1], pf[2],
                        eyf[0], eyf[1], eyf[2],
                        length=attitude_axis_scale * 1.8,
                        color="lime",
                        alpha=1.0,
                        linewidth=2.0,
                    )
                    ax.quiver(
                        pf[0], pf[1], pf[2],
                        ezf[0], ezf[1], ezf[2],
                        length=attitude_axis_scale * 1.8,
                        color="dodgerblue",
                        alpha=1.0,
                        linewidth=2.0,
                    )

    # Plot obstacles
    if terrain_type == "forest":
        # Draw trees as vertical green lines (cylinders approximated)
        for o in obstacles:
            c = o.center
            z0 = float(c[2])
            if hasattr(o, "height"):
                h = float(getattr(o, "height"))
            elif hasattr(o, "size"):
                h = float(getattr(o, "size")[2])
            else:
                h = 2.0
            radius = float(getattr(o, "radius", tree_radius_ref))
            radius_scale = max(0.4, min(3.0, radius / max(tree_radius_ref, 1e-6)))
            ax.plot(
                [float(c[0]), float(c[0])],
                [float(c[1]), float(c[1])],
                [z0, z0 + h],
                color="green",
                linewidth=tree_linewidth * radius_scale,
                alpha=tree_alpha,
            )
    elif terrain_type == "mountains":
        terrain = next((o for o in obstacles if isinstance(o, HeightFieldTerrain)), None)
        cylinders = [o for o in obstacles if hasattr(o, "radius")]

        if terrain is not None:
            X, Y = np.meshgrid(terrain.xs, terrain.ys, indexing="ij")
            ax.plot_surface(
                X,
                Y,
                terrain.heights,
                cmap=terrain_cmap,
                linewidth=0.0,
                antialiased=True,
                alpha=terrain_alpha,
                zorder=1,
            )

        if cylinders:
            obs_x = [float(c.center[0]) for c in cylinders]
            obs_y = [float(c.center[1]) for c in cylinders]
            obs_z = [float(c.center[2]) + float(getattr(c, "height", 0.0)) / 2.0 for c in cylinders]
            ax.scatter(obs_x, obs_y, obs_z, color="gray", alpha=0.6, label="Rocks", zorder=2)
    else:
        # Fallback: gray points at obstacle centers
        obs_x: List[float] = []
        obs_y: List[float] = []
        obs_z: List[float] = []
        for o in obstacles:
            c = o.center
            obs_x.append(float(c[0]))
            obs_y.append(float(c[1]))
            z_center = float(c[2])
            if hasattr(o, "height"):
                z_center += float(getattr(o, "height")) / 2.0
            elif hasattr(o, "size"):
                z_center += float(getattr(o, "size")[2]) / 2.0
            obs_z.append(z_center)

        if obs_x:
            ax.scatter(obs_x, obs_y, obs_z, color="gray", alpha=0.6, label="Obstacles")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    title_terrain = (terrain_type or "terrain").capitalize()
    title_planner = (planner_type or "planner").upper()
    title_backend = f" [{str(backend_name).upper()}]" if backend_name else ""
    ax.set_title(f"{title_terrain} path planner with {title_planner}{title_backend}")

    if space_dim is not None:
        ax.set_xlim(0.0, float(space_dim[0]))
        ax.set_ylim(0.0, float(space_dim[1]))
        ax.set_zlim(0.0, float(space_dim[2]))

    ax.legend()
    ax.view_init(elev=30, azim=135)
    ax.grid(True)

    if show:
        plt.tight_layout()
        plt.show()

    return fig


def _index_of_readable_bank(attitude_quats: np.ndarray, *, target_rad: float = 0.52) -> int:
    """Mid-flight frame whose bank is closest to ``target_rad`` (about 30 deg)."""
    from aerial_kit.controllers.fixed_wing import body_axis_pitch_bank

    banks = np.array(
        [abs(body_axis_pitch_bank(q)[1]) for q in np.asarray(attitude_quats, dtype=float)],
        dtype=float,
    )
    n = len(banks)
    if n == 0 or not np.any(np.isfinite(banks)):
        return 0
    lo, hi = int(0.18 * n), max(int(0.82 * n), int(0.18 * n) + 1)
    window = banks[lo:hi]
    return lo + int(np.nanargmin(np.abs(window - float(target_rad))))


def plot_follow_view(
    trajectory: np.ndarray,
    *,
    attitude_quats: np.ndarray | None = None,
    planned_waypoints: np.ndarray | None = None,
    space_dim: np.ndarray | None = None,
    follow_index: int | None = None,
    follow_radius: float = 12.0,
    elevation_deg: float = 30.0,
    azimuth_offset_deg: float = -115.0,
    backend_name: str | None = None,
    planner_type: str | None = None,
    motor_thrust: np.ndarray | None = None,
    show: bool = True,
) -> plt.Figure:
    """Local chase camera of the aircraft, not a whole-map overview.

    Used for README stills where the vehicle's pitch/roll/yaw has to be readable.

    ``elevation_deg`` and ``azimuth_offset_deg`` (the latter measured from the
    aircraft's heading) place the camera. A low elevation sights along the
    flown path and foreshortens a turn into a straight line, so a still meant
    to show the aircraft following a curve wants to look down on it.
    """
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    from .teleop.camera import CameraController
    from .teleop.model import (
        WingGeometry,
        quat_to_euler_rpy,
        quat_to_rotation_matrix,
        transform_points,
    )

    traj = np.asarray(trajectory, dtype=float).reshape(-1, 3)
    if traj.shape[0] < 2:
        raise ValueError("plot_follow_view needs at least two trajectory samples")

    quats = None if attitude_quats is None else np.asarray(attitude_quats, dtype=float)
    if follow_index is None:
        follow_index = _index_of_readable_bank(quats) if quats is not None else (len(traj) * 2) // 3
    follow_index = int(np.clip(follow_index, 0, len(traj) - 1))
    position = traj[follow_index]
    quat = None if quats is None else quats[min(follow_index, len(quats) - 1)]

    if space_dim is None:
        hi = np.max(traj, axis=0) + follow_radius
        bounds = np.vstack([np.zeros(3), np.maximum(hi, position + follow_radius)])
    else:
        bounds = np.vstack([np.zeros(3), np.asarray(space_dim, dtype=float).reshape(3)])

    _, _, yaw = quat_to_euler_rpy(quat)
    camera = CameraController(
        world_bounds=bounds,
        radius=float(follow_radius),
        min_radius=4.0,
        max_radius=80.0,
        elevation_deg=float(elevation_deg),
        azimuth_deg=float(np.degrees(yaw) + float(azimuth_offset_deg)),
        canvas_fill=1.45,
    )

    background = "#10131a"
    fig = plt.figure(figsize=(11.0, 7.5), facecolor=background)
    ax = fig.add_subplot(111, projection="3d", facecolor=background)
    ax.tick_params(colors="#8b93a7", labelsize=7)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((0.06, 0.07, 0.10, 1.0))
        axis._axinfo["grid"]["color"] = "#2b3242"
        axis._axinfo["grid"]["linewidth"] = 0.5
    ax.grid(True)

    ax.plot(
        traj[:, 0], traj[:, 1], traj[:, 2],
        color="#2f8fa8", linewidth=2.2, alpha=0.95, zorder=6,
    )

    geometry = WingGeometry()
    parts = geometry.segments_body()
    colors, widths = geometry.segment_styles(motor_thrust=motor_thrust)
    rotation = quat_to_rotation_matrix(quat)
    world_parts = [transform_points(part, rotation, position) for part in parts]
    ax.add_collection3d(Line3DCollection(world_parts, colors=colors, linewidths=widths, zorder=12))

    camera.apply(ax, position)
    fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.04)

    roll, pitch, yaw = quat_to_euler_rpy(quat)
    bank = roll
    try:
        from aerial_kit.controllers.fixed_wing import body_axis_pitch_bank

        pitch, bank = body_axis_pitch_bank(quat)
    except Exception:
        pass
    title = "twin-wing" if str(backend_name or "").lower() == "fixedwing" else (backend_name or "aircraft")
    planner = f" | {planner_type}" if planner_type else ""
    ax.text2D(
        0.02,
        0.96,
        (
            f"{title}{planner}  follow r={follow_radius:.0f}m\n"
            f"bank {np.degrees(bank):6.1f}  pitch {np.degrees(pitch):6.1f}  yaw {np.degrees(yaw):6.1f} deg"
        ),
        transform=ax.transAxes,
        fontsize=9.5,
        family="monospace",
        color="#e6e9f0",
        va="top",
        bbox={"facecolor": "#171b25", "alpha": 0.9, "edgecolor": "#2b3242", "pad": 4.0},
    )

    if show:
        plt.show()
    return fig
