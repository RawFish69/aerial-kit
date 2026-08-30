"""Simulation runner orchestration for planners/controllers/backends."""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from ..core.config import NormalizedSimConfig
from ..core.registry import (
    create_airframe,
    create_backend,
    create_controller,
    create_planner,
    register_builtin_components,
)
from ..core.types import CommandKind, ControlTarget, SimState, TrajectoryLog, Waypoint, Wrench
from ..terrain_wrapper import (
    TerrainConfig,
    generate_terrain,
    is_point_in_collision,
    load_terrain_config,
)

logger = logging.getLogger(__name__)


def run_simulation(cfg_norm: NormalizedSimConfig) -> TrajectoryLog:
    """Run one simulation and return collected outputs/diagnostics."""
    if cfg_norm.seed is not None:
        np.random.seed(cfg_norm.seed)
        logger.info(f"Random seed set to {cfg_norm.seed}")

    path_cfg = dict(cfg_norm.path_cfg)
    ctrl_cfg = dict(cfg_norm.controller_cfg)
    vis_cfg = dict(cfg_norm.visual_cfg)

    # Terrain configuration and generation
    cfg: TerrainConfig
    if cfg_norm.terrain_config_path is not None:
        cfg = load_terrain_config(Path(cfg_norm.terrain_config_path))
    else:
        cfg = load_terrain_config()

    if cfg_norm.terrain_override is not None:
        cfg.terrain_type = cfg_norm.terrain_override

    logger.info(f"Terrain type: {cfg.terrain_type}")
    logger.info(f"Space dimensions: {cfg.space_dim}")

    obstacles = generate_terrain(
        cfg,
        forest_density_scale=float(vis_cfg.get("forest_density_scale", 1.0)),
        tree_height_scale=float(vis_cfg.get("tree_height_scale", 1.0)),
    )
    logger.info(f"Generated {len(obstacles)} obstacles")
    terrain = next((o for o in obstacles if hasattr(o, "height_at")), None)

    # Compute max allowed flight altitude: cannot fly above tallest obstacle/terrain
    max_tree_top = 0.0
    for o in obstacles:
        if hasattr(o, "heights"):  # HeightFieldTerrain
            top = float(np.max(getattr(o, "heights")))
        else:
            c = o.center
            top = float(c[2])
            if hasattr(o, "height"):
                top += float(getattr(o, "height"))
            elif hasattr(o, "size"):
                top += float(getattr(o, "size")[2]) / 2.0
        if top > max_tree_top:
            max_tree_top = top

    # Give plot headroom above tallest obstacle/terrain.
    if cfg.terrain_type in {"forest", "mountains"} and max_tree_top > 0.0:
        height_ratio = float(vis_cfg.get("height_ratio", 1.2))
        cfg.space_dim = cfg.space_dim.copy()
        cfg.space_dim[2] = max_tree_top * height_ratio
        label = "tallest tree" if cfg.terrain_type == "forest" else "tallest peak"
        logger.info(
            f"Adjusted space Z to {cfg.space_dim[2]:.2f} m "
            f"(ratio {height_ratio:.2f} * {label})"
        )

    max_z_allowed = min(max_tree_top, float(cfg.space_dim[2]))
    logger.info(f"Max allowed altitude: {max_z_allowed:.2f} m (tallest obstacle: {max_tree_top:.2f} m)")

    # Compute nominal start/goal positions from config
    sx_rel = float(path_cfg.get("start_relative_x", 0.02))
    sy_rel = float(path_cfg.get("start_relative_y", 0.02))
    sz_rel = float(path_cfg.get("start_relative_z", 0.05))
    gx_rel = float(path_cfg.get("end_relative_x", 0.98))
    gy_rel = float(path_cfg.get("end_relative_y", 0.98))
    gz_raw = path_cfg.get("end_relative_z", 0.9)
    auto_goal_z = isinstance(gz_raw, str) and gz_raw.lower() in {"auto", "auto_tree", "tree"}

    sx = sx_rel * float(cfg.space_dim[0])
    sy = sy_rel * float(cfg.space_dim[1])
    sz = sz_rel * float(cfg.space_dim[2])
    gx = gx_rel * float(cfg.space_dim[0])
    gy = gy_rel * float(cfg.space_dim[1])

    if auto_goal_z:
        gz = float(np.random.uniform(0.0, max_z_allowed))
        gz_rel = gz / float(cfg.space_dim[2]) if float(cfg.space_dim[2]) > 0 else 0.0
        logger.info(f"Goal Z set randomly in [0, {max_z_allowed:.2f}] m")
    else:
        gz_rel = float(gz_raw)
        gz = gz_rel * float(cfg.space_dim[2])

    # Optional random XY offsets (meters).
    start_xy_off = float(path_cfg.get("start_xy_offset_range", 0.0))
    end_xy_off = float(path_cfg.get("end_xy_offset_range", 0.0))
    if start_xy_off > 0.0:
        dx_s = float(np.random.uniform(-start_xy_off, start_xy_off))
        dy_s = float(np.random.uniform(-start_xy_off, start_xy_off))
        sx += dx_s
        sy += dy_s
        logger.info(f"Start XY offset: dx={dx_s:.2f} m, dy={dy_s:.2f} m (range +/-{start_xy_off:.2f})")
    if end_xy_off > 0.0:
        dx_g = float(np.random.uniform(-end_xy_off, end_xy_off))
        dy_g = float(np.random.uniform(-end_xy_off, end_xy_off))
        gx += dx_g
        gy += dy_g
        logger.info(f"Goal XY offset: dx={dx_g:.2f} m, dy={dy_g:.2f} m (range +/-{end_xy_off:.2f})")

    # Clamp XY within map bounds
    sx = float(np.clip(sx, 0.0, float(cfg.space_dim[0])))
    sy = float(np.clip(sy, 0.0, float(cfg.space_dim[1])))
    gx = float(np.clip(gx, 0.0, float(cfg.space_dim[0])))
    gy = float(np.clip(gy, 0.0, float(cfg.space_dim[1])))

    # For mountains: ensure goal isn't inside terrain surface.
    terrain_clearance = float(path_cfg.get("terrain_clearance", 2.0))
    if cfg.terrain_type == "mountains" and terrain is not None:
        ground_goal = float(terrain.height_at(gx, gy))
        min_goal_z = ground_goal + terrain_clearance
        if gz < min_goal_z:
            old_gz = gz
            gz = min_goal_z
            gz_rel = gz / float(cfg.space_dim[2]) if float(cfg.space_dim[2]) > 0 else 0.0
            logger.info(
                f"Raised goal Z above terrain: {old_gz:.2f} -> {gz:.2f} m "
                f"(ground={ground_goal:.2f}, clearance={terrain_clearance:.2f})"
            )

    path_start = np.array([sx, sy, sz], dtype=float)
    path_goal = np.array([gx, gy, gz], dtype=float)

    # Normalize path config and pass absolute start/goal.
    path_cfg["end_relative_z"] = gz_rel
    path_cfg["start_abs"] = path_start.tolist()
    path_cfg["goal_abs"] = path_goal.tolist()

    logger.info(f"Start relative: [{sx_rel:.3f}, {sy_rel:.3f}, {sz_rel:.3f}]")
    logger.info(f"Goal relative: [{gx_rel:.3f}, {gy_rel:.3f}, {gz_rel:.3f}]")
    logger.info(f"Space dimensions: [{cfg.space_dim[0]:.1f}, {cfg.space_dim[1]:.1f}, {cfg.space_dim[2]:.1f}]")
    logger.info(f"Start position (absolute): [{path_start[0]:.2f}, {path_start[1]:.2f}, {path_start[2]:.2f}]")
    logger.info(f"Goal position (absolute): [{path_goal[0]:.2f}, {path_goal[1]:.2f}, {path_goal[2]:.2f}]")

    register_builtin_components()

    # Airframe is created before planning (not just before backend/controller) so a
    # curvature-aware planner (e.g. "dubins") can read min_turn_radius_m from
    # Capabilities, per plan 04's design -- quad/hex/octo report None here (they can
    # turn in place), so this is a no-op for them.
    airframe_name = str(cfg_norm.airframe_name).lower()
    airframe = create_airframe(airframe_name)
    logger.info(
        f"Airframe: {airframe_name.upper()} ({airframe.capabilities.n_actuators} actuators, "
        f"command_kind={airframe.capabilities.command_kind.name})"
    )
    if airframe.capabilities.min_turn_radius_m is not None:
        path_cfg.setdefault("turn_radius_m", airframe.capabilities.min_turn_radius_m)

    planner_type_raw = str(path_cfg.get("planner_type", "straight")).lower()
    planner_type = "rrtstar" if planner_type_raw in {"rrt*", "rrt_star", "rrtstar"} else planner_type_raw
    planner = create_planner(planner_type)

    logger.info(f"Planning path using: {planner_type.upper()}")
    waypoints = planner.plan(
        start=path_start,
        goal=path_goal,
        obstacles=obstacles,
        cfg={
            "path": path_cfg,
            "space_dim": cfg.space_dim,
            "start_pos": cfg.start_pos,
        },
    )
    logger.info(f"Planned path with {len(waypoints)} waypoints")

    # Verify planned path for collisions
    inflation = float(path_cfg.get("collision_inflation", 0.5))
    path_collisions = 0
    for i, wp in enumerate(waypoints):
        if is_point_in_collision(wp.position, obstacles, inflation=inflation):
            path_collisions += 1
            if path_collisions == 1:
                logger.warning(
                    f"Planned path has collisions! Waypoint {i} at "
                    f"[{wp.position[0]:.2f}, {wp.position[1]:.2f}, {wp.position[2]:.2f}]"
                )
    if path_collisions > 0:
        logger.warning(f"WARNING: {path_collisions}/{len(waypoints)} waypoints are in collision!")
    else:
        logger.info("Planned path verified: all waypoints collision-free")

    controller_name = str(cfg_norm.controller_name).lower()
    controller = create_controller(controller_name)

    backend_name = str(cfg_norm.backend_name).lower()
    backend = create_backend(backend_name)

    if controller.command_kind != airframe.capabilities.command_kind:
        raise ValueError(
            f"Controller '{controller_name}' produces {controller.command_kind.name} "
            f"control targets, but airframe '{airframe_name}' ({airframe.name}) accepts "
            f"{airframe.capabilities.command_kind.name}. Pick a controller/airframe pair "
            f"with matching command kinds."
        )

    initial_state = SimState(
        position=path_start.copy(),
        velocity=np.zeros(3, dtype=float),
        t=0.0,
    )
    backend.reset(
        initial_state=initial_state,
        world={
            "space_dim": cfg.space_dim.copy(),
            "max_z_allowed": max_z_allowed,
            "terrain": terrain,
            "terrain_clearance": terrain_clearance,
        },
        cfg={
            "controller": ctrl_cfg,
            "simulation": cfg_norm.simulation_cfg,
        },
    )

    dt = float(cfg_norm.dt)
    max_t = float(cfg_norm.sim_time)
    logger.info(f"Controller: {controller_name.upper()}")
    logger.info(f"Backend: {backend_name.upper()}")
    logger.info(f"Simulation time: {max_t:.1f} s, dt: {dt:.3f} s")

    n_cyl = sum(1 for o in obstacles if hasattr(o, "radius"))
    n_box = sum(1 for o in obstacles if hasattr(o, "size"))
    logger.info(f"Obstacle breakdown: {n_box} boxes, {n_cyl} cylinders (total {len(obstacles)})")

    steps = int(max_t / max(dt, 1e-9))
    logger.info(f"Sim steps: ~{steps} iterations")

    state0 = backend.state()
    traj_positions = [state0.position.copy()]
    traj_attitudes: list[np.ndarray] = []
    if state0.attitude_quat is None:
        traj_attitudes.append(np.full(4, np.nan, dtype=float))
    else:
        traj_attitudes.append(np.asarray(state0.attitude_quat, dtype=float).reshape(4))

    wp_idx = 0
    t = 0.0
    last_log_time = 0.0
    log_interval = 5.0
    collisions_detected = 0

    min_bounds = np.array([0.0, 0.0, 0.0], dtype=float)
    max_bounds = np.array([float(cfg.space_dim[0]), float(cfg.space_dim[1]), max_z_allowed], dtype=float)

    # PointMassBackend integrates accel_cmd directly and has no actuator concept, so
    # allocate() here is exercised for validation/logging only -- its output does not
    # feed back into the physics. This keeps the quad trajectory unchanged regardless
    # of which airframe is selected (CommandKind dispatch lands in phase 3).
    pm_cfg = dict(cfg_norm.simulation_cfg.get("pointmass", {}) or {})
    airframe_mass_kg = float(pm_cfg.get("mass", 1.0))
    gravity_mps2 = 9.81

    while t < max_t and wp_idx < len(waypoints):
        state = backend.state()
        pos = state.position
        vel = state.velocity

        target_wp = waypoints[wp_idx]
        dist_to_target = np.linalg.norm(target_wp.position - pos)
        if dist_to_target < 1.0 and wp_idx < len(waypoints) - 1:
            wp_idx += 1
            target_wp = waypoints[wp_idx]
            logger.info(f"t={t:.2f}s: Reached waypoint {wp_idx-1}/{len(waypoints)-1}, advancing")

        if is_point_in_collision(pos, obstacles, inflation=0.0):
            collisions_detected += 1
            if collisions_detected == 1:
                logger.warning(
                    f"t={t:.2f}s: COLLISION DETECTED at [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
                )

        control_target = controller.compute(
            state=state,
            target_waypoint=Waypoint(position=np.asarray(target_wp.position, dtype=float)),
            cfg={"controller": ctrl_cfg},
        )

        if airframe.capabilities.command_kind == CommandKind.AIRSPEED_NAV:
            # Actuator-level path: the controller already produced a Wrench (it
            # has no accel_cmd concept), allocate() turns it into actuator
            # commands, and the backend consumes those via metadata rather than
            # accel_cmd -- see FixedWingBackend's docstring for why.
            wrench = control_target.metadata["wrench"]
            actuator_cmd = airframe.allocate(wrench, state)
            step_metadata = dict(control_target.metadata)
            step_metadata["actuator_cmd"] = actuator_cmd
            backend.step(ControlTarget(accel_cmd=np.zeros(3, dtype=float), metadata=step_metadata), dt)
        else:
            acc_cmd = np.asarray(control_target.accel_cmd, dtype=float)
            acc_mag = np.linalg.norm(acc_cmd)
            acc_max = float(ctrl_cfg.get("acc_max", 20.0))
            if acc_mag > acc_max:
                acc_cmd = acc_cmd * (acc_max / (acc_mag + 1e-6))

            wrench = Wrench(
                force_body=airframe_mass_kg * (acc_cmd + np.array([0.0, 0.0, gravity_mps2])),
                moment_body=np.zeros(3, dtype=float),
            )
            airframe.allocate(wrench, state)

            backend.step(ControlTarget(accel_cmd=acc_cmd, metadata=control_target.metadata), dt)
        backend.apply_constraints(
            min_bounds=min_bounds,
            max_bounds=max_bounds,
            terrain=terrain,
            terrain_clearance=terrain_clearance,
        )

        new_state = backend.state()
        t = float(new_state.t)
        traj_positions.append(new_state.position.copy())
        if new_state.attitude_quat is None:
            traj_attitudes.append(np.full(4, np.nan, dtype=float))
        else:
            traj_attitudes.append(np.asarray(new_state.attitude_quat, dtype=float).reshape(4))

        if t - last_log_time >= log_interval:
            dist_to_goal = np.linalg.norm(waypoints[-1].position - new_state.position)
            logger.info(
                f"t={t:.1f}s: pos=[{new_state.position[0]:.1f}, {new_state.position[1]:.1f}, "
                f"{new_state.position[2]:.1f}], vel={np.linalg.norm(new_state.velocity):.2f} m/s, "
                f"waypoint {wp_idx}/{len(waypoints)-1}, dist_to_goal={dist_to_goal:.1f} m"
            )
            last_log_time = t

    trajectory = np.vstack(traj_positions)
    planned_waypoints = np.array([wp.position for wp in waypoints])
    attitude_quats_arr = np.vstack(traj_attitudes)
    if not np.isfinite(attitude_quats_arr).any():
        attitude_quats = None
    else:
        attitude_quats = attitude_quats_arr

    dist_to_goal = np.linalg.norm(waypoints[-1].position - trajectory[-1])
    logger.info("Simulation completed")
    logger.info(f"Final time: {t:.2f} s")
    logger.info(f"Final position: [{trajectory[-1, 0]:.2f}, {trajectory[-1, 1]:.2f}, {trajectory[-1, 2]:.2f}]")
    logger.info(f"Final waypoint reached: {wp_idx}/{len(waypoints)-1}")
    logger.info(f"Distance to goal: {dist_to_goal:.2f} m")

    return TrajectoryLog(
        trajectory=trajectory,
        planned_waypoints=planned_waypoints,
        obstacles=list(obstacles),
        space_dim=np.asarray(cfg.space_dim, dtype=float).copy(),
        terrain_type=str(cfg.terrain_type),
        visual_cfg=vis_cfg,
        goal_position=path_goal,
        planner_type=planner_type,
        final_time=float(t),
        final_waypoint_index=int(wp_idx),
        final_waypoint_count=max(0, len(waypoints) - 1),
        distance_to_goal=float(dist_to_goal),
        collisions_detected=int(collisions_detected),
        attitude_quats=attitude_quats,
        backend_name=backend_name,
    )
