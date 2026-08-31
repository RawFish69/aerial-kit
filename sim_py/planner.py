"""
Very simple path planner for the standalone simulator.

Currently we generate a straight-line path in 3D from start to goal and
sample waypoints along it. Obstacle avoidance is not yet implemented but
the obstacle list is passed through so this can be extended later.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Tuple, Dict

import heapq
import math
import numpy as np

from aerial_kit.sim.terrain import (
    BoxObstacle,
    CylinderObstacle,
    HeightFieldTerrain,
    is_point_in_collision,
)

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    position: np.ndarray  # [x, y, z]


def straight_line_path(
    start: np.ndarray,
    goal: np.ndarray,
    num_waypoints: int = 50,
) -> List[Waypoint]:
    """Generate a simple straight-line set of waypoints."""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)

    waypoints: List[Waypoint] = []
    for s in np.linspace(0.0, 1.0, num_waypoints):
        pos = (1.0 - s) * start + s * goal
        waypoints.append(Waypoint(position=pos))
    return waypoints


def _astar_2d(
    start_xy: np.ndarray,
    goal_xy: np.ndarray,
    space_dim: np.ndarray,
    obstacles: list[CylinderObstacle | BoxObstacle],
    grid_resolution: float,
    inflation: float,
) -> List[np.ndarray]:
    """Grid-based A* in XY plane, ignoring Z."""
    start_xy = np.asarray(start_xy, dtype=float)
    goal_xy = np.asarray(goal_xy, dtype=float)
    space_dim = np.asarray(space_dim, dtype=float)

    res = float(grid_resolution)
    nx = int(math.ceil(space_dim[0] / res)) + 1
    ny = int(math.ceil(space_dim[1] / res)) + 1
    logger.info(f"  A* grid: {nx}x{ny} cells, resolution={res:.2f}m, inflation={inflation:.2f}m")

    def to_grid(p: np.ndarray) -> Tuple[int, int]:
        return int(round(p[0] / res)), int(round(p[1] / res))

    def to_world(i: int, j: int) -> np.ndarray:
        return np.array([i * res, j * res], dtype=float)

    # Ensure start/goal are collision-free (adjust slightly if needed)
    if is_point_in_collision(start_xy, obstacles, inflation=inflation):
        logger.warning(f"  Start position in collision, adjusting...")
        # Try nearby positions
        for offset in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0), (1.0, 1.0)]:
            test_pos = start_xy + np.array(offset, dtype=float)
            if not is_point_in_collision(test_pos, obstacles, inflation=inflation):
                start_xy = test_pos
                logger.info(f"  Adjusted start to: [{start_xy[0]:.2f}, {start_xy[1]:.2f}]")
                break

    if is_point_in_collision(goal_xy, obstacles, inflation=inflation):
        logger.warning(f"  Goal position in collision, adjusting...")
        for offset in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0), (1.0, 1.0)]:
            test_pos = goal_xy + np.array(offset, dtype=float)
            if not is_point_in_collision(test_pos, obstacles, inflation=inflation):
                goal_xy = test_pos
                logger.info(f"  Adjusted goal to: [{goal_xy[0]:.2f}, {goal_xy[1]:.2f}]")
                break

    start_node = to_grid(start_xy)
    goal_node = to_grid(goal_xy)

    def in_bounds(i: int, j: int) -> bool:
        return 0 <= i < nx and 0 <= j < ny

    def is_free(i: int, j: int) -> bool:
        p = to_world(i, j)
        return not is_point_in_collision(p, obstacles, inflation=inflation)

    def edge_free(p1: np.ndarray, p2: np.ndarray) -> bool:
        """Check if edge between two points is collision-free."""
        check_res = min(0.3, float(res))
        steps = max(2, int(np.ceil(np.linalg.norm(p2 - p1) / check_res)))
        for s in np.linspace(0.0, 1.0, steps):
            p = (1.0 - s) * p1 + s * p2
            if is_point_in_collision(p, obstacles, inflation=inflation):
                return False
        return True

    open_set: List[Tuple[float, Tuple[int, int]]] = []
    heapq.heappush(open_set, (0.0, start_node))
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], float] = {start_node: 0.0}

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        pa, pb = to_world(*a), to_world(*b)
        return float(np.linalg.norm(pa - pb))

    neighbors = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    ]

    closed: set[Tuple[int, int]] = set()
    iterations = 0

    while open_set:
        iterations += 1
        _, current = heapq.heappop(open_set)
        if current == goal_node:
            # Reconstruct path
            path_nodes = [current]
            while current in came_from:
                current = came_from[current]
                path_nodes.append(current)
            path_nodes.reverse()
            # Convert to world coordinates and verify edges are collision-free
            world_path = [to_world(i, j) for (i, j) in path_nodes]
            logger.info(f"  A* found path in {iterations} iterations, {len(world_path)} nodes")
            # Smooth/verify: check each edge
            verified_path = [world_path[0]]
            edges_removed = 0
            for i in range(1, len(world_path)):
                if edge_free(verified_path[-1], world_path[i]):
                    verified_path.append(world_path[i])
                else:
                    # Edge has collision, try intermediate point
                    edges_removed += 1
                    mid = (verified_path[-1] + world_path[i]) / 2.0
                    if not is_point_in_collision(mid, obstacles, inflation=inflation):
                        verified_path.append(mid)
            if edges_removed > 0:
                logger.warning(f"  Removed {edges_removed} edges with collisions, final path: {len(verified_path)} points")
            return verified_path

        if current in closed:
            continue
        closed.add(current)

        if iterations % 5000 == 0:
            logger.info(
                f"  A* progress: it={iterations}, open={len(open_set)}, closed={len(closed)}"
            )

        for di, dj in neighbors:
            ni, nj = current[0] + di, current[1] + dj
            if not in_bounds(ni, nj):
                continue
            if not is_free(ni, nj):
                continue
            # Check edge collision
            p_current = to_world(*current)
            p_neighbor = to_world(ni, nj)
            if not edge_free(p_current, p_neighbor):
                continue
            tentative_g = g_score[current] + heuristic(current, (ni, nj))
            if tentative_g < g_score.get((ni, nj), float("inf")):
                came_from[(ni, nj)] = current
                g_score[(ni, nj)] = tentative_g
                f = tentative_g + heuristic((ni, nj), goal_node)
                heapq.heappush(open_set, (f, (ni, nj)))

    # Fallback: no path found
    logger.error(f"  A* failed to find path after {iterations} iterations!")
    return [start_xy, goal_xy]


def _rrt_2d(
    start_xy: np.ndarray,
    goal_xy: np.ndarray,
    space_dim: np.ndarray,
    obstacles: list[CylinderObstacle | BoxObstacle],
    step_size: float,
    goal_tolerance: float,
    max_iterations: int,
    inflation: float,
) -> List[np.ndarray]:
    """Simple 2D RRT in XY plane."""
    start_xy = np.asarray(start_xy, dtype=float)
    goal_xy = np.asarray(goal_xy, dtype=float)
    space_dim = np.asarray(space_dim, dtype=float)

    nodes: List[np.ndarray] = [start_xy]
    parents: Dict[int, int] = {0: -1}

    def sample() -> np.ndarray:
        if np.random.rand() < 0.1:
            return goal_xy
        return np.array(
            [
                np.random.uniform(0.0, space_dim[0]),
                np.random.uniform(0.0, space_dim[1]),
            ],
            dtype=float,
        )

    def nearest(p: np.ndarray) -> int:
        dists = [np.linalg.norm(n - p) for n in nodes]
        return int(np.argmin(dists))

    def steer(p_from: np.ndarray, p_to: np.ndarray) -> np.ndarray:
        direction = p_to - p_from
        dist = np.linalg.norm(direction)
        if dist <= step_size:
            return p_to
        return p_from + direction / dist * step_size

    def collision_free(p_from: np.ndarray, p_to: np.ndarray) -> bool:
        # Check a few intermediate points with fine resolution
        check_dist = min(0.3, max(step_size / 2.0, 0.1))
        steps = max(2, int(np.ceil(np.linalg.norm(p_to - p_from) / check_dist)))
        for s in np.linspace(0.0, 1.0, steps):
            p = (1.0 - s) * p_from + s * p_to
            if is_point_in_collision(p, obstacles, inflation=inflation):
                return False
        return True

    for it in range(max_iterations):
        p_rand = sample()
        idx_near = nearest(p_rand)
        p_near = nodes[idx_near]
        p_new = steer(p_near, p_rand)
        if not collision_free(p_near, p_new):
            continue
        nodes.append(p_new)
        parents[len(nodes) - 1] = idx_near
        if np.linalg.norm(p_new - goal_xy) <= goal_tolerance:
            # Reconstruct path
            path_points: List[np.ndarray] = [p_new]
            cur = len(nodes) - 1
            while parents[cur] != -1:
                cur = parents[cur]
                path_points.append(nodes[cur])
            path_points.reverse()
            return path_points

    # Fallback: connect directly if no path found
    return [start_xy, goal_xy]


def _rrt_star_2d(
    start_xy: np.ndarray,
    goal_xy: np.ndarray,
    space_dim: np.ndarray,
    obstacles: list[CylinderObstacle | BoxObstacle | HeightFieldTerrain],
    step_size: float,
    goal_tolerance: float,
    max_iterations: int,
    inflation: float,
    rewire_radius: float | None = None,
    goal_sample_rate: float = 0.1,
) -> List[np.ndarray]:
    """2D RRT* in XY plane (rewiring for lower-cost paths)."""
    start_xy = np.asarray(start_xy, dtype=float)
    goal_xy = np.asarray(goal_xy, dtype=float)
    space_dim = np.asarray(space_dim, dtype=float)

    nodes: List[np.ndarray] = [start_xy]
    parents: Dict[int, int] = {0: -1}
    costs: Dict[int, float] = {0: 0.0}

    step_size = float(step_size)
    goal_tolerance = float(goal_tolerance)
    max_iterations = int(max_iterations)
    inflation = float(inflation)
    goal_sample_rate = float(goal_sample_rate)

    def sample() -> np.ndarray:
        if np.random.rand() < goal_sample_rate:
            return goal_xy
        return np.array(
            [
                np.random.uniform(0.0, space_dim[0]),
                np.random.uniform(0.0, space_dim[1]),
            ],
            dtype=float,
        )

    def nearest(p: np.ndarray) -> int:
        dists = [np.linalg.norm(n - p) for n in nodes]
        return int(np.argmin(dists))

    def steer(p_from: np.ndarray, p_to: np.ndarray) -> np.ndarray:
        direction = p_to - p_from
        dist = np.linalg.norm(direction)
        if dist <= step_size:
            return p_to
        return p_from + direction / dist * step_size

    def collision_free(p_from: np.ndarray, p_to: np.ndarray) -> bool:
        check_dist = min(0.3, max(step_size / 2.0, 0.1))
        steps = max(2, int(np.ceil(np.linalg.norm(p_to - p_from) / check_dist)))
        for s in np.linspace(0.0, 1.0, steps):
            p = (1.0 - s) * p_from + s * p_to
            if is_point_in_collision(p, obstacles, inflation=inflation):
                return False
        return True

    def near_indices(p: np.ndarray, radius: float) -> List[int]:
        r = float(radius)
        idxs: List[int] = []
        for i, n in enumerate(nodes):
            if np.linalg.norm(n - p) <= r:
                idxs.append(i)
        return idxs

    best_goal_idx: int | None = None
    best_goal_cost = float("inf")

    for it in range(max_iterations):
        if it > 0 and it % 5000 == 0:
            logger.info(f"  RRT*: it={it}, nodes={len(nodes)}, best_goal_cost={best_goal_cost:.1f}")

        p_rand = sample()
        idx_near = nearest(p_rand)
        p_near = nodes[idx_near]
        p_new = steer(p_near, p_rand)

        if not collision_free(p_near, p_new):
            continue

        # Choose a parent among nearby nodes that minimizes cost
        if rewire_radius is None:
            # Standard RRT* radius heuristic in 2D: r(n) ~ gamma * sqrt(log(n)/n)
            n = max(2, len(nodes))
            gamma = 2.0 * step_size
            r = gamma * math.sqrt(math.log(n) / n)
            r = max(r, step_size * 1.5)
        else:
            r = float(rewire_radius)

        near = near_indices(p_new, r)
        parent = idx_near
        best_cost = costs[idx_near] + float(np.linalg.norm(p_new - p_near))

        for j in near:
            pj = nodes[j]
            c = costs[j] + float(np.linalg.norm(p_new - pj))
            if c < best_cost and collision_free(pj, p_new):
                best_cost = c
                parent = j

        nodes.append(p_new)
        new_idx = len(nodes) - 1
        parents[new_idx] = parent
        costs[new_idx] = best_cost

        # Rewire nearby nodes through this new node if cheaper
        for j in near:
            if j == 0 or j == new_idx:
                continue
            pj = nodes[j]
            c_through = best_cost + float(np.linalg.norm(pj - p_new))
            if c_through + 1e-9 < costs.get(j, float("inf")) and collision_free(p_new, pj):
                parents[j] = new_idx
                costs[j] = c_through

        # Track best node within goal tolerance (keeps optimizing even after first reach)
        d_goal = float(np.linalg.norm(p_new - goal_xy))
        if d_goal <= goal_tolerance:
            if best_cost < best_goal_cost:
                best_goal_cost = best_cost
                best_goal_idx = new_idx

    # Reconstruct best path if reached; else fallback
    if best_goal_idx is None:
        return [start_xy, goal_xy]

    path_points: List[np.ndarray] = [nodes[best_goal_idx]]
    cur = best_goal_idx
    while parents[cur] != -1:
        cur = parents[cur]
        path_points.append(nodes[cur])
    path_points.reverse()
    return path_points


def default_plan(
    space_dim: np.ndarray,
    start_pos: np.ndarray,
    obstacles: list[CylinderObstacle | BoxObstacle | HeightFieldTerrain],
    path_cfg: dict | None = None,
) -> List[Waypoint]:
    """
    Create a path from start to goal.

    Planner type is selected via path_cfg["planner_type"]:
      - "straight" (default)
      - "astar"
      - "rrt"
    """
    if path_cfg is None:
        path_cfg = {}

    # Allow simulator to pass absolute start/goal (keeps planner consistent with run_sim offsets).
    start_abs = path_cfg.get("start_abs")
    goal_abs = path_cfg.get("goal_abs")
    if start_abs is not None and goal_abs is not None:
        start = np.asarray(start_abs, dtype=float).reshape(3)
        goal = np.asarray(goal_abs, dtype=float).reshape(3)
    else:
        sx_rel = float(path_cfg.get("start_relative_x", 0.02))
        sy_rel = float(path_cfg.get("start_relative_y", 0.02))
        sz_rel = float(path_cfg.get("start_relative_z", 0.05))
        gx_rel = float(path_cfg.get("end_relative_x", 0.98))
        gy_rel = float(path_cfg.get("end_relative_y", 0.98))
        gz_rel = float(path_cfg.get("end_relative_z", 0.9))

        sx = sx_rel * float(space_dim[0])
        sy = sy_rel * float(space_dim[1])
        sz = sz_rel * float(space_dim[2])

        gx = gx_rel * float(space_dim[0])
        gy = gy_rel * float(space_dim[1])
        gz = gz_rel * float(space_dim[2])

        start = np.array([sx, sy, sz], dtype=float)
        goal = np.array([gx, gy, gz], dtype=float)

    planner_type = str(path_cfg.get("planner_type", "straight")).lower()
    if planner_type in {"rrt*", "rrtstar", "rrt_star"}:
        planner_type = "rrtstar"
    terrain = next((o for o in obstacles if isinstance(o, HeightFieldTerrain)), None)
    terrain_clearance = float(path_cfg.get("terrain_clearance", 2.0))
    if planner_type == "astar":
        grid_res = float(path_cfg.get("grid_resolution", 2.0))
        inflation = float(path_cfg.get("collision_inflation", 0.5))
        logger.info(f"  Running A* planner (avoiding {len(obstacles)} obstacles)...")
        xy_path = _astar_2d(start[:2], goal[:2], space_dim, obstacles, grid_res, inflation)
        logger.info(f"  A* path has {len(xy_path)} waypoints")
        waypoints: List[Waypoint] = []
        for i, p_xy in enumerate(xy_path):
            s = i / max(1, len(xy_path) - 1)
            z = (1.0 - s) * start[2] + s * goal[2]
            if terrain is not None:
                z = max(z, terrain.height_at(float(p_xy[0]), float(p_xy[1])) + terrain_clearance)
            waypoints.append(Waypoint(position=np.array([p_xy[0], p_xy[1], z], dtype=float)))
        return waypoints
    elif planner_type == "rrt":
        step_size = float(path_cfg.get("rrt_step_size", 5.0))
        goal_tol = float(path_cfg.get("goal_tolerance", 2.0))
        max_iter = int(path_cfg.get("rrt_max_iterations", 2000))
        inflation = float(path_cfg.get("collision_inflation", 0.5))
        logger.info(f"  Running RRT planner (avoiding {len(obstacles)} obstacles)...")
        xy_path = _rrt_2d(
            start[:2],
            goal[:2],
            space_dim,
            obstacles,
            step_size=step_size,
            goal_tolerance=goal_tol,
            max_iterations=max_iter,
            inflation=inflation,
        )
        logger.info(f"  RRT path has {len(xy_path)} waypoints")
        waypoints: List[Waypoint] = []
        for i, p_xy in enumerate(xy_path):
            s = i / max(1, len(xy_path) - 1)
            z = (1.0 - s) * start[2] + s * goal[2]
            if terrain is not None:
                z = max(z, terrain.height_at(float(p_xy[0]), float(p_xy[1])) + terrain_clearance)
            waypoints.append(Waypoint(position=np.array([p_xy[0], p_xy[1], z], dtype=float)))
        return waypoints
    elif planner_type == "rrtstar":
        step_size = float(path_cfg.get("rrt_step_size", 5.0))
        goal_tol = float(path_cfg.get("goal_tolerance", 2.0))
        max_iter = int(path_cfg.get("rrt_max_iterations", 2000))
        inflation = float(path_cfg.get("collision_inflation", 0.5))
        rewire_radius = path_cfg.get("rrt_star_rewire_radius", None)
        rewire_radius_f = None if rewire_radius is None else float(rewire_radius)
        goal_sample_rate = float(path_cfg.get("rrt_goal_sample_rate", 0.1))
        logger.info(f"  Running RRT* planner (avoiding {len(obstacles)} obstacles)...")
        xy_path = _rrt_star_2d(
            start[:2],
            goal[:2],
            space_dim,
            obstacles,
            step_size=step_size,
            goal_tolerance=goal_tol,
            max_iterations=max_iter,
            inflation=inflation,
            rewire_radius=rewire_radius_f,
            goal_sample_rate=goal_sample_rate,
        )
        logger.info(f"  RRT* path has {len(xy_path)} waypoints")
        waypoints: List[Waypoint] = []
        for i, p_xy in enumerate(xy_path):
            s = i / max(1, len(xy_path) - 1)
            z = (1.0 - s) * start[2] + s * goal[2]
            if terrain is not None:
                z = max(z, terrain.height_at(float(p_xy[0]), float(p_xy[1])) + terrain_clearance)
            waypoints.append(Waypoint(position=np.array([p_xy[0], p_xy[1], z], dtype=float)))
        return waypoints
    else:
        logger.warning(f"  Using STRAIGHT-LINE planner (NO obstacle avoidance!)")
        waypoints = straight_line_path(start, goal, num_waypoints=80)
        if terrain is not None:
            for wp in waypoints:
                wp.position[2] = max(
                    float(wp.position[2]),
                    terrain.height_at(float(wp.position[0]), float(wp.position[1])) + terrain_clearance,
                )
        return waypoints


