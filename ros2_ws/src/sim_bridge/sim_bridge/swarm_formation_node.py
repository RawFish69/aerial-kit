"""Fly a swarm in formation over the per-drone backend topics.

Reads ``/<prefix><i>/backend/odom`` for every drone and sends
``/<prefix><i>/backend/cmd_twist`` + ``enable``: the same velocity law as
``sim_py.swarm`` (slot P-control + short-range separation, nearest-slot
assignment on every formation change).

Commands on ``/swarm/formation`` (std_msgs/String): ``takeoff [z]``,
``grid [z]``, ``circle [z]``, ``line [z]``, ``land``. With ``demo:=true``
it runs takeoff -> circle -> line -> grid -> land by itself. Status (JSON) on
``/swarm/status``: phase, worst slot error, closest pair.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

for _parent in Path(__file__).resolve().parents:
    if (_parent / 'aerial_kit').is_dir() and (_parent / 'sim_py').is_dir():
        if str(_parent) not in sys.path:
            sys.path.insert(0, str(_parent))
        break

from sim_py.swarm import (  # noqa: E402
    FORMATIONS, FormationGains, assign_slots, formation_slots, formation_velocity, transit_targets,
)

STAGE_TOLERANCE_M = 0.2

DEMO = (('takeoff', 2.0, 6.0), ('circle', 3.0, 14.0), ('line', 2.5, 14.0), ('grid', 2.0, 12.0), ('land', 0.0, 10.0))


class SwarmFormationNode(Node):
    def __init__(self) -> None:
        super().__init__('swarm_formation_node')
        for name, default in (('n', 4), ('namespace_prefix', 'uav'), ('rate_hz', 20.0), ('spacing_m', 1.5),
                              ('demo', False), ('land_rate_mps', 0.6)):
            self.declare_parameter(name, default)
        g = self.get_parameter
        self.n = int(g('n').value)
        self.prefix = str(g('namespace_prefix').value).strip('/') or 'uav'
        self.spacing = float(g('spacing_m').value)
        self.land_rate = float(g('land_rate_mps').value)
        self.gains = FormationGains()
        self.pos = np.full((self.n, 3), np.nan)
        self.cmd_pubs, self.en_pubs = [], []
        for i in range(self.n):
            ns = f'/{self.prefix}{i}/backend'
            self.create_subscription(Odometry, f'{ns}/odom', lambda m, i=i: self._on_odom(i, m), 10)
            self.cmd_pubs.append(self.create_publisher(Twist, f'{ns}/cmd_twist', 10))
            self.en_pubs.append(self.create_publisher(Bool, f'{ns}/enable', 10))
        self.create_subscription(String, '/swarm/formation', self._on_formation, 10)
        self.pub_status = self.create_publisher(String, '/swarm/status', 10)
        self.phase = 'idle'
        self.targets: np.ndarray | None = None
        self.stages: list[np.ndarray] = []      # transit legs still to fly before ``targets``
        self.armed = False
        self.min_sep_seen = math.inf
        self.demo = list(DEMO) if bool(g('demo').value) else []
        self.phase_end = None
        self.create_timer(1.0 / float(g('rate_hz').value), self._tick)

    def _on_odom(self, i: int, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos[i] = (p.x, p.y, p.z)

    def _on_formation(self, msg: String) -> None:
        parts = msg.data.split()
        if not parts:
            return
        try:
            z = float(parts[1]) if len(parts) > 1 else None
            self.command(parts[0].lower(), z)
        except ValueError as exc:
            self.get_logger().warning(f'bad /swarm/formation {msg.data!r}: {exc}')

    def command(self, name: str, z: float | None = None) -> None:
        if np.isnan(self.pos).any():
            raise ValueError('no odometry from every drone yet')
        if name == 'takeoff':
            self.armed = True
            self.targets = np.column_stack([self.pos[:, :2], np.full(self.n, z if z is not None else 2.0)])
            self.stages = []
        elif name == 'land':
            self.targets = None
            self.stages = []
        elif name in FORMATIONS:
            if not self.armed:
                raise ValueError('take off first')
            slots = formation_slots(name, self.n, z if z is not None else 2.5, spacing_m=self.spacing)
            self.targets = slots[assign_slots(self.pos, slots)]
            layered = transit_targets(self.targets)
            # Into altitude layers in place, across, then down/up into the slots.
            self.stages = [np.column_stack([self.pos[:, :2], layered[:, 2]]), layered]
        else:
            raise ValueError(f'unknown formation {name!r}')
        self.phase = name
        self.get_logger().info(f'swarm: {name}' + ('' if z is None else f' at {z:.1f} m'))

    def _tick(self) -> None:
        now = time.monotonic()
        if self.demo and not np.isnan(self.pos).any() and (self.phase_end is None or now >= self.phase_end):
            name, z, seconds = self.demo.pop(0)
            self.command(name, z if name != 'land' else None)
            self.phase_end = now + seconds
        if np.isnan(self.pos).any():
            return
        if self.phase == 'land':
            v = formation_velocity(self.pos, self.pos, self.gains)
            v[:, 2] = -self.land_rate
            if np.all(self.pos[:, 2] < 0.15):
                self.armed = False      # adapters idle the motors once disarmed on the ground
        elif self.targets is not None:
            goal = self.stages[0] if self.stages else self.targets
            if self.stages and np.linalg.norm(self.pos - goal, axis=1).max() < STAGE_TOLERANCE_M:
                self.stages.pop(0)
                goal = self.stages[0] if self.stages else self.targets
            v = formation_velocity(self.pos, goal, self.gains)
        else:
            v = np.zeros((self.n, 3))
        for i in range(self.n):
            self.en_pubs[i].publish(Bool(data=self.armed))
            if self.armed:
                t = Twist()
                t.linear.x, t.linear.y, t.linear.z = (float(x) for x in v[i])
                self.cmd_pubs[i].publish(t)
        d = np.linalg.norm(self.pos[:, None] - self.pos[None], axis=-1) + np.eye(self.n) * 1e9
        sep = float(d.min()) if self.n > 1 else math.inf
        if self.armed:
            self.min_sep_seen = min(self.min_sep_seen, sep)
        err = None if self.targets is None else float(np.linalg.norm(self.pos - self.targets, axis=1).max())
        self.pub_status.publish(String(data=json.dumps({
            'phase': self.phase, 'armed': self.armed, 'max_slot_error_m': err,
            'min_separation_m': sep, 'min_separation_seen_m': self.min_sep_seen,
            'demo_remaining': [p[0] for p in self.demo]})))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SwarmFormationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
