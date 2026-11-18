#!/usr/bin/env python3
"""Record /uav/backend/odom during a multirotor demo and summarise it as JSON.

    python3 scripts/multirotor_mission_monitor.py --out summary.json --timeout 300

Stops when the log file given by --log shows 'Demo: disarm' (or on timeout).
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

GOAL = (170.0, 120.0, 3.0)


class Monitor(Node):
    def __init__(self):
        super().__init__('multirotor_mission_monitor')
        self.samples = []
        self.create_subscription(Odometry, '/uav/backend/odom', self._on, 50)

    def _on(self, m):
        p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        tilt = math.degrees(2 * math.asin(min(1.0, math.hypot(q.x, q.y))))
        self.samples.append((time.monotonic(), p.x, p.y, p.z, math.hypot(v.x, v.y), v.z, tilt))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--log', default='')
    ap.add_argument('--timeout', type=float, default=300.0)
    a = ap.parse_args()
    rclpy.init()
    node = Monitor()
    end = time.monotonic() + a.timeout
    done_at = None
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if a.log and done_at is None and Path(a.log).is_file() and 'Demo: disarm' in Path(a.log).read_text(errors='ignore'):
            done_at = time.monotonic()
        if done_at is not None and time.monotonic() - done_at > 3.0:
            break
    s = node.samples
    finite = all(all(math.isfinite(x) for x in r[1:]) for r in s)
    summary = {
        'samples': len(s), 'all_finite': finite, 'demo_disarmed': done_at is not None,
        'max_altitude_m': round(max(r[3] for r in s), 2) if s else None,
        'max_ground_speed_mps': round(max(r[4] for r in s), 2) if s else None,
        'max_tilt_deg': round(max(r[6] for r in s), 1) if s else None,
        'final_position_m': [round(x, 2) for x in s[-1][1:4]] if s else None,
        'closest_to_goal_xy_m': round(min(math.hypot(r[1] - GOAL[0], r[2] - GOAL[1]) for r in s), 2) if s else None,
    }
    text = json.dumps(summary, indent=1)
    Path(a.out).write_text(text + '\n')
    print(text)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
