#!/usr/bin/env python3
"""Record /uav/backend/telemetry_raw during a twin-wing demo and summarise it.

Run beside a fixed-wing launch (ROS sourced). Exits when the demo's final
waypoint is reached or ``--timeout`` passes, then prints a JSON summary:
altitude band after arming, horizontal error to the final waypoint,
airspeed band, NaN count. Used to produce docs/evidence measurements.

    python3 scripts/fixed_wing_mission_monitor.py --out summary.json
"""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from uav_msgs.msg import Telemetry

FINAL_WP = (260.0, -40.0)
CRUISE_Z = 55.0


class Monitor(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('fixed_wing_mission_monitor')
        self.samples = []
        self.create_subscription(Telemetry, topic, self._on, 50)

    def _on(self, msg: Telemetry) -> None:
        p = msg.pose.position
        self.samples.append((time.monotonic(), bool(msg.armed), p.x, p.y, p.z,
                             float(msg.airspeed_mps)))


def summarise(samples, accept_m: float = 12.0) -> dict:
    armed = [s for s in samples if s[1]]
    nan = sum(1 for s in samples if not all(math.isfinite(v) for v in s[2:]))
    if not armed:
        return {'samples': len(samples), 'armed_samples': 0, 'nan_samples': nan}
    t0 = armed[0][0]
    # Ignore the first 2 s after arming: the start transient is not cruise.
    cruise = [s for s in armed if s[0] - t0 > 2.0] or armed
    zs = [s[4] for s in cruise]
    vs = [s[5] for s in cruise]
    last = armed[-1]
    reached = next(
        (s for s in armed if math.hypot(s[2] - FINAL_WP[0], s[3] - FINAL_WP[1]) < accept_m),
        None)
    end = reached or last
    return {
        'samples': len(samples),
        'armed_samples': len(armed),
        'nan_samples': nan,
        'flight_time_s': round(end[0] - t0, 2),
        'altitude_min_m': round(min(zs), 2),
        'altitude_max_m': round(max(zs), 2),
        'altitude_max_abs_error_m': round(max(abs(z - CRUISE_Z) for z in zs), 2),
        'airspeed_min_mps': round(min(vs), 2),
        'airspeed_max_mps': round(max(vs), 2),
        'final_position_m': [round(end[2], 2), round(end[3], 2), round(end[4], 2)],
        'final_horizontal_error_m': round(
            math.hypot(end[2] - FINAL_WP[0], end[3] - FINAL_WP[1]), 2),
        'final_waypoint_reached': reached is not None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/uav/backend/telemetry_raw')
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    rclpy.init()
    node = Monitor(args.topic)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.samples and node.samples[-1][1]:
                s = node.samples[-1]
                if math.hypot(s[2] - FINAL_WP[0], s[3] - FINAL_WP[1]) < 12.0:
                    break
    except KeyboardInterrupt:
        pass
    summary = summarise(node.samples)
    text = json.dumps(summary, indent=2)
    print(text)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as handle:
            handle.write(text + '\n')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
