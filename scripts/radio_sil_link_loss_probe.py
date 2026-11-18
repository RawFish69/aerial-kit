#!/usr/bin/env python3
"""Cut the simulated ELRS link in flight and record the plant failsafe."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class LinkLossProbe(Node):
    def __init__(self) -> None:
        super().__init__('radio_sil_link_loss_probe')
        self.cut_requested = False
        self.cut_at = None
        self.cut_altitude = None
        self.max_altitude_after_cut = 0.0
        self.latest_altitude = None
        self.latest_enabled = None
        self.disable_seen_after_cut = False
        self.samples = 0

        self.pub_cut = self.create_publisher(Bool, '/uav/radio/link_cut', 10)
        self.create_subscription(Odometry, '/uav/radio/plant_odom', self._on_odom, 50)
        self.create_subscription(Bool, '/uav/radio/decoded_enable', self._on_enable, 20)

    def cut_now(self) -> None:
        msg = Bool()
        msg.data = True
        self.pub_cut.publish(msg)
        self.cut_requested = True
        self.cut_at = time.monotonic()
        self.get_logger().warning(f'link cut at {self.cut_altitude:.2f} m')

    def _on_odom(self, msg: Odometry) -> None:
        altitude = float(msg.pose.pose.position.z)
        if not math.isfinite(altitude):
            return
        self.samples += 1
        self.latest_altitude = altitude
        if self.cut_requested:
            self.max_altitude_after_cut = max(self.max_altitude_after_cut, altitude)
        elif altitude >= 2.0:
            self.cut_altitude = altitude
            self.cut_now()

    def _on_enable(self, msg: Bool) -> None:
        self.latest_enabled = bool(msg.data)
        if self.cut_requested and not self.latest_enabled:
            self.disable_seen_after_cut = True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = LinkLossProbe()
    started = time.monotonic()
    reason = 'timeout before takeoff'
    success = False
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.cut_requested:
                continue
            elapsed_after_cut = time.monotonic() - node.cut_at
            if (
                node.disable_seen_after_cut
                and elapsed_after_cut >= 2.0
                and node.latest_altitude is not None
                and node.latest_altitude <= 0.4
            ):
                reason = 'plant disabled and landed after link cut'
                success = True
                break
            if elapsed_after_cut > 75.0:
                reason = 'plant did not land within 75 s of link cut'
                break
    finally:
        summary = {
            'success': success,
            'reason': reason,
            'cut_published': node.cut_requested,
            'altitude_at_cut_m': round(node.cut_altitude, 2) if node.cut_altitude is not None else None,
            'max_altitude_after_cut_m': round(node.max_altitude_after_cut, 2),
            'final_altitude_m': round(node.latest_altitude, 2) if node.latest_altitude is not None else None,
            'receiver_disable_seen': node.disable_seen_after_cut,
            'final_receiver_enable': node.latest_enabled,
            'plant_samples': node.samples,
        }
        text = json.dumps(summary, indent=1)
        Path(args.out).write_text(text + '\n')
        print(text)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
