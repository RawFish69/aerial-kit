"""Deterministic simulated ELRS RF link with rate, latency and packet loss."""

from __future__ import annotations

import random
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray

from aerial_kit.comms.crsf import (
    LinkStatisticsTelemetry,
    make_link_statistics_frame,
)


def delivery_enabled_for_cut(link_cut: bool) -> bool:
    """Map the ``link_cut`` topic's positive flag to delivery state."""
    return not bool(link_cut)


@dataclass(order=True)
class _Delivery:
    due: float
    sequence: int
    direction: str
    data: bytes


class SimElrsLinkNode(Node):
    def __init__(self) -> None:
        super().__init__('sim_elrs_link_node')
        defaults = {
            'rate_hz': 150.0,
            'latency_ms': 8.0,
            'packet_loss_percent': 0.0,
            'seed': 7,
            'host_tx_topic': '/uav/radio/host_tx_bytes',
            'host_rx_topic': '/uav/radio/host_rx_bytes',
            'receiver_rx_topic': '/uav/radio/receiver_rx_bytes',
            'receiver_tx_topic': '/uav/radio/receiver_tx_bytes',
            'link_cut_topic': '/uav/radio/link_cut',
            'link_stats_topic': '/uav/radio/link_stats',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        g = self.get_parameter
        self.rate_hz = max(float(g('rate_hz').value), 1.0)
        self.period = 1.0 / self.rate_hz
        self.latency = max(float(g('latency_ms').value), 0.0) / 1000.0
        self.loss_percent = max(0.0, min(100.0, float(g('packet_loss_percent').value)))
        self.rng = random.Random(int(g('seed').value))
        self.link_enabled = True
        self.queue: deque[_Delivery] = deque()
        self.sequence = 0
        self.last_forward = 0.0
        self.last_reverse = 0.0

        self.pub_receiver_rx = self.create_publisher(
            UInt8MultiArray, str(g('receiver_rx_topic').value), 20
        )
        self.pub_host_rx = self.create_publisher(
            UInt8MultiArray, str(g('host_rx_topic').value), 20
        )
        self.pub_stats = self.create_publisher(String, str(g('link_stats_topic').value), 10)
        self.create_subscription(
            UInt8MultiArray, str(g('host_tx_topic').value), self._on_host_tx, 20
        )
        self.create_subscription(
            UInt8MultiArray, str(g('receiver_tx_topic').value), self._on_receiver_tx, 20
        )
        self.create_subscription(Bool, str(g('link_cut_topic').value), self._on_cut, 10)
        self.create_timer(0.002, self._flush)
        self.create_timer(0.5, self._publish_stats)
        self.create_timer(0.2, self._emit_link_stats)
        self.get_logger().info(
            f'ELRS SIL: rate={self.rate_hz:.0f}Hz latency={self.latency*1000:.1f}ms '
            f'loss={self.loss_percent:.1f}%'
        )

    def _on_cut(self, msg: Bool) -> None:
        # /uav/radio/link_cut is true while the link is cut. Keep delivery
        # enabled for the normal false state so startup and recovery are safe.
        self.link_enabled = delivery_enabled_for_cut(bool(msg.data))
        self.get_logger().warn(f'RF link {"enabled" if self.link_enabled else "CUT"}')

    def _accept(self, now: float, last: float) -> tuple[bool, float]:
        if not self.link_enabled:
            return False, last
        if now - last < self.period * 0.8:
            return False, last
        if self.rng.random() * 100.0 < self.loss_percent:
            return False, now
        return True, now

    def _enqueue(self, direction: str, data: bytes) -> None:
        self.sequence += 1
        self.queue.append(
            _Delivery(time.monotonic() + self.latency, self.sequence, direction, bytes(data))
        )

    def _on_host_tx(self, msg: UInt8MultiArray) -> None:
        accepted, self.last_forward = self._accept(time.monotonic(), self.last_forward)
        if accepted:
            self._enqueue('forward', bytes(msg.data))

    def _on_receiver_tx(self, msg: UInt8MultiArray) -> None:
        accepted, self.last_reverse = self._accept(time.monotonic(), self.last_reverse)
        if accepted:
            self._enqueue('reverse', bytes(msg.data))

    def _publish_bytes(self, publisher, data: bytes) -> None:
        msg = UInt8MultiArray()
        msg.data = list(data)
        publisher.publish(msg)

    def _flush(self) -> None:
        now = time.monotonic()
        while self.queue and self.queue[0].due <= now:
            delivery = self.queue.popleft()
            if not self.link_enabled:
                continue
            if delivery.direction == 'forward':
                self._publish_bytes(self.pub_receiver_rx, delivery.data)
            else:
                self._publish_bytes(self.pub_host_rx, delivery.data)

    def _emit_link_stats(self) -> None:
        if not self.link_enabled:
            return
        stats = LinkStatisticsTelemetry(
            uplink_rssi_1_dbm=round(-45 - self.loss_percent * 0.4),
            uplink_rssi_2_dbm=-125,
            uplink_link_quality=round(100 - self.loss_percent),
            uplink_snr_db=round(10 - self.loss_percent * 0.08),
            active_antenna=1,
            rf_mode=2,
            uplink_tx_power=1,
            downlink_rssi_dbm=round(-50 - self.loss_percent * 0.4),
            downlink_link_quality=round(100 - self.loss_percent),
            downlink_snr_db=round(8 - self.loss_percent * 0.08),
        )
        self._publish_bytes(self.pub_host_rx, make_link_statistics_frame(stats))

    def _publish_stats(self) -> None:
        msg = String()
        msg.data = (
            f'link={"up" if self.link_enabled else "cut"} '
            f'rate={self.rate_hz:.0f}Hz latency={self.latency*1000:.1f}ms '
            f'configured_loss={self.loss_percent:.1f}% queued={len(self.queue)}'
        )
        self.pub_stats.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimElrsLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
