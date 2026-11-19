"""ROS nodes for AKP-over-CRSF through the simulated ELRS link."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from aerial_kit.comms.akp import AkClient
from aerial_kit.comms.crsf import CrsfAkpTunnel, CrsfFrame, CrsfLink, CrsfParser
from aerial_kit.comms.transport import Link

from hw_bridge.akp_sim import AkpSimDevice


class RosByteTransport(Link):
    def __init__(self, node: Node, tx_topic: str, rx_topic: str) -> None:
        super().__init__(timeout=0.05)
        self._condition = threading.Condition()
        self._rx = bytearray()
        self._pub = node.create_publisher(UInt8MultiArray, tx_topic, 20)
        self._sub = node.create_subscription(
            UInt8MultiArray, rx_topic, self._on_rx, 20
        )

    def wait_ready(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._pub.get_subscription_count() > 0 and \
                    self._sub.get_publisher_count() > 0:
                return True
            time.sleep(0.05)
        return False

    def _on_rx(self, msg: UInt8MultiArray) -> None:
        with self._condition:
            self._rx.extend(bytes(msg.data))
            self._condition.notify_all()

    def _read(self, size: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not self._rx:
                left = deadline - time.monotonic()
                if left <= 0:
                    return b""
                self._condition.wait(left)
            count = len(self._rx) if size < 0 else min(size, len(self._rx))
            data = bytes(self._rx[:count])
            del self._rx[:count]
            return data

    def _write(self, data: bytes) -> int:
        msg = UInt8MultiArray()
        msg.data = list(data)
        self._pub.publish(msg)
        return len(data)

    def _close(self) -> None:
        pass


class CrsfAkpSimDeviceNode(Node):
    def __init__(self) -> None:
        super().__init__('crsf_akp_sim_device_node')
        self.declare_parameter('receiver_rx_topic', '/uav/radio/receiver_rx_bytes')
        self.declare_parameter('receiver_tx_topic', '/uav/radio/receiver_tx_bytes')
        self.parser = CrsfParser()
        self.device = AkpSimDevice()
        self.pub_tx = self.create_publisher(
            UInt8MultiArray, str(self.get_parameter('receiver_tx_topic').value), 20
        )
        self.create_subscription(
            UInt8MultiArray,
            str(self.get_parameter('receiver_rx_topic').value),
            self._on_rx,
            20,
        )

    def _on_rx(self, msg: UInt8MultiArray) -> None:
        for frame in self.parser.feed(bytes(msg.data)):
            if frame.frame_type in (0x7A, 0x7C):
                self.get_logger().info(
                    f'AKP device received type=0x{frame.frame_type:02x}'
                )
            before_errors = self.device.errors
            replies = self.device.feed(frame)
            if self.device.errors != before_errors:
                self.get_logger().warning(
                    f'AKP device rejected a segment errors={self.device.errors}'
                )
            for reply in replies:
                self.get_logger().info(
                    f'AKP device reply type=0x{reply.frame_type:02x} '
                    f'requests={self.device.requests} errors={self.device.errors}'
                )
                out = UInt8MultiArray()
                out.data = list(reply.encode())
                self.pub_tx.publish(out)


class CrsfAkpProbeNode(Node):
    def __init__(self) -> None:
        super().__init__('crsf_akp_probe_node')
        self.declare_parameter('host_tx_topic', '/uav/radio/host_tx_bytes')
        self.declare_parameter('host_rx_topic', '/uav/radio/host_rx_bytes')
        self.declare_parameter('output_path', '')
        self.declare_parameter('timeout_sec', 4.0)
        self.declare_parameter('attempts', 8)
        self.declare_parameter('send_interval_sec', 0.025)
        self.output_path = str(self.get_parameter('output_path').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.attempts = max(1, int(self.get_parameter('attempts').value))
        transport = RosByteTransport(
            self,
            str(self.get_parameter('host_tx_topic').value),
            str(self.get_parameter('host_rx_topic').value),
        )
        self._transport = transport
        self.tunnel = CrsfAkpTunnel(
            CrsfLink(transport),
            send_interval_s=float(self.get_parameter('send_interval_sec').value),
        )
        self.client = AkClient(self.tunnel, timeout=self.timeout)
        self.result: dict = {
            'ok': False,
            'attempts': 0,
            'stage': 'starting',
            'latency_ms': {},
        }
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _attempt(self, name: str, operation):
        last = None
        for attempt in range(1, self.attempts + 1):
            self.result['attempts'] += 1
            started = time.monotonic()
            try:
                value = operation()
                self.result['latency_ms'][name] = round(
                    (time.monotonic() - started) * 1000.0, 3
                )
                self.result.setdefault('retries', {})[name] = attempt - 1
                return value
            except (OSError, RuntimeError, TimeoutError) as exc:
                last = str(exc)
                self.get_logger().warning(
                    f'AKP {name} attempt {attempt} failed: {exc}'
                )
                time.sleep(0.03 * attempt)
        raise RuntimeError(f'{name} failed after {self.attempts} attempts: {last}')

    def _run(self) -> None:
        try:
            if not self._transport.wait_ready(10.0):
                raise RuntimeError('AKP simulator topics did not connect')
            self.result['stage'] = 'hello'
            hello = self._attempt('hello', self.client.hello)
            self.result['hello'] = {
                'product': hello.product,
                'parameters': hello.parameter_count,
            }
            self.result['stage'] = 'status'
            status = self._attempt('status', self.client.status)
            self.result['status'] = {
                'flight_state': status.flight_state,
                'link_live': status.link_live,
                'gps_satellites': status.gps_satellites,
            }
            self.result['stage'] = 'parameter'
            parameter = self._attempt('parameter', lambda: self.client.get_parameter(0))
            self.result['parameter'] = {
                'name': parameter.name,
                'value': parameter.value,
                'status': parameter.status,
            }
            self.result['ok'] = True
            self.result['stage'] = 'done'
        except Exception as exc:  # noqa: BLE001 - evidence must record the failure
            self.result['error'] = str(exc)
            self.result['stage'] = 'failed'
        finally:
            self._write_result()

    def _write_result(self) -> None:
        if not self.output_path:
            return
        path = Path(self.output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.result, indent=2) + '\n')


def device_main(args=None) -> None:
    rclpy.init(args=args)
    node = CrsfAkpSimDeviceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def probe_main(args=None) -> None:
    rclpy.init(args=args)
    node = CrsfAkpProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

