from __future__ import annotations

import socket
import sys
import threading
from types import SimpleNamespace

import pytest

from aerial_kit.comms import (
    CrsfLink,
    CrsfParser,
    MemoryTransport,
    SerialTransport,
    TcpTransport,
    TransportClosed,
    TransportTimeout,
    UdpTransport,
    loopback_pair,
)


def test_public_compatibility_imports() -> None:
    from aerial_kit.comms.crsf import CrsfFrame, CrsfParser as NestedParser
    from aerial_kit.comms.crsf.codec import pack_channels

    assert CrsfParser is NestedParser
    assert CrsfFrame(0xC8, 0x16, bytes(22)).encode()
    assert len(pack_channels((992,) * 16)) == 22


def test_memory_transport_read_write_and_close() -> None:
    transport = MemoryTransport()
    assert transport.write(b"abc") == 3
    assert transport.drain_tx() == b"abc"
    transport.feed(b"123456")
    assert transport.available == 6
    assert transport.read(2) == b"12"
    assert transport.poll() == b"3456"
    transport.close()
    with pytest.raises(TransportClosed):
        transport.read()


def test_memory_transport_timeout() -> None:
    transport = MemoryTransport(timeout=0.01)
    assert transport.read(timeout=0.005) == b""
    with pytest.raises(TransportTimeout):
        transport.read_exact(1, timeout=0.005)


def test_loopback_pair_carries_bytes_in_both_directions() -> None:
    left, right = loopback_pair()
    left.write(b"to-right")
    assert right.read() == b"to-right"
    right.write(b"to-left")
    assert left.read() == b"to-left"


def test_link_reader_thread_dispatches_bytes() -> None:
    left, right = loopback_pair(timeout=0.005)
    received = bytearray()
    ready = threading.Event()

    def callback(data: bytes) -> None:
        received.extend(data)
        ready.set()

    left.start_reader(callback)
    try:
        right.write(b"threaded")
        assert ready.wait(1.0)
    finally:
        left.close()
    assert received == b"threaded"


def test_crsf_link_uses_shared_memory_transport() -> None:
    memory = MemoryTransport()
    link = CrsfLink(memory)
    link.send_channels((992,) * 16)
    frames = CrsfParser().feed(memory.drain_tx())
    assert len(frames) == 1
    assert frames[0].frame_type == 0x16


class _FakePort:
    instances: list[_FakePort] = []

    def __init__(self, **kwargs: object) -> None:
        self.kwargs = kwargs
        self.in_waiting = 0
        self.timeout = float(kwargs.get("timeout", 0))
        self.is_open = True
        self.write_calls = 0
        type(self).instances.append(self)

    def write(self, data: bytes) -> int:
        self.write_calls += 1
        if len(type(self).instances) == 1 and self.write_calls == 1:
            raise OSError("unplugged")
        return len(data)

    def read(self, size: int) -> bytes:
        return b""

    def close(self) -> None:
        self.is_open = False


def test_serial_transport_is_lazy_and_reconnects(monkeypatch: pytest.MonkeyPatch) -> None:
    _FakePort.instances.clear()
    fake = SimpleNamespace(
        Serial=_FakePort,
        EIGHTBITS=8,
        PARITY_NONE="N",
        STOPBITS_ONE=1,
    )
    monkeypatch.setitem(sys.modules, "serial", fake)
    transport = SerialTransport("/dev/fake", reconnect_delay=0)
    assert transport.write(b"x") == 1
    assert len(_FakePort.instances) == 2
    assert not _FakePort.instances[0].is_open
    transport.close()


def test_serial_transport_requires_optional_dependency(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setitem(sys.modules, "serial", None)
    with pytest.raises(ImportError, match=r"aerial-kit\[radio\]"):
        SerialTransport("/dev/missing")


def test_tcp_transport_round_trip() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    host, port = server.getsockname()

    def echo() -> None:
        connection, _ = server.accept()
        with connection:
            while data := connection.recv(4096):
                connection.sendall(data)
        server.close()

    thread = threading.Thread(target=echo, daemon=True)
    thread.start()
    transport = TcpTransport.connect(host, port, timeout=1.0)
    try:
        transport.write(b"tcp")
        assert transport.read(3) == b"tcp"
    finally:
        transport.close()
    thread.join(1.0)


def test_udp_transport_round_trip() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    host, port = server.getsockname()

    def echo() -> None:
        data, peer = server.recvfrom(4096)
        server.sendto(data, peer)

    thread = threading.Thread(target=echo, daemon=True)
    thread.start()
    transport = UdpTransport.connect(
        host, port, local_addr=("127.0.0.1", 0), timeout=1.0
    )
    try:
        transport.write(b"udp")
        assert transport.read(3) == b"udp"
    finally:
        transport.close()
        server.close()
    thread.join(1.0)


def test_udp_transport_buffers_one_datagram_for_small_reads() -> None:
    receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver_socket.bind(("127.0.0.1", 0))
    receiver = UdpTransport(receiver_socket, timeout=1.0)
    port = receiver.socket.getsockname()[1]
    sender = UdpTransport.connect(
        "127.0.0.1", port, local_addr=("127.0.0.1", 0), timeout=1.0
    )
    try:
        sender.write(b"abcdef")
        assert receiver.read(1, timeout=1.0) == b"a"
        assert receiver.available == 5
        assert receiver.read(5, timeout=0.0) == b"bcdef"
        assert receiver.available == 0
    finally:
        receiver.close()
        sender.close()
