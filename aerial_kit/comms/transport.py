"""Byte transports and a pollable link base.

The module has no optional imports at load time. ``SerialTransport`` imports
pyserial only when opened, and creating an object does not open a device.
"""

from __future__ import annotations

import select
import socket
import threading
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from types import TracebackType
from typing import Protocol, cast


class TransportError(OSError):
    """Base class for transport failures."""


class TransportClosed(TransportError):
    """Raised when an operation is attempted after close."""


class TransportTimeout(TransportError):
    """Raised when a bounded read receives no bytes."""


class TransportProtocol(Protocol):
    def read(self, size: int = -1, timeout: float | None = None) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


Transport = TransportProtocol


class _SerialPortProtocol(Protocol):
    is_open: bool
    in_waiting: int
    timeout: float

    def read(self, size: int) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


class Link(ABC):
    """Thread-safe byte link with single-reader polling or callback mode.

    Polling is the default. ``start_reader`` adds one background thread that
    drains the transport and hands bytes to a callback. The callback runs on
    that reader thread and must return promptly.
    """

    def __init__(self, *, timeout: float = 0.05) -> None:
        if timeout < 0:
            raise ValueError("timeout must be non-negative")
        self.timeout = float(timeout)
        self._read_lock = threading.RLock()
        self._write_lock = threading.RLock()
        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()
        self._closed = False

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def available(self) -> int:
        return 0

    def read(self, size: int = -1, timeout: float | None = None) -> bytes:
        if self._closed:
            raise TransportClosed("transport is closed")
        if size == 0:
            return b""
        with self._read_lock:
            return self._read(size, self.timeout if timeout is None else timeout)

    def read_exact(self, size: int, timeout: float | None = None) -> bytes:
        if size < 0:
            raise ValueError("read_exact size must be non-negative")
        if size == 0:
            return b""
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        output = bytearray()
        while len(output) < size:
            remaining = max(0.0, deadline - time.monotonic())
            data = self.read(size - len(output), remaining)
            if data:
                output.extend(data)
            elif time.monotonic() >= deadline:
                break
        if len(output) != size:
            raise TransportTimeout(
                f"wanted {size} bytes, received {len(output)} before timeout"
            )
        return bytes(output)

    def write(self, data: bytes) -> int:
        if self._closed:
            raise TransportClosed("transport is closed")
        with self._write_lock:
            return self._write(bytes(data))

    def poll(self, size: int = 4096, timeout: float | None = None) -> bytes:
        return self.read(size, timeout)

    def start_reader(
        self,
        callback: Callable[[bytes], None],
        *,
        chunk_size: int = 4096,
    ) -> None:
        if self._reader_thread is not None and self._reader_thread.is_alive():
            raise RuntimeError("reader is already running")
        if self._closed:
            raise TransportClosed("transport is closed")
        self._reader_stop.clear()

        def run() -> None:
            while not self._reader_stop.is_set():
                try:
                    data = self.read(chunk_size)
                except TransportTimeout:
                    continue
                except (TransportClosed, TransportError, OSError):
                    break
                if data:
                    callback(data)

        self._reader_thread = threading.Thread(
            target=run, name=f"{type(self).__name__}-reader", daemon=True
        )
        self._reader_thread.start()

    def stop_reader(self, *, timeout: float = 1.0) -> None:
        self._reader_stop.set()
        thread = self._reader_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout)
        self._reader_thread = None

    def close(self) -> None:
        if self._closed:
            return
        self.stop_reader()
        self._close()
        self._closed = True

    def __enter__(self) -> Link:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    @abstractmethod
    def _read(self, size: int, timeout: float) -> bytes:
        raise NotImplementedError

    @abstractmethod
    def _write(self, data: bytes) -> int:
        raise NotImplementedError

    @abstractmethod
    def _close(self) -> None:
        raise NotImplementedError


class MemoryTransport(Link):
    """Deterministic in-memory transport for unit tests and fake devices."""

    def __init__(self, *, timeout: float = 0.0) -> None:
        super().__init__(timeout=timeout)
        self._rx = bytearray()
        self._tx = bytearray()

    @property
    def available(self) -> int:
        return len(self._rx)

    def feed(self, data: bytes | bytearray | memoryview) -> None:
        if self.closed:
            raise TransportClosed("transport is closed")
        self._rx.extend(data)

    def drain_tx(self) -> bytes:
        if self.closed:
            raise TransportClosed("transport is closed")
        output = bytes(self._tx)
        self._tx.clear()
        return output

    def _read(self, size: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        while not self._rx and time.monotonic() < deadline:
            time.sleep(min(0.001, max(0.0, deadline - time.monotonic())))
        if not self._rx:
            return b""
        count = len(self._rx) if size < 0 else min(size, len(self._rx))
        output = bytes(self._rx[:count])
        del self._rx[:count]
        return output

    def _write(self, data: bytes) -> int:
        self._tx.extend(data)
        return len(data)

    def _close(self) -> None:
        pass


class LoopbackTransport(MemoryTransport):
    """One end of a connected pair; writes are delivered to the peer."""

    def __init__(self, *, timeout: float = 0.0) -> None:
        super().__init__(timeout=timeout)
        self._peer: LoopbackTransport | None = None

    def _write(self, data: bytes) -> int:
        if self._peer is None or self._peer.closed:
            raise TransportClosed("loopback peer is closed")
        self._peer.feed(data)
        return len(data)


def loopback_pair(*, timeout: float = 0.0) -> tuple[LoopbackTransport, LoopbackTransport]:
    left = LoopbackTransport(timeout=timeout)
    right = LoopbackTransport(timeout=timeout)
    left._peer = right
    right._peer = left
    return left, right


class SerialTransport(Link):
    """Serial transport with lazy pyserial import and reconnect support.

    The port is opened explicitly, or by the first read/write when
    ``auto_open=True``. A failed read/write closes the handle and reconnects
    once before surfacing the transport error.
    """

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 115_200,
        timeout: float = 0.05,
        reconnect: bool = True,
        reconnect_delay: float = 0.25,
        auto_open: bool = True,
    ) -> None:
        super().__init__(timeout=timeout)
        self.port_name = port
        self.baudrate = int(baudrate)
        self.reconnect = bool(reconnect)
        self.reconnect_delay = max(0.0, float(reconnect_delay))
        self._port: _SerialPortProtocol | None = None
        if auto_open:
            self.open()

    @property
    def available(self) -> int:
        port = self._port
        return int(getattr(port, "in_waiting", 0)) if port is not None else 0

    def open(self) -> None:
        if self.closed:
            raise TransportClosed("transport is closed")
        if self._port is not None and bool(getattr(self._port, "is_open", True)):
            return
        try:
            import serial  # type: ignore[import-untyped]
        except ImportError as exc:
            raise ImportError(
                "SerialTransport requires pyserial; install aerial-kit[radio]"
            ) from exc
        self._port = cast(_SerialPortProtocol, serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        ))

    def _reopen(self) -> None:
        self._close_port()
        if not self.reconnect:
            return
        if self.reconnect_delay:
            time.sleep(self.reconnect_delay)
        self.open()

    def _with_reconnect(self, operation: Callable[[], bytes | int]) -> bytes | int:
        try:
            return operation()
        except OSError as first:
            if not self.reconnect:
                raise TransportError(str(first)) from first
            try:
                self._reopen()
                return operation()
            except OSError as second:
                raise TransportError(
                    f"serial reconnect to {self.port_name!r} failed: {second}"
                ) from second

    def _read(self, size: int, timeout: float) -> bytes:
        if self._port is None:
            self.open()

        def receive() -> bytes:
            port = self._port
            assert port is not None
            port.timeout = timeout
            count = int(port.in_waiting) if size < 0 else size
            return port.read(count)

        data = self._with_reconnect(receive)
        assert isinstance(data, bytes)
        return data

    def _write(self, data: bytes) -> int:
        if self._port is None:
            self.open()

        def send() -> int:
            port = self._port
            assert port is not None
            return port.write(data)

        written = self._with_reconnect(send)
        assert isinstance(written, int)
        return written

    def _close_port(self) -> None:
        port, self._port = self._port, None
        if port is not None:
            try:
                port.close()
            except OSError:
                pass

    def _close(self) -> None:
        self._close_port()


class _SocketTransport(Link):
    def __init__(self, sock: socket.socket, *, timeout: float = 0.05) -> None:
        super().__init__(timeout=timeout)
        self.socket = sock
        self.socket.settimeout(timeout)

    @property
    def available(self) -> int:
        if self.closed:
            return 0
        readable, _, _ = select.select([self.socket], [], [], 0.0)
        return 1 if readable else 0

    def _read(self, size: int, timeout: float) -> bytes:
        self.socket.settimeout(timeout)
        try:
            return self.socket.recv(4096 if size < 0 else size)
        except socket.timeout:
            return b""
        except OSError as exc:
            raise TransportError(str(exc)) from exc

    def _write(self, data: bytes) -> int:
        try:
            return self.socket.send(data)
        except OSError as exc:
            raise TransportError(str(exc)) from exc

    def _close(self) -> None:
        self.socket.close()


class TcpTransport(_SocketTransport):
    """Connected TCP byte stream."""

    @classmethod
    def connect(
        cls,
        host: str,
        port: int,
        *,
        timeout: float = 0.05,
        connect_timeout: float = 5.0,
    ) -> TcpTransport:
        sock = socket.create_connection((host, int(port)), timeout=connect_timeout)
        sock.settimeout(timeout)
        return cls(sock, timeout=timeout)


class UdpTransport(_SocketTransport):
    """Connected UDP endpoint.

    ``local_addr`` binds a receive address before connecting. The peer is fixed
    by ``peer_addr`` so the transport cannot accidentally answer a different
    sender.
    """

    @classmethod
    def connect(
        cls,
        peer_host: str,
        peer_port: int,
        *,
        local_addr: tuple[str, int] | None = None,
        timeout: float = 0.05,
    ) -> UdpTransport:
        family = socket.AF_INET6 if ":" in peer_host else socket.AF_INET
        sock = socket.socket(family, socket.SOCK_DGRAM)
        if local_addr is not None:
            sock.bind(local_addr)
        sock.connect((peer_host, int(peer_port)))
        sock.settimeout(timeout)
        return cls(sock, timeout=timeout)
