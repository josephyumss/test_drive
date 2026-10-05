"""Byte transport abstraction for memory tests and real UART."""

from __future__ import annotations

from collections import deque
from threading import Lock
from typing import Protocol, runtime_checkable


@runtime_checkable
class ByteTransport(Protocol):
    def write(self, data: bytes) -> int: ...

    def read(self, max_bytes: int = 4096) -> bytes: ...

    def close(self) -> None: ...


class MemoryTransport:
    """One endpoint of an in-memory full-duplex byte link."""

    def __init__(self) -> None:
        self._incoming: deque[int] = deque()
        self._lock = Lock()
        self._peer: MemoryTransport | None = None
        self._closed = False

    def connect(self, peer: "MemoryTransport") -> None:
        if peer is self:
            raise ValueError("transport cannot connect to itself")
        self._peer = peer

    def write(self, data: bytes) -> int:
        if self._closed:
            raise RuntimeError("transport is closed")
        if self._peer is None or self._peer._closed:
            raise RuntimeError("transport peer is unavailable")
        with self._peer._lock:
            self._peer._incoming.extend(data)
        return len(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        if max_bytes <= 0:
            raise ValueError("max_bytes must be positive")
        with self._lock:
            count = min(max_bytes, len(self._incoming))
            return bytes(self._incoming.popleft() for _ in range(count))

    def close(self) -> None:
        self._closed = True


def memory_transport_pair() -> tuple[MemoryTransport, MemoryTransport]:
    first = MemoryTransport()
    second = MemoryTransport()
    first.connect(second)
    second.connect(first)
    return first, second


class SerialTransport:
    """Thin optional pyserial adapter.

    Importing this module does not require pyserial. The dependency is checked
    only when a real serial port is opened.
    """

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 115200,
        timeout_s: float = 0.0,
    ) -> None:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required for SerialTransport: pip install pyserial"
            ) from exc
        self._serial = serial.Serial(port, baudrate=baudrate, timeout=timeout_s)

    def write(self, data: bytes) -> int:
        return int(self._serial.write(data))

    def read(self, max_bytes: int = 4096) -> bytes:
        available = int(self._serial.in_waiting)
        if available <= 0:
            return b""
        return bytes(self._serial.read(min(max_bytes, available)))

    def close(self) -> None:
        self._serial.close()
