"""Jetson-side command/status bridge over a byte transport."""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
from typing import Callable

from protocol.protocol_constants import MessageId, PROTOCOL_VERSION

from .packet import (
    DriveCommand,
    Packet,
    RobotStatus,
    WheelCommand,
    decode_robot_status,
    encode_drive_command,
    encode_wheel_command,
    encode_packet,
    extract_packets,
)
from .transport import ByteTransport


@dataclass(frozen=True)
class BridgeDiagnostics:
    packets_received: int
    ignored_packets: int
    last_status_age_s: float | None
    status_timed_out: bool


class SerialBridge:
    def __init__(
        self,
        transport: ByteTransport,
        *,
        status_timeout_s: float = 0.3,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if status_timeout_s <= 0:
            raise ValueError("status_timeout_s must be positive")
        self.transport = transport
        self.status_timeout_s = status_timeout_s
        self.clock = clock
        self._rx_buffer = bytearray()
        self._sequence = 0
        self._packets_received = 0
        self._ignored_packets = 0
        self._last_status_time: float | None = None
        self._latest_status: RobotStatus | None = None

    @property
    def latest_status(self) -> RobotStatus | None:
        return self._latest_status

    def send_drive_command(self, command: DriveCommand) -> int:
        sequence = self._next_sequence()
        self.transport.write(encode_drive_command(command, sequence))
        return sequence

    def send_wheel_command(self, command: WheelCommand) -> int:
        sequence = self._next_sequence()
        self.transport.write(encode_wheel_command(command, sequence))
        return sequence

    def send_heartbeat(self, jetson_state: int, uptime_ms: int) -> int:
        if not 0 <= jetson_state <= 0xFF:
            raise ValueError("jetson_state must fit in uint8")
        if not 0 <= uptime_ms <= 0xFFFFFFFF:
            raise ValueError("uptime_ms must fit in uint32")
        sequence = self._next_sequence()
        payload = struct.pack("<BI", jetson_state, uptime_ms)
        self.transport.write(
            encode_packet(Packet(MessageId.HEARTBEAT, sequence, payload))
        )
        return sequence

    def poll(self, *, now_s: float | None = None) -> list[RobotStatus]:
        now = self.clock() if now_s is None else now_s
        incoming = self.transport.read()
        if incoming:
            self._rx_buffer.extend(incoming)
        packets, self._rx_buffer = extract_packets(self._rx_buffer)
        statuses: list[RobotStatus] = []
        for packet in packets:
            self._packets_received += 1
            if packet.version != PROTOCOL_VERSION:
                self._ignored_packets += 1
                continue
            if packet.message_id != MessageId.ROBOT_STATUS:
                self._ignored_packets += 1
                continue
            status = decode_robot_status(packet)
            self._latest_status = status
            self._last_status_time = now
            statuses.append(status)
        return statuses

    def diagnostics(self, *, now_s: float | None = None) -> BridgeDiagnostics:
        now = self.clock() if now_s is None else now_s
        age = None if self._last_status_time is None else max(0.0, now - self._last_status_time)
        return BridgeDiagnostics(
            packets_received=self._packets_received,
            ignored_packets=self._ignored_packets,
            last_status_age_s=age,
            status_timed_out=age is None or age > self.status_timeout_s,
        )

    def close(self) -> None:
        self.transport.close()

    def _next_sequence(self) -> int:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFF
        return sequence
