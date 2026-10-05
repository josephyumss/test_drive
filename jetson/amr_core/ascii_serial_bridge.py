"""ASCII UART bridge for the electronics team's STM32 firmware."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable

from .serial_bridge import BridgeDiagnostics
from .transport import ByteTransport


@dataclass(frozen=True)
class AsciiMcuStatus:
    base_rpm: int
    sharp_adc: int
    sharp_distance_cm: int
    left_target_rpm: int
    right_target_rpm: int
    left_rpm: int
    right_rpm: int
    left_pwm: int
    right_pwm: int
    emergency: bool


def encode_ascii_command(left_rpm: int, right_rpm: int, emergency: bool) -> bytes:
    """Encode `$CMD,<LEFT>,<RIGHT>,<EMERGENCY>\r\n`."""
    return f"$CMD,{int(left_rpm)},{int(right_rpm)},{int(bool(emergency))}\r\n".encode(
        "ascii"
    )


def decode_ascii_status(line: str) -> AsciiMcuStatus:
    """Decode the exact STATUS format emitted by the current main.c."""
    fields = line.strip().split(",")
    if len(fields) != 11 or fields[0] != "$STATUS":
        raise ValueError("invalid STM32 STATUS line")
    try:
        values = [int(value) for value in fields[1:]]
    except ValueError as exc:
        raise ValueError("STATUS fields must be integers") from exc
    if values[1] < 0 or values[7] < 0 or values[8] < 0:
        raise ValueError("ADC and PWM fields cannot be negative")
    if values[9] not in (0, 1):
        raise ValueError("ESTOP must be 0 or 1")
    return AsciiMcuStatus(*values[:9], bool(values[9]))


class AsciiSerialBridge:
    """Non-blocking line-based STM32 command/status bridge."""

    def __init__(
        self,
        transport: ByteTransport,
        *,
        status_timeout_s: float = 0.35,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if status_timeout_s <= 0:
            raise ValueError("status_timeout_s must be positive")
        self.transport = transport
        self.status_timeout_s = status_timeout_s
        self.clock = clock
        self._rx_buffer = bytearray()
        self._packets_received = 0
        self._ignored_packets = 0
        self._last_status_time: float | None = None
        self._latest_status: AsciiMcuStatus | None = None

    @property
    def latest_status(self) -> AsciiMcuStatus | None:
        return self._latest_status

    def send_wheel_command(
        self, left_rpm: int, right_rpm: int, emergency: bool = False
    ) -> None:
        self.transport.write(encode_ascii_command(left_rpm, right_rpm, emergency))

    def poll(self, *, now_s: float | None = None) -> list[AsciiMcuStatus]:
        now = self.clock() if now_s is None else now_s
        incoming = self.transport.read()
        if incoming:
            self._rx_buffer.extend(incoming)

        statuses: list[AsciiMcuStatus] = []
        while b"\n" in self._rx_buffer:
            raw_line, _, remainder = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remainder)
            try:
                line = raw_line.rstrip(b"\r").decode("ascii")
                status = decode_ascii_status(line)
            except (UnicodeDecodeError, ValueError):
                self._ignored_packets += 1
                continue
            self._packets_received += 1
            self._latest_status = status
            self._last_status_time = now
            statuses.append(status)
        return statuses

    def diagnostics(self, *, now_s: float | None = None) -> BridgeDiagnostics:
        now = self.clock() if now_s is None else now_s
        age = (
            None
            if self._last_status_time is None
            else max(0.0, now - self._last_status_time)
        )
        return BridgeDiagnostics(
            packets_received=self._packets_received,
            ignored_packets=self._ignored_packets,
            last_status_age_s=age,
            status_timed_out=age is None or age > self.status_timeout_s,
        )

    def close(self) -> None:
        self.transport.close()
