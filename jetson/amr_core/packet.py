"""Binary frame and payload codecs for the Jetson–STM32 link."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable

from protocol.protocol_constants import (
    MAX_PAYLOAD_SIZE,
    PROTOCOL_VERSION,
    SOF,
    MessageId,
    SafetyFlag,
    SystemState,
)

from .crc16 import crc16_ccitt_false


class PacketError(ValueError):
    """Raised when a received frame is invalid."""


@dataclass(frozen=True)
class Packet:
    message_id: int
    sequence: int
    payload: bytes = b""
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class DriveCommand:
    command_id: int
    linear_velocity_mm_s: int
    angular_velocity_mrad_s: int
    speed_limit_mm_s: int
    control_flags: int


@dataclass(frozen=True)
class WheelCommand:
    command_id: int
    left_target_rpm: int
    right_target_rpm: int
    control_flags: int
    emergency: int = 0


@dataclass(frozen=True)
class RobotStatus:
    system_state: SystemState
    safety_flags: SafetyFlag
    left_velocity_mm_s: int
    right_velocity_mm_s: int
    battery_voltage_mv: int
    motor_error: int
    last_command_id: int
    rx_error_count: int
    uptime_ms: int
    ultrasonic_front_mm: int = 0xFFFF
    sharp_left_mm: int = 0xFFFF
    sharp_right_mm: int = 0xFFFF
    push_switch_pressed: int = 0
    requested_base_rpm: int = 0
    left_velocity_rpm: int = 0
    right_velocity_rpm: int = 0


_FRAME_HEADER = struct.Struct("<2sBBBB")
_DRIVE_COMMAND = struct.Struct("<HhhHBB")
_WHEEL_COMMAND = struct.Struct("<HhhBB")
_ROBOT_STATUS = struct.Struct("<BHhhHHHHIHHHBhhh")


def encode_packet(packet: Packet) -> bytes:
    if not 0 <= packet.message_id <= 0xFF:
        raise PacketError("message_id must fit in uint8")
    if not 0 <= packet.sequence <= 0xFF:
        raise PacketError("sequence must fit in uint8")
    if len(packet.payload) > MAX_PAYLOAD_SIZE:
        raise PacketError("payload exceeds maximum size")

    body = bytes(
        (packet.version, packet.message_id, packet.sequence, len(packet.payload))
    ) + packet.payload
    crc = crc16_ccitt_false(body)
    return SOF + body + struct.pack("<H", crc)


def decode_packet(frame: bytes) -> Packet:
    minimum_size = _FRAME_HEADER.size + 2
    if len(frame) < minimum_size:
        raise PacketError("frame is too short")

    sof, version, message_id, sequence, payload_length = _FRAME_HEADER.unpack_from(frame)
    if sof != SOF:
        raise PacketError("invalid start of frame")
    if version != PROTOCOL_VERSION:
        raise PacketError(
            f"protocol version mismatch: got {version}, expected {PROTOCOL_VERSION}"
        )
    if payload_length > MAX_PAYLOAD_SIZE:
        raise PacketError("payload exceeds maximum size")

    expected_size = _FRAME_HEADER.size + payload_length + 2
    if len(frame) != expected_size:
        raise PacketError("frame length does not match payload length")

    body = frame[2:-2]
    received_crc = struct.unpack_from("<H", frame, len(frame) - 2)[0]
    expected_crc = crc16_ccitt_false(body)
    if received_crc != expected_crc:
        raise PacketError("CRC mismatch")

    payload = frame[_FRAME_HEADER.size : -2]
    return Packet(message_id, sequence, payload, version)


def extract_packets(buffer: bytearray) -> tuple[list[Packet], bytearray]:
    """Extract all valid packets and return remaining incomplete bytes.

    Corrupted bytes are skipped until the next SOF. A CRC-invalid frame loses
    only its first SOF byte so that a following valid frame can be recovered.
    """

    packets: list[Packet] = []
    work = bytearray(buffer)
    minimum_size = _FRAME_HEADER.size + 2

    while True:
        start = work.find(SOF)
        if start < 0:
            return packets, work[-1:] if work.endswith(SOF[:1]) else bytearray()
        if start:
            del work[:start]
        if len(work) < minimum_size:
            return packets, work

        payload_length = work[5]
        if payload_length > MAX_PAYLOAD_SIZE:
            del work[0]
            continue
        frame_size = _FRAME_HEADER.size + payload_length + 2
        if len(work) < frame_size:
            return packets, work

        candidate = bytes(work[:frame_size])
        try:
            packets.append(decode_packet(candidate))
            del work[:frame_size]
        except PacketError:
            del work[0]


def encode_drive_command(command: DriveCommand, sequence: int) -> bytes:
    if not -32768 <= command.linear_velocity_mm_s <= 32767:
        raise PacketError("linear velocity must fit in int16")
    if not -32768 <= command.angular_velocity_mrad_s <= 32767:
        raise PacketError("angular velocity must fit in int16")
    if not 0 <= command.speed_limit_mm_s <= 65535:
        raise PacketError("speed limit must fit in uint16")

    payload = _DRIVE_COMMAND.pack(
        command.command_id,
        command.linear_velocity_mm_s,
        command.angular_velocity_mrad_s,
        command.speed_limit_mm_s,
        command.control_flags,
        0,
    )
    return encode_packet(Packet(MessageId.DRIVE_COMMAND, sequence, payload))


def decode_drive_command(packet: Packet) -> DriveCommand:
    if packet.message_id != MessageId.DRIVE_COMMAND:
        raise PacketError("not a DRIVE_COMMAND packet")
    if len(packet.payload) != _DRIVE_COMMAND.size:
        raise PacketError("invalid DRIVE_COMMAND payload size")

    command_id, linear, angular, speed_limit, flags, _reserved = (
        _DRIVE_COMMAND.unpack(packet.payload)
    )
    return DriveCommand(command_id, linear, angular, speed_limit, flags)


def encode_wheel_command(command: WheelCommand, sequence: int) -> bytes:
    if not -32768 <= command.left_target_rpm <= 32767:
        raise PacketError("left target RPM must fit in int16")
    if not -32768 <= command.right_target_rpm <= 32767:
        raise PacketError("right target RPM must fit in int16")
    if command.emergency not in (0, 1):
        raise PacketError("emergency must be 0 or 1")
    payload = _WHEEL_COMMAND.pack(
        command.command_id,
        command.left_target_rpm,
        command.right_target_rpm,
        command.control_flags,
        command.emergency,
    )
    return encode_packet(Packet(MessageId.WHEEL_COMMAND, sequence, payload))


def decode_wheel_command(packet: Packet) -> WheelCommand:
    if packet.message_id != MessageId.WHEEL_COMMAND:
        raise PacketError("not a WHEEL_COMMAND packet")
    if len(packet.payload) != _WHEEL_COMMAND.size:
        raise PacketError("invalid WHEEL_COMMAND payload size")
    values = _WHEEL_COMMAND.unpack(packet.payload)
    return WheelCommand(*values)


def encode_robot_status(status: RobotStatus, sequence: int) -> bytes:
    payload = _ROBOT_STATUS.pack(
        int(status.system_state),
        int(status.safety_flags),
        status.left_velocity_mm_s,
        status.right_velocity_mm_s,
        status.battery_voltage_mv,
        status.motor_error,
        status.last_command_id,
        status.rx_error_count,
        status.uptime_ms,
        status.ultrasonic_front_mm,
        status.sharp_left_mm,
        status.sharp_right_mm,
        status.push_switch_pressed,
        status.requested_base_rpm,
        status.left_velocity_rpm,
        status.right_velocity_rpm,
    )
    return encode_packet(Packet(MessageId.ROBOT_STATUS, sequence, payload))


def decode_robot_status(packet: Packet) -> RobotStatus:
    if packet.message_id != MessageId.ROBOT_STATUS:
        raise PacketError("not a ROBOT_STATUS packet")
    if len(packet.payload) != _ROBOT_STATUS.size:
        raise PacketError("invalid ROBOT_STATUS payload size")

    values: Iterable[int] = _ROBOT_STATUS.unpack(packet.payload)
    (
        state,
        flags,
        left_velocity,
        right_velocity,
        battery_voltage,
        motor_error,
        last_command_id,
        rx_error_count,
        uptime_ms,
        ultrasonic_front_mm,
        sharp_left_mm,
        sharp_right_mm,
        push_switch_pressed,
        requested_base_rpm,
        left_velocity_rpm,
        right_velocity_rpm,
    ) = values
    return RobotStatus(
        SystemState(state),
        SafetyFlag(flags),
        left_velocity,
        right_velocity,
        battery_voltage,
        motor_error,
        last_command_id,
        rx_error_count,
        uptime_ms,
        ultrasonic_front_mm,
        sharp_left_mm,
        sharp_right_mm,
        push_switch_pressed,
        requested_base_rpm,
        left_velocity_rpm,
        right_velocity_rpm,
    )
