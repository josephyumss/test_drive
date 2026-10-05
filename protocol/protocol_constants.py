"""Jetson/PC side protocol constants.

Keep numeric values synchronized with protocol_constants.h.
"""

from enum import IntEnum, IntFlag


PROTOCOL_VERSION = 2
SOF = b"\xAA\x55"
MAX_PAYLOAD_SIZE = 32


class MessageId(IntEnum):
    HEARTBEAT = 0x01
    DRIVE_COMMAND = 0x10
    HAPTIC_COMMAND = 0x11
    SYSTEM_COMMAND = 0x12
    WHEEL_COMMAND = 0x13
    ROBOT_STATUS = 0x80
    SAFETY_EVENT = 0x81
    DIAGNOSTIC = 0x82


class SystemState(IntEnum):
    INIT = 0
    READY = 1
    RUN = 2
    SLOW = 3
    CONTROLLED_STOP = 4
    EMERGENCY_STOP = 5
    FAULT = 6


class SafetyFlag(IntFlag):
    NONE = 0
    ESTOP_ACTIVE = 1 << 0
    CLIFF_LEFT = 1 << 1
    CLIFF_RIGHT = 1 << 2
    TOF_INVALID = 1 << 3
    COMM_TIMEOUT = 1 << 4
    MOTOR_FAULT = 1 << 5
    OVERCURRENT = 1 << 6
    UNDERVOLTAGE = 1 << 7
    IMU_FAULT = 1 << 8
    ENCODER_FAULT = 1 << 9
    USER_RELEASED = 1 << 10
    PROTOCOL_MISMATCH = 1 << 11


class HapticPattern(IntEnum):
    OFF = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    SLOW = 4
    STOP = 5
    EMERGENCY = 6
    FAULT = 7


class DriveControlFlag(IntFlag):
    NONE = 0
    DRIVE_ENABLE = 1 << 0
    CONTROLLED_STOP = 1 << 1
    NAVIGATION_ACTIVE = 1 << 2
    MANUAL_MODE = 1 << 3
    RESET_REQUEST = 1 << 4
    SLOW_MODE = 1 << 5
