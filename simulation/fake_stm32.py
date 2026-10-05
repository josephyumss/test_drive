"""Deterministic STM32 and differential-drive simulator."""

from __future__ import annotations

from dataclasses import dataclass

from protocol.protocol_constants import (
    DriveControlFlag,
    MessageId,
    SafetyFlag,
    SystemState,
)
from jetson.amr_core.packet import (
    DriveCommand,
    RobotStatus,
    WheelCommand,
    decode_drive_command,
    decode_wheel_command,
    encode_robot_status,
    extract_packets,
)
from jetson.amr_core.transport import ByteTransport


@dataclass
class SimulatedHazards:
    estop_active: bool = False
    cliff_left: bool = False
    cliff_right: bool = False
    motor_fault: bool = False


class FakeSTM32:
    """Model the safety-relevant behavior expected from STM32 firmware."""

    def __init__(
        self,
        transport: ByteTransport,
        *,
        command_timeout_s: float = 0.5,
        wheel_base_m: float = 0.5,
        wheel_diameter_m: float = 0.2,
        acceleration_mps2: float = 1.0,
        battery_voltage_mv: int = 24_000,
    ) -> None:
        if command_timeout_s <= 0:
            raise ValueError("command_timeout_s must be positive")
        if wheel_base_m <= 0:
            raise ValueError("wheel_base_m must be positive")
        if acceleration_mps2 <= 0:
            raise ValueError("acceleration_mps2 must be positive")
        self.transport = transport
        self.command_timeout_s = command_timeout_s
        self.wheel_base_m = wheel_base_m
        self.wheel_diameter_m = wheel_diameter_m
        self.acceleration_mps2 = acceleration_mps2
        self.battery_voltage_mv = battery_voltage_mv
        self.hazards = SimulatedHazards()
        self.state = SystemState.READY
        self.flags = SafetyFlag.NONE
        self.last_command_id = 0
        self.rx_error_count = 0
        self.left_velocity_mps = 0.0
        self.right_velocity_mps = 0.0
        self._target_left_mps = 0.0
        self._target_right_mps = 0.0
        self._last_drive_time: float | None = None
        self._start_time: float | None = None
        self._restart_latched = False
        self._rx_buffer = bytearray()
        self._tx_sequence = 0
        self.push_switch_pressed = True
        self.requested_base_rpm = 30

    def step(self, now_s: float, dt_s: float, *, publish_status: bool = True) -> RobotStatus:
        if dt_s < 0:
            raise ValueError("dt_s cannot be negative")
        if self._start_time is None:
            self._start_time = now_s

        self._receive_commands(now_s)
        self._apply_safety(now_s)
        self.left_velocity_mps = self._approach(
            self.left_velocity_mps, self._target_left_mps, dt_s
        )
        self.right_velocity_mps = self._approach(
            self.right_velocity_mps, self._target_right_mps, dt_s
        )

        status = self.status(now_s)
        if publish_status:
            self.transport.write(encode_robot_status(status, self._tx_sequence))
            self._tx_sequence = (self._tx_sequence + 1) & 0xFF
        return status

    def status(self, now_s: float) -> RobotStatus:
        start = self._start_time if self._start_time is not None else now_s
        return RobotStatus(
            system_state=self.state,
            safety_flags=self.flags,
            left_velocity_mm_s=round(self.left_velocity_mps * 1000),
            right_velocity_mm_s=round(self.right_velocity_mps * 1000),
            battery_voltage_mv=self.battery_voltage_mv,
            motor_error=1 if self.hazards.motor_fault else 0,
            last_command_id=self.last_command_id,
            rx_error_count=self.rx_error_count,
            uptime_ms=max(0, round((now_s - start) * 1000)),
            push_switch_pressed=self.push_switch_pressed,
            requested_base_rpm=self.requested_base_rpm,
            left_velocity_rpm=round(
                self.left_velocity_mps * 60.0 / (3.141592653589793 * self.wheel_diameter_m)
            ),
            right_velocity_rpm=round(
                self.right_velocity_mps * 60.0 / (3.141592653589793 * self.wheel_diameter_m)
            ),
        )

    def _receive_commands(self, now_s: float) -> None:
        incoming = self.transport.read()
        if incoming:
            self._rx_buffer.extend(incoming)
        packets, self._rx_buffer = extract_packets(self._rx_buffer)
        for packet in packets:
            try:
                if packet.message_id == MessageId.WHEEL_COMMAND:
                    command = decode_wheel_command(packet)
                elif packet.message_id == MessageId.DRIVE_COMMAND:
                    command = decode_drive_command(packet)
                else:
                    continue
            except ValueError:
                self.rx_error_count += 1
                continue
            self._last_drive_time = now_s
            self.last_command_id = command.command_id
            if isinstance(command, WheelCommand):
                self._apply_wheel_command(command)
            else:
                self._apply_drive_command(command)

    def _apply_wheel_command(self, command: WheelCommand) -> None:
        flags = DriveControlFlag(command.control_flags)
        if command.emergency:
            self.state = SystemState.EMERGENCY_STOP
            self._restart_latched = True
            self._stop_targets()
            return
        if flags & DriveControlFlag.RESET_REQUEST:
            if not self._physical_hazard_active():
                self._restart_latched = False
                self.state = SystemState.READY
                self.flags &= ~SafetyFlag.COMM_TIMEOUT
            self._stop_targets()
            return
        if self._restart_latched:
            self._stop_targets()
            return
        if flags & DriveControlFlag.CONTROLLED_STOP:
            self.state = SystemState.CONTROLLED_STOP
            self._restart_latched = True
            self._stop_targets()
            return
        if not flags & DriveControlFlag.DRIVE_ENABLE:
            self.state = SystemState.READY
            self._stop_targets()
            return
        metres_per_rev = 3.141592653589793 * self.wheel_diameter_m
        self._target_left_mps = command.left_target_rpm * metres_per_rev / 60.0
        self._target_right_mps = command.right_target_rpm * metres_per_rev / 60.0
        self.state = SystemState.SLOW if flags & DriveControlFlag.SLOW_MODE else SystemState.RUN

    def _apply_drive_command(self, command: DriveCommand) -> None:
        flags = DriveControlFlag(command.control_flags)
        if flags & DriveControlFlag.RESET_REQUEST:
            if not self._physical_hazard_active():
                self._restart_latched = False
                self.state = SystemState.READY
                self.flags &= ~SafetyFlag.COMM_TIMEOUT
            self._stop_targets()
            return

        if self._restart_latched:
            self._stop_targets()
            return

        if flags & DriveControlFlag.CONTROLLED_STOP:
            self.state = SystemState.CONTROLLED_STOP
            self._restart_latched = True
            self._stop_targets()
            return

        if not flags & DriveControlFlag.DRIVE_ENABLE:
            self.state = SystemState.READY
            self._stop_targets()
            return

        linear = command.linear_velocity_mm_s / 1000.0
        angular = command.angular_velocity_mrad_s / 1000.0
        speed_limit = command.speed_limit_mm_s / 1000.0
        linear = max(-speed_limit, min(speed_limit, linear))
        half_turn = angular * self.wheel_base_m / 2.0
        self._target_left_mps = linear - half_turn
        self._target_right_mps = linear + half_turn
        self.state = (
            SystemState.SLOW
            if flags & DriveControlFlag.SLOW_MODE
            else SystemState.RUN
        )

    def _apply_safety(self, now_s: float) -> None:
        self.flags = SafetyFlag.NONE
        if self.hazards.motor_fault:
            self.flags |= SafetyFlag.MOTOR_FAULT
            self.state = SystemState.FAULT
            self._restart_latched = True
        elif self.hazards.estop_active:
            self.flags |= SafetyFlag.ESTOP_ACTIVE
            self.state = SystemState.EMERGENCY_STOP
            self._restart_latched = True
        elif self.hazards.cliff_left or self.hazards.cliff_right:
            if self.hazards.cliff_left:
                self.flags |= SafetyFlag.CLIFF_LEFT
            if self.hazards.cliff_right:
                self.flags |= SafetyFlag.CLIFF_RIGHT
            self.state = SystemState.EMERGENCY_STOP
            self._restart_latched = True
        elif (
            self._last_drive_time is not None
            and now_s - self._last_drive_time > self.command_timeout_s
        ):
            self.flags |= SafetyFlag.COMM_TIMEOUT
            self.state = SystemState.EMERGENCY_STOP
            self._restart_latched = True

        if self._restart_latched:
            self._stop_targets()

    def _physical_hazard_active(self) -> bool:
        return (
            self.hazards.estop_active
            or self.hazards.cliff_left
            or self.hazards.cliff_right
            or self.hazards.motor_fault
        )

    def _stop_targets(self) -> None:
        self._target_left_mps = 0.0
        self._target_right_mps = 0.0

    def _approach(self, current: float, target: float, dt_s: float) -> float:
        maximum_change = self.acceleration_mps2 * dt_s
        difference = target - current
        if abs(difference) <= maximum_change:
            return target
        return current + maximum_change * (1 if difference > 0 else -1)
