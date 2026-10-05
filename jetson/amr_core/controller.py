"""Integrated perception-to-safe-drive control pipeline."""

from __future__ import annotations

from dataclasses import dataclass

from protocol.protocol_constants import DriveControlFlag, SystemState

from .obstacle import ObstacleAssessment, ObstacleEvaluator
from .packet import DriveCommand
from .safety import SafetyInputs, SafetyOutput, SafetyStateMachine
from .velocity import VelocityCommand, VelocityLimiter


@dataclass(frozen=True)
class ControllerInputs:
    requested_velocity: VelocityCommand = VelocityCommand()
    obstacle_distance_m: float = 10.0
    obstacle_closing_speed_mps: float = 0.0
    user_speed_limit_mps: float | None = None
    initialization_complete: bool = True
    drive_enable: bool = False
    reset_requested: bool = False
    estop_active: bool = False
    cliff_left: bool = False
    cliff_right: bool = False
    communication_timeout: bool = False
    user_released: bool = False
    motor_fault: bool = False
    critical_sensor_fault: bool = False
    degraded_sensor: bool = False
    low_battery_warning: bool = False
    controlled_stop_requested: bool = False


@dataclass(frozen=True)
class ControllerOutput:
    safety: SafetyOutput
    obstacle: ObstacleAssessment
    safe_velocity: VelocityCommand
    drive_command: DriveCommand


class IntegratedController:
    """Connect obstacle evaluation, safety state, and velocity limiting."""

    def __init__(
        self,
        *,
        obstacle_evaluator: ObstacleEvaluator | None = None,
        state_machine: SafetyStateMachine | None = None,
        velocity_limiter: VelocityLimiter | None = None,
    ) -> None:
        self.obstacle_evaluator = obstacle_evaluator or ObstacleEvaluator()
        self.state_machine = state_machine or SafetyStateMachine()
        self.velocity_limiter = velocity_limiter or VelocityLimiter()
        self._command_id = 0

    def update(self, inputs: ControllerInputs) -> ControllerOutput:
        obstacle = self.obstacle_evaluator.evaluate(
            inputs.obstacle_distance_m,
            inputs.obstacle_closing_speed_mps,
        )
        safety = self.state_machine.update(
            SafetyInputs(
                initialization_complete=inputs.initialization_complete,
                drive_enable=inputs.drive_enable,
                reset_requested=inputs.reset_requested,
                obstacle_decision=obstacle.decision,
                estop_active=inputs.estop_active,
                cliff_left=inputs.cliff_left,
                cliff_right=inputs.cliff_right,
                communication_timeout=inputs.communication_timeout,
                user_released=inputs.user_released,
                motor_fault=inputs.motor_fault,
                critical_sensor_fault=inputs.critical_sensor_fault,
                degraded_sensor=inputs.degraded_sensor,
                low_battery_warning=inputs.low_battery_warning,
                controlled_stop_requested=inputs.controlled_stop_requested,
            )
        )
        safe_velocity = self.velocity_limiter.limit(
            inputs.requested_velocity,
            safety.state,
            inputs.user_speed_limit_mps,
        )
        self._command_id = (self._command_id + 1) & 0xFFFF
        flags = self._control_flags(inputs, safety.state)
        maximum_speed = (
            self.velocity_limiter.limits.slow_linear_mps
            if safety.state == SystemState.SLOW
            else self.velocity_limiter.limits.maximum_linear_mps
        )
        if inputs.user_speed_limit_mps is not None:
            maximum_speed = min(maximum_speed, max(0.0, inputs.user_speed_limit_mps))
        if safety.state not in (SystemState.RUN, SystemState.SLOW):
            maximum_speed = 0.0

        drive_command = DriveCommand(
            command_id=self._command_id,
            linear_velocity_mm_s=round(safe_velocity.linear_mps * 1000),
            angular_velocity_mrad_s=round(safe_velocity.angular_rad_s * 1000),
            speed_limit_mm_s=round(maximum_speed * 1000),
            control_flags=int(flags),
        )
        return ControllerOutput(safety, obstacle, safe_velocity, drive_command)

    @staticmethod
    def _control_flags(
        inputs: ControllerInputs, state: SystemState
    ) -> DriveControlFlag:
        flags = DriveControlFlag.NONE
        if state in (SystemState.RUN, SystemState.SLOW):
            flags |= DriveControlFlag.DRIVE_ENABLE
        if state == SystemState.SLOW:
            flags |= DriveControlFlag.SLOW_MODE
        if state == SystemState.CONTROLLED_STOP:
            flags |= DriveControlFlag.CONTROLLED_STOP
        if inputs.reset_requested and state == SystemState.READY:
            flags |= DriveControlFlag.RESET_REQUEST
        return flags

