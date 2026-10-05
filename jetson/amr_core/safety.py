"""Fail-safe system state machine independent of ROS2 and hardware drivers."""

from __future__ import annotations

from dataclasses import dataclass

from protocol.protocol_constants import SafetyFlag, SystemState

from .obstacle import ObstacleDecision


@dataclass(frozen=True)
class SafetyInputs:
    initialization_complete: bool = False
    drive_enable: bool = False
    reset_requested: bool = False
    obstacle_decision: ObstacleDecision = ObstacleDecision.RUN
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
class SafetyOutput:
    state: SystemState
    flags: SafetyFlag
    reason: str
    restart_latched: bool


class SafetyStateMachine:
    """State machine with manual restart after stop, emergency, or fault.

    Calling ``update`` never causes a direct STOP/FAULT -> RUN transition.
    A valid reset first returns the machine to READY; drive_enable must be
    applied in a later update.
    """

    def __init__(self) -> None:
        self._state = SystemState.INIT
        self._restart_latched = False

    @property
    def state(self) -> SystemState:
        return self._state

    @property
    def restart_latched(self) -> bool:
        return self._restart_latched

    def update(self, inputs: SafetyInputs) -> SafetyOutput:
        flags = self._flags(inputs)

        # Physical emergency and critical failures have highest priority.
        if inputs.motor_fault or inputs.critical_sensor_fault:
            return self._set(SystemState.FAULT, flags, "critical system fault", latch=True)

        if inputs.estop_active:
            return self._set(
                SystemState.EMERGENCY_STOP, flags, "physical E-Stop active", latch=True
            )

        if inputs.cliff_left or inputs.cliff_right:
            return self._set(
                SystemState.EMERGENCY_STOP, flags, "cliff hazard detected", latch=True
            )

        if inputs.communication_timeout:
            return self._set(
                SystemState.EMERGENCY_STOP, flags, "command communication timeout", latch=True
            )

        if inputs.user_released:
            return self._set(
                SystemState.EMERGENCY_STOP, flags, "guide handle released", latch=True
            )

        if not inputs.initialization_complete:
            self._restart_latched = False
            return self._set(SystemState.INIT, flags, "initialization in progress")

        # A reset is accepted only when all higher-priority hazards are clear.
        if self._restart_latched:
            if inputs.reset_requested:
                self._restart_latched = False
                return self._set(SystemState.READY, flags, "manual reset accepted")
            return self._set(self._state, flags, "manual reset required")

        if inputs.controlled_stop_requested:
            return self._set(
                SystemState.CONTROLLED_STOP,
                flags,
                "controlled stop requested",
                latch=True,
            )

        if inputs.obstacle_decision == ObstacleDecision.EMERGENCY_STOP:
            return self._set(
                SystemState.EMERGENCY_STOP,
                flags,
                "imminent obstacle collision",
                latch=True,
            )

        if inputs.obstacle_decision == ObstacleDecision.CONTROLLED_STOP:
            return self._set(
                SystemState.CONTROLLED_STOP,
                flags,
                "obstacle within stopping zone",
                latch=True,
            )

        if not inputs.drive_enable:
            return self._set(SystemState.READY, flags, "waiting for drive enable")

        if (
            inputs.obstacle_decision == ObstacleDecision.SLOW
            or inputs.degraded_sensor
            or inputs.low_battery_warning
        ):
            return self._set(SystemState.SLOW, flags, "reduced-speed condition")

        return self._set(SystemState.RUN, flags, "all required checks passed")

    def _set(
        self,
        state: SystemState,
        flags: SafetyFlag,
        reason: str,
        *,
        latch: bool = False,
    ) -> SafetyOutput:
        self._state = state
        if latch:
            self._restart_latched = True
        return SafetyOutput(state, flags, reason, self._restart_latched)

    @staticmethod
    def _flags(inputs: SafetyInputs) -> SafetyFlag:
        flags = SafetyFlag.NONE
        if inputs.estop_active:
            flags |= SafetyFlag.ESTOP_ACTIVE
        if inputs.cliff_left:
            flags |= SafetyFlag.CLIFF_LEFT
        if inputs.cliff_right:
            flags |= SafetyFlag.CLIFF_RIGHT
        if inputs.communication_timeout:
            flags |= SafetyFlag.COMM_TIMEOUT
        if inputs.user_released:
            flags |= SafetyFlag.USER_RELEASED
        if inputs.motor_fault:
            flags |= SafetyFlag.MOTOR_FAULT
        return flags

