"""State-aware velocity limiting."""

from __future__ import annotations

from dataclasses import dataclass

from protocol.protocol_constants import SystemState


@dataclass(frozen=True)
class VelocityCommand:
    linear_mps: float = 0.0
    angular_rad_s: float = 0.0


@dataclass(frozen=True)
class VelocityLimits:
    maximum_linear_mps: float = 0.5
    slow_linear_mps: float = 0.25
    maximum_angular_rad_s: float = 0.8
    slow_angular_rad_s: float = 0.4
    allow_reverse: bool = False


class VelocityLimiter:
    def __init__(self, limits: VelocityLimits | None = None) -> None:
        self.limits = limits or VelocityLimits()

    def limit(
        self,
        requested: VelocityCommand,
        state: SystemState,
        user_speed_limit_mps: float | None = None,
    ) -> VelocityCommand:
        if state not in (SystemState.RUN, SystemState.SLOW):
            return VelocityCommand()

        max_linear = (
            self.limits.slow_linear_mps
            if state == SystemState.SLOW
            else self.limits.maximum_linear_mps
        )
        max_angular = (
            self.limits.slow_angular_rad_s
            if state == SystemState.SLOW
            else self.limits.maximum_angular_rad_s
        )
        if user_speed_limit_mps is not None:
            max_linear = min(max_linear, max(0.0, user_speed_limit_mps))

        lower_linear = -max_linear if self.limits.allow_reverse else 0.0
        linear = min(max(requested.linear_mps, lower_linear), max_linear)
        angular = min(max(requested.angular_rad_s, -max_angular), max_angular)
        return VelocityCommand(linear, angular)
