"""Differential-drive conversion shared by real and dummy MCU bridges."""

from __future__ import annotations

import math


def twist_to_wheel_rpm(
    linear_mps: float,
    angular_rad_s: float,
    *,
    wheel_diameter_m: float,
    wheel_base_m: float,
    maximum_rpm: int,
    left_trim_rpm: int = 0,
    right_trim_rpm: int = 0,
) -> tuple[int, int]:
    """Convert a ROS base twist to signed left/right wheel RPM targets."""
    if wheel_diameter_m <= 0 or wheel_base_m <= 0 or maximum_rpm <= 0:
        raise ValueError("wheel geometry and maximum_rpm must be positive")
    half_turn_mps = angular_rad_s * wheel_base_m * 0.5
    circumference_m = math.pi * wheel_diameter_m
    left = round((linear_mps - half_turn_mps) * 60.0 / circumference_m)
    right = round((linear_mps + half_turn_mps) * 60.0 / circumference_m)
    if linear_mps > 0:
        left += left_trim_rpm
        right += right_trim_rpm
    return (
        max(-maximum_rpm, min(maximum_rpm, left)),
        max(-maximum_rpm, min(maximum_rpm, right)),
    )
