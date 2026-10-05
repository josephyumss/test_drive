"""Camera/LiDAR/encoder guidance calculations independent of ROS."""

from __future__ import annotations

from dataclasses import dataclass

from .velocity import VelocityCommand


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass(frozen=True)
class GuidanceConfig:
    cruise_speed_mps: float = 0.30
    maximum_angular_rad_s: float = 0.65
    camera_gain: float = 0.55
    lidar_center_gain: float = 0.75
    lidar_heading_gain: float = 0.60
    encoder_gain: float = 0.50
    camera_weight: float = 0.45
    lidar_weight: float = 0.55
    deadband: float = 0.015


@dataclass(frozen=True)
class GuidanceInputs:
    camera_lateral_error: float | None = None
    lidar_lateral_error_m: float | None = None
    lidar_heading_error_rad: float | None = None
    left_velocity_mps: float | None = None
    right_velocity_mps: float | None = None


class PathGuidanceController:
    """Outer-loop correction; the MCU remains responsible for wheel PID."""

    def __init__(self, config: GuidanceConfig | None = None) -> None:
        self.config = config or GuidanceConfig()

    def update(self, inputs: GuidanceInputs) -> VelocityCommand:
        cfg = self.config
        correction = 0.0
        weight = 0.0

        if inputs.camera_lateral_error is not None:
            correction += (
                cfg.camera_weight
                * cfg.camera_gain
                * clamp(inputs.camera_lateral_error, -1.0, 1.0)
            )
            weight += cfg.camera_weight

        if inputs.lidar_lateral_error_m is not None:
            lidar = cfg.lidar_center_gain * inputs.lidar_lateral_error_m
            if inputs.lidar_heading_error_rad is not None:
                lidar += cfg.lidar_heading_gain * inputs.lidar_heading_error_rad
            correction += cfg.lidar_weight * lidar
            weight += cfg.lidar_weight

        if weight > 0:
            correction /= weight

        # Positive (right-left) mismatch means the vehicle naturally yaws left;
        # request the opposite angular velocity as a slow outer-loop trim.
        if (
            inputs.left_velocity_mps is not None
            and inputs.right_velocity_mps is not None
        ):
            correction -= cfg.encoder_gain * (
                inputs.right_velocity_mps - inputs.left_velocity_mps
            )

        if abs(correction) < cfg.deadband:
            correction = 0.0
        angular = clamp(
            correction,
            -cfg.maximum_angular_rad_s,
            cfg.maximum_angular_rad_s,
        )
        return VelocityCommand(cfg.cruise_speed_mps, angular)
