import math

from protocol.protocol_constants import SafetyFlag, SystemState


def _optional_finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def active_flag_names(flags: int) -> list[str]:
    value = SafetyFlag(flags)
    return [flag.name for flag in SafetyFlag if flag != SafetyFlag.NONE and flag & value]


def make_monitoring_payload(
    *,
    safety=None,
    mcu=None,
    obstacle=None,
    cliff=None,
    velocity=None,
    battery=None,
    dummy_active: bool = False,
    scenario_name: str = "",
    camera_connected: bool = False,
    lidar_connected: bool = False,
) -> dict:
    state_value = int(safety.state) if safety is not None else int(SystemState.INIT)
    try:
        state_name = SystemState(state_value).name
    except ValueError:
        state_name = "FAULT"
    reason = safety.reason if safety is not None else "ROS2 상태 대기 중"
    if dummy_active:
        reason = f"[DUMMY:{scenario_name or 'active'}] {reason}"

    actual = 0.0
    left = 0.0
    right = 0.0
    if mcu is not None:
        left = float(mcu.left_velocity_mps)
        right = float(mcu.right_velocity_mps)
        actual = (left + right) / 2.0

    obstacle_detected = bool(obstacle and obstacle.detected and obstacle.valid)
    flags = int(safety.safety_flags) if safety is not None else 0
    return {
        "system_state": state_name,
        "state_reason": reason,
        "safety_flags": active_flag_names(flags),
        "connection": {
            "jetson": safety is not None,
            "stm32": bool(mcu and mcu.connected),
            "lidar": lidar_connected,
            "camera": camera_connected,
        },
        "velocity": {
            "target_linear_mps": float(velocity.linear.x) if velocity else 0.0,
            "actual_linear_mps": actual,
            "angular_rad_s": float(velocity.angular.z) if velocity else 0.0,
            "left_mps": left,
            "right_mps": right,
        },
        "obstacle": {
            "detected": obstacle_detected,
            "object_class": obstacle.object_class if obstacle_detected else "없음",
            "distance_m": _optional_finite(obstacle.distance_m) if obstacle_detected else None,
            "ttc_s": _optional_finite(obstacle.ttc_s) if obstacle_detected else None,
            "direction": obstacle.direction if obstacle_detected else "전방",
        },
        "cliff": {
            "left": bool(cliff and cliff.left_detected),
            "right": bool(cliff and cliff.right_detected),
            "tof_danger": bool(cliff and cliff.tof_danger),
            "left_distance_m": (
                _optional_finite(cliff.left_distance_m) if cliff else None
            ),
            "right_distance_m": (
                _optional_finite(cliff.right_distance_m) if cliff else None
            ),
        },
        "range_sensors": {
            "ultrasonic_front_m": _optional_finite(getattr(mcu, "ultrasonic_front_m", math.nan)) if mcu else None,
            "sharp_left_m": _optional_finite(getattr(mcu, "sharp_left_m", math.nan)) if mcu else None,
            "sharp_right_m": _optional_finite(getattr(mcu, "sharp_right_m", math.nan)) if mcu else None,
        },
        "battery": {
            "voltage_v": _optional_finite(battery.voltage) if battery else None,
            "percent": (
                max(0.0, min(100.0, float(battery.percentage) * 100.0))
                if battery and math.isfinite(float(battery.percentage))
                else None
            ),
            "warning": bool(battery and battery.percentage < 0.2),
        },
        "diagnostics": {
            "protocol_version": 2,
            "motor_error": int(mcu.motor_error) if mcu else 0,
            "rx_error_count": int(mcu.rx_error_count) if mcu else 0,
            "last_command_id": int(mcu.last_command_id) if mcu else 0,
            "uptime_ms": int(mcu.uptime_ms) if mcu else 0,
        },
        "last_stop_reason": (
            reason if state_name in {"CONTROLLED_STOP", "EMERGENCY_STOP", "FAULT"} else "없음"
        ),
    }
