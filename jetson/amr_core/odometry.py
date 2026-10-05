"""ROS-independent differential-drive odometry math."""

import math


def integrate_pose(x, y, yaw, left_mps, right_mps, wheel_base_m, dt_s):
    if wheel_base_m <= 0.0 or dt_s < 0.0:
        raise ValueError("invalid wheel_base_m or dt_s")
    linear = 0.5 * (left_mps + right_mps)
    angular = (right_mps - left_mps) / wheel_base_m
    midpoint = yaw + 0.5 * angular * dt_s
    return (
        x + linear * math.cos(midpoint) * dt_s,
        y + linear * math.sin(midpoint) * dt_s,
        math.atan2(math.sin(yaw + angular * dt_s), math.cos(yaw + angular * dt_s)),
        linear,
        angular,
    )
