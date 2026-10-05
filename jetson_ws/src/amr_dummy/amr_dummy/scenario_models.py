from dataclasses import dataclass


@dataclass(frozen=True)
class ScenarioState:
    name: str = "normal"
    obstacle_distance_m: float | None = None
    obstacle_closing_speed_mps: float = 0.0
    obstacle_direction_deg: float = 0.0
    obstacle_width_deg: float = 20.0
    commanded_linear_mps: float = 0.45
    commanded_angular_rad_s: float = 0.0
    actual_linear_mps: float = 0.43
    gyro_bias_z_rad_s: float = 0.0
    imu_noise_stddev: float = 0.0
    cliff_left: bool = False
    cliff_right: bool = False
    tof_distance_m: float = 0.12
    battery_voltage_v: float = 23.8
    battery_percent: float = 0.82
    motor_fault: bool = False
    reset_requested: bool = False
    publish_lidar: bool = True
    publish_imu: bool = True
    publish_mcu_status: bool = True
    wheel_slip_ratio: float = 0.0
