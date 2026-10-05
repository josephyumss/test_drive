from dataclasses import dataclass

from jetson.amr_core.controller import ControllerInputs
from jetson.amr_core.velocity import VelocityCommand


@dataclass(frozen=True)
class NormalizedInputs:
    requested_linear_mps: float = 0.0
    requested_angular_rad_s: float = 0.0
    obstacle_distance_m: float = 10.0
    obstacle_closing_speed_mps: float = 0.0
    obstacle_valid: bool = False
    cliff_left: bool = False
    cliff_right: bool = False
    cliff_sensor_fault: bool = False
    mcu_connected: bool = False
    motor_fault: bool = False
    initialization_complete: bool = False
    reset_requested: bool = False
    drive_enable: bool = True
    obstacle_timed_out: bool = False
    cliff_timed_out: bool = False
    mcu_timed_out: bool = False


def to_controller_inputs(values: NormalizedInputs) -> ControllerInputs:
    distance = values.obstacle_distance_m if values.obstacle_valid else 10.0
    return ControllerInputs(
        requested_velocity=VelocityCommand(
            values.requested_linear_mps,
            values.requested_angular_rad_s,
        ),
        obstacle_distance_m=distance,
        obstacle_closing_speed_mps=(
            values.obstacle_closing_speed_mps if values.obstacle_valid else 0.0
        ),
        initialization_complete=values.initialization_complete,
        drive_enable=values.drive_enable,
        reset_requested=values.reset_requested,
        cliff_left=values.cliff_left,
        cliff_right=values.cliff_right,
        communication_timeout=values.mcu_timed_out,
        motor_fault=values.motor_fault,
        critical_sensor_fault=values.cliff_sensor_fault or values.cliff_timed_out,
        degraded_sensor=not values.mcu_connected,
        controlled_stop_requested=values.obstacle_timed_out,
    )
