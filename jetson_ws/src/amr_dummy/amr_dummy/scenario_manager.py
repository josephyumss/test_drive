from .scenario_models import ScenarioState


PRESETS = {
    "normal": ScenarioState(),
    "obstacle_slow": ScenarioState(
        name="obstacle_slow",
        obstacle_distance_m=1.6,
        obstacle_closing_speed_mps=0.45,
        commanded_linear_mps=0.45,
    ),
    "obstacle_stop": ScenarioState(
        name="obstacle_stop",
        obstacle_distance_m=1.0,
        obstacle_closing_speed_mps=0.2,
        commanded_linear_mps=0.45,
    ),
    "cliff_stop": ScenarioState(
        name="cliff_stop",
        cliff_left=True,
        tof_distance_m=0.8,
        commanded_linear_mps=0.35,
    ),
    "communication_loss": ScenarioState(
        name="communication_loss",
        publish_mcu_status=False,
    ),
    "imu_timeout": ScenarioState(name="imu_timeout", publish_imu=False),
    "imu_drift": ScenarioState(
        name="imu_drift",
        gyro_bias_z_rad_s=0.0035,
        imu_noise_stddev=0.001,
    ),
    "wheel_slip": ScenarioState(
        name="wheel_slip",
        commanded_linear_mps=0.45,
        actual_linear_mps=0.25,
        wheel_slip_ratio=0.44,
    ),
    "motor_fault": ScenarioState(name="motor_fault", motor_fault=True),
    "reset": ScenarioState(
        name="reset",
        commanded_linear_mps=0.0,
        actual_linear_mps=0.0,
        reset_requested=True,
    ),
}


class ScenarioManager:
    """Return deterministic dummy states from a named or timed scenario."""

    AUTOMATIC_PHASES = (
        (5.0, "normal"),
        (10.0, "obstacle_slow"),
        (15.0, "obstacle_stop"),
        (17.0, "reset"),
        (21.0, "normal"),
        (25.0, "cliff_stop"),
        (27.0, "reset"),
        (32.0, "communication_loss"),
        (34.0, "reset"),
        (39.0, "imu_drift"),
        (44.0, "wheel_slip"),
    )

    def __init__(self, scenario: str = "automatic") -> None:
        if scenario != "automatic" and scenario not in PRESETS:
            raise ValueError(f"unknown scenario: {scenario}")
        self.scenario = scenario

    def state_at(self, elapsed_s: float) -> ScenarioState:
        if elapsed_s < 0:
            raise ValueError("elapsed_s cannot be negative")
        if self.scenario != "automatic":
            return PRESETS[self.scenario]
        cycle = elapsed_s % self.AUTOMATIC_PHASES[-1][0]
        for end_s, name in self.AUTOMATIC_PHASES:
            if cycle < end_s:
                return PRESETS[name]
        return PRESETS["normal"]
