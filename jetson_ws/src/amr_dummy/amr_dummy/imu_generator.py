from dataclasses import dataclass
import random


@dataclass(frozen=True)
class ImuData:
    angular_velocity_z: float
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float


class ImuGenerator:
    def __init__(self, seed: int = 11) -> None:
        self.random = random.Random(seed)

    def generate(
        self,
        *,
        commanded_angular_velocity_z: float = 0.0,
        gyro_bias_z: float = 0.0,
        noise_stddev: float = 0.0,
        acceleration_x: float = 0.0,
    ) -> ImuData:
        if noise_stddev < 0:
            raise ValueError("noise_stddev cannot be negative")
        gyro_z = commanded_angular_velocity_z + gyro_bias_z
        gyro_z += self.random.gauss(0.0, noise_stddev)
        return ImuData(gyro_z, acceleration_x, 0.0, 9.80665)
