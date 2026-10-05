from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class LidarScanData:
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple[float, ...]


class LidarGenerator:
    def __init__(
        self,
        *,
        angle_min: float = -math.pi,
        angle_max: float = math.pi,
        angle_increment: float = math.radians(1.0),
        range_min: float = 0.12,
        range_max: float = 10.0,
        seed: int = 7,
    ) -> None:
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max
        self.random = random.Random(seed)

    def generate(
        self,
        obstacle_distance_m: float | None,
        obstacle_direction_deg: float = 0.0,
        obstacle_width_deg: float = 20.0,
        noise_stddev_m: float = 0.0,
    ) -> LidarScanData:
        count = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        ranges = [self.range_max] * count
        if obstacle_distance_m is not None:
            if not self.range_min <= obstacle_distance_m <= self.range_max:
                raise ValueError("obstacle distance is outside LiDAR range")
            center_angle = math.radians(obstacle_direction_deg)
            half_width = math.radians(obstacle_width_deg) / 2.0
            for index in range(count):
                angle = self.angle_min + index * self.angle_increment
                if abs(angle - center_angle) <= half_width:
                    noisy = obstacle_distance_m + self.random.gauss(0.0, noise_stddev_m)
                    ranges[index] = min(self.range_max, max(self.range_min, noisy))
        return LidarScanData(
            self.angle_min,
            self.angle_max,
            self.angle_increment,
            self.range_min,
            self.range_max,
            tuple(ranges),
        )

