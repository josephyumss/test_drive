"""Hardware-independent fusion for short-range and LiDAR measurements."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class RangeReading:
    source: str
    distance_m: float
    stamp_s: float
    valid: bool = True


@dataclass(frozen=True)
class FusedRange:
    distance_m: float
    source: str
    closing_speed_mps: float
    valid: bool


class RangeFusion:
    """Choose the nearest fresh reading and estimate closing speed."""

    def __init__(self, *, timeout_s: float = 0.3, speed_alpha: float = 0.35) -> None:
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if not 0 < speed_alpha <= 1:
            raise ValueError("speed_alpha must be in (0, 1]")
        self.timeout_s = timeout_s
        self.speed_alpha = speed_alpha
        self._previous: dict[str, RangeReading] = {}
        self._closing_speed: dict[str, float] = {}

    def update(self, reading: RangeReading) -> None:
        if not reading.valid or not math.isfinite(reading.distance_m):
            return
        if reading.distance_m < 0:
            return
        previous = self._previous.get(reading.source)
        if previous is not None:
            dt = reading.stamp_s - previous.stamp_s
            if 0 < dt <= 1.0:
                instantaneous = max(0.0, (previous.distance_m - reading.distance_m) / dt)
                old = self._closing_speed.get(reading.source, 0.0)
                self._closing_speed[reading.source] = (
                    self.speed_alpha * instantaneous + (1.0 - self.speed_alpha) * old
                )
        self._previous[reading.source] = reading

    def result(self, now_s: float) -> FusedRange:
        fresh = [
            item
            for item in self._previous.values()
            if 0 <= now_s - item.stamp_s <= self.timeout_s
        ]
        if not fresh:
            return FusedRange(math.inf, "none", 0.0, False)
        nearest = min(fresh, key=lambda item: item.distance_m)
        return FusedRange(
            nearest.distance_m,
            nearest.source,
            self._closing_speed.get(nearest.source, 0.0),
            True,
        )


def sector_minimum(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    sector_min: float,
    sector_max: float,
    range_min: float,
    range_max: float,
) -> float | None:
    valid: list[float] = []
    for index, value in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if sector_min <= angle <= sector_max and math.isfinite(value):
            if range_min <= value <= range_max:
                valid.append(float(value))
    return min(valid) if valid else None
