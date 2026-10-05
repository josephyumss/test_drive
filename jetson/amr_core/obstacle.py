"""Distance and time-to-collision based obstacle evaluation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math


class ObstacleDecision(IntEnum):
    RUN = 0
    SLOW = 1
    CONTROLLED_STOP = 2
    EMERGENCY_STOP = 3


@dataclass(frozen=True)
class ObstacleThresholds:
    slow_distance_m: float = 2.0
    stop_distance_m: float = 1.2
    emergency_distance_m: float = 0.8
    slow_ttc_s: float = 4.0
    stop_ttc_s: float = 2.0
    emergency_ttc_s: float = 1.0

    def __post_init__(self) -> None:
        if not (
            self.emergency_distance_m
            < self.stop_distance_m
            < self.slow_distance_m
        ):
            raise ValueError("distance thresholds must be emergency < stop < slow")
        if not self.emergency_ttc_s < self.stop_ttc_s < self.slow_ttc_s:
            raise ValueError("TTC thresholds must be emergency < stop < slow")


@dataclass(frozen=True)
class ObstacleAssessment:
    decision: ObstacleDecision
    distance_m: float
    closing_speed_mps: float
    ttc_s: float


class ObstacleEvaluator:
    def __init__(self, thresholds: ObstacleThresholds | None = None) -> None:
        self.thresholds = thresholds or ObstacleThresholds()

    @staticmethod
    def calculate_ttc(distance_m: float, closing_speed_mps: float) -> float:
        if distance_m < 0:
            raise ValueError("distance cannot be negative")
        if closing_speed_mps <= 0:
            return math.inf
        return distance_m / closing_speed_mps

    def evaluate(
        self, distance_m: float, closing_speed_mps: float
    ) -> ObstacleAssessment:
        if not math.isfinite(distance_m) or distance_m < 0:
            raise ValueError("distance must be a finite non-negative value")
        if not math.isfinite(closing_speed_mps):
            raise ValueError("closing speed must be finite")

        ttc_s = self.calculate_ttc(distance_m, closing_speed_mps)
        distance_decision = self._decision_from_distance(distance_m)
        ttc_decision = self._decision_from_ttc(ttc_s)
        decision = max(distance_decision, ttc_decision)
        return ObstacleAssessment(decision, distance_m, closing_speed_mps, ttc_s)

    def _decision_from_distance(self, distance_m: float) -> ObstacleDecision:
        threshold = self.thresholds
        if distance_m < threshold.emergency_distance_m:
            return ObstacleDecision.EMERGENCY_STOP
        if distance_m < threshold.stop_distance_m:
            return ObstacleDecision.CONTROLLED_STOP
        if distance_m < threshold.slow_distance_m:
            return ObstacleDecision.SLOW
        return ObstacleDecision.RUN

    def _decision_from_ttc(self, ttc_s: float) -> ObstacleDecision:
        threshold = self.thresholds
        if ttc_s < threshold.emergency_ttc_s:
            return ObstacleDecision.EMERGENCY_STOP
        if ttc_s < threshold.stop_ttc_s:
            return ObstacleDecision.CONTROLLED_STOP
        if ttc_s < threshold.slow_ttc_s:
            return ObstacleDecision.SLOW
        return ObstacleDecision.RUN

