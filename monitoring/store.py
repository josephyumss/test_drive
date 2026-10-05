"""Thread-safe dashboard state and event history."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import math
from threading import Lock
import time
from typing import Any


VALID_STATES = {
    "INIT",
    "READY",
    "RUN",
    "SLOW",
    "CONTROLLED_STOP",
    "EMERGENCY_STOP",
    "FAULT",
}


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def default_snapshot() -> dict[str, Any]:
    return {
        "system_state": "INIT",
        "state_reason": "관제 데이터 대기 중",
        "safety_flags": [],
        "connection": {
            "jetson": False,
            "stm32": False,
            "lidar": False,
            "camera": False,
        },
        "velocity": {
            "target_linear_mps": 0.0,
            "actual_linear_mps": 0.0,
            "angular_rad_s": 0.0,
            "left_mps": 0.0,
            "right_mps": 0.0,
        },
        "obstacle": {
            "detected": False,
            "object_class": "없음",
            "distance_m": None,
            "ttc_s": None,
            "direction": "전방",
        },
        "cliff": {
            "left": False,
            "right": False,
            "tof_danger": False,
            "left_distance_m": None,
            "right_distance_m": None,
        },
        "range_sensors": {
            "ultrasonic_front_m": None,
            "sharp_left_m": None,
            "sharp_right_m": None,
        },
        "battery": {
            "voltage_v": None,
            "percent": None,
            "warning": False,
        },
        "diagnostics": {
            "protocol_version": 2,
            "motor_error": 0,
            "rx_error_count": 0,
            "last_command_id": 0,
            "uptime_ms": 0,
        },
        "last_stop_reason": "없음",
        "source_timestamp": None,
    }


class MonitoringStore:
    """Keep the most recent robot snapshot and a bounded event history."""

    def __init__(self, *, stale_after_s: float = 1.0, event_limit: int = 100) -> None:
        if stale_after_s <= 0:
            raise ValueError("stale_after_s must be positive")
        if event_limit <= 0:
            raise ValueError("event_limit must be positive")
        self._lock = Lock()
        self._snapshot = default_snapshot()
        self._events: list[dict[str, Any]] = []
        self._last_update_monotonic: float | None = None
        self._stale_after_s = stale_after_s
        self._event_limit = event_limit

    def update(self, incoming: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(incoming, dict):
            raise ValueError("status body must be a JSON object")

        with self._lock:
            previous = deepcopy(self._snapshot)
            candidate = deepcopy(self._snapshot)
            self._merge_known(candidate, incoming)
            self._validate(candidate)
            candidate["source_timestamp"] = incoming.get(
                "source_timestamp", utc_now_iso()
            )
            self._snapshot = candidate
            self._last_update_monotonic = time.monotonic()
            self._record_changes(previous, candidate)
            return self._build_response_locked()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._build_response_locked()

    def add_event(self, level: str, message: str) -> None:
        if level not in {"info", "warning", "critical"}:
            raise ValueError("invalid event level")
        if not message.strip():
            raise ValueError("event message cannot be empty")
        with self._lock:
            self._append_event(level, message.strip())

    def _build_response_locked(self) -> dict[str, Any]:
        now = time.monotonic()
        age_s = (
            None
            if self._last_update_monotonic is None
            else max(0.0, now - self._last_update_monotonic)
        )
        stale = age_s is None or age_s > self._stale_after_s
        return {
            "status": deepcopy(self._snapshot),
            "events": deepcopy(self._events[-30:]),
            "monitoring": {
                "stale": stale,
                "age_ms": None if age_s is None else round(age_s * 1000),
                "server_time": utc_now_iso(),
            },
        }

    @classmethod
    def _merge_known(cls, target: dict[str, Any], source: dict[str, Any]) -> None:
        for key, value in source.items():
            if key not in target:
                continue
            if isinstance(target[key], dict):
                if not isinstance(value, dict):
                    raise ValueError(f"{key} must be an object")
                cls._merge_known(target[key], value)
            else:
                target[key] = value

    @staticmethod
    def _validate(snapshot: dict[str, Any]) -> None:
        if snapshot["system_state"] not in VALID_STATES:
            raise ValueError("invalid system_state")
        if not isinstance(snapshot["state_reason"], str):
            raise ValueError("state_reason must be a string")
        if not isinstance(snapshot["safety_flags"], list) or not all(
            isinstance(item, str) for item in snapshot["safety_flags"]
        ):
            raise ValueError("safety_flags must be a list of strings")

        for name, connected in snapshot["connection"].items():
            if not isinstance(connected, bool):
                raise ValueError(f"connection.{name} must be boolean")

        for name, value in snapshot["velocity"].items():
            MonitoringStore._finite_number(value, f"velocity.{name}")

        obstacle = snapshot["obstacle"]
        if not isinstance(obstacle["detected"], bool):
            raise ValueError("obstacle.detected must be boolean")
        MonitoringStore._optional_non_negative(
            obstacle["distance_m"], "obstacle.distance_m"
        )
        MonitoringStore._optional_non_negative(obstacle["ttc_s"], "obstacle.ttc_s")

        cliff = snapshot["cliff"]
        for name in ("left", "right", "tof_danger"):
            if not isinstance(cliff[name], bool):
                raise ValueError(f"cliff.{name} must be boolean")
        MonitoringStore._optional_non_negative(
            cliff["left_distance_m"], "cliff.left_distance_m"
        )
        MonitoringStore._optional_non_negative(
            cliff["right_distance_m"], "cliff.right_distance_m"
        )
        for name, value in snapshot["range_sensors"].items():
            MonitoringStore._optional_non_negative(value, f"range_sensors.{name}")

        battery = snapshot["battery"]
        MonitoringStore._optional_non_negative(
            battery["voltage_v"], "battery.voltage_v"
        )
        if battery["percent"] is not None:
            MonitoringStore._finite_number(battery["percent"], "battery.percent")
            if not 0 <= battery["percent"] <= 100:
                raise ValueError("battery.percent must be between 0 and 100")
        if not isinstance(battery["warning"], bool):
            raise ValueError("battery.warning must be boolean")

        for name, value in snapshot["diagnostics"].items():
            if not isinstance(value, int) or value < 0:
                raise ValueError(f"diagnostics.{name} must be a non-negative integer")

    @staticmethod
    def _finite_number(value: Any, field: str) -> None:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{field} must be numeric")
        if not math.isfinite(value):
            raise ValueError(f"{field} must be finite")

    @staticmethod
    def _optional_non_negative(value: Any, field: str) -> None:
        if value is None:
            return
        MonitoringStore._finite_number(value, field)
        if value < 0:
            raise ValueError(f"{field} cannot be negative")

    def _record_changes(
        self, previous: dict[str, Any], current: dict[str, Any]
    ) -> None:
        if previous["system_state"] != current["system_state"]:
            level = (
                "critical"
                if current["system_state"] in {"EMERGENCY_STOP", "FAULT"}
                else "warning"
                if current["system_state"] in {"SLOW", "CONTROLLED_STOP"}
                else "info"
            )
            self._append_event(
                level,
                f"상태 변경: {previous['system_state']} → {current['system_state']} "
                f"({current['state_reason']})",
            )

        previous_flags = set(previous["safety_flags"])
        for flag in set(current["safety_flags"]) - previous_flags:
            self._append_event("critical", f"안전 플래그 발생: {flag}")

        for name, connected in current["connection"].items():
            if previous["connection"][name] and not connected:
                self._append_event("critical", f"{name.upper()} 연결 끊김")
            elif not previous["connection"][name] and connected:
                self._append_event("info", f"{name.upper()} 연결됨")

    def _append_event(self, level: str, message: str) -> None:
        self._events.append(
            {"time": utc_now_iso(), "level": level, "message": message}
        )
        if len(self._events) > self._event_limit:
            del self._events[: len(self._events) - self._event_limit]
