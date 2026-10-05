"""Send representative status updates to an already-running dashboard server."""

from __future__ import annotations

import argparse
import json
import time
from urllib.request import Request, urlopen


SCENARIOS = {
    "run": {
        "system_state": "RUN",
        "state_reason": "정상 주행",
        "connection": {
            "jetson": True,
            "stm32": True,
            "lidar": True,
            "camera": True,
        },
        "velocity": {
            "target_linear_mps": 0.45,
            "actual_linear_mps": 0.43,
            "angular_rad_s": 0.0,
            "left_mps": 0.43,
            "right_mps": 0.43,
        },
        "obstacle": {
            "detected": False,
            "object_class": "없음",
            "distance_m": 3.2,
            "ttc_s": 7.4,
            "direction": "전방",
        },
        "battery": {"voltage_v": 23.8, "percent": 82, "warning": False},
    },
    "slow": {
        "system_state": "SLOW",
        "state_reason": "전방 장애물 접근",
        "velocity": {
            "target_linear_mps": 0.22,
            "actual_linear_mps": 0.24,
        },
        "obstacle": {
            "detected": True,
            "object_class": "보행자",
            "distance_m": 1.6,
            "ttc_s": 3.0,
            "direction": "전방 좌측",
        },
    },
    "cliff": {
        "system_state": "EMERGENCY_STOP",
        "state_reason": "좌측 낙차 위험 감지",
        "safety_flags": ["CLIFF_LEFT"],
        "velocity": {
            "target_linear_mps": 0.0,
            "actual_linear_mps": 0.0,
            "left_mps": 0.0,
            "right_mps": 0.0,
        },
        "cliff": {
            "left": True,
            "right": False,
            "tof_danger": True,
            "left_distance_m": 0.62,
            "right_distance_m": 0.11,
        },
        "last_stop_reason": "좌측 낙차 위험",
    },
}


def publish(url: str, payload: dict) -> None:
    request = Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=2) as response:
        if response.status != 200:
            raise RuntimeError(f"dashboard returned HTTP {response.status}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=SCENARIOS)
    parser.add_argument("--url", default="http://127.0.0.1:8080/api/status")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args()

    for index in range(args.repeat):
        publish(args.url, SCENARIOS[args.scenario])
        print(f"published {args.scenario} ({index + 1}/{args.repeat})")
        if index + 1 < args.repeat:
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
