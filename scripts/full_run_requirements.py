#!/usr/bin/env python3
"""Check every import/source identity without opening GPIO/UART or moving motors."""
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import traceback

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))


def verify_imports(project=PROJECT):
    results = []
    names = ["serial", "rclpy", "sensor_msgs.msg:LaserScan", "std_msgs.msg:String",
             "amr_interfaces.msg:ObstacleInfo", "amr_vision.yolo_udp_bridge_node"]
    if os.environ.get("SIDE_SENSOR_SOURCE", "mcu") == "gpio":
        names.insert(2, "Jetson.GPIO")
    for name in names:
        entry = {"name": name, "ok": False}
        try:
            module_name, _, attribute = name.partition(":")
            module = importlib.import_module(module_name)
            if attribute:
                getattr(module, attribute)
            entry["file"] = getattr(module, "__file__", None)
            if module_name == "amr_vision.yolo_udp_bridge_node":
                expected = Path(project) / "jetson_ws/src/amr_vision/amr_vision/yolo_udp_bridge_node.py"
                entry["expected_sha256"] = hashlib.sha256(expected.read_bytes()).hexdigest()
                entry["installed_sha256"] = hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
                if entry["expected_sha256"] != entry["installed_sha256"]:
                    raise RuntimeError("Installed ROS bridge differs from checkout; rerun setup to rebuild")
            entry["ok"] = True
        except Exception:
            entry["traceback"] = traceback.format_exc()
        results.append(entry)
    return results


if __name__ == "__main__":
    report = verify_imports()
    print(json.dumps({"checks": report, "failed": [r["name"] for r in report if not r["ok"]]}, indent=2))
    raise SystemExit(int(any(not r["ok"] for r in report)))
