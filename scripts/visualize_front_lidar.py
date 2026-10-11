#!/usr/bin/env python3
"""Live terminal view of the /scan points used by the full-run front guard."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from jetson.amr_core.full_run import FullRunConfig
from scripts.full_run_controller import parse_scan


def analyse_scan(message, config):
    metrics = {}
    points, radial_front_m = parse_scan(message, config, metrics)
    half_corridor_m = config.robot_width_m / 2 + config.side_safety_margin_m
    corridor = []
    for angle, distance in points:
        x_m = distance * math.cos(angle)
        y_m = distance * math.sin(angle)
        if x_m > 0.01 and abs(y_m) <= half_corridor_m:
            corridor.append({
                "x_m": x_m,
                "y_m": y_m,
                "range_m": distance,
                "angle_deg": math.degrees(angle),
                "bumper_clearance_m": x_m - config.robot_length_m / 2,
            })
    corridor.sort(key=lambda item: item["bumper_clearance_m"])
    fallback_clearance_m = radial_front_m - config.robot_length_m / 2
    clearance_m = corridor[0]["bumper_clearance_m"] if corridor else fallback_clearance_m
    if clearance_m <= config.front_stop_m:
        decision = "STOP"
    elif clearance_m <= config.front_resume_m:
        decision = "START BLOCKED"
    else:
        decision = "CLEAR"
    return {
        "points": points,
        "corridor": corridor,
        "nearest": corridor[0] if corridor else None,
        "radial_front_m": radial_front_m,
        "clearance_m": clearance_m,
        "half_corridor_m": half_corridor_m,
        "decision": decision,
        "metrics": metrics,
    }


def render_map(result, config, *, width=73, height=25, forward_m=4.0, lateral_m=2.0):
    grid = [[" " for _ in range(width)] for _ in range(height)]

    def cell(x_m, y_m):
        if not 0.0 <= x_m <= forward_m or not -lateral_m <= y_m <= lateral_m:
            return None
        row = height - 1 - round(x_m / forward_m * (height - 1))
        # Positive y is robot-left, shown on the left side of the terminal.
        col = round((lateral_m - y_m) / (2 * lateral_m) * (width - 1))
        return row, col

    def put(x_m, y_m, character):
        location = cell(x_m, y_m)
        if location is not None:
            row, col = location
            grid[row][col] = character

    half_corridor = result["half_corridor_m"]
    for row in range(height):
        x_m = (height - 1 - row) / (height - 1) * forward_m
        put(x_m, half_corridor, "|")
        put(x_m, -half_corridor, "|")

    bumper_x = config.robot_length_m / 2
    for x_m, character in ((bumper_x + config.front_stop_m, "-"),
                           (bumper_x + config.front_resume_m, "=")):
        for col in range(width):
            y_m = lateral_m - col / (width - 1) * 2 * lateral_m
            if abs(y_m) <= half_corridor:
                put(x_m, y_m, character)

    for angle, distance in result["points"]:
        x_m = distance * math.cos(angle)
        y_m = distance * math.sin(angle)
        put(x_m, y_m, "#" if abs(y_m) <= half_corridor and x_m > 0 else ".")

    nearest = result["nearest"]
    if nearest:
        put(nearest["x_m"], nearest["y_m"], "X")

    half_width = config.robot_width_m / 2
    half_length = config.robot_length_m / 2
    for row in range(height):
        x_m = (height - 1 - row) / (height - 1) * forward_m
        if 0 <= x_m <= half_length:
            for col in range(width):
                y_m = lateral_m - col / (width - 1) * 2 * lateral_m
                if abs(y_m) <= half_width:
                    grid[row][col] = "R"
    put(config.lidar_x_m, config.lidar_y_m, "L")
    return "\n".join("".join(row) for row in grid)


def format_screen(result, config, sequence, age_s):
    nearest = result["nearest"]
    if nearest:
        nearest_text = (
            f"x={nearest['x_m']:.3f}m  y={nearest['y_m']:+.3f}m  "
            f"angle={nearest['angle_deg']:+.1f}deg  range={nearest['range_m']:.3f}m"
        )
    else:
        nearest_text = "none (front-sector fallback is being used)"
    closest = result["corridor"][:5]
    rows = [
        "LIVE FRONT LiDAR  /scan   Ctrl+C: exit",
        f"scan={sequence} age={age_s * 1000:.0f}ms  decision={result['decision']}",
        f"controller front bumper clearance = {result['clearance_m']:.3f}m "
        f"({result['clearance_m'] * 100:.1f}cm)",
        f"thresholds: STOP <= {config.front_stop_m:.2f}m, "
        f"START BLOCKED <= {config.front_resume_m:.2f}m",
        f"front corridor: y=+/-{result['half_corridor_m']:.3f}m "
        f"(robot half width {config.robot_width_m / 2:.3f} + margin {config.side_safety_margin_m:.3f})",
        f"nearest controller-visible return: {nearest_text}",
        f"raw +/-15deg radial minimum={result['radial_front_m']:.3f}m  "
        f"valid={result['metrics']['valid_points']} self-filtered={result['metrics']['self_filtered_points']} "
        f"retained={result['metrics']['retained_points']}",
        f"self-filter envelope margin={config.lidar_self_filter_margin_m:.3f}m "
        f"(does not change stop/resume thresholds)",
        "",
        "TOP VIEW: forward=up, left=screen-left; X=blocking return, #=corridor point, .=other point",
        "          R=robot, L=LiDAR, -=stop line, ==resume line, |=corridor edge",
        render_map(result, config),
        "",
        "Nearest five points inside the controller corridor:",
    ]
    if closest:
        rows.extend(
            f"  {index}. bumper={item['bumper_clearance_m'] * 100:6.1f}cm  "
            f"x={item['x_m']:.3f}m y={item['y_m']:+.3f}m angle={item['angle_deg']:+.1f}deg"
            for index, item in enumerate(closest, 1)
        )
    else:
        rows.append("  no finite return inside the corridor")
    return "\n".join(rows)


def main(arguments=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT / "config/full_run.json"))
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--refresh-hz", type=float, default=4.0)
    parser.add_argument("--no-clear", action="store_true", help="Do not redraw the terminal in place")
    args = parser.parse_args(arguments)
    if args.refresh_hz <= 0:
        parser.error("--refresh-hz must be positive")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    config = FullRunConfig.read(args.config)

    class Viewer(Node):
        def __init__(self):
            super().__init__("full_run_front_lidar_viewer")
            self.latest = None
            self.received_at = None
            self.sequence = 0
            self.error = None
            self.create_subscription(LaserScan, args.topic, self.on_scan, qos_profile_sensor_data)

        def on_scan(self, message):
            self.sequence += 1
            self.received_at = time.monotonic()
            try:
                self.latest = analyse_scan(message, config)
                self.error = None
            except (TypeError, ValueError) as exc:
                self.error = str(exc)

    rclpy.init()
    node = Viewer()
    print(f"Waiting for {args.topic} ...", flush=True)
    next_render = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now < next_render:
                continue
            next_render = now + 1.0 / args.refresh_hz
            if node.error:
                screen = f"[INVALID SCAN] {node.error}"
            elif node.latest is None:
                screen = f"[WAIT] No {args.topic} message received yet"
            else:
                screen = format_screen(node.latest, config, node.sequence, now - node.received_at)
            if args.no_clear:
                print(screen, "\n", flush=True)
            else:
                print("\033[2J\033[H" + screen, end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
