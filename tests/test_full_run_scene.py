"""Ideal encoders with geometric camera/LiDAR/side rays against real shapes."""
from dataclasses import replace
import math
import unittest

from jetson.amr_core.full_run import FullRunConfig, FullRunController, ControlStatus, SideReading
from jetson.amr_core.full_run_legacy import LegacyControlAdapter
from jetson.amr_core.reactive_avoidance import WheelOdometry


def ray_rectangle(origin, direction, rectangle):
    enter, leave = -math.inf, math.inf
    for coordinate, velocity, low, high in zip(origin, direction, rectangle[:2], rectangle[2:]):
        if abs(velocity) < 1e-9:
            if not low <= coordinate <= high:
                return math.inf
            continue
        first, second = (low - coordinate) / velocity, (high - coordinate) / velocity
        enter, leave = max(enter, min(first, second)), min(leave, max(first, second))
    if leave < max(0, enter):
        return math.inf
    return max(0, enter)


class FullRunSceneTests(unittest.TestCase):
    def run_scene(self, rectangle, protocol="ctrl", pause_entry=False):
        config = FullRunConfig()
        now = 100.0
        core = FullRunController(config, 456, now, mcu_protocol=protocol)
        legacy = LegacyControlAdapter(456) if protocol == "legacy" else None
        truth = WheelOdometry(wheel_diameter_m=config.wheel_diameter_m, wheel_base_m=config.wheel_base_m)
        counts = [0.0, 0.0]
        up = 0
        phases = []
        returned = False
        pause_index = None
        pause_follower = None
        for index in range(18000):
            now += .02
            truth.update(*core.command, .02)
            for wheel in (0, 1):
                counts[wheel] += core.command[wheel] * config.encoder_counts_per_output_rev / 60 * .02
            if legacy:
                if index % 5 == 0:  # Old firmware reports integer RPM at 10 Hz.
                    base = 15 if up else 0
                    if pause_index is not None:
                        base = 0 if index - pause_index < 100 else 5
                    wheels = core.command
                    core.update_status(legacy.decode(f"$STATUS,{base},1234,45,0,0,{wheels[0]},{wheels[1]},0,0,0", now), now)
            else:
                core.update_status(ControlStatus(456, 15 if up else 0, up, 0, 0, 0, 0,
                                                 *core.command, int(index*20), int(counts[0]), int(counts[1])), now)
            pose = truth
            origin = (pose.x_m, pose.y_m)
            points, frontal = [], []
            for degree in range(-180, 180):
                angle = math.radians(degree)
                direction = (math.cos(angle + pose.yaw_rad), math.sin(angle + pose.yaw_rad))
                distance = ray_rectangle(origin, direction, rectangle)
                if math.isfinite(distance):
                    points.append((angle, max(.02, distance)))
                if abs(degree) <= 15:
                    frontal.append(min(distance, 12))
            corners = [(rectangle[x], rectangle[y]) for x in (0, 2) for y in (1, 3)]
            bearings = []
            for x, y in corners:
                dx, dy = x - pose.x_m, y - pose.y_m
                local_x = dx * math.cos(pose.yaw_rad) + dy * math.sin(pose.yaw_rad)
                local_y = -dx * math.sin(pose.yaw_rad) + dy * math.cos(pose.yaw_rad)
                if local_x > 0:
                    bearings.append(math.atan2(local_y, local_x))
            detections = []
            if bearings and max(bearings) >= -math.radians(34.5) and min(bearings) <= math.radians(34.5):
                focal = 320 / math.tan(math.radians(34.5))
                edges = [max(0, min(640, 320 - focal * math.tan(a))) for a in bearings]
                if max(edges) > min(edges):
                    detections = [{"class": "box", "confidence": .9,
                                   "xyxy": [min(edges), 80, max(edges), 400]}]
            core.update_camera(detections, now)
            core.update_scan(points, min(frontal), now)
            if index % 3 == 0:  # 60ms measurement, not a perfect echo every tick
                for side, sign in (("left", 1), ("right", -1)):
                    direction = (-sign * math.sin(pose.yaw_rad), sign * math.cos(pose.yaw_rad))
                    sensor = (pose.x_m + direction[0] * config.side_sensor_y_m,
                              pose.y_m + direction[1] * config.side_sensor_y_m)
                    distance = ray_rectangle(sensor, direction, rectangle)
                    valid = .02 <= distance <= config.ultrasonic_max_m
                    core.update_side(SideReading(side, now, "VALID" if valid else "NO_ECHO", distance if valid else None))
            core.tick(now)
            if core.state == "READY":
                up = 1
            if pause_entry and pause_index is None and core.phase == "ENTRY" and core.follower.progress_ratio > .3:
                pause_index, pause_follower = index, core.follower
            expected_pause = pause_index is not None and index - pause_index < 106
            if core.state == "FAULT_STOP" or (core.state == "PAUSED" and not expected_pause):
                self.fail(f"Geometric scene stopped: {core.snapshot(now)}")
            if expected_pause and core.phase == "ENTRY":
                self.assertIs(core.follower, pause_follower)
            if not phases or phases[-1] != core.phase:
                phases.append(core.phase)
                if core.phase == "DRIVE" and "RETURN" in phases:
                    returned = True
                    break
        self.assertTrue(returned, core.snapshot(now))
        self.assertIn("PASS", phases)
        self.assertIn("REAR_CLEARANCE", phases)
        self.assertLessEqual(abs(core.odom.y_m), config.path_completion_m)
        self.assertLessEqual(abs(truth.y_m), config.path_completion_m)
        self.assertGreater(truth.x_m - config.robot_length_m / 2, rectangle[2])
        if pause_entry:
            self.assertIsNotNone(pause_index)

    def test_thin_square_pole(self):
        self.run_scene((1.85, -.06, 1.97, .06))

    def test_long_rectangular_object(self):
        self.run_scene((1.85, -.15, 4.0, .15))

    def test_legacy_100ms_RPM_with_thin_pole(self):
        self.run_scene((1.85, -.06, 1.97, .06), protocol="legacy")

    def test_legacy_100ms_RPM_with_long_object(self):
        self.run_scene((1.85, -.15, 4.0, .15), protocol="legacy")

    def test_legacy_100ms_RPM_pause_resumes_same_entry_curve(self):
        self.run_scene((1.85, -.06, 1.97, .06), protocol="legacy", pause_entry=True)


if __name__ == "__main__":
    unittest.main()
