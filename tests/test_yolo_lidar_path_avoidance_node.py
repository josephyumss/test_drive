"""Hardware-free integration checks for the one-shot YOLO/LiDAR controller."""

import importlib.util
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


class FakeSerial:
    def __init__(self, *_args, **_kwargs):
        self.incoming = bytearray()
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def reset_input_buffer(self):
        self.incoming.clear()

    def read(self, count):
        payload = bytes(self.incoming[:count])
        del self.incoming[:count]
        return payload

    def write(self, payload):
        if self.closed:
            raise OSError("serial port is closed")
        self.writes.append(payload)
        return len(payload)

    def flush(self):
        pass

    def close(self):
        self.closed = True


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.subscriptions = []

    def create_subscription(self, *arguments):
        self.subscriptions.append(arguments)

    def destroy_node(self):
        pass


class FakeMessage:
    pass


def load_node_module():
    serial = types.ModuleType("serial")
    serial.Serial = FakeSerial
    serial.SerialException = OSError
    rclpy = types.ModuleType("rclpy")
    rclpy.__path__ = []
    rclpy.init = lambda **_kwargs: None
    rclpy.ok = lambda: False
    rclpy.spin_once = lambda *_args, **_kwargs: None
    rclpy.shutdown = lambda: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    stubs = {
        "serial": serial,
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        # Importing GPIO is forbidden: this controller must run without it.
        "Jetson": None,
        "Jetson.GPIO": None,
    }
    for package, message_type in (("sensor_msgs", "LaserScan"), ("std_msgs", "String")):
        parent = types.ModuleType(package)
        parent.__path__ = []
        messages = types.ModuleType(f"{package}.msg")
        setattr(messages, message_type, FakeMessage)
        parent.msg = messages
        stubs[package] = parent
        stubs[f"{package}.msg"] = messages
    script = Path(__file__).resolve().parents[1] / "scripts/test_yolo_lidar_path_avoidance.py"
    specification = importlib.util.spec_from_file_location(
        "yolo_lidar_path_avoidance_under_test", script
    )
    module = importlib.util.module_from_spec(specification)
    with mock.patch.dict(sys.modules, stubs):
        specification.loader.exec_module(module)
    return module


class YoloLidarPathAvoidanceNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_module()

    def setUp(self):
        self.now = 100.0
        self.clock = mock.patch.object(self.module.time, "monotonic", side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.sleep = mock.patch.object(self.module.time, "sleep", return_value=None)
        self.sleep.start()
        self.addCleanup(self.sleep.stop)
        self.node = self.module.YoloLidarAvoidance()
        self.addCleanup(self.node.shutdown)

    def scan(self, *, start_deg=-180, count=360, distance=4.0, overrides=None):
        ranges = [distance] * count
        for angle, value in (overrides or {}).items():
            ranges[(angle - start_deg) % 360] = value
        return types.SimpleNamespace(
            ranges=ranges,
            angle_min=math.radians(start_deg),
            angle_increment=math.radians(1.0),
            range_min=0.05,
            range_max=12.0,
        )

    def yolo_message(self, detections=None):
        if detections is None:
            detections = [{"class": "person", "confidence": 0.9, "xyxy": [280, 80, 360, 360]}]
        return types.SimpleNamespace(data=json.dumps(detections))

    def status(self, *, left=0, right=0, emergency=0, sharp_adc="unavailable", sharp_cm="unavailable"):
        payload = f"$STATUS,0,{sharp_adc},{sharp_cm},0,0,{left},{right},0,0,{emergency}\r\n"
        self.node.serial.incoming.extend(payload.encode("ascii"))
        self.node.read_stm32()

    def refresh_feeds(self):
        self.node.on_scan(self.scan())
        self.node.on_yolo(self.yolo_message([]))
        self.status()

    def start_driving(self):
        self.refresh_feeds()
        self.node.check_health(self.now)
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.DRIVE)
        self.assertEqual(self.node.motor_command(self.now), (self.module.CRUISE_RPM,) * 2)

    def detect_obstacle(self):
        self.node.on_scan(self.scan(overrides={angle: 1.5 for angle in range(-2, 3)}))
        for _ in range(self.module.REQUIRED_DETECTION_FRAMES):
            self.node.on_yolo(self.yolo_message())
        self.node.update_fused_objects()
        self.assertIsNotNone(self.node.priority_target)
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.STOPPING)

    def complete_avoidance(self):
        self.start_driving()
        self.detect_obstacle()
        self.now += self.module.STOP_BEFORE_TURN_S + 0.01
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.TURN_LEFT)
        self.node.odometry.x_m, self.node.odometry.y_m = self.node.entry_follower.path.p3
        self.node.odometry.yaw_rad = 0.0
        self.refresh_feeds()
        self.node.update_fused_objects()
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.BYPASS)
        self.node.odometry.distance_travelled_m = self.node.bypass_start_distance_m + self.module.BYPASS_DISTANCE_M + 0.01
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.RETURN_PATH)
        self.node.odometry.x_m, self.node.odometry.y_m = self.node.return_follower.path.p3
        self.node.odometry.yaw_rad = 0.0
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.POST_AVOIDANCE)

    def test_startup_holds_motors_until_required_feeds_arrive(self):
        self.assertEqual(self.node.motor_command(self.now), (0, 0))
        self.node.on_scan(self.scan())
        self.node.on_yolo(self.yolo_message([]))
        self.node.check_health(self.now)
        self.node.update_state(self.now)
        self.assertEqual(self.node.motor_command(self.now), (0, 0))
        self.status()
        self.node.check_health(self.now)
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.DRIVE)

    def test_missing_sharp_and_ultrasonic_do_not_block_driving(self):
        self.start_driving()
        self.assertEqual(set(self.node.stamps), {"camera", "lidar", "stm32"})
        self.assertTrue(self.node.left_clear())
        self.assertTrue(self.node.right_clear())
        self.assertTrue(self.node.all_sensors_fresh(self.now))

    def test_complete_fused_avoidance_then_exactly_two_seconds_forward_and_stop(self):
        self.complete_avoidance()
        started = self.now
        self.assertEqual(self.module.POST_AVOIDANCE_FORWARD_S, 2.0)
        self.now = started + 1.99
        self.refresh_feeds()
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.POST_AVOIDANCE)
        self.assertEqual(self.node.motor_command(self.now), (self.module.CRUISE_RPM,) * 2)
        self.now = started + 2.0
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.COMPLETE)
        self.assertEqual(self.node.motor_command(self.now), (0, 0))
        self.now += 20.0
        self.refresh_feeds()
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.COMPLETE)
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_post_avoidance_motor_deadline_stops_without_state_update(self):
        self.complete_avoidance()
        self.now += 2.0
        self.refresh_feeds()
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_closed_loop_wheel_feedback_converges_and_finishes_once(self):
        started, step_s = self.now, 0.03
        transitions = {}
        recovered_pose = None
        # Ideal wheel tracking: previous commands become measured RPM. Pose is
        # integrated only by the real controller, never set to path endpoints.
        with contextlib.redirect_stdout(io.StringIO()):
            for tick in range(int(self.module.MAXIMUM_RUNTIME_S / step_s) + 2):
                self.now = started + tick * step_s
                visible = tick >= 35 and not self.node.maneuver_started
                obstacle = {angle: 1.8 for angle in range(-2, 3)} if visible else {}
                self.node.on_scan(self.scan(overrides=obstacle))
                self.node.on_yolo(self.yolo_message(None if visible else []))
                left, right = self.node.last_command
                self.status(left=left, right=right)
                self.node.tick()
                if self.node.state not in transitions:
                    transitions[self.node.state] = self.now
                    if self.node.state == self.node.POST_AVOIDANCE:
                        recovered_pose = (
                            self.node.odometry.x_m,
                            self.node.odometry.y_m,
                            self.node.odometry.yaw_rad,
                        )
                if self.node.finished:
                    break

        self.assertEqual(self.node.state, self.node.COMPLETE, self.node.abort_reason)
        self.assertEqual(list(transitions), [
            self.node.DRIVE, self.node.STOPPING, self.node.TURN_LEFT,
            self.node.BYPASS, self.node.RETURN_PATH,
            self.node.POST_AVOIDANCE, self.node.COMPLETE,
        ])
        self.assertLess(transitions[self.node.BYPASS] - transitions[self.node.TURN_LEFT],
                        self.module.MAXIMUM_ENTRY_PATH_S)
        self.assertLess(transitions[self.node.POST_AVOIDANCE] - transitions[self.node.RETURN_PATH],
                        self.module.MAXIMUM_RETURN_PATH_S)
        self.assertLessEqual(abs(recovered_pose[1]), self.module.PATH_OFFSET_TOLERANCE_M)
        self.assertLessEqual(abs(math.degrees(recovered_pose[2])), self.module.HEADING_TOLERANCE_DEG)
        final_duration = transitions[self.node.COMPLETE] - transitions[self.node.POST_AVOIDANCE]
        self.assertGreaterEqual(final_duration, 2.0)
        self.assertLessEqual(final_duration, 2.0 + step_s)
        self.assertGreater(self.node.odometry.x_m - recovered_pose[0], 0.35)
        self.assertEqual(self.node.serial.writes[-1], b"$CMD,0,0,0\r\n")

    def test_yolo_without_matching_lidar_cannot_start_avoidance(self):
        self.start_driving()
        for _ in range(self.module.REQUIRED_DETECTION_FRAMES):
            self.node.on_yolo(self.yolo_message())
        self.node.update_fused_objects()
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.DRIVE)

    def test_lidar_without_yolo_cannot_start_normal_avoidance(self):
        self.start_driving()
        self.node.on_scan(self.scan(overrides={angle: 1.0 for angle in range(-2, 3)}))
        self.node.update_fused_objects()
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.DRIVE)

    def test_disappearing_target_before_maneuver_resumes_search_instead_of_finishing(self):
        self.start_driving()
        self.detect_obstacle()
        self.refresh_feeds()
        self.node.update_fused_objects()
        for _ in range(self.module.REQUIRED_CLEAR_SAMPLES):
            self.now += 0.01
            self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.DRIVE)
        self.assertEqual(self.node.motor_command(self.now), (self.module.CRUISE_RPM,) * 2)

    def test_front_lidar_hard_stop_does_not_require_yolo(self):
        self.start_driving()
        self.node.on_scan(self.scan(overrides={0: self.module.LIDAR_HARD_STOP_M}))
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_lidar_side_clearance_selects_available_side(self):
        self.start_driving()
        self.detect_obstacle()
        self.node.lidar_left_m = self.module.LIDAR_SIDE_CLEAR_M - 0.01
        self.node.lidar_right_m = self.module.LIDAR_SIDE_CLEAR_M + 0.1
        self.now += self.module.STOP_BEFORE_TURN_S + 0.01
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.TURN_RIGHT)

    def test_blocked_sides_hold_stop_before_avoidance(self):
        self.start_driving()
        self.detect_obstacle()
        self.node.lidar_left_m = self.node.lidar_right_m = self.module.LIDAR_SIDE_CLEAR_M - 0.01
        self.now += self.module.STOP_BEFORE_TURN_S + 0.01
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.WAIT_CLEAR)
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_side_obstacle_stops_active_turn(self):
        self.start_driving()
        self.detect_obstacle()
        self.now += self.module.STOP_BEFORE_TURN_S + 0.01
        self.node.update_state(self.now)
        self.node.lidar_left_m = self.module.LIDAR_SIDE_STOP_M
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_each_stale_required_feed_stops_commands(self):
        self.start_driving()
        for name in ("camera", "lidar", "stm32"):
            with self.subTest(feed=name):
                self.refresh_feeds()
                self.node.stamps[name] = self.now - self.module.SENSOR_TIMEOUT_S - 0.01
                self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_emergency_status_stops_commands_even_without_sharp(self):
        self.start_driving()
        self.status(emergency=1)
        self.assertTrue(self.node.stm32_emergency)
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_emergency_stays_latched_and_is_preserved_in_stop_command(self):
        self.start_driving()
        self.status(emergency=1)
        self.status(emergency=0)
        self.node.tick()
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.assertTrue(self.node.stm32_emergency)
        self.assertEqual(self.node.serial.writes[-1], b"$CMD,0,0,1\r\n")

    def test_invalid_status_cannot_satisfy_mcu_readiness(self):
        for payload in (
            b"$STATUS,0,123,30\r\n",
            b"$STATUS,0,123,30,0,0,broken,0,0,0,0\r\n",
            b"$STATUS,0,123,30,0,0,0,0,0,0,broken\r\n",
        ):
            with self.subTest(payload=payload):
                self.node.stamps["stm32"] = None
                self.node.serial.incoming.extend(payload)
                self.node.read_stm32()
                self.assertIsNone(self.node.stamps["stm32"])

    def test_partial_status_waits_for_complete_line(self):
        self.node.serial.incoming.extend(b"$STATUS,0,0,0,0,0,12,")
        self.node.read_stm32()
        self.assertIsNone(self.node.stamps["stm32"])
        self.node.serial.incoming.extend(b"13,0,0,0\r\n")
        self.node.read_stm32()
        self.assertEqual(self.node.left_wheel_rpm, 12.0)
        self.assertEqual(self.node.right_wheel_rpm, 13.0)
        self.assertEqual(self.node.stamps["stm32"], self.now)

    def test_scan_zero_to_360_normalizes_front_and_right_sectors(self):
        self.node.on_scan(self.scan(start_deg=0, overrides={359: 0.8, 315: 0.4, 45: 1.2}))
        self.assertEqual(self.node.lidar_front_m, 0.8)
        self.assertEqual(self.node.lidar_left_m, 1.2)
        self.assertEqual(self.node.lidar_right_m, 0.4)
        self.assertTrue(any(angle < 0 for angle, _distance in self.node.lidar_points))

    def test_scan_with_missing_sector_is_not_ready(self):
        self.node.on_scan(self.scan(start_deg=0, count=90))
        self.assertIsNone(self.node.stamps["lidar"])
        self.assertFalse(self.node.right_clear())

    def test_all_invalid_scan_invalidates_previous_fresh_feed(self):
        self.start_driving()
        for invalid in (float("nan"), 0.0):
            with self.subTest(invalid=invalid):
                self.node.on_scan(self.scan(distance=invalid))
                self.assertIsNone(self.node.stamps["lidar"])
                self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_positive_infinite_returns_mean_clear_only_in_covered_sectors(self):
        self.node.on_scan(self.scan(distance=float("inf")))
        self.assertEqual(self.node.lidar_front_m, 12.0)
        self.assertEqual(self.node.stamps["lidar"], self.now)
        self.node.on_scan(self.scan(start_deg=0, count=90, distance=float("inf")))
        self.assertIsNone(self.node.stamps["lidar"])

    def test_malformed_yolo_invalidates_fresh_camera_feed(self):
        self.start_driving()
        self.node.on_yolo(types.SimpleNamespace(data="not JSON"))
        self.assertIsNone(self.node.stamps["camera"])
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_post_avoidance_stale_feed_aborts_instead_of_false_completion(self):
        self.complete_avoidance()
        self.now += self.module.SENSOR_TIMEOUT_S + 0.1
        self.node.check_health(self.now)
        self.node.update_state(self.now)
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_stale_feed_abort_stays_stopped_after_feed_recovers(self):
        self.start_driving()
        self.now += self.module.SENSOR_TIMEOUT_S + 0.1
        self.node.tick()
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.assertEqual(self.node.serial.writes[-1], b"$CMD,0,0,0\r\n")
        self.refresh_feeds()
        self.node.tick()
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.assertEqual(self.node.motor_command(self.now), (0, 0))

    def test_startup_timeout_aborts_without_clearing_unknown_emergency(self):
        self.node.tick()
        self.assertEqual(self.node.serial.writes, [])
        self.now += self.module.STARTUP_TIMEOUT_S + 0.1
        self.node.tick()
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.node.shutdown()
        self.assertTrue(self.node.serial.writes)
        self.assertTrue(all(command == b"$CMD,0,0,1\r\n" for command in self.node.serial.writes))

    def test_no_wheel_progress_aborts_despite_fresh_healthy_sensors(self):
        self.start_driving()
        self.node.tick()
        self.now += self.module.NO_PROGRESS_TIMEOUT_S + 0.1
        self.refresh_feeds()
        self.node.tick()
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.assertIn("progress", self.node.abort_reason.lower())
        self.assertEqual(self.node.serial.writes[-1], b"$CMD,0,0,0\r\n")

    def test_runtime_limit_stops_forward_search_without_an_obstacle(self):
        self.start_driving()
        self.now += self.module.MAXIMUM_RUNTIME_S + 0.1
        self.refresh_feeds()
        self.node.tick()
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.assertIn("duration", self.node.abort_reason.lower())
        self.assertEqual(self.node.serial.writes[-1], b"$CMD,0,0,0\r\n")

    def test_preflight_completes_without_any_nonzero_motor_command(self):
        self.node.shutdown()
        self.node = self.module.YoloLidarAvoidance(preflight_only=True)
        self.addCleanup(self.node.shutdown)
        self.refresh_feeds()
        self.node.tick()
        self.assertEqual(self.node.state, self.node.COMPLETE)
        self.assertTrue(self.node.serial.writes)
        self.assertTrue(all(command == b"$CMD,0,0,0\r\n" for command in self.node.serial.writes))

    def test_tick_preserves_ascii_uart_format(self):
        self.start_driving()
        self.node.tick()
        self.assertEqual(self.node.serial.writes[-1], b"$CMD,20,20,0\r\n")

    def test_shutdown_sends_only_zero_commands_and_closes_serial(self):
        self.start_driving()
        self.node.serial.writes.clear()
        self.node.shutdown()
        self.assertTrue(self.node.serial.closed)
        self.assertGreaterEqual(len(self.node.serial.writes), 1)
        self.assertTrue(all(command == b"$CMD,0,0,0\r\n" for command in self.node.serial.writes))

    def test_sigterm_during_ros_poll_exits_without_another_motion_command(self):
        self.start_driving()
        handlers = {}

        def register(signum, handler):
            handlers[signum] = handler
            return lambda *_arguments: None

        def interrupt(*_arguments, **_kwargs):
            handlers[self.module.signal.SIGTERM](self.module.signal.SIGTERM, None)

        with mock.patch.object(self.module, "YoloLidarAvoidance", return_value=self.node), \
                mock.patch.object(self.module.signal, "signal", side_effect=register), \
                mock.patch.object(self.module.rclpy, "ok", return_value=True), \
                mock.patch.object(self.module.rclpy, "spin_once", side_effect=interrupt):
            result = self.module.main([])
        self.assertEqual(result, 1)
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.assertTrue(self.node.serial.closed)
        self.assertTrue(self.node.serial.writes)
        self.assertTrue(all(command == b"$CMD,0,0,0\r\n" for command in self.node.serial.writes))

    def test_unexpected_ros_error_sends_stop_and_closes_serial(self):
        self.start_driving()
        with mock.patch.object(self.module, "YoloLidarAvoidance", return_value=self.node), \
                mock.patch.object(self.module.signal, "signal", return_value=lambda *_args: None), \
                mock.patch.object(self.module.rclpy, "ok", return_value=True), \
                mock.patch.object(self.module.rclpy, "spin_once", side_effect=RuntimeError("ROS fault")):
            result = self.module.main([])
        self.assertEqual(result, 1)
        self.assertEqual(self.node.state, self.node.ABORTED)
        self.assertTrue(self.node.serial.closed)
        self.assertEqual(self.node.serial.writes[-1], b"$CMD,0,0,0\r\n")


if __name__ == "__main__":
    unittest.main()
