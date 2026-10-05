import importlib.util
from pathlib import Path
import sys
import time
import types
import unittest
from unittest import mock


class FakeSerial:
    def __init__(self, *_args, **_kwargs):
        self.in_waiting = 0
        self.writes = []
        self.closed = False

    def reset_input_buffer(self):
        pass

    def read(self, _count):
        return b""

    def write(self, payload):
        self.writes.append(payload)

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
    gpio = types.ModuleType("Jetson.GPIO")
    gpio.BOARD = 10
    gpio.OUT = 1
    gpio.IN = 0
    gpio.LOW = 0
    gpio.HIGH = 1
    gpio.setwarnings = lambda *_args, **_kwargs: None
    gpio.setmode = lambda *_args, **_kwargs: None
    gpio.setup = lambda *_args, **_kwargs: None
    gpio.output = lambda *_args, **_kwargs: None
    gpio.input = lambda *_args, **_kwargs: gpio.LOW
    gpio.cleanup = lambda *_args, **_kwargs: None

    jetson_gpio = types.ModuleType("Jetson")
    jetson_gpio.__path__ = []
    jetson_gpio.GPIO = gpio

    serial = types.ModuleType("serial")
    serial.Serial = FakeSerial

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.ok = lambda: False
    rclpy.spin_once = lambda *_args, **_kwargs: None
    rclpy.shutdown = lambda: None
    rclpy.__path__ = []

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.__path__ = []
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = FakeMessage
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.__path__ = []
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = FakeMessage
    std_msgs.msg = std_msgs_msg

    stubs = {
        "Jetson": jetson_gpio,
        "Jetson.GPIO": gpio,
        "serial": serial,
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    script = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "test_fused_path_avoidance.py"
    )
    specification = importlib.util.spec_from_file_location(
        "fused_path_avoidance_under_test", script
    )
    module = importlib.util.module_from_spec(specification)
    with mock.patch.dict(sys.modules, stubs):
        specification.loader.exec_module(module)
    return module


class FusedPathAvoidanceNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_module()

    def setUp(self):
        with mock.patch.object(self.module.time, "sleep", return_value=None):
            self.node = self.module.IntegratedAvoidance()

    def tearDown(self):
        with mock.patch.object(self.module.time, "sleep", return_value=None):
            self.node.shutdown()

    def mark_sensors_fresh(self):
        now = time.monotonic()
        for name in self.node.stamps:
            self.node.stamps[name] = now
        self.node.sharp_cm = 60
        self.node.left_us_cm = 100.0
        self.node.right_us_cm = 100.0
        self.node.lidar_front_m = 3.0
        self.node.lidar_left_m = 2.0
        self.node.lidar_right_m = 2.0
        return now

    def make_target(self, track_id=1, distance=1.0):
        return self.module.FusedObject(
            track_id=track_id,
            label="person",
            confidence=0.9,
            confirmed_frames=3,
            distance_m=distance,
            forward_distance_m=distance,
            bearing_rad=0.0,
            bearing_min_rad=-0.05,
            bearing_max_rad=0.05,
            lateral_min_m=-0.05,
            lateral_max_m=0.05,
            intersects_corridor=True,
        )

    def test_full_avoidance_sequence_returns_to_original_path_state(self):
        self.mark_sensors_fresh()
        target = self.make_target()
        self.node.priority_target = target
        self.node.fused_objects = [target]
        started = 100.0

        self.node.update_state(started)
        self.assertEqual(self.node.state, self.node.STOPPING)
        self.node.update_state(started + self.module.STOP_BEFORE_TURN_S + 0.01)
        self.assertEqual(self.node.state, self.node.TURN_LEFT)

        entry_end_x, entry_end_y = self.node.entry_follower.path.p3
        self.node.odometry.x_m = entry_end_x
        self.node.odometry.y_m = entry_end_y
        self.node.odometry.yaw_rad = 0.0
        self.node.priority_target = None
        self.node.fused_objects = []
        self.node.update_state(started + 0.4)
        self.assertEqual(self.node.state, self.node.BYPASS)

        self.node.odometry.distance_travelled_m = (
            self.node.bypass_start_distance_m + self.module.BYPASS_DISTANCE_M + 0.01
        )
        self.node.update_state(started + 1.0)
        self.assertEqual(self.node.state, self.node.RETURN_PATH)
        return_end_x, return_end_y = self.node.return_follower.path.p3
        self.node.odometry.x_m = return_end_x
        self.node.odometry.y_m = return_end_y
        self.node.odometry.yaw_rad = 0.0
        self.node.update_state(started + 1.1)
        self.assertEqual(self.node.state, self.node.DRIVE)

    def test_new_target_preempts_return_to_path(self):
        self.mark_sensors_fresh()
        target = self.make_target(track_id=2, distance=0.8)
        self.node.state = self.node.RETURN_PATH
        self.node.priority_target = target
        self.node.fused_objects = [target]
        self.node.update_state(200.0)
        self.assertEqual(self.node.state, self.node.STOPPING)
        self.assertEqual(self.node.pending_target_track_id, 2)

    def test_same_planned_obstacle_does_not_restart_bypass(self):
        self.mark_sensors_fresh()
        target = self.make_target(track_id=1, distance=1.0)
        self.node.target_track_id = 1
        self.node.planned_obstacle_x_m = 1.0
        self.node.planned_obstacle_y_m = 0.0
        self.node.state = self.node.BYPASS
        self.node.bypass_start_distance_m = 0.0
        self.node.odometry.distance_travelled_m = 0.1
        self.node.fused_objects = [target]
        self.node.priority_target = target
        self.node.update_state(300.0)
        self.assertEqual(self.node.state, self.node.BYPASS)

    def test_sharp_is_an_independent_emergency_brake(self):
        now = self.mark_sensors_fresh()
        self.node.sharp_cm = self.module.SHARP_EMERGENCY_CM
        self.assertEqual(self.node.motor_command(now), (0, 0))

    def test_stale_sensor_is_fail_safe_stop(self):
        now = self.mark_sensors_fresh()
        self.node.stamps["camera"] = now - self.module.SENSOR_TIMEOUT_S - 0.1
        self.assertEqual(self.node.motor_command(now), (0, 0))

    def test_tick_preserves_existing_ascii_uart_command_format(self):
        self.mark_sensors_fresh()
        self.node.read_stm32 = lambda: None
        self.node.update_ultrasonic = lambda: None
        self.node.update_odometry = lambda _now: None
        self.node.update_fused_objects = lambda: None
        self.node.update_state = lambda _now: None
        self.node.tick()
        self.assertEqual(
            self.node.serial.writes[-1],
            b"$CMD,20,20,0\r\n",
        )


if __name__ == "__main__":
    unittest.main()
