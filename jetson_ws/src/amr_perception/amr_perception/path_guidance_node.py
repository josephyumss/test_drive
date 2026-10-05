import math
import statistics

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from amr_interfaces.msg import McuStatus
from jetson.amr_core.path_guidance import (
    GuidanceConfig,
    GuidanceInputs,
    PathGuidanceController,
)


class PathGuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("path_guidance_node")
        defaults = {
            "enabled": False,
            "cruise_speed_mps": 0.25,
            "maximum_angular_rad_s": 0.55,
            "camera_gain": 0.55,
            "lidar_center_gain": 0.75,
            "lidar_heading_gain": 0.60,
            "encoder_gain": 0.50,
            "command_timeout_s": 0.4,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.controller = PathGuidanceController(
            GuidanceConfig(
                cruise_speed_mps=float(self.get_parameter("cruise_speed_mps").value),
                maximum_angular_rad_s=float(self.get_parameter("maximum_angular_rad_s").value),
                camera_gain=float(self.get_parameter("camera_gain").value),
                lidar_center_gain=float(self.get_parameter("lidar_center_gain").value),
                lidar_heading_gain=float(self.get_parameter("lidar_heading_gain").value),
                encoder_gain=float(self.get_parameter("encoder_gain").value),
            )
        )
        self.camera_error: float | None = None
        self.camera_stamp = 0.0
        self.lidar_center: float | None = None
        self.lidar_heading: float | None = None
        self.lidar_stamp = 0.0
        self.left_velocity: float | None = None
        self.right_velocity: float | None = None
        self.requested_speed_mps: float | None = None
        self.publisher = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.create_subscription(Float32, "/camera/path_error", self._on_camera, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(McuStatus, "/mcu/status", self._on_mcu, 10)
        self.create_subscription(Float32, "/drive/base_speed_mps", self._on_base_speed, 10)
        self.create_timer(0.05, self._tick)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_camera(self, message: Float32) -> None:
        self.camera_error = float(message.data)
        self.camera_stamp = self.now_s()

    @staticmethod
    def _sector(message: LaserScan, low: float, high: float) -> float | None:
        values = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if low <= angle <= high and math.isfinite(value):
                if message.range_min <= value <= message.range_max:
                    values.append(float(value))
        return statistics.median(values) if values else None

    def _on_scan(self, message: LaserScan) -> None:
        near_left = self._sector(message, math.radians(55), math.radians(85))
        near_right = self._sector(message, math.radians(-85), math.radians(-55))
        far_left = self._sector(message, math.radians(25), math.radians(45))
        far_right = self._sector(message, math.radians(-45), math.radians(-25))
        if near_left is not None and near_right is not None:
            self.lidar_center = (near_left - near_right) * 0.5
            if far_left is not None and far_right is not None:
                self.lidar_heading = math.atan2(
                    (far_left - far_right) - (near_left - near_right), 1.0
                )
            self.lidar_stamp = self.now_s()

    def _on_mcu(self, message: McuStatus) -> None:
        if message.connected:
            self.left_velocity = float(message.left_velocity_mps)
            self.right_velocity = float(message.right_velocity_mps)

    def _on_base_speed(self, message: Float32) -> None:
        self.requested_speed_mps = max(0.0, float(message.data))

    def _tick(self) -> None:
        message = Twist()
        if not bool(self.get_parameter("enabled").value):
            return
        now = self.now_s()
        timeout = float(self.get_parameter("command_timeout_s").value)
        inputs = GuidanceInputs(
            camera_lateral_error=(self.camera_error if now - self.camera_stamp <= timeout else None),
            lidar_lateral_error_m=(self.lidar_center if now - self.lidar_stamp <= timeout else None),
            lidar_heading_error_rad=(self.lidar_heading if now - self.lidar_stamp <= timeout else None),
            left_velocity_mps=self.left_velocity,
            right_velocity_mps=self.right_velocity,
        )
        command = self.controller.update(inputs)
        # Do not move unless at least one environmental guidance source is fresh.
        if inputs.camera_lateral_error is None and inputs.lidar_lateral_error_m is None:
            self.publisher.publish(message)
            return
        message.linear.x = (
            self.requested_speed_mps
            if self.requested_speed_mps is not None
            else command.linear_mps
        )
        message.angular.z = command.angular_rad_s
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
