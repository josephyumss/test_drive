"""Receive Docker YOLO UDP results and expose them to ROS 2 safety nodes."""

from __future__ import annotations

import json
import math
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from amr_interfaces.msg import ObstacleInfo
from jetson.amr_core.range_fusion import sector_minimum
from jetson.amr_core.vision_detection import (
    decode_detection_packet,
    select_centered_person,
)


class YoloUdpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_udp_bridge_node")
        self.declare_parameter("bind_address", "127.0.0.1")
        self.declare_parameter("port", 5005)
        self.declare_parameter("receive_rate_hz", 50.0)
        self.declare_parameter("detection_timeout_s", 0.35)
        self.declare_parameter("lidar_timeout_s", 0.35)
        self.declare_parameter("lidar_front_half_angle_deg", 12.0)
        self.declare_parameter("person_confidence", 0.45)
        self.declare_parameter("person_center_min_ratio", 0.20)
        self.declare_parameter("person_center_max_ratio", 0.80)
        self.declare_parameter("person_minimum_height_ratio", 0.12)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind(
            (
                str(self.get_parameter("bind_address").value),
                int(self.get_parameter("port").value),
            )
        )
        self.latest_person: dict | None = None
        self.detection_stamp_s: float | None = None
        self.lidar_distance_m: float | None = None
        self.lidar_stamp_s: float | None = None
        self.detections_pub = self.create_publisher(String, "/yolo/detections", 10)
        self.person_pub = self.create_publisher(
            ObstacleInfo, "/vision/person_obstacle", 10
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        rate = float(self.get_parameter("receive_rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate), self._poll)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_scan(self, message: LaserScan) -> None:
        half = math.radians(
            float(self.get_parameter("lidar_front_half_angle_deg").value)
        )
        distance = sector_minimum(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            -half,
            half,
            message.range_min,
            message.range_max,
        )
        if distance is not None:
            self.lidar_distance_m = distance
            self.lidar_stamp_s = self._now_s()

    def _poll(self) -> None:
        newest = None
        while True:
            try:
                raw, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                newest = decode_detection_packet(raw)
            except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError):
                self.get_logger().warning(
                    "Invalid YOLO UDP packet ignored", throttle_duration_sec=2.0
                )
        if newest is not None:
            summary = String()
            summary.data = json.dumps(newest["detections"], ensure_ascii=False)
            self.detections_pub.publish(summary)
            self.latest_person = select_centered_person(
                newest,
                minimum_confidence=float(
                    self.get_parameter("person_confidence").value
                ),
                center_min_ratio=float(
                    self.get_parameter("person_center_min_ratio").value
                ),
                center_max_ratio=float(
                    self.get_parameter("person_center_max_ratio").value
                ),
                minimum_height_ratio=float(
                    self.get_parameter("person_minimum_height_ratio").value
                ),
            )
            self.detection_stamp_s = self._now_s()
        self._publish_person_obstacle()

    def _publish_person_obstacle(self) -> None:
        now = self._now_s()
        detection_fresh = (
            self.detection_stamp_s is not None
            and now - self.detection_stamp_s
            <= float(self.get_parameter("detection_timeout_s").value)
        )
        lidar_fresh = (
            self.lidar_stamp_s is not None
            and now - self.lidar_stamp_s
            <= float(self.get_parameter("lidar_timeout_s").value)
        )
        if not detection_fresh or not lidar_fresh or self.latest_person is None:
            return
        message = ObstacleInfo()
        message.stamp = self.get_clock().now().to_msg()
        message.detected = True
        message.object_class = "사람"
        message.distance_m = float(self.lidar_distance_m)
        message.closing_speed_mps = 0.0
        message.ttc_s = math.inf
        message.direction = "front"
        message.valid = True
        self.person_pub.publish(message)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloUdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
