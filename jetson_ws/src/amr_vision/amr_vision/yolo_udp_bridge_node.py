"""Receive Docker YOLO UDP results and expose them to ROS 2 safety nodes."""

from __future__ import annotations

import json
import math
import socket
import time
import traceback

import rclpy
from rclpy.executors import ExternalShutdownException
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
        self.received_packets = self.valid_packets = self.invalid_packets = self.published_packets = 0
        self.last_udp_s = self.last_valid_s = self.last_frame_id = self.last_source_unix_s = None
        self.last_health_s = self.last_invalid_log_s = -math.inf
        self.duplicate_frame_ids = 0
        self.get_logger().info(f"UDP bridge listening on {self.socket.getsockname()} source={__file__}")
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
                raw, sender = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            self.received_packets += 1
            self.last_udp_s = time.monotonic()
            try:
                candidate = decode_detection_packet(raw)
                if any(not isinstance(d, dict) for d in candidate["detections"]):
                    raise ValueError("Detection entries must be objects")
                newest = candidate
                self.valid_packets += 1
                self.last_valid_s = time.monotonic()
            except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError, AttributeError) as exc:
                self.invalid_packets += 1
                if time.monotonic() - self.last_invalid_log_s >= 2:
                    self.get_logger().warning(json.dumps({"event": "invalid_UDP", "error": str(exc),
                        "sender": sender, "bytes": len(raw), "hex_prefix": raw[:2048].hex(),
                        "truncated": len(raw) > 2048, "invalid_count": self.invalid_packets}))
                    self.last_invalid_log_s = time.monotonic()
        if newest is not None:
            summary = String()
            summary.data = json.dumps(newest["detections"], ensure_ascii=False)
            self.detections_pub.publish(summary)
            self.published_packets += 1
            frame_id = newest.get("frame_id")
            if frame_id is not None and frame_id == self.last_frame_id:
                self.duplicate_frame_ids += 1
            self.last_frame_id = frame_id
            self.last_source_unix_s = newest.get("timestamp")
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
        now = time.monotonic()
        if now - self.last_health_s >= 5:
            self.get_logger().info(json.dumps({"event": "bridge_health", "monotonic_s": now,
                "received": self.received_packets, "valid": self.valid_packets, "invalid": self.invalid_packets,
                "published": self.published_packets, "duplicate_frame_ids": self.duplicate_frame_ids,
                "last_frame_id": self.last_frame_id, "last_source_unix_s": self.last_source_unix_s,
                "UDP_age_s": None if self.last_udp_s is None else now - self.last_udp_s,
                "valid_age_s": None if self.last_valid_s is None else now - self.last_valid_s}))
            self.last_health_s = now
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
    node = None
    try:
        node = YoloUdpBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        print(json.dumps({"event": "bridge_exception", "traceback": traceback.format_exc()}), flush=True)
        raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            # SIGTERM may already have shut down the ROS context. Normal
            # launcher cleanup must not produce a second shutdown traceback.
            rclpy.try_shutdown()
