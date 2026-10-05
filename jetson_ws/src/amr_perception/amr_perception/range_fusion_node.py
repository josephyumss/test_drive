import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

from amr_interfaces.msg import ObstacleInfo
from jetson.amr_core.range_fusion import RangeFusion, RangeReading, sector_minimum


class RangeFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("range_fusion_node")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("sensor_timeout_s", 0.35)
        self.declare_parameter("lidar_front_half_angle_deg", 25.0)
        self.fusion = RangeFusion(
            timeout_s=float(self.get_parameter("sensor_timeout_s").value)
        )
        self.publisher = self.create_publisher(ObstacleInfo, "/obstacle/info", 10)
        self.create_subscription(
            Range, "/ultrasonic/front", self._on_ultrasonic, qos_profile_sensor_data
        )
        self.create_subscription(
            Range, "/sharp/left", self._on_sharp_left, qos_profile_sensor_data
        )
        self.create_subscription(
            Range, "/sharp/right", self._on_sharp_right, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            ObstacleInfo,
            "/vision/person_obstacle",
            self._on_person_obstacle,
            10,
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _update_range(self, source: str, message: Range) -> None:
        valid = math.isfinite(message.range) and message.min_range <= message.range <= message.max_range
        self.fusion.update(RangeReading(source, float(message.range), self.now_s(), valid))

    def _on_ultrasonic(self, message: Range) -> None:
        self._update_range("ultrasonic_front", message)

    def _on_sharp_left(self, message: Range) -> None:
        self._update_range("sharp_left", message)

    def _on_sharp_right(self, message: Range) -> None:
        self._update_range("sharp_right", message)

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
            self.fusion.update(RangeReading("lidar_front", distance, self.now_s()))

    def _on_person_obstacle(self, message: ObstacleInfo) -> None:
        if message.valid and message.detected and math.isfinite(message.distance_m):
            self.fusion.update(
                RangeReading("camera_person", float(message.distance_m), self.now_s())
            )

    def _publish(self) -> None:
        result = self.fusion.result(self.now_s())
        message = ObstacleInfo()
        message.stamp = self.get_clock().now().to_msg()
        message.detected = result.valid
        labels = {"camera_person": "사람"}
        message.object_class = labels.get(result.source, result.source) if result.valid else "none"
        message.distance_m = float(result.distance_m if result.valid else 10.0)
        message.closing_speed_mps = float(result.closing_speed_mps)
        message.ttc_s = (
            float(result.distance_m / result.closing_speed_mps)
            if result.valid and result.closing_speed_mps > 0
            else math.inf
        )
        message.direction = "front"
        message.valid = result.valid
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RangeFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
