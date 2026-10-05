"""Ultralytics YOLO ROS 2 node; the OAK driver remains the sole camera owner."""

import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("model", "yolo11n.pt")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("device", "0")
        self.declare_parameter("maximum_rate_hz", 15.0)
        from ultralytics import YOLO
        self.model = YOLO(str(self.get_parameter("model").value))
        self.bridge = CvBridge()
        self.last_run = 0.0
        self.annotated_pub = self.create_publisher(Image, "/yolo/annotated_image", 2)
        self.detections_pub = self.create_publisher(String, "/yolo/detections", 10)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self.on_image, qos_profile_sensor_data,
        )

    def on_image(self, message):
        now = time.monotonic()
        period = 1.0 / max(1.0, float(self.get_parameter("maximum_rate_hz").value))
        if now - self.last_run < period:
            return
        self.last_run = now
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        results = self.model.predict(
            frame, conf=float(self.get_parameter("confidence").value),
            device=str(self.get_parameter("device").value), verbose=False,
        )
        result = results[0]
        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0])
                detections.append({
                    "class": result.names[class_id],
                    "confidence": round(float(box.conf[0]), 3),
                    "xyxy": [round(float(value), 1) for value in box.xyxy[0]],
                })
        annotated = result.plot()
        image = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        image.header = message.header
        self.annotated_pub.publish(image)
        summary = String()
        summary.data = json.dumps(detections, ensure_ascii=False)
        self.detections_pub.publish(summary)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
