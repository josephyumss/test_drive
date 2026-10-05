import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class CameraPathNode(Node):
    """Estimate normalized floor-path center error from the lower image ROI.

    This is a baseline for marked corridors/guide lines. It must be validated on
    the target floor; a missing observation is not interpreted as safe.
    """

    def __init__(self) -> None:
        super().__init__("camera_path_node")
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("output_topic", "/camera/path_error")
        self.declare_parameter("roi_top_ratio", 0.55)
        self.declare_parameter("minimum_line_length_px", 45)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Float32, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_image(self, message: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        height, width = image.shape[:2]
        top = int(height * float(self.get_parameter("roi_top_ratio").value))
        roi = image[top:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 60, 160)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            threshold=35,
            minLineLength=int(self.get_parameter("minimum_line_length_px").value),
            maxLineGap=35,
        )
        if lines is None:
            return
        left: list[float] = []
        right: list[float] = []
        sample_y = roi.shape[0] * 0.8
        for x1, y1, x2, y2 in lines[:, 0]:
            dy = float(y2 - y1)
            if abs(dy) < 10:
                continue
            slope = float(x2 - x1) / dy
            x_at_sample = x1 + slope * (sample_y - y1)
            if slope < -0.15:
                left.append(x_at_sample)
            elif slope > 0.15:
                right.append(x_at_sample)
        if not left or not right:
            return
        path_center = (float(np.median(left)) + float(np.median(right))) * 0.5
        # Positive means the perceived path center is to the left, requiring a left turn.
        error = (width * 0.5 - path_center) / (width * 0.5)
        output = Float32()
        output.data = float(max(-1.0, min(1.0, error)))
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
