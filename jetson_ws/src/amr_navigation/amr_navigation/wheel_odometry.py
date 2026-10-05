"""Publish differential-drive odometry from MCU wheel feedback."""

import math

import rclpy
from amr_interfaces.msg import McuStatus
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from jetson.amr_core.odometry import integrate_pose


def yaw_quaternion(yaw):
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry_node")
        self.declare_parameter("wheel_base_m", 0.50)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_frame", "wheel_odom")
        self.declare_parameter("base_frame", "base_link")
        self.x = self.y = self.yaw = 0.0
        self.last_stamp_ns = None
        self.publisher = self.create_publisher(Odometry, "/wheel/odometry", 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(McuStatus, "/mcu/status", self.on_status, 20)

    def on_status(self, status):
        if not status.connected:
            self.last_stamp_ns = None
            return
        now = self.get_clock().now()
        stamp_ns = now.nanoseconds
        if self.last_stamp_ns is None:
            self.last_stamp_ns = stamp_ns
            return
        dt = min(0.25, max(0.0, (stamp_ns - self.last_stamp_ns) / 1e9))
        self.last_stamp_ns = stamp_ns
        self.x, self.y, self.yaw, linear, angular = integrate_pose(
            self.x, self.y, self.yaw,
            status.left_velocity_mps, status.right_velocity_mps,
            float(self.get_parameter("wheel_base_m").value), dt,
        )
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = str(self.get_parameter("odom_frame").value)
        msg.child_frame_id = str(self.get_parameter("base_frame").value)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = yaw_quaternion(self.yaw)
        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        msg.twist.covariance[0] = msg.twist.covariance[7] = 0.03
        msg.twist.covariance[35] = 0.06
        self.publisher.publish(msg)
        if bool(self.get_parameter("publish_tf").value):
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
