"""Send a YAML waypoint mission to Nav2 FollowWaypoints."""

import math
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node


class WaypointMissionNode(Node):
    def __init__(self):
        super().__init__("waypoint_mission_node")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("autostart", False)
        self.client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        if bool(self.get_parameter("autostart").value):
            self.create_timer(2.0, self.start_once)
        self.started = False

    def start_once(self):
        if self.started:
            return
        path = str(self.get_parameter("waypoint_file").value)
        if not path:
            self.get_logger().error("waypoint_file is empty")
            self.started = True
            return
        with open(path, encoding="utf-8") as stream:
            points = yaml.safe_load(stream).get("waypoints", [])
        goal = FollowWaypoints.Goal()
        now = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            yaw = float(point.get("yaw", 0.0))
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            goal.poses.append(pose)
        self.started = True
        if not goal.poses:
            self.get_logger().error("no waypoints in mission file")
            return
        self.client.wait_for_server()
        self.client.send_goal_async(goal)
        self.get_logger().info(f"sent {len(goal.poses)} waypoints")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
