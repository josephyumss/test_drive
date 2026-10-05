import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, LaserScan, Range
from std_msgs.msg import Bool, String
from amr_interfaces.msg import CliffState, ObstacleInfo

from .imu_generator import ImuGenerator
from .lidar_generator import LidarGenerator
from .scenario_manager import ScenarioManager


class DummySensorNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_sensor_node")
        self.declare_parameter("scenario", "automatic")
        self.declare_parameter("publish_rate_hz", 20.0)
        scenario = str(self.get_parameter("scenario").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")

        self.manager = ScenarioManager(scenario)
        self.lidar = LidarGenerator()
        self.imu = ImuGenerator()
        self.start_ns = self.get_clock().now().nanoseconds

        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odometry", 10)
        self.tof_pub = self.create_publisher(Range, "/tof/downward", qos_profile_sensor_data)
        self.battery_pub = self.create_publisher(BatteryState, "/battery/state", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.obstacle_pub = self.create_publisher(ObstacleInfo, "/obstacle/info", 10)
        self.cliff_pub = self.create_publisher(CliffState, "/cliff/state", 10)
        self.active_pub = self.create_publisher(Bool, "/dummy/active", 10)
        self.hardware_enable_pub = self.create_publisher(Bool, "/hardware/motor_enable", 10)
        self.reset_pub = self.create_publisher(Bool, "/safety/reset_request", 10)
        self.scenario_pub = self.create_publisher(String, "/dummy/scenario_name", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_all)
        self.get_logger().warning("DUMMY MODE active: real motor hardware is disabled")

    def elapsed_s(self) -> float:
        return (self.get_clock().now().nanoseconds - self.start_ns) / 1_000_000_000.0

    def publish_all(self) -> None:
        state = self.manager.state_at(self.elapsed_s())
        stamp = self.get_clock().now().to_msg()

        active = Bool(data=True)
        disabled = Bool(data=False)
        scenario = String(data=state.name)
        self.active_pub.publish(active)
        self.hardware_enable_pub.publish(disabled)
        self.scenario_pub.publish(scenario)
        self.reset_pub.publish(Bool(data=state.reset_requested))

        command = Twist()
        command.linear.x = state.commanded_linear_mps
        command.angular.z = state.commanded_angular_rad_s
        self.cmd_pub.publish(command)

        self._publish_obstacle(state, stamp)
        self._publish_cliff(state, stamp)
        self._publish_tof(state, stamp)
        self._publish_battery(state, stamp)
        self._publish_odometry(state, stamp)
        if state.publish_lidar:
            self._publish_scan(state, stamp)
        if state.publish_imu:
            self._publish_imu(state, stamp)

    def _publish_scan(self, state, stamp) -> None:
        data = self.lidar.generate(
            state.obstacle_distance_m,
            state.obstacle_direction_deg,
            state.obstacle_width_deg,
            noise_stddev_m=0.005,
        )
        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = "laser_frame"
        message.angle_min = data.angle_min
        message.angle_max = data.angle_max
        message.angle_increment = data.angle_increment
        message.scan_time = 0.05
        message.range_min = data.range_min
        message.range_max = data.range_max
        message.ranges = list(data.ranges)
        self.scan_pub.publish(message)

    def _publish_imu(self, state, stamp) -> None:
        drift = state.gyro_bias_z_rad_s
        if state.name == "imu_drift":
            drift += 0.00005 * self.elapsed_s()
        data = self.imu.generate(
            commanded_angular_velocity_z=state.commanded_angular_rad_s,
            gyro_bias_z=drift,
            noise_stddev=state.imu_noise_stddev,
        )
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = "imu_link"
        message.orientation.w = 1.0
        message.angular_velocity.z = data.angular_velocity_z
        message.linear_acceleration.x = data.linear_acceleration_x
        message.linear_acceleration.y = data.linear_acceleration_y
        message.linear_acceleration.z = data.linear_acceleration_z
        message.orientation_covariance[0] = 0.02
        message.orientation_covariance[4] = 0.02
        message.orientation_covariance[8] = 0.04
        message.angular_velocity_covariance[8] = 0.02
        self.imu_pub.publish(message)

    def _publish_obstacle(self, state, stamp) -> None:
        detected = state.obstacle_distance_m is not None
        distance = state.obstacle_distance_m if detected else 10.0
        closing = state.obstacle_closing_speed_mps
        message = ObstacleInfo()
        message.stamp = stamp
        message.detected = detected
        message.object_class = "보행자" if detected else "없음"
        message.distance_m = float(distance)
        message.closing_speed_mps = float(closing)
        message.ttc_s = float(distance / closing) if detected and closing > 0 else math.inf
        message.direction = "전방"
        message.valid = True
        self.obstacle_pub.publish(message)

    def _publish_cliff(self, state, stamp) -> None:
        message = CliffState()
        message.stamp = stamp
        message.left_detected = state.cliff_left
        message.right_detected = state.cliff_right
        message.tof_danger = state.tof_distance_m > 0.4
        message.left_distance_m = state.tof_distance_m
        message.right_distance_m = 0.12
        message.sensor_fault = False
        message.valid = True
        self.cliff_pub.publish(message)

    def _publish_tof(self, state, stamp) -> None:
        message = Range()
        message.header.stamp = stamp
        message.header.frame_id = "downward_tof_link"
        message.radiation_type = Range.INFRARED
        message.field_of_view = math.radians(25.0)
        message.min_range = 0.03
        message.max_range = 2.0
        message.range = state.tof_distance_m
        self.tof_pub.publish(message)

    def _publish_battery(self, state, stamp) -> None:
        message = BatteryState()
        message.header.stamp = stamp
        message.voltage = state.battery_voltage_v
        message.percentage = state.battery_percent
        message.present = True
        self.battery_pub.publish(message)

    def _publish_odometry(self, state, stamp) -> None:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        wheel_velocity = state.actual_linear_mps * (1.0 + state.wheel_slip_ratio)
        message.twist.twist.linear.x = wheel_velocity
        message.twist.twist.angular.z = state.commanded_angular_rad_s
        self.odom_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummySensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
