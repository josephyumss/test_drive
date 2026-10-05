import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from amr_interfaces.msg import CliffState, McuStatus, ObstacleInfo, SafetyState

from jetson.amr_core.controller import IntegratedController

from .input_mapper import NormalizedInputs, to_controller_inputs
from .sensor_timeout_monitor import SensorTimeoutMonitor


class SafetyControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_controller_node")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("obstacle_timeout_s", 0.3)
        self.declare_parameter("cliff_timeout_s", 0.15)
        self.declare_parameter("mcu_timeout_s", 0.3)
        self.declare_parameter("drive_enable", True)
        rate = float(self.get_parameter("control_rate_hz").value)
        self.controller = IntegratedController()
        self.timeout = SensorTimeoutMonitor(
            {
                "obstacle": float(self.get_parameter("obstacle_timeout_s").value),
                "cliff": float(self.get_parameter("cliff_timeout_s").value),
                "mcu": float(self.get_parameter("mcu_timeout_s").value),
            }
        )
        self.command = Twist()
        self.obstacle = None
        self.cliff = None
        self.mcu = None
        self.dummy_active = False
        self.reset_requested = False
        self.drive_requested = False

        self.create_subscription(Twist, "/cmd_vel_raw", self._on_command, 10)
        self.create_subscription(ObstacleInfo, "/obstacle/info", self._on_obstacle, 10)
        self.create_subscription(CliffState, "/cliff/state", self._on_cliff, 10)
        self.create_subscription(McuStatus, "/mcu/status", self._on_mcu, 10)
        self.create_subscription(Bool, "/dummy/active", self._on_dummy, 10)
        self.create_subscription(Bool, "/safety/reset_request", self._on_reset, 10)
        self.create_subscription(Bool, "/drive/request", self._on_drive_request, 10)
        self.velocity_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.state_pub = self.create_publisher(SafetyState, "/safety/state", 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_command(self, message: Twist) -> None:
        self.command = message

    def _on_obstacle(self, message: ObstacleInfo) -> None:
        self.obstacle = message
        self.timeout.update("obstacle", self.now_s())

    def _on_cliff(self, message: CliffState) -> None:
        self.cliff = message
        self.timeout.update("cliff", self.now_s())

    def _on_mcu(self, message: McuStatus) -> None:
        self.mcu = message
        self.timeout.update("mcu", self.now_s())

    def _on_dummy(self, message: Bool) -> None:
        self.dummy_active = bool(message.data)

    def _on_reset(self, message: Bool) -> None:
        self.reset_requested = bool(message.data)

    def _on_drive_request(self, message: Bool) -> None:
        self.drive_requested = bool(message.data)

    def _tick(self) -> None:
        now = self.now_s()
        initialized = self.timeout.all_seen("obstacle", "cliff", "mcu")
        values = NormalizedInputs(
            requested_linear_mps=self.command.linear.x,
            requested_angular_rad_s=self.command.angular.z,
            obstacle_distance_m=(self.obstacle.distance_m if self.obstacle else 10.0),
            obstacle_closing_speed_mps=(
                self.obstacle.closing_speed_mps if self.obstacle else 0.0
            ),
            obstacle_valid=bool(self.obstacle and self.obstacle.valid),
            cliff_left=bool(self.cliff and self.cliff.left_detected),
            cliff_right=bool(self.cliff and self.cliff.right_detected),
            cliff_sensor_fault=bool(self.cliff and self.cliff.sensor_fault),
            mcu_connected=bool(self.mcu and self.mcu.connected),
            motor_fault=bool(self.mcu and self.mcu.motor_error),
            initialization_complete=initialized,
            reset_requested=self.reset_requested,
            drive_enable=(
                bool(self.get_parameter("drive_enable").value)
                and self.drive_requested
                and not self.reset_requested
            ),
            obstacle_timed_out=self.timeout.is_timed_out("obstacle", now),
            cliff_timed_out=self.timeout.is_timed_out("cliff", now),
            mcu_timed_out=self.timeout.is_timed_out("mcu", now),
        )
        output = self.controller.update(to_controller_inputs(values))
        self.reset_requested = False

        velocity = Twist()
        velocity.linear.x = output.safe_velocity.linear_mps
        velocity.angular.z = output.safe_velocity.angular_rad_s
        self.velocity_pub.publish(velocity)

        state = SafetyState()
        state.stamp = self.get_clock().now().to_msg()
        state.state = int(output.safety.state)
        state.safety_flags = int(output.safety.flags)
        state.reason = output.safety.reason
        state.speed_limit_mps = abs(output.safe_velocity.linear_mps)
        state.restart_latched = output.safety.restart_latched
        state.dummy_active = self.dummy_active
        self.state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
