import rclpy
import math

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32

from amr_interfaces.msg import McuStatus, SafetyState
from jetson.amr_core.ascii_serial_bridge import AsciiSerialBridge
from jetson.amr_core.transport import SerialTransport
from jetson.amr_core.wheel_kinematics import twist_to_wheel_rpm
from protocol.protocol_constants import DriveControlFlag, SystemState


class McuBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mcu_bridge_node")
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("status_timeout_s", 0.35)
        self.declare_parameter("velocity_input_timeout_s", 0.25)
        self.declare_parameter("safety_input_timeout_s", 0.35)
        self.declare_parameter("maximum_speed_mps", 0.03)
        self.declare_parameter("wheel_diameter_m", 0.20)
        self.declare_parameter("wheel_base_m", 0.50)
        self.declare_parameter("maximum_wheel_rpm", 3)
        self.declare_parameter("left_trim_rpm", 0)
        self.declare_parameter("right_trim_rpm", 0)
        self.declare_parameter("left_scale", 1.0)
        self.declare_parameter("right_scale", 1.0)
        self.declare_parameter("switch_mode", "hold")
        transport = SerialTransport(
            str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
        )
        self.bridge = AsciiSerialBridge(
            transport,
            status_timeout_s=float(self.get_parameter("status_timeout_s").value),
        )
        self.velocity = Twist()
        self.safety: SafetyState | None = None
        self.command_id = 0
        self.drive_requested = False
        self.previous_switch = False
        self.reset_requested = False
        self.velocity_stamp_s: float | None = None
        self.safety_stamp_s: float | None = None
        self.publisher = self.create_publisher(McuStatus, "/mcu/status", 10)
        self.ultrasonic_pub = self.create_publisher(Range, "/ultrasonic/front", 10)
        self.sharp_left_pub = self.create_publisher(Range, "/sharp/left", 10)
        self.sharp_right_pub = self.create_publisher(Range, "/sharp/right", 10)
        self.drive_request_pub = self.create_publisher(Bool, "/drive/request", 10)
        self.base_speed_pub = self.create_publisher(Float32, "/drive/base_speed_mps", 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._on_velocity, 10)
        self.create_subscription(SafetyState, "/safety/state", self._on_safety, 10)
        self.create_subscription(Bool, "/safety/reset_request", self._on_reset, 10)
        rate = float(self.get_parameter("command_rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)

    def _on_velocity(self, message: Twist) -> None:
        self.velocity = message
        self.velocity_stamp_s = self._now_s()

    def _on_safety(self, message: SafetyState) -> None:
        self.safety = message
        self.safety_stamp_s = self._now_s()

    def _on_reset(self, message: Bool) -> None:
        self.reset_requested = bool(message.data)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _wheel_command(self) -> tuple[int, int, bool]:
        self.command_id = (self.command_id + 1) & 0xFFFF
        flags = DriveControlFlag.NONE
        now = self._now_s()
        safety_fresh = (
            self.safety is not None
            and self.safety_stamp_s is not None
            and now - self.safety_stamp_s
            <= float(self.get_parameter("safety_input_timeout_s").value)
        )
        velocity_fresh = (
            self.velocity_stamp_s is not None
            and now - self.velocity_stamp_s
            <= float(self.get_parameter("velocity_input_timeout_s").value)
        )
        state = (
            SystemState(int(self.safety.state)) if safety_fresh else SystemState.INIT
        )
        if state in (SystemState.RUN, SystemState.SLOW):
            flags |= DriveControlFlag.DRIVE_ENABLE
        if state == SystemState.SLOW:
            flags |= DriveControlFlag.SLOW_MODE
        if state == SystemState.CONTROLLED_STOP:
            flags |= DriveControlFlag.CONTROLLED_STOP
        if self.reset_requested:
            flags |= DriveControlFlag.RESET_REQUEST
        enabled = bool(flags & DriveControlFlag.DRIVE_ENABLE) and velocity_fresh
        if not velocity_fresh:
            flags &= ~DriveControlFlag.DRIVE_ENABLE
        diameter = float(self.get_parameter("wheel_diameter_m").value)
        wheel_base = float(self.get_parameter("wheel_base_m").value)
        linear = float(self.velocity.linear.x) if enabled else 0.0
        angular = float(self.velocity.angular.z) if enabled else 0.0
        limit = int(self.get_parameter("maximum_wheel_rpm").value)
        left_rpm, right_rpm = twist_to_wheel_rpm(
            linear,
            angular,
            wheel_diameter_m=diameter,
            wheel_base_m=wheel_base,
            maximum_rpm=limit,
            left_trim_rpm=int(self.get_parameter("left_trim_rpm").value),
            right_trim_rpm=int(self.get_parameter("right_trim_rpm").value),
        )
        emergency = int(state in (SystemState.EMERGENCY_STOP, SystemState.FAULT))
        if enabled:
            left_rpm = round(left_rpm * float(self.get_parameter("left_scale").value))
            right_rpm = round(
                right_rpm * float(self.get_parameter("right_scale").value)
            )
            left_rpm = max(-limit, min(limit, left_rpm))
            right_rpm = max(-limit, min(limit, right_rpm))
        else:
            left_rpm = 0
            right_rpm = 0
        return left_rpm, right_rpm, bool(emergency)

    def _tick(self) -> None:
        try:
            left_rpm, right_rpm, emergency = self._wheel_command()
            self.bridge.send_wheel_command(left_rpm, right_rpm, emergency)
            self.reset_requested = False
            statuses = self.bridge.poll()
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f"MCU serial error: {error}", throttle_duration_sec=2.0)
            statuses = []
        diagnostics = self.bridge.diagnostics()
        if statuses:
            status = statuses[-1]
            message = McuStatus()
            message.stamp = self.get_clock().now().to_msg()
            message.system_state = int(
                SystemState.EMERGENCY_STOP if status.emergency else SystemState.RUN
            )
            message.safety_flags = 0
            diameter = float(self.get_parameter("wheel_diameter_m").value)
            circumference = math.pi * diameter
            message.left_velocity_mps = status.left_rpm * circumference / 60.0
            message.right_velocity_mps = status.right_rpm * circumference / 60.0
            message.battery_voltage_v = 0.0
            message.motor_error = 0
            message.last_command_id = self.command_id
            message.rx_error_count = diagnostics.ignored_packets
            message.uptime_ms = 0
            message.connected = not diagnostics.status_timed_out
            message.dummy = False
            message.ultrasonic_front_m = float("nan")
            message.sharp_left_m = status.sharp_distance_cm / 100.0
            message.sharp_right_m = float("nan")
            message.push_switch_pressed = status.base_rpm > 0
            message.requested_base_rpm = status.base_rpm
            message.left_velocity_rpm = status.left_rpm
            message.right_velocity_rpm = status.right_rpm
            message.left_target_rpm = status.left_target_rpm
            message.right_target_rpm = status.right_target_rpm
            message.sharp_adc = status.sharp_adc
            message.left_pwm = status.left_pwm
            message.right_pwm = status.right_pwm
            message.emergency_stop = status.emergency
            self.publisher.publish(message)
            self._update_drive_request(
                status.base_rpm > 0, status.base_rpm
            )
            self._publish_range(
                self.sharp_left_pub,
                "sharp_left_link",
                Range.INFRARED,
                status.sharp_distance_cm * 10,
                0.04,
                1.5,
            )
        elif diagnostics.status_timed_out:
            message = McuStatus()
            message.stamp = self.get_clock().now().to_msg()
            message.connected = False
            message.dummy = False
            self.publisher.publish(message)
            self.drive_requested = False
            self._publish_drive_request(0)

    def _update_drive_request(self, pressed: bool, requested_base_rpm: int) -> None:
        mode = str(self.get_parameter("switch_mode").value).lower()
        if mode == "toggle":
            if pressed and not self.previous_switch:
                self.drive_requested = not self.drive_requested
        else:
            self.drive_requested = pressed
        self.previous_switch = pressed
        self._publish_drive_request(requested_base_rpm)

    def _publish_drive_request(self, requested_base_rpm: int) -> None:
        request = Bool()
        request.data = self.drive_requested
        self.drive_request_pub.publish(request)
        speed = Float32()
        circumference = math.pi * float(self.get_parameter("wheel_diameter_m").value)
        requested_speed = (
            max(0, requested_base_rpm) * circumference / 60.0
            if self.drive_requested
            else 0.0
        )
        speed.data = min(
            requested_speed,
            float(self.get_parameter("maximum_speed_mps").value),
        )
        self.base_speed_pub.publish(speed)

    @staticmethod
    def _meters(distance_mm: int) -> float:
        return float("nan") if distance_mm == 0xFFFF else distance_mm / 1000.0

    def _publish_range(
        self,
        publisher,
        frame_id: str,
        radiation: int,
        distance_mm: int,
        minimum: float,
        maximum: float,
    ) -> None:
        if distance_mm == 0xFFFF:
            return
        message = Range()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.radiation_type = radiation
        message.field_of_view = 0.35
        message.min_range = minimum
        message.max_range = maximum
        message.range = distance_mm / 1000.0
        publisher.publish(message)

    def destroy_node(self):
        self.bridge.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
