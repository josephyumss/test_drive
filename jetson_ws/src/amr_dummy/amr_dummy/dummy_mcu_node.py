import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from amr_interfaces.msg import CliffState, McuStatus, SafetyState

from jetson.amr_core.packet import WheelCommand
from jetson.amr_core.serial_bridge import SerialBridge
from jetson.amr_core.transport import memory_transport_pair
from jetson.amr_core.wheel_kinematics import twist_to_wheel_rpm
from protocol.protocol_constants import DriveControlFlag, SystemState
from simulation.fake_stm32 import FakeSTM32


class DummyMcuNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_mcu_node")
        self.declare_parameter("update_rate_hz", 20.0)
        rate = float(self.get_parameter("update_rate_hz").value)
        jetson_link, stm32_link = memory_transport_pair()
        self.bridge = SerialBridge(jetson_link)
        self.mcu = FakeSTM32(stm32_link)
        self.safe_cmd = Twist()
        self.safety_state = SystemState.READY
        self.reset_requested = False
        self.suppress_status = False
        self.command_id = 0
        self.last_ns = self.get_clock().now().nanoseconds
        self.start_ns = self.last_ns

        self.create_subscription(Twist, "/cmd_vel_safe", self._on_velocity, 10)
        self.create_subscription(SafetyState, "/safety/state", self._on_safety, 10)
        self.create_subscription(CliffState, "/cliff/state", self._on_cliff, 10)
        self.create_subscription(Bool, "/safety/reset_request", self._on_reset, 10)
        self.create_subscription(String, "/dummy/scenario_name", self._on_scenario, 10)
        self.status_pub = self.create_publisher(McuStatus, "/mcu/status", 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _on_velocity(self, message: Twist) -> None:
        self.safe_cmd = message

    def _on_safety(self, message: SafetyState) -> None:
        self.safety_state = SystemState(message.state)

    def _on_cliff(self, message: CliffState) -> None:
        self.mcu.hazards.cliff_left = message.left_detected or message.tof_danger
        self.mcu.hazards.cliff_right = message.right_detected
        self.mcu.hazards.motor_fault = message.sensor_fault

    def _on_reset(self, message: Bool) -> None:
        self.reset_requested = bool(message.data)

    def _on_scenario(self, message: String) -> None:
        self.suppress_status = message.data == "communication_loss"

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        now_s = (now_ns - self.start_ns) / 1_000_000_000.0
        dt_s = max(0.0, (now_ns - self.last_ns) / 1_000_000_000.0)
        self.last_ns = now_ns
        self.command_id = (self.command_id + 1) & 0xFFFF
        flags = DriveControlFlag.NONE
        if self.reset_requested:
            flags |= DriveControlFlag.RESET_REQUEST
            self.reset_requested = False
        elif self.safety_state in (SystemState.RUN, SystemState.SLOW):
            flags |= DriveControlFlag.DRIVE_ENABLE
            if self.safety_state == SystemState.SLOW:
                flags |= DriveControlFlag.SLOW_MODE
        elif self.safety_state == SystemState.CONTROLLED_STOP:
            flags |= DriveControlFlag.CONTROLLED_STOP

        left_rpm, right_rpm = twist_to_wheel_rpm(
            self.safe_cmd.linear.x,
            self.safe_cmd.angular.z,
            wheel_diameter_m=0.2,
            wheel_base_m=0.5,
            maximum_rpm=300,
        )
        command = WheelCommand(
            self.command_id, left_rpm, right_rpm, int(flags),
            int(self.safety_state in (SystemState.EMERGENCY_STOP, SystemState.FAULT)),
        )
        self.bridge.send_wheel_command(command)
        status = self.mcu.step(now_s, dt_s)
        self.bridge.poll(now_s=now_s)

        message = McuStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.system_state = int(status.system_state)
        message.safety_flags = int(status.safety_flags)
        message.left_velocity_mps = status.left_velocity_mm_s / 1000.0
        message.right_velocity_mps = status.right_velocity_mm_s / 1000.0
        message.battery_voltage_v = status.battery_voltage_mv / 1000.0
        message.motor_error = status.motor_error
        message.last_command_id = status.last_command_id
        message.rx_error_count = status.rx_error_count
        message.uptime_ms = status.uptime_ms
        message.connected = True
        message.dummy = True
        message.push_switch_pressed = status.push_switch_pressed
        message.requested_base_rpm = status.requested_base_rpm
        message.left_velocity_rpm = status.left_velocity_rpm
        message.right_velocity_rpm = status.right_velocity_rpm
        if not self.suppress_status:
            self.status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyMcuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
