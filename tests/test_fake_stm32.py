import unittest

from protocol.protocol_constants import DriveControlFlag, SafetyFlag, SystemState
from jetson.amr_core.packet import DriveCommand, WheelCommand
from jetson.amr_core.serial_bridge import SerialBridge
from jetson.amr_core.transport import memory_transport_pair
from simulation.fake_stm32 import FakeSTM32


class FakeSTM32Tests(unittest.TestCase):
    def setUp(self) -> None:
        jetson_link, stm32_link = memory_transport_pair()
        self.bridge = SerialBridge(jetson_link)
        self.stm32 = FakeSTM32(stm32_link, acceleration_mps2=10.0)

    def send_drive(
        self,
        now_s: float,
        *,
        speed_mm_s: int = 300,
        flags: DriveControlFlag = DriveControlFlag.DRIVE_ENABLE,
    ):
        self.bridge.send_drive_command(
            DriveCommand(
                command_id=1,
                linear_velocity_mm_s=speed_mm_s,
                angular_velocity_mrad_s=0,
                speed_limit_mm_s=500,
                control_flags=int(flags),
            )
        )
        status = self.stm32.step(now_s, 0.1)
        self.bridge.poll(now_s=now_s)
        return status

    def test_drive_command_moves_simulated_robot(self) -> None:
        status = self.send_drive(0.0)
        self.assertEqual(status.system_state, SystemState.RUN)
        self.assertEqual(status.left_velocity_mm_s, 300)
        self.assertEqual(status.right_velocity_mm_s, 300)

    def test_command_timeout_stops_and_latches(self) -> None:
        self.send_drive(0.0)
        status = self.stm32.step(0.51, 0.1)
        self.assertEqual(status.system_state, SystemState.EMERGENCY_STOP)
        self.assertTrue(status.safety_flags & SafetyFlag.COMM_TIMEOUT)
        self.assertEqual(status.left_velocity_mm_s, 0)

        status = self.send_drive(0.52)
        self.assertEqual(status.system_state, SystemState.EMERGENCY_STOP)

    def test_reset_required_after_timeout(self) -> None:
        self.send_drive(0.0)
        self.stm32.step(0.51, 0.1)
        reset = self.send_drive(
            0.52,
            speed_mm_s=0,
            flags=DriveControlFlag.RESET_REQUEST,
        )
        self.assertEqual(reset.system_state, SystemState.READY)
        running = self.send_drive(0.62)
        self.assertEqual(running.system_state, SystemState.RUN)

    def test_wheel_command_accepts_independent_rpm(self) -> None:
        self.bridge.send_wheel_command(
            WheelCommand(7, 40, 20, int(DriveControlFlag.DRIVE_ENABLE), 0)
        )
        status = self.stm32.step(0.0, 0.1)
        self.assertEqual(status.system_state, SystemState.RUN)
        self.assertGreater(status.left_velocity_rpm, status.right_velocity_rpm)

    def test_cliff_overrides_drive_command(self) -> None:
        self.stm32.hazards.cliff_left = True
        status = self.send_drive(0.0)
        self.assertEqual(status.system_state, SystemState.EMERGENCY_STOP)
        self.assertTrue(status.safety_flags & SafetyFlag.CLIFF_LEFT)
        self.assertEqual(status.left_velocity_mm_s, 0)

    def test_motor_fault_has_fault_state(self) -> None:
        self.stm32.hazards.motor_fault = True
        status = self.send_drive(0.0)
        self.assertEqual(status.system_state, SystemState.FAULT)
        self.assertTrue(status.safety_flags & SafetyFlag.MOTOR_FAULT)


if __name__ == "__main__":
    unittest.main()
