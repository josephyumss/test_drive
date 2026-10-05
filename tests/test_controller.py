import unittest

from protocol.protocol_constants import DriveControlFlag, SystemState
from jetson.amr_core.controller import ControllerInputs, IntegratedController
from jetson.amr_core.velocity import VelocityCommand


class IntegratedControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = IntegratedController()

    def test_normal_input_generates_enabled_drive_command(self) -> None:
        output = self.controller.update(
            ControllerInputs(
                requested_velocity=VelocityCommand(0.4, 0.2),
                obstacle_distance_m=3.0,
                drive_enable=True,
            )
        )
        self.assertEqual(output.safety.state, SystemState.RUN)
        self.assertEqual(output.drive_command.linear_velocity_mm_s, 400)
        self.assertTrue(
            DriveControlFlag(output.drive_command.control_flags)
            & DriveControlFlag.DRIVE_ENABLE
        )

    def test_obstacle_slow_limits_velocity(self) -> None:
        output = self.controller.update(
            ControllerInputs(
                requested_velocity=VelocityCommand(0.5, 0.8),
                obstacle_distance_m=1.6,
                obstacle_closing_speed_mps=0.4,
                drive_enable=True,
            )
        )
        self.assertEqual(output.safety.state, SystemState.SLOW)
        self.assertEqual(output.safe_velocity, VelocityCommand(0.25, 0.4))
        flags = DriveControlFlag(output.drive_command.control_flags)
        self.assertTrue(flags & DriveControlFlag.DRIVE_ENABLE)
        self.assertTrue(flags & DriveControlFlag.SLOW_MODE)

    def test_stop_generates_zero_and_latches(self) -> None:
        stopped = self.controller.update(
            ControllerInputs(
                requested_velocity=VelocityCommand(0.5, 0.0),
                obstacle_distance_m=1.0,
                drive_enable=True,
            )
        )
        self.assertEqual(stopped.safety.state, SystemState.CONTROLLED_STOP)
        self.assertEqual(stopped.safe_velocity, VelocityCommand())

        still_stopped = self.controller.update(
            ControllerInputs(
                requested_velocity=VelocityCommand(0.5, 0.0),
                obstacle_distance_m=3.0,
                drive_enable=True,
            )
        )
        self.assertEqual(still_stopped.safety.state, SystemState.CONTROLLED_STOP)

    def test_reset_returns_ready_and_sets_reset_flag(self) -> None:
        self.controller.update(
            ControllerInputs(obstacle_distance_m=1.0, drive_enable=True)
        )
        reset = self.controller.update(
            ControllerInputs(
                obstacle_distance_m=3.0,
                reset_requested=True,
                drive_enable=False,
            )
        )
        self.assertEqual(reset.safety.state, SystemState.READY)
        self.assertTrue(
            DriveControlFlag(reset.drive_command.control_flags)
            & DriveControlFlag.RESET_REQUEST
        )


if __name__ == "__main__":
    unittest.main()
