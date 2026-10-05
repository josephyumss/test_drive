import unittest

from jetson.amr_core.path_guidance import GuidanceConfig, GuidanceInputs, PathGuidanceController


class PathGuidanceTests(unittest.TestCase):
    def test_no_error_commands_straight_motion(self) -> None:
        controller = PathGuidanceController(GuidanceConfig(cruise_speed_mps=0.25))
        command = controller.update(GuidanceInputs(camera_lateral_error=0.0))
        self.assertEqual(command.linear_mps, 0.25)
        self.assertEqual(command.angular_rad_s, 0.0)

    def test_encoder_mismatch_applies_opposite_trim(self) -> None:
        controller = PathGuidanceController()
        command = controller.update(
            GuidanceInputs(left_velocity_mps=0.2, right_velocity_mps=0.3)
        )
        self.assertLess(command.angular_rad_s, 0.0)


if __name__ == "__main__":
    unittest.main()
