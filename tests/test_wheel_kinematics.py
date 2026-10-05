import unittest

from jetson.amr_core.wheel_kinematics import twist_to_wheel_rpm


class WheelKinematicsTests(unittest.TestCase):
    def test_right_turn_commands_left_wheel_faster(self):
        left, right = twist_to_wheel_rpm(
            0.3, -0.4, wheel_diameter_m=0.2, wheel_base_m=0.5, maximum_rpm=300
        )
        self.assertGreater(left, right)

    def test_left_trim_compensates_left_drift(self):
        left, right = twist_to_wheel_rpm(
            0.3, 0.0, wheel_diameter_m=0.2, wheel_base_m=0.5,
            maximum_rpm=300, left_trim_rpm=2,
        )
        self.assertEqual(left, right + 2)

    def test_output_is_limited(self):
        self.assertEqual(
            twist_to_wheel_rpm(
                10.0, 0.0, wheel_diameter_m=0.2, wheel_base_m=0.5,
                maximum_rpm=100,
            ),
            (100, 100),
        )


if __name__ == "__main__":
    unittest.main()
