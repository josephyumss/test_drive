import math
import unittest

from jetson.amr_core.odometry import integrate_pose


class WheelOdometryTests(unittest.TestCase):
    def test_straight(self):
        x, y, yaw, linear, angular = integrate_pose(0, 0, 0, 1, 1, 0.5, 1)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, 0.0)
        self.assertAlmostEqual(linear, 1.0)
        self.assertAlmostEqual(angular, 0.0)

    def test_left_arc(self):
        _, y, yaw, _, angular = integrate_pose(0, 0, 0, 0.2, 0.4, 0.5, 1)
        self.assertGreater(y, 0.0)
        self.assertAlmostEqual(yaw, 0.4)
        self.assertAlmostEqual(angular, 0.4)

    def test_yaw_wraps(self):
        *_, yaw, __, ___ = integrate_pose(0, 0, math.pi - 0.1, 0, 1, 0.5, 1)
        self.assertTrue(-math.pi <= yaw <= math.pi)


if __name__ == "__main__":
    unittest.main()
