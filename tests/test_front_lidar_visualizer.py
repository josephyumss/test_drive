import math
from types import SimpleNamespace
import unittest

from jetson.amr_core.full_run import FullRunConfig
from scripts.visualize_front_lidar import analyse_scan, format_screen


class FrontLidarVisualizerTests(unittest.TestCase):
    def setUp(self):
        self.config = FullRunConfig(lidar_x_m=0.40)

    @staticmethod
    def scan(front_distance):
        ranges = [math.inf] * 360
        ranges[180] = front_distance
        return SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi / 180,
                               range_min=0.05, range_max=12.0, ranges=ranges)

    def test_reports_same_bumper_clearance_and_stop_decision_as_controller(self):
        result = analyse_scan(self.scan(0.20), self.config)
        self.assertAlmostEqual(result["clearance_m"], 0.175)
        self.assertEqual(result["decision"], "STOP")
        self.assertAlmostEqual(result["nearest"]["x_m"], 0.60)
        self.assertAlmostEqual(result["nearest"]["y_m"], 0.0)

    def test_distinguishes_resume_deadband_from_clear_space(self):
        blocked = analyse_scan(self.scan(0.35), self.config)
        clear = analyse_scan(self.scan(0.60), self.config)
        self.assertEqual(blocked["decision"], "START BLOCKED")
        self.assertEqual(clear["decision"], "CLEAR")

    def test_terminal_screen_identifies_the_blocking_point(self):
        result = analyse_scan(self.scan(0.20), self.config)
        screen = format_screen(result, self.config, sequence=7, age_s=0.01)
        self.assertIn("decision=STOP", screen)
        self.assertIn("17.5cm", screen)
        self.assertIn("X=blocking return", screen)


if __name__ == "__main__":
    unittest.main()
