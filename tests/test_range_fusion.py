import math
import unittest

from jetson.amr_core.range_fusion import RangeFusion, RangeReading, sector_minimum


class RangeFusionTests(unittest.TestCase):
    def test_nearest_fresh_sensor_wins(self) -> None:
        fusion = RangeFusion(timeout_s=0.5)
        fusion.update(RangeReading("ultrasonic", 1.2, 1.0))
        fusion.update(RangeReading("sharp_left", 0.4, 1.0))
        result = fusion.result(1.1)
        self.assertTrue(result.valid)
        self.assertEqual(result.source, "sharp_left")
        self.assertAlmostEqual(result.distance_m, 0.4)

    def test_stale_data_is_invalid(self) -> None:
        fusion = RangeFusion(timeout_s=0.2)
        fusion.update(RangeReading("lidar", 1.0, 1.0))
        result = fusion.result(1.3)
        self.assertFalse(result.valid)
        self.assertTrue(math.isinf(result.distance_m))

    def test_sector_minimum_ignores_invalid_values(self) -> None:
        value = sector_minimum(
            [math.inf, 2.0, 0.5], -0.1, 0.1, -0.01, 0.01, 0.1, 10.0
        )
        self.assertEqual(value, 2.0)


if __name__ == "__main__":
    unittest.main()
