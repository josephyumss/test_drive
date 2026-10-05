import math
import unittest

from jetson.amr_core.obstacle import ObstacleDecision, ObstacleEvaluator


class ObstacleEvaluatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.evaluator = ObstacleEvaluator()

    def test_clear_stationary_obstacle_allows_run(self) -> None:
        assessment = self.evaluator.evaluate(3.0, 0.0)
        self.assertEqual(assessment.decision, ObstacleDecision.RUN)
        self.assertTrue(math.isinf(assessment.ttc_s))

    def test_distance_causes_slow(self) -> None:
        assessment = self.evaluator.evaluate(1.8, 0.1)
        self.assertEqual(assessment.decision, ObstacleDecision.SLOW)

    def test_ttc_can_be_more_severe_than_distance(self) -> None:
        assessment = self.evaluator.evaluate(2.5, 2.0)
        self.assertEqual(assessment.decision, ObstacleDecision.CONTROLLED_STOP)
        self.assertAlmostEqual(assessment.ttc_s, 1.25)

    def test_close_obstacle_causes_emergency(self) -> None:
        assessment = self.evaluator.evaluate(0.5, 0.1)
        self.assertEqual(assessment.decision, ObstacleDecision.EMERGENCY_STOP)

    def test_invalid_distance_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.evaluator.evaluate(float("nan"), 0.2)


if __name__ == "__main__":
    unittest.main()
