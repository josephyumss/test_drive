import unittest

from simulation.scenario_runner import run_demo_scenario


class ScenarioTests(unittest.TestCase):
    def test_end_to_end_demo_reaches_expected_states(self) -> None:
        records = run_demo_scenario()
        by_phase = {}
        for record in records:
            by_phase.setdefault(record.phase, []).append(record)

        self.assertEqual(by_phase["normal"][-1].jetson_state, "RUN")
        self.assertEqual(by_phase["normal"][-1].stm32_state, "RUN")
        self.assertEqual(by_phase["slow"][-1].jetson_state, "SLOW")
        self.assertEqual(by_phase["slow"][-1].stm32_state, "SLOW")
        self.assertEqual(
            by_phase["obstacle_stop"][-1].stm32_state, "CONTROLLED_STOP"
        )
        self.assertEqual(by_phase["manual_reset"][-1].stm32_state, "READY")
        self.assertEqual(by_phase["restart"][-1].stm32_state, "RUN")
        self.assertEqual(by_phase["cliff"][-1].stm32_state, "EMERGENCY_STOP")
        self.assertEqual(by_phase["cliff"][-1].actual_speed_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
