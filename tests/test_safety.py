import unittest

from protocol.protocol_constants import SafetyFlag, SystemState
from jetson.amr_core.obstacle import ObstacleDecision
from jetson.amr_core.safety import SafetyInputs, SafetyStateMachine


class SafetyStateMachineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.machine = SafetyStateMachine()

    def test_initialization_then_ready_then_run(self) -> None:
        output = self.machine.update(SafetyInputs())
        self.assertEqual(output.state, SystemState.INIT)

        output = self.machine.update(SafetyInputs(initialization_complete=True))
        self.assertEqual(output.state, SystemState.READY)

        output = self.machine.update(
            SafetyInputs(initialization_complete=True, drive_enable=True)
        )
        self.assertEqual(output.state, SystemState.RUN)

    def test_cliff_has_priority_and_latches_restart(self) -> None:
        output = self.machine.update(
            SafetyInputs(
                initialization_complete=True,
                drive_enable=True,
                cliff_left=True,
            )
        )
        self.assertEqual(output.state, SystemState.EMERGENCY_STOP)
        self.assertTrue(output.flags & SafetyFlag.CLIFF_LEFT)
        self.assertTrue(output.restart_latched)

        output = self.machine.update(
            SafetyInputs(initialization_complete=True, drive_enable=True)
        )
        self.assertEqual(output.state, SystemState.EMERGENCY_STOP)
        self.assertTrue(output.restart_latched)

    def test_manual_reset_returns_to_ready_not_run(self) -> None:
        self.machine.update(
            SafetyInputs(initialization_complete=True, communication_timeout=True)
        )
        output = self.machine.update(
            SafetyInputs(
                initialization_complete=True,
                drive_enable=True,
                reset_requested=True,
            )
        )
        self.assertEqual(output.state, SystemState.READY)
        self.assertFalse(output.restart_latched)

        output = self.machine.update(
            SafetyInputs(initialization_complete=True, drive_enable=True)
        )
        self.assertEqual(output.state, SystemState.RUN)

    def test_fault_has_priority_over_cliff(self) -> None:
        output = self.machine.update(
            SafetyInputs(
                initialization_complete=True,
                drive_enable=True,
                cliff_left=True,
                motor_fault=True,
            )
        )
        self.assertEqual(output.state, SystemState.FAULT)
        self.assertTrue(output.flags & SafetyFlag.MOTOR_FAULT)

    def test_obstacle_decisions_map_to_states(self) -> None:
        slow = self.machine.update(
            SafetyInputs(
                initialization_complete=True,
                drive_enable=True,
                obstacle_decision=ObstacleDecision.SLOW,
            )
        )
        self.assertEqual(slow.state, SystemState.SLOW)

        stop = self.machine.update(
            SafetyInputs(
                initialization_complete=True,
                drive_enable=True,
                obstacle_decision=ObstacleDecision.CONTROLLED_STOP,
            )
        )
        self.assertEqual(stop.state, SystemState.CONTROLLED_STOP)
        self.assertTrue(stop.restart_latched)


if __name__ == "__main__":
    unittest.main()

