import unittest

from protocol.protocol_constants import SystemState
from jetson.amr_core.velocity import VelocityCommand, VelocityLimiter


class VelocityLimiterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.limiter = VelocityLimiter()

    def test_non_driving_state_forces_zero(self) -> None:
        requested = VelocityCommand(0.4, 0.3)
        for state in (
            SystemState.INIT,
            SystemState.READY,
            SystemState.CONTROLLED_STOP,
            SystemState.EMERGENCY_STOP,
            SystemState.FAULT,
        ):
            with self.subTest(state=state):
                self.assertEqual(
                    self.limiter.limit(requested, state), VelocityCommand()
                )

    def test_run_limits_maximum_velocity(self) -> None:
        limited = self.limiter.limit(
            VelocityCommand(1.2, -2.0), SystemState.RUN
        )
        self.assertEqual(limited, VelocityCommand(0.5, -0.8))

    def test_slow_uses_reduced_limits(self) -> None:
        limited = self.limiter.limit(
            VelocityCommand(0.5, 0.8), SystemState.SLOW
        )
        self.assertEqual(limited, VelocityCommand(0.25, 0.4))

    def test_reverse_is_blocked_by_default(self) -> None:
        limited = self.limiter.limit(
            VelocityCommand(-0.3, 0.0), SystemState.RUN
        )
        self.assertEqual(limited.linear_mps, 0.0)

    def test_user_speed_limit_is_applied(self) -> None:
        limited = self.limiter.limit(
            VelocityCommand(0.4, 0.0),
            SystemState.RUN,
            user_speed_limit_mps=0.2,
        )
        self.assertEqual(limited.linear_mps, 0.2)


if __name__ == "__main__":
    unittest.main()

