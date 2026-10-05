from pathlib import Path
from types import SimpleNamespace
import math
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
for package_dir in (
    ROOT / "jetson_ws/src/amr_dummy",
    ROOT / "jetson_ws/src/amr_safety_node",
    ROOT / "jetson_ws/src/amr_monitoring_adapter",
):
    sys.path.insert(0, str(package_dir))

from amr_dummy.imu_generator import ImuGenerator
from amr_dummy.lidar_generator import LidarGenerator
from amr_dummy.scenario_manager import ScenarioManager
from amr_monitoring_adapter.status_mapper import make_monitoring_payload
from amr_safety_node.input_mapper import NormalizedInputs, to_controller_inputs
from amr_safety_node.sensor_timeout_monitor import SensorTimeoutMonitor
from protocol.protocol_constants import SystemState


class ScenarioManagerTests(unittest.TestCase):
    def test_named_scenario(self) -> None:
        state = ScenarioManager("cliff_stop").state_at(0.0)
        self.assertTrue(state.cliff_left)
        self.assertGreater(state.tof_distance_m, 0.4)

    def test_automatic_contains_reset_after_stop(self) -> None:
        manager = ScenarioManager("automatic")
        self.assertEqual(manager.state_at(12.0).name, "obstacle_stop")
        self.assertEqual(manager.state_at(16.0).name, "reset")
        self.assertTrue(manager.state_at(16.0).reset_requested)
        self.assertEqual(manager.state_at(22.0).name, "cliff_stop")


class DummyGeneratorTests(unittest.TestCase):
    def test_lidar_places_obstacle_in_front(self) -> None:
        generator = LidarGenerator()
        scan = generator.generate(1.5, 0.0, 20.0)
        front = round((0.0 - scan.angle_min) / scan.angle_increment)
        self.assertAlmostEqual(scan.ranges[front], 1.5)
        self.assertEqual(scan.ranges[0], scan.range_max)

    def test_imu_bias_is_injected(self) -> None:
        data = ImuGenerator().generate(gyro_bias_z=0.0035)
        self.assertAlmostEqual(data.angular_velocity_z, 0.0035)
        self.assertAlmostEqual(data.linear_acceleration_z, 9.80665)


class TimeoutMonitorTests(unittest.TestCase):
    def test_unseen_is_not_timeout_but_not_ready(self) -> None:
        monitor = SensorTimeoutMonitor({"mcu": 0.3})
        self.assertFalse(monitor.is_timed_out("mcu", 1.0))
        self.assertFalse(monitor.all_seen("mcu"))

    def test_seen_source_times_out(self) -> None:
        monitor = SensorTimeoutMonitor({"mcu": 0.3})
        monitor.update("mcu", 1.0)
        self.assertFalse(monitor.is_timed_out("mcu", 1.3))
        self.assertTrue(monitor.is_timed_out("mcu", 1.31))


class InputMapperTests(unittest.TestCase):
    def test_mcu_timeout_maps_to_communication_timeout(self) -> None:
        mapped = to_controller_inputs(
            NormalizedInputs(
                initialization_complete=True,
                obstacle_valid=True,
                obstacle_distance_m=3.0,
                mcu_timed_out=True,
            )
        )
        self.assertTrue(mapped.communication_timeout)

    def test_obstacle_timeout_requests_controlled_stop(self) -> None:
        mapped = to_controller_inputs(
            NormalizedInputs(initialization_complete=True, obstacle_timed_out=True)
        )
        self.assertTrue(mapped.controlled_stop_requested)


class MonitoringMapperTests(unittest.TestCase):
    def test_dummy_status_is_finite_and_identified(self) -> None:
        safety = SimpleNamespace(
            state=int(SystemState.RUN), safety_flags=0, reason="all checks passed"
        )
        mcu = SimpleNamespace(
            connected=True,
            left_velocity_mps=0.3,
            right_velocity_mps=0.4,
            motor_error=0,
            rx_error_count=0,
            last_command_id=4,
            uptime_ms=1000,
        )
        obstacle = SimpleNamespace(
            detected=False,
            valid=True,
            object_class="없음",
            distance_m=10.0,
            ttc_s=math.inf,
            direction="전방",
        )
        payload = make_monitoring_payload(
            safety=safety,
            mcu=mcu,
            obstacle=obstacle,
            dummy_active=True,
            scenario_name="normal",
        )
        self.assertEqual(payload["system_state"], "RUN")
        self.assertIn("DUMMY:normal", payload["state_reason"])
        self.assertIsNone(payload["obstacle"]["ttc_s"])
        self.assertAlmostEqual(payload["velocity"]["actual_linear_mps"], 0.35)


if __name__ == "__main__":
    unittest.main()

