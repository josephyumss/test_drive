"""Normal ROS signal shutdown is not a sensor-process crash."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


class ExternalShutdown(Exception):
    pass


class BridgeShutdownTests(unittest.TestCase):
    def load_bridge(self, spin_error=None, init_node_error=None, destroy_error=None):
        ros = types.ModuleType("rclpy")
        ros.init = mock.Mock()
        ros.spin = mock.Mock(side_effect=spin_error)
        ros.try_shutdown = mock.Mock()
        modules = {"rclpy": ros}
        for name, attributes in {
            "rclpy.executors": {"ExternalShutdownException": ExternalShutdown},
            "rclpy.node": {"Node": object},
            "rclpy.qos": {"qos_profile_sensor_data": object()},
            "sensor_msgs": {}, "sensor_msgs.msg": {"LaserScan": object},
            "std_msgs": {}, "std_msgs.msg": {"String": object},
            "amr_interfaces": {}, "amr_interfaces.msg": {"ObstacleInfo": object},
        }.items():
            module = types.ModuleType(name)
            module.__dict__.update(attributes)
            modules[name] = module
        node = types.SimpleNamespace(destroy_node=mock.Mock(side_effect=destroy_error))
        with mock.patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location(
                "bridge_shutdown_test", ROOT / "jetson_ws/src/amr_vision/amr_vision/yolo_udp_bridge_node.py")
            bridge = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(bridge)
        # No real ROS node, GPIO, socket or robot is touched in this test.
        bridge.YoloUdpBridgeNode = mock.Mock(return_value=node, side_effect=init_node_error)
        return bridge, ros, node

    def test_normal_spin_completion_cleans_up_once(self):
        bridge, ros, node = self.load_bridge()
        bridge.main(["--ros-args"])
        ros.init.assert_called_once_with(args=["--ros-args"])
        ros.spin.assert_called_once_with(node)
        node.destroy_node.assert_called_once_with()
        ros.try_shutdown.assert_called_once_with()

    def test_external_shutdown_and_keyboard_interrupt_are_normal_cleanup(self):
        for error in (ExternalShutdown(), KeyboardInterrupt()):
            with self.subTest(error=error):
                bridge, ros, node = self.load_bridge(spin_error=error)
                bridge.main()
                node.destroy_node.assert_called_once_with()
                ros.try_shutdown.assert_called_once_with()

    def test_real_spin_failure_is_not_swallowed(self):
        bridge, ros, node = self.load_bridge(spin_error=RuntimeError("real sensor failure"))
        with self.assertRaisesRegex(RuntimeError, "real sensor failure"):
            bridge.main()
        node.destroy_node.assert_called_once_with()
        ros.try_shutdown.assert_called_once_with()

    def test_constructor_failure_still_releases_ROS_context(self):
        bridge, ros, node = self.load_bridge(init_node_error=OSError("UDP bind failed"))
        with self.assertRaisesRegex(OSError, "UDP bind failed"):
            bridge.main()
        node.destroy_node.assert_not_called()
        ros.try_shutdown.assert_called_once_with()

    def test_cleanup_failure_still_attempts_context_shutdown(self):
        bridge, ros, node = self.load_bridge(destroy_error=RuntimeError("cleanup failed"))
        with self.assertRaisesRegex(RuntimeError, "cleanup failed"):
            bridge.main()
        ros.try_shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
