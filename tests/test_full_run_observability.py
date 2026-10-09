"""Read-only inventories, independent import errors and GPIO failure evidence."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import types
import unittest
from unittest import mock

from jetson.amr_core.full_run import FullRunConfig
from jetson.amr_core.full_run_environment import collect_environment, command_evidence, read_evidence
from jetson.amr_core.full_run_ultrasonic import UltrasonicWorker

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("requirements_test", ROOT / "scripts/full_run_requirements.py")
requirements = importlib.util.module_from_spec(spec)
spec.loader.exec_module(requirements)


class InventoryTests(unittest.TestCase):
    def test_inventory_does_not_dump_secrets_or_run_mutating_commands(self):
        with mock.patch.dict("os.environ", {"GITHUB_TOKEN": "do-not-log-this", "MCU_PROTOCOL": "legacy"}), \
                mock.patch("jetson.amr_core.full_run_environment.command_evidence", return_value={}) as commands:
            report = collect_environment(ROOT)
        self.assertNotIn("do-not-log-this", json.dumps(report))
        self.assertEqual(report["settings_allowlist_only"]["MCU_PROTOCOL"], "legacy")
        for call in commands.call_args_list:
            argv = call.args[0]
            self.assertFalse(any(v in argv for v in ("start", "stop", "enable", "disable", "push", "pull")))

    def test_command_timeout_is_evidence_not_a_global_failure(self):
        with mock.patch("shutil.which", return_value="fake"), \
                mock.patch("subprocess.run", side_effect=subprocess.TimeoutExpired(["fake"], 2)):
            result = command_evidence(["fake"])
        self.assertIn("TimeoutExpired", result["error"])

    def test_missing_and_truncated_files_are_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "info"
            self.assertIn("error", read_evidence(path))
            path.write_bytes(b"x" * 100)
            evidence = read_evidence(path, 12)
            self.assertTrue(evidence["truncated"])
            self.assertEqual(len(evidence["text"]), 12)

    def test_every_failed_import_is_recorded(self):
        with mock.patch.object(requirements.importlib, "import_module", side_effect=ImportError("missing")):
            report = requirements.verify_imports()
        self.assertEqual(len(report), 6)
        self.assertTrue(all(not r["ok"] and "ImportError" in r["traceback"] for r in report))

    def test_old_installed_bridge_is_rejected_while_other_imports_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            expected = root / "jetson_ws/src/amr_vision/amr_vision/yolo_udp_bridge_node.py"
            expected.parent.mkdir(parents=True)
            expected.write_text("new bridge")
            old = root / "old.py"
            old.write_text("old bridge")
            module = types.SimpleNamespace(__file__=str(old), LaserScan=object, String=object, ObstacleInfo=object)
            with mock.patch.object(requirements.importlib, "import_module", return_value=module):
                report = requirements.verify_imports(root)
            self.assertTrue(all(r["ok"] for r in report[:-1]))
            self.assertFalse(report[-1]["ok"])
            self.assertNotEqual(report[-1]["expected_sha256"], report[-1]["installed_sha256"])

    def test_STM32_side_mode_does_not_require_Jetson_GPIO_import(self):
        with mock.patch.dict("os.environ", {"SIDE_SENSOR_SOURCE": "mcu"}), \
                mock.patch.object(requirements.importlib, "import_module", side_effect=ImportError("missing")) as imports:
            report = requirements.verify_imports()
        self.assertFalse(any(r["name"] == "Jetson.GPIO" for r in report))
        self.assertFalse(any(c.args[0] == "Jetson.GPIO" for c in imports.call_args_list))


class GpioDiagnosticsTests(unittest.TestCase):
    def worker(self):
        gpio = types.SimpleNamespace(BOARD=1, OUT=2, IN=3, BOTH=4, LOW=0, HIGH=1,
            setmode=mock.Mock(), setup=mock.Mock(), input=mock.Mock(return_value=0), output=mock.Mock(),
            add_event_detect=mock.Mock(), remove_event_detect=mock.Mock(), cleanup=mock.Mock())
        thread = types.SimpleNamespace(start=mock.Mock(), join=mock.Mock(), is_alive=lambda: False)
        events = []
        with mock.patch("jetson.amr_core.full_run_ultrasonic.threading.Thread", return_value=thread):
            worker = UltrasonicWorker(gpio, FullRunConfig(), lambda event, **data: events.append((event, data)))
        return worker, gpio, events

    def test_callback_exception_becomes_FAULT_with_traceback(self):
        worker, gpio, events = self.worker()
        worker.active_echo = worker.pins["left"][1]
        gpio.input.side_effect = OSError("edge GPIO failure")
        worker.on_edge(worker.active_echo)
        self.assertTrue(worker.echo_done.is_set())
        self.assertIn("OSError", worker.edge_error)
        self.assertTrue(any(e == "ultrasonic_exception" and d["stage"] == "GPIO_edge_callback" for e, d in events))
        worker.close()

    def test_cleanup_continues_after_one_pin_failure_and_reports_failure(self):
        worker, gpio, events = self.worker()
        gpio.remove_event_detect.side_effect = [OSError("pin failure"), None]
        with self.assertRaisesRegex(RuntimeError, "GPIO cleanup failures"):
            worker.close()
        self.assertEqual(gpio.remove_event_detect.call_count, 2)
        gpio.cleanup.assert_called_once()
        self.assertTrue(any(e == "ultrasonic_exception" for e, _ in events))


if __name__ == "__main__":
    unittest.main()
