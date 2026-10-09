"""Automatic archives, failures, deduplication and early shell-exit capture."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock

from jetson.amr_core.full_run_diagnostics import REQUEST_FILE, STATUS_FILE, create_bundle, request_auto_bundle

ROOT = Path(__file__).resolve().parents[1]


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="full-run-bundle-")
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name) / "run"
        self.folder.mkdir()
        (self.folder / "launcher.log").write_bytes(b"startup\n")
        (self.folder / "events.jsonl").write_text('{"event":"state","after":"FAULT_STOP"}\n', encoding="utf8")
        (self.folder / "config.json").write_text('{"wheel_base_m":0.85}', encoding="utf8")

    def test_first_fault_reason_is_preserved(self):
        self.assertTrue(request_auto_bundle(self.folder, "STM32_data_lost"))
        self.assertFalse(request_auto_bundle(self.folder, "later_shutdown"))
        request = json.loads((self.folder / REQUEST_FILE).read_text())
        self.assertEqual(request["reason"], "STM32_data_lost")

    def test_automatic_archive_contains_logs_config_and_capture_metadata(self):
        request_auto_bundle(self.folder, "STM32_data_lost")
        destination = create_bundle(self.folder, automatic=True)
        self.assertTrue(destination.is_file())
        self.assertTrue((self.folder / "events.jsonl").is_file())
        with tarfile.open(destination, "r:gz") as archive:
            self.assertIn("run/config.json", archive.getnames())
            metadata = json.load(archive.extractfile("run/bundle_metadata.json"))
            self.assertTrue(metadata["automatic"])
            self.assertEqual(metadata["trigger"]["reason"], "STM32_data_lost")
            self.assertEqual(metadata["captured_files"]["launcher.log"], len(b"startup\n"))
        status = json.loads((self.folder / STATUS_FILE).read_text())
        self.assertTrue(status["complete"])

    def test_successful_automatic_capture_is_not_overwritten_by_later_shutdown(self):
        destination = create_bundle(self.folder, automatic=True, reason="first_fault")
        original = destination.read_bytes()
        (self.folder / "launcher.log").write_text("changed after fault; user stopped\n")
        self.assertEqual(create_bundle(self.folder, automatic=True, reason="later_shutdown"), destination)
        self.assertEqual(destination.read_bytes(), original)

    def test_manual_capture_can_refresh_same_path(self):
        destination = create_bundle(self.folder, automatic=True)
        original = destination.read_bytes()
        (self.folder / "launcher.log").write_text("new manual snapshot\n")
        self.assertEqual(create_bundle(self.folder), destination)
        self.assertNotEqual(destination.read_bytes(), original)

    def test_failed_compression_does_not_publish_partial_archive_or_remove_logs(self):
        with mock.patch("jetson.amr_core.full_run_diagnostics.tarfile.open", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                create_bundle(self.folder, automatic=True, reason="sensor_fault")
        self.assertEqual(list(self.folder.parent.glob("*-debug.tar.gz")), [])
        self.assertEqual(list(self.folder.parent.glob("*.part")), [])
        self.assertFalse((self.folder / STATUS_FILE).exists())
        self.assertTrue((self.folder / "events.jsonl").exists())
        self.assertTrue(create_bundle(self.folder, automatic=True).exists())
        self.assertEqual(json.loads((self.folder / REQUEST_FILE).read_text())["reason"], "sensor_fault")

    def test_log_append_during_copy_does_not_corrupt_archive(self):
        original = tarfile.TarFile.addfile
        appended = []
        def append_then_add(archive, entry, stream=None):
            if entry.name.endswith("launcher.log"):
                with (self.folder / "launcher.log").open("a", encoding="utf8") as writer:
                    writer.write("appended while compression was reading\n")
                appended.append(True)
            return original(archive, entry, stream)
        with mock.patch.object(tarfile.TarFile, "addfile", append_then_add):
            destination = create_bundle(self.folder)
        self.assertTrue(appended)
        with tarfile.open(destination, "r:gz") as archive:
            self.assertEqual(archive.extractfile("run/launcher.log").read(), b"startup\n")

    def test_invalid_folder_is_rejected(self):
        with self.assertRaises(ValueError):
            create_bundle(self.folder.parent, automatic=True)

    def test_cli_automatic_mode(self):
        result = subprocess.run([sys.executable, str(ROOT / "scripts/full_run_diagnostics.py"),
                                 str(self.folder), "--auto-bundle", "--reason", "configuration_error"],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(Path(result.stdout.strip()).is_file())


class EarlyLauncherFailureTests(unittest.TestCase):
    def test_startup_failure_archives_without_opening_actuator(self):
        # Only the platform guard is replaced in a disposable checkout. No
        # device exists and startup fails at dependency discovery. This tests
        # the real EXIT trap on Windows Git Bash as well as Linux Bash.
        bash = shutil.which("bash") if os.name == "posix" else r"C:\Program Files\Git\bin\bash.exe"
        if not bash or not Path(bash).is_file():
            self.skipTest("No runnable Bash")
        with tempfile.TemporaryDirectory(prefix="full-run-early-exit-") as temporary:
            folder = Path(temporary)
            (folder / "scripts").mkdir()
            (folder / "jetson/amr_core").mkdir(parents=True)
            source = (ROOT / "scripts/start_full_run.sh").read_text()
            source = source.replace('[[ "$(uname -s)" == Linux ]]', '[[ 1 == 1 ]]')
            (folder / "scripts/start_full_run.sh").write_text(source, encoding="utf8")
            shutil.copyfile(ROOT / "scripts/full_run_diagnostics.py", folder / "scripts/full_run_diagnostics.py")
            shutil.copyfile(ROOT / "jetson/amr_core/full_run_diagnostics.py", folder / "jetson/amr_core/full_run_diagnostics.py")
            environment = dict(os.environ, PYTHON_BIN=sys.executable.replace("\\", "/"),
                               ROS_SETUP=str(folder / "missing-ros-setup"))
            for key in ("FULL_RUN_LOG_ROOT", "FULL_RUN_LOG_DIR", "LIDAR_SETUP", "PYTHONPATH"):
                environment.pop(key, None)
            result = subprocess.run([bash, str(folder / "scripts/start_full_run.sh")],
                                    env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=30)
            self.assertNotEqual(result.returncode, 0, result.stdout)
            archives = list((folder / "logs/full_run").glob("*-debug.tar.gz"))
            self.assertEqual(len(archives), 1, result.stdout)
            self.assertIn("[BUNDLE] Saved:", result.stdout)
            with tarfile.open(archives[0], "r:gz") as archive:
                self.assertTrue(any(path.endswith("launcher.log") for path in archive.getnames()))


if __name__ == "__main__":
    unittest.main()
