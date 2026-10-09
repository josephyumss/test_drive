"""Keep robot evidence shareable without exposing private/runtime files."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class DiagnosticGitIgnoreTests(unittest.TestCase):
    def setUp(self):
        self.git = shutil.which("git")
        if not self.git:
            self.skipTest("Git is not installed")
        temporary = tempfile.TemporaryDirectory(prefix="full-run-gitignore-")
        self.addCleanup(temporary.cleanup)
        self.folder = Path(temporary.name)
        shutil.copyfile(ROOT / ".gitignore", self.folder / ".gitignore")
        self.command = [self.git, "-c", f"safe.directory={self.folder.as_posix()}",
                        "-c", f"core.excludesFile={os.devnull}"]
        subprocess.run(self.command + ["init", "--quiet"], cwd=self.folder,
                       check=True, capture_output=True, timeout=15)

    def ignored_paths(self, paths):
        # NUL separators avoid Windows text-mode CRLF becoming part of a path.
        result = subprocess.run(self.command + ["check-ignore", "--no-index", "--stdin", "-z"],
                                cwd=self.folder, input=("\0".join(paths) + "\0").encode("utf8"),
                                capture_output=True, timeout=15)
        self.assertIn(result.returncode, (0, 1), result.stderr)
        return {path for path in result.stdout.decode("utf8").split("\0") if path}

    def test_raw_logs_and_completed_archives_are_not_ignored(self):
        paths = [
            "logs/full_run/20261009T010000Z-ABCDEF-debug.tar.gz",
            "logs/full_run/20261009T010000Z-ABCDEF/launcher.log",
            "logs/full_run/20261009T010000Z-ABCDEF/controller.log",
            "logs/full_run/20261009T010000Z-ABCDEF/events.jsonl",
            "logs/full_run/20261009T010000Z-ABCDEF/events.jsonl.24",
            "logs/full_run/20261009T010000Z-ABCDEF/config.json",
            "logs/full_run/20261009T010000Z-ABCDEF/source.tar.gz",
            "logs/full_run/20261009T010000Z-ABCDEF/auto_bundle.status.json",
            "logs/full_run_setup/20261009T010000Z-ABCDEF-debug.tar.gz",
            "logs/full_run_setup/20261009T010000Z-ABCDEF/launcher.log",
            "logs/full_run_setup/20261009T010000Z-ABCDEF/apt_base_update.log",
            "logs/full_run_setup/20261009T010000Z-ABCDEF/stages.tsv",
            "logs/full_run_setup/20261009T010000Z-ABCDEF/summary.txt",
            "logs/full_run_setup/20261009T010000Z-ABCDEF/source/scripts/setup_full_run.sh",
        ]
        self.assertEqual(self.ignored_paths(paths), set())

    def test_alias_and_incomplete_archives_remain_ignored(self):
        paths = [
            "logs/full_run/latest",
            "logs/full_run/.20261009T010000Z-ABCDEF-debug-X.part",
            "logs/full_run/20261009T010000Z-ABCDEF/unfinished.part",
            "logs/full_run_setup/.setup-debug-X.part",
        ]
        self.assertEqual(self.ignored_paths(paths), set(paths))

    def test_private_configuration_and_unrelated_logs_remain_ignored(self):
        paths = [
            "config/full_run.env", "config/runtime.env", "config/local.yaml",
            ".run/full_run.pid", ".runtime/ldlidar_ros2_ws/build/cache",
            "logs/other_run/controller.log", "logs/other_run/events.jsonl",
            "jetson_ws/log/colcon.log", "models/yolo11n.pt", "other.log",
        ]
        self.assertEqual(self.ignored_paths(paths), set(paths))


if __name__ == "__main__":
    unittest.main()
