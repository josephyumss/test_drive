"""Real Bash setup runner with inert workers: no apt, Docker or robot commands."""
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASH = shutil.which("bash") if os.name == "posix" else r"C:\Program Files\Git\bin\bash.exe"
HAS_BASH = bool(BASH and Path(BASH).is_file())

FAKE_WORKER = r'''#!/usr/bin/env bash
set -Eeuo pipefail
step="$1"
echo "$step" >> "$CALLS_FILE"
echo "stdout from $step"
echo "stderr from $step" >&2
code="$(awk -F '\t' -v name="$step" '$1 == name {print $2}' "$RESULTS_FILE")"
code="${code:-0}"
if [[ "${RAW_STATUS_STEP:-}" != "$step" ]]; then
    case "$code" in
        77) printf 'SKIP\n' > "$FULL_RUN_SETUP_LOG_DIR/$step.status" ;;
        78) printf 'BLOCKED\n' > "$FULL_RUN_SETUP_LOG_DIR/$step.status" ;;
    esac
fi
if [[ "$step" == base_packages && "${STRICT_FAILURE:-0}" == 1 ]]; then
    trap 'code=$?; echo "step=$step exit=$code line=$LINENO command=$BASH_COMMAND"; exit "$code"' ERR
    failing_function() {
        false
        echo 'BUG: continued inside failed step'
    }
    failing_function
fi
if [[ "$step" == configuration ]]; then echo 'last configuration check ran'; fi
exit "$code"
'''


@unittest.skipUnless(HAS_BASH, "No runnable Bash")
class SetupRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="full-run-setup-")
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        (self.folder / "scripts").mkdir()
        (self.folder / "jetson/amr_core").mkdir(parents=True)
        for filename in ("setup_full_run.sh", "full_run_diagnostics.py", "full_run_log_access.sh"):
            shutil.copyfile(ROOT / "scripts" / filename, self.folder / "scripts" / filename)
        shutil.copyfile(ROOT / "jetson/amr_core/full_run_diagnostics.py",
                        self.folder / "jetson/amr_core/full_run_diagnostics.py")
        (self.folder / "scripts/full_run_setup_steps.sh").write_text(FAKE_WORKER, encoding="utf8")
        self.calls = self.folder / "calls.txt"
        self.results = self.folder / "results.tsv"
        self.results.write_text("", encoding="utf8")
        self.environment = dict(os.environ, PYTHON_BIN=sys.executable.replace("\\", "/"),
                                CALLS_FILE=self.calls.as_posix(), RESULTS_FILE=self.results.as_posix())
        for key in ("FULL_RUN_SETUP_LOG_DIR", "FULL_RUN_SETUP_CONTEXT", "PROJECT_DIR", "STRICT_FAILURE", "RAW_STATUS_STEP"):
            self.environment.pop(key, None)

    def run_setup(self, outcomes=None, **environment):
        self.results.write_text("".join(f"{name}\t{code}\n" for name, code in (outcomes or {}).items()), encoding="utf8")
        result = subprocess.run([BASH, str(self.folder / "scripts/setup_full_run.sh")],
                                env=dict(self.environment, **environment), stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=30)
        folders = sorted(path for path in (self.folder / "logs/full_run_setup").iterdir() if path.is_dir())
        self.assertTrue(folders, result.stdout)
        self.log_dir = max(folders, key=lambda path: path.stat().st_mtime_ns)
        with (self.log_dir / "stages.tsv").open(encoding="utf8", newline="") as stream:
            self.rows = {row["step"]: row for row in csv.DictReader(stream, delimiter="\t")}
        self.called = self.calls.read_text().splitlines()
        return result

    def archive_contents(self):
        archives = list((self.folder / "logs/full_run_setup").glob("*-debug.tar.gz"))
        self.assertEqual(len(archives), 1)
        with tarfile.open(archives[0], "r:gz") as archive:
            return {Path(entry.name).name: archive.extractfile(entry).read()
                    for entry in archive.getmembers() if entry.isfile()}

    def test_multiple_errors_are_saved_and_remaining_independent_steps_run(self):
        result = self.run_setup({"base_packages": 11, "ros_bridge_build": 12,
                                 "model_download": 13, "jetson_gpio": 14, "configuration": 15})
        self.assertEqual(result.returncode, 1, result.stdout)
        for name, code in (("base_packages", 11), ("ros_bridge_build", 12), ("model_download", 13),
                           ("jetson_gpio", 14), ("configuration", 15)):
            self.assertEqual(self.rows[name]["status"], "FAILED")
            self.assertEqual(self.rows[name]["exit_code"], str(code))
            log = (self.log_dir / f"{name}.log").read_text()
            self.assertIn(f"stdout from {name}", log)
            self.assertIn(f"stderr from {name}", log)
            self.assertTrue(self.rows[name]["started_utc"].endswith("Z"))
        self.assertIn("lidar_build", self.called)
        self.assertIn("verify_python", self.called)
        self.assertIn("system_after", self.called)
        summary = (self.log_dir / "summary.txt").read_text()
        self.assertIn("FAILED=5 BLOCKED=0", summary)
        self.assertIn("NOT complete", summary)
        contents = self.archive_contents()
        self.assertIn(b"last configuration check ran", contents["launcher.log"])
        self.assertIn(b"stdout from system_after", contents["launcher.log"])
        self.assertIn(b"FAILED=5", contents["summary.txt"])
        self.assertTrue((self.log_dir / "base_packages.log").exists())

    def test_successful_and_reused_install_has_logs_but_no_failure_archive(self):
        result = self.run_setup({"base_packages": 77, "ros_apt_source": 77,
                                 "lidar_source": 77, "yolo_image": 77, "jetson_gpio": 77})
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.rows["base_packages"]["status"], "SKIP")
        self.assertEqual(self.rows["lidar_build"]["status"], "OK")
        self.assertEqual(self.rows["apt_ros_update"]["status"], "OK")
        self.assertIn("FAILED=0 BLOCKED=0", (self.log_dir / "summary.txt").read_text())
        self.assertIn("Dependencies prepared", result.stdout)
        self.assertEqual(list((self.folder / "logs/full_run_setup").glob("*-debug.tar.gz")), [])

    def test_docker_and_lidar_failures_block_only_their_dependents(self):
        result = self.run_setup({"docker_daemon": 23, "lidar_source": 24})
        self.assertEqual(result.returncode, 1)
        for name in ("docker_runtime", "yolo_image", "lidar_build"):
            self.assertNotIn(name, self.called)
            self.assertEqual(self.rows[name]["status"], "BLOCKED")
        for name in ("base_packages", "ros_bridge_build", "model_download", "jetson_gpio",
                     "verify_lidar", "verify_yolo_image", "configuration"):
            self.assertIn(name, self.called)
        self.assertIn("Prerequisite docker_daemon is FAILED", (self.log_dir / "yolo_image.log").read_text())
        self.assertIn("FAILED=2 BLOCKED=3", (self.log_dir / "summary.txt").read_text())

    def test_failed_apt_update_does_not_prevent_trying_cached_package_install(self):
        result = self.run_setup({"apt_base_update": 31, "apt_ros_update": 32})
        self.assertEqual(result.returncode, 1)
        self.assertEqual(self.rows["base_packages"]["status"], "OK")
        self.assertEqual(self.rows["ros_packages"]["status"], "OK")
        self.assertEqual(self.rows["ros_bridge_build"]["status"], "OK")

    def test_unsupported_platform_blocks_mutation_but_still_checks_configuration(self):
        result = self.run_setup({"platform": 1})
        self.assertEqual(result.returncode, 1)
        for name in ("base_packages", "ros_packages", "ros_bridge_build", "lidar_source",
                     "lidar_build", "model_download", "yolo_image", "jetson_gpio"):
            self.assertNotIn(name, self.called)
            self.assertEqual(self.rows[name]["status"], "BLOCKED")
        self.assertIn("configuration", self.called)
        self.assertIn("verify_python", self.called)
        self.assertIn("system_after", self.called)

    def test_worker_reported_blocked_is_not_success(self):
        result = self.run_setup({"ros_bridge_build": 78})
        self.assertEqual(result.returncode, 1)
        self.assertEqual(self.rows["ros_bridge_build"]["status"], "BLOCKED")
        self.assertIn("jetson_gpio", self.called)
        self.assertIn("FAILED=0 BLOCKED=1", (self.log_dir / "summary.txt").read_text())

    def test_curl_exit_77_is_failure_not_already_installed(self):
        result = self.run_setup({"ros_apt_source": 77}, RAW_STATUS_STEP="ros_apt_source")
        self.assertEqual(result.returncode, 1)
        self.assertEqual(self.rows["ros_apt_source"]["status"], "FAILED")
        self.assertEqual(self.rows["apt_ros_update"]["status"], "BLOCKED")
        self.assertIn("ros_packages", self.called)
        self.assertIn("FAILED=1 BLOCKED=1", (self.log_dir / "summary.txt").read_text())

    def test_worker_keeps_errexit_even_when_runner_handles_failure(self):
        result = self.run_setup(STRICT_FAILURE="1")
        self.assertEqual(result.returncode, 1)
        log = (self.log_dir / "base_packages.log").read_text()
        self.assertIn("exit=1", log)
        self.assertIn("command=false", log)
        self.assertNotIn("BUG: continued", result.stdout)
        self.assertIn("configuration", self.called)

    def test_tar_fallback_preserves_all_errors_when_python_bundler_is_broken(self):
        (self.folder / "scripts/full_run_diagnostics.py").unlink()
        result = self.run_setup({"base_packages": 41, "configuration": 42})
        self.assertEqual(result.returncode, 1)
        self.assertIn("(tar fallback)", result.stdout)
        contents = self.archive_contents()
        self.assertIn(b"FAILED=2", contents["summary.txt"])
        self.assertIn(b"last configuration check ran", contents["launcher.log"])
        self.assertIn("can't open file", (self.log_dir / "bundle.log").read_text())
        self.assertEqual(list((self.folder / "logs/full_run_setup").glob("*.part")), [])

    def test_rerun_keeps_previous_attempt_logs_and_archive(self):
        first = self.run_setup({"base_packages": 51})
        first_dir = self.log_dir
        first_archive = Path(str(first_dir) + "-debug.tar.gz")
        original = first_archive.read_bytes()
        second = self.run_setup()
        self.assertEqual(first.returncode, 1)
        self.assertEqual(second.returncode, 0)
        self.assertNotEqual(first_dir, self.log_dir)
        self.assertTrue((first_dir / "base_packages.log").is_file())
        self.assertEqual(first_archive.read_bytes(), original)

    def test_summary_log_write_failure_cannot_report_success(self):
        bin_dir = self.folder / "bin"
        bin_dir.mkdir()
        tee = bin_dir / "tee"
        tee.write_bytes(b'#!/usr/bin/env bash\n/usr/bin/tee "$@"\n'
                        b'if [[ "$*" == *"summary.txt"* ]]; then exit 95; fi\n')
        tee.chmod(0o755)
        launcher = self.folder / "scripts/setup_full_run.sh"
        source = launcher.read_text()
        # Only PATH setup changes in the disposable fixture; production has no
        # testing override and all installation workers remain inert stubs.
        prefix = 'if command -v cygpath >/dev/null; then setup_test_bin="$(cygpath -u "$SETUP_TEST_BIN")"; '
        prefix += 'else setup_test_bin="$SETUP_TEST_BIN"; fi\nexport PATH="$setup_test_bin:$PATH"\n'
        launcher.write_bytes(source.replace('set -Eeuo pipefail\n', 'set -Eeuo pipefail\n' + prefix, 1).encode())
        result = self.run_setup(SETUP_TEST_BIN=bin_dir.as_posix())
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("summary could not be written", result.stdout)
        self.assertIn(b"summary could not be written", self.archive_contents()["launcher.log"])


@unittest.skipUnless(HAS_BASH, "No runnable Bash")
class SetupWorkerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="full-run-worker-")
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.bin_dir = self.folder / "bin"
        self.bin_dir.mkdir()
        self.environment = dict(os.environ, PROJECT_DIR=ROOT.as_posix(),
                                FULL_RUN_SETUP_LOG_DIR=self.folder.as_posix(),
                                FULL_RUN_SETUP_CONTEXT=(self.folder / "context.env").as_posix(),
                                PYTHON_BIN=sys.executable.replace("\\", "/"), SIDE_SENSOR_SOURCE="gpio")
        # Bash, not Windows, splits PATH on ':'; construct it in the child.
        self.environment["SETUP_TEST_BIN"] = self.bin_dir.as_posix()
        self.environment["TRACE_FILE"] = (self.folder / "trace.txt").as_posix()
        self.environment.pop("ROS_APT_SOURCE_VERSION", None)
        self.environment.pop("JETSON_MODEL_NAME", None)

    def stub(self, name, source):
        script = self.bin_dir / name
        script.write_bytes(("#!/usr/bin/env bash\n" + source).encode("utf8"))
        script.chmod(0o755)

    def run_worker(self, step):
        return subprocess.run([BASH, "-c", 'if command -v cygpath >/dev/null; then '
                               'setup_bin="$(cygpath -u "$SETUP_TEST_BIN")"; else setup_bin="$SETUP_TEST_BIN"; fi; '
                               'export PATH="$setup_bin:$PATH"; exec /bin/bash "$1" "$2"',
                               "test-setup-worker", str(ROOT / "scripts/full_run_setup_steps.sh"), step],
                              env=self.environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=20)

    def test_configuration_check_runs_without_opening_hardware(self):
        result = self.run_worker("configuration")
        self.assertEqual(result.returncode, 0, result.stdout)
        config = json.loads(result.stdout)
        self.assertEqual(config["configuration"]["robot_width_m"], 0.9)
        self.assertFalse((self.folder / "config-check").exists())

    def test_existing_apt_packages_do_not_run_apt_update_or_install(self):
        self.stub("dpkg-query", "printf 'install ok installed'\n")
        self.stub("apt-get", 'echo "BUG: apt-get called" >> "$TRACE_FILE"; exit 91\n')
        for step in ("apt_base_update", "base_packages", "apt_universe", "ros_apt_source", "apt_ros_update", "ros_packages"):
            result = self.run_worker(step)
            self.assertEqual(result.returncode, 77, result.stdout)
            self.assertIn("[REUSE]", result.stdout)
        self.assertFalse((self.folder / "trace.txt").exists())

    def test_bootstrap_dpkg_failure_is_logged_and_owned_temp_directory_cleaned(self):
        self.stub("dpkg-query", "exit 1\n")
        self.stub("curl", r'''
output=''
while (( $# > 0 )); do
    if [[ "$1" == --output ]]; then output="$2"; shift; fi
    shift
done
if [[ -n "$output" ]]; then
    printf 'fake deb' > "$output"
    echo "$output" >> "$TRACE_FILE"
else
    printf '{"tag_name":"1.2.3"}\n'
fi
''')
        self.stub("dpkg", 'echo "dpkg simulated failure"; exit 67\n')
        result = self.run_worker("ros_apt_source")
        self.assertEqual(result.returncode, 67, result.stdout)
        self.assertIn("step=ros_apt_source exit=67", result.stdout)
        self.assertIn("command=dpkg", result.stdout)
        self.assertNotIn("unbound variable", result.stdout)
        downloaded = (self.folder / "trace.txt").read_text().strip()
        cleanup_check = subprocess.run([BASH, "-c", '[[ ! -e "$(dirname "$1")" ]]', "check-cleanup", downloaded],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        self.assertEqual(cleanup_check.returncode, 0, cleanup_check.stdout)

    def test_unknown_worker_step_is_rejected(self):
        result = self.run_worker("unknown")
        self.assertEqual(result.returncode, 2)
        self.assertIn("Unknown internal setup step", result.stdout)

    def test_python_verification_reports_every_failed_module(self):
        fake_modules = self.folder / "modules"
        fake_modules.mkdir()
        for name in ("serial", "numpy", "yaml", "cv2"):
            (fake_modules / f"{name}.py").write_bytes(f"raise RuntimeError('simulated {name} import failure')\n".encode())
        (fake_modules / "Jetson").mkdir()
        (fake_modules / "Jetson/__init__.py").write_bytes(b"")
        (fake_modules / "Jetson/GPIO.py").write_bytes(b"raise RuntimeError('simulated GPIO import failure')\n")
        self.environment["PYTHONPATH"] = str(fake_modules)
        result = self.run_worker("verify_python")
        self.assertEqual(result.returncode, 1, result.stdout)
        for name in ("serial", "numpy", "yaml", "cv2", "Jetson.GPIO"):
            self.assertIn(f"[ERROR] Import failed: {name}", result.stdout)
        self.assertIn("[SUMMARY] Failed imports:", result.stdout)

    def test_gpio_model_fallback_is_present_in_every_python_verification(self):
        fake_modules = self.folder / "modules"
        fake_modules.mkdir()
        for name in ("serial", "numpy", "yaml", "cv2"):
            (fake_modules / f"{name}.py").write_bytes(b"__version__ = 'test'\n")
        (fake_modules / "Jetson").mkdir()
        (fake_modules / "Jetson/__init__.py").write_bytes(b"")
        (fake_modules / "Jetson/GPIO.py").write_bytes(
            b"import os\n"
            b"assert os.environ.get('JETSON_MODEL_NAME') == os.environ['EXPECTED_GPIO_MODEL'], 'Could not determine Jetson model'\n")
        self.environment["PYTHONPATH"] = str(fake_modules)
        for override, expected in ((None, "JETSON_ORIN_NANO"), ("JETSON_ORIN_NX", "JETSON_ORIN_NX")):
            with self.subTest(model=override):
                if override is None:
                    self.environment.pop("JETSON_MODEL_NAME", None)
                else:
                    self.environment["JETSON_MODEL_NAME"] = override
                self.environment["EXPECTED_GPIO_MODEL"] = expected
                result = self.run_worker("verify_python")
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertIn(f"[GPIO] JETSON_MODEL_NAME={expected}", result.stdout)
                self.assertIn("[OK] Jetson.GPIO", result.stdout)

    def test_installed_but_unusable_gpio_is_not_reinstalled_or_reported_as_success(self):
        self.stub("python-test", r'''
echo "$* model=$JETSON_MODEL_NAME" >> "$TRACE_FILE"
if [[ "$1" == -c && "$2" == *'importlib.metadata'* ]]; then echo 2.1.7; exit 0; fi
if [[ "$1" == -c ]]; then echo 'Could not determine Jetson model' >&2; exit 1; fi
echo 'BUG: pip called'; exit 91
''')
        self.environment["PYTHON_BIN"] = (self.bin_dir / "python-test").as_posix()
        result = self.run_worker("jetson_gpio")
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("installed but import failed", result.stdout)
        self.assertNotIn("-m pip", self.folder.joinpath("trace.txt").read_text())

    def test_gpio_installation_requires_a_successful_post_install_import(self):
        self.stub("python-test", r'''
echo "$* model=$JETSON_MODEL_NAME" >> "$TRACE_FILE"
if [[ "$1" == -c && "$2" == *'importlib.metadata'* ]]; then exit 1; fi
if [[ "$1" == -m && "$2" == pip ]]; then exit 0; fi
if [[ "$1" == -c ]]; then echo 'simulated GPIO import failure' >&2; exit 1; fi
exit 92
''')
        self.environment["PYTHON_BIN"] = (self.bin_dir / "python-test").as_posix()
        result = self.run_worker("jetson_gpio")
        self.assertEqual(result.returncode, 1, result.stdout)
        trace = self.folder.joinpath("trace.txt").read_text().splitlines()
        self.assertTrue(any("-m pip install --no-deps Jetson.GPIO" in line for line in trace))
        self.assertEqual(sum(line.startswith("-c import Jetson.GPIO") for line in trace), 2)

    def test_STM32_side_sensors_skip_unneeded_GPIO_install_and_import(self):
        self.environment["SIDE_SENSOR_SOURCE"] = "mcu"
        self.stub("python-test", 'echo "BUG: Python/GPIO install called" >> "$TRACE_FILE"; exit 91\n')
        self.environment["PYTHON_BIN"] = (self.bin_dir / "python-test").as_posix()
        result = self.run_worker("jetson_gpio")
        self.assertEqual(result.returncode, 77, result.stdout)
        self.assertIn("Side sensors are on STM32", result.stdout)
        self.assertFalse((self.folder / "trace.txt").exists())


if __name__ == "__main__":
    unittest.main()
