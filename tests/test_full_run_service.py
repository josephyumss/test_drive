"""Service registration with inert commands; never installs/starts real services."""
import csv
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASH = shutil.which("bash") if os.name == "posix" else r"C:\Program Files\Git\bin\bash.exe"
HAS_BASH = bool(BASH and Path(BASH).is_file())

SYSTEMCTL = r'''
printf 'systemctl %s\n' "$*" >> "$SERVICE_TEST_TRACE"
case "$1" in
    is-active) [[ "${SERVICE_TEST_OLD:-missing}" == active ]] ;;
    is-enabled)
        if [[ "${SERVICE_TEST_OLD:-missing}" == enabled ]]; then exit 0; fi
        echo 'Failed to get unit file state for amr-core.service: No such file or directory' >&2
        exit 1 ;;
    daemon-reload) echo 'daemon reload stub'; exit "${SERVICE_TEST_RELOAD_EXIT:-0}" ;;
    enable) echo 'enable stub'; exit "${SERVICE_TEST_ENABLE_EXIT:-0}" ;;
    *) echo "Unexpected command (no real service operation): $*" >&2; exit 98 ;;
esac
'''

ANALYZER = r'''
printf 'systemd-analyze %s\n' "$*" >> "$SERVICE_TEST_TRACE"
if [[ "$1" == --version ]]; then
    echo 'systemd 249 (inert test stub)'
    exit "${SERVICE_TEST_VERSION_EXIT:-0}"
fi
[[ "$1" == verify && $# == 2 ]] || exit 98
# Regression predicate matches systemd v249 config_parse_working_directory:
# the raw RHS is passed to the absolute-path check, WITHOUT unquoting.
working_dir="$(sed -n 's/^WorkingDirectory=//p' "$2")"
if [[ "$working_dir" != /* ]]; then
    echo "$2: WorkingDirectory= path is not absolute: $working_dir" >&2
    echo 'Unit configuration has fatal error, unit will not be started.' >&2
    exit 1
fi
echo 'Unrelated OS unit warning, ignoring.' >&2
exit "${SERVICE_TEST_VERIFY_EXIT:-0}"
'''

INSTALL = r'''
printf 'install %s\n' "$*" >> "$SERVICE_TEST_TRACE"
[[ $# == 4 && "$1" == -m && "$2" == 644 && "$4" == /etc/systemd/system/amr-full-run.service ]] || exit 98
if [[ "${SERVICE_TEST_INSTALL_EXIT:-0}" != 0 ]]; then exit "$SERVICE_TEST_INSTALL_EXIT"; fi
# The only destination is the disposable fixture, NEVER the real /etc tree.
cp -- "$3" "$SERVICE_TEST_INSTALLED"
'''


class ServiceTemplateTests(unittest.TestCase):
    def test_working_directory_is_a_raw_path_not_a_quoted_argument(self):
        lines = (ROOT / "deploy/amr-full-run.service").read_text().splitlines()
        self.assertEqual([line for line in lines if line.startswith("WorkingDirectory=")],
                         ["WorkingDirectory=@PROJECT_DIR@"])
        self.assertIn('ExecStart=/bin/bash "@PROJECT_DIR@/scripts/start_full_run.sh"', lines)
        self.assertIn("KillMode=control-group", lines)
        self.assertIn("SendSIGKILL=yes", lines)

    @unittest.skipUnless(HAS_BASH, "No runnable Bash")
    def test_path_guard_accepts_internal_spaces_and_rejects_ambiguous_values(self):
        source = (ROOT / "scripts/install_full_run_service.sh").read_text()
        guard = source[source.index('[[ "$PROJECT_DIR" !='):
                       source.index(" || {\n    echo 'Unsupported character")]
        cases = [("/home/user/robot & checkout", True), ("/home/user/robot%checkout", False),
                 ("/home/user/robot\\checkout", False), ("/home/user/robot ", False),
                 ('/home/user/robot"checkout', False), ("/home/user/robot\ncheckout", False),
                 ("/home/user/robot\rcheckout", False)]
        for path, expected in cases:
            with self.subTest(path=path):
                # Bash literals avoid MSYS rewriting control characters in
                # Windows argv paths before the actual guard can inspect them.
                literal = shlex.quote(path)
                if "\n" in path or "\r" in path:
                    literal = "$'" + path.replace("\n", "\\n").replace("\r", "\\r") + "'"
                script = f"PROJECT_DIR={literal}\nif {guard}; then exit 0; else exit 1; fi"
                result = subprocess.run([BASH, "-c", script], capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode == 0, expected, result.stderr)

    @unittest.skipUnless(os.name == "posix" and shutil.which("systemd-analyze"),
                         "requires actual Linux systemd-analyze")
    def test_actual_systemd_parser_accepts_raw_path_and_rejects_old_quotes(self):
        with tempfile.TemporaryDirectory(prefix="full run systemd ") as temporary:
            folder = Path(temporary)
            unit = folder / "amr-full-run.service"
            # Minimal inert unit isolates this setting from host Docker units.
            # verify only parses it; /bin/true is never executed.
            for quoted, expected in ((True, False), (False, True)):
                working_dir = f'"{folder}"' if quoted else str(folder)
                unit.write_text(f"[Service]\nType=oneshot\nWorkingDirectory={working_dir}\nExecStart=/bin/true\n")
                result = subprocess.run(["systemd-analyze", "verify", str(unit)],
                                        capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode == 0, expected, result.stderr)


@unittest.skipUnless(HAS_BASH, "No runnable Bash")
class ServiceInstallerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="full-run-service-")
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.folder = self.base / "robot & checkout"
        self.bin = self.base / "bin"
        self.bin.mkdir()
        for name in ("scripts", "deploy", "jetson/amr_core"):
            (self.folder / name).mkdir(parents=True, exist_ok=True)
        for name in ("install_full_run_service.sh", "full_run_log_access.sh", "full_run_diagnostics.py"):
            shutil.copyfile(ROOT / "scripts" / name, self.folder / "scripts" / name)
        for name in ("full_run_diagnostics.py", "full_run_environment.py", "full_run_summary.py"):
            shutil.copyfile(ROOT / "jetson/amr_core" / name, self.folder / "jetson/amr_core" / name)
        shutil.copyfile(ROOT / "deploy/amr-full-run.service", self.folder / "deploy/amr-full-run.service")
        self.source = (self.folder / "scripts/install_full_run_service.sh").read_text()
        # Simulate privilege in this disposable copy only. All mutating tools
        # remain inert and no production root bypass/environment option exists.
        self.set_privilege_check(True)
        self.stub("systemctl", SYSTEMCTL)
        self.stub("systemd-analyze", ANALYZER)
        self.stub("install", INSTALL)
        self.stub("python3", 'exec "$SERVICE_TEST_PYTHON" "$@"\n')
        self.trace = self.base / "trace.txt"
        self.installed = self.base / "installed.service"
        self.environment = dict(os.environ, SERVICE_TEST_BIN=self.bin.as_posix(),
                                SERVICE_TEST_TRACE=self.trace.as_posix(),
                                SERVICE_TEST_INSTALLED=self.installed.as_posix(),
                                SERVICE_TEST_PYTHON=sys.executable.replace("\\", "/"))
        # Parent test/real deployment variables must not alter this fixture.
        for key in tuple(self.environment):
            if key.startswith("SERVICE_TEST_") and key not in {
                    "SERVICE_TEST_BIN", "SERVICE_TEST_TRACE", "SERVICE_TEST_INSTALLED", "SERVICE_TEST_PYTHON"}:
                self.environment.pop(key)

    def set_privilege_check(self, allowed):
        replacement = "[[ 1 == 1 ]]" if allowed else "[[ 1 == 0 ]]"
        source = self.source.replace("(( EUID == 0 ))", replacement, 1)
        self.folder.joinpath("scripts/install_full_run_service.sh").write_bytes(source.encode())

    def stub(self, name, source):
        path = self.bin / name
        path.write_bytes(("#!/usr/bin/env bash\n" + source).encode())
        path.chmod(0o755)

    def run_install(self, **overrides):
        result = subprocess.run([BASH, "-c", '''
if command -v cygpath >/dev/null; then service_test_bin="$(cygpath -u "$SERVICE_TEST_BIN")";
else service_test_bin="$SERVICE_TEST_BIN"; fi
export PATH="$service_test_bin:$PATH"
exec /bin/bash "$1"
''', "test-service-installer", str(self.folder / "scripts/install_full_run_service.sh")],
                                env=dict(self.environment, **overrides), stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=30)
        log_root = self.folder / "logs/full_run_setup"
        self.log_dir = max((p for p in log_root.iterdir() if p.is_dir()), key=lambda p: p.stat().st_mtime_ns)
        with (self.log_dir / "stages.tsv").open(newline="") as stream:
            self.stages = {row["step"]: row for row in csv.DictReader(stream, delimiter="\t")}
        self.calls = self.trace.read_text().splitlines() if self.trace.exists() else []
        self.assertFalse(any(call.startswith(("systemctl start", "systemctl stop", "systemctl restart",
                                             "systemctl disable")) for call in self.calls))
        return result

    def archive_files(self):
        archive = Path(str(self.log_dir) + "-debug.tar.gz")
        self.assertTrue(archive.is_file())
        with tarfile.open(archive) as t:
            return {p.name.split("/", 1)[1]: t.extractfile(p).read() for p in t.getmembers() if p.isfile()}

    def test_success_retains_exact_generated_unit_and_does_not_start_robot(self):
        result = self.run_install()
        self.assertEqual(result.returncode, 0, result.stdout)
        generated = self.log_dir / "amr-full-run.service"
        self.assertEqual(generated.read_bytes(), self.installed.read_bytes())
        working_dir = next(line.split("=", 1)[1] for line in generated.read_text().splitlines()
                           if line.startswith("WorkingDirectory="))
        self.assertTrue(working_dir.startswith("/"))
        self.assertTrue(working_dir.endswith("robot & checkout"))
        self.assertNotIn('"', working_dir)
        self.assertEqual(set(self.stages), {"systemd_version", "verify_unit", "install_unit", "daemon_reload", "enable_unit"})
        self.assertTrue(all(stage["status"] == "OK" for stage in self.stages.values()))
        self.assertIn("Unrelated OS unit warning", result.stdout)
        self.assertNotIn("No such file or directory", result.stdout)
        self.assertIn("No such file or directory", (self.log_dir / "old-service-check.log").read_text())
        self.assertFalse(Path(str(self.log_dir) + "-debug.tar.gz").exists())
        self.assertIn("Boot auto-start enabled", result.stdout)
        self.assertTrue((self.log_dir / "unit-source.sha256").is_file())

    def test_old_quoted_template_reproduces_failure_and_keeps_unit_in_bundle(self):
        unit = self.folder / "deploy/amr-full-run.service"
        unit.write_bytes(unit.read_bytes().replace(b"WorkingDirectory=@PROJECT_DIR@",
                                                 b'WorkingDirectory="@PROJECT_DIR@"'))
        result = self.run_install()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("WorkingDirectory= path is not absolute", result.stdout)
        self.assertEqual(self.stages["verify_unit"]["status"], "FAILED")
        self.assertFalse(self.installed.exists())
        self.assertFalse(any(call.startswith("systemctl daemon-reload") for call in self.calls))
        archive = self.archive_files()
        self.assertIn(b'WorkingDirectory="', archive["amr-full-run.service"])
        self.assertIn(b"not absolute", archive["verify_unit.log"])
        self.assertIn(b"systemd 249", archive["systemd_version.log"])
        self.assertIn(b"stage=verify_unit", archive["launcher.log"])
        summary = json.loads(archive["diagnostic_summary.json"])
        self.assertEqual([row["step"] for row in summary["failed_or_blocked_setup_stages"]], ["verify_unit"])

    def test_systemd_tool_failure_is_archived_before_any_unit_install(self):
        result = self.run_install(SERVICE_TEST_VERSION_EXIT="127")
        self.assertEqual(result.returncode, 127, result.stdout)
        self.assertEqual(self.stages["systemd_version"]["status"], "FAILED")
        self.assertFalse(self.installed.exists())
        self.assertNotIn("verify_unit", self.stages)
        self.assertIn(b"stage=systemd_version", self.archive_files()["launcher.log"])

    def test_arbitrary_validation_failure_cannot_install_or_enable(self):
        result = self.run_install(SERVICE_TEST_VERIFY_EXIT="23")
        self.assertEqual(result.returncode, 23, result.stdout)
        self.assertFalse(self.installed.exists())
        self.assertEqual(self.stages["verify_unit"]["exit_code"], "23")
        self.assertIn("[BUNDLE] Saved:", result.stdout)
        self.assertIn("unit-template.service", self.archive_files())

    def test_install_failure_cannot_reload_or_enable(self):
        result = self.run_install(SERVICE_TEST_INSTALL_EXIT="43")
        self.assertEqual(result.returncode, 43, result.stdout)
        self.assertEqual(self.stages["install_unit"]["status"], "FAILED")
        self.assertNotIn("daemon_reload", self.stages)
        self.assertFalse(self.installed.exists())
        self.assertIn("install_unit.log", self.archive_files())

    def test_reload_failure_cannot_enable(self):
        result = self.run_install(SERVICE_TEST_RELOAD_EXIT="44")
        self.assertEqual(result.returncode, 44, result.stdout)
        self.assertEqual(self.stages["daemon_reload"]["exit_code"], "44")
        self.assertNotIn("enable_unit", self.stages)
        self.assertIn(b"stage=daemon_reload", self.archive_files()["launcher.log"])

    def test_enable_failure_is_not_reported_as_success(self):
        result = self.run_install(SERVICE_TEST_ENABLE_EXIT="45")
        self.assertEqual(result.returncode, 45, result.stdout)
        self.assertEqual(self.stages["enable_unit"]["status"], "FAILED")
        self.assertNotIn("Boot auto-start enabled", result.stdout)
        self.assertIn("enable_unit.log", self.archive_files())

    def test_active_old_service_is_refused_without_stopping_it(self):
        result = self.run_install(SERVICE_TEST_OLD="active")
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("active or enabled", result.stdout)
        self.assertFalse(self.installed.exists())
        self.assertFalse(any(call.startswith("systemd-analyze") for call in self.calls))
        self.assertIn(b"stage=check_old_device_owner", self.archive_files()["launcher.log"])

    def test_enabled_old_service_is_refused_even_when_inactive(self):
        result = self.run_install(SERVICE_TEST_OLD="enabled")
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("active or enabled", result.stdout)
        self.assertFalse(self.installed.exists())
        self.assertTrue(self.archive_files())

    def test_unprivileged_invocation_is_refused_and_archived(self):
        self.set_privilege_check(False)
        result = self.run_install()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("Run: sudo bash", result.stdout)
        self.assertFalse(self.calls)
        self.assertIn(b"stage=check_privileges_and_path", self.archive_files()["launcher.log"])

    def test_unsupported_specifier_path_is_rejected_before_system_mutation(self):
        changed = self.base / "robot%checkout"
        self.folder.rename(changed)
        self.folder = changed
        result = self.run_install()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("Unsupported character", result.stdout)
        self.assertFalse(self.calls)
        self.assertFalse(self.installed.exists())
        self.assertTrue(self.archive_files())

    def test_rerun_is_idempotent_and_preserves_previous_attempt_logs(self):
        first = self.run_install()
        first_dir = self.log_dir
        unit = self.installed.read_bytes()
        second = self.run_install()
        self.assertEqual(first.returncode, 0, first.stdout)
        self.assertEqual(second.returncode, 0, second.stdout)
        self.assertEqual(self.installed.read_bytes(), unit)
        self.assertNotEqual(self.log_dir, first_dir)
        self.assertTrue((first_dir / "verify_unit.log").exists())


if __name__ == "__main__":
    unittest.main()
