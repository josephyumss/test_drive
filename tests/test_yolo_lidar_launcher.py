"""Exercise launcher process ownership without ROS, Docker, or physical devices."""

import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]

FAKE_TOOL = r'''#!/usr/bin/python3
import json
import os
from pathlib import Path
import signal
import sys
import time

args = sys.argv[1:]
name = Path(sys.argv[0]).name

def event(message):
    with open(os.environ['LAUNCHER_EVENTS'], 'a') as stream:
        stream.write(message + '\n')

def child(kind, behavior='normal'):
    event('START ' + kind)
    terminated = False
    def stop(signum, frame):
        nonlocal terminated
        terminated = True
        event('TERM ' + kind)
    signal.signal(signal.SIGTERM, stop)
    started = time.monotonic()
    while not terminated:
        elapsed = time.monotonic() - started
        if kind == 'controller':
            if behavior == 'normal' and elapsed > 0.4:
                event('COMPLETE controller')
                return 0
            if behavior == 'killed' and elapsed > 0.25:
                os.kill(os.getpid(), signal.SIGKILL)
        elif os.environ.get('SENSOR_FAILURE') == kind and elapsed > 0.08:
            event('FAIL ' + kind)
            return 2
        time.sleep(0.01)
    return 0

if name == 'docker':
    if args[:2] == ['container', 'inspect']:
        sys.exit(0 if os.environ.get('EXISTING_CONTAINER') else 1)
    if args and args[0] == 'info' and '--format' in args:
        print(json.dumps({'nvidia': {}}))
    if args and args[0] == 'stop':
        event('DOCKER_STOP')
    sys.exit(0)
if name == 'fuser':
    sys.exit(0 if os.environ.get('BUSY_DEVICE') else 1)
if name == 'ros2':
    if args[:2] == ['pkg', 'prefix']:
        sys.exit(0)
    kind = ('lidar' if 'ldlidar_stl_ros2' in args else
            'bridge' if 'amr_vision' in args else 'yolo')
    sys.exit(child(kind))
if name == 'python-stub':
    if args and args[0] == '-c':
        sys.stdin.read()
        sys.exit(0)
    if args and args[0] == '-':
        sys.stdin.read()
        if len(args) > 1:
            event('EMERGENCY_STOP')
        sys.exit(0)
    event('CONTROLLER_ARGS ' + ' '.join(args))
    sys.exit(child('controller', os.environ.get('CONTROLLER_BEHAVIOR', 'normal')))
raise SystemExit('Unexpected stub invocation: ' + name + repr(args))
'''


@unittest.skipUnless(shutil.which("setsid") and shutil.which("flock"),
                     "launcher requires Linux setsid and flock")
class YoloLidarLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="amr-launcher-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.events = self.root / "events"
        self.events.touch()
        for name in ("docker", "fuser", "ros2", "python-stub"):
            program = self.bin / name
            program.write_text(FAKE_TOOL)
            program.chmod(0o755)
        for relative in ("scripts", "models", "usb", "jetson_ws/install",
                         ".runtime/ldlidar_ros2_ws/install", ".run"):
            (self.root / relative).mkdir(parents=True, exist_ok=True)
        for relative in ("ros-setup.bash", "jetson_ws/install/setup.bash",
                         ".runtime/ldlidar_ros2_ws/install/setup.bash",
                         "mcu", "lidar"):
            (self.root / relative).touch()
        (self.root / "models/yolo11n.pt").write_bytes(b"mock-model")
        yolo = self.root / "scripts/start_yolo_docker.sh"
        yolo.write_text("#!/bin/bash\nexec ros2 fake_sensor yolo\n")
        yolo.chmod(0o755)

        # Replace only hardcoded external dependency locations. The launched
        # shell logic, traps, process groups, locking, and waiting stay intact.
        source = (PROJECT / "scripts/start_yolo_lidar_path_avoidance.sh").read_text()
        source = source.replace("/dev/bus/usb", str(self.root / "usb"))
        source = source.replace("/usr/bin/python3", str(self.bin / "python-stub"))
        self.launcher = self.root / "scripts/start_yolo_lidar_path_avoidance.sh"
        self.launcher.write_text(source)
        self.launcher.chmod(0o755)
        self.environment = os.environ.copy()
        for key in ("LIDAR_SETUP", "LIDAR_WS", "SUDO_USER", "BUSY_DEVICE",
                    "EXISTING_CONTAINER", "SENSOR_FAILURE", "CONTROLLER_BEHAVIOR"):
            self.environment.pop(key, None)
        self.environment.update({
            "PATH": str(self.bin) + os.pathsep + os.environ["PATH"],
            "ROS_SETUP": str(self.root / "ros-setup.bash"),
            "MCU_DEVICE": str(self.root / "mcu"),
            "LIDAR_DEVICE": str(self.root / "lidar"),
            "LAUNCHER_EVENTS": str(self.events),
        })

    def run_launcher(self, *args, **overrides):
        environment = dict(self.environment, **overrides)
        result = subprocess.run(["/bin/bash", str(self.launcher), *args],
                                env=environment, stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=15)
        return result, self.events.read_text().splitlines()

    def assert_all_started_sensors_stopped(self, events):
        for sensor in ("lidar", "bridge", "yolo"):
            if "START " + sensor in events and "FAIL " + sensor not in events:
                self.assertIn("TERM " + sensor, events)
        self.assertIn("DOCKER_STOP", events)
        self.assertFalse((self.root / ".run/yolo_lidar_path.pid").exists())

    def test_normal_completion_stops_owned_sensors_and_removes_pid(self):
        result, events = self.run_launcher()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("COMPLETE controller", events)
        self.assert_all_started_sensors_stopped(events)
        self.assertNotIn("EMERGENCY_STOP", events)

    def test_sensor_failure_interrupts_controller_and_cleans_up(self):
        result, events = self.run_launcher(SENSOR_FAILURE="lidar",
                                           CONTROLLER_BEHAVIOR="long")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("FAIL lidar", events)
        self.assertIn("TERM controller", events)
        self.assert_all_started_sensors_stopped(events)

    def test_occupied_serial_refuses_to_start_or_stop_other_runtime(self):
        result, events = self.run_launcher(BUSY_DEVICE="1")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Another process owns", result.stdout)
        self.assertEqual(events, [])

    def test_existing_container_refuses_to_start_or_stop_other_runtime(self):
        result, events = self.run_launcher(EXISTING_CONTAINER="1")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("already exists", result.stdout)
        self.assertEqual(events, [])

    def test_concurrent_run_lock_refuses_second_controller(self):
        lock_path = self.root / ".run/yolo_lidar_path.lock"
        with lock_path.open("w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result, events = self.run_launcher()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("already active", result.stdout)
        self.assertEqual(events, [])

    def test_preflight_passes_zero_motion_mode_and_cleans_up(self):
        result, events = self.run_launcher("--preflight-only")
        self.assertEqual(result.returncode, 0, result.stdout)
        arguments = next(event for event in events if event.startswith("CONTROLLER_ARGS"))
        self.assertIn("--preflight-only", arguments)
        self.assertIn("--open-loop", arguments)
        self.assert_all_started_sensors_stopped(events)

    def test_check_does_not_start_any_runtime_component(self):
        result, events = self.run_launcher("--check")
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(events, [])

    def test_killed_controller_triggers_emergency_stop_and_sensor_cleanup(self):
        result, events = self.run_launcher(CONTROLLER_BEHAVIOR="killed")
        self.assertEqual(result.returncode, 137, result.stdout)
        self.assertIn("EMERGENCY_STOP", events)
        self.assert_all_started_sensors_stopped(events)


if __name__ == "__main__":
    unittest.main()
