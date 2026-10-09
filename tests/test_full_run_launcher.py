"""Real Linux shell/process ownership checks with fake robot devices."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

if os.name == "posix":
    import fcntl

ROOT = Path(__file__).resolve().parents[1]
FAKE = r'''#!/usr/bin/python3
import json, os, signal, sys, time
from pathlib import Path
name=Path(sys.argv[0]).name
args=sys.argv[1:]
def event(s):
    with open(os.environ['EVENTS'], 'a') as f: f.write(s+'\n')
def run(kind):
    event('START '+kind)
    stopped=False
    fault_written=False
    def stop(*_):
        nonlocal stopped
        stopped=True
        event('TERM '+kind)
    signal.signal(signal.SIGTERM, stop)
    started=time.monotonic()
    while not stopped:
        if kind=='controller' and os.environ.get('BEHAVIOR')=='fault' and not fault_written and time.monotonic()-started>.10:
            log_dir=Path(args[args.index('--log-dir')+1])
            (log_dir/'auto_bundle.request.json').write_text(json.dumps({'reason':'sensor_data_lost','source':'controller'}))
            event('FAULT controller')
            fault_written=True
        if kind == os.environ.get('SENSOR_FAILURE') and time.monotonic()-started>.10:
            event('FAIL '+kind)
            return 2
        if kind=='controller' and time.monotonic()-started>.5:
            if os.environ.get('BEHAVIOR')=='killed': os.kill(os.getpid(), signal.SIGKILL)
            if os.environ.get('BEHAVIOR') not in ('long','fault'):
                event('COMPLETE controller')
                return 0
        time.sleep(.01)
    return 0
if name=='docker':
    if args[:2]==['container','inspect'] and '--format' in args:
        print(os.environ.get('YOLO_RUNTIME_OWNER',''))
        sys.exit(0)
    if args[:2]==['container','inspect']: sys.exit(0 if os.environ.get('CONTAINER') else 1)
    if args[:1]==['info']: print(json.dumps({'nvidia':{}}))
    if args[:1]==['stop']: event('DOCKER_STOP')
    sys.exit(0)
if name=='fuser': sys.exit(0 if os.environ.get('BUSY') else 1)
if name=='ros2':
    if args[:2]==['pkg','prefix']: sys.exit(0)
    sys.exit(run('lidar' if 'ldlidar_stl_ros2' in args else 'bridge'))
if name=='python-stub':
    if any(a.endswith('full_run_diagnostics.py') for a in args):
        event('AUTO_BUNDLE')
        os.execv('/usr/bin/python3', ['/usr/bin/python3', *args])
    if '-c' in args or '--version' in args or '--check-config' in args:
        sys.stdin.read() if not sys.stdin.isatty() and '-c' in args else None
        sys.exit(0)
    if args and args[0]=='-':
        sys.stdin.read()
        if len(args)>1: event('BRAKE')
        sys.exit(0)
    event('ARGS '+' '.join(args))
    sys.exit(run('controller'))
if name=='yolo-stub': sys.exit(run('yolo'))
sys.exit('unexpected '+name+repr(args))
'''


@unittest.skipUnless(os.name == "posix" and shutil.which("setsid") and shutil.which("flock"),
                     "requires Linux process groups")
class FullRunLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="full-run-launcher-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        for name in ("docker", "fuser", "ros2", "python-stub", "yolo-stub"):
            path = self.bin / name
            path.write_text(FAKE)
            path.chmod(0o755)
        for directory in ("scripts", "config", "models", "usb", "jetson_ws/install", "jetson/amr_core", "docker",
                          "jetson_ws/src/amr_vision/amr_vision", "jetson_ws/src/amr_vision/config",
                          "stm32/Core/Src", "stm32/Core/Inc", "protocol", "logs", ".run"):
            (self.root / directory).mkdir(parents=True, exist_ok=True)
        for path in ("ros.bash", "lidar.bash", "jetson_ws/install/setup.bash", "mcu", "status", "lidar",
                     "jetson/amr_core/full_run.py", "jetson/amr_core/reactive_avoidance.py",
                     "jetson/amr_core/full_run_log.py", "jetson/amr_core/full_run_ultrasonic.py",
                     "jetson/amr_core/full_run_legacy.py", "jetson/amr_core/full_run_user_stop.py",
                     "jetson/amr_core/ascii_serial_bridge.py", "jetson/amr_core/serial_bridge.py",
                     "jetson/amr_core/transport.py", "jetson/amr_core/packet.py", "jetson/amr_core/crc16.py",
                     "protocol/protocol_constants.py", "scripts/full_run_user_stop.py",
                     "docker/oak_yolo_udp.py", "jetson_ws/src/amr_vision/amr_vision/yolo_udp_bridge_node.py",
                     "jetson_ws/src/amr_vision/config/yolo.yaml",
                     "stm32/Core/Src/main.c", "stm32/Core/Inc/full_run_control.h"):
            (self.root / path).touch()
        (self.root / "models/yolo11n.pt").write_bytes(b"model")
        shutil.copyfile(ROOT / "config/full_run.json", self.root / "config/full_run.json")
        shutil.copyfile(ROOT / "scripts/stop_full_run.sh", self.root / "scripts/stop_full_run.sh")
        shutil.copyfile(ROOT / "scripts/full_run_diagnostics.py", self.root / "scripts/full_run_diagnostics.py")
        shutil.copyfile(ROOT / "jetson/amr_core/full_run_diagnostics.py", self.root / "jetson/amr_core/full_run_diagnostics.py")
        (self.root / "scripts/full_run_controller.py").touch()
        (self.root / "scripts/start_yolo_docker.sh").write_text(f'#!/bin/bash\nexec "{self.bin / "yolo-stub"}"\n')
        source = (ROOT / "scripts/start_full_run.sh").read_text().replace("/dev/bus/usb", str(self.root / "usb"))
        self.launcher = self.root / "scripts/start_full_run.sh"
        self.launcher.write_text(source)
        self.environment = dict(os.environ, PATH=str(self.bin)+os.pathsep+os.environ["PATH"],
                                ROS_SETUP=str(self.root / "ros.bash"), LIDAR_SETUP=str(self.root / "lidar.bash"),
                                PYTHON_BIN=str(self.bin / "python-stub"), MCU_DEVICE=str(self.root / "mcu"),
                                MCU_STATUS_DEVICE=str(self.root / "status"), LIDAR_DEVICE=str(self.root / "lidar"),
                                EVENTS=str(self.root / "events"))

    def run_launcher(self, option=None, **overrides):
        args = ["/bin/bash", str(self.launcher)] + ([option] if option else [])
        result = subprocess.run(args, env=dict(self.environment, **overrides), stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=15)
        return result, self.events()

    def events(self):
        path = self.root / "events"
        return path.read_text().splitlines() if path.exists() else []

    def check_stopped(self, events):
        for name in ("lidar", "bridge", "yolo"):
            if "START "+name in events and "FAIL "+name not in events:
                self.assertIn("TERM "+name, events)
        self.assertIn("BRAKE", events)
        self.assertIn("DOCKER_STOP", events)
        self.assertFalse((self.root / ".run/full_run.pid").exists())

    def test_normal_cleanup_and_logs(self):
        result, events = self.run_launcher()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.check_stopped(events)
        folder = self.root / "logs/full_run/latest"
        self.assertTrue((folder / "config.json").exists())
        self.assertTrue((folder / "source.sha256").exists())
        self.assertTrue((folder / "system.txt").exists())
        self.assertFalse(list((self.root / "logs/full_run").glob("*-debug.tar.gz")))

    def test_preflight_never_passes_open_loop(self):
        result, events = self.run_launcher("--preflight-only")
        self.assertEqual(result.returncode, 0, result.stdout)
        args = next(v for v in events if v.startswith("ARGS "))
        self.assertIn("--preflight-only", args)
        self.assertIn("--status-port", args)
        self.assertNotIn("--open-loop", args)
        self.check_stopped(events)

    def test_check_never_starts_or_sends_commands(self):
        result, events = self.run_launcher("--check")
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(events, [])

    def test_busy_port_and_existing_container_are_not_stopped(self):
        for overrides in ({"BUSY":"1"}, {"CONTAINER":"1"}):
            result, events = self.run_launcher(**overrides)
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertTrue(all(event == "AUTO_BUNDLE" for event in events), events)
            self.assertTrue(list((self.root / "logs/full_run").glob("*-debug.tar.gz")))

    def test_concurrent_run_rejected(self):
        with (self.root / ".run/full_run.lock").open("w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result, events = self.run_launcher()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(events, ["AUTO_BUNDLE"])

    def test_sensor_failure_stops_controller(self):
        result, events = self.run_launcher(SENSOR_FAILURE="lidar", BEHAVIOR="long")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("TERM controller", events)
        self.check_stopped(events)
        self.assertIn("AUTO_BUNDLE", events)
        self.assertTrue(list((self.root / "logs/full_run").glob("*-debug.tar.gz")))

    def test_killed_controller_sends_fallback_brake(self):
        result, events = self.run_launcher(BEHAVIOR="killed")
        self.assertEqual(result.returncode, 137, result.stdout)
        self.check_stopped(events)
        self.assertIn("AUTO_BUNDLE", events)

    def test_fault_while_controller_remains_alive_is_bundled_once(self):
        import time
        process = subprocess.Popen(["/bin/bash", str(self.launcher)], cwd=self.root,
                                   env=dict(self.environment, BEHAVIOR="fault"), stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
        try:
            deadline = time.monotonic() + 8
            while not list((self.root / "logs/full_run").glob("*-debug.tar.gz")) and time.monotonic() < deadline:
                time.sleep(.02)
            archives = list((self.root / "logs/full_run").glob("*-debug.tar.gz"))
            self.assertEqual(len(archives), 1, self.events())
            original = archives[0].read_bytes()
            self.assertIsNone(process.poll())
            result = subprocess.run(["/bin/bash", str(self.root / "scripts/stop_full_run.sh")],
                                    env=self.environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertEqual(process.wait(timeout=10), 0)
            self.assertEqual(archives[0].read_bytes(), original)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)

    def test_manual_relative_path_stop_and_restart(self):
        process = subprocess.Popen(["/bin/bash", "scripts/start_full_run.sh"], cwd=self.root,
                                   env=dict(self.environment, BEHAVIOR="long"), stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
        import time
        deadline = time.monotonic()+5
        while "START controller" not in self.events() and time.monotonic() < deadline:
            time.sleep(.02)
        try:
            result = subprocess.run(["/bin/bash", "scripts/stop_full_run.sh"], cwd=self.root,
                                    env=self.environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertEqual(process.wait(timeout=10), 0)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
        self.check_stopped(self.events())
        result, _ = self.run_launcher()
        self.assertEqual(result.returncode, 0, result.stdout)


if __name__ == "__main__":
    unittest.main()
