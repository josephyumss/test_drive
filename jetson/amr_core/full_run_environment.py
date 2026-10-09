"""Read-only, bounded environment evidence; no device opens or full env dump."""
from datetime import datetime, timezone
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import platform
import shutil
import stat
import subprocess
import sys
import time


SETTINGS = ("MCU_DEVICE", "MCU_STATUS_DEVICE", "MCU_BAUDRATE", "MCU_PROTOCOL",
            "MCU_STATUS_COMMAND_FALLBACK", "LIDAR_DEVICE", "LIDAR_SETUP", "ROS_SETUP",
            "JETSON_SETUP", "FULL_RUN_CONFIG", "JETSON_MODEL_NAME", "ROS_DOMAIN_ID",
            "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION", "YOLO_DEVICE", "YOLO_IMGSZ", "YOLO_HALF")


def read_evidence(path, limit=16384):
    try:
        with Path(path).open("rb") as stream:
            payload = stream.read(limit + 1)
        return {"text": payload[:limit].replace(b"\0", b"\n").decode("utf8", "replace"),
                "truncated": len(payload) > limit}
    except OSError as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}


def command_evidence(argv, timeout=2):
    if not shutil.which(argv[0]):
        return {"argv": argv, "error": "executable_not_found"}
    started = time.monotonic()
    # A temporary spool, not PIPE, bounds memory even for a broken executable.
    import tempfile
    with tempfile.TemporaryFile() as stream:
        try:
            result = subprocess.run(argv, stdout=stream, stderr=subprocess.STDOUT,
                                    timeout=timeout, check=False)
            code, error = result.returncode, None
        except (OSError, subprocess.TimeoutExpired) as exc:
            code, error = None, f"{type(exc).__name__}: {exc}"
        stream.seek(0)
        payload = stream.read(32769)
    return {"argv": argv, "exit_code": code, "error": error,
            "elapsed_s": time.monotonic() - started,
            "output": payload[:32768].decode("utf8", "replace"), "truncated": len(payload) > 32768}


def file_identity(path):
    path = Path(path)
    result = {"requested": str(path), "resolved": os.path.realpath(path),
              "readable": os.access(path, os.R_OK), "writable": os.access(path, os.W_OK)}
    try:
        info = path.stat()
        result.update(mode=oct(stat.S_IMODE(info.st_mode)), uid=info.st_uid, gid=info.st_gid,
                      size=info.st_size, character_device=stat.S_ISCHR(info.st_mode))
    except OSError as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def collect_environment(project, *, detailed=False):
    project = Path(project).resolve()
    result = {"utc": datetime.now(timezone.utc).isoformat(), "project": str(project),
              "python": sys.version, "python_executable": sys.executable,
              "platform": platform.platform(), "pid": os.getpid(),
              "settings_allowlist_only": {key: os.environ.get(key) for key in SETTINGS},
              "safety": "Read-only metadata. No serial/GPIO opening, pinmux changes or service control.",
              "files": {}, "devices": {}, "packages": {}, "commands": {}}
    if hasattr(os, "getuid"):
        result["identity"] = {"uid": os.getuid(), "gid": os.getgid(), "groups": os.getgroups()}
    for path in ("/etc/os-release", "/etc/nv_tegra_release", "/proc/device-tree/model",
                 "/proc/device-tree/compatible", "/proc/meminfo", "/proc/loadavg", "/proc/uptime"):
        result["files"][path] = read_evidence(path)
    try:
        result["disk"] = dict(zip(("total", "used", "free"), shutil.disk_usage(project)))
    except OSError as exc:
        result["disk"] = {"error": str(exc)}
    for distribution in ("pyserial", "Jetson.GPIO", "numpy", "PyYAML", "depthai", "ultralytics"):
        try:
            result["packages"][distribution] = metadata.version(distribution)
        except metadata.PackageNotFoundError:
            result["packages"][distribution] = "not_installed_in_this_interpreter"
        except Exception as exc:
            result["packages"][distribution] = f"{type(exc).__name__}: {exc}"
    defaults = {"MCU_DEVICE": "/dev/ttyTHS1", "MCU_STATUS_DEVICE": "/dev/ttyTHS2",
                "LIDAR_DEVICE": "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"}
    for key, default in defaults.items():
        device = os.environ.get(key, default)
        entry = file_identity(device)
        sysfs = Path("/sys/class/tty") / Path(entry["resolved"]).name / "device"
        entry["sysfs"] = {part: os.path.realpath(sysfs / part) for part in (".", "driver", "of_node")}
        entry["owners"] = command_evidence(["fuser", "-v", device])
        result["devices"][key] = entry
    # Versions/status only: no remote URL, credentials, full process argv/env,
    # private config/full_run.env or camera images are collected.
    commands = {"git_head": ["git", "-c", f"safe.directory={project.as_posix()}", "-C", str(project), "rev-parse", "HEAD"],
                "git_branch": ["git", "-c", f"safe.directory={project.as_posix()}", "-C", str(project), "branch", "--show-current"],
                "git_dirty": ["git", "-c", f"safe.directory={project.as_posix()}", "-C", str(project), "status", "--porcelain", "--untracked-files=no"],
                "services": ["systemctl", "show", "amr-full-run.service", "amr-core.service", "nvgetty.service",
                             "-p", "ActiveState", "-p", "SubState", "-p", "MainPID", "-p", "Result"],
                "usb": ["lsusb"],
                "docker_state": ["docker", "container", "inspect", "socialguide-amr-yolo", "--format", "{{json .State}}"]}
    if detailed:
        commands["kernel_recent"] = ["journalctl", "-k", "-n", "100", "--no-pager", "-o", "short-iso"]
        commands["service_recent"] = ["journalctl", "-u", "amr-full-run.service", "-n", "60", "--no-pager", "-o", "short-iso"]
    for name, argv in commands.items():
        result["commands"][name] = command_evidence(argv)
    source = project / "jetson_ws/src/amr_vision/amr_vision/yolo_udp_bridge_node.py"
    try:
        result["bridge_source"] = {"path": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
    except OSError as exc:
        result["bridge_source"] = {"error": str(exc)}
    result["finished_utc"] = datetime.now(timezone.utc).isoformat()
    return result


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project")
    parser.add_argument("--detailed", action="store_true")
    args = parser.parse_args()
    print(json.dumps(collect_environment(args.project, detailed=args.detailed), ensure_ascii=False, indent=2))
