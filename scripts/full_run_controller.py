#!/usr/bin/env python3
"""Live ROS/GPIO/UART adapter for the testable full-run controller."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import queue
import secrets
import signal
import sys
import time
import traceback

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from jetson.amr_core.full_run import ControlStatus, FullRunConfig, FullRunController
from jetson.amr_core.full_run_log import FlightRecorder
from jetson.amr_core.full_run_diagnostics import request_auto_bundle
from jetson.amr_core.full_run_ultrasonic import UltrasonicWorker, McuUltrasonicFeed
from jetson.amr_core.full_run_legacy import LegacyControlAdapter, LegacyStatusLayoutChanged
from jetson.amr_core.full_run_user_stop import read_stop_request
from jetson.amr_core.full_run_uart import StatusReceiver
from jetson.amr_core.reactive_avoidance import wrap_angle


def parse_scan(message, config, metrics=None):
    if (not all(math.isfinite(v) for v in (message.angle_min, message.angle_increment,
                                          message.range_min, message.range_max))
            or message.angle_increment == 0 or not 0 <= message.range_min < message.range_max):
        raise ValueError("Invalid LaserScan geometry")
    points, front, covered = [], [], {"left": 0, "right": 0}
    valid_points = self_filtered_points = 0
    yaw = math.radians(config.lidar_yaw_deg)
    for index, distance in enumerate(message.ranges):
        angle = wrap_angle(message.angle_min + index * message.angle_increment + yaw)
        valid = math.isfinite(distance) and message.range_min <= distance <= message.range_max and distance > 0
        clear = distance == math.inf
        if not valid and not clear:
            continue
        if math.radians(70) <= angle <= math.radians(110):
            covered["left"] += 1
        if -math.radians(110) <= angle <= -math.radians(70):
            covered["right"] += 1
        if valid:
            x = config.lidar_x_m + distance * math.cos(angle)
            y = config.lidar_y_m + distance * math.sin(angle)
            valid_points += 1
            # A return inside the configured chassis envelope is the robot
            # itself, not traversable-space evidence.  Keeping these returns
            # made side/rear chassis reflections look like obstacles in front
            # of the bumper.  Do not add a margin here: points immediately
            # outside the measured footprint remain safety obstacles.
            if (abs(x) <= config.robot_length_m / 2
                    and abs(y) <= config.robot_width_m / 2):
                self_filtered_points += 1
                continue
            robot_angle, robot_range = math.atan2(y, x), math.hypot(x, y)
            points.append((robot_angle, robot_range))
            if abs(robot_angle) <= math.radians(15):
                front.append(robot_range)
        elif abs(angle) <= math.radians(15):
            front.append(message.range_max)
    if not all(covered.values()) or not front:
        raise ValueError(f"Missing valid front/side scan coverage: {covered}")
    if metrics is not None:
        metrics.update(valid_points=valid_points,
                       self_filtered_points=self_filtered_points,
                       retained_points=len(points), front_samples=len(front),
                       angular_coverage=covered)
    return sorted(points), min(front)


def make_node(config, core, recorder):
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    class FullRunNode(Node):
        def __init__(self):
            super().__init__("amr_full_run")
            self.counts = Counter()
            self.received = {}
            self.last_scan_metrics = {}
            self.create_subscription(String, "/yolo/detections", self.camera, 10)
            self.create_subscription(LaserScan, "/scan", self.scan, qos_profile_sensor_data)

        def diagnostics_snapshot(self):
            result = {"counts": dict(self.counts), "last_received_monotonic_s": dict(self.received),
                      "last_scan": dict(self.last_scan_metrics),
                      "publishers": {}}
            for topic in ("/scan", "/yolo/detections"):
                try:
                    result["publishers"][topic] = [
                        {"node": p.node_name, "namespace": p.node_namespace, "type": p.topic_type,
                         "qos": str(p.qos_profile)} for p in self.get_publishers_info_by_topic(topic)[:20]]
                except Exception as exc:
                    result["publishers"][topic] = {"inspection_error": repr(exc)}
            return result

        def camera(self, message):
            now = time.monotonic()
            self.counts["camera_received"] += 1
            self.received["camera"] = now
            recorder.emit("yolo_raw", payload=message.data)
            try:
                detections = json.loads(message.data)
                if not isinstance(detections, list) or any(not isinstance(d, dict) for d in detections):
                    raise ValueError("Expected a list of detection objects")
                core.update_camera(detections, now)
                self.counts["camera_valid"] += 1
            except (ValueError, TypeError, KeyError) as exc:
                core.camera_stamp = None
                self.counts["camera_invalid"] += 1
                recorder.emit("yolo_invalid", error=str(exc))

        def scan(self, message):
            now = time.monotonic()
            self.counts["lidar_received"] += 1
            self.received["lidar"] = now
            recorder.emit("scan_raw", frame_id=message.header.frame_id,
                          source_stamp={"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec},
                          angle_min=message.angle_min, angle_increment=message.angle_increment,
                          range_min=message.range_min, range_max=message.range_max, ranges=list(message.ranges))
            try:
                scan_metrics = {}
                points, front = parse_scan(message, config, scan_metrics)
                self.last_scan_metrics = scan_metrics
                self.counts["lidar_self_filtered_points"] += scan_metrics["self_filtered_points"]
                core.update_scan(points, front, now)
                self.counts["lidar_valid"] += 1
            except (ValueError, TypeError) as exc:
                core.scan_stamp = None
                self.counts["lidar_invalid"] += 1
                recorder.emit("scan_invalid", error=str(exc))
    return FullRunNode()


def main(arguments=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT / "config/full_run.json"))
    parser.add_argument("--port", default="/dev/ttyTHS1")
    parser.add_argument("--status-port", default="/dev/ttyTHS2")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--mcu-protocol", choices=("legacy", "ctrl"), default="legacy",
                        help="Default legacy uses existing $CMD/$STATUS; no firmware update")
    parser.add_argument("--side-sensor-source", choices=("mcu", "gpio"), default="mcu",
                        help="Default mcu: side centimetres in confirmed compact STATUS; gpio only for Jetson-wired sensors")
    parser.add_argument("--no-command-status-fallback", action="store_true",
                        help="Receive telemetry only on --status-port; do not also inspect command UART RX")
    parser.add_argument("--user-stop-file")
    parser.add_argument("--log-dir", required=True)
    parser.add_argument("--supervisor-pid", type=int)
    parser.add_argument("--supervisor-start-ticks")
    parser.add_argument("--check-config", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args(arguments)
    if args.side_sensor_source == "mcu" and args.mcu_protocol != "legacy":
        parser.error("MCU side sensors require legacy compact STATUS; extended ctrl currently requires gpio")
    config = FullRunConfig.read(args.config)
    if args.check_config:
        print(json.dumps({"configuration": asdict(config), "mcu_protocol": args.mcu_protocol,
                          "side_sensor_source": args.side_sensor_source,
                          "expected_side_range_m": config.expected_side_range_m,
                          "rear_clearance_m": config.rear_clearance_m}, indent=2))
        return 0
    recorder = FlightRecorder(args.log_dir, config.log_max_bytes, config.log_backup_count)
    recorder.emit("configuration", config=asdict(config), port=args.port, status_port=args.status_port,
                  mcu_protocol=args.mcu_protocol,
                  side_sensor_source=args.side_sensor_source,
                  command_status_fallback=not args.no_command_status_fallback,
                  argv=sys.argv, python=sys.version, expected_side_range_m=config.expected_side_range_m,
                  rear_clearance_m=config.rear_clearance_m)
    core = FullRunController(config, secrets.randbelow(0x7ffffffe) + 1, time.monotonic(), recorder.emit,
                             mcu_protocol=args.mcu_protocol)
    legacy = LegacyControlAdapter(core.session, config.maximum_rpm, recorder.emit) if args.mcu_protocol == "legacy" else None
    command_uart = status_uart = worker = node = receiver = None
    ros_initialized = False
    interrupted = False
    bundle_requested = False
    failed = False
    last_stop_request = None
    instant_stop_pending = False
    result = 0
    startup_stage = "before_imports"
    tx_attempt = None
    loop_metrics = {"cycles": 0, "max_heartbeat_gap_s": 0.0, "max_cycle_work_s": 0.0}

    def startup_step(stage, **details):
        nonlocal startup_stage
        startup_stage = stage
        recorder.emit("startup_step", stage=stage, **details)

    def runtime_health():
        now = time.monotonic()
        recorder.emit("runtime_health", state=core.state, phase=core.phase,
                      health_errors=core.health_errors(now), loop=dict(loop_metrics),
                      ROS=node.diagnostics_snapshot() if node and hasattr(node, "diagnostics_snapshot") else None,
                      ultrasonic=worker.snapshot() if worker and hasattr(worker, "snapshot") else None)

    def report_exception(context):
        details = {"context": context, "startup_stage": startup_stage, "traceback": traceback.format_exc(),
                   "last_TX_attempt": tx_attempt, "snapshot": core.snapshot(time.monotonic()),
                   "uart_rx": receiver.snapshot(time.monotonic()) if receiver else None}
        try:
            recorder.emit("exception", **details)
        except Exception:
            # A full/unwritable disk may also break the JSONL writer. Preserve
            # the original exception on stderr/systemd journal if available.
            print("[FLIGHT RECORDER FAILURE] " + json.dumps(details, default=str), file=sys.stderr, flush=True)

    def interrupt(signum, _frame):
        nonlocal interrupted
        interrupted = True
        recorder.emit("signal", number=signum)

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        startup_step("import_serial")
        import serial
        startup_step("import_ROS")
        import rclpy
        if args.side_sensor_source == "gpio":
            startup_step("import_Jetson_GPIO")
            import Jetson.GPIO as GPIO
        startup_step("open_command_UART", requested=args.port, resolved=os.path.realpath(args.port),
                     baudrate=args.baudrate)
        command_uart = serial.Serial(args.port, args.baudrate, timeout=0, write_timeout=0.15, exclusive=True)
        same_uart = os.path.realpath(args.status_port) == os.path.realpath(args.port)
        startup_step("open_status_UART", requested=args.status_port, resolved=os.path.realpath(args.status_port),
                     shared_handle=same_uart, baudrate=args.baudrate)
        status_uart = command_uart if same_uart else serial.Serial(
            args.status_port, args.baudrate, timeout=0, write_timeout=0.15, exclusive=True)
        receiver = StatusReceiver(command_uart, status_uart, args.port, args.status_port,
                                  args.mcu_protocol, core.session,
                                  command_fallback=not args.no_command_status_fallback, emit=recorder.emit)
        startup_step("initialize_ROS_context")
        rclpy.init()
        ros_initialized = True
        # Keep the controller's own stop path after ROS installs its handlers.
        signal.signal(signal.SIGINT, interrupt)
        signal.signal(signal.SIGTERM, interrupt)
        startup_step("create_ROS_subscriptions")
        node = make_node(config, core, recorder)
        if args.side_sensor_source == "gpio":
            startup_step("initialize_ultrasonic_GPIO")
            worker = UltrasonicWorker(GPIO, config, recorder.emit)
        else:
            startup_step("side_sensors_from_STM32_STATUS_no_GPIO")
            worker = McuUltrasonicFeed(config, recorder.emit)
        startup_step("adapter_initialized_waiting_for_live_inputs")
        last_handshake = -math.inf
        last_snapshot = -math.inf
        last_uart_health = -math.inf
        last_command_s = time.monotonic()
        recorder.emit("handshake", session=core.session,
                      mcu_protocol=args.mcu_protocol,
                      message=("Waiting for existing 7/11-field $STATUS. No firmware update or $FULL handshake."
                               if legacy else "Waiting for matching $CTRL v1 (optional extended firmware)."))
        # Imports and GPIO setup can legitimately take seconds before the
        # heartbeat begins. Only running-loop gaps count as scheduler faults.
        core.last_tick = time.monotonic()
        while rclpy.ok() and not interrupted:
            cycle_started = time.monotonic()
            if args.supervisor_pid is not None:
                try:
                    stat = Path(f"/proc/{args.supervisor_pid}/stat").read_text()
                    start_ticks = stat.rsplit(")", 1)[1].split()[19]
                except (OSError, IndexError):
                    start_ticks = None
                if start_ticks != args.supervisor_start_ticks:
                    recorder.emit("exception", reason="launcher_disappeared_or_PID_reused")
                    core.fault("launcher_disappeared_or_PID_reused")
                    result = 1
                    break
            rclpy.spin_once(node, timeout_sec=0)
            lines, active_overflow = receiver.poll(time.monotonic())
            if active_overflow:
                core.status_stamp = None
            for decoded in lines:
                if legacy and decoded.startswith("$STATUS"):
                    try:
                        received_at = time.monotonic()
                        core.update_status(legacy.decode(decoded, received_at,
                                                        allow_controls=core.state != "STARTUP"), received_at)
                        if args.side_sensor_source == "mcu" and not worker.update(legacy.raw, received_at):
                            core.fault("STM32_STATUS_missing_MCU_side_sensor_fields")
                    except ValueError as exc:
                        core.status_stamp = None
                        recorder.emit("STATUS_invalid", error=str(exc), line=decoded)
                        if isinstance(exc, LegacyStatusLayoutChanged):
                            core.fault("STM32_STATUS_layout_changed_within_run")
                elif not legacy and decoded.startswith("$CTRL"):
                    try:
                        core.update_status(ControlStatus.decode(decoded), time.monotonic())
                    except ValueError as exc:
                        core.status_stamp = None
                        recorder.emit("CTRL_invalid", error=str(exc), line=decoded)
                elif not legacy and decoded.startswith("$STATUS") and core.status is None:
                    recorder.emit("legacy_firmware", instruction="Set MCU_PROTOCOL=legacy to use existing firmware without flashing")
            mismatch = receiver.unsupported_layout()
            if core.state == "STARTUP" and mismatch is not None:
                recorder.emit("MCU_schema_mismatch", **mismatch,
                              instruction="Provide the actual STM32 $STATUS printf/snprintf field definition. "
                                          "RX exists; do not label this as disconnected wiring or fake missing fields.")
                core.fault(f"STM32_STATUS_unsupported_layout:port={mismatch['port']}:"
                           f"received_fields={mismatch['field_count']}:expected_fields=7_or_11")
            while True:
                try:
                    core.update_side(worker.results.get_nowait())
                except queue.Empty:
                    break
            now = time.monotonic()
            if not legacy and core.status is None and now - last_handshake >= 0.5:
                payload = f"$FULL,{core.session}\r\n".encode("ascii")
                tx_attempt = {"port": args.port, "text": payload.decode("ascii"), "monotonic_s": now}
                command_uart.write(payload)
                recorder.emit("uart_tx", kind="handshake", text=payload.decode("ascii"))
                last_handshake = now
            if args.user_stop_file and args.supervisor_pid is not None:
                try:
                    request = read_stop_request(args.user_stop_file, args.supervisor_pid,
                                                args.supervisor_start_ticks, last_stop_request)
                    if request:
                        last_stop_request = request["request_id"]
                        recorder.emit("user_instant_stop_request", request=request)
                        if core.state == "STARTUP":
                            recorder.emit("user_instant_stop_ignored", reason="startup is already stationary")
                        elif core.state != "FAULT_STOP":
                            if legacy:
                                legacy.request_instant_stop()
                            else:
                                instant_stop_pending = True
                            # Consume any UP already received in this cycle,
                            # even if we were still READY/already PAUSED.
                            core.change("PAUSED", "user_instant_stop_terminal")
                except (OSError, ValueError) as exc:
                    core.fault(f"user_stop_request_IO_or_format:{exc}")
                    recorder.emit("exception", context="user_stop_request", error=str(exc))
            command = core.tick(now)
            # Preflight can never issue a nonzero command, even if UP is pressed.
            if args.preflight_only:
                command = (0, 0)
            payload = f"$CMD,{command[0]},{command[1]},{int(instant_stop_pending)}\r\n".encode("ascii")
            tx_attempt = {"port": args.port, "text": payload.decode("ascii"), "monotonic_s": now}
            count = command_uart.write(payload)
            if count != len(payload):
                raise IOError(f"Short UART write: {count}/{len(payload)}")
            if legacy:
                legacy.note_command_written(command[0], command[1], int(instant_stop_pending), time.monotonic())
            instant_stop_pending = False
            heartbeat_gap = now - last_command_s
            recorder.emit("uart_tx", kind="command", text=payload.decode("ascii"),
                          heartbeat_gap_s=heartbeat_gap, state=core.state, phase=core.phase)
            last_command_s = now
            loop_metrics["cycles"] += 1
            loop_metrics["max_heartbeat_gap_s"] = max(loop_metrics["max_heartbeat_gap_s"],
                                                      heartbeat_gap)
            worker.inside = core.inside if core.phase in ("BASELINE", "PASS", "REAR_CLEARANCE") else None
            if now - last_snapshot >= 0.10:
                snapshot = core.snapshot(now)
                if legacy:
                    snapshot["legacy"] = legacy.snapshot()
                snapshot["uart_rx"] = receiver.snapshot(now)
                recorder.emit("snapshot", **snapshot)
                last_snapshot = now
            if now - last_uart_health >= 5 or (core.state == "FAULT_STOP" and not bundle_requested):
                runtime_health()
                recorder.emit("uart_health", state=core.state, **receiver.snapshot(now))
                last_uart_health = now
            if core.state == "FAULT_STOP" and not bundle_requested:
                # Zero RPM was already sent above. The launcher performs
                # compression in a separate process, keeping this heartbeat
                # loop responsive even with hundreds of MB of sensor logs.
                bundle_requested = True
                try:
                    if request_auto_bundle(args.log_dir, core.fault_reason):
                        recorder.emit("auto_bundle_requested", reason=core.fault_reason)
                except OSError as exc:
                    print(f"[BUNDLE FAIL] Cannot request diagnostics: {exc}", file=sys.stderr, flush=True)
            if args.preflight_only and core.preflight_ready:
                recorder.emit("ready", mcu_protocol=args.mcu_protocol,
                              message=f"Preflight passed: fresh camera/LiDAR/MCU status/{args.side_sensor_source}_side_sensors, zero motion only")
                result = 0
                break
            if args.preflight_only and core.state == "FAULT_STOP":
                result = 1
                break
            work_s = time.monotonic() - cycle_started
            loop_metrics["max_cycle_work_s"] = max(loop_metrics["max_cycle_work_s"], work_s)
            time.sleep(max(0, 0.02 - work_s))
        recorder.emit("loop_exit", interrupted=interrupted, result=result, ROS_context_ok=rclpy.ok(),
                      state=core.state, phase=core.phase, loop=loop_metrics)
    except Exception:
        failed = True
        report_exception("controller_main_loop_or_initialization")
        result = 1
    finally:
        # Stop the actuator first. GPIO teardown/ROS shutdown may take time.
        if command_uart is not None:
            try:
                for _ in range(3):
                    command_uart.write(b"$CMD,0,0,1\r\n")
                    time.sleep(0.02)
                recorder.emit("shutdown", reason="stop_and_brake_sent", snapshot=core.snapshot(time.monotonic()),
                              uart_rx=receiver.snapshot(time.monotonic()) if receiver else None)
            except Exception:
                failed = True
                report_exception("shutdown_stop")
        if receiver is not None:
            try:
                runtime_health()
            except Exception:
                failed = True
                report_exception("final_runtime_health")
        for name, cleanup in (("gpio", worker.close if worker else None),
                              ("ROS_node", node.destroy_node if node else None),
                              ("status_UART", status_uart.close if status_uart and status_uart is not command_uart else None),
                              ("command_UART", command_uart.close if command_uart else None)):
            if cleanup:
                try:
                    cleanup()
                except Exception:
                    failed = True
                    report_exception(name)
        if ros_initialized:
            try:
                rclpy.try_shutdown()
            except Exception:
                failed = True
                report_exception("ROS_shutdown")
        if failed:
            try:
                request_auto_bundle(args.log_dir, "controller_exception_or_cleanup_failure")
            except OSError as exc:
                print(f"[BUNDLE FAIL] Cannot request diagnostics: {exc}", file=sys.stderr, flush=True)
        try:
            recorder.close()
        except Exception:
            failed = True
            print("[FLIGHT RECORDER CLOSE FAILURE] " + traceback.format_exc(), file=sys.stderr, flush=True)
    return 1 if failed else result


if __name__ == "__main__":
    raise SystemExit(main())
